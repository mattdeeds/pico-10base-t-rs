# Full-duplex 10BASE-T — feasibility analysis

**One-liner:** the board is **full-duplex-capable hardware** running a
**half-duplex MAC by policy**. Going full-duplex is a firmware + link-negotiation
exercise, **not** a board respin. The headline win is killing the half-duplex
collision/RTO collapse on bidirectional TCP (gotcha #10), not a clean 2× of line
rate.

**Status:** analysis only — nothing implemented. Written 2026-06-03 after a
hardware-fact correction (see §2). Sits in the §4 backlog next to radio
modularization (`docs/perf-characterization-plan.md` §4-B).

---

## 1. Why this matters

Today the 10BASE-T (WAN) link runs **half-duplex**: TX carrier-senses the RX line
and defers, then does CSMA/CA random backoff (`eth_tx.rs` — `wait_carrier_idle`
:237, `csma_acquire` :208). That's the correct MAC behaviour for a link both ends
treat as half-duplex, but it's also the root of the dominant bidirectional-TCP
limiter: **gotcha #10** — the Pico's TX colliding with the host's ACKs, each
collision costing a TCP RTO (`docs/cpu-dpll-plan.md:665`; the 596 → 45 kB/s cliff
that Phase 3d/3e carrier-sense + CSMA/CA only partially patch).

Full-duplex removes collisions *by definition*. The question is what it would take.

---

## 2. Hardware verdict: full-duplex-CAPABLE (corrected)

**Prior misconception (now corrected).** An earlier read assumed "ISL3177E =
RS-485 transceiver = single shared differential pair = half-duplex, hears its own
TX." That is **wrong for this part.**

Per the Renesas/Intersil datasheet, the ISL317xE family — ISL3170E, ISL3171E,
ISL3173E, ISL3174E, ISL3176E, **ISL3177E** — is **configured for full duplex
(separate Rx-input and Tx-output pins)**: a separate driver pair and receiver
pair (four bus pins), *not* a shared A/B node. Mouser's line description confirms:
"RS-422/RS-485 … FL DUPLX." Combined with the **HR911105A** RJ45 magnetics
(standard separate TX and RX transformer windings), the analog front end supports
**simultaneous bidirectional** signalling.

**The collision problem is NOT evidence of a shared physical medium.** In
half-duplex 10BASE-T a "collision" is a *MAC-policy definition* — transmitting
while RX carrier is present — even on a link whose TX and RX pairs are physically
independent. Gotcha #10 happens because *both ends run half-duplex CSMA/CD*, not
because the wire forces it. The pairs never physically interfere; the half-duplex
peer simply declares a collision and jams/aborts → lost frame → RTO.

Corroborating evidence the pairs are separate: the carrier-detect SM watches RO
(GP13) to sense the **peer's** carrier (`eth_tx.rs:149`). If RO echoed the Pico's
own TX, that gate would read "busy" on every one of its own frames and be
nonsensical. It senses the peer ⇒ separate pairs.

> **One thing still worth confirming from the schematic** (not in-repo): that the
> board wires the driver pair and receiver pair to *separate* magnetics windings
> (near-certain given an FD transceiver + standard module). Only `DI`←GP14 and
> `RO`→GP13 reach the MCU (`README.md:30-31`); DE/RE are tied permanently enabled,
> so the driver always drives the TX pair and the receiver always listens on the
> RX pair — exactly the full-duplex wiring.

---

## 3. What's already in place (digital datapath)

The silicon side is ~80% ready — TX and RX are fully disjoint resources:

| Resource | TX | RX |
|---|---|---|
| PIO state machine | PIO0 **SM0** (`eth_tx.rs:119`) | PIO0 **SM1** (`eth_rx.rs`) |
| GPIO | GP14 (DI) | GP13 (RO) |
| Clock | 20 MHz Manchester | 60 MHz sampler |
| DMA | none | CH0/CH1 double-buffer |
| CPU | **core 0** (`send_raw_frame`) | **core 1** (`DMA_IRQ_0` decode) |

`main.rs:303` notes "SM3 + PIO1 are free." The chip can clock out a frame on
SM0/GP14 while SM1/GP13 samples an inbound frame into DMA and core 1 decodes it —
concurrently, on a different core. Nothing in the datapath serializes the two
directions.

**Cross-core concurrency is already safe.** TX wraps its FIFO writes in
`critical_section::with` (`eth_tx.rs:310`), but the rp-hal critical-section impl
masks only the *local* core's interrupts + holds `Spinlock<31>`. Core 1's RX
decode IRQ keeps running during a TX frame, and RX shared state uses `Spinlock<0>`
(`eth_mac.rs:142`) — a different lock, no contention. TX-on-core-0 and
RX-decode-on-core-1 do not block each other today.

---

## 4. What full-duplex actually requires

| Requirement | Status | Effort |
|---|---|---|
| Analog: separate TX/RX pairs | ✅ already capable (FD transceiver + separate-winding magnetics) | none |
| Disjoint PIO SMs / DMA / cores | ✅ already (SM0 TX, SM1 RX, core 0/1) | none |
| Cross-core TX↔RX-decode concurrency safe | ✅ already (CS masks core 0 only; RX uses `Spinlock<0>`) | none |
| Drop CSMA/CD MAC policy | gate carrier-sense + `csma_acquire` behind a `full-duplex` mode | easy |
| **Both ends agree full-duplex** | ❌ we emit only NLPs → peer defaults to HALF | **the real work** |
| Sustain bidirectional rate | bounded by core-1 RX decode (existing limit), not by FD | see §5 |

### 4.1 The real blocker: duplex agreement

We send **Normal Link Pulses** every 16 ms (`main.rs:566`) and nothing else. NLPs
carry no duplex info, so a switch/NIC parallel-detects us as **10BASE-T half**.
Run FD on our side without the peer agreeing and you get a **duplex mismatch**
(one end late-collisions, the other FCS-errors → *worse* than half-duplex). Two
paths:

1. **Forced config (pragmatic — do this first to prove it).** Force both ends to
   10M-FD: on a Linux peer `ethtool -s <if> speed 10 duplex full autoneg off`, or a
   managed switch port set to "10 full." Zero negotiation code; lets us measure FD
   immediately behind the firmware mode flag.
2. **FLP auto-negotiation (proper, interoperable).** Replace NLPs with Fast Link
   Pulse bursts advertising 10BASE-T-FD and parse the partner's 16-bit link code
   word. This is the only genuinely new subsystem: generate/decode the FLP
   clock+data pulse train (PIO or software), implement the negotiation state
   machine. Moderate effort; required to interoperate with an unconfigured switch.

`docs/r15-plan.md:318` already flags that real switches usually won't negotiate
with us at all today — auto-neg (path 2) is what fixes that more generally.

### 4.2 Firmware MAC mode (easy)

Gate the half-duplex policy behind a `full-duplex` feature / runtime mode that
no-ops:
- `wait_carrier_idle()` (`eth_tx.rs:237`) — transmit regardless of RX carrier.
- `csma_acquire()` (`eth_tx.rs:208`) — no backoff.
- the carrier-detect SM2 (`eth_tx.rs:149`) — no longer needed to gate TX; frees a
  state machine (keep only if you still want RX-activity stats).

**Keep** the IFG padding + TP_IDL (`eth_tx.rs:331-345`): 802.3 still mandates
≥ 9.6 µs inter-frame gap between *your own* frames even in full-duplex.

Make it a *mode*, not a deletion — the half-duplex MAC must stay for hubs / shared
segments / un-negotiated links.

---

## 5. The real ceiling: core-1 RX decode (not FD itself)

Full-duplex does **not** add RX decode load — RX cost is set by the inbound rate
regardless of duplex. TX is on a different core and cheap (FIFO writes + a
precomputed CRC). So FD lets each direction run at its *own* per-direction limit,
simultaneously:

- **TX:** near line rate (10 Mbps), as today.
- **RX:** bounded by core-1 decode reliability at full MTU — the standing limiter
  (the PIO-DPLL retro capped ~40% full-MTU; the CPU-DPLL on core 1 is the current
  approach; ≤ 2.57 ms worst-case decode IRQ; ambient decode already ~42% busy).

So the aggregate is **additive over today's "one direction at a time,"** but it is
*not* a clean 2× of line rate — the RX decode ceiling that already exists is
inherited unchanged. FD widens the pipe to "TX-at-line-rate **and**
RX-at-decode-limit at once," which is a real gain for the router's bidirectional
workload (download = RX data + TX ACKs; upload = TX data + RX ACKs).

Clock drift is **orthogonal** to duplex mode — it persists on a full-duplex link
too (`docs/clock-recovery-decoder-plan.md` §1). FD is not a workaround for the
decoder's clock-recovery requirement.

---

## 6. What full-duplex actually buys

1. **Eliminates the half-duplex collision/RTO collapse** (gotcha #10) — the
   documented dominant bidirectional-TCP limiter. No collisions in FD by
   definition, so the 596 → 45 kB/s cliff and the RTO stalls disappear. This is
   the headline win, larger in practice than raw bandwidth.
2. **Concurrent bidirectional throughput** at per-direction limits (§5) instead of
   the half-duplex alternation + CSMA/CA backoff overhead.
3. **Lower TX latency** — no carrier-sense deferral / backoff before each frame.

Not bought: a 2× line-rate ceiling (capped by RX decode), nor any fix to clock
drift or full-MTU decode reliability.

---

## 7. Implementation sketch (if pursued)

1. **Prove it cheaply.** Add `full-duplex` feature flag → no-op the carrier-sense
   + CSMA path (§4.2). Force the peer to 10M-FD via `ethtool` (§4.1 path 1). Run a
   bidirectional TCP test; confirm FCS-OK stays clean and the gotcha-#10 collisions
   in `/proc/net/dev` go to zero. **This validates the whole premise before any
   negotiation work.**
2. **Measure the real ceiling.** With collisions gone, characterize concurrent
   TX+RX throughput and core-1 decode CPU under sustained bidirectional load —
   confirm RX decode (not FD) is the limiter.
3. **If worth productizing:** implement FLP auto-negotiation (§4.1 path 2) so it
   works against an unconfigured switch without manual forcing.

---

## 8. Open items / to verify

- **Schematic** — confirm the driver pair and receiver pair go to separate
  magnetics windings (§2; near-certain, not in-repo).
- **NEXT during simultaneous TX** — verify the receiver isn't desensitized by
  near-end crosstalk while driving (real magnetics handle it; confirm FCS-OK under
  a real bidirectional test in step 1 above).

---

## 9. Where it sits on the value/effort curve

**Re-ranked upward** after the hardware correction: this is firmware + negotiation,
not a board respin. The cheap proof (step 1) is a day of work and directly attacks
the documented bidirectional-TCP limiter. Versus radio modularization (§4-B, now
*not* justified — the cyw43 LAN ceiling was the gSPI transport clock, since fixed)
and the other §4-G levers, a forced-FD proof-of-concept is high-information for low
cost. Full FLP auto-negotiation is the larger, optional follow-on.
