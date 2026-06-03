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

## 7. Forced-FD experiment spec

Prove §2 (half-duplex is policy, not a hardware wall) and quantify the gain by
forcing both ends to 10M-FD and flipping a firmware FD mode that disables
carrier-sense/CSMA. Grounded in the default-NIC build (static `192.168.37.24/24`;
`http-bulk-test` already provides the download source; `CORE1_BUSY` already
accumulates at `eth_mac.rs:342`; RX `fcs_fail` is the device-side collision proxy).
Two tiers so the core hypothesis costs near-zero new code.

### 7.0 Hypotheses

| # | Hypothesis | Pass signal |
|---|---|---|
| **H1** | Collisions are MAC-policy, not physical | FD build → host `colls` ≈ 0, device `fcs_fail` drops to no-collision floor |
| **H2** | FD removes turnaround/backoff on a single flow | FD download kB/s ≥ HD, no RTO variance |
| **H3** | FD enables concurrent bidirectional throughput | FD (down+up at once) aggregate ≫ HD aggregate |
| **H4** | The real ceiling is core-1 decode, not FD | `cpu1` saturates / `fcs_fail` climbs under sustained RX before line rate |

### 7.1 Topology & prerequisites

```
[Linux host NIC] ──RJ45 (10BASE-T)── [HR911105A] ── ISL3177E ── Pico 2 (GP13/14)
   192.168.37.x                                                  192.168.37.24
```

- **Use the NIC that the R4–R8 recipes already link at 10BASE-T.** Verify it
  advertises full: `ethtool <if> | grep -A2 "Supported link modes"` lists
  `10baseT/Full`.
- **Top risk / hard prerequisite:** many modern NICs won't go to 10M at all.
  Confirm a working 10M-FD-capable port (known-good NIC, USB-100M dongle, or a
  managed switch port set "10 full") *before writing firmware.*
- Forcing autoneg **off** can disable auto-MDIX — if link won't come up, try a
  crossover cable or `ethtool -s <if> mdix on`.

### 7.2 Firmware — Tier 1 (the MAC-mode gate; only variable under test)

`Cargo.toml [features]`:
```toml
full-duplex = []   # disables carrier-sense + CSMA/CA; ONLY safe vs a forced-10-FD
                   # peer (mismatch = worse than HD); default off → NIC binary byte-identical
```
Three gates in `src/eth_tx.rs` (transmit unconditionally in FD mode):
- `csma_acquire()` in `send_raw_frame` → `#[cfg(not(feature = "full-duplex"))]`
- `wait_carrier_idle()` in `send_udp_broadcast` → same gate
- `wait_carrier_idle()` in `send_nlp` → same gate

**Keep** NLPs (link integrity for the forced-10 link), IFG/TP_IDL padding, and the
carrier-detect SM2 (harmless; leave it to minimize diff). Total Tier-1 delta ≈ 3
one-line `cfg` gates.

### 7.3 Firmware — Tier 2 (concurrent-bidirectional measurement, gated)

Behind `full-duplex` (or an `fd-bench` sub-feature) so production stays untouched:
- **Upload sink:** TCP listener on **port 9999**, read-drains + counts into a
  `FD_SINK_RX` static, re-listens per connection — copy the wifi `serve_lan_sink`
  pattern. Needs +1 `SocketStorage` (5→6) + a `[0u8; 32*1024]` rx buffer.
- **Download source:** reuse existing `http-bulk-test` (1 MB on :80, 32 KB TX win).
- **Telemetry on the NIC heartbeat:** add `cpu1=` via
  `cycles::permille_over(CORE1_BUSY delta, elapsed_us)` + a `last_emit_us` (the
  measured-window normalization from the [Lan] fix — do **not** assume a 1 s
  window), plus `tx/rx KB/s` from the byte counters and `fcs_ok/fcs_fail`.

### 7.4 Build matrix (duplex MUST match the build)

| Run | Firmware | Host duplex | Tests |
|---|---|---|---|
| **A — HD control** | `--features http-bulk-test` | `autoneg on` (or forced 10-HD) | baseline |
| **B — FD treatment** | `--features "http-bulk-test full-duplex"` | forced 10-FD | H1, H2 |
| **C — FD bidir** (Tier 2) | B + sink | forced 10-FD | H3, H4 |

A duplex mismatch (FD firmware ↔ HD host) is garbage, not a result. **Flash
gotcha:** build the variant you flash *last* and verify the binary before each run.

### 7.5 Host setup

```bash
IF=enpXsY
sudo ip addr flush dev $IF; sudo ip addr add 192.168.37.1/24 dev $IF; sudo ip link set $IF up
# Run A (HD):  sudo ethtool -s $IF autoneg on
# Run B/C (FD): sudo ethtool -s $IF speed 10 duplex full autoneg off
ethtool $IF | grep -E "Speed|Duplex|Link"   # expect 10Mb/s, Full, Link detected: yes
```

### 7.6 Measurement

Proven `/proc/net/dev` method (gotcha-#10 work logged ~30 colls/curl in HD):
```bash
read_ctr(){ grep "$IF:" /proc/net/dev; ethtool -S $IF 2>/dev/null | grep -iE "colli|crc|abort|carrier"; }
# H2 single flow (data down + ACKs up):
read_ctr; curl -s -o /dev/null -w "down=%{speed_download}\n" http://192.168.37.24/; read_ctr
# H3 concurrent bidir (Tier 2):
curl -s -o /dev/null -w "down=%{speed_download}\n" http://192.168.37.24/ &
dd if=/dev/zero bs=64k count=512 2>/dev/null | pv -b | nc -q1 192.168.37.24 9999; wait
```
Device side (CDC): `fcs_ok`/`fcs_fail` deltas (H1, H4), `cpu1=` under load (H4).
Always cross-check device `tx/rx KB/s` against the host number (the [Lan]-fix lesson).

### 7.7 Acceptance / decision gate

- **H1 confirmed** (host colls→0, device fcs_fail→floor) ⇒ §2 proven on-wire; the
  headline result on its own.
- **H3 large** ⇒ **justifies building FLP auto-negotiation** (§4.1 path 2).
- **H3 marginal** (decode-capped per H4, or unidirectional workload) ⇒ FD stays a
  documented option; **do not** build auto-neg. Record numbers and stop.

### 7.8 Risks & gotchas

1. Host NIC can't do 10M-FD — the gating risk; verify first (§7.1).
2. Link won't come up with autoneg off — MDIX/crossover fallback.
3. Duplex mismatch — strictly pair build↔host; mismatch is garbage.
4. Production byte-drift — `full-duplex` off by default; confirm the default NIC
   binary is unchanged.
5. `fcs_fail` has two sources — collisions *and* the clock-drift full-MTU tail.
   Separate them: idle fcs_fail (drift only) vs under-contention (drift +
   collisions); FD should null only the contention delta.

**Effort:** Tier 1 ≈ ½ day (3 gates + host recipe; tests H1/H2 with no new
measurement code). Tier 2 ≈ +½ day (sink + cpu1 telemetry; tests H3/H4).

### 7.9 Results — RAN IT (2026-06-03)

Ran Tier 1 + Tier 2 on-device: `enp1s0f0` (supports `10baseT/Full`) ↔ Pico 10BT,
host duplex forced via `ethtool` to match the firmware mode each run. Builds:
`http-bulk-test [+ fd-bench] [+ full-duplex]`, SWD-flashed; download = host `curl`
of the 1 MB `/bulk`, upload = host `dd | nc … :9999` into the sink, collisions =
`/proc/net/dev` deltas, device RX from the `[Rx]`/`[Sink]` CDC lines.

**Single flow (download, ~unidirectional), per 10 MB:**

| | HD (host Half) | FD (host Full) |
|---|---|---|
| download avg (range) | 619 (341–998) | 567 (335–981) |
| host TX collisions | 5 | **0** |
| device RX `fcs_fail` | ~5–6 /s | ~2–10 /s (**unchanged**) |

**Concurrent bidirectional (download + bulk upload at once):**

| | HD (host Half) | FD (host Full) |
|---|---|---|
| download avg (range) | **434** (147–917) | **736** (204–1004) |
| host TX collisions | 8 | **0** |
| upload solo ceiling | — | ~102 KB/s (steady, `dec≈140/s`) |

**Verdict:**
- **H1 ✅ confirmed on-wire** — forcing both ends to 10-Full drove host collisions
  to **0** and carried full transfers. The board IS full-duplex-capable; HD was a
  MAC policy. §2 validated empirically.
- **H2 ❌ no single-flow gain** — 619→567 (noise). With CSMA/CA already in the HD
  build, collisions were already rare (~0.5/MB), so removing them moves nothing on
  a ~unidirectional flow.
- **H3 ✅ qualified — FD helps the *contended* case** — under concurrent up+down,
  FD download **736 vs 434 (+70 %)** with **0 vs 8** collisions: in HD the heavy
  upload collides with the download and craters it; FD avoids that (this is §6's
  predicted win, invisible to the single-flow H2). **But bounded:** the upload
  (device RX of bulk) is **RX-decode-capped at ~102 KB/s regardless of duplex** and
  stalls under concurrency in *both* modes, so FD protects the download from
  collision-collapse rather than unlocking a 2× aggregate.
- **H4 ✅✅ strongly — RX decode is THE ceiling.** Device `fcs_fail` is identical
  HD↔FD (~5/s → it's clock-drift, not collisions), and **device RX-of-bulk tops out
  ~102 KB/s — ~9× below device TX (~970)**. That TX/RX asymmetry is a notable
  standalone characterization finding (the upload path, never measured before).
  **Now characterized in `docs/rx-bulk-ceiling.md`:** cause is full-MTU FCS-fail
  (clock drift) — clean ≤512 B, cliff to ~72 % at 1472 B; not inbox/DMA/window.

**Decision (§7.7 gate): do NOT build FLP auto-negotiation now.** FD's throughput
value is real but confined to the contended / multi-client case, requires FLP
auto-neg (a real new subsystem) to be practical, and is bounded by the RX-decode
ceiling. The higher-leverage move is **fixing RX decode** (the existing CPU-DPLL
track) — it lifts the ~102 KB/s upload ceiling *and* helps every case. Keep FD
documented as a contended-case lever; the experiment harness (`full-duplex` +
`fd-bench` features) stays in-tree, off by default, for re-running later.

---

## 8. Open items / to verify

- **Schematic** — confirm the driver pair and receiver pair go to separate
  magnetics windings (§2; near-certain, not in-repo).
- **NEXT during simultaneous TX** — verify the receiver isn't desensitized by
  near-end crosstalk while driving (real magnetics handle it; confirm FCS-OK under
  a real bidirectional test in step 1 above).

---

## 9. Where it sits on the value/effort curve

**Settled by the §7.9 experiment.** The forced-FD proof-of-concept was the
high-information / low-cost move, and it ran: FD is real (H1) and helps the
contended bidirectional case (H3, +70 % download under concurrent load, zero
collisions) — but it gives nothing on single flows (H2) and is bounded by the
RX-decode ceiling (H4: device RX-of-bulk ~102 KB/s, ~9× below TX).

**Net: FD is a *secondary* lever, not the next move.** Pursuing it for throughput
would mean building FLP auto-negotiation (a real subsystem) for a win that only
materialises under concurrent/multi-client load and is still capped by RX decode.
The **primary lever is RX decode** (the existing CPU-DPLL track): it lifts the
~102 KB/s upload ceiling *and* improves every case, FD or not. Radio
modularization (§4-B) remains *not* justified (the cyw43 LAN ceiling was the gSPI
clock, since fixed). Keep the `full-duplex` + `fd-bench` experiment harness in-tree
(off by default) so FD can be revisited cheaply once RX decode is faster — at which
point FD's contended-case win would actually have headroom to matter.
