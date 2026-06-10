# RX-of-bulk decode ceiling — characterization

**One-liner:** the device receives bulk TCP at only **~100 KB/s** (vs ~970 KB/s for
TX) because at full-MTU **~30 % of inbound frames fail FCS from clock drift**, which
collapses the host's TCP congestion window to 1–2 segments (`ss`: cwnd 10→1–2,
ssthresh→2, thousands of retransmits, RTT a healthy 3 ms). It is **loss-limited**,
**not** loop/`max_burst` (the main loop runs ~107 K iters/s), RTT, inbox, or DMA.
A **secondary receive-window ceiling** (~1–2 segments in flight) surfaces once loss
is removed. **The intuitive MSS-clamp fix was tested and REFUTED** (§5) — small
frames decode clean but hit the window ceiling at *lower* throughput (34 KB/s).

**Status:** characterization + MSS-clamp + frame-rate-ceiling + decode-fix
experiments done (2026-06-03). Follow-on from the full-duplex experiment
(`docs/full-duplex-analysis.md` §7.9 / H4), which surfaced the ~102 KB/s figure.
**The decode loss is PHY-limited (§8) — the firmware decoder is near its floor;
the durable fix is a hardware PHY**, not more decoder work
(`docs/cpu-dpll-plan.md` §9d).

> **RE-OPENED (2026-06-10): see §9.** The ~100 KB/s ceiling is reproduced almost
> exactly, at every tested MTU, by a previously-missed smoltcp behaviour:
> `max_burst_size = Some(1)` clamps the advertised TCP receive window to ONE
> segment, serializing bulk uploads to one segment per (10 ms delayed-ACK + RTT)
> cycle. The FCS loss is real but was not the binding constraint. Fixed in
> `eth_mac.rs` (`max_burst_size = Some(INBOX_SLOTS)`); needs on-hardware
> re-measurement.

> **DECISION (2026-06-03): ACCEPTED — track closed.** RX-of-bulk stays ~100 KB/s;
> this is a PHY limit, not a firmware bug worth more decoder effort. The device is
> solid for low-rate / small-frame traffic; bulk RX is the documented ceiling. The
> durable fix (a real Ethernet PHY) is **deferred to a future board revision**, not
> scheduled. Still open (firmware, separate): the sustained-full-MTU **hang** (§6) →
> add the RP2350 watchdog. The optional load-dependence re-check (§7) was not run —
> we accepted the PHY-limited verdict on the existing §9d + §8 evidence.

---

## 1. Why this matters

The FD experiment found device **TX of bulk ≈ 970 KB/s** but **RX of bulk
≈ 102 KB/s** — a ~9× asymmetry, and the binding constraint on bidirectional
throughput (the upload direction, never benchmarked before). TX is cheap
(Manchester-encode + PIO out); RX is the expensive path (60 MHz PIO sample → DMA →
core-1 DPLL decode + FCS). This characterizes *what* caps RX.

## 2. Method

- Build: `--features "http-bulk-test fd-bench diag"` (NIC build; `diag` lights up
  the `[Mac]` line with `inbox_drop`/`inbox_hwm`/`carry_cap`; `fd-bench` adds the
  port-9999 TCP sink + `[Sink]` rate). No new firmware — existing counters.
- Host = the 10BT NIC `enp1s0f0` (10M, duplex matched to firmware), device static
  `192.168.37.24`. Bulk upload = `dd | nc … :9999`. Pure-decode size sweep =
  `tools/rx-decode-sweep.py` UDP to a **closed port (1239)** → frames are decoded +
  FCS-counted on core 1 *before* smoltcp drops them (no socket, no ICMP in this
  build) → isolates RX decode from TCP/ACK/echo entirely.
- Read `[Rx] dec/ok/fail`, `[Mac] inbox_drop/hwm/carry_cap`, `[Sink] rx KB/s` over
  CDC.

## 3. Findings

### 3.1 It is decode FCS-failure, not handoff or DMA

Sustained bulk upload (device RX, ~100 KB/s):
`[Rx] dec≈190/s ok≈137/s fail≈53/s` (**~28 % FCS fail on full-MTU TCP**), with
`[Mac] inbox_drop=0  inbox_hwm=1  carry_cap=0`.

| Hypothesis | Verdict |
|---|---|
| **D** core-0 can't drain the inbox | ❌ ruled out — `inbox_drop=0`, `inbox_hwm=1` |
| **E** decode-per-DMA-half > 2.18 ms half-fill | ❌ ruled out — `carry_cap=0` (wire ~idle at 100 KB/s) |
| **C** TCP window / ACK cadence | ❌ not the cause — pure-UDP decode (no TCP) fails the same |
| **B** FCS-fail / clock-drift at full-MTU | ✅ **confirmed** (see 3.2) |

### 3.2 The size cliff (pure RX decode, random payload)

| frame size | 64 | 512 | 700 | 850 | 1000 | 1150 | 1300 | 1472 |
|---|---|---|---|---|---|---|---|---|
| **RX FCS-fail %** | ~4 | ~3 | 29 | 49 | 41 | 16\* | 42 | **72** |

\* clock-offset (δ) noise — see 3.3. **Knee ≈ 600–700 B.** Clean below it,
catastrophic above. This is the textbook clock-drift signature: bit errors
accumulate with frame length, so P(≥1 bit error → FCS fail) climbs with size — the
same mechanism `docs/clock-recovery-decoder-plan.md` §1 models (50 % errors at
~byte 1050 for δ≈60 ppm; this device's knee sits a bit earlier → higher current δ).

### 3.3 Caveats

- **Payload content matters.** An all-`0x55` payload (preamble-like) inflated the
  mid-size points (64→7.7 %, 512→35 %) vs **random** payload (64→~4 %, 512→~3 %).
  The random/representative curve above is the one to trust; `0x55` is an artifact.
- **δ-variance noise.** Each ~4 s window samples a different instantaneous oscillator
  offset (temperature/warm-up), so near-threshold fail% swings run-to-run
  (e.g. 1150 B read 16 % between 1000 B@41 % and 1300 B@42 %). Trend is robust; point
  values are noisy.
- **Rate matters too.** Sustained UDP full-MTU at ~400 pps read ~72 % vs ~28 % under
  real TCP bulk — tighter inter-frame spacing likely starves the decoder's per-frame
  re-acquire. Both confirm "full-MTU fails badly"; the exact % is size × rate × δ.

## 4. Why this caps RX-of-bulk at ~100 KB/s — `ss`-grounded

Diagnosed with the host TCP state (`ss -tino` of the upload connection) + an
on-device main-loop counter (`[Sink] loop=/s`). Two facts kill the candidate
explanations and pin the real one:

- **Not the loop / `max_burst_size`.** The main loop runs **~140 K iters/s idle,
  ~107 K/s under upload** — with `max_burst_size = Some(1)` that's ~107 K frames/s
  of capacity, ~700× the actual ~150 frames/s. The earlier "~150 frames/s is a
  per-poll rate cap" hypothesis is **refuted**.
- **Not RTT.** `ss` RTT is **2–5 ms** throughout (healthy LAN).

**Full-MTU (the real case) is LOSS-limited.** During a full-MTU upload, `ss` shows
the host's **cwnd collapse 10 → 1–2, ssthresh 64076 → 2, with thousands of
retransmits** — the textbook signature of a high-loss path. The 32 % FCS decode
failures (§3) make TCP treat the link as congested; cwnd pins at 1–2 segments →
~100 KB/s. **Decode reliability is the binding constraint for full-MTU RX-of-bulk.**

**A second, lower ceiling lurks underneath: receive-window / in-flight.** With the
MSS clamp (small frames, 0 % loss) `ss` shows **cwnd 10 (idle), 0 retrans, but only
unacked 1–2** — the host has cwnd headroom yet keeps ~1–2 segments outstanding, i.e.
the device's advertised window (or app pacing) caps in-flight depth. That's why the
clamp's clean-decode run still only reached 34 KB/s.

## 5. Mitigations — MSS clamp TESTED and REFUTED

**TCP MSS clamp (the intuitive "cheap fix") does NOT work — it makes throughput
worse.** `mss-clamp` feature lowers `eth_mac::MTU` → smaller advertised MSS → peer
sends sub-knee frames; bulk upload into the :9999 sink:

| device MTU | on-wire frame | RX FCS-fail | upload goodput | `ss` limit |
|---|---|---|---|---|
| 500  | ~526 B | ~0 % | **34 KB/s** | rwnd/in-flight (cwnd idle) |
| 1000 | ~1026 B | ~1–13 % | **68 KB/s** | mixed |
| 1500 | ~1526 B | ~32 % | **99 KB/s** | loss (cwnd collapse) |

Clamping removes decode loss but trades it for the receive-window ceiling at *lower*
absolute throughput (smaller frames). **Conclusion: do not clamp MSS.**

Real levers, in priority order:
1. **Fix full-MTU decode — but it's PHY-limited (firmware near-exhausted; see §8).**
   Eliminating the FCS loss would stop the cwnd collapse, but `cpu-dpll-plan.md` §9d
   already showed the residual is **analog PHY noise** (flat per-byte error profile,
   ~5.8e-5/bit), and the §8 offline experiment confirms a noise-robust (matched-
   filter) bit decision gives no net gain. **The durable fix is hardware** (a real
   Ethernet PHY / better analog front-end), not the firmware decoder.
2. **Then raise the receive-window / in-flight depth.** Once loss is gone, the
   ~1–2-segment in-flight cap (rwnd or app pacing) becomes the limit — investigate
   smoltcp's advertised window vs the 32 KB sink buffer (and whether window scaling
   is off). Only worth chasing after lever 1.
3. **Not the loop** — `max_burst_size`/main-loop is not a bottleneck (107 K/s); do
   not spend effort there.

## 6. Robustness bug found (separate)

A **sustained full-MTU inbound stream hung the device** — first seen under a
max-rate UDP flood, then **again during a full-MTU TCP bulk upload** (the MTU-1500
baseline run): link dropped (no NLPs → host `Link detected: no`), CDC went silent,
but USB stayed enumerated and SWD still worked; a reflash/reset recovered it
cleanly. Rate-limiting (~400 pps UDP) and small frames (the clamp runs) avoided it,
so it correlates with **sustained full-MTU inbound volume**, not a specific size.
No `inbox_drop`/`carry_cap` preceded it → the hang is elsewhere (decode/IRQ
livelock or a panic). DoS-shaped, but full-MTU bulk RX is normal traffic, so this
matters.

**RECOVERY DONE (2026-06-03): RP2350 hardware watchdog added** (`main.rs`
`WDT_TIMEOUT_US` = 6 s; fed from the core-0 poll loop on the NIC build and a
dedicated `watchdog_feed_task` on the router/wireless executor). The device now
**self-reboots + recovers** if the loop/executor wedges, instead of needing a
manual SWD reflash. Validated on-device: (a) **no false-trip** — `t`/`hb` climb
continuously past the timeout under idle + heavy flood/upload stress (NIC and
router builds); (b) **fires + recovers** — a deliberate-stall test build (stop
feeding after 15 s) rebooted ~6 s later (USB re-enumerated, `t` reset to 1). The
intermittent hang itself did **not** reproduce across three flood/upload attempts
this session, so it wasn't observed being recovered directly — but the watchdog is
armed and the firing path is proven. **Still OPEN: root-cause the hang** (it's a
recovery, not a fix) + a reliable repro (backlog §4-F).

## 7. Next steps

- **Decode is PHY-limited (§8) — the durable fix is HARDWARE** (a real Ethernet PHY
  / better analog front-end). The firmware edge-track decoder is near its floor.
- **One rigorous check before fully closing the firmware door:** the on-device fail
  rate varies (≈50 % at light load §9d vs 28–72 % this session) — if part is
  *load-dependent* (not pure PHY) it'd be firmware-addressable. Confirm by re-running
  the §9d per-byte-error dump **under sustained bulk load** (instrumentation
  recoverable from commits `ab72c89..f0253c8`); a flat profile = pure PHY, a
  ramp/cliff = a firmware-fixable load component.
- **Receive-window / in-flight depth** (§4) — secondary; only matters once loss is
  gone (i.e. after a PHY fix). Cheap to check smoltcp's window vs the 32 KB buffer.
- **The sustained-full-MTU hang (§6): RP2350 watchdog DONE** (recovery validated).
  Still open: a reliable repro + root-cause (the watchdog recovers it, doesn't fix it).
- **Ruled out, don't pursue:** `max_burst_size`/main-loop (107 K iters/s), RTT
  (3 ms), MSS clamp (§5), and a naive matched-filter decision (§8).

## 8. Decode-fix investigation — PHY-limited (firmware near-exhausted)

The full-MTU FCS loss that drives the §4 cwnd collapse is **analog PHY noise**, not
a decoder bug:

- **Prior (`cpu-dpll-plan.md` §9d):** the edge-track DPLL is offline-validated
  perfect (FCS N/N on the corpus) and fits the IRQ budget. On-device it gets ~50 %
  full-MTU; a failed-frame **per-byte error dump was FLAT** (~0.1–1.1 %, ~5.8e-5/bit),
  matching iid noise statistics — verdict *"as good as it can get against this PHY."*
- **This session (offline `tools/clock-recovery/noise_compare.py`):** tested the one
  untried firmware lever — a **matched-filter (integrate-both-half-bits) bit
  decision** vs the current single-sample (`tr-1`) — by injecting per-sample noise
  into the corpus. At the operating point it gives **no net gain** (p=3e-4: edge 33 %
  vs MF 31 %) and is *worse* on clean for some frames (66 % vs 100 %) because half-bit
  integration needs precise half-bit phase that varies frame-to-frame, while `tr-1`
  sits robustly at the half-bit centre. (iid noise is an upper bound — real
  correlated/baseline-wander noise helps the MF even less.)

**Conclusion:** firmware decode is near its floor. The remaining firmware avenue (a
full NCO-phase-tracked matched filter) is complex and §9d predicts marginal returns.
The high-value lever for full-MTU RX is a **hardware PHY** — ties to the
`docs/full-duplex-analysis.md` "real PHY" option and any board respin.

## 9. RE-OPENED (2026-06-10) — the ceiling is the advertised-window clamp, not loss

The §4/§5 "secondary receive-window ceiling" was never root-caused ("investigate
smoltcp's advertised window" was lever 2, deferred). Root cause found by reading
smoltcp 0.13 source:

- **`smoltcp` clamps the advertised TCP window to `max_burst_size × MSS`**
  (`iface/packet.rs`, TCP dispatch: `window_len = min(window_len,
  max_burst_size * max_segment_size)`). `EthMac::capabilities()` set
  `caps.max_burst_size = Some(1)` — so the device advertised a **one-segment
  receive window on every ACK**, regardless of the 32 KB sink buffer. This is
  exactly the `ss` signature in §4: host has cwnd headroom (cwnd 10) but keeps
  only `unacked 1–2` outstanding.
- **With a 1-MSS window the 10 ms delayed ACK gates every segment.** smoltcp's
  default `ack_delay` is 10 ms, and its immediate-ACK rule
  (`immediate_ack_to_transmit`) only fires once *more than* 1 MSS of unACKed
  data is buffered — impossible when the window admits exactly one segment. So
  steady state is: host sends 1 segment → device sits the full 10 ms delayed-ACK
  timer → ACK (+~3 ms RTT) → next segment.

**This model reproduces the §5 measurements at every MTU with no free
parameters** (payload-per-segment ÷ ~13.5 ms):

| device MTU | payload/segment | predicted | measured (§5) |
|---|---|---|---|
| 1500 | 1448 B | ~107 KB/s | ~99 KB/s |
| 1000 | 960 B  | ~71 KB/s  | 68 KB/s |
| 500  | 460 B  | ~34 KB/s  | 34 KB/s |

The clamp also explains why the loss looked so expensive: with only 1 segment in
flight, **fast retransmit is impossible** (no dup-ACKs can ever be generated), so
every FCS-failed frame costs a host TLP/RTO stall rather than a ~RTT recovery.

**Corrections to earlier verdicts:**
- §4 "full-MTU is LOSS-limited" — overstated. The 28 % FCS loss is real and does
  collapse cwnd, but cwnd 1–2 vs rwnd 1 bind at the same point; the *ceiling
  shape* (and the exact ~100 KB/s figure) is the window clamp + delayed ACK.
- §5 "MSS clamp TESTED and REFUTED" — the experiment was run *under* the window
  clamp, so it measured `MSS/13.5 ms` scaling, not the clamp's real effect.
  Worth re-running after the fix: sub-knee frames (~600–1000 B on-wire) at ~0–5 %
  decode loss with a real window could plausibly reach several hundred KB/s.

**FIX (this commit):** `caps.max_burst_size = Some(INBOX_SLOTS)` (= 4). Window
becomes min(socket buffer, 4 × MSS ≈ 5.8 KB), which covers the 10 Mbit
half-duplex BDP (~3.75 KB @ 3 ms RTT) and matches what the decoded-frame inbox
can buffer per burst. With ≥2 segments in flight the immediate-ACK rule engages
(ACK every 2nd segment, no 10 ms stall) and dup-ACKs/fast-retransmit work again.

**To re-measure on hardware** (same method as §2): full-MTU bulk into :9999 —
expect the ceiling to move well above 100 KB/s until FCS loss binds; then
optionally re-run the §5 MSS sweep, where the knee (§3.2) says ~600–1000 B
frames should now win. Watch `inbox_drop`/`inbox_hwm` — if drops appear, bump
`INBOX_SLOTS` alongside `max_burst_size` (each slot is ~1.6 KB static RAM).
README throughput table should be updated only after re-measurement.
