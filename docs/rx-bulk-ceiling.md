# RX-of-bulk decode ceiling — characterization

**One-liner:** the device receives bulk TCP at only **~100 KB/s** (vs ~970 KB/s for
TX) because at full-MTU **~30 % of inbound frames fail FCS from clock drift**, which
collapses the host's TCP congestion window to 1–2 segments (`ss`: cwnd 10→1–2,
ssthresh→2, thousands of retransmits, RTT a healthy 3 ms). It is **loss-limited**,
**not** loop/`max_burst` (the main loop runs ~107 K iters/s), RTT, inbox, or DMA.
A **secondary receive-window ceiling** (~1–2 segments in flight) surfaces once loss
is removed. **The intuitive MSS-clamp fix was tested and REFUTED** (§5) — small
frames decode clean but hit the window ceiling at *lower* throughput (34 KB/s).

**Status:** characterization + MSS-clamp + frame-rate-ceiling experiments done
(2026-06-03). Follow-on from the full-duplex experiment
(`docs/full-duplex-analysis.md` §7.9 / H4), which surfaced the ~102 KB/s figure.
**Primary fix = the CPU-DPLL decoder** (`docs/clock-recovery-decoder-plan.md`,
`docs/cpu-dpll-plan.md`); then the receive-window depth (§5).

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
1. **Fix full-MTU decode (the primary lever).** Eliminating the 32 % FCS loss stops
   the cwnd collapse → throughput rises toward the window ceiling. The CPU-DPLL
   clock-recovery track (`docs/cpu-dpll-plan.md`). This is the one that matters for
   real (full-MTU) bulk.
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
livelock or a panic; no watchdog is enabled). Needs a dedicated repro + the RP2350
watchdog (backlog §4-F). DoS-shaped, but full-MTU bulk RX is normal traffic, so
this matters.

## 7. Next steps

- **Fix full-MTU decode (the primary lever).** Removing the 32 % FCS loss stops the
  cwnd collapse → throughput rises toward the window ceiling. CPU-DPLL clock
  recovery (`docs/cpu-dpll-plan.md`). Re-measure RX-of-bulk + the §3 curve + `ss`
  cwnd/retrans after.
- **Then raise receive-window / in-flight depth** (§4) — once loss is gone the
  ~1–2-segment cap bites; check smoltcp's advertised window vs the 32 KB sink buffer
  and whether window scaling is off.
- **Repro + root-cause the sustained-full-MTU hang** (§6); add the RP2350 watchdog.
- **Ruled out, don't pursue:** `max_burst_size`/main-loop (107 K iters/s), RTT
  (3 ms), and MSS clamp (refuted, §5).
