# RX-of-bulk decode ceiling — characterization

**One-liner:** the device receives bulk TCP at only **~100 KB/s** (vs ~970 KB/s
for TX) due to **two stacked limits** (§4): a **primary ~150 good-frames/s rate
ceiling** (size-independent — prime suspect smoltcp `max_burst_size = Some(1)` /
the HD ACK-TX path) and a **secondary ~30 % full-MTU FCS-fail tax** from clock
drift (clean ~3–4 % up to ~512 B, cliff to ~72 % at full-MTU). It is **not**
inbox/DMA/window-limited. **The intuitive MSS-clamp fix was tested and REFUTED**
(§5) — smaller frames decode clean but goodput drops 3× because the rate ceiling
is size-independent.

**Status:** characterization + MSS-clamp experiment done (2026-06-03). Follow-on
from the full-duplex experiment (`docs/full-duplex-analysis.md` §7.9 / H4), which
surfaced the ~102 KB/s figure. Decode-fix track = the CPU-DPLL decoder
(`docs/clock-recovery-decoder-plan.md`, `docs/cpu-dpll-plan.md`); the bigger lever
is the rate ceiling (§5).

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

## 4. Why this caps RX-of-bulk at ~100 KB/s — TWO limits (revised)

The MSS-clamp experiment (§5) revised this. There are **two** stacked limits, and
decode-fail is the *smaller* one:

1. **PRIMARY — a ~150 good-frames/s rate ceiling, independent of frame size.**
   Across MTU 500/1000/1500 the device delivered a near-constant **~136–152 good
   frames/s** (`[Rx] ok/s`). So goodput ≈ `frame_rate × payload`, and the wire sits
   ~idle (150 × 1526 B ≈ 230 KB/s ≪ 1.25 MB/s line). The cause is per-frame, not
   bandwidth — prime suspect `eth_mac.rs:442` **`max_burst_size = Some(1)`**
   (smoltcp processes one packet per `poll()`, so RX is capped by the main-loop
   iteration rate), and/or the half-duplex ACK-TX carrier-wait stalling each
   iteration.
2. **SECONDARY — a decode-fail tax at full-MTU (~30 %).** The §3 clock-drift cliff
   costs ~28–32 % of full-MTU frames → retransmits, knocking the full-MTU number
   from a potential ~140 down to ~99 KB/s.

Goodput model `≈ frame_rate × payload × (1−loss)` fits all three rows in §5.

## 5. Mitigations — MSS clamp TESTED and REFUTED

**TCP MSS clamp (was the leading "cheap fix") does NOT work — it makes throughput
worse.** Measured on-device (`mss-clamp` feature lowers `eth_mac::MTU` → smoltcp
advertises a smaller MSS → peer sends sub-knee frames; bulk upload into the :9999
sink):

| device MTU | on-wire frame | RX FCS-fail | **upload goodput** |
|---|---|---|---|
| 500  | ~526 B | **~0 %** | **34 KB/s** |
| 1000 | ~1026 B | ~1–13 % | **68 KB/s** |
| 1500 | ~1526 B | ~32 % | **99 KB/s** |

Clamping cleaned decode (32 %→0 %) but **cut throughput 3×** — because the ~150
frames/s ceiling is size-independent, so smaller frames just carry less. Bigger
frames win even with loss. **Conclusion: do not clamp MSS.**

Real levers, in priority order:
1. **Lift the ~150 frames/s ceiling (the big win).** Investigate
   `max_burst_size = Some(1)` (let smoltcp drain multiple frames per poll), the
   main-loop per-iteration cost, and the HD ACK-TX carrier-wait. This raises *all*
   frame sizes proportionally and is independent of decode.
2. **Fix full-MTU decode (removes the ~30 % tax).** The CPU-DPLL clock-recovery
   track (`docs/cpu-dpll-plan.md`) — would take the full-MTU number from ~99 toward
   the frame-rate ceiling (~140+). Complementary to lever 1.

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

- **Chase the ~150 frames/s rate ceiling (highest leverage).** Try
  `max_burst_size` > 1 (drain several frames per `iface.poll`) and/or profile the
  main-loop per-iteration cost + the HD ACK-TX carrier-wait; re-measure RX-of-bulk.
  This is size-independent so it lifts every case.
- **Decode fix** (CPU-DPLL, `docs/cpu-dpll-plan.md`) — removes the ~30 % full-MTU
  tax; re-measure the §3 curve after. Complementary to the rate-ceiling work.
- **Repro + root-cause the sustained-full-MTU hang** (§6); add the RP2350 watchdog.
- MSS clamp: **done, refuted** (§5) — do not pursue.
