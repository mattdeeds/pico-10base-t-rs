# RX-of-bulk decode ceiling — characterization

**One-liner:** the device receives bulk TCP at only **~100 KB/s** (vs ~970 KB/s
for TX) because **full-MTU inbound frames fail FCS at a high, clock-drift-driven
rate** — clean (~3–4 %) up to ~512 B, then a **cliff** to tens-of-% by ~700 B and
**~72 %** at full-MTU (1472 B). It is **not** inbox/DMA/window-limited. This is the
documented clock-recovery decoder ceiling, now isolated on the RX-of-bulk path.

**Status:** first characterization milestone (2026-06-03). Follow-on from the
full-duplex experiment (`docs/full-duplex-analysis.md` §7.9 / H4), which surfaced
the ~102 KB/s figure. Root-cause track = the CPU-DPLL decoder
(`docs/clock-recovery-decoder-plan.md`, `docs/cpu-dpll-plan.md`).

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

## 4. Why this caps RX-of-bulk at ~100 KB/s

Real bulk traffic is full-MTU. At ~28–72 % frame loss, TCP can't sustain a window —
constant retransmits + cwnd collapse — so goodput settles at ~100 KB/s (and the
"ok" frames at that rate are ~half retransmits). The wire sits mostly idle; the
limit is purely decode reliability, not bandwidth.

## 5. Mitigations

1. **Fix the decoder (the real fix).** Better clock recovery so full-MTU frames
   decode — the existing CPU-DPLL-on-core-1 track (`docs/cpu-dpll-plan.md`). Lifts
   the ceiling for *all* RX, not just bulk.
2. **TCP MSS clamp (cheap partial mitigation).** Clamp the advertised MSS so on-wire
   frames stay **below the knee (~512 B)**, where loss is ~3–4 % (TCP-survivable).
   Trades per-frame efficiency for staying out of the failure region — could lift
   RX-of-bulk substantially without touching the decoder. Ties to backlog §4-E
   (`docs/perf-characterization-plan.md`). Caveat: the knee is δ-dependent, so clamp
   conservatively, and this only helps RX (TX is already fine).

## 6. Robustness bug found (separate)

A **sustained max-rate full-MTU UDP flood hung the device** — link dropped (no
NLPs → host `Link detected: no`), CDC went silent, but USB stayed enumerated and
SWD still worked; a reflash/reset recovered it cleanly. Rate-limiting (~400 pps)
avoided it, so it's **saturation-related, not size-related**. No `inbox_drop`/
`carry_cap` preceded it, so the hang is elsewhere (decode/IRQ livelock or a panic
without the watchdog — none is enabled). Worth a dedicated repro + a hardware
watchdog (backlog §4-F). Not a normal-traffic risk, but a DoS-shaped one.

## 7. Next steps

- Confirm the MSS-clamp win: clamp to ~512 B and re-measure RX-of-bulk (cheap, could
  be a big practical lever before the decoder work).
- Decoder: the CPU-DPLL track is the durable fix — re-measure this curve after.
- Repro + root-cause the saturation hang (§6); add the RP2350 watchdog.
- Optional: instrument core-1 decode cycles/frame vs size (the router-gated mcycle
  `CycleSpan`) to correlate decode cost with the fail cliff.
