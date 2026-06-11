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

## 9. RX-of-bulk: ACK pacing + window + MTU sweep — 96 → ~200 KB/s (2026-06-10)

Branch `rx-ack-pacing`. Three coordinated changes, each A/B'd on the wired rig
(`/tmp/rx_bench.sh`-style: `cat /dev/zero | nc <dev> 9999`, flow control =
device RX rate, 3-5 runs per config; FCS% from `[Rx]` lines with dec>50):

| config | mean RX | FCS-fail |
|---|---|---|
| baseline: `Some(1)` + 10 ms delayed-ACK | 93–96 KB/s | 27–33 % |
| immediate ACK, `Some(1)` | 129–141 | 3–5 % |
| immediate ACK, `Some(2)` (full MTU) | 163–198 | ~27 % |
| immediate ACK, `Some(4)` (full MTU) | 163–186 | ~32 % |
| **immediate ACK, `Some(2)`, `mss-clamp` MTU=1000** | **174–208** | **1–3 %** |

1. **`set_ack_delay(None)`** on the :9999 sink (main.rs). At a 1-segment
   advertised window smoltcp's immediate-ACK rule (>1 MSS unacked) can never
   fire, so every segment paid the full 10 ms delayed-ACK timer — and that
   fixed 10 ms phase sits exactly on Linux's tail-loss-probe timer
   (`max(2·srtt, 10 ms)` = 10 ms at our sub-5 ms srtt), so host probe
   retransmits and our delayed ACKs repeatedly transmitted into each other on
   the half-duplex wire. Most of the "27–33 % decode loss" at baseline was
   actually this collision storm (device decoded 2–3× more frames than
   goodput). Immediate ACKs ride the post-frame quiet gap. ACK *coalescing*
   (1 ms delay ≈ ACK every 2nd segment) is strictly worse (54–78 KB/s, 43 %):
   a timer-fired ACK lands mid-stream of the next inbound segment.
2. **`max_burst_size = Some(2)`** (eth_mac.rs). smoltcp clamps the advertised
   window to `max_burst_size × MSS` (iface/packet.rs; RX-window-only — TX
   verified unaffected at 505–941 KB/s). With ACK pacing fixed, 2 segments
   in flight lets the host pipeline; `Some(4)` adds loss with no gain
   (PR #1 regressed mainly because it opened the window *without* fixing
   ACK pacing).
3. **`mss-clamp` MTU 500 → 1000** (eth_mac.rs). The decode cliff (§5/§8) is
   sharp and sits just above 1004 B on-wire: 1004 B → ~1 % fail, 1104 B →
   12 %, 1204 B → ~25 %, 1504 B → ~30 %. MTU=1000 rides just under it.
   §5's refutation of MTU=500 still stands — and is now explained: at the
   old serialized 15 ms/segment regime, small segments just meant less data
   per cycle; under pipelining, tripling the frame/ACK rate explodes
   post-frame-gap contention (44 % fail). Bigger-but-sub-cliff wins.

The §4 "track CLOSED at ~100 KB/s" verdict is superseded: ~200 KB/s with the
clamp feature on, ~180 KB/s at stock MTU. The decode cliff itself (§8,
PHY-limited) still stands and is the binding constraint again.

### 9.1 Tried and rejected: DMA-half latency attacks

The per-segment cycle still carries the DMA half-fill latency (a frame can't
decode until the 16 KB / 2.18 ms half containing its end completes — also why
idle RTT is ~2.5 ms). Three attempts to cut it, all benched, none kept:

- **8 KB halves** (with carry accumulated across swaps — that fix IS kept in
  `poll_with`): decode (~1.7 ms for a 1 kB frame) runs inline in `DMA_IRQ_0`,
  so the half-fill time is also the decode deadline; 1.09 ms halves overran
  it and clipped the following back-to-back segment. 128–157 KB/s @ 50 %
  fail. Don't shrink `BUF_WORDS` without first moving decode out of the
  re-arm path.
- **Core-1 busy-poll live decode** (scan the in-flight half via the DMA write
  cursor, decode each frame as its run terminates): cut idle RTT to ~1.1 ms
  but regressed bulk. Root cause of the regression was never pinned —
  the UDP "discriminator" used to chase it was confounded (see below).
- **In-IRQ live scan piggyback** (one early scan at the end of each
  `DMA_IRQ_0`): clean, but a wash (161–198 KB/s) — at `Some(2)`/MTU-1000 both
  in-flight segments usually land in the *same* half and decode together, so
  the boundary case it accelerates is rare. Removed; not worth the unsafe
  cursor-aliasing complexity.

Next lever for the cycle time is **decode CPU cost** (~1.7 ms/kB-frame on the
240 MHz Hazard3): the DPLL inner loop, not more TCP/window tuning.

### 9.2 Methodology gotchas (so the next session doesn't repay this tuition)

- **UDP-to-closed-port is NOT a "no device TX" RX test**: smoltcp answers
  every datagram with ICMP port-unreachable (~160/s measured) and those
  replies collide with subsequent inbound frames. A whole bisect tree
  ("busy core 1 corrupts sampling") was built on this confound before the
  `tx_icmp` counter exposed it. Use **UDP broadcast** (no ICMP reply,
  `tx_icmp=0`) for a true zero-TX RX test.
- **Open question (pre-existing, NOT from this session's changes):** even on
  production builds, an unsynchronized 250 f/s × 996 B UDP broadcast blast
  fails ~50 % FCS while flow-controlled TCP at the same frame size fails ~1 %.
  Candidates: NLP TX landing mid-frame, the 4 ms cadence vs 2.18 ms half
  cadence, host NIC burst behavior. Worth its own investigation.
