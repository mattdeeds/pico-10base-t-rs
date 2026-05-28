# PIO clock-recovery decoder — design plan (Phase 2)

Status: **Phase 2b brought up on hardware — works to ~554 B, full-MTU slips**
(2026-05-27). The decoder runs on PIO1 SM0 at zero CPU cost and **cancels the A1
clock drift**, but a residual jitter-limited slip caps reliable frames below
full MTU. Next: a more robust PIO edge classifier (see §2b results) before 2c
integration. Parent: `docs/clock-recovery-decoder-plan.md`.

### Phase 2b on-wire results (2026-05-27)

Brought up `src/eth_rx_pio.rs` + the `main.rs` TEMP dump scaffolding on hardware
(flashed via picotool — no SWD probe attached; the change is additive so the
device stays live). Validated with `tools/clock-recovery/pio_dump.py` (blasts
known-pattern frames `--size N`, reassembles the device's decoded-byte UDP dumps,
SFD + FCS + per-byte bins) and `analyze.py` (post-mortems saved `dumps/win_*.bin`).

| payload | frame | FCS-OK |
|---|---|---|
| 256 B | ~298 B | 8/8 |
| 512 B | ~554 B | 8/8 |
| 768 B | ~810 B | 6/8 |
| 1472 B | ~1518 B | ~0 (slips at byte ~1000–1346) |

**The decode is flat-perfect (no drift ramp) up to an abrupt loss-of-lock**, after
which the decoder free-runs emitting a steady `0xaa`/`0x55` (it starts catching
*every* Manchester edge — mid-bit and boundary). So clock recovery **works** — it
cancels the gradual A1 drift (open-loop capped at ~575 B with a 50% ramp; this is
flat to ≥554 B then a cliff). The residual is a *different* problem: jitter margin
in the fixed-delay boundary skip.

**`[8]` (SKIP_DELAY) is optimal — do not raise it.** Both the resample (`jmp pin`,
~T+D+2) and the wait re-arm (~T+D+4) must fall between the boundary edge (~T+7.5)
and the next mid-bit (~T+15); centring the pair ⇒ D+3 = 11.25 ⇒ D≈8. On wire,
`[8]` slips at *varying* bytes (jitter-limited = well-centred); `[9]` was *worse*
— a *deterministic* slip at byte 960 (wait re-arm at T+13 misses early-jittered
mid-bit edges). So the fixed-delay scheme is at its margin limit.

**Inter-edge-interval classifier (Phase 2c) — tried, did NOT beat the fixed
delay.** Implemented a 4-state PIO program (level × at-mid/after-boundary) that
*measures* each inter-edge interval via a `jmp x--` countdown and classifies
boundary (~T/2) vs mid-bit (~T); validated offline (`decode_pio_interval_model`,
FCS-ok N/N). On-wire it slipped at the same region (~893–1216 B) as `[8]`.
**Why: slips occur *inside runs of identical bits* — a Manchester square wave,
edges uniformly every T/2 — where the interval gives no discrimination; the
decode rides on mid/boundary state alternation and a single jittered/missed edge
flips the phase.** The divergence is a *clean* bit-slip (perfect up to the exact
byte, then abrupt `0xaa`), i.e. single-edge sensitivity, not gradual PHY noise.
THRESH=4 is well-centred, so it's not a tuning miss.

**Conclusion: per-edge-decision decoders (fixed-delay AND interval-classifier)
are fundamentally limited by single-edge sensitivity.** To reach full-MTU
≥95% FCS-OK, the next direction is a **DPLL-style decoder** (Candidate A in the
parent plan): a phase-locked bit clock with a loop filter that samples at bit
centre and is *averaged*, so it rides through a single bad edge rather than
slipping on it. Sub-~600 B frames already decode clean with either scheme today.

Production implementation of the clock recovery validated offline in Phase 1.
Parent: `docs/clock-recovery-decoder-plan.md`.

**Phase 2a result:** the streaming PIO decoder *logic* — poll for a level
change, emit the pre-edge level as the bit, skip `D` samples past the boundary
edge, resample, then CPU-side SFD-find on the emitted bitstream — is **validated
on the corpus** (`harness.py::decode_pio_model`): **FCS-ok N/N, tail bin 0%**,
matching edge-track. The boundary-skip delay works at **D=4** (of 6 samples/bit
at the 60 MHz corpus rate); D=3 catches the boundary edge and D=5 overshoots
when drift quantizes a bit to 5 samples — i.e. the working window is *one
sample* at this coarse rate. The real PIO at a 150 MHz SM (15 cycles/bit) has
2.5× finer timing, mapping D to **~10–11 cycles with ~7 cycles of margin** — far
more robust. To tune the delay precisely (and confirm the margin), **capture a
150 MHz-resolution corpus** before/while writing the PIO program (2b).

## 1. Why PIO (not CPU)

Phase 1 proved the **edge-relative** algorithm cancels drift (corpus FCS-ok N/N,
flat bins). But a CPU port does a per-bit edge search (~2–4× the current per-bit
cost), so a full-MTU decode (~12 k bits) is ~1.6–3.3 ms — over the 2.18 ms IRQ
budget. PIO does the same edge-relative work **in hardware at zero CPU cost**,
so it fixes *both* A1 blockers in one move:
- **Finding 1 (drift):** re-syncing to each transition is drift-immune by
  construction.
- **Finding 2 (single-core load collapse):** the CPU no longer decodes samples
  — the PIO emits decoded bytes; the CPU only finds SFD + checks FCS.

It also shrinks the RX memory/DMA footprint ~6× (decoded bytes, not 60 MHz
samples) and largely retires the carry/stitch machinery.

## 2. How the validated algorithm maps to PIO

Phase 1 algorithm: transitions recur ~6 samples apart at the per-bit mid-bit
edge; the bit value is the line level just *before* the edge; skip the
conditional boundary edge (a half-bit off) by only looking ~a full bit after
the last edge. In PIO that becomes: **wait for a level change (the edge),
emit the pre-edge level as the bit, delay ~0.6–0.7 bit to step past the boundary
edge, resample the level, repeat.** The edge is the clock reference, so drift
never accumulates.

## 3. PIO decoder design

Run the decode SM at **sysclk (150 MHz)** for fine edge resolution (~15 SM
cycles / 100 ns bit; ~3-cycle poll ⇒ ~20 ns edge resolution, < the 50 ns
half-bit). `Y` holds the current pre-edge level (= pending bit value); `D` is
the tuned boundary-skip delay:

```
.wrap_target
poll:
    mov  x, pins            ; x = current line level (1 pin)
    jmp  x != y, edge       ; level changed -> a transition
    jmp  poll
edge:
    in   y, 1   [D]         ; emit the pre-edge level as the bit; delay D
    mov  y, pins            ; resample after D (past the boundary edge)
    jmp  poll
.wrap
```

- **Clock recovery** is implicit: every bit re-references timing to its own
  edge; `D` is a *sub-bit* delay re-referenced each bit, so it can't accumulate.
- **Data capture:** the bit = `Y`, the level held before the detected edge
  (stable for ~half a bit, so robust to the poll latency).
- **Output:** `in y, 1` shifts bits into the ISR; autopush at 8 (bytes) or 32
  (words) → RX FIFO → DMA → CPU. ~6 instructions; fits easily.
- `D` and the exact poll structure are tuned by the Phase-2a model (below).

## 4. Idle, framing, byte alignment

- **Idle** (no edges, TP_IDL): `poll` spins (`x == y`) and emits nothing — frame
  gating falls out for free; no idle garbage.
- **Framing / byte alignment:** the FIFO is a *bitstream*; byte boundaries are
  arbitrary and a frame's trailing partial byte abuts the next frame. The CPU
  re-aligns at every frame by scanning the decoded bitstream for the
  preamble/SFD (`…0x55 0xD5`) — cheap byte/bit scanning, *not* sample decoding —
  then reads the frame and checks FCS. This is the entire remaining CPU RX cost.
- Optional refinement: a PIO stall-timeout (countdown in X) that flushes the
  ISR + raises an IRQ on idle, to mark frame ends explicitly. Adds instructions;
  start with CPU-side SFD framing.

## 5. Integration (replaces the sample path)

- New decode PIO program **replaces** the `in pins,1` sampler. Put it on its own
  PIO block (**RP2350 has PIO0/1/2** — move RX to PIO1 so it doesn't fight the
  TX program for PIO0's 32-instruction space; gotcha #1 `.origin 0` only matters
  if we use `out pc`, which this program does not).
- DMA the decoded-byte FIFO into a (much smaller, ~6×) buffer; IRQ on
  half-fill as today.
- `eth_rx.rs`: the IRQ handler shrinks to **SFD-scan + frame-extract + FCS** on
  the decoded byte stream. `decode_frame` (sample → bits), the carry/stitch, and
  `find_active_run_from` (sample-domain) largely retire or simplify.
- `eth_mac.rs` / inbox / smoltcp glue unchanged downstream.

## 6. Phase 2a — software model first (de-risk before any PIO)

As in Phases 0/1: **model the PIO logic in software against the corpus before
writing PIO.** Step the loop (poll for level change → capture `Y` → delay `D` →
resample) over the captured sample stream and check it yields the known frame
(flat bins, FCS N/N). Sweep `D` and the poll cadence to find the working window.
- Resolution caveat: the corpus is 60 MHz (6 samples/bit); modeling a 150 MHz SM
  needs upsampling (hold) or a **finer capture** (bump the capture sampler to
  150 MHz for a timing-accurate corpus). Start with the 60 MHz logic model to
  confirm correctness; capture finer if timing/jitter margins look tight.
- Deliverable: a `decode_pio_model` in `tools/clock-recovery/harness.py` scoring
  N/N, plus the chosen `D` / SM clock.

## 7. Phase 2b/2c/2d

- **2b — PIO program:** write + assemble the decoder (`pio_asm!`), bring it up on
  PIO1, confirm it produces decoded bytes for a live frame.
- **2c — integration:** rework `eth_rx.rs` RX path to the decoded-byte stream
  (DMA + SFD-scan + FCS); wire into the existing inbox/smoltcp path.
- **2d — on-device acceptance** (same gates as the parent plan):
  per-byte error bins flat; **full-MTU FCS-ok ≥ ~95%**; multi-size echo
  round-trips; gotcha-#10 stress at/above baseline; and **measure the CPU RX
  load drop** (the Finding-2 payoff — expect the IRQ to fall from ~ms to ~µs).

## 8. Risks & open questions

- **Edge resolution vs SM clock** — the poll loop must catch edges well within a
  half-bit; 150 MHz / 3-cycle poll ≈ 20 ns should be ample, but confirm in 2a/2d.
- **`D` tuning** — must land the resample between the boundary edge (~0.5 bit)
  and the next mid edge (~1.0 bit); wide margin (~±100 ppm is tiny vs that
  window), but tune on the model.
- **Polarity** — `Y` is the pre-edge level; the CPU's SFD-find establishes
  polarity (invert if needed), as today.
- **Idle/false edges** — noise during TP_IDL could emit junk bits; the CPU SFD
  framing discards inter-frame junk, but watch for pathological cases.
- **Bitstream framing robustness** — back-to-back frames, runts, and lost
  preambles must not desync the CPU scanner; design the SFD-scan to resync
  per frame.
- **TX coexistence** — keep TX on PIO0; RX decoder on PIO1 (independent).
- **First on-wire bring-up** — flash via SWD the first time in case the RX path
  rework is mid-change (lesson from the wedged-app incident).

## 9. Fallback

If the PIO proves too jittery or infeasible, fall back to the **CPU edge-track**
(Phase 1, validated) **+ dedicate the 2nd Hazard3 core to the NIC** so the
per-bit cost fits. The offline model + corpus carry over either way.

## 10. Acceptance criteria

1. Full-MTU FCS-ok ≥ ~95% at low + moderate rate (today ~1.7%).
2. Per-byte error bins flat across all positions.
3. **CPU RX load drastically down** (decode off the CPU) — measured.
4. gotcha-#10 on-wire stress at/above baseline (ping/UDP/errs).
5. Multi-size echo round-trips, including large frames.

## 11. PIO DPLL — goal condition (acceptance, locked 2026-05-27)

Supersedes §10 for the DPLL route. The per-edge decoders (fixed-`[8]`-delay,
interval-classifier) decode clean to ~554 B then a *single* bad edge causes
permanent loss-of-lock (the 0xaa cascade). The DPLL must hold a loop-/window-
filtered bit clock that samples at bit-centre and **rides through individual bad
edges** instead of slipping on them.

**Primary**
- **P1 — Full-MTU correctness.** At a non-saturating rate, full-MTU (1518 B)
  frames decode **FCS-ok ≥ 95 %** (stretch ≈ small-frame ~98 %); today ~0 %.
  *Escape hatch:* if a residual floor sits below 95 %, it must be **PHY-limited,
  not decoder-limited** — per-byte bins **flat/uniform** across positions, **no
  drift ramp** and **no tail cliff**. A flat residual = decoder goal met; the
  floor is then a hardware matter. *(Accepted 2026-05-27.)*
- **P2 — No loss-of-lock cascade (the DPLL property).** A single bit/edge error
  stays **local**: no frame shows the slip signature (clean prefix → 0xaa/0x55
  run to end-of-frame); failing frames show **isolated** byte-error blips.

**Secondary**
- **S1 — No small-frame regression.** 256 B & 512 B payloads still FCS-ok ~8/8.
- **S2 — Zero CPU decode cost.** Decode stays in PIO; CPU RX = SFD-scan + FCS
  only; RX IRQ worst case drops from ~2.57 ms toward ~µs (measured).
- **S3 — gotcha-#10 stress (post-integration).** ping ≥ 99.7 %, UDP echo
  ~100 %, host RX errs ≤ 2 / 30 s.

**Constraints:** fits PIO1's 32-instruction space; TX stays on PIO0; fixed-point
only (no FPU); honour `.origin 0` if `out pc` is used (gotcha #1).

**Measurement:** P1 — `pio_dump.py --size 1472` at a reduced (non-saturating)
blast rate, ≥100 windows (bring-up); device `dec/ok/fail` counters @~150 pps
(integration). P2 — `analyze.py` per-64-B-block error *pattern*. S1 —
`pio_dump.py --size 256/512`. S2 — `mcycle`. S3 — 30-s concurrent stress.

**Milestones:** bring-up (DPLL in parallel + `pio_dump.py`) proves P1/P2/S1;
integration (2c) proves S2/S3.

## 12. PIO DPLL — design (windowed absolute-phase tracking)

**Principle.** Maintain a free-running bit-clock phase (an NCO: a counter that
spans one bit period). **Sample** the data at a fixed phase in the 2nd half-bit
(= the bit value, per this convention). **Resync** the phase to Manchester
**mid-bit** edges that land within a window around the expected mid-bit phase;
**ignore** edges outside it (boundary edges, noise).

**Why this gets P2 (and why the interval-classifier didn't).** There is **no
per-edge alternation state** to corrupt — the phase is *absolute*. Mid-bit vs
boundary edges are separated by **phase** (mid-bit ≈ ½-bit, boundary ≈ 0/1-bit),
**not** by interval-since-last-edge — which is exactly what failed inside runs of
identical bits (a square wave where *every* interval is T/2 and the decode rode
on a flippable alternation bit). Effects of a single perturbation are bounded:
an in-window bad edge → phase bump ≤ window (data sample still in the right
half); an out-of-window spurious edge → ignored; a missed mid-bit edge → coast
one bit at the nominal period, re-lock on the next. None cascade.

**Parameters.** 150 MHz ⇒ N = 15 cycles/bit; mid-bit phase ≈ N/2 = 7.5; sample
phase ≈ 3N/4 ≈ 11; window ≈ ±N/4 ≈ ±3.75 (separates mid-bit at 7.5 from
boundary at 0/15). Offline model (60 MHz corpus): N = 6, mid = 3, samp ≈ 4–5,
window ±1. (Optionally 2nd-order later: track the period N itself to null the
ppm steady-state lag; start 1st-order.)

**PIO mapping (the hard part — main risk).** A per-cycle poll loop is ~2 instr
(`jmp pin` + `jmp x--`) ⇒ ~7–8 phase ticks/bit at 150 MHz, so the window/sample
landmarks live in tick units (coarse). Phase counter in X; level/edge via
`jmp pin` + the code path; resync = reload the phase counter to the mid tick;
sample = `in` the level at the sample tick. No `out pc` ⇒ `.origin 0` not needed.
Coarse time resolution is the chief feasibility risk — validate the achievable
window offline and, if tight, on a 150 MHz capture.

**Offline validation plan (de-risk before any PIO).**
1. `decode_dpll_model` on the clean corpus → FCS-ok N/N (logic + polarity + params).
2. **Jitter/glitch injection:** perturb corpus edges (±sample displacement,
   dropped/added edges) and show the DPLL model **rides through** where
   `decode_pio_interval_model` slips — proves P2 offline.
3. Capture a 150 MHz corpus for timing-accurate window/sample tuning if the
   tick resolution looks tight.
4. Then write the PIO program; bring up in parallel; measure against §11.

**Model — first result (2026-05-27).** `decode_dpll_model` (harness.py, window-
only, N=6/samp=4/win=1) on the clean 60 MHz corpus: **2/3 frames decode the
ENTIRE 1518 B payload flat-perfect, FCS-OK** — the DPLL **holds full-MTU lock**,
which no per-edge decoder did (P2 hypothesis confirmed in principle). The bins
are uniform across all positions (no drift ramp, no tail cliff): a frame either
locks (whole frame perfect) or misses acquisition (uniform garbage) — all-or-
nothing, the DPLL signature. The 3rd frame (cap_5) fails **acquisition**: the
free-running phase's constant offset during the preamble never enters the narrow
window, and the corpus captures **start mid-junk with no preceding idle** so
wide-window / first-edge acquisition snaps to leading noise (both tried, 0/3).
*Two limits are corpus artifacts, not algorithm flaws:* (a) 60 MHz forces
win=±1 (only value separating mid-bit phase 3 from boundary 0/5) — 150 MHz gives
±~3.75; (b) no preceding idle — real frames have an idle gap before the preamble,
so the first post-idle edge is a clean mid-bit (easier acquisition than the
corpus). **Next:** capture a 150 MHz corpus (finer window + idle context) to tune
window/sample/acquisition, OR go straight to the PIO program and tune acquisition
on-wire (idle context present) with `pio_dump.py`. The full-MTU-lock result
de-risks the core idea; acquisition robustness is the remaining open item.
