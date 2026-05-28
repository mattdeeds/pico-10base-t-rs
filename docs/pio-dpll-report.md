# PIO DPLL — what we tried, why it stops at ~40 % full-MTU

A retrospective on the Phase 2b–2d attempt to build a Manchester clock-recovery
decoder entirely in PIO on the RP2350. Companion to `pio-decoder-plan.md`
(detailed plan + iteration log) and the `pio-decoder-phase2b-onwire` memory.

Written 2026-05-28 after deciding to pivot to a CPU DPLL on the 2nd Hazard3
core.

## TL;DR

**Best PIO-only result: ~40 % full-MTU FCS-OK** (v1+180 MHz: `eth_rx_pio.rs`
at commit `d72360f`), with 6/6 FCS-OK at 512 B / flat 0 % bins on successful
decodes. **Ceiling is structural, not a tuning miss**: PIO has no arithmetic
for a real loop-filtered DPLL, the 2-cycle input synchronizer forces a
mutual-exclusion between "polls land on the edge" and "loop = bit period," and
mid-bit vs boundary Manchester edges are both real, validated transitions —
distinguished only by *phase*, which PIO can't track. CPU DPLL on the unused
2nd Hazard3 core is the path to ≥95 %.

## Goal (locked acceptance criteria, see `pio-decoder-plan.md` §11)

- **P1**: Full-MTU (1518 B frame) FCS-OK **≥ 95 %** at low rate, OR — if a
  residual floor remains — that floor is *PHY-limited* (flat per-byte error
  bins, no drift ramp, no slip cascade), not decoder-limited.
- **P2**: No loss-of-lock cascade. A single bit/edge error stays local; no
  frame may show the `0xaa`/`0x55` cascade signature.
- **S1**: small frames (≤512 B) no worse than baseline.
- **S2**: zero CPU decode cost (the Finding-2 payoff).

A1 baseline (open-loop CPU decoder, what we were trying to beat): ~575 B clean
then a drift ramp through 50 % to ~89 % errors at full MTU; full-MTU FCS-OK
~1.7 %.

## What we tried (iterations)

| Phase | Approach | Result | Why it didn't clear the bar |
|---|---|---|---|
| 2b — fixed `[8]` delay (`b863651`) | After mid-bit edge, blind delay D ≈ 0.67 bit, resample level, repeat | Clean to ~554 B (single-edge slip past that) | Wait catches *any* edge after the coast — a jittered boundary edge or noise becomes the new phase reference, slipping the loop |
| 2c — interval classifier (offline + PIO) | Measure inter-edge interval; classify ~T (mid-bit) vs ~T/2 (boundary); track at-mid/after-boundary state | Same slip rate as `[8]` | Inside runs of identical bits the Manchester signal is a clean square wave (edges every T/2); *every* interval is T/2, so interval gives no discrimination. The mid/boundary state alternation is then the entire decode — and a single bad edge flips it forever |
| 2d v1 — sample-by-pin (`5148c8a`) | 7 PIO instr: `nop[2] + in pins,1 [7] + jmp_pin + wait`. Bit value comes from `in pins, 1` (line level) not from state | 56–141 B @150 MHz; **926+ B with ~40 % full-MTU @180 MHz** (`d72360f`) | Sample-by-pin breaks bit-level cascade (a mis-tracked edge doesn't propagate to all subsequent bits via the LOW/HIGH state). But the **wait still has no phase qualification** — a single noise/jittered edge becomes the new phase and the SM slips into the half-bit boundary stream, producing the `0xaa`/`0x55` cascade |
| 2d v2 — windowed counter polling (`02bc710`) | Replace `wait` with `set x` + `jmp x--, poll` for a counter-driven window | Decoded 1 full 512 B payload byte-perfect once! But full-MTU drift +9 cyc/missed-window, desync within bits | Pre-poll (10) + polling fall-through (7) + jmp coast (1) + nop[4] coast (5) + jmp sample (1) = **24 cycles on the no-edge path** vs the **15-cycle bit period at 150 MHz**. The counter overhead doesn't fit |
| 240 MHz overclock + v1 (`8845a38`) | Same v1 PIO, just sysclk 180 → 240 MHz ⇒ 24 SM cyc/bit | Boots clean (ping 1.7–5.7 ms RTT), TX/RX dividers integer (TX÷12, RX÷4), no fractional jitter | Auto-adjusting `wait` still works at the new bit period. Useful infra but the structural blockers below don't change |
| 2d v3 — unrolled hard window (per GPT-5.5 review) | Trade instruction space (we have 25 free slots) for cycles — unroll `set x` + `jmp x--` into explicit `jmp pin, edge` polls; miss-coast via `.wrap` for 0-cyc return | Produced mostly idle `0xff` reads (1608 / 2048 bytes per window). No preamble visible | The math is *mutually exclusive*: to land polls on the synchronizer-delayed expected edge (PIO cyc 18 at 180 MHz) needs pre-poll = 17 cyc, leaving 1 cyc for the entire polling window — no fit. Keeping miss-coast = bit period forces polls to cyc 15-16, *before* the edge, so polls never catch → no acquisition |
| **2-SM clock+data split** (analyzed) | SM1 owns edge detection + windowing + IRQ at validated mid-bit; SM2 waits IRQ + samples at calibrated delay | Not implemented; analysis says won't break the ceiling | Same boundary-edge ambiguity. Glitch validation (delay + recheck after wait) filters noise but **not** boundary edges (which are real, validated line transitions). Distinguishing mid-bit from boundary requires phase tracking → arithmetic → PIO can't |

## The structural blockers (root cause analysis)

**Why no PIO design we tried clears ≥95 % full-MTU.** Three independent
constraints all hit at the same ceiling:

### 1. PIO has no arithmetic — can't do a loop-filtered DPLL

A real DPLL maintains a fractional phase accumulator and nudges it by a small
fraction of the edge error each iteration. The averaging is what gives
*single-edge ride-through* (P2): one bad edge moves phase by a tiny fraction,
not a whole bit period.

PIO has only **decrement** (`jmp x--`/`jmp y--`) — no add, no subtract, no
shift-by-N, no fractional arithmetic. The best PIO can do is *fully* re-anchor
to each detected edge — which is exactly the single-edge-sensitivity v1 has.
The offline-validated `decode_dpll_model` uses Python arithmetic; that
algorithm doesn't port to PIO.

### 2. The 2-cycle input synchronizer creates a structural cycle-budget mutual-exclusion

PIO sees pin transitions 2 SM cycles after the real-world edge (synchronizer
delay, in PADS). For a windowed DPLL where the polling lands on the expected
edge AND the loop period equals the bit period, the math requires *both*:

- Pre-poll cycles (sample + direction-decide) + N polls + return ≤ bit period
- First poll cycle ≥ bit period (so poll sees the synchronizer-delayed edge)

These are mutually exclusive: if first poll is at the bit period, you've
consumed the whole budget with pre-poll, leaving 0 cycles for the polling
window. Going to 180 MHz (18 cyc/bit) or 240 MHz (24 cyc/bit) doesn't change
this — the structural ratio is the same. Synchronizer bypass via
`PIOx.INPUT_SYNC_BYPASS` saves 2 cyc absolute but doesn't change the
relative cycle math inside a self-clocked loop, and adds metastability risk
on the irreversible phase decisions.

### 3. Mid-bit vs boundary edges are distinguished only by phase

In a Manchester signal during a run of identical bits, the line is a clean
square wave at the half-bit rate. *Every* edge is a real, validated line
transition (the line really does flip and stay flipped for ≥ a half-bit).
The only thing distinguishing a mid-bit edge (= data event) from a boundary
edge (= clock-only artifact) is **absolute phase relative to the bit clock**.

- Glitch validation (delay + recheck) filters only *noise glitches* (line
  blips back) — not boundary edges.
- Interval classification (T vs T/2) fails inside identical-bit runs (every
  interval is T/2 — see Phase 2c).
- Per-edge-decision designs (`wait`-based, polling-based) all catch boundary
  edges as if they were mid-bit edges, half-bit-slipping the phase forever
  after (the `0xaa`/`0x55` cascade signature).

The only fix is *absolute phase tracking* — which needs arithmetic (blocker #1).

### 4. Single-edge sensitivity → cascade slips

Once a wrong edge captures the phase reference, the `in pins, 1` sample lands
in the *wrong* half-bit forever after — the SM is decoding the half-bit
boundary stream of the same data. Output is a steady `0xaa`/`0x55` to
end-of-frame. This is what failed P2.

Sample-by-pin (v1's win over the `[8]` edge-tracker) helps because the bit
value comes from the line level, not from a LOW/HIGH state code path that
would also flip. But it doesn't help with phase capture — once phase is
half-a-bit off, sample-by-pin still reads the wrong half.

## What we did achieve

- **10× improvement over the A1 open-loop ceiling.** Open-loop CPU decoder
  capped at ~575 B with a 50 % drift ramp. v1+180 MHz reaches 926–1472 B
  with ~40 % full-MTU FCS-OK and a *flat* 0 % bin profile on every successful
  decode (no drift signature — the DPLL effect *is* real, just below 100 %).
- **Clean small-frame decode**: 6/6 FCS-OK at 512 B with flat 0 % bins.
- **S2 (zero CPU decode cost) achieved** for the bring-up. v1 PIO does all
  the Manchester decoding; CPU sees decoded bytes. (Frames still need SFD +
  FCS in CPU but those are far cheaper than the per-bit sample decode.)
- **Sample-by-pin is a genuine architectural insight.** Worth keeping in any
  future PIO Manchester work — bit-from-line-level beats bit-from-state for
  cascade resistance.
- **240 MHz overclock validated as safe** on this board (flash QMI handles
  60 MHz QSPI SCK; TX/RX dividers become integer; CPU 33 % faster). Useful
  infrastructure regardless of decoder direction.

## Useful artifacts to keep

- `src/eth_rx_pio.rs` — v1 sample-by-pin + wait PIO program (committed at
  `d72360f`). The reference PIO Manchester decoder for this PHY.
- `tools/clock-recovery/harness.py` — offline decoder bench. Contains
  `decode_current` (open-loop reference), `decode_edge_track` (CPU DPLL,
  FCS-OK N/N), `decode_pio_model` (validated for v1), `decode_pio_interval_model`
  (the failed interval classifier — useful negative reference),
  `decode_dpll_model` (the windowed-phase algorithm we couldn't fit in PIO).
- `tools/clock-recovery/pio_dump.py` + `analyze.py` — live on-wire validator
  and dump analyzer. Reusable for any future PIO RX work.
- `tools/clock-recovery/corpus/*.bin` — captured raw sample buffers from the
  real PHY. Carries the wire's real ppm + jitter.
- `docs/pio-decoder-plan.md` — full plan with all iteration logs, cycle math,
  and the GPT-5.5 design review corrections.

## Lessons learned (carry forward)

1. **Statistics: need ≥100 frames per measurement point.** Our N-sweep used
   3–5 windows per point and back-to-back identical-code runs went 40 % → 0 %.
   Most of the apparent N-sweep signal was statistical noise.

2. **Flat bins on successful decodes ≠ P2 met.** P2 is about *failure* mode —
   our failures cascade with `0xaa`/`0x55` to end-of-frame, so P2 was never
   actually satisfied at the v1+180 MHz baseline (one of GPT-5.5's catches).
   Always look at the failure pattern, not just the success rate.

3. **Synchronizer bypass doesn't save cycle budget** in a self-clocked PIO
   loop. The 2-cycle synchronizer is a constant offset, not freed cycles.

4. **PIO `wait` is the wrong primitive when the failure mode is "wrong edge
   captured."** `wait` has no phase qualification; once it completes, you're
   committed. Polling allows windowing but has its own structural-budget
   problems.

5. **Verify on hardware with ≥100 windows before claiming a design works.**
   Cycle math on paper is necessary but not sufficient — small effects
   (synchronizer behaviour, PHY edge jitter) can invalidate a design that
   looks clean.

6. **Get a second opinion when you're hitting a wall.** GPT-5.5's review
   caught two real errors in my reasoning (P2 metric, synchronizer cycle
   accounting) and pushed v3 (which didn't work, but for *informative*
   reasons we now understand structurally).

## Path forward — CPU DPLL on the 2nd Hazard3 core

The validated `decode_edge_track` algorithm (Phase 1, offline FCS-OK N/N on
the corpus) runs on a CPU core. Dedicate the unused 2nd Hazard3 core to the
NIC RX path. Sidesteps PIO's lack of arithmetic and synchronizer constraints;
also addresses A1 Finding 2 (single-core load collapse) via core separation.

Plan in `pio-decoder-plan.md` §9 (Fallback). Bigger refactor than the PIO
attempts (smoltcp glue + IRQ migration + DMA into the second core's RAM
region) but it's the design that has the **arithmetic and state needed for
a real DPLL with loop filter**, which is what ≥95 % full-MTU requires.
