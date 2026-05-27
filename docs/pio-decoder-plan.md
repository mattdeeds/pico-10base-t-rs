# PIO clock-recovery decoder — design plan (Phase 2)

Status: planning (2026-05-27). Production implementation of the clock recovery
validated offline in Phase 1. Parent: `docs/clock-recovery-decoder-plan.md`.

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
