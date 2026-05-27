# Clock-recovery decoder — design plan

Status: **Phase 0 + 1 algorithm DONE** (2026-05-27); Phase 2 = PIO-side
production implementation, planned in **`docs/pio-decoder-plan.md`**. Unblocks full-MTU RX, the #1 router-foundation
blocker found in A1. See RESUME.md "Router project — A1 link characterization"
and the `project-vision-router` memory. The offline bench lives in
`tools/clock-recovery/` (corpus + `harness.py` + `capture.py`).

**Phase 1 result:** the **edge-relative re-anchoring** algorithm (track each
per-bit Manchester transition, sample one sample before it) **fully cancels the
drift** on the corpus — every position bin flat 0%, **FCS-ok N/N** (vs the
open-loop baseline's ~0→89% tail, FCS 0/N), robust across search-window
W = 1–3. Algorithm validated; see `harness.py::decode_edge_track`.

**Phase 2 caveat (drives the CPU-vs-PIO decision):** edge-track does a small
per-bit edge search (a few extra sample reads/bit). Rough estimate: ~2–4× the
current per-bit decode cost → a full-MTU CPU decode (~12 k bits) lands around
~1.6–3.3 ms, i.e. *borderline-to-over* the 2.18 ms IRQ half-fill budget. So the
CPU port needs an on-device cost measurement, and this strongly favours
**Candidate C (PIO-side recovery)** for production — the PIO does exactly this
(`wait` for each transition, sample just before) in hardware at zero CPU cost,
which *also* fixes A1 Finding 2 (single-core load collapse). The offline-
validated algorithm maps directly onto a PIO `wait`-on-edge program.

## 1. Problem (from A1, confirmed)

`decode_frame` locks sampling phase **once** at the SFD, then reads each data
bit open-loop at a rigid stride `sample = F + 4 + 6·k` (6 samples/bit at the
60 MHz sampler). There is **no feedback**, so the unavoidable ppm mismatch
between the host's 10BASE-T bit clock and our RP2350-derived sampler makes the
sample point walk off the bit center, linearly, as the frame goes on.

Per-byte error-position test (full-MTU known pattern, 120 pps):

| frame bytes | 42–593 | 594–777 | 778–961 | 962–1145 | 1146–1329 | 1330–1513 |
|---|---|---|---|---|---|---|
| byte-error rate | **0.0%** | 2.8% | 24% | **50%** | 74% | 89% |

Perfect for ~575 B, then a clean monotonic ramp through 50% (sample point on a
bit boundary, reading randomly) to 89%. Flat would mean PHY noise; this is the
textbook clock-drift signature.

**Quantitative model.** If the true bit period is `P = 6/(1+δ)` samples (δ = ppm
offset), the open-loop error after `k` bits is `(6 − P)·k ≈ 6δ·k` samples. Half
a bit (3 samples → 50% errors) is reached at `k ≈ 3/(6δ)` bits. The observed 50%
at ~byte 1050 (≈8400 bits) ⇒ **δ ≈ 60 ppm** — squarely inside 10BASE-T's
±100 ppm spec. So this is a *normal* clock offset, not a defect: the decoder
fundamentally needs clock recovery. **Firmware-fixable; not the analog PHY, and
full-duplex hardware would not address it.**

## 2. Goal & success criteria

Restore reliable full-MTU RX by continuously cancelling drift.

Primary acceptance (all measured offline first, then on-device):
- **Per-byte error bins flat** across all positions at the small-frame floor
  (drive the 778–1513 B bins from 24–89% down to ≈ the 64-B baseline).
- **Full-MTU (1518 B) FCS-ok ≥ ~95%** at low rate (today ~1.7%); ideally close
  to the 64-B figure (~98%).
- **No regression** in small-frame reliability or the gotcha-#10 on-wire stress.
- Decode cost re-measured; IRQ worst-case impact understood (see Finding 2 /
  §6 risks — the loop must stay cheap or move to PIO/2nd core).

## 3. Design principle

Replace open-loop striding with **closed-loop phase tracking** anchored to the
Manchester **mid-bit transition**, which is *guaranteed present every bit*
(every 100 ns). Re-aligning to it each bit (or filtering toward it) cancels
accumulated drift by construction. With 6× oversampling, edges are locatable to
±1 sample (±16.7 ns) — ample to keep the sample centered against ~tens-of-ns
drift accumulated over hundreds of µs.

Convention reminder (from the current decoder): `F` = first H→L = start of
HB[0]; data bit `k` value = level of the 2nd half-bit, sampled at `F + 4 + 6k`;
the mid-bit transition (HB[0]→HB[1]) is at `F + 3 + 6k`; a bit-boundary
transition at `F + 6(k+1)` appears only between equal-valued consecutive bits.
Clock recovery tracks **mid-bit** edges and must ignore/handle boundary edges.

## 4. Phase 0 — Offline bench & sample corpus  ✅ DONE (2026-05-27)

Built and committed in `tools/clock-recovery/`: temporary capture firmware
exfiltrated full-MTU run samples over UDP; collected a corpus of full-MTU
captures (`corpus/*.bin`, payload `i&0xFF`); `harness.py` runs a candidate
decoder and scores per-byte error-position bins + FCS-ok. The current open-loop
decoder reproduces offline exactly: **0% for ~575 B, ramp to ~82–89%, FCS-ok
0/N** — the validated baseline for Phase 1. Capture firmware reverted from
`main` (re-creation documented in the tools README). Original design below.

A decoder rewrite iterated on-device is slow (flash cycle per try) and
non-reproducible (the wire varies run-to-run). Capturing real samples once and
iterating offline makes it deterministic and fast — this is the key enabler.

- **Capture mode (temp firmware):** when a full-MTU known-pattern test frame is
  detected, copy its raw active-run samples (~9 KB) into a static buffer and
  exfiltrate over UDP in sequenced ≤512-B chunks (device→host TX of small frames
  is reliable — verify during capture). Host reassembles and saves with the
  known transmitted payload.
- **Corpus:** many full-MTU frames plus a few mid/small sizes, ideally across a
  couple of power cycles / warm-up states so the ppm offset and jitter vary
  (so a tuned loop generalizes, doesn't overfit one capture).
- **Offline harness (host):** run candidate decoders over the corpus and emit
  the same per-byte error-position profile + FCS-ok, with the **current**
  algorithm as the baseline (must reproduce the ~89% tail — sanity check).
  Iterate algorithms in seconds.
- **Deliverable:** reproducible corpus + harness reproducing the A1 tail, ready
  to score improvements. Carries over to *every* candidate (CPU or PIO).

## 5. Phase 1 — Algorithm design & offline validation

Develop on the corpus; pick by per-byte-error residual + cost.

**Candidate A — software DPLL (decision-directed phase tracking), CPU.**
State: fractional sample phase `pos` and period estimate `T ≈ 6.0`. Per bit:
(1) sample the bit at the recovered phase; (2) locate the mid-bit edge near its
expected time; (3) phase error = observed − expected; (4) correct `pos`
(1st-order) and optionally `T` (2nd-order — directly tracks the frequency/ppm
offset); (5) `pos += T`. Fixed-point (no FPU on Hazard3). Pros: robust, standard,
handles jitter+drift. Cons: per-bit cost (worsens Finding 2 if on CPU).

**Candidate B — edge-relative resampling, CPU (simpler).** Re-anchor each bit
(or every N bits) to the detected mid-bit edge and sample relative to it, with a
smoothed period. Cheaper than a full loop filter; possibly jitterier. Good
fallback if the DPLL is too costly.

**Candidate C — PIO-side clock recovery (streaming, hardware).** Reprogram the
RX PIO to `wait` for each mid-bit transition, skip the boundary edge by a timed
delay, and sample/emit bits (or pack bytes). Drift-immune by construction
(re-syncs every bit) **and offloads decode from the CPU — so it also addresses
A1 Finding 2 (single-core load collapse).** Cons: hard — PIO instruction limits,
the `out pc`/`.origin 0` gotcha (gotcha #1), and getting mid-vs-boundary edge
timing right. Develop by modelling the PIO state machine in software against the
corpus first, then writing the PIO program.

**Approach:** prove the *algorithm* cancels drift on the corpus with A or B
(fast), and quantify the residual error (this also reveals any secondary PHY/
baseline-wander component that clock recovery alone can't fix). In parallel, do a
**PIO feasibility spike** for C (it's the long-term win: correctness + CPU
offload in one). Decide CPU-vs-PIO for production from the offline numbers + the
spike. Note: A/B and C are different implementations of the same principle — the
corpus validates either, but a CPU design does *not* port mechanically to PIO.

## 6. Phase 2 — Firmware implementation

- Implement the chosen algorithm in `eth_rx.rs`, keeping the
  `find_active_run_from` / stitch / inbox / `verify_fcs` plumbing intact.
- Fixed-point throughout (no FPU). Keep per-bit cost bounded — re-measure
  decode cyc/byte and the IRQ worst case (the recent single-pass / stride /
  `get_unchecked` / decode-length-cap optimizations should be preserved or
  re-applied where compatible).
- If PIO (Candidate C): mind gotcha #1 (`.origin 0` for `out pc`), and redefine
  the capture/telemetry format (PIO emits bits/bytes, not raw samples).

## 7. Phase 3 — On-device validation (acceptance gate)

- Re-run the A1 **per-byte error-position test** → expect flat ~floor tail.
- **Full-MTU FCS-ok** at low + moderate rate → expect ≥ ~95%.
- **Multi-size echo** → large frames should now round-trip (today ~0%).
- **gotcha-#10 stress** (ping / UDP echo / host RX errs) → at/above baseline.
- **Re-measure decode cyc/byte + IRQ worst case** → quantify Finding 2 impact;
  decide if PIO/2nd-core is now forced.

## 8. Phase 4 — CPU-offload decision (if not already PIO)

If CPU clock-recovery decode is correct but too costly under load (Finding 2),
pursue the PIO clock-recovery decoder (Candidate C) and/or dedicate the 2nd
Hazard3 core to the NIC. The corpus + offline harness carry over.

## 9. Risks & open questions

- **No FPU** → fixed-point DPLL; watch phase-accumulator scaling/precision.
- **IRQ budget** — a CPU per-bit loop could push the 2.57 ms worst case up;
  may force PIO/2nd-core sooner (ties to Finding 2).
- **Edge robustness** — noise/jitter near transitions; 6× oversampling helps;
  mid-vs-boundary edge classification must be correct.
- **Residual PHY/baseline-wander** — A1's clean 0% for the first 575 B argues
  PHY noise is small, but a secondary baseline component could remain at full
  MTU; the corpus residual (after clock recovery) will reveal it. If present,
  that part is hardware (AC-coupling / transceiver), not firmware.
- **TX large frames** — we rely on device→host TX of ~512-B exfil frames being
  reliable (host PHY recovers our stable TX clock); confirm during capture.
- **Corpus coverage** — must span enough ppm/jitter (reboots, warm-up) so the
  tuned loop generalizes rather than overfitting one capture.

## 10. Validation metrics (carry through every phase)

1. Per-byte error-position bins (the A1 test) — **primary**.
2. Full-MTU FCS-ok % at low/medium rate.
3. Decode cyc/byte + IRQ worst case.
4. gotcha-#10 on-wire stress (ping% / UDP% / host RX errs).

## 11. First concrete step

Build the Phase 0 sample-capture firmware + corpus + offline harness, and
reproduce the current ~89% tail offline as the baseline to beat. Everything
else builds on that.
