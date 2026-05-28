# Clock-recovery offline bench (Phase 0)

Offline platform for developing the clock-recovery decoder without flashing the
device per iteration. See `docs/clock-recovery-decoder-plan.md` for the full
plan and A1 for why this is needed (open-loop `F+4+6k` sampling drifts off over
a long frame; full-MTU RX is ~1.7% FCS-ok).

## What's here

- `corpus/*.bin` — raw RX **sample** buffers (packed sample bits, LSB-first,
  8 samples/byte) of full-MTU frames captured off the device. Each is one
  active run (~9155 bytes) for a frame whose payload is `payload[i] = i & 0xFF`.
  These carry the *real* ppm drift + jitter, so a decoder tested against them
  faces exactly what the device faces.
- `harness.py` — runs a candidate decoder over the corpus and scores it:
  per-byte payload error rate binned by frame position (shows drift) + FCS-ok
  count (the acceptance metric). Ships with `decode_current`, a faithful model
  of the on-device open-loop decoder.
- `capture.py` — host side of corpus collection (needs the capture firmware).
- `pio_dump.py` — host side of the **Phase 2b live PIO-decoder bring-up** (see
  below). Validates the real PIO decoder's decoded-byte output off the wire, and
  has an offline `--selftest` that needs no hardware.

## Baseline (current open-loop decoder)

```
$ python3 harness.py
... FCS-ok 0/N ...
  bin 0 frame-bytes   42-225     0.0%
  bin 1 frame-bytes  226-409     0.0%
  bin 2 frame-bytes  410-593     0.0%
  bin 3 frame-bytes  594-777     ~0-3%
  bin 4 frame-bytes  778-961    ~14-24%
  bin 5 frame-bytes  962-1145   ~17-50%   <- ~50% = sample point on a bit boundary
  bin 6 frame-bytes 1146-1329   ~35-74%
  bin 7 frame-bytes 1330-1513   ~82-89%
```

Perfect for ~575 bytes, then a monotonic ramp = clock drift.

## Phase 1 result — clock recovery (DONE)

`decode_edge_track` re-anchors to each per-bit Manchester transition (recurs
~6 samples apart at `F+5+6m`) and samples one sample before it (= `F+4+6m`), so
drift can't accumulate. On the corpus it drives **every bin to 0% and FCS-ok to
N/N** (robust across window W=1–3) — see `harness.py` output. This is the
algorithm to take to firmware (Phase 2); the per-bit edge search makes a CPU
port borderline vs the IRQ budget, so PIO-side recovery is the likely
production path (see `docs/clock-recovery-decoder-plan.md`).

## Workflow

1. Develop a candidate `decode_X(buf) -> (F, frame_bytes)` in `harness.py`,
   set `DECODER = decode_X`, run `python3 harness.py`, iterate to flat/N-of-N.
2. Port the validated algorithm into `src/eth_rx.rs` (Phase 2) and re-run the
   on-device acceptance tests (Phase 3).

## Phase 2b — live PIO decoder bring-up (`pio_dump.py`)

`src/eth_rx_pio.rs` is the production decoder: a PIO program (PIO1 SM0) that
re-syncs to every Manchester edge in hardware (the `decode_pio_model` algorithm,
SM @ 150 MHz, `[8]`-cycle boundary-skip). The TEMP scaffolding in `main.rs`
(Phase 2b) runs it **in parallel** with the working RX so the device stays
functional, drains its decoded-byte FIFO into a 2048-byte window, and dumps that
window over UDP broadcast `:1234` in 512-byte chunks
(`dec_id|seq|cap_len | data[512]`).

The decoded bytes ARE the decoded bitstream, LSB-first per byte (PIO `in`
shift-right + autopush(32) + `to_le_bytes`) — the exact bit order
`harness.sample_bit` uses — so `pio_dump.py` reassembles a window, finds the SFD,
extracts the frame, checks FCS, and scores the same per-byte error bins.

```
# Offline first — proves the host pipeline (model -> pack -> reassemble -> FCS)
# end-to-end without flashing. Expect FCS-ok N/N, flat bins, "self-test PASS".
$ python3 pio_dump.py --selftest

# On-wire (after flashing the Phase 2b firmware; SWD-flash the first time per
# docs/pio-decoder-plan.md §8). Blasts known-pattern full-MTU frames at the
# device and validates the dumped windows.
$ python3 pio_dump.py
#   dec_id 0: frame 1518B  sfd@bit 61  inv=False  FCS OK
#   ... FCS-ok N/N, flat bins  => PIO decoder works, D=8 is in the window.
```

Reading the result: **flat bins + FCS-ok** = the PIO decoder produces correct
full-MTU bytes and the `[8]` skip delay lands in the working window. A **drift
ramp** (like the open-loop baseline) = the skip delay is slightly off (resample
straying toward a bit boundary) — nudge the `[8]` in `eth_rx_pio.rs`. **No SFD /
garbage** = polarity or gross timing wrong. The valid `[n]` range is ~6–12
cycles (resample at `n+2`, between the boundary edge ~7.5 and the next mid-bit
~15); `8` targets the centre.

## Re-collecting / expanding the corpus

The corpus here is enough to start Phase 1, but the plan calls for more ppm/
jitter variation (capture across reboots / warm-up). To re-capture you need the
**temporary capture firmware** (kept out of `main` to keep it shippable):

- `src/eth_mac.rs` — add to `EthRxShared`: `cap_buf: [u8; CAP_MAX]`, `cap_len`,
  `cap_id`, `cap_armed` (`CAP_MAX = 10240`). In `process_completed_half`, after
  the `mac_accept` check, when `cap_armed && dst == our_mac && len > 8000`, copy
  `bytes[off..off+len]` into `cap_buf`, set `cap_len`, bump `cap_id`, clear
  `cap_armed`. Add `arm_capture()`, `cap_status() -> (id, len)`,
  `cap_copy(off, &mut [u8]) -> usize`.
- `src/main.rs` — every 20 ms send one chunk over `send_udp_broadcast` with the
  header `cap_id|cap_len|n_chunks|chunk_idx` + up to 512 data bytes, cycling
  `chunk_idx`; re-arm every ~200 chunks (~4 s); `arm_capture()` once at startup.

Then `python3 capture.py` (sends test frames + reassembles chunks into
`corpus/`). Revert the firmware before shipping.
