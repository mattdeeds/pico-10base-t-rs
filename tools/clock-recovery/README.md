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

Perfect for ~575 bytes, then a monotonic ramp = clock drift. **Phase 1 goal:**
a clock-recovery decoder that drives all bins flat (~0%) and **FCS-ok to N/N.**

## Workflow

1. Develop a candidate `decode_X(buf) -> (F, frame_bytes)` in `harness.py`,
   set `DECODER = decode_X`, run `python3 harness.py`, iterate to flat/N-of-N.
2. Port the validated algorithm into `src/eth_rx.rs` (Phase 2) and re-run the
   on-device acceptance tests (Phase 3).

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
