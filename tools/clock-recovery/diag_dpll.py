#!/usr/bin/env python3
# Phase 3b — host-side analyzer for the DPLL on-wire failed-frame dump.
#
# The device (when built with --features dpll) captures every FCS-failed RX
# frame into a per-board buffer and dumps the latest one to UDP host:1235
# every 50 ms. This tool listens, accumulates dumps, and computes per-byte
# error-position bins versus the known counter payload (payload[j] = j & 0xFF)
# to distinguish PHY-limited (flat) from decoder-limited (ramp/cliff).
#
# Wire format (LE):
#   magic_u32 (0xDEFA17ED) | frame_id_u32 | orig_len_u16 | frame_data[<=1460]
#
# Run alongside a host that's blasting known-pattern full-MTU UDP at the device.
# Example: in one terminal,
#   python3 -c 'import socket,time; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("192.168.37.19", 0)); msg=bytes((i&0xff) for i in range(1472));
#       [time.sleep(0.05) or s.sendto(msg, ("192.168.37.24", 1234)) for _ in range(1000)]'
# in another, this script.

import socket
import struct
import sys
import time

MAGIC = 0xDEFA17ED
DEFAULT_DURATION = 60  # seconds

# Counter payload starts at frame offset 42 (14 eth + 20 ip + 8 udp).
# Score per-byte errors in 8 bins of width BIN_SIZE across the 1472-byte payload.
BIN_SIZE = 184
NUM_BINS = 8

def parse_dump(data: bytes):
    if len(data) < 10:
        return None
    magic, frame_id, orig_len = struct.unpack_from("<IIH", data, 0)
    if magic != MAGIC:
        return None
    body = data[10:10 + orig_len]
    return frame_id, orig_len, body

def score_frame(body: bytes, bins_err, bins_tot, byte_err_hist):
    # Each payload byte j (0-indexed) should be (j & 0xFF). Compare and bin.
    if len(body) < 42:
        return
    plen = min(len(body) - 42, 1472)
    for j in range(plen):
        b = min(j // BIN_SIZE, NUM_BINS - 1)
        bins_tot[b] += 1
        if body[42 + j] != (j & 0xFF):
            bins_err[b] += 1
            byte_err_hist[j] += 1

def print_report(seen, bins_err, bins_tot, byte_err_hist, max_pos):
    print(f"\n=== {seen} failed-frame dump(s) analyzed ===")
    for b in range(NUM_BINS):
        lo = 42 + b * BIN_SIZE
        r = 100.0 * bins_err[b] / bins_tot[b] if bins_tot[b] else 0.0
        print(f"  bin {b} frame-bytes {lo:>4}-{lo + BIN_SIZE - 1:<4} {r:6.1f}%  "
              f"{'#' * int(r / 2)}")
    print()
    # Failure-mode shape hint:
    if seen == 0:
        return
    nonzero = [b for b in range(NUM_BINS) if bins_tot[b] > 0]
    if not nonzero:
        return
    rates = [100.0 * bins_err[b] / bins_tot[b] if bins_tot[b] else 0.0
             for b in range(NUM_BINS)]
    span = max(rates) - min(rates)
    if span < 10:
        print("Shape: FLAT — residual looks PHY-limited (the goal-condition escape hatch).")
    elif rates == sorted(rates):
        print("Shape: RAMP (monotonic increase) — decoder clock drift, not corrected.")
    elif any(rates[i+1] - rates[i] > 30 for i in range(NUM_BINS - 1)):
        print("Shape: CLIFF — mid-frame slip / loss-of-lock cascade.")
    else:
        print("Shape: MIXED — multiple failure modes, look at byte-level histogram.")

    # Peek the first ~10 highest-error positions (only if there's a cliff/ramp).
    if span >= 10:
        srt = sorted(enumerate(byte_err_hist[:max_pos]),
                     key=lambda kv: -kv[1])[:10]
        print("Top 10 error positions (payload byte → error count):")
        for pos, cnt in srt:
            print(f"  byte {pos:>4} → {cnt}")

def main():
    duration = DEFAULT_DURATION
    if len(sys.argv) > 1:
        duration = int(sys.argv[1])

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 1235))
    s.settimeout(1.0)

    bins_err = [0] * NUM_BINS
    bins_tot = [0] * NUM_BINS
    byte_err_hist = [0] * 1472
    seen_ids = set()
    seen = 0
    t_end = time.time() + duration

    print(f"listening on :1235 for {duration}s — blast known-pattern full-MTU "
          f"UDP at the device while this runs.")
    while time.time() < t_end:
        try:
            data, _ = s.recvfrom(2048)
        except socket.timeout:
            continue
        parsed = parse_dump(data)
        if parsed is None:
            continue
        frame_id, orig_len, body = parsed
        if frame_id in seen_ids:
            continue
        seen_ids.add(frame_id)
        seen += 1
        if seen <= 5:
            print(f"  dump id={frame_id} len={orig_len} (body bytes captured: {len(body)})")
        score_frame(body, bins_err, bins_tot, byte_err_hist)

    print_report(seen, bins_err, bins_tot, byte_err_hist, max_pos=1472)
    return 0 if seen > 0 else 1

if __name__ == "__main__":
    sys.exit(main())
