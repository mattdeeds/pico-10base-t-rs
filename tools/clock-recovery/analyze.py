#!/usr/bin/env python3
# Offline analysis of saved live PIO-decoder windows (dumps/win_*.bin).
# For each window: try every SFD candidate, extract the frame, and measure how
# many leading payload bytes match the known pattern (payload[j] = j & 0xFF,
# starting at frame offset 42). Reports the best-aligned candidate + where the
# decode first diverges — distinguishing drift (late divergence, ramp) from a
# bit-slip / boundary mishandling (early, abrupt divergence).
import sys, os, glob
sys.path.insert(0, os.path.dirname(__file__))
from harness import sample_bit

def bits_after(buf, sfd):
    nbits = len(buf) * 8
    start = sfd + 1
    frame = bytearray(); i = 0
    while start + i * 8 + 8 <= nbits:
        b = 0
        for j in range(8):
            b |= sample_bit(buf, start + i * 8 + j) << j
        frame.append(b); i += 1
    return bytes(frame)

def sfd_candidates(buf):
    nbits = len(buf) * 8
    prev = sample_bit(buf, 0)
    for k in range(1, nbits):
        cur = sample_bit(buf, k)
        if cur == 1 and prev == 1:
            yield k
        prev = cur

def payload_match_len(frame):
    """# of leading payload bytes (offset 42+) matching j&0xFF before first mismatch."""
    n = 0
    while 42 + n < len(frame) and frame[42 + n] == (n & 0xFF):
        n += 1
    return n

def analyze(fn):
    buf = open(fn, "rb").read()
    best = None
    for inv in (False, True):
        b = bytes(x ^ 0xFF for x in buf) if inv else buf
        for sfd in sfd_candidates(b):
            f = bits_after(b, sfd)
            if len(f) < 60:
                continue
            ml = payload_match_len(f)
            if best is None or ml > best[0]:
                best = (ml, sfd, inv, f)
    print("\n=== {} ({} bytes) ===".format(os.path.basename(fn), len(buf)))
    if best is None:
        print("  no candidate"); return
    ml, sfd, inv, f = best
    print("  best: sfd@bit {}  inv={}  payload-match {} bytes (then diverges)".format(sfd, inv, ml))
    print("  header (post-SFD), first 42 bytes:")
    print("   ", " ".join("{:02x}".format(x) for x in f[:42]))
    print("  payload region offset 42, expected j&0xFF — first 32 bytes:")
    print("    got: ", " ".join("{:02x}".format(x) for x in f[42:42 + 32]))
    print("    exp: ", " ".join("{:02x}".format(j & 0xFF) for j in range(32)))
    # Per-32-byte error rate across the whole payload, to see drift vs uniform.
    plen = max(0, len(f) - 42)
    print("  payload byte-error rate per 64-byte block:")
    line = "    "
    for blk in range(0, min(plen, 1536), 64):
        errs = sum(1 for j in range(blk, min(blk + 64, plen)) if f[42 + j] != (j & 0xFF))
        tot = min(blk + 64, plen) - blk
        line += "{:3.0f} ".format(100.0 * errs / tot if tot else 0)
    print(line)
    # Context around the first divergence: is it a 1-bit slip (got[k] looks like
    # exp shifted by a bit) or random? Show 8 bytes before/after.
    if 42 + ml + 8 <= len(f):
        lo = max(0, ml - 8)
        print("  around first divergence at payload byte {} (frame bit {}):".format(
            ml, sfd + 1 + (42 + ml) * 8))
        print("    got: ", " ".join("{:02x}".format(f[42 + j]) for j in range(lo, ml + 8)))
        print("    exp: ", " ".join("{:02x}".format(j & 0xFF) for j in range(lo, ml + 8)))

if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "dumps", "*.bin")))
    if not files:
        print("no dumps/*.bin — run pio_dump.py --save first"); sys.exit(1)
    for fn in files:
        analyze(fn)
