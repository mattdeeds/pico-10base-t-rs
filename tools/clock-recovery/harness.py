#!/usr/bin/env python3
# Offline decoder harness for the clock-recovery work (Phase 0/1).
#
# Runs a candidate decoder over the captured raw-sample corpus (corpus/*.bin,
# each = the raw active-run samples of one full-MTU frame whose payload is
# payload[i] = i & 0xFF) and scores it two ways:
#   - per-byte payload error rate binned by frame position (shows drift), and
#   - FCS-ok count (the acceptance metric).
#
# `decode_current` is a faithful model of the on-device open-loop decoder
# (sample data bit k at F + 4 + 6k). It reproduces the measured ~0%->ramp->~82%
# tail — the baseline a clock-recovery decoder must beat (flat bins, FCS 6/6).
#
# Add clock-recovery candidates as new decode_* functions and point DECODER at
# them; the scoring stays the same.
import glob, os

CORPUS = os.path.join(os.path.dirname(__file__), "corpus")

def sample_bit(buf, off):
    return (buf[off >> 3] >> (off & 7)) & 1

def decode_current(buf):
    """Open-loop F+4+6k decoder (current firmware). Returns (F, frame_bytes)."""
    ns = len(buf) * 8
    prev = sample_bit(buf, 0); F = None
    for i in range(1, ns):
        s = sample_bit(buf, i)
        if prev == 1 and s == 0: F = i; break
        prev = s
    if F is None: return None, None
    def dbit(k):
        idx = F + 4 + 6 * k
        return sample_bit(buf, idx) if idx < ns else None
    prev = dbit(0); sfd = None
    for k in range(1, 1600):
        c = dbit(k)
        if c is None: break
        if c == 1 and prev == 1: sfd = k; break
        prev = c
    if sfd is None: return F, None
    start = sfd + 1
    frame = bytearray(); i = 0
    while i < 1600:
        byte = 0; done = False
        for j in range(8):
            b = dbit(start + i * 8 + j)
            if b is None: done = True; break
            byte |= b << j
        if done: break
        frame.append(byte); i += 1
    return F, bytes(frame)

DECODER = decode_current

# --- CRC-32 / IEEE-802.3 for the FCS-ok acceptance metric ---
_CRC_T = []
for _n in range(256):
    _c = _n
    for _ in range(8):
        _c = (_c >> 1) ^ 0xEDB88320 if _c & 1 else _c >> 1
    _CRC_T.append(_c)
def crc32(d):
    c = 0xFFFFFFFF
    for x in d:
        c = (c >> 8) ^ _CRC_T[(c ^ x) & 0xFF]
    return c ^ 0xFFFFFFFF
def derive_len(f):
    if len(f) < 18: return len(f)
    if (f[12] << 8 | f[13]) == 0x0800:
        c = max(14 + (f[16] << 8 | f[17]) + 4, 64)
        return c if c <= len(f) else len(f)
    return len(f)
def fcs_ok(f):
    fl = derive_len(f)
    if fl < 18 or fl > len(f): return False
    return crc32(f[:fl - 4]) == int.from_bytes(f[fl - 4:fl], "little")

def main():
    NB, BINSZ = 8, 184
    be = [0] * NB; bt = [0] * NB; nf = 0; nok = 0
    files = sorted(glob.glob(os.path.join(CORPUS, "*.bin")))
    if not files:
        print("no corpus files in", CORPUS); return
    for fn in files:
        buf = open(fn, "rb").read()
        F, frame = DECODER(buf)
        if frame is None:
            print("{}: decode failed".format(os.path.basename(fn))); continue
        nf += 1
        ok = fcs_ok(frame); nok += 1 if ok else 0
        plen = max(0, len(frame) - 42 - 4)
        for j in range(min(plen, 1472)):
            b = min(j // BINSZ, NB - 1); bt[b] += 1
            if frame[42 + j] != (j & 0xFF): be[b] += 1
        print("{}: F={} decoded {}B  FCS {}".format(os.path.basename(fn), F, len(frame), "OK" if ok else "FAIL"))
    print("\n{} frames, FCS-ok {}/{}; per-bin payload byte-error rate ({}):".format(nf, nok, nf, DECODER.__name__))
    for b in range(NB):
        lo = 42 + b * BINSZ
        r = 100.0 * be[b] / bt[b] if bt[b] else 0
        print("  bin {} frame-bytes {:>4}-{:<4} {:6.1f}%  {}".format(b, lo, lo + BINSZ - 1, r, "#" * int(r / 2)))

if __name__ == "__main__":
    main()
