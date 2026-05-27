#!/usr/bin/env python3
# Offline decoder harness for the clock-recovery work (Phase 0/1).
#
# Runs decoder candidates over the captured raw-sample corpus (corpus/*.bin,
# each = the raw active-run samples of one full-MTU frame, payload[i]=i&0xFF)
# and scores each two ways: per-byte payload error rate binned by frame
# position (shows drift), and FCS-ok count (the acceptance metric).
#
# Candidates:
#   decode_current     - faithful model of the on-device OPEN-LOOP decoder
#                        (sample data bit k at F+4+6k). Reproduces the measured
#                        ~0%->ramp->~82-89% drift tail, FCS-ok 0/N. Baseline.
#   decode_edge_track  - CLOCK RECOVERY (Phase 1, validated): re-anchor to each
#                        per-bit Manchester transition (recurs ~6 samples apart
#                        at F+5+6m) and sample one sample before it (= F+4+6m),
#                        so drift can't accumulate. -> flat bins, FCS-ok N/N.
import glob, os
CORPUS = os.path.join(os.path.dirname(__file__), "corpus")

def sample_bit(buf, off):
    return (buf[off >> 3] >> (off & 7)) & 1

def find_F(buf, ns):
    prev = sample_bit(buf, 0)
    for i in range(1, ns):
        s = sample_bit(buf, i)
        if prev == 1 and s == 0: return i
        prev = s
    return None

def find_SFD(buf, ns, F):
    def d(k):
        idx = F + 4 + 6 * k
        return sample_bit(buf, idx) if idx < ns else None
    prev = d(0)
    for k in range(1, 1600):
        c = d(k)
        if c is None: return None
        if c == 1 and prev == 1: return k
        prev = c
    return None

def _pack(bits):
    f = bytearray()
    for i in range(len(bits) // 8):
        b = 0
        for j in range(8): b |= bits[i * 8 + j] << j
        f.append(b)
    return bytes(f)

def decode_current(buf):
    ns = len(buf) * 8
    F = find_F(buf, ns)
    if F is None: return None, None
    sfd = find_SFD(buf, ns, F)
    if sfd is None: return F, None
    start = sfd + 1
    bits, k = [], 0
    while True:
        idx = F + 4 + 6 * (start + k)
        if idx >= ns or k >= 1600 * 8: break
        bits.append(sample_bit(buf, idx)); k += 1
    return F, _pack(bits)

def find_edge(buf, center, W, ns):
    lo = max(1, center - W); hi = min(ns - 1, center + W)
    best, bestd = None, None
    for i in range(lo, hi + 1):
        if sample_bit(buf, i) != sample_bit(buf, i - 1):
            dd = abs(i - center)
            if bestd is None or dd < bestd: best, bestd = i, dd
    return best

def decode_edge_track(buf, W=1):
    ns = len(buf) * 8
    F = find_F(buf, ns)
    if F is None: return None, None
    sfd = find_SFD(buf, ns, F)
    if sfd is None: return F, None
    start = sfd + 1
    P = 6
    # Per-bit transition recurs ~6 samples apart at F+5+6m; the data bit is the
    # sample just before it (= F+4+6m). Re-anchor each bit -> no drift.
    tr = find_edge(buf, F + 5 + 6 * start, W, ns) or (F + 5 + 6 * start)
    bits = []
    while True:
        si = tr - 1
        if si < 0 or si >= ns: break
        bits.append(sample_bit(buf, si))
        nxt = find_edge(buf, tr + P, W, ns)
        tr = nxt if nxt is not None else tr + P  # coast through a missed edge
    return F, _pack(bits)

# --- CRC-32 / IEEE-802.3 for FCS-ok ---
_T = []
for _n in range(256):
    _c = _n
    for _ in range(8): _c = (_c >> 1) ^ 0xEDB88320 if _c & 1 else _c >> 1
    _T.append(_c)
def crc32(d):
    c = 0xFFFFFFFF
    for x in d: c = (c >> 8) ^ _T[(c ^ x) & 0xFF]
    return c ^ 0xFFFFFFFF
def fcs_ok(f):
    if len(f) < 18: return False
    fl = max(14 + (f[16] << 8 | f[17]) + 4, 64) if (f[12] << 8 | f[13]) == 0x0800 else len(f)
    if fl < 18 or fl > len(f): return False
    return crc32(f[:fl - 4]) == int.from_bytes(f[fl - 4:fl], "little")

def score(name, dec):
    NB, BINSZ = 8, 184
    be = [0] * NB; bt = [0] * NB; nf = 0; nok = 0
    for fn in sorted(glob.glob(os.path.join(CORPUS, "*.bin"))):
        buf = open(fn, "rb").read()
        _, frame = dec(buf)
        if frame is None: continue
        nf += 1; nok += 1 if fcs_ok(frame) else 0
        plen = max(0, len(frame) - 42 - 4)
        for j in range(min(plen, 1472)):
            b = min(j // BINSZ, NB - 1); bt[b] += 1
            if frame[42 + j] != (j & 0xFF): be[b] += 1
    print("\n=== {} ===  FCS-ok {}/{}".format(name, nok, nf))
    for b in range(NB):
        lo = 42 + b * BINSZ
        r = 100.0 * be[b] / bt[b] if bt[b] else 0
        print("  bin {} frame-bytes {:>4}-{:<4} {:6.1f}%  {}".format(b, lo, lo + BINSZ - 1, r, "#" * int(r / 2)))

if __name__ == "__main__":
    score("current (open-loop) — baseline", decode_current)
    score("edge-track (clock recovery)", decode_edge_track)
