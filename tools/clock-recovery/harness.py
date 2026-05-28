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

def decode_pio_interval_model(buf, thresh=5):
    # Phase 2c: inter-edge-INTERVAL classifier (more robust than the fixed-delay
    # decode_pio_model). Instead of blindly skipping ~0.67 bit past every edge,
    # MEASURE the samples between edges. After a mid-bit edge, the next edge is
    # either a boundary edge at ~T/2 (3 samples @60MHz) or the next mid-bit at
    # ~T (6 samples). Classify with a threshold between them; emit a data bit
    # (= the post-mid-bit level) at each mid-bit edge. Drift-immune (re-anchors
    # to every edge) AND jitter-robust (discriminates two well-separated interval
    # classes instead of threading a blind delay). T=6, T/2=3 @60MHz; thresh~4-5.
    # NB: on-wire (2026-05-27) this did NOT beat the fixed-delay [8] decoder —
    # slips persist inside runs of identical bits (uniform-T/2 square wave, where
    # interval gives no discrimination). Next direction is a DPLL (loop-filtered
    # phase tracking); see the pio-decoder-phase2b-onwire memory.
    ns = len(buf) * 8

    def edge_after(p, lvl):
        # first sample index > p whose level differs from lvl (the next edge),
        # or None past end.
        q = p + 1
        while q < ns and sample_bit(buf, q) == lvl:
            q += 1
        return q if q < ns else None

    # State: at_mid = True means the last edge crossed was a mid-bit edge, so
    # the NEXT edge is classified (mid-bit at ~T, or boundary at ~T/2). When the
    # last edge was a boundary, the next edge is unconditionally the mid-bit.
    # Emit the PRE-edge level (the level before crossing the mid-bit), matching
    # decode_pio_model / the on-wire PIO. Treat the first edge as a mid-bit.
    pos = 0
    y = sample_bit(buf, 0)
    bits = []
    at_mid = True
    while True:
        q = edge_after(pos, y)
        if q is None:
            break
        if at_mid:
            if q - pos >= thresh:        # ~T -> this edge is the next mid-bit
                bits.append(y)           # emit pre-edge level
            else:                        # ~T/2 -> boundary edge; don't emit
                at_mid = False
        else:                            # boundary just crossed -> this is mid-bit
            bits.append(y)
            at_mid = True
        y = sample_bit(buf, q)
        pos = q
    # SFD-find + byte extract, identical to decode_pio_model.
    sfd = None
    for k in range(1, len(bits)):
        if bits[k] == 1 and bits[k - 1] == 1: sfd = k; break
    if sfd is None: return None, None
    start = sfd + 1
    frame = bytearray(); i = 0
    while start + i * 8 + 8 <= len(bits):
        b = 0
        for j in range(8): b |= bits[start + i * 8 + j] << j
        frame.append(b); i += 1
    return None, bytes(frame)

def decode_dpll_model(buf, N=6, samp=4, win=1, invert=False):
    # Phase 2d: windowed absolute-phase DPLL (the route after the interval
    # classifier's run-internal slips). A free-running bit-clock PHASE (NCO,
    # period N samples) samples the data at a fixed 2nd-half phase `samp`, and
    # RESYNCS to Manchester mid-bit edges that fall within +/-win of the mid-bit
    # phase (N//2) — edges outside the window (boundary edges, noise) are
    # IGNORED. No per-edge alternation state ⇒ a single bad edge can't cascade
    # (bounded phase bump or ignored); the boundary/mid-bit split is by PHASE,
    # not interval (which failed inside identical-bit runs). 60MHz corpus: N=6,
    # mid=3, samp~4-5, win=1. Maps to N=15 @150MHz PIO.
    ns = len(buf) * 8
    mid = N // 2
    bits = []
    prev = sample_bit(buf, 0)
    phase = 0
    sampled = False
    for t in range(1, ns):
        phase += 1
        s = sample_bit(buf, t)
        edge = (s != prev); prev = s
        if edge and abs(phase - mid) <= win:   # a mid-bit edge -> resync phase
            phase = mid
        if phase == samp and not sampled:        # sample data (2nd-half level)
            bits.append(s ^ (1 if invert else 0))
            sampled = True
        if phase >= N:                           # bit boundary -> wrap
            phase -= N
            sampled = False
    # SFD-find + byte extract, identical to decode_pio_model.
    sfd = None
    for k in range(1, len(bits)):
        if bits[k] == 1 and bits[k - 1] == 1: sfd = k; break
    if sfd is None: return None, None
    start = sfd + 1
    frame = bytearray(); i = 0
    while start + i * 8 + 8 <= len(bits):
        b = 0
        for j in range(8): b |= bits[start + i * 8 + j] << j
        frame.append(b); i += 1
    return None, bytes(frame)

def decode_pio_model(buf, D=4):
    # Phase 2a: software model of the PIO decoder (streaming, no F/SFD
    # pre-align). Poll sample-by-sample for a level change (edge); emit the
    # pre-edge level as the bit; skip D samples past the boundary edge;
    # resample. Then CPU-side: find SFD in the emitted bitstream + extract.
    # D=4 at 60 MHz (6 samples/bit); maps to ~10-11 cycles at a 150 MHz SM.
    ns = len(buf) * 8
    pos = 0; y = sample_bit(buf, 0); bits = []
    while True:
        pos += 1
        if pos >= ns: break
        if sample_bit(buf, pos) != y:
            bits.append(y)
            pos += D
            if pos >= ns: break
            y = sample_bit(buf, pos)
    sfd = None
    for k in range(1, len(bits)):
        if bits[k] == 1 and bits[k - 1] == 1: sfd = k; break
    if sfd is None: return None, None
    start = sfd + 1
    frame = bytearray(); i = 0
    while start + i * 8 + 8 <= len(bits):
        b = 0
        for j in range(8): b |= bits[start + i * 8 + j] << j
        frame.append(b); i += 1
    return None, bytes(frame)

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
    score("pio-model (streaming, D=4)", decode_pio_model)
    for t in (4, 5):
        score("pio-interval (classifier, thresh={})".format(t),
               lambda buf, t=t: decode_pio_interval_model(buf, thresh=t))
    score("dpll (windowed phase, N=6 samp=4 win=1)",
          lambda buf: decode_dpll_model(buf, N=6, samp=4, win=1))
