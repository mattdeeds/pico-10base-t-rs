#!/usr/bin/env python3
# Decode-fix experiment (docs/rx-bulk-ceiling.md): the §9d analysis found the
# full-MTU residual is PHY noise (~flat, ~5.8e-5/bit single-sample errors). The
# current edge-track decoder makes a SINGLE-SAMPLE bit decision (tr-1). Does a
# MATCHED-FILTER decision (compare the integrated levels of the bit's two
# half-bits, using all 6 oversamples) survive more per-sample noise?
#
# Method: inject independent per-sample bit-flips at rate p into the corpus,
# decode with each candidate (same edge-track TIMING; only the data DECISION
# differs), score FCS-ok over many trials. Upper-bound on the matched-filter
# benefit (real PHY noise may be correlated within a bit; this models it as iid).
#
# RESULT (2026-06-03): the matched filter is NOT a win. At the relevant operating
# point (p~1-3e-4, where edge-track sits near the on-device ~30-70% full-MTU) it
# is no better than the single sample (p=3e-4: edge 33% vs MF 31%), and it even
# fails on CLEAN signal for some frames (66% vs 100%) because half-bit integration
# needs precise half-bit PHASE that varies frame-to-frame, whereas the single
# sample tr-1 sits robustly at the half-bit centre. Corroborates cpu-dpll-plan.md
# §9d: the residual is PHY noise, not something the decision can fix. Firmware
# decode is near its floor; the durable fix is hardware (a real Ethernet PHY).
import glob, os, random
from harness import find_F, find_SFD, find_edge, sample_bit, _pack, fcs_ok, decode_edge_track, CORPUS

def flip(buf, p, rng):
    ns = len(buf) * 8
    out = bytearray(buf)
    for off in range(ns):
        if rng.random() < p:
            out[off >> 3] ^= (1 << (off & 7))
    return bytes(out)

def decode_matched(buf, W=1):
    # Same edge-track timing as decode_edge_track, but decide each bit by
    # comparing the integrated level of the data half-bit {tr-2,tr-1,tr} vs the
    # other half-bit {tr-5,tr-4,tr-3} — an integrate-and-dump matched filter.
    # Differential (baseline-robust) + averages 3 samples/half (noise-robust).
    ns = len(buf) * 8
    F = find_F(buf, ns)
    if F is None: return None, None
    sfd = find_SFD(buf, ns, F)
    if sfd is None: return F, None
    start = sfd + 1
    P = 6
    tr = find_edge(buf, F + 5 + 6 * start, W, ns) or (F + 5 + 6 * start)
    bits = []
    while True:
        # Data sample is tr-1. Compare the two samples just inside the data half
        # {tr-2,tr-1} vs the other half {tr+1,tr+2}, avoiding the edge sample tr
        # and the previous-bit-adjacent tr-3. Tie → fall back to the single
        # sample tr-1 (so on clean signal this degrades to edge-track exactly).
        if tr - 2 < 0 or tr + 2 >= ns: break
        data_h = sample_bit(buf, tr - 2) + sample_bit(buf, tr - 1)
        other_h = sample_bit(buf, tr + 1) + sample_bit(buf, tr + 2)
        if data_h != other_h:
            bits.append(1 if data_h > other_h else 0)
        else:
            bits.append(sample_bit(buf, tr - 1))
        nxt = find_edge(buf, tr + P, W, ns)
        tr = nxt if nxt is not None else tr + P
    return F, _pack(bits)

def fcs_rate(dec, p, trials, seed):
    rng = random.Random(seed)
    ok = tot = 0
    files = sorted(glob.glob(os.path.join(CORPUS, "*.bin")))
    for fn in files:
        base = open(fn, "rb").read()
        for _ in range(trials):
            nb = flip(base, p, rng) if p > 0 else base
            _, frame = dec(nb)
            tot += 1
            if frame is not None and fcs_ok(frame):
                ok += 1
    return 100.0 * ok / tot if tot else 0.0

if __name__ == "__main__":
    TRIALS = 60
    print(f"FCS-ok %% over {TRIALS} noisy trials x 3 corpus frames, per per-sample flip rate p")
    print(f"{'p':>8} | {'edge-track (1-sample)':>22} | {'matched (6-sample MF)':>22}")
    print("-" * 60)
    for p in (0.0, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 2e-2, 3e-2):
        e = fcs_rate(decode_edge_track, p, TRIALS, seed=1234)
        m = fcs_rate(decode_matched, p, TRIALS, seed=1234)
        print(f"{p:>8.4f} | {e:>21.1f}% | {m:>21.1f}%")
