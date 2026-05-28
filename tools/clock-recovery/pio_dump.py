#!/usr/bin/env python3
# Phase 2b host validator for the PIO clock-recovery decoder (eth_rx_pio.rs).
#
# The bring-up firmware (main.rs, TEMP Phase 2b) runs the PIO decoder on PIO1
# SM0 in parallel with the working RX, drains its decoded-byte FIFO into a
# 2048-byte window, and dumps that window over UDP broadcast :1234 in 512-byte
# chunks. This tool blasts known-pattern full-MTU frames at the device (so the
# PIO keeps decoding), reassembles each dumped window, finds the SFD in the
# decoded bitstream, extracts the frame, checks FCS, and scores the per-byte
# error position bins (the A1 drift signature) — the Phase 2d acceptance gate,
# but here just to confirm the live PIO decoder produces correct bytes.
#
# Dump chunk wire format (must match main.rs):
#   dec_id(u32 LE) | seq(u32 LE) | cap_len(u32 LE) | data[512]
# n_chunks = cap_len // 512; reassemble seq 0..n-1 into cap_len decoded bytes.
#
# The decoded bytes ARE the decoded bitstream, LSB-first per byte (PIO `in`
# with shift-right + autopush(32) + to_le_bytes), i.e. the exact bit order
# harness.sample_bit expects — so we reuse harness's SFD/FCS logic directly.
#
# Offline self-test (no hardware):  python3 pio_dump.py --selftest
#   Runs the validated PIO model over corpus/*.bin, packs the emitted bits into
#   the same LSB-first bytes the device would dump, then runs THIS tool's
#   reassembly/SFD/FCS path — proving the host pipeline end-to-end.
import socket, struct, threading, time, os, sys, glob

sys.path.insert(0, os.path.dirname(__file__))
from harness import sample_bit, crc32, fcs_ok, CORPUS

DEV = "192.168.37.24"; HOST = "192.168.37.19"
DEAD_PORT = 48879   # 0xBEEF — pure RX, no echo-back to perturb the wire
N_WANT = 8          # report after this many decodable windows (or timeout)

# --- decoded bitstream -> frame (reuses harness sample_bit bit order) ---

def extract_from_sfd(buf, nbits, sfd):
    """Pack bytes (LSB-first) from the bit AFTER the SFD's trailing '11'."""
    start = sfd + 1
    frame = bytearray(); i = 0
    while start + i * 8 + 8 <= nbits:
        b = 0
        for j in range(8):
            b |= sample_bit(buf, start + i * 8 + j) << j
        frame.append(b); i += 1
    return bytes(frame)

def candidate_sfds(buf, nbits):
    """Every position k where decoded bits k-1,k are both 1 (end of 0xD5 SFD,
    LSB-first). The preamble alternates 1010..., so on a clean stream the first
    such k is the real SFD; on live junk we try each and keep the FCS pass."""
    prev = sample_bit(buf, 0)
    for k in range(1, nbits):
        cur = sample_bit(buf, k)
        if cur == 1 and prev == 1:
            yield k
        prev = cur

def best_frame(decoded):
    """Try every SFD candidate (and inverted polarity); return the first frame
    that passes FCS as (frame, inverted, sfd_bit), else the longest candidate
    frame found (for drift diagnostics) with fcs=False."""
    nbits = len(decoded) * 8
    fallback = None
    for inv in (False, True):
        buf = bytes(b ^ 0xFF for b in decoded) if inv else decoded
        for sfd in candidate_sfds(buf, nbits):
            f = extract_from_sfd(buf, nbits, sfd)
            if len(f) < 64:
                continue
            if fcs_ok(f):
                return f, inv, sfd, True
            if fallback is None or len(f) > len(fallback[0]):
                fallback = (f, inv, sfd)
    if fallback:
        return fallback[0], fallback[1], fallback[2], False
    return None, False, None, False

# --- per-byte error scoring (the A1 drift bins) ---
NB, BINSZ = 8, 184

def score_frame(frame, be, bt):
    """Bin payload byte-errors vs the known pattern (payload[j] = j & 0xFF).
    Payload starts at offset 42 (14 eth + 20 ip + 8 udp) and runs for
    (IP total length - 28) bytes — bound to that so we don't score post-frame
    junk (the extracted window extends past the real frame)."""
    ip_total = (frame[16] << 8) | frame[17] if len(frame) >= 18 else 0
    plen = max(0, ip_total - 28)
    for j in range(min(plen, 1472)):
        b = min(j // BINSZ, NB - 1); bt[b] += 1
        if frame[42 + j] != (j & 0xFF):
            be[b] += 1

def print_bins(be, bt, nok, nf):
    print("\n=== live PIO decoder ===  FCS-ok {}/{}".format(nok, nf))
    for b in range(NB):
        lo = 42 + b * BINSZ
        r = 100.0 * be[b] / bt[b] if bt[b] else 0.0
        print("  bin {} frame-bytes {:>4}-{:<4} {:6.1f}%  {}".format(
            b, lo, lo + BINSZ - 1, r, "#" * int(r / 2)))

# --- offline self-test: model -> pack -> parse, no hardware ---

def pio_bitstream(buf, D=4):
    """Just the emitted bits of the validated decode_pio_model (no SFD/extract),
    so we can pack them exactly as the device's FIFO -> dump would."""
    ns = len(buf) * 8
    pos = 0; y = sample_bit(buf, 0); bits = []
    while True:
        pos += 1
        if pos >= ns: break
        if sample_bit(buf, pos) != y:
            bits.append(y); pos += D
            if pos >= ns: break
            y = sample_bit(buf, pos)
    return bits

def pack_lsb_first(bits):
    out = bytearray((len(bits) + 7) // 8)
    for i, b in enumerate(bits):
        if b: out[i >> 3] |= 1 << (i & 7)
    return bytes(out)

def selftest():
    files = sorted(glob.glob(os.path.join(CORPUS, "*.bin")))
    if not files:
        print("no corpus in {} — run capture.py first".format(CORPUS)); return 1
    be = [0] * NB; bt = [0] * NB; nok = 0; nf = 0
    for fn in files:
        raw = open(fn, "rb").read()
        decoded = pack_lsb_first(pio_bitstream(raw, D=4))  # simulate device dump
        frame, inv, sfd, ok = best_frame(decoded)
        if frame is None:
            print("  {}: NO SFD".format(os.path.basename(fn))); continue
        nf += 1; nok += 1 if ok else 0
        print("  {}: frame {}B  sfd@bit {}  inv={}  FCS {}".format(
            os.path.basename(fn), len(frame), sfd, inv, "OK" if ok else "FAIL"))
        if ok: score_frame(frame, be, bt)
    print_bins(be, bt, nok, nf)
    ok_all = (nok == nf and nf > 0 and all(be[b] == 0 for b in range(NB)))
    print("\nself-test {}".format("PASS" if ok_all else "FAIL"))
    return 0 if ok_all else 1

# --- live capture from the device dump ---

def live():
    stop = threading.Event()
    psize = 1472
    for i, a in enumerate(sys.argv):
        if a == "--size" and i + 1 < len(sys.argv):
            psize = int(sys.argv[i + 1])
    msg = bytes((i & 0xFF) for i in range(psize))
    def blast():
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); tx.bind((HOST, 0))
        while not stop.is_set():
            try: tx.sendto(msg, (DEV, DEAD_PORT))
            except OSError: pass
            time.sleep(0.008)
    threading.Thread(target=blast, daemon=True).start()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 1234)); s.settimeout(1.0)

    parts = {}; done = set()
    be = [0] * NB; bt = [0] * NB; nok = 0; nf = 0
    t_end = time.time() + 60
    print("listening for PIO-decoder dumps on :1234 (blasting {}B payloads -> {}:{}) ...".format(
        len(msg), DEV, DEAD_PORT))
    while len(done) < N_WANT and time.time() < t_end:
        try: d, _ = s.recvfrom(2048)
        except socket.timeout: continue
        if len(d) < 12: continue
        dec_id, seq, clen = struct.unpack_from("<III", d, 0)
        if dec_id in done or clen == 0 or clen > 1 << 20: continue
        nchunks = (clen + 511) // 512
        p = parts.setdefault(dec_id, {"clen": clen, "nc": nchunks, "chunks": {}})
        p["chunks"][seq] = d[12:]
        if len(p["chunks"]) < p["nc"]:
            continue
        decoded = bytearray()
        for i in range(p["nc"]):
            decoded += p["chunks"].get(i, b"")
        decoded = bytes(decoded[:clen]); done.add(dec_id)
        if "--save" in sys.argv:
            os.makedirs(os.path.join(os.path.dirname(__file__), "dumps"), exist_ok=True)
            open(os.path.join(os.path.dirname(__file__), "dumps",
                              "win_{}.bin".format(dec_id)), "wb").write(decoded)
        frame, inv, sfd, ok = best_frame(decoded)
        if frame is None:
            print("  dec_id {}: {}B window, NO SFD found".format(dec_id, len(decoded)))
            continue
        nf += 1; nok += 1 if ok else 0
        print("  dec_id {}: frame {}B  sfd@bit {}  inv={}  FCS {}".format(
            dec_id, len(frame), sfd, inv, "OK" if ok else "FAIL"))
        if ok: score_frame(frame, be, bt)
    stop.set()
    print_bins(be, bt, nok, nf)
    if nf == 0:
        print("\nNo decodable windows. If windows arrived but no SFD: the PIO bit\n"
              "polarity/timing is off — check the [8] skip delay in eth_rx_pio.rs.")
    return 0

if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    sys.exit(live())
