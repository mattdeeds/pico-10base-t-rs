#!/usr/bin/env python3
# Phase 0 host capture: collect a corpus of raw RX sample buffers from the
# device (requires the temporary capture firmware — see README).
#
# Sends full-MTU known-pattern frames (payload[i] = i & 0xFF) continuously so
# the device keeps capturing, reassembles the device's exfil chunks per
# cap_id, and writes each complete capture's raw run samples to corpus/.
#
# Chunk wire format (UDP broadcast to host:1234):
#   cap_id(u32 LE) | cap_len(u32 LE) | n_chunks(u16 LE) | chunk_idx(u16 LE) | data
import socket, struct, threading, time, os

DEV = "192.168.37.24"; HOST = "192.168.37.19"
CORPUS = os.path.join(os.path.dirname(__file__), "corpus")
N_WANT = 6

def main():
    os.makedirs(CORPUS, exist_ok=True)
    msg = bytes((i & 0xFF) for i in range(1472))
    stop = threading.Event()
    def blast():
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); tx.bind((HOST, 0))
        while not stop.is_set():
            try: tx.sendto(msg, (DEV, 48879))
            except OSError: pass
            time.sleep(0.008)
    threading.Thread(target=blast, daemon=True).start()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 1234)); s.settimeout(1.0)
    parts = {}; saved = set(); t_end = time.time() + 70
    while len(saved) < N_WANT and time.time() < t_end:
        try: d, _ = s.recvfrom(2048)
        except socket.timeout: continue
        if len(d) < 12: continue
        cid, clen, nc, idx = struct.unpack_from("<IIHH", d, 0)
        if cid in saved: continue
        p = parts.setdefault(cid, {"clen": clen, "nc": nc, "chunks": {}})
        p["chunks"][idx] = d[12:]
        if len(p["chunks"]) == nc:
            buf = bytearray()
            for i in range(nc): buf += p["chunks"][i]
            fn = os.path.join(CORPUS, "cap_{}.bin".format(cid))
            open(fn, "wb").write(bytes(buf[:clen]))
            print("saved {} ({} bytes)".format(fn, clen)); saved.add(cid)
    stop.set()
    print("collected {} captures".format(len(saved)))

if __name__ == "__main__":
    main()
