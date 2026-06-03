#!/usr/bin/env python3
"""RX decode FCS-fail vs frame-size sweep (docs/rx-bulk-ceiling.md).

Blasts UDP to a *closed* port on the device so each frame is decoded + FCS-counted
on core 1 BEFORE smoltcp drops it (no socket / no ICMP in the NIC `fd-bench diag`
build) -> isolates RX decode from TCP/ACK/echo. Read the device's per-second
`[Rx] dec=.. ok=.. fail=..` CDC line alongside this (e.g. /tmp/cdc_read.py) to get
the fail% at each size.

Usage:  rx-decode-sweep.py <size> [dur_s=4] [pps_cap=0]
  size      UDP payload bytes (on-wire frame ~= size + 42).
  pps_cap   0 = max rate (CAUTION: saturating full-MTU has hung the device — see
            rx-bulk-ceiling.md §6); use ~400 for a safe rate-limited sweep.

Payload is os.urandom (representative). An all-0x55 payload is preamble-like and
skews mid-size results — don't use it. Target IP/port below match the default NIC
build (static 192.168.37.24) + a port with no listener.
"""
import socket, sys, os, time

IP = "192.168.37.24"
PORT = 1239  # no socket in the fd-bench/diag NIC build -> pure RX decode, no TX

size = int(sys.argv[1])
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
pps = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
delay = (1.0 / pps) if pps > 0 else 0.0

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
end = time.time() + dur
sent = 0
while time.time() < end:
    try:
        s.sendto(os.urandom(size), (IP, PORT))
        sent += 1
    except OSError:
        time.sleep(0.001)
    if delay:
        time.sleep(delay)
print(f"sent {sent} pkts of {size}B in {dur}s (pps_cap={pps})")
