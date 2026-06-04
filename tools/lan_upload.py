#!/usr/bin/env python3
"""Measure cyw43 Wi-Fi LAN upload (client -> device RX) for the router build.

Streams bulk into the device's TCP sink on the LAN AP (192.168.4.1:9999) and
reports the achieved rate. Source-binds to the DHCP-leased client IP so the
traffic takes the Wi-Fi path (with the policy-routing recipe in
docs/perf-characterization-plan.md §3.5, source IP 192.168.4.10 -> table 100 ->
the wlx Wi-Fi interface, not the wired WAN).

Adjust SRC/DST to your lease + AP IP. The download direction is just
`curl http://192.168.4.1/bulk` (see the §3.5 recipe).
"""
import socket, time

SRC = ("192.168.4.10", 0)   # the client's DHCP lease on the Pico AP
DST = ("192.168.4.1", 9999) # the Pico's LAN IP + the fd-bench/router TCP sink
DUR = 14.0

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(SRC)
s.connect(DST)
buf = b"\x55" * 65536
sent = 0
t0 = time.time()
try:
    while time.time() - t0 < DUR:
        sent += s.send(buf)
except OSError as e:
    print("send err:", e)
dt = time.time() - t0
s.close()
print("upload: sent=%d B  rate=%d B/s  time=%.1fs" % (sent, sent / dt, dt))
