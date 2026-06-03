#!/usr/bin/env bash
# Convenience wrapper (run as ROOT): the whole routed-throughput rig in one shot —
# rig-up -> measure -> rig-down. Measures the FULL routed/NAT'd path (WiFi client
# -> Pico NAPT -> 10BASE-T WAN), which the R11/R12 numbers never covered.
#
# For the route-1 workflow — one-time root setup, then *repeated NON-root*
# measurement (so Claude can drive the loop without root) — use the pieces:
#   sudo tools/router-rig-up.sh     # once   (root: associate Wi-Fi client + /32 route)
#   tools/router-measure.sh         # repeat (NO root: iperf3 + mgmt snaps)
#   sudo tools/router-rig-down.sh   # teardown (root)
#
# Prereq either way: `sudo tools/wan-test-host.sh` up in another terminal (the
# Pico must show a WAN lease). Shared config lives in tools/rig-env.sh.
#
# SAFE BY DESIGN: only touches the Wi-Fi client iface + one /32 host route to the
# iperf server via the Pico. Never changes the default route / eno1 (SSH).
set -u
cd "$(dirname "$0")"

[ "$(id -u)" = 0 ] || { echo "must run as root (or use the rig-up/measure/down split — see header)"; exit 1; }

./router-rig-up.sh && ./router-measure.sh
./router-rig-down.sh
