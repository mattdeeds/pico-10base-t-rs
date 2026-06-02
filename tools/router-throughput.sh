#!/usr/bin/env bash
# Routed-throughput characterization — run as ROOT on the WAN test host.
#
# Measures the FULL routed/NAT'd path (WiFi client → Pico NAPT → 10BASE-T WAN),
# which the R11/R12 numbers never covered (those were the 10BT NIC in isolation).
# A WiFi client runs iperf3 against a server on the WAN-side host *through* the
# Pico, while we snapshot the device's mgmt-page perf counters before/after.
#
# PREREQS:
#   - iperf3 installed on the host   (apt install iperf3)
#   - the WAN upstream up            (tools/wan-test-host.sh — Pico must show a lease)
#   - power-meter NOT required (this is throughput, not power)
#
# SAFE BY DESIGN: only touches the Wi-Fi client iface ($WLAN) + one /32 host route
# (to the iperf server via the Pico). Never changes the default route / eno1 (SSH).
#
# NOTE: untested end-to-end until the rig is live — refine on first real run.
set -u

WLAN=wlx1cbfcefa0796
AP_SSID=pico-rp2350-router
AP_PSK=picorouter2350
GW=192.168.4.1                 # Pico LAN gateway
SRV=192.168.37.19              # iperf3 server = the WAN-side host (reached via the Pico's NAT)
DUR=10                         # seconds per test

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 1; }
command -v iperf3 >/dev/null || { echo "need iperf3 (apt install iperf3)"; exit 1; }

snap() { echo "--- mgmt page ($1) ---"; curl -s --interface "$WLAN" --max-time 6 "http://$GW/" \
           | grep -E 'Forward|Bytes|Queue|NAT:' || echo "(mgmt page unreachable)"; }

echo "== 1. associate + lease the Wi-Fi client =="
pkill -f "wpa_supplicant.*$WLAN" 2>/dev/null; sleep 0.3
ip addr flush dev "$WLAN" 2>/dev/null
ip link set "$WLAN" up
wpa_passphrase "$AP_SSID" "$AP_PSK" > /tmp/rt-wpa.conf
wpa_supplicant -B -i "$WLAN" -c /tmp/rt-wpa.conf
for i in $(seq 1 20); do
  [ "$(wpa_cli -i "$WLAN" status 2>/dev/null | sed -n 's/^wpa_state=//p')" = COMPLETED ] && break
  sleep 1
done
rm -f /tmp/rt.leases
dhclient -1 -v -lf /tmp/rt.leases -sf /bin/true "$WLAN" 2>&1 | grep -i bound || true
LEASE_IP=$(grep -oP 'fixed-address \K[0-9.]+' /tmp/rt.leases | tail -1); LEASE_IP=${LEASE_IP:-192.168.4.10}
ip addr add "$LEASE_IP/24" dev "$WLAN" 2>/dev/null
echo "  client IP = $LEASE_IP"

echo "== 2. start the iperf3 server on the host (background) =="
pkill -x iperf3 2>/dev/null; sleep 0.3
iperf3 -s -1 -D            # -1 = serve one session then exit; -D = daemonize
# route ONLY the iperf server through the Pico (more-specific than the eno1 default)
ip route add "$SRV/32" via "$GW" dev "$WLAN" 2>/dev/null

snap before
echo "== 3a. TCP download (WAN→client, the common case) =="
iperf3 -c "$SRV" -t "$DUR" -R -b 0 -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|sender|bitrate' || true
echo "== 3b. TCP upload (client→WAN) =="
pkill -x iperf3 2>/dev/null; iperf3 -s -1 -D; sleep 0.3
iperf3 -c "$SRV" -t "$DUR" -b 0 -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|sender|bitrate' || true
echo "== 3c. UDP (find the pps/loss knee — bump -b until loss climbs) =="
pkill -x iperf3 2>/dev/null; iperf3 -s -1 -D; sleep 0.3
iperf3 -c "$SRV" -t "$DUR" -u -b 5M -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|lost|bitrate' || true
snap after

echo "== 4. cleanup (route only; leaves the client associated) =="
ip route del "$SRV/32" via "$GW" dev "$WLAN" 2>/dev/null
pkill -x iperf3 2>/dev/null
echo "done. Read the device [Perf] line over CDC during the runs for live rates,"
echo "and compare the before/after mgmt Bytes/Queue/drop counters above."
