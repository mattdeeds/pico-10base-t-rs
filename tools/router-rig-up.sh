#!/usr/bin/env bash
# Route-1 step A — ONE-TIME ROOT setup of the Wi-Fi *client* side of the
# routed-throughput rig. Associates this host's Wi-Fi adapter to the Pico's AP,
# leases an IP, and installs a /32 route so traffic to the iperf3 server ($SRV)
# goes THROUGH the Pico's NAT — never touching the eno1 default route / SSH.
#
# Leaves the client associated + the route in place so `tools/router-measure.sh`
# can run the actual iperf3 tests with NO root, repeatedly. Tear down later with
# `tools/router-rig-down.sh`.
#
# Prereq: the WAN gateway is already up in another terminal
# (`sudo tools/wan-test-host.sh`) and the Pico shows a lease.
#
#   sudo tools/router-rig-up.sh
set -u
cd "$(dirname "$0")"; . ./rig-env.sh

[ "$(id -u)" = 0 ] || { echo "must run as root (sudo tools/router-rig-up.sh)"; exit 1; }

echo "== associate + lease the Wi-Fi client ($WLAN -> $AP_SSID) =="
pkill -f "wpa_supplicant.*$WLAN" 2>/dev/null; sleep 0.3
ip addr flush dev "$WLAN" 2>/dev/null
ip link set "$WLAN" up
wpa_passphrase "$AP_SSID" "$AP_PSK" > /tmp/rt-wpa.conf
wpa_supplicant -B -i "$WLAN" -c /tmp/rt-wpa.conf
for _ in $(seq 1 20); do
  [ "$(wpa_cli -i "$WLAN" status 2>/dev/null | sed -n 's/^wpa_state=//p')" = COMPLETED ] && break
  sleep 1
done
rm -f /tmp/rt.leases
dhclient -1 -v -lf /tmp/rt.leases -sf /bin/true "$WLAN" 2>&1 | grep -i bound || true
LEASE_IP=$(grep -oP 'fixed-address \K[0-9.]+' /tmp/rt.leases | tail -1); LEASE_IP=${LEASE_IP:-192.168.4.10}
ip addr add "$LEASE_IP/24" dev "$WLAN" 2>/dev/null
echo "  client IP = $LEASE_IP"

# Route ONLY the iperf server through the Pico (more-specific than the eno1
# default). `replace` is idempotent across re-runs.
ip route replace "$SRV/32" via "$GW" dev "$WLAN"
echo "  route: $SRV/32 via $GW dev $WLAN"

# Hand the leased client IP to the (non-root) measurement step, world-readable.
printf 'LEASE_IP=%s\n' "$LEASE_IP" > "$RIG_ENV_FILE"
chmod 0644 "$RIG_ENV_FILE"
echo "wrote $RIG_ENV_FILE -- now run (NO root):  tools/router-measure.sh"
