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
[ "$SRV" = CHANGE_ME ] && { echo "set SRV to the separate WAN host's IP, e.g.:  SRV=192.168.0.50 tools/router-rig-up.sh"; exit 1; }

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

# Route the iperf server through the Pico, but ONLY for traffic this host
# *originates* as the client (src = $LEASE_IP). A plain main-table /32 also
# catches the packets this host *forwards* as the WAN gateway — the Pico's
# NAT'd frames (src = the Pico's WAN IP) — and bounces them back to the Pico to
# be re-NAT'd, an infinite loop (this one host is BOTH the client and the
# gateway). Source policy routing scopes the /32 to client-originated traffic;
# forwarded traffic falls through to `main` (→ out eno1 to the real server).
RT_TABLE="${RT_TABLE:-100}"
ip route del "$SRV/32" 2>/dev/null || true                      # drop any stale main-table /32
ip rule del from "$LEASE_IP" lookup "$RT_TABLE" 2>/dev/null || true   # idempotent re-run
ip rule add from "$LEASE_IP" lookup "$RT_TABLE"
ip route replace "${GW%.*}.0/24" dev "$WLAN" table "$RT_TABLE"  # on-link, so the nexthop resolves
ip route replace "$SRV/32" via "$GW" dev "$WLAN" table "$RT_TABLE"
echo "  route: $SRV/32 via $GW dev $WLAN (table $RT_TABLE, from $LEASE_IP only — no gateway-forward loop)"

# Hand the leased client IP + the resolved server to the (non-root) measurement
# step, world-readable, so it uses the same values without re-deriving them.
{ printf 'LEASE_IP=%s\n' "$LEASE_IP"; printf 'SRV=%s\n' "$SRV"; printf 'RT_TABLE=%s\n' "$RT_TABLE"; } > "$RIG_ENV_FILE"
chmod 0644 "$RIG_ENV_FILE"
echo "wrote $RIG_ENV_FILE (LEASE_IP=$LEASE_IP SRV=$SRV RT_TABLE=$RT_TABLE) -- now run (NO root):  tools/router-measure.sh"
