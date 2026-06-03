#!/usr/bin/env bash
# Route-1 step B — the NON-ROOT measurement loop. Assumes `router-rig-up.sh`
# already associated the Wi-Fi client + installed the /32 route to $SRV via the
# Pico, and that a SEPARATE WAN host ($SRV) is running `iperf3 -s` (see rig-env.sh).
# This runs only the iperf3 *client* over the routed/NAT'd path and snapshots the
# Pico mgmt page before/after. Nothing here needs root — no `ip`/`wpa`/`dhclient`,
# no local server, just iperf3 -c + curl.
#
# Run as often as you like:  tools/router-measure.sh
# (Watch the device [Perf]/CPU line over CDC during the runs; the mgmt snaps
# below capture the Forward/Bytes/Queue/CPU counters either side of the load.)
set -u
cd "$(dirname "$0")"; . ./rig-env.sh
[ -f "$RIG_ENV_FILE" ] && . "$RIG_ENV_FILE"   # pulls LEASE_IP from rig-up
LEASE_IP="${LEASE_IP:-192.168.4.10}"

command -v iperf3 >/dev/null || { echo "need iperf3 (sudo apt install iperf3)"; exit 1; }
[ "$SRV" = CHANGE_ME ] && { echo "SRV unset — run 'SRV=<wan-host-ip> sudo tools/router-rig-up.sh' first (it persists SRV for this step)"; exit 1; }
# Fail fast if rig-up wasn't run — the /32 route is what makes $SRV reachable via
# the Pico (and `ip route get` is a read, so it needs no privilege).
# Check the path as the *client* would: `from $LEASE_IP` applies the source
# policy rule (table $RT_TABLE → via the Pico). A plain `ip route get $SRV`
# would show eno1 (main table), which is correct for forwarded traffic but not
# what we're measuring.
ip route get "$SRV" from "$LEASE_IP" 2>/dev/null | grep -q "dev $WLAN" \
  || { echo "no $SRV route via $WLAN from $LEASE_IP — run 'sudo tools/router-rig-up.sh' first"; exit 1; }
# Confirm the remote iperf3 server answers through the Pico before the real runs.
iperf3 -c "$SRV" -t 1 --connect-timeout 3000 --bind "$LEASE_IP" >/dev/null 2>&1 \
  || { echo "can't reach iperf3 server $SRV via the Pico — is 'iperf3 -s' running on the WAN host, and the Pico's 10BT on enp1s0f0?"; exit 1; }

snap() { echo "--- mgmt page ($1) ---"; curl -s --interface "$WLAN" --max-time 6 "http://$GW/" \
           | grep -E 'Forward|Bytes|Queue|NAT:|CPU:' || echo "(mgmt page unreachable)"; }

snap before
echo "== 3a. TCP download (WAN->client, the common case) =="
iperf3 -c "$SRV" -t "$DUR" -R -b 0 -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|sender|bitrate' || true
echo "== 3b. TCP upload (client->WAN) =="
iperf3 -c "$SRV" -t "$DUR" -b 0 -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|sender|bitrate' || true
echo "== 3c. UDP (find the pps/loss knee — bump -b until loss climbs) =="
iperf3 -c "$SRV" -t "$DUR" -u -b 5M -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|lost|bitrate' || true
snap after

echo "done — compare the before/after mgmt Bytes/Queue/CPU + drop counters, and the device [Perf] line over CDC."
