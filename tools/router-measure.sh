#!/usr/bin/env bash
# Route-1 step B — the NON-ROOT measurement loop. Assumes `router-rig-up.sh`
# already associated the Wi-Fi client + installed the /32 route to $SRV via the
# Pico. Runs iperf3 (a one-shot server on this host + the client over the
# routed/NAT'd path) and snapshots the Pico mgmt page before/after. Nothing here
# needs root — no `ip`/`wpa`/`dhclient`, just iperf3 + curl.
#
# Run as often as you like:  tools/router-measure.sh
# (Watch the device [Perf]/CPU line over CDC during the runs; the mgmt snaps
# below capture the Forward/Bytes/Queue/CPU counters either side of the load.)
set -u
cd "$(dirname "$0")"; . ./rig-env.sh
[ -f "$RIG_ENV_FILE" ] && . "$RIG_ENV_FILE"   # pulls LEASE_IP from rig-up
LEASE_IP="${LEASE_IP:-192.168.4.10}"

command -v iperf3 >/dev/null || { echo "need iperf3 (sudo apt install iperf3)"; exit 1; }
# Fail fast if rig-up wasn't run — the /32 route is what makes $SRV reachable via
# the Pico (and `ip route get` is a read, so it needs no privilege).
ip route get "$SRV" 2>/dev/null | grep -q "dev $WLAN" \
  || { echo "no $SRV route via $WLAN — run 'sudo tools/router-rig-up.sh' first"; exit 1; }

snap() { echo "--- mgmt page ($1) ---"; curl -s --interface "$WLAN" --max-time 6 "http://$GW/" \
           | grep -E 'Forward|Bytes|Queue|NAT:|CPU:' || echo "(mgmt page unreachable)"; }
srv()  { pkill -x iperf3 2>/dev/null; sleep 0.3; iperf3 -s -1 -D; }   # one-shot server

snap before
echo "== 3a. TCP download (WAN->client, the common case) =="
srv; iperf3 -c "$SRV" -t "$DUR" -R -b 0 -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|sender|bitrate' || true
echo "== 3b. TCP upload (client->WAN) =="
srv; iperf3 -c "$SRV" -t "$DUR" -b 0 -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|sender|bitrate' || true
echo "== 3c. UDP (find the pps/loss knee — bump -b until loss climbs) =="
srv; iperf3 -c "$SRV" -t "$DUR" -u -b 5M -f k --bind "$LEASE_IP" 2>&1 | grep -E 'receiver|lost|bitrate' || true
snap after

pkill -x iperf3 2>/dev/null
echo "done — compare the before/after mgmt Bytes/Queue/CPU + drop counters, and the device [Perf] line over CDC."
