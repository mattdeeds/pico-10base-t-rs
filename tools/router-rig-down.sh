#!/usr/bin/env bash
# Route-1 teardown — remove the Wi-Fi client config that `router-rig-up.sh` added
# (the /32 route, the address, the association). Idempotent. The WAN side is
# reverted separately by Ctrl-C'ing `tools/wan-test-host.sh`.
#
#   sudo tools/router-rig-down.sh
set -u
cd "$(dirname "$0")"; . ./rig-env.sh

[ "$(id -u)" = 0 ] || { echo "must run as root (sudo tools/router-rig-down.sh)"; exit 1; }

ip route del "$SRV/32" via "$GW" dev "$WLAN" 2>/dev/null || true
pkill -f "wpa_supplicant.*$WLAN" 2>/dev/null || true
ip addr flush dev "$WLAN" 2>/dev/null || true
ip link set "$WLAN" down 2>/dev/null || true
rm -f "$RIG_ENV_FILE"
echo "rig down: removed $SRV/32 route, deassociated $WLAN, cleared its address."
