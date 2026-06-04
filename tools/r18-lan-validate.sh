#!/usr/bin/env bash
# R18 LAN-side validation — run as ROOT on the WAN test host.
#
# Proves the two R18 acceptance items from a real Wi-Fi client:
#   (1) the LAN DHCP server now hands out a DNS server, and a client resolves
#       names through the Pico's NAT (R17 NAPT carries port 53), and
#   (2) the mgmt page shows the connected client + WAN link + NAT counters.
#
# SAFE BY DESIGN: only touches the Wi-Fi client iface ($WLAN) and a single /32
# host route. It NEVER changes the default route, enp1s0f0 (the Pico's WAN
# upstream), or eno1 (your SSH path). Run `wan-test-host.sh` first (the WAN
# upstream must be up — it already is if the Pico shows ip=192.168.37.129).
set -u

WLAN=wlx1cbfcefa0796
AP_SSID=pico-10bt-router    # match src/wireless.rs AP_SSID
AP_PSK=change-me-please     # match src/wireless.rs AP_PASSPHRASE
GW=192.168.4.1            # Pico LAN gateway
DNS_VIA_NAT=1.1.1.1       # off-host resolver, reached only via the Pico's NAT
NAME=example.com

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 1; }

echo "== 1. associate $WLAN ($(cat /sys/class/net/$WLAN/address)) to $AP_SSID =="
pkill -f "wpa_supplicant.*$WLAN" 2>/dev/null; sleep 0.3
ip addr flush dev "$WLAN" 2>/dev/null
ip link set "$WLAN" up
wpa_passphrase "$AP_SSID" "$AP_PSK" > /tmp/r18-wpa.conf
wpa_supplicant -B -i "$WLAN" -c /tmp/r18-wpa.conf
for i in $(seq 1 20); do
  st=$(wpa_cli -i "$WLAN" status 2>/dev/null | sed -n 's/^wpa_state=//p')
  echo "  wpa_state=$st"
  [ "$st" = COMPLETED ] && break
  sleep 1
done

echo "== 2. DHCP exchange (no reconfig) — prove the DNS option is handed out =="
rm -f /tmp/r18-dhcp.leases
dhclient -1 -v -lf /tmp/r18-dhcp.leases -sf /bin/true "$WLAN" 2>&1 | sed -n '1,30p'
echo "--- lease contents (R18: expect option domain-name-servers 192.168.37.19) ---"
grep -E 'fixed-address|option (routers|domain-name-servers)' /tmp/r18-dhcp.leases \
  || echo "(no lease captured — is the AP associated?)"

echo "== 3. assign the leased IP + GET the mgmt page over the LAN =="
LEASE_IP=$(grep -oP 'fixed-address \K[0-9.]+' /tmp/r18-dhcp.leases | tail -1)
LEASE_IP=${LEASE_IP:-192.168.4.10}
ip addr add "$LEASE_IP/24" dev "$WLAN" 2>/dev/null
echo "  client IP = $LEASE_IP"
curl -s --interface "$WLAN" --max-time 6 "http://$GW/" || echo "(curl failed)"

echo
echo "== 4. resolve a name through the Pico's NAT (off-host resolver) =="
# /32 via the Pico is more-specific than the eno1 default → only this query
# detours through the Pico; everything else (incl. your SSH) stays on eno1.
ip route add "$DNS_VIA_NAT/32" via "$GW" dev "$WLAN" 2>/dev/null
echo "  dig @$DNS_VIA_NAT $NAME (routed via $GW):"
dig +time=3 +tries=2 -b "$LEASE_IP" @"$DNS_VIA_NAT" "$NAME" A +short || echo "(dig failed)"
ip route del "$DNS_VIA_NAT/32" via "$GW" dev "$WLAN" 2>/dev/null

echo
echo "== done. Watch the Pico CDC: [Fwd] sent climbs, [Nat] ct>=1 out/in climb. =="
echo "   To fully tear down the client:"
echo "     pkill -f 'wpa_supplicant.*$WLAN'; ip addr flush dev $WLAN; ip link set $WLAN down"
