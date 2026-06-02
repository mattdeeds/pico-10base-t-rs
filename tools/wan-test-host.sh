#!/usr/bin/env bash
#
# wan-test-host.sh — host-side harness for R15a (WAN-as-DHCP-client).
#
# Turns this Linux host into the Pico's upstream gateway on the wired 10BASE-T
# link so the device can DHCP-lease, ping 8.8.8.8, and resolve a name:
#
#   * forces the device-facing NIC to 10BASE-T half-duplex (autoneg off)
#   * runs dnsmasq on it as a DHCP server + DNS forwarder
#   * NATs (masquerade) the device subnet out the host's real uplink
#
# Uses nftables (Debian 13+ native; no iptables needed). Everything it changes
# is recorded and reverted on exit (Ctrl-C) — or run `wan-test-host.sh down`
# to force-clean after an ungraceful kill.
#
# Requires: nftables (`nft`, preinstalled on Debian 13) + dnsmasq.
# dnsmasq missing?  sudo apt install dnsmasq-base   (the binary, no system service)
#
# Usage:
#   sudo tools/wan-test-host.sh            # set up, run dnsmasq, clean up on Ctrl-C
#   sudo tools/wan-test-host.sh down       # force teardown (idempotent)
#
#   WAN_IF=enp1s0f0  UPLINK_IF=eno1  sudo -E tools/wan-test-host.sh
#     WAN_IF    device-facing NIC      (default: enp1s0f0)
#     UPLINK_IF host's internet iface  (default: auto-detected default route)
#
# Then flash the device:  cargo run --release --features wan-dhcp
# and watch the CDC `[Wan]` line for: ip=… gw=… dns=… ping=ok/sent <name>=<A>.

set -euo pipefail

# ── Config (override via env) ──────────────────────────────────────────────
WAN_IF="${WAN_IF:-enp1s0f0}"
UPLINK_IF="${UPLINK_IF:-$(ip route show default 2>/dev/null \
    | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')}"
HOST_IP4="192.168.37.19"
HOST_CIDR="${HOST_IP4}/24"
SUBNET="192.168.37.0/24"
RANGE_LO="192.168.37.100"
RANGE_HI="192.168.37.150"
LEASE="1h"
UPSTREAM_DNS=(8.8.8.8 1.1.1.1)
NFT_TABLE="pico_wan"   # our own nft table — deleted wholesale on teardown

# ── R16 forwarding: the LAN subnet behind the Pico + the route back ─────────
# For L3 forwarding (no NAT) the upstream must know how to return packets to the
# LAN — add a route to it via the Pico's WAN IP, and loosen reverse-path filter
# (192.168.4.x arriving on the WAN NIC looks asymmetric). PICO_WAN_IP is the
# Pico's DHCP lease (deterministic for its fixed MAC); override if it differs.
LAN_SUBNET="192.168.4.0/24"
PICO_WAN_IP="${PICO_WAN_IP:-192.168.37.129}"
# NAT_LAN=1 also masquerades the LAN subnet out the uplink, so a LAN client can
# reach the *internet* via the host's NAT (the Pico itself still does NOT NAT —
# that's R17). Leave 0 for the pure no-NAT forwarding test.
NAT_LAN="${NAT_LAN:-0}"

# ── State files (so `down` can revert a previous run) ──────────────────────
FWSTATE="/tmp/pico-wan-ipforward.save"
ADDRFLAG="/tmp/pico-wan-added-addr"
RPSTATE="/tmp/pico-wan-rpfilter.save"

log()  { printf '\033[36m[wan-host]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[wan-host] WARN:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31m[wan-host] ERROR:\033[0m %s\n' "$*" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || die "must run as root (sudo)."
command -v ip  >/dev/null || die "missing command: ip (iproute2)."
command -v nft >/dev/null || die "missing command: nft — install with: sudo apt install nftables"
command -v dnsmasq >/dev/null || die "missing command: dnsmasq — install with: sudo apt install dnsmasq-base"

# ── nftables NAT/forward (a dedicated table; delete-all on teardown) ────────
nft_up() {
    # NAT_LAN=1 adds a masquerade for the LAN subnet (internet-via-host demo).
    local lan_nat=""
    [ "$NAT_LAN" = "1" ] && lan_nat="        ip saddr ${LAN_SUBNET} oifname \"${UPLINK_IF}\" masquerade"
    nft -f - <<EOF
table ip ${NFT_TABLE} {
    chain postrouting {
        type nat hook postrouting priority srcnat;
        ip saddr ${SUBNET} oifname "${UPLINK_IF}" masquerade
${lan_nat}
    }
    chain forward {
        type filter hook forward priority filter;
        iifname "${WAN_IF}" oifname "${UPLINK_IF}" accept
        iifname "${UPLINK_IF}" oifname "${WAN_IF}" ct state related,established accept
    }
}
EOF
}
nft_down() { nft delete table ip "${NFT_TABLE}" 2>/dev/null || true; }

teardown() {
    # Idempotent — safe to call twice (trap + explicit) or against a dead run.
    trap - EXIT INT TERM
    log "tearing down…"
    [ -n "${DNSMASQ_PID:-}" ] && kill "$DNSMASQ_PID" 2>/dev/null || true
    nft_down
    if [ -f "$FWSTATE" ]; then
        sysctl -wq "net.ipv4.ip_forward=$(cat "$FWSTATE")" || true
        rm -f "$FWSTATE"
    fi
    if [ -f "$ADDRFLAG" ]; then
        ip addr del "$HOST_CIDR" dev "$WAN_IF" 2>/dev/null || true
        rm -f "$ADDRFLAG"
    fi
    # R16: remove the LAN route-back + restore reverse-path filter.
    ip route del "$LAN_SUBNET" via "$PICO_WAN_IP" dev "$WAN_IF" 2>/dev/null || true
    if [ -f "$RPSTATE" ]; then
        sysctl -wq "net.ipv4.conf.${WAN_IF}.rp_filter=$(cat "$RPSTATE")" 2>/dev/null || true
        rm -f "$RPSTATE"
    fi
    log "done — host restored (NIC link/autoneg left as-is)."
}

if [ "${1:-run}" = "down" ]; then
    teardown
    exit 0
fi

# ── Pre-flight ─────────────────────────────────────────────────────────────
ip link show "$WAN_IF" >/dev/null 2>&1 || die "device-facing iface '$WAN_IF' not found (set WAN_IF=…)."
[ -n "$UPLINK_IF" ] || die "could not auto-detect the uplink iface (set UPLINK_IF=…)."
ip link show "$UPLINK_IF" >/dev/null 2>&1 || die "uplink iface '$UPLINK_IF' not found (set UPLINK_IF=…)."
[ "$WAN_IF" != "$UPLINK_IF" ] || die "WAN_IF and UPLINK_IF are the same ($WAN_IF) — that can't NAT."

log "device-facing NIC : $WAN_IF  ($HOST_CIDR, 10BASE-T half-duplex)"
log "internet uplink   : $UPLINK_IF  (masquerade $SUBNET)"
log "DHCP pool         : $RANGE_LO–$RANGE_HI  gw/dns=$HOST_IP4  lease=$LEASE"

trap teardown EXIT
trap 'exit 130' INT TERM

# ── Setup ──────────────────────────────────────────────────────────────────
ip link set "$WAN_IF" up
if command -v ethtool >/dev/null; then
    ethtool -s "$WAN_IF" speed 10 duplex half autoneg off 2>/dev/null \
        || warn "ethtool 10HD force failed (driver may not support it) — continuing."
else
    warn "ethtool not installed — skipping the 10BASE-T half-duplex force."
fi

if ip -o -4 addr show dev "$WAN_IF" | grep -qw "$HOST_IP4"; then
    log "$HOST_IP4 already on $WAN_IF — leaving it."
else
    ip addr add "$HOST_CIDR" dev "$WAN_IF"
    touch "$ADDRFLAG"
fi

cat /proc/sys/net/ipv4/ip_forward > "$FWSTATE"
sysctl -wq net.ipv4.ip_forward=1

# R16: route LAN-subnet replies back via the Pico, and loosen reverse-path filter
# on the WAN NIC (192.168.4.x arriving there is asymmetric-looking). Inert for the
# R15 client tests; required for forwarding. The route's next-hop resolves once
# the Pico is up at PICO_WAN_IP.
ip route replace "$LAN_SUBNET" via "$PICO_WAN_IP" dev "$WAN_IF"
cat "/proc/sys/net/ipv4/conf/${WAN_IF}/rp_filter" > "$RPSTATE" 2>/dev/null || true
sysctl -wq "net.ipv4.conf.${WAN_IF}.rp_filter=2" 2>/dev/null || true
log "R16 route-back: $LAN_SUBNET via $PICO_WAN_IP (NAT_LAN=$NAT_LAN)"

nft_down            # clear any stale table from a prior run
nft_up || die "failed to install nft NAT/forward table."

# A packaged dnsmasq.service (if present + running) would fight us for the DHCP
# socket — stop it; we run our own foreground instance below.
systemctl stop dnsmasq 2>/dev/null || true

# ── Run dnsmasq in the foreground (Ctrl-C → EXIT trap → teardown) ───────────
server_args=()
for s in "${UPSTREAM_DNS[@]}"; do server_args+=(--server="$s"); done

log "starting dnsmasq — Ctrl-C to stop and revert. Now flash the device:"
log "    cargo run --release --features wan-dhcp"
echo

dnsmasq \
    --interface="$WAN_IF" --bind-interfaces --except-interface=lo \
    --dhcp-range="$RANGE_LO,$RANGE_HI,$LEASE" \
    --dhcp-option=3,"$HOST_IP4" \
    --dhcp-option=6,"$HOST_IP4" \
    --dhcp-authoritative \
    --no-resolv "${server_args[@]}" \
    --log-dhcp --log-queries --log-facility=- \
    --no-daemon &
DNSMASQ_PID=$!
wait "$DNSMASQ_PID"
