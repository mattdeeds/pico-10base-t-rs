# Shared config for the routed-throughput rig scripts. SOURCED, not executed
# (`. ./rig-env.sh`). Override any value from the environment, e.g.:
#   WLAN=wlp3s0 DUR=20 tools/router-measure.sh
WLAN="${WLAN:-wlx1cbfcefa0796}"            # this host's Wi-Fi client adapter
AP_SSID="${AP_SSID:-pico-rp2350-router}"   # the Pico's AP
AP_PSK="${AP_PSK:-picorouter2350}"
GW="${GW:-192.168.4.1}"                    # Pico LAN gateway (its mgmt page lives here)
SRV="${SRV:-192.168.37.19}"               # iperf3 server = the WAN-side host IP (reached via the Pico's NAT)
DUR="${DUR:-10}"                          # seconds per iperf3 test
# rig-up (root) writes the leased client IP here; measure (non-root) reads it.
RIG_ENV_FILE="${RIG_ENV_FILE:-/tmp/pico-rig.env}"
