# Shared config for the routed-throughput rig scripts. SOURCED, not executed
# (`. ./rig-env.sh`). Override any value from the environment, e.g.:
#   WLAN=wlp3s0 DUR=20 tools/router-measure.sh
WLAN="${WLAN:-wlx1cbfcefa0796}"            # this host's Wi-Fi client adapter
AP_SSID="${AP_SSID:-pico-10bt-router}"      # the Pico's AP (match src/wireless.rs AP_SSID)
AP_PSK="${AP_PSK:-change-me-please}"        # match src/wireless.rs AP_PASSPHRASE
GW="${GW:-192.168.4.1}"                    # Pico LAN gateway (its mgmt page lives here)
# iperf3 server = a SEPARATE machine reachable ONLY via the Pico's WAN. It must
# NOT be an IP local to THIS host (a local IP short-circuits via `lo`, never
# traversing the Pico). Run `iperf3 -s` there. Set this to its IP, e.g.:
#   SRV=192.168.0.50 tools/router-measure.sh
SRV="${SRV:-CHANGE_ME}"
DUR="${DUR:-10}"                          # seconds per iperf3 test
# rig-up (root) writes the leased client IP here; measure (non-root) reads it.
RIG_ENV_FILE="${RIG_ENV_FILE:-/tmp/pico-rig.env}"
