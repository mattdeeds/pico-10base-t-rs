# tools/

Host-side scripts for testing and characterizing the firmware. Most are plain
Python/bash with no dependencies beyond `iperf3`/`nc`/`ethtool` where noted.

## Measurement / telemetry
- **`cdc_read.py`** — read the device's USB-CDC telemetry for N seconds
  (`python3 tools/cdc_read.py 12`). Auto-detects the Pico CDC by product string;
  asserts DTR. The workhorse for reading `[R2b]`/`[Rx]`/`[Perf]`/… lines.
- **`rx-decode-sweep.py`** — RX FCS-fail rate vs frame size (UDP to a closed port =
  pure decode, no TCP). Drives the `docs/rx-bulk-ceiling.md` size curve.
- **`lan_upload.py`** — cyw43 Wi-Fi LAN upload throughput (client → device `:9999`
  sink); pairs with `curl http://<ap>/bulk` for download.

## Offline decoder bench
- **`clock-recovery/`** — develop the RX Manchester decoder without flashing:
  a captured raw-sample corpus + `harness.py` (scores per-byte error bins + FCS-OK)
  + `noise_compare.py` (matched-filter vs single-sample noise robustness). See its
  own README and `docs/clock-recovery-decoder-plan.md`.
- **`dpll-rust/`** — a small standalone Rust crate mirroring the decoder for offline
  iteration.
- **`conntrack_selftest.rs`** — host self-test of the NAPT conntrack logic
  (`rustc -O tools/conntrack_selftest.rs && ./conntrack_selftest`).

## Router / routed-throughput rig (needs root for setup)
- **`wan-test-host.sh`** — turn this host into the device's WAN upstream gateway
  (dnsmasq DHCP+DNS on the wired NIC + nftables masquerade out your uplink).
- **`rig-env.sh`** — shared config for the rig (sourced; override via env). ⚠️ set
  `AP_SSID`/`AP_PSK` to match your firmware AP creds (`src/wireless.rs`).
- **`router-rig-up.sh`** (root) → **`router-measure.sh`** (no root, iperf3 through
  the NAT) → **`router-rig-down.sh`** (teardown); **`router-throughput.sh`** is the
  all-in-one wrapper. See `docs/perf-characterization-plan.md`.
- **`r18-lan-validate.sh`** — LAN join + DNS-through-NAT + mgmt-page check.

## Misc
- **`99-pico-rust.rules`** — udev rules so the Pico CDC + CMSIS-DAP probe are
  accessible without root (`sudo cp tools/99-pico-rust.rules /etc/udev/rules.d/`).
