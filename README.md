# pico-10base-t-rs

Rust port of [Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T), targeting the **Hazard3 RISC-V** cores of the RP2350 on a Raspberry Pi Pico 2 board, with an external ISL3177E + HR911105A magnetics ethernet module.

Companion repo: [../Pico-10BASE-T/](../Pico-10BASE-T/) is the C reference implementation with TX (Phase 1) and RX (Phase 2) both fully working. See its `RESUME.md` and `CLAUDE.md` for hardware schematic, host setup, decoder algorithm details, and lessons learned. This Rust port reuses all the proven design — phase formula, polarity convention, CRC poly, host autoneg-off requirement — but rebuilds the implementation on top of `rp235x-hal` so the larger Rust project can consume it as a [`smoltcp`](https://github.com/smoltcp-rs/smoltcp) `phy::Device`.

## Status

| Phase | Status | What it does |
|---|---|---|
| R0 — Blinky smoke test | ✅ | Confirms toolchain, linker, picotool flashing, RISC-V boot |
| R1 — defmt-rtt logging | 🚧 in progress | Replace `printf` with `defmt::info!` over RTT via probe-rs |
| R2 — TX path (Manchester PIO + DMA) | ⏳ | Port `ser_10base_t.pio` + UDP/IPv4/Ethernet frame builder |
| R3 — RX path (sampler + decoder) | ⏳ | Port PIO sampler + DMA + Niccle-style Manchester decoder |
| R4 — smoltcp Device trait | ⏳ | Implement `phy::Device`; ARP/IPv4/UDP from smoltcp for free |

## Toolchain

- Rust stable (>= 1.82)
- Target: `riscv32imac-unknown-none-elf` (installed via `rustup target add riscv32imac-unknown-none-elf`)
- [picotool](https://github.com/raspberrypi/picotool) for flashing via USB BOOTSEL
- [probe-rs](https://probe.rs/) for flashing + RTT log streaming via SWD

## Hardware

Same as the C repo — see `../Pico-10BASE-T/CLAUDE.md` for the full schematic.

| Signal | Pico 2 pin |
|---|---|
| ISL3177E RO (receiver out → MCU) | GP13 |
| ISL3177E DI (driver in ← MCU) | GP14 |
| Onboard LED (heartbeat) | GP25 |
| SWD debug probe | SWCLK + SWDIO + GND |

## Build & flash

```bash
cargo build --release
cargo run --release        # uses probe-rs runner (SWD flash + RTT stream)
```

If no debug probe is available, swap the runner in `.cargo/config.toml` for `picotool load -fux -t elf` and skip `defmt::info!` output.

## Host setup (same as C repo)

Mandatory after every host reboot — the Pico transmits NLPs only (no FLP bursts), so the NIC has to be coaxed into parallel detection:

```bash
# as root
ip link set enp1s0f0 up
ethtool -s enp1s0f0 speed 10 duplex half autoneg off
ip addr add 192.168.37.19/24 dev enp1s0f0
```

Verify `cat /sys/class/net/enp1s0f0/carrier` reads `1` once the Pico is sending NLPs.

## What carries over from the C version (knowledge, not code)

- **PIO TX**: 20 MHz state machine driving GP14 single-ended, encoding via a 256-entry Manchester table indexed by data byte. Side-set value 1 = DI high = positive line diff.
- **PIO RX**: 60 MHz sampler running `in pins, 1` into a 32 KB ring buffer. 3 samples per Manchester half-bit.
- **Decoder phase formula**: data bit `k` value = sample at index `F + 4 + 6*k`, where `F` is the first H→L transition (= start of HB[0] when entering from idle).
- **SFD detection**: scan decoded bit stream for the first `1,1` pair (preamble is alternating; SFD's last two bits break that pattern).
- **CRC-32 polynomial**: `0xEDB88320` (reflected IEEE 802.3). FCS transmitted little-endian on the wire.
- **End-of-frame**: ≥3 consecutive same-level samples = TP_IDL or idle (Niccle heuristic).

## What changes (and why)

- **Continuous DMA via double-buffer**, not single-channel ring-mode. `rp235x-hal` exposes endless transfers through chained channels + `EndlessWriteTarget`; the RP2350-specific single-channel endless mode isn't surfaced cleanly. Two channels instead of one.
- **Frame buffer above the decoder is owned by smoltcp**, not us. Once R4 lands, our role ends at "decoded Ethernet frame" — smoltcp does ARP, IPv4, UDP, ICMP.
- **Logging via defmt-rtt**, not USB CDC. Lower overhead, structured logs, smaller binary, but requires SWD debug probe.
