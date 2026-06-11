//! Software 10BASE-T Ethernet for the RP2350 (Hazard3 RISC-V): PIO Manchester
//! TX + PIO/DMA RX sampler with an IRQ-captured / thread-decoded pipeline,
//! bridged to smoltcp via `eth_mac::EthMac`.
//!
//! This library is the reusable transport core. The router application
//! (`src/main.rs` + `wireless`/`forward`/`wan`/... modules) is the in-tree
//! consumer; external projects (e.g. `pico-remote-probe`) depend on this crate
//! instead of carrying copies of these modules.
//!
//! Layout:
//! - [`eth_tx`] / [`eth_rx`] — PIO TX state machines + DMA double-buffer RX
//!   sampler (the `DMA_IRQ_0` handler lives in [`eth_mac`]).
//! - [`eth_rx_dpll`] — edge-track DPLL Manchester decoder (swapped for the
//!   open-loop decoder under `--features decoder-openloop`).
//! - [`eth_mac`] — smoltcp `phy::Device` over the TX/RX pair + RX frame inbox.
//! - [`manchester`] / [`crc`] — encode tables + FCS.
//! - [`multicore_riscv`] — hand-rolled core-1 launch for Hazard3 (no
//!   `cortex-m` multicore support on this target).
//! - [`pico_reset`] — `picotool`-compatible USB vendor reset interface.
//! - [`pio_util`] — shared PIO divider/program helpers.
//! - [`cyw43_phy`] (feature `cyw43-phy`) — smoltcp `phy::Device` adapter over
//!   cyw43's `NetDriver`, for boards that also use the Pico 2 W radio.
//! - [`cycles`] (feature `router`) — `mcycle`-based per-core CPU-utilisation
//!   counters for perf characterization.

#![no_std]

pub mod crc;
pub mod eth_mac;
pub mod eth_rx;
// Edge-track DPLL Manchester decoder (productized — Phase 3b). Excluded
// from the openloop A/B build so the dead-code warnings don't fire.
#[cfg(not(feature = "decoder-openloop"))]
pub mod eth_rx_dpll;
pub mod eth_tx;
pub mod manchester;
pub mod multicore_riscv;
pub mod pico_reset;
pub mod pio_util;
// R14.3 — smoltcp phy::Device adapter over cyw43's NetDriver. Gated on the
// narrow `cyw43-phy` feature (just cyw43 + embassy-net-driver) so consumers
// can use it without pulling the full `wireless` executor stack.
#[cfg(feature = "cyw43-phy")]
pub mod cyw43_phy;
// Perf characterization step 2 — `mcycle`-based per-core CPU-utilisation
// counters (core-1 RX decode + core-0 forwarding fast-path). Router build only
// (eth_mac's decode span is gated the same way).
#[cfg(feature = "router")]
pub mod cycles;
