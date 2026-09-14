//! Software 10BASE-T Ethernet for the RP2350 (Hazard3 RISC-V).
//!
//! Shared transport core for the router binary and other crates.
//!
//! - [`eth_tx`] / [`eth_rx`] — PIO TX and PIO/DMA RX sampler.
//! - [`eth_rx_dpll`] — edge-track DPLL Manchester decoder.
//! - [`eth_mac`] — smoltcp `phy::Device`, RX IRQ, and frame inbox.
//! - [`manchester`] / [`crc`] — encode table and FCS.
//! - [`multicore_riscv`] — core-1 launch for Hazard3.
//! - [`pico_reset`] — `picotool` USB reset interface.
//! - [`pio_util`] — PIO divider helper.
//! - [`cyw43_phy`] (feature `cyw43-phy`) — smoltcp device over cyw43.
//! - [`cycles`] (feature `router`) — per-core CPU counters.

#![no_std]

pub mod crc;
pub mod eth_mac;
pub mod eth_rx;
// The openloop A/B build replaces the DPLL.
#[cfg(not(feature = "decoder-openloop"))]
pub mod eth_rx_dpll;
pub mod eth_tx;
pub mod manchester;
pub mod multicore_riscv;
pub mod pico_reset;
pub mod pio_util;
// Narrow feature: no full wireless stack needed.
#[cfg(feature = "cyw43-phy")]
pub mod cyw43_phy;
// Router build only, matching eth_mac's decode span.
#[cfg(feature = "router")]
pub mod cycles;
