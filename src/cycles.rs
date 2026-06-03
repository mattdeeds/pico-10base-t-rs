//! Hazard3 `mcycle`-based CPU-utilization counters — perf characterization
//! step 2 (`docs/perf-characterization-plan.md` §2). Router build only.
//!
//! Two cores share the routing work and we want to know which (if either) is the
//! ceiling under load:
//!
//! - **core 1** runs the 10BT RX-decode `DMA_IRQ_0` handler (the ≤2.57 ms
//!   Manchester + FCS pipeline). [`CORE1_BUSY`] accumulates the cycles spent
//!   there → ≈ core-1 utilisation (core 1 otherwise just `wfi`s between IRQs).
//! - **core 0** runs the forwarding fast-path ([`crate::forward::ForwardingDevice`]'s
//!   `receive` classify + `egress` NAPT / TTL / L2-rewrite). [`FWD_BUSY`]
//!   accumulates those cycles → the *fraction of core-0 wall-clock spent
//!   forwarding* (NOT total core-0 load — the executor / smoltcp / cyw43-SPI cost
//!   is outside the brackets). This is the "cycles/sec in the routing path"
//!   number `docs/router-plan.md` §8.3 asks for.
//!
//! `usb_task` samples both accumulators once a second, divides each delta by
//! [`SYS_CLK_HZ`] (the wall-clock cycles in that ~1 s window), and publishes the
//! result as per-mille into [`CPU1_PERMILLE`] / [`CPU0_PERMILLE`] — read by both
//! the `[Perf]` CDC line and the mgmt page. No external hardware needed.

use core::sync::atomic::{AtomicU32, Ordering};

/// sys_clk in Hz — the per-second utilisation denominator. MUST track `main.rs`'s
/// PLL selection: 240 MHz overclock by default, 150 MHz with `clock-150mhz`.
#[cfg(not(feature = "clock-150mhz"))]
pub const SYS_CLK_HZ: u32 = 240_000_000;
#[cfg(feature = "clock-150mhz")]
pub const SYS_CLK_HZ: u32 = 150_000_000;

/// Cumulative cycles spent in core 1's RX-decode IRQ handler (written by core 1
/// only; read cross-core by `usb_task` on core 0 — RP2350 SRAM is coherent).
pub static CORE1_BUSY: AtomicU32 = AtomicU32::new(0);
/// Cumulative cycles spent in core 0's forwarding fast-path (written by core 0).
pub static FWD_BUSY: AtomicU32 = AtomicU32::new(0);

/// LAN-isolation perf step 4 (`docs/perf-characterization-plan.md` §3.5) —
/// core-0 cost split for the cyw43 LAN, since [`FWD_BUSY`] only brackets the
/// forwarding fast-path (≈0 in a LAN-only test where traffic terminates on the
/// Pico). Both are written on core 0 (the executor's hart). `spi0 + net0` ≈
/// core-0 utilisation under a LAN test.
///
/// [`CYW43_SPI_BUSY`] = cycles inside the busy-poll gSPI transport
/// (`PioSpiCyw43::cmd_read`/`cmd_write`), driven by the cyw43 `Runner` task —
/// the Runner's real CPU cost, the prime "core 0 pinned by the busy-poll SPI
/// transport" suspect (decision-matrix row 3 → gSPI DMA, §4-G).
pub static CYW43_SPI_BUSY: AtomicU32 = AtomicU32::new(0);
/// [`LAN_NET_BUSY`] = cycles in `net_task`'s per-poll body (smoltcp `iface.poll`,
/// the DHCP/HTTP/sink handlers, the `Cyw43Phy` channel ops) — the stack/app cost
/// *excluding* the gSPI (that's the Runner's, decoupled via the NetDriver
/// channel).
pub static LAN_NET_BUSY: AtomicU32 = AtomicU32::new(0);

/// Latest sampled utilisation, per-mille (0..=1000 ≈ 0.0..=100.0 %). Published by
/// `usb_task` each second; read by the `[Perf]` line and the mgmt page.
pub static CPU1_PERMILLE: AtomicU32 = AtomicU32::new(0);
pub static CPU0_PERMILLE: AtomicU32 = AtomicU32::new(0);
/// Latest LAN-isolation core-0 split, per-mille (see [`CYW43_SPI_BUSY`] /
/// [`LAN_NET_BUSY`]). Published by `usb_task`; read by the `[Lan]` line + mgmt.
pub static SPI0_PERMILLE: AtomicU32 = AtomicU32::new(0);
pub static NET0_PERMILLE: AtomicU32 = AtomicU32::new(0);

/// Per-mille utilisation over a *measured* window: `busy_delta` cycles divided by
/// the wall-clock cycles in `elapsed_us` (= `elapsed_us * SYS_CLK_HZ / 1e6`),
/// scaled to thousandths (0..=1000 ≈ 0.0..=100.0 %).
///
/// We can't assume the sample window is exactly 1 s: the sampler (`usb_task`, a
/// 1 ms poll cadence) keeps up only while core 0 is idle. Under heavy LAN load
/// core 0 saturates (the busy-poll gSPI Runner), `Timer::after(1ms)` slips, and
/// the `n % 1000` window stretches to *several* seconds — a fixed-1 s divisor
/// would over-read (>100 %, observed on the first LAN run). Dividing by the
/// measured µs keeps every rate/% correct regardless of cadence slip.
#[inline]
pub fn permille_over(busy_delta: u32, elapsed_us: u64) -> u32 {
    if elapsed_us == 0 {
        return 0;
    }
    // busy_delta·1e9 ≤ ~1.2e18 (a few s at 240 MHz) fits u64; denom ≈ 1e14.
    (busy_delta as u64 * 1_000 * 1_000_000 / (elapsed_us * SYS_CLK_HZ as u64)) as u32
}

/// Clear `mcountinhibit` (CSR `0x320`) so `mcycle` advances. Hazard3 boots with
/// the counters inhibited; without this every `mcycle` read returns the same
/// value and all deltas read 0. Call once per core, early (core 0 in `main`, core
/// 1 in its entry point). Verified not to fault on RP2350.
#[inline]
pub fn enable_mcycle() {
    // Safety: writing the zero register to mcountinhibit only un-inhibits the
    // performance counters; it has no other architectural effect.
    unsafe {
        core::arch::asm!("csrw 0x320, x0", options(nomem, nostack));
    }
}

/// Read the low 32 bits of `mcycle` (CSR `0xB00`). Wraps every ~18 s at 240 MHz
/// / ~28 s at 150 MHz — always consume via `wrapping_sub` deltas.
#[inline(always)]
pub fn mcycle() -> u32 {
    let c: u32;
    // Safety: reading a counter CSR has no side effects.
    unsafe {
        core::arch::asm!("csrr {}, 0xb00", out(reg) c, options(nomem, nostack, preserves_flags));
    }
    c
}

/// RAII span: reads `mcycle` on construction and, on drop, adds the elapsed
/// cycles to `acc` (wrap-safe). Drop runs on *every* exit path of the bracketed
/// scope — including the `?` early-returns in `ForwardingDevice::receive`. Place
/// it on the same core that owns `acc`; `mcycle` is per-hart.
pub struct CycleSpan {
    acc: &'static AtomicU32,
    start: u32,
}

impl CycleSpan {
    #[inline(always)]
    pub fn new(acc: &'static AtomicU32) -> Self {
        Self {
            acc,
            start: mcycle(),
        }
    }
}

impl Drop for CycleSpan {
    #[inline(always)]
    fn drop(&mut self) {
        self.acc
            .fetch_add(mcycle().wrapping_sub(self.start), Ordering::Relaxed);
    }
}
