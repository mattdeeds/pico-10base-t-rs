//! Per-core CPU utilisation from Hazard3 `mcycle` (router build).
//!
//! Spans add elapsed cycles to accumulators. `usb_task` converts them to
//! per-mille once a second for `[Perf]`, `[Lan]`, and the mgmt page.

use core::sync::atomic::{AtomicU32, Ordering};

/// sys_clk in Hz. Must match the PLL choice in `main.rs`.
#[cfg(not(feature = "clock-150mhz"))]
pub const SYS_CLK_HZ: u32 = 240_000_000;
#[cfg(feature = "clock-150mhz")]
pub const SYS_CLK_HZ: u32 = 150_000_000;

/// Cycles in core 1's `DMA_IRQ_0` (capture + re-arm only).
/// Thread-context decode (`drain_rx_images`) is not counted.
pub static CORE1_BUSY: AtomicU32 = AtomicU32::new(0);
/// Cycles in core 0's forwarding fast path.
pub static FWD_BUSY: AtomicU32 = AtomicU32::new(0);

/// Cycles in the cyw43 gSPI transport (`cmd_read`/`cmd_write`).
pub static CYW43_SPI_BUSY: AtomicU32 = AtomicU32::new(0);
/// Cycles in `net_task`'s poll body, excluding gSPI.
pub static LAN_NET_BUSY: AtomicU32 = AtomicU32::new(0);

/// Latest utilisation, per-mille. Published by `usb_task`.
pub static CPU1_PERMILLE: AtomicU32 = AtomicU32::new(0);
pub static CPU0_PERMILLE: AtomicU32 = AtomicU32::new(0);
/// Latest cyw43 LAN core-0 split, per-mille.
pub static SPI0_PERMILLE: AtomicU32 = AtomicU32::new(0);
pub static NET0_PERMILLE: AtomicU32 = AtomicU32::new(0);

/// Per-mille utilisation of `busy_delta` over `elapsed_us`.
/// Uses measured time because the sample window slips under load.
#[inline]
pub fn permille_over(busy_delta: u32, elapsed_us: u64) -> u32 {
    if elapsed_us == 0 {
        return 0;
    }
    // busy_delta·1e9 ≤ ~1.2e18 (a few s at 240 MHz) fits u64; denom ≈ 1e14.
    (busy_delta as u64 * 1_000 * 1_000_000 / (elapsed_us * SYS_CLK_HZ as u64)) as u32
}

/// Clear `mcountinhibit` so `mcycle` counts. Call once per core, early.
#[inline]
pub fn enable_mcycle() {
    // Safety: writing the zero register to mcountinhibit only un-inhibits the
    // performance counters; it has no other architectural effect.
    unsafe {
        core::arch::asm!("csrw 0x320, x0", options(nomem, nostack));
    }
}

/// Low 32 bits of `mcycle`. Wraps in ~18 s; use `wrapping_sub`.
#[inline(always)]
pub fn mcycle() -> u32 {
    let c: u32;
    // Safety: reading a counter CSR has no side effects.
    unsafe {
        core::arch::asm!("csrr {}, 0xb00", out(reg) c, options(nomem, nostack, preserves_flags));
    }
    c
}

/// Adds elapsed cycles to `acc` on drop, on every exit path.
/// Use on the core that owns `acc`; `mcycle` is per-hart.
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
