//! Hazard3 (RISC-V) core-1 launch — Phase 3a multicore foundation.
//!
//! `rp235x-hal` 0.4's `multicore::Multicore::spawn` is written for the
//! Cortex-M cores: it pokes `ICB.ACTLR` (`enable_actlr_extexclall`) and reads
//! `PPB.VTOR` for the core-1 vector table. On the RP2350's Hazard3 RISC-V
//! cores the Cortex-M PPB is powered down, so `ppb.vtor()` returns garbage and
//! the ACTLR write faults — which is why the earlier Phase-3a attempt hung core
//! 0 in `read_blocking()` (see `docs/cpu-dpll-plan.md` §9a).
//!
//! This module drives the documented bootrom FIFO launch protocol
//! (`[0, 0, 1, vector_table, sp, entry]`, RP2350 datasheet §5.3 / §5.5.5)
//! ourselves, with the RISC-V specifics taken from the pico-sdk
//! (`pico_multicore/multicore.c`):
//!
//! - **`vector_table` is our `mtvec` CSR**, not the Cortex-M VTOR. Core 0 and
//!   core 1 share the same `riscv-rt` trap vector, so we hand core 1 the mtvec
//!   core 0 is already running with.
//! - **A naked trampoline restores `gp`** before jumping to the entry point.
//!   The bootrom jumps straight to the address we give it, bypassing `_start`,
//!   so the global pointer (used for `.sdata`/`.sbss`-relative addressing) is
//!   never set up. We capture core 0's `gp` at launch time and push it for the
//!   trampoline to reload.
//! - **No `ACTLR`/coprocessor setup.** Hazard3 has no data caches (SRAM is
//!   coherent between cores) and implements the A extension, so cross-core
//!   atomics and the SIO hardware spinlocks work without the Cortex-M
//!   shareability fix-up.
//!
//! Unlike the HAL's `spawn`, the handshake here uses *bounded* FIFO polling
//! instead of `read_blocking()`, so a core 1 that never responds returns
//! `Err(LaunchError::Unresponsive)` rather than wedging core 0 forever.

use core::sync::atomic::{compiler_fence, Ordering};

use rp235x_hal as hal;

/// Why a core-1 launch failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaunchError {
    /// Core 1 never echoed the launch handshake within the retry budget.
    Unresponsive,
}

// Naked trampoline. The bootrom enters here with `sp` pointing at the four
// words we pushed and `gp` uninitialised. We must not make any function call
// (or touch a global) until `gp` is restored, so this is hand-written asm that
// pops the four words and tail-calls the wrapper:
//   a0 = entry, a1 = stack_bottom, a2 = core1_wrapper, gp = core 0's gp
// then `jr a2` into `core1_wrapper(entry, stack_bottom)`. Mirrors the pico-sdk
// `core1_trampoline` byte-for-byte so the proven layout is preserved.
core::arch::global_asm!(
    ".pushsection .text.core1_trampoline, \"ax\", @progbits",
    ".global core1_trampoline",
    ".type core1_trampoline, @function",
    "core1_trampoline:",
    "lw   a0, 0(sp)",
    "lw   a1, 4(sp)",
    "lw   a2, 8(sp)",
    "lw   gp, 12(sp)",
    "addi sp, sp, 16",
    "jr   a2",
    ".popsection",
);

extern "C" {
    fn core1_trampoline();
}

/// Reached via the trampoline with `gp` already valid. Kept as a real (non
/// naked) function so it has a normal prologue; it simply hands control to the
/// caller-supplied entry. `stack_bottom` is unused for now (the pico-sdk uses
/// it to install a stack guard — skipped here for the Phase-3a bring-up).
extern "C" fn core1_wrapper(entry: extern "C" fn() -> !, _stack_bottom: *mut usize) -> ! {
    entry()
}

/// Poll the inter-core FIFO for a value, giving up after `spins` iterations so
/// an unresponsive core 1 can't hang core 0 (the §9a failure mode).
fn read_with_timeout(fifo: &mut hal::sio::SioFifo, spins: u32) -> Option<u32> {
    for _ in 0..spins {
        if let Some(v) = fifo.read() {
            return Some(v);
        }
        hal::arch::nop();
    }
    None
}

/// Launch `entry` on core 1 (the second Hazard3 core).
///
/// `stack` is core 1's stack region; it must be `'static`, exclusively owned,
/// and at least 16-byte aligned at its top (a `#[repr(align(16))]` static
/// satisfies this). `entry` must never return.
///
/// # Safety
/// - Call exactly once, from core 0, before enabling any SIO-FIFO interrupt on
///   core 0 (the handshake assumes it owns the FIFO).
/// - `stack` must not alias any other live object and must outlive core 1.
pub unsafe fn launch_core1_riscv(
    psm: &mut hal::pac::PSM,
    fifo: &mut hal::sio::SioFifo,
    stack: &'static mut [usize],
    entry: extern "C" fn() -> !,
) -> Result<(), LaunchError> {
    // 1. Hard-reset core 1 into a known state. Reading `frce_off` back both
    //    confirms the reset took and fences any buffered APB writes (same as
    //    the HAL's `spawn` / pico-sdk `multicore_reset_core1`).
    psm.frce_off().modify(|_, w| w.proc1().set_bit());
    while !psm.frce_off().read().proc1().bit_is_set() {
        hal::arch::nop();
    }
    psm.frce_off().modify(|_, w| w.proc1().clear_bit());

    // 2. Capture the values core 1 needs but the bootrom won't set up: our
    //    trap vector (mtvec) and global pointer (gp).
    let mtvec: usize;
    let gp: usize;
    core::arch::asm!("csrr {}, mtvec", out(reg) mtvec, options(nomem, nostack, preserves_flags));
    core::arch::asm!("mv {}, gp", out(reg) gp, options(nomem, nostack, preserves_flags));

    // 3. Push the four trampoline words at the top of core 1's stack:
    //    [entry, stack_bottom, core1_wrapper, gp]. 16 bytes keeps the RV32
    //    16-byte sp alignment that the entry function's prologue expects.
    let base = stack.as_mut_ptr();
    let top = base.add(stack.len()); // one-past-end
    let sp = top.sub(4);
    sp.add(0).write(entry as usize);
    sp.add(1).write(base as usize);
    sp.add(2).write(core1_wrapper as *const () as usize);
    sp.add(3).write(gp);

    // Ensure the stack writes are emitted before the FIFO handshake below, so
    // core 1 observes them. RP2350 has no caches, so a compiler fence is
    // sufficient (no cache maintenance needed).
    compiler_fence(Ordering::Release);

    // 4. Drive the bootrom launch handshake. Each command is echoed by core 1;
    //    on a mismatch we restart the sequence, and after too many failures we
    //    bail rather than spin forever.
    let cmd_seq: [u32; 6] = [
        0,
        0,
        1,
        mtvec as u32,
        sp as u32,
        core1_trampoline as *const () as u32,
    ];

    // ~2M nop spins between echoes is multiple ms at 240 MHz — orders of
    // magnitude more than the bootrom's near-instant echo.
    const READ_SPINS: u32 = 2_000_000;
    const MAX_FAILS: u32 = 16;

    let mut seq = 0usize;
    let mut fails = 0u32;
    loop {
        let cmd = cmd_seq[seq];
        // Before sending a 0, flush any stale echo (incl. core 1's post-reset
        // ready signal) and wake core 1 in case it's blocked in `wfe`.
        if cmd == 0 {
            fifo.drain();
            hal::arch::sev();
        }
        fifo.write_blocking(cmd);

        match read_with_timeout(fifo, READ_SPINS) {
            Some(resp) if resp == cmd => {
                seq += 1;
                if seq >= cmd_seq.len() {
                    return Ok(());
                }
            }
            _ => {
                seq = 0;
                fails += 1;
                if fails > MAX_FAILS {
                    return Err(LaunchError::Unresponsive);
                }
            }
        }
    }
}
