//! Core-1 launch for Hazard3 (RISC-V).
//!
//! `rp235x-hal`'s `Multicore::spawn` is Cortex-M only (ACTLR/VTOR fault here).
//! This drives the bootrom FIFO protocol directly (RP2350 datasheet §5.3):
//!
//! - The vector table word is our `mtvec`, shared with core 0.
//! - A naked trampoline restores `gp`, since the bootrom skips `_start`.
//! - No ACTLR setup: Hazard3 has no caches and has the A extension.
//! - The handshake polls with a timeout instead of blocking forever.

use core::sync::atomic::{compiler_fence, Ordering};

use rp235x_hal as hal;

/// Why a core-1 launch failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaunchError {
    /// Core 1 never echoed the launch handshake within the retry budget.
    Unresponsive,
}

// Naked trampoline, as in pico-sdk's `core1_trampoline`. Pops
// a0=entry, a1=stack_bottom, a2=core1_wrapper, gp, then `jr a2`.
// No calls or globals until `gp` is restored.
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

/// Entered from the trampoline with `gp` valid. `stack_bottom` is unused
/// (no stack guard).
extern "C" fn core1_wrapper(entry: extern "C" fn() -> !, _stack_bottom: *mut usize) -> ! {
    entry()
}

/// Read the FIFO, giving up after `spins` polls.
fn read_with_timeout(fifo: &mut hal::sio::SioFifo, spins: u32) -> Option<u32> {
    for _ in 0..spins {
        if let Some(v) = fifo.read() {
            return Some(v);
        }
        hal::arch::nop();
    }
    None
}

/// Launch `entry` on core 1. `stack` must be `'static`, exclusive, and
/// 16-byte aligned at its top. `entry` must never return.
///
/// # Safety
/// - Call once, from core 0, before enabling any SIO-FIFO interrupt on core 0.
/// - `stack` must not alias any live object and must outlive core 1.
pub unsafe fn launch_core1_riscv(
    psm: &mut hal::pac::PSM,
    fifo: &mut hal::sio::SioFifo,
    stack: &'static mut [usize],
    entry: extern "C" fn() -> !,
) -> Result<(), LaunchError> {
    // 1. Reset core 1. Reading `frce_off` back confirms it.
    psm.frce_off().modify(|_, w| w.proc1().set_bit());
    while !psm.frce_off().read().proc1().bit_is_set() {
        hal::arch::nop();
    }
    psm.frce_off().modify(|_, w| w.proc1().clear_bit());

    // 2. Capture `mtvec` and `gp`; the bootrom won't set them.
    let mtvec: usize;
    let gp: usize;
    core::arch::asm!("csrr {}, mtvec", out(reg) mtvec, options(nomem, nostack, preserves_flags));
    core::arch::asm!("mv {}, gp", out(reg) gp, options(nomem, nostack, preserves_flags));

    // 3. Push [entry, stack_bottom, core1_wrapper, gp]; 16 bytes keeps sp aligned.
    let base = stack.as_mut_ptr();
    let top = base.add(stack.len()); // one-past-end
    let sp = top.sub(4);
    sp.add(0).write(entry as usize);
    sp.add(1).write(base as usize);
    sp.add(2).write(core1_wrapper as *const () as usize);
    sp.add(3).write(gp);

    // Emit the stack writes before the handshake (no caches, so a fence suffices).
    compiler_fence(Ordering::Release);

    // 4. Bootrom handshake: core 1 echoes each word; restart on mismatch.
    let cmd_seq: [u32; 6] = [
        0,
        0,
        1,
        mtvec as u32,
        sp as u32,
        core1_trampoline as *const () as u32,
    ];

    // Several ms per echo at 240 MHz; the bootrom replies near-instantly.
    const READ_SPINS: u32 = 2_000_000;
    const MAX_FAILS: u32 = 16;

    let mut seq = 0usize;
    let mut fails = 0u32;
    loop {
        let cmd = cmd_seq[seq];
        // Before each 0: flush stale echoes and wake core 1 from `wfe`.
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
