//! PIO-side clock-recovery Manchester decoder (Phase 2d — DPLL, experimental).
//!
//! Replaces the open-loop CPU decoder's drift problem (A1) AND the per-edge
//! decoders' single-edge-slip problem (fixed-delay `[8]` and interval-
//! classifier, both ~554 B clean ceiling; Phase 2b/2c, see git history).
//!
//! **Sample-by-pin DPLL.** The crucial mechanism: the emitted bit value comes
//! from a direct pin sample (`in pins, 1`) taken at a fixed phase in the bit
//! period — NOT from an alternation/level state that previous decoders flipped
//! on a single bad edge. After detecting a mid-bit edge:
//!   - Cycles 1-4: delay (skip near-edge instability).
//!   - Cycle 5: `in pins, 1` reads the line level → emit bit (in the bit's 2nd
//!     half-bit; whatever the line level is, that's the data bit per IEEE 802.3
//!     Manchester — or its complement on this inverted-pin polarity; the host
//!     SFD-finder is bi-polarity, so emit polarity is don't-care).
//!   - Cycles 5-13: delay through the rest of the 2nd half and past the
//!     potential boundary edge at ~+7.5 cycles.
//!   - Cycle 14: `jmp pin` reads the *current* line level and branches into
//!     wait_high or wait_low — at cycle 14 the next mid-bit edge will transition
//!     the line to its opposite, regardless of whether there was a boundary
//!     edge in between (case-different: line still at V, next mid-bit flips it
//!     to !V; case-same: line at !V after boundary, next mid-bit flips it to V).
//!     Either way: wait for the OPPOSITE of the current level.
//!   - Cycle 15: `wait` catches the next mid-bit edge, jmp back to top.
//!
//! Loop is 15 SM cycles = exactly one bit period at 150 MHz, so the wait stalls
//! 0-1 cycles at the actual edge — self-clocked. **DPLL ride-through (P2):** a
//! single jitter-induced edge mis-catch shifts phase by ≤ a few cycles, but the
//! next bit's `in pins, 1` reads the actual line level — the bit is still right
//! if the sample lands anywhere in the right half-bit. No alternation state to
//! cascade-corrupt, unlike the per-edge decoders. Offline-validated as
//! `decode_dpll_model` in `tools/clock-recovery/harness.py` (holds full-MTU
//! lock on 2/3 corpus frames, no drift ramp / no tail cliff — see plan §12).
//!
//! 7 PIO1 instructions, no `out pc` (no `.origin 0` needed), no clock divider.

use rp235x_hal as hal;
use hal::pac::PIO1;
use hal::pio::{
    Buffers, Rx, Running, ShiftDirection, StateMachine, UninitStateMachine, SM0,
};

pub type DecRxFifo = Rx<(PIO1, SM0)>;

/// PIO1 SM0 clock-recovery DPLL decoder. Holds the running SM so it isn't dropped.
pub struct EthRxPio {
    _sm: StateMachine<(PIO1, SM0), Running>,
}

impl EthRxPio {
    /// Install + start the decoder on PIO1 SM0, sampling `rx_pin_id` (must
    /// already be assigned to a PIO function — GPIO inputs are visible to all
    /// PIO blocks, so PIO0's funcsel on RXD is fine). Returns the decoder
    /// handle and the RX FIFO (decoded-byte words, LSB-first).
    pub fn new(
        pio: &mut hal::pio::PIO<PIO1>,
        sm: UninitStateMachine<(PIO1, SM0)>,
        rx_pin_id: u8,
    ) -> (Self, DecRxFifo) {
        // Single-path loop (sample-by-pin makes the LOW/HIGH state split
        // unnecessary — bit value comes from in pins, 1, not from a code branch).
        // 7 instructions, total 15 SM cycles/iter at the steady-state bit period.
        let program = pio::pio_asm!(
            "sample_path:",
            "    nop          [2]",   // pre-sample delay (3 cyc total)
            "    in pins, 1   [7]",   // SAMPLE pin level (2nd half-bit); coast 7 cyc
            "    jmp pin, wait_low",  // read pin → choose wait direction
            // Loop totals exactly 15 SM cycles between wait completions = 1 bit
            // @150 MHz: jmp(1)+nop[2](3)+in[7](8)+jmp_pin(1)+wait(1, stall 1 to
            // catch edge)+jmp(1) = 16 with the wait stall — the wait STALLS until
            // the next mid-bit edge transitions the line and the synchronizer
            // delivers it, so the loop self-clocks to the actual bit rate.
            "wait_high:",
            "    wait 1 pin 0",       // wait for rising mid-bit edge
            "    jmp sample_path",
            "wait_low:",
            "    wait 0 pin 0",       // wait for falling mid-bit edge
            "    jmp sample_path",
        );

        let installed = pio.install(&program.program).unwrap();

        let (mut sm, rx, _tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .in_pin_base(rx_pin_id) // `in pins, 1` reads this pin
            .jmp_pin(rx_pin_id)     // `jmp pin` tests this pin's level
            .in_shift_direction(ShiftDirection::Right) // LSB-first, like frame bytes
            .autopush(true)
            .push_threshold(32)
            .buffers(Buffers::OnlyRx)
            .build(sm);

        // No clock divisor => SM runs at sysclk (150 MHz, ~15 cycles/bit).
        let _ = &mut sm;
        let sm = sm.start();
        (Self { _sm: sm }, rx)
    }
}
