//! PIO-side clock-recovery Manchester decoder (Phase 2b, experimental).
//!
//! Replaces the open-loop CPU decoder's drift problem (A1) by re-synchronising
//! to every Manchester transition in hardware. Single-pin design (only reads
//! RXD via `wait`/`jmp pin`, so transmit toggling GPIO14 can't introduce stray
//! edges). Validated as `decode_pio_model` in tools/clock-recovery offline.
//!
//! Algorithm (a two-state edge tracker, SM @ 150 MHz ≈ 15 cycles/bit):
//!   - LOW state: line is low; `wait 1 pin` blocks until the rising mid-bit
//!     edge, then emit a `0` bit (the pre-edge level).
//!   - HIGH state: line is high; `wait 0 pin` blocks until the falling edge,
//!     then emit a `1`.
//!   - After each edge: delay D cycles to step past the conditional boundary
//!     edge (~half a bit later), then resample the line (`jmp pin`) to pick the
//!     next state. Re-anchoring every bit means clock drift can't accumulate.
//!
//! Decoded bits autopush LSB-first into 32-bit words → RX FIFO. Idle (no edges)
//! blocks in `wait`, emitting nothing — frame gating is free. The CPU finds the
//! preamble/SFD in the decoded bitstream and checks FCS (cheap vs sample decode).

use rp235x_hal as hal;
use hal::pac::PIO1;
use hal::pio::{
    Buffers, Rx, Running, ShiftDirection, StateMachine, UninitStateMachine, SM0,
};

/// Boundary-skip delay in SM cycles (see module doc / docs/pio-decoder-plan.md).
/// 15 SM cycles/bit at 150 MHz. After catching a mid-bit edge, the resample
/// (`jmp pin`) lands at ~T+(D+2) and the wait re-arms at ~T+(D+4); BOTH must
/// fall in the next bit's first half-bit, between the boundary edge (~T+7.5)
/// and the next mid-bit edge (~T+15). Centring that pair ⇒ D+3 = 11.25 ⇒ D≈8.
/// On the wire (2026-05-27): D=8 decodes ~1000–1346 B of a full-MTU frame, then
/// loses lock into a steady 0xaa/0x55 run at a *varying* byte (jitter-limited —
/// the centring is right, the residual is edge jitter). D=9 was *worse*: a
/// *deterministic* slip at byte 960 (wait re-arm at T+13 misses early-jittered
/// mid-bit edges), confirming D=8 is the centre. The full-MTU tail slip is the
/// fixed-delay scheme's residual jitter margin; sub-~900 B frames decode clean.
/// (Hardcoded in the asm below — `pio_asm!` can't interpolate a Rust const.)
#[allow(dead_code)]
pub const SKIP_DELAY: u8 = 8;

pub type DecRxFifo = Rx<(PIO1, SM0)>;

/// PIO1 SM0 clock-recovery decoder. Holds the running SM so it isn't dropped.
pub struct EthRxPio {
    _sm: StateMachine<(PIO1, SM0), Running>,
}

impl EthRxPio {
    /// Install + start the decoder on PIO1 SM0, sampling `rx_pin_id` (must
    /// already be assigned to the PIO1 function). Returns the decoder handle
    /// and the RX FIFO (decoded-byte words, LSB-first).
    pub fn new(
        pio: &mut hal::pio::PIO<PIO1>,
        sm: UninitStateMachine<(PIO1, SM0)>,
        rx_pin_id: u8,
    ) -> (Self, DecRxFifo) {
        // Two-state edge tracker. `mov x, !null` seeds X = all-ones so the HIGH
        // path can shift a `1` bit via `in x, 1`; the LOW path shifts `0` via
        // `in null, 1`. `[SKIP_DELAY]` on each `in` steps past the boundary edge.
        let program = pio::pio_asm!(
            "    mov x, !null",
            "loop_low:",
            "    wait 1 pin 0",
            "    in null, 1   [8]",
            "    jmp pin, loop_high",
            "    jmp loop_low",
            "loop_high:",
            "    wait 0 pin 0",
            "    in x, 1      [8]",
            "    jmp pin, loop_high",
            "    jmp loop_low",
        );

        let installed = pio.install(&program.program).unwrap();

        let (mut sm, rx, _tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .in_pin_base(rx_pin_id) // `wait ... pin 0` => this GPIO
            .jmp_pin(rx_pin_id) // `jmp pin` => same GPIO (resample)
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
