//! PIO-side clock-recovery Manchester decoder (Phase 2d v2 — windowed DPLL).
//!
//! v1 (sample-by-pin, see git history) decoded valid Manchester (preamble +
//! 56–141 payload bytes byte-perfect on wire) but slipped on a single
//! jittered/noise edge because the `wait` had no phase window — it caught the
//! first edge after a fixed coast. v2 ADDS the windowing half of the design:
//! the wait is replaced by an explicit poll loop with a phase counter (`set x`
//! + `jmp x--`). The loop polls for the expected mid-bit edge only in a small
//! cycle window around its predicted arrival time; if no edge is found, the
//! program COASTS one bit period (free-runs, preserving the bit clock from the
//! prior edge) instead of resync-ing to whatever stray edge appeared next.
//!
//! No per-edge alternation state to flip — combined with `in pins, 1` sampling
//! it gives the offline-validated `decode_dpll_model` behavior: a single bad
//! edge can shift phase by at most a window-width's worth, but the next bit's
//! sample still reads the actual line level, so a bit slip stays local instead
//! of cascading.
//!
//! Cycle plan (bit period = 15 SM cycles at 150 MHz, anchor at cycle 0 = just
//! after a detected mid-bit edge):
//!   - cycles 1–4 : pre-sample coast
//!   - cycle 5    : `in pins, 1` SAMPLE → emit bit (level-based, not state)
//!   - cycles 6–9 : post-sample coast (through boundary edge region ~+7.5)
//!   - cycle 10   : `jmp pin` reads current line level → pick window path
//!   - cycle 11   : `set x` loads the window counter
//!   - cycles 12+ : poll loop (2 cyc/iter via `jmp pin` + `jmp x--`); accepts
//!                  an edge within ~±jitter of the expected +15 PIO cycle.
//!   - on edge found → `jmp new_bit` resync (anchor moves to detected edge).
//!   - on window timeout → coast routine free-runs to next bit's window
//!                          (preserves the bit clock from the last good edge).

use rp235x_hal as hal;
use hal::pac::PIO1;
use hal::pio::{
    Buffers, Rx, Running, ShiftDirection, StateMachine, UninitStateMachine, SM0,
};

pub type DecRxFifo = Rx<(PIO1, SM0)>;

/// PIO1 SM0 windowed-DPLL Manchester decoder. Holds the running SM.
pub struct EthRxPio {
    _sm: StateMachine<(PIO1, SM0), Running>,
}

impl EthRxPio {
    pub fn new(
        pio: &mut hal::pio::PIO<PIO1>,
        sm: UninitStateMachine<(PIO1, SM0)>,
        rx_pin_id: u8,
    ) -> (Self, DecRxFifo) {
        // Windowed-DPLL Manchester decoder. Two-path edge detection (rising
        // mid-bit when line is currently LOW; falling when HIGH — picked by
        // `jmp pin` at cycle 10). Each polling iteration is 2 cycles: `jmp pin`
        // checks the awaited edge condition; `jmp x--` decrements the window
        // counter. X=2 ⇒ 3 polls @ cycles 12, 14, 16 — a 5-cycle window around
        // the expected PIO cycle 15 (where the next mid-bit edge becomes visible
        // through the 2-cycle input synchronizer when the real bit period is 15
        // SM cycles). The coast routine after a missed edge nudges the loop
        // toward the next expected window without resync-ing the phase.
        let program = pio::pio_asm!(
            "sample_path:",
            "    nop          [2]",          // cycles 1-3
            "    in pins, 1   [4]",          // cycle 4 SAMPLE; 4 delay → ends cyc 9
            "    jmp pin, w_low",            // cycle 10: pick window direction
            // ----- window: line was LOW, waiting for RISING mid-bit edge -----
            "w_high:",
            "    set x, 2",                  // cycle 11: 3 polls
            "poll_h:",
            "    jmp pin, sync_edge",        // even cycles: check pin HIGH
            "    jmp x--, poll_h",           // odd cycles: dec & retry
            "    jmp coast_miss",            // window elapsed
            // ----- window: line was HIGH, waiting for FALLING mid-bit edge -----
            "w_low:",
            "    set x, 2",
            "poll_l:",
            "    jmp pin, still_h",          // pin HIGH = no edge yet
            "    jmp sync_edge",             // pin LOW = falling edge caught
            "still_h:",
            "    jmp x--, poll_l",
            "    jmp coast_miss",
            // ----- edge found: resync (anchor → detected edge) -----
            "sync_edge:",
            "    jmp sample_path",
            // ----- no edge in window: coast one bit, preserve phase -----
            "coast_miss:",
            "    nop          [4]",          // free-run a fraction of a bit
            "    jmp sample_path",
        );

        let installed = pio.install(&program.program).unwrap();

        let (mut sm, rx, _tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .in_pin_base(rx_pin_id)
            .jmp_pin(rx_pin_id)
            .in_shift_direction(ShiftDirection::Right)
            .autopush(true)
            .push_threshold(32)
            .buffers(Buffers::OnlyRx)
            .build(sm);

        let _ = &mut sm;
        let sm = sm.start();
        (Self { _sm: sm }, rx)
    }
}
