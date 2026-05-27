//! Small PIO helpers shared between the TX and RX state machines.

/// Derive the `(int, frac)` fixed-point clock divider that takes `sys_clk_hz`
/// down to `target_hz`. `frac` is in 1/256ths, matching
/// `clock_divisor_fixed_point`. Both TX (20 MHz half-bit) and RX (60 MHz
/// sampler) use this; at sys_clk = 150 MHz that's 7.5 and 2.5 respectively,
/// each with ±3.3 ns jitter — well within 10BASE-T tolerance.
#[inline]
pub fn clock_divider(sys_clk_hz: u32, target_hz: f32) -> (u16, u8) {
    let div = sys_clk_hz as f32 / target_hz;
    let div_int = div as u16;
    let div_frac = ((div - div_int as f32) * 256.0) as u8;
    (div_int, div_frac)
}
