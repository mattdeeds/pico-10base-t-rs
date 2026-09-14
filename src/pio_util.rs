//! PIO helpers shared by TX and RX.

/// Fixed-point PIO divider `(int, frac/256)` from `sys_clk_hz` to `target_hz`.
#[inline]
pub fn clock_divider(sys_clk_hz: u32, target_hz: f32) -> (u16, u8) {
    let div = sys_clk_hz as f32 / target_hz;
    let div_int = div as u16;
    let div_frac = ((div - div_int as f32) * 256.0) as u8;
    (div_int, div_frac)
}
