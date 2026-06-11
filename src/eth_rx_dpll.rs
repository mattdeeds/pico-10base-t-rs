//! Edge-track DPLL Manchester decoder (Phase 3b — CPU DPLL port, optimized).
//!
//! Rust port of `decode_edge_track` from `tools/clock-recovery/harness.py`.
//! Validated against the corpus (FCS-OK N/N, flat per-byte error bins). The
//! decoder re-anchors to each per-bit Manchester transition (search ±W
//! samples around the expected mid-bit edge position) so accumulated clock
//! drift can't walk the sample point off the bit-centre — fixes the open-loop
//! decoder's A1 ramp-from-575 B failure mode.
//!
//! Sampler runs at 60 MHz (T = 6 samples/bit). Edge expected at `F + 5 + 6·k`
//! from the F=first-H→L anchor; data bit `k` is sampled one sample BEFORE
//! the resync'd edge (= `tr − 1`).
//!
//! **Optimized for IRQ-budget fit** (Phase 3b second-pass, after the naive
//! port confirmed the budget concern on-wire at ~3-9 ms/frame). Applies the
//! same playbook the open-loop went through:
//! 1. `sample_bit` via `get_unchecked` after proving the upper-bound `ns`
//!    is in-bounds; this drops the bounds-check load per sample.
//! 2. `find_edge` inlined + unrolled for W=1 (4-sample slide-window check).
//! 3. Decode-length cap derived from the IP-header total-length once the
//!    first 18 bytes are decoded, so an over-long active run can't force a
//!    full-MAX_FRAME_BYTES decode.
//!
//! Pure `no_std`, no allocator. Same I/O shape as `eth_rx::decode_frame`.

use heapless::Vec;

/// Same as `eth_mac::MAX_FRAME_BYTES`. Kept in sync by convention (1600 bytes).
pub const MAX_FRAME_BYTES: usize = 1600;

/// Read 1 bit at `off` in LSB-first packed bytes. SAFETY: caller must ensure
/// `off >> 3 < buf.len()` (which holds when `off < buf.len() * 8`).
#[inline(always)]
unsafe fn sample_bit_unchecked(buf: &[u8], off: usize) -> u8 {
    let b = unsafe { *buf.get_unchecked(off >> 3) };
    (b >> (off & 7)) & 1
}

#[inline]
fn sample_bit(buf: &[u8], off: usize) -> u8 {
    (buf[off >> 3] >> (off & 7)) & 1
}

/// First H→L (1→0) edge in the sample stream — the F anchor used by the
/// open-loop decoder. Returns `None` if no falling edge in the window.
fn find_f(buf: &[u8], ns: usize) -> Option<usize> {
    let mut prev = sample_bit(buf, 0);
    for i in 1..ns {
        let s = sample_bit(buf, i);
        if prev == 1 && s == 0 {
            return Some(i);
        }
        prev = s;
    }
    None
}

/// Find the SFD (`...0xD5`) end inside the preamble: first place where two
/// consecutive open-loop data bits (sampled at F+4+6k) are both 1.
fn find_sfd(buf: &[u8], ns: usize, f: usize) -> Option<usize> {
    let read = |k: usize| -> Option<u8> {
        let idx = f + 4 + 6 * k;
        if idx < ns {
            Some(sample_bit(buf, idx))
        } else {
            None
        }
    };
    let mut prev = read(0)?;
    for k in 1..1600 {
        let c = read(k)?;
        if c == 1 && prev == 1 {
            return Some(k);
        }
        prev = c;
    }
    None
}

/// W=1 windowed edge search around `center`, unrolled (4-sample slide-window).
/// Returns the nearest edge to `center` (smaller distance wins; ties → lower
/// i, matching Python `find_edge`'s strict `<` tie-break), or `center` if no
/// edge in window (coast). Per the on-wire sweep, W=1/W=2/W=3 all gave the
/// same ~50 % full-MTU FCS-OK, so the failures aren't jitter beyond ±W —
/// W=1 is the cheapest within-budget choice.
/// SAFETY: caller must ensure `center >= 2 && center + 1 < ns_full`.
#[inline(always)]
unsafe fn find_edge_w1(buf: &[u8], center: usize) -> usize {
    unsafe {
        let s_m2 = sample_bit_unchecked(buf, center - 2);
        let s_m1 = sample_bit_unchecked(buf, center - 1);
        let s_0 = sample_bit_unchecked(buf, center);
        let s_p1 = sample_bit_unchecked(buf, center + 1);
        if s_m1 != s_0 {
            center // d=0
        } else if s_m2 != s_m1 {
            center - 1 // d=1, lower i
        } else if s_0 != s_p1 {
            center + 1 // d=1, higher i
        } else {
            center // coast
        }
    }
}

/// Branchless W=1 edge select, indexed by the 3 pairwise-difference bits of
/// the window `[c-2, c-1, c, c+1]`: bit0 = edge(c-2,c-1), bit1 = edge(c-1,c),
/// bit2 = edge(c,c+1). Priority identical to `find_edge_w1`: d=0 (bit1) wins,
/// then d=1-lower (bit0), then d=1-higher (bit2), else coast.
const EDGE_DELTA: [i32; 8] = [0, -1, 0, 0, 1, -1, 0, 0];

/// Edge-track DPLL decode. Returns the decoded frame bytes (LSB-first) up to
/// `MAX_FRAME_BYTES` (capped further by the IP-derived frame length once the
/// header is in), or `None` if F or SFD not found.
///
/// Hot loop (third-pass optimization, 2026-06-10): the W=1 edge window for
/// `next_center = tr+6` is 4 *contiguous* sample bits (`tr+4..=tr+7`), and the
/// resync'd data bit (`new_tr - 1` ∈ `tr+4..=tr+6`) always lands inside that
/// same window. So each bit needs ONE two-byte window load: edge select is an
/// XOR + 8-entry table on the window's pairwise-difference bits, and the data
/// bit is a shift of the same register — replacing the previous 5 separate
/// load+shift+mask sample reads per bit. The fast loop runs while the window
/// is provably in range (`tr + 20 <= ns_full`); the last few bits near the
/// buffer end fall through to the original per-sample loop, which keeps the
/// boundary-coast semantics exactly (matches the Python `find_edge` clamp).
pub fn decode_frame_edge_track(buf: &[u8]) -> Option<Vec<u8, MAX_FRAME_BYTES>> {
    let ns_full = buf.len().checked_mul(8)?;
    let f = find_f(buf, ns_full)?;
    let sfd = find_sfd(buf, ns_full, f)?;
    let start = sfd + 1;
    let initial_center = f.checked_add(5 + 6 * start)?;

    // Initial mid-bit-edge anchor.
    let mut tr = if initial_center >= 2 && initial_center + 1 < ns_full {
        // SAFETY: bounds proven directly above (W=1 needs center-2..center+1).
        unsafe { find_edge_w1(buf, initial_center) }
    } else {
        initial_center
    };

    let mut frame: Vec<u8, MAX_FRAME_BYTES> = Vec::new();
    let mut byte: u8 = 0;
    let mut bit_idx: u8 = 0;
    // Decode-length cap from the IP header — bounds the loop to the declared
    // frame length once we've decoded enough to read total-length. Starts at
    // MAX_FRAME_BYTES so the loop runs at least the header bytes.
    let mut cap_bytes: usize = MAX_FRAME_BYTES;

    // Fast path. Loop state at top: the data bit at `tr - 1` has NOT been
    // emitted yet (same as the slow loop below). `pending` carries that bit
    // when the previous iteration already extracted it from its window.
    let mut pending: Option<u32> = None;
    while tr >= 1 && tr + 20 <= ns_full {
        let bit = match pending {
            Some(b) => b as u8,
            // SAFETY: tr-1 < ns_full ⇒ in-bounds (guard above).
            None => unsafe { sample_bit_unchecked(buf, tr - 1) },
        };
        byte |= bit << bit_idx;
        bit_idx += 1;
        if bit_idx == 8 {
            if frame.push(byte).is_err() {
                return Some(frame);
            }
            byte = 0;
            bit_idx = 0;
            if frame.len() == 18 && cap_bytes == MAX_FRAME_BYTES {
                let f_slice = frame.as_slice();
                let ethertype = u16::from_be_bytes([f_slice[12], f_slice[13]]);
                if ethertype == 0x0800 {
                    let ip_total = u16::from_be_bytes([f_slice[16], f_slice[17]]) as usize;
                    let derived = (14 + ip_total + 4).max(64);
                    cap_bytes = derived.min(MAX_FRAME_BYTES);
                }
            }
            if frame.len() >= cap_bytes {
                return Some(frame);
            }
        }

        // Advance: window anchored at lo = tr+4 covers sample bits
        // lo..lo+3 = (nc-2)..(nc+1) for nc = tr+6. Guard proves
        // (tr+19) < ns_full ⇒ byte index (lo>>3)+1 <= (tr+12)>>3 < buf.len().
        let lo = tr + 4;
        let bi = lo >> 3;
        // SAFETY: bi+1 < buf.len() per the loop guard (see above).
        let w = unsafe {
            ((*buf.get_unchecked(bi) as u32) | ((*buf.get_unchecked(bi + 1) as u32) << 8))
                >> (lo & 7)
        };
        // Pairwise differences of [nc-2, nc-1, nc, nc+1] → 3-bit edge index.
        let e = ((w ^ (w >> 1)) & 7) as usize;
        let delta = EDGE_DELTA[e];
        tr = (tr as i32 + 6 + delta) as usize;
        // Data bit at new tr-1 = lo + (1+delta) — still inside the window.
        pending = Some((w >> (1 + delta) as u32) & 1);
    }

    // Tail: original per-sample loop, preserving the boundary-coast and
    // termination semantics for the last few bits near the buffer end.
    loop {
        // Sample the data bit one before the resync'd edge.
        if tr == 0 || tr > ns_full {
            break;
        }
        let bit = match pending.take() {
            Some(b) => b as u8,
            // SAFETY: tr-1 < ns_full ⇒ (tr-1) >> 3 < buf.len().
            None => unsafe { sample_bit_unchecked(buf, tr - 1) },
        };
        byte |= bit << bit_idx;
        bit_idx += 1;
        if bit_idx == 8 {
            if frame.push(byte).is_err() {
                break;
            }
            byte = 0;
            bit_idx = 0;
            // Right after the full header is in (14 eth + 4 IP-start = 18),
            // derive the frame-length cap. Same logic as the open-loop
            // decoder's `derive_frame_len` and the Python `fcs_ok`.
            if frame.len() == 18 && cap_bytes == MAX_FRAME_BYTES {
                let f_slice = frame.as_slice();
                let ethertype = u16::from_be_bytes([f_slice[12], f_slice[13]]);
                if ethertype == 0x0800 {
                    let ip_total = u16::from_be_bytes([f_slice[16], f_slice[17]]) as usize;
                    let derived = (14 + ip_total + 4).max(64);
                    cap_bytes = derived.min(MAX_FRAME_BYTES);
                }
            }
            if frame.len() >= cap_bytes {
                break;
            }
        }

        // Find next mid-bit edge at tr + 6, W=1 window. Near the buffer
        // boundary the window is partly out of range — coast (use center
        // unchanged) instead of breaking, so the loop's `tr - 1 < ns_full`
        // termination at the top picks up the very last bit. (Matches the
        // Python `find_edge`'s `hi = min(ns-1, center+W)` clamp behaviour.)
        let next_center = match tr.checked_add(6) {
            Some(v) => v,
            None => break,
        };
        if next_center < 2 || next_center + 1 >= ns_full {
            tr = next_center;
        } else {
            // SAFETY: bounds proven directly above (W=1 needs center-2..center+1).
            tr = unsafe { find_edge_w1(buf, next_center) };
        }
    }

    Some(frame)
}
