//! Edge-track DPLL Manchester decoder.
//!
//! Re-anchors to each mid-bit edge (±1 sample) so clock drift can't walk the
//! sample point off center. At 60 MHz the edge is expected at `F + 5 + 6k`;
//! each data bit is sampled one before its edge.
//! Port of `decode_edge_track` in `tools/clock-recovery/harness.py`.

use heapless::Vec;

/// Must match `eth_mac::MAX_FRAME_BYTES`.
pub const MAX_FRAME_BYTES: usize = 1600;

/// Bit at `off`, LSB-first. SAFETY: caller ensures `off >> 3 < buf.len()`.
#[inline(always)]
unsafe fn sample_bit_unchecked(buf: &[u8], off: usize) -> u8 {
    let b = unsafe { *buf.get_unchecked(off >> 3) };
    (b >> (off & 7)) & 1
}

#[inline]
fn sample_bit(buf: &[u8], off: usize) -> u8 {
    (buf[off >> 3] >> (off & 7)) & 1
}

/// First H→L edge (the F anchor), if any.
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

/// End of the SFD: first two consecutive 1 bits sampled at F+4+6k.
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

/// Nearest edge to `center` within ±1 sample, ties low; `center` if none.
/// SAFETY: caller ensures `center >= 2 && center + 1 < ns_full`.
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

/// Branchless `find_edge_w1`: edge offset indexed by 3 pairwise-difference bits.
const EDGE_DELTA: [i32; 8] = [0, -1, 0, 0, 1, -1, 0, 0];

/// Decode a frame from `buf`: bytes LSB-first, capped by IP length.
/// `None` if F or SFD isn't found.
///
/// The fast path loads each bit's 4-sample window once. The last bits near
/// the buffer end use the per-sample loop to keep boundary coasting exact.
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
    // Length cap from the IP header; starts at the max.
    let mut cap_bytes: usize = MAX_FRAME_BYTES;

    // Fast path. `pending` holds the next data bit if already extracted.
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

        // Window lo..lo+3 = (nc-2)..(nc+1), nc = tr+6. The loop guard keeps
        // byte (lo>>3)+1 in bounds.
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

    // Tail: per-sample loop for the last bits near the buffer end.
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
            // Header in: derive the length cap.
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

        // Next edge near tr + 6. Coast near the buffer end so the last bit
        // still decodes.
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
