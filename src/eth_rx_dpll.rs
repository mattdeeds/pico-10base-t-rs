//! Edge-track DPLL Manchester decoder (Phase 3b — CPU DPLL port).
//!
//! Rust port of `decode_edge_track` from `tools/clock-recovery/harness.py`,
//! validated against the corpus (FCS-OK N/N, flat per-byte error bins). The
//! decoder re-anchors to each per-bit Manchester transition (search ±W
//! samples around the expected mid-bit edge position), so accumulated clock
//! drift can't walk the sample point off the bit-centre — fixes the open-loop
//! decoder's A1 ramp-from-575 B failure mode.
//!
//! Sampler runs at 60 MHz (T = 6 samples/bit). Edge expected at
//! `F + 5 + 6·k` from the F=first-H→L anchor; data bit `k` is sampled one
//! sample BEFORE the resync'd edge (= `tr − 1`).
//!
//! Pure `no_std`, no allocator. Same I/O shape as `eth_rx::decode_frame` so
//! it can drop-replace the open-loop sampler in the IRQ handler later (Phase
//! 3b second half). Host-side validation via `tools/dpll-rust/`.

use heapless::Vec;

/// Same as `eth_mac::MAX_FRAME_BYTES`. Kept in sync by convention (1600 = full
/// MTU + slack). If the eth_mac constant changes, change this too.
pub const MAX_FRAME_BYTES: usize = 1600;

/// Edge-search half-window in samples. The Python harness validates the
/// algorithm at W=1; W=2 and W=3 also work (slightly more tolerant of jitter
/// at the cost of more reads per bit). 60 MHz sampling, T = 6 samples/bit,
/// so W ∈ {1,2} stays well under T/2.
const W: usize = 1;

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
/// consecutive *open-loop* data bits (sampled at the F+4+6k stride) are both 1.
/// Returns the bit-index of the second `1` (i.e. the last bit of the SFD).
/// Acquisition is still open-loop here — the edge-track DPLL kicks in for
/// the data region after the SFD.
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

/// Find the closest sample-to-sample transition within ±W of `center`. Mirrors
/// the Python `find_edge`: scan `[center-W .. center+W]`, return the index `i`
/// where `sample(i) != sample(i-1)` that's nearest to `center` (smaller offset
/// preferred; ties broken by `<`, matching Python's first-found-wins).
fn find_edge(buf: &[u8], center: usize, ns: usize) -> Option<usize> {
    let lo = if center > W { center - W } else { 1 };
    let lo = lo.max(1);
    let hi_inclusive = (center + W).min(ns.saturating_sub(1));
    if hi_inclusive < lo {
        return None;
    }
    let mut best: Option<usize> = None;
    let mut best_d = usize::MAX;
    for i in lo..=hi_inclusive {
        if sample_bit(buf, i) != sample_bit(buf, i - 1) {
            let d = i.abs_diff(center);
            if d < best_d {
                best = Some(i);
                best_d = d;
            }
        }
    }
    best
}

/// Edge-track DPLL decode. Returns the decoded frame bytes (LSB-first) up to
/// `MAX_FRAME_BYTES`, or `None` if F or SFD not found.
///
/// The decode steps:
/// 1. Open-loop F search (first H→L).
/// 2. Open-loop SFD search on the F+4+6k stride. The preamble is alternating
///    bits so no boundary edges → open-loop is reliable here.
/// 3. From the first data-bit edge onwards, *closed-loop*: search ±W around
///    each expected next edge (`tr + 6`), sample the bit one before the
///    detected edge, advance `tr`. Coast through a missed edge at the
///    nominal `tr + 6` so a single noisy bit doesn't desync the stream.
pub fn decode_frame_edge_track(buf: &[u8]) -> Option<Vec<u8, MAX_FRAME_BYTES>> {
    let ns = buf.len().checked_mul(8)?;
    let f = find_f(buf, ns)?;
    let sfd = find_sfd(buf, ns, f)?;
    let start = sfd + 1;
    const P: usize = 6;

    // First-data-bit edge anchor. The Python uses `find_edge(...) or center`
    // — if no edge found, coast to the nominal center.
    let initial_center = f.checked_add(5 + 6 * start)?;
    let mut tr = find_edge(buf, initial_center, ns).unwrap_or(initial_center);

    let mut frame: Vec<u8, MAX_FRAME_BYTES> = Vec::new();
    let mut byte: u8 = 0;
    let mut bit_idx: u8 = 0;

    loop {
        // Data bit for this position = sample one BEFORE the (resync'd) edge.
        if tr == 0 {
            break;
        }
        let si = tr - 1;
        if si >= ns {
            break;
        }
        let bit = sample_bit(buf, si);
        byte |= bit << bit_idx;
        bit_idx += 1;
        if bit_idx == 8 {
            if frame.push(byte).is_err() {
                break; // MAX_FRAME_BYTES cap reached
            }
            byte = 0;
            bit_idx = 0;
        }

        // Next mid-bit edge expected at tr + 6. Search ±W and coast through
        // a missed edge.
        let next_center = tr.checked_add(P)?;
        if next_center >= ns {
            break;
        }
        tr = find_edge(buf, next_center, ns).unwrap_or(next_center);
    }

    Some(frame)
}
