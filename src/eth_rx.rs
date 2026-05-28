//! 10BASE-T Ethernet RX over PIO + DMA double-buffer.
//!
//! Ports `src/rx_10base_t.pio` and (in later phases) the decoder from
//! `src/eth_rx.c` in the C reference repo.
//!
//! Layer 1: a continuous PIO sampler — `in pins, 1` at 60 MHz on the
//! ISL3177E RO pin (= GP13). Autopush 32 bits per FIFO word, LSB-first.
//! 60 MHz = 3 samples per Manchester half-bit, matching the C version.
//!
//! Layer 2 (this phase, R3.3): DMA writes the PIO RX FIFO into two
//! `[u32; 4096]` half-buffers in turn, chained so that when one fills the
//! other automatically starts. `poll_with` returns the just-completed half
//! to the caller as a `&[u8]` and immediately re-arms it. Caller must poll
//! at least every 2.18 ms (= one half-fill time) or samples drop.

use rp235x_hal as hal;
use hal::dma::{double_buffer, Channel, SingleChannel, CH0, CH1};
use hal::pac::PIO0;
use hal::pio::{
    Buffers, PinDir, Rx, Running, ShiftDirection, StateMachine, UninitStateMachine, SM1,
};

use crate::eth_mac::MAX_FRAME_BYTES;

/// Sample rate — 3 samples per Manchester half-bit (half-bit = 50 ns @ 10 Mbps).
pub const SAMPLE_HZ: u32 = 60_000_000;

/// Words per half-buffer. 4096 u32 = 16 KB = 16384 bytes = ~2.18 ms of audio.
pub const BUF_WORDS: usize = 4096;
/// Bytes per half-buffer.
pub const BUF_BYTES: usize = BUF_WORDS * 4;

/// Maximum trailing-active bytes we carry from one DMA half into the next so
/// frames straddling the boundary aren't truncated. A max-sized Ethernet
/// frame (1518 bytes) + preamble/SFD (8) = 1526 data bytes × 6 sample-bytes
/// per data byte ≈ 9.2 KB on the wire. 16 KB gives ample slack for TP_IDL
/// straggle + a few NLPs trailing into the carry. Bumped from 12 KB after
/// post-R5 telemetry suggested a small fraction of frames were getting their
/// preamble clipped at the cap (see `carry_capped` on `EthRx`).
pub const MAX_CARRY_BYTES: usize = 16 * 1024;

/// Stitch buffer = prior half's trailing active tail + the just-finished half.
/// Caller sees one contiguous slice across the boundary.
pub const STITCH_BUF_BYTES: usize = BUF_BYTES + MAX_CARRY_BYTES;

/// Upper bound on the SFD search in `decode_frame` — preserves the historical
/// 1600-bit extraction cap. The SFD normally appears within the first ~64 data
/// bits (7-byte preamble + SFD); this only bounds the pathological no-SFD case
/// so a noise run can't spin the search unboundedly.
const SFD_SEARCH_BITS: usize = 1600;

type RxFifo = Rx<(PIO0, SM1)>;
pub type RxBuf = &'static mut [u32; BUF_WORDS];
pub type CarryBuf = &'static mut [u8; MAX_CARRY_BYTES];
pub type StitchBuf = &'static mut [u8; STITCH_BUF_BYTES];
type Xfer = double_buffer::Transfer<
    Channel<CH0>,
    Channel<CH1>,
    RxFifo,
    RxBuf,
    double_buffer::WriteNext<RxBuf>,
>;

/// Read one sample bit out of the packed PIO buffer. `bit_offset` is the
/// absolute bit index from the start of `bytes`; PIO autopush packs samples
/// LSB-first within each byte (sample 0 → bit 0 of byte 0).
#[inline]
fn sample_bit(bytes: &[u8], bit_offset: usize) -> u8 {
    (bytes[bit_offset >> 3] >> (bit_offset & 7)) & 1
}

/// Find F — the first H→L transition within the first `nsamples` samples of
/// the run, as a sample offset from `base_bit`. F marks the start of
/// half-bit 0. Returns `None` if no falling edge is present. Shared by
/// `decode_frame` and `peek_dst_mac`.
#[inline]
fn find_first_falling_edge(bytes: &[u8], base_bit: usize, nsamples: usize) -> Option<usize> {
    let mut prev = sample_bit(bytes, base_bit);
    for i in 1..nsamples {
        let s = sample_bit(bytes, base_bit + i);
        if prev == 1 && s == 0 {
            return Some(i);
        }
        prev = s;
    }
    None
}

/// The phase-locked data bit at logical half-bit index `k`: the sample at
/// `F + 4 + 6k` (the midpoint of the second half-bit of Manchester pair k —
/// 3 samples per half-bit). Returns `None` once that sample would fall past
/// `nsamples`, i.e. the run ran out before bit `k`. This is the per-bit
/// primitive the single-pass decoder reads on demand, instead of
/// materializing every bit into an intermediate `Vec`.
#[inline]
fn data_bit(bytes: &[u8], base_bit: usize, f: usize, k: usize, nsamples: usize) -> Option<u8> {
    let idx = f + 4 + 6 * k;
    if idx >= nsamples {
        None
    } else {
        Some(sample_bit(bytes, base_bit + idx))
    }
}

/// Locate the SFD end: the index of the *second* `1` in the first `1,1`
/// data-bit pair (the trailing two bits of the 0xD5 SFD byte, LSB-first).
/// Reads data bits on demand up to `max_bits`, stopping early if the run's
/// samples are exhausted. Frame data starts at the next bit (`sfd_end + 1`).
/// Returns `None` if no SFD pair is found within the searched window.
#[inline]
fn find_sfd_end(
    bytes: &[u8],
    base_bit: usize,
    f: usize,
    nsamples: usize,
    max_bits: usize,
) -> Option<usize> {
    let mut prev = data_bit(bytes, base_bit, f, 0, nsamples)?;
    for k in 1..max_bits {
        let cur = data_bit(bytes, base_bit, f, k, nsamples)?;
        if cur == 1 && prev == 1 {
            return Some(k);
        }
        prev = cur;
    }
    None
}

/// PIO RX + DMA double-buffer state. Holds the running SM (so it isn't
/// dropped), and the current `WriteNext` transfer wrapped in an `Option`
/// so `poll_with` can `take()` it for the wait/re-arm cycle.
///
/// `carry` holds the trailing active bytes of the previous half so a frame
/// straddling the half boundary survives across DMA swap. `stitch` is the
/// scratch the carry + current half get concatenated into before the
/// caller's decoder scans it.
pub struct EthRx {
    _sm: StateMachine<(PIO0, SM1), Running>,
    xfer: Option<Xfer>,
    carry: CarryBuf,
    carry_len: usize,
    stitch: StitchBuf,
    /// Number of times the trailing-active walkback in `poll_with` hit the
    /// MAX_CARRY_BYTES cap (vs. terminating on a non-active byte). Each
    /// occurrence is a frame whose start got clipped — read via
    /// `take_carry_capped()` to monitor + tune the carry budget.
    pub carry_capped: u32,
}

impl EthRx {
    /// Install the RX PIO program on PIO0 SM1, start it sampling `rx_pin_id`,
    /// and arm the DMA double-buffer between the PIO RX FIFO and the two
    /// caller-provided buffers. Caller must have already reassigned the GPIO
    /// to PIO0 function.
    pub fn new(
        pio: &mut hal::pio::PIO<PIO0>,
        sm: UninitStateMachine<(PIO0, SM1)>,
        rx_pin_id: u8,
        sys_clk_hz: u32,
        mut dma_ch_a: Channel<CH0>,
        mut dma_ch_b: Channel<CH1>,
        buf_a: RxBuf,
        buf_b: RxBuf,
        carry: CarryBuf,
        stitch: StitchBuf,
    ) -> Self {
        let program = pio::pio_asm!(".wrap_target", "    in pins, 1", ".wrap",);

        let installed = pio.install(&program.program).unwrap();

        // 60 MHz from sys_clk_hz. At sys_clk=150 MHz that's div=2.5
        // (int=2, frac=128/256). ~3.3 ns jitter — well within tolerance.
        let (div_int, div_frac) = crate::pio_util::clock_divider(sys_clk_hz, SAMPLE_HZ as f32);

        let (mut sm, rx, _tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .in_pin_base(rx_pin_id)
            .in_shift_direction(ShiftDirection::Right)
            .autopush(true)
            .push_threshold(32)
            .clock_divisor_fixed_point(div_int, div_frac)
            .buffers(Buffers::OnlyRx)
            .build(sm);

        sm.set_pindirs([(rx_pin_id, PinDir::Input)]);
        let sm = sm.start();

        // Enable per-channel DMA_IRQ_0 BEFORE handing channels off — once
        // the channels are consumed by `Config::new` they're only reachable
        // through the Transfer's delegating `check_irq0` (which clears the
        // active-channel pending bit, but doesn't set the enable bit). The
        // enable bit is persistent across chain swaps.
        dma_ch_a.enable_irq0();
        dma_ch_b.enable_irq0();

        // Start ch_a writing buf_a, then arm ch_b with buf_b. The HAL chains
        // ch_a → ch_b so when ch_a completes, ch_b starts automatically.
        let xfer = double_buffer::Config::new((dma_ch_a, dma_ch_b), rx, buf_a).start();
        let xfer = xfer.write_next(buf_b);

        Self {
            _sm: sm,
            xfer: Some(xfer),
            carry,
            carry_len: 0,
            stitch,
            carry_capped: 0,
        }
    }

    /// Read + reset `carry_capped`. Wraps the typical "snapshot every 1 s
    /// for the log line" usage so callers don't have to remember to reset.
    pub fn take_carry_capped(&mut self) -> u32 {
        let v = self.carry_capped;
        self.carry_capped = 0;
        v
    }

    /// Check + clear the active channel's DMA_IRQ_0 pending bit. Returns
    /// true if the just-completed half generated the interrupt we're in
    /// (false → stale or someone else's DMA channel sharing the line).
    /// Called from the DMA_IRQ_0 handler before `poll_with`.
    pub fn dma_irq_pending(&mut self) -> bool {
        self.xfer
            .as_mut()
            .map(|x| x.check_irq0())
            .unwrap_or(false)
    }

    /// Find the next active run (bytes that are neither 0x00 nor 0xFF) of
    /// length ≥ `min_len`, starting at or after `start`. Skips any runs
    /// shorter than `min_len` (NLPs / noise). The DMA_IRQ_0 handler
    /// (`EthRxShared::process_completed_half`) calls this in a loop to walk
    /// every frame-shaped run in a stitched buffer, not just the longest
    /// one — fixes loss when two frames land in the same DMA half.
    pub fn find_active_run_from(
        bytes: &[u8],
        start: usize,
        min_len: usize,
    ) -> Option<(usize, usize)> {
        let mut i = start;
        loop {
            while i < bytes.len() && (bytes[i] == 0x00 || bytes[i] == 0xFF) {
                i += 1;
            }
            if i >= bytes.len() {
                return None;
            }
            let run_start = i;
            while i < bytes.len() && bytes[i] != 0x00 && bytes[i] != 0xFF {
                i += 1;
            }
            let run_len = i - run_start;
            if run_len >= min_len {
                return Some((run_start, run_len));
            }
            if i >= bytes.len() {
                return None;
            }
        }
    }

    /// Decode just the destination MAC of a frame-shaped active run —
    /// stops as soon as the 6 dst-MAC bytes are recovered. Same phase-lock
    /// and SFD-find logic as [`decode_frame`], but capped at ~200 bits
    /// (preamble + SFD slack + 48 MAC bits) and uses a fixed stack array
    /// instead of allocating a `Vec`. Cost is ~1–2 µs per call vs ~10 µs
    /// for a full `decode_frame`, so the IRQ-side MAC filter can skip the
    /// expensive full decode + CRC + inbox push for frames not addressed
    /// to us. Returns `None` if F or SFD couldn't be located, or if there
    /// weren't enough samples to recover all 6 bytes.
    pub fn peek_dst_mac(bytes: &[u8], base: usize, nbytes: usize) -> Option<[u8; 6]> {
        let nsamples = nbytes * 8;
        let base_bit = base * 8;

        let f = find_first_falling_edge(bytes, base_bit, nsamples)?;

        // Cap the SFD search at 200 bits: 56 preamble + 8 SFD + 48 MAC = 112
        // minimum, but the SFD can appear later if the first H→L wasn't
        // exactly at HB[0]. 200 gives slack without wasting work.
        const MAX_BITS: usize = 200;
        let sfd_end = find_sfd_end(bytes, base_bit, f, nsamples, MAX_BITS)?;

        // The 48 dst-MAC bits must fit within the same 200-bit window
        // (matches the old `start_bit + 48 > nbits` guard).
        let start_bit = sfd_end + 1;
        if start_bit + 48 > MAX_BITS {
            return None;
        }
        let mut mac = [0u8; 6];
        for (i, slot) in mac.iter_mut().enumerate() {
            let mut b: u8 = 0;
            for j in 0..8 {
                let k = start_bit + i * 8 + j;
                b |= data_bit(bytes, base_bit, f, k, nsamples)? << j;
            }
            *slot = b;
        }
        Some(mac)
    }

    /// Phase-lock + Manchester-decode + SFD-align a frame-shaped active
    /// run in `bytes`. `base` is the byte offset of the run within `bytes`,
    /// `nbytes` its length. Returns the unverified post-SFD frame bytes
    /// (CRC verification happens in [`verify_fcs`] / R3.6).
    ///
    /// Algorithm — see `eth_rx_decode_frame` in `../Pico-10BASE-T/src/eth_rx.c`:
    /// 1. Find F = first H→L transition in the run = start of HB[0].
    /// 2. Data bit k value = sample at F + 4 + 6k (3 samples per half-bit,
    ///    so the midpoint of HB[2k+1] is sample 4 + 6k from F).
    /// 3. SFD = first `1,1` pair in the decoded bit stream — the last two
    ///    bits of the 0xD5 SFD byte (LSB-first).
    /// 4. Pack post-SFD bits LSB-first straight into frame bytes — single
    ///    pass, no intermediate bit `Vec`. The packer is the decode hot path
    ///    (~71% of the cost — see RESUME perf notes), so it hoists the
    ///    sample-availability bound out of the loop (whole-byte count up
    ///    front), strides the sample offset by 6 instead of recomputing
    ///    `f + 4 + 6k` per bit, and reads the packed buffer unchecked over a
    ///    range proven in-bounds. Output is sized to `MAX_FRAME_BYTES` (a
    ///    full 1518-byte frame), so unlike the old two-pass version it does
    ///    not truncate frames past ~199 bytes — but it *is* bounded by the
    ///    header-declared frame length (see inline) so an over-long active
    ///    run can't force a full-buffer decode.
    pub fn decode_frame(
        bytes: &[u8],
        base: usize,
        nbytes: usize,
    ) -> Option<heapless::Vec<u8, MAX_FRAME_BYTES>> {
        let base_bit = base * 8;
        // Clamp samples to the buffer so the unchecked reads in the packer
        // are sound even if a caller passes nbytes past the slice end. For
        // valid runs (base + nbytes <= bytes.len()) this is a no-op.
        let buf_bits = bytes.len() * 8;
        if base_bit >= buf_bits {
            return None;
        }
        let nsamples = (nbytes * 8).min(buf_bits - base_bit);

        let f = find_first_falling_edge(bytes, base_bit, nsamples)?;
        let sfd_end = find_sfd_end(bytes, base_bit, f, nsamples, SFD_SEARCH_BITS)?;

        // Data bit m of the frame (m = 0 at start_bit) is the sample at
        // absolute offset `first_off + 6*m`. Compute how many *whole* frame
        // bytes the run can supply once, up front — dropping any partial
        // trailing byte matches the old `avail / 8` truncation — then pack
        // with a striding offset and no per-bit bound check or `Option`.
        let start_bit = sfd_end + 1;
        let limit = base_bit + nsamples;
        let first_off = base_bit + f + 4 + 6 * start_bit;
        let avail_bits = if first_off < limit {
            (limit - 1 - first_off) / 6 + 1
        } else {
            0
        };
        let nframe_avail = (avail_bits / 8).min(MAX_FRAME_BYTES);

        // Decode-length cap: pack the 18-byte header (14 Ethernet + 4 IPv4
        // ver..total-len), then bound the rest to the length the header
        // *declares* rather than however long the active run happens to be.
        // Keeps a long run (merged frames / noise) from costing a full
        // MAX_FRAME_BYTES decode — the old two-pass `for j in 0..1600` used to
        // bound this accidentally. Behaviour-preserving: a normal run is ~its
        // own frame length, and `verify_fcs`/`derive_frame_len` already use
        // the same declared length, so the decoded bytes the caller keeps are
        // identical — only trailing bytes past the frame are no longer packed.
        // Unknown EtherTypes can't be bounded from the header → uncapped.
        const HDR_BYTES: usize = 18;
        let mut nframe = nframe_avail;

        let mut frame: heapless::Vec<u8, MAX_FRAME_BYTES> = heapless::Vec::new();
        let mut off = first_off;
        let mut i = 0;
        while i < nframe {
            let mut byte: u8 = 0;
            for j in 0..8 {
                // SAFETY: nframe <= nframe_avail, so the last bit packed is at
                // off <= first_off + 6*(nframe_avail*8 - 1) < limit =
                // base_bit + nsamples <= buf_bits (nsamples is clamped to the
                // buffer), so off >> 3 < bytes.len() for every read.
                let s = (unsafe { *bytes.get_unchecked(off >> 3) } >> (off & 7)) & 1;
                byte |= s << j;
                off += 6;
            }
            // nframe <= MAX_FRAME_BYTES == capacity, so this never fails.
            let _ = frame.push(byte);
            i += 1;
            if i == HDR_BYTES {
                let etype = u16::from_be_bytes([frame[12], frame[13]]);
                let declared = match etype {
                    0x0800 => {
                        (14 + u16::from_be_bytes([frame[16], frame[17]]) as usize + 4).max(64)
                    }
                    0x0806 => 64,
                    _ => nframe_avail, // unknown — leave uncapped
                };
                nframe = declared.min(nframe_avail);
            }
        }
        Some(frame)
    }

    /// Best-effort frame length from EtherType + IP header. Returns the
    /// total frame length *including* the 4-byte FCS, so the caller can
    /// CRC over `frame[..frame_len - 4]`.
    ///
    /// IEEE 802.3 minimum frame size is 64 bytes (60 body + 4 FCS). For
    /// packets where the IP-declared length yields a body < 60 bytes,
    /// the sender pads with zeros *before* the FCS — so the actual frame
    /// length and FCS position are at the 64-byte mark, not the IP one.
    ///
    /// - IPv4 (0x0800): `max(14 + ip_total_len + 4, 64)` (clamped to buf).
    /// - ARP  (0x0806): a flat 64.
    /// - Anything else: `frame.len()`.
    pub fn derive_frame_len(frame: &[u8]) -> usize {
        if frame.len() < 18 {
            return frame.len();
        }
        let etype = u16::from_be_bytes([frame[12], frame[13]]);
        match etype {
            0x0800 => {
                let ip_total_len = u16::from_be_bytes([frame[16], frame[17]]) as usize;
                let computed = (14 + ip_total_len + 4).max(64);
                if computed <= frame.len() {
                    computed
                } else {
                    frame.len()
                }
            }
            0x0806 if frame.len() >= 64 => 64,
            _ => frame.len(),
        }
    }

    /// CRC-32 the first `frame_len - 4` bytes and compare against the
    /// trailing 4 bytes (little-endian on the wire). Returns true if the
    /// FCS matches — i.e. the frame decoded byte-perfect.
    pub fn verify_fcs(frame: &[u8], frame_len: usize) -> bool {
        if frame_len < 14 + 4 || frame_len > frame.len() {
            return false;
        }
        let computed = crate::crc::crc32_ieee802_3(&frame[..frame_len - 4]);
        let on_wire = u32::from_le_bytes([
            frame[frame_len - 4],
            frame[frame_len - 3],
            frame[frame_len - 2],
            frame[frame_len - 1],
        ]);
        computed == on_wire
    }

    /// Non-blocking. If a DMA half just completed, hand the just-finished
    /// half to `f` for scanning, carrying any frame that straddled the
    /// boundary across the swap, then update the carry and re-arm DMA.
    /// Caller must call this at least once per ~2.18 ms (one half-fill time)
    /// or samples drop.
    ///
    /// Scan-in-place to avoid a 16 KB copy every half (it was ~296 µs of the
    /// RX IRQ budget). The previous half's trailing-active tail lives in
    /// `carry`:
    /// - `carry_len == 0` (the common case — previous half ended idle):
    ///   nothing straddles, so `f` is called once on the new half directly,
    ///   no copy.
    /// - `carry_len > 0`: a frame straddled — its head is in `carry`, its
    ///   tail is the leading active run of the new half (up to the first idle
    ///   byte). Only `carry + that tail` is stitched into `stitch` (small);
    ///   `f` is then called a second time on the remainder of the new half in
    ///   place. The split point is an idle byte, so no active run is cut
    ///   across the two calls — `f` sees every frame whole, exactly as it did
    ///   with the old full-buffer stitch.
    ///
    /// So `f` may be invoked once or twice per completed half; each call gets
    /// a self-contained slice and the caller scans runs within it.
    pub fn poll_with<F: FnMut(&[u8])>(&mut self, mut f: F) {
        let xfer = self.xfer.take().unwrap();
        if !xfer.is_done() {
            self.xfer = Some(xfer);
            return;
        }
        // is_done() was true so wait() returns immediately.
        let (finished, idle) = xfer.wait();
        let new_bytes: &[u8] = unsafe {
            core::slice::from_raw_parts(finished.as_ptr() as *const u8, BUF_BYTES)
        };

        let cl = self.carry_len;
        if cl == 0 {
            f(new_bytes);
        } else {
            // Leading active run of the new half = the straddling frame's
            // tail; it ends at the first idle byte.
            let mut k = 0;
            while k < BUF_BYTES && new_bytes[k] != 0x00 && new_bytes[k] != 0xFF {
                k += 1;
            }
            // Stitch only carry + that tail (cl + k <= STITCH_BUF_BYTES).
            self.stitch[..cl].copy_from_slice(&self.carry[..cl]);
            self.stitch[cl..cl + k].copy_from_slice(&new_bytes[..k]);
            f(&self.stitch[..cl + k]);
            // Remainder of the new half, scanned in place (no copy).
            f(&new_bytes[k..]);
        }

        // Build the next carry: walk back from the end of the just-
        // finished half while bytes are "active" (non-0x00, non-0xFF), so
        // any frame whose tail is still in this half gets carried forward.
        // Cap at MAX_CARRY_BYTES; bump `carry_capped` if we hit the cap so
        // we can tell over-budget frames apart from clean termination.
        let mut tail_start = BUF_BYTES;
        loop {
            if tail_start == 0 {
                break;
            }
            if BUF_BYTES - tail_start >= MAX_CARRY_BYTES {
                self.carry_capped = self.carry_capped.wrapping_add(1);
                break;
            }
            let b = new_bytes[tail_start - 1];
            if b == 0x00 || b == 0xFF {
                break;
            }
            tail_start -= 1;
        }
        let new_carry_len = BUF_BYTES - tail_start;
        self.carry[..new_carry_len].copy_from_slice(&new_bytes[tail_start..]);
        self.carry_len = new_carry_len;

        self.xfer = Some(idle.write_next(finished));
    }
}
