//! 10BASE-T RX: PIO sampler + DMA double-buffer.
//!
//! PIO samples RO (GP13) at `SAMPLE_HZ`, 32 bits per FIFO word, LSB-first.
//! DMA fills two half-buffers in turn. `poll_into` must re-arm within
//! ~4.4 ms or the PIO FIFO overflows and samples are lost (`rxstall`).

use rp235x_hal as hal;
use hal::dma::{double_buffer, Channel, SingleChannel, CH0, CH1};
use hal::pac::PIO0;
use hal::pio::{
    Buffers, PinDir, Rx, Running, ShiftDirection, StateMachine, UninitStateMachine, SM1,
};

#[cfg(feature = "decoder-openloop")]
use crate::eth_mac::MAX_FRAME_BYTES;

/// Sampler rate: 60 MHz = 3 samples per half-bit.
/// `sample-rate-20mhz` gives 1 per half-bit and forces the open-loop decoder.
#[cfg(not(feature = "sample-rate-20mhz"))]
pub const SAMPLE_HZ: u32 = 60_000_000;
#[cfg(feature = "sample-rate-20mhz")]
pub const SAMPLE_HZ: u32 = 20_000_000;

/// Samples per data bit, and offset from bit start to mid second half-bit.
#[cfg(not(feature = "sample-rate-20mhz"))]
const SAMPLES_PER_BIT: usize = 6;
#[cfg(feature = "sample-rate-20mhz")]
const SAMPLES_PER_BIT: usize = 2;

#[cfg(not(feature = "sample-rate-20mhz"))]
const HB1_CENTER_OFFSET: usize = 4;
#[cfg(feature = "sample-rate-20mhz")]
const HB1_CENTER_OFFSET: usize = 1;

/// Words per half-buffer: 16 KB, ~2.18 ms at 60 MHz.
/// Also the RX latency floor. Other sizes are untested.
pub const BUF_WORDS: usize = 4096;
/// Bytes per half-buffer.
pub const BUF_BYTES: usize = BUF_WORDS * 4;

/// Max bytes carried into the next half for straddling frames.
/// A 1526-byte frame is ~9.2 KB of samples at 60 MHz; 16 KB adds slack.
pub const MAX_CARRY_BYTES: usize = 16 * 1024;

/// Image slot size: max carry plus one half.
pub const STITCH_BUF_BYTES: usize = BUF_BYTES + MAX_CARRY_BYTES;

/// Cap on the open-loop SFD search, bounding noise runs.
#[cfg(feature = "decoder-openloop")]
const SFD_SEARCH_BITS: usize = 1600;

type RxFifo = Rx<(PIO0, SM1)>;
pub type RxBuf = &'static mut [u32; BUF_WORDS];
pub type CarryBuf = &'static mut [u8; MAX_CARRY_BYTES];
type Xfer = double_buffer::Transfer<
    Channel<CH0>,
    Channel<CH1>,
    RxFifo,
    RxBuf,
    double_buffer::WriteNext<RxBuf>,
>;

/// Sample bit at `bit_offset`, packed LSB-first per byte.
#[inline]
fn sample_bit(bytes: &[u8], bit_offset: usize) -> u8 {
    (bytes[bit_offset >> 3] >> (bit_offset & 7)) & 1
}

/// Offset of the first H→L edge (start of half-bit 0), if any.
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

/// Open-loop data bit `k`: the sample at `F + HB1_CENTER_OFFSET + SAMPLES_PER_BIT * k`.
/// `None` once past `nsamples`.
#[inline]
fn data_bit(bytes: &[u8], base_bit: usize, f: usize, k: usize, nsamples: usize) -> Option<u8> {
    let idx = f + HB1_CENTER_OFFSET + SAMPLES_PER_BIT * k;
    if idx >= nsamples {
        None
    } else {
        Some(sample_bit(bytes, base_bit + idx))
    }
}

/// Bit index ending the SFD (second bit of the first `1,1` pair) within
/// `max_bits`. Frame data starts at the next bit.
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

/// PIO RX + DMA double-buffer state. `carry` holds a frame straddling halves.
pub struct EthRx {
    _sm: StateMachine<(PIO0, SM1), Running>,
    xfer: Option<Xfer>,
    carry: CarryBuf,
    carry_len: usize,
    /// Times the carry hit `MAX_CARRY_BYTES`, clipping a frame.
    pub carry_capped: u32,
}

impl EthRx {
    /// Start the sampler on PIO0 SM1 and arm the DMA double-buffer.
    /// The pin must already be in PIO0 function.
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
    ) -> Self {
        let program = pio::pio_asm!(".wrap_target", "    in pins, 1", ".wrap",);

        let installed = pio.install(&program.program).unwrap();

        // Sample clock divider.
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

        // Enable DMA_IRQ_0 before `Config::new` consumes the channels.
        // The enable bit survives chain swaps.
        dma_ch_a.enable_irq0();
        dma_ch_b.enable_irq0();

        // ch_a fills buf_a, then chains to ch_b with buf_b.
        let xfer = double_buffer::Config::new((dma_ch_a, dma_ch_b), rx, buf_a).start();
        let xfer = xfer.write_next(buf_b);

        Self {
            _sm: sm,
            xfer: Some(xfer),
            carry,
            carry_len: 0,
            carry_capped: 0,
        }
    }

    /// Read and reset `carry_capped`.
    pub fn take_carry_capped(&mut self) -> u32 {
        let v = self.carry_capped;
        self.carry_capped = 0;
        v
    }

    /// Check and clear the active channel's DMA_IRQ_0 pending bit.
    pub fn dma_irq_pending(&mut self) -> bool {
        self.xfer
            .as_mut()
            .map(|x| x.check_irq0())
            .unwrap_or(false)
    }

    /// Next active run (bytes not 0x00/0xFF) of at least `min_len`, from `start`.
    /// Shorter runs (NLPs, noise) are skipped.
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

    /// Open-loop decode of just the destination MAC (~1–2 µs). The MAC is early
    /// enough to ignore drift. `None` if F or SFD isn't found in 200 bits.
    pub fn peek_dst_mac(bytes: &[u8], base: usize, nbytes: usize) -> Option<[u8; 6]> {
        let nsamples = nbytes * 8;
        let base_bit = base * 8;

        let f = find_first_falling_edge(bytes, base_bit, nsamples)?;

        // 112 bits minimum, plus slack for a late first H→L edge.
        const MAX_BITS: usize = 200;
        let sfd_end = find_sfd_end(bytes, base_bit, f, nsamples, MAX_BITS)?;

        // The dst MAC must fit in the same window.
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

    /// Open-loop Manchester decode (`decoder-openloop` A/B build only).
    /// Returns unverified post-SFD bytes; see `verify_fcs`.
    ///
    /// Locks to the first H→L edge, samples each bit at a fixed stride, and
    /// stops at the header-declared length.
    #[cfg(feature = "decoder-openloop")]
    pub fn decode_frame(
        bytes: &[u8],
        base: usize,
        nbytes: usize,
    ) -> Option<heapless::Vec<u8, MAX_FRAME_BYTES>> {
        let base_bit = base * 8;
        // Clamp to the buffer so the unchecked reads stay sound.
        let buf_bits = bytes.len() * 8;
        if base_bit >= buf_bits {
            return None;
        }
        let nsamples = (nbytes * 8).min(buf_bits - base_bit);

        let f = find_first_falling_edge(bytes, base_bit, nsamples)?;
        let sfd_end = find_sfd_end(bytes, base_bit, f, nsamples, SFD_SEARCH_BITS)?;

        // Whole frame bytes available; pack with a striding offset.
        let start_bit = sfd_end + 1;
        let limit = base_bit + nsamples;
        let first_off = base_bit + f + HB1_CENTER_OFFSET + SAMPLES_PER_BIT * start_bit;
        let avail_bits = if first_off < limit {
            (limit - 1 - first_off) / SAMPLES_PER_BIT + 1
        } else {
            0
        };
        let nframe_avail = (avail_bits / 8).min(MAX_FRAME_BYTES);

        // After the 18-byte header, cap to the declared length so long runs
        // (merged frames, noise) stay cheap. Unknown EtherTypes stay uncapped.
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
                off += SAMPLES_PER_BIT;
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

    /// Frame length including FCS, from EtherType and IP length.
    /// IPv4: `max(14 + ip_total_len + 4, 64)`, clamped to the buffer. ARP: 64.
    /// Otherwise `frame.len()`.
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

    /// True if the trailing 4-byte FCS (little-endian) matches.
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

}

/// Result of `poll_into`.
pub enum PollOutcome {
    /// No image to process.
    Nothing,
    /// `slot[..len]` holds the continuous sample image to scan + decode.
    Image { len: usize, carry_prefix: usize },
}

impl EthRx {
    /// Service a completed half: write carry ++ settled bytes into `slot`,
    /// update the carry, re-arm the DMA. Bounded (~0.6 ms worst).
    /// `slot` must be at least `STITCH_BUF_BYTES`.
    pub fn poll_into(&mut self, slot: &mut [u8]) -> PollOutcome {
        let xfer = self.xfer.take().unwrap();
        if !xfer.is_done() {
            self.xfer = Some(xfer);
            return PollOutcome::Nothing;
        }
        // is_done() was true so wait() returns immediately.
        let (finished, idle) = xfer.wait();
        let new_bytes: &[u8] = unsafe {
            core::slice::from_raw_parts(finished.as_ptr() as *const u8, BUF_BYTES)
        };

        let cl = self.carry_len;

        // Leading active run; only meaningful if a frame straddled in.
        let mut k = 0;
        if cl > 0 {
            while k < BUF_BYTES && new_bytes[k] != 0x00 && new_bytes[k] != 0xFF {
                k += 1;
            }
            if k == BUF_BYTES {
                // Whole half active: the frame hasn't ended, so carry it.
                // Unreachable with 16 KB halves and legal frames.
                if cl + BUF_BYTES <= MAX_CARRY_BYTES {
                    self.carry[cl..cl + BUF_BYTES].copy_from_slice(new_bytes);
                    self.carry_len = cl + BUF_BYTES;
                } else {
                    // Over budget means sustained noise. Drop and count.
                    self.carry_capped = self.carry_capped.wrapping_add(1);
                    self.carry_len = 0;
                }
                self.xfer = Some(idle.write_next(finished));
                return PollOutcome::Nothing;
            }
        }

        // Carry the unterminated trailing run into the next half rather than
        // decode a truncated head. Capped at MAX_CARRY_BYTES.
        let mut tail_start = BUF_BYTES;
        loop {
            if tail_start <= k {
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

        // Image = carry ++ settled bytes: one continuous sample stream.
        slot[..cl].copy_from_slice(&self.carry[..cl]);
        slot[cl..cl + tail_start].copy_from_slice(&new_bytes[..tail_start]);
        let image_len = cl + tail_start;

        // Next carry = the trailing in-flight run.
        let new_carry_len = BUF_BYTES - tail_start;
        self.carry[..new_carry_len].copy_from_slice(&new_bytes[tail_start..]);
        self.carry_len = new_carry_len;

        // Re-arm. The DMA now runs two half-fills without us.
        self.xfer = Some(idle.write_next(finished));

        PollOutcome::Image {
            len: image_len,
            carry_prefix: cl,
        }
    }
}
