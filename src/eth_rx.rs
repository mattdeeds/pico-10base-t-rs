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
        let div = (sys_clk_hz as f32) / (SAMPLE_HZ as f32);
        let div_int = div as u16;
        let div_frac = ((div - div_int as f32) * 256.0) as u8;

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
    /// (false → spurious or someone else's DMA channel sharing the line).
    /// Called from the DMA_IRQ_0 handler before `poll_with`.
    pub fn dma_irq_pending(&mut self) -> bool {
        self.xfer
            .as_mut()
            .map(|x| x.check_irq0())
            .unwrap_or(false)
    }

    /// Find the next active run (bytes that are neither 0x00 nor 0xFF) of
    /// length ≥ `min_len`, starting at or after `start`. Skips any runs
    /// shorter than `min_len` (NLPs / noise). Used by [`EthMac::poll`] to
    /// walk every frame-shaped run in a stitched buffer, not just the
    /// longest one — fixes loss when two frames land in the same DMA half.
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

    /// Phase-lock + Manchester-decode + SFD-align a frame-shaped active
    /// run in `bytes`. `base` is the byte offset of the run within `bytes`,
    /// `nbytes` its length. Returns the unverified post-SFD frame bytes
    /// (CRC verification happens in [`crc32_ieee802_3`] / R3.6).
    ///
    /// Algorithm — see `eth_rx_decode_frame` in `../Pico-10BASE-T/src/eth_rx.c`:
    /// 1. Find F = first H→L transition in the run = start of HB[0].
    /// 2. Data bit k value = sample at F + 4 + 6k (3 samples per half-bit,
    ///    so the midpoint of HB[2k+1] is sample 4 + 6k from F).
    /// 3. SFD = first `1,1` pair in the decoded bit stream — the last two
    ///    bits of the 0xD5 SFD byte (LSB-first).
    /// 4. Pack post-SFD bits LSB-first into frame bytes.
    pub fn decode_frame(
        bytes: &[u8],
        base: usize,
        nbytes: usize,
    ) -> Option<heapless::Vec<u8, 1600>> {
        let nsamples = nbytes * 8;
        let base_bit = base * 8;

        // First H→L transition.
        let mut f: Option<usize> = None;
        let mut prev = sample_bit(bytes, base_bit);
        for i in 1..nsamples {
            let s = sample_bit(bytes, base_bit + i);
            if prev == 1 && s == 0 {
                f = Some(i);
                break;
            }
            prev = s;
        }
        let f = f?;

        // Phase-locked data bits.
        let mut bits: heapless::Vec<u8, 2048> = heapless::Vec::new();
        for j in 0..1600usize {
            let idx = f + 4 + 6 * j;
            if idx >= nsamples {
                break;
            }
            if bits.push(sample_bit(bytes, base_bit + idx)).is_err() {
                break;
            }
        }

        // SFD: first `1,1` pair.
        let mut sfd_end: Option<usize> = None;
        for i in 1..bits.len() {
            if bits[i] == 1 && bits[i - 1] == 1 {
                sfd_end = Some(i);
                break;
            }
        }
        let sfd_end = sfd_end?;

        // Pack post-SFD bits LSB-first into bytes.
        let start_bit = sfd_end + 1;
        let avail = bits.len().saturating_sub(start_bit);
        let nframe = (avail / 8).min(1600);

        let mut frame: heapless::Vec<u8, 1600> = heapless::Vec::new();
        for i in 0..nframe {
            let mut byte: u8 = 0;
            for j in 0..8 {
                byte |= bits[start_bit + i * 8 + j] << j;
            }
            let _ = frame.push(byte);
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

    /// Non-blocking. If a DMA half just completed, invoke `f` with a
    /// stitched view = previous half's trailing active tail + the just-
    /// finished half. Then update the carry from the trailing active tail
    /// of the just-finished half, and re-arm DMA. Caller must call this
    /// at least once per ~2.18 ms (one half-fill time) or samples drop.
    ///
    /// The stitching means a frame straddling the half boundary appears
    /// as one contiguous active run starting somewhere inside the carry
    /// region and ending somewhere inside the new half — the decoder
    /// scans it as a single frame instead of seeing two truncated halves.
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

        // Stitch: prev carry | current half.
        let cl = self.carry_len;
        self.stitch[..cl].copy_from_slice(&self.carry[..cl]);
        self.stitch[cl..cl + BUF_BYTES].copy_from_slice(new_bytes);
        let total = cl + BUF_BYTES;
        f(&self.stitch[..total]);

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
