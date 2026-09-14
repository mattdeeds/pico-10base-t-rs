//! smoltcp `phy::Device` over `EthTx` + `EthRx`, split across two cores.
//!
//! - `EthMac` (core 0): TX, and `receive` pops the inbox.
//! - `RX_ENGINE` (core 1 IRQ): captures DMA halves into the image ring.
//! - `drain_rx_images` (core 1 thread): scan, decode, FCS check, publish.
//! - `RX_SHARED` (`Spinlock<0>`): inbox + stats, never held across decode.

use crate::eth_rx::EthRx;
use crate::eth_tx::{EthTx, UdpEndpoint};

use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, Ordering};
use heapless::{Deque, Vec};
use rp235x_hal::sio::Spinlock;
use smoltcp::phy::{self, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

#[cfg(not(feature = "mss-clamp"))]
pub const MTU: usize = 1500;
/// `mss-clamp`: smaller MTU so peers send smaller segments. Experiment only.
#[cfg(feature = "mss-clamp")]
pub const MTU: usize = 1000;
/// Slack over 1518-byte max Ethernet frame; decoder allocates this much.
pub const MAX_FRAME_BYTES: usize = 1600;
/// Inbox depth. The oldest frame drops when full.
pub const INBOX_SLOTS: usize = 4;
/// Bytes of each decoded frame kept for the log dump.
pub const FRAME_SNAP_BYTES: usize = 128;

/// Core-1 RX engine, touched only by `DMA_IRQ_0` after [`install_rx`].
struct RxEngine {
    rx: EthRx,
    /// Our MAC, for the RX filter.
    our_mac: [u8; 6],
}

/// Cross-core shared RX state: the decoded-frame inbox + stats. Guarded by
/// `Spinlock<0>` (see [`with_rx_shared`]). Core 1 publishes here under brief
/// locks; core 0 pops the inbox + snapshots stats under the same lock.
struct RxShared {
    inbox: Deque<Vec<u8, MAX_FRAME_BYTES>, INBOX_SLOTS>,
    stats: EthRxStats,
}

#[derive(Clone, Copy)]
pub struct EthRxStats {
    /// Decode attempts on runs that passed the MAC filter.
    pub frames_decoded: u32,
    pub fcs_ok: u32,
    pub fcs_fail: u32,
    /// Runs dropped by the MAC filter.
    pub frames_filtered: u32,
    pub inbox_dropped: u32,
    pub inbox_high_water: u8,
    pub carry_capped: u32,
    /// Snapshot of the most recently decoded frame for the log line.
    pub last_frame_len: usize,
    pub last_frame_was_ok: bool,
    pub last_frame_snapshot: [u8; FRAME_SNAP_BYTES],
    pub last_frame_snapshot_len: usize,
}

impl EthRxStats {
    /// `const` constructor so `RX_SHARED` can be a zero-initialized static.
    pub const fn new() -> Self {
        Self {
            frames_decoded: 0,
            fcs_ok: 0,
            fcs_fail: 0,
            frames_filtered: 0,
            inbox_dropped: 0,
            inbox_high_water: 0,
            carry_capped: 0,
            last_frame_len: 0,
            last_frame_was_ok: false,
            last_frame_snapshot: [0; FRAME_SNAP_BYTES],
            last_frame_snapshot_len: 0,
        }
    }
}

impl Default for EthRxStats {
    fn default() -> Self {
        Self::new()
    }
}

/// Holds [`RxEngine`]. No lock: core 0 writes once before core 1's IRQ runs.
struct EngineCell(UnsafeCell<Option<RxEngine>>);
// Safety: access is serialized by boot order.
unsafe impl Sync for EngineCell {}
static RX_ENGINE: EngineCell = EngineCell(UnsafeCell::new(None));

/// Image ring: `DMA_IRQ_0` produces, core 1's thread consumes.
/// SPSC on one core (IRQ preempts thread). Full ring drops images (`IMG_DROP`).
pub const IMG_SLOTS: usize = 6;
struct ImgRing(UnsafeCell<[[u8; crate::eth_rx::STITCH_BUF_BYTES]; IMG_SLOTS]>);
unsafe impl Sync for ImgRing {}
static IMG_RING: ImgRing =
    ImgRing(UnsafeCell::new([[0; crate::eth_rx::STITCH_BUF_BYTES]; IMG_SLOTS]));
#[allow(clippy::declare_interior_mutable_const)]
const ATOMIC_ZERO: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static IMG_LEN: [core::sync::atomic::AtomicU32; IMG_SLOTS] = [ATOMIC_ZERO; IMG_SLOTS];
static IMG_CL: [core::sync::atomic::AtomicU32; IMG_SLOTS] = [ATOMIC_ZERO; IMG_SLOTS];
/// Monotonic produce / consume sequence numbers (slot = seq % IMG_SLOTS).
static IMG_W: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static IMG_R: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Completed halves dropped because the ring was full (decode backlog).
pub static IMG_DROP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Overload discard slot. The DMA must be serviced even when the ring is
/// full; skipping it desyncs the HAL Transfer and kills RX permanently.
struct DiscardSlot(UnsafeCell<[u8; crate::eth_rx::STITCH_BUF_BYTES]>);
unsafe impl Sync for DiscardSlot {}
static IMG_DISCARD: DiscardSlot =
    DiscardSlot(UnsafeCell::new([0; crate::eth_rx::STITCH_BUF_BYTES]));

/// Decodes and FCS fails of carry-prefixed images.
pub static STITCH_DEC: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static STITCH_FAIL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Halves where the PIO RX FIFO overflowed. Nonzero means lost samples.
pub static RXSTALL_HALVES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Cross-core inbox + stats, guarded by `Spinlock<0>`.
struct SharedCell(UnsafeCell<RxShared>);
// Safety: every access goes through `with_rx_shared`, which holds `Spinlock<0>`.
unsafe impl Sync for SharedCell {}
static RX_SHARED: SharedCell = SharedCell(UnsafeCell::new(RxShared {
    inbox: Deque::new(),
    stats: EthRxStats::new(),
}));

/// Run `f` holding `Spinlock<0>`. Not re-entrant; never hold across decode.
#[inline]
fn with_rx_shared<R>(f: impl FnOnce(&mut RxShared) -> R) -> R {
    let _lock = Spinlock::<0>::claim();
    // Safety: `Spinlock<0>` guards all access to RX_SHARED on both cores.
    let shared = unsafe { &mut *RX_SHARED.0.get() };
    f(shared)
}

/// Move `rx` into the core-1 engine. Call once on core 0 before launching
/// core 1. Returns `false` if already installed.
pub fn install_rx(rx: EthRx, our_mac: [u8; 6]) -> bool {
    // Safety: core 1 can't touch RX_ENGINE yet. The fence publishes the write.
    let slot = unsafe { &mut *RX_ENGINE.0.get() };
    if slot.is_some() {
        return false;
    }
    *slot = Some(RxEngine { rx, our_mac });
    compiler_fence(Ordering::Release);
    true
}

/// Accept unicast to us, broadcast, and multicast. smoltcp filters further.
#[inline]
fn mac_accept(dst: &[u8; 6], our: &[u8; 6]) -> bool {
    dst == our || (dst[0] & 0x01) != 0
}

/// Snapshot and reset RX stats. Called once a second.
pub fn snapshot_rx_stats() -> EthRxStats {
    with_rx_shared(|shared| {
        let out = shared.stats;
        shared.stats = EthRxStats::default();
        out
    })
}

/// Stat deltas built lock-free during decode, merged once after.
struct StatsDelta {
    decoded: u32,
    ok: u32,
    fail: u32,
    filtered: u32,
    carry: u32,
    /// Most-recently decoded frame snapshot (for the 1 Hz log line).
    last_snap: [u8; FRAME_SNAP_BYTES],
    last_snap_len: usize,
    last_len: usize,
    last_ok: bool,
    have_last: bool,
}

impl StatsDelta {
    fn new() -> Self {
        Self {
            decoded: 0,
            ok: 0,
            fail: 0,
            filtered: 0,
            carry: 0,
            last_snap: [0; FRAME_SNAP_BYTES],
            last_snap_len: 0,
            last_len: 0,
            last_ok: false,
            have_last: false,
        }
    }
}

/// `DMA_IRQ_0` body: capture the finished half into the ring and re-arm.
/// Bounded time; decode runs later in [`drain_rx_images`].
fn process_completed_half(engine: &mut RxEngine) {
    // Count RX FIFO overflows (write-1-to-clear).
    {
        let pio = unsafe { &*rp235x_hal::pac::PIO0::ptr() };
        if pio.fdebug().read().rxstall().bits() & (1 << 1) != 0 {
            pio.fdebug().write(|w| unsafe { w.rxstall().bits(1 << 1) });
            RXSTALL_HALVES.fetch_add(1, Ordering::Relaxed);
        }
    }
    let w = IMG_W.load(Ordering::Relaxed);
    let r = IMG_R.load(Ordering::Acquire);
    if w.wrapping_sub(r) >= IMG_SLOTS as u32 {
        // Ring full: still service the DMA, then drop the image.
        // Safety: the discard slot is IRQ-only.
        let slot = unsafe { &mut *IMG_DISCARD.0.get() };
        if let crate::eth_rx::PollOutcome::Image { .. } = engine.rx.poll_into(&mut slot[..]) {
            IMG_DROP.fetch_add(1, Ordering::Relaxed);
        }
        return;
    }
    let slot_idx = (w as usize) % IMG_SLOTS;
    // Safety: producer-exclusive slot; the consumer reads only slots < w.
    let slot = unsafe { &mut (*IMG_RING.0.get())[slot_idx] };
    match engine.rx.poll_into(&mut slot[..]) {
        crate::eth_rx::PollOutcome::Nothing => {}
        crate::eth_rx::PollOutcome::Image { len, carry_prefix } => {
            IMG_LEN[slot_idx].store(len as u32, Ordering::Relaxed);
            IMG_CL[slot_idx].store(carry_prefix as u32, Ordering::Relaxed);
            IMG_W.store(w.wrapping_add(1), Ordering::Release);
        }
    }
    // Merge the carry-cap count under the brief lock.
    let capped = engine.rx.take_carry_capped();
    if capped != 0 {
        with_rx_shared(|shared| {
            shared.stats.carry_capped = shared.stats.carry_capped.wrapping_add(capped);
        });
    }
}

/// Core 1 thread: scan and decode queued images. No deadline.
pub fn drain_rx_images() {
    loop {
        let r = IMG_R.load(Ordering::Relaxed);
        let w = IMG_W.load(Ordering::Acquire);
        if r == w {
            return;
        }
        let slot_idx = (r as usize) % IMG_SLOTS;
        let len = IMG_LEN[slot_idx].load(Ordering::Relaxed) as usize;
        let cl = IMG_CL[slot_idx].load(Ordering::Relaxed) as usize;
        // Safety: consumer-exclusive slot (r < w; producer writes only at
        // w % IMG_SLOTS and refuses when the ring is full).
        let image: &[u8] = unsafe {
            let ring: *const [[u8; crate::eth_rx::STITCH_BUF_BYTES]; IMG_SLOTS] =
                IMG_RING.0.get();
            core::slice::from_raw_parts((*ring)[slot_idx].as_ptr(), len)
        };
        // our_mac is stable after install_rx.
        let our_mac = unsafe {
            (*RX_ENGINE.0.get())
                .as_ref()
                .map(|e| e.our_mac)
                .unwrap_or([0; 6])
        };
        let mut acc = StatsDelta::new();
        scan_runs(&our_mac, image, if cl > 0 { Some(cl) } else { None }, &mut acc);
        merge_stats(&acc);
        IMG_R.store(r.wrapping_add(1), Ordering::Release);
    }
}

/// Decode and verify runs addressed to us; publish FCS-OK frames.
fn scan_runs(our_mac: &[u8; 6], bytes: &[u8], stitch_cl: Option<usize>, acc: &mut StatsDelta) {
    let mut cursor = 0;
    while let Some((off, len)) = EthRx::find_active_run_from(bytes, cursor, 100) {
        cursor = off + len;
        // Cheap MAC peek (~1–2 µs) before the full decode.
        let Some(dst) = EthRx::peek_dst_mac(bytes, off, len) else {
            continue;
        };
        if !mac_accept(&dst, our_mac) {
            acc.filtered = acc.filtered.wrapping_add(1);
            continue;
        }
        // DPLL decoder by default; open-loop under `decoder-openloop`.
        #[cfg(feature = "decoder-openloop")]
        let decoded = EthRx::decode_frame(bytes, off, len);
        #[cfg(not(feature = "decoder-openloop"))]
        let decoded = crate::eth_rx_dpll::decode_frame_edge_track(&bytes[off..off + len]);
        let Some(mut frame) = decoded else {
            continue;
        };
        let flen = EthRx::derive_frame_len(&frame);
        let ok = EthRx::verify_fcs(&frame, flen);
        let n = flen.min(frame.len());

        acc.decoded = acc.decoded.wrapping_add(1);
        if ok {
            acc.ok = acc.ok.wrapping_add(1);
        } else {
            acc.fail = acc.fail.wrapping_add(1);
        }
        // Carry-prefixed-image provenance (straddler-path health).
        if let Some(_cl) = stitch_cl {
            STITCH_DEC.fetch_add(1, Ordering::Relaxed);
            if !ok {
                STITCH_FAIL.fetch_add(1, Ordering::Relaxed);
            }
        }
        let snap_n = n.min(acc.last_snap.len());
        acc.last_snap[..snap_n].copy_from_slice(&frame[..snap_n]);
        acc.last_snap_len = snap_n;
        acc.last_len = n;
        acc.last_ok = ok;
        acc.have_last = true;

        if !ok {
            continue;
        }
        frame.truncate(n);
        // Brief cross-core lock: publish this frame + inbox-side stats.
        with_rx_shared(|shared| {
            if shared.inbox.is_full() {
                let _ = shared.inbox.pop_front();
                shared.stats.inbox_dropped = shared.stats.inbox_dropped.wrapping_add(1);
            }
            let _ = shared.inbox.push_back(frame);
            let depth = shared.inbox.len() as u8;
            if depth > shared.stats.inbox_high_water {
                shared.stats.inbox_high_water = depth;
            }
        });
    }
}

/// Merge stat deltas and the last-frame snapshot under the lock.
fn merge_stats(acc: &StatsDelta) {
    with_rx_shared(|shared| {
        let s = &mut shared.stats;
        s.frames_decoded = s.frames_decoded.wrapping_add(acc.decoded);
        s.fcs_ok = s.fcs_ok.wrapping_add(acc.ok);
        s.fcs_fail = s.fcs_fail.wrapping_add(acc.fail);
        s.frames_filtered = s.frames_filtered.wrapping_add(acc.filtered);
        s.carry_capped = s.carry_capped.wrapping_add(acc.carry);
        if acc.have_last {
            s.last_frame_snapshot[..acc.last_snap_len]
                .copy_from_slice(&acc.last_snap[..acc.last_snap_len]);
            s.last_frame_snapshot_len = acc.last_snap_len;
            s.last_frame_len = acc.last_len;
            s.last_frame_was_ok = acc.last_ok;
        }
    });
}


/// DMA completion IRQ, once per half-buffer. Runs on core 1 only.
/// `no_mangle` so `rp235x-hal::arch` links to it.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn DMA_IRQ_0() {
    // Safety: only this handler touches RX_ENGINE after install_rx.
    let Some(engine) = (unsafe { (*RX_ENGINE.0.get()).as_mut() }) else {
        return;
    };
    // Clear the pending bit but don't gate on it: accounting can lag a half
    // after overload. `poll_into`'s `is_done()` is authoritative.
    let _ = engine.rx.dma_irq_pending();
    // Router build: count core-1 IRQ cycles (decode isn't in here).
    #[cfg(feature = "router")]
    let _cyc = crate::cycles::CycleSpan::new(&crate::cycles::CORE1_BUSY);
    process_completed_half(engine);

}

pub struct EthMac {
    tx: EthTx,
    tx_buf: [u8; MAX_FRAME_BYTES],
    /// TX-side diagnostic counters surfaced to the main loop log.
    pub stats: EthMacStats,
}

pub struct EthMacStats {
    /// `Device::receive` was asked and we returned `Some`.
    pub rx_handed_out: u32,
    /// `Device::transmit` was called and we returned `Some`.
    pub tx_handed_out: u32,
    /// `TxToken::consume` ran — i.e. smoltcp actually filled & dispatched.
    pub tx_consumed: u32,
    pub tx_arp: u32,
    pub tx_icmp: u32,
    pub tx_udp: u32,
    pub tx_other: u32,
    /// Bytes of the most recent TxToken::consume (frame body, pre-FCS).
    pub last_tx_len: u16,
    /// Snapshot of the most recent TX body (first 128 bytes).
    pub last_tx: [u8; 128],
}

impl Default for EthMacStats {
    fn default() -> Self {
        Self {
            rx_handed_out: 0,
            tx_handed_out: 0,
            tx_consumed: 0,
            tx_arp: 0,
            tx_icmp: 0,
            tx_udp: 0,
            tx_other: 0,
            last_tx_len: 0,
            last_tx: [0; 128],
        }
    }
}

impl EthMac {
    pub fn new(tx: EthTx) -> Self {
        Self {
            tx,
            tx_buf: [0; MAX_FRAME_BYTES],
            stats: EthMacStats::default(),
        }
    }

    pub fn send_nlp(&mut self) {
        self.tx.send_nlp();
    }

    pub fn send_udp_broadcast(&mut self, ep: &UdpEndpoint, payload: &[u8]) {
        self.tx.send_udp_broadcast(ep, payload);
    }
}

impl phy::Device for EthMac {
    type RxToken<'a>
        = EthRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = EthTxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, _ts: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // Pop one frame; the lock covers only `pop_front`.
        let buf = with_rx_shared(|shared| shared.inbox.pop_front())?;
        self.stats.rx_handed_out = self.stats.rx_handed_out.wrapping_add(1);
        Some((
            EthRxToken { buf },
            EthTxToken {
                tx: &mut self.tx,
                buf: &mut self.tx_buf,
                stats: &mut self.stats,
            },
        ))
    }

    fn transmit(&mut self, _ts: Instant) -> Option<Self::TxToken<'_>> {
        self.stats.tx_handed_out = self.stats.tx_handed_out.wrapping_add(1);
        Some(EthTxToken {
            tx: &mut self.tx,
            buf: &mut self.tx_buf,
            stats: &mut self.stats,
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = MTU;
        // smoltcp clamps the TCP receive window to max_burst_size × MSS.
        // 2 measured fastest; wider adds half-duplex collision loss.
        caps.max_burst_size = Some(2);
        caps
    }
}

/// RX token owning one decoded frame.
pub struct EthRxToken {
    buf: Vec<u8, MAX_FRAME_BYTES>,
}

impl phy::RxToken for EthRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(&self.buf)
    }
}

/// TX token. smoltcp fills the body; `send_raw_frame` adds preamble and FCS.
pub struct EthTxToken<'a> {
    tx: &'a mut EthTx,
    buf: &'a mut [u8; MAX_FRAME_BYTES],
    stats: &'a mut EthMacStats,
}

impl<'a> phy::TxToken for EthTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // Forwarded frames can reach MAX_FRAME_BYTES; clamp so `len` can't panic.
        let len = len.min(self.buf.len());
        let slice = &mut self.buf[..len];
        let result = f(slice);
        // Categorize: ARP, ICMP (IPv4 proto 1), UDP (IPv4 proto 17), other.
        if len >= 14 {
            let ethertype = u16::from_be_bytes([slice[12], slice[13]]);
            match ethertype {
                0x0806 => self.stats.tx_arp = self.stats.tx_arp.wrapping_add(1),
                0x0800 if len >= 24 => match slice[23] {
                    1 => {
                        self.stats.tx_icmp = self.stats.tx_icmp.wrapping_add(1);
                        let snap_n = len.min(self.stats.last_tx.len());
                        self.stats.last_tx[..snap_n].copy_from_slice(&slice[..snap_n]);
                        self.stats.last_tx_len = len as u16;
                    }
                    17 => self.stats.tx_udp = self.stats.tx_udp.wrapping_add(1),
                    _ => self.stats.tx_other = self.stats.tx_other.wrapping_add(1),
                },
                _ => self.stats.tx_other = self.stats.tx_other.wrapping_add(1),
            }
        }
        self.tx.send_raw_frame(slice);
        self.stats.tx_consumed = self.stats.tx_consumed.wrapping_add(1);
        result
    }
}
