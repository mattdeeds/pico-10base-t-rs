//! EthMac — bridge between {EthTx, EthRx} and smoltcp's `phy::Device`.
//!
//! After R6 (IRQ-driven RX) and R12c (RX decode on core 1), responsibilities
//! are split across two axes — TX vs RX, and core 0 vs core 1:
//!
//! - **EthMac** owns just `EthTx` + the TX scratch buffer + TX stats. It
//!   implements `smoltcp::phy::Device` on **core 0**; `transmit` hands out a
//!   `TxToken` that uses the local TX state directly, and `receive` pops the
//!   shared RX inbox (below).
//!
//! - **`RX_ENGINE`** (core-1-exclusive) holds the `EthRx` state machine + our
//!   MAC. After `install_rx` populates it on core 0 (once, before core 1
//!   enables `DMA_IRQ_0`), it is touched **only by core 1's `DMA_IRQ_0`
//!   handler**, which captures each completed half into the image ring and
//!   re-arms the DMA in bounded time. The scan + decode + verify pipeline
//!   runs in core 1's *thread* loop (`drain_rx_images`) from the ring — out
//!   of the IRQ, so a long decode can never starve the DMA re-arm (which
//!   silently truncated frames; see `eth_rx::poll_into`).
//!
//! - **`RX_SHARED`** (cross-core, guarded by `Spinlock<0>`) holds just the
//!   decoded-frame inbox + RX stats. Core 1 publishes FCS-OK frames + stat
//!   deltas under *brief* locks; core 0 pops the inbox in `receive` and reads
//!   stats in `snapshot_rx_stats` under the same lock. The lock is never held
//!   across the decode, so core 1's ≤2.57 ms decode can't starve core 0
//!   (R12c — the whole point of moving RX off core 0). `Spinlock<0>` is
//!   distinct from `critical_section`'s reserved `Spinlock<31>`.
//!
//! Pass-through TX (`send_nlp`, `send_udp_broadcast`) on `EthMac` still
//! works alongside smoltcp for the NLP keepalive + smoke-test UDP loop.

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
/// `mss-clamp` (`docs/rx-bulk-ceiling.md` §5/§9/§10): clamp the advertised IP
/// MTU so peers send smaller TCP segments. OBSOLETE as a performance lever
/// since the 2026-06-10 decode-out-of-IRQ restructure: the "decode cliff"
/// this worked around was DMA-starvation sample loss, not clock drift, and
/// full-MTU RX now decodes at ~0.2% loss / ~310 KB/s (≥ any clamped value).
/// Kept only as an experiment knob.
#[cfg(feature = "mss-clamp")]
pub const MTU: usize = 1000;
/// Slack over 1518-byte max Ethernet frame; decoder allocates this much.
pub const MAX_FRAME_BYTES: usize = 1600;
/// How many decoded frames the inbox can hold before back-pressure forces
/// us to drop the oldest. 4 covers the burst case of two concurrent flows
/// (e.g. ping + UDP echo) landing several frames in the same DMA half.
pub const INBOX_SLOTS: usize = 4;
/// Bytes of each decoded frame we copy into stats for the 1 Hz log dump.
/// 128 is enough for dst/src MACs + EtherType + an IPv4 header + first
/// few payload bytes — matches the existing main.rs hex-dump width.
pub const FRAME_SNAP_BYTES: usize = 128;

/// Core-1-exclusive RX engine: the `EthRx` state machine + our MAC for the
/// IRQ-side filter. Written once by core 0 in [`install_rx`] (before core 1
/// enables `DMA_IRQ_0`), then touched **only** by core 1's `DMA_IRQ_0`
/// handler — so the decode pipeline needs no lock.
struct RxEngine {
    rx: EthRx,
    /// Our 6-byte MAC. Used by the IRQ-side filter to skip frames not
    /// addressed to us before paying for the full decode + CRC + push.
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
    /// Total decode attempts in the window (one per active run found
    /// AND accepted by the MAC filter).
    pub frames_decoded: u32,
    pub fcs_ok: u32,
    pub fcs_fail: u32,
    /// Active runs that peek_dst_mac accepted but couldn't actually
    /// decode (rare — usually means active-run length was a noise blob).
    /// Note: separate from `fcs_fail` which is for runs that decoded
    /// but failed CRC.
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

/// Core-1-exclusive RX engine. `None` until [`install_rx`]. Wrapped in an
/// `UnsafeCell` because access is single-owner-after-init (core 0 writes once
/// before core 1's IRQ is live; core 1's handler reads thereafter), so no lock
/// is needed — the `Sync` impl asserts that contract.
struct EngineCell(UnsafeCell<Option<RxEngine>>);
// Safety: see `RxEngine` / `install_rx` — access is serialized by boot order,
// never genuinely concurrent.
unsafe impl Sync for EngineCell {}
static RX_ENGINE: EngineCell = EngineCell(UnsafeCell::new(None));

/// Image ring: the DMA_IRQ_0 handler (producer) writes each completed half's
/// processing image (carry ++ settled bytes) into a slot and re-arms the DMA
/// immediately; core 1's thread loop (consumer, `drain_rx_images`) scans +
/// decodes the slots with no deadline pressure on the DMA. SPSC on one core:
/// the IRQ preempts the thread, never the reverse, so sequence counters with
/// Acquire/Release are sufficient. On overload (ring full) the IRQ drops the
/// whole image and counts it — bounded, visible degradation instead of
/// silent FIFO-overflow sample corruption.
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
/// Overload discard slot: when the ring is full the DMA must STILL be
/// serviced (carry bookkeeping + re-arm — skipping it desyncs the HAL
/// Transfer's channel accounting and kills RX permanently). The image goes
/// here and is dropped, counted in IMG_DROP.
struct DiscardSlot(UnsafeCell<[u8; crate::eth_rx::STITCH_BUF_BYTES]>);
unsafe impl Sync for DiscardSlot {}
static IMG_DISCARD: DiscardSlot =
    DiscardSlot(UnsafeCell::new([0; crate::eth_rx::STITCH_BUF_BYTES]));

/// Diagnostic: decode outcomes for runs that came through the carry+stitch
/// straddler path (vs scanned in place). Read+reset by the 1 Hz diag log.
pub static STITCH_DEC: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static STITCH_FAIL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// DIAG: halves during which the RX sampler's FIFO overflowed (PIO0 FDEBUG
/// RXSTALL bit for SM1, checked+cleared once per completed half). Nonzero ⇒
/// the DMA fell behind and samples were LOST — frames in flight at that
/// moment are silently truncated.
pub static RXSTALL_HALVES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Cross-core shared inbox + stats, guarded by `Spinlock<0>`. Const-init so it
/// needs no run-time setup.
struct SharedCell(UnsafeCell<RxShared>);
// Safety: every access goes through `with_rx_shared`, which holds `Spinlock<0>`.
unsafe impl Sync for SharedCell {}
static RX_SHARED: SharedCell = SharedCell(UnsafeCell::new(RxShared {
    inbox: Deque::new(),
    stats: EthRxStats::new(),
}));

/// Run `f` with exclusive cross-core access to `RX_SHARED`, holding
/// `Spinlock<0>` for the (brief) duration. Both cores funnel inbox + stats
/// access through here. Never call it while already holding `Spinlock<0>`
/// (the spinlock is not re-entrant) and never hold it across the decode.
#[inline]
fn with_rx_shared<R>(f: impl FnOnce(&mut RxShared) -> R) -> R {
    let _lock = Spinlock::<0>::claim();
    // Safety: `Spinlock<0>` guards all access to RX_SHARED on both cores.
    let shared = unsafe { &mut *RX_SHARED.0.get() };
    f(shared)
}

/// Move an `EthRx` into the core-1-exclusive engine, along with our MAC for
/// the IRQ-side MAC filter. Call once on core 0, after constructing `EthRx`
/// and **before launching core 1** (which enables `DMA_IRQ_0`). Returns
/// `false` if already populated (programmer error).
pub fn install_rx(rx: EthRx, our_mac: [u8; 6]) -> bool {
    // Single-owner write: this runs on core 0 before core 1's handler can
    // touch RX_ENGINE, so no lock is needed. A Release fence makes the write
    // visible to core 1 (RP2350 has no caches, so a compiler fence suffices).
    // Safety: no concurrent access at install time (see `RxEngine`).
    let slot = unsafe { &mut *RX_ENGINE.0.get() };
    if slot.is_some() {
        return false;
    }
    *slot = Some(RxEngine { rx, our_mac });
    compiler_fence(Ordering::Release);
    true
}

/// Should the IRQ-side MAC filter accept a frame with this destination?
/// Accepts: unicast addressed to us, broadcast, any multicast (per the
/// I/G bit — bit 0 of byte 0). smoltcp does the finer-grained filtering
/// later (it'll silently drop multicasts we're not subscribed to), but
/// this gate at least skips the bulk of stranger-unicast traffic.
#[inline]
fn mac_accept(dst: &[u8; 6], our: &[u8; 6]) -> bool {
    dst == our || (dst[0] & 0x01) != 0
}

/// Snapshot + reset the IRQ-managed RX stats. Called by the main loop
/// every second for the log line. Enters a critical section briefly —
/// fine because the operation is just struct copy + scalar zero-out.
pub fn snapshot_rx_stats() -> EthRxStats {
    with_rx_shared(|shared| {
        let out = shared.stats;
        shared.stats = EthRxStats::default();
        out
    })
}

/// Stat deltas accumulated locally (lock-free) while decoding a half, then
/// merged into the shared stats once at the end under a single brief lock.
/// Keeping these off the shared struct during the decode is what lets the
/// ≤2.57 ms decode run without holding `Spinlock<0>`.
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

/// Stitch the just-finished half, walk every active run, decode + verify, and
/// publish FCS-OK frames to the shared inbox. Runs on **core 1** in the
/// `DMA_IRQ_0` handler. The decode itself touches only the core-1-exclusive
/// `engine` (no lock); `Spinlock<0>` is taken only to push each frame and to
/// merge the stat deltas at the end — never across the decode.
fn process_completed_half(engine: &mut RxEngine) {
    // DIAG: did the RX FIFO overflow since the last check? (write-1-to-clear)
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
        // Ring full: decode backlog. The DMA must STILL be serviced (carry
        // bookkeeping + re-arm), so capture into the discard slot and drop
        // the image — bounded, counted overload behavior.
        // Safety: the discard slot is producer-exclusive (IRQ context only).
        let slot = unsafe { &mut *IMG_DISCARD.0.get() };
        if let crate::eth_rx::PollOutcome::Image { .. } = engine.rx.poll_into(&mut slot[..]) {
            IMG_DROP.fetch_add(1, Ordering::Relaxed);
        }
        return;
    }
    let slot_idx = (w as usize) % IMG_SLOTS;
    // Safety: producer-exclusive slot (w - r < IMG_SLOTS checked above);
    // the consumer only reads slots < w.
    let slot = unsafe { &mut (*IMG_RING.0.get())[slot_idx] };
    match engine.rx.poll_into(&mut slot[..]) {
        crate::eth_rx::PollOutcome::Nothing => {}
        crate::eth_rx::PollOutcome::Image { len, carry_prefix } => {
            IMG_LEN[slot_idx].store(len as u32, Ordering::Relaxed);
            IMG_CL[slot_idx].store(carry_prefix as u32, Ordering::Relaxed);
            IMG_W.store(w.wrapping_add(1), Ordering::Release);
        }
    }
    // Carry-cap accounting (engine-owned counter): merge under the brief
    // cross-core lock from IRQ context — cheap, bounded.
    let capped = engine.rx.take_carry_capped();
    if capped != 0 {
        with_rx_shared(|shared| {
            shared.stats.carry_capped = shared.stats.carry_capped.wrapping_add(capped);
        });
    }
}

/// Core 1 thread context: scan + decode any queued images. Called from the
/// core-1 main loop after each WFI wake. No deadline — the IRQ side keeps
/// the DMA fed regardless of how long a decode takes here.
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
        // our_mac: stable after install_rx; read it without touching the
        // engine (which is IRQ-owned).
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

/// Walk every frame-shaped active run in `bytes`, decode + verify the ones
/// addressed to us, publish FCS-OK frames to the shared inbox, and tally
/// stat deltas into `acc`. Shared by the half-completion path (the slices
/// `poll_with` hands out) and the live-decode path (a terminated-run window
/// of the in-flight half).
fn scan_runs(our_mac: &[u8; 6], bytes: &[u8], stitch_cl: Option<usize>, acc: &mut StatsDelta) {
    let mut cursor = 0;
    while let Some((off, len)) = EthRx::find_active_run_from(bytes, cursor, 100) {
        cursor = off + len;
        // MAC filter first — cheap peek (~1–2 µs) that skips the full
        // decode + CRC + push for frames not addressed to us. We accept
        // unicast-to-us + all multicast/broadcast.
        let Some(dst) = EthRx::peek_dst_mac(bytes, off, len) else {
            continue;
        };
        if !mac_accept(&dst, our_mac) {
            acc.filtered = acc.filtered.wrapping_add(1);
            continue;
        }
        // Manchester decoder: by default the edge-track DPLL (productized
        // in R10 — re-anchors to each per-bit mid-bit transition so
        // accumulated clock drift can't walk the sample point off). With
        // `--features decoder-openloop` the pre-R10 fixed-stride open-loop
        // decoder is used instead, for FCS-ceiling A/B vs Niccle. See
        // triage plan.
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

/// Brief cross-core lock: merge the decode-side stat deltas + last-frame
/// snapshot in one shot.
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


/// DMA channel-completion IRQ. Linker-resolved via `extern "Rust"` in
/// `rp235x-hal::arch`; needs `#[unsafe(no_mangle)]` for the symbol name to
/// match. Both RX DMA channels were `enable_irq0()`'d in `EthRx::new`, so this
/// fires once per half-buffer fill.
///
/// **Runs on core 1 (R12c).** Only core 1 unmasks `DMA_IRQ_0` in its xh3irq;
/// core 0 never sees it. The handler touches the core-1-exclusive `RX_ENGINE`
/// directly (no lock) and publishes into `RX_SHARED` under `Spinlock<0>`.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn DMA_IRQ_0() {
    // Safety: RX_ENGINE is touched only by this handler after `install_rx`
    // (which completes on core 0 before core 1 enables the IRQ), so this
    // `&mut` is exclusive.
    let Some(engine) = (unsafe { (*RX_ENGINE.0.get()).as_mut() }) else {
        return;
    };
    // Clear the per-channel pending bit. Do NOT gate processing on it: after
    // an overload deferral the Transfer's active-channel accounting can lag
    // the pending bits by one half, and `poll_into`'s is_done() is the
    // source of truth anyway.
    let _ = engine.rx.dma_irq_pending();
    // Perf step 2 (router build): bracket the decode so core 1's RX utilisation
    // is readable off `mcycle`. The span drops at function exit, right after the
    // decode returns; cost is negligible vs the ≤2.57 ms pipeline, and it's
    // absent from the production NIC build (its proven hot path stays unchanged).
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
        // Pop one frame from the core-1-populated inbox. The `Spinlock<0>`
        // hold is microseconds — only the inbox `pop_front` happens inside it.
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
        // smoltcp clamps the advertised TCP receive window to
        // `max_burst_size × MSS` (iface/packet.rs), so this is really the
        // RX-of-bulk pipelining knob (it does NOT limit TX). Measured on the
        // wired rig (2026-06-10, with the immediate-ACK sink in main.rs):
        //   Some(1) → ~135 KB/s  (serialized: one segment per ACK round-trip;
        //                         also keeps the host at sub-MTU segments)
        //   Some(2) → ~183 KB/s  (host pipelines full-MTU segments; the ~27%
        //                         full-MTU decode loss is absorbed by TCP
        //                         fast-retransmit instead of stalling)
        //   Some(4) → ~178 KB/s  (no further gain, more loss: ~32% FCS-fail)
        // Some(2) is BDP-matched for the 10BASE-T half-duplex link; going
        // wider only adds contention/decode loss without throughput.
        caps.max_burst_size = Some(2);
        caps
    }
}

/// RX token: owns the decoded Ethernet frame (no lifetime parameter —
/// the buffer was moved out of the shared inbox via `pop_front`).
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

/// TX token: borrows the TX state machine and the scratch buffer from
/// EthMac. `consume(len, f)` exposes a `&mut [u8]` of `len` bytes for
/// smoltcp to fill with a complete Ethernet frame body (dst MAC..end of
/// payload), then ships it via [`EthTx::send_raw_frame`] which prepends
/// preamble+SFD and appends the FCS.
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
        // `len` can be a full Ethernet frame: smoltcp's own egress is capped at the
        // IP MTU, but *forwarded* frames (R17 NAPT) pass the raw frame length here,
        // up to MAX_FRAME_BYTES. The buffer is sized to MAX_FRAME_BYTES, and the
        // forwarding `Frame` is `Vec<_, FRAME_CAP=MAX_FRAME_BYTES>`, so this never
        // exceeds bounds — but clamp defensively so an oversized `len` can never
        // panic the router (vs. the old `[u8; MTU]` buffer, which a 1514 B forwarded
        // frame overflowed → halt).
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
