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
//!   handler** — so the long stitch + decode + verify pipeline runs with no
//!   lock at all.
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
/// Experiment (`docs/rx-bulk-ceiling.md` §5): clamp the advertised IP MTU so our
/// SYN/SYN-ACK advertises a small TCP MSS (≈ MTU−40) → peers send sub-knee
/// segments (on-wire frame ≈ MTU+26 B; the RX decode cliff starts ~600 B on-wire).
/// Tests whether keeping inbound frames below the clock-drift cliff lifts
/// RX-of-bulk past the ~100 KB/s ceiling. Only `max_transmission_unit` uses this
/// const (no buffers), so clamping is safe; default off → production unchanged.
/// 500 → MSS ≈ 460, on-wire ≈ 526 B (clean ~3-4 % decode region).
#[cfg(feature = "mss-clamp")]
pub const MTU: usize = 500;
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
    let our_mac = engine.our_mac;
    let mut acc = StatsDelta::new();

    engine.rx.poll_with(|bytes| {
        let mut cursor = 0;
        while let Some((off, len)) = EthRx::find_active_run_from(bytes, cursor, 100) {
            cursor = off + len;
            // MAC filter first — cheap peek (~1–2 µs) that skips the full
            // decode + CRC + push for frames not addressed to us. We accept
            // unicast-to-us + all multicast/broadcast.
            let Some(dst) = EthRx::peek_dst_mac(bytes, off, len) else {
                continue;
            };
            if !mac_accept(&dst, &our_mac) {
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
    });
    acc.carry = engine.rx.take_carry_capped();

    // Brief cross-core lock: merge the decode-side stat deltas + last-frame
    // snapshot in one shot.
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
    // `dma_irq_pending` clears the per-channel pending bit. Return early if
    // it's not actually ours (shouldn't happen — we own both channels — but
    // cheap defensive check).
    if !engine.rx.dma_irq_pending() {
        return;
    }
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
        // smoltcp clamps every advertised TCP receive window to
        // `max_burst_size × MSS` (iface/packet.rs). `Some(1)` therefore
        // advertised a ONE-segment window no matter how big the socket RX
        // buffer was, serializing bulk uploads to one segment per
        // (10 ms delayed-ACK + RTT) cycle — which is the ~100 KB/s
        // RX-of-bulk ceiling in docs/rx-bulk-ceiling.md (§5 numbers match
        // payload/13.5 ms at every MTU, see §9). INBOX_SLOTS segments is
        // what the decoded-frame inbox can actually buffer per burst, and
        // 4 × MSS ≈ 5.8 KB comfortably covers the 10 Mbit half-duplex
        // bandwidth-delay product (~3.75 KB at the measured 3 ms RTT).
        caps.max_burst_size = Some(INBOX_SLOTS);
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
