//! EthMac — bridge between {EthTx, EthRx} and smoltcp's `phy::Device`.
//!
//! After R6 (IRQ-driven RX), responsibilities are split:
//!
//! - **EthMac** owns just `EthTx` + the TX scratch buffer + TX stats. It
//!   implements `smoltcp::phy::Device`; `transmit` hands out a `TxToken`
//!   that uses the local TX state directly, and `receive` reaches into
//!   the shared RX inbox (below) via a critical section.
//!
//! - **`SHARED_RX`** is a module-level `Mutex<RefCell<Option<EthRxShared>>>`
//!   holding the `EthRx` state machine + the inbox deque + RX-side stats.
//!   The `DMA_IRQ_0` handler (defined here) enters the critical section,
//!   runs the stitch + decode + verify pipeline, and pushes FCS-OK frames
//!   to the inbox. Main loop reads + resets stats via `snapshot_rx_stats`.
//!
//! Pass-through TX (`send_nlp`, `send_udp_broadcast`) on `EthMac` still
//! works alongside smoltcp for the NLP keepalive + smoke-test UDP loop.

use crate::eth_rx::EthRx;
use crate::eth_tx::{EthTx, UdpEndpoint};

use core::cell::RefCell;
use critical_section::Mutex;
use heapless::{Deque, Vec};
use smoltcp::phy::{self, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

pub const MTU: usize = 1500;
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

/// RX state owned by the DMA_IRQ_0 handler. Mutated only inside a
/// critical section; main loop touches it through `snapshot_rx_stats`
/// (for the 1 Hz log) and `Device::receive` (to pop from the inbox).
pub struct EthRxShared {
    rx: EthRx,
    inbox: Deque<Vec<u8, MAX_FRAME_BYTES>, INBOX_SLOTS>,
    /// Our 6-byte MAC. Used by the IRQ-side filter to skip frames not
    /// addressed to us before paying for the full decode + CRC + push.
    our_mac: [u8; 6],
    pub stats: EthRxStats,

    /// Phase 3b diagnostic — most recent FCS-failed frame, for off-device
    /// per-byte error analysis. Updated in the IRQ; copy-out via
    /// `snapshot_diag_failed`. Always present (small RAM cost vs the
    /// existing 4-slot inbox); only meaningfully consumed under
    /// `--features dpll`.
    diag_fail_data: [u8; MAX_FRAME_BYTES],
    diag_fail_len: usize,
    diag_fail_id: u32,
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

impl Default for EthRxStats {
    fn default() -> Self {
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

/// The shared RX state. Initially `None` — main installs an `EthRxShared`
/// via [`install_rx`] after constructing `EthRx`, before unmasking the
/// DMA IRQ. The IRQ handler refuses to run if this is still `None`.
static SHARED_RX: Mutex<RefCell<Option<EthRxShared>>> = Mutex::new(RefCell::new(None));

/// Move an `EthRx` into the shared static, along with our MAC for the
/// IRQ-side MAC filter. Call once, after constructing `EthRx` and before
/// unmasking `DMA_IRQ_0`. Returns `false` if `SHARED_RX` was already
/// populated (shouldn't happen — programmer error).
pub fn install_rx(rx: EthRx, our_mac: [u8; 6]) -> bool {
    critical_section::with(|cs| {
        let mut slot = SHARED_RX.borrow_ref_mut(cs);
        if slot.is_some() {
            return false;
        }
        slot.replace(EthRxShared {
            rx,
            inbox: Deque::new(),
            our_mac,
            stats: EthRxStats::default(),
            diag_fail_data: [0; MAX_FRAME_BYTES],
            diag_fail_len: 0,
            diag_fail_id: 0,
        });
        true
    })
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

/// Phase 3b diagnostic — copy the most recent FCS-failed frame into `out`.
/// Returns `Some((frame_id, len))` if there's a failed frame to dump
/// (frame_id is a monotonically increasing counter — callers track the
/// last-dumped id to avoid duplicate dumps). `None` if no failure has
/// occurred yet, or if the shared RX isn't installed.
pub fn snapshot_diag_failed(out: &mut [u8]) -> Option<(u32, usize)> {
    critical_section::with(|cs| {
        let slot = SHARED_RX.borrow_ref(cs);
        let shared = slot.as_ref()?;
        if shared.diag_fail_len == 0 {
            return None;
        }
        let n = shared.diag_fail_len.min(out.len());
        out[..n].copy_from_slice(&shared.diag_fail_data[..n]);
        Some((shared.diag_fail_id, n))
    })
}

/// Snapshot + reset the IRQ-managed RX stats. Called by the main loop
/// every second for the log line. Enters a critical section briefly —
/// fine because the operation is just struct copy + scalar zero-out.
pub fn snapshot_rx_stats() -> EthRxStats {
    critical_section::with(|cs| {
        let mut slot = SHARED_RX.borrow_ref_mut(cs);
        let Some(shared) = slot.as_mut() else {
            return EthRxStats::default();
        };
        let out = shared.stats;
        shared.stats = EthRxStats::default();
        out
    })
}

impl EthRxShared {
    /// Body of what `EthMac::poll` used to do, but now driven from the
    /// DMA_IRQ_0 handler instead of the main loop. Stitches the just-
    /// finished half, walks every active run, decodes + verifies + pushes
    /// FCS-OK frames to the inbox. Stats live in `self.stats` and are
    /// drained by the main loop via `snapshot_rx_stats`.
    fn process_completed_half(&mut self) {
        let inbox = &mut self.inbox;
        let stats = &mut self.stats;
        let our_mac = self.our_mac;
        let diag_fail_data = &mut self.diag_fail_data;
        let diag_fail_len = &mut self.diag_fail_len;
        let diag_fail_id = &mut self.diag_fail_id;
        self.rx.poll_with(|bytes| {
            let mut cursor = 0;
            while let Some((off, len)) = EthRx::find_active_run_from(bytes, cursor, 100) {
                cursor = off + len;
                // MAC filter first — cheap peek (~1–2 µs) that skips the
                // full decode + CRC + push for frames not addressed to
                // us. We accept unicast-to-us + all multicast/broadcast.
                let Some(dst) = EthRx::peek_dst_mac(bytes, off, len) else {
                    continue;
                };
                if !mac_accept(&dst, &our_mac) {
                    stats.frames_filtered = stats.frames_filtered.wrapping_add(1);
                    continue;
                }
                // Phase 3b: edge-track DPLL when `--features dpll`, open-loop otherwise.
                #[cfg(not(feature = "dpll"))]
                let Some(mut frame) = EthRx::decode_frame(bytes, off, len) else {
                    continue;
                };
                #[cfg(feature = "dpll")]
                let Some(mut frame) = crate::eth_rx_dpll::decode_frame_edge_track(
                    &bytes[off..off + len],
                ) else {
                    continue;
                };
                let flen = EthRx::derive_frame_len(&frame);
                let ok = EthRx::verify_fcs(&frame, flen);
                let n = flen.min(frame.len());

                stats.frames_decoded = stats.frames_decoded.wrapping_add(1);
                if ok {
                    stats.fcs_ok = stats.fcs_ok.wrapping_add(1);
                } else {
                    stats.fcs_fail = stats.fcs_fail.wrapping_add(1);
                }
                let snap_n = n.min(stats.last_frame_snapshot.len());
                stats.last_frame_snapshot[..snap_n].copy_from_slice(&frame[..snap_n]);
                stats.last_frame_snapshot_len = snap_n;
                stats.last_frame_len = n;
                stats.last_frame_was_ok = ok;

                if !ok {
                    // Phase 3b diagnostic: capture full failed-frame bytes
                    // (up to MAX_FRAME_BYTES) for off-device per-byte error
                    // analysis. Main loop polls this via snapshot_diag_failed
                    // and dumps over UDP when --features dpll is on.
                    let n_diag = frame.len().min(diag_fail_data.len());
                    diag_fail_data[..n_diag].copy_from_slice(&frame[..n_diag]);
                    *diag_fail_len = n_diag;
                    *diag_fail_id = (*diag_fail_id).wrapping_add(1);
                    continue;
                }
                frame.truncate(n);
                if inbox.is_full() {
                    let _ = inbox.pop_front();
                    stats.inbox_dropped = stats.inbox_dropped.wrapping_add(1);
                }
                let _ = inbox.push_back(frame);
                let depth = inbox.len() as u8;
                if depth > stats.inbox_high_water {
                    stats.inbox_high_water = depth;
                }
            }
        });
        // Fold the EthRx-local carry-cap counter into our window stats.
        stats.carry_capped = stats.carry_capped.wrapping_add(self.rx.take_carry_capped());
    }
}

/// DMA channel-completion IRQ. Linker-resolved via `extern "Rust"` in
/// `rp235x-hal::arch`; needs `#[unsafe(no_mangle)]` for the symbol name
/// to match. Both RX DMA channels were `enable_irq0()`'d in `EthRx::new`,
/// so this fires once per half-buffer fill.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn DMA_IRQ_0() {
    critical_section::with(|cs| {
        let mut slot = SHARED_RX.borrow_ref_mut(cs);
        let Some(shared) = slot.as_mut() else {
            return;
        };
        // `dma_irq_pending` clears the per-channel pending bit. Return
        // early if it's not actually ours (shouldn't happen — we own
        // both channels — but cheap defensive check).
        if !shared.rx.dma_irq_pending() {
            return;
        }
        shared.process_completed_half();
    });
}

pub struct EthMac {
    tx: EthTx,
    tx_buf: [u8; MTU],
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
            tx_buf: [0; MTU],
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
        // Pop one frame from the IRQ-populated inbox. Critical section is
        // microseconds — only the inbox::pop_front happens inside it.
        let buf = critical_section::with(|cs| {
            SHARED_RX
                .borrow_ref_mut(cs)
                .as_mut()
                .and_then(|s| s.inbox.pop_front())
        })?;
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
        caps.max_burst_size = Some(1);
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
    buf: &'a mut [u8; MTU],
    stats: &'a mut EthMacStats,
}

impl<'a> phy::TxToken for EthTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        debug_assert!(len <= MTU, "smoltcp asked for {len} bytes > MTU {MTU}");
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
