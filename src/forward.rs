//! R16 — L3 forwarding (LAN ↔ WAN transit, no NAT).
//!
//! smoltcp is an endpoint stack: a frame arriving with our gateway MAC but a
//! dst-IP that isn't ours is silently dropped, and its neighbor cache is
//! private. So forwarding is custom code beside the two `Interface`s.
//!
//! [`ForwardingDevice<D>`] wraps each phy and is what `iface.poll` pulls from.
//! On `receive()` it peeks every frame and either replays it to smoltcp (local:
//! ARP / our-IP / broadcast / the stack's own sockets) or **diverts** it to the
//! other interface via an [`embassy_sync`] channel. Each task drains the channel
//! pointing at *its* interface and re-emits the frame ([`ForwardingDevice::egress`]),
//! rewriting the L2 header + decrementing TTL. Next-hop MACs come from a small
//! per-interface table, **passively learned** from on-subnet source addresses.
//!
//! Both tasks run on the one core-0 executor (cooperative), so the shared
//! channels + tables are guarded by `critical_section` (which also covers the
//! TIMER0 IRQ). NAPT/conntrack is R17. Full design: `docs/r16-plan.md`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::Vec;
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::time::Instant;
use smoltcp::wire::{Ipv4Address, Ipv4Cidr, Ipv4Packet};

/// Max bytes of a forwarded L2 frame (matches `eth_mac::MAX_FRAME_BYTES`).
pub const FRAME_CAP: usize = 1600;
/// Per-direction forward-queue depth.
const CHAN_DEPTH: usize = 4;

/// A captured L2 frame in flight between the two interfaces.
pub type Frame = Vec<u8, FRAME_CAP>;
type FwdChannel = Channel<CriticalSectionRawMutex, Frame, CHAN_DEPTH>;

/// Frames the LAN side diverted, awaiting egress out the WAN (10BT) phy.
pub static LAN_TO_WAN: FwdChannel = Channel::new();
/// Frames the WAN side diverted, awaiting egress out the LAN (cyw43) phy.
pub static WAN_TO_LAN: FwdChannel = Channel::new();

// Telemetry — surfaced in the `[Wan]`/`[Cyw43]` CDC lines.
pub static FWD_L2W: AtomicU32 = AtomicU32::new(0); // diverted LAN→WAN (enqueued)
pub static FWD_W2L: AtomicU32 = AtomicU32::new(0); // diverted WAN→LAN (enqueued)
pub static FWD_SENT: AtomicU32 = AtomicU32::new(0); // egressed (TX'd out the other phy)
pub static FWD_DROP: AtomicU32 = AtomicU32::new(0); // dropped (queue full / no next-hop / TTL / TX busy)

/// Which interface a [`ForwardingDevice`] / neighbor table belongs to.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Iface {
    Lan,
    Wan,
}

// =====================================================================
// Per-interface neighbor table (passive, on-subnet learning)
// =====================================================================

const NEIGH_CAP: usize = 8;

/// A tiny fixed `IPv4 → MAC` map, learned by snooping on-subnet source
/// addresses. Round-robin overwrite when full.
struct NeighborTable {
    ip: [Ipv4Address; NEIGH_CAP],
    mac: [[u8; 6]; NEIGH_CAP],
    used: usize,
    next: usize,
}

impl NeighborTable {
    const fn new() -> Self {
        Self {
            ip: [Ipv4Address::UNSPECIFIED; NEIGH_CAP],
            mac: [[0u8; 6]; NEIGH_CAP],
            used: 0,
            next: 0,
        }
    }

    fn insert(&mut self, ip: Ipv4Address, mac: [u8; 6]) {
        for i in 0..self.used {
            if self.ip[i] == ip {
                self.mac[i] = mac;
                return;
            }
        }
        let slot = if self.used < NEIGH_CAP {
            let s = self.used;
            self.used += 1;
            s
        } else {
            let s = self.next;
            self.next = (self.next + 1) % NEIGH_CAP;
            s
        };
        self.ip[slot] = ip;
        self.mac[slot] = mac;
    }

    fn lookup(&self, ip: Ipv4Address) -> Option<[u8; 6]> {
        (0..self.used)
            .find(|&i| self.ip[i] == ip)
            .map(|i| self.mac[i])
    }
}

static LAN_NEIGH: Mutex<RefCell<NeighborTable>> = Mutex::new(RefCell::new(NeighborTable::new()));
static WAN_NEIGH: Mutex<RefCell<NeighborTable>> = Mutex::new(RefCell::new(NeighborTable::new()));

fn neigh(iface: Iface) -> &'static Mutex<RefCell<NeighborTable>> {
    match iface {
        Iface::Lan => &LAN_NEIGH,
        Iface::Wan => &WAN_NEIGH,
    }
}

// =====================================================================
// Per-interface forwarding config + frame classification
// =====================================================================

/// Static config for one interface's [`ForwardingDevice`]. `Copy` so the owning
/// task can update it (e.g. the WAN gateway as DHCP re-leases).
#[derive(Clone, Copy)]
pub struct IfaceCfg {
    pub iface: Iface,
    /// This interface's MAC — the egress L2 source + the "addressed to us" test.
    pub our_mac: [u8; 6],
    /// This interface's IP (frames to it are local). `UNSPECIFIED` until leased.
    pub our_ip: Ipv4Address,
    /// This interface's subnet — for on-subnet neighbor learning + egress next-hop.
    pub subnet: Ipv4Cidr,
    /// This interface's gateway — egress next-hop for off-subnet dsts (WAN only).
    pub gateway: Option<Ipv4Address>,
    /// Ingress routing filter: divert a transit frame only if its dst is in this
    /// subnet. `None` = divert any off-local dst (the LAN side → everything to WAN).
    pub accept_dst: Option<Ipv4Cidr>,
}

enum Class {
    /// Hand to smoltcp (ARP, broadcast/multicast, our-IP, the stack's sockets).
    Local,
    /// Forward out the other interface.
    Transit,
    /// Addressed to us at L2 but not routable here — drop.
    Drop,
}

const ETHERTYPE_IPV4: u16 = 0x0800;
const ETHERTYPE_ARP: u16 = 0x0806;

/// IPv4 dst address of an Ethernet frame, if it is a long-enough IPv4 frame.
fn ipv4_dst(frame: &[u8]) -> Option<Ipv4Address> {
    if frame.len() < 14 + 20 || u16::from_be_bytes([frame[12], frame[13]]) != ETHERTYPE_IPV4 {
        return None;
    }
    Some(Ipv4Address::new(frame[30], frame[31], frame[32], frame[33])) // L2(14) + IP dst(16)
}

fn classify(cfg: &IfaceCfg, frame: &[u8]) -> Class {
    // Forwarding is off until this interface is configured (the WAN side has no
    // IP/route until DHCP leases) — pass everything to smoltcp until then.
    if frame.len() < 14 || cfg.our_ip.is_unspecified() {
        return Class::Local;
    }
    // Only unicast-to-our-MAC is a forwarding candidate. Broadcast / multicast /
    // ARP go to smoltcp (which answers ARP and the LAN DHCP/ICMP, etc.).
    if frame[0..6] != cfg.our_mac {
        return Class::Local;
    }
    let Some(dst) = ipv4_dst(frame) else {
        return Class::Local; // non-IPv4 unicast to us → smoltcp
    };
    if dst == cfg.our_ip {
        return Class::Local; // for our own stack
    }
    // Transit. Apply the ingress routing filter.
    match cfg.accept_dst {
        None => Class::Transit,
        Some(net) if net.contains_addr(&dst) => Class::Transit,
        Some(_) => Class::Drop,
    }
}

/// Learn `IP → MAC` for next-hop resolution, but only when the source is on this
/// interface's subnet — otherwise an off-subnet sender (e.g. an `8.8.8.8` reply,
/// which arrives with the *gateway's* MAC) would poison the table.
///
/// Learns from **ARP** (the reliable source — both request and reply advertise
/// sender IP + sender MAC, and ARP precedes IP traffic) *and* IPv4 source
/// addresses. ARP is what actually populates the gateway/client MACs, since the
/// useful IPv4 traffic (ping replies) carries off-subnet source IPs.
fn learn(cfg: &IfaceCfg, frame: &[u8]) {
    if frame.len() < 14 || cfg.our_ip.is_unspecified() {
        return;
    }
    let (src_ip, src_mac): (Ipv4Address, [u8; 6]) = match u16::from_be_bytes([frame[12], frame[13]])
    {
        // Ethernet/IPv4 ARP (fixed layout): htype=1 ptype=0x0800 hlen=6 plen=4,
        // then oper(2), sha(6)=frame[22..28], spa(4)=frame[28..32].
        ETHERTYPE_ARP
            if frame.len() >= 14 + 28
                && frame[14..20] == [0x00, 0x01, 0x08, 0x00, 0x06, 0x04] =>
        {
            (
                Ipv4Address::new(frame[28], frame[29], frame[30], frame[31]),
                frame[22..28].try_into().unwrap(),
            )
        }
        ETHERTYPE_IPV4 if frame.len() >= 14 + 20 => (
            Ipv4Address::new(frame[26], frame[27], frame[28], frame[29]), // L2(14)+IP src(12)
            frame[6..12].try_into().unwrap(),
        ),
        _ => return,
    };
    if !cfg.subnet.contains_addr(&src_ip) {
        return;
    }
    critical_section::with(|cs| neigh(cfg.iface).borrow_ref_mut(cs).insert(src_ip, src_mac));
}

/// Next hop for `dst` egressing this interface: the dst itself if on-subnet,
/// else the interface's gateway.
fn nexthop(dst: Ipv4Address, subnet: Ipv4Cidr, gateway: Option<Ipv4Address>) -> Option<Ipv4Address> {
    if subnet.contains_addr(&dst) {
        Some(dst)
    } else {
        gateway
    }
}

// =====================================================================
// ForwardingDevice<D> — the classifying phy::Device wrapper
// =====================================================================

/// Wraps an inner `phy::Device` (cyw43 `Cyw43Phy` or 10BT `EthMac`) and the
/// channel this interface diverts *to*. `iface.poll` drives `receive`, which
/// skims transit frames into the egress channel and replays only local frames
/// to smoltcp.
pub struct ForwardingDevice<D: Device> {
    inner: D,
    cfg: IfaceCfg,
    /// The channel transit frames from *this* interface are pushed onto.
    egress: &'static FwdChannel,
}

impl<D: Device> ForwardingDevice<D> {
    pub fn new(inner: D, cfg: IfaceCfg, egress: &'static FwdChannel) -> Self {
        Self { inner, cfg, egress }
    }

    /// Sync this interface's address / subnet / gateway from a DHCP lease (the
    /// WAN side calls this each time `wan::dhcp_apply` updates the lease). Setting
    /// a non-`UNSPECIFIED` `our_ip` also *enables* forwarding on this interface.
    pub fn set_lease(&mut self, cidr: Ipv4Cidr, gateway: Option<Ipv4Address>) {
        self.cfg.our_ip = cidr.address();
        self.cfg.subnet = cidr;
        self.cfg.gateway = gateway;
    }

    /// Access the inner phy (e.g. for `EthMac::send_nlp` — not part of `Device`).
    pub fn inner_mut(&mut self) -> &mut D {
        &mut self.inner
    }

    /// Re-emit one forwarded frame (drained from the *ingress* channel by the
    /// owning task) out this interface: decrement TTL, refresh the IPv4 header
    /// checksum, resolve the next-hop MAC, rewrite the L2 header, and TX via the
    /// inner phy's normal token (EthMac → FCS/IFG/CSMA; cyw43 → NetDriver).
    pub fn egress(&mut self, frame: &mut Frame, now: Instant) {
        // L3: TTL + checksum + capture dst (drop non-IPv4 / runt / TTL-expired).
        let dst = {
            let Ok(mut ip) = Ipv4Packet::new_checked(&mut frame[14..]) else {
                FWD_DROP.fetch_add(1, Ordering::Relaxed);
                return;
            };
            let ttl = ip.hop_limit();
            if ttl <= 1 {
                FWD_DROP.fetch_add(1, Ordering::Relaxed);
                return;
            }
            ip.set_hop_limit(ttl - 1);
            ip.fill_checksum();
            ip.dst_addr()
        };
        let Some(nh) = nexthop(dst, self.cfg.subnet, self.cfg.gateway) else {
            FWD_DROP.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let Some(dmac) = critical_section::with(|cs| neigh(self.cfg.iface).borrow_ref(cs).lookup(nh))
        else {
            FWD_DROP.fetch_add(1, Ordering::Relaxed); // next-hop MAC not learned yet
            return;
        };
        // L2: dst = next-hop MAC, src = this interface's MAC. (EtherType unchanged.)
        frame[0..6].copy_from_slice(&dmac);
        frame[6..12].copy_from_slice(&self.cfg.our_mac);

        let len = frame.len();
        if let Some(tx) = self.inner.transmit(now) {
            tx.consume(len, |buf| buf.copy_from_slice(&frame[..len]));
            FWD_SENT.fetch_add(1, Ordering::Relaxed);
        } else {
            FWD_DROP.fetch_add(1, Ordering::Relaxed);
        }
    }
}

impl<D: Device> Device for ForwardingDevice<D> {
    type RxToken<'a>
        = ReplayRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = D::TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, ts: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // Phase 1 — skim transit/drop frames (so they don't stall local ones
        // queued behind them) until the inbox yields a *local* frame or empties.
        // Each iteration's inner `tx` (a reply token) is dropped in-iteration; we
        // must NOT return it from the loop, or its `&mut self.inner` borrow would
        // collide with the next iteration's `receive` (no Polonius on stable).
        let frame = loop {
            let (rx, _tx) = self.inner.receive(ts)?;
            let mut frame: Frame = Vec::new();
            rx.consume(|buf| {
                let n = buf.len().min(FRAME_CAP);
                let _ = frame.extend_from_slice(&buf[..n]);
            });
            learn(&self.cfg, &frame);
            match classify(&self.cfg, &frame) {
                Class::Local => break frame,
                Class::Transit => {
                    if self.egress.try_send(frame).is_ok() {
                        match self.cfg.iface {
                            Iface::Lan => FWD_L2W.fetch_add(1, Ordering::Relaxed),
                            Iface::Wan => FWD_W2L.fetch_add(1, Ordering::Relaxed),
                        };
                    } else {
                        FWD_DROP.fetch_add(1, Ordering::Relaxed); // egress queue full
                    }
                }
                Class::Drop => {
                    FWD_DROP.fetch_add(1, Ordering::Relaxed);
                }
            }
        };
        // Phase 2 — hand the local frame to smoltcp with a *fresh* TX token for
        // its reply (ARP/ICMP/socket). EthMac::transmit is always `Some`; cyw43
        // can be `None` under TX pressure, in which case we drop this RX frame
        // (the upper layer retransmits) rather than stall.
        let tx = self.inner.transmit(ts)?;
        Some((ReplayRxToken { frame }, tx))
    }

    fn transmit(&mut self, ts: Instant) -> Option<Self::TxToken<'_>> {
        self.inner.transmit(ts) // smoltcp's own egress (ARP/ICMP/sockets) is unchanged
    }

    fn capabilities(&self) -> DeviceCapabilities {
        self.inner.capabilities()
    }
}

/// Owned RX token replaying a peeked local frame into smoltcp (no borrow, like
/// `eth_mac::EthRxToken`).
pub struct ReplayRxToken {
    frame: Frame,
}

impl RxToken for ReplayRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(&self.frame)
    }
}
