//! L3 forwarding between LAN and WAN, with NAPT on the WAN side.
//!
//! smoltcp is an endpoint stack, so forwarding is custom. [`ForwardingDevice`]
//! wraps each phy: `receive` passes local frames to smoltcp and diverts transit
//! frames to the other interface's channel; `egress` rewrites L2, decrements
//! TTL, and transmits. Next-hop MACs are learned passively.
//!
//! Both tasks share the core-0 executor; shared tables use `critical_section`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::Vec;
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::time::Instant;
use smoltcp::wire::{Ipv4Address, Ipv4Cidr, Ipv4Packet};

use crate::conntrack;

/// Max bytes of a forwarded L2 frame (matches `eth_mac::MAX_FRAME_BYTES`).
pub const FRAME_CAP: usize = 1600;
/// Per-direction forward-queue depth.
pub const CHAN_DEPTH: usize = 4;

/// A captured L2 frame in flight between the two interfaces.
pub type Frame = Vec<u8, FRAME_CAP>;
type FwdChannel = Channel<CriticalSectionRawMutex, Frame, CHAN_DEPTH>;

/// Frames the LAN side diverted, awaiting egress out the WAN (10BT) phy.
pub static LAN_TO_WAN: FwdChannel = Channel::new();
/// Frames the WAN side diverted, awaiting egress out the LAN (cyw43) phy.
pub static WAN_TO_LAN: FwdChannel = Channel::new();

// Telemetry — surfaced in the `[Wan]`/`[Cyw43]`/`[Perf]` CDC lines.
pub static FWD_L2W: AtomicU32 = AtomicU32::new(0); // diverted LAN→WAN (enqueued)
pub static FWD_W2L: AtomicU32 = AtomicU32::new(0); // diverted WAN→LAN (enqueued)
pub static FWD_SENT: AtomicU32 = AtomicU32::new(0); // egressed (TX'd out the other phy)
pub static FWD_DROP: AtomicU32 = AtomicU32::new(0); // total dropped (sum of the breakdown below)

// Egressed L2 bytes per direction. Wraps; read as 1 Hz deltas.
pub static FWD_BYTES_TO_WAN: AtomicU32 = AtomicU32::new(0);
pub static FWD_BYTES_TO_LAN: AtomicU32 = AtomicU32::new(0);
// `FWD_DROP` split by cause, so load tests show *why* frames drop.
pub static FWD_DROP_QFULL: AtomicU32 = AtomicU32::new(0); // egress channel full (backpressure)
pub static FWD_DROP_NONH: AtomicU32 = AtomicU32::new(0); // next-hop unresolved (no gw / no MAC)
pub static FWD_DROP_NAT: AtomicU32 = AtomicU32::new(0); // NAPT port/id exhaustion
pub static FWD_DROP_TXBUSY: AtomicU32 = AtomicU32::new(0); // inner phy TX not ready
pub static FWD_DROP_OTHER: AtomicU32 = AtomicU32::new(0); // malformed / runt / TTL-expired
// Max egress-queue depth seen per direction (vs `CHAN_DEPTH`) — a saturation signal.
pub static FWD_QHWM_L2W: AtomicU32 = AtomicU32::new(0);
pub static FWD_QHWM_W2L: AtomicU32 = AtomicU32::new(0);

/// Count a drop in both `FWD_DROP` and its reason counter.
fn count_drop(reason: &AtomicU32) {
    FWD_DROP.fetch_add(1, Ordering::Relaxed);
    reason.fetch_add(1, Ordering::Relaxed);
}

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

/// Forwarding config for one interface. The WAN side updates it per lease.
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
    /// Divert transit only to dsts in this subnet; `None` diverts all.
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
/// Ethernet/IPv4 ARP header prefix, shared by the ARP writer and parser.
const ARP_ETH_IPV4_PREFIX: [u8; 6] = [0x00, 0x01, 0x08, 0x00, 0x06, 0x04];

/// IPv4 dst address of an Ethernet frame, if it is a long-enough IPv4 frame.
fn ipv4_dst(frame: &[u8]) -> Option<Ipv4Address> {
    if frame.len() < 14 + 20 || u16::from_be_bytes([frame[12], frame[13]]) != ETHERTYPE_IPV4 {
        return None;
    }
    Some(Ipv4Address::new(frame[30], frame[31], frame[32], frame[33])) // L2(14) + IP dst(16)
}

fn classify(cfg: &IfaceCfg, frame: &[u8]) -> Class {
    // Until this interface has an IP, everything goes to smoltcp.
    if frame.len() < 14 || cfg.our_ip.is_unspecified() {
        return Class::Local;
    }
    // Only unicast to our MAC can be transit; the rest goes to smoltcp.
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

/// Learn IP → MAC from ARP and IPv4 sources on this subnet only.
/// Off-subnet sources carry the gateway's MAC and would poison the table.
fn learn(cfg: &IfaceCfg, frame: &[u8]) {
    if frame.len() < 14 || cfg.our_ip.is_unspecified() {
        return;
    }
    let (src_ip, src_mac): (Ipv4Address, [u8; 6]) = match u16::from_be_bytes([frame[12], frame[13]])
    {
        // ARP: sha = frame[22..28], spa = frame[28..32].
        ETHERTYPE_ARP
            if frame.len() >= 14 + 28 && frame[14..20] == ARP_ETH_IPV4_PREFIX =>
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

/// True if `ip`'s MAC is in the WAN neighbor table.
pub fn wan_neigh_known(ip: Ipv4Address) -> bool {
    critical_section::with(|cs| WAN_NEIGH.borrow_ref(cs).lookup(ip).is_some())
}

/// Broadcast ARP request "who-has `tpa`, tell `spa`" (42 bytes; TX pads it).
fn build_arp_request(our_mac: [u8; 6], spa: Ipv4Address, tpa: Ipv4Address) -> [u8; 42] {
    let mut f = [0u8; 42];
    f[0..6].copy_from_slice(&[0xff; 6]); // dst MAC = broadcast
    f[6..12].copy_from_slice(&our_mac); // src MAC
    f[12..14].copy_from_slice(&ETHERTYPE_ARP.to_be_bytes());
    f[14..20].copy_from_slice(&ARP_ETH_IPV4_PREFIX); // htype/ptype/hlen/plen
    f[20..22].copy_from_slice(&1u16.to_be_bytes()); // oper = request
    f[22..28].copy_from_slice(&our_mac); // sha
    f[28..32].copy_from_slice(&spa.octets()); // spa (sender = us)
    // tha (f[32..38]) left zero — unknown, that's what we're asking for
    f[38..42].copy_from_slice(&tpa.octets()); // tpa (target = the gateway)
    f
}

// =====================================================================
// NAPT: conntrack table, L4 parse, and address rewrite
// =====================================================================

/// The NAPT conntrack table, used only by the WAN device on `wan_task`.
static WAN_CT: Mutex<RefCell<conntrack::Conntrack>> =
    Mutex::new(RefCell::new(conntrack::Conntrack::new()));

/// Sweep idle NAPT entries. Called once a second by `wan_task`.
pub fn nat_reap(now_ms: u64) {
    critical_section::with(|cs| WAN_CT.borrow_ref_mut(cs).reap(now_ms));
}

const IPPROTO_ICMP: u8 = 1;
const IPPROTO_TCP: u8 = 6;
const IPPROTO_UDP: u8 = 17;

/// L4 offsets (IHL-aware), ids, and TCP flags. Ids are ports or the ICMP echo id.
struct L4 {
    proto: conntrack::Proto,
    l4_off: usize,
    src_id: u16,
    dst_id: u16,
    csum_off: usize,
    tcp_flags: u8,
}

fn parse_l4(frame: &[u8]) -> Option<L4> {
    if frame.len() < 14 + 20 || u16::from_be_bytes([frame[12], frame[13]]) != ETHERTYPE_IPV4 {
        return None;
    }
    let ihl = (frame[14] & 0x0f) as usize * 4;
    if ihl < 20 {
        return None;
    }
    let l4 = 14 + ihl;
    match frame[14 + 9] {
        IPPROTO_TCP if frame.len() >= l4 + 20 => Some(L4 {
            proto: conntrack::Proto::Tcp,
            l4_off: l4,
            src_id: u16::from_be_bytes([frame[l4], frame[l4 + 1]]),
            dst_id: u16::from_be_bytes([frame[l4 + 2], frame[l4 + 3]]),
            csum_off: l4 + 16,
            tcp_flags: frame[l4 + 13],
        }),
        IPPROTO_UDP if frame.len() >= l4 + 8 => Some(L4 {
            proto: conntrack::Proto::Udp,
            l4_off: l4,
            src_id: u16::from_be_bytes([frame[l4], frame[l4 + 1]]),
            dst_id: u16::from_be_bytes([frame[l4 + 2], frame[l4 + 3]]),
            csum_off: l4 + 6,
            tcp_flags: 0,
        }),
        // ICMP echo reply (0) / request (8): the identifier is the "port".
        IPPROTO_ICMP if frame.len() >= l4 + 8 && (frame[l4] == 0 || frame[l4] == 8) => Some(L4 {
            proto: conntrack::Proto::IcmpEcho,
            l4_off: l4,
            src_id: u16::from_be_bytes([frame[l4 + 4], frame[l4 + 5]]),
            dst_id: 0,
            csum_off: l4 + 2,
            tcp_flags: 0,
        }),
        _ => None,
    }
}

fn rd16(b: &[u8], i: usize) -> u16 {
    u16::from_be_bytes([b[i], b[i + 1]])
}
fn wr16(b: &mut [u8], i: usize, v: u16) {
    b[i..i + 2].copy_from_slice(&v.to_be_bytes());
}

/// Rewrite source IP and port/id, fixing the L4 checksum.
/// `egress` recomputes the IPv4 header checksum after the TTL change.
fn napt_rewrite_src(frame: &mut [u8], l4: &L4, new_ip: Ipv4Address, new_id: u16) {
    let old_ip = Ipv4Address::new(frame[26], frame[27], frame[28], frame[29]);
    match l4.proto {
        conntrack::Proto::Tcp | conntrack::Proto::Udp => {
            let old_port = rd16(frame, l4.l4_off);
            let old_csum = rd16(frame, l4.csum_off);
            // UDP checksum 0 == "none" → leave it disabled.
            if !(l4.proto == conntrack::Proto::Udp && old_csum == 0) {
                let (oh, ol) = conntrack::addr_words(old_ip);
                let (nh, nl) = conntrack::addr_words(new_ip);
                let c =
                    conntrack::checksum_incr(old_csum, &[(oh, nh), (ol, nl), (old_port, new_id)]);
                wr16(frame, l4.csum_off, c);
            }
            wr16(frame, l4.l4_off, new_id); // src port
        }
        conntrack::Proto::IcmpEcho => {
            let old_id = rd16(frame, l4.l4_off + 4);
            let old_csum = rd16(frame, l4.csum_off);
            let c = conntrack::checksum_incr(old_csum, &[(old_id, new_id)]);
            wr16(frame, l4.csum_off, c);
            wr16(frame, l4.l4_off + 4, new_id); // echo id
        }
    }
    frame[26..30].copy_from_slice(&new_ip.octets()); // IP src (read above, write last)
}

/// Rewrite destination IP and port/id, fixing the L4 checksum.
fn napt_rewrite_dst(frame: &mut [u8], l4: &L4, new_ip: Ipv4Address, new_id: u16) {
    let old_ip = Ipv4Address::new(frame[30], frame[31], frame[32], frame[33]);
    match l4.proto {
        conntrack::Proto::Tcp | conntrack::Proto::Udp => {
            let old_port = rd16(frame, l4.l4_off + 2);
            let old_csum = rd16(frame, l4.csum_off);
            if !(l4.proto == conntrack::Proto::Udp && old_csum == 0) {
                let (oh, ol) = conntrack::addr_words(old_ip);
                let (nh, nl) = conntrack::addr_words(new_ip);
                let c =
                    conntrack::checksum_incr(old_csum, &[(oh, nh), (ol, nl), (old_port, new_id)]);
                wr16(frame, l4.csum_off, c);
            }
            wr16(frame, l4.l4_off + 2, new_id); // dst port
        }
        conntrack::Proto::IcmpEcho => {
            let old_id = rd16(frame, l4.l4_off + 4);
            let old_csum = rd16(frame, l4.csum_off);
            let c = conntrack::checksum_incr(old_csum, &[(old_id, new_id)]);
            wr16(frame, l4.csum_off, c);
            wr16(frame, l4.l4_off + 4, new_id);
        }
    }
    frame[30..34].copy_from_slice(&new_ip.octets()); // IP dst
}

// =====================================================================
// ForwardingDevice<D> — the classifying phy::Device wrapper
// =====================================================================

/// Wraps a phy (`Cyw43Phy` or `EthMac`) and the channel it diverts transit to.
pub struct ForwardingDevice<D: Device> {
    inner: D,
    cfg: IfaceCfg,
    /// The channel transit frames from *this* interface are pushed onto.
    egress: &'static FwdChannel,
    /// WAN device: NAPT via `WAN_CT` on transit.
    nat: bool,
}

impl<D: Device> ForwardingDevice<D> {
    /// Plain L3-forwarding device (LAN side): no NAT.
    pub fn new(inner: D, cfg: IfaceCfg, egress: &'static FwdChannel) -> Self {
        Self { inner, cfg, egress, nat: false }
    }

    /// NAPT device (WAN side), using the shared `WAN_CT` table.
    pub fn new_napt(inner: D, cfg: IfaceCfg, egress: &'static FwdChannel) -> Self {
        Self { inner, cfg, egress, nat: true }
    }

    /// Set address, subnet, and gateway from a lease. A real IP enables forwarding.
    pub fn set_lease(&mut self, cidr: Ipv4Cidr, gateway: Option<Ipv4Address>) {
        self.cfg.our_ip = cidr.address();
        self.cfg.subnet = cidr;
        self.cfg.gateway = gateway;
    }

    /// Access the inner phy (e.g. for `EthMac::send_nlp` — not part of `Device`).
    pub fn inner_mut(&mut self) -> &mut D {
        &mut self.inner
    }

    /// ARP the gateway so its MAC is known before the first forwarded frame.
    /// No-op until a gateway and IP are leased.
    pub fn arp_gateway(&mut self, now: Instant) {
        let Some(gw) = self.cfg.gateway else {
            return;
        };
        if self.cfg.our_ip.is_unspecified() {
            return;
        }
        let arp = build_arp_request(self.cfg.our_mac, self.cfg.our_ip, gw);
        if let Some(tx) = self.inner.transmit(now) {
            tx.consume(arp.len(), |buf| buf.copy_from_slice(&arp));
        }
    }

    /// Send a forwarded frame out this interface: NAPT (WAN), TTL, checksum,
    /// next-hop MAC, L2 rewrite, then TX.
    pub fn egress(&mut self, frame: &mut Frame, now: Instant) {
        // Count forwarding cycles toward `FWD_BUSY`.
        let _cyc = crate::cycles::CycleSpan::new(&crate::cycles::FWD_BUSY);
        // WAN: NAPT the source and track the flow. L4 checksum is fixed here.
        if self.nat {
            if let Some(l4) = parse_l4(&frame[..]) {
                let src_ip = Ipv4Address::new(frame[26], frame[27], frame[28], frame[29]);
                let dst_ip = Ipv4Address::new(frame[30], frame[31], frame[32], frame[33]);
                // ICMP echo has no dst "port"; key the flow on its id alone.
                let dst_id = if l4.proto == conntrack::Proto::IcmpEcho { 0 } else { l4.dst_id };
                let tuple = conntrack::Tuple {
                    proto: l4.proto,
                    src_ip,
                    src_id: l4.src_id,
                    dst_ip,
                    dst_id,
                };
                let now_ms = now.total_millis().max(0) as u64;
                let wan_id = critical_section::with(|cs| {
                    WAN_CT.borrow_ref_mut(cs).outbound(&tuple, l4.tcp_flags, now_ms)
                });
                match wan_id {
                    Some(id) => napt_rewrite_src(&mut frame[..], &l4, self.cfg.our_ip, id),
                    None => {
                        count_drop(&FWD_DROP_NAT); // port/id exhaustion
                        return;
                    }
                }
            }
            // Other protocols forward unmodified.
        }

        // L3: TTL, checksum, and dst; drop runts and expired TTL.
        let dst = {
            let Ok(mut ip) = Ipv4Packet::new_checked(&mut frame[14..]) else {
                count_drop(&FWD_DROP_OTHER); // malformed / runt
                return;
            };
            let ttl = ip.hop_limit();
            if ttl <= 1 {
                count_drop(&FWD_DROP_OTHER); // TTL expired
                return;
            }
            ip.set_hop_limit(ttl - 1);
            ip.fill_checksum();
            ip.dst_addr()
        };
        let Some(nh) = nexthop(dst, self.cfg.subnet, self.cfg.gateway) else {
            count_drop(&FWD_DROP_NONH); // no gateway for an off-subnet dst
            return;
        };
        let Some(dmac) = critical_section::with(|cs| neigh(self.cfg.iface).borrow_ref(cs).lookup(nh))
        else {
            count_drop(&FWD_DROP_NONH); // next-hop MAC not learned yet
            return;
        };
        // L2: dst = next-hop MAC, src = this interface's MAC. (EtherType unchanged.)
        frame[0..6].copy_from_slice(&dmac);
        frame[6..12].copy_from_slice(&self.cfg.our_mac);

        let len = frame.len();
        if let Some(tx) = self.inner.transmit(now) {
            tx.consume(len, |buf| buf.copy_from_slice(&frame[..len]));
            FWD_SENT.fetch_add(1, Ordering::Relaxed);
            // Count egressed bytes by direction.
            match self.cfg.iface {
                Iface::Wan => FWD_BYTES_TO_WAN.fetch_add(len as u32, Ordering::Relaxed),
                Iface::Lan => FWD_BYTES_TO_LAN.fetch_add(len as u32, Ordering::Relaxed),
            };
        } else {
            count_drop(&FWD_DROP_TXBUSY);
        }
    }

    /// Classify an ingress frame. On WAN, a reply matching conntrack is rewritten
    /// to the LAN client and marked `Transit`; misses fall through to `classify`.
    fn classify_frame(&self, frame: &mut Frame, ts: Instant) -> Class {
        if self.nat {
            let cfg = self.cfg;
            if !cfg.our_ip.is_unspecified()
                && frame.len() >= 14 + 20
                && frame[0..6] == cfg.our_mac
                && ipv4_dst(&frame[..]) == Some(cfg.our_ip)
            {
                if let Some(l4) = parse_l4(&frame[..]) {
                    let remote_ip = Ipv4Address::new(frame[26], frame[27], frame[28], frame[29]);
                    // The WAN peer is the source; our id is the dst port or echo id.
                    let (remote_id, wan_id) = match l4.proto {
                        conntrack::Proto::IcmpEcho => (0, l4.src_id),
                        _ => (l4.src_id, l4.dst_id),
                    };
                    let now_ms = ts.total_millis().max(0) as u64;
                    let m = critical_section::with(|cs| {
                        WAN_CT.borrow_ref_mut(cs).inbound(
                            l4.proto,
                            remote_ip,
                            remote_id,
                            wan_id,
                            l4.tcp_flags,
                            now_ms,
                        )
                    });
                    if let Some((lan_ip, lan_id)) = m {
                        napt_rewrite_dst(&mut frame[..], &l4, lan_ip, lan_id);
                        return Class::Transit;
                    }
                }
            }
        }
        classify(&self.cfg, &frame[..])
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
        // Count ingress classify cycles toward `FWD_BUSY`. Drop runs on every exit.
        let _cyc = crate::cycles::CycleSpan::new(&crate::cycles::FWD_BUSY);
        // Skim transit and drop frames until a local frame arrives or the inbox empties.
        // Reply tokens drop each iteration; returning one would double-borrow `inner`.
        let frame = loop {
            let mut frame: Frame = Vec::new();
            {
                // Scope the tokens so the inner borrow ends before `classify_frame`.
                let (rx, _tx) = self.inner.receive(ts)?;
                rx.consume(|buf| {
                    let n = buf.len().min(FRAME_CAP);
                    let _ = frame.extend_from_slice(&buf[..n]);
                });
            }
            learn(&self.cfg, &frame);
            match self.classify_frame(&mut frame, ts) {
                Class::Local => break frame,
                Class::Transit => {
                    if self.egress.try_send(frame).is_ok() {
                        // Queue depth after enqueue → high-water (saturation signal).
                        let depth = self.egress.len() as u32;
                        match self.cfg.iface {
                            Iface::Lan => {
                                FWD_L2W.fetch_add(1, Ordering::Relaxed);
                                FWD_QHWM_L2W.fetch_max(depth, Ordering::Relaxed);
                            }
                            Iface::Wan => {
                                FWD_W2L.fetch_add(1, Ordering::Relaxed);
                                FWD_QHWM_W2L.fetch_max(depth, Ordering::Relaxed);
                            }
                        };
                    } else {
                        count_drop(&FWD_DROP_QFULL); // egress queue full
                    }
                }
                Class::Drop => {
                    count_drop(&FWD_DROP_OTHER);
                }
            }
        };
        // Replay the local frame with a fresh TX token. If TX isn't ready, drop it.
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

/// Owned RX token replaying a local frame into smoltcp.
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
