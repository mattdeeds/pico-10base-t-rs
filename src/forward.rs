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

use crate::conntrack; // R17 — NAPT connection tracking

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

/// Whether `ip`'s MAC is already in the WAN neighbor table — lets `wan_task`
/// stop re-ARPing the gateway once the table is warm (R19 cold-start fix).
pub fn wan_neigh_known(ip: Ipv4Address) -> bool {
    critical_section::with(|cs| WAN_NEIGH.borrow_ref(cs).lookup(ip).is_some())
}

/// Build a broadcast ARP request ("who-has `tpa`, tell `spa`") as a 42-byte
/// Ethernet/IPv4 frame. `EthMac::send_raw_frame` pads it to the 60-byte minimum
/// and appends the FCS. The byte layout matches [`learn`]'s ARP parser, so the
/// target's *reply* repopulates the neighbor table. R19 cold-start pre-warm.
fn build_arp_request(our_mac: [u8; 6], spa: Ipv4Address, tpa: Ipv4Address) -> [u8; 42] {
    let mut f = [0u8; 42];
    f[0..6].copy_from_slice(&[0xff; 6]); // dst MAC = broadcast
    f[6..12].copy_from_slice(&our_mac); // src MAC
    f[12..14].copy_from_slice(&ETHERTYPE_ARP.to_be_bytes());
    f[14..20].copy_from_slice(&[0x00, 0x01, 0x08, 0x00, 0x06, 0x04]); // htype/ptype/hlen/plen
    f[20..22].copy_from_slice(&1u16.to_be_bytes()); // oper = request
    f[22..28].copy_from_slice(&our_mac); // sha
    f[28..32].copy_from_slice(&spa.octets()); // spa (sender = us)
    // tha (f[32..38]) left zero — unknown, that's what we're asking for
    f[38..42].copy_from_slice(&tpa.octets()); // tpa (target = the gateway)
    f
}

// =====================================================================
// R17 — NAPT: the single conntrack table + L4 parse / src-dst rewrite
// =====================================================================

/// The one NAPT conntrack table, owned by the WAN `ForwardingDevice`. Touched only
/// from `wan_task` (core 0); the `critical_section` matches the neighbor-table
/// pattern and guards against the TIMER0 IRQ. The LAN device never touches it, so
/// it carries no per-device cost (vs. an `Option<Conntrack>` field).
static WAN_CT: Mutex<RefCell<conntrack::Conntrack>> =
    Mutex::new(RefCell::new(conntrack::Conntrack::new()));

/// Periodic idle-entry sweep + the live count, for `wan_task` / the `[Nat]` line.
pub fn nat_reap(now_ms: u64) {
    critical_section::with(|cs| WAN_CT.borrow_ref_mut(cs).reap(now_ms));
}

const IPPROTO_ICMP: u8 = 1;
const IPPROTO_TCP: u8 = 6;
const IPPROTO_UDP: u8 = 17;

/// Where a frame's L4 ports/id + checksum live (offsets honor the IHL), plus the
/// proto + TCP flags. `src_id`/`dst_id` are TCP/UDP ports, or the ICMP echo id.
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

/// NAPT the **source**: IP src → `new_ip`, L4 src port / ICMP id → `new_id`. Fixes
/// the L4 checksum incrementally; the IPv4 header checksum is recomputed by
/// `egress()` after TTL--, so it's left untouched here.
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

/// NAPT the **destination**: IP dst → `new_ip`, L4 dst port / ICMP id → `new_id`.
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

/// Wraps an inner `phy::Device` (cyw43 `Cyw43Phy` or 10BT `EthMac`) and the
/// channel this interface diverts *to*. `iface.poll` drives `receive`, which
/// skims transit frames into the egress channel and replays only local frames
/// to smoltcp.
pub struct ForwardingDevice<D: Device> {
    inner: D,
    cfg: IfaceCfg,
    /// The channel transit frames from *this* interface are pushed onto.
    egress: &'static FwdChannel,
    /// R17: this is the WAN device → do NAPT (via the `WAN_CT` static) on the
    /// transit path. The LAN device leaves this `false` and just forwards.
    nat: bool,
}

impl<D: Device> ForwardingDevice<D> {
    /// Plain L3-forwarding device (LAN side): no NAT.
    pub fn new(inner: D, cfg: IfaceCfg, egress: &'static FwdChannel) -> Self {
        Self { inner, cfg, egress, nat: false }
    }

    /// NAPT device (WAN side): rewrites src on egress + does conntrack-aware
    /// ingress classification. Shares the single `WAN_CT` table.
    pub fn new_napt(inner: D, cfg: IfaceCfg, egress: &'static FwdChannel) -> Self {
        Self { inner, cfg, egress, nat: true }
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

    /// R19: proactively ARP this interface's gateway so its next-hop MAC lands in
    /// the neighbor table (via the passive [`learn`] on the reply) *before* the
    /// first forwarded frame — eliminating the cold-start `[Fwd] drop` while
    /// `WAN_NEIGH` is still empty. No-op until a gateway + IP are leased. The owner
    /// (`wan_task`) calls this once per second until [`wan_neigh_known`] is true.
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

    /// Re-emit one forwarded frame (drained from the *ingress* channel by the
    /// owning task) out this interface: decrement TTL, refresh the IPv4 header
    /// checksum, resolve the next-hop MAC, rewrite the L2 header, and TX via the
    /// inner phy's normal token (EthMac → FCS/IFG/CSMA; cyw43 → NetDriver).
    pub fn egress(&mut self, frame: &mut Frame, now: Instant) {
        // R17: NAPT the source on the way out the WAN (LAN→WAN). Rewrite IP src →
        // our WAN IP + L4 src port/id → an allocated WAN id, tracked in conntrack so
        // the reply can be matched back. The IP-header checksum fixup is handled by
        // the `fill_checksum()` below (after TTL--); we only fix the L4 checksum.
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
                        FWD_DROP.fetch_add(1, Ordering::Relaxed); // port/id exhaustion
                        return;
                    }
                }
            }
            // non-IPv4 / non-TCP-UDP-ICMP transit forwards unmodified (rare).
        }

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

    /// Classify an ingress frame. On the WAN (NAPT) device, a reply addressed to
    /// our WAN IP that matches a tracked flow is a NAT-return: rewrite the dst back
    /// to the LAN client (in place) and forward it (`Transit`). Everything else
    /// falls through to the plain R16 classifier — so a conntrack *miss* on a frame
    /// to our WAN IP stays `Local` (the Pico's own ping/DNS), untouched.
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
                    // Inbound key: the WAN peer is the *source*; our allocated id is
                    // the *dst* port (TCP/UDP) or the echo id (ICMP).
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
        // Phase 1 — skim transit/drop frames (so they don't stall local ones
        // queued behind them) until the inbox yields a *local* frame or empties.
        // Each iteration's inner `tx` (a reply token) is dropped in-iteration; we
        // must NOT return it from the loop, or its `&mut self.inner` borrow would
        // collide with the next iteration's `receive` (no Polonius on stable).
        let frame = loop {
            let mut frame: Frame = Vec::new();
            {
                // Scope the RX/TX tokens so the inner-device borrow ends before
                // classify_frame borrows `&self` (the skim phase never replies).
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
