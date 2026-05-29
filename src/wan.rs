//! Shared WAN-as-DHCP-client logic (R15a/R15b).
//!
//! The 10BASE-T side acts as a DHCP *client*: it leases an upstream IP + default
//! route + DNS, then proves internet reachability by pinging 8.8.8.8 and
//! resolving a name out the wired link. NAPT/forwarding stay R16/R17 — this is
//! the router box being a *client*, not routing other clients' traffic.
//!
//! These are plain synchronous smoltcp socket operations (no async, no timer),
//! so the *same* functions drive both:
//!   * **R15a** — the blocking `main_10bt` loop (`--features wan-dhcp`), and
//!   * **R15b** — the executor's `wan_task` (`--features router`), beside the
//!     cyw43 LAN.
//!
//! See `docs/r15-plan.md` §5/§6.

use core::fmt::Write;

use smoltcp::iface::{Interface, SocketHandle, SocketSet};
use smoltcp::phy::ChecksumCapabilities;
use smoltcp::socket::{dhcpv4, dns, icmp};
use smoltcp::wire::{
    DnsQueryType, Icmpv4Packet, Icmpv4Repr, IpAddress, IpCidr, Ipv4Address, Ipv4Cidr,
};

/// Off-link ping target (Google public DNS) — reachable only via the default
/// route, so a reply proves the DHCP-installed gateway + upstream NAT work.
pub const PING_TARGET: Ipv4Address = Ipv4Address::new(8, 8, 8, 8);
/// Name to resolve via the DHCP-provided DNS server (acceptance #3).
pub const DNS_NAME: &str = "example.com";
/// ICMP echo identifier we bind to + match replies against.
pub const ICMP_IDENT: u16 = 0x42;

/// Live WAN-client state, surfaced once per second as the `[Wan]` telemetry
/// line — the on-device evidence for the R15 WAN acceptance. `Copy` so the
/// router build can publish a snapshot through a `Cell` for `usb_task` to read.
#[derive(Clone, Copy)]
pub struct WanState {
    /// Address the dhcpv4 client leased (`None` until configured).
    pub addr: Option<Ipv4Cidr>,
    /// Default gateway from the lease.
    pub gw: Option<Ipv4Address>,
    /// First DNS server from the lease.
    pub dns0: Option<Ipv4Address>,
    /// ICMP echo sequence counter + sent/replied tallies.
    pub ping_seq: u16,
    pub ping_sent: u32,
    pub ping_ok: u32,
    /// In-flight DNS query handle, if any.
    pub dns_query: Option<dns::QueryHandle>,
    /// Last A record resolved for [`DNS_NAME`].
    pub resolved: Option<Ipv4Address>,
}

impl WanState {
    pub const fn new() -> Self {
        Self {
            addr: None,
            gw: None,
            dns0: None,
            ping_seq: 0,
            ping_sent: 0,
            ping_ok: 0,
            dns_query: None,
            resolved: None,
        }
    }

    /// Write the `[Wan]` body (no prefix, no newline) into `w`: lease IP /
    /// gateway / DNS, the ICMP ping tally, and the last resolved A record.
    pub fn write_status(&self, w: &mut impl Write) {
        match self.addr {
            Some(c) => {
                let _ = write!(w, "ip={} ", c);
            }
            None => {
                let _ = write!(w, "ip=none ");
            }
        }
        match self.gw {
            Some(g) => {
                let _ = write!(w, "gw={} ", g);
            }
            None => {
                let _ = write!(w, "gw=none ");
            }
        }
        match self.dns0 {
            Some(d) => {
                let _ = write!(w, "dns={} ", d);
            }
            None => {
                let _ = write!(w, "dns=none ");
            }
        }
        let _ = write!(w, "ping={}/{} ", self.ping_ok, self.ping_sent);
        match self.resolved {
            Some(a) => {
                let _ = write!(w, "{}={}", DNS_NAME, a);
            }
            None => {
                let _ = write!(w, "{}=?", DNS_NAME);
            }
        }
    }
}

impl Default for WanState {
    fn default() -> Self {
        Self::new()
    }
}

/// Apply a dhcpv4 lease change: install/clear the interface address + default
/// route and feed (or clear) the DNS socket's server list. Call every poll,
/// after `iface.poll`. The lease data is copied out of the borrowed `Event`
/// first (it borrows the SocketSet) so we can then touch the dns socket.
pub fn dhcp_apply(
    iface: &mut Interface,
    sockets: &mut SocketSet,
    dhcp_handle: SocketHandle,
    dns_handle: SocketHandle,
    wan: &mut WanState,
) {
    enum Action {
        None,
        Deconfig,
        Config {
            addr: Ipv4Cidr,
            router: Option<Ipv4Address>,
            dns0: Option<Ipv4Address>,
            servers: heapless::Vec<IpAddress, 4>,
        },
    }
    // Drain the event into owned values; the dhcp-socket borrow ends here.
    let action = match sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll() {
        None => Action::None,
        Some(dhcpv4::Event::Deconfigured) => Action::Deconfig,
        Some(dhcpv4::Event::Configured(cfg)) => {
            let mut servers: heapless::Vec<IpAddress, 4> = heapless::Vec::new();
            for s in cfg.dns_servers.iter() {
                let _ = servers.push(IpAddress::Ipv4(*s));
            }
            Action::Config {
                addr: cfg.address,
                router: cfg.router,
                dns0: cfg.dns_servers.first().copied(),
                servers,
            }
        }
    };

    match action {
        Action::None => {}
        Action::Deconfig => {
            iface.update_ip_addrs(|a| a.clear());
            iface.routes_mut().remove_default_ipv4_route();
            sockets.get_mut::<dns::Socket>(dns_handle).update_servers(&[]);
            *wan = WanState::new();
        }
        Action::Config {
            addr,
            router,
            dns0,
            servers,
        } => {
            iface.update_ip_addrs(|a| {
                a.clear();
                let _ = a.push(IpCidr::Ipv4(addr));
            });
            iface.routes_mut().remove_default_ipv4_route();
            if let Some(gw) = router {
                let _ = iface.routes_mut().add_default_ipv4_route(gw);
            }
            sockets
                .get_mut::<dns::Socket>(dns_handle)
                .update_servers(&servers);
            wan.addr = Some(addr);
            wan.gw = router;
            wan.dns0 = dns0;
            // Servers may have changed — abandon any in-flight query.
            wan.dns_query = None;
        }
    }
}

/// Send one ICMP echo request to [`PING_TARGET`] (binds the socket lazily).
/// Replies are tallied in [`ping_drain`].
pub fn ping_send(
    sockets: &mut SocketSet,
    icmp_handle: SocketHandle,
    wan: &mut WanState,
    checksum: &ChecksumCapabilities,
) {
    let sock = sockets.get_mut::<icmp::Socket>(icmp_handle);
    if !sock.is_open() && sock.bind(icmp::Endpoint::Ident(ICMP_IDENT)).is_err() {
        return;
    }
    if !sock.can_send() {
        return;
    }
    let repr = Icmpv4Repr::EchoRequest {
        ident: ICMP_IDENT,
        seq_no: wan.ping_seq,
        data: b"pico-wan",
    };
    if let Ok(payload) = sock.send(repr.buffer_len(), IpAddress::Ipv4(PING_TARGET)) {
        let mut pkt = Icmpv4Packet::new_unchecked(payload);
        repr.emit(&mut pkt, checksum);
        wan.ping_seq = wan.ping_seq.wrapping_add(1);
        wan.ping_sent = wan.ping_sent.wrapping_add(1);
    }
}

/// Drain ICMP echo replies; count the ones matching our ident.
pub fn ping_drain(
    sockets: &mut SocketSet,
    icmp_handle: SocketHandle,
    wan: &mut WanState,
    checksum: &ChecksumCapabilities,
) {
    let sock = sockets.get_mut::<icmp::Socket>(icmp_handle);
    while sock.can_recv() {
        let Ok((payload, _from)) = sock.recv() else {
            break;
        };
        let Ok(pkt) = Icmpv4Packet::new_checked(payload) else {
            continue;
        };
        if let Ok(Icmpv4Repr::EchoReply { ident, .. }) = Icmpv4Repr::parse(&pkt, checksum) {
            if ident == ICMP_IDENT {
                wan.ping_ok = wan.ping_ok.wrapping_add(1);
            }
        }
    }
}

/// Start a DNS A-record query for [`DNS_NAME`] when none is in flight and a
/// server is known.
pub fn dns_start(
    iface: &mut Interface,
    sockets: &mut SocketSet,
    dns_handle: SocketHandle,
    wan: &mut WanState,
) {
    if wan.dns_query.is_some() || wan.dns0.is_none() {
        return;
    }
    let cx = iface.context();
    if let Ok(handle) = sockets
        .get_mut::<dns::Socket>(dns_handle)
        .start_query(cx, DNS_NAME, DnsQueryType::A)
    {
        wan.dns_query = Some(handle);
    }
}

/// Harvest a finished DNS query: record the first A record + free the slot.
pub fn dns_harvest(sockets: &mut SocketSet, dns_handle: SocketHandle, wan: &mut WanState) {
    let Some(handle) = wan.dns_query else {
        return;
    };
    match sockets
        .get_mut::<dns::Socket>(dns_handle)
        .get_query_result(handle)
    {
        Ok(addrs) => {
            wan.dns_query = None;
            wan.resolved = addrs.iter().find_map(|ip| match ip {
                IpAddress::Ipv4(v4) => Some(*v4),
                #[allow(unreachable_patterns)] // ipv6 disabled — single variant
                _ => None,
            });
        }
        Err(dns::GetQueryResultError::Pending) => {}
        // Failed (NXDOMAIN / SERVFAIL / timeout) — free the slot, retry later.
        Err(_) => wan.dns_query = None,
    }
}
