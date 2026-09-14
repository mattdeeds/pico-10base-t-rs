//! Minimal LAN DHCP server using smoltcp's DHCP codec.
//!
//! Answers DISCOVER with OFFER and REQUEST with ACK from a fixed pool in
//! `192.168.4.0/24`, keyed by client MAC. Replies are broadcast.
//! Gateway is `192.168.4.1`; DNS is [`LAN_DNS_OFFER`].

use core::sync::atomic::{AtomicU32, Ordering};

use smoltcp::socket::udp;
use smoltcp::wire::{
    DhcpMessageType, DhcpOption, DhcpPacket, DhcpRepr, IpAddress, IpEndpoint, Ipv4Address,
    DHCP_CLIENT_PORT, DHCP_SERVER_PORT,
};

/// The Pico's LAN address — gateway, DNS, and DHCP server identifier.
const SERVER_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
const SUBNET_MASK: Ipv4Address = Ipv4Address::new(255, 255, 255, 0);
/// Lease pool: `192.168.4.{POOL_BASE .. POOL_BASE+POOL_LEN}` (slot i ↔ that IP).
const POOL_BASE: u8 = 10;
/// Lease pool size. Public so the mgmt page can size its body.
pub const POOL_LEN: usize = 32;
/// Lease time handed to clients (1 hour).
const LEASE_SECS: u32 = 3600;
/// DHCP option code for "Domain Name Server" (RFC 2132 §3.8).
const OPT_DNS_SERVER: u8 = 6;

/// Count of DHCP replies (OFFER + ACK) emitted — surfaced in the `[Cyw43]` line.
pub static DHCP_TX: AtomicU32 = AtomicU32::new(0);

/// DNS server offered to LAN clients, as big-endian octets. Starts at 8.8.8.8;
/// `wan_task` sets it from the WAN lease. NAPT forwards the queries.
pub static LAN_DNS_OFFER: AtomicU32 = AtomicU32::new(u32::from_be_bytes([8, 8, 8, 8]));

/// Fixed MAC→IP lease allocator. `leases[i] == Some(mac)` means
/// `192.168.4.(POOL_BASE+i)` is held by `mac`.
pub struct DhcpServer {
    leases: [Option<[u8; 6]>; POOL_LEN],
}

impl DhcpServer {
    pub const fn new() -> Self {
        Self {
            leases: [None; POOL_LEN],
        }
    }

    /// Currently held leases as `(ip, mac)`.
    pub fn active_leases(&self) -> impl Iterator<Item = (Ipv4Address, [u8; 6])> + '_ {
        self.leases
            .iter()
            .enumerate()
            .filter_map(|(i, slot)| slot.map(|mac| (ip_for(i), mac)))
    }

    /// Reuse this MAC's existing lease, else claim the first free slot.
    fn allocate(&mut self, mac: [u8; 6]) -> Option<Ipv4Address> {
        if let Some(i) = self.leases.iter().position(|s| *s == Some(mac)) {
            return Some(ip_for(i));
        }
        let i = self.leases.iter().position(|s| s.is_none())?;
        self.leases[i] = Some(mac);
        Some(ip_for(i))
    }

    /// Answer queued DISCOVER/REQUESTs. Call after each `iface.poll`. Binds :67 lazily.
    pub fn poll(&mut self, socket: &mut udp::Socket) {
        if !socket.is_open() {
            let _ = socket.bind(DHCP_SERVER_PORT);
        }

        // At most 4 per call to bound the loop.
        let mut req_buf = [0u8; 1024];
        for _ in 0..4 {
            let len = match socket.recv_slice(&mut req_buf) {
                Ok((len, _meta)) => len,
                Err(_) => break,
            };
            if let Some((reply, blen)) = self.build_reply(&req_buf[..len]) {
                let dst = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::BROADCAST), DHCP_CLIENT_PORT);
                if socket.send_slice(&reply[..blen], dst).is_ok() {
                    DHCP_TX.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    /// Parse one request and, for DISCOVER/REQUEST, build the OFFER/ACK bytes.
    /// Returns the reply buffer + its length (other message types → `None`).
    fn build_reply(&mut self, req_bytes: &[u8]) -> Option<([u8; 1024], usize)> {
        let packet = DhcpPacket::new_checked(req_bytes).ok()?;
        let req = DhcpRepr::parse(&packet).ok()?;

        let reply_type = match req.message_type {
            DhcpMessageType::Discover => DhcpMessageType::Offer,
            DhcpMessageType::Request => DhcpMessageType::Ack,
            // Other message types are ignored.
            _ => return None,
        };

        let your_ip = self.allocate(req.client_hardware_address.0)?;

        // DNS goes in as raw option 6: `DhcpRepr.dns_servers` uses smoltcp's
        // heapless 0.9 Vec, which our heapless 0.8 can't name.
        let dns_octets = LAN_DNS_OFFER.load(Ordering::Relaxed).to_be_bytes();
        let extra_opts = [DhcpOption {
            kind: OPT_DNS_SERVER,
            data: &dns_octets,
        }];

        let reply = DhcpRepr {
            message_type: reply_type,
            transaction_id: req.transaction_id,
            secs: 0,
            client_hardware_address: req.client_hardware_address,
            client_ip: Ipv4Address::UNSPECIFIED,
            your_ip,
            server_ip: Ipv4Address::UNSPECIFIED,
            router: Some(SERVER_IP),
            subnet_mask: Some(SUBNET_MASK),
            relay_agent_ip: Ipv4Address::UNSPECIFIED,
            // Broadcast: the client has no IP yet.
            broadcast: true,
            requested_ip: None,
            client_identifier: None,
            server_identifier: Some(SERVER_IP),
            parameter_request_list: None,
            // DNS is sent via `additional_options`.
            dns_servers: None,
            max_size: None,
            lease_duration: Some(LEASE_SECS),
            renew_duration: None,
            rebind_duration: None,
            additional_options: &extra_opts,
        };

        let blen = reply.buffer_len();
        let mut out = [0u8; 1024];
        if blen > out.len() {
            return None;
        }
        let mut pkt = DhcpPacket::new_unchecked(&mut out[..blen]);
        reply.emit(&mut pkt).ok()?;
        Some((out, blen))
    }
}

/// IP for pool slot `i`: `192.168.4.(POOL_BASE + i)`.
fn ip_for(i: usize) -> Ipv4Address {
    Ipv4Address::new(192, 168, 4, POOL_BASE + i as u8)
}
