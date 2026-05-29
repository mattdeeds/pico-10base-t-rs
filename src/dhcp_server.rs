//! Minimal LAN DHCP server (R14.4).
//!
//! smoltcp ships only a DHCP *client*, so the server is ours — but we reuse
//! smoltcp's DHCP *wire codec* (`DhcpRepr`/`DhcpPacket`, gated by the
//! `proto-dhcpv4` feature) to parse requests and emit replies, so there's no
//! hand-rolled BOOTP byte layout.
//!
//! It runs as a UDP socket on port 67 inside the wireless `net_task`'s
//! `SocketSet`: each poll we drain the socket, answer DISCOVER with OFFER and
//! REQUEST with ACK, handing out a fixed `192.168.4.0/24` pool with the Pico's
//! LAN IP (`192.168.4.1`) as gateway + DNS. Replies are broadcast to
//! `255.255.255.255:68` (the client has no IP/ARP entry yet). Leases are keyed
//! by client MAC so a given client keeps its address across DISCOVER→REQUEST and
//! reconnects; the table is fixed-size (no alloc).

use core::sync::atomic::{AtomicU32, Ordering};

use smoltcp::socket::udp;
use smoltcp::wire::{
    DhcpMessageType, DhcpPacket, DhcpRepr, IpAddress, IpEndpoint, Ipv4Address, DHCP_CLIENT_PORT,
    DHCP_SERVER_PORT,
};

/// The Pico's LAN address — gateway, DNS, and DHCP server identifier.
const SERVER_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
const SUBNET_MASK: Ipv4Address = Ipv4Address::new(255, 255, 255, 0);
/// Lease pool: `192.168.4.{POOL_BASE .. POOL_BASE+POOL_LEN}` (slot i ↔ that IP).
const POOL_BASE: u8 = 10;
const POOL_LEN: usize = 32;
/// Lease time handed to clients (1 hour).
const LEASE_SECS: u32 = 3600;

/// Count of DHCP replies (OFFER + ACK) emitted — surfaced in the `[Cyw43]` line.
pub static DHCP_TX: AtomicU32 = AtomicU32::new(0);

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

    /// Reuse this MAC's existing lease, else claim the first free slot.
    fn allocate(&mut self, mac: [u8; 6]) -> Option<Ipv4Address> {
        if let Some(i) = self.leases.iter().position(|s| *s == Some(mac)) {
            return Some(ip_for(i));
        }
        let i = self.leases.iter().position(|s| s.is_none())?;
        self.leases[i] = Some(mac);
        Some(ip_for(i))
    }

    /// Drain the DHCP socket and answer DISCOVER/REQUEST. Call every poll, after
    /// `iface.poll` has delivered any inbound datagrams. Binds lazily to :67.
    pub fn poll(&mut self, socket: &mut udp::Socket) {
        if !socket.is_open() {
            let _ = socket.bind(DHCP_SERVER_PORT);
        }

        // A handful per call keeps the loop bounded; `recv_slice` errors when
        // the queue is drained.
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
            // Release/Decline/Inform/etc. — ignored for this LAN-bring-up server.
            _ => return None,
        };

        let your_ip = self.allocate(req.client_hardware_address.0)?;

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
            // Broadcast the reply: the client has no IP (and we no ARP entry for
            // it) until it applies this lease.
            broadcast: true,
            requested_ip: None,
            client_identifier: None,
            server_identifier: Some(SERVER_IP),
            parameter_request_list: None,
            // No DNS option for now — clients get IP + gateway, enough to reach
            // the LAN gateway. A DNS-server option (and relay) is R18; building
            // smoltcp's `dns_servers` Vec here also needs its heapless 0.9, not
            // our 0.8.
            dns_servers: None,
            max_size: None,
            lease_duration: Some(LEASE_SECS),
            renew_duration: None,
            rebind_duration: None,
            additional_options: &[],
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
