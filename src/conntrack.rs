//! NAPT connection tracking. Design: `docs/r17-plan.md`.
//!
//! Only the WAN `ForwardingDevice` touches the table, so there are no locks.
//! NAT ids ([`NAT_LO`]..=[`NAT_HI`]) avoid smoltcp's own WAN ports and ids,
//! so a conntrack miss on WAN ingress means the frame is `Local`.

use core::sync::atomic::{AtomicU32, Ordering};

use smoltcp::wire::Ipv4Address;

// ── Telemetry (formatted in usb_task's `[Nat]` line) ───────────────────────
pub static NAT_OUT: AtomicU32 = AtomicU32::new(0); // outbound packets NAPT-rewritten
pub static NAT_IN: AtomicU32 = AtomicU32::new(0); // inbound replies matched + rewritten
pub static NAT_NEW: AtomicU32 = AtomicU32::new(0); // new conntrack entries created
pub static NAT_EVICT: AtomicU32 = AtomicU32::new(0); // entries reclaimed (timeout/LRU)
pub static NAT_DROP: AtomicU32 = AtomicU32::new(0); // real drops: outbound port/id exhaustion

/// Live conntrack entry count (for the `ct=<n>/<cap>` readout).
pub fn live_count() -> usize {
    NAT_LIVE.load(Ordering::Relaxed) as usize
}
static NAT_LIVE: AtomicU32 = AtomicU32::new(0);

// =====================================================================
// Incremental internet checksum (RFC 1624) — pure, offline-verified.
// =====================================================================

/// One's-complement 16-bit add with end-around carry.
pub fn add1c(a: u16, b: u16) -> u16 {
    let s = a as u32 + b as u32;
    ((s & 0xffff) + (s >> 16)) as u16
}

/// New checksum after `(old_word, new_word)` changes (RFC 1624).
/// For L4, include the pseudo-header address words and port/id words.
pub fn checksum_incr(old_check: u16, changes: &[(u16, u16)]) -> u16 {
    let mut acc: u16 = !old_check; // ~HC
    for &(m, mp) in changes {
        acc = add1c(acc, !m); // + ~m
        acc = add1c(acc, mp); // + m'
    }
    !acc
}

/// The two 16-bit halves of an IPv4 address, big-endian (for `checksum_incr`).
pub fn addr_words(ip: Ipv4Address) -> (u16, u16) {
    let o = ip.octets();
    (u16::from_be_bytes([o[0], o[1]]), u16::from_be_bytes([o[2], o[3]]))
}

// =====================================================================
// Conntrack table
// =====================================================================

/// Flow transport. The id is the TCP/UDP port or ICMP echo id.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Proto {
    Tcp,
    Udp,
    IcmpEcho,
}

/// LAN-side outbound 5-tuple: LAN client source, WAN peer destination.
#[derive(Clone, Copy)]
pub struct Tuple {
    pub proto: Proto,
    pub src_ip: Ipv4Address,
    pub src_id: u16,
    pub dst_ip: Ipv4Address,
    pub dst_id: u16,
}

/// NAT WAN id range, above smoltcp's own WAN ports and the `0x42` ping id.
pub const NAT_LO: u16 = 49152;
pub const NAT_HI: u16 = 65535;

/// Table capacity. ~40 B/entry ⇒ ~2.5 KB. A phone idles a few dozen flows.
pub const CT_CAP: usize = 64;

// Per-proto idle timeouts (ms).
const T_TCP_EST: u64 = 60_000;
const T_TCP_TRANS: u64 = 10_000; // handshake / closing
const T_UDP: u64 = 30_000;
const T_ICMP: u64 = 10_000;

// TCP flag bits we care about (just enough for a coarse timeout state).
const TCP_FIN: u8 = 0x01;
const TCP_SYN: u8 = 0x02;
const TCP_RST: u8 = 0x04;

#[derive(Clone, Copy, PartialEq, Eq)]
enum TcpState {
    New,         // SYN seen, handshake not complete
    Established, // bidirectional data
    Closing,     // FIN/RST seen
}

#[derive(Clone, Copy)]
struct Entry {
    used: bool,
    proto: Proto,
    lan_ip: Ipv4Address,
    lan_id: u16,
    remote_ip: Ipv4Address,
    remote_id: u16,
    wan_id: u16,
    last_seen: u64,
    tcp: TcpState,
}

impl Entry {
    const EMPTY: Entry = Entry {
        used: false,
        proto: Proto::Udp,
        lan_ip: Ipv4Address::UNSPECIFIED,
        lan_id: 0,
        remote_ip: Ipv4Address::UNSPECIFIED,
        remote_id: 0,
        wan_id: 0,
        last_seen: 0,
        tcp: TcpState::New,
    };

    fn timeout_ms(&self) -> u64 {
        match self.proto {
            Proto::Udp => T_UDP,
            Proto::IcmpEcho => T_ICMP,
            Proto::Tcp => match self.tcp {
                TcpState::Established => T_TCP_EST,
                _ => T_TCP_TRANS,
            },
        }
    }
}

/// The NAPT connection-tracking table.
pub struct Conntrack {
    slots: [Entry; CT_CAP],
    cursor: u16, // rolling id-allocation cursor
}

impl Conntrack {
    pub const fn new() -> Self {
        Self {
            slots: [Entry::EMPTY; CT_CAP],
            cursor: NAT_LO,
        }
    }

    /// LAN→WAN: find or create the flow and return its WAN id. `None` on exhaustion.
    pub fn outbound(&mut self, t: &Tuple, tcp_flags: u8, now_ms: u64) -> Option<u16> {
        // Existing flow? (keyed by the full LAN-side 5-tuple)
        for s in self.slots.iter_mut() {
            if s.used
                && s.proto == t.proto
                && s.lan_ip == t.src_ip
                && s.lan_id == t.src_id
                && s.remote_ip == t.dst_ip
                && s.remote_id == t.dst_id
            {
                s.last_seen = now_ms;
                s.tcp = next_tcp_state(s.tcp, tcp_flags);
                NAT_OUT.fetch_add(1, Ordering::Relaxed);
                return Some(s.wan_id);
            }
        }

        // New flow: allocate a WAN id + a slot.
        let wan_id = match self.alloc_id() {
            Some(id) => id,
            None => {
                NAT_DROP.fetch_add(1, Ordering::Relaxed); // port/id space exhausted
                return None;
            }
        };
        let idx = self.pick_slot(now_ms);
        if !self.slots[idx].used {
            NAT_LIVE.fetch_add(1, Ordering::Relaxed);
        } else {
            NAT_EVICT.fetch_add(1, Ordering::Relaxed);
        }
        self.slots[idx] = Entry {
            used: true,
            proto: t.proto,
            lan_ip: t.src_ip,
            lan_id: t.src_id,
            remote_ip: t.dst_ip,
            remote_id: t.dst_id,
            wan_id,
            last_seen: now_ms,
            tcp: next_tcp_state(TcpState::New, tcp_flags),
        };
        NAT_NEW.fetch_add(1, Ordering::Relaxed);
        NAT_OUT.fetch_add(1, Ordering::Relaxed);
        Some(wan_id)
    }

    /// WAN→LAN: match a reply to a live flow and return the LAN `(ip, id)`.
    /// `None` means it isn't NAT traffic.
    pub fn inbound(
        &mut self,
        proto: Proto,
        remote_ip: Ipv4Address,
        remote_id: u16,
        wan_id: u16,
        tcp_flags: u8,
        now_ms: u64,
    ) -> Option<(Ipv4Address, u16)> {
        for s in self.slots.iter_mut() {
            if s.used
                && s.proto == proto
                && s.wan_id == wan_id
                && s.remote_ip == remote_ip
                && s.remote_id == remote_id
                && now_ms.saturating_sub(s.last_seen) < s.timeout_ms()
            {
                s.last_seen = now_ms;
                s.tcp = next_tcp_state(s.tcp, tcp_flags);
                NAT_IN.fetch_add(1, Ordering::Relaxed);
                return Some((s.lan_ip, s.lan_id));
            }
        }
        // A miss is normal for the Pico's own traffic, not a drop.
        None
    }

    /// Next free WAN id, probing from the rolling cursor.
    fn alloc_id(&mut self) -> Option<u16> {
        let span = (NAT_HI - NAT_LO) as u32 + 1;
        for _ in 0..span {
            let id = self.cursor;
            self.cursor = if self.cursor == NAT_HI { NAT_LO } else { self.cursor + 1 };
            if !self.id_in_use(id) {
                return Some(id);
            }
        }
        None
    }

    fn id_in_use(&self, id: u16) -> bool {
        self.slots.iter().any(|s| s.used && s.wan_id == id)
    }

    /// Pick a slot for a new entry: a free slot, else an expired one, else the LRU.
    fn pick_slot(&self, now_ms: u64) -> usize {
        if let Some(i) = self.slots.iter().position(|s| !s.used) {
            return i;
        }
        if let Some(i) = self
            .slots
            .iter()
            .position(|s| now_ms.saturating_sub(s.last_seen) >= s.timeout_ms())
        {
            return i;
        }
        let mut best = 0usize;
        for i in 1..CT_CAP {
            if self.slots[i].last_seen < self.slots[best].last_seen {
                best = i;
            }
        }
        best
    }

    /// Sweep expired entries. Call periodically from `wan_task`.
    pub fn reap(&mut self, now_ms: u64) {
        for s in self.slots.iter_mut() {
            if s.used && now_ms.saturating_sub(s.last_seen) >= s.timeout_ms() {
                s.used = false;
                NAT_LIVE.fetch_sub(1, Ordering::Relaxed);
                NAT_EVICT.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

impl Default for Conntrack {
    fn default() -> Self {
        Self::new()
    }
}

/// Coarse TCP state for timeouts only (no sequence checks).
fn next_tcp_state(cur: TcpState, flags: u8) -> TcpState {
    if flags & (TCP_FIN | TCP_RST) != 0 {
        TcpState::Closing
    } else if flags & TCP_SYN != 0 {
        // SYN stays New unless already closing.
        if cur == TcpState::Closing {
            cur
        } else {
            TcpState::New
        }
    } else {
        TcpState::Established
    }
}
