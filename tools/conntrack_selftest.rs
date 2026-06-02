// Standalone host verification of the R17 conntrack pure logic (no_std-free copy).
// The functions here are byte-identical to what goes into src/conntrack.rs; this
// is the offline "unit test" since the bin crate has no host test harness.
// Build + run:  rustc -O /tmp/ct_selftest.rs -o /tmp/ct_selftest && /tmp/ct_selftest

// ---- deterministic PRNG (no external crates) ----
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u32 {
        // xorshift64*
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        ((x.wrapping_mul(0x2545F4914F6CDD1D)) >> 32) as u32
    }
    fn u16(&mut self) -> u16 { self.next() as u16 }
}

// ===================== incremental internet checksum =====================

/// One's-complement 16-bit add with end-around carry.
fn add1c(a: u16, b: u16) -> u16 {
    let s = a as u32 + b as u32;
    ((s & 0xffff) + (s >> 16)) as u16
}

/// Full one's-complement sum over a byte slice (big-endian 16-bit words),
/// returning the *checksum* (folded + complemented). The reference path.
fn checksum_full(data: &[u8]) -> u16 {
    let mut acc: u16 = 0;
    let mut i = 0;
    while i + 1 < data.len() {
        acc = add1c(acc, u16::from_be_bytes([data[i], data[i + 1]]));
        i += 2;
    }
    if i < data.len() {
        acc = add1c(acc, u16::from_be_bytes([data[i], 0]));
    }
    !acc
}

/// RFC 1624 incremental update: given the OLD checksum and a list of
/// (old_word, new_word) 16-bit changes, return the NEW checksum.
///   HC' = ~(~HC + Σ(~m_i) + Σ(m'_i))
fn checksum_incr(old_check: u16, changes: &[(u16, u16)]) -> u16 {
    let mut acc: u16 = !old_check; // ~HC
    for &(m, mp) in changes {
        acc = add1c(acc, !m); // + ~m
        acc = add1c(acc, mp); // + m'
    }
    !acc
}

fn test_checksum() {
    let mut rng = Rng(0x1234_5678_9abc_def0);
    let mut fails = 0u32;

    // (a) Known IPv4 header vector (RFC-style). 20-byte header, checksum field
    //     at bytes 10..12. Wikipedia's canonical example → checksum 0xb861.
    let mut hdr: [u8; 20] = [
        0x45, 0x00, 0x00, 0x73, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, 0x00, 0x00,
        0xc0, 0xa8, 0x00, 0x01, 0xc0, 0xa8, 0x00, 0xc7,
    ];
    hdr[10] = 0;
    hdr[11] = 0;
    let c = checksum_full(&hdr);
    if c != 0xb861 {
        println!("  KNOWN-VECTOR FAIL: got {:#06x} want 0xb861", c);
        fails += 1;
    } else {
        println!("  known IPv4 header vector: {:#06x} OK", c);
    }

    // (b) Randomized: build a 20-byte header, zero its check field, compute the
    //     base checksum, then NAPT-rewrite src IP (bytes 12..16) + dst port-like
    //     word; compare full-recompute vs incremental.
    for _ in 0..200_000 {
        let mut h = [0u8; 20];
        for b in h.iter_mut() {
            *b = rng.next() as u8;
        }
        h[10] = 0;
        h[11] = 0;
        let base = checksum_full(&h);

        // change two 16-bit words: src-IP hi (12..14) and src-IP lo (14..16)
        let old_hi = u16::from_be_bytes([h[12], h[13]]);
        let old_lo = u16::from_be_bytes([h[14], h[15]]);
        let new_hi = rng.u16();
        let new_lo = rng.u16();
        let nh = new_hi.to_be_bytes();
        let nl = new_lo.to_be_bytes();
        h[12] = nh[0];
        h[13] = nh[1];
        h[14] = nl[0];
        h[15] = nl[1];

        let full = checksum_full(&h);
        let incr = checksum_incr(base, &[(old_hi, new_hi), (old_lo, new_lo)]);
        if full != incr {
            if fails < 5 {
                println!("  RANDOM FAIL: full={:#06x} incr={:#06x}", full, incr);
            }
            fails += 1;
        }
    }

    // (c) Edge: a change that produces 0x0000 vs 0xffff (one's-complement -0).
    //     Verify incr matches full across the carry boundary.
    for _ in 0..50_000 {
        let mut h = [0u8; 8];
        for b in h.iter_mut() {
            *b = rng.next() as u8;
        }
        // treat bytes 0..2 as a checksum-carrying field set to 0
        h[0] = 0;
        h[1] = 0;
        let base = checksum_full(&h);
        let old = u16::from_be_bytes([h[2], h[3]]);
        let new = rng.u16();
        let nb = new.to_be_bytes();
        h[2] = nb[0];
        h[3] = nb[1];
        let full = checksum_full(&h);
        let incr = checksum_incr(base, &[(old, new)]);
        if full != incr {
            if fails < 10 {
                println!("  EDGE FAIL: full={:#06x} incr={:#06x}", full, incr);
            }
            fails += 1;
        }
    }

    println!(
        "test_checksum: {}",
        if fails == 0 { "PASS (250k random + known vector + carry edges)" } else { "FAIL" }
    );
    assert_eq!(fails, 0, "checksum incremental != full recompute");
}

// ===================== NAT port/id allocator =====================

const NAT_LO: u16 = 49152;
const NAT_HI: u16 = 65535;

/// Allocate the next free id in [NAT_LO, NAT_HI], linear-probing from a rolling
/// cursor. `in_use(id)` reports whether an id is taken. Returns None if full.
fn alloc_id<F: Fn(u16) -> bool>(cursor: &mut u16, in_use: F) -> Option<u16> {
    let span = (NAT_HI - NAT_LO) as u32 + 1;
    for _ in 0..span {
        let id = *cursor;
        *cursor = if *cursor == NAT_HI { NAT_LO } else { *cursor + 1 };
        if !in_use(id) {
            return Some(id);
        }
    }
    None
}

fn test_allocator() {
    use std::collections::HashSet;
    let mut used: HashSet<u16> = HashSet::new();
    let mut cursor = NAT_LO;
    let mut fails = 0u32;

    // disjointness: allocate 1000 ids, all distinct + in range
    for _ in 0..1000 {
        let id = alloc_id(&mut cursor, |x| used.contains(&x)).expect("space");
        if id < NAT_LO || id > NAT_HI {
            println!("  ALLOC OUT OF RANGE: {}", id);
            fails += 1;
        }
        if !used.insert(id) {
            println!("  ALLOC DUP: {}", id);
            fails += 1;
        }
    }

    // exhaustion: fill the whole range, next alloc must be None
    used.clear();
    cursor = NAT_LO;
    let span = (NAT_HI - NAT_LO) as usize + 1;
    for _ in 0..span {
        let id = alloc_id(&mut cursor, |x| used.contains(&x)).expect("space");
        used.insert(id);
    }
    if alloc_id(&mut cursor, |x| used.contains(&x)).is_some() {
        println!("  EXHAUSTION FAIL: allocated past full");
        fails += 1;
    }

    // disjoint from a reserved id (e.g. the WAN ping ICMP id 0x42 lives below
    // NAT_LO, so it can never be allocated): NAT_LO > 0x42
    if NAT_LO <= 0x0042 {
        println!("  RANGE OVERLAPS RESERVED smoltcp ids");
        fails += 1;
    }

    println!("test_allocator: {}", if fails == 0 { "PASS (disjoint + in-range + exhaustion + reserved-gap)" } else { "FAIL" });
    assert_eq!(fails, 0);
}

// ===================== eviction (timeout + LRU) =====================

#[derive(Clone, Copy)]
struct Slot {
    used: bool,
    last_seen_ms: u64,
    timeout_ms: u64,
}

/// Pick a slot to (re)use: first an expired one, else the LRU (oldest last_seen).
fn pick_evict(slots: &[Slot], now_ms: u64) -> usize {
    // free slot first
    if let Some(i) = slots.iter().position(|s| !s.used) {
        return i;
    }
    // expired
    if let Some(i) = slots
        .iter()
        .position(|s| now_ms.saturating_sub(s.last_seen_ms) >= s.timeout_ms)
    {
        return i;
    }
    // LRU
    let mut best = 0usize;
    for i in 1..slots.len() {
        if slots[i].last_seen_ms < slots[best].last_seen_ms {
            best = i;
        }
    }
    best
}

fn test_eviction() {
    let mut fails = 0u32;
    // 4 live slots, none free
    let mut slots = [
        Slot { used: true, last_seen_ms: 100, timeout_ms: 1000 },
        Slot { used: true, last_seen_ms: 50, timeout_ms: 1000 }, // oldest → LRU pick
        Slot { used: true, last_seen_ms: 200, timeout_ms: 1000 },
        Slot { used: true, last_seen_ms: 300, timeout_ms: 1000 },
    ];

    // at now=500 nothing expired → LRU = index 1 (last_seen 50)
    if pick_evict(&slots, 500) != 1 {
        println!("  LRU FAIL: picked {}", pick_evict(&slots, 500));
        fails += 1;
    }

    // at now=1300, index 0 (100+1000=1100<=1300) is expired and comes first
    if pick_evict(&slots, 1300) != 0 {
        println!("  TIMEOUT FAIL: picked {}", pick_evict(&slots, 1300));
        fails += 1;
    }

    // a free slot beats both
    slots[2].used = false;
    if pick_evict(&slots, 1300) != 2 {
        println!("  FREE-SLOT FAIL: picked {}", pick_evict(&slots, 1300));
        fails += 1;
    }

    println!("test_eviction: {}", if fails == 0 { "PASS (free > expired > LRU)" } else { "FAIL" });
    assert_eq!(fails, 0);
}

// ===================== NAPT rewrite: offsets + pseudo-header =====================
// Mirrors forward.rs parse_l4 / napt_rewrite_src / napt_rewrite_dst (using the
// already-verified checksum_incr) and proves the wire offsets + pseudo-header
// coverage by recomputing the FULL L4 checksum from scratch after each rewrite.
const ETH: usize = 14;

fn fold(acc0: u16, data: &[u8]) -> u16 {
    let mut acc = acc0;
    let mut i = 0;
    while i + 1 < data.len() {
        acc = add1c(acc, u16::from_be_bytes([data[i], data[i + 1]]));
        i += 2;
    }
    if i < data.len() {
        acc = add1c(acc, u16::from_be_bytes([data[i], 0]));
    }
    acc
}

/// (l4_off, l4_len, proto, csum_off)
fn l4_bounds(frame: &[u8]) -> (usize, usize, u8, usize) {
    let ihl = (frame[ETH] & 0x0f) as usize * 4;
    let l4 = ETH + ihl;
    let proto = frame[ETH + 9];
    let ip_total = u16::from_be_bytes([frame[ETH + 2], frame[ETH + 3]]) as usize;
    let l4_len = ip_total - ihl;
    let cko = match proto {
        6 => l4 + 16,
        17 => l4 + 6,
        1 => l4 + 2,
        _ => l4,
    };
    (l4, l4_len, proto, cko)
}

fn pseudo_acc(frame: &[u8], l4_len: usize, proto: u8) -> u16 {
    let mut acc = 0u16;
    for off in [ETH + 12, ETH + 14, ETH + 16, ETH + 18] {
        acc = add1c(acc, u16::from_be_bytes([frame[off], frame[off + 1]]));
    }
    acc = add1c(acc, proto as u16);
    add1c(acc, l4_len as u16)
}

/// A valid internet checksum makes the sum over (pseudo-header + segment,
/// INCLUDING the checksum field) fold to 0xffff.
fn l4_valid(frame: &[u8]) -> bool {
    let (l4, l4_len, proto, _) = l4_bounds(frame);
    let seg = &frame[l4..l4 + l4_len];
    let acc = if proto == 1 {
        fold(0, seg) // ICMP: no pseudo-header
    } else {
        fold(pseudo_acc(frame, l4_len, proto), seg)
    };
    acc == 0xffff
}

fn set_l4_checksum(frame: &mut [u8]) {
    let (l4, l4_len, proto, cko) = l4_bounds(frame);
    frame[cko] = 0;
    frame[cko + 1] = 0;
    let acc = if proto == 1 {
        fold(0, &frame[l4..l4 + l4_len])
    } else {
        fold(pseudo_acc(frame, l4_len, proto), &frame[l4..l4 + l4_len])
    };
    let ck = !acc;
    frame[cko..cko + 2].copy_from_slice(&ck.to_be_bytes());
}

fn rewrite_src(frame: &mut [u8], new_ip: [u8; 4], new_id: u16) {
    let (l4, _, proto, cko) = l4_bounds(frame);
    let old_csum = u16::from_be_bytes([frame[cko], frame[cko + 1]]);
    if proto == 1 {
        let old_id = u16::from_be_bytes([frame[l4 + 4], frame[l4 + 5]]);
        let c = checksum_incr(old_csum, &[(old_id, new_id)]);
        frame[cko..cko + 2].copy_from_slice(&c.to_be_bytes());
        frame[l4 + 4..l4 + 6].copy_from_slice(&new_id.to_be_bytes());
    } else {
        let old_hi = u16::from_be_bytes([frame[ETH + 12], frame[ETH + 13]]);
        let old_lo = u16::from_be_bytes([frame[ETH + 14], frame[ETH + 15]]);
        let new_hi = u16::from_be_bytes([new_ip[0], new_ip[1]]);
        let new_lo = u16::from_be_bytes([new_ip[2], new_ip[3]]);
        let old_port = u16::from_be_bytes([frame[l4], frame[l4 + 1]]);
        if !(proto == 17 && old_csum == 0) {
            let c =
                checksum_incr(old_csum, &[(old_hi, new_hi), (old_lo, new_lo), (old_port, new_id)]);
            frame[cko..cko + 2].copy_from_slice(&c.to_be_bytes());
        }
        frame[l4..l4 + 2].copy_from_slice(&new_id.to_be_bytes());
    }
    frame[ETH + 12..ETH + 16].copy_from_slice(&new_ip);
}

fn rewrite_dst(frame: &mut [u8], new_ip: [u8; 4], new_id: u16) {
    let (l4, _, proto, cko) = l4_bounds(frame);
    let old_csum = u16::from_be_bytes([frame[cko], frame[cko + 1]]);
    if proto == 1 {
        let old_id = u16::from_be_bytes([frame[l4 + 4], frame[l4 + 5]]);
        let c = checksum_incr(old_csum, &[(old_id, new_id)]);
        frame[cko..cko + 2].copy_from_slice(&c.to_be_bytes());
        frame[l4 + 4..l4 + 6].copy_from_slice(&new_id.to_be_bytes());
    } else {
        let old_hi = u16::from_be_bytes([frame[ETH + 16], frame[ETH + 17]]);
        let old_lo = u16::from_be_bytes([frame[ETH + 18], frame[ETH + 19]]);
        let new_hi = u16::from_be_bytes([new_ip[0], new_ip[1]]);
        let new_lo = u16::from_be_bytes([new_ip[2], new_ip[3]]);
        let old_port = u16::from_be_bytes([frame[l4 + 2], frame[l4 + 3]]);
        if !(proto == 17 && old_csum == 0) {
            let c =
                checksum_incr(old_csum, &[(old_hi, new_hi), (old_lo, new_lo), (old_port, new_id)]);
            frame[cko..cko + 2].copy_from_slice(&c.to_be_bytes());
        }
        frame[l4 + 2..l4 + 4].copy_from_slice(&new_id.to_be_bytes());
    }
    frame[ETH + 16..ETH + 20].copy_from_slice(&new_ip);
}

fn build(proto: u8, payload: usize) -> Vec<u8> {
    let ihl = 20usize;
    let l4hdr = match proto {
        6 => 20,
        17 => 8,
        1 => 8,
        _ => 0,
    };
    let ip_total = ihl + l4hdr + payload;
    let mut f = vec![0u8; ETH + ip_total];
    f[12] = 0x08;
    f[13] = 0x00; // ethertype IPv4
    f[ETH] = 0x45; // version 4, IHL 5
    f[ETH + 2..ETH + 4].copy_from_slice(&(ip_total as u16).to_be_bytes());
    f[ETH + 8] = 64; // TTL
    f[ETH + 9] = proto;
    f[ETH + 12..ETH + 16].copy_from_slice(&[192, 168, 4, 10]); // src
    f[ETH + 16..ETH + 20].copy_from_slice(&[8, 8, 8, 8]); // dst
    let l4 = ETH + ihl;
    match proto {
        6 => {
            f[l4..l4 + 2].copy_from_slice(&51000u16.to_be_bytes());
            f[l4 + 2..l4 + 4].copy_from_slice(&443u16.to_be_bytes());
            f[l4 + 12] = 0x50; // data offset 5
            f[l4 + 13] = 0x18; // PSH+ACK
        }
        17 => {
            f[l4..l4 + 2].copy_from_slice(&51000u16.to_be_bytes());
            f[l4 + 2..l4 + 4].copy_from_slice(&53u16.to_be_bytes());
            f[l4 + 4..l4 + 6].copy_from_slice(&((l4hdr + payload) as u16).to_be_bytes());
        }
        1 => {
            f[l4] = 8; // echo request
            f[l4 + 4..l4 + 6].copy_from_slice(&0x1234u16.to_be_bytes()); // id
            f[l4 + 6..l4 + 8].copy_from_slice(&1u16.to_be_bytes()); // seq
        }
        _ => {}
    }
    for i in 0..payload {
        f[ETH + ihl + l4hdr + i] = (i as u8).wrapping_mul(7).wrapping_add(1);
    }
    f
}

fn test_rewrite() {
    let mut fails = 0u32;
    for &(proto, name) in &[(6u8, "TCP"), (17u8, "UDP"), (1u8, "ICMP")] {
        // Outbound: src 192.168.4.10:51000 → 192.168.37.129:50000 (NAPT).
        let mut f = build(proto, 6);
        set_l4_checksum(&mut f);
        if !l4_valid(&f) {
            println!("  {} initial checksum INVALID", name);
            fails += 1;
        }
        rewrite_src(&mut f, [192, 168, 37, 129], 50000);
        if !l4_valid(&f) {
            println!("  {} post src-rewrite checksum INVALID", name);
            fails += 1;
        }
        if f[ETH + 12..ETH + 16] != [192u8, 168, 37, 129] {
            println!("  {} IP src not rewritten", name);
            fails += 1;
        }

        // Inbound reply: src 8.8.8.8 → dst 192.168.37.129:50000, rewritten back to
        // the LAN client 192.168.4.10:51000.
        let mut r = build(proto, 6);
        r[ETH + 12..ETH + 16].copy_from_slice(&[8, 8, 8, 8]);
        r[ETH + 16..ETH + 20].copy_from_slice(&[192, 168, 37, 129]);
        let l4 = ETH + 20;
        match proto {
            6 | 17 => {
                r[l4..l4 + 2].copy_from_slice(&443u16.to_be_bytes()); // remote src port
                r[l4 + 2..l4 + 4].copy_from_slice(&50000u16.to_be_bytes()); // our wan port
            }
            1 => {
                r[l4] = 0; // echo reply
                r[l4 + 4..l4 + 6].copy_from_slice(&50000u16.to_be_bytes()); // id = wan id
            }
            _ => {}
        }
        set_l4_checksum(&mut r);
        rewrite_dst(&mut r, [192, 168, 4, 10], 51000);
        if !l4_valid(&r) {
            println!("  {} post dst-rewrite checksum INVALID", name);
            fails += 1;
        }
        if r[ETH + 16..ETH + 20] != [192u8, 168, 4, 10] {
            println!("  {} IP dst not rewritten", name);
            fails += 1;
        }
    }
    println!(
        "test_rewrite: {}",
        if fails == 0 { "PASS (TCP/UDP/ICMP src+dst rewrite → checksum valid)" } else { "FAIL" }
    );
    assert_eq!(fails, 0);
}

fn main() {
    test_checksum();
    test_allocator();
    test_eviction();
    test_rewrite();
    println!("\nALL CONNTRACK SELF-TESTS PASSED");
}
