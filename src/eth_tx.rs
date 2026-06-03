//! 10BASE-T Ethernet TX over PIO.
//!
//! Ports `src/ser_10base_t.pio` and `src/udp.c` from the C reference repo.
//!
//! Layer 1 (PIO): single-ended Manchester-encoded NRZ on a single GPIO at
//! 10 Mbps line rate (= 20 MHz half-bit clock). Drives the ISL3177E DI pin.
//! Each data byte expands to one 32-bit PIO "instruction stream" via the
//! Manchester lookup table; the PIO state machine consumes 2 bits at a time
//! from each word, jumping among IDLE / LOW / HIGH dispatch instructions.
//!
//! Layer 2 (data): Ethernet + IPv4 + UDP frame builder, FCS computed over
//! dst-MAC..end-of-UDP-payload.

use rp235x_hal as hal;
use hal::pac::PIO0;
use hal::pio::{
    PinDir, PinState, Running, ShiftDirection, StateMachine, Tx, SM0, SM2, UninitStateMachine,
};

use crate::manchester::MANCHESTER_TABLE;

/// IPv4/UDP socket parameters that don't change frame-to-frame.
pub struct UdpEndpoint {
    pub src_mac: [u8; 6],
    pub dst_mac: [u8; 6], // FF:FF:FF:FF:FF:FF for broadcast
    pub src_ip: [u8; 4],
    pub dst_ip: [u8; 4],
    pub src_port: u16,
    pub dst_port: u16,
}

/// Maximum on-wire frame the UDP builder can produce:
/// preamble(7) + SFD(1) + eth(14) + ip(20) + udp(8) + payload(<=1472) + FCS(4).
const MAX_TX_FRAME: usize = 1526;

/// Host-visible PIO IRQ flag the carrier-detect SM raises while RO (GP13) is
/// active. Flags 0–3 are readable by the CPU; we don't enable its system
/// interrupt, so it's purely a status bit the TX path polls. See
/// `carrier_present` / `wait_carrier_idle`.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CARRIER_IRQ_FLAG: u8 = 0;

/// Upper bound on how long a `send_*` call will defer for carrier (Phase 3d).
/// Generous enough to wait out a full-MTU frame in flight (~1.2 ms), but
/// bounded so a stuck-busy flag (RO noise / detector bug) degrades to
/// "no carrier-sense" rather than wedging TX entirely. Counted in poll
/// iterations of the PIO IRQ register, not wall-clock — rough by design.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CARRIER_WAIT_SPINS: u32 = 200_000;

/// Phase 3e — CSMA/CA randomized backoff (collision *avoidance*).
///
/// Carrier-sense (3d) stops us starting *during* a host frame, but two
/// stations that both have traffic queued while the wire is busy will both
/// jump in the instant it frees → collision. That's our dominant residual:
/// the Pico finishes a data segment, the wire frees, and both the Pico (next
/// segment) and the host (ACK) start together. A small *random* backoff before
/// each TCP frame desyncs us from the host so one clearly wins the wire.
///
/// One backoff "slot" in CPU spin cycles (~1 µs @ 240 MHz / ~1.6 µs @ 150).
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CSMA_SLOT_CYCLES: u32 = 240;
/// Backoff window mask: random 0..=15 slots (~0–15 µs) per attempt.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CSMA_BACKOFF_MASK: u32 = 0x0F;
/// Bounded CSMA attempts before transmitting anyway (so a persistently busy
/// wire can't wedge TX — same safety stance as `CARRIER_WAIT_SPINS`).
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CSMA_MAX_ATTEMPTS: u32 = 10;

/// PIO TX state machine handle. We hold onto the running StateMachine so that
/// Rust's type-state ownership doesn't drop it (which would be a footgun).
pub struct EthTx {
    _sm: StateMachine<(PIO0, SM0), Running>,
    /// Carrier-detect SM (Phase 3d). Runs autonomously watching RO; we only
    /// keep the handle alive so it isn't dropped (which would stop the SM).
    _cs_sm: StateMachine<(PIO0, SM2), Running>,
    tx: Tx<(PIO0, SM0)>,
    ip_identifier: u16,
    /// xorshift32 state for the Phase-3e CSMA/CA random backoff. Seeded to a
    /// fixed nonzero constant — the absolute sequence doesn't matter, only
    /// that our backoffs vary independently of the host's frame timing.
    #[cfg_attr(feature = "full-duplex", allow(dead_code))] // unused in FD mode (no CSMA)
    lfsr: u32,
    /// Scratch buffer the UDP broadcast builder assembles into before the
    /// per-byte Manchester FIFO writes. Owned here (was a `static mut`) so
    /// there's no aliasing footgun and no Rust-2024 hard error.
    raw_frame: [u8; MAX_TX_FRAME],
}

// In `full-duplex` mode the half-duplex MAC helpers (`wait_carrier_idle`,
// `csma_acquire`, `carrier_present`, `next_rand`) are uncalled — the carrier-sense
// + CSMA/CA gates are `#[cfg]`-disabled (see the call sites). Allow them to remain
// defined (HD is still the default build) without a dead-code warning under FD.
#[cfg_attr(feature = "full-duplex", allow(dead_code))]
impl EthTx {
    /// Initialize the TX PIO program (PIO0 SM0) + the carrier-detect SM
    /// (PIO0 SM2, Phase 3d). `tx_pin_id` drives the ISL3177E DI input
    /// (= GP14); `rx_pin_id` is the RO line (= GP13) the carrier detector
    /// watches. `sys_clk_hz` is clk_sys, used for both PIO dividers.
    pub fn new(
        pio: &mut hal::pio::PIO<PIO0>,
        sm: UninitStateMachine<(PIO0, SM0)>,
        cs_sm: UninitStateMachine<(PIO0, SM2)>,
        tx_pin_id: u8,
        rx_pin_id: u8,
        sys_clk_hz: u32,
    ) -> Self {
        // PIO program: 1-bit side-set, dispatches IDLE/LOW/HIGH based on the
        // 2-bit value popped by `out pc, 2` from the Manchester-encoded word.
        // `.origin 0` is REQUIRED because `out pc, 2` jumps to absolute PIO
        // memory addresses 0/1/2 — the program must live there.
        let program = pio::pio_asm!(
            ".side_set 1",
            ".origin 0",
            ".wrap_target",
            "    out pc, 2  side 0",   // 0 = IDLE (DI=0, line idle)
            "    out pc, 2  side 0",   // 1 = LOW  (DI=0, negative half-bit)
            "    out pc, 2  side 1",   // 2 = HIGH (DI=1, positive half-bit)
            ".wrap",
        );

        let installed = pio.install(&program.program).unwrap();

        // 20 MHz PIO clock from sys_clk_hz; one PIO cycle = 50 ns = Manchester
        // half-bit. At sys_clk=150 MHz the divider is 7.5 (= 7 + 128/256);
        // ±3.3 ns jitter is well within 10BASE-T tolerance.
        let (div_int, div_frac) = crate::pio_util::clock_divider(sys_clk_hz, 20_000_000.0);

        let (mut sm, _rx, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(tx_pin_id)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(div_int, div_frac)
            .buffers(hal::pio::Buffers::OnlyTx)
            .build(sm);

        // Match the C `pio_sm_set_pins_with_mask` + `pio_sm_set_pindirs_with_mask`:
        // initial output value 0, direction output.
        sm.set_pins([(tx_pin_id, PinState::Low)]);
        sm.set_pindirs([(tx_pin_id, PinDir::Output)]);
        let sm = sm.start();

        // --- Phase 3d: carrier-detect state machine (PIO0 SM2) ---
        //
        // Watches RO (GP13) and raises host-visible PIO IRQ flag 0 while the
        // line is *active* (toggling = a frame on the wire), clearing it once
        // the line has been *stable* for GUARD samples (idle). 10BASE-T idle
        // is a steady level (the RX `find_active_run` logic relies on idle
        // bytes being 0x00/0xFF), so "carrier present" == "transitions seen
        // recently". The TX path polls this flag before grabbing the wire,
        // restoring the carrier-sense that Phase 3c's multicore RX removed
        // (gotcha #10). Detection lives in PIO; the TX gate is in software.
        //
        // GUARD = 8 iterations × 2 cyc @ 60 MHz ≈ 267 ns of quiet ⇒ idle.
        // The max gap between edges in live Manchester is one bit (100 ns ≈
        // 3 iterations), so GUARD=8 never falsely declares idle mid-frame yet
        // releases TX quickly once a frame ends.
        let cs_program = pio::pio_asm!(
            "restart:",
            "    set x, 8",            // GUARD: stable-sample countdown
            "    jmp pin, track_high", // branch on current RO level
            "track_low:",
            "    jmp pin, edge",       // RO went high while low ⇒ edge
            "    jmp x--, track_low",  // still low; count down
            "    jmp idle",            // stable low for GUARD ⇒ idle
            "track_high:",
            "    jmp pin, high_cont",  // still high
            "    jmp edge",            // RO went low ⇒ edge
            "high_cont:",
            "    jmp x--, track_high",
            "    jmp idle",
            "edge:",
            "    irq set 0",           // carrier present (busy)
            "    jmp restart",
            "idle:",
            "    irq clear 0",         // line idle
            "    jmp restart",
        );
        let cs_installed = pio.install(&cs_program.program).unwrap();
        // 60 MHz, same as the RX sampler — fine edge resolution for GUARD.
        let (cs_div_int, cs_div_frac) = crate::pio_util::clock_divider(sys_clk_hz, 60_000_000.0);
        let (mut cs_sm, _cs_rx, _cs_tx) = hal::pio::PIOBuilder::from_installed_program(cs_installed)
            .jmp_pin(rx_pin_id)
            .clock_divisor_fixed_point(cs_div_int, cs_div_frac)
            .build(cs_sm);
        // RO is an input (driven by the ISL3177E). Set it explicitly so the
        // detector reads a valid level even before EthRx::new configures it.
        cs_sm.set_pindirs([(rx_pin_id, PinDir::Input)]);
        let cs_sm = cs_sm.start();

        Self {
            _sm: sm,
            _cs_sm: cs_sm,
            tx,
            ip_identifier: 0,
            lfsr: 0x2545_F491, // fixed nonzero xorshift seed
            raw_frame: [0; MAX_TX_FRAME],
        }
    }

    /// Step the xorshift32 PRNG (Phase 3e backoff entropy).
    #[inline]
    fn next_rand(&mut self) -> u32 {
        let mut x = self.lfsr;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.lfsr = x;
        x
    }

    /// CSMA/CA acquire (Phase 3e): sense the wire idle, wait a *random* backoff,
    /// then re-sense; if someone grabbed the wire during the backoff, defer and
    /// retry. Bounded attempts so a persistently busy wire can't wedge TX (we
    /// transmit anyway after `CSMA_MAX_ATTEMPTS`). The random backoff is what
    /// breaks the synchronized-start tie with the host after the wire frees.
    fn csma_acquire(&mut self) {
        for _ in 0..CSMA_MAX_ATTEMPTS {
            Self::wait_carrier_idle();
            let slots = self.next_rand() & CSMA_BACKOFF_MASK;
            if slots != 0 {
                hal::arch::delay(slots * CSMA_SLOT_CYCLES);
            }
            if !Self::carrier_present() {
                return; // wire still idle after our backoff → take it
            }
            // Someone started during our backoff — loop, re-sense, back off again.
        }
    }

    /// Is the carrier-detect SM currently reporting RO activity (a frame on
    /// the wire)? Reads the host-visible PIO IRQ flag the SM maintains.
    #[inline]
    fn carrier_present() -> bool {
        // Safety: read-only access to PIO0's IRQ status register.
        let pio = unsafe { &*hal::pac::PIO0::ptr() };
        (pio.irq().read().irq().bits() & (1 << CARRIER_IRQ_FLAG)) != 0
    }

    /// Carrier-sense: spin until the wire is idle (or the timeout fires).
    /// Call before grabbing the wire for a frame so we don't transmit on top
    /// of an in-flight host frame (collision). Bounded so a stuck-busy flag
    /// degrades to "transmit anyway" rather than wedging TX. Returns the
    /// number of spins spent waiting (0 = wire was already idle).
    #[inline]
    fn wait_carrier_idle() -> u32 {
        let mut spins = 0u32;
        while Self::carrier_present() {
            spins += 1;
            if spins >= CARRIER_WAIT_SPINS {
                break;
            }
        }
        spins
    }

    /// Emit a 10BASE-T Normal Link Pulse: 100 ns positive pulse, then idle.
    ///
    /// `0x0000000A` per the C reference:
    ///   bits[1:0]=10 -> HIGH, bits[3:2]=10 -> HIGH, rest=IDLE.
    /// 2× HIGH = 100 ns of DI=1 = single positive pulse on the line.
    ///
    /// Pads with 12 IDLE words after the NLP for the same reason
    /// `send_raw_frame` does: if a frame TX is dispatched immediately
    /// after the NLP (e.g. iface.poll runs right after the NLP-tick in
    /// the main loop), the next preamble would land inside the host's
    /// expected post-NLP/IFG window and FCS-fail. Critical section
    /// keeps the DMA_IRQ_0 decoder from preempting the FIFO writes
    /// (one NLP write alone is safe, but the 12 padding writes spin on
    /// FIFO availability and so could be preempted mid-loop).
    pub fn send_nlp(&mut self) {
        // Carrier-sense before transmitting (Phase 3d) — don't drive an NLP
        // on top of an in-flight host frame. Skipped in `full-duplex` mode:
        // on a forced-10-FD link RX carrier is the peer's traffic on a
        // separate pair, so there's nothing to defer to (see
        // docs/full-duplex-analysis.md §7).
        #[cfg(not(feature = "full-duplex"))]
        Self::wait_carrier_idle();
        critical_section::with(|_| {
            let _ = self.tx.write(0x0000_000A_u32);
            for _ in 0..12 {
                while !self.tx.write(0u32) {}
            }
        });
    }

    /// Transmit a raw Ethernet frame body (dst MAC..end of payload, NO
    /// preamble / SFD / FCS — this method adds those). Smoltcp's TxToken
    /// hands us exactly this shape. Pushes preamble+SFD, then each body
    /// byte (Manchester-encoded), pads with zero bytes to the IEEE 802.3
    /// 60-byte minimum if needed (Linux NICs silently drop runt frames),
    /// then emits the FCS over body+padding, then TP_IDL.
    ///
    /// **Compute CRC before any PIO writes** — otherwise the ~27 µs spent
    /// in the bit-by-bit CRC mid-flight (between body and FCS push)
    /// underruns the 8-deep TX FIFO (drains in ~6 µs), the line stalls
    /// mid-frame, and the receiver's NIC marks the resulting bytes as bad
    /// FCS. With CRC precomputed, the per-byte FIFO writes run uninterrupted
    /// and the PIO has continuous Manchester to emit.
    ///
    /// **Critical section around the FIFO writes** — once DMA_IRQ_0 was
    /// enabled (R6, IRQ-driven RX), the decoder running in the IRQ handler
    /// could pre-empt this loop for ~100 µs, underrunning the FIFO the same
    /// way the bit-by-bit CRC used to. Disabling interrupts for the ~50 µs
    /// of a max-size frame is trivial against the 2.18 ms DMA half-fill
    /// budget — the IRQ just becomes pending and runs as soon as we exit.
    pub fn send_raw_frame(&mut self, body: &[u8]) {
        let pad_len = 60usize.saturating_sub(body.len());
        let crc = if pad_len == 0 {
            crate::crc::crc32_ieee802_3(body)
        } else {
            crate::crc::crc32_ieee802_3_padded(body, pad_len)
        };
        let crc_bytes = crc.to_le_bytes();

        // CSMA/CA before grabbing the wire (Phase 3d carrier-sense + Phase 3e
        // random backoff). Done outside the critical section so we defer with
        // interrupts enabled. This is the path the curl's TCP segments take, so
        // it's where the gotcha-#10 collisions concentrate; the random backoff
        // desyncs us from the host's ACKs after the wire frees.
        //
        // Skipped in `full-duplex` mode: with both ends forced to 10M-FD there
        // is no shared collision domain, so we transmit immediately regardless
        // of RX carrier — the whole point of the experiment
        // (docs/full-duplex-analysis.md §7). ONLY correct against a forced-FD
        // peer; a duplex mismatch corrupts both directions.
        #[cfg(not(feature = "full-duplex"))]
        self.csma_acquire();

        critical_section::with(|_| {
            // 7 × preamble byte (0x55) + 1 × SFD byte (0xD5).
            let pre_word = MANCHESTER_TABLE[0x55];
            for _ in 0..7 {
                while !self.tx.write(pre_word) {}
            }
            while !self.tx.write(MANCHESTER_TABLE[0xD5]) {}
            // Body.
            for &b in body.iter() {
                while !self.tx.write(MANCHESTER_TABLE[b as usize]) {}
            }
            // Padding (zero bytes to 60-byte minimum).
            let zero_word = MANCHESTER_TABLE[0x00];
            for _ in 0..pad_len {
                while !self.tx.write(zero_word) {}
            }
            // FCS (little-endian on wire).
            for &b in crc_bytes.iter() {
                while !self.tx.write(MANCHESTER_TABLE[b as usize]) {}
            }
            // TP_IDL: end-of-frame marker.
            while !self.tx.write(0x0000_0AAA_u32) {}
            // IFG padding: ≥ 9.6 µs of idle after TP_IDL before the next
            // preamble can start (IEEE 802.3 minimum inter-frame gap).
            // Each all-zero FIFO word = 16 PIO IDLE dispatches × 50 ns ≈
            // 800 ns, so 12 words ≈ 9.6 µs. The FIFO is only 8-deep, so
            // most pushes will spin until the PIO drains earlier ones —
            // that's the point: keep the FIFO full of IDLE so the line
            // stays quiet long enough for the host NIC to be ready for
            // the next preamble. Without this, back-to-back smoltcp
            // egresses (e.g. queued ARP→ICMP, or ICMP-reply followed by
            // a UDP TX) leave < 9.6 µs gap and the host scores the
            // second frame as bad-FCS (regression vs. polled mode).
            for _ in 0..12 {
                while !self.tx.write(0u32) {}
            }
        });
    }

    /// Build a broadcast UDP packet around `payload` and emit it on the line.
    /// Payload max is set by the size of the internal frame buffer (~1472 B).
    pub fn send_udp_broadcast(&mut self, ep: &UdpEndpoint, payload: &[u8]) {
        let total_bytes =
            build_eth_ipv4_udp_frame(ep, payload, &mut self.raw_frame, self.ip_identifier);
        self.ip_identifier = self.ip_identifier.wrapping_add(1);

        // Carrier-sense before grabbing the wire (Phase 3d). Skipped in
        // `full-duplex` mode (docs/full-duplex-analysis.md §7).
        #[cfg(not(feature = "full-duplex"))]
        Self::wait_carrier_idle();

        // Critical section: same reason as send_raw_frame — keep the
        // DMA_IRQ_0 decoder from preempting these per-byte writes and
        // underrunning the PIO TX FIFO.
        critical_section::with(|_| {
            // Manchester-encode each byte and push to the PIO FIFO.
            // (disjoint borrows: reads self.raw_frame, writes self.tx)
            for &b in self.raw_frame[..total_bytes].iter() {
                let word = MANCHESTER_TABLE[b as usize];
                while !self.tx.write(word) {
                    // Spin until FIFO has space.
                }
            }
            // TP_IDL: end-of-frame marker — single positive pulse so the
            // magnetics secondary returns cleanly to 0 differential.
            while !self.tx.write(0x0000_0AAA_u32) {}
            // IFG padding (same as send_raw_frame) — ensures the next
            // preamble has ≥ 9.6 µs of clear air after TP_IDL.
            for _ in 0..12 {
                while !self.tx.write(0u32) {}
            }
        });
    }
}

/// Compute the IPv4 one's-complement header checksum over `header_words`
/// (16-bit big-endian values).
fn ipv4_checksum(words: impl IntoIterator<Item = u16>) -> u16 {
    let mut sum: u32 = 0;
    for w in words {
        sum += w as u32;
    }
    while (sum >> 16) != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    !(sum as u16)
}

/// Build the raw on-wire frame bytes (pre-Manchester) into `out`. Returns the
/// total number of bytes written (preamble + SFD + Ethernet/IP/UDP + FCS).
fn build_eth_ipv4_udp_frame(
    ep: &UdpEndpoint,
    payload: &[u8],
    out: &mut [u8],
    ip_id: u16,
) -> usize {
    let mut i = 0usize;

    // Preamble + SFD
    for _ in 0..7 {
        out[i] = 0x55;
        i += 1;
    }
    out[i] = 0xD5;
    i += 1;
    let frame_start = i;

    // Ethernet header
    out[i..i + 6].copy_from_slice(&ep.dst_mac);
    i += 6;
    out[i..i + 6].copy_from_slice(&ep.src_mac);
    i += 6;
    out[i..i + 2].copy_from_slice(&[0x08, 0x00]); // EtherType IPv4
    i += 2;

    // IPv4 header (20 bytes, no options)
    let udp_len = (payload.len() + 8) as u16;
    let ip_total_len = 20u16 + udp_len;
    let ip_header_start = i;
    out[i] = 0x45; // v4, IHL=5
    out[i + 1] = 0x00; // ToS
    out[i + 2..i + 4].copy_from_slice(&ip_total_len.to_be_bytes());
    out[i + 4..i + 6].copy_from_slice(&ip_id.to_be_bytes());
    out[i + 6] = 0x40; // Don't fragment
    out[i + 7] = 0x00;
    out[i + 8] = 0x40; // TTL
    out[i + 9] = 0x11; // Protocol = UDP
    out[i + 10] = 0;
    out[i + 11] = 0; // Header checksum (placeholder)
    out[i + 12..i + 16].copy_from_slice(&ep.src_ip);
    out[i + 16..i + 20].copy_from_slice(&ep.dst_ip);
    // Compute IP checksum over the 20-byte header.
    let cksum = ipv4_checksum((0..10).map(|k| {
        u16::from_be_bytes([out[ip_header_start + 2 * k], out[ip_header_start + 2 * k + 1]])
    }));
    out[i + 10..i + 12].copy_from_slice(&cksum.to_be_bytes());
    i += 20;

    // UDP header (8 bytes) — checksum zero (legal for IPv4).
    out[i..i + 2].copy_from_slice(&ep.src_port.to_be_bytes());
    i += 2;
    out[i..i + 2].copy_from_slice(&ep.dst_port.to_be_bytes());
    i += 2;
    out[i..i + 2].copy_from_slice(&udp_len.to_be_bytes());
    i += 2;
    out[i] = 0;
    out[i + 1] = 0; // udp checksum = 0
    i += 2;

    // Payload
    out[i..i + payload.len()].copy_from_slice(payload);
    i += payload.len();

    // FCS (CRC-32 / IEEE 802.3) over dst MAC .. end of payload.
    let crc = crate::crc::crc32_ieee802_3(&out[frame_start..i]);
    out[i..i + 4].copy_from_slice(&crc.to_le_bytes());
    i += 4;

    i
}
