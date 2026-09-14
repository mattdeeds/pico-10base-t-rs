//! 10BASE-T TX over PIO, with carrier sense and CSMA/CA.
//!
//! PIO emits Manchester on DI (GP14) at a 20 MHz half-bit clock, dispatching
//! 2 bits at a time from `MANCHESTER_TABLE` words. Also builds UDP frames.

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

/// PIO IRQ flag the carrier-detect SM sets while RO is active. Polled only.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CARRIER_IRQ_FLAG: u8 = 0;

/// Max carrier-wait spins (~one full-MTU frame). Bounded so a stuck flag can't wedge TX.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CARRIER_WAIT_SPINS: u32 = 200_000;

/// CSMA/CA backoff slot in spin cycles (~1 µs at 240 MHz).
/// Random backoff keeps us from starting in sync with the host's ACK.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CSMA_SLOT_CYCLES: u32 = 240;
/// Backoff window mask: random 0..=15 slots (~0–15 µs) per attempt.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CSMA_BACKOFF_MASK: u32 = 0x0F;
/// CSMA attempts before transmitting anyway.
#[cfg_attr(feature = "full-duplex", allow(dead_code))] // HD-only MAC; unused in FD mode
const CSMA_MAX_ATTEMPTS: u32 = 10;

/// PIO TX state. Holds the running SMs so they aren't dropped.
pub struct EthTx {
    _sm: StateMachine<(PIO0, SM0), Running>,
    /// Carrier-detect SM, held only to keep it running.
    _cs_sm: StateMachine<(PIO0, SM2), Running>,
    tx: Tx<(PIO0, SM0)>,
    ip_identifier: u16,
    /// xorshift32 state for CSMA backoff.
    #[cfg_attr(feature = "full-duplex", allow(dead_code))] // unused in FD mode (no CSMA)
    lfsr: u32,
    /// Scratch buffer for the UDP frame builder.
    raw_frame: [u8; MAX_TX_FRAME],
}

// Half-duplex helpers are unused under `full-duplex`.
#[cfg_attr(feature = "full-duplex", allow(dead_code))]
impl EthTx {
    /// Start TX (PIO0 SM0 on DI `tx_pin_id`) and carrier detect (SM2 on RO `rx_pin_id`).
    pub fn new(
        pio: &mut hal::pio::PIO<PIO0>,
        sm: UninitStateMachine<(PIO0, SM0)>,
        cs_sm: UninitStateMachine<(PIO0, SM2)>,
        tx_pin_id: u8,
        rx_pin_id: u8,
        sys_clk_hz: u32,
    ) -> Self {
        // `out pc, 2` jumps to address 0/1/2 (IDLE/LOW/HIGH), so `.origin 0` is required.
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

        // 20 MHz PIO clock: one cycle per Manchester half-bit.
        let (div_int, div_frac) = crate::pio_util::clock_divider(sys_clk_hz, 20_000_000.0);

        let (mut sm, _rx, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(tx_pin_id)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(div_int, div_frac)
            .buffers(hal::pio::Buffers::OnlyTx)
            .build(sm);

        // Output, initially low.
        sm.set_pins([(tx_pin_id, PinState::Low)]);
        sm.set_pindirs([(tx_pin_id, PinDir::Output)]);
        let sm = sm.start();

        // Carrier-detect SM (PIO0 SM2): sets IRQ flag 0 on RO edges, clears it
        // after GUARD quiet iterations. Idle 10BASE-T holds a steady level.
        // GUARD = 8 × 2 cycles at 60 MHz ≈ 267 ns, above the 100 ns max edge gap.
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
        // 60 MHz, like the RX sampler.
        let (cs_div_int, cs_div_frac) = crate::pio_util::clock_divider(sys_clk_hz, 60_000_000.0);
        let (mut cs_sm, _cs_rx, _cs_tx) = hal::pio::PIOBuilder::from_installed_program(cs_installed)
            .jmp_pin(rx_pin_id)
            .clock_divisor_fixed_point(cs_div_int, cs_div_frac)
            .build(cs_sm);
        // RO is an input; set it before EthRx::new does.
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

    /// Step the xorshift32 PRNG.
    #[inline]
    fn next_rand(&mut self) -> u32 {
        let mut x = self.lfsr;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.lfsr = x;
        x
    }

    /// Wait for idle, back off randomly, re-sense; retry if taken.
    /// Transmits anyway after `CSMA_MAX_ATTEMPTS`.
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

    /// True while the carrier-detect SM sees RO activity.
    #[inline]
    fn carrier_present() -> bool {
        // Safety: read-only access to PIO0's IRQ status register.
        let pio = unsafe { &*hal::pac::PIO0::ptr() };
        (pio.irq().read().irq().bits() & (1 << CARRIER_IRQ_FLAG)) != 0
    }

    /// Spin until the wire is idle or the cap hits. Returns spins waited.
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

    /// Send a Normal Link Pulse (100 ns high), then 12 idle words.
    /// The idle pad keeps a following preamble out of the host's post-NLP window.
    pub fn send_nlp(&mut self) {
        // Carrier sense, except under full-duplex (nothing to defer to).
        #[cfg(not(feature = "full-duplex"))]
        Self::wait_carrier_idle();
        critical_section::with(|_| {
            let _ = self.tx.write(0x0000_000A_u32);
            for _ in 0..12 {
                while !self.tx.write(0u32) {}
            }
        });
    }

    /// Send a frame body (dst MAC..payload). Adds preamble, SFD, padding to
    /// 60 bytes, FCS, and TP_IDL.
    ///
    /// CRC is computed before any PIO writes, and writes run with interrupts
    /// off: a stall mid-frame underruns the 8-deep TX FIFO (~6 µs) and
    /// corrupts the frame.
    pub fn send_raw_frame(&mut self, body: &[u8]) {
        let pad_len = 60usize.saturating_sub(body.len());
        let crc = if pad_len == 0 {
            crate::crc::crc32_ieee802_3(body)
        } else {
            crate::crc::crc32_ieee802_3_padded(body, pad_len)
        };
        let crc_bytes = crc.to_le_bytes();

        // CSMA/CA with interrupts enabled. Skipped under full-duplex, which is
        // only correct against a forced-FD peer.
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
            // IFG: 12 idle words ≈ 9.6 µs, the IEEE 802.3 minimum gap.
            // Without it, back-to-back frames fail FCS at the host.
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

        // Carrier sense, except under full-duplex.
        #[cfg(not(feature = "full-duplex"))]
        Self::wait_carrier_idle();

        // Interrupts off: same FIFO underrun risk as send_raw_frame.
        critical_section::with(|_| {
            // Manchester-encode each byte and push to the PIO FIFO.
            // (disjoint borrows: reads self.raw_frame, writes self.tx)
            for &b in self.raw_frame[..total_bytes].iter() {
                let word = MANCHESTER_TABLE[b as usize];
                while !self.tx.write(word) {
                    // Spin until FIFO has space.
                }
            }
            // TP_IDL: end-of-frame pulse so the line returns to zero.
            while !self.tx.write(0x0000_0AAA_u32) {}
            // IFG padding, as in send_raw_frame.
            for _ in 0..12 {
                while !self.tx.write(0u32) {}
            }
        });
    }
}

/// IPv4 one's-complement checksum over big-endian 16-bit `words`.
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
