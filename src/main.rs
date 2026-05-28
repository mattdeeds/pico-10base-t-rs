//! Pico-10BASE-T (Rust / RP2350 Hazard3 / rp235x-hal)
//!
//! Full software 10BASE-T NIC: PIO Manchester TX + PIO/DMA RX sampler with
//! an IRQ-driven decoder, bridged to smoltcp (ARP + ICMP + UDP echo + a tiny
//! HTTP server). USB CDC carries the debug log; a vendor reset interface lets
//! `picotool -f` self-reboot into BOOTSEL. See RESUME.md for the phase log.
//!
//! See ../Pico-10BASE-T/ for the C reference implementation we're porting from.

#![no_std]
#![no_main]

mod crc;
mod eth_mac;
mod eth_rx;
mod eth_rx_dpll; // Edge-track DPLL Manchester decoder (productized — Phase 3b)
mod eth_tx;
mod manchester;
mod pico_reset;
mod pio_util;

use panic_halt as _;

use core::fmt::Write;
use embedded_hal::digital::OutputPin;
use heapless::String;
use rp235x_hal as hal;
use hal::dma::DMAExt;
use hal::fugit::{HertzU32, RateExtU32};
use hal::gpio::FunctionPio0;
use hal::pio::PIOExt;
use hal::singleton;
use hal::Clock; // brings .freq() into scope
use hal::pll::{setup_pll_blocking, common_configs::PLL_USB_48MHZ, PLLConfig};
use hal::xosc::setup_xosc_blocking;
use hal::clocks::ClocksManager;
use smoltcp::iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage};
use smoltcp::socket::{tcp, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

const HW_PIN_TXD: u8 = 14; // ISL3177E DI
const HW_PIN_RXD: u8 = 13; // ISL3177E RO

/// Tell the Boot ROM about our application.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Pico 2 board has a 12 MHz crystal.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Phase 2d v3 — 240 MHz overclock. Buys 3 SM cycles/bit of PIO budget for the
/// windowed DPLL (18 → 24 cyc/bit), AND gives integer PIO dividers (TX÷9 RX÷3,
/// no fractional jitter). VCO 1080 MHz / (6 × 1) = 180 MHz. Recovery via SWD
/// if flash gets corrupted at the higher QMI SCK (the DAPLink probe is
/// attached; see the `sysclk-integer-pio-dividers` memory).
const PLL_SYS_240MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1200),
    refdiv: 1,
    post_div1: 5,
    post_div2: 1,
};

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // Manual clock setup so PLL_SYS goes to 180 MHz (vs the hal default 150 MHz).
    let xosc = setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz()).unwrap();
    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u16);
    let mut clocks = ClocksManager::new(pac.CLOCKS);
    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        PLL_SYS_240MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led = pins.gpio25.into_push_pull_output();

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // GP14 → ISL3177E DI, GP13 → ISL3177E RO. Reassign both to PIO0 function.
    let _tx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio14.into_function();
    let _rx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio13.into_function();

    // PIO0 SM0 drives the Manchester TX; SM1 runs the RX sampler. The CPU
    // edge-track DPLL (eth_rx_dpll) handles clock recovery in software off
    // the same 60 MHz sample stream — PIO1 is free for future use.
    let (mut pio0, sm0, sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
    let sys_clk_hz = clocks.system_clock.freq().to_Hz();
    let eth_tx = eth_tx::EthTx::new(&mut pio0, sm0, HW_PIN_TXD, sys_clk_hz);

    // DMA channels 0 and 1 ferry samples from the PIO RX FIFO into two
    // 16 KB half-buffers. EthRx::poll_with hands the just-filled half to
    // the decoder while DMA continues filling the other.
    let dma = pac.DMA.split(&mut pac.RESETS);
    let rx_buf_a = singleton!(: [u32; eth_rx::BUF_WORDS] = [0; eth_rx::BUF_WORDS]).unwrap();
    let rx_buf_b = singleton!(: [u32; eth_rx::BUF_WORDS] = [0; eth_rx::BUF_WORDS]).unwrap();
    // Carry + stitch buffers for ring-aware scan across DMA half boundaries.
    // Both static (in BSS via singleton!) — too big to want on the stack.
    let rx_carry =
        singleton!(: [u8; eth_rx::MAX_CARRY_BYTES] = [0; eth_rx::MAX_CARRY_BYTES]).unwrap();
    let rx_stitch =
        singleton!(: [u8; eth_rx::STITCH_BUF_BYTES] = [0; eth_rx::STITCH_BUF_BYTES]).unwrap();
    let eth_rx = eth_rx::EthRx::new(
        &mut pio0,
        sm1,
        HW_PIN_RXD,
        sys_clk_hz,
        dma.ch0,
        dma.ch1,
        rx_buf_a,
        rx_buf_b,
        rx_carry,
        rx_stitch,
    );

    // Install EthRx + our MAC (for the IRQ-side MAC filter) into the
    // shared static the DMA_IRQ_0 handler reads, then unmask the IRQ +
    // enable Hazard3 machine-external interrupts. From this point on,
    // decoding is interrupt-driven — the main loop never polls EthRx;
    // smoltcp drains the inbox via EthMac::Device::receive instead.
    let our_mac_bytes: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
    let _ = eth_mac::install_rx(eth_rx, our_mac_bytes);
    unsafe {
        hal::arch::interrupt_unmask(hal::pac::Interrupt::DMA_IRQ_0);
        hal::arch::interrupt_enable();
    }

    // EthMac now owns just the TX side + a scratch buffer; RX state lives
    // in the shared static. smoltcp's Interface still sees a single
    // Device handle.
    let mut mac = eth_mac::EthMac::new(eth_tx);

    let our_mac = EthernetAddress(our_mac_bytes);
    let our_ip = IpAddress::Ipv4(Ipv4Address::new(192, 168, 37, 24));
    let mut iface_config = Config::new(HardwareAddress::Ethernet(our_mac));
    iface_config.random_seed = timer.get_counter().ticks();
    let now0_inst = Instant::from_micros(timer.get_counter().ticks() as i64);
    let mut iface = Interface::new(iface_config, &mut mac, now0_inst);
    iface.update_ip_addrs(|addrs| {
        addrs.push(IpCidr::new(our_ip, 24)).unwrap();
    });

    let mut sockets_storage: [SocketStorage; 5] = [SocketStorage::EMPTY; 5];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    // R4.6: UDP echo socket on port 1234. Storage lives in main's stack
    // (the singleton! macro would be nicer but for a one-off it's overkill).
    let mut udp_rx_meta = [udp::PacketMetadata::EMPTY; 8];
    let mut udp_rx_payload = [0u8; 2048];
    let mut udp_tx_meta = [udp::PacketMetadata::EMPTY; 8];
    let mut udp_tx_payload = [0u8; 2048];
    let udp_rx_buffer = udp::PacketBuffer::new(&mut udp_rx_meta[..], &mut udp_rx_payload[..]);
    let udp_tx_buffer = udp::PacketBuffer::new(&mut udp_tx_meta[..], &mut udp_tx_payload[..]);
    let udp_socket = udp::Socket::new(udp_rx_buffer, udp_tx_buffer);
    let udp_handle: SocketHandle = sockets.add(udp_socket);

    // R7: a tiny HTTP server on port 80. 1 KB each direction is enough
    // for a 1-shot GET / response and validates retransmission/windowing
    // through our RX path under a more demanding protocol than UDP.
    let mut tcp_rx_storage = [0u8; 1024];
    let mut tcp_tx_storage = [0u8; 1024];
    let tcp_rx_buffer = tcp::SocketBuffer::new(&mut tcp_rx_storage[..]);
    let tcp_tx_buffer = tcp::SocketBuffer::new(&mut tcp_tx_storage[..]);
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);
    let tcp_handle: SocketHandle = sockets.add(tcp_socket);

    // Mirror the C reference's network parameters so the host's existing
    // ethtool / IP route setup keeps working.
    let endpoint = eth_tx::UdpEndpoint {
        src_mac: our_mac_bytes,
        dst_mac: [0xFF; 6], // broadcast
        src_ip: [192, 168, 37, 24],
        dst_ip: [192, 168, 37, 19],
        src_port: 1234,
        dst_port: 1234,
    };
    // USB CDC: appears on the host as /dev/ttyACM0.
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    // Vendor "reset interface" so `picotool -f` can self-reboot us into
    // BOOTSEL without the manual button-press / OpenOCD fallback.
    let mut reset_iface = pico_reset::PicoResetInterface::new(&usb_bus);

    // Serial number = chip ID, matching the format the BOOTSEL bootrom
    // advertises (16 hex chars = wafer_id || device_id). picotool tracks
    // serials across the app→BOOTSEL transition; if they don't match
    // (e.g. with a static string), `picotool -f` reboots us into BOOTSEL
    // successfully but then fails to identify the BOOTSEL device as the
    // same one it asked to reboot, and gives up.
    let mut serial_str: String<16> = String::new();
    match hal::rom_data::sys_info_api::chip_info() {
        Ok(Some(info)) => {
            let _ = write!(serial_str, "{:08X}{:08X}", info.wafer_id, info.device_id);
        }
        _ => {
            let _ = write!(serial_str, "0000000000000000");
        }
    }

    // VID:PID 2e8a:000a is the Raspberry Pi Foundation's allocation for the
    // pico-sdk "stdio_usb" CDC device. picotool recognizes this pair and can
    // force-reboot the chip into BOOTSEL mode via the vendor reset interface
    // we register below.
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2e8a, 0x000a))
        .strings(&[StringDescriptors::default()
            .manufacturer("pico-10base-t-rs")
            .product("Pico-10BASE-T (Rust)")
            .serial_number(serial_str.as_str())])
        .unwrap()
        .max_packet_size_0(64)
        .unwrap()
        .device_class(2) // USB CDC
        .build();

    // NLPs at 16 ms intervals; UDP broadcast at 200 ms; heartbeat log at 1 s.
    let now0 = timer.get_counter().ticks();
    let mut next_nlp = now0;
    let mut next_udp = now0 + 200_000;
    let mut next_log = now0 + 1_000_000;
    let mut nlps_sent: u32 = 0;
    let mut udp_sent: u32 = 0;
    let mut log_tick: u32 = 0;
    let mut led_state = false;

    let mut payload_buf: String<64> = String::new();
    let mut line: String<160> = String::new();

    loop {
        usb_dev.poll(&mut [&mut serial, &mut reset_iface]);
        // If a USB control transfer requested a reboot (e.g. picotool -f),
        // honor it from clean main-loop context so the STATUS stage of
        // the originating SETUP transaction completed first.
        if let Some(kind) = reset_iface.take_pending_reboot() {
            hal::reboot::reboot(kind, pico_reset::RebootArch::Normal);
        }
        let now = timer.get_counter().ticks();

        // RX decoding is interrupt-driven now (DMA_IRQ_0 handler in
        // eth_mac.rs) — no per-iteration poll needed. The main loop's
        // 2.18 ms iteration budget is gone.

        // R4.4: smoltcp drives ingress (drains the inbox) + egress (any
        // queued ARP/ICMP/socket TX). Cheap when there's nothing to do.
        let now_inst = Instant::from_micros(now as i64);
        iface.poll(now_inst, &mut mac, &mut sockets);

        // R4.6: UDP echo on port 1234. R7: tiny HTTP server on port 80.
        serve_udp_echo(&mut sockets, udp_handle);
        serve_http(&mut sockets, tcp_handle, log_tick, nlps_sent, udp_sent);

        // NLP every 16 ms — IEEE 802.3 link-integrity keepalive.
        if now >= next_nlp {
            next_nlp = next_nlp.wrapping_add(16_000);
            mac.send_nlp();
            nlps_sent = nlps_sent.wrapping_add(1);
        }

        // UDP broadcast every 200 ms — mirrors the C reference's payload.
        if now >= next_udp {
            next_udp = next_udp.wrapping_add(200_000);
            payload_buf.clear();
            let _ = write!(
                payload_buf,
                "Hello World!! Raspico 10BASE-T Rust !! n={}",
                udp_sent
            );
            mac.send_udp_broadcast(&endpoint, payload_buf.as_bytes());
            udp_sent = udp_sent.wrapping_add(1);
        }

        // Heartbeat + status print every 1 s.
        if now >= next_log {
            next_log = next_log.wrapping_add(1_000_000);
            log_tick = log_tick.wrapping_add(1);
            led_state = !led_state;
            if led_state {
                led.set_high().unwrap();
            } else {
                led.set_low().unwrap();
            }
            log_status(&mut serial, &mut line, &mut mac, log_tick, nlps_sent, udp_sent);
            nlps_sent = 0; // [R2b] reports nlps as a per-second rate
        }
    }
}

/// R4.6: UDP echo on port 1234 — bind lazily, then drain up to a few received
/// datagrams per poll and echo each back to its sender (the per-poll cap keeps
/// the loop bounded). `echo_buf` is sized for a full-MTU UDP payload
/// (1500 − 20 IP − 8 UDP = 1472), so larger echoes aren't truncated.
fn serve_udp_echo(sockets: &mut SocketSet, handle: SocketHandle) {
    let socket = sockets.get_mut::<udp::Socket>(handle);
    if !socket.is_open() {
        let _ = socket.bind(1234);
    }
    let mut echo_buf = [0u8; 1472];
    for _ in 0..4 {
        match socket.recv_slice(&mut echo_buf) {
            Ok((len, meta)) => {
                let _ = socket.send_slice(&echo_buf[..len], meta.endpoint);
            }
            Err(_) => break,
        }
    }
}

/// R7: tiny HTTP/1.0 server on port 80 — re-listens after each closed
/// connection, drains the request best-effort (we don't parse it), and writes
/// a fixed-shape 200 OK with build info + uptime, then closes. Exercises TCP
/// handshake / retransmission / windowing through the RX path.
fn serve_http(
    sockets: &mut SocketSet,
    handle: SocketHandle,
    uptime_s: u32,
    nlps: u32,
    udp_sent: u32,
) {
    let socket = sockets.get_mut::<tcp::Socket>(handle);
    if !socket.is_open() {
        let _ = socket.listen(80);
    }
    if socket.may_recv() {
        let _ = socket.recv(|buf| (buf.len(), ()));
    }
    if socket.can_send() {
        let mut body: String<160> = String::new();
        let _ = write!(
            body,
            "Hello from Pico-10BASE-T (Rust)!\r\n\
             uptime={}s nlps={} udp_sent={}\r\n",
            uptime_s, nlps, udp_sent
        );
        let mut head: String<128> = String::new();
        let _ = write!(
            head,
            "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\n\
             Content-Length: {}\r\nConnection: close\r\n\r\n",
            body.len()
        );
        let _ = socket.send_slice(head.as_bytes());
        let _ = socket.send_slice(body.as_bytes());
        socket.close();
    }
}

/// Emit the once-per-second status block over USB CDC: the `[R2b]` heartbeat,
/// the `[Rx]` decode summary (snapshots + resets the IRQ-managed RX stats),
/// the `[Mac]` TX-categorization line + TX hex dump, and a pretty-print + hex
/// dump of the most recently decoded frame.
#[cfg_attr(not(feature = "diag"), allow(unused_variables))] // `mac` only used under `diag`
fn log_status<B: UsbBus>(
    serial: &mut SerialPort<'_, B>,
    line: &mut String<160>,
    mac: &mut eth_mac::EthMac,
    log_tick: u32,
    nlps_sent: u32,
    udp_sent: u32,
) {
    line.clear();
    let _ = writeln!(line, "[R2b] t={} nlps={} udp_sent={}", log_tick, nlps_sent, udp_sent);
    let _ = serial.write(line.as_bytes());

    // Snapshot the IRQ-managed RX stats (also resets the window-scoped fields).
    let rx = eth_mac::snapshot_rx_stats();
    let last_dst_mac: [u8; 6] = if rx.last_frame_snapshot_len >= 6 {
        let mut m = [0u8; 6];
        m.copy_from_slice(&rx.last_frame_snapshot[..6]);
        m
    } else {
        [0; 6]
    };
    line.clear();
    let _ = writeln!(
        line,
        "[Rx] dec={} ok={} fail={} filt={} dst={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        rx.frames_decoded, rx.fcs_ok, rx.fcs_fail, rx.frames_filtered,
        last_dst_mac[0], last_dst_mac[1], last_dst_mac[2],
        last_dst_mac[3], last_dst_mac[4], last_dst_mac[5]
    );
    let _ = serial.write(line.as_bytes());

    // Verbose diagnostics (off unless `--features diag`): the [Mac]
    // TX-categorization line + TX hex dump, and the decoded-frame
    // pretty-print + hex dump. These dominate the per-second CDC output and
    // pull in the EthMac TX stats; the lean default build skips them.
    #[cfg(feature = "diag")]
    {
        line.clear();
        let _ = writeln!(
            line,
            "[Mac] iface_rx={} tx_arp={} tx_icmp={} tx_udp={} tx_other={} inbox_drop={} inbox_hwm={} carry_cap={} last_tx_len={}",
            mac.stats.rx_handed_out, mac.stats.tx_arp, mac.stats.tx_icmp, mac.stats.tx_udp,
            mac.stats.tx_other, rx.inbox_dropped, rx.inbox_high_water, rx.carry_capped,
            mac.stats.last_tx_len,
        );
        let _ = serial.write(line.as_bytes());
        let tx_n = (mac.stats.last_tx_len as usize).min(mac.stats.last_tx.len());
        hex_dump(serial, line, "tx ", &mac.stats.last_tx[..tx_n]);
        mac.stats.rx_handed_out = 0;
        mac.stats.tx_handed_out = 0;
        mac.stats.tx_consumed = 0;
        mac.stats.tx_arp = 0;
        mac.stats.tx_icmp = 0;
        mac.stats.tx_udp = 0;
        mac.stats.tx_other = 0;

        // Pretty-print the most recently decoded frame, same shape as
        // ../Pico-10BASE-T/src/eth_rx.c:eth_rx_decode_frame() output.
        if rx.last_frame_snapshot_len > 0 {
            let f = &rx.last_frame_snapshot[..rx.last_frame_snapshot_len];
            let etype = if f.len() >= 14 {
                u16::from_be_bytes([f[12], f[13]])
            } else {
                0
            };
            line.clear();
            let _ = writeln!(
                line,
                "[Rx] frame {} bytes, FCS {} - dst {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} src {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} type={:04x}",
                rx.last_frame_len,
                if rx.last_frame_was_ok { "OK" } else { "FAIL" },
                f[0], f[1], f[2], f[3], f[4], f[5],
                f[6], f[7], f[8], f[9], f[10], f[11],
                etype,
            );
            let _ = serial.write(line.as_bytes());
            let dump_n = f.len().min(64);
            hex_dump(serial, line, "", &f[..dump_n]);
        }
    }
}

/// Write a 16-bytes-per-row hex dump of `data` to the USB CDC serial port,
/// one `serial.write` per row. `label` is inserted after the two-space
/// indent and before the offset (e.g. `"tx "` → `  tx 0000: ..`, `""` →
/// `  0000: ..`). `line` is a caller-owned scratch buffer, reused per row.
/// Only used by the verbose `diag` diagnostics.
#[cfg(feature = "diag")]
fn hex_dump<B: UsbBus>(
    serial: &mut SerialPort<'_, B>,
    line: &mut String<160>,
    label: &str,
    data: &[u8],
) {
    for (row, chunk) in data.chunks(16).enumerate() {
        line.clear();
        let _ = write!(line, "  {}{:04x}:", label, row * 16);
        for b in chunk {
            let _ = write!(line, " {:02x}", b);
        }
        let _ = writeln!(line);
        let _ = serial.write(line.as_bytes());
    }
}

/// Picotool 'binary info' so `picotool info` reports something useful.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Pico-10BASE-T (Rust port)"),
    hal::binary_info::rp_program_url!(c"https://github.com/kingyoPiyo/Pico-10BASE-T"),
];
