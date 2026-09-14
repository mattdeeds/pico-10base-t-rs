//! Pico-10BASE-T firmware: 10BASE-T NIC, wireless AP, or router.
//!
//! Default build: software 10BASE-T NIC with smoltcp (ARP, ICMP, UDP echo,
//! HTTP) and USB CDC logs. `wireless` and `router` hand core 0 to embassy.
//! C reference: `../Pico-10BASE-T/`.

#![no_std]
#![no_main]
// `wireless` builds (router included) leave 10BT code unused; allow it.
#![cfg_attr(
    feature = "wireless",
    allow(dead_code, unused_variables, unused_mut, unused_imports)
)]

// Transport modules come from the library crate.
use pico_10base_t_rs::{eth_mac, eth_rx, eth_tx, multicore_riscv, pico_reset};
// `pio_util` is used by `wireless` for the gSPI divider.
#[cfg(feature = "wireless")]
use pico_10base_t_rs::{cyw43_phy, pio_util};
// Per-core CPU counters (router only).
#[cfg(feature = "router")]
use pico_10base_t_rs::cycles;
// cyw43 LAN: gSPI transport, executor, and tasks.
#[cfg(feature = "wireless")]
mod wireless;
// LAN DHCP server.
#[cfg(feature = "wireless")]
mod dhcp_server;
// WAN DHCP client, ping, and DNS.
#[cfg(any(feature = "wan-dhcp", feature = "router"))]
mod wan;
// L3 forwarding and NAPT device (router only).
#[cfg(feature = "router")]
mod forward;
// NAPT conntrack table (router only).
#[cfg(feature = "router")]
mod conntrack;

use panic_halt as _;

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};
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
#[cfg(feature = "clock-150mhz")]
use hal::pll::common_configs::PLL_SYS_150MHZ;
use hal::xosc::setup_xosc_blocking;
use hal::clocks::ClocksManager;
use smoltcp::iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage};
use smoltcp::socket::{tcp, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress};
// Static IP types, used only without `wan-dhcp`.
#[cfg(not(feature = "wan-dhcp"))]
use smoltcp::wire::{IpAddress, IpCidr, Ipv4Address};
// WAN DHCP client sockets (`wan-dhcp`).
#[cfg(feature = "wan-dhcp")]
use smoltcp::socket::{dhcpv4, dns, icmp};
#[cfg(feature = "wan-dhcp")]
use smoltcp::phy::{ChecksumCapabilities, Device as _};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

const HW_PIN_TXD: u8 = 14; // ISL3177E DI
const HW_PIN_RXD: u8 = 13; // ISL3177E RO

/// Our 10BASE-T (WAN) MAC: RX filter and smoltcp address.
const OUR_MAC: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];

/// Tell the Boot ROM about our application.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Pico 2 board has a 12 MHz crystal.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Watchdog timeout (HAL max ~8.38 s). Reboots the chip if core 0 stalls.
/// Fed every [`WDT_FEED_MS`], ~12× margin over normal stalls.
pub const WDT_TIMEOUT_US: u32 = 6_000_000;
/// Watchdog feed interval for the executor builds' dedicated feeder task.
pub const WDT_FEED_MS: u64 = 500;

/// 240 MHz overclock (VCO 1200 / 5). Integer PIO dividers: TX ÷12, RX ÷4.
#[cfg_attr(feature = "clock-150mhz", allow(dead_code))]
const PLL_SYS_240MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1200),
    refdiv: 1,
    post_div1: 5,
    post_div2: 1,
};

/// PLL choice: 240 MHz, or stock 150 MHz with `clock-150mhz` (fractional dividers).
#[cfg(not(feature = "clock-150mhz"))]
const PLL_SYS_SELECTED: PLLConfig = PLL_SYS_240MHZ;
#[cfg(feature = "clock-150mhz")]
const PLL_SYS_SELECTED: PLLConfig = PLL_SYS_150MHZ;

/// Core-1 liveness counter: core 1 increments it, core 0 logs it.
static CORE1_TICKS: AtomicU32 = AtomicU32::new(0);

/// Core 1's stack (16 KB), aligned for the launch trampoline.
/// Sized for decode: a 1600-byte frame `Vec` plus trap and call frames.
#[repr(align(16))]
struct Core1Stack([usize; 4096]);
static mut CORE1_STACK: Core1Stack = Core1Stack([0; 4096]);

/// Core 1 entry: take `DMA_IRQ_0`, then decode queued images after each wake.
extern "C" fn core1_entry() -> ! {
    // Enable core-1 `mcycle` for CPU counters (router build).
    #[cfg(feature = "router")]
    cycles::enable_mcycle();
    // Safety: core 0 installed the RX engine before launching this core.
    unsafe {
        hal::arch::interrupt_unmask(hal::pac::Interrupt::DMA_IRQ_0);
        hal::arch::interrupt_enable();
    }
    let mut n: u32 = 0;
    loop {
        hal::arch::wfi();
        // The IRQ captured an image and re-armed. Decode here, with no deadline.
        eth_mac::drain_rx_images();
        n = n.wrapping_add(1);
        CORE1_TICKS.store(n, Ordering::Relaxed);
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // PLL_SYS: see `PLL_SYS_SELECTED`.
    let xosc = setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz()).unwrap();
    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u16);
    let mut clocks = ClocksManager::new(pac.CLOCKS);
    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        PLL_SYS_SELECTED,
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
    // TIMER0 out of reset. The wireless time driver owns ALARM0; don't arm alarms.
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // ── Router image: WAN (10BASE-T, core-1 RX) + cyw43 LAN under one executor.
    // No GP25 LED: GP25 is the gSPI CS.
    #[cfg(feature = "router")]
    {
        let _ = &timer; // the time driver reads TIMER0 directly
        // Enable core-0 `mcycle` for the forwarding counters.
        cycles::enable_mcycle();
        let sys_clk_hz = clocks.system_clock.freq().to_Hz();

        // WAN: GP14/GP13 to PIO0. Pin handles must outlive the executor.
        let _tx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio14.into_function();
        let _rx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio13.into_function();
        let mut fifo = sio.fifo;
        let (mac, core1_ok) = setup_eth_mac(
            pac.PIO0, pac.DMA, &mut pac.PSM, &mut fifo, &mut pac.RESETS, sys_clk_hz,
        );

        // LAN: GP24/GP29 to PIO1, WL_ON = GP23, CS = GP25. Bus idles through power-up.
        let _cyw_data: hal::gpio::Pin<_, hal::gpio::FunctionPio1, _> = pins.gpio24.into_function();
        let _cyw_clk: hal::gpio::Pin<_, hal::gpio::FunctionPio1, _> = pins.gpio29.into_function();
        let (mut pio1, pio1_sm0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
        let pwr = pins.gpio23.into_push_pull_output();
        let spi = wireless::PioSpiCyw43::new(&mut pio1, pio1_sm0, sys_clk_hz);

        wireless::run_router(
            mac, core1_ok, pwr, spi, pac.USB, pac.USB_DPRAM, clocks.usb_clock, &mut pac.RESETS,
            watchdog,
        );
    }

    // ── Wireless-only image: cyw43 LAN under the executor; 10BASE-T not started.
    #[cfg(all(feature = "wireless", not(feature = "router")))]
    {
        let _ = &timer; // keep TIMER0 out of reset
        let _cyw_data: hal::gpio::Pin<_, hal::gpio::FunctionPio1, _> = pins.gpio24.into_function();
        let _cyw_clk: hal::gpio::Pin<_, hal::gpio::FunctionPio1, _> = pins.gpio29.into_function();
        let (mut pio1, pio1_sm0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
        let sys_clk_hz_w = clocks.system_clock.freq().to_Hz();
        let pwr = pins.gpio23.into_push_pull_output();
        let spi = wireless::PioSpiCyw43::new(&mut pio1, pio1_sm0, sys_clk_hz_w);
        wireless::run(pwr, spi, pac.USB, pac.USB_DPRAM, clocks.usb_clock, &mut pac.RESETS, watchdog);
    }

    // ── 10BASE-T NIC image (default build). Exactly one arm is compiled per build.
    #[cfg(not(feature = "wireless"))]
    main_10bt(
        pac.PIO0, pac.DMA, pac.PSM, pac.USB, pac.USB_DPRAM, pac.RESETS, sio.fifo, clocks, pins, timer,
        watchdog,
    );
}

/// Build the 10BASE-T path: TX, carrier detect, and RX on PIO0, then launch
/// core 1 for RX. Returns the `EthMac` and whether core 1 launched.
///
/// GP13/GP14 must already be `FunctionPio0`. Call once. Order matters:
/// `EthRx::new` → `install_rx` → `launch_core1_riscv` (enables the IRQ).
fn setup_eth_mac(
    pio0: hal::pac::PIO0,
    dma: hal::pac::DMA,
    psm: &mut hal::pac::PSM,
    sio_fifo: &mut hal::sio::SioFifo,
    resets: &mut hal::pac::RESETS,
    sys_clk_hz: u32,
) -> (eth_mac::EthMac, bool) {
    // PIO0: SM0 TX, SM1 RX sampler, SM2 carrier detect. SM3 and PIO1 are free.
    let (mut pio0, sm0, sm1, sm2, _sm3) = pio0.split(resets);
    let eth_tx = eth_tx::EthTx::new(&mut pio0, sm0, sm2, HW_PIN_TXD, HW_PIN_RXD, sys_clk_hz);

    // DMA ch0/ch1 fill two static 16 KB half-buffers from the RX FIFO.
    let dma = dma.split(resets);
    let rx_buf_a = singleton!(: [u32; eth_rx::BUF_WORDS] = [0; eth_rx::BUF_WORDS]).unwrap();
    let rx_buf_b = singleton!(: [u32; eth_rx::BUF_WORDS] = [0; eth_rx::BUF_WORDS]).unwrap();
    let rx_carry =
        singleton!(: [u8; eth_rx::MAX_CARRY_BYTES] = [0; eth_rx::MAX_CARRY_BYTES]).unwrap();
    let eth_rx = eth_rx::EthRx::new(
        &mut pio0, sm1, HW_PIN_RXD, sys_clk_hz, dma.ch0, dma.ch1, rx_buf_a, rx_buf_b, rx_carry,
    );

    // Install RX before launching core 1. The launch is bounded, so a dead
    // core 1 returns an error instead of hanging.
    let _ = eth_mac::install_rx(eth_rx, OUR_MAC);
    let core1_stack = unsafe { &mut (*core::ptr::addr_of_mut!(CORE1_STACK)).0 };
    let core1_launch_ok =
        unsafe { multicore_riscv::launch_core1_riscv(psm, sio_fifo, core1_stack, core1_entry).is_ok() };

    // EthMac owns TX only; RX state is static.
    (eth_mac::EthMac::new(eth_tx), core1_launch_ok)
}

/// Default NIC build: smoltcp on a blocking core-0 poll loop. Never returns.
#[cfg(not(feature = "wireless"))]
#[allow(clippy::too_many_arguments)] // dispatch boundary — resources handed in by `main`
fn main_10bt(
    pio0: hal::pac::PIO0,
    dma: hal::pac::DMA,
    mut psm: hal::pac::PSM,
    usb: hal::pac::USB,
    usb_dpram: hal::pac::USB_DPRAM,
    mut resets: hal::pac::RESETS,
    mut sio_fifo: hal::sio::SioFifo,
    clocks: hal::clocks::ClocksManager,
    pins: hal::gpio::Pins,
    timer: hal::Timer<hal::timer::CopyableTimer0>,
    mut watchdog: hal::Watchdog,
) -> ! {
    let mut led = pins.gpio25.into_push_pull_output();

    // GP14 = DI, GP13 = RO, both PIO0. Handles must outlive the loop.
    let _tx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio14.into_function();
    let _rx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio13.into_function();
    let sys_clk_hz = clocks.system_clock.freq().to_Hz();
    let (mut mac, core1_launch_ok) =
        setup_eth_mac(pio0, dma, &mut psm, &mut sio_fifo, &mut resets, sys_clk_hz);

    let our_mac = EthernetAddress(OUR_MAC);
    let mut iface_config = Config::new(HardwareAddress::Ethernet(our_mac));
    iface_config.random_seed = timer.get_counter().ticks();
    let now0_inst = Instant::from_micros(timer.get_counter().ticks() as i64);
    let mut iface = Interface::new(iface_config, &mut mac, now0_inst);
    // Static IP matching the C reference. `wan-dhcp` leaves it to the DHCP client.
    #[cfg(not(feature = "wan-dhcp"))]
    iface.update_ip_addrs(|addrs| {
        addrs
            .push(IpCidr::new(IpAddress::Ipv4(Ipv4Address::new(192, 168, 37, 24)), 24))
            .unwrap();
    });

    // Socket slots: 5 by default, 6 with fd-bench, 8 with wan-dhcp.
    #[cfg(all(not(feature = "wan-dhcp"), not(feature = "fd-bench")))]
    let mut sockets_storage: [SocketStorage; 5] = [SocketStorage::EMPTY; 5];
    #[cfg(all(not(feature = "wan-dhcp"), feature = "fd-bench"))]
    let mut sockets_storage: [SocketStorage; 6] = [SocketStorage::EMPTY; 6];
    #[cfg(feature = "wan-dhcp")]
    let mut sockets_storage: [SocketStorage; 8] = [SocketStorage::EMPTY; 8];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    // UDP echo socket on port 1234.
    let mut udp_rx_meta = [udp::PacketMetadata::EMPTY; 8];
    let mut udp_rx_payload = [0u8; 2048];
    let mut udp_tx_meta = [udp::PacketMetadata::EMPTY; 8];
    let mut udp_tx_payload = [0u8; 2048];
    let udp_rx_buffer = udp::PacketBuffer::new(&mut udp_rx_meta[..], &mut udp_rx_payload[..]);
    let udp_tx_buffer = udp::PacketBuffer::new(&mut udp_tx_meta[..], &mut udp_tx_payload[..]);
    let udp_socket = udp::Socket::new(udp_rx_buffer, udp_tx_buffer);
    let udp_handle: SocketHandle = sockets.add(udp_socket);

    // HTTP server on port 80. `http-bulk-test` streams 1 MB instead.
    let mut tcp_rx_storage = [0u8; 1024];
    #[cfg(not(feature = "http-bulk-test"))]
    let mut tcp_tx_storage = [0u8; 1024];
    // Bulk test: a 32 KB send window keeps enough segments in flight that
    // residual collisions trigger fast retransmit, not an RTO stall.
    #[cfg(feature = "http-bulk-test")]
    let mut tcp_tx_storage = [0u8; 32 * 1024];
    let tcp_rx_buffer = tcp::SocketBuffer::new(&mut tcp_rx_storage[..]);
    let tcp_tx_buffer = tcp::SocketBuffer::new(&mut tcp_tx_storage[..]);
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);
    let tcp_handle: SocketHandle = sockets.add(tcp_socket);

    // fd-bench: TCP upload sink on port 9999, run alongside the :80 download.
    #[cfg(feature = "fd-bench")]
    let mut sink_rx_storage = [0u8; 32 * 1024];
    #[cfg(feature = "fd-bench")]
    let mut sink_tx_storage = [0u8; 2048];
    #[cfg(feature = "fd-bench")]
    let sink_handle: SocketHandle = sockets.add({
        let mut sink = tcp::Socket::new(
            tcp::SocketBuffer::new(&mut sink_rx_storage[..]),
            tcp::SocketBuffer::new(&mut sink_tx_storage[..]),
        );
        // Immediate ACKs ride the gap after each data frame. A 10 ms delayed ACK
        // collided with Linux's tail-loss probe on the half-duplex wire.
        sink.set_ack_delay(None);
        sink
    });

    // WAN DHCP client sockets. Buffers live on this never-returning stack.
    #[cfg(feature = "wan-dhcp")]
    let dhcp_handle: SocketHandle = sockets.add(dhcpv4::Socket::new());

    #[cfg(feature = "wan-dhcp")]
    let mut icmp_rx_meta = [icmp::PacketMetadata::EMPTY; 8];
    #[cfg(feature = "wan-dhcp")]
    let mut icmp_rx_payload = [0u8; 512];
    #[cfg(feature = "wan-dhcp")]
    let mut icmp_tx_meta = [icmp::PacketMetadata::EMPTY; 8];
    #[cfg(feature = "wan-dhcp")]
    let mut icmp_tx_payload = [0u8; 512];
    #[cfg(feature = "wan-dhcp")]
    let icmp_handle: SocketHandle = sockets.add(icmp::Socket::new(
        icmp::PacketBuffer::new(&mut icmp_rx_meta[..], &mut icmp_rx_payload[..]),
        icmp::PacketBuffer::new(&mut icmp_tx_meta[..], &mut icmp_tx_payload[..]),
    ));

    // DNS query slots. Servers come from the DHCP lease.
    #[cfg(feature = "wan-dhcp")]
    let mut dns_queries: [Option<dns::DnsQuery>; 2] = [None, None];
    #[cfg(feature = "wan-dhcp")]
    let dns_handle: SocketHandle = sockets.add(dns::Socket::new(&[], &mut dns_queries[..]));

    #[cfg(feature = "wan-dhcp")]
    let mut wan = crate::wan::WanState::new();
    // The device's TX checksum capabilities — used to emit/parse ICMP echoes.
    #[cfg(feature = "wan-dhcp")]
    let wan_checksum: ChecksumCapabilities = mac.capabilities().checksum;

    // Broadcast UDP endpoint, matching the C reference host setup.
    let endpoint = eth_tx::UdpEndpoint {
        src_mac: OUR_MAC,
        dst_mac: [0xFF; 6], // broadcast
        src_ip: [192, 168, 37, 24],
        dst_ip: [192, 168, 37, 19],
        src_port: 1234,
        dst_port: 1234,
    };
    // USB CDC: appears on the host as /dev/ttyACM0.
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        usb,
        usb_dpram,
        clocks.usb_clock,
        true,
        &mut resets,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    // Reset interface so `picotool -f` can reboot us to BOOTSEL.
    let mut reset_iface = pico_reset::PicoResetInterface::new(&usb_bus);

    // Serial = chip ID, as BOOTSEL reports it, so picotool can track the reboot.
    let mut serial_str: String<16> = String::new();
    match hal::rom_data::sys_info_api::chip_info() {
        Ok(Some(info)) => {
            let _ = write!(serial_str, "{:08X}{:08X}", info.wafer_id, info.device_id);
        }
        _ => {
            let _ = write!(serial_str, "0000000000000000");
        }
    }

    // 2e8a:000a is pico-sdk's stdio_usb CDC ID, which picotool recognizes.
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
    // wan-dhcp: ping and DNS once a second.
    #[cfg(feature = "wan-dhcp")]
    let mut next_ping = now0 + 1_000_000;
    let mut nlps_sent: u32 = 0;
    let mut udp_sent: u32 = 0;
    let mut log_tick: u32 = 0;
    let mut led_state = false;

    let mut payload_buf: String<64> = String::new();
    let mut line: String<160> = String::new();

    #[cfg(feature = "http-bulk-test")]
    let mut http_bulk_state = HttpBulkState::Idle;

    // fd-bench: state for the per-second `[Sink]` line.
    #[cfg(feature = "fd-bench")]
    let mut fd_prev_sink: u32 = 0;
    #[cfg(feature = "fd-bench")]
    let mut fd_last_us: u64 = now0;
    // Loop iterations/s separates a loop-rate cap from a TCP cap.
    #[cfg(feature = "fd-bench")]
    let mut fd_loop_iters: u32 = 0;

    // Arm the watchdog; fed at the top of each iteration.
    watchdog.start(hal::fugit::MicrosDurationU32::micros(WDT_TIMEOUT_US));

    loop {
        watchdog.feed();
        #[cfg(feature = "fd-bench")]
        {
            fd_loop_iters = fd_loop_iters.wrapping_add(1);
        }
        usb_dev.poll(&mut [&mut serial, &mut reset_iface]);
        // Reboot on request here, after the SETUP STATUS stage completed.
        if let Some(kind) = reset_iface.take_pending_reboot() {
            hal::reboot::reboot(kind, pico_reset::RebootArch::Normal);
        }
        let now = timer.get_counter().ticks();

        // RX decodes on core 1; nothing to poll here.

        // smoltcp ingress (inbox) and egress.
        let now_inst = Instant::from_micros(now as i64);
        iface.poll(now_inst, &mut mac, &mut sockets);

        // WAN client work after iface.poll: lease, ping replies, DNS results.
        #[cfg(feature = "wan-dhcp")]
        {
            crate::wan::dhcp_apply(&mut iface, &mut sockets, dhcp_handle, dns_handle, &mut wan);
            crate::wan::ping_drain(&mut sockets, icmp_handle, &mut wan, &wan_checksum);
            crate::wan::dns_harvest(&mut sockets, dns_handle, &mut wan);
        }

        // UDP echo :1234 and HTTP :80.
        serve_udp_echo(&mut sockets, udp_handle);
        #[cfg(not(feature = "http-bulk-test"))]
        serve_http(&mut sockets, tcp_handle, log_tick, nlps_sent, udp_sent);
        #[cfg(feature = "http-bulk-test")]
        serve_http_bulk(&mut sockets, tcp_handle, &mut http_bulk_state);
        // fd-bench: drain the port-9999 upload sink (counts bytes, re-listens).
        #[cfg(feature = "fd-bench")]
        serve_fd_sink(&mut sockets, sink_handle);

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

        // Once leased: ping and start a DNS query each second.
        #[cfg(feature = "wan-dhcp")]
        if now >= next_ping {
            next_ping = next_ping.wrapping_add(1_000_000);
            if wan.addr.is_some() {
                crate::wan::ping_send(&mut sockets, icmp_handle, &mut wan, &wan_checksum);
                crate::wan::dns_start(&mut iface, &mut sockets, dns_handle, &mut wan);
            }
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
            log_status(
                &mut serial, &mut line, &mut mac, log_tick, nlps_sent, udp_sent, core1_launch_ok,
            );
            #[cfg(feature = "wan-dhcp")]
            log_wan(&mut serial, &mut line, &wan);
            // fd-bench: `[Sink]` upload rate over the measured window.
            #[cfg(feature = "fd-bench")]
            {
                let elapsed_us = now.wrapping_sub(fd_last_us).max(1);
                fd_last_us = now;
                let sink_now = FD_SINK_RX.load(Ordering::Relaxed);
                let d_sink = sink_now.wrapping_sub(fd_prev_sink);
                fd_prev_sink = sink_now;
                let rx_kbps = (d_sink as u64 * 1_000 / elapsed_us) as u32;
                let iters_per_s = (fd_loop_iters as u64 * 1_000_000 / elapsed_us) as u32;
                fd_loop_iters = 0;
                line.clear();
                let _ = writeln!(
                    line,
                    "[Sink] rx={}KB/s total={}KB loop={}/s",
                    rx_kbps, sink_now / 1024, iters_per_s
                );
                let _ = serial.write(line.as_bytes());
            }
            nlps_sent = 0; // [R2b] reports nlps as a per-second rate
        }
    }
}

/// Write the 1 Hz `[Wan]` line over CDC.
#[cfg(feature = "wan-dhcp")]
fn log_wan<B: UsbBus>(serial: &mut SerialPort<'_, B>, line: &mut String<160>, wan: &crate::wan::WanState) {
    line.clear();
    let _ = write!(line, "[Wan] ");
    wan.write_status(line);
    let _ = writeln!(line);
    let _ = serial.write(line.as_bytes());
}

/// UDP echo on port 1234. Up to 4 datagrams per poll, full-MTU payloads.
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

/// HTTP/1.0 on port 80: fixed 200 OK with uptime, then close.
#[cfg(not(feature = "http-bulk-test"))]
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

/// `http-bulk-test` body: 1 MB of 0x55, refilled across polls.
#[cfg(feature = "http-bulk-test")]
const HTTP_BULK_BYTES: usize = 1024 * 1024;

#[cfg(feature = "http-bulk-test")]
enum HttpBulkState {
    /// Waiting for a fresh connection to open + reach the may-send state.
    Idle,
    /// Header has been queued; body sender writes up to `remaining` bytes.
    Sending { remaining: usize },
}

#[cfg(feature = "http-bulk-test")]
fn serve_http_bulk(
    sockets: &mut SocketSet,
    handle: SocketHandle,
    state: &mut HttpBulkState,
) {
    use core::fmt::Write as _;
    // 1 KB refill chunk of 0x55.
    const CHUNK: [u8; 1024] = [0x55; 1024];

    let socket = sockets.get_mut::<tcp::Socket>(handle);

    if !socket.is_open() {
        *state = HttpBulkState::Idle;
        let _ = socket.listen(80);
        return;
    }
    if socket.may_recv() {
        let _ = socket.recv(|buf| (buf.len(), ()));
    }

    match state {
        HttpBulkState::Idle => {
            if socket.can_send() {
                let mut head: String<128> = String::new();
                let _ = write!(
                    head,
                    "HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\n\
                     Content-Length: {}\r\nConnection: close\r\n\r\n",
                    HTTP_BULK_BYTES
                );
                let _ = socket.send_slice(head.as_bytes());
                *state = HttpBulkState::Sending { remaining: HTTP_BULK_BYTES };
            }
        }
        HttpBulkState::Sending { remaining } => {
            while *remaining > 0 && socket.can_send() {
                let n = (*remaining).min(CHUNK.len());
                let sent = socket.send_slice(&CHUNK[..n]).unwrap_or(0);
                if sent == 0 {
                    break;
                }
                *remaining -= sent;
            }
            if *remaining == 0 {
                socket.close();
                *state = HttpBulkState::Idle;
            }
        }
    }
}

/// fd-bench: bytes received on the port-9999 sink.
#[cfg(feature = "fd-bench")]
static FD_SINK_RX: AtomicU32 = AtomicU32::new(0);

/// fd-bench: drain and count the port-9999 upload; re-listen after FIN.
#[cfg(feature = "fd-bench")]
fn serve_fd_sink(sockets: &mut SocketSet, handle: SocketHandle) {
    let socket = sockets.get_mut::<tcp::Socket>(handle);
    if !socket.is_open() {
        let _ = socket.listen(9999);
        return;
    }
    if socket.may_recv() {
        let n = socket.recv(|buf| (buf.len(), buf.len())).unwrap_or(0);
        if n > 0 {
            FD_SINK_RX.fetch_add(n as u32, Ordering::Relaxed);
        }
    }
    // Peer sent FIN: close to re-listen.
    if !socket.may_recv() && socket.may_send() {
        socket.close();
    }
}

/// Write the 1 Hz status lines: `[R2b]`, `[Core1]`, `[Rx]`, and `diag` extras.
#[cfg_attr(not(feature = "diag"), allow(unused_variables))] // `mac` only used under `diag`
fn log_status<B: UsbBus>(
    serial: &mut SerialPort<'_, B>,
    line: &mut String<160>,
    mac: &mut eth_mac::EthMac,
    log_tick: u32,
    nlps_sent: u32,
    udp_sent: u32,
    core1_launch_ok: bool,
) {
    line.clear();
    let _ = writeln!(line, "[R2b] t={} nlps={} udp_sent={}", log_tick, nlps_sent, udp_sent);
    let _ = serial.write(line.as_bytes());

    // Core-1 liveness: `launch=ok` and climbing `ticks`.
    line.clear();
    let _ = writeln!(
        line,
        "[Core1] launch={} ticks={}",
        if core1_launch_ok { "ok" } else { "FAIL" },
        CORE1_TICKS.load(Ordering::Relaxed),
    );
    let _ = serial.write(line.as_bytes());

    // cyw43 stage flags. Never runs: only non-wireless builds call log_status.
    #[cfg(feature = "wireless")]
    {
        line.clear();
        let _ = writeln!(
            line,
            "[Cyw43] new={} init={} led={} (1=ok: new=fw+nvram, init=CLM+wifi, led=gpio_set blink)",
            wireless::CYW43_NEW_DONE.load(Ordering::Relaxed),
            wireless::CYW43_INIT_DONE.load(Ordering::Relaxed),
            wireless::CYW43_LED_DONE.load(Ordering::Relaxed),
        );
        let _ = serial.write(line.as_bytes());
    }

    // Snapshot and reset RX stats.
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
        "[Rx] dec={} ok={} fail={} filt={} dst={}",
        rx.frames_decoded, rx.fcs_ok, rx.fcs_fail, rx.frames_filtered,
        mac_str(last_dst_mac)
    );
    let _ = serial.write(line.as_bytes());

    // `diag` only: stitch, TX, and last-frame dumps.
    #[cfg(feature = "diag")]
    {
        // Carry-path decode health and RX overflow counters.
        line.clear();
        let _ = writeln!(
            line,
            "[Stitch] dec={} fail={} rxstall={} img_drop={}",
            eth_mac::STITCH_DEC.swap(0, Ordering::Relaxed),
            eth_mac::STITCH_FAIL.swap(0, Ordering::Relaxed),
            eth_mac::RXSTALL_HALVES.swap(0, Ordering::Relaxed),
            eth_mac::IMG_DROP.swap(0, Ordering::Relaxed),
        );
        let _ = serial.write(line.as_bytes());
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

        // Pretty-print the last decoded frame, like the C reference.
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
                "[Rx] frame {} bytes, FCS {} - dst {} src {} type={:04x}",
                rx.last_frame_len,
                if rx.last_frame_was_ok { "OK" } else { "FAIL" },
                mac_str([f[0], f[1], f[2], f[3], f[4], f[5]]),
                mac_str([f[6], f[7], f[8], f[9], f[10], f[11]]),
                etype,
            );
            let _ = serial.write(line.as_bytes());
            let dump_n = f.len().min(64);
            hex_dump(serial, line, "", &f[..dump_n]);
        }
    }
}

/// Format a MAC as `aa:bb:cc:dd:ee:ff`.
pub fn mac_str(mac: [u8; 6]) -> String<17> {
    let mut s = String::new();
    let _ = write!(
        s,
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    s
}

/// Hex-dump `data` to CDC, 16 bytes per row, prefixed by `label`.
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
