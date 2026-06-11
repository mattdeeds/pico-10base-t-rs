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
// The `wireless` image is a standalone build (docs/router-plan.md §11/§12): the
// embassy executor owns core 0 and 10BASE-T is not started, so the 10BT data
// path (main_10bt + the eth_* modules + core-1 launch) is compiled but unused.
// Silence the resulting dead-code/unused noise for that build only; the default
// build keeps full lint coverage.
#![cfg_attr(
    feature = "wireless",
    allow(dead_code, unused_variables, unused_mut, unused_imports)
)]

mod crc;
mod eth_mac;
mod eth_rx;
// Edge-track DPLL Manchester decoder (productized — Phase 3b). Excluded
// from the openloop A/B build so the dead-code warnings don't fire.
#[cfg(not(feature = "decoder-openloop"))]
mod eth_rx_dpll;
mod eth_tx;
mod manchester;
mod multicore_riscv;
mod pico_reset;
mod pio_util;
// R13 — wireless router scaffolding (Pico 2 W / CYW43). Gated off by default;
// `--features wireless` compile-checks the cyw43 + async-runtime integration.
#[cfg(feature = "wireless")]
mod wireless;
// R14.3 — smoltcp phy::Device adapter over cyw43's NetDriver (wireless LAN).
#[cfg(feature = "wireless")]
mod cyw43_phy;
// R14.4 — minimal LAN DHCP server (smoltcp UDP :67).
#[cfg(feature = "wireless")]
mod dhcp_server;
// R15 — shared WAN-as-DHCP-client logic (10BASE-T side: dhcpv4 client + ICMP
// ping + DNS resolve). Used by `main_10bt` (R15a, `wan-dhcp`) and the executor's
// `wan_task` (R15b, `router`).
#[cfg(any(feature = "wan-dhcp", feature = "router"))]
mod wan;
// R16 — L3 forwarding (LAN↔WAN transit, no NAT): the ForwardingDevice phy
// wrapper + cross-interface queues + neighbor learning. Router build only.
#[cfg(feature = "router")]
mod forward;
// R17 — NAPT connection tracking (heapless conntrack table + incremental
// checksum helpers). Wired into the WAN ForwardingDevice. Router build only.
#[cfg(feature = "router")]
mod conntrack;
// Perf characterization step 2 — `mcycle`-based per-core CPU-utilisation
// counters (core-1 RX decode + core-0 forwarding fast-path). Router build only.
#[cfg(feature = "router")]
mod cycles;

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
// IpAddress/IpCidr/Ipv4Address are only used by the static-IP push in `main_10bt`
// (compiled when neither `wan-dhcp` nor `wireless` is set). The wireless/router
// builds blanket-allow unused imports (see the crate attr), so gating on
// `not(wan-dhcp)` keeps the non-wireless `wan-dhcp` build warning-free.
#[cfg(not(feature = "wan-dhcp"))]
use smoltcp::wire::{IpAddress, IpCidr, Ipv4Address};
// R15a — WAN-as-DHCP-client: the dhcpv4/icmp/dns sockets are constructed here;
// the per-poll logic lives in `crate::wan`. We also need the device's checksum
// capabilities to seed the ICMP socket. Gated so the default build pulls none.
#[cfg(feature = "wan-dhcp")]
use smoltcp::socket::{dhcpv4, dns, icmp};
#[cfg(feature = "wan-dhcp")]
use smoltcp::phy::{ChecksumCapabilities, Device as _};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

const HW_PIN_TXD: u8 = 14; // ISL3177E DI
const HW_PIN_RXD: u8 = 13; // ISL3177E RO

/// Our 10BASE-T (WAN) MAC. Used both by the IRQ-side RX filter (`install_rx`)
/// and as the smoltcp `Interface` hardware address — shared by `main_10bt` and
/// the router build's `wan_task`, so it lives in one place.
const OUR_MAC: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];

/// Tell the Boot ROM about our application.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Pico 2 board has a 12 MHz crystal.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// RX-hang watchdog (`docs/rx-bulk-ceiling.md` §6): the device has wedged under
/// sustained full-MTU inbound (link drops / no NLPs, CDC silent, only
/// SWD-recoverable). The RP2350 hardware watchdog reboots the chip if the main
/// loop / executor stops feeding it, so the device self-recovers instead of
/// needing a manual reflash. Fed from the core-0 poll loop (NIC build) or a
/// dedicated executor task (router/wireless). 6 s timeout (HAL max ~8.38 s), fed
/// every [`WDT_FEED_MS`] → ~12× margin over any legitimate core-0 stall (TX
/// critical section ~50 µs, cyw43 gSPI bursts ~ms) so it never false-reboots.
pub const WDT_TIMEOUT_US: u32 = 6_000_000;
/// Watchdog feed interval for the executor builds' dedicated feeder task.
pub const WDT_FEED_MS: u64 = 500;

/// Phase 2d v3 — 240 MHz overclock. VCO 1200 MHz / (5 × 1) = 240 MHz.
/// Integer PIO dividers at this clock: TX 20 MHz = ÷12, RX 60 MHz = ÷4
/// (no fractional jitter). Recovery via SWD if flash gets corrupted at the
/// higher QMI SCK (the DAPLink probe is attached; see the
/// `sysclk-integer-pio-dividers` memory).
#[cfg_attr(feature = "clock-150mhz", allow(dead_code))]
const PLL_SYS_240MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1200),
    refdiv: 1,
    post_div1: 5,
    post_div2: 1,
};

/// Cargo feature `clock-150mhz` selects the HAL's stock 150 MHz PLL config
/// instead of the 240 MHz overclock — for the FCS-ceiling triage (experiment
/// 6: rule overclock side-effects in or out as a contributor). At 150 MHz
/// the PIO dividers go back to fractional (RX ÷2.5, TX ÷7.5, ±3.3 ns jitter).
#[cfg(not(feature = "clock-150mhz"))]
const PLL_SYS_SELECTED: PLLConfig = PLL_SYS_240MHZ;
#[cfg(feature = "clock-150mhz")]
const PLL_SYS_SELECTED: PLLConfig = PLL_SYS_150MHZ;

/// Phase 3a — core-1 liveness counter. Core 1 stores a monotonically
/// increasing value here; core 0 reads it once per second to prove the second
/// Hazard3 core launched and that cross-core shared SRAM is coherent (RP2350
/// has no data caches, so no cache maintenance is needed). Plain atomic
/// store/load only (compiles to `sw`/`lw`) — no lr/sc reservation across cores.
static CORE1_TICKS: AtomicU32 = AtomicU32::new(0);

/// Core 1's stack (16 KB). `#[repr(align(16))]` keeps the launch trampoline's
/// `sp` RV32-ABI aligned. Owned exclusively by core 1 once launched. Sized
/// generously because core 1's `DMA_IRQ_0` handler runs the full decode
/// pipeline (a 1600-byte frame `Vec` is built on this stack per frame, plus
/// the trap frame + nested call frames).
#[repr(align(16))]
struct Core1Stack([usize; 4096]);
static mut CORE1_STACK: Core1Stack = Core1Stack([0; 4096]);

/// Phase 3c core-1 entry point: own the RX decode. Route `DMA_IRQ_0` to this
/// core's interrupt controller (per-hart xh3irq CSR) + enable machine-external
/// interrupts, then sleep between IRQs. The `DMA_IRQ_0` handler (in
/// `eth_mac.rs`) runs the stitch + decode + verify pipeline here — off core 0,
/// so the main loop + smoltcp can't be starved by decode work under load.
extern "C" fn core1_entry() -> ! {
    // Perf step 2: un-inhibit this core's `mcycle` so the DMA_IRQ_0 handler can
    // bracket its own RX-decode cost (router build — see `cycles`).
    #[cfg(feature = "router")]
    cycles::enable_mcycle();
    // Safety: enabling this core's own interrupts. The handler + shared RX
    // state were installed by core 0 (`install_rx`) before this core launched.
    unsafe {
        hal::arch::interrupt_unmask(hal::pac::Interrupt::DMA_IRQ_0);
        hal::arch::interrupt_enable();
    }
    let mut n: u32 = 0;
    loop {
        hal::arch::wfi();
        // Woke to service a DMA-half IRQ (handled via the trap before we
        // resume here). Bump the liveness counter so core 0's 1 Hz log shows
        // core 1 processing halves; ticks climbing == core 1 alive + working.
        n = n.wrapping_add(1);
        CORE1_TICKS.store(n, Ordering::Relaxed);
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // PLL_SYS config selected by the `clock-150mhz` cargo feature:
    // default = 240 MHz overclock (integer PIO dividers), feature-on = the
    // HAL stock 150 MHz (fractional PIO dividers, ±3.3 ns jitter).
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
    // TIMER0 out of reset → its free-running µs counter ticks. The 10BASE-T path
    // uses this hal handle; the wireless time-driver reads TIMER0 directly (and
    // owns ALARM0 / TIMER0_IRQ_0 — see wireless.rs), so don't arm alarms here.
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // ── Router image (R15b, `--features router`): BOTH interfaces live under one
    // embassy executor. Bring up the 10BASE-T (WAN) device on PIO0 + launch core 1
    // for its RX decode, build the PIO1 gSPI transport for the cyw43 LAN, then
    // hand both to wireless::run_router (executor: cyw43 Runner + LAN net_task +
    // the WAN wan_task + USB). No GP25 LED — GP25 is the gSPI CS. See r15-plan §6.
    #[cfg(feature = "router")]
    {
        let _ = &timer; // TIMER0 stays un-reset; the time-driver reads it directly
        // Perf step 2: un-inhibit core-0 `mcycle` before the executor takes over,
        // so the forwarding fast-path can measure its own cycles (see `cycles`).
        cycles::enable_mcycle();
        let sys_clk_hz = clocks.system_clock.freq().to_Hz();

        // WAN: GP14/GP13 → PIO0; build EthMac + launch core 1 (RX decode). Pin
        // handles held so the PIO0 function assignment outlives the executor.
        let _tx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio14.into_function();
        let _rx_pin: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio13.into_function();
        let mut fifo = sio.fifo;
        let (mac, core1_ok) = setup_eth_mac(
            pac.PIO0, pac.DMA, &mut pac.PSM, &mut fifo, &mut pac.RESETS, sys_clk_hz,
        );

        // LAN: GP24/GP29 → PIO1, WL_ON = GP23, CS = GP25 (driven inside
        // PioSpiCyw43); bus held idle through WL_ON power-up (gotcha #11).
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

    // ── Wireless-only image (R14 LAN, `--features wireless` without `router`).
    // The embassy executor owns core 0 and never returns — a *standalone* build,
    // 10BASE-T not started (docs/router-plan.md §11/§12). Build the PIO1 gSPI
    // transport (CLK/CS/DATA held idle through WL_ON power-up, gotcha #11) + WL_ON,
    // then hand to wireless::run (cyw43 Runner + LAN net_task + USB).
    #[cfg(all(feature = "wireless", not(feature = "router")))]
    {
        let _ = &timer; // keep TIMER0 un-reset; the hal handle is unused here
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

/// Build the 10BASE-T data path: PIO0 Manchester TX (SM0) + carrier-detect
/// (SM2) + RX sampler (SM1) with the DMA double-buffer, install the RX engine,
/// and launch core 1 to own the `DMA_IRQ_0` decode. Returns the core-0 `EthMac`
/// (smoltcp `phy::Device`) + whether core 1 launched. Shared by `main_10bt`
/// (R12e production loop) and the router build's dispatch (R15b) so the subtle
/// RX-on-core-1 ordering lives in exactly one place.
///
/// The caller must have already routed GP13/GP14 to `FunctionPio0` (and must
/// keep those pin handles alive). `OUR_MAC` is installed for the IRQ-side MAC
/// filter. Call once (it claims the RX DMA buffers via `singleton!` + launches
/// core 1). Ordering matches R12c: `EthRx::new` (PIO+DMA running) → `install_rx`
/// (populate `RX_ENGINE`) → `launch_core1_riscv` (core 1 enables the IRQ).
fn setup_eth_mac(
    pio0: hal::pac::PIO0,
    dma: hal::pac::DMA,
    psm: &mut hal::pac::PSM,
    sio_fifo: &mut hal::sio::SioFifo,
    resets: &mut hal::pac::RESETS,
    sys_clk_hz: u32,
) -> (eth_mac::EthMac, bool) {
    // PIO0 SM0 = Manchester TX; SM1 = RX sampler; SM2 = Phase-3d carrier detector
    // (watches RO/GP13 for the TX carrier-sense gate). SM3 + PIO1 are free.
    let (mut pio0, sm0, sm1, sm2, _sm3) = pio0.split(resets);
    let eth_tx = eth_tx::EthTx::new(&mut pio0, sm0, sm2, HW_PIN_TXD, HW_PIN_RXD, sys_clk_hz);

    // DMA channels 0 and 1 ferry samples from the PIO RX FIFO into two 16 KB
    // half-buffers. EthRx::poll_with hands the just-filled half to the decoder
    // while DMA continues filling the other. Buffers static via singleton!.
    let dma = dma.split(resets);
    let rx_buf_a = singleton!(: [u32; eth_rx::BUF_WORDS] = [0; eth_rx::BUF_WORDS]).unwrap();
    let rx_buf_b = singleton!(: [u32; eth_rx::BUF_WORDS] = [0; eth_rx::BUF_WORDS]).unwrap();
    let rx_carry =
        singleton!(: [u8; eth_rx::MAX_CARRY_BYTES] = [0; eth_rx::MAX_CARRY_BYTES]).unwrap();
    let rx_stitch =
        singleton!(: [u8; eth_rx::STITCH_BUF_BYTES] = [0; eth_rx::STITCH_BUF_BYTES]).unwrap();
    let eth_rx = eth_rx::EthRx::new(
        &mut pio0, sm1, HW_PIN_RXD, sys_clk_hz, dma.ch0, dma.ch1, rx_buf_a, rx_buf_b, rx_carry,
        rx_stitch,
    );

    // Install EthRx + our MAC into the core-1-exclusive RX engine *before*
    // launching core 1 (which enables DMA_IRQ_0). Core 0 unmasks nothing — the
    // decode never runs here, so the loop/executor can't be starved (R12c).
    // Bounded launch → a dead core 1 returns an error rather than hanging core 0.
    let _ = eth_mac::install_rx(eth_rx, OUR_MAC);
    let core1_stack = unsafe { &mut (*core::ptr::addr_of_mut!(CORE1_STACK)).0 };
    let core1_launch_ok =
        unsafe { multicore_riscv::launch_core1_riscv(psm, sio_fifo, core1_stack, core1_entry).is_ok() };

    // EthMac owns just the TX side + scratch; RX state lives in the shared static.
    (eth_mac::EthMac::new(eth_tx), core1_launch_ok)
}

/// The 10BASE-T NIC (default build): PIO Manchester TX + PIO/DMA RX decoded on
/// core 1, the smoltcp endpoint stack, USB CDC + the picotool reset interface —
/// driven by a blocking poll loop on core 0 that never returns. Split out of
/// `main` so the `wireless` build can replace this data path wholesale (R14.1);
/// `main` does the shared clock/pin setup and hands the resources in.
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

    // GP14 → ISL3177E DI, GP13 → ISL3177E RO. Reassign both to PIO0 function;
    // the handles are held (`_`) so the assignment outlives the never-returning
    // loop. The PIO/DMA/RX-engine/core-1 bring-up is shared with the router
    // build via `setup_eth_mac`.
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
    // Default build: static IP (mirrors the C reference + the host test recipes
    // in RESUME.md). With `--features wan-dhcp` the address + default route are
    // left empty — the dhcpv4 client installs them once it gets a lease (see the
    // WAN socket wiring below).
    #[cfg(not(feature = "wan-dhcp"))]
    iface.update_ip_addrs(|addrs| {
        addrs
            .push(IpCidr::new(IpAddress::Ipv4(Ipv4Address::new(192, 168, 37, 24)), 24))
            .unwrap();
    });

    // Default: UDP echo + HTTP = 2 sockets (5 slots of headroom). The wan-dhcp
    // build adds 3 more (dhcpv4 + icmp + dns), so size for 8.
    #[cfg(all(not(feature = "wan-dhcp"), not(feature = "fd-bench")))]
    let mut sockets_storage: [SocketStorage; 5] = [SocketStorage::EMPTY; 5];
    // fd-bench adds the port-9999 upload sink → one more socket slot.
    #[cfg(all(not(feature = "wan-dhcp"), feature = "fd-bench"))]
    let mut sockets_storage: [SocketStorage; 6] = [SocketStorage::EMPTY; 6];
    #[cfg(feature = "wan-dhcp")]
    let mut sockets_storage: [SocketStorage; 8] = [SocketStorage::EMPTY; 8];
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
    //
    // With `--features http-bulk-test` the response becomes a 1 MB stream
    // for the experiment-4 methodology cross-check, so we bump the TX
    // buffer to keep the pipe full between poll cycles.
    let mut tcp_rx_storage = [0u8; 1024];
    #[cfg(not(feature = "http-bulk-test"))]
    let mut tcp_tx_storage = [0u8; 1024];
    // Phase 3e: a larger send window keeps enough segments in flight that the
    // residual CS-gap collisions (which carrier-sense + backoff can't fully
    // eliminate without true collision-detect) trigger TCP fast-retransmit
    // (~ms) instead of an RTO stall (~200 ms) — converting the *cost* of each
    // residual loss, which is what drives the throughput variance.
    #[cfg(feature = "http-bulk-test")]
    let mut tcp_tx_storage = [0u8; 32 * 1024];
    let tcp_rx_buffer = tcp::SocketBuffer::new(&mut tcp_rx_storage[..]);
    let tcp_tx_buffer = tcp::SocketBuffer::new(&mut tcp_tx_storage[..]);
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);
    let tcp_handle: SocketHandle = sockets.add(tcp_socket);

    // fd-bench (full-duplex Tier-2): TCP upload sink on port 9999 — the host
    // pushes bulk data INTO the device here while `http-bulk-test` streams OUT
    // on :80, exercising the 10BT link in both directions at once. A big RX
    // buffer keeps the receive window open so the host can fill the pipe.
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
        // RX-of-bulk pacing: disable the 10 ms delayed-ACK timer. With the
        // small advertised window (max_burst_size clamp in eth_mac.rs) the
        // timer dominated the per-segment cycle (~15 ms → ~96 KB/s), and
        // worse, its fixed 10 ms phase sat right on Linux's tail-loss-probe
        // timer (max(2·srtt, 10 ms)) — host probe retransmits and our delayed
        // ACKs repeatedly transmitted into each other on the half-duplex
        // wire, showing up as a 27–33% FCS-fail floor that vanished (→3–5%
        // at a 1-seg window) the moment ACKs went immediate. An immediate
        // ACK instead rides the quiet inter-frame gap right after the data
        // frame it acknowledges. Coalescing (ack_delay = 1 ms ≈ ACK every
        // 2nd segment) was also tried and is strictly worse (~54–78 KB/s,
        // 43% fail): a timer-fired ACK lands mid-stream of the next inbound
        // segment instead of in the post-frame gap.
        sink.set_ack_delay(None);
        sink
    });

    // R15a — WAN-as-DHCP-client sockets. Buffers live in this never-returning
    // fn's stack (same pattern as the udp/tcp buffers above), so the borrows the
    // SocketSet holds are valid for the life of the loop.
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

    // DNS query slots (no_std: a borrowed fixed array, not a Vec). Servers are
    // empty until the dhcpv4 client hands us the lease's DNS option.
    #[cfg(feature = "wan-dhcp")]
    let mut dns_queries: [Option<dns::DnsQuery>; 2] = [None, None];
    #[cfg(feature = "wan-dhcp")]
    let dns_handle: SocketHandle = sockets.add(dns::Socket::new(&[], &mut dns_queries[..]));

    #[cfg(feature = "wan-dhcp")]
    let mut wan = crate::wan::WanState::new();
    // The device's TX checksum capabilities — used to emit/parse ICMP echoes.
    #[cfg(feature = "wan-dhcp")]
    let wan_checksum: ChecksumCapabilities = mac.capabilities().checksum;

    // Mirror the C reference's network parameters so the host's existing
    // ethtool / IP route setup keeps working.
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
    // R15a — once per second: emit one ICMP echo to 8.8.8.8 + (re)start a DNS
    // query when idle. Reply draining + result polling happen every iteration.
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

    // fd-bench: per-second deltas for the [Sink] upload-RX-rate line. (The
    // decode-ceiling signal under bidir load is the always-printed [Rx] fail/dec
    // line; the mcycle cpu1 counters are router-gated and not worth un-gating.)
    #[cfg(feature = "fd-bench")]
    let mut fd_prev_sink: u32 = 0;
    #[cfg(feature = "fd-bench")]
    let mut fd_last_us: u64 = now0;
    // fd-bench: main-loop iteration counter — discriminates a loop/max_burst rate
    // cap (iters/s ≈ frames/s) from a TCP-level cap (loop fast, frames/s low).
    #[cfg(feature = "fd-bench")]
    let mut fd_loop_iters: u32 = 0;

    // Arm the RX-hang watchdog (see WDT_TIMEOUT_US); fed at the top of every loop
    // iteration below. If the poll loop wedges, the chip reboots and recovers.
    watchdog.start(hal::fugit::MicrosDurationU32::micros(WDT_TIMEOUT_US));

    loop {
        watchdog.feed();
        #[cfg(feature = "fd-bench")]
        {
            fd_loop_iters = fd_loop_iters.wrapping_add(1);
        }
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

        // R15a — WAN client work that runs every iteration (after iface.poll has
        // delivered inbound datagrams): apply any DHCP lease change to the
        // interface, drain ICMP echo replies, and harvest a finished DNS query.
        #[cfg(feature = "wan-dhcp")]
        {
            crate::wan::dhcp_apply(&mut iface, &mut sockets, dhcp_handle, dns_handle, &mut wan);
            crate::wan::ping_drain(&mut sockets, icmp_handle, &mut wan, &wan_checksum);
            crate::wan::dns_harvest(&mut sockets, dns_handle, &mut wan);
        }

        // R4.6: UDP echo on port 1234. R7: tiny HTTP server on port 80.
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

        // R15a — once per second: send one ICMP echo to 8.8.8.8 and, if no DNS
        // query is in flight and we know a server, kick off a name lookup. Only
        // fires once a lease has installed an address (else there's no source IP).
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
            // fd-bench: [Sink] upload RX rate, normalised to the measured window
            // (the loop cadence can slip under load — same lesson as the cyw43
            // [Lan] line). Pair with the [Rx] fail/dec line for the decode signal.
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

/// Emit the once-per-second `[Wan]` status line over CDC (R15a): lease IP /
/// gateway / DNS, the ICMP ping tally, and the last resolved A record. The body
/// is formatted by [`crate::wan::WanState::write_status`] (shared with the
/// router build's `usb_task`).
#[cfg(feature = "wan-dhcp")]
fn log_wan<B: UsbBus>(serial: &mut SerialPort<'_, B>, line: &mut String<160>, wan: &crate::wan::WanState) {
    line.clear();
    let _ = write!(line, "[Wan] ");
    wan.write_status(line);
    let _ = writeln!(line);
    let _ = serial.write(line.as_bytes());
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

/// Experiment 4 — bulk HTTP variant. Streams a fixed 1 MB body so a host
/// `curl` measures sustained TCP throughput across our RX path. State
/// machine is driven across many smoltcp poll cycles because the 8 KB TX
/// buffer would otherwise saturate; we keep the pipe full by refilling
/// whenever `socket.can_send` reports headroom. The dummy body is the
/// 0x55 toggling pattern matching `/tmp/blast_udp_full_mtu.py` so any
/// future per-byte-error scan would tie back to the same payload basis.
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
    // Refill chunk: 1 KB of 0x55. Matches the UDP-blast payload pattern.
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

/// fd-bench (full-duplex Tier-2): cumulative bytes sunk on the port-9999 upload
/// socket. Read once per second by the `[Sink]` heartbeat to derive an RX rate.
#[cfg(feature = "fd-bench")]
static FD_SINK_RX: AtomicU32 = AtomicU32::new(0);

/// fd-bench: drain the TCP upload sink on port 9999 — discard + count the
/// inbound bytes (no echo), and close on the peer's FIN so we re-listen for the
/// next connection. The host pushes bulk here (`nc`/`dd`) concurrently with the
/// :80 download, driving the 10BT link in both directions at once.
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
    // Peer finished sending (FIN) but our half is still open → close to re-listen.
    if !socket.may_recv() && socket.may_send() {
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
    core1_launch_ok: bool,
) {
    line.clear();
    let _ = writeln!(line, "[R2b] t={} nlps={} udp_sent={}", log_tick, nlps_sent, udp_sent);
    let _ = serial.write(line.as_bytes());

    // Phase 3a — core-1 liveness. `launch=ok` + a climbing `ticks` proves the
    // 2nd Hazard3 core came up and shares SRAM coherently with core 0.
    line.clear();
    let _ = writeln!(
        line,
        "[Core1] launch={} ticks={}",
        if core1_launch_ok { "ok" } else { "FAIL" },
        CORE1_TICKS.load(Ordering::Relaxed),
    );
    let _ = serial.write(line.as_bytes());

    // R13 Step 3a — cyw43 bring-up result (wireless build only). new_done=1 means
    // cyw43::new() completed: the 231 KB firmware + nvram downloaded over our PIO1
    // gSPI transport and the bus handshake passed under the real driver.
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
        "[Rx] dec={} ok={} fail={} filt={} dst={}",
        rx.frames_decoded, rx.fcs_ok, rx.fcs_fail, rx.frames_filtered,
        mac_str(last_dst_mac)
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

/// Format a MAC as `aa:bb:cc:dd:ee:ff` (lowercase hex) into a small stack
/// string, so it drops into a `write!` like any `Display` value. Shared by the
/// `[Rx]` log lines below and the wireless mgmt page
/// (`wireless::serve_status_http`).
pub fn mac_str(mac: [u8; 6]) -> String<17> {
    let mut s = String::new();
    let _ = write!(
        s,
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    s
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
