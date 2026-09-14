//! cyw43 (Pico 2 W) LAN on `rp235x-hal`, plus the router's executor tasks.
//!
//! Shims to the embassy/cyw43 world:
//! 1. **Time driver** — `embassy-time-driver` on TIMER0 (µs counter, ALARM0 IRQ).
//! 2. **Executor** — `embassy-executor` on `platform-riscv32`, run from main.
//! 3. **gSPI transport** — [`PioSpiCyw43`] on PIO1, instead of `cyw43-pio`.
//!
//! Design: `docs/router-plan.md` §4/§5.

use core::cell::RefCell;
use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex;
use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Timer};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use heapless::String;
use rp235x_hal as hal;
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::socket::{tcp, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address, Ipv4Cidr};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use crate::cyw43_phy::Cyw43Phy;
use crate::dhcp_server::DhcpServer;
use hal::pac::PIO1;
use hal::pio::{
    PIOBuilder, PinDir, PinState, Running, Rx, ShiftDirection, StateMachine, Stopped, Tx,
    UninitStateMachine, SM0,
};

/// WL_ON (GP23) push-pull output, concrete so the Runner task can name it.
pub type WlOnPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio23, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;

// =====================================================================
// 0. Bit-bang gSPI probe (unused bring-up code)
// =====================================================================
//
// Reads the bus test register (0x14), expecting 0xFEEDBEAD.
// Pico 2 W CYW43 pins: GP23 = WL_ON, GP24 = DATA, GP25 = CS, GP29 = CLK.

/// Last probe read; 0xFEEDBEAD means the bus is alive. 0 = not run.
pub static CYW43_PROBE: AtomicU32 = AtomicU32::new(0);
/// First probe read, and whether all reads matched (stable = timing, varying = floating).
pub static CYW43_PROBE_FIRST: AtomicU32 = AtomicU32::new(0);
pub static CYW43_PROBE_STABLE: AtomicU32 = AtomicU32::new(0);

const PIN_PWR: u32 = 23;
const PIN_DATA: u32 = 24;
const PIN_CS: u32 = 25;
const PIN_CLK: u32 = 29;
/// Bit-bang half-clock spin (~2 MHz gSPI at 240 MHz).
#[allow(dead_code)] // unused bring-up probe
const PHASE: u32 = 60;

#[inline]
fn cmd_word(write: bool, incr: bool, func: u32, addr: u32, len: u32) -> u32 {
    (write as u32) << 31 | (incr as u32) << 30 | (func & 0b11) << 28 | (addr & 0x1_FFFF) << 11
        | (len & 0x7FF)
}
#[inline]
fn swap16(x: u32) -> u32 {
    x.rotate_left(16)
}

// Raw SIO GPIO access, without claiming typed HAL pins.
#[inline]
fn gpio_set(n: u32) {
    unsafe { (*hal::pac::SIO::ptr()).gpio_out_set().write(|w| w.bits(1 << n)) };
}
#[inline]
fn gpio_clr(n: u32) {
    unsafe { (*hal::pac::SIO::ptr()).gpio_out_clr().write(|w| w.bits(1 << n)) };
}
#[inline]
fn gpio_oe(n: u32, output: bool) {
    let sio = unsafe { &*hal::pac::SIO::ptr() };
    if output {
        sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << n) });
    } else {
        sio.gpio_oe_clr().write(|w| unsafe { w.bits(1 << n) });
    }
}
#[inline]
#[allow(dead_code)] // bit-bang probe only
fn gpio_read(n: u32) -> u32 {
    (unsafe { (*hal::pac::SIO::ptr()).gpio_in().read().bits() } >> n) & 1
}

/// Route a pin to SIO and clear pad isolation (RP2350 pads boot isolated).
fn gpio_to_sio(n: u32) {
    let io = unsafe { &*hal::pac::IO_BANK0::ptr() };
    let pads = unsafe { &*hal::pac::PADS_BANK0::ptr() };
    pads.gpio(n as usize).modify(|_, w| {
        w.ie().set_bit();
        w.od().clear_bit();
        w.iso().clear_bit()
    });
    io.gpio(n as usize)
        .gpio_ctrl()
        .write(|w| unsafe { w.funcsel().bits(5) }); // 5 = SIO
}

/// Bit-banged gSPI command + 32-bit read. CS low throughout.
#[allow(dead_code)] // unused bring-up probe
fn bitbang_cmd_read(cmd: u32) -> u32 {
    gpio_clr(PIN_CS); // CS low — start transaction
    hal::arch::delay(PHASE);

    // --- write 32 cmd bits, MSB first ---
    gpio_oe(PIN_DATA, true);
    for i in (0..32).rev() {
        if (cmd >> i) & 1 != 0 {
            gpio_set(PIN_DATA);
        } else {
            gpio_clr(PIN_DATA);
        }
        hal::arch::delay(PHASE);
        gpio_set(PIN_CLK); // rising edge latches the bit
        hal::arch::delay(PHASE);
        gpio_clr(PIN_CLK);
    }

    // --- turnaround: DATA becomes an input ---
    gpio_oe(PIN_DATA, false);
    hal::arch::delay(PHASE);

    // --- read 32 bits, MSB first (sample while CLK high) ---
    let mut r: u32 = 0;
    for _ in 0..32 {
        gpio_set(PIN_CLK);
        hal::arch::delay(PHASE);
        r = (r << 1) | gpio_read(PIN_DATA);
        gpio_clr(PIN_CLK);
        hal::arch::delay(PHASE);
    }

    gpio_set(PIN_CS); // CS high — end transaction
    r
}

/// Power the CYW43 and read TEST_RO into [`CYW43_PROBE`]. Blocks ~270 ms.
#[allow(dead_code)] // unused bring-up probe
pub fn probe_cyw43() {
    // Pins: CLK/CS/PWR as outputs (CS + CLK idle high/low), DATA starts output.
    for &n in &[PIN_CLK, PIN_CS, PIN_PWR, PIN_DATA] {
        gpio_to_sio(n);
        gpio_oe(n, true);
    }
    gpio_clr(PIN_CLK);
    gpio_set(PIN_CS); // CS idle high
    gpio_clr(PIN_DATA);

    // Power-cycle WL_ON: low 20 ms, high, settle 250 ms (matches cyw43 init).
    let ms = |n: u32| hal::arch::delay(n.saturating_mul(240_000)); // ~ms @ 240 MHz
    gpio_clr(PIN_PWR);
    ms(20);
    gpio_set(PIN_PWR);
    ms(250);

    // TEST_RO read. Initial gSPI mode is 16-bit swapped; retry until it settles.
    let cmd = swap16(cmd_word(false /*read*/, true /*incr*/, 0, 0x14, 4));
    let mut last = 0u32;
    let mut first = 0u32;
    let mut stable = true;
    for i in 0..64 {
        let v = swap16(bitbang_cmd_read(cmd));
        if i == 0 {
            first = v;
        } else if v != last {
            stable = false; // some read differed from the previous → DATA varying
        }
        last = v;
        if v == 0xFEED_BEAD {
            break;
        }
        hal::arch::delay(24_000); // ~100 µs between attempts
    }
    // 0xDEAD0000 = ran but read all-zero; otherwise the last value (FEEDBEAD = win).
    CYW43_PROBE.store(if last == 0 { 0xDEAD_0000 } else { last }, Ordering::Relaxed);
    CYW43_PROBE_FIRST.store(first, Ordering::Relaxed);
    CYW43_PROBE_STABLE.store(stable as u32, Ordering::Relaxed);
}

// =====================================================================
// 0b. PIO1 gSPI state machine, plus an unused probe
// =====================================================================
//
// Each transaction pushes [write_bits-1, read_bits-1, words...]. The SM clocks
// writes out MSB-first (latched on CLK rise), turns DATA around, then samples
// reads while CLK is high. Timing matches embassy's cyw43-pio program.
// CLK = GP29 (side-set), DATA = GP24; CS and WL_ON are SIO.

/// Build the PIO1 gSPI SM with the bus idle (CLK, DATA low); returned stopped.
/// Doesn't touch CS or WL_ON.
fn build_gspi_sm(
    pio: &mut hal::pio::PIO<PIO1>,
    sm: UninitStateMachine<(PIO1, SM0)>,
    sys_clk_hz: u32,
) -> (
    StateMachine<(PIO1, SM0), Stopped>,
    Tx<(PIO1, SM0)>,
    Rx<(PIO1, SM0)>,
) {
    // Program format: see the section comment (docs/router-plan.md §11 #1).
    let program = pio::pio_asm!(
        ".side_set 1",
        ".wrap_target",
        "    out x, 32      side 0", // X = write_bits-1  (autopull from FIFO)
        "    out y, 32      side 0", // Y = read_bits-1
        "    set pindirs, 1 side 0", // DATA = output
        "wloop:",
        "    out pins, 1    side 0", // drive DATA bit (MSB first), CLK low
        "    jmp x-- wloop  side 1", // CLK high → chip latches on rising edge
        "    set pindirs, 0 side 0", // turnaround: DATA = input, CLK low
        "    nop            side 0", // CLK stays LOW through turnaround
        "rloop:",
        "    in pins, 1     side 1", // CLK high → sample chip's DATA
        "    jmp y-- rloop  side 0", // CLK low; loop
        ".wrap",
    );
    let installed = pio.install(&program.program).unwrap();

    // 30 MHz PIO = 15 MHz gSPI (2 cycles per bit). embassy runs ~33 MHz.
    const GSPI_PIO_HZ: f32 = 30_000_000.0;
    let (div_int, div_frac) = crate::pio_util::clock_divider(sys_clk_hz, GSPI_PIO_HZ);
    let (mut sm, rx, tx) = PIOBuilder::from_installed_program(installed)
        .out_pins(PIN_DATA as u8, 1)
        .set_pins(PIN_DATA as u8, 1)
        .in_pin_base(PIN_DATA as u8)
        .side_set_pin_base(PIN_CLK as u8)
        .out_shift_direction(ShiftDirection::Left) // MSB first
        .in_shift_direction(ShiftDirection::Left) // MSB first
        .autopull(true)
        .pull_threshold(32)
        .autopush(true)
        .push_threshold(32)
        .clock_divisor_fixed_point(div_int, div_frac)
        .build(sm);
    // Bus idle: CLK and DATA driven low until start().
    sm.set_pindirs([
        (PIN_CLK as u8, PinDir::Output),
        (PIN_DATA as u8, PinDir::Output),
    ]);
    sm.set_pins([
        (PIN_CLK as u8, PinState::Low),
        (PIN_DATA as u8, PinState::Low),
    ]);
    (sm, tx, rx)
}

/// PIO1 probe: power the CYW43 and read TEST_RO into [`CYW43_PROBE`].
/// 0xDEAD_0001 = SM stalled; 0xDEAD_0000 = read zeros.
#[allow(dead_code)] // unused bring-up probe
pub fn probe_cyw43_pio(
    pio: &mut hal::pio::PIO<PIO1>,
    sm: UninitStateMachine<(PIO1, SM0)>,
    sys_clk_hz: u32,
) {
    // WL_ON (GP23) + CS (GP25) as SIO; DATA/CLK are PIO (caller-routed).
    for &n in &[PIN_PWR, PIN_CS] {
        gpio_to_sio(n);
        gpio_oe(n, true);
    }
    gpio_set(PIN_CS); // CS idle high
    gpio_clr(PIN_PWR); // WL_ON low (chip off) while we configure the bus pins

    let cyc_per_ms = (sys_clk_hz / 1000).max(1);
    let ms = |n: u32| hal::arch::delay(n.saturating_mul(cyc_per_ms));

    // Bus idles low through power-up; start the SM after.
    let (sm, mut tx, mut rx) = build_gspi_sm(pio, sm, sys_clk_hz);

    // Power-cycle WL_ON only after the bus is idle. A floating bus during
    // power-up can latch the wrong gSPI mode.
    ms(20); // WL_ON held low ≥20 ms (chip off)
    gpio_set(PIN_PWR); // WL_ON high
    ms(250); // settle while the chip boots its gSPI

    let _sm = sm.start(); // keep alive (drop would stop the SM)

    // TEST_RO read; initial gSPI mode is 16-bit swapped.
    let cmd = swap16(cmd_word(false /*read*/, true /*incr*/, 0, 0x14, 4));
    let mut last = 0u32;
    let mut first = 0u32;
    let mut stable = true;
    for i in 0..64 {
        let v = swap16(pio_cmd_read32(&mut tx, &mut rx, cmd));
        if i == 0 {
            first = v;
        } else if v != last {
            stable = false; // some read differed → DATA varying
        }
        last = v;
        if v == 0xFEED_BEAD {
            break;
        }
        ms(1); // bus can take a few reads to settle after power-up
    }
    CYW43_PROBE.store(if last == 0 { 0xDEAD_0000 } else { last }, Ordering::Relaxed);
    CYW43_PROBE_FIRST.store(first, Ordering::Relaxed);
    CYW43_PROBE_STABLE.store(stable as u32, Ordering::Relaxed);
}

/// gSPI cmd + read32 over PIO1: 32 bits out, 64 in (data + status).
/// Returns the data word, or 0xDEAD_0001 if the SM stalled.
#[allow(dead_code)] // unused bring-up probe
fn pio_cmd_read32(
    tx: &mut Tx<(PIO1, SM0)>,
    rx: &mut Rx<(PIO1, SM0)>,
    cmd: u32,
) -> u32 {
    while rx.read().is_some() {} // drain any stale words
    gpio_clr(PIN_CS); // CS low — start transaction
    hal::arch::delay(60);
    while !tx.write(31) {} // X = write_bits-1 → clock 32 cmd bits out
    while !tx.write(63) {} // Y = read_bits-1  → clock 64 bits in (data + status)
    while !tx.write(cmd) {} // 32-bit command word
    // Response: two autopushed words — data, then the gSPI status word. Take data.
    let mut got = 0u32;
    let mut data = 0xDEAD_0001u32; // default: SM produced nothing
    let mut spins = 0u32;
    while got < 2 {
        if let Some(w) = rx.read() {
            if got == 0 {
                data = w;
            }
            got += 1;
        } else {
            spins += 1;
            if spins > 2_000_000 {
                break;
            }
        }
    }
    hal::arch::delay(60);
    gpio_set(PIN_CS); // CS high — end transaction
    data
}

// =====================================================================
// 0c. Pin self-test (unused bring-up code)
// =====================================================================
//
// Drives each CYW43 pin low then high as SIO and reads it back.
// Bits land at each pin's index.

/// Per-pin GPIO_IN readback when the pin was driven LOW (want 0 at bits 23/24/25/29).
#[allow(dead_code)] // unused bring-up code
pub static CYW43_PIN_LO: AtomicU32 = AtomicU32::new(0);
/// Per-pin GPIO_IN readback when the pin was driven HIGH (want 1 at bits 23/24/25/29).
#[allow(dead_code)] // unused bring-up code
pub static CYW43_PIN_HI: AtomicU32 = AtomicU32::new(0);

/// Drive each CYW43 pin low then high and record readbacks. Leaves WL_ON high.
#[allow(dead_code)] // unused bring-up code
pub fn pin_selftest() {
    let mut lo = 0u32;
    let mut hi = 0u32;
    for &pin in &[PIN_PWR, PIN_CS, PIN_CLK, PIN_DATA] {
        gpio_to_sio(pin);
        gpio_oe(pin, true);
        gpio_clr(pin);
        hal::arch::delay(200); // let the input synchronizer settle
        lo |= gpio_read(pin) << pin;
        gpio_set(pin);
        hal::arch::delay(200);
        hi |= gpio_read(pin) << pin;
    }
    CYW43_PIN_LO.store(lo, Ordering::Relaxed);
    CYW43_PIN_HI.store(hi, Ordering::Relaxed);
}

// =====================================================================
// 1. embassy-time driver on RP2350 TIMER0
// =====================================================================
//
// A 1 MHz tick matches TIMER0's µs counter. ALARM0 drives wakeups.

/// Time-driver alarm IRQ.
#[allow(dead_code)]
const ALARM_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::TIMER0_IRQ_0;

struct RpTimeDriver {
    /// The 16-slot generic timer queue (waker storage). Guarded by a
    /// `critical_section` mutex so the IRQ and task contexts can't race.
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(
    static TIME_DRIVER: RpTimeDriver = RpTimeDriver {
        queue: Mutex::new(RefCell::new(Queue::new())),
    }
);

impl RpTimeDriver {
    /// Read TIMER0's free-running 64-bit µs counter (re-read the high word to
    /// guard against a low-word rollover between the two 32-bit reads).
    #[inline]
    fn now_us() -> u64 {
        let t = unsafe { &*hal::pac::TIMER0::ptr() };
        loop {
            let hi = t.timerawh().read().bits();
            let lo = t.timerawl().read().bits();
            let hi2 = t.timerawh().read().bits();
            if hi == hi2 {
                return ((hi as u64) << 32) | (lo as u64);
            }
        }
    }

    /// Arm ALARM0 for `at` (µs), or mask it for `u64::MAX`. ALARM0 is 32-bit,
    /// so far deadlines chain through an intermediate fire.
    fn arm_alarm(&self, at: u64) {
        let t = unsafe { &*hal::pac::TIMER0::ptr() };
        if at == u64::MAX {
            // No timers pending — disable the alarm interrupt.
            t.inte().modify(|_, w| w.alarm_0().clear_bit());
            return;
        }
        let now = Self::now_us();
        let target = if at <= now {
            now.wrapping_add(2) // already due — fire ASAP
        } else if at - now > u32::MAX as u64 {
            now.wrapping_add(u32::MAX as u64) // too far — chain via an intermediate fire
        } else {
            at
        };
        t.inte().modify(|_, w| w.alarm_0().set_bit());
        // Writing ALARM0 arms it against the low 32 bits of the counter.
        t.alarm0().write(|w| unsafe { w.bits(target as u32) });
    }
}

impl Driver for RpTimeDriver {
    fn now(&self) -> u64 {
        Self::now_us()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow_ref_mut(cs);
            // `schedule_wake` returns true when the earliest deadline changed,
            // i.e. we need to (re)arm the hardware alarm.
            if queue.schedule_wake(at, waker) {
                let next = queue.next_expiration(Self::now_us());
                self.arm_alarm(next);
            }
        });
    }
}

/// ALARM0 fired: clear the latched IRQ, wake any expired timers, re-arm for the
/// next one. Runs on whichever core enabled `TIMER0_IRQ_0` (the executor core).
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn TIMER0_IRQ_0() {
    let t = unsafe { &*hal::pac::TIMER0::ptr() };
    // Clear the ALARM0 latched interrupt (write-1-to-clear in INTR).
    t.intr().write(|w| w.alarm_0().clear_bit_by_one());
    critical_section::with(|cs| {
        let mut queue = TIME_DRIVER.queue.borrow_ref_mut(cs);
        let next = queue.next_expiration(RpTimeDriver::now_us());
        TIME_DRIVER.arm_alarm(next);
    });
}

// =====================================================================
// 2. gSPI transport for cyw43 (`SpiBusCyw43` on PIO1)
// =====================================================================
//
// Synchronous busy-poll over the PIO1 FIFOs. Bit counts match cyw43-pio 0.7.0.

/// PIO1 gSPI transport for the CYW43439. CS (GP25) is SIO.
/// Build before `cyw43::new` so the bus idles through power-up.
#[allow(dead_code)]
pub struct PioSpiCyw43 {
    _sm: StateMachine<(PIO1, SM0), Running>,
    tx: Tx<(PIO1, SM0)>,
    rx: Rx<(PIO1, SM0)>,
}

#[allow(dead_code)]
impl PioSpiCyw43 {
    /// CS idle high, gSPI SM started with the bus idle. Doesn't power WL_ON;
    /// `cyw43::new` does that later.
    pub fn new(
        pio: &mut hal::pio::PIO<PIO1>,
        sm: UninitStateMachine<(PIO1, SM0)>,
        sys_clk_hz: u32,
    ) -> Self {
        gpio_to_sio(PIN_CS);
        gpio_oe(PIN_CS, true);
        gpio_set(PIN_CS); // CS idle high
        let (sm, tx, rx) = build_gspi_sm(pio, sm, sys_clk_hz);
        Self {
            _sm: sm.start(),
            tx,
            rx,
        }
    }

    /// Drain any stale RX words (defensive — a normal txn leaves RX empty).
    fn drain_rx(&mut self) {
        while self.rx.read().is_some() {}
    }

    /// Push one word, bounded so a stalled SM can't hang the executor.
    fn push(&mut self, w: u32) {
        let mut spins = 0u32;
        while !self.tx.write(w) {
            spins += 1;
            if spins > 8_000_000 {
                break;
            }
        }
    }

    /// Pull one word from the RX FIFO (bounded busy-wait; 0 on timeout).
    fn pull(&mut self) -> u32 {
        let mut spins = 0u32;
        loop {
            if let Some(w) = self.rx.read() {
                return w;
            }
            spins += 1;
            if spins > 8_000_000 {
                return 0;
            }
        }
    }
}

impl cyw43::SpiBusCyw43 for PioSpiCyw43 {
    /// Clock `write` out MSB-first, then read back the gSPI status word.
    /// X = write.len()*32 - 1 (write bits); Y = 31 (read one status word).
    async fn cmd_write(&mut self, write: &[u32]) -> u32 {
        // Count gSPI cycles toward `spi0` (router).
        #[cfg(feature = "router")]
        let _span = crate::cycles::CycleSpan::new(&crate::cycles::CYW43_SPI_BUSY);
        self.drain_rx();
        gpio_clr(PIN_CS); // CS low
        let wbits = (write.len() as u32).saturating_mul(32).saturating_sub(1);
        self.push(wbits);
        self.push(31); // read 32 bits = one status word
        for &w in write {
            self.push(w);
        }
        let status = self.pull();
        gpio_set(PIN_CS); // CS high
        status
    }

    /// Send the 32-bit cmd, then read `read.len()` words plus status.
    /// X = 31; Y = (read.len()+1)*32 - 1. Matches cyw43-pio.
    async fn cmd_read(&mut self, write: u32, read: &mut [u32]) -> u32 {
        #[cfg(feature = "router")]
        let _span = crate::cycles::CycleSpan::new(&crate::cycles::CYW43_SPI_BUSY);
        self.drain_rx();
        gpio_clr(PIN_CS); // CS low
        self.push(31); // write 32 cmd bits
        let rbits = (read.len() as u32)
            .saturating_add(1)
            .saturating_mul(32)
            .saturating_sub(1);
        self.push(rbits);
        self.push(write);
        for slot in read.iter_mut() {
            *slot = self.pull();
        }
        let status = self.pull();
        gpio_set(PIN_CS); // CS high
        status
    }

    // `wait_for_event` uses default active polling; host-wake IRQ isn't wired.
}

// =====================================================================
// 2b. cyw43 stage flags and LAN config
// =====================================================================

/// 1 once `cyw43::new()` returned (firmware + nvram loaded + bus handshake OK).
pub static CYW43_NEW_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once `Control::init(clm)` returned (CLM loaded + WiFi firmware up).
pub static CYW43_INIT_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once the onboard LED has toggled; stays set while the Runner lives.
pub static CYW43_LED_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once the AP is up (`start_ap_wpa2` returned).
pub static CYW43_AP_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once the LAN smoltcp `Interface` is polling.
pub static CYW43_NET_UP: AtomicU32 = AtomicU32::new(0);

/// LAN gateway IP.
const LAN_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
const LAN_PREFIX: u8 = 24;

// =====================================================================
// LAN-isolation perf: traffic terminated on the Pico
// =====================================================================

/// `/bulk` download size (Pico → client, pure cyw43 TX).
const LAN_BULK_BYTES: usize = 8 * 1024 * 1024;

/// Upload sink port (client → Pico, pure cyw43 RX). Drains and counts.
const LAN_SINK_PORT: u16 = 9999;

/// Bytes sent by `/bulk`; `[Lan] tx=` is the per-second delta.
pub static LAN_BULK_TX_BYTES: AtomicU32 = AtomicU32::new(0);
/// Bytes drained at the sink; `[Lan] rx=` is the per-second delta.
pub static LAN_SINK_RX_BYTES: AtomicU32 = AtomicU32::new(0);

/// LAN HTTP state, so `/bulk` can stream across many polls.
enum LanHttp {
    /// Listening / the request hasn't been routed yet.
    Idle,
    /// `/bulk` in flight. The header waits for a poll where the socket can send.
    Bulk { remaining: usize, header_sent: bool },
}

// AP settings, compiled into the firmware.
// ⚠️ CHANGE before deploying: the passphrase is a placeholder.
// WPA2 passphrase: 8..=63 bytes. Channel is 2.4 GHz.
const AP_SSID: &str = "pico-10bt-router";
const AP_PASSPHRASE: &str = "change-me-please";
const AP_CHANNEL: u8 = 6;

/// Blocking cyw43 bring-up without an executor (unused; `run` replaced it).
/// Runs `cyw43::new`, then `Control::init` and LED blinks beside the Runner
/// via `select`, then returns.
pub fn cyw43_bringup_blocking<PWR: embedded_hal::digital::OutputPin>(pwr: PWR, spi: PioSpiCyw43) {
    let fw = cyw43::aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let nvram = cyw43::aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");
    let clm: &[u8] = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // cyw43::State is large (driver + channel buffers) — keep it in a static.
    static mut STATE: cyw43::State = cyw43::State::new();
    let state = unsafe { &mut *core::ptr::addr_of_mut!(STATE) };

    embassy_futures::block_on(async move {
        let (_net, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
        CYW43_NEW_DONE.store(1, Ordering::Relaxed);

        let seq = async {
            control.init(clm).await;
            CYW43_INIT_DONE.store(1, Ordering::Relaxed);
            // Blink the onboard LED through the running Runner.
            for _ in 0..6 {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(150)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(150)).await;
            }
            CYW43_LED_DONE.store(1, Ordering::Relaxed);
        };
        // Drive the Runner (cyw43 event loop) until `seq` completes, then return.
        embassy_futures::select::select(runner.run(), seq).await;
    });
}

// =====================================================================
// 3. cyw43 handle types and USB
// =====================================================================

// Concrete cyw43 handle types over our PIO1 transport — needed to name the
// long-lived `Runner` task's argument (embassy `#[task]`s can't be generic).
type CywBus = cyw43::SpiBus<WlOnPin, PioSpiCyw43>;
type CywRunner = cyw43::Runner<'static, CywBus>;

/// Build USB (CDC + picotool reset) on a `'static` allocator for `usb_task`.
/// Same VID:PID and serial scheme as `main.rs`. Call once.
#[allow(clippy::type_complexity)] // a 3-tuple of usb-device handles reads fine here
fn build_usb(
    usb: hal::pac::USB,
    usb_dpram: hal::pac::USB_DPRAM,
    usb_clock: hal::clocks::UsbClock,
    resets: &mut hal::pac::RESETS,
) -> (
    UsbDevice<'static, hal::usb::UsbBus>,
    SerialPort<'static, hal::usb::UsbBus>,
    crate::pico_reset::PicoResetInterface,
) {
    static mut USB_BUS: core::mem::MaybeUninit<UsbBusAllocator<hal::usb::UsbBus>> =
        core::mem::MaybeUninit::uninit();
    let usb_bus: &'static UsbBusAllocator<hal::usb::UsbBus> = unsafe {
        let p = core::ptr::addr_of_mut!(USB_BUS);
        (*p).write(UsbBusAllocator::new(hal::usb::UsbBus::new(
            usb, usb_dpram, usb_clock, true, resets,
        )));
        &*(*p).as_ptr()
    };

    let serial = SerialPort::new(usb_bus);
    let reset_iface = crate::pico_reset::PicoResetInterface::new(usb_bus);

    // Serial = chip ID so picotool tracks the BOOTSEL reboot. Must be `'static`.
    static mut SERIAL_STR: core::mem::MaybeUninit<String<16>> =
        core::mem::MaybeUninit::uninit();
    let serial_str: &'static str = unsafe {
        let s = (*core::ptr::addr_of_mut!(SERIAL_STR)).write(String::new());
        match hal::rom_data::sys_info_api::chip_info() {
            Ok(Some(info)) => {
                let _ = write!(s, "{:08X}{:08X}", info.wafer_id, info.device_id);
            }
            _ => {
                let _ = write!(s, "0000000000000000");
            }
        }
        s.as_str()
    };

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x2e8a, 0x000a))
        .strings(&[StringDescriptors::default()
            .manufacturer("pico-10base-t-rs")
            .product("Pico-10BASE-T (Rust) — wireless")
            .serial_number(serial_str)])
        .unwrap()
        .max_packet_size_0(64)
        .unwrap()
        .device_class(2) // USB CDC
        .build();

    (usb_dev, serial, reset_iface)
}

// =====================================================================
// 3b. Async runtime entry
// =====================================================================

/// Write all of `bytes` to CDC, polling USB between chunks.
/// `serial.write` alone drops the tail once the ~128 B buffer fills.
/// Bounded so a host that isn't reading can't hang us.
fn cdc_write_all(
    usb_dev: &mut UsbDevice<'static, hal::usb::UsbBus>,
    serial: &mut SerialPort<'static, hal::usb::UsbBus>,
    reset_iface: &mut crate::pico_reset::PicoResetInterface,
    bytes: &[u8],
) {
    let mut rest = bytes;
    let mut guard = 0u32;
    while !rest.is_empty() && guard < 64 {
        match serial.write(rest) {
            Ok(n) => rest = &rest[n..],
            Err(usb_device::UsbError::WouldBlock) => {}
            Err(_) => break,
        }
        usb_dev.poll(&mut [&mut *serial, &mut *reset_iface]); // flush the IN buffer
        guard += 1;
    }
}

/// Feed the watchdog every [`crate::WDT_FEED_MS`]. If the executor stalls,
/// feeding stops and the chip reboots.
#[embassy_executor::task]
async fn watchdog_feed_task(wd: hal::Watchdog) -> ! {
    loop {
        wd.feed();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(crate::WDT_FEED_MS)).await;
    }
}

/// Service USB (CDC + picotool reset) and emit the 1 Hz status lines.
/// The host must assert DTR to see output.
#[embassy_executor::task]
async fn usb_task(
    mut usb_dev: UsbDevice<'static, hal::usb::UsbBus>,
    mut serial: SerialPort<'static, hal::usb::UsbBus>,
    mut reset_iface: crate::pico_reset::PicoResetInterface,
) -> ! {
    let mut n: u32 = 0;
    // Rates use measured elapsed time; the 1 ms cadence slips under load.
    let mut last_emit_us = embassy_time::Instant::now().as_micros();
    // Previous counters for `[Lan]` deltas.
    let (mut prev_lan_tx, mut prev_lan_rx) = (0u32, 0u32);
    #[cfg(feature = "router")]
    let (mut prev_spi, mut prev_net) = (0u32, 0u32);
    // Previous counters and conntrack high-water for `[Perf]`.
    #[cfg(feature = "router")]
    let (mut prev_to_wan, mut prev_to_lan, mut prev_sent, mut ct_hwm, mut prev_c1, mut prev_fwd) =
        (0u32, 0u32, 0u32, 0usize, 0u32, 0u32);
    loop {
        usb_dev.poll(&mut [&mut serial, &mut reset_iface]);
        // Honor a picotool -f reboot request from clean (non-IRQ) context.
        if let Some(kind) = reset_iface.take_pending_reboot() {
            hal::reboot::reboot(kind, crate::pico_reset::RebootArch::Normal);
        }
        n = n.wrapping_add(1);
        // ~1 Hz at the 1 ms poll cadence (slower under load — normalised below).
        if n % 1000 == 0 {
            // Measured window for all per-second rates (see last_emit_us above).
            let now_us = embassy_time::Instant::now().as_micros();
            let elapsed_us = now_us.wrapping_sub(last_emit_us).max(1);
            last_emit_us = now_us;

            let mut line: String<96> = String::new();
            let _ = write!(
                line,
                "[Cyw43] new={} init={} led={} ap={} net={} rx={} dhcp={} hb={}\r\n",
                CYW43_NEW_DONE.load(Ordering::Relaxed),
                CYW43_INIT_DONE.load(Ordering::Relaxed),
                CYW43_LED_DONE.load(Ordering::Relaxed),
                CYW43_AP_DONE.load(Ordering::Relaxed),
                CYW43_NET_UP.load(Ordering::Relaxed),
                crate::cyw43_phy::CYW43_RX_FRAMES.load(Ordering::Relaxed),
                crate::dhcp_server::DHCP_TX.load(Ordering::Relaxed),
                n / 1000,
            );
            cdc_write_all(&mut usb_dev, &mut serial, &mut reset_iface, line.as_bytes());

            // [Lan]: cyw43 TX/RX rates, TX backpressure, and (router) the core-0 split.
            // Its own line so CDC framing can't truncate it.
            {
                let tx = LAN_BULK_TX_BYTES.load(Ordering::Relaxed);
                let rx = LAN_SINK_RX_BYTES.load(Ordering::Relaxed);
                let tx_kbs =
                    (tx.wrapping_sub(prev_lan_tx) as u64 * 1_000_000 / elapsed_us / 1024) as u32;
                let rx_kbs =
                    (rx.wrapping_sub(prev_lan_rx) as u64 * 1_000_000 / elapsed_us / 1024) as u32;
                prev_lan_tx = tx;
                prev_lan_rx = rx;
                let mut lline: String<160> = String::new();
                let _ = write!(
                    lline,
                    "[Lan] tx={}KB/s rx={}KB/s txbusy={} rxframes={}",
                    tx_kbs,
                    rx_kbs,
                    crate::cyw43_phy::CYW43_TX_BUSY.load(Ordering::Relaxed),
                    crate::cyw43_phy::CYW43_RX_FRAMES.load(Ordering::Relaxed),
                );
                #[cfg(feature = "router")]
                {
                    let spi = crate::cycles::CYW43_SPI_BUSY.load(Ordering::Relaxed);
                    let net = crate::cycles::LAN_NET_BUSY.load(Ordering::Relaxed);
                    let spi0 = crate::cycles::permille_over(spi.wrapping_sub(prev_spi), elapsed_us);
                    let net0 = crate::cycles::permille_over(net.wrapping_sub(prev_net), elapsed_us);
                    prev_spi = spi;
                    prev_net = net;
                    crate::cycles::SPI0_PERMILLE.store(spi0, Ordering::Relaxed);
                    crate::cycles::NET0_PERMILLE.store(net0, Ordering::Relaxed);
                    let _ = write!(
                        lline,
                        " spi0={}.{}% net0={}.{}%",
                        spi0 / 10,
                        spi0 % 10,
                        net0 / 10,
                        net0 % 10,
                    );
                }
                let _ = write!(lline, "\r\n");
                cdc_write_all(&mut usb_dev, &mut serial, &mut reset_iface, lline.as_bytes());
            }

            // [Wan]: formatted from wan_task's snapshot, keeping CDC output on one task.
            #[cfg(feature = "router")]
            {
                let snap = critical_section::with(|cs| WAN_PUB.borrow(cs).get());
                let mut wline: String<160> = String::new();
                let _ = write!(
                    wline,
                    "[Wan] core1={} ",
                    if WAN_CORE1_OK.load(Ordering::Relaxed) != 0 { "ok" } else { "FAIL" }
                );
                match snap {
                    Some(w) => w.write_status(&mut wline),
                    None => {
                        let _ = write!(wline, "(no lease yet)");
                    }
                }
                let _ = write!(wline, "\r\n");
                cdc_write_all(&mut usb_dev, &mut serial, &mut reset_iface, wline.as_bytes());

                // [Fwd] on its own line so CDC framing can't truncate counters.
                let mut fline: String<96> = String::new();
                let _ = write!(
                    fline,
                    "[Fwd] l2w={} w2l={} sent={} drop={}\r\n",
                    crate::forward::FWD_L2W.load(Ordering::Relaxed),
                    crate::forward::FWD_W2L.load(Ordering::Relaxed),
                    crate::forward::FWD_SENT.load(Ordering::Relaxed),
                    crate::forward::FWD_DROP.load(Ordering::Relaxed),
                );
                cdc_write_all(&mut usb_dev, &mut serial, &mut reset_iface, fline.as_bytes());

                // [Nat] conntrack counters.
                let mut nline: String<96> = String::new();
                let _ = write!(
                    nline,
                    "[Nat] ct={}/{} out={} in={} new={} evict={} drop={}\r\n",
                    crate::conntrack::live_count(),
                    crate::conntrack::CT_CAP,
                    crate::conntrack::NAT_OUT.load(Ordering::Relaxed),
                    crate::conntrack::NAT_IN.load(Ordering::Relaxed),
                    crate::conntrack::NAT_NEW.load(Ordering::Relaxed),
                    crate::conntrack::NAT_EVICT.load(Ordering::Relaxed),
                    crate::conntrack::NAT_DROP.load(Ordering::Relaxed),
                );
                cdc_write_all(&mut usb_dev, &mut serial, &mut reset_iface, nline.as_bytes());

                // [Perf]: routed rates (up = LAN→WAN, dn = WAN→LAN) and drop causes.
                let to_wan = crate::forward::FWD_BYTES_TO_WAN.load(Ordering::Relaxed);
                let to_lan = crate::forward::FWD_BYTES_TO_LAN.load(Ordering::Relaxed);
                let sent = crate::forward::FWD_SENT.load(Ordering::Relaxed);
                let up_kbs =
                    (to_wan.wrapping_sub(prev_to_wan) as u64 * 1_000_000 / elapsed_us / 1024) as u32;
                let dn_kbs =
                    (to_lan.wrapping_sub(prev_to_lan) as u64 * 1_000_000 / elapsed_us / 1024) as u32;
                let pps = (sent.wrapping_sub(prev_sent) as u64 * 1_000_000 / elapsed_us) as u32;
                prev_to_wan = to_wan;
                prev_to_lan = to_lan;
                prev_sent = sent;
                let ct = crate::conntrack::live_count();
                if ct > ct_hwm {
                    ct_hwm = ct;
                }
                // cpu1 = core-1 DMA IRQ time only (thread decode isn't counted).
                // cpu0 = forwarding share of core 0, not total load.
                let c1 = crate::cycles::CORE1_BUSY.load(Ordering::Relaxed);
                let fwd = crate::cycles::FWD_BUSY.load(Ordering::Relaxed);
                let cpu1 = crate::cycles::permille_over(c1.wrapping_sub(prev_c1), elapsed_us);
                let cpu0 = crate::cycles::permille_over(fwd.wrapping_sub(prev_fwd), elapsed_us);
                prev_c1 = c1;
                prev_fwd = fwd;
                crate::cycles::CPU1_PERMILLE.store(cpu1, Ordering::Relaxed);
                crate::cycles::CPU0_PERMILLE.store(cpu0, Ordering::Relaxed);
                let mut pline: String<256> = String::new();
                let _ = write!(
                    pline,
                    "[Perf] up={}KB/s dn={}KB/s pps={} cpu1={}.{}% cpu0={}.{}% \
                     qmax={}/{} cthwm={}/{} \
                     drop[qf={} nh={} nat={} txb={} oth={}]\r\n",
                    up_kbs,
                    dn_kbs,
                    pps,
                    cpu1 / 10,
                    cpu1 % 10,
                    cpu0 / 10,
                    cpu0 % 10,
                    crate::forward::FWD_QHWM_L2W.load(Ordering::Relaxed),
                    crate::forward::FWD_QHWM_W2L.load(Ordering::Relaxed),
                    ct_hwm,
                    crate::conntrack::CT_CAP,
                    crate::forward::FWD_DROP_QFULL.load(Ordering::Relaxed),
                    crate::forward::FWD_DROP_NONH.load(Ordering::Relaxed),
                    crate::forward::FWD_DROP_NAT.load(Ordering::Relaxed),
                    crate::forward::FWD_DROP_TXBUSY.load(Ordering::Relaxed),
                    crate::forward::FWD_DROP_OTHER.load(Ordering::Relaxed),
                );
                cdc_write_all(&mut usb_dev, &mut serial, &mut reset_iface, pline.as_bytes());
            }
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

/// cyw43 event loop. Must run continuously for the chip to stay up.
#[embassy_executor::task]
async fn cyw43_runner_task(runner: CywRunner) -> ! {
    runner.run().await
}

/// LAN side: smoltcp `Interface` over [`Cyw43Phy`] at `192.168.4.1/24`.
/// `auto-icmp-echo-reply` answers pings without a socket.
#[embassy_executor::task]
async fn net_task(net: cyw43::NetDriver<'static>, mac: [u8; 6]) -> ! {
    // Router: wrap the phy so off-LAN traffic diverts to the WAN.
    // The LAN IP is static, so forwarding starts immediately.
    #[cfg(not(feature = "router"))]
    let mut device = Cyw43Phy::new(net);
    #[cfg(feature = "router")]
    let mut device = crate::forward::ForwardingDevice::new(
        Cyw43Phy::new(net),
        crate::forward::IfaceCfg {
            iface: crate::forward::Iface::Lan,
            our_mac: mac,
            our_ip: LAN_IP,
            subnet: Ipv4Cidr::new(LAN_IP, LAN_PREFIX),
            gateway: None,
            accept_dst: None,
        },
        &crate::forward::LAN_TO_WAN,
    );

    let now = || Instant::from_micros(embassy_time::Instant::now().as_micros() as i64);
    let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(mac)));
    config.random_seed = embassy_time::Instant::now().as_ticks();
    let mut iface = Interface::new(config, &mut device, now());
    iface.update_ip_addrs(|addrs| {
        let _ = addrs.push(IpCidr::new(IpAddress::Ipv4(LAN_IP), LAN_PREFIX));
    });

    // Sockets: DHCP server (UDP :67), mgmt HTTP + `/bulk` (TCP :80), sink (TCP :9999).
    let mut sockets_storage: [SocketStorage; 3] = [SocketStorage::EMPTY; 3];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let mut dhcp_rx_meta = [udp::PacketMetadata::EMPTY; 4];
    let mut dhcp_rx_payload = [0u8; 1536];
    let mut dhcp_tx_meta = [udp::PacketMetadata::EMPTY; 4];
    let mut dhcp_tx_payload = [0u8; 1536];
    let dhcp_socket = udp::Socket::new(
        udp::PacketBuffer::new(&mut dhcp_rx_meta[..], &mut dhcp_rx_payload[..]),
        udp::PacketBuffer::new(&mut dhcp_tx_meta[..], &mut dhcp_tx_payload[..]),
    );
    let dhcp_handle = sockets.add(dhcp_socket);
    let mut dhcp = DhcpServer::new();

    // Mgmt HTTP: small RX; 32 KB TX so `/bulk` is cyw43-limited, not poll-limited.
    let mut http_rx = [0u8; 1024];
    let mut http_tx = [0u8; 32 * 1024];
    let http_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut http_rx[..]),
        tcp::SocketBuffer::new(&mut http_tx[..]),
    );
    let http_handle = sockets.add(http_socket);
    let mut lan_http = LanHttp::Idle;

    // Upload sink: large RX buffer so the TCP window doesn't throttle uploads.
    let mut sink_rx = [0u8; 32 * 1024];
    let mut sink_tx = [0u8; 2048];
    let sink_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut sink_rx[..]),
        tcp::SocketBuffer::new(&mut sink_tx[..]),
    );
    let sink_handle = sockets.add(sink_socket);

    CYW43_NET_UP.store(1, Ordering::Relaxed);
    loop {
        {
            // Count net_task poll cycles toward `net0` (router). gSPI counts separately.
            #[cfg(feature = "router")]
            let _span = crate::cycles::CycleSpan::new(&crate::cycles::LAN_NET_BUSY);
            iface.poll(now(), &mut device, &mut sockets);
            // Answer DHCP after iface.poll; replies go out next poll.
            dhcp.poll(sockets.get_mut::<udp::Socket>(dhcp_handle));
            serve_status_http(sockets.get_mut::<tcp::Socket>(http_handle), &dhcp, &mut lan_http);
            serve_lan_sink(sockets.get_mut::<tcp::Socket>(sink_handle));
            // Router: send WAN→LAN forwarded frames out the cyw43 phy.
            #[cfg(feature = "router")]
            while let Ok(mut frame) = crate::forward::WAN_TO_LAN.try_receive() {
                device.egress(&mut frame, now());
            }
        }
        Timer::after(Duration::from_millis(5)).await;
    }
}

/// Upload sink on [`LAN_SINK_PORT`]: drain and count into [`LAN_SINK_RX_BYTES`].
/// Re-listens after close. Drive with `head -c 64M /dev/zero | nc 192.168.4.1 9999`.
fn serve_lan_sink(socket: &mut tcp::Socket) {
    if !socket.is_open() {
        let _ = socket.listen(LAN_SINK_PORT);
        return;
    }
    // Drain and count all available RX.
    while socket.can_recv() {
        match socket.recv(|buf| {
            let n = buf.len();
            (n, n)
        }) {
            Ok(0) | Err(_) => break,
            Ok(n) => {
                LAN_SINK_RX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
            }
        }
    }
    // Peer sent FIN and we've drained it (CLOSE_WAIT) → close so we re-listen.
    if !socket.may_recv() && socket.may_send() {
        socket.close();
    }
}

/// LAN mgmt HTTP on `192.168.4.1:80`:
/// - `GET /bulk`: stream [`LAN_BULK_BYTES`] of filler across polls.
/// - otherwise: one-shot status page (AP, LAN, DNS, clients, router stats).
///
/// Re-listens after each closed connection.
fn serve_status_http(socket: &mut tcp::Socket, dhcp: &DhcpServer, state: &mut LanHttp) {
    // 1 KB of 0x55 filler.
    const BULK_CHUNK: [u8; 1024] = [0x55; 1024];

    if !socket.is_open() {
        *state = LanHttp::Idle;
        let _ = socket.listen(80);
        return;
    }

    // Continue `/bulk`: header once, then top up the TX buffer each poll.
    if let LanHttp::Bulk {
        remaining,
        header_sent,
    } = state
    {
        if !*header_sent {
            if !socket.can_send() {
                return; // retry next poll
            }
            let mut head: String<128> = String::new();
            let _ = write!(
                head,
                "HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\n\
                 Content-Length: {LAN_BULK_BYTES}\r\nConnection: close\r\n\r\n",
            );
            let _ = socket.send_slice(head.as_bytes());
            *header_sent = true;
        }
        while *remaining > 0 && socket.can_send() {
            let n = (*remaining).min(BULK_CHUNK.len());
            match socket.send_slice(&BULK_CHUNK[..n]) {
                Ok(0) | Err(_) => break,
                Ok(sent) => {
                    *remaining -= sent;
                    LAN_BULK_TX_BYTES.fetch_add(sent as u32, Ordering::Relaxed);
                }
            }
        }
        if *remaining == 0 {
            socket.close();
            *state = LanHttp::Idle;
        }
        return;
    }

    // Idle: route once the request arrives (it fits in one segment).
    let mut route_bulk = false;
    let mut have_req = false;
    if socket.may_recv() {
        let _ = socket.recv(|buf| {
            have_req = !buf.is_empty();
            route_bulk = buf.starts_with(b"GET /bulk");
            (buf.len(), ())
        });
    }
    if !have_req {
        return; // respond only once the request has arrived
    }

    if route_bulk {
        // The Bulk branch sends the header on a later poll.
        *state = LanHttp::Bulk {
            remaining: LAN_BULK_BYTES,
            header_sent: false,
        };
        return;
    }

    // Default route `/` — the one-shot status page.
    if socket.can_send() {
        let uptime_s = embassy_time::Instant::now().as_secs();
        let dns = Ipv4Address::from(
            crate::dhcp_server::LAN_DNS_OFFER
                .load(Ordering::Relaxed)
                .to_be_bytes(),
        );
        // Body sized from POOL_LEN (~40 B/lease); `write!` truncates gracefully.
        const STATUS_BODY_CAP: usize = 256 + crate::dhcp_server::POOL_LEN * 40 + 768;
        let mut body: String<STATUS_BODY_CAP> = String::new();
        let _ = write!(
            body,
            "Pico RP2350 Wireless Router (Rust)\r\n\
             AP SSID:     {AP_SSID}\r\n\
             LAN gateway: 192.168.4.1/24\r\n\
             DNS offered: {dns}\r\n\
             uptime:      {uptime_s}s\r\n\
             DHCP replies:{}\r\n\
             LAN rx:      {}\r\n",
            crate::dhcp_server::DHCP_TX.load(Ordering::Relaxed),
            crate::cyw43_phy::CYW43_RX_FRAMES.load(Ordering::Relaxed),
        );

        // Connected clients — the active DHCP leases (IP + MAC).
        let _ = write!(body, "Clients:\r\n");
        let mut nclients = 0u32;
        for (ip, mac) in dhcp.active_leases() {
            nclients += 1;
            let _ = write!(body, "  {ip}  {}\r\n", crate::mac_str(mac));
        }
        if nclients == 0 {
            let _ = write!(body, "  (none)\r\n");
        }

        // Cumulative LAN bytes; rates and CPU% are on `[Lan]`.
        let _ = write!(
            body,
            "LAN perf:    tx={}B rx={}B txbusy={}\r\n",
            LAN_BULK_TX_BYTES.load(Ordering::Relaxed),
            LAN_SINK_RX_BYTES.load(Ordering::Relaxed),
            crate::cyw43_phy::CYW43_TX_BUSY.load(Ordering::Relaxed),
        );

        // WAN and NAPT (router only).
        #[cfg(feature = "router")]
        {
            let _ = write!(body, "WAN:         ");
            match critical_section::with(|cs| WAN_PUB.borrow(cs).get()) {
                Some(w) => w.write_status(&mut body),
                None => {
                    let _ = write!(body, "(no lease)");
                }
            }
            let _ = write!(
                body,
                "\r\nNAT:         ct={}/{} out={} in={} drop={}\r\n\
                 Forward:     sent={} drop={} (qf={} nh={} nat={} txb={} oth={})\r\n\
                 Bytes:       up={} dn={}\r\n\
                 Queue hwm:   l2w={} w2l={} (cap {})\r\n",
                crate::conntrack::live_count(),
                crate::conntrack::CT_CAP,
                crate::conntrack::NAT_OUT.load(Ordering::Relaxed),
                crate::conntrack::NAT_IN.load(Ordering::Relaxed),
                crate::conntrack::NAT_DROP.load(Ordering::Relaxed),
                crate::forward::FWD_SENT.load(Ordering::Relaxed),
                crate::forward::FWD_DROP.load(Ordering::Relaxed),
                crate::forward::FWD_DROP_QFULL.load(Ordering::Relaxed),
                crate::forward::FWD_DROP_NONH.load(Ordering::Relaxed),
                crate::forward::FWD_DROP_NAT.load(Ordering::Relaxed),
                crate::forward::FWD_DROP_TXBUSY.load(Ordering::Relaxed),
                crate::forward::FWD_DROP_OTHER.load(Ordering::Relaxed),
                crate::forward::FWD_BYTES_TO_WAN.load(Ordering::Relaxed),
                crate::forward::FWD_BYTES_TO_LAN.load(Ordering::Relaxed),
                crate::forward::FWD_QHWM_L2W.load(Ordering::Relaxed),
                crate::forward::FWD_QHWM_W2L.load(Ordering::Relaxed),
                crate::forward::CHAN_DEPTH,
            );
            // Latest CPU sample from usb_task. core1 counts DMA IRQ time only.
            let c1 = crate::cycles::CPU1_PERMILLE.load(Ordering::Relaxed);
            let c0 = crate::cycles::CPU0_PERMILLE.load(Ordering::Relaxed);
            let _ = write!(
                body,
                "CPU:         core1(rx-decode)={}.{}% core0(forward)={}.{}%\r\n",
                c1 / 10,
                c1 % 10,
                c0 / 10,
                c0 % 10,
            );
            // spi0 = gSPI transport; net0 = net_task stack.
            let spi0 = crate::cycles::SPI0_PERMILLE.load(Ordering::Relaxed);
            let net0 = crate::cycles::NET0_PERMILLE.load(Ordering::Relaxed);
            let _ = write!(
                body,
                "LAN cpu0:    spi0(gspi)={}.{}% net0(stack)={}.{}%\r\n",
                spi0 / 10,
                spi0 % 10,
                net0 / 10,
                net0 % 10,
            );
        }

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

/// Bring up cyw43: firmware, Runner task, CLM, AP, then `net_task`.
/// Then blinks the LED forever as a Runner liveness signal.
#[embassy_executor::task]
async fn cyw43_bootstrap_task(spawner: Spawner, pwr: WlOnPin, spi: PioSpiCyw43) -> ! {
    let fw = cyw43::aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let nvram = cyw43::aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");
    let clm: &[u8] = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // cyw43::State is large (driver + channel buffers) — keep it in a static.
    static mut STATE: cyw43::State = cyw43::State::new();
    let state = unsafe { &mut *core::ptr::addr_of_mut!(STATE) };

    let (net, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    CYW43_NEW_DONE.store(1, Ordering::Relaxed);
    // Hand the event loop to its own task so it runs concurrently with init.
    if let Ok(t) = cyw43_runner_task(runner) {
        spawner.spawn(t);
    }

    control.init(clm).await;
    CYW43_INIT_DONE.store(1, Ordering::Relaxed);

    // AP with power save off, so the radio doesn't miss probes.
    control
        .set_power_management(cyw43::PowerManagementMode::None)
        .await;
    control
        .start_ap_wpa2(AP_SSID, AP_PASSPHRASE, AP_CHANNEL)
        .await;
    CYW43_AP_DONE.store(1, Ordering::Relaxed);

    // Hand the NetDriver and our MAC to net_task.
    let mac = control.address().await;
    if let Ok(t) = net_task(net, mac) {
        spawner.spawn(t);
    }

    // Blink forever: proves the Runner stays up.
    let mut on = false;
    loop {
        on = !on;
        control.gpio_set(0, on).await;
        CYW43_LED_DONE.store(1, Ordering::Relaxed);
        Timer::after(Duration::from_millis(250)).await;
    }
}

/// Wireless-only entry: USB, time-driver IRQ, then the executor forever.
/// Spawns the watchdog feeder, `usb_task`, and cyw43 bring-up.
///
/// `spi` must be a fresh [`PioSpiCyw43`]. The time driver owns TIMER0 ALARM0.
pub fn run(
    pwr: WlOnPin,
    spi: PioSpiCyw43,
    usb: hal::pac::USB,
    usb_dpram: hal::pac::USB_DPRAM,
    usb_clock: hal::clocks::UsbClock,
    resets: &mut hal::pac::RESETS,
    mut watchdog: hal::Watchdog,
) -> ! {
    let (usb_dev, serial, reset_iface) = build_usb(usb, usb_dpram, usb_clock, resets);

    // Let the time-driver's ALARM0 fire on this core.
    unsafe {
        hal::arch::interrupt_unmask(ALARM_IRQ);
        hal::arch::interrupt_enable();
    }

    // The executor must live for 'static; stash it in a one-shot static.
    static mut EXECUTOR: core::mem::MaybeUninit<Executor> = core::mem::MaybeUninit::uninit();
    let executor = unsafe {
        let p = core::ptr::addr_of_mut!(EXECUTOR);
        (*p).write(Executor::new());
        &mut *(*p).as_mut_ptr()
    };

    // Arm the watchdog before the executor runs.
    watchdog.start(hal::fugit::MicrosDurationU32::micros(crate::WDT_TIMEOUT_US));

    executor.run(|spawner| {
        // `#[task]` returns a `Result` (the task arena can be full).
        if let Ok(t) = watchdog_feed_task(watchdog) {
            spawner.spawn(t);
        }
        if let Ok(t) = usb_task(usb_dev, serial, reset_iface) {
            spawner.spawn(t);
        }
        if let Ok(t) = cyw43_bootstrap_task(spawner, pwr, spi) {
            spawner.spawn(t);
        }
    });
}

// =====================================================================
// 4. Router: both interfaces under one executor
// =====================================================================
//
// `run` plus `wan_task`, a second smoltcp `Interface` over `EthMac`. 10BT RX
// decodes on core 1; `wan_task` only drains the inbox. Design: docs/r15-plan.md §6.

/// `wan_task`'s snapshot for `usb_task`'s `[Wan]` line.
#[cfg(feature = "router")]
static WAN_PUB: Mutex<core::cell::Cell<Option<crate::wan::WanState>>> =
    Mutex::new(core::cell::Cell::new(None));
/// Whether core 1 (the 10BT RX engine) launched — surfaced in the `[Wan]` line.
#[cfg(feature = "router")]
static WAN_CORE1_OK: AtomicU32 = AtomicU32::new(0);

/// WAN task: smoltcp `Interface` over `EthMac` with DHCP client, ping, DNS,
/// NAPT forwarding, and NLP keepalive. Polls every 1 ms.
#[cfg(feature = "router")]
#[embassy_executor::task]
async fn wan_task(mut mac: crate::eth_mac::EthMac) -> ! {
    use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
    use smoltcp::phy::Device as _; // bring `capabilities()` into scope
    use smoltcp::socket::{dhcpv4, dns, icmp};
    use smoltcp::wire::{EthernetAddress, HardwareAddress};

    // Checksum caps for ICMP echoes.
    let dev_checksum = mac.capabilities().checksum;

    // Wrap the phy for NAPT forwarding to the LAN subnet. Disabled until leased.
    let mut device = crate::forward::ForwardingDevice::new_napt(
        mac,
        crate::forward::IfaceCfg {
            iface: crate::forward::Iface::Wan,
            our_mac: crate::OUR_MAC,
            our_ip: Ipv4Address::UNSPECIFIED,
            subnet: Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0),
            gateway: None,
            accept_dst: Some(Ipv4Cidr::new(LAN_IP, LAN_PREFIX)),
        },
        &crate::forward::WAN_TO_LAN,
    );

    let now = || Instant::from_micros(embassy_time::Instant::now().as_micros() as i64);
    let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(crate::OUR_MAC)));
    config.random_seed = embassy_time::Instant::now().as_ticks();
    let mut iface = Interface::new(config, &mut device, now());
    // No static IP — the dhcpv4 client installs address + default route on lease.

    let mut sockets_storage: [SocketStorage; 4] = [SocketStorage::EMPTY; 4];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let dhcp_handle = sockets.add(dhcpv4::Socket::new());

    let mut icmp_rx_meta = [icmp::PacketMetadata::EMPTY; 8];
    let mut icmp_rx_payload = [0u8; 512];
    let mut icmp_tx_meta = [icmp::PacketMetadata::EMPTY; 8];
    let mut icmp_tx_payload = [0u8; 512];
    let icmp_handle = sockets.add(icmp::Socket::new(
        icmp::PacketBuffer::new(&mut icmp_rx_meta[..], &mut icmp_rx_payload[..]),
        icmp::PacketBuffer::new(&mut icmp_tx_meta[..], &mut icmp_tx_payload[..]),
    ));

    let mut dns_queries: [Option<dns::DnsQuery>; 2] = [None, None];
    let dns_handle = sockets.add(dns::Socket::new(&[], &mut dns_queries[..]));

    let mut wan = crate::wan::WanState::new();

    let mut next_nlp = embassy_time::Instant::now();
    let mut next_ping = embassy_time::Instant::now();
    // Last lease's gateway, to ARP it as soon as it changes.
    let mut prev_gw: Option<Ipv4Address> = None;
    loop {
        iface.poll(now(), &mut device, &mut sockets);
        crate::wan::dhcp_apply(&mut iface, &mut sockets, dhcp_handle, dns_handle, &mut wan);
        // Sync the forwarder with the lease (enables WAN forwarding).
        if let Some(cidr) = wan.addr {
            device.set_lease(cidr, wan.gw);
        }
        // New gateway: ARP it now so the first forwarded frame finds its MAC.
        if wan.gw.is_some() && wan.gw != prev_gw {
            device.arp_gateway(now());
        }
        prev_gw = wan.gw;
        // Offer the WAN resolver to LAN clients; NAPT forwards their queries.
        if let Some(dns) = wan.dns0 {
            crate::dhcp_server::LAN_DNS_OFFER
                .store(u32::from_be_bytes(dns.octets()), Ordering::Relaxed);
        }
        crate::wan::ping_drain(&mut sockets, icmp_handle, &mut wan, &dev_checksum);
        crate::wan::dns_harvest(&mut sockets, dns_handle, &mut wan);

        let nowt = embassy_time::Instant::now();
        // NLP keepalive every 16 ms (IEEE 802.3 link integrity).
        if nowt >= next_nlp {
            next_nlp = nowt + Duration::from_millis(16);
            device.inner_mut().send_nlp();
        }
        // Ping 8.8.8.8 + (re)start a DNS query once a second, once we hold a lease.
        if nowt >= next_ping {
            next_ping = nowt + Duration::from_secs(1);
            if wan.addr.is_some() {
                crate::wan::ping_send(&mut sockets, icmp_handle, &mut wan, &dev_checksum);
                crate::wan::dns_start(&mut iface, &mut sockets, dns_handle, &mut wan);
                // Re-ARP the gateway each second until learned.
                if let Some(gw) = wan.gw {
                    if !crate::forward::wan_neigh_known(gw) {
                        device.arp_gateway(now());
                    }
                }
            }
            // Sweep idle NAPT entries once a second.
            crate::forward::nat_reap(now().total_millis().max(0) as u64);
        }
        // Send LAN→WAN forwarded frames out the 10BT phy.
        while let Ok(mut frame) = crate::forward::LAN_TO_WAN.try_receive() {
            device.egress(&mut frame, now());
        }
        // Publish a snapshot for usb_task's [Wan] line.
        critical_section::with(|cs| WAN_PUB.borrow(cs).set(Some(wan)));

        Timer::after(Duration::from_millis(1)).await;
    }
}

/// Router entry: `run` plus `wan_task(mac)`. Core 1 must already be running
/// (`setup_eth_mac`). Never returns.
#[cfg(feature = "router")]
#[allow(clippy::too_many_arguments)] // dispatch boundary — both interfaces' resources
pub fn run_router(
    mac: crate::eth_mac::EthMac,
    core1_ok: bool,
    pwr: WlOnPin,
    spi: PioSpiCyw43,
    usb: hal::pac::USB,
    usb_dpram: hal::pac::USB_DPRAM,
    usb_clock: hal::clocks::UsbClock,
    resets: &mut hal::pac::RESETS,
    mut watchdog: hal::Watchdog,
) -> ! {
    WAN_CORE1_OK.store(core1_ok as u32, Ordering::Relaxed);
    let (usb_dev, serial, reset_iface) = build_usb(usb, usb_dpram, usb_clock, resets);

    unsafe {
        hal::arch::interrupt_unmask(ALARM_IRQ);
        hal::arch::interrupt_enable();
    }

    static mut EXECUTOR: core::mem::MaybeUninit<Executor> = core::mem::MaybeUninit::uninit();
    let executor = unsafe {
        let p = core::ptr::addr_of_mut!(EXECUTOR);
        (*p).write(Executor::new());
        &mut *(*p).as_mut_ptr()
    };

    // Arm the watchdog before the executor runs; the feeder task pets it.
    watchdog.start(hal::fugit::MicrosDurationU32::micros(crate::WDT_TIMEOUT_US));

    executor.run(|spawner| {
        if let Ok(t) = watchdog_feed_task(watchdog) {
            spawner.spawn(t);
        }
        if let Ok(t) = usb_task(usb_dev, serial, reset_iface) {
            spawner.spawn(t);
        }
        if let Ok(t) = cyw43_bootstrap_task(spawner, pwr, spi) {
            spawner.spawn(t);
        }
        if let Ok(t) = wan_task(mac) {
            spawner.spawn(t);
        }
    });
}
