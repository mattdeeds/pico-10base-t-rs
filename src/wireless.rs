//! R13 — wireless router scaffolding (Pico 2 W / CYW43439), Option A.
//!
//! This module is gated behind the `wireless` cargo feature and is the
//! board-independent foundation for the CYW43 integration (see
//! `docs/router-plan.md` §4/§5). It keeps the Hazard3 RISC-V / `rp235x-hal`
//! stack and bridges to the embassy/cyw43 world via three shims:
//!
//! 1. **Time driver** — an `embassy-time-driver` impl backed by RP2350 TIMER0
//!    (`now()` from the µs counter, `schedule_wake()` via ALARM0 + its IRQ).
//!    This is what lets `embassy-time` (and therefore cyw43's `Timer::after`
//!    delays) work without `embassy-rp`.
//! 2. **Async runtime** — `embassy-executor` with its `platform-riscv32`
//!    backend, run from inside our `#[hal::entry]` main. Proves async tasks
//!    run on Hazard3.
//! 3. **`SpiBusCyw43` transport** — our own half-duplex gSPI implementation on
//!    the free **PIO1** (instead of embassy-rp's `cyw43-pio`). *Skeleton here;
//!    the real gSPI PIO program + on-board bring-up is the R13 on-device step.*
//!
//! Nothing in here has run on hardware yet — this is the "compiles + links +
//! reviewable before the Pico 2 W arrives" milestone.

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

/// WL_ON (GP23) as a push-pull SIO output — the CYW43 `pwr` pin cyw43 owns and
/// power-cycles during `cyw43::new`. Concrete (not generic) so it can name the
/// type of the long-lived `Runner` task. GP23 resets to `(PullDown, Null)`, so
/// `into_push_pull_output()` yields exactly this type.
pub type WlOnPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio23, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>;

// =====================================================================
// 0. Synchronous gSPI bring-up probe (R13 first on-board milestone)
// =====================================================================
//
// Before the full async cyw43 stack, prove the most uncertain things — the
// pin map, the power sequence, and our understanding of the gSPI protocol —
// with a slow software bit-bang that reads the CYW43's bus *test register*
// (REG_BUS_TEST_RO = 0x14), which must read back `0xFEEDBEAD`. The result is
// stashed in `CYW43_PROBE` and logged over the existing 10BASE-T CDC/UDP
// telemetry (this probe runs inside the normal production build, gated by the
// `wireless` feature — so we keep that telemetry instead of needing USB-in-async
// or LED visibility). Once this reads 0xFEEDBEAD, the transport is proven and we
// move the same wire format onto the PIO1 gSPI program + the async cyw43 driver.
//
// Pico 2 W CYW43 pins (fixed on the PCB): GP23 = WL_ON (power), GP24 = DATA
// (bidir gSPI), GP25 = CS, GP29 = CLK.

/// Result of the last gSPI test-register probe. 0 = not run; otherwise the
/// (byte-order-corrected) value read — `0xFEEDBEAD` means the bus is alive.
pub static CYW43_PROBE: AtomicU32 = AtomicU32::new(0);
/// First read of the probe loop + a "all 64 reads identical?" flag, for
/// diagnosis: first==last & stable ⇒ chip driving a mis-framed value (timing);
/// varying ⇒ DATA floating / chip not responding (power/setup).
pub static CYW43_PROBE_FIRST: AtomicU32 = AtomicU32::new(0);
pub static CYW43_PROBE_STABLE: AtomicU32 = AtomicU32::new(0);

const PIN_PWR: u32 = 23;
const PIN_DATA: u32 = 24;
const PIN_CS: u32 = 25;
const PIN_CLK: u32 = 29;
/// Half-clock-phase spin (~0.25 µs @ 240 MHz → ~2 MHz gSPI). Slow + safe for
/// bring-up; the real PIO transport will run far faster.
#[allow(dead_code)] // bit-bang probe machinery — superseded by probe_cyw43_pio, kept for reference
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

// Raw SIO/IO_BANK0/PADS_BANK0 GPIO bit-banging — the probe doesn't consume the
// typed HAL pins (avoids the ownership tangle with the production `led` on
// GP25); it poetically pokes the registers directly. Experimental + gated.
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
#[allow(dead_code)] // only the bit-bang read needs this; the PIO probe reads via the SM
fn gpio_read(n: u32) -> u32 {
    (unsafe { (*hal::pac::SIO::ptr()).gpio_in().read().bits() } >> n) & 1
}

/// Route a pin to SIO function and de-isolate its pad (RP2350 pads power up
/// isolated — the `iso` bit must be cleared before use). `ie` enabled so we can
/// also read it (for the bidirectional DATA line).
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

/// One gSPI command+read transaction, bit-banged. Clocks `cmd` out MSB-first on
/// DATA (chip latches on the rising CLK edge), turns the line around, then
/// clocks 32 bits back in (sampled while CLK high). CS held low throughout.
#[allow(dead_code)] // superseded by the PIO1 transport (pio_cmd_read32)
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

/// R13 bring-up probe: power the CYW43, read its bus test register, stash the
/// result in [`CYW43_PROBE`]. Synchronous; call once at boot (it spins ~270 ms
/// for the power-up sequence). Reads `0xFEEDBEAD` iff the transport + chip are
/// alive. Uses `arch::delay` for timing (no Timer needed).
#[allow(dead_code)] // superseded by probe_cyw43_pio (R13 Step 1); kept for reference
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

    // read32_swapped(FUNC_BUS=0, REG_BUS_TEST_RO=0x14) — the initial gSPI mode
    // is 16-bit-swapped, so swap the cmd and the result (per cyw43 spi.rs). The
    // real driver loops `while != FEEDBEAD` because the bus can take a few reads
    // to settle after power-up — do the same, reporting if it ever latches.
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
// 0b. PIO1 gSPI bring-up probe (R13 Step 1 — supersedes the bit-bang)
// =====================================================================
//
// Same goal as `probe_cyw43` (read TEST_RO, expect 0xFEEDBEAD) but over a real
// PIO1 gSPI state machine instead of a CPU bit-bang — so the chip's input
// synchronizer sees the clock/data timing it actually expects (the bit-bang's
// documented blind spot). This is the de-risk before the async cyw43 stack:
// 0xFEEDBEAD here ⇒ our transport is proven, and we layer cyw43 on top.
//
// Self-contained + FIFO-driven (rp235x-hal has no easy set_x/set_y on a running
// SM): each transaction pushes [write_bits-1, read_bits-1, data words...]. The
// program loads X/Y from the first two words (autopull), clocks `write_bits` out
// MSB-first (DATA driven, latched on the rising CLK edge), turns DATA around,
// then clocks `read_bits` in (sampled while CLK is high) and autopushes — phase
// + turnaround matched to embassy's proven cyw43-pio default program.
//
// Wiring: CLK = GP29 (side-set), DATA = GP24 (out/set/in — bidirectional), both
// routed to FunctionPio1 by the caller. CS = GP25 and WL_ON = GP23 are driven
// directly as SIO here (CS held low for the whole transaction).

/// Build + configure the PIO1 gSPI state machine — the single source of truth
/// for the program, shared by the probe and the async [`PioSpiCyw43`] transport.
/// Drives the bus idle (CLK low, DATA low via `set_pins`) and returns the
/// *stopped* SM so the caller starts it at the right moment: the probe starts it
/// *after* the WL_ON power-up (gotcha #11); `PioSpiCyw43::new` starts it in
/// `new()` (before cyw43 powers the chip). Does NOT touch CS or WL_ON.
fn build_gspi_sm(
    pio: &mut hal::pio::PIO<PIO1>,
    sm: UninitStateMachine<(PIO1, SM0)>,
    sys_clk_hz: u32,
) -> (
    StateMachine<(PIO1, SM0), Stopped>,
    Tx<(PIO1, SM0)>,
    Rx<(PIO1, SM0)>,
) {
    // Self-contained FIFO-driven gSPI program (docs/router-plan.md §11 #1): each
    // txn pushes [write_bits-1, read_bits-1, words...]. MSB-first; CLK side-set
    // idles low; chip latches our DATA on the rising edge, drives its DATA which
    // we sample while CLK is high. Phase + turnaround matched to embassy
    // cyw43-pio 0.7.0's default program.
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

    // ~2 MHz gSPI clock (2 PIO cycles/bit) — slow + safe for bring-up.
    let (div_int, div_frac) = crate::pio_util::clock_divider(sys_clk_hz, 4_000_000.0);
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
    // Bus idle: CLK + DATA outputs, both low. set_pins latches the pad outputs
    // low and they hold until the caller start()s the SM.
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

/// Build the PIO1 gSPI SM, power the CYW43, and read TEST_RO. Result lands in
/// [`CYW43_PROBE`] (0xFEEDBEAD = transport + chip alive; 0xDEAD_0001 = the SM
/// produced no word = stalled; 0xDEAD_0000 = never read). One-shot at boot —
/// the SM is dropped (stopped) on return. `sys_clk_hz` sizes the PIO divider
/// (~2 MHz gSPI clock — conservative for first bring-up) and the power delays.
#[allow(dead_code)] // Step 1 transport probe — superseded by cyw43_new_blocking; kept as a minimal fallback
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

    // Build the gSPI SM (shared with PioSpiCyw43); drives CLK/DATA low so the bus
    // is held idle through the WL_ON power-up below. Returned stopped — we start
    // it AFTER power-up (the proven ordering).
    let (sm, mut tx, mut rx) = build_gspi_sm(pio, sm, sys_clk_hz);

    // Power-cycle WL_ON *after* the bus is held idle (CLK low, CS high, DATA low),
    // matching embassy/cyw43 (PioSpi is built — pins low — then init() powers the
    // chip). A floating CLK/DATA during power-up can latch the CYW43 into a wrong
    // gSPI mode; this ordering was the one thing mine got wrong vs the driver.
    ms(20); // WL_ON held low ≥20 ms (chip off)
    gpio_set(PIN_PWR); // WL_ON high
    ms(250); // settle while the chip boots its gSPI

    let _sm = sm.start(); // keep alive (drop would stop the SM)

    // Read TEST_RO (FUNC_BUS=0, addr=0x14) — initial gSPI mode is 16-bit-swapped,
    // so swap the cmd and the result (same as the bit-bang / cyw43 read32_swapped).
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

/// One gSPI cmd+read32 transaction over the PIO1 SM. CS is held low for the
/// whole transaction. Pushes `[31, 63, cmd]` — X=write_bits-1 (32 cmd bits out),
/// Y=read_bits-1 (64 bits in = data word + trailing status word, exactly as
/// cyw43's `cmd_read` does: `read.len()*32 + 32 - 1` with `read.len()==1`). The
/// SM autopushes two words; we return the first (the data). Returns
/// `0xDEAD_0001` if the SM produced nothing (stalled).
#[allow(dead_code)] // used by the Step 1 probe (probe_cyw43_pio), kept as a fallback
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
// 0c. Pin self-test (R13 Step 1 debug)
// =====================================================================
//
// The PIO gSPI probe read floating data — same as the bit-bang — despite the
// board being MicroPython-verified good. Before blaming the transport, confirm
// the RP2350 can actually *drive* the four CYW43 pins: drive each as a plain
// SIO output, low then high, and read it back via GPIO_IN. For a healthy pad
// (OE on, RP2350 pad iso cleared, IE on, nothing external holding it) the
// readback follows the driven level. A pin whose readback doesn't follow points
// at a pad/funcsel/iso/OE fault on our side; WL_ON not following ⇒ the chip is
// never powered (would explain everything). Result bits land at each pin's index.

/// Per-pin GPIO_IN readback when the pin was driven LOW (want 0 at bits 23/24/25/29).
#[allow(dead_code)] // pin self-test (Step 1 debug) — pads proven; kept for reference
pub static CYW43_PIN_LO: AtomicU32 = AtomicU32::new(0);
/// Per-pin GPIO_IN readback when the pin was driven HIGH (want 1 at bits 23/24/25/29).
#[allow(dead_code)] // pin self-test (Step 1 debug) — pads proven; kept for reference
pub static CYW43_PIN_HI: AtomicU32 = AtomicU32::new(0);

/// Drive WL_ON/CS/CLK/DATA low then high as SIO outputs, read each back. See the
/// section comment. Leaves WL_ON high; the gSPI probe re-power-cycles after, so
/// any toggling here is reset before the real transaction.
#[allow(dead_code)] // pin self-test (Step 1 debug) — pads proven; kept for reference
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
// embassy-time is configured for a 1 MHz tick (`tick-hz-1_000_000`), which
// matches TIMER0's 1 µs counter exactly — so `now()` is the raw µs count and
// no scaling is needed. ALARM0 (+ TIMER0_IRQ_0) drives the wakeups.

/// We drive wakeups off TIMER0 ALARM0 → the TIMER0_IRQ_0 line.
// Part of the async-runtime scaffolding (not used by the sync bring-up probe).
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

    /// Arm (or disarm) ALARM0 for the next deadline `at` (µs). ALARM0 compares
    /// only the low 32 bits, so for a deadline more than ~71 min out we arm a
    /// near-max intermediate point and re-arm when it fires (the IRQ handler
    /// finds nothing expired and reschedules). `u64::MAX` = no pending timer →
    /// mask the IRQ.
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
// 2. SpiBusCyw43 transport on PIO1 (skeleton — real gSPI PIO is on-board work)
// =====================================================================
//
// cyw43's core is transport-agnostic via `SpiBusCyw43`. embassy's `cyw43-pio`
// is the embassy-rp reference impl; we provide our own on `rp235x-hal`'s free
// PIO1 (a half-duplex "gSPI": shared DATA line, ~33 MHz clock). The cyw43 CS is
// held low for the whole transfer by the impl.
//
// R13 Step 2: real cmd_write/cmd_read over the proven PIO1 FIFO push/pull
// (busy-poll; DMA later). Bit counts match embassy cyw43-pio 0.7.0.

/// Our PIO1-based half-duplex gSPI transport for the CYW43439. Owns the running
/// gSPI SM + its FIFOs; CS (GP25) is driven directly as SIO. Built by
/// [`PioSpiCyw43::new`] *before* `cyw43::new`, so the bus is already idle through
/// the chip's WL_ON power-up (gotcha #11). (Constructed in R13 Step 3.)
#[allow(dead_code)]
pub struct PioSpiCyw43 {
    _sm: StateMachine<(PIO1, SM0), Running>,
    tx: Tx<(PIO1, SM0)>,
    rx: Rx<(PIO1, SM0)>,
}

#[allow(dead_code)]
impl PioSpiCyw43 {
    /// Build the gSPI transport: CS (GP25) as SIO output idle-high, the PIO1 gSPI
    /// SM built with the bus held idle (CLK low, DATA low) and started. Does NOT
    /// power WL_ON — that's cyw43's `pwr` pin, raised inside `cyw43::new` *after*
    /// this exists, so the bus stays idle through the chip's power-up. Construct
    /// BEFORE `cyw43::new`.
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

    /// Push one word to the TX FIFO. Bounded busy-wait so a stalled SM can't wedge
    /// the executor forever — a timeout corrupts the txn (which cyw43's handshake
    /// then catches) rather than hanging hard.
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

    /// Clock the 32-bit `write` cmd out, then read `read.len()` data words plus
    /// the trailing status word. X = 31 (32 cmd bits); Y = (read.len()+1)*32 - 1.
    /// (The backplane's extra leading word is already counted in `read.len()` by
    /// cyw43's caller — see the trait docs.) Matches embassy cyw43-pio.
    async fn cmd_read(&mut self, write: u32, read: &mut [u32]) -> u32 {
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

    // `wait_for_event` uses the default (active-polling) impl for now; the real
    // one waits on the CYW43 IRQ/host-wake line.
}

// =====================================================================
// 2b. cyw43 bring-up via block_on (R13 Step 3 — no executor)
// =====================================================================
//
// We drive the whole bring-up with `embassy_futures::block_on` instead of the
// embassy executor: block_on busy-spins the future, cyw43's `Timer::after`
// delays resolve against the TIMER0 time-driver's `now()`, and our transport's
// cmd_read/cmd_write are synchronous busy-polls — so no async runtime / alarm
// IRQ is needed yet. `cyw43::new()` runs self-contained; `Control::init` + the
// LED blink need the Runner running *concurrently*, so we `select(runner.run(),
// seq)`: select returns when `seq` finishes (Runner then dropped) and block_on
// returns, so the normal 10BASE-T loop continues and reports the stage flags
// over CDC. (A persistent executor + continuous Runner come with R14+.)

/// 1 once `cyw43::new()` returned (firmware + nvram loaded + bus handshake OK).
pub static CYW43_NEW_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once `Control::init(clm)` returned (CLM loaded + WiFi firmware up).
pub static CYW43_INIT_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once at least one onboard-LED toggle has run (`gpio_set` ioctls work); the
/// blink loop re-sets it every cycle, so it staying 1 while `hb` climbs proves
/// the Runner is alive.
pub static CYW43_LED_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once `Control::start_ap_wpa2(...)` returned (R14.2 — AP beaconing).
pub static CYW43_AP_DONE: AtomicU32 = AtomicU32::new(0);
/// 1 once the LAN smoltcp `Interface` is built + polling (R14.3 — data path up).
pub static CYW43_NET_UP: AtomicU32 = AtomicU32::new(0);

/// LAN gateway IP (the Pico's address on the wireless subnet, R14.3).
const LAN_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
const LAN_PREFIX: u8 = 24;

// R14.2 AP parameters (dev defaults for the LAN-side bring-up). WPA2 passphrase
// must be 8..=63 bytes. 2.4 GHz channel 6. These become configurable later.
const AP_SSID: &str = "pico-rp2350-router";
const AP_PASSPHRASE: &str = "picorouter2350";
const AP_CHANNEL: u8 = 6;

/// R13 Step 3 — full cyw43 bring-up via `block_on`: `cyw43::new()` (firmware +
/// nvram over our PIO1 transport) → run the Runner concurrently with
/// `Control::init(clm)` + a few onboard-LED blinks (`gpio_set` ioctls), then
/// return. Stage flags ([`CYW43_NEW_DONE`]/[`CYW43_INIT_DONE`]/[`CYW43_LED_DONE`])
/// are reported by `log_status`. Blocks a few seconds at our 2 MHz gSPI. `spi`
/// must be a fresh [`PioSpiCyw43`] (bus idle); `pwr` is WL_ON (cyw43 power-cycles
/// it during init — bus already idle, gotcha #11). Call once at boot, before the
/// USB/10BASE-T loop. A failure inside cyw43 panics (`.unwrap()`); a chip that
/// never answers an ioctl would hang here (the corresponding flag stays 0).
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
            // Blink the onboard LED (CYW43 GPIO0) — visible proof + exercises
            // gpio_set ioctls through the concurrently-running Runner.
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
// 3. Async runtime entry
// =====================================================================

// Concrete cyw43 handle types over our PIO1 transport — needed to name the
// long-lived `Runner` task's argument (embassy `#[task]`s can't be generic).
type CywBus = cyw43::SpiBus<WlOnPin, PioSpiCyw43>;
type CywRunner = cyw43::Runner<'static, CywBus>;

/// Build the USB stack (CDC telemetry + the picotool vendor reset interface) on
/// a `'static` allocator, so it can move into the executor's `usb_task` and keep
/// being polled while the executor owns core 0. Mirrors `main`'s 10BASE-T USB
/// setup (same VID:PID, chip-ID serial number — so `picotool -f`/`cargo run`
/// keep working). Call once.
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

    // Serial = chip ID, so picotool tracks us across the app→BOOTSEL reboot
    // (see main.rs / gotcha #4). `usb_dev` borrows this string, and we return
    // `usb_dev`, so the serial must be `'static` — stash it in a one-shot static.
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
// 3. Async runtime entry (R14.1 — persistent executor, continuous Runner)
// =====================================================================

/// Write a full byte slice to the CDC, polling the USB device between chunks so
/// the ~128 B `usbd-serial` IN buffer is flushed to the host as it fills. The
/// bare `serial.write()` writes only what currently fits and returns a partial
/// count; ignoring that count silently drops the tail (R16: the longer `[Wan]`
/// line overflowed it, truncating the `w2l`/`sent`/`drop` counters — and it was
/// the long-standing "CDC drops bytes under load" limitation generally). The
/// `guard` bounds the loop so a host that isn't reading (no DTR) can't hang us.
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

/// USB poll loop, in the executor. Keeps CDC + the picotool reset interface
/// serviced while the executor owns core 0 (so `cargo run`/`picotool -f` still
/// reboot us into BOOTSEL), and emits a 1 Hz `[Cyw43]` status line so the cyw43
/// bring-up stages + a live heartbeat are visible over CDC (gotcha #5: the host
/// must assert DTR to see the bytes).
#[embassy_executor::task]
async fn usb_task(
    mut usb_dev: UsbDevice<'static, hal::usb::UsbBus>,
    mut serial: SerialPort<'static, hal::usb::UsbBus>,
    mut reset_iface: crate::pico_reset::PicoResetInterface,
) -> ! {
    let mut n: u32 = 0;
    // [Perf] rate state: previous cumulative counters (for per-second deltas) +
    // the conntrack live high-water. Router build only.
    #[cfg(feature = "router")]
    let (mut prev_to_wan, mut prev_to_lan, mut prev_sent, mut ct_hwm) =
        (0u32, 0u32, 0u32, 0usize);
    loop {
        usb_dev.poll(&mut [&mut serial, &mut reset_iface]);
        // Honor a picotool -f reboot request from clean (non-IRQ) context.
        if let Some(kind) = reset_iface.take_pending_reboot() {
            hal::reboot::reboot(kind, crate::pico_reset::RebootArch::Normal);
        }
        n = n.wrapping_add(1);
        // ~1 Hz at the 1 ms poll cadence.
        if n % 1000 == 0 {
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

            // R15b — the WAN (10BASE-T) side, when the router build is active.
            // wan_task publishes a snapshot we format here, so all CDC output
            // stays on this one task (no serial contention).
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

                // R16 forwarding counters on their own line, so they can't be
                // truncated mid-counter by CDC framing (they live past the point
                // where the combined line used to overflow the IN buffer).
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

                // R17 NAPT conntrack counters.
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

                // [Perf] — per-second routed throughput + saturation/drop diagnostics.
                // up = LAN→WAN (client upload), dn = WAN→LAN (download); rates are the
                // 1 Hz delta of the cumulative byte/frame counters.
                let to_wan = crate::forward::FWD_BYTES_TO_WAN.load(Ordering::Relaxed);
                let to_lan = crate::forward::FWD_BYTES_TO_LAN.load(Ordering::Relaxed);
                let sent = crate::forward::FWD_SENT.load(Ordering::Relaxed);
                let up_kbs = to_wan.wrapping_sub(prev_to_wan) / 1024;
                let dn_kbs = to_lan.wrapping_sub(prev_to_lan) / 1024;
                let pps = sent.wrapping_sub(prev_sent);
                prev_to_wan = to_wan;
                prev_to_lan = to_lan;
                prev_sent = sent;
                let ct = crate::conntrack::live_count();
                if ct > ct_hwm {
                    ct_hwm = ct;
                }
                let mut pline: String<192> = String::new();
                let _ = write!(
                    pline,
                    "[Perf] up={}KB/s dn={}KB/s pps={} qmax={}/{} cthwm={}/{} \
                     drop[qf={} nh={} nat={} txb={} oth={}]\r\n",
                    up_kbs,
                    dn_kbs,
                    pps,
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

/// The cyw43 event loop — drives every gSPI transaction + chip event. Must run
/// continuously for the chip to stay up (this is what R13's `block_on` could not
/// provide once it returned). `runner.run()` itself diverges.
#[embassy_executor::task]
async fn cyw43_runner_task(runner: CywRunner) -> ! {
    runner.run().await
}

/// R14.3 — the LAN side: wrap cyw43's `NetDriver` in a smoltcp `phy::Device`
/// (via [`Cyw43Phy`]) and run the Interface poll loop. Static gateway
/// `192.168.4.1/24`; with smoltcp's `auto-icmp-echo-reply` the Interface answers
/// ARP + ICMP echo with no sockets. R14.3 acceptance: a client statically
/// configured as `192.168.4.2/24` (after joining the AP) pings `192.168.4.1`.
#[embassy_executor::task]
async fn net_task(net: cyw43::NetDriver<'static>, mac: [u8; 6]) -> ! {
    // R16: in the router build, wrap the LAN phy so transit frames (a client's
    // off-LAN traffic) are diverted to the WAN via LAN_TO_WAN; smoltcp still gets
    // ARP / DHCP / mgmt / ICMP-to-us. `accept_dst: None` ⇒ forward any off-local
    // dst. The LAN gateway IP is static, so forwarding is enabled immediately.
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

    // ARP + ICMP echo are answered by the Interface itself; the one socket is
    // the R14.4 DHCP server on UDP :67. Buffers sized for a few BOOTP packets.
    let mut sockets_storage: [SocketStorage; 2] = [SocketStorage::EMPTY; 2];
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

    // R14.5/R18 — a tiny mgmt HTTP server on the LAN gateway IP (192.168.4.1:80).
    // TX sized to hold the whole status page (clients + WAN + NAT + perf) in one send.
    let mut http_rx = [0u8; 1024];
    let mut http_tx = [0u8; 2560];
    let http_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut http_rx[..]),
        tcp::SocketBuffer::new(&mut http_tx[..]),
    );
    let http_handle = sockets.add(http_socket);

    CYW43_NET_UP.store(1, Ordering::Relaxed);
    loop {
        iface.poll(now(), &mut device, &mut sockets);
        // Drain + answer DHCP after the iface has delivered inbound datagrams;
        // the queued replies go out on the next poll.
        dhcp.poll(sockets.get_mut::<udp::Socket>(dhcp_handle));
        serve_status_http(sockets.get_mut::<tcp::Socket>(http_handle), &dhcp);
        // R16: re-emit frames the WAN side forwarded to a LAN client out the cyw43
        // phy (next-hop = the client, MAC from the LAN neighbor table).
        #[cfg(feature = "router")]
        while let Ok(mut frame) = crate::forward::WAN_TO_LAN.try_receive() {
            device.egress(&mut frame, now());
        }
        Timer::after(Duration::from_millis(5)).await;
    }
}

/// R14.5/R18 — a one-shot HTTP/1.0 status page on `192.168.4.1:80`. Re-listens
/// after each closed connection; ignores the request line (every GET gets the
/// same page). Shows the AP + LAN config, the DNS handed out, the connected
/// clients (active DHCP leases), and — in the router build — the WAN link state
/// plus the NAPT/forwarding counters, so a joined client can confirm and inspect
/// the router from the LAN side.
fn serve_status_http(socket: &mut tcp::Socket, dhcp: &DhcpServer) {
    if !socket.is_open() {
        let _ = socket.listen(80);
    }
    if socket.may_recv() {
        let _ = socket.recv(|buf| (buf.len(), ()));
    }
    if socket.can_send() {
        let uptime_s = embassy_time::Instant::now().as_secs();
        let dns = Ipv4Address::from(
            crate::dhcp_server::LAN_DNS_OFFER
                .load(Ordering::Relaxed)
                .to_be_bytes(),
        );
        // Body = header block + one line per possible lease + the WAN/NAT/Perf
        // block, sized from POOL_LEN (~40 B/lease) so it can't silently undersize
        // if the pool grows. head + body stays under the TX buffer; `write!`
        // truncation is a graceful backstop.
        const STATUS_BODY_CAP: usize = 256 + crate::dhcp_server::POOL_LEN * 40 + 512;
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

        // WAN link + NAPT — router build only (the WAN/conntrack subsystems
        // don't exist in the wireless-only image).
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

/// One-shot bring-up: `cyw43::new()` (firmware + nvram over PIO1) → spawn the
/// Runner → `Control::init(clm)` → blink the onboard LED forever. The LED can
/// only keep toggling if the Runner task is continuously servicing `gpio_set`
/// ioctls — so an indefinitely-blinking LED is the R14.1 acceptance signal.
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

    // R14.2 — bring up the AP. No power-save: an AP must stay available for
    // clients (the default PM would let the radio nap and miss beacons/probes).
    control
        .set_power_management(cyw43::PowerManagementMode::None)
        .await;
    control
        .start_ap_wpa2(AP_SSID, AP_PASSPHRASE, AP_CHANNEL)
        .await;
    CYW43_AP_DONE.store(1, Ordering::Relaxed);

    // R14.3 — wire the NetDriver into a smoltcp LAN Interface. Read our MAC for
    // the Interface's hardware address, then hand the NetDriver to net_task. The
    // Runner keeps the chip's RX/TX channel pumped; net_task drains/fills it.
    let mac = control.address().await;
    if let Ok(t) = net_task(net, mac) {
        spawner.spawn(t);
    }

    // Keep blinking the onboard LED forever — our liveness proof that the Runner
    // stays up (AP beaconing + LAN Interface polling alongside it).
    let mut on = false;
    loop {
        on = !on;
        control.gpio_set(0, on).await;
        CYW43_LED_DONE.store(1, Ordering::Relaxed);
        Timer::after(Duration::from_millis(250)).await;
    }
}

/// R14.1 wireless-image entry: set up the USB stack, enable the time-driver
/// alarm IRQ, then hand core 0 to the embassy executor forever. Spawns the USB
/// poll loop + the cyw43 bring-up (which spawns the continuous Runner). Never
/// returns — this is the standalone-wireless build, so 10BASE-T is not started
/// (docs/router-plan.md §11/§12).
///
/// `spi` must be a fresh [`PioSpiCyw43`] (bus idle); `pwr` is WL_ON (cyw43
/// power-cycles it during init — bus already idle, gotcha #11). The caller must
/// not also drive TIMER0 ALARM0 / `TIMER0_IRQ_0` (the time-driver owns them).
pub fn run(
    pwr: WlOnPin,
    spi: PioSpiCyw43,
    usb: hal::pac::USB,
    usb_dpram: hal::pac::USB_DPRAM,
    usb_clock: hal::clocks::UsbClock,
    resets: &mut hal::pac::RESETS,
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

    executor.run(|spawner| {
        // embassy-executor 0.10's `#[task]` macro returns a `Result` (the task
        // arena slot can be exhausted); spawn the startup tasks.
        if let Ok(t) = usb_task(usb_dev, serial, reset_iface) {
            spawner.spawn(t);
        }
        if let Ok(t) = cyw43_bootstrap_task(spawner, pwr, spi) {
            spawner.spawn(t);
        }
    });
}

// =====================================================================
// 4. R15b — the router: both interfaces live under one executor
// =====================================================================
//
// `run_router` is `run` plus a `wan_task` that drives the 10BASE-T (WAN) side
// as a 2nd smoltcp `Interface` over `EthMac`. The 10BT RX IRQ runs on **core 1**
// (launched by `setup_eth_mac` before we hand core 0 to the executor), so the
// long decode never starves the executor; `wan_task` only drains the inbox via
// `EthMac::receive` (a brief `Spinlock<0>`) on its poll cadence. The cyw43 LAN
// (`cyw43_bootstrap_task` → Runner + `net_task`) is unchanged. See
// docs/r15-plan.md §6.

/// WAN-client snapshot published by `wan_task` for `usb_task`'s `[Wan]` line —
/// keeps all CDC output on the one task. `WanState` is `Copy`, so a plain `Cell`
/// suffices under the `critical_section` mutex.
#[cfg(feature = "router")]
static WAN_PUB: Mutex<core::cell::Cell<Option<crate::wan::WanState>>> =
    Mutex::new(core::cell::Cell::new(None));
/// Whether core 1 (the 10BT RX engine) launched — surfaced in the `[Wan]` line.
#[cfg(feature = "router")]
static WAN_CORE1_OK: AtomicU32 = AtomicU32::new(0);

/// R15b WAN task: a 2nd smoltcp `Interface` on `EthMac` (the 10BASE-T phy) that
/// runs the shared `crate::wan` DHCP-client / ping / DNS logic + the NLP
/// keepalive, polled on a 1 ms cadence. Mirrors `main_10bt`'s WAN handling but
/// async, beside the cyw43 LAN. `EthMac` is `'static` (owns its TX state), so it
/// moves into the task cleanly — same shape as `net_task` taking `NetDriver`.
#[cfg(feature = "router")]
#[embassy_executor::task]
async fn wan_task(mut mac: crate::eth_mac::EthMac) -> ! {
    use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
    use smoltcp::phy::Device as _; // bring `capabilities()` into scope
    use smoltcp::socket::{dhcpv4, dns, icmp};
    use smoltcp::wire::{EthernetAddress, HardwareAddress};

    // TX checksum caps for emitting/parsing ICMP echoes (read once; the device
    // computes L3/L4 checksums on egress, no offload).
    let dev_checksum = mac.capabilities().checksum;

    // R16: wrap the 10BT phy so transit frames (replies destined to a LAN client)
    // are diverted to the LAN via WAN_TO_LAN. `accept_dst` restricts forwarding to
    // the LAN subnet; `our_ip`/`subnet`/`gateway` are UNSPECIFIED until the DHCP
    // lease lands (forwarding stays disabled until then — see set_lease below).
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
    // R19: gateway from the last lease — to ARP it the instant it first
    // appears/changes (cold-start pre-warm, see below).
    let mut prev_gw: Option<Ipv4Address> = None;
    loop {
        iface.poll(now(), &mut device, &mut sockets);
        crate::wan::dhcp_apply(&mut iface, &mut sockets, dhcp_handle, dns_handle, &mut wan);
        // R16: keep the forwarder's WAN address/subnet/gateway in step with the
        // lease (this also enables WAN-side forwarding once the lease lands).
        if let Some(cidr) = wan.addr {
            device.set_lease(cidr, wan.gw);
        }
        // R19: the instant the lease first provides a gateway (or it changes), ARP
        // it right away — don't wait for the 1 Hz retry below — so WAN_NEIGH is warm
        // before any client's first forwarded frame can race it.
        if wan.gw.is_some() && wan.gw != prev_gw {
            device.arp_gateway(now());
        }
        prev_gw = wan.gw;
        // R18: hand the WAN-learned upstream resolver to LAN DHCP clients. With
        // R17 NAPT forwarding their port-53 UDP, no DNS relay is needed — they
        // query it directly through our NAT. Until a lease provides one, the
        // 8.8.8.8 default in `LAN_DNS_OFFER` stands.
        if let Some(dns) = wan.dns0 {
            crate::dhcp_server::LAN_DNS_OFFER
                .store(u32::from_be_bytes(dns.octets()), Ordering::Relaxed);
        }
        crate::wan::ping_drain(&mut sockets, icmp_handle, &mut wan, &dev_checksum);
        crate::wan::dns_harvest(&mut sockets, dns_handle, &mut wan);

        let nowt = embassy_time::Instant::now();
        // NLP keepalive every 16 ms — IEEE 802.3 link integrity (gotcha #9).
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
                // R19: pre-warm the WAN gateway's MAC so the first forwarded
                // LAN→WAN frame isn't dropped while WAN_NEIGH is still empty (the
                // cold-start `[Fwd] drop`). Re-ARP each second until the passive
                // learner has it — robust to a reply lost on the half-duplex wire
                // — then stop (no steady-state spam).
                if let Some(gw) = wan.gw {
                    if !crate::forward::wan_neigh_known(gw) {
                        device.arp_gateway(now());
                    }
                }
            }
            // R17: sweep idle NAPT conntrack entries once a second.
            crate::forward::nat_reap(now().total_millis().max(0) as u64);
        }
        // R16: re-emit frames the LAN side forwarded toward the WAN out the 10BT
        // phy (next-hop = the WAN gateway for off-subnet dsts, MAC from the WAN
        // neighbor table; EthMac TX adds FCS/IFG/CSMA).
        while let Ok(mut frame) = crate::forward::LAN_TO_WAN.try_receive() {
            device.egress(&mut frame, now());
        }
        // Publish a snapshot for usb_task's [Wan] line.
        critical_section::with(|cs| WAN_PUB.borrow(cs).set(Some(wan)));

        Timer::after(Duration::from_millis(1)).await;
    }
}

/// R15b entry: `run` + the WAN task. Core 1 (10BT RX decode) must already be
/// launched by `setup_eth_mac` before this is called (it seizes core 0). Spawns
/// the USB poll/telemetry task, the cyw43 bring-up (→ Runner + LAN `net_task`),
/// and `wan_task(mac)`. Never returns.
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

    executor.run(|spawner| {
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
