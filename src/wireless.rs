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
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex;
use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use rp235x_hal as hal;

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
// TODO(R13 on-device): port the gSPI PIO program (ref: pico-sdk
// `cyw43_bus_pio_spi.c` + embassy `cyw43-pio`) and fill in cmd_write/cmd_read
// with real FIFO push/pull + DMA. The bodies below are placeholders that
// type-check the wiring against `cyw43::new`.

/// Our PIO1-based half-duplex SPI transport for the CYW43439.
// Not yet wired into `cyw43::new` — that's the R13 on-board step (needs the
// real gSPI PIO program + the firmware blobs + the board).
#[allow(dead_code)]
pub struct PioSpiCyw43 {
    // TODO(R13): hold the PIO1 SM handle, the DATA/CLK pins, the DMA channel,
    // and the CS pin here. Left empty for the compile-only skeleton so we don't
    // commit to peripheral types before the on-board bring-up.
    _private: (),
}

#[allow(dead_code)]
impl PioSpiCyw43 {
    /// Placeholder constructor — the real one will take the PIO1 SM, the gSPI
    /// pins (DATA/CLK/CS), and a DMA channel.
    pub fn new_placeholder() -> Self {
        Self { _private: () }
    }
}

impl cyw43::SpiBusCyw43 for PioSpiCyw43 {
    async fn cmd_write(&mut self, _write: &[u32]) -> u32 {
        // TODO(R13): drive CS low, clock out `write` MSB-first on PIO1, read the
        // status word back, release CS. Returns the bus status word.
        0
    }

    async fn cmd_read(&mut self, _write: u32, _read: &mut [u32]) -> u32 {
        // TODO(R13): clock out the 32-bit cmd, then clock `read.len()` words in
        // (backplane reads have one extra leading word — see the trait docs).
        0
    }

    // `wait_for_event` uses the default (active-polling) impl for now; the real
    // one waits on the CYW43 IRQ/host-wake line.
}

// =====================================================================
// 3. Async runtime entry
// =====================================================================

/// Heartbeat task — exercises `embassy-time` (and therefore the time driver
/// above) so the whole async/time stack is link-checked. On hardware this
/// would toggle the CYW43's onboard LED via `Control::gpio_set`.
#[embassy_executor::task]
async fn heartbeat() {
    let mut n: u32 = 0;
    loop {
        Timer::after(Duration::from_millis(500)).await;
        n = n.wrapping_add(1);
        core::hint::black_box(n); // keep the loop from being optimised away
    }
}

/// Run the wireless stack on this core. Sets up the time-driver alarm IRQ and
/// hands control to the embassy executor (never returns).
///
/// # Safety
/// Call once, from a single core, after clocks + TIMER0 are running. The caller
/// must not also be using TIMER0 ALARM0 / `TIMER0_IRQ_0`.
// Async-runtime entry — validated-compiling future infra for the async cyw43
// path; the current R13 milestone uses the sync `probe_cyw43` above instead.
#[allow(dead_code)]
pub unsafe fn run_executor() -> ! {
    // Let the time-driver's ALARM0 fire on this core.
    hal::arch::interrupt_unmask(ALARM_IRQ);
    hal::arch::interrupt_enable();

    // The executor must live for 'static; stash it in a one-shot static.
    static mut EXECUTOR: core::mem::MaybeUninit<Executor> = core::mem::MaybeUninit::uninit();
    let executor = {
        let p = core::ptr::addr_of_mut!(EXECUTOR);
        (*p).write(Executor::new());
        &mut *(*p).as_mut_ptr()
    };

    executor.run(|spawner| {
        // embassy-executor 0.10's `#[task]` macro returns a `Result` (the task
        // arena slot can be exhausted); unwrap the one-shot startup spawn.
        if let Ok(token) = heartbeat() {
            spawner.spawn(token);
        }
        // TODO(R13 on-device): build PioSpiCyw43 + PWR pin, load the CYW43
        // firmware blobs (cyw43-firmware), call cyw43::new(...) to get
        // (NetDriver, Control, Runner); spawn Runner::run(); Control::init() +
        // start_ap_wpa2(); wrap NetDriver in a smoltcp Interface for the LAN.
    });
}
