//! smoltcp `phy::Device` over cyw43's async `NetDriver`.
//!
//! Calls the poll-style `Driver::receive`/`transmit` with a no-op waker.
//! The net task re-polls in a loop, so wakeups aren't needed.

use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, RawWaker, RawWakerVTable, Waker};

use cyw43::NetDriver;
use smoltcp::phy::{Device, DeviceCapabilities, Medium, RxToken, TxToken};
use smoltcp::time::Instant;

/// cyw43 frame MTU (L2, including the 14-byte Ethernet header).
const CYW43_MTU: usize = 1514;

/// No-op waker. Hand-rolled: `Waker::noop()` needs Rust 1.85 (MSRV 1.82).
fn noop_waker() -> Waker {
    const fn raw() -> RawWaker {
        fn no_op(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            raw()
        }
        RawWaker::new(
            core::ptr::null(),
            &RawWakerVTable::new(clone, no_op, no_op, no_op),
        )
    }
    // Safety: the vtable's fns are all no-ops over a null data pointer.
    unsafe { Waker::from_raw(raw()) }
}

/// Frames handed to smoltcp from the LAN (`[Cyw43]` line).
pub static CYW43_RX_FRAMES: AtomicU32 = AtomicU32::new(0);

/// Times `transmit()` found no free cyw43 TX buffer (backpressure).
/// cyw43 drops RX silently inside its Runner, so there's no RX-drop counter.
pub static CYW43_TX_BUSY: AtomicU32 = AtomicU32::new(0);

// cyw43 token types, named without depending on embassy-net-driver-channel.
type NetRx<'a> = <NetDriver<'static> as embassy_net_driver::Driver>::RxToken<'a>;
type NetTx<'a> = <NetDriver<'static> as embassy_net_driver::Driver>::TxToken<'a>;

/// Wraps cyw43's `NetDriver` as a smoltcp `phy::Device` for the wireless LAN.
pub struct Cyw43Phy {
    net: NetDriver<'static>,
}

impl Cyw43Phy {
    pub fn new(net: NetDriver<'static>) -> Self {
        Self { net }
    }

    /// True once cyw43 reports the link up (associated, WPA handshake done).
    pub fn link_up(&mut self) -> bool {
        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);
        embassy_net_driver::Driver::link_state(&mut self.net, &mut cx)
            == embassy_net_driver::LinkState::Up
    }
}

impl Device for Cyw43Phy {
    type RxToken<'a>
        = Cyw43RxToken<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = Cyw43TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, _ts: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);
        embassy_net_driver::Driver::receive(&mut self.net, &mut cx).map(|(rx, tx)| {
            CYW43_RX_FRAMES.fetch_add(1, Ordering::Relaxed);
            (Cyw43RxToken(rx), Cyw43TxToken(tx))
        })
    }

    fn transmit(&mut self, _ts: Instant) -> Option<Self::TxToken<'_>> {
        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);
        match embassy_net_driver::Driver::transmit(&mut self.net, &mut cx) {
            Some(tx) => Some(Cyw43TxToken(tx)),
            None => {
                // TX channel full: count backpressure; smoltcp retries.
                CYW43_TX_BUSY.fetch_add(1, Ordering::Relaxed);
                None
            }
        }
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = CYW43_MTU;
        caps
    }
}

/// RX token wrapping cyw43's.
pub struct Cyw43RxToken<'a>(NetRx<'a>);

impl RxToken for Cyw43RxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        embassy_net_driver::RxToken::consume(self.0, |buf| f(buf))
    }
}

/// TX token wrapping cyw43's.
pub struct Cyw43TxToken<'a>(NetTx<'a>);

impl TxToken for Cyw43TxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        embassy_net_driver::TxToken::consume(self.0, len, f)
    }
}
