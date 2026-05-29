//! smoltcp `phy::Device` adapter over cyw43's `NetDriver` (R14.3).
//!
//! cyw43 hands us `NetDriver = embassy_net_driver_channel::Device`, which only
//! implements the *async, waker-based* `embassy_net_driver::Driver` trait — NOT
//! smoltcp's synchronous `phy::Device`. (The sync `try_rx_buf`/`try_tx_buf`
//! buffer API lives on the *producer-side* `ch::Runner`, which cyw43's `Runner`
//! owns internally and never exposes — so it's not reachable from the
//! `NetDriver` we get. This corrects router-plan §12.1, which assumed otherwise.)
//!
//! The bridge: call the async `Driver::receive`/`transmit` with a **no-op-waker
//! `Context`**. Those methods are poll-style — `Some(tokens)` when a frame is
//! ready / a TX slot is free, else `None` after registering the waker (which we
//! discard) — which is exactly smoltcp's synchronous `phy::Device` contract.
//! Our net task polls `iface.poll` in a loop, so discarding the waker is fine.
//! No `embassy-net` dependency; one smoltcp stack serves both interfaces.

use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, RawWaker, RawWakerVTable, Waker};

use cyw43::NetDriver;
use smoltcp::phy::{Device, DeviceCapabilities, Medium, RxToken, TxToken};
use smoltcp::time::Instant;

/// cyw43 frame MTU (L2, including the 14-byte Ethernet header).
const CYW43_MTU: usize = 1514;

/// A no-op `Waker` — `Driver::receive`/`transmit` register it when they return
/// `None`, but we never need waking (the net task re-polls on its own cadence).
/// Hand-rolled rather than `Waker::noop()` to keep the 1.82 MSRV (that's 1.85+).
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

/// Count of frames handed up to smoltcp from the LAN (ARP/ICMP/etc.). Climbs
/// when a joined client sends traffic — the device-side signal that the data
/// path through cyw43 + this adapter works (reported in the `[Cyw43]` line).
pub static CYW43_RX_FRAMES: AtomicU32 = AtomicU32::new(0);

// The cyw43 `NetDriver`'s own token types, named via the trait projection so we
// don't have to depend on `embassy-net-driver-channel` directly.
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
        embassy_net_driver::Driver::transmit(&mut self.net, &mut cx).map(Cyw43TxToken)
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = CYW43_MTU;
        caps
    }
}

/// RX token — delegates to the cyw43 `NetDriver`'s RX token. embassy exposes the
/// received frame as `&mut [u8]`; smoltcp's `RxToken` only needs `&[u8]`.
pub struct Cyw43RxToken<'a>(NetRx<'a>);

impl RxToken for Cyw43RxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        embassy_net_driver::RxToken::consume(self.0, |buf| f(buf))
    }
}

/// TX token — delegates to the cyw43 `NetDriver`'s TX token. Both sides hand the
/// caller a `&mut [u8]` of `len` bytes to fill with a complete Ethernet frame.
pub struct Cyw43TxToken<'a>(NetTx<'a>);

impl TxToken for Cyw43TxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        embassy_net_driver::TxToken::consume(self.0, len, f)
    }
}
