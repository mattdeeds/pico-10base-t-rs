//! pico-sdk-compatible USB "reset interface" so `picotool -f` can
//! self-reboot us into BOOTSEL.
//!
//! Picotool's force-reboot flow looks for a vendor-specific USB
//! interface (class=0xFF, sub=0x00, proto=0x01) with no endpoints and
//! sends a control transfer to bRequest=0x01 (`RESET_REQUEST_BOOTSEL`).
//! The pico-sdk's `stdio_usb` library exposes exactly that interface
//! and reboots into BOOTSEL when it sees the request. We mirror it here
//! so `picotool load -fux -t elf <elf>` works without the manual BOOTSEL
//! / OpenOCD fallback that R0–R8 needed (see RESUME.md gotcha #4).
//!
//! Definitions are sourced from pico-sdk's `pico/usb_reset_interface.h`.

pub use rp235x_hal::reboot::{RebootArch, RebootKind};
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, RequestType};

/// Vendor subclass for the reset interface (matches pico-sdk).
const RESET_INTERFACE_SUBCLASS: u8 = 0x00;
/// Vendor protocol for the reset interface (matches pico-sdk).
const RESET_INTERFACE_PROTOCOL: u8 = 0x01;

/// Reboot into BOOTSEL. Triggered by `picotool -f`.
const RESET_REQUEST_BOOTSEL: u8 = 0x01;
/// Regular app reboot (rare; picotool uses this after flashing
/// with `--no-reboot` cleared).
const RESET_REQUEST_FLASH: u8 = 0x02;

pub struct PicoResetInterface {
    iface: InterfaceNumber,
    /// If `Some`, the main loop will reboot the chip with this kind on
    /// the next iteration. Set by `control_out`; cleared (and acted on)
    /// by `take_pending_reboot`. Deferring rather than rebooting from
    /// inside `control_out` lets `usb_dev.poll()` complete the STATUS
    /// stage of the SETUP transaction cleanly before the reset fires.
    pending: Option<RebootKind>,
}

impl PicoResetInterface {
    pub fn new<B: UsbBus>(alloc: &UsbBusAllocator<B>) -> Self {
        Self {
            iface: alloc.interface(),
            pending: None,
        }
    }

    /// If a USB control transfer requested a reboot, return its kind.
    /// Main is expected to call this after every `usb_dev.poll()` and
    /// `reboot(kind, RebootArch::Normal)` when it returns `Some`.
    pub fn take_pending_reboot(&mut self) -> Option<RebootKind> {
        self.pending.take()
    }
}

impl<B: UsbBus> UsbClass<B> for PicoResetInterface {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        writer.interface(
            self.iface,
            0xFF, // vendor-specific class
            RESET_INTERFACE_SUBCLASS,
            RESET_INTERFACE_PROTOCOL,
        )?;
        Ok(())
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        // Accept both Class and Vendor request types. Picotool actually
        // sends a Class-type request (bmRequestType=0x21) even though the
        // pico-sdk reset interface is declared with interface class=0xFF
        // (vendor) — TinyUSB's vendor driver dispatches both, so the SDK
        // happens to work either way; usb-device routes more strictly,
        // so we have to explicitly accept Class too.
        let req_type_ok =
            req.request_type == RequestType::Class || req.request_type == RequestType::Vendor;
        if !req_type_ok
            || req.recipient != Recipient::Interface
            || req.index as u8 != u8::from(self.iface)
        {
            return;
        }
        match req.request {
            RESET_REQUEST_BOOTSEL => {
                let _ = xfer.accept();
                self.pending = Some(RebootKind::BootSel {
                    picoboot_disabled: false,
                    msd_disabled: false,
                });
            }
            RESET_REQUEST_FLASH => {
                let _ = xfer.accept();
                self.pending = Some(RebootKind::Normal);
            }
            _ => {}
        }
    }
}
