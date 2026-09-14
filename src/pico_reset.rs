//! pico-sdk USB reset interface so `picotool -f` can reboot to BOOTSEL.
//!
//! Vendor interface (class 0xFF, sub 0x00, proto 0x01) with no endpoints.
//! Mirrors pico-sdk's `pico/usb_reset_interface.h`.

pub use rp235x_hal::reboot::{RebootArch, RebootKind};
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, RequestType};

/// Vendor subclass for the reset interface (matches pico-sdk).
const RESET_INTERFACE_SUBCLASS: u8 = 0x00;
/// Vendor protocol for the reset interface (matches pico-sdk).
const RESET_INTERFACE_PROTOCOL: u8 = 0x01;

/// Reboot into BOOTSEL. Triggered by `picotool -f`.
const RESET_REQUEST_BOOTSEL: u8 = 0x01;
/// Normal app reboot.
const RESET_REQUEST_FLASH: u8 = 0x02;

pub struct PicoResetInterface {
    iface: InterfaceNumber,
    /// Reboot for the main loop to perform.
    /// Deferred so `usb_dev.poll()` finishes the STATUS stage first.
    pending: Option<RebootKind>,
}

impl PicoResetInterface {
    pub fn new<B: UsbBus>(alloc: &UsbBusAllocator<B>) -> Self {
        Self {
            iface: alloc.interface(),
            pending: None,
        }
    }

    /// Take a requested reboot. Call after each `usb_dev.poll()`.
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
        // picotool sends Class requests to this vendor interface; accept both.
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
