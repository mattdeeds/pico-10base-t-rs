# CYW43439 firmware blobs (vendored)

Binary blobs for the CYW43439 wireless chip on the Pico 2 W, vendored from the
[embassy](https://github.com/embassy-rs/embassy) project's `cyw43-firmware/`
directory (which in turn sources them from
<https://github.com/georgerobotics/cyw43-driver/tree/main/firmware>).

| File | Purpose |
|---|---|
| `43439A0.bin` | WiFi firmware (231077 B) |
| `nvram_rp2040.bin` | Board NVRAM for the Raspberry Pi Pico-W-family module (used on the Pico 2 W) |
| `43439A0_clm.bin` | CLM (country/regulatory) blob — loaded by `Control::init` |

Loaded 4-byte-aligned via `cyw43::aligned_bytes!`. See `src/wireless.rs`
(`cyw43_new_blocking`) and `docs/router-plan.md` §11. Licensed under the
Infineon permissive binary license — see the `LICENSE-*.txt` here.
