//! CRC-32 / IEEE 802.3, used by TX (FCS generation) and RX (FCS verify).
//!
//! Reflected polynomial 0xEDB88320, init 0xFFFFFFFF, xor-out 0xFFFFFFFF.
//! Transmitted little-endian on the wire (standard Ethernet FCS layout).

/// Compute the CRC-32/IEEE-802.3 of `data`. Bit-by-bit implementation —
/// no lookup table needed: at our frame rates (<1 K frame/sec) the CPU
/// cost is negligible (≈100 µs/sec at 150 MHz for typical Ethernet sizes).
pub fn crc32_ieee802_3(data: &[u8]) -> u32 {
    finalize(update(0xFFFF_FFFF, data))
}

/// Compute the CRC over `data` followed by `pad_len` zero bytes. Saves
/// allocating a padded buffer just to hand it to the CRC routine.
pub fn crc32_ieee802_3_padded(data: &[u8], pad_len: usize) -> u32 {
    let mut crc = update(0xFFFF_FFFF, data);
    for _ in 0..pad_len {
        crc = step_byte(crc, 0);
    }
    finalize(crc)
}

#[inline]
fn step_byte(mut crc: u32, b: u8) -> u32 {
    crc ^= b as u32;
    for _ in 0..8 {
        crc = if crc & 1 != 0 {
            (crc >> 1) ^ 0xEDB88320
        } else {
            crc >> 1
        };
    }
    crc
}

#[inline]
fn update(mut crc: u32, data: &[u8]) -> u32 {
    for &b in data.iter() {
        crc = step_byte(crc, b);
    }
    crc
}

#[inline]
fn finalize(crc: u32) -> u32 {
    crc ^ 0xFFFF_FFFF
}
