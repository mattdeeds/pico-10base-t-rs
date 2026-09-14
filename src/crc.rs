//! CRC-32 (IEEE 802.3) for Ethernet FCS.
//!
//! Reflected poly 0xEDB88320, init/xor-out 0xFFFFFFFF, sent little-endian.

/// CRC-32 of `data`. Bitwise; fast enough at our frame rates.
pub fn crc32_ieee802_3(data: &[u8]) -> u32 {
    finalize(update(0xFFFF_FFFF, data))
}

/// CRC-32 of `data` followed by `pad_len` zero bytes.
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
