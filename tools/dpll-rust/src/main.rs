//! Phase 3b — host-side offline validator for the Rust port of
//! `decode_edge_track`. Loads `tools/clock-recovery/corpus/*.bin`, runs the
//! Rust decoder, and scores FCS-OK + per-byte payload error bins. Pass =
//! every corpus frame decodes byte-perfect (matching `harness.py`'s
//! Python reference).
//!
//! The decoder source itself is shared with the firmware via `#[path]` —
//! one source of truth, no copy-paste drift.

// Pull the firmware's edge-track decoder module in by path. This file (the
// host bin's main.rs) is at tools/dpll-rust/src/main.rs; the firmware module
// is at src/eth_rx_dpll.rs — three `..` to get there.
#[path = "../../../src/eth_rx_dpll.rs"]
mod eth_rx_dpll;

use std::env;
use std::fs;
use std::path::PathBuf;

const POLY_REVERSED: u32 = 0xEDB8_8320;

fn crc32_ieee802_3(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { (c >> 1) ^ POLY_REVERSED } else { c >> 1 };
        }
    }
    c ^ 0xFFFF_FFFF
}

fn fcs_ok(frame: &[u8]) -> bool {
    if frame.len() < 18 {
        return false;
    }
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    let frame_len = if ethertype == 0x0800 {
        let ip_total = u16::from_be_bytes([frame[16], frame[17]]) as usize;
        (14 + ip_total + 4).max(64)
    } else {
        frame.len()
    };
    if frame_len < 18 || frame_len > frame.len() {
        return false;
    }
    let computed = crc32_ieee802_3(&frame[..frame_len - 4]);
    let stored = u32::from_le_bytes([
        frame[frame_len - 4],
        frame[frame_len - 3],
        frame[frame_len - 2],
        frame[frame_len - 1],
    ]);
    computed == stored
}

fn main() {
    // Default corpus path is relative to this bin's package root.
    let corpus_dir = env::var("CORPUS_DIR").unwrap_or_else(|_| {
        // tools/dpll-rust → tools/clock-recovery/corpus
        let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        p.pop(); // tools/dpll-rust → tools/
        p.push("clock-recovery/corpus");
        p.to_string_lossy().into_owned()
    });

    let mut files: Vec<PathBuf> = match fs::read_dir(&corpus_dir) {
        Ok(rd) => rd
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().and_then(|x| x.to_str()) == Some("bin"))
            .collect(),
        Err(e) => {
            eprintln!("can't read corpus dir {}: {}", corpus_dir, e);
            std::process::exit(2);
        }
    };
    files.sort();
    if files.is_empty() {
        eprintln!("no .bin files in {}", corpus_dir);
        std::process::exit(2);
    }

    const BIN_SIZE: usize = 184;
    let mut total = 0;
    let mut ok = 0;
    let mut bins_err = [0u32; 8];
    let mut bins_tot = [0u32; 8];

    println!("Phase 3b — Rust decode_edge_track vs. corpus at {}", corpus_dir);
    for path in &files {
        let raw = fs::read(path).expect("can't read corpus file");
        total += 1;
        let name = path.file_name().unwrap().to_string_lossy();
        match eth_rx_dpll::decode_frame_edge_track(&raw) {
            None => println!("  {}: NO SFD", name),
            Some(frame) => {
                let passes = fcs_ok(&frame);
                if passes {
                    ok += 1;
                    // Score payload error bins; payload[j] = j & 0xFF
                    if frame.len() >= 46 {
                        let plen = (frame.len() - 42 - 4).min(1472);
                        for j in 0..plen {
                            let b = (j / BIN_SIZE).min(7);
                            bins_tot[b] += 1;
                            if frame[42 + j] != (j & 0xFF) as u8 {
                                bins_err[b] += 1;
                            }
                        }
                    }
                    println!("  {}: len {} FCS OK", name, frame.len());
                } else {
                    println!("  {}: len {} FCS FAIL", name, frame.len());
                }
            }
        }
    }

    println!("\n=== Rust decode_edge_track ===  FCS-ok {}/{}", ok, total);
    for b in 0..8 {
        let lo = 42 + b * BIN_SIZE;
        let r = if bins_tot[b] > 0 {
            100.0 * bins_err[b] as f64 / bins_tot[b] as f64
        } else {
            0.0
        };
        println!(
            "  bin {} frame-bytes {:>4}-{:<4} {:6.1}%  {}",
            b,
            lo,
            lo + BIN_SIZE - 1,
            r,
            "#".repeat((r / 2.0) as usize)
        );
    }

    let pass = ok == total && total > 0 && bins_err.iter().all(|&e| e == 0);
    if pass {
        println!("\nPASS: all {} corpus frames decoded byte-perfect, flat 0% bins", total);
        std::process::exit(0);
    } else {
        println!("\nFAIL: {}/{} FCS-ok, bins not all 0%", ok, total);
        std::process::exit(1);
    }
}
