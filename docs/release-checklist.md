# Release / open-source prep checklist

**Progress:** Phase 1 ✅ (`b2c8d99`) · Phase 2 ✅ (`f910d0b`) · Phase 3 ✅ (README +
`docs/README.md` index; RESUME.md kept as dev notes) · Phase 4 (tooling) next ·
Phase 5 (publish: fresh-clone build, tag v0.1.0, flip public).

Wrapping the project for public sharing (2026-06-03). Decisions locked:

- **Structure:** one binary, feature-gated — default build = standalone software
  10BASE-T NIC; `--features router` = full wireless router on top. No crate split
  (documented layering instead; a future "extract the PHY as a crate" is optional).
- **Performance:** fresh on-device re-run of the 10BASE-T headline numbers →
  `docs/performance.md` + a README table; cite the established cyw43-LAN / routed
  numbers with dates/conditions.
- **License:** dual **MIT OR Apache-2.0** (Rust standard). Upstream
  kingyoPiyo/Pico-10BASE-T is **MIT, Copyright (c) 2022 kingyo** — compatible;
  preserve its notice (this is a port).
- **Dev docs:** keep all 14 (the engineering log is a selling point) + add an index.

## Phase 1 — legal / safety (blocking)
- [ ] `LICENSE-MIT` (our copyright + upstream kingyo attribution) + `LICENSE-APACHE`.
- [ ] `Cargo.toml` metadata: license, repository, authors, keywords, categories, readme.
- [ ] **Scrub hardcoded AP creds** (`wireless.rs` `AP_SSID`/`AP_PASSPHRASE`) → obvious
      placeholders + "change me" warning. (Was a real WPA2 passphrase.)
- [ ] cyw43 blobs: confirm the Infineon permissive-binary license stays + is referenced
      (already in `cyw43-firmware/`).
- [ ] Credits: kingyoPiyo/Pico-10BASE-T (port source) + Niccle (reference) in README.

## Phase 2 — final performance characterization
- [ ] Fresh on-device (10BT NIC, the centerpiece): TX, RX-bulk, small/idle, full-MTU
      FCS reliability, latency, CPU. (Current host↔Pico-10BT rig.)
- [ ] Cite (recent committed runs): cyw43 LAN down/up (909/716, §3.5), routed/NAPT,
      the full-duplex finding, the decode/PHY ceiling, the watchdog.
- [ ] One digestible `docs/performance.md` ("what to expect") + README summary table +
      honest caveats + how-to-reproduce.

## Phase 3 — README + docs index
- [ ] Rewrite `README.md`: what it is → the two layers → hardware + wiring → build/flash
      + toolchain → performance summary → architecture overview → limitations → credits
      → license.
- [ ] `docs/README.md` index framing the 14 dev-log docs as "how we got here"; point to
      the key ones (architecture, performance, full-duplex, rx-bulk-ceiling).
- [ ] Decide RESUME.md fate (internal session-pointer — keep as dev notes or tuck).

## Phase 4 — tooling tidy
- [ ] Commit the `/tmp` measurement scripts that matter (`cdc_read.py`, the upload/blast
      helpers) into `tools/` with a `tools/README.md` so the characterization reproduces.
- [ ] Keep `full-duplex` / `fd-bench` / `mss-clamp` features (off by default), documented
      as "measurement/experiment features."
- [ ] Optional: CI (GitHub Actions) building all 4 feature configs.

## Phase 5 — publish
- [ ] Fresh-clone build test (toolchain instructions actually work).
- [ ] Tag `v0.1.0`.
- [ ] Flip the repo public (owner action).
