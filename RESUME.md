# pico-10base-t-rs — Resume

Checkpoint for picking up the Rust port of [Pico-10BASE-T](../Pico-10BASE-T/) after a break. Targets the **Hazard3 RISC-V** cores of the RP2350 (Pico 2) with `rp235x-hal`. Same external hardware as the C repo — ISL3177E + HR911105A + AC-coupling caps + 50 Ω source termination.

For the C reference and the proven Manchester / decoder design, see [`../Pico-10BASE-T/RESUME.md`](../Pico-10BASE-T/RESUME.md) and [`../Pico-10BASE-T/CLAUDE.md`](../Pico-10BASE-T/CLAUDE.md).

## 👉 Next session — start here: R15 — WAN as DHCP client (10BASE-T → upstream IP/route/DNS via smoltcp's `dhcpv4` socket). **R14 "LAN up" is COMPLETE & on-device-validated (all of R14.1–R14.5): a client joins the WPA2 AP, auto-gets a DHCP lease (`192.168.4.10`), reaches the gateway, and loads the `192.168.4.1:80` status page.** R15 is the first step with BOTH interfaces live, so it forces the **executor-⊥-10BT runtime unification** deferred from R14 (§11): 10BT RX IRQ on core 1 while the embassy executor owns core 0. NAPT still stays R17. **Step plan: [`docs/router-plan.md`](docs/router-plan.md) §7 + §12.**

**⚠️ The board currently runs the `--features wireless` image** (flashed 2026-05-29 for R14.1 validation), NOT the 10BT production build. To restore 10BT: `cargo run --release` (default build). To keep iterating on wireless: `cargo run --release --features wireless`.

**⚠️ You are on branch `r13-wireless-scaffold`** (R13 wireless work; `main` = the merged R12e production baseline, untouched).

**✅ R13 COMPLETE (2026-05-28) — the cyw43 wireless stack inits end-to-end over our own PIO transport on Hazard3/RISC-V, no embassy-rp.** On-device: **`[Cyw43] new=1 init=1 led=1`** — `cyw43::new()` (231 KB firmware + nvram over PIO1) + `Control::init(clm)` + onboard-LED blink, all gated by `--features wireless`. The journey: (1) board verified good via stock MicroPython (12-AP scan); (2) PIO1 gSPI transport → `0xFEEDBEAD` — the **power-up bus-idle ordering** (gotcha #11) + matching embassy `cyw43-pio` 0.7.0's program (sample DATA on CLK-high, `nop side 0` turnaround); (3) async `SpiBusCyw43` impl (`PioSpiCyw43`); (4) **`cyw43_bringup_blocking` drives the whole thing via `embassy_futures::block_on` + `select(runner.run(), init+blink)`** — so **no persistent executor / async-USB telemetry was needed** (observed via the existing 10BT CDC). Blobs vendored in `cyw43-firmware/` (fw + `nvram_rp2040.bin` + clm). Pin map (pico-sdk `pico2_w.h`): WL_ON=23/DATA=24/CS=25/CLK=29.

**✅ R14.1 COMPLETE (2026-05-29) — persistent embassy executor + continuous cyw43 `Runner` on Hazard3, validated on-device (commit `6f998fa`).** Graduated R13's `block_on(select(runner, seq))` (which *returned* after 6 blinks) to the executor owning core 0 forever: `wireless::run()` spawns `Runner::run()` as a long-lived task (chip stays up), a USB poll task (CDC telemetry + picotool reset, so `cargo run` still reflashes while the executor owns the core), and a bootstrap task (`cyw43::new` → spawn Runner → `Control::init` → blink the onboard LED *forever*). `main()` now does shared clock/pin setup then dispatches; the 10BT path moved verbatim into `#[cfg(not(wireless))] fn main_10bt(...)`. **On-device: `[Cyw43] new=1 init=1 led=1` with `hb` climbing 1→28+ over CDC** — executor + continuous Runner + USB all alive. **Retires the project's #1 deferred risk** (async runtime on Hazard3 at *runtime*). LED relocated to `Control::gpio_set(0,…)` (GP25 is WL_CS).

**✅ R14.2 COMPLETE (2026-05-29) — WPA2 AP up (commit `c7ad6ae`).** The bootstrap task now does `set_power_management(None)` (an AP must stay awake) + `start_ap_wpa2("pico-rp2350-router", "picorouter2350", channel 6)` after `Control::init`. **On-device `[Cyw43] … ap=1` (hb climbing); SSID `pico-rp2350-router` confirmed visible on a phone.** Dev creds + channel in `wireless.rs` (`AP_SSID`/`AP_PASSPHRASE`/`AP_CHANNEL`).

**✅ R14.3 COMPLETE (2026-05-29) — `NetDriver` → smoltcp `phy::Device` adapter + LAN `Interface` (commit `5039a11`).** New `src/cyw43_phy.rs`: `Cyw43Phy` bridges cyw43's `NetDriver` — which is an **async `embassy_net_driver::Driver`, NOT a sync smoltcp plug** — to smoltcp `phy::Device` via a **no-op-waker `Context`** (poll-style `Some`/`None` == smoltcp's sync contract; our `net_task` re-polls). **This corrected §12.1's wrong assumption** that the sync `try_rx_buf`/`borrow_split` API was reachable — those live on the producer-side `ch::Runner` cyw43 owns internally. `net_task` builds a 2nd smoltcp `Interface` (`192.168.4.1/24`, MAC from `Control::address()`); ARP + auto-icmp-echo-reply answer with no sockets. **On-device: an RTL88x2bu Wi-Fi client on the host joined the AP (static `192.168.4.2/24`) and pinged `192.168.4.1` 5/5 ~6 ms; device `rx` counter climbed in step — data path proven both directions.** New optional dep `embassy-net-driver` 0.2.0. (Host setup, one-time: `firmware-realtek` + `iw` + `wpasupplicant` for the RTL88x2bu — the rtw88 driver was loaded but its firmware blob was missing.) NB still no DHCP — clients need a static IP until R14.4.

**✅ R14.4 COMPLETE (2026-05-29) — LAN DHCP server (commit `364281f`).** New `src/dhcp_server.rs`: a smoltcp UDP `:67` socket in `net_task` reuses smoltcp's `DhcpRepr`/`DhcpPacket` wire codec (wireless-only `proto-dhcpv4` feature) — DISCOVER→OFFER, REQUEST→ACK; yiaddr from a fixed MAC-keyed pool `192.168.4.10..=.41`, router + server-id = `192.168.4.1`, /24 mask, 1 h lease; broadcast to `255.255.255.255:68`. **No DNS option** (deferred to R18; also dodges smoltcp's heapless-0.9 `dns_servers` Vec vs our 0.8). `DHCP_TX` counter → `dhcp=` in the `[Cyw43]` line. **On-device: host RTL88x2bu client `DHCPACK of 192.168.4.10 from 192.168.4.1`; device `dhcp` climbed.** ⚠️ **Validation gotcha (§12.3 risk 6):** the DHCP-handed gateway hijacks a multi-homed test host's default route → kills its SSH/internet (the Pico can't forward yet). Test with `dhclient -v -1 -sf /bin/true <wlan>` (proves the exchange, configures nothing), then `ip addr add` the leased IP for the ping. **⇒ R14 "LAN up" is functionally DONE: join the WPA2 AP → auto-lease → reach the gateway.**

**✅ R14.5 COMPLETE (2026-05-29) — LAN mgmt HTTP status page (commit `a745875`).** `net_task` gained a TCP `:80` socket + `serve_status_http` (one-shot HTTP/1.0, re-listens per connection) showing AP SSID, LAN gateway, uptime, and the live DHCP/rx counters. **On-device: a joined client `curl http://192.168.4.1/` returned the page (uptime 124 s, LAN rx 5).** ⇒ **R14 "LAN up" is COMPLETE** — the whole wireless LAN (AP + DHCP + smoltcp stack + mgmt page) runs on the embassy executor over our PIO1 gSPI transport on Hazard3 RISC-V.

**▶ FIRST ACTION — R15: WAN as a DHCP client (first dual-interface step).** Add a smoltcp `dhcpv4` *client* socket on the **10BASE-T** side so the Pico itself gets an upstream IP + default route + DNS from the wired LAN. **The hard part is runtime unification** (the §11 deferral): R14 is standalone-wireless (10BT not started). R15 must run BOTH — keep the 10BT PIO/DMA RX-IRQ-decode on **core 1** (as `main` already does) while the **embassy executor owns core 0** and drives a 2nd smoltcp `Interface` on the `EthMac` phy + the cyw43 stack. Decide how the executor co-exists with the existing `EthMac`/`DMA_IRQ_0` path (likely: core 1 keeps the RX engine; an executor task on core 0 polls the 10BT `Interface` by draining the inbox, like `main_10bt`'s loop but async). **Accept:** the Pico pings `8.8.8.8` + resolves a name out the 10BASE-T WAN. NAPT/forwarding stays R16/R17. **Step plan:** [`docs/router-plan.md`](docs/router-plan.md) §7 (R15 row) — the §12 detail is LAN-only; R15 needs a fresh sub-plan when you start it.

**▶ Tooling now in place (from the board-verify session):** `mpremote` installed (`~/.local/bin/mpremote`, via pipx); MicroPython UF2 cached at `/tmp/mp-pico2w.uf2`. To re-verify the board if ever needed: `picotool load -x -f /tmp/mp-pico2w.uf2`, then `mpremote connect /dev/ttyACM1 exec "import network; w=network.WLAN(network.STA_IF); w.active(True); print(w.scan())"`, then restore with `picotool load -x -t elf target/riscv32imac-unknown-none-elf/release/pico-10base-t-rs`. **NB: a Raspberry Pi Debug Probe (CMSIS-DAP) is attached on `/dev/ttyACM0`** — SWD/OpenOCD may let us finally *observe* the off-header gSPI signals while bringing up the PIO transport (the bit-bang's blind spot).

---

**Background — the multicore RX win (R12, on `main`):** DONE, a net win on every axis, MERGED to `main` (2026-05-28, `8883e05`). Phases 3a→3e turned the R11 23× collapse into a *consistent ≥-baseline-with-headroom* result. `main` now runs the multicore RX + carrier-sense + CSMA/CA backoff in production.

- **3a** multicore foundation ✅ — Hazard3 RISC-V core-1 launch (`src/multicore_riscv.rs`; §9f).
- **3c** RX decode on core 1 ✅ — fixed CPU starvation; revealed carrier-sense (not core separation) is the real TCP lever (§9g).
- **3d** PIO carrier-sense ✅ — carrier-detect SM (PIO0 SM2) + `wait_carrier_idle` before TX (§9h).
- **3e** CSMA/CA backoff + 32 KB TCP window ✅ — `csma_acquire()` random backoff in `send_raw_frame` + larger send window so residual losses fast-retransmit instead of RTO-stall (§9i).

**Final on-wire (240 MHz, http-bulk-test), merged stack vs the old single-core:**

| Metric | single-core (pre-R12) | **now (R12e)** |
|---|---|---|
| Idle 1 MB curl | 596 stable | **500–987, avg 742** (> baseline) |
| Blast 1 MB curl (50 pps) | 26 | **251–988** (10–38×) |
| Collisions / curl | ~0 | **~0.5** |
| ping / UDP echo | 100% | **100% / 10-10** |
| CPU starvation under load | yes | **fixed** |

The merged feature branches (`r12c`/`r12d`/`r12e`) can be deleted now that `main` has the squash-free fast-forward history. The `project-vision-router` goal is now unblocked on the throughput/starvation front.

### ➡️ The router is now scoped — see [`docs/router-plan.md`](docs/router-plan.md)

Full scoping + architecture + phased roadmap (R13→R19) is in `docs/router-plan.md`. Headlines:
- **Architecture fork DECIDED = Option A: keep RISC-V, port the cyw43 transport.** The CYW43 driver ecosystem is embassy/ARM-centric and **embassy supports RP2350 only on the Cortex-M33 cores** — but `cyw43`'s core is transport-agnostic (`SpiBusCyw43` trait) and `embassy-executor` has an `arch-riscv32` backend, so we keep the whole Hazard3 stack and write our own `SpiBusCyw43` on free **PIO1** + an async-runtime shim. (Option B = switch to ARM+embassy, rewrite the NIC — rejected.)
- **smoltcp stays** as the control-plane stack on *both* interfaces (DHCP client, DNS, mgmt HTTP, + cyw43's `NetDriver` glue); the **forwarding + NAPT data path is new custom code** beside it (smoltcp doesn't forward/NAT).
- **First concrete step = R13 wireless de-risk spike** (custom PIO1 SPI + `embassy-executor`(riscv32) + time-driver shim + CYW43 firmware → chip inits/beacons). The async-runtime adoption is the #1 risk — prove it before building the router on it. **Needs the Pico 2 W board swapped in.**
- **Hardware note:** on Pico 2 W the CYW43 takes GP23/24/25/29 and the LED moves to the CYW43 GPIO — our 10BT (GP13/14) coexists, but **GP25 (today's `led` pin) becomes the wireless CS**, so the LED code must relocate.

**Optional MAC polish (orthogonal to the router; none are blockers — already above baseline):**
1. **True per-bit collision-*detect*** (abort+jam mid-frame) — would kill the last ~0.5 coll/curl + the bimodal ~500 lows. Hard/fragile PIO (RO-vs-DI per-cycle compare; loopback-latch schemes have false-positive windows — see §9i). Future polish.
2. **Tune** the CSMA backoff window / TX-window size; bump the *default*-build TCP buffers if real forwarded traffic needs throughput.

**Working state:** `main` = the full multicore-RX win (`8883e05`). Device has the **default production build** of `main` flashed (verified: core 1 up, ping 100%, curl 200, UDP 10/10). For throughput measurement, rebuild with `cargo run --release --features http-bulk-test`.

## Where we are

| Phase | Status | What it does |
|---|---|---|
| **R0** — blinky smoke test | ✅ | Toolchain, linker scripts, picotool flashing, RISC-V boot all verified |
| **R1** — USB CDC serial logging | ✅ | `/dev/ttyACM1` prints `[Rx] tick N` lines once per second; mirrors the C `pico_enable_stdio_usb` workflow |
| **R2** — TX path (PIO Manchester + UDP frame builder + FCS) | ✅ | NLPs at 63/sec → host `carrier=1`; UDP frames at ~5/sec arrive byte-perfect on `192.168.37.19:1234` with payload `"Hello World!! Raspico 10BASE-T Rust !! n=N"` |
| **R3** — RX path (PIO sampler + DMA double-buffer + Manchester decoder + FCS) | ✅ | 60 MHz PIO sampler on GP13 → 2× 16 KB DMA halves (chained, 458 halves/sec) → longest-active-run scan → phase-lock + Manchester decode + SFD → frame-length derivation + CRC-32 verify. ~450 UDP broadcasts/sec decoded byte-perfect with 95–98% FCS OK during host blast. |
| **R4** — smoltcp `phy::Device` integration (ARP + ICMP + UDP) | ✅ | `EthMac` implements `phy::Device`; smoltcp `Interface` answers ARP + ICMP echo, plus a UDP echo socket on port 1234. `ping 192.168.37.24` = 96% success at 10 Hz (RTT 2–4 ms), UDP echo = 90% standalone / 52% under concurrent ping load. |
| **R5** — ring-aware RX scan + multi-slot inbox | ✅ | `EthRx::poll_with` now stitches the previous half's trailing-active tail in front of the new half before invoking the decoder, so frames straddling the DMA boundary survive. `EthMac::poll` walks every active run in the stitched buffer (not just the longest), and the inbox is now a 4-slot `heapless::Deque` (last-writer-wins with drop-oldest on overflow). Concurrent ping+UDP-echo under load: **UDP 98.3% / ping 99.3%** (up from 52% / 96%). |
| **R6** — IRQ-driven RX | ✅ | RX state moved into a module-level `Mutex<RefCell<Option<EthRxShared>>>`; DMA channels `enable_irq0()`'d so each half-completion fires `DMA_IRQ_0`, whose handler runs the full stitch + decode + inbox-push pipeline. Main loop no longer polls — `iface.poll` drains the inbox via `Device::receive`. **2.18 ms main-loop budget is gone.** `EthTx::send_raw_frame`, `send_udp_broadcast`, and `send_nlp` wrap their PIO writes in `critical_section::with` (so the IRQ can't preempt mid-frame and underrun the FIFO) and pad ≥ 9.6 µs of IDLE after every TP_IDL / NLP (so back-to-back TX paths leave the IEEE 802.3-required inter-frame gap before the next preamble). Concurrent stress matches the polled R5 baseline: **UDP 100%, ping 99.7%, host RX errs 0–2 / 30 s.** |
| **R7** — MAC filtering | ✅ | New `EthRx::peek_dst_mac` decodes just the 6 dst-MAC bytes (no Vec allocation, ~1–2 µs) before the IRQ handler decides whether to pay for the full decode + CRC + inbox push. `EthRxShared` learns our MAC via the updated `install_rx(rx, our_mac)` signature; accepts unicast-to-us + all multicast/broadcast (smoltcp does finer-grained filtering downstream). Adds `frames_filtered` to the 1 Hz log. Concurrent stress unaffected: UDP 99.7%, ping 100%, errs ≤1. `filt=0` during normal traffic on this LAN because everything visible is either to-us or IPv6 link-local multicast — the reject path is verified by inspection rather than counter (AF_PACKET-injected unicast-to-unknown-MAC test frames don't actually leave the host's Broadcom NIC in 10HD-half mode, presumably driver-side filtering on raw frames with no ARP target). |
| **R8** — TCP listener | ✅ | `socket-tcp` added to smoltcp feature set; tiny HTTP server on port 80 serves a 200 OK with build info + per-second nlps/udp_sent counters. 1 KB RX + 1 KB TX buffers, re-listens after each closed connection. Concurrent stress (ping + UDP echo + 15 sequential curls): ping 300/300, UDP 299/300, curls 15/15, errs 1/30s — every protocol still at or above polled R5 baseline. Validates that the IRQ-driven RX path + smoltcp handle full TCP handshake + retransmission/windowing/FIN cleanly. |
| **R9** — picotool reset interface | ✅ | New `src/pico_reset.rs` implements a `UsbClass` with a single vendor-specific interface (class=0xFF, sub=0x00, proto=0x01, no endpoints) matching the pico-sdk's `stdio_usb` reset interface. Picotool sends a control transfer (request 0x01 = BOOTSEL); our `control_out` queues the reboot, the next main-loop iteration calls `hal::reboot::reboot(BootSel{...}, Normal)`. Also derives the USB serial from the chip ID (`{wafer_id:08X}{device_id:08X}` via `rom_data::sys_info_api::chip_info()`) so it matches the bootrom's BOOTSEL serial — picotool tracks serials across the app→BOOTSEL transition. `cargo run` / `picotool load -fux -t elf` now self-reboot + flash with **no manual BOOTSEL and no OpenOCD fallback**. Gotcha #4 retired. |
| **R10** — edge-track DPLL Manchester decoder (productized) | ✅ | New `src/eth_rx_dpll.rs` — a per-bit edge-tracking Manchester decoder that re-anchors to each mid-bit transition (search ±1 sample around the expected position) so accumulated clock drift can't walk the sample point off the bit-centre. Replaces the open-loop `EthRx::decode_frame` in the RX IRQ handler. **On-wire, full-MTU FCS-OK jumps from ~1.7 % → ~50 %**, and failed frames now show **flat per-byte error bins (0.1–1.1 %)** vs the open-loop's monotonic ramp from byte 575 — i.e. the residual is PHY-limited, not decoder-limited (locked acceptance criterion §11 escape hatch met; per-byte rate matches the 5.8e-5 per-bit BER predicted from 50 % FCS-OK at 12 000 bits/frame). Sized for the 2.18 ms RX-IRQ half-fill budget at 240 MHz overclock via `get_unchecked` sampling, unrolled W=1 edge search, and an IP-header-derived decode-length cap. Phase log + investigation in `docs/cpu-dpll-plan.md` + `docs/pio-dpll-report.md` (PIO route was tried first, capped at ~40 % due to PIO architectural limits documented there). Same retention is available on small frames (ping 100 %, UDP echo clean). The cargo `--features dpll` opt-in is gone — DPLL is the only decoder; the open-loop and the PIO experiment are preserved in git history (commits `cc09e11`..`8845a38` for PIO; `acdc746`..`f0253c8` for CPU DPLL bring-up). |
| **R11** — FCS-ceiling triage (4 experiments, methodology = gap) | ✅ | Prompted by the Niccle project (ctrlsrc.io) reporting 0 % CRC-fail at full-MTU on functionally identical hardware (same ISL3177E + 10 nF + 100 Ω). Six on-wire measurements (decoder × clock) + a TCP cross-check resolve the apparent gap. See `niccle-comparison-fcs-ceiling` memory + the triage plan at `~/.claude/plans/lets-come-up-with-velvet-stroustrup.md`. Four new feature flags, all off by default (production binary unchanged): `decoder-openloop` (restores pre-R10 fixed-stride decoder), `sample-rate-20mhz` (drops PIO to 20 MHz, matches Niccle's pipeline, auto-implies openloop), `clock-150mhz` (drops sys_clk back to HAL stock 150 MHz), `http-bulk-test` (1 MB TCP streaming endpoint for throughput measurement). **Headline: idle-wire TCP at 596 kB/s = 96 % of Niccle's 620 kB/s.** Under matched conditions our reliability is identical to theirs — the 30–70 % "FCS-fail" we'd been measuring is from a stress-blast methodology (A1 introduced) that nobody actually runs in practice. Real lever is the 23× TCP throughput collapse under concurrent broadcast load (A1 Finding 2), which is now the natural next priority and pulls Phase 3a/3c (multicore RX on the 2nd Hazard3 core) back to the top of the queue. Commits `a6a5af3` (exp 3), `d7a88ab` (exp 5+6), `6bbecee` (exp 4). |
| **R12a** — multicore foundation (Phase 3a) | ✅ | Custom `src/multicore_riscv.rs::launch_core1_riscv` brings up the 2nd Hazard3 core via the bootrom FIFO protocol — clearing the §9a blocker (rp235x-hal 0.4's `multicore::spawn` is Cortex-M only). Four fixes vs the failed attempt: read `mtvec` not `PPB.VTOR`; a `global_asm!` trampoline restores `gp` (bootrom bypasses `_start`); drop the Cortex-M `ACTLR` write (Hazard3 SRAM is coherent, A-ext atomics work); bounded FIFO reads so a dead core 1 returns `launch=FAIL` instead of hanging core 0. On-wire: `[Core1] launch=ok ticks=N` climbing ~1000/s, ping 20/20, curl 200 OK, UDP echo 10/10 — R10 production behaviour preserved. Write-up: [`docs/cpu-dpll-plan.md`](docs/cpu-dpll-plan.md) §9f. |
| **R12c** — move RX IRQ to core 1 (Phase 3c) | ✅ (merged on `main` via R12e) | `DMA_IRQ_0` + the RX decode now run on core 1 (`eth_mac.rs` split into core-1-exclusive `RX_ENGINE` + `Spinlock<0>`-guarded `RX_SHARED`; the decode is lock-free, only the brief inbox/stats publish locks — so core 1's ≤2.57 ms decode never blocks core 0). **Result: starvation FIXED but TCP throughput regressed.** Under a 50 pps full-MTU blast core 1 decodes the full rate (`ok≈50/s`) while core 0 stays at full cadence (`nlps=62–63/s`) — A1 Finding 2 solved. **But** removing the accidental carrier-sense (gotcha #10) collapses idle 1 MB curl from 596 → ~45 kB/s (~12×), with ~30 host collisions/curl — confirmed by `/proc/net/dev` TX-coll + RX-err deltas. Carrier-sense, not core separation, is the real TCP lever. On its own this regressed throughput, so it wasn't merged alone — it landed on `main` as part of the R12e stack once carrier-sense + CSMA closed the gap. Full write-up: [`docs/cpu-dpll-plan.md`](docs/cpu-dpll-plan.md) §9g. |
| **R12d** — PIO carrier-sense (Phase 3d) | ✅ (merged on `main` via R12e) | Carrier-detect SM (PIO0 SM2) watches RO (GP13) and raises host-visible PIO IRQ flag 0 while the line toggles, clearing it after ~267 ns of quiet; `eth_tx.rs`'s `send_*` paths `wait_carrier_idle()` (bounded spin) before the preamble, restoring the carrier-sense Phase 3c removed. **Idle 1 MB curl recovered 114–914 kB/s (avg ~340, peaks > the 596 baseline); collisions cut ~30 → ~1–4.5/curl; blast curl 156–470 vs single-core's 26 (6–18×); ping 100%, TX healthy.** But CS-only ⇒ *residual* collisions remain → variable throughput, idle median still < 596 — closed by R12e. Landed on `main` in the R12e stack. Write-up: [`docs/cpu-dpll-plan.md`](docs/cpu-dpll-plan.md) §9h. |
| **R12e** — CSMA/CA backoff + larger TCP window (Phase 3e) | ✅ **merged to `main`** (`8883e05`) | `eth_tx.rs` `csma_acquire()` adds a random xorshift backoff (0–15 µs) after carrier-sense in `send_raw_frame`, desyncing the Pico from the host's ACKs to cut synchronized-start collisions; `main.rs` bumps the http-bulk-test TCP send window 8→32 KB so the *irreducible* CS-gap residual losses fast-retransmit (~ms) instead of RTO-stalling (~200 ms). **Result — net win on every axis: idle 1 MB curl 500–987 kB/s (avg 742, > the 596 baseline); blast curl 251–988 vs single-core's 26 (10–38×); collisions ~0.5/curl; ping 100%, UDP 10/10; no starvation.** Merged the whole multicore stack (3a+3c+3d+3e) to `main` via fast-forward. True per-bit collision-*detect* deferred (hard PIO, not needed — CSMA/CA + fast-retransmit already clear the baseline). Write-up: [`docs/cpu-dpll-plan.md`](docs/cpu-dpll-plan.md) §9i. |
| **R13** — wireless bring-up (Pico 2 W / CYW43439) | ✅ (branch `r13-wireless-scaffold`, not merged) | The `cyw43` driver inits **end-to-end over our own PIO1 gSPI transport** on Hazard3/RISC-V (no embassy-rp): on-device `[Cyw43] new=1 init=1 led=1` — `cyw43::new()` (231 KB firmware + nvram over PIO1), `Control::init(clm)`, onboard-LED blink — gated by `--features wireless`. Board verified good first via stock MicroPython (12-AP scan); transport proven by reading `0xFEEDBEAD` (**gotcha #11** = hold the gSPI bus idle CLK-low/CS-high/DATA-low through WL_ON power-up; PIO program matched to embassy `cyw43-pio` 0.7.0 — sample DATA CLK-high, `nop side 0` turnaround). Async `SpiBusCyw43` on `PioSpiCyw43` (busy-poll FIFO); the whole bring-up is driven by `embassy_futures::block_on` + `select(runner.run(), init+blink)` — **no persistent executor / async telemetry needed** (observed via the existing 10BT CDC). Blobs vendored in `cyw43-firmware/` (fw + `nvram_rp2040.bin` + clm). Validates the **Option-A architecture fork** (keep RISC-V, port the transport). Plan + findings: [`docs/router-plan.md`](docs/router-plan.md) §10/§11. |
| **R14.1** — persistent executor + continuous `Runner` (R14 "LAN up", step 1) | ✅ on-device-validated (branch `r13-wireless-scaffold`, `6f998fa`) | Graduated R13's `block_on` (which *returned* after 6 blinks) to the **embassy executor owning core 0 forever**: `wireless::run()` spawns `Runner::run()` (long-lived, chip stays up) + a USB poll task (CDC telemetry + picotool reset — `cargo run` still reflashes) + a bootstrap task (`cyw43::new` → spawn Runner → `Control::init(clm)` → blink onboard LED *forever*). `main()` now does shared clock/pin setup then dispatches; the 10BT path moved verbatim into `#[cfg(not(wireless))] fn main_10bt(...)`. **On-device: `[Cyw43] new=1 init=1 led=1`, `hb` climbing 1→28+ over CDC** — executor + continuous Runner + USB all alive. **Retires the #1 deferred risk** (async runtime on Hazard3 at *runtime*). LED relocated to `Control::gpio_set(0,…)` (GP25 = WL_CS). Step plan: [`docs/router-plan.md`](docs/router-plan.md) §12.2. |
| **R14.2** — WPA2 AP up (R14 "LAN up", step 2) | ✅ on-device-validated (branch `r13-wireless-scaffold`, `c7ad6ae`) | Bootstrap task does `set_power_management(None)` (AP must stay awake) + `start_ap_wpa2("pico-rp2350-router", "picorouter2350", ch 6)` after `Control::init`. **On-device `[Cyw43] … ap=1` (hb climbing); SSID confirmed visible on a phone.** Creds/channel = `AP_SSID`/`AP_PASSPHRASE`/`AP_CHANNEL` in `wireless.rs`. NB `NetDriver` (`_net`) still dropped → passive-scan only; joining needs R14.3 (phy adapter) + R14.4 (DHCP). |
| **R14.3** — phy adapter + LAN `Interface` (R14 "LAN up", step 3) | ✅ on-device-validated (branch `r13-wireless-scaffold`, `5039a11`) | New `src/cyw43_phy.rs`: `Cyw43Phy` bridges cyw43's `NetDriver` (an **async `embassy_net_driver::Driver`, not a sync smoltcp plug**) to smoltcp `phy::Device` via a **no-op-waker `Context`** (poll-style `Some`/`None` == smoltcp's sync contract; `net_task` re-polls). **Corrected §12.1** (the sync `try_rx_buf`/`borrow_split` API is on the producer-side `ch::Runner`, not the `NetDriver`). `net_task` builds a 2nd smoltcp `Interface` (`192.168.4.1/24`, MAC from `Control::address()`); ARP + auto-icmp-echo-reply, no sockets. **On-device: host RTL88x2bu client joined the AP (static `192.168.4.2/24`), `ping 192.168.4.1` 5/5 ~6 ms; device `rx` climbed in step — both directions proven.** New dep `embassy-net-driver` 0.2.0. |
| **R14.4** — LAN DHCP server (R14 "LAN up", step 4 — milestone) | ✅ on-device-validated (branch `r13-wireless-scaffold`, `364281f`) | New `src/dhcp_server.rs`: smoltcp UDP `:67` socket in `net_task` reuses `DhcpRepr`/`DhcpPacket` (wireless-only `proto-dhcpv4` feature) — DISCOVER→OFFER, REQUEST→ACK; MAC-keyed pool `192.168.4.10..=.41`, router/server-id `192.168.4.1`, /24, 1 h, broadcast `255.255.255.255:68`. No DNS (R18). `dhcp=` in `[Cyw43]`. **On-device: host client `DHCPACK of 192.168.4.10 from 192.168.4.1`; `dhcp` climbed.** ⚠️ DHCP gateway hijacks a multi-homed host's default route (test via `dhclient -sf /bin/true`; §12.3 risk 6). |
| **R14.5** — LAN mgmt HTTP page (R14 "LAN up", step 5 — **R14 COMPLETE**) | ✅ on-device-validated (branch `r13-wireless-scaffold`, `a745875`) | `net_task` gained a TCP `:80` socket + `serve_status_http` (one-shot HTTP/1.0) showing AP SSID, LAN gateway, uptime, DHCP/rx counters. **On-device: joined client `curl http://192.168.4.1/` returned the page (uptime 124 s, rx 5).** ⇒ **R14 "LAN up" done** — AP + DHCP + smoltcp stack + mgmt page on the executor over PIO1 gSPI. NEXT = R15 (WAN as DHCP client + executor⊥10BT unification). |

Last verified: **R13 wireless (2026-05-28): cyw43 stack inits over our PIO1 gSPI transport — `[Cyw43] new=1 init=1 led=1` (`--features wireless`); LED blinks at boot. 10BASE-T (post-R12e — multicore RX + carrier-sense + CSMA/CA, merged to `main` `8883e05`).** Idle 1 MB curl avg 742 kB/s (500–987, > the 596 baseline), blast 1 MB curl 251–988 (vs single-core's 26), ~0.5 collisions/curl, ping 100%, UDP echo 10/10, no CPU starvation — net win on every axis. Default production build re-flashed + smoke-tested (core 1 up, ping/curl/UDP all green). Full numbers in the callout table above + `docs/cpu-dpll-plan.md` §9f–§9i. Earlier baseline (2026-05-26, post-R6, IRQ-driven RX with TX critsec + IFG padding on every TX path): two-run avg of the 30-sec concurrent stress: ping 99.7%, UDP echo 100.0%, host RX errs ≤2/30s — matched or exceeded the polled R5 baseline on every metric while keeping the IRQ architectural benefit. Telemetry: `dec=20 ok=20 fail=0 inbox_drop=0 inbox_hwm=1–2 carry_cap=0`. The journey from R6's initial 20 errs/30s down to ~1: TX critsec (20 → 8), `send_raw_frame` IFG padding (8 → 4), `send_nlp` IFG padding (4 → 2.5), `send_udp_broadcast` IFG padding (2.5 → ≤2). The pattern was the same every time — once IRQs can preempt the main loop, any TX path that doesn't both critsec its FIFO writes *and* pad post-TP_IDL with ≥ 9.6 µs of IDLE can land its tail under the host NIC's expected IFG window and corrupt the next frame the host receives.

**Performance + idiom review (2026-05-27, branch `review-efficiency-idiom`):** efficiency/idiom pass with on-device cycle measurement (Hazard3 `mcycle` CSR @ 150 MHz, telemetry exported over the UDP broadcast because USB CDC reads go flaky after BOOTSEL re-enumeration — see the `on-device-benchmarking` memory). Applied four safe, behavior-preserving idiom fixes, verified on the wire (UDP 5/s byte-perfect, ping 5/5 @ 2.4–4.9 ms RTT). Measurement **re-prioritized** the deferred efficiency work (decode beats CRC) — see "Performance: measured hot-path costs + plans" under Future work. Headline: worst-case RX IRQ handler = **2.57 ms**, *over* the 2.18 ms half-fill budget under heavy RX load.

**R11 FCS-ceiling matrix (2026-05-28):** six on-wire measurements + a TCP throughput cross-check, same wire / same host / same session.

| sys_clk | DPLL @ 60 MHz | Openloop @ 60 MHz | Openloop @ 20 MHz |
|---|---|---|---|
| 240 MHz | 70.7 / 89.7 % | 19.9 / 95.7 % | 35.6 / 80.7 % |
| 150 MHz | 62.4 / 71.5 % | 19.0 / 92.0 % | 36.3 / 80.0 % |

Format: stress-blast full-MTU FCS-OK% / 600× ping reply%. Stress = 30 s broadcast blast @ 50 pps (1472 B UDP) + concurrent ping flood. **Idle-wire TCP** (1 MB curl, three back-to-back runs, no concurrent stress): **595–600 kB/s — 96 % of Niccle's published 620 kB/s.** Stressed first-curl: 26 kB/s (overlaps the blast); subsequent curls after blast ends: ~596 kB/s. So under matched test conditions our hardware + software performs identically to Niccle's; the FCS gap they don't see is an artifact of our stress methodology, not a real reliability gap. The 23× TCP throughput collapse under stress IS real — same load-collapse story as A1 Finding 2, now confirmed at the user-visible TCP layer.

## File map

| File | Purpose |
|---|---|
| `src/main.rs` | Boot, USB CDC setup, NLP cadence (16 ms), UDP send loop (200 ms), UDP echo socket (port 1234), HTTP server (port 80, R8), heartbeat log + per-second RX status & frame hex dump |
| `src/eth_tx.rs` | `EthTx` struct — Manchester TX PIO (SM0) + frame builder, `send_raw_frame` / `send_nlp` / `send_udp_broadcast`. Owns the `raw_frame` scratch buffer. **R12d/R12e: carrier-sense + CSMA/CA** — installs a carrier-detect SM on PIO0 SM2 (watches RO, raises host-visible PIO IRQ flag 0 while the line toggles); `wait_carrier_idle()` (3d) and `csma_acquire()` (3e: carrier-sense + random xorshift backoff) gate `send_raw_frame` before the preamble so we don't TX into in-flight host frames. |
| `src/pio_util.rs` | `clock_divider(sys_clk_hz, target_hz) -> (int, frac)` — shared PIO fixed-point divider math used by both TX (20 MHz) and RX (60 MHz) `new()` (2026-05-27 review) |
| `src/eth_rx.rs` | `EthRx` struct — PIO sampler, DMA double-buffer with **carry+stitch buffers** (R5), `poll_with` closure handoff over the stitched view, `find_active_run_from` (iterates all runs, not just longest), `peek_dst_mac` (R7, no-alloc dst-MAC pre-decode for the IRQ-side filter), `derive_frame_len`, `verify_fcs`. The full-frame open-loop `decode_frame` was retired in R10 — only the cheap `peek_dst_mac` (always within the no-drift window of the SFD) still uses the open-loop helpers (`find_first_falling_edge` / `find_sfd_end` / `data_bit`). |
| `src/eth_rx_dpll.rs` | Edge-track DPLL Manchester decoder (R10). `decode_frame_edge_track(buf)` is the only full-frame decoder — re-anchors to each mid-bit transition (W=1 sample window), `get_unchecked` sampling once bounds are proven, IP-header-derived decode-length cap. Validated against the offline corpus (FCS-OK N/N) before bring-up; ~50 % full-MTU on-wire is PHY-limited (flat per-byte residual). |
| `src/eth_mac.rs` | `EthMac` — wraps just `EthTx` + a TX scratch buffer + TX stats. **R12c (multicore RX):** RX state split into core-1-exclusive `RX_ENGINE` (`EthRx` + MAC, populated by `install_rx` before core 1 launches; the ≤2.57 ms decode runs lock-free) and `Spinlock<0>`-guarded `RX_SHARED` (inbox + stats, brief publish locks only). The `DMA_IRQ_0` handler runs **on core 1** (stitch + `peek_dst_mac` filter + DPLL decode + publish); `Device::receive`/`snapshot_rx_stats` (core 0) take the spinlock only to pop the inbox / read stats. Decoupling the long decode from the brief shared publish is what makes core separation actually unstarve core 0. |
| `src/crc.rs` | CRC-32/IEEE-802.3 (poly `0xEDB88320`), shared by TX (FCS gen) and RX (FCS verify). Provides `crc32_ieee802_3_padded` for runt-frame TX that pads body to 60 bytes before the FCS |
| `src/manchester.rs` | 256-entry Manchester lookup table, copied verbatim from `../Pico-10BASE-T/src/udp.c` |
| `Cargo.toml` | rp235x-hal, smoltcp 0.13 (`medium-ethernet, proto-ipv4, socket-udp, socket-tcp, auto-icmp-echo-reply` — no defaults, no alloc, no log), usb-device, usbd-serial, heapless, pio |
| `.cargo/config.toml` | RISC-V target, linker args, picotool runner (with OpenOCD fallback) |
| `memory.x` + `rp235x_riscv.x` | Linker scripts for Hazard3 |
| `tools/99-pico-rust.rules` | udev rule to put `/dev/ttyACM*` in the `plugdev` group |
| `src/pico_reset.rs` | `PicoResetInterface` — vendor USB class implementing the pico-sdk reset interface so `picotool -f` can self-reboot us into BOOTSEL (R9) |
| `src/multicore_riscv.rs` | `launch_core1_riscv` (R12a/Phase 3a) — custom Hazard3 RISC-V core-1 bring-up (bootrom FIFO handshake + `gp`-restoring trampoline) because rp235x-hal's `multicore::spawn` is Cortex-M only. See `docs/cpu-dpll-plan.md` §9f. |

## Toolchain summary

| Tool | Use | Where |
|---|---|---|
| `cargo build --release` | Build for `riscv32imac-unknown-none-elf` | Rust stable ≥ 1.82 |
| `picotool load -fux -t elf` | Flash + reboot (works once USB CDC is exposed) | `~/.local/bin/picotool` |
| `openocd ... -f target/rp2350-riscv.cfg` | Flash via SWD (fallback if picotool can't see the device) | `~/src/openocd-rp/` |
| Raspberry Pi Debug Probe (CMSIS-DAP) | OpenOCD's debug probe | SWCLK + SWDIO + GND on the Pico 2 |

**Why not probe-rs/defmt-rtt:** probe-rs 0.31's `RP235x` target only knows the ARM Cortex-M33 cores, not the Hazard3 RISC-V cores. And `defmt-rtt`'s `.uninit` buffer doesn't NOLOAD correctly under `riscv-rt` without a custom linker script rewrite. USB CDC was the pragmatic choice — see `~/.claude/projects/.../memory/rust-port-tooling.md` for the full story.

## Build / flash / smoke test from a fresh checkout

```bash
# 1. Build + flash via `cargo run` — auto-reboots from app into BOOTSEL
#    via the R9 reset interface, no manual button-press needed.
cd ~/projects/pico-10base-t-rs
cargo run --release

# 2. OpenOCD fallback (only needed for first flash onto a chip whose app
#    doesn't yet expose the R9 reset interface, or for recovery):
openocd -s ~/src/openocd-rp/tcl \
        -f interface/cmsis-dap.cfg -f target/rp2350-riscv.cfg \
        -c "adapter speed 5000" -c "init" \
        -c "program target/riscv32imac-unknown-none-elf/release/pico-10base-t-rs verify reset exit"

# 3. Host setup (as root, after host reboot — non-persistent)
ip link set enp1s0f0 up
ethtool -s enp1s0f0 speed 10 duplex half autoneg off
ip addr add 192.168.37.19/24 dev enp1s0f0    # if not already set

# 4. Verify link + RX/TX
cat /sys/class/net/enp1s0f0/carrier   # expect 1

# 4a. RX: blast UDP broadcasts and watch the Pico decode them.
#     Note: `cat /dev/ttyACM1` won't see output because it doesn't assert DTR.
#     usbd-serial buffers writes until a host has DTR set, so use pyserial-
#     style termios (TIOCMBIS + TIOCM_DTR) or a real terminal emulator.
python3 /tmp/blast_udp.py 3000 0.002 &
python3 -c '
import os, time, fcntl, struct
fd = os.open("/dev/ttyACM1", os.O_RDONLY | os.O_NONBLOCK)
fcntl.ioctl(fd, 0x5416, struct.pack("I", 0x002))  # TIOCMBIS, TIOCM_DTR
end = time.time() + 6
buf = b""
while time.time() < end:
    try:
        d = os.read(fd, 4096)
        if d: buf += d
    except BlockingIOError:
        time.sleep(0.05)
print(buf.decode("ascii","replace"))'
# Expect per-second blocks like:
#   [R2b] t=N nlps=63 udp_sent=N
#   [Rx] cand=~450 dec=~450 ok=~430-445 fail=~5-25
#   [Rx] frame 86 bytes, FCS OK - dst ff:ff:ff:ff:ff:ff src 6c:ae:8b:02:9a:1c type=0800
#     0000: ff ff ff ff ff ff 6c ae 8b 02 9a 1c 08 00 45 00
#     0010: 00 44 4? ?? 40 00 40 11 ?? ?? c0 a8 25 13 c0 a8
#     ... (ARPCAPTUREXXX... payload visible from offset 0x32)

# 4b. TX: host receives Pico's UDP broadcasts on 1234.
python3 -c 'import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0", 1234))
while True:
    d, a = s.recvfrom(2048); print(a, d.decode(errors="replace"))'
# expect "Hello World!! Raspico 10BASE-T Rust !! n=..." lines

# 4c. IP-stack verify (R4): ARP, ICMP, UDP echo.
ping -c 1 -W 1 192.168.37.24                     # populates ARP cache
ip neigh show 192.168.37.24                       # expect REACHABLE with our MAC
ping -c 10 -i 0.1 192.168.37.24                  # expect ~95% reply rate
python3 -c '
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(0.5)
s.bind(("192.168.37.19", 0))
for i in range(10):
    msg = f"echo-test-{i:03d} hello".encode()
    s.sendto(msg, ("192.168.37.24", 1234))
    try: print(s.recvfrom(2048)[0].decode())
    except socket.timeout: print(f"TIMEOUT {msg.decode()}")'
# expect 9-10 of 10 echoed back byte-perfect

# 4d. TCP verify (R8): GET / on port 80.
curl -s --max-time 5 http://192.168.37.24/
# expect:
#   Hello from Pico-10BASE-T (Rust)!
#   uptime=<n>s nlps=<n> udp_sent=<n>

# Tip: a fresh-cache ARP probe sometimes lands in a "FAILED" state from a
# prior stale entry; the first `ping -c 1` clears it, subsequent pings work.
```

## Hard-won gotchas

1. **`out pc, N` in PIO jumps to *absolute* addresses.** The Manchester dispatch table MUST live at PIO instruction offsets 0..2. Without `.origin 0` in the `pio_asm!` block, `pio::install()` puts the program elsewhere (we saw offset 26), and the SM jumps off into empty `0x0000` slots, silently looping. The symptom is sneaky: SM reports "running," FIFO drains, pin reads as `Output`/`PIO0`-funcseled, GPIO_IN shows toggling — but on the wire there are no NLPs and the host carrier never comes up.
2. **`StateMachine::start()` consumes `self`.** If you do `sm.start();` without binding the returned `StateMachine<_, Running>`, you've created and immediately dropped the running handle. Whether that disables the SM depends on internals; always bind it: `let sm = sm.start();` and store in your struct.
3. **`panic-probe` is Cortex-M only** — it emits a `compile_error!` on `riscv32`. Use a plain `#[panic_handler]` that logs via your own channel (we use defmt+RTT-style printf via USB CDC).
4. **picotool's `-f` auto-reboot needs the pico-sdk's "reset interface"** (vendor-specific USB endpoint), not just a CDC ACM with `VID:PID=2e8a:000a`. Bare `usbd-serial` advertises the right VID:PID but doesn't expose the reset interface, so picotool errors with `Unable to locate reset interface`. **Resolved in R9** — `src/pico_reset.rs` implements the interface as a `UsbClass` (vendor class, sub=0x00, proto=0x01, no endpoints) and reboots from main-loop context via `hal::reboot::reboot(BootSel{...}, Normal)`. Two gotchas inside the gotcha: (a) picotool sends a **Class** request type (`bmRequestType=0x21`), not Vendor, even though our interface descriptor says class=0xFF — TinyUSB's vendor driver dispatches both; usb-device routes strictly, so we have to accept both `RequestType::Class` *and* `RequestType::Vendor`. (b) Picotool tracks the device by USB serial number across the app→BOOTSEL reboot, so the app's serial must match what the bootrom advertises in BOOTSEL mode (= the chip ID, formatted as `{wafer_id:08X}{device_id:08X}` from `rom_data::sys_info_api::chip_info()`); using a static string like `"R1"` triggers a successful reboot followed by "no accessible RP-series devices in BOOTSEL mode were found with serial number R1".
5. **`cat /dev/ttyACM1` may show nothing** even when the firmware is writing fine. `usbd-serial` only delivers buffered bytes once a host asserts DTR; plain `cat` doesn't set DTR via termios. Use a tool that does (pyserial, `minicom`, `screen`, or the `TIOCMBIS + TIOCM_DTR` ioctl shown in the verify recipe). Dropped diagnostic time chasing this once — easy to forget.
6. **`hal::singleton!(: [u32; N] = ...)` is the canonical way to allocate a `&'static mut` DMA buffer** in rp235x-hal. `&'static mut [u32; N]` impls `StableDeref` (via `stable_deref_trait`) and behaves correctly through `embedded-dma`'s blanket `WriteBuffer` impl. No `Box`, no `UnsafeCell` wrapping needed; no special alignment beyond u32 since we use `double_buffer` (not RP2350's endless-ring mode).
7. **PIO TX FIFO underruns mid-frame if the CPU pauses between writes.** The original `EthTx::send_raw_frame` pushed the body bytes, then computed CRC-32 (bit-by-bit, ~27 µs at 150 MHz for a 98-byte frame), then pushed FCS bytes. The 8-deep TX FIFO drains in ~6 µs at 20 MHz half-bit rate, so during the CRC compute the wire stalled, the receiver lost Manchester sync, and the host NIC scored a bad FCS on every frame that hit this path. **Fix: precompute the CRC before *any* PIO writes** so the per-byte writes run uninterrupted. Symptoms were sneaky — UDP broadcasts (built whole-frame in a buffer first) worked perfectly, but anything routed through smoltcp's `TxToken::consume → send_raw_frame` (ARP replies, ICMP echo replies, smoltcp-emitted UDP) failed silently because we didn't see the NIC's RX-error counter until we explicitly looked. Verified by `cat /proc/net/dev` ticking up RX-errors by exactly one per sent frame.
8. **Runt-frame padding moves the FCS.** `EthRx::derive_frame_len` originally trusted the IPv4 total-length field and computed `14 + ip_total_len + 4`. But IEEE 802.3 requires the *frame* to be ≥ 60 bytes pre-FCS; the host pads short IP packets with zeros before appending the FCS. A short UDP echo (e.g. 10-byte payload → 52-byte body) gets padded to 60, so the FCS lives at bytes 60..63, not at `ip_total_len`. The decoder was running CRC over the wrong range and FCS-failing every short reply, while default-sized pings (56-byte payload → 98-byte body) sailed through. **Fix: `max(14 + ip_total_len + 4, 64)`.**
9. **Once IRQs are enabled, every TX path needs `critical_section` *and* IFG padding.** R6 enabled `DMA_IRQ_0`, whose handler runs the decoder (~100 µs of work). Without protection, that IRQ pre-empts mid-frame FIFO writes (same symptom as gotcha #7, different cause) — wrapping the FIFO loop in `critical_section::with` fixes that. But there's a second, subtler bug: any TX path that ends with TP_IDL and *doesn't* pad the line with ≥ 9.6 µs of IDLE (IEEE 802.3 minimum IFG) lets the next frame's preamble land too close to the previous tail, and the host NIC scores it bad-FCS. In polled mode this never showed up because `mac.poll`'s decode time naturally introduced > 100 µs of dead air between back-to-back smoltcp egresses; in IRQ mode that dead time is gone and back-to-back TXs can be < 10 µs apart. **Fix:** push 12 all-zero FIFO words (≈ 9.6 µs of IDLE dispatches) after every TP_IDL / NLP — applies to `send_raw_frame`, `send_udp_broadcast`, *and* `send_nlp`. Skipping any one of them leaves residual host RX errs. Tried gating NLPs on "no recent frame TX" first — counter-intuitively that made ping *worse*, suggesting the Broadcom NIC's link-integrity logic does want the steady NLP cadence even during traffic.
10. **No CSMA/CD = anything that makes the IRQ handler shorter risks half-duplex collisions.** Followup to #9: the IRQ handler's runtime *also* acts as accidental carrier-sense. The current MAC filter (R7) accepts all multicast and pays ~100 µs of full decode per multicast frame; while the IRQ is decoding, main can't TX, so a reply queued by `iface.poll` waits until the wire has been quiet for that decode duration. Narrowing the filter to reject most multicast (draft R10, reverted) cuts the IRQ to ~1–2 µs at the peek stage — and immediately exposes the missing carrier-sense. Replies start landing on the wire while the host is still mid-transmitting an IPv6 multicast, both frames collide, both get scored bad-FCS at the host. The clean test: pre-subscribe to the observed multicast (i.e. re-introduce the long decode) restored numbers to baseline. Real fix is CSMA in PIO; until then, anything that *speeds up* the IRQ handler (MAC filter, lighter decoder, IRQ-side decoder bypass) needs to keep this trade-off in mind. See "Beyond R9" #1 for the deferred multicast work. **✅ RESOLVED in R12c–R12e (2026-05-28):** R12c moved the RX IRQ to core 1 — which removed this *accidental* carrier-sense and made the collisions acute (idle TCP 596→45), proving the trade-off was real and load-bearing. R12d then added explicit **PIO carrier-sense** (a carrier-detect SM on RO + `wait_carrier_idle` before TX) and R12e added **CSMA/CA backoff**, so the "speeding up the IRQ handler" hazard no longer applies — TX now defers on real wire activity regardless of how fast the handler runs. (True per-bit collision-*detect* is still future polish — see §9i.)
11. **The CYW43 gSPI bus must be held idle THROUGH the WL_ON power-up** (R13 Step 1). Bringing up the cyw43 transport, our PIO1 gSPI probe read floating data (`0x5fffffff`, varying) — identical to the earlier bit-bang — even though: the board is MicroPython-verified-good (12-AP scan), the pin map is correct (pico-sdk `pico2_w.h`: WL_ON=23/DATA=24/CS=25/CLK=29), the pads drive (a `pin_selftest` confirmed lo=0/hi=1 on all four), the power+read sequence matches cyw43's `Bus::init`, and the PIO program matches embassy `cyw43-pio` 0.7.0. The bug was **ordering**: we power-cycled WL_ON *before* configuring the gSPI pins, so **CLK floated during the chip's power-up**, latching the CYW43 into a wrong gSPI mode → it never drove DATA back. **Fix: build the PIO SM and drive CLK low / CS high / DATA low BEFORE raising WL_ON** (embassy constructs `PioSpi` with pins low, *then* `init()` powers up). With the bus held idle through power-up, the test register reads `0xFEEDBEAD` first try. Sneaky because every *logical* thing (pins, command, sequence, program, sample edge) was correct — only the power-up pin state was wrong, and it's invisible on off-header lines. Two transport-timing details also had to match embassy to be safe: sample DATA on CLK-**high** (`in side 1` / `jmp side 0`), and turnaround `nop side 0` (no spurious extra clock pulse).

## Known limitations / TODOs

- **Residual FCS fails (~0–1/sec under load).** A few RX decodes per second still mark FCS-fail (the `fail=N` field in the `[Rx]` log line). `carry_cap=0` rules out cap-clipping, so the cause is elsewhere — likely some combination of: (a) genuine wire bit-errors, (b) phase-lock edge cases when the run starts on a noisy NLP, (c) the decoder's "longest run" → "find next run" change occasionally finding a spurious noise blob between frames. Not affecting user-visible reliability (smoltcp doesn't see these); worth instrumenting only if it becomes the bottleneck.
- **RX IRQ handler worst case (2.57 ms) exceeds the 2.18 ms half-fill budget under heavy load.** Measured 2026-05-27 via the `mcycle` CSR. The `DMA_IRQ_0` handler (`process_completed_half`) must finish before the *other* DMA half fills (2.18 ms) or samples drop. Steady state is fine, but a half densely packed with active runs during a UDP blast can push it over. Decomposition of the worst case: stitch copy ≈ 296 µs (16 KB memcpy), plus per-frame `decode_frame`+`verify_fcs` ≈ 238 µs each (dominated by the two-pass bit ops, **not** the CRC — see below). **NB: the 238 µs is the pre-plan-#1 two-pass figure at the old ~199-byte cap. Plan #1 (single-pass, done) was re-measured on device: comparable at 199 B (185 µs avg / 258 µs worst) but, because it removed the cap, a large-frame decode now scales up to ~1217 µs at 1600 B — so the single-pass change does NOT shrink this worst case and can grow it under large-frame RX. See the plan-#1 measurement under Future work.** Rare today (still ~99% under stress) but real headroom pressure. **Progress (2026-05-27):** the single-pass packing opt cut per-decode ~16% (worst large-frame ~1217→~940 µs); the decode-length cap bounds a decode to the header-declared length (no full-buffer decode on an over-long run); and stitch scan-in-place removed the 296 µs copy entirely under light load (100% of halves) / cut it 77% under blast. So the three biggest contributors to the 2.57 ms worst case are all down materially. (Table CRC, the other documented lever, was tried and dropped — measured only ~8 % even with a RAM table on this single-issue core; see Performance plan #3.) Note the same handler runtime doubles as accidental carrier-sense (gotcha #10), so shortening it is a genuine trade-off, not a free win — all of the above were on-wire-validated at/above baseline. **Update (R12c, 2026-05-28):** this handler now runs on **core 1**, which does nothing else — so a long decode no longer starves core 0's main loop/smoltcp, and the "shortening it loses carrier-sense" trade-off is retired (carrier-sense is now explicit PIO, R12d/R12e). The budget concern is narrower now: the decode of one half must still finish before the *other* half fills (else samples drop), but it's core-1-exclusive headroom, not a whole-system bottleneck.
- ~~**`decode_frame` truncates frames larger than ~199 bytes.**~~ **FIXED in Performance plan #1 (2026-05-27, single-pass decoder).** Was: the bit loop `for j in 0..1600` into a `Vec<u8, 2048>` recovered at most ~199 frame bytes, so full-MTU RX never actually worked despite `MTU = 1500`. The single-pass rewrite sizes output to `MAX_FRAME_BYTES` and bounds the walk only by available samples. Verified on the wire: a UDP echo at payload 600 B (frame 646 B) now round-trips 40/40 byte-perfect (was hard 0% above ~199 B). Frames up to ≥1246 B decode (must, to echo at all); round-trip echo % then falls off with frame size — 846 B 70%, 1046 B 28%, 1246 B 15%, 1518 B ~0% — which is **wire/PHY round-trip reliability** (RX + TX both over half-duplex 10BT, longer frame = more bit-error exposure), not a decoder cap. (Also bumped the UDP echo handler's `echo_buf` from 512 → 1472 B so the echo service no longer silently truncates datagrams > 512 B, which had masked the RX fix.)
- **ARP cache can stick in `FAILED` state on the host** if an early ARP probe times out (before the Pico is up, or during a flash cycle). Linux backoffs prevent retries for minutes, making `ping` look broken when it's actually waiting. Workaround: a single `ping -c 1 192.168.37.24` (or `ip neigh del 192.168.37.24` with root) clears the FAILED entry; subsequent traffic re-resolves.
- ~~**picotool reset interface not implemented**~~ — done in R9 (gotcha #4 retired).
- ~~**`static mut RAW_FRAME` in `send_udp_broadcast`** triggers a Rust 2024 warning~~ — fixed in the 2026-05-27 review: it's now the owned `EthTx.raw_frame` field. Disjoint-borrow trick lets the critsec loop read `self.raw_frame` while writing `self.tx`.
- **sys_clk runs at 150 MHz**, not 120 MHz like the C version. Both PIO TX (div 7.5 → 20 MHz half-bit) and PIO RX (div 2.5 → 60 MHz sample) use fractional dividers with ±3.3 ns jitter. Confirmed working end-to-end at this rate; could be cleaned up by dropping to 120 MHz for integer dividers.
- **USB CDC drops bytes when log throughput is high.** Frame hex dumps occasionally come through truncated/interleaved at the host. The data we get is correct; this is just a TX-buffer-full silent-drop on the device side (`let _ = serial.write(...)`). Throttle further or implement a write loop that yields if it becomes a real problem.

## Future work

### Router project — A1 link characterization (2026-05-27)

**Context:** the end goal is for this NIC to be the WAN uplink of a low-power
RP2350 wireless router (clients on a wireless module, NAT-routed out 10BASE-T —
see the `project-vision-router` memory). **A1** = measure whether the link can
actually carry real bidirectional/routed traffic. **Verdict: not yet — two
blockers, one of them fundamental to the current decoder.**

Method: device cumulative RX telemetry (decoded/ok/fail/drop/cap) dumped over
the UDP broadcast; host floods of (a) broadcast→dead-port = pure RX with no
TX-back, and (b) UDP-echo = RX-decode + TX-encode per packet (router proxy).

**Finding 1 — full-MTU RX is broken; FCS-ok collapses with frame size, even at
low rate.** At a non-saturating 150 pps: 64 B **98 %**, 256 B 93 %, 512 B 85 %,
1024 B **38 %**, 1518 B **1.7 %**. The implied per-bit error rate *rises ~10×*
with frame length (≈3.5e-5 → 3.3e-4) — NOT uniform noise (which is constant per
bit and would predict ~34 % at 1518 B). That's the signature of **accumulated
clock drift**: `decode_frame` locks phase once at the SFD and then samples at a
fixed `F + 4 + 6k` stride with **no clock recovery**, so any TX/RX oscillator
mismatch (±100 ppm 10BT tolerance + our 150 MHz fractional-divider jitter) walks
the sample point off the bit centre over a long frame — drift can exceed a full
bit over a 1.2 ms full-MTU frame. (AC-coupling baseline wander may compound it.)
**Fundamental to the decoder — full-duplex hardware does NOT fix this.** ⇒ can't
carry full-MTU TCP bulk traffic. Fix needs decoder **clock recovery** (re-sync
phase on the Manchester mid-bit transition each bit).

**CONFIRMED by a per-byte error-position test** (full-MTU known-pattern frames
at 120 pps; device bins payload byte-errors by position): error rate vs frame
offset — bytes 42–593 **0.0 %**, 594–777 2.8 %, 778–961 24 %, 962–1145 **50 %**,
1146–1329 74 %, 1330–1513 89 %. The first ~575 bytes are *perfect*, then errors
ramp monotonically through exactly **50 % near byte ~1050** (sample point landing
on a bit boundary) to ~89 % at the tail. Uniform PHY noise would be *flat*; this
clean 0 %→ramp is the textbook clock-drift signature. **So the blocker is our
decode algorithm (firmware-fixable via clock recovery), NOT the analog PHY.**
Usable frame size today ≈ ~575 B (matches 512 B @85 %, 1518 B @1.7 %). The
sample point drifts ~half a bit over ~500 µs, so recovery need only re-center
every ≪ that — trivial given a Manchester transition every 100 ns.

**Finding 2 — single core collapses under load.** Under a saturating flood the
RX IRQ starves the main loop and the 4-slot inbox overflows, so bidirectional
echo goodput falls to **0.02–0.13 Mbit/s at 0.6–2.2 % round-trip success** (vs
~100 % at light rates). Pure RX decode ceiling: ~3370 pps @64 B (1.7 Mbit/s)
down to ~400 pps @1518 B; small frames are decode-bound and the inbox drains at
only ~250–500/s under load. ⇒ need **core separation** (NIC IRQ on one Hazard3
core, stack/routing on the other) + a bigger inbox + flow control.

**Revised priority order (A1 reshaped it — these now precede NAT/wireless):**
1. ~~**Decoder clock recovery** (full-MTU RX).~~ — **DONE in R10** (edge-track DPLL,
   `docs/cpu-dpll-plan.md`). R11 confirmed that under matched conditions (idle
   TCP) we deliver Niccle's throughput; the residual flat-bin per-byte error
   under stress is methodology-bound, not a real reliability ceiling.
2. ~~**Core separation + buffering — the top blocker.**~~ — **DONE in R12c**
   (RX IRQ + decode on the 2nd Hazard3 core; `docs/cpu-dpll-plan.md` §9f/§9g).
   Fixed the A1 Finding 2 / R11 starvation, but exposed that collisions — not
   CPU — were the deeper TCP limiter (carrier-sense is the real lever).
3. ~~**Collisions / half-duplex** (full-duplex HW or PIO CSMA).~~ — **DONE in
   R12d/R12e** (PIO carrier-sense + CSMA/CA backoff + larger TCP window; §9h/§9i).
   The merged stack now beats single-core on every axis. True per-bit
   collision-*detect* is optional future polish, not a blocker.
4. **…then NAT/forwarding, the wireless interface, DHCP (the router proper).**
   ⇐ now the top of the queue.

### Performance: measured hot-path costs + plans (2026-05-27)

On-device measurement (Hazard3 `mcycle` @ 150 MHz, 6.67 ns/cyc), worst case under a UDP blast + ping:

| What | Cost | Notes |
|---|---|---|
| Isolated CRC-32 | ~12.2 cyc/byte (~81 ns/B) | 60 B = 4.9 µs; ~123 µs at full MTU |
| `decode_frame` + `verify_fcs` | **238 µs** worst/frame | ~214 µs is bit extraction+packing; only ~16 µs CRC at current ~199 B frames |
| Stitch copy (`poll_with`) | **296 µs** worst | 16 KB `copy_from_slice`, ~458×/s |
| Full RX IRQ handler | **2.57 ms** worst | **over** the 2.18 ms half-fill budget under load |

**Surprise from measuring: decode beats CRC.** By inspection I'd ranked the bit-by-bit CRC #1; on-device it's the two-pass bit twiddling in `decode_frame` that dominates the IRQ.

**Every item below shortens the RX IRQ handler — which is also the accidental carrier-sense window (gotcha #10).** So none is a guaranteed win; each MUST be validated on-wire, not assumed. The reverted R10 multicast attempt hit exactly this wall.

**Validation protocol (run after EACH change):** 30-sec concurrent stress — `ping -c 600 -i 0.05 192.168.37.24` + a host UDP echo loop + the host UDP listener on 1234 — and record (a) ping reply %, (b) UDP echo %, (c) host RX-error delta from `cat /proc/net/dev`. Baseline to beat: ping ≥ 99.7%, UDP echo ~100%, host RX errs ≤ 2/30 s. Any drop = carrier-sense loss → the speedup traded latency for collisions; back it out or pair it with real PIO carrier-sense.

1. ~~**Single-pass decoder — priority #1, biggest lever.**~~ **DONE (2026-05-27).** Replaced the two-pass `decode_frame` (sample bits → `Vec<u8,2048>` → pack → `Vec<u8,1600>`) with a single pass that reads each data bit on demand via a shared `data_bit()` helper and packs straight to bytes — no per-bit intermediate `Vec`, no second pass.
   - (a) ✅ After F-find + SFD-find, output bytes are built directly from `data_bit(f + 4 + 6*k)` reads.
   - (b) ✅ Walk is bounded only by available samples and `MAX_FRAME_BYTES` (= 1600, from `eth_mac`), not a magic 1600-*bit* cap — **fixes the ~199-byte truncation**; full-MTU-range RX now works (see Known limitations).
   - (c) ✅ F-find + SFD-find + per-bit read factored into shared private helpers (`find_first_falling_edge`, `find_sfd_end`, `data_bit`) used by both `decode_frame` and `peek_dst_mac`. `peek_dst_mac` is now also single-pass (dropped its 200-byte stack array).
   - **Validation (gotcha-#10 protocol, same-day before/after on a slightly noisy wire):** new firmware ping 99.5–99.7% / UDP echo 96.8–97.5% / host RX errs Δ6–8 per 30 s, vs old-firmware baseline 99.3–100% / 95.2–96.2% / Δ8–9. **Matches or beats baseline on every metric — no carrier-sense regression.** Correctness: payload-600 UDP echo now 40/40 byte-perfect (was 0% above ~199 B). Clippy clean (bar the pre-existing `too_many_arguments`).
   - **Measured `mcycle` cost (2026-05-27) — the "~half" hypothesis did NOT hold; measuring flipped it again.** New `decode_frame`+`derive`+`verify_fcs`, avg over thousands of frames per size (150 MHz, 6.667 ns/cyc): 90 B 118 µs, **199 B 185 µs avg / 258 µs worst**, 400 B 285 µs, 800 B 515 µs, 1200 B 677 µs. At the old 199-byte cap point the new decoder is **comparable** to the old ~238 µs worst (modestly cheaper on average, not half). **And removing the old `for j in 0..1600` cap raised the worst case:** that cap implicitly bounded any decode to ~199 B ≈ 238 µs; uncapped, a large run decodes up to `MAX_FRAME_BYTES` = 1600 B ≈ **1217 µs for a single decode**. So **plan #1 does not reduce the 2.57 ms IRQ worst case — it can raise it under large-frame RX** (frames that simply didn't decode at all before). **Net: plan #1 is a correctness (full-MTU RX) + code-clarity win, not the IRQ-budget win the plan predicted.**
   - **Stage decomposition (2026-05-27, follow-up measurement) — corrects the "~88 µs fixed overhead" framing, which was a linear-fit artifact.** Per-stage `mcycle` timing inside `decode_frame` (stages sum to the totals above, e.g. 0.9 + 11.0 + 130.6 + 42.5 = 185 µs at 199 B): **F-find 0.9 µs** (flat), **SFD scan 11.0 µs** (flat — SFD found cleanly at bit 60 *every* time, `sfd_iters_max` = 60 cumulative, so the "noisy late-SFD" hypothesis was **wrong**), **packing ~0.70 µs/byte = 13.3 cyc/bit** (the dominant cost — 71% of a 199 B decode — and it scales: 0.70 × 1600 ≈ matches the 1217 µs worst), **verify/CRC ≈ 27 µs fixed + 0.075 µs/byte**. The true size-independent cost is only ~40 µs (and ~27 µs of it is inside the CRC, → plan #3 territory). **The real lever to make decode cheaper is the per-bit `data_bit` packing cost, not "fixed overhead":** stride the sample index (`+6`, drop the per-bit multiply), hoist the sample-availability bound out of the inner loop, `get_unchecked` after proving the range, drop the per-bit `Option`. This attacks the 13.3 cyc/bit and so helps both typical decodes *and* the large-frame worst case (which is ~90 % packing). Plus follow-up (i): cap decode length to a sane bound (the old 199 B cap was accidentally a decode-time DoS bound on the IRQ). All ⚠️ gotcha #10 — validate on wire.
   - **Packing optimization DONE (2026-05-27).** Rewrote the packing loop: whole-byte count hoisted out, sample offset strided by 6 (no per-bit multiply), per-bit `Option` dropped, `bytes.get_unchecked` over a range proven in-bounds (with `nsamples` clamped to the buffer so the `unsafe` is sound for any caller). Re-measured: **packing ~0.70 → ~0.51 µs/byte (12.9 → 9.4 cyc/bit, ~27 %)**; whole-decode **199 B 185 → 155 µs (−16 %)**, 90 B 110 → 98 µs; large-frame worst ~1217 → ~940 µs (−23 %). A real but modest win (the residual 9.4 cyc/bit is mostly the irreducible load+shift+mask per bit; going further needs word-at-a-time extraction — diminishing returns). On-wire: byte-perfect decodes at every size (199 B echo 20/20), stress ping 99.5–100 % / UDP 95–97 % / RX errs 4–9 per 30 s — no correctness or gotcha-#10 regression. SFD/F-find untouched (still flat ~12 µs). Remaining lever for the IRQ worst case: follow-up (i) decode-length cap + stitch scan-in-place (#2).

2. ~~**Stitch scan-in-place — priority #2.**~~ **DONE (2026-05-27).** The 16 KB `copy_from_slice` ran every half (~296 µs of the IRQ).
   - (a) ✅ `carry_len == 0` (common — previous half ended idle): `f` is called once on `new_bytes` directly, no copy.
   - (b) ✅ `carry_len > 0`: stitch only `carry + leading active run` (the straddling frame's tail, up to the first idle byte), then call `f` a second time on the remainder of the half in place. Split point is an idle byte, so no run is cut — `f` sees every frame whole, identical to the old full-stitch.
   - **Also implemented follow-up (i): decode-length cap.** `decode_frame` now packs the 18-byte header, derives the declared frame length (EtherType + IPv4 total-len, runt-padded to 64), and bounds the rest to it — so an over-long active run (merged frames / noise) can't force a full `MAX_FRAME_BYTES` decode. Behaviour-preserving (a normal run ≈ its own frame length; `verify_fcs`/`derive_frame_len` already use the same declared length).
   - **Validation (on wire):** stress ×3 ping 99.7–100 % / UDP echo 99.7–100 % / host RX errs 0–2 per 30 s (best of the session — at/above every prior baseline); multi-size echo 20/20 byte-perfect at 96/199/346/646 B (straddle handling correct — UDP echoes cross boundaries under load). Clippy clean.
   - **Copy-elimination measured on device (counters, not assumed):** under light traffic **100 % of halves skip the copy entirely (0 B vs 16384 B)** — the full ~296 µs gone every half. Under a heavy large-frame blast (the load that actually stresses the budget), 29 % still skip and the rest stitch only `carry + one frame tail`, so **avg bytes copied/half = 3796 B, a 77 % cut** from the flat 16384 B. So (b) delivers under load, not just (a) in the common case.

3. ~~**Table-driven CRC-32 — priority #3, TX-side win.**~~ **TRIED, measured no worthwhile benefit, REVERTED (2026-05-27).** Implemented a `const fn`-generated 256-entry table (verified bit-identical to the bit-by-bit CRC: standard vector `0xcbf43926` + 20 000 random frames) and benched it on device (boot-time 256 × 60-byte isolated CRC, `mcycle`):
   - bit-by-bit (original): **12.2 cyc/byte**
   - table in **flash** (`.rodata`): **12.2 cyc/byte (0 %)** — each lookup pays XIP/flash latency ≈ the 8 shifts it replaced.
   - table in **RAM** (`.data` via `link_section`): **11.2 cyc/byte (~8 %)** — better, but nowhere near the hoped ~8×.
   - **Why:** Hazard3 is single-issue in-order; the table version has a load-use stall (table read feeds straight into the XOR) that offsets its lower instruction count vs the bit-by-bit shifts. The "~8× table win" intuition is for superscalar cores, not this one.
   - **Verdict:** an ~8 % gain on a path that is **not a bottleneck** (CRC ≈ 16 µs of a 155 µs decode; TX FCS ≈ 7 µs/frame and TX has no budget pressure) does not justify table generation + 1 KB flash/RAM + the `link_section` hack. Kept the simpler bit-by-bit. **All three documented Performance plans are now resolved (#1 done, #2 done, #3 dropped on evidence).**

### Beyond R9 — improvements (priority order, pick whichever bites)

1. **Multicast group subscriptions — INVESTIGATED, deferred.** Attempted in a draft R10 (commit `a843066`, since reverted): narrow `mac_accept` to accept only unicast-to-us, broadcast, and explicitly subscribed multicast MACs (with a `subscribe_multicast`/`unsubscribe_multicast` API). The narrow filter measurably *worsened* user-visible reliability: when we pre-subscribed to the actual IPv6 multicast we observed on the wire (`33:33:00:00:00:16`), stress numbers returned to baseline (~100% / 99.7% / 2 errs); with the default empty list, they dropped to ~95% / 80% / 20–30 errs. **Hypothesis:** the IRQ handler exits much faster when it rejects a multicast at the cheap `peek_dst_mac` stage instead of doing the full Manchester decode. That extra ~100 µs of "IRQ busy" was acting as accidental carrier-sense on the half-duplex 10BT wire — without it, main-loop TX racing against still-in-flight host multicasts causes uncatchable collisions (we have no CSMA/CD in PIO). Before re-attempting: either (a) add real carrier-sense to PIO TX, (b) gate the filter on full-duplex mode only, or (c) leave the default permissive and only narrow when the caller knows the wire is full-duplex. Today's wire was also unusually unstable, which made the magnitude hard to pin down — would benefit from a scope check on DI/RO during the next investigation.

2. **Pico-side HTTP request parsing.** The R8 server ignores the request line entirely — every GET (and every other verb) gets the same response. Route on method+path so we can expose distinct endpoints (e.g., `/stats`, `/frames`, `/reset`).

3. ~~**Clean up the `static mut RAW_FRAME` warning**~~ — done in the 2026-05-27 review (now `EthTx.raw_frame`).

### Cleanup wishlist
- ~~Add picotool reset interface~~ — done (R9).
- ~~Replace `static mut RAW_FRAME` with an owned-by-`EthTx` buffer~~ — done (2026-05-27 review).
- ~~Replace the `EthMac` diagnostic stats fields with a compile-time toggle~~ — done (2026-05-27): `diag` cargo feature (off by default). Gates the verbose per-second CDC output (the `[Mac]` TX-categorization line, the TX/frame hex dumps, the decoded-frame pretty-print) + `hex_dump`; the cheap `[R2b]` heartbeat + `[Rx]` decode summary always print. Lean default build is ~60 KB smaller ELF; `--features diag` restores full diagnostics. (Gated the *output* rather than threading `#[cfg]` through the EthMac/EthTxToken/IRQ hot paths — the now-unused stat writes are dead-store-eliminated by LTO, and the stat structs' `pub` fields avoid dead-code warnings.)
- ~~Decompose `main()` (~450 lines)~~ — done (2026-05-27): UDP-echo / HTTP-serve / 1 Hz-logging blocks extracted into `serve_udp_echo` / `serve_http` / `log_status` free functions; loop body is now ~10 lines of orchestration. The R4.2 smoltcp-UDP demo block (`next_smol_udp`/`smol_udp_sent` + the hand-built "smoltcp tx n=" broadcast) was removed as dead scaffolding — its `EthTxToken` egress path is exercised by all real smoltcp traffic now — which also dropped a pile of unused `smoltcp::wire`/`phy` imports.
- Inbox copies move the full 1600-byte `Vec` per push/pop (~1.4 MB/s) regardless of frame length; a length-prefixed byte ring would be compact but more complex — low priority.
- Consider dropping sys_clk to 120 MHz to get integer PIO dividers (matches the C version's choice and reduces TX jitter)
- ~~Move `EthTx::new` to consume rather than borrow `pio`~~ — not feasible: `EthRx` needs the same `PIO0` borrow for SM1, so `pio` must be shared by reference.
- USB CDC frame-dump throttling — currently the 1 Hz hex dump can interleave with `[Mac]` lines when the USB IN buffer is near full; implement a small write-loop with `usb_dev.poll()` between chunks. (Note: CDC reads also go unreliable after repeated BOOTSEL re-enumeration — use the UDP payload as a telemetry channel instead; see the `on-device-benchmarking` memory.)

## Memory cues for future Claude

Auto-memory directory: `~/.claude/projects/-home-mattdeeds-projects-Pico-10BASE-T/memory/` (shared with the C repo, since the projects are sibling). Key entries:
- `rust-port-tooling.md` — what works for Hazard3 RP2350 (USB CDC, OpenOCD-RP, picotool) and what doesn't (probe-rs, defmt-rtt with riscv-rt out of the box)
- `pio-origin-zero-gotcha.md` — why `out pc, N` programs need `.origin 0`
- `hardware-isl3177e.md` — pin assignments + Plan A → Plan B decision
- `network-setup.md` — `ethtool autoneg off` requirement after every host reboot

`MEMORY.md` in that directory is the index.

This Rust repo also has its own memory dir: `~/.claude/projects/-home-mattdeeds-projects-pico-10base-t-rs/memory/`:
- `on-device-benchmarking.md` — `mcycle` CSR + `mcountinhibit` enable, and why telemetry goes over UDP not USB CDC
- `review-2026-05-efficiency-findings.md` — measured RX IRQ hot-path costs; decode beats CRC; 2.57 ms worst-case IRQ
