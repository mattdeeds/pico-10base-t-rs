# pico-10base-t-rs — Resume

Checkpoint for picking up the Rust port of [Pico-10BASE-T](../Pico-10BASE-T/) after a break. Targets the **Hazard3 RISC-V** cores of the RP2350 (Pico 2) with `rp235x-hal`. Same external hardware as the C repo — ISL3177E + HR911105A + AC-coupling caps + 50 Ω source termination.

For the C reference and the proven Manchester / decoder design, see [`../Pico-10BASE-T/RESUME.md`](../Pico-10BASE-T/RESUME.md) and [`../Pico-10BASE-T/CLAUDE.md`](../Pico-10BASE-T/CLAUDE.md).

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
| **Beyond R10** — pick from improvements list below | ⏳ | next |

Last verified: 2026-05-26 (post-R6, IRQ-driven RX with TX critsec + IFG padding on every TX path). Two-run avg of the 30-sec concurrent stress: ping 99.7%, UDP echo 100.0%, host RX errs ≤2/30s — matches or exceeds the polled R5 baseline on every metric while keeping the IRQ architectural benefit. Telemetry: `dec=20 ok=20 fail=0 inbox_drop=0 inbox_hwm=1–2 carry_cap=0`. The journey from R6's initial 20 errs/30s down to ~1: TX critsec (20 → 8), `send_raw_frame` IFG padding (8 → 4), `send_nlp` IFG padding (4 → 2.5), `send_udp_broadcast` IFG padding (2.5 → ≤2). The pattern was the same every time — once IRQs can preempt the main loop, any TX path that doesn't both critsec its FIFO writes *and* pad post-TP_IDL with ≥ 9.6 µs of IDLE can land its tail under the host NIC's expected IFG window and corrupt the next frame the host receives.

**Performance + idiom review (2026-05-27, branch `review-efficiency-idiom`):** efficiency/idiom pass with on-device cycle measurement (Hazard3 `mcycle` CSR @ 150 MHz, telemetry exported over the UDP broadcast because USB CDC reads go flaky after BOOTSEL re-enumeration — see the `on-device-benchmarking` memory). Applied four safe, behavior-preserving idiom fixes, verified on the wire (UDP 5/s byte-perfect, ping 5/5 @ 2.4–4.9 ms RTT). Measurement **re-prioritized** the deferred efficiency work (decode beats CRC) — see "Performance: measured hot-path costs + plans" under Future work. Headline: worst-case RX IRQ handler = **2.57 ms**, *over* the 2.18 ms half-fill budget under heavy RX load.

## File map

| File | Purpose |
|---|---|
| `src/main.rs` | Boot, USB CDC setup, NLP cadence (16 ms), UDP send loop (200 ms), UDP echo socket (port 1234), HTTP server (port 80, R8), heartbeat log + per-second RX status & frame hex dump |
| `src/eth_tx.rs` | `EthTx` struct — PIO program install, frame builder, `send_nlp` / `send_udp_broadcast`. Owns the `raw_frame` UDP-build scratch buffer (was a `static mut`, fixed in the 2026-05-27 review) |
| `src/pio_util.rs` | `clock_divider(sys_clk_hz, target_hz) -> (int, frac)` — shared PIO fixed-point divider math used by both TX (20 MHz) and RX (60 MHz) `new()` (2026-05-27 review) |
| `src/eth_rx.rs` | `EthRx` struct — PIO sampler, DMA double-buffer with **carry+stitch buffers** (R5), `poll_with` closure handoff over the stitched view, `find_active_run_from` (iterates all runs, not just longest), `peek_dst_mac` (R7, no-alloc dst-MAC pre-decode for the IRQ-side filter), `derive_frame_len`, `verify_fcs`. The full-frame open-loop `decode_frame` was retired in R10 — only the cheap `peek_dst_mac` (always within the no-drift window of the SFD) still uses the open-loop helpers (`find_first_falling_edge` / `find_sfd_end` / `data_bit`). |
| `src/eth_rx_dpll.rs` | Edge-track DPLL Manchester decoder (R10). `decode_frame_edge_track(buf)` is the only full-frame decoder — re-anchors to each mid-bit transition (W=1 sample window), `get_unchecked` sampling once bounds are proven, IP-header-derived decode-length cap. Validated against the offline corpus (FCS-OK N/N) before bring-up; ~50 % full-MTU on-wire is PHY-limited (flat per-byte residual). |
| `src/eth_mac.rs` | `EthMac` — wraps just `EthTx` + a TX scratch buffer + TX stats. RX state lives in a module-level `Mutex<RefCell<Option<EthRxShared>>>` populated via `install_rx(rx, our_mac)`; the `DMA_IRQ_0` handler enters a critical section to run the stitch + `peek_dst_mac` filter + DPLL decode + push pipeline. `Device::receive` pops from the shared inbox via a small critical section. |
| `src/crc.rs` | CRC-32/IEEE-802.3 (poly `0xEDB88320`), shared by TX (FCS gen) and RX (FCS verify). Provides `crc32_ieee802_3_padded` for runt-frame TX that pads body to 60 bytes before the FCS |
| `src/manchester.rs` | 256-entry Manchester lookup table, copied verbatim from `../Pico-10BASE-T/src/udp.c` |
| `Cargo.toml` | rp235x-hal, smoltcp 0.13 (`medium-ethernet, proto-ipv4, socket-udp, socket-tcp, auto-icmp-echo-reply` — no defaults, no alloc, no log), usb-device, usbd-serial, heapless, pio |
| `.cargo/config.toml` | RISC-V target, linker args, picotool runner (with OpenOCD fallback) |
| `memory.x` + `rp235x_riscv.x` | Linker scripts for Hazard3 |
| `tools/99-pico-rust.rules` | udev rule to put `/dev/ttyACM*` in the `plugdev` group |
| `src/pico_reset.rs` | `PicoResetInterface` — vendor USB class implementing the pico-sdk reset interface so `picotool -f` can self-reboot us into BOOTSEL (R9) |

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
10. **No CSMA/CD = anything that makes the IRQ handler shorter risks half-duplex collisions.** Followup to #9: the IRQ handler's runtime *also* acts as accidental carrier-sense. The current MAC filter (R7) accepts all multicast and pays ~100 µs of full decode per multicast frame; while the IRQ is decoding, main can't TX, so a reply queued by `iface.poll` waits until the wire has been quiet for that decode duration. Narrowing the filter to reject most multicast (draft R10, reverted) cuts the IRQ to ~1–2 µs at the peek stage — and immediately exposes the missing carrier-sense. Replies start landing on the wire while the host is still mid-transmitting an IPv6 multicast, both frames collide, both get scored bad-FCS at the host. The clean test: pre-subscribe to the observed multicast (i.e. re-introduce the long decode) restored numbers to baseline. Real fix is CSMA in PIO; until then, anything that *speeds up* the IRQ handler (MAC filter, lighter decoder, IRQ-side decoder bypass) needs to keep this trade-off in mind. See "Beyond R9" #1 for the deferred multicast work.

## Known limitations / TODOs

- **Residual FCS fails (~0–1/sec under load).** A few RX decodes per second still mark FCS-fail (the `fail=N` field in the `[Rx]` log line). `carry_cap=0` rules out cap-clipping, so the cause is elsewhere — likely some combination of: (a) genuine wire bit-errors, (b) phase-lock edge cases when the run starts on a noisy NLP, (c) the decoder's "longest run" → "find next run" change occasionally finding a spurious noise blob between frames. Not affecting user-visible reliability (smoltcp doesn't see these); worth instrumenting only if it becomes the bottleneck.
- **RX IRQ handler worst case (2.57 ms) exceeds the 2.18 ms half-fill budget under heavy load.** Measured 2026-05-27 via the `mcycle` CSR. The `DMA_IRQ_0` handler (`process_completed_half`) must finish before the *other* DMA half fills (2.18 ms) or samples drop. Steady state is fine, but a half densely packed with active runs during a UDP blast can push it over. Decomposition of the worst case: stitch copy ≈ 296 µs (16 KB memcpy), plus per-frame `decode_frame`+`verify_fcs` ≈ 238 µs each (dominated by the two-pass bit ops, **not** the CRC — see below). **NB: the 238 µs is the pre-plan-#1 two-pass figure at the old ~199-byte cap. Plan #1 (single-pass, done) was re-measured on device: comparable at 199 B (185 µs avg / 258 µs worst) but, because it removed the cap, a large-frame decode now scales up to ~1217 µs at 1600 B — so the single-pass change does NOT shrink this worst case and can grow it under large-frame RX. See the plan-#1 measurement under Future work.** Rare today (still ~99% under stress) but real headroom pressure. **Progress (2026-05-27):** the single-pass packing opt cut per-decode ~16% (worst large-frame ~1217→~940 µs); the decode-length cap bounds a decode to the header-declared length (no full-buffer decode on an over-long run); and stitch scan-in-place removed the 296 µs copy entirely under light load (100% of halves) / cut it 77% under blast. So the three biggest contributors to the 2.57 ms worst case are all down materially. (Table CRC, the other documented lever, was tried and dropped — measured only ~8 % even with a RAM table on this single-issue core; see Performance plan #3.) Note the same handler runtime doubles as accidental carrier-sense (gotcha #10), so shortening it is a genuine trade-off, not a free win — all of the above were on-wire-validated at/above baseline.
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
1. **Decoder clock recovery** (full-MTU RX). Without it the link can't carry
   real traffic at all. Biggest blocker; was hidden until A1. **Design plan:
   [`docs/clock-recovery-decoder-plan.md`](docs/clock-recovery-decoder-plan.md)**
   (offline sample corpus first, then DPLL/edge-resync vs PIO-side recovery).
2. **Core separation + buffering** so decode-under-load doesn't starve routing.
3. **Collisions / half-duplex** (full-duplex HW or PIO CSMA) — matters once 1+2
   are fixed; secondary today.
4. …then NAT/forwarding, the wireless interface, DHCP (the router proper).

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
