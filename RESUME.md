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
| **Beyond R8** — pick from improvements list below | ⏳ | next |

Last verified: 2026-05-26 (post-R6, IRQ-driven RX with TX critsec + IFG padding on every TX path). Two-run avg of the 30-sec concurrent stress: ping 99.7%, UDP echo 100.0%, host RX errs ≤2/30s — matches or exceeds the polled R5 baseline on every metric while keeping the IRQ architectural benefit. Telemetry: `dec=20 ok=20 fail=0 inbox_drop=0 inbox_hwm=1–2 carry_cap=0`. The journey from R6's initial 20 errs/30s down to ~1: TX critsec (20 → 8), `send_raw_frame` IFG padding (8 → 4), `send_nlp` IFG padding (4 → 2.5), `send_udp_broadcast` IFG padding (2.5 → ≤2). The pattern was the same every time — once IRQs can preempt the main loop, any TX path that doesn't both critsec its FIFO writes *and* pad post-TP_IDL with ≥ 9.6 µs of IDLE can land its tail under the host NIC's expected IFG window and corrupt the next frame the host receives.

## File map

| File | Purpose |
|---|---|
| `src/main.rs` | Boot, USB CDC setup, NLP cadence (16 ms), UDP send loop (200 ms), UDP echo socket (port 1234), HTTP server (port 80, R8), heartbeat log + per-second RX status & frame hex dump |
| `src/eth_tx.rs` | `EthTx` struct — PIO program install, frame builder, `send_nlp` / `send_udp_broadcast` |
| `src/eth_rx.rs` | `EthRx` struct — PIO sampler, DMA double-buffer with **carry+stitch buffers** (R5), `poll_with` closure handoff over the stitched view, `find_active_run_from` (iterates all runs, not just longest), `peek_dst_mac` (R7, no-alloc dst-MAC pre-decode for the IRQ-side filter), `decode_frame` + `derive_frame_len` + `verify_fcs` |
| `src/eth_mac.rs` | `EthMac` — wraps just `EthTx` + a TX scratch buffer + TX stats. RX state lives in a module-level `Mutex<RefCell<Option<EthRxShared>>>` populated via `install_rx(rx, our_mac)`; the `DMA_IRQ_0` handler enters a critical section to run the stitch + `peek_dst_mac` filter + decode + push pipeline. `Device::receive` pops from the shared inbox via a small critical section. |
| `src/crc.rs` | CRC-32/IEEE-802.3 (poly `0xEDB88320`), shared by TX (FCS gen) and RX (FCS verify). Provides `crc32_ieee802_3_padded` for runt-frame TX that pads body to 60 bytes before the FCS |
| `src/manchester.rs` | 256-entry Manchester lookup table, copied verbatim from `../Pico-10BASE-T/src/udp.c` |
| `Cargo.toml` | rp235x-hal, smoltcp 0.13 (`medium-ethernet, proto-ipv4, socket-udp, socket-tcp, auto-icmp-echo-reply` — no defaults, no alloc, no log), usb-device, usbd-serial, heapless, pio |
| `.cargo/config.toml` | RISC-V target, linker args, picotool runner (with OpenOCD fallback) |
| `memory.x` + `rp235x_riscv.x` | Linker scripts for Hazard3 |
| `tools/99-pico-rust.rules` | udev rule to put `/dev/ttyACM*` in the `plugdev` group |

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
# 1. Build (cargo run auto-flashes via picotool when the device exposes USB CDC)
cd ~/projects/pico-10base-t-rs
cargo build --release
cargo run --release    # may need OpenOCD fallback on the very first flash

# 2. OpenOCD fallback (use if picotool reports "Unable to locate reset interface"):
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
4. **picotool's `-f` auto-reboot needs the pico-sdk's "reset interface"** (vendor-specific USB endpoint), not just a CDC ACM with `VID:PID=2e8a:000a`. Bare `usbd-serial` advertises the right VID:PID but doesn't expose the reset interface, so picotool errors with `Unable to locate reset interface`. Solutions: either fall back to OpenOCD for flashing, or add a custom USB vendor interface. **Open TODO**, deferred for now.
5. **`cat /dev/ttyACM1` may show nothing** even when the firmware is writing fine. `usbd-serial` only delivers buffered bytes once a host asserts DTR; plain `cat` doesn't set DTR via termios. Use a tool that does (pyserial, `minicom`, `screen`, or the `TIOCMBIS + TIOCM_DTR` ioctl shown in the verify recipe). Dropped diagnostic time chasing this once — easy to forget.
6. **`hal::singleton!(: [u32; N] = ...)` is the canonical way to allocate a `&'static mut` DMA buffer** in rp235x-hal. `&'static mut [u32; N]` impls `StableDeref` (via `stable_deref_trait`) and behaves correctly through `embedded-dma`'s blanket `WriteBuffer` impl. No `Box`, no `UnsafeCell` wrapping needed; no special alignment beyond u32 since we use `double_buffer` (not RP2350's endless-ring mode).
7. **PIO TX FIFO underruns mid-frame if the CPU pauses between writes.** The original `EthTx::send_raw_frame` pushed the body bytes, then computed CRC-32 (bit-by-bit, ~27 µs at 150 MHz for a 98-byte frame), then pushed FCS bytes. The 8-deep TX FIFO drains in ~6 µs at 20 MHz half-bit rate, so during the CRC compute the wire stalled, the receiver lost Manchester sync, and the host NIC scored a bad FCS on every frame that hit this path. **Fix: precompute the CRC before *any* PIO writes** so the per-byte writes run uninterrupted. Symptoms were sneaky — UDP broadcasts (built whole-frame in a buffer first) worked perfectly, but anything routed through smoltcp's `TxToken::consume → send_raw_frame` (ARP replies, ICMP echo replies, smoltcp-emitted UDP) failed silently because we didn't see the NIC's RX-error counter until we explicitly looked. Verified by `cat /proc/net/dev` ticking up RX-errors by exactly one per sent frame.
8. **Runt-frame padding moves the FCS.** `EthRx::derive_frame_len` originally trusted the IPv4 total-length field and computed `14 + ip_total_len + 4`. But IEEE 802.3 requires the *frame* to be ≥ 60 bytes pre-FCS; the host pads short IP packets with zeros before appending the FCS. A short UDP echo (e.g. 10-byte payload → 52-byte body) gets padded to 60, so the FCS lives at bytes 60..63, not at `ip_total_len`. The decoder was running CRC over the wrong range and FCS-failing every short reply, while default-sized pings (56-byte payload → 98-byte body) sailed through. **Fix: `max(14 + ip_total_len + 4, 64)`.**
9. **Once IRQs are enabled, every TX path needs `critical_section` *and* IFG padding.** R6 enabled `DMA_IRQ_0`, whose handler runs the decoder (~100 µs of work). Without protection, that IRQ pre-empts mid-frame FIFO writes (same symptom as gotcha #7, different cause) — wrapping the FIFO loop in `critical_section::with` fixes that. But there's a second, subtler bug: any TX path that ends with TP_IDL and *doesn't* pad the line with ≥ 9.6 µs of IDLE (IEEE 802.3 minimum IFG) lets the next frame's preamble land too close to the previous tail, and the host NIC scores it bad-FCS. In polled mode this never showed up because `mac.poll`'s decode time naturally introduced > 100 µs of dead air between back-to-back smoltcp egresses; in IRQ mode that dead time is gone and back-to-back TXs can be < 10 µs apart. **Fix:** push 12 all-zero FIFO words (≈ 9.6 µs of IDLE dispatches) after every TP_IDL / NLP — applies to `send_raw_frame`, `send_udp_broadcast`, *and* `send_nlp`. Skipping any one of them leaves residual host RX errs. Tried gating NLPs on "no recent frame TX" first — counter-intuitively that made ping *worse*, suggesting the Broadcom NIC's link-integrity logic does want the steady NLP cadence even during traffic.

## Known limitations / TODOs

- **Residual FCS fails (~0–1/sec under load).** A few RX decodes per second still mark FCS-fail (the `fail=N` field in the `[Rx]` log line). `carry_cap=0` rules out cap-clipping, so the cause is elsewhere — likely some combination of: (a) genuine wire bit-errors, (b) phase-lock edge cases when the run starts on a noisy NLP, (c) the decoder's "longest run" → "find next run" change occasionally finding a spurious noise blob between frames. Not affecting user-visible reliability (smoltcp doesn't see these); worth instrumenting only if it becomes the bottleneck.
- **RX is polled, not IRQ-driven.** `EthMac::poll` is called every main-loop iteration; the main loop must complete each iteration in under 2.18 ms (one DMA-half fill time) or samples drop. Currently safe — the longest blocking call (TX of an 86-byte UDP frame) is ~200 µs, and `iface.poll` itself is fast.
- **ARP cache can stick in `FAILED` state on the host** if an early ARP probe times out (before the Pico is up, or during a flash cycle). Linux backoffs prevent retries for minutes, making `ping` look broken when it's actually waiting. Workaround: a single `ping -c 1 192.168.37.24` (or `ip neigh del 192.168.37.24` with root) clears the FAILED entry; subsequent traffic re-resolves.
- **picotool reset interface not implemented** — see gotcha #4. Manual OpenOCD flash works fine as a fallback.
- **`static mut RAW_FRAME` in `send_udp_broadcast`** triggers a Rust 2024 compatibility warning. Functionally fine on a single-threaded core; would clean up with `UnsafeCell` or move the buffer to `EthTx` state.
- **sys_clk runs at 150 MHz**, not 120 MHz like the C version. Both PIO TX (div 7.5 → 20 MHz half-bit) and PIO RX (div 2.5 → 60 MHz sample) use fractional dividers with ±3.3 ns jitter. Confirmed working end-to-end at this rate; could be cleaned up by dropping to 120 MHz for integer dividers.
- **USB CDC drops bytes when log throughput is high.** Frame hex dumps occasionally come through truncated/interleaved at the host. The data we get is correct; this is just a TX-buffer-full silent-drop on the device side (`let _ = serial.write(...)`). Throttle further or implement a write loop that yields if it becomes a real problem.

## Future work

### Beyond R8 — improvements (priority order, pick whichever bites)

1. **Multicast group subscriptions.** Currently the MAC filter accepts *all* multicast (anything with I/G bit set) and relies on smoltcp to drop ones we don't care about. Fine for small workloads, wasteful on a busy multicast LAN. Once smoltcp's multicast subscription API is wired, narrow `mac_accept` to specific group MACs.

2. **Pico-side HTTP request parsing.** The R8 server ignores the request line entirely — every GET (and every other verb) gets the same response. Route on method+path so we can expose distinct endpoints (e.g., `/stats`, `/frames`, `/reset`).

3. **picotool reset interface.** Gotcha #4 — `cargo run` still needs an OpenOCD fallback or a manual BOOTSEL because `usbd-serial` doesn't expose the pico-sdk vendor reset endpoint. Adding a custom USB vendor class would let `picotool load -fux` self-reboot.

4. **Clean up the `static mut RAW_FRAME` warning** in `send_udp_broadcast` — pre-existing, harmless, but it'll become a hard error in a future Rust edition.

### Cleanup wishlist
- Add picotool reset interface so `cargo run` flashes without the OpenOCD fallback (custom usb-device vendor class)
- Replace `static mut RAW_FRAME` in `send_udp_broadcast` with an owned-by-`EthTx` buffer (the legacy `static mut` warning has been there since R2)
- Replace the `EthMac` diagnostic stats fields (`tx_arp`, `tx_icmp`, `tx_udp`, `tx_other`, `last_tx`, etc.) with a compile-time toggle — they're useful when bringing up a new feature but bloat both code and the 1 Hz log line in steady state
- Consider dropping sys_clk to 120 MHz to get integer PIO dividers (matches the C version's choice and reduces TX jitter)
- Move `EthTx::new` to consume rather than borrow `pio` so the type is cleaner
- USB CDC frame-dump throttling — currently the 1 Hz hex dump can interleave with `[Mac]` lines when the USB IN buffer is near full; implement a small write-loop with `usb_dev.poll()` between chunks

## Memory cues for future Claude

Auto-memory directory: `~/.claude/projects/-home-mattdeeds-projects-Pico-10BASE-T/memory/` (shared with the C repo, since the projects are sibling). Key entries:
- `rust-port-tooling.md` — what works for Hazard3 RP2350 (USB CDC, OpenOCD-RP, picotool) and what doesn't (probe-rs, defmt-rtt with riscv-rt out of the box)
- `pio-origin-zero-gotcha.md` — why `out pc, N` programs need `.origin 0`
- `hardware-isl3177e.md` — pin assignments + Plan A → Plan B decision
- `network-setup.md` — `ethtool autoneg off` requirement after every host reboot

`MEMORY.md` in that directory is the index.
