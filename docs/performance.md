# Performance — what to expect

Measured on real hardware. The short version: the **bit-bang 10BASE-T TX is near
line rate**, but two things shape real throughput — the link is **half-duplex**
(so single-flow TCP collides with its own ACKs and varies a lot), and **10BASE-T
RX runs ~310 KB/s at stock MTU with ~0.2% wire loss** — the former ~100 KB/s
"decode/PHY ceiling" was root-caused (2026-06-10) to DMA-starvation sample
loss plus ACK mis-pacing, and fixed; see `rx-bulk-ceiling.md` §9-§10.
Latency is excellent. This is a fun, capable, *educational* software-PHY NIC and a
working small router — not a fast router.

## TL;DR

| Path | Direction | Throughput | Notes |
|---|---|---|---|
| **10BASE-T** (wired, the bit-bang) | TX, device→host (TCP) | **best ~0.95–1.0 MB/s**, typical single-flow ~0.4–0.7 MB/s | near line rate when clean; half-duplex collision variance + occasional RTO stalls |
| **10BASE-T** | RX, host→device (TCP bulk) | **~310 KB/s** | stock MTU, ~0.2% wire loss; immediate-ACK + 2-seg window + decode-out-of-IRQ (`rx-bulk-ceiling.md` §9-§10) |
| **10BASE-T** | round-trip latency | **~2.6 ms** (0% loss) | ping, 30/30 |
| **Wi-Fi LAN** (cyw43 AP, router build) | download (device→client) | **~909 KB/s** | after the gSPI 2→15 MHz fix |
| **Wi-Fi LAN** (cyw43 AP) | upload (client→device) | **~716 KB/s** | |
| **Routed LAN↔WAN** (NAPT) | bidirectional | bounded by the 10BT half-duplex link (TX ~1 MB/s, RX ~310 KB/s) | not the wire or CPU; see §Router |
| **CPU** | core 1 (RX decode) | **~40 %** at idle | cost of the always-on 60 MHz sampler decoding ambient wire traffic |

Line rate for reference: 10 Mbit/s ≈ 1.25 MB/s raw, ≈ 1.18 MB/s of TCP payload.

## Test setup (and how to reproduce)

- **Board:** Raspberry Pi Pico 2 W (RP2350, Hazard3 RISC-V), `clk_sys` 240 MHz.
- **10BASE-T front end:** ISL3177E transceiver + HR911105A RJ45 magnetics
  (GP14 = DI/TX, GP13 = RO/RX). See the README wiring table.
- **Wired peer:** a Linux host NIC linked at 10BASE-T (`enp1s0f0`), forced to
  10 Mb/s half-duplex; device static `192.168.37.24`.
- **Builds:** the wired numbers use `--features "http-bulk-test fd-bench diag"`
  (adds a 1 MB HTTP source on :80, a TCP sink on :9999, and the `[Rx]` decode
  stats over USB CDC). The Wi-Fi/router numbers use `--features router`.
- **Tools:** `tools/` (the measurement scripts) + the device's USB-CDC telemetry
  (`[R2b]`/`[Rx]`/`[Sink]`/`[Perf]` lines; assert DTR to read them).

## Detail

### 10BASE-T TX (device → host) — fast PHY, half-duplex-limited
The Manchester bit-bang TX runs at the full 10 Mbit/s line clock, so a *clean* 1 MB
HTTP download lands at **~0.95–1.0 MB/s** (near the ~1.18 MB/s payload ceiling).
But the link is **half-duplex** with a CSMA/CA (not CSMA/CD) MAC: the device's data
segments collide with the peer's TCP ACKs, so single-flow TCP throughput is
**bimodal** — clean transfers ~0.94–1.0 MB/s, collision-stalled ones ~0.25–0.5 MB/s,
and the occasional RTO storm times out entirely. Typical single-flow average lands
~0.4–0.7 MB/s. (Carrier-sense + randomized backoff keep this from collapsing the way
naive multicore TX did — see `docs/cpu-dpll-plan.md` gotcha #10.)

### 10BASE-T RX (host → device) — ~310 KB/s after the 2026-06 fixes
Sustained bulk RX runs **~310 KB/s at stock MTU 1500 with ~0.2% wire loss**
(was ~100 KB/s at ~30% full-MTU FCS-fail). Two rounds of fixes, both in
`rx-bulk-ceiling.md`:
- **§9 — ACK pacing + window:** the 10 ms delayed-ACK timer sat exactly on
  Linux's tail-loss-probe timer (collision storm), and the 1-segment
  advertised window serialized the link. Immediate ACKs + a 2-segment window.
- **§10 — the "decode cliff" was DMA starvation, not clock drift:** decode ran
  in the DMA IRQ in front of the re-arm; under load it overran the half-fill
  period and the PIO RX FIFO overflowed (~200×/s), silently truncating frames.
  Decode now runs in core-1 thread context from a 6-slot image ring; the IRQ
  only captures + re-arms in bounded time. Health counters: `rxstall` (must
  stay 0) and `img_drop` (decode backlog) on the diag `[Stitch]` line.

Remaining bounds: decode CPU time (~1-1.5 ms/frame) and the ~2.2 ms DMA
half-fill ACK-latency floor (smaller halves are viable again now that the
decode deadline coupling is gone).
### Latency — excellent
`ping` over the wired link: **min/avg/max 2.1 / 2.6 / 3.0 ms, 0 % loss** (30 pkts).

### Wi-Fi LAN (cyw43 AP, router build)
After raising the custom PIO gSPI transport clock from the 2 MHz bring-up value to
15 MHz, the cyw43 2.4 GHz AP does **~909 KB/s down / ~716 KB/s up** — on par with the
wired NIC, no longer the router bottleneck (the radio was never the limit; the
bring-up SPI clock was). See `docs/perf-characterization-plan.md` §3.5.

### Router (LAN ↔ WAN, NAPT)
The full routed/NAT path (Wi-Fi client ↔ Pico ↔ 10BT WAN) is **not** CPU- or
forwarding-limited (core-0 forwarding fast-path is ~idle, drops ~0). It's bounded by
the **slower link and half-duplex contention** on the 10BT side —
i.e. the per-link ceilings above, with the 10BT RX path bounding any
WAN→LAN-heavy (download) flow. Best for low-rate / IoT-scale traffic.

### CPU
Core 1 owns the RX decode (the `DMA_IRQ_0` Manchester + FCS pipeline). It sits at
**~40 % even at idle**, just decoding ambient 10BASE-T wire traffic — the cost of the
always-on 60 MHz sampler + per-DMA-half wake. Core 0 (the executor / forwarding) is
near-idle without load.

## Honest limitations

- **RX bulk ~310 KB/s** — next bounds are decode CPU time and the DMA
  half-fill latency floor; see `docs/rx-bulk-ceiling.md` §10.
- **Half-duplex only** by MAC policy. The transceiver is *full-duplex-capable*, but
  FD only helps contended/multi-client traffic (measured before the 2026-06 RX
  fixes — worth re-testing against the now-clean link); not
  worth the auto-negotiation work — see `docs/full-duplex-analysis.md`.
- **No auto-negotiation** — emits link pulses (NLPs) only; a switch parallel-detects
  it as 10BASE-T half-duplex (which is what we want).
- **Sustained full-MTU inbound can wedge the device** (intermittent). An RP2350
  **hardware watchdog** auto-reboots + recovers it (~6 s); root-cause is still open.
- Educational / hobby project. Great for learning software PHYs and as a slow but
  real 10BASE-T NIC / small router; not a production-grade fast router.

## See also (the engineering log)
`docs/rx-bulk-ceiling.md` (RX ceiling), `docs/full-duplex-analysis.md` (FD),
`docs/cpu-dpll-plan.md` + `docs/clock-recovery-decoder-plan.md` (the RX decoder),
`docs/perf-characterization-plan.md` (cyw43 LAN + routed), `docs/router-plan.md`
(router architecture).
