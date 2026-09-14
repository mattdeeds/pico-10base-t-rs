# Engineering log

Raw notes: history, experiments, lessons learned. Code comments describe the
current state only; the "why it got this way" lives here. Sections are by
module. Dates are when the change or finding happened, where known.

Much of this was moved out of source comments on 2026-09-14. Git history
before that date has the original wording.

## Open issues

- **`cpu1=` under-reads core-1 load.** `CORE1_BUSY` wraps only `DMA_IRQ_0`
  (`eth_mac.rs`). Perf step 2 added it when the IRQ ran the whole ≤2.57 ms
  decode. Since the 2026-06-10 restructure (`212662d`), decode runs in
  `drain_rx_images` on core 1's thread, outside the span, so `[Perf] cpu1=`
  counts only capture + re-arm. Fix: also bracket `drain_rx_images`.
- **`EthRxStats::frames_filtered` doc was wrong.** It said "runs that
  peek_dst_mac accepted but couldn't decode". The code counts MAC-filter
  rejects. Doc fixed 2026-09-14; the code was not changed.
- **`StatsDelta::carry` is never incremented.** Carry caps are merged
  separately in `process_completed_half`. Harmless, but dead.
- **Mislabeled CPU readouts.** The mgmt page prints `core1(rx-decode)=` and
  `[Perf]` prints `cpu1=`, but both come from `CORE1_BUSY` (IRQ only; see above).
- **`nb` dependency looks unused.** Nothing in `src/` references it.

## Transport history (RX)

- **Port origin.** `eth_rx.rs` ports `rx_10base_t.pio` and the decoder from
  `eth_rx.c` in the C reference repo (`../Pico-10BASE-T`). The 60 MHz sampler
  (3 samples per half-bit) matches the C version.
- **R6:** RX became IRQ-driven (`DMA_IRQ_0`).
- **R10:** the edge-track DPLL (`eth_rx_dpll`) replaced the open-loop
  locked-once decoder. The stated reason was clock drift: open-loop per-byte
  errors ramped from byte ~575 to ~89% at the tail of full-MTU frames (the
  "A1 ramp-from-575 B" failure mode). The open-loop decoder came back behind
  `decoder-openloop` for the FCS-ceiling A/B against Niccle's fixed-stride
  pipeline. Note: the 2026-06-10 finding below suggests much of the
  full-MTU "cliff" was DMA starvation, not drift.
- **R12c:** RX decode moved to core 1. `RX_SHARED` locks are never held across
  the decode, so core 1's decode can't starve core 0 (the point of the move).
  `Spinlock<0>` is used because `critical_section` reserves `Spinlock<31>`.
- **2026-06-10, `212662d`: decode moved out of the IRQ.** Scan + decode used
  to run inline in `DMA_IRQ_0`, ahead of the DMA re-arm. Under load that took
  longer than one half-fill (~2.18 ms), the chain completed with nothing
  armed, and the 8-word PIO RX FIFO overflowed ~200×/s, silently truncating
  frames in flight. This was misread for weeks as a clock-drift "decode
  cliff". Now the IRQ only captures the image and re-arms (~0.6 ms worst);
  core 1's thread decodes from a 6-slot image ring. Result: RX ~310 KB/s at
  ~0.2% loss at stock MTU. The `rxstall` and `img_drop` counters watch it.
  Lesson: the DMA must always be serviced, even on overload (discard slot),
  or the HAL Transfer desyncs and RX dies permanently.
- **`BUF_WORDS` (16 KB halves):** the half-fill time is a latency floor under
  every RX→response cycle. With decode out of the IRQ there's no decode
  deadline, so smaller halves are viable if the per-half overhead is worth
  it. `poll_into` handles frames longer than a half via carry accumulation.
  Not tested at other sizes since the restructure.
- **`MAX_CARRY_BYTES`:** raised from 12 KB to 16 KB after post-R5 telemetry
  showed some frames getting their preamble clipped at the cap.
- **`find_active_run_from`:** walks every run in an image, not just the
  longest. That fixed loss when two frames landed in the same DMA half.
- **Open-loop packer optimizations:** single pass with no intermediate bit
  `Vec`; the sample bound is hoisted out of the loop; the offset strides by 6
  instead of recomputing `f + 4 + 6k`; unchecked reads over a proven range;
  decode capped at the header-declared length. Dropping a partial trailing
  byte matches the old `avail / 8` truncation.
- **`sample-rate-20mhz`** (FCS-ceiling triage, experiment 5): 1 sample per
  half-bit, matching Niccle's pipeline. This is the Nyquist minimum, so edge
  tracking can't work and the feature forces open-loop.
- **`mss-clamp`** (`docs/rx-bulk-ceiling.md` §5/§9/§10): clamped MTU to 1000
  to dodge the "decode cliff". Obsolete as a lever since 2026-06-10: full-MTU
  RX now beats any clamped value. Kept only as an experiment knob.
- **`INBOX_SLOTS = 4`:** covers two concurrent flows (e.g. ping + UDP echo)
  landing several frames in the same DMA half.
- **`FRAME_SNAP_BYTES = 128`:** dst/src MAC + EtherType + IPv4 header + a few
  payload bytes, matching the `main.rs` hex-dump width.

## DPLL decoder (`eth_rx_dpll.rs`)

- Phase 3b CPU DPLL port of `decode_edge_track` from
  `tools/clock-recovery/harness.py`. Validated against the corpus: FCS-OK N/N
  with flat per-byte error bins.
- The naive port cost ~3–9 ms/frame on the wire, a problem while decode ran in
  the IRQ. Second pass: `get_unchecked` after proving the bound, `find_edge`
  inlined and unrolled for W=1, and a decode-length cap from the IP header.
- Third pass (2026-06-10): the W=1 window for `next_center = tr+6` is 4
  contiguous samples (`tr+4..=tr+7`), and the resynced data bit (`new_tr - 1`)
  always lands inside it. So each bit needs one two-byte load, an XOR, and an
  8-entry table, replacing 5 separate sample reads per bit.
- Window sweep: W=1/2/3 all gave the same ~50% full-MTU FCS-OK, so failures
  weren't jitter beyond ±W. W=1 is the cheapest. (Measured before the
  DMA-starvation fix, so the 50% was mostly sample loss.)
- Tie-break and boundary coasting match Python `find_edge` (strict `<`,
  `hi = min(ns-1, center+W)` clamp).

## MAC (`eth_mac.rs`)

- **`max_burst_size`** (2026-06-10, wired rig, immediate-ACK sink in
  `main.rs`): smoltcp clamps the advertised TCP window to
  `max_burst_size × MSS` (`iface/packet.rs`), so this is the RX-of-bulk
  pipelining knob. It does not limit TX.
  - `Some(1)`: ~135 KB/s. Serialized, one segment per ACK round trip.
  - `Some(2)`: ~183 KB/s. The host pipelines full-MTU segments; ~27% decode
    loss is absorbed by TCP fast retransmit.
  - `Some(4)`: ~178 KB/s. No gain, more loss (~32% FCS-fail).
  - `Some(2)` is BDP-matched for half-duplex 10BASE-T.
- **TX token clamp:** smoltcp's own egress is capped at the IP MTU, but R17
  NAPT forwarded frames pass raw frame lengths up to `MAX_FRAME_BYTES`. The
  old `[u8; MTU]` buffer overflowed on a 1514 B forwarded frame and halted.
- **Perf span in `DMA_IRQ_0`:** perf step 2 bracketed the IRQ so core-1 RX
  load showed in `mcycle`. The NIC build omits it to keep the proven hot
  path unchanged. See the open issue above.

## TX (`eth_tx.rs`)

- **Port origin:** `ser_10base_t.pio` and `udp.c` from the C reference repo.
  `MANCHESTER_TABLE` is copied verbatim from `udp.c`.
- **CRC before PIO writes:** computing the bitwise CRC mid-flight (~27 µs
  between the body and FCS pushes) underran the 8-deep TX FIFO (~6 µs to
  drain). The line stalled mid-frame and the host saw bad FCS.
- **Critical section around FIFO writes:** added at R6, when the RX decoder in
  `DMA_IRQ_0` could preempt the TX loop for ~100 µs. Since R12c that IRQ runs
  on core 1, but the section still blocks other core-0 interrupts. Cost is
  ~50 µs for a max-size frame.
- **IFG padding (12 idle words ≈ 9.6 µs):** without it, back-to-back smoltcp
  egress (queued ARP→ICMP, or ICMP reply then UDP) left < 9.6 µs between
  frames and the host scored the second as bad FCS. This was a regression
  versus polled mode.
- **NLP padding:** if `iface.poll` sends a frame right after the NLP tick, the
  preamble lands in the host's post-NLP/IFG window and fails FCS.
- **Phase 3d carrier sense:** Phase 3c's multicore RX removed the implicit
  carrier sense (gotcha #10), exposing collisions (idle TCP 596 → 45 KB/s).
  A PIO SM now watches RO; the TX gate is in software.
- **Phase 3e CSMA/CA:** after carrier sense, the main residual was a
  synchronized start. The Pico finishes a segment, the wire frees, and the
  Pico's next segment and the host's ACK start together. A random 0–15 µs
  backoff per TCP frame breaks the tie. Curl's TCP segments go through
  `send_raw_frame`, so collisions concentrated there.
- **`raw_frame`** was a `static mut`; now owned by `EthTx` (no aliasing hazard,
  no Rust 2024 hard error).
- **Dividers at 150 MHz sys_clk:** TX ÷7.5 and RX ÷2.5, each ±3.3 ns jitter,
  within 10BASE-T tolerance. Integer at 240 MHz.
- **`full-duplex`:** see `docs/full-duplex-analysis.md` §7. Only correct
  against a peer forced to 10M full duplex.

## Multicore (`multicore_riscv.rs`)

- The first Phase 3a attempt used `rp235x-hal` 0.4's `Multicore::spawn`. It
  pokes `ICB.ACTLR` (`enable_actlr_extexclall`) and reads `PPB.VTOR`. On
  Hazard3 the Cortex-M PPB is powered down, so VTOR reads garbage and the
  ACTLR write faults. Core 0 hung in `read_blocking()`
  (`docs/cpu-dpll-plan.md` §9a).
- RISC-V specifics come from pico-sdk `pico_multicore/multicore.c`. The
  bootrom protocol is `[0, 0, 1, vector_table, sp, entry]` (RP2350 datasheet
  §5.3 / §5.5.5). The trampoline mirrors pico-sdk byte for byte.
- The stack guard (`stack_bottom`) was skipped for the Phase 3a bring-up.
- Core reset reads `frce_off` back, as in HAL `spawn` and pico-sdk
  `multicore_reset_core1`, which also fences buffered APB writes.

## CPU counters (`cycles.rs`)

- Perf characterization step 2 (`docs/perf-characterization-plan.md` §2).
  `FWD_BUSY` is the fraction of core-0 wall clock spent forwarding, not total
  core-0 load (executor, smoltcp, and cyw43 SPI are outside the spans). This
  is the "cycles/sec in the routing path" from `docs/router-plan.md` §8.3.
- LAN-isolation step 4 (§3.5) added `CYW43_SPI_BUSY` and `LAN_NET_BUSY`.
  `spi0 + net0` ≈ core-0 load in a LAN-only test. The gSPI busy-poll was the
  prime suspect (decision matrix row 3 → gSPI DMA, §4-G); the 2→15 MHz gSPI
  fix came out of this.
- `permille_over` divides by measured µs because on the first LAN run core 0
  saturated, `Timer::after(1ms)` slipped, the `n % 1000` window stretched to
  several seconds, and a fixed 1 s divisor read >100%.
- `enable_mcycle` was verified not to fault on RP2350. `mcycle` wraps every
  ~18 s at 240 MHz, ~28 s at 150 MHz.

## cyw43 adapter (`cyw43_phy.rs`)

- R14.3. cyw43's `NetDriver` implements only the async
  `embassy_net_driver::Driver`. The sync `try_rx_buf`/`try_tx_buf` API lives on
  the producer-side `ch::Runner`, which cyw43's `Runner` owns and never
  exposes. This corrected `docs/router-plan.md` §12.1, which assumed otherwise.
- No `embassy-net` dependency: one smoltcp stack serves both interfaces.
- `CYW43_TX_BUSY` (LAN-isolation step 3, §3.5): a high count under `/bulk`
  download means cyw43 TX buffering or the gSPI Runner is the wall, not the
  radio. Near-zero TX-busy with low throughput points at the air.
- No RX-drop counter: cyw43's `runner.rs` drops inbound frames silently
  (`try_rx_buf()→None`, only a defmt `warn!`). At the smoltcp boundary
  `receive()→None` just means idle. RX loss is inferred: low sink KB/s with
  low `net0`/`spi0` means the radio; with core 0 pinned means we can't drain.
- `link_up` was folded back from pico-remote-probe's copy when the module
  moved into the library.

## DHCP server (`dhcp_server.rs`)

- R14.4. smoltcp ships only a DHCP client; the server reuses smoltcp's
  `DhcpRepr`/`DhcpPacket` codec (`proto-dhcpv4`) instead of hand-rolled BOOTP.
- R18 added the DNS option. No DNS relay needed: R17 NAPT forwards LAN→WAN
  UDP, so queries to the offered resolver are NAT'd like any flow
  (`docs/router-plan.md` §6.4). The default 8.8.8.8 lets resolution work
  before the WAN lease lands.
- `POOL_LEN` is public so the R18 mgmt page can't silently undersize its body.

## WAN client (`wan.rs`)

- R15a: blocking `main_10bt` loop (`--features wan-dhcp`). R15b: the
  executor's `wan_task` (`--features router`), beside the cyw43 LAN. Same
  functions drive both. Design: `docs/r15-plan.md` §5/§6.
- Ping to 8.8.8.8 and resolving `example.com` were the R15 acceptance checks
  (#3 is DNS). NAPT/forwarding came later (R16/R17).

## Small modules

- **`pico_reset.rs`:** R0–R8 flashing needed a manual BOOTSEL press or OpenOCD
  (RESUME.md gotcha #4). picotool sends a Class-type request
  (`bmRequestType=0x21`) to the vendor interface. TinyUSB's vendor driver
  dispatches both types, so pico-sdk works either way; usb-device routes
  strictly, so we accept Class and Vendor.
- **`crc.rs`:** bitwise, no table. At <1K frames/s it costs ≈100 µs/s at
  150 MHz.
- **`lib.rs`:** the DPLL was productized in Phase 3b; `cyw43_phy` (R14.3) is
  behind the narrow `cyw43-phy` feature so consumers skip the full wireless
  stack.

## Conntrack (`conntrack.rs`)

- R17. The incremental checksum helpers (`add1c`, `checksum_incr`) were
  verified offline against full recompute before landing: a known IPv4
  vector, 250k random rewrites, and one's-complement carry edges
  (`docs/r17-plan.md` §5).

## Forwarding (`forward.rs`)

- R16 added L3 forwarding (design: `docs/r16-plan.md`). smoltcp silently
  drops frames to our gateway MAC with a foreign dst IP, and its neighbor
  cache is private, so forwarding lives beside the two `Interface`s.
- R17 added NAPT in the same device. `WAN_CT` is a static, not an
  `Option<Conntrack>` field, so the LAN device carries no cost.
- Neighbor learning uses ARP as well as IPv4 sources because the useful IPv4
  traffic (ping replies) has off-subnet source IPs.
- R19 cold-start fix: the first forwarded LAN→WAN frame was dropped
  (`[Fwd] drop`) while `WAN_NEIGH` was empty. `wan_task` now ARPs the gateway
  as soon as a lease provides one, then once a second until learned (robust to
  a reply lost on the half-duplex wire), then stops.
- `critical_section` also covers the TIMER0 IRQ (time driver).

## Firmware entry (`main.rs`)

- Build history: R4.4 smoltcp, R4.6 UDP echo, R7 HTTP server, R12c/R12e
  production loop, R14.1 split `main_10bt` out so `wireless` can replace the
  data path, R15a `wan-dhcp`, R15b router dispatch. Host test recipes for the
  static IP live in RESUME.md.
- **Watchdog** (`docs/rx-bulk-ceiling.md` §6): the device wedged under
  sustained full-MTU inbound (link drops, no NLPs, CDC silent, only SWD could
  recover it). A 6 s hardware watchdog fed from the core-0 loop or an executor
  task gives ~12× margin over legitimate stalls (TX critical section ~50 µs,
  cyw43 gSPI bursts ~ms). The root cause of the hang is still open.
- **240 MHz overclock** (Phase 2d v3): integer PIO dividers remove fractional
  jitter. Flash corruption at the higher QMI SCK is recoverable via SWD.
  `clock-150mhz` was FCS-ceiling triage experiment 6.
- `CORE1_TICKS` (Phase 3a) proved core 1 launched and that SRAM is coherent
  across cores: plain atomic store/load (`sw`/`lw`), no lr/sc reservations.
- **Bulk-test 32 KB send window** (Phase 3e): residual carrier-sense-gap
  collisions (no true collision detect) trigger fast retransmit (~ms) instead
  of an RTO stall (~200 ms), which was what drove throughput variance.
- **fd-bench sink immediate ACK** (branch `rx-ack-pacing`): with the small
  advertised window, the 10 ms delayed-ACK timer dominated the per-segment
  cycle (~15 ms → ~96 KB/s). Worse, its fixed phase sat on Linux's tail-loss
  probe timer (`max(2·srtt, 10 ms)`): host probe retransmits and our delayed
  ACKs collided on the half-duplex wire, a 27–33% FCS-fail floor that fell to
  3–5% (1-segment window) once ACKs went immediate. Coalescing
  (`ack_delay = 1 ms`) was strictly worse (~54–78 KB/s, 43% fail): a
  timer-fired ACK lands mid-stream of the next inbound segment.
- **USB serial = chip ID** (gotcha #4): with a static string, `picotool -f`
  rebooted the chip into BOOTSEL but couldn't match the BOOTSEL device and gave
  up.
- **`http-bulk-test`** (FCS-ceiling triage, experiment 4): 1 MB over HTTP to
  compare against Niccle's 620 kB/s at "0 invalid CRC". The 0x55 payload
  matches `/tmp/blast_udp_full_mtu.py` so per-byte error scans share a basis.
  An old comment claimed an 8 KB TX buffer; it is 32 KB.
- fd-bench's loop-iteration counter separates a loop/`max_burst` rate cap
  (iters/s ≈ frames/s) from a TCP-level cap. The `mcycle` counters stay
  router-only.

## Wireless (`wireless.rs`)

- **R13 "Option A":** keep Hazard3 + `rp235x-hal` and bridge to embassy with a
  time driver, the RISC-V executor, and our own PIO1 gSPI (no `embassy-rp`, no
  `cyw43-pio`). The module was first written as a compile-only milestone
  before the Pico 2 W arrived.
- **Bring-up sequence (R13):**
  1. Bit-bang probe of TEST_RO (expect 0xFEEDBEAD), logged over the existing
     10BT CDC/UDP telemetry.
  2. PIO1 probe, since a CPU bit-bang can't match the timing the chip's input
     synchronizer expects.
  3. The PIO probe also read floating data on a MicroPython-verified board, so
     a pin self-test confirmed the pads could be driven.
  4. Gotcha #11: a floating CLK/DATA during WL_ON power-up can latch the CYW43
     into the wrong gSPI mode. Holding the bus idle before power-up (as
     embassy does) was "the one thing we got wrong vs the driver".
  5. Step 2: real `cmd_read`/`cmd_write`. Step 3: bring-up via `block_on`.
  6. R14.1: a persistent executor with a continuous Runner, which `block_on`
     couldn't provide once it returned.
- **gSPI clock:** bring-up ran at 2 MHz (PIO 4 MHz). The LAN-isolation run
  (`docs/perf-characterization-plan.md` §3.5) showed that 2 MHz bus (≈250 KB/s
  raw), not the radio, capped throughput (download 168 KB/s). Raised to 15 MHz
  gSPI (30 MHz PIO; ÷8 at 240 MHz, ÷5 at 150 MHz): download 168 → 909 KB/s,
  upload 30 → 716 KB/s. embassy runs the same 2-cycle program at ~33 MHz;
  30 MHz gSPI is an optional next step.
- `wait_for_event` still actively polls; the host-wake IRQ isn't wired (idle
  `spi0` ~72%).
- **`cdc_write_all`** (R16): `serial.write` returns a partial count. The longer
  `[Wan]` line overflowed the ~128 B usbd-serial IN buffer and truncated
  counters. This was also the long-standing "CDC drops bytes under load".
- **Measured rate windows:** on the first LAN run core 0 saturated,
  `usb_task`'s 1 ms cadence slipped, and fixed-1 s rates over-read
  (`spi0` > 100%).
- Milestones: R14.2 AP, R14.3 LAN `Interface` (acceptance: a client at
  192.168.4.2 pings .1), R14.4 DHCP, R14.5/R18 mgmt page, R15b router, R16
  forwarding, R17 NAPT, R18 DNS offer, R19 gateway pre-ARP.
- The mgmt HTTP 32 KB TX buffer at a 5 ms poll gives a ~6.4 MB/s ceiling,
  above any 2.4 GHz rate, mirroring the 10BT `http-bulk-test` window.
- The standalone `wireless` image doesn't start 10BASE-T
  (`docs/router-plan.md` §11/§12). Gotcha #5: the host must assert DTR to see
  CDC output. Gotcha #9: NLP keepalive.

- **Removed 2026-09-14:** the unused R13 bring-up code: the bit-bang probe
  (`probe_cyw43`, `bitbang_cmd_read`, `PHASE`, `gpio_read`), the PIO probe
  (`probe_cyw43_pio`, `pio_cmd_read32`), the pin self-test (`pin_selftest`,
  `CYW43_PIN_LO/HI`), the `CYW43_PROBE*` statics, `cmd_word`, `swap16`,
  `PIN_PWR`, and `cyw43_bringup_blocking` (the `block_on` bring-up) with its
  `embassy-futures` dependency. Also the `[Cyw43]` block in `main.rs`'s
  `log_status`, which could never run. Recover them from git history before
  that date.

## Build features (`Cargo.toml`)

- **`diag`:** off by default to keep the log short and the binary small. The
  `[R2b]` heartbeat and `[Rx]` summary always print.
- **`decoder-openloop`** (FCS-ceiling triage, post-R10): swaps the DPLL for the
  pre-R10 open-loop decoder at the same call site, for on-wire comparison with
  Niccle's fixed-stride pipeline. The default binary is byte-identical.
- **`sample-rate-20mhz`** (experiment 5): tests whether 60 MHz oversampling
  exposes transient noise that a 20 MHz fixed-stride pipeline can't see.
- **`clock-150mhz`** (experiment 6): rules out supply-noise or VREG-margin side
  effects of the overclock. Combined with the decoder features it maps the
  6-cell decoder × clock matrix.
- **`http-bulk-test`** (experiment 4): replaces the R8 info page with a 1 MB
  stream to compare against Niccle's 620 kB/s at "0 invalid CRC". If throughput
  matched while our FCS counter read 30–70% fail, hypothesis #4 (their counter
  masks silent SFD-lock failures) would make the gap paper-only.
- **`full-duplex`:** the ISL3177E is full-duplex capable; half duplex is a MAC
  policy. A duplex mismatch is worse than half duplex.
- **`fd-bench`** (`docs/full-duplex-analysis.md` §7.3, Tier 2): push bulk into
  the device while `http-bulk-test` streams out, to measure the concurrent
  aggregate (H3) and the core-1 decode ceiling (H4). Used for both the
  HD-bidir control and the FD-bidir run.
- **`mss-clamp`** (`docs/rx-bulk-ceiling.md` §5): kept inbound frames under the
  supposed ~600 B clock-drift decode cliff. That cliff was later found to be
  DMA starvation.
- **`wan-dhcp`** (R15a) and **`router`** (R15b): `wan-dhcp` leaves the static-IP
  NIC build and its R4–R8 host recipes unchanged. Design: `docs/r15-plan.md`.
- **`cyw43-phy`:** exists so pico-remote-probe's `wifi` build can reuse the
  adapter with its own runtime.
- **`wireless`** (R13): "Option A", keeping Hazard3 and porting the cyw43 SPI
  transport (`docs/router-plan.md` §4/§5). `embassy-executor` is decoupled
  from `embassy-time` and has the riscv32 backend Hazard3 needs.
- **`embassy-time` `generic-queue-16`:** a fixed-capacity queue that works with
  any waker, independent of the executor's timer storage. It is the robust
  choice for a hand-rolled driver outside embassy-rp.
- **`embassy-net-driver` 0.2.0** must match cyw43 0.7.0 exactly, or the
  `Driver` trait is a different type (R14.3 wraps it with a no-op waker).
