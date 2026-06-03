# Performance characterization (+ the post-router backlog)

The router is feature-complete + robust through R19a. Low-power (roadmap E) is
**scoped but backburnered** (`docs/low-power-plan.md`, gated on a power meter).
The active track is now **characterizing the router's performance** — and the
expanded backlog of post-router directions is captured at the end of this doc.

---

## 1. Goal

Measure the **full routed / NAT'd path** — WiFi client → Pico NAPT → 10BASE-T
WAN — which has **never been measured**. The R11/R12 numbers (596–987 kB/s) were
the 10BT NIC *in isolation*, not transit through the AP + NAPT.

The headline question: **where is the routed-throughput ceiling?** — the cyw43
2.4 GHz 11n radio, the 10BT WAN, a saturated Hazard3 core, or conntrack. That
answer decides whether the **radio-modularization** work (below) is worth it: the
user's motivation for it is *higher throughput*, so it only pays off if
measurement shows the cyw43 radio is actually the bottleneck.

---

## 2. Instrumentation — the prerequisite (build before measuring)

### Step 1 — data-plane counters ✅ DONE (`0285e31`, on `main`, flashed)
`src/forward.rs`:
- per-direction egressed bytes (`FWD_BYTES_TO_WAN`/`_TO_LAN`) → throughput
- `FWD_DROP` split by cause (`count_drop` helper): `qfull` / `nonh` / `nat` /
  `txbusy` / `other` — so load tests show *why* it drops
- per-direction egress-queue high-water (`FWD_QHWM_L2W`/`_W2L`, vs `CHAN_DEPTH=4`)

Surfaced as: a 1 Hz `[Perf]` CDC line in `usb_task` (per-second up/dn KB/s, pps,
queue/conntrack high-water, drop breakdown) **and** the mgmt page
(`serve_status_http` — byte totals, drop breakdown, queue hwm; body cap bumped to
`POOL_LEN*40+512`, `http_tx` 2048→2560). The mgmt page is the reliable readout (the
5th CDC line adds interleaving pressure).

On-device: `[Perf]` renders, all counters read sanely at idle (0). **Real numbers
need traffic** (the harness, §3).

### Step 2 — `mcycle` CPU utilization ✅ DONE (code complete, builds clean; on-device idle-validation pending a flash)
New `src/cycles.rs` (router-only): `mcycle()` (CSR `0xB00`), `enable_mcycle()`
(clears `mcountinhibit` CSR `0x320` — called per-core: core 0 in `main`'s router
arm, core 1 at the top of `core1_entry`), and a `CycleSpan` RAII guard that
brackets a scope and adds its `mcycle` delta (wrap-safe) to an accumulator.
- **core 1**: a `CycleSpan` around `process_completed_half` in the `DMA_IRQ_0`
  handler (`eth_mac.rs`) → `CORE1_BUSY` ≈ core-1 RX-decode utilization.
- **core 0**: a `CycleSpan` at the top of `ForwardingDevice::receive` (classify/
  skim) and `egress` (NAPT/TTL/L2-rewrite) in `forward.rs` → `FWD_BUSY` = the
  *fraction of core-0 wall-clock spent forwarding* (NOT total core-0 load — the
  executor/smoltcp/cyw43-SPI cost is outside the brackets; that's the
  "cycles/sec in the routing path" number `router-plan.md` §8.3 wants).

`usb_task` samples both accumulators once a second, divides each delta by
`SYS_CLK_HZ` (240 MHz, or 150 with `clock-150mhz`), and publishes per-mille into
`CPU1_PERMILLE`/`CPU0_PERMILLE` — surfaced as `cpu1=NN.N% cpu0=NN.N%` on the
`[Perf]` CDC line **and** a `CPU:` row on the mgmt page (the reliable readout).
Instrumentation is router-gated, so the production NIC build's hot path is
byte-unchanged. No external hardware needed (unlike low-power).

**Caveat for validation:** at idle, busy-cycles ≈ 0 *regardless* of whether the
counter is enabled — so an idle read only confirms the fields render + read sane.
Confirming the counters actually move (and the numbers mean something) needs
traffic, i.e. the §3 run. Low 32 bits wrap ~18 s at 240 MHz / ~28 s at 150 MHz →
always `wrapping_sub` deltas.

---

## 3. The measurement run (gated on the rig)

Needs: `tools/wan-test-host.sh` up (Pico holds a WAN lease); `iperf3` is already
installed. The rig is split **route-1 style** so the recurring measurement needs
no root (shared config in `tools/rig-env.sh`):
- **`sudo tools/router-rig-up.sh`** — *one-time root*: associate this host's Wi-Fi
  client to the Pico AP, lease an IP, install the `/32` route to `$SRV` via the
  Pico (route-safe — never touches the eno1 default / SSH). Leaves it associated +
  writes the client IP to `/tmp/pico-rig.env`.
- **`tools/router-measure.sh`** — *NO root, repeatable* (Claude can drive this):
  iperf3 server + client over the routed/NAT'd path (TCP down/up + UDP) + before/
  after mgmt-page snaps (`Forward|Bytes|Queue|NAT:|CPU:`).
- **`sudo tools/router-rig-down.sh`** — *root*: remove the route + deassociate.

`tools/router-throughput.sh` remains an all-as-root convenience wrapper
(rig-up → measure → rig-down). Untested end-to-end until the first live run —
refine then.

Record, idle vs under each load:
- routed **TCP** up (client→WAN) + down (WAN→client) kB/s
- **UDP** pps + loss knee (raise `-b` until loss climbs) → the pps ceiling
- the `[Perf]` counters: queue hwm (backpressure?), drop breakdown (where), ct hwm
- **CPU util** (step 2): which core, if any, pins at 100%

Cross-check against the in-isolation 10BT ceiling (~1 MB/s idle) and the cyw43
2.4 GHz practical ceiling. **Deliverable:** a table + a named bottleneck + a
recommendation (does the radio swap help? is core 0 saturated → rebalance? is it
the 10BT half-duplex? conntrack?).

Then multi-client (the user's #3): N clients associated, measure aggregate +
per-client fairness, conntrack pressure (`cthwm` vs `CT_CAP=64`), AP stability.

---

## 4. Broader backlog (brainstorm 2026-06-02, with decisions)

The user's three + additions. Decisions captured: **radio-modular is
throughput-motivated**; **start instrumentation-first** (this track).

**A. Performance characterization (#1 + #3)** — THIS track. Single-client
(throughput/latency/pps/CPU) then multi-client (fairness/scaling/conntrack).

**B. Radio/AP modularization (#2) — throughput-motivated.** A `WirelessLan` trait
(init / start_ap / address / `phy::Device`) decoupling `net_task` + the router from
cyw43, so a faster module can drop in (the transport `SpiBusCyw43` is already
abstracted; this is the layer above). **Only worth building if §3 shows the cyw43
2.4 GHz 11n radio is the ceiling.** Candidate replacements: SDIO/ESP32-C6 (WiFi 6),
5 GHz modules. Sequence *after* characterization confirms the need.

**C. Observability** (enabler, overlaps step 2): per-core CPU%, a `/metrics`
endpoint, the routed-throughput harness (started, `tools/router-throughput.sh`).

**D. Make it a real product:** mgmt UI as a control plane (the HTTP server ignores
the request line today — add routing `/stats` `/clients` `/conntrack` + actions:
kick client, clear conntrack, change AP config); config persistence in flash (SSID/
passphrase/channel + leases survive reboot; today hardcoded consts + RAM-only pool);
security hardening (mgmt page is unauthenticated + LAN-open, dev creds in source,
NAT wide open).

**E. NAT / protocol correctness** (can bite under real traffic): TCP MSS clamping
(half-duplex 10BT + large-frame tail makes full-MSS fragile — R17.x); ICMP-error
embedded-header rewriting (R17 punted → breaks PMTUD/traceroute); a real DNS relay
(vs today's NAT-passthrough → enables logging/filtering/caching).

**F. Reliability:** WAN link-loss / lease-loss recovery (a dead upstream just
stalls today); RP2350 hardware watchdog; backpressure / conntrack-pressure
(R19+); the optional ARP hold-and-retry for a true worst-case-0 (R19a deferral).

**G. Architecture / perf** (falls out of §3): core-balance rebalance (move the
forwarding fast-path or cyw43 Runner to core 1 if core 0 saturates); gSPI DMA
(replace the busy-poll cyw43 transport — cuts core-0 load + a low-power win).

**Dependency spine:** instrumentation → characterize (single → multi-client) → the
revealed bottleneck drives the next move (core rebalance / gSPI DMA / radio swap /
conntrack-pressure). The "real product" track (D) is independent + parallelizable.

---

## 5. Status / restart pointer

- **Step 1 instrumentation: DONE** (`0285e31`), flashed, idle-validated.
- **Step 2 (`mcycle` CPU util): DONE** (on `main`), all 4 configs/clippy
  clean, SWD-flashed + idle-validated (fields render; `cpu1≈42% cpu0≈0.3%` at idle —
  core 1 already busy decoding ambient 10BT, see the surprise finding above). The
  counters can only be *meaningfully* exercised under load (§3).
- **▶ NEXT:** bring the rig up (`wan-test-host.sh` + `iperf3`), run
  `tools/router-throughput.sh`, watch `cpu1`/`cpu0` under routed load, fill in §3,
  name the bottleneck.
- **Push held at the user's request.** Per git, local `main` is 2 ahead of
  `origin/main` (`1d1a1a1`): `a58d51f` (instr step 2) + the rig-split tooling
  (this). The older "`b32042d`/`0285e31` unpushed" note looks stale — `origin/main`
  already contains both (confirm with `git fetch`).
