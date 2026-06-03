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

## 3.5 — Isolating the cyw43 LAN link (NEXT — the first routed run pointed here)

**First routed run, 2026-06-03** (rig = this host as Wi-Fi client `wlx` + WAN
gateway `enp1s0f0`, iperf3 server on a separate `192.168.0.80` via `eno1`):

- A **rig routing loop** had to be fixed first — the `/32 SRV-via-Pico` route
  (for the client) also caught the packets this host *forwards as the gateway*
  (the Pico's NAT'd frames), bouncing them back to the Pico to be re-NAT'd in a
  loop (this one host is both client AND gateway). Fixed with **source policy
  routing** (`ip rule from $LEASE_IP lookup 100`) in the rig scripts. See
  `perf-char-step3-rig-loop` memory. The forwarding code is byte-identical to the
  validated R19a — it was never the bug.
- Post-fix the routed path works: ICMP 4/4, clean TCP handshake, `[Nat] out≈in`.
  **Routed TCP upload ≈ 349 Kbit/s (~44 KB/s)**, lossy (SACK recovery, ~13
  retr/3s). **TCP download (WAN→LAN bulk) stalls and *wedged* the cyw43 LAN**
  (client lost the AP; the Pico stayed up `ap=1 net=1`, no panic) — a separate
  robustness item.
- **Key: the Pico was nearly idle** — `core0(forward)=0.2%`, `core1≈34%` (the
  ambient-10BT-decode baseline), forward drops negligible during the clean run
  (`nh +3 qf +1`; the large `nh`/`macmiss=192.168.4.1` was stale loop-era
  residue). So the ~13× gap vs the bare-10BT 596–987 kB/s is **NOT** the Hazard3
  cores or the NAPT/forward path. The loss is in the **cyw43 Wi-Fi LAN or the
  10BT WAN link**, uncounted by `FWD_DROP`.

**Why isolate the LAN:** the routed path conflates LAN(wifi)+forward+WAN(10BT);
the 10BT alone does 596–987 kB/s, so the prime suspect is the cyw43 RX/TX path.
44 KB/s is *suspiciously* low for 2.4 GHz 11n — must distinguish "cyw43's real
ceiling" from "a loss/buffering bug under burst." **The LAN-only rig is far
simpler than §3:** no WAN host, no `$SRV`, no `/32`, no NAPT — just `wlx` ↔ the
Pico AP, traffic terminating *on the Pico* (none of the loop/double-NAT fragility).

**Instrumentation — BUILT 2026-06-03** (compiles all 4 configs + clippy clean,
only the 2 pre-existing warnings; production NIC build byte-unchanged — the new
code lives only in the `wireless`/`router` modules; the per-core CPU% (step 4) is
router-gated since it uses the router-only `cycles`). New readouts: a **`[Lan]`
CDC line** (`tx=KB/s rx=KB/s txbusy=N rxframes=N` + router `spi0=% net0=%`) and
mgmt-page rows (`LAN perf:` totals + `LAN cpu0:` split). **The device must be
reflashed with this router build** before the rig run (it's the current
`release/` ELF; the [[flash-wrong-feature-build-gotcha]] verify = `strings <ELF> |
grep -F "[Lan]"`).
1. **LAN bulk *source*** (download = Pico→client, cyw43 TX) — DONE: a `GET /bulk`
   route in `wireless::serve_status_http` streams `LAN_BULK_BYTES` (8 MB) of 0x55
   filler via a persistent `LanHttp::Bulk{remaining, header_sent}` state (mirrors
   `main.rs`'s `serve_http_bulk`). The `:80` TX buffer was bumped 2.5→32 KB so the
   stream stays cyw43-TX-limited, not net_task-5ms-cadence-limited (32 KB/5 ms ≈
   6.4 MB/s ceiling). Counts `LAN_BULK_TX_BYTES`. Drive:
   `curl http://192.168.4.1/bulk >/dev/null` and read the steady-state `[Lan] tx=`
   (Ctrl-C once it settles — 8 MB needn't finish).
2. **LAN *sink*** (upload = client→Pico, cyw43 RX) — DONE: a TCP listener on
   **port 9999** in `net_task` (`serve_lan_sink`) read-drains + counts every byte
   (no echo) into `LAN_SINK_RX_BYTES`, re-listening per connection. 32 KB RX
   buffer so the advertised window doesn't throttle. Drive (iperf3-free):
   `head -c 64M /dev/zero | nc 192.168.4.1 9999` and read `[Lan] rx=`.
3. **cyw43 TX-backpressure counter** — DONE (`CYW43_TX_BUSY` in `src/cyw43_phy.rs`,
   incremented on `transmit()→None`). ⚠️ **Reframed after reading the cyw43
   source:** there is **no observable RX-drop counter**. `Cyw43Phy::receive()→None`
   is dominated by *idle polls* (the cyw43 `NetDriver::receive` returns `None`
   whenever no RX frame is ready — hundreds/sec — and *also* when the TX half is
   full; it can't be decomposed at the `phy` boundary), so counting it would be
   noise, not drops. The *real* cyw43 RX drop happens **inside** cyw43's `Runner`
   (`runner.rs`: `try_rx_buf()→None ⇒ silently drop + a defmt warn!` we don't
   capture) — upstream of us, uncounted. So `transmit()→None` is the one genuine
   TX-backpressure signal (high under `/bulk` ⇒ cyw43 TX is the wall), and the
   **RX-side discriminator is sink-throughput-vs-`spi0`/`net0`** (step 4), not a
   device counter: low sink kB/s + low core-0 ⇒ the air/radio; low sink kB/s +
   pinned core 0 ⇒ we can't drain fast enough and cyw43 drops upstream. (This
   replaces the "high RX-drop counter ⇒ buffering bug" matrix row with a
   throughput+CPU read — same decision, different evidence.)
4. **CPU during LAN-only load** — DONE: two router-gated `CycleSpan`s on core 0,
   since step-2 `FWD_BUSY` only brackets forwarding (≈0 here). `CYW43_SPI_BUSY`
   wraps the busy-poll gSPI transport (`PioSpiCyw43::cmd_read`/`cmd_write`, the
   cyw43 Runner's real cost) → `spi0%`; `LAN_NET_BUSY` wraps `net_task`'s per-poll
   body (smoltcp + handlers + `Cyw43Phy` channel ops) → `net0%`. `spi0+net0` ≈
   core-0 utilisation under a LAN test; `spi0` high ⇒ the busy-poll transport is
   CPU-bound (matrix row 3 → gSPI DMA, §4-G).

**LAN-isolation run — RESULTS (2026-06-03, `--features router`, SWD-flashed; host
`wlx` joined the AP, source-bound to the lease IP `192.168.4.10` so traffic takes
table-100 → `wlx`, i.e. the cyw43 LAN, NOT the 10BT WAN):**

| Direction | Throughput (client-measured) | core-0 `spi0` | `net0` | `cpu1` | notes |
|---|---|---|---|---|---|
| **idle** (no traffic) | — | **80–98%** | ~0.4% | ~40% (ambient 10BT) | `spi0` pegged *with zero traffic* |
| **download** (`/bulk`, cyw43 **TX**) | **~168 KB/s** | ~90–95% | ~2% | ~40% | `txbusy` climbing (real TX backpressure) |
| **upload** (`:9999` sink, cyw43 **RX**) | **~30 KB/s** | ~90% | ~1% | ~40% | `txbusy` frozen (RX-heavy; Pico only ACKs) |

**Verdict — the ceiling is our gSPI TRANSPORT, not the 2.4 GHz radio (matrix row
3 → §4-G, NOT radio modularization §4-B):**
- **`spi0`≈80–98% even at idle** — the cyw43 `Runner` *active-polls* the chip over
  gSPI continuously (its `wait_for_event` uses the default poll impl; the
  host-wake IRQ line is **not wired** — see `PioSpiCyw43`). Core 0 is near-pegged
  by the transport before any data moves.
- **The gSPI clock is ~2 MHz** (`build_gspi_sm`, "slow + safe for bring-up", never
  raised; embassy's `cyw43-pio` runs ~33 MHz). 2 MHz half-duplex = **~250 KB/s raw
  bus ceiling** — and download (168 KB/s) sits right under it. The radio was never
  the limiter; the bus was.
- **Asymmetry (TX 168 ≫ RX 30):** inbound frames wait for the active-poll cycle to
  notice them (no host-wake IRQ ⇒ RX latency), and each TCP ACK is a half-duplex
  gSPI write stealing from reads. Both point back at the transport.
- **Measurement-bug found + fixed mid-run:** the 1 Hz telemetry assumed each
  `n%1000` window = 1.000 s, but core-0 saturation slips `usb_task`'s cadence, so
  the first run over-read (`spi0=607%`, `cpu1=249%`, `tx=1137KB/s`). Fixed by
  normalising every rate/% to the **measured** elapsed µs (`cycles::permille_over`);
  re-run device `tx=174KB/s` now agrees with client `168KB/s`. (Also fixes the
  pre-existing step-2 `cpu1/cpu0` under load.)

**▶ The actionable lever is §4-G transport work, in priority order:** (1) **raise
the gSPI clock** 2 MHz → toward ~33 MHz (≈16×, the single biggest win — lifts the
raw ceiling), (2) **wire the cyw43 host-wake IRQ** so the Runner stops active-polling
(frees core 0 + cuts RX latency), (3) **gSPI DMA** (offload the busy-poll). Radio
modularization (§4-B) is **NOT** justified by this data — the air was never reached.

**gSPI clock bump — CONFIRMED 2026-06-03** (`build_gspi_sm` `GSPI_PIO_HZ`
4 MHz → 30 MHz, i.e. gSPI **2 → 15 MHz**, ÷8 at 240 MHz; cyw43 handshake still
passes — `new=1 init=1 ap=1 net=1`, WAN/ping healthy, no instability):

| Direction | 2 MHz gSPI | **15 MHz gSPI** | gain |
|---|---|---|---|
| download (cyw43 TX) | 168 KB/s | **909 KB/s** | **5.4×** |
| upload (cyw43 RX) | 30 KB/s | **716 KB/s** | **24×** |
| idle `spi0` | ~90% | ~72% | — |

Both directions now ~700–900 KB/s — on par with the bare 10BT NIC (596–987 KB/s)
and within the WAN's ~1.1 MB/s envelope, so **the cyw43 LAN is no longer the router
bottleneck.** Proof-positive that the *transport clock* (not the radio) was the
ceiling. Headroom remains: 15 MHz raw = 1.875 MB/s, so we're at ~48% (TX) / ~38%
(RX) of raw — the rest is the active-polling overhead + half-duplex ACKs. Next
levers: push gSPI → 30 MHz (÷4, optional — already beats the WAN), then **wire the
host-wake IRQ** to reclaim the idle `spi0`≈72% (the Runner still active-polls — a
core-0 + low-power win, no longer throughput-limiting). Uncommitted working tree.

**Independent baseline (would further rule out the `wlx` adapter / air):** iperf3
the host's `wlx` against a *known-good* AP (a normal router, same channel/distance).
Not yet run — but the idle `spi0`≈90% + the 2 MHz bus ceiling already pin the limit
on the transport regardless, so this is now confirmatory, not load-bearing.

**Decision matrix (the radio-modularization gate, §4-B):**

| Isolated LAN result | Interpretation | Action |
|---|---|---|
| ≫ 44 KB/s (multi-Mbit), low txbusy | routed slowness is the LAN↔WAN *interaction* (10BT half-duplex backpressure / MSS), not the radio | don't swap the radio; chase the interaction (MSS clamp §4-E, core balance §4-G) |
| download (`/bulk`) ≈ low, **high `txbusy`**, core 0 not pinned | cyw43 *TX* buffering/backpressure is the wall | software fix (bigger TX queue / gSPI DMA §4-G) before any hardware |
| up/down ≈ low, low txbusy, **core 0 pinned (`spi0`/`net0` high)** | the busy-poll cyw43 SPI transport is CPU-bound (can't drain fast enough → cyw43 drops RX upstream) | gSPI DMA (§4-G) / core rebalance |
| up/down ≈ low, low txbusy, **core 0 idle**, ≈ the wlx-vs-good-AP ceiling too | the 2.4 GHz radio / air link is the real ceiling | **radio modularization (§4-B) is justified** |

**Also characterize the download-wedge** (WAN→LAN bulk stalled + dropped the AP):
cyw43 TX backpressure (the `transmit()`→`None` ⇒ drop-the-RX-frame path in
`forward.rs::receive`), the `WAN_TO_LAN` channel (depth 4) saturating, or the AP
dropping the client under load? The LAN bulk *source* test (#1) exercises cyw43 TX
in isolation — if it also stalls/wedges, the TX path is implicated, not the WAN.

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
- **Step 3 (first routed run): DONE 2026-06-03** — fixed the rig loop (source
  policy routing, in the committed rig scripts), got the routed path working, and
  took the first numbers (§3.5). Headline: **routed TCP up ≈ 44 KB/s, Pico CPU
  idle (`core0=0.2%`) ⇒ link-limited, not CPU/forward-limited.** The bottleneck is
  the cyw43 LAN or the 10BT link.
- **Step 4 — LAN-isolation instrumentation: BUILT + RUN 2026-06-03** (uncommitted
  working tree). `/bulk` source + `:9999` sink + `CYW43_TX_BUSY` + `spi0`/`net0`
  core-0 spans → a `[Lan]` CDC line + mgmt rows; all 4 configs + clippy clean.
  **Flashed + measured** (§3.5 results table): download **168 KB/s**, upload **30
  KB/s**, core-0 `spi0`≈90% (even ~90% at idle). **Verdict: the ceiling is the
  ~2 MHz busy-poll gSPI transport, NOT the radio** → §4-G (raise gSPI clock / wire
  host-wake IRQ / gSPI DMA), **not** radio modularization §4-B. Also fixed a
  rate-normalisation bug (1 Hz window stretches under core-0 load → over-read;
  now normalised to measured µs via `cycles::permille_over`).
- **gSPI clock bump — DONE + CONFIRMED 2026-06-03** (`GSPI_PIO_HZ` 4→30 MHz =
  gSPI 2→15 MHz): download **168→909 KB/s** (5.4×), upload **30→716 KB/s** (24×);
  handshake/WAN/ping all healthy. The cyw43 LAN is no longer the router bottleneck
  (now ~on par with the bare 10BT NIC). See the §3.5 results table.
- **▶ NEXT:** (a) optional — push gSPI 15→30 MHz (÷4) for more headroom (already
  beats the WAN, so low urgency); (b) **wire the cyw43 host-wake IRQ** so the
  Runner stops active-polling — idle `spi0` is still ~72% (wasted core 0 + a
  low-power cost), no longer throughput-limiting; (c) commit the LAN-isolation
  instrumentation + the gSPI bump + the rate-normalisation fix (held for the
  user). Optional confirmatory: iperf3 `wlx` vs a known-good AP — not load-bearing.
- **Push held at the user's request.** Per git, local `main` is 2 ahead of
  `origin/main` (`1d1a1a1`): `a58d51f` (instr step 2) + the rig-split tooling
  (this). The older "`b32042d`/`0285e31` unpushed" note looks stale — `origin/main`
  already contains both (confirm with `git fetch`).
