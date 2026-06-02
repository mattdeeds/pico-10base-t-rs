# Low-power — characterization plan (roadmap E)

**Status: GATED on a power meter.** The phase goal is *characterize first* —
measure where the power actually goes before changing the always-on data path.
That needs an inline draw measurement we don't have wired yet. This doc is the
turnkey plan to execute the moment the meter is connected; **no functional code
changes land until a measurement justifies them** (the explicit "characterize
first, avoid blind optimization" decision).

`router-plan.md` §6 + §7 call low-power "a known unresolved tension": a 10BASE-T +
WiFi-AP router is fundamentally always-on and always-listening, so the big
consumers are load-bearing. The realistic aim is **reduce active draw**, not deep
sleep.

---

## 1. Goal & acceptance

**Goal:** a measured power breakdown of the running router — idle and under load —
attributing draw to the major subsystems, so we know the floor and which levers
are worth pursuing.

**Acceptance:** a filled-in version of the §4 matrix (mA at each config), a ranked
lever list backed by measured deltas, and a recommendation of which (if any) to
implement. NOT "make it use less power" yet — that's the *next* sub-phase, driven
by this data.

---

## 2. Measurement setup (the prerequisite)

The board is USB-powered today, so the low-friction path is an **inline USB power
meter** (USB-A pass-through, reads V/I/W) or a USB-PD analyzer between the host and
the Pico. Alternatives: bench PSU with current readout on VSYS, or a current-sense
resistor + DMM in series with VSYS (more precise, can isolate rails).

Caveats to control for:
- The **CMSIS-DAP debug probe** and the **USB-CDC** both draw — measure with the
  same cabling each run; if possible power the Pico from a separate USB port than
  the probe so the meter sees only the board.
- Record **steady-state** (let it settle ≥10 s); the cyw43 TX bursts and the 10BT
  TX make instantaneous draw spiky — capture average + peak.
- Fix the environment: same host, same `wan-test-host.sh` upstream state, same
  ambient. Note whether a client is associated (idle-AP vs active-LAN differ).

---

## 3. Suspected consumers (the hypothesis to test)

Grounded in the current config (to be confirmed/overturned by measurement):

| Subsystem | Why it draws | Reducible? |
|---|---|---|
| **cyw43 radio, AP `PM::None`** | An AP must stay awake to beacon + answer probes; `set_power_management(None)` keeps the radio fully on. | Hard — an AP can't deep-sleep. Maybe an intermediate PM mode. Likely the single biggest + least-reducible block → sets the floor. |
| **Always-on 60 MHz PIO RX sampler + continuous DMA** | The 10BT sampler free-runs at 60 MHz and DMAs continuously regardless of WAN traffic, so the double-buffer halves fill at a constant rate and wake **core 1** (which otherwise `wfi`s) constantly. | Maybe — gate sampling on the R12d carrier-detect SM when the WAN is idle. **Highest-value firmware lever** *if* measurement shows it's significant. Risk: missing frame starts. |
| **sys_clk 240 MHz @ default VREG** | Dynamic power ∝ f·V². We run the 240 MHz overclock at the bootrom-default core voltage (VREG is never set in code). | Yes — drop to 150 MHz (`clock-150mhz`, already on-wire-proven) and possibly undervolt VREG below default. Low-risk, quadratic on V. |
| **Core 0: executor + cyw43 Runner + USB + gSPI busy-poll** | Executor wakes on 1–5 ms timers; the gSPI transport busy-spins during cyw43 transactions; USB is always enumerated. | Partly — audit that idle truly `wfi`s (core 1 does; confirm the riscv32 executor does too), DMA the gSPI instead of spinning (deferred since R11). |
| **Core 1: RX decode** | `wfi`s between DMA IRQs (already power-aware) but woken constantly by the always-on sampler (see row 2). | Coupled to the sampler lever. |

**Rough order-of-magnitude only (must be replaced by measurement, NOT trusted):**
a CYW43439 AP and an RP2350 at 240 MHz are each plausibly tens of mA; total board
draw is likely ~100–150 mA @ 5 V (~0.5–0.75 W). These figures exist only to set
expectations for the meter range — do not cite them as results.

---

## 4. Measurement matrix

Each row isolates one lever by A/B against the baseline. Measure **idle** (AP up,
no client) and **load** (a client doing a ping flood + curl, or the host blasting
the WAN) for each. Most rows need a small **measurement knob** (built only when we
get here — see §5); rows marked ✓ are already buildable today.

| # | Config | Isolates | Knob |
|---|---|---|---|
| 0 | Router build, 240 MHz, default VREG | **Baseline** | ✓ (`--features router`) |
| 1 | + sys_clk 150 MHz | clock f | ✓ (`--features router,clock-150mhz`) |
| 2 | 150 MHz + VREG stepped down (e.g. 1.10→1.00→0.90 V) | core V | build: VREG knob |
| 3 | Baseline − cyw43 brought up (radio off / not started) | the radio's share | build: skip-AP test mode |
| 4 | Baseline − 10BT RX engine (sampler+DMA+core1 halted) | the always-on sampler's share | build: skip-RX test mode |
| 5 | Baseline + carrier-gated sampler (idle ⇒ sampler paused) | sampler-gating payoff | build: the actual lever (after #4 justifies it) |
| 6 | cyw43 `PM::None` vs an intermediate PM mode | radio PM headroom | build: PM knob |
| 7 | Executor idle: confirm `wfi` vs spin | core-0 idle | audit (may need no change) |

Deltas (baseline − row) attribute the draw. Rows 3 & 4 are *diagnostic* test builds
(a non-functional router) purely to size each block; rows 1/2/5/6 are candidate
real changes.

---

## 5. Knobs to build (only as the matrix needs them)

All off by default, behavior-preserving for the production build:
- **VREG scaling** — set the RP2350 core regulator voltage at boot (HAL/PAC
  `VREG_AND_CHIP_RESET`/`POWMAN` path); a `cargo` feature or const to pick the
  level. Pair with the clock so we never undervolt at 240 MHz.
- **skip-AP / skip-RX test modes** — `cfg`-gated boot paths that bring up the
  router *without* the cyw43 bring-up (row 3) or *without* `setup_eth_mac` + core 1
  (row 4), so the meter sees the remaining draw. Diagnostic only.
- **carrier-gated sampler** (row 5) — the real lever: pause the 60 MHz sampler SM +
  its DMA when the R12d carrier-detect SM reports the WAN idle, re-arm on carrier.
  Needs care: re-arm latency must not clip a frame's preamble (measure FCS-OK +
  the existing `[Rx]` reliability counters alongside the power delta).
- **cyw43 PM mode** (row 6) — try `PowerManagementMode` variants; verify the AP
  still beacons + a client stays associated.
- **idle audit** (row 7) — confirm the embassy riscv32 executor `wfi`s when no task
  is ready (core 1 already does at `main.rs:163`); if it busy-spins, that's a free
  win.

---

## 6. Levers ranked by *likely* payoff (pre-measurement hypothesis)

1. **Carrier-gate the RX sampler/DMA when idle** — potentially the biggest *idle*
   firmware win, but only if row 4 shows the sampler is a meaningful share, and only
   if re-arm latency stays clip-free. Highest value / highest risk.
2. **150 MHz + VREG undervolt** — low-risk, quadratic on V, already-proven clock
   path. Likely the best effort/reward ratio.
3. **cyw43 PM mode** — bounded by the AP-must-beacon floor; measure to know it.
4. **idle/`wfi` audit, peripheral clock-gating, gSPI DMA** — small, cleanup-grade.

The **floor** is whatever the AP radio + minimal MCU draw, which rows 3/4 reveal —
that bounds how much any of this can achieve.

---

## 7. Risks

1. **No measurement = no characterization.** Everything here waits on the meter;
   resist "optimizing" blind (the whole point of characterize-first).
2. **150 MHz changes PIO dividers to fractional** (±3.3 ns jitter) — already
   characterized in R11 (link stayed healthy); re-confirm FCS-OK at each clock.
3. **Undervolting risks instability** — step down gradually, soak-test, watch for
   hangs/resets; back off on any flakiness.
4. **Sampler-gating risks missed frames** — pair every power reading with the
   `[Rx]`/`[Fwd]` reliability counters; a power win that drops frames is a loss.
5. **Test builds (rows 3/4) are non-functional** — they exist only to size blocks;
   don't confuse them with shippable configs.

---

## 8. Step checklist

- [ ] **(gating)** wire up an inline USB power meter (or VSYS current sense); record
      the baseline (row 0) idle + load.
- [ ] Run rows 1, 3, 4, 7 with today's buildable/diagnostic knobs → first breakdown.
- [ ] Build the VREG knob; run row 2 (clock × voltage sweep).
- [ ] Decide from the data whether the sampler-gate (row 5) and/or cyw43 PM (row 6)
      are worth building; if so, build + measure them *with* reliability counters.
- [ ] Write up the measured breakdown + a recommendation; update RESUME + this doc.
- [ ] Only then implement the justified lever(s) as real, default-on-if-safe changes.
