# RP2350 wireless router — design plan

The end goal the whole project serves (see the `project-vision-router` memory):
a **low-power wireless router on the RP2350**. Wireless clients associate on the
LAN side; their traffic is **NAT-routed out the 10BASE-T WAN** to a wired
network / the internet.

With R0–R12e the **WAN half is essentially done** — a robust 10BASE-T NIC
(PIO Manchester TX/RX, edge-track DPLL, multicore RX, carrier-sense + CSMA/CA;
~742 kB/s idle, no starvation, 10–38× better under load than the old
single-core). This doc scopes the **router half**: the wireless LAN, the
forwarding/NAT data path, and how they fit our Hazard3 RISC-V / `rp235x-hal` /
no-embassy stack.

Status: **scoping (2026-05-28).** Architecture fork decided = **Option A**
(keep RISC-V, port the cyw43 transport — see §4). No router code written yet.

---

## 1. Target topology

```
   wireless clients (phones/laptops)
            │  (WPA2, 2.4 GHz)
            ▼
   ┌─────────────────────────────────────────────┐
   │  Pico 2 W (RP2350)                            │
   │                                               │
   │   LAN: CYW43439 in AP mode                    │
   │     • SSID + WPA2, DHCP server, gateway IP    │
   │            │                                  │
   │     ┌──────┴───────┐  forwarding + NAPT       │
   │     │  router core │  (conntrack TCP/UDP/ICMP)│
   │     └──────┬───────┘                          │
   │            │                                  │
   │   WAN: 10BASE-T NIC (R0–R12e)                 │
   │     • DHCP client, default route, DNS         │
   └────────────┼──────────────────────────────────┘
                ▼
        wired LAN / internet
```

The 10BASE-T WAN (~6 Mbit/s effective) is the throughput bottleneck — fine for
a "low-power router." The CYW43 2.4 GHz LAN has headroom to spare.

## 2. What we have vs. what a router needs

| Piece | Have? | Notes |
|---|---|---|
| WAN PHY (10BASE-T) | ✅ R0–R12e | PIO TX/RX, DPLL, multicore RX, CSMA/CA |
| Endpoint stack (the device's own IP) | ✅ smoltcp | ARP/ICMP/UDP/TCP terminate today |
| LAN PHY (wireless AP) | ❌ | CYW43439 in AP mode (§5) |
| DHCP **client** (WAN) | ❌ | smoltcp has a `dhcpv4` socket — small |
| DHCP **server** (LAN) | ❌ | smoltcp has no server — new code (§6.3) |
| L3 **forwarding** between interfaces | ❌ | new — smoltcp won't (§3, §6.1) |
| **NAPT** + connection tracking | ❌ | new — the bulk of the work (§6.2) |
| DNS relay (LAN → WAN) | ❌ | new-ish (§6.4) |
| Management UI | ◑ | reuse the HTTP server on the LAN gateway IP |

## 3. smoltcp's role — keep it, but it is NOT the router

smoltcp is an **endpoint** TCP/IP stack: an `Interface` owns sockets and
terminates traffic addressed to its own IP(s). It has **no packet forwarding
between interfaces and no NAT/NAPT** — it drops anything not addressed to it.

So the router splits cleanly into two planes:

- **Control plane → smoltcp (keep it).** The device's *own* traffic on *both*
  interfaces: DHCP client (WAN), DNS resolver, the management HTTP/UI, ICMP.
  And `cyw43` exposes a **`NetDriver` that plugs straight into a smoltcp
  `Interface`** — the standard, supported LAN glue. Ripping smoltcp out would
  be a mistake; it's doing real work.
- **Data plane → NEW custom code.** Transit packets (a LAN client → the
  internet, and the return path) bypass smoltcp entirely: parse the IP packet,
  NAPT-rewrite it, re-emit on the other interface.

**Architecture = hybrid with a per-frame classifier.** A frame arriving on
either interface is classified:
- **for-us** (dst MAC = our iface MAC, dst IP = our iface IP, broadcast/ARP,
  or a NAPT-return flow) → hand to that interface's smoltcp `Interface`, OR to
  the NAPT reverse path;
- **transit** (routable, not for us) → the forwarding + NAPT fast-path.

This matches the `project-vision-router` instinct ("per-interface raw frames +
our own forwarding/NAPT layer") — but only for the *fast path*; smoltcp stays
the control-plane brain. Two smoltcp `Interface`s (one per phy) coexist with the
forwarding layer; they don't talk to each other through smoltcp.

## 4. The architecture fork — DECIDED: Option A (keep RISC-V)

**The finding that drove this:** the CYW43 driver ecosystem (`cyw43`,
`cyw43-pio`, `embassy-net`) is embassy-centric, and **embassy supports RP2350
only on the ARM Cortex-M33 cores** — to target the **Hazard3 RISC-V** cores you
drop embassy and use `rp235x-hal`, which is precisely our entire codebase. So
the stock wireless stack and our stack sit on opposite sides of an ARM↔RISC-V
line.

Options considered:

| Option | Keeps R0–R12e? | Wireless effort | Decision |
|---|---|---|---|
| **A. Keep RISC-V, port the cyw43 transport** | ✅ all | custom `SpiBusCyw43` + async glue | **CHOSEN** |
| B. Switch the project to ARM + embassy | ❌ rewrite NIC | cyw43 "just works" | rejected — throws away the Hazard3 multicore/PIO/DPLL work |
| C. External wireless (ESP32-AT over SPI) | ✅ | different driver | fallback only — not the onboard radio the user wants |

**Why Option A is feasible** (verified during scoping):
1. `cyw43`'s core driver is **transport-agnostic** via the `SpiBusCyw43` trait;
   `cyw43-pio`'s `PioSpi` is just the embassy-rp reference impl. We write our
   own `SpiBusCyw43` on `rp235x-hal`'s **free PIO1**.
2. `embassy-executor` has a real **`arch-riscv32`** backend (used by ESP32-C3/
   C6, which are riscv32imc/imac) — *separate* from the ARM-only `embassy-rp`
   HAL. So async tasks can run on Hazard3 (`riscv32imac`).
3. A small **`embassy-time-driver`** shim backed by our existing `rp235x-hal`
   `Timer` satisfies cyw43's `Timer::after` delays.

So we pull in `cyw43` (core) + arch-agnostic embassy support crates
(`embassy-executor` arch-riscv32, `embassy-time`, `embassy-sync`,
`embassy-futures`) and skip `embassy-rp` and `cyw43-pio` entirely. The
**biggest new infrastructure + risk is adopting an async runtime** on our
currently-blocking main loop — de-risked first (§7, R13).

## 5. Wireless integration design (Option A)

```
   cyw43 core (transport-agnostic)
     │  SpiBusCyw43 trait
     ▼
   OUR PioSpiCyw43  ── rp235x-hal PIO1 (half-duplex SPI) + DMA + CS/PWR pins
     │
     ├─ Runner::run()  ── async task, driven by embassy-executor (arch-riscv32)
     │                     uses embassy-time (our Timer shim) for delays
     │
     └─ Control         ── async: init(), start_ap_wpa2(SSID, pass), gpio_set(LED)…
     └─ NetDriver       ── impls a Device → smoltcp Interface (LAN control plane)
```

Open items to nail in the R13 spike:
- **Half-duplex SPI PIO program.** The CYW43 uses a nonstandard half-duplex
  SPI (shared data line). Reference: the pico-sdk `cyw43_bus_pio_spi.c` PIO
  program and embassy's `cyw43-pio` — port the program to `rp235x-hal`'s PIO
  API on PIO1.
- **Firmware blobs.** CYW43439 firmware (`43439A0.bin`) + CLM blob, loaded at
  runtime; embed in flash (`include_bytes!`) and feed to `cyw43::new`.
- **Async runtime shape.** embassy-executor on core 0 (restructure `main`'s
  poll loop into async tasks: cyw43 Runner, smoltcp poll×2, router). The
  10BASE-T PIO/DMA/core-1 RX path is IRQ-driven and stays as-is; an async task
  drains its inbox via smoltcp. Decide whether the cyw43 Runner shares core 0
  or moves to core 1 (see §6 core budget).
- **embassy-time-driver** on `rp235x-hal` `Timer` (TIMER0): implement the
  `embassy_time_driver::Driver` trait (now(), schedule_wake()).

## 6. Router data path (the new feature work)

### 6.1 Forwarding
Classify each received frame (§3). Transit IP packets: decrement TTL, look up
the egress interface (LAN-side = the AP subnet; everything else = default route
out WAN), re-emit. Start with plain forwarding (no NAT) tested via static host
routes, then layer NAPT on top.

### 6.2 NAPT + connection tracking
- Outbound (LAN→WAN): rewrite src IP→WAN IP, src port→an allocated port;
  insert/refresh a conntrack entry keyed by (proto, orig-src, orig-sport,
  dst, dport).
- Inbound (WAN→LAN): match the conntrack entry, rewrite dst back to the LAN
  client, re-emit on LAN.
- Per-proto: TCP (track via ports + a coarse state/timeout), UDP (ports +
  idle timeout), ICMP echo (track via id). Fix up IP + L4 checksums on rewrite
  (incremental checksum update).
- Fixed-size conntrack table (heapless) with LRU/timeout eviction — bounded RAM,
  no alloc. This is the bulk of the new code.

### 6.3 DHCP server (LAN)
smoltcp has only a DHCP *client*. The LAN DHCP server is new: a small UDP
responder (DISCOVER→OFFER, REQUEST→ACK) handing out leases from the AP subnet
pool, with gateway = our LAN IP and DNS = our LAN IP (we relay). Fixed lease
table.

### 6.4 DNS relay + management
- DNS: LAN clients use us as resolver; forward their queries out the WAN to the
  upstream DNS (learned via WAN DHCP), relay answers back. (Or run NAPT over the
  DNS UDP flow like any other — simplest is to just NAT port 53 through.)
- Management: reuse the existing HTTP server, bound to the LAN gateway IP —
  status (clients, conntrack, WAN link), and later config.

### Core / PIO / DMA / pin budget
- **PIO:** PIO0 = SM0 10BT-TX, SM1 10BT-RX, SM2 carrier-detect (SM3 free).
  **PIO1 → cyw43 half-duplex SPI** (1 SM). Fits.
- **DMA:** ch0/ch1 = 10BT RX double-buffer; cyw43 SPI wants 1–2 channels —
  available.
- **Cores:** core 0 = main/async-exec/WAN-TX/smoltcp×2/router/USB; core 1 =
  10BT RX decode. The cyw43 Runner + forwarding add load to core 0 — **open
  question:** may need to rebalance (e.g. forwarding fast-path or the cyw43
  Runner on core 1). Measure before deciding.
- **Pins (Pico 2 W):** the CYW43 consumes **GP23 (WL_ON), GP24 (WL_DATA),
  GP25 (WL_CS), GP29 (WL_CLK)**, and the user **LED moves onto the CYW43's own
  GPIO0**. Our 10BASE-T (GP13 RO, GP14 DI) is clear of all of these →
  coexists. **But GP25 (today's `led` pin) becomes WL_CS** — the LED code must
  move to the CYW43 GPIO (via the driver) or another free pin. *(Verify exact
  Pico 2 W pinout in R13.)*
- **Power:** "low-power" fights the always-on 60 MHz RX sampler + continuous
  DMA + cyw43. Deferred to last (roadmap E) — a known unresolved tension.

## 7. Phased roadmap

Riskiest piece (wireless on Hazard3) first, so Option A is proven before we
build the router on top of it.

| Phase | Goal | Acceptance |
|---|---|---|
| **R13 — wireless de-risk spike** | Custom `SpiBusCyw43` on PIO1 + embassy-executor(riscv32) + time-driver shim + CYW43 firmware load. | Chip inits; beacons (AP) or associates (STA); a client sees the SSID / the chip's own LED toggles via the driver. *Proves Option A.* |
| **R14 — LAN up** | CYW43 **AP mode** (WPA2) + smoltcp on its `NetDriver` + **DHCP server**. | A phone joins the SSID, gets a DHCP lease, pings the Pico's LAN gateway IP. |
| **R15 — WAN as client** | **DHCP client** on the 10BASE-T (smoltcp `dhcpv4`) → upstream IP, default route, DNS. | The Pico itself reaches the internet (ping 8.8.8.8, resolve a name). |
| **R16 — forwarding** | L3 forward LAN↔WAN for transit packets (static routes, no NAT). | A LAN client with a manual route reaches a WAN host. |
| **R17 — NAPT** | NAT/NAPT + conntrack (TCP/UDP/ICMP) + the per-frame classifier. | **A phone on WiFi browses the internet through the Pico.** The milestone. |
| **R18 — DNS relay + mgmt UI** | LAN DNS relay; HTTP status/config on the LAN gateway IP. | Clients resolve names via the Pico; mgmt page shows clients + WAN link. |
| **R19+ — robustness, backpressure, then low-power** | Flow control, conntrack pressure handling; then the deferred power work. | — |

## 8. Risks + open questions

1. **Async runtime adoption (R13, the #1 risk).** embassy-executor on
   `riscv32imac` is real but unproven *in this project*; restructuring the
   blocking main loop into async tasks touches everything. De-risk in R13 with
   the minimum (just blink the CYW43 LED via async) before committing.
2. **Half-duplex SPI PIO port.** The CYW43 SPI program must be ported to
   `rp235x-hal` PIO1. The pico-sdk C program + cyw43-pio are the references.
3. **Core balance.** cyw43 Runner + NAPT forwarding + two smoltcp polls on
   core 0 may not fit; rebalancing onto core 1 (which currently only decodes
   10BT RX) is the likely lever — measure, don't assume (the project's MO).
4. **Memory.** Two smoltcp `Interface`s + socket buffers + conntrack + DHCP/
   ARP tables + cyw43 firmware + core-1 stack. RP2350 has 520 KB SRAM — should
   fit, but budget it.
5. **Low-power vs always-on sampler** — unresolved; explicitly last.
6. **cyw43 on Hazard3 has little/no precedent** — we may hit
   embassy-executor/embassy-time integration rough edges that the ARM path
   doesn't. Keep Option C (external module) as the fallback if R13 stalls.

## 9. References

- cyw43 driver + AP mode + RP2350/Pico 2 W support: <https://docs.embassy.dev/cyw43>,
  <https://github.com/embassy-rs/embassy/tree/main/cyw43>
- embassy RISC-V on RP2350 = drop embassy, use rp235x-hal (ARM-only embassy):
  <https://riscv.org/blog/raspberry-pi-launch-new-rp2350-microcontroller-and-pico-2-development-board-with-risc-v-support/>
- embassy-executor `arch-riscv32`:
  <https://github.com/embassy-rs/embassy/blob/main/embassy-executor/src/arch/riscv32.rs>
- `SpiBusCyw43` / custom transport + the half-duplex SPI PIO:
  <https://github.com/embassy-rs/embassy/blob/main/cyw43-pio/src/lib.rs>,
  <https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c>

## 10. R13 progress — board-independent scaffolding DONE (2026-05-28)

Built on branch `r13-wireless-scaffold` (not merged). Gated behind a new
`wireless` cargo feature, OFF by default — the production 10BASE-T build is
byte-unchanged. **`cargo build --release --features wireless` compiles AND
links for `riscv32imac`.**

**De-risking results (the reason for the spike):**
- **cyw43 0.7.0 + its ARM-named deps (`cortex-m` 0.7.7, `cortex-m-rt` 0.7.5)
  compile for riscv32imac.** The fear that `cortex-m-rt` would `compile_error!`
  on non-ARM was wrong — it target-gates to ~nothing. So Option A is viable.
- **`embassy-executor` 0.10 `platform-riscv32` backend links on Hazard3** —
  the #1 risk (async runtime on RISC-V, separate from the ARM-only
  `embassy-rp`) is cleared at the link level.
- Compatible version set pinned by cyw43 0.7.0: embassy-time 0.5.1 (with
  `tick-hz-1_000_000` + `generic-queue-16`), embassy-time-driver 0.2.2,
  embassy-time-queue-utils 0.3.2, embassy-sync 0.8.0, embassy-futures 0.1.2.
  The executor feature is `platform-riscv32` (renamed from `arch-riscv32`).
- embassy-executor 0.10's `#[task]` macro returns a `Result<SpawnToken,_>`
  (arena slot) — spawn via `if let Ok(t) = task() { spawner.spawn(t) }`.

**What's in `src/wireless.rs` (compiles + links, not yet run on hardware):**
1. **embassy-time driver on TIMER0** — `now()` reads the µs counter (1 MHz tick
   = no scaling); `schedule_wake()` uses ALARM0 + the `TIMER0_IRQ_0` handler +
   the generic 16-slot timer queue. The RP2350 TIMER0 PAC accessors
   (`timerawh/l`, `alarm0`, `inte.alarm_0`, `intr` write-1-to-clear) all
   resolved correctly.
2. **Async executor** — `run_executor()` enables the alarm IRQ, then runs the
   `embassy-executor` thread executor with a heartbeat task that `await`s an
   `embassy_time::Timer` (which link-exercises the whole time stack).
3. **`SpiBusCyw43` transport skeleton** — `PioSpiCyw43` + the trait impl
   (`cmd_write`/`cmd_read`) type-check against `cyw43::new`'s bounds. **Bodies
   are stubs** — the real half-duplex gSPI PIO1 program is the on-board step.

**Remaining for R13 (needs the Pico 2 W in hand):**
- Port the gSPI PIO program to PIO1 + fill in `cmd_write`/`cmd_read` (FIFO/DMA).
  Refs: pico-sdk `cyw43_bus_pio_spi.c`, embassy `cyw43-pio`.
- Embed the CYW43 firmware + CLM blobs (`cyw43-firmware`).
- Build `PioSpiCyw43` + the PWR (WL_ON) pin, call `cyw43::new(...)`, spawn
  `Runner::run()`, `Control::init()` → toggle the onboard LED / beacon.
- Verify on-wire: chip inits; a client sees the SSID. (R14 then adds AP +
  DHCP server + the smoltcp LAN `Interface`.)
- Relocate the user LED off GP25 (= wireless CS on Pico 2 W).
