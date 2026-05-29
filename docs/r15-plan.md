# R15 — WAN as a DHCP client + executor⊥10BT runtime unification

**Status:** planning (2026-05-29). Branch `r13-wireless-scaffold`.
**Predecessor:** R14 "LAN up" COMPLETE (R14.1–R14.5, on-device validated) — see
`RESUME.md` + `router-plan.md` §12. **Successor:** R16 forwarding, R17 NAPT.

---

## 1. Goal & acceptance

Give the Pico an **upstream identity on the 10BASE-T (WAN) side**: a DHCP-leased
IP + default route + DNS, so the Pico *itself* reaches the internet out the wired
link.

**Acceptance (= `router-plan.md` §7's R15 row):**
1. The device DHCP-leases an address/route/DNS on the 10BASE-T side (telemetry
   shows the lease).
2. The device **pings `8.8.8.8`** (device-originated ICMP echo, replies seen).
3. The device **resolves a name** (e.g. `example.com` → A record) via the
   DHCP-provided DNS server.

NAPT / forwarding stay **R16/R17** — R15 is *the router box being a client*, not
*routing other clients' traffic*.

---

## 2. Reconciling the two docs first (do this, it removes a real ambiguity)

`RESUME.md` and `router-plan.md` §12 currently **disagree** on where the runtime
unification lands:

- `RESUME.md` ("Next session" + §7 row): R15 is *the first both-interfaces-live
  step* and therefore forces the executor-⊥-10BT unification (§11 deferral).
- `router-plan.md` §12 (line ~409): *"The hard executor-⊥-10BT runtime
  unification is **R16's** problem,"* deferred until transit traffic crosses the
  two interfaces.

**Resolution adopted by this plan:** split R15 into two sub-steps so the cheap,
isolated piece (DHCP client + WAN reachability) is proven *before* the
unification, and the unification is pulled forward into **R15b** (not deferred to
R16 as §12 says) — because making the Pico a real WAN client is far more
meaningful and testable with the LAN also live, and R16 (forwarding) genuinely
*requires* both interfaces up anyway. **Action:** once R15b lands, update §12's
line to point the unification at R15b and keep RESUME's framing. (Strictly,
R15a's acceptance — "Pico pings 8.8.8.8" — can be met on a 10BT-only build, so
§12 isn't *wrong*; it's just less useful than doing R15b now.)

---

## 3. The reframe: the unification is mostly wiring, not new risk

RESUME calls the unification "the hard part." That was true *before* R14.1.
After R14.1 it is mostly an **integration** task, because the two scary halves
are each already proven on this exact hardware:

| Capability | Where it's proven | Used in R15b as |
|---|---|---|
| embassy executor owns core 0 forever, drives continuous cyw43 Runner + USB + periodic `Timer` tasks on Hazard3 | **R14.1** (`wireless::run`, on-device) | unchanged |
| a 2nd smoltcp `Interface` runs as an executor task over a `phy::Device` | **R14.3** (`net_task` over `Cyw43Phy`) | the WAN task is the *same pattern* on `EthMac` |
| 10BT RX decode on **core 1** (`DMA_IRQ_0` → `RX_ENGINE`/`RX_SHARED`), `EthMac` as a core-0 `phy::Device` | **R12c–R12e** (`main_10bt`, merged to `main`) | unchanged; launched before `executor.run()` |
| DHCP wire codec, smoltcp on `EthMac`, NLP/IFG/CSMA TX discipline | R4–R12e | unchanged |

**What is genuinely new in R15b** (and therefore where to spend validation):
- **(N1)** core 1 (RX engine) launched in the *same binary* as the executor.
  Core-1 launch is independent of the executor (it happens on core 0 *before*
  `executor.run()` seizes core 0), and both are individually proven — but they've
  never run in one image.
- **(N2)** `main()` peripheral plumbing for **both** PIO0+DMA+PSM (10BT) **and**
  PIO1+USB (wireless) in one dispatch arm.
- **(N3)** lock coexistence: core 0 takes `Spinlock<0>` (RX inbox) *and*
  `critical_section`/`Spinlock<31>` (embassy time-driver queue); core 1 takes
  only `Spinlock<0>`. Different locks, but verify no path nests them in opposite
  orders (see Risks).
- **(N4)** RX delivery latency: core 1 publishes to the inbox but does **not**
  wake core 0's executor; the WAN task picks frames up on its next `Timer::after`
  poll (≈1 ms). Fine for R15 (the Pico is a light client); revisit for R16/R17
  throughput.

Everything else is the same code, re-wired. Treat R15b as careful wiring +
on-wire revalidation, not a rewrite.

---

## 4. Resource / peripheral map for the unified (router) build

| Resource | 10BT (today) | Wireless (today) | **Router (R15b)** |
|---|---|---|---|
| PIO0 | SM0 TX, SM1 RX, SM2 carrier-detect | — | **10BT (unchanged)** |
| PIO1 | — | SM0 gSPI | **cyw43 gSPI (unchanged)** |
| DMA ch0/ch1 | RX double-buffer | — | **10BT RX (unchanged)** |
| Core 1 | RX decode (`DMA_IRQ_0`) | not launched | **launched: RX decode** |
| Core 0 | blocking `main_10bt` loop | embassy executor | **embassy executor** |
| TIMER0 | `hal::Timer` reads (`get_counter`) | time-driver owns ALARM0 + `TIMER0_IRQ_0` | **time-driver owns it; smoltcp uses `embassy_time::Instant`** |
| USB | `main_10bt` polls inline | `usb_task` polls | **`usb_task` (unchanged)** |
| GP13/14 | RO / DI (10BT) | — | **10BT (unchanged)** |
| GP23/24/25/29 | — | WL_ON/DATA/CS/CLK | **cyw43 (unchanged)** |
| **GP25** | **10BT heartbeat LED** | cyw43 gSPI **CS** | **CS — the 10BT LED is dropped** (heartbeat = cyw43 GPIO0 blink, as R14.1) |

**Only two contended resources** and both are already resolved:
- **GP25**: LED in `main_10bt:229` vs CS in the wireless build → in the router
  build it is **CS**; drop the GP25 LED, keep the cyw43-GPIO0 heartbeat.
- **TIMER0**: the 10BT path only *reads* `get_counter()` for smoltcp timestamps;
  under the executor the WAN task uses `embassy_time::Instant::now()` instead
  (same as `net_task`), and the time-driver owns ALARM0 / `TIMER0_IRQ_0`. No
  contention. `eth_tx`'s CSMA backoff uses `hal::arch::delay` (CPU cycles), **not
  TIMER0** (verified), so the TX path is executor-safe.

PIO0/PIO1, the two DMA channels, and the two cores are fully disjoint between the
interfaces — no sharing.

---

## 5. R15a — DHCP client on the standalone 10BT build (de-risk in isolation)

**Build:** default 10BT (`main_10bt`), gated behind a new `wan-dhcp` cargo
feature so the production static-IP build (`192.168.37.24`, all the R4–R8 test
recipes) stays the default and reproducible.

**Why standalone first:** isolates the two R15a unknowns (does smoltcp's DHCP
client get us a real lease/route/DNS on *our* `EthMac` phy? can the device
originate ICMP + DNS and get replies through an upstream NAT?) from the
unification (N1–N4). No executor, no cyw43 — just new sockets in the existing
blocking loop.

### 5.1 Cargo features

```toml
# R15a — make the 10BT (WAN) side a DHCP client + originate ping/DNS.
wan-dhcp = [
    "smoltcp/socket-dhcpv4",   # DHCP *client* socket (proto-dhcpv4 already on for the server codec)
    "smoltcp/socket-icmp",     # device-originated ping (acceptance #2)
    "smoltcp/socket-dns",      # device-originated name resolution (acceptance #3)
]
```

### 5.2 Code (all in `main.rs::main_10bt`, gated `#[cfg(feature = "wan-dhcp")]`)

- **Drop the static `192.168.37.24/24`** when `wan-dhcp` is on; start the
  interface with no address (`iface.update_ip_addrs(|a| a.clear())` / just don't
  push). DHCP will install one.
- **Add a `dhcpv4::Socket`** to the `SocketSet` (bump `sockets_storage` from 5 →
  8 to fit dhcp + icmp + dns). Canonical smoltcp usage in the poll loop, *after*
  `iface.poll`:

  ```rust
  let event = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll();
  match event {
      Some(dhcpv4::Event::Configured(cfg)) => {
          iface.update_ip_addrs(|a| { a.clear(); let _ = a.push(IpCidr::Ipv4(cfg.address)); });
          if let Some(gw) = cfg.router {
              let _ = iface.routes_mut().add_default_ipv4_route(gw);   // verify exact 0.13 sig
          } else {
              iface.routes_mut().remove_default_ipv4_route();
          }
          // cfg.dns_servers: &[Ipv4Address] → feed the dns socket (5.3)
          dns_servers_into(&mut sockets, dns_handle, &cfg.dns_servers);
          // stash lease for telemetry
      }
      Some(dhcpv4::Event::Deconfigured) => {
          iface.update_ip_addrs(|a| a.clear());
          iface.routes_mut().remove_default_ipv4_route();
      }
      None => {}
  }
  ```

  ⚠️ **Verify against smoltcp 0.13 docs:** the exact `routes_mut()` /
  `add_default_ipv4_route` signature and whether `dhcpv4::Config.dns_servers` is
  `&[Ipv4Address]` or `Vec`. (The server side avoided the `dns_servers` Vec for a
  heapless-version reason — see `dhcp_server.rs:118` — the *client* `Config` is a
  different type; confirm.)

- **Acceptance #2 — ping `8.8.8.8`:** an `icmp::Socket` bound to
  `Endpoint::Ident(id)`; every ~1 s build an `Icmpv4Repr::EchoRequest`, `emit`
  into `socket.send(...)` to `8.8.8.8`, and count replies via `socket.recv`.
  Surface `wan_ping_ok/wan_ping_sent` over CDC.
- **Acceptance #3 — resolve a name:** a `dns::Socket::new(&dns_servers, &mut
  queries)`; `start_query(iface.context(), "example.com", Type::A)`, poll
  `get_query_result` until `Ok(addrs)`. Surface the first resolved A record.

### 5.3 Telemetry

Add a `[Wan]` line to `log_status` (1 Hz): leased IP, gateway, dns[0], ping
sent/ok, last resolved name+IP. This is the on-device acceptance evidence.

### 5.4 Acceptance (R15a)

With the host configured as the upstream NAT gateway (§7): device shows a lease,
`[Wan] ping ok` climbing, and a resolved A record. The existing 10BT production
smoke test (no `wan-dhcp`) still passes byte-unchanged.

---

## 6. R15b — runtime unification (both interfaces live)

**Build:** a new `router` cargo feature = `wireless` deps + the WAN sockets:

```toml
router = [
    "wireless",            # cyw43 + executor + time-driver + cyw43_phy + dhcp_server (LAN)
    "smoltcp/socket-dhcpv4",
    "smoltcp/socket-icmp",
    "smoltcp/socket-dns",
]
```

Keep three reproducible build configs:
- `(default)` → `main_10bt`, static WAN IP (production NIC, R4–R12e).
- `--features wireless` (and **not** `router`) → standalone LAN (R14, reproducible).
- `--features router` → **dual interface (R15b)**.

### 6.1 `main()` dispatch

Add a `#[cfg(feature = "router")]` arm (taking precedence over the existing
`wireless`-only and `not(wireless)` arms — gate them
`all(feature="wireless", not(feature="router"))` and `not(feature="wireless")`
respectively). The router arm does **the union of both setups** on core 0:

1. Shared clock/pin setup (already in `main()` before the dispatch).
2. **10BT bring-up** (lifted from `main_10bt:231`–`300`): GP14/GP13 → PIO0;
   `EthTx::new` (PIO0 SM0+SM2), DMA split, `EthRx::new`, `install_rx(eth_rx,
   our_mac)`, `launch_core1_riscv(...)`, `let mac = EthMac::new(eth_tx)`.
   **Omit the GP25 LED** (GP25 is CS).
3. **Wireless transport bring-up** (lifted from the `wireless` arm,
   `main.rs:193`–`198`): GP24/GP29 → PIO1, `PioSpiCyw43::new(...)`, WL_ON pin.
4. Hand **both** to a new `wireless::run_router(mac, pwr, spi, usb, usb_dpram,
   usb_clock, &mut resets)`.

### 6.2 `wireless::run_router` + the WAN task

`run_router` mirrors `run` (build USB, unmask `ALARM_IRQ`, create the static
`Executor`, `executor.run(...)`) but spawns **one extra task**: `wan_task(mac)`.

```rust
#[embassy_executor::task]
async fn wan_task(mut mac: EthMac) -> ! {
    // Same shape as net_task, but Device = EthMac and the sockets are the
    // R15a WAN set (dhcpv4 client + icmp + dns). MAC = the 10BT 12:34:..:BC.
    // smoltcp Instant from embassy_time (TIMER0 owned by the time-driver).
    let now = || Instant::from_micros(embassy_time::Instant::now().as_micros() as i64);
    let mut iface = /* Interface::new on &mut mac, no static IP — DHCP fills it */;
    // sockets: dhcpv4 + icmp + dns (+ later: whatever R16 needs)
    let mut next_nlp = embassy_time::Instant::now();
    loop {
        iface.poll(now(), &mut mac, &mut sockets);
        // R15a DHCP-client config-apply + ping + dns logic, verbatim
        // NLP keepalive every 16 ms — REQUIRED for link integrity (gotcha #9).
        if embassy_time::Instant::now() >= next_nlp {
            next_nlp += embassy_time::Duration::from_millis(16);
            mac.send_nlp();
        }
        Timer::after(Duration::from_millis(1)).await;   // keeps the executor live + bounds RX latency
    }
}
```

Key points:
- `EthMac` is `'static` (owns its `EthTx` + buffers; nothing borrowed) → moves
  into the task cleanly, exactly like `net_task` takes `NetDriver<'static>`.
- The `1 ms` poll cadence (a) guarantees the executor always has a pending timer
  so it never sleeps indefinitely and misses RX, and (b) bounds the core-1→core-0
  RX hand-off latency (N4). NLP every 16 ms preserves link integrity (gotcha #9);
  IFG/CSMA discipline is already inside `EthTx`.
- The LAN side (`cyw43_bootstrap_task` → Runner + `net_task`) is spawned exactly
  as in R14 — **no change**. The heartbeat LED stays the cyw43-GPIO0 blink.

### 6.3 Acceptance (R15b)

All three must hold **concurrently**:
1. **WAN:** device leases on 10BT + pings `8.8.8.8` + resolves a name (R15a
   criteria, now under the executor).
2. **LAN:** a Wi-Fi client still joins the AP, gets a `192.168.4.x` lease, pings
   `192.168.4.1`, loads the `:80` status page (re-run R14's accept).
3. **10BT RX engine:** core 1 up (`launch=ok`, ticks climbing), RX decode works
   (the WAN DHCP/ICMP/DNS replies *are* RX frames decoded on core 1 → proves the
   `RX_ENGINE`/`RX_SHARED`/inbox path runs under the executor).

---

## 7. Test harness — the host must become the upstream NAT gateway ⚠️

This is **new setup and non-trivial** — budget time. The current 10BT host setup
is a static point-to-point link with **no DHCP server and no upstream route**, so
nothing will answer the device's DISCOVER or forward its ping today.

Make the Linux host the device's upstream router on `enp1s0f0`:

```bash
# 1. Keep the 10HD link forcing (as today)
ip link set enp1s0f0 up
ethtool -s enp1s0f0 speed 10 duplex half autoneg off
ip addr add 192.168.37.19/24 dev enp1s0f0

# 2. DHCP + DNS-forwarder on the wired link (dnsmasq)
#    Hands out 192.168.37.100-150, gateway+DNS = the host (.19).
dnsmasq --interface=enp1s0f0 --bind-interfaces \
        --dhcp-range=192.168.37.100,192.168.37.150,1h \
        --dhcp-option=3,192.168.37.19 \
        --dhcp-option=6,192.168.37.19 \
        --no-daemon            # foreground, watch the DISCOVER/OFFER/ACK

# 3. NAT: forward enp1s0f0 → the host's real uplink (e.g. wlan0/eth0)
UP=<host-uplink-iface>
sysctl -w net.ipv4.ip_forward=1
iptables -t nat -A POSTROUTING -o "$UP" -s 192.168.37.0/24 -j MASQUERADE
iptables -A FORWARD -i enp1s0f0 -o "$UP" -j ACCEPT
iptables -A FORWARD -i "$UP" -o enp1s0f0 -m state --state RELATED,ESTABLISHED -j ACCEPT
```

Then: device gets a lease from dnsmasq → default route `.19` → ping `8.8.8.8`
NATs out the host's uplink → replies return; DNS queries to `.19` are forwarded.

**Gotchas to expect:**
- Like the LAN DHCP server (§12.3 risk 6, inverted), this **adds a route on the
  host** but should not hijack anything — the host keeps its own default route;
  we only add a MASQUERADE for `192.168.37.0/24`. Tear down the iptables rules
  after testing.
- 10BASE-T half-duplex into a real switch/router usually won't negotiate, which
  is exactly why the host-as-gateway approach (forcing 10HD) is the path — don't
  expect to plug into a home router.
- dnsmasq may refuse to bind if another resolver (systemd-resolved) holds :53 —
  use `--bind-interfaces` (above) and/or a distinct `--listen-address`.

---

## 8. Risks & open questions

1. **(N3) Lock coexistence.** Core 0 now takes `Spinlock<0>` (RX inbox pop in
   `EthMac::receive`) *and* `critical_section`/`Spinlock<31>` (embassy time-driver
   queue, via `Timer`). They're distinct locks and `EthMac::receive` doesn't run
   inside a `critical_section`, so no nested ordering — **but verify** `iface.poll`
   on the WAN path never holds `Spinlock<0>` across an embassy wake that takes
   `critical_section` (it shouldn't: smoltcp is sync and doesn't touch embassy).
   Low risk, explicit check.
2. **(N1) core 1 + executor in one image.** Launch core 1 *before*
   `executor.run()`. If core-1 launch ever hangs (`launch_core1_riscv` is bounded
   → returns err, doesn't hang core 0), the executor still comes up; surface
   `launch=ok` in the `[Cyw43]`/`[Wan]` telemetry so a core-1 failure is visible.
3. **smoltcp API drift (0.13).** Confirm: `dhcpv4::Event`/`Config` field names,
   `routes_mut().add_default_ipv4_route`, `icmp`/`dns` socket constructors and the
   `socket-icmp`/`socket-dns`/`socket-dhcpv4` feature names. Pin down before
   coding R15a (cheap to get wrong, cheap to check).
4. **Memory.** Two smoltcp `Interface`s + socket buffers (LAN: dhcp-server+http;
   WAN: dhcp-client+icmp+dns) + cyw43 `State` + core-1 16 KB stack + 2×16 KB DMA +
   carry/stitch. 520 KB SRAM — budget it; the cyw43 firmware/CLM/nvram are in
   flash (`include_bytes!`/`aligned_bytes!`), not RAM.
5. **RX latency (N4).** 1 ms poll hand-off is fine for a light client; if R16/R17
   forwarding needs lower latency, have core 1 signal core 0 (inter-core FIFO or
   an `embassy_sync` signal/waker) instead of relying on the timer tick. Defer.
6. **DNS server plumbing.** The DHCP-provided DNS servers must reach the `dns`
   socket; re-feed `socket.update_servers(...)` on every `Configured` event (and
   clear on `Deconfigured`).

---

## 9. Step checklist

**R15a (standalone 10BT, `--features wan-dhcp`) — ✅ COMPLETE & on-device-validated 2026-05-29:**
- [x] Confirm smoltcp 0.13 dhcpv4/icmp/dns/routes APIs + feature names. (Verified against the cached 0.13.1 source: `dhcpv4::Socket::new()`/`poll`→`Event::Configured(Config{address: Ipv4Cidr, router, dns_servers})`; `routes_mut().add_default_ipv4_route(gw)`; `icmp` `Endpoint::Ident` + `Icmpv4Repr::EchoRequest/Reply{ident,seq_no,data}`; `dns::Socket::new(&[], &mut [Option<DnsQuery>])` + `update_servers`/`start_query(iface.context(), name, DnsQueryType::A)`/`get_query_result`. `Ipv4Address = core::net::Ipv4Addr` (Display).)
- [x] Add `wan-dhcp` feature; bump `sockets_storage` to 8.
- [x] Replace static WAN IP with a `dhcpv4` client + config-apply (IP/route/DNS). (`wan_dhcp_apply` — copies the lease out of the borrowed `Event` before touching the dns socket.)
- [x] Add `icmp` socket: device pings `8.8.8.8` (ident `0x42`); count replies. (`wan_ping_send`/`wan_ping_drain`.)
- [x] Add `dns` socket: resolve `example.com`; surface the A record. (`wan_dns_start`/`wan_dns_harvest`.)
- [x] `[Wan]` telemetry line. (`log_wan` — `ip=… gw=… dns=… ping=ok/sent <name>=<A>`.)
- [x] Host harness (`tools/wan-test-host.sh`, nftables not iptables); validated acceptance #1–#3; default build byte-unchanged (all changes `#[cfg]`-gated). **Evidence:** `DHCPACK 192.168.37.129`; `tcpdump -ni eno1` device→`8.8.8.8` echo (id 66, len 16) + replies ~13 ms; `example.com` forwarded/resolved.

**R15b (`--features router`) — ✅ COMPLETE & on-device-validated 2026-05-29:**
- [x] Add `router` feature; restructure the three `main()` dispatch arms
      (`#[cfg(router)]` first, wireless-only re-gated `all(wireless, not(router))`).
- [x] Extracted shared `src/wan.rs` (WAN logic) + `setup_eth_mac` (10BT bring-up)
      so `main_10bt` and the router arm share one copy. `OUR_MAC` is one const.
- [x] Router arm: 10BT bring-up (no GP25 LED — it's the gSPI CS) + core-1 launch
      + PIO1 gSPI + WL_ON.
- [x] `wireless::run_router` + `wan_task(EthMac)` (R15a logic, async, NLP @16 ms);
      WAN status published to a `critical_section` cell, printed as `[Wan]` by
      `usb_task`. `WAN_CORE1_OK` surfaces the core-1 launch result.
- [x] All four build configs (default / wan-dhcp / wireless / router) compile
      clean — zero new warnings (only the two pre-existing: `eth_rx.rs` args,
      `build_gspi_sm` type).
- [x] Validated all three concurrent acceptances (§6.3): **WAN out** (`[Wan]
      core1=ok ip=192.168.37.129/24 ping=38/38 example.com=…`), **LAN intact**
      (Wi-Fi client joined → `ping 192.168.4.1` 3/3 → `curl :80` page,
      `DHCP replies:3`/`LAN rx:11`), **core 1 up** (`core1=ok`; WAN ping replies
      are 10BT RX frames decoded there) — all at once, with `[Cyw43] ap=1 net=1`.
- [x] N1–N4 (§3) confirmed: core 1 + executor in one image (N1) works; `main()`
      plumbing (N2) clean; `Spinlock<0>`/`critical_section` coexistence (N3) holds
      under concurrent load; 1 ms poll hand-off (N4) fine for a light client.
- [ ] (Carryover) Update `router-plan.md` §12/§7 to point the unification at
      R15b (§2) — RESUME + r15-plan are updated; router-plan §12 wording still
      says R16. Low priority; fold into the R16 plan.
