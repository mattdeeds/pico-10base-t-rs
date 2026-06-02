# R18 — DNS (LAN clients resolve names) + management UI

**Goal:** a LAN client on the WiFi AP resolves names *through the Pico* (so it can
browse by hostname, not just by IP — closing R17's known gap), and the LAN mgmt
page shows the connected clients + the WAN link + the NAT state.

Prereq context: R17 NAPT (`docs/r17-plan.md`, `src/conntrack.rs` + the WAN
`ForwardingDevice` in `src/forward.rs`) already NATs arbitrary LAN→WAN UDP and
routes replies back. R14.4's `src/dhcp_server.rs` already leases addresses but
hands out **no** DNS server. `router-plan.md` §6.4 + §7's R18 row are authoritative.

---

## 1. Goal & acceptance

**Acceptance (= `router-plan.md` §7's R18 row):** *clients resolve names via the
Pico; the mgmt page shows clients + WAN link.* Concretely, from a laptop joined to
`pico-rp2350-router` (normal DHCP, no static host config):

```bash
# DHCP now includes a DNS server (the R18 change):
#   dhclient lease shows  option domain-name-servers <upstream>;
# name resolution works through the Pico's NAT:
dig @<offered-dns> example.com        # returns an A record
# the mgmt page surfaces the live router state:
curl http://192.168.4.1/              # shows DNS-offered, Clients, WAN, NAT
```

Accept = the client gets an A record (resolved out the WAN via the Pico) **and**
the mgmt page lists the client's lease + the WAN link + NAT counters. On the
device, `[Fwd]` and `[Nat]` climb for the client's DNS/traffic with `drop=0`.

---

## 2. DNS approach — NAT-passthrough, not a relay (decided)

`router-plan.md` §6.4 offers two paths: (a) a true DNS *relay* on the LAN gateway
(`192.168.4.1:53` forwards queries out the WAN and relays answers), or (b) **"just
NAT port 53 through" — the simplest.** We took (b).

**Why (b):** R17 NAPT already forwards LAN→WAN UDP and routes replies back. If the
DHCP server simply hands clients a DNS server that is reachable *through* the NAT,
a client's `:53` query rides the exact path the R17 milestone already proved
(`ping 8.8.8.8`) — it NATs out, the reply NATs back. **Zero new DNS code**, no
cross-task relay, no transaction correlation/timeouts. The cost: clients see the
upstream resolver's IP (not the Pico) and the mgmt page shows DNS only as NAT
flows — acceptable for the milestone. A true relay (clients use `192.168.4.1` as
resolver, enables DNS stats) is a possible future nicety, not the bar.

**Which address to hand out:** the **WAN-learned upstream resolver** (`wan.dns0`
from the WAN DHCP lease) — respecting the upstream network — with a public
fallback (`8.8.8.8`) until the lease lands. Both are NAT-reachable.

```
LAN client ── DNS query to <offered-dns>:53 ──▶ [LAN dev] Transit ──▶ LAN_TO_WAN
   [WAN egress] NAPT src→WAN IP (R17, unchanged) ──▶ <offered-dns> resolves
   reply ──▶ [WAN recv] conntrack match (R17) ──▶ WAN_TO_LAN ──▶ client
```

The only new wiring: the DHCP server must *advertise* a DNS server, and learn
which one from the WAN side.

---

## 3. DHCP DNS option (`src/dhcp_server.rs`)

The OFFER/ACK must carry DHCP option 6 (Domain Name Server, RFC 2132 §3.8).

**The heapless snag (why R14.4 punted this):** `DhcpRepr.dns_servers` is
`Option<heapless::Vec<Ipv4Address, 3>>` — but that `Vec` is **smoltcp-0.13's
internal heapless 0.9**, a different crate version than our heapless 0.8, so we
can't name/construct it.

**The sidestep:** emit the option as a *raw* `DhcpOption { kind: 6, data }` via
`DhcpRepr.additional_options: &[DhcpOption]` (verified: smoltcp counts it in
`buffer_len` and emits it in `emit`, identical wire bytes to the typed field).
`dns_servers` stays `None`. The 4 octet bytes + the 1-element option array are
locals that outlive the `reply.emit` call.

**Where the address comes from:** a new `pub static LAN_DNS_OFFER: AtomicU32`
(the IPv4 octets packed big-endian, default `8.8.8.8`). The DHCP server reads it
at reply time; `wan_task` writes it (§4).

**Mgmt-page accessor:** add `DhcpServer::active_leases() -> impl Iterator<Item =
(Ipv4Address, [u8;6])>` so the status page can list connected clients without
exposing the lease array.

---

## 4. Publishing the upstream resolver (`wan_task`, `src/wireless.rs`)

`wan_task` already learns `wan.dns0` from the WAN dhcpv4 lease (used for the WAN's
own `dns::Socket`). Right after `dhcp_apply`, mirror it to the LAN side:

```rust
if let Some(dns) = wan.dns0 {
    crate::dhcp_server::LAN_DNS_OFFER
        .store(u32::from_be_bytes(dns.octets()), Ordering::Relaxed);
}
```

`router`-only by construction (`wan_task` is `#[cfg(feature = "router")]`). In the
`wireless`-only build there is no WAN, so the `8.8.8.8` default stands (harmless;
no internet there anyway).

---

## 5. Management UI (`wireless::serve_status_http`)

The R14.5 page showed AP SSID / LAN gateway / uptime / DHCP-reply count / LAN rx.
R18 adds, into the same one-shot HTTP/1.0 response:

- **DNS offered** — `LAN_DNS_OFFER` formatted as a dotted quad.
- **Clients** — every active DHCP lease as `IP  MAC` (via `active_leases()`); `(none)`
  when empty.
- **WAN** *(router build)* — reuse `WanState::write_status` on the `WAN_PUB`
  snapshot (ip / gw / dns / ping / resolved name), or `(no lease)`.
- **NAT** *(router build)* — `conntrack::live_count()/CT_CAP`, `NAT_OUT/IN/DROP`,
  and `FWD_SENT/DROP`.

The WAN/NAT block is `#[cfg(feature = "router")]` (those subsystems don't exist in
the wireless-only image). **Buffer sizing:** worst case = header + 32 lease lines
(~38 B each) + WAN/NAT ≈ 1.7 KB, so the body is a `String<1792>` and the socket TX
buffer goes 1024 → 2048 (head + body < 2 KB, queues in one `send_slice`); `write!`
truncation is a graceful backstop.

---

## 6. Test harness

The WAN upstream (`tools/wan-test-host.sh`) is unchanged — its dnsmasq already
forwards to `8.8.8.8`/`1.1.1.1`, and the Pico learns `dns=192.168.37.19` from it.

New: **`tools/r18-lan-validate.sh`** (run as root on the WAN test host) —
associates the Wi-Fi client to the AP, runs a no-reconfig DHCP exchange and greps
the lease for the DNS option, curls the mgmt page, and `dig`s a name through the
Pico's NAT. **Route-safe by design:** it only touches the `wlx…` adapter and a
single `/32` host route — never the default route / `enp1s0f0` / the SSH path.

**Validation-rig note:** flashing is **SWD/OpenOCD** (picotool BOOTSEL reboot is
flaky → detaches the CDC); read the device CDC over `/dev/ttyACM1` with a
DTR-asserting reader. The dev account has no root, so the root LAN-client steps run
from `r18-lan-validate.sh`, not inline.

---

## 7. Risks & open questions

1. **Hand-out address reachability.** Handing out the WAN-learned resolver is
   correct only if it's reachable through the NAT — it is (same path as the R17
   `ping 8.8.8.8`). If the upstream resolver is the test host's *own* WAN IP, a
   `dig @that-IP` *via the Pico* is a hairpin the host's `rp_filter` may drop —
   so the validate script `dig`s an off-host resolver (`1.1.1.1`) for a clean NAT
   proof; the handed-out-address path is exercised by real client name lookups.
2. **No DNS stats on the mgmt page.** NAT-passthrough means DNS shows up only as
   NAT/forward counts, not a query log. Accepted (a relay would add this — future).
3. **mgmt-page buffer pressure** at a full 32-client lease table — sized for it
   (§5); `write!` truncates rather than panicking if ever exceeded.
4. **Not addressed here (→ R19):** the cold-start gateway-ARP miss (`[Fwd] drop≈4`)
   and true per-bit collision-detect.

---

## 8. Step checklist

- [x] `src/dhcp_server.rs`: `LAN_DNS_OFFER` atomic (default `8.8.8.8`); emit DHCP
      option 6 via `additional_options` (heapless-version sidestep); `active_leases()`.
- [x] `wan_task`: publish `wan.dns0` → `LAN_DNS_OFFER` after `dhcp_apply`.
- [x] `serve_status_http`: DNS-offered + Clients + (router) WAN + NAT; TX buffer
      1024→2048, body `String<1792>`; `#[cfg(router)]` on the WAN/NAT block.
- [x] `tools/r18-lan-validate.sh`: route-safe LAN-client bring-up + the 3 checks.
- [x] Build all 4 configs (`default` / `wan-dhcp` / `wireless` / `router`) + clippy —
      clean (only the 2 pre-existing warnings). **⚠️ flash gotcha: force-recompile the
      router variant LAST + verify `strings <ELF> | grep -F "DNS offered:"` (and
      `"[Nat] ct="`) — a stale output binary flashes the wrong variant.**
- [x] On-device (`--features router`, SWD-flashed): a 2nd laptop on the AP —
      `curl http://192.168.4.1/` returned the enhanced page; `dig @1.1.1.1 example.com`
      resolved through the Pico's NAT; `[Fwd] l2w=21 w2l=21 sent=42 drop=0`,
      `[Nat] ct=5/64 out=21 in=21 drop=0` (out==in, no NAT drops); WAN's own ping/DNS
      intact; LAN/AP intact.
- [x] Commit to `main`; update `RESUME.md` (headline, table, Last-verified, fixed the
      stale `r13-wireless-scaffold` branch callout) + `project-vision-router` memory.
- [ ] (carry-overs → R19) proactive gateway-ARP on lease (`[Fwd] drop≈4`); true
      collision-detect. Optional future: a real DNS relay on `192.168.4.1:53` (clients
      use the Pico as resolver + DNS query stats).
