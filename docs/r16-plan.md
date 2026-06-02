# R16 — L3 forwarding (LAN ↔ WAN transit, no NAT)

**Status:** planning (2026-05-29). Branch `r13-wireless-scaffold`.
**Predecessor:** R15 COMPLETE (`75a408a` + `ed26eb7`) — both interfaces live under
one executor (cyw43 LAN on core 0 + 10BT WAN, RX decode on core 1). See
`docs/r15-plan.md` + `RESUME.md`. **Successor:** R17 NAPT/conntrack (the
milestone: a phone browses the internet through the Pico).

---

## 1. Goal & acceptance

Move **other clients'** packets across the two interfaces. R15 made the Pico a
*client* on both sides; R16 makes it a *router* for transit traffic — pure L3
forwarding, **no NAT** (addresses unchanged), static routing between the two
subnets.

- **LAN** `192.168.4.0/24` (cyw43 AP, gateway `192.168.4.1` = us)
- **WAN** `192.168.37.0/24` (10BASE-T, our DHCP-leased IP e.g. `192.168.37.129`,
  upstream gateway `192.168.37.19`)

**Acceptance (= `router-plan.md` §7's R16 row):** *a LAN client reaches a WAN
host* with the routes set up but **no NAT** — concretely, a Wi-Fi client
(`192.168.4.x`, default route = the Pico) pings/curls the WAN host
`192.168.37.19` directly, and the reply routes back (the WAN host has a manual
route `192.168.4.0/24 → <pico-wan-ip>`). The LAN client's source IP is
*preserved* on the wire (verify with `tcpdump` on the host: `192.168.4.x >
192.168.37.19`, not a NAT'd address).

NAPT (source rewriting + conntrack, so the LAN client can reach the *internet*
without the upstream needing a route back) stays **R17**.

---

## 2. The core problem: smoltcp does not forward

smoltcp is an *endpoint* stack. A LAN client sending to a WAN host sets
**dst-MAC = our gateway MAC** (so the frame *does* reach our `Interface`) but
**dst-IP = the remote** (not any of our addresses). smoltcp receives the frame,
finds no local address/socket for that dst-IP, and **silently drops it**. There
is no forward hook, and the neighbor cache (IP→MAC) is private. So forwarding is
**new custom data-path code beside the two `Interface`s** (router-plan §3/§6.1) —
this plan is how.

---

## 3. Architecture — a classifying `phy::Device` wrapper + two cross-task queues

The clean way to keep smoltcp handling *local* traffic (DHCP, ARP, ICMP-to-us,
mgmt HTTP, the WAN client's own ping/DNS) while *diverting* transit is a
**`ForwardingDevice<D>`** that wraps each phy and is what `iface.poll` pulls
from. On `receive()` it peeks each frame and either lets smoltcp have it (local)
or diverts it to the other interface (transit). New module: **`src/forward.rs`**.

```
   cyw43 NetDriver                                       EthMac (core-1 RX inbox)
        │                                                       │
   ┌────▼─────────────────┐                          ┌──────────▼───────────┐
   │ ForwardingDevice<Cyw43Phy>                       │ ForwardingDevice<EthMac>
   │  receive(): peek dst  │                          │  receive(): peek dst │
   │   local→smoltcp       │                          │   local→smoltcp      │
   │   transit→LAN_TO_WAN ─┼──────► LAN_TO_WAN ───────┼─► drain → egress out │
   │  drain WAN_TO_LAN ◄───┼──────  WAN_TO_LAN ◄──────┼── transit→WAN_TO_LAN │
   │   → egress out cyw43  │       (2 embassy Channels)│                      │
   └───────────────────────┘                          └──────────────────────┘
        net_task (core 0)                                    wan_task (core 0)
```

Three new shared statics in `forward.rs`:
- **Two forward channels** — `embassy_sync::channel::Channel<CriticalSectionRawMutex,
  heapless::Vec<u8, 1600>, N>` (verified API: `try_send`/`try_receive`). `LAN_TO_WAN`
  carries frames the LAN side diverted (wan_task drains + egresses); `WAN_TO_LAN`
  the reverse. Both tasks are on the **one core-0 executor** (cooperative, no
  preemption between them), so a critical-section channel is simple + sufficient;
  it also covers the TIMER0 IRQ. `try_send` full → drop + bump a counter.
- **Two neighbor tables** (one per interface) — small fixed `IP→MAC` maps,
  **passively learned** (§5). Per-interface so egress resolves next-hop on its own
  interface; no cross-task sharing of the table (only the channels cross tasks).

`ForwardingDevice<D>` itself holds only `inner: D` + `Copy` config (our MAC, our
IP, our /24 subnet, and — WAN side — the DHCP-learned default gateway). All
*mutable* forwarding state lives in the statics, so it doesn't conflict with the
`&mut self.inner` borrow the smoltcp TxToken holds (§4).

**Why a wrapper, not a single router task that drains both phys manually:** if we
drained a phy ourselves we'd have to *re-inject* local frames into smoltcp, which
smoltcp can't do. Letting `iface.poll` pull *through* the wrapper is the only way
to keep smoltcp processing local traffic while we skim off transit. Keeps R15b's
two-task structure intact — R16 just wraps the two devices + drains the opposite
channel each loop.

---

## 4. `ForwardingDevice<D>` — receive/transmit, token lifetimes, the divert loop

`phy::Device::receive(&mut self) -> Option<(RxToken<'_>, TxToken<'_>)>`.

```rust
fn receive(&mut self, ts) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
    loop {                                    // skip past transit frames in one poll
        let (rx, tx) = self.inner.receive(ts)?;   // None → inbox truly empty → return None
        let mut frame: Vec<u8, 1600> = Vec::new();
        rx.consume(|buf| frame.extend_from_slice(buf).ok());  // materialize (rx borrow ends)
        forward::learn(self.iface, &frame);        // passive neighbor learning (§5), into a static
        match self.classify(&frame) {              // reads Copy config only
            Class::Local => return Some((ReplayRx(frame), tx)),   // hand to smoltcp
            Class::Transit => { let _ = self.egress_chan.try_send(frame); /* drop tx, keep looping */ }
            Class::Drop    => { /* TTL=0 / unroutable — drop, keep looping */ }
        }
    }
}
fn transmit(&mut self, ts) -> Option<Self::TxToken<'_>> { self.inner.transmit(ts) }  // smoltcp's own TX
```

Key points:
- **`Self::RxToken` = `ReplayRx(Vec<u8,1600>)`** — an owned, no-lifetime token whose
  `consume(f)` calls `f(&vec)`. (Same trick `EthMac`'s `EthRxToken` uses.) So local
  frames are replayed into smoltcp from the copy we peeked.
- **`Self::TxToken<'a> = D::TxToken<'a>`** — delegated. The inner `tx` co-borrows
  `self.inner` alongside the (already-consumed) `rx`; returning it for local frames
  lets smoltcp emit ARP/ICMP replies normally. For transit/drop we let `tx` drop.
- **The divert loop** matters: returning `None` on a transit frame would tell
  smoltcp "RX empty" and stall *local* frames queued behind it for a poll cycle.
  Looping until a local frame (or true empty) drains the whole inbox each poll.
- **One copy per frame.** For `Cyw43Phy` the cyw43 token is borrow-based, so a copy
  is unavoidable. For `EthMac` the inbox already hands us an owned `Vec` — a future
  optimization can move it instead of copying (R16: copy for uniformity/correctness).

`egress_chan` is the static channel this interface diverts *to* (`forward::LAN_TO_WAN`
for the LAN device, `forward::WAN_TO_LAN` for the WAN device).

---

## 5. Neighbor learning + next-hop resolution (the on-subnet rule)

Forwarding needs the **next-hop MAC** for the egress frame, and smoltcp's neighbor
cache is private. So `forward.rs` keeps its own tiny `IP→MAC` table per interface,
**passively learned** from received frames — with one essential rule:

> **Only learn `src-IP → src-MAC` when `src-IP` is on this interface's subnet.**

Otherwise a ping reply from `8.8.8.8` (which arrives with the *gateway's* src-MAC)
would wrongly map `8.8.8.8 → gatewayMAC`. On-subnet-only correctly learns the WAN
gateway (`192.168.37.19 ∈ WAN/24`) and each LAN client (`192.168.4.x ∈ LAN/24`),
and ignores off-subnet sources. The gateway + clients are chatty (DHCP, ARP,
pings), so the table fills within the first packets.

**Next-hop selection** at egress on interface X (subnet `Sx`, gateway `Gx`):
- `dst-IP ∈ Sx` → next hop = `dst-IP` (on-link) → look up its MAC.
- `dst-IP ∉ Sx` → next hop = `Gx` (the egress interface's gateway) → look up `Gx`'s MAC.
- MAC unknown → **drop** (bump a counter); succeeds once traffic has populated the
  table. (R16 keeps it simple; emitting our own ARP on a miss is a possible later
  refinement, but passive learning suffices for the chatty test flows.)

- **LAN egress** (cyw43, carries WAN→LAN traffic): the dst is always a LAN client
  on `192.168.4.0/24`, so next hop = dst directly → look up the client's MAC.
- **WAN egress** (10BT, carries LAN→WAN traffic): the dst is e.g. `192.168.37.19`
  (∈ WAN/24 → on-link, next hop = dst) or `8.8.8.8` (∉ WAN/24 → next hop = the WAN
  gateway `192.168.37.19`). The WAN gateway `Gx` is the DHCP-learned `WanState.gw`,
  already tracked.

---

## 6. Static routing decision (which egress)

R16 hardcodes the two-subnet topology (a real route table is overkill until there
are >2 interfaces):
- **LAN `ForwardingDevice`:** transit (dst-IP ≠ our LAN IP) → **WAN egress**
  (`LAN_TO_WAN`). (dst on LAN-but-not-us = LAN↔LAN, handled by the cyw43 AP's own
  intra-BSS bridging — shouldn't reach us; ignore.)
- **WAN `ForwardingDevice`:** transit with **dst-IP ∈ `192.168.4.0/24`** → **LAN
  egress** (`WAN_TO_LAN`); any other transit dst → **drop** (R16 doesn't route
  WAN↔WAN / arbitrary destinations).

A 2–4 entry static route table (`dst-cidr → egress`) would generalize this cleanly
and is cheap; optional for R16.

---

## 7. Egress: L2 rewrite + TTL/checksum (verified smoltcp wire APIs)

Each task, after `iface.poll`, drains the channel pointing **at** its interface
(`wan_task` drains `LAN_TO_WAN`; `net_task` drains `WAN_TO_LAN`) and egresses each
frame through the inner phy's TX token:

```rust
while let Ok(mut frame) = ingress_chan.try_receive() {
    // L3: decrement TTL, drop at 0, refresh the IPv4 header checksum.
    let mut eth = EthernetFrame::new_checked(&mut frame[..])?;     // verified
    {
        let mut ip = Ipv4Packet::new_checked(eth.payload_mut())?;  // verified
        let ttl = ip.hop_limit();
        if ttl <= 1 { continue; }              // drop (R16: no ICMP time-exceeded — polish)
        ip.set_hop_limit(ttl - 1);
        ip.fill_checksum();                     // verified
        let nexthop = nexthop_ip(ip.dst_addr(), my_subnet, my_gw);   // §5
        let Some(dmac) = forward::lookup(self.iface, nexthop) else { continue; }; // drop if unknown
        eth.set_dst_addr(dmac);                 // verified
        eth.set_src_addr(my_mac);               // verified — egress interface's MAC
    }
    // TX out this phy (EthMac → FCS + IFG + CSMA; cyw43 → NetDriver).
    if let Some(tx) = self.inner.transmit(now) {
        tx.consume(frame.len(), |b| b.copy_from_slice(&frame));
    }
}
```

`EthMac`'s `send_raw_frame` (via its TxToken) already prepends preamble/SFD,
appends FCS, and does carrier-sense + CSMA/CA + IFG (gotcha #9) — so forwarded
frames get the same disciplined TX as smoltcp's. The cyw43 TxToken hands the frame
to the NetDriver.

---

## 8. Wiring into `net_task` / `wan_task`

Minimal change to R15b's two tasks:
1. Wrap the phy: `let mut dev = ForwardingDevice::new(Cyw43Phy::new(net), our_lan_mac,
   LAN_IP, LAN_CIDR, None /*no gw*/, &forward::LAN_TO_WAN);` (LAN) and the analogous
   `EthMac` wrap in `wan_task` (gateway = `WanState.gw`, kept current as DHCP updates).
2. `iface.poll(now, &mut dev, &mut sockets)` — unchanged call; the wrapper diverts
   transit during the poll.
3. After the existing control handling, **drain the opposite channel** and egress
   (§7). For `wan_task`, the WAN gateway for next-hop comes from `WanState.gw`, so
   the egress closure reads the current lease's gateway.
4. Add `fwd_l2w` / `fwd_w2l` / `fwd_drop` counters to the `[Wan]`/`[Cyw43]` telemetry
   so forwarding is observable on-device.

Everything stays on the core-0 executor; core 1 still only decodes 10BT RX into the
inbox (which the WAN `ForwardingDevice` drains via `EthMac::receive`). No new cores,
no new IRQs.

---

## 9. Test harness (extend `tools/wan-test-host.sh`)

R16 needs a **route back** on the WAN host (the no-NAT requirement) — the upstream
must know how to return packets to the LAN subnet:

```bash
# After the Pico has its WAN lease (192.168.37.129):
sudo ip route add 192.168.4.0/24 via 192.168.37.129 dev enp1s0f0
# rp_filter can drop 192.168.4.x arriving on enp1s0f0 (asymmetric-looking) — loosen it:
sudo sysctl -w net.ipv4.conf.enp1s0f0.rp_filter=2
```

**Pure no-NAT acceptance** (the clean R16 test): from a Wi-Fi client on the AP
(default route = the Pico `192.168.4.1`, via the LAN DHCP), reach the WAN host
*directly*:
```bash
ping -c 3 192.168.37.19          # from the LAN client
curl -s --max-time 5 http://192.168.37.19/   # if the host serves one
# On the host, confirm the source IP is preserved (NOT NAT'd):
sudo tcpdump -ni enp1s0f0 'host 192.168.4.0/24'   # expect 192.168.4.x > 192.168.37.19
```
Accept = replies return **and** the host sees the LAN client's real `192.168.4.x`
source. That proves L3 forwarding without NAT.

**Internet-via-host-NAT demo** (optional, not the R16 bar): to let the LAN client
reach `8.8.8.8` *without* Pico-NAT (that's R17), widen the host masquerade to cover
the LAN subnet (`nft ... ip saddr 192.168.4.0/24 oifname eno1 masquerade`). The Pico
still just forwards; the *host* NATs. Fold a `--lan-route` / wider-masquerade option
into `wan-test-host.sh`, reverted on teardown.

Read the device counters over CDC (`/dev/ttyACM1`, 0666) as before — `fwd_l2w` /
`fwd_w2l` should climb in step with the client's traffic.

---

## 10. Risks & open questions

1. **`ForwardingDevice` token lifetimes (the keystone).** The wrapper must consume
   the inner RxToken (to peek/copy) yet return the co-borrowed inner TxToken for
   local frames, with a GAT `RxToken`/`TxToken<'_>`. The `ReplayRx(Vec)` owned token
   + delegated `D::TxToken<'a>` should satisfy it (EthMac already models the owned
   RxToken), but this is the fiddliest part — prototype it first against both inner
   devices.
2. **cyw43 AP sees transit at all?** A STA→internet frame arrives at the AP with
   dst-MAC = our gateway MAC, so cyw43's `NetDriver` *should* hand it up (it's
   addressed to us at L2). Verify on-device early — if the cyw43 firmware does its
   own L3 handling/filtering in AP mode, the tap point shifts. (R15b already shows
   client→`192.168.4.1` frames arrive, so L2-to-us delivery works.)
3. **Passive learning gaps.** First transit packet to an unlearned next-hop drops.
   Acceptable for the chatty test (gateway/client announce themselves fast); if it
   bites, add active ARP on miss. The on-subnet learning rule (§5) is essential —
   get it wrong and off-subnet dsts resolve to the wrong MAC.
4. **`rp_filter` / host route-back.** The most likely "it doesn't work" cause is the
   host silently dropping the returning/forwarded packets (reverse-path filter, or
   missing route back) — not the Pico. The §9 `ip route` + `rp_filter=2` address it;
   check `tcpdump` on *both* host interfaces when debugging.
5. **Throughput / copies.** Every forwarded frame is copied (peek) + moved through a
   1600-byte channel slot + copied into the TxToken. Fine for R16's low-rate
   correctness test; for sustained throughput (R17+/router-plan throughput goals)
   revisit (move the EthMac inbox Vec; zero-copy channel; larger windows).
6. **Latency.** LAN→WAN crosses two core-0 tasks (net_task enqueue → wan_task drain),
   so a forwarded packet waits up to one scheduling round (~poll cadence). Tighten
   the poll interval or signal across tasks if it matters; fine for R16.

---

## 11. Step checklist

- [x] Prototype `ForwardingDevice<D>` token plumbing against both `Cyw43Phy` and
      `EthMac` (Risk 1) — `receive` (divert loop + owned `ReplayRxToken`) + delegated
      `D::TxToken<'a>` compile for both. (Risk 1 keystone cleared.)
- [x] `src/forward.rs`: the two `Channel` statics + per-interface `NeighborTable`
      (passive on-subnet learning) + `classify` + `nexthop` + egress L2/TTL rewrite.
- [x] Wrap the LAN device in `net_task`; divert transit → `LAN_TO_WAN`; drain
      `WAN_TO_LAN` → egress out cyw43.
- [x] Wrap the WAN device in `wan_task`; divert transit (dst ∈ LAN/24) → `WAN_TO_LAN`;
      drain `LAN_TO_WAN` → egress out EthMac (next-hop via the lease's `gw`).
- [x] `l2w`/`w2l`/`sent`/`drop` counters — on their own `[Fwd]` CDC line (split off the
      `[Wan]` line + `cdc_write_all` flush, so the counters survive CDC framing).
- [x] Extend `tools/wan-test-host.sh` with the LAN route-back (+ `rp_filter`) and the
      optional `NAT_LAN=1` wider-masquerade for the internet demo.
- [~] Validate (2026-06-02, `--features router`, SWD-flashed): **LAN→WAN DONE** — phone
      on the AP, off-LAN traffic forwarded, `fwd l2w==sent` >1400 / `drop≈0`, host
      `tcpdump` shows source `192.168.4.x` preserved (no NAT); LAN intact; WAN's own
      ping/DNS work; core 1 up. **WAN→LAN code-complete but NOT observed** — the iOS test
      client wouldn't route over the no-internet Wi-Fi (cellular fallback); host
      `tcpdump 'dst host 192.168.4.10'` saw 0 return frames → Pico exonerated. Closing it
      needs a deterministic non-iOS client (laptop pinging `192.168.37.19`).
- [x] Build all 4 configs clean + clippy clean (only the 2 pre-existing warnings); commit;
      update `RESUME.md` + this checklist.
- [ ] (carry-over) fix the stale `router-plan.md` §12 "unification is R16's" wording
      (now R15b); finish the WAN→LAN observation with a non-iOS client.
```
