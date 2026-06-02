# R17 — NAPT + connection tracking (the milestone)

**Goal:** a phone on the WiFi AP browses the *real* internet through the Pico,
with the **Pico itself** doing the NAT (not the host). This is the project
milestone (`router-plan.md` §7). R16 proved L3 forwarding (LAN→WAN transit,
source preserved); R17 adds NAPT so many LAN clients share the single WAN IP and
return traffic finds its way back.

Prereq context: read `docs/r16-plan.md` (the forwarding layer this builds on) and
`router-plan.md` §6.2. The code lives beside R16's `src/forward.rs`.

---

## 1. Goal & acceptance

**Acceptance (= `router-plan.md` §7's R17 row):** *a phone on `pico-rp2350-router`
(default route = the Pico, normal DHCP, **no static host route, no host NAT**)
loads a real web page / pings `8.8.8.8` and gets answers.* Concretely, with
`tools/wan-test-host.sh` run at **`NAT_LAN=0`** (the host does NOT masquerade the
LAN subnet — the Pico must do it):

```bash
# from the LAN client:
ping -c 3 8.8.8.8                 # replies return
curl -s --max-time 8 http://example.com/   # page loads
# on the host WAN NIC, the client's packets appear NAT'd to the Pico's WAN IP:
tcpdump -ni enp1s0f0 'host 8.8.8.8'    # expect 192.168.37.129 > 8.8.8.8 (NOT 192.168.4.x)
```

Accept = replies return to the client **and** the host sees the Pico's WAN IP
(`192.168.37.129`) as the source, not the client's `192.168.4.x`. That's the
inverse of R16's check (R16 wanted the source *preserved*; R17 wants it *rewritten*).

This also finally closes the R16 WAN→LAN observation gap (return traffic now comes
back to the WAN IP and is conntrack-routed to the client) — and removes the need
for the host route-back + `rp_filter` loosening entirely.

---

## 2. Where NAT sits — the WAN boundary

R16 has two `ForwardingDevice`s (LAN = cyw43, WAN = 10BT) exchanging transit
frames over the `LAN_TO_WAN` / `WAN_TO_LAN` channels; `egress()` re-emits a drained
frame out the owning phy. **All NAT happens at the WAN `ForwardingDevice`** — it
already knows `our_ip` (the WAN IP = the NAT address, tracked via `set_lease`) and
`gateway` (the nexthop). The LAN side keeps forwarding *already-translated* frames.
This keeps the conntrack table single-owner (the `wan_task` on core 0) — no locking
across tasks for the hot path.

Two hooks:

1. **Outbound — WAN `egress()` (LAN→WAN), drained from `LAN_TO_WAN`:**
   before the existing TTL/checksum/L2 rewrite, NAPT-rewrite the source:
   `src_ip → our_ip`, `src_l4_id → an allocated WAN id` (TCP/UDP port, or ICMP echo
   id), then insert/refresh a conntrack entry. Fix IP + L4 checksums.

2. **Inbound — WAN `receive()` classify (WAN→LAN):** **this is the keystone
   change.** Under NAPT the reply's dst is `our_ip`, which R16's `classify()` returns
   `Local`. R17 inserts a conntrack lookup *before* the `dst == our_ip → Local`
   verdict: if `(proto, src=remote, src_l4=remote_port, dst_l4=allocated_id)` matches
   a live entry, it's a **NAT-return** → rewrite dst back to the client
   (`our_ip:alloc → lan_client:orig`), fix checksums, divert to `WAN_TO_LAN`. No
   match → `Local` (the Pico's *own* ping/DNS replies fall through here untouched,
   because conntrack only holds entries we created).

The LAN `net_task` then drains `WAN_TO_LAN` and `egress()`-es to the client exactly
as in R16 (nexthop = the on-subnet client, already learned). No NAT on the LAN side.

```
LAN client 192.168.4.10:51000 ── to 8.8.8.8:443 ──▶ [LAN dev] Transit ──▶ LAN_TO_WAN
                                                                              │
   [WAN egress] NAPT: src 192.168.4.10:51000 → 192.168.37.129:GP;  conntrack.insert
                TTL--, L4+IP csum fixup, TX ──▶ 8.8.8.8 sees 192.168.37.129:GP
   8.8.8.8:443 ── reply to 192.168.37.129:GP ──▶ [WAN recv] dst==our_ip → conntrack.match!
                NAPT: dst 192.168.37.129:GP → 192.168.4.10:51000;  csum fixup ──▶ WAN_TO_LAN
                                                                              │
   [LAN egress] nexthop = 192.168.4.10 (learned), L2 rewrite, TX ──▶ client
```

---

## 3. Conntrack table (`src/conntrack.rs`, new)

Fixed-size, heapless, no alloc, single-owner (wan_task). The bulk of the new code.

**Entry (per tracked flow):**
| field | use |
|---|---|
| `proto` | TCP / UDP / ICMP-echo |
| `lan_ip`, `lan_id` | original LAN client src IP + src port (or ICMP id) |
| `remote_ip`, `remote_id` | the WAN peer IP + port (dst of the outbound flow) |
| `wan_id` | the port/id we allocated on the WAN IP (the NAT handle) |
| `last_seen` | for idle-timeout eviction (ms from `embassy_time`) |
| `tcp_state` | coarse: SYN-seen / established / FIN-seen (TCP only) |

**Two lookups (both O(N) scan over a small table, or a tiny hash):**
- **Outbound** key `(proto, lan_ip, lan_id, remote_ip, remote_id)` → entry (find-or-
  create, allocating a fresh `wan_id`). Refresh `last_seen`.
- **Inbound** key `(proto, remote_ip=src, remote_id=src_port, wan_id=dst_port)` →
  entry. Refresh `last_seen`. Miss ⇒ not ours ⇒ `Local`.

**Port/id allocation:** a dedicated ephemeral range for the WAN side, e.g.
`49152..=65535`, kept **disjoint from anything smoltcp uses for the WAN's own
sockets** (its DHCP/DNS ephemeral ports + the ICMP id `0x42`) so a NAT-allocated id
can never shadow the Pico's own traffic. Allocate by linear probe from a rolling
cursor; on exhaustion, evict the LRU entry (or drop + count). ICMP echo: the "port"
is the echo `id` (same range concept).

**Sizing:** start `CT_CAP = 64` entries (a phone idles ~dozens of flows). Each entry
~32–40 B ⇒ ~2.5 KB. Eviction: idle timeout per proto (UDP ~30 s, ICMP ~10 s, TCP
~established 60 s / handshake-or-FIN ~10 s) + LRU on full. **Log evictions/drops**
(no silent caps).

---

## 4. Per-protocol handling

- **UDP** — rewrite src/dst port; idle timeout. (DNS to the upstream is just UDP:53
  NAT'd like anything else, satisfying §6.4's "simplest" path — no separate relay.)
- **TCP** — rewrite src/dst port; coarse state machine off the flags (SYN → new,
  SYN+ACK seen → established, FIN/RST → closing) only to pick a timeout; we do **not**
  do full TCP tracking/seq validation (out of scope; a NAT, not a firewall).
- **ICMP echo** — rewrite the `id` field (acts as the "port"); match replies by id.
  **ICMP errors** (dest-unreachable/time-exceeded) embed the original IP+L4 header —
  R17 may ignore these initially (count + drop) and add embedded-header rewriting as
  a follow-up; note the limitation, don't pretend it works.

---

## 5. Checksums — incremental update

Every rewrite changes IP src/dst and an L4 port/id, so fix:
- **IPv4 header checksum** — incremental (RFC 1624 `HC' = ~(~HC + ~m + m')`) over the
  changed 16-bit words; cheaper and less error-prone than full recompute.
- **L4 checksum** — TCP/UDP carry a pseudo-header over src/dst IP **and** the L4
  ports, so both the IP change and the port change must fold in. UDP checksum `0`
  means "none" → leave it `0`. ICMP echo checksum covers the `id` → fold the id delta.
- Reuse smoltcp's `wire` checksum helpers where they fit; otherwise a small
  `incr_checksum(old, new, &mut sum)` util. Unit-test the incremental math against a
  full recompute on a corpus before trusting it on-wire (mirrors R10's offline check).

---

## 6. Telemetry

Extend the `[Fwd]` line (R16's `cdc_write_all` already makes long lines safe):
`[Nat] ct=<live>/<cap> out=<n> in=<n> evict=<n> drop=<reason counts>`. Counters as
`AtomicU32` in `conntrack.rs`, formatted in `usb_task`. Surfacing `ct` live-count +
evictions is how we'll see table pressure (Risk below).

---

## 7. Test harness (`tools/wan-test-host.sh`)

R17 is what lets us **stop** faking it host-side:
- Run at **`NAT_LAN=0`** — the host must NOT masquerade `192.168.4.0/24`; the Pico
  NATs. (If it still works with the host masquerade *on*, you haven't proven Pico
  NAT — turn it off.)
- The R16 **route-back + `rp_filter` loosening become unnecessary** (return traffic
  now arrives at the Pico's WAN IP, not at a `192.168.4.x` the host must route).
  Leave them harmless or gate them off; note in the script.
- Use a **deterministic client** to dodge the R16 iOS-cellular-fallback trap: a
  laptop on the AP, or a phone in airplane-mode+WiFi. `ping 8.8.8.8` + `curl` + the
  `tcpdump -ni enp1s0f0 'host 8.8.8.8'` source check (§1).

---

## 8. Risks & open questions

1. **The WAN-ingress classify change is load-bearing and subtle.** Mis-scoping the
   conntrack match can either (a) steal the Pico's own ping/DNS replies (if a NAT id
   collides with smoltcp's) or (b) leak forwarded replies into our stack. The
   disjoint-id-range rule (§3) + "conntrack-miss ⇒ Local" must be exactly right.
   Verify the WAN's own `ping=N/N` keeps climbing *while* a client is NAT'd.
2. **Core balance (the standing open question, §8.3 router-plan).** R16 already noted
   forwarding adds core-0 load; NAPT adds the conntrack scan + checksum fixups per
   forwarded frame. Measure (the project MO) — if core 0 saturates, move the conntrack
   fast-path or the cyw43 Runner to core 1.
3. **Port/id exhaustion** under a chatty client — bounded table + LRU; `log()` drops.
4. **Checksum math** — incremental updates are a classic bug source; unit-test first.
5. **ICMP errors / fragmentation** — punted initially (count + drop); document.
6. **MTU / TCP MSS** — half-duplex 10BT + the R16 large-frame reliability tail mean
   full-MSS forwarded TCP may suffer; MSS clamping is a possible R17.x follow-up, not
   the acceptance bar.

---

## 9. Step checklist

- [ ] `src/conntrack.rs`: the heapless table + two lookups + allocator + timeout/LRU
      eviction + `incr_checksum` util. Unit-test the checksum math + alloc/evict.
- [ ] WAN `egress()` (outbound): NAPT src rewrite + conntrack insert/refresh + csum.
- [ ] WAN `receive()` classify (inbound): conntrack-aware — match → dst rewrite +
      `Transit`/`WAN_TO_LAN`; miss → `Local`. Keep the Pico's own sockets working.
- [ ] Per-proto (TCP coarse-state, UDP, ICMP-echo id).
- [ ] `[Nat]` telemetry (live count / evict / drop).
- [ ] `wan-test-host.sh`: document `NAT_LAN=0` as the R17 mode; retire/gate the R16
      route-back + `rp_filter` (now unnecessary).
- [ ] Validate (deterministic client, `NAT_LAN=0`): `ping 8.8.8.8` + `curl` from a LAN
      client; host `tcpdump` shows source = the Pico WAN IP; WAN's own ping/DNS intact;
      LAN/AP intact; core 1 up; measure core-0 headroom (Risk 2).
- [ ] Build all 4 configs + clippy clean; commit; update `RESUME.md` + this checklist.
- [ ] (carry-over from R16) fix the stale `router-plan.md` §12 "unification is R16's"
      wording (now R15b).
