# R19 — robustness: cold-start gateway-ARP (the `[Fwd] drop≈4` fix)

**Goal:** eliminate the constant cold-start `[Fwd] drop≈4` — the handful of
forwarded LAN→WAN frames dropped at the start of a session before the WAN
neighbor table has learned the gateway's MAC.

R19's other scoped item, **true per-bit collision-detect**, is **deferred** (hard/
fragile PIO, explicitly "optional polish, not a blocker" — CSMA/CA from R12e
already holds collisions to ~0.5/curl). See `router-plan.md` §7 (R19+) and the
"Optional MAC polish" note. This doc covers the cold-start fix (call it R19a).

---

## 1. Goal & acceptance

**Acceptance:** at cold-start, a LAN client's forwarded traffic shows `[Fwd]
drop=0` in the normal case (client joins after the router is up). Concretely:
reboot the router, let the WAN lease land, join a client, generate forwarded
traffic (`curl`/`dig`/browse) → `[Fwd] sent` climbs with `drop=0`.

---

## 2. Root cause

The WAN `ForwardingDevice::egress()` resolves the next-hop (gateway) MAC from the
`WAN_NEIGH` table, which is populated **only** by the passive `learn()` (snooping
on-subnet source MACs from ARP + IPv4). But `learn()` is gated off until the DHCP
lease sets `our_ip` (forwarding is disabled pre-lease). So immediately after the
lease lands there's a window where:

- the LAN side already diverts a client's transit frames into `LAN_TO_WAN`
  (its gateway IP `192.168.4.1` is static, so it forwards from boot), and
- the WAN `egress()` has no gateway MAC yet → `FWD_DROP` ("next-hop MAC not
  learned yet", `forward.rs`).

The table self-heals within ~1 s — the Pico's own `ping 8.8.8.8` makes smoltcp ARP
the gateway, and `learn()` catches that reply — but the first few client frames in
that window drop. That's the documented constant `≈4`.

---

## 3. Fix — proactively ARP the gateway on lease

Don't wait for the passive learner; actively ARP the gateway the moment the lease
provides one, so `WAN_NEIGH` is warm before any client frame.

**`src/forward.rs`:**
- `build_arp_request(our_mac, spa, tpa) -> [u8; 42]` — a broadcast "who-has tpa,
  tell spa" Ethernet/IPv4 ARP request. Byte layout matches `learn()`'s ARP parser,
  so the gateway's *reply* repopulates `WAN_NEIGH`. `EthMac::send_raw_frame` pads
  the 42 B to the 60 B minimum + appends FCS (same path as smoltcp's own ARP).
- `ForwardingDevice::arp_gateway(now)` — builds the request for `cfg.gateway` and
  TXs it via the inner phy's `transmit()` token. No-op until a gateway + IP are
  leased.
- `pub fn wan_neigh_known(ip) -> bool` — lets the caller stop re-ARPing once warm.

**`src/wireless.rs` `wan_task`:**
- **Immediate:** track `prev_gw`; the instant the lease's gateway first appears (or
  changes), `arp_gateway(now())` right away — don't wait for the 1 Hz tick.
- **Retry:** in the 1 Hz block, while `wan.gw` is set but `!wan_neigh_known(gw)`,
  re-ARP once per second (robust to a reply lost on the half-duplex wire), then
  stop — no steady-state spam.

Nothing else changes: the reply flows through the existing `receive()` → `learn()`
→ `WAN_NEIGH.insert()`, and `egress()` then finds the gateway MAC.

---

## 4. Validation (on-device, `--features router`, SWD-flashed)

| Scenario | `[Fwd] drop` |
|---|---|
| **Normal cold-start** (client joins after the router is up) | **0** ✅ |
| Worst-case concurrent cold-start (client already blasting at boot) | **1** (was ≈4) |

- **Normal:** fresh boot with no client → `[Fwd] 0/0/0/0`, WAN up (`ip=192.168.37.129
  gw=192.168.37.19 ping=5/5`). Client rejoins + `curl`/`dig` → `[Fwd] l2w=11 w2l=11
  sent=22 drop=0`, `[Nat] out=11 in=11 drop=0`. **drop=0 confirmed.**
- **Worst case:** the laptop stayed associated across the reflash and forwarded the
  instant the router booted → a single transient `drop=1` (the first frame races the
  gateway-ARP round-trip / arrives before the WAN lease fully lands — a frame that
  can't be forwarded yet regardless), then `drop` frozen.

The residual worst-case `1` is the practical floor without a packet-queue-pending-
ARP buffer; **accepted** (the constant `≈4` is eliminated, and the normal case is
0). A real ARP hold-and-retry queue for a true 0 is noted as optional future polish.

---

## 5. Risks & notes

1. **Half-duplex ARP loss** — a lost request/reply just means the 1 Hz retry fires
   again next second; bounded, self-correcting.
2. **No steady-state cost** — `arp_gateway` stops once `wan_neigh_known(gw)`; the
   immediate path is gated on the `prev_gw` transition.
3. **Gateway change / re-lease** — the `prev_gw` transition re-fires the ARP, and
   `set_lease` already tracks the new gateway, so a DHCP gateway change re-warms.

---

## 6. Step checklist

- [x] `forward.rs`: `build_arp_request`, `ForwardingDevice::arp_gateway`,
      `wan_neigh_known`.
- [x] `wan_task`: immediate ARP on the gateway transition + 1 Hz retry until warm.
- [x] Build all 4 configs + clippy clean (only the 2 pre-existing warnings).
      **⚠️ flash gotcha: force-recompile the router variant LAST + verify
      `strings <ELF> | grep -F "[Nat] ct="`.**
- [x] On-device validated: normal cold-start `[Fwd] drop=0`; worst-case concurrent
      cold-start `drop=1` (was ≈4). WAN/LAN/NAT intact.
- [x] Commit to `main`; update `RESUME.md` + this doc.
- [ ] (deferred) true per-bit collision-detect; optional ARP hold-and-retry for a
      true worst-case 0; then backpressure / conntrack-pressure, then low-power.
