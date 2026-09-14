# docs/ — engineering log & design notes

This folder is the project's **engineering log**: how the software 10BASE-T NIC and
the router were designed, built, and characterized — including the dead ends and
the honest limits. It's deliberately kept (not pruned) because the "how we got
here" is half the value of a project like this.

## Start here

- **[performance.md](performance.md)** — *what to expect*: measured throughput,
  latency, CPU, and the honest limits. Read this first.
- **[router-plan.md](router-plan.md)** — overall router architecture: the "Option-A"
  decision (keep Hazard3 RISC-V, port the cyw43 transport — no embassy-rp), the
  PIO/DMA/core layout, the data path.

## Key findings (the interesting bits)

- **[rx-bulk-ceiling.md](rx-bulk-ceiling.md)** — the RX-of-bulk investigation. Its
  "PHY limit at ~100 KB/s" verdict was later overturned: the cause was DMA-starved
  sample loss, and RX now runs ~310 KB/s (see [performance.md](performance.md)).
- **[full-duplex-analysis.md](full-duplex-analysis.md)** — the ISL3177E is
  full-duplex-*capable* (half-duplex is a MAC policy). Forced-FD on-wire experiment:
  FD helps only contended traffic and is still RX-decode-bounded → not worth the
  auto-negotiation work.
- **[cpu-dpll-plan.md](cpu-dpll-plan.md)** + **[clock-recovery-decoder-plan.md](clock-recovery-decoder-plan.md)**
  — the RX Manchester decoder: an edge-tracking DPLL on core 1 that cancels clock
  drift; the offline corpus bench; and the §9d "residual is PHY-limited" verdict (since overturned).
- **[pio-dpll-report.md](pio-dpll-report.md)** — retrospective on the *PIO-side*
  decoder attempt (capped ~40% full-MTU → pivoted to the CPU DPLL on core 1).

## The build log (roughly chronological)

- **[pio-decoder-plan.md](pio-decoder-plan.md)** — the PIO clock-recovery decoder plan.
- **[r15-plan.md](r15-plan.md)** — WAN-as-DHCP-client + unifying both interfaces under one executor.
- **[r16-plan.md](r16-plan.md)** — L3 forwarding (LAN↔WAN transit).
- **[r17-plan.md](r17-plan.md)** — NAPT / conntrack (the router milestone).
- **[r18-plan.md](r18-plan.md)** — DNS relay + management status page.
- **[r19-plan.md](r19-plan.md)** — cold-start gateway-ARP fix.
- **[perf-characterization-plan.md](perf-characterization-plan.md)** — the perf track:
  the cyw43 LAN isolation + the gSPI 2→15 MHz fix, the routed-throughput rig.
- **[low-power-plan.md](low-power-plan.md)** — low-power roadmap (scoped, backburnered;
  gated on a power meter).

## Meta

- **[release-checklist.md](release-checklist.md)** — the open-source prep checklist.
- **[log.md](log.md)** — raw notes: history and lessons moved out of code comments.

> Note: some docs reference internal "R-numbers", "gotcha #N", and session notes
> (`RESUME.md`, not published) — that's the raw lab notebook.
