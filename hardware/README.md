# Hardware — 10BASE-T front-end board

KiCad project for the **10BASE-T analog front end** used by this project (and by
its [C ancestor](https://github.com/mattdeeds/Pico-10BASE-T)). It breaks an
**ISL3177E** RS-485 transceiver and an **HR911105A** RJ45 jack (integrated
magnetics) out to single-ended GP13 (RX) / GP14 (TX) on a Raspberry Pi Pico 2, so
the Pico's PIO can both drive and receive a real 10BASE-T differential line.

The circuit is based on the [Niccle](https://github.com/timonvo/niccle) reference
design. This is `v2` — a small prototype/test board, not a product.

> ⚠️ **Do not connect this to PoE equipment.** It's an educational software-PHY
> front end with no isolation guarantees beyond the magnetics.

## How it works

The ISL3177E is an RS-485 transceiver repurposed as a 10BASE-T line driver +
slicer:

- **TX:** Pico `GP14` → `DI` (driver input) → differential `Y`/`Z` → AC-coupling →
  HR911105A magnetics → RJ45 TX pair.
- **RX:** RJ45 RX pair → magnetics → differential `A`/`B` → `RO` (receiver output,
  single-ended Manchester) → Pico `GP13`, where the PIO sampler + software DPLL
  decode it.

See the repo's top-level [`README.md`](../README.md) ("How it works") for the
firmware side and the exact Pico pin map.

## Bill of materials

| Ref | Value | Footprint | Role |
|---|---|---|---|
| U1 | **ISL3177EIBZ** | SOIC-8 | RS-485 transceiver (TX driver + RX slicer) |
| J6 | **HR911105A** | RJ45 (Hanrun, horizontal) | jack + integrated magnetics |
| C5, C8, C9 | 100 nF | 0603 | supply decoupling |
| C6, C7 | 10 nF | 0603 | AC-coupling to the line |
| R28 | 100 Ω | 0603 | differential line termination |
| R29, R30 | 50 Ω | 0603 | source termination |
| J1 | 1×02 header | 2.54 mm | power (3V3 / GND) |
| J2, J3 | 1×04 header | 2.54 mm | signal / Pico connections |

(Source of truth: [`production/bom.csv`](production/bom.csv).)

## Files

- `10BASE-T_Test-v2.kicad_pro` / `.kicad_sch` / `.kicad_pcb` — the KiCad 10 project
  (schematic + PCB). Open `.kicad_pro` in KiCad.
- `production/` — fabrication outputs:
  - `10BASE-T_Test-v2.zip` — Gerbers + drill files (2-layer board), ready to send
    to any board house.
  - `bom.csv` — bill of materials.
  - `positions.csv` — pick-and-place / centroid (for assembly).
  - `designators.csv`, `netlist.ipc` — supporting fab data.

## Fabricating it

Upload `production/10BASE-T_Test-v2.zip` to a PCB fab (JLCPCB, PCBWay, OSH Park,
etc.) as a standard 2-layer board. For assembly, supply `bom.csv` + `positions.csv`.
To regenerate any of these, re-run the export from KiCad (the
[Fabrication Toolkit](https://github.com/bennymeg/Fabrication-Toolkit) settings
used are in `fabrication-toolkit-options.json`).
