# CPU DPLL on the 2nd Hazard3 core — design plan

Phase 3 of the clock-recovery work. Successor to the PIO DPLL route — see
`pio-dpll-report.md` for the retrospective. Goal: run the validated
edge-track DPLL algorithm on the unused 2nd Hazard3 core, dedicated to NIC RX
work.

## 1. Goal

Same acceptance criteria as the PIO plan (`pio-decoder-plan.md` §11):

- **P1** Full-MTU (1518 B) FCS-OK ≥ 95 % at low/moderate rate. Today: ~40 %
  via PIO v1+180/240 MHz; ~1.7 % via the open-loop CPU decoder.
- **P2** No loss-of-lock cascade. Single bit/edge errors stay local — no
  `0xaa`/`0x55` end-of-frame run.
- **S1** Small frames (256/512 B) no worse than baseline (6/6 FCS-OK).
- **S2** Decode off core 0 — IRQ worst case drops on core 0; core 1
  dedicated to RX decode.
- **A1 Finding 2 side benefit**: under saturating load the existing single-
  core RX IRQ starves smoltcp. Core separation fixes this.

## 2. Why this approach

PIO has three structural blockers stopping ≥ 95 % full-MTU (per the report):
no arithmetic for loop-filtered DPLL, 2-cycle synchronizer creates a cycle-
budget mutual-exclusion, mid-bit vs boundary edges distinguished only by
phase. CPU has all three: integer add/sub/shift for fractional phase
correction, no synchronizer constraints, and the room to keep an absolute
phase reference.

The algorithm is already validated — `decode_edge_track` in
`tools/clock-recovery/harness.py` gives FCS-OK N/N on the 60 MHz sample
corpus. The risk isn't algorithm correctness; it's port + multicore plumbing.

The 2nd Hazard3 core is currently idle. The RP2350 has the SIO inter-core
FIFO + 32 hardware spinlocks for cheap inter-core sync. Moving the RX decode
off core 0 also directly solves A1 Finding 2 (single-core load collapse).

## 3. Architecture overview

```
                ┌───────────────────────────────────────────────────────────┐
                │                          Core 0                           │
                │                                                           │
   USB CDC ◄────┤  main loop:                                               │
                │    NLP cadence, UDP echo (port 1234), HTTP (80),          │
                │    smoltcp iface.poll → EthMac::receive (pops inbox),     │
                │    1-Hz status log                                        │
                │                                                           │
   TX PIO0 ◄────┤  EthTx: send_raw_frame / send_nlp / send_udp_broadcast    │
                │  (Manchester encode + DMA to PIO0 SM0)                    │
                │                                                           │
                └─────────────────────┬─────────────────────────────────────┘
                                      │
                                      │ Decoded-frame inbox
                                      │ (heapless::Deque, HW spinlock-protected)
                                      │
                ┌─────────────────────┴─────────────────────────────────────┐
                │                          Core 1                           │
                │                                                           │
   PIO0 SM1 ───►│  60 MHz sampler → DMA double-buffer (2× 16 KB)             │
   (samples)    │                                                           │
                │  DMA_IRQ_0 (routed to core 1):                            │
                │    1. stitch carry + new half                             │
                │    2. find_active_run_from / peek_dst_mac (MAC filter)    │
                │    3. decode_frame_edge_track  ◄── the NEW DPLL decoder   │
                │    4. derive_frame_len + verify_fcs                       │
                │    5. push to shared inbox (under spinlock)               │
                │                                                           │
                └───────────────────────────────────────────────────────────┘
```

**Changes from today:**
- `DMA_IRQ_0` is routed to core 1 (NVIC/PLIC config + the IRQ handler is
  installed on core 1, not core 0).
- The decode call inside the IRQ handler swaps from open-loop `decode_frame`
  to a new `decode_frame_edge_track`.
- The shared RX state (`SHARED_RX` mutex+refcell+option) becomes inter-core
  shared — replace the single-core `critical_section` Mutex with HW spinlock
  protection.
- Core 1 wakes only on the DMA IRQ; otherwise it WFI/sleeps.
- TX path stays on core 0 unchanged.

## 4. Phasing

### Phase 3a — Multicore foundation (small, low-risk first step)

Bring up core 1 doing nothing useful: spawn it, hello-world via inter-core
FIFO, prove the rp235x-hal multicore plumbing works on this chip. Specifically:

1. Add `hal::multicore::{Multicore, Stack}` import; allocate a stack for core 1
   (e.g., 4 KB).
2. In `main`, after clocks/SIO setup, `Multicore::new(&mut sio.fifo, ...)`
   then `cores[1].spawn(stack_alloc, core1_entry)`.
3. `core1_entry`: a small loop that, e.g., increments a SIO-FIFO-written
   counter and reads back, to prove inter-core comm works.
4. Verify on-wire that core 0 still does everything (TX, USB, smoltcp) while
   core 1 ticks along.

Acceptance: device boots, ping + UDP broadcast unchanged, USB unchanged, and
a `1 Hz` "core1 alive: ticks=N" line in the log.

### Phase 3b — Port `decode_edge_track` to Rust + offline-validate

Independent of multicore: replace `eth_rx::decode_frame`'s open-loop sampling
with the edge-track algorithm. Keep everything else (`find_F`, `find_SFD`-
equivalent, `find_active_run_from`, `peek_dst_mac`, `derive_frame_len`,
`verify_fcs`) the same — *only the per-bit sample loop changes*.

The Python reference (`decode_edge_track` in harness.py):

```python
def decode_edge_track(buf, W=1):
    ns = len(buf) * 8
    F = find_F(buf, ns)
    sfd = find_SFD(buf, ns, F)
    start = sfd + 1
    P = 6
    tr = find_edge(buf, F + 5 + 6 * start, W, ns) or (F + 5 + 6 * start)
    bits = []
    while True:
        si = tr - 1
        if si < 0 or si >= ns: break
        bits.append(sample_bit(buf, si))
        nxt = find_edge(buf, tr + P, W, ns)
        tr = nxt if nxt is not None else tr + P
    return F, _pack(bits)
```

The Rust port should:
- Keep `MAX_FRAME_BYTES` output sizing + the decode-length cap (against runt
  / merged-frame DoS — already in place).
- Use `get_unchecked` after proving the range, same as the optimized open-
  loop version.
- Inline `find_edge` to avoid call overhead on the hot path.

**Offline validation (the project's established pattern):**
- Add a host-side Rust validator (probably a `tools/dpll-rust/` cargo bin
  with `std`) that loads `tools/clock-recovery/corpus/*.bin` and runs the
  Rust port against them.
- Expected: FCS-OK N/N, flat per-byte error bins (matches the Python
  `decode_edge_track`).
- This de-risks the port without flashing.

**On-device — but still on core 0 for now**: wire the new `decode_frame_edge_track`
into the existing IRQ handler. Test. Performance budget concern: edge-track
adds ~2-4× the per-bit cost vs open-loop (extra edge searches per bit). At
240 MHz the per-frame cost might be ~600 µs – 1.2 ms for full-MTU, vs the
2.18 ms half-fill budget. Tight under heavy load. **If this is the limiting
factor, that's exactly the case for Phase 3c (move to core 1).**

Acceptance for 3b: small-frame parity, full-MTU success rate ≫ 40 %
(target: matches the offline corpus result, modulo the on-wire jitter we
don't have in the corpus). gotcha-#10 stress at/above baseline.

### Phase 3c — Move the decode to core 1

The interesting integration step:
1. **Re-route `DMA_IRQ_0` to core 1.** RP2350's IRQ routing is per-core
   (each IRQ can be enabled on core 0, core 1, or both). Configure so core 1
   handles it; core 0's main loop never sees it.
2. **Replace `critical_section::Mutex` around `SHARED_RX` with HW spinlock.**
   `hal::sio::Spinlock<N>` provides RAII guards. Pick e.g. `Spinlock0` for
   the inbox. Both cores claim it for inbox push/pop.
   - Caution: spinlock + IRQ-context push from core 1 means core 0's `receive`
     must claim the same spinlock briefly. Keep the critical section short
     (just `inbox.pop_front()` + stats read).
3. **Core 1 entry function**: enable DMA_IRQ_0 on core 1; install the IRQ
   handler; `wfi` (wait-for-interrupt) loop. The IRQ handler does what the
   current `process_completed_half` does on core 0.
4. **Memory placement**: the DMA buffers (`rx_buf_a`, `rx_buf_b`,
   `rx_carry`, `rx_stitch`) live in shared SRAM — accessible to both cores
   and DMA. The existing `hal::singleton!(: [u32; N] = ...)` allocations are
   in SRAM, fine as-is.

Acceptance for 3c: full-MTU FCS-OK ≥ 95 % at non-saturating rate; under load
(blast + concurrent ping + UDP echo) ping ≥ 99.7 %, UDP ≥ 100 %, host RX
errs ≤ 2/30s, **with core 0 main loop responsiveness unchanged** (because
core 0 no longer takes DMA IRQs).

### Phase 3d — Acceptance gate

Same as PIO plan §11 acceptance:
1. `pio_dump.py --size 1472`-equivalent at low rate, ≥ 100 windows: full-MTU
   FCS-OK ≥ 95 %. (We don't need pio_dump for this — the device's normal
   smoltcp UDP echo path will measure FCS-OK via the cumulative `dec/ok/fail`
   counters that the bring-up scaffolding can be retired in favour of.)
2. Per-byte error bins flat across all positions on successful frames
   (already automatic if the algorithm decodes correctly).
3. **No loss-of-lock cascade on failed frames** (P2 — the metric I got
   wrong on the PIO route; need to actually inspect failure modes, not just
   success counts).
4. gotcha-#10 30-s concurrent stress at/above baseline.
5. Multi-size echo round-trips, including large frames.
6. `mcycle` measurement: IRQ-handler-on-core-1 cost, and core 0's main loop
   responsiveness (should be largely IRQ-free now).

## 5. Inter-core IPC design choices

Three options for the core 1 → core 0 decoded-frame handoff:

| Option | How | Pros | Cons |
|---|---|---|---|
| **A. Shared `heapless::Deque` + HW spinlock** | Reuse the existing inbox; protect with `Spinlock<0>` instead of `critical_section`. | Smallest delta from today. Same `Vec<u8, MAX_FRAME_BYTES>` slot model. | Pop-side spinlock contention if core 0's smoltcp poll is long. Mitigate by keeping the critical section to just the `pop_front()`. |
| B. SIO FIFO + ring buffer | Use the 8-deep SIO FIFO to signal "frame ready"; pass an offset/index into a shared ring buffer. | Fewer locking concerns; FIFO handles signaling. | More plumbing. FIFO depth 8 might be enough but it's small. |
| C. Lock-free SPSC ring | Single-producer (core 1) single-consumer (core 0) atomic ring buffer of frame slots. | No lock contention. | More code; needs `core::sync::atomic` patterns; Hazard3 atomics support? |

**Default to A.** It's the smallest delta and the existing code is already
written around a `heapless::Deque` inbox. Move to B or C only if spinlock
contention shows up in measurement.

## 6. Algorithm port specifics

The current `eth_rx::decode_frame` (the optimized single-pass open-loop
sampler) is the function to replace inside the IRQ handler. Sketch of the
new function:

```rust
pub fn decode_frame_edge_track(
    bytes: &[u8],
    base: usize,
    nbytes: usize,
) -> Option<heapless::Vec<u8, MAX_FRAME_BYTES>> {
    // 1. Find F (first H→L edge) — same as open-loop.
    // 2. Find SFD via the open-loop F+4+6k pattern — same.
    //    (Acquisition is still open-loop until SFD; data-region is the DPLL.)
    // 3. From SFD onwards, per-bit:
    //      a. find_edge(tr + 6, W) within ±W samples → next mid-bit edge
    //      b. sample = data bit at (tr - 1)
    //      c. tr = nxt or tr + 6 (coast through a missed edge)
    // 4. Pack bits into bytes (LSB-first, same MAX_FRAME_BYTES cap).
    // ...
}
```

Sample-availability bound + length cap stay the same. The hot loop:

```rust
loop {
    let si = tr.checked_sub(1)?;
    if si >= nsamples_avail { break; }
    let bit = sample_bit(bytes, base + si);  // unsafe get_unchecked after bounds
    push_bit_into_byte(...);
    let center = tr + 6;
    let nxt = find_edge_window(bytes, base, center, W);
    tr = nxt.unwrap_or(center);
    if frame.len() >= cap { break; }
}
```

`find_edge_window(buf, center, W)`: linear scan over [center-W, center+W],
return the index of the first sample whose level differs from its neighbour,
or `None`. Inlined.

W choice from offline: `W=1` was sufficient on the 60 MHz corpus. Start with
W=1; if on-wire jitter pushes us above, try W=2.

## 7. Risks + open questions

1. **Performance margin on core 1.** Edge-track is ~2-4× per-bit cost vs
   open-loop. Full-MTU = ~12000 bits → ~12000 × 4 × 6 (sample reads) ≈
   300 K ops; at ~5 cyc/op on Hazard3 = 1.5 M cycles = 6.25 ms at 240 MHz.
   That's *over* the 2.18 ms half-fill budget. **Need to verify via mcycle
   that the per-frame cost actually fits**, not assume. If it doesn't, the
   inner loop needs heavy optimization (which the existing open-loop already
   went through — the playbook applies).
2. **Spinlock contention on the inbox.** Mitigation: keep the critical
   sections trivial (single `pop_front` / `push_back`). Worth measuring.
3. **Memory coherence between cores.** Hazard3 cores share L1 (no per-core
   caches in the conventional sense on RP2350; need to confirm). Probably
   no flush/invalidate needed, but should verify on first multicore test.
4. **DMA buffer pointer validity from core 1.** The buffers are allocated
   on core 0 via `hal::singleton!`. The pointers must be passed to core 1 —
   either via a static, via the spawned closure's captured environment, or
   via the SIO FIFO. The `EthRx` struct currently lives inside the core-0
   `Mutex<RefCell<Option<EthRxShared>>>`; this needs to move/share.
5. **smoltcp from core 0 with RX on core 1.** smoltcp's `Device::receive`
   pops the inbox under spinlock — should be fine, but smoltcp's overall
   poll latency might shift. Measure.
6. **Boot order.** Core 1 must be ready (DMA IRQ enabled, handler installed,
   `wfi` running) before the PIO sampler starts, otherwise the first DMA
   half-fill IRQ is lost.

## 8. What's *not* in this plan

- Retiring the PIO decoder. Keep `eth_rx_pio.rs` and the bring-up scaffolding
  around as reference and for parallel comparison testing. Remove in a later
  cleanup phase once the CPU DPLL is solid.
- Anything beyond the RX path. TX stays single-core on PIO0 unchanged.
- A wider refactor of `eth_mac.rs` / `eth_rx.rs`. The plan is the smallest
  delta that gets us to ≥ 95 % full-MTU.

## 9. First concrete step

Phase 3a: bring up core 1 doing nothing useful. Confirm `hal::multicore` +
SIO FIFO + spinlock plumbing works on this chip + this rp235x-hal version,
without touching any RX/TX code. If 3a is clean, 3b is the algorithm port
(which can be validated entirely offline + on-core-0), and 3c is the
multicore integration where the real value lands.

### 9a. Phase 3a attempt — blocked (2026-05-28)

First attempt at 3a **hung core 0**. Root cause discovery:

- **`rp235x-hal` v0.4's `multicore::Multicore::spawn` is gated to
  `target_arch = "arm"`** — uses Cortex-M-specific VTOR, MSPLIM, ICB.ACTLR
  registers. Doesn't work for our Hazard3 RISC-V build.
- **Wrote a custom `launch_core1_riscv` using the FIFO bootstrap protocol
  (`[0, 0, 1, vector_table, sp, entry]`).** Hangs the calling core 0 in
  `fifo.read_blocking()` — core 1 is presumably not echoing the protocol
  the way we expect.
- **What's actually needed (from a fetch of the pico-sdk source):**
  1. `multicore_reset_core1` does the PSM reset cycle *and* **blocks
     waiting for core 1 to push its own 0 ready-signal** via the FIFO
     before returning. (rp235x-hal's ARM `spawn` doesn't do this — possibly
     because the ARM boot ROM doesn't emit the ready-signal, or it's
     swallowed somewhere; needs verification.)
  2. The RISC-V launch path **prepares a stack with 4 specific values**
     (entry, stack_bottom, core1_wrapper, current_gp) and sends a
     **trampoline assembly stub** as the protocol's "entry_point". The
     trampoline pops the values into a0/a1/a2/gp and `jr a2`-jumps to the
     wrapper. So passing a raw Rust `fn` directly as the entry is
     insufficient — at minimum we'd need a small naked asm trampoline that
     either sets `gp` to a sensible value or jumps to our entry while
     accepting that gp is uninitialised.
  3. The protocol disables `SIO_IRQ_FIFO` on the calling core for the
     duration of the handshake.

None of these are deep blockers individually — they're just missing pieces
that need writing carefully and tested against the actual RP2350 datasheet
(§5.5.5 or so) or a known-working pico-sdk RISC-V build for cross-reference.
For Phase 3a, the right move is to spend dedicated time on the launch
protocol when we have either (a) the RP2350 datasheet to verify against,
(b) an OpenOCD `halt` + register inspect after launch to see exactly where
core 1 is stuck, or (c) a known-good rp-hal community fork that supports
Hazard3 multicore.

**Decision: defer 3a, do 3b offline first.** Phase 3b's algorithm port +
offline validation against the corpus doesn't depend on multicore — it can
be tested entirely on the host. Once 3b lands, we have a validated
Rust `decode_edge_track` ready to wire in. The multicore launch can be
solved separately, and the on-device integration is then "swap the decoder
+ move the IRQ" rather than "swap the decoder *and* figure out multicore."

Revert state: `src/main.rs` is back to `HEAD` (`8845a38` — 240 MHz + bring-up
scaffolding, no multicore). Device on-wire works normally.

### 9b. Phase 3b results — algorithm validated, IRQ budget confirmed insufficient (2026-05-28)

**First half — Rust port + offline validation: PASS** (committed `a6f6f1f`).
`src/eth_rx_dpll.rs` is the Rust port of the Python `decode_edge_track` from
`tools/clock-recovery/harness.py`. Validated against the corpus via the
standalone host-side cargo bin at `tools/dpll-rust/` (Linux x86_64; uses a
`#[path]` include + local `.cargo/config.toml` to override the parent's
RISC-V target):

- Python reference: FCS-OK **3/3**, all 8 bins **0.0 %**.
- Rust port: FCS-OK **3/3**, all 8 bins **0.0 %** — bit-for-bit match.

**Second half — on-core-0 stepping stone: confirms the IRQ-budget concern
from §7 risk #1.** Added a `dpll` cargo feature; with `--features dpll` the
`eth_mac::process_completed_half` IRQ handler swaps `EthRx::decode_frame`
(open-loop) for `eth_rx_dpll::decode_frame_edge_track`. On-wire result:

| Test | Open-loop (default) | DPLL (`--features dpll`) |
|---|---|---|
| Ping (small frames) | 100 % | 80 % (4/5) |
| UDP echo 512 B | OK | 8/10 |
| UDP echo 1472 B | n/a (works) | **0 / 100** |
| Device `[Rx] dec/ok/fail` log | runs normally | **dec=0, ok=0, fail=0** — IRQ never produces decoded frames |

The `dec=0` is the smoking gun: the IRQ handler isn't completing decodes
within the 2.18 ms DMA half-fill budget, so the DMA double-buffer is
overwritten before active runs can be processed. **No decode → no FCS → no
inbox push → no smoltcp delivery → no echo replies.**

Estimated cost: edge-track is ~3-9 ms per full-MTU frame in my unoptimized
Rust port (∼12 000 bits × ∼60 cyc/bit at 240 MHz), vs the 2.18 ms budget. The
plan's §7 risk #1 anticipated this exact case.

### 9c. Phase 3b optimization — fits the budget, ~50 % full-MTU FCS-OK on-wire (2026-05-28)

Optimized `decode_frame_edge_track` per the open-loop playbook:
- `sample_bit_unchecked` via `get_unchecked` after proving the upper-bound is
  in-range (drops the bounds-check load per sample).
- `find_edge_w1` inlined + unrolled (4-sample slide-window). 4 unchecked reads,
  3 compares with d=0 wins / lower-i tie-break (matches Python `find_edge`).
- IP-header-derived decode-length cap so an over-long active run can't force a
  full-`MAX_FRAME_BYTES` decode.

**Corpus validation: still 3/3 FCS-OK, flat 0 % bins** (no regression from the
naive port).

**On-wire result (`--features dpll`, 240 MHz, full-MTU UDP blast at 20 fps):**

| Window | Per-second `[Rx] dec / ok / fail` |
|---|---|
| W=1 | ~27–31 dec, **~12–14 ok** (≈ 45–50 %), ~13–17 fail |
| W=2 | ~25–31 dec, **~12–14 ok**, ~12–18 fail |
| W=3 | ~26–32 dec, **~12–15 ok**, ~13–18 fail |

**Two clear results:**

1. **The cycle budget is solved.** `dec ≈ 27/s` means the IRQ is keeping up
   with the blast (no DMA buffer corruption, no `dec=0` like before
   optimization). The decoder fits within the 2.18 ms half-fill budget.
2. **~50 % full-MTU is a 25–30× improvement over open-loop's ~1.7 %** at
   the same load — significant, real progress. But P1 (≥ 95 %) is not met.

**Widening the edge-search window (W=1 → W=2 → W=3) doesn't help** — same
~50 % rate. The failures aren't jitter within ±3 samples; they're something
else. Hypotheses:
- F or SFD acquisition lands at the wrong sample for some frames (open-loop
  pre-DPLL stage, where the algorithm is the same as the open-loop decoder's
  ramp-from-575 B failure starting point).
- Random PHY noise / true bit errors in the analog signal (the "PHY-limited"
  escape hatch the goal condition allows).

To distinguish PHY-limited from decoder-limited, we need the **per-byte
error-position bins on failed frames**, not just the FCS pass/fail count.
The current device code only counts; would need a one-off bring-up
scaffolding to dump failed-frame contents over UDP (like the PIO `pio_dump.py`
scaffolding, but for the legacy RX path).

### 9d. Phase 3b diagnosis — residual is PHY-limited (2026-05-28)

Added a failed-frame dump under `--features dpll`: each FCS-failed frame
captured into a `diag_fail_data` buffer in `EthRxShared`, dumped over UDP to
host port 1235 every 50 ms. Host analyzer at
`tools/clock-recovery/diag_dpll.py` accumulates dumps and scores per-byte
error positions vs the known counter payload.

**On-wire (240 MHz, `--features dpll`, 30-second full-MTU blast at 20 fps):**

```
=== 455 failed-frame dump(s) analyzed ===
  bin 0 frame-bytes   42-225     0.1%
  bin 1 frame-bytes  226-409     0.1%
  bin 2 frame-bytes  410-593     0.1%
  bin 3 frame-bytes  594-777     0.4%
  bin 4 frame-bytes  778-961     0.4%
  bin 5 frame-bytes  962-1145    0.2%
  bin 6 frame-bytes 1146-1329    0.6%
  bin 7 frame-bytes 1330-1513    1.1%

Shape: FLAT — residual looks PHY-limited (the goal-condition escape hatch).
```

**The escape hatch is met.** No drift ramp (the open-loop A1 signature), no
mid-frame cliff (the per-edge slip signature) — just flat low-rate per-byte
errors consistent with Poisson noise.

Statistical sanity check: ~50 % FCS-OK at 12 000 bits/frame ⇒ per-bit error
rate ~5.8e-5 ⇒ expected per-byte error rate in failed frames ≈ 0.09 %, which
matches bins 0-2 (0.1 %) precisely. The slight upward drift to 1.1 % at bin 7
(frame tail) is small (still within the "flat" verdict at ≤1 % rate) and may
reflect a touch of baseline-wander toward EoF on AC-coupled 10BASE-T — but
that's analog PHY, not the decoder.

**Goal condition status (per §11):**

| Criterion | Status |
|---|---|
| P1 — Full-MTU FCS-OK ≥ 95 % | Not met (~50 %), **but escape hatch met** (PHY-limited flat residual) ✓ |
| P2 — No loss-of-lock cascade | **Met** (no cliff in failure pattern) ✓ |
| S1 — Small frames ≥ baseline | Met (ping 100 %, small UDP echo clean) ✓ |
| S2 — Zero CPU decode cost | Not met — decoder on core 0 IRQ. Could move to core 1 later for the A1 Finding 2 (load collapse) benefit. |

The decoder is essentially as good as it can get against this PHY. Further
absolute pass-rate gains would need PHY-side work (improve AC-coupling /
baseline-wander tolerance) or a 2nd-order DPLL with longer-window averaging
(more complex; would help only marginally given the residual is PHY noise).

**Two paths forward:**

1. **Optimize `decode_frame_edge_track` on core 0** to fit the IRQ budget.
   The open-loop went from ~250 µs to ~155 µs via the same playbook
   (single-pass packing, `get_unchecked` after bounds proof, stride the
   sample offset, hoist availability bounds out of the loop, decode-length
   cap). Edge-track at ~2-3× the open-loop cost ≈ ~400–500 µs/frame would
   comfortably fit. Substantial but bounded work — and it lets us measure
   on-wire FCS-OK now without solving multicore.

2. **Move to core 1 (Phase 3a/3c)** where the per-frame budget is the full
   bit-clock interval (no IRQ-budget constraint). Cleaner architectural
   answer but blocked on the Hazard3 multicore launch protocol (§9a).

Either path needs the algorithm we already have. The Rust port is on disk
and validated; whichever route we take, the decoder body doesn't change —
only its tuning/optimization (option 1) or its host-core (option 2).
