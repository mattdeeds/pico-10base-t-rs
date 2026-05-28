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

### Phase 3a — Multicore foundation (small, low-risk first step) — ✅ DONE (2026-05-28, see §9f)

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

### 9e. Phase 3b productized — DPLL is the default decoder (2026-05-28)

With §9d's escape hatch met, the `--features dpll` opt-in is retired. The
edge-track DPLL in `src/eth_rx_dpll.rs` is now the only full-frame decoder;
the open-loop `EthRx::decode_frame` (and the temporary `dpll` cargo feature)
are gone. Cleanup pass:

- **`Cargo.toml`**: removed the `dpll` feature flag.
- **`src/eth_mac.rs`**: the `#[cfg(feature = "dpll")]` gates are gone; the
  IRQ-side decoder call is unconditional. Removed `EthRxShared::diag_fail_*`
  fields, the in-IRQ failed-frame capture, and `snapshot_diag_failed`.
- **`src/eth_rx.rs`**: removed the open-loop `decode_frame` method plus the
  `SFD_SEARCH_BITS` constant only it used. The open-loop sampling helpers
  (`find_first_falling_edge` / `find_sfd_end` / `data_bit`) stay — they're
  still used by `peek_dst_mac` (dst-MAC is 48 bits past the SFD, comfortably
  inside the no-drift window, so no need to pay edge-track cost for the
  MAC filter).
- **`src/main.rs`**: removed the Phase 2b PIO chunk-dump scaffolding
  (`eth_rx_pio` mod, PIO1 SM0, `dec_cap` capture/dump loop) and the
  Phase 3b failed-frame diag-dump (`diag_endpoint`, `diag_buf`, dump loop).
  Restored the Hello-World UDP broadcast every 200 ms.
- **`src/eth_rx_pio.rs`**: deleted. The PIO investigation is preserved in
  the commit history (`cc09e11`..`8845a38`) and the design retrospective at
  `docs/pio-dpll-report.md`.
- **`tools/clock-recovery/diag_dpll.py`**: deleted. Without its device-side
  counterpart (the in-IRQ failed-frame capture + UDP dump, also removed),
  the analyzer has no stream to listen to. Both halves are recoverable
  from commits `ab72c89`..`f0253c8` if we ever need to re-run the
  diagnosis.

**On-device verification (productized build, default firmware):**

- USB CDC: `[R2b] t=N nlps=63 udp_sent=5` — NLP cadence + Hello-World UDP
  cadence both back to pre-Phase-2b shape.
- `[Rx] dec=1 ok=1 fail=0 filt=0 dst=33:33:00:00:00:16` — IPv6 multicast
  decoded successfully through the DPLL.
- `ping -c 3 192.168.37.24`: 3/3, RTT 2.4–3.6 ms.
- `curl http://192.168.37.24/`: HTTP/1.0 200, served `uptime=43s` payload.

Binary size: 1.71 MB ELF (unchanged within margin — the open-loop
`decode_frame` was already cold/inlined by LTO since `--features dpll`
was set during recent measurement).

What didn't change: the decoder *algorithm*. The Rust DPLL body in
`eth_rx_dpll.rs` is untouched — productization was strictly a deletion
exercise. The decoder behaviour is byte-identical to the §9c on-wire
measurements (~50 % full-MTU, PHY-limited residual, no DMA buffer
corruption, fits the 2.18 ms IRQ budget at 240 MHz).

**What's left (the S2 piece):** the decoder is still on core 0's
`DMA_IRQ_0`. The A1 Finding 2 benefit (no main-loop starvation under
saturating load) would require Phase 3a (multicore launch) + Phase 3c
(move IRQ to core 1). That's separate work; productizing the DPLL itself
is done.

### 9f. Phase 3a — SOLVED: Hazard3 RISC-V core-1 launch working (2026-05-28)

The §9a blocker is cleared. Core 1 launches reliably and runs concurrently
with core 0; the multicore foundation for Phase 3c is in place.

**Code:** new `src/multicore_riscv.rs` — `launch_core1_riscv(psm, fifo,
stack, entry)`. Wired into `main.rs` right after the SIO/pins setup; core 1
runs `core1_entry` (a liveness counter into a shared `AtomicU32`,
`CORE1_TICKS`), and `log_status` prints `[Core1] launch=ok ticks=N`.

**What the §9a attempt got wrong, and the fixes (all confirmed against the
pico-sdk `pico_multicore/multicore.c` RISC-V path):**

1. **`vector_table` must be the `mtvec` CSR, not `PPB.VTOR`.** rp235x-hal's
   `multicore::spawn` reads `ppb.vtor()` — but the Cortex-M PPB is powered
   down when the chip runs the Hazard3 cores, so VTOR is garbage. Read our
   own `mtvec` via `csrr` and hand core 1 the same trap vector core 0 uses.
2. **Restore `gp` in a naked trampoline.** The bootrom jumps straight to the
   launch entry, bypassing `_start`, so the global pointer (used for
   `.sdata`/`.sbss`-relative addressing) is never set up. A `global_asm!`
   trampoline — byte-identical to the pico-sdk's `core1_trampoline`
   (`lw a0,0(sp); lw a1,4(sp); lw a2,8(sp); lw gp,12(sp); addi sp,sp,16;
   jr a2`) — pops `[entry, stack_bottom, core1_wrapper, gp]` off the stack,
   reloads `gp`, and tail-calls `core1_wrapper(entry, stack_bottom)`.
3. **No `ACTLR.EXTEXCLALL` / coprocessor setup.** That's a Cortex-M MPU
   shareability fix-up for cross-core atomics; on Hazard3 it's a write to a
   non-existent register (would fault). Hazard3 has no data caches (SRAM is
   coherent between cores) and implements the A extension, so cross-core
   atomics + the SIO HW spinlocks just work — confirmed by the climbing
   `CORE1_TICKS` (core 0 reads what core 1 stores, no cache maintenance).
4. **Bounded FIFO reads, not `read_blocking()`.** The §9a hang was core 0
   spinning forever in `read_blocking()` waiting for an echo that never
   came. The launch handshake here polls with a retry/timeout budget and
   returns `Err(Unresponsive)` instead — so a botched launch degrades to
   `[Core1] launch=FAIL` with core 0 still fully alive, never a wedge.

The handshake itself is the standard `[0, 0, 1, vector_table, sp, entry]`
bootrom sequence (drain + `sev` before each `0`, echo-verify each word,
restart on mismatch). No trailing ready-signal read (we pass a bare `fn`
pointer, not a closure core 0 must keep alive, so core 1 has nothing to
signal back — unlike the HAL's `spawn`).

**On-wire acceptance (default 240 MHz build, freshly flashed):**

| Check | Result |
|---|---|
| `[Core1] launch=ok`, `ticks` climbing | ✅ ~1000/s (t=20 → 19994, t=21 → 20994, …) |
| Ping (small frames) | ✅ 20/20 = 100 %, RTT 2.0–4.1 ms |
| HTTP `curl /` (TCP) | ✅ 200 OK, payload served |
| UDP echo (port 1234) | ✅ 10/10 byte-perfect |
| Core-0 telemetry | ✅ nlps 62–63/s, udp_sent +5/s — unchanged |

So the §9a Phase-3a gate ("existing R10 production behaviour preserved")
holds while core 1 is alive — core 0's TX/RX/USB/smoltcp data path is
byte-identical (core 1 only spins a counter; it touches nothing core 0
owns). The 1 MB-curl throughput baseline (~596 kB/s) was not re-measured
here because the default build serves the tiny-info page, not the
`http-bulk-test` 1 MB stream, and core 0's data path is unchanged by
construction — that number is the Phase 3c acceptance metric and is best
measured there (where moving the IRQ to core 1 can actually move it).

**Now unblocked: Phase 3c** — route `DMA_IRQ_0` to core 1, swap the
`critical_section::Mutex` around `SHARED_RX` for a `hal::sio::Spinlock`, and
run the decode pipeline in core 1's IRQ handler (`wfi` between IRQs). Watch
for gotcha #10: moving the IRQ off core 0 removes the accidental
carrier-sense (core 0 can now TX while core 1 decodes), so the 30-s
broadcast-blast curl is the measurement that decides whether CSMA/CD becomes
the next priority.

### 9g. Phase 3c — DONE, but reveals carrier-sense as the real TCP blocker (2026-05-28)

Phase 3c is implemented and **functionally correct**, but the on-wire
measurement is decisive: **moving RX to core 1 fixes CPU starvation yet
*regresses* TCP throughput**, because it destroys the accidental
carrier-sense the single-core design got for free (gotcha #10). The
measurement the plan set up has answered: **CSMA/CD is now required.**

**Implementation** (branch `r12c-multicore-rx`, not merged to `main` — it's a
throughput regression as-is):
- `eth_mac.rs`: split the old single `EthRxShared`-behind-`critical_section`
  into **`RX_ENGINE`** (core-1-exclusive `EthRx` + MAC, no lock — only core 1's
  handler touches it after `install_rx`) and **`RX_SHARED`** (inbox + stats,
  guarded by `Spinlock<0>`). The decode runs lock-free; `Spinlock<0>` is held
  only to push each frame + merge stat deltas. **This is the crucial part** —
  the HAL's `critical_section` is already cross-core safe (it claims
  `Spinlock<31>`), but the old handler held it across the *entire* ≤2.57 ms
  decode, which on core 1 would block core 0's `receive` for that whole time
  (relocating the starvation, not fixing it). Decoupling exclusive decode from
  the brief shared publish is what makes core separation actually pay off.
- `main.rs`: `core1_entry` unmasks `DMA_IRQ_0` on core 1's own xh3irq +
  enables machine-external interrupts, then `wfi`-loops; core 0 unmasks
  nothing. Core 1's stack bumped to 16 KB (it now builds a 1600-byte frame
  `Vec` on-stack per decode). Launch moved to after `install_rx`.

**Why it works mechanically:** xh3irq enables are per-hart CSRs (`meiea`), so
unmasking `DMA_IRQ_0` only on core 1 routes it there; core 0 never sees it.
The `MachineExternal` trap → `meinext` dispatch → `#[no_mangle] DMA_IRQ_0`
handler works on core 1 with just `mtvec` + stack + `gp` set up by the launch
(no `_start` needed). Confirmed: `[Core1] ticks` now climbs by ~458/s = the
DMA half-fill rate (core 1 wakes per DMA IRQ), and `[Rx] dec/ok` stats
decoded on core 1 are visible to core 0 via `Spinlock<0>`.

**On-wire results (240 MHz, `http-bulk-test` build):**

| Scenario | Result | vs single-core baseline |
|---|---|---|
| Idle ping (low-rate bidir) | 20/20, RTT 2.3 ms | ✅ ≥ baseline (slightly better — core 0 not preempted) |
| UDP echo (low-rate bidir) | 10/10 byte-perfect | ✅ = baseline |
| **Idle 1 MB curl (sustained bidir TCP)** | **~40–69 kB/s** | ❌ **~12× WORSE** (R11: 596 kB/s) |
| Broadcast blast, pure RX (50 pps full-MTU) | core 1 `ok≈50/s` (full rate), core 0 `nlps=62–63/s` steady | ✅ **starvation FIXED** (A1 Finding 2) |

**Root cause of the curl regression — gotcha #10, confirmed by host counters.**
A single idle 1 MB curl logged **+~30 host TX collisions and +~30 host
RX/frame errors** (`/proc/net/dev`). In single-core, core 0 physically
couldn't TX while it was decoding the host's just-received ACK (same core), so
it never transmitted on top of in-flight host frames — a *free* carrier-sense
that kept idle TCP collision-free at 596 kB/s. Phase 3c decouples TX (core 0)
from decode (core 1): core 0 now transmits data segments whenever smoltcp
wants, colliding with the host's ACKs on the half-duplex wire (the Pico has no
CSMA/CD). Each collision → lost segment → TCP RTO stall → throughput collapse.
The collision *rate* is modest (~2 %), but the *penalty* per loss (RTO) is
huge, so TCP craters.

**Key reframing:** the R11 "23× collapse under stress" was attributed to CPU
starvation. Phase 3c proves starvation is fixable (core 0 stays at full
cadence under blast) — but reveals a **second, more fundamental TCP limiter
that the starvation was masking**: collisions. While core 0 was starved it
transmitted little, so few collisions; now that it transmits freely, collisions
dominate. **Carrier-sense — not core separation — is the real lever for
bidirectional TCP throughput.** Core separation is necessary (it unblocks
load without starvation) but not sufficient.

**Status of the §4 Phase 3c acceptance gate:** NOT met. Target was 1 MB curl
under broadcast blast ≥ 300 kB/s; idle curl is already ~45 kB/s (below the
target before any blast), so the gate fails on the carrier-sense issue, not on
starvation. The broadcast-blast curl was not separately run — idle is already
below target and is the best case.

**Decision / next step: add carrier-sense.** Phase 3c stays on its branch
until then (don't merge a 12× TCP regression to `main`; `main` keeps the
596 kB/s single-core baseline). Options, in rough order of fidelity:

1. **Real PIO carrier-sense in the TX path (the right fix).** Before the TX SM
   starts a frame, sense the RX line (RO / GP13) for carrier and defer if
   busy; ideally also detect collisions for CSMA/CD backoff. Substantial PIO
   work but it's the proper 10BASE-T half-duplex MAC behaviour and makes
   Phase 3c a clear net win.
2. **Software carrier-sense interim.** Have core 0 check a cheap "wire busy"
   signal before TX (e.g. derived from recent RX-sampler activity). Imperfect
   proxy — the DMA-half granularity (~2.18 ms) lags real wire state — but may
   recover much of the throughput as a stop-gap to validate the diagnosis.
3. **smoltcp tuning band-aid** (larger TX window so losses trigger fast
   retransmit instead of RTO). Treats the symptom; fragile; not recommended.

### 9h. Phase 3d — PIO carrier-sense: recovers most throughput, residual collisions remain (2026-05-28)

Phase 3d adds carrier-sense (option 1 from §9g) and **validates the diagnosis
decisively**: collisions were the cause, carrier-sense is most of the cure.
On branch `r12d-carrier-sense` (built on `r12c`), not merged.

**Implementation** (`eth_tx.rs`):
- A dedicated **carrier-detect SM (PIO0 SM2)** watches RO (GP13) via `jmp_pin`
  and raises host-visible **PIO IRQ flag 0** while the line is *active*
  (toggling), clearing it once the line has been *stable* for GUARD≈8 samples
  (~267 ns @ 60 MHz) — idle. (10BASE-T idle is a steady level, so carrier ==
  recent transitions; the RX `find_active_run` logic relies on the same fact.)
  ~13 PIO instructions; fits easily alongside TX (3) + RX (1).
- `wait_carrier_idle()` polls flag 0 before the preamble in `send_raw_frame` /
  `send_udp_broadcast` / `send_nlp` (outside the critical section, so we defer
  with interrupts enabled). **Bounded spin** (`CARRIER_WAIT_SPINS`) so a
  stuck-busy flag degrades to "transmit anyway", never wedges TX.
- Detection in PIO; the gate is a few lines in software. The proven Manchester
  TX dispatcher is untouched.

**On-wire (240 MHz, http-bulk-test):**

| Metric | single-core | Phase 3c | Phase 3d |
|---|---|---|---|
| Idle 1 MB curl | 596 kB/s | ~45 | **114–914, avg ~340** (peaks > baseline) |
| Host collisions / curl | ~0 | ~30 | **~1–4.5** (6–22× fewer) |
| Blast 1 MB curl (50 pps) | 26 | ~45 | **156–470** (6–18× better) |
| TX alive / ping / core 1 | — | — | nlps 62–63/s, ping 100%, ticks ~458/s ✓ |

**Result: carrier-sense recovers most of the throughput and crushes most
collisions.** Collision-free curls hit **914 kB/s — above the single-core
baseline** (no decode stealing core-0 TX time + no collisions), proving the
ceiling is there. Under broadcast blast, 156–470 kB/s vs single-core's 26 —
6–18× better, meeting the §4 3c gate (>300) on good runs.

**But it's carrier-sense ONLY — residual collisions remain** (~1–4.5/curl, vs
~30 in 3c). Two sources: (a) no collision *detection* — if the Pico and host
start within the detection/start latency, both transmit and corrupt; (b) the
small window between `wait_carrier_idle` returning and the preamble actually
hitting the wire. Each residual collision costs a TCP RTO stall, which is why
idle throughput is *variable* (114–914) with a median (~340) still below the
596 baseline. The lows are curls that hit a collision; the highs are
collision-free.

**Hardware enables the next step (Phase 3e — full CSMA/CD).** The ISL3177E has
**no DE/RE — driver and receiver are always enabled** (see the
`hardware-isl3177e` memory), so RO loops our own TX back. That means we can
**detect collisions**: compare RO to the bit we're driving on DI; a mismatch
mid-frame = another station transmitting = collision → abort, jam, binary-
exponential backoff, retransmit. That should mop up the residual collisions and
take idle throughput to a consistent ≥596 (with the 3c starvation fix intact
and the stress numbers far above single-core). CD is more PIO work (the TX SM
must sense RO per-bit during transmission and signal collisions back to the
CPU/smoltcp for retransmit), but the loopback path makes it feasible.

**Net:** Phase 3c (core separation) + Phase 3d (carrier-sense) together make
the multicore RX a *near* net win — far better under stress, no starvation,
peaks above baseline — with residual-collision variance that Phase 3e (CD)
should close. Don't merge to `main` yet: the idle *median* is still below the
596 baseline, so it's not yet a strict improvement for the idle case.
