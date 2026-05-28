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
