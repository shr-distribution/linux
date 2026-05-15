# Gemini consultation: A22X period-16 — round 4, packet-source bisect complete

**Continuation of:** `gemini-summary-2026-05-13-round3.md`
**Date:** 2026-05-13

## TL;DR

We've now bisected the 0x0ee2 advance source down to **`CP_DRAW_INDX` execution itself**. No prepended, appended, or substituted event packet affects the +0x00020200/render advance. Direct register writes, RBBM_SOFT_RESET, and TC_CNTL_STATUS L2_INVALIDATE all fail to reset it. The cycle is **intrinsic to the draws Mesa emits** — and Mesa always emits exactly 2 draws/render on A22X (1 user triangle + 1 RECTLIST resolve quad), matching the observed +2 slot advance.

## What changed since round 3

We built a 3-way env-gated bisect (Mesa patch 0015):

**1. `FD2_NO_CACHE_FLUSH_INV=1` — skip both existing `CACHE_FLUSH_AND_INV_EVENT` (0x16) emissions** (one at start of `fd2_emit_restore`, one at per-tile renderprep).
- Result: SAFE. Per-cap advance UNCHANGED at +0x00020200.
- → Event 0x16 is NOT the source of advance. Patch 0046's cache flushes are decorative for the cycle.

**2. `FD2_START_NOP_COUNT=14` — emit 14 extra `CP_NOP` at start of `fd2_emit_restore`** (right after the existing WAIT_FOR_IDLE).
- Result: HANGS GPU on the first submit. CP firmware doesn't tolerate extra NOPs in that position.
- → Per-packet-count is not the driver; OR position-sensitive cmdstream prologue is fragile.

**3. `FD2_END_CACHE_FLUSH_INV_COUNT=14` — emit 14 extra `CACHE_FLUSH_AND_INV_EVENT` (0x16) at end of tile loop**.
- Result: SAFE. Per-cap advance UNCHANGED at +0x00020200. Cycle still period-16.
- → Adding event 0x16 N times at end has zero effect on cycle counter (consistent with (1)).

## Complete falsification table

| Test | Effect on 0x0ee2 advance | Stable? |
|------|--------------------------|---------|
| `FD2_NO_CACHE_FLUSH_INV=1` (remove existing 0x16) | none | yes |
| `FD2_END_CACHE_FLUSH_INV_COUNT=N` (add 0x16 at end) | none | yes |
| `FD2_END_CTX_DONE=1` (event 0x05 at end) | none | yes |
| `FD2_END_FACENESS=1` (event 0x1c at end) | none | yes |
| `FD2_END_DEALLOC=N` (events 0x00+0x01, N=0..8 pairs) | none | yes |
| `FD2_START_NOP_COUNT=14` (CP_NOP at start) | n/a | **HANG** |
| Direct MMIO write to 0x0ee2 | silently ignored | yes |
| RBBM_SOFT_RESET=0xffffffff | none | yes |
| TC_CNTL_STATUS bit 0 (L2_INVALIDATE) | none | yes |

## Strong conclusion

**0x0ee2 is advanced by `CP_DRAW_INDX` execution.**  Mesa emits 2 draws/render = +0x00020200 advance/render → 16 distinct slot states after 16 renders → wraps mod-16.  We cannot:
- Skip the draws (rendering would not happen)
- Reset the slot pointer from CPU (RO register, no reset bit found)
- Cancel the slot consumption via deallocation events (none of {0x00, 0x01, 0x05, 0x1c} work)
- Add filler packets (CP_NOPs at start hang the GPU)

## The HTC / Samsung / Xiaomi mystery

Vendor libGLESv2 binaries for HTC, Samsung, and Xiaomi (all Adreno 220-class) emit **ZERO `CP_EVENT_WRITE` packets** yet produce correct output.  They must structure their cmdstream very differently from Mesa.  Hypothesis: they either
- Use a single mega-IB with no `CP_INDIRECT_BUFFER` calls (everything inlined), or
- Use a different submit topology (one submit per tile, or one submit covering 16 renders, etc.), or  
- Use a non-standard `DRAW` packet that doesn't advance the slot allocator.

webOS DOES emit events (0x06 CACHE_FLUSH, 0x17, 0x18, 0x1c FACENESS_FLUSH) but doesn't use the period-16-cycle-prone path that Mesa takes — webOS's strace showed 16 ioctls/render, while Mesa does 1.

## Concrete questions for round 4

### 1. The "no-advance" draw packet variant

On A2XX, are there alternative draw packets that don't consume slot allocations?  Specifically:
- **`CP_DRAW_INDX_BIN` (binning-path draw)** vs `CP_DRAW_INDX` — does one advance slots and the other not?  Mesa's binning IB on A20X uses CP_DRAW_INDX_BIN; we don't currently use it for A22X.
- **`CP_DRAW_INDX_2`** (if it exists on A2XX) — same draw but with different slot semantics?
- **`CP_SET_DRAW_INIT_FLAGS`** + restricted draw — can we set a flag that disables slot tracking for the duration?
- **Mesa emits `RECTLIST` (primtype 8?) for the resolve quad.**  RECTLIST is a privileged primtype on Adreno used for tile resolves.  Could the RECTLIST draw NOT consume slots while the regular `TRIANGLES`-prim user draw does?  That would imply only 1 of our 2 draws advances 0x0ee2 and we're at +1 not +2 per render.  But the data shows 16/cap, not 8/cap, so both draws advance.

### 2. Per-tile split — would it help?

Currently Mesa builds 1 ringbuffer covering: binning IB + per-tile (state setup + user draws + resolve quad) for all tiles + tile_store IB, then submits the whole thing once.  Single MSM_SUBMIT per render.

If we split each tile into its own MSM_SUBMIT, we'd have N submissions per render (where N = number of tiles).  For a small framebuffer this is maybe 1 tile = 1 submit (same as now).  For larger framebuffers, more submits, more draws.  Each tile = 2 draws (user + resolve) = +2 slots.  Multi-tile renders would still hit period-16, just at a different rate.

Does per-tile split help on A22X?  Or is the underlying issue that **Mesa always renders 16 frames into 16 different slots** — and the fix is to keep all 16 slot states valid?  In which case the right fix is patch 0070's "8x dummy POINT draws to scrub SQ slots" extended to **16x and run only once at GPU power-up**, not per-render.

### 3. The 14-NOP hang — what does this tell us?

`FD2_START_NOP_COUNT=14` injected 14 `CP_NOP` (header + 1 zero payload, 28 dwords total) right after the existing `CP_WAIT_FOR_IDLE` in `fd2_emit_restore`.  The CP hangs immediately on submit.  What's the failure mode?  
- The state initialization that follows the NOPs is timing-dependent and the extra NOPs broke it?
- The CP firmware has a max NOP-stream length before it loses track?
- The shadow register state load (which happens around this position) doesn't tolerate NOPs?

Knowing this might unlock a "less invasive" position to inject events.

### 4. Decompiling vendor binaries deeper

We checked vendor libGLESv2 for `CP_EVENT_WRITE` (0xC0004600 byte sequence) and got zero hits in HTC/Samsung/Xiaomi.  What ELSE should we grep for?  Specifically, what byte patterns would identify:
- `CP_DRAW_INDX_BIN` (different header) vs regular `CP_DRAW_INDX`?
- `CP_SET_BIN_BASE_OFFSET` (binning setup)?
- `CP_LOAD_CONSTANT_CONTEXT` (state load that might "tag" slots)?
- `CP_WAIT_FOR_IDLE` (the only known sync packet)?

If we can fingerprint each vendor's draw structure, we'd know whether they use mega-IB, per-tile submits, or some other topology.

### 5. Final-ditch: pin via SQ wavefront slot scrub at boot-only

Patch 0070 emits "8x dummy POINT draws" in `fd2_clear` to scrub SQ slots.  This runs per batch.  What if instead we scrub 16x and only at GPU power-up (kernel hw_init)?  That way every slot gets initialized once, and the period-16 cycle no longer matters because all 16 slots have identical state.

The "5adc3160 is the only correct frame" symptom suggests only slot 0 is initialized.  If we initialize all 16, all renders should be correct (just using different but equivalently-initialized slots).

Is this the right model?

## Repo references

- Mesa patch 0014: `fd2_emit_tile_fini` with end-of-tile-loop event emission (CONTEXT_DONE, FACENESS_FLUSH, VS/PS_DEALLOC×N)
- Mesa patch 0015: 3-way env-gated bisect (FD2_NO_CACHE_FLUSH_INV, FD2_START_NOP_COUNT, FD2_END_CACHE_FLUSH_INV_COUNT)
- Mesa patch 0070: "8x dummy POINT draws" SQ slot scrub in `fd2_clear` (could be retargeted to boot-time-only 16x)
- Kernel debugfs `/sys/kernel/debug/dri/0/regrw_value` (kernel patch a9668aedb53b) makes 0x0ee2 writable for testing — write is silently ignored, confirming RO

## What we DON'T need

- More event-emission variants — we've tested 5 different events at every reasonable position
- More register-write attempts — every reset register we know about has been tried
- More multi-flush variants — falsified across 5 rounds

We need **either** (a) the name of the A2XX register or PM4 packet field that controls slot tracking, **or** (b) a concrete pointer to how vendor drivers avoid the cycle (cmdstream structure / draw packet variant).
