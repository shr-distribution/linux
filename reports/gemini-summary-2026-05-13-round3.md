# Gemini consultation: A22X period-16 — round 3, post-event-emission tests

**Continuation of:** `gemini-summary-2026-05-13-cycle-register.md`
**Date:** 2026-05-13

## TL;DR

Implemented your round-2 suggestion: a new `fd2_emit_tile_fini` for A22X that conditionally emits `CONTEXT_DONE` (0x05), `FACENESS_FLUSH` (0x1c), and `VS_DEALLOC + PS_DEALLOC` (0x00 + 0x01) at end of tile loop. Tested all combinations from a fresh boot, with `0x0ee2` snapshot per cap. **None of them pin the cycle.** But the test was extremely informative.

## Results (5 caps per phase, FD2_FLUSH_MULTIPLIER=1, fresh kernel boot)

```
boot 0x0ee2 = 0x15000002

=== Phase A_baseline (no env vars) ===
  cap 1: hash=5adc316085  0x0ee2=0x15020202   <- CORRECT FRAME
  cap 2: hash=a3a0e05ec9  0x0ee2=0x15040402
  cap 3: hash=1b4ff379d4  0x0ee2=0x15060602
  cap 4: hash=b06f86c019  0x0ee2=0x15080802
  cap 5: hash=5066066706  0x0ee2=0x1501c1c2   <- byte0 wraps

=== Phase B_CTX_DONE (FD2_END_CTX_DONE=1, event 0x05) ===
  cap 1-5: 0x0ee2 continues 0x1503c3c2 ... 0x15038382
  NO behavioural change vs baseline (advance rate identical,
  per-cap continuation of the same 16-cycle).

=== Phase C_FACENESS (FD2_END_FACENESS=1, event 0x1c) ===
  cap 1: hash=6f744c5c21  0x0ee2=0x15058582
  cap 2: hash=869e023fdc  0x0ee2=0x15078782
  cap 3: hash=a29232c844  0x0ee2=0x15014142   <- byte0=42 (new!)
  cap 4: hash=a2b2b7c054  0x0ee2=0x15034342
  cap 5: hash=a93284ac91  0x0ee2=0x15054542
  Byte 0 took values 82, 82, 42, 42, 42 (baseline: c2, c2, c2, c2, 82).
  FACENESS_FLUSH shifts byte 0 to "earlier" state of the inner counter.

=== Phase D_DEALLOC (FD2_END_DEALLOC=1, events 0x00 + 0x01) ===
  cap 1: hash=fd260df660  0x0ee2=0x15074742   <- inherited carryover from C
  cap 2: hash=5adc316085  0x0ee2=0x15010102   <- CORRECT FRAME at cap 2!
  cap 3: hash=a3a0e05ec9  0x0ee2=0x15030302
  cap 4: hash=1b4ff379d4  0x0ee2=0x15050502
  cap 5: hash=b06f86c019  0x0ee2=0x15070702
  Caps 2..5 hashes IDENTICAL to baseline caps 1..4!
  Cycle phase SHIFTED by 1 cap. Byte 1/2 starts at 01 not 02
  (half-step offset).

=== Phase E_ALL (all three events combined) ===
  Similar to FACENESS alone - no improvement from combining.
```

## What we learned

1. **CONTEXT_DONE (0x05) is a no-op on A2XX.**  Adding it has zero effect on 0x0ee2.  Either this event opcode is unimplemented on Adreno 220 or it gates on hardware we're not using.

2. **FACENESS_FLUSH (0x1c) modifies the cycle parameterization but doesn't pin.**  Byte 0 of 0x0ee2 takes different values (82, 42 — values not seen in baseline) but the per-cap advance rate `+0x00020200` is unchanged.

3. **VS_DEALLOC + PS_DEALLOC (0x00 + 0x01) causes a phase shift, not a pin.**  After DEALLOC, the cycle continues at the same +0x00020200/cap rate, but starting from a half-step offset (byte 1/2 = 01 instead of 02).  Importantly, the *hash sequence* is identical to baseline shifted by 1 — so the underlying GPU state for each phase is the same, only the *entry point* into the cycle changes.

4. **Per-cap +0x00020200 advance is INVARIANT across all event combinations.**  This is the key new finding.  Whatever HW operation advances 0x0ee2 by this amount per render, it's **not** any of {0x00, 0x01, 0x05, 0x1c} events.

## Concrete questions for round 3

### 1. What advances 0x0ee2 by +0x00020200 per render?

Given:
- Per-cap advance is `0x00020200` (= bytes [1]+=2, [2]+=2, [0] unchanged within a 4-cap block)
- Boot starts at `0x15000002`, first render leaves it at `0x15020202`
- This +0x00020200 happens regardless of which extra events we append
- Adding more `CP_EVENT_WRITE` packets at end doesn't add to the increment

What single operation in a Mesa A22X render flow contributes this delta?  Candidates we can think of:
- The `CP_DRAW_INDX` of the resolve quad (RECTLIST that copies GMEM→sysmem) — emitted once per tile
- The per-tile `fd2_emit_tile_renderprep` register-write block
- `CACHE_FLUSH_AND_INV_EVENT` (event 0x16) which Mesa already emits — *could this be the actual advancer?*
- Vertex shader binding (`SQ_PROGRAM_CNTL` write)

If you had to bet on ONE PM4 packet type that's most likely advancing 0x0ee2 by ~0x00020200, what would it be?

### 2. Is byte 0 (currently `02, c2, 82, 42, 02, ...` block-state) a separate state machine?

Within each "byte-0 block" (4 caps), bytes 1-2 advance linearly.  Then byte 0 changes and bytes 1-2 reset.  This looks like a **2-level state machine**:
- inner counter: bytes 1-2, period 4
- outer counter: byte 0, period 4
- combined period: 16

Does this match any A2XX architectural model you've seen — e.g., **4 wavefronts × 4 contexts per wavefront**, or **4 tile-bin partitions × 4 visibility slots**?

### 3. The shift, not pin, pattern from DEALLOC

DEALLOC produced an interesting half-step shift (byte 1/2 starting at 01 instead of 02).  This implies VS_DEALLOC + PS_DEALLOC contribute **0x00010100** (half) of the per-cap advance, while the rest of the render contributes the other 0x00010100.

If true, then 0x0ee2's increment is composed of multiple sub-deltas from different GPU stages.  Each stage contributes a fraction.  Adding DEALLOC events means: render emits its 0x00010100 + DEALLOC emits 0x00010100 (or maybe -0x00010100 cancelling something).

**Does this composite-delta model make sense for A2XX?**  And if so, what fraction does `CACHE_FLUSH_AND_INV_EVENT` (0x16) contribute?  Should we *remove* the existing 0x16 from `fd2_emit_restore` (patch 0046) and see what happens?

### 4. We may have been chasing the wrong reset target

What if 0x0ee2 isn't a "state machine to reset" but a **literal monotonic event counter that wraps mod 16**, and the slot-state confusion is happening *because of* its current value being used as a lookup key?  In that model, the fix isn't to reset the counter but to ensure the slot pointed-to by every counter value is initialized consistently.

If so, the patch 0070 "8x dummy POINT draws" SQ slot scrub should be extended to 16x — and should be done from the BOOT path, not per-render.  Have we conclusively ruled this out?  (We tested 8x and got the cycle, but never tried 16x.)

### 5. What does webOS actually do?

We surveyed `CP_EVENT_WRITE` packets and only found 5 sites in webOS libGLESv2.  But Mesa emits other packet types too (`CP_SET_CONSTANT`, `CP_DRAW_INDX`, `CP_INDIRECT_BUFFER`, `CP_REG_RMW`, `CP_LOAD_CONSTANT_CONTEXT`).  Where in webOS would you look for the "magic" that prevents the cycle?  Specifically: are there CP_REG_RMW or CP_LOAD_CONSTANT_CONTEXT calls that touch a different register near 0x0ee0-0x0ee2 that we should mimic?

## Repo references

- `meta-smartphone/meta-mainline/recipes-graphics/mesa/files/0014-freedreno-a2xx-emit-end-of-tile-loop-CP_EVENT_WRITE-.patch` (the test patch)
- `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c::fd2_emit_tile_fini` (where events are emitted)

## What would conclusively answer this

If Gemini can pinpoint **which PM4 packet** advances 0x0ee2 by `0x00020200`, we can:
- emit that packet 15 more times per render to wrap the counter mod-16 → all renders land at same phase
- OR find the packet's "no-advance" alternative form

This is now the single most decisive question.
