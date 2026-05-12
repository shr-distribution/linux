# v8 FD2_PIN_BIN_ID sweep — SQ-slot-via-bin_id hypothesis FALSIFIED

**Date:** 2026-05-12
**Mesa libgallium md5:** `b725f49bd54612a3db478dfcb77d03b2`
**Kernel:** `6.18.0-luneos-gb6dc680b4167`
**Test:** Run 5 caps per PIN value 0..7 with `FD2_PIN_BIN_ID=<n>` pinning `VGT_CURRENT_BIN_ID_MIN/MAX` to a constant `<n>` for all tiles.

## Hypothesis being tested

The visual analysis of v6 T1 default suggested the period-N cycle was per-tile vertex-attribute corruption, with each tile picking a different SQ wavefront slot based on `VGT_CURRENT_BIN_ID` (which Mesa emits as `tile->n`, cycling 0..N-1).  If pinning all tiles to the SAME `bin_id` value made them all pick the same SQ slot, the cycle should collapse:

- Collapse to clean correct render (5adc3160 x N) → slot hypothesis CONFIRMED, slot had correct state
- Collapse to single wrong/blank → bin_id IS the selector but no slot has correct state
- No collapse (cycle persists) → hypothesis FALSIFIED, slot selector is something else

## Result: NO COLLAPSE — hypothesis FALSIFIED

All 8 PIN values produced different hashes per rep.  Even pinning ALL tiles to the same bin_id, each successive cap produced a different hash.

| PIN | rep1 | rep2 | rep3 | rep4 | rep5 |
|---|---|---|---|---|---|
| 0 | 5adc3160 ✓ | 4477e60a | bebc09c6 | 3625b67f | 03dee03a |
| 1 | 9fb336c8 | e93d10aa | 202cfe9f | e21b9529 | 3c9c950b |
| 2 | ab2c6c00 | 6c10867f | 0cbed90a | c60a6009 | 5908eca8 |
| 3 | 7325d713 | 5adc3160 ✓ | 4477e60a | bebc09c6 | 3625b67f |
| 4 | 03dee03a | 9fb336c8 | e93d10aa | 202cfe9f | e21b9529 |
| 5 | 3c9c950b | ab2c6c00 | 6c10867f | 0cbed90a | c60a6009 |
| 6 | 5908eca8 | 7325d713 | 5adc3160 ✓ | 4477e60a | bebc09c6 |
| 7 | 3625b67f | 03dee03a | 9fb336c8 | e93d10aa | 202cfe9f |

## What we actually learned: cycle is period-16, per-submit

Concatenating all 40 hashes in order reveals a **deterministic period-16 cycle** through these unique hashes (in submit order from PIN=0 rep1):

```
0: 5adc3160 ✓ CORRECT (RGB triangle)
1: 4477e60a
2: bebc09c6
3: 3625b67f
4: 03dee03a
5: 9fb336c8
6: e93d10aa
7: 202cfe9f
8: e21b9529
9: 3c9c950b
10: ab2c6c00  (new, never seen before this run)
11: 6c10867f  (new)
12: 0cbed90a  (new)
13: c60a6009  (new)
14: 5908eca8  (new)
15: 7325d713  (was singleton in earlier runs, not part of 8-cycle)
```

Each successive cap (same PIN, same env) advances by +1 mod 16.  Changing PIN by +1 advances by +5 mod 16 (so PIN=0 phase=0, PIN=1 phase=5, PIN=2 phase=10, PIN=3 phase=15, PIN=4 phase=4, ...).  PIN value affects the *initial* phase but not the increment-per-submit.

**The cycle is per-submit, NOT per-tile.** The per-tile mosaic we see in any single cap is consistent *within* that submit — what cycles is the entire image across submits.  Previous "8-cycle" reports were sampling 8 of the 16 phases.

## Implications

- `VGT_CURRENT_BIN_ID` is not the SQ slot selector for the cycle.
- The cycle counter is some other GPU internal state that advances by 1 per submit.
- The shipping SQ-slot scrub (patch 0070) which touches 8 slots is insufficient — period is 16, not 8.
- Pinning bin_id only changes the *starting offset* of the cycle — not useful as a fix on its own (the cycle would just start at a different "lucky" phase).

## What might still work

1. **Find what advances per submit.**  Look for GPU registers that increment with each draw / each EVENT_WRITE / each cmdstream submission and check if their value mod 16 correlates with the rendered hash.
2. **16-slot scrub** instead of 8.  Extend patch 0070 to 16 dummy POINT draws if there's an internal state pool of 16.
3. **Reset state per submit.**  Find a CACHE_FLUSH or context-invalidation event that webOS emits per draw which we're not emitting.
4. **Look at submit-counter registers**: VGT_DRAW_INITIATOR, VGT_NUM_INSTANCES, CP_PROG_COUNTER, CP_PERF_COUNTERS — any of which might be the cycle source.

## Files

| file | meaning |
|---|---|
| `pin<N>_<hash>.png` | first cap at FD2_PIN_BIN_ID=N (8 distinct starting phases) |
| `pin<N>_<hash>.bin` | raw RGBA8 1024x768 |
