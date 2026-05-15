# Update 10 for Gemini: cycle decomposes into spatial × color, MMU-recovery sync was masking everything

## Critical correction from update 9

I told you in update 9 that glmark2 was smooth after our VA_RANGE fix.
**That was wrong** — the user corrected me. Here's the actual mapping:

| Build state | MMU faults | glmark2 visual | 100-cap test |
|---|---|---|---|
| Pre-VM-base-fix (with MMU faults) | many | **smooth, ~4 FPS** | 100/100 same hash |
| Post-VM-base-fix (no MMU faults) | 0 | **broken visuals, ~20 FPS** | 8 unique hashes |

So MMU fault recovery's **forced sync** (full GPU reset, ~1-2s pause) was
masking the bug. The "100/100 same hash" we briefly saw was the actual
correct render path being achievable when the GPU was forced to wait.
**Without those forced waits, a race condition surfaces.**

This validates your earlier sync/timing analysis.

## Sleep test: did NOT collapse the cycle

Per your sync hypothesis I ran `usleep(100000)` between each
`gl-cap-and-regdump-mainline` invocation. Result: **no change** —
still 8 unique hashes ABCDEFGH. So the race is **WITHIN a single
submit/batch**, not BETWEEN batches/processes.

## SCRATCH_REG2 was a red herring

You may recall I shared yesterday's empirical finding from the user
that `submit->seqno & 0xF` deterministically controlled rendering
(16-cycle pattern, 9 stable + 7 broken). The user's prior fix was
to mask the lo-nibble (`seqno & ~0xFu`) so SCRATCH_REG2 always lands
on the "stable" lo=0 slot.

**Today I tested writing 0 unconditionally to SCRATCH_REG2** — zero
effect. Same 8 unique hashes. So the previous correlation
(`seqno`-cycle ↔ rendering) was correlation not causation. SCRATCH_REG2
is innocent.

## The wptr_delay experiment: timing changes content but not structure

Mainline already has a debug knob `a2xx_debug_wptr_delay` that adds
`udelay(N)` before the WPTR write at submit time. Tested several
values:

| Setting | Unique hashes | Notable |
|---|---|---|
| 0 (original kernel) | 8 | original set: 5adc3160, ebb5d66d, ... |
| 0 (current build) | 8 | NEW set after some kernel changes |
| 5000 (5ms) | 9 | NEW set + 1 transient outlier |
| 50000 (50ms) | 8 | same NEW set, no outlier |

**Critical finding**: timing perturbation **changes the content** of
the broken outputs but leaves the **structural cycle of 8 unchanged**.
None of the 8 hashes at any timing match the expected correct render
(R~49, G~64, B~79).

## The big new insight: cycle decomposes into 2 axes (spatial × color)

I generated per-tile spatial corruption masks for all 8 new hashes
(triangle covers 12 GMEM tiles, each 256×256). Looking at WHICH
tiles are corrupted in each cycle position:

```
                  col0  col1  col2  col3
11fe5f2e (40%):    .    24    24    .    row 0    | spatial
                   4    85    85    4    row 1    | mask = "all"
                  44    85    85   44    row 2    |

4c0bb4a5 (40%):    .    24    24    .    row 0    | IDENTICAL spatial
                   4    85    85    4    row 1    | mask to 11fe5f2e
                  44    85    85   44    row 2    | but different hash

699a8434 (2%):     .    24     .    .    row 0    | "almost good"
                   .     .     .    .    row 1    | only 1 tile bad
                   .     .     .    .    row 2    |

4032a7a2 (10%):    .     .     .    .    row 0    | bottom-right only
                   .     .     .    .    row 1    |
                   .     .    85   44    row 2    |
```

**Decomposition**:
* About **5-6 distinct spatial masks** observed across the 8 hashes
* Each spatial mask has **1-2 color variants** (different hash, identical
  bitmask of corrupted tiles)
* 11fe5f2e + 4c0bb4a5 are the clearest example: same tile mask,
  different content

Total unique outputs = (spatial masks) × (color variants) = 8.

## What this implies

The bug is NOT a single 8-element hardware counter. It's **two
independent state sources combining**:

1. **Spatial source** (~4-bit-ish, ~5-6 distinct values): determines
   WHICH GMEM tiles get corrupted
2. **Color source** (~1-bit-ish, ~1-2 distinct values): determines
   what the corrupted color content looks like for the bad tiles

These cycle at different rates, producing 8 unique hash combinations.

The **spatial source** behaves exactly like "which of N internal
processing slots is currently in use, and which slots are
contaminated." The "almost good" 2%-corruption case (`699a8434`)
suggests there's a configuration where only ONE slot is bad —
strongly hinting at the slot-rotation model from earlier discussion,
just with different topology than we initially assumed.

The **color source** suggests an ADDITIONAL independent state that
modulates fragment output. Possibilities:
- An alternating ping-pong buffer for fragment data
- Two parallel SRAM banks for parameter storage
- A 1-bit endianness/byteswap state that flips per-batch

## Direct asks

1. **Two-source bug architecture** — does Adreno 220 have any known
   pair of internal state machines that could combine to give
   "spatial × color" cycling? E.g., a per-tile slot counter (4-state)
   crossed with a per-batch flip-flop (2-state)?

2. **Why does the per-tile mask vary across cycle positions even
   though the triangle is byte-identical**? If the slot allocation
   is purely round-robin per tile, every cycle position should
   corrupt the same tiles. The fact that different cycle positions
   have different bitmasks means the slot-to-tile mapping ITSELF
   varies per cycle position. What hardware structure allows
   per-tile slot assignment to vary per-batch?

3. **The "best" cycle position (`699a8434`, 2% corruption)** —
   only ONE tile bad. This is by far the closest to "good" we've
   ever achieved. Is there a way to **lock the cycle counter** to
   always land on this position? On other hardware that's done via
   a CP_LOAD_CONSTANT_CONTEXT broadcast, but that hangs A22X in
   mainline.

4. **What changed about glmark2 between MMU-fault-state and
   no-MMU-fault-state**? glmark2 doesn't use a deterministic
   capture; if MMU recovery's sync was making it correct and
   without sync it's broken, the race must be triggering on
   ordinary user-app draw flow too — not just our test. So fixing
   this would benefit ALL apps, not just our test harness.

## Files

* PNGs of all 8 new broken outputs:
  `reports/fb-captures/wptr-delay-NEW-hashes/sample-*.png`
* Per-position spatial corruption masks:
  `reports/fb-captures/wptr-delay-NEW-hashes/diffs/diff-*.png`

We're closer than ever - the 2-axis decomposition is the cleanest
structural insight yet. What experiment would best disambiguate the
spatial source from the color source?
