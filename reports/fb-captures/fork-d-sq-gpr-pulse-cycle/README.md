# Fork D — SQ_GPR_MANAGEMENT pulse (2026-05-11)

## Source

Mesa patch 0093 Fork D applied on top of Fork C v2:
- Added `OUT_PKT0(SQ_GPR_MANAGEMENT) = 0x0007f010` (binning shader partition: VTX=127, PIX=1, static) after the LRZ_VSC_CONTROL=3 + WFI
- Followed by `OUT_PKT0(SQ_GPR_MANAGEMENT) = 0x00040401` (Mesa's normal dynamic value: VS=64, PS=64, REG_DYNAMIC=1) to restore for the real shader
- Fixed WFI ordering: WFI is now AFTER LRZ_VSC_CONTROL=3 per webOS rather than before
- libgallium md5: `c17b080b010e93d8d2fd23e9d8f8f01d`

## Result

Zero GPU hangs across 30 captures (clean). Cycle remains period-8 but composition changed.

### Hash table — Fork D's 8-cycle

```
Order: 6ed2ef94 → ba830c14 → 2c65153c → d3e2a6fe → ae2f9645 → c8705ac5 → 15ad14f0 → 5adc3160 → (repeat)
```

### Side-by-side with Fork C v2

| Slot | Fork C v2 | Fork D | Status |
|---|---|---|---|
| 0 | 5adc3160 (err=0) | 5adc3160 (err=0) | GOLDEN preserved |
| 1 | 2c65153c (err=64) | 2c65153c (err=64) | "5/6 correct" preserved |
| 2 | 27b74302 (err=230) | c8705ac5 (err=219) | NEW, slightly better |
| 3 | add5096d (err=312) | ae2f9645 (err=316) | NEW, comparable |
| 4 | 6ed2ef94 (err=385) | ba830c14 (err=331) | NEW, ~14% better |
| 5 | 8cbc2640 (err=414) | 6ed2ef94 (err=385) | shifted |
| 6 | d71843ae (err=476) | 15ad14f0 (err=490) | shifted |
| 7 | 15ad14f0 (err=490) | d3e2a6fe (err=518) | NEW, slightly worse |
| **Sum (wrong)** | **2371** | **2323** | **-2.0% net** |

Err = Manhattan distance of per-tile mean RGB from the golden render. Lower is closer to correct.

### Per-hash interesting tile patterns

- `ae2f9645`: TOP ROW byte-identical to golden (T00, T01, T02 all delta=0). Bottom row entirely wrong. → "half correct"
- `c8705ac5`: All 6 tiles slightly off, no tile bit-exact. → "uniformly close"
- `2c65153c` (preserved): T00/T02/T10/T11/T12 byte-identical to golden, only T01 partial blue loss. → "5/6 correct" (still the closest non-golden in either fork)

## Interpretation

Fork D's SQ_GPR_MANAGEMENT pulse changed 4 of 7 wrong hashes (50% replacement rate) but did NOT collapse the cycle. Total error reduced by only **2%** — the period-8 structure is statistically robust against this register pulse.

This **falsifies the Gemini "master key" hypothesis** that SQ_GPR_MANAGEMENT controls the wavefront-slot allocator driving the cycle. The cycle has 8 stable internal states; SQ_GPR_MANAGEMENT shuffles which 8 are used but doesn't reduce the count.

The "5/6 correct" anchor (2c65153c, err=64) persists unchanged across both forks, suggesting it represents a specific binner-state-machine outcome that's not register-pulse-sensitive.

## What we've learned

After Fork C v2 + Fork D:

1. **OUT_PKT0 is the correct way to write sub-0x2000 regs** (Fork C v2 fix). Real behavioural effect confirmed.
2. **0xC00=1 + LRZ_VSC_CONTROL=3 + VSC_PIPE config** engages the binner cleanly (no hangs).
3. **SQ_GPR_MANAGEMENT pulse has minor effect** — shifts 4 of 7 wrong hashes but doesn't break the 8-state structure.
4. **The cycle is internal binner state**, not influenced by any register write Mesa is making at submit time.

## Strategic options remaining

1. **Kernel-side SQ_GPR_MANAGEMENT pulse** — Gemini suggested doing it at `a2xx_power_on` rather than per-batch. Could test if early-boot pulse has different effect than per-batch pulse.
2. **Real binning shader pass (Fork A/B full port)** — emit a VS-only stripped shader binning pass with `batch->binning` IB, then render pass. The proper webOS model. Substantial effort.
3. **Capture register state across the 8 hashes** — re-verify the "register invariant" finding from earlier. With Fork D active, maybe `LEIA_SQ_DEBUG_*` registers (0x0DAE..0x0DB7) finally expose the cycle phase.
4. **Accept the 12.5% rate and ship Fork C v2 / Fork D as the new baseline** — at minimum we have a stable no-hang state with deterministic outputs.
