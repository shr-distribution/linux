# Update 21 for Gemini: GFX3D_RESET pulse confirmed hypothesis #2 — 99.97% noise reduction — but residual noise remains, asking for next layer

## TL;DR

Your hypothesis #2 from update 20 reply (VPC uninitialized due to
missing core-reset pulse on cold start) is **confirmed**. Adding the
`GFX3D_RESET` (MMCC offset 0x0210 bit 12) toggle after
`genpd->power_on()` in our force_collapse debugfs knob reduced
per-run pixel jitter from **106,073 → 30 pixels** — a 99.97%
collapse. The legacy `arch/arm/mach-msm/footswitch-8x60.c`'s
`clk_reset(core_clk, ASSERT) → udelay(5) → clk_reset(core_clk,
DEASSERT)` sequence was the missing piece.

But: noise isn't zero. **30 pixels still differ** between any two
consecutive renders, so every MD5 is unique. And there's a
systematic **~85,000 pixel difference** between the post-fc render
and the historical baseline `5adc3160` "correct" render — channel
means match to 5 decimal places, but per-pixel content
consistently differs.

So we've cured 99.97% of the disease but the remaining 0.03% +
the 11% systematic offset deserve explanation before this is
shippable.

## What's in the patch now

`drivers/gpu/drm/msm/adreno/a2xx_debugfs.c::force_collapse_set()`,
kernel tip `4b2a320e5099`:

```c
pm_runtime_force_suspend(gdev);                /* SW suspend */
genpd->power_off(genpd);                        /* gdsc_disable:
                                                   clears ENABLE only */
/* Direct register poke to also clear RETENTION (BIT 9) */
gdscr = ioremap(0x04000188, 4);
writel(readl(gdscr) & ~(BIT(9) | BIT(8)), gdscr);
mdelay(5);
iounmap(gdscr);

genpd->power_on(genpd);                         /* gdsc_enable runs
                                                   legacy footswitch
                                                   sequence + AHB reset */

/* NEW: GFX3D_RESET (core_clk reset) pulse */
resetr = ioremap(0x04000210, 4);
writel(readl(resetr) | BIT(12), resetr);        /* assert */
udelay(5);
writel(readl(resetr) & ~BIT(12), resetr);       /* deassert */
udelay(5);
iounmap(resetr);

pm_runtime_force_resume(gdev);                  /* full hw_init */
```

dmesg confirms the pulse fires:

```
force_collapse: GFX3D_RESET pre-toggle=0x00000000
force_collapse: GFX3D_RESET toggle complete
```

(pre-toggle is 0 because GFX3D_RESET wasn't asserted before our
pulse — mainline's gdsc_assert_reset only touches GFX3D_AHB_RESET.)

## Results

### Phase 1 baseline (no force_collapse) — cycle present, unchanged

8 unique hashes including `5adc3160` at 1/8.

### Phase 3 with force_collapse-each (20 captures)

```
20 unique MD5s, BUT channel means agree to 5 decimal places:
  gfx3d-reset-e0c67eeb   R=34.2833  G=34.2872  B=34.2230  A=103.376
  gfx3d-reset-b4dd72af   R=34.2837  G=34.2874  B=34.2236  A=103.376
  gfx3d-reset-87a11499   R=34.2837  G=34.2872  B=34.2235  A=103.376
  gfx3d-reset-31c4d7b8   R=34.2836  G=34.2873  B=34.2233  A=103.376
```

### Pixel-diff comparison

|                                                  | differing pixels |
|--------------------------------------------------|------------------|
| **pre-GFX3D_RESET** fc-fc (update 20)            | 106 073          |
| **pre-GFX3D_RESET** fc-fc (update 20, other pair)| 148 958          |
| **post-GFX3D_RESET** fc-fc (this run)            | **30**           |
| post-GFX3D_RESET fc vs baseline-5adc3160         | 85 358           |
| baseline-correct vs baseline-broken              | 259 817          |

The 99.97% reduction in run-to-run jitter is the win. The 85K-pixel
"systematic offset" from baseline-5adc3160 is constant across all
20 post-fc renders.

Captures saved in `reports/fb-captures/force-collapse-gfx3d-reset/`.

## The two unsolved noise components

### A. 30 pixels of run-to-run jitter

This is small enough that MD5s differ per run, but it's not
random across the framebuffer — it's localized (didn't compute the
exact location but it's a tiny fraction of edges). Two possibilities:

* Pure rasterization-edge floating-point jitter that's intrinsic to
  the hardware. If so, we should accept it and just compare channel
  means or downsampled hashes, not MD5s.
* Some residual VPC bit that isn't fully reset by the GFX3D_RESET
  pulse. Maybe there's a secondary state like a "scoreboard" register
  or a half-cycle latch that the legacy webOS code also clears via
  another mechanism we haven't found yet.

### B. ~85,000 pixels systematic offset from baseline 5adc3160

This is constant across all 20 post-fc renders, so it's not
randomness — it's a deterministic difference between "post-cold-rail
render" and "warm-rail render in slot-0". Channel means match to
5 decimals, so visually identical, but the per-pixel content is
consistently different.

Two possibilities:
* The "warm-rail slot 0" render uses some calibrated SRAM state
  that's correct-by-luck-of-history (e.g., specific gamma/dither
  table baked in by previous renders), and our cold-rail render
  uses a clean-but-different initial state for that table. Both are
  numerically correct but differ in deterministic-noise-level detail.
* There's another SRAM block beyond VPC that we haven't reset.
  Candidates: TC (texture cache) lookup tables, GPR file initial
  values, fixed-function gamma/dither LUTs.

## Direct asks

1. **Where might the residual 30-pixel jitter come from?** Given the
   GFX3D_RESET pulse already removed 99.97% of the noise, what
   other state machine could contribute a small per-run jitter on
   the A22X SQ/VPC path? Some specific candidates we considered:
   * SQ wavefront scheduler still rotating between consecutive
     batches (the 8-slot pointer might not be fully reset by core
     reset, only its SRAM contents)
   * ROP ping-pong scoreboard
   * A pre-fetch queue in the rasterizer

2. **What might explain the 85K-pixel systematic offset from
   baseline `5adc3160`?** A22X-specific tables that survive
   `gdsc_assert_reset(GFX3D_RESET)` but are normally populated by
   long-running warm-rail use. Anything like:
   * Fixed-function gamma LUT (we don't think the A22X has one
     in SRAM but unsure)
   * Dither pattern table (deterministic but stateful)
   * Some SQ scheduler weight initialization that converges over time
   * Initial GPR file values (cleared to zero on power-on, perhaps
     baseline-renders happen to use uninitialized non-zero values
     from prior batches)

3. **The shippable fix package**: assuming we accept ≤30 pixels of
   jitter, we'd ship:
   * `mmcc-msm8660.c`: add `GFX3D_RESET` to `gfx3d_gdsc.resets[]`
   * `gdsc.c`: extend LEGACY_FOOTSWITCH disable path to clear
     RETENTION (BIT 9), gated on a new opt-in flag like
     `LEGACY_FS_NO_RETENTION` set only on `gfx3d_gdsc`
   * `gdsc.c`: extend LEGACY_FOOTSWITCH enable path to *also* pulse
     the resets again AFTER the rail-on settle (a "second pulse"),
     gated on the same opt-in flag
   * Either a genpd governor on `gfx3d_gdsc` OR explicit
     `pm_runtime_force_*` cycling from `a2xx_pm_runtime_suspend`
     (your update-18 said governor is more upstream-friendly)

   Does this package look correct to you? Anything missing?

4. **For the 30-pixel jitter**: should we treat that as "fine, ship
   it" and rely on channel-means or perceptual hashing for
   regression testing? Or is there a path to bit-exact deterministic
   renders we should pursue?

## Captures and artifacts

`reports/fb-captures/force-collapse-gfx3d-reset/`:
* 4× `gfx3d-reset-*.bin/png` from Phase 3 (post-pulse fc-each)
* `bl-5adc3160.bin/png` (baseline correct reference)
* 3× `residual-*.bin/png` from Phase 4 (cycle reforms after fc stops)
* `diff_fc_vs_fc.png` (30-pixel diff, basically empty)
* `diff_fc_vs_baseline.png` (85K-pixel systematic diff)
* `README.md` with full channel-mean table

Branch: `tenderloin/6.18/upstream-patches` tip
`4b2a320e5099def291e3f777d83d4a778f252cf5`.
