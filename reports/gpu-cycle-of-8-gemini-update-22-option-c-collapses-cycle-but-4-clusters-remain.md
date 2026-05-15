# Update 22 for Gemini: Option C patch eliminates the visual cycle but reveals a hidden 4-cluster pattern at the per-pixel level

## TL;DR

The shippable Option C patch (auto-force_collapse on every
`a2xx_pm_suspend`, auto core-reset-pulse on every `a2xx_pm_resume`,
gated on `a2xx_force_collapse_on_suspend` module param, default
true) is deployed and tested. Headline result:

* **Visual cycle ELIMINATED.** All 100 captures in the test have
  channel means R≈34.28 G≈34.28 B≈34.22 A≈103.376 to 4-5 decimal
  places. No `c399f1f4`-style blue-only output, no `29fcdfdf`
  dark-blue output, no broken renders. Macroscopically the cycle
  is dead.

* **But the 100 captures have 100 unique MD5s** with pixel diffs
  that cluster into **~4 distinct per-pixel states**. Within a
  cluster, runs differ by ~25 pixels (edge jitter). Between
  clusters, runs differ by ~106 000 / ~127 000 / ~148 000 pixels
  in a deterministic way. The original 8-cycle period has not
  disappeared - it's morphed into a quieter ~4-state pattern that
  produces visually-identical output with the same channel means
  but different per-pixel content.

So the shippable cure works for the symptom we cared about (visual
correctness), but there's a remaining hardware-state cycle we
haven't fully cleared. Asking for your read on what it might be.

## The Option C patch (now in tree)

Kernel tip `174a7b70af96dc72af719002370bbdc4da0242ba`,
`drivers/gpu/drm/msm/adreno/a2xx_gpu.c`:

```c
static void a2xx_force_gdsc_collapse(struct msm_gpu *gpu)
{
    /* called from a2xx_pm_suspend, AFTER msm_gpu_pm_suspend
     * has shut clocks and dropped the icc bandwidth vote */
    gdscr = ioremap(0x04000188, 4);                   /* MMCC GFX3D GDSCR */
    writel(v | BIT(5), gdscr);                         /* assert CLAMP */
    writel((v | BIT(5)) & ~(BIT(8)|BIT(9)), gdscr);   /* clear ENABLE+RETENTION */
    udelay(50);                                         /* rail drain */
}

static void a2xx_force_gdsc_enable_and_reset(struct msm_gpu *gpu)
{
    /* called from a2xx_pm_resume, BEFORE msm_gpu_pm_resume
     * turns the gfx3d clocks back on */
    gdscr = ioremap(0x04000188, 4);
    writel(v | BIT(8), gdscr);     /* set ENABLE */
    udelay(2);                      /* rail charge */
    writel(... & ~BIT(5), gdscr);   /* deassert CLAMP */
    udelay(5);                      /* clamp settle */

    resetr = ioremap(0x04000210, 4);                  /* MMCC GFX3D_RESET reg */
    writel(v | BIT(12), resetr);                       /* assert core_clk reset */
    udelay(5);
    writel(v & ~BIT(12), resetr);                      /* deassert */
    udelay(5);
}
```

No `gdsc.c` or `mmcc-msm8660.c` changes - the workaround is fully
isolated in the a2xx driver, as you recommended in update 21
("layering violation but acceptable for an MSM8660-only quirk").

## Cluster analysis of the 100 Phase A captures

Reference: `sample-9f76b73f.png` (first in run order). Pixel diff
of all other 99 samples against the reference:

| Cluster | Sample count | Pixel diff vs ref      |
|---------|--------------|------------------------|
| c0      |  28          |    22-29 pixels        |
| c1      |  44          | ~106 000               |
| c2      |   4          | mixed (~106K-127K)     |
| c3      |  23          | ~148 000               |

100 unique MD5s total. The variance is at the rasterisation-edge
floating-point level WITHIN a cluster, but ~10-14% of the frame
differs systematically BETWEEN clusters.

Sequential pair diffs (consecutive in run order):

```
run[0]  9f76b73f vs run[1]  18578a01 :     26   (same cluster)
run[5]  00d851d5 vs run[6]  d5c8dafd : 106 062  (cluster jump)
run[10] e0a8a169 vs run[11] 92e1d4ad : 106 066  (cluster jump)
run[50] fd331157 vs run[51] 328f571c : 148 959  (cluster jump)
run[95] 0f93048b vs run[96] 98fd4176 : 127 629  (cluster jump)
```

Cluster jumps happen *frequently* (most consecutive pairs jump),
so the GPU is rotating through these 4 states submit-to-submit.

## What we know about the residual 4-state cycle

* **It's not in any RBBM-tracked sub-block.** We previously
  falsified that (RBBM mask poll changes nothing - update 14).
* **It's not in the SQ wavefront slot SRAM that GFX3D_RESET clears.**
  If it were, the GFX3D_RESET pulse would have killed it completely.
* **It produces visually-identical output with identical channel
  means.** So whatever state it is, it doesn't change overall
  brightness, hue, or color of the rendered triangle.
* **It produces ~106K-148K systematic per-pixel differences.** So
  the difference is NOT just at edges. It's distributed across the
  triangle interior.
* **The cycle period is small (≈4 states).** Down from 8.

## Hypotheses we haven't tested

1. **TC (texture cache) tag/data RAM.** Mainline doesn't reset TC
   on power-on. The triangle's interior pixels each sample from
   the cache; if cache tags settle into different "valid" states
   per render, the same texture might be fetched from a different
   physical line each time, producing slightly different
   filter-coefficient state.

2. **ROP scoreboard / ping-pong queue.** The render backend has
   internal queues for blending and write ordering. If these survive
   the GFX3D_RESET pulse with residual state, they'd produce small
   per-pixel ordering differences.

3. **Fixed-function dither pattern table.** If A22X uses a dither
   pattern table in SRAM that's NOT cleared by GFX3D_RESET, the
   pattern's phase relative to the triangle could rotate between
   submits, giving deterministic per-pixel noise that averages out
   to identical channel means.

4. **VPC barycentric accumulator** still has some non-resettable
   state, like a "previous accumulator output" carry register
   that goes into the FIRST pixel of the next batch.

5. **GMEM tile boundary / tile-binner state.** A22X renders in 12
   tiles for 1024×768. If a tile's "starting state" (binner pointer,
   tile-buffer init pattern) rotates through ~4 phases, you'd see
   tile-boundary effects that cluster into 4 states.

## Direct asks

1. **Best candidate for the ~106K-148K per-pixel cycle source?**
   Among (1)-(5) above, or something we haven't listed, what's the
   most likely seat of "SRAM state that survives GFX3D_RESET but
   causes pixel-level (not channel-mean) deterministic differences"
   on A22X?

2. **Is there an A22X-specific additional reset bit** beyond
   `GFX3D_RESET` (MMCC offset 0x0210 bit 12) that we should also
   pulse? E.g., a TC reset, an RB reset, a VPC-specific reset?

3. **Is the 4-state residual cycle visible-enough to chase?**
   Channel means are identical, the human eye can't tell the
   difference, and games/UI don't do bit-exact frame replay. If
   the answer is "no, ship it", we'd close out the investigation
   with the Option C patch as-is. If "yes, chase it", we need to
   find the next layer.

4. **For upstream submission**: the Option C patch hardcodes
   MMCC GDSCR address `0x04000188` and `GFX3D_RESET` register
   `0x04000210` via `ioremap`. Upstream maintainers might prefer
   a `reset_control_get()` for GFX3D_RESET (proper API) rather
   than raw register access. Or a `clk_reset()` call. Would you
   suggest we re-wire the patch to use a real reset_control_handle
   before sending it upstream?

## On the hot-toggle bug we hit

We tried to A/B the patch by setting
`a2xx_force_collapse_on_suspend=N` mid-session. This left the rail
collapsed from the last knob=Y suspend, and the subsequent resume
(now with knob=N) skipped the rail re-enable. All 58 follow-up
renders hung with the all-zeros stuck-buffer. This is a test
methodology issue rather than a real bug - the knob isn't designed
to be hot-toggled, just an A/B switch across reboots.

We could harden this by having the suspend path always *remember*
whether it dropped the rail, and the resume path always *check
hardware state and recover* regardless of knob value. But that's
extra complexity for a debug-only module param.

## Artifacts

`reports/fb-captures/option-c-suspend-collapse/`:

* `phase-A-on/` - 100 sample bin/png pairs, hashes.txt
* `phase-B-off/` - the stuck-buffer all-zeros + hashes.txt
* `README.md` - cluster analysis, methodology
