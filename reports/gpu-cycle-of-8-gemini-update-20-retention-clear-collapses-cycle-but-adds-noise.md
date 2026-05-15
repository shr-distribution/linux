# Update 20 for Gemini: RETENTION-clear collapses the cycle to the correct render — but introduces new per-pixel noise we don't have a source for

## TL;DR

The new force_collapse path that *also* clears `LEGACY_FS_RETENTION_MASK`
(BIT 9) of the GFX3D GDSCR collapses the period-8 cycle to a single
visual render with channel means identical to historical correct
hash `5adc3160` to 5 decimal places. Architecturally this is the
proof we wanted — clearing real GDSC state breaks the SRAM-cycle.

But: **the rendering is now bit-non-deterministic.** Channel means
are stable across runs, but ~100,000 pixels (≈13% of the framebuffer)
differ between any two consecutive renders. That noise was *not*
present in the baseline — there each cycle slot produced 12-13
bit-for-bit identical renders. Whatever we just unblocked by
clearing RETENTION introduces per-pixel noise that needs an
explanation before we can call this shippable.

## What changed in the kernel

`drivers/gpu/drm/msm/adreno/a2xx_debugfs.c` `force_collapse_set()`
now does:

```c
ret = pm_runtime_force_suspend(gdev);          /* SW suspend */
ret = genpd->power_off(genpd);                  /* gdsc_disable: clears
                                                   ENABLE (BIT 8) only */
{
    void __iomem *gdscr = ioremap(0x04000188, 4);
    u32 v = readl(gdscr);                        /* observed: 0x00000220
                                                   (RETENTION + CLAMP) */
    writel(v & ~(BIT(9) | BIT(8)), gdscr);       /* now clears BOTH
                                                   RETENTION + ENABLE */
    /* observed post-clear: 0x00000020 (CLAMP only) */
    mdelay(5);
    iounmap(gdscr);
}
ret = genpd->power_on(genpd);                    /* gdsc_enable: sets
                                                   ENABLE again, 7us legacy
                                                   footswitch sequence */
ret = pm_runtime_force_resume(gdev);             /* SW resume → triggers
                                                   a2xx_pm_resume which sets
                                                   needs_hw_init=true and
                                                   the next submit runs
                                                   a2xx_hw_init + a2xx_me_init */
```

Final GDSCR after the cycle reads `0x00000100` (ENABLE only, no
RETENTION). Confirmed in dmesg. So we genuinely cleared retention
between off and on.

## The headline data

100-cap-style test, all single-triangle renders via
`gl-cap-and-regdump-mainline`. `R_mean G_mean B_mean A_mean` are
ImageMagick channel-mean readouts on the 1024×768 RGBA captures.

### Baseline (no force_collapse) — Phase 1, 8-cycle present

```
bl-sample-5adc3160   R=34.27  G=34.28  B=34.21  A=103.37
                     ^^ uniform-grey triangle, the historical "correct"
                        render; appears 12-13× per 100 captures, each
                        instance bit-for-bit identical
bl-sample-29fcdfdf   R= 7.46  G= 2.46  B=21.25  A=103.37
                     ^^ broken: dark blue tint, R/G near-zero
bl-sample-c399f1f4   R= 0.00  G= 0.00  B=61.90  A=103.37
                     ^^ broken: ONLY blue channel, R/G fully zero
bl-sample-fb12cd4c   R= 1.05  G= 1.05  B=35.19  A=103.37
                     ^^ broken: faint R/G, blue dominant
... [4 more 'wrong' hashes with similar broken character]
```

### Phase 3 — force_collapse fired before each of 20 captures

20 unique MD5s out of 20 captures. All four sampled have:
```
fc-sample-1e563a4c   R=34.2833  G=34.2874  B=34.2235  A=103.3760
fc-sample-25797394   R=34.2838  G=34.2870  B=34.2231  A=103.3760
fc-sample-d83a1af1   R=34.2835  G=34.2875  B=34.2235  A=103.3760
fc-sample-f8e4d15e   R=34.2837  G=34.2872  B=34.2234  A=103.3760
```

Channel means agree with `5adc3160` to **4-5 decimal places**. Visually
identical to the historical correct render. The MD5 difference is
~100K differing pixels per pair (~13% of frame).

### Phase 4 — no force_collapse after Phase 3

Cycle reformed but with a new hash set:
```
res-sample-2821b4f2  R=10.97  G=10.98  B=27.83  A=103.37   (new shade)
res-sample-2fffa135  R= 7.46  G= 2.46  B=21.25  A=103.37   (= bl-29fcdfdf)
res-sample-82b8940b  R= 1.05  G= 1.05  B=35.19  A=103.37   (= bl-fb12cd4c)
res-sample-dec69f80  R=23.56  G=24.10  B=13.80  A=103.37   (new shade)
```

So the cycle returns once force_collapse stops, but the SRAM lottery
lands on a *different* mix of slots than before — consistent with
the GDSC having genuinely been cycled.

## Pixel-level diff measurements

`compare -metric AE -fuzz 0%`:

| pair                                             | differing pixels |
|--------------------------------------------------|------------------|
| fc-1e563a4c vs fc-25797394 (post-fc, run-to-run) |       106 073    |
| fc-1e563a4c vs fc-d83a1af1 (post-fc, run-to-run) |       148 958    |
| bl-5adc3160 vs fc-1e563a4c (correct vs post-fc)  |        63 675    |
| bl-29fcdfdf vs bl-5adc3160 (broken vs correct)   |       259 817    |

Observations:
1. The "wrong-vs-correct" baseline diff (the actual cycle slot
   difference) is ~260K pixels. That's the size of the *real* visual
   error. So the post-fc per-run noise (~100K-150K pixels) is
   smaller than the cycle slots' visible error, but still big.
2. **Post-fc renders are *closer* to baseline correct than they are
   to each other.** Run-to-run noise (~106-149K) is bigger than the
   distance from "correct" (~63K). This is consistent with each
   post-fc render being independently close to "correct" but each
   landing slightly differently.

PNG samples + diffs saved at:
`reports/fb-captures/force-collapse-retention-test/`
including channel-mean READMEs and pixel-diff visualizations.

## What we know about the noise

- Not in baseline: the 8-cycle had each slot produce 12-13
  bit-for-bit identical renders. Same-MD5 outputs, every time.
- Only emerges after force_collapse with RETENTION-clear.
- Symmetric noise (channel means unchanged across runs → pixels
  going +N and -N must average out).
- ~13% of pixels affected — far more than just rasterization-edge
  jitter (which would be ~0.5%).
- Triangle interior, not just edges (means change too much to be
  just edge antialiasing).

## What it could be

Hypotheses, asking your read:

1. **TC (texture cache) reading uninit values.** Our test shader
   doesn't sample textures, but the SQ might still walk through
   TC for varying-fetch on some path. If TC SRAM is now uninit
   (after RETENTION clear) and the CP doesn't re-init it before
   the first user draw, fetched values are random.

2. **VPC (Vertex Parameter Cache) holding garbage.** VPC interpolates
   varyings VS→FS. If after a RETENTION-clear the VPC SRAM is
   uninit and the rasterizer reads it, fragment color computations
   would have noise overlay.

3. **GMEM tile state per-tile inconsistency.** A22X tile-binner
   produces 12 tiles for 1024×768. If only some tiles are
   correctly initialized after our power cycle (Mesa programs
   things assuming GDSC-persistent state), some tiles render fine
   and others have garbage.

4. **Some clock-tree start-up jitter.** When `gdsc_enable` brings
   the rail back, internal PLLs/clocks may take a few µs to settle.
   If rendering starts before they're stable, sampling/interpolation
   precision suffers. The 7 µs total `gdsc_enable` could be too
   short for the AHB/MH clocks to be fully clean.

5. **Microcode/firmware load incomplete.** `pm_runtime_force_resume`
   triggers `a2xx_hw_init` which reloads PM4/PFP firmware via
   register-block uploads. If the upload completes but the CP
   hasn't fully internalized it before ME_INIT, ME_INIT could
   leave residual state.

We have a `a2xx_emit_sanitizer_preamble` that runs 8x scrub of
ALU/TEX/Bool/Loop banks per cross-context submit. That should
cover #1 if TC SRAM is constants-shaped — but TC may have
separate state for actual texture-sample cache lines that the
preamble doesn't touch.

## Direct asks

1. **Which of (1)-(5) is most likely** to produce ~100K-pixel,
   channel-mean-stable noise? You have the deepest model of the
   A22x SQ/VPC/RB pipeline.

2. **Is there a "mainline missing init" pattern** that emerges
   only when the rail is *truly* power-collapsed, that we should
   add to `a2xx_hw_init` or `a2xx_me_init`? Examples we might be
   missing:
   - TC SRAM zero-initialization sequence
   - VPC varying-cache reset
   - PM_OVERRIDE adjustments for cold-start vs warm-resume
   - Some ME_INIT field we're getting wrong only when the CP
     started cold

3. **Should the noise be fixable by adding a cycle of dummy
   draws after force_collapse?** Idea: after the GDSC cycle, emit
   a few dummy "scrub" draws to populate VPC/TC with deterministic
   values before the user's first real draw. Different from the
   prior failed `VS_REGS=63` patch — these would just prime the
   caches, not try to allocate slots.

## Why this matters for the shippable fix

If we ship the "Option A" governor fix (gfx3d_gdsc gets a real
governor that auto-collapses on idle, with RETENTION-clear), every
runtime-suspend/resume cycle will produce this noise. That's not
acceptable — single-pixel noise is fine, ~13% framebuffer
difference per frame is not.

If we ship the "Option B" driver-side toggle, same concern.

So we need to find the noise source and eliminate it as part of
the fix package. The cycle is fixable; we just have a follow-on
problem to solve.

## Tip / SHA reference

Branch `tenderloin/6.18/upstream-patches`,
tip `457e098800b1ada76b62d825e5d2c83f134d6d68`:

* `drivers/gpu/drm/msm/adreno/a2xx_debugfs.c` — force_collapse with
  RETENTION-clear via ioremap of MMCC GDSCR @ 0x04000188

Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046
active.

Captures and diffs:
`reports/fb-captures/force-collapse-retention-test/`
