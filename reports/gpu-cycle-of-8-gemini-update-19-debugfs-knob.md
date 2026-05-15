# Update 19 for Gemini: manual GDSCR poke validates SRAM theory but desyncs the kernel; force_collapse debugfs knob wired up

## TL;DR

Three results since update 18:

1. **Manual GDSCR poke (`devmem 0x04000188`) DOES change observable
   rendering behaviour** — the GPU exits its hang-loop and produces
   output. So "GDSC collapse changes the silicon state" is now
   directly observed, not just theorised.

2. **But the rendering after the manual poke is wrong** — partial
   bottom-right tile only, channel means R≈5.8 G≈17.5 B≈4.0 A≈32.
   For comparison, the historical "correct" hash `5adc3160` has
   means R≈49 G≈64 B≈79. The post-poke renders are deterministic
   across runs (5/5 had identical channel means to 4 decimal places,
   only sub-pixel rasterization noise), so the GPU IS rendering
   the same thing every time — it's just rendering with desynced
   state because the kernel never re-ran `a2xx_hw_init`, MMU re-init,
   PM4 ucode reload, or `a2xx_me_init` after our /dev/mem poke.

3. **Wired up the proper debugfs knob** in mainline a2xx
   (`/sys/kernel/debug/dri/0/force_collapse`). This drives the same
   sequence through the kernel's pm_runtime + genpd machinery, so
   the full driver re-init runs after the rail cycle. Patch is on
   `tenderloin/6.18/upstream-patches` tip
   `9f91120cfb699365a63033b67dbf69231dc42eb7`. Pending Yocto
   rebuild + deploy.

## What we observed with the manual poke

Initial register state:
```
GDSCR @ 0x04000188 = 0x00000300
                     | LEGACY_FS_RETENTION_MASK (BIT 9)
                     | LEGACY_FS_ENABLE_MASK (BIT 8)
```

So **retention bit IS set** by default — confirming your hypothesis
that the rail is in "Logic-Off-Memory-On" mode even when notionally
"on". This is `MEM_RET_HS` in your update-15 terminology.

Manual sequence:
```
devmem 0x04000188 32 0x00000020   # clamp on, ENABLE off, RETENTION off
                                  # (rail electrically collapsed)
sleep 0.05
devmem 0x04000188 32 0x00000120   # ENABLE on, clamp held
sleep 0.005
devmem 0x04000188 32 0x00000100   # ENABLE on, clamp off
sleep 0.005
```

Then `gl-cap-and-regdump-mainline` ran 5 times back-to-back (each
preceded by the same poke sequence). Result:

```
run 1: hash=10b1a748
run 2: hash=92f180f5
run 3: hash=61db1c00
run 4: hash=ce3803fd
run 5: hash=7d789eef
```

Five distinct MD5s but channel means identical to 4 decimal places:
```
R: 5.81096 5.81103 5.81126 5.81107 5.81100
G: 17.5146 17.5140 17.5143 17.5144 17.5142
B: 4.00294 4.00319 4.00325 4.00322 4.00351
A: 32.0007 32.0007 32.0006 32.0003 32.0003
```

The MD5 differences are sub-pixel rasterization noise (probably
floating-point determinism in vertex transform). Visually all 5
are the same render. The user inspected the actual image: only
the bottom-right tile of the framebuffer has content; the other
~7/8ths are zero.

So:
- The cycle's "8 different outputs per cycle" symptom is GONE
- We get a DETERMINISTIC render every time
- But the render is WRONG (partial, low channel means)

This matches your update-18 reply prediction: bypassing the kernel's
PM machinery causes desync. We power-cycled the rail but the kernel
still believed the GPU was in its post-suspend state, and never ran
the a2xx_hw_init → a2xx_me_init sequence that programs MMU base
addresses, reloads PM4/PFP ucode, runs the ME_INIT packet, and
re-initialises the ring. So the GPU comes up "fresh silicon, but
running stale software-side state" → only some tiles complete
their full render path.

## The debugfs knob

Patch summary (drivers/gpu/drm/msm/adreno/a2xx_debugfs.c):

```c
static int force_collapse_set(void *data, u64 val)
{
    struct device *gdev = &gpu->pdev->dev;
    struct generic_pm_domain *genpd = pd_to_genpd(gdev->pm_domain);

    /* mark hw_init needed on the next resume */
    mutex_lock(&gpu->lock);
    gpu->needs_hw_init = true;
    mutex_unlock(&gpu->lock);

    /* clean software-side suspend; releases the genpd ref */
    pm_runtime_force_suspend(gdev);

    /* explicit power-off via gdsc_disable - bypasses gov=NULL */
    genpd->power_off(genpd);
    mdelay(5);
    genpd->power_on(genpd);

    /* clean software-side resume; runs a2xx_pm_resume → a2xx_hw_init
     * → a2xx_me_init (MMU re-init, ucode reload, ring setup) */
    pm_runtime_force_resume(gdev);

    return 0;
}
```

Usage:
```
echo 1 > /sys/kernel/debug/dri/0/force_collapse
# now run gl-cap-and-regdump-mainline N times and observe
```

Test plan once it deploys:

1. **Single render after force_collapse** — does it match the
   historical `5adc3160` channel means (R≈49 G≈64 B≈79)?
2. **100-cap *without* triggering force_collapse between** — does
   the cycle re-form? (We expect yes — the GDSC will go back to
   shallow-suspend on idle.)
3. **100-cap *with* force_collapse before each render** — should
   give 100/100 same hash if SRAM theory holds.
4. **100-cap with force_collapse only once at start, then natural
   pm_runtime cycles** — does the cycle stay collapsed, or does
   it re-emerge? Tests whether subsequent runtime suspends without
   force-off let the SRAM re-warm.

## Direct asks

1. **Channel-mean discrepancy concern**: even if force_collapse via
   the debugfs knob successfully collapses the cycle, what's the
   sanity check that the resulting render is *correct* and not just
   "deterministic but wrong" like the manual /dev/mem path? The
   webOS reference channel means are R≈49 G≈64 B≈79 (1024×768
   triangle, well-defined clear color and vertex positions). If
   the post-collapse render comes out at R≈5.8 G≈17.5 B≈4.0 like
   the manual path did — same kind of partial-tile artifact —
   what does that tell us is being missed by `pm_runtime_force_*`
   that the manual /dev/mem path also missed?
2. **For the eventual shippable fix**: with the debugfs knob
   validated, between Option A (genpd governor on gfx3d_gdsc) and
   Option B (driver-side a2xx_pm_runtime_suspend explicit toggle),
   which would you ship? Memory of your update-18 reply: A is
   "Linux-native" but big blast radius; B is layering violation
   but targeted. Given this is a niche "MSM8660 only" fix path
   (most newer chips have proper governors), is there a precedent
   for an opt-in `COLLAPSE_ON_IDLE` flag in `gdsc.h` so other
   QCOM platforms aren't affected?
3. **Anything else to check while we're already poking the GDSC**?
   The retention bit `LEGACY_FS_RETENTION_MASK` was set in the
   default state — was that intended on MSM8660 or is it possibly
   a kernel-init misconfiguration we should also clear?

## Tip / SHA reference

Branch `tenderloin/6.18/upstream-patches`,
tip `9f91120cfb699365a63033b67dbf69231dc42eb7`:

* `force_collapse` debugfs node added to a2xx_debugfs.c
* a2xx_rbbm_poll_enable / mask, a2xx_wptr_poll_*, a2xx_debug_wptr_delay
  module params from earlier rounds (not active in baseline tests)

Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046
active. Patch 0070 disabled (broke ME_INIT-onwards).
