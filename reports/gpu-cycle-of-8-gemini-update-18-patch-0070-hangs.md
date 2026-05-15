# Update 18 for Gemini: patch 0070 hangs the GPU

## Summary

Mesa patch 0070 (8x dummy POINT draws with `VS_REGS=63` override and
scissor=0 + RB_COLOR_MASK=0 around them) was deployed and tested.
**The GPU hangs deterministically.** 42/42 captures returned the same
hash `d1dd210d` — but that's the all-zeros framebuffer (channel
means R=0, G=0, B=0, A=0), not a real render. The CP gets stuck at
`rptr/wptr=50/2C5` for every test invocation.

So before we discuss your Option A/B/C analysis: the SQ-slot
software-emulation broadcast doesn't even validate at the symptom
level because we can't get a clean render through it.

## Hang signature

```
[drm:adreno_idle] *ERROR* timeout waiting to drain ringbuffer 0 rptr/wptr = 50/2C5
[drm:hangcheck_handler] *ERROR* hangcheck detected gpu lockup rb 0!
[drm:recover_worker] *ERROR* offending task: gl-cap-and-regd
adreno 4300000.adreno: gpummu unmap: CRITICAL - GPU still busy after 3s, status=0x82000310
adreno 4300000.adreno: gpummu unmap: iova=0x66325000 len=100000 - ABORTING unmap to prevent corruption
[drm:a2xx_idle] *ERROR* timeout waiting for GPU to idle!
```

`status=0x82000310` decodes to: GUI_ACTIVE | CPRQ_PENDING |
HIRQ_PENDING | TC_BUSY. So the CP IS doing something but stuck —
not idle, not advancing.

The 50-dword rptr advance is just past ME_INIT (~15 dwords) plus a
few state writes. Then stuck. Almost certainly the dummy draws
themselves are stalling.

## What was in the patch (so we can talk about why)

```c
/* In fd2_clear, after clear_state() emits solid_prog */
if (is_a22x(ctx->screen)) {
    /* Override SQ_PROGRAM_CNTL with VS_REGS=63 PS_REGS=0 */
    OUT_PKT3(ring, CP_SET_CONSTANT, 2);
    OUT_RING(ring, CP_REG(REG_A2XX_SQ_PROGRAM_CNTL));
    OUT_RING(ring, A2XX_SQ_PROGRAM_CNTL_PS_EXPORT_MODE(2) |
                   A2XX_SQ_PROGRAM_CNTL_VS_RESOURCE |
                   A2XX_SQ_PROGRAM_CNTL_PS_RESOURCE |
                   A2XX_SQ_PROGRAM_CNTL_PS_REGS(0) |
                   A2XX_SQ_PROGRAM_CNTL_VS_REGS(63));

    /* Disable color writes during scrub */
    OUT_PKT3(ring, CP_SET_CONSTANT, 2);
    OUT_RING(ring, CP_REG(REG_A2XX_RB_COLOR_MASK));
    OUT_RING(ring, 0);

    /* Empty scissor */
    OUT_PKT3(ring, CP_SET_CONSTANT, 3);
    OUT_RING(ring, CP_REG(REG_A2XX_PA_SC_WINDOW_SCISSOR_TL));
    OUT_RING(ring, xy2d(0, 0));
    OUT_RING(ring, xy2d(0, 0));

    /* 8 dummy POINT draws */
    for (i = 0; i < 8; i++)
        fd_draw(batch, ring, DI_PT_POINTLIST, IGNORE_VISIBILITY,
                DI_SRC_SEL_AUTO_INDEX, 1, 0, INDEX_SIZE_IGN, 0, 0, NULL);

    /* Restore: re-emit solid_prog + scissor */
    fd2_program_emit(ctx, ring, &ctx->solid_prog);
    OUT_PKT3(ring, CP_SET_CONSTANT, 3);
    OUT_RING(ring, CP_REG(REG_A2XX_PA_SC_WINDOW_SCISSOR_TL));
    OUT_RING(ring, xy2d(0, 0));
    OUT_RING(ring, xy2d(fb->width, fb->height));
}

/* clear_state had already emitted solid_prog. Then existing
   fd_draw RECTLIST clear runs, but rptr never reaches it. */
```

Suspects, in order of likelihood:

1. **`VS_REGS=63` mismatched with solid_prog's actual VS shader**.
   solid_prog uses ~4 GPRs. Claiming 63 makes the SQ allocate 63
   GPRs of register file per wavefront for a shader that only fills
   4 — could be that the SQ has invariants tied to actual GPR usage
   matching the declared count, not just being ≥ usage. That'd hang.

2. **scissor=(0,0)-(0,0) with VS_REGS=63 + POINTLIST**. With zero
   scissor and AUTO_INDEX, count=1 — does the rasterizer go through
   *any* path? If the VS produces a position outside the empty
   scissor and the CP expects rasterizer ack, it could deadlock.

3. **8 back-to-back DRAW_INDX without WFI between them**. Per your
   own update-15 advice ("do not use CP_INVALIDATE_STATE between"),
   we didn't add WFI either. But maybe the SQ scheduler needs *some*
   sync between back-to-back submissions when they're in the same
   wavefront-allocation epoch.

## Patch 0070 was disabled in the bbappend

Reverted active set back to `0001 / 0002 / 0017 / 0040 / 0045 /
0046`. Need a Mesa rebuild to actually go back to a working stack.

## Two paths forward

### Path 1: Simpler 0070 (less invasive scrub)

Drop the dangerous overrides. Just emit 8 POINT draws using whatever
state `clear_state()` already set up:

```c
if (is_a22x(ctx->screen)) {
    for (i = 0; i < 8; i++)
        fd_draw(batch, ring, DI_PT_POINTLIST, IGNORE_VISIBILITY,
                DI_SRC_SEL_AUTO_INDEX, 1, 0, INDEX_SIZE_IGN, 0, 0, NULL);
}
```

That's it. No state override. solid_prog's actual VS_REGS, current
scissor, current RB_COLOR_MASK. Wavefronts will pack into fewer
SQ slots (probably 1 or 2), so the scrub is partial. But it should
at least not hang.

If even this hangs, the SQ-slot theory is in trouble.
If it doesn't hang and the cycle persists, partial scrub isn't
enough — would need to find a way to force fresh slot allocation
without the VS_REGS=63 trick (maybe explicit WFI between draws,
or `CP_INVALIDATE_STATE` despite your earlier advice).

### Path 2: Skip Mesa, go to Option C (kernel debugfs)

Per your reply: validate the SRAM theory by adding a debugfs knob
in our existing a2xx kernel module that, when written, calls
`gdsc_disable(domain)` directly via the genpd's `power_off`
callback.

Code shape:
```c
/* Inside a2xx_gpu.c, debugfs node */
static int a2xx_force_collapse_set(void *data, u64 val) {
    struct msm_gpu *gpu = data;
    struct device *dev = &gpu->pdev->dev;
    struct generic_pm_domain *pd = pd_to_genpd(dev->pm_domain);

    if (val) {
        pd->power_off(pd);  /* explicit gdsc_disable */
        msleep(1);
        pd->power_on(pd);   /* explicit gdsc_enable */
    }
    return 0;
}
```

Toggle between submits in our test, see if cycle collapses.

Risk: calling `pd->power_off` on a "running" device might confuse
things. Maybe the right form is to do it inside our own
`a2xx_pm_runtime_suspend` rather than as a synchronous knob.

## Direct asks

1. **For Path 1 (simpler 0070)**: which of the three suspects above
   is most likely the hang root cause? The user's instinct is
   `VS_REGS=63` — should we drop it and accept partial scrub, or is
   there a way to make the SQ rotate slots without claiming high
   GPR count? (E.g., `CP_INVALIDATE_STATE 0x100` per draw to force
   VS context invalidation without rotating the SET_CONSTANT path?)

2. **For Path 2 (debugfs knob)**: best primitive to call from a
   sysfs/debugfs handler? Direct `pd->power_off` callback invoke is
   what comes to mind. Or do you suggest something else (e.g.,
   `genpd_power_off_unused` after dropping device refs)?

3. **Question carrying over from update 17**: would your read on
   the safest test sequence be:
   * Path 1 first (10-min Mesa rebuild, low risk)
   * Then path 2 (kernel patch + rebuild, more invasive but tests
     the actual GDSC theory rather than a software emulation of it)
   * If both fail, conclude the cycle's mechanism is something
     other than SQ slot SRAM and reset our model.

We're particularly interested in your read on whether the empty
scissor or the VS_REGS=63 is the bigger hang risk — both are
plausible but they're independently controllable so we can split
the difference if needed.
