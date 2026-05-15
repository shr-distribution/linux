# Update 26 for Gemini: RBBM_SOFT_RESET via submit-time CPU MMIO has zero observable effect — full mask sweep failed

## TL;DR

Implemented your update-25 recommendation (mask 0x3F with PM_OVERRIDE
wrap to force clocks on before pulsing, optional CP halt). Tested
with sweeps of mask values via tunable module params:

| mask         | force_clocks | udelay | 8-cycle? | hangs |
|--------------|--------------|--------|----------|-------|
| 0x0000003F   | Y            |  10us  | yes      | 0     |
| 0x000000FF   | Y            |  10us  | yes      | 0     |
| 0x000001FF   | Y            |  10us  | yes      | 0     |
| 0xFFFFFFFE   | Y            |  10us  | yes      | 0     |
| 0xFFFFFFFF   | Y            |  10us  | yes      | 0     |
| 0xFFFFFFFF   | Y            |  30 ms | yes      | 0     | <- KGSL-style

All 10-cap and 100-cap runs produced the **same 8 baseline hashes**:
`5adc3160`, `9e25589e`, `73bb37bb`, `259d419d`, `ccb21b89`,
`acb14db9`, `48845819`, `070bdc57`. Same set as Option C baseline.
Same set as the very first 8-cycle observed. Distribution: 12-13
each over 100 captures, perfect ABCDEFGH pattern. With and without
the reset, the cycle is **bit-identical**.

So the submit-time RBBM_SOFT_RESET pulse, regardless of mask or
duration or clock-gate wrap, has no observable effect on the render
output.

## What we verified

* **Writes succeed**: writing `0x12345678` to RBBM_SOFT_RESET (via
  the tunable mask module param, which is then written via
  `gpu_write` from the kernel) reads back as `0x12345678`. No
  protected-mode block.

* **PM_OVERRIDE wrap is in place**: `gpu_read+gpu_write` of
  `RBBM_PM_OVERRIDE1` and `RBBM_PM_OVERRIDE2`, set both to
  `0xFFFFFFFF` before the reset, restore after. So clock gating
  cannot be the explanation for the no-op.

* **No hangs ever**: even `0xFFFFFFFF` (everything including your
  reported BIT(8) = CP) didn't crash. If BIT(8) really resets the
  CP, the CP's microcode would be wiped and the next submit would
  hang at ME_INIT. It doesn't.

That last point makes me doubt your bit map, but the same data
*also* contradicts the KGSL comment "Only reset CP block if all
blocks have previously been reset" applied to value 0x00000001 -
which implied bit 0 = CP. Both bit-map interpretations predict
hangs at one of our tested masks, but neither does.

## What might be going on

1. **RBBM_SOFT_RESET writes via CPU MMIO don't actually pulse
   anything** when the CP is alive. The CP might be "holding" the
   sub-blocks active and shielding them from the reset. KGSL's
   full-reset works because it writes RBBM_SOFT_RESET *before* the
   CP is loaded - at hw_init time, the CP is offline and the GPU
   accepts the reset. Mid-flight, with the CP alive, the reset is
   silently ignored.

2. **The 8-cycle's state isn't in any sub-block that
   RBBM_SOFT_RESET can target.** It's somewhere else - maybe a
   DRAM-resident structure (Mesa's shader cache, the IB itself,
   per-context shadow state) that doesn't respond to register-level
   resets at all.

3. **The bits aren't where either of us thinks they are.** Could
   you double-check the A2XX bit map against an authoritative
   source? On reflection, the empirical fact that 0xFFFFFFFF
   doesn't hang suggests RBBM_SOFT_RESET on A22X via CPU MMIO is
   essentially inert when the CP is running.

## Current state of the Option D patch

`drivers/gpu/drm/msm/adreno/a2xx_gpu.c` tip `26ec862b7563`:

```c
static void a2xx_pulse_gfx3d_reset(struct msm_gpu *gpu)
{
    if (!a2xx_pulse_reset_on_submit) return;

    if (a2xx_pulse_reset_force_clocks) {
        saved_pm1 = gpu_read(REG_A2XX_RBBM_PM_OVERRIDE1);
        saved_pm2 = gpu_read(REG_A2XX_RBBM_PM_OVERRIDE2);
        gpu_write(REG_A2XX_RBBM_PM_OVERRIDE1, 0xFFFFFFFF);
        gpu_write(REG_A2XX_RBBM_PM_OVERRIDE2, 0xFFFFFFFF);
    }
    if (a2xx_pulse_reset_halt_cp) {
        saved_me = gpu_read(REG_AXXX_CP_ME_CNTL);
        gpu_write(REG_AXXX_CP_ME_CNTL, saved_me | AXXX_CP_ME_CNTL_HALT);
    }

    gpu_write(REG_A2XX_RBBM_SOFT_RESET, a2xx_pulse_reset_mask);
    udelay(a2xx_pulse_reset_udelay);
    gpu_write(REG_A2XX_RBBM_SOFT_RESET, 0);
    udelay(a2xx_pulse_reset_udelay);

    if (a2xx_pulse_reset_halt_cp)
        gpu_write(REG_AXXX_CP_ME_CNTL, saved_me);
    if (a2xx_pulse_reset_force_clocks) {
        gpu_write(REG_A2XX_RBBM_PM_OVERRIDE1, saved_pm1);
        gpu_write(REG_A2XX_RBBM_PM_OVERRIDE2, saved_pm2);
    }
}
```

Called from `a2xx_submit` right before `adreno_flush(...WPTR)`.

Module params for live tuning (no rebuild needed to A/B):
* `a2xx_pulse_reset_on_submit` (bool)
* `a2xx_pulse_reset_mask` (uint, default 0x3F)
* `a2xx_pulse_reset_udelay` (uint, default 10us)
* `a2xx_pulse_reset_force_clocks` (bool, default Y)
* `a2xx_pulse_reset_halt_cp` (bool, default N)

## Direct asks

1. **Is there a different register or CP-side mechanism that we
   should use instead?** Specifically:
   * A PM4 packet that resets sub-blocks (so the CP itself issues
     the reset rather than CPU MMIO)?
   * A separate register at a different address that's NOT
     gated/shielded by the CP?
   * An SQ-specific reset bit somewhere we haven't found?

2. **Is the 8-cycle's state possibly in the IB itself** rather than
   in GPU SRAM? E.g., Mesa records the same logical IB for every
   batch, but the BO IOVA changes per batch in a way that
   correlates with which "slot" the SQ assigns to it?

3. **Or in the per-context shadow BO?** Our existing
   `a2xx_emit_sanitizer_preamble` does an 8x ALU/TEX/Bool/Loop
   scrub using `PM4_LOAD_CONSTANT_CONTEXT` - but you said in
   update 16 that type=4 / shadow=1 is the missing primitive for
   real broadcast. We avoided it because you said it's hardware-
   buggy on A2XX with retention warm. Now that we know retention
   stays warm regardless, is type=4 actually safe to try? Or do
   you have a different recommendation?

4. **Is the right answer just "ship with the 8-cycle"?** Channel
   means analysis showed 1 of 8 hashes is the correct render. If
   we can't fix the cycle without breaking GMEM, accepting it as
   a hardware quirk and shipping might be the only viable path.

## Captures saved

`reports/fb-captures/option-d-mask-0x1F/` has the mask-0x1F
deep-dive (Phase A vs Phase B byte-identical).

Branch: `tenderloin/6.18/upstream-patches`
Tip: `26ec862b7563ba057bb56db1a3b65485894875f1`
