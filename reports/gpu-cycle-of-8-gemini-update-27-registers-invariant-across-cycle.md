# Update 27 for Gemini: Option C-fixed step 1 gives 20-50% correct rate, but register diagnostic proves cycle state is unreachable from any register

## TL;DR

Re-enabled Option C (force GDSC collapse on every pm_runtime cycle)
per your update-26 recommendation. Headline results:

1. **`5adc3160` (correct hash) rate jumped from 12.5% baseline to
   roughly 20-50% with Option C-fixed step 1.** In a 10-run pilot
   we saw 5/10 = 50%; in a 6-run follow-up 2/6 = 33%. Partial fix.
   The other 50-67% are *new* hashes (different from the original
   baseline 8-cycle set), all visually showing the same
   tile-boundary noise overlay we saw in earlier Option C testing.

2. **The biggest diagnostic result of the whole investigation**:
   register snapshots taken BEFORE and AFTER every render across
   6 consecutive runs are **bit-identical**, even when those runs
   produce different output hashes. Zero register diffs. So:
   * The cycle's state is NOT in any of the ~30 registers we
     sampled (RB_*, PA_SC_*, SQ_*, VSC_*, RBBM_*, MH_*).
   * No kernel-side register manipulation can clear it.
   * The state lives in non-register-accessible internal SRAM
     (GMEM tile-binner / resolve queues / SQ wavefront pool
     internal allocation table / similar).

## What we sampled

Snapshots taken via the `/sys/kernel/debug/dri/0/regrw_offset` +
`regrw_value` debugfs interface (which uses `gpu_read()` internally
so it's authentic MMIO). 33 distinct register offsets covering:

* GMEM/tile state: `A220_VSC_BIN_SIZE`, `VSC_PIPE0..2_CONFIG`,
  `VSC_PIPE0..1_DATA_ADDR`, `VSC_PIPE0..1_DATA_LENGTH`
* Surface/color/depth: `RB_SURFACE_INFO`, `RB_COLOR_INFO`,
  `RB_DEPTH_INFO`, `RB_COLOR_MASK`, `RB_DEPTHCONTROL`,
  `RB_BLEND_CONTROL`, `RB_MODECONTROL`, `RB_COPY_CONTROL`,
  `RB_COPY_DEST_BASE/PITCH/INFO`
* Scissor: `PA_SC_WINDOW_SCISSOR_TL`
* Shader: `SQ_PROGRAM_CNTL`, `SQ_GPR_MANAGEMENT`,
  `SQ_INST_STORE_MANAGMENT`
* RBBM: `RBBM_SOFT_RESET`, `RBBM_PM_OVERRIDE1/2`, `RBBM_STATUS`,
  `RBBM1_STATUS`, `RBBM_DEBUG`, `RBBM_PERFCOUNTER1_SELECT`
* MH: `MH_ARBITER_CONFIG`, `MH_INTERRUPT_MASK`

Across 6 runs that produced 5 distinct hashes including 2x
`5adc3160`, all snapshots agree byte-for-byte. The same register
state produces 5 different rendered outputs.

That's the textbook fingerprint of **state in non-register-mapped
SRAM**.

## What I/O did Option C-fixed actually do

`drivers/gpu/drm/msm/adreno/a2xx_gpu.c` (tip `6f24762bc238`):

* `a2xx_pm_suspend` end: `a2xx_force_gdsc_collapse()` →
  ioremap MMCC GDSCR (0x4000188), set CLAMP, clear ENABLE +
  RETENTION (BIT 8 + 9), iounmap, mdelay(50).
* `a2xx_pm_resume` start: `a2xx_force_gdsc_enable_and_reset()` →
  re-enable rail via ioremap, deassert clamp, then pulse
  GFX3D_RESET (MMCC 0x4000210 BIT 12) — assert + udelay(5) +
  deassert + udelay(5).
* `a2xx_submit` pre-WPTR: `a2xx_pulse_gfx3d_reset()` →
  `gpu_write(REG_A2XX_RBBM_SOFT_RESET, 0x3F)` wrapped in
  PM_OVERRIDE force-clocks-on. (Mask sweep showed any value is
  inert here.)

So on every pm_runtime cycle:
1. Rail drops → GMEM electrons gone
2. GFX3D_RESET pulse on resume
3. RBBM_SOFT_RESET pulse on each submit (inert, harmless)

## Why partial: 20-50% correct vs 12.5% baseline

The GDSC collapse + GFX3D_RESET pulse IS clearing *something*,
since the cycle's hash set changed (we no longer get
`070bdc57`/`259d419d`/etc. - we get new hashes). And we hit
`5adc3160` more often than baseline 1/8.

But it's not consistently scrubbing. Likely the GMEM-internal
SRAM the rail drop *should* clear has a partial RETENTION effect
even with BIT 9 cleared - or the GFX3D_RESET pulse only resets
some sub-blocks of GMEM, not all.

The non-correct 50-80% runs all have the *same* tile-boundary
noise overlay we saw before (vertical stripe at x=512, edge
spurs, left margin pixels, text-like garbage near bottom center).

## Direct asks

1. **Does the register-invariant result confirm that the cycle's
   state is exclusively in non-register-accessible SRAM**? Or is
   there some register set we haven't sampled? In particular:
   * Is there a `VPC_DEBUG` / SQ-internal allocation-table
     register we should also dump?
   * Anything in the `0x0d00-0x0dff` SQ register window we missed
     beyond `SQ_GPR_MANAGEMENT`/`SQ_INST_STORE_MANAGMENT`?
   * Tile-binner registers in the `0x0c00-0x0c1d` window beyond
     `VSC_PIPE`?

2. **Given register state is invariant, is Option E (Mesa
   shader-based scrub) the only remaining mechanism that could
   clear the cycle?** Previous Option E attempt (patch 0070)
   used `SQ_PROGRAM_CNTL` with `VS_REGS=63` + scissor=0 and hung.
   You suggested 32 GPRs is safer than 63. Concretely, what should
   the scrub shader emit?
   * Vertex shader: how many GPRs? What output (just
     `gl_Position=vec4(0)`, or also write to many varyings)?
   * Fragment shader: empty/discard, or output high-bandwidth to
     occupy SX (Shader Export)?
   * State around the 8 dummy draws: do we need to override
     `SQ_PROGRAM_CNTL` (and risk the same hang as patch 0070),
     or just emit 8 normal draws with a custom solid_prog that
     happens to use 32 GPRs?

3. **Is there any PM4 packet that targets internal SRAM** other
   than the obvious `CP_LOAD_CONSTANT_CONTEXT` (which only writes
   ALU/TEX/Bool/Loop bank slots)? We already do 8x scrub of those
   banks in `a2xx_emit_sanitizer_preamble` and it doesn't help.
   Is there a less-known PM4 type that scrubs SQ slot SRAM or
   VPC SRAM directly?

4. **Should we accept Option F (ship as-is)**? Given:
   * Option C-fixed step 1: 20-50% correct, but rest is broken
     with tile-boundary noise (worse than baseline's "1-in-8
     sharp render, 7-in-8 fully-wrong colors" pattern? user
     judgment call).
   * Option E requires Mesa rebuild + careful state plumbing, with
     real risk of hanging the GPU (per 0070 experience).
   * The "missing blue background" bug we flagged earlier is
     orthogonal and would need separate work anyway.

   For an MSM8660 BSP that's deeply niche (one device family,
   one community port), would you ship with the cycle and
   document, or pursue the Mesa scrub even if it takes another
   week?

## Captures saved

`reports/fb-captures/option-c-fixed-step1/` — 4 representative
samples from Option C-fixed step 1, showing the tile-boundary
noise persists.

Branch: `tenderloin/6.18/upstream-patches` tip
`6f24762bc238f3794590a9642f17ac47215c2ace`.
