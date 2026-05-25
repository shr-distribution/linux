# Adreno 220 (a22x) "period-8" render cycle — summary for external review (2026-05-23)

## TL;DR / the question
On mainline Linux 6.18 + Mesa freedreno, the HP TouchPad's Adreno 220 GPU renders a
**deterministic period-8 cycle**: an identical, trivial GLES2 draw issued N times produces
**8 distinct framebuffers that repeat every 8 submits, of which exactly 1 in 8 is correct**.
We have narrowed it precisely (below) to a **per-tile vertex-parameter slot rotation**: the
fragment stage reads each vertex's interpolated varying from a parameter-cache slot offset by a
**per-submit rotating phase**, and one offset hits an **uninitialised (zero) slot**. It is
**GPU-global** (persists across separate processes/GL contexts). Every per-draw register value
we can match to the working webOS/KGSL stack has been tried and **does not fix it**. The only
structural difference left is that webOS/KGSL does a **full HW context save/restore
(PM4_LOAD_CONSTANT_CONTEXT)** per context switch, which **hangs** mainline freedreno on a2xx.

**Question for you:** On Adreno 2xx (Yamato/Leia), what hardware state determines the
**parameter-cache read-slot index / the vertex→fragment varying slot mapping**, such that it
would rotate per-submit with stale/uninitialised contents? Is there a param-cache base/pointer
or VS-export-base register we could reset per draw, or is the rotating state only resettable via
the CP context-shadow mechanism (PM4_LOAD_CONSTANT_CONTEXT / PM4_CONTEXT_UPDATE)? If the latter,
what is the correct minimal setup (shadow buffer alloc / alignment / VA) so it doesn't hang?

## Hardware / software
- SoC: APQ8060 (== MSM8660 w/o modem), dual Scorpion, **Adreno 220 "Leia" REV470**, chip_id 0x02020000, 512KB GMEM.
- Broken: mainline **Linux 6.18** (drm/msm a2xx) + **Mesa freedreno** (26.1-devel, gallium fd2/ir2).
- Working reference: legacy **webOS 2.6.35** + **KGSL** ("Yamato"-era) + proprietary libGLESv2 — renders 100% correct on the same silicon.

## The test rig (objective, deterministic)
`gl-capture`: off-screen GLES2 render to a 1024x768 FBO on /dev/dri/renderD128 (surfaceless,
no compositor). Fixed scene: clear to (0.1,0.2,0.3), one triangle, per-vertex colours
(bottom-left RED, bottom-right GREEN, top BLUE), smooth `varying vec3 v_color`, FS =
`gl_FragColor = vec4(v_color,1.0)`. glReadPixels → raw RGBA → md5. Run N times, tally unique
hashes. **Correct render hashes to `5adc3160` (clean RGB gradient).** freedreno uses GMEM
tiling: 1024x768 → 2x3 grid of ~512x256 tiles.

## Precise characterization of the bug (from framebuffer visualisation)
Running gl-capture x100 → 6-8 unique hashes in a clean **period-8** sequence; the correct
`5adc3160` appears **exactly 1/8** of the time (≈12.5%), **invariant** across every change we
have made. The 7 "broken" frames are NOT random garbage — they are **vertex-colour rotations**:
- the three vertex colours are read shifted by one slot (blue→green→red→**black/zero**);
- the recurring **black** = a vertex reading a **zero/uninitialised** param-cache slot;
- the rotation is applied **per GMEM tile-row** (clean horizontal tile boundaries between a
  "correct" row and a "rotated" row); the most common frame is a **whole-triangle** rotation
  (all rows shifted identically, perfectly smooth, no tile boundary).
- With `SQ_INTERPOLATOR_CNTL=0` (flat shading) the same bug shows as **each GMEM tile filled
  with one solid vertex colour**, the colour varying per tile/submit — i.e. the flat/provoking
  value is read from the rotating slot.

**It is GPU-global / per-submit, not per-context:** each gl-capture is a brand-new process and
GL context, yet the period-8 sequence continues seamlessly across them. So a GPU-internal state
advances once per submit (mod 8) and selects which slot-rotation is used; 1 of 8 phases is
"aligned" (correct), the other 7 are rotated/stale.

## What we have RULED OUT (all tested on-device with the gl-capture x16/x100 hash test)
Every one of these only **reshuffles which broken frames appear** — none changes the 1/8 correct
rate or the period-8 structure:
1. **VSC hardware tile-binner config** (program VSC_BIN_SIZE + 8 VSC_PIPE config/addr/len with
   256K BOs + VSC_SIZE_ADDRESS, matching a captured webOS config; plain DRAW_INDX, no binning
   pass). → This **DID fix tile COVERAGE** (pre-fix the GPU dropped whole tiles to black /
   missing geometry; after, full coverage every frame) but left the colour rotation. Big visible
   improvement, not a period-8 fix.
2. **VGT_VERTEX_REUSE_BLOCK_CNTL + VGT_OUT_DEALLOC_CNTL** set per tile to the EXACT values the
   working webOS stack writes per draw (captured: **0x28f** and **0x0**). No fix (made the
   rotations more coherent / whole-row).
3. **SQ_GPR_MANAGEMENT** swept at runtime (kernel module param) over 0x40400 (default),
   0x7f010 (webOS binning value), 0x40401 (REG_DYNAMIC), 0x7f07f (max): **identical 8-frame
   cycle** for all non-zero values; 0x0 = degenerate (shader can't run, blank). GPR/param-bank
   sizing is NOT the lever.
4. **SQ_INTERPOLATOR_CNTL=0** (webOS writes 0 for its compositor shaders; freedreno hardcodes
   0xffffffff): this just **disables smooth interpolation → flat shading**. webOS's 0 is
   per-shader (its compositor uses flat/texture), not universal.
5. **Heavy pipeline drain** (CACHE_FLUSH_AND_INV_EVENT + WAIT_FOR_IDLE) before every tile's
   draws: **zero effect**. Important: the stale state **survives a full pipeline idle + cache
   flush/invalidate**, so it lives in **persistent SRAM (parameter cache / GPR file)**, not in
   the pipeline or the texture/shader caches.
6. Kernel-side (older, all falsified): RBBM_SOFT_RESET pulses (any mask), GFX3D GDSC
   collapse/retention toggling, CP_SCRATCH pinning, MH_ARBITER IN_FLIGHT_LIMIT changes,
   SQ constant-bank scrubs, "sanitizer preamble".

## The one structural difference that remains
The working **KGSL** (kgsl_drawctxt.c) does a full **hardware context save/restore on every
context switch**: `PM4_CONTEXT_UPDATE` + `PM4_LOAD_CONSTANT_CONTEXT` against shadow buffers,
shadowing **46 register ranges** (incl. SQ_PROGRAM_CNTL..SQ_WRAPPING_1) + **ALU/TEX/BOOL/LOOP
constant** shadows + **shader-instruction** shadows, with implicit pipeline drains. This
re-initialises the **complete** context (including internal state explicit register writes can't
reach) per switch. **Mainline freedreno does none of this** (it emits explicit state only), and
a prior attempt to use `PM4_LOAD_CONSTANT_CONTEXT` in freedreno **hung the a2xx** (suspected
incorrect shadow-buffer alloc/alignment/VA).

Per-draw register diff (webOS capture vs freedreno) shows webOS writes essentially the **same
standard register set** (RB_*, PA_SC_*, PA_SU_SC_MODE_CNTL incl provoking-vertex, PA_CL_VTE_CNTL,
SQ_*, VGT_*) — no exotic "param-cache base/pointer" register that freedreno is missing. That
points to the rotating slot state being **internal** and only resettable via the CP
context-shadow mechanism.

## Our working hypothesis
The a2xx parameter cache / SQ holds the per-vertex exported params in a small set of slots/banks
(circumstantially 8). A GPU-global index advances per submit; freedreno never reloads/realigns
this internal state, so 7 of 8 submits read the params from a rotated slot (one rotation lands on
an uninitialised → zero slot). webOS's per-context `LOAD_CONSTANT_CONTEXT` is what realigns it
(by fully reloading constants/params/registers from the shadow). Hence: not a register *value*,
but the missing full-context **reload**.

## Specific things we'd value your input on
1. On Adreno 2xx, what register / mechanism controls the **parameter-cache read-slot index** or
   the **VS-export-to-PS-interpolant slot mapping**? Is there a per-context base/pointer that
   freedreno could explicitly reset per draw (cheaper than the full context shadow)?
2. Is "8" meaningful here (8 SQ wavefront contexts? 8 param-cache banks? something keyed off the
   submit/context counter)? What internal a2xx resource is 8-deep and selected round-robin per
   submit?
3. Why would `PM4_LOAD_CONSTANT_CONTEXT` hang on a2xx in freedreno but work in KGSL — what is the
   exact required shadow-buffer layout (size, 16KB alignment noted in KGSL, GPU VA, the
   CONTEXT_UPDATE prologue) to make it safe?
4. Is there a simpler init (e.g. a one-time PM4 sequence or a register write at GPU init) that
   primes/aligns all 8 internal slots so every submit reads the correct slot, without the full
   per-switch shadow restore?

## Artifacts
- gl-capture tool + source: linux-6.18-tenderloin/tools/gl-capture/
- Current broken-frame PNGs (smooth, vertex-rotation): /tmp/p8frames/vgt2/ (v4=correct,
  v3=dominant whole-triangle rotation, v1/v2/v6/v7=per-tile-row rotations)
- Flat-shading PNGs (per-tile solid colours): /tmp/p8frames/interp0/
- Pre-cleanup coverage-failure mosaic: reports/fb-captures/perturbed-7-wrong-hashes/mosaic_8way.png
- webOS cmdstream capture (per-draw register payloads): reports/webos-binning-capture-*.log
- KGSL context save/restore source: webos-linux-kernel-touchpad/drivers/gpu/msm/kgsl_drawctxt.c
- Mesa patch series (0001-0007) + kernel: branch herrie/a22x-clean / shr-github tenderloin/6.18/upstream-patches
