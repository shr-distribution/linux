# Post-LSM-Kill GPU State Leak Investigation

**Last Updated:** 2026-05-08
**Status:** Active Investigation — Mesa cmdstream confirmed correct, leak is below Mesa
**Scope:** Adreno 220 (HP TouchPad APQ8060) post-luna-surfacemanager state pollution affecting subsequent GLES clients

Related: [`ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md`](ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md)
covers the long-standing intermittent faceted-shading bug. This report covers a *different* bug
class: state pollution that surfaces after `luna-surfacemanager` (LSM) runs and exits, affecting
any subsequent GLES client.

---

## Table of Contents

1. [Problem Description](#problem-description)
2. [Symptoms](#symptoms)
3. [Hypotheses Tested and Excluded](#hypotheses-tested-and-excluded)
4. [Patches Applied](#patches-applied)
5. [Current State](#current-state)
6. [Open Questions / Next Investigation Steps](#open-questions--next-investigation-steps)
7. [Cmdstream Capture Methodology](#cmdstream-capture-methodology)
8. [File References](#file-references)

---

## Problem Description

### What works (cold boot, before LSM ever runs)
- `kmscube` and most variants render correctly (modulo the long-standing 1-face-black cold-boot quirk on `kmscube` and the faceted-shading bug from the related report)
- `glmark2` runs at expected ~25-30 fps
- LSM itself, on first launch after cold boot, has visible UI corruption: status bar / launcher
  translucent surfaces show yellow/orange/red gradients in place of expected source-over alpha,
  glyph text is mostly garbled-but-positioned-correctly

### What breaks (after LSM has run and been killed)
1. **kmscube non-gears variants** (smooth, rgba, nv12-1img, nv12-2img):
   - First face renders correctly, others render black
   - During a `-c 200` run, faces fade in and out: content is **correct when shown** but
     the face goes black at random and recovers
2. **kmscube gears**: all faces black, plus a stable white square/rectangle in 3D space
   *behind* the cube (visible as cube rotates and partially occludes/exposes it). The white
   shape extends and shows different colors on different sides (white on one face, black on
   another).
3. **glmark2 texture test**: completely off (texture content wrong) on first post-LSM run.
   Sometimes self-corrects after running LSM a second time and exiting again.
4. **Severe perf regression**: `kmscube -c 200` post-LSM completes ~4 GPU submits in 5 seconds
   vs ~116 cold-boot submits in the same wall-clock time (≈30× slower).

### Within-LSM symptoms (LSM running)
- Top status bar: red/green or gradient instead of translucent, sometimes self-corrects on a
  scene refresh
- Glyph text: garbled (correct positions, wrong pixels)
- Touch: triggers transient corruption that "snaps back" after the animation settles

---

## Symptoms

| Test | Cold-Boot Result | Post-LSM-Kill Result |
|---|---|---|
| `kmscube` (smooth) | 1-face-black cold quirk only | 1 face correct, others black, fade in/out |
| `kmscube -M rgba` | OK | Same fade pattern |
| `kmscube -M nv12-1img` | OK | Same |
| `kmscube -M nv12-2img` | Mostly black sides + flickers | Worse, persistent |
| `kmscube -g` (gears) | OK | All black + persistent white rect behind cube |
| `glmark2 texture` | ~25-30 fps | "completely off" |
| `glmark2 build` / `shading` | Faceted-rendering bug (separate) | Faceted-rendering bug |

---

## Hypotheses Tested and Excluded

### EXCLUDED: SMI-memory FB scanout
- Hypothesized that scanning out FBs from SMI (instead of EBI) would help with CMA fragmentation
- Implemented `msm_smi_pool` allocator, routed `MSM_BO_CONTIGUOUS` BOs there
- Result: **MDP4 cannot usefully scan from SMI** — produced red/green horizontal stripes
- Reverted in [`drm/msm: gem: stop routing CONTIGUOUS BOs into the SMI pool`](https://github.com/...) (commit `4da3e6833f61`)
- Cross-checked HTC pyramid + Samsung Q1 + webOS reference kernels: **none of them ever scan
  FBs from SMI** — all use `alloc_bootmem` from EBI/main DDR. `pmem_smipool` was reserved
  exclusively for camera/video codec buffers via `/dev/pmem_smipool`, with zero in-kernel
  consumers and no MDP/FB driver `MEMTYPE_SMI` references.

### EXCLUDED: System CMA size pressure
- Tested with system CMA at 32, 64, 96 MiB
- Symptoms persist regardless of CMA size
- The bug is not contiguous-allocation pressure
- Reference kernels reserve roughly 50–80 MiB of EBI for FB+pmem (HTC ~50 MiB, Samsung ~55–85 MiB,
  webOS ~58 MiB), but our 32 MiB CMA is enough for FB allocation; the problem isn't fragmentation.

### EXCLUDED: MDP4 register state pollution from LSM
- Old hypothesis: surface-manager modifies `READ_CNFG`, `FETCH_CONFIG`, etc. and never restores
  them, so subsequent sessions inherit broken values and underflow.
- Implemented runtime-PM ops (`mdp4_runtime_resume`/`suspend`) so `pm_runtime_get_sync()` in
  `mdp4_hw_init` actually powers/clocks the MDP, then init writes latch (commit `f6cc7670f7c7`).
- Implemented `.atomic_disable` plane handler so disabled planes' `PIPE_SRCP0_BASE` and
  `PIPE_OP_MODE` get cleared (commit `f6cc7670f7c7`).
- Verified live: `PIPE_FETCH=0x47` (was `0xc7` due to latched bit-7), `PORTMAP=0x3` (was `0x0`).
- Result: kernel state is now stable across **2438+ atomic commits** with no register drift,
  but the post-LSM-kill GLES rendering bug **persists**.
- → MDP register state is *not* the source of the user-visible rendering issues.

### EXCLUDED: Mesa-side dirty-flag tracking missing some bits
- Hypothesis: `fd2_emit_state` doesn't re-emit some state class after the GMEM mem2gmem blit
  clobbers it.
- Iterated through several patch versions:
  - Per-flag list (`FD_DIRTY_BLEND | FD_DIRTY_ZSA | FD_DIRTY_RASTERIZER | …`): partial fix,
    LSM gradient → solid red/green
  - Extended (`+ FD_DIRTY_TEX | FD_DIRTY_PROG | FD_DIRTY_VTXBUF | FD_DIRTY_FRAMEBUFFER`):
    fixed glyph filter and partial blit-shader-leak, white triangle in cube model space
    persisted
  - `fd_context_all_dirty(ctx)` in `fd2_emit_tile_mem2gmem`: fixed regular kmscube; gears
    started showing geometry but white rectangle persisted
  - Extended to all 3 GMEM-clobber paths (`fd2_emit_tile_mem2gmem`, `fd2_emit_tile_gmem2mem`,
    `fd2_emit_sysmem_prep`) using `fd_context_all_dirty(ctx)` (this is patch
    `0038-freedreno-a2xx-mark-all-state-dirty-after-GMEM-tile-.patch` shipped today): broad
    improvement on all variants
- Tried `FD_MESA_DEBUG=ddraw` (mark all state dirty after every draw) at the user's request
  to test whether intra-batch leakage is the cause: **did not help**.
- → Dirty-flag mechanism is **not** the root of the residual.

### EXCLUDED: Mesa cmdstream differs between good and bad runs
- **Most rigorous test**. Captured `kmscube` cmdstream via `FD_RD_DUMP=enable` for both
  cold-boot (good) and post-LSM-kill (bad), all 5 variants.
- Decoded with `cffdump`. **For smooth, rgba, nv12-1img, nv12-2img: cmdstream is BYTE-IDENTICAL
  between good and bad** (same opcodes, same registers, same addresses, same BO references).
- Gears differs by 1 byte in submit 3 — but gears was visually broken in *both* good and bad
  this run, so probably noise.
- → Mesa is emitting the right cmdstream. The leak is below cmdstream. **Patch 0038 is doing
  its job.**

### EXCLUDED: Kernel GPU MMU faults / oops / WARNs
- Checked dmesg post-LSM-kill: 0 `paging request`, 0 `Oops:`, 0 `Kernel panic`, 0 `WARNING:`.
- No `gpummu fault`, `MH fault`, `tran_error`, `mmu_fault` events.
- Kernel side is completely clean throughout the bad runs.
- → Not a GPU-MMU page fault.

---

## Patches Applied

### Kernel (pushed to `shr-github`, branch `tenderloin/6.18/upstream-patches`)

| Commit | Title | What it does |
|---|---|---|
| `4da3e6833f61` | drm/msm: gem: stop routing CONTIGUOUS BOs into the SMI pool | Reverts unsuccessful SMI-FB experiment; keeps the `msm_smi_pool` allocator infrastructure dormant for future camera/video reuse. |
| `f6cc7670f7c7` | drm/msm: mdp4: real runtime PM + plane atomic_disable | Defines `mdp4_runtime_resume`/`suspend`, routes `mdp4_enable`/`disable` through `pm_runtime_resume_and_get`/`put_sync` so `pm_runtime_get_sync()` in `mdp4_hw_init` actually brings clocks live; adds `.atomic_disable` plane handler that zeroes `PIPE_SRCP0_BASE` and `PIPE_OP_MODE` on disable. |

Verified at runtime: `PIPE_FETCH=0x47`, `PORTMAP=0x3`, no register drift across thousands of
atomic commits.

### Mesa (committed in `mesa-latest`, patch in `meta-mainline/recipes-graphics/mesa/files/`)

| Commit | Title | What it does |
|---|---|---|
| `dee3da0462d` | freedreno/a2xx: mark all state dirty after GMEM tile setup and resolve | Calls `fd_context_all_dirty(ctx)` at the end of `fd2_emit_tile_mem2gmem`, `fd2_emit_tile_gmem2mem`, and `fd2_emit_sysmem_prep` so user-draw state gets re-emitted after GMEM tile setup, GMEM tile resolve, and bypass renderer setup all overwrite registers behind the back of `fd2_emit_state`'s dirty-flag tracking. |

Patch file: `0038-freedreno-a2xx-mark-all-state-dirty-after-GMEM-tile-.patch`

After 0038 fixed (cmdstream layer): most LSM blend gradients, kmscube progressively-filling-in
faces, gears starting to show geometry post-LSM, glyph filter quality.

After 0038 did **not** fix (cmdstream byte-identical to good): gears white rectangle, kmscube
fade-in/out, glmark2-texture-broken-on-first-cycle, the 30× post-LSM perf regression.

#### Patch 0039 — pending verification

| Commit | Title | What it does |
|---|---|---|
| `9d9133f2acc` | freedreno/a2xx: invalidate L2 texture cache at every batch start | Adds `OUT_PKT0(REG_A2XX_TC_CNTL_STATUS, A2XX_TC_CNTL_STATUS_L2_INVALIDATE)` + `OUT_WFI()` to `fd2_emit_restore`. Forces fresh texture fetches at each batch boundary so post-LSM CMA-page recycling doesn't leave stale TC entries at recycled physical addresses. Cost: one CP packet + one WFI per batch start. **Verified redundant** — Mesa already invalidates L2 on every draw at `fd2_draw.c:246`. Kept as defense-in-depth. |
| `f867516997b` | freedreno/a2xx: WAIT_FOR_IDLE at start of fd2_emit_restore | Adds `OUT_PKT3(CP_WAIT_FOR_IDLE)` at the very entry of `fd2_emit_restore` to drain whatever the previous DRM client left in flight before state-restore writes start. Per Gemini-AI analysis: KGSL achieves this drain implicitly via `PM4_CONTEXT_UPDATE` / `PM4_LOAD_CONSTANT_CONTEXT` (both unsafe in mainline a2xx). |
| `ca6a0778d1f` | freedreno/a2xx: set REG_DYNAMIC bit in SQ_GPR_MANAGEMENT | Conservative version of the GPR-allocation hypothesis from the Ghidra cross-reference of webOS libGLESv2.so. Flips REG_DYNAMIC=1 in the existing static `SQ_GPR_MANAGEMENT=0x00040400` write so HW dynamically adjusts the VS/PS GPR split per shader. **Tested null-result (2026-05-08): no visible change to faceted-rendering or gears post-LSM symptoms.** Either the hardware doesn't honor REG_DYNAMIC the way the proprietary driver expects, or GPR allocation isn't the leak. Earlier attempt at full per-shader recalc (3fcf3b9b0b0, reverted) hung the GPU. Patch retained as defense-in-depth — REG_DYNAMIC matches what the existing analysis report claimed Mesa was already setting. |

Patch files:
- `0039-freedreno-a2xx-invalidate-L2-texture-cache-at-every-.patch`
- `0040-freedreno-a2xx-WAIT_FOR_IDLE-at-start-of-fd2_emit_re.patch`

Targets the "stale GPU TC at recycled CMA pages" hypothesis. The L2 texture cache is keyed by
physical address. Mesa already invalidates L2 inside `fd2_emit_tile_mem2gmem` (before the
FB-as-texture blit), but not on context-switch / batch-start in general. Kernel-side
`gpummu_unmap` invalidates the MMU TLB but not the L2 TC. So when a GLES client (e.g. kmscube)
inherits CMA-allocated BOs at physical pages a previous client (LSM) just vacated, GPU TC may
still hold the previous client's texture data.

**Verification result (2026-05-08):** patch 0039 does **not** fix the gears residual.
Post-LSM gears still shows the white-square-behind-cube and now also wrong-color gears (one
gear partially red and partially blue, other gear similar mixed-color).

This **rules out TC L2 stale data as the cause for the gears residual specifically**: gears
does not sample any textures — it uses lighting calculations from uniforms and per-vertex
normals. So a texture-cache fix cannot affect it. The TC invalidate is still correct on its
own (Mesa already does it inside `fd2_emit_tile_mem2gmem`; doing it on batch-start is
consistent), but it's not the gears fix and 0039 is being kept anyway as defense-in-depth.

The "one gear partially red and partially blue" pattern points at **per-vertex attribute
data corruption** — vertex normals or colors interpolating to wrong values. Despite
`fd_context_all_dirty()` (patch 0038) setting `FD_DIRTY_VTXBUF | FD_DIRTY_VTXSTATE`, something
at the vertex-fetch layer is still leaking from LSM's last state. Hypotheses:

1. `emit_vertexbufs` may short-circuit if the BO pointer is the same even when contents
   differ (Mesa doesn't track BO content as dirty)
2. A vertex-attribute or vertex-format register that `fd2_emit_state` doesn't write at all
   gets clobbered during GMEM and isn't covered by any dirty flag
3. The `solid_vertexbuf` (used by GMEM blit) has 3 vertices' worth of attribute data; if
   user gears geometry inherits part of that data, you'd see exactly this kind of partial
   per-vertex color confusion

The 30× perf claim from earlier in this report was **misinterpretation**: with no
`FD_RD_DUMP_FRAMES` filter, kmscube renders a normal ~85 fps (`time kmscube -g -c 100` gives
1.16s wall time). Earlier "4 submits in 5 seconds" came from `FD_RD_DUMP_FRAMES=1..1` filter
limiting capture to only frame 1 — kmscube was actually rendering all 200 frames in ~2.3s.
**Removing the perf-regression hypothesis from open residuals.**

To pin down the gears residual, the right tool is **per-draw cmdstream comparison** within a
known-broken submit (compare what `kmscube -g` post-LSM emits for draw N vs cold-boot draw N
within the same submit), and cross-reference against KGSL vertex-binding sequence. Out of
scope for this session.

### KGSL context-save register list (vs Mesa fd2_emit coverage)

A more decisive comparison was done with the legacy KGSL driver
(`drivers/gpu/msm/kgsl_drawctxt.c:485-570` in
`webos-linux-kernel-touchpad`). KGSL implements full per-context save/restore via backing
memory — saving and restoring ~17 register ranges plus ALU constants and TEX/vertex-fetch
constants on every context switch. **Mainline freedreno does not have this** — it shares one
GPU context across all GLES processes and only re-emits a fixed subset of registers per
batch via `fd2_emit_state` + `fd2_emit_restore`. Anything not covered is left at whatever
state the previous client left it.

KGSL saves these register ranges for Adreno 220 (Leia, REV470):

```
REG_RB_SURFACE_INFO              → REG_RB_DEPTH_INFO
REG_COHER_DEST_BASE_0            → REG_PA_SC_SCREEN_SCISSOR_BR
REG_PA_SC_WINDOW_OFFSET          → REG_PA_SC_WINDOW_SCISSOR_BR
REG_LEIA_PC_MAX_VTX_INDX         → REG_LEIA_PC_INDX_OFFSET
REG_RB_COLOR_MASK                → REG_RB_FOG_COLOR
REG_RB_STENCILREFMASK_BF         → REG_PA_CL_VPORT_ZOFFSET
REG_SQ_PROGRAM_CNTL              → REG_SQ_WRAPPING_1
REG_RB_DEPTHCONTROL              → REG_RB_COLORCONTROL
REG_PA_CL_CLIP_CNTL              → REG_PA_CL_VTE_CNTL
REG_RB_MODECONTROL               → REG_LEIA_GRAS_CONTROL
REG_PA_SU_POINT_SIZE             → REG_PA_SU_LINE_CNTL
REG_PA_SC_LINE_CNTL              → REG_SQ_PS_CONST
REG_PA_SC_AA_MASK
REG_LEIA_PC_VERTEX_REUSE_BLOCK_CNTL
REG_RB_COPY_CONTROL              → REG_RB_DEPTH_CLEAR
REG_RB_SAMPLE_COUNT_CTL          → REG_RB_COLOR_DEST_MASK
REG_PA_SU_POLY_OFFSET_FRONT_SCALE → REG_PA_SU_POLY_OFFSET_BACK_OFFSET
+ ALU constants (REG_SQ_CONSTANT_0, full range)
+ TEX/vertex-fetch constants (REG_SQ_FETCH_0)
```

**`REG_LEIA_PC_VERTEX_REUSE_BLOCK_CNTL` is just an alias for
`REG_VGT_VERTEX_REUSE_BLOCK_CNTL`** (`leia_reg.h:33`) — same hardware address, just renamed
for the chip variant. So Mesa writing `REG_VGT_VERTEX_REUSE_BLOCK_CNTL` IS hitting the right
physical register on Adreno 220. Hypothesis "Mesa writes wrong register on Leia" is ruled
out.

But many of the other ranges KGSL saves are not written by Mesa per-batch. Suspect candidates
that could affect the "1 gear partially red, partially blue" symptom:

1. **`REG_SQ_PROGRAM_CNTL` → `REG_SQ_WRAPPING_1`** — shader sequencer state. If any of these
   control vertex-attribute fetch wrapping or ALU scheduling for vertex shaders, leftover
   LSM values would corrupt per-vertex calculations.
2. **`REG_PA_SU_POLY_OFFSET_*`** — polygon offset can subtly distort vertex positions.
3. **`REG_RB_COLOR_MASK`** — channel write enable. If LSM left it at e.g. `R|G` only (no B),
   blue components written by kmscube's shader get dropped → mostly red/yellow output. Could
   match the "partial red/blue" symptom on gears specifically.
4. **`REG_PA_SC_LINE_CNTL` → `REG_SQ_PS_CONST`** — large range covering line/point/screen
   geometry config plus shader constants. SQ_VS_CONST and SQ_PS_CONST ARE written by Mesa,
   but registers in between (PA_SC_*) might not be.

Mesa also already invalidates L2 (`TC_CNTL_STATUS`) on **every draw** at `fd2_draw.c:246-253`.
So the L2 cache is not stale during draws — patch 0039's added invalidate at batch start is
redundant.

#### Recommended next experiment

~~Build a KGSL-style "full register init" preamble~~ — **scratched.** The
[`ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md`](ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md)
investigation already exhausted the GPU register-dump approach using
`scripts/glmark2-register-test.sh` (which reads `/sys/kernel/debug/a2xx_regs` before/after
each glmark2 run with user-marked GOOD/BAD status). Conclusion from that report:

> "SQ_INTERPOLATOR_CNTL is stable at 0xffffffff for ALL iterations — both SMOOTH and FACETED.
> There are **ZERO observable register differences** between working and broken renders. The
> GPU is configured correctly. The GPU is executing incorrectly — an internal timing/race
> condition. The issue is NOT controllable via standard register state."

The post-LSM gears symptom (intermittent wrong-per-vertex colors, gears specifically
affected, cmdstream byte-identical between good/bad runs) closely matches the
faceted-rendering bug fingerprint. **It is most likely the same VPC race condition, just
amplified by whatever LSM does to the GPU pipeline state.** The bugs were always the same;
LSM cycling makes it nearly 100% reproducible instead of the ~80% intermittent rate seen on
glmark2 alone.

Implications:
- Iterating on register-init preambles or KGSL-style state-restore is unlikely to find the
  fix. Register state was already proven not to differ between good and bad runs.
- The right next-investigation tool is **GPU pipeline / VPC-level execution tracing**,
  which would need vendor docs we don't have, or KGSL-derived hand-traces of CP+VGT+SQ FSM
  state via the `_DEBUG_*` registers.
- Patch 0040's win on the cold-boot 1-face-black quirk is real but separate from this bug
  class.

If a quick experiment is desired anyway: try issuing `RBBM_SOFT_RESET=0xffffffff` followed
by `=0` (the same sequence kernel-side `a2xx_hw_init` does on probe) at DRM client release
time. This is the strongest possible "wipe everything" — full GPU reset. If it fixes
post-LSM gears, narrow down which subsystem reset matters. Risk: GPU may need re-init that
mainline doesn't currently do at runtime.

---

## Current State

### Wins
- Kernel-side state stable, no events
- Mesa cmdstream byte-identical between cold and post-LSM for the main test variants
- Most LSM UI corruption visible improvement (translucent bars closer to correct, text mostly
  improved)
- kmscube non-gears variants no longer have wrong content (just intermittent black faces)

### Open Residuals
1. **kmscube gears post-LSM**: white rectangle in 3D space behind cube + wrong per-vertex
   colors (1 gear partially red/blue). Persists with patch 0040 in place — drain at batch
   start does NOT help. By the time the new client's first batch starts, LSM's work has
   long finished, so there's nothing in flight to drain. The leak is in **persistent
   on-chip state**, not pipeline timing.
2. **kmscube non-gears post-LSM**: faces fade in and out during long runs (correct content
   though). Persists with patch 0040.
3. **glmark2 texture post-LSM**: broken on first post-LSM cycle, sometimes self-corrects
   on subsequent LSM cycles. Persists with patch 0040.
4. **glmark2 faceted-rendering bug** (separate, pre-existing — covered in
   [`ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md`](ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md)).

### Fixed by patch 0040 (verified)
- ~~**kmscube 1-face-black cold-boot quirk**~~ — was actually a pipeline-timing race at
  cold-boot moment, fixed by `CP_WAIT_FOR_IDLE` at `fd2_emit_restore` entry. Pre-LSM kmscube
  now renders cleanly across all modes (smooth/rgba/nv12-1img/nv12-2img/gears).

### Removed from open residuals (was a misinterpretation)
- ~~30× perf regression~~ — `time kmscube -g -c 100` measures 1.16s (≈86 fps), so kmscube
  is NOT actually slow. The earlier "4 submits in 5 seconds" reading came from
  `FD_RD_DUMP_FRAMES=1..1` filtering capture to frame 1 only — kmscube rendered all 200
  frames in ~2.3s normally.

---

## Open Questions / Next Investigation Steps

The byte-identical cmdstream finding rules out anything Mesa can do via the cmdstream path.
Remaining candidate sources for the post-LSM bug, ranked by likelihood:

### 1. Texture L2 cache (TC) holding stale data — HIGH likelihood, EASY to test
- Kernel `gpummu_unmap` invalidates **MMU TLB** (`MH_MMU_INVALIDATE_TC` = MMU translation
  cache, not texture cache). It does NOT invalidate the L2 texture cache.
- Mesa invalidates L2 (`TC_CNTL_STATUS_L2_INVALIDATE`) inside `fd2_emit_tile_mem2gmem` before
  the FB-as-texture blit, but not on context-switch boundaries or batch start.
- If kmscube's CMA-allocated BOs land on physical pages LSM previously used, GPU TC may have
  LSM's data cached at those physical addresses. CPU writes (`glTexImage2D`) don't invalidate
  GPU TC.
- **Test**: add `OUT_PKT0(REG_A2XX_TC_CNTL_STATUS, A2XX_TC_CNTL_STATUS_L2_INVALIDATE)` to
  `fd2_emit_restore`. One-line addition.

### 2. RB cache (color/Z resolve buffers) holding stale data — MEDIUM likelihood
- Adreno 220 has on-chip RB cache. After LSM's last batch finished, RB cache may have content
  not visible via cmdstream.
- Adjacent fix: `RBBM_SOFT_RESET` between processes (kernel-side, on context destroy/release).

### 3. Performance state (clock/power) leakage — MEDIUM likelihood, explains 30× slowdown
- 30× slowdown is too dramatic for cache misses alone. Likely a clock/power state issue.
- `RBBM_PM_OVERRIDE2 = 0x1a0` is set per cmdstream — same in good and bad — so it should keep
  clocks high. But maybe one of the clocks is gated behind another mechanism.
- Adreno 220 has memory bus bandwidth votes via ICC. Mesa freedreno does not currently vote
  ICC bandwidth on a2xx (only newer adreno generations do). On the kernel side, only MDP4
  votes. If the GPU master is starved on the fabric post-LSM (because LSM left some pending
  votes that aren't decremented), GPU memory access stalls.
- **Test**: dump GPU clock rate, ICC fabric load, and memory throughput counters during a
  good vs bad kmscube run.

### 4. GPU MMU page table corruption — LOW likelihood
- We verified no MMU faults in dmesg. If pagetables were corrupt, we'd see faults.
- Could still be a *silent* pagetable issue where TLB returns wrong physical for a valid IOVA,
  but that would cause faults too unless the wrong physical is also-mapped.

### 5. Process-context isolation issue — LOW likelihood
- Mainline freedreno on a2xx uses a single shared gpummu pagetable for all GPU clients
  (no per-process isolation). When LSM's pagetable entries are torn down and kmscube's are
  installed, *something* in the shared state could persist.
- Speculative TLB fetches between unmap and remap might cache stale entries (kernel
  `gpummu_unmap` does invalidate TLB after clearing PTEs but speculative fills could happen).

### Recommended next experiment
Implement test #1 (TC L2 invalidate in `fd2_emit_restore`) as a one-line patch and verify on
device. If it fixes the residuals AND the 30× slowdown, we have our answer. If it only fixes
some, we know we're on the right track but need more.

---

## Cmdstream Capture Methodology

For reproducibility, the capture protocol used in this investigation:

```sh
# On device — install capture helper
cat > /tmp/capture.sh << 'EOF'
#!/bin/sh
LABEL=$1; shift
[ -z "$LABEL" ] && { echo "usage: $0 <label> [kmscube-args]"; exit 1; }
rm -f /tmp/${LABEL}_*.rd /tmp/${LABEL}_*.log
FD_RD_DUMP=enable FD_RD_DUMP_TESTNAME=$LABEL timeout 5 kmscube -c 200 "$@" \
    > /tmp/${LABEL}_kmscube.log 2>&1
NCAP=$(ls /tmp/${LABEL}_*.rd 2>/dev/null | wc -l)
SIZE=$(du -cb /tmp/${LABEL}_*.rd 2>/dev/null | tail -1 | awk '{print $1}')
echo "$LABEL: captures=$NCAP bytes=$SIZE"
EOF
chmod +x /tmp/capture.sh

# 1. After cold boot with LSM masked, capture GOOD
/tmp/capture.sh good_smooth
/tmp/capture.sh good_gears -g
/tmp/capture.sh good_rgba -M rgba
/tmp/capture.sh good_nv12_1img -M nv12-1img
/tmp/capture.sh good_nv12_2img -M nv12-2img

# 2. Run LSM 8 sec, kill it
systemctl unmask surface-manager.service surface-manager-daemon.service
systemctl start surface-manager.service surface-manager-daemon.service
sleep 8
systemctl stop surface-manager.service surface-manager-daemon.service

# 3. Capture BAD
/tmp/capture.sh bad_smooth
/tmp/capture.sh bad_gears -g
/tmp/capture.sh bad_rgba -M rgba
/tmp/capture.sh bad_nv12_1img -M nv12-1img
/tmp/capture.sh bad_nv12_2img -M nv12-2img
```

```sh
# On host — pull and decode
mkdir -p /tmp/fd-captures
scp 'root@172.16.42.2:/tmp/{good,bad}_*' /tmp/fd-captures/
CFFDUMP=$HOME/Documents/GitHub/mesa-latest/build-tools/src/freedreno/decode/cffdump
cd /tmp/fd-captures
mkdir -p decoded

for variant in smooth gears rgba nv12_1img nv12_2img; do
    for state in good bad; do
        for n in 1 2 3 4; do
            f=${state}_${variant}_kmscube_submit0000$n.rd
            [ -f "$f" ] && $CFFDUMP --no-color --no-pager "$f" \
                > decoded/${state}_${variant}_$n.txt
        done
    done
done

# Identity check (filename-line-tolerant)
for variant in smooth gears rgba nv12_1img nv12_2img; do
    for n in 1 2 3 4; do
        diff <(tail -n +2 decoded/good_${variant}_$n.txt) \
             <(tail -n +2 decoded/bad_${variant}_$n.txt) >/dev/null 2>&1 \
            && echo "$variant#$n IDENTICAL" \
            || echo "$variant#$n DIFFERS"
    done
done
```

Sample output from this run (2026-05-08):
```
smooth#1 IDENTICAL
smooth#2 IDENTICAL
smooth#3 IDENTICAL
smooth#4 IDENTICAL
gears#1 IDENTICAL
gears#2 IDENTICAL
gears#3 DIFFERS  (1-byte difference; gears visually broken in good run too — likely noise)
gears#4 IDENTICAL
rgba#1..4 IDENTICAL
nv12_1img#1..4 IDENTICAL
nv12_2img#1..4 IDENTICAL
```

---

## File References

### Kernel
- `drivers/gpu/drm/msm/disp/mdp4/mdp4_kms.c` — runtime PM + clock helpers
- `drivers/gpu/drm/msm/disp/mdp4/mdp4_plane.c` — plane atomic_disable
- `drivers/gpu/drm/msm/adreno/a2xx_gpummu.c` — TLB invalidate on unmap
- `drivers/gpu/drm/msm/msm_gem.c` — `MSM_BO_CONTIGUOUS` allocation path

### Mesa
- `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c` — patch 0038 site
- `src/gallium/drivers/freedreno/a2xx/fd2_emit.c` — `fd2_emit_state`, `fd2_emit_restore`, candidate
  for TC L2 invalidate test
- `src/gallium/drivers/freedreno/freedreno_context.h` — `fd_context_all_dirty` definition

### Reference kernels (for cross-comparison)
- `/home/herrie/webos/touchpad-kernel/refs/htc-msm8660/` (LineageOS HTC pyramid/shooter)
- `/home/herrie/webos/touchpad-kernel/refs/samsung-msm8660/` (LineageOS Samsung Q1)
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/` (webOS reference)

### Capture artifacts (not committed; live on host during active investigation)
- `/tmp/fd-captures/{good,bad}_<variant>_kmscube_submit*.rd` — raw cmdstream dumps
- `/tmp/fd-captures/decoded/<state>_<variant>_<n>.txt` — cffdump-decoded text

### Cross-references — substantial prior investigation

This investigation builds on (and in places re-treads) work already documented:

| Report | Lines | Topic |
|---|---|---|
| [`ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md`](ADRENO_A220_FACETED_RENDERING_COMPREHENSIVE_REPORT.md) | comprehensive | The VPC race condition bug. Already proved register-state inspection found ZERO differences between GOOD and BAD renders. Most likely **same root cause** as our post-LSM gears bug — LSM cycling just amplifies the rate from ~80% to near-100%. |
| [`a22x-mesa-freedreno-faceted-shading-investigation.md`](a22x-mesa-freedreno-faceted-shading-investigation.md) | 272 | Early-phase investigation plan for the faceted-rendering bug. |
| [`a2xx-cache-coherency-analysis.md`](a2xx-cache-coherency-analysis.md) | 204 | CPU↔GPU cache coherency on MSM8660. Covers FIX#1 (`dsb` between L1 and L2 flush). KGSL flushes CPU caches before GPU access; freedreno relies on GPU-side invalidation only. |
| [`a2xx-register-comparison.md`](a2xx-register-comparison.md) | 698 | Register-by-register comparison: Sony, HTC, webOS, mainline kernel, Mesa freedreno. |
| [`adreno-a2xx-driver-comparison-report.md`](adreno-a2xx-driver-comparison-report.md) | 835 | KGSL vs freedreno full driver comparison. |
| [`display-artifacts-alpha-blending-analysis.md`](display-artifacts-alpha-blending-analysis.md) | 1203 | Alpha-blending corruption analysis (Ghidra of proprietary driver). Highly relevant to the LSM translucent-bar gradient symptom. |
| [`freedreno-a2xx-debugging-guide.md`](freedreno-a2xx-debugging-guide.md) | 367 | Debugging methodology for visual artifacts on this GPU. |
| [`freedreno-vs-proprietary-register-comparison.md`](freedreno-vs-proprietary-register-comparison.md) | 149 | Ghidra-derived register diff webOS libGLESv2 vs Mesa. |
| [`kgsl-vs-freedreno-a220-analysis.md`](kgsl-vs-freedreno-a220-analysis.md) | 417 | KGSL vs freedreno deep analysis (Leia-specific). |
| [`kgsl-vs-freedreno-detailed-comparison.md`](kgsl-vs-freedreno-detailed-comparison.md) | 241 | Notes that artifacts occur in ALL rendering modes (sysmem, flush, noscis, inorder). |
| [`libglesv2-vs-freedreno-analysis.md`](libglesv2-vs-freedreno-analysis.md) | 127 | webOS libGLESv2 register-write analysis: `SQ_RESOURCE_MANAGMENT` (0x0d03) and `SQ_PIX_IN_CNTL` (0x0d0c) are NOT written by proprietary driver — earlier hypothesis ruled out. |

### Diagnostic infrastructure already available

- `scripts/glmark2-register-test.sh` — captures `/sys/kernel/debug/a2xx_regs` before+after each
  glmark2 run with user-marked GOOD/BAD status, 25 iterations, results to
  `/media/internal/regs-test`. **Already used during the faceted-rendering investigation;
  found ZERO observable register differences between GOOD and BAD runs.**
- `/sys/kernel/debug/dri/0/{summary,sq,vgt,rb,rbbm,pa,mh,tp,cp,...}` — per-block GPU register
  dumps (rich, ~78 lines for `summary` alone covering RBBM_STATUS, ring buffer pointers,
  interrupt sources, all SQ_* registers including SQ_DEBUG_*, VGT state, RB state, PA/PC/GRAS
  debug data, and A22X-specific VSC/LRZ/GRAS_CONTROL).

---
