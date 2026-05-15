# Adreno 220 (A22X / Leia) — period-8 rendering nondeterminism
## Update report for Gemini after exhaustive kernel-side experiments

## Recap of the original problem

HP TouchPad / Qualcomm APQ8060 / Adreno 220 (Leia) running mainline Linux
6.18 + Mesa 26.1 freedreno + msm DRM. Single-process gl-cap-and-
regdump-mainline test (renders one coloured-gradient triangle and
glReadPixels). Output is **fully deterministic, period 8**:

  - 100 consecutive runs produce hashes in pattern `ABCDEFGH ABCDEFGH ...`
  - 8 unique pixel hashes per period, perfectly equipartitioned (~12-13/100)
  - **1 of 8** outputs (`5adc3160`) renders a CORRECT smooth-gradient
    triangle that matches the legacy webOS proprietary stack reference
  - 7 of 8 outputs are broken: 6 of them have one full colour channel
    (red OR blue, never green) reading as zero; all 7 have 1-2 stale
    GMEM tile-bin rectangles in different positions

## What we've ruled out by experiment since the previous report

### 1. Kernel cross-context sanitizer preamble — fully no-op for rendering

We have an 8-iteration scrub loop in the kernel that emits, per iteration:

  - `CP_LOAD_CONSTANT_CONTEXT` for ALU bank (2048 dwords from a kernel-
    pinned, webOS-sampled-defaults shadow BO)
  - `CP_LOAD_CONSTANT_CONTEXT` for TEX bank (192 dwords, zero or with
    valid 4-channel descriptor — both tested)
  - `CP_SET_CONSTANT` for Bool bank (8 dwords zero)
  - `CP_SET_CONSTANT` for Loop bank (56 dwords zero)
  - `CP_EVENT_WRITE CACHE_FLUSH_AND_INV_EVENT`
  - `CP_WAIT_FOR_IDLE`

Toggling this entire preamble on/off via a `module_param`
(`a2xx_skip_preamble=0/1`) produces **byte-identical pixel hashes in
the same `ABCDEFGH` order**, with 8 MMU faults vs 8 (unrelated, see
below). So all the constant-bank scrubbing is irrelevant to the
rendering output.

### 2. CP_LOAD_CONSTANT_CONTEXT *does* broadcast to all 8 SQ slots

Diagnostic: filled the TEX shadow BO with `0xDEADBEEF` instead of
zeros. **All 8 cycle outputs changed**, all became distinct broken
patterns. This proves CP_LOAD_CONSTANT_CONTEXT mechanism reaches
all 8 wavefront slots' constant SRAM (not just the active slot).
So the kernel scrub is mechanically working — it just isn't where
the variance lives.

### 3. CP_INVALIDATE_STATE doesn't advance the SQ slot pointer

We tested two variants of CP_INVALIDATE_STATE in the cross-context
preamble:

  - `0x00000300` (Vertex 0x100 + Pixel 0x200 shader instruction
    state) — KGSL's mask. No effect on rendering or convergence.
  - `0x00007fff` (all state-class bits) — Mesa's old patch 0036
    "invalidate all state" mask. Per-iteration, prepended to the
    8x scrub loop. **No effect on rendering or convergence** —
    same `ABCDEFGH` cycle, same 1/8 = `5adc3160`, same 0 faults.

So neither shader-only nor all-bits CP_INVALIDATE_STATE rotates the
SQ wavefront slot scheduler pointer.

### 4. Mesa-patch bisect against minimal-Mesa baseline

Built a `mesa_%.bbappend.minimal` with only 3 patches: `is_a22x`
helper (0001), shader scheduler instruction-limit raise (0002),
and the `fd2_clear` color-via-PS-CONST fix (0045). Then added
groups of patches back to test what affects convergence.

| Group | Verdict |
| --- | --- |
| `minimal` alone | 1/8, 0 faults |
| `+ 0017` (VSC register init) | 1/8, 0 faults — innocent |
| `+ 0038a` (mark all 3D state dirty after GMEM tile-fini) | 1/8, 0 — innocent |
| `+ 0046a` (= 0040 WFI + 0046 cache-flush events at batch start) | 1/8, **100 faults** — **fault source** |
| `+ Group A` (9 register-init patches: 0007, 0008, 0017, 0021, 0022, 0026, 0027, 0035, 0041) | 1/8, 0 — innocent |
| `+ Group B` (8 sync/WFI/cache patches: 0006, 0014, 0015, 0016, 0018, 0020, 0023, 0039) | 1/8, 0 — innocent |
| Full Mesa | 1/8, 100 faults — same convergence as minimal |

22 of ~26 Mesa patches confirmed do **NOT** affect the 5adc3160
convergence rate. The 4 untested are all debug-logging-only patches
that emit no PM4 (compile to nothing without `FD_MESA_DEBUG=msgs`).

### 5. The previously-reported "3/8 convergence with full Mesa" was actually a stale-test artifact

Re-checking earlier 100-cap runs, the 3/8 we celebrated came from
old kernels (`fb2cdbb4` 8x ALU scrub-only, `36d78c83` with TEX scrub
added) at a time when reverted Mesa patches `0042/0043` were still
active. Those patches added a large bool/loop bulk-zero
`CP_SET_CONSTANT` block (~72 dwords) to fd2_emit_restore. **That
extra PM4 footprint shifted slot phase enough that 3/8 cycle
positions happened to align on the good slot**. Once 0042/0043
were dropped, we're back to 1/8 across all kernel + mesa
combinations we've tested.

So phase-shifting via more PM4 packets is **lucky alignment, not a
real fix**. The variance source is the SQ wavefront slot scheduler
rotation that mainline cannot reset.

### 6. Diagnostic 0xc000428b GPU MMU fault

Independently of the cycle-of-8 issue, the full-Mesa configuration
emits exactly 100 GPU MMU faults per 100 submits, all at the same
deterministic GPU IOVA `0xc000428b`. Bisect confirmed this comes
from patch 0046 + 0040 combined (their batch-start `WFI +
CACHE_FLUSH_AND_INV_EVENT + VS_FETCH_DONE + SC_WAIT_WC + WFI`
sequence). The fault is non-fatal — GPU recovers — but it's a real
bug in those two patches that we should fix or revert.

`0xc000428b` decodes as a `CP_TYPE3_PKT | (count-1=0) << 16 |
(opcode 0x42 = CP_MEM_TO_REG) << 8 | 0x8b`. A2XX hardware doesn't
implement CP_MEM_TO_REG, so the GPU is **dereferencing this value
as if it were a memory address** and the MMU rejects 0xc000428b
because it's outside the 0x01000000-0x10ff0000 GPU VA range.
Dump-of-IB1 + page-table-walk diagnostics in the kernel didn't
turn up the literal value 0xc000428b in any cmdstream we could
inspect, so it's something the hardware itself is generating
internally.

## What the post-render MMIO state looks like

We added a per-block debugfs interface and an extended IRQ-handler
register dump (TRAN_ERROR, RBBM_STATUS, RB_BASE/RPTR/WPTR,
IB1_BASE/BUFSZ, MH_CLNT_INTF_CFG1/2). Findings:

  - `/sys/kernel/debug/dri/0/{summary,cp,rbbm,mh,sq,rb,pa}` are
    **byte-identical** between a `5adc3160` (good) submit and a
    `4b895c7a` (bad) submit. Same md5 across all 7 files.
  - At submit time IB1_BASE = `0x0130d000`, IB1 size = 272 dwords,
    every time, every cycle position.
  - hw_init register snapshot is identical bit-for-bit across all 8
    cycle positions (PT_BASE, MMU_CONFIG, RBBM_STATUS, PM_OVERRIDE1/2,
    SQ_INTERP, SQ_GPR_MGMT all same).

So whatever determines which cycle position a given submit lands on
is **completely invisible to MMIO**. It must be in hardware-internal
SRAM/cache state that has no documented register port.

## Where we want to go next

Per your earlier analysis, only **real CP_DRAW_INDX packets advance
the SQ wavefront slot scheduler pointer**. KGSL achieves this
implicitly via every per-context-restore `IM_LOAD` + `SET_SHADER_BASES`
+ `DRAW_INDX` (the GMEM blit) sequence, and via `PM4_CONTEXT_UPDATE`
which uses HW shadow memory that mainline cannot use safely on this
SoC.

**Proposed Mesa-side patch**: in `fd2_emit_restore` (called once at
batch start), after the existing state restore, emit **8 dummy
`CP_DRAW_INDX` packets** using Mesa's existing infrastructure:

  - `solid_prog` — the simple "MOV OUT[0], CONST[0]" shader pair
    Mesa already uses for `fd2_clear` and the GMEM blits.
    `fd2_program_emit(ctx, ring, &ctx->solid_prog)` to bind it.
  - `solid_vertexbuf` — a 36-byte BO with a 3-vertex quad, already
    allocated per-context in `fd2_context_init`.
  - Set `PA_SC_WINDOW_SCISSOR_BR` to `0x00000000` so all rasterized
    fragments are scissored away (no actual pixels written).
  - 8x `CP_DRAW_INDX` with `CP_DRAW_INDX_BUFSIZE=0` (RectList,
    AutoIndex, NumIndices=3).
  - After the 8 dummies, restore Mesa's normal scissor / shader-program
    dirty bits so the user IB rebinds its real state.

If this rotates the SQ slot pointer 8 times, all 8 internal slots
get hit by a real draw with known shader+vbuf binding, scrubbing
their per-slot state caches. The user's first real draw should
then land on a slot with clean state, and all 8 cycle positions
should converge to the perfect-gradient `5adc3160`.

## Questions for Gemini

1. **Is the `solid_prog` + `solid_vertexbuf` + scissor-zero approach
   you'd recommend?** Or do you have a better "blackhole draw" pattern
   that's more likely to consume an SQ slot without risking a hang?
   Specifically — is there a register/state that needs to be set so
   the 1-pixel-out-of-bounds point doesn't get culled before SQ
   allocates a slot for it?

2. **Order matters: should the 8 dummy draws fire BEFORE or AFTER
   the user IB?** Our cross-context preamble runs at the START of
   `fd2_emit_restore`. If the SQ allocates a slot per
   `CP_DRAW_INDX`, the user's first draw lands on slot N+1 (where N
   is whatever the dummies consumed up to). To consistently land
   user draws on slot 0 (or whichever is "good"), do we need a known
   resync packet between dummies and user IB?

3. **Should the dummies emit `RB_COLOR_MASK = 0x0` instead of (or
   in addition to) zero scissor?** Belt-and-braces: even if scissor
   doesn't fully cull, color-mask-zero ensures no pixel writes
   even if a fragment makes it to ROP.

4. **Cross-tile interaction with GMEM binning** — `fd2_emit_restore`
   runs before the GMEM tile-bin loop. If we emit dummies there,
   they'll fire ONCE per batch. If the SQ slot pointer is also
   advanced per *tile* (because each tile re-runs the rendering
   IB), do we need to re-emit dummies per tile too? Or does the
   tile-bin loop preserve the slot pointer state from the
   `tile_init`?

5. **Risk of GPU hang**: Mesa's `solid_prog` was originally written
   for clear/blit paths where the hardware is in a known-clean
   state. If we fire it during cross-context boundary while
   constants/textures are still in flux, is there a known sequence
   (CP_INVALIDATE_STATE before? WFI after?) that we should use to
   keep it safe?

6. **Is there an A22X-specific RIB / SQ control register that
   directly resets the SQ slot allocation pointer to 0?** Something
   we could write at hw_init that would forcibly cycle-reset
   everything without needing dummy draws at all? KGSL's reg-save
   list (REG_SQ_GPR_MANAGEMENT, REG_SQ_INST_STORE_MANAGMENT,
   REG_TP0_CHICKEN, REG_RBBM_PM_OVERRIDE2) doesn't seem to include
   one, but maybe there's an undocumented one.

7. **The 0xc000428b MMU fault** — does this look like a known
   hardware errata you've seen? The fact that it's deterministic at
   exactly the address that decodes as a CP_MEM_TO_REG packet header
   value suggests something INSIDE the GPU's PM4 parser is
   misinterpreting a value as that opcode and trying to dereference
   the next dword as a memory address. We're emitting cache-flush
   events (`CACHE_FLUSH_AND_INV_EVENT 0x4`, `VS_FETCH_DONE 0x9`,
   `SC_WAIT_WC 0xC`) — could a misordered or under-WFI'd event
   cause this? If so, what's the safe sequence?
