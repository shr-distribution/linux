# Adreno 220 (A22X / Leia) — period-8 rendering nondeterminism on
# mainline Linux 6.18 + Mesa 26.1 freedreno + msm DRM

## Hardware
- HP TouchPad — Qualcomm APQ8060 SoC (dual-core ARMv7 Scorpion)
- GPU: Adreno 220 (A22X "Leia") — same hardware that the legacy
  proprietary KGSL+webOS stack drives correctly
- Display: 1024×768 LCDC via MDP4

## Test program
`gl-cap-and-regdump-mainline` — single-process GBM/EGL/GLES2 program
that:
1. Opens DRM device, sets up GBM surface (1024×768 RGBA8)
2. Compiles a deterministic vertex+fragment shader pair
3. Submits ONE draw of a 3-vertex coloured-gradient triangle
4. `glReadPixels()` into a binary file
5. Exits

Each invocation is a fresh DRM client / fresh DRM context.

## Reference
The same hardware running the legacy webOS proprietary stack
(KGSL kernel + Qualcomm libGLESv2) produces a consistent
smooth-gradient triangle on a dark-navy background. PNG saved as
the reference baseline.

## The bug
Run `gl-cap-and-regdump-mainline` 100 times in a tight bash loop:

  - Output is **fully deterministic per cycle position**
  - **Period 8**: hashes follow `ABCDEFGH ABCDEFGH ...` exactly
  - **8 unique pixel hashes** per period
  - **1 of 8** (md5 `5adc3160`) renders a CORRECT smooth gradient
    matching the webOS reference (mean RGB 34/34/34, delta vs
    reference = 30.1)
  - **7 of 8** show specific failure modes:
      - 6 of 7 have one full colour channel reading as zero across
        the entire frame (red OR blue, never green)
      - All 7 have 1-2 stale rectangular tiles that look like
        leftover GMEM tile content
  - The same 8 hashes appear in the same cycle position on every
    boot

Visual examples (all 8 outputs in the corresponding folder):
- `5adc3160` — the GOOD output, smooth gradient
- `fb0772c9` — gradient + 1 black tile top-center
- `91666260` — missing red channel + 2 tiles bottom
- `4b895c7a` — missing blue channel + center tile
- `37242f10`, `3584c308`, `f197cb26`, `aae50ced` — variants of
  missing-blue + tile leftovers

## What we have ruled out (by experiment)

### Kernel-side cross-context sanitizer (no effect)
We added an 8-iteration cross-context preamble that emits
`CP_LOAD_CONSTANT_CONTEXT` for ALU bank (2048 dwords) and TEX bank
(192 dwords), plus `CP_SET_CONSTANT` for Bool (8) and Loop (56)
banks, with `CACHE_FLUSH_AND_INV_EVENT + WAIT_FOR_IDLE` between
iterations. Goal: scrub all 8 hypothesized SQ wavefront slots'
constant SRAMs.

A/B with module-param `a2xx_skip_preamble`:
- preamble enabled: 8/8 hashes ABCDEFGH
- preamble disabled: 8/8 hashes ABCDEFGH **byte-identical**

Conclusion: kernel sanitizer is a no-op for this rendering.
Either CP_LOAD_CONSTANT_CONTEXT doesn't reach the slots that
matter, OR these constant banks aren't the source of variance.

### A `0xDEADBEEF` sentinel diagnostic in the TEX shadow CHANGED
all 8 hashes — proving CP_LOAD_CONSTANT_CONTEXT for TEX **does**
broadcast to all 8 slots, the kernel scrub is mechanically
working, but its content (zero) happens to produce the same 7-of-
8 broken pattern as no scrub.

### Mesa patches (mostly ruled out)
We had ~30 freedreno A22X patches (cache flushes, register inits,
WFIs, GPR allocation tweak, clear-color fix etc.). Stripped Mesa
back to **3 patches only** (`is_a22x` helper, scheduler limit,
clear-color via PS-CONST instead of CLEAR_COLOR register).

Result with **minimal** Mesa:
- The `0xc000428b` MMU page fault that fired on EVERY submit
  with full Mesa is **completely gone** (0/100 vs 100/100)
- The 8 unique hashes are **byte-identical** to full Mesa
- The cycle is **same period 8**, same 1/8 perfect-gradient

So one of our removed Mesa patches caused the deterministic MMU
fault, but the page-8 cycle is **independent** of all our patches.
The variance is upstream-mainline freedreno or kernel-msm or
hardware.

### MMIO register state at fault time / post-render (identical)
We dump all of these on every fault and post-render via the
existing per-block debugfs files (summary, cp, rbbm, mh, sq, rb,
pa) plus an extended IRQ handler:

  TRAN_ERROR, RBBM_STATUS, RB_BASE, RB_RPTR, RB_WPTR,
  IB1_BASE, IB1_BUFSZ, MH_CLNT_INTF_CFG1/2,
  PT_BASE, MMU_CONFIG, PM_OVERRIDE1/2,
  SQ_INTERP, SQ_GPR_MGMT, SQ_PROGRAM_CNTL, SQ_CONTEXT_MISC,
  SQ_VS_PROGRAM, SQ_PS_PROGRAM, SQ_INST_STORE_MGMT,
  SQ_INT_CNTL/STATUS, SQ_DEBUG_*

**Every byte of every debugfs file is identical** between a "good"
cycle position (5adc3160) and a "bad" cycle position (4b895c7a)
post-render. md5 of all 7 files matches across positions.

### Per-submit allocator state (identical)
Logged across 8 cycle positions with minimal Mesa:

  IB1 IOVA = 0x0130d000 every time
  IB1 size = 272 dwords every time
  RPTR = WPTR = 0x2a5 every time post-submit
  hw_init seqno register snapshot identical bit-for-bit

So: every kernel-visible state is IDENTICAL across the 8 cycle
positions. The variance is entirely in HARDWARE state that is
not exposed via any MMIO we have access to.

## Failure-mode taxonomy (7/8 broken)

The "missing colour channel" pattern is striking. Six of seven
broken outputs lose exactly one channel (red OR blue, NEVER
green). The triangle has 3 vertex colours (1,0,0), (0,1,0),
(0,0,1) — green being the centre vertex always rendering correctly
suggests it's NOT a vertex-fetch base-address problem (that would
affect all three channels equally). It looks more like a **per-
channel write mask** or **per-channel ROP path** failure on
specific cycles.

Tile leftovers in 7 of 8 outputs are GMEM bin-sized rectangles
in different positions per cycle — consistent with GMEM tile
resolve writing to memory at the wrong target offset, OR with
some tile bins not resolving at all and leaving prior-frame
content. Mainline freedreno A22X uses GMEM tile binning by
default for this 1024×768 render.

## Asks for Gemini

1. **What hidden hardware state in Adreno 220 (Leia) could
   exhibit a clean period-8 cycle that survives `pm_runtime`
   suspend/resume and full hw_init?** We've ruled out every
   readable SRAM, register, IOVA, and submit-time variable.
   The candidate must:
   - Cycle deterministically with period 8
   - Persist across pm_runtime auto-suspend (~1-2 sec idle)
   - Not be readable via the documented A2XX MMIO map
   - Affect per-channel rendering (RGB) and per-tile GMEM
     resolve

2. **Is there a known A2XX/A22X hardware bug where a particular
   subset of internal wavefront slots have stuck/wrong write-mask
   or ROP-channel state?** The 6-of-7-broken-outputs-missing-blue-
   or-red pattern (never green) is suspicious.

3. **What primitive could "advance the period-8 cycle" so that we
   can land on the good slot deterministically?** We tested
   adding/removing kernel-side PM4 packets (CP_NOPs of various
   counts, the entire 8x scrub block on/off). All shifts produce
   different cycle phases but never collapse to a single output.
   Is there an explicit register or PM4 to "reset" this hidden
   state?

4. **Could this be tied to the hardware `RBBM_PM_OVERRIDE2` clock-
   gating bits or to a power-domain transition?** Our register
   reads `PM_OVERRIDE2 = 0x000003a0` every hw_init — same value,
   but maybe specific bits affect cycle phase.

5. **The legacy KGSL driver does not seem to exhibit this.** Why?
   Possible answers we want to evaluate:
   - KGSL uses CP_IM_LOAD/CP_SET_SHADER_BASES which mainline
     freedreno doesn't (mainline patches the vfetch instruction
     dwords directly). Could the IM_LOAD path "broadcast-init" all
     8 slots' shader instruction caches in a way mainline can't?
   - KGSL keeps the GPU awake longer via per-context refcounts
     rather than mainline's pm_runtime auto-suspend.
   - KGSL uses PM4_CONTEXT_UPDATE which mainline cannot use
     (HW shadow memory hangs the GPU on this SoC).
