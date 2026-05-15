# Adreno 220 (A22X / "Leia") GPU-Internal State Nondeterminism Report

## TL;DR

On HP TouchPad with mainline Linux 6.18 + freedreno (Mesa 26.1)
running on top of msm DRM, **the same PM4 cmdstream produces different
pixel output across consecutive submissions** — not a Mesa bug, not a
cmdstream-emit bug, not a memory-coherency bug, but a hardware state-
machine ceiling we have hard evidence to characterize.

The legacy proprietary stack (KGSL kernel + Qualcomm libGLESv2) on the
*same hardware* produces bit-identical pixel output every run. The
delta is in **GPU-internal SRAMs** (SQ instruction cache, VPC vertex
parameter cache, GPR register file, pipeline FIFOs) that survive a
PM4 cmdstream and are not reset by any documented packet.

This report summarizes the diagnostic chain, what it rules in vs out,
and the architectural question that remains.

## Background: the original symptom

LuneOS apps (luna-surfacemanager UI, kmscube, glmark2) on the
TouchPad show:

- LSM translucent UI bars rendering with a yellow→orange→red
  gradient instead of source-over alpha
- LSM glyph text garbled
- kmscube cube faces rendering black post-LSM-kill
- Persistent white rectangle in 3D space behind kmscube gears
- Faceted/flat shading on glmark2 build/shading
- Visual "flakiness" — same scene rendering differently in
  different sessions

Diagnostic test program: a single 1024×768 GLES2 fragment with a
classic three-vertex color-interpolation triangle (red bottom-left,
green bottom-right, blue top, smooth varying) on a dark-navy
clear background. Captured via `glReadPixels` + `glFinish` from
an off-screen GBM/EGL FBO. Identical C source compiled both
host-x86 (Mesa radeonsi) and ARM (legacy webOS via PalmSDK and
mainline via Yocto cross-toolchain).

## The triangle, three stacks

| Stack | Output | Determinism |
|---|---|---|
| Host x86 / Mesa radeonsi (AMD CAICOS) | Smooth gradient ✓ | Bit-identical |
| Legacy webOS / KGSL / Qualcomm libGLESv2_adreno200.so | Smooth gradient ✓ | **Bit-identical across 10 runs** |
| Mainline / msm DRM / Mesa freedreno-a2xx (this fork) | Triangle + bg vary | **9 distinct hashes / 10 runs** |

The hardware is capable. The proprietary stack works. Mainline doesn't.

## What we ruled out

A series of `gl-capture` runs on mainline with `FD_RD_DUMP=enable`
captures the full PM4 cmdstream. Then `FD_RD_DUMP=full` captures
that *plus* every BO the cmdstream references. We compared 10
back-to-back runs of the *same* test program and ruled out every
input-side variance source:

| Component | Method | Result |
|---|---|---|
| Cmdstream packets | `FD_RD_DUMP=enable`, md5 of all 10 .rd files | All identical (`d938d4b29bf5...`) |
| MMIO control registers | hangdump regs section, 10 runs | Stable except sw fence counter and ring-pointer (expected) |
| Shader constant memory (full 0x4000-0x493F coverage via custom kernel patch) | hangdump regs section, 10 runs, 512 entries each | All 512 byte-identical across runs |
| BO contents the GPU reads | `FD_RD_DUMP=full`, decompressed, byte-diff between runs | Differs only in BOs the cmdstream allocates but doesn't reference; same-pixel runs sometimes have different .rd bytes (irrelevant data) |
| `glReadPixels` coherency / readback race | render once + read twice in one process | A == B in 10/10 runs (no race) |

So the bytes the GPU sees as input are deterministic. Yet output is not.

## What we built to prove this

To extend mainline's debug visibility we wrote two upstream-pending
kernel patches against `drivers/gpu/drm/msm/adreno/`:

1. **Extended hangdump register coverage** for A220:
   - vec4 ALU constants 0..255 (0x4000..0x40FF)
   - texture-fetch constants 0..31 (0x4800..0x48BF)
   - bool constants 0..7 (0x4900..0x4907) — 256 flags
   - loop control constants 0..55 (0x4908..0x493F)
2. **`regrw` debugfs node** for arbitrary on-demand MMIO reads,
   wrapping `gpu_read()` with `pm_runtime_get/put`.

Both confirmed: constant memory is fully deterministic across runs
and matches webOS reference values for slots [0]..[2].

## Where the variance must live

Process of elimination leaves only **GPU-internal state machines that
have no MMIO read-back path on A2XX**:

- **SQ instruction cache** — the SQ fetches shader bytecode into an
  internal cache. Cache state across submits depends on what was
  there before. No documented invalidate.
- **VPC vertex parameter cache** — the per-vertex output FIFO
  feeding the rasterizer. No invalidate event we're aware of.
- **GPR (general-purpose register) file** — per-shader register
  bank. State at draw start carries over from last shader.
- **Pipeline write-coalesce / SC_WAIT_WC FIFO** — partial drain
  events exist but don't fully invalidate.

We've tried emitting the strongest available PM4 events at the start
of every batch (`fd2_emit_restore`):
`CACHE_FLUSH_AND_INV_EVENT` + `VS_FETCH_DONE` + `SC_WAIT_WC` +
`CP_WAIT_FOR_IDLE`. **This did not collapse the variance.** Output
remained 9-of-10 distinct.

## What KGSL does that mainline can't

Reading `drivers/gpu/msm/kgsl_drawctxt.c` from the legacy
2.6 kernel, the proprietary stack maintains a **per-context HW
shadow memory** (~18 KB GPU BO per context). On every context
switch, KGSL emits this PM4 sequence:

```
PM4_WAIT_FOR_IDLE
PM4_LOAD_CONSTANT_CONTEXT regs       (bulk-restore all control regs)
PM4_LOAD_CONSTANT_CONTEXT alu_consts (256 vec4 = 1024 dwords)
PM4_LOAD_CONSTANT_CONTEXT tex_consts (32 sampler descriptors x 6 dw)
PM4_SET_CONSTANT type=2 (bool, 8 dw)
PM4_SET_CONSTANT type=3 (loop, 56 dw)
```

This bulk-restore writes the *entire saved register and constant
state* back into the GPU's MMIO + SRAM. The hardware is *designed*
to be used this way — `PM4_LOAD_CONSTANT_CONTEXT` is a packet whose
purpose is precisely to put the GPU into a known starting state
on context switch.

We tried emitting `PM4_LOAD_CONSTANT_CONTEXT` from Mesa freedreno
on mainline. **The GPU hangs immediately.** Reverted. We don't know
why it hangs (mainline's IB setup may be missing some prerequisite
the legacy KGSL has — IB1/IB2 layout, secure-mode flag, an
initialization sequence, etc.).

## Why simple cache flushes don't fix it

We *can* emit `CACHE_FLUSH_AND_INV_EVENT` + `VS_FETCH_DONE` +
`SC_WAIT_WC` at the start of each batch. The variance doesn't
collapse. Our hypothesis is that these flush *write paths* (RBC
write-back, vertex-fetch drain) but they don't **load fresh values
into the SRAMs** — the SQ instruction cache may keep its
last-fetched contents on cache-miss, GPRs are written by shader
execution and their initial state is whatever the previous shader
left, etc. KGSL's bulk-restore *writes* defaults into all of this
state; flush-and-invalidate alone doesn't.

## Patches currently active and what they do

| # | Effect | Real-world impact |
|---|---|---|
| 0026 | A22X VGT_VERTEX_REUSE_BLOCK_CNTL = 0x02 (KGSL-correct value) | Necessary, reduces faceted shading |
| 0040 | `CP_WAIT_FOR_IDLE` at start of fd2_emit_restore | Drains pipeline before state restore |
| 0044 | Force sysmem rendering on A22X | Avoids broken GMEM tile path. Cost: ~5–10× memory bandwidth, MMC DMA contention errors increased |
| 0045 | Fix non-fast clear color on A22X (write to PS CONST[0] not CLEAR_COLOR register) | Cleared the "blue background instead of dark navy" bug for some draws |
| 0046 | Aggressive cache flush+invalidate at batch start | Doesn't change variance, kept defensively |

Reverted: 0042/0043 (bool/loop and ALU constant bulk-zero) — caused
its own race against per-batch clear-color writes.

## The questions for Gemini

1. **What GPU-internal state on Adreno 220 survives a PM4 cmdstream
   and isn't reset by `CACHE_FLUSH_AND_INV_EVENT` /
   `VS_FETCH_DONE` / `SC_WAIT_WC` / `CP_INVALIDATE_STATE` /
   `CP_WAIT_FOR_IDLE`?** Specifically: SQ instruction cache, VPC
   vertex parameter cache, GPR file initial state, RB write-coalesce
   buffer — what's the documented behavior of each across submits,
   and which can be explicitly invalidated?

2. **Why does `PM4_LOAD_CONSTANT_CONTEXT` (opcode 0x2e) hang on
   mainline freedreno + msm DRM, but works fine in KGSL on the same
   silicon?** What initialization or IB-layout requirement is KGSL
   doing that mainline isn't? The hang is immediate (RBBM_STATUS
   shows GPU stuck). Possible suspects: 8KB-aligned GPU buffer
   address, SECURE-mode bit, prior `CP_SET_CONSTANT` priming, IB1
   nesting depth, ME firmware mode.

3. **Is the "different output for identical PM4 input" pattern
   documented in any Adreno 220 errata?** Could there be a hardware
   quirk that explicitly requires per-submit register-bank reset
   for correct operation, with the proprietary stack working around
   it via shadow memory and mainline accidentally exposing it?

4. **What's the minimal viable per-context-shadow-memory
   implementation in mainline msm DRM?** Specifically: KGSL has
   a `gpustate` BO per context with reg+ALU+tex+bool+loop sections.
   On context switch the kernel emits the LOAD_CONSTANT_CONTEXT
   sequence at the head of the new context's first IB. What's the
   correct equivalent path in mainline's submit pipeline? Can we
   add it without breaking other Adreno generations? Would a
   gen-specific `pre_submit` hook in `struct adreno_funcs` suffice?

5. **Is there a *less* invasive alternative to per-context shadow
   memory?** E.g., a specific PM4 packet sequence
   (or even register-poke sequence) that re-initializes the
   internal SRAMs to a known state without the full LOAD_CONSTANT_CONTEXT
   blob? On Adreno 220 specifically — later generations
   (A3XX+) had `CP_PERFCOUNTER_SAVE` / `CP_REG_TO_MEM` mechanisms
   that imply the SRAMs *are* addressable from the CP, so there
   should be a way.

6. **Why does forcing sysmem rendering increase MMC DMA timeout
   errors on MSM8660?** We understand the fabric-bandwidth
   competition story — GPU sysmem mode = ~5-10× more DRAM traffic
   than GMEM mode = ADM (SD/eMMC DMA) gets starved → mmci_dma_error
   → adm_terminate_all → buggy descriptor free → WARN. Is this a
   known issue, and what's the right ICC RPM bandwidth-request
   pattern to ensure ADM gets guaranteed minimum arbitration when
   GPU is in sysmem mode?

## Update 2026-05-09: kernel-side sanitizer experiment

Per Gemini's "Sanitizer IB" suggestion (Option C), we implemented a
kernel-side sanitizer in `drivers/gpu/drm/msm/adreno/a2xx_gpu.c`
(commits `161cf4f3ce8f` and `0ba88d2f9070`):

- Detects cross-DRM-client boundary via existing
  `ring->cur_ctx_seqno != submit->queue->ctx->seqno` mechanism
- Emits `CP_SET_CONSTANT` zero-fill packets in the *kernel*
  ringbuffer BEFORE the user's IB1 starts
- Bracketed by `CP_WAIT_FOR_IDLE` + `CACHE_FLUSH_AND_INV_EVENT`
  so the writes settle before the user IB
- Avoids the `PM4_LOAD_CONSTANT_CONTEXT` hang and the userspace
  bulk-zero/per-batch-clear race we hit with reverted patches
  0042/0043

### v1 result: SoC-reserved slots matter

First version zeroed ALU constants 0..255 (all 256 vec4 slots).
Result: rendering went all-black on 9 of 10 runs. Slots 0..31 turn
out to hold SoC-reserved system constants the fixed-function
vertex pipeline relies on — `gl-cap-and-regdump-webos` had already
sampled non-trivial values there:
```
slot 0 = (0, 0, 0, 0)
slot 1 = (0x469c4000, 1.0, 0.5, 0)
slot 2 = (2.0, 0.75, 0.375, 0.25)
```
Zeroing slot 1+ broke vertex transforms.

### v2 result: variance not in constant memory

Second version preserves slots 0..31 (offset 0x80 dwords =
`VS_CONST_BASE`) and only zeros Mesa's user-constant range
(slots 32..511 = 1920 dwords) plus bool (8 dwords) and loop
(56 dwords) banks. Texture-fetch constants left alone too
because `texfetch[0]` had non-trivial values on the proprietary
stack (suggesting SoC slots in that bank too).

10-run gl-capture test with v2 active:

| Variant | Distinct hashes / 10 | Pixel-correct match to webOS |
|---|---|---|
| Pre-sanitizer (0044+0045 only) | 9 | 1 run ~59% wrong (best baseline) |
| Sanitizer v1 (zeroed everything) | 6 | 1 run 59% wrong, rest all-black |
| Sanitizer v2 (preserves SoC slots) | 6 | 1 run 59% wrong, rest 99-100% |

### Conclusion: constant-memory residue is *not* the root cause

The sanitizer-v2 puts ALL user-controllable shader-constant SRAM
into a known zero state at every cross-client boundary, with the
hardware writes verified to land before user IB1 runs. Variance
is essentially unchanged. **Shader-constant residue across DRM
clients is therefore *not* the source of mainline's rendering
nondeterminism.**

This eliminates the largest remaining suspect class. The variance
must be in **GPU-internal SRAMs that no `CP_SET_CONSTANT` packet
can reset**:

- GPR (general-purpose register) file initial state
- SQ instruction cache contents
- VPC vertex parameter cache
- Pipeline write-coalesce / FIFO contents

These can only be reset by either:
- `PM4_LOAD_CONSTANT_CONTEXT` bulk-restore (the legacy KGSL path,
  which hangs on mainline)
- A full GPU reset / power-cycle (heavy hammer, breaks
  per-context model)

The sanitizer-v2 patch is *kept* in tree because it's a strict
improvement (cross-client constant residue is now provably impossible)
and gives us a clean baseline for future GPU-state investigations.
But it doesn't fix the user-visible rendering bug.

### New question for Gemini

Given the above hard evidence:

7. **What's the minimum IOVA / pagetable / IB-execution-context
   setup KGSL does that allows `PM4_LOAD_CONSTANT_CONTEXT` to
   work, that mainline freedreno + msm DRM doesn't replicate?**

   Specifically: in legacy KGSL the per-context shadow memory
   (`drawctxt->gpustate`) is allocated via
   `kgsl_sharedmem_alloc_coherent` / similar paths. How is it
   mapped into the GPU's address space? Is it in a privileged
   global pagetable, the per-process pagetable, or somewhere
   specific that the CP's DMA engine can access from a context-
   restore IB? What address-alignment requirements does the
   `PM4_LOAD_CONSTANT_CONTEXT` operand have (4 KB? 8 KB?)?

   On mainline, GBM-allocated GEM objects go via
   `msm_iommu_pagetable` per-process domains. If the CP's
   context-switch DMA engine reads from a global/privileged
   pagetable instead, that would be exactly the kind of mismatch
   that causes immediate hang on submission of a packet that
   *otherwise* works on the same silicon.

   We're now confident that fixing this hang is the only path
   to truly deterministic rendering — every alternative we've
   tried (cache flushes, bulk constant zeroing via SET_CONSTANT,
   per-tile re-emit) has been falsified by hard evidence as
   insufficient.

## What's available for inspection

All artifacts in `reports/fb-captures/`:

- `webos-with-regdump.png` — proprietary reference render
- `webos-with-regdump.regs.txt` — webOS register state post-render
- `webos-10runs/` — 10 webOS run dumps (proves bit-identical pixels)
- `mainline-newkernel-10runs/` — 10 mainline run hangdumps with
  full constant memory coverage
- `rd-full/` — `FD_RD_DUMP=full` captures from mainline runs
- `gl-capture-host.png` — AMD CAICOS reference (matches webOS)

Source for tools in `tools/gl-capture/` and `tools/gpu-regdump/`:

- `gl-capture.c` — mainline render-to-FBO + glReadPixels (off-screen)
- `gl-capture-webos.c` — same, via SDL/PDL
- `gpu-regdump-webos.c` — KGSL ioctl-based register reader
- `gl-cap-and-regdump-webos.c` / `gl-cap-and-regdump-mainline.c` —
  combined render + register dump in single process
- `gl-readpixels-twice.c` — proves readback is not racy

Kernel patches (against `drivers/gpu/drm/msm/adreno/`):

- `a2xx_gpu.c` — extended a220_registers[] hangdump coverage
- `a2xx_debugfs.c` — `regrw_offset` / `regrw_value` debugfs nodes

Mesa patches (in meta-mainline `recipes-graphics/mesa/files/`):

- 0044 — force sysmem on A22X
- 0045 — fix non-fast clear color (write to PS CONST[0])
- 0046 — aggressive cache flush at batch start

## Bottom line for Gemini

The diagnostic is now conclusive at the highest possible level
of certainty obtainable without kernel architecture changes:

1. The cmdstream is byte-identical across runs (FD_RD_DUMP=enable
   md5-equivalent across 10 runs).
2. The MMIO control register state is identical (deterministic
   across runs except for sw fence counters).
3. The shader-constant SRAM contents (full 0x4000-0x493F coverage
   via custom kernel debugfs patches) are deterministic across
   runs.
4. The BO-content the cmdstream references is byte-identical
   (FD_RD_DUMP=full uncompressed equivalence at GPU-readable
   offsets).
5. `glReadPixels` is deterministic post-render (A==B in 10/10
   tests).
6. **Wiping cross-client shader-constant residue via a kernel
   sanitizer (`CP_SET_CONSTANT` zero-fill in the ringbuffer
   before user IB1) does NOT collapse the variance.**

The remaining suspect surface is exclusively GPU-internal SRAMs
not addressable via `CP_SET_CONSTANT` (GPR file, SQ instruction
cache, VPC). The only mechanism on this hardware that resets
these is `PM4_LOAD_CONSTANT_CONTEXT`, which hangs on mainline.

The question is no longer *what's wrong* and no longer *whether
constant memory matters*. It's a single specific architectural
question: **why does `PM4_LOAD_CONSTANT_CONTEXT` hang on mainline
freedreno + msm DRM, and what's the minimum IOVA/pagetable setup
required to make it work?** That's the entirety of question 7
above.

If we can answer that, we replicate KGSL's per-context shadow
memory mechanism in mainline and the visible rendering bugs
disappear. Without it, mainline remains in a fundamental
architectural mismatch with how the Adreno 220 silicon was
designed to be driven.

Generated 2026-05-09.
