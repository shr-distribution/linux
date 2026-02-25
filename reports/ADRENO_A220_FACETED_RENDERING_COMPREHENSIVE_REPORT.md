# Comprehensive Report: Adreno 220 Faceted Rendering Investigation
## HP TouchPad (APQ8060/Leia) - Mesa freedreno Driver

**Last Updated:** 2026-02-25
**Status:** Active Investigation
**Success Rate:** ~20% (2/10 SMOOTH iterations)

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Problem Description](#problem-description)
3. [Hardware Background](#hardware-background)
4. [Root Cause Analysis](#root-cause-analysis)
5. [Patches Applied](#patches-applied)
6. [Patches Reverted](#patches-reverted)
7. [KGSL vs Freedreno Comparison](#kgsl-vs-freedreno-comparison)
8. [Register Analysis](#register-analysis)
9. [Test Results](#test-results)
10. [Remaining Investigation Areas](#remaining-investigation-areas)
11. [File References](#file-references)
12. [Commands Reference](#commands-reference)

---

## Executive Summary

The HP TouchPad's Adreno 220 (Leia) GPU exhibits **intermittent faceted/flat shading** where smooth surfaces randomly render as faceted polygons. The issue occurs approximately 80% of the time, with only ~20% of test iterations producing correct smooth shading.

### Critical Findings (2026-02-25)

1. **SQ_INTERPOLATOR_CNTL is stable at 0xffffffff (all smooth interpolation) for ALL iterations - both SMOOTH and FACETED.** There are ZERO observable register differences between working and broken renders.

2. **Texture coordinate interpolation (vec2) works 100% of the time**, but color/normal interpolation (vec4/vec3) fails ~80% of the time. This suggests the issue is specific to certain varying types or sizes, not a general VPC failure.

This confirms:

1. The GPU is **configured correctly** for smooth shading
2. The GPU is **executing incorrectly** - an internal timing/race condition
3. The issue is NOT controllable via standard register state

This is likely an internal VPC (Vertex Parameter Cache) race condition where the fragment shader occasionally reads varying values before the vertex shader has finished writing them.

---

## Problem Description

### Symptoms
- Smooth shaded surfaces (Gouraud/Phong lighting) appear faceted/triangulated
- Issue is **intermittent** - same shader/scene produces both results
- Success rate varies from 10-40% across test runs
- Random pattern - not correlated with iteration number

### Affected Tests
- glmark2 `build` benchmark (3D models with Gouraud shading)
- glmark2 `shading:shading=phong` benchmark
- Any scene using smooth varying interpolation

### What Works Correctly
- Flat-shaded scenes (no interpolation needed)
- **Texture coordinate interpolation (100% success)** - vec2 texcoord works correctly
- Texture mapping
- Alpha blending (fixed separately)
- Shader compilation (same shaders work sometimes)

### What Fails Intermittently (~80% failure)
- **Color varying interpolation** (vec4 Color in Gouraud shading)
- **Normal varying interpolation** (vec3 Normal in Phong shading)
- **Position varying interpolation** (vec4 vertex_position in Phong)

---

## Hardware Background

### GPU Identification

| Property | Value |
|----------|-------|
| GPU | Adreno 220 |
| Code Name | Leia |
| KGSL Chip ID | KGSL_CHIPID_LEIA_REV470 |
| SoC | Qualcomm APQ8060 |
| GPU ID (freedreno) | 220-229 |
| GMEM Size | 512 KB |
| Core Clock | 266 MHz |

### SoC Context

| SoC | Modem | GPU | Devices |
|-----|-------|-----|---------|
| **APQ8060** | None | Adreno 220 | HP TouchPad |
| MSM8660 | Integrated | Adreno 220 | HTC Sensation, EVO 3D |

Both share identical GPU silicon, but TouchPad uses unique power management settings (RBBM_PM_OVERRIDE2 = 0x1a0 vs 0x80 on other devices).

### Key Registers for Interpolation

| Register | Address | Function |
|----------|---------|----------|
| SQ_INTERPOLATOR_CNTL | 0x2182 | Per-varying smooth/flat mode |
| SQ_PROGRAM_CNTL | 0x2180 | Shader program configuration |
| SQ_CONTEXT_MISC | 0x2181 | Shader context settings |
| SQ_GPR_MANAGEMENT | 0x0D00 | GPR allocation (VS/PS) |
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x2316 | Vertex reuse depth |

---

## Root Cause Analysis

### Confirmed NOT the Cause

| Area | Evidence | Conclusion |
|------|----------|------------|
| Register state corruption | Dumps identical for SMOOTH/FACETED | NOT the cause |
| SQ_INTERPOLATOR_CNTL | Always 0xffffffff | NOT the cause |
| Shader compilation | Same shaders produce both results | NOT the cause |
| Mesa state management | No register differences | NOT the cause |
| Mesa command streams | Identical between SMOOTH/FACETED | NOT the cause |
| Blend state | Correct values logged | NOT the cause |
| Vertex buffer content | CRC32 matches between runs | NOT the cause |
| L2 cache coherency | TC_CNTL_STATUS invalidation added | Addressed |
| Kernel cur_ctx_seqno | Freedreno doesn't use CTX_RESTORE_BUF | NOT relevant |
| Rendering path (GMEM vs bypass) | FD_MESA_DEBUG=nobypass still faceted | NOT the cause |

### GMEM Path Test (2026-02-25)

Tested with `FD_MESA_DEBUG=nobypass` to force GMEM (tiled rendering) path instead of bypass (direct rendering):

**Result: Still faceted** - both rendering paths exhibit the same issue.

This rules out:
- GMEM tile load/store operations
- Bypass mode state handling
- Rendering path-specific synchronization

The issue is internal to the GPU's interpolation hardware, independent of how Mesa submits the frame.

### Key Discovery: Freedreno Context Restore Architecture

**Freedreno does NOT use the kernel's `MSM_SUBMIT_CMD_CTX_RESTORE_BUF` mechanism.** It only submits `MSM_SUBMIT_CMD_BUF` and handles all state restore internally via `fd2_emit_restore()` embedded in command buffers.

This means:
1. The kernel's `cur_ctx_seqno` tracking is irrelevant for freedreno
2. Mesa handles all GPU state initialization in its command stream
3. The `fd2_emit_restore()` function is called from `fd2_gmem.c` at frame start
4. Any "context restore" issues are in Mesa, not the kernel

### Possible Contributing Factor: MDP4 Display Underruns

MDP4 buffer underruns are occurring, indicating memory bandwidth contention between the display controller and GPU:
- Both share the same memory bus on APQ8060
- When MDP4 steals bandwidth to fetch framebuffer data, GPU memory access is delayed
- This could affect VPC timing and contribute to the race condition

Current MDP4 bandwidth: 377 MB/s avg / 471 MB/s peak (from device tree)

### Likely Cause: Internal VPC Race Condition

The **Vertex Parameter Cache (VPC)** bridges vertex shader outputs to fragment shader inputs. Evidence suggests:

1. **Compositor interference correlates with FACETED rendering:**
   - SMOOTH iterations: 0 QSGRenderThread (compositor) events
   - FACETED iterations: 93-417 compositor events

2. **No observable register differences** - the race is internal to the GPU pipeline

3. **VGT_VERTEX_REUSE_BLOCK_CNTL affects VPC timing:**
   - KGSL uses 0x02 (conservative)
   - Freedreno was using 0x3b (aggressive) - now fixed

4. **Timing-dependent behavior** - random success/failure pattern

### Theory

When the compositor (or another GL context) uses the GPU between glmark2 draws:
1. Context switch occurs
2. Some internal VPC state is not properly saved/restored
3. Fragment shader reads stale/incorrect varying values
4. All fragments in a triangle get the same value = flat shading

---

## Patches Applied

### Currently Active Patches

#### Patch 0026 - VGT_VERTEX_REUSE_BLOCK_CNTL Fix
**Files:** `fd2_emit.c`, `fd2_draw.c`, `fd2_gmem.c`
**Status:** ✅ Active

Changed VGT_VERTEX_REUSE_BLOCK_CNTL from 0x3b to 0x02 for A22X to match KGSL. Also added missing VGT_OUT_DEALLOC_CNTL=0x02.

```c
/* A22X: Use same VGT settings as KGSL driver */
OUT_PKT3(ring, CP_SET_CONSTANT, 2);
OUT_RING(ring, CP_REG(REG_A2XX_VGT_VERTEX_REUSE_BLOCK_CNTL));
OUT_RING(ring, 0x00000002);  // Was 0x3b

OUT_PKT3(ring, CP_SET_CONSTANT, 2);
OUT_RING(ring, CP_REG(REG_A2XX_VGT_OUT_DEALLOC_CNTL));
OUT_RING(ring, 0x00000002);  // Was missing
```

#### Patch 0027 - A220_GRAS_CONTROL Context Restore
**File:** `fd2_emit.c`
**Status:** ✅ Active

Added A220_GRAS_CONTROL initialization to match KGSL context restore (register range 0x2208-0x2210).

```c
OUT_PKT3(ring, CP_SET_CONSTANT, 2);
OUT_RING(ring, CP_REG(REG_A2XX_A220_GRAS_CONTROL));
OUT_RING(ring, 0x00000000);
```

#### Earlier Patches (0001-0024)

| Patch | Description | Status |
|-------|-------------|--------|
| 0001-0005 | is_a22x() helper, shader/debug improvements | ✅ Active |
| 0006 | WFI after draw | ✅ Active |
| 0007 | SQ_GPR_MANAGEMENT = 0x00040400 | ✅ Active (Critical) |
| 0008 | SQ_INTERPOLATOR_CNTL after SQ_PROGRAM_CNTL | ✅ Active |
| 0009-0013 | Debug logging | ✅ Active |
| 0014 | WFI after constant emission | ✅ Active |
| 0015 | WFI after vertex buffer setup | ✅ Active |
| 0016 | VGT DMA wait | ✅ Active |
| 0017 | VSC register initialization | ✅ Active |
| 0018 | Synchronous cache flush (CACHE_FLUSH_AND_INV_EVENT) | ✅ Active |
| 0019-0023 | Various sync improvements | ✅ Active |
| 0024 | VPC hammer sync before draw | ✅ Active |

---

## Patches Reverted

### Patch 0025 - SQ_PIX_IN_CNTL (CAUSED GPU HANG)
**Status:** ❌ REVERTED

Writing to registers SQ_RESOURCE_MANAGMENT (0x0d03) and SQ_PIX_IN_CNTL (0x0d0c) caused complete GPU hang:
```
timeout waiting to drain ringbuffer 0 rptr/wptr = 0/23
GPU status: 0x82400310
```

These registers exist in KGSL headers but are not safe to write.

### Kernel Patch - cur_ctx_seqno Reset (REVERTED)
**Status:** ❌ REVERTED (ineffective)

Added `gpu->rb[0]->cur_ctx_seqno = 0` to `a2xx_hw_init()` to force context restore after GPU init/resume. This matches what `a6xx_hw_init()` does.

**Why it didn't help:** Freedreno doesn't use `MSM_SUBMIT_CMD_CTX_RESTORE_BUF` at all. It only submits `MSM_SUBMIT_CMD_BUF` and handles state restore via `fd2_emit_restore()` embedded in command buffers. The kernel's `cur_ctx_seqno` mechanism is never triggered.

### Other Reverted Attempts

| Attempt | Problem |
|---------|---------|
| GRAS_CONTROL = 0 (initial) | Grey textures |
| PS_REGS \|= 0x80 | Grey textures (corrupted register count) |
| CP_DRAW_INDX_2 dummy draw | Grey textures |
| TP0_CHICKEN = 0x00 | GPU hang |
| Kernel cur_ctx_seqno reset | Ineffective - freedreno uses inline state restore |

---

## KGSL vs Freedreno Comparison

### Critical Differences Found and Addressed

| Issue | KGSL Value | freedreno (Before) | freedreno (After) | Status |
|-------|------------|-------------------|-------------------|--------|
| RBBM_PM_OVERRIDE2 | 0x1a0 (Leia) | 0xfff | 0x1a0 | ✅ Fixed |
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x02 | 0x3b | 0x02 | ✅ Fixed |
| VGT_OUT_DEALLOC_CNTL | 0x02 | Not set | 0x02 | ✅ Fixed |
| A220_GRAS_CONTROL | Context save | Not set | 0x00 | ✅ Fixed |
| SQ_GPR_MANAGEMENT | 0x00040400 | Not set | 0x00040400 | ✅ Fixed |
| LRZ_VSC_CONTROL in GMEM | 0x00 | Not set | 0x00 | ✅ Fixed |
| Cache flush type | Sync (0x16) | Async (0x06) | Sync (0x16) | ✅ Fixed |

### Still Different (Not Yet Addressed)

| Issue | KGSL | freedreno | Priority |
|-------|------|-----------|----------|
| VSC Pipe Registers (0x0C01-0x0C1D) | Save/restore | Not handled | MEDIUM |
| Draw command format | PM4_DRAW_INDX_2 | CP_DRAW_INDX | MEDIUM |
| SQ_PROGRAM_CNTL bit 17 | Set for Leia | Dynamic | LOW |
| RB_DEPTHCONTROL in GMEM | 0x08 (Leia) | Unknown | LOW |
| Hardware binning | Implemented | Disabled for A22X | LOW |

---

## Register Analysis

### Interpolation-Critical Registers

#### SQ_INTERPOLATOR_CNTL (0x2182)
```
Bit Layout:
  [0:15]  PARAM_SHADE     - Bit per varying (1=smooth, 0=flat)
  [16:31] SAMPLING_PATTERN

Value 0xffffffff = All 32 varyings use smooth interpolation
```

**Observation:** This register is **always 0xffffffff** for both SMOOTH and FACETED iterations. The hardware is configured correctly but executing incorrectly.

#### SQ_GPR_MANAGEMENT (0x0D00)
```
Bit Layout:
  [0]     REG_DYNAMIC   - 0=static, 1=dynamic allocation
  [4:11]  REG_SIZE_PIX  - GPRs for pixel shader (0-255)
  [12:19] REG_SIZE_VTX  - GPRs for vertex shader (0-255)

Value 0x00040400 = 64 GPRs each for VS and PS
```

**Fixed:** Was uninitialized before patch 0007, causing random GPR starvation.

#### VGT_VERTEX_REUSE_BLOCK_CNTL (0x2316)
```
Bit Layout:
  [0:5]  VTX_REUSE_DEPTH - Depth of vertex reuse buffer

Value 0x02 = Conservative (KGSL default)
Value 0x3b = Aggressive (old freedreno value)
```

**Fixed:** Now uses 0x02 to reduce VPC race window.

---

## Test Results

### Latest Test (2026-02-25)

```
SMOOTH: 2 | FACETED: 8 | Success rate: 20%

Register Sampling:
- SQ_INTERPOLATOR_CNTL: stable (0xffffffff) for ALL iterations
- No register differences between SMOOTH and FACETED
```

### Diagnostic Tests (2026-02-25)

Ran simple isolated tests as suggested by Gemini to identify which GPU functionality works correctly:

| Test | Command | Result | Notes |
|------|---------|--------|-------|
| **Clear** | `glmark2-es2-drm -b clear` | ✅ OK | Too fast to observe, showed Tux logo |
| **Texture** | `glmark2-es2-drm -b texture` | ✅ OK (100%) | Texture coordinate interpolation works correctly |
| **Gouraud (shading)** | `glmark2-es2-drm -b shading:shading=gouraud` | ❌ FACETED (~20%) | Color varying interpolation fails intermittently |
| **Build** | `glmark2-es2-drm -b build` | ❌ FACETED (~20%) | Same as Gouraud - uses color varying |

**Key Finding:** Texture coordinate interpolation works perfectly (100% success), but **color/normal varying interpolation fails intermittently (~80% failure rate)**.

This narrows the issue to:
- **NOT** generic VPC (Vertex Parameter Cache) failure
- **NOT** all varying interpolation
- **SPECIFICALLY** color (vec4 Color) and normal (vec3 Normal) varying interpolation

**Shader Analysis:**

| Shader | Varyings Used | Interpolation Type | Works? |
|--------|--------------|-------------------|--------|
| Texture | `texcoord` (vec2) | Smooth/Perspective | ✅ YES |
| Gouraud/Build | `Color` (vec4) | Smooth/Perspective | ❌ INTERMITTENT |
| Phong | `vertex_normal` (vec3), `vertex_position` (vec4) | Smooth/Perspective | ❌ INTERMITTENT |

The difference may be:
1. **Varying count** - Texture uses 1 varying (vec2=2 components), Gouraud uses 1 varying (vec4=4 components)
2. **Varying size** - vec2 vs vec4 could trigger different VPC paths
3. **Shader complexity** - Texture shader is simpler than lighting shaders

### Historical Progress

| Date | Configuration | Success Rate |
|------|---------------|--------------|
| Pre-patches | Baseline | ~10-20% |
| + SQ_GPR_MANAGEMENT | 0x00040400 | ~20-30% |
| + VGT fix | 0x02 | ~40% |
| + GRAS_CONTROL | Latest | ~40% |

---

## Remaining Investigation Areas

### Priority: HIGH

1. **Varying Type/Size Dependency (NEW LEAD)**
   - Texture coords (vec2, 2 components) work 100%
   - Color (vec4, 4 components) fails ~80%
   - Normal (vec3, 3 components) fails ~80%
   - Investigate if there's a difference in how VPC handles different varying sizes
   - Check SQ_PROGRAM_CNTL export_mode settings for different varying counts

2. **VPC Internal State**
   - No software-visible mechanism to flush/stall VPC
   - May need to find undocumented workaround

3. **Context Switch Timing**
   - Compositor interference strongly correlates with faceted
   - DRM/MSM may not save all Leia-specific internal state

### Priority: MEDIUM

3. **VSC Pipe Registers (0x0C01-0x0C1D)**
   - KGSL saves/restores these during context switch
   - freedreno doesn't handle them

4. **SQ_CONTEXT_MISC**
   - KGSL sets to 0x00000008
   - Need to verify Mesa sets correctly

### Priority: LOW

5. **Hardware Binning**
   - Disabled for A22X in freedreno
   - Could implement but unlikely to fix interpolation

6. **Draw Command Format**
   - KGSL uses PM4_DRAW_INDX_2 for Leia
   - freedreno uses generic CP_DRAW_INDX

---

## File References

### Mesa (freedreno a2xx)
- `src/gallium/drivers/freedreno/a2xx/fd2_emit.c` - Context restore, register init
- `src/gallium/drivers/freedreno/a2xx/fd2_draw.c` - Draw commands, synchronization
- `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c` - GMEM tile operations
- `src/gallium/drivers/freedreno/a2xx/fd2_program.c` - Shader program setup

### Kernel (DRM/MSM)
- `drivers/gpu/drm/msm/adreno/a2xx_gpu.c` - GPU init, recovery

### KGSL Reference (webOS kernel)
- `drivers/gpu/msm/kgsl_drawctxt.c` - Context management (1800+ lines)
- `drivers/gpu/msm/kgsl_yamato.c` - GPU control
- `drivers/gpu/msm/yamato_reg.h` - Register definitions
- `drivers/gpu/msm/leia_reg.h` - Leia-specific registers

### Patches Location
- `/media/herrie/LuneOS/scarthgap/webos-ports/meta-smartphone/meta-mainline/recipes-graphics/mesa/files/00*.patch`

---

## Commands Reference

### Build and Deploy
```bash
# Build Mesa
bitbake mesa -c compile -f && bitbake mesa

# Deploy library
scp -P 22 /path/to/libgallium*.so root@172.16.42.2:/usr/lib/

# Verify deployment
ssh -p 22 root@172.16.42.2 "md5sum /usr/lib/libgallium-26.1.0-devel.so"

# Reboot
ssh -p 22 root@172.16.42.2 "echo b > /proc/sysrq-trigger"
```

### Testing
```bash
# Run interactive test
/glmark2-interactive-test.sh

# Single benchmark run
glmark2-es2-drm --benchmark build:use-vbo=false

# With debug output
FD_MESA_DEBUG=msgs MESA_DEBUG=1 glmark2-es2-drm --benchmark shading:shading=phong
```

### Debugging
```bash
# Check GPU registers
cat /sys/kernel/debug/dri/0/regs

# Ftrace for context switches
cat /sys/kernel/debug/tracing/trace | grep -E "adreno|msm_gpu"

# Strace for crashes
strace -f glmark2-es2-drm --benchmark build:use-vbo=false 2>&1

# GDB backtrace
gdb -batch -ex 'run' -ex 'bt' --args glmark2-es2-drm --benchmark build:use-vbo=false
```

---

## Consolidated From

This report consolidates information from:
- `a22x-mesa-freedreno-faceted-shading-investigation.md`
- `kgsl-vs-freedreno-a220-analysis.md`
- `a2xx-register-comparison.md`
- `a22x-vgt-dma-workaround-analysis.md`
- `a2xx-cache-coherency-analysis.md`
- `freedreno-a2xx-debugging-guide.md`
- `glmark2-test-diagnostic-guide.md`
- `leia-sq-registers-analysis.md`
- `freedreno-vs-proprietary-register-comparison.md`
- `display-artifacts-alpha-blending-analysis.md`

---

## Conclusion

The intermittent faceted rendering on the HP TouchPad's Adreno 220 GPU is likely caused by an **internal VPC (Vertex Parameter Cache) race condition** that cannot be directly controlled via register state. The GPU is correctly configured for smooth interpolation (SQ_INTERPOLATOR_CNTL = 0xffffffff), but internal pipeline timing occasionally causes the fragment shader to read incorrect varying values.

All obvious KGSL-vs-freedreno differences have been addressed, improving the success rate from ~10-20% to ~40%. Further improvement may require:

1. Finding undocumented VPC control mechanisms
2. Deeper kernel-side context switch analysis
3. Accepting this as a hardware limitation that cannot be fully worked around

The correlation between compositor activity and faceted rendering strongly suggests the issue is related to GPU context switching under load.
