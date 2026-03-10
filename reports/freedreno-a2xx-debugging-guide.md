# Freedreno A2XX (Adreno 220) Debugging Guide

## Overview

This document provides a comprehensive guide for debugging visual artifacts on the HP TouchPad's Adreno 220 (Leia) GPU using the freedreno Mesa driver. The artifacts appear as squares/tiles with wrong colors, triangulated surfaces, or corrupted rendering.

---

## 1. Mesa FD_MESA_DEBUG Environment Variables

The freedreno driver has extensive built-in debugging. Set these via environment variables before running your application.

### Available Debug Flags

| Flag | Description | Use Case |
|------|-------------|----------|
| `msgs` | Print debug messages | General debugging output |
| `disasm` | Dump shader disassembly | Shader debugging (a2xx only) |
| `dclear` | Mark all state dirty after clear | State tracking issues |
| `ddraw` | Mark all state dirty after draw | State tracking issues |
| `noscis` | Disable scissor optimization | Scissor-related artifacts |
| `direct` | Force inline (SS_DIRECT) state loads | State loading issues |
| `gmem` | Force gmem rendering when permitted | Test GMEM path |
| `perf` | Enable performance warnings | Performance analysis |
| `nobin` | Disable hw binning | Binning-related artifacts |
| `sysmem` | Use sysmem only (no tiling) | **CRITICAL: Isolates GMEM issues** |
| `serialc` | Disable async shader compile | Shader compilation races |
| `flush` | Force flush after every draw | **CRITICAL: Sync issues** |
| `inorder` | Disable reordering for draws/blits | Ordering-related artifacts |
| `ttile` | Enable texture tiling | Texture format issues |
| `notile` | Disable tiling for internal buffers | Tiling-related artifacts |

### Recommended Test Sequence

#### Test 1: Disable Tiling (Force System Memory Rendering)

```bash
FD_MESA_DEBUG=sysmem glmark2-es2
```

**Hypothesis:** The Adreno 220 is a tile-based renderer. The "squares" artifacts are likely corrupted GMEM (internal tile memory) resolves - either during mem2gmem (load) or gmem2mem (store) operations.

**If this fixes artifacts:** Bug is in GMEM Resolve/Restore logic in `fd2_gmem.c`
- Check `emit_mem2gmem_surf()` - loading tiles from system memory
- Check `emit_gmem2mem_surf()` - storing tiles back to system memory
- Check `prepare_tile_fini_ib()` - tile finalization

**If still broken:** Bug is in rasterization/shader logic, not GMEM tiling.

#### Test 2: Force Synchronization

```bash
FD_MESA_DEBUG=flush glmark2-es2
```

**Hypothesis:** Since artifacts occur intermittently (works 5-10% of time), the GPU might be reading a surface before it's finished writing, or the CPU is queuing commands too fast.

**If this fixes artifacts:** Bug is synchronization/fencing issue
- Missing `OUT_WFI()` (wait-for-idle) in command stream
- Race between CPU command submission and GPU execution
- Missing cache invalidation between operations

**If still broken:** Not a synchronization issue.

#### Test 3: Disable Scissor Optimization

```bash
FD_MESA_DEBUG=noscis glmark2-es2
```

**Hypothesis:** Scissor optimization might be clipping incorrectly on A2XX.

**If this fixes artifacts:** Scissor optimization has A2XX-specific bugs.

#### Test 4: Disable Hardware Binning

```bash
FD_MESA_DEBUG=nobin glmark2-es2
```

**Note:** A22X (Adreno 220/225) already has hw binning disabled by default in freedreno. This is mainly for A20X testing.

```c
// fd2_gmem.c:76-80
/* only a20x hw binning is implemented
 * a22x is more like a3xx, but perhaps the a20x works? (TODO)
 */
if (!is_a20x(batch->ctx->screen))
   return false;
```

#### Test 5: Force Draw Order

```bash
FD_MESA_DEBUG=inorder glmark2-es2
```

**Hypothesis:** Draw reordering might be causing incorrect blending/depth results.

#### Test 6: Combined Test

```bash
FD_MESA_DEBUG=sysmem,flush,msgs glmark2-es2 2>&1 | tee /tmp/mesa-debug.log
```

This combines sysmem rendering with forced flushes and debug messages.

---

## 2. Isolating Triangle/Flat Surface Artifacts

If surfaces look "flat" (individual triangles visible instead of smooth shading), the issue is likely in **varying interpolation** - data passed from Vertex Shader to Fragment Shader.

### Test: Disable Early Z

```bash
# Note: There's no direct "noearlyz" flag, but we can test by modifying the driver
```

Early Z-testing can conflict with late varying updates on older GPUs. Check these locations in the code:

**fd2_zsa.c:53** - Early Z enable logic:
```c
COND(!cso->alpha_enabled, A2XX_RB_DEPTHCONTROL_EARLY_Z_ENABLE);
```

**fd2_draw.c:277** - Clear path:
```c
A2XX_RB_DEPTHCONTROL_EARLY_Z_ENABLE;
```

**fd2_emit.c:262** - Conditional disable:
```c
val &= ~A2XX_RB_DEPTHCONTROL_EARLY_Z_ENABLE;
```

### Varying Corruption Indicators

1. **Triangles visible in smooth surfaces** - Varying interpolation broken
2. **Color banding where gradients expected** - Precision issues in varyings
3. **Flickering vertices** - Vertex shader output corruption

---

## 3. Kernel-Side Debugging (MSM DRM)

### Check dmesg for GPU Faults

```bash
dmesg | grep -i "adreno\|gpu\|fault\|mmu"
```

Look for:
- `MMU_PAGE_FAULT` - GPU accessed unmapped memory
- `GPUMMU fault` - Detailed page table entry dump
- `IB error` - Indirect buffer parsing error
- `gpu fault ring` - Command ring fault

### A2XX Specific Fault Handling

The kernel reports A2XX faults in `a2xx_gpu.c:415-422`:

```c
fault_addr = gpu_read(gpu, REG_A2XX_MH_MMU_PAGE_FAULT);
dev_warn(gpu->dev->dev, "MMU_PAGE_FAULT: %08X\n", fault_addr);
if (status & A2XX_MH_INTERRUPT_MASK_MMU_PAGE_FAULT) {
    a2xx_gpummu_debug_fault(to_msm_vm(gpu->vm)->mmu, fault_addr);
}
```

### GPU Status Registers

```bash
# Check GPU busy status
cat /sys/kernel/debug/dri/0/gpu

# Check current frequency
cat /sys/class/devfreq/*/cur_freq
```

### Disable Power Management (Test)

To rule out power-state restore corruption:

```bash
# Force maximum GPU frequency
echo performance > /sys/class/devfreq/4300000.adreno/governor

# Or set specific frequency (320 MHz = 320000000)
echo 320000000 > /sys/class/devfreq/4300000.adreno/min_freq
echo 320000000 > /sys/class/devfreq/4300000.adreno/max_freq
```

### Cache Coherence Issues

A2XX has a simpler cache hierarchy than modern GPUs. If the kernel doesn't invalidate the cache correctly between submits, you get "stale" tiles.

Check `msm_gem_submit.c` for cache sync operations:

```bash
grep -n "cache\|sync\|flush" drivers/gpu/drm/msm/msm_gem_submit.c
```

---

## 4. Shader Debugging

### Enable Shader Disassembly

```bash
FD_MESA_DEBUG=disasm glmark2-es2 2>&1 | head -500
```

This dumps TGSI and A2XX shader assembly for debugging.

### IR2 Compiler Error Messages

The A2XX shader compiler (IR2) logs errors via mesa_loge():

| Error Message | Cause |
|---------------|-------|
| `ir2: no instruction available in sched_next, aborting` | Scheduler deadlock - shader too complex |
| `ir2: scheduler failed to find valid instruction` | Cannot schedule within constraints |
| `ir2: too many scheduled instructions` | Exceeds 1024 instruction limit |
| `fd2: shader has no instructions` | Compilation produced empty shader |

### A2XX Hardware Limits

| Limit | Value | Impact |
|-------|-------|--------|
| Registers | 64 max | Complex shaders fail |
| Scheduled instructions | 1024 max | Long shaders fail |
| Loop iterations | 32 max (unrolled) | Loops must be short |
| Integer ops | None | Lowered to float |

---

## 5. GMEM (Tile Memory) Debugging

### Understanding GMEM Operations

The Adreno 220 has 256KB of internal GMEM for tile-based rendering:

1. **mem2gmem** - Load tile from system memory to GMEM
2. **Render** - Execute draw calls into GMEM
3. **gmem2mem** - Store tile from GMEM back to system memory

### Key Functions to Check

**fd2_gmem.c:**
- `fd2_emit_tile_mem2gmem()` - Loads tile content
- `emit_gmem2mem_surf()` - Stores rendered tile
- `prepare_tile_fini_ib()` - Prepares tile finalization
- `fd2_emit_tile_init()` - Initial tile setup

### GMEM Debug Trace

Add trace points by rebuilding Mesa with:

```bash
MESA_DEBUG=1 FD_MESA_DEBUG=msgs,gmem glmark2-es2
```

---

## 6. Blend State Debugging

Our patch 0006 adds blend state logging. Look for:

```
A2XX blend: rgb_src=X rgb_dst=Y rgb_func=Z ...
```

### Common Blend Issues on A2XX

1. **No MOD_ALPHA support** - A2XX cannot compensate for premultiplied alpha in hardware
2. **Wrong blend factors** - GL_ONE vs GL_SRC_ALPHA confusion
3. **Missing blend equation** - Default assumed when not set

---

## 7. Diagnostic Command Sequence

Run this sequence to systematically isolate the bug:

```bash
#!/bin/bash

echo "=== Test 1: Sysmem only (no GMEM tiling) ==="
FD_MESA_DEBUG=sysmem glmark2-es2 -b build 2>&1 | tail -5
read -p "Artifacts? (y/n): " t1

echo "=== Test 2: Force flush after every draw ==="
FD_MESA_DEBUG=flush glmark2-es2 -b build 2>&1 | tail -5
read -p "Artifacts? (y/n): " t2

echo "=== Test 3: Sysmem + flush combined ==="
FD_MESA_DEBUG=sysmem,flush glmark2-es2 -b build 2>&1 | tail -5
read -p "Artifacts? (y/n): " t3

echo "=== Test 4: Disable scissor optimization ==="
FD_MESA_DEBUG=noscis glmark2-es2 -b build 2>&1 | tail -5
read -p "Artifacts? (y/n): " t4

echo "=== Test 5: Force in-order rendering ==="
FD_MESA_DEBUG=inorder glmark2-es2 -b build 2>&1 | tail -5
read -p "Artifacts? (y/n): " t5

echo ""
echo "=== Results ==="
echo "Sysmem only: $t1"
echo "Flush: $t2"
echo "Sysmem+flush: $t3"
echo "No scissor: $t4"
echo "In-order: $t5"
echo ""
echo "Interpretation:"
[ "$t1" = "n" ] && echo "  - GMEM tiling is the issue (mem2gmem/gmem2mem)"
[ "$t2" = "n" ] && echo "  - Synchronization/fencing is the issue"
[ "$t3" = "n" ] && echo "  - Combined GMEM + sync issue"
[ "$t4" = "n" ] && echo "  - Scissor optimization is the issue"
[ "$t5" = "n" ] && echo "  - Draw reordering is the issue"
```

---

## 8. Capturing a Crash Dump

If GPU hangs occur:

```bash
# Check for devcoredump
ls /sys/class/devcoredump/

# Read the dump
cat /sys/class/devcoredump/devcd*/data > /tmp/gpu-dump.bin

# Clear the dump to re-enable capture
echo 1 > /sys/class/devcoredump/devcd*/data
```

---

## 9. Summary: Most Likely Causes

Based on the symptoms (intermittent artifacts, works 5-10% of time):

1. **GMEM Synchronization** - Missing WFI before/after tile operations
   - Test: `FD_MESA_DEBUG=sysmem` or `FD_MESA_DEBUG=flush`

2. **Cache Coherence** - Stale data in GPU cache
   - Test: `FD_MESA_DEBUG=flush`

3. **Race Conditions** - CPU queuing faster than GPU consumes
   - Test: `FD_MESA_DEBUG=flush,serialc`

4. **Shader Compilation Timing** - Async compile races
   - Test: `FD_MESA_DEBUG=serialc`

---

## 10. References

- Mesa freedreno source: `src/gallium/drivers/freedreno/`
- A2XX specific: `src/gallium/drivers/freedreno/a2xx/`
- Kernel MSM DRM: `drivers/gpu/drm/msm/`
- A2XX GPU driver: `drivers/gpu/drm/msm/adreno/a2xx_gpu.c`
