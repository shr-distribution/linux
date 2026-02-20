# Freedreno vs Proprietary Driver Register Comparison

**Date:** February 19, 2026
**Analysis:** Ghidra decompilation of webOS libGLESv2.so vs Mesa freedreno A2XX

---

## Summary

After comprehensive comparison, most registers **MATCH** between proprietary and freedreno.
The key differences are in **write order** and some **initialization values**.

---

## Register Comparison Table

### Shader Sequencer (SQ) Registers

| Register | Address | Proprietary | freedreno | Match? |
|----------|---------|-------------|-----------|--------|
| SQ_PROGRAM_CNTL | 0x2180 | Per-draw | Per-draw | ✅ |
| SQ_CONTEXT_MISC | 0x2181 | 4 (init) | 4 (CENTERS_ONLY << 2) | ✅ |
| SQ_INTERPOLATOR_CNTL | 0x2182 | 0xffffffff | 0xffffffff | ✅ |
| SQ_WRAPPING_0 | 0x2183 | 0 | Per-draw | ✅ |
| SQ_WRAPPING_1 | 0x2184 | 0 | Per-draw | ✅ |
| SQ_GPR_MANAGEMENT | 0x0d00 | Dynamic | 0x00040401 | ⚠️ Different |
| SQ_INST_STORE_MANAGMENT | 0x0d02 | 0x180/0x300 | 0x00000180 | ✅ |
| SQ_RESOURCE_MANAGMENT | 0x0d03 | NOT WRITTEN | NOT WRITTEN | ✅ |
| SQ_PIX_IN_CNTL | 0x0d0c | NOT WRITTEN | NOT WRITTEN | ✅ |

### Render Backend (RB) Registers

| Register | Address | Proprietary | freedreno | Match? |
|----------|---------|-------------|-----------|--------|
| RB_DEPTHCONTROL | 0x2200 | Per-draw | Per-draw | ✅ |
| RB_BLEND_CONTROL | 0x2201 | 0x10001 | Per-draw | ✅ |
| RB_COLORCONTROL | 0x2202 | 4 | Per-draw | ✅ |
| RB_MODECONTROL | 0x2208 | Per-context | COLOR_DEPTH/EDRAM_COPY | ✅ |
| RB_SAMPLE_POS | 0x220a | 0x88888888 | 0x88888888 | ✅ |
| RB_BC_CONTROL | 0x0f01 | N/A | A20X only | ⚠️ A22X skips |
| RB_EDRAM_INFO | 0x0f02 | Calculated | KERNEL SETS | ✅ |
| RB_LRZ_VSC_CONTROL | 0x2209 | N/A | 0 (A22X) | ✅ |

### Primitive Assembly (PA) Registers

| Register | Address | Proprietary | freedreno | Match? |
|----------|---------|-------------|-----------|--------|
| PA_CL_VTE_CNTL | 0x2206 | 0 | Per-draw | ✅ |
| PA_CL_CLIP_CNTL | 0x2204 | 0x43f? | 0 in clear | ⚠️ Check |
| PA_SC_WINDOW_OFFSET | 0x2316 | 0x3b | 0 / per-tile | ⚠️ Init value |
| PA_SC_WINDOW_SCISSOR_TL | 0x2302 | 1 | Per-draw | ✅ |
| PA_SC_AA_MASK | 0x2312 | 0xffff | Per-draw | ✅ |
| PA_SC_LINE_STIPPLE | 0x2301 | 0 | Per-draw | ✅ |

### Vertex/Geometry (VGT) Registers

| Register | Address | Proprietary | freedreno | Match? |
|----------|---------|-------------|-----------|--------|
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x2307 | 0x02? | 0x3b (A22X), 0x02 (A20X) | ⚠️ Check |
| VGT_OUT_DEALLOC_CNTL | 0x2308 | N/A | 0x02 (A20X only) | ⚠️ A22X skips |

### Power/Clock Registers

| Register | Address | Proprietary | freedreno | Match? |
|----------|---------|-------------|-----------|--------|
| RBBM_PM_OVERRIDE1 | 0x039c | 0xffffffff | 0xffffffff | ✅ |
| RBBM_PM_OVERRIDE2 | 0x039d | 0x1a0 (Leia) | 0xfff (A22X) | ⚠️ Different |
| TP0_CHICKEN | 0x0e1e | 0 or 2 | 0x02 | ✅ |

---

## Critical Findings

### 1. Register Write Order (CRITICAL - FIXED)

**Problem:** Patch 0016 changed write order:
- Original: SQ_CONTEXT_MISC → SQ_PROGRAM_CNTL
- Patch 0016: SQ_PROGRAM_CNTL → SQ_CONTEXT_MISC (via atomic write)

**Solution:** Remove patch 0016, use separate writes in original order.

### 2. Registers That DON'T Need Fixing

| Register | Why |
|----------|-----|
| SQ_RESOURCE_MANAGMENT (0x0d03) | Proprietary NEVER writes, use HW default |
| SQ_PIX_IN_CNTL (0x0d0c) | Proprietary NEVER writes, use HW default |
| RB_EDRAM_INFO (0x0f02) | Kernel driver sets this |
| RB_SAMPLE_POS (0x220a) | Already matches (0x88888888) |
| SQ_INTERPOLATOR_CNTL | Already matches (0xffffffff) |

### 3. Registers That MIGHT Need Investigation

| Register | Current | Issue |
|----------|---------|-------|
| SQ_GPR_MANAGEMENT | Fixed 0x00040401 | Proprietary calculates dynamically |
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x3b (A22X) | KGSL uses 0x02 |
| RBBM_PM_OVERRIDE2 | 0xfff | KGSL uses 0x1a0 |
| PA_CL_CLIP_CNTL | 0 in clear | Proprietary may use 0x43f |

---

## Recommended Actions

### Immediate (High Priority)

1. ✅ **Remove patch 0016** - Register write order matters
2. ✅ **Remove patch 0017** - Registers not used by proprietary

### Medium Priority (Test After Above)

3. **Test VGT_VERTEX_REUSE_BLOCK_CNTL = 0x02** for A22X
   - Currently: 0x3b
   - KGSL reference: 0x02

4. **Test PA_CL_CLIP_CNTL initialization**
   - May need non-zero init value

### Low Priority (If Issues Persist)

5. **Dynamic SQ_GPR_MANAGEMENT**
   - Proprietary calculates per-shader
   - Current fixed allocation may be suboptimal

6. **RBBM_PM_OVERRIDE2 = 0x1a0**
   - May improve power/timing
   - Risk: Clock gating issues

---

## Kernel vs Mesa Responsibility

| Component | Handled By |
|-----------|-----------|
| GPU power management | Kernel (msm/drm) |
| Memory management | Kernel (GPUMMU) |
| Command submission | Kernel |
| Firmware loading | Kernel |
| RB_EDRAM_INFO | Kernel ✅ |
| All SQ/RB/PA registers | Mesa (freedreno) |
| Shader compilation | Mesa |
| Blend/depth state | Mesa |

**Conclusion:** Visual artifacts are Mesa/freedreno issues. Kernel is working correctly.

---

*Generated: February 19, 2026*
*Analysis: Claude Code with Ghidra decompilation*
