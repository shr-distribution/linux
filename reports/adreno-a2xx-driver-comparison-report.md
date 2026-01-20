# Adreno A2XX Driver Comparison Report

## Mesa Freedreno vs Legacy KGSL vs Mainline DRM/MSM Kernel Drivers

**Date:** January 2026
**Target Device:** HP TouchPad (APQ8060 with Adreno 220)
**Analysis Scope:** GPU initialization, command submission, MMU handling, and hardware workarounds

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Driver Overview](#driver-overview)
3. [Critical Issues](#critical-issues)
4. [Detailed Comparison](#detailed-comparison)
5. [Hardware Workarounds](#hardware-workarounds)
6. [Register Configuration Differences](#register-configuration-differences)
7. [MMU Implementation](#mmu-implementation)
8. [Recommendations](#recommendations)
9. [Appendix: Source References](#appendix-source-references)

---

## Executive Summary

This report analyzes the Mesa freedreno userspace driver for Adreno A2XX GPUs and compares it against two kernel drivers:

1. **Legacy KGSL Driver** - Original Qualcomm/Palm driver from webOS
2. **Mainline DRM/MSM Driver** - Modern Linux kernel driver

### Key Findings

| Finding | Severity | Affected Driver |
|---------|----------|-----------------|
| VGT DMA alignment workaround missing | **HIGH** | Mainline |
| MH_CLNT_INTF_CTRL_CONFIG2 not configured | **MEDIUM** | Mainline |
| PM_OVERRIDE2 register values incorrect | **MEDIUM** | Mainline |
| Post-draw WFI synchronization undocumented | **LOW** | Both kernel |
| Vertex count limitation (32K) | **INFO** | Mesa/Userspace |

### Recommendation

The mainline DRM/MSM driver is missing critical hardware workarounds present in both the legacy KGSL driver and documented in the Mesa freedreno driver. The most critical is the **VGT DMA alignment workaround** which can cause GPU hangs and MMU page faults on A200/A220 hardware.

---

## Driver Overview

### Mesa Freedreno A2XX Userspace Driver

**Location:** `src/gallium/drivers/freedreno/a2xx/`

The freedreno driver is a Gallium3D driver providing OpenGL ES support for Adreno GPUs. For A2XX, it includes:

| File | Purpose | Size |
|------|---------|------|
| `fd2_emit.c` | GPU command emission, state setup | ~20KB |
| `fd2_gmem.c` | GMEM tile-based rendering | ~28KB |
| `fd2_draw.c` | Draw call implementation | ~22KB |
| `ir2_nir.c` | NIR to IR2 shader translation | ~35KB |
| `fd2_context.c` | Context management | ~8KB |
| `fd2_screen.c` | Device capabilities | ~6KB |

**Key Characteristics:**
- Implements tile-based deferred rendering (TBDR) using on-chip GMEM
- Contains extensive hardware quirk documentation in comments
- Splits draws exceeding 32,766 vertices due to hardware limitations
- Includes A20x-specific DMA alignment workarounds

### Legacy KGSL Kernel Driver

**Location:** `drivers/gpu/msm/` (webOS kernel)

The KGSL (Kernel Graphics Support Library) driver is the original Qualcomm driver used in Android and webOS devices.

| File | Purpose | Lines |
|------|---------|-------|
| `kgsl_yamato.c` | Yamato (A2XX) GPU core | ~1,370 |
| `kgsl_ringbuffer.c` | Command submission | ~805 |
| `kgsl_mmu.c` | Memory management unit | ~1,100 |
| `kgsl_drawctxt.c` | Draw context management | ~800 |
| `kgsl_g12.c` | G12 2D GPU (OpenVG) | ~900 |
| `yamato_reg.h` | Register definitions | ~3,500 |

**Key Characteristics:**
- Proprietary IOCTL interface (not DRM)
- Supports both Yamato (3D) and G12 (2D) GPU cores
- Per-process page table support
- Extensive power management and clock gating
- Hardware-specific workarounds for multiple chip revisions

### Mainline DRM/MSM Kernel Driver

**Location:** `drivers/gpu/drm/msm/adreno/`

The modern DRM-based driver integrated into the mainline Linux kernel.

| File | Purpose | Lines |
|------|---------|-------|
| `a2xx_gpu.c` | A2XX GPU implementation | ~560 |
| `a2xx_gpummu.c` | GPU MMU for A2XX | ~131 |
| `a2xx_catalog.c` | Device specifications | ~48 |
| `adreno_gpu.c` | Common Adreno functions | ~1,264 |

**Key Characteristics:**
- Standard DRM/KMS interface
- Simplified codebase compared to KGSL
- Uses common MSM GPU infrastructure
- Missing some hardware-specific workarounds

---

## Critical Issues

### Issue 1: VGT DMA Alignment Workaround (HIGH SEVERITY)

#### Description

The Adreno A200/A220 GPU has a hardware bug where the VGT (Vertex Geometry Tessellation) unit can cause DMA alignment issues, leading to MMU page faults. This occurs when:
- Switching page tables
- Performing indexed draws
- Reading binning data

#### Mesa Documentation

From `fd2_draw.c`:
```c
/* a20x hw bug.. also, curved polygon rendering seems to work better
 * this way. */
/* wait for DMA to finish and dummy draw one triangle with
 * indexes 0,0,0. with PRE_FETCH_CULL_ENABLE | GRP_CULL_ENABLE
 */
```

#### Legacy KGSL Implementation

From `kgsl_yamato.c:373-410`:
```c
if (flags & KGSL_MMUFLAGS_PTUPDATE &&
    device->chip_id != KGSL_CHIPID_LEIA_REV470) {
    /* HW workaround: to resolve MMU page fault interrupts
     * caused by the VGT. It prevents the CP PFP from filling
     * the VGT DMA request fifo too early, thereby ensuring
     * that the VGT will not fetch vertex/bin data until
     * after the page table base register has been updated.
     *
     * Two null DRAW_INDX_BIN packets are inserted right
     * after the page table base update, followed by a
     * wait for idle. */
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = (0x4 << 16) | (REG_PA_SU_SC_MODE_CNTL - 0x2000);
    *cmds++ = 0;          /* disable faceness generation */
    *cmds++ = pm4_type3_packet(PM4_SET_BIN_BASE_OFFSET, 1);
    *cmds++ = device->mmu.dummyspace.gpuaddr;
    *cmds++ = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
    *cmds++ = 0;          /* viz query info */
    *cmds++ = 0x0003C004; /* draw indicator */
    *cmds++ = 0;          /* bin base */
    *cmds++ = 3;          /* bin size */
    *cmds++ = device->mmu.dummyspace.gpuaddr; /* dma base */
    *cmds++ = 6;          /* dma size */
    /* ... second DRAW_INDX_BIN packet ... */
    *cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmds++ = 0x00000000;
}
```

#### Mainline Implementation

**NOT PRESENT** - The mainline `a2xx_gpu.c` has no VGT DMA workaround.

#### Impact

- Random GPU hangs during normal operation
- MMU page fault interrupts
- Corruption when switching contexts or page tables
- Instability in graphics-heavy applications

#### Affected Hardware

- Adreno 200 (A200)
- Adreno 220 (A220)
- NOT affected: Adreno 225 (Leia REV470)

#### Recommendation

Implement the VGT DMA workaround in the mainline driver. This can be done either:
1. In the `a2xx_submit()` function for every submission
2. In a new `a2xx_switch_pagetable()` function called during context switches

---

### Issue 2: MH_CLNT_INTF_CTRL_CONFIG Registers (MEDIUM SEVERITY)

#### Description

The Memory Hub Client Interface Control registers configure memory access patterns and timing. Different chip revisions require different values.

#### Legacy KGSL Implementation

From `kgsl_yamato.c:816-828`:
```c
if (device->chip_id != KGSL_CHIPID_LEIA_REV470) {
    kgsl_yamato_regwrite(device, REG_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030f27);
    kgsl_yamato_regwrite(device, REG_MH_CLNT_INTF_CTRL_CONFIG2, 0x00472747);
}

/* Remove 1k boundary check in z470 to avoid GPU hang.
   Notice that, this solution won't work if both EBI and SMI are used */
if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
    kgsl_yamato_regwrite(device, REG_MH_CLNT_INTF_CTRL_CONFIG1, 0x00032f07);
}
```

#### Mainline Implementation

From `a2xx_gpu.c:179-180`:
```c
if (!adreno_is_a20x(adreno_gpu))
    gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG1, 0x00032f07);
```

#### Comparison Table

| Register | A200 (Legacy) | A220 (Legacy) | A225/Leia (Legacy) | Mainline |
|----------|---------------|---------------|---------------------|----------|
| CONFIG1 | 0x00030f27 | 0x00030f27 | 0x00032f07 | 0x00032f07 (non-A20x) |
| CONFIG2 | 0x00472747 | 0x00472747 | Not set | **Not set** |

#### Issues

1. **CONFIG2 never set in mainline** - The legacy driver sets CONFIG2 for A200/A220
2. **A200 uses wrong CONFIG1 value** - Mainline skips CONFIG1 entirely for A20x
3. **1K boundary check comment** - Legacy explicitly mentions this is needed to avoid GPU hangs

#### Impact

- Potential memory access issues on A200
- Suboptimal memory interface timing
- Possible GPU hangs related to 1K boundary crossing

#### Recommendation

Update mainline to match legacy driver's register configuration based on chip revision.

---

### Issue 3: PM_OVERRIDE2 Register (MEDIUM SEVERITY)

#### Description

The `RBBM_PM_OVERRIDE2` register controls clock gating for various GPU subsystems. Incorrect values can cause power management issues or premature clock gating.

#### Legacy KGSL Implementation

From `kgsl_yamato.c:792-837`:
```c
// During init, enable all clocks:
if (device->chip_id == CHIP_REV_251)
    kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x000000ff);
else
    kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0xffffffff);

// After soft reset, restore normal clock gating:
if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
    kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0);
else
    kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x1a0);
```

#### Mainline Implementation

From `a2xx_gpu.c:186`:
```c
gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0); /* 0x80/0x1a0 for a22x? */
```

#### Comparison Table

| Chip | Init Value (Legacy) | Final Value (Legacy) | Mainline |
|------|---------------------|----------------------|----------|
| A200 | 0xffffffff | 0x00000000 | 0x00000000 |
| A220 | 0xffffffff | 0x00000000 | 0x00000000 |
| A225 (Leia) | 0xffffffff | 0x000001a0 | 0x00000000 |
| REV251 | 0x000000ff | 0x00000000 | 0x00000000 |

#### Issues

1. **A225/Leia needs 0x1a0** - Mainline uses 0x0 for all chips
2. **The comment admits uncertainty** - `/* 0x80/0x1a0 for a22x? */`

#### Impact

- Clock gating issues on A225
- Potential power management problems
- Possible GPU subsystem instability

#### Recommendation

Set PM_OVERRIDE2 to 0x1a0 for A225 chips (Leia REV470).

---

### Issue 4: Vertex Count Limitation (INFORMATIONAL)

#### Description

The A2XX GPU hardware has limitations on the number of vertices that can be processed in a single draw call.

#### Mesa Documentation

From `fd2_draw.c`:
```c
/* a2xx can draw only 65535 vertices at once
 * on a22x the field in the draw command is 32bits but seems limited too
 * using a limit of 32k because it fixes an unexplained hang
 * 32766 works for all primitives */
```

#### Impact

- Mesa handles this by splitting large draws
- Kernel drivers should not make assumptions about draw sizes
- Not a kernel driver bug, but important for understanding workload patterns

---

## Hardware Workarounds

### A20x-Specific Workarounds

| Workaround | Description | Legacy | Mainline | Mesa |
|------------|-------------|--------|----------|------|
| VGT DMA alignment | Dummy draws after PT switch | ✅ | ❌ | ✅ |
| Post-draw WFI | Wait-for-idle after draws | N/A | N/A | ✅ |
| DMA sync for binning | Wait for DMA before indexed draw | N/A | N/A | ✅ |

### A22x-Specific Workarounds

| Workaround | Description | Legacy | Mainline | Mesa |
|------------|-------------|--------|----------|------|
| 1K boundary check disable | CONFIG1 = 0x00032f07 | ✅ | ✅ | N/A |
| Post-draw register clear | Unknown register set to 0 | N/A | N/A | ✅ |
| Hardware binning different | A22x binning like A3xx | N/A | N/A | ✅ |

### A225 (Leia)-Specific Workarounds

| Workaround | Description | Legacy | Mainline | Mesa |
|------------|-------------|--------|----------|------|
| SQ_FLOW_CONTROL | Set to 0x18000000 | ❌ | ✅ | N/A |
| PM_OVERRIDE2 | Set to 0x1a0 | ✅ | ❌ | N/A |
| Skip VGT workaround | Not needed for Leia | ✅ | N/A | ✅ |

---

## Register Configuration Differences

### Hardware Initialization Sequence

#### Memory Arbiter Configuration

```c
// Legacy (kgsl_yamato.c)
#define KGSL_CFG_YAMATO_MHARB \
    (0x10 \
        | (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
        | (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
        | (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
        | (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
        | (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

// Mainline (a2xx_gpu.c)
gpu_write(gpu, REG_A2XX_MH_ARBITER_CONFIG,
    A2XX_MH_ARBITER_CONFIG_SAME_PAGE_LIMIT(16) |
    A2XX_MH_ARBITER_CONFIG_L1_ARB_ENABLE |
    A2XX_MH_ARBITER_CONFIG_L1_ARB_HOLD_ENABLE |
    A2XX_MH_ARBITER_CONFIG_PAGE_SIZE(1) |
    A2XX_MH_ARBITER_CONFIG_TC_REORDER_ENABLE |
    A2XX_MH_ARBITER_CONFIG_TC_ARB_HOLD_ENABLE |
    A2XX_MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_ENABLE |  // Different!
    A2XX_MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT(8) |
    A2XX_MH_ARBITER_CONFIG_CP_CLNT_ENABLE |
    A2XX_MH_ARBITER_CONFIG_VGT_CLNT_ENABLE |
    A2XX_MH_ARBITER_CONFIG_TC_CLNT_ENABLE |
    A2XX_MH_ARBITER_CONFIG_RB_CLNT_ENABLE |
    A2XX_MH_ARBITER_CONFIG_PA_CLNT_ENABLE);
```

**Difference:** Mainline enables `IN_FLIGHT_LIMIT_ENABLE` while legacy does not. This may affect memory throughput.

#### RBBM_CNTL Register

Both drivers use `0x00004442`. Mesa documentation notes:
> "kgsl uses 0x0000ffff for a20x"

This suggests an older KGSL version used a different value for A200.

#### RBBM_DEBUG Register

Both drivers set `0x00080000`, which is undocumented but appears to be a debug/performance setting.

---

## MMU Implementation

### Page Table Structure

| Aspect | Legacy KGSL | Mainline DRM |
|--------|-------------|--------------|
| VA Base | `KGSL_PAGETABLE_BASE` (config) | `SZ_16M` (16MB) |
| VA Range | `CONFIG_MSM_KGSL_PAGE_TABLE_SIZE` | `0xfff * SZ_64K` (~64MB) |
| Page Size | 4KB | 4KB |
| PTE Format | `physaddr \| protflags` | `(addr + offset) \| prot_bits` |

### TLB Invalidation

#### Legacy KGSL

Uses a sophisticated TLB flush filter with super-PTE tracking:
```c
// From kgsl_mmu.c
pagetable->tlbflushfilter.size = (pagetable->va_range /
    (PAGE_SIZE * GSL_PT_SUPER_PTE * 8)) + 1;
// ...
if (flushtlb) {
    pagetable->tlb_flags = UINT_MAX;
    GSL_TLBFLUSH_FILTER_RESET();
}
```

Only invalidates TLB when entries cross super-PTE boundaries.

#### Mainline DRM

Always performs full TLB invalidation:
```c
// From a2xx_gpummu.c
gpu_write(gpummu->gpu, REG_A2XX_MH_MMU_INVALIDATE,
    A2XX_MH_MMU_INVALIDATE_INVALIDATE_ALL |
    A2XX_MH_MMU_INVALIDATE_INVALIDATE_TC);
```

**Trade-off:** Mainline is simpler and safer but may have slightly higher overhead for frequent map/unmap operations.

### DMA Coherency

#### Legacy KGSL

Uses explicit memory barriers:
```c
mb();
dsb();
outer_sync();
```

#### Mainline DRM

Uses DMA API for coherency:
```c
dma_sync_single_for_device(mmu->dev, gpummu->pt_base, TABLE_SIZE,
    DMA_TO_DEVICE);
```

**Note:** The mainline approach is more portable and correct for modern kernels.

---

## Recommendations

### High Priority

1. **Implement VGT DMA Workaround**

   Add to `a2xx_gpu.c` in the `a2xx_submit()` function or create a new page table switch handler:

   ```c
   static void a2xx_vgt_dma_workaround(struct msm_gpu *gpu)
   {
       struct msm_ringbuffer *ring = gpu->rb[0];

       if (adreno_is_a225(to_adreno_gpu(gpu)))
           return;  /* Not needed for Leia */

       /* Disable faceness generation */
       OUT_PKT3(ring, CP_SET_CONSTANT, 2);
       OUT_RING(ring, (0x4 << 16) | (REG_A2XX_PA_SU_SC_MODE_CNTL - 0x2000));
       OUT_RING(ring, 0);

       /* Two dummy DRAW_INDX_BIN packets */
       /* ... implementation ... */

       /* Wait for idle */
       OUT_PKT3(ring, CP_WAIT_FOR_IDLE, 1);
       OUT_RING(ring, 0);
   }
   ```

### Medium Priority

2. **Fix MH_CLNT_INTF_CTRL_CONFIG Registers**

   Update `a2xx_hw_init()`:
   ```c
   if (adreno_is_a20x(adreno_gpu)) {
       gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030f27);
       gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG2, 0x00472747);
   } else if (!adreno_is_a225(adreno_gpu)) {
       /* A220 */
       gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030f27);
       gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG2, 0x00472747);
   } else {
       /* A225/Leia - disable 1K boundary check */
       gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG1, 0x00032f07);
   }
   ```

3. **Fix PM_OVERRIDE2 for A225**

   Update `a2xx_hw_init()`:
   ```c
   if (adreno_is_a225(adreno_gpu))
       gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0x1a0);
   else
       gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0);
   ```

### Low Priority

4. **Consider Selective TLB Invalidation**

   The current full-invalidation approach is correct and safe. Only optimize if profiling shows it's a bottleneck.

5. **Add Debug Logging for Hardware Quirks**

   Add dev_dbg() calls when applying hardware workarounds to aid debugging.

---

## Appendix: Source References

### Mesa Freedreno

- Repository: https://cgit.freedesktop.org/mesa/mesa/
- Path: `src/gallium/drivers/freedreno/a2xx/`
- Key files: `fd2_emit.c`, `fd2_gmem.c`, `fd2_draw.c`

### Legacy KGSL Driver

- Path: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/`
- Key files: `kgsl_yamato.c`, `kgsl_ringbuffer.c`, `kgsl_mmu.c`

### Mainline DRM/MSM Driver

- Path: `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/drivers/gpu/drm/msm/adreno/`
- Key files: `a2xx_gpu.c`, `a2xx_gpummu.c`, `a2xx_catalog.c`

### Chip Identification

| Marketing Name | Code Name | Chip ID | GMEM |
|----------------|-----------|---------|------|
| Adreno 200 | Yamato | 0x02000000 | 256KB |
| Adreno 200 (i.MX51) | Yamato | 0x02000001 | 128KB |
| Adreno 220 | Leia | 0x02020000 | 512KB |
| Adreno 225 | Leia REV470 | 0x02010000 | 512KB |

### Register References

| Register | Address | Description |
|----------|---------|-------------|
| RBBM_SOFT_RESET | 0x003C | Soft reset control |
| RBBM_CNTL | 0x003B | RBBM configuration |
| RBBM_PM_OVERRIDE1 | 0x039C | Power management override 1 |
| RBBM_PM_OVERRIDE2 | 0x039D | Power management override 2 |
| MH_ARBITER_CONFIG | 0x00A1 | Memory arbiter configuration |
| MH_CLNT_INTF_CTRL_CONFIG1 | 0x00A2 | Client interface control 1 |
| MH_CLNT_INTF_CTRL_CONFIG2 | 0x00A3 | Client interface control 2 |
| MH_MMU_CONFIG | 0x0040 | MMU configuration |
| MH_MMU_INVALIDATE | 0x0045 | TLB invalidation |
| CP_ME_CNTL | 0x01D9 | Micro-engine control |
| RB_EDRAM_INFO | 0x0F02 | EDRAM/GMEM configuration |

---

*Report generated from analysis of Mesa freedreno, legacy KGSL, and mainline DRM/MSM drivers for Adreno A2XX GPUs.*
