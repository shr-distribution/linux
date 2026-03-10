# IOMMU/MMU Analysis Report: Legacy webOS vs Linux 6.18

**Date:** 2026-01-29
**Device:** HP TouchPad (APQ8060 / MSM8660)
**Kernel Comparison:** webOS 3.0.5 (2.6.35-based) vs mainline 6.18

---

## Executive Summary

The APQ8060 SoC contains **12 hardware IOMMU units** with **21 context banks** serving
GPU, display, camera, video codec, and other multimedia subsystems. The legacy webOS
kernel has these fully operational. The 6.18 kernel has **no IOMMU driver enabled**
(`CONFIG_MSM_IOMMU` is not set), which means:

1. The GPU (Adreno 220) works using its own internal MMU (`a2xx_gpummu`) - independent
   of system IOMMU - but the `iommus` property in its DT node may cause probe issues
2. MDP4 display operates without IOMMU (graceful fallback) but with reduced capability
3. All other multimedia IOMMU-dependent subsystems (camera, video codec, rotator) are
   non-functional without IOMMU

**Critical issues found:**
- `CONFIG_MSM_IOMMU` not enabled (no IOMMU driver compiled)
- `CONFIG_CMA` not enabled in debug defconfig (GPU MMU page table allocation may fail)
- `gpu_iommu` node enabled in DT but no driver to service it (potential probe deferral)
- MDP IOMMU nodes explicitly disabled due to driver issue with multiple instances
- `CONFIG_DRM_MSM_Z180` not enabled (no 2D GPU acceleration)

---

## 1. IOMMU Hardware on APQ8060

The APQ8060/MSM8660 has 12 independent IOMMU units at fixed physical addresses:

| ID | Device | Base Address | Context Banks | Purpose |
|----|--------|-------------|---------------|---------|
| 0 | JPEGD | 0x07300000 | 2 (src, dst) | JPEG decoder |
| 1 | VPE | 0x07400000 | 2 (src, dst) | Video post-processor |
| 2 | MDP0 | 0x07500000 | 2 (vg1, rgb1) | Display port 0 |
| 3 | MDP1 | 0x07600000 | 2 (vg2, rgb2) | Display port 1 |
| 4 | ROT | 0x07700000 | 2 (src, dst) | Image rotator |
| 5 | IJPEG | 0x07800000 | 2 (src, dst) | JPEG encoder |
| 6 | VFE | 0x07900000 | 2 (imgwr, misc) | Camera front-end |
| 7 | VCODEC_A | 0x07A00000 | 2 (stream, mm1) | Video codec A |
| 8 | VCODEC_B | 0x07B00000 | 1 (mm2) | Video codec B |
| 9 | GFX3D | 0x07C00000 | 2 (user, priv) | Adreno 220 3D GPU |
| 10 | GFX2D0 | 0x07D00000 | 1 (2d0) | Z180 2D GPU #0 |
| 11 | GFX2D1 | 0x07E00000 | 1 (2d1) | Z180 2D GPU #1 |

Each IOMMU unit has:
- Two-level page tables (FL: 16KB/4096 entries, SL: 4KB/256 entries per table)
- Hardware table walk (HTW)
- Supports 4KB, 64KB, 1MB, 16MB page/section sizes
- Separate secure and non-secure IRQs
- Per-context ASID tagging for TLB entries
- Fault stalling with interrupt notification

---

## 2. Legacy webOS Kernel Configuration

### 2.1 IOMMU Driver

**Source files:**
- `arch/arm/mach-msm/iommu.c` (734 lines) - Core IOMMU driver
- `arch/arm/mach-msm/iommu_dev.c` (418 lines) - IOMMU device driver
- `arch/arm/mach-msm/devices-msm8x60-iommu.c` (902 lines) - Platform device definitions
- `arch/arm/mach-msm/include/mach/iommu_hw-8xxx.h` - Register definitions

**Initialization sequence:**
1. `msm8x60_iommu_init()` (subsys_initcall) - Registers 12 IOMMU + 21 context devices
2. `msm_iommu_probe()` - Maps registers, validates IDR, registers fault IRQ
3. `msm_iommu_ctx_probe()` - Programs M2V tables, VMID, CBNDX, ASID for each MID

**Context bank programming:**
- TLB miss hardware table walk enabled (TLBMCFG=3, V2PCFG=3)
- Context fault interrupts enabled (CFEIE=1)
- Fault stalling enabled (CFCFG=1)
- L2 cache coherency for slave port
- TEX remap enabled
- BFB prefetching enabled

### 2.2 GPU Memory Management (KGSL)

The legacy kernel uses a **proprietary KGSL driver** with its own GPU MMU:

```
CONFIG_MSM_KGSL=y
CONFIG_MSM_KGSL_2D=y
CONFIG_MSM_KGSL_MMU=y
CONFIG_KGSL_PER_PROCESS_PAGE_TABLE=y
CONFIG_MSM_KGSL_PAGE_TABLE_SIZE=0xFFF0000   (≈256MB VA space)
CONFIG_MSM_KGSL_PAGE_TABLE_COUNT=8
CONFIG_MSM_KGSL_MMU_PAGE_FAULT=y
CONFIG_MSM_KGSL_DISABLE_SHADOW_WRITES=y
```

- GPU uses its own KGSL MMU, NOT the system IOMMU
- 256MB virtual address space per process
- 8 pre-allocated page table pools
- Independent from system IOMMU (parallel MMU)

### 2.3 Display (MDP4)

- MDP4 uses system IOMMU via MDP0/MDP1 IOMMU instances
- Framebuffer mapped through IOMMU context banks
- VG (video/graphics) and RGB contexts separated
- Enables non-contiguous physical memory for framebuffers

### 2.4 Defconfig IOMMU Settings

```
# CONFIG_SMMU is not set          (FSM alternative - not used)
# CONFIG_VCM is not set           (Virtual Contiguous Memory - not used)
CONFIG_MSM_KGSL_MMU=y             (KGSL manages its own GPU page tables)
```

Note: The legacy kernel's IOMMU is compiled into `mach-msm/` code, not as a
separate CONFIG option. It's always built when targeting MSM8x60.

---

## 3. Linux 6.18 Kernel Configuration

### 3.1 IOMMU Driver Status

**Build output (.config):**
```
CONFIG_IOMMU_SUPPORT=y              # Framework enabled
CONFIG_IOMMU_IO_PGTABLE=y           # Page table support
# CONFIG_IOMMU_IO_PGTABLE_ARMV7S is not set  # REQUIRED by MSM_IOMMU - MISSING
# CONFIG_MSM_IOMMU is not set       # *** DRIVER NOT COMPILED ***
# CONFIG_ARM_SMMU is not set         # Modern ARM SMMU - not applicable
# CONFIG_QCOM_IOMMU is not set       # Newer Qualcomm IOMMU - not applicable
```

The mainline `drivers/iommu/msm_iommu.c` driver exists and handles
`compatible = "qcom,msm8660-iommu"` (line 840). It is a `builtin_platform_driver`.
When `CONFIG_MSM_IOMMU=y` is set, it will `select IOMMU_IO_PGTABLE_ARMV7S`
automatically (Kconfig line 186).

### 3.2 Device Tree IOMMU Nodes

**In `qcom-msm8660.dtsi`:** All 12 IOMMU nodes defined with `status = "disabled"`.

**In `qcom-apq8060-tenderloin-common.dtsi`:**
```dts
/* GPU IOMMU - ENABLED */
&gpu_iommu {
    status = "okay";
};

/* MDP IOMMUs - DISABLED (commented out) */
/* &mdp_port0_iommu {
    status = "okay";
};
&mdp_port1_iommu {
    status = "okay";
}; */
```

Comment says: *"MDP IOMMUs disabled due to msm_iommu driver issues with multiple
IOMMU instances referencing the same device."*

GPU node references IOMMU:
```dts
gpu: adreno@4300000 {
    iommus = <&gpu_iommu 0>, <&gpu_iommu 1>, ... <&gpu_iommu 31>;
};
```

MDP node references IOMMUs (even though they're disabled):
```dts
mdp: mdp@5100000 {
    iommus = <&mdp_port0_iommu 0>, ... <&mdp_port1_iommu 10>;
};
```

### 3.3 GPU Memory Management (DRM MSM)

The 6.18 kernel uses the mainline DRM MSM framework:

**A2xx GPU MMU** (`drivers/gpu/drm/msm/adreno/a2xx_gpummu.c`):
- The A2xx (Adreno 220) driver uses its **own GPU MMU**, NOT system IOMMU
- `a2xx_create_vm()` calls `a2xx_gpummu_new()` - independent of system IOMMU
- GPU page table: `dma_alloc_attrs(DMA_ATTR_FORCE_CONTIGUOUS)` - needs CMA or DMA pool
- VA range: 16MB - (0xfff * 64KB) ≈ 4GB, page size 4KB
- Table size: ~256KB contiguous physical memory

**System IOMMU path** (`drivers/gpu/drm/msm/msm_iommu.c`):
- `msm_iommu_new()` at line 728: `if (!device_iommu_mapped(dev)) return ERR_PTR(-ENODEV);`
- Only used by A3xx+ GPUs, NOT by A2xx
- A2xx completely bypasses this check

**MDP4 IOMMU fallback** (`drivers/gpu/drm/msm/msm_kms.c:193-204`):
```c
if (device_iommu_mapped(mdp_dev))
    iommu_dev = mdp_dev;
else {
    drm_info(dev, "no IOMMU, continuing without virtual address space\n");
    return NULL;
}
```
MDP4 gracefully operates without IOMMU but with limitations.

### 3.4 DRM MSM Configuration

```
CONFIG_DRM_MSM=y
CONFIG_DRM_MSM_KMS=y
CONFIG_DRM_MSM_MDP4=y
# CONFIG_DRM_MSM_Z180 is not set   # *** Z180 2D GPU DISABLED ***
CONFIG_DRM_MSM_KMS_FBDEV=y
```

### 3.5 CMA Configuration

**tenderloin_defconfig:**
```
CONFIG_CMA=y
CONFIG_DMA_CMA=y
CONFIG_CMA_SIZE_MBYTES=256
```

**tenderloin_debug_defconfig:**
```
# CONFIG_CMA is not set              # *** CMA DISABLED ***
```

**Build output (.config):**
```
# CONFIG_CMA is not set              # *** Built with debug defconfig ***
```

### 3.6 Reserved Memory

```dts
drm_smi_mem: framebuffer@38300000 {
    compatible = "shared-dma-pool";
    reg = <0x38300000 0x3d00000>;  /* 61MB - exact webOS size */
    no-map;
};
```

MDP4 uses `memory-region = <&drm_smi_mem>` for framebuffer allocation from SMI.

---

## 4. Gap Analysis: What's Missing

### 4.1 CRITICAL: IOMMU Driver Not Enabled

| Aspect | Legacy webOS | Linux 6.18 | Impact |
|--------|-------------|------------|--------|
| IOMMU driver | Built-in (mach-msm) | `CONFIG_MSM_IOMMU` not set | No IOMMU for any device |
| GPU IOMMU | Available (GFX3D) | Node enabled, no driver | Potential probe deferral |
| MDP IOMMU | Active (MDP0+MDP1) | Nodes disabled | MDP runs without VA space |
| Media IOMMU | Active (VFE, VCODEC, etc.) | Nodes disabled | Camera/video non-functional |

**Fix:** Enable `CONFIG_MSM_IOMMU=y` in all defconfigs. This will also select
`CONFIG_IOMMU_IO_PGTABLE_ARMV7S` automatically.

### 4.2 CRITICAL: CMA Disabled in Debug Build

The current build uses `tenderloin_debug_defconfig` which has CMA disabled.
The A2xx GPU MMU allocates its page table with `DMA_ATTR_FORCE_CONTIGUOUS`:

```c
// a2xx_gpummu.c:110-111
gpummu->table = dma_alloc_attrs(dev, TABLE_SIZE + 32, &gpummu->pt_base,
    GFP_KERNEL | __GFP_ZERO, DMA_ATTR_FORCE_CONTIGUOUS);
```

Without CMA, this ~256KB contiguous allocation relies on the DMA coherent pool or
boot-time contiguous pages. On a fragmented system, this can fail with `-ENOMEM`.

**Fix:** Enable `CONFIG_CMA=y` and `CONFIG_DMA_CMA=y` with
`CONFIG_CMA_SIZE_MBYTES=256` in `tenderloin_debug_defconfig`.

### 4.3 HIGH: GPU IOMMU DT Node May Cause Probe Deferral

The GPU DT node has `iommus = <&gpu_iommu 0>...` and `gpu_iommu` has
`status = "okay"`. With `CONFIG_IOMMU_SUPPORT=y`, the kernel's `of_iommu_configure()`
will attempt to find the IOMMU for the GPU device. Since no driver probes the IOMMU
(CONFIG_MSM_IOMMU is not set), this could:

1. Return `-EPROBE_DEFER` indefinitely - blocking GPU initialization
2. Return `-ENODEV` - possibly handled gracefully by A2xx driver

The A2xx driver bypasses the system IOMMU entirely (uses `a2xx_gpummu_new()` instead
of `msm_iommu_gpu_new()`), so if the probe succeeds, the GPU works without IOMMU.

**Fix:** Either:
- Enable `CONFIG_MSM_IOMMU=y` so the IOMMU driver probes (preferred)
- Or disable `gpu_iommu` in DT if IOMMU is not needed for GPU (workaround)

### 4.4 HIGH: MDP IOMMU Disabled

MDP IOMMUs are commented out in the device tree. The comment says this is due to
"msm_iommu driver issues with multiple IOMMU instances referencing the same device."

Without MDP IOMMU:
- MDP4 works via direct physical addressing
- No cursor support
- Limited GEM buffer management
- No virtual address space for display composition

The legacy kernel uses both MDP0 and MDP1 IOMMUs with separate context banks for
VG (video/graphics) and RGB pipes. This provides:
- Flexible non-contiguous framebuffer memory
- Memory protection between display layers
- Multiple independent video/graphics layers

**Fix:** Investigate and fix the multi-instance IOMMU driver issue, then enable
MDP IOMMU nodes.

### 4.5 MEDIUM: Z180 2D GPU Disabled

`CONFIG_DRM_MSM_Z180` is not set in any defconfig. The APQ8060 has two Z180
2D graphics engines (GFX2D0 at 0x04100000, GFX2D1 at 0x04200000) that
provide hardware-accelerated 2D operations. The legacy kernel uses these via
KGSL with CONFIG_MSM_KGSL_2D=y.

The Z180 engines have their own IOMMU units (GFX2D0_IOMMU, GFX2D1_IOMMU)
which are currently disabled in the device tree.

**Fix:** Enable `CONFIG_DRM_MSM_Z180=y` and enable `gfx2d0_iommu` / `gfx2d1_iommu`
DT nodes (after enabling CONFIG_MSM_IOMMU).

### 4.6 LOW: Missing KGSL-Specific Features

The legacy KGSL driver had features not present in the DRM MSM A2xx driver:

| Feature | Legacy KGSL | DRM MSM A2xx |
|---------|-------------|--------------|
| Per-process page tables | Yes (CONFIG_KGSL_PER_PROCESS_PAGE_TABLE) | No (single table) |
| Page table pools | 8 pre-allocated | On-demand |
| Shadow register writes | Configurable | Not applicable |
| GPU power management | Custom KGSL PM | Standard devfreq |
| Dedicated GPU VA space | 256MB per process | ~4GB shared |

These are architectural differences, not bugs. The DRM framework handles these
differently and the A2xx GPU MMU implementation is simpler but functional.

---

## 5. Legacy Kernel Features NOT Needed

### 5.1 VCM (Virtual Contiguous Memory)

VCM was disabled even in the legacy kernel (`# CONFIG_VCM is not set`).
Not needed in 6.18.

### 5.2 PMEM (Physical Memory Allocator)

Legacy `drivers/misc/pmem.c` replaced by CMA/DMA-BUF in modern kernels.
Not needed in 6.18.

### 5.3 FSM SMMU

Alternative SMMU implementation for FSM SoCs. Never used on TouchPad.
Not needed in 6.18.

---

## 6. Recommended Actions (Priority Order)

### Immediate (Graphics Stack)

1. **Enable CMA in debug defconfig** - Add to `tenderloin_debug_defconfig`:
   ```
   CONFIG_CMA=y
   CONFIG_DMA_CMA=y
   CONFIG_CMA_SIZE_MBYTES=256
   ```

2. **Enable MSM IOMMU driver** - Add to all defconfigs:
   ```
   CONFIG_MSM_IOMMU=y
   ```
   (Automatically selects `CONFIG_IOMMU_IO_PGTABLE_ARMV7S`)

3. **Verify GPU probe succeeds** - Check dmesg after enabling MSM_IOMMU:
   - `msm_iommu` should log `device mapped at <addr>, irq <n> with <n> ctx banks`
   - GPU should probe without `-EPROBE_DEFER`

### Next Steps (Display Enhancement)

4. **Fix MDP IOMMU multi-instance issue** - Investigate what causes
   "msm_iommu driver issues with multiple IOMMU instances referencing the
   same device" and fix it in the mainline msm_iommu.c driver

5. **Enable MDP IOMMU nodes** - Uncomment in tenderloin-common.dtsi:
   ```dts
   &mdp_port0_iommu {
       status = "okay";
   };
   &mdp_port1_iommu {
       status = "okay";
   };
   ```

### Future (2D Acceleration)

6. **Enable Z180 2D GPU** - Add to defconfigs:
   ```
   CONFIG_DRM_MSM_Z180=y
   ```
   Enable DT nodes: `&gfx2d0_iommu`, `&gfx2d1_iommu`

---

## 7. IOMMU Register Reference

### Physical Base Addresses and IRQs

| Device | Base | Size | Non-Secure IRQ | Secure IRQ |
|--------|------|------|---------------|------------|
| JPEGD | 0x07300000 | 1MB | GIC+66 (98) | GIC+65 (97) |
| VPE | 0x07400000 | 1MB | GIC+52 (84) | GIC+51 (83) |
| MDP0 | 0x07500000 | 1MB | GIC+64 (96) | GIC+63 (95) |
| MDP1 | 0x07600000 | 1MB | GIC+62 (94) | GIC+61 (93) |
| ROT | 0x07700000 | 1MB | GIC+60 (92) | GIC+59 (91) |
| IJPEG | 0x07800000 | 1MB | GIC+68 (100) | GIC+67 (99) |
| VFE | 0x07900000 | 1MB | GIC+54 (86) | GIC+53 (85) |
| VCODEC_A | 0x07A00000 | 1MB | GIC+58 (90) | GIC+57 (89) |
| VCODEC_B | 0x07B00000 | 1MB | GIC+56 (88) | GIC+55 (87) |
| GFX3D | 0x07C00000 | 1MB | GIC+70 (102) | GIC+69 (101) |
| GFX2D0 | 0x07D00000 | 1MB | GIC+72 (104) | GIC+71 (103) |
| GFX2D1 | 0x07E00000 | 1MB | GIC+211 (243) | GIC+210 (242) |

### GPU Register Blocks

| Device | Base | Size | IRQ |
|--------|------|------|-----|
| GFX3D (Adreno 220) | 0x04300000 | 128KB | GFX3D_IRQ |
| GFX2D0 (Z180) | 0x04100000 | 4KB | GFX2D0_IRQ |
| GFX2D1 (Z180) | 0x04200000 | 4KB | GFX2D1_IRQ |

### Context Bank MID Mappings

**GFX3D IOMMU:**
- Context 0 (gfx3d_user): MIDs 0-15 (user-mode GPU)
- Context 1 (gfx3d_priv): MIDs 16-31 (privileged GPU)

**MDP0 IOMMU:**
- Context 0 (mdp_vg1): MIDs 0, 2 (VG pipe 1)
- Context 1 (mdp_rgb1): MIDs 1, 3-10 (RGB pipe 1)

**MDP1 IOMMU:**
- Context 0 (mdp_vg2): MIDs 0, 2 (VG pipe 2)
- Context 1 (mdp_rgb2): MIDs 1, 3-10 (RGB pipe 2)

**GFX2D0/GFX2D1 IOMMU:**
- Context 0: MIDs 0-7

---

## 8. Key Source Files

### Legacy Kernel (webOS)
- `arch/arm/mach-msm/iommu.c` - IOMMU core driver
- `arch/arm/mach-msm/iommu_dev.c` - IOMMU device driver
- `arch/arm/mach-msm/devices-msm8x60-iommu.c` - Platform device definitions
- `arch/arm/mach-msm/include/mach/iommu_hw-8xxx.h` - Hardware registers
- `arch/arm/mach-msm/include/mach/msm_iomap-8x60.h` - Physical addresses
- `drivers/gpu/msm/kgsl_mmu.h` - KGSL GPU MMU
- `arch/arm/mach-msm/board-tenderloin.c` - Board-level KGSL/MDP config

### Linux 6.18
- `drivers/iommu/msm_iommu.c` - Mainline MSM IOMMU driver (line 840: `qcom,msm8660-iommu`)
- `drivers/iommu/msm_iommu.h` - Driver header
- `drivers/iommu/msm_iommu_hw-8xxx.h` - Hardware register definitions
- `drivers/gpu/drm/msm/msm_iommu.c` - DRM IOMMU abstraction (line 728: `device_iommu_mapped()`)
- `drivers/gpu/drm/msm/adreno/a2xx_gpummu.c` - A2xx GPU MMU (independent of system IOMMU)
- `drivers/gpu/drm/msm/adreno/a2xx_gpu.c` - A2xx GPU driver (line 491: `a2xx_gpummu_new()`)
- `drivers/gpu/drm/msm/msm_kms.c` - KMS IOMMU fallback (line 197-204: no-IOMMU path)
- `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi` - SoC IOMMU DT nodes
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - Board IOMMU overrides
