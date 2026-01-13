# MSM8660/APQ8060 Interconnect Requirements

**Date:** 2026-01-13 (updated)
**Based on:** webOS kernel analysis (webos-linux-kernel-opal)

---

## Overview

This document catalogs all hardware components that used the `msm_bus_scale` API in the original webOS kernel. These components may benefit from or require interconnect framework integration in mainline Linux for proper bus bandwidth coordination.

---

## Interconnect Consumers Summary

| Component | Master Port(s) | Destination | Status in Mainline |
|-----------|---------------|-------------|-------------------|
| MDP4 Display | MDP_PORT0, MDP_PORT1 | SMI, EBI_CH0 | **ENABLED** (cross-fabric paths fixed) |
| GPU 3D (Adreno 220) | GRAPHICS_3D | EBI_CH0 | **NOT NEEDED** (a2xx driver doesn't use ICC) |
| GPU 2D Core0 | GRAPHICS_2D_CORE0 | SMI, EBI_CH0 | **ENABLED** (Z180 driver added) |
| GPU 2D Core1 | GRAPHICS_2D_CORE1 | SMI, EBI_CH0 | **ENABLED** (Z180 driver added) |
| Camera (VFE) | VFE | SMI, EBI_CH0 | **ENABLED** (CAMSS driver updated) |
| Video Processor (VPE) | VPE | SMI, EBI_CH0 | **ENABLED** (VPE driver added) |
| JPEG Encoder | JPEG_ENC | SMI, EBI_CH0 | **ENABLED** (Gemini driver added) |
| Video Codec Port0 | HD_CODEC_PORT0 | SMI, EBI_CH0 | **ENABLED** (VIDC driver updated) |
| Video Codec Port1 | HD_CODEC_PORT1 | SMI, EBI_CH0 | **ENABLED** (VIDC driver updated) |
| Rotator | ROTATOR | SMI, EBI_CH0 | **ENABLED** (Rotator driver added) |
| DTV (HDMI output) | MDP_PORT0 | SMI, EBI_CH0 | Uses MDP4 paths |
| CPU (for video) | AMPSS_M0 | EBI_CH0, SMI | Not required |

---

## Detailed Component Analysis

### 1. MDP4 Display Controller

**Status: ENABLED (cross-fabric paths fixed)**

**webOS Implementation:**
```c
/* From board-tenderloin.c */
static struct msm_bus_vectors mdp_bus_scale_usecases[] = {
    .src = MSM_BUS_MASTER_MDP_PORT0,
    .dst = MSM_BUS_SLAVE_SMI,      /* SMI memory */
    .dst = MSM_BUS_SLAVE_EBI_CH0,  /* Main memory */
};
```

**Mainline Implementation:**
The `mdp4_kms.c` driver has `mdp4_setup_interconnect()` which uses `msm_icc_get()`.
Cross-fabric paths from MMSS fabric to APPSS fabric memory (EBI_CH0) are now
enabled via a fix to the gateway node links in `msm8660.c`.

**Fix Applied:**
The gateway nodes (`afab_to_mmss`, `afab_to_system`) were missing links to the
memory slave (`slv_ebi_ch0`), which prevented `path_find()` BFS from traversing
from MMSS fabric masters to main memory. Fixed by adding `MSM8660_AFAB_SLV_EBI_CH0`
to the gateway node link arrays.

**Device Tree Configuration:**
```dts
interconnects = <&mmss_fabric MMFAB_MAS_MDP_PORT0 &apps_fabric AFAB_SLV_EBI_CH0>,
                <&mmss_fabric MMFAB_MAS_MDP_PORT1 &apps_fabric AFAB_SLV_EBI_CH0>;
interconnect-names = "mdp0-mem", "mdp1-mem";
```

**Path Resolution:**
MDP_PORT0/1 → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**Bandwidth:** 6400 MBps peak

---

### 2. GPU 3D (Adreno 220 / Yamato)

**Status: NOT NEEDED**

**webOS Implementation:**
```c
/* From devices-msm8x60.c */
static struct msm_bus_vectors grp3d_max_vectors[] = {
    .src = MSM_BUS_MASTER_GRAPHICS_3D,
    .dst = MSM_BUS_SLAVE_EBI_CH0,
    .ab  = 0,
    .ib  = 2096000000U,  /* ~2 GB/s */
};
```

**Mainline Implementation:**
The Adreno 220 uses the `a2xx_gpu.c` driver which does NOT have any interconnect
API calls (no `devm_of_icc_get()` unlike a3xx/a4xx drivers).

The `dev_pm_opp_of_find_icc_paths()` call in `adreno_device.c` gracefully returns
success (0) when no `interconnects` property exists in device tree.

**Why interconnect is not implemented:**
1. The a2xx driver doesn't require it - bandwidth is managed via GPU clock scaling
2. Cross-fabric paths (MMSS_FABRIC -> APPSS_FABRIC) would require interconnect
   framework enhancements to resolve paths across multiple providers
3. The GPU works correctly without explicit interconnect bandwidth votes

**If interconnect were to be added (requires framework work):**
```dts
gpu@4300000 {
    interconnects = <&mmss_fabric MMFAB_MAS_GRAPHICS_3D &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "gfx-mem";
};
```

---

### 3. GPU 2D Cores (Z180)

**Status: ENABLED (Z180 driver added)**

The Z180 2D graphics driver (`drivers/gpu/drm/msm/z180/`) provides hardware-accelerated
2D vector graphics operations. Two independent cores are available.

**Implementation:**
- Driver: `drivers/gpu/drm/msm/z180/`
- Core 0 Base Address: 0x04100000
- Core 1 Base Address: 0x04200000
- Core 0 IRQ: SPI 81 (level high)
- Core 1 IRQ: SPI 212 (level high)
- Clocks: GFX2D0_CLK/GFX2D1_CLK, GFX2D0_AHB_CLK/GFX2D1_AHB_CLK

**Features:**
- Command stream submission with ringbuffer
- Timestamp-based synchronization
- Miscdevice interface (/dev/z180-0, /dev/z180-1)
- Runtime PM with interconnect bandwidth voting

**Device Tree Configuration:**
```dts
gpu_2d0: gpu@4100000 {
    compatible = "qcom,msm8660-z180";
    reg = <0x04100000 0x1000>;
    interrupts = <GIC_SPI 81 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&mmcc GFX2D0_CLK>,
             <&mmcc GFX2D0_AHB_CLK>;
    clock-names = "core", "iface";
    interconnects = <&mmss_fabric MMFAB_MAS_GRAPHICS_2D_CORE0
                     &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "gfx-mem";
    status = "disabled";
};

gpu_2d1: gpu@4200000 {
    compatible = "qcom,msm8660-z180";
    reg = <0x04200000 0x1000>;
    interrupts = <GIC_SPI 212 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&mmcc GFX2D1_CLK>,
             <&mmcc GFX2D1_AHB_CLK>;
    clock-names = "core", "iface";
    interconnects = <&mmss_fabric MMFAB_MAS_GRAPHICS_2D_CORE1
                     &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "gfx-mem";
    status = "disabled";
};
```

**Path Resolution:**
GRAPHICS_2D_CORE0/1 → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**Bandwidth:** 128 MB/s average, 256 MB/s peak

**webOS Reference:**
```c
/* From devices-msm8x60.c */
struct msm_bus_scale_pdata grp2d0_bus_scale_pdata = {
    .src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
    .dst = MSM_BUS_SLAVE_EBI_CH0,
};
struct msm_bus_scale_pdata grp2d1_bus_scale_pdata = {
    .src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
    .dst = MSM_BUS_SLAVE_EBI_CH0,
};
```

---

### 4. Camera Subsystem (VFE, VPE, JPEG)

**Status: ENABLED (CAMSS driver updated)**

The CAMSS driver (`drivers/media/platform/qcom/camss/`) now has interconnect support
for MSM8660/APQ8060. The `icc_res_8x60` array and `msm8660_resources` have been updated.

**Implementation:**
- Added `icc_res_8x60[]` with VFE memory bandwidth (~1.45 GB/s)
- Updated `msm8660_resources` with `.icc_res` and `.icc_path_num`
- Device tree includes `interconnects` and `interconnect-names` properties

**Device Tree Configuration:**
```dts
interconnects = <&mmss_fabric MMFAB_MAS_VFE &apps_fabric AFAB_SLV_EBI_CH0>;
interconnect-names = "vfe-mem";
```

**Path Resolution:**
VFE → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**Note:** VPE and JPEG Encoder are separate drivers (not part of CAMSS) and now
have their own mainline V4L2 mem2mem drivers with interconnect support (see below).

**webOS Reference:**
```c
/* From msm_io_8x60.c */
static struct msm_bus_vectors cam_bus_client_config[] = {
    /* VFE - Video Front End (camera sensor interface) */
    .src = MSM_BUS_MASTER_VFE,
    .dst = MSM_BUS_SLAVE_SMI,
    .dst = MSM_BUS_SLAVE_EBI_CH0,

    /* VPE - Video Processing Engine */
    .src = MSM_BUS_MASTER_VPE,
    .dst = MSM_BUS_SLAVE_SMI,

    /* JPEG Encoder */
    .src = MSM_BUS_MASTER_JPEG_ENC,
    .dst = MSM_BUS_SLAVE_SMI,
};
```

**Required for:**
- Camera preview and capture
- Video recording
- Image processing

**Legacy Recommended DT (obsolete):**
```dts
camss@4500000 {
    interconnects = <&mmss_fabric MMFAB_MAS_VFE &apps_fabric AFAB_SLV_EBI_CH0>,
                    <&mmss_fabric MMFAB_MAS_VFE &mmss_fabric MMFAB_SLV_SMI>;
    interconnect-names = "vfe-mem", "vfe-smi";
};
```

---

### 5. Video Processing Engine (VPE)

**Status: ENABLED (VPE driver added)**

The VPE driver (`drivers/media/platform/qcom/vpe/`) provides hardware-accelerated
video scaling and rotation. Implemented as a V4L2 mem2mem driver.

**Implementation:**
- Driver: `drivers/media/platform/qcom/vpe/`
- Base Address: 0x05300000
- IRQ: SPI 47 (edge rising)
- Clocks: VPE_CLK (160 MHz), VPE_AXI_CLK, VPE_AHB_CLK

**Features:**
- Hardware scaling (FIR/M-N interpolation)
- Rotation (0/90/180/270 degrees)
- Format: NV12 (Y/CbCr planar)

**Device Tree Configuration:**
```dts
vpe: video-processing@5300000 {
    compatible = "qcom,msm8660-vpe";
    reg = <0x05300000 0x100000>;
    interrupts = <GIC_SPI 47 IRQ_TYPE_EDGE_RISING>;
    clocks = <&mmcc VPE_CLK>,
             <&mmcc VPE_AXI_CLK>,
             <&mmcc VPE_AHB_CLK>;
    clock-names = "core", "axi", "ahb";
    interconnects = <&mmss_fabric MMFAB_MAS_VPE &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "vpe-mem";
    status = "disabled";
};
```

**Path Resolution:**
VPE → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**Bandwidth:** ~1.5 GB/s peak (1080p processing)

---

### 6. JPEG Encoder (Gemini)

**Status: ENABLED (Gemini driver added)**

The Gemini JPEG driver (`drivers/media/platform/qcom/gemini/`) provides
hardware-accelerated JPEG encoding. Implemented as a V4L2 mem2mem driver.

**Implementation:**
- Driver: `drivers/media/platform/qcom/gemini/`
- Base Address: 0x04600000
- IRQ: SPI 77 (edge rising)
- Clocks: IJPEG_CLK, IJPEG_AXI_CLK, IJPEG_AHB_CLK

**Features:**
- Hardware JPEG encoding
- Input: NV12 raw YUV
- Output: JPEG compressed data
- Ping-pong buffer management

**Device Tree Configuration:**
```dts
gemini: jpeg@4600000 {
    compatible = "qcom,msm8660-gemini";
    reg = <0x04600000 0x100000>;
    interrupts = <GIC_SPI 77 IRQ_TYPE_EDGE_RISING>;
    clocks = <&mmcc IJPEG_CLK>,
             <&mmcc IJPEG_AXI_CLK>,
             <&mmcc IJPEG_AHB_CLK>;
    clock-names = "core", "axi", "ahb";
    interconnects = <&mmss_fabric MMFAB_MAS_JPEG_ENC &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "jpeg-mem";
    status = "disabled";
};
```

**Path Resolution:**
JPEG_ENC → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**Bandwidth:** ~1 GB/s peak (high resolution encoding)

---

### 7. Video Codec (VIDC 1080p)

**Status: ENABLED (VIDC driver updated)**

The VIDC 1080p driver (`drivers/media/platform/qcom/vidc/`) now has interconnect
framework integration for MSM8660/APQ8060.

**Implementation:**
- Driver: `drivers/media/platform/qcom/vidc/`
- Base Address: 0x04400000
- IRQ: SPI 49 (level high)
- Clocks: VCODEC_CLK, VCODEC_AHB_CLK, VCODEC_AXI_CLK

**Features:**
- Hardware video decode (H.264, MPEG4, VC1, MPEG2, H.263, DivX)
- Hardware video encode (H.264, MPEG4, H.263)
- 1080p resolution support
- Interconnect bandwidth management

**Device Tree Configuration:**
```dts
vidc: video-codec@4400000 {
    compatible = "qcom,msm8660-vidc";
    reg = <0x04400000 0x100000>;
    interrupts = <GIC_SPI 49 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&mmcc VCODEC_CLK>,
             <&mmcc VCODEC_AHB_CLK>,
             <&mmcc VCODEC_AXI_CLK>;
    clock-names = "core", "iface", "axi";
    interconnects = <&mmss_fabric MMFAB_MAS_HD_CODEC_PORT0
                     &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "video-mem";
    status = "disabled";
};
```

**Path Resolution:**
HD_CODEC_PORT0 → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**webOS Implementation:**
```c
/* From vcd_res_tracker.h */
static struct msm_bus_vectors vidc_init_vectors[] = {
    .src = MSM_BUS_MASTER_HD_CODEC_PORT0,
    .dst = MSM_BUS_SLAVE_SMI,

    .src = MSM_BUS_MASTER_HD_CODEC_PORT1,
    .dst = MSM_BUS_SLAVE_SMI,

    /* CPU access for buffer management */
    .src = MSM_BUS_MASTER_AMPSS_M0,
    .dst = MSM_BUS_SLAVE_EBI_CH0,
    .dst = MSM_BUS_SLAVE_SMI,
};
```

**Bandwidth Levels:**
- VGA: ~54 MB/s average, ~436 MB/s peak
- 720p: ~108 MB/s average
- 1080p: ~245 MB/s average

**Required for:**
- Hardware video decode (H.264, etc.)
- Hardware video encode
- Critical for media playback

**Recommended DT (when driver is modified):**
```dts
vidc@4400000 {
    interconnects = <&mmss_fabric MMFAB_MAS_HD_CODEC_PORT0 &mmss_fabric MMFAB_SLV_SMI>,
                    <&mmss_fabric MMFAB_MAS_HD_CODEC_PORT1 &mmss_fabric MMFAB_SLV_SMI>;
    interconnect-names = "video0-smi", "video1-smi";
};
```

---

### 8. Image Rotator

**Status: ENABLED (Rotator driver added)**

The Rotator driver (`drivers/media/platform/qcom/rotator/`) provides
hardware-accelerated image rotation. Implemented as a V4L2 mem2mem driver,
separate from MDP4 display controller.

**Implementation:**
- Driver: `drivers/media/platform/qcom/rotator/`
- Base Address: 0x04E00000
- IRQ: SPI 73 (edge rising)
- Clocks: ROT_CLK, ROT_AHB_CLK, ROT_AXI_CLK

**Features:**
- Hardware rotation: 0°, 90°, 180°, 270°
- Horizontal and vertical flip
- RGB formats: RGB565, XRGB32, ARGB32
- YUV formats: NV12, NV21, NV16, NV61
- Max resolution: 8191 x 8191 pixels

**Device Tree Configuration:**
```dts
rotator: rotator@4e00000 {
    compatible = "qcom,msm8660-rotator";
    reg = <0x04e00000 0x100000>;
    interrupts = <GIC_SPI 73 IRQ_TYPE_EDGE_RISING>;
    clocks = <&mmcc ROT_CLK>,
             <&mmcc ROT_AHB_CLK>,
             <&mmcc ROT_AXI_CLK>;
    clock-names = "core", "iface", "axi";
    interconnects = <&mmss_fabric MMFAB_MAS_ROTATOR
                     &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "rotator-mem";
    status = "disabled";
};
```

**Path Resolution:**
ROTATOR → MMFAB_TO_APPSS → AFAB_TO_MMSS → SLV_EBI_CH0

**Bandwidth:** ~1 GB/s peak (large image rotation)

---

### 9. USB

**Status: Implicit via DFAB clocks**

**webOS Implementation:**
The USB subsystem didn't use explicit msm_bus_scale calls, but relied on DFAB (Data Fabric) clocks:
```c
/* From board-tenderloin.c */
.pclk_src_name = "dfab_usb_hs_clk",
.pclk_src_dfab = 1,
```

**Notes:**
- USB uses the System Fabric for memory access
- The dfab_usb_hs_clk votes on the data fabric clock
- In mainline, this is handled by the rpmcc fabric clocks
- USB driver should implicitly benefit from fabric clock coordination

---

## Priority for Implementation

### Completed
1. **MDP4 Display** - Cross-fabric paths working
2. **Camera VFE (CAMSS)** - Interconnect support added
3. **VPE (Video Processing)** - New V4L2 mem2mem driver with interconnect
4. **JPEG Encoder (Gemini)** - New V4L2 mem2mem driver with interconnect
5. **Video Codec (VIDC)** - Interconnect support added to existing driver
6. **Rotator** - New V4L2 mem2mem driver with interconnect
7. **GPU 2D (Z180)** - New driver with command stream and interconnect support

### High Priority (Affects Core Functionality)

(All high priority items completed!)

### Medium Priority (Improves Performance)

(All medium priority items completed!)

### Low Priority (Nice to Have)

8. **DTV** - HDMI output (uses MDP4 paths)

---

## Implementation Pattern

For each component, follow the MDP4 pattern:

### 1. Add to interconnect DT bindings header

Add master port IDs if not already present in `include/dt-bindings/interconnect/qcom,msm8660.h`.

### 2. Add device tree bindings

```dts
component@address {
    interconnects = <&mmss_fabric MMFAB_MAS_XXX &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "xxx-mem";
};
```

### 3. Add driver support

```c
#include <linux/interconnect.h>

static int xxx_setup_interconnect(struct platform_device *pdev)
{
    struct icc_path *path = msm_icc_get(&pdev->dev, "xxx-mem");

    if (IS_ERR(path))
        return PTR_ERR(path);

    if (!path) {
        dev_warn(&pdev->dev, "No interconnect support\n");
        return 0;
    }

    icc_set_bw(path, avg_bw, peak_bw);
    return 0;
}
```

---

## Testing Considerations

When testing interconnect integration:

1. **USB RNDIS should remain functional** when component is active
2. **No display artifacts** or underruns during high bandwidth operations
3. **dmesg should show** interconnect paths being acquired

---

## Related Files

- `drivers/interconnect/qcom/msm8660.c` - Interconnect driver
- `include/dt-bindings/interconnect/qcom,msm8660.h` - DT bindings
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - Device tree

---

## References

- webOS kernel: `arch/arm/mach-msm/msm_bus_board_8660.c`
- webOS kernel: `arch/arm/mach-msm/board-tenderloin.c`
- webOS kernel: `drivers/video/msm/vidc/1080p/resource_tracker/vcd_res_tracker.h`
- webOS kernel: `drivers/media/video/msm/msm_io_8x60.c`
- webOS kernel: `arch/arm/mach-msm/devices-msm8x60.c`
