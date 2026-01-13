# MSM8660/APQ8060 Interconnect Requirements

**Date:** 2026-01-08
**Based on:** webOS kernel analysis (webos-linux-kernel-opal)

---

## Overview

This document catalogs all hardware components that used the `msm_bus_scale` API in the original webOS kernel. These components may benefit from or require interconnect framework integration in mainline Linux for proper bus bandwidth coordination.

---

## Interconnect Consumers Summary

| Component | Master Port(s) | Destination | Status in Mainline |
|-----------|---------------|-------------|-------------------|
| MDP4 Display | MDP_PORT0, MDP_PORT1 | SMI, EBI_CH0 | **OPTIONAL** (works via clock parent) |
| GPU 3D (Adreno 220) | GRAPHICS_3D | EBI_CH0 | **NOT NEEDED** (a2xx driver doesn't use ICC) |
| GPU 2D Core0 | GRAPHICS_2D_CORE0 | SMI | No driver (Z180 not in mainline) |
| GPU 2D Core1 | GRAPHICS_2D_CORE1 | SMI | No driver (Z180 not in mainline) |
| Camera (VFE) | VFE | SMI, EBI_CH0 | Driver needs modification |
| Video Processor (VPE) | VPE | SMI | No driver in mainline |
| JPEG Encoder | JPEG_ENC | SMI | No driver in mainline |
| Video Codec Port0 | HD_CODEC_PORT0 | SMI | Driver needs modification |
| Video Codec Port1 | HD_CODEC_PORT1 | SMI | Driver needs modification |
| Rotator | ROTATOR | SMI | No driver (MDP4 rotator not in mainline) |
| DTV (HDMI output) | MDP_PORT0 | SMI, EBI_CH0 | Uses MDP4 paths |
| CPU (for video) | AMPSS_M0 | EBI_CH0, SMI | Not required |

---

## Detailed Component Analysis

### 1. MDP4 Display Controller

**Status: OPTIONAL (works without interconnect)**

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
This function gracefully handles missing interconnect paths - it warns but doesn't
fail, allowing display to work without explicit bandwidth votes.

**USB/Display Coexistence:**
The USB vs display conflict is solved via clock parent relationships in
`mmcc-msm8960.c`, which ensures fabric clocks stay coordinated when both
subsystems are active. This workaround is functional without interconnect.

**If interconnect were to be added (requires framework work):**
- Device tree paths: `mdp0-mem`, `mdp1-mem`
- Bandwidth: 6400 MBps peak
- Requires cross-fabric path resolution (MMSS -> APPSS)

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

**Status: Not implemented**

**webOS Implementation:**
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

**Notes:**
- Two 2D cores (Z180) for 2D acceleration
- Lower bandwidth requirements than 3D
- May not be critical for basic functionality

---

### 4. Camera Subsystem (VFE, VPE, JPEG)

**Status: Driver needs modification**

The CAMSS driver (`drivers/media/platform/qcom/camss/`) has interconnect support for
newer SoCs (MSM8953+), but the MSM8660 resources don't include interconnect paths.
To add support, the driver would need:
1. Define `icc_res_8x60` array with path names
2. Add `.icc_path_num` to `msm8660_resources` structure

**webOS Implementation:**
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

**Recommended DT (when driver is modified):**
```dts
camss@4500000 {
    interconnects = <&mmss_fabric MMFAB_MAS_VFE &apps_fabric AFAB_SLV_EBI_CH0>,
                    <&mmss_fabric MMFAB_MAS_VFE &mmss_fabric MMFAB_SLV_SMI>;
    interconnect-names = "vfe-mem", "vfe-smi";
};
```

---

### 5. Video Codec (VIDC 1080p)

**Status: Driver needs modification**

The VIDC 1080p driver exists (`drivers/media/platform/qcom/vidc/`) with MSM8660 support,
but it doesn't have interconnect framework integration. The driver would need:
1. Add `#include <linux/interconnect.h>`
2. Get interconnect paths at probe time
3. Set bandwidth before encode/decode operations

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

### 6. Rotator

**Status: No driver in mainline**

The MDP4 driver in mainline (`drivers/gpu/drm/msm/disp/mdp4/`) does not include
rotator support. The MDP5 driver has rotator with interconnect, but MDP4 would
need significant work to add hardware rotation.

**webOS Implementation:**
```c
/* From msm_bus_board_8660.c */
.src = MSM_BUS_MASTER_ROTATOR,
.dst = MSM_BUS_SLAVE_SMI,
```

**Notes:**
- Used for hardware rotation of display buffers
- Lower priority than display/video
- Software rotation via GPU is the current alternative

---

### 7. USB

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

### High Priority (Affects Core Functionality)

1. **Video Codec** - Required for any video playback
2. **Camera VFE** - Required for camera functionality

### Medium Priority (Improves Performance)

3. **GPU 3D** - Better graphics performance
4. **Rotator** - Hardware rotation support

### Low Priority (Nice to Have)

5. **GPU 2D** - 2D acceleration
6. **DTV** - HDMI output (if display works via LCDC)

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
