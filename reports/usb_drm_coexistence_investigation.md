# USB/DRM Coexistence Investigation Report
**Date:** 2026-01-08
**Kernel Version:** 6.18.0
**Hardware:** HP TouchPad (APQ8060/MSM8660)

---

## EXECUTIVE SUMMARY

**Status: ROOT CAUSE IDENTIFIED - INTERCONNECT DRIVER CREATED**

The MSM DRM driver (msm.ko) interferes with USB RNDIS networking on the HP TouchPad due to missing bus fabric arbitration infrastructure in mainline Linux. We have:

1. **Identified the root cause**: Missing interconnect/bus fabric coordination between MDP and USB
2. **Analyzed webOS kernel**: Found proprietary `msm_bus_scale` API that handled this
3. **Tested MDP4 workarounds**: Clock manipulation workarounds did NOT prevent USB failure
4. **Implemented interconnect driver**: Created `drivers/interconnect/qcom/msm8660.c` as foundation for proper fix

---

## PROBLEM DESCRIPTION

### Symptoms
When loading the msm.ko DRM driver module, USB RNDIS networking immediately fails:
- Telnet connection drops
- Host loses connectivity to 172.16.42.2
- USB gadget stops responding
- Device requires reboot to recover

### Reproduction Steps
1. Boot HP TouchPad with USB RNDIS initramfs
2. Establish telnet connection to 172.16.42.2
3. Run: `insmod /lib/modules/6.18.0/msm.ko`
4. Connection immediately fails

---

## ROOT CAUSE ANALYSIS

### The Core Issue

MSM8660/APQ8060 uses a **fabric-based bus architecture** where multiple peripherals share bus bandwidth. The original webOS kernel coordinated this through a proprietary `msm_bus_scale` API. Mainline Linux lacks this infrastructure for MSM8660.

### Bus Fabric Architecture

```
              +------------------+
              |   APPSS Fabric   |  (CPU, L2, Memory)
              +--------+---------+
                       |
         +-------------+-------------+
         |                           |
  +------+------+            +-------+-------+
  | MMSS Fabric |            | System Fabric |
  | (MDP, GPU,  |            | (USB, DMA,    |
  |  Camera)    |            |  Peripherals) |
  +-------------+            +---------------+
```

### Why USB Fails When msm.ko Loads

1. **AXI Bus Contention**: MDP4's AXI clock (`bus_clk`) shares the AXI bus with USB
2. **No Fabric Arbitration**: Without interconnect framework, no coordination between MDP and USB
3. **Clock Enable/Disable Cycles**: MDP4 initialization toggles clocks that affect USB
4. **Missing DFAB Voter**: USB doesn't have a "voter" to keep the data fabric active

### webOS Solution (Not in Mainline)

webOS used separate clock domains managed by RPM:

| Peripheral | Clock | Fabric | Voter Clock |
|------------|-------|--------|-------------|
| USB HS | dfab_usb_hs_clk | DFAB (Data Fabric) | Votes on dfab_clk |
| MDP | mdp_axi_clk | MMFAB (MM Fabric) | Direct enable |
| SDCC | dfab_sdc_clk | DFAB | Votes on dfab_clk |

From webOS `devices-msm8x60.c`:
```c
CLK_VOTER("dfab_usb_hs_clk", DFAB_USB_HS_CLK, "dfab_clk", NULL, 0),
CLK_8X60("mdp_axi_clk", MDP_AXI_CLK, NULL, OFF),
```

---

## ATTEMPTED WORKAROUNDS

### MDP4 Clock Workarounds (FAILED)

We tried modifying `drivers/gpu/drm/msm/disp/mdp4/mdp4_kms.c`:

1. **Hardcoded MDP4 version** to skip `read_mdp_hw_revision()` clock cycle
2. **Reduced MDP core clock** from 266MHz to 128MHz
3. **Skipped DTV/LCDC/DSI disable cycle** during init
4. **Used optional VDD regulator**

**Result**: USB still failed immediately when msm.ko loaded. The issue is more fundamental than individual clock operations.

---

## SOLUTION IMPLEMENTED

### MSM8660 Interconnect Driver

Created a proper interconnect driver framework for MSM8660/APQ8060:

**Commit**: `8049c1235494` - interconnect: qcom: Add MSM8660/APQ8060 interconnect driver

#### Files Created

| File | Purpose |
|------|---------|
| `drivers/interconnect/qcom/msm8660.c` | Main interconnect driver (487 lines) |
| `include/dt-bindings/interconnect/qcom,msm8660.h` | DT bindings with node IDs |
| `drivers/interconnect/qcom/Kconfig` | Build configuration |
| `drivers/interconnect/qcom/Makefile` | Build rules |

#### Fabric Nodes Defined

**APPSS Fabric** (CPU and memory):
- `AFAB_MAS_AMPSS_M0/M1` - CPU masters
- `AFAB_SLV_EBI_CH0` - Memory slave
- `AFAB_SLV_AMPSS_L2` - L2 cache

**System Fabric** (Peripherals):
- `SFAB_MAS_SPS` - SPS master
- `SFAB_MAS_ADM0/1` - DMA masters
- `SFAB_SLV_*` - Various peripheral slaves

**MMSS Fabric** (Multimedia):
- `MMFAB_MAS_MDP_PORT0/1` - Display masters
- `MMFAB_MAS_GRAPHICS_3D` - GPU master
- `MMFAB_MAS_VFE` - Camera master
- `MMFAB_SLV_SMI` - SMI memory slave

#### Device Tree Integration

Added to `qcom-apq8060-tenderloin-common.dtsi`:
```dts
apps_fabric: interconnect@0 {
    compatible = "qcom,msm8660-apps-fabric";
    #interconnect-cells = <1>;
};

system_fabric: interconnect@1 {
    compatible = "qcom,msm8660-system-fabric";
    #interconnect-cells = <1>;
};

mmss_fabric: interconnect@2 {
    compatible = "qcom,msm8660-mmss-fabric";
    #interconnect-cells = <1>;
};
```

---

## REMAINING WORK

### For Full USB/DRM Coexistence

The interconnect driver provides infrastructure but doesn't fully solve the problem yet:

1. ~~**RPM Fabric Clock Support**~~: **DONE** - MSM8660 fabric clocks are now wired to the interconnect driver via device tree. The rpmcc provides `RPM_APPS_FABRIC_CLK`, `RPM_SYS_FABRIC_CLK`, and `RPM_MM_FABRIC_CLK` to the corresponding interconnect fabric nodes.

2. ~~**MDP4 Interconnect Integration**~~: **DONE** - Added to MDP4 device tree node:
   ```dts
   interconnects = <&mmss_fabric MMFAB_MAS_MDP_PORT0 &apps_fabric AFAB_SLV_EBI_CH0>,
                   <&mmss_fabric MMFAB_MAS_MDP_PORT1 &apps_fabric AFAB_SLV_EBI_CH0>;
   interconnect-names = "mdp0-mem", "mdp1-mem";
   ```

3. ~~**MDP4 Driver Changes**~~: **DONE** - Added `mdp4_setup_interconnect()` function that:
   - Gets interconnect paths via `msm_icc_get()`
   - Sets initial bandwidth to 6400 MBps peak
   - Called at probe time before other initialization

4. **USB Fabric Voting**: Ensure USB maintains its fabric vote during MDP operations (may be handled automatically by interconnect framework)

---

## CURRENT WORKAROUND

Until full interconnect support is implemented:

1. **Do NOT auto-load msm.ko** in initramfs
2. **Use USB RNDIS** for development and debugging
3. **Test display separately** by booting directly to LuneOS
4. **Load msm.ko manually** only when USB is not needed

### Initramfs Module Strategy

```bash
# Load DRM helper modules
for mod in drm_kms_helper drm_display_helper ...; do
    insmod "$MODDIR/${mod}.ko"
done

# DO NOT auto-load msm.ko
echo "*** msm.ko NOT auto-loaded ***"
echo "Load manually with: insmod /lib/modules/6.18.0/msm.ko"
```

---

## HARDWARE TEST RESULTS

### With msm.ko NOT Loaded
- USB RNDIS: Working
- Telnet: Working (172.16.42.2:23)
- Display: Not initialized (expected)

### With msm.ko Loaded
- USB RNDIS: **FAILS IMMEDIATELY**
- Connection lost, device needs reboot

---

## TECHNICAL DETAILS

### Kernel Configuration
```
CONFIG_INTERCONNECT=y
CONFIG_INTERCONNECT_QCOM=y
CONFIG_INTERCONNECT_QCOM_MSM8660=y
CONFIG_DRM=y
CONFIG_DRM_MSM=m
CONFIG_DRM_MSM_MDP4=y
```

### Clock Dependencies

**MDP4 Clocks** (from MMCC):
- `core_clk`: MDP4 core clock
- `iface_clk`: MDP4 interface clock
- `bus_clk`: MDP AXI clock (`MDP_AXI_CLK`)
- `lut_clk`: LUT clock

**USB Clocks** (from GCC):
- `USB_HS1_XCVR_CLK`: USB transceiver clock
- `USB_HS1_H_CLK`: USB AHB clock

---

## RELATED COMMITS

- `189f9c7dd7f0` - ARM: dts: qcom: tenderloin: Add MDP4 interconnect paths
- `ebe8a636673a` - drm/msm/mdp4: Add interconnect support for bus bandwidth coordination
- `2e484450a05c` - ARM: dts: qcom: tenderloin: Wire RPM fabric clocks to interconnect
- `8049c1235494` - interconnect: qcom: Add MSM8660/APQ8060 interconnect driver
- `d4920537a235` - ARM: configs: tenderloin: Disable module signing, add DRM helpers
- `a92c2d3d0747` - ARM: configs: tenderloin: Enable IOMMU and disable RNDIS
- `0dc9159c9da9` - pwm: pm8058: Fix deadlock in pm8058_pwm_bank_enable
- `a35f89c26526` - ARM: dts: qcom: tenderloin: Remove HDMI disable to fix USB

---

## CONCLUSION

The USB/DRM coexistence issue on APQ8060 is caused by **missing bus fabric arbitration infrastructure** in mainline Linux. The interconnect driver (`msm8660.c`) has been created as the foundation for a proper fix.

### Progress Made
1. Root cause identified (fabric arbitration)
2. webOS implementation analyzed
3. Interconnect driver framework created
4. Device tree nodes added
5. RPM fabric clocks wired to interconnect driver
6. MDP4 interconnect paths added to device tree
7. MDP4 driver interconnect support implemented

### Next Steps
1. ~~Implement RPM fabric clock support~~ **DONE**
2. ~~Add interconnect bindings to MDP4 driver~~ **DONE**
3. ~~Implement bandwidth requests in MDP4 initialization~~ **DONE**
4. Test with full interconnect integration (load msm.ko with USB active)

### Current Recommendation
Continue using USB RNDIS without msm.ko for kernel development. Test display functionality by booting directly to LuneOS userspace.

---

## UPDATE: GPU (KMSCUBE) USB FAILURE INVESTIGATION

**Date:** 2026-01-25

### New Symptom

After the MDP4 interconnect fix, `msm.ko` can load without killing USB. However, running **kmscube** (3D GPU test via Mesa/freedreno) kills USB networking. Display-only tests (modetest, vbltest, proptest) work fine.

### Root Cause: GPU devfreq Not Scaling

The Adreno 220 GPU's devfreq (dynamic frequency scaling) doesn't scale up from minimum frequency:

| Metric | Observed | Expected |
|--------|----------|----------|
| GPU Frequency | 27 MHz (stuck) | 27-320 MHz (scaling) |
| Interconnect Bandwidth | 170 kBps | 170-2008 MBps |
| USB Bandwidth | 131 MBps | 131 MBps |

When the GPU is active at 27 MHz with only 170 kBps bandwidth, it monopolizes the EBI bus, starving USB.

### Why devfreq Doesn't Scale

The `simple_ondemand` governor sees 0% GPU utilization because `df->suspended` is true:

```c
// drivers/gpu/drm/msm/msm_gpu_devfreq.c
static int msm_devfreq_get_dev_status(...) {
    if (df->suspended) {
        status->busy_time = 0;  // Always reports 0% load
        return 0;
    }
    // ... actual busy cycle counting
}
```

The `df->suspended` flag lifecycle:
1. **Init**: Set to `true` in `msm_devfreq_init()`
2. **Resume**: Set to `false` in `msm_devfreq_resume()` (called from `adreno_runtime_resume()`)
3. **Suspend**: Set to `true` in `msm_devfreq_suspend()` (called from `adreno_runtime_suspend()`)

### Investigation Findings

1. **Performance governor works correctly**:
   ```bash
   echo performance > /sys/class/devfreq/4300000.adreno/governor
   # GPU jumps to 320 MHz, interconnect to 2008 MBps
   ```

2. **Interconnect is properly configured**:
   - GPU path: `MMFAB_MAS_GRAPHICS_3D → AFAB_SLV_EBI_CH0`
   - USB path: `DFAB_MAS_USB_HS → DFAB_TO_SFAB → SFAB_SLV_EBI_CH0`
   - OPP table correctly maps frequency to bandwidth

3. **Clock switching works**:
   ```
   gfx3d_src: found freq=320000000 src=2 pre_div=0 m=2 n=5
   gfx3d_src: parent hw=pll2
   ```

4. **Initial boot behavior**:
   - GPU reaches 320 MHz for ~500ms at boot
   - Drops to 27 MHz and stays there
   - `trans_stat` shows 0 transitions after initial drop

### Current Workaround Status

**WORKAROUND INSUFFICIENT**: Performance governor alone does NOT prevent USB failure.

Testing showed:
1. Performance governor correctly sets GPU to 320 MHz
2. Initially, interconnect bandwidth is 2008 MBps (correct)
3. When GPU goes to pm_runtime suspend, bandwidth drops to 170 kBps
4. When GPU wakes for rendering, bandwidth stays at 170 kBps
5. **USB still fails during GPU rendering**

```bash
# This helps but doesn't fully fix the issue
echo performance > /sys/class/devfreq/4300000.adreno/governor
```

### Root Cause (Revised)

The issue is **interconnect bandwidth not being re-established** when GPU wakes from pm_runtime suspend:

1. GPU pm_runtime suspend → bandwidth released (170 kBps)
2. GPU pm_runtime resume → bandwidth NOT re-requested
3. Even at 320 MHz, with only 170 kBps bandwidth, USB fails

The OPP table correctly maps 320 MHz to 2008 MBps, but the GPU driver's pm_runtime resume path doesn't re-apply the bandwidth vote.

### Potential Fixes

1. **Kernel parameter or DT override**: Default to performance governor for GPU devfreq
2. **Fix devfreq suspend/resume**: Investigate why `df->suspended` stays true after pm_runtime resume
3. **Add minimum bandwidth for USB**: Ensure USB path has guaranteed bandwidth regardless of GPU state
4. **webOS-style EBI voter**: Add clock voter coordination like webOS's `CLK_VOTER` system

### Files Involved

| File | Relevance |
|------|-----------|
| `drivers/gpu/drm/msm/msm_gpu_devfreq.c` | devfreq init/suspend/resume logic |
| `drivers/gpu/drm/msm/msm_gpu.c:876-914` | `msm_gpu_submit()` - pm_runtime and devfreq calls |
| `drivers/gpu/drm/msm/adreno/adreno_device.c:304-323` | `adreno_runtime_resume/suspend()` callbacks |
| `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` | GPU OPP table and interconnect paths |

### Interconnect Bandwidth at Different Frequencies

From device tree OPP table:

| GPU Frequency | Interconnect Bandwidth |
|---------------|----------------------|
| 27 MHz | 170 kBps |
| 48 MHz | 230 kBps |
| 64 MHz | 375 kBps |
| 76.8 MHz | 450 kBps |
| 96 MHz | 563 kBps |
| 128 MHz | 750 kBps |
| 160 MHz | 938 kBps |
| 200 MHz | 1172 kBps |
| 228.571 MHz | 1340 kBps |
| 266.667 MHz | 1563 kBps |
| 320 MHz | 2008 kBps |

### Tested Commands

```bash
# Check devfreq status
cat /sys/class/devfreq/4300000.adreno/cur_freq
cat /sys/class/devfreq/4300000.adreno/trans_stat
cat /sys/class/devfreq/4300000.adreno/governor

# Check pm_runtime status
cat /sys/devices/platform/soc/4300000.adreno/power/runtime_status
cat /sys/devices/platform/soc/4300000.adreno/power/runtime_active_time

# Check interconnect bandwidth
cat /sys/kernel/debug/interconnect/interconnect_summary

# Apply workaround
echo performance > /sys/class/devfreq/4300000.adreno/governor
```

---

## UPDATE: Display Output Format Cross-Check (2026-01-25)

### Problem: Blue Vertical Lines on Display

When running kmscube or modetest, display shows blue vertical lines instead of expected content.

### webOS Legacy Kernel Cross-Check

Analyzed webOS kernel source to compare display configuration:

#### webOS Panel Configuration (lcdc_lg_xga.c)
```c
pinfo->bpp = 18;           // Panel capability: 18-bit (RGB666)
pinfo->clk_rate = 96000000; // 96 MHz pixel clock
// Note: output_format defaults to 0 = MSM_MDP_OUT_IF_FMT_RGB565
```

#### webOS DMA Configuration (mdp_lcdc.c lines 153-165)
```c
dma_cfg |= (DMA_PACK_ALIGN_LSB |    // LSB alignment!
            DMA_PACK_PATTERN_RGB |   // Pack pattern 0x21
            DMA_DITHER_EN);          // Enable dithering

// For default output_format=0 (RGB565):
dma_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
// Result: G=6 bits, B=5 bits, R=5 bits (16-bit total)
```

### Configuration Comparison

| Setting | webOS | Mainline | Issue |
|---------|-------|----------|-------|
| Output BPP | 16 (RGB565) | 24 (RGB888) | **Different** |
| Pack Align | LSB | MSB (default) | **CRITICAL** |
| Pack Pattern | 0x21 | 0x21 | Same |
| Dithering | Enabled | Disabled for 8bpc | Different |
| Pixel Clock | 96 MHz | 96 MHz | Same |
| Panel Timings | 400/272/328/6/10/7 | Same | Same |

### Key Discrepancy: Pack Alignment

webOS used **DMA_PACK_ALIGN_LSB** but mainline uses **PACK_ALIGN_MSB** by default.

From webOS mdp.h:
```c
#define DMA_PACK_ALIGN_LSB                  0
/*
 * use DMA_PACK_ALIGN_MSB if the upper 6 bits from 8 bits output
 * from LCDC block maps into 6 pins out to the panel
 */
#define DMA_PACK_ALIGN_MSB                  BIT(7)
```

The mainline mdp4_lcdc_encoder.c line 331-332:
```c
if (!of_property_read_bool(dev->dev->of_node, "qcom,lcdc-align-lsb"))
    config |= MDP4_DMA_CONFIG_PACK_ALIGN_MSB;
```

### Physical Interface

The TouchPad has 24 physical LCDC data lines (GPIO 4-27):
- Red: lcdc_red7-red0 (8 bits)
- Green: lcdc_grn7-grn0 (8 bits)
- Blue: lcdc_blu7-blu0 (8 bits)

Hardware supports 24-bit, but webOS chose 16-bit output for bandwidth efficiency.

### Recommended Fixes

1. **Immediate: Add LSB alignment property**
   ```dts
   &mdp {
       qcom,lcdc-align-lsb;
   };
   ```

2. **Alternative: Use 18-bit mode like webOS**
   ```dts
   lg_panel: panel {
       data-mapping = "jeida-18";  // Instead of "vesa-24"
   };
   ```

### Files Modified

- `drivers/gpu/drm/panel/panel-lvds.c`: Added bpc field derivation from bus_format
- Pending: DT change for LSB alignment or 18-bit mode
