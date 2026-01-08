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

1. **RPM Fabric Clock Support**: MSM8660 fabric clocks (`afab_clk`, `sfab_clk`, `mmfab_clk`) require RPM driver integration not yet in mainline

2. **MDP4 Interconnect Integration**: Add to MDP4 device tree node:
   ```dts
   interconnects = <&mmss_fabric MMFAB_MAS_MDP_PORT0 &apps_fabric AFAB_SLV_EBI_CH0>;
   interconnect-names = "mdp";
   ```

3. **MDP4 Driver Changes**: Request bandwidth before enabling clocks:
   ```c
   icc_set_bw(mdp4_kms->icc_path, avg_bw, peak_bw);
   ```

4. **USB Fabric Voting**: Ensure USB maintains its fabric vote during MDP operations

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

### Next Steps
1. Implement RPM fabric clock support
2. Add interconnect bindings to MDP4 driver
3. Implement bandwidth requests in MDP4 initialization
4. Test with full interconnect integration

### Current Recommendation
Continue using USB RNDIS without msm.ko for kernel development. Test display functionality by booting directly to LuneOS userspace.
