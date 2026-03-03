# MSM8660/APQ8060 Register Comparison: WebOS Legacy vs Mainline

This document provides a comprehensive comparison of register definitions between the HP TouchPad's
legacy webOS kernel and the mainline Linux 6.18 kernel. It identifies discrepancies that have caused
or may cause driver issues.

## Executive Summary

Multiple register offset mismatches have been identified across camera, audio, and other subsystems.
These mismatches occurred because:

1. **VFE 4.x copy-paste**: The mainline VFE 3.1 driver was adapted from VFE 4.x drivers, which have
   different register layouts
2. **Documentation gaps**: MSM8660 hardware documentation is limited, making verification difficult
3. **Different architectural approaches**: WebOS uses monolithic board files; mainline uses device tree

## Critical Issues Found and Fixed

| Component | Register | WebOS Offset | Mainline (Before) | Status |
|-----------|----------|--------------|-------------------|--------|
| VFE31 | CAMIF_CMD | 0x1E0 | 0x1EC | **FIXED** |
| VFE31 | CAMIF_STATUS | 0x204 | 0x1E0 | **FIXED** |
| VFE31 | CAMIF_CMD_START | 0x5 | 0x1 | **FIXED** |
| VFE31 | WM registers | 0x04C + 0x18*n | 0x06C + 0x24*n | **FIXED** (earlier) |

---

## 1. VFE31 (Video Front End) Register Comparison

### 1.1 Core Control Registers

| Register | WebOS (msm_vfe31.h) | Mainline (camss-vfe-3-1.c) | Match? |
|----------|---------------------|---------------------------|--------|
| VFE_GLOBAL_RESET | 0x004 | 0x004 | YES |
| VFE_CGC_OVERRIDE | 0x00C | 0x00C | YES |
| VFE_MODULE_CFG | 0x010 | 0x010 | YES |
| VFE_CFG_OFF | 0x014 | 0x014 (VFE_0_CORE_CFG) | YES |
| VFE_IRQ_CMD | 0x018 | 0x018 | YES |
| VFE_IRQ_MASK_0 | 0x01C | 0x01C | YES |
| VFE_IRQ_MASK_1 | 0x020 | 0x020 | YES |
| VFE_IRQ_CLEAR_0 | 0x024 | 0x024 | YES |
| VFE_IRQ_CLEAR_1 | 0x028 | 0x028 | YES |
| VFE_IRQ_STATUS_0 | 0x02C | 0x02C | YES |
| VFE_IRQ_STATUS_1 | 0x030 | 0x030 | YES |
| VFE_IRQ_COMP_MASK | 0x034 | 0x034 | YES |

### 1.2 Bus Configuration Registers

| Register | WebOS | Mainline | Match? |
|----------|-------|----------|--------|
| VFE_BUS_CMD | 0x038 | 0x038 | YES |
| VFE_BUS_CFG | N/A | 0x03C | N/A |
| AXI_OUT_MODE | 0x040 (V31_AXI_OUT_OFF=0x038) | 0x040 | CHECK |
| VFE_BUS_PING_PONG_STATUS | 0x180 | 0x180 | YES |

### 1.3 Write Master (WM) Registers - **CRITICAL DIFFERENCE**

VFE31 uses stride 0x18, VFE41 uses stride 0x24!

| Register | WebOS VFE31 | Mainline VFE41 (Wrong) | Mainline VFE31 (Fixed) |
|----------|-------------|------------------------|------------------------|
| WM0_WR_CFG | 0x04C | 0x06C | 0x04C |
| WM0_PING_ADDR | 0x050 | 0x070 | 0x050 |
| WM0_PONG_ADDR | 0x054 | 0x074 | 0x054 |
| WM0_ADDR_CFG | 0x058 | 0x078 | 0x058 |
| WM0_UB_CFG | 0x05C | 0x07C | 0x05C |
| WM0_IMAGE_SIZE | 0x060 | 0x080 | 0x060 |
| **Stride** | **0x18** | **0x24** | **0x18** |

### 1.4 CAMIF Registers - **CRITICAL DIFFERENCE (NOW FIXED)**

| Register | WebOS | Mainline (Before Fix) | Mainline (After Fix) | Status |
|----------|-------|----------------------|---------------------|--------|
| VFE_CAMIF_COMMAND | 0x1E0 | 0x1EC | 0x1E0 | **FIXED** |
| V31_CAMIF_OFF (config) | 0x1E4 | 0x1E4 | 0x1E4 | OK |
| VFE_CAMIF_FRAME_CFG | 0x1E8 | 0x1E8 | 0x1E8 | OK |
| VFE_CAMIF_WINDOW_W | ~0x1F0 | 0x1F0 | 0x1F0 | OK |
| VFE_CAMIF_WINDOW_H | ~0x1F4 | 0x1F4 | 0x1F4 | OK |
| VFE_CAMIF_SUBSAMPLE | ~0x1F8 | 0x1F8 | 0x1F8 | OK |
| VFE_CAMIF_IRQ_PAT | ~0x1FC | 0x1FC | 0x1FC | OK |
| VFE_CAMIF_STATUS | **0x204** | **0x1E0** | **0x204** | **FIXED** |

### 1.5 CAMIF Command Values - **CRITICAL DIFFERENCE (NOW FIXED)**

| Command | WebOS | Mainline (Before) | Mainline (After) | Status |
|---------|-------|-------------------|------------------|--------|
| START | 0x5 | 0x1 | 0x5 | **FIXED** |
| STOP_AT_BOUNDARY | 0x0 | 0x0 | 0x0 | OK |
| STOP_IMMEDIATELY | 0x2 | N/A | 0x2 | Added |
| CLEAR_STATUS | 0x4 | 0x4 | 0x4 | OK |

### 1.6 AXI Control Registers

| Register | WebOS | Mainline | Match? |
|----------|-------|----------|--------|
| VFE_AXI_CMD | 0x1D8 | 0x1D8 | YES |
| VFE_AXI_STATUS | 0x1DC | 0x1DC | YES |

### 1.7 Statistics Registers

| Register | WebOS | Mainline | Match? |
|----------|-------|----------|--------|
| VFE_STATS_AE_CFG | 0x534 | 0x534 | YES |
| VFE_STATS_AF_CFG | 0x53C | 0x53C | YES |
| VFE_STATS_AWB_CFG | 0x54C | 0x54C | YES |
| VFE_STATS_RS_CFG | 0x56C | 0x56C | YES |
| VFE_STATS_CS_CFG | 0x574 | 0x574 | YES |
| VFE_STATS_IHIST_CFG | 0x57C | 0x57C | YES |

### 1.8 Framedrop Registers

| Register | WebOS | Mainline | Match? |
|----------|-------|----------|--------|
| FRAMEDROP_ENC_Y_CFG | 0x504 | 0x504 | YES |
| FRAMEDROP_ENC_CBCR_CFG | 0x508 | 0x508 | YES |
| FRAMEDROP_ENC_Y_PAT | 0x50C | 0x50C | YES |
| FRAMEDROP_ENC_CBCR_PAT | 0x510 | 0x510 | YES |
| FRAMEDROP_VIEW_Y_CFG | 0x514 | 0x514 | YES |
| FRAMEDROP_VIEW_CBCR_CFG | 0x518 | 0x518 | YES |

### 1.9 Processing Pipeline Registers

| Register | WebOS | Mainline | Match? |
|----------|-------|----------|--------|
| VFE_DEMUX_CFG | 0x284 | 0x284 | YES |
| VFE_DEMUX_GAIN_0 | 0x288 | 0x288 | YES |
| VFE_DEMUX_GAIN_1 | 0x28C | 0x28C | YES |
| VFE_CHROMA_SUBS | 0x4F8 | 0x4F8 | YES |
| VFE_CLAMP_MAX | 0x524 | 0x524 | YES |
| VFE_CLAMP_MIN | 0x528 | 0x528 | YES |
| VFE_REG_UPDATE_CMD | 0x260 | 0x260 | YES |

---

## 2. CSIPHY (CSI Physical Layer) Register Comparison

| Register | WebOS (msm_io_8x60.c) | Mainline (camss-csiphy-8x60.c) | Match? |
|----------|----------------------|-------------------------------|--------|
| MIPI_PHY_CONTROL | 0x00 | 0x00 | YES |
| MIPI_PROTOCOL_CONTROL | 0x04 | 0x04 | YES |
| MIPI_INTERRUPT_STATUS | 0x08 | 0x08 | YES |
| MIPI_INTERRUPT_MASK | 0x0C | 0x0C | YES |
| MIPI_CALIBRATION_CONTROL | 0x18 | 0x18 | YES |
| MIPI_PHY_D1_CONTROL | 0x20 | 0x20 | YES |
| MIPI_CAMERA_CNTL | 0x24 | 0x24 | YES |
| MIPI_PHY_D2_CONTROL | 0x2C | 0x2C | YES |
| MIPI_PHY_D3_CONTROL | 0x30 | 0x30 | YES |
| MIPI_PHY_D0_CONTROL | 0x34 | 0x34 | YES |
| MIPI_PHY_D0_CONTROL2 | 0x38 | 0x38 | YES |
| MIPI_PHY_D1_CONTROL2 | 0x3C | 0x3C | YES |
| MIPI_PHY_D2_CONTROL2 | 0x40 | 0x40 | YES |
| MIPI_PHY_D3_CONTROL2 | 0x44 | 0x44 | YES |
| MIPI_PHY_CL_CONTROL | 0x48 | 0x48 | YES |

### CSIPHY Bit Masks

| Mask | WebOS | Mainline | Match? |
|------|-------|----------|--------|
| PROTOCOL_CONTROL_SW_RST | 0x8000000 | Not used (causes hang) | N/A |
| PROTOCOL_CONTROL_LONG_PKT | 0x200000 | 0x200000 | YES |
| PROTOCOL_CONTROL_DATA_FORMAT | 0x180000 | (0x3 << 19) | YES |
| PROTOCOL_CONTROL_DECODE_ID | 0x40000 | 0x40000 | YES |
| PROTOCOL_CONTROL_ECC_EN | 0x20000 | 0x20000 | YES |
| DATA_FORMAT_SHFT | 0x13 (19) | 19 | YES |

---

## 3. Audio Subsystem Register Comparison

### 3.1 LPASS (Low Power Audio SubSystem)

The mainline kernel uses the modern LPASS framework which differs significantly from
the webOS legacy MSM audio architecture.

**WebOS Files:**
- `sound/soc/msm/msm8660.c` - Machine driver
- `sound/soc/msm/msm8x60.c` - CPU DAI driver
- `sound/soc/msm/msm8660-pcm.c` - PCM platform driver

**Mainline Files:**
- `sound/soc/qcom/apq8060.c` - Machine driver (NEW)
- `sound/soc/qcom/lpass-cpu.c` - LPASS CPU DAI
- `sound/soc/qcom/lpass-platform.c` - LPASS platform

**Known Issues:**
- Audio routing not fully functional
- QDSP6 firmware loading issues
- Codec initialization timing differences

### 3.2 WCD9xxx Codec

Both use similar WCD9xxx codec drivers, but initialization sequences may differ.

---

## 4. Display Subsystem (MDP4)

### 4.1 MDP4 Registers

The mainline DRM/MSM driver generally has correct MDP4 register definitions.
Display is working on the TouchPad.

**Files:**
- `drivers/gpu/drm/msm/disp/mdp4/` - MDP4 display controller

---

## 5. Clock Controller Comparison

### 5.1 MMCC (Multimedia Clock Controller)

| Clock | WebOS Name | Mainline Name | Status |
|-------|-----------|---------------|--------|
| VFE clock | vfe_clk | VFE_CLK | OK |
| VFE AXI | vfe_axi_clk | VFE_AXI_CLK | OK |
| CSI0 | csi0_clk | CSI0_CLK | OK |
| CSI1 | csi1_clk | CSI1_CLK | OK |
| CSI0 PHY | csi0_phy_clk | CSI0_PHY_CLK | OK |
| CSI1 PHY | csi1_phy_clk | CSI1_PHY_CLK | OK |
| VFE-CSI0 bridge | csi_vfe_clk (CSI0) | VFE_CSI0_CLK | OK |
| VFE-CSI1 bridge | csi_vfe_clk (CSI1) | VFE_CSI1_CLK | OK |

---

## 6. GPIO/Pinctrl

The mainline kernel uses the pinctrl framework instead of the legacy gpiomux.
Pin configurations are in device tree.

**Potential Issues:**
- Pull-up/pull-down configurations may differ
- Drive strength settings may differ

---

## 7. Recommendations

### 7.1 Immediate Actions

1. **Test camera with fixed CAMIF registers** - The CAMIF_CMD and CAMIF_STATUS offset
   fixes should resolve the SOF timeout issue.

2. **Verify AXI output mode** - Ensure 0x040 register is written correctly for raw capture.

### 7.2 Future Verification Needed

1. **Audio registers** - Comprehensive comparison of LPASS register configuration
2. **Bluetooth power** - Compare power sequencing with webOS bluetooth-power.c
3. **WiFi SDIO** - Compare SDIO host configuration

### 7.3 Testing Methodology

For any suspected register mismatch:

1. Add debug printk to dump register reads/writes
2. Compare with webOS kernel debug output (if available)
3. Cross-reference with any available MSM8660 documentation
4. Test functionality after each change

---

## 8. Version History

| Date | Changes |
|------|---------|
| 2026-03-03 | Initial document, fixed CAMIF register offsets |
| (earlier) | Fixed WM register offsets and stride |
| (earlier) | Added VFE/CSI clock enables for CSIPHY |
| (earlier) | Added CSIPHY DATA_FORMAT configuration |

---

## Appendix A: Full WebOS VFE31 Register Map

```
VFE_GLOBAL_RESET                0x004
VFE_CGC_OVERRIDE                0x00C
VFE_MODULE_CFG                  0x010
VFE_CFG_OFF                     0x014
VFE_IRQ_CMD                     0x018
VFE_IRQ_MASK_0                  0x01C
VFE_IRQ_MASK_1                  0x020
VFE_IRQ_CLEAR_0                 0x024
VFE_IRQ_CLEAR_1                 0x028
VFE_IRQ_STATUS_0                0x02C
VFE_IRQ_STATUS_1                0x030
VFE_IRQ_COMP_MASK               0x034
VFE_BUS_CMD                     0x038
V31_AXI_OUT_OFF                 0x038
VFE_BUS_PING_PONG_STATUS        0x180
VFE_AXI_CMD                     0x1D8
VFE_AXI_STATUS                  0x1DC
VFE_CAMIF_COMMAND               0x1E0
V31_CAMIF_OFF (config start)    0x1E4
VFE_CAMIF_STATUS                0x204
VFE_REG_UPDATE_CMD              0x260
VFE_DEMUX_GAIN_0                0x288
VFE_DEMUX_GAIN_1                0x28C
VFE_CHROMA_UP                   0x35C
VFE_FRAMEDROP_ENC_Y_CFG         0x504
VFE_FRAMEDROP_ENC_CBCR_CFG      0x508
VFE_CLAMP_MAX                   0x524
VFE_CLAMP_MIN                   0x528
VFE_REALIGN_BUF                 0x52C
VFE_STATS_CFG                   0x530
VFE_STATS_AE_CFG                0x534
VFE_STATS_AF_CFG                0x53C
VFE_STATS_AWB_CFG               0x54C
VFE_STATS_RS_CFG                0x56C
VFE_STATS_CS_CFG                0x574
VFE_STATS_IHIST_CFG             0x57C
```

## Appendix B: References

- WebOS kernel: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.h`
- WebOS CSIPHY: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_io_8x60.c`
- Mainline VFE31: `linux-6.18-tenderloin/drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- Mainline CSIPHY: `linux-6.18-tenderloin/drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
