# VFE31 RDI Raw Bypass Investigation - Help Needed

## Hardware
- **SoC**: Qualcomm APQ8060 (same silicon as MSM8660)
- **VFE**: Version 3.1 (HW version 0x00030217)
- **Sensor**: Aptina MT9M113 1.3MP (1280x1024) connected via MIPI CSI-2, 1 data lane
- **Device**: HP TouchPad running Linux 6.18

## Working Modes
- **PIX mode** (AXI=0x01): NV12/NV16 capture at 640x480 and 1280x1024 - fully working with good colors, ~30fps
- **VIDEO mode**: Working alongside PIX
- **ZSL mode** (AXI=0x101): Simultaneous PIX preview + snapshot - working with correct colors
- All working modes use the ISP pipeline: CAMIF → DEMUX → XBAR → Write Masters

## Problem: RDI/Raw Bypass Mode (AXI=0x60) - Zero Data
Raw bypass mode should route data: CAMIF → AXI bus → WM0 (bypassing DEMUX/ISP).
**Result**: Zero DMA data captured. No VFE interrupts fire during streaming.

## What We Know Works
- **CSIPHY receives data**: 9000+ data IRQs from sensor during RDI streaming
- **Sensor is streaming RAW**: MT9M113 confirmed in Context B (1280x1024), OFIFO=0x0080 (sensor bypass through FIFO), CAM_OUTPUT_FORMAT=0x0200 (RAW10), color pipeline disabled
- **CAMIF "starts" successfully**: CAMIF_STATUS bit 31 clears (active), CAMIF_CMD=0x5 accepted
- **PIX mode works on same boot**: If we run PIX first, it captures perfectly. Then RDI fails.

## What Fails
- CAMIF_STATUS shows lines=0, pixels=0 even after 10+ seconds of streaming
- PP_STATUS (ping-pong) = 0x00000000 (no DMA activity)
- Zero VFE interrupts during RDI streaming (only 2 from resets)
- AXI_STATUS = 0x00000000

## Register Configuration (Matches Samsung/Opal Exactly)

All values confirmed from decompiled Opal (APQ8060, same SoC) HAL `opal_libqcameralib.c` functions `vfe_raw_snapshot_config()` and `mt9m113_raw_snapshot_config()`, and Samsung Quincy (MSM8660) HAL.

| Register | Offset | Value | Source/Confirmation |
|----------|--------|-------|---------------------|
| CORE_CFG | 0x014 | 0x01 | Pattern=SGRBG(1), NO bit 6. Samsung does `& 0xbf` to clear bit 6 for raw |
| MODULE_CFG | 0x010 | 0x00 | All ISP modules disabled. Samsung raw sets 0 |
| CAMIF_CFG | 0x1E4 | 0x10 | Bit 4 set. Opal: `vfe_set_default_cmd` zeros blob, then `|= 0x10`. Samsung: same |
| FRAME_CFG | 0x1E8 | 0x00 | Zeroed. Both Samsung and Opal leave at 0 for raw |
| WINDOW_WIDTH | 0x1EC | 0x04000A00 | height=1024, width=2560 (width*2, CAMIF counts 16-bit units). Opal uses `firstPixel << 1` |
| WINDOW_HEIGHT | 0x1F0 | 0x09FF | lastPixel = 2560-1 = 2559 |
| SUBSAMPLE_0 | 0x1F4 | 0x03FF | height-1 = 1023 |
| SUBSAMPLE_1 | 0x1F8 | 0xFFFFFFFF | No frame skip |
| AXI_OUT_MODE | 0x040 | 0x60 | CAMIF_TO_AXI_VIA_OUTPUT_2. Samsung/webOS/HTC all use 0x60 for raw |
| BUS_CFG | 0x03C | 0x02AAA775 | 10-bit raw (bits[3:2]=01). HTC HAL: 8-bit=0x771, 10-bit=0x775, 12-bit=0x779 |
| CAMIF_CMD | 0x1E0 | 0x05 | START(bit0) + CLEAR(bit2). Samsung: CAMIF_COMMAND_START=0x00000005 |
| WM0 CFG | 0x04C | 0x01 | WM0 enabled |
| WM0 PING | 0x050 | 0x7C500000 | Valid DMA address |
| WM0 PONG | 0x054 | 0x7C700000 | Valid DMA address |
| WM0 ADDR_CFG | 0x058 | 0x0000038F | UB start=0, depth=911 (full budget) |
| WM0 UB_CFG | 0x05C | 0x003103FF | depth=49 ((1600/32)-1), height=1023 |
| WM0 IMG_SIZE | 0x060 | 0x00643FF2 | stride=1600 (raw10 packed), height=1023, flags=2 |
| IRQ_MASK_0 | 0x01C | 0x00EFE121 | COMPOSITE_DONE + WM0 ping-pong + SOF + REG_UPDATE |
| IRQ_COMP_MASK | 0x034 | 0x00000100 | WM0 in composite group 1 |
| CGC_OVERRIDE | 0x00C | 0x000FFFFF | All clocks enabled |
| XBAR_CFG1 | 0x044 | 0x00000000 | Not used for raw bypass |

## MMCC Clock Configuration
- VFE clock: 266MHz
- CSI1 clock: 384MHz
- csi_pix and csi_rdi both enabled, both select CSI1
- MISC_CC_REG: 0x06003400 (csi_pix_sel=CSI1, csi_pix_en=ON, csi_rdi_sel=CSI1, csi_rdi_en=ON)
- CSI_CC: 0x00000285

## MSM8660 CSID Note
On MSM8660/APQ8060, the CSID is integrated with CSIPHY. There are NO separate CID (Context ID) configuration registers. The CSID is a pass-through - data flows CSIPHY → CSID → VFE CAMIF automatically. No data type filtering or virtual channel selection at the CSID level.

## Samsung Kernel Reference
Source: `LineageOS/android_kernel_samsung_msm8660-q1` branch `ics`

Samsung's `vfe31_config_axi()` for CAMIF_TO_AXI:
```c
case CAMIF_TO_AXI_VIA_OUTPUT_2: {  /* use wm0 only */
    *p = 0x60;    /* raw snapshot with wm0 */
    vfe31_ctrl->outpath.out1.ch0 = 0; /* raw */
    p1 = ao + 6;    /* wm0 for y */
    *p1 = (regp1->paddr + regp1->info.y_off);
}
```

Samsung's IRQ composite mask for raw:
```c
if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
    irq_comp_mask |= (0x1 << (vfe31_ctrl->outpath.out1.ch0 + 8));
    // = 0x1 << (0 + 8) = 0x100
}
```

## Key Observation
The CAMIF receives the start command and shows "active" (CAMIF_STATUS bit 31 = 1 → 0 after start). But it NEVER counts any pixels or lines (both stay at 0). This means data from CSIPHY/CSID is NOT reaching the CAMIF input in raw mode, despite the CSIPHY confirming data is arriving from the sensor.

In PIX mode (AXI=0x01), the same CSIPHY→CSID→CAMIF path works perfectly. The CAMIF counts pixels and lines, generates SOF/EOF, and data flows through DEMUX to WMs.

## What Differs Between PIX (works) and RDI (fails)
| Aspect | PIX (works) | RDI (fails) |
|--------|-------------|-------------|
| AXI_OUT_MODE | 0x01 | 0x60 |
| CORE_CFG | 0x46 (pattern + bit 6) | 0x01 (pattern only) |
| CAMIF_CFG | 0x40 (bit 6) | 0x10 (bit 4) |
| MODULE_CFG | 0x01C00C0C | 0x00 |
| Sensor format | UYVY 8-bit | RAW10 Bayer |
| CSIPHY DATA_FORMAT | 0 (8-bit) | 1 (10-bit) |
| CAMIF pixel counting | Works | Zero |

## Bugs Found and Fixed (but RDI still fails)

### 1. CSIPHY DATA_FORMAT was hardcoded to 8-bit (FIXED)
**Commit**: `ce6759e866ac` and `02bbf2cdb20b`

The MIPI_PROTOCOL_CONTROL register (CSIPHY offset 0x04) DATA_FORMAT field
at bits [20:19] was hardcoded to 0 (8-bit). For RAW10 MIPI packets (DT=0x2B),
this must be 1 (10-bit). Additionally, the `stream_on` function issued a
SW_RST that cleared the register, losing the DATA_FORMAT setting.

**Fix**: Detect sensor format via media pad link, set DATA_FORMAT accordingly,
store in csiphy_device struct, and re-apply after SW_RST in stream_on.

**Verified**: PROTOCOL_CONTROL now shows `0x002e0000 (data_fmt=1)` for RAW10
in both lanes_enable and stream_on. However, CAMIF still counts zero pixels.

### 2. CAMIF_CFG was using VFE8x-only bit (FIXED)
**Commit**: `ae9baea05971`

CAMIF_CFG (0x1E4) was set to 0x80 (bit 7, camif2bus) which doesn't exist on
VFE31. Opal and Samsung HALs use 0x10 (bit 4) for raw snapshot mode.

### 3. CORE_CFG INPUT_MUX_ENABLE for raw mode (RE-ENABLED)
**Commit**: `57ec55a39d93` (cleared), `0866425079f2` (re-enabled for all modes)

Previously cleared bit 6 based on Samsung HAL analysis (`& 0xbf`). However,
re-analysis of the Samsung decompiled code shows the `& 0xbf` operates on
buffer offset 0, NOT CORE_CFG at offset 0x1488. Samsung's actual CORE_CFG
handling: `& 0x8f | 0x10` (always bit 4, no interface-type check).

HTC's decompiled HAL (vfe_operation_config) sets CORE_CFG bits 6:4 based
on sensor interface type: MIPI CSI-2 → 0x40 (bit 6 = INPUT_MUX_ENABLE).
This runs for ALL operation modes including raw snapshot.

Live register readback during working PIX capture confirms: CORE_CFG=0x46
(bit 6 set) → CAMIF counts pixels. Without bit 6 → CAMIF_STATUS=0.

### 4. FRAME_CFG and WINDOW width (FIXED)
**Commit**: `b9a88529442a`

FRAME_CFG was being set with dimensions but Samsung/Opal leave it at 0.
WINDOW_WIDTH was using raw stride (1600 for RAW10) instead of width*2 (2560).
CAMIF counts in 16-bit pixel units. Confirmed from Opal mt9m113_raw_snapshot_config.

### 5. CAMIF_CMD_START was 0x1 instead of 0x5 (FIXED)
**Commit**: `a7a557e0c13b`

Samsung/webOS define CAMIF_COMMAND_START as 0x5 (bit 0 START + bit 2 CLEAR).
The CLEAR bit resets CAMIF status counters on start.

## Gemini AI Analysis (2026-04-22)

Gemini identified three hypotheses:

1. **MIPI Data Type Filter** - CSIPHY DATA_FORMAT field was wrong for RAW10.
   **CONFIRMED AND FIXED** but not sufficient alone.

2. **EFS vs MIPI Short Packet Sync** - CAMIF_CFG bit 4 (0x10) may configure
   hardware sync mode vs embedded frame sync. If the MT9M113 RAW output uses
   MIPI FS/FE short packets but the CAMIF expects EFS codes, it will never
   see frame boundaries. **NOT YET TESTED** - need to investigate EFS register
   configuration and whether CAMIF_CFG=0x10 properly enables MIPI sync mode.

3. **CORE_CFG Input Mux** - Clearing bit 6 bypasses ISP input mux for raw.
   **IMPLEMENTED** per Samsung HAL, but didn't resolve the issue.

## RAW-through-PIX Diagnostic Result (2026-04-23)

**CRITICAL FINDING**: Setting the sensor to RAW10 output while keeping the VFE
in PIX mode (AXI=0x01, CORE_CFG=0x46, CAMIF_CFG=0x40) **successfully captures
data**. The CAMIF counts pixels, VFE IRQs fire, and 5 frames of 1280x1024 are
captured at 3.4fps. The resulting image (Y plane as grayscale) shows a clear,
recognizable scene with proper exposure.

This proves:
1. **The CAMIF front-end CAN see RAW10 MIPI packets** (DT=0x2B)
2. **The CSIPHY/CSID/CAMIF pipeline is working correctly for RAW10**
3. **The issue is strictly in the AXI=0x60 back-end routing**
4. The DATA_FORMAT=0 (8-bit) in CSIPHY doesn't actually block RAW10 at the
   CAMIF level - it affects unpacking but packets still reach the CAMIF

The AXI=0x60 (CAMIF_TO_AXI_VIA_OUTPUT_2) raw bypass path appears non-functional
on this VFE31 revision despite matching all known register values from Samsung
and Opal HALs.

## WM Assignment for Raw Capture

**Confirmed: Samsung kernel uses WM0 only for CAMIF_TO_AXI raw snapshot.**

From Samsung kernel `vfe31_config_axi()`:
```c
case CAMIF_TO_AXI_VIA_OUTPUT_2:  /* use wm0 only */
    vfe31_ctrl->outpath.out1.ch0 = 0;  /* raw = WM0 */
    p1 = ao + 6;   /* ao[6] → VFE 0x050 = WM0 PING_ADDR */
```

Samsung HAL `vfe_snapshot_raw_axi_init()`:
- blob+0x08 = AXI_OUT_MODE = 0x60
- blob+0x14 = WM0 PING_ADDR
- blob+0x40 = WM0 ADDR_CFG = 0x397 (UB depth 911)
- All other WMs zeroed

No evidence of any vendor using WM1-WM6 for raw bypass. The CAMIF_TO_AXI
path is hardwired to route through output2 to WM0 on VFE31.

## Viable Workaround: RAW-through-PIX

Since the CAMIF sees RAW10 through the PIX path, a workaround is possible:
- Set AXI=0x01 (PIX mode) with MODULE_CFG=0 (DEMUX disabled)
- Sensor outputs RAW10, VFE DEMUX is disabled
- WM0 captures the raw data as-is (single WM, no Y/CbCr split)
- Data needs Bayer demosaicing in userspace

This approach was tested successfully: the Y plane of the NV12 output
contains recognizable Bayer RAW data that can be processed offline.

## Remaining Questions
1. Why does AXI=0x60 not work despite matching all vendor register values?
   Is CAMIF_TO_AXI a silicon-level path that may be disabled/broken on
   certain APQ8060 stepping/revision?
2. Is there an EFS (Embedded Frame Sync) register that needs to be disabled
   for RAW MIPI mode? CAMIF_CFG=0x10 may not be sufficient.
3. Samsung's raw snapshot uses a V31_START ioctl (command 3) after AXI config.
   Does this trigger additional kernel-side register writes?
4. Could the VFE31 require a "warm" CAMIF (run PIX first, then switch to
   AXI=0x60 mid-stream) rather than a cold start in raw mode?

## Files
- Driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- CSIPHY: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- CSIPHY generic: `drivers/media/platform/qcom/camss/camss-csiphy.c`
- Sensor: `drivers/media/i2c/mt9m113.c`
- Register reference: `reports/vfe31-register-reference.md`
- Samsung kernel: `https://github.com/LineageOS/android_kernel_samsung_msm8660-q1/tree/ics/drivers/media/video/msm`
- Opal HAL (unstripped, APQ8060): `reports/opal-camera/opal_libqcameralib.c`
- Samsung HAL: `reports/samsung-quincy/quincy_liboemcamera.c`
