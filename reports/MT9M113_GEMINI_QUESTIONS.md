# MT9M113 Camera Debug - Questions for Gemini

## Hardware Overview

- **Device:** HP TouchPad tablet
- **SoC:** Qualcomm APQ8060 (MSM8660 variant, dual Scorpion cores)
- **Kernel:** Linux 6.18 mainline with custom camera drivers
- **Sensor:** Aptina MT9M113 front camera (1.3MP, chip ID 0x2480)
- **Interface:** MIPI CSI-2, 1 data lane, UYVY 8-bit output
- **Camera Subsystem:** VFE31 (Video Front End version 3.1)

## The Problem

We're porting the HP TouchPad camera drivers to mainline Linux. The MT9M113 front camera sensor initializes correctly and streams MIPI data, but **no frames are captured to memory**. Output files are always 0 bytes.

## What Works

1. **Sensor initialization** - MT9M113 detected, 522-entry register table applied
2. **MIPI data reception** - CSIPHY receives SOT (Start of Transmission) and DATA IRQs
3. **CSI mux routing** - Fixed to correctly select CSI1 (MISC_CC_REG = 0x06003400)
4. **VFE CAMIF** - Receives complete frames (CAMIF_STATUS shows 480+ lines, 1280 pixels)
5. **VFE Write Master** - Configured with valid ping/pong DMA addresses, stride, UB_CFG
6. **Software SOF** - sof_count increments via BIT(22) IRQ at frame rate (~6 fps)

## What Fails

Every frame, the VFE reports these IRQ status bits:
```
status0 = 0x0000001d
  - Bit 0: CAMIF_SOF (Start of Frame detected)
  - Bit 2: CAMIF_NO_SOT (No Start of Transmission)
  - Bit 3: CAMIF_EOF_MISMATCH (End of Frame mismatch)
  - Bit 4: REG_UPDATE

status1 = 0x00000001
  - Bit 0: CAMIF_ERROR
```

The `ping_pong` register stays at 0x00010000 (never toggles), meaning no DMA writes occur.

## Root Cause Analysis

The MT9M113 sensor appears to NOT send proper **MIPI Frame Start (FS) short packets**:

1. CSIPHY `sof_count` from hardware FS detection = 0 (never increments via 0x10000 IRQ)
2. BIT(22) fires at frame boundaries but is undocumented - we use it for software SOF
3. MT9M113 has `CUSTOM_SHORT_PKT` register (0x3404) set to 0x0080 which should enable FS/FE
4. VFE CAMIF reports NO_SOT and EOF_MISMATCH because it can't sync frame boundaries

## Hardware Data Path

```
MT9M113 Sensor (I2C 0x3c)
    │
    ▼ MIPI CSI-2 (1 lane, UYVY 8-bit)
CSIPHY1 @ 0x04900000
    │
    ▼ CSI mux (MISC_CC_REG bits 25,12 = CSI1)
VFE31 CAMIF @ 0x04500000
    │
    ▼ AXI Write Master
DDR Memory → /dev/video3 (0 bytes)
```

## Key Registers

| Register | Address | Value | Purpose |
|----------|---------|-------|---------|
| MISC_CC_REG | 0x04000058 | 0x06003400 | CSI1 mux selection |
| AXI_OUT_MODE | 0x04500040 | 0x00000200 | PIX/preview mode |
| CAMIF_FRAME_CFG | 0x045001E8 | 0x01e00500 | 480 lines, 1280 pixels |
| IRQ_COMPOSITE_MASK | 0x04500034 | 0x00000001 | WM0 → COMP0 |
| UB_CFG | WM offset | 0x000003ff | Full unified buffer |
| CAMIF_STATUS | 0x04500204 | 0x82000500 | Halted, 512 lines received |

## What We've Tried

1. ✅ Fixed CSI mux from CSI0 to CSI1 selection
2. ✅ Fixed SUBSAMPLE_CFG_1 from 0xFFFFFFFF to 0 (was skipping 15/16 frames)
3. ✅ Added IRQ_COMPOSITE_MASK configuration
4. ✅ Added UB_CFG (Unified Buffer) configuration
5. ✅ Implemented software SOF via BIT(22) detection
6. ✅ Tried RDI mode - same CAMIF errors (VFE31 has no true RDI bypass)
7. ✅ Verified MT9M113 CUSTOM_SHORT_PKT = 0x0080 (FS/FE supposedly enabled)

## Questions for Gemini

### 1. MIPI Frame Start Packets
The MT9M113 datasheet mentions CUSTOM_SHORT_PKT register (0x3404) with bit 7 enabling "frame count in short packet". We set this to 0x0080 but CSIPHY never detects Frame Start IRQs.

**Question:** Are there additional MT9M113 registers needed to enable MIPI Frame Start/End short packet transmission? Could there be a separate MIPI timing or output control register?

### 2. VFE31 CAMIF Frame Sync Bypass
The VFE31 CAMIF expects MIPI FS packets for frame synchronization. Without them, it reports CAMIF_NO_SOT and CAMIF_EOF_MISMATCH errors.

**Question:** Is there a VFE31 register or mode to disable frame boundary checking and capture data continuously based on line count alone? Some cameras use "line-based" capture instead of frame-based.

### 3. webOS Frame Sync Mechanism
The original webOS kernel (Linux 2.6.35) had working camera. We have the source code but haven't found explicit frame sync handling for MT9M113.

**Question:** How did Qualcomm MSM8660 cameras typically handle frame synchronization? Did they rely on hardware FS packets, software frame counting, or VSYNC signals?

### 4. CAMIF_ERROR Impact
CAMIF_ERROR (status1 bit 0) fires every frame. The IRQ handler doesn't explicitly block on this, but ping_pong never toggles.

**Question:** Does CAMIF_ERROR actually prevent the Write Master from writing data, or is it just a status indicator? Is there a way to mask or ignore this error?

### 5. Alternative Approaches
Given that MT9M113 may not properly send MIPI FS packets:

**Question:** What are alternative approaches for frame synchronization on VFE31?
- Line counting in software?
- Using VSYNC GPIO if available?
- Modifying CAMIF to use embedded sync codes?
- Patching the sensor firmware?

### 6. Qualcomm Camera HAL
We have Ghidra decompilation of webOS libqcameralib.so which shows VFE configuration.

**Question:** In Qualcomm's camera HAL architecture, where does frame synchronization typically happen - in the kernel VFE driver, userspace HAL, or dedicated hardware? Any insights into how qcameralib handled frame boundaries?

### 7. MT9M113 vs MT9M114
The mainline driver supports both MT9M113 (chip ID 0x2480) and MT9M114 (chip ID 0x2481). MT9M114 is newer and better documented.

**Question:** Are there known differences in MIPI short packet behavior between MT9M113 and MT9M114? Does MT9M114 have more reliable FS/FE packet transmission?

## Reference Code Locations

- VFE31 driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- VFE common: `drivers/media/platform/qcom/camss/camss-vfe.c`
- CSIPHY driver: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- MT9M113 sensor: `drivers/media/i2c/mt9m114.c`
- webOS VFE reference: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c`

## Relevant Datasheets/Docs

- MT9M113 datasheet (Aptina/ON Semiconductor)
- MSM8660 Camera Subsystem documentation (limited availability)
- MIPI CSI-2 specification (for short packet format)

---

Any insights or suggestions would be greatly appreciated!
