# MT9M113 Camera Debug - Questions for Gemini

## Hardware Overview

- **Device:** HP TouchPad tablet
- **SoC:** Qualcomm APQ8060 (MSM8660 variant, dual Scorpion cores)
- **Kernel:** Linux 6.18 mainline with custom camera drivers
- **Sensor:** Aptina MT9M114 front camera (1.3MP, chip ID 0x2481)
- **Interface:** MIPI CSI-2, 1 data lane, UYVY 8-bit output
- **Camera Subsystem:** VFE31 (Video Front End version 3.1)

## Current Status (2026-03-31)

**Critical Discovery:** The VFE31 uses **IMAGE_COMPOSITE_DONE** IRQs (bits 21-23 of IRQ_STATUS_0) for frame completion, NOT MIPI Frame Start/End packets. The webOS VFE31 driver confirms this pattern. However, COMPOSITE_DONE never fires in our driver - **this is the core problem**.

## The Problem

We're porting the HP TouchPad camera drivers to mainline Linux. The MT9M114 front camera sensor initializes correctly and streams MIPI data. The CAMIF receives complete frames (pixel/line counts match), but **no frames are captured to memory**. Output files are always 0 bytes.

## What Works

1. **Sensor initialization** - MT9M114 detected, register table applied
2. **MIPI data reception** - CSIPHY receives DATA IRQs
3. **CSI mux routing** - Correctly selects CSI1 (MISC_CC_REG = 0x06003400)
4. **VFE CAMIF frame reception** - Complete frames received:
   - `CAMIF_STATUS=0x01e00500` (1280 pixels = 640*2 for UYVY, 480 lines)
   - `FRAME_CFG=0x01e00500` (matching expected dimensions)
5. **VFE Write Master** - Configured with valid ping/pong DMA addresses
6. **CAMIF SOF/EOF** - Both fire (bits 0 and 2 of IRQ_STATUS_0)

## What Fails

1. **COMPOSITE_DONE never fires** - Bits 21-23 of IRQ_STATUS_0 stay 0
2. **ping_pong register stays 0x00010000** - Never toggles (WM not completing buffers)
3. **CAMIF_ERROR fires every frame** - status1 bit 0

### VFE IRQ Status Pattern
```
IRQ #11211: status0=0x00000001 status1=0x00000000 (SOF only, good)
IRQ #11212: status0=0x0000001d status1=0x00000001 (SOF+EOF+more, CAMIF_ERROR)
```
status0=0x0000001d breakdown:
- Bit 0: CAMIF_SOF ✓
- Bit 2: CAMIF_EOF ✓
- Bit 3: REG_UPDATE?
- Bit 4: ?

**Critical:** We never see bits 21-23 (IMAGE_COMPOSITE_DONE_0/1/2).

## Root Cause Analysis

### 1. MT9M114 Doesn't Send MIPI FS/FE Packets
The MT9M114 sensor does not transmit proper MIPI Frame Start (FS) or Frame End (FE) short packets. Evidence:
- CSIPHY `sof_count` from hardware FS detection = 0 (never increments)
- This is why CAMIF_ERROR fires - it detects frame boundary issues

### 2. webOS Doesn't Use MIPI FS/FE Either
Analysis of webOS msm_vfe31.c confirms they handle this differently:
- **IRQ_MASK_1 = 0x00400000** (only RESET_ACK, bit 22)
- **CAMIF_ERROR (bit 0) is NOT enabled** in IRQ mask
- Frame completion via **IMAGE_COMPOSITE_DONE**, not MIPI FS/FE
- CAMIF uses pixel/line counting, not packet-based frame sync

### 3. VFE31 Architecture
VFE31 frame completion flow (from webOS):
```
Sensor → CSIPHY → CSID → VFE CAMIF → ISP/Bypass → Write Master → Memory
                                          ↓
                              COMPOSITE_DONE IRQ when WM completes buffer
```

IRQ_COMPOSITE_MASK_0 (0x034) maps Write Masters to COMPOSITE_DONE groups:
- Bits 0-7: WMs mapped to COMPOSITE_DONE_0 (bit 21)
- Bits 8-15: WMs mapped to COMPOSITE_DONE_1 (bit 22)
- Bits 16-23: WMs mapped to COMPOSITE_DONE_2 (bit 23)

### 4. The Catch-22 Problem
We have a timing problem with VFE/CAMIF start:

**Start CAMIF before sensor:**
- CAMIF starts but no data present
- CAMIF_ERROR fires ("no data")
- Previous behavior, was changed

**Start CAMIF after sensor (current):**
- Sensor already streaming
- When CAMIF starts, it immediately sees mid-frame data
- `CAMIF_STATUS=0x01e00500` (full 480 lines) right after START
- CAMIF_ERROR fires ("frame sync lost")

Log evidence:
```
[TIMING] video_start_streaming: enabling VFE0 CAMIF after sensor streaming
VFE: After START: CAMIF_STATUS=0x01e00500 IRQ_STATUS1=0x00000001
```

### 5. Write Master Not Completing
Despite CAMIF receiving complete frames, the Write Master never finishes:
- ping_pong=0x00010000 never changes (should toggle on buffer completion)
- COMPOSITE_DONE never fires
- Data flow from CAMIF to WM appears broken

## What We've Tried

### Previous Attempts
1. ✅ Fixed CSI mux from CSI0 to CSI1 selection
2. ✅ Fixed SUBSAMPLE_CFG_1 from 0xFFFFFFFF to 0 (was skipping 15/16 frames)
3. ✅ Added IRQ_COMPOSITE_MASK configuration
4. ✅ Added UB_CFG (Unified Buffer) configuration
5. ✅ Implemented software SOF via BIT(22) detection
6. ✅ Tried RDI mode - same CAMIF errors

### Recent Attempts (March 2026)
7. ✅ Committed fix to disable CAMIF_ERROR in IRQ_MASK_1 (matches webOS)
8. ✅ Fixed XBAR_CFG1 for PIX mode data routing
9. ✅ Fixed WR_CFG register (removed invalid frame_based bit)
10. ✅ Tested PIX mode (AXI 0x200) vs RAW mode (AXI 0x60) - both fail same way
11. ✅ Verified FRAME_CFG matches received data (1280 pixels, 480 lines)
12. ✅ Added software_eof workaround (enabled but condition not triggering properly)
13. ✅ Decompiled libcamsrc-msm.so - it's just a GStreamer wrapper, no VFE config

### Key Finding
Even though CAMIF_ERROR is not in IRQ_MASK_1, the ISR still reads and processes it from the raw status registers. The ISR is triggered by masked events (SOF, etc.) and then reads ALL status bits.

## Detailed Hardware Data Path

```
MT9M114 Sensor (I2C 0x3c on GSBI4)
    │
    ▼ MIPI CSI-2 (1 lane, UYVY 8-bit, no FS/FE packets)
CSIPHY1 @ 0x04900000
    │
    ▼
CSID1 @ 0x04800400
    │
    ├──────────────────────┐
    │ PIX path (pad 4)     │ RDI path (pad 1)
    ▼                      ▼
VFE31 CAMIF               VFE31 RDI (bypassed)
    │
    ▼ AXI mode 0x200 (PIX) or 0x60 (RAW)
Write Master WM0
    │
    ▼ (DATA NEVER REACHES HERE)
DDR Memory → /dev/video3 (0 bytes)
```

## Key Registers (Latest Values)

| Register | Address | Value | Purpose |
|----------|---------|-------|---------|
| IRQ_MASK_0 | 0x01C | 0x00EFE121 | SOF, EOF, REG_UPDATE, COMPOSITE_DONE, PING_PONG |
| IRQ_MASK_1 | 0x020 | 0x00400000 | Only RESET_ACK (matches webOS) |
| IRQ_COMPOSITE_MASK | 0x034 | 0x00000001 | WM0 → COMP0 for PIX mode |
| AXI_OUT_MODE | 0x040 | 0x00000200 | PIX/preview mode |
| MODULE_CFG | 0x010 | 0x01C00C0C | webOS value |
| CORE_CFG | 0x014 | 0x00000046 | UYVY pixel pattern |
| CAMIF_FRAME_CFG | 0x1E8 | 0x01E00500 | 1280 pixels, 480 lines |
| CAMIF_STATUS | 0x204 | 0x01E00500 | Frame received (matches CFG) |
| XBAR_CFG1 | 0xAF0 | 0x00001A03 | ISP → WM routing |
| WM0_WR_CFG | 0x04C | 0x00000001 | WM enabled |
| WM0_UB_CFG | var | 0x000003FF | Full unified buffer |

## Questions for Gemini

### 1. COMPOSITE_DONE Not Firing
We configure IRQ_COMPOSITE_MASK_0 = 0x00000001 (WM0 → COMP0) and enable COMPOSITE_DONE_0 in IRQ_MASK_0. CAMIF receives complete frames. But IMAGE_COMPOSITE_DONE_0 (bit 21) never fires.

**Question:** What prevents COMPOSITE_DONE from firing even when CAMIF shows complete frames? Is there a register we're missing that connects CAMIF output to the Write Master?

### 2. CAMIF Error Aborting Frames
CAMIF_ERROR fires every frame (due to missing MIPI FS/FE packets). Even though we don't mask it, the error status is set. Does VFE31 hardware abort frame capture when CAMIF_ERROR occurs?

**Question:** Does CAMIF_ERROR actually abort the Write Master, or is it purely informational? webOS doesn't mask CAMIF_ERROR but still captures frames - how?

### 3. VFE31 Data Path from CAMIF to WM
In PIX mode (AXI 0x200), data flows: CAMIF → DEMUX → XBAR → Write Master. In RAW mode (AXI 0x60), data should flow: CAMIF → Write Master (bypassing ISP).

Both modes fail identically with no COMPOSITE_DONE.

**Question:** Is there a specific register that enables the data path from CAMIF to the AXI bus? Something like a gate or enable that we might be missing?

### 4. VFE Start Timing
We start VFE CAMIF after the sensor begins streaming, causing CAMIF to immediately see a full frame. Starting before the sensor caused different errors.

**Question:** How did webOS handle VFE/sensor start ordering? Did they have a mechanism to ignore the first partial frame?

### 5. webOS Frame Sync
Looking at webOS msm_vfe31.c, they:
- Set IRQ_MASK_1 = 0x00400000 (no CAMIF_ERROR)
- Use VFE_IMASK_WHILE_STOPPING_1 = 0x00400000
- Clear IRQ status before starting
- Use COMPOSITE_DONE for frame completion

**Question:** Did webOS have additional initialization we're missing? Perhaps a "sync to frame boundary" step or VFE reset sequence between frames?

### 6. MT9M114 MIPI Timing
The MT9M114 outputs UYVY via MIPI CSI-2 but appears to skip Frame Start/End short packets entirely (or sends non-compliant ones).

**Question:** Is there an MT9M114 register to enable proper MIPI FS/FE packet transmission? Or did webOS work around this entirely via VFE pixel/line counting?

### 7. AXI Bus Issues
The Write Master has valid addresses (ping=0x41E80000, pong=0x41D00000) but never completes a buffer write.

**Question:** Could this be an AXI bus permission or VBIF (VFE Bus Interface) configuration issue? We set VBIF registers but maybe incorrectly.

### 8. Alternative: Test Generator
VFE31 has an internal test generator that bypasses the camera sensor entirely. This could help isolate if the problem is sensor-side or VFE-side.

**Question:** Have you seen VFE31 test generator mode work? We have `vfe31_use_testgen` parameter but haven't fully tested it.

## Reference Code Locations

- VFE31 driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- VFE common: `drivers/media/platform/qcom/camss/camss-vfe.c`
- CSIPHY driver: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- MT9M114 sensor: `drivers/media/i2c/mt9m114.c`
- webOS VFE reference: `~/webos/touchpad-kernel/doctor305/extracted-rootfs/...`

## webOS Reference Data

From analyzing webOS msm_vfe31.c:

**IRQ Masks:**
```c
vfe31_irq_mask_0 = 0x00EFE021  // SOF, REG_UPDATE, PING_PONG, COMPOSITE_DONE
vfe31_irq_mask_1 = 0x00400000  // Only RESET_ACK (bit 22)
VFE_IMASK_WHILE_STOPPING_1 = 0x00400000
```

**AXI Output Modes:**
```c
OUTPUT_2 (0x200) = PIX/preview mode
CAMIF_TO_AXI (0x60) = Raw mode
```

**CAMIF Sync:**
```c
syncMode = 0 (APS mode) for MIPI CSI-2
EFS_CFG = 0 (no embedded sync codes)
```

---

Any insights on why COMPOSITE_DONE doesn't fire would be greatly appreciated!
