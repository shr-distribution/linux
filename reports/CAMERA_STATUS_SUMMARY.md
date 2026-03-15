# HP TouchPad Camera Driver Status Summary

**Date:** March 2026
**Kernel:** Linux 6.18 mainline
**Target:** HP TouchPad (APQ8060 / MSM8660 SoC)
**Goal:** Get MT9M113 front camera working with mainline CAMSS drivers

---

## Hardware Overview

### Platform
- **SoC:** Qualcomm APQ8060 (dual-core Scorpion ARMv7 @ 1.5GHz)
- **Camera Subsystem:** MSM8660 CAMSS (VFE31 architecture)
- **Sensor:** Aptina MT9M113 front-facing camera
  - Chip ID: 0x2480
  - Interface: MIPI CSI-2, 1 data lane
  - Connected to: CSIPHY1 / CSI1

### MT9M113 Sensor Characteristics
- **Resolution Contexts:**
  - Context A (Preview): 640x480 @ 30fps
  - Context B (Capture): 1280x1024 @ 15fps
- **Output Format:** UYVY 8-bit (YUV422)
- **MIPI Link Frequency:** 96 MHz
- **MCU Architecture:** Uses XDMA indirect access (0x098C/0x0990) for MCU variables
- **Command Interface:** SEQ_CMD variable (0xA103) - different from MT9M114's hardware register

### MSM8660 CAMSS Architecture
```
MT9M113 Sensor
     |
     v (MIPI CSI-2, 1 lane, UYVY 8-bit)
CSIPHY1 (0x04900000)
     |
     v
CSID1 (unified with CSIPHY on MSM8660 - no separate ISPIF)
     |
     +---> RDI path (pads 1-3) -> /dev/video0-2 (raw passthrough)
     |
     +---> PIX path (pad 4) -> VFE31 CAMIF -> /dev/video3 (ISP processed)
```

**Key MSM8660 differences from newer SoCs:**
- No ISPIF (ISP Interface) - CSID connects directly to VFE
- VFE31 architecture (older than VFE40+)
- Unified CSIPHY+CSID register space
- CAMIF uses APS (Active Pixel Sync) mode by default

---

## Driver Implementation Status

### MT9M113 Sensor Driver (`drivers/media/i2c/mt9m114.c`)

**Status: COMPLETE - Verified against webOS legacy kernel**

| Component | Status | Notes |
|-----------|--------|-------|
| Chip detection | ✅ Done | Detects 0x2480, selects MT9M113 code path |
| Init table (480 entries) | ✅ Done | Byte-for-byte match with webOS `preview_snapshot_mode_reg_tbl` |
| MCU variable helpers | ✅ Done | `mt9m113_write_mcu_var()`, `mt9m113_read_mcu_var()`, `mt9m113_poll_mcu_var()` |
| PLL configuration | ✅ Done | In init table, matches webOS (0x0114 dividers) |
| Preview AE table | ✅ Done | Applied during `start_streaming()` |
| Snapshot AE table | ✅ Done | `mt9m113_set_snapshot_mode()` function available |
| Streaming control | ✅ Done | SEQ_CAP_MODE, SEQ_CMD, RESET_REGISTER sequence |
| MIPI OUTPUT_CONTROL | ✅ Done | 0x7A08 (LP clock mode), deferred to streaming |
| Stop streaming | ✅ Done | Uses SEQ_CMD_STANDBY (0x0003) |

**Streaming Sequence (matches webOS exactly):**
1. Write OUTPUT_CONTROL = 0x7A08 (enable MIPI)
2. Write RESET_REGISTER = 0x120C (streaming mode)
3. Write SEQ_CAP_MODE = 0x0030 (preview mode)
4. Delay 40ms
5. Write SEQ_CMD = 0x0001 (RUN)
6. Poll SEQ_STATE for 0x03 (preview active)
7. Apply preview AE table (5 MCU variable writes)

### CAMSS/VFE31 Driver (`drivers/media/platform/qcom/camss/`)

**Status: MOSTLY COMPLETE - Some infrastructure fixes applied**

| Component | Status | Notes |
|-----------|--------|-------|
| CSIPHY 8x60 | ✅ Verified | Register offsets match webOS, settle count calculation correct |
| CSID 8x60 | ✅ Verified | Pass-through architecture correct for MSM8660 |
| VFE31 core | ✅ Fixed | CAMIF register layout corrected |
| VFE31 clocks | ⚠️ Needs verification | vfe_csi1_clk path may need debugging |
| CAMIF sync mode | ✅ Fixed | Uses APS mode (EFS_CFG = 0) |
| AXI output mode | ✅ Fixed | Uses 0x60 for CAMIF_TO_AXI |

**VFE31 Fixes Applied:**
1. `BUS_CFG (0x03C)`: Correctly NOT written (part of AXI block, not config)
2. `EFS_CFG (0x1E4)`: Set to 0 for APS mode
3. `FRAME_CFG (0x1E8)`: Correct dimensions layout
4. `CAMIF_CMD`: Uses 1 (not legacy 0x5)

---

## Known Issues / Areas Needing Investigation

### 1. ~~OUTPUT_CONTROL Register Write~~ ✅ RESOLVED

**Previous Concern:** OUTPUT_CONTROL (0x3400) write not persisting.

**Resolution (Session 2):** Write IS working correctly:
```
MT9M113: OUTPUT_CONTROL=0x7a2c (before write)
MT9M113: OUTPUT_CONTROL after write: 0x7a08 (expected 0x7A08) ✓
```
The CCI write to 0x3400 succeeds. Register resets to 0 between boots (expected behavior).

### 2. CRITICAL: CSIPHY→VFE Data Path Gap

**Symptom:** CSIPHY receives MIPI line data but VFE CAMIF never receives pixels.

**Evidence from dmesg (Session 3):**
```
CSIPHY1: IRQ status=0x00000030 [SOT ECC ] sof_count=0
CSIPHY1: IRQ status=0x00000830 [SOT ECC ] sof_count=0
```

- **SOT (Start of Transmission)**: MIPI lane-level data arriving ✓
- **ECC**: Error correction processing ✓
- **BIT(16) Frame Start**: Never fires ❌
- **sof_count=0**: No frame sync detected

**CSIPHY Configuration (verified correct):**
- `D1_CONTROL=0x00000300` (PHY enabled)
- `PROTOCOL=0x00260000`
- `CAMERA_CNTL=0x0000e404` (1 lane mode)

**Root Cause Hypothesis:**
Something between CSIPHY and VFE CAMIF is not routing data correctly. The csi_pix_clk/csi_rdi_clk clocks may be irrelevant on MSM8660 (webOS doesn't use them).

### 3. VFE31 Configuration Verified Correct ✅

The VFE31 side appears correct:
- `AXI_OUT_MODE=0x200` (OUTPUT_2/preview mode)
- `EFS_CFG=0x0` (APS mode)
- `CAMIF_STATUS` transitions from 0x80000000 (halted) to 0x0 (active)
- IRQ masks properly configured

At shutdown, CAMIF_STATUS returns to 0x80000000 (halted) - indicating no data was ever received and processed.

### 4. MIPI Frame Start Packets Missing

**Key Finding:** CSIPHY sees SOT (line-level) but not Frame Start (frame-level).

**Possible Causes:**
1. MT9M113 not sending Short Packet Frame Start (expected for SOF interrupt)
2. MT9M113 using Embedded Data for frame sync instead
3. OUTPUT_CONTROL (0x3400) value needs different bits for frame packets

**Note:** The MT9M113 MIPI timing registers (0xC988-0xC992) are defined but not written. webOS also doesn't write them.

### 5. Clock Architecture Difference from webOS

**webOS does NOT use csi_pix_clk or csi_rdi_clk:**
```c
// webOS MISC_CC_REG = 0x00000000 (no mux bits)
// Yet camera works - implies different data path
```

**Mainline adds csi_pix/csi_rdi for MSM8960+ style routing:**
```c
// Mainline MISC_CC_REG = 0x06003440 (mux bits set)
// May not be correct for MSM8660 architecture
```

**Hypothesis:** MSM8660 may have simpler CSI→VFE routing that doesn't require these clocks.

---

## Test Modes Available

The test script (`scripts/test-camera.sh`) supports these modes:

| Mode | Command | Description |
|------|---------|-------------|
| Preview | `./test-camera.sh preview` | 640x480 via VFE PIX (MT9M113 Context A) |
| PIX | `./test-camera.sh pix` | 1280x968 via VFE PIX (ISP processed) |
| RAW | `./test-camera.sh raw` | 1288x968 via RDI (raw passthrough) |
| Test Pattern | `./test-camera.sh testpattern` | Enable sensor color bars |
| EFS Mode | `./test-camera.sh efs` | Try EFS sync instead of APS |
| Debug | `./test-camera.sh debug` | Full clock/register dump |

---

## Code Locations

| File | Description |
|------|-------------|
| `drivers/media/i2c/mt9m114.c` | MT9M113/MT9M114 sensor driver |
| `drivers/media/platform/qcom/camss/camss-vfe-4-1.c` | VFE31 implementation |
| `drivers/media/platform/qcom/camss/camss-csiphy.c` | CSIPHY driver |
| `drivers/media/platform/qcom/camss/camss-csid.c` | CSID driver |
| `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` | Device tree |

---

## Reference Documentation

### webOS Legacy Kernel Files (for comparison)
- `webos-linux-kernel-touchpad/drivers/media/video/msm/mt9m113.c`
- `webos-linux-kernel-touchpad/drivers/media/video/msm/mt9m113_reg.c`
- `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_io_8x60.c`

### Key Registers

**MT9M113 Sensor:**
| Register | Address | Purpose |
|----------|---------|---------|
| MODEL_ID | 0x0000 | Chip ID (0x2480) |
| RESET_REGISTER | 0x301A | 0x120C=streaming, 0x12CE=snapshot |
| OUTPUT_CONTROL | 0x3400 | 0x7A08=MIPI LP mode |
| MCU_ADDRESS | 0x098C | XDMA address register |
| MCU_DATA | 0x0990 | XDMA data register |
| SEQ_CMD | 0xA103 (MCU) | 0x01=RUN, 0x02=CAPTURE, 0x03=STANDBY |
| SEQ_STATE | 0xA104 (MCU) | 0x03=preview, 0x07=capture |
| SEQ_CAP_MODE | 0xA115 (MCU) | 0x0030=preview, 0x0000=snapshot |

**VFE31:**
| Register | Offset | Purpose |
|----------|--------|---------|
| AXI_OUT_MODE | 0x040 | 0x60=CAMIF_TO_AXI |
| EFS_CFG | 0x1E4 | 0x00=APS mode |
| FRAME_CFG | 0x1E8 | Frame dimensions |
| CAMIF_CMD | 0x1EC | 0x01=enable CAMIF |

---

## Questions for Investigation

### Answered ✅

1. **Clock Enable Path:** Is `vfe_csi1_clk` being enabled?
   - **Answer:** Yes, all clocks enabled correctly. MISC_CC_REG shows proper mux selection.

2. **Software SOF:** Should we try triggering software SOF?
   - **Answer:** Yes, tested with `software_sof_enable=1`. SOF triggers work but REG_UPDATE still times out because no pixel data.

### Open Questions

3. **Why no Frame Start (BIT 16) in CSIPHY?**
   - CSIPHY sees SOT (line level) but never Frame Start (frame level)
   - Does MT9M113 send MIPI Short Packets for frame start?
   - Or does it use embedded data sync?

4. **Are csi_pix_clk/csi_rdi_clk relevant to MSM8660?**
   - webOS doesn't use these clocks
   - webOS MISC_CC_REG = 0x00000000
   - May be MSM8960+ only additions

5. **What is the actual CSIPHY→VFE data path on MSM8660?**
   - webOS uses CSI0_VFE_CLK / CSI1_VFE_CLK directly
   - No intermediate csi_pix/csi_rdi layer
   - May need to trace webOS msm_io_8x60.c more carefully

6. **VFE31 Input Source Configuration:**
   - Does VFE31 have an OPERATION_CFG or input source register?
   - How does VFE know to read from CSI1 vs CSI0?
   - webOS may configure this somewhere we haven't found

7. **MIPI Data Format:**
   - Is MT9M113 outputting data in the format VFE expects?
   - Check DATA_TYPE setting in CSIPHY matches sensor output

---

## Recent Commits

```
0164a187d91f mmc: mmci: Fix race between mmci_pre_request and mmci_dmae_error
6db84f96d35a media: qcom: camss: Set csi_pix/csi_rdi clock parents to CSI1 for MSM8660
f36b6df90003 clk: qcom: mmcc-msm8660: Add CSI PIX/RDI clock parent mux support
26cbbd91a90e media: i2c: mt9m114: Add OUTPUT_CONTROL write verification for MT9M113
afbe509fff76 mmc: mmci: Fix use-after-free in DMA error recovery path
1f45115b7ffe media: i2c: mt9m114: Add MT9M113 preview/snapshot AE tables and mode support
38545b2fab58 media: qcom: camss: Fix hardcoded bpp in MSM8660 set_power path
d341a03f989a media: i2c: mt9m114: Add sequencer refresh when MT9M113 not in preview mode
```

---

## Summary

**Current Status: BLOCKED - Pixel data not reaching VFE CAMIF despite correct clock configuration**

### Major Findings (March 15, 2026 - Session 3)

#### 1. Clock Mux Fix Applied ✅
```c
// mmcc-msm8660.c - Now has proper mux support
static struct clk_pix_rdi csi_pix_clk = {
    .s_reg = 0x0058,
    .s_mask = BIT(25),  // Mux select: 0=CSI0, 1=CSI1
    .clkr = {
        .enable_reg = 0x0058,
        .enable_mask = BIT(26),
        // parents = csi0_clk, csi1_clk
    },
};
```
CAMSS driver now calls `clk_set_parent()` to select CSI1 for MT9M113.

#### 2. All Clocks Verified Enabled ✅
```
clock 'csi_pix' enabled, rate=384000000
clock 'csi_rdi' enabled, rate=384000000
MISC_CC_REG = 0x06003440
  csi_pix_sel: CSI1 (bit 25 set)
  csi_rdi_sel: CSI1 (bit 12 set)
```
Both clock enables and mux selections are correct.

#### 3. CSIPHY Receiving Line-Level MIPI Data ✅
```
CSIPHY1: IRQ #3 status=0x00000030 [SOT ECC ] sof_count=0
CSIPHY1: IRQ #4 status=0x00000830 [SOT ECC ] sof_count=0
```
- **SOT (Start of Transmission)** interrupts: MIPI PHY receiving lane data ✓
- **ECC** interrupts: Error correction active ✓
- **BIT(16) Frame Start**: Never fires ❌
- **sof_count=0**: No frame-level sync packets detected

#### 4. Software SOF Workaround Tested ✅
With `software_sof_enable=1`:
```
CSIPHY1: sof_count=410, 411, 412... (incrementing)
```
Software SOF triggers VFE interrupt handler, advancing past "SOF timeout" to "REG_UPDATE timeout".

#### 5. VFE CAMIF Never Receives Pixel Data ❌
```
Before start: CAMIF_STATUS = 0x80000000 (HALTED)
After start:  CAMIF_STATUS = 0x00000000 (ACTIVE)
At shutdown:  CAMIF_STATUS = 0x80000000 (still HALTED - no data received!)
```
VFE CAMIF is enabled and waiting but never receives any pixel data.

#### 6. Both PIX and RDI Modes Fail ❌
- **PIX mode (line_id=3)**: VFE SOF timeout → REG_UPDATE timeout
- **RDI mode (line_id=0)**: Same failure pattern

This confirms the issue is not specific to the VFE processing path.

### Critical Discovery: webOS Clock Architecture Difference

**webOS does NOT use csi_pix_clk or csi_rdi_clk!**

```c
// webOS msm_io_8x60.c - No csi_pix or csi_rdi clocks
static struct clk_info msm_8x60_clk_info[] = {
    {"csi_phy_clk", CSI0_PHY_CLK, NULL, 0},  // Only CSI clocks
    {"csi_clk",     CSI0_CLK,     NULL, 0},
    {"csi_vfe_clk", CSI0_VFE_CLK, NULL, 0},  // CSI0_VFE_CLK
    // MT9M113 uses CSI1 variants via WEBCAM_DEV lookup
};
```

The webOS MISC_CC_REG value is **0x00000000** (no mux selection bits set), yet camera works. This suggests:
1. csi_pix_clk/csi_rdi_clk may be mainline-only additions
2. The actual data path doesn't require these clocks on MSM8660
3. Data routing may use a different hardware mechanism

---

## Next Steps

### Immediate Investigation

1. **Compare CSI-to-VFE data path with webOS**
   - webOS uses CSI0_VFE_CLK and CSI1_VFE_CLK directly
   - No intermediate csi_pix/csi_rdi layer
   - May need to bypass these clocks entirely

2. **Check VFE Input Selection**
   - VFE31 may have an input source register not yet configured
   - Look for OPERATION_CFG or similar in VFE31 register set
   - webOS VFE may configure this in a different path

3. **MIPI Frame Sync Packets**
   - Sensor sends SOT (line level) but no Frame Start (BIT 16)
   - Check if MT9M113 needs explicit configuration for frame sync
   - Verify OUTPUT_CONTROL value enables frame packets

### Hardware Data Path Investigation

Current understanding:
```
MT9M113 Sensor (MIPI CSI-2, 1 lane)
     │
     ▼ (SOT/ECC interrupts confirm line data arriving)
CSIPHY1 (0x04900000) ✓
     │
     ├──► csi1_phy_clk (enabled ✓)
     ├──► vfe_csi1_clk (enabled ✓)
     │
     ▼
??? Unknown routing point ???
     │
     ├──► csi_pix_clk (enabled, parent=CSI1) - may be unused on MSM8660
     │
     ▼
VFE31 CAMIF ❌ (no pixel data arrives)
```

**Key Question:** What connects vfe_csi1_clk output to VFE CAMIF input on MSM8660?

### Possible Root Causes

1. **Missing hardware mux register** - The data path has a mux not yet configured
2. **Clock domain crossing issue** - csi_pix/csi_rdi clocks don't exist on MSM8660
3. **VFE input source config** - VFE31 needs explicit CSI1 selection
4. **MIPI frame format** - Sensor outputting data but not in expected format

---

## Data Path Analysis

### Current Understanding (After Session 3)

```
MT9M113 Sensor (MIPI CSI-2, 1 lane)
     │
     ▼ (SOT/ECC interrupts confirm line data)
CSIPHY1 (0x04900000) ✓
     │
     ├──► csi1_clk, csi1_phy_clk (enabled ✓)
     │
     ├──► vfe_csi1_clk (enabled ✓, connects CSI1 to VFE)
     │
     ├──► csi_pix_clk (enabled ✓, mux=CSI1) - may be MSM8960+ only?
     │
     │    ??? DATA PATH GAP ???
     │
     ▼
VFE31 CAMIF (0x04500000 + 0x1E0)
     │
     └──► Waiting for pixel data that never arrives
          REG_UPDATE timeout
```

### webOS Clock Architecture (for comparison)

webOS uses direct clock mapping without csi_pix/csi_rdi:
```c
// devices-msm8x60.c - Direct CSI-to-VFE clocks
CLK_8X60("csi_vfe_clk", CSI0_VFE_CLK, NULL, OFF),      // Default (back camera)
CLK_8X60("csi_vfe_clk", CSI1_VFE_CLK, WEBCAM_DEV, OFF), // MT9M113 (front camera)
```

MISC_CC_REG = 0x00000000 in webOS (no mux bits set).
