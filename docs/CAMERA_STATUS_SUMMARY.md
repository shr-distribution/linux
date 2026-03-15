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

### 1. CRITICAL: OUTPUT_CONTROL Register Write Not Persisting

**Symptom:** Camera streaming times out waiting for VFE SOF (Start of Frame) interrupt. CSIPHY shows `sof_count=0` - no MIPI frames received.

**Root Cause Investigation (March 15, 2026):**

Testing revealed that the OUTPUT_CONTROL register (0x3400) write is NOT persisting:
```
First run:  OUTPUT_CONTROL=0x0, then writes 0x7A08
Second run: OUTPUT_CONTROL=0x0 again (should be 0x7A08 from previous!)
```

Compare with RESET_REGISTER which DOES persist:
```
First run:  RESET_REGISTER was 0x12cc, writes 0x120C
Second run: RESET_REGISTER was 0x120c (correctly persisted)
```

**Possible Causes:**
1. CCI regmap write to 0x3400 is silently failing
2. The register requires a specific access sequence
3. Hardware issue with this particular register

**Debug Commit Added:** `26cbbd91a90e` adds write verification and readback logging.

### 2. CSIPHY Receiving Zero Frames

**Evidence from dmesg:**
```
CSIPHY1: IRQ status=0x00000000 [] sof_count=0
```

The CSIPHY is correctly configured:
- `D1_CONTROL=0x00000300` (PHY enabled)
- `PROTOCOL=0x00260000`
- `CAMERA_CNTL=0x0000e404` (1 lane mode)

But no MIPI data is being received, confirming the sensor is not outputting data.

### 3. VFE31 Configuration Verified Correct

The VFE31 side appears correct:
- `AXI_OUT_MODE=0x200` (OUTPUT_2/preview mode)
- `EFS_CFG=0x0` (APS mode)
- `CAMIF_STATUS` transitions from 0x80000000 (halted) to 0x0 (active)
- IRQ masks properly configured

The VFE is waiting for data that never arrives from the sensor/CSIPHY.

### 4. Potential MIPI Timing Issues

The MT9M113 MIPI timing registers (0xC988-0xC992) are defined in the driver but NOT written during init. The webOS MT9M113 driver also doesn't write them (they're commented out), so this may be intentional - but worth investigating if MIPI errors occur.

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

1. **Clock Enable Path:** Is `vfe_csi1_clk` being enabled by the clock framework when VFE starts? The debug dump should show this.

2. **CSIPHY→VFE Timing:** Does CSIPHY need to be receiving valid MIPI data before VFE CAMIF is started? Or can they be enabled in any order?

3. **MIPI D-PHY Settle Count:** The driver calculates settle count dynamically with 0x14 fallback. Is this correct for MT9M113's link frequency?

4. **Software SOF:** Should we try triggering software SOF to kickstart the VFE state machine?

5. **Test Pattern Mode:** If the sensor's test pattern works but real data doesn't, what does that tell us about the data path?

---

## Recent Commits

```
26cbbd91a90e media: i2c: mt9m114: Add OUTPUT_CONTROL write verification for MT9M113
afbe509fff76 mmc: mmci: Fix use-after-free in DMA error recovery path
1f45115b7ffe media: i2c: mt9m114: Add MT9M113 preview/snapshot AE tables and mode support
38545b2fab58 media: qcom: camss: Fix hardcoded bpp in MSM8660 set_power path
d341a03f989a media: i2c: mt9m114: Add sequencer refresh when MT9M113 not in preview mode
ffbc7313adb6 media: i2c: mt9m114: Fix MT9M113 streaming by always writing RESET_REGISTER
```

---

## Summary

**Current Status: BLOCKED - Clock architecture issue with CSI1 pixel path**

### Major Findings (March 15, 2026 - Session 2)

#### 1. OUTPUT_CONTROL Register Write IS Working ✅
```
MT9M113: OUTPUT_CONTROL=0x7a2c (before write)
MT9M113: OUTPUT_CONTROL after write: 0x7a08 (expected 0x7A08) ✓
```
The CCI write to 0x3400 is succeeding! Previous session's concern was unfounded.

#### 2. CSIPHY IS Receiving MIPI Data ✅
```
CSIPHY1: IRQ #3 status=0x00000030 [SOT ECC ] sof_count=0
CSIPHY1: IRQ #4 status=0x00000830 [SOT ECC ] sof_count=0
```
SOT (Start of Transmission) interrupts prove MIPI PHY is receiving data from sensor!

#### 3. Software SOF Working ✅
With `software_sof_enable=1`:
```
CSIPHY1: sof_count=410, 411, 412... (incrementing)
```
Software SOF triggers VFE interrupt handler, advancing past "SOF timeout" to "REG_UPDATE timeout".

#### 4. CRITICAL: Clock Architecture Issue ❌

**Root Cause Identified:** `csi_pix_clk` in mainline clock driver only has `csi0_src` as parent!

```c
// mmcc-msm8660.c - WRONG for CSI1
static struct clk_branch csi_pix_clk = {
    .parent_hws = (const struct clk_hw*[]){
        &csi0_src.clkr.hw  // <- Only CSI0, no CSI1!
    },
    .num_parents = 1,
    ...
};
```

Compare with MSM8960 which has proper mux:
```c
// mmcc-msm8960.c - CORRECT
static const struct clk_hw *pix_rdi_parents[] = {
    &csi0_clk.clkr.hw,
    &csi1_clk.clkr.hw,  // CSI1 supported!
    &csi2_clk.clkr.hw,
};
```

**Impact:** The pixel interface clock (`csi_pix_clk`) used by VFE's PIX/CAMIF path can only receive data from CSI0 source clock, not CSI1. Since MT9M113 is on CSI1, pixel data cannot flow through to VFE.

---

## Next Steps

### Immediate: Fix Clock Architecture

**Option A: Add clock mux to mmcc-msm8660.c**
- Change `csi_pix_clk` from `clk_branch` to `clk_pix_rdi` type
- Add both `csi0_src` and `csi1_src` as parents
- Add mux selection registers (need to find correct bits)

**Option B: Use RDI path instead of PIX path**
- RDI (Raw Data Interface) might have different routing
- Would require changes to VFE31 configuration

**Option C: Hardware mux register**
- Check if there's a TCSR or other register that selects CSI input
- webOS might configure this in board code we haven't checked

### Investigation Needed

1. **Find CSI mux register:** Search webOS board-tenderloin.c or msm_io_8x60.c for any mux configuration
2. **Check RDI clock path:** Does `csi_rdi_clk` have same limitation?
3. **TCSR configuration:** The TCSR debug dump shows all zeros - is there a mux that needs to be set?

---

## Data Path Analysis

### Current Understanding (VFE PIX mode for MT9M113)

```
MT9M113 Sensor (MIPI CSI-2, 1 lane)
     │
     ▼
CSIPHY1 (0x04900000) ─────── SOT interrupts ✓
     │
     ├──► csi1_src ──► csi1_clk ──► csi1_phy_clk (all enabled ✓)
     │
     ├──► vfe_csi1_clk (enabled ✓, connects CSI1 to VFE)
     │
     ▼
csi_pix_clk ──► parent=csi0_src ONLY ❌
     │                   │
     │              (Cannot receive CSI1 data!)
     ▼
VFE31 CAMIF ─────── Waiting for pixel data that never arrives
     │
     ▼
REG_UPDATE timeout
```

### webOS Clock Binding (for reference)

webOS uses device-specific clock lookup:
```c
// devices-msm8x60.c
CLK_8X60("csi_pclk", CSI0_P_CLK, NULL, OFF),        // Default
CLK_8X60("csi_pclk", CSI1_P_CLK, WEBCAM_DEV, OFF),  // MT9M113
```

Where `WEBCAM_DEV = "msm_camera_mt9m113.0"` gets CSI1 clocks.
