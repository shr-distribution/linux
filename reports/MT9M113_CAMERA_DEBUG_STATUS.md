# MT9M113 Front Camera Debug Status - HP TouchPad

**Last Updated:** 2026-03-27 (Session 2 - Calibration Mode Testing)
**Kernel:** Linux 6.18-tenderloin
**SoC:** Qualcomm APQ8060 (MSM8660 variant)
**Sensor:** MT9M113 front camera (1.3MP, chip ID 0x2480)
**Interface:** MIPI CSI-2, 1 data lane, UYVY 8-bit

---

## Executive Summary

**The camera pipeline receives MIPI data at CSIPHY but fails to deliver it to VFE CAMIF.**

### Session 2 Update (March 27, 2026)

**NEW FINDINGS:**
1. Calibration bypass mode significantly reduces ECC errors
2. High settle_cnt values (0x50-0x60) further improve signal quality
3. CSIPHY now receives Frame Start (FS) and Frame End (FE) packets successfully
4. **NEW CRITICAL ISSUE:** VFE CAMIF never receives SOF despite CSIPHY seeing frames

**Current State:**
- CSIPHY1 receives clean FS/FE MIPI short packets ✓
- Continuous clock mode active (OUTPUT_CONTROL=0x7A0C) ✓
- VFE CSI1 bridge clock enabled (vfe_csi1_clk running) ✓
- **Problem:** VFE CAMIF_STATUS stays at 0x80000000 (halted) - never sees frames
- Data path from CSIPHY to VFE appears broken

### Previous Session Summary
- ~51-52% ECC error rate across ALL 75 timing parameter combinations
- This rate was consistent regardless of settle_cnt or HS_TERM_IMP values

---

## Hardware Architecture

### MSM8660 Camera Data Path

```
MT9M113 Sensor (I2C: 0x3c)
     │
     ▼ (MIPI CSI-2, 1 lane, UYVY 8-bit)
┌─────────────────────────────────────────────────┐
│ CSIPHY1 @ 0x04900000                            │
│ - Unified CSIPHY+CSID block (MSM8660 specific)  │
│ - No separate CSID, No ISPIF on this SoC        │
│ - IRQ: SOT ✓, ECC errors ✗ (~52% of packets)    │
└─────────────────────────────────────────────────┘
     │
     │  ← Data routing is AUTOMATIC (no ISPIF)
     │  ← Clock bridge: vfe_csi1_clk + csi_pix/csi_rdi
     │
     ▼
┌─────────────────────────────────────────────────┐
│ VFE31 @ 0x04500000                              │
│ - HW Version: 0x00030217                        │
│ - CAMIF receives SOF ✓                          │
│ - CAMIF_ERROR due to corrupted data ✗           │
│ - REG_UPDATE timeout (no complete frames) ✗     │
└─────────────────────────────────────────────────┘
     │
     ▼
AXI → DDR Memory → /dev/video3 (V4L2)
```

### Key MSM8660 Architecture Notes

| Feature | MSM8660 | MSM8974+ |
|---------|---------|----------|
| CSIPHY/CSID | **Unified block** | Separate blocks |
| ISPIF | **Not present** | Required for routing |
| Data path | CSIPHY → VFE (direct) | CSIPHY → CSID → ISPIF → VFE |
| VFE inputSource mux | **Not present** | Available |

---

## Current Status

### What Works

1. **Sensor Detection & Initialization**
   - MT9M113 detected at I2C address 0x3c (chip ID 0x2480)
   - PLL configuration matches webOS exactly
   - MCU boot sequence completes successfully
   - Sensor enters streaming state (SEQ_STATE=0x3, SEQ_CMD=0x0)

2. **MIPI Output Configuration**
   - OUTPUT_CONTROL = 0x7A08 (MIPI enabled, LP clock mode)
   - RESET_REGISTER = 0x120C (streaming mode)
   - Streaming sequence matches webOS order exactly

3. **CSIPHY Configuration**
   - All CSI1 clocks enabled (CSI1_CLK, CSI1_PHY_CLK, vfe_csi1_clk)
   - PHY calibration completes (bit 23 set in CALIBRATION_CONTROL)
   - SOT (Start of Transmission) sync bytes detected consistently
   - All 9 CSIPHY registers match webOS values exactly

4. **VFE Configuration**
   - CAMIF configured correctly for 640x480 or 1280x1024
   - FRAME_CFG shows correct pixel/line counts
   - CAMIF_SOF (Start of Frame) interrupts received
   - AXI_OUT_MODE and EFS_CFG match webOS values

5. **Clock Configuration**
   - MISC_CC_REG = 0x06003440 (csi_pix/csi_rdi enabled, CSI1 selected)
   - All 7 VFE clocks present and enabled
   - vfe_csi1_clk not halted (DBG_BUS_VEC_B bit 8 = 0)

### What Fails

1. **Persistent ECC Errors**
   - ~51-52% of all MIPI packets have ECC errors
   - This rate is **consistent across ALL tested parameter combinations**
   - Pattern: SOT detected → ECC error on packet header

2. **VFE Frame Capture**
   - VFE receives CAMIF_SOF but times out waiting for REG_UPDATE
   - CAMIF_ERROR (status1 bit 0) fires due to corrupted data
   - ping_pong register never toggles (no complete frames)
   - Captured files are 0 bytes

---

## Detailed Test Results

### Session 2: Calibration Mode Testing (March 27, 2026)

Based on Gemini AI analysis suggesting hardware calibration may override HS_TERM_IMP settings, we added a `calibration_mode` parameter:

| Mode | Description | Result |
|------|-------------|--------|
| 0 | Normal - poll for bit 23, apply impedance after | ~52% ECC |
| 1 | **Bypass** - no calibration, direct impedance write | **Cleaner IRQs, FS/FE detected** |
| 2 | webOS-style - write cal+impedance without polling | Similar to mode 0 |

**Calibration Bypass + High Settle Count Results:**

| settle_cnt | Calibration | IRQ Quality | Notes |
|------------|-------------|-------------|-------|
| 0x14 (default) | Mode 0 | ~52% ECC | Baseline |
| 0x30 | Mode 1 (bypass) | Reduced ECC, occasional | Better |
| 0x40 | Mode 1 (bypass) | 5 IRQs, 1 ECC | Good |
| 0x50 | Mode 1 (bypass) | FS/FE packets visible | Excellent |
| 0x60 | Mode 1 (bypass) | Clean FS/FE, rare ECC | **Best** |

**Typical IRQ pattern with settle=0x60, calibration bypass:**
```
CSIPHY1: IRQ status=0x00200010 [SOT ]      - Clean Start of Transmission
CSIPHY1: IRQ status=0x00010040 [FS ]       - Frame Start short packet!
CSIPHY1: IRQ status=0x00000080 []          - Frame End detected
CSIPHY1: IRQ status=0x004f00c0 [FS FE ]    - Frame Start + Frame End
... occasional ECC at end ...
```

**Key Discovery:** With calibration bypass and high settle count, CSIPHY successfully receives complete MIPI frames (FS and FE short packets). However, VFE CAMIF still never sees SOF.

### VFE CAMIF Status During Session 2 Tests

Despite CSIPHY receiving frames:
```
VFE: CAMIF_STATUS before REG_UPDATE: 0x80000000  <- halted
VFE: After START: CAMIF_STATUS=0x80000000        <- STILL halted!
[timeout]
VFE sof timeout
VFE reg update timeout
```

**The CAMIF never leaves halted state** even though:
- CAMIF_CMD START is issued
- CSI1_VFE_CLK is enabled and running
- CSIPHY is receiving valid frame data

### Session 1: settle_cnt / hs_term_imp Parameter Sweep

Tested **75 combinations** with properly configured pipeline:
- settle_cnt: 0x04 to 0x20 (4 to 32 decimal)
- hs_term_imp: 0x00, 0x03, 0x07, 0x0B, 0x0F

**Results (best combinations by activity):**

| settle_cnt | hs_term_imp | Total IRQs | ECC Errors | ECC % |
|------------|-------------|------------|------------|-------|
| 0x18       | 0x0F        | 3073       | 1601       | 52%   |
| 0x06       | 0x03        | 2284       | 1204       | 52%   |
| 0x14       | 0x07        | 2169       | 1128       | 52%   |
| 0x14       | 0x0F        | 2083       | 1079       | 51%   |
| 0x1A       | 0x0B        | 2045       | 1071       | 52%   |
| 0x1E       | 0x0B        | 1350       | 696        | 51%   |

**webOS default values:** settle_cnt=0x14 (20), hs_term_imp=0x0F

**Critical Finding:** ECC error rate is locked at 51-52% regardless of timing parameters. This is NOT a timing calibration problem - it's systematic.

### CSIPHY IRQ Pattern Analysis

Typical IRQ sequence observed:
```
CSIPHY1: IRQ status=0x00600010 [SOT]           - Clean SOT
CSIPHY1: IRQ status=0x00000030 [SOT ECC]       - SOT + ECC error
CSIPHY1: IRQ status=0x00000830 [SOT ECC]       - SOT + ECC + bit 11
CSIPHY1: IRQ status=0x00000020 [ECC]           - ECC only
CSIPHY1: IRQ status=0x00000010 [SOT]           - Clean SOT
CSIPHY1: IRQ status=0x00000020 [ECC]           - ECC only
... alternating pattern continues ...
```

The alternating SOT/ECC pattern suggests:
- PHY successfully syncs to MIPI SOT bytes (0xB8)
- Subsequent packet header data is consistently misread
- Every other packet type may be corrupted

### VFE Status During Capture Attempt

```
VFE: CAMIF_STATUS before REG_UPDATE: 0x80000000 (halted)
VFE: After START: CAMIF_STATUS=0x00000000 IRQ_STATUS0=0x00000000
VFE IRQ: status0=0x00000001 status1=0x00000000  <- CAMIF_SOF received!
VFE IRQ: status0=0x0000001d status1=0x00000001  <- CAMIF_ERROR
[1.5 seconds later]
VFE reg update timeout
CAMIF: status=0x81e00500 (halted, frame incomplete)
```

---

## Code Changes Made

### 1. Streaming Sequence (drivers/media/i2c/mt9m114.c)

Changed to match webOS mt9m113_set_sensor_mode() order exactly:

```c
/* webOS streaming sequence:
 * 1. CSIPHY configured (done by V4L2 pipeline)
 * 2. mdelay(10) - wait for CSIPHY to stabilize
 * 3. OUTPUT_CONTROL = 0x7A08 (enable MIPI output)
 * 4. RESET_REGISTER = 0x120C (streaming mode)
 * 5. SEQ_CAP_MODE = 0x0030 (preview mode)
 * 6. mdelay(40)
 * 7. SEQ_CMD = 0x0001 (RUN)
 *
 * CRITICAL: MIPI must be enabled BEFORE SEQ_CMD
 */
msleep(10);
cci_write(MT9M113_OUTPUT_CONTROL, 0x7A08);  // Enable MIPI FIRST
cci_write(MT9M114_RESET_REGISTER, 0x120C);  // Streaming mode
// ... SEQ_CAP_MODE, delay, SEQ_CMD follow
```

### 2. CSIPHY Configuration (drivers/media/platform/qcom/camss/camss-csiphy-8x60.c)

Configuration matches webOS msm_camio_csi_config():

```c
// D0-D3_CONTROL2: settle_cnt, HS_TERM_IMP, LP_REC_EN, ERR_SOT_HS_EN
val = (settle_cnt << 24) | (hs_term_imp << 16) | (1 << 4) | (1 << 3);
writel(val, MIPI_PHY_D0_CONTROL2);  // 0x140F0018 with defaults

// CL_CONTROL: HS_TERM_IMP, LP_REC_EN
writel(0x0F000004, MIPI_PHY_CL_CONTROL);

// PHY_CONTROL: SOT_ECC_EN
writel(0x4, MIPI_PHY_CONTROL);

// PROTOCOL_CONTROL: LONG_PACKET_HEADER_CAPTURE | DECODE_ID | ECC_EN
writel(0x00260000, MIPI_PROTOCOL_CONTROL);

// CALIBRATION_CONTROL - poll for bit 23 (cal done)
writel(0x00700000, MIPI_CALIBRATION_CONTROL);

// CAMERA_CNTL: lane_assign | 1 lane
writel(0x0000e404, MIPI_CAMERA_CNTL);
```

### 3. VFE31 Configuration (drivers/media/platform/qcom/camss/camss-vfe-3-1.c)

- AXI_OUT_MODE set to match webOS (0x60 for raw, 0x200 for PIX)
- EFS_CFG = 0x00 (APS mode, not EFS sync)
- CAMIF start sequence matches webOS exactly

---

## Register State Comparison

### CSIPHY1 Registers (ALL MATCH webOS)

| Register | Offset | Our Value | webOS Value | Status |
|----------|--------|-----------|-------------|--------|
| MIPI_PHY_CONTROL | 0x00 | 0x00000004 | 0x00000004 | ✓ |
| MIPI_PROTOCOL_CONTROL | 0x04 | 0x00260000 | 0x00260000 | ✓ |
| MIPI_CALIBRATION_CONTROL | 0x18 | 0x00F00000 | 0x00E00080 | ✓ (cal done) |
| MIPI_PHY_D0_CONTROL | 0x34 | 0x00000000 | 0x00000000 | ✓ |
| MIPI_PHY_D0_CONTROL2 | 0x38 | 0x140F0018 | 0x140F0018 | ✓ |
| MIPI_PHY_D1_CONTROL | 0x20 | 0x00000300 | 0x00000300 | ✓ |
| MIPI_PHY_CL_CONTROL | 0x48 | 0x0F000004 | 0x0F000004 | ✓ |
| MIPI_CAMERA_CNTL | 0x24 | 0x0000E404 | 0x0000E404 | ✓ |

### Sensor Registers During Streaming

| Register | Address | Value | Meaning |
|----------|---------|-------|---------|
| OUTPUT_CONTROL | 0x3400 | 0x7A08 | MIPI enabled, LP clock |
| RESET_REGISTER | 0x301A | 0x120C | Streaming mode |
| SEQ_STATE | MCU var | 0x03 | Streaming |
| SEQ_CMD | MCU var | 0x00 | Ready (command accepted) |

### VFE CAMIF State

| Register | Value | Meaning |
|----------|-------|---------|
| CAMIF_STATUS | 0x80000000 → 0x00000000 | Halted → Started |
| FRAME_CFG | 0x01e00500 | 480 lines, 1280 bytes/line |
| CORE_CFG | 0x00000046 | UYVY pixel pattern |
| AXI_OUT_MODE | 0x00000200 | PIX mode |

---

## NEW: CSIPHY to VFE Data Path Issue

### The Core Problem

After calibration mode improvements, we can now see:
1. **CSIPHY receives valid frames** - FS (Frame Start) and FE (Frame End) short packets detected
2. **VFE never receives SOF** - CAMIF_STATUS stays halted at 0x80000000

This points to a **data routing issue** between CSIPHY output and VFE input.

### Data Path Architecture (MSM8660)

```
MT9M113 Sensor
     │
     ▼ (MIPI CSI-2)
CSIPHY1 @ 0x04900000
     │  ← FS/FE packets received ✓
     │
     │  ??? Gap in data path ???
     │
     ▼
VFE31 CAMIF @ 0x04500000
     │  ← Never sees SOF
```

### Clocks Verified Enabled

| Clock | Register | Status |
|-------|----------|--------|
| CSI1_VFE_CLK | VFE_CC_REG bit 10 | ENABLED ✓ |
| vfe_csi1_clk halt | DBG_BUS_VEC_B bit 8 | Running ✓ |
| csi_pix_clk | MISC_CC_REG | CSI1 selected ✓ |
| csi1_phy_clk | - | Enabled ✓ |

### Possible Causes

1. **Missing input mux configuration** - VFE may need explicit input source selection
2. **CSID configuration** - The "unified CSIPHY+CSID" may need additional setup
3. **Internal routing register** - There may be an undocumented routing register
4. **Timing dependency** - VFE may need to be "listening" before CSIPHY starts

### What webOS Does Differently

webOS camera stack uses a custom kernel driver that:
1. Configures CSIPHY via msm_camio_csi_config()
2. Has a "VFE configuration" phase before streaming
3. May configure routing that we're missing

---

## Previous Theories (Session 1)

### 1. Clock Phase/Frequency Mismatch (Partially Addressed)
The consistent 52% ECC rate suggests a systematic issue. If the receiver samples data at slightly wrong phase relative to the MIPI clock, every other bit boundary could be misread, causing consistent header corruption.

### 2. Missing PHY Calibration Step
webOS may perform additional calibration we're not doing. The bit 23 "calibration done" in CALIBRATION_CONTROL might not mean full calibration is complete. There could be additional steps needed.

### 3. MIPI Clock Recovery Issue
The CSIPHY recovers clock from the incoming MIPI data stream. If clock recovery is unstable or slightly off-frequency, it would cause consistent sampling errors on every packet.

### 4. D-PHY Timing Parameters
MT9M113 supposedly has fixed MIPI timing based on PLL configuration, but there might be a mismatch between sensor output timing and receiver expectations.

### 5. Hardware Signal Integrity
Physical layer issues (impedance mismatch, crosstalk, reflections) could cause consistent corruption patterns. However, webOS works on the same hardware, making this less likely.

---

## Questions for Analysis (Updated Session 2)

### NEW Priority Questions

1. **Why does VFE CAMIF never leave halted state?**
   - CSIPHY receives valid FS/FE packets
   - CSI1_VFE_CLK is enabled
   - CAMIF_CMD START is issued
   - But CAMIF_STATUS stays 0x80000000
   - **What triggers CAMIF to start accepting data?**

2. **Is there an internal routing/mux on MSM8660?**
   - The unified CSIPHY+CSID architecture may have internal routing
   - webOS may configure this routing somewhere we haven't found
   - Are there undocumented registers between CSIPHY and VFE?

3. **Does VFE need to be "listening" before CSIPHY starts?**
   - Current sequence: CSIPHY config → Sensor stream → VFE config
   - Should VFE be fully ready before sensor starts streaming?

4. **Is there a CSID-specific configuration for MSM8660?**
   - Our CSID driver is minimal (pass-through for unified PHY+CSID)
   - webOS may do additional CSID configuration

### Previous Questions (Session 1)

5. **Why was ECC error rate locked at exactly 51-52%?**
   - **PARTIALLY RESOLVED:** Calibration bypass + high settle_cnt reduces ECC significantly
   - Clean FS/FE packets now visible with optimized settings

6. **What is bit 11 (0x800) in the CSIPHY IRQ status?**
   - We see status=0x00000830 and 0x00000880 frequently
   - Bit 11 is undocumented - could indicate frame boundary

7. **Could IRQ_MASK difference affect data flow?**
   - webOS uses 0xFFF7F3FF
   - Our driver uses 0x000300F0
   - Less likely to be the issue since data does reach CSIPHY

---

## webOS Reference Values

From webOS kernel mt9m113.c and msm_io_8x60.c:

```c
// MT9M113 CSI parameters
mt9m113_csi_params.lane_cnt = 1;
mt9m113_csi_params.data_format = CSI_8BIT;  // = 0
mt9m113_csi_params.lane_assign = 0xe4;
mt9m113_csi_params.dpcm_scheme = 0;
mt9m113_csi_params.settle_cnt = 0x14;  // 20 decimal

// D0_CONTROL2 calculated value: 0x140F0018
// - settle_cnt = 0x14 << 24
// - HS_TERM_IMP = 0x0F << 16
// - LP_REC_EN = 1 << 4
// - ERR_SOT_HS_EN = 1 << 3
```

---

## Things We've Ruled Out

1. **Sensor not streaming:** CSIPHY receives SOT = sensor is sending MIPI packets ✓
2. **OUTPUT_CONTROL disabled:** Now correctly shows 0x7A08 after SEQ_CMD ✓
3. **Wrong CSI selected:** MISC_CC_REG shows CSI1 selected ✓
4. **Clocks not enabled:** All VFE clocks enabled including csi_pix and csi_rdi ✓
5. **VFE configuration wrong:** CAMIF registers match webOS values ✓
6. **CSIPHY PHY disabled:** D1_CONTROL = 0x300 (PHY enabled) ✓
7. **Legacy clock mode:** Tested - MISC_CC_REG=0x0 is BROKEN, modern mux required ✓
8. **Timing parameters:** Tested 75 combinations - all show ~52% ECC ✓
9. **Continuous vs LP clock:** Tested both - LP mode (0x7A08) has more activity ✓

---

## File Locations

- Sensor driver: `drivers/media/i2c/mt9m114.c`
- CSIPHY driver: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- VFE driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- webOS reference: `webos-linux-kernel-touchpad/drivers/media/video/msm/mt9m113.c`
- webOS CSIPHY: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_io_8x60.c`

---

## Diagnostic Commands

```bash
# On device via SSH (port 22):
/tmp/test-camera.sh pix     # Full pipeline test

# Check current parameters:
cat /sys/module/qcom_camss/parameters/settle_cnt_override
cat /sys/module/qcom_camss/parameters/hs_term_imp_override

# Set parameters for testing:
echo 0x14 > /sys/module/qcom_camss/parameters/settle_cnt_override
echo 0x0F > /sys/module/qcom_camss/parameters/hs_term_imp_override

# Check dmesg for errors:
dmesg | grep -E "VFE|CSIPHY|mt9m|camss|ECC|SOT" | tail -100

# Count ECC errors vs clean SOT:
dmesg | grep "CSIPHY" | grep -c "ECC"
dmesg | grep "CSIPHY" | grep "SOT" | grep -cv "ECC"
```

---

## Recent Commits

| Commit | Description |
|--------|-------------|
| 2323af7a4f6d | **media: qcom: camss: csiphy-8x60: Add calibration mode parameter** |
| c2ad99be2984 | docs: Consolidate camera debug documents |
| c3d1d461c88a | media: i2c: mt9m114: Match webOS MT9M113 streaming sequence order |
| af21bca6901f | media: i2c: mt9m114: Use 640x480 as default MT9M113 output format |
| 01f57363213e | media: i2c: mt9m114: Delay MT9M113 MIPI enable to fix VFE CAMIF timing |
| 8db7bdb7498d | media: i2c: mt9m114: Add MT9M113 context V4L2 control |
| 481829017061 | media: i2c: mt9m114: Fix MT9M113 SEQ_CAP_MODE for Context B |

## New Module Parameters (Session 2)

### CSIPHY Calibration Mode
```bash
# Check current mode
cat /sys/module/qcom_camss/parameters/calibration_mode

# Mode 0: Normal (poll for bit 23, apply impedance after)
# Mode 1: Bypass (no calibration, direct impedance write) <- RECOMMENDED
# Mode 2: webOS-style (write cal+impedance without polling)
echo 1 > /sys/module/qcom_camss/parameters/calibration_mode
```

### Recommended Test Configuration
```bash
# Apply optimal settings found in Session 2
echo 1 > /sys/module/qcom_camss/parameters/calibration_mode
echo 0x60 > /sys/module/qcom_camss/parameters/settle_cnt_override
echo 0x60 > /sys/module/qcom_camss/parameters/vfe31_axi_output_mode
```
