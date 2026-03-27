# MT9M113 Front Camera Debug Status - HP TouchPad

## Hardware Overview

- **Device:** HP TouchPad tablet
- **SoC:** Qualcomm APQ8060 (MSM8660 family), dual-core ARMv7 Scorpion @ 1.5GHz
- **Front Camera Sensor:** Aptina MT9M113 (1.3MP, chip ID 0x2480)
- **Interface:** MIPI CSI-2, 1 data lane
- **Connection:** Sensor → CSIPHY1 → CSID1 → VFE0 PIX → /dev/video3
- **Kernel:** Linux 6.18 mainline with camss driver

## Current Status

**The camera pipeline partially works but fails to capture frames due to persistent MIPI ECC errors.**

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

4. **VFE Configuration**
   - CAMIF configured correctly for 640x480 or 1280x1024
   - FRAME_CFG shows correct pixel/line counts
   - CAMIF_SOF (Start of Frame) interrupts received
   - VFE hardware version 0x00030217 detected

### What Fails

1. **Persistent ECC Errors**
   - ~51-52% of all MIPI packets have ECC errors
   - This rate is consistent across ALL tested parameter combinations
   - Pattern: SOT detected → ECC error on packet header

2. **VFE Timeout**
   - VFE receives CAMIF_SOF but times out waiting for REG_UPDATE
   - CAMIF_ERROR (status1 bit 0) fires due to corrupted data
   - ping_pong register never toggles (no complete frames)

## Detailed Test Results

### settle_cnt / hs_term_imp Parameter Sweep

Tested 75 combinations of:
- settle_cnt: 0x04 to 0x20
- hs_term_imp: 0x00, 0x03, 0x07, 0x0B, 0x0F

**Results (best combinations by activity):**

| settle_cnt | hs_term_imp | Total IRQs | ECC Errors | ECC % |
|------------|-------------|------------|------------|-------|
| 0x18       | 0x0F        | 3073       | 1601       | 52%   |
| 0x06       | 0x03        | 2284       | 1204       | 52%   |
| 0x14       | 0x07        | 2169       | 1128       | 52%   |
| 0x14       | 0x0F        | 2083       | 1079       | 51%   |
| 0x1A       | 0x0B        | 2045       | 1071       | 52%   |

**Key observation:** ECC error rate is remarkably consistent at 51-52% regardless of timing parameters. This suggests a systematic issue, not a timing calibration problem.

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

The alternating SOT/ECC pattern suggests the PHY successfully syncs to MIPI SOT bytes but then misreads the subsequent packet header data.

### VFE Status During Capture Attempt

```
VFE IRQ: status0=0x00000001 status1=0x00000000  <- CAMIF_SOF received
VFE IRQ: status0=0x0000001d status1=0x00000001  <- Multiple errors + CAMIF_ERROR
VFE reg update timeout
CAMIF: status=0x81e00500 (halted)
```

## Code Changes Made

### 1. Streaming Sequence (mt9m114.c)

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

### 2. CSIPHY Configuration (camss-csiphy-8x60.c)

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

// CALIBRATION_CONTROL
writel(0x00700000, MIPI_CALIBRATION_CONTROL);
// Poll for bit 23 (calibration done)

// CAMERA_CNTL: lane_assign | 1 lane
writel(0x0000e404, MIPI_CAMERA_CNTL);
```

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

## Register Dumps

### Sensor State During Streaming
```
OUTPUT_CONTROL (0x3400) = 0x7A08  (MIPI enabled, LP clock)
RESET_REGISTER (0x301A) = 0x120C  (streaming)
SEQ_STATE = 0x03 (streaming)
SEQ_CMD = 0x00 (ready/accepted)
MODE_A = 640x480, frame_len=814, line_len=1228
```

### CSIPHY State
```
PHY_CONTROL (0x00) = 0x00000004  (SOT_ECC_EN)
PROTOCOL_CONTROL (0x04) = 0x00260000
D0_CONTROL2 (0x38) = varies with test
CL_CONTROL (0x48) = 0x0F000004
CALIBRATION_CONTROL (0x18) = 0x00F00000 (bit 23 set = cal done)
CAMERA_CNTL (0x24) = 0x0000E404 (1 lane)
IRQ_MASK (0x0C) = 0x000300F0 (SOT/ECC/FS/FE enabled)
```

### VFE CAMIF State
```
CAMIF_STATUS = 0x80000000 (halted initially)
            -> 0x00000000 (after CAMIF_START)
            -> 0x81e00500 (halted after timeout)
FRAME_CFG = 0x01e00500 (480 lines, 1280 bytes/line for 640x480 UYVY)
CORE_CFG = 0x00000046 (UYVY pixel pattern)
```

## Theories / Hypotheses

### 1. Clock Phase/Frequency Mismatch
The consistent 52% ECC rate suggests a systematic issue. If the receiver samples data at slightly wrong phase, every other bit boundary could be misread, causing consistent header corruption.

### 2. Missing PHY Calibration Step
webOS may perform additional calibration we're not doing. The bit 23 "calibration done" might not mean full calibration is complete.

### 3. MIPI Clock Mode Issue
We use LP (Low Power) clock mode (0x7A08). Continuous clock mode (0x7A0C) was tested but showed even less activity. The sensor might require specific clock handling.

### 4. D-PHY Timing Parameters
The MIPI D-PHY has many timing parameters (T_HS_ZERO, T_HS_EXIT, T_CLK_POST, etc.). MT9M113 supposedly has fixed MIPI timing based on PLL, but there might be a mismatch.

### 5. Hardware Signal Integrity
Physical layer issues (impedance mismatch, crosstalk, reflections) could cause consistent corruption patterns. However, this seems less likely given webOS works on same hardware.

## Questions for Analysis

1. **Why is the ECC error rate so consistent at 51-52%?** This pattern suggests a systematic issue rather than random noise. What could cause exactly half the packets to have header errors?

2. **What does the alternating SOT/ECC pattern indicate?** We see clean SOT followed by ECC errors in a regular pattern. Is the PHY losing sync after each SOT?

3. **Is there additional CSIPHY initialization in webOS we're missing?** We've matched msm_camio_csi_config() but there might be earlier initialization in msm_camio_enable() or elsewhere.

4. **Could the IRQ_MASK difference matter?** webOS uses 0xFFF7F3FF vs our 0x000300F0. We enable fewer interrupts, but this shouldn't affect data flow.

5. **Is there a required delay or sequence we're missing between CSIPHY config and sensor streaming?** The webOS code has specific delay sequences that might be critical.

6. **What is bit 11 (0x800) in the CSIPHY IRQ status?** We see status=0x00000830 frequently. Bit 11 is undocumented - could it indicate a specific error condition?

## File Locations

- Sensor driver: `drivers/media/i2c/mt9m114.c`
- CSIPHY driver: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- VFE driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- webOS reference: `webos-linux-kernel-touchpad/drivers/media/video/msm/mt9m113.c`
- webOS CSIPHY: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_io_8x60.c`

## Environment

- Kernel: Linux 6.18.0 (mainline)
- Branch: tenderloin/6.18/upstream-patches
- Test device: HP TouchPad (Topaz WiFi variant)
- Connection: USB gadget network at 172.16.42.2
