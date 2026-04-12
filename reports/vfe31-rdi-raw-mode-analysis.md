# VFE31 RDI/Raw Mode Analysis - HP TouchPad (APQ8060)

## Executive Summary

We are attempting to implement RAW Bayer capture on the HP TouchPad (Qualcomm APQ8060 SoC) using the VFE31 (Video Front End) camera ISP. YUV capture via PIX/VIDEO modes works correctly, but RAW capture via RDI (Raw Dump Interface) mode fails completely with no data captured.

## Hardware Configuration

- **SoC**: Qualcomm APQ8060 (dual-core ARMv7 Scorpion @ 1.5GHz)
- **Camera ISP**: VFE31 (Video Front End version 3.1)
- **Camera Sensor**: Aptina MT9M114/MT9M113 (1.26MP, supports YUV422 and RAW Bayer output)
- **MIPI Interface**: 1-lane CSI-2
- **CSIPHY**: MSM8660 integrated CSIPHY (no separate ISPIF on this SoC)
- **CSID**: Pass-through on MSM8660 (integrated with CSIPHY, no VC filtering)

## Working Mode: PIX (YUV Capture)

### Configuration
```
Sensor Output:      UYVY (YUV422, 16 bits/pixel)
MIPI Data Type:     0x1E (YUV422 8-bit)
VFE AXI_OUT_MODE:   0x01 (OUTPUT_1_AND_3)
VFE XBAR_CFG1:      0x1A1B (routes DEMUX outputs to WMs)
VFE MODULE_CFG:     0x01C00C0C (DEMUX + processing enabled)
VFE CORE_CFG:       0x46 (UYVY pixel pattern + input mux enable)
Write Masters:      WM0 (Y plane), WM4 (CbCr plane)
IRQ:                COMPOSITE_DONE_0 (group 0 completion)
```

### Data Flow
```
Sensor -> CSIPHY1 -> CSID1 -> VFE CAMIF -> DEMUX -> Scaler/Crop -> WM0/WM4 -> Memory
```

### Result
**PASS** - 3,145,728 bytes captured (3 frames × 640×480 NV12)

### CAMIF Status During Capture
- Lines counted: 480
- Pixels counted: 1280 (width × 2 bytes for UYVY)
- Status bit 31: CAMIF_ACTIVE set

---

## Failing Mode: RDI (RAW Capture)

### Configuration
```
Sensor Output:      RAW8 Bayer GRBG (8 bits/pixel)
MIPI Data Type:     0x2A (RAW8)
VFE AXI_OUT_MODE:   0x60 (CAMIF_TO_AXI_VIA_OUTPUT_2)
VFE XBAR_CFG1:      0x00 (no XBAR routing - raw bypass)
VFE MODULE_CFG:     0x00 (all processing disabled)
VFE CORE_CFG:       0x40 (no pixel pattern, input mux enable only)
Write Masters:      WM0 only (single plane raw)
IRQ:                COMPOSITE_DONE_1 (group 1 completion, WM0 bit 8)
```

### Expected Data Flow
```
Sensor -> CSIPHY1 -> CSID1 -> VFE CAMIF -> WM0 -> Memory (bypassing DEMUX)
```

### Result
**FAIL** - 0 bytes captured, continuous VIOLATION IRQs

### CAMIF Status During Capture
- Lines counted: **0**
- Pixels counted: **0**
- Status bit 31: CAMIF_ACTIVE set (0x80000000)

---

## Detailed Symptoms

### 1. VIOLATION IRQs
```
[  839.394142] VFE31 VIOLATION IRQ (status=0, spurious)
[  839.460986] VFE31 VIOLATION IRQ (status=0, spurious)
[  839.527827] VFE31 VIOLATION IRQ (status=0, spurious)
... (repeats at ~66ms intervals = 15 fps frame rate)
```

- VIOLATION fires continuously at frame rate
- VIOLATION_STATUS register at 0x048 always reads 0
- Note: Register 0x048 may not exist on VFE31 (not defined in webOS headers)

### 2. No Frame Completion
- COMPOSITE_DONE_1 IRQ never fires
- No ping/pong buffer switching occurs
- WM0 buffer addresses remain unchanged

### 3. CAMIF Not Counting
Despite sensor streaming RAW8 data:
```
CAMIF_STATUS = 0x80000000
  - Bit 31 (CAMIF_ACTIVE): SET
  - Bits 29:16 (received_lines): 0
  - Bits 13:0 (received_pixels): 0
```

---

## Register Comparison

### VFE Core Registers

| Register | PIX Mode | RDI Mode | Notes |
|----------|----------|----------|-------|
| MODULE_CFG (0x010) | 0x01C00C0C | 0x00000000 | RDI disables all modules |
| CORE_CFG (0x014) | 0x00000046 | 0x00000040 | RDI has no pixel pattern |
| AXI_OUT_MODE (0x040) | 0x00000001 | 0x00000060 | Different output paths |
| XBAR_CFG1 (0x044) | 0x00001A1B | 0x00000000 | RDI bypasses XBAR |

### CAMIF Registers

| Register | PIX Mode | RDI Mode | Notes |
|----------|----------|----------|-------|
| EFS_CFG (0x1E4) | 0x40 | 0x40 | Same (webOS default) |
| FRAME_CFG (0x1E8) | 0x00000000 | 0x01E00280 | RDI sets frame dims |
| WINDOW_WIDTH (0x1EC) | 0x01E00500 | 0x01E00280 | Different widths |
| WINDOW_HEIGHT (0x1F0) | 0x000004FF | 0x0000027F | Different heights |
| SUBSAMPLE_0 (0x1F4) | 0x000001DF | 0x000001DF | Same (479) |

### IRQ Configuration

| Register | PIX Mode | RDI Mode |
|----------|----------|----------|
| IRQ_MASK_0 | 0x00EFE021 | 0x00EFE021 |
| IRQ_MASK_1 | 0x00400000 | 0x00400000 |
| IRQ_COMPOSITE_MASK | 0x00000011 | 0x00000100 |

---

## WebOS Reference Code Analysis

### Raw Snapshot Mode in webOS (msm_vfe31.c)

```c
case CAMIF_TO_AXI_VIA_OUTPUT_2: {  /* use wm0 only */
    if (ad->bufnum2 < 1)
        return -EINVAL;
    CDBG("config axi for raw snapshot.\n");
    *p = 0x60;    /* raw snapshot with wm0 */
    vfe31_ctrl->outpath.out1.ch0 = 0; /* raw */
    regp1 = &(ad->region[ad->bufnum1]);
    vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_S;
    p1 = ao + 6;    /* wm0 for y  */
    *p1 = (regp1->paddr + regp1->info.y_off);
}
```

Key observations:
1. Only configures ping address, not pong (single-shot design)
2. Sets `VFE31_OUTPUT_MODE_S` flag (snapshot mode)
3. Uses `operation_mode = 0` for raw snapshot (vs 1 for normal snapshot)

### IRQ Composite Mask for Raw Mode

```c
/* this is raw snapshot mode. */
CDBG("config the comp imask for raw snapshot mode. \n");
if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
    irq_comp_mask |= (0x1 << (vfe31_ctrl->outpath.out1.ch0 + 8));
    // Results in: 0x1 << (0 + 8) = 0x100 (WM0 in group 1)
}
```

### Critical Finding: webOS Never Used Raw Mode

From conversation with user:
> "webOS didn't have raw mode" - webOS never used the CAMIF_TO_AXI_VIA_OUTPUT_2 (0x60) path in production.

The code existed but was never tested/validated on this hardware.

---

## Hypotheses

### Hypothesis 1: VFE31 Raw Bypass Not Functional on APQ8060
The VFE31 silicon on APQ8060 may not properly implement the CAMIF_TO_AXI raw bypass path. Evidence:
- CAMIF receives no data despite correct CSIPHY/CSID configuration
- Same sensor/CSIPHY/CSID works perfectly in PIX mode
- webOS never validated this code path

### Hypothesis 2: Missing Input Configuration
There may be additional registers that need to be configured for the VFE to accept RAW data format at its input:
- Perhaps the input expects specific embedded sync codes
- Perhaps there's a data type or format selection register we're missing
- The CAMIF might need different configuration for RAW vs YUV input

### Hypothesis 3: Clock/Timing Issue
RAW8 has half the data rate of UYVY (8 vs 16 bits/pixel):
- Different MIPI lane timing
- Perhaps VFE input clocks need different configuration
- Possibly a synchronization issue at the CAMIF input

### Hypothesis 4: VIOLATION Indicates Invalid Data Path
The continuous VIOLATION IRQs might indicate:
- The AXI_OUT_MODE 0x60 activates an invalid/unimplemented path
- Internal VFE state machine encounters undefined condition
- Hardware attempting to read from unconnected internal bus

---

## Sensor Configuration Verification

### RAW8 Mode Configuration
```
MT9M113: requested 640x480 format=0x3002 (SGRBG8_1X8)
MT9M113: MODE_OUTPUT_FORMAT_A current=0x0000, new=0x0100
MT9M113: OUTPUT_CONTROL=0xaa0c (RAW8 dt=0x2A, cont_clk)
MT9M113: SEQ_STATE=0x3 (streaming) SEQ_CMD=0x0 (ready)
```

### YUV Mode Configuration (working)
```
MT9M113: requested 640x480 format=0x200f (UYVY8_2X8)
MT9M113: MODE_OUTPUT_FORMAT_A current=0x0000, new=0x0000
MT9M113: OUTPUT_CONTROL=0x7a0c (YUV dt=0x1E, cont_clk)
```

The sensor IS being configured differently and should be outputting RAW8 data with correct MIPI data type.

---

## CSIPHY/CSID Configuration

Both modes use identical CSIPHY/CSID configuration:
```
CSIPHY1: lanes_enable: lanes=1 settle_cnt=0x14 link_freq=48000000
CSIPHY1: PROTOCOL_CONTROL=0x00260000 (ECC enabled)
CSID1: configure_stream enable=1 phy=1 lanes=1
```

MSM8660 CSID is a pass-through - no data type filtering or virtual channel configuration.

---

## Questions for Further Investigation

1. **Is there a VFE31 input format/data type register?**
   - Does the VFE need to know it's receiving RAW vs YUV data?
   - Is there a register that selects the input data width (8-bit vs 16-bit)?

2. **What exactly triggers VIOLATION on VFE31?**
   - The VIOLATION_STATUS register at 0x048 doesn't seem to exist on VFE31
   - What internal conditions cause the VIOLATION bit to be set?

3. **Is the CAMIF_TO_AXI path implemented in silicon?**
   - Was this feature ever validated by Qualcomm on MSM8660/APQ8060?
   - Are there known errata for this SoC's VFE implementation?

4. **Alternative approaches?**
   - Can we use the VFE's internal test pattern generator to verify the raw bypass path?
   - Is there a way to capture raw data through a different path (e.g., modified PIX mode)?
   - Could the DEMUX be configured as a pass-through for raw data?

5. **Debug suggestions?**
   - What additional registers should we dump during RDI mode?
   - Are there VFE debug/diagnostic registers that could indicate where data flow stops?
   - Is there a way to verify data is actually arriving at the VFE input?

---

## File References

- **VFE31 Driver**: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- **CSID Driver**: `drivers/media/platform/qcom/camss/camss-csid-8x60.c`
- **CSIPHY Driver**: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- **Sensor Driver**: `drivers/media/i2c/mt9m114.c`
- **webOS VFE31**: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c`
- **webOS VFE31 Header**: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.h`
- **webOS Register Dump**: `reports/webos-preview-mode-dump.txt`

---

## Test Commands

```bash
# Working PIX mode test
./test-camera.sh pix640

# Failing RDI mode test
./test-camera.sh rdi640

# Check dmesg for VFE messages
dmesg | grep -E 'VFE|CAMIF|VIOLATION|COMPOSITE'
```

---

## Conclusion

The VFE31 raw bypass mode (CAMIF_TO_AXI_VIA_OUTPUT_2) does not appear to function on the APQ8060 platform. The CAMIF reports receiving no data despite the sensor correctly outputting RAW8 and the same CSIPHY/CSID path working perfectly for YUV capture. This may be a hardware limitation, an untested/unvalidated feature in the original Qualcomm implementation, or a missing configuration that we haven't identified.

Any insights into VFE31 architecture, MSM8660 camera subsystem quirks, or alternative approaches to raw capture would be appreciated.
