# HP TouchPad Camera Debugging Status

**Last Updated:** 2026-03-18
**Kernel:** Linux 6.18-tenderloin
**SoC:** Qualcomm APQ8060 (MSM8660 variant)
**Sensor:** MT9M113 front camera (MIPI CSI-2, 1 lane, UYVY 8-bit)

---

## Current Issue

**Problem:** VFE (Video Front End) never receives pixel data from the MIPI CSI camera chain, resulting in "VFE sof timeout" and "VFE reg update timeout" errors.

**Symptoms:**
- Capture attempts result in 0-byte output files
- VFE timeout errors in dmesg
- CSIPHY receives MIPI data (confirmed by IRQ activity)
- But VFE CAMIF never receives Start-of-Frame or pixel data

---

## Hardware Data Path

```
MT9M113 Sensor (I2C: 0x3c)
     │
     ▼ (MIPI CSI-2, 1 lane, 640x480 UYVY)
CSIPHY1 (0x04900000) - Integrated CSID on MSM8660
     │
     ├──→ CSI1_VFE_CLK bridge (MMCC VFE_CC_REG BIT 10)
     │
     ▼
VFE31 CAMIF (0x04500000 + 0x1E0-0x204)
     │
     ├──→ AXI_OUT_MODE at 0x040:
     │    - 0x60: CAMIF_TO_AXI (raw bypass mode)
     │    - 0x200: OUTPUT_2 (VFE ISP processing)
     │
     ▼
AXI Write Master → DDR Memory
```

---

## Components Verified MATCHING webOS

### 1. CSIPHY Configuration ✓ VERIFIED

| Register | Offset | Our Value | webOS Value | Status |
|----------|--------|-----------|-------------|--------|
| MIPI_PHY_CONTROL | 0x00 | 0x04 | 0x04 | ✓ MATCH |
| MIPI_PROTOCOL_CONTROL | 0x04 | 0x00260000 | 0x00260000 | ✓ MATCH |
| MIPI_CALIBRATION_CONTROL | 0x18 | 0x00E00080 | 0x00E00080 | ✓ MATCH |
| MIPI_PHY_D0_CONTROL | 0x34 | 0x00000000 | 0x00000000 | ✓ MATCH |
| MIPI_PHY_D0_CONTROL2 | 0x38 | 0x140F0018 | 0x140F0018 | ✓ MATCH |
| MIPI_PHY_D1_CONTROL | 0x20 | 0x00000300 | 0x00000300 | ✓ MATCH |
| MIPI_PHY_CL_CONTROL | 0x48 | 0x0F000004 | 0x0F000004 | ✓ MATCH |
| MIPI_CAMERA_CNTL | 0x24 | 0x0000E404 | 0x0000E404 | ✓ MATCH |
| MIPI_INTERRUPT_MASK | 0x0C | 0xFFF7F3FF | 0xFFF7F3FF | ✓ MATCH |

**D0_CONTROL2 breakdown (0x140F0018):**
- settle_cnt = 0x14 (20 cycles) at bits [31:24]
- HS_TERM_IMP = 0x0F at bits [23:16]
- LP_REC_EN = 1 at bit 4
- ERR_SOT_HS_EN = 1 at bit 3

**PROTOCOL_CONTROL breakdown (0x00260000):**
- LONG_PACKET_HEADER_CAPTURE = 1 at bit 21
- DECODE_ID = 1 at bit 18
- ECC_EN = 1 at bit 17

**CAMERA_CNTL breakdown (0x0000E404):**
- lane_assign = 0xE4 at bits [15:8]
- lane count field = 0x04 (1 lane)

### 2. CSI Clocks ✓ VERIFIED

| Clock | Register | Bit | Status |
|-------|----------|-----|--------|
| CSI0_CLK | CSI_CC_REG (0x04000040) | BIT(0) | ENABLED |
| CSI1_CLK | CSI_CC_REG (0x04000040) | BIT(7) | ENABLED |
| CSI0_PHY_CLK | CSI_CC_REG (0x04000040) | BIT(8) | ENABLED |
| CSI1_PHY_CLK | CSI_CC_REG (0x04000040) | BIT(9) | ENABLED |
| CSI0_VFE_CLK | VFE_CC_REG (0x04000104) | BIT(12) | ENABLED |
| CSI1_VFE_CLK | VFE_CC_REG (0x04000104) | BIT(10) | ENABLED |

### 3. CSI Mux Configuration ✓ VERIFIED

| Setting | Register | Bits | Value | Meaning |
|---------|----------|------|-------|---------|
| csi_pix_sel | MISC_CC_REG (0x04000058) | BIT(25) | 1 | CSI1 selected |
| csi_pix_en | MISC_CC_REG (0x04000058) | BIT(26) | 1 | Enabled |
| csi_rdi_sel | MISC_CC_REG (0x04000058) | BIT(12) | 1 | CSI1 selected |
| csi_rdi_en | MISC_CC_REG (0x04000058) | BIT(13) | 1 | Enabled |

### 4. VFE31 CAMIF Configuration ✓ VERIFIED

| Register | Offset | Value | Purpose |
|----------|--------|-------|---------|
| AXI_OUT_MODE | 0x040 | 0x60 | CAMIF_TO_AXI raw mode |
| EFS_CFG | 0x1E4 | 0x00 | APS mode (not EFS sync) |
| FRAME_CFG | 0x1E8 | width\|height<<16 | Frame dimensions |
| WINDOW_WIDTH | 0x1EC | lastPixel\|firstPixel<<16 | Active window |
| WINDOW_HEIGHT | 0x1F0 | lastLine\|firstLine<<16 | Active window |
| CAMIF_CMD | 0x1E0 | 0x01 | Start capture |

**Note:** VFE31 does NOT have camif2vfeEnable/camif2busEnable bits like VFE8x. Data routing is via AXI_OUT_MODE only.

### 5. MT9M113 Sensor Configuration ✓ VERIFIED

| Register | Value | Purpose |
|----------|-------|---------|
| OUTPUT_CONTROL (0x3400) | 0x7A08 | MIPI output enable, LP mode |
| RESET_REGISTER (0x301A) | 0x120C | Streaming mode |
| CUSTOM_SHORT_PKT (0x3404) | 0x0080 | Short packet config |
| SEQ_STATE (0xA104) | 0x03 | Streaming state |
| CHIP_ID (0x0000) | 0x2480 | MT9M113 detected |

Full 522-entry initialization table applied from webOS.

### 6. CSIPHY IRQ Activity ✓ CONFIRMED RECEIVING DATA

**IRQ status patterns observed during capture:**
- 0x00000010 (BIT 4): SOT_SYNC - Start of Transmission sync
- 0x00000020 (BIT 5): ECC_ERROR - ECC decode activity
- 0x00000830 (BIT 4+5+11): SOT + ECC + DATA_CMM_ERR (de-featured)

**Key observation:** sof_count increments at ~100Hz rate (every ~10ms), suggesting line-level detection rather than frame-level (30fps would be ~33ms per frame).

---

## Known Issues / Differences from webOS

### 1. Software SOF Implementation

webOS doesn't use CSIPHY Frame Start short packets. Frame sync comes from VFE CAMIF internal logic. Our software SOF detection (timing-based from SOT gaps) works but VFE still doesn't receive actual pixel data.

### 2. IRQ Bit 11 (DATA_CMM_ERR)

This bit fires frequently. webOS masks it as "de-featured" (IRQ mask 0xFFF7F3FF masks bits 10, 11, 19). Our driver also masks it but it still appears in status reads.

### 3. REG_UPDATE Timeout

VFE REG_UPDATE polls time out because no SOF arrives to latch shadow registers. This is a symptom, not the cause.

### 4. Timer Clock Rate = 0

The csiphytimer_src RCG fails to enable (rate=0). We use hardcoded settle_cnt=0x14 which matches webOS default.

---

## Capture Paths Tested

### PIX Path (video3)
- Pipeline: CSIPHY1 → CSID1 → VFE (with ISP processing)
- AXI_OUT_MODE: 0x200 (OUTPUT_2)
- Result: 0-byte capture, VFE timeout

### RDI Path (video0)
- Pipeline: CSIPHY1 → CSID1 → VFE (raw bypass)
- AXI_OUT_MODE: 0x60 (CAMIF_TO_AXI)
- Result: 0-byte capture, VFE timeout

---

## What We've Eliminated

1. **Clock issues** - All CSI and VFE clocks verified enabled
2. **Mux issues** - CSI mux correctly selects CSI1 for both pix and rdi
3. **CSIPHY configuration** - Matches webOS exactly
4. **Sensor configuration** - 522-entry table applied, sensor streaming
5. **VFE CAMIF config** - Register layout corrected for VFE31 (not VFE8x)
6. **CSID configuration** - MSM8660 CSID is pass-through, no CID config needed
7. **AXI_OUT_MODE** - Both raw (0x60) and preview (0x200) tested

---

## Current Hypothesis

**The decoded pixel data from CSIPHY is not reaching VFE CAMIF.**

Evidence:
- CSIPHY sees SOT (line sync) and ECC (packet decode) interrupts
- This proves MIPI physical layer is working
- This proves MIPI protocol decode is working
- But VFE CAMIF never sees CAMIF_SOF or pixel data
- The bridge between CSIPHY decode output and VFE CAMIF input is broken

Possible causes:
1. **Missing clock or gate** - Some internal bridge clock not enabled
2. **Hardware bug** - VFE31 input logic issue
3. **Timing issue** - VFE CAMIF must be started before CSIPHY sees data?
4. **Format mismatch** - VFE expects different data format than CSIPHY outputs

---

## Next Investigation Steps

1. **Add more debug to VFE CAMIF input**
   - Dump CAMIF_STATUS register during capture
   - Check if CAMIF sees any pixel count or line count

2. **Try different startup order**
   - Start VFE CAMIF before CSIPHY
   - Start sensor streaming after VFE ready

3. **Check VFE internal pixel counters**
   - VFE may have debug registers showing pixel flow

4. **Compare VFE reset sequence**
   - Ensure VFE is properly reset before each capture

5. **Verify VFE AXI write master**
   - Check if WM0 is properly configured for output

---

## Files Modified from Upstream

| File | Purpose |
|------|---------|
| drivers/media/platform/qcom/camss/camss-csiphy-8x60.c | MSM8660 CSIPHY driver |
| drivers/media/platform/qcom/camss/camss-csid-8x60.c | MSM8660 CSID driver (pass-through) |
| drivers/media/platform/qcom/camss/camss-vfe.c | VFE31-specific CAMIF code |
| drivers/media/platform/qcom/camss/camss-vfe-gen1.c | VFE gen1 common code |
| drivers/media/i2c/mt9m114.c | MT9M113 sensor support |

---

## Reference Documentation

- **webOS kernel:** /home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/
  - drivers/media/video/msm/msm_io_8x60.c - CSIPHY config
  - drivers/media/video/msm/msm_vfe31.c - VFE31 driver
  - drivers/media/video/msm/mt9m113.c - Sensor driver
  - drivers/media/video/msm/mt9m113_reg.c - Register table

- **Ghidra decompiled:** /home/herrie/webos/touchpad-kernel/doctor305/ghidra_decompiled/
  - libqcameralib_decompiled.c - HAL functions

---

## dmesg Excerpt (Typical Capture Attempt)

```
MT9M113: Successfully detected MT9M113 (chip ID 0x2480)
MT9M113: Applying 522-entry initialization table
CSIPHY1: lanes_enable complete
CSIPHY1: IRQ #1 status=0x00000010 [SOT ] sof_count=0
CSIPHY1: IRQ #2 status=0x00000030 [SOT ECC ] sof_count=1
VFE: MMCC VFE_CC_REG: 0x80ff14a5 (CSI0_VFE=ON, CSI1_VFE=ON)
VFE: MMCC MISC_CC_REG: 0x06003440 (csi_pix_sel=CSI1, csi_rdi_sel=CSI1)
VFE: AXI_OUT_MODE set to 0x00000060 (raw mode)
VFE: CAMIF_STATUS before REG_UPDATE: 0x00000000
VFE: Writing CAMIF_CMD=1 to start
CSIPHY1: IRQ #100 status=0x00000830 [SOT ECC ] sof_count=95
...
VFE: sof timeout
VFE: reg update timeout
```

---

## Additional Verification (2026-03-18)

### PROTOCOL_CONTROL Calculation Verified

Value: 0x00260000
- LONG_PACKET_HEADER_CAPTURE (bit 21): 0x200000 ✓
- DECODE_ID (bit 18): 0x40000 ✓
- ECC_EN (bit 17): 0x20000 ✓
- DATA_FORMAT (bits 19-20): 0 (CSI_8BIT) ✓

### VFE Startup Sequence Verified

webOS vfe31_start_common() does:
1. IRQ_MASK_0 = 0x00EFE021 ✓
2. IRQ_MASK_1 = 0x00400000 ✓
3. REG_UPDATE_CMD = 1 (with memory barrier) ✓
4. CAMIF_COMMAND = 1 ✓

Our driver uses the same values.

---

## Questions for Further Investigation

### For AI Analysis (Gemini, Claude, etc.)

1. **MSM8660 CSI-VFE Bridge Architecture:**
   - Is there a CSID CID (Context ID) configuration needed even on MSM8660?
   - Does the integrated CSIPHY+CSID on MSM8660 have a different output interface than later chips?

2. **VFE31 CAMIF Input Path:**
   - Does VFE31 have a "CAMIF input select" register we haven't found?
   - Is there a pixel FIFO between CSID output and VFE CAMIF input that needs enabling?

3. **Timing Requirements:**
   - Must VFE CAMIF be started BEFORE CSIPHY enables lanes?
   - Is there a minimum time between CSIPHY config and VFE CAMIF start?

4. **Data Format Mapping:**
   - MIPI CSI-2 data type 0x1E (YUV422-8) - is this correctly mapped to VFE pixel pattern?
   - Does VFE31 auto-detect data format or require explicit configuration?

5. **Undocumented Registers:**
   - Are there any MSM8660-specific "chicken bits" or hidden enables in TCSR?
   - Does MMSS have any additional routing registers beyond MISC_CC_REG?

### Specific Code Analysis Questions

1. **webOS msm_vfe31.c line 787-788:**
   ```c
   msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[V31_AXI_OUT_CFG].offset,
       ao, vfe31_cmd[V31_AXI_OUT_CFG].length);
   ```
   - What values does `ao` contain for raw capture mode?
   - Is there something in the 188-byte AXI config that enables pixel input?

2. **webOS libqcameralib vfe_raw_snapshot_config():**
   - What's the complete VFE configuration sequence before streaming?
   - Are there userspace ioctls we're not emulating?

3. **CSIPHY IRQ Bit 11 (DATA_CMM_ERR):**
   - webOS masks this as "de-featured" - what does it actually indicate?
   - Could this be signaling a decode failure rather than just a spurious interrupt?

---

## Diagnostic Experiments to Try

1. **Continuous CAMIF_STATUS polling:**
   Add kernel thread to poll VFE CAMIF_STATUS every 10ms and log pixel/line count changes

2. **VFE test generator (if available):**
   Enable VFE internal test pattern generator to verify VFE pipeline independently

3. **Different CSIPHY/VFE startup orders:**
   - Try: VFE start -> sensor stream -> CSIPHY enable
   - Try: CSIPHY enable -> VFE start -> sensor stream

4. **Loopback test:**
   Configure AXI read master to feed VFE input, verify VFE processes data

5. **Raw register dump during webOS capture:**
   Boot webOS on TouchPad, capture frame, dump all MMSS registers
