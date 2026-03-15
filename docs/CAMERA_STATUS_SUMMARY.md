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

### 1. VFE31 Not Receiving Data from CSIPHY

**Symptom:** Camera streaming times out waiting for VFE SOF (Start of Frame) interrupt.

**Analysis:**
- CSIPHY1 appears to be configured correctly
- MT9M113 sensor initialization completes successfully
- SEQ_STATE reaches 0x03 (preview mode active)
- But VFE31 CAMIF never receives frame data

**Suspected Cause:** CSI-to-VFE clock bridge

The pixel data path from CSIPHY to VFE requires specific bridge clocks:

| Clock | Register | Bit | Purpose |
|-------|----------|-----|---------|
| vfe_csi0_clk | VFE_CC_REG (0x04000104) | BIT(12) | CSI0 → VFE data path |
| vfe_csi1_clk | VFE_CC_REG (0x04000104) | BIT(10) | CSI1 → VFE data path |
| csi_pix_clk | 0x04000058 | BIT(26) | Pixel interface |

**MT9M113 uses CSI1**, so `vfe_csi1_clk` (BIT 10) must be enabled.

**Debug function added:** `vfe31_debug_dump_external_regs()` checks and logs CSI1_VFE_CLK status.

### 2. Potential MIPI Timing Issues

The MT9M113 MIPI timing registers (0xC988-0xC992) are defined in the driver but NOT written during init. The webOS MT9M113 driver also doesn't write them (they're commented out), so this may be intentional - but worth investigating if MIPI errors occur.

### 3. Software SOF Workaround

The VFE31 driver has a software SOF trigger mechanism as a workaround for sensors that don't generate proper hardware frame sync. This may be needed for MT9M113 but hasn't been fully tested.

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
1f45115b7ffe media: i2c: mt9m114: Add MT9M113 preview/snapshot AE tables and mode support
38545b2fab58 media: qcom: camss: Fix hardcoded bpp in MSM8660 set_power path
d341a03f989a media: i2c: mt9m114: Add sequencer refresh when MT9M113 not in preview mode
ffbc7313adb6 media: i2c: mt9m114: Fix MT9M113 streaming by always writing RESET_REGISTER
```

---

## Summary

The MT9M113 sensor driver is **complete and verified** against the webOS legacy kernel. The streaming sequence, register values, and AE table configuration all match exactly.

The issue appears to be in the **CAMSS/VFE31 data path**, specifically:
- The CSI1→VFE clock bridge may not be enabled
- Or there's a timing/sequencing issue between CSIPHY and VFE CAMIF

Next steps should focus on:
1. Running `debug` mode to capture clock enable status
2. Verifying CSIPHY1 is receiving MIPI data (SOF count > 0)
3. Checking VFE_CC_REG for CSI1_VFE_CLK enable bit
4. Testing with sensor test pattern to isolate data path issues
