# MT9M113 Front Camera Debug Status - HP TouchPad

**Last Updated:** 2026-03-28 (Session 3 - VFE Write Master Configuration)
**Kernel:** Linux 6.18-tenderloin
**SoC:** Qualcomm APQ8060 (MSM8660 variant)
**Sensor:** MT9M113 front camera (1.3MP, chip ID 0x2480)
**Interface:** MIPI CSI-2, 1 data lane, UYVY 8-bit

---

## Executive Summary

**CAMIF receives complete frames but VFE Write Master doesn't write to memory.**

### Session 3 Update (March 28, 2026)

**KEY DISCOVERY:**
The VFE31 IRQ STATUS_0 bits reveal the root cause:
- **Bit 2 (0x04): CAMIF_NO_SOT** - No Start of Transmission detected
- **Bit 3 (0x08): CAMIF_EOF_MISMATCH** - End of Frame mismatch

These are MIPI frame synchronization errors. The VFE cannot properly sync frames without MIPI Frame Start (FS) short packets.

**NEW FINDINGS:**
1. CAMIF_STATUS shows complete frames: 480 lines × 1280 pixels (640×480 UYVY)
2. CSIPHY `sof_count=0` - No MIPI Frame Start packets detected
3. BIT(22) in CSIPHY IRQ fires at exactly frame rate (~330ms = 3fps)
4. VFE IRQ pattern: `status0=0x0000001d` = SOF + NO_SOT + EOF_MISMATCH
5. ping_pong register never changes (no frames written)
6. Software SOF mechanism exists but requires proper frame start detection

**FIXES APPLIED THIS SESSION:**
1. IRQ_COMPOSITE_MASK (0x034) configuration for IMAGE_COMPOSITE_DONE IRQs
2. SUBSAMPLE_CFG_1 fixed from 0xFFFFFFFF to 0 (was causing 15/16 frame skip)
3. UB_CFG (Unified Buffer) configuration for Write Master
4. BIT(22) added as frame start trigger for software SOF
5. MT9M113 Context A dimensions configured dynamically

**Current State:**
- CSIPHY receives DATA IRQs (0x800) and BIT(22) at frame boundaries ✓
- CAMIF sees complete 640×480 frames (status=0x81e00500) ✓
- CAMIF_ERROR fires every frame due to missing MIPI FS packets ✗
- VFE Write Master configured but ping_pong never toggles ✗
- Captured files are 0 bytes ✗

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
│ - IRQ: DATA (0x800) ✓, BIT(22) at frame rate ✓  │
│ - IRQ: No FS (0x10000) or SOT (0x10) detected ✗ │
└─────────────────────────────────────────────────┘
     │
     │  ← Data routing is AUTOMATIC (no ISPIF)
     │  ← Clock bridge: vfe_csi1_clk + csi_pix/csi_rdi
     │
     ▼
┌─────────────────────────────────────────────────┐
│ VFE31 @ 0x04500000                              │
│ - HW Version: 0x00030217                        │
│ - CAMIF receives 480 lines × 1280 pixels ✓     │
│ - CAMIF_SOF (bit 0) fires ✓                     │
│ - CAMIF_NO_SOT (bit 2) fires ✗                  │
│ - CAMIF_EOF_MISMATCH (bit 3) fires ✗            │
│ - CAMIF_ERROR (status1 bit 0) fires ✗           │
│ - ping_pong never toggles (no DMA writes) ✗     │
└─────────────────────────────────────────────────┘
     │
     ▼
AXI → DDR Memory → /dev/video3 (V4L2) [0 bytes]
```

---

## Current Status

### What Works

1. **Sensor Detection & Initialization**
   - MT9M113 detected at I2C address 0x3c (chip ID 0x2480)
   - Full 522-entry initialization table applied
   - Context A configured dynamically for requested resolution
   - Sensor streams correctly (CAMIF sees complete frames)

2. **CSIPHY Configuration**
   - DATA lane IRQs (0x800) firing during frame reception
   - BIT(22) IRQs firing at frame boundaries (~3fps)
   - Module parameters available for tuning (settle_cnt, hs_term_imp)

3. **VFE CAMIF Configuration**
   - FRAME_CFG = 0x01e00500 (480 lines, 1280 bytes/line) ✓
   - WINDOW_WIDTH = 0x000004ff (last=1279, first=0) ✓
   - WINDOW_HEIGHT = 0x000001df (last=479, first=0) ✓
   - EFS_CFG = 0 (APS mode for MIPI) ✓
   - SUBSAMPLE_CFG_1 = 0 (no frame skip) ✓

4. **VFE Write Master Configuration**
   - WM0 ping/pong addresses valid (e.g., 0x72d00000/0x72d80000)
   - WM0 stride = 1280 (correct for 640×2 UYVY)
   - UB_CFG = 0x000003ff (offset=0, depth=1023) ✓
   - WR_CFG = 0x00000001 (enabled, but bit 1 doesn't stick)
   - IRQ_COMPOSITE_MASK = 0x00000001 (WM0 → COMP0) ✓

### What Fails

1. **MIPI Frame Sync Detection**
   - CSIPHY never detects FS (Frame Start) short packets
   - CSIPHY `sof_count=0` throughout streaming
   - VFE reports CAMIF_NO_SOT and CAMIF_EOF_MISMATCH

2. **VFE Frame Capture**
   - CAMIF_ERROR (status1 bit 0) fires every frame
   - IRQ_STATUS_0 = 0x1d (SOF + sync errors)
   - No IMAGE_COMPOSITE_DONE IRQs (bit 21 never fires)
   - ping_pong register stuck at 0x00010000
   - Captured files are 0 bytes

---

## Session 3 Detailed Findings

### VFE IRQ Status Decode

Observed pattern: `status0=0x0000001d status1=0x00000001`

**STATUS_0 bits (0x1d = 0b11101):**
| Bit | Value | IRQ Name | Meaning |
|-----|-------|----------|---------|
| 0 | 0x01 | CAMIF_SOF | Start of Frame detected ✓ |
| 2 | 0x04 | CAMIF_NO_SOT | **No MIPI Start of Transmission** |
| 3 | 0x08 | CAMIF_EOF_MISMATCH | **Frame end timing mismatch** |
| 4 | 0x10 | REG_UPDATE | Register update (intermittent) |

**STATUS_1 bits (0x01):**
| Bit | Value | IRQ Name | Meaning |
|-----|-------|----------|---------|
| 0 | 0x01 | CAMIF_ERROR | **Frame sync error** |

### CSIPHY IRQ Pattern

```
CSIPHY1: IRQ status=0x00000800 [DATA ] sof_count=0   <- During line data
CSIPHY1: IRQ status=0x00400000 []      sof_count=0   <- Frame boundary (BIT 22)
CSIPHY1: IRQ status=0x00000800 [DATA ] sof_count=0   <- During line data
CSIPHY1: IRQ status=0x00400000 []      sof_count=0   <- Frame boundary
... pattern repeats at ~3fps ...
```

**Key observation:** BIT(22) fires exactly at frame rate but is undocumented. We've added it as a frame start trigger for software SOF generation.

### CAMIF Status Analysis

At stream stop: `CAMIF_STATUS=0x81e00500`
- Bit 31 (0x80000000): Halted
- Bits 28:16 (0x1e0): 480 = lines received ✓
- Bits 11:0 (0x500): 1280 = pixels per line ✓

**This confirms CAMIF receives complete 640×480 UYVY frames!**

The data path from sensor to CAMIF works. The issue is:
1. Missing MIPI FS packets prevent proper frame sync
2. CAMIF_ERROR blocks frame from being sent to Write Master
3. No data reaches memory despite complete frames in CAMIF

---

## Code Changes This Session

### 1. IRQ_COMPOSITE_MASK Configuration (camss-vfe.c)

```c
/*
 * CRITICAL: Configure IRQ_COMPOSITE_MASK (0x034) to map Write Masters
 * to composite interrupt groups. Without this, IMAGE_COMPOSITE_DONE
 * IRQs (bits 21-23) never fire!
 */
u32 comp_mask = BIT(vfe->camif_pending_wm);
writel_relaxed(comp_mask, vfe->base + VFE31_IRQ_COMPOSITE_MASK_0);
```

### 2. SUBSAMPLE_CFG_1 Fix (camss-vfe.c)

```c
/* SUBSAMPLE_CFG_1: Must be 0 for no frame skipping!
 * Bits [3:0] = frameSkip count. 0xFFFFFFFF sets frameSkip=0xF,
 * which skips 15 out of 16 frames. */
writel_relaxed(0, vfe->base + VFE31_CAMIF_SUBSAMPLE_CFG_1);
```

### 3. UB_CFG Configuration (camss-vfe.c)

```c
/* VFE31 has a 1024-byte unified buffer shared among Write Masters.
 * For single WM operation, use full buffer: offset=0, depth=1023. */
u32 ub_cfg = (0 << 16) | 1023;
writel_relaxed(ub_cfg, vfe->base + VFE31_WM_WR_UB_CFG(wm));
```

### 4. BIT(22) Frame Start Detection (camss-csiphy-8x60.c)

```c
/* BIT(22) fires at frame boundaries on MT9M113.
 * Use it as frame start trigger for software SOF. */
if (status & MIPI_IRQ_FRAME_START) {
    frame_start_detected = true;
} else if (status & BIT(22)) {
    frame_start_detected = true;  // Added this case
} else if (status & MIPI_IRQ_SOT_SYNC) {
    // timing-based detection...
}
```

### 5. MT9M113 Context A Dynamic Configuration (mt9m114.c)

```c
/* Configure Context A dimensions based on requested format */
compose = v4l2_subdev_state_get_compose(ifp_state, 0);
if (compose->width <= 640 && compose->height <= 480) {
    mt9m113_write_mcu_var(sensor, 0x2703, 640);   // OUTPUT_WIDTH_A
    mt9m113_write_mcu_var(sensor, 0x2705, 480);   // OUTPUT_HEIGHT_A
}
```

---

## Recent Commits

| Commit | Description |
|--------|-------------|
| 0fa29de2d5fc | **media: qcom: camss: csiphy-8x60: Use BIT(22) for frame start detection** |
| 04ff38089234 | **media: qcom: camss: vfe31: Add UB_CFG configuration to deferred CAMIF path** |
| 4e9e1fe4198c | **media: qcom: camss: vfe31: Add IRQ_COMPOSITE_MASK to deferred CAMIF path** |
| c789223a9f8b | media: qcom: camss: vfe31: Fix SUBSAMPLE_CFG_1 frame skip configuration |
| 387c7ac35106 | media: i2c: mt9m114: Configure MT9M113 Context A dimensions dynamically |

---

## Root Cause Analysis

### The Core Problem

The MT9M113 sensor is NOT sending proper MIPI Frame Start (FS) short packets, OR the CSIPHY isn't detecting them. Without FS packets:
1. VFE CAMIF cannot properly sync to frame boundaries
2. CAMIF_NO_SOT and CAMIF_EOF_MISMATCH errors fire
3. CAMIF_ERROR prevents data from reaching Write Master
4. No frames are written to memory

### Evidence

1. **CSIPHY `sof_count=0`** - No FS packets detected in hardware
2. **MT9M113 CUSTOM_SHORT_PKT=0x0080** - FS/FE supposedly configured
3. **BIT(22) fires at frame rate** - Some frame boundary signal exists
4. **CAMIF sees complete frames** - Data path to CAMIF works
5. **No MIPI_IRQ_FRAME_START (0x10000)** - Standard FS bit never fires

### Possible Causes

1. **MT9M113 not actually sending FS packets** despite register config
2. **CSIPHY IRQ routing issue** - FS might be at different bit position
3. **MIPI timing mismatch** - FS packets corrupted/missed
4. **webOS uses different frame sync** - Maybe line counting or software sync

---

## Next Steps

### Immediate (Needs Testing)

1. **Test with BIT(22) software SOF** - Rebuild kernel and test
   ```bash
   echo 1 > /sys/module/qcom_camss/parameters/software_sof_enable
   ```

2. **Check if software SOF triggers VFE properly**
   - If software SOF fires, does CAMIF_ERROR clear?
   - Does ping_pong start toggling?

### If Software SOF Doesn't Help

3. **Investigate CAMIF_ERROR source**
   - What specific condition triggers CAMIF_ERROR?
   - Can we mask or ignore frame sync errors?

4. **Try RDI bypass mode**
   - RDI might work without proper frame sync
   - Data goes directly to memory bypassing CAMIF

5. **Check webOS frame sync mechanism**
   - Does webOS rely on FS packets?
   - How does webOS handle MT9M113 frame boundaries?

---

## Module Parameters

### Current Recommended Settings

```bash
# Enable software SOF (uses BIT(22) for frame start)
echo 1 > /sys/module/qcom_camss/parameters/software_sof_enable

# AXI output mode (512=0x200 for PIX, 96=0x60 for RDI)
echo 512 > /sys/module/qcom_camss/parameters/vfe31_axi_output_mode

# MIPI timing parameters (optional)
echo 0x14 > /sys/module/qcom_camss/parameters/settle_cnt_override
echo 0x0F > /sys/module/qcom_camss/parameters/hs_term_imp_override
```

### All Available Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| software_sof_enable | N | Enable software SOF generation from CSIPHY |
| vfe31_axi_output_mode | 1 | AXI output mode (0x200=PIX, 0x60=RDI) |
| settle_cnt_override | -1 | MIPI settle count (-1=use default 0x14) |
| hs_term_imp_override | -1 | HS termination impedance (-1=use default 0x0F) |
| calibration_mode | 0 | PHY calibration mode (0=normal, 1=bypass) |

---

## Diagnostic Commands

```bash
# Run camera test
ssh root@172.16.42.2 "cd /tmp && ./test-camera.sh pix"

# Check VFE IRQ status and CAMIF state
ssh root@172.16.42.2 "dmesg | grep -E 'status0=|CAMIF_STATUS|ping_pong' | tail -30"

# Check CSIPHY frame detection
ssh root@172.16.42.2 "dmesg | grep -E 'CSIPHY.*sof_count|BIT.22|Software SOF' | tail -20"

# Enable software SOF
ssh root@172.16.42.2 "echo 1 > /sys/module/qcom_camss/parameters/software_sof_enable"

# Check current module parameters
ssh root@172.16.42.2 "cat /sys/module/qcom_camss/parameters/*"
```

---

## File Locations

- Sensor driver: `drivers/media/i2c/mt9m114.c`
- CSIPHY driver: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- VFE driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- VFE common: `drivers/media/platform/qcom/camss/camss-vfe.c`
- webOS reference: `webos-linux-kernel-touchpad/drivers/media/video/msm/`

---

## Session History

| Session | Date | Focus | Key Finding |
|---------|------|-------|-------------|
| 1 | 2026-03-26 | Parameter sweep | 52% ECC rate consistent across all params |
| 2 | 2026-03-27 | Calibration modes | Calibration bypass + high settle helps |
| 3 | 2026-03-28 | VFE Write Master | CAMIF sees frames but sync errors block WM |
