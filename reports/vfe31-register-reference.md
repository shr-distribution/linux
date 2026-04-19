# VFE31 Register Reference and Analysis

## Overview

VFE31 (Video Front End version 3.1) is the camera ISP in MSM8660/APQ8060 SoCs.
This document summarizes our understanding of the register configuration for
semi-planar (NV12/NV16) output modes.

## Hardware Block Diagram

```
Sensor (UYVY 8-bit)
    ↓
CSIPHY (MIPI CSI-2 receiver)
    ↓
CSID (CSI Decoder)
    ↓
VFE31 Input (CAMIF)
    ↓
DEMUX (separates Y and CbCr from interleaved UYVY)
    ↓
Scaler/Chroma Subsample
    ↓
XBAR (routes Y/CbCr to Write Masters)
    ↓
Write Masters (WM0-WM7) → DMA to memory
```

## Write Masters (WM)

VFE31 has 8 Write Masters (WM0-WM7) organized into outputs:

| WM | Output | Channel | Typical Use |
|----|--------|---------|-------------|
| WM0 | output0 | ch0 | Preview Y |
| WM1 | output1 | ch0 | Video Y (or shared) |
| WM2 | output2 | ch0 | RDI0 |
| WM3 | output3 | ch0 | RDI1 |
| WM4 | output0 | ch1 | Preview CbCr |
| WM5 | output1 | ch1 | Video CbCr |
| WM6 | output2 | ch1 | RDI2 |
| WM7 | output3 | ch1 | Snapshot/RDI |

**Vendor WM Assignment Table:**

| Vendor | Preview Y | Preview CbCr | Video Y | Video CbCr | Source |
|--------|-----------|--------------|---------|------------|--------|
| **webOS TouchPad** | WM0 | WM4 | WM1 | WM5 | msm_vfe31.c:711-722 |
| **LG G2** | WM0 | WM4 | WM1 | WM5 | gitlab.com/k2wl/g2_kernel |
| **HTC** | WM0 | WM4 | WM1 | WM5 | HAL binary analysis |
| **Mako (Nexus 4)** | WM0 | WM4 | WM1 | WM5 | GitHub kernel source |

**Our Current Configuration:**
- PIX line: WM0 (Y) + WM4 (CbCr) via module params
- VIDEO line: WM1 (Y) + WM5 (CbCr) - should use separate WMs

**ANSWERED:** Yes, VIDEO should use WM1+WM5 to avoid conflicts. All vendors do this.

## Register Addresses

Base: 0x04500000 (VFE)

### Per-WM Registers (WM0 example, add 0x18 per WM)

| Register | Offset | Description |
|----------|--------|-------------|
| WM0_WR_CFG | 0x04C | Enable/config (bit 0 = enable) |
| WM0_WR_PING_ADDR | 0x050 | Ping buffer DMA address |
| WM0_WR_PONG_ADDR | 0x054 | Pong buffer DMA address |
| WM0_WR_ADDR_CFG | 0x058 | Lines (upper 16) + Burst (lower 16) |
| WM0_WR_UB_CFG | 0x05C | FIFO depth (upper 16) + height (lower 16) |
| WM0_WR_IMAGE_SIZE | 0x060 | Stride + height encoding |

Formula for WMn: base + 0x04C + (n * 0x18)

## AXI_OUTPUT_MODE (0x040)

Controls which output mode VFE uses:

| Value | Mode | Description | Source |
|-------|------|-------------|--------|
| 0x00 | Output 1 | Single output | - |
| 0x01 | Output 1 and 3 | PIX + VIDEO (our mode) | webOS/Samsung |
| 0x02 | Output 1 and 2 | PIX + RDI | - |
| 0x03 | Camif to AXI | Raw CAMIF output | - |
| 0x60 | RAW snapshot | CAMIF_TO_AXI_VIA_OUTPUT_2 | webOS/HTC |
| 0x101 | ZSL dual output | Preview + Snapshot | Samsung |
| **0x214101** | RAW capture | Bypass ISP, direct to memory | Samsung |

**Our setting:** 0x01 (OUTPUT_1_AND_3)

**RAW/RDI Mode Configuration:**

| Vendor | AXI_OUT_MODE | Notes |
|--------|--------------|-------|
| **HTC** | **0x60** | Standard Qualcomm CAMIF_TO_AXI_VIA_OUTPUT_2 |
| **Samsung** | 0x214101 | Samsung-specific extended mode |
| **Sony** | Kernel default | No HAL override |

**HTC RAW BUS_CFG by depth:**
- 8-bit RAW: 0x2AAA771 (divisor 8)
- 10-bit RAW: 0x2AAA775 (divisor 6)
- 12-bit RAW: 0x2AAA779 (divisor 5)

**Recommendation:** Use **0x60** for RAW mode (HTC/Qualcomm standard)

## CORE_CFG (0x014)

Controls VFE input source selection and pixel pattern.

### Bit Layout

| Bits | Field | Description |
|------|-------|-------------|
| [2:0] | PIXEL_PATTERN | Input pixel format pattern (0-7) |
| [5:4] | INPUT_MUX | Input source: 0=CAMIF, 1=TESTGEN, 2=unused, 3=AXI |
| [6] | INPUT_MUX_EN | Input mux enable (always set by webOS) |

### Pixel Pattern Values

**CRITICAL:** Samsung/HTC binary analysis confirms that the correct Bayer pixel
pattern MUST be set even for RAW bypass mode (AXI=0x60). Setting pattern=0 for
all RAW formats causes CAMIF to not recognize input data.

| Value | Pattern Name | First Pixel | Linux Format | Use |
|-------|--------------|-------------|--------------|-----|
| **0** | RGRGRG | R | SRGGB8/10/12 | Bayer R-first |
| **1** | GRGRGR | Gr | SGRBG8/10/12 | Bayer Gr-first |
| **2** | BGBGBG | B | SBGGR8/10/12 | Bayer B-first |
| **3** | GBGBGB | Gb | SGBRG8/10/12 | Bayer Gb-first |
| **4** | YCBYCR | Y | YUYV | YUV Y-Cb-Y-Cr |
| **5** | YCRYCB | Y | YVYU | YUV Y-Cr-Y-Cb |
| **6** | CBYCRY | Cb | UYVY | YUV Cb-Y-Cr-Y (webOS default) |
| **7** | CRYCBY | Cr | VYUY | YUV Cr-Y-Cb-Y |

### Vendor Values

| Mode | CORE_CFG | Breakdown | Source |
|------|----------|-----------|--------|
| **UYVY input (webOS)** | **0x46** | pattern=6 + bit6 | Register dump |
| **RAW SBGGR** | 0x42 | pattern=2 + bit6 | Samsung binary |
| **RAW SRGGB** | 0x40 | pattern=0 + bit6 | Samsung binary |

**Samsung vfe_raw_snapshot_config() pattern mapping:**
```c
switch (sensor_pattern) {  // at param_1 + 0x4a8
case 0: pattern = 3;  // GBGBGB
case 1: pattern = 2;  // BGBGBG
case 2: pattern = 1;  // GRGRGR
case 3: pattern = 0;  // RGRGRG
case 4: pattern = 4;  // YCBYCR
case 5: pattern = 5;  // YCRYCB
case 6: pattern = 6;  // CBYCRY (default for YUV)
case 7: pattern = 7;  // CRYCBY
}
```

**Our setting:** Pattern based on mbus format code + bit 6 always set

## XBAR_CFG1 (0x044)

Routes DEMUX output (Y and CbCr) to Write Masters.

### Bit Layout (16-bit value)

```
Bits [15:12] = Reserved/unused
Bits [11:8]  = CbCr routing to output1 (VIDEO)
Bits [7:4]   = CbCr routing to output0 (PIX)
Bits [3:0]   = Y routing
```

### Y Routing (bits [3:0])

| Value | Meaning |
|-------|---------|
| 0x3 | Y → WM0 only |
| 0xB | Y → WM0 AND WM4 (duplicate!) |

### CbCr Routing (bits [7:4] for output0)

| Value | Meaning |
|-------|---------|
| 0x0 | CbCr disabled |
| 0x1 | CbCr → output0.ch1 (WM4) |
| 0x9 | CbCr → output0+output2 (WM4+WM5?) |

### Common XBAR Values

| Value | Decoded | Use Case | Vendors Using |
|-------|---------|----------|---------------|
| **0x1A03** | Y→WM0, CbCr DISABLED | Broken! | LG G2 kernel (bad default) |
| **0x1A13** | Y→WM0, CbCr→WM4 | PIX-only with CbCr | Theoretical correct PIX-only |
| **0x1A1B** | Y→WM0+WM1, CbCr→WM4 | PIX+VIDEO | **webOS TouchPad**, HTC (actual HW) |
| 0x1A9B | Y→WM0+WM1, CbCr→WM4+WM5 | Full dual output | Hypothetical |

**Vendor XBAR Configuration Table:**

| Vendor | SoC | XBAR Value | Evidence |
|--------|-----|------------|----------|
| **webOS TouchPad** | APQ8060 | **0x1A1B** | Register dump from live HW |
| **webOS kernel code** | APQ8060 | 0x1A03 | Inside #ifdef not compiled |
| **LG G2** | MSM8974 | 0x1A03 | Kernel source (gitlab.com/k2wl/g2_kernel) |
| **HTC Sensation** | MSM8660 | 0x1A1B | vfe31-htc-vs-mainline-comparison.md |
| **Samsung Galaxy S II** | MSM8660 | 0x1A1B (preview) / 0x1A03 (video) | Decompiled HAL (lines 41380-41615) |
| **Sony Xperia S** | MSM8960 | Kernel default | No HAL override found |

**Our setting:** 0x1A1B (matching webOS actual hardware)

**ANSWERED:** bits [15:8] = 0x1A is the ISP path selector. All vendors use 0x1A.

## DEMUX Configuration

DEMUX separates interleaved UYVY into Y and CbCr planes.

### DEMUX_EVEN_CFG (0x290) and DEMUX_ODD_CFG (0x294)

For UYVY input (U0 Y0 V0 Y1 U1 Y2 V1 Y3...):

```
UYVY byte order: [U][Y][V][Y] [U][Y][V][Y]...
Byte positions:   0  1  2  3   4  5  6  7
```

Each config register has format: (value1 << 8) | value2

**webOS values:**
- DEMUX_EVEN_CFG = 0xC9CA
- DEMUX_ODD_CFG = 0xC9CA

**Our current values:**
- vfe31_demux_even = 201 (0xC9) - only lower byte!
- vfe31_demux_odd = 202 (0xCA) - only lower byte!

**QUESTION:** Should these be 0xC9CA (16-bit) or just 0xC9/0xCA (8-bit)?

The register appears to be 32-bit. webOS writes 0xC9CA to both EVEN and ODD.

### DEMUX Channel Mapping

The 0xC9 and 0xCA values likely encode:
- Which input byte position maps to Y output
- Which input byte positions map to Cb and Cr outputs

**QUESTION:** What is the exact bit field layout of DEMUX_CFG?

## IMAGE_SIZE Register Format

Each WM has an IMAGE_SIZE register controlling DMA behavior.

### Observed Values

| Resolution | Y WM0 | CbCr WM4 |
|------------|-------|----------|
| 640x480 | 0x00501df2 | 0x00500ef2 |
| 1280x1024 | 0x00a03ff2 | 0x00a01ff2 |

### Bit Layout Analysis

For 1280x1024 Y (0x00a03ff2):
```
0x00a0 = 160 decimal
0x3ff2 = lower 16 bits
  0x3ff = 1023 = height - 1
  0x2 = flags?
```

**Hypothesis:**
- Bits [31:16] = stride / 16 = 2560 / 16 = 160 ✓
- Bits [15:4] = height - 1 = 1023 ✓
- Bits [3:0] = mode flags (0x2)

For 1280x1024 CbCr (0x00a01ff2):
```
0x00a0 = 160 = stride / 16 = 2560 / 16 ✓
0x1ff = 511 = cbcr_height - 1 (512 for NV12 4:2:0) ✓
0x2 = flags
```

**Formula:** `IMAGE_SIZE = ((stride/16) << 16) | ((height-1) << 4) | flags`

**QUESTION:** What do the flag bits [3:0] mean? Always 0x2?

## ADDR_CFG Register Format

Controls burst size and line count for DMA.

### Observed Values

| Resolution | Y WM0 | CbCr WM4 |
|------------|-------|----------|
| 640x480 | 0x01e0012f | 0x01300097 |
| 1280x1024 | 0x01e0012f | 0x02400137 |

### Bit Layout

```
Bits [31:16] = lines
Bits [15:0] = burst
```

For 640x480 Y (0x01e0012f):
- lines = 0x01e0 = 480
- burst = 0x012f = 303 = (640/4) - 1 + some offset?

For 640x480 CbCr (0x01300097):
- lines = 0x0130 = 304 = 240 + 64 (headroom!)
- burst = 0x0097 = 151 = (640/4) - 9

**Vendor Burst Formulas (verified across sources):**

| Component | Formula | Example 640px | Example 1280px |
|-----------|---------|---------------|----------------|
| **Y burst** | (input_stride/4) - 17 | (1280/4)-17 = 303 | (2560/4)-17 = 623 |
| **CbCr burst** | (width/4) - 9 | (640/4)-9 = 151 | (1280/4)-9 = 311 |
| **Y lines (PIX)** | 0 | 0 | 0 |
| **Y lines (VIDEO)** | height - 24 | 480-24 = 456 | 1024-24 = 1000 |
| **CbCr lines (NV12)** | cbcr_h + 64 | 240+64 = 304 | 512+64 = 576 |
| **CbCr lines (NV16)** | cbcr_h | 480 | 1024 |

**webOS Register Values (640x480 preview):**
- WM0_WR_CFG: 0x0000012F (burst=303, lines=0)
- WM1_WR_CFG: 0x01C8012F (burst=303, lines=456)
- WM4_WR_CFG: 0x01300097 (burst=151, lines=304)
- WM5_WR_CFG: 0x02F80097 (burst=151, lines=760)

**ANSWERED:** +64 headroom for CbCr is pipeline flush requirement. DEMUX/Chroma blocks process 16x16 macroblocks; extra lines ensure EOF isn't asserted before final AXI transactions complete.

## UB_CFG Register Format

Controls the Unified Buffer (FIFO) allocation per WM.

### Observed Values

| Resolution | Y WM0 | CbCr WM4 |
|------------|-------|----------|
| 640x480 | 0x002701df | 0x002700ef |
| 1280x1024 | 0x004f03ff | 0x004f01ff |

### Bit Layout

```
Bits [31:16] = ub_depth (FIFO depth in 256-byte chunks?)
Bits [15:0] = ub_height (height - 1)
```

For 640x480 Y:
- ub_depth = 0x0027 = 39
- ub_height = 0x01df = 479 = height - 1

For 1280x1024 Y:
- ub_depth = 0x004f = 79
- ub_height = 0x03ff = 1023 = height - 1

**Observation:** ub_depth scales with resolution:
- 640: depth = 39
- 1280: depth = 79 (roughly 2x)

**QUESTION:** How is ub_depth calculated? Total UB is shared across all active WMs.

## Semi-Planar Format Handling

### NV12 (4:2:0)
- Y plane: width × height bytes
- CbCr plane: width × (height/2) bytes (interleaved Cb,Cr)
- Total: width × height × 1.5 bytes

### NV16 (4:2:2)
- Y plane: width × height bytes
- CbCr plane: width × height bytes (interleaved Cb,Cr)
- Total: width × height × 2 bytes

### Stride Confusion

**Input stride** (from sensor): width × 2 (UYVY = 2 bytes/pixel)
**Output stride** (Y plane): width × 1 (Y = 1 byte/pixel)

**CRITICAL QUESTION:** Does VFE31 write Y/CbCr at:
- Input stride (width × 2)?
- Output stride (width × 1)?

Our `stride_factor = 2` suggests input stride, but comments in code are contradictory.

### Buffer Layout for 640x480 NV12

If output stride = 640:
```
Y plane:    640 × 480 = 307,200 bytes
CbCr plane: 640 × 240 = 153,600 bytes
Total: 460,800 bytes
```

If input stride = 1280:
```
Y plane:    1280 × 480 = 614,400 bytes
CbCr plane: 1280 × 240 = 307,200 bytes
Total: 921,600 bytes
```

**Our buffer sizing:** Uses stride_factor=2, so 921,600 bytes (input stride).

### CbCr Offset Calculation

**Current code (vfe31_cbcr_offset_mode=1):**
```c
cbcr_offset = (width * 2) * height  // Input stride × height
```

For 640x480: cbcr_offset = 1280 × 480 = 614,400

**QUESTION:** Is this correct? Does VFE31 really write at input stride?

## IRQ and Composite Mask

### IRQ_COMPOSITE_MASK (0x01C)

Groups WMs for combined completion interrupt.

```
Bits [7:0]   = Group 0 (COMPOSITE_DONE_0)
Bits [15:8]  = Group 1 (COMPOSITE_DONE_1)
Bits [23:16] = Group 2 (COMPOSITE_DONE_2)
```

Within each group:
- Bit 0 = WM0
- Bit 1 = WM1
- Bit 4 = WM4
- Bit 5 = WM5

**Our setting:** 0x00220011
- Group 0: 0x11 = WM0 + WM4 (PIX Y + CbCr)
- Group 1: 0x00 = none
- Group 2: 0x22 = WM1 + WM5 (VIDEO Y + CbCr)

## webOS Reference Values

From webOS kernel dumps:

| Register | Value | Notes |
|----------|-------|-------|
| AXI_OUT_MODE | 0x01 | OUTPUT_1_AND_3 |
| XBAR_CFG1 | 0x1A1B | PIX+VIDEO routing |
| DEMUX_EVEN_CFG | 0xC9CA | UYVY separation |
| DEMUX_ODD_CFG | 0xC9CA | Same |
| MODULE_CFG | 0x01C00C0C | Enables DEMUX etc |
| BUS_CFG | 0x02AAA771 | Bus arbitration |

## Open Questions - ANSWERED (via Gemini/Copilot Review)

### 1. XBAR bits [15:8]: What does 0x1A prefix mean?
**ANSWER:** Bits [15:8] control Output 1 (VIDEO) and Output 2 (RDI) routing.
- `0x1A` routes Y/CbCr of secondary output to WM1/WM5
- `0x1A03` disables/parks secondary routing (PIX-only mode)
- For concurrent PIX+VIDEO, need different XBAR value

### 2. DEMUX values: Should we write 0xC9CA (16-bit) or 0xC9/0xCA (8-bit)?
**ANSWER:** Must write full 16-bit `0xC9CA` to both registers.
- Writing only 0xC9 (= 0x00C9) zeroes upper byte
- This corrupts chroma extraction mapping
- **Our code is CORRECT** - we do `(0xC9 << 8) | 0xCA = 0xC9CA`

### 3. IMAGE_SIZE flags: What do bits [3:0] control?
**ANSWER:** AXI write formatting and memory arrangement flags.
- `0x2` = linear memory layout with 16-byte aligned bursts
- Changing this causes skewed images or bus faults

### 4. ADDR_CFG lines: Why +64 headroom for CbCr?
**ANSWER:** Hardware pipeline flush requirement.
- DEMUX/Chroma blocks process 16x16 macroblocks
- Extra lines ensure EOF isn't asserted before final AXI transactions complete
- Required to prevent bottom rows being cut off

### 5. Stride: Does VFE31 write at input stride or output stride?
**ANSWER:** VFE31 writes at **OUTPUT stride** (width bytes for Y).
- DEMUX unpacks UYVY into separate 8-bit Y and CbCr streams
- Y WM receives pure 8-bit data, stride = width
- **OUR CODE IS WRONG** - we configure IMAGE_SIZE for input stride (width*2)!

### 6. WM assignment: Should VIDEO use WM1+WM5 instead of WM0+WM4?
**ANSWER:** **YES, absolutely.**
- Using same WMs for PIX and VIDEO causes race conditions
- Both outputs program same ping/pong addresses = buffer corruption
- **PIX:** WM0 (Y) + WM4 (CbCr)
- **VIDEO:** WM1 (Y) + WM5 (CbCr)

### 7. UB_CFG depth: How is FIFO depth calculated/shared?
**ANSWER:** UB is fixed SRAM pool shared among active WMs.
- Must absorb DDR latency during AXI stalls
- Larger lines consume FIFO faster → need more depth
- Sum of all ub_depth must not exceed total physical UB size

### 8. CbCr offset: Is (width×2×height) correct for semi-planar?
**ANSWER:** **NO, it is INCORRECT.**
- Y plane written at output stride: width × height bytes
- CbCr should start immediately after Y
- **Correct formula:** `cbcr_offset = width * height`
- Our formula leaves massive empty gap in buffer

## Known Bugs (Fixed)

1. **Deferred address path used wrong WM defines** (commit 4130ab0b8379)
   - `VFE31_VIDEO_WM_Y=1` but module param `vfe31_video_y_wm=0`
   - Caused pending_ping_addr not to be set for VIDEO line
   - Fixed by using module params instead of hardcoded defines

## Critical Bugs To Fix (from Gemini Review)

### Bug 1: IMAGE_SIZE stride is wrong
**Current:** `image_stride = width * 2` (input stride)
**Correct:** `image_stride = width` (output stride)

The DEMUX outputs 8-bit streams. IMAGE_SIZE should use output stride.
This likely causes the "half-frame" issue - hardware writes at wrong stride.

**Files to fix:**
- `camss-vfe-3-1.c`: All IMAGE_SIZE calculations
- Change from `input_stride / 16` to `output_stride / 16`

### Bug 2: stride_factor should be 1, not 2
**Current:** `pix_stride_factor = 2` in camss.c
**Correct:** `pix_stride_factor = 1`

We're allocating 2x the memory needed. VFE writes at output stride.

**Files to fix:**
- `camss.c`: line 199

### Bug 3: cbcr_offset calculation is wrong
**Current:** `cbcr_offset = width * 2 * height`
**Correct:** `cbcr_offset = width * height`

Y plane is width×height bytes at output stride. CbCr follows immediately.

**Files to fix:**
- `camss-vfe-3-1.c`: Multiple locations where y_plane_size calculated

### Bug 4: VIDEO line should use WM1+WM5
**Current:** `vfe31_video_y_wm = 0`, `vfe31_video_cbcr_wm = 4`
**Correct:** `vfe31_video_y_wm = 1`, `vfe31_video_cbcr_wm = 5`

Sharing WMs with PIX causes corruption when both active.

**Files to fix:**
- `camss-vfe-3-1.c`: Module param defaults

## Test Results

| Test | Result | Notes |
|------|--------|-------|
| pix640 | PASS | 3,145,728 bytes |
| pix1280 | PASS | 12,582,912 bytes |
| video640 | PASS | 3,145,728 bytes |
| video1280 | CRASH | list_add corruption (before fix) |

## References

- webOS kernel: `drivers/media/video/msm/msm_vfe31.c`
- Mako kernel: `drivers/media/video/msm/vfe/msm_vfe31.c`
- Local dumps: `reports/webos-preview-mode-dump.txt`
