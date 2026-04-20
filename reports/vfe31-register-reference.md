# VFE31 Register Reference

## 1. Overview

VFE31 (Video Front End v3.1) is the camera ISP in MSM8660/APQ8060 SoCs.
This document covers register configuration for semi-planar (NV12/NV16) output.

### Block Diagram

```
Sensor (UYVY 8-bit)
    |
CSIPHY (MIPI CSI-2 receiver)
    |
CSID (CSI Decoder)
    |
VFE31 Input (CAMIF)
    |
DEMUX (separates Y and CbCr from interleaved UYVY)
    |
Scaler / Chroma Subsample
    |
XBAR (routes Y/CbCr to Write Masters)
    |
Write Masters (WM0-WM7) --> DMA to memory
```

## 2. Write Masters (WM)

VFE31 has 8 Write Masters (WM0-WM7) organized into 3 output groups:

| WM | Offset | Output | Channel | OUTPUT_1_AND_3 | ZSL/Snapshot |
|----|--------|--------|---------|----------------|--------------|
| WM0 | 0x04C | output0 | ch0 | Preview Y | Preview Y / RAW bypass |
| WM1 | 0x064 | output1 | ch0 | Video Y | Thumbnail Y |
| WM2 | 0x07C | output2 | ch0 | (unused) | Snapshot Y |
| WM3 | 0x094 | output3 | ch0 | (unused) | Snapshot CbCr (ZSL_ALL) |
| WM4 | 0x0AC | output0 | ch1 | Preview CbCr | Preview CbCr |
| WM5 | 0x0C4 | output1 | ch1 | Video CbCr | Thumbnail CbCr |
| WM6 | 0x0DC | output2 | ch1 | (unused) | Snapshot CbCr |
| WM7 | 0x0F4 | output3 | ch1 | (unused) | (unused by all vendors) |

Per-WM register stride: 0x18 bytes. Formula: `base + 0x04C + (n * 0x18)`.

VFE31 has no true RDI hardware. Raw bypass (CAMIF_TO_AXI, AXI=0x60) routes to
WM0 only, confirmed by webOS (`out1.ch0 = 0`) and Samsung kernels.

### Per-Mode WM Assignments (cross-vendor verified)

**OUTPUT_1_AND_3 (Preview + Video):**

| Vendor | SoC | Out0 Y | Out0 CbCr | Out1 Y | Out1 CbCr | Source |
|--------|-----|--------|-----------|--------|-----------|--------|
| webOS kernel | APQ8060 | WM0 | WM4 | WM1 | WM5 | msm_vfe31.c:711-722 |
| webOS Opal HAL | APQ8060 | WM0 | WM4 | WM1 | WM5 | Opal libqcameralib.so |
| Samsung Quincy | MSM8660 | WM0 | WM4 | WM1 | WM5 | Quincy liboemcamera.so |
| Samsung SII | MSM8660 | WM0 | WM4 | WM1 | WM5 | SII liboemcamera.so |
| HTC Sensation | MSM8660 | WM0 | WM1 | WM4 | WM5 | HTC liboemcamera.so |
| Sony Xperia S | MSM8960 | WM0 | WM1 | WM4 | WM5 | Sony liboemcamera.so |
| LG G2 | MSM8974 | WM0 | WM4 | WM1 | WM5 | GitLab kernel source |
| Mako (Nexus 4) | MSM8960 | WM0 | WM4 | WM1 | WM5 | GitHub kernel source |

HTC/Sony pair Out0 as WM0+WM1 (sequential); webOS/Samsung pair as WM0+WM4
(ch0+ch1 of same output). XBAR routes data regardless of blob WM assignment.

**ZSL (Samsung only):**

| Mode | Out0 | Out1 | Out2 | UB constant |
|------|------|------|------|-------------|
| `vfe_zsl_axi_init` | WM0+WM4 | WM1+WM5 | WM2+WM6 | 0x218 (536) |
| `vfe_zsl_axi_init_all_chnls` | WM4+WM5 | WM0+WM1 | WM2+WM3 | 0x2B8 (696) |

**Raw snapshot:**

| Vendor | WM | UB depth | AXI mode | Source |
|--------|------|----------|----------|--------|
| webOS Opal | WM0 | 0x38F (911) | 0x00 | Opal libqcameralib.so |
| Samsung | WM0 | 0x397 (919) | 0x60 | Quincy liboemcamera.so |

### Current Driver Configuration

| Line | Y WM | CbCr WM | Notes |
|------|------|---------|-------|
| PIX (output0) | WM0 | WM4 | Preview/viewfinder |
| VIDEO (output1) | WM1 | WM5 | Video recording |
| ZSL (output2) | WM2 | WM6 | Zero-shutter-lag snapshot |
| RDI | WM0 | -- | CAMIF_TO_AXI bypass, single WM |

### UB Stacking Order (cross-vendor verified)

| Mode | Order |
|------|-------|
| Preview (OUTPUT_2) | WM0 -> WM1 |
| Snapshot/Video (OUTPUT_1_AND_3) | WM0 -> WM4 -> WM1 -> WM5 |
| ZSL | WM0 -> WM4 -> WM1 -> WM5 -> WM2 -> WM6 |

Confirmed identical in webOS Opal, Samsung Quincy, and Samsung SII.

## 3. Global Registers

### AXI_OUTPUT_MODE (0x040)

| Value | Mode | Description | Source |
|-------|------|-------------|--------|
| 0x00 | Output 1 | Single output | -- |
| 0x01 | Output 1 and 3 | PIX + VIDEO | webOS/Samsung |
| 0x02 | Output 1 and 2 | PIX + RDI | -- |
| 0x03 | Camif to AXI | Raw CAMIF output | -- |
| 0x60 | RAW snapshot | CAMIF_TO_AXI_VIA_OUTPUT_2 | webOS/HTC |
| 0x101 | ZSL dual | Preview + Snapshot | Samsung |
| 0x200 | OUTPUT_2 | Preview only (XBAR bypassed) | Samsung/Opal |
| 0x214101 | RAW capture | Samsung-specific extended mode | Samsung |

**Driver setting:** 0x01 (OUTPUT_1_AND_3) for PIX/VIDEO, 0x60 for RDI.

**RAW BUS_CFG by depth (HTC):** 8-bit: 0x2AAA771, 10-bit: 0x2AAA775, 12-bit: 0x2AAA779.

### BUS_CFG (0x03C)

All vendors use 0x02AAA771 for NV12/NV16 output.

```
0x02AAA771 = 0000_0010 1010_1010 1010_0111 0111_0001
```

| Bits | Value | Field | Source |
|------|-------|-------|--------|
| 0 | 1 | stripeRdPathEn | VFE8x struct (confirmed) |
| 1 | 0 | reserved | |
| 3:2 | 00 | rawPixelDataSize: 00=8bit, 01=10bit, 10=12bit | HTC HAL (3 RAW values confirm) |
| 4 | 1 | encYWrPathEn | VFE8x mapping (inferred) |
| 5 | 1 | encCbcrWrPathEn | VFE8x mapping (inferred) |
| 6 | 1 | viewYWrPathEn | VFE8x mapping (inferred) |
| 7 | 0 | viewCbcrWrPathEn (DISABLED) | VFE8x mapping (note: CbCr works despite this) |
| 9:8 | 11 | unknown (always set in all modes) | |
| 11:10 | 10 | rawWritePathSelect: 2=VIEW_CBCR_PATH | VFE_RAW_WR_PATH_SEL enum |
| 25:12 | 0x2AAA | per-WM burst config (7x 2-bit = "10" for WM0-WM6) | Pattern analysis |
| 31:26 | 000000 | reserved | |

**RAW mode variants** (only bits 3:2 change):
- 8-bit RAW: 0x2AAA7**71** (bits 3:2 = 00)
- 10-bit RAW: 0x2AAA7**75** (bits 3:2 = 01)
- 12-bit RAW: 0x2AAA7**79** (bits 3:2 = 10)

### MODULE_CFG (0x010)

webOS value: 0x01C00C0C. Enables ISP processing modules needed for YUV pipeline.

```
0x01C00C0C = 0000_0001 1100_0000 0000_1100 0000_1100
```

| Bit | Value | Field | Confidence |
|-----|-------|-------|------------|
| 0 | 0 | Black Level Correction (BLC) | VFE8x confirmed |
| 1 | 0 | Lens Rolloff | VFE8x confirmed |
| 2 | **1** | **DEMUX** | All kernels confirmed |
| 3 | **1** | **CHROMA_UPSAMPLE** | All kernels confirmed |
| 4 | 0 | Demosaic (CFA interpolation) | VFE8x mapping |
| 5 | 0 | AE stats (AE_ENABLE_MASK = 0x20) | Samsung header confirmed |
| 6 | 0 | AF stats (AF_ENABLE_MASK = 0x40) | Samsung header confirmed |
| 7 | 0 | AWB stats (AWB_ENABLE_MASK = 0x80) | Samsung header confirmed |
| 8 | 0 | RS stats (RS_ENABLE_MASK = 0x100) | Samsung header confirmed |
| 9 | 0 | CS stats (CS_ENABLE_MASK = 0x200) | Samsung header confirmed |
| 10 | **1** | Y Histogram | VFE8x mapping (probable) |
| 11 | **1** | Skin Tone Enhancement | VFE8x mapping (probable) |
| 12-14 | 000 | lumaAdaptation/rgbLUT/chromaEnhan | VFE8x mapping |
| 15 | 0 | IHIST stats (IHIST_ENABLE_MASK = 0x8000) | Samsung header confirmed |
| 16-21 | 000000 | reserved | |
| 22 | **1** | output path module (possibly SCALE_VIEW) | unknown |
| 23 | **1** | **SCALE_ENC** (encoder scaler) | mainline VFE4-1 confirmed |
| 24 | **1** | output path module (possibly CROP_VIEW) | unknown |
| 25-26 | 00 | reserved | |
| 27 | 0 | CROP_ENC (NOT enabled by webOS) | mainline VFE4-1 |
| 28-31 | 0000 | reserved | |

**Stats enable mask** (Samsung header): `STATS_ENABLE_MASK = 0x000483E0` (bits 5,6,7,8,9,15,18).
Samsung `VFE_CMD_MODULE_CFG` preserves stats bits via read-modify-write to avoid
clobbering stats configuration when updating ISP modules.

### CORE_CFG (0x014)

Controls input source and pixel pattern. Also known as `VFE_CFG_OFF` in webOS kernel.

```
Bits [2:0]  = PIXEL_PATTERN  (input pixel format)
Bits [6:4]  = INPUT_MUX      (input source selector)
Bits 3,7    = reserved (zero)
```

**INPUT_MUX values** (from HTC binary `& 0x8f | cVar3 << 4`):

| INPUT_MUX | bits[6:4] | Description |
|-----------|-----------|-------------|
| CAMIF | 001 | Normal sensor input |
| TESTGEN | 011 | Test pattern generator (non-functional on VFE31) |
| AXI | 100 | Memory read input |

**PIXEL_PATTERN values:**

| Value | webOS Enum | Pattern | Linux Format |
|-------|------------|---------|--------------|
| 0 | VFE_BAYER_RGRGRG | RGRGRG | SRGGB8/10/12 |
| 1 | VFE_BAYER_GRGRGR | GRGRGR | SGRBG8/10/12 |
| 2 | VFE_BAYER_BGBGBG | BGBGBG | SBGGR8/10/12 |
| 3 | VFE_BAYER_GBGBGB | GBGBGB | SGBRG8/10/12 |
| 4 | VFE_YUV_YCbYCr | YCbYCr | YUYV |
| 5 | VFE_YUV_YCrYCb | YCrYCb | YVYU |
| 6 | VFE_YUV_CbYCrY | CbYCrY | UYVY (webOS default) |
| 7 | VFE_YUV_CrYCbY | CrYCbY | VYUY |

Bayer patterns are inverted relative to sensor order: sensor 0 -> VFE 3,
sensor 3 -> VFE 0. YUV patterns (4-7) map directly. Verified in HTC and
Samsung binaries with identical lookup tables.

**REG_UPDATE behavior:** YUV modes (4-7) require REG_UPDATE after configuration;
Bayer modes (0-3) do not. From webOS `vfe31_start_recording()` line 1014.

**Common CORE_CFG values:**

| Mode | Value | Breakdown |
|------|-------|-----------|
| UYVY from CAMIF (webOS) | 0x46 | pattern=6, INPUT_MUX=CAMIF |
| Bayer SRGGB from CAMIF | 0x40 | pattern=0, INPUT_MUX=CAMIF |
| Bayer SGRBG from CAMIF | 0x41 | pattern=1, INPUT_MUX=CAMIF |
| Bayer SBGGR from CAMIF | 0x42 | pattern=2, INPUT_MUX=CAMIF |
| Bayer SGBRG from CAMIF | 0x43 | pattern=3, INPUT_MUX=CAMIF |
| Test Generator | 0x56 | pattern=6, INPUT_MUX=TESTGEN |

### V31_OPERATION_CFG Command (28 bytes)

CORE_CFG is written via the V31_OPERATION_CFG ioctl (command ID = 5):

| Offset | Field | Target Register |
|--------|-------|-----------------|
| 0x00 | operation_mode | (software state) |
| 0x04 | stats_comp | (software state) |
| 0x08 | VFE_CFG_OFF | CORE_CFG (0x014) |
| 0x0C | VFE_MODULE_CFG | MODULE_CFG (0x010) |
| 0x10 | VFE_REALIGN_BUF | REALIGN_BUF register |
| 0x14 | VFE_CHROMA_UP | CHROMA_UP register |
| 0x18 | VFE_STATS_CFG | STATS_CFG register |

HTC stores CORE_CFG at HAL buffer offset 0x96c; Samsung at 0x1488.

## 4. XBAR Registers (0x040-0x044)

The XBAR consists of two registers (V31_XBAR_CFG_LEN = 8 bytes):

**XBAR_CFG0 (0x040):** Same as AXI_OUT_MODE (see section 3).

**XBAR_CFG1 (0x044):** Per-output routing register, 24 bits:

```
Bits [7:0]   = output0 routing byte (PIX/preview -> WM0/WM4)
Bits [15:8]  = output1 routing byte (VIDEO/snapshot -> WM1/WM5)
Bits [23:16] = output2 routing byte (ZSL/third output -> WM2/WM6)
Bits [31:24] = reserved (zero)
```

Each routing byte encodes Y and CbCr nibbles:
```
bits[3:0] = Y routing nibble
bits[7:4] = CbCr routing nibble
```

### Routing Byte Values

| Byte | Y[3:0] | CbCr[7:4] | Effect |
|------|--------|-----------|--------|
| 0x00 | 0x0 | 0x0 | Output disabled |
| 0x03 | 0x3 | 0x0 | Y to ch0+ch1, CbCr disabled |
| 0x1A | 0xA | 0x1 | Y to ch1, CbCr to ch0 |
| 0x1B | 0xB | 0x1 | Y to ch0+ch1, CbCr to ch0 |

### Cross-Vendor XBAR_CFG1 Values

| Value | out0 | out1 | out2 | Use Case | Vendors |
|-------|------|------|------|----------|---------|
| 0x000000 | off | off | off | OUTPUT_2 preview (XBAR bypassed) | webOS Opal, Samsung |
| 0x001A03 | 0x03 | 0x1A | -- | NV16 format (format=1,2) | Samsung, Mako/G2 |
| 0x001A1B | 0x1B | 0x1A | -- | NV12 video/snapshot | webOS, Samsung, HTC |
| 0x00021B | 0x1B | 0x02 | -- | NV12 sensor input=1 | Samsung, Opal |
| 0x1A1B1B | 0x1B | 0x1B | 0x1A | ZSL (3 outputs) | Samsung SII |
| 0x001B01 | 0x01 | 0x1B | -- | ZSL all channels | Samsung Quincy |

Samsung dynamically switches output0 byte between 0x1B (Y+CbCr) and 0x03
(Y-only) at runtime via V31_XBAR_CFG ioctl. The kernel stages changes in
xbar_cfg[2] with xbar_update_pending, applied at next REG_UPDATE IRQ.

| Vendor | SoC | Preview | Video/Snapshot | Raw | ZSL | Source |
|--------|-----|---------|----------------|-----|-----|--------|
| webOS Opal | APQ8060 | 0x0000 | 0x1A1B | 0x0000 | N/A | Opal libqcameralib.so |
| webOS Topaz | APQ8060 | 0x1A1B | 0x1A1B | N/A | N/A | Live register dump |
| Samsung Quincy | MSM8660 | 0x0000 | 0x1A1B | 0x1A00 | 0x1B01 | Quincy liboemcamera.so |
| Samsung SII | MSM8660 | 0x0000 | 0x1A1B | 0x1A00 | 0x1B01 | SII liboemcamera.so |
| HTC Sensation | MSM8660 | 0x1A1B | 0x1A1B | N/A | N/A | HTC liboemcamera.so |
| LG G2 | MSM8974 | 0x1A03 | 0x1A03 | N/A | N/A | GitLab kernel source |

webOS Topaz shows 0x1A1B even in preview because it uses OUTPUT_1_AND_3 for
preview (not OUTPUT_2). Opal uses OUTPUT_2 with XBAR=0. Both work.

**Driver setting:** 0x1A1B (OUTPUT_1_AND_3 with PIX+VIDEO routing).

## 5. DEMUX Configuration

DEMUX separates interleaved UYVY into Y and CbCr planes.

### DEMUX_CFG (0x284)

- Bits [2:0] = 1: Bayer mode
- Bits [2:0] = 3: YUV mode

### DEMUX_EVEN_CFG (0x290) and DEMUX_ODD_CFG (0x294)

For YUV modes, both registers receive the same 16-bit value.
For Bayer modes, different 8-bit values go to separate config struct offsets.

**YUV patterns (16-bit, same to EVEN and ODD):**

| Pattern | Value | DEMUX mode |
|---------|-------|------------|
| YCbYCr (YUYV) | 0x9CAC | 3 |
| YCrYCb (YVYU) | 0xAC9C | 3 |
| CbYCrY (UYVY) | 0xC9CA | 3 |
| CrYCbY (VYUY) | 0xCAC9 | 3 |

**Bayer patterns (8-bit values at separate config offsets):**

| Pattern | Offset 0x0C | Offset 0x10 | DEMUX mode |
|---------|-------------|-------------|------------|
| RGRGRG | 0xAC | 0xC9 | 1 |
| GRGRGR | 0xCA | 0x9C | 1 |
| BGBGBG | 0x9C | 0xCA | 1 |
| GBGBGB | 0xC9 | 0xAC | 1 |

The values 0xC9/0xCA/0xAC/0x9C encode byte extraction positions within the
pixel data. All vendors (HTC, Samsung, Sony) use identical lookup tables.
Source: HTC `vfe_demux_set_cfg_parms()` line 56150.

**Driver setting:** 0xC9CA for UYVY, matching webOS register dumps.

## 6. Per-WM Registers

Each WM has 6 registers at stride 0x18:

| Register | WM0 Offset | Description |
|----------|------------|-------------|
| WR_CFG | 0x04C | Enable (bit 0) |
| WR_PING_ADDR | 0x050 | Ping buffer DMA address |
| WR_PONG_ADDR | 0x054 | Pong buffer DMA address |
| WR_ADDR_CFG | 0x058 | UB SRAM allocation |
| WR_UB_CFG | 0x05C | UB depth + height |
| WR_IMAGE_SIZE | 0x060 | Stride + height + flags |

### WR_IMAGE_SIZE

```
Bits [27:16] = stride field
Bits [15:4]  = height - 1
Bits [1:0]   = 0x2 (linear memory, 16-byte aligned bursts)
```

The stride field uses input stride: `input_stride / 16`.
This is equivalent to `(pixel_width + 7) / 8` since `width*2/16 = width/8`.

**Important:** IMAGE_SIZE and UB_CFG use input stride for VFE pipeline timing.
The DEMUX splits UYVY internally, and WMs write at compact output stride
(width bytes/line for Y, width bytes/line for interleaved CbCr). Buffer
allocation and CbCr offset use output stride: `cbcr_offset = width * height`.

| Resolution | Y value | CbCr (NV12) | CbCr (NV16) |
|------------|---------|-------------|-------------|
| 640x480 | 0x00501DF2 | 0x00500EF2 | 0x00501DF2 |
| 1280x1024 | 0x00A03FF2 | 0x00A01FF2 | 0x00A03FF2 |

For 640x480 Y: stride = 0x50 = 80 = 1280/16, height = 0x1DF = 479.

**Cross-vendor:** UB_CFG and IMAGE_SIZE values match across all vendors.
The apparent stride discrepancy (webOS: input_stride/16 vs HAL: (width+15)/16-1)
produces identical results for widths that are multiples of 8.

**Y vs CbCr IMAGE_SIZE:** All vendors use identical stride for Y and CbCr,
except Samsung snapshot mode which uniquely uses half-width for CbCr stride.
CbCr height is `height/2 - 1` for NV12, `height - 1` for NV16.

All vendors set flag `| 0x2` explicitly. Samsung masks with 0x1ff for stride,
HTC/Sony with 0x3ff.

### WR_ADDR_CFG (UB SRAM Allocation)

```
Bits [25:16] = UB start offset (10-bit, entry index in UB SRAM)
Bits [9:0]   = UB depth (10-bit, entries allocated to this WM)
```

Uses proportional allocation from the 912-entry image WM budget (see section 9).

**webOS values (640x480 NV12, OUTPUT_1_AND_3):**

| WM | Purpose | Value | UB Start | UB Depth | UB Range |
|----|---------|-------|----------|----------|----------|
| WM0 | Preview Y | 0x0000012F | 0 | 303 | [0..302] |
| WM4 | Preview CbCr | 0x01300097 | 304 | 151 | [304..454] |
| WM1 | Video Y | 0x01C8012F | 456 | 303 | [456..758] |
| WM5 | Video CbCr | 0x02F80097 | 760 | 151 | [760..910] |

Total: 911 of 912 UB entries used (sequential stacking, +1 gap between WMs).

### WR_UB_CFG

```
Bits [24:16] = depth = (input_stride / 32) - 1
Bits [11:0]  = height - 1
```

| Resolution | Y value | CbCr (NV12) | CbCr (NV16) |
|------------|---------|-------------|-------------|
| 640x480 | 0x002701DF | 0x002700EF | 0x002701DF |
| 1280x1024 | 0x004F03FF | 0x004F01FF | 0x004F03FF |

For 640x480 Y: depth = 0x27 = 39 = (1280/32)-1, height = 0x1DF = 479.

HTC/Samsung/Sony HALs use a proportional formula `(pixels * K / total) - 1 + 64`
with K=664 or K=792. All add `+64` headroom (`+ 0x40U & 0x3ff`). For
single-output scenarios the proportional formula converges to the simpler
stride/32-1 formula, so all vendors produce the same register values.

## 7. Chroma Scaling

### CHROMA_H_IMAGE

Horizontal chroma subsampling image width: `(output_width << 16) | input_width`.

- NV12 (4:2:0): output = input (no horizontal subsampling; CbCr is interleaved)
- NV16 (4:2:2): output = input (same -- no horizontal subsampling)

Source: webOS register analysis, confirmed by driver commit 178d58f331ff.

### CHROMA_V_IMAGE

Vertical chroma subsampling image height: `(output_height << 16) | input_height`.

- NV12: output_height = height/2
- NV16: output_height = height (no vertical subsampling)

### CHROMA_SUBS_CFG

| Value | Meaning | Source |
|-------|---------|--------|
| 0x30 | Enable + vsubSample (NV12, 4:2:0) | webOS register dumps |
| 0x10 | Enable only (NV16, 4:2:2 -- no vertical subsampling) | webOS register analysis |

## 8. IRQ and Composite Mask

### IRQ_COMPOSITE_MASK (0x034)

Groups WMs for combined completion interrupts:

```
Bits [7:0]   = Group 0 -> IMAGE_COMPOSITE_DONE_0 (IRQ_STATUS_0 bit 21)
Bits [15:8]  = Group 1 -> IMAGE_COMPOSITE_DONE_1 (IRQ_STATUS_0 bit 22)
Bits [23:16] = Group 2 -> IMAGE_COMPOSITE_DONE_2 (IRQ_STATUS_0 bit 23)
```

Each bit corresponds to a WM (bit 0=WM0, bit 4=WM4, etc.).

| Mode | Value | Group 0 | Group 1 | Group 2 | Source |
|------|-------|---------|---------|---------|--------|
| Preview+Video (OUTPUT_1_AND_3) | 0x00220011 | WM0+WM4 (PIX) | -- | WM1+WM5 (VIDEO) | webOS vfe31_start() |
| Preview only (OUTPUT_2) | 0x00000003 | WM0+WM1 | -- | -- | webOS vfe31_start() |
| Snapshot (capture, op_mode=1) | 0x00002211 | WM0+WM4 (thumb) | WM1+WM5 (main) | -- | webOS vfe31_capture() |
| Raw snapshot (CAMIF_TO_AXI) | 0x00000100 | -- | WM0 | -- | webOS vfe31_capture() |

All vendors use kernel-computed masks; no HAL overrides.

**Driver defines:**

| Define | Value | Usage |
|--------|-------|-------|
| PIX_ONLY | 0x00000011 | Group 0: WM0+WM4 |
| PIX_VIDEO | 0x00220011 | Group 0: WM0+WM4, Group 2: WM1+WM5 |
| VIDEO_ONLY | 0x00220000 | Group 2: WM1+WM5 |
| ZSL_ONLY | 0x00004400 | Group 1: WM2+WM6 |
| PIX_ZSL | 0x00004411 | Group 0: WM0+WM4, Group 1: WM2+WM6 |
| PIX_VID_ZSL | 0x00224411 | All three groups |

## 9. UB SRAM Layout and Proportional Allocation

### Total UB SRAM: 1024 entries (indices 0-1023)

| Region | Entries | Purpose |
|--------|---------|---------|
| 0-911 | 912 | Image write masters |
| 912-919 | 8 | AEC stats |
| 920-927 | 8 | AF stats |
| 928-943 | 16 | AWB stats |
| 944-951 | 8 | RS stats |
| 952-983 | 32 | CS stats |
| 984-1015 | 32 | HIST stats |
| 1016-1023 | 8 | SKIN stats |

Stats allocation is hardcoded in webOS msm_vfe31.c.

### Proportional Allocation Formula

All vendor HALs distribute the 912-entry image budget proportionally:

```
UB_depth = floor(plane_pixels * UB_budget / total_bandwidth) - 1
UB_start = previous_WM_end + 1  (sequential stacking)
```

Where:
- `plane_pixels = width * height` for Y, `(width/2) * height` for CbCr (NV12)
- `total_bandwidth = sum of all active plane sizes`
- `UB_budget` = mode-dependent constant (see below)

### UB Budget Constants

All vendor constants derive from the same formula: `920 - 64 * num_active_wms`
(920 = 912 usable UB entries + 8 margin, 64 = per-WM FIFO headroom).

| Constant | Decimal | Active WMs | Formula | Vendors |
|----------|---------|------------|---------|---------|
| 0x390 | 912 | N/A | Direct budget, no per-WM headroom | webOS/Opal |
| 0x318 | 792 | 2 | 920 - 64*2 = 792 | Samsung (single-output) |
| 0x2B8 | 696 | 3-4 | 920 - 64*3.5 ≈ 696 | Samsung (ZSL all channels) |
| 0x298 | 664 | 4 | 920 - 64*4 = 664 | HTC, Sony, Samsung (multi-output) |
| 0x258 | 600 | 5 | 920 - 64*5 = 600 | Samsung (raw video, case 4) |
| 0x218 | 536 | 6 | 920 - 64*6 = 536 | Samsung (ZSL) |

HTC/Samsung/Sony pre-subtract headroom then add `+64` per result (`(depth + 0x40) & 0x3ff`).
webOS/Opal uses 912 directly without per-WM headroom. Both approaches converge to ~912 total.

**Verification (640x480 NV12, OUTPUT_1_AND_3, UB_budget=912):**
```
total_bw = 2 * (640 * 480 * 1.5) = 921,600
Y_depth  = (640*480 * 912) / 921,600 - 1 = 303  (matches webOS 0x012F)
Cb_depth = (320*480 * 912) / 921,600 - 1 = 151  (matches webOS 0x0097)
```

## 10. AXI Config Blob (Vendor HAL Analysis)

All vendor HALs construct a pre-computed AXI config blob sent to the kernel via
`MSM_CAM_IOCTL_AXI_CONFIG` (ioctl 0x40046d10). The kernel writes this directly
to VFE registers at offset 0x038 (V31_AXI_OUT_OFF), then patches in buffer
physical addresses.

### Blob Sizes

| Vendor | SoC | Size | Construction | Source |
|--------|-----|------|--------------|--------|
| webOS | APQ8060 | 188 bytes (0xBC) | Userspace HAL | kernel: msm_vfe31.c:787 |
| HTC | MSM8660 | 216 bytes (0xD8) | Stack buffer in axi_config() | liboemcamera.so:36072 |
| Samsung | MSM8660 | 212 bytes (0xD4) | Global struct at cfgctrl+0x20 | liboemcamera.so:41834 |
| Sony | MSM8960 | 224 bytes (0xE0) | Stack buffer in axi_config() | liboemcamera.so:28274 |

### Blob-to-VFE Register Layout

```
Blob+0x00  -> VFE 0x038  AXI_OUT_CFG (BUS_CMD reload mask)
Blob+0x04  -> VFE 0x03C  BUS_CFG
Blob+0x08  -> VFE 0x040  AXI_OUT_MODE (XBAR_CFG0)
Blob+0x0C  -> VFE 0x044  XBAR_CFG1
Blob+0x10  -> VFE 0x048  Reserved
Blob+0x14  -> VFE 0x04C  WM0_WR_CFG
Blob+0x18  -> VFE 0x050  WM0_WR_PING_ADDR
Blob+0x1C  -> VFE 0x054  WM0_WR_PONG_ADDR
Blob+0x20  -> VFE 0x058  WM0_WR_ADDR_CFG
Blob+0x24  -> VFE 0x05C  WM0_WR_UB_CFG
Blob+0x28  -> VFE 0x060  WM0_WR_IMAGE_SIZE
...repeat for WM1-WM7 at +0x18 each...
Blob+0xB8  -> VFE 0x0F0  WM7_WR_IMAGE_SIZE (end of 188-byte region)
```

### Notable Vendor Divergences

- HTC writes a format descriptor (0x200 for NV12) into the AXI_OUT_MODE blob
  position; the kernel overwrites with the actual mode. webOS/Samsung write
  the standard mode value (0x01) directly.
- ADDR_CFG values differ between vendors because webOS uses direct UB offsets
  while HTC/Samsung use proportional allocation with +64 headroom, but the
  resulting UB layouts are functionally equivalent.
- UB_CFG and IMAGE_SIZE values match across all vendors.

### Sony VFE32 Differences

Sony Xperia S (MSM8960) uses VFE32 with slightly different register layout:
WR_CFG carries expanded fields, WR_ADDR_CFG positions are zeroed (computed by
kernel), and WM pairing is WM0+WM1 for preview, WM4+WM5 for video (same as HTC).
UB formula identical to HTC (constant 0x298 = 664).

## 11. Cross-Vendor Verification Summary

| Parameter | HTC | Samsung | Sony | Status |
|-----------|-----|---------|------|--------|
| IMAGE_SIZE stride | `(width+0xf>>4)-1` | `(width+0xf>>4)-1` | `(width+0xf>>4)-1` | All identical |
| IMAGE_SIZE flags | `\| 2` | `& 0xfc \| 2` | `& 0xc \| ... \| 2` | Flag 0x2 required |
| UB_CFG headroom | `+ 0x40U` (+64) | `+ 0x40U` (+64) | `+ 0x40U` (+64) | All add +64 |
| DEMUX UYVY | 0xC9CA | 0xC9CA | 0xC9CA | All identical |
| XBAR routing | 0x1A1B | 0x1A1B | kernel default | HTC/Samsung match webOS |
| BUS_CFG | 0x02AAA771 | 0x02AAA771 | 0x02AAA771 | All identical |

**Source files analyzed:**
- HTC: `reports/htc-camera-decompiled/liboemcamera.so_decompiled.c` (lines 36127-36170, 56150)
- Samsung: `reports/Samsung II/decompiled/samsung_liboemcamera.so_decompiled.c` (lines 41177-41264)
- Sony: `reports/sony_nozomi/decompiled/liboemcamera.so_decompiled.c` (lines 28320-28400)

## 12. TESTGEN (Non-Functional on VFE31)

VFE31 (MSM8660/APQ8060) does not have a working test pattern generator.

**Evidence:**
1. webOS VFE31 has V31_TEST_GEN_START in the command array but no handler in
   `vfe31_proc_general()` -- the command is silently ignored.
2. TESTGEN_CFG (0x15C) reads back 0x00 after writing. Adjacent registers work.
3. VFE8x TESTGEN addresses (0x364-0x39C) are repurposed in VFE31 for
   FOV (0x360), MAIN_SCALER (0x368), WB (0x384), COLOR_COR (0x388).

| VFE | TESTGEN Status | Notes |
|-----|----------------|-------|
| VFE8x | Working (0x364-0x39C) | Full hardware TPG |
| VFE31 | Non-functional (0x158-0x174) | Writes don't stick |
| VFE32 | PM registers (0x188/0x18C) | Performance monitor, not TPG |
| VFE41+ | Removed | Only reset bit exists |

The CORE_CFG INPUT_MUX=0x03 (TESTGEN) field exists but leads to unimplemented
silicon. All vendor implementations use only CAMIF or AXI input.

### VFE8x TESTGEN Registers (Working Reference)

| Register | Offset | Description |
|----------|--------|-------------|
| VFE_TESTGEN_CFG | 0x364 | Configuration |
| VFE_SW_TESTGEN_CMD | 0x368 | Software command |
| VFE_HW_TESTGEN_CMD | 0x36C | Hardware command (GO=0x01, STOP=0x02) |
| VFE_HW_TESTGEN_CFG | 0x370 | Hardware config |
| VFE_HW_TESTGEN_IMAGE_CFG | 0x374 | Image dimensions |
| VFE_HW_TESTGEN_SOF_OFFSET_CFG | 0x378 | Start of frame offset |
| VFE_HW_TESTGEN_EOF_NOFFSET_CFG | 0x37C | End of frame offset |
| VFE_HW_TESTGEN_SOL_OFFSET_CFG | 0x380 | Start of line offset |
| VFE_HW_TESTGEN_EOL_NOFFSET_CFG | 0x384 | End of line offset |
| VFE_HW_TESTGEN_HBI_CFG | 0x388 | Horizontal blanking |
| VFE_HW_TESTGEN_VBL_CFG | 0x38C | Vertical blanking |
| VFE_HW_TESTGEN_COLOR_BARS_CFG | 0x398 | Color bar pattern |
| VFE_HW_TESTGEN_RANDOM_CFG | 0x39C | Random pattern seed |

VFE8x enable sequence: configure TESTGEN params -> start CAMIF -> send GO
command to 0x36C.

## 13. References

### Local Sources
- webOS kernel: `drivers/media/video/msm/msm_vfe31.c` and `msm_vfe31.h`
- webOS VFE8x: `drivers/media/video/msm/msm_vfe8x_proc.c` and `msm_vfe8x_proc.h`
- Mako kernel: `drivers/media/video/msm/vfe/msm_vfe31.c`
- Register dumps: `reports/webos-preview-mode-dump.txt`, `reports/webos-video-mode-dump.txt`

### Decompiled Vendor Binaries
- HTC: `reports/htc-camera-decompiled/liboemcamera.so_decompiled.c`
- Samsung: `reports/Samsung II/decompiled/samsung_liboemcamera.so_decompiled.c`
- Sony: `reports/sony_nozomi/decompiled/liboemcamera.so_decompiled.c`

### External Sources
- [Google MSM Kernel VFE32](https://android.googlesource.com/kernel/msm/+/android-msm-hammerhead-3.4-kk-fr2/drivers/media/platform/msm/camera_v1/vfe/)
- [Linux Mainline CAMSS VFE](https://github.com/torvalds/linux/tree/master/drivers/media/platform/qcom/camss)
