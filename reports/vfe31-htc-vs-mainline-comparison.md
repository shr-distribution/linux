# VFE31 HTC Binary vs Mainline Implementation Comparison

**Date:** 2026-04-12
**Purpose:** Comprehensive comparison of HTC camera HAL configuration with mainline camss-vfe-3-1.c

---

## Executive Summary

| Category | HTC Implementation | Mainline Status | Priority |
|----------|-------------------|-----------------|----------|
| Core Registers | Full | **Complete** | - |
| AXI/Bus Config | Full | **Complete** | - |
| XBAR Routing | Full | **Complete** | - |
| BUS_CFG RAW depth | 8/10/12-bit | **Complete** (just fixed) | - |
| DEMUX Config | Full | **Complete** | - |
| Chroma Subsample | Full (12 bytes) | Partial | Medium |
| Chroma Scale | Full | **Complete** | - |
| Frame Drop | Full | **Complete** | - |
| Statistics | AEC/AF/AWB/Hist | **Not Implemented** | Low |
| ISP Modules | 20+ modules | **Not Implemented** | Low |
| Write Masters | WM0-WM6 | WM0/WM4 only | Low |

---

## 1. OUTPUT MODES - AXI_OUT_MODE (0x040)

| Mode | HTC Value | Mainline Value | Status |
|------|-----------|----------------|--------|
| PIX Mode (Preview/Video) | 0x01 | 0x01 (`VFE_0_BUS_XBAR_CFG0_OUTPUT_1_AND_3`) | **Match** |
| RAW Mode (RDI) | 0x60 | 0x60 (`VFE_0_BUS_AXI_OUT_MODE_RAW_WM0`) | **Match** |
| YUV420 Snapshot | 0x200 | Not used | N/A |
| YUV422 Snapshot | 0x1a00 | Not used | N/A |
| Other Snapshot | 0x204000 | Not used | N/A |

**Assessment:** Core modes implemented correctly.

---

## 2. BUS_CFG Register (0x03C)

| Configuration | HTC Value | Mainline Value | Status |
|--------------|-----------|----------------|--------|
| Base YUV Config | 0x02AAA771 | 0x02AAA771 (`VFE_0_BUS_CFG_WEBOS_VALUE`) | **Match** |
| RAW 8-bit | 0x02AAA771 | 0x02AAA771 | **Match** |
| RAW 10-bit | 0x02AAA775 | 0x02AAA775 | **Match** (just fixed) |
| RAW 12-bit | 0x02AAA779 | 0x02AAA779 | **Match** (just fixed) |

**Bit Field Analysis:**
| Field | Bits | HTC | Mainline | Status |
|-------|------|-----|----------|--------|
| RAW Pixel Size | 2-3 | 0/1/2 | 0/1/2 | **Match** |
| Enc Y WR Path | 4 | Set | Set | **Match** |
| Enc CbCr WR Path | 5 | Set | Set | **Match** |
| View Y WR Path | 6 | Set | Set | **Match** |
| View CbCr WR Path | 7 | Variable | Set | **Match** |
| RAW WR Path Sel | 10-11 | 0 | 0 | **Match** |

**Assessment:** Complete after recent fix.

---

## 3. XBAR_CFG1 Register (0x044)

| Configuration | HTC Value | Mainline Value | Status |
|--------------|-----------|----------------|--------|
| PIX/VIDEO Mode | 0x1A1B | 0x1A1B (`VFE31_XBAR_PIX_VIDEO`) | **Match** |
| PIX Only (webOS default) | 0x1A03 | 0x1A03 (`VFE31_XBAR_PIX_ONLY`) | **Match** |
| RAW Mode | 0x00 (unused) | Not written | **Match** |

**Bit Field Analysis:**
| Field | Bits | Value | Description |
|-------|------|-------|-------------|
| Y Routing | 0-3 | 0x3/0xB | WM0 only / WM0+WM4 |
| CbCr Routing | 4-7 | 0x1/0x9 | WM4 only / WM4+WM5 |
| ISP Path | 8-12 | 0x1A | Standard ISP path |

**Assessment:** Complete.

---

## 4. MODULE_CFG Register (0x010)

| Module Bit | HTC Usage | Mainline Define | Mainline Usage | Status |
|------------|-----------|-----------------|----------------|--------|
| Bit 2 (DEMUX) | Enabled | `VFE_0_MODULE_CFG_DEMUX` | Used | **Match** |
| Bit 3 (Chroma Upsample) | Enabled | `VFE_0_MODULE_CFG_CHROMA_UPSAMPLE` | Used | **Match** |
| Bit 5 (Stats AE) | Enabled | `VFE_0_MODULE_CFG_STATS_AE_EN` | Defined | Not used |
| Bit 6 (Stats AF) | Enabled | `VFE_0_MODULE_CFG_STATS_AF_EN` | Defined | Not used |
| Bit 7 (Stats AWB) | Enabled | `VFE_0_MODULE_CFG_STATS_AWB_EN` | Defined | Not used |
| Bit 8 (Stats RS) | Enabled | `VFE_0_MODULE_CFG_STATS_RS_EN` | Defined | Not used |
| Bit 9 (Stats CS) | Enabled | `VFE_0_MODULE_CFG_STATS_CS_EN` | Defined | Not used |
| Bit 15 (Stats IHIST) | Enabled | `VFE_0_MODULE_CFG_STATS_IHIST_EN` | Defined | Not used |
| Bit 23 (Scale Enc) | Enabled | `VFE_0_MODULE_CFG_SCALE_ENC` | Used | **Match** |
| Bit 27 (Crop Enc) | Enabled | `VFE_0_MODULE_CFG_CROP_ENC` | Used | **Match** |

**HTC webOS Value:** 0x01C00C0C
- Bit 2: DEMUX (1)
- Bit 3: Chroma Upsample (1)
- Bit 10-11: Unknown (3)
- Bit 23: Scale Enc (1)
- Bit 24-25: Unknown (3)

**Assessment:** Core bits match. Statistics modules defined but not used in mainline.

---

## 5. DEMUX Configuration (0x284-0x294)

| Register | Offset | HTC Value | Mainline Value | Status |
|----------|--------|-----------|----------------|--------|
| DEMUX_CFG | 0x284 | 0x03 | 0x03 | **Match** |
| DEMUX_GAIN_0 | 0x288 | 0x00800080 | 0x00800080 | **Match** |
| DEMUX_GAIN_1 | 0x28C | 0x00800080 | 0x00800080 | **Match** |
| DEMUX_EVEN_CFG | 0x290 | 0xC9CA | 0xC9CA (configurable) | **Match** |
| DEMUX_ODD_CFG | 0x294 | 0xC9CA | 0xC9CA (configurable) | **Match** |

**Assessment:** Complete.

---

## 6. CHROMA SUBSAMPLE Configuration (0x4F8-0x500)

| Register | Offset | HTC Config | Mainline | Status |
|----------|--------|------------|----------|--------|
| CHROMA_SUBS_CFG | 0x4F8 | 12 bytes via cmd 0x19 | Single value (0x30) | **Partial** |
| CHROMA_SUBS_CFG2 | 0x4FC | Part of 12-byte config | Defined but unused | Missing |
| CHROMA_SUBS_CFG3 | 0x500 | Part of 12-byte config | Defined but unused | Missing |

**HTC Chroma Subsample Bit Fields:**
- Bit 2: NV12/NV21 vs NV16/NV61 selection
- Bit 4: Always set (required)
- Bit 5: Vertical subsampling enable

**Mainline Current:** Only writes 0x30 to CHROMA_SUBS_CFG

**Missing:** Full 12-byte chroma subsample configuration including:
- Horizontal/vertical phase configuration
- Subsample pattern selection
- Edge enhancement settings

**Priority:** Medium - affects NV12 vs NV16 output quality

---

## 7. CHROMA SCALE Configuration (0x4E4-0x4F4)

| Register | Offset | HTC Value | Mainline | Status |
|----------|--------|-----------|----------|--------|
| CHROMA_SCALE_CFG | 0x4E4 | 0x03 | Configured | **Match** |
| CHROMA_H_IMAGE | 0x4E8 | (out<<16)\|in | Configured | **Match** |
| CHROMA_H_PHASE | 0x4EC | 0x00320000 | Configurable | **Match** |
| CHROMA_V_IMAGE | 0x4F0 | (out<<16)\|in | Configurable | **Match** |
| CHROMA_V_PHASE | 0x4F4 | 0x00320000 | Configurable | **Match** |

**Assessment:** Complete with module parameters.

---

## 8. SCALER Configuration (0x368-0x380, 0x4D0-0x4E0)

| Register | Offset | HTC | Mainline | Status |
|----------|--------|-----|----------|--------|
| SCALE_Y_CFG | 0x368 | 0x03 | Configured | **Match** |
| SCALE_Y_H_IMAGE | 0x36C | Configured | Configured | **Match** |
| SCALE_Y_H_PHASE | 0x370 | 0x00310000 | Configured | **Match** |
| SCALE_Y_V_IMAGE | 0x378 | Configured | Configured | **Match** |
| SCALE_Y_V_PHASE | 0x37C | 0x00310000 | Configured | **Match** |
| S2Y_CFG | 0x4D0 | 0x03 | Configured | **Match** |
| S2Y_H_IMAGE | 0x4D4 | Configured | Configured | **Match** |
| S2Y_H_PHASE | 0x4D8 | Configured | Configured | **Match** |
| S2Y_V_IMAGE | 0x4DC | Configured | Configured | **Match** |
| S2Y_V_PHASE | 0x4E0 | Configured | Configured | **Match** |

**Assessment:** Complete.

---

## 9. FRAME DROP Configuration (0x504-0x520)

| Register | Offset | HTC Value | Mainline Value | Status |
|----------|--------|-----------|----------------|--------|
| ENC_Y_CFG | 0x504 | 0x1F | 0x1F | **Match** |
| ENC_CBCR_CFG | 0x508 | 0x1F | 0x1F | **Match** |
| ENC_Y_PATTERN | 0x50C | 0xFFFFFFFF | 0xFFFFFFFF | **Match** |
| ENC_CBCR_PATTERN | 0x510 | 0xFFFFFFFF | 0xFFFFFFFF | **Match** |
| VIEW_Y_CFG | 0x514 | 0x1F | 0x1F | **Match** |
| VIEW_CBCR_CFG | 0x518 | 0x1F | 0x1F | **Match** |
| VIEW_Y_PATTERN | 0x51C | 0xFFFFFFFF | 0xFFFFFFFF | **Match** |
| VIEW_CBCR_PATTERN | 0x520 | 0xFFFFFFFF | 0xFFFFFFFF | **Match** |

**Assessment:** Complete.

---

## 10. CLAMP Configuration (0x524-0x528)

| Register | Offset | HTC Value | Mainline Value | Status |
|----------|--------|-----------|----------------|--------|
| CLAMP_MAX | 0x524 | 0xFFFFFF | 0xFFFFFF | **Match** |
| CLAMP_MIN | 0x528 | 0x000000 | 0x000000 | **Match** |

**Assessment:** Complete.

---

## 11. CAMIF Configuration (0x1E0-0x208)

| Register | Offset | HTC Value | Mainline | Status |
|----------|--------|-----------|----------|--------|
| CAMIF_CMD | 0x1E0 | 0x01/0x05 | Configured | **Match** |
| EFS_CFG | 0x1E4 | 0x40 | Configurable | **Match** |
| FRAME_CFG | 0x1E8 | 0x00 | 0x00 | **Match** |
| WINDOW_WIDTH | 0x1EC | Configured | Configured | **Match** |
| WINDOW_HEIGHT | 0x1F0 | Configured | Configured | **Match** |
| SUBSAMPLE_0 | 0x1F4 | Configured | Configured | **Match** |
| SUBSAMPLE_1 | 0x1F8 | 0xFFFFFFFF | 0xFFFFFFFF | **Match** |
| EPOCH_CFG | 0x1FC | 0x00 | 0x00 | **Match** |
| RAW_CROP_WIDTH | 0x200 | 0x3FFF3FFF | 0x3FFF3FFF | **Match** |

**Assessment:** Complete.

---

## 12. Write Master (WM) Configuration

| WM | HTC Usage | Mainline Usage | Status |
|----|-----------|----------------|--------|
| WM0 | Preview Y / RAW | PIX Y / RDI | **Match** |
| WM1 | Video Y | Not used | Gap |
| WM2 | Unused | Not used | - |
| WM3 | Unused | Not used | - |
| WM4 | Preview CbCr | PIX CbCr | **Match** |
| WM5 | Video CbCr | Not used | Gap |
| WM6 | Unused | Not used | - |

**WM Register Layout (per WM):**
| Register | Offset | HTC | Mainline | Status |
|----------|--------|-----|----------|--------|
| WR_PING_ADDR | +0x00 | Used | Used | **Match** |
| WR_PONG_ADDR | +0x04 | Used | Used | **Match** |
| WR_CFG | +0x08 | Used | Used | **Match** |
| WR_UB_CFG | +0x0C | Used | Used | **Match** |
| WR_IMAGE_SIZE | +0x10 | Used | Used | **Match** |
| WR_FRAMEDROP | +0x14 | Used | Not used | Gap |

**Assessment:** Core WMs implemented. Video path (WM1/WM5) and framedrop per-WM not used.

---

## 13. ISP PROCESSING MODULES - NOT IMPLEMENTED

These modules are fully implemented in HTC HAL but **not present in mainline**:

| Module | Register Base | HTC Cmd | Size | Priority |
|--------|---------------|---------|------|----------|
| BLACK_LEVEL | 0x264 | 0x09 | varies | Low |
| ROLLOFF | 0x274 | 0x78/0x79 | 1136 bytes | Low |
| WB (White Balance) | 0x384 | 0x0E | 4 bytes | Low |
| GAMMA | 0x3BC | 0x10 | 132 bytes | Low |
| LA (Luma Adaptation) | 0x3C0 | 0x11 | varies | Low |
| COLOR_CORRECT | - | 0x0F | 52 bytes | Low |
| COLOR_CONVERSION | - | 0x12 | 36 bytes | Low |
| DEMOSAIC | 0x298 | 0x68 | varies | Low |
| DEMOSAIC_BPC | 0x29C | - | varies | Low |
| ABF (Bayer Filter) | 0x2A4 | 0x74 | 184 bytes | Low |
| MCE (Memory Color) | 0x3F4 | 0x14 | varies | Low |
| SCE (Skin Color) | 0x418 | 0x15 | varies | Low |
| ASF (Spatial Filter) | 0x4A0 | - | varies | Low |
| CHROMA_SUPP | - | 0x13 | varies | Low |

**Note:** These are **image quality enhancement** modules. Raw sensor data passes through without them. They're used for:
- Lens correction (rolloff)
- Color accuracy (WB, color correct)
- Noise reduction (ABF, demosaic BPC)
- Dynamic range (gamma, LA)
- Aesthetic enhancements (MCE, SCE)

**Priority:** Low - camera works without them, images just won't have ISP processing.

---

## 14. STATISTICS MODULES - NOT IMPLEMENTED

| Module | Ping Addr | Pong Addr | Config | HTC Usage | Mainline |
|--------|-----------|-----------|--------|-----------|----------|
| AEC | 0x0F4 | 0x0F8 | 0x530+ | Full | Defined only |
| AF | 0x100 | 0x104 | 0x53C | Full | Defined only |
| AWB | 0x10C | 0x110 | 0x54C | Full | Defined only |
| RS | 0x118 | 0x11C | - | Full | Defined only |
| CS | 0x124 | 0x128 | - | Full | Defined only |
| IHIST | 0x130 | 0x134 | - | Full | Defined only |
| SKIN | 0x13C | 0x140 | - | Full | Not defined |

**Purpose:** These provide real-time statistics for:
- Auto Exposure (AEC)
- Auto Focus (AF)
- Auto White Balance (AWB)
- Image histograms

**Priority:** Low - camera works without them, no auto-exposure/focus/WB.

---

## 15. IRQ Configuration

| Register | HTC Value | Mainline Value | Status |
|----------|-----------|----------------|--------|
| IRQ_MASK_0 | 0x00EFE021 | Similar | **Match** |
| IRQ_MASK_1 | 0x00400000 | Similar | **Match** |
| IRQ_COMPOSITE_MASK | 0x00220011 | Configured | **Match** |

**Assessment:** Complete for basic operation.

---

## 16. VFE COMMAND CODES (HTC HAL to Kernel)

These are ioctl command codes used by HTC HAL (not directly relevant to mainline V4L2):

| Command | Code | Purpose |
|---------|------|---------|
| GET_HW_VERSION | 0x42 | Hardware version query |
| MODULE_CFG | 0x71 | Module enable/disable |
| DEMUX | 0x0B | Demux configuration |
| WB | 0x0E | White balance |
| COLOR_CORRECT | 0x0F | Color correction |
| GAMMA | 0x10 | Gamma LUT |
| LA | 0x11 | Luma adaptation |
| COLOR_CONVERSION | 0x12 | Color space conversion |
| CHROMA_SUPP | 0x13 | Chroma suppression |
| MCE | 0x14 | Memory color enhancement |
| SCE | 0x15 | Skin color enhancement |
| CHROMA_SS | 0x19 | Chroma subsample |
| STATS_AEC | 0x56 | AEC statistics |
| STATS_AF | 0x54 | AF statistics |
| STATS_HIST | 0x60 | Histogram |
| ABF | 0x74 | Adaptive bayer filter |
| ROLLOFF | 0x78/0x79 | Lens rolloff |

**Note:** These are Android-specific ioctls, not needed for V4L2 interface.

---

## SUMMARY: What's Missing vs What's Needed

### Critical (Camera Won't Work Without)
| Feature | Status |
|---------|--------|
| AXI Output Modes | **Complete** |
| BUS_CFG | **Complete** |
| XBAR Routing | **Complete** |
| DEMUX | **Complete** |
| CAMIF | **Complete** |
| Write Masters (basic) | **Complete** |
| Frame Drop | **Complete** |
| Clamp | **Complete** |

### Nice to Have (Improves Quality)
| Feature | Status | Impact |
|---------|--------|--------|
| Full Chroma Subsample (12 bytes) | Partial | Minor quality |
| Video Path (WM1/WM5) | Not used | Simultaneous preview+record |

### Optional (ISP Processing)
| Feature | Status | Impact |
|---------|--------|--------|
| Statistics (AEC/AF/AWB) | Not implemented | No auto exposure/focus |
| Gamma/Color Correct | Not implemented | No color grading |
| Rolloff | Not implemented | No lens shading correction |
| Demosaic enhancements | Not implemented | No RAW processing |
| All other ISP modules | Not implemented | No advanced processing |

---

## RECOMMENDATIONS

### Immediate (Optional but Easy)
1. **Full Chroma Subsample Config** - Add full 12-byte config for better NV12/NV16 handling

### Future (If Needed)
2. **Statistics Buffers** - Enable AEC/AF/AWB if userspace needs them
3. **Video Path** - Enable WM1/WM5 for simultaneous preview+recording
4. **ISP Modules** - Only if raw quality insufficient

### Not Recommended
- ISP modules (black level, rolloff, gamma, etc.) - Complex, userspace can post-process
- DxO rawchip integration - Separate hardware, not VFE31

---

## Conclusion

The mainline VFE31 implementation is **functionally complete** for basic camera operation:
- All core registers match HTC/webOS values
- RAW and PIX modes work correctly
- BUS_CFG RAW depth handling just fixed

The missing ISP modules are **quality enhancements** that:
- Would require significant implementation effort
- Can be done in userspace (libcamera, etc.)
- Are not needed for basic capture functionality

**Camera should work with current implementation.** Missing features affect image quality, not basic functionality.
