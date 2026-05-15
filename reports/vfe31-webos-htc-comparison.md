# VFE31 Register Comparison: webOS vs HTC vs Mainline

**Date:** 2026-04-17
**Purpose:** Comprehensive comparison of VFE31 register values across implementations

---

## Executive Summary

| Register Category | webOS | HTC | Mainline | Status |
|-------------------|-------|-----|----------|--------|
| Core/IRQ | Complete | Complete | **Match** | OK |
| AXI/XBAR | 0x1A1B | 0x1A1B | **Match** | OK |
| DEMUX | 0xC9CA | 0xC9CA | **Match** | OK |
| WM Burst (Y) | INPUT stride | INPUT stride | **OUTPUT stride** | **DIFFERENT** |
| WM Burst (CbCr) | OUTPUT width | OUTPUT width | **Match** | OK |
| WM Lines | 0 (Y), h+64 (CbCr) | Similar | **Match** | OK |

---

## 1. Write Master Configuration

### WM Register Layout (per WM, offset +0x18 each)

| Offset | Register | Description |
|--------|----------|-------------|
| +0x00 | WM_CFG_PNTR | Enable (bit 0) |
| +0x04 | WM_PING_ADDR | Ping buffer DMA address |
| +0x08 | WM_PONG_ADDR | Pong buffer DMA address |
| +0x0C | WM_WR_CFG | (lines << 16) | burst_words |
| +0x10 | WM_y/x_off | UB_CFG: (depth << 16) | height |
| +0x14 | WM_FRAMEDROP | IMAGE_SIZE config |

### webOS Values for 640x480 Preview

| WM | Purpose | WR_CFG | lines | burst | Formula |
|----|---------|--------|-------|-------|---------|
| WM0 | PIX Y | 0x0000012F | 0 | 303 | (1280/4)-17 = INPUT stride |
| WM1 | VIDEO Y | 0x01C8012F | 456 | 303 | lines = 480-24 |
| WM4 | PIX CbCr | 0x01300097 | 304 | 151 | (640/4)-9, lines=240+64 |
| WM5 | VIDEO CbCr | 0x02F80097 | 760 | 151 | Different lines formula |

### Our Current Mainline Values

| WM | Purpose | burst Formula | lines Formula |
|----|---------|---------------|---------------|
| WM0 | PIX Y | (width/4)-17 = OUTPUT | 0 |
| WM4 | PIX CbCr | (width/4)-9 = OUTPUT | cbcr_h+64 (NV12) or cbcr_h (NV16) |

### Key Difference: Y Burst Calculation

**webOS:** `burst = (width * 2 / 4) - 17 = (input_stride / 4) - 17`
**Mainline:** `burst = (width / 4) - 17 = (output_stride / 4) - 17`

This was changed because pix1280 crashed with INPUT stride burst causing buffer overflow.

---

## 2. ADDR_CFG Register Format

```
Bits [31:16] = lines (number of lines for DMA transaction)
Bits [15:0]  = burst_words (32-bit words per burst)
```

### Burst Words Calculation

**Y WM (webOS):** `(input_stride / 4) - 17`
- 640×480: (1280/4) - 17 = 303
- 1280×1024: (2560/4) - 17 = 623

**CbCr WM (webOS):** `(width / 4) - 9`
- 640×480: (640/4) - 9 = 151
- 1280×1024: (1280/4) - 9 = 311

### Lines Value

**Y WM:**
- PIX mode: lines = 0 (VFE uses full frame)
- VIDEO mode: lines = height - 24 (for some reason)

**CbCr WM:**
- NV12 (4:2:0): lines = cbcr_height + 64 (pipeline flush headroom)
- NV16 (4:2:2): lines = cbcr_height

---

## 3. IMAGE_SIZE Register Format

```
Bits [31:16] = stride / 16 (in 128-bit units)
Bits [15:4]  = height - 1
Bits [3:0]   = mode flags (0x2 = linear memory)
```

### webOS Values

| Resolution | Y WM | CbCr WM |
|------------|------|---------|
| 640×480 | 0x00501DF2 | 0x00500EF2 |
| 1280×1024 | 0x00A03FF2 | 0x00A01FF2 |

**Decoded 640×480:**
- stride/16 = 0x50 = 80 (stride = 1280 = INPUT)
- height-1 = 0x1DF = 479
- flags = 0x2

---

## 4. UB_CFG Register Format

```
Bits [31:16] = ub_depth (FIFO depth)
Bits [15:0]  = ub_height (height - 1)
```

### webOS Values

| Resolution | Y WM | CbCr WM |
|------------|------|---------|
| 640×480 | 0x002701DF | 0x002700EF |
| 1280×1024 | 0x004F03FF | 0x004F01FF |

**Decoded 640×480:**
- ub_depth = 0x27 = 39
- ub_height = 0x1DF = 479

---

## 5. XBAR_CFG1 (0x044)

Routes DEMUX outputs to Write Masters.

### Bit Layout

```
Bits [15:12] = Reserved
Bits [11:8]  = CbCr routing to output1 (VIDEO)
Bits [7:4]   = CbCr routing to output0 (PIX)
Bits [3:0]   = Y routing
```

### Values

| Mode | Value | Description |
|------|-------|-------------|
| PIX only | 0x1A03 | Y→WM0, CbCr→WM4 |
| PIX+VIDEO | 0x1A1B | Y→WM0+WM1, CbCr→WM4+WM5 |

---

## 6. DEMUX Configuration

### DEMUX_EVEN_CFG (0x290) and DEMUX_ODD_CFG (0x294)

**webOS Value:** 0xC9CA for both

This configures UYVY byte extraction:
- 0xC9 = Y channel mapping
- 0xCA = Cb/Cr channel mapping

---

## 7. MODULE_CFG (0x010)

**webOS Value:** 0x01C00C0C

| Bit | Module | Status |
|-----|--------|--------|
| 2 | DEMUX | Enabled |
| 3 | Chroma Upsample | Enabled |
| 23 | Scale Enc | Enabled |

---

## 8. Critical Observations

### The Stride Paradox

webOS uses **INPUT stride** (width×2) for:
- IMAGE_SIZE stride calculation
- Y WM burst calculation

But buffer allocation uses **OUTPUT stride** (width).

**Question:** How does this work without buffer overflow?

**Possible Answer:** The IMAGE_SIZE stride controls VFE pipeline timing, but actual DMA writes may be controlled differently by hardware. The burst_words may define burst SIZE but not burst COUNT.

### pix1280 Crash Analysis

Our crash at 1280×1024 with INPUT stride burst:
- VFE burst = (2560/4) - 17 = 623 words × 4 = 2492 bytes/burst
- Buffer stride = 1280 bytes

If VFE writes 2560 bytes per line but buffer only has 1280 bytes stride, overflow occurs.

**Fix Applied:** Changed Y burst to OUTPUT stride: (1280/4) - 17 = 303

This fixed the crash but differs from webOS formula.

### Remaining Issues

1. **CbCr 205-line gap:** At pix1280, CbCr data starts ~205 lines late
   - May be related to burst_lines value
   - webOS uses cbcr_height + 64 = 512 + 64 = 576 for NV12

2. **NV16 format error:** V4L2 format negotiation fails
   - Need to verify format registration

---

## 9. Register Dump Quick Reference

### webOS 640×480 Preview

```
AXI_OUT_MODE (0x040): 0x00000001
XBAR_CFG1 (0x044):    0x00001A1B
MODULE_CFG (0x010):   0x01C00C0C
DEMUX_CFG (0x284):    0x00000003
DEMUX_EVEN (0x290):   0x0000C9CA
DEMUX_ODD (0x294):    0x0000C9CA

WM0_WR_CFG (0x058):   0x0000012F (burst=303, lines=0)
WM0_UB_CFG (0x05C):   0x002701DF
WM0_IMG_SIZE (0x060): 0x00501DF2

WM4_WR_CFG (0x0B8):   0x01300097 (burst=151, lines=304)
WM4_UB_CFG (0x0BC):   0x002700EF
WM4_IMG_SIZE (0x0C0): 0x00500EF2
```

---

## 10. Recommendations

### Immediate

1. **Test with webOS Y burst formula** on pix640 to see if it works
2. **Check buffer allocation** - may need stride_factor=2 with INPUT stride burst
3. **Investigate CbCr delay** - compare burst_lines with webOS values

### For Investigation

1. Why does webOS use INPUT stride for burst but OUTPUT stride for buffers?
2. Is there a hardware register we're missing that controls actual DMA stride?
3. Does the chroma scaler introduce delay that requires different lines value?
