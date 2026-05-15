# VFE31 Video Path (WM1/WM5) and NV16 Configuration Analysis

**Date:** 2026-04-12
**Purpose:** Analyze why video path and NV16 don't work in mainline VFE31

---

## Part 1: Video Path (WM1/WM5) Issue

### WebOS Kernel Configuration (OUTPUT_1_AND_3 mode)

From `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c` lines 710-770:

```c
case OUTPUT_1_AND_3: {
    /* use wm0 & 4 for preview, wm1 & 5 for video */
    *p++ = 0x1;       /* XBAR_CFG0 = 0x01 */
    *p = 0x1a03;      /* XBAR_CFG1 = 0x1A03 */

    vfe31_ctrl->outpath.out0.ch0 = 0;  /* preview Y   → WM0 */
    vfe31_ctrl->outpath.out0.ch1 = 4;  /* preview CbCr → WM4 */
    vfe31_ctrl->outpath.out2.ch0 = 1;  /* video Y     → WM1 */
    vfe31_ctrl->outpath.out2.ch1 = 5;  /* video CbCr  → WM5 */
}
```

### XBAR_CFG1 Bit Analysis

| Bits | 0x1A03 Value | 0x1A1B Value | 0x1A9B Value |
|------|--------------|--------------|--------------|
| [3:0] Y routing | 0x3 = WM0 only | 0xB = WM0+WM4 | 0xB = WM0+WM4 |
| [7:4] CbCr routing | 0x0 = **DISABLED** | 0x1 = WM1 only | 0x9 = WM1+WM5 |
| [15:8] ISP path | 0x1A = standard | 0x1A = standard | 0x1A = standard |

### The Problem

**webOS code sets XBAR_CFG1 = 0x1A03** but **register dumps show 0x1A1B**.

This means userspace HAL overwrites the kernel value. The HAL likely uses:
- 0x1A1B for preview (CbCr → WM1 only)
- Different value for video recording

### Critical Issue for Video Path

Looking at CbCr routing bits [7:4]:

| Value | Meaning | Effect |
|-------|---------|--------|
| 0x0 | CbCr disabled | No chroma output |
| 0x1 | CbCr → WM1 only | Works for PIX mode (WM0/WM1) |
| 0x9 | CbCr → WM1 + WM5 | **Required for VIDEO mode (WM4/WM5)** |

**Root Cause:** If mainline uses WM5 for CbCr but XBAR_CFG1 = 0x1A1B, CbCr data only goes to WM1, NOT to WM5!

### Fix for Video Path

To enable simultaneous preview+video (or use WM4/WM5 for video):

```c
/* For VIDEO path using WM5 for CbCr, use 0x1A9B */
#define VFE31_XBAR_VIDEO_DUAL   0x1A9B   /* Y→WM0+WM4, CbCr→WM1+WM5 */

/* Alternative: Only use PIX path (WM0/WM1) - simpler */
#define VFE31_XBAR_PIX_ONLY     0x1A1B   /* Y→WM0+WM4, CbCr→WM1 */
```

### WebOS WM Enable Behavior

From register dumps:
```
PREVIEW/VIDEO MODE:
  WM0 (Y)    = 0x01 (ENABLED)
  WM1 (CbCr) = 0x00 (DISABLED!)   ← Only Y captured!
  WM4 (Y)    = 0x01 (ENABLED)
  WM5 (CbCr) = 0x00 (DISABLED!)   ← Only Y captured!

PICTURE CAPTURE MODE:
  WM0 (Y)    = 0x01 (ENABLED)
  WM1 (CbCr) = 0x01 (ENABLED!)    ← Full YCbCr
  WM4 (Y)    = 0x01 (ENABLED)
  WM5 (CbCr) = 0x01 (ENABLED!)    ← Full YCbCr
```

**Key insight:** WebOS only enables CbCr write masters during picture capture, not during preview/video! For preview, it captures only Y plane and reconstructs color in software.

---

## Part 2: NV16 vs NV12 Configuration

### Chroma Subsample Register Block (0x4F8, 12 bytes)

From webOS header `msm_vfe31.h`:
```c
#define V31_CHROMA_SUBS_OFF   0x000004F8
#define V31_CHROMA_SUBS_LEN   12

struct vfe_cmd_chroma_subsample_config {
    uint8_t enable;              /* offset 0x00 */
    uint8_t cropEnable;          /* offset 0x01 */
    uint8_t vsubSampleEnable;    /* offset 0x02 - KEY for NV12 vs NV16 */
    uint8_t hsubSampleEnable;    /* offset 0x03 */
    uint8_t vCosited;            /* offset 0x04 */
    uint8_t hCosited;            /* offset 0x05 */
    uint8_t vCositedPhase;       /* offset 0x06 */
    uint8_t hCositedPhase;       /* offset 0x07 */
    uint16_t cropWidthFirstPixel;  /* offset 0x08 */
    uint16_t cropWidthLastPixel;   /* offset 0x0A */
    uint16_t cropHeightFirstLine;  /* offset 0x0C (part of 12 bytes) */
    uint16_t cropHeightLastLine;   /* offset 0x0E (extends beyond?) */
};
```

### HTC Binary Chroma Subsample Logic

From `liboemcamera.so` `vfe_chroma_subsample_config` (line 59404):

```c
// param_2[2] = operation mode (5=video, 2=snapshot, etc.)
// param_3 = format type (6 or 7 = special formats)

// Clear bits 0-1
*param_1 = *param_1 & 0xfc;

// Set bit 2 based on mode and format
bVar1 = (mode == 5 || mode == 2) && (format == 6 || format == 7);
*param_1 = *param_1 & 0xf3 | (bVar1 << 2) | 0x10;  // Always set bit 4

// Set bit 5 (vsubSampleEnable) - controls NV12 vs NV16
*param_1 = *param_1 & 0x9f | (enable_vsub << 5);

// Clear lower 12 bits of crop config
*(param_1 + 4) &= 0xf000;
*(param_1 + 6) &= 0xf000;
*(param_1 + 8) &= 0xf000;
*(param_1 + 10) &= 0xf000;

// Write 12 bytes with command 0x19
vfe_util_write_hw_cmd(fd, 0, param_1, 0xc, 0x19, ctx);
```

### Bit Field Analysis for CHROMA_SUBS_CFG (0x4F8)

| Bit | Name | NV12 | NV16 | Description |
|-----|------|------|------|-------------|
| 0-1 | Reserved | 0 | 0 | Cleared |
| 2 | Format select | varies | varies | Mode-dependent |
| 4 | Enable | 1 | 1 | Always set |
| 5 | vsubSampleEnable | **1** | **0** | NV12=1 (2:1 vertical), NV16=0 |

### Current Mainline vs Required Configuration

**Mainline (camss-vfe-3-1.c):**
```c
writel_relaxed(0x30, vfe->base + VFE_0_CHROMA_SUBS_CFG);
// 0x30 = 0011 0000 binary
// bit 4 = 1 (enable)
// bit 5 = 1 (vsubSampleEnable = NV12 mode)
```

**For NV16:**
```c
// Need bit 5 = 0 for NV16 (no vertical subsample)
writel_relaxed(0x10, vfe->base + VFE_0_CHROMA_SUBS_CFG);
// 0x10 = 0001 0000 binary
// bit 4 = 1 (enable)
// bit 5 = 0 (no vsubSample = NV16 mode)
```

### Full 12-Byte Chroma Subsample Configuration

The register block is 12 bytes:
```
Offset 0x4F8: [7:0]  = config byte (bits 0-7 above)
Offset 0x4F9: [7:0]  = crop enable, etc.
Offset 0x4FA-0x4FB: crop width first pixel (16-bit)
Offset 0x4FC-0x4FD: crop width last pixel (16-bit)
Offset 0x4FE-0x4FF: crop height first line (16-bit)
Offset 0x500-0x501: crop height last line (16-bit)
```

Currently mainline only writes the first byte. For proper NV16 support, need full 12-byte config.

---

## Part 3: HTC Output Mode Values

From HTC binary `axi_vfe_config` (line 36350):

| Format Mask | Output Mode | Format Type |
|-------------|-------------|-------------|
| 0x86 | 0x200 | YUV420 (NV12/NV21) |
| 0x41 | 0x1a00 | YUV422 (NV16/NV61) |
| 0x20 | 0x204000 | Other format |
| RAW | 0x60 | Raw bypass |

**Note:** NV16/YUV422 uses a completely different output mode (0x1a00) vs NV12 (0x200)!

---

## Summary: What's Missing

### For Video Path (WM1/WM5):

1. **XBAR_CFG1 wrong value** - Need 0x1A9B if using WM5 for CbCr
2. **Alternative:** Use PIX path (WM0/WM1) only, avoid VIDEO path complexity
3. **WM enable sequence** - webOS only enables CbCr WMs during capture

### For NV16:

1. **CHROMA_SUBS_CFG bit 5** - Must be 0 for NV16 (no vertical subsample)
2. **Full 12-byte config** - Currently only writing 1 byte
3. **Different output mode?** - HTC uses 0x1a00 for YUV422 vs 0x200 for YUV420

---

## Recommended Fixes

### Fix 1: Video Path - Use PIX Only (Simplest)

Keep using WM0/WM1 (PIX path) for everything, avoid WM4/WM5:
```c
/* In vfe31_enable(), always use PIX path */
wm_y = 0;     /* WM0 for Y */
wm_cbcr = 1;  /* WM1 for CbCr - matches XBAR 0x1A1B routing */
```

### Fix 2: Video Path - Dual Output

If simultaneous preview+video needed:
```c
/* Use XBAR 0x1A9B for CbCr to reach both WM1 and WM5 */
writel_relaxed(0x1A9B, vfe->base + VFE_0_BUS_XBAR_CFG1);

/* Preview: WM0 (Y) + WM4 (CbCr) */
/* Video:   WM1 (Y) + WM5 (CbCr) */
```

### Fix 3: NV16 Support

```c
static void vfe31_set_chroma_subsample(struct vfe_device *vfe, bool is_nv16)
{
    u8 config[12] = {0};

    config[0] = 0x10;  /* bit 4 = enable */
    if (!is_nv16) {
        config[0] |= 0x20;  /* bit 5 = vsubSampleEnable for NV12 */
    }
    /* Remaining bytes are crop config - set to 0 or actual values */

    /* Write all 12 bytes */
    for (int i = 0; i < 12; i += 4) {
        writel_relaxed(*(u32 *)&config[i],
                       vfe->base + VFE_0_CHROMA_SUBS_CFG + i);
    }
}
```

### Fix 4: Full Output Mode Support (Complex)

For proper NV16 snapshot mode, may need different AXI output mode:
```c
if (format is YUV422/NV16) {
    axi_mode = 0x1a00;  /* YUV422 output mode */
} else if (format is YUV420/NV12) {
    axi_mode = 0x200;   /* YUV420 output mode */
}
```

---

## Testing Plan

1. **Test XBAR 0x1A9B:**
   ```bash
   echo 0x1a9b > /sys/module/qcom_camss/parameters/vfe31_xbar_cfg1
   # Then capture and check if CbCr appears in WM5
   ```

2. **Test NV16 chroma config:**
   ```bash
   echo 0x10 > /sys/module/qcom_camss/parameters/vfe31_chroma_subs_cfg
   # Then capture NV16 format
   ```

3. **Verify with register dump:**
   ```bash
   # On device during capture
   devmem2 0x04500044 w  # Read XBAR_CFG1
   devmem2 0x045004F8 w  # Read CHROMA_SUBS_CFG
   ```
