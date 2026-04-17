# VFE31 Register Reference

This document provides a comprehensive reference for the Qualcomm VFE 3.1 (Video Front End)
registers as used on the MSM8660/APQ8060 SoC (HP TouchPad). Information derived from:
- webOS kernel sources (msm_vfe31.h/c)
- Decompiled libqcamera HAL
- Empirical testing and register dumps
- Linux mainline camss driver

## Hardware Overview

- **VFE Version:** 3.1 (HW_VERSION = 0x00030217)
- **SoC:** MSM8660 / APQ8060
- **Base Address:** 0x04500000 (via device tree)
- **Write Masters:** WM0-WM6 (7 total)
- **Output Paths:** Preview (WM0/WM1), Video (WM4/WM5), RDI (WM2/WM3/WM6)

---

## 1. Global/Core Registers (0x000 - 0x014)

### VFE_0_HW_VERSION (0x000) - Read Only
Hardware version identifier.
```
[31:0] VERSION = 0x00030217 for VFE 3.1
```

### VFE_0_GLOBAL_RESET_CMD (0x004) - Write Only
Per-subsystem reset control. Write 1 to reset specific blocks.
```
Bit  0: CORE        - Core logic reset
Bit  1: CAMIF       - Camera interface reset
Bit  2: BUS         - Bus interface reset
Bit  3: BUS_BDG     - Bus bridge reset
Bit  4: REGISTER    - Register block reset
Bit  5: TIMER       - Timer reset
Bit  6: PM          - Power management reset
Bit  7: BUS_MISR    - Bus MISR reset
Bit  8: TESTGEN     - Test generator reset
Bit  9: AXI         - AXI interface reset

Common values:
  0x3FF = Full hardware reset (all bits)
```

### VFE_0_CGC_OVERRIDE (0x00C)
Clock gating control override.
```
[19:0] CGC_OVERRIDE = 0xFFFFF (webOS default - disable clock gating)
```

### VFE_0_MODULE_CFG (0x010)
Module enable flags for processing pipeline.
```
Bit  2: DEMUX_EN           - Enable demux (CRITICAL for YUV processing)
Bit  3: CHROMA_UPSAMPLE_EN - Enable chroma upsampling
Bit 23: SCALE_ENC_EN       - Enable encoder scaler
Bit 27: CROP_ENC_EN        - Enable encoder crop
```

### VFE_0_CORE_CFG (0x014)
Core configuration including pixel pattern.

**Confirmed from webOS msm_vfe31.c:**
- Line 1014: `switch (msm_io_r(vfe31_ctrl->vfebase + VFE_CFG_OFF) & 0x7)`
  This confirms bits [2:0] are the pixel pattern.

```
[2:0] PIXEL_PATTERN (enum VFE_START_PIXEL_PATTERN):
      0x0 = VFE_BAYER_RGRGRG - Bayer Red-Green
      0x1 = VFE_BAYER_GRGRGR - Bayer Green-Red
      0x2 = VFE_BAYER_BGBGBG - Bayer Blue-Green
      0x3 = VFE_BAYER_GBGBGB - Bayer Green-Blue
      0x4 = VFE_YUV_YCbYCr   - YUYV (Y before Cb)
      0x5 = VFE_YUV_YCrYCb   - YVYU (Y before Cr)
      0x6 = VFE_YUV_CbYCrY   - UYVY (Cb-Y-Cr-Y) **webOS/MT9M113 uses this**
      0x7 = VFE_YUV_CrYCbY   - VYUY (Cr-Y-Cb-Y)

[5:3] Unknown (webOS value has bits 3-5 = 0)

Bit 6: VFE_OUTPUT_EN or similar
       webOS sets this bit (0x40). Without it, output may not work.
       Exact function undocumented.

[31:7] Unknown on VFE31 (webOS sets all to 0)

webOS value: 0x00000046
  Breakdown: 0b01000110
    - Bits [2:0] = 0x6 (UYVY pixel pattern)
    - Bits [5:3] = 0x0
    - Bit 6 = 1 (required enable bit)
    - Bits [31:7] = 0

IMPORTANT: VFE31 layout differs from VFE8x. Do not use VFE8x struct vfe_cfg
directly - VFE8x has INPUT_SOURCE at bits [17:16] which is not used by VFE31.
```

---

## 2. Interrupt Registers (0x018 - 0x048)

### VFE_0_IRQ_CMD (0x018) - Write Only
Interrupt command register.
```
Bit 0: GLOBAL_CLEAR - Clear all pending interrupts
```

### VFE_0_IRQ_MASK_0 (0x01C) / VFE_0_IRQ_STATUS_0 (0x02C)
Primary interrupt mask and status.
```
Bit  0: CAMIF_SOF              - Start of frame from CAMIF
Bit  1: CAMIF_EOF              - End of frame from CAMIF
Bit  2: CAMIF_SOL              - Start of line
Bit  3: CAMIF_EOL              - End of line
Bit  4: CAMIF_EPOCH1           - Epoch 1 line reached
Bit  5: REG_UPDATE             - Register update acknowledged
Bit  6: RESET_ACK              - Reset complete (also in STATUS_1 bit 22)
Bit  7: (reserved)

Bit  8: IMAGE_MASTER_0_PING_PONG  - WM0 buffer toggle
Bit  9: IMAGE_MASTER_1_PING_PONG  - WM1 buffer toggle
Bit 10: IMAGE_MASTER_2_PING_PONG  - WM2 buffer toggle
Bit 11: IMAGE_MASTER_3_PING_PONG  - WM3 buffer toggle
Bit 12: IMAGE_MASTER_4_PING_PONG  - WM4 buffer toggle
Bit 13: IMAGE_MASTER_5_PING_PONG  - WM5 buffer toggle
Bit 14: IMAGE_MASTER_6_PING_PONG  - WM6 buffer toggle

Bit 21: IMAGE_COMPOSITE_DONE_0    - Composite group 0 complete
Bit 22: IMAGE_COMPOSITE_DONE_1    - Composite group 1 complete
Bit 23: IMAGE_COMPOSITE_DONE_2    - Composite group 2 complete
Bit 24: STATS_COMPOSITE_DONE      - Statistics complete
Bit 31: RESET_ACK                 - Reset acknowledgement

webOS mask: 0x00E00127 (SOF, EOF, REG_UPDATE, RESET, COMPOSITE_DONE_0/1/2)
```

### VFE_0_IRQ_MASK_1 (0x020) / VFE_0_IRQ_STATUS_1 (0x030)
Secondary interrupt mask and status (errors and control).
```
Bit  0: CAMIF_ERROR            - CAMIF overflow/underflow error
Bit  6: VIOLATION              - VFE violation (NOTE: SHR uses bit 7)

Bit  7: IMAGE_MASTER_0_BUS_OVERFLOW
Bit  8: IMAGE_MASTER_1_BUS_OVERFLOW
Bit  9: IMAGE_MASTER_2_BUS_OVERFLOW
Bit 10: IMAGE_MASTER_3_BUS_OVERFLOW
Bit 11: IMAGE_MASTER_4_BUS_OVERFLOW
Bit 12: IMAGE_MASTER_5_BUS_OVERFLOW
Bit 13: IMAGE_MASTER_6_BUS_OVERFLOW

Bit 22: RESET_ACK              - VFE31-specific reset ack location
Bit 23: BUS_BDG_HALT_ACK       - Bus bridge halt acknowledged

webOS mask: 0x00C00040 (VIOLATION, RESET_ACK, HALT_ACK)
```

### VFE_0_IRQ_CLEAR_0 (0x024) / VFE_0_IRQ_CLEAR_1 (0x028) - Write Only
Write 1 to clear corresponding status bits.

### VFE_0_IRQ_COMPOSITE_MASK_0 (0x034)
Composite interrupt grouping configuration.
```
[7:0]   COMP0_MASK  - Write masters in composite group 0
[15:8]  COMP1_MASK  - Write masters in composite group 1
[23:16] COMP2_MASK  - Write masters in composite group 2
[31:24] (reserved)

WM bit mapping within each group:
  Bit 0 = WM0, Bit 1 = WM1, Bit 2 = WM2, etc.

Common configurations:
  0x00220011 = Preview+Video mode (COMP0: WM0+WM4, COMP2: WM1+WM5)
  0x00020001 = Preview only (COMP0: WM0, COMP2: WM1)
  0x00000003 = Both preview WMs in COMP0 (WM0+WM1)
```

### VFE_0_VIOLATION_STATUS (0x048) - Read Only
Detailed violation information when VIOLATION IRQ fires.

---

## 3. Bus/AXI/XBAR Configuration (0x038 - 0x04C)

**Important VFE31 Architecture Note:**
VFE31 uses a single 188-byte AXI output configuration block starting at 0x038:
- webOS header: `V31_AXI_OUT_OFF = 0x00000038`, `V31_AXI_OUT_LEN = 188`
- This block includes BUS_CMD, bus config, XBAR, and WM registers
- Unlike newer VFEs, there is no standalone BUS_CFG register

### VFE_0_BUS_CMD (0x038) - Write Only
Write master reload command.
```
Bit n: Reload WM n configuration from shadow registers

Example: 0x3F = Reload WM0-WM5
         0x33 = Reload WM0, WM1, WM4, WM5 (preview + video semi-planar)
```

### VFE_0_BUS_CFG (0x03C)
Bus configuration - path enables and raw pixel settings.

**WARNING:** This is part of the V31_AXI_OUT block, NOT a standalone register.
webOS writes this as part of a 188-byte memcpy from userspace via VFE_CMD_AXI_OUT_CFG.
```
[3:0]   Base config (webOS sets 0x1, exact function unknown)

Bit  4: ENC_Y_WR_PATH_EN      - Encoder Y write path enable
Bit  5: ENC_CBCR_WR_PATH_EN   - Encoder CbCr write path enable
Bit  6: VIEW_Y_WR_PATH_EN     - Viewfinder Y write path enable
Bit  7: VIEW_CBCR_WR_PATH_EN  - Viewfinder CbCr write path enable

[9:8]   RAW_PIXEL_DATA_SIZE (enum VFE_RAW_PIXEL_DATA_SIZE):
        0 = 8-bit raw
        1 = 10-bit raw
        2 = 12-bit raw

[11:10] RAW_WR_PATH_SEL (enum VFE_RAW_WR_PATH_SEL):
        0 = VFE_RAW_OUTPUT_DISABLED
        1 = VFE_RAW_OUTPUT_ENC_CBCR_PATH - Raw to encoder CbCr
        2 = VFE_RAW_OUTPUT_VIEW_CBCR_PATH - Raw to viewfinder CbCr

[31:12] Timing/strobe config (0x02AAA in webOS)
        Exact bit definitions unknown. Likely AXI burst/strobe timing.

webOS values:
  0x02AAA771 = YUV mode (all Y/CbCr paths enabled, 8-bit)
  0x02AAA775 = RAW 10-bit mode
  0x02AAA779 = RAW 12-bit mode
```

### VFE_0_BUS_XBAR_CFG0 (0x040)
AXI output mode selection (enum VFE_AXI_OUTPUT_MODE).
```
[31:0] AXI_OUTPUT_MODE:
       0x00 = VFE_AXI_OUTPUT_MODE_Output1 (WM0 only)
       0x01 = VFE_AXI_OUTPUT_MODE_Output1AndOutput2 (WM0/1 + WM4/5)
       0x02 = VFE_AXI_OUTPUT_MODE_Output2 (WM4/5 only, legacy)
       0x03 = VFE_AXI_OUTPUT_MODE_CAMIFToAXIViaOutput2 (Raw bypass)
       0x04 = VFE_AXI_OUTPUT_MODE_Output2AndCAMIFToAXIViaOutput1
       0x05 = VFE_AXI_OUTPUT_MODE_Output1AndCAMIFToAXIViaOutput2

webOS uses: 0x01 (OUTPUT_1_AND_3 in comments, which enables both paths)

NOTE: VFE31 uses global XBAR routing, NOT per-WM configuration like VFE41+.
      The mode here affects which Write Master pairs are active.
```

### VFE_0_BUS_XBAR_CFG1 (0x044)
Data routing configuration - CRITICAL for correct semi-planar output.
```
[3:0]   Y_ROUTING (confirmed by empirical testing):
        0x3 = Y to WM0 only (preview path)
        0xB = Y to WM0 + WM4 (preview + video paths)

[7:4]   CBCR_ROUTING (confirmed by empirical testing):
        0x0 = DISABLED - CbCr not routed (breaks NV12/NV16!)
        0x1 = CbCr to WM1 only (preview CbCr)
        0x9 = CbCr to WM1 + WM5 (preview + video CbCr)

[15:8]  ISP_PATH_CFG (bits 9, 11, 12 typically set):
        0x1A = Standard ISP processed output
        Exact bit meanings undocumented.

Common configurations:
  0x1A03 = BROKEN - Y to WM0, CbCr DISABLED (Qualcomm default, fails NV16)
  0x1A13 = Preview only - Y to WM0, CbCr to WM1
  0x1A1B = webOS VIDEO mode - Y to WM0+WM4, CbCr to WM1 (NOT WM5!)
  0x1A9B = Full dual - Y to WM0+WM4, CbCr to WM1+WM5

webOS uses: 0x1A1B for both preview and video modes

CRITICAL BUG FOUND: Original Qualcomm camss driver used 0x1A03 which
completely disables CbCr routing, causing pure green images for NV12/NV16.

[15:8]  ISP_PATH_CFG = 0x1A (standard ISP processed output)
        Bits 9, 11, 12 set - exact meaning unknown

Common configurations:
  0x1A03 = BROKEN - Y to WM0, CbCr disabled (Qualcomm default)
  0x1A13 = Preview only - Y to WM0, CbCr to WM1
  0x1A1B = Video capable - Y to WM0+WM4, CbCr to WM1
  0x1A9B = Full dual - Y to WM0+WM4, CbCr to WM1+WM5

CRITICAL: Original Qualcomm code used 0x1A03 which breaks NV16 output!
          Must use 0x1A1B or 0x1A9B for working semi-planar formats.
```

---

## 4. Write Master (WM) Registers

### Register Layout
Each write master has 6 registers at stride 0x18:
```
WM0: 0x04C - 0x060
WM1: 0x064 - 0x078
WM2: 0x07C - 0x090
WM3: 0x094 - 0x0A8
WM4: 0x0AC - 0x0C0
WM5: 0x0C4 - 0x0D8
WM6: 0x0DC - 0x0F0

Formula: base = 0x04C + (wm_index * 0x18)
```

### WM_WR_CFG (base + 0x00)
Write master enable.
```
Bit 0: ENABLE - Write master enable

NOTE: VFE31 does NOT have frame_based mode bit like later versions.
      webOS writes 0x1 (enable only).
```

### WM_WR_PING_ADDR (base + 0x04)
Ping buffer physical address (32-bit aligned).

### WM_WR_PONG_ADDR (base + 0x08)
Pong buffer physical address (32-bit aligned).

### WM_WR_ADDR_CFG (base + 0x0C)
Address configuration - burst length and line count.
```
[15:0]  BURST_WORDS = (bytes_per_line / 4) - 17
        Y WM: (output_stride / 4) - 17
        CbCr WM: (width / 4) - 9  (chroma horizontally downsampled)

        Examples @ 640x480:
          Y burst: (640 / 4) - 17 = 143 (if compact) or (1280/4)-17=303 (UYVY)
          CbCr burst: (640 / 4) - 9 = 151

[31:16] LINES - DMA line count / timing control
        This field has DIFFERENT meanings for Y vs CbCr Write Masters:

        Y WMs (WM0 for PIX Y, WM1 for VIDEO Y):
          PIX Y (WM0):   lines = 0 (relies on IMAGE_SIZE for height)
          VIDEO Y (WM1): lines = height - 24 (timing offset)

        CbCr WMs (WM4 for PIX CbCr, WM5 for VIDEO CbCr):
          CbCr:          lines = cbcr_height + 64 (DMA headroom)

        The +64 provides pipeline flush headroom. Sony Nozomi decompilation
        confirms this pattern: "(short)iVar15 + 0x40U" (0x40 = 64 decimal).

webOS examples @ 640x480 NV12 (cbcr_height=240):
  WM0 (PIX Y):    0x0000012F → burst=303, lines=0
  WM1 (VIDEO Y):  0x01C8012F → burst=303, lines=456 (480-24)
  WM4 (PIX CbCr): 0x01300097 → burst=151, lines=304 (240+64)
```

### WM_WR_UB_CFG (base + 0x10)
Micro-block / FIFO configuration.
```
[15:0]  HEIGHT_MINUS_1 = height - 1
        Example: 480 - 1 = 479 = 0x1DF

[31:16] UB_DEPTH = (bytes_per_line / 32) - 1
        Example: (1280 / 32) - 1 = 39 = 0x27

Combined: 0x002701DF for 640x480 @ 1280 bytes/line
```

### WM_WR_IMAGE_SIZE (base + 0x14)
Image size and stride configuration.
```
[15:0]  SIZE_CFG = ((height - 1) << 4) | 0x2
        Example: ((480 - 1) << 4) | 2 = 0x1DF2

[31:16] STRIDE = bytes_per_line / 16
        Example: 1280 / 16 = 80 = 0x50

Combined: 0x00501DF2 for 640x480 @ 1280 bytes/line
```

---

## 5. CAMIF Configuration (0x1D8 - 0x204)

### VFE_0_AXI_CMD (0x1D8)
AXI halt command.
```
Bit 0: HALT - Request AXI halt
```

### VFE_0_AXI_STATUS (0x1DC) - Read Only
AXI status.
```
Bit 0: HALT_ACK - AXI halt acknowledged
```

### VFE_0_CAMIF_CMD (0x1E0)
CAMIF command register.
```
0x0 = STOP_AT_FRAME   - Stop at frame boundary
0x1 = START           - Start CAMIF (enable bit 0 only)
0x2 = STOP_IMMEDIATE  - Stop immediately
0x4 = CLEAR_STATUS    - Clear CAMIF status

NOTE: webOS uses 0x1 for start, NOT 0x5 as some headers suggest.
```

### VFE_0_CAMIF_CFG (0x1E4)
CAMIF configuration.
```
[5:0]   EFS_CODES      - Embedded frame sync codes
Bit  6: VFE_OUTPUT_EN  - Enable VFE pipeline output

webOS: Sets bit 6 (0x40) to enable output path
```

### VFE_0_CAMIF_FRAME_CFG (0x1E8)
Frame dimensions.
```
[13:0]  PIXELS_PER_LINE - Pixels per line (width)
[29:16] LINES_PER_FRAME - Lines per frame (height)

Example: 1280x1024 = 0x04000500
```

### VFE_0_CAMIF_WINDOW_WIDTH_CFG (0x1EC)
Horizontal window configuration.
```
[13:0]  FIRST_PIXEL - First pixel (0-indexed)
[29:16] LAST_PIXEL  - Last pixel (width - 1)

Example: 1280 wide = 0x04FF0000 (first=0, last=1279)
```

### VFE_0_CAMIF_WINDOW_HEIGHT_CFG (0x1F0)
Vertical window configuration.
```
[13:0]  FIRST_LINE - First line (0-indexed)
[29:16] LAST_LINE  - Last line (height - 1)

Example: 1024 high = 0x03FF0000 (first=0, last=1023)
```

### VFE_0_CAMIF_SUBSAMPLE_CFG_0 (0x1F4)
Subsampling configuration.
```
[15:0]  PIXEL_SKIP - Pixel skip pattern (0xFFFF = no skip)
[31:16] LINE_SKIP  - Line skip pattern (0xFFFF = no skip)
```

### VFE_0_CAMIF_STATUS (0x204) - Read Only
CAMIF status.
```
Bit 31: BUSY - CAMIF is active
[13:0]  CURRENT_PIXEL
[29:16] CURRENT_LINE
```

---

## 6. DEMUX Configuration (0x284 - 0x294)

The DEMUX module separates interleaved YUV (UYVY) into separate Y and CbCr streams.

### VFE_0_DEMUX_CFG (0x284)
Demux period configuration.
```
[1:0] PERIOD = 0x3 (process every 4 pixels for Bayer CFA)

webOS: 0x3
```

### VFE_0_DEMUX_GAIN_0 (0x288)
Channel 0 gain (typically Y channel).
```
[7:0]   CH0_EVEN_GAIN = 0x80 (unity gain = 1.0)
[31:16] CH0_ODD_GAIN  = 0x80 (unity gain = 1.0)

webOS: 0x00800080
```

### VFE_0_DEMUX_GAIN_1 (0x28C)
Channels 1 and 2 gain (Cb and Cr).
```
[7:0]   CH1_GAIN = 0x80 (unity gain)
[31:16] CH2_GAIN = 0x80 (unity gain)

webOS: 0x00800080
```

### VFE_0_DEMUX_EVEN_CFG (0x290)
YUV byte order mapping - CRITICAL for correct color output.
```
[7:0]   EVEN_CFG - Even pixel byte routing
[15:8]  ODD_CFG  - Odd pixel byte routing

Byte codes (speculative based on testing):
  0xC9 = Y from position 1
  0xCA = Cb from position 0
  0xAC = Cr from position 2
  0xA9 = Cr from position 1

Format mappings:
  UYVY: 0xCAC9 (even=0xC9, odd=0xCA)
  VYUY: 0xACC9 (swapped UV)
  YUYV: 0xC9CA
  YVYU: 0xC9AC
```

### VFE_0_DEMUX_ODD_CFG (0x294)
Odd line configuration (for interlaced or Bayer patterns).

---

## 7. Pipeline Processing Registers

### VFE_0_REG_UPDATE_CMD (0x260) - Write Only
Trigger configuration update from shadow to active registers.
```
Bit 0: UPDATE - Write 1 to latch shadow registers

IMPORTANT: All configuration writes go to shadow registers.
           REG_UPDATE_CMD latches them into active configuration.
           VFE fires REG_UPDATE IRQ when latch completes.
```

### VFE_0_SCALE_ENC_Y_CFG (0x368)
Encoder Y scaler configuration (typically 0 for no scaling).

### VFE_0_SCALE_ENC_CBCR_CFG (0x36C)
Encoder CbCr scaler configuration.

### VFE_0_CROP_ENC_Y_WIDTH (0x378)
Encoder Y crop width.
```
[13:0]  FIRST_PIXEL
[29:16] LAST_PIXEL
```

### VFE_0_CROP_ENC_Y_HEIGHT (0x37C)
Encoder Y crop height.
```
[13:0]  FIRST_LINE
[29:16] LAST_LINE
```

### VFE_0_CROP_ENC_CBCR_WIDTH (0x380)
Encoder CbCr crop width.

### VFE_0_CROP_ENC_CBCR_HEIGHT (0x384)
Encoder CbCr crop height.

---

## 8. Framedrop and Clamp (0x504 - 0x528)

### Framedrop Registers
Control frame dropping for encoder and viewfinder paths.
```
0x504: FRAMEDROP_ENC_Y_CFG        - Encoder Y framedrop enable
0x508: FRAMEDROP_ENC_CBCR_CFG     - Encoder CbCr framedrop
0x50C: FRAMEDROP_ENC_Y_PATTERN    - Y drop pattern (1=keep)
0x510: FRAMEDROP_ENC_CBCR_PATTERN - CbCr drop pattern
0x514: FRAMEDROP_VIEW_Y_CFG       - Viewfinder Y framedrop
0x518: FRAMEDROP_VIEW_CBCR_CFG    - Viewfinder CbCr framedrop
0x51C: FRAMEDROP_VIEW_Y_PATTERN   - Viewfinder Y pattern
0x520: FRAMEDROP_VIEW_CBCR_PATTERN- Viewfinder CbCr pattern

webOS: All patterns = 0xFFFFFFFF (keep all frames)
```

### VFE_0_CLAMP_ENC_MAX_CFG (0x524)
Output maximum clamp values.
```
[7:0]   Y_MAX  = 0xFF
[15:8]  CB_MAX = 0xFF
[23:16] CR_MAX = 0xFF

webOS: 0x00FFFFFF (no clamping)
```

### VFE_0_CLAMP_ENC_MIN_CFG (0x528)
Output minimum clamp values.
```
[7:0]   Y_MIN  = 0x00
[15:8]  CB_MIN = 0x00
[23:16] CR_MIN = 0x00

webOS: 0x00000000 (no clamping)
```

---

## 9. Ping-Pong Status (0x180)

### VFE_0_BUS_PING_PONG_STATUS (0x180) - Read Only
Current active buffer for each write master.
```
Bit n: WM n buffer status
       0 = Currently writing to PING buffer
       1 = Currently writing to PONG buffer

Use this to determine which buffer is safe to read/process.
Toggle indicates data flow is occurring.
```

---

## 10. webOS Register Sequences

### Preview Mode Initialization (640x480 UYVY -> NV16)
```
1. VFE_0_GLOBAL_RESET_CMD = 0x3FF (full reset)
2. Wait for IRQ_STATUS_1 bit 22 (RESET_ACK)
3. VFE_0_CGC_OVERRIDE = 0xFFFFF
4. VFE_0_MODULE_CFG = 0x00000004 (DEMUX enable)
5. VFE_0_CORE_CFG = 0x00000046 (CBYCRY + bit 6)
6. VFE_0_DEMUX_GAIN_0 = 0x00800080
7. VFE_0_DEMUX_GAIN_1 = 0x00800080
8. Configure WM0/WM1 registers
9. VFE_0_BUS_CFG = 0x02AAA771
10. VFE_0_BUS_XBAR_CFG0 = 0x01
11. VFE_0_BUS_XBAR_CFG1 = 0x1A1B
12. VFE_0_IRQ_MASK_0 = 0x00E00127
13. VFE_0_IRQ_MASK_1 = 0x00C00040
14. VFE_0_IRQ_COMPOSITE_MASK_0 = 0x00020001
15. Configure CAMIF registers
16. Enable WM0 and WM1 (WR_CFG = 0x1)
17. VFE_0_BUS_CMD = 0x3 (reload WM0, WM1)
18. VFE_0_REG_UPDATE_CMD = 0x1
19. VFE_0_CAMIF_CMD = 0x1 (start)
```

### webOS WM Configuration @ 640x480 NV12

**WM Assignment (corrected 2026-04-17):**
- WM0 = PIX Y (preview Y plane)
- WM1 = VIDEO Y (video recording Y plane)
- WM4 = PIX CbCr (preview CbCr plane)
- WM5 = VIDEO CbCr (video recording CbCr - NOT enabled by webOS!)

```
WM0 (PIX Y):
  WR_CFG       = 0x00000001
  WR_PING_ADDR = <buffer_phys>
  WR_PONG_ADDR = <buffer_phys + frame_size>
  WR_ADDR_CFG  = 0x0000012F (burst=303, lines=0)
  WR_UB_CFG    = 0x002701DF (ub=39, height=479)
  WR_IMAGE_SIZE= 0x00501DF2 (stride=80, size=0x1DF2)

WM1 (VIDEO Y) - only active in video recording:
  WR_CFG       = 0x00000001
  WR_ADDR_CFG  = 0x01C8012F (burst=303, lines=456=height-24)
  WR_UB_CFG    = 0x002701DF (same as WM0)
  WR_IMAGE_SIZE= 0x00501DF2 (same as WM0)

WM4 (PIX CbCr) - for NV12/NV16 semi-planar output:
  WR_CFG       = 0x00000001
  WR_PING_ADDR = <buffer_phys + y_plane_size>
  WR_PONG_ADDR = <buffer_phys + frame_size + y_plane_size>
  WR_ADDR_CFG  = 0x01300097 (burst=151, lines=304=cbcr_height+64)
  WR_UB_CFG    = 0x002700EF (ub=39, height=239 for NV12)
  WR_IMAGE_SIZE= 0x005000F2 (stride=80, cbcr height in size field)
```

---

## 11. Known Issues and Quirks

### XBAR_CFG1 CbCr Routing
The original Qualcomm/mainline driver used XBAR_CFG1 = 0x1A03 which disables
CbCr routing to WM1. This causes NV16 output to have no UV data (green image).
Must use 0x1A1B or 0x1A9B for working semi-planar output.

### WM burst_lines Configuration (ADDR_CFG upper 16 bits)

**Y Write Masters (WM0/WM4):**
- PIX Y (WM0): lines = 0 (uses IMAGE_SIZE height)
- VIDEO Y (WM1 in dual mode): lines = height - 24

**CbCr Write Masters (WM1/WM5 for semi-planar, or WM4 when used for CbCr):**
- CbCr: lines = cbcr_height + 64 (provides DMA headroom)

Evidence from webOS register dumps (640x480 NV12):
- WM0 (PIX Y): ADDR_CFG = 0x0000012F → lines=0, burst=303
- WM1 (VIDEO Y): ADDR_CFG = 0x01C8012F → lines=456 (480-24), burst=303
- WM4 (PIX CbCr): ADDR_CFG = 0x01300097 → lines=304 (240+64), burst=151

Sony Nozomi decompilation confirms +64 pattern: `(short)iVar15 + 0x40U`

**CORRECTION (2026-04-17):** Previous documentation incorrectly stated CbCr
uses height-24. This was a misreading of WM1 (VIDEO Y) data as CbCr. The
actual CbCr WMs use cbcr_height + 64 for DMA pipeline headroom.

### IRQ_STATUS_1 RESET_ACK Location
VFE31 places RESET_ACK in IRQ_STATUS_1 bit 22, which differs from later VFE
versions. The common driver code checks bit 31 of IRQ_STATUS_0 which doesn't
work for VFE31.

### Shadow Register Timing
All writes go to shadow registers. Must issue REG_UPDATE_CMD to latch values
into active configuration. WM enable bits must be written BEFORE REG_UPDATE,
not after, or they won't take effect until the next REG_UPDATE cycle.

### CAMIF Window vs Output Resolution
CAMIF window can be larger than output resolution. webOS uses 1280x1279 CAMIF
window but outputs 640x480 via the scaler. The CAMIF_ERROR status shows the
CAMIF window dimensions, not the output dimensions.

---

---

## 12. Additional Registers (from Mako/G2 kernels)

### Bus Performance Monitor Registers
```
0x184: VFE_BUS_OPERATION_STATUS    - Bus operation status
0x188: VFE_BUS_PM_CMD              - Performance monitor command
0x18C: VFE_BUS_PM_CFG              - Performance monitor configuration
0x190: VFE_BUS_IMAGE_MASTER_0_WR_PM_STATS_0
0x194: VFE_BUS_IMAGE_MASTER_0_WR_PM_STATS_1
```

### Statistics Buffer Registers
VFE31 has dedicated write paths for statistics data (AEC, AF, AWB, etc.)
Each statistics type has its own ping/pong buffers and address configuration.
These are separate from the image write masters (WM0-WM6).
```
AEC (Auto-Exposure Control):
  0x0F4: VFE_BUS_STATS_AEC_WR_PING_ADDR
  0x0F8: VFE_BUS_STATS_AEC_WR_PONG_ADDR
  0x0FC: VFE_BUS_STATS_AEC_WR_ADDR_CFG

AF (Auto-Focus):
  0x100: VFE_BUS_STATS_AF_WR_PING_ADDR
  0x104: VFE_BUS_STATS_AF_WR_PONG_ADDR
  0x108: VFE_BUS_STATS_AF_WR_ADDR_CFG

AWB (Auto White Balance):
  0x10C: VFE_BUS_STATS_AWB_WR_PING_ADDR
  0x110: VFE_BUS_STATS_AWB_WR_PONG_ADDR
  0x114: VFE_BUS_STATS_AWB_WR_ADDR_CFG

RS (Row Sum):
  0x118: VFE_BUS_STATS_RS_WR_PING_ADDR
  0x11C: VFE_BUS_STATS_RS_WR_PONG_ADDR
  0x120: VFE_BUS_STATS_RS_WR_ADDR_CFG

CS (Column Sum):
  0x124: VFE_BUS_STATS_CS_WR_PING_ADDR
  0x128: VFE_BUS_STATS_CS_WR_PONG_ADDR
  0x12C: VFE_BUS_STATS_CS_WR_ADDR_CFG

HIST (Histogram):
  0x130: VFE_BUS_STATS_HIST_WR_PING_ADDR
  0x134: VFE_BUS_STATS_HIST_WR_PONG_ADDR
  0x138: VFE_BUS_STATS_HIST_WR_ADDR_CFG

SKIN (Skin Detection):
  0x13C: VFE_BUS_STATS_SKIN_WR_PING_ADDR
  0x140: VFE_BUS_STATS_SKIN_WR_PONG_ADDR
  0x144: VFE_BUS_STATS_SKIN_WR_ADDR_CFG

Composite:
  0x148: VFE_STATS_COMP_GRP_CFG     - Statistics composite group config
```

### Test Pattern Generator Registers
Used for internal testing without a real sensor.
```
0x158: VFE_TESTGEN_STATUS     - Test generator status
0x15C: VFE_TESTGEN_CFG        - Test generator configuration
0x160: VFE_TESTGEN_SEED_0     - Random seed 0
0x164: VFE_TESTGEN_SEED_1     - Random seed 1
0x168: VFE_TESTGEN_SEED_2     - Random seed 2
0x16C: VFE_TESTGEN_SEED_3     - Random seed 3
0x170: VFE_TESTGEN_DIMS       - Test image dimensions
0x174: VFE_TESTGEN_START_PIXEL - Start pixel configuration
```

### MISR (Multiple Input Signature Register)
Used for debug and verification.
```
0x178: VFE_BUS_MISR_CFG       - MISR configuration
0x17C: VFE_BUS_MISR_VALUE     - MISR computed value
```

### Additional Module Offsets
```
0x298: V31_DEMOSAIC_0_OFF   (length: 4)   - Demosaic general config
0x29C: V31_DEMOSAIC_2_OFF   (length: 8)   - BPC config
0x2A4: V31_DEMOSAIC_1_OFF   (length: 180) - ABF config
0x360: V31_FOV_OFF          (length: 8)   - Field of view
0x368: V31_MAIN_SCALER_OFF  (length: 28)  - Main scaler config
0x384: V31_WB_OFF           (length: 4)   - White balance
0x388: V31_COLOR_COR_OFF    (length: 52)  - Color correction
0x3BC: V31_GAMMA_CFG_OFF    (length: 4)   - Gamma configuration
0x3C0: V31_LA_OFF           (length: 4)   - Luma adaptation
0x3C4: V31_CHROMA_EN_OFF    (length: 36)  - Chroma enhancement
0x3E8: V31_CHROMA_SUP_OFF   (length: 12)  - Chroma suppression
0x3F4: V31_MCE_OFF          (length: 36)  - Memory color enhancement
0x418: V31_SCE_OFF          (length: 136) - Skin color enhancement
0x4A0: V31_ASF_OFF          (length: 48)  - Adaptive spatial filter
0x4D0: V31_S2Y_OFF          (length: 20)  - Scaler 2 Y
0x4E4: V31_S2CbCr_OFF       (length: 20)  - Scaler 2 CbCr
0x4F8: V31_CHROMA_SUBS_OFF  (length: 12)  - Chroma subsampling
0x52C: VFE_REALIGN_BUF                    - Buffer realignment
0x530: VFE_STATS_CFG                      - Statistics configuration
0x534: V31_STATS_AE_OFF     (length: 8)   - Auto-exposure stats
0x53C: V31_STATS_AF_OFF     (length: 16)  - Auto-focus stats
0x54C: V31_STATS_AWB_OFF    (length: 32)  - Auto white balance stats
0x554: VFE_STATS_AWB_SGW_CFG              - AWB simple gray world config
0x56C: V31_STATS_RS_OFF     (length: 8)   - Row sum stats
0x574: V31_STATS_CS_OFF     (length: 8)   - Column sum stats
0x57C: V31_STATS_IHIST_OFF  (length: 8)   - Image histogram stats
0x598: VFE_DMI_CFG                        - DMI configuration
0x59C: VFE_DMI_ADDR                       - DMI address
0x5A4: VFE_DMI_DATA_LO                    - DMI data low
0x600: VFE_AXI_CFG                        - AXI configuration
```

### Timer Registers
```
0x20C: V31_SYNC_TIMER_OFF          (length: 28)
0x234: V31_SYNC_TIMER_POLARITY_OFF
0x238: V31_ASYNC_TIMER_OFF         (length: 28)
0x25C: V31_TIMER_SELECT_OFF
```

### Black Level and Roll-off
```
0x264: V31_BLACK_LEVEL_OFF  (length: 16)
0x274: V31_ROLL_OFF_CFG_OFF (length: 16)
```

---

## 13. Complete IRQ Bit Reference

### IRQ_STATUS_0 Bits (0x02C)
```
Bit  0: CAMIF_SOF              - Start of frame
Bit  1: (reserved)
Bit  2: CAMIF_EOF              - End of frame (NOTE: bit 2, not bit 1!)
Bit  3: (reserved)
Bit  4: EPOCH_IRQ_0
Bit  5: REG_UPDATE             - Register update complete
Bit  6: RESET_ACK              - Reset acknowledged (also in STATUS_1)
Bit  7: (reserved)
Bit  8: IMAGE_MASTER_0_PING_PONG (WM0)
Bit  9: IMAGE_MASTER_1_PING_PONG (WM1)
Bit 10: IMAGE_MASTER_2_PING_PONG (WM2)
Bit 11: IMAGE_MASTER_3_PING_PONG (WM3)
Bit 12: IMAGE_MASTER_4_PING_PONG (WM4)
Bit 13: IMAGE_MASTER_5_PING_PONG (WM5)
Bit 14: IMAGE_MASTER_6_PING_PONG (WM6)
Bit 15: STATS_AWB
Bit 16: STATS_RS
Bit 17: STATS_CS
Bit 18: STATS_IHIST
Bit 19-20: (reserved)
Bit 21: IMAGE_COMPOSITE_DONE_0 - Group 0 complete
Bit 22: IMAGE_COMPOSITE_DONE_1 - Group 1 complete
Bit 23: IMAGE_COMPOSITE_DONE_2 - Group 2 complete
Bit 24: STATS_COMPOSITE        - Statistics complete
Bit 25: SYNC_TIMER0
Bit 26: SYNC_TIMER1
Bit 27: SYNC_TIMER2
Bit 28: ASYNC_TIMER0
Bit 29: ASYNC_TIMER1
Bit 30: ASYNC_TIMER2
Bit 31: ASYNC_TIMER3
```

### IRQ_STATUS_1 Bits (0x030) - Error Interrupts
```
Bit  0: CAMIF_ERROR
Bit  1: STATS_CS_OVERFLOW
Bit  2: STATS_IHIST_OVERFLOW
Bit  3: REALIGN_BUF_Y_OVERFLOW
Bit  4: REALIGN_BUF_CB_OVERFLOW
Bit  5: REALIGN_BUF_CR_OVERFLOW
Bit  6: VIOLATION
Bit  7: IMAGE_MASTER_0_BUS_OVERFLOW (WM0)
Bit  8: IMAGE_MASTER_1_BUS_OVERFLOW (WM1)
Bit  9: IMAGE_MASTER_2_BUS_OVERFLOW (WM2)
Bit 10: IMAGE_MASTER_3_BUS_OVERFLOW (WM3)
Bit 11: IMAGE_MASTER_4_BUS_OVERFLOW (WM4)
Bit 12: IMAGE_MASTER_5_BUS_OVERFLOW (WM5)
Bit 13: IMAGE_MASTER_6_BUS_OVERFLOW (WM6)
Bit 14: STATS_AE_BUS_OVERFLOW
Bit 15: STATS_AF_BUS_OVERFLOW
Bit 16: STATS_AWB_BUS_OVERFLOW
Bit 17: STATS_RS_BUS_OVERFLOW
Bit 18: STATS_CS_BUS_OVERFLOW
Bit 19: STATS_IHIST_BUS_OVERFLOW
Bit 20: STATS_SKIN_BUS_OVERFLOW
Bit 21: AXI_ERROR
Bit 22: RESET_ACK              - VFE31-specific reset ack location!
Bit 23: BUS_BDG_HALT_ACK       - AXI halt acknowledged
```

---

## 14. Processing Module Enable Masks (MODULE_CFG 0x010)

```
Bit  2: DEMUX_EN
Bit  3: CHROMA_UPSAMPLE_EN
Bit  5: AE_ENABLE
Bit  6: AF_ENABLE
Bit  7: AWB_ENABLE
Bit  8: RS_ENABLE
Bit  9: CS_ENABLE
Bit 15: IHIST_ENABLE
Bit 23: SCALE_ENC_EN
Bit 27: CROP_ENC_EN

Combined masks:
  RS_CS_ENABLE_MASK  = 0x00000300 (bits 8,9)
  STATS_ENABLE_MASK  = 0x000483E0 (bits 18,15,9,8,7,6,5)
```

---

## 15. Hardware Constants

```
VFE_AXI_OUTPUT_BURST_LENGTH      = 4
VFE_MAX_NUM_FRAGMENTS_PER_FRAME  = 4
VFE_AXI_OUTPUT_CFG_FRAME_COUNT   = 3
VFE31_ROLL_OFF_INIT_TABLE_SIZE   = 13
VFE31_ROLL_OFF_DELTA_TABLE_SIZE  = 208
LENS_ROLL_OFF_DELTA_TABLE_OFFSET = 32
VFE31_GAMMA_NUM_ENTRIES          = 64
VFE31_LA_TABLE_LENGTH            = 64
VFE31_HIST_TABLE_LENGTH          = 256
VFE_GAMMA_TABLE_LENGTH           = 256
VFE_STATS_BUFFER_COUNT           = 3
VFE_AXI_CH_INF_LEN               = 32
VFE_AXI_CFG_LEN                  = 47
V31_OPERATION_CFG_LEN            = 32
```

---

## 16. DMI (Direct Memory Interface) RAM Selection

Used with VFE_DMI_CFG (0x598) for table access:
```
NO_MEM_SELECTED           = 0x0
ROLLOFF_RAM               = 0x1
RGBLUT_RAM_CH0_BANK0      = 0x2
RGBLUT_RAM_CH0_BANK1      = 0x3
RGBLUT_RAM_CH1_BANK0      = 0x4
RGBLUT_RAM_CH1_BANK1      = 0x5
RGBLUT_RAM_CH2_BANK0      = 0x6
RGBLUT_RAM_CH2_BANK1      = 0x7
STATS_HIST_RAM            = 0x8
RGBLUT_CHX_BANK0          = 0x9
RGBLUT_CHX_BANK1          = 0xA
LUMA_ADAPT_LUT_RAM_BANK0  = 0xB
LUMA_ADAPT_LUT_RAM_BANK1  = 0xC

VFE_DMI_CFG_DEFAULT = 0x00000100
```

---

## 17. ISP Processing Modules

This section documents the Image Signal Processing (ISP) modules in the VFE31 pipeline.
Information derived from webOS kernel headers (msm_vfe31.h, msm_vfe8x_proc.h) and
decompiled vendor binaries.

### 17.1 Pipeline Overview

The VFE31 ISP pipeline processes data in this order:
```
CAMIF → BLACK_LEVEL → ROLLOFF → DEMUX → DEMOSAIC → WB → COLOR_CORRECT →
GAMMA → LUMA_ADAPT → CHROMA_ENHANCE → CHROMA_SUPPRESS → MCE/SCE →
ASF → SCALE → FOV_CROP → CHROMA_SUBSAMPLE → OUTPUT_CLAMP → WM
```

### 17.2 Black Level Correction (0x264)

Corrects sensor black level offset. Applies per-pixel adjustment based on Bayer position.

**Registers:**
```
V31_BLACK_LEVEL_OFF = 0x264 (length: 16 bytes)
```

**Configuration Fields:**
```c
struct vfe_blacklevel_cfg {
    [8:0]  evenEvenAdjustment;  // 9-bit signed adjustment
    [8:0]  evenOddAdjustment;   // for even row, odd column
    [8:0]  oddEvenAdjustment;   // for odd row, even column
    [8:0]  oddOddAdjustment;    // for odd row, odd column
};
```

**webOS default:** All adjustments = 0 (no correction)

### 17.3 Lens Rolloff / Shading Correction (0x274)

Corrects lens vignetting (darker corners). Uses a 13×10 grid lookup table.

**Registers:**
```
V31_ROLL_OFF_CFG_OFF = 0x274 (length: 16 bytes)
VFE31_ROLL_OFF_INIT_TABLE_SIZE   = 13
VFE31_ROLL_OFF_DELTA_TABLE_SIZE  = 208
LENS_ROLL_OFF_DELTA_TABLE_OFFSET = 32
```

**Configuration Fields:**
```c
struct vfe_rolloff_cfg {
    // Rolloff 0 Config
    [8:0]  gridWidth;    // Grid cell width
    [17:9] gridHeight;   // Grid cell height
    [26:18] yDelta;      // Y delta accumulator init

    // Rolloff 1 Config
    [3:0]  gridX;        // Start grid X position
    [7:4]  gridY;        // Start grid Y position
    [16:8] pixelX;       // Start pixel X within grid
    [28:19] pixelY;      // Start pixel Y within grid

    // Rolloff 2 Config
    [11:0] yDeltaAccum;  // Y delta accumulator
};
```

**Table access:** Via DMI (ROLLOFF_RAM = 0x1)

### 17.4 DEMUX Module (0x284 - 0x294)

Separates interleaved sensor data into color channels. For YUV sensors (like MT9M113),
separates UYVY into Y and CbCr streams.

**Registers:**
```
V31_DEMUX_OFF = 0x284 (length: 20 bytes)
0x284: DEMUX_CFG       - Period configuration
0x288: DEMUX_GAIN_0    - Channel 0 gains
0x28C: DEMUX_GAIN_1    - Channel 1/2 gains
0x290: DEMUX_EVEN_CFG  - Even line byte routing
0x294: DEMUX_ODD_CFG   - Odd line byte routing
```

**Configuration:**
```c
struct vfe_demux_cfg {
    // DEMUX_GAIN_0
    [9:0]  ch0EvenGain;  // 10-bit gain (0x80 = 1.0)
    [25:16] ch0OddGain;

    // DEMUX_GAIN_1
    [9:0]  ch1Gain;      // Cb channel gain
    [25:16] ch2Gain;     // Cr channel gain
};
```

**webOS values for UYVY input:**
```
DEMUX_CFG      = 0x03      (period = 3 for YUV 4-byte pattern)
DEMUX_GAIN_0   = 0x00800080 (unity gain)
DEMUX_GAIN_1   = 0x00800080 (unity gain)
DEMUX_EVEN_CFG = 0xC9CA    (UYVY byte routing)
DEMUX_ODD_CFG  = 0xC9CA    (same for all lines)
```

### 17.5 Demosaic Module (0x298 - 0x358)

Converts Bayer pattern to RGB. Includes ABF (Adaptive Bayer Filtering) and
BPC (Bad Pixel Correction) sub-modules.

**Registers:**
```
V31_DEMOSAIC_0_OFF = 0x298 (length: 4)   - General demosaic config
V31_DEMOSAIC_2_OFF = 0x29C (length: 8)   - BPC config
V31_DEMOSAIC_1_OFF = 0x2A4 (length: 180) - ABF config
```

**Demosaic General Config (0x298):**
```c
struct vfe_demosaic_cfg {
    Bit 0: abfEnable;           // Enable ABF
    Bit 1: badPixelCorrEnable;  // Enable BPC
    Bit 2: forceAbfOn;          // Force ABF always on
    [7:4]  abfShift;            // ABF shift value
    [14:8] fminThreshold;       // 7-bit min threshold
    [22:16] fmaxThreshold;      // 7-bit max threshold
    [30:28] slopeShift;         // 3-bit slope shift
};
```

**BPC Config (0x29C - 0x2A0):**
```c
struct vfe_demosaic_bpc_cfg {
    // 0x29C
    [11:0] blueDiffThreshold;   // Blue difference threshold
    [23:12] redDiffThreshold;   // Red difference threshold

    // 0x2A0
    [11:0] greenDiffThreshold;  // Green difference threshold
};
```

**ABF Config (0x2A4+):**
```c
struct vfe_cmds_demosaic_abf {
    uint8_t  enable;
    uint8_t  forceOn;
    uint8_t  shift;
    uint16_t lpThreshold;  // Low-pass threshold (10-bit)
    uint16_t max;          // Max value (10-bit)
    uint16_t min;          // Min value (10-bit)
    uint8_t  ratio;        // 4-bit ratio
};
```

**NOTE:** For YUV sensors like MT9M113, demosaic is bypassed (DEMUX handles YUV directly).

### 17.6 White Balance (0x384)

Applies per-channel gain to correct color temperature.

**Registers:**
```
V31_WB_OFF = 0x384 (length: 4 bytes)
```

**Configuration:**
```c
struct vfe_wb_cfg {
    [8:0]  ch0Gain;  // 9-bit gain (channel 0 / R)
    [17:9] ch1Gain;  // 9-bit gain (channel 1 / G)
    [26:18] ch2Gain; // 9-bit gain (channel 2 / B)
};
```

**Command struct:**
```c
struct vfe_cmd_white_balance_config {
    uint8_t  enable;
    uint16_t ch2Gain;  // 0x100 = 1.0
    uint16_t ch1Gain;
    uint16_t ch0Gain;
};
```

### 17.7 Color Correction (0x388)

Applies 3×3 color correction matrix plus offsets for accurate color reproduction.

**Registers:**
```
V31_COLOR_COR_OFF = 0x388 (length: 52 bytes)
```

**Configuration:**
```c
struct vfe_color_correction_cfg {
    // 9 matrix coefficients (12-bit signed each)
    [11:0] c0, c1, c2;  // Row 0: affects R output
    [11:0] c3, c4, c5;  // Row 1: affects G output
    [11:0] c6, c7, c8;  // Row 2: affects B output

    // 3 offset values (11-bit signed each)
    [10:0] k0;  // R offset
    [10:0] k1;  // G offset
    [10:0] k2;  // B offset

    // Q-factor for coefficient precision
    [1:0] coefQFactor;  // 0=Q7, 1=Q8, 2=Q9, 3=Q10
};
```

**Matrix operation:**
```
R_out = c0*R + c1*G + c2*B + k0
G_out = c3*R + c4*G + c5*B + k1
B_out = c6*R + c7*G + c8*B + k2
```

### 17.8 Gamma / RGB LUT (0x3BC)

Applies gamma correction via lookup tables. Supports independent or shared tables
for R, G, B channels.

**Registers:**
```
V31_GAMMA_CFG_OFF = 0x3BC (length: 4 bytes)
V31_RGB_G_OFF     = 0x3BC (length: 4 bytes)
```

**Configuration:**
```c
struct VFE_GammaLutSelect_ConfigCmdType {
    Bit 0: ch0BankSelect;  // Bank select for channel 0 (R)
    Bit 1: ch1BankSelect;  // Bank select for channel 1 (G)
    Bit 2: ch2BankSelect;  // Bank select for channel 2 (B)
};

enum VFE_RGB_GAMMA_TABLE_SELECT {
    RGB_GAMMA_CH0_SELECTED,
    RGB_GAMMA_CH1_SELECTED,
    RGB_GAMMA_CH2_SELECTED,
    RGB_GAMMA_CH0_CH1_SELECTED,
    RGB_GAMMA_CH0_CH2_SELECTED,
    RGB_GAMMA_CH1_CH2_SELECTED,
    RGB_GAMMA_CH0_CH1_CH2_SELECTED  // All channels share same table
};
```

**Constants:**
```
VFE31_GAMMA_NUM_ENTRIES  = 64
VFE_GAMMA_TABLE_LENGTH   = 256
```

**Table access:** Via DMI with bank selection:
```
RGBLUT_RAM_CH0_BANK0 = 0x2
RGBLUT_RAM_CH0_BANK1 = 0x3
RGBLUT_RAM_CH1_BANK0 = 0x4
RGBLUT_RAM_CH1_BANK1 = 0x5
RGBLUT_RAM_CH2_BANK0 = 0x6
RGBLUT_RAM_CH2_BANK1 = 0x7
RGBLUT_CHX_BANK0     = 0x9  // Shared table
RGBLUT_CHX_BANK1     = 0xA
```

### 17.9 Luma Adaptation (0x3C0)

Adaptive luma enhancement using lookup table.

**Registers:**
```
V31_LA_OFF = 0x3C0 (length: 4 bytes)
V31_LUMA_CFG_OFF = 0x3C0
```

**Configuration:**
```c
struct VFE_LumaAdaptation_ConfigCmdType {
    Bit 0: lutBankSelect;  // Which LUT bank is active
};

struct vfe_cmd_la_config {
    uint8_t enable;
    int16_t table[VFE_LA_TABLE_LENGTH];  // 64 entries
};
```

**Table access:** Via DMI:
```
LUMA_ADAPT_LUT_RAM_BANK0 = 0xB
LUMA_ADAPT_LUT_RAM_BANK1 = 0xC
```

### 17.10 Chroma Enhancement (0x3C4)

Adjusts chroma saturation and hue. Implements 2×2 matrix transform on Cb/Cr.

**Registers:**
```
V31_CHROMA_EN_OFF = 0x3C4 (length: 36 bytes)
```

**Configuration:**
```c
struct vfe_cmd_chroma_enhan_config {
    uint8_t enable;
    // 2×2 matrix coefficients (11-bit signed)
    int16_t am, ap;  // Cb coefficients
    int16_t bm, bp;  // Cr coefficients
    int16_t cm, cp;  // Cross-channel Cb
    int16_t dm, dp;  // Cross-channel Cr

    // DC offsets
    int16_t kcr;     // Cr offset
    int16_t kcb;     // Cb offset

    // RGB to Y conversion
    int16_t RGBtoYConversionV0;  // R coefficient
    int16_t RGBtoYConversionV1;  // G coefficient
    int16_t RGBtoYConversionV2;  // B coefficient
    uint8_t RGBtoYConversionOffset;
};
```

**Register layout:**
```
0x3C4: ap[10:0], am[26:16]
0x3C8: bp[10:0], bm[26:16]
0x3CC: cp[10:0], cm[26:16]
0x3D0: dp[10:0], dm[26:16]
0x3D4: kcr[10:0], kcb[26:16]
```

### 17.11 Chroma Suppression (0x3E8)

Suppresses chroma noise in low-light / dark regions.

**Registers:**
```
V31_CHROMA_SUP_OFF = 0x3E8 (length: 12 bytes)
```

**Configuration:**
```c
struct vfe_cmd_chroma_suppression_config {
    uint8_t enable;
    uint8_t m1;   // Threshold 1
    uint8_t m3;   // Threshold 3
    uint8_t n1;   // 3-bit suppression level 1
    uint8_t n3;   // 3-bit suppression level 3
    uint8_t nn1;  // 3-bit
    uint8_t mm1;  // 8-bit
};

struct vfe_chroma_suppress_cfg {
    // 0x3E8: Chroma Suppress 0
    [7:0]  m1;
    [15:8] m3;
    [18:16] n1;
    [22:20] n3;

    // 0x3EC: Chroma Suppress 1
    [7:0]  mm1;
    [10:8] nn1;
};
```

### 17.12 Memory Color Enhancement - MCE (0x3F4)

Enhances specific memory colors (sky blue, grass green, skin tones).

**Registers:**
```
V31_MCE_OFF = 0x3F4 (length: 36 bytes)
```

**Enable mask:**
```c
#define MCE_EN_MASK  0xEFFFFFFF  // Bit 28 = MCE enable
#define MCE_Q_K_MASK 0x0FFFFFFF  // Bits 28-31 = Q_K value
```

### 17.13 Skin Color Enhancement - SCE (0x418)

Special enhancement for skin tones to improve portrait appearance.

**Registers:**
```
V31_SCE_OFF = 0x418 (length: 136 bytes)
V31_SK_ENHAN_CFG = 24 (command ID)
```

### 17.14 Adaptive Spatial Filter - ASF (0x4A0)

Edge enhancement / sharpening filter with adaptive behavior.

**Registers:**
```
V31_ASF_OFF = 0x4A0 (length: 48 bytes)
V31_ASF_UPDATE_LEN = 36
```

**Configuration:**
```c
struct vfe_cmd_asf_config {
    uint8_t enable;
    uint8_t smoothFilterEnabled;
    uint8_t sharpMode;           // 2-bit: sharpening mode
    uint8_t smoothCoefCenter;    // Center coefficient
    uint8_t smoothCoefSurr;      // Surrounding coefficient
    uint8_t normalizeFactor;     // 7-bit normalization
    uint8_t sharpK1;             // 5-bit sharpening K1
    uint8_t sharpK2;             // 5-bit sharpening K2
    uint8_t sharpThreshE1;       // Edge threshold 1 (7-bit)
    int8_t  sharpThreshE2;       // Edge threshold 2 (8-bit signed)
    int8_t  sharpThreshE3;       // Edge threshold 3
    int8_t  sharpThreshE4;       // Edge threshold 4
    int8_t  sharpThreshE5;       // Edge threshold 5
    int8_t  filter1Coefficients[9];  // 3×3 filter 1
    int8_t  filter2Coefficients[9];  // 3×3 filter 2
    // Crop configuration
    uint8_t  cropEnable;
    uint16_t cropFirstPixel;
    uint16_t cropLastPixel;
    uint16_t cropFirstLine;
    uint16_t cropLastLine;
};
```

**ASF output info:**
```c
struct vfe_asf_info {
    [12:0] maxEdge;    // Maximum edge value detected
    [27:16] HBICount;  // Horizontal blanking count
};
```

### 17.15 Main Scaler (0x368)

Scales image to output resolution. Supports independent H/V scaling.

**Registers:**
```
V31_MAIN_SCALER_OFF = 0x368 (length: 28 bytes)
```

**Configuration:**
```c
struct vfe_cmd_main_scaler_config {
    uint8_t enable;
    struct vfe_cmds_scaler_one_dimension hconfig;
    struct vfe_cmds_scaler_one_dimension vconfig;
    struct vfe_cmds_main_scaler_stripe_init MNInitH;
    struct vfe_cmds_main_scaler_stripe_init MNInitV;
};

struct vfe_cmds_scaler_one_dimension {
    uint8_t  enable;
    uint16_t inputSize;           // Input dimension
    uint16_t outputSize;          // Output dimension
    uint32_t phaseMultiplicationFactor;  // 18-bit phase mult
    uint8_t  interpolationResolution;    // 2-bit interp res
};
```

**Register layout:**
```
0x368: hEnable[0], vEnable[1]
0x36C: inWidth[11:0], outWidth[27:16]
0x370: horizPhaseMult[17:0], horizInterRes[21:20]
0x374: horizMNInit[11:0], horizPhaseInit[30:16]
0x378: inHeight[11:0], outHeight[27:16]
0x37C: vertPhaseMult[17:0], vertInterRes[21:20]
0x380: vertMNInit[11:0], vertPhaseInit[30:16]
```

### 17.16 Scaler 2 Y/CbCr (0x4D0 / 0x4E4)

Secondary scaler for encoder/viewfinder output paths.

**Registers:**
```
V31_S2Y_OFF    = 0x4D0 (length: 20 bytes) - Y scaler
V31_S2CbCr_OFF = 0x4E4 (length: 20 bytes) - CbCr scaler
```

**Configuration:**
```c
struct vfe_cmd_scaler2_config {
    uint8_t enable;
    struct vfe_cmds_scaler_one_dimension hconfig;
    struct vfe_cmds_scaler_one_dimension vconfig;
};
```

### 17.17 Field of View Crop (0x360)

Crops input image to region of interest.

**Registers:**
```
V31_FOV_OFF = 0x360 (length: 8 bytes)
```

**Configuration:**
```c
struct vfe_cmd_fov_crop_config {
    uint8_t  enable;
    uint16_t firstPixel;  // [11:0] First pixel (0-indexed)
    uint16_t lastPixel;   // [27:16] Last pixel
    uint16_t firstLine;   // [11:0] First line
    uint16_t lastLine;    // [27:16] Last line
};
```

### 17.18 Chroma Subsample (0x4F8)

Downsamples chroma for 4:2:0 output (NV12) or passes through for 4:2:2 (NV16).

**Registers:**
```
V31_CHROMA_SUBS_OFF = 0x4F8 (length: 12 bytes)
```

**Configuration:**
```c
struct vfe_cmd_chroma_subsample_config {
    uint8_t enable;
    uint8_t cropEnable;
    uint8_t vsubSampleEnable;   // Vertical subsampling (4:2:0)
    uint8_t hsubSampleEnable;   // Horizontal subsampling
    uint8_t vCosited;           // Vertical cositing
    uint8_t hCosited;           // Horizontal cositing
    uint8_t vCositedPhase;
    uint8_t hCositedPhase;
    uint16_t cropWidthFirstPixel;
    uint16_t cropWidthLastPixel;
    uint16_t cropHeightFirstLine;
    uint16_t cropHeightLastLine;
};
```

**Register layout:**
```
0x4F8: hCositedPhase[0], vCositedPhase[1], hCosited[2], vCosited[3],
       hsubSampleEnable[4], vsubSampleEnable[5], cropEnable[6]
0x4FC: cropWidthLastPixel[11:0], cropWidthFirstPixel[27:16]
0x500: cropHeightLastLine[11:0], cropHeightFirstLine[27:16]
```

**For NV12 (4:2:0):** vsubSampleEnable = 1
**For NV16 (4:2:2):** vsubSampleEnable = 0

### 17.19 Output Clamp (0x524)

Clamps output values to valid range.

**Registers:**
```
V31_OUT_CLAMP_OFF = 0x524 (length: 8 bytes)
```

**Configuration:**
```c
struct vfe_cmd_output_clamp_config {
    uint8_t minCh0;  // Y min (typically 0)
    uint8_t minCh1;  // Cb min (typically 0)
    uint8_t minCh2;  // Cr min (typically 0)
    uint8_t maxCh0;  // Y max (typically 255)
    uint8_t maxCh1;  // Cb max (typically 255)
    uint8_t maxCh2;  // Cr max (typically 255)
};
```

**webOS values:**
```
CLAMP_MAX = 0x00FFFFFF (no max clamping)
CLAMP_MIN = 0x00000000 (no min clamping)
```

### 17.20 Module Enable Bits (MODULE_CFG 0x010)

Summary of which modules are enabled in MODULE_CFG register:

```c
struct vfe_mod_enable {
    Bit  0: blackLevelCorrectionEnable;
    Bit  1: lensRollOffEnable;
    Bit  2: demuxEnable;              // CRITICAL for YUV processing
    Bit  3: chromaUpsampleEnable;
    Bit  4: demosaicEnable;
    Bit  5: statsEnable;              // AE statistics
    Bit  6: cropEnable;               // AF statistics enable
    Bit  7: mainScalerEnable;         // AWB statistics enable
    Bit  8: whiteBalanceEnable;       // RS statistics enable
    Bit  9: colorCorrectionEnable;    // CS statistics enable
    Bit 10: yHistEnable;
    Bit 11: skinToneEnable;
    Bit 12: lumaAdaptationEnable;
    Bit 13: rgbLUTEnable;
    Bit 14: chromaEnhanEnable;
    Bit 15: asfEnable;                // Also IHIST enable
    Bit 16: chromaSuppressionEnable;
    Bit 17: chromaSubsampleEnable;
    Bit 18: scaler2YEnable;
    Bit 19: scaler2CbcrEnable;
    Bit 23: SCALE_ENC_EN;
    Bit 27: CROP_ENC_EN;
};
```

**webOS minimal config:** 0x00000004 (only DEMUX enabled for YUV bypass)

---

## 18. Validated Findings (2026-04-17)

This section documents findings validated through raw data capture analysis and testing.

### Write Master DMA Stride Behavior

**Finding:** VFE31 Y and CbCr Write Masters write at **OUTPUT stride (width)**, NOT input stride (width×2).

**Evidence from raw data capture (640×480 NV12):**
- Y line N found at offset N×640 (compact, no gaps)
- CbCr line N found at offset Y_size + N×640 (compact)
- No sparse gaps within planes
- Total Y plane: 640×480 = 307,200 bytes (compact)
- Total frame: 307,200 + 153,600 = 460,800 bytes (compact NV12)

**Implication:** The IMAGE_SIZE stride field (width×2/16) controls VFE internal pipeline timing, but ADDR_CFG burst controls actual DMA write width. The VFE writes data compactly at output width, not at input UYVY width.

### CbCr Offset Calculation

**Validated formula:** `cbcr_offset = width × height` (NOT bytesperline × height)

Since VFE writes compactly at output stride, CbCr immediately follows Y without gaps.
```
640×480 NV12:  cbcr_offset = 640 × 480 = 307,200
640×480 NV16:  cbcr_offset = 640 × 480 = 307,200
1280×1024 NV12: cbcr_offset = 1280 × 1024 = 1,310,720
```

### Buffer Allocation

**Current approach:** Allocate with `stride_factor=2` for safety margin.
```c
effective_bpl = width × stride_factor;  // = width × 2
y_size = height × effective_bpl;
cbcr_size = cbcr_height × effective_bpl;
total = y_size + cbcr_size;
```

**Open question:** If VFE writes compactly at output stride, why do we need stride_factor=2? Testing suggests it may be:
- Required for certain resolutions (pix1280)
- A safety margin for worst-case DMA timing
- Related to UB (Unified Buffer) configuration

### NV16 Buffer Allocation Fix

**Bug found:** Previous condition `vsub_num > 1` only matched NV12/NV21 (4:2:0) and missed NV16/NV61 (4:2:2 where vsub_num=1). This caused NV16 to bypass stride_factor allocation path.

**Result:** Buffer allocated 614,400 bytes but VFE potentially wrote 1,228,800 bytes → memory corruption.

**Fix (commit 16036c3fd206):** Changed condition to `stride_factor > 1` to catch all semi-planar formats on VFE31.

### DMA Cache Sync

**Finding:** VFE31 should NOT perform manual DMA cache sync.

- `dma_sync_single_for_cpu()` crashes - wrong for SG (scatter-gather) buffers
- `dma_sync_sgtable_for_cpu()` causes hangs
- Other VFE implementations (gen1, 17x) rely on vb2's built-in `finish()` memop

**Current approach:** Call `vb2_buffer_done()` directly without manual sync, matching other VFE implementations.

---

## 19. Open Questions for Investigation

### 1. stride_factor Inconsistency
Raw data shows VFE writes compactly at OUTPUT stride, yet we allocate buffers at 2× size with stride_factor=2. Is this:
- A safety margin for worst-case scenarios?
- Required for higher resolutions (1280×1024)?
- Wasteful but harmless?

### 2. pix1280 Y/CbCr Swap Issue
At 1280×1024 resolution, Y/CbCr planes appear swapped or offset incorrectly. Possible causes:
- Different DEMUX/XBAR routing for higher resolutions?
- UB_CFG depth overflow at larger widths?
- IMAGE_SIZE stride field overflow (width×2/16 = 160, fits in 16 bits)?

### 3. DEMUX Configuration
Is DEMUX correctly separating Y and CbCr? Earlier analysis suggested raw data showed UYVY pattern in Y plane, but pix640 NV12 now works correctly.

Key registers to verify:
```
DEMUX_EVEN_CFG (0x290) = 0xC9CA (webOS)
DEMUX_ODD_CFG (0x294) = 0xC9CA (webOS)
```

### 4. VIDEO Mode CbCr Capture
webOS never enabled VIDEO CbCr WM (WM5). VIDEO mode uses:
- WM1 for Y (vs PIX uses WM0)
- WM5 for CbCr (vs PIX uses WM4)

Is full semi-planar capture in VIDEO mode architecturally possible on VFE31?

### 5. WM Configuration Differences
PIX mode (preview):
- Y → WM0, CbCr → WM4
- XBAR_CFG1 = 0x1A13

VIDEO mode (recording):
- Y → WM1, CbCr → WM5 (theoretically)
- XBAR_CFG1 = 0x1A1B (enables both paths)
- webOS only enabled WM1 (Y), not WM5 (CbCr)

---

## 20. Test Results Summary

| Test Mode | Resolution | Format | Status | Notes |
|-----------|------------|--------|--------|-------|
| pix640 | 640×480 | NV12 | PASS | 3 frames captured correctly |
| pix640 | 640×480 | NV16 | UNTESTED | After buffer fix |
| pix1280 | 1280×1024 | NV12 | FAIL | Y/CbCr swap issue |
| video640 | 640×480 | NV12 | UNTESTED | |
| video1280 | 1280×1024 | NV12 | UNTESTED | |

---

## References

- webOS kernel: `drivers/media/video/msm/msm_vfe31.h` (1077 lines)
- webOS kernel: `drivers/media/video/msm/msm_vfe31.c`
- Mako kernel: [raden/ampang-AOSP-mako-kernel](https://github.com/raden/ampang-AOSP-mako-kernel/blob/master/drivers/media/video/msm/vfe/msm_vfe31.h)
- Android kernel/msm: [VFE32 header](https://android.googlesource.com/kernel/msm/+/android-msm-hammerhead-3.4-kk-fr2/drivers/media/platform/msm/camera_v1/vfe/msm_vfe32.h)
- Linux mainline: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- Decompiled libqcamera HAL
- HP TouchPad hardware testing (APQ8060)

---

*Document version: 1.3*
*Last updated: 2026-04-17*
*Based on VFE HW version 0x00030217*

**Changelog:**
- v1.3: Added comprehensive ISP Processing Modules section (Section 17) with:
  - Black level, lens rolloff, DEMUX, demosaic (ABF/BPC)
  - White balance, color correction, gamma, luma adaptation
  - Chroma enhancement, suppression, MCE, SCE
  - ASF (sharpening), scalers, FOV crop, chroma subsample
  - Output clamp, MODULE_CFG bit summary
- v1.2: Added CORE_CFG, BUS_CFG, XBAR details; corrected CbCr burst_lines (+64)
- v1.1: Initial comprehensive register documentation
