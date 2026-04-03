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
```
[2:0] PIXEL_PATTERN:
      0x4 = YCBYCR (Cb before Cr)
      0x5 = YCRYCB (Cr before Cb)
      0x6 = CBYCRY
      0x7 = CRYCBY

Bit  6: INPUT_MUX_ENABLE   - Required by webOS (exact function unknown)

webOS value: 0x00000046 (CBYCRY + bit 6)
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

## 3. Bus/AXI/XBAR Configuration (0x038 - 0x044)

### VFE_0_BUS_CMD (0x038) - Write Only
Write master reload command.
```
Bit n: Reload WM n configuration from shadow registers

Example: 0x3F = Reload WM0-WM5
```

### VFE_0_BUS_CFG (0x03C)
Bus configuration - path enables and raw pixel settings.
```
[3:0]   (unknown, webOS sets 0x1)
Bit  4: ENC_Y_WR_PATH_EN      - Encoder Y write path enable
Bit  5: ENC_CBCR_WR_PATH_EN   - Encoder CbCr write path enable
Bit  6: VIEW_Y_WR_PATH_EN     - Viewfinder Y write path enable
Bit  7: VIEW_CBCR_WR_PATH_EN  - Viewfinder CbCr write path enable
[9:8]   RAW_PIXEL_DATA_SIZE:
        0 = 8-bit
        1 = 10-bit
        2 = 12-bit
[11:10] RAW_WR_PATH_SEL:
        0 = Disabled
        1 = Route to encoder CbCr path
        2 = Route to viewfinder CbCr path
[31:12] Timing/strobe config (0x02AAA in webOS, details unknown)

webOS value: 0x02AAA771 (all paths enabled, 8-bit raw)
```

### VFE_0_BUS_XBAR_CFG0 (0x040)
AXI output mode selection.
```
[31:0] AXI_OUTPUT_MODE:
       0x01  = OUTPUT_1_AND_3 (Preview WM0/1 + Video WM4/5)
       0x200 = OUTPUT_2 (Preview only, older mode)
       0x60  = CAMIF_TO_AXI_VIA_OUT2 (Raw bypass to WM0)

NOTE: VFE31 uses global XBAR, NOT per-WM like VFE41+
```

### VFE_0_BUS_XBAR_CFG1 (0x044)
Data routing configuration - CRITICAL for correct output.
```
[3:0]   Y_ROUTING:
        0x3 = Y to WM0 only (preview)
        0xB = Y to WM0 + WM4 (preview + video)

[7:4]   CBCR_ROUTING:
        0x0 = DISABLED (broken for semi-planar!)
        0x1 = CbCr to WM1 only (preview)
        0x9 = CbCr to WM1 + WM5 (preview + video)

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
[15:0]  BURST = (bytes_per_line / 4) - 17
        For UYVY input (2 bytes/pixel): (width * 2 / 4) - 17
        Example: 640x480 UYVY = (1280 / 4) - 17 = 303 = 0x12F

[31:16] LINES - Line count for CbCr plane timing
        WM0/WM4 (Y plane): 0 (uses IMAGE_SIZE height)
        WM1/WM5 (CbCr plane): height - 24 = 456 for 480 lines

        CRITICAL: The -24 offset is empirically required for correct
        NV16 output. Without it, only top portion of UV plane is written.
        Reason unknown - may be VFE31-specific timing requirement.

Example @ 640x480:
  WM0: WR_ADDR_CFG = 0x0000012F (burst=303, lines=0)
  WM1: WR_ADDR_CFG = 0x01C8012F (burst=303, lines=456)
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

### webOS WM Configuration @ 640x480
```
WM0 (Y plane):
  WR_CFG       = 0x00000001
  WR_PING_ADDR = <buffer_phys>
  WR_PONG_ADDR = <buffer_phys + frame_size>
  WR_ADDR_CFG  = 0x0000012F (burst=303, lines=0)
  WR_UB_CFG    = 0x002701DF (ub=39, height=479)
  WR_IMAGE_SIZE= 0x00501DF2 (stride=80, size=0x1DF2)

WM1 (CbCr plane):
  WR_CFG       = 0x00000001
  WR_PING_ADDR = <buffer_phys + y_plane_size>
  WR_PONG_ADDR = <buffer_phys + frame_size + y_plane_size>
  WR_ADDR_CFG  = 0x01C8012F (burst=303, lines=456)
  WR_UB_CFG    = 0x002701DF (same as WM0)
  WR_IMAGE_SIZE= 0x00501DF2 (same as WM0)
```

---

## 11. Known Issues and Quirks

### XBAR_CFG1 CbCr Routing
The original Qualcomm/mainline driver used XBAR_CFG1 = 0x1A03 which disables
CbCr routing to WM1. This causes NV16 output to have no UV data (green image).
Must use 0x1A1B or 0x1A9B for working semi-planar output.

### WM1 Lines Field (-24 Offset)
For unknown reasons, WM1 (CbCr plane) requires the lines field in WR_ADDR_CFG
to be set to (height - 24). Without this, only the top portion of the UV plane
is written. This appears to be a VFE31-specific timing requirement.

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

## References

- webOS kernel: `drivers/media/video/msm/msm_vfe31.h` (1077 lines)
- webOS kernel: `drivers/media/video/msm/msm_vfe31.c`
- Mako kernel: [raden/ampang-AOSP-mako-kernel](https://github.com/raden/ampang-AOSP-mako-kernel/blob/master/drivers/media/video/msm/vfe/msm_vfe31.h)
- Android kernel/msm: [VFE32 header](https://android.googlesource.com/kernel/msm/+/android-msm-hammerhead-3.4-kk-fr2/drivers/media/platform/msm/camera_v1/vfe/msm_vfe32.h)
- Linux mainline: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- Decompiled libqcamera HAL
- HP TouchPad hardware testing (APQ8060)

---

*Document version: 1.1*
*Last updated: 2026-04-03*
*Based on VFE HW version 0x00030217*
