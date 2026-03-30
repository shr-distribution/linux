# MT9M113 Datasheet Analysis for HP TouchPad Camera Driver

This document provides a complete analysis of the MT9M113 sensor datasheet for
implementing the mainline Linux driver with CSI-2 MIPI output on HP TouchPad.

## Document Structure

| Part | Pages | Content |
|------|-------|---------|
| Part 1 | 1-40 | General description, architecture, MIPI interface overview |
| Part 2 | 41-80 | Register map, PLL configuration, MIPI timing registers |
| Part 3 | 81-121 | MCU variables, timing specs, electrical specs, I2C timing |

---

## 1. Basic Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Chip ID | 0x2480 | Confirmed in driver |
| Resolution | 1280x1024 (SXGA) | Context B full |
| Pixel Size | 1.75µm × 1.75µm | |
| Input Clock | 8-54 MHz | HP TouchPad uses 24 MHz |
| Max Pixel Clock | 60 MHz | |
| Max Frame Rate | 15 fps (full), 24 fps (preview), 30 fps (video) | |
| ADC Resolution | 10-bit | |
| Color Filter | RGB Bayer pattern | |
| Interface | Sub-LVDS differential (MIPI-like) | Single data lane |

---

## 2. Power Supply Requirements

| Supply | Voltage | Purpose |
|--------|---------|---------|
| VAA | 2.5-3.1V | Analog power |
| VAA_PIX | 2.5-3.1V | Pixel array (tie to VAA) |
| VDD | 1.7-1.95V | Digital core |
| VDD_IO | 1.7-3.1V | I/O power |
| VDD_PLL | 2.5-3.1V | PLL power (tie to VDD_IO) |
| **VDDIO_TX** | **1.7-1.95V** | **MIPI interface power (tie to VDD)** |

**Critical Notes:**
- VDD_PLL and VDD_IO should be tied together
- VDD and VDDIO_TX should be tied together
- VDDIO_TX must be 1.7-1.95V for proper MIPI operation

---

## 3. MIPI/CSI-2 Interface Configuration

### 3.1 Interface Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Interface Type | Sub-LVDS differential | Not standard MIPI D-PHY |
| Max Data Rate | 480 Mb/s | Per datasheet |
| Data Lanes | 1 | Single lane only |
| Clock Mode | LP or Continuous | Continuous recommended for MSM8660 |
| Signals | DOUT_P, DOUT_N, CLK_P, CLK_N | Differential pairs |

### 3.2 MIPI Data Types (from Table 6)

| Data Format | Data Type Code | Notes |
|-------------|----------------|-------|
| YUV 422 8-bit | 0x1E | **Used by HP TouchPad (UYVY)** |
| RGB565 | 0x22 | |
| RGB555 | 0x21 | |
| RGB444 | 0x20 | |
| RAW8 | 0x2A | |
| RAW10 | 0x2B | |

### 3.3 OUTPUT_CONTROL Register (0x3400)

This is the primary MIPI control register:

```
Bits [15:10]: data_type    - MIPI data type (0x1E = YUV422)
Bit  [9]:     mipi_en      - MIPI enable (0=disabled, 1=enabled)
Bits [8:6]:   chan_num     - Virtual channel number
Bit  [4]:     standby_eof  - Wait until EOF for standby
Bit  [3]:     reg_frame_sync - READ-ONLY frame sync status
Bit  [2]:     cont_mipi_clk - Continuous MIPI clock
Bit  [1]:     standby_en   - MIPI standby (0=active, 1=standby)
Bit  [0]:     restart_en   - MIPI restart enable
```

**Key Values:**
- `0x7A08` - MIPI enabled, LP clock mode (webOS default)
- `0x7A0C` - MIPI enabled, continuous clock mode (eliminates ECC errors)
- `0x0000` - MIPI disabled

**IMPORTANT:** Bit 3 is READ-ONLY! Writes to bit 3 are ignored by hardware.

### 3.4 MIPI Timing Registers (0xC988-0xC992)

| Register | Address | webOS Value | Description |
|----------|---------|-------------|-------------|
| T_HS_ZERO | 0xC988 | 0x0F00 | HS transmit zero time |
| T_HS_EXIT_TRAIL | 0xC98A | 0x0B07 | HS exit and trail time |
| T_CLK_POST_PRE | 0xC98C | 0x0D01 | Clock post and pre time |
| T_CLK_TRAIL_ZERO | 0xC98E | 0x071D | Clock trail and zero time |
| T_LPX | 0xC990 | 0x0006 | Low-power transmit time |
| TIMING_INIT | 0xC992 | 0x0A0C | Initialization timing |

These are MCU variables accessed via indirect addressing.

---

## 4. MCU Variable Access (XDMA)

The MT9M113 uses indirect addressing for MCU variables:

| Register | Address | Purpose |
|----------|---------|---------|
| MCU_ADDRESS | 0x098C | Write variable address here |
| MCU_DATA | 0x0990 | Read/write variable data |

**Access Sequence:**
1. Write variable address to 0x098C
2. Read/write data at 0x0990

**Important:** MT9M113 uses 0x098C, MT9M114 uses 0x098E!

---

## 5. MCU Variable Reference (Part 3 Analysis)

### 5.1 Sequencer Variables (Driver ID = 1)

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| SEQ_CMD | 0xA103 | - | Sequencer command register |
| SEQ_STATE | 0xA104 | - | Current sequencer state (RO) |
| SEQ_CAP_MODE | 0xA115 | - | Capture mode selection |

**SEQ_CMD Values:**
| Command | Value | Action |
|---------|-------|--------|
| RUN | 0x0001 | Start streaming |
| STANDBY | 0x0003 | Enter standby |
| LOCK | 0x0004 | Lock current settings |
| REFRESH | 0x0005 | Refresh all parameters |
| REFRESH_MODE | 0x0006 | Refresh mode only |

**SEQ_CAP_MODE Values:**
- `0x0030` - Preview mode (Context A)
- `0x0002` - Capture mode (Context B)

### 5.2 Mode Variables (Driver ID = 7)

**Context A (Preview) Configuration:**

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| MODE_OUTPUT_WIDTH_A | 0x2703 | 0x0280 (640) | Output width |
| MODE_OUTPUT_HEIGHT_A | 0x2705 | 0x0200 (512) | Output height |
| MODE_SENSOR_ROW_START_A | 0x270D | 0x0000 | First row |
| MODE_SENSOR_COL_START_A | 0x270F | 0x0000 | First column |
| MODE_SENSOR_ROW_END_A | 0x2711 | 0x04BD | Last row |
| MODE_SENSOR_COL_END_A | 0x2713 | 0x064D | Last column |
| MODE_SENSOR_ROW_SPEED_A | 0x2715 | 0x2112 | Row speed (bits 2:0 = pix_clk divider) |
| MODE_SENSOR_READ_MODE_A | 0x2717 | 0x046C | Read mode (binning, skip, mirror) |
| MODE_SENSOR_FRAME_LENGTH_A | 0x271F | 0x0293 | Frame length in lines |
| MODE_SENSOR_LINE_LENGTH_PCK_A | 0x2721 | 0x07D0 | Line length in pixel clocks |
| MODE_OUTPUT_FORMAT_A | 0x2755 | 0x0000 | Output format configuration |

**Context B (Capture) Configuration:**

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| MODE_OUTPUT_WIDTH_B | 0x2707 | 0x0500 (1280) | Output width |
| MODE_OUTPUT_HEIGHT_B | 0x2709 | 0x0400 (1024) | Output height |
| MODE_SENSOR_ROW_START_B | 0x2723 | 0x0004 | First row |
| MODE_SENSOR_COL_START_B | 0x2725 | 0x0004 | First column |
| MODE_SENSOR_ROW_END_B | 0x2727 | 0x04BB | Last row |
| MODE_SENSOR_COL_END_B | 0x2729 | 0x064B | Last column |
| MODE_SENSOR_ROW_SPEED_B | 0x272B | 0x2111 | Row speed |
| MODE_SENSOR_READ_MODE_B | 0x272D | 0x0024 | Read mode (no binning) |
| MODE_SENSOR_FRAME_LENGTH_B | 0x2735 | 0x04ED | Frame length |
| MODE_SENSOR_LINE_LENGTH_PCK_B | 0x2737 | 0x0D06 | Line length |
| MODE_OUTPUT_FORMAT_B | 0x2757 | 0x0000 | Output format configuration |

**MODE_OUTPUT_FORMAT Bit Layout:**

```
Bit  8:    config_processed_bayer - Output Bayer (reduces data rate by 2x)
Bits 7:6:  rgb_format - 0=RGB565, 1=RGB555, 2=RGB444x, 3=RGBx444
Bit  5:    output_mode - 0=YUV, 1=RGB
Bit  3:    monochrome - Force monochrome output
Bit  1:    swap_chrominance_luma - Swap Y and C in YUV mode
Bit  0:    swap_channels - Swap Cb/Cr in YUV, R/B in RGB
```

**MODE_SENSOR_READ_MODE Bit Layout:**

```
Bit  11:   x_bin_en - X-only binning (x_odd_inc=3, y_odd_inc=1)
Bit  10:   xy_bin_en - X+Y binning (x_odd_inc=3, y_odd_inc=3)
Bit  9:    low_power_enable - Low power mode
Bits 7:5:  x_odd_inc - Column address increment (1=normal, 3=skip)
Bits 4:2:  y_odd_inc - Row address increment (1=normal, 3=skip)
Bit  1:    vert_flip - Vertical flip
Bit  0:    horiz_mirror - Horizontal mirror
```

**Changes require REFRESH_MODE command to take effect!**

### 5.3 Auto Exposure Variables (Driver ID = 2)

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| AE_GATE | 0xA207 | 0x0004 | AE sensitivity |
| AE_MIN_INDEX | 0xA20B | 0x0000 | Minimum zone number |
| AE_MAX_INDEX | 0xA20C | 0x0018 | Maximum zone number |
| AE_MIN_VIRT_GAIN | 0xA20D | 0x0010 | Min virtual gain (0x20=1.0) |
| AE_MAX_VIRT_GAIN | 0xA20E | 0x0080 | Max virtual gain |
| AE_STATUS | 0xA217 | - | AE status (bit 2=ready) |
| AE_CURRENT_Y | 0xA218 | - | Current luminance (RO) |
| AE_BASE_TARGET | 0xA24F | 0x0036 | Base target brightness |

### 5.4 Auto White Balance Variables (Driver ID = 3)

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| AWB_CCM_L_0-8 | 0xA306+ | varies | Left color correction matrix |
| AWB_CCM_RL_0-8 | 0xA31C+ | varies | Delta color correction matrix |
| AWB_GAIN_R | 0xA34E | 0x0080 | Red digital gain (0x80=1.0) |
| AWB_GAIN_G | 0xA34F | 0x0080 | Green digital gain |
| AWB_GAIN_B | 0xA350 | 0x0080 | Blue digital gain |
| AWB_CCM_POSITION | 0xA353 | 0x0040 | Matrix position (0=incandescent, 0x7F=daylight) |
| AWB_SATURATION | 0xA354 | 0x0080 | Color saturation (0x80=100%) |

### 5.5 Anti-Flicker Variables (Driver ID = 4)

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| FD_MODE | 0xA404 | 0x0000 | Flicker detection mode |
| FD_R9_STEP_F60_A | 0xA411 | 0x009D | 60Hz step for Context A |
| FD_R9_STEP_F50_A | 0xA413 | 0x00BC | 50Hz step for Context A |

**FD_MODE Bit Layout:**
```
Bit 7: manual_mode - 0=auto, 1=manual
Bit 6: curr_settings - 0=60Hz, 1=50Hz (when manual)
Bit 5: curr_flicker_state - Current setting (RO)
```

### 5.6 Histogram Variables (Driver ID = 11)

| Variable | Address | Default | Description |
|----------|---------|---------|-------------|
| HG_MAX_DARK_LEVEL | 0xAB04 | 0x0040 | Dark level limit |
| HG_BRIGHTNESS_METRIC | 0xAB1B | - | Scene brightness (RO) |
| HG_GAMMA_MORPH_CTRL | 0xAB37 | 0x0003 | Gamma table control |
| HG_GAMMA_TABLE_A_0-18 | 0xAB3C+ | varies | Normal light gamma |
| HG_GAMMA_TABLE_B_0-18 | 0xAB4F+ | varies | Low light gamma |

---

## 6. Timing Specifications (Part 3)

### 6.1 Power-Up Sequence

**Order:** VDD_IO → (VAA, VAA_PIX, VDD, VDD_PLL) → EXTCLK → RESET_BAR → I2C

| Parameter | Symbol | Min | Typ | Max | Units |
|-----------|--------|-----|-----|-----|-------|
| Delay VDD_IO to VAA/VAA_PIX | t1 | 0 | - | 500 | ms |
| Delay VDD_IO to VDD | t2 | 0 | - | 500 | ms |
| Delay VDD_IO to VDD_PLL | t3 | 0 | - | 500 | ms |
| EXTCLK activation | t4 | - | 500 | - | ms |
| RESET_BAR activation time | t5 | - | 70 | 1 | EXTCLKs |
| RESET_BAR init time | t6 | - | 2 | - | ms |
| First I2C write | t7 | - | 100 | - | EXTCLKs |

### 6.2 Hard Reset Timing

| Parameter | Symbol | Min | Typ | Max | Units |
|-----------|--------|-----|-----|-----|-------|
| RESET_BAR pulse width | t1 | 70 | - | - | EXTCLKs |
| Active EXTCLK after RESET_BAR | t2 | 10 | - | - | EXTCLKs |
| Active EXTCLK before RESET_BAR de-assert | t3 | 10 | - | - | EXTCLKs |
| Max ROM read time | t4 | - | - | 6000 | EXTCLKs |

At 24 MHz: 6000 EXTCLKs = 250 µs

### 6.3 Soft Reset Timing

| Parameter | Symbol | Min | Typ | Max | Units |
|-----------|--------|-----|-----|-----|-------|
| Active EXTCLK after soft reset | t1 | - | 6000 | - | EXTCLKs |

### 6.4 Hard Standby Timing

| Parameter | Symbol | Min | Typ | Max | Units |
|-----------|--------|-----|-----|-----|-------|
| Standby effective delay | t1 | 20 | - | - | EXTCLKs |
| Active EXTCLK after STANDBY | t2 | 10 | - | - | EXTCLKs |
| Active EXTCLK before STANDBY de-assert | t3 | 10 | - | - | EXTCLKs |
| STANDBY pulse width | t4 | 100 | - | - | EXTCLKs |

### 6.5 Soft Standby Timing

| Parameter | Symbol | Min | Typ | Max | Units |
|-----------|--------|-----|-----|-----|-------|
| Standby entry complete | t1 | - | 20 | - | EXTCLKs |
| Active EXTCLK after soft standby | t2 | - | 10 | - | EXTCLKs |
| Active EXTCLK before soft standby de-activate | t3 | - | 10 | - | EXTCLKs |
| Minimum standby time | t4 | - | 100 | - | EXTCLKs |

---

## 7. Signal States During Different Modes

| Signal | Reset | Post-Reset | Standby | Power Down |
|--------|-------|------------|---------|------------|
| DOUT[7:0] | High-Z | High-Z | High-Z (configurable) | X |
| PIXCLK | High-Z | High-Z | High-Z (configurable) | X |
| LV/FV | High-Z | High-Z | High-Z (configurable) | X |
| **DOUT_P/N** | **0** | **0** | **0** | X |
| **CLK_P/N** | **0** | **0** | **0** | X |
| GPIO[4:0] | High-Z | High-Z | Depends on config | X |
| SDATA | Input | I/O | Input | X |
| SCLK | Input | Input | Input | X |

**Important for MIPI:** DOUT and CLK differential pairs are held at 0 during reset/standby, not High-Z.

---

## 8. Electrical Specifications for MIPI Mode

### 8.1 DC Characteristics (Serial/MIPI Mode)

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| VDD (Core digital) | 1.7 | 1.8 | 1.95 | V |
| VDD_IO | 1.7 | 1.8 | 1.95 | V |
| VAA | 2.5 | 2.8 | 3.1 | V |
| VAA_PIX | 2.5 | 2.8 | 3.1 | V |
| VDD_PLL | 2.5 | 2.8 | 3.1 | V |
| Max standby current | - | - | 20 | µA |

### 8.2 I/O Timing (MIPI Mode)

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| PIXCLK frequency | 30 | - | 60 | MHz |
| External clock (PLL enabled) | 8 | - | 54 | MHz |
| Clock duty cycle | 40 | 50 | 60 | % |
| Input clock jitter | - | 0.5 | 1 | ns |

---

## 9. I2C Interface Specifications

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| Serial clock frequency | 100 | - | 400 | kHz |
| Serial clock period | 2.5 | - | 10 | µs |
| SCLK duty cycle | 40 | 50 | 60 | % |
| Start hold time | 0.6 | - | - | µs |
| SDATA hold | 0.6 | - | - | µs |
| SDATA setup | 0.6 | - | - | µs |
| Stop setup time | 0.6 | - | - | µs |
| Stop hold time | 0.6 | - | - | µs |

**I2C Addresses:**
- Default: 0x78 (write), 0x79 (read) - 7-bit: 0x3C
- Alternate (SADDR=HIGH): 0x7A (write), 0x7B (read) - 7-bit: 0x3D

---

## 10. GPIO Configuration (Part 3)

### 10.1 GPIO Register (0x1070)

| Bits | Name | Description |
|------|------|-------------|
| 12:8 | gpio_4_0_data | GPIO state/control |
| 7:0 | Reserved | |

### 10.2 GPIO Direction Register (0x1078)

| Bit | Default | Function when 0 | Function when 1 |
|-----|---------|-----------------|-----------------|
| 12 | 1 | Flash output | General input |
| 11 | 1 | General output | General input |
| 10 | 1 | General output | OE_BAR |
| 9 | 1 | DOUT_LSB1 | General input |
| 8 | 1 | DOUT_LSB0 | General input |

**Note:** GPIO[2] can be configured as OE_BAR by setting oe_n_enable (0x001A[8])=1

---

## 11. Streaming Start Sequence (HP TouchPad)

Based on webOS driver analysis and datasheet timing:

```
1. Power up sequence (see section 6.1)
2. Wait 200ms after reset
3. Load initialization registers
4. Configure MIPI timing registers (0xC988-0xC992)
5. Wait for CSIPHY to be configured (done by V4L2 pipeline)
6. Wait 10ms for CSIPHY stabilization
7. Enable MIPI output: OUTPUT_CONTROL (0x3400) = 0x7A08 (or 0x7A0C)
8. Set streaming mode: RESET_REGISTER (0x301A) = 0x120C
9. Set capture mode: SEQ_CAP_MODE (0xA115) = 0x0030
10. Wait 40ms
11. Start streaming: SEQ_CMD (0xA103) = 0x0001
12. Poll SEQ_CMD until it returns 0x0000
```

**Critical Points:**
- MIPI output must be enabled BEFORE SEQ_CMD=RUN
- CSIPHY must be configured BEFORE MIPI output is enabled
- Use continuous clock mode (0x7A0C) if LP mode causes ECC errors

---

## 12. Known Issues and Workarounds

### 12.1 ECC Errors with LP Clock Mode
- **Problem:** MSM8660 CSIPHY sees ECC errors with LP clock mode
- **Solution:** Use continuous clock mode (OUTPUT_CONTROL bit 2 = 1)

### 12.2 MCU Health Check
- **Problem:** MCU can become unresponsive after extended idle
- **Solution:** Read MODE_OUTPUT_WIDTH_A before streaming; if 0, reinitialize

### 12.3 Bit 3 of OUTPUT_CONTROL
- **Problem:** Bit 3 is read-only but driver may try to verify writes
- **Solution:** Mask out bit 3 when verifying OUTPUT_CONTROL writes

### 12.4 MIPI Output on Stream Stop
- **Problem:** Leaving MIPI enabled during power suspend causes errors on next stream
- **Solution:** Explicitly write OUTPUT_CONTROL=0x0000 before stopping

---

## 13. Register Quick Reference

### Core Registers
| Register | Address | Description |
|----------|---------|-------------|
| CHIP_ID | 0x0000 | Chip identification (0x2480) |
| RESET_AND_MISC_CONTROL | 0x001A | Soft reset control |
| ACCESS_CTL_STAT | 0x0982 | Register access unlock |
| MCU_ADDRESS | 0x098C | MCU variable address |
| MCU_DATA | 0x0990 | MCU variable data |
| RESET_REGISTER | 0x301A | Streaming control |
| OUTPUT_CONTROL | 0x3400 | MIPI output control |
| CUSTOM_SHORT_PKT | 0x3404 | FS/FE packet control |

### Key MCU Variables
| Variable | Address | Description |
|----------|---------|-------------|
| SEQ_CMD | 0xA103 | Sequencer command |
| SEQ_STATE | 0xA104 | Sequencer state (RO) |
| SEQ_CAP_MODE | 0xA115 | Capture mode |
| MODE_OUTPUT_WIDTH_A | 0x2703 | Context A width |
| MODE_OUTPUT_HEIGHT_A | 0x2705 | Context A height |
| MODE_OUTPUT_WIDTH_B | 0x2707 | Context B width |
| MODE_OUTPUT_HEIGHT_B | 0x2709 | Context B height |

---

## 14. Comparison: Driver vs Datasheet

| Feature | Datasheet | Driver | Status |
|---------|-----------|--------|--------|
| MCU_ADDRESS register | 0x098C | 0x098C | ✅ Match |
| MCU_DATA register | 0x0990 | 0x0990 | ✅ Match |
| OUTPUT_CONTROL | 0x3400 | 0x3400 | ✅ Match |
| RESET_REGISTER | 0x301A | 0x301A | ✅ Match |
| SEQ_CMD | 0xA103 | 0xA103 | ✅ Match |
| Chip ID | 0x2480 | 0x2480 | ✅ Match |
| MIPI Data Type (YUV422) | 0x1E | 0x1E | ✅ Match |
| Reset timing | 70 EXTCLKs min | 200ms delay | ✅ Exceeds min |
| I2C wait after reset | 100 EXTCLKs | 200ms | ✅ Exceeds min |

---

*Document generated from MT9M113 Datasheet Rev. B 3/07*
*Last updated: Analysis of Parts 1, 2, and 3 complete*
