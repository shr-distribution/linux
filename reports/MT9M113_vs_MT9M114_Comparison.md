# MT9M111 vs MT9M113 vs MT9M114 Register Architecture Comparison

**Date:** 2026-03-10
**Purpose:** Compare register layouts between MT9M111, MT9M113 (1.3MP) and MT9M114 (720p) SOC sensors

---

## 1. Executive Summary

The MT9M111, MT9M113, and MT9M114 are Micron/Aptina/onsemi SOC image sensors with different architectures:

- **MT9M111**: 1/3" sensor with hardware IFP (no embedded MCU), uses page-based register access
- **MT9M113**: 1/6" sensor with embedded MCU and firmware drivers, uses XDMA variable access
- **MT9M114**: 1/6" 720p sensor with embedded MCU, uses XDMA with direct addressing

### Key Differences

| Feature | MT9M111 | MT9M113 | MT9M114 |
|---------|---------|---------|---------|
| Chip ID | 0x143A | 0x2480 | (different) |
| Optical Format | 1/3" | 1/6" | 1/6" |
| Resolution | 1280x1024 (1.3MP) | 1280x1024 (1.3MP SXGA) | 1280x720 (720p) |
| Architecture | Hardware IFP | MCU + Firmware | MCU + Firmware |
| Register Access | Page Map (0xF0) | XDMA (0x098C) | XDMA (0x098E) |
| MCU/Firmware | **None** | Yes | Yes |
| MIPI Control | N/A (parallel only) | 0x3400 | 0x3C40 |
| Command Interface | Direct registers | seq_cmd variable | 0x0080 register |
| Context A/B | Yes | Yes | Single-stream |
| Linux Driver | `mt9m111.c` | (custom) | `mt9m114.c` |

---

## 2. I2C/Two-Wire Serial Interface

### MT9M111
- Standard I2C with 8-bit register addresses, 16-bit data
- Uses page mapping via register 0xF0
- Chip ID at register 0x00 returns 0x143A

### MT9M113
- Standard I2C with 16-bit register addresses, 16-bit data
- READ/WRITE protocols identical to MT9M114
- Sequential burst access supported

### MT9M114
- Same I2C protocol as MT9M113
- Same 16-bit address, 16-bit data format

**Verdict:** MT9M111 uses different addressing (8-bit vs 16-bit), MT9M113/MT9M114 compatible at protocol level

---

## 3. MT9M111 Architecture (Hardware IFP - No MCU)

### Key Architectural Difference

The MT9M111 does **NOT** have an embedded MCU or firmware drivers. Instead, it uses a **hardware-based Image Flow Processor (IFP)** with direct register access.

### Register Page Organization

| Page | Address Range | Description |
|------|---------------|-------------|
| Page 0 | 0x000-0x0FF | Sensor Core |
| Page 1 | 0x100-0x1FF | Colorpipe (IFP) |
| Page 2 | 0x200-0x2FF | Camera Controller |

**Page Selection:** Write to register 0xF0 (PAGE_MAP) to switch pages

### Key MT9M111 Registers

#### Sensor Core (Page 0)

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | CHIP_VERSION | Returns 0x143A |
| 0x01 | ROW_START | Crop row start |
| 0x02 | COLUMN_START | Crop column start |
| 0x03 | WINDOW_HEIGHT | Active image height |
| 0x04 | WINDOW_WIDTH | Active image width |
| 0x05 | HORIZONTAL_BLANKING_A | Context A H-blank |
| 0x06 | VERTICAL_BLANKING_A | Context A V-blank |
| 0x07 | HORIZONTAL_BLANKING_B | Context B H-blank |
| 0x08 | VERTICAL_BLANKING_B | Context B V-blank |
| 0x09 | SHUTTER_WIDTH | Exposure time |
| 0x0D | RESET | Chip enable, power mode, restart |
| 0x20 | READ_MODE_B | Context B skip/mirror |
| 0x21 | READ_MODE_A | Context A skip/mirror |
| 0x2B | GREEN1_GAIN | Green1 channel gain |
| 0x2C | BLUE_GAIN | Blue channel gain |
| 0x2D | RED_GAIN | Red channel gain |
| 0x2E | GREEN2_GAIN | Green2 channel gain |
| 0x2F | GLOBAL_GAIN | Global analog gain |
| 0xC8 | CONTEXT_CONTROL | Select context A or B |
| 0xF0 | PAGE_MAP | Page selection |
| 0xF1 | BYTE_WISE_ADDR | Byte-wise addressing |

#### Colorpipe (Page 1)

| Address | Name | Description |
|---------|------|-------------|
| 0x06 | OPER_MODE_CTRL | AE/AWB enable flags |
| 0x08 | OUTPUT_FORMAT_CTRL | Output format selection |
| 0x3A | OUTPUT_FORMAT_CTRL2_A | Context A format |
| 0x48 | TPG_CTRL | Test pattern generator |
| 0x9B | OUTPUT_FORMAT_CTRL2_B | Context B format |
| 0xE2 | EFFECTS_MODE | Color effects (sepia, mono, etc.) |

### Context A/B System

MT9M111 uses a context-based system similar to MT9M113:
- **Context A**: Preview/viewfinder mode
- **Context B**: Snapshot/capture mode

Context switching is controlled by register 0xC8 (CONTEXT_CONTROL).

### IFP Architecture

```
Sensor Core → Colorpipe (CP) → Camera Controller (CC) → Output
              ├─ Color interpolation
              ├─ Color correction
              ├─ Gamma correction
              ├─ Lens shading correction
              └─ Defect correction
```

The IFP can be bypassed to output raw Bayer data directly.

### Gain Encoding (MT9M111)

Gain formula: `Gain = (analog_gain + 1) × (1 + digital_gain/32)`

| Analog Gain | Register Value |
|-------------|----------------|
| 1.0x | 0x0000 |
| 2.0x | 0x0010 |
| 4.0x | 0x0030 |
| 8.0x | 0x0070 |

---

## 4. Register Page Organization Comparison

### MT9M111 Register Pages (via 0xF0)

| Page | Address Range | Description |
|------|---------------|-------------|
| Page 0 | 0x000-0x0FF | Sensor Core (gain, exposure, timing) |
| Page 1 | 0x100-0x1FF | Colorpipe (format, effects, scaling) |
| Page 2 | 0x200-0x2FF | Camera Controller (AE, AWB) |

### MT9M113 Register Pages

| Page | Address Range | Description |
|------|---------------|-------------|
| Page 0 (Sensor Core) | 0x3000-0x31FE | Sensor controls (gain, integration) |
| Page 1 (SOC1) | 0x3200-0x33FE | Color pipeline controls |
| Page 2 (SOC2) | 0x3400-0x35BE | MIPI, output FIFO, more color pipeline |
| SYSCTL | 0x0000-0x0028 | PLL, standby, reset |
| XDMA | 0x098C-0x099E | MCU variable access |

### MT9M114 Register Pages

| Page | Address Range | Description |
|------|---------------|-------------|
| Core | 0x3000-0x31FE | Similar to MT9M113 |
| SOC1 | 0x3200-0x33FE | Color pipeline |
| SOC2 | 0x3400-0x35FE | Additional SOC |
| SYSCTL | 0x0000-0x00FF | Extended system control |
| XDMA | 0x0982-0x099E | MCU variable access |
| **MIPI** | **0x3C40-0x3C52** | MIPI PHY control |

**Key Difference:** MT9M113 MIPI at 0x3400, MT9M114 MIPI at 0x3C40

---

## 5. XDMA Variable Access Architecture

### MT9M113 Variable Access (via 0x098C/0x0990)

```
R0x098C: MCU Variable Address
  [15]     - 8-bit/16-bit access (1=8-bit, 0=16-bit)
  [14:13]  - Access type (01=logical)
  [12:8]   - Driver ID (0-31)
  [7:0]    - Variable offset

R0x0990: MCU Variable Data (read/write)
```

**Example - Set ae.Target = 0x0050:**
```
R0x098C = 0xA206  // Driver 2, offset 6, logical, 8-bit
R0x0990 = 0x0050  // Value
```

### MT9M114 Variable Access (via 0x098E/0x0990)

```
R0x098E: Logical Address Access
  [15]     - 8-bit/16-bit access
  [14:10]  - Driver number (0-31)
  [9:0]    - Variable offset

R0x0990: MCU Variable Data

Additionally supports DIRECT ACCESS:
  Address = 0x8000 | (driver# << 10) | offset
```

**Example - Direct access to CamControl variable:**
```
// Direct read of 0xC868 (cam_output_width)
// = 0x8000 | (18 << 10) | 0x68
```

### Address Register Difference

| Register | MT9M113 | MT9M114 |
|----------|---------|---------|
| XDMA Address | **0x098C** | **0x098E** |
| XDMA Data | 0x0990 | 0x0990 |
| Logical bit position | [14:13]=01 | [14:10]=driver |
| Driver ID bits | [12:8] | [14:10] |
| Offset bits | [7:0] | [9:0] |

**Verdict:** XDMA addressing is INCOMPATIBLE - different address register and bit layouts

---

## 6. Driver ID Assignments

### MT9M113 Driver Numbers

| Driver# | Name | Description |
|---------|------|-------------|
| 0 | mon | Monitor (system info) |
| 1 | seq | Sequencer |
| 2 | ae | Auto Exposure |
| 3 | awb | Auto White Balance |
| 4 | fd | Flicker Detection |
| 7 | mode | Mode/Resolution control |
| 11 | hg | Histogram |

### MT9M114 Driver Numbers

| Driver# | Name | Base Address | Description |
|---------|------|--------------|-------------|
| 0 | Monitor | 0x8000 | Firmware version |
| 1 | Sequencer | 0x8400 | Sequencer control |
| 9 | AE_Rule | 0xA400 | AE algorithm rules |
| 10 | AE_Track | 0xA800 | AE tracking |
| 11 | AWB | 0xAC00 | Auto white balance |
| 12 | BlackLevel | 0xB000 | Black level correction |
| 13 | CCM | 0xB400 | Color correction matrix |
| 15 | LowLight | 0xBC00 | Low-light processing |
| 18 | CamControl | 0xC800 | Main camera control |
| 19 | UVC | 0xCC00 | USB Video Class |
| 23 | SysMgr | 0xDC00 | System state machine |
| 24 | PatchLoader | 0xE000 | Firmware patching |
| 31 | CmdHandler | 0xFC00 | Command processing |

### Driver Number Comparison

| Function | MT9M113 Driver# | MT9M114 Driver# |
|----------|-----------------|-----------------|
| Monitor | 0 | 0 |
| Sequencer | 1 | 1 |
| Auto Exposure | 2 | 9, 10 (split) |
| Auto White Balance | 3 | 11 |
| Flicker Detection | 4 | (integrated in AE) |
| Mode Control | 7 | 18 (CamControl) |
| Histogram | 11 | (integrated in LL) |

**Verdict:** Driver numbers are INCOMPATIBLE - completely different assignments

---

## 7. Command/State Machine Interface

### MT9M113: Sequencer Command (Driver 1, Variable)

```
seq_cmd (R0x0003 via XDMA):
  0x0000: Run
  0x0001: Go to preview mode
  0x0002: Go to capture mode
  0x0005: Refresh
  0x0006: Refresh mode

seq_state (R0x0004 via XDMA):
  0x0002: Enter preview
  0x0003: Preview
  0x0004: Leave preview
  0x0006: Enter capture
  0x0007: Capture
```

### MT9M114: Host Command Register (Hardware Register)

```
R0x0080: command_register
  [15] host_command_ok - Command completed
  [3]  WAIT_FOR_EVENT (0x0008)
  [2]  REFRESH (0x0004)
  [1]  SET_STATE (0x0002)
  [0]  APPLY_PATCH (0x0001)

sysmgr_next_state (0xDC00):
  0x28: ENTER_CONFIG_CHANGE
  0x31: STREAMING
  0x34: START_STREAMING
  0x40: ENTER_SUSPEND
  0x41: SUSPENDED
  0x50: ENTER_STANDBY
  0x52: STANDBY
  0x54: LEAVE_STANDBY
```

**Verdict:** Command interface is INCOMPATIBLE - firmware variables vs hardware register

---

## 8. Sensor Core Registers (0x3000+)

### Common Registers

| Address | MT9M113 Name | MT9M114 Name | Compatible? |
|---------|--------------|--------------|-------------|
| 0x3012 | coarse_integration_time | (similar) | Similar |
| 0x3014 | fine_integration_time | (similar) | Similar |
| 0x301E | data_pedestal | (similar) | Similar |
| 0x3028 | analog_gain_code_global | (similar) | Similar |
| 0x302A-0x3030 | analog_gain_code_* | (similar) | Similar |
| 0x3032-0x3038 | digital_gain_* | (similar) | Similar |

**Verdict:** Sensor core registers are LIKELY COMPATIBLE

---

## 9. PLL Configuration

### MT9M113 PLL Registers

| Address | Name | Description |
|---------|------|-------------|
| 0x0010 | pll_dividers | [13:8]=N, [7:0]=M |
| 0x0012 | pll_p_dividers | [13:12]=word_clk, [11:8]=P3, [7:4]=P2, [3:0]=P1 |
| 0x0014 | pll_control | [15]=lock, [1]=enable, [0]=bypass |

### MT9M114 PLL (via CAM Variables)

| Address | Name | Description |
|---------|------|-------------|
| 0xC97E | cam_sysctl_pll_enable | PLL enable |
| 0xC980 | cam_sysctl_pll_divider_m_n | M and N dividers |
| 0xC982 | cam_sysctl_pll_divider_p | P divider |

**PLL Formula (both):** `Fout = (Fin × 2 × M) / ((N+1) × (P+1))`

**Verdict:** PLL formula similar, but interface INCOMPATIBLE

---

## 10. MIPI Interface

### MT9M113 MIPI (0x3400)

| Address | Name | Key Bits |
|---------|------|----------|
| 0x3400 | mipi_control | [15:10]=data_type, [9]=mipi_en, [8:6]=chan_num |
| 0x3402 | mipi_status | [5]=rdy_for_data, [4]=idle, [0]=stdby |
| 0x3408 | line_byte_cnt | Line byte count |

**Data Types:**
- 0x1E: YUV422 8-bit
- 0x22: RGB565
- 0x2A: RAW8
- 0x2B: RAW10

### MT9M114 MIPI (0x3C40-0x3C52)

| Address | Name | Key Bits |
|---------|------|----------|
| 0x3C40 | mipi_control | Different bit layout |
| 0x3C44 | mipi_status | Different |
| 0xC984 | cam_port_output_control | Via CAM variables |

**Verdict:** MIPI registers at DIFFERENT ADDRESSES with DIFFERENT BIT LAYOUTS

---

## 11. Output Format Control

### MT9M113 Output Control

| Address | Name | Description |
|---------|------|-------------|
| 0x321C | output_control_status | Output select, FIFO control |
| 0x321E | output_control_status_2 | Pixel clock gating |
| 0x337C | yuv_ycbcr_control | YUV clip, offset, normalize |

### MT9M114 Output Control

| Address | Name | Description |
|---------|------|-------------|
| 0xC86C | cam_output_format | Format selection |
| 0xC86E | cam_output_format_yuv | YUV-specific options |
| 0xC870 | cam_output_y_offset | Y offset |

**Verdict:** Output format control is INCOMPATIBLE - different register locations

---

## 12. Auto Exposure Comparison

### MT9M113 AE (Driver 2) - Complete Variable List

| Offset | Default | Name | Description |
|--------|---------|------|-------------|
| 0x02 | 0x0000 | ae_window_pos | [7:4]=Y0, [3:0]=X0 in 1/16 frame units |
| 0x03 | 0x00FF | ae_window_size | [7:4]=height-1, [3:0]=width-1 in 1/16 frame units |
| 0x07 | 0x0004 | ae_gate | AE sensitivity (changes take effect immediately) |
| 0x0B | 0x0000 | ae_min_index | Min zone number (needs REFRESH) |
| 0x0C | 0x0018 | ae_max_index | Max zone number / max integration time (needs REFRESH) |
| 0x0D | 0x0010 | ae_min_virt_gain | Min virtual gain (gain = value/0x20) |
| 0x0E | 0x0080 | ae_max_virt_gain | Max virtual gain (0x20=1.0, 0x80=4.0) |
| 0x17 | 0x0000 | ae_status | [2]=ready, [1]=r9_changed, [0]=at_limit |
| 0x18 | 0x004B | ae_current_y | Last measured luminance (RO) |
| 0x19 | 0x0279 | ae_r12 | Current shutter delay (RO) |
| 0x1B | 0x0004 | ae_index | Current zone/integration time (RO) |
| 0x1C | 0x0010 | ae_virt_gain | Current virtual gain (RO) |
| 0x22 | 0x0010 | ae_r9 | Current integration time value |
| 0x2D | 0x009D | ae_r9_step | Integration time of one zone |
| 0x4A | 0x0032 | ae_target_min | Min target (50) |
| 0x4B | 0x0096 | ae_target_max | Max target (150) |
| 0x4C | 0x000C | ae_target_buffer_speed | Max change (0x20=fastest, 0x01=slowest) |
| 0x4F | 0x0036 | ae_base_target | Base target luma (54) |

### MT9M114 AE (Driver 9+10 + CAM)

| Address | Name | Description |
|---------|------|-------------|
| 0xA804 | ae_track_algo | AE algorithm |
| 0xA80A | ae_track_ae_tracking_dampening | Tracking speed |
| 0xC87A | cam_aet_target_average_luma | Target luma |
| 0xC88C | cam_aet_max_frame_rate | Max FPS |
| 0xC88E | cam_aet_min_frame_rate | Min FPS |

**Key Difference:** MT9M113 uses zone-based AE with ae_index/ae_r9_step, MT9M114 uses frame-rate based AE

**Verdict:** AE interface is INCOMPATIBLE - different driver numbers and variable layout

---

## 13. Standby/Power Control

### MT9M113 Standby

| Address | Name | Description |
|---------|------|-------------|
| 0x0018 | standby_control_and_status | [14]=standby_done, [3]=en_xirq, [0]=standby_i2c |
| 0x0028 | hard_standby_sel | Hard standby mode select |

### MT9M114 Standby

| Address | Name | Description |
|---------|------|-------------|
| 0xDC00 | sysmgr_next_state | Set to 0x50 (ENTER_STANDBY) |
| 0xDC01 | sysmgr_current_state | Read to verify 0x52 (STANDBY) |

**Verdict:** Standby interface is INCOMPATIBLE

---

## 14. Auto White Balance Comparison

### MT9M113 AWB (Driver 3) - Complete Variable List

| Offset | Default | Name | Description |
|--------|---------|------|-------------|
| 0x02 | 0x0000 | awb_window_pos | [7:4]=Y0, [3:0]=X0 in 1/16 frame units |
| 0x03 | 0x00EF | awb_window_size | [7:4]=height-1, [3:0]=width-1 in 1/16 frame units |
| 0x06-0x16 | varies | awb_ccm_l_0-8 | Left CCM matrix (3x3), 0x100=1.0 |
| 0x18-0x1A | varies | awb_ccm_l_9-10 | Left CCM R/G and B/G ratios (0x20=1.0) |
| 0x1C-0x30 | varies | awb_ccm_rl_0-10 | Delta CCM (interpolation from left to right) |
| 0x32-0x46 | varies | awb_ccm_0-10 | Current CCM (calculated) |
| 0x48 | 0x0008 | awb_gain_buffer_speed | Buffering speed (0x20=fastest, 0x01=slowest) |
| 0x49 | 0x0002 | awb_jump_divisor | Steps to reach color target |
| 0x4A | 0x0059 | awb_gain_min_r | Min R gain (0x80=1.0) |
| 0x4B | 0x00A6 | awb_gain_max_r | Max R gain |
| 0x4C | 0x0059 | awb_gain_min_b | Min B gain |
| 0x4D | 0x00A6 | awb_gain_max_b | Max B gain |
| 0x4E | 0x0080 | awb_gain_r | Current R gain |
| 0x4F | 0x0080 | awb_gain_g | Current G gain |
| 0x50 | 0x0080 | awb_gain_b | Current B gain |
| 0x51 | 0x0000 | awb_ccm_position_min | Leftmost CCM position (incandescent) |
| 0x52 | 0x007F | awb_ccm_position_max | Rightmost CCM position (daylight) |
| 0x53 | 0x0040 | awb_ccm_position | Current position (0x00=incandescent, 0x7F=daylight) |
| 0x54 | 0x0080 | awb_saturation | CCM saturation (0x80=100%) |
| 0x55 | 0x0000 | awb_mode | [6]=normccm_off, [5]=force_unit_dgains |
| 0x5D-0x60 | varies | awb_steady_b_gain_* | Blue gain thresholds for search |
| 0x65-0x6B | varies | awb_kr/kg/kb_l/r | CCM normalization coefficients |

### MT9M114 AWB (Driver 11)

MT9M114 uses a different CCM architecture with separate drivers for AWB control and CCM coefficients.

**Key Difference:** MT9M113 has integrated CCM in AWB driver with interpolation, MT9M114 separates AWB and CCM

---

## 15. Mode/Resolution Control Comparison

### MT9M113 Mode Driver (Driver 7) - Context A/B Variables

| Offset | Default | Name | Description |
|--------|---------|------|-------------|
| 0x03 | 0x0280 | mode_output_width_a | Context A output width (640 default) |
| 0x05 | 0x0200 | mode_output_height_a | Context A output height (512 default) |
| 0x07 | 0x0500 | mode_output_width_b | Context B output width (1280 default) |
| 0x09 | 0x0400 | mode_output_height_b | Context B output height (1024 default) |
| 0x0D | 0x0000 | mode_sensor_row_start_a | Context A first row |
| 0x0F | 0x0000 | mode_sensor_col_start_a | Context A first column |
| 0x11 | 0x04BD | mode_sensor_row_end_a | Context A last row |
| 0x13 | 0x064D | mode_sensor_col_end_a | Context A last column |
| 0x15 | 0x2112 | mode_sensor_row_speed_a | [2:0]=pix_clk_speed divider |
| 0x17 | 0x046C | mode_sensor_read_mode_a | [11]=x_bin, [10]=xy_bin, [7:5]=x_odd_inc, [4:2]=y_odd_inc, [1]=vflip, [0]=hmirror |
| 0x1F | 0x0293 | mode_sensor_frame_length_a | Context A total lines |
| 0x21 | 0x07D0 | mode_sensor_line_length_pck_a | Context A line length in PIXCLKs |
| 0x55 | 0x0000 | mode_output_format_a | [8]=processed_bayer, [7:6]=rgb_format, [5]=rgb_mode, [3]=mono, [1]=swap_CrY, [0]=swap_RB |
| 0x59 | 0x6440 | mode_spec_effects_a | [15:8]=solar_thresh, [6]=dither_luma, [5:3]=dither_width, [2:0]=effect |

**Output Formats:**
- RGB565, RGB555, RGB444x, RGBx444
- YUV422 (default)
- Processed Bayer

**Special Effects:**
- 0=Disabled, 1=Mono, 2=Sepia, 3=Negative, 4=Solarization, 5=Solarization-UV

### MT9M114 CamControl (Driver 18)

MT9M114 uses cam_output_width/height at 0xC868/0xC86A with different format control.

**Key Difference:** MT9M113 uses context A/B paradigm (preview/capture), MT9M114 has single-context streaming model

---

## 16. Histogram/Low-Light Comparison

### MT9M113 Histogram Driver (Driver 11)

| Offset | Default | Name | Description |
|--------|---------|------|-------------|
| 0x04 | 0x0040 | hg_max_dark_level | Max dark level |
| 0x06 | 0x0003 | hg_black_clip_percent | Black clip % × 10 (0.3% default) |
| 0x08 | 0x0010 | hg_dark_level | Dark level adjustment |
| 0x1B | 0x0000 | hg_brightness_metric | Scene brightness (increases as darker) |
| 0x1F | 0x0084 | hg_llmode | Low-light mode control |
| 0x28 | 0x157C | hg_ll_brightness_start | Start applying LL settings |
| 0x2A | 0x1B58 | hg_ll_brightness_stop | Full LL settings applied |
| 0x37 | 0x0003 | hg_gamma_morph_ctrl | [1:0]=gamma morph mode (0=off, 1=A, 2=B, 3=auto) |
| 0x38 | 0x157C | hg_gamma_start_morph | Start gamma morphing |
| 0x3A | 0x1B58 | hg_gamma_stop_morph | End gamma morphing |
| 0x3C-0x4E | varies | hg_gamma_table_a_0-18 | Gamma table A (19 entries, normal light) |
| 0x4F-0x61 | varies | hg_gamma_table_b_0-18 | Gamma table B (19 entries, low light) |

### MT9M114 Low-Light (Driver 15)

MT9M114 has a separate LowLight driver with similar gamma morphing capability.

**Key Difference:** Similar concept but different driver number and variable layout

---

## 17. Anti-Flicker Detection Comparison

### MT9M113 Flicker Detection (Driver 4)

| Offset | Default | Name | Description |
|--------|---------|------|-------------|
| 0x02 | 0x001D | fd_window_posh | [7:4]=X0, [3:0]=width-1 |
| 0x03 | 0x0004 | fd_window_height | Window height in rows |
| 0x04 | 0x0000 | fd_mode | [7]=manual, [6]=50/60Hz, [5]=current state (RO) |
| 0x08 | 0x001E | fd_search_f1_50 | 50Hz period lower limit |
| 0x09 | 0x0020 | fd_search_f2_50 | 50Hz period upper limit |
| 0x0A | 0x0025 | fd_search_f1_60 | 60Hz period lower limit |
| 0x0B | 0x0027 | fd_search_f2_60 | 60Hz period upper limit |
| 0x0D | 0x0003 | fd_stat_min | Min frames with flicker to detect |
| 0x0E | 0x0005 | fd_stat_max | Total frames to check |
| 0x10 | 0x0005 | fd_min_amplitude | Min signal amplitude |
| 0x11 | 0x009D | fd_r9_step_f60_a | 60Hz shutter step context A |
| 0x13 | 0x00BC | fd_r9_step_f50_a | 50Hz shutter step context A |
| 0x15 | 0x0000 | fd_r9_step_f60_b | 60Hz shutter step context B |
| 0x17 | 0x00E0 | fd_r9_step_f50_b | 50Hz shutter step context B |

### MT9M114 Flicker Detection

MT9M114 integrates flicker detection into AE driver.

**Key Difference:** MT9M113 has separate FD driver, MT9M114 integrates in AE

---

## 18. GPIO Configuration

### MT9M113 GPIO (MCU SFR Space)

| Address | Default | Name | Description |
|---------|---------|------|-------------|
| 0x1070 | 0x0000 | gpio_data | [12:8]=GPIO[4:0] data |
| 0x1074 | 0x0000 | gpio_output_set | Write 1 to set output HIGH |
| 0x1076 | 0x0000 | gpio_output_clear | Write 1 to set output LOW |
| 0x1078 | 0x1F00 | gpio_dir | [12]=flash, [11]=GP3, [10]=OE_BAR, [9]=DOUT_LSB1, [8]=DOUT_LSB0 |

**GPIO Functions:**
- GPIO[4]: Flash output (controlled by sequencer)
- GPIO[3]: General-purpose I/O
- GPIO[2]: OE_BAR (output enable, active low)
- GPIO[1]: DOUT_LSB1 (10-bit output)
- GPIO[0]: DOUT_LSB0 (10-bit output)

### MT9M114 GPIO

GPIO control through different mechanism (CAM variables or direct registers)

**Verdict:** GPIO access mechanism differs

---

## 19. Timing Specifications Comparison

### Power-Up Sequence (MT9M113)

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| VDD_IO to VAA/VAA_PIX delay (t1) | 0 | - | 500 | ms |
| VDD_IO to VDD delay (t2) | 0 | - | 500 | ms |
| EXTCLK activation (t4) | - | 500 | - | ms |
| RESET_BAR init time (t6) | - | 2 | - | ms |
| RESET_BAR pulse width (t5) | - | 70 | 1 | EXTCLKs |
| First serial write (t7) | - | 100 | - | EXTCLKs |

### Reset Timing (MT9M113)

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| Hard reset pulse width | 70 | - | - | EXTCLKs |
| Active EXTCLK after reset | 10 | - | - | EXTCLKs |
| ROM read time | - | - | 6000 | EXTCLKs |
| Soft reset time | - | 6000 | - | EXTCLKs |

### Clock Specifications (MT9M113)

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| EXTCLK frequency (PLL enabled) | 8 | - | 54 | MHz |
| EXTCLK duty cycle | 40 | 50 | 60 | % |
| PIXCLK frequency | 30 | - | 60 | MHz |
| I2C SCLK frequency | 100 | - | 400 | kHz |

### Standby Timing (MT9M113)

**Hard Standby:** Uses STANDBY pin, loses all settings, equivalent to power-up on exit

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| Standby effective delay (t1) | 20 | - | - | EXTCLKs |
| STANDBY pulse width (t4) | 100 | - | - | EXTCLKs |

**Soft Standby:** Via register (0x0018[0]=1), preserves settings, I2C active

| Parameter | Min | Typ | Max | Units |
|-----------|-----|-----|-----|-------|
| Standby entry complete (t1) | - | 20 | - | EXTCLKs |
| Minimum standby time (t4) | - | 100 | - | EXTCLKs |

---

## 20. Power Consumption Comparison (MT9M113)

### Parallel Mode (fEXTCLK=27MHz, fPIXCLK=60MHz, 2.8V I/O)

| Mode | Total Current | Total Power |
|------|---------------|-------------|
| Context A (preview) / EXTCLK | 37 mA | 90 mW |
| Context B (capture) / EXTCLK | 61 mA | 146 mW |
| Context A / PLL | 62 mA | 156 mW |
| Context B / PLL | 89 mA | 222 mW |

### Supply Voltages

| Supply | Min | Typ | Max |
|--------|-----|-----|-----|
| VDD (core digital) | 1.7V | 1.8V | 1.95V |
| VDD_IO (I/O) | 1.7V / 2.5V | 1.8V / 2.8V | 1.95V / 3.1V |
| VAA (analog) | 2.5V | 2.8V | 3.1V |
| VAA_PIX (pixel) | 2.5V | 2.8V | 3.1V |
| VDD_PLL | 2.5V | 2.8V | 3.1V |

---

## 21. Similarities (Potentially Compatible Areas)

1. **I2C Protocol** - Same 16-bit address, 16-bit data format
2. **Sensor Core Registers (0x3000+)** - Similar gain, integration time registers
3. **Basic Register Structure** - Both use page-based organization
4. **MCU Architecture** - Both have embedded MCU with firmware drivers
5. **PLL Formula** - Same calculation formula
6. **Gamma Tables** - Both support 19-entry gamma tables with morphing
7. **CCM Concept** - Both use left/right matrix interpolation for AWB

---

## 22. Sequencer State Machine Comparison

### MT9M113 Sequencer States (Driver 1)

Each state configures AE, FD, AWB, HG, Flash, and SkipFrame independently:

| State | Name | Description |
|-------|------|-------------|
| seq_preview_0_* | Preview-Enter | Transition into preview |
| seq_preview_1_* | Preview | Active preview state |
| seq_preview_2_* | Preview-Leave | Transition out of preview |
| seq_preview_3_* | Capture-Enter | Transition into capture |

**State Configuration Variables (for each state):**
- `_ae`: AE mode (0=off, 1=DR track, 2=brightness track, 3=rules only)
- `_fd`: Flicker (0=off, 1=on, 2=manual)
- `_awb`: AWB (0=off, 1=on)
- `_hg`: Histogram (0=off, 1=on)
- `_flash`: Flash config (0=off, 1=on, 2=locked, 3=auto)
- `_skipframe`: Frame skip control

### MT9M114 State Machine

MT9M114 uses SysMgr (Driver 23) with different state values and hardware command register.

**Key Difference:** MT9M113 states are driver variables, MT9M114 uses hardware command interface

---

## 23. Conclusion: Migration Strategy

### If Porting MT9M114 Driver to Support MT9M113:

1. **XDMA Access** - Must change address register from 0x098E to 0x098C
2. **Driver Numbers** - Complete remapping required:
   - AE: 9,10 → 2
   - AWB: 11 → 3
   - FD: (integrated) → 4
   - Mode: 18 → 7
   - Histogram: (LL) → 11
3. **Command Interface** - Must use seq_cmd variable instead of 0x0080 register
4. **MIPI Registers** - Different base address (0x3400 vs 0x3C40)
5. **Variable Layout** - Different offsets within drivers
6. **State Machine** - Different state values and transitions
7. **Context Model** - MT9M113 uses A/B contexts, MT9M114 uses single-stream

### Recommendation

The MT9M113 and MT9M114 are **NOT code-compatible**. A driver supporting both would need:
- Chip ID detection at probe time (0x2480 for MT9M113)
- Completely separate initialization sequences
- Different register definitions for most functions
- Only sensor core registers could potentially share code

### Shared Code Opportunities

1. I2C communication layer
2. Sensor core gain/integration calculations
3. General frame format calculations
4. Gamma table handling concepts (19-entry tables)
5. CCM interpolation algorithms (conceptually similar)

---

## 24. Quick Reference: Critical Register Differences

### Three-Way Comparison

| Function | MT9M111 | MT9M113 | MT9M114 |
|----------|---------|---------|---------|
| Chip ID Register | 0x00 (page 0) | 0x0000 | 0x0000 |
| Expected Chip ID | **0x143A** | **0x2480** | (different) |
| Architecture | Hardware IFP | MCU + Firmware | MCU + Firmware |
| Register Access | Page Map (0xF0) | XDMA (0x098C) | XDMA (0x098E) |
| XDMA Data | N/A | 0x0990 | 0x0990 |
| Command Interface | Direct registers | seq_cmd (driver 1) | 0x0080 |
| MIPI Control | N/A (parallel) | 0x3400 | 0x3C40 |
| Output Format | 0x108 (page 1) | 0x321C | 0xC86C (CAM) |
| Gain Registers | 0x2B-0x2F (page 0) | Driver 2 vars | Driver 9,10 vars |
| Context Support | A/B | A/B | Single-stream |
| Linux Driver | `mt9m111.c` | (custom) | `mt9m114.c` |

### MT9M111 Key Registers (Summary)

| Page | Register | Name | Description |
|------|----------|------|-------------|
| 0 | 0x00 | CHIP_VERSION | 0x143A |
| 0 | 0x09 | SHUTTER_WIDTH | Exposure time |
| 0 | 0x2F | GLOBAL_GAIN | Analog gain |
| 0 | 0xC8 | CONTEXT_CONTROL | A/B selection |
| 0 | 0xF0 | PAGE_MAP | Page select |
| 1 | 0x06 | OPER_MODE_CTRL | AE/AWB enable |
| 1 | 0x08 | OUTPUT_FORMAT_CTRL | Format select |

### MT9M113/MT9M114 Only

| Function | MT9M113 | MT9M114 |
|----------|---------|---------|
| PLL Control | 0x0010-0x0014 | 0xC97E-0xC982 (CAM) |
| Standby | 0x0018 | 0xDC00 (SysMgr) |
| AE Driver | 2 | 9, 10 |
| AWB Driver | 3 | 11 |
| FD Driver | 4 | (in AE) |
| Mode Driver | 7 | 18 (CamControl) |
| Histogram | 11 | 15 (LowLight) |

---

## 25. XDMA Variable Access Examples

### MT9M113 Variable Access

To read/write MT9M113 firmware variables via XDMA:

```
// Write ae_base_target (Driver 2, offset 0x4F) = 0x0036
// Logical address format: [15]=8bit, [14:13]=01(logical), [12:8]=driver, [7:0]=offset
// = 0x8000 | 0x2000 | (2<<8) | 0x4F = 0xA24F

W R0x098C = 0xA24F   // Set address (8-bit logical, driver 2, offset 0x4F)
W R0x0990 = 0x0036   // Write data

// Read ae_current_y (Driver 2, offset 0x18) - read-only
W R0x098C = 0xA218   // Set address (8-bit logical, driver 2, offset 0x18)
R R0x0990           // Read data (returns current luminance)

// 16-bit access: mode_output_width_a (Driver 7, offset 0x03)
W R0x098C = 0x2703   // Set address (16-bit logical, driver 7, offset 0x03)
R R0x0990           // Returns 0x0280 (640)
```

### MT9M114 Variable Access

MT9M114 uses direct addressing (0x8000+) or XDMA via 0x098E:

```
// Direct access to cam_output_width (0xC868)
W R0xC868 = 0x0280   // Direct write

// Or via XDMA
W R0x098E = 0xC868   // Set logical address
W R0x0990 = 0x0280   // Write data
```

---

## 26. Default Resolution Comparison

### MT9M113 Default Modes

| Context | Resolution | Binning | Notes |
|---------|------------|---------|-------|
| A (Preview) | 640×512 | 2×2 XY | Default preview |
| B (Capture) | 1280×1024 | None | Full resolution |

**Sensor Array:** 1280×1024 active pixels (1.3MP SXGA)

### MT9M114 Default Modes

| Mode | Resolution | Notes |
|------|------------|-------|
| Default | 1280×720 | 720p streaming |

**Sensor Array:** 1280×960 active pixels (720p optimized)

---

*Report generated for HP TouchPad camera driver development*
*Last updated: 2026-03-10 with Part 3 datasheet information*
