# WebOS Audio Configuration Analysis Report

**Date:** 2026-03-03
**Purpose:** Compare working webOS audio configuration with mainline Linux 6.18 to identify why audio output is not working.

## Executive Summary

Analysis of webOS audio configuration revealed several key differences from our mainline implementation:

1. **webOS uses PRIMARY I2S (codec_i2s_spkr), NOT MI2S** - The mi2s_bit_clk is disabled
2. **GPIO 109 is labeled "MSM_SNDDEV_RX_MCLK"** - Used as master clock for sound device
3. **Class-D amplifier GPIOs are LOW in idle state** - Not HIGH as we assumed
4. **Codec is on I2C bus 4 at address 0x1a** - Different from mainline bus 1

## Detailed Findings

### 1. Clock Configuration

**webOS Clock State (from `/sys/kernel/debug/clk/clk_summary`):**

| Clock | Rate | Enabled |
|-------|------|---------|
| codec_i2s_spkr_bit_clk | 8 Hz | Yes (1) |
| codec_i2s_spkr_osr_clk | 12,288,000 Hz | Yes (1) |
| mi2s_bit_clk | 0 Hz | No (0) |

**Critical Finding:** webOS uses `codec_i2s_spkr_*` clocks (PRIMARY_I2S interface), NOT MI2S. The MI2S bit clock is completely disabled (rate=0, enable=0).

This maps to the PRIMARY_I2S interface on GPIOs 108-110, not the MI2S interface on GPIOs 101-107.

### 2. GPIO Configuration

**I2S-Related GPIOs (from webOS `/sys/kernel/debug/gpio`):**

| GPIO | Label | Direction | Value | Notes |
|------|-------|-----------|-------|-------|
| 107 | mt9m113 | out | hi | Webcam power down (NOT audio!) |
| 109 | MSM_SNDDEV_RX_MCLK | in | lo | Sound device master clock |

**Class-D Amplifier GPIOs:**

| GPIO | PM Chip | MPP | Direction | Value | Notes |
|------|---------|-----|-----------|-------|-------|
| 223 | PM8058 | MPP11 | sink | lo | Class-D amp enable (LEFT?) |
| 225 | PM8901 | MPP1 | d_out | lo | Class-D amp enable (RIGHT?) |
| 227 | - | - | - | - | Additional amp GPIO |

**Key Observation:** In webOS idle state, Class-D amplifier GPIOs are LOW, not HIGH. This differs from our mainline implementation which sets them HIGH at boot.

### 3. GPIO Function Mapping (from pinctrl-msm8660.c)

**PRIMARY_I2S GPIOs (i2s function):**
- GPIO 108: I2S_WS (Word Select / LRCK)
- GPIO 109: I2S_SCK (Bit Clock)
- GPIO 110: I2S_DATA (likely SD0)
- GPIO 115-122: Additional I2S pins

**MI2S GPIOs (mi2s function):**
- GPIO 101: MI2S_WS
- GPIO 102: MI2S_SCK
- GPIO 103: MI2S_SD0
- GPIO 104: MI2S_SD1
- GPIO 105: MI2S_SD2
- GPIO 106: MI2S_SD3
- GPIO 107: MI2S_SD3 (alternate, conflicts with webcam!)

### 4. I2C Codec Location

**webOS:**
```
/sys/bus/i2c/devices/4-001a/
```
- I2C bus 4, address 0x1a (26 decimal)
- Uses GSBI4 in I2C mode

**Mainline:**
- I2C bus 1, address 0x1a
- Uses GSBI3 in I2C mode

### 5. ALSA Device Structure

**webOS `/proc/asound/devices`:**
```
  1:        : sequencer
  2: [ 0- 0]: digital audio playback
  3: [ 0- 0]: digital audio capture
  4: [ 0- 1]: digital audio playback
  5: [ 0- 2]: digital audio capture
  6: [ 0- 0]: hardware dependent
  7: [ 0]   : control
  8: [ 1- 3]: digital audio playback
  9: [ 1- 0]: hardware dependent
 10: [ 1]   : control
 33:        : timer
```

Two sound cards are present:
- Card 0: Multiple playback/capture devices (likely LPASS/Q6)
- Card 1: Device 3 playback (likely codec direct?)

### 6. webOS Kernel GPIO Configuration (from gpiomux-tenderloin.c)

```c
// I2S pins (GPIO 108, 109) - FUNC_1 = I2S function
static struct msm_gpiomux_config msm8x60_snd_configs[] = {
    { .gpio = 108, ... FUNC_1 },  // I2S_WS
    { .gpio = 109, ... FUNC_1 },  // I2S_SCK/MCLK
};

// MI2S pins (GPIO 101-103, 107) - FUNC_1 = MI2S function
static struct msm_gpiomux_config msm8x60_mi2s_configs[] = {
    { .gpio = 101, ... FUNC_1 },  // MI2S_WS
    { .gpio = 102, ... FUNC_1 },  // MI2S_SCK
    { .gpio = 103, ... FUNC_1 },  // MI2S_SD0
    { .gpio = 107, ... FUNC_1 },  // MI2S_SD3 (conflicts with webcam!)
};

// Webcam power down
#define TENDERLOIN_WEBCAM_PWDN 107  // Same GPIO as MI2S_SD3!
```

## Comparison: webOS vs Mainline

| Aspect | webOS | Mainline | Status |
|--------|-------|----------|--------|
| I2S Interface | PRIMARY_I2S (codec_i2s_spkr) | Attempting MI2S | **WRONG** |
| I2S Clock Source | codec_i2s_spkr_bit_clk @ 8Hz | mi2s_bit_clk | **WRONG** |
| I2S OSR Clock | codec_i2s_spkr_osr_clk @ 12.288MHz | - | Needs verification |
| Data GPIO | GPIO 110 (i2s function) | GPIO 110 (recently added) | Possibly correct |
| WS/LRCK GPIO | GPIO 108 | GPIO 108 | Correct |
| SCK/MCLK GPIO | GPIO 109 | GPIO 109 | Correct |
| Class-D GPIOs | LOW in idle | HIGH at boot | **DIFFERENT** |
| I2C Bus | Bus 4 (GSBI4) | Bus 1 (GSBI3) | Different but working |

## Root Cause Analysis

### Primary Issue: Wrong I2S Interface

Mainline is configured for MI2S but webOS uses PRIMARY_I2S (codec_i2s_spkr). The clock trees are different:

**MI2S Clock Path:**
```
mi2s_bit_clk → MI2S controller → GPIO 101-107
```

**PRIMARY_I2S Clock Path (what webOS uses):**
```
codec_i2s_spkr_bit_clk → LPA/LPASS I2S controller → GPIO 108-110
```

The WM8994 codec is connected to PRIMARY_I2S (GPIO 108-110), not MI2S.

### Secondary Issue: Class-D Amplifier Polarity

Our mainline sets Class-D amp GPIOs HIGH at boot, but webOS shows them LOW in idle. This could mean:
1. Active-low enables (HIGH = disabled)
2. Dynamic control (only enabled during playback)

## Recommendations

### 1. Fix I2S Interface Selection

The LPASS/Q6 audio driver needs to use PRIMARY_I2S clocks:
- `codec_i2s_spkr_bit_clk` instead of `mi2s_bit_clk`
- `codec_i2s_spkr_osr_clk` for oversampling

### 2. Verify Clock Configuration in Device Tree

Ensure the sound card node references the correct clocks:
```dts
clocks = <&lcc CODEC_I2S_SPKR_BIT_CLK>,
         <&lcc CODEC_I2S_SPKR_OSR_CLK>;
```

### 3. Review Class-D Amplifier Control

Investigate whether Class-D amp GPIOs should be:
- Actively driven by DAPM (Dynamic Audio Power Management)
- Only enabled during actual audio playback
- Inverted polarity (active-low)

### 4. Capture Active Playback State

Still needed: GPIO and codec register capture during ACTIVE audio playback in webOS to see:
- Which GPIOs change state
- Actual clock rates during playback
- Class-D amp behavior

## Files to Modify

1. **`arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`**
   - Update clock references to use codec_i2s_spkr clocks
   - Verify I2S pinctrl configuration

2. **`sound/soc/qcom/` driver files**
   - Ensure PRIMARY_I2S path is used, not MI2S

3. **Class-D amplifier GPIO handling**
   - Review polarity and DAPM integration

## Next Steps

1. Review the LCC (LPASS Clock Controller) driver to understand codec_i2s_spkr vs mi2s clocks
2. Check the Q6 LPASS driver for I2S interface selection
3. Test with Class-D GPIOs set LOW instead of HIGH
4. Capture webOS state during active playback (requires longer audio file or loop)

## Appendix: Raw Data Captures

### webOS GPIO Dump (Idle State)
```
gpio-107 (mt9m113            ) out hi
gpio-109 (MSM_SNDDEV_RX_MCLK ) in  lo
gpio-223 (PM8058 MPP11       ) sink lo
gpio-225 (PM8901 MPP1        ) d_out lo
```

### webOS Clock Summary (Idle State)
```
codec_i2s_spkr_bit_clk    8          1
codec_i2s_spkr_osr_clk    12288000   1
mi2s_bit_clk              0          0
```
