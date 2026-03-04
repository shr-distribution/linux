# WebOS vs Mainline Audio Configuration Comparison

**Date:** 2026-03-04
**Device:** HP TouchPad Topaz 3G running webOS with working audio

## Summary

Captured working audio configuration from webOS during active video playback and compared with our mainline implementation.

## Key Register Comparison

| Register | Description | WebOS (working) | Mainline | Notes |
|----------|-------------|-----------------|----------|-------|
| 0x01 (PM1) | Power Management 1 | 0x0323 | Need to verify | HP outputs + bias enabled |
| 0x03 (PM3) | Power Management 3 | 0x3CF0 | Need to verify | All LINEOUTs + mixers ON |
| 0x05 (PM5) | Power Management 5 | 0x0303 | Need to verify | DAC1 + AIF1DAC1 enabled |
| 0x2d (Out Mixer 1) | Output Mixer 1 | 0x0001 | 0x0001 | DAC to left output |
| 0x2e (Out Mixer 2) | Output Mixer 2 | 0x0001 | 0x0001 | DAC to right output |
| 0x34 (Line Mixer 1) | Line Mixer 1 | 0x0001 | 0x0040? | Different routing? |
| 0x35 (Line Mixer 2) | Line Mixer 2 | 0x0001 | 0x0040? | Different routing? |
| 0x200 (AIF1 Clocking) | AIF1 Clock | 0x0011 | Need to verify | AIF1CLK enabled, div2 |
| 0x220 (FLL1 Control) | FLL1 Enable | 0x0001 | 0x0001 | FLL1 enabled |
| 0x224 (FLL1 Control 5) | FLL1 Source | 0x0C83 | Need to verify | MCLK1 source |
| 0x420 (AIF1DAC1 Filters) | DAC Mute | 0x0000 | 0x0000 | Unmuted during playback |
| 0x700 (GPIO1) | Amp Enable | 0x0001 | 0x0041 | Both output HIGH |
| 0x731 (Int Status 2) | FLL Lock | 0x5062 | Need to verify | FLL1 locked (bit 12) |

## Critical Findings

### 1. Class-D Amplifier GPIO Configuration

**WebOS uses GPIO 227 and 229:**
```c
#define SNDDEV_GPIO_CLASS_D0_EN 227  // PM8901 MPP2
#define SNDDEV_GPIO_CLASS_D1_EN 229  // I2C GPIO Expander
```

**Our mainline uses different GPIOs:**
```dts
class-d0-gpios = <&pm8058_mpps 11 GPIO_ACTIVE_HIGH>;  // GPIO 223
class-d1-gpios = <&pm8901_mpps 1 GPIO_ACTIVE_HIGH>;   // GPIO 225
```

**GPIO Debug During Playback:**
All Class-D GPIOs show LOW in webOS during active playback:
- gpio-223 (PM8058 MPP11): sink lo
- gpio-225 (PM8901 MPP1): d_out lo
- gpio-227 (PM8901 MPP3): d_out lo

This suggests the external Class-D amplifiers may be controlled ONLY by WM8994 GPIO1, not by PM8058/PM8901 MPPs.

### 2. WM8994 GPIO1 (Amp Enable)

WebOS register 0x700 = 0x0001:
- GP1_FN bits [4:0] = 0x01 = Logic level output, drive HIGH
- This should enable the external amplifier

Our mainline writes 0x0041:
- GP1_DIR bit 6 = 0x40 = Output direction
- GP1_FN bits [4:0] = 0x01 = Logic level output, drive HIGH
- Both should work identically

### 3. I2S Clock Configuration

WebOS uses:
- GPIO 108 = msm_snddev_tx_mclk
- GPIO 109 = msm_snddev_rx_mclk (shown in GPIO debug)
- codec_i2s_spkr clocks (PRIMARY I2S interface)

Our mainline (after LCC fix):
- GPIO 108, 109, 110 in I2S mode
- codec_i2s_spkr_osr_clk @ 12.288 MHz
- codec_i2s_spkr_bit_clk @ 1.536 MHz

### 4. FLL Configuration

WebOS FLL registers during playback:
- 0x220 = 0x0001 (FLL1 enabled)
- 0x221 = 0x0700 (FLL1_OUTDIV)
- 0x224 = 0x0C83 (FLL1 reference from MCLK)
- 0x731 bit 12 = 1 (FLL1 locked)

Our mainline uses LRCLK as FLL source (WM8994_FLL_SRC_LRCLK).

### 5. DAPM Widget States

All key widgets ON during webOS playback:
- AIF1CLK: On
- CLK_SYS: On
- AIF1DAC1L/R: On
- DAC1L/R: On
- Left/Right Output Mixer: On
- Left/Right Output PGA: On
- LINEOUT1P/1N/2P/2N: On
- LINEOUT1P/1N/2P/2N Driver: On
- Speaker: On

## Recommendations

### 1. Remove External Class-D GPIO Control

The PM8058/PM8901 MPP GPIOs may not be needed. WM8994 GPIO1 appears to be the sole amplifier control:

```dts
// Remove these from device tree:
// class-d0-gpios = <&pm8058_mpps 11 GPIO_ACTIVE_HIGH>;
// class-d1-gpios = <&pm8901_mpps 1 GPIO_ACTIVE_HIGH>;
```

### 2. Verify WM8994 GPIO1 Configuration

Ensure GPIO1 is set to 0x0001 (or 0x0041) to output HIGH:
```c
snd_soc_component_write(component, WM8994_GPIO_1, 0x0001);
```

### 3. Compare FLL Configuration

WebOS uses MCLK as FLL source (0x0C83), we use LRCLK. Consider testing with MCLK source.

### 4. Verify Line Mixer Routing

WebOS shows 0x34 and 0x35 = 0x0001, but we write 0x0040. The routing may work differently.

## Legacy Kernel Board File Analysis

### GPIO 110 is NOT Used for I2S Audio

Analysis of `gpiomux-tenderloin.c` shows:

```c
/* Sound devices - only GPIO 108 and 109 */
static struct msm_gpiomux_config msm8x60_snd_configs[] = {
    { .gpio = 108, ... },  // I2S MCLK
    { .gpio = 109, ... },  // I2S MCLK
};
// NO GPIO 110!
```

**Our mainline incorrectly added GPIO 110 to I2S pinctrl.** The PRIMARY I2S interface
routes audio data internally through LPASS, not via external GPIO. The audio data
path uses the Q6 LPASS internal routing, not external GPIO pins.

### WiFi vs 3G GPIO Differences

From `gpiomux-tenderloin.h`:

| Function | WiFi Variant | 3G Variant |
|----------|--------------|------------|
| BT_POWER | GPIO 130 | GPIO 110 |
| BT_RST_N | GPIO 138 | GPIO 122 |
| 3G modem GPIOs | N/A | GPIOs 38, 61, 171, etc. |

**GPIO 110 is BT_POWER on 3G variants - cannot be used for I2S!**

### Class-D GPIO Mapping (webOS vs Our DT)

**webOS kernel (`board-msm8x60-audio.c`):**
```c
#define SNDDEV_GPIO_CLASS_D0_EN 227  // PM8901 MPP3 (uses pm8901_mpp_config_digital_out(PM8901_MPP_3))
#define SNDDEV_GPIO_CLASS_D1_EN 229  // Separate I2C GPIO expander (NOT PM8901!)
```

**Our mainline device tree:**
```dts
class-d0-gpios = <&pm8058_mpps 11>;  // WRONG PMIC! Should be PM8901
class-d1-gpios = <&pm8901_mpps 1>;   // WRONG! GPIO 229 is not on PM8901
```

**GPIO number calculation:**
- PM8901 MPP base = 225
- GPIO 227 = 225 + 2 = PM8901 MPP3 (index 2 in code, DT uses &pm8901_mpps 2)
- GPIO 229 = Beyond PM8901 range (225-228), must be on separate I2C expander

### All External Class-D GPIOs are LOW During Working Playback

This is the critical finding from the webOS GPIO capture during active playback:

```
gpio-223 (PM8058 MPP11): sink lo
gpio-225 (PM8901 MPP1): d_out lo
gpio-226 (PM8901 MPP2): d_out lo
gpio-227 (PM8901 MPP3): d_out lo
```

**All PM8901/PM8058 MPPs are LOW during working audio!**

This strongly suggests **WM8994 GPIO1 is the sole amplifier enable**, and the
external SoC GPIOs are either:
1. Not needed at all
2. Using active-LOW logic
3. Controlled differently than we expected

## Updated Recommendations

### 1. Remove GPIO 110 from I2S Pinctrl (CRITICAL)

GPIO 110 is not used for audio and conflicts with Bluetooth on 3G variants:

```dts
// Remove from primary_i2s_pins:
// pins = "gpio108", "gpio109", "gpio110";  // WRONG
   pins = "gpio108", "gpio109";              // CORRECT
```

### 2. Remove or Fix External Class-D GPIO Control

Since all external GPIOs are LOW in webOS during playback, remove them:

```dts
// Remove these from device tree:
// class-d0-gpios = <&pm8058_mpps 11 GPIO_ACTIVE_HIGH>;
// class-d1-gpios = <&pm8901_mpps 1 GPIO_ACTIVE_HIGH>;
```

WM8994 GPIO1 (register 0x700 = 0x0041) should be the sole amplifier enable.

### 3. Verify WM8994 GPIO1 Configuration

Both webOS (0x0001) and our mainline (0x0041) configure GPIO1 to output HIGH:
- GP1_FN bits [4:0] = 0x01 = Logic level output HIGH

This should enable the external amplifier correctly.

### 4. Consider FLL Source Change

WebOS uses MCLK as FLL source (register 0x224 = 0x0C83):
- FLL1_CLK_REF_SRC = MCLK1
- FLL1_FRATIO and FLL1_OUTDIV configured for 48kHz output

Our mainline uses LRCLK as FLL source. Consider testing with MCLK source.

## Raw Data Files

- `/tmp/webos_audio_capture.txt` - Initial GPIO/clock capture
- `/tmp/webos_3g_audio_analysis.txt` - Analysis summary
- `/tmp/webos_codec_regs.txt` - Full codec register dump

## Legacy Kernel Source Files Analyzed

- `arch/arm/mach-msm/gpiomux-tenderloin.c` - GPIO pinmux configuration
- `arch/arm/mach-msm/gpiomux-tenderloin.h` - GPIO definitions (WiFi vs 3G)
- `arch/arm/mach-msm/qdsp6v2/board-msm8x60-audio.c` - Audio GPIO/amplifier control
