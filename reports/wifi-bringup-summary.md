# HP TouchPad WiFi (AR6003) Bringup Summary

## Hardware
- Atheros AR6003 Rev2 WiFi chip
- Connected via SDIO to SDCC4 (0x121c0000)
- Uses ADM DMA controller for data transfer

## What We've Tried

### 1. SDIO DMA Configuration
**Problem:** WiFi SDIO showed -110 timeout errors during BMI communication.

**Attempted Solutions:**
- Used shr-linux style 1-cell DMA with `qcom,rx-crci`/`qcom,tx-crci` properties
  - **Result:** FAIL - mainline mmci doesn't parse these properties
- Switched to ADM native 2-cell format: `dmas = <&adm_dma1 5 5>, <&adm_dma1 5 5>`
  - **Result:** DMA channels detected but released immediately

**Root Cause Found:** qcom_dma_setup in mmci_qcom_dml.c requires BOTH tx and rx DMA channels (lines 129-134).

**Fix:** Added both channels:
```dts
dmas = <&adm_dma1 5 5>, <&adm_dma1 5 5>;
dma-names = "rx", "tx";
```

### 2. eMMC DMA Issue
**Problem:** Adding DMA to SDCC1 (eMMC) caused boot hangs.

**Solution:** Removed DMA from SDCC1 entirely - PIO mode works reliably for eMMC.

### 3. Power Sequencing (pwrseq)
**Problem:** WiFi showed "bmi communication timeout" (-110) errors.

**Attempted Solutions:**
- 3-GPIO config (93, 137, 135) - FAIL, still timed out
- Added `post-power-on-delay-ms = <500>` - FAIL
- Added 32kHz sleep clock - FAIL

**Breakthrough:** Simplified to single GPIO 135 (reset line) with 200ms delay:
```dts
ath6kl_pwrseq: ath6kl-pwrseq {
    compatible = "mmc-pwrseq-simple";
    reset-gpios = <&tlmm 135 GPIO_ACTIVE_LOW>;
    clocks = <&sleep_clk>;
    clock-names = "ext_clock";
    post-power-on-delay-ms = <200>;
};
```
**Result:** BMI timeout FIXED! Error changed from -110 to -2 (ENOENT).

### 4. Firmware Format
**Problem:** After fixing BMI timeout, got "Magic is invalid, magic_len: 11" errors.

**Root Cause:** webOS firmware uses proprietary "SGMT" (Segmented) format. Mainline ath6kl expects "QCA-ATH6KL" IE (Information Element) format.

**Solution:** Use linux-firmware files instead of webOS firmware:
- fw-2.bin, fw-3.bin from linux-firmware (have proper QCA-ATH6KL header)
- bdata.SD32.bin from webOS (board-specific calibration data)

**Result:** "Magic is invalid" errors FIXED!

### 5. Built-in vs Module
**Problem:** ath6kl probed before firmware was available on boot partition.

**Solution:** Changed from CONFIG_ATH6KL=y to CONFIG_ATH6KL=m, added module loading to init script after firmware mount.

### 6. Current Status (Build #129)
- ath6kl_core and ath6kl_sdio load as modules
- Firmware format accepted (no magic errors)
- "temporary war to avoid sdio crc error" triggers
- DMA transaction submitted
- **STUCK:** Driver hangs after first DMA transaction
- insmod never completes, module stays in "Loading" state

## Current DT Configuration

```dts
&sdcc4 {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&sdcc4_pins>, <&wlan_gpios>;
    vmmc-supply = <&pm8901_l1>;
    vqmmc-supply = <&pm8058_s3>;
    mmc-pwrseq = <&ath6kl_pwrseq>;
    dmas = <&adm_dma1 5 5>, <&adm_dma1 5 5>;
    dma-names = "rx", "tx";
    wifi@1 {
        compatible = "atheros,ath6kl";
        reg = <1>;
        atheros,board-id = "SD32";
    };
};
```

## Remaining Issues

### Build #130 - PIO Mode Test
Disabled DMA to test PIO mode. Results:
- **With DMA**: Hangs after first DMA transaction (no completion interrupt)
- **With PIO**: Gets -110 timeout during "write extended board data"

Error sequence with PIO:
```
ath6kl: temporary war to avoid sdio crc error
ath6kl: Unable to decrement the command credit count register: -110
ath6kl: Unable to write to the device: -110
ath6kl: Failed to write extended board data: -110
ath6kl: Failed to start hardware: -110
```

### Root Cause Analysis
The issue is not DMA-specific - it's at the SDIO command level. After the CRC workaround triggers, subsequent SDIO commands timeout. Possible causes:

1. **Power sequencing timing**: May need longer delay after reset
2. **Clock frequency**: High-speed SDIO may be too fast
3. **Missing regulator**: WiFi PA or other supply not enabled
4. **Firmware compatibility**: Board data format mismatch

### SDIO Card Info
- Vendor ID: 0271
- Device ID: 0301
- Mode: High Speed SDIO
- Host: PL180 (mmci-pl18x) at 0x121c0000

## Next Steps to Try

1. Increase post-power-on delay (500ms, 1000ms)
2. Force lower SDIO clock speed
3. Check all WiFi-related regulators are enabled
4. Compare webOS kernel power sequence with mainline
5. Add debug prints to ath6kl BMI code
6. Check if board data (bdata.SD32.bin) format is correct

## Files Modified

- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - DT configuration
- `arch/arm/configs/tenderloin_defconfig` - ATH6KL=m
- `scripts/initramfs/init` - WiFi module loading
- Firmware: linux-firmware fw-2.bin/fw-3.bin + webOS bdata.SD32.bin
