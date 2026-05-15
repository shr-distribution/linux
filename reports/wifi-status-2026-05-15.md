# WiFi Status Report - 2026-05-15

**Kernel:** `6.18.0-luneos-ge65c651a4001`  
**Test Date:** 2026-05-15 12:07 UTC  
**Device:** HP TouchPad (Topaz WiFi)

## Current Status: ❌ BROKEN

WiFi initialization **completely fails** with SDIO timeout errors during firmware upload.

## Test Results

### Hardware Detection: ✅ PASS
```
SDIO device detected: mmc1:0001:1
SDIO_ID: 0271:0301 (Atheros AR6003)
OF_COMPATIBLE: atheros,ath6kl
```

### Driver Loading: ✅ PASS (Built-in)
```
CONFIG_ATH6KL=y
CONFIG_ATH6KL_SDIO=y
Driver present: /sys/bus/sdio/drivers/ath6kl_sdio
```

### Firmware Files: ✅ PASS
```
/lib/firmware/ath6k/AR6003/hw2.1.1/:
- fw-2.bin (89844 bytes)
- fw-3.bin (104562 bytes)
- fw-4.bin -> fw-3.bin (symlink)
- fw-5.bin -> fw-3.bin (symlink)
- bdata.bin -> bdata.SD32.bin (symlink) ← FIXED
- bdata.SD32.bin (1792 bytes)
- otp.bin (2783 bytes)
```

### Driver Probe: ❌ FAIL - CMDTIMEOUT

**Error log:**
```
[  304.617927] ath6kl: temporary war to avoid sdio crc error
[  304.637002] mmci-pl18x 121c0000.mmc: CMDTIMEOUT: cmd53 arg=0x1408a004 status=0x00000004 data=yes
[  304.637127] ath6kl: Unable to decrement the command credit count register: -110
[  304.662561] ath6kl: Unable to write to the device: -110
[  304.662662] ath6kl: Failed to upload OTP file: -110
[  304.667496] ath6kl: Failed to start hardware: -110
[  304.683394] ath6kl: Failed to init ath6kl core
[  304.683511] ath6kl_sdio mmc1:0001:1: probe with driver ath6kl_sdio failed with error -110
```

**Multiple retry attempts all fail with same error (-110 = -ETIMEDOUT)**

## Root Cause Analysis

### Problem: SDIO CMD53 Timeout

The driver fails to communicate with the WiFi chip during the OTP (One-Time Programmable memory) firmware upload phase. This is a **CMD53 (SDIO I/O read/write)** timeout.

**Error details:**
- Command: `cmd53 arg=0x1408a004` (SDIO block write)
- Status: `0x00000004` (CMDTIMEOUT bit set)
- Error code: -110 (-ETIMEDOUT)
- Phase: OTP file upload (early firmware init)

### Message: "temporary war to avoid sdio crc error"

This debug message appears in the ath6kl driver code as a **workaround** for SDIO CRC errors, but the workaround itself is timing out.

## Comparison with Previous Reports

### wifi-bringup-summary.md (2026-01-29)

**Claimed status:** "WiFi scanning WORKING"
- ✅ Firmware upload succeeds (~34s via PIO)
- ✅ WMI initialization succeeds
- ✅ Scan finds 10+ APs on 2.4GHz + 5GHz

**Current status:** WiFi initialization **completely fails** at OTP upload

### Conclusion: **REGRESSION**

Somewhere between 2026-01-29 and 2026-05-15, WiFi broke completely.

## Recent Commits That May Have Caused Regression

From git log since 2026-01-29:

1. **`3d8b14013619`** (recent) - ARM: dts: tenderloin: force HPM on WiFi-related PMIC regulators
2. **`2263a6546598`** - mmc: mmci: disable MCI_CLK_PWRSAVE on SDIO buses (qcom variant)
3. **`6884e69a61d2`** - ARM: dts: tenderloin: declare explicit bus-width = <4> on sdcc4 (WiFi)

These commits were attempting to fix WiFi stability but may have introduced new issues.

## Technical Details

### SDIO Configuration (from DT)

```dts
&sdcc4 {
    status = "okay";
    bus-width = <4>;
    max-frequency = <24000000>;
    qcom,datactrl-first;
    cap-sdio-irq;
    keep-power-in-suspend;
    vmmc-supply = <&pm8901_l1>;   /* 3.3V VDD_WLAN_3V3 */
    vqmmc-supply = <&pm8058_s3>;  /* 1.8V DVDD_SDIO_1V8 */
    dmas = <&adm_dma1 5>, <&adm_dma1 5>;
    dma-names = "rx", "tx";
    qcom,sdcc-crci = <5>;
};
```

### Possible Causes

1. **PMIC regulator issue** - HPM mode change may have affected power delivery
2. **Clock power save issue** - Disabling MCI_CLK_PWRSAVE may have timing side effects
3. **DMA issue** - Though timeout is in PIO upload phase, DMA config may affect init
4. **Power sequencing** - ath6kl_pwrseq timing may be wrong

## Recommended Next Steps

### 1. Revert Recent WiFi Commits (Bisection)

Start with the most recent WiFi-related commits and bisect:

```bash
git revert 3d8b14013619  # HPM PMIC change
# Test
git revert 2263a6546598  # PWRSAVE disable
# Test
git revert 6884e69a61d2  # bus-width explicit
# Test
```

### 2. Check PMIC Regulator Status

The HPM (High Power Mode) change for WiFi regulators may be causing issues:

```bash
# On device, check regulator status
cat /sys/kernel/debug/regulator/*/state
cat /sys/kernel/debug/regulator/8901_l1/state  # VDD_WLAN_3V3
cat /sys/kernel/debug/regulator/8058_s3/state  # DVDD_SDIO_1V8
```

### 3. Check for SDIO Clock Issues

The PWRSAVE disable may have introduced clock timing problems:

```bash
# Check SDIO clock status in dmesg
dmesg | grep -E "mmc1.*clock|PWRSAVE"
```

### 4. Compare with Working Kernel

If you have a known-working kernel build from January 2026:
- Boot it and confirm WiFi works
- Compare dmesg SDIO init sequence
- Compare regulator states

### 5. Enable Debug Logging

```bash
# In kernel config
CONFIG_ATH6KL_DEBUG=y
CONFIG_MMC_DEBUG=y
CONFIG_DYNAMIC_DEBUG=y

# At runtime
echo "module ath6kl_core +p" > /sys/kernel/debug/dynamic_debug/control
echo "module ath6kl_sdio +p" > /sys/kernel/debug/dynamic_debug/control
echo "module mmci +p" > /sys/kernel/debug/dynamic_debug/control
```

## Related Issues

### Issue #1: Firmware loads in PIO mode (34s)
- **Status:** Still relevant (if we can get WiFi working at all)
- **Current:** Can't even test this - OTP upload fails before firmware load

### Issue #2: Scan doesn't find networks
- **Status:** SUPERSEDED - WiFi doesn't initialize at all
- **Root cause:** SDIO CMD53 timeout during OTP upload

## Files for Reference

- Current dmesg saved: `/tmp/device-wifi-fail-dmesg.txt`
- Working kernel report: `reports/wifi-bringup-summary.md` (2026-01-29)
- Token analysis: `reports/wifi-calibration-token-analysis.md`

---

**Bottom line:** WiFi is completely broken on current kernel. This is a regression from the January 2026 working state. Priority should be reverting recent WiFi commits to find which one broke it.
