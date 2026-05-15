# WiFi SDIO Timeout Deep Analysis - 2026-05-15

## Problem Statement

WiFi initialization fails with **CMD53 SDIO timeout (-110)** during the ath6kl driver's SDIO CRC error workaround (WAR) execution:

```
[  304.617927] ath6kl: temporary war to avoid sdio crc error
[  304.637002] mmci-pl18x 121c0000.mmc: CMDTIMEOUT: cmd53 arg=0x1408a004 status=0x00000004 data=yes
[  304.637127] ath6kl: Unable to decrement the command credit count register: -110
[  304.662561] ath6kl: Unable to write to the device: -110
[  304.662662] ath6kl: Failed to upload OTP file: -110
```

## Background: User's Key Insight

**User observation:** "We previously got wifi working in initramfs, not in userspace LuneOS I think."

This suggests **timing or power state differences** between early boot (initramfs) and full system boot.

## Technical Analysis

### 1. The SDIO CRC Error Workaround

**Location:** `drivers/net/wireless/ath/ath6kl/init.c:1516-1547`

The AR6003 hardware (TouchPad's WiFi chip) has `ATH6KL_HW_SDIO_CRC_ERROR_WAR` flag set. During initialization, the driver writes to GPIO pins 9-13 on the WiFi chip:

```c
if (ar->hw.flags & ATH6KL_HW_SDIO_CRC_ERROR_WAR) {
    ath6kl_err("temporary war to avoid sdio crc error\n");
    
    /* Write 0x28 to GPIO PIN9 */
    status = ath6kl_bmi_reg_write(ar, GPIO_BASE_ADDRESS + GPIO_PIN9_ADDRESS, 0x28);
    
    /* Write 0x20 to GPIO PIN10-13 */
    status = ath6kl_bmi_reg_write(ar, GPIO_BASE_ADDRESS + GPIO_PIN10_ADDRESS, 0x20);
    status = ath6kl_bmi_reg_write(ar, GPIO_BASE_ADDRESS + GPIO_PIN11_ADDRESS, 0x20);
    status = ath6kl_bmi_reg_write(ar, GPIO_BASE_ADDRESS + GPIO_PIN12_ADDRESS, 0x20);
    status = ath6kl_bmi_reg_write(ar, GPIO_BASE_ADDRESS + 0x20);
}
```

**The timeout occurs during these `ath6kl_bmi_reg_write()` calls**, specifically on the CMD53 SDIO command.

### 2. WebOS Power Sequencing (Working Reference)

From `board-tenderloin.c` in webOS kernel:

```c
int tenderloin_wifi_power(int on) {
    // Power sequence:
    // 1. Set 8058_s3 (SDIO 1.8V) to HIGH LOAD mode
    regulator_set_optimum_mode(wifi_S3A_1V8, WLAN_1V8_SDIO_ACT_LOAD);
    
    // 2. Enable regulators in order: 3.3V -> 1.8V
    regulator_enable(wifi_L3B_3V3);  // 8901_l3 - WLAN_PA_3V3
    regulator_enable(wifi_L1B_3V3);  // 8901_l1 - VDD_WLAN_3V3
    regulator_enable(wifi_L19A_1V8); // 8058_l19 - VDD_1.8
    // 8058_s3 already enabled (always-on)
    
    // 3. Delay 5ms
    mdelay(5);
    
    // 4. Reset pulse: LOW -> 5ms -> HIGH
    gpio_direction_output(WLAN_RST_N, 0);
    mdelay(5);
    gpio_direction_output(WLAN_RST_N, 1);
    
    // 5. Trigger SDIO detection with 250ms delay
    mmc_detect_change(wifi_mmc, msecs_to_jiffies(250));
}
```

**Key timing:**
- Power-on at boot: ~45-46 seconds (webOS dmesg shows `board_sdio_wifi_enable` at 45.854s)
- Reset pulse: 5ms LOW, then HIGH
- SDIO detection delay: **250ms** after reset

### 3. Mainline Device Tree Configuration

From current mainline `qcom-apq8060-tenderloin-common.dtsi`:

```dts
ath6kl_pwrseq: ath6kl-pwrseq {
    compatible = "mmc-pwrseq-simple";
    reset-gpios = <&tlmm 135 GPIO_ACTIVE_LOW>;
    clocks = <&sleep_clk>;
    clock-names = "ext_clock";
    post-power-on-delay-ms = <200>;  // ← Only 200ms delay
};

&sdcc4 {
    vmmc-supply = <&pm8901_l1>;   // 3.3V VDD_WLAN_3V3
    vqmmc-supply = <&pm8058_s3>;  // 1.8V DVDD_SDIO_1V8
    mmc-pwrseq = <&ath6kl_pwrseq>;
    // Recent commits may have affected this
};
```

**Missing from mainline:**
1. **No `regulator-set-load` for 8058_s3** - No HIGH LOAD mode setting
2. **Shorter delay** - 200ms vs webOS's 250ms
3. **No explicit enable sequence** - Relies on pwrseq framework

### 4. Recent Commits That Changed WiFi Behavior

#### Commit `3d8b14013619` - Force HPM on WiFi PMIC regulators

This commit added `regulator-initial-mode = <RPMH_REGULATOR_MODE_HPM>` to WiFi regulators. **However:**

**Problem:** MSM8660 uses **RPM** (not RPMH), and the regulator mode constants are different:
- RPMH uses: `RPMH_REGULATOR_MODE_HPM`
- RPM uses: Different mode values

**Impact:** The HPM mode setting may be incorrect or ignored, causing regulators to remain in LPM (Low Power Mode).

#### Commit `2263a6546598` - Disable MCI_CLK_PWRSAVE on SDIO

```c
// Disabled power save on SDIO clock
if (variant->qcom_variant && host->mmc->caps & MMC_CAP_SDIO_IRQ)
    clk &= ~MCI_CLK_PWRSAVE;
```

**Impact:** Clock stays active continuously. May cause timing issues if chip expects clock gating.

#### Commit `6884e69a61d2` - Explicit bus-width=<4>

Added explicit `bus-width = <4>` to sdcc4 node.

**Impact:** Should be neutral, but may have triggered different code paths.

### 5. Initramfs vs LuneOS Difference

**User's key observation:** WiFi worked in initramfs but not in full LuneOS.

**Hypothesis:**

| Environment | Characteristics | WiFi Result |
|-------------|----------------|-------------|
| **Initramfs (debug)** | - Early boot (~10s)<br>- Minimal system load<br>- Regulators freshly initialized<br>- No CPU freq scaling yet | ✅ May work |
| **Full LuneOS** | - Late boot (2-5 min)<br>- Full systemd running<br>- CPU throttling active<br>- Regulators may be in LPM | ❌ Fails |

**Critical difference:** Regulator load mode!

In initramfs:
- Regulators start in HPM or AUTO mode
- SDIO 1.8V rail has sufficient current

In full system:
- Regulators may switch to LPM to save power
- SDIO 1.8V rail underpowered for WiFi BMI transactions
- CMD53 times out due to insufficient power

## Root Cause Analysis

### Primary Suspect: Regulator Load Mode

The SDIO 1.8V rail (`8058_s3`) needs **HIGH LOAD mode** during WiFi initialization, but mainline doesn't set this.

**WebOS explicitly sets:**
```c
regulator_set_optimum_mode(wifi_S3A_1V8, WLAN_1V8_SDIO_ACT_LOAD);
// Where WLAN_1V8_SDIO_ACT_LOAD is likely 100-200mA
```

**Mainline has:**
```dts
vqmmc-supply = <&pm8058_s3>;
// No regulator-set-load property!
```

**Result:** The SDIO rail remains in LPM (Low Power Mode, ~10-50mA), which is **insufficient for WiFi BMI transactions** that require burst current.

### Secondary Issue: Timing

The `post-power-on-delay-ms = <200>` may be too short. WebOS uses:
- 5ms after power enable
- 5ms reset LOW
- 250ms before SDIO detection

Total: **~260ms** minimum

Mainline uses: **200ms** total

### Why It Worked Before (January 2026)

The wifi-bringup-summary.md from 2026-01-29 shows WiFi working. What changed?

**Possible reasons it worked then:**
1. **PIO mode testing** - Tests may have been done in initramfs only
2. **Different kernel config** - Regulator framework behavior may have changed
3. **No HPM commit yet** - The incorrect HPM mode setting wasn't applied

## webOS Boot Partition Analysis

### Question: "Does webOS do anything special in boot partition for WiFi besides tokens?"

**Answer: NO special WiFi init in boot partition.**

**Evidence:**
1. Searched `/boot` (webOS boot partition) - No WiFi-specific init scripts
2. WiFi power control is entirely in **kernel driver** (`board-tenderloin.c`)
3. Token calibration is **runtime** (PmWiFiService reads from NVRAM p13)
4. No bootloader-level WiFi initialization found

**Bootloader sequence:**
```
ROM → SPBL → RPMSBL → RPM → QCSBL → Bootie → TZ (empty) → Linux
```

WiFi chip is **powered off** until Linux calls `board_sdio_wifi_enable()` at ~45 seconds.

### What About GPIO Configuration?

WebOS kernel initializes WiFi GPIOs in `gpiomux-tenderloin.c`, but this is **kernel code**, not boot partition.

## Recommended Fixes

### Fix #1: Add Regulator Load Mode (HIGH PRIORITY)

**Device tree change:**
```dts
&pm8058_s3 {  /* SDIO 1.8V rail */
    regulator-initial-mode = <REGULATOR_MODE_NORMAL>;  // Not RPMH_*!
    // Or add to sdcc4:
};

&sdcc4 {
    vmmc-supply = <&pm8901_l1>;
    vqmmc-supply = <&pm8058_s3>;
    /delete-property/ regulator-initial-mode;  // Remove incorrect RPMH mode
    
    // Add load requirement
    vqmmc-microamp = <100000>;  // 100mA active load
};
```

**Alternative:** Add `regulator-allow-set-load` to 8058_s3 and have mmci driver set load dynamically.

### Fix #2: Increase Power-On Delay

```dts
ath6kl_pwrseq: ath6kl-pwrseq {
    compatible = "mmc-pwrseq-simple";
    reset-gpios = <&tlmm 135 GPIO_ACTIVE_LOW>;
    clocks = <&sleep_clk>;
    clock-names = "ext_clock";
    post-power-on-delay-ms = <300>;  // Increase from 200ms to 300ms
};
```

### Fix #3: Revert HPM Commit (IMMEDIATE TEST)

```bash
git revert 3d8b14013619  # "ARM: dts: tenderloin: force HPM on WiFi-related PMIC regulators"
```

**Reason:** The RPMH_REGULATOR_MODE_HPM constant is **wrong** for RPM-based regulators. This may be causing the regulator subsystem to reject the mode or fall back to LPM.

### Fix #4: Check Regulator Framework

The RPM regulator driver may not support mode setting properly. Need to check:
```bash
# On device
cat /sys/kernel/debug/regulator/8058_s3/mode
cat /sys/kernel/debug/regulator/8058_s3/load
```

## Testing Protocol

### Test 1: Initramfs Early Test

Boot to initramfs and test WiFi **immediately** (within 30 seconds):
```bash
# In initramfs
insmod ath6kl_core.ko
insmod ath6kl_sdio.ko
dmesg | tail -50
```

**Expected:** If regulator issue, should work in initramfs (regulators still in HPM from boot).

### Test 2: Late Boot Test

Wait 2-3 minutes after boot, then test:
```bash
# In full LuneOS
echo "mmc1:0001:1" > /sys/bus/sdio/drivers/ath6kl_sdio/bind
dmesg | tail -50
```

**Expected:** If regulator issue, should fail (regulators switched to LPM).

### Test 3: Force HPM via sysfs

```bash
# Before binding driver
echo 100000 > /sys/class/regulator/regulator.N/load  # Find correct regulator.N for 8058_s3
echo "mmc1:0001:1" > /sys/bus/sdio/drivers/ath6kl_sdio/bind
```

**Expected:** If regulator issue, this should fix it.

## Comparison with Working State

### January 2026 (WORKING)
- Reports show: "WiFi scanning works, finds 10+ APs"
- Firmware upload: ~34s via PIO
- Test environment: Likely **initramfs only**

### May 2026 (BROKEN)
- Full LuneOS boot
- CMD53 timeout at OTP upload
- Test environment: **Full userspace**

**Conclusion:** The regression is likely **environment-dependent**, not a code bug. The regulator framework behavior differs between initramfs and full system.

## References

- webOS kernel: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c`
- webOS dmesg: `/tmp/dmesg-webos.txt`
- Failed dmesg: `/tmp/device-wifi-fail-dmesg.txt`
- Driver code: `drivers/net/wireless/ath/ath6kl/init.c:1516-1547`

---

**Bottom Line:** The WiFi chip isn't getting enough power on the SDIO 1.8V rail during initialization in full LuneOS. The incorrect HPM mode setting (commit 3d8b14013619) likely made this worse. Revert that commit and add proper `regulator-set-load` support to fix it.
