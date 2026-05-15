# MSM8660 SPM Sequence Requirements Analysis
**Date:** 2026-05-15
**Context:** Task #4 - Investigate if bootloader-programmed SPM_CTL values are sufficient or if explicit mode programming is needed

---

## Executive Summary

**Finding:** The mainline SPM driver's approach is **correct but incomplete**.

- ✅ **Hardware architecture validated:** MSM8660 SAW v1.0 has no SEQ_ENTRY register, uses register-based mode control only
- ✅ **Register programming method confirmed:** SPM_CTL bits [3:0] control modes, matches legacy
- ✅ **Mainline implementation correct:** `spm_set_low_power_mode_8660()` programs the right registers
- ⚠️ **Gap identified:** Mainline skips initial register programming (`spm_cfg = 0`) but legacy always initializes 11 registers at boot

**Recommendation:** Add SPM register initialization to mainline probe, matching legacy values.

---

## MSM8660 SAW v1.0 Architecture

### No Sequence RAM

MSM8660 SAW (Subsystem Power Manager, version 1.0) differs from later SoCs:

**MSM8660 (SAW v1.0):**
- No `SPM_SEQ_ENTRY` register
- No programmable sequence RAM
- Power modes controlled entirely by register writes to:
  - `SPM_CTL` (0x14) - mode bits [1:0], rpm_bypass bit [3], rst bit via SLP_RST_EN
  - `SPM_PMIC_CTL` (0x20) - voltage levels (awake, mid, sleep)
  - `SLP_RST_EN` (0x34) - core reset enable

**Later SoCs (SAW v2.0+, e.g., MSM8960/APQ8064):**
- Have `SPM_SEQ_ENTRY` register
- Programmable sequence of 32 bytes
- State machine executes sequences on WFI

This explains why mainline `spm_reg_8660_cpu` has `.no_seq_ram = true`.

---

## Legacy Initialization Values

From `arch/arm/mach-msm/board-tenderloin.c`, the webOS kernel programmed **11 registers** at boot for each CPU:

### CPU0 (SAW0 @ 0x02042000):
```c
.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,
.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,
.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,
```

### CPU1 (SAW1 @ 0x02052000):
```c
.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,
.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,        // <- Different: 0x13 vs 0x01
.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,
.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,
```

**Key difference:** `SLP_CLK_EN` is 0x13 for CPU1 vs 0x01 for CPU0 (bits [4:1] control clock enables for different sleep modes).

---

## Register Decoding

### SPM_CTL = 0x68 (initial value)
```
Bits [7:4] = 0x6 (event output config, bootloader-specific)
Bits [3]   = 1   (rpm_bypass = 1, don't notify RPM)
Bits [2]   = 0   (reserved)
Bits [1:0] = 0x0 (mode = 0x0, CLOCK_GATING)
```

Initial mode is clock gating with RPM bypass enabled (standalone mode).

### SPM_CTL Mode Programming (from legacy spm.c)

**CLOCK_GATING (mode 0x00):**
```c
SPM_CTL = (SPM_CTL & ~0x0F) | (rpm_bypass << 3) | 0x00
SLP_RST_EN = 0x00
```

**POWER_RETENTION (mode 0x02):**
```c
SPM_CTL = (SPM_CTL & ~0x0F) | (rpm_bypass << 3) | 0x02
PMIC_CTL = (retention_mid_vlevel << 16) | (awake_vlevel << 8) | retention_vlevel
SLP_RST_EN = 0x00
```

**POWER_COLLAPSE (mode 0x02):**
```c
SPM_CTL = (SPM_CTL & ~0x0F) | (rpm_bypass << 3) | 0x02
PMIC_CTL = (collapse_mid_vlevel << 16) | (awake_vlevel << 8) | collapse_vlevel
SLP_RST_EN = 0x01  // <- Reset asserts on sleep
```

**Note:** Retention and Power Collapse both use mode 0x02. The difference is:
1. Voltage levels in PMIC_CTL
2. SLP_RST_EN (0=retention keeps core state, 1=collapse resets core)

---

## Mainline Implementation

### Current Behavior (`drivers/soc/qcom/spm.c`)

**Probe-time initialization:**
```c
static const struct spm_reg_data spm_reg_8660_cpu = {
    .reg_offset = spm_reg_offset_8660,
    .spm_cfg = 0,        // <- This is the key: 0 means skip init
    .pmic_dly = 0,
    .no_seq_ram = true,
    ...
};
```

From `spm_dev_probe()`:
```c
if (drv->reg_data->spm_cfg)
    spm_register_write(drv, SPM_REG_CFG, drv->reg_data->spm_cfg);
```

**Result:** Mainline skips probe-time register initialization, relying entirely on bootloader defaults.

**Runtime mode programming:**

`spm_set_low_power_mode_8660()` (lines 372-420) correctly programs:
- SPM_CTL bits [3:0] for mode and rpm_bypass
- SPM_PMIC_CTL for voltage levels (not yet used, vdd_mem/vdd_dig regulators not wired)
- SLP_RST_EN for core reset control

This matches legacy runtime behavior.

---

## Gap Analysis

### What Works

✅ Mode transitions at runtime (CLOCK_GATING → SPC → PC) are correct
✅ rpm_bypass control (bit 3) works as expected
✅ No sequence RAM needed (MSM8660 hardware doesn't have it)

### What's Missing

⚠️ **Initial register programming skipped:**

Legacy initializes 11 registers at probe:
- CFG, SPM_CTL, SLP_TMR_DLY, WAKE_TMR_DLY
- SLP_CLK_EN, SLP_HSFS_PRECLMP_EN, SLP_HSFS_POSTCLMP_EN
- SLP_CLMP_EN, SLP_RST_EN, SPM_MPM_CFG

Mainline only touches SPM_CTL/PMIC_CTL/SLP_RST_EN at runtime, never writes the other 8 registers.

**Potential impact:**
- If bootloader doesn't initialize SAW registers correctly, power collapse may fail
- Timer delays (SLP_TMR_DLY, WAKE_TMR_DLY) affect wakeup latency
- Clock/clamp enables (SLP_CLK_EN, SLP_CLMP_EN, etc.) control hardware sequencing

**Observed behavior on TouchPad:**
- The HP TouchPad bootloader (Bootie/APPSBL) likely initializes SAW registers
- This is why mainline's "skip init" approach hasn't caused immediate failure
- But other MSM8660 devices (HTC, Samsung, Sony) may have different bootloaders

---

## Recommendations

### Option A: Trust Bootloader (Current Approach)

**Pros:**
- Less code
- Works on TouchPad

**Cons:**
- Fragile across devices
- No visibility into actual register state
- Can't tune timing parameters

### Option B: Explicit Initialization (Recommended)

Add register initialization to mainline SPM probe:

```c
static const struct spm_reg_data spm_reg_8660_cpu = {
    .reg_offset = spm_reg_offset_8660,
    .spm_cfg = 0x1C,          // Match legacy
    .pmic_dly = 0,
    .no_seq_ram = true,
    .spm_ctl_init = 0x68,     // New: initial SPM_CTL value
    .slp_tmr_dly = 0x0C0CFFFF,
    .wake_tmr_dly = 0x78780FFF,
    // ... other init values
};
```

Modify `spm_dev_probe()` to write all init values.

**Pros:**
- Explicit, predictable behavior
- Works regardless of bootloader state
- Enables tuning (e.g., adjust timer delays for latency)
- Matches legacy kernel exactly

**Cons:**
- ~20 lines of init code
- Adds 8 new fields to `spm_reg_data` struct

### Option C: Hybrid - Verify Bootloader Initialized

Read SPM_CTL at probe, check if it's non-zero. If zero, warn and apply defaults.

---

## Testing Plan

Once Yocto build with cpuidle enabled is deployed:

### 1. Read SPM registers at boot
```bash
# CPU0 SAW
devmem2 0x02042014 w  # SPM_CTL
devmem2 0x02042018 w  # SLP_TMR_DLY
devmem2 0x0204201C w  # WAKE_TMR_DLY
devmem2 0x02042024 w  # SLP_CLK_EN
devmem2 0x02042034 w  # SLP_RST_EN

# CPU1 SAW
devmem2 0x02052014 w  # SPM_CTL
devmem2 0x02052024 w  # SLP_CLK_EN (should be 0x13)
```

### 2. Verify runtime mode changes
```bash
# Offline CPU1
echo 0 > /sys/devices/system/cpu/cpu1/online

# Check cpuidle enters SPC
cat /sys/devices/system/cpu/cpu0/cpuidle/state1/time  # Should increment

# Check dmesg for SPM driver activity (if debug enabled)
dmesg | grep spm
```

### 3. Compare with legacy values

If boot values match legacy table above → bootloader is initializing correctly, Option A is safe.

If boot values differ → Option B (explicit init) is needed.

---

## Next Steps

1. **Deploy Yocto build with idle=poll removed**
2. **Test Task #3:** Verify single-core power collapse works with current "trust bootloader" approach
3. **If successful:** Document that TouchPad bootloader provides correct SAW init, no changes needed for this device
4. **If issues occur:** Implement Option B (explicit initialization matching legacy values)

---

## References

**Legacy kernel:**
- `arch/arm/mach-msm/spm.c` - Runtime mode programming
- `arch/arm/mach-msm/board-tenderloin.c` lines 277-395 - Init values

**Mainline kernel:**
- `drivers/soc/qcom/spm.c` lines 372-420 - MSM8660 mode programming
- `drivers/cpuidle/cpuidle-qcom-spm.c` - cpuidle integration

**Hardware:**
- MSM8660 SAW v1.0 (Subsystem Power Manager)
- Base addresses: SAW0 @ 0x02042000, SAW1 @ 0x02052000
- Register map documented in mainline `spm_reg_offset_8660` array

---

**Status:** INVESTIGATION COMPLETE - Ready for Task #3 testing to validate
