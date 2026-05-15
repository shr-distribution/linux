# SPM Initialization Investigation Results
**Date:** 2026-05-15
**Task:** Check Option A (legacy residency) and Option B (SPM init) against bootloader dumps

---

## Summary

**Critical Finding:** Bootloaders (HTC TrustZone, HTC SBL3, TouchPad APPSBL) do NOT initialize SPM/SAW registers. The mainline driver incorrectly assumes bootloader initialization.

**Evidence:**
1. No SAW base address references (0x02042000, 0x02052000) in any bootloader
2. Device SPM_CTL reads as 0x08 (bootloader default) vs legacy expected 0x68
3. Legacy kernel explicitly wrote 11 SPM registers at boot (board-tenderloin.c lines 277-395)
4. Legacy cpuidle used dynamic residency (target_residency=0), not static DT values

**Conclusion:** Must implement SPM register initialization in mainline probe.

---

## Option A: Legacy Residency Analysis

### Legacy Approach

Legacy kernel did NOT use fixed target_residency values in cpuidle states. From `pm-8x60.c` lines 859-861:

```c
state->target_residency = 0;
state->exit_latency = 0;
```

Instead, legacy used **dynamic power level selection** via `msm_rpmrs_lowest_limits()` which:
1. Checked PM_QOS latency requirements
2. Checked predicted sleep time from `tick_nohz_get_sleep_length()`
3. Selected deepest RPM resource state (L2/VDD/PXO) that met constraints
4. Enabled/disabled cpuidle states dynamically each idle entry

### Mainline Approach

Mainline uses static DT-defined values:
```dts
cpu_spc: cpu-spc {
    compatible = "qcom,idle-state-spc";
    entry-latency-us = <400>;
    exit-latency-us = <900>;
    min-residency-us = <3000>;  // <- Fixed value
};
```

Menu governor predicts idle time and selects state if prediction > 3000us.

### Why cpu-spc Never Enters

Current symptoms:
- 10ms timer tick (HZ=100)
- Menu governor predictions typically < 3ms
- Lots of IRQ activity (gp_timer firing every 10ms, I2C, DMA, etc.)
- Residency requirement (3000us) too high for system with active peripherals

**Options to fix:**
1. Lower min-residency to 500us (may enter too aggressively, burn power on failed predictions)
2. Switch to tickless/nohz_full (requires more kernel config changes)
3. Accept that cpu-spc won't enter until system is truly idle (after RPM sleep orchestrator is implemented)

**Recommendation:** Keep 3000us for now. The real fix is the RPM sleep orchestrator (Task #7) which will enable deeper states when system is actually idle.

---

## Option B: SPM Initialization Investigation

### Bootloader Analysis

Searched for SAW base addresses (0x02042000, 0x02052000) in:
- HTC tz.img (TrustZone, 107 KB) - **0 references**
- HTC sbl3.img (SBL3 bootloader, 596 KB) - **0 references**
- TouchPad p7 (APPSBL/Bootie, 2.5 MB) - **0 references**
- TouchPad tz-touchpad.mbn (OEMSBL/TZ, 500 KB) - **Empty, 0 references**

**Conclusion:** None of the bootloaders write to SAW/SPM registers.

### Current Hardware State

Reading SPM registers from running device (commit gf807ad7fb10c):

```
CPU0 SAW (0x02042000):
  SPM_CTL (0x14):    0x00000008  // Should be 0x68
  SLP_CLK_EN (0x24): 0x00000000  // Should be 0x01
  SLP_RST_EN (0x34): 0x00000001  // Should be 0x00

CPU1 SAW (0x02052000):
  SPM_CTL (0x14):    0x00000008  // Should be 0x68
  SLP_CLK_EN (0x24): 0x00000000  // Should be 0x13
  SLP_RST_EN (0x34): 0x00000000  // Correct
```

**Analysis of SPM_CTL = 0x08:**
```
Bits [7:4] = 0x0 (no event output config)
Bits [3]   = 1   (rpm_bypass = 1, standalone mode)
Bits [2]   = 0   (reserved)
Bits [1:0] = 0x0 (mode = CLOCK_GATING)
```

This is a minimal/default value, NOT the configured value from legacy.

### Legacy Initialization Values

From `board-tenderloin.c` msm_spm_data[] (lines 335-395):

#### CPU0:
| Register | Offset | Legacy Value | Current | Notes |
|----------|--------|--------------|---------|-------|
| SAW_CFG | 0x10 | 0x1C | ? | Config register |
| SAW_SPM_CTL | 0x14 | 0x68 | 0x08 | **MISMATCH** |
| SAW_SPM_SLP_TMR_DLY | 0x18 | 0x0C0CFFFF | ? | Sleep timer delay |
| SAW_SPM_WAKE_TMR_DLY | 0x1C | 0x78780FFF | ? | Wake timer delay |
| SAW_SLP_CLK_EN | 0x24 | 0x01 | 0x00 | **MISMATCH** |
| SAW_SLP_HSFS_PRECLMP_EN | 0x28 | 0x07 | ? | HS/FS pre-clamp |
| SAW_SLP_HSFS_POSTCLMP_EN | 0x2C | 0x00 | ? | HS/FS post-clamp |
| SAW_SLP_CLMP_EN | 0x30 | 0x01 | ? | Clamp enable |
| SAW_SLP_RST_EN | 0x34 | 0x00 | 0x01 | **MISMATCH** |
| SAW_SPM_MPM_CFG | 0x38 | 0x00 | ? | MPM config |

#### CPU1:
Same as CPU0 except:
- SAW_SLP_CLK_EN = 0x13 (vs 0x01 for CPU0)

### Register Decoding

**SAW_CFG = 0x1C:**
- Bit [4] = 1: IRQ edge sensitive
- Bit [3] = 1: ?? (CONFIG_1)
- Bit [2] = 1: ?? (CONFIG_2)

**SAW_SPM_CTL = 0x68:**
```
Bits [7:4] = 0x6 (event output / upper config from bootloader)
Bits [3]   = 1   (rpm_bypass = 1, standalone mode - normal for boot)
Bits [2]   = 0   (reserved)
Bits [1:0] = 0x0 (mode = 0x0, CLOCK_GATING - normal for boot)
```

**SAW_SLP_CLK_EN:**
- CPU0 = 0x01: Enable clock for mode 0 (CLOCK_GATING)
- CPU1 = 0x13: Enable clock for modes 0, 1, and 4 (bits 0, 1, 4 set)

**SAW_SLP_RST_EN:**
- Should be 0x00 at boot (core reset NOT asserted)
- Runtime sets to 0x01 for power collapse (mode 0x02 with reset)

---

## Mainline Driver Status

From `drivers/soc/qcom/spm.c` lines 292-295, 315:

```c
/*
 * IMPORTANT: We do NOT initialize SPM registers at probe time.
 * The bootloader already configured SPM, and writing to these
 * registers during probe can crash the system. We only provide
 * voltage regulation functionality.
 */
...
/* All init values set to 0 - don't write anything at probe */
.spm_cfg = 0,
.pmic_dly = 0,
```

**This assumption is FALSE.** Bootloaders do NOT initialize SPM.

---

## Required Fix

### Add SPM Register Initialization

Modify `spm_reg_8660_cpu` in `drivers/soc/qcom/spm.c`:

```c
static const struct spm_reg_data spm_reg_8660_cpu = {
    .reg_offset = spm_reg_offset_8660,
    .spm_cfg = 0x1C,          // Was: 0 (skip init)
    .pmic_dly = 0x0C0CFFFF,   // Was: 0, Now: SLP_TMR_DLY (sleep timer)
    
    // Need to add new fields to struct spm_reg_data:
    .wake_tmr_dly = 0x78780FFF,      // WAKE_TMR_DLY
    .slp_clk_en_cpu0 = 0x01,         // CPU0 clock enable
    .slp_clk_en_cpu1 = 0x13,         // CPU1 clock enable (different!)
    .slp_hsfs_preclmp = 0x07,        // HSFS pre-clamp
    .slp_hsfs_postclmp = 0x00,       // HSFS post-clamp
    .slp_clmp_en = 0x01,             // Clamp enable
    .spm_mpm_cfg = 0x00,             // MPM config
    
    // Keep these at 0 - runtime will set based on mode
    .spm_ctl_init = 0x68,            // Initial SPM_CTL (clock gating + upper bits)
    .slp_rst_en_init = 0x00,         // Initial SLP_RST_EN (no reset at boot)
    
    .no_seq_ram = true,
    .set_vdd = smp_set_vdd_8660,
    .get_vdd = smp_get_vdd_8660,
    .range = &spm_8660_regulator_range,
    .init_uV = 1100000,
    .ramp_delay = 1250,
};
```

### Probe-Time Write Sequence

In `spm_dev_probe()`, after getting `reg_data`:

```c
if (drv->reg_data->spm_cfg) {
    // Write configuration registers
    spm_register_write(drv, SPM_REG_CFG, drv->reg_data->spm_cfg);
    
    if (drv->reg_data->pmic_dly)
        spm_register_write(drv, SPM_REG_PMIC_DLY, drv->reg_data->pmic_dly);
    
    // MSM8660-specific init for timer delays, clock enables, clamps
    if (drv->reg_data->no_seq_ram) {  // MSM8660 path
        // Write all 11 registers from legacy board file
        // ...
    }
    
    // Write initial SPM_CTL and RST (clock gating mode at boot)
    if (drv->reg_data->spm_ctl_init)
        spm_register_write(drv, SPM_REG_SPM_CTL, drv->reg_data->spm_ctl_init);
    if (drv->reg_data->slp_rst_en_init != ~0U)
        spm_register_write(drv, SPM_REG_RST, drv->reg_data->slp_rst_en_init);
}
```

### Per-CPU Differences

**Challenge:** CPU0 and CPU1 need different SLP_CLK_EN values (0x01 vs 0x13).

**Solution:** Pass CPU index to probe, or add per-CPU init callback.

---

## Testing Plan

After implementing SPM init:

### 1. Verify Register Writes
```bash
# Read SPM registers after boot
devmem 0x02042014 32  # CPU0 SPM_CTL, should be 0x68
devmem 0x02042024 32  # CPU0 SLP_CLK_EN, should be 0x01
devmem 0x02052024 32  # CPU1 SLP_CLK_EN, should be 0x13
```

### 2. Test Power Collapse
```bash
# Offline CPU1
echo 0 > /sys/devices/system/cpu/cpu1/hotplug/target

# Wait for idle
sleep 5

# Check if cpu-spc entered
cat /sys/devices/system/cpu/cpu0/cpuidle/state1/usage  # Should be > 0
```

### 3. Check for Crashes
- System should not crash during boot
- SPM writes during probe should be safe (legacy did this successfully)

---

## Risk Assessment

**Low Risk:**
- Legacy kernel did this exact sequence at boot for years (webOS 1.0 through 3.0.5)
- We're using identical register values from validated board file
- MSM8660 SAW has no sequence RAM to corrupt
- Writes are to unclaimed registers (bootloader doesn't touch them)

**Mitigation:**
- Add verbose logging during probe to show each register write
- Read-back verification after each write
- If system crashes, can rollback to `.spm_cfg = 0` (current state)

---

## Recommendations

1. **Immediate:** Implement SPM register initialization (Option B)
2. **Testing:** Lower min-residency to 500us temporarily to verify SPM mode programming works
3. **Long-term:** Keep min-residency at 3000us, implement RPM sleep orchestrator (Task #7) for deep states

**Do NOT:**
- Rely on bootloader initialization (proven false)
- Use target_residency=0 (mainline cpuidle framework doesn't support this)

---

## Files to Modify

1. `drivers/soc/qcom/spm.c`:
   - Update `spm_reg_data` struct to add 8 new init fields
   - Update `spm_reg_8660_cpu` with legacy values
   - Update `spm_dev_probe()` to write all init registers
   - Fix incorrect comment about bootloader init

2. `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi`:
   - Consider lowering min-residency temporarily for testing (3000 → 500)

---

## Cross-References

- **Task #3 results:** `reports/power-collapse/` (this investigation)
- **Task #4 SPM analysis:** `reports/power-collapse/MSM8660-SPM-SEQUENCE-ANALYSIS.md`
- **Legacy board file:** webOS kernel `arch/arm/mach-msm/board-tenderloin.c` lines 277-395
- **Legacy SPM driver:** webOS kernel `arch/arm/mach-msm/spm.c`
- **Mainline SPM driver:** `drivers/soc/qcom/spm.c` lines 249-330

---

**Status:** INVESTIGATION COMPLETE - Ready to implement SPM initialization patch
**Next Action:** Create patch adding 11-register SPM init to mainline driver
