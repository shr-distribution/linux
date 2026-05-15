---
domain: spm-init
created: "2026-05-15"
last_edited: "2026-05-15"
status: draft
---

# Cavekit: SPM Register Initialization

## Scope

Initialize Subsystem Power Manager (SPM/SAW) registers on MSM8660/APQ8060 at driver probe time to enable CPU power collapse states.

## Context

**Problem:** Mainline `drivers/soc/qcom/spm.c` assumes bootloaders initialize SPM registers, but bootloader analysis proves they do not. Current hardware reads show SPM_CTL=0x08 (default) vs expected 0x68.

**Evidence:** Investigation in `reports/power-collapse/SPM-INIT-INVESTIGATION-RESULTS.md` shows:
- HTC TrustZone, SBL3, and TouchPad APPSBL contain 0 references to SAW base addresses
- Legacy webOS kernel explicitly wrote 11 SPM registers at boot (`board-tenderloin.c` lines 277-395)
- Current device registers are in uninitialized state

**Impact:** Without initialization, CPU idle power collapse (cpu-spc) cannot function.

## Requirements

### R1: SPM Register Data Structure Extension
**Description:** Extend `struct spm_reg_data` to hold all 11 MSM8660 initialization values

**Acceptance Criteria:**
- [ ] Add fields: `wake_tmr_dly`, `slp_clk_en_cpu0`, `slp_clk_en_cpu1`, `slp_hsfs_preclmp`, `slp_hsfs_postclmp`, `slp_clmp_en`, `spm_mpm_cfg`, `spm_ctl_init`, `slp_rst_en_init`
- [ ] Fields use exact register widths (u32)
- [ ] Per-CPU clock enable fields allow CPU0/CPU1 differences

### R2: MSM8660 CPU Register Values
**Description:** Populate `spm_reg_8660_cpu` with validated legacy values

**Acceptance Criteria:**
- [ ] `spm_cfg = 0x1C` (was 0)
- [ ] `pmic_dly = 0x0C0CFFFF` (SLP_TMR_DLY, was 0)
- [ ] `wake_tmr_dly = 0x78780FFF`
- [ ] `slp_clk_en_cpu0 = 0x01`
- [ ] `slp_clk_en_cpu1 = 0x13`
- [ ] `slp_hsfs_preclmp = 0x07`
- [ ] `slp_hsfs_postclmp = 0x00`
- [ ] `slp_clmp_en = 0x01`
- [ ] `spm_mpm_cfg = 0x00`
- [ ] `spm_ctl_init = 0x68`
- [ ] `slp_rst_en_init = 0x00`
- [ ] All values match `board-tenderloin.c` msm_spm_data[] array

### R3: Probe-Time Initialization Sequence
**Description:** Write all SPM registers during `spm_dev_probe()` when `reg_data->spm_cfg != 0`

**Acceptance Criteria:**
- [ ] Write SAW_CFG register
- [ ] Write SAW_SPM_SLP_TMR_DLY register
- [ ] Write SAW_SPM_WAKE_TMR_DLY register
- [ ] Write SAW_SLP_CLK_EN register (per-CPU value)
- [ ] Write SAW_SLP_HSFS_PRECLMP_EN register
- [ ] Write SAW_SLP_HSFS_POSTCLMP_EN register
- [ ] Write SAW_SLP_CLMP_EN register
- [ ] Write SAW_SLP_RST_EN register
- [ ] Write SAW_SPM_MPM_CFG register
- [ ] Write SAW_SPM_CTL register (last)
- [ ] Sequence matches legacy initialization order
- [ ] Read-back verification after critical writes
- [ ] Verbose logging shows each register write

### R4: Per-CPU Clock Enable Handling
**Description:** Apply correct SLP_CLK_EN value based on CPU index

**Acceptance Criteria:**
- [ ] CPU0 writes 0x01 to SAW_SLP_CLK_EN (offset 0x24)
- [ ] CPU1 writes 0x13 to SAW_SLP_CLK_EN (offset 0x24)
- [ ] Implementation uses CPU index from DT or platform data
- [ ] Fallback handles single-CPU case gracefully

### R5: Documentation Update
**Description:** Fix incorrect bootloader assumption comment

**Acceptance Criteria:**
- [ ] Remove comment claiming "bootloader already configured SPM"
- [ ] Add comment explaining bootloaders do NOT initialize SPM
- [ ] Reference investigation report in comment

## Out of Scope

- Runtime SPM mode switching (already implemented in `spm_set_low_power_mode()`)
- SPM sequence RAM programming (MSM8660 has no sequence RAM)
- L2/VDD voltage switching (handled by separate RPM orchestrator)
- Target residency tuning (separate DT/cpuidle concern)

## Dependencies

**Depends On:**
- Existing `drivers/soc/qcom/spm.c` driver structure
- `SPM_REG_*` offset definitions

**Enables:**
- CPU hotplug power collapse (when CPU offlined)
- CPU idle power collapse via cpuidle (cpu-spc state)

## Testing Strategy

### Unit Test: Register Initialization
```bash
# Boot with patch, read registers via devmem
devmem 0x02042010 32  # CPU0 SAW_CFG, expect 0x1C
devmem 0x02042014 32  # CPU0 SPM_CTL, expect 0x68
devmem 0x02042024 32  # CPU0 SLP_CLK_EN, expect 0x01
devmem 0x02052024 32  # CPU1 SLP_CLK_EN, expect 0x13
devmem 0x02042034 32  # CPU0 SLP_RST_EN, expect 0x00
```

### Integration Test: Power Collapse Entry
```bash
# Offline CPU1 to trigger hotplug power collapse
echo 0 > /sys/devices/system/cpu/cpu1/hotplug/target
sleep 5

# Verify SPM mode was programmed
devmem 0x02052014 32  # CPU1 SPM_CTL, should show power collapse mode

# Check cpuidle state usage
cat /sys/devices/system/cpu/cpu0/cpuidle/state1/usage  # Should be > 0
```

### Regression Test: Boot Stability
- [ ] System boots successfully
- [ ] No SPM-related crashes in dmesg
- [ ] All CPUs online after boot
- [ ] CPU hotplug still functional

## Risk Assessment

**Low Risk** because:
- Legacy kernel used this exact sequence for years (webOS 1.0-3.0.5)
- Register values are validated from production board file
- MSM8660 SAW has no sequence RAM to corrupt
- Writes are to unclaimed registers (bootloader doesn't touch them)

**Mitigation:**
- Add verbose logging during probe
- Read-back verification after each write
- Can rollback to `.spm_cfg = 0` if crashes occur

## Cross-References

**Investigation Report:**
- `reports/power-collapse/SPM-INIT-INVESTIGATION-RESULTS.md`

**Legacy Reference:**
- webOS kernel `arch/arm/mach-msm/board-tenderloin.c` lines 277-395
- webOS kernel `arch/arm/mach-msm/spm.c`

**Mainline Files to Modify:**
- `drivers/soc/qcom/spm.c` (primary)
- `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi` (optional min-residency test)

**Related Work:**
- Power collapse hotplug implementation (commit 3e823a185257)
- CPU hotplug enablement (commit d9ff091caa56)
- SPM driver foundation (commit efcc84bb7319)
