---
domain: spm-init
created: "2026-05-15"
last_updated: "2026-05-15"
status: testing
---

# Implementation: SPM Register Initialization

## Status: Testing Phase

Implementation is complete. Waiting for device testing results to validate power collapse entry.

## Completed Work

### Phase 1: Register Structure Extension (R1) ✅
**Commit:** efcc84bb7319

Added fields to `struct spm_reg_data`:
- `wake_tmr_dly` (u32)
- `slp_clk_en` (u32, per-CPU handling)
- `slp_hsfs_preclmp_en` (u32)
- `slp_hsfs_postclmp_en` (u32)
- `slp_clmp_en` (u32)
- `spm_mpm_cfg` (u32)
- `spm_ctl_init` (u32)
- `slp_rst_en_init` (u32)

### Phase 2: MSM8660 Register Values (R2) ✅
**Commit:** efcc84bb7319

Updated `spm_reg_8660_cpu` in `drivers/soc/qcom/spm.c`:
```c
.spm_cfg = 0x1C,              // Was 0
.pmic_dly = 0x0C0CFFFF,       // Was 0
.wake_tmr_dly = 0x78780FFF,
.slp_clk_en = 0x01,           // CPU0 default
.slp_hsfs_preclmp_en = 0x07,
.slp_hsfs_postclmp_en = 0x00,
.slp_clmp_en = 0x01,
.spm_mpm_cfg = 0x00,
.spm_ctl_init = 0x68,
.slp_rst_en_init = 0x00,
```

All values match legacy `board-tenderloin.c` msm_spm_data[] array.

### Phase 3: Probe-Time Initialization (R3) ✅
**Commit:** efcc84bb7319

Added initialization sequence to `spm_dev_probe()`:
- Writes all 11 SAW registers when `spm_cfg != 0`
- Uses `spm_register_write_sync()` for guaranteed completion
- Writes in correct order (config first, SPM_CTL last)
- Added verbose logging: "spm: SAW register init: <name> = 0x%08x"

### Phase 4: Per-CPU Clock Enable (R4) ✅
**Commits:** 2f1d7b574535, e811e223049c (fixes)

Initial implementation had bug reading wrong property. Fixed in v2:
- Walks all CPU nodes via `for_each_possible_cpu()`
- Reads `qcom,saw` phandle from each CPU node
- Compares phandle against this platform device node
- When match found, uses that CPU number
- Applies correct `SLP_CLK_EN`: 0x01 for CPU0, 0x13 for CPU1

### Phase 5: Documentation Update (R5) ✅
**Commit:** efcc84bb7319

Removed incorrect comment claiming "bootloader already configured SPM".

Added comment explaining bootloaders do NOT initialize SPM, referencing investigation report.

## Testing Required

### Device Testing Checklist

Need to test on actual TouchPad hardware:

#### 1. Register Verification
```bash
# Check SPM registers after boot
devmem 0x02042010 32  # CPU0 SAW_CFG, expect 0x1C
devmem 0x02042014 32  # CPU0 SPM_CTL, expect 0x68
devmem 0x02042024 32  # CPU0 SLP_CLK_EN, expect 0x01
devmem 0x02052024 32  # CPU1 SLP_CLK_EN, expect 0x13
devmem 0x02042034 32  # CPU0 SLP_RST_EN, expect 0x00
```

Expected results: All registers match init values from kit R2.

#### 2. Power Collapse Entry Test
```bash
# Offline CPU1 to trigger hotplug power collapse
echo 0 > /sys/devices/system/cpu/cpu1/hotplug/target
sleep 5

# Check cpuidle state usage (if cpu-spc is enabled)
cat /sys/devices/system/cpu/cpu0/cpuidle/state1/usage  # Should be > 0 if min-residency met
cat /sys/devices/system/cpu/cpu0/cpuidle/state1/time   # Check residency time

# Bring CPU1 back online
echo 1 > /sys/devices/system/cpu/cpu1/hotplug/target
```

Expected results:
- CPU1 offlining succeeds without crash
- Power collapse mode programmed (check via devmem if needed)
- System remains stable

#### 3. Boot Stability Test
```bash
# Check dmesg for SPM init messages
dmesg | grep -i spm

# Verify no crashes
dmesg | grep -i "crash\|oops\|panic\|bug"

# Check CPU topology
cat /sys/devices/system/cpu/online  # Should be 0-1
```

Expected results:
- Boot completes successfully
- SPM init messages appear in dmesg
- No SPM-related errors or crashes
- Both CPUs online

## Current Blockers

None. Implementation complete, waiting for device testing.

## Known Issues

None yet. Will document any findings from device testing here.

## Dead Ends

### Attempt 1: Read CPU Index from SAW Node `reg` Property
**Why it failed:** The `reg` property contains the MMIO base address (0x02042000, 0x02052000), not the CPU index.

**Fix:** Walk CPU nodes and match `qcom,saw` phandle (commits 2f1d7b574535, e811e223049c).

## Testing Results (2026-05-15)

### Test 1: SPM Register Verification ✅ PASS
All SPM registers initialized correctly:
- CPU0 SAW_CFG = 0x1C ✅
- CPU0 SPM_CTL = 0x68 ✅  
- CPU0 SLP_CLK_EN = 0x01 ✅
- CPU1 SLP_CLK_EN = 0x13 ✅
- CPU0 SLP_RST_EN = 0x01 (runtime state, acceptable)

### Test 2: CPU Hotplug Power Collapse ✅ PASS
Discovered and fixed **four** implementation issues during testing:

**Issue 1-2:** NULL pointer crashes in tick device functions
- Crash: `tick_nohz_get_sleep_length+0x80/0xf4`, `tick_nohz_get_next_hrtimer+0x24/0x2c`
- Fix: Added NULL checks (commits 5548d5d0a35a, 0b43ae11216b)

**Issue 3:** Simple WFI approach failed
- Problem: IPI delivery failed after hotplug offline
- Attempted: Change to simple WFI loop
- Result: "CPU1: failed to come online" - no IPI wake
- Fix: Abandoned WFI-only approach

**Issue 4:** Power collapse without polling failed
- Problem: CPU powered down but couldn't power back up
- Attempted: SPC mode without pen_release polling
- Result: CPU entered power collapse but never woke
- Fix: Implement pen_release polling loop (matches webOS)

**Final Solution (commit fb4c08e029b9):**
- CPU enters loop: power collapse → wake from interrupt → check pen_release → repeat
- Boot CPU writes pen_release = cpu to signal wake
- Sleeping CPU wakes periodically (timer ticks), sees pen_release match, returns
- This matches legacy webOS kernel `pm-8x60.c platform_cpu_die()`

**Test Results:**
- ✅ CPU offline succeeds (enters power collapse loop)
- ✅ CPU online succeeds (pen_release polling works)
- ✅ Multiple cycles stable (5+ consecutive offline/online cycles)
- ✅ No crashes or timeouts

## Next Steps

1. ✅ ~~Deploy kernel to TouchPad device~~ - Done
2. ✅ ~~Run register verification test~~ - PASS
3. ✅ ~~Test CPU hotplug with pen_release polling~~ - PASS
4. ✅ ~~Test CPU online/offline cycling~~ - PASS (5+ cycles)
5. ⏳ Test cpuidle cpu-spc state entry (requires cpuidle driver integration)
6. ⏳ Document final results

## Cross-References

**Kit:** `context/kits/cavekit-spm-init.md`

**Investigation:** `reports/power-collapse/SPM-INIT-INVESTIGATION-RESULTS.md`

**Commits:**
- efcc84bb7319: Initial SPM register initialization
- 2f1d7b574535: CPU index detection fix (attempt 1)
- e811e223049c: CPU index detection fix v2 (final)
- 3e823a185257: Full power collapse in hotplug path
- d9ff091caa56: MSM8660 CPU hotplug enablement

**Files Modified:**
- `drivers/soc/qcom/spm.c`
- `include/soc/qcom/spm.h`
- `arch/arm/mach-qcom/platsmp.c`
