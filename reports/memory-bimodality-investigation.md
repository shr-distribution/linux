# Memory Test Bimodality Investigation

## Problem Statement
Memory performance tests on the HP TouchPad show bimodal distribution - two distinct performance levels appearing in test results, suggesting some system component is switching between two states during testing.

## Excluded Causes

### 1. CPU Idle States (cpuidle)
- **Status:** Excluded
- **Config:** `CONFIG_CPU_IDLE=y`, `CONFIG_ARM_CPUIDLE=y`
- **Notes:** Testing confirmed cpuidle is not the cause

### 2. CPU Frequency Scaling (cpufreq)
- **Status:** Excluded
- **Config:** `CONFIG_CPU_FREQ=y` with multiple governors enabled
- **Testing:** Ran with `performance` governor and kept CPU busy
- **Notes:** Bimodality persists even with locked CPU frequency

## Architecture Overview

### Power Management Components

```
+------------------+     +------------------+
|   CPU0 (Scorpion)|     |   CPU1 (Scorpion)|
|   SAW0 (SPM)     |     |   SAW1 (SPM)     |
|   PM8901 S0 reg  |     |   PM8901 S1 reg  |
+--------+---------+     +--------+---------+
         |                        |
         +------------+-----------+
                      |
              +-------v-------+
              |   L2 Cache    |
              +-------+-------+
                      |
         +------------+------------+
         |                         |
+--------v--------+      +---------v--------+
|  APPSS Fabric   |      |   MMSS Fabric    |
|  (RPM_AFAB_CLK) |      | (RPM_MMFAB_CLK)  |
+---------+-------+      |  (RPM_SMI_CLK)   |
          |              +---------+--------+
          |                        |
+---------v------------------------v--------+
|              System Fabric                |
|            (RPM_SFAB_CLK)                 |
+---------------------+---------------------+
                      |
          +-----------v-----------+
          |    EBI (DDR Memory)   |
          |    (RPM_EBI1_CLK)     |
          +-----------------------+
```

### RPM-Controlled Clocks (Potential Bimodality Sources)

| Clock | Description | DT Reference |
|-------|-------------|--------------|
| `RPM_APPS_FABRIC_CLK` | CPU/Apps fabric clock | `apps_fabric` interconnect |
| `RPM_SYS_FABRIC_CLK` | System peripherals fabric | `system_fabric` interconnect |
| `RPM_MM_FABRIC_CLK` | Multimedia fabric | `mmss_fabric` interconnect |
| `RPM_SMI_CLK` | Scalable Memory Interface | `mmss_fabric` interconnect |
| `RPM_EBI1_CLK` | DDR memory controller | Not exposed in DT |

### SAW/SPM Configuration

The SPM driver (`drivers/soc/qcom/spm.c`) for MSM8660/APQ8060:
- Uses `spm_reg_8660_cpu` configuration
- Skips register initialization at probe (relies on bootloader config)
- Provides voltage regulation via `smp_set_vdd_8660()`
- Voltage range: 700mV - 1400mV (PM8901 SMPS Band 2)

**Key code path:** `spm.c:438` - `smp_set_vdd_8660()` writes to SAW VCTL register

## Remaining Suspects (Priority Order)

### 1. RPM Fabric Clock Scaling (HIGH)
- **Location:** `drivers/clk/qcom/clk-rpm.c`, `drivers/interconnect/qcom/msm8660.c`
- **Mechanism:** Interconnect driver calls `clk_set_rate()` based on bandwidth demands
- **Investigation:**
  ```bash
  # Check current fabric clock rates
  cat /sys/kernel/debug/clk/clk_summary | grep -E "afab|sfab|mmfab|smi|ebi"

  # Monitor clock changes during test
  while true; do cat /sys/kernel/debug/clk/clk_summary | grep afab; sleep 0.1; done
  ```

### 2. EBI1/DDR Clock Scaling (HIGH)
- **Location:** RPM firmware controls `RPM_EBI1_CLK`
- **Mechanism:** Memory controller clock may scale based on traffic
- **Investigation:**
  ```bash
  # Check EBI clock if exposed
  cat /sys/kernel/debug/clk/clk_summary | grep ebi

  # Check RPM message log if available
  cat /sys/kernel/debug/rpm_log 2>/dev/null
  ```

### 3. L2 Cache Power States (MEDIUM)
- **Location:** L2 cache controller, potentially managed by SPM
- **Mechanism:** L2 may have retention/active states affecting latency
- **Investigation:**
  ```bash
  # Check L2 cache info
  cat /sys/devices/system/cpu/cpu0/cache/index2/size

  # Look for L2-related power controls
  find /sys -name "*l2*" 2>/dev/null
  ```

### 4. Memory Self-Refresh (MEDIUM)
- **Location:** DDR controller / RPM firmware
- **Mechanism:** DDR entering self-refresh between accesses
- **Investigation:** Requires traffic analysis or RPM debug

### 5. SAW/SPM Voltage Transitions (LOW)
- **Location:** `drivers/soc/qcom/spm.c`
- **Mechanism:** Voltage changes during regulator operations
- **Investigation:**
  ```bash
  # Check regulator status
  cat /sys/class/regulator/*/microvolts 2>/dev/null

  # Monitor SAW registers (if debugfs available)
  cat /sys/kernel/debug/regulator/*/status
  ```

### 6. Thermal Throttling (LOW)
- **Location:** Thermal framework
- **Mechanism:** Could cause frequency/voltage changes
- **Investigation:**
  ```bash
  cat /sys/class/thermal/thermal_zone*/temp
  cat /sys/class/thermal/thermal_zone*/policy
  ```

## Investigation Plan

### Phase 1: Clock Rate Monitoring
1. Enable debugfs clock tracing
2. Run memory benchmark while monitoring all RPM clock rates
3. Correlate clock rate changes with performance dips

### Phase 2: Interconnect Bandwidth Locking
1. Check if interconnect bandwidth can be locked via sysfs
2. Potentially modify `msm8660.c` to skip dynamic clock adjustment
3. Test with fixed fabric clock rates

### Phase 3: RPM Communication Tracing
1. Enable RPM message tracing if available
2. Monitor what clock/voltage requests are being made
3. Identify which subsystem is triggering changes

### Phase 4: Kernel Instrumentation
1. Add tracepoints to `clk_set_rate()` calls in interconnect driver
2. Add tracepoints to SPM voltage setting functions
3. Correlate with memory test timing

## Quick Test Commands

```bash
# Comprehensive clock status
cat /sys/kernel/debug/clk/clk_summary

# Interconnect paths and bandwidth
cat /sys/kernel/debug/interconnect/interconnect_summary 2>/dev/null

# All regulator states
for r in /sys/class/regulator/regulator.*; do
  echo "=== $(cat $r/name) ==="
  cat $r/state $r/microvolts 2>/dev/null
done

# Power domain states
cat /sys/kernel/debug/pm_genpd/pm_genpd_summary 2>/dev/null

# Check for any runtime PM activity
cat /sys/kernel/debug/runtime_pm 2>/dev/null
```

## Files of Interest

| File | Purpose |
|------|---------|
| `drivers/soc/qcom/spm.c` | SAW/SPM power manager driver |
| `drivers/clk/qcom/clk-rpm.c` | RPM clock driver |
| `drivers/interconnect/qcom/msm8660.c` | Bus fabric interconnect driver |
| `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi` | SoC device tree (clocks, fabrics) |

## Next Steps

1. [ ] Run clock monitoring commands on device during memory test
2. [ ] Check if fabric clocks are changing during bimodal performance
3. [ ] If clocks are stable, investigate DDR self-refresh via RPM
4. [ ] Consider adding kernel tracing for deeper analysis
