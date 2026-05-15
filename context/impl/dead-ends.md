---
created: "2026-05-15"
last_updated: "2026-05-15"
---

# Dead Ends & Failed Approaches

## SPM Init CPU Hotplug Panic (2026-05-15)

### What We Tried
Tested CPU hotplug power collapse after SPM register initialization:
```bash
echo 0 > /sys/devices/system/cpu/cpu1/hotplug/target
```

### What Happened
**Kernel panic** in `tick_nohz_get_sleep_length+0x80/0xf4` called from `menu_select+0x538/0x9d8`

**Full panic:**
```
[  537.500880][    T0] Code: e3001547 e3a0e001 e5cce005 ebfcbd2c (e5993010) 
[  537.511988][    T0] Kernel panic - not syncing: Fatal exception
```

**Context:** CPU1 going offline, entering idle state selection, menu governor calls tick_nohz_get_sleep_length() which dereferences NULL/invalid pointer.

### Why It Failed
**NOT an SPM register issue** - all SPM registers verified correctly:
- ✅ CPU0 SAW_CFG = 0x1C
- ✅ CPU0 SPM_CTL = 0x68  
- ✅ CPU0 SLP_CLK_EN = 0x01
- ✅ CPU1 SLP_CLK_EN = 0x13

**Root cause:** cpuidle menu governor / timer tick subsystem bug during CPU hotplug. The tick_sched structure or timer state for CPU1 is being accessed after teardown or before proper initialization.

### Lesson Learned
SPM initialization is correct, but CPU hotplug with cpuidle enabled triggers a separate kernel bug in the timer/cpuidle interaction. This is a **cpuidle hotplug ordering issue**, not a power management hardware issue.

### What to Do Instead
**Option 1:** Disable cpuidle during hotplug testing
```bash
# Disable cpuidle before testing hotplug
echo 1 > /sys/devices/system/cpu/cpuidle/off
echo 0 > /sys/devices/system/cpu/cpu1/hotplug/target
```

**Option 2:** Test SPM power collapse via direct SPM register writes (bypass cpuidle)

**Option 3:** Fix cpuidle/hotplug interaction first before testing integrated flow

**Option 4:** Use idle=poll boot parameter to disable cpuidle entirely during testing

### Status
**FIXED:** Commit 5548d5d0a35a adds NULL check in tick_nohz_get_sleep_length()

**Fix Details:**
```c
if (unlikely(!dev)) {
    *delta_next = 0;
    return 0;  // Force shortest sleep during hotplug teardown
}
```

Returns 0 during the race window when tick device is torn down but cpuidle still active. Safe because CPU is being shut down anyway.

**Next Steps:** Rebuild kernel and retest CPU hotplug power collapse
