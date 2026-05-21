---
domain: mpm-boot-hang
created: "2026-05-15"
last_updated: "2026-05-21"
status: RESOLVED-msm8660-mpm-driver-working-on-device
---

# Implementation: MPM Boot Hang Investigation

## Status: RESOLVED. MSM8660 MPM driver working on-device 2026-05-21.

The MPM boot-hang blocker is resolved. The active driver is
`drivers/irqchip/irq-msm8660-mpm.c` (the existing `irq-msm8660-wakeup.c`
was renamed and extended in commit `993a638936e4`).

### Origin of resolution

The deep-dive in `reports/MPM-LEGACY-DEEPDIVE-2026-05-21.md` confirmed
that the mainline `qcom-mpm` driver is fundamentally incompatible with
MSM8660 for three reasons:

  1. vMPM lives inside the RPM control block (not a separate SRAM)
  2. Wake notification is a raw GCC MMIO write (not IPCC mailbox)
  3. `IRQCHIP_DECLARE` early init runs before platform devices exist

The existing `irq-msm8660-wakeup.c` (commits `de8f2cabf6f1`,
`b92823ef0859`, `23de1095f7d2` from May 16) already implemented the
working approach — a regular platform driver accessing the vMPM via a
syscon phandle to the RPM block. It just wasn't documented here.

Commit `993a638936e4` then:
  - Renamed the driver from `msm8660-wakeup` to `msm8660-mpm` (matches
    mainline Qualcomm naming convention)
  - Added a raw-pin API (`msm8660_mpm_set_pin_wake/enable_pin/
    set_pin_type/get`) for wake sources without GIC IRQ mapping
    (SDC3/4 DATx pins 21-24)
  - Removed the duplicate driver attempt I had landed in `4397a0b20949`
  - Removed the conflicting `qcom,msm8660-mpm` IRQCHIP_MATCH from
    mainline `irq-qcom-mpm.c` so it doesn't race our platform driver

### On-device verification (kernel `g993a638936e4`, 2026-05-21)

```
$ ls /sys/bus/platform/drivers/msm8660-mpm/
soc:interrupt-controller          <-- driver bound to MPM DT node

$ cat /proc/interrupts | grep mpm
 29:          0          0 GIC-0  34 Level     msm8660-mpm
 32:        997          0 msm8660-mpm 100 Level     ci_hdrc_msm
                                              ^^^ USB1_HS routed
                                                  through MPM irqdomain

$ grep msm8660_mpm /proc/kallsyms
T msm8660_mpm_enable_pin
T msm8660_mpm_set_pin_wake
T msm8660_mpm_set_pin_type
T msm8660_mpm_get
T msm8660_mpm_remove
T msm8660_mpm_probe                <-- all consumer API exported
```

No `qcom_mpm` probe-failure noise in journal — the mainline driver
no longer matches our compatible.

### What this unblocks

- **wifi-suspend-wake R1**: MPM is functional as wakeup-interrupt
  controller. ✓
- **wifi-suspend-wake R2**: SDC4 DAT1 wake-source registration can
  now proceed; mmci shim or DT wiring to call
  `msm8660_mpm_set_pin_wake(handle, MSM8660_MPM_PIN_SDC4_DAT1, true)`.
- **spm-init PM-2**: cpuidle deep sleep (Power Collapse) wake-IRQ
  delivery is now possible through MPM.

## Historical: Three Failed Mainline Attempts (kept for reference)

PM-2 (cpuidle deep sleep) requires MPM for wake interrupt delivery, but enabling the MPM device tree node causes an early boot hang.

## Problem Summary

**Symptom:** Boot hangs at "Zone ranges:" during early memory initialization when MPM node status is "okay".

**Boot log:**
```
[    0.000000][    T0] Booting Linux on physical CPU 0x0
[    0.000000][    T0] Linux version 6.18.0-luneos-gbfce295a7a15
[    0.000000][    T0] CPU: ARMv7 Processor [510f02d2] revision 2 (ARMv7)
[    0.000000][    T0] OF: fdt: Machine model: APQ8060 HP TouchPad (Topaz WiFi)
[    0.000000][    T0] earlycon: msm_serial_dm0 at MMIO 0x19c40000
[    0.000000][    T0] Memory policy: Data cache writealloc
[    0.000000][    T0] cma: Reserved 32 MiB at 0x7a000000
[    0.000000][    T0] Zone ranges:
<HANG>
```

The hang occurs **before** any driver probe, during early device tree resource parsing.

## Root Cause Analysis

### Memory Region Overlap

MPM vMPM registers are INSIDE the RPM memory region:
- **RPM region:** `reg = <0x00104000 0x1000>` (0x00104000 to 0x00104FFF, 4KB)
- **MPM region:** `reg = <0x001049d8 0x448>` (0x001049D8 to 0x00104E1F, 1096 bytes)

MPM base is RPM_BASE + 0x9D8, well within RPM's 4KB region.

### Why It Hangs

During early boot, the kernel's device tree parsing code (`drivers/of/address.c`) walks all nodes with `reg` properties and reserves memory regions. When it encounters the MPM node:

1. MPM node has `reg = <0x001049d8 0x448>`
2. Kernel tries to reserve this range
3. This range **overlaps** with the already-claimed RPM region at `0x00104000`
4. The reservation conflict causes a silent hang during `Zone ranges:` setup

This happens even with `status = "disabled"` removed - the hang is in DT parsing, not driver probe.

### Why devm_ioremap Was Supposed to Help

The MPM driver uses `devm_ioremap()` (non-exclusive) instead of `devm_ioremap_resource()` (exclusive):

```c
/* MSM8660 vMPM registers reside within the RPM's memory
 * region (RPM_BASE + 0x9D8) which is already claimed by
 * the RPM driver. Use devm_ioremap to map without
 * exclusive reservation.
 */
priv->base = devm_ioremap(dev, r->start, resource_size(r));
```

But this only helps at **driver probe time**. The hang happens **earlier**, during device tree resource reservation, before any drivers load.

## Previous Attempts

### Attempt 1: reserved-memory Node (FAILED)
**Commit:** 8b5eac1107df, then reverted by 4c8f9ad4a200

Used `reserved-memory` with `no-map` flag:
```dts
mpm_sram: mpm@1049d8 {
    compatible = "qcom,msm-imem";
    reg = <0x001049d8 0x448>;
    no-map;
};
```

**Result:** Boot hang in reserved-memory parsing. The reserved-memory framework is designed for DRAM regions only and cannot handle MMIO addresses.

### Attempt 2: Direct reg Property (FAILED)
**Commits:** bfce295a7a15, reverted by dcd05c5f0d78

Removed reserved-memory node, used direct `reg` property with driver's non-exclusive ioremap:
```dts
mpm: interrupt-controller@1049d8 {
    compatible = "qcom,msm8660-mpm", "qcom,mpm";
    reg = <0x001049d8 0x448>;
    ...
    status = "okay";  /* enabled in tenderloin-common.dtsi */
};
```

**Result:** Still hangs at "Zone ranges:" during early boot. The overlap with RPM memory causes conflict during DT resource parsing.

### Attempt 3: Syscon Approach (FAILED)
**Commits:** 87ab6e9eee4a, then disabled

Implemented full syscon support:
- Added "syscon" compatible to RPM node
- Modified MPM driver to use regmap (syscon_regmap_lookup_by_phandle)
- Removed `reg` property from MPM node
- Added `qcom,rpm-syscon` and `qcom,mpm-offset` properties

```dts
rpm: rpm@104000 {
    compatible = "qcom,rpm-msm8660", "syscon";
    reg = <0x00104000 0x1000>;
    ...
};

mpm: interrupt-controller {
    compatible = "qcom,msm8660-mpm", "qcom,mpm";
    qcom,rpm-syscon = <&rpm>;
    qcom,mpm-offset = <0x9d8>;
    ...
};
```

**Result:** Boot progressed past "Zone ranges:" (avoiding previous hang!) but now hangs at reserved memory parsing for smem@40000000:
```
[    0.000000][    T0] Zone ranges:
[    0.000000][    T0]   Normal   [mem 0x0000000040200000-0x000000007f5fffff]
[    0.000000][    T0] Movable zone start for each node
...
[    0.000000][    T0] OF: reserved mem: 0x40000000..0x401fffff (2048 KiB) nomap non-reusable smem@40000000
<HANG>
```

**Root cause:** The MPM driver uses `IRQCHIP_MATCH` which initializes during early irqchip init, before platform devices are ready. The `of_find_device_by_node()` call in `qcom_mpm_init()` fails or hangs because platform device infrastructure isn't fully initialized yet.

## Possible Solutions

### Option A: Remove reg Property, Use Syscon
Use a syscon phandle to access RPM memory region without claiming it:

```dts
rpm: rpm@104000 {
    compatible = "qcom,rpm-msm8660", "syscon";
    reg = <0x00104000 0x1000>;
    ...
};

mpm: interrupt-controller {
    compatible = "qcom,msm8660-mpm", "qcom,mpm";
    /* No reg property */
    qcom,rpm-syscon = <&rpm>;
    qcom,mpm-offset = <0x9d8>;
    ...
};
```

**Driver changes needed:**
- Parse `qcom,rpm-syscon` phandle
- Use `syscon_regmap_lookup_by_phandle()` to get regmap
- Use `regmap_read/write()` instead of direct MMIO
- Handle offset within RPM region

### Option B: Let RPM Driver Export MPM Interface
Have the RPM driver expose MPM register access via a custom API:

```c
/* In drivers/soc/qcom/rpm-msm8660.c */
void __iomem *qcom_rpm_get_mpm_base(struct device *rpm_dev) {
    return rpm->base + 0x9d8;
}
EXPORT_SYMBOL_GPL(qcom_rpm_get_mpm_base);
```

MPM driver finds RPM device and calls this function.

**Issues:**
- Creates tight coupling between RPM and MPM drivers
- Load order dependency (MPM must probe after RPM)

### Option C: Platform-Specific IRQ-Only MPM
Skip MPM driver entirely for MSM8660, implement minimal wake interrupt routing:

- Create `drivers/irqchip/irq-msm8660-wakeup.c`
- Manually program vMPM registers during suspend
- No genpd, no hierarchical irqdomain
- Just enough to enable cpuidle SPC/PC states

**Pros:**
- Avoids DT resource conflict
- Simpler than full MPM driver

**Cons:**
- MSM8660-specific code duplication
- Doesn't follow mainline MPM architecture

### Option D: Move MPM Under RPM Node
Make MPM a child node of RPM:

```dts
rpm: rpm@104000 {
    compatible = "qcom,rpm-msm8660";
    reg = <0x00104000 0x1000>;
    
    mpm: interrupt-controller {
        compatible = "qcom,msm8660-mpm", "qcom,mpm";
        reg = <0x9d8 0x448>;  /* Relative to RPM base */
        ...
    };
};
```

**Driver changes:**
- MPM driver becomes a sub-device of RPM
- Parse `reg` relative to parent's base
- RPM driver must instantiate MPM sub-device

## Legacy Kernel Approach

The webOS kernel (`arch/arm/mach-msm/mpm.c`) directly accesses vMPM registers:

```c
#define MSM_RPM_BASE		IOMEM(0xFA800000)
#define MSM_MPM_BASE		(MSM_RPM_BASE + 0x200 + 0x9d8)

static void __iomem *msm_mpm_base;

static void msm_mpm_init(void) {
    msm_mpm_base = MSM_MPM_BASE;  /* Static mapping */
    /* Direct register access via writel_relaxed() */
}
```

No device tree node, no resource reservation - just a hardcoded address offset within RPM's already-mapped region.

## Root Cause: IRQCHIP_MATCH Early Init Timing

The fundamental issue is that MPM uses `IRQCHIP_MATCH` which initializes during `irqchip_init()`, very early in boot. At this point:
- Platform device infrastructure is not ready
- `of_find_device_by_node()` cannot create platform devices
- Syscon/regmap infrastructure may not be fully initialized

The MPM driver was designed for newer SoCs where vMPM lives in dedicated shared memory that doesn't overlap with other devices. MSM8660's architecture (vMPM inside RPM region) is incompatible with early irqchip init.

## Recommendation: Option C - Platform Driver Conversion

Convert MPM from early irqchip (IRQCHIP_MATCH) to regular platform driver:

**Advantages:**
- Probes after platform device infrastructure is ready
- Can use syscon/regmap without timing issues  
- Can properly handle -EPROBE_DEFER for RPM dependency

**Disadvantages:**
- Not available during early boot (but MSM8660 doesn't need that)
- Requires refactoring driver from IRQCHIP_MATCH to platform_driver

**Alternative: Option D - MSM8660-Specific Minimal Driver**

Create `drivers/irqchip/irq-msm8660-wakeup.c` as a platform driver:
- Minimal implementation, just enough for cpuidle SPC/PC
- Direct register access (RPM driver can export base + offset)
- No genpd, no hierarchical irqdomain complexity
- MSM8660-specific, not trying to fit generic MPM model

## Next Steps

1. Implement Option A (syscon approach)
2. Test boot with MPM enabled
3. Verify MPM driver probe succeeds
4. Test cpuidle SPC state entry with MPM
5. Document final solution in cavekit

## Related Documents

- **Cavekit:** `context/kits/cavekit-spm-init.md` (PM-1 complete, PM-2 blocked)
- **Implementation:** `context/impl/impl-spm-init.md` (SPM registers working)
- **Reference:** `context/impl/tier3-mpm-sleep-xo.md` (MPM architecture analysis)
- **Build site:** `context/plans/build-site.md` (T-038: MPM integration task)
- **Tracker:** `reports/OUTSTANDING-WORK-TRACKER.md` (PM-2 blocked by MPM)

## Commits

- bfce295a7a15: ARM: dts: tenderloin: Test enabling MPM (FAILED - boot hang at "Zone ranges:")
- dcd05c5f0d78: Revert "ARM: dts: tenderloin: Test enabling MPM"
- 7594ac5e722b: Comment out MPM node, document boot hang issue
- 87ab6e9eee4a: irqchip/qcom-mpm: Add syscon support (PARTIAL - boot progressed but still hangs)
- (uncommitted): Disable MPM again, syscon approach hangs at reserved memory parsing
- 4c8f9ad4a200: Remove broken mpm_sram reserved-memory node
- e1e634e559d7: Revert previous MPM re-enable attempt

## Next Action Required

Either:
1. Convert irq-qcom-mpm.c from IRQCHIP_MATCH to platform_driver (complex, affects all SoCs)
2. Create new drivers/irqchip/irq-msm8660-wakeup.c as platform driver (simpler, MSM8660-specific)

Option 2 is recommended for faster progress on PM-2 (cpuidle deep sleep).
