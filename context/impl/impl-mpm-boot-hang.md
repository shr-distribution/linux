---
domain: mpm-boot-hang
created: "2026-05-15"
last_updated: "2026-05-15"
status: investigating
---

# Implementation: MPM Boot Hang Investigation

## Status: BLOCKED - Early boot hang with MPM node

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

## Recommendation

**Option A (syscon)** is the cleanest mainline-compatible approach:
1. Add "syscon" compatible to RPM node
2. Remove `reg` property from MPM node
3. Add `qcom,rpm-syscon` and `qcom,mpm-offset` properties
4. Modify MPM driver to use regmap instead of direct MMIO

This avoids the resource conflict while maintaining proper abstraction.

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

- bfce295a7a15: ARM: dts: tenderloin: Test enabling MPM (FAILED - boot hang)
- dcd05c5f0d78: Revert "ARM: dts: tenderloin: Test enabling MPM" (this revert)
- 4c8f9ad4a200: Remove broken mpm_sram reserved-memory node
- e1e634e559d7: Revert previous MPM re-enable attempt
