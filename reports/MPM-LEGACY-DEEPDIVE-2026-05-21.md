# MPM Deep-Dive: Legacy 2.6.35-palm vs Mainline qcom-mpm

Date: 2026-05-21
Scope: Understand exactly how legacy webOS makes MPM work on MSM8660,
identify the gaps that cause mainline qcom-mpm to hang at boot, and
recommend a concrete path forward to unblock wifi-suspend-wake.

## Legacy Architecture (works on hardware)

**File:** `webos-linux-kernel-touchpad/arch/arm/mach-msm/mpm.c` (594 lines)

### Address Layout

| Constant | Value | Notes |
|----------|-------|-------|
| `MSM_RPM_PHYS` | `0x00104000` | RPM control block physical base |
| `MSM_RPM_BASE` | `0xFA008000` | Statically mapped virtual base (early IOMEM table) |
| `MSM_MPM_REQUEST_REG_START` | `0x9d8` | Offset within RPM — control regs |
| `MSM_MPM_STATUS_REG_START` | `0xdf8` | Offset within RPM — status regs |
| `MSM_MPM_REQUEST_BASE` | `RPM_BASE + 0x9d8` | enable, detect_ctl, polarity, clear |
| `MSM_MPM_STATUS_BASE` | `RPM_BASE + 0xdf8` | pending |
| `MSM_GCC_PHYS` | `0x02082000` | Global Clock Controller (also used for IPC) |
| `MSM_MPM_APPS_IPC` | `GCC + 0x008` | Apps IPC trigger (write BIT(1) to nudge MPM) |
| `MSM_MPM_IPC_IRQ` | GIC SPI 20 (`RPM_SCSS_CPU0_GP_MEDIUM_IRQ`) | MPM-to-CPU wake IRQ |

**Total MPM register footprint:** `0x9d8` to `0xe17` (~0x440 bytes),
spread across two distinct windows inside the 4 KB RPM block.

### Init Sequence

```c
core_initcall(msm_mpm_early_init);   // sets up a2m/m2a IRQ mapping tables
device_initcall(msm_mpm_init);       // request IPC IRQ + set_irq_wake
```

**Critical detail:** legacy uses **late initcalls**, not `IRQCHIP_DECLARE`.
By the time these run, platform devices and IRQ infrastructure are ready.

### Memory Access

```c
static inline uint32_t msm_mpm_read(unsigned int reg, unsigned int subreg_index) {
    unsigned int offset = reg * MSM_MPM_REG_WIDTH + subreg_index;
    return readl(MSM_MPM_STATUS_BASE + offset * 4);
}

static inline void msm_mpm_write(unsigned int reg, unsigned int subreg_index, uint32_t value) {
    unsigned int offset = reg * MSM_MPM_REG_WIDTH + subreg_index;
    writel(value, MSM_MPM_REQUEST_BASE + offset * 4);
}
```

Just plain MMIO via statically-mapped `MSM_RPM_BASE`. **No DT, no
ioremap, no resource-claim conflict with the RPM driver** because both
use the same static mapping (and legacy has no formal "RPM driver"
claiming the region exclusively).

### Wake-Source Trigger (the "set" path)

```c
static void msm_mpm_set(bool wakeset) {
    uint32_t *irqs = wakeset ? msm_mpm_wake_irq : msm_mpm_enabled_irq;
    for (i = 0; i < MSM_MPM_REG_WIDTH; i++) {
        msm_mpm_write(REG_ENABLE,     i, irqs[i]);
        msm_mpm_write(REG_DETECT_CTL, i, msm_mpm_detect_ctl[i]);
        msm_mpm_write(REG_POLARITY,   i, msm_mpm_polarity[i]);
        msm_mpm_write(REG_CLEAR,      i, 0xffffffff);
    }
    msm_mpm_write_barrier();
    msm_mpm_send_interrupt();   // writel(BIT(1), GCC+0x008)
}
```

Called from `msm_mpm_enter_sleep(true/false)` — sets the active wake
mask, then writes to GCC IPC register to wake MPM and have it latch
the new configuration.

### Public API (used by board file)

```c
int msm_mpm_enable_pin(enum msm_mpm_pin pin, unsigned int enable);
int msm_mpm_set_pin_wake(enum msm_mpm_pin pin, unsigned int on);
int msm_mpm_set_pin_type(enum msm_mpm_pin pin, unsigned int flow_type);
int msm_mpm_set_irq_wake(unsigned int irq, unsigned int on);
void msm_mpm_enter_sleep(bool from_idle);
void msm_mpm_exit_sleep(bool from_idle);

enum msm_mpm_pin {
    MSM_MPM_PIN_SDC3_DAT1 = 21,
    MSM_MPM_PIN_SDC3_DAT3 = 22,
    MSM_MPM_PIN_SDC4_DAT1 = 23,   // <-- what we need for WiFi wake
    MSM_MPM_PIN_SDC4_DAT3 = 24,
};
```

### Board-file consumer (tenderloin)

```c
static int msm_sdcc_cfg_mpm_sdiowakeup(struct device *dev, unsigned mode) {
    pdev = container_of(dev, struct platform_device, dev);
    if (pdev->id == 4)
        pin = MSM_MPM_PIN_SDC4_DAT1;
    else
        return -EINVAL;

    switch (mode) {
    case SDC_DAT1_DISABLE: msm_mpm_enable_pin(pin, 0); break;
    case SDC_DAT1_ENABLE:
        msm_mpm_set_pin_type(pin, IRQ_TYPE_LEVEL_LOW);
        msm_mpm_enable_pin(pin, 1);
        break;
    case SDC_DAT1_ENWAKE: ...
    }
}

static struct mmc_platform_data tenderloin_sdc4_data = {
    ...
    .cfg_mpm_sdiowakeup = msm_sdcc_cfg_mpm_sdiowakeup,
};
```

This is the entire "WiFi wake-from-suspend" mechanism on legacy. SDC4
driver calls `cfg_mpm_sdiowakeup(...)` during suspend prep; MPM marks
SDC4_DAT1 as wake source; SoC enters PC; SDIO activity wakes MPM →
MPM raises IPC IRQ → CPU wakes.

## Mainline qcom-mpm Architecture (designed for newer SoCs)

**File:** `drivers/irqchip/irq-qcom-mpm.c`

### Init Mechanism

```c
IRQCHIP_MATCH("qcom,mpm", qcom_mpm_init)
IRQCHIP_MATCH("qcom,msm8660-mpm", qcom_mpm_init)
```

`IRQCHIP_MATCH` runs the init callback during `of_irq_init()`, which
is called from `init_IRQ()` — **very early** in `setup_arch()`, before
platform devices exist.

In the callback:

```c
static int qcom_mpm_init(struct device_node *np, struct device_node *parent) {
    struct platform_device *pdev = of_find_device_by_node(np);
    if (!pdev)
        return -EPROBE_DEFER;
    ...
}
```

This **always returns -EPROBE_DEFER on MSM8660** at this point — the
platform device for the MPM node hasn't been created by
`of_platform_populate()` yet (that runs as a `arch_initcall`, much
later). `of_irq_init()` retries deferred nodes a few times then gives
up.

### DT Binding (designed for SoCs like sm6375)

```dts
mpm: interrupt-controller {
    compatible = "qcom,mpm";
    qcom,rpm-msg-ram = <&apss_mpm>;     // phandle to mmio-sram sub-partition
    mboxes = <&ipcc IPCC_CLIENT_AOP IPCC_MPROC_SIGNAL_SMP2P>;
    interrupts = <GIC_SPI 197 IRQ_TYPE_EDGE_RISING>;
    qcom,mpm-pin-count = <96>;
    qcom,mpm-pin-map = <5 296>, ...;
};

rpm_msg_ram: sram@45f0000 {
    compatible = "qcom,rpm-msg-ram", "mmio-sram";
    reg = <0x045f0000 0x7000>;
    apss_mpm: sram@1b8 { reg = <0x1b8 0x48>; };
};
```

Notice:
- The MPM node has **no `reg`** property — it references a sub-region of an external mmio-sram node.
- Wake notification uses `mboxes` (IPCC — InterProcessor Communication Controller).
- The `rpm_msg_ram` is a SEPARATE region from the RPM control block.

### What Mainline Assumes (and MSM8660 violates)

| Mainline assumption | MSM8660 reality |
|--------------------|-----------------|
| MPM has its own dedicated SRAM region | MPM regs live INSIDE the RPM control block |
| Wake IPC via mailbox controller (IPCC) | Wake IPC via raw GCC MMIO write |
| Platform infrastructure ready at init | IRQCHIP_DECLARE runs before platform_devices exist |
| One contiguous register window | Two disjoint windows (request@0x9d8 + status@0xdf8) |
| RPM driver doesn't claim MPM region | Mainline RPM driver claims full 0x104000 + 0x1000 |

## Why All Three Prior Attempts Failed

### Attempt 1: reserved-memory with no-map (be070d0aa35f-equiv, reverted)

```dts
mpm_sram: mpm@1049d8 {
    compatible = "qcom,msm-imem";
    reg = <0x001049d8 0x448>;
    no-map;
};
```

**Why it hangs:** The `/reserved-memory` framework is designed for
DRAM regions, not MMIO. It uses `memblock_remove()` / `memblock_reserve()`
which operate on DRAM banks. Pointing it at MMIO (`0x1049d8` is in
the device address space, not DRAM) confuses the early-boot memory
parser → hang.

### Attempt 2: Direct reg property (bfce295a7a15, reverted)

```dts
mpm: interrupt-controller@1049d8 {
    compatible = "qcom,msm8660-mpm", "qcom,mpm";
    reg = <0x001049d8 0x448>;
    ...
};
```

**Why it hangs:** This region OVERLAPS with the RPM node's
`reg = <0x00104000 0x1000>`. The early DT resource-reservation phase
notices the conflict and hangs. Even adjusting the driver to use
non-exclusive `devm_ioremap()` doesn't help — the conflict is in
**DT parsing**, before drivers load.

### Attempt 3: Syscon approach (87ab6e9eee4a, disabled)

```dts
rpm: rpm@104000 {
    compatible = "qcom,rpm-msm8660", "syscon";
    ...
};
mpm: interrupt-controller {
    compatible = "qcom,msm8660-mpm", "qcom,mpm";
    qcom,rpm-syscon = <&rpm>;
    qcom,mpm-offset = <0x9d8>;
    ...
};
```

**Why it hangs:** Boot got past "Zone ranges:" (so the DT overlap is
gone — syscon parent owns the region cleanly) BUT then hangs after
the smem@40000000 reserved-memory line. Root cause per the impl
doc: `IRQCHIP_MATCH` runs `qcom_mpm_init()` → calls
`of_find_device_by_node()` → returns NULL (no platform device yet) →
returns `-EPROBE_DEFER` → `of_irq_init()` retries → still NULL →
eventually gives up, but in the process tries to acquire locks /
resources that are in an undefined state → silent hang. (The impl
doc's claim that the hang is in "reserved memory parsing" is the
last log line printed, NOT the actual hang location.)

## Concrete Path Forward — Recommendation

**Write a small MSM8660-specific platform driver that replicates the
legacy mechanism, ignoring the mainline qcom-mpm machinery.**

### Why this works

1. **Platform driver = late init** — matches legacy's
   `core_initcall`/`device_initcall` timing. Platform devices exist
   by the time `module_platform_driver` probes.
2. **No DT resource conflict** — get the MPM region via a syscon
   phandle to the RPM block (or via a `qcom,rpm-msg-ram` style
   mmio-sram phandle if we add one).
3. **Match legacy semantics exactly** — wake-source enable, IPC IRQ
   request, GCC+0x008 IPC trigger.
4. **No genpd, no hierarchical irqdomain** — wifi-suspend-wake R1-R4
   only need wake-source registration + IPC IRQ wake propagation,
   not the full mainline MPM model.

### Sketch of Implementation

**New driver:** `drivers/soc/qcom/msm8660-mpm.c` (NOT under
`drivers/irqchip/` — this is a wake-source helper, not a top-level
irqchip).

```c
struct msm8660_mpm {
    struct device *dev;
    struct regmap *rpm_regmap;      // syscon to RPM block (or msg-ram)
    u32 request_offset;             // 0x9d8
    u32 status_offset;              // 0xdf8
    void __iomem *gcc_ipc;          // ioremap of GCC+0x008
    int ipc_irq;                    // GIC SPI 20

    u32 enabled_mask[2];            // 64-bit pin mask
    u32 wake_mask[2];
    u32 detect_ctl[2];
    u32 polarity[2];
    raw_spinlock_t lock;
};

static int msm8660_mpm_probe(struct platform_device *pdev) {
    // get regmap via syscon phandle
    mpm->rpm_regmap = syscon_regmap_lookup_by_phandle(np, "qcom,rpm-syscon");
    of_property_read_u32(np, "qcom,mpm-request-offset", &mpm->request_offset);
    of_property_read_u32(np, "qcom,mpm-status-offset", &mpm->status_offset);

    // ioremap GCC IPC register (a single u32 inside GCC block)
    mpm->gcc_ipc = devm_ioremap(dev, 0x02082008, 4);

    // request the IPC IRQ
    mpm->ipc_irq = platform_get_irq(pdev, 0);
    devm_request_irq(dev, mpm->ipc_irq, msm8660_mpm_irq, IRQF_TRIGGER_RISING,
                     "mpm", mpm);
    enable_irq_wake(mpm->ipc_irq);

    // expose public API to other drivers (or via a small irq_chip facade)
    msm8660_mpm_ptr = mpm;
    return 0;
}

// Public API consumed by SDC4 driver
int msm8660_mpm_set_pin_wake(unsigned int pin, bool on);
int msm8660_mpm_set_pin_type(unsigned int pin, unsigned int flow_type);
int msm8660_mpm_enable_pin(unsigned int pin, bool enable);
void msm8660_mpm_enter_sleep(bool from_idle);
void msm8660_mpm_exit_sleep(bool from_idle);
```

### DT Binding

```dts
&rpm {
    compatible = "qcom,rpm-msm8660", "syscon";   // add "syscon"
    /* reg unchanged: <0x00104000 0x1000> */
};

soc {
    msm8660_mpm: msm8660-mpm {
        compatible = "qcom,msm8660-mpm";
        qcom,rpm-syscon = <&rpm>;
        qcom,mpm-request-offset = <0x9d8>;
        qcom,mpm-status-offset = <0xdf8>;
        interrupts = <GIC_SPI 20 IRQ_TYPE_EDGE_RISING>;
        interrupt-parent = <&intc>;
        qcom,mpm-pin-count = <64>;
        qcom,mpm-pin-map = <23 SDC4_IRQ_NUM>,  /* SDC4 DAT1 */
                           <24 SDC4_IRQ_NUM>;
    };
};
```

No `reg` property on the MPM node itself → no DT resource conflict.
RPM keeps its `reg` and adds `syscon` compatible so it can be looked
up by the MPM driver.

### Integration With SDC4

In `qcom-apq8060-tenderloin-common.dtsi`, add a phandle from SDC4 to
the MPM:

```dts
&sdcc4 {
    qcom,mpm-wake = <&msm8660_mpm 23>;   // pin 23 = SDC4_DAT1
    wakeup-source;
};
```

The mmci driver (or a small shim) reads the phandle and calls
`msm8660_mpm_*` API at suspend/resume time, exactly like legacy
`cfg_mpm_sdiowakeup`.

## Estimated Effort

| Step | Effort |
|------|--------|
| Write `drivers/soc/qcom/msm8660-mpm.c` | 1-2 hr |
| Write `Documentation/devicetree/bindings/soc/qcom/qcom,msm8660-mpm.yaml` | 0.5 hr |
| Add DT node + RPM `syscon` compatible | 0.25 hr |
| Add `Kconfig` symbol + `Makefile` entry + defconfig | 0.25 hr |
| Wire SDC4 consumer (mmci shim or direct API call) | 0.5 hr |
| On-device test: boot, suspend, ping-wake | 1 hr |

**Total: ~3-5 hr of focused work.**

## Why Not Mainline qcom-mpm (any of the three branches)

- **Direct reg:** dies in DT overlap.
- **reserved-memory:** misuse of framework.
- **syscon:** stuck on IRQCHIP_DECLARE timing — the driver would need
  fundamental refactoring to support late-bind, which affects ALL
  SoCs using qcom-mpm. Out of scope for our needs.

## What This Unblocks

- **wifi-suspend-wake R1**: MPM is functional as wakeup-interrupt
  controller (our new driver IS the MPM functional path).
- **wifi-suspend-wake R2**: SDC4 DAT1 wake source registered via
  `qcom,mpm-wake` phandle.
- **spm-init PM-2**: cpuidle deep sleep (Power Collapse) needs MPM
  for wake-IRQ delivery during CPU shutdown. Same mechanism.

## Open Questions

1. **`qcom,rpm-syscon` is a non-standard binding** — should we instead
   use the existing `qcom,rpm-msg-ram` style with mmio-sram? Issue:
   the RPM block on MSM8660 is a CONTROL block, not message RAM; the
   `mmio-sram` driver may not like binding to it. Investigate
   feasibility before committing to syscon.
2. **Locking with RPM driver** — when the RPM driver writes its own
   control regs and our MPM driver writes MPM regs (both inside the
   same 4 KB block), do they need to serialize? Legacy doesn't (no
   formal RPM driver to coordinate with). Mainline RPM driver might.
3. **Whether SDC4 (mmci-pl18x) needs a wakeup hook patch** or whether
   the wake-source machinery alone (via `wakeup-source` DT property
   and `device_init_wakeup`) is enough.

## Related Files

- **Legacy reference:** `webos-linux-kernel-touchpad/arch/arm/mach-msm/mpm.c`
- **Legacy header:** `webos-linux-kernel-touchpad/arch/arm/mach-msm/mpm.h`
- **Legacy consumer:** `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:5417`
- **Mainline driver:** `drivers/irqchip/irq-qcom-mpm.c`
- **Prior investigation:** `context/impl/impl-mpm-boot-hang.md`
- **Kit:** `context/kits/cavekit-wifi-suspend-wake.md`
- **Mainline DT reference (different design):** `arch/arm64/boot/dts/qcom/sm6375.dtsi`
