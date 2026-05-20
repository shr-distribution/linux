---
domain: soc-watchdog
created: "2026-05-20"
last_updated: "2026-05-20"
status: investigated-enable-path
---

# Implementation: SoC Hardware Watchdog (MSM8660)

Build site: context/plans/build-site.md

## Status

**T-001 DONE** — investigation concludes the **enable path is viable** with a
minimal DT addition. Decision recorded below. Implementation deferred to
T-008 / T-010 / T-011 / T-021 (Tier 1+).

## R1: Mainline Binding Applicability Investigation

### Candidate driver

- **File:** `drivers/watchdog/qcom-wdt.c`
- **Binding doc:** `Documentation/devicetree/bindings/watchdog/qcom-wdt.yaml`
- **Compatible combinations recognised by the driver `of_match_table`:**
  - `qcom,apss-wdt-ipq5424` → `match_data_ipq5424` (KPSS offsets, pretimeout)
  - `qcom,kpss-timer` → `match_data_apcs_tmr`
  - `qcom,scss-timer` → `match_data_apcs_tmr`
  - `qcom,kpss-wdt` → `match_data_kpss` (KPSS offsets, pretimeout)

The driver also accepts the binding-YAML two-cell form
`qcom,scss-timer + qcom,msm-timer` (verified at
`Documentation/devicetree/bindings/watchdog/qcom-wdt.yaml` lines for
`compatible: oneOf`):

```yaml
- items:
    - const: qcom,scss-timer
    - const: qcom,msm-timer
```

### Register block compatibility (apcs_tmr layout)

`reg_offset_data_apcs_tmr[]` in `drivers/watchdog/qcom-wdt.c`:

| Reg | Offset |
|------|--------|
| WDT_RST | 0x38 |
| WDT_EN | 0x40 |
| WDT_STS | 0x44 |
| WDT_BARK_TIME | 0x4C |
| WDT_BITE_TIME | 0x5C |

Legacy webOS programming
(`/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/arch/arm/mach-msm/msm_watchdog.c`)
defines `WDT0_RST = MSM_TMR0_BASE + 0x38`. `MSM_TMR0_PHYS = 0x02040000`
(`arch/arm/mach-msm/include/mach/msm_iomap-8x60.h:79`). The +0x38 offset
matches the mainline `apcs_tmr` `WDT_RST` exactly. The legacy
`MSM_WATCHDOG=y` driver wrote `1` to that register every pet — same
semantics the mainline driver produces in `qcom_wdt_ping()`.

**Verdict:** Register-block layout is **identical** between the MSM8660
APCS TMR0 watchdog block and the apcs_tmr layout the mainline driver
expects. No driver patch needed.

### Reaching MSM_TMR0_BASE via `cpu-offset`

Current msm8660 timer node (`arch/arm/boot/dts/qcom/qcom-msm8660.dtsi`):

```dts
timer@2000000 {
    compatible = "qcom,scss-timer", "qcom,msm-timer";
    interrupts = <GIC_PPI 0 ...>, <GIC_PPI 1 ...>, <GIC_PPI 2 ...>;
    reg = <0x02000000 0x100>;
    clock-frequency = <27000000>;
    cpu-offset = <0x40000>;
};
```

`qcom_wdt_probe()` reads `cpu-offset` and applies it to the resource
base (`drivers/watchdog/qcom-wdt.c:222-226`):

```c
if (of_property_read_u32(np, "cpu-offset", &percpu_offset))
    percpu_offset = 0;
res->start += percpu_offset;
res->end += percpu_offset;
```

So the watchdog driver, on bind, ioremaps `0x02000000 + 0x40000 = 0x02040000`
— precisely the MSM_TMR0_PHYS the legacy kernel used.

### Missing piece (what blocks driver probe today)

The driver requires a clocks property (`devm_clk_get_enabled(dev, NULL)`,
`drivers/watchdog/qcom-wdt.c:232`). The current MSM8660 timer node carries
`clock-frequency` (used by the timer-qcom clocksource code path) but **no
`clocks` reference**. Without it the watchdog probe returns `-EINVAL`.

apq8064/msm8960 both have it
(`arch/arm/boot/dts/qcom/qcom-apq8064.dtsi:timer@200a000`):

```dts
clocks = <&sleep_clk>;
clock-names = "sleep";
```

`sleep_clk` (32 768 Hz fixed) already exists in tenderloin DT at
`arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi:86`.

### Decision: ENABLE

- Driver: `drivers/watchdog/qcom-wdt.c`
- Binding combo: `qcom,scss-timer` + `qcom,msm-timer` (already on the
  existing timer@2000000 node)
- Action for T-008: add two properties to the existing timer node:
  - `clocks = <&sleep_clk>;`
  - `clock-names = "sleep";`
- Action for T-010 (defconfig): add `CONFIG_QCOM_WDT=y` (or `=m`) to
  `arch/arm/configs/tenderloin_defconfig`,
  `arch/arm/configs/tenderloin_debug_defconfig`,
  `arch/arm/configs/tenderloin_fast_defconfig`. `CONFIG_WATCHDOG=y` is
  already present in mainline defaults but verify in each variant.
- No new compatible, no driver patch, no new DT node.

### Rationale

- Legacy webOS already armed this hardware via `CONFIG_MSM_WATCHDOG=y`
  with the identical register offsets — the silicon is the same APCS
  TMR0 block the mainline driver supports.
- Mainline's binding YAML explicitly lists the `qcom,scss-timer +
  qcom,msm-timer` combo as a watchdog form, so we are inside the
  documented supported surface.
- Per `feedback_replicate_webos.md`: keep legacy behaviour where it
  exists. The mainline driver writes `WDT_RST=1` to ping — identical to
  legacy `writel(1, WDT0_RST)`.
- Risk surface: limited to the `clocks` addition and the defconfig
  flips. If the watchdog mis-fires it can be disabled by removing the
  defconfig flag; no other code path depends on its presence.

## R2/R3/R4 Path Forward

- **R2 (enable):** T-008 wires the DT clock property; `/dev/watchdog0`
  should appear at boot and `wdctl` should report a non-zero timeout
  (mainline default 30 s if `timeout-sec` unspecified — confirmable from
  `drivers/watchdog/qcom-wdt.c:280-285`).
- **R3 (no regression):** T-011 covers 5 cold boots + 30-min idle pet +
  orderly poweroff. Userspace pet path: standard `watchdog` daemon or
  systemd `WatchdogSec=`.
- **R4 (real lock-recovery):** T-021 covers induced lockup → hardware
  reset → pstore-preserved previous boot log.

## Task Tracking

| Task | Status | Notes |
|------|--------|-------|
| T-001 | DONE | enable path verified; driver=qcom-wdt.c, combo=`qcom,scss-timer + qcom,msm-timer`, reg via cpu-offset 0x40000, legacy offset 0x38 matches apcs_tmr.WDT_RST |
| T-008 | TODO | add `clocks=<&sleep_clk>; clock-names = "sleep";` to `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi` timer@2000000 |
| T-010 | TODO | add `CONFIG_QCOM_WDT=y` to three defconfigs |
| T-011 | TODO | 5 cold boots + 30-min pet + orderly poweroff |
| T-021 | TODO | induced lockup → HW reset + pstore preservation |
| T-009 | N/A | won't-fix path not taken |

## Cross-References

- **Kit:** `context/kits/cavekit-soc-watchdog.md`
- **Source traceability:** see kit §Source Traceability
- **Legacy programming:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/arch/arm/mach-msm/msm_watchdog.c`
  (`WDT0_RST = MSM_TMR0_BASE + 0x38`)
- **Mainline driver:** `drivers/watchdog/qcom-wdt.c`
- **Mainline binding:** `Documentation/devicetree/bindings/watchdog/qcom-wdt.yaml`
- **Build site row:** `context/plans/build-site.md` Tier 0 T-001
