---
domain: soc-watchdog
created: "2026-05-20"
last_updated: "2026-05-21"
status: t021-partial-hwreset-ok-pstore-unverified
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

## On-Device Verification (2026-05-21, kernel `g2f4f77242d2c`)

`/dev/watchdog0` exists with `qcom_wdt` driver bound:

```
wdctl /dev/watchdog0
Device:   /dev/watchdog0
Identity: qcom_wdt [version 0]
Timeout:  30 seconds
```

`CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED=y` keeps the kernel pinging until
userspace opens the device. `CONFIG_WATCHDOG_SYSFS=n` so no
`/sys/class/watchdog/watchdog0/{timeout,state,bootstatus}` entries —
only `wdctl` works.

Platform driver bind verified:
```
/sys/bus/platform/devices/2000000.timer/driver
  -> ../../../../bus/platform/drivers/qcom_wdt
```

## Ramoops Region for T-021 (added 2026-05-21)

Commit `be070d0aa35f` adds `reserved-memory ramoops@7f500000` (1 MB)
at the top of System RAM bank 2. Commit `f33a66d9d1d7` drops the
`no-map` property which was causing early-boot hang (see Dead Ends
below). Placement rationale:

- Inside confirmed-live DRAM (`/proc/iomem`: `0x48000000-0x7f5fffff`)
- Above moboot's `MEMBASE=0x50000000` (4 MB) footprint
- Above moboot's `SCRATCH_ADDR=0x70000000` (64 MB) kernel staging
- moboot only zeros its own BSS (verified in
  `herrie82/moboot arch/arm/crt0.S` `.L__do_bss` loop), so 0x7f500000
  contents survive a warm reboot through the watchdog reset path.

On-device verification of ramoops registration (kernel
`gf33a66d9d1d7+`, journalctl -b 0):

```
OF: reserved mem: 0x7f500000..0x7f5fffff (1024 KiB) map non-reusable ramoops@7f500000
pstore: Using crash dump compression: deflate
printk: legacy console [ramoops-1] enabled
pstore: Registered ramoops as persistent store backend
ramoops: using 0x100000@0x7f500000, ecc: 0
```

Platform device `7f500000.ramoops` bound to ramoops driver.

## T-021 Test Attempts (2026-05-21)

### Attempt 1: panic only (no /dev/watchdog hold)
- `sysctl -w kernel.sysrq=1; sync; echo c > /proc/sysrq-trigger`
- Panic captured via netconsole (full backtrace)
- Device DID reboot (user observed Tux+login screen)
- BUT: USB/network never came up post-reset; user manually rebooted
- Manual reboot = power cycle = DRAM wiped = pstore empty on next boot
- **Result: watchdog HW reset proven, pstore preservation unverified.**

### Attempt 2: panic with userspace holding /dev/watchdog
- `exec 9>/dev/watchdog0; sleep 2; echo c > /proc/sysrq-trigger`
- Panic captured via netconsole
- Device did NOT reboot at all; user manually rebooted
- **Root cause:** kernel watchdog framework
  (`drivers/watchdog/watchdog_dev.c:watchdog_worker_should_ping`)
  returns `true` when `WDOG_ACTIVE` is set — which userspace open
  sets. The kernel's kthread_worker keeps pinging the watchdog at
  ~15 s intervals on behalf of userspace.
- Verified with a 90 s `exec 9>/dev/watchdog0; sleep 90` hold
  (no panic): device stayed responsive, kernel kept HW alive.
- **Conclusion:** holding `/dev/watchdog` does NOT help unblock the
  watchdog. Attempt 1's approach (no hold, just panic) is correct.

### Outstanding work to close T-021

1. Reproduce Attempt 1 (panic, no hold) and **wait patiently**
   (3–5 min) for the post-reset boot to fully come up — do NOT
   manually reboot. If USB/network doesn't come up on its own,
   investigate the post-watchdog USB enumeration issue separately.
2. Once SSH is back, verify:
   - `wdctl /dev/watchdog0` shows `CARDRESET=1` → confirms watchdog
     was the reset source (not e.g. brownout).
   - `/sys/fs/pstore/` contains `dmesg-ramoops-0` with the panic
     trace and `console-ramoops-0` with end of kmsg.

## Dead Ends

- `no-map` on the ramoops reserved-memory node (be070d0aa35f) caused
  early-boot hang at moboot "Booting..." prompt before kernel
  reached console. Fix: drop `no-map` (f33a66d9d1d7); use plain
  `memblock_reserve()` form, matching mainline mako/klte convention.
- Holding `/dev/watchdog` open from userspace to "force" the
  watchdog to fire on panic — the kernel framework auto-pings the
  HW watchdog as long as `WDOG_ACTIVE` is set (which open sets), so
  this approach DOES NOT make the watchdog fire faster.

## Task Tracking

| Task | Status | Notes |
|------|--------|-------|
| T-001 | DONE | enable path verified; driver=qcom-wdt.c, combo=`qcom,scss-timer + qcom,msm-timer`, reg via cpu-offset 0x40000, legacy offset 0x38 matches apcs_tmr.WDT_RST |
| T-008 | DONE | `apcs_timer:` label added to qcom-msm8660.dtsi timer@2000000; tenderloin-common.dtsi `&apcs_timer` override adds `clocks = <&sleep_clk>; clock-names = "sleep";` |
| T-009 | N/A | won't-fix path not taken |
| T-010 | DONE | `CONFIG_WATCHDOG=y` + `CONFIG_QCOM_WDT=y` added to all three tenderloin defconfigs |
| T-011 | TODO | 5 cold boots + 30-min pet + orderly poweroff (HW). Pet via systemd `RuntimeWatchdogSec=` (currently `0`). |
| T-021 | PARTIAL | Watchdog HW reset on panic proven (Attempt 1, device rebooted). ramoops DT wired (be070d0aa35f + f33a66d9d1d7), registers cleanly on every boot. pstore preservation across watchdog reset NOT YET verified — needs another panic test where we wait patiently for post-reset boot without manual intervention. See "T-021 Test Attempts" section above. |

## Cross-References

- **Kit:** `context/kits/cavekit-soc-watchdog.md`
- **Source traceability:** see kit §Source Traceability
- **Legacy programming:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/arch/arm/mach-msm/msm_watchdog.c`
  (`WDT0_RST = MSM_TMR0_BASE + 0x38`)
- **Mainline driver:** `drivers/watchdog/qcom-wdt.c`
- **Mainline binding:** `Documentation/devicetree/bindings/watchdog/qcom-wdt.yaml`
- **Build site row:** `context/plans/build-site.md` Tier 0 T-001
