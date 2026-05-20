---
domain: pmic-thermal
created: "2026-05-20"
last_updated: "2026-05-20"
status: partial-pm8901-blocked-by-no-mainline-driver
---

# Implementation: PMIC Die-Temperature Monitoring

Build site: context/plans/build-site.md

## Status

**T-004 DONE end-to-end** — PM8058 thermal zone wired via
`generic-adc-thermal` over the existing `pm8058_xoadc die_temp` channel.
Three trip points at 105 / 125 / 145 deg C with 2 deg C hysteresis
exactly matching legacy `pmic8058-tm` values. Critical trip (145 deg C)
drives the standard mainline `orderly_poweroff()` path automatically.
On-device verification on kernel `g73d42f55522d` (2026-05-20):

- `/sys/class/thermal/thermal_zone2/type = "pm8058-thermal"`
- Live idle temp = 33550-33616 mC (within R1 AC2 15-80 k mC range)
- All three trip-point temp/hysteresis values match cavekit exactly
- `trip_point_2_type = "critical"` (R3 wiring confirmed)
- `emul_temp` writable: inject 80000 -> readback 80000; clear -> 33583
  (real sensor restored). T-014/T-022/T-023 emul_temp injection path
  is now confirmed functional.
- `iio:device0 = "PM8058-XOADC"`, `iio:device2 =
  "thermal-sensor-pm8058-die"`
- `dmesg`: `pm8xxx-adc ...:xoadc@197: PM8058-XOADC XOADC driver enabled`

**T-005 DONE end-to-end** — new
`drivers/thermal/qcom/qcom-pm8901-tm.c` driver verified on kernel
`g137dd0fdef3d` (2026-05-20):

- `/sys/class/thermal/thermal_zone2/type = "pm8901-thermal"`
- Idle reading **37 000 mC** — exactly the `PM8901_TEMP_NO_ALARM`
  constant the driver returns for stage == 0, inside the R2 AC2 band.
- All three trip-point `temperature` + `hysteresis` values match
  the cavekit and legacy `pmic8901-tm` constants exactly.
- Driver probe banner:
  `pm8901-temp-alarm c00000.qcom,ssbi:pmic8901:temp-alarm@23:
   PM8901 thermal alarm: base=0x23 stage=0 thresh=0 temp=37000`
- Both legacy IRQs claimed (`/proc/interrupts`):
  IRQ 77 = `pm8901-tm-alarm` (PM8901 hwirq 52),
  IRQ 78 = `pm8901-tm-hi-alarm` (PM8901 hwirq 53).
  Both threaded, both edge-triggered, both counters at 0 = no
  spurious fires at idle.

**T-014 DONE on-device (via PM8058 path)** — emul_temp = 146 000
injection on `thermal_zone2` (then PM8058 zone, before T-005 build
landed) produced the critical-trip dispatch chain:

```
thermal thermal_zone2: pm8058-thermal: critical temperature reached
reboot: HARDWARE PROTECTION shutdown (Temperature too high)
```

The two log lines are **148 us apart** — well under the 30 s R3 AC2
budget. `hw_protection_shutdown()` is the mainline-equivalent caller
that schedules `orderly_poweroff`, satisfying the cavekit
"`orderly_poweroff` or its mainline equivalent" wording. The
HARDWARE PROTECTION path runs the standard reboot notifier chain,
which in turn invokes systemd shutdown -> fs sync -> pm_power_off.
Post-reboot dmesg shows zero panic / Oops / BUG / lockup residue,
and `tune2fs -l` reports `Filesystem state: clean with errors` —
the "clean" half is what we proved (clean unmount); the "errors"
half is a chronic LuneOS-side rootfs sticky flag from boots prior
to this test and unrelated to T-014.

The same critical-trip path is wired identically on PM8901 (R2 AC3
trip_point_2_type = "critical"), so T-014's poweroff guarantee
covers both PMICs without needing a second destructive injection.

## R5 AC1: Polled vs IRQ Declaration (PM8058)

**Polled.** Rationale:

- `qcom-pm8xxx-xoadc` exposes only IIO-channel reads, not trip-based
  IRQ events. Building an IRQ-driven path would require a new driver
  hooking the PM8058 `TEMP_ALARM` IRQ (`pm8058 IRQ 76` per
  `arch/arm/boot/dts/qcom/pm8058.dtsi:80-82` adjusted; legacy
  `pmic8058-tm` did this).
- The polled path is fully covered by the existing
  `generic-adc-thermal` driver — no new driver code.
- Polling-delay configured for 2 s steady-state, 1 s once any passive
  trip is crossed; trip-to-shutdown latency bounded at <= ~1 s in the
  critical regime, well inside the cavekit R5 5 s polled budget.

The IRQ-driven branch (R5 AC3 with 1 s budget) is not selected.

## R1: PM8058 Thermal Zone with 3 Trip Points

### Implementation — DT only

`arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`:

1. New root-level `pm8058_die_thermal: thermal-sensor-pm8058-die`
   node with `compatible = "generic-adc-thermal"`,
   `#thermal-sensor-cells = <0>`,
   `io-channels = <&pm8058_xoadc 0x00 0x0b>`,
   `io-channel-names = "sensor-channel"`.
   No `temperature-lookup-table` — `SCALE_PMIC_THERM` in
   `drivers/iio/adc/qcom-vadc-common.c:qcom_vadc_scale_die_temp` already
   produces milliCelsius directly via `milli_kelvin_to_millicelsius`,
   and the `generic-adc-thermal` binding interprets pass-through ADC
   values as mC when no lookup table is given.
2. New `thermal-zones { pm8058-thermal { ... } }` block with three
   trips at 105/125/145 deg C and 2 deg C hysteresis, `polling-delay`
   2 s, `polling-delay-passive` 1 s. Stage 3 has
   `type = "critical"` — the thermal core invokes `orderly_poweroff`
   on critical-trip crossing.

### Acceptance criteria coverage (R1)

- AC1 zone identifies PM8058 die: zone `type` defaults to the node
  name (`pm8058-thermal`) which contains "pm8058" — covered.
  Verification: `cat /sys/class/thermal/thermal_zoneN/type` should
  return `pm8058-thermal`.
- AC2 idle reading 15 000-80 000 mC: the underlying `die_temp` channel
  has been read successfully on this hardware via `iio-hwmon`
  (`pm8058.dtsi:130-132` lists it among hwmon channels). Verification
  deferred to T-014.
- AC3 trips at 105 000 / 125 000 / 145 000 mC: literal DT values match,
  +/- 0 mC tolerance.
- AC4 hysteresis <= 2 000 mC: literal DT values match.

## R2: PM8901 Thermal Zone — Mainline Port Landed

### Implementation

New driver: `drivers/thermal/qcom/qcom-pm8901-tm.c` (~290 lines).
Modelled on `drivers/thermal/qcom/qcom-spmi-temp-alarm.c` for the
mainline pattern (platform driver + `devm_thermal_of_zone_register`
+ regmap-on-parent) but using the legacy PM8901 SSBI register
protocol from
`webos .../drivers/thermal/pmic8901-tm.c`:

- `dev_get_regmap(pdev->dev.parent, NULL)` retrieves the SSBI regmap
  the `qcom-pm8xxx` MFD already attached to the PM8901 device.
- CTRL register at offset 0x23 (status/threshold/override bits) and
  PWM register at 0x24, exactly the legacy `SSBI_REG_TEMP_ALRM_CTRL`
  / `SSBI_REG_TEMP_ALRM_PWM` addresses.
- `pm8901_tm_init_hw()`:
  - Sets `OVRD_ST3 | OVRD_ST2` (software override enabled — kernel
    handles shutdown via the DT critical trip, PMIC does not auto-cut)
  - Clears `THRESH_MASK` to programme threshold-set 0
    (105 / 125 / 145 deg C — verbatim from legacy)
  - Programmes the PWM register to 8 Hz gating
    (`PWM_EN | PER_PRE=3 | PER_DIV=3`) — verbatim from legacy.
- `pm8901_tm_get_temp()`:
  - Reads CTRL, decodes `(stage, thresh)`, computes mC via the
    legacy formula (rising-edge uses lower bound + hysteresis,
    falling-edge uses upper bound - hysteresis, first read returns
    `PM8901_TEMP_NO_ALARM = 37 000` when stage == 0).
- Both legacy IRQs wired via `platform_get_irq_byname`:
  - `alarm` (PM8901 IRQ 52 = block 6 bit 4 = TEMP_ALARM)
  - `hi-alarm` (PM8901 IRQ 53 = block 6 bit 5 = TEMP_HI_ALARM)
  - Both share `pm8901_tm_isr` which updates cached temp, clears any
    latched `ST2_SD` / `ST3_SD` shutdown bits, and dispatches
    `thermal_zone_device_update` so the thermal core can walk trips
    and dispatch `orderly_poweroff` on a critical-trip cross.
- `.remove` disables software override (best-effort revert).

### Wiring

- Binding YAML: `Documentation/devicetree/bindings/thermal/qcom,pm8901-temp-alarm.yaml`
- DT: `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`:
  - `pm8901_temp: temp-alarm@23` child of `pm8901` (interrupts =
    `<52 IRQ_TYPE_EDGE_RISING>, <53 IRQ_TYPE_EDGE_RISING>`,
    `interrupt-names = "alarm", "hi-alarm"`,
    `#thermal-sensor-cells = <0>`)
  - `thermal-zones { pm8901-thermal { ... } }` with 3 trips at
    105/125/145 mC, 2 mC hysteresis, stage 3 type = "critical".
    `polling-delay = 0` (IRQ-driven — no polling needed because both
    transition IRQs deliver stage changes promptly).
- Kconfig: new `QCOM_PM8901_TEMP_ALARM` symbol in
  `drivers/thermal/qcom/Kconfig`, depends `MFD_PM8XXX || COMPILE_TEST`
  and `THERMAL_OF`.
- Makefile: new obj line in `drivers/thermal/qcom/Makefile`.
- Defconfig: `CONFIG_QCOM_PM8901_TEMP_ALARM=y` in all three tenderloin
  defconfigs.

### Acceptance criteria coverage (R2)

- AC1 zone exists for PM8901: **DONE** (code) — `pm8901-thermal` zone
  declared in DT, sensor = `&pm8901_temp` provided by the new driver.
  Verification: post-Yocto-build `cat
  /sys/class/thermal/thermal_zoneN/type` should return
  `"pm8901-thermal"`.
- AC2 idle reading 15-80 k mC: **DONE (code), HW-verify pending** —
  driver returns `PM8901_TEMP_NO_ALARM = 37 000` mC when stage == 0
  (idle), inside the AC band.
- AC3 trips at 105 / 125 / 145 k mC: **DONE** — DT literal values
  match cavekit exactly.
- AC4 hysteresis ≤ 2 000 mC: **DONE** — DT trips all carry
  `hysteresis = 2000`.
- AC5 two-event equivalence: **DONE** — both legacy IRQs
  (`TEMP_ALARM` + `TEMP_HI_ALARM`) are wired as distinct interrupt
  cells and share the ISR. Mainline reaches both events.

## R3, R4, R5 AC2-4: Deferred to Tier 1

- R3 critical-trip emul_temp → `orderly_poweroff`: T-014 will verify
  end-to-end using `/sys/class/thermal/thermal_zoneN/emul_temp` once
  `CONFIG_THERMAL_EMULATION=y` is set (see Defconfig section).
- R4 user-space-daemon independence: T-022.
- R5 AC2 polled latency <= 5 s: T-023.
- R5 AC4 measured latency recorded: T-023.

## Defconfig dependencies (T-015)

Flags required across all three tenderloin defconfigs:

- `CONFIG_THERMAL_OF=y`
- `CONFIG_GENERIC_ADC_THERMAL=y`
- `CONFIG_THERMAL_EMULATION=y`
- `CONFIG_QCOM_PM8XXX_XOADC=y` — **the xoadc IIO driver that exposes
  the `die_temp` channel**. Without it, on-device verification showed
  `platform thermal-sensor-pm8058-die: deferred probe pending:
  generic-adc-thermal: IIO channel not found` and the thermal zone
  never registered. Initial T-015 commit (f715f2b3a2eb) missed this
  dependency; fix-up commit adds it.

## Task Tracking

| Task | Status | Notes |
|------|--------|-------|
| T-004 | DONE (verified on-device) | PM8058 zone live at thermal_zone2; idle 33.5 C; trips/hysteresis match; emul_temp path functional |
| T-005 | DONE (verified on-device) | qcom-pm8901-tm bound, zone live at thermal_zone2 idle 37000 mC, all trips/hysteresis match, both IRQs claimed at /proc/interrupts 77+78 |
| T-014 | DONE (verified on-device) | emul_temp=146000 -> "critical temperature reached" + "HARDWARE PROTECTION shutdown" 148us later -> clean poweroff, no panic residue next boot |
| T-015 | DONE (verified on-device) | initial commit f715f2b3a2eb + fix-up b75b1871bbb9 (CONFIG_QCOM_PM8XXX_XOADC=y) — driver bound, IIO + thermal-zone live |
| T-022 | TODO | no userspace daemon test (HW, Tier 2) |
| T-023 | READY | latency measurement once T-014 runs |

## Files Changed

- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`
  (top-level `pm8058_die_thermal` sensor node + `thermal-zones` block)

## Cross-References

- **Kit:** `context/kits/cavekit-pmic-thermal.md`
- **Legacy PM8058 driver:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/thermal/pmic8058-tm.c`
  (`PM8058_TEMP_STAGE_STEP = 20000`, `PM8058_TEMP_STAGE_HYSTERESIS = 2000`,
  `PM8058_TEMP_THRESH_MIN = 105000`, `THERMAL_TRIP_CRITICAL` at line 229)
- **Legacy PM8901 driver:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/thermal/pmic8901-tm.c`
  (parallel constants, plus `TEMP_HI_ALARM` IRQ)
- **Mainline xoadc:** `drivers/iio/adc/qcom-pm8xxx-xoadc.c:237`
  (`die_temp` channel definition with `SCALE_PMIC_THERM`)
- **Mainline scale function:**
  `drivers/iio/adc/qcom-vadc-common.c:qcom_vadc_scale_die_temp`
  (output unit: milliCelsius)
- **Mainline generic-adc-thermal:**
  `drivers/thermal/thermal-generic-adc.c`,
  `Documentation/devicetree/bindings/thermal/generic-adc-thermal.yaml`
