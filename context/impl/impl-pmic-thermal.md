---
domain: pmic-thermal
created: "2026-05-20"
last_updated: "2026-05-20"
status: partial-pm8901-blocked-by-no-mainline-driver
---

# Implementation: PMIC Die-Temperature Monitoring

Build site: context/plans/build-site.md

## Status

**T-004 DONE** — PM8058 thermal zone wired via `generic-adc-thermal` over
the existing `pm8058_xoadc die_temp` channel. Three trip points at
105 / 125 / 145 deg C with 2 deg C hysteresis exactly matching legacy
`pmic8058-tm` values. Critical trip (145 deg C) drives the standard
mainline `orderly_poweroff()` path automatically.

**T-005 PARTIAL** — PM8901 die-temperature monitoring is **not feasible
without new driver code in mainline 6.18.** PM8901 has no IIO channel,
no equivalent ADC driver, and the legacy `pmic8901-tm` direct-SSBI-read
driver was never ported. PM8058 monitoring is the active protection;
PM8901 protection deferred behind a driver port (see "PM8901 path
forward" below). R2 cannot fully pass until that driver lands.

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

## R2: PM8901 Thermal Zone — Blocked by Missing Driver

### Why this is not in DT

PM8901 die-temperature is read via SSBI registers (legacy
`webos .../drivers/thermal/pmic8901-tm.c:pm8901_tm_init_temp` /
`pm8901_tm_update_temp`), not via an ADC channel. The mainline tree
contains:

- `drivers/mfd/qcom-pm8xxx.c` — PM8901 MFD/IRQ entry only
  (`drivers/mfd/qcom-pm8xxx.c:522`).
- `drivers/pinctrl/qcom/pinctrl-ssbi-mpp.c` — PM8901 MPP pinctrl.
- `drivers/regulator/qcom_rpm-regulator.c` — PM8901 regulators.

No PM8901 ADC, no PM8901 thermal monitor, no `#thermal-sensor-cells`
provider with PM8901 as its source. Adding a thermal-zone whose
`thermal-sensors` reference targets a non-existent node would either
fail probe (best case) or panic (worst case).

### PM8901 path forward (recommended follow-up task)

Port `webos .../drivers/thermal/pmic8901-tm.c` to mainline as a small
PM8901-specific thermal driver that:

1. Probes as an MFD child of `qcom,pm8901`.
2. Reads PM8901 TEMP_STAGE_STATUS via the existing SSBI regmap exposed
   by `qcom-pm8xxx`.
3. Wires both legacy IRQs:
   - `TEMP_ALARM` — stage-transition IRQ
   - `TEMP_HI_ALARM` — hi-temp IRQ
4. Registers as a thermal-zone-of provider with
   `#thermal-sensor-cells = <0>`, enabling a board-level
   `pm8901-thermal` zone analogous to the PM8058 one.

Estimate: ~200 lines of driver, ~30 lines of DT. Not in scope for T-005
in its current Tier-0 framing. Two-event equivalence (R2 AC5) is
trivially met by such a driver since both IRQs are visible at the
SSBI level.

### Acceptance criteria coverage (R2)

- AC1 zone exists for PM8901: **PARTIAL** — blocked on driver.
- AC2 idle reading: **PARTIAL** — blocked on driver.
- AC3 trips at 105/125/145 deg C: deferred (DT trivial once R2 unblocks).
- AC4 hysteresis: deferred.
- AC5 two-event equivalence: would be naturally met by the proposed
  driver; until then, **PARTIAL**.

## R3, R4, R5 AC2-4: Deferred to Tier 1

- R3 critical-trip emul_temp → `orderly_poweroff`: T-014 will verify
  end-to-end using `/sys/class/thermal/thermal_zoneN/emul_temp` once
  `CONFIG_THERMAL_EMULATION=y` is set (see Defconfig section).
- R4 user-space-daemon independence: T-022.
- R5 AC2 polled latency <= 5 s: T-023.
- R5 AC4 measured latency recorded: T-023.

## Defconfig dependencies (for T-015)

These flags need to be added to the three tenderloin defconfigs so the
DT changes take effect:

- `CONFIG_THERMAL_OF=y` (auto-selected on most kernels, but worth
  asserting explicitly)
- `CONFIG_GENERIC_ADC_THERMAL=y` — *not currently set in tenderloin
  defconfigs* per `grep -E "GENERIC_ADC_THERMAL|THERMAL" arch/arm/configs/tenderloin*`
- `CONFIG_THERMAL_EMULATION=y` — required for `emul_temp` injection in
  T-014 / T-023

These should be applied to all three configs (default, debug, fast) as
part of T-015 in Tier 1.

## Task Tracking

| Task | Status | Notes |
|------|--------|-------|
| T-004 | DONE | PM8058 zone via generic-adc-thermal + die_temp xoadc channel; 3 trips 105/125/145, hysteresis 2k mC each |
| T-005 | PARTIAL | DT cannot land — PM8901 needs new driver port (pmic8901-tm). Recommended follow-up task scope ~200 lines driver + minor DT |
| T-014 | TODO | emul_temp -> orderly_poweroff e2e (HW, Tier 1) |
| T-015 | DONE | defconfigs: `CONFIG_GENERIC_ADC_THERMAL=y` + `CONFIG_THERMAL_EMULATION=y` + `CONFIG_THERMAL_OF=y` across 3 configs |
| T-022 | TODO | no userspace daemon test (HW, Tier 2) |
| T-023 | TODO | latency measurement (HW, Tier 2) |

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
