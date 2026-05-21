---
domain: pmic-thermal
created: "2026-05-19"
last_edited: "2026-05-19"
status: draft
---

# Cavekit: PMIC Die-Temperature Monitoring

## Scope

PM8058 and PM8901 die temperatures are exposed as Linux thermal zones with
the legacy webOS 3-stage trip model (105 °C / 125 °C / 145 °C), and a
critical trip triggers a kernel-driven orderly poweroff. This replicates
the legacy `pmic8058-tm` / `pmic8901-tm` behaviour so the device retains
silicon over-temperature protection.

## Context

The legacy 2.6.35-palm kernel registered both PMICs as 3-stage
`thermal_zone_device`s. The stage geometry was:

- Stage step: 20 °C (`PM8058_TEMP_STAGE_STEP = 20000`,
  `PM8901_TEMP_STAGE_STEP = 20000`)
- Hysteresis: 2 °C (`PM8058_TEMP_STAGE_HYSTERESIS = 2000`,
  `PM8901_TEMP_STAGE_HYSTERESIS = 2000`)
- Threshold floor: 105 °C (`PM8058_TEMP_THRESH_MIN = 105000`,
  `PM8901_TEMP_THRESH_MIN = 105000`)

Yielding three trip points per PMIC at **105 °C, 125 °C, 145 °C**, with
Stage 3 = `THERMAL_TRIP_CRITICAL` invoking the kernel's
`orderly_poweroff()`. Both PMICs ran in software-override mode
(`SOFTWARE_OVERRIDE_ENABLED`) so the kernel manages shutdown rather than
the PMIC hardware-cut path. PM8058 has one temperature IRQ
(`TEMP_ALARM`); PM8901 has two (`TEMP_ALARM` + `TEMP_HI_ALARM`).

Mainline 6.18 has no `pm8058-tm` or `pm8901-tm` driver, and the PMICs are
SSBI so `QCOM_SPMI_TEMP_ALARM` is not applicable. However,
`pm8058_xoadc` already exposes a `die_temp` IIO channel
(`pm8058.dtsi:113`, `adc-channel@b` with `SCALE_PMIC_THERM`), so a polled
thermal-zone path is feasible without porting the legacy driver. The kit
must NOT pin a specific implementation path; two plausible routes exist
(polled via xoadc, or IRQ-driven port of the legacy driver) and either
satisfies the requirements provided the acceptance criteria are met.

Without this protection, a runaway thermal event silently damages
silicon rather than triggering an orderly shutdown — listed as Audit
Gap #6 (MEDIUM severity, genuine and previously untracked).

## Requirements

### R1: PM8058 Thermal Zone with 3 Trip Points
**Description:** PM8058 die temperature is exposed as a Linux thermal
zone with three trip points matching the legacy values.

**Acceptance Criteria:**
- [ ] A thermal zone exists whose `type` identifies it as the PM8058 die
- [ ] `/sys/class/thermal/thermal_zoneN/temp` returns a plausible idle
      reading in the range 15 000 to 80 000 (mC) when the device is at
      typical ambient + chassis temperature
- [ ] The zone exposes three trip points whose temperatures match
      105 000 mC, 125 000 mC, and 145 000 mC within ±1 000 mC
- [ ] The reported hysteresis (where exposed) is ≤ 2 000 mC, matching the
      legacy hysteresis constant

**Dependencies:** none

### R2: PM8901 Thermal Zone with 3 Trip Points
**Description:** PM8901 die temperature is exposed as its own Linux
thermal zone with the same three-trip geometry. PM8901 is distinguished
from PM8058 by exposing **two** temperature event sources (the legacy
`TEMP_ALARM` stage-transition IRQ and the `TEMP_HI_ALARM` hi-temp IRQ);
both must be reachable by the kernel.

**Acceptance Criteria:**
- [ ] A second thermal zone exists whose `type` identifies it as the
      PM8901 die (distinct from R1's zone)
- [ ] `/sys/class/thermal/thermal_zoneN/temp` returns a plausible idle
      reading in the range 15 000 to 80 000 (mC)
- [ ] Three trip points match 105 000 mC, 125 000 mC, and 145 000 mC
      within ±1 000 mC
- [ ] Reported hysteresis ≤ 2 000 mC
- [ ] Both legacy PM8901 IRQ semantics — stage-transition (`TEMP_ALARM`)
      and hi-temp (`TEMP_HI_ALARM`) — are observable to the kernel
      (either as two distinct interrupt sources, or as a documented
      functional equivalence: the impl plan declares which and explains
      why a single-event model fully covers the two-event behaviour if
      that path is chosen)

**Dependencies:** none

### R3: Critical Trip Triggers Orderly Poweroff
**Description:** The 145 °C trip is wired as critical and causes the
kernel to power the device off cleanly when reached.

**Acceptance Criteria:**
- [ ] Using `/sys/class/thermal/thermal_zoneN/emul_temp` (or an
      equivalent test injection mechanism), injecting a value ≥ 145 000 mC
      on either PMIC zone produces a kernel log line identifying the
      critical trip
- [ ] The kernel dispatches `orderly_poweroff` (or its mainline
      equivalent) within 30 s of the emulated critical trip
- [ ] The system completes a clean poweroff (filesystems sync, no kernel
      panic recorded in pstore on next boot)

**Dependencies:** R1, R2

### R4: Independent of User-Space Thermal Daemon
**Description:** The critical-trip response works without any user-space
thermal management agent running.

**Acceptance Criteria:**
- [ ] With no `thermald`, `tmon`, or equivalent user-space daemon running
      (verified via `ps`), R3's emul_temp injection still triggers
      orderly poweroff

**Dependencies:** R3

### R5: Response Latency Bound
**Description:** The trip-to-shutdown latency is fast enough to be useful
on a real over-temperature event.

**Acceptance Criteria:**
- [ ] The impl plan explicitly declares whether the chosen approach is
      polled or IRQ-driven, with rationale
- [ ] For a polled implementation: time from `emul_temp` write to
      `orderly_poweroff` dispatch is measured and is ≤ 5 s
- [ ] For an IRQ-driven implementation: the same latency is ≤ 1 s
- [ ] The measured latency is recorded in the impl doc

**Dependencies:** R3

## Out of Scope

- Battery thermal monitoring (handled by the battery driver / fuel-gauge
  path)
- Per-core CPU thermal throttling (separate concern, no CPU temp sensor
  exposed on this SoC in mainline)
- User-space thermal daemons (`thermald`/`tmon`) beyond R4's check that
  the kernel path stands alone
- PMIC hardware-cut behaviour (the kit explicitly continues the legacy
  software-override model)

## Cross-References

none

## Source Traceability

- `reports/BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md` — Gap #6 (Executive
  Summary §1), §2.2 row `pm8058-tm + pm8901-tm`, §3 row
  `THERMAL_PM8901=y`, §4 action #4
- Legacy PM8058 constants:
  `webos-linux-kernel-touchpad/drivers/thermal/pmic8058-tm.c:51`
  (`PM8058_TEMP_STAGE_STEP = 20000`), `:52`
  (`PM8058_TEMP_STAGE_HYSTERESIS = 2000`), `:54`
  (`PM8058_TEMP_THRESH_MIN = 105000`)
- Legacy PM8058 trip table:
  `webos-linux-kernel-touchpad/drivers/thermal/pmic8058-tm.c:244-271`
  (Stage1/2/3 computed from threshold + stage step), `:229`
  (`THERMAL_TRIP_CRITICAL`)
- Legacy PM8901 constants:
  `webos-linux-kernel-touchpad/drivers/thermal/pmic8901-tm.c:46-49`
  (parallel `_STAGE_STEP = 20000`, `_STAGE_HYSTERESIS = 2000`,
  `_THRESH_MIN = 105000`)
- Legacy software-override sequence:
  `webos-linux-kernel-touchpad/drivers/thermal/pmic8058-tm.c:85-86,134,
  209-212,413,465,478` (`SOFTWARE_OVERRIDE_ENABLED/DISABLED` lifecycle)
- Current DT die-temp channel: `pm8058.dtsi:113` (`adc-channel@b`,
  `SCALE_PMIC_THERM` — already exposed via `pm8058_xoadc`)
