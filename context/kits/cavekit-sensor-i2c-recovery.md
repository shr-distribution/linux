---
domain: sensor-i2c-recovery
created: "2026-05-19"
last_edited: "2026-05-19"
status: draft
---

# Cavekit: GSBI3 Sensor I2C Bus Recovery

## Scope

The GSBI3 sensor I2C bus (gyro mpu3050, ambient-light isl29023, and any
other gsbi3 child) recovers automatically when a slave wedges SDA low,
replicating the legacy `board_i2c_recover()` behavior so the bus is usable
again without a reboot.

## Context

The legacy 2.6.35-palm kernel registered `board_i2c_recover()` as the
GSBI3 recovery callback in its platform data
(`arch/arm/mach-msm/board-tenderloin.c:1855-1975`). Recovery toggled
SCL (GPIO43) for `I2C_RECOVER_CLOCK_CYCLES = 36` cycles
(`gpiomux-tenderloin.h:173`) while monitoring SDA (GPIO44), then issued a
manual START/STOP to release a slave stuck mid-transfer.

The 6.18 port leaves `gsbi3_i2c` with pinctrl but no bus-recovery
declaration. A wedged mpu3050 or isl29023 therefore makes the entire bus
unusable until reboot, regressing field reliability vs the legacy kernel.
This was flagged as Hardware Quirks Inventory Q8 / Audit Gap #11
(MEDIUM-LOW severity — rare but unrecoverable when it bites).

This kit is bus-recovery-only; it does **not** address why a sensor might
wedge in the first place. Recovery is a backstop.

## Requirements

### R1: Automatic Recovery When SDA Is Held Low
**Description:** When a slave on GSBI3 holds SDA low, the controller
attempts bus recovery and the bus returns to a usable state without a
reboot.

**Acceptance Criteria:**
- [ ] With mpu3050 or isl29023 forced into a stuck-SDA condition (via a
      test rig or driver-injected fault), a subsequent I2C transfer on
      the same bus either completes successfully or returns `-EAGAIN`
- [ ] A bus-recovery attempt is visible in `dmesg` (or equivalent kernel
      log) when the wedge is detected
- [ ] After the recovery attempt, a subsequent normal I2C transfer to a
      working slave on GSBI3 succeeds — confirmed by reading a known
      register on at least one healthy child device
- [ ] No system reboot is required to restore bus function

**Dependencies:** none

### R2: No Regression on Healthy Bus
**Description:** Normal probe and steady-state I/O of all GSBI3 children
is unaffected by the presence of the recovery declaration.

**Acceptance Criteria:**
- [ ] On cold boot, the mpu3050 gyro probes successfully (visible as an
      IIO device with expected channel set)
- [ ] On cold boot, the isl29023 ambient-light sensor probes successfully
      (visible as an IIO device with expected channel set)
- [ ] Over a 24-hour run of nominal sensor polling, no bus-recovery
      attempt is logged
- [ ] Steady-state read latency for either child device is not measurably
      worse than the pre-recovery baseline (within run-to-run noise)

**Dependencies:** R1

### R3: Recovery Cycle Count Is Sufficient
**Description:** Enough SCL clock pulses are issued during recovery to
clear any in-flight 9-bit I2C slave state.

**Acceptance Criteria:**
- [ ] The chosen recovery cycle count is at least 9 SCL clocks before
      START/STOP is issued — the I2C spec minimum to clock through any
      stuck byte
- [ ] The exact chosen count and its rationale are recorded in the impl
      plan (legacy used 36; mainline standard recovery typically uses 9 —
      the impl plan must declare which and why)
- [ ] The synthetic-wedge release rate over 100 trials ties to the chosen
      count: **100 %** if the plan chooses ≥ 36 cycles (the legacy
      worst-case figure — any non-100 % rate at that count is a defect);
      **≥ 95 %** if the plan chooses the 9-cycle spec minimum (the lower
      margin reflects that 9 is the bare-minimum protocol bound, not a
      headroom-padded value). Any intermediate count must define its
      target rate in the impl plan with rationale.

**Dependencies:** R1

## Out of Scope

- Recovery for other GSBI I2C buses (4, 7, 8, 10, 12) — separate concern,
  no evidence those wedge in practice
- Root-causing why a sensor wedges (driver bug, ESD event, etc.) —
  recovery is the backstop, not the fix
- Generic I2C controller register-level reset (whole-controller reset is
  a different escalation path)

## Cross-References

none

## Source Traceability

- `reports/HARDWARE-QUIRKS-INVENTORY-2026-05-19.md` — Q8 (table row),
  "Net-new items" §Gap #11
- `reports/BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md` — Gap #11 (Executive
  Summary §1), §4 action #7
- Legacy implementation:
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1855-1975`
  (`board_i2c_recover()`)
- Legacy constant:
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/gpiomux-tenderloin.h:173`
  (`I2C_RECOVER_CLOCK_CYCLES = 36`)
- Legacy plat-data wiring:
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1975`
  (`.msm_i2c_recover = board_i2c_recover`)
