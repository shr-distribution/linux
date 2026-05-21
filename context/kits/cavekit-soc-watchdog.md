---
domain: soc-watchdog
created: "2026-05-19"
last_edited: "2026-05-19"
status: draft
---

# Cavekit: SoC Hardware Watchdog Enablement

## Scope

A hardware watchdog on the MSM8660/APQ8060 SoC is enabled, exposed to the
kernel watchdog framework, and usable from userspace to recover from a
fully-locked system — *provided* mainline supports a watchdog binding for
this SoC family. The kit covers the investigation that must precede any
enablement decision.

## Context

The legacy 2.6.35-palm kernel set `CONFIG_MSM_WATCHDOG=y` and armed the
SoC watchdog at boot. The current 6.18 port has no watchdog enabled in
defconfig and `qcom-msm8660.dtsi` carries no watchdog DT node. Mainline
ships a `qcom,kpss-wdt` binding for newer Krait-based SoCs, but its
applicability to Scorpion / MSM8660 has **not been verified** — the
watchdog block address and register layout for MSM8660 may differ from the
KPSS variants, and no in-tree DTSI references the binding for an MSM8660
target. The kit therefore wraps the investigation, not just a defconfig
flip.

This is a low-severity gap (the device boots and runs without it), but
absence of a hardware watchdog means a hard kernel lock is unrecoverable
without manual power-cycle, weakening field reliability.

## Requirements

### R1: Mainline Binding Applicability Investigation
**Description:** Determine whether any in-tree mainline driver/binding can
manage a hardware watchdog on MSM8660 (Scorpion), citing concrete driver
source and binding documentation in the conclusion.

**Acceptance Criteria:**
- [ ] Investigation conclusion is written to
      `context/impl/impl-soc-watchdog.md`
- [ ] Conclusion identifies the candidate mainline driver (or states that
      none applies) by file path and binding name
- [ ] If a binding is identified, the register block address and reset
      behaviour expected by that driver are matched against the MSM8660
      memory map and noted as compatible or incompatible
- [ ] The decision (enable / won't-fix) is explicitly recorded with
      rationale

**Dependencies:** none

### R2: Watchdog DT Node and Bind (Conditional)
**Description:** If R1 concludes a mainline binding is applicable, the
watchdog is wired up in the board DTS and the kernel exposes the standard
watchdog interface. If R1 concludes no binding applies, this requirement
is satisfied by documenting the won't-fix decision.

**Acceptance Criteria:**
- [ ] If R1 = "binding exists": at boot, the kernel exposes `/dev/watchdogN`
- [ ] If R1 = "binding exists": `wdctl /dev/watchdogN` reports a non-zero
      timeout value
- [ ] If R1 = "no binding exists": `impl-soc-watchdog.md` records the
      won't-fix decision, the reason mainline cannot drive this hardware,
      and what would be needed to change that conclusion

**Dependencies:** R1

### R3: No Boot or Shutdown Regression
**Description:** Enabling the watchdog must not destabilize boot or
graceful shutdown — a watchdog that fires during normal operation is worse
than no watchdog.

**Acceptance Criteria:**
- [ ] At least 5 consecutive cold boots complete to userspace login without
      a watchdog-induced reset
- [ ] A userspace watchdog pet (the distro's standard watchdog service)
      keeps the device alive for at least 30 minutes of normal idle
- [ ] An orderly poweroff completes within the configured shutdown grace
      window without producing a spurious reset

**Dependencies:** R2 (only meaningful if R2 enabled the watchdog)

### R4: Watchdog Recovers a Locked Kernel
**Description:** A real (not pet-only) watchdog: when the kernel is fully
locked, the watchdog timeout must fire and reset the device. This is the
recovery property the Scope statement promises; without it the kit only
proves the watchdog doesn't false-fire.

**Acceptance Criteria:**
- [ ] An induced full-kernel lockup (e.g. SysRq-c crash, or a synthetic
      "hung task" hold-CPU-with-IRQ-disabled stub) causes a hardware reset
      within the configured watchdog window (±1 watchdog tick tolerance)
- [ ] The reset is observed as a fresh boot in the kernel log on the next
      power-on, with the previous boot's last kernel log preserved in
      pstore (where pstore RAM region is configured) — confirming the
      reset path is the watchdog, not a power loss
- [ ] If R1 concluded "no binding exists" and R2 is satisfied as
      won't-fix, R4 is satisfied by recording in the impl doc that
      lock-recovery is intentionally unimplemented and what would change
      that conclusion

**Dependencies:** R2 (meaningful only if R2 enabled the watchdog)

## Out of Scope

- Software watchdog (`softdog`) — kernel-only, does not survive a hard lock
- Userspace watchdog daemon configuration beyond what R3 needs for
  verification
- IPC/Q6/RPM-side watchdogs (different subsystem)
- Watchdog-based debug-trace / scandump features

## Cross-References

none

## Source Traceability

- `reports/BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md` — Gap #8 (Executive
  Summary §1), §3 row `MSM_WATCHDOG=y`, §4 action #1 list (note: low priority)
- Legacy defconfig setting: `CONFIG_MSM_WATCHDOG=y`
- Current DTSI: `qcom-msm8660.dtsi` — no watchdog node present (verified
  in audit)
