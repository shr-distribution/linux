---
created: "2026-05-19"
last_edited: "2026-05-19"
---

# Cavekit Overview

## Project
linux-6.18-tenderloin — Linux 6.18 kernel port for the HP TouchPad
(Qualcomm APQ8060, LuneOS). Faithful port of the legacy webOS
2.6.35-palm board file: when a legacy behavior exists, the kits replicate
it (same trip values, IRQ semantics, register sequences) unless
explicitly stated otherwise.

## Domain Index
| Domain | File | Summary | Status |
|--------|------|---------|--------|
| spm-init | cavekit-spm-init.md | Initialize SPM/SAW registers at probe for MSM8660 power collapse | draft |
| soc-watchdog | cavekit-soc-watchdog.md | Hardware SoC watchdog enablement (gated on mainline-binding investigation) | draft |
| sensor-i2c-recovery | cavekit-sensor-i2c-recovery.md | GSBI3 sensor I2C bus auto-recovery when a slave wedges SDA low | draft |
| pmic-thermal | cavekit-pmic-thermal.md | PM8058/PM8901 die-temp thermal zones + 105/125/145 °C trip model + critical poweroff | draft |
| wifi-suspend-wake | cavekit-wifi-suspend-wake.md | WiFi wake-from-suspend via SDC4 DAT1 routed through MPM (hard-gated on MPM) | draft |
| usb-phy-tuning | cavekit-usb-phy-tuning.md | Investigate-then-conditional USB ULPI PHY signal-quality tuning replicating legacy values | draft |
| usb-charger-detection | cavekit-usb-charger-detection.md | Detect USB charger type (SDP/CDP/DCP + HP Touchstone variants) and expose to userspace | draft |

## Out-of-Kit Tracked Items

The following audit findings are intentionally **not** cavekits, because
existing impl docs already track them and creating kits would duplicate
state. They remain part of the port's outstanding work surface.

| Gap | Subject | Tracked in |
|-----|---------|-----------|
| #1 | WiFi 1.8 V rail (`pm8058_s3`) HPM/load management — root cause of CMD53 -110 userspace regression | de-scoped from this kit batch; see `reports/BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md` §1 finding #1 |
| #2 | DVT/EVT eMMC enumerates at 0 B (Samsung fw 9.0 CMD8 sequencing) | `context/impl/impl-emmc-dvt-firmware.md` |
| #3 | MPM device-tree node disabled (early boot hang blocks deep idle / suspend) | `context/impl/impl-mpm-boot-hang.md` |
| #4 | VIDC video codec hangs at firmware boot | tracked in audit; no kit yet |
| #5 | GPU Adreno-220 "period-8" rendering (1/8 frames correct) | tracked in audit; no kit yet |
| #7 | PMIC RTC writable (`allow-set-time`) | **RESOLVED / false-positive audit finding** — property already present at `arch/arm/boot/dts/qcom/pm8058.dtsi:138`; original audit grep only searched `common.dtsi` + `topaz.dts` and missed the included `pm8058.dtsi` |

## Cross-Reference Map

| Domain A | Interacts With | Interaction Type |
|----------|----------------|------------------|
| spm-init | cpuidle (cpu-spc) | Enables — provides register init for cpu-spc state |
| spm-init | cpu-hotplug | Enables — provides register init for hotplug power collapse |
| wifi-suspend-wake | impl-mpm-boot-hang.md | **Hard precondition** — MPM platform driver must land first |
| pmic-thermal | (none) | Independent |
| soc-watchdog | (none) | Independent |
| sensor-i2c-recovery | (none) | Independent |
| usb-phy-tuning | (none) | Independent |
| usb-charger-detection | usb-phy-tuning | Same regression-guard bar; ULPI plug/unplug infrastructure shared |

## Dependency Graph

```
spm-init (foundation, in test)
  ├─> cpu-hotplug (already implemented)
  └─> cpuidle-spc (pending RPM orchestrator)

impl-mpm-boot-hang.md   (BLOCKER — not a kit)
  └─> wifi-suspend-wake (gated by R1 / MPM precondition)

soc-watchdog          (independent; R1 is investigation-only)
sensor-i2c-recovery   (independent)
pmic-thermal          (independent)
usb-phy-tuning        (independent; R1 is investigation-only)
usb-charger-detection (R1 is investigation-only; shares regression bar with usb-phy-tuning R3)
```

## Coverage Summary

- Total kits: 7 (1 existing + 5 from 2026-05-19 batch + 1 from 2026-05-21)
- New kits this batch: soc-watchdog, sensor-i2c-recovery, pmic-thermal,
  wifi-suspend-wake, usb-phy-tuning
- New kit 2026-05-21: usb-charger-detection
- Investigation-first kits (R1 = investigate, downstream R's conditional):
  soc-watchdog, usb-phy-tuning, usb-charger-detection
- Hard-precondition kits: wifi-suspend-wake (gated on
  `impl-mpm-boot-hang.md`)
- Originally planned but withdrawn: `rtc-writable` — see Out-of-Kit
  Tracked Items row Gap #7 (audit finding was a false positive; the
  required DT property already exists)
