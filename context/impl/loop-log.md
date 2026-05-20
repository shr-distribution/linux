---
created: "2026-05-20"
last_edited: "2026-05-20"
---

# /ck:make Loop Log

Build site: context/plans/build-site.md

### Iteration 1 — 2026-05-20 (Wave 1: Tier 0 subagent dispatch — FAILED)

- **Wave:** 4 packets dispatched in parallel (opus task-builder, worktree iso)
  - Packet A: T-001 (soc-watchdog R1)
  - Packet B: T-002 + T-003 (sensor-i2c-recovery R1, R3a/b)
  - Packet C: T-004 + T-005 (pmic-thermal R1, R2, R5 AC1)
  - Packet D: T-006 (usb-phy-tuning R1)
- **Result:** 4/4 FAILED — each agent emitted text descriptions of tool
  calls but never invoked actual tools (0 tool_uses per agent, durations
  18-47 s). HEAD unchanged, no worktrees left. Mode: opus subagent
  tool-rendering failure.
- **Decision:** abandon delegation for this loop, do work inline.

### Iteration 2 — 2026-05-20 (Tier 0 inline)

- **Task:** T-001 — soc-watchdog R1 mainline binding investigation
- **Tier:** 0
- **Status:** DONE. Decision: enable path. Driver=`drivers/watchdog/qcom-wdt.c`,
  combo `qcom,scss-timer + qcom,msm-timer` already on the existing
  `timer@2000000` node; only `clocks = <&sleep_clk>; clock-names = "sleep";`
  addition needed (T-008). reg via cpu-offset 0x40000 lands at
  MSM_TMR0_BASE=0x02040000, WDT_RST at +0x38 matches legacy exactly.
- **Files:** context/impl/impl-soc-watchdog.md, context/impl/impl-overview.md
- **Validation:** Build N/A (no build per `feedback_no_local_builds.md`),
  Tests N/A (decision doc only), Acceptance 4/4 (R1 ACs documented)
- **Commit:** a70852cbd40c

### Iteration 3 — 2026-05-20

- **Task:** T-006 — usb-phy-tuning R1 ULPI vendor-reg investigation
- **Tier:** 0
- **Status:** DONE. Decision: path-found. Existing `qcom,init-seq` in
  `drivers/phy/qualcomm/phy-qcom-usb-hs.c:145` writes to
  `ULPI_EXT_VENDOR_SPECIFIC + addr` (0x80+); legacy regs 0x32 + 0x36
  unreachable. Recommendation: add new optional DT property
  `qcom,vendor-init-seq` + ~30 line driver patch using direct
  `ulpi_write(addr, val)`. T-016 will apply legacy values once driver
  patch lands; T-017 (won't-fix path) not active.
- **Files:** context/impl/impl-usb-phy-tuning.md, context/impl/impl-overview.md
- **Validation:** Acceptance 4/4 (R1 AC1-4 documented)
- **Commit:** b1a8a32796db

### Iteration 4 — 2026-05-20

- **Task:** T-002 + T-003 — sensor-i2c-recovery R1 + R3a/b
- **Tier:** 0
- **Status:** DONE. i2c-qup driver patch (include pinctrl/consumer.h,
  embed `struct i2c_bus_recovery_info bri` in `qup_i2c_dev`, probe-time
  `devm_pinctrl_get()` + `adap.bus_recovery_info = &bri`). DT adds new
  `gsbi3_i2c_gpio_pins` pinmux state (gpio function on gpio43/44) plus
  `&gsbi3_i2c` override with `pinctrl-names = "default","gpio"`,
  `pinctrl-1`, `scl-gpios`, `sda-gpios`. T-003 cycle count = 9
  (mainline `i2c_generic_scl_recovery` default, spec minimum) with
  target rate >= 95 % deferred to T-012 HW test.
- **Files:** drivers/i2c/busses/i2c-qup.c,
  arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi,
  context/impl/impl-sensor-i2c-recovery.md, context/impl/impl-overview.md
- **Validation:** Acceptance R1 4/4 code-path covered (HW verification
  deferred to T-012), R3 ACa/ACb 2/2 (AC c deferred)
- **Commit:** 44412f8e1532

### Iteration 5 — 2026-05-20

- **Task:** T-004 + T-005 — pmic-thermal R1 + R2 (PARTIAL) + R5 AC1
- **Tier:** 0
- **Status:** T-004 DONE / T-005 PARTIAL. PM8058 zone via
  `generic-adc-thermal` wrapper on `pm8058_xoadc die_temp` channel,
  three trips 105/125/145 deg C with 2 deg C hysteresis, polling 1-2 s,
  critical trip on stage 3. PM8901 zone NOT landed — no mainline IIO/ADC
  driver for PM8901 exists; would require porting legacy
  `pmic8901-tm` SSBI-direct driver (~200 lines). Documented as
  recommended follow-up task in impl-pmic-thermal.md.
  Defconfig requirements for T-015 noted: `CONFIG_GENERIC_ADC_THERMAL=y`,
  `CONFIG_THERMAL_EMULATION=y`, `CONFIG_THERMAL_OF=y`.
- **Files:** arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi,
  context/impl/impl-pmic-thermal.md, context/impl/impl-overview.md
- **Validation:** R1 4/4 (DT literal values match cavekit + legacy),
  R5 AC1 (polled approach declared), R2 PARTIAL (gap documented),
  R3/R4/R5 AC2-4 deferred to Tier 1+
- **Commit:** 98f0688d46d2

### Tier 0 summary

- DONE: T-001, T-002, T-003, T-004, T-006
- PARTIAL: T-005 (PM8901 driver port required — recommended follow-up)
- BLOCKED: T-007 (external — see context/impl/impl-mpm-boot-hang.md)
- N/A this run: T-009, T-017 (won't-fix branches not taken)

### Iteration 6 — 2026-05-20 (Tier 1 code work)

- **Task:** T-008 — wire SoC watchdog (qcom-wdt on apcs_timer)
- **Tier:** 1
- **Status:** DONE. msm8660.dtsi: added `apcs_timer:` label to
  `timer@2000000` (no functional change). tenderloin-common.dtsi:
  added `&apcs_timer { clocks = <&sleep_clk>; clock-names = "sleep"; };`
  override. Driver bind picks up apcs_tmr offsets → WDT MMIO at
  0x02040000 + 0x38 = legacy MSM_WATCHDOG path exactly.
- **Files:** arch/arm/boot/dts/qcom/qcom-msm8660.dtsi,
  arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi,
  context/impl/impl-soc-watchdog.md, context/impl/impl-overview.md
- **Validation:** R2 enable-path DT change matches kit description;
  AC1/AC2 HW-verify deferred to T-011
- **Commit:** efab94f83e41

### Iteration 7 — 2026-05-20

- **Task:** T-010 + T-015 — defconfig watchdog + thermal flags
- **Tier:** 1
- **Status:** DONE. All three tenderloin defconfigs updated:
  `CONFIG_WATCHDOG=y` + `CONFIG_QCOM_WDT=y` for T-010;
  `CONFIG_THERMAL_OF=y` + `CONFIG_THERMAL_EMULATION=y` +
  `CONFIG_GENERIC_ADC_THERMAL=y` for T-015. tenderloin_debug_defconfig
  previously had explicit `# CONFIG_WATCHDOG is not set` — flipped to
  `=y`.
- **Files:** arch/arm/configs/tenderloin_defconfig,
  arch/arm/configs/tenderloin_fast_defconfig,
  arch/arm/configs/tenderloin_debug_defconfig,
  context/impl/impl-pmic-thermal.md, context/impl/impl-overview.md
- **Validation:** flags present in all 3 configs; HW-effect deferred
  to Yocto build + boot
- **Commit:** f715f2b3a2eb

### Iteration 8 — 2026-05-20

- **Task:** T-016 — USB PHY apply legacy values via new
  `qcom,vendor-init-seq` property
- **Tier:** 1
- **Status:** DONE. Driver patch (struct field +
  `qcom_usb_hs_phy_init()` post-reset write loop +
  `qcom_usb_hs_phy_probe()` parser) lands the property as a small
  ~30-line addition to `drivers/phy/qualcomm/phy-qcom-usb-hs.c`.
  Binding YAML extended to declare the new property. DT applies 3 of
  4 legacy values: `reg 0x32 = 0x35` (pre-emphasis 20 % + HS slope
  0x05), `reg 0x36 = 0x02` (CDR auto-reset disable). SE1-gating-disable
  intentionally skipped — bit position not in the legacy headers
  surveyed; risks regression if guessed.
- **Files:** drivers/phy/qualcomm/phy-qcom-usb-hs.c,
  Documentation/devicetree/bindings/phy/qcom,usb-hs-phy.yaml,
  arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi,
  context/impl/impl-usb-phy-tuning.md, context/impl/impl-overview.md
- **Validation:** R2 AC1/2/3 done (DT literal values match legacy),
  AC4 SE1 deferred (documented), AC5 verifiable-from-driver-state
  deferred to T-018 HW or ad-hoc ulpi_read shim
- **Commit:** c6e9bd277bd6

### Run summary (autonomous /ck:make end state)

| Task | Status | Notes |
|------|--------|-------|
| T-001 | DONE | watchdog R1 investigation -> enable path |
| T-002 | DONE | i2c-qup recovery driver patch + DT |
| T-003 | DONE | 9-cycle SCL chosen + rationale |
| T-004 | DONE | PM8058 polled thermal zone (3 trips, 2 deg C hys) |
| T-005 | PARTIAL | PM8901 zone needs new driver port (~200 lines) |
| T-006 | DONE | USB PHY R1 investigation -> path-found |
| T-007 | BLOCKED | external (impl-mpm-boot-hang.md, 3 dead-ends) |
| T-008 | DONE | watchdog DT clocks reference via apcs_timer label |
| T-009 | N/A | won't-fix branch not taken (T-001 = enable) |
| T-010 | DONE | defconfig WATCHDOG + QCOM_WDT in 3 configs |
| T-011 | TODO | HW: 5 cold boots + 30 min idle pet + poweroff |
| T-012 | TODO | HW: synthetic-wedge 100 trials >= 95 % |
| T-013 | TODO | HW: 24-h healthy-bus + cold-boot probe |
| T-014 | TODO | HW: emul_temp >= 145C -> orderly_poweroff |
| T-015 | DONE | defconfig THERMAL_OF + THERMAL_EMULATION + GENERIC_ADC_THERMAL in 3 configs |
| T-016 | DONE | qcom,vendor-init-seq driver patch + DT 3/4 values |
| T-017 | N/A | won't-fix branch not taken (T-006 = path-found) |
| T-018 | TODO | HW: 10 plug/unplug + 1 GiB bulk + 10 % rate |
| T-019 | BLOCKED | depends on T-007 |
| T-020 | BLOCKED | depends on T-007 |
| T-021 | TODO | HW: induced lockup -> HW reset (only if T-008 active) |
| T-022 | TODO | HW: no-userspace-daemon test |
| T-023 | TODO | HW: latency measurement <= 5 s |
| T-024 | BLOCKED | depends on T-007, T-019, T-020 |

**Totals:** 9 DONE + 1 PARTIAL + 2 N/A + 4 BLOCKED (external) + 8 HW-only TODO = 24 tasks accounted for.

**Stop reason:** loop has exhausted what is doable autonomously without
device access. All TODO items in the run-summary table require either
hardware verification (8 tasks) or resolution of the MPM external
blocker (4 tasks) or a new mainline driver port (1 task). The
completion promise is intentionally NOT emitted — work remains, but
none of it is code-only doable in this loop.
