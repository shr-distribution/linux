# HP TouchPad (Tenderloin / MSM8x60) Upstream Plan

This document is the **canonical plan** for the upstream effort. The list is
the source of truth for what each upstream series contains, which `submit/*`
branch holds it, and the order in which they should hit lkml.

When a series is sent, queued, applied, or reworked, update its row here so
the next contributor (or session) starts with the correct picture.

## Stage 1 — clocks, interconnect, wake, thermal

Independent low-level enabling work. Cross-dependencies inside the stage are
limited (LCC depends on gcc-cleanup PATCH 3/3 for PLL4_VOTE). All Stage 1
series can be sent on Day 1 in parallel; they go to separate subsystems and
no maintainer sees more than one bundle at once.

| # | Bundle | `submit/*` branch | Subsystem | Status |
|---|---|---|---|---|
| 0 | gdsc framework (2 patches: LEGACY_FOOTSWITCH + RPM_ALWAYS_ON) | `submit/clk-gdsc-msm8x60-legacy` | clk | **v1 sent 2026-06-02 (msg-id 20260602050840.435933-1)**; prereq for #2; v2 pending |
| 0b | gdsc 3 pre-existing fixes (poll_status / ALWAYS_ON retval / unregister) | `submit/clk-gdsc-preexisting-fixes` | clk | **v1 sent 2026-06-02 (msg-id 20260602140934.796697-1)** independent series |
| 1 | gcc-cleanup (v1: CE2 H + PLL4_VOTE; v2 adds strict YAML pll4 + halt-bit comment) | `submit/dt-bindings-gcc-msm8660-cleanup` | clk | **v1 sent 2026-06-02 (msg-id 20260602042747.277270-1)**; v2 staged (3 patches); hold for lkml v1 feedback |
| 2 | mmcc-msm8660 | `submit/clk-mmcc-msm8660` | clk | **v1 sent 2026-06-02 (msg-id 20260602043623.285901-1)**; v2 pending (unhalt -EPROBE_DEFER fix + cover dep on #0); depends on #0 |
| 3a | lcc hardening (3 patches, all compatibles) | `submit/clk-lcc-msm8960-hardening` | clk | **v1 sent 2026-06-02 (msg-id 20260602045002.290918-1)** |
| 3b | lcc add-MSM8x60 (2 patches) | `submit/clk-lcc-msm8660` | clk | held — depends on #1 (PLL4_VOTE) landing first |
| 4 | interconnect-msm8660 | `submit/interconnect-msm8660` | icc | ready to send |
| 5 | irqchip-msm8660-mpm | `submit/irqchip-msm8660-mpm` | irqchip | ready to send |
| 6 | thermal-pm8901-tm | `submit/thermal-pm8901-tm` | thermal | ready to send |
| 6b | qcom-msm8660.dtsi: add LCC node + gcc pll4 reference (2 patches) | `submit/arm-dts-qcom-msm8660-gcc-pll4` | arm-dts | depends on #1 v2 (strict binding) + #3b lcc-add-MSM8x60 (driver); schedule after both. DTB builds clean on its own; dtbs_check passes once #1 v2 lands. |

## Stage 2 — charger, phy, crypto, media VFE fix

Independent driver/binding additions for the rest of the on-die peripheral
stack the board needs. Fired in parallel with Stage 1.

| # | Bundle | `submit/*` branch | Subsystem | Status |
|---|---|---|---|---|
| 7 | camss vfe-17x wm-done fix (2 patches) | `submit/media-camss-vfe-17x-wm-done-fix` | media | v3 ready |
| 8 | phy-usb-hs qcom,vendor-init-seq (2 patches) | `submit/phy-usb-hs-vendor-init-seq` | phy | v5 ready (uint8-array schema) |
| 9 | max8903 DC/USB current-limit (2 patches) | `submit/power-max8903-dc-limit` | power-supply | v5 ready (source_lock + DT gpio_value=0 require) |
| 10 | qce CE2 support (2 patches) | `submit/crypto-qce-msm8660` | crypto | v5 ready (cpu_to_be32; arch deferrals documented) |

## Stage 3 — standalone new drivers (no cross-deps)

Sent ~Day 7 after first round of review feedback for Stages 1+2.

| # | Bundle | `submit/*` branch | Subsystem | Note |
|---|---|---|---|---|
| 11 | mt9m113 sensor | `submit/media-mt9m113` | media-i2c | must land before camss DTS sets it as remote-endpoint |
| 12 | cy8ctma395 touchscreen | `submit/input-cy8ctma395` | input | independent tree |
| 13 | Palm A6 battery/EC | `submit/power-supply-palm-a6` | power-supply | 2 patches: dt-binding + driver. Tenderloin-specific; large driver (~6.4 KLOC) but standalone. |
| 14 | TI LM8502 combo binding | `submit/dt-bindings-mfd-lm8502` | dt-bindings | base binding; depended on by #15-#17 |
| 15 | TI LM8502 MFD core | `submit/mfd-lm8502` | mfd | parent driver |
| 16 | TI LM8502 LED child | `submit/leds-lm8502` | leds | depends on #15 |
| 17 | TI LM8502 haptic (FF_RUMBLE) child | `submit/input-lm8502-haptic` | input | depends on #15; TouchPad vibrator motor |

## Stage 4 — MSM8x60 multimedia stack

Camss first, then mem2mem siblings in parallel. Sent ~Day 14 as Stage 1
(clocks, interconnect) stabilises in -next.

| # | Bundle | `submit/*` branch | Subsystem | Note |
|---|---|---|---|---|
| 18 | camss-msm8660 (largest, 16 files) | `submit/media-camss-msm8660` | media | send after Stage 2 #7 lands in -next to avoid conflict |
| 19 | msm8660 rotator | `submit/media-msm8660-rotator` | media | mem2mem |
| 20 | msm8660 vpe | `submit/media-msm8660-vpe` | media | mem2mem |
| 21 | msm8660 vidc | `submit/media-msm8660-vidc` | media | mem2mem, largest of the three (1080p codec) |

## Stage 5 — Tenderloin sensor enablement (IIO)

Independent IIO patches needed for the TouchPad's sensor stack
(sensorfw via iio-sensors-adaptor). All standalone — no cross-deps within
the stage. Most extend existing bindings/drivers (no new compatible
strings); rotator/vidc-style "new driver" patches do not appear here.

Two of these are pure bugfixes applicable to ALL users of the driver
(isl29018 overflow + st-sensors endianness) — consider splitting them off
as `Fixes:`-tagged fix-only patches if maintainer pushback on the combined
series is likely. Held grouped for now.

Can be sent in parallel with Stage 1+2 (Day 1) since they touch unrelated
subsystems, but holding until first Stage 1/2 round settles to keep the
maintainer overhead bounded.

| # | Bundle | `submit/*` branch | Subsystem | Note |
|---|---|---|---|---|
| 22 | isl29018 cover-glass gain compensation + lux overflow fix (3 patches) | `submit/iio-isl29018-cover-comp` | iio (light) | extends `isil,isl29018`; first patch is standalone `Fixes:` for `isl29018_read_lux()` overflow + precision |
| 23 | lsm303dlh magn full-scale + endianness (3 patches) | `submit/iio-lsm303dlh-magn-fixes` | iio (magnetometer) | extends st-sensors with `st,fullscale-mg`; ≥±2.5G needed on tenderloin to avoid X saturation per [[lsm303dlh-be-and-bias]]; magn registers are BE at 0x03/0x05/0x07 |
| 24 | mpu3050 gyro FIFO raw-read (1 patch) | `submit/iio-mpu3050-fifo-raw-read` | iio (gyro) | reads gyro samples via FIFO in `IIO_CHAN_INFO_RAW`; driver-only |
| 25 | mpu3050 PM resume restores sample rate (1 patch) | `submit/iio-mpu3050-pm-resume-restore-state` | iio (gyro) | restores cached sample rate on runtime resume; driver-only |

## Capstone — board DTS

`arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin.dts`. Sent ~Day 30 once
all 21 series above are in linux-next (or at least queued in their
respective `-next` branches).

Cross-tree references the board DTS depends on:

- mmcc / lcc clock IDs (#2, #3a/b)
- gcc clock IDs from cleanup (#1)
- interconnect IDs (#4)
- `qcom,msm8660-mpm` (#5)
- `qcom,pm8901-temp-alarm` (#6)
- `cypress,cy8ctma395-ts` (#12)
- `aptina,mt9m113` (#11)
- `qcom,msm8660-camss` / `qcom,msm8660-rotator` / `qcom,msm8660-vpe` / `qcom,msm8660-vidc` (#18–#21)
- `qcom,msm8660-qce` (#10), `qcom,vendor-init-seq` USB phy (#8), max8903 DC/USB limit (#9)
- `isil,cover-comp-gain` extension on `isil,isl29018` (#22)
- `st,fullscale-mg` extension on st-sensors magn binding (#23)

## Cadence

- **Day 1** — fire Stages 1 + 2 in parallel (10 emails to ~6 subsystems).
- **Day ~7** — address first round of review feedback; send Stage 3 (#11, #12).
- **Day ~10** — send Stage 5 (sensors). Independent of multimedia; spacing
  it after Stage 3 keeps per-subsystem inbox load reasonable.
- **Day ~14** — send Stage 4 multimedia stack as it stabilises.
  Optionally a single cover-letter "MSM8x60 multimedia stack" referencing all four.
- **Day ~30** — send board DTS with a cover-letter listing all 21 series as prerequisites.

Each bundle is 1–3 commits and goes to its own subsystem tree; send each
bundle as its own `git format-patch --cover-letter` series. Do **not**
build a 50-patch megaseries — qcom reviewers will reject it.

## Workflow notes

- `submit/*` branches are **local-only**. Never push to `shr-github` (the
  remote lacks `arm-msm/for-next` history and rejects the 2 GiB pack).
  Integration branch goes to `shr-github:tenderloin/linux-next` only.
- Always run `pre-send-check.sh` (or `send-series.sh`) before
  `git send-email` — catches placeholder leaks, duplicate `v-prefix`,
  mangled copyright lines; enforces 60s spacing between series.
- Sashiko preflight wrapper: `sashiko-preflight.sh <branch> <base> <label>`.
  Stop iterating after ~2 substantive rounds — when a new finding contradicts
  a v(n-1) fix on the same line, ship rather than chase zero findings.
