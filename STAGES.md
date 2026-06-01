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
| 1 | gcc-cleanup (3 patches) | `submit/dt-bindings-gcc-msm8660-cleanup` | clk | v3 ready under /tmp/v3-out/gcc-cleanup |
| 2 | mmcc-msm8660 | `submit/clk-mmcc-msm8660` | clk | ready |
| 3a | lcc hardening (3 patches, all compatibles) | `submit/clk-lcc-msm8960-hardening` | clk | v2 ready |
| 3b | lcc add-MSM8x60 (2 patches) | `submit/clk-lcc-msm8660` | clk | v2 ready; depends on 1 (PLL4_VOTE) |
| 4 | interconnect-msm8660 | `submit/interconnect-msm8660` | icc | v5-r2 ready under /tmp/v3-out/interconnect |
| 5 | irqchip-msm8660-mpm | `submit/irqchip-msm8660-mpm` | irqchip | v5 ready |
| 6 | thermal-pm8901-tm | `submit/thermal-pm8901-tm` | thermal | v5-r2 ready under /tmp/v3-out/pm8901-tm |

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
| 13 | camss-msm8660 (largest, 16 files) | `submit/media-camss-msm8660` | media | send after Stage 2 #7 lands in -next to avoid conflict |
| 14 | msm8660 rotator | `submit/media-msm8660-rotator` | media | mem2mem |
| 15 | msm8660 vpe | `submit/media-msm8660-vpe` | media | mem2mem |
| 16 | msm8660 vidc | `submit/media-msm8660-vidc` | media | mem2mem, largest of the three (1080p codec) |

## Capstone — board DTS

`arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin.dts`. Sent ~Day 30 once all
16 series above are in linux-next (or at least queued in their respective
`-next` branches).

Cross-tree references the board DTS depends on:

- mmcc / lcc clock IDs (#2, #3a/b)
- gcc clock IDs from cleanup (#1)
- interconnect IDs (#4)
- `qcom,msm8660-mpm` (#5)
- `qcom,pm8901-temp-alarm` (#6)
- `cypress,cy8ctma395-ts` (#12)
- `aptina,mt9m113` (#11)
- `qcom,msm8660-camss` / `qcom,msm8660-rotator` / `qcom,msm8660-vpe` / `qcom,msm8660-vidc` (#13–#16)
- `qcom,msm8660-qce` (#10), `qcom,vendor-init-seq` USB phy (#8), max8903 DC/USB limit (#9)

## Cadence

- **Day 1** — fire Stages 1 + 2 in parallel (10 emails to ~6 subsystems).
- **Day ~7** — address first round of review feedback; send Stage 3 (#11, #12).
- **Day ~14** — send Stage 4 multimedia stack as it stabilises.
  Optionally a single cover-letter "MSM8x60 multimedia stack" referencing all four.
- **Day ~30** — send board DTS with a cover-letter listing all 16 series as prerequisites.

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
