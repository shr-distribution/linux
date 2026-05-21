# Hardware Quirks & Errata Inventory — board-tenderloin.c → 6.18

**Date:** 2026-05-19
**Companion to:** `BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md`
**Scope:** Every workaround / errata / silicon-bug mitigation / revision-conditional hack found in `board-tenderloin.c`, its companions (`devices-tenderloin.c`, `gpiomux-tenderloin.[ch]`, `a6_sbw_impl_tenderloin.*`, `devices-msm8x60.c`) **and the vendor drivers they configure** (`drivers/mmc/host/msm_sdcc.c`), cross-checked against the 6.18 DT/driver port.

The §2 audit matrix covered *devices*. This document covers *quirks* — the non-obvious behavioural workarounds that don't show up as a device node and are easy to lose in a board-file → device-tree migration.

Legend: ✅ carried correctly · 🟡 partial / open risk · 🟥 missed (genuine gap) · ➖ N/A (block not in mainline path)

---

## Headline clarification re: "the CMD52 errata"

The CMD52 errata **is the SDCC `dummy52_required` quirk** (`msm_sdcc.c:75-85, 888, 1046` — a dummy `CMD52` is injected around data commands to work around a Qualcomm SDCC SDIO controller state-machine bug).

Verified fact that matters: in the old kernel it is enabled **only for SDC4 (WiFi SDIO)** — `CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED=y` — and **explicitly OFF for SDC1 (eMMC)** — `# CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED is not set`. The 6.18 DT mirrors this exactly: `qcom,dummy52-required` is on `sdcc4` and **absent on `sdcc1`**.

**Consequence:** the DVT/EVT eMMC boot blocker (audit Gap #2) is **not** a missing CMD52/dummy52 — sdcc1 never used it on any revision. Do not chase dummy52 for eMMC; the impl-doc CMD8-sequencing hypothesis stands.

---

## Quirk table

| # | Quirk / errata | Old source | What & why | 6.18 status | Sev |
|---|---|---|---|---|---|
| Q1 | **CMD52 errata (`dummy52_required`)** | `msm_sdcc.c:888,1046`; `board-tenderloin.c:5482` (SDC4) / `:5468` (SDC1, `#ifdef` OFF) | Dummy CMD52 around data cmds works around SDCC SDIO controller bug. SDIO-only; eMMC excluded. | `qcom,dummy52-required` on `sdcc4` ✅, correctly absent on `sdcc1` ✅ | ✅ |
| Q2 | **DFAB-sourced peripheral clock (`pclk_src_dfab=1`)** | `board-tenderloin.c:5467,5487` | SDC1/SDC4 pclk comes from Daytona Fabric; legacy **force-votes DFAB = 64 MHz** for the controller's lifetime. | Path present (`interconnects = <&daytona_fabric DFAB_MAS_SDC1 …>`) but **interconnect-derived DFAB rate at boot may be < 64 MHz** → the documented `CMDTIMEOUT cmd52/cmd5/cmd55` init chain (see `mmci-legacy-deep-dive.md` §5). | 🟡 |
| Q3 | **eMMC clock cap 48 MHz** | `board-tenderloin.c:5465` `.msmsdcc_fmax=48000000` | SanDisk SEM32G reports 52 MHz HS but DATACRCFAILs above 48 MHz. | `max-frequency = <48000000>` on `sdcc1`, fully commented ✅ | ✅ |
| Q4 | **Narrow OCR window 2.7–2.9 V** | `board-tenderloin.c:5456,5477` `.ocr_mask` | eMMC/SDIO OCR negotiation restricted to 2.7–2.9 V. | Enforced implicitly by fixed `vmmc` regulators (`pm8901_l5` 2.85 V / `pm8901_l1`) — equivalent, not an explicit mask | ✅ |
| Q5 | **MPM SDIO wake (`cfg_mpm_sdiowakeup` → `MSM_MPM_PIN_SDC4_DAT1`)** | `board-tenderloin.c:5488`; `:5416` | Lets the WiFi chip wake the SoC from suspend via SDC4 DAT1 through the MPM. | **MPM node disabled** (audit Gap #3) ⇒ **WiFi cannot wake system from suspend.** Previously-undocumented concrete impact of the MPM gap. | 🟥 |
| Q6 | **WLAN/codec 1.8 V rail forced HPM-always-on (`board_setup_S3A_1V8`); SDC4 vdd op-pwr-mode + lpm/hpm load** | `board-tenderloin.c:6363,6379,5515+` | `8058_s3` pinned HPM always-on; SDC4 vdd has `op_pwr_mode_sup=1`, explicit lpm/hpm µA. | `pm8058_s3` now has `regulator-system-load = <100000>` (commit `27b7108047ba`, 2026-05-20). Gap #1 RESOLVED. **Caveat:** must remain narrow to `pm8058_s3` — applying the same property to other WiFi rails bootloops the device (`feedback_regulator_system_load_bootloop.md`). | ✅ |
| Q7 | **MAX8903B `USUS` pin polarity reversal on DVT+** | `board-tenderloin.c:6667-6671` (`board_type > TOPAZ_EVT1 && != TOPAZ_3G_PROTO`) | DVT and later WiFi boards invert the charger USB-suspend pin polarity. | common.dtsi `usus-gpios = <&tlmm 33 GPIO_ACTIVE_LOW>` = the DVT/PVT (reversed) polarity ✅ for shipping target. EVT1/pre-DVT would need active-high (same edge-case class as the DVT-eMMC issue). | ✅ (PVT/DVT) |
| Q8 | **GSBI3 sensor-I2C bus recovery (`board_i2c_recover`, 36 SCL cycles + START/STOP, GPIO43/44)** | `board-tenderloin.c:1855-1975`; `I2C_RECOVER_CLOCK_CYCLES=36` | If a sensor (mpu3050/isl29023) hangs SDA low, bit-bang 36 clocks to free the bus. | `gsbi3_i2c` has pinctrl but **no `scl-gpios`/`sda-gpios` / i2c-recovery declared** → mainline i2c-qup cannot auto-recover a wedged sensor bus. **Genuine missed quirk.** | 🟥 |
| Q9 | **USB ULPI PHY tuning (pre-emphasis 20% / hsdrvslope 0x05 / CDR-autoreset & SE1-gating disable, ULPI regs 0x32/0x36)** | `board-tenderloin.c:1133,1136` | Signal-integrity tuning written to ULPI vendor regs 0x30-0x3F. | **Not applied** — common.dtsi has an explicit TODO: mainline `qcom-usb-hs-phy` only exposes vendor regs ≥0x80 via `qcom,init-seq`; legacy targeted 0x32/0x36. Running on PHY defaults. | 🟡 |
| Q10 | **gpio_keys debounce 20 ms / core-navi 500 ms** | `board-tenderloin.c:3194,3206,3215,3229` | 20 ms for vol/keys; **500 ms** long-debounce on core-navi/center to reject accidental home presses. | WiFi: volume via PMIC keypad (no gpio-keys). 3G gpio-keys uses uniform `debounce-interval = <15>` — the **500 ms core-navi long-debounce is not reproduced**. Minor. | 🟡 |
| Q11 | **A6 Spy-Bi-Wire bit-bang timing** | `a6_sbw_impl_tenderloin.c/.h` | Proprietary SBW (JTAG-like) timing to talk to the Palm A6 battery MCU. | Folded into the custom `palm,a6-battery` driver ✅ | ✅ |
| Q12 | **Z180 2D: `set_grp2d_async=NULL` run SYNC @192 MHz; `msm8x60_check_2d_hardware` CPU-rev detect (8660 v1 lacks 2D)** | `devices-msm8x60.c:750,783` | HW workaround for the Z180 2D core on early 8660 silicon. | Z180 disabled in mainline path (audit §5 non-gap) | ➖ |
| Q13 | **AVS SW interrupts not installed (`FIXME` AVS_SVICINT / AVS_SVICINTSWDONE)** | `devices-msm8x60.c:182` | Adaptive Voltage Scaling SW IRQs deliberately not wired. | Mainline uses RPM/SAW voltage mgmt; AVS not a concept here | ➖ |
| Q14 | **LCDC power-seq ordering TODO (gpios/vreg ref-count "should be requested earlier")** | `board-tenderloin.c:5719,5741` | Display enable/reset GPIO + LVDS regulator ordering fragility. | DRM `panel-lvds` owns enable(tlmm62)/reset(tlmm63) + `lvds-vccs-3p3v` sequencing; display verified working ✅ | ✅ |
| Q15 | **PMIC GPIO debounce for USB-ID detect** ("work to allow debounce on gpio") | `board-tenderloin.c:801` | Debounce on the USB ID / PMIC line. | Handled by extcon/PMIC-gpio + chipidea; USB OTG/ID working ✅ | ✅ |

---

## Net-new items for the audit gap list

Two quirks are genuine misses not already in `BOARDFILE-DEFCONFIG-AUDIT`'s gap table:

- **Gap #11 (NEW) — GSBI3 sensor-I2C has no bus-recovery (Q8).** Add `pinctrl` "recovery" state + `scl-gpios`/`sda-gpios` (or `qcom,` recovery) on `gsbi3_i2c` so a wedged mpu3050/isl29023 can be cleared as the legacy `board_i2c_recover()` did. **Severity: MEDIUM-LOW** (only bites on a sensor lockup, but then it's unrecoverable without reboot).
- **Gap #12 (NEW) — WiFi cannot wake from suspend (Q5).** The `cfg_mpm_sdiowakeup` (SDC4 DAT1 → MPM) path is dead because the MPM node is disabled. This is a **specific, testable consequence of Gap #3** (MPM) that should be recorded as an explicit acceptance criterion for the MPM platform-driver work: *after MPM lands, verify WoWLAN/SDIO DAT1 wake from suspend.* **Severity: tied to Gap #3.**

Plus two open risks to track (not new gaps, but under-documented):

- **Q2 — DFAB rate adequacy.** ~~Open risk.~~ **RESOLVED (2026-05-21
  measurement).** Captured on a running tenderloin (kernel
  `g339219fa4743`):

  ```
  $ cat /sys/kernel/debug/clk/daytona_clk/clk_rate
  384000000                                    # 384 MHz (6x legacy 64 MHz min)

  $ cat /sys/kernel/debug/interconnect/interconnect_summary
  dfab_mas_sdc1      0      512000             # 512 MB/s peak (=> >=64 MHz @ 64-bit)
  dfab_mas_sdc4      0      512000             # 512 MB/s peak
  ```

  The mainline interconnect framework correctly sets both the DFAB
  clock rate AND the per-port bandwidth votes. Operating well above
  legacy minimums. Q2 is therefore **not** the cause of the
  `cmd52/cmd5/cmd55` CMDTIMEOUT family — if those errors still appear
  they have a different root cause (likely the eMMC-DVT firmware-9.0
  CMD8 sequencing per `impl-emmc-dvt-firmware.md`).
- **Q9 — USB PHY tuning unported.** Already TODO'd in DT; low priority while USB is stable, but document as a known deferred quirk (revisit if USB enumeration/signal issues appear on marginal cables/hubs).

---

## One-paragraph summary

Of 15 board-file quirks, **10 are correctly carried** (incl. the CMD52/dummy52 errata — properly SDIO-only — and the WLAN rail load Q6 = Gap #1 just landed as `27b7108047ba` on 2026-05-20), **3 are N/A** (Z180/AVS — not in the mainline path), and **2 are real problems remaining**: the lost SDIO-wake (Q5 → ties to Gap #3) and the missing sensor-I2C bus recovery (Q8 = new Gap #11). The DFAB clock-source quirk (Q2) is the named mechanism behind the `cmd52/cmd5/cmd55` timeout family and should be measured on hardware. The USB PHY tuning (Q9) and core-navi long-debounce (Q10) are minor, already-deferred items.
