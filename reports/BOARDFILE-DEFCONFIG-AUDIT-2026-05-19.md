# Board-File & Defconfig Audit — webOS 2.6.35 → Mainline 6.18 (Tenderloin)

**Date:** 2026-05-19
**Correction (2026-05-19 later same day):**
- **Gap #1** ("WiFi rail / `pm8058_s3` HPM") — **RESOLVED before this audit was finalised.** Commit `27b7108047ba` ("ARM: dts: qcom: tenderloin: Force pm8058_s3 SMPS into HPM via system-load", 2026-05-20 05:26) added `regulator-system-load = <100000>` to `pm8058_s3`, matching the legacy `regulator_set_optimum_mode(vreg_S3A_1V8, 100000)` call. Audit diagnosis was correct; fix landed earlier the same day in a parallel session. Rows below struck for traceability.
- **Gap #7** ("RTC writable — no `allow-set-time` in DT") — **false positive.** The property is present at `arch/arm/boot/dts/qcom/pm8058.dtsi:138`. The original audit grep searched only `common.dtsi` and `topaz.dts` and missed the included `pm8058.dtsi`. Marked RESOLVED below; row retained for traceability. Lesson: DT-property checks must walk the include chain, not just leaf files.
**Inputs compared:**
- OLD: `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c` (6,732 lines) + companions (`devices-tenderloin.c`, `gpiomux-tenderloin.[ch]`, `a6_sbw_impl_tenderloin.h`)
- OLD: `webos-linux-kernel-touchpad/arch/arm/configs/tenderloin_defconfig` (2,785 lines)
- NEW: `linux-6.18-tenderloin` DTS set (`qcom-msm8660.dtsi`, `qcom-apq8060-tenderloin-common.dtsi`, `qcom-apq8060-topaz.dts`, `-topaz-3g.dts`, `-tenderloin-lpaif.dtsi`, `-pre-dvt.dtso`, `pm8058.dtsi`)
- NEW: `linux-6.18-tenderloin/arch/arm/configs/tenderloin_defconfig` (457 lines)
- Existing port docs (Jan–May 2026 reports, `context/impl/*`) used for status corroboration only.
- **Companion:** `HARDWARE-QUIRKS-INVENTORY-2026-05-19.md` — the non-device workarounds/errata (incl. the CMD52/`dummy52` errata) extracted from the board file *and the vendor drivers it configures*.

**Verdict in one line:** The hardware port is **substantially complete and correct**. Every real TouchPad (Topaz WiFi) hardware block from the old board file is represented in the 6.18 DT. The remaining gaps are **not missing devices** — they are (a) known functional regressions/blockers already in the issue list, (b) a small set of genuinely-missing low-risk capabilities, and (c) doc/process drift. A large fraction of apparent "missing" items are **correctly absent** and listed explicitly in §5 to prevent wasted effort.

---

## 1. Executive Summary — what actually matters

| # | Finding | Severity | Status |
|---|---------|----------|--------|
| 1 | ~~**WLAN 1.8 V rail (`pm8058_s3`) not load-managed.**~~ | ~~**HIGH**~~ | **RESOLVED by `27b7108047ba`** — `regulator-system-load = <100000>` added to `pm8058_s3`, matching legacy `regulator_set_optimum_mode(..., 100000)`. |
| 2 | DVT/EVT eMMC (Samsung fw rev 9.0) enumerates at 0 B — boot blocker on pre-PVT units | **HIGH** | **PARTIAL** — `ac08169e898f` reverted `MMC_CAP_BUS_WIDTH_TEST` (Samsung SEM32G unsupported), but core CMD8/EXT_CSD sequence fix (per `impl-emmc-dvt-firmware.md` 5-step plan) NOT executed. Workaround: cold-boot via webOS first. Affects production PRV=0x90 units. |
| 3 | MPM node disabled → deep cpuidle (PM-2) blocked; suspend-to-RAM never tested | **HIGH (power)** | **MPM RESOLVED** (driver landed `993a638936e4`, USB1_HS routes through MPM). PM-2 (cpuidle SPC/PC) blocked by TZ TERMINATE_PC no-op — requires legacy assembly port (~15-20 hr estimated, see `impl-spm-init.md`). |
| 4 | VIDC video codec hangs at firmware boot (old `MSM_VIDC` worked) | **MEDIUM** | Known, ongoing investigation |
| 5 | GPU Adreno-220 "period-8" — only 1/8 frames render correctly | **MEDIUM** | Shipped w/ workaround |
| 6 | ~~PMIC thermal protection absent~~ | ~~**MEDIUM**~~ | **RESOLVED** — full pmic-thermal kit landed; both PM8058 + PM8901 zones live, critical-trip→poweroff verified at 148 µs latency (impl-pmic-thermal.md). |
| 7 | ~~PMIC RTC is read-only: no `allow-set-time` in DT~~ | ~~**LOW-MED**~~ | **RESOLVED — false positive.** `allow-set-time` IS present (`pm8058.dtsi:138`). Audit grep was incomplete (didn't walk include chain). |
| 8 | SoC watchdog not enabled (old `MSM_WATCHDOG=y`; new no `CONFIG_QCOM_WDT`) | **LOW** | **PARTIAL** — driver + DT + defconfig landed (impl-soc-watchdog.md T-001/T-008/T-010 done). T-011 (5 cold boots + 30-min pet) + T-021 (HW reset + pstore preservation) HW tests pending. |
| 9 | Upstream patch plan references wrong driver names (`vx6953`/`cyttsp`) vs as-built (`mt9m113`/`cy8ctma395`) | **LOW (doc)** | NOT FIXED — doc drift remains |
| 10 | ~~Cavekit context tree covers only `spm-init`; port knowledge lives in ~150 ad-hoc reports~~ | ~~**LOW (process)**~~ | **RESOLVED** — 9 cavekits now exist (pmic-thermal, sensor-i2c-recovery, soc-watchdog, spm-init, usb-charger-detection, usb-otg-host, usb-phy-tuning, wifi-suspend-wake, plus overview). |
| 11 | **GSBI3 sensor-I2C has no bus-recovery** — old `board_i2c_recover()` bit-banged 36 SCL cycles to clear a wedged mpu3050/isl29023; new `gsbi3_i2c` declares no `scl-gpios`/`sda-gpios` recovery | **MED-LOW** | **PARTIAL** — driver + DT recovery wired (T-002/T-003 done, impl-sensor-i2c-recovery.md). T-012/T-013 HW stress tests pending. |
| 12 | **WiFi can't wake from suspend** — old SDC4 `cfg_mpm_sdiowakeup` (DAT1→MPM) is dead because the MPM node is disabled | tied to #3 | **PARTIAL** — R1 (MPM functional) DONE, R2 (SDC4 wakeup-source DT) DONE (`wakeup-source` on sdcc4, mmc1 in `/sys/kernel/debug/wakeup_sources`). R3 (WoWLAN config) + R4 (end-to-end wake) blocked on ath6kl WIP. |

Items 6, 7, 8, **11** are the previously-untracked genuine gaps this audit surfaced; #12 is a concrete consequence of #3. Everything else is either on the issue list (1–5) or correctly absent (§5).

**CMD52 errata note:** the SDCC `dummy52_required` (CMD52) errata is carried **correctly** — enabled on `sdcc4` (WiFi SDIO) and correctly absent on `sdcc1` (eMMC), exactly matching the old kernel (`CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED=y` / SDC1 unset). This **rules out a missing dummy52 as the cause of the DVT eMMC blocker (Gap #2)**. The related `cmd52/cmd5/cmd55` CMDTIMEOUT family instead traces to the DFAB clock-source quirk — see quirk inventory Q2. Full quirk treatment: `HARDWARE-QUIRKS-INVENTORY-2026-05-19.md`.

---

## 2. Subsystem coverage matrix: board-tenderloin.c → 6.18 DT

Legend: ✅ ported & working · 🟡 ported, partial/fragile · 🟥 ported but broken/blocked · ⛔ deliberately dropped (no HW) · ➖ correctly absent (never real HW)

### 2.1 SoC core

| Old block | New DT / driver | State | Notes |
|---|---|---|---|
| machine `TENDERLOIN`, `msm8x60_*` | `hp,topaz`/`qcom,apq8060`/`qcom,msm8660`, DT-driven | ✅ | Board-file era → DT |
| SMD/RPM/`msm_xo`/SMSM | `qcom,smem`, `qcom,rpm-msm8660`, `rpmcc`, `qcom,smsm` | ✅ | |
| ADM DMA adm0/adm1 | `qcom,adm` ×2 (`CONFIG_QCOM_ADM=y`) | ✅ | New also adds BAM DMA |
| clocks 8x60 + acpuclock + MDP/PIXEL fixup | `gcc`/`mmcc`/`lcc`/`rpmcc` (`MSM_GCC/LCC/MMCC_8660`) | ✅ | Upstream Series 1 |
| bus-scaling fabrics ×5 | `interconnect` apps/system/mmss/daytona (`INTERCONNECT_QCOM_MSM8660`) | ✅ | |
| SPM/SAW per-core + saw-regulator | `saw0/saw1` + `qcom,...saw2-v1.1-cpu`, `QCOM_SPM` | ✅ | SPM init HW-verified |
| `msm_pm_data` C-states / cpuidle / RPM sleep | `cpu_spc` wired; **`cpu_pc` defined but NOT wired**; MPM node commented out | 🟥 | **Gap #3** — deep idle/suspend blocked on MPM |
| IOMMU (not in old board file) | 11/12 IOMMUs `okay`; `gpu_iommu` deliberately off | ✅ | New is *better* than old |
| EBI2 CS2 (SMSC LAN9221), CS3/4 (3G ISP1763) | `ebi2` disabled on WiFi; enabled only on 3G for ISP1763 | ➖/⛔ | LAN9221 = MSM8660 ref-board vestige, no HW on TouchPad (§5) |
| GSBI3/4/7/8/9/10 i2c, GSBI1 SPI, GSBI6 UART | `gsbi3/4/7/8/10/12` nodes; **GSBI9 i2c & GSBI1 SPI not present/used** | ✅ | GSBI9 had no devices in old either (§5) |
| video codec `msm_vidc` | `qcom,msm8660-vidc` enabled, **hangs at fw boot** | 🟥 | **Gap #4** |
| GPU kgsl/Adreno-220 | `qcom,adreno-220` (`DRM_MSM`) | 🟡 | **Gap #5** period-8 |
| `msm_rotator`, `msm_vpe` | `qcom,msm8660-rotator`/`-vpe` present but `disabled` | ⛔ | Legacy MM accel, not needed (§5) |

### 2.2 Power / regulators

| Old block | New DT / driver | State | Notes |
|---|---|---|---|
| PM8058 core (ssbi1, 0x55, INT gpio88) + gpio/mpp/pwrkey/vib/pwm/rtc/tm/upl/charger | `qcom,pm8058` on `ssbi@500000`, IRQ tlmm 88; gpio/mpp/pwrkey/rtc/keypad/pwm/xoadc subnodes | ✅ | `MFD_PM8XXX`, `PINCTRL_QCOM_SSBI_PMIC` |
| PM8901 (ssbi2, 0x55, INT gpio91) + mpp/tm/regulators | `qcom,pm8901` on `ssbi@C00000`, IRQ tlmm 91; mpps | ✅ | |
| RPM regulators PM8058 L0–L25/S0–S4/LVS/NCP, PM8901 | `qcom,rpm-pm8058-regulators` / `-pm8901-regulators` (`REGULATOR_QCOM_RPM`) | ✅ | |
| `board_setup_S3A_1V8` — **S3 forced HPM always-on** for WLAN/codec | `pm8058_s3` now has `regulator-system-load = <100000>` (commit `27b7108047ba`) | ✅ | **Gap #1 RESOLVED** — fix matches legacy `regulator_set_optimum_mode(..., 100000)` |
| `power_up_gyroscope()` — L15/LVS3 (+L12 DVT) forced at boot | gyro rails `pm8058_l15`/`pm8901_lvs3` **not in always-on set** | 🟡 | Secondary load-mgmt risk (gyro under PM/suspend) |
| fixed reg `vdd50_boost` (5 V, gpio102) | `vdd50_boost` `reg-fixed-voltage` gpio102 | ✅ | |
| `pm8058-tm` + `pm8901-tm` thermal alarms (`THERMAL_PM8901=y`) | **no PMIC thermal driver, no `thermal-zones`** | 🟥 | **Gap #6 — genuine, untracked** |
| `pm8058-rtc` (+ `RTC_PM8058_WRITE_ENABLE`) | `qcom,pm8058-rtc` (`RTC_DRV_PM8XXX`) — **no `allow-set-time`** | 🟡 | **Gap #7 — RTC read-only** |
| `pm8058-pwrkey` | `qcom,pm8058-pwrkey` (`INPUT_PMIC8XXX_PWRKEY`) | ✅ | |

### 2.3 Battery / charging

| Old | New | State | Notes |
|---|---|---|---|
| MAX8903B (`max8903b_chg`, per-rev ISET tables, USUS polarity fixups) | `maxim,max8903` (`CHARGER_MAX8903`), GPIOs incl. 3G overrides | ✅ | |
| Dual Palm A6 fuel-gauge @ i2c 0x31/0x32 GSBI8, SBW bit-bang | `palm,a6-battery` ×2 @0x31/0x32 gsbi8 (`BATTERY_PALM_A6`); pre-DVT/DVT IRQ overlays present | ✅ | Custom driver ported, working |
| PM8058 internal charger (~20 IRQs), `msm-charger`/`msm-battery` | folded into PMIC/`max8903` + power-supply core | ✅ | Vendor framework correctly dropped |

### 2.4 Storage

| Old | New | State | Notes |
|---|---|---|---|
| SDCC1 eMMC 8-bit (gpio159–168, vdd `8901_l5`, vccq `8901_lvs0`) | `sdcc1` `arm,pl18x` mmc0, 8-bit non-removable, vmmc `pm8901_l5`, vqmmc `pm8901_lvs0` (`MMC_SDHCI_MSM`) | 🟥 on DVT/EVT, ✅ on PVT | **Gap #2** — Samsung fw 9.0 CMD8 sequencing |
| SDCC4 WiFi SDIO (vdd `8058_s3`, MPM SDC4_DAT1 wake) | `sdcc4` 4-bit SDIO + `ath6kl_pwrseq` | 🟥 | Tied to **Gap #1** (rail) and SDIO timeout |
| SDCC2/3/5 unused | `disabled` | ✅ | Matches old |

### 2.5 USB

| Old | New | State | Notes |
|---|---|---|---|
| `msm_otg` + ULPI 45nm PHY + peripheral gadget + EHCI host | `qcom,ci-hdrc` peripheral + `usb_hs1_phy` (`USB_CHIPIDEA_UDC`, `PHY_QCOM_USB_HS`) | ✅ | RNDIS/ECM working; PHY tuning values noted but not applied |
| ISP1763A external host (3G only) | `nxp,usb-isp1763` 3G-only, `disabled` (boot-hang diag) | ⛔ | Out of scope for WiFi device |
| `USB_ROCKHOPPER` composite gadget | configfs composite (ECM/NCM/ACM/MASS_STORAGE) | ✅ | Modernized |

### 2.6 Display / GPU

| Old | New | State | Notes |
|---|---|---|---|
| `msm_fb` + `lcdc_lg_xga` 1024×768 LVDS, LCD_PWR_EN 62, LVDS_SHDN 63 | `qcom,mdp4` + `panel-lvds` (enable tlmm62, reset tlmm63), rotation 180 (`DRM_MSM_MDP4`) | ✅ | Blue-line & vblank bugs fixed |
| backlight 2×LPG (PMIC gpio23 PWM / gpio24 EN) | `pwm-backlight` + `qcom,pm8058-lpg` | ✅ | |
| MDP/LCDC/MIPI-DSI/DTV(HDMI)/ATV TV-out registered | DSI/HDMI/DTV/ATV/Z180 all `disabled` | ⛔ | TouchPad has no HDMI/DSI/TV connector (§5) |
| GPU Adreno-220 + 2D (Z180) | `qcom,adreno-220`; `z180` disabled | 🟡 | **Gap #5**; 2D drop is acceptable (§5) |

### 2.7 Touchscreen / input

| Old | New | State | Notes |
|---|---|---|---|
| Dual-IC: Atmel mXT1386 @i2c 0x4C (raw) → Cypress CY8CTMA395 (UART, GSBI10) | `cypress,cy8ctma395-ts` serdev on gsbi10_serial @4 Mbps; i2c cyttsp node commented out | 🟡 | Architecture matches reality; single-touch verified, multitouch unconfirmed |
| `gpio-keys` (vol±, core-navi/center, UIM 3G) | WiFi: `pm8058_keypad` + `pwrkey`; **`gpio-keys` only added in 3G DTS** | ✅ | Volume on WiFi via PMIC keypad matrix |
| headset/mic detect (gpio57 via WM8958, JACK_DET 67) | WM8958 micdet tlmm57 + lpaif `hp_det_pins` gpio67 | ✅ | |
| `hres_counter`, `nduid`, diag "holy trinity", `user-pins`, `bt_power` | vendor glue — no mainline equivalent (DT/gpio handles) | ➖ | Correctly obsolete |

### 2.8 Audio

| Old | New | State | Notes |
|---|---|---|---|
| WM8958 codec @i2c 0x1A GSBI7, LDO-EN gpio66/108 | `wlf,wm8958` on gsbi7_i2c @0x1a, micdet tlmm57, ldo2ena tlmm108, aud-ldo1-hog tlmm66 | ✅ | |
| `MSM8X60_AUDIO` Q6/DSP path, MI2S/AUX-PCM | **direct LPAIF path** (`SND_SOC_APQ8060_LPAIF`); Q6/APR deliberately disabled | ✅ | Speaker+HP+jack-detect verified 2026-05-15 |
| speaker amp (WM8958 gpio1) | external Class-D via WM8958 internal GPIO1 (`wlf,gpio-cfg 0x0041`) | ✅ | |

### 2.9 Sensors

| Old | New | State | Notes |
|---|---|---|---|
| Gyro INT gpio125 (`user-pins`), no i2c chip in board file | `invensense,mpu3050` @0x68 gsbi3, irq tlmm125 (`MPU3050_I2C`) | ✅ | New *adds* proper IIO driver (old defconfig had none) |
| GSENS_INT gpio124 **#define'd but never in any pin_table** | `st,lsm303dlh-accel`@0x18 & `-magn`@0x1e **disabled on Topaz WiFi** | ➖ | **Confirmed correct** — old never wired accel/magn on WiFi; all accel/mag drivers `is not set` in old defconfig (§5) |
| ALS / proximity (PMIC gpio35/39, no chip registered) | `isil,isl29023`@0x44 gsbi3 (`SENSORS_ISL29018`); **no proximity** | ✅/➖ | Proximity never existed on TouchPad (§5) |

### 2.10 Connectivity

| Old | New | State | Notes |
|---|---|---|---|
| WiFi Atheros AR6003 SDIO (SDCC4; "WCN1314" comment is wrong) | `atheros,ath6kl` on sdcc4 + `ath6kl_pwrseq` (`ATH6KL_SDIO=m`, cfg80211/mac80211) | 🟥 | **Gap #1** — CMD53 -110 in full userspace |
| BT BCM4329 UART HCI on GSBI6 (`CONFIG_BT` **off** in old) | `palm,bcm4329-bcsp` on gsbi6_serial (`BT=m`, HCIUART BCSP) | ✅ | New properly enables BT (old relied on userspace) |
| GPS (none on shipping HW) | `brcm,bcm4751` present but gsbi5 disabled on both variants | ➖ | No GPS HW on Topaz (§5) |
| NFC | none | ➖ | No NFC HW |

### 2.11 Camera

| Old | New | State | Notes |
|---|---|---|---|
| Front Aptina MT9M113 @i2c 0x78(8-bit) GSBI4, RST106/PWDN107 | `aptina,mt9m113`@0x3c(7-bit ⇔ 0x78) gsbi4, reset tlmm106/pwdn tlmm107 | 🟡 | Address consistent; driver "verified vs webOS" but fragile |
| CSI/VFE31, Gemini JPEG, VPE | `qcom,msm8660-camss` + `-gemini` enabled; `vpe` disabled | 🟡 | VFE31 RDI hard-won; no rear cam (matches old) |

### 2.12 Misc / 3G

| Old | New | State | Notes |
|---|---|---|---|
| LM8502 LED/lighting @i2c GSBI8 (core-navi L/R, flash/torch), EN121/INT128 | `ti,lm8502`@0x33 gsbi8 (`LEDS_LM8502`), navi_left/right | ✅ | |
| `pm8058-vib` / LM8502 vib | `gpio-vibrator` enable tlmm79, vcc `pm8058_l5` (`INPUT_PM8XXX_VIBRATOR`) | ✅ | |
| PMEM regions (smi/adsp/audio/...) | `reserved-memory` + CMA + dmabuf-heaps | ✅ | Modernized |
| 3G: `mdmgpio`, ISP1763, UIM/SIM, reboot notifier | `motorola,mdm6600-simple` + isp1763 in 3G DTS, `disabled` | ⛔ | 3G untested, out of scope for WiFi device |

---

## 3. Defconfig delta (2.6.35 → 6.18) — only the real gaps

Most old `CONFIG_MSM_*`/`FB_MSM_*`/`PMIC8058*` symbols map cleanly to renamed mainline equivalents (full table in working notes). After accounting for renames and the **confirmed boot-to-userspace** (which disproves the theoretical rootfs gaps), the residual defconfig gaps are:

| Old CONFIG | New status | Real gap? | Action |
|---|---|---|---|
| `THERMAL_PM8901=y` | no PMIC thermal driver | **YES (Gap #6)** | Add a PM8058/8901 temp-alarm path or `thermal-zones` via xoadc die-temp; note SSBI PMIC ⇒ `QCOM_SPMI_TEMP_ALARM` does **not** apply |
| ~~`RTC_PM8058_WRITE_ENABLE=y`~~ | ~~no `allow-set-time` in DT~~ | ~~YES~~ | **RESOLVED — false positive.** Property already at `pm8058.dtsi:138`. No action needed; verify hwclock persistence at next test pass. |
| `MSM_WATCHDOG=y` | no `CONFIG_QCOM_WDT`/`WATCHDOG` | **YES (Gap #8, low)** | Enable `CONFIG_WATCHDOG`+`CONFIG_QCOM_WDT` only if bootloader arms SoC WDT (tenderloin usually does not) |
| `# CONFIG_SQUASHFS is not set` | still unset | **NO** | Disproven — device boots full LuneOS userspace (ext-based rootfs). Add only if a squashfs recovery/overlay image is introduced |
| `CONFIG_BLK_DEV_RAM=y` | `is not set` | **NO** | Disproven — boot uses external cpio initramfs (`initramfs-uImage.bin`); cpio needs no `BLK_DEV_RAM` |
| `GPIO_SX150X=y` | not enabled | **NO** | Disproven — no SX150x expander anywhere in old board file/headers (§5) |
| `IIO_ST_ACCEL_3AXIS=y` (new-only) vs no accel in old | node disabled on Topaz | **NO** | Harmless dead config; no accel HW on WiFi (§5) |
| `MSM_KGSL_2D=y` (Z180 2D) | `DRM_MSM_Z180` off | **NO** | No mainline userspace uses Z180; 3D via DRM_MSM |
| netfilter/NAT/PPP/bridge/SLIP, USB-host webcam/storage quirks | mostly dropped | **NO (software)** | Add iptables/NAT modules only if LuneOS tethering/firewall is a product requirement |
| RPC-router/RMNET/PMEM/AMSS/SMD-TTY | dropped | **NO** | Obsolete vendor cruft; correctly gone |

Net new-tree improvements (not gaps): full IIO sensor stack, BT stack, cfg80211/mac80211, CMA/dmabuf-heaps, pstore/ramoops, FS encryption, QCE crypto, ADM+BAM DMA, interconnect, OCMEM, binder, NFS/CIFS/overlayfs.

---

## 4. Prioritized action list

1. ~~**WiFi rail (Gap #1) — highest leverage.**~~ **DONE — landed as `27b7108047ba` on 2026-05-20 05:26**, adding `regulator-system-load = <100000>` to `pm8058_s3` (matches legacy `regulator_set_optimum_mode(..., 100000)`). Note: per `feedback_regulator_system_load_bootloop.md`, applying the same property to other WiFi-adjacent regulators bootloops the device — the fix must remain narrow to `pm8058_s3`. Gyro rails `pm8058_l15`/`pm8901_lvs3` are still left non-always-on and remain a secondary item to audit if gyro misbehaves under PM/suspend.
2. **eMMC DVT (Gap #2):** execute the 5-step plan in `context/impl/impl-emmc-dvt-firmware.md` (long DATATIMER → pre-CMD8 CMD13 → doubled CMD0 → 200 kHz init → ADM trace). Cross-device test already isolated it to mmci-pl18x vs Samsung fw-9.0 CMD8 sequencing.
3. **MPM (Gap #3):** convert MPM to a `platform_driver` (or a minimal `irq-msm8660-wakeup.c`) per `impl-mpm-boot-hang.md` recommendation; unblocks PM-2 cpuidle and enables the first-ever suspend-to-RAM test.
4. **PMIC thermal (Gap #6, new):** add die-temp monitoring. There is no mainline `pm8058-tm` equivalent and the PMICs are SSBI (so `QCOM_SPMI_TEMP_ALARM` is inapplicable) — implement via `pm8058_xoadc` die-temp channel + a `thermal-zones`/`iio-hwmon` trip, or a small pm8xxx tm shim. Low effort, removes a silent over-temp risk.
5. ~~**RTC writable (Gap #7, new):** add `allow-set-time;` to `rtc@1e8` (one-line DT override) so the system clock survives full power-off.~~ **WITHDRAWN — false positive (see correction at top); property already present.**
6. **VIDC (Gap #4) / GPU period-8 (Gap #5):** continue existing investigations; both already have extensive dead-end logs — see §5/§11 of `STATUS-FOR-FRESH-AI-CONSULTATION.md` before proposing register-level fixes (register-write fixes for period-8 are proven futile).
7. **Sensor-I2C recovery (Gap #11, new):** add a `recovery`/`gpio` pinctrl state + `scl-gpios`/`sda-gpios` on `gsbi3_i2c` (or `qcom,` recovery) to replicate `board_i2c_recover()` (GPIO43/44, 36 cycles). One-time DT change; prevents an unrecoverable sensor-bus lockup.
8. **DFAB rate measurement (quirk Q2 — cmd52/cmd5/cmd55 family):** on a working PVT unit, dump the interconnect-resolved DFAB rate while SDCC is active and confirm ≥64 MHz (legacy force-voted 64 MHz). If short, add an explicit interconnect bandwidth floor on the `sdcc1`/`sdcc4` "sdc" path. Directly de-risks Gaps #1 and #2.
9. **Doc hygiene (Gaps #9–10):** reconcile `UPSTREAM_PATCH_PLAN.md` (vx6953/cyttsp) with as-built (`mt9m113`/`cy8ctma395`); refresh the stale 2026-01-31 status report; optionally kit the open domains (wifi-rail, emmc-dvt, mpm, pmic-thermal, i2c-recovery) into `context/kits/` so the iteration loop can act on them.

> Note for the MPM work (Gap #3): add an explicit acceptance criterion — *after MPM lands, verify WiFi/SDIO DAT1 wake-from-suspend* (quirk Q5). The old `cfg_mpm_sdiowakeup` path is currently dead.

---

## 5. Non-gaps — do NOT spend effort here (verified correctly absent)

These look "missing" against the old board file/defconfig but are **correct omissions**, verified in this audit:

- **Accelerometer & magnetometer** — `TENDERLOIN_GSENS_INT` (gpio124) is `#define`'d in `gpiomux-tenderloin.h` but **never placed in any `pin_table[]`** (only `GYRO_INT_PIN` is). No accel/mag/ALS `i2c_board_info` exists anywhere in the old tree, and the old defconfig has `# CONFIG_BOSCH_BMA150/AK8975/ISL29003 is not set`. The new `topaz.dts` disabling `lsm303dlh_accel`/`lsm303dlh_magn` with an explanatory comment is **correct**. (Sanity-check only if product needs auto-rotate — webOS did orientation via the gyro/DSPS, not a discrete accel, on the WiFi unit.)
- **SX150x I2C GPIO expander** — zero references in old board file or headers. Not on tenderloin.
- **SMSC LAN9221 Ethernet (EBI2 CS2)** — appears only in EBI2 *timing comments*; it is the MSM8660 SURF/FFA reference-board NIC. No Ethernet on the TouchPad. New correctly enables EBI2 only on 3G for ISP1763.
- **Proximity sensor** — old set PMIC gpio35/39 "for proximity" but registered no chip; the TouchPad has no proximity sensor (new DT comment confirms).
- **GPS (BCM4751)** — present in common.dtsi but gsbi5 disabled on both Topaz WiFi and 3G; no GPS silicon on shipping HW.
- **GSBI9 I2C / GSBI1 SPI** — old registered the controllers but bound no devices; correctly absent.
- **HDMI / MIPI-DSI / DTV / ATV TV-out / Z180 2D / msm_rotator / VPE** — old registered the IP blocks generically; the TouchPad exposes none of these connectors and no mainline userspace needs Z180/rotator/VPE. Deliberate, correct drops.
- **SquashFS / BLK_DEV_RAM** — disproven by the confirmed full-userspace boot via external cpio initramfs.
- **Q6/APR DSP audio path** — intentionally replaced by direct LPAIF; audio is verified working end-to-end.
- **RPC-router / RMNET / PMEM / AMSS-versioning / SMD vendor TTYs / USER_PINS / MDMGPIO / hres_counter / nduid** — webOS-vendor cruft with no mainline role; DT/gpio/standard subsystems cover the real function.

---

## 6. Conclusion

The 6.18 port faithfully covers the **entire real Topaz-WiFi hardware set** described by `board-tenderloin.c`. No physical device is unaccounted for. The work remaining is concentrated in **five known hard problems** (WiFi rail/SDIO, DVT eMMC, MPM/suspend, VIDC, GPU period-8) plus **three small, genuinely-new gaps this audit adds to the tracker** (PMIC thermal, writable RTC, SoC watchdog). The single highest-value, lowest-cost lead is **Finding #1**: the board-file↔DT diff shows the old kernel force-pinned the WLAN/codec 1.8 V rail to HPM-always-on while the new DT leaves it load-unmanaged — a direct, testable explanation for the otherwise-mysterious userspace-only WiFi CMD53 -110 regression.
