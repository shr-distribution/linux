---
domain: usb-otg-host
created: "2026-05-21"
last_updated: "2026-05-21"
status: r1-investigation-done-implement
---

# Implementation: USB OTG Host Mode

Kit: `context/kits/cavekit-usb-otg-host.md`.
Build site: not yet in a tier (new kit, post-2026-05-19 batch).

## Status

**R1 DONE — decision: IMPLEMENT (DT-only changes).**
R2-R6: not started.

## R1: Hardware-Feasibility Investigation

### Schematic caveat

Available schematics are for the early Topaz **3G** and **Opal**
variants:

- `schematics/Schematics-Topaz3G-6050A2428701-MB-X01.20101012.pdf`
- `schematics/Schematics-Opal.6050A2430401.rX01.pdf`
- `schematics/Schematics-Opal_6050A2430401-mb_dvt_0704.pdf`

**No schematic available for production Topaz WiFi.** This kit's
critical questions were therefore cross-checked against the **live
running production device** (`uname -r =
6.18.0-luneos-gb338c2d8b9f3`) before committing to a design, in
addition to the schematic + legacy code references.

### Q1: GPIO 102 pinmux conflict

**Question:** Topaz3G schematic shows MSM TLMM ball AJ10 with alt
functions GPIO_102 / FM_I2S_SCK / MI2S_SCK.  Legacy
`gpiomux-tenderloin.c:874` configures `gpio = 102` as `MI2S_ACTIVE_CFG`
(i.e. audio MI2S serial clock).  Does production Topaz WiFi use GPIO
102 for audio?

**Live-device evidence** (`/sys/kernel/debug/gpio` on the running
kernel):

```
gpio101 : in  low  func0 2mA pull down
gpio102 : out high func0 16mA no pull        <-- driven HIGH, plain GPIO mode
gpio103 : in  high func0 10mA pull up
```

All three legacy "MI2S" pins (101/102/103) are in **func0 (plain
GPIO)** mode at runtime — none are in an I2S alt function. Production
audio (WM8994 codec) is routed through a different set of GPIOs:

```
gpio108 : out high func0 8mA no pull         <-- LRCLK after probe
gpio109 : out low  func1 8mA no pull         <-- I2S alt
gpio110 : out low  func1 8mA no pull         <-- I2S alt
```

This is consistent with `qcom-apq8060-tenderloin-common.dtsi:2032`
which defines `primary_i2s_pins` as a pinmux state on GPIO 108-110.

**Conclusion:** GPIO 102 is **not used by audio** on production
Topaz WiFi.  Legacy `gpiomux-tenderloin.c`'s MI2S assignment to
GPIO 102 was either obsolete (kept from earlier Topaz3G revisions)
or only applied to the 3G variant.

**Plus a major finding:** GPIO 102 is **already in use as the 5V
boost regulator enable** in our mainline DT (`vdd50_boost` at
`qcom-apq8060-tenderloin-common.dtsi:207-220`):

```dts
vdd50_boost: vdd50 {
    compatible = "regulator-fixed";
    regulator-name = "vdd50_boost";
    regulator-min-microvolt = <5000000>;
    regulator-max-microvolt = <5000000>;
    gpio = <&tlmm 102 GPIO_ACTIVE_HIGH>;
    enable-active-high;
    regulator-always-on;
    regulator-boot-on;
};
```

This explains the live state perfectly: `regulator-always-on` →
GPIO 102 driven HIGH at boot → 16mA drive strength (sized to drive a
boost regulator's EN pin) → boost is enabled → 5V available at the
MVS switch input.

### Q2: PM8901 MPP0 necessity

**Question:** Legacy `msm_hsusb_vbus_power()`
(`board-tenderloin.c:1040-1085`) enables two regulators in sequence:
`8901_mpp0` (PM8901 MPP **pin 0**, marked `active_high = 0` for
tenderloin per `board-tenderloin.c:4526`) and `8901_usb_otg` (PM8901
MVS).  Is the MPP0 step necessary on mainline?

**Analysis:**

- Legacy `MPP(PM8901_VREG_ID_MPP0, 0)` at `pmic8901-regulator.c:242`
  confirms `8901_mpp0` is **physical MPP pin 0** (the first MPP), not
  MPP1.
- The tenderloin-specific `active_high = 0` (vs default 1 for other
  boards) inverts the polarity: `regulator_enable("8901_mpp0")` →
  drive MPP0 LOW; `regulator_disable()` → drive MPP0 HIGH.
- This step is *only* called from `msm_hsusb_vbus_power()`, which is
  itself gated on `CONFIG_USB_EHCI_MSM` (host mode).  It has **no
  other consumer in the legacy tree**.
- The actual 5V boost on production Topaz WiFi is enabled by **TLMM
  GPIO 102** (per Q1), not by PM8901 MPP0.  The boost is
  `regulator-always-on` in mainline today — the 5V rail at the MVS
  input is permanently present.
- The only remaining gating element between the always-on 5V boost
  and the connector VBUS is the **MVS switch** (`pm8901_mvs`).

**Hypothesis:** PM8901 MPP0 may be a no-op on tenderloin's production
PCB (e.g. unconnected pin, or controlling a subsystem unrelated to
VBUS), kept in legacy code as boilerplate from the MSM8X60-FFA
reference platform.  The `active_high = 0` override smells like a
"don't drive this signal" hack — on tenderloin the MPP0 pin starts
HIGH (after `regulator_disable`) and goes LOW only when "enabled".

**Decision:** **Skip MPP0 control on mainline.**  Use `vdd50_boost`
(always-on, already in DT) → `pm8901_mvs` (gated by chipidea
`vbus-supply`) → VBUS_USB_CONN.  If host-mode VBUS does not actually
appear on the connector when MVS is enabled (verifiable with a
multimeter), revisit and add PM8901 MPP0 control via the
`pinctrl-ssbi-mpp` GPIO controller binding (which mainline does
support: `drivers/pinctrl/qcom/pinctrl-ssbi-mpp.c:885` has
`qcom,pm8901-mpp`).

### Q3: USB_ID wiring

**Question:** Is the micro-USB ID pin routed to the PHY's ID input
on tenderloin?  Legacy uses PHY-internal ID detection
(`msm_hsusb_pmic_id_notif_init()` returns -ENOTSUPP at
`board-tenderloin.c:821-825`), so PMIC-routed ID is *not* used.

**Schematic evidence** (Topaz3G):

```
R211     100K_5%      ID_TOPAZ
```

A 100 kΩ pull-up resistor on the `ID_TOPAZ` net — standard OTG cable
detection topology.  When a regular USB cable is plugged in, the ID
pin floats and the pull-up drives the PHY ID input HIGH (= device
mode).  When an OTG cable (with ID pin grounded internally) is
plugged, the PHY ID input goes LOW (= host mode).

The PHY's `set_mode()` op
(`drivers/phy/qualcomm/phy-qcom-usb-hs.c:50-97`) enables
`ULPI_INT_IDGRD` interrupts for `PHY_MODE_USB_OTG` /
`PHY_MODE_USB_HOST`, which is exactly the mechanism we need.

**Conclusion:** USB_ID is wired through to the PHY.  PHY-internal
detection will work as legacy did.  (Per the schematic caveat, this
specifically refers to the Topaz3G layout — production Topaz WiFi
should match, but is unconfirmable without that schematic.  However,
if the production PCB were materially different, USB peripheral mode
would also have failed — which it has not, per
`impl-usb-phy-tuning.md` and `impl-usb-charger-detection.md`.)

### Decision summary

| Question | Answer | Action |
|----------|--------|--------|
| GPIO 102 conflict | No.  `vdd50_boost` already uses it; production audio is on different pins | Re-use existing `vdd50_boost` |
| PM8901 MPP0 needed | Probably not; vdd50_boost is always-on | Skip; revisit if R4 fails |
| USB_ID wired | Yes, with 100k pull-up to PHY | Enable PHY OTG mode in DT |

**Implement, not won't-fix.**  R6 is therefore not invoked.

### R1 acceptance criteria coverage

- [x] **AC1** Conclusion written — this document.
- [x] **AC2** GPIO 102 conflict resolved — production audio uses
      different pins; `vdd50_boost` already owns GPIO 102 (live state
      confirms boost is enabled).
- [x] **AC3** PM8901 MPP1/0 role decided — skip on mainline; mainline
      uses always-on `vdd50_boost` as the equivalent of legacy's
      "enable the boost" step.  Documented revisit condition.
- [x] **AC4** USB_ID schematic trace confirmed — 100kΩ pull-up to PHY
      ID input.
- [x] **AC5** Decision recorded: **implement** (DT-only).

## R2-R6: not started

Next step is **R2 (5V VBUS supply chain in DT)** which is now much
smaller than originally scoped — the boost regulator already exists.
Required changes:

1. Override `mvs_in-supply` from `pm8058_s3` (1.8 V — incorrect) to
   `vdd50_boost` (5 V — correct).  Pre-existing miswiring; this
   incidentally fixes any future HDMI 5V hot-plug detect work too.
2. No new fixed-regulator needed (`vdd50_boost` already provides 5V
   via GPIO 102).
3. No new pinctrl needed (TLMM pinmux for GPIO 102 already
   established by the existing `vdd50_boost` consumer).

Then R3:

4. `&usb1: dr_mode = "otg"` (was `"peripheral"`).
5. `&usb1: vbus-supply = <&pm8901_mvs>`.

Then R4 hardware verification with OTG cable + USB keyboard / mass
storage.

R5 regression bar: existing peripheral-mode plug/unplug (cycle test
in `impl-usb-phy-tuning.md`) + BC 1.2 detection
(`impl-usb-charger-detection.md`) must keep working.

## Cross-References

- **Kit:** `context/kits/cavekit-usb-otg-host.md`
- **Sibling impl (regression baseline):**
  `context/impl/impl-usb-phy-tuning.md` (6/6 plug/unplug),
  `context/impl/impl-usb-charger-detection.md` (R2-R4 done)
- **Legacy reference:**
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1040-1090`
  (`msm_hsusb_vbus_power` + `msm_usb_host_pdata`)
- **Mainline DT consumer already in place:**
  `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi:207-220`
  (`vdd50_boost` fixed-regulator on TLMM GPIO 102)
- **PHY OTG support:**
  `drivers/phy/qualcomm/phy-qcom-usb-hs.c:50-97`
  (`qcom_usb_hs_phy_set_mode` enables `ULPI_INT_IDGRD` for OTG/HOST)
- **PM8901 MPP fallback path (if needed):**
  `drivers/pinctrl/qcom/pinctrl-ssbi-mpp.c:885`
  (`qcom,pm8901-mpp` binding)
