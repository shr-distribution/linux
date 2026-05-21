---
domain: usb-charger-detection
created: "2026-05-21"
last_updated: "2026-05-21"
status: r1-investigation-complete
---

# Implementation: USB Charger Type Detection

Build site: not yet in a tier (kit is new, not in current build-site.md).
Kit: `context/kits/cavekit-usb-charger-detection.md`.

## Status

**R1 DONE** — investigation conclusion below.
R2–R5: not started.

## R1: Mainline Programming-Path Investigation

### Architectural seams considered

The kit listed three candidates. Each was examined in source.

#### Option (a): qcom-usb-hs PHY driver hosts detection

**File:** `drivers/phy/qualcomm/phy-qcom-usb-hs.c`

The PHY already has direct ULPI access through `uphy->ulpi`. Adding a
`charger_detect` op would be natural. **But** the chipidea framework's
charger-state hook only fires when `ci->usb_phy` is populated:

`drivers/usb/chipidea/udc.c:1886-1888`:
```c
if (ci->usb_phy)
    usb_phy_set_charger_state(ci->usb_phy, is_active ?
        USB_CHARGER_PRESENT : USB_CHARGER_ABSENT);
```

`ci->usb_phy` is set only by the `else` branch in
`drivers/usb/chipidea/core.c:1097-1136` — it is mutually exclusive with
`ci->phy` (the generic PHY). With qcom-usb-hs registered as a generic
PHY, `ci->usb_phy` is **always NULL** in our setup.

To make this work, qcom-usb-hs would need to **dual-register** as both
a generic PHY and a legacy `usb_phy`. Sample-checking apq8064 and
msm8960 DTs (which also use qcom-usb-hs) confirms they rely on
`ci->phy` only. Converting the driver to dual API risks breaking
those platforms and is hard to justify upstream when the new generic
PHY API is the preferred mainline direction.

**Verdict: rejected.** Architecturally invasive; would touch generic
code that other platforms depend on for a tenderloin-specific feature.

#### Option (b): ci_hdrc_msm.c hooks CI_HDRC_CONTROLLER_VBUS_EVENT

**File:** `drivers/usb/chipidea/ci_hdrc_msm.c`

`CI_HDRC_CONTROLLER_VBUS_EVENT` fires from
`drivers/usb/chipidea/udc.c:1890-1892`, called from
`ci_udc_vbus_session()` on every VBUS state change. The hook fires
**before** `ci_hdrc_gadget_connect()` runs the gadget reset path,
giving us the right window for detection.

ULPI access from ci_hdrc_msm is straightforward: we already do
`ulpi_write(ci->ulpi, addr, val)` in our settle sequence
(`ci_hdrc_msm.c:189-194`, the `qcom,phy-settle-seq` writes). The exact
same machinery handles the legacy charger-detection ULPI sequence
(reg 0x04, 0x0b, 0x0c, 0x34).

The imx driver follows the same shape and is the closest in-tree
precedent: `drivers/usb/chipidea/ci_hdrc_imx.c:332-339` hooks
`CI_HDRC_CONTROLLER_VBUS_EVENT` and calls
`imx_usbmisc_charger_detection()` which writes to `usb_phy->chg_type`.

**Verdict: chosen.** Same plug-in point as imx, reuses existing ULPI
infrastructure, isolates HP-specific code to platform-specific
`ci_hdrc_msm.c`, no impact on apq8064/msm8960.

#### Option (c): new chipidea sub-driver analogous to usbmisc_imx.c

**Reference:** `drivers/usb/chipidea/usbmisc_imx.c` (~1300 lines).

The imx pattern factors out platform-specific bits (charger detection,
HSIC, PHY workarounds) into a sibling driver bound by phandle. For
Qualcomm/MSM8660 there is no comparable misc-register block — all the
USB controller/PHY register access we need is reachable from
ci_hdrc_msm directly. Spinning up a new sub-driver would be code
without benefit.

**Verdict: rejected.** No architectural payoff over option (b) for
this hardware.

### `usb_phy` registration question

The kit R1 asks whether qcom-usb-hs needs to register a `struct usb_phy`
for `usb_phy_set_charger_state()` to fire. Answer: **no — and we
should avoid that path entirely.**

Reasons:
1. Dual-registering would touch apq8064/msm8960 (option (a) above).
2. The `usb_phy` charger state machinery only supports BC 1.2 types
   (SDP/CDP/DCP/ACA) and a single binary present/absent state — see
   `drivers/usb/phy/phy.c:177-197` (`__usb_phy_get_charger_type`). It
   has no way to express HP Touchstone 10 W vs HP Phone 900 mA — both
   would collapse to `DCP_TYPE`.
3. The newer `power_supply` framework (`include/linux/power_supply.h`)
   already supports `POWER_SUPPLY_TYPE_USB` with `usb_type` subtype
   property and `constant_charge_current_max` to convey the maxpower
   distinction — strictly richer than `enum usb_charger_type`.
4. The MAX8903B charger IC is already a `power_supply` class device
   (`drivers/power/supply/max8903_charger.c`), so reporting via
   `power_supply` integrates naturally with what userspace already
   reads.

### Chosen architecture

**Detection logic:** `ci_hdrc_msm.c` hooks
`CI_HDRC_CONTROLLER_VBUS_EVENT`. On VBUS rise, runs the legacy ULPI
register sequence (reg 0x04, 0x0b, 0x0c, 0x34 manipulation) via
`ulpi_write(ci->ulpi, ...)`, reads `OP_PORTSC` line state bits to
classify the source.

**Reporting:** register a `struct power_supply` of type
`POWER_SUPPLY_TYPE_USB` from `ci_hdrc_msm` probe. Expose:
- `online` property — set on VBUS present
- `usb_type` property — `POWER_SUPPLY_USB_TYPE_{SDP, CDP, DCP, UNKNOWN}`
- `constant_charge_current_max` property — 500 / 900 / 1400 / 2000 mA
  per legacy table, conveying the HP-specific variant maxpower
- (DT-gated, optional) vendor-specific sysfs node for the variant
  name itself (`hp-touchstone-10w` / `hp-phone-900ma` / `omtp-900ma`),
  for userspace that wants to distinguish without inferring from
  current values

Userspace surface:
- `/sys/class/power_supply/ci_hdrc_usb/online`
- `/sys/class/power_supply/ci_hdrc_usb/usb_type`
- `/sys/class/power_supply/ci_hdrc_usb/constant_charge_current_max`
- `/sys/class/power_supply/ci_hdrc_usb/vendor_charger_variant` (HP-only,
  DT-gated)

### Path NOT chosen branch — does not apply

R1 AC for the "no path exists" branch is not invoked: option (b) is a
clean, in-tree, upstream-friendly route.

### Acceptance criteria coverage (R1)

- **AC1** Investigation conclusion written: DONE — this document.
- **AC2** Conclusion identifies which seam owns detection: DONE —
  option (b), `ci_hdrc_msm.c` `CI_HDRC_CONTROLLER_VBUS_EVENT` handler.
- **AC3** Conclusion confirms `usb_phy` registration is not the chosen
  path: DONE — `power_supply` framework chosen instead, rationale
  recorded above.
- **AC4** Driver source citations: DONE — every claim cites file path
  and line number.

## R2-R5: not started

Implementation pending. R2 will:
1. Add `struct power_supply` registration to `ci_hdrc_msm` probe.
2. Add `CI_HDRC_CONTROLLER_VBUS_EVENT` handler that triggers ULPI-based
   detection.
3. Port the legacy `usb_multi_chg_detect()` BC 1.2 portion (D+/D- short
   detection, CDP differentiation via reg 0x34 bit 4).
4. Map detected type to `power_supply` properties.

R3 will extend the detection with HP variants (Touchstone, OMTP),
DT-gated by a `qcom,hp-charger-detect` boolean property.

## Cross-References

- **Kit:** `context/kits/cavekit-usb-charger-detection.md`
- **Source traceability:** see kit §Source Traceability
- **Sibling impl:** `context/impl/impl-usb-phy-tuning.md`
- **Mainline reference (closest pattern):**
  `drivers/usb/chipidea/ci_hdrc_imx.c:332-339`,
  `drivers/usb/chipidea/usbmisc_imx.c:931-969`
  (`imx7d_charger_detection`)
- **Mainline framework chosen:**
  `include/linux/power_supply.h` — `POWER_SUPPLY_TYPE_USB`,
  `POWER_SUPPLY_USB_TYPE_*` enum, `constant_charge_current_max`
- **Mainline framework NOT chosen (with reason):**
  `drivers/usb/phy/phy.c:177-197` — `usb_phy` charger machinery, only
  supports BC 1.2 types, no HP-variant slot
- **Hardware integration target:**
  `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi:1577-1607`
  (MAX8903B, also a `power_supply`)
