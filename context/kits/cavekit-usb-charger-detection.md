---
domain: usb-charger-detection
created: "2026-05-21"
last_edited: "2026-05-21"
status: draft
---

# Cavekit: USB Charger Type Detection

## Scope

Detect the type of power source connected to the TouchPad's USB port
(host SDP, CDP, dedicated DCP, HP Touchstone wireless charger 10 W,
HP OMTP wall charger 900 mA) and expose the result through a standard
mainline interface so userspace and downstream battery/charger logic
can act on it. **Detection only** — wiring the result to the MAX8903B
charger IC or coordinating with the A6 battery controller is out of
scope.

## Context

The legacy 2.6.35-palm kernel implemented a custom multi-stage USB
charger detection routine in
`webos-linux-kernel-touchpad/drivers/usb/gadget/msm72k_udc.c:508-606`
(`usb_multi_chg_detect()`). The routine manipulates D+/D- pull-up/down
resistors via ULPI register writes and reads `PORTSC_LS` line state to
distinguish between five charger types:

| Type | D+/D- response | Current | Source |
|------|---------------|---------|--------|
| `USB_CHG_TYPE__SDP` | D+/D- both low, post-CDP-current-source D+ still low, reg 0x34 bit 4 clear | 500 mA | USB host |
| `USB_CHG_TYPE__SDP` (CDP) | same as SDP but reg 0x34 bit 4 set | 1400 mA | Charging Downstream Port |
| `USB_CHG_TYPE__WALLCHARGER` (HP Phone 900 mA) | D+/D- both high (DCP shorted) | 900 mA | HP phone adaptor |
| `USB_CHG_TYPE__WALLCHARGER` (HP 10 W) | D+/D- both high, D- still high after pull-down on D+ | 2000 mA | Touchstone 10 W |
| `USB_CHG_TYPE__WALLCHARGER` (OMTP) | D+/D- both low initially, both high after current source on D+ | 900 mA | OMTP wall charger |
| `USB_CHG_TYPE__WALLCHARGER` (unknown) | various other patterns | 100 mA | unknown adaptor |

The legacy ULPI register sequence is documented in
`msm72k_udc.c:517-598`:
- `ulpi_write(0x0f, 0x34)` — reset vendor state
- `ulpi_write(0x4d, 0x04)` — configure for charger detect
- `ulpi_write(0x06, 0x0c)` — pull-down config
- `ulpi_write(0x45, 0x04)` + `ulpi_write(0x02, 0x0b)` — pull-down D+
- `ulpi_write(0x25, 0x34)` + `ulpi_write(0x24, 0x34)` — current source on D+
- Final: `ulpi_write(0x0f, 0x34)` — clean up

The mainline chipidea driver already has a charger-state notification
hook in `drivers/usb/chipidea/udc.c:1887` that calls
`usb_phy_set_charger_state(ci->usb_phy, …)` when VBUS changes — but
only if `ci->usb_phy` is registered. The qcom-usb-hs PHY driver uses
the new generic PHY API and does NOT register a `struct usb_phy`, so
the hook never fires today.

The MAX8903B charger IC is already wired in DT
(`qcom-apq8060-tenderloin-common.dtsi:1577-1607`) with sysfs control of
DC and USB current limits — userspace can already configure charging,
it just has no information about what type of charger is connected.

This kit deliberately stops at the detection-and-reporting layer.
Hooking detection results into MAX8903B current selection or A6
battery driver state belongs in a separate kit/follow-up.

## Requirements

### R1: Mainline Programming-Path Investigation

**Description:** Determine whether mainline chipidea + qcom-usb-hs can
host a charger-detection routine equivalent to the legacy
`usb_multi_chg_detect()`, and pick the cleanest architectural seam.

**Acceptance Criteria:**
- [ ] Investigation conclusion written to
      `context/impl/impl-usb-charger-detection.md`
- [ ] Conclusion identifies whether the detection lives:
      (a) in the qcom-usb-hs PHY driver (PHY exposes ULPI access),
      (b) in ci_hdrc_msm.c via direct ULPI viewport, or
      (c) in a new chipidea sub-driver hooking
          `CI_HDRC_CONTROLLER_VBUS_EVENT`,
      with rationale per option
- [ ] Conclusion confirms whether the qcom-usb-hs PHY needs to register
      a `struct usb_phy` for `usb_phy_set_charger_state()` to fire, or
      whether an alternate reporting path is preferred
- [ ] Driver source citations for every claim (file path + line number)

**Dependencies:** none

### R2: Standard BC 1.2 Detection (SDP/CDP/DCP)

**Description:** Detect the three standard BC 1.2 charger types on USB
attach.

**Acceptance Criteria:**
- [ ] On VBUS rise from a USB host, the kernel logs the charger as SDP
- [ ] On VBUS rise from a CDP (charging downstream port), the kernel
      logs the charger as CDP
- [ ] On VBUS rise from a DCP (D+/D- shorted wall charger), the kernel
      logs the charger as DCP
- [ ] Detection completes within 2 seconds of VBUS-rise interrupt
- [ ] The result is exposed via the `usb_phy->chg_type` field
      (preferred — this is the in-tree interface chipidea already
      consumes at `udc.c:1886-1898`); if R1 concludes that interface
      is unreachable for qcom-usb-hs without dual-API registration,
      the impl doc explicitly records the chosen fallback
      (`power_supply` type/online, extcon, or documented sysfs node)
      and the reason

**Dependencies:** R1 (path chosen)

### R3: HP/Palm-Specific Charger Variants

**Description:** Distinguish the HP-specific charger variants the
legacy kernel detected, because these advertise different max-current
capabilities (900 mA, 2000 mA) that drive significantly different
charging behaviour.

**Acceptance Criteria:**
- [ ] HP Touchstone 10 W wireless charger detected as a distinct type
      from generic DCP (legacy `2-1-1-2` path: D+/D- high, D- still high
      after D+ pull-down)
- [ ] HP Phone Adaptor 900 mA detected as a distinct type from generic
      DCP (legacy `2-1-1-1` path: D+/D- both high)
- [ ] OMTP-class 900 mA adaptor detected (legacy `2-2-1-1` path: D+/D-
      both low, both high after current source on D+)
- [ ] Each detected type maps to a current value matching the legacy
      table (500/900/1400/2000 mA) — recorded in the impl doc
- [ ] A DT property or compatible-string match gates the HP-specific
      detection branches; removing the property at boot causes a wall
      charger that would otherwise be classified as "HP Touchstone
      10 W" or "HP Phone Adaptor 900 mA" to be reported as generic
      DCP instead — verified by booting with and without the gating
      property set and observing the differing log line / sysfs value

**Dependencies:** R2 (standard detection working)

### R4: No Regression on USB Function

**Description:** Charger detection must not break the existing
plug/unplug behaviour delivered by the usb-phy-tuning kit.

**Acceptance Criteria:**
- [ ] 10 consecutive plug/unplug cycles with a USB host enumerate
      cleanly (matches the bar set in cavekit-usb-phy-tuning R3)
- [ ] No new `ci_hdrc` / `chipidea` error lines in dmesg compared to
      pre-change baseline
- [ ] If detection sequence runs as part of the enumeration path,
      detection does not delay enumeration beyond what is observed
      without the detection patch on 5 consecutive plug cycles:
      measure `STOPPED_EVENT` → `g_ether: high-speed config #1` dmesg
      timestamp delta with and without the patch, compare the medians,
      regression budget = +200 ms (one-sigma of the existing ~1.9–2.7 s
      spread observed in `impl-usb-phy-tuning.md`)

**Dependencies:** R2

### R5: Won't-Fix / Partial-Detect Documentation (Conditional)

**Description:** If R1 concludes the cleanest path requires a non-trivial
mainline change (e.g. converting qcom-usb-hs to dual-API), or if the
HP-specific variant detection is not portable to mainline ULPI access
without driver hacks, the kit closes with documented limitations rather
than partial code.

**Acceptance Criteria:**
- [ ] `context/impl/impl-usb-charger-detection.md` records which
      requirements (R2 / R3) are achievable and which are gated on
      future mainline changes
- [ ] Concrete driver/binding file citations explain each unreachable
      path
- [ ] Revisit conditions are listed (e.g. "qcom-usb-hs gains
      `charger_detect` op", "BC 1.2 framework lands in chipidea-msm")
- [ ] R4 still passes — USB does not regress

**Dependencies:** R1

## Out of Scope

- Wiring detected charger type to MAX8903B GPIO current selection
  (separate kit; MAX8903B driver already exposes
  `input_current_limit` sysfs that userspace can drive)
- A6 battery controller coordination — A6 already knows about the
  Touchstone via its own GPIO state, this kit doesn't change that
- USB Power Delivery (PD) — not supported by this SoC's PHY
- USB-C orientation / role switching — TouchPad is micro-B only
- Charge-current ramp logic, battery state of charge — owned by
  A6/MAX8903B drivers
- DCP fast-charge wake-from-suspend — depends on `wifi-suspend-wake`
  resolving first

## Cross-References

- `cavekit-usb-phy-tuning.md` — R3 (regression guard) is the same bar
  as this kit's R4; the plug/unplug test infrastructure is shared
- Legacy reference:
  `webos-linux-kernel-touchpad/drivers/usb/gadget/msm72k_udc.c:461-606`
- Hardware integration: MAX8903B already in DT at
  `qcom-apq8060-tenderloin-common.dtsi:1577-1607`
- Battery controller: `drivers/misc/a6/` (no current USB awareness)

## Source Traceability

- Legacy detection routine:
  `webos .../drivers/usb/gadget/msm72k_udc.c:461-606` —
  `usb_chg_detect()` work function and `usb_multi_chg_detect()` core
  logic
- Legacy charger-type enum:
  `webos .../arch/arm/mach-msm/include/mach/msm_hsusb.h` — defines
  `USB_CHG_TYPE__SDP`, `USB_CHG_TYPE__WALLCHARGER`, `USB_CHG_TYPE__INVALID`
- Mainline chipidea hook:
  `drivers/usb/chipidea/udc.c:1886-1898` —
  `usb_phy_set_charger_state()` call site in `ci_udc_vbus_session`
- Mainline qcom PHY (no charger detect today):
  `drivers/phy/qualcomm/phy-qcom-usb-hs.c` — uses generic PHY API only
- Reference implementation (imx):
  `drivers/usb/chipidea/usbmisc_imx.c:931` —
  `imx7d_charger_detection()` shows how a chipidea sub-driver wires
  charger detection into `usb_phy->chg_type`
- MAX8903B integration target:
  `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi:1577-1607`
