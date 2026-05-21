---
domain: usb-otg-host
created: "2026-05-21"
last_edited: "2026-05-21"
status: draft
---

# Cavekit: USB OTG Host Mode

## Scope

Enable USB host mode on the HP TouchPad's micro-USB connector so the
device can act as a USB host (USB keyboards, mass storage, etc.) when
an OTG cable with the ID pin grounded is connected.  Investigate
first, then wire the 5V VBUS supply path, switch the chipidea
controller from peripheral-only to OTG dual-role, and verify on
hardware.  Peripheral-mode behaviour (existing plug/unplug + BC 1.2
charger detection) must not regress.

## Context

The HP TouchPad has a micro-USB-B connector with the ID pin wired
through to the ULPI PHY (verified from Topaz3G schematic --
`USB_ID` net at the connector terminates on the PHY's ID input).
Legacy 2.6.35-palm webOS built OTG host support
(`CONFIG_USB_EHCI_MSM=y`, `CONFIG_USB_MSM_OTG_72K=y`) and used PHY-
internal ID detection via the OTGSC register (the legacy
`msm_hsusb_pmic_id_notif_init()` explicitly returns `-ENOTSUPP` for
tenderloin -- PMIC-based ID detection is *not* wired; it's the PHY
path that works).

Legacy `msm_hsusb_vbus_power()` (`board-tenderloin.c:1040-1085`)
enables host-mode VBUS via two regulators in sequence:

1. `8901_mpp0`  -- PM8901 MPP1 pin configured as active-high digital
                   output; legacy regulator label "8901_mpp0".
                   Believed to gate an additional load switch in the
                   5V path.
2. `8901_usb_otg` -- PM8901 MVS (Medium Voltage Switch) USB_OTG
                     output; gates 5V from the boost into
                     `VBUS_USB_CONN`.

The Topaz3G schematic shows the actual 5V boost is enabled by **MSM
TLMM GPIO 102** (net `VDD5V_EN`) -- not by PM8901 MPP.  GPIO 102 also
has audio MI2S/FM_I2S alt functions in the SoC pinmux which must not
be selected when used as a plain GPIO.

Mainline state in `qcom-apq8060-tenderloin-common.dtsi`:

- `&usb1: dr_mode = "peripheral"` -- locks the chipidea controller
  to gadget-only.
- No `vbus-supply` on the USB controller.
- `pm8901_mvs` exists with `mvs_in-supply = <&pm8058_s3>` which is
  wrong (1.8V cannot supply a 5V switch).  Currently consumed by
  `hdmi-mux-supply` which therefore cannot have ever worked
  properly; HDMI is not in active use on this port.
- No 5V boost regulator defined.

The downstream chipidea-msm + qcom-usb-hs PHY pair already supports
OTG dual-role when `dr_mode = "otg"`: the PHY's `set_mode()` op
(`drivers/phy/qualcomm/phy-qcom-usb-hs.c:50-97`) enables ULPI_INT_IDGRD
interrupts for OTG/HOST modes, and the chipidea core consumes the
ID bit from OTGSC to drive the role switch.

This kit replicates the legacy power-up sequence in mainline DT
terms, switches to `dr_mode = "otg"`, and validates host-mode
enumeration with a downstream USB device.

## Requirements

### R1: Hardware-feasibility investigation

**Description:** Confirm the hardware can do OTG host mode on
mainline and identify the exact supply-chain topology before
committing to DT changes.

**Acceptance Criteria:**
- [ ] Investigation conclusion written to
      `context/impl/impl-usb-otg-host.md`
- [ ] GPIO 102 confirmed unclaimed by any existing pinmux state in
      `qcom-apq8060-tenderloin-common.dtsi` and
      `qcom-msm8660.dtsi` (no MI2S/FM_I2S consumer); if claimed,
      conflict resolution documented
- [ ] Role of PM8901 MPP1 (`8901_mpp0` regulator in legacy) decided:
      either confirmed required as a secondary load switch (with
      cite to schematic net + legacy power sequence), or
      confirmed optional (legacy comment / behaviour cite)
- [ ] USB_ID wiring confirmed at the schematic level: net traces
      from the micro-USB connector ID pin to the PHY ID input,
      with 100kΩ pull-up reference
- [ ] Decision recorded: implement (DT-only) OR won't-fix (with
      reason, e.g. missing PMIC driver support for MVS)

**Dependencies:** none

### R2: 5V VBUS Supply Chain in DT

**Description:** Define the mainline regulators and pinmux that
mirror the legacy VBUS power-up sequence.

**Acceptance Criteria:**
- [ ] A `regulator-fixed` node represents the external 5V boost,
      gated by **MSM TLMM GPIO 102** active-high, with 5V min/max
      and a `vin-supply` pointing to `vph`
- [ ] A pinctrl entry sets GPIO 102 to plain GPIO mode (function
      "gpio"), `output-low` by default, bias-disable
- [ ] `pm8901_mvs.mvs_in-supply` is overridden in the tenderloin
      DTSI to point at the new 5V boost regulator (correcting the
      pre-existing 1.8V miswiring)
- [ ] If R1 concluded PM8901 MPP1 is required: a second
      `regulator-fixed` (or PM8901 MPP regulator binding, if
      mainline supports it) is wired into the chain
- [ ] `dtbs_check` passes with no new schema warnings introduced
      by these nodes

**Dependencies:** R1 (implement decision)

### R3: chipidea Controller Switched to OTG Dual-Role

**Description:** Switch the USB controller from peripheral-only to
OTG and wire the VBUS regulator.

**Acceptance Criteria:**
- [ ] `&usb1.dr_mode` changed from `"peripheral"` to `"otg"` in
      `qcom-apq8060-tenderloin-common.dtsi`
- [ ] `&usb1.vbus-supply = <&pm8901_mvs>` added (or whatever the
      R2 final consumer is)
- [ ] Boot still completes without USB-related probe failures or
      MAX8903B charger regression
- [ ] Without any cable plugged: VBUS not asserted on the
      connector — verified by a multimeter reading 0 V at the
      connector's VBUS pin (authoritative).  The
      `/sys/class/power_supply/ci_hdrc_usb/online` node (if
      present from `cavekit-usb-charger-detection` R2) should also
      read 0, but its presence is conditional on that kit having
      landed; the multimeter check is the canonical pass

**Dependencies:** R2

### R4: Host-Mode Functional Verification

**Description:** Plug an OTG cable + downstream USB device and
verify enumeration / data exchange.

**Acceptance Criteria:**
- [ ] With OTG cable connected (ID grounded) and no downstream
      device: VBUS asserted on the connector measured 5.0 V ± 0.25 V
      (authoritative pass).  Additionally, the chipidea controller
      reports HOST role via one of: `/sys/devices/platform/soc/
      12500000.usb/ci_hdrc.0/role`, the `role` attribute under the
      gadget UDC node, or a `dmesg` line indicating role change to
      host -- the exact log string to be recorded from on-device
      output during this AC's verification rather than committed
      to a literal match here
- [ ] With OTG cable + a USB **keyboard** plugged: `lsusb` lists
      the keyboard, `/dev/input/event*` appears, `evtest` shows
      key presses
- [ ] With OTG cable + a USB **mass-storage device**: the device
      enumerates, `/dev/sd*` appears, a small read (e.g. `dd
      if=/dev/sda of=/dev/null bs=4k count=16`) completes without
      I/O errors
- [ ] Host current budget: kernel does not enforce more than the
      schematic-supported budget (~390 mA per legacy
      `msm_usb_host_pdata.power_budget`) without explicit DT
      `vbus-supply` current limits

**Dependencies:** R3

### R5: Peripheral Mode Regression Guard

**Description:** Peripheral mode (existing plug-to-laptop SDP
detection, plug-to-charger DCP detection) must continue to work
unchanged after `dr_mode = "otg"`.

**Acceptance Criteria:**
- [ ] Standard USB cable plug to PC: `usb_type=SDP`,
      `current_max=500000`, SSH (via g_ether) comes up within the
      previously measured ~2 s budget (matches usb-charger-detection
      R4 baseline)
- [ ] HP wall charger plug: `usb_type=DCP`,
      `vendor_charger_variant=hp-touchstone-10w`,
      `current_max=2000000` (matches usb-charger-detection R3
      results)
- [ ] 10 consecutive plug/unplug cycles with standard cable
      enumerate cleanly; `gether_disconnect` / `notify connect true`
      latency matches the pre-OTG baseline within +200 ms
- [ ] The plug/unplug enumeration-regression bar already recorded
      in `context/impl/impl-usb-phy-tuning.md` (6/6 cycles passing,
      median ~2 s plug-to-enumerated) continues to pass with
      `dr_mode = "otg"` -- this is the stable ground truth, not the
      conditional ACs of the charger-detection kit

**Dependencies:** R3, R4

### R6: Won't-Fix Documentation (Conditional)

**Description:** If R1 concludes the hardware cannot be driven on
mainline (missing PMIC support, schematic incompatibility, etc.),
record the limitation rather than land partial code.

**Acceptance Criteria:**
- [ ] `context/impl/impl-usb-otg-host.md` records what mainline
      driver / binding is missing, with file:line citations
- [ ] Revisit conditions noted (e.g. "future PM8901 MVS MPP
      binding lands", "qcom-usb-hs adds extcon support", etc.)
- [ ] No regression: R5 still passes (the won't-fix path leaves
      `dr_mode = "peripheral"` unchanged)

**Dependencies:** R1 (won't-fix path)

## Out of Scope

- USB-C / Power Delivery — TouchPad is micro-USB-B only
- USB role switching from userspace (`usb-role-switch`) — the
  chipidea OTG state machine handles ID-based switching
  automatically; userspace control is not requested
- A bus-powered USB hub with >390 mA total downstream current —
  schematic budget limits us; documented but not implemented
- HDMI 5V-MVS rework — this kit *incidentally* fixes a
  pre-existing `mvs_in-supply` miswiring, but does not test
  HDMI behaviour
- USB OTG protocol features (HNP/SRP) — TouchPad uses a fixed
  cable ID convention, not full OTG negotiation
- Bus-powered USB device that draws less than 100 mA via the
  inrush-during-enumeration handshake — standard chipidea code
  handles this; no special wiring required

## Cross-References

- `cavekit-usb-phy-tuning.md` — R3 regression bar shared with R5
- `cavekit-usb-charger-detection.md` — R5 verifies the peripheral
  detection path continues to work
- `context/impl/impl-usb-phy-tuning.md` — current peripheral-mode
  plug/unplug baseline
- `context/impl/impl-usb-charger-detection.md` — current BC 1.2 +
  HP variant detection baseline

## Source Traceability

- **Legacy VBUS power sequence:**
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1040-1085`
  (`msm_hsusb_vbus_power`)
- **Legacy regulator labels:**
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:4278`
  (`8901_mpp0`),
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:4279`
  (`8901_usb_otg`)
- **Legacy PMIC ID is unsupported on tenderloin:**
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:821-825`
  (`msm_hsusb_pmic_id_notif_init` returns `-ENOTSUPP`)
- **Legacy host platform data:**
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1087-1090`
  (`msm_usb_host_pdata` with `power_budget = 390`)
- **Mainline PHY OTG support:**
  `drivers/phy/qualcomm/phy-qcom-usb-hs.c:50-97`
  (`qcom_usb_hs_phy_set_mode` enables `ULPI_INT_IDGRD` for
  `PHY_MODE_USB_OTG` / `PHY_MODE_USB_HOST`)
- **Mainline PM8901 MVS regulator:**
  `drivers/regulator/qcom_rpm-regulator.c:865`
  (`{ "mvs", QCOM_RPM_PM8901_MVS, &pm8901_switch, "mvs_in" }`)
- **Current DT state to be modified:**
  `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`
  - `mvs_in-supply = <&pm8058_s3>` (line ~577) -- to be corrected
  - `pm8901_mvs: mvs { bias-pull-down; }` (line ~681) -- existing,
    will gain a consumer (`vbus-supply`)
  - `&usb1 { dr_mode = "peripheral"; ... }` (line ~3706) -- to be
    `"otg"`
- **Schematic citations (Topaz3G):**
  `schematics/Schematics-Topaz3G-6050A2428701-MB-X01.20101012.pdf`:
  - VDD5V_EN net on TLMM GPIO 102 (ball AJ10)
  - VBUS_USB_CONN at micro-USB connector
  - USB_ID with 100kΩ pull-up to PHY ID input
