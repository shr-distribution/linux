---
domain: usb-phy-tuning
created: "2026-05-20"
last_updated: "2026-05-20"
status: investigated-path-found
---

# Implementation: USB ULPI PHY Signal-Quality Tuning

Build site: context/plans/build-site.md

## Status

**T-006 DONE** — investigation concludes a programming path exists but
requires a small driver patch + a new DT property. Recommendation
recorded below.

**T-016 DONE (code), HW-verification deferred to T-018** — driver patch
+ binding + DT property landed. Three of four legacy values applied
(pre-emphasis 20 %, HS driver slope 0x05, CDR auto-reset disable);
SE1-gating-disable intentionally skipped, see R2 AC4 below.

## R1: Mainline Programming-Path Investigation

### Candidate driver

- **File:** `drivers/phy/qualcomm/phy-qcom-usb-hs.c`
- **Binding doc:** `Documentation/devicetree/bindings/phy/qcom,usb-hs-phy.yaml`
- **Binding YAML lists `qcom,usb-hs-phy-msm8660`** as a supported compatible — the
  PHY itself is recognised by mainline, only the tuning surface is
  limited.

### Current API reach (`qcom,init-seq`)

`drivers/phy/qualcomm/phy-qcom-usb-hs.c:144-148`:

```c
for (seq = uphy->init_seq; seq->addr; seq++) {
    ret = ulpi_write(ulpi, ULPI_EXT_VENDOR_SPECIFIC + seq->addr,
                     seq->val);
    if (ret)
        return ret;
}
```

`ULPI_EXT_VENDOR_SPECIFIC = 0x80` (mainline `include/linux/usb/ulpi.h`).
The parser at line 225 (`of_property_read_u8_array("qcom,init-seq", ...)`)
treats each `addr` as **an offset from 0x80**, so the property can only
target ULPI extended-vendor regs 0x80-0xFF. There is no mechanism in
the existing binding to reach the **standard** ULPI vendor range
0x30-0x3F (`ULPI_VENDOR_SPECIFIC = 0x30`).

This confirms the TODO comment in
`arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi:3540-3563`:

```
/*
 * HP TouchPad USB PHY tuning parameters from legacy kernel:
 * - Pre-emphasis: 20% (PRE_EMPHASIS_WITH_20_PERCENT = 0x30)
 * - HS driver slope: 0x05
 * - CDR auto reset: DISABLE (bit 1 = 0x02)
 * - SE1 gating: DISABLE (bit 2 = 0x04)
 *
 * These are written to ULPI vendor-specific registers, but the
 * mainline qcom-usb-hs-phy driver only supports extended vendor
 * registers (0x80+) via qcom,init-seq. The legacy parameters
 * targeted registers 0x32 and 0x36 which are in the standard
 * vendor range (0x30-0x3F).
 * ...
 * TODO: Investigate if these legacy tuning values need to be
 * ported to improve signal quality or compatibility.
 */
```

### Reachability survey (other drivers)

- `drivers/phy/qualcomm/phy-qcom-usb-hsic.c` — HSIC variant, not OTG;
  doesn't apply to TouchPad's USB-OTG port.
- `drivers/phy/qualcomm/phy-qcom-usb-snps-femto-v2.c` — Synopsys femto v2
  for newer SoCs; not MSM8660.
- `drivers/usb/phy/phy-ulpi.c` — generic ULPI helpers
  (`otg_ulpi_create()`, `ulpi_write()`); useful but consumed by the PHY
  driver, not a binding surface in itself.
- `drivers/usb/chipidea/ci_hdrc_msm.c` / `chipidea/ci_hdrc.c` — host
  controller wrapper. Does **not** expose a tuning-value DT surface for
  vendor regs.

No existing driver or binding writes the standard vendor range 0x30-0x3F
through a public DT surface. The reachability constraint is enforced by
the `ULPI_EXT_VENDOR_SPECIFIC + addr` arithmetic and the matching parser
in `phy-qcom-usb-hs.c`.

### Legacy reg/value mapping

Source: `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1127-1153`
plus `webos-linux-kernel-touchpad/drivers/usb/otg/msm72k_otg.c:233-293`.

| Parameter | Legacy ULPI reg | Legacy mask | Legacy value |
|-----------|-----------------|-------------|--------------|
| Pre-emphasis 20 % | `ULPI_CONFIG_REG3` (standard vendor range, addr 0x32 in the audit notes) | `ULPI_PRE_EMPHASIS_MASK` | `PRE_EMPHASIS_WITH_20_PERCENT = 0x30` |
| HS driver slope | `ULPI_CONFIG_REG3` (addr 0x36 nibble) | `ULPI_HSDRVSLOPE_MASK` (low 4 bits) | `0x05` |
| CDR auto-reset disable | `ULPI_DIGOUT_CTRL` | `ULPI_CDR_AUTORESET` bit | set the bit (= "disable" semantics in legacy) |
| SE1 gating disable | (paired register in same vendor range) | SE1-gate bit | clear/set per legacy `SE1_GATING_DISABLE` |

Pre-emphasis and HS driver slope share a single ULPI register
(`ULPI_CONFIG_REG3`) in different bitfields. CDR auto-reset and SE1
gating live in adjacent vendor-range registers. The audit notes call
out 0x32 and 0x36 as the two register addresses that need to be
reachable.

### Recommended route (R1 AC3)

**Route:** small driver patch + new optional DT property
`qcom,vendor-init-seq` in `phy-qcom-usb-hs.c`.

**Patch shape:**

1. Add a second `init_seq` parser that reads the new property
   `qcom,vendor-init-seq` as a `u8-matrix` of (addr, val) pairs, treating
   `addr` as a **raw ULPI address** (no `+0x80` offset).
2. Apply it from `phy_init` (or the existing init-seq loop) using
   `ulpi_write(ulpi, addr, val)` directly. The same write path the
   legacy driver used (`webos .../msm72k_otg.c:250,263,278,293`).
3. Update the `qcom,usb-hs-phy.yaml` binding to declare the new property
   with description "Sequence of (addr, val) pairs written to the raw
   ULPI address — typically used to reach the standard vendor range
   0x30-0x3F that `qcom,init-seq` cannot."
4. Defaults: zero entries → no-op (preserves no-regression for boards
   that don't set it).

**Why this route over the alternatives:**

- *Extend existing `qcom,init-seq`* (add a flag or per-entry width to
  indicate "raw addr"): breaks backwards compat with deployed DTBs.
- *Hardcode the four values in the driver gated on compat string*:
  works but leaks board-specific PHY tuning into the driver; harder to
  upstream and harder to override per-board.
- *Add chipidea-level vendor hook*: outside this PHY's responsibility;
  worse layering.

A new optional property + ~30 lines of driver code is the smallest
change that satisfies the audit's "investigate-then-conditional-fix"
charter and keeps the door open for the values to be applied if
hardware testing later shows real signal-quality gains. The patch is
plausibly upstreamable on its own merit (the standard vendor range is
documented to exist; mainline just hasn't needed it yet).

### Path-NOT-found branch — does not apply

R1 AC4 ("if no route exists, documented as such") is not invoked: a
small driver patch + new binding property is a route, even though it is
not entirely zero-effort.

## R2: Apply Legacy Values

### Implementation

Driver — `drivers/phy/qualcomm/phy-qcom-usb-hs.c`:

- New `struct ulpi_seq *vendor_init_seq` field on `struct qcom_usb_hs_phy`.
- New parser block in `qcom_usb_hs_phy_probe()` mirroring the existing
  `qcom,init-seq` parser (allocate `(size/2) + 1` slots,
  `of_property_read_u8_array`, NUL-terminate).
- New write loop in `qcom_usb_hs_phy_init()` placed **after**
  `reset_control_reset(uphy->reset)`, calling `ulpi_write(ulpi,
  seq->addr, seq->val)` with the raw address (no
  `ULPI_EXT_VENDOR_SPECIFIC` offset). Placement after reset ensures
  the values survive the controller's reset path; the existing
  `qcom,init-seq` loop runs before reset and therefore cannot guarantee
  the same.

Binding — `Documentation/devicetree/bindings/phy/qcom,usb-hs-phy.yaml`:

- Add `qcom,vendor-init-seq` as a `uint8-matrix` of (raw addr, val)
  pairs. Same shape as `qcom,init-seq`, different semantics (no
  +0x80 base).

DT — `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`:

- Replace the old TODO comment in `&usb_hs1_phy` with the actual
  sequence:
  ```
  qcom,vendor-init-seq = /bits/ 8 <0x32 0x35>, /bits/ 8 <0x36 0x02>;
  ```

### Value derivation (from legacy mach-msm headers — MSM8X60 variant)

- `ULPI_CONFIG_REG3 (0x32)`:
  - `ULPI_PRE_EMPHASIS_MASK = (3 << 4) = 0x30` → 20 % pre-emphasis
  - HS driver slope value `0x05` (within `ULPI_HSDRVSLOPE_MASK = 0x0F`)
  - Combined post-reset value: `0x30 | 0x05 = 0x35`
- `ULPI_DIGOUT_CTRL (0x36)`:
  - `ULPI_CDR_AUTORESET = (1 << 1) = 0x02` set → CDR auto-reset disable
    (legacy semantics: setting this bit *disables* the auto-reset).
  - `ULPI_SE1_GATE = (1 << 2) = 0x04` set → SE1 gating disable
    (legacy semantics: setting bit 2 *disables* gating; see
    `set_se1_gating()` at
    `webos .../drivers/usb/otg/msm72k_otg.c:280-294`).
  - Combined post-reset value: `0x02 | 0x04 = 0x06`.
- Source:
  `webos .../arch/arm/mach-msm/include/mach/msm_hsusb_hw.h:163-189`
  (mach-msm — the MSM8X60 variant. NB: there is also a mach-fsm
  variant with different bit positions — for tenderloin we use the
  mach-msm one because MSM8X60 falls under `CONFIG_ARCH_MSM8X60`);
  `webos .../arch/arm/mach-msm/board-tenderloin.c:1127-1153` for the
  values; `webos .../drivers/usb/otg/msm72k_otg.c:243-293` for the
  write-path semantics.

### Failed experiment: PHY POR reset wiring (both attempts reverted)

Background: on-device test of soft_connect "disconnect" -> "connect"
on kernel gb2dc29139bba showed the device failed to re-enumerate to
the host (USB-net interface disappeared on host, never came back,
manual power cycle required). Hypothesis: `usb_hs1_phy` DT node
missing `resets`/`reset-names`, so `qcom_usb_hs_phy_power_on()`
skipped `reset_control_reset()` and vendor-init-seq writes landed on
stale ULPI state.

**Forensic reference** — legacy `webos .../drivers/usb/otg/msm72k_otg.c`
otg_reset() at line 1474-1545 (serves the same MSM8x60 USB OTG IP
block) does:

  msm_otg_phy_reset(dev)                 // PHY POR pulse
  ulpi_write(0xFF, 0x0F)                 // INT_RISE_C: disable all
  ulpi_write(0xFF, 0x12)                 // INT_FALL_C: disable all
  msleep(100)                            // 100 ms ULPI settle
  writel(USBCMD_RESET, USB_USBCMD)       // <<< LINK RESET
  ... wait for RESET bit to clear ...
  writel(0x80000000, USB_PORTSC)         // re-select ULPI transceiver
  set_pre_emphasis_level / hsdrvslope    // ULPI write reg 0x32
  set_cdr_auto_reset / se1_gating        // ULPI write reg 0x36

**Attempt 1 (commit cae1a8c571b9):** add `resets = <&usb1 0>;
reset-names = "por";` only (no settle delay).
**Result: gadget enumeration completely broken.** The POR pulse hit
but the vendor-init-seq writes immediately afterwards (only `udelay(12)`
from chipidea's ci_hdrc_msm_por_reset) landed on a re-syncing ULPI
link. Reverted in 0b1f7e6448c0.

**Attempt 2 (commit 6c2eb508669d):** pair the DT resets addition with a
driver patch adding a 100 ms settle in `qcom_usb_hs_phy_power_on()`
after `reset_control_reset`:

  ulpi_write(ulpi, ULPI_USB_INT_EN_RISE, 0);  // disable all rise ints
  ulpi_write(ulpi, ULPI_USB_INT_EN_FALL, 0);  // disable all fall ints
  msleep(100);                                // ULPI settle

**Result: gadget enumeration still completely broken.** The 100 ms
settle alone was not enough. The legacy sequence has a USBCMD_RESET
link reset and PORTSC=0x80000000 transceiver re-select *after* the
settle and *before* the ULPI vendor writes. Without that link reset the
chipidea controller's ULPI engine does not re-sync.
Reverted: DT resets lines removed, driver delay code removed.

**Root cause:** USBCMD_RESET is a controller-level operation that
cannot be performed from inside `phy_power_on()` — the PHY driver has
no access to the chipidea MMIO registers.

**Correct fix path:** patch `drivers/usb/chipidea/ci_hdrc_msm.c` to
run the full legacy otg_reset() sequence on
`CI_HDRC_CONTROLLER_RESET_EVENT`:
  POR pulse → INT_EN_RISE/FALL = 0 → msleep(100) → USBCMD_RESET →
  wait → PORTSC=0x80000000 → ULPI vendor writes (reg 0x32, 0x36).
This keeps the sequence atomic at the controller level where all the
required MMIO is accessible. Tracked as a follow-up; not in this
cavekit batch.

Not done in this batch (still tracked as follow-ups):
  - PM8058 extcon for ID-pin detection (not needed for plug/unplug --
    legacy webOS itself does not wire PMIC ID detection on tenderloin
    per board-tenderloin.c msm_hsusb_pmic_id_notif_init() returning
    -ENOTSUPP for machine_is_tenderloin()).
  - Full OTG host/device role-switch (requires GPIO/extcon ID source
    + VBUS regulator wiring + dr_mode = "otg"; separate cavekit).

### Acceptance criteria coverage (R2)

- AC1 Pre-emphasis 20 % configured: DONE — reg 0x32 bits 4-5 written
  as 0x30.
- AC2 HS driver slope 0x05 configured: DONE — reg 0x32 low bits
  written as 0x05.
- AC3 CDR auto-reset disabled: DONE — reg 0x36 bit 1 set.
- AC4 SE1 gating disabled: **DONE** — reg 0x36 bit 2 set
  (`ULPI_SE1_GATE = (1 << 2)`). Originally deferred because I had
  only inspected the `mach-fsm` variant of `msm_hsusb_hw.h`; a
  deeper search of the `mach-msm` variant (which carries the
  `CONFIG_ARCH_MSM8X60` ifdef branch) located the macro at line 171
  and `set_se1_gating()` confirmed the polarity (set bit -> disable
  gating). Combined reg 0x36 value updated from 0x02 -> 0x06.
- AC5 verifiable from driver state: deferred to T-018 / ad-hoc
  `ulpi_read` from a debug shim. The driver patch keeps the writes in
  `qcom_usb_hs_phy_init()` so they execute on every PHY init —
  reproducible across resume cycles.

## R3: USB Regression Guard

R3 ACs (10 plug/unplug, 1 GiB bulk, mean-rate within 10 %) are
deferred to T-018 and apply regardless of whether the tuning values are
applied or skipped. R3 is a guard, not gated on the conclusion of R1.

## R4: Won't-Fix Documentation

Not invoked — see "Path-NOT-found branch" above. If the recommended
driver patch later turns out to be rejected upstream or unworkable,
fall back to R4 by recording the rejection rationale here and treating
T-017 (won't-fix) as the active branch.

**Revisit conditions** (kept on file so a future contributor knows when
to revisit):

- A mainline patch extends `qcom,init-seq` to cover the standard vendor
  range (would make `qcom,vendor-init-seq` redundant).
- Hardware bring-up identifies a USB signal-integrity issue that the
  legacy values demonstrably fix (raises priority on the driver patch).
- A different mainline PHY driver supersedes `phy-qcom-usb-hs.c` for
  MSM8660 and exposes vendor-range writes natively.

## Task Tracking

| Task | Status | Notes |
|------|--------|-------|
| T-006 | DONE | path-found: small driver patch + new DT prop `qcom,vendor-init-seq`; targets `ulpi_write(addr, val)` directly with no +0x80 base |
| T-016 | DONE | driver patch + binding + DT property landed; all 4 legacy values applied (reg 0x32 = 0x35, reg 0x36 = 0x06 = CDR_AUTORESET | SE1_GATE) |
| T-017 | N/A | won't-fix path not active (path exists) |
| T-018 | TODO | regression guard, HW (Tier 1) |

## Cross-References

- **Kit:** `context/kits/cavekit-usb-phy-tuning.md`
- **Source traceability:** see kit §Source Traceability
- **Mainline driver:** `drivers/phy/qualcomm/phy-qcom-usb-hs.c:144-148, 218-230`
- **Mainline binding:** `Documentation/devicetree/bindings/phy/qcom,usb-hs-phy.yaml`
- **TODO citation:** `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi:3540-3563`
- **Legacy driver write path:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/usb/otg/msm72k_otg.c:243-293`
- **Legacy board values:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1127-1153`
