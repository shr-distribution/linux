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
recorded below. Apply step (T-016) and regression guard (T-018) deferred
to Tier 1.

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

## R2: Apply Legacy Values (Conditional on T-016)

T-016 will, when executed:

- Add `qcom,vendor-init-seq = /bits/ 8 < 0x32 0x30  0x36 0x05  ... >;`
  (or whichever exact pair encoding the new binding uses) to the
  `&usb_hs1_phy` board node in
  `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`,
  replacing the TODO comment with the actual sequence.
- Land the matching `phy-qcom-usb-hs.c` driver patch.
- Verify the applied values from a register dump or debugfs (whichever
  the driver exposes; if neither exists, add a debugfs node behind a
  KConfig).

R2 stays open until T-016 lands.

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
| T-016 | TODO | apply legacy values via the new property after the driver patch lands |
| T-017 | N/A | won't-fix path not active (path exists) |
| T-018 | TODO | regression guard (runs in either outcome) |

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
