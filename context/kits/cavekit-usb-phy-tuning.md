---
domain: usb-phy-tuning
created: "2026-05-19"
last_edited: "2026-05-19"
status: draft
---

# Cavekit: USB ULPI PHY Signal-Quality Tuning

## Scope

Replicate the legacy webOS USB ULPI PHY tuning (pre-emphasis,
HS-driver slope, CDR-autoreset disable, SE1-gating disable) on the
mainline USB stack, **if** mainline allows programming the relevant
ULPI vendor registers. If no path exists, document the limitation as a
won't-fix. USB enumeration must not regress in either outcome.

## Context

The legacy 2.6.35-palm kernel applied four signal-integrity tuning
writes to the USB ULPI PHY at init via `msm_otg_pdata`
(`webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1127-1153`):

- Pre-emphasis: `pemp_level = PRE_EMPHASIS_WITH_20_PERCENT` (legacy
  encoding 0x30 → ULPI vendor reg 0x32)
- HS driver slope: `hsdrvslope = 0x05` → ULPI vendor reg 0x36
- CDR auto-reset: disabled
- SE1 gating: disabled

The mainline `qcom-usb-hs-phy` driver only exposes ULPI extended-vendor
registers ≥ 0x80 via its `qcom,init-seq` mechanism; the legacy values
target the **standard** vendor range 0x30–0x3F, which the existing API
does not appear to reach. A TODO comment in `common.dtsi` (around
lines 3525–3548) flags this explicitly.

USB currently works on PHY default settings — this is a deferred
signal-quality-only item (Hardware Quirks Inventory Q9, Audit gap
"open risk"). It is intentionally scoped as
*investigate-then-conditional-fix*, because the answer might be "no
mainline path exists" and that conclusion is itself valuable to record
to close the open risk.

## Requirements

### R1: Mainline Programming-Path Investigation
**Description:** Determine whether any in-tree mainline PHY driver
applicable to APQ8060 can program ULPI standard vendor registers in the
0x30–0x3F range, citing concrete driver source and binding
documentation.

**Acceptance Criteria:**
- [ ] An investigation conclusion is written to
      `context/impl/impl-usb-phy-tuning.md`
- [ ] The conclusion identifies the candidate PHY driver(s) by file
      path and binding name
- [ ] For each candidate, the conclusion states explicitly whether
      ULPI vendor regs 0x30–0x3F are reachable via the existing
      driver/binding API, a new binding property, or a small driver
      patch — and which of those routes is recommended
- [ ] If no route exists, the conclusion is documented as such with
      reference to the relevant driver source explaining why

**Dependencies:** none

### R2: Legacy Values Are Applied at PHY Init (Conditional)
**Description:** If R1 identifies a programming path, the four legacy
tuning values are applied at PHY init.

**Acceptance Criteria:**
- [ ] Pre-emphasis 20 % is configured (the legacy ULPI reg 0x32 value
      0x30, or its functional equivalent in mainline terms)
- [ ] HS driver slope is configured (the legacy ULPI reg 0x36 value
      0x05, or its functional equivalent)
- [ ] CDR auto-reset is disabled (matching legacy)
- [ ] SE1 gating is disabled (matching legacy)
- [ ] The applied values are verifiable from driver state, a PHY
      register dump, or a debugfs interface — whichever the chosen
      path exposes

**Dependencies:** R1 (path identified)

### R3: USB Function Is Not Worse Than the Default-PHY Baseline
**Description:** Whether tuning is applied (R2) or skipped (R4), USB
enumeration and steady-state I/O must be at least as reliable as the
current default-PHY baseline.

**Acceptance Criteria:**
- [ ] 10 consecutive plug/unplug cycles with a known-good USB-OTG cable
      all enumerate cleanly — no enumeration timeout, no -EPROTO
      observed in kernel log
- [ ] A 1 GiB sequential bulk transfer via USB mass-storage or RNDIS
      completes without controller-driver errors logged
      (no `dwc` / `ci-hdrc` / `chipidea` error lines during the
      transfer)
- [ ] The mean transfer rate is within 10% of the pre-change baseline
      on the same hardware (regression guard against accidental
      bandwidth loss)

**Dependencies:** none — applies in both R2-applied and R4-skipped
outcomes

### R4: Won't-Fix Documentation (Conditional)
**Description:** If R1 concludes no programming path exists, the kit
closes as documented-not-fixed with concrete reference to the limiting
driver code.

**Acceptance Criteria:**
- [ ] `context/impl/impl-usb-phy-tuning.md` records the limitation,
      citing the specific mainline driver/binding lines that prevent
      access to ULPI vendor regs 0x30–0x3F
- [ ] The document notes the conditions under which the conclusion
      should be revisited (e.g. a future mainline patch extending
      `qcom,init-seq` to cover the standard vendor range)
- [ ] No regression is introduced — R3 still passes with the default
      PHY behaviour

**Dependencies:** R1 (path NOT identified)

## Out of Scope

- Re-tuning of ULPI registers ≥ 0x80 (different concern; those are
  already reachable via `qcom,init-seq` if needed)
- USB host-controller-side timing issues (chipidea / device-controller
  bugs)
- USB3 / USB-C — neither exists on this SoC
- Cable / connector signal integrity (hardware concern, not PHY tuning)

## Cross-References

none

## Source Traceability

- `reports/HARDWARE-QUIRKS-INVENTORY-2026-05-19.md` — Q9 (table row),
  "Open risks to track" §Q9
- `reports/BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md` — §2.5 row
  "msm_otg + ULPI 45nm PHY"
- Legacy tuning values:
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:1127-1153`,
  specifically `:1133` (`pemp_level = PRE_EMPHASIS_WITH_20_PERCENT`)
  and `:1136` (`hsdrvslope = 0x05`)
- Current limitation: `common.dtsi` TODO comment around lines
  3525-3548 (mainline `qcom-usb-hs-phy` only exposes vendor regs
  ≥ 0x80 via `qcom,init-seq`)
