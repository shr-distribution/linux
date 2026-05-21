---
domain: wifi-suspend-wake
created: "2026-05-19"
last_edited: "2026-05-19"
status: draft
---

# Cavekit: WiFi Wake-From-Suspend (SDC4 DAT1 → MPM)

## Scope

The system can be woken from suspend-to-RAM by an inbound network packet
on the AR6003 WiFi interface, routed via SDC4 DAT1 through the MPM
wakeup-interrupt controller. This replicates the legacy
`cfg_mpm_sdiowakeup` path so that WoWLAN works end-to-end.

## Context

The legacy 2.6.35-palm kernel installed
`msm_sdcc_cfg_mpm_sdiowakeup` as SDC4's MPM-wake configuration callback,
mapping the SDC4 wake source to `MSM_MPM_PIN_SDC4_DAT1`
(`webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:5417,
5426,5488`). This made the WiFi chip's SDIO DAT1 line a legitimate wake
source for the SoC, gated through the MPM.

The current 6.18 port has the MPM device-tree node **disabled**, because
enabling it provokes an early-boot hang before any driver probes
(documented at length in `context/impl/impl-mpm-boot-hang.md`, which
catalogs three independent dead-end attempts: reserved-memory,
direct-reg, syscon). Without MPM, SDC4 DAT1 cannot be registered as a
wake source and any "WiFi wake-from-suspend" attempt is impossible.

This kit is therefore **hard-gated** on the separate MPM platform-driver
effort. R1 declares that dependency explicitly so the kit fails closed
rather than appearing to be independently testable.

This is Hardware Quirks Inventory Q5 / Audit Gap #12 — a concrete,
testable consequence of the broader MPM blocker (Audit Gap #3).

## Requirements

### R1: MPM Wakeup-Interrupt Controller Is Functional (Precondition)
**Description:** The MPM hardware block is exposed to the kernel as a
working wakeup-interrupt controller. This requirement is the property
this kit depends on; it is **owned** by the MPM impl-doc effort, not by
this kit, and is listed here only as the explicit dependency edge.

**Acceptance Criteria:**
- [ ] An MPM driver (platform or equivalent) successfully probes at boot
      with the MPM DT node enabled
- [ ] The boot completes to userspace login without the
      `impl-mpm-boot-hang.md`-class early hang
- [ ] The kernel exposes wakeup sources routed through MPM via the
      standard wakeup-source interface
      (e.g. `/sys/kernel/debug/wakeup_sources` or equivalent)

**Dependencies:** Owned by `context/impl/impl-mpm-boot-hang.md`

### R2: SDC4 DAT1 Is Declared as an MPM-Routed Wakeup Source
**Description:** With R1 satisfied, the SDC4 host (or its ath6kl SDIO
child) reports DAT1 as a wakeup source via the MPM.

**Acceptance Criteria:**
- [ ] The SDC4 host or its child WiFi device exposes a `power/wakeup`
      sysfs entry whose value is `enabled` (or the platform-equivalent
      writable wakeup attribute)
- [ ] At boot, the kernel log records wakeup-source registration for the
      SDC4-DAT1 / WiFi path
- [ ] An MPM-side trace (debugfs or kernel log) confirms that the SDC4
      DAT1 line is routed through MPM to the SoC wake controller

**Dependencies:** R1

### R3: WoWLAN Is Configurable on the AR6003
**Description:** The ath6kl driver accepts a "wake on any packet" WoWLAN
configuration via the standard nl80211/cfg80211 interface and the
configuration survives a suspend cycle.

**Acceptance Criteria:**
- [ ] `iw phy phy0 wowlan enable any` (or the cfg80211 equivalent for
      whichever WoWLAN command set the mainline ath6kl exposes) returns
      success
- [ ] After a successful suspend / resume cycle, `iw phy phy0 wowlan show`
      reports the same WoWLAN configuration that was set before suspend
- [ ] The configuration takes effect even when set while the device is
      already associated with a working WiFi network

**Dependencies:** none — but ineffective for end-to-end wake unless
R2 is also met

### R4: Inbound Packet Wakes the Device
**Description:** With R2 and R3 satisfied, an inbound network packet
arriving on the associated WiFi interface causes the device to resume
from suspend-to-RAM.

**Acceptance Criteria:**
- [ ] `echo mem > /sys/power/state` enters suspend-to-RAM and the shell
      blocks on the write (return only on resume)
- [ ] From a separate host on the same WiFi network, sending an ICMP
      echo to the device's WiFi IP elicits an echo reply within 5 s
- [ ] The wake event is recorded in `dmesg` (or pstore) with a
      cause-of-wake annotation identifying the MPM / SDC4-DAT1 source
- [ ] The device returns to a fully-functional userspace post-resume
      (no kernel panic, no driver lockup) — measured by issuing a normal
      shell command in a pre-existing session

**Dependencies:** R1, R2, R3

## Out of Scope

- General suspend-to-RAM functionality (covered by the MPM and SPM work,
  not this kit)
- Pattern-matched WoWLAN (e.g. magic-packet, ARP, specific TCP ports) —
  follow-up beyond "wake on any packet"
- Wake from other wake sources (PMIC keys, USB attach, etc.) — separate
  concerns
- WiFi performance / association-stability work — handled elsewhere

## Cross-References

- **Hard dependency:** `context/impl/impl-mpm-boot-hang.md`
  (MPM platform driver / wakeup-controller must land before R1 can pass)
- **Informational:** `context/kits/cavekit-spm-init.md` touches the same
  low-power state machinery on the same SoC, but is not a hard dependency
  (overview's dependency graph is authoritative)

## Source Traceability

- `reports/HARDWARE-QUIRKS-INVENTORY-2026-05-19.md` — Q5 (table row),
  "Net-new items" §Gap #12
- `reports/BOARDFILE-DEFCONFIG-AUDIT-2026-05-19.md` — Gap #12 (Executive
  Summary §1), §2.4 row "SDCC4 WiFi SDIO", §4 note for MPM work
- Legacy SDC4 wake-source wiring:
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:5417`
  (`msm_sdcc_cfg_mpm_sdiowakeup`), `:5426`
  (`pin = MSM_MPM_PIN_SDC4_DAT1`), `:5488`
  (`.cfg_mpm_sdiowakeup = msm_sdcc_cfg_mpm_sdiowakeup`)
- Current MPM-blocker analysis: `context/impl/impl-mpm-boot-hang.md`
  (Attempts 1-3, all dead-ends; current recommendation: convert to
  platform driver or write a minimal MSM8660-specific wakeup chip)
