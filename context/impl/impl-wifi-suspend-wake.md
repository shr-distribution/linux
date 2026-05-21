---
domain: wifi-suspend-wake
created: "2026-05-21"
last_updated: "2026-05-21"
status: r1-done-r2-dt-landed-r3r4-test-pending
---

# Implementation: WiFi Wake-From-Suspend (AR6003 SDIO via MPM)

Kit: `context/kits/cavekit-wifi-suspend-wake.md` (R1..R4)
Build site: `context/plans/build-site.md` (T-007 external, T-019, T-020, T-024)

## Status snapshot

| Req | Description | Status |
|-----|-------------|--------|
| R1 | MPM functional as wake controller | ✅ DONE (commit 993a638936e4; `impl-mpm-boot-hang.md`) |
| R2 | SDC4 DAT1 declared as MPM-routed wake source | 🟡 DT plumbing landed; on-device verification pending |
| R3 | WoWLAN configurable on AR6003 via cfg80211 | ⏸ Pending test |
| R4 | Inbound packet wakes device from suspend-to-RAM | ⏸ Blocked on R2 + R3 verification |

## R2: SDC4 DAT1 wakeup source — design choice

### Approach picked: mainline-standard MMC DT properties

Three documented MMC DT properties handle SDIO wake without any
Qualcomm-specific code:

| Property | Effect |
|----------|--------|
| `cap-sdio-irq;` | Controller advertises SDIO IRQ (DAT1) detection support |
| `keep-power-in-suspend;` | Preserve SDIO card power during suspend |
| `wakeup-source;` | Sets `MMC_PM_WAKE_SDIO_IRQ` on the host; mmc core calls `enable_irq_wake(host->irq)` during suspend |

These are defined in
`Documentation/devicetree/bindings/mmc/mmc-controller-common.yaml`
and used in production by mediatek/samsung SDIO WiFi platforms. Fully
upstream-friendly.

The first two were already present in `&sdcc4` in
`qcom-apq8060-tenderloin-common.dtsi`. `wakeup-source;` was added on
2026-05-21.

### Approaches considered and not taken

1. **Raw MPM pin 23 (DAT1) shim** — would replicate the legacy
   `msm_sdcc_cfg_mpm_sdiowakeup` callback that switches between
   GIC-routed SDC4 IRQ (level-high, normal operation) and MPM-watched
   DAT1 (level-low, wake mode). Requires either a vendor callback in
   the generic mmci driver or a separate Qualcomm-specific aux driver.
   Not mainline-shaped — rejected as the primary path.

2. **`interrupts-extended = <&msm8660_mpm 23 ...>` hierarchical routing** —
   requires adding `<23 101>` to `qcom,mpm-pin-map` (mapping pin 23 to
   SDC4 SPI 101). But pin 23 watches DAT1 (a different signal from
   SPI 101) and at a different polarity (level-low vs level-high).
   Conflicts with normal-operation IRQ flow. Rejected.

3. **Patch ath6kl-sdio directly to call `msm8660_mpm_set_pin_wake`** —
   tight coupling between ath6kl and msm8660-mpm; ath6kl is multi-platform
   and shouldn't carry SoC-specific wake plumbing. Rejected.

If the standard wake path (option above) doesn't carry through deep
suspend-to-RAM where the SDC4 controller is fully powered off, the
follow-up would be to write a small `mmci_qcom_wake.c` aux driver that
twiddles MPM pin 23 around suspend, mirroring legacy semantics. That
is OPTIONAL OPTIMIZATION, not required for R2/R3/R4 to pass with the
controller-stays-powered path.

## R2 verification plan

After the DT change lands (next Yocto rebuild):

```bash
# Confirm host pm_caps reflect the wake-source declaration
cat /sys/class/mmc_host/mmc1/device/of_node/wakeup-source     # exists
cat /sys/class/mmc_host/mmc1/power/wakeup                      # enabled

# Kernel log evidence
journalctl -b 0 | grep -iE "mmc1.*wake|MMC_PM_WAKE_SDIO_IRQ"

# AR6003 sees the host's wake capability
journalctl -b 0 | grep -iE "ath6kl.*pm|ath6kl.*wow"
```

## R3 / R4 plan

R3 — userspace only (no kernel changes needed):

```bash
iw phy phy0 wowlan enable any              # config
iw phy phy0 wowlan show                    # verify pre-suspend
# (suspend / resume)
iw phy phy0 wowlan show                    # config survived?
```

R4 — end-to-end test:

```bash
echo +5 > /sys/class/rtc/rtc0/wakealarm    # safety net
echo mem > /sys/power/state                # suspend-to-RAM
# from peer host: ping <device-wifi-ip>
# device should wake within 5 s
# verify /sys/power/wakeup_irq or pstore for cause-of-wake
```

## Commits

- `993a638936e4` — MPM driver rename + raw-pin API (unblocks R1)
- `0b5fc704069e` — context: MPM blocker resolved (R1 documented as done)
- (this commit) — ARM: dts: tenderloin: sdcc4: add wakeup-source for R2

## Cross-References

- **Kit:** `context/kits/cavekit-wifi-suspend-wake.md`
- **External dep (R1):** `context/impl/impl-mpm-boot-hang.md` (RESOLVED)
- **MMC binding:** `Documentation/devicetree/bindings/mmc/mmc-controller-common.yaml`
- **MPM consumer API (for follow-up Option B):** `include/soc/qcom/msm8660-mpm.h`
- **Legacy reference (raw-pin path):**
  `webos-linux-kernel-touchpad/arch/arm/mach-msm/board-tenderloin.c:5417`
  (`msm_sdcc_cfg_mpm_sdiowakeup`)
