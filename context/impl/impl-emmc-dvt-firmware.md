---
domain: emmc-dvt-firmware
created: "2026-05-17"
last_updated: "2026-05-17"
status: open
related:
  - impl-overview.md
---

# Implementation: eMMC EXT_CSD returns OTP-only on DVT/EVT cards

## Status: OPEN — affects pre-PVT Topaz hardware only; PVT cards work

A Topaz3G unit with **Samsung SEM32G PRV 9.0** (DVT/EVT-era eMMC firmware)
enumerates with capacity 0 B under mainline 6.18 + mmci-pl18x. A
production-PVT Topaz unit with **Samsung SEM32G PRV 11.0** boots normally
on the same kernel binary. WebOS legacy `msm_sdcc` enumerates the DVT
card correctly. So the bug is a sequencing mismatch between mainline
mmci-pl18x and Samsung eMMC firmware revision 9.0 — not voltage, not
DMA-engine plumbing, not the recent WiFi-regulator work.

## Symptom on DVT/EVT

Boot stops in initramfs:

```
mmc0: new MMC card at address 0001            ← no "high speed" prefix
mmcblk0: mmc0:0001 SEM32G 0 B                 ← capacity zero
initramfs: Waiting for mmcblk0p13...
initramfs FAILED: /dev/mmcblk0p13 not found
```

`/sys/kernel/debug/mmc0/mmc0:0001/ext_csd` returns a 512-byte buffer
with only **3 non-zero bytes**, bit-identical across multiple reads:

| Offset | Value | JEDEC field |
|--------|-------|-------------|
| 175 | 0x01 | `RST_n_FUNCTION` (OTP) |
| 214 | 0xb7 | `SEC_COUNT[2]` (factory) |
| 215 | 0x03 | `SEC_COUNT[3]` (factory) |

The dynamic / CMD6-writable fields (`EXT_CSD_REV` at 192,
`EXT_CSD_STRUCTURE` at 194, `CARD_TYPE` at 196, `HS_TIMING` at 185,
`BUS_WIDTH` at 183, `POWER_CLASS` at 187, `PARTITION_CONFIG` at 179,
`ERASE_TIMEOUT_MULT` at 222 …) are **all zero**. Without `EXT_CSD_REV`
the kernel treats the card as MMC v4.0, skips the SEC_COUNT branch,
falls back to CSD — and the CSD says `CSD_STRUCTURE=3` ("see EXT_CSD")
so capacity stays 0.

## Why this is firmware-specific, not driver-generic

Cross-device comparison on the **same** kernel binary
(`6.18.0-luneos-g8610aa3f35c8`, since-reverted diagnostic build):

| Device | SoC midr | SoC rev | CID (mfg/oem/name/**PRV**/serial/crc) | EXT_CSD non-zero | Capacity |
|--------|----------|---------|---------------------------------------|------------------|----------|
| DVT/EVT | 0x510f02d2 | 0.0 | `02 0100 53454d3332 47 **90** 70c3e3e0 bc6e` | 3 / 512 | 0 B |
| PVT | 0x510f02d2 | 0.0 | `02 0100 53454d3332 47 **b0** a9006225 eec` | 32 / 512 | 29.7 GiB |

Same Samsung manufacturer (0x02), same OEM (0x0100), same model (SEM32G,
`53454d3332`). The only differences in the CID are the **PRV byte**
(0x90 vs 0xb0, i.e. product revision 9.0 vs 11.0) and the serial. CSD
is byte-identical (`d00f00320f5903ffffffffff92404010`) on both.

The PVT card returns a healthy EXT_CSD with `EXT_CSD_REV=0x05` (v4.41),
`CARD_TYPE=0x03` (HS-26 + HS-52), and switches to `MMC_TIMING_MMC_HS`
at 48 MHz × 8-bit. The DVT card, on the same kernel, returns only the
3 OTP/factory bytes.

WebOS dmesg from the **same physical DVT device** (`/tmp/webos-dmesg.log`):

```
mmc0: Qualcomm MSM SDCC at 0x0000000012400000 irq 136,0 dma 18
mmc0: new high speed MMC card at address 0001
mmcblk0: mmc0:0001 SEM32G 29.7 GiB
```

WebOS enumerates the DVT card correctly. Therefore the firmware can
return a full EXT_CSD — it just needs a CMD8 sequence the older firmware
accepts. Mainline mmci-pl18x's CMD8 sequence is different from
`drivers/mmc/host/msm_sdcc.c` in some way that the 9.0 firmware does
not tolerate.

## Things ruled out by the cross-device test

These were considered during diagnosis and **falsified** by the PVT card
working on the same kernel:

1. **WiFi voltage / bus changes** (`4ad3478965ae`, `c572406e1e1c`,
   `0b79efd1c41c`). The DTS / regulator state is identical between
   DVT and PVT runs.
2. **vdd_dig / vdd_mem floor** (`5f7812c6baea`, `48aeede3bdfe`,
   `b3d215b043a5`, `54ae8959ef04`). PVT silicon revision is the same
   (rev 2 / midr 0x510f02d2) — these are not silicon-rev-specific.
3. **ADM DMA path** (`b3c0c60b56d7` full-FIFO burst, `qcom_adm.c`
   box-mode flow). EXT_CSD on DVT is read at the 400 kHz init clock,
   which is signal-margin-safe even with any plausible DMA descriptor
   issue. The bit-identical pattern across reads also rules out DMA
   noise. (A diagnostic that forced PIO via `dma_threshold = 4096`
   was committed as `8610aa3f35c8` and reverted as `67cf748b5981`
   once the cross-device test pinpointed the firmware variable.)
4. **CRCI / fabric unhalt** (`ca1687f77e89`). dmesg confirms all
   three fabrics (`MMSS 0-13`, `system 0-16`, `apps 0-3`) are
   unhalted before mmc probe at t≈1.35 s.

## Hypotheses for the firmware-rejected step

webOS `msm_sdcc.c` vs mainline `mmci-pl18x` differ in several ways that
could matter to a firmware that expects the legacy sequence:

| Aspect | webOS msm_sdcc | mainline mmci-pl18x |
|--------|---------------|---------------------|
| Init clock | 400 kHz, free-running | 400 kHz, with PWRSAVE-gated clkreg (gated off at 400 kHz, on above) |
| Power-up ramp | `udelay` between vmmc enable and CMD0 | regulator framework + `mmc_power_up` |
| CMD0 retries | Two CMD0 (GO_IDLE) with delay | Single CMD0 |
| CMD1 polling | Polled every 50 ms up to 1 s | `mmc_send_op_cond` retries 100× with ~10 ms backoff |
| CMD8 framing | `data->blocks=1 blksz=512`, no PIO/DMA decision (always DMA via ADM box) | `data->blocks=1 blksz=512`, DMA path with our `dma_threshold` rule |
| CMD8 DPSM bits | Sets data direction + transfer mode, then writes CMD register | Same path but with different `data_cmd_enable` (`MCI_CPSM_QCOM_DATCMD`) |
| Pre-CMD8 status | CMD13 (SEND_STATUS) between CMD7 and CMD8 | No explicit CMD13 between |
| DATATIMER | Set to 0xffffffff (max) before each data cmd | Set from `data->timeout_ns` (driver computes a smaller value) |
| Bus width during EXT_CSD read | 1-bit (CMD6 to switch to 8-bit happens AFTER EXT_CSD) | 1-bit (matches webOS) |

The most plausible candidates for "old firmware rejects":
1. Missing pre-CMD8 CMD13 status poll.
2. `DATATIMER` too short — at 400 kHz with `data->timeout_ns` default,
   the timer can fire while the card is still preparing the EXT_CSD
   response, causing the controller to retire the descriptor with a
   short transfer and no CRC error.
3. Power-up ramp / CMD0 timing too tight for the 9.0 firmware to
   complete its internal RAM initialization before CMD8.

## Plan

Each step is independent and can be tested as a separate commit on the
DVT device. Order is cheapest-first.

### Step 1: Long DATATIMER for EXT_CSD reads

Set the SDCC `DATATIMER` to `0xffffffff` for the CMD8 (and any other
CMD-with-data while still at the 400 kHz init clock) to remove timer
expiry as a candidate. Test: dmesg shows `EXT_CSD_REV=5` /
`CARD_TYPE=3` and `mmcblk0: SEM32G 29.7 GiB`.

### Step 2: Pre-CMD8 CMD13 status poll

Match the webOS sequence by issuing CMD13 between CMD7 (SELECT_CARD)
and CMD8 (SEND_EXT_CSD). Likely done as a mainline quirk
(`MMC_QUIRK_NO_EXT_CSD_NO_CMD13` reversed) or a small targeted
modification to `mmc_get_ext_csd`.

### Step 3: Doubled CMD0 + extra init delay

Issue CMD0 twice with a 1 ms gap, then wait 10 ms before CMD1.
Matches `msm_sdcc_set_ios(POWER_UP)` behaviour.

### Step 4: Lower init clock to 200 kHz

The 9.0 firmware may have looser timing requirements; running the
init at 200 kHz exaggerates the margin window so any timing-sensitive
step is more forgiving. This is purely diagnostic — would not stay in
final code.

### Step 5: Capture an ADM bus trace on DVT

If steps 1–4 don't isolate the cause, hook the mmci `dev_dbg` for
`MCI_DATATIMER`, `MCI_DATACTRL`, and `MCI_CMD` writes at probe time
and compare the exact register sequence to a similarly-instrumented
webOS msm_sdcc trace (already available at
`reports/mmci-legacy-deep-dive.md`).

## Validation plan

For each step, run the following on the DVT device after deploy:

```
# 1. Confirm card was detected and capacity is non-zero
cat /sys/class/mmc_host/mmc0/mmc0:0001/name        # SEM32G
cat /sys/block/mmcblk0/size                        # 62324736 expected

# 2. Confirm EXT_CSD has the runtime fields populated
mount -t debugfs none /sys/kernel/debug
cat /sys/kernel/debug/mmc0/mmc0:0001/ext_csd \
  | awk '{ s=$0; n=0; for(i=1;i<=length(s);i+=2) if(substr(s,i,2)!="00") n++; print n }'
# Healthy: >= 30. Broken: 3.

# 3. Confirm partitions enumerate
ls /dev/mmcblk0p*                                  # p1..p14 expected

# 4. Confirm initramfs hands off to LuneOS
hostname                                           # "tenderloin", not "(none)"
```

A step is considered to clear the bug only when all four pass on the
DVT card. Steps that don't fix it should be reverted before trying
the next step so the diff is clean per attempt.

## Addendum 2026-05-17: cross-rev DTS delta audit

After the first investigation suggested the recent voltage / WiFi
regulator commits, the legacy webOS kernel was audited end-to-end for
board-revision-conditional code paths (every subsystem). This rules
out the board-rev-misconfiguration class of explanation for the
EXT_CSD failure on the DVT/EVT WiFi unit.

### Topaz WiFi board-rev delta (webOS view)

The Topaz WiFi pre-DVT vs DVT+ delta in the entire webOS kernel
amounts to **one configuration swap**: the A6 IRQ GPIO pins.

  - `gpiomux-tenderloin.c:1711` `tenderloin_gpiomux_cfgs` (proto/EVT)
    contains `msm8x60_a6_configs`
  - `gpiomux-tenderloin.c:1817` `tenderloin_dvt_gpiomux_cfgs` (DVT+)
    contains `msm8x60_a6_configs_dvt`
  - every other entry (PMIC, UART, BT, WLAN, LCDC, sensor, lighting,
    kbdgpio, charger, touchscreen, cam, audio, aux_pcm,
    system_gpio, ctp) is byte-identical between the two tables.

The pin-table delta (`gpiomux-tenderloin.h:205 tenderloin_pins_wifi`
vs `:257 tenderloin_pins_wifi_dvt`) is the same single difference,
just expressed as a per-pin assignment:

  - `[TENDERLOIN_A6_0_MSM_IRQ_PIN] = TENDERLOIN_A6_0_MSM_IRQ` (proto:
    gpio156) -> `..._DVT` (DVT+: gpio37)
  - `[TENDERLOIN_A6_1_MSM_IRQ_PIN] = TENDERLOIN_A6_1_MSM_IRQ` (proto:
    gpio132) -> `..._DVT` (DVT+: gpio94)

The MAX8903B charger has a separate three-way split on PMIC current
table (`board-tenderloin.c:1186-1230`):
  - PROTO/PROTO2: 900/1000/1500/2000 mA
  - EVT1 only: 750/900/1500/1400 mA, with 2000 mA blocked -> 1500 mA
  - **EVT2+/DVT/PVT**: 750/900/2000/1400 mA

### Mainline DTS state vs the webOS rule

| Item | webOS rule | mainline DTS | match? |
|------|-----------|--------------|--------|
| A6_0 IRQ GPIO | proto: gpio156 / DVT+: gpio37 | `gpio37` (a6_0_default irq-pins) | DVT/PVT |
| A6_1 IRQ GPIO | proto: gpio132 / DVT+: gpio94 | `gpio94` (a6_1_default irq-pins) | DVT/PVT |
| A6 SBW (TCK/TDIO/WAKEUP) | same across all WiFi revs | gpio157/158/155 + gpio115/116/141 | match |
| MAX8903B current table | EVT2+/DVT/PVT order | `750000 0, 900000 1, 1400000 3, 2000000 2` (max8903b node) | EVT2+ |
| USUS_in polarity | inverted on >EVT1 | `usus-gpios = ... GPIO_ACTIVE_LOW` | >EVT1 |
| L12 (gyro 1.8 V) | enabled only WiFi-PVT (>DVT) | declared as supply on mpu3050 | needs runtime verify |
| 3G modem (mdmgpio / ISP1763) | only `boardtype_is_3g()` | qcom-apq8060-topaz-3g.dts only | match |

### Implication for this bug

Our mainline DTS is **fully calibrated for DVT/PVT WiFi boards**.
For the broken unit:

  - **If it is DVT**: the DTS is correct. The eMMC failure has to be
    driver-side (CMD8 sequencing vs Samsung firmware revision 9.0).
  - **If it is EVT1-3**: only A6 IRQ pins are wrong (gpio37/94 vs
    gpio132/156); that would cause battery/charging anomalies, not
    the EXT_CSD shape observed. Charger current table is
    EVT2+-compatible (works on EVT2-3, only EVT1 is special).

Either way the EXT_CSD-returns-3-OTP-bytes symptom is **not** board
revision miswiring -- the audit positively excludes that.

### How to determine EVT vs DVT on the broken unit (without opening)

Boot the diagnostic kernel and probe both candidate A6 IRQ pins:

```
mount -t debugfs none /sys/kernel/debug 2>/dev/null
# Pre-DVT A6_0 IRQ candidate
cat /sys/kernel/debug/gpio | grep -E "gpio-(156|37|132|94) "
# Then press the power button briefly and re-read; whichever pin
# shows a transient transition is the live A6 IRQ line.
```

DVT/PVT shows activity on gpio37/94, EVT1-3 shows it on gpio156/132.
Visible inspection of the device's QA sticker (see ../schematics/
page 12 marker resistor) also identifies the rev.

## References

- `reports/adm-dma-emmc-analysis.md` — prior ADM burst-size and
  CH_CONF investigation (different class of bug, but shares the
  background)
- `reports/mmci-legacy-deep-dive.md` — webOS msm_sdcc.c sequence trace
- `/tmp/webos-dmesg.log` — DVT device booting webOS successfully
- WebOS `msm_sdcc.c` source — `../webos-linux-kernel-touchpad/drivers/mmc/host/msm_sdcc.c`
- WebOS gpiomux source —
  `../webos-linux-kernel-touchpad/arch/arm/mach-msm/gpiomux-tenderloin.c`
- WebOS pin tables —
  `../webos-linux-kernel-touchpad/arch/arm/mach-msm/gpiomux-tenderloin.h`
- Reverted diagnostic commit: `67cf748b5981`
  (reverts `8610aa3f35c8` "bump qcom dma_threshold to 4096 to force PIO")
