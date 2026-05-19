# ADM EE=0 ground truth on Tenderloin (APQ8060 / MSM8660)

**Date:** 2026-05-19
**Source:** /dev/mem dump of running webOS 2.6.35-palm kernel on HP TouchPad
**Tool:** `tools/webos-live-dump.c` (static ARM binary, runs via novacom)
**Raw dump:** `reports/adm-investigation/webos-live-dump.txt`

## TL;DR

- **EE=0 is the live CH_CONF / CRCI_CTL window on this SoC.** EE=1, EE=2, EE=3 all read zeros for these registers.
- Both legacy webOS and mainline preserve the **bootloader-programmed** CH_CONF values because their writes go to non-live EE windows (legacy `DMOV_SD_MASTER=1` = EE=1; mainline `qcom,ee=1`).
- **Bootloader explicitly programs WiFi at priority 6 and eMMC at priority 5**, so the WiFi channel wins arbitration. This is preserved by accident on both kernels.
- Mainline `qcom_adm.c` boot diagnostic was reading at `adev->ee=1` and returning all zeros (silently broken). Fixed to read at EE=0.

## Live CH_CONF values, all 16 channels of ADM1 (Tenderloin's eMMC + SDIO)

Decoded by the legacy webOS `dma.h` bit layout:
- bits 0-3: PRIORITY
- bit 4: SD bit 0; bit 5: SD bit 1; bit 14: SD bit 2
- bit 6: IRQ_EN; bit 7: FORCE_RSLT_EN; bit 11: MPU_DISABLE; bit 12: SHADOW_EN
- bits 26-28: per-channel attribute (set on ADM0 ch0-3 audio only, NOT SD bits)

| Channel | EE=0 value | Priority | SD | Bits 26-28 | Used by |
|---|---|---|---|---|---|
| ch0 | 0x000008D5 | 5 | 1 | — | unused |
| ch1 | 0x0C0008D5 | 5 | 1 | 0x3 | unused |
| **ch2** | **0x000008D5** | **5** | **1** | — | **sdcc1 eMMC (CRCI 1)** |
| ch3 | 0x000008D5 | 5 | 1 | — | unused |
| ch4 | 0x000008D6 | 6 | 1 | — | unused |
| **ch5** | **0x000008D6** | **6** | **1** | — | **sdcc4 SDIO/WiFi (CRCI 5)** |
| ch6 | 0x000008D6 | 6 | 1 | — | unused |
| ch7 | 0x000008D6 | 6 | 1 | — | unused |
| ch8 | 0x000008D6 | 6 | 1 | — | unused |
| ch9 | 0x000008D6 | 6 | 1 | — | unused |
| ch10 | 0x080208F6 | 6 | 3 | 0x2 | modem |
| ch11 | 0x04022846 | 6 | 0 | 0x1 | modem |
| ch12 | 0x000008C6 | 6 | 0 | — | modem |
| ch13 | 0x00002842 | 2 | 0 | — | modem |
| ch14 | 0x00002843 | 3 | 0 | — | modem |
| ch15 | 0x000008E4 | 4 | 2 | — | modem |

Note bit 12 (SHADOW_EN) is NOT set by bootloader on any channel. Legacy `config_datamover()` does write SHADOW_EN ON via its RMW pattern — but writes go to EE=1 and are silently dropped on this SoC, so SHADOW_EN never actually gets set in legacy either.

## Live CH_CONF values, ADM0 (Tenderloin's crypto + audio path)

| Channel | EE=0 value | Priority | SD | Bits 26-28 | Notes |
|---|---|---|---|---|---|
| ch0 | 0x180008D5 | 5 | 1 | **0x6** | audio path (LPASS) |
| ch1 | 0x180008D5 | 5 | 1 | 0x6 | audio path |
| ch2 | 0x180008D5 | 5 | 1 | 0x6 | audio path |
| ch3 | 0x180008D5 | 5 | 1 | 0x6 | audio path |
| ch4 | 0x000008D6 | 6 | 1 | — | crypto / general |
| ch5 | 0x000008D6 | 6 | 1 | — | crypto / general |
| ch6 | 0x000008D6 | 6 | 1 | — | (often QCE CE_IN) |
| ch7 | 0x000008D6 | 6 | 1 | — | (often QCE CE_OUT) |
| ch8-9 | 0x000008D6 | 6 | 1 | — | unused |
| ch10 | 0x000008F6 | 6 | 3 | — | secure |
| ch11 | 0x000308C6 | 6 | 0 | — | secure |
| ch12 | 0x000308C1 | 1 | 0 | — | secure |
| ch13 | 0x000C08F2 | 2 | 3 | — | secure |
| ch14 | 0x000C08F3 | 3 | 3 | — | secure |
| ch15 | 0x000008D4 | 4 | 1 | — | crypto |

**Bits 26-28 set only on ADM0 ch0-3** = the LPASS audio DMA channels. Hypothesis: these bits encode a low-latency / isochronous QoS class for audio. They are NOT TrustZone Security Domain bits as previously speculated.

## Live CRCI_CTL values, ADM1 (EE=0, offset 0x400)

| CRCI | Value | Notes |
|---|---|---|
| 0 | 0x00010000 | MUX_SEL=1 (bit 16) |
| 1 | 0x00000001 | sdcc1 eMMC — blk_size=1 |
| 2 | 0x00000001 | |
| 3 | 0x00000006 | blk_size=6 |
| 4 | 0x00000001 | |
| 5 | 0x00000001 | sdcc4 WiFi — blk_size=1 |
| 6 | 0x00000000 | unused |
| ... | | |
| 14 | 0x00000001 | |
| 15 | 0x00000006 | blk_size=6 |

CRCI 1 (eMMC) and CRCI 5 (WiFi) both have blk_size=1. The CRCI handshake itself is symmetric for the two SDCC slots; priority differentiation happens entirely at CH_CONF.

## What this falsifies and what it confirms

| Hypothesis | Verdict | Source |
|---|---|---|
| Bootloader gives WiFi higher priority than eMMC | **CONFIRMED** | Live ch5=pri6 vs ch2=pri5 |
| Mainline preserves bootloader values | **CONFIRMED** | We don't write CH_CONF |
| Legacy preserves bootloader values | **CONFIRMED** | Its writes hit EE=1 (dead window) |
| `qcom_adm` boot diagnostic shows real CH_CONF | **FALSIFIED** | Reads EE=1 = all zeros |
| Bits 27-28 are TrustZone SD (Gemini hypothesis) | **FALSIFIED** | SD bits are 4, 5, 14 per legacy header |
| Priority differential is the legacy-vs-mainline differentiator | **FALSIFIED** | Both kernels see identical bootloader values |
| Clock controller registers are HLOS-readable | **FALSIFIED** | All guessed addresses returned 0 — RPM-owned |

## What remains as the real legacy-vs-mainline differentiator

With bootloader priority ruled out, the structural difference is:

**Atomic submission via `exec_func`**. Legacy `msmsdcc_dma_exec_func()` writes all five SDCC registers (DATATIMER, DATALENGTH, DATACTRL, CMD_ARG, CMD_REG) inside the ADM `submit_lock`. Mainline `mmci_request()` writes them outside any cross-controller lock. Under heavy fabric contention, the gap between mmci's individual writes and the ADM's CMD_PTR write can be split by another controller's burst.

`include/linux/dma/qcom_adm.h` already exposes `exec_func` / `exec_user` in `qcom_adm_peripheral_config`. The qcom_adm submission path already calls it. **mmci just needs to be wired to use it.**

## Reproduction

```
# Build (host)
/opt/PalmPDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc \
    -O2 -static -o webos-live-dump tools/webos-live-dump.c

# Deploy to webOS (boot device into webOS via moboot first)
novacom put file:///tmp/webos-live-dump < webos-live-dump
novacom run file:///bin/sh -c "chmod +x /tmp/webos-live-dump && \
    /tmp/webos-live-dump > /tmp/webos-dump.txt"
novacom get file:///tmp/webos-dump.txt > webos-dump.txt
```
