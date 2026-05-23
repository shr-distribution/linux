# ADM DMA + MSM UART: legacy vs mainline findings & fix plan

**Date:** 2026-05-23
**SoC:** Qualcomm APQ8060 / MSM8660 (HP TouchPad), ADM = "Application Data Mover" (ADM3 generation)
**Drivers:** mainline `drivers/dma/qcom/qcom_adm.c`, `drivers/tty/serial/msm_serial.c`
**Legacy refs:** webOS 2.6.35 `arch/arm/mach-msm/{dma.c,msm_uart_dm.c,msm_hsuart.c}`, `drivers/misc/hsuart.c`; HTC MSM8960 3.4 (LineageOS cm-14.1) `arch/arm/mach-msm/dma.c`, `drivers/tty/serial/msm_serial_hs.c`

---

## 1. Symptoms this explains (all ADM1 read-completion failures)

- **BT (GSBI6 UART) RX DMA:** delivered the correct byte COUNT (from `UARTDM_RX_TOTAL_SNAP`, a HW register) but **ZERO data** — the chip's 12-byte SYNC bursts arrived as `00 00 .. 00`. PIO RX works.
- **WiFi (SDCC4 / mmc1):** `cmd53 ... error during DMA transfer`, `ath6kl probe failed -110`.
- **eMMC (SDCC1 / mmc0):** ext4/jbd2 writeback wedge (`folio_wait_writeback` never completes) under load.
- **CE2 crypto:** needed careful CRCI + EE=0 + per-op reset workarounds; AES capped at 4 blocks/op (chunked).

Common thread: the mainline `qcom_adm` driver mishandles **read/partial-flush completion** and **EE (security-domain) register banking** that the legacy `msm_dmov` got right.

---

## 2. GROUND TRUTH: EE register-bank split (verified via /dev/mem on webOS)

ADM register apertures are `base + 0x800*ee` (mainline `ADM_EE_MULTI=0x800`, `ADM_CHAN_MULTI=0x4` — identical to legacy `DMOV_SD_SIZE=0x800`, `ch<<2`). ADM0 base `0x18320000`, ADM1 base `0x18420000`.

**Live values read on the working webOS kernel (ADM1, `/tmp/rdmem_adm`):**

| Register (offset) | EE=0 (`+0`) | EE=1 (`+0x800`) | LIVE at |
|---|---|---|---|
| CONF (0x240) ch2 eMMC | `0x000008d5` | `0` | **EE=0** |
| CONF ch5/6/7 WiFi/BT | `0x000008d6` | `0` | **EE=0** |
| CRCI_CTL (0x400) crci1/5 eMMC/WiFi | `0x00000001` (blk=1) | `0` | **EE=0** |
| CRCI_CTL crci8/9 BT UART | `0` (blk=0 default) | `0` | EE=0 (default) |
| CMD_PTR (0x000) ch2 eMMC | `0` | `0x2fd57c60` | **EE=1** |

**Conclusion — the banks SPLIT:**
- **Config bank → EE=0:** `CONF` (0x240), `CRCI_CTL` (0x400) [and by extension CI_CONF/CRCI_CONF master regs].
- **Command bank → EE=1:** `CMD_PTR` (0x000), `RSLT` (0x040), `FLUSH_STATE0` (0x080), `STATUS` (0x200), `RSLT_CONF` (0x300).

**Implications for mainline (`adev->ee = 1` from `qcom,ee=<1>`):**
- ✅ `RSLT`/`STATUS` read at `adev->ee=1` — **correct** (command bank).
- ✅ `CMD_PTR` write at `adev->ee=1` — **correct** (command bank). The extra EE=0 dual-write (qcom_adm.c ~960) hits a dead mirror — harmless but pointless.
- ✅ `FLUSH_STATE0` at `adev->ee=1` — **correct** (command bank). (Fix #1's value was wrong, not its EE.)
- ❌ `CRCI_CTL` write at `adev->ee=1` (qcom_adm.c ~924) — **WRONG**, hits dead EE=1 mirror. Live CRCI_CTL is at **EE=0**. So mainline's SDCC block-size programming (blk_size=1) **never takes effect**; SDCC runs on whatever the bootloader left at EE=0. **Strong WiFi/eMMC suspect.**
- ⚠️ `CONF` (0x240): mainline skips CH_CONF writes; the boot diagnostic reads it at EE=0 (correct). If CONF is ever written, it must be at EE=0.
- The boot log `ADM ch0 (EE=0 live): CH_CONF=0x180008d5` matches webOS CONF low bits (`0x8d5`) — confirms EE=0 for CONF.

---

## 3. Legacy vs mainline behavioral gaps (deep code comparison)

### 3a. Graceful flush vs abort  *(Fix #1)*
- Legacy `msm_dmov_flush` writes `DMOV_FLUSH_TYPE = (1<<31)` = **graceful** → ADM drains in-flight data to memory and posts a result. HTC 3.4 `msm_hs_isr` does `msm_dmov_flush(chan, 1)` (graceful) on real RX stale, `0` (discard) only during termios reconfig.
- Mainline `adm_terminate_all` wrote **`0x0` = abrupt abort** → discards residual partial-box bytes before they retire to RAM. msm_serial calls this on RXSTALE to "retrieve" data → gets zeros.
- **FIX:** write `BIT(31)` (graceful) to `ADM_CH_FLUSH_STATE0(chan, adev->ee=1)`. ✅ done.

### 3b. FLUSH-state snapshot / partial residue not reported  *(Fix #2)*
- Legacy + HTC 3.4: on a FLUSH (non-DONE) result, `fill_errdata()` reads `FLUSH0..5` and passes the partial-transfer state to `complete_func(cmd, result, &errdata)`. Consumer recovers the true transferred count from HW, never trusting the descriptor's nominal length.
- Mainline `adm_dma_irq` reads `RSLT` once, treats FLUSH as benign (qcom_adm.c ~1045-1048), completes with **residue 0**. Consumer told "full transfer complete" on a flush.
- **FIX:** read `FLUSH_STATE0..5`, compute real residue, report via `dmaengine_desc_get_callback` / tx_status.

### 3c. Result-FIFO drain loop  *(Fix #3)*
- Legacy drains results in `do { } while (ch_status & DMOV_STATUS_RSLT_VALID)` — ADM can post DONE *and* FLUSH for one channel; both must be consumed.
- Mainline reads `RSLT` exactly once per channel per IRQ (qcom_adm.c ~1039). On a flow-controlled RX that completes a box then flushes, the 2nd result is left in the FIFO / lost (`curr_txd` already NULLed).
- **FIX:** loop reading `RSLT` while `RSLT_VALID` for the channel.

### 3d. CRCI_CTL at wrong EE  *(Fix #4 — hardware-confirmed)*
- See §2. `CRCI_CTL` write at `adev->ee=1` is dropped; must be **EE=0**. Also legacy programs `blk_size=1` for **all** SDCC CRCIs (1,2,4,5,14); mainline only forces CRCI 1/5. Set `SHADOW_EN` on CONF too (legacy `config_datamover` writes `conf | DMOV_CONF_SHADOW_EN`).
- **FIX:** write CRCI_CTL at EE=0; extend blk_size=1 to CRCI 2,4,14 if those SDCCs use DMA.

### 3e. RX buffer cache coherency  *(Fix #5 — UART-specific)*
- Legacy hsuart RX buffer = `dma_alloc_coherent` (uncached) → CPU always sees ADM writes. HTC 3.4 = non-coherent pool + `memset+mb()` before submit, `rmb()` before read.
- Mainline msm_serial RX buffer = `kzalloc` + streaming `dma_map_single(DMA_FROM_DEVICE)` (msm_serial.c ~380, ~641). If the ADM partial write doesn't retire before the invalidate, the CPU reads back the original kzalloc **zeros** while `RX_TOTAL_SNAP` (independent HW counter) reports the right count. Exact match for "correct count, zero data."
- **FIX:** switch RX buffer to `dma_alloc_coherent`, or add `dma_rmb()` + barrier discipline before reading in `msm_complete_rx_dma`. (Lower priority — BT RX is on PIO now; matters for restoring DMA-RX.)

### 3f. RX stale control-flow (NOT a bug — falsified)
- The "read-before-flush" hypothesis was falsified: both legacy and mainline read the RX buffer only from the post-flush completion callback (`msm_complete_rx_dma` after `dmaengine_terminate_all`). Ordering is equivalent. The divergence is the flush *type* (3a) + coherency (3e), not ordering.

---

## 4. Board-file quirks (webOS `dma.c` `config_datamover`, ADM3)

`CONFIG_MSM_ADM3=y` → `DMOV_SD_SIZE=0x800`, `DMOV_SD_AARM=1`, `DMOV_SD_MASTER=1`.

ADM1 channel/CRCI config webOS programs (`.sd` = security domain, `.blk_size`):
- **ADM1 channels** (incl. GSBI6 UART loc6/7, SDCC1 loc2, SDCC4 loc5): `DEFAULT_CONF` = `.sd=1, .block=0, .priority=0`; loc10-15 = MODEM `.sd=3`. CONF written with `DMOV_CONF_SHADOW_EN` set, only for `sd <= 1` channels.
- **ADM1 CRCI blk_size:** crci 1,2,4,5,14 → `blk_size=1`; crci 8,9 (UART) → `blk_size=0` (default). All `.sd=1`.
- webOS SDCC CRCI map: SDC1=1, SDC3=2, SDC2=4, SDC4=5, SDC5=14. GSBI6 UART TX/RX = CRCI 8/9.

DMA channel/CRCI cross-check (mainline DT vs webOS) — **all match** (global→ADM1-local = −16):
| Consumer | mainline | webOS | ok |
|---|---|---|---|
| Crypto IN | adm0 ch2 crci4 | CE_IN ch2 crci4 | ✅ |
| Crypto OUT | adm0 ch3 crci15(hash)/5(cipher runtime) | CE_OUT ch3 crci5, HASH 15 | ✅ (per-op) |
| GSBI10 UART RX (touch) | adm0 ch8 crci26 | HSUART2_RX ch8 crci(16+10)=26 | ✅ |
| eMMC SDCC1 | adm1 ch2 crci1 | SDC1 ch18→loc2 crci1 | ✅ |
| WiFi SDCC4 | adm1 ch5 crci5 | SDC4 ch21→loc5 crci5 | ✅ |
| BT UART TX/RX | adm1 ch6/7 crci8/9 | HSUART1 ch22/23→loc6/7 crci8/9 | ✅ |

No channel/CRCI *misassignment* anywhere — the gaps are in completion/flush/EE handling, not routing.

---

## 5. Fix plan (incremental, one commit each, test between)

| # | Fix | File | Status |
|---|-----|------|--------|
| 1 | Graceful flush (BIT 31) in `adm_terminate_all` at EE=1 | qcom_adm.c | ✅ committed `fe1c54809ec5` |
| 2 | Read `FLUSH_STATE0..5` on flush result, report real residue | qcom_adm.c | queued |
| 3 | Result-FIFO drain loop (handle DONE+FLUSH per IRQ) | qcom_adm.c | in progress |
| 4 | `CRCI_CTL` write at **EE=0** + blk_size=1 for CRCI 2,4,14 + `SHADOW_EN` | qcom_adm.c | queued (HW-confirmed) |
| 5 | msm_serial RX buffer `dma_alloc_coherent` / `dma_rmb()` | msm_serial.c | queued (for BT DMA-RX) |

**Recommended test signals after each:** WiFi `ath6kl` probe (-110 gone?), eMMC stability under load (no jbd2 writeback wedge), BT DMA-RX (real SYNC bytes vs zeros).

---

## 6. Tools / how to re-verify

- **webOS register read:** `reports/bt-trace/rdmem_adm.c` (freestanding, `-nostdlib -static`, build with `arm-linux-gnueabihf-gcc`). Push via `novacom put file:///tmp/rdmem_adm < rdmem_adm`, `novacom run file:///tmp/rdmem_adm`. Edit the `addrs[]` array for other registers. ADM addr = `base(0x18420000) + 0x800*ee + reg_off + chan*4`.
- **mainline device:** `/dev/mem` via `devmem`, or the in-kernel boot diagnostic `ADM ch%d (EE=0 live)`.
- **Module load without eMMC writes** (eMMC wedges on big writes): gunzip `.ko` to tmpfs and `insmod` from `/tmp` — don't `scp` to `/lib/modules` or `depmod`.

Cross-refs (memory): [[bcm4329-dma-vs-pio-root-cause]], [[mmci-dma-read-no-dataend]], [[tenderloin-dma-concurrent]], [[adm-ee0-ground-truth]], [[ce2-hash-resolved]].

---

## 7. ON-DEVICE RESULT: Fix #1 verified (2026-05-23)

Kernel `gfe1c54809ec5` (Fix #1 only). Verified live + via netconsole:
- **WiFi FIXED:** ath6kl downloads fw (`ar6003 hw 2.1.1 sdio fw 3.2.0.144 api 3`)
  and `wlan0` comes UP and passes traffic. The BMI firmware download is all
  `cmd53` data DMA on ADM1 — it failed before with `cmd53 data=yes -> error
  during DMA transfer -> ath6kl Failed to start hardware -110`. Graceful flush
  fixed it. `error during DMA transfer` on the WiFi channel: 0 this boot.
- **eMMC improved:** still 2 transient `error during DMA transfer` (mmc0/SDCC1
  at 328s/338s) but they now RECOVER — no more jbd2 writeback wedge / hang.
- Confirms the abort-vs-graceful root cause AND the EE=1 command-bank truth.

Remaining eMMC DMA errors -> next target is Fix #4 (CRCI_CTL blk_size at EE=0;
mainline writes it to the dead EE=1 mirror so SDCC pacing is wrong).
