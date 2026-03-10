# HP TouchPad WiFi (AR6003) Bringup Summary

**Last Updated:** 2026-01-29

## Hardware
- Atheros AR6003 Rev2 WiFi chip
- Connected via SDIO to SDCC4 (0x121c0000)
- Uses ADM DMA controller for data transfer

## What We've Tried

### 1. SDIO DMA Configuration
**Problem:** WiFi SDIO showed -110 timeout errors during BMI communication.

**Attempted Solutions:**
- Used shr-linux style 1-cell DMA with `qcom,rx-crci`/`qcom,tx-crci` properties
  - **Result:** FAIL - mainline mmci doesn't parse these properties
- Switched to ADM native 2-cell format: `dmas = <&adm_dma1 5 5>, <&adm_dma1 5 5>`
  - **Result:** DMA channels detected but released immediately

**Root Cause Found:** qcom_dma_setup in mmci_qcom_dml.c requires BOTH tx and rx DMA channels (lines 129-134).

**Fix:** Added both channels:
```dts
dmas = <&adm_dma1 5 5>, <&adm_dma1 5 5>;
dma-names = "rx", "tx";
```

### 2. eMMC DMA Issue
**Problem:** Adding DMA to SDCC1 (eMMC) caused boot hangs.

**Solution:** Removed DMA from SDCC1 entirely - PIO mode works reliably for eMMC.

### 3. Power Sequencing (pwrseq)
**Problem:** WiFi showed "bmi communication timeout" (-110) errors.

**Attempted Solutions:**
- 3-GPIO config (93, 137, 135) - FAIL, still timed out
- Added `post-power-on-delay-ms = <500>` - FAIL
- Added 32kHz sleep clock - FAIL

**Breakthrough:** Simplified to single GPIO 135 (reset line) with 200ms delay:
```dts
ath6kl_pwrseq: ath6kl-pwrseq {
    compatible = "mmc-pwrseq-simple";
    reset-gpios = <&tlmm 135 GPIO_ACTIVE_LOW>;
    clocks = <&sleep_clk>;
    clock-names = "ext_clock";
    post-power-on-delay-ms = <200>;
};
```
**Result:** BMI timeout FIXED! Error changed from -110 to -2 (ENOENT).

### 4. Firmware Format
**Problem:** After fixing BMI timeout, got "Magic is invalid, magic_len: 11" errors.

**Root Cause:** webOS firmware uses proprietary "SGMT" (Segmented) format. Mainline ath6kl expects "QCA-ATH6KL" IE (Information Element) format.

**Solution:** Use linux-firmware files instead of webOS firmware:
- fw-2.bin, fw-3.bin from linux-firmware (have proper QCA-ATH6KL header)
- bdata.SD32.bin from webOS (board-specific calibration data)

**Result:** "Magic is invalid" errors FIXED!

### 5. Built-in vs Module
**Problem:** ath6kl probed before firmware was available on boot partition.

**Solution:** Changed from CONFIG_ATH6KL=y to CONFIG_ATH6KL=m, added module loading to init script after firmware mount.

### 6. DMA Hang Issue (Build #129-130)
**Problem:** DMA transaction submitted but never completed. Driver hangs after first DMA, insmod never completes.

**Investigation:**
- With DMA: Hangs after first DMA transaction (no completion interrupt)
- With PIO: Gets -110 timeout during "write extended board data"

### 7. CRCI Flow Control - ROOT CAUSE FOUND

**Problem:** ADM DMA was hanging because CRCI (Client Request Control Interface) was not being passed to the ADM driver.

**Deep Investigation Findings:**

1. **`#dma-cells = <1>` limitation**: With `#dma-cells = <1>` in the ADM DMA controller node, the second cell in `<&adm_dma1 5 5>` is silently ignored by the DT parsing code. The `adm_dma_xlate()` function only receives `args_count=1`, so CRCI is set to 0.

2. **Missing `dma_flow_controller` flag**: The `variant_qcom` in mmci.c did NOT have `dma_flow_controller = true`, which meant `device_fc = false` was passed in the DMA slave config.

3. **ADM driver requires flow control for CRCI**: In `adm_prep_slave_sg()`, CRCI is only used when `achan->slave.device_fc` is true:
   ```c
   if (achan->slave.device_fc) {
       crci = achan->crci & 0xf;
       // ... use CRCI for flow control
   }
   ```

4. **Legacy kernel reference**: The webOS kernel defined `DMOV_SDC4_CRCI = 5` and `DMOV_SDC4_CHAN = 21` (channel 21 = ADM1 channel 5) in `arch/arm/mach-msm/include/mach/dma.h`.

**Why CRCI is Critical:**
Without CRCI, the ADM DMA controller has no flow control signal from the SDIO peripheral. The DMA doesn't know when the peripheral is ready for data transfer, causing it to hang waiting for a transfer completion that never comes.

**Solution - Three-Part Fix:**

1. **mmci.c - Enable flow control for QCOM variant:**
   ```c
   static struct variant_data variant_qcom = {
       // ... existing fields ...
       .dma_flow_controller = true,  // NEW: Enable device flow control
   };
   ```

2. **mmci.c - Pass CRCI via peripheral_config:**
   - Added `#include <linux/dma/qcom_adm.h>`
   - Added `crci` field to `struct mmci_dmae_priv`
   - Read CRCI from DT property `qcom,sdcc-crci` in `mmci_dmae_setup()`
   - Pass CRCI via `peripheral_config` in `_mmci_dmae_prep_data()`:
   ```c
   struct qcom_adm_peripheral_config periph_conf = {};
   if (dmae->crci) {
       periph_conf.crci = dmae->crci;
       conf.peripheral_config = &periph_conf;
       conf.peripheral_size = sizeof(periph_conf);
   }
   ```

3. **Device Tree - Add CRCI property:**
   ```dts
   &sdcc4 {
       dmas = <&adm_dma1 5>, <&adm_dma1 5>;
       dma-names = "rx", "tx";
       qcom,sdcc-crci = <5>;  // NEW: CRCI for SDC4
   };
   ```

**Note on #dma-cells:**
We cannot change `#dma-cells` from 1 to 2 because it breaks the touchscreen UART DMA which shares ADM1. The `qcom,sdcc-crci` property approach follows the same pattern used by `msm_serial.c` with `qcom,tx-crci`/`qcom,rx-crci`.

### 8. Data/Command Ordering Issue - NEW FIX (2026-01-19)

**Problem:** Even with CRCI fix, WiFi firmware upload still times out after 4-5 successful transfers.

**Failure Pattern:**
```
1. ath6kl_sdio probes mmc1:0001:1
2. Chip ID read succeeds
3. OTP upload starts (3998 bytes)
4. BMI LZ stream begins
5. 4 x 256-byte chunks write successfully
6. 5th credit register read times out (-110)  ← TIMEOUT
```

**Investigation:** Compared legacy `msm_sdcc.c` driver with mainline `mmci.c`:

**Legacy msm_sdcc.c behavior (DMA mode):**
```c
msmsdcc_dma_exec_func() {
    // Called when DMA is ready
    writel(host->cmd_timeout, host->base + MMCIDATATIMER);
    writel(host->curr.xfer_size, host->base + MMCIDATALENGTH);
    writel(host->cmd_datactrl, host->base + MMCIDATACTRL);
    msmsdcc_delay(host);  // Wait for data path ready

    msmsdcc_start_command_exec(host, ...);  // THEN send command
}
```

**Mainline mmci.c behavior (WITHOUT datactrl_first):**
```c
mmci_request() {
    // For READ: data setup first, then command
    // For WRITE: command first, data setup after!
    if (mrq->data &&
        (host->variant->datactrl_first || mrq->data->flags & MMC_DATA_READ))
        mmci_start_data(host, mrq->data);

    mmci_start_command(host, mrq->cmd, 0);
}
```

**Root Cause:** The `variant_qcom` was missing `datactrl_first = true`, causing WRITE operations (like firmware upload) to send the command BEFORE the data path was ready. The AR6003 expects the data path to be set up before the command completes.

**Fix:** Add `datactrl_first = true` to `variant_qcom` in `drivers/mmc/host/mmci.c`:

```c
static struct variant_data variant_qcom = {
    // ... existing fields ...
    /*
     * Legacy msm_sdcc driver sets up data registers before sending command
     * in DMA mode (via msmsdcc_dma_exec_func callback). Match this behavior
     * by enabling datactrl_first to fix SDIO timeouts during WiFi firmware
     * upload on devices like HP TouchPad.
     */
    .datactrl_first     = true,
};
```

**Reference:** Similar fix in upstream commit 66b512ed ("mmc: atmel-mci: fix timeout errors in SDIO mode when using DMA") addressed the same class of timing issue on Atmel hardware.

**Commit:** `1d37e9cbbfc3` - mmc: mmci: qcom: Enable datactrl_first to fix SDIO timeouts

### 9. Block Size Encoding & CRC Errors (2026-01-26)

**Problem:** SDIO data transfers failed with CRC errors (-84 EILSEQ).

**Investigation:** Compared DATACTRL register encoding between legacy msm_sdcc and mainline mmci:

| Driver | Block Size Encoding | For blksz=12 |
|--------|---------------------|--------------|
| msm_sdcc (legacy) | `blksz << 4` (raw bytes) | 0xC0 |
| mmci (standard ARM) | `log2(blksz) << 4` | 0x30 (wrong!) |

The standard ARM PL180 MMCI expects log2(block_size) in bits 4-7 of DATACTRL. But Qualcomm SDCC expects the raw byte count, allowing arbitrary block sizes for SDIO byte mode (CMD53).

**Fix:** The `qcom_get_dctrl_cfg()` in `mmci_qcom_dml.c` already uses raw byte encoding:
```c
static u32 qcom_get_dctrl_cfg(struct mmci_host *host)
{
    return MCI_DPSM_ENABLE | (host->data->blksz << 4);
}
```

This is loaded via `qcom_variant_init()` which sets `host->ops = &qcom_variant_ops`.

### 10. DMA CRC Error Root Cause & Fix (2026-01-26 → 2026-01-27)

**Key Finding:** PIO mode works, DMA mode initially failed with CRC errors!

**Testing Results (before fix):**

| Mode | Block Sizes | Result |
|------|-------------|--------|
| PIO | 4, 12, 24, 128, 256 bytes | ✓ All succeed |
| DMA | 256+ bytes | ✗ CRC error at end of transfer |

**Debug output with DMA (before fix):**
```
DMA datactrl=0x1009 blksz=256 size=256
error during DMA transfer!
MCI ERROR IRQ, status 0x00000002 at 0x00000100
DATACRCFAIL: blksz=256 blocks=1 flags=0x100
```

**Root Cause: DMA issues before CMD53 sends**

The `qcom,datactrl-first` property causes DATACTRL to be written before CMD53.
When DMA `issue_pending` is called immediately after DATACTRL, the ADM fills
the SDCC FIFO before the card knows data is coming (CMD53 hasn't been sent).
The card receives data without a preceding command → CRC fail.

The legacy msm_sdcc driver used an atomic `exec_func` callback:
```c
msmsdcc_dma_exec_func() {
    writel(DATACTRL);   // 1. Enable DPSM
    msmsdcc_delay();    // 2. Wait for DPSM init
    start_command();    // 3. Send CMD53
    // DMA is enabled at this point - fills FIFO AFTER cmd
}
```

**Fix: Deferred DMA issue_pending** (`mmci.c`, `mmci_qcom_dml.c`)

Split `mmci_dmae_start()` into `mmci_dmae_submit()` + `mmci_dmae_issue_pending()`:

1. `mmci_dma_start()`: Submit DMA descriptor, write DATACTRL, but **defer** `issue_pending`
2. `mmci_start_command()`: Send CMD53
3. `mmci_cmd_irq()`: After CMD53 response, **issue** the deferred DMA

This gives the correct sequence: **DMA submit → DATACTRL → CMD53 → DMA issue**

```c
// In mmci_dma_start():
if (host->datactrl_first && !(data->flags & MMC_DATA_READ) &&
    host->variant->qcom_dml) {
    host->dma_issue_deferred = true;  // Don't issue yet
} else {
    host->ops->dma_issue_pending(host);  // Issue immediately
}

// In mmci_cmd_irq():
if (host->dma_issue_deferred) {
    host->dma_issue_deferred = false;
    host->ops->dma_issue_pending(host);  // Issue after CMD response
}
```

**Result: CRC errors ELIMINATED.** 665+ DMA transfers complete with zero errors.

**Also fixed:** Clear `dma_issue_deferred` flag in error paths to prevent stale
DMA issue after `mmci_dma_error()` terminates the channel.

### 11. Additional Qualcomm SDCC Fixes (2026-01-26)

**a) Remove STARTBITERR for Qualcomm:**

The legacy msm_sdcc driver does NOT enable STARTBITERR in the interrupt mask. Enabling it causes spurious -ECOMM errors:
```c
static struct variant_data variant_qcom = {
    /* Do NOT set start_err for Qualcomm SDCC */
    .opendrain = MCI_ROD,
    // .start_err = MCI_STARTBITERR,  // REMOVED
};
```

**b) Double data timeout:**

Legacy driver uses `clks * 2` for data timeout:
```c
if (variant->qcom_data_timeout_2x)
    clks *= 2;
```

**c) Delay between ARGUMENT and COMMAND:**

Legacy driver uses `msmsdcc_delay()` between register writes:
```c
writel(cmd->arg, base + MMCIARGUMENT);
if (host->variant->qcom_datactrl_delay)
    udelay(1);
writel(c, base + MMCICOMMAND);
```

### 12. WMI Timeout Issue (2026-01-26) - RESOLVED via SDIO IRQ

**Problem:** With PIO mode working, ath6kl gets further but fails at WMI initialization:
```
ath6kl: wmi is not ready or wait was interrupted: -512
ath6kl: Failed to start hardware: -5
```

**Root Cause:** SDIO IRQ was not working. The Qualcomm SDCC variant needed
`supports_sdio_irq` added and the SDIO IRQ bit (`MCI_ST_SDIOIT`) properly
handled in the interrupt handler.

**Fix:** Added SDIO IRQ support to `variant_qcom` and implemented
`mmci_signal_sdio_irq()` for the Qualcomm SDCC variant.

### 13. DMA Firmware Upload Failure (2026-01-27)

**Problem:** With DMA CRC errors fixed (deferred issue), all DMA transfers
complete successfully (665+ writes, zero errors), but the WiFi chip becomes
unresponsive after firmware upload.

**Workaround:** Reduced BMI max_data_size to 52 bytes so ALL BMI transfers
stay at or below 64 bytes, which forces PIO mode (Qualcomm SDCC uses PIO for
transfers ≤ 64 bytes). Firmware upload now takes ~78 seconds via PIO but
completes successfully every time.

**Code change (sdio.c):**
```c
ar->bmi.max_data_size = 52;  // Force PIO by keeping all transfers ≤64 bytes
```

### 14. WMI Power Save Issue - Firmware Goes to Sleep (2026-01-27) - FIXED

**Problem:** After firmware upload succeeds, WiFi scan returns -16 (EBUSY).
Debugfs showed `Error Int status: 0x4` = `ERROR_INT_STATUS_WAKEUP`.

**Root Cause:** Firmware defaults to REC_POWER (recommended power mode) and
enters deep sleep after initialization. SDIO WAKEUP interrupts confirm the
chip is asleep and not processing WMI commands.

**Fix 1 - init.c:** Added `ath6kl_wmi_powermode_cmd(MAX_PERF_POWER)` at the
end of `ath6kl_target_config_wlan_params()`:
```c
ret = ath6kl_wmi_powermode_cmd(ar->wmi, idx, MAX_PERF_POWER);
```

**Fix 2 - cfg80211.c:** The interface UP path (cfg80211 `set_power_mgmt`
callback) was OVERRIDING our MAX_PERF_POWER with REC_POWER. Fixed by forcing
MAX_PERF_POWER for SDIO devices:
```c
if (pmgmt && ar->hif_type != ATH6KL_HIF_TYPE_SDIO) {
    mode.pwr_mode = REC_POWER;
} else {
    mode.pwr_mode = MAX_PERF_POWER;  // Force for SDIO
}
```

**Result:** Scan no longer returns EBUSY. Firmware stays awake.

### 15. WiFi Scan - BEGIN_SCAN Broken, START_SCAN Works (2026-01-28) - FIXED

**Problem:** User-triggered WiFi scan returned 0 APs despite the firmware's
autonomous scan finding 12+ APs on 2.4GHz and 5GHz bands.

**Root Cause:** The AR6003 firmware (version 3.2.0.144 api 5) reports
`sta-p2pdev-duplex` capability, causing ath6kl to use `WMI_BEGIN_SCAN_CMDID`
(the P2P-enhanced scan). However, this firmware does NOT respond to
BEGIN_SCAN - no WMI_BSSINFO_EVENTID events, no WMI_SCAN_COMPLETE_EVENTID.

**Fix (wmi.c):** Force legacy `WMI_START_SCAN_CMDID` for SDIO devices:
```c
if (!test_bit(ATH6KL_FW_CAPABILITY_STA_P2PDEV_DUPLEX,
              ar->fw_capabilities) ||
    ar->hif_type == ATH6KL_HIF_TYPE_SDIO) {
    return ath6kl_wmi_startscan_cmd(...);
}
```

**Result - 10 APs found on first scan:**
```
AP 1: cc:28:aa:a1:45:80  2417 MHz  -65 dBm  "HerrieVlada"
AP 3: 3e:97:f6:01:b9:f8  2437 MHz  -55 dBm  "Herrie-Guest-Free WiFi_24G"
AP 4: 34:97:f6:01:b9:f8  2437 MHz  -57 dBm  "Herrie_2.4GHz"
AP 10: 3e:97:f6:01:b9:fc 5180 MHz  -59 dBm  "Herrie-Guest-Free WiFi_5G"
```

Both 2.4GHz and 5GHz bands are working. Endpoint stats show healthy credit
flow: tx_issued=28, cred_retnd=28, rx_pkts=54, no starvation.

**CMDTIMEOUT issue (separate, not yet resolved):**
Occasional `CMDTIMEOUT: cmd53` errors cause WMI credit starvation:
```
CMDTIMEOUT: cmd53 arg=0x14080018 status=0x00000004 data=yes
```
After a CMDTIMEOUT, EP1 (WMI) credits drop to 0 with qdepth=37+, causing
all subsequent WMI commands to queue indefinitely.

**rfkill observation:** On some boots, rfkill soft-blocks the interface
(soft=1). Unblocking via `echo 0 > /sys/class/rfkill/rfkill0/soft` is
needed before `ip link set wlan0 up` works. Not consistent between boots.

### 16. Firmware Upload Performance - PIO Threshold Optimization (2026-01-29) - FIXED

**Problem:** Firmware upload took ~78 seconds with `max_data_size=52` (64-byte PIO).

**Investigation:** The MMCI DMA/PIO decision is based on `variant->fifosize` (64
bytes for Qualcomm SDCC). To allow larger PIO transfers without affecting the
FIFO mechanics, added a separate `dma_threshold` field to `variant_data`.

**PIO size testing results:**

| Transfer Size | PIO Fills | Upload Time | Result |
|--------------|-----------|-------------|--------|
| 64 bytes (52+12) | 1x FIFO | ~78s | **OK** |
| 128 bytes (116+12) | 2x FIFO | ~34s | **OK** |
| 192 bytes (180+12) | 3x FIFO | - | **FAIL** |
| 256 bytes (244+12) | 4x FIFO | - | **FAIL** |

PIO transfers > 128 bytes fail because the Qualcomm SDCC DPSM times out
before the interrupt-driven PIO handler can refill the 64-byte FIFO 3+ times
with `datactrl_first=true`.

**Fix (mmci.h, mmci.c, sdio.c):**
1. Added `dma_threshold` to `variant_data` struct (defaults to fifosize)
2. Set `dma_threshold=256` for `variant_qcom` — transfers ≤256 bytes use PIO
3. Raised `max_data_size` from 52 to 116 bytes (128-byte total transfers)

**Result:** Firmware upload improved from **78s to 34s** (2.3x speedup).

### 17. rfkill Soft-Block Race Condition (2026-01-29) - FIXED

**Problem:** WiFi interface randomly soft-blocked between boots. `rfkill0/soft`
showed `1` on some boots, requiring manual unblocking before `ip link set wlan0 up`.

**Root Cause:** cfg80211 allocates an rfkill device but never calls
`rfkill_init_sw_state()` before `rfkill_register()`. Without explicit init,
`rfkill_register()` schedules async `sync_work` that races with userspace.
The initial state is non-deterministic depending on when `sync_work` runs.

**Fix (net/wireless/core.c):**
```c
rfkill_init_sw_state(rdev->wiphy.rfkill, false);
```
Added before `rfkill_register()` — makes initial state synchronous and
deterministic (always unblocked).

**Result:** rfkill `soft=0` on every boot tested.

## Current DT Configuration

```dts
&sdcc4 {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&sdcc4_pins>, <&wlan_gpios>;
    max-frequency = <24000000>;
    qcom,datactrl-first;
    cap-sdio-irq;
    keep-power-in-suspend;
    vmmc-supply = <&pm8901_l1>;
    vqmmc-supply = <&pm8058_s3>;
    mmc-pwrseq = <&ath6kl_pwrseq>;

    /* DMA re-enabled with deferred issue fix */
    dmas = <&adm_dma1 5>, <&adm_dma1 5>;
    dma-names = "rx", "tx";
    qcom,sdcc-crci = <5>;

    wifi@1 {
        compatible = "atheros,ath6kl";
        reg = <1>;
        atheros,board-id = "SD32";
    };
};
```

## Debug Instrumentation

**mmci.c** (DMA transfer tracking):
- `DMA#N RD/WR datactrl=... blksz=... size=...` - Per-transfer counter with direction
- `deferred DMA issue: cmdN status=... resp=...` - Deferred issue confirmation with SDCC status
- `DATAEND#N status=... err=... bytes=...` - Data completion counter

**qcom_adm.c** (ADM DMA tracking):
- `ADM cpl#N chan=... crci=... result=...` - Per-completion counter with result
- `ADM DMA error: chan=... result=...` - Error detail on failure
- Box descriptor dump and CRCI_CTL logging (dev_dbg level)

## Files Modified

- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - DT config with DMA + CRCI
- `arch/arm/configs/tenderloin_defconfig` - ATH6KL=m
- `arch/arm/configs/tenderloin_debug_defconfig` - Regulatory config, CFG80211_DEBUGFS
- `drivers/mmc/host/mmci.c` - Deferred DMA issue, CRCI support, timing fixes
- `drivers/mmc/host/mmci.h` - `dma_issue_deferred` flag, `dma_issue_pending` callback
- `drivers/mmc/host/mmci_qcom_dml.c` - ADM submit/issue split, `qcom_dma_issue_pending()`
- `drivers/dma/qcom/qcom_adm.c` - Debug output, completion tracking
- `drivers/net/wireless/ath/ath6kl/init.c` - MAX_PERF_POWER mode, non-interruptible WMI wait
- `drivers/net/wireless/ath/ath6kl/sdio.c` - BMI max_data_size=116 for PIO (128-byte transfers)
- `drivers/net/wireless/ath/ath6kl/cfg80211.c` - Force MAX_PERF_POWER for SDIO on set_power_mgmt
- `drivers/net/wireless/ath/ath6kl/wmi.c` - Force START_SCAN for SDIO devices
- `net/wireless/core.c` - Fix rfkill init state race condition
- `scripts/test-wifi.sh` - WiFi test script with module deployment, wifi_tool + regulatory.db
- Firmware: linux-firmware fw-2.bin/fw-3.bin + webOS bdata.SD32.bin

## Key Commits

| Commit | Description |
|--------|-------------|
| `9cc04a36ff1a` | WIP: ath6kl: Fix WiFi scan on AR6003 SDIO (power mgmt + scan command) |
| `37d55b9678c0` | WIP: ath6kl/cfg80211: Fix WiFi scan on HP TouchPad (msm8x60) |
| `5f891c33491d` | WIP: mmc: mmci: Add deferred DMA issue for Qualcomm ADM writes |
| `72442e7a8f02` | WIP: mmc: mmci: Fix Qualcomm SDCC block size encoding and timing |
| `69ccd450c385` | WIP: mmc: mmci: Add Qualcomm SDCC data timing delays |
| `91e5b97abefa` | WIP: mmc: mmci: Add SDIO IRQ support for Qualcomm and fix SDIO bit |

## Current Status Summary

| Component | Status | Notes |
|-----------|--------|-------|
| SDIO bus (PIO) | **WORKING** | All transfers succeed |
| SDIO bus (DMA reads) | **WORKING** | No errors |
| SDIO bus (DMA writes) | **DEFERRED** | Deferred issue workaround; occasional CMDTIMEOUT |
| Firmware upload | **WORKING** | ~34s via PIO (max_data_size=116, was 78s at 52) |
| WMI initialization | **WORKING** | Firmware reports ar6003 hw 2.1.1 fw 3.2.0.144 api 5 |
| Radio/RF | **WORKING** | Autonomous scan finds 12+ APs on 2.4GHz |
| User-triggered scan | **WORKING** | Forced START_SCAN; finds 10+ APs on 2.4+5GHz |
| Power management | **FIXED** | MAX_PERF_POWER forced for SDIO |
| rfkill init state | **FIXED** | Always unblocked on boot (was random) |

## Next Steps

1. **Fix CMDTIMEOUT and credit starvation**
   - Occasional CMD53 timeouts cause EP1 credit loss
   - After timeout: qdepth accumulates, credits=0, all WMI operations fail
   - No credit recovery exists in the SDIO error path; `htc_tx_comp_update()` can
     reclaim credits on send failure but SDIO timeouts don't trigger it
   - Possible fix: detect `-ETIMEDOUT` in `ath6kl_sdio_scat_rw()` and force
     credit reclaim for failed transfers

2. **DMA firmware upload investigation** (deferred)
   - PIO workaround is stable; DMA upload corrupts firmware data
   - Multiple potential causes: ADM BOX descriptor misalignment for non-burst-aligned
     byte-mode SDIO transfers, missing `dma_sync_sg_for_cpu()` in finalization,
     bounce buffer allocated with `kzalloc()` instead of `dma_alloc_coherent()`
   - Low priority since PIO at 128 bytes works reliably (~34s upload)

3. **Clean up and upstream preparation**
   - Remove debug prints and rate-limit remaining ones
   - Clean up WIP commits into proper patch series
   - Consider upstream submission for MMCI Qualcomm fixes

---

## Firmware Analysis (2025-01-17)

### webOS vs linux-firmware Comparison

The webOS firmware in `/lib/firmware/ath6k/hw2.1.1/` is incompatible with mainline ath6kl for several reasons:

#### Missing API Firmware Files (Critical)

Mainline ath6kl expects fw-2.bin/fw-3.bin which are absent from webOS firmware:

| File | webOS | linux-firmware | Status |
|------|-------|----------------|--------|
| fw-3.bin | Missing | 104,562 bytes | **REQUIRED** |
| fw-2.bin | Missing | 89,844 bytes | **REQUIRED** |
| athwlan.bin | 70,612 bytes | 68,975 bytes | Different format |
| otp.bin | 2,822 bytes | 2,783 bytes | Different |
| data.patch.bin | 172 bytes | 140 bytes | Different |

The mainline driver loads firmware in order: `fw-4.bin` → `fw-3.bin` → `fw-2.bin` → `athwlan.bin`

#### Firmware Format Difference

webOS `athwlan.bin` uses "SGMT" (Segmented) format:
```
00000000: 5347 4d54 0100 0000 0050 5400  SGMT.....PT.
```

linux-firmware uses "QCA-ATH6KL" IE (Information Element) format expected by mainline ath6kl.

#### Directory Structure

| webOS Path | Mainline Expected Path |
|------------|------------------------|
| `ath6k/hw2.1.1/` | `ath6k/AR6003/hw2.1.1/` |

### Board Data (bdata.SD32.bin) Analysis

Both files are 1,792 bytes but contain different calibration data:

#### Header Differences

| Offset | webOS | linux-firmware | Meaning |
|--------|-------|----------------|---------|
| 0x04-0x07 | `3793 0960` | `1375 0760` | Board revision |
| 0x0A-0x0B | `440c` | `400e` | Config flags |
| 0x18-0x1B | `fed5 68a4` | `7f04 028f` | MAC address fragment |
| 0x20-0x30 | `ff...` | `SD3242D_0655a00` | Board ID string |

#### TX Power Calibration Tables

The bdata contains per-channel TX power calibration. ASCII-encoded power level offsets differ:

**webOS (HP TouchPad factory calibration):**
```
IHHJICB, HHHIHBB, GGGHGAA, GFFGF@?, GGFHG@@, GGGGFAA...
```

**linux-firmware (generic reference board):**
```
ILNKPGO, JMNKPIQ, GIKGLEL, HJLHMCJ, GJLHL@G...
```

#### 2.4GHz Power Backoff Data

webOS bdata contains detailed per-rate power backoff values at offset 0xA0 that linux-firmware lacks:
```
Xh..","$)%)-2<I9>56DL
```

#### Device Identification

Both share factory QA identifier at offset 0x2A0:
```
FABE0681257
```

### Recommended Firmware Configuration

For optimal HP TouchPad WiFi operation with mainline kernel:

```
/lib/firmware/ath6k/AR6003/hw2.1.1/
├── fw-3.bin          ← linux-firmware (mainline format)
├── fw-2.bin          ← linux-firmware (mainline format)
├── athwlan.bin       ← linux-firmware (mainline format)
├── otp.bin           ← linux-firmware
├── data.patch.bin    ← linux-firmware
├── endpointping.bin  ← linux-firmware
├── bdata.SD31.bin    ← linux-firmware
├── bdata.SD32.bin    ← webOS (HP TouchPad RF calibration)
├── bdata.WB31.bin    ← linux-firmware
└── bdata.bin         → bdata.SD32.bin (symlink)
```

**Rationale:** Use mainline-compatible firmware binaries (fw-3.bin, otp.bin) with HP TouchPad-specific RF calibration data (bdata.SD32.bin) for optimal performance.

### Dynamic Board Data Symlink (webOS)

webOS used a runtime-generated board data file:
```
token_bdata.SD32.bin -> /tmp/ath6k/token_bdata.SD32.bin
```

This suggests webOS may have dynamically patched board data at boot. For mainline, we use the static `bdata.SD32.bin` directly.

### Checksums

```
webOS bdata.SD32.bin:        999762b1922a2dd8635f281365ac73eb
linux-firmware bdata.SD32.bin: b859aac5ac533aee2ddacd71ddead817
```
