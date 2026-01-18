# HP TouchPad WiFi (AR6003) Bringup Summary

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

## Current DT Configuration

```dts
&sdcc4 {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&sdcc4_pins>, <&wlan_gpios>;
    vmmc-supply = <&pm8901_l1>;
    vqmmc-supply = <&pm8058_s3>;
    mmc-pwrseq = <&ath6kl_pwrseq>;
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

## Debug Output Added

Debug prints added to `drivers/dma/qcom/qcom_adm.c`:
- `adm_dma_xlate`: Shows channel/args_count/CRCI parsing
- `adm_slave_config`: Shows device_fc and peripheral_size
- `adm_prep_slave_sg`: Shows device_fc, CRCI, and burst values
- `adm_start_dma`: Shows CRCI_CTL register configuration
- `adm_dma_irq`: Shows IRQ status when interrupt fires

Debug print in `drivers/mmc/host/mmci.c`:
- `mmci_dmae_setup`: Shows DMA channels and CRCI value read from DT

## Files Modified

- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - DT configuration with CRCI
- `arch/arm/configs/tenderloin_defconfig` - ATH6KL=m
- `drivers/mmc/host/mmci.c` - CRCI support for QCOM ADM DMA
- `drivers/dma/qcom/qcom_adm.c` - Debug output for DMA operations
- `scripts/initramfs/init` - WiFi module loading
- Firmware: linux-firmware fw-2.bin/fw-3.bin + webOS bdata.SD32.bin

## Next Steps

1. Build and test with CRCI fix
2. Verify debug output shows:
   - `ADM xlate: chan=5 args_count=1 crci=0` (from DT)
   - `ADM slave_config: device_fc=1 peripheral_size=4 crci=5` (from MMCI)
   - `ADM prep_slave_sg: device_fc=1 crci=5`
   - `ADM start_dma: setting CRCI_CTL(5) = ...`
   - `ADM IRQ: srcs=0x...` (DMA completion)
3. If DMA works, WiFi initialization should proceed past firmware upload

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
