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
