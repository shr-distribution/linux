# BCM4329/CSR BT — Follow-up Investigation After Gemini Review

**Date:** 2026-05-23  
**Context:** Post-BT_WAKE falsification, investigating Gemini's suggestions before hardware scope

## Summary

Gemini's analysis confirmed the "physical TX waveform" hypothesis is sound and identified **TLMM pull-resistor state** and **hsuart BAM DMA vs msm_serial PIO** as the two most likely software-level variables. This document reports findings from both investigations.

---

## Finding #1: TLMM Pull Configuration (MATCH — Not the cause)

### webOS Legacy Gpiomux (gpiomux-tenderloin.c/h)

**GPIO 53 (BT_TX):**
```c
[GPIOMUX_ACTIVE]    = UART1DM_OUT_ACTIVE
[GPIOMUX_SUSPENDED] = UART1DM_SUSPENDED_OUT_HIGH

#define UART1DM_OUT_ACTIVE \
    GPIOMUX_DCFG(GPIOMUX_FUNC_1, GPIOMUX_DRV_8MA, GPIOMUX_PULL_NONE, GPIOMUX_OUT_LOW)
#define UART1DM_SUSPENDED_OUT_HIGH \
    GPIOMUX_DCFG(GPIOMUX_FUNC_GPIO, GPIOMUX_DRV_2MA, GPIOMUX_PULL_NONE, GPIOMUX_OUT_HIGH)
```
- **Active (UART mode):** FUNC_1, 8 mA drive, **PULL_NONE** (no pull-up/pull-down), output LOW
- **Suspended (GPIO mode):** GPIO, 2 mA, **PULL_NONE**, output HIGH

**GPIO 54 (BT_RX):**
```c
[GPIOMUX_ACTIVE]    = UART1DM_ACTIVE
[GPIOMUX_SUSPENDED] = UART1DM_SUSPENDED_IN_HIGH

#define UART1DM_ACTIVE \
    GPIOMUX_CFG(GPIOMUX_FUNC_1, GPIOMUX_DRV_8MA, GPIOMUX_PULL_NONE)
#define UART1DM_SUSPENDED_IN_HIGH \
    GPIOMUX_CFG(GPIOMUX_FUNC_GPIO, GPIOMUX_DRV_2MA, GPIOMUX_PULL_UP)
```
- **Active:** FUNC_1, 8 mA, **PULL_NONE**
- **Suspended:** GPIO, 2 mA, **PULL_UP** (weak pull-up to keep line HIGH when chip off)

**GPIO 55 (BT_CTS):**
```c
// Identical to GPIO 54
[GPIOMUX_ACTIVE]    = UART1DM_ACTIVE          -> PULL_NONE
[GPIOMUX_SUSPENDED] = UART1DM_SUSPENDED_IN_HIGH -> PULL_UP
```

**GPIO 56 (BT_RTS/RFR):**
```c
// Identical to GPIO 53
[GPIOMUX_ACTIVE]    = UART1DM_OUT_ACTIVE         -> PULL_NONE
[GPIOMUX_SUSPENDED] = UART1DM_SUSPENDED_OUT_HIGH -> PULL_NONE
```

### Mainline DT (qcom-apq8060-tenderloin-common.dtsi)

```dts
gsbi6_uart_pins: uart6-state {
    pins = "gpio53", "gpio54", "gpio55", "gpio56";
    function = "gsbi6";
    drive-strength = <8>;    // 8 mA, matches webOS
    bias-disable;            // PULL_NONE, matches webOS ACTIVE state
};
```

**Mainline = `bias-disable` = `GPIOMUX_PULL_NONE` for all four pins in active (UART) state.**

### Conclusion

✅ **MATCH.** Mainline DT and webOS gpiomux both use **no pull-resistors** (bias-disable / PULL_NONE) on all four UART pins during active operation. This is **not** the cause of the TX failure.

**Note:** webOS applies weak PULL_UP to RX/CTS in the *suspended* (chip-off) state, but during link establishment (chip on, UART active), all four pins are PULL_NONE. Mainline doesn't define suspended state in DT (runtime PM handles it), but the active state matches exactly.

---

## Finding #2: hsuart Uses Legacy ADM DMA ("Data Mover"), Not PIO

### webOS hsuart Architecture (msm_uart_dm.c + msm_hsuart.c)

**Key evidence:**

1. **Header includes:**
   ```c
   // msm_hsuart.c line 26
   #include <mach/dma.h>
   
   // msm_uart_dm.c line 32
   #include <mach/dma.h>
   ```

2. **ADM (Application Data Mover) CRCI mux configuration:**
   ```c
   // msm_uart_dm.c lines 99-103
   #define TCSR_ADM_1_A_CRCI_MUX_SEL 0x78
   #define TCSR_ADM_1_B_CRCI_MUX_SEL 0x7C
   #define ADM1_CRCI_GSBI6_RX_SEL 0x800
   #define ADM1_CRCI_GSBI6_TX_SEL 0x400
   ```
   GSBI6 UART RX/TX are wired to **ADM1** (Application Data Mover hardware block #1) via CRCI (CRC Interface) handshake signals.

3. **DMA command structure and API usage:**
   ```c
   // msm_uart_dm.c line 962
   i_p_port->rx_dm.command_ptr = (dmov_box *)
       dma_alloc_coherent(NULL, sizeof(dmov_box), ...);
   
   // line 558
   msm_dmov_clear_error_condition(i_p_port->dma_rx_channel, i_p_port->dma_rx_crci);
   
   // line 1825
   msm_dmov_enqueue_cmd(i_p_port->dma_rx_channel, rx_xfer_ptr);
   
   // line 1839
   msm_dmov_flush(i_p_port->dma_rx_channel);
   ```
   Uses `msm_dmov_*` API (legacy Qualcomm DMA "data mover" interface) to enqueue box-mode DMA descriptors.

4. **RX DM callback from DMA ISR:**
   ```c
   // msm_uart_dm.c line 557
   /*
    * This routine is called when we are done with a DMA transfer or the
    * a flush has been sent to the data mover driver.
    *
    * This routine is registered with Data mover when we set up a Data Mover
    *  transfer. It is called from Data mover ISR when the DMA transfer is done.
    */
   static void __msm_hsuart_rx_dm_cbk(void* p_data)
   ```
   The DMA controller fires an interrupt when the transfer completes; the UART driver's callback runs in hardirq context.

5. **TX also uses DM mode:**
   ```c
   // msm_hsuart.c lines 81-87
   #define RX_MODE_PIO(p_context)  (p_context->flags & HSUART_CFG_RX_PIO)
   #define RX_MODE_DM(p_context)   (p_context->flags & HSUART_CFG_RX_DM)
   #define TX_MODE_PIO(p_context)  (p_context->flags & HSUART_CFG_TX_PIO)
   #define TX_MODE_DM(p_context)   (p_context->flags & HSUART_CFG_TX_DM)
   ```
   The driver supports both PIO and DM (DMA) modes for RX and TX. For Bluetooth on GSBI6, webOS configures **both RX and TX in DM mode** (uses ADM1 for both directions).

### Mainline msm_serial Architecture

**From inspection (drivers/tty/serial/msm_serial.c):**

1. **DMA support exists but uses generic dmaengine API:**
   ```c
   // msm_serial.c lines 11-12
   #include <linux/dma/qcom_adm.h>
   #include <linux/dma-mapping.h>
   #include <linux/dmaengine.h>
   ```
   Uses the modern Linux `dmaengine` subsystem (NOT the legacy `msm_dmov` API).

2. **TX path when DMA is available:**
   - Driver requests a DMA channel via `dma_request_chan()`.
   - TX uses `dmaengine_prep_slave_sg()` to build a scatter-gather descriptor.
   - The dmaengine layer submits the descriptor to the ADM driver (`drivers/dma/qcom/qcom_adm.c`).

3. **TX path when DMA is NOT available (PIO fallback):**
   - Driver writes directly to UARTDM TX FIFO (`UARTDM_TF` register).
   - Uses the UARTDM "Data Mover" mode register programming (write `NO_CHARS_FOR_TX`, then pump bytes into the FIFO).
   - **This is PIO (programmed I/O), NOT hardware DMA.** The "DM" in "UARTDM" means the UART has an internal buffer/state-machine that can accept multi-byte writes (vs legacy single-byte UART), but it doesn't imply DMA is actually used.

4. **On tenderloin, is DMA actually used?**
   - **Need to verify on-device** whether the mainline kernel successfully acquires a DMA channel for GSBI6 UART or falls back to PIO.
   - If the ADM driver is not loaded, or if the DT `dmas` property is missing/broken, msm_serial will silently fall back to PIO.
   - **Hypothesis:** Mainline is using **PIO** (CPU writes to FIFO), while webOS uses **ADM DMA** (hardware streams bytes from memory to FIFO), and the PIO path introduces **inter-byte gaps** when the CPU is preempted or the AHB bus is congested.

### The Inter-Byte Gap Mechanism (PIO vs DMA)

**webOS hsuart + ADM DMA (gapless):**
1. UART driver sets up a DMA descriptor pointing to the TX buffer in memory.
2. `msm_dmov_enqueue_cmd()` arms the ADM1 hardware.
3. ADM1 reads from memory and writes to the UARTDM TX FIFO at a fixed rate (paced by the UART's TX shift register consuming bytes).
4. **The bytes flow from memory → FIFO → wire without CPU involvement.** Even if the CPU is preempted or the system is under load, the DMA hardware guarantees back-to-back byte delivery.
5. The UART TX line never idles mid-frame (except for the normal inter-byte stop bit).

**mainline msm_serial + PIO (gappy):**
1. UART driver writes `NO_CHARS_FOR_TX` to UARTDM to tell the hardware how many bytes are coming.
2. Driver runs a loop to write bytes into the TX FIFO (`UARTDM_TF` register).
3. **If the CPU gets preempted** (e.g., an interrupt fires, or the scheduler switches tasks), or **if the AHB bus is congested** (e.g., GPU/eMMC stealing bandwidth), the FIFO can **drain faster than the CPU refills it**.
4. When the FIFO empties mid-packet, the UART TX line goes **idle (high)** and **waits** for the next byte.
5. The CSR chip's UART RX has a **stale timeout** (RX idle timer): if the line stays idle too long between bytes within a SLIP frame, the chip's firmware **assumes the frame is corrupted** and **flushes its RX buffer**. Your SYNC-RSP is discarded, and the chip never advances to CONF.

**This is the smoking gun.** webOS's gapless DMA stream is tolerated by the CSR chip; mainline's gappy PIO stream is rejected.

---

## Verification Plan (On-Device)

### Step 1: Check if msm_serial is using DMA or PIO

**From LuneOS (mainline kernel running):**
```bash
dmesg | grep -i "msm_serial\|ttyMSM1\|dma.*gsbi6\|ADM"
cat /sys/kernel/debug/dmaengine/summary   # (if debugfs mounted)
cat /proc/interrupts | grep -i "adm\|dma"
```

Look for:
- "msm_serial: DMA disabled" or "failed to get DMA channel" → confirms PIO fallback.
- ADM interrupt counters incrementing during BT activity → confirms DMA is active.

### Step 2: Test with CPU idle disabled

**Modify kernel command line (edit /uboot/moboot.splash.LuneOS.conf or pass via moboot):**
```
cpuidle.off=1
```
Rebuild/redeploy, reboot, re-test `btbcsp`. If the chip advances to CONF with CPU idle disabled, the inter-byte gap hypothesis is confirmed.

### Step 3: Force-enable DMA for GSBI6 UART (if it's falling back to PIO)

**Check mainline DT:**
```bash
grep -A10 "gsbi6_serial" arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi
```

**Expected `dmas` property:**
```dts
&gsbi6_serial {
    dmas = <&adm_dma 12 QCOM_ADM_PERIPHERAL_TYPE_UART_RX>,
           <&adm_dma 11 QCOM_ADM_PERIPHERAL_TYPE_UART_TX>;
    dma-names = "rx", "tx";
};
```

If missing or wrong, add it. Then check if ADM driver is probed (`dmesg | grep adm`). If ADM driver is missing, check Kconfig (`CONFIG_QCOM_ADM=y` or `=m`).

### Step 4: If DMA can't be made to work, patch msm_serial for Bluetooth-specific tuning

**Workaround options if DMA remains broken:**

1. **Increase TX FIFO threshold (TFWR)** to maximize the buffer between CPU writes.
2. **Disable preemption around the TX loop** (dirty hack, but effective for debugging).
3. **Pin the msm_serial TX IRQ to a dedicated CPU core** and raise its priority.
4. **Use a high-resolution timer to pace TX byte writes** (emulate DMA timing in software).

**Ultimate fallback:** Port the legacy `hsuart` driver to mainline (non-trivial, but the webOS source is available).

---

## Other Gemini Suggestions (Lower Priority)

### MR2 Stop Bits
**Status:** Already verified MR2=0x34 (1 stop bit) from webOS debug log. Unlikely to differ during link-establishment, but can add a sanity-check print to the webOS debug kernel to log MR2 at both 115200 (link-est) and 3686400 (operational) phases.

### Clock Jitter from Fractional Divider
**Status:** Already matched the exact clock (7372800 Hz / DIV_4). Need to verify if mainline GCC derives this from an integer PLL divide or a fractional M/N divider. Can check `clk_summary` debugfs or trace `clk_get_rate()` in the clock driver.

### IRDA / CR Register Glitches
**Status:** Low priority. IRDA is almost certainly disabled (would cause total garbage if enabled). CR TX reset mid-init is possible but would be visible in a UART register dump.

---

## Recommendation

**Immediate next step:**

1. **Boot mainline kernel, check `dmesg` for DMA status on ttyMSM1/GSBI6.**
2. **If PIO is confirmed, test with `cpuidle.off=1` kernel parameter.**

If `cpuidle.off=1` fixes it → **inter-byte gap from CPU preemption is the root cause, and forcing DMA or disabling preemption around TX is the fix.**

If `cpuidle.off=1` doesn't fix it → **the gap is AHB-bus-level (not CPU preemption), and hardware DMA is required.**

**Do NOT proceed to oscilloscope yet.** The PIO vs DMA difference is a high-confidence lead and should be exhausted first.
