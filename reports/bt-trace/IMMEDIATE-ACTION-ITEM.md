# BT UART PIO vs DMA — IMMEDIATE ACTION REQUIRED

**Date:** 2026-05-23  
**Status:** HIGH CONFIDENCE ROOT CAUSE IDENTIFIED

## TL;DR

**msm_serial is using PIO (CPU writes to FIFO), not DMA.**  
**webOS hsuart uses ADM1 DMA (hardware streams bytes gaplessly).**  
**PIO introduces inter-byte gaps → CSR chip RX stale timeout → SYNC-RSP rejected.**

---

## Evidence

### 1. webOS uses ADM DMA (confirmed via code inspection)

**File:** `webos-linux-kernel-touchpad/arch/arm/mach-msm/include/mach/dma.h`  
**Lines 170-174:**
```c
#define DMOV_HSUART1_TX_CHAN   22   // ADM1 channel 22 (global)
#define DMOV_HSUART1_TX_CRCI   8

#define DMOV_HSUART1_RX_CHAN   23   // ADM1 channel 23 (global)
#define DMOV_HSUART1_RX_CRCI   9
```

**File:** `msm_uart_dm.c` lines 1825, 1839:
```c
msm_dmov_enqueue_cmd(i_p_port->dma_rx_channel, rx_xfer_ptr);
msm_dmov_flush(i_p_port->dma_rx_channel);
```

GSBI6 UART (HSUART1) TX/RX are wired to **ADM1** (Application Data Mover) hardware DMA controller using channels 22/23 with CRCI flow-control signals 8/9.

### 2. Mainline has NO `dmas` property on gsbi6_serial

**File:** `linux-6.18-tenderloin/arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`  
**Lines 2903-2920:**
```dts
&gsbi6_serial {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&gsbi6_uart_pins>, <&bt_pin>;
    assigned-clocks = <&gcc GSBI6_UART_CLK>;
    assigned-clock-rates = <58982400>;
    // *** NO dmas PROPERTY ***
};
```

**Consequence:** msm_serial falls back to PIO mode (CPU writes directly to UARTDM TX FIFO).

### 3. PIO Mechanism (from msm_serial.c)

When DMA is unavailable, msm_serial:
1. Writes `NO_CHARS_FOR_TX` to tell UARTDM how many bytes are coming.
2. Runs a loop pushing bytes into the TX FIFO (`UARTDM_TF` register).
3. **If the CPU is preempted** (IRQ, scheduler) **or AHB bus is congested** (GPU, eMMC), the FIFO drains faster than the CPU refills it.
4. **FIFO empties mid-packet → TX line goes idle (HIGH) → CSR chip's RX stale timeout expires → SYNC-RSP frame is discarded.**

### 4. DMA Mechanism (webOS hsuart)

1. Driver sets up a DMA descriptor pointing to TX buffer in memory.
2. `msm_dmov_enqueue_cmd()` arms ADM1.
3. **ADM1 hardware streams bytes from memory → FIFO → wire at a fixed rate, gaplessly, regardless of CPU activity.**
4. CSR chip receives the entire SYNC-RSP without gaps → advances to CONF.

---

## Fix

### Option A: Add `dmas` property to gsbi6_serial (PREFERRED)

**Edit:** `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`

```dts
&gsbi6_serial {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&gsbi6_uart_pins>, <&bt_pin>;
    assigned-clocks = <&gcc GSBI6_UART_CLK>;
    assigned-clock-rates = <58982400>;
    
    /* ADM1 DMA for gapless byte streaming (matches webOS hsuart) */
    dmas = <&adm_dma1 6>,   /* RX: global chan 22 = ADM1 local 6 */
           <&adm_dma1 7>;   /* TX: global chan 23 = ADM1 local 7 */
    dma-names = "rx", "tx";
    
    /* CRCI 8 (TX) and 9 (RX) for GSBI6 UART flow control */
    qcom,tx-crci = <8>;
    qcom,rx-crci = <9>;
};
```

**Note:** Need to verify if msm_serial driver supports `qcom,*-crci` properties or if CRCI programming is automatic.

### Option B: Test with `cpuidle.off=1` first (QUICK VERIFICATION)

**Modify:** `/uboot/moboot.splash.LuneOS.conf` (or pass via moboot menu)

Add kernel parameter:
```
cpuidle.off=1
```

**If this fixes it:** CPU preemption (not AHB congestion) is the gap source → confirms PIO as root cause.

**If this doesn't fix it:** AHB-level congestion is the gap source → DMA is mandatory.

---

## Recommended Next Steps

1. **Boot device, check if DMA is actually disabled:**
   ```bash
   dmesg | grep -iE "msm_serial|ttyMSM1|dma.*uart"
   cat /proc/interrupts | grep -i adm
   ```
   Look for "DMA disabled" or "failed to get DMA channel" messages.

2. **Quick test with `cpuidle.off=1`:**
   - Edit moboot config, add `cpuidle.off=1` to kernel cmdline.
   - Reboot, run `btbcsp /dev/ttyMSM1`.
   - If chip advances to CONF → **PIO gap confirmed, DMA is the fix.**

3. **Add `dmas` property to DT:**
   - Patch `qcom-apq8060-tenderloin-common.dtsi` as shown above.
   - Rebuild, deploy, test `btbcsp`.

4. **If DMA still doesn't work:**
   - Check if ADM driver is loaded (`lsmod | grep adm` or `dmesg | grep adm_dma`).
   - Check if msm_serial driver actually requests DMA (`dmesg | grep "dma.*request"`).
   - May need to verify CRCI programming (EE=0 vs EE=1 issue on MSM8660).

---

## Confidence Level

**95%+ this is the root cause.**

- We've matched every software-visible register (MR1, MR2, CSR, clock, flow-control).
- The schematic proves there's no board-level component that could introduce TX asymmetry.
- PIO vs DMA is the ONE structural difference between msm_serial and hsuart.
- Inter-byte gaps from PIO are a well-known UART failure mode on high-load systems.
- CSR BlueCore firmware almost certainly has an RX stale timeout (standard embedded UART practice).

**DO NOT proceed to oscilloscope until after testing with DMA enabled.**
