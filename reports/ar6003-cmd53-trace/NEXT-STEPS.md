# WiFi AR6003 CMD53 FIXED-address read — Next Steps

**Date:** 2026-05-21  
**Status:** DMA transfer completes, but data corruption suspected

---

## Summary of Investigation

### What We Fixed
1. ✅ **IRQ handler works correctly** — confirmed ch2 (eMMC) and ch5 (WiFi) IRQs fire and are processed at EE=1
2. ✅ **curr_txd clears properly** — no stuck "skipped start_dma" state
3. ✅ **Transfers complete** — 128B FIXED CMD53 issues at T+0ms, IRQ fires at T+10ms
4. ✅ **EE=1 register reads work** — the documented EE=0 quirk does NOT apply to IRQ_STATUS (EE=1=live, EE=0=0)

### What We Ruled Out
- ❌ CMD53 byte vs block mode (both legacy and mainline use byte mode)
- ❌ PIO vs DMA (both use DMA with DMAENABLE=1)
- ❌ Address calculation bugs (both use addr=0x800 for FIXED reads)
- ❌ IRQ delivery (IRQs fire reliably for both eMMC and WiFi)
- ❌ ADM channel never starting (devmem showed CMD_PTR=0 was a symptom, not root cause)
- ❌ EE=0 vs EE=1 register access (IRQ_STATUS is readable at EE=1, forcing EE=0 broke boot)

### Current Problem

**Symptom:** The 128B FIXED CMD53 read from AR6003 mailbox (addr=0x800) completes from the DMA perspective (IRQ fires), but ath6kl driver stalls afterward and never continues probe.

**Evidence:**
```
[  372.397390] CMD53 arg=0x10100080 RD addr=0x800 blksz=128 BYTE FIXED
[  372.397706] mmci: issue_pending called
[  372.407395] ADM IRQ: srcs@EE1=0x00000020 (ch5 completed)
[  ...silence...]
[  303.660821] ath6kl: Failed request firmware (trying fw-5/fw-4)
```

The transfer hardware-completes in 10ms, but ath6kl never processes the HTC header data and hangs waiting for firmware load to proceed.

**Hypothesis:** DMA wrote data to the wrong address, or AR6003 didn't send valid data on the FIXED-address read.

---

## Remaining Issues to Debug

### Issue 1: Data Integrity After 128B FIXED Read

**Priority:** HIGH — this is the blocker for WiFi probe  
**Next Steps:**
1. Add a memory dump right after the 128B read completion (in mmci IRQ handler or ath6kl completion) to see if the buffer contains valid HTC header data
2. Compare the dumped bytes against the expected HTC body format from legacy captures
3. If data is all-zeros or garbage:
   - Check ADM descriptor src/dst address setup for FIXED-address reads
   - Check CRCI block size matches transfer size (128B)
   - Check if AR6003 requires a different SDIO timing for FIXED vs INCR

**Expected HTC header format (from legacy):**
- Bytes 0-3: endpoint/flags/length
- Should NOT be all zeros or 0xFF

### Issue 2: CRCI Block Size vs Transfer Size

**Priority:** MEDIUM — potential root cause for issue 1  
**Background:** CRCI (Client Rate Control Interface) flow-controls the DMA based on peripheral readiness. The block size programmed in CRCI_CTL must match the peripheral's transfer unit.

**Current state:**
- mmci programs `async_desc->blk_size` from `data->blksz` (128 for this transfer)
- adm_start_dma writes this to `ADM_CRCI_CTL(crci, ee)` at line 878
- AR6003 mailbox block size is 128B (HIF_MBOX_BLOCK_SIZE)

**Verification needed:**
- Dump CRCI_CTL register value during the 128B read to confirm it's 0x80 (128)
- Check if CRCI mux is set correctly (bit 18 = MUX_SEL)

### Issue 3: FIXED-Address DMA Descriptor Setup

**Priority:** MEDIUM  
**Question:** Does the ADM descriptor properly handle FIXED-address reads where the SDIO address doesn't increment?

**Current ADM descriptor (box mode):**
- `src_addr` = SDIO FIFO address (presumably 0x121c0080 + some offset for AR6003 mailbox)
- `dst_addr` = host memory buffer
- `row_len` = 128
- `num_rows` = 1
- `row_offset` = 0 (for FIXED, should this be 0 to re-read same address?)

**Legacy msm_dmov comparison needed:**
- Check webOS kernel's ADM descriptor setup for SDIO FIXED reads
- Verify row_offset and src_addr handling

---

## Debug Plan (Priority Order)

### Step 1: Add Memory Dump After 128B Read
**Goal:** Confirm whether data reached host memory  
**Where:** mmci.c IRQ handler, right after DATAEND for WiFi transfers  
**What to log:** First 32 bytes of the DMA buffer, formatted as hex

**Expected outcome:**
- If valid HTC header → data arrived, issue is in ath6kl or timing
- If zeros/garbage → DMA descriptor or AR6003 issue

### Step 2: Compare Against Legacy Descriptor Setup
**Goal:** Verify ADM descriptor matches working legacy pattern  
**Where:** qcom_adm.c descriptor prep vs webOS msm_dmov.c  
**What to check:**
- row_offset for FIXED-address (0 vs something else?)
- src_addr calculation
- CRCI_CTL programming

### Step 3: Force PIO Test
**Goal:** Rule out ADM entirely as a variable  
**How:** Remove `dmas` property from sdcc4 DT node, rebuild  
**Expected outcome:**
- If PIO works → confirms ADM descriptor/CRCI issue
- If PIO fails → AR6003 chip or mmci timing issue

### Step 4: CRCI Block Size Experiment
**Goal:** Test if CRCI needs different block size for FIXED reads  
**How:** Hardcode CRCI block size to 1, 64, or 512 for WiFi ch5  
**Rationale:** Maybe AR6003 expects byte-by-byte CRCI strobing on FIXED reads?

---

## Key Files for Next Round

**Mainline (linux-6.18-tenderloin):**
- `drivers/dma/qcom/qcom_adm.c` — ADM descriptor setup, CRCI programming
- `drivers/mmc/host/mmci.c` — IRQ handler, data completion
- `drivers/net/wireless/ath/ath6kl/sdio.c` — HTC body read completion
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` — sdcc4 DT node

**Legacy webOS (webos-linux-kernel-touchpad):**
- `arch/arm/mach-msm/dma.c` — msm_dmov descriptor setup for comparison
- `drivers/mmc/host/msm_sdcc.c` — legacy SDIO controller for timing comparison

**Artifacts:**
- `reports/ar6003-cmd53-trace/ANALYSIS.md` — full investigation log
- `reports/ar6003-cmd53-trace/adm1-ch5-hang-state.txt` — devmem snapshot (now outdated, transfers DO complete)
- `reports/ar6003-cmd53-trace/webos-debug-ar6k-only.log` — legacy working trace

---

## Success Criteria

1. ath6kl driver completes BMI phase and loads firmware
2. wlan0 interface appears
3. WiFi scan works
4. Sustained traffic (ping, iperf) stable under concurrent eMMC load

The hang is now solved (transfers complete). Focus shifts to **data integrity** of the FIXED-address read.
