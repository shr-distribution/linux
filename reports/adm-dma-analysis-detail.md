# ADM DMA Analysis: webOS vs Mainline Kernel

## Summary

The mainline qcom_adm.c driver has several differences from the webOS dma.c implementation that prevent DMA completion interrupts from firing.

**Status (January 2026)**: CH_CONF registers read back as 0x00000000 after writes, indicating the writes are not taking effect. Investigation ongoing to determine if this is a register offset issue, clock/reset issue, or TrustZone security restriction.

## EE/SD Mapping (Execution Environment / Security Domain)

The webOS kernel uses "SD" (Security Domain) terminology, while mainline uses "EE" (Execution Environment). The mapping is:

| Mainline EE | Offset | webOS SD |
|-------------|--------|----------|
| EE 0 | 0x000 | SD 0 |
| EE 1 | 0x800 | SD 1 |

The webOS kernel uses **SD 1** for Scorpion/Apps processor DMA channels (with `DMOV_SD_AARM = 1` and `DMOV_SD_MASTER = 1` for MSM_ADM3).

Register offset calculation: `base + (0x800 * EE) + register_offset + (channel * 4)`

## Critical Differences Found

### 1. CRCI MUX Bit Position (CRITICAL BUG)

**webOS (arch/arm/mach-msm/include/mach/dma.h:128):**
```c
#define DMOV_CRCI_MUX  (1 << 18)
```

**Mainline (drivers/dma/qcom/qcom_adm.c:97):**
```c
#define ADM_CRCI_CTL_MUX_SEL  BIT(4)
```

The MUX select bit is at position 18, not position 4! This is likely causing CRCI misconfiguration.

### 2. Channel Configuration at Probe Time (KEY DIFFERENCE)

**webOS initializes channels at probe (dma.c:733-758):**
```c
static void config_datamover(int adm)
{
    for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
        struct msm_dmov_chan_conf *chan_conf = dmov_conf[adm].chan_conf;
        unsigned conf;
        /* Only configure scorpion channels (SD <= 1) */
        if (chan_conf[i].sd <= 1) {
            conf = readl(DMOV_REG(DMOV_CONF(i), adm));
            conf &= ~DMOV_CONF_SD(7);
            conf |= DMOV_CONF_SD(chan_conf[i].sd);
            writel(conf | DMOV_CONF_SHADOW_EN,
                   DMOV_REG(DMOV_CONF(i), adm));
        }
    }
    for (i = 0; i < MSM_DMOV_CRCI_COUNT; i++) {
        writel(DMOV_CRCI_CTL_BLK_SZ(crci_conf[i].blk_size),
               DMOV_REG(DMOV_CRCI_CTL(i), adm));
    }
}
```

**Key observations:**
1. webOS reads CH_CONF first, then modifies specific bits, then writes back
2. webOS ONLY writes **SHADOW_EN** to CH_CONF - NOT IRQ_EN or FORCE_RSLT_EN
3. webOS only configures channels where SD <= 1 (channels 0-9), not all 16
4. `DMOV_CONF_IRQ_EN` and `DMOV_CONF_FORCE_RSLT_EN` are defined but **never used**

**Mainline previously wrote too many flags:**
```c
/* WRONG - webOS doesn't write IRQ_EN or FORCE_RSLT_EN to CH_CONF */
writel(ADM_CH_CONF_SHADOW_EN | ADM_CH_CONF_IRQ_EN |
       ADM_CH_CONF_FORCE_RSLT_EN | ADM_CH_CONF_SEC_DOMAIN(adev->ee),
       adev->regs + ADM_CH_CONF(achan->id, adev->ee));
```

**Correct approach (matching webOS):**
```c
/* Read-modify-write, only set SHADOW_EN and SD */
conf = readl_relaxed(adev->regs + ADM_CH_CONF(i, adev->ee));
conf &= ~ADM_CH_CONF_SEC_DOMAIN(7);
conf |= ADM_CH_CONF_SEC_DOMAIN(adev->ee) | ADM_CH_CONF_SHADOW_EN;
writel(conf, adev->regs + ADM_CH_CONF(i, adev->ee));
```

### 3. RSLT_CONF IRQ Enable for All Channels

**webOS enables IRQ for ALL channels at probe (dma.c:790-797):**
```c
for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
    writel(DMOV_RSLT_CONF_IRQ_EN | DMOV_RSLT_CONF_FORCE_FLUSH_RSLT,
           DMOV_REG(DMOV_RSLT_CONF(i), adm));
}
```

**Mainline only enables for the specific channel being used.**

### 4. IRQ Enable/Disable Pattern

**webOS:**
- IRQ disabled at probe: `disable_irq(dmov_conf[adm].irq);`
- IRQ enabled when first command enqueued: `enable_irq(dmov_conf[adm].irq);`
- IRQ disabled when no channels active: `disable_irq_nosync(dmov_conf[adm].irq);`

**Mainline:**
- IRQ always enabled via `devm_request_irq()`
- No dynamic enable/disable

### 5. Pre-configured CRCI Block Sizes

**webOS has static CRCI configuration (dma.c:142-159):**
```c
static struct msm_dmov_crci_conf adm1_crci_conf[] = {
    DMOV_CRCI_DEFAULT_CONF,
    DMOV_CRCI_CONF(1, 1),  // CRCI 1: SD=1, blk_size=1 (32 bytes)
    DMOV_CRCI_CONF(1, 1),  // CRCI 2: SD=1, blk_size=1 (32 bytes)
    // ...
};
```

The block sizes are pre-configured at probe time, not per-transaction.

### 6. Two-Level Command Pointer Indirection

**webOS msm_sdcc.c uses:**
```c
nc->cmdptr = (host->dma.cmd_busaddr >> 3) | CMD_PTR_LP;
host->dma.hdr.cmdptr = DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(host->dma.cmdptr_busaddr);
```

Structure:
1. `hdr.cmdptr` → points to `nc->cmdptr` (with DMOV_CMD_PTR_LIST)
2. `nc->cmdptr` → points to command array (with CMD_PTR_LP)
3. Commands have CMD_LC on last one

**Mainline uses similar structure but in CPLE format.**

### 7. Box Descriptor Row Length

**webOS uses MCI_FIFOSIZE (64 bytes):**
```c
box->src_dst_len = (MCI_FIFOSIZE << 16) | (MCI_FIFOSIZE);
box->row_offset = MCI_FIFOSIZE;  // for reads
```

**Mainline uses burst size (32 bytes):**
```c
box_desc->row_len = burst << 16 | burst;  // burst = 32
```

### 8. Channel Number Encoding

For MSM8X60:
- `DMOV_SDC1_CHAN = 18`
- This means: ADM = 18/16 = 1, Channel = 18%16 = 2

So SDC1 uses ADM1 channel 2, which matches our DT configuration.

## Recommended Fixes

1. **Fix CRCI MUX bit position:** Change from BIT(4) to BIT(18)

2. **Initialize all channels at probe:** Configure DMOV_CONF and DMOV_RSLT_CONF for all 16 channels

3. **Pre-configure CRCI block sizes:** Set up CRCI_CTL registers at probe based on known configurations

4. **Consider IRQ enable/disable pattern:** May need to match webOS behavior

5. **Verify row length:** Check if MMCI expects 64-byte (FIFO size) or 32-byte (burst) transfers

### 9. Reset Sequence

**webOS does NOT reset ADM in probe**. The webOS `msm_dmov_probe()` only:
1. Gets IRQ and base address
2. Requests IRQ (then disables it)
3. Initializes clocks (but doesn't enable them yet)
4. Calls `config_datamover()` to set up channels
5. Sets up RSLT_CONF for all channels

**Mainline performs full reset:**
```c
reset_control_assert(adev->clk_reset);
reset_control_assert(adev->c0_reset);
reset_control_assert(adev->c1_reset);
reset_control_assert(adev->c2_reset);
udelay(2);
reset_control_deassert(adev->clk_reset);
reset_control_deassert(adev->c0_reset);
reset_control_deassert(adev->c1_reset);
reset_control_deassert(adev->c2_reset);
```

This reset may be clearing bootloader/TrustZone configuration that the ADM needs.

## Test Results

### Current Observations (January 2026)

1. **CH_CONF reads as 0x00000000**: After writing to CH_CONF registers, reading them back returns 0x00000000. This indicates:
   - Either the write is not taking effect (wrong address or security restriction)
   - Or the register doesn't exist at the calculated offset

2. **DMA starts but never completes**:
   - DMA transfer initiates (cmd_ptr written)
   - CRCI configured
   - But NO completion IRQ fires (0 interrupts in /proc/interrupts)
   - eMMC partitions fail to be detected due to DMA timeout

3. **Debug output example**:
   ```
   ADM probe: base=<addr> EE=1, initial CH_CONF[0]=0x00000000
   ADM probe: after init CH_CONF[0]=0x00000000 RSLT_CONF[0]=0x00000000
   ```

### Potential Causes

1. **Register offset calculation**: May be incorrect despite matching webOS macros
2. **Reset clearing configuration**: The reset sequence may clear pre-configured state
3. **TrustZone restriction**: ARM non-secure world may not have write access to these registers
4. **Clock gating**: Registers may not be accessible without proper clock sequencing

## Files Referenced

- webOS: `arch/arm/mach-msm/dma.c`
- webOS: `arch/arm/mach-msm/include/mach/dma.h`
- webOS: `drivers/mmc/host/msm_sdcc.c`
- webOS: `drivers/mmc/host/msm_sdcc.h`
- Mainline: `drivers/dma/qcom/qcom_adm.c`
- Mainline: `drivers/mmc/host/mmci_qcom_dml.c`
