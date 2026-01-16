# ADM DMA for eMMC/SDCC Analysis Report

## Executive Summary

This report documents the investigation into enabling ADM (Application Data Mover) DMA
for the eMMC controller on the HP TouchPad (APQ8060). The eMMC was running in PIO mode
at ~73 MB/s, while DMA should provide better throughput. Analysis of the legacy webOS
kernel revealed a critical bug in the mainline ADM driver's burst size handling.

## Background

### Hardware Configuration
- **SoC**: Qualcomm APQ8060 (MSM8660 variant)
- **eMMC Controller**: SDCC1 at 0x12400000
- **DMA Engine**: ADM1 (Application Data Mover)
- **DMA Channel**: Channel 2 (global channel 18)
- **CRCI**: 1 (Client Request Control Interface for flow control)

### webOS Kernel Confirmation
The webOS 3.0.5 kernel successfully uses ADM DMA for eMMC:
```
mmc0: Qualcomm MSM SDCC at 0x0000000012400000 irq 136,0 dma 18
mmc0: DM non-cached buffer at ff01f000, dma_addr 0x7eabe000
mmc0: DM cmd busaddr 0x7eabe000, cmdptr busaddr 0x7eabe300
```

## Problem Analysis

### Issue 1: DML Block Not Present on APQ8060

The mainline MMCI driver's Qualcomm variant (`mmci_qcom_dml.c`) was designed for
newer SoCs (MSM8960+) that have a DML (Data Mover Layer) block between the SDCC
and BAM DMA engine. The APQ8060 uses the older ADM DMA engine which doesn't have
DML.

**Solution**: Added `qcom_dma_is_adm()` function to detect ADM DMA controller and
skip DML configuration when ADM is used.

### Issue 2: Burst Size Unit Mismatch (CRITICAL BUG)

The mainline ADM driver (`drivers/dma/qcom/qcom_adm.c`) has a bug in how it
interprets the burst size from the DMA slave configuration.

#### How MMCI Configures DMA
```c
struct dma_slave_config conf = {
    .src_addr = host->phybase + MMCIFIFO,
    .dst_addr = host->phybase + MMCIFIFO,
    .src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
    .dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
    .src_maxburst = variant->fifohalfsize >> 2,  /* 8 words */
    .dst_maxburst = variant->fifohalfsize >> 2,  /* 8 words */
};
```

For `variant_qcom`:
- `fifohalfsize` = 32 bytes
- `maxburst` = 32 >> 2 = **8 words** (meaning 8 × 4 = 32 bytes)

#### The Bug in ADM Driver
The ADM driver retrieved the burst value but used it directly as bytes:
```c
burst = achan->slave.dst_maxburst;  /* Gets 8 */
box_desc->row_len = burst << 16 | burst;  /* Sets 8 bytes, should be 32! */
```

This resulted in:
- `row_len` = 0x00080008 (8 bytes src, 8 bytes dst)
- Should be 0x00200020 (32 bytes src, 32 bytes dst)

#### webOS Kernel Comparison
The webOS kernel uses the full FIFO size (64 bytes) directly:
```c
#define MCI_FIFOSIZE (16*4)  /* 64 bytes */

box->src_dst_len = (MCI_FIFOSIZE << 16) | MCI_FIFOSIZE;  /* 0x00400040 */
```

**Solution**: Multiply burst by addr_width to convert from words to bytes:
```c
if (direction == DMA_MEM_TO_DEV) {
    burst = achan->slave.dst_maxburst * achan->slave.dst_addr_width;
} else {
    burst = achan->slave.src_maxburst * achan->slave.src_addr_width;
}
```

### Issue 3: adm_get_blksize() Special Case

The `adm_get_blksize()` function had a special case for burst=8 with a comment
explaining it was for "8 words = 32 bytes". After the burst conversion fix,
this special case is no longer needed since burst is now always in bytes.

## Code Changes

### 1. drivers/mmc/host/mmci_qcom_dml.c

Added ADM detection and DML bypass:

```c
static bool qcom_dma_is_adm(struct device_node *np)
{
    struct of_phandle_args dma_spec;
    struct device_node *dma_node;
    bool is_adm = false;

    if (of_parse_phandle_with_args(np, "dmas", "#dma-cells", 0, &dma_spec))
        return false;

    dma_node = dma_spec.np;
    if (dma_node) {
        is_adm = of_device_is_compatible(dma_node, "qcom,adm");
        of_node_put(dma_node);
    }

    return is_adm;
}
```

Modified `qcom_dma_setup()` to:
1. Propagate `-EPROBE_DEFER` for proper deferred probe handling
2. Skip DML configuration when ADM is detected

Modified `qcom_dma_start()` to skip DML trigger for ADM.

### 2. drivers/dma/qcom/qcom_adm.c

Fixed burst size conversion in `adm_prep_slave_sg()`:

```c
/* Before (BUG): */
burst = achan->slave.dst_maxburst;  /* 8 words, used as 8 bytes */

/* After (FIXED): */
burst = achan->slave.dst_maxburst * achan->slave.dst_addr_width;  /* 32 bytes */
```

Updated `adm_get_blksize()` to remove the special case for burst=8 since burst
is now always in bytes.

## ADM Box Mode DMA Descriptor Format

For reference, the ADM box descriptor format used for SDCC DMA:

| Field | Read (DEV_TO_MEM) | Write (MEM_TO_DEV) |
|-------|-------------------|---------------------|
| cmd | CMD_MODE_BOX \| CMD_SRC_CRCI(crci) | CMD_MODE_BOX \| CMD_DST_CRCI(crci) |
| src_addr | FIFO address (fixed) | Memory address (increments) |
| dst_addr | Memory address (increments) | FIFO address (fixed) |
| row_len | burst \| (burst << 16) | burst \| (burst << 16) |
| row_offset | burst (dst increments) | burst << 16 (src increments) |
| num_rows | rows \| (rows << 16) | rows \| (rows << 16) |

## Testing Status

- **PIO Mode**: Working, ~73 MB/s throughput (current fallback)
- **DMA Mode**: Under investigation - CH_CONF registers read back as 0x00000000

### Current Issue (January 2026)

After enabling ADM DMA, the CH_CONF and CH_RSLT_CONF registers read back as 0x00000000 after writes. This prevents DMA completion interrupts from firing.

**Debug output:**
```
ADM probe: base=0xXXXXXXXX EE=1, initial CH_CONF[0]=0x00000000
ADM probe: after init CH_CONF[0]=0x00000000 RSLT_CONF[0]=0x00000000
ADM start_dma: CH_CONF[2]=0x00000000 CH_RSLT_CONF=0x00000000
```

**Investigation findings:**
1. webOS kernel uses SD 1 (= mainline EE 1, offset 0x800) for SDCC DMA
2. webOS does NOT reset ADM in probe - mainline reset may be clearing config
3. webOS only writes SHADOW_EN to CH_CONF, not IRQ_EN or FORCE_RSLT_EN
4. DMOV_CONF_IRQ_EN is defined in webOS but never actually used

**Current changes being tested:**
1. Skip ADM reset in probe (match webOS behavior)
2. Only write SHADOW_EN + SEC_DOMAIN to CH_CONF
3. Read-modify-write pattern for CH_CONF (not overwrite)
4. Write IRQ_EN only to RSLT_CONF

## Recommendations

1. **Test the burst fix**: Deploy the kernel with the ADM burst conversion fix
2. **Consider full FIFO burst**: webOS uses 64-byte burst (full FIFO), mainline
   uses 32-byte (half FIFO). If performance is suboptimal, consider increasing
   to full FIFO size.
3. **Upstream the fix**: The ADM burst size bug affects any driver using ADM DMA
   with flow control - this should be submitted upstream.

## Files Modified

1. `drivers/mmc/host/mmci_qcom_dml.c` - ADM detection and DML bypass
2. `drivers/dma/qcom/qcom_adm.c` - Burst size conversion fix
3. `arch/arm/configs/tenderloin_debug_defconfig` - Increased deferred_probe_timeout

## References

- webOS kernel source: `webos-linux-kernel-touchpad/drivers/mmc/host/msm_sdcc.c`
- ADM hardware documentation: Qualcomm MSM8660 TRM
- DMA slave config API: `include/linux/dmaengine.h`
