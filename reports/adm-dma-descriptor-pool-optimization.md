# ADM DMA Descriptor Pool Optimization

## Overview

This document describes the descriptor pool optimization added to the QCOM ADM
DMA driver to reduce per-transfer allocation overhead and improve eMMC performance.

## Problem Statement

The original ADM DMA driver had significant per-transfer overhead:

```
Per-transfer allocations (original):
- kzalloc(async_desc)           ~120 bytes
- kzalloc(cpl buffer)           Variable, 1-12KB typical
- dma_map_single()              IOMMU/cache operations
- dma_sync_single_for_cpu()     Cache sync
- dma_sync_single_for_device()  Cache sync
- kfree(cpl)                    On completion
- kfree(async_desc)             On completion
- dma_unmap_single()            On completion
```

### Performance Impact

Testing on HP TouchPad (APQ8060) showed:

| Test | webOS (msm_sdcc) | Mainline (original) | Ratio |
|------|------------------|---------------------|-------|
| Cached read | ~340 MB/s | ~66 MB/s | 19% |
| Uncached DMA | ~22 MB/s | ~17 MB/s | 77% |
| Uncached PIO | N/A | ~6 MB/s | - |

The ~25% performance gap vs webOS for uncached reads was primarily due to
per-transfer allocation overhead.

## Analysis

### webOS msm_sdcc Approach

The webOS kernel's msm_sdcc driver uses pre-allocated DMA buffers:

```c
struct msmsdcc_nc_dmadata {
    dmov_box    cmd[NR_SG];  // 32 box descriptors (768 bytes)
    uint32_t    cmdptr;       // Command pointer (4 bytes)
};
```

- Allocated once at probe via `dma_alloc_coherent()` (~772 bytes)
- Reused for every transfer - just overwrites descriptors
- No per-transfer allocation or DMA mapping
- Max 32 SG entries per transfer

### Linux Kernel DMA Patterns

Examined similar patterns in mainline drivers:

1. **Xilinx AXIDMA** (xilinx_dma.c):
   - Pre-allocates 512 descriptors via `dma_alloc_coherent()`
   - Uses free list with spinlock protection
   - Zero per-transfer allocation overhead

2. **Atmel HDMA** (at_hdmac.c):
   - Uses `dma_pool_create()` for descriptor allocation
   - Per-transfer allocation from pool

3. **FSL eDMA** (fsl-edma-common.c):
   - Hybrid: `dma_pool` for hardware descriptors
   - kzalloc for software wrapper

## Solution

Implemented a hybrid approach combining webOS and Xilinx patterns:

### Memory Layout

```
CPL Pool (256 KB coherent DMA memory):
+------------------+------------------+-----+
| CPL Buffer 0     | CPL Buffer 1     | ... | (128 x 2KB)
+------------------+------------------+-----+

Descriptor Array (13 KB regular memory):
+------------------+------------------+-----+
| adm_async_desc 0 | adm_async_desc 1 | ... | (128 structs)
| cpl -> pool[0]   | cpl -> pool[1]   |     |
+------------------+------------------+-----+
```

### Key Changes

1. **Pool Constants**:
   ```c
   #define ADM_MAX_SG_PER_DESC     64      /* Max SG entries per pooled desc */
   #define ADM_CPL_BUF_SIZE        2048    /* CPL buffer size per descriptor */
   #define ADM_DESC_POOL_SIZE      8       /* Descriptors per channel */
   #define ADM_TOTAL_DESC_POOL     128     /* 16 channels * 8 */
   ```

2. **New Data Structures**:
   - Added `pool_node` and `pool_index` to `struct adm_async_desc`
   - Added pool management fields to `struct adm_device`

3. **New Functions**:
   - `adm_desc_pool_init()` - Allocate coherent pool at probe
   - `adm_desc_pool_destroy()` - Free pool at remove
   - `adm_desc_get()` - O(1) allocation from free list
   - `adm_desc_put()` - O(1) return to free list
   - `adm_desc_alloc_fallback()` - Dynamic alloc for oversized transfers

4. **Modified Functions**:
   - `adm_prep_slave_sg()` - Uses pool instead of kzalloc
   - `adm_dma_free_desc()` - Returns to pool or frees dynamic

### Per-Transfer Overhead (After Optimization)

```
Pooled transfer:
- adm_desc_get()                O(1) list operation
- Fill descriptors              Same as before
- Set cmdptr                    No sync needed (coherent)
- adm_desc_put()                O(1) list operation

Fallback (rare, >64 SG entries):
- Same as original path
- Uses dma_alloc_coherent (not streaming DMA)
```

## Memory Usage

| Component | Size |
|-----------|------|
| CPL pool (coherent) | 256 KB |
| Descriptor array | ~13 KB |
| **Total** | **~270 KB** |

This is a one-time allocation at driver probe, eliminating all per-transfer
allocation overhead for typical transfers.

## Trade-offs

### Advantages
- Zero per-transfer allocation for typical transfers (<=64 SG entries)
- Eliminates dma_map_single/unmap calls
- Eliminates dma_sync calls (coherent memory)
- O(1) descriptor allocation from pool
- Maintains virt-dma framework compatibility
- Graceful fallback for oversized transfers

### Limitations
- Fixed 270 KB memory overhead
- Pool exhaustion triggers fallback (rare with 128 descriptors)
- 64 SG entry limit per pooled descriptor

## Testing

Tested on HP TouchPad (APQ8060) with LuneOS:

```bash
# Uncached read test
sync; echo 3 > /proc/sys/vm/drop_caches
time dd if=/dev/mmcblk0 of=/dev/null bs=1M count=64
```

ADM interrupts verified via `/proc/interrupts`.

## References

- webOS kernel: `drivers/mmc/host/msm_sdcc.c` (lines 330-437, 1376-1387)
- webOS header: `drivers/mmc/host/msm_sdcc.h` (msmsdcc_nc_dmadata struct)
- Xilinx AXIDMA: `drivers/dma/xilinx/xilinx_dma.c` (lines 1127-1192)
- Original ADM analysis: `reports/adm-dma-analysis-detail.md`
