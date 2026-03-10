# Adreno 2XX Cache Coherency Analysis: KGSL vs Freedreno

## Executive Summary

**Critical Finding:** The KGSL driver explicitly flushes CPU caches before GPU access, while freedreno relies solely on GPU-side cache invalidation. On write-combine memory without hardware cache coherency (A2XX), this can cause vertex data corruption.

## The Problem

Adreno 2XX (A200/A220/A225) does NOT support hardware cache coherency:
- All buffers use `MSM_BO_WC` (write-combine) memory
- CPU writes may remain in write-combine buffers
- GPU may read stale/uninitialized data

## KGSL Cache Synchronization (webOS)

### CPU Cache Flush Operations

**File:** `kgsl_sharedmem.c:76-78`
```c
if (flags & KGSL_MEMFLAGS_CACHE_FLUSH)
    dmac_flush_range((const void *)addr,
                     (const void *)(addr + size));
```

**File:** `kgsl_sharedmem.c:55`
```c
outer_flush_range(physaddr, physaddr + KGSL_PAGESIZE);
```

### Memory Barriers Before GPU Submission

**File:** `kgsl_ringbuffer.c:165-174`
```c
/* Drain write buffer and data memory barrier */
dsb();           /* Data synchronization barrier */
wmb();           /* Write memory barrier */

/* Memory fence to ensure all data has posted */
mb();            /* Full memory barrier */
outer_sync();    /* Outer cache sync */
```

### Ring Buffer Uses Coherent Memory

**File:** `kgsl_sharedmem.h:85-96`
```c
memdesc->hostptr = dma_alloc_coherent(NULL, size, &memdesc->physaddr,
                                      GFP_KERNEL);
```

KGSL uses DMA-coherent memory for:
- Ring buffers
- Memory pointers
- Critical command data

## Freedreno Cache Synchronization (Mesa)

### GPU-Side Only Invalidation

**File:** `fd2_draw.c:96-97`
```c
OUT_PKT0(ring, REG_A2XX_TC_CNTL_STATUS, 1);
OUT_RING(ring, A2XX_TC_CNTL_STATUS_L2_INVALIDATE);
```

**File:** `fd2_draw.c:51-59` (Post-draw)
```c
static void
emit_cacheflush(struct fd_ringbuffer *ring)
{
   unsigned i;
   for (i = 0; i < 12; i++) {
      OUT_PKT3(ring, CP_EVENT_WRITE, 1);
      OUT_RING(ring, CACHE_FLUSH);
   }
}
```

### No CPU Cache Flush

Freedreno does NOT call:
- `dmac_flush_range()` - CPU L1 cache flush
- `outer_flush_range()` - CPU L2 cache flush
- Any memory barriers before GPU access

## Why This Causes Intermittent Artifacts

1. **CPU writes vertex data** to write-combine buffer
2. **Write-combine buffer NOT flushed** to physical memory
3. **GPU accesses vertex buffer** via DMA
4. **GPU reads stale/garbage data** because CPU writes haven't reached memory

### Why It's Intermittent (~80% failure rate)

- Sometimes write-combine buffer flushes naturally (timing)
- Sometimes buffer is still empty when GPU reads
- CPU load affects flush timing
- GMEM tile boundaries affect access patterns

## The Critical Gap

| Operation | KGSL | Freedreno |
|-----------|------|-----------|
| CPU cache flush before GPU | **dmac_flush_range()** | **NONE** |
| L2 cache flush | **outer_flush_range()** | **NONE** |
| Memory barriers | **dsb/wmb/mb/outer_sync** | **NONE** |
| Ring buffer memory | **dma_alloc_coherent** | **Write-combine** |
| GPU cache invalidate | TC_CNTL_STATUS | TC_CNTL_STATUS |
| Post-draw GPU flush | CACHE_FLUSH events | CACHE_FLUSH events |

## Proposed Solutions

### Option 1: Kernel-Side Cache Flush (Recommended)

Add cache synchronization in the MSM DRM driver before GPU command submission.

**File:** `drivers/gpu/drm/msm/msm_gem_submit.c`

Before submitting commands:
```c
/* Flush CPU caches for non-coherent platforms */
if (!adreno_gpu->info->quirks & ADRENO_QUIRK_HAS_CACHED_COHERENT) {
    for each buffer in submit:
        dma_sync_single_for_device(dev, bo->paddr, bo->size, DMA_TO_DEVICE);
}
```

### Option 2: Use Uncached/Coherent Memory

Force DMA coherent memory for vertex buffers on A2XX:
```c
if (is_a2xx(screen))
    flags |= MSM_BO_UNCACHED;  /* or use dma_alloc_coherent */
```

### Option 3: Explicit Sync in Mesa

Add explicit buffer sync calls in freedreno before draws:
```c
/* In fd2_draw.c before draw_impl() */
fd_resource_flush(ctx, vertex_buffer);
```

## VGT Configuration Comparison

Both drivers configure VGT similarly:

| Register | KGSL | Freedreno |
|----------|------|-----------|
| VGT_MAX_VTX_INDX | 0x00ffffff | 0xffffffff |
| VGT_MIN_VTX_INDX | 0x0 | 0x0 |
| VGT_INDX_OFFSET | 0x0 | 0x0 (or draw->start) |
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x2 | 0x2 (A20x) / 0x3b (A22x) |

The VGT configuration is similar - the difference is in cache coherency, not VGT setup.

## L2 Cache Invalidation Points

### KGSL L2 Invalidation

**File:** `kgsl_drawctxt.c:689-691`
```c
/* Invalidate L2 cache to make sure vertices are updated */
*cmds++ = pm4_type0_packet(REG_TC_CNTL_STATUS, 1);
*cmds++ = 0x1;  /* Invalidate L2 */
```

This happens:
- Before quad vertex setup in gmem save (gmem→sys)
- Before texture/vertex data load in gmem restore (sys→gmem)

### Freedreno L2 Invalidation

**File:** `fd2_draw.c:96-97`
```c
OUT_PKT0(ring, REG_A2XX_TC_CNTL_STATUS, 1);
OUT_RING(ring, A2XX_TC_CNTL_STATUS_L2_INVALIDATE);
```

This happens:
- Before every draw in draw_impl()
- Before mem2gmem transfers

Both invalidate the GPU's texture cache (L2) - but this only helps if data is already in physical memory. If CPU hasn't flushed its cache, the GPU still reads stale data.

## Conclusion

The root cause of the rendering artifacts is likely **missing CPU cache synchronization** before GPU vertex access:

1. **TC_CNTL_STATUS_L2_INVALIDATE** only invalidates GPU's texture cache
2. **MH_MMU_INVALIDATE** only invalidates GPU's MMU TLB
3. **Neither flushes CPU write-combine buffers to physical memory**

The fix must ensure CPU writes reach physical memory before GPU reads:
- Either via `dma_sync_*` calls in the kernel
- Or via coherent memory allocation
- Or via Mesa calling into kernel for explicit sync

## References

- KGSL source: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/drivers/gpu/msm/`
- freedreno source: Mesa `src/gallium/drivers/freedreno/a2xx/`
- ARM cache architecture: ARMv7 dmac_flush_range flushes L1 D-cache
- Outer cache: L2 cache shared between cores
