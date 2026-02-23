# Plan: Solving Intermittent Faceted Rendering on Adreno 220 (HP TouchPad)

## Problem Summary

The HP TouchPad's Adreno 220 (Leia) GPU exhibits **intermittent faceted rendering** where smooth shading randomly appears as faceted/flat shading. The issue is:
- **Intermittent**: Success rate varies from 10-40% smooth across test runs
- **Timing-related**: Register dumps show identical state between smooth and faceted runs
- **Not shader compilation**: Same compiled shaders produce both smooth and faceted results

## Current Status

With patches 0001-0018 applied:
- Patches 0001-0017: Various synchronization and initialization fixes
- Patch 0018: Synchronous cache flush (CACHE_FLUSH_AND_INV_EVENT) - **TESTING**

---

## Session Log (2026-02-23)

### Attempted and Reverted

| Patch | Description | Result | Action |
|-------|-------------|--------|--------|
| 0018 (v1) | Initialize A220_GRAS_CONTROL to 0 | **Grey textures** | REVERTED |
| 0019 | Set PS_REGS high bit (0x80) for A22X | **Grey textures** | REVERTED |
| 0020 | CP_DRAW_INDX_2 dummy draw | **Grey textures** | REVERTED |

**Root cause of grey textures:**
- PS_REGS `|= 0x80` corrupted shader register allocation (was ORing with actual count, not replacing)
- CP_DRAW_INDX_2 dummy draw corrupted primitive/texture state
- GRAS_CONTROL=0 may have disabled rasterizer features

### Key Discovery: Async vs Sync Cache Flush

**Found in KGSL/adreno_pm4.xml:**
```c
CACHE_FLUSH = 0x06              // Asynchronous (freedreno uses this)
CACHE_FLUSH_AND_INV_EVENT = 0x16 // Synchronous, A2XX only (KGSL has this)
VS_FETCH_DONE = 0x1b            // Vertex fetch completion event
```

**Problem:** Freedreno uses async `CACHE_FLUSH` (12 times in a loop), but this doesn't wait for completion. KGSL defines `CACHE_FLUSH_AND_INV_EVENT` specifically for A2XX which is synchronous.

### New Patch 0018 (v2) - Synchronous Cache Flush

**Changes:**
1. Replace async `CACHE_FLUSH` with sync `CACHE_FLUSH_AND_INV_EVENT` for A22X
2. Add WFI after TC_CNTL_STATUS L2 cache invalidation
3. Keep VGT DMA wait from patch 0016

**File:** `fd2_draw.c`

```c
// emit_cacheflush now takes sync parameter
static void emit_cacheflush(struct fd_ringbuffer *ring, bool sync) {
   if (sync) {
      OUT_PKT3(ring, CP_EVENT_WRITE, 1);
      OUT_RING(ring, CACHE_FLUSH_AND_INV_EVENT);  // 0x16 - synchronous
   } else {
      for (i = 0; i < 12; i++) {
         OUT_PKT3(ring, CP_EVENT_WRITE, 1);
         OUT_RING(ring, CACHE_FLUSH);  // 0x06 - async
      }
   }
}

// In draw_impl():
OUT_PKT0(ring, REG_A2XX_TC_CNTL_STATUS, 1);
OUT_RING(ring, A2XX_TC_CNTL_STATUS_L2_INVALIDATE);
if (is_a22x(ctx->screen))
   OUT_WFI(ring);  // Wait for L2 invalidation to complete

// At end of draw:
emit_cacheflush(ring, is_a22x(ctx->screen));
```

---

## What Has Been Tried and Excluded

### Excluded as Root Cause

| Area | Evidence | Conclusion |
|------|----------|------------|
| Shader compilation | Same shaders produce both results | NOT the cause |
| Mesa state management | Register dumps identical | NOT the cause |
| Blend state | Logs show correct values | NOT the cause |
| Vertex buffer content | CRC32 matches between runs | NOT the cause |
| GPR allocation | Fixed with SQ_GPR_MANAGEMENT init | Addressed |
| TP0_CHICKEN=0x00 | Causes GPU hang | Must stay at 0x02 |
| A20x-style dummy draw | Causes GPU hang on A22X | Cannot use CP_DRAW_INDX_BIN |
| CP_DRAW_INDX_2 dummy draw | Causes grey textures | Cannot use for sync |
| PS_REGS \|= 0x80 | Causes grey textures | Corrupts register count |
| GRAS_CONTROL=0 | Causes grey textures | May disable features |

### Patches Applied (Current)

| Patch | Description | Result |
|-------|-------------|--------|
| 0001 | is_a22x() helper | Required |
| 0002-0005 | Shader/debug improvements | Helpful for debugging |
| 0006 | WFI after draw | Partial help |
| 0007 | SQ_GPR_MANAGEMENT init | Required |
| 0008 | SQ_INTERPOLATOR_CNTL after SQ_PROGRAM_CNTL | Required for correctness |
| 0009-0013 | Debug logging | Helpful for analysis |
| 0014 | WFI after constant emission | Partial help |
| 0015 | WFI after vertex buffer setup | Partial help |
| 0016 | VGT DMA wait (CP_WAIT_REG_EQ) | Improved to ~40% |
| 0017 | VSC register initialization | Inconclusive |
| 0018 | Synchronous cache flush | **TESTING** |

---

## Remaining Possibilities to Investigate

### Priority: HIGH

#### 1. GMEM Tile Operations Synchronization
- No explicit WFI around RB_MODECONTROL changes
- Tile load/store may have race conditions
- **File:** `fd2_gmem.c`

#### 2. Kernel-Side Context Switch
- DRM/MSM driver may not save/restore all Leia-specific registers
- KGSL saves fragmented register ranges for Leia
- **File:** `drivers/gpu/drm/msm/adreno/a2xx_gpu.c`

#### 3. VS_FETCH_DONE Event
- KGSL defines this event (0x1b) for vertex fetch completion
- Freedreno doesn't use it
- Could add before draw to ensure vertex data is ready

### Priority: MEDIUM

#### 4. SQ Context Switch Timing
- Shader state may not be stable between draws
- GPR allocation, instruction base timing

#### 5. Interpolator CNTL vs Varying Linkage
- Even with WFI, VS export count and PS input linkage may race

---

## Implementation Order (Updated)

1. ✅ **Patch 0018 (v2)**: Synchronous cache flush - IN TESTING
2. ⏳ **Patch 0019**: Add VS_FETCH_DONE event before draw
3. ⏳ **Patch 0020**: GMEM operation synchronization
4. ⏳ **Kernel audit**: Verify all Leia registers saved/restored

---

## Verification Plan

For each patch:
1. Build Mesa: `bitbake mesa -c compile -f && bitbake mesa`
2. Deploy: `scp .../package/usr/lib/libgallium*.so root@172.16.42.2:/usr/lib/`
3. Verify binary: `strings libgallium*.so | grep "unique_string_from_patch"`
4. Run 10 glmark2 build tests, record smooth/faceted ratio
5. If GPU hangs or grey textures: revert immediately, note failure mode

---

## Key Files

| File | Purpose |
|------|---------|
| `fd2_emit.c` | Context restore, register initialization |
| `fd2_program.c` | Shader program setup, SQ_PROGRAM_CNTL |
| `fd2_draw.c` | Draw command emission, sync points, cache flush |
| `fd2_gmem.c` | GMEM tile operations |
| `a2xx_gpu.c` (kernel) | GPU init, context switch |

---

## Reference Documents

- `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/kgsl-vs-freedreno-a220-analysis.md`
- `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/a22x-vgt-dma-workaround-analysis.md`
- Legacy KGSL: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/`
- adreno_pm4.xml: `/home/herrie/Documents/GitHub/mesa-latest/src/freedreno/registers/adreno/adreno_pm4.xml`

---

## Likelihood Assessment

**Patch 0018 (sync cache flush):** 30-50% chance of significant improvement
- Addresses real gap between freedreno (async) and KGSL (sync available)
- Cache coherency is plausible cause for intermittent issues
- But we've tried many sync approaches with only incremental improvement
- May need combination of fixes for complete solution
