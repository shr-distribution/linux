# KGSL vs Freedreno: Adreno 220 (Leia) Deep Analysis

## Executive Summary

This document provides an exhaustive comparison between the proprietary KGSL driver (used in webOS) and the open-source freedreno Mesa driver for the Adreno 220 (Leia) GPU found in the HP TouchPad.

**Key Finding:** Multiple critical differences exist between KGSL and freedreno that could explain the rendering artifacts (triangles instead of smooth surfaces) seen in ~80% of glmark2 build test runs.

---

## GPU Identification

| Property | Yamato (A200) | Leia (A220) |
|----------|---------------|-------------|
| Chip ID (KGSL) | 0x20100/0x20101 | 0x2010000 |
| GPU ID (freedreno) | < 220 | 220-229 |
| GMEM Size | 256 KB | **512 KB** |
| Codename | Yamato | Leia |

The HP TouchPad uses **Adreno 220 (Leia)**, NOT Yamato.

---

## Critical Differences Found

### 1. RBBM_PM_OVERRIDE2 Register

| Driver | Value | Purpose |
|--------|-------|---------|
| **KGSL** | **0x1a0** | Leia-specific power domain control |
| **freedreno** | 0x0 | Generic value (no Leia handling) |

**KGSL Code (kgsl_yamato.c:834-837):**
```c
if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
    kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0);
else
    kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x1a0);  // Leia-specific
```

**freedreno Code (a2xx_gpu.c):**
```c
gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0); /* 0x80/0x1a0 for a22x? */
```
Note the comment indicating uncertainty about A22x value!

**Impact:** Power management affects GPU block availability and timing. Wrong value could cause intermittent rendering failures.

---

### 2. MH_CLNT_INTF_CTRL_CONFIG1 Register

| Driver | Value | Notes |
|--------|-------|-------|
| **KGSL (Leia)** | **0x00032f07** | Avoids 1KB boundary GPU hang |
| **KGSL (Yamato)** | 0x00030f27 | Different burst config |
| **freedreno** | 0x00032f07 | Matches KGSL Leia (correct) |

**KGSL Comment:**
> "Remove 1k boundary check in z470 to avoid GPU hang. This solution won't work if both EBI and SMI are used"

This is correctly handled in freedreno.

---

### 3. LRZ/VSC Control Register (REG_A2XX_A220_RB_LRZ_VSC_CONTROL)

**This is a Leia-ONLY register at 0x2209.**

| Operation | KGSL Value | freedreno Value |
|-----------|------------|-----------------|
| Context Init | 0x0 | 0x0 (correct) |
| Clear State | 0x84 | **0x84** (correct) |
| GMEM2SYS | 0x0 | **Not set** |
| SYS2GMEM | 0x0 | **Not set** |

**KGSL sets this register during GMEM operations:**
```c
// kgsl_drawctxt.c:825-829 (gmem2sys)
if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(REG_LEIA_RB_LRZ_VSC_CONTROL);
    *cmds++ = 0;
}
```

**freedreno sets it in clear_state() but NOT in GMEM operations!**

**Impact:** LRZ (Low Resolution Z) and VSC (Visibility Stream Cache) control visibility culling. Missing initialization during GMEM tile operations could cause incorrect pixel visibility decisions.

---

### 4. Draw Command Format

| Driver | Packet | Format |
|--------|--------|--------|
| **KGSL (Leia)** | **PM4_DRAW_INDX_2** (0xc0043600) | 6 dwords, triangle strip |
| **KGSL (Yamato)** | PM4_DRAW_INDX | 3 dwords, rect list |
| **freedreno** | CP_DRAW_INDX | Same for both (no Leia special handling) |

**KGSL Leia Draw (kgsl_drawctxt.c:835-841):**
```c
*cmds++ = 0xc0043600;        // PM4_DRAW_INDX_2 packet
*cmds++ = 0x0;               // Query info
*cmds++ = 0x00004046;        // Triangle strip primitive
*cmds++ = 0x00000004;        // NUM_INDICES = 4
*cmds++ = 0x00010000;        // Index pair: 0x00, 0x01
*cmds++ = 0x00030002;        // Index pair: 0x02, 0x03
```

**KGSL Yamato Draw:**
```c
*cmds++ = pm4_type3_packet(PM4_DRAW_INDX, 2);
*cmds++ = 0;                 // viz query info
*cmds++ = 0x00030088;        // PrimType=RectList, NumIndices=3
```

**freedreno uses the same CP_DRAW_INDX for all A2XX variants.**

**Impact:** Different draw command format could affect primitive assembly and visibility calculations.

---

### 5. SQ_PROGRAM_CNTL Value

| Driver | Value | Bit 17 |
|--------|-------|--------|
| **KGSL (Leia)** | **0x10018001** | Set (1) |
| **KGSL (Yamato)** | 0x10010001 | Clear (0) |
| **freedreno** | Varies | Dynamically calculated |

**KGSL Code (kgsl_drawctxt.c:717-720):**
```c
if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
    *cmds++ = 0x10018001;  // Leia-specific value
else
    *cmds++ = 0x10010001;  // Standard value
```

**Impact:** SQ_PROGRAM_CNTL controls shader execution. Bit 17 may affect shader fetching or execution mode.

---

### 6. RB_DEPTHCONTROL During GMEM Operations

| Driver | Leia Value | Yamato Value |
|--------|------------|--------------|
| **KGSL** | **0x08** | 0x00 |
| **freedreno** | ? | ? |

**KGSL Code (kgsl_drawctxt.c:755-758):**
```c
if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
    *cmds++ = 0x08;  // Leia depth control value
else
    *cmds++ = 0;     // Standard (no depth control)
```

**Impact:** Depth control affects Z-buffer operations during GMEM tile transfers.

---

### 7. VSC Pipe Data Registers (Leia-Only)

KGSL saves/restores 8 VSC registers during context switches:
- REG_LEIA_VSC_BIN_SIZE (0x0C01)
- Through REG_LEIA_VSC_PIPE_DATA_LENGTH_7 (0x0C1D)

**KGSL Code (kgsl_drawctxt.c:620-630):**
```c
if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
    for (i = REG_LEIA_VSC_BIN_SIZE; i <= REG_LEIA_VSC_PIPE_DATA_LENGTH_7; i++) {
        *cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
        *cmd++ = i;
        *cmd++ = ctx->reg_values[j];
        j++;
    }
}
```

**freedreno does NOT handle VSC pipe registers for A22x!**

**Impact:** VSC (Visibility Stream Cache) handles hardware binning/visibility. Missing register save/restore could corrupt visibility state between draws.

---

### 8. Soft Reset Behavior

| Driver | First Reset | Warm Reset |
|--------|-------------|------------|
| **KGSL (Leia)** | 0xFFFFFFFF (all blocks) | **0x00000001** (CP only) |
| **KGSL (Yamato)** | 0xFFFFFFFF | 0xFFFFFFFF |
| **freedreno** | 0xFFFFFFFF | 0xFFFFFFFF |

**KGSL Code (kgsl_yamato.c:797-802):**
```c
if (!(device->flags & KGSL_FLAGS_SOFT_RESET) ||
    (device->chip_id != KGSL_CHIPID_LEIA_REV470)) {
    kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0xFFFFFFFF);
} else {
    // Leia: Reset only CP block
    kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0x00000001);
}
```

**Impact:** Leia cannot handle full reset after initialization - CP would hang. freedreno may cause GPU hangs on recovery.

---

### 9. Hardware Binning

| Driver | A20x | A22x |
|--------|------|------|
| **KGSL** | Implemented | Implemented (different format) |
| **freedreno** | Implemented | **NOT implemented (disabled)** |

**freedreno Code (fd2_gmem.c:79-99):**
```c
/* only a20x hw binning is implement
 * a22x is more like a3xx, but perhaps the a20x works? (TODO)
 */
if (!is_a20x(batch->ctx->screen))
    return false;  // A22x binning NOT implemented
```

**Impact:** Missing HW binning means all rendering uses tile-based deferred rendering without visibility optimization. This shouldn't cause artifacts but affects performance.

---

### 10. VGT_VERTEX_REUSE_BLOCK_CNTL Register

| Operation | KGSL Leia | freedreno A22x |
|-----------|-----------|----------------|
| Init | ? | 0x0000003b |
| Clear State | 0x0000028f | 0x0000028f (correct) |
| GMEM Tile Store | 0x0000028f | 0x0000028f (correct) |
| Restore | 0x0000003b | 0x0000003b (correct) |

This appears to be correctly handled in freedreno.

---

### 11. Register Save/Restore Ranges

KGSL uses different register ranges for Leia vs Yamato:

**Leia-Specific Ranges:**
- REG_LEIA_PC_MAX_VTX_INDX to REG_LEIA_PC_INDX_OFFSET (instead of VGT_*)
- REG_RB_MODECONTROL to REG_LEIA_GRAS_CONTROL (extra GRAS_CONTROL)
- REG_LEIA_PC_VERTEX_REUSE_BLOCK_CNTL (instead of VGT_VERTEX_REUSE_BLOCK_CNTL)

**freedreno uses VGT-based ranges for all A2XX, not the PC-based Leia registers.**

---

### 12. Bin Base Offset Command

| Driver | Behavior |
|--------|----------|
| **KGSL (Leia)** | **Skipped** |
| **KGSL (Yamato)** | Issued |
| **freedreno** | ? |

**KGSL Code (kgsl_drawctxt.c:1921-1924):**
```c
cmds[0] = pm4_type3_packet(PM4_SET_BIN_BASE_OFFSET, 1);
cmds[1] = drawctxt->bin_base_offset;
if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
    kgsl_ringbuffer_issuecmds(device, 0, cmds, 2);
// Leia: Bin base offset command is NOT issued
```

---

## Summary of Missing Leia-Specific Handling in freedreno

| Issue | Severity | Description |
|-------|----------|-------------|
| **RBBM_PM_OVERRIDE2** | HIGH | Wrong power management value (0 vs 0x1a0) |
| **LRZ_VSC_CONTROL in GMEM** | HIGH | Not set during GMEM tile operations |
| **VSC Pipe Registers** | MEDIUM | Not saved/restored during context switch |
| **Draw Command Format** | MEDIUM | Uses generic format, not PM4_DRAW_INDX_2 |
| **SQ_PROGRAM_CNTL bit 17** | MEDIUM | Not set for Leia |
| **RB_DEPTHCONTROL in GMEM** | LOW | May use wrong value (0 vs 0x08) |
| **PC-based registers** | LOW | Uses VGT-based instead of PC-based |
| **Soft Reset** | LOW | May cause hang on GPU recovery |

---

## Recommended Fixes

### Priority 1: RBBM_PM_OVERRIDE2

Add to a2xx_gpu.c `a2xx_hw_init()`:
```c
if (adreno_is_a220(adreno_gpu) || adreno_is_a225(adreno_gpu))
    gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0x1a0);
else
    gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0);
```

### Priority 2: LRZ_VSC_CONTROL in GMEM Operations

Add to fd2_gmem.c GMEM operations:
```c
if (is_a22x(batch->ctx->screen)) {
    OUT_PKT3(ring, CP_SET_CONSTANT, 2);
    OUT_RING(ring, CP_REG(REG_A2XX_A220_RB_LRZ_VSC_CONTROL));
    OUT_RING(ring, 0x00000000);
}
```

### Priority 3: Investigate VSC Pipe Register Handling

The 8 VSC registers (0x0C01-0x0C1D) may need to be properly initialized or saved/restored for correct visibility streaming on Leia.

---

## Testing Methodology

To verify fixes:

1. Run glmark2 build test 20 times
2. Count smooth (correct) vs triangulated (artifact) results
3. Before fix: ~20% success rate
4. Target: 100% success rate

```bash
for i in $(seq 1 20); do
    timeout 20 glmark2-es2-drm -b build 2>&1 | grep FPS
    # Observe display for artifacts
done
```

---

## Files Referenced

### KGSL (webOS kernel):
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/drivers/gpu/msm/kgsl_yamato.c`
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/drivers/gpu/msm/kgsl_drawctxt.c`
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/drivers/gpu/msm/kgsl_ringbuffer.c`
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/drivers/gpu/msm/leia_reg.h`

### freedreno (Mesa):
- `src/gallium/drivers/freedreno/a2xx/fd2_emit.c`
- `src/gallium/drivers/freedreno/a2xx/fd2_draw.c`
- `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c`
- `src/gallium/drivers/freedreno/freedreno_screen.h`

### Mainline kernel:
- `drivers/gpu/drm/msm/adreno/a2xx_gpu.c`

---

## Conclusion

The rendering artifacts on the HP TouchPad with Adreno 220 are likely caused by missing Leia-specific register initialization in freedreno, particularly:

1. **RBBM_PM_OVERRIDE2 = 0x1a0** (power management)
2. **LRZ_VSC_CONTROL = 0** during GMEM operations (visibility control)
3. **VSC pipe register handling** (hardware binning/visibility)

These differences affect GPU power states, visibility calculations, and tile-based rendering - all of which could cause intermittent rendering failures matching the observed 80% artifact rate.

---

## Implementation Status (Updated Feb 8, 2026)

### Kernel Fixes (Already Applied)

| Fix | Status | Commit/File |
|-----|--------|-------------|
| RBBM_PM_OVERRIDE2 = 0x1a0 for A22x | **DONE** | `a2xx_gpu.c` lines 190-194 |
| Full GPU reset sequence | **DONE** | `a2xx_gpu.c` a2xx_recover() |
| GPU bandwidth scaling | **DONE** | `a2xx_gpu.c` a2xx_gpu_set_freq() |

### Mesa Patches Created

| Patch | File | Description |
|-------|------|-------------|
| 0002 | `freedreno_screen.h` | `is_a22x()` helper function |
| 0003 | `fd2_emit.c` | LRZ_VSC_CONTROL in emit_restore() |
| 0005 | `fd2_gmem.c` | Cache flush after GMEM operations |
| 0009 | `fd2_emit.c` | TP0_CHICKEN = 0 to match KGSL |
| 0010 | `fd2_draw.c` | MH cache invalidate before draw |
| **0011** | `fd2_gmem.c` | **LRZ_VSC_CONTROL = 0 during GMEM tile store** |
| **0012** | `fd2_emit.c` | **RBBM_PM_OVERRIDE2 = 0x1a0 for A22x in emit_restore** |

### Key Finding: Mesa Was Overriding Kernel Setting

A critical issue discovered: Mesa's `fd2_emit_restore()` was setting `RBBM_PM_OVERRIDE2 = 0x00000fff` for all A2XX GPUs, overriding the kernel's correct `0x1a0` setting for Leia.

**fd2_emit.c original code:**
```c
OUT_PKT0(ring, REG_A2XX_RBBM_PM_OVERRIDE1, 2);
OUT_RING(ring, 0xffffffff);
OUT_RING(ring, 0x00000fff);  // WRONG for A22x!
```

**Fixed code (patch 0012):**
```c
OUT_PKT0(ring, REG_A2XX_RBBM_PM_OVERRIDE1, 2);
OUT_RING(ring, 0xffffffff);
OUT_RING(ring, is_a22x(ctx->screen) ? 0x000001a0 : 0x00000fff);
```

This explains why the kernel fix alone didn't resolve the artifacts - the GPU command stream was immediately overwriting the correct value.

### Remaining Items to Investigate

| Issue | Priority | Status |
|-------|----------|--------|
| VSC Pipe Registers (0x0C01-0x0C1D) | MEDIUM | Not yet investigated |
| Draw Command Format (PM4_DRAW_INDX_2) | MEDIUM | Not yet investigated |
| SQ_PROGRAM_CNTL bit 17 | LOW | Not yet investigated |
| RB_DEPTHCONTROL in GMEM (0x08) | LOW | Not yet investigated |
