# webOS Kernel Driver vs freedreno Register Analysis

## Summary

Analysis of the webOS kernel KGSL driver (`kgsl_drawctxt.c`) compared to the Mesa freedreno A2XX driver for Adreno 220 (Leia).

## Key Register Differences

### VGT_VERTEX_REUSE_BLOCK_CNTL (0x2316)

| Driver | Value | Notes |
|--------|-------|-------|
| **webOS** | `0x00000002` | VTX_REUSE_DEPTH = 2 |
| **freedreno** | `0x0000028f` | VTX_REUSE_DEPTH = 7 + extra bits set |

**SIGNIFICANT DIFFERENCE!** freedreno sets bits 3, 7, and 9 which are not documented. webOS uses a simpler value.

### SQ_INTERPOLATOR_CNTL (0x2182)

| Driver | Value | Notes |
|--------|-------|-------|
| **webOS** | `0xffffffff` | All varyings use smooth interpolation |
| **freedreno** | `0xffffffff` | Same |

**MATCH** - Both drivers set all varyings to smooth interpolation.

### PA_SU_SC_MODE_CNTL (0x2205)

| Driver | Value | Notes |
|--------|-------|-------|
| **webOS** | `0x00080240` | PROVOKING_VTX_LAST, FRONT/BACK_PTYPE=triangles |
| **freedreno** | `0x00080240` | Same |

**MATCH**

### VGT_OUT_DEALLOC_CNTL (0x2317)

| Driver | Value | Notes |
|--------|-------|-------|
| **webOS** | `0x00000002` | DEALLOC_DIST = 2 |
| **freedreno** | `0x00000002` | Same |

**MATCH**

### RB_DEPTHCONTROL

| Driver | Value | Notes |
|--------|-------|-------|
| **webOS (Leia)** | `0x08` | Special value for A220 |
| **webOS (other)** | `0x00` | Default |

## LEIA (A220) Specific Registers

The webOS driver has special handling for LEIA_REV470 (Adreno 220):

1. **REG_LEIA_RB_LRZ_VSC_CONTROL (0x2209)** - Set to 0 during context setup
2. **REG_LEIA_GRAS_CONTROL (0x2210)** - GRAS control register
3. **REG_LEIA_VSC_BIN_SIZE (0x0C01)** - Visibility stream control
4. **REG_LEIA_SQ_PIX_IN_CNTL (0x0D0C)** - Pixel input control

## webOS Context Save/Restore Registers

The webOS driver saves/restores these register ranges for LEIA:

- `REG_LEIA_PC_MAX_VTX_INDX` to `REG_LEIA_PC_INDX_OFFSET`
- `REG_LEIA_GRAS_CONTROL`
- `REG_LEIA_PC_VERTEX_REUSE_BLOCK_CNTL`
- `REG_LEIA_VSC_BIN_SIZE` to `REG_LEIA_VSC_PIPE_DATA_LENGTH_7`

## Recommendations

1. **Change VGT_VERTEX_REUSE_BLOCK_CNTL from 0x0000028f to 0x00000002**
   - This matches the webOS driver exactly
   - Current freedreno value has undocumented bits set

2. **Verify LEIA-specific registers are being set:**
   - REG_LEIA_RB_LRZ_VSC_CONTROL should be initialized to 0
   - REG_LEIA_GRAS_CONTROL might need initialization

3. **Check RB_DEPTHCONTROL initialization** for A220

## Source Files Analyzed

- webOS: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/kgsl_drawctxt.c`
- webOS: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/leia_reg.h`
- freedreno: `src/gallium/drivers/freedreno/a2xx/fd2_emit.c`
- freedreno: `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c`

## Date

Generated: 2026-04-13
