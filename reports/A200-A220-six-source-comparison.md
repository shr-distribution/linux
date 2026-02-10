# Adreno A200 vs A220 Register Comparison: All Sources

## Sources

| Source | Description |
|--------|-------------|
| **Sony KGSL** | github.com/hsr0/android_kernel_sony_msm7x27a (cm-11.0) |
| **TouchPad KGSL** | github.com/webos-internals/webos-linux-kernel (touchpad branch) |
| **Mesa 24.0.7** | gitlab.freedesktop.org/mesa/mesa (tag: mesa-24.0.7) |

---

## VGT_VERTEX_REUSE_BLOCK_CNTL (0x2316)

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **emit_restore / init** | 0x02 | *N/A* | 0x02 | *N/A* | 0x02 | **0x3b** |
| **sys2gmem / clear_state** | 0x02 | *NOT SET* | 0x02 | *NOT SET* | 0x02 | **0x28f** |
| **clear_state_restore** | - | - | - | - | - | **0x3b** |
| **GMEM tile start** | - | - | - | - | - | **0x28f** |
| **GMEM tile end** | - | - | - | - | - | **0x3b** |
| **binning** | - | - | - | - | 0x00→0x02 | - |

**Key Finding:** KGSL does NOT set this register for A220 in GMEM paths. Mesa uses 0x28f during ops, 0x3b after.

---

## VGT_OUT_DEALLOC_CNTL (0x2317)

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **emit_restore / init** | 0x02 | 0x02 | 0x02 | 0x02 | 0x02 | **NOT SET** |
| **sys2gmem** | 0x02 | *NOT SET* | 0x02 | *NOT SET* | 0x02 | - |

**Key Finding:** KGSL sets this for both, but skips for A22X in GMEM. Mesa does NOT set for A22X at all.

---

## RB_BC_CONTROL

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **emit_restore / init** | *not shown* | *not shown* | *not shown* | *not shown* | **SET** (flags) | **NOT SET** |

**Key Finding:** Mesa only sets RB_BC_CONTROL for A20X, not A22X.

---

## RB_LRZ_VSC_CONTROL (0x2209) - A22X Only

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **clear_state** | N/A | 0x00 | N/A | 0x00 | N/A | **0x84** |
| **clear_state_restore** | N/A | 0x00 | N/A | 0x00 | N/A | **0x00** |
| **gmem2sys** | N/A | 0x00 | N/A | 0x00 | N/A | - |
| **sys2gmem** | N/A | 0x00 | N/A | 0x00 | N/A | - |

**Key Finding:** KGSL always uses 0x00. Mesa uses 0x84 during clear, 0x00 after.

---

## RB_DEPTHCONTROL

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **gmem2sys/sys2gmem** | 0x00 | **0x08** | 0x00 | **0x08** | 0x00 | **0x08** (Early Z) |

**Consistent:** All sources agree - A200=0x00, A220=0x08 (Early Z enable).

---

## PA_SC_VIZ_QUERY

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **sys2gmem** | 0x00 | *NOT SET* | 0x00 | *NOT SET* | SET (ID=16) | **NOT SET** |

**Consistent:** All sources skip PA_SC_VIZ_QUERY for A22X.

---

## SQ_PROGRAM_CNTL

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **gmem2sys** | 0x10010001 | **0x10018001** | 0x10010001 | **0x10018001** | - | - |
| **sys2gmem restore** | 0x10030002 | 0x10030002 | 0x10030002 | 0x10030002 | - | - |

**Consistent:** gmem2sys differs (bit 15 set for A22X), sys2gmem same.

---

## RBBM_PM_OVERRIDE2

| Context | Sony A200 | Sony A220 | TP A200 | TP A220 | Mesa A20X | Mesa A22X |
|---------|-----------|-----------|---------|---------|-----------|-----------|
| **final value** | 0x00 | **0x80** | 0x00 | **0x1a0** | - | - |

**Note:** TouchPad uses 0x1a0, Sony uses 0x80 for A220.

---

## Summary: Key Differences Between A200 and A220

| Register | A200/A20X | A220/A22X | Notes |
|----------|-----------|-----------|-------|
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x02 | 0x3b / 0x28f | KGSL skips for A22X; Mesa uses 0x28f during ops |
| VGT_OUT_DEALLOC_CNTL | 0x02 | NOT SET | Mesa does NOT set for A22X |
| RB_BC_CONTROL | SET | NOT SET | Mesa only sets for A20X |
| RB_LRZ_VSC_CONTROL | N/A | 0x00 / 0x84 | Mesa uses 0x84 during clear |
| RB_DEPTHCONTROL | 0x00 | 0x08 | Early Z enable for A22X |
| PA_SC_VIZ_QUERY | SET | NOT SET | A22X skips this |
| SQ_PROGRAM_CNTL (gmem2sys) | 0x10010001 | 0x10018001 | Bit 15 differs |
| RBBM_PM_OVERRIDE2 | 0x00 | 0x80-0x1a0 | Varies by platform |

---

## Analysis: KGSL vs Mesa Differences

### What KGSL Does That Mesa Doesn't:
1. **Different register naming:** A220 uses `PC_*` registers instead of `VGT_*` for some
2. **Complete skip of VGT registers:** KGSL completely skips VGT_VERTEX_REUSE and VGT_OUT_DEALLOC for A22X in GMEM paths

### What Mesa Does That KGSL Doesn't:
1. **Dynamic VGT_VERTEX_REUSE values:** Mesa switches between 0x28f (during operations) and 0x3b (after)
2. **RB_LRZ_VSC_CONTROL=0x84:** Mesa uses 0x84 during clear operations, KGSL uses 0x00

### Possible Issues with Mesa A22X Support:
1. Mesa sets VGT_VERTEX_REUSE but KGSL skips it entirely - maybe Mesa should too?
2. Mesa's 0x84 for RB_LRZ_VSC differs from KGSL's 0x00 - could cause issues?
3. Mesa might need platform-specific RBBM_PM_OVERRIDE2 (0x1a0 for TouchPad)

---

## Recommendations

### Option A: Match KGSL Behavior
For A22X, consider skipping VGT_VERTEX_REUSE_BLOCK_CNTL entirely in GMEM paths like KGSL does.

### Option B: Keep Mesa Values
Mesa's current values (0x28f/0x3b) appear to be well-tested upstream. The 0x84 for RB_LRZ_VSC during clear may be an optimization.

### Testing Required:
1. Try KGSL behavior: skip VGT_VERTEX_REUSE for A22X in GMEM
2. Try KGSL behavior: always use 0x00 for RB_LRZ_VSC_CONTROL
3. Add RBBM_PM_OVERRIDE2 = 0x1a0 for TouchPad

---

## Register Address Reference

| Register | Address |
|----------|---------|
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x2316 |
| VGT_OUT_DEALLOC_CNTL | 0x2317 |
| RB_LRZ_VSC_CONTROL (A220) | 0x2209 |
| RB_DEPTHCONTROL | 0x2200 |
| PA_SC_VIZ_QUERY | 0x2293 |
| SQ_PROGRAM_CNTL | 0x2180 |
| RBBM_PM_OVERRIDE1 | 0x039C |
| RBBM_PM_OVERRIDE2 | 0x039D |
