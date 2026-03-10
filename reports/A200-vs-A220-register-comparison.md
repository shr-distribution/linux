# Adreno 200 (A20X) vs Adreno 220 (A22X/Leia) Register Comparison

## Sources Analyzed

| Source | Chip | URL |
|--------|------|-----|
| **KGSL A200** | A200/A205/A220/A225 | github.com/hsr0/android_kernel_sony_msm7x27a (cm-11.0) |
| **KGSL Leia** | A220 (TouchPad) | github.com/webos-internals/webos-linux-kernel (touchpad) |
| **Mesa freedreno** | A20X/A22X | gitlab.freedesktop.org/mesa/mesa |

---

## Key Difference: Register Naming

A220 (Leia) uses **PC_** (Primitive Coprocessor) prefixed registers instead of **VGT_** (Vertex Grouper/Tessellator):

| A200 (A20X) Register | A220 (Leia) Register |
|---------------------|---------------------|
| REG_VGT_MAX_VTX_INDX | REG_LEIA_PC_MAX_VTX_INDX / REG_A220_PC_MAX_VTX_INDX |
| REG_VGT_MIN_VTX_INDX | REG_LEIA_PC_MIN_VTX_INDX |
| REG_VGT_INDX_OFFSET | REG_LEIA_PC_INDX_OFFSET / REG_A220_PC_INDX_OFFSET |
| REG_VGT_VERTEX_REUSE_BLOCK_CNTL | REG_LEIA_PC_VERTEX_REUSE_BLOCK_CNTL / REG_A220_PC_VERTEX_REUSE_BLOCK_CNTL |

---

## Register Value Comparison (KGSL Drivers)

### VGT_VERTEX_REUSE_BLOCK_CNTL (0x2316)

| Source | A200 (A20X) | A220 (Leia/A22X) | Notes |
|--------|-------------|------------------|-------|
| **KGSL sys2gmem** | 0x02 | **NOT SET** | A22X skips this entirely! |
| **KGSL context save** | Uses REG_VGT_* | Uses REG_LEIA_PC_* | Different register! |

**KGSL Code (adreno_a2xx.c):**
```c
if (!adreno_is_a22x(adreno_dev)) {
    *cmds++ = CP_REG(REG_VGT_VERTEX_REUSE_BLOCK_CNTL);
    *cmds++ = 0x00000002;  /* mmVGT_VERTEX_REUSE_BLOCK_CNTL */
    *cmds++ = 0x00000002;  /* mmVGT_OUT_DEALLOC_CNTL */
}
```

**Key Finding:** KGSL does NOT write VGT_VERTEX_REUSE_BLOCK_CNTL for A22X in the sys2gmem path!

---

### VGT_OUT_DEALLOC_CNTL (0x2317)

| Source | A200 (A20X) | A220 (Leia/A22X) | Notes |
|--------|-------------|------------------|-------|
| **KGSL sys2gmem** | 0x02 | **NOT SET** | A22X skips this |

---

### SQ_PROGRAM_CNTL

| Source | A200 (A20X) | A220 (Leia/A22X) | Notes |
|--------|-------------|------------------|-------|
| **KGSL gmem2sys** | 0x10010001 | **0x10018001** | Different shader config |
| **KGSL sys2gmem** | 0x10030002 | 0x10030002 | Same |

---

### RB_DEPTHCONTROL

| Source | A200 (A20X) | A220 (Leia/A22X) | Notes |
|--------|-------------|------------------|-------|
| **KGSL gmem2sys/sys2gmem** | 0x00 (Z disabled) | **0x08** (Early Z enable) | A22X enables early Z |

**KGSL Code:**
```c
*cmds++ = CP_REG(REG_RB_DEPTHCONTROL);
if (adreno_is_a22x(adreno_dev))
    *cmds++ = 0x08;
else
    *cmds++ = 0x00;
```

---

### RB_LRZ_VSC_CONTROL (0x2209) - A22X ONLY

| Source | A200 (A20X) | A220 (Leia/A22X) | Notes |
|--------|-------------|------------------|-------|
| **KGSL gmem2sys** | N/A | **0x00** | Leia-specific register |
| **KGSL sys2gmem** | N/A | **0x00** | Leia-specific register |

**KGSL Code:**
```c
if (adreno_is_a22x(adreno_dev)) {
    *cmds++ = CP_REG(REG_A220_RB_LRZ_VSC_CONTROL);
    *cmds++ = 0x0000000;
}
```

---

### PA_SC_VIZ_QUERY

| Source | A200 (A20X) | A220 (Leia/A22X) | Notes |
|--------|-------------|------------------|-------|
| **KGSL sys2gmem** | 0x00 | **NOT SET** | A22X skips this |

**KGSL Code:**
```c
if (!adreno_is_a22x(adreno_dev)) {
    *cmds++ = PM4_REG(REG_PA_SC_VIZ_QUERY);
    *cmds++ = 0x0;
}
```

---

### Draw Command Format

| Chip | Draw Command | Notes |
|------|--------------|-------|
| **A200 (A20X)** | `CP_DRAW_INDX` with 2 params | Standard format |
| **A220 (Leia)** | `3D_DRAW_INDX_2` (0xc0043600) with 3 params | Different packet type! |

**KGSL Code (Leia path):**
```c
if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
    *cmds++ = 0xc0043600; /* packet 3 3D_DRAW_INDX_2 */
    *cmds++ = 0x0;
    *cmds++ = 0x00004046; /* tristrip */
    *cmds++ = 0x00000004; /* NUM_INDICES */
    *cmds++ = 0x00010000; /* index: 0x00, 0x01 */
    *cmds++ = 0x00030002; /* index: 0x02, 0x03 */
}
```

---

### RBBM_PM_OVERRIDE Registers

| Register | A200 (A20X) | A220 (Leia/A22X) | Notes |
|----------|-------------|------------------|-------|
| **RBBM_PM_OVERRIDE1** | 0xfffffffe → 0x00 | 0xfffffffe → 0x00 | Same |
| **RBBM_PM_OVERRIDE2** | 0xffffffff → 0x00 | 0xffffffff → **0x1a0** | Different final value! |

---

### MH_CLNT_INTF_CTRL Registers

| Register | A200 (A20X) | A220 (Leia/A22X) | Notes |
|----------|-------------|------------------|-------|
| **MH_CLNT_INTF_CTRL_CONFIG1** | 0x00030f27 | **0x00032f07** | Different |
| **MH_CLNT_INTF_CTRL_CONFIG2** | 0x00472747 | 0x00472747 | Same |

---

### GMEM Configuration

| Parameter | A200 (A20X) | A220 (Leia/A22X) | Notes |
|-----------|-------------|------------------|-------|
| **GMEM Size** | 256 KB | **512 KB** | Leia has more GMEM |

---

## Mesa Freedreno Values

### VGT_VERTEX_REUSE_BLOCK_CNTL

| Context | A20X (Mesa) | A22X (Mesa) | Notes |
|---------|-------------|-------------|-------|
| **fd2_emit_restore** (normal) | 0x02 | **0x3b** | Normal rendering state |
| **clear_state** (during clear) | N/A | **0x28f** | Temporary for clear |
| **clear_state_restore** (after) | N/A | **0x3b** | Restore after clear |
| **prepare_tile_fini_ib** (GMEM start) | N/A | **0x28f** | GMEM operations |
| **prepare_tile_fini_ib** (GMEM end) | N/A | **0x3b** | After GMEM |

### RB_LRZ_VSC_CONTROL

| Context | A20X (Mesa) | A22X (Mesa) | Notes |
|---------|-------------|-------------|-------|
| **clear_state** | N/A | **0x84** | During clear |
| **clear_state_restore** | N/A | **0x00** | After clear |

### RB_BC_CONTROL

| Context | A20X (Mesa) | A22X (Mesa) | Notes |
|---------|-------------|-------------|-------|
| **fd2_emit_restore** | Set with flags | **NOT SET** | Mesa doesn't set for A22X |

### VGT_OUT_DEALLOC_CNTL

| Context | A20X (Mesa) | A22X (Mesa) | Notes |
|---------|-------------|-------------|-------|
| **fd2_emit_restore** | 0x02 | **NOT SET** | Mesa doesn't set for A22X |

---

## Summary: Key A220 Differences

1. **Different register namespace**: Uses `PC_*` instead of `VGT_*` for some registers
2. **VGT_VERTEX_REUSE_BLOCK_CNTL**:
   - KGSL doesn't set it for A22X in GMEM paths
   - Mesa uses 0x3b normally, 0x28f during clear/GMEM
3. **VGT_OUT_DEALLOC_CNTL**: Not set for A22X
4. **RB_DEPTHCONTROL**: A22X uses 0x08 (early Z), A20X uses 0x00
5. **RB_LRZ_VSC_CONTROL**: A22X-only register (0x84 during clear, 0x00 after)
6. **RB_BC_CONTROL**: NOT set for A22X in Mesa
7. **SQ_PROGRAM_CNTL**: A22X uses 0x10018001, A20X uses 0x10010001
8. **Draw packet format**: A22X uses different draw command (3D_DRAW_INDX_2)
9. **GMEM size**: A22X has 512KB vs 256KB for A20X
10. **RBBM_PM_OVERRIDE2**: A22X final value is 0x1a0, A20X is 0x00

---

## Recommendations for Our Mesa Patches

### Issues Found in Our Patches:

| Register | Our Value | Correct Value | Issue |
|----------|-----------|---------------|-------|
| VGT_VERTEX_REUSE (clear) | 0x3b | **0x28f** | Wrong during clear ops |
| VGT_VERTEX_REUSE (GMEM) | 0x3b | **0x28f** | Wrong during GMEM ops |
| RB_LRZ_VSC (clear) | 0x00 | **0x84** | Wrong during clear |
| RB_BC_CONTROL | Set | **NOT SET** | Unnecessary for A22X |
| VGT_OUT_DEALLOC | Set | **NOT SET** | Unnecessary for A22X |

### Correct Values for A22X:

```c
// Normal rendering (fd2_emit_restore):
VGT_VERTEX_REUSE_BLOCK_CNTL = 0x3b
// Do NOT set RB_BC_CONTROL for A22X
// Do NOT set VGT_OUT_DEALLOC_CNTL for A22X

// During clear operations (clear_state):
RB_LRZ_VSC_CONTROL = 0x84
VGT_VERTEX_REUSE_BLOCK_CNTL = 0x28f

// After clear operations (clear_state_restore):
RB_LRZ_VSC_CONTROL = 0x00
VGT_VERTEX_REUSE_BLOCK_CNTL = 0x3b

// During GMEM tile operations:
VGT_VERTEX_REUSE_BLOCK_CNTL = 0x28f (at start)
VGT_VERTEX_REUSE_BLOCK_CNTL = 0x3b (at end)
```
