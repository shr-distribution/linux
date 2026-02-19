# KGSL vs Freedreno A22X (Leia) Detailed Comparison

## Summary

Since artifacts occur in ALL rendering modes (sysmem, flush, noscis, inorder), the issue is NOT:
- GMEM tiling/resolve
- Synchronization timing
- Scissor optimization
- Draw reordering

The issue IS in core rendering state - something that affects both sysmem and GMEM paths.

---

## 1. Kernel Initialization (VERIFIED CORRECT)

The mainline kernel correctly sets Leia-specific registers in `a2xx_gpu.c`:

```c
// MH client interface config for Leia (avoids GPU hang with 1KB boundary)
if (!adreno_is_a20x(adreno_gpu))
    gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG1, 0x00032f07);

// Power management override for Leia
if (!adreno_is_a20x(adreno_gpu))
    gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0x1a0);
else
    gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0);
```

These match KGSL values.

---

## 2. Leia-Specific KGSL Differences NOT in Freedreno

### 2.1 RB_DEPTHCONTROL for Leia

KGSL uses different depth control for Leia:

```c
// kgsl_drawctxt.c:1000-1003
if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
    *cmds++ = 8;        // EARLY_Z_ENABLE
else
    *cmds++ = 0;        // disable Z
```

**Freedreno behavior:** Sets EARLY_Z_ENABLE conditionally based on alpha test, not GPU type.

**Potential Impact:** Early Z might conflict with certain render states on Leia.

### 2.2 PA_SC_VIZ_QUERY for Leia

KGSL skips PA_SC_VIZ_QUERY for Leia:

```c
// kgsl_drawctxt.c:922-926
if (device->chip_id != KGSL_CHIPID_LEIA_REV470) {
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(REG_PA_SC_VIZ_QUERY);
    *cmds++ = 0x0;
}
```

**Freedreno behavior:** Sets PA_SC_VIZ_QUERY for A20X but not A22X (partial match).

### 2.3 SQ_PROGRAM_CNTL for Leia GMEM Operations

KGSL uses different shader setup for Leia:

```c
// kgsl_drawctxt.c:717-720
if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
    *cmds++ = 0x10018001;  // Leia-specific
else
    *cmds++ = 0x10010001;  // A20X
```

Breaking down `0x10018001`:
- VS_REGS = 1
- PS_REGS = 0x80 (high bit = 0 regs)
- VS_RESOURCE = true
- PS_EXPORT_MODE = 2

**Freedreno behavior:** Calculates dynamically based on shader. May not match Leia requirements.

### 2.4 Additional Leia-Only Registers

KGSL references these Leia-specific registers not found in freedreno:

| Register | Address | Purpose |
|----------|---------|---------|
| `REG_LEIA_GRAS_CONTROL` | 0x2210 | Graphics rasterizer control |
| `REG_LEIA_RB_LRZ_VSC_CONTROL` | 0x2209 | LRZ/VSC control |
| `REG_LEIA_SQ_RESOURCE_MANAGMENT` | 0x0d03 | Shader queue resource management |
| `REG_LEIA_SQ_PIX_IN_CNTL` | 0x0d0c | Pixel shader input control |
| `REG_LEIA_VSC_BIN_SIZE` | 0x0c01 | Visibility stream bin size |

**Freedreno behavior:** None of these Leia-specific registers are programmed.

---

## 3. Context Save/Restore Differences

KGSL saves/restores ~18KB of shader state per context. This includes:

```c
// kgsl_drawctxt.c:1128
cmd = reg_range(cmd, REG_SQ_PROGRAM_CNTL, REG_SQ_WRAPPING_1);
```

Registers saved include:
- `SQ_PROGRAM_CNTL` (0x2180)
- `SQ_CONTEXT_MISC` (0x2181)
- `SQ_INTERPOLATOR_CNTL` (0x2182)
- `SQ_WRAPPING_0` (0x2183)
- `SQ_WRAPPING_1` (0x2184)

**Freedreno behavior:** Does NOT cache shader state between submits. Each batch re-emits all state.

---

## 4. Shader Compiler (IR2) Potential Issues

The freedreno IR2 compiler for A2XX has known limitations:

### 4.1 Scheduler Failures

```c
// ir2.c:296
mesa_loge("ir2: no instruction available in sched_next, aborting");
```

When scheduler fails, shader is left EMPTY but still used.

### 4.2 Instruction Limit

```c
// ir2.c:411
mesa_loge("ir2: too many scheduled instructions");
```

Limit is 1024 scheduled instructions.

### 4.3 Register Limit

64 registers maximum. Complex shaders silently fail.

**Potential Impact:** Failed shader compilation produces garbage output but rendering continues.

---

## 5. Recommended Mesa Patches for Investigation

### 5.1 Add Leia-Specific Early Z Disable

```c
// fd2_emit.c - in fd2_emit_restore()
if (!is_a20x(ctx->screen)) {
    // Leia: force early Z disable like KGSL
    OUT_PKT3(ring, CP_SET_CONSTANT, 2);
    OUT_RING(ring, CP_REG(REG_A2XX_RB_DEPTHCONTROL));
    OUT_RING(ring, 8);  // EARLY_Z_ENABLE for Leia
}
```

### 5.2 Add Leia GRAS_CONTROL Register

```c
// fd2_emit.c - in fd2_emit_restore()
if (!is_a20x(ctx->screen)) {
    OUT_PKT3(ring, CP_SET_CONSTANT, 2);
    OUT_RING(ring, CP_REG(0x2210));  // REG_LEIA_GRAS_CONTROL
    OUT_RING(ring, 0x00000000);  // Need to find correct KGSL value
}
```

### 5.3 Add Shader Compilation Failure Detection

```c
// fd2_draw.c - before draw
if (!ctx->prog.fs->variant[0].info.sizedwords ||
    !ctx->prog.vs->variant[0].info.sizedwords) {
    mesa_loge("fd2: shader has 0 instructions, skipping draw");
    return false;
}
```

---

## 6. Test Recommendations

### 6.1 Force Disable Early Z

Create test patch to unconditionally disable early Z on A22X to see if artifacts change.

### 6.2 Dump Shader State

Add mesa debug to dump all shader registers being emitted to compare with KGSL captures.

### 6.3 Try KGSL GMEM Shader Values

Force freedreno to use exact KGSL SQ_PROGRAM_CNTL values (0x10018001) for comparison.

### 6.4 Check for Shader Compile Failures

Enable `FD_MESA_DEBUG=msgs` and check for any shader compile errors during glmark2.

---

## 7. GPU Page Fault Analysis

The user saw:
```
MMU_PAGE_FAULT: 02992475
GPUMMU fault: addr=0x02992475 idx=6546 pte=0x00000000
```

This shows:
- PTE index 6546 is **unmapped** (pte=0x00000000)
- Neighboring PTEs are valid (6545 and 6547 have 0x...003)

**Root Cause:** A buffer was unmapped while GPU still had pending commands using it. This is a race between:
1. Mesa freeing a buffer
2. GPU command queue still referencing it

This is separate from the visual artifacts but indicates synchronization issues.

---

## 8. Conclusion

Most likely causes of persistent artifacts (in order of probability):

1. **Missing Leia-specific register setup** - GRAS_CONTROL, SQ_RESOURCE_MANAGMENT not programmed
2. **Early Z handling difference** - KGSL uses EARLY_Z differently on Leia
3. **Shader compilation failures** - Some shaders fail but garbage is rendered
4. **SQ_PROGRAM_CNTL mismatch** - Freedreno calculates dynamically vs KGSL hardcoded values

Next step: Add mesa debug logging for shader register state and compare with KGSL values.
