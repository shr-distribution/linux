# Adreno 200/220 GPU Register Comparison

**Generated:** 2026-02-11
**Purpose:** Compare register initialization between Sony kernel, webOS TouchPad kernel, mainline kernel, and Mesa freedreno

## Executive Summary

This document compares GPU register initialization across 5 different driver implementations:

| Source | GPU Target | Type |
|--------|------------|------|
| **Sony kernel** (msm7x27a) | A200/A203/A205/A220/A225 | Proprietary KGSL |
| **webOS TouchPad kernel** | A220 (Leia) | Proprietary KGSL |
| **Mainline kernel** | A200/A220/A225 | Open source DRM/MSM |
| **Mesa freedreno** | A200/A220/A225 | Open source Mesa |
| **Mesa freedreno + patches** | A220 (TouchPad) | Patched Mesa |

### Key Findings

| Register | Sony KGSL | webOS KGSL | Mainline | freedreno | Issue |
|----------|-----------|------------|----------|-----------|-------|
| **SQ_GPR_MANAGEMENT (0x0D00)** | **0x00040400** | **0x00040400** | NOT SET | **0x00040400** (patch 0012) | **ROOT CAUSE of faceted shading** |
| RBBM_PM_OVERRIDE2 (0x039D) | 0x80 (A22X) | 0x1a0 (Leia) | 0x1a0 (A22X) | 0xfff (A22X) | Clock gating |
| A220_RB_LRZ_VSC_CONTROL (0x2209) | 0x00000000 | Context save | 0x00000000 | 0x00000000 | Matches |
| A220_GRAS_CONTROL (0x2210) | 0x00000000 | Context save | 0x00000000 | 0x00000000 | Matches |
| SQ_INTERPOLATOR_CNTL (0x2182) | 0xffffffff | 0xffffffff | NOT SET | 0xffffffff | Mesa correct |
| TP0_CHICKEN (0x0E1E) | 0x00000000 | 0x00000000 | NOT SET | 0x00000002 | Minor diff |
| SQ_FLOW_CONTROL (0x0D01) | 0x18000000 (A225) | N/A | 0x18000000 (A225) | N/A | A225 only |

---

## GPU Variant Identification

### A200 Series (Yamato)
- **A200**: Original Adreno GPU
- **A203**: Lower power variant
- **A205**: Slightly enhanced A200

### A220 Series (Leia)
- **A220**: Enhanced A200 with binning support, larger GMEM
- **A225**: A220 with extended instruction store (used in MSM8960)

### TouchPad Specifics
- **GPU**: Adreno 220 (code name "Leia")
- **Chip ID**: KGSL_CHIPID_LEIA_REV470
- **GMEM**: 256KB (vs 128KB on A200)
- **Binning**: VSC (Visibility Stream Controller) support
- **Clock**: 320 MHz core, 200 MHz MDP

---

## 1. Shader Unit (SQ) Registers

### Critical: SQ_GPR_MANAGEMENT (0x0D00)

**ROOT CAUSE OF INTERMITTENT FACETED SHADING**

```
Register Layout:
  [0]     REG_DYNAMIC   - 0=static, 1=dynamic allocation
  [4:11]  REG_SIZE_PIX  - GPRs for pixel shader (0-255)
  [12:19] REG_SIZE_VTX  - GPRs for vertex shader (0-255)

Value 0x00040400 = 64 GPRs each for VS and PS
```

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x00040400 | Set in build_regrestore_cmds() |
| webOS kernel | 0x00040400 | Set at line 1166 in kgsl_drawctxt.c |
| Mainline kernel | NOT SET | Missing initialization |
| freedreno | 0x00040400 | **NEW: patch 0012** |

**Impact when uninitialized**: Random power-on value causes GPR starvation, leading to:
- Varying interpolation failures
- Intermittent faceted shading
- Same shader produces different results across boots

### SQ_INTERPOLATOR_CNTL (0x2182)

Controls smooth/flat shading for varyings.

```
Bit Layout:
  [0:15]  PARAM_SHADE     - Bit per varying (1=smooth, 0=flat)
  [16:31] SAMPLING_PATTERN - Texture sampling pattern
```

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0xffffffff | All smooth interpolation |
| webOS kernel | 0xffffffff | All smooth (was 0x0000ffff, changed) |
| Mainline kernel | NOT SET | Relies on Mesa |
| freedreno | 0xffffffff | Correct setting |

### SQ_INST_STORE_MANAGMENT (0x0D02)

Instruction store allocation between VS and PS.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | N/A | |
| webOS kernel | N/A | |
| Mainline kernel | N/A | |
| freedreno | 0x00000180 | BASE_PIX=0x180 |

### SQ_FLOW_CONTROL (0x0D01)

**A225 only** - Enables large instruction store.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x18000000 | A225 only |
| webOS kernel | N/A | Not A225 |
| Mainline kernel | 0x18000000 | A225 only |
| freedreno | N/A | |

### SQ_PROGRAM_CNTL (0x2180)

Per-shader program configuration.

```
Bit Layout:
  [0:7]   VS_REGS        - Registers used by VS
  [8:15]  PS_REGS        - Registers used by PS
  [16]    VS_RESOURCE    - VS uses resources
  [17]    PS_RESOURCE    - PS uses resources
  [20:23] VS_EXPORT_COUNT - Varyings exported
  [24:26] VS_EXPORT_MODE
  [27:30] PS_EXPORT_MODE
```

| Source | A200 Value | A220 Value |
|--------|------------|------------|
| Sony kernel | 0x10018001 | 0x10010001 |
| webOS kernel | Per-shader | Per-shader |
| freedreno | Per-shader | Per-shader |

---

## 2. RBBM (Ring Buffer Block Manager) Registers

### RBBM_PM_OVERRIDE1 (0x039C)

Power management override - keeps clocks running.

| Source | Init Value | Final Value |
|--------|------------|-------------|
| Sony kernel | 0xfffffffe | 0x0 or 0x200 |
| webOS kernel | 0xffffffff | 0x0 |
| Mainline kernel | 0xfffffffe | 0x0 |
| freedreno | 0xffffffff | N/A |

### RBBM_PM_OVERRIDE2 (0x039D)

**A220/A225 specific** - Additional clock gating control.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x80 (A22X) | Different from TouchPad |
| webOS kernel | **0x1a0** | TouchPad specific value |
| Mainline kernel | 0x1a0 (A22X) | Matches webOS |
| freedreno | 0xfff | Keeps all clocks on |

**TouchPad-specific value 0x1a0** may be optimized for the APQ8060 SoC.

### RBBM_SOFT_RESET (0x003C)

Software reset control.

| Source | Value | Notes |
|--------|-------|-------|
| All sources | 0xFFFFFFFF→0x0 | Full reset, then clear |

### RBBM_CNTL (0x003B)

RBBM configuration.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x0000ffff (A20X) / 0x00004442 (A22X) | Variant-specific |
| webOS kernel | 0x00004442 | A22X value |
| Mainline kernel | 0x00004442 | Matches |
| freedreno | N/A | Kernel sets |

### RBBM_DEBUG (0x039B)

Debug configuration.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | N/A | |
| webOS kernel | N/A | |
| Mainline kernel | 0x00080000 | Enable debug |
| freedreno | N/A | |

---

## 3. Render Backend (RB) Registers

### A220_RB_LRZ_VSC_CONTROL (0x2209)

**A220+ only** - Low Resolution Z and Visibility Stream Control.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x00000000 | Init in restore |
| webOS kernel | Context save/restore | Dynamic |
| Mainline kernel | 0x00000000 | Explicit init |
| freedreno | 0x00000000 | Matches (patch 0011) |

### A220_GRAS_CONTROL (0x2210)

**A220+ only** - GRAS unit control.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x00000000 | Init |
| webOS kernel | Context save/restore | Dynamic |
| Mainline kernel | 0x00000000 | Explicit init |
| freedreno | 0x00000000 | Matches (patch 0011) |

### RB_MODECONTROL (0x2208)

GMEM resolve mode control.

| Source | A200 Value | A220 Value |
|--------|------------|------------|
| Sony kernel | 0x6 | 0x4 |
| webOS kernel | 0x6 (Yamato) / 0x4 (Leia) | Per-variant |
| Mainline kernel | N/A | |
| freedreno | Per-op | Per-op |

### RB_EDRAM_INFO (0x0F02)

GMEM (embedded DRAM) configuration.

| Source | Value | Notes |
|--------|-------|-------|
| All sources | Computed | Based on GMEM size |

A220 has 256KB GMEM (value = 5), A200 has 128KB (value = 4).

### RB_BC_CONTROL (0x0F01)

**A200 only** - Blending/Compression control.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | A20X only | Not for A220 |
| webOS kernel | N/A | A220 device |
| freedreno | A20X only | Conditional |

### RB_SAMPLE_POS (0x220A)

Multisampling sample positions.

| Source | Value | Notes |
|--------|-------|-------|
| freedreno | 0x88888888 | Center sampling |
| Others | N/A | |

---

## 4. Texture Processor (TP/TC) Registers

### TP0_CHICKEN (0x0E1E)

Texture processor workarounds.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x00000000 | |
| webOS kernel | 0x00000000 | Context init |
| Mainline kernel | NOT SET | |
| freedreno | 0x00000002 | Different! |

**Note**: freedreno sets a different value which may enable/disable specific features.

### TC_CNTL_STATUS (0x0E00)

Texture cache control.

| Source | Value | Notes |
|--------|-------|-------|
| freedreno | L2_INVALIDATE | Per-draw cache invalidate |
| Others | N/A | |

---

## 5. Command Processor (CP) Registers

### CP_QUEUE_THRESHOLDS (0x01D5)

Command queue thresholds.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x000C0804 | Standard |
| webOS kernel | 0x000C0804 | Matches |
| Mainline kernel | 0x000C0804 | Matches |
| freedreno | N/A | Kernel sets |

### CP_DEBUG (0x01FC)

Command processor debug settings.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x02000000 | MIU_128BIT_WRITE |
| webOS kernel | N/A | |
| Mainline kernel | MIU_128BIT_WRITE | Matches |
| freedreno | N/A | Kernel sets |

---

## 6. Memory Hub/MMU (MH) Registers

### MH_MMU_CONFIG (0x0040)

MMU configuration with client behaviors.

| Source | Value | Notes |
|--------|-------|-------|
| All sources | Complex | Client behavior flags |

All sources use BEH_TRAN_RNG for translation range behavior.

### MH_ARBITER_CONFIG (0x0A40)

Memory arbiter configuration.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | Complex | Full config |
| Mainline kernel | Complex | Matches Sony |
| freedreno | N/A | Kernel sets |

### MH_CLNT_INTF_CTRL_CONFIG1 (0x0A54)

**A22X only** - Memory interface control.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | Set for A22X | |
| webOS kernel | N/A | |
| Mainline kernel | 0x00032f07 | A22X only |
| freedreno | N/A | Kernel sets |

---

## 7. VSC (Visibility Stream Controller) - A220+ Only

The VSC provides hardware binning support on A220/A225.

### A220_VSC_BIN_SIZE (0x0C01)

Bin tile size configuration.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | Context | Per-render target |
| webOS kernel | 0x00000000 | Init to zero |
| Mainline kernel | N/A | |
| freedreno | N/A | Binning disabled |

**Note**: freedreno disables hardware binning on A22X (`use_hw_binning()` returns false).

### VSC_PIPE_* (0x0C06-0x0C1D)

Visibility stream pipe configuration (8 pipes).

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | Context save/restore | Dynamic |
| webOS kernel | All 0x00000000 | Initialized |
| Mainline kernel | N/A | |
| freedreno | N/A | Binning disabled |

---

## 8. Primitive Assembly (PA) Registers

### PA_SC_VIZ_QUERY (0x2293)

Visibility query control.

| Source | A200 Value | A220 Value |
|--------|------------|------------|
| KGSL | ID(16) | 0x0 |
| freedreno | ID(16) | ID(16) (A20X only) |

### VGT_VERTEX_REUSE_BLOCK_CNTL (0x2316)

Vertex reuse optimization.

| Source | A200 Value | A220 Value |
|--------|------------|------------|
| Sony kernel | 0x00000002 | 0x0000003b |
| webOS kernel | 0x02 | 0x3b |
| Mainline kernel | N/A | |
| freedreno | 0x02 | 0x3b |

### VGT_OUT_DEALLOC_CNTL (0x2317)

**A200 only** - Output deallocation control.

| Source | Value | Notes |
|--------|-------|-------|
| Sony kernel | 0x02 | A20X only |
| freedreno | 0x02 | A20X only |

---

## 9. TouchPad-Specific Differences

### Registers Unique to webOS TouchPad Kernel

| Register | Value | Purpose |
|----------|-------|---------|
| RBBM_PM_OVERRIDE2 | **0x1a0** | APQ8060-specific clock gating |
| SQ_GPR_MANAGEMENT | 0x00040400 | GPR allocation (also in Sony) |
| VSC pipes | All zeroed | Explicit initialization |

### Registers with TouchPad-Specific Values

1. **RBBM_PM_OVERRIDE2 = 0x1a0**
   - Sony kernel uses 0x80 for generic A22X
   - TouchPad uses 0x1a0 for APQ8060
   - Mainline matches TouchPad value

2. **KGSL_CHIPID_LEIA_REV470**
   - TouchPad-specific chip revision
   - Triggers additional initialization (VSC pipes)

### Leia-Specific Debug Registers

webOS kernel defines additional Leia debug registers not in Sony kernel:

| Register | Address | Purpose |
|----------|---------|---------|
| REG_LEIA_SQ_RESOURCE_MANAGMENT | 0x0D03 | Resource management |
| REG_LEIA_SQ_PIX_IN_CNTL | 0x0D0C | Pixel input control |
| REG_LEIA_CP_ME_STATUS | 0x01F7 | ME status |

---

## 10. Implementation Differences Summary

### Sony KGSL vs webOS KGSL

| Aspect | Sony | webOS |
|--------|------|-------|
| A22X PM_OVERRIDE2 | 0x80 | 0x1a0 |
| VSC pipe init | Not explicit | All zeroed |
| Chip detection | Generic | LEIA_REV470 |

### KGSL vs Mainline Kernel

| Aspect | KGSL | Mainline |
|--------|------|----------|
| SQ_GPR_MANAGEMENT | **Set to 0x00040400** | **NOT SET** |
| SQ_INTERPOLATOR_CNTL | Set to 0xffffffff | Not set (Mesa does) |
| Context shadowing | 18KB shadow memory | No shadowing |
| VSC binning | Supported | Supported (A22X) |

### Mainline vs freedreno (Mesa)

| Aspect | Mainline Kernel | freedreno (Mesa) |
|--------|-----------------|------------------|
| SQ_GPR_MANAGEMENT | NOT SET | **0x00040400** (patch 0012) |
| A220_RB_LRZ_VSC_CONTROL | 0x00000000 | 0x00000000 (patch 0011) |
| A220_GRAS_CONTROL | 0x00000000 | 0x00000000 (patch 0011) |
| RBBM_PM_OVERRIDE2 | 0x1a0 | 0xfff (keep clocks on) |
| TP0_CHICKEN | NOT SET | 0x00000002 |
| Hardware binning | Supported | Disabled for A22X |

---

## 11. Patches Applied to freedreno

| Patch | Register | Value | Effect |
|-------|----------|-------|--------|
| 0011 | A220_RB_LRZ_VSC_CONTROL | 0x00000000 | Initialize LRZ |
| 0011 | A220_GRAS_CONTROL | 0x00000000 | Initialize GRAS |
| 0011 | RBBM_PM_OVERRIDE2 | 0x00000fff | Keep clocks enabled |
| **0012** | **SQ_GPR_MANAGEMENT** | **0x00040400** | **FIX: GPR allocation** |

---

## 12. Potential Areas for Further Investigation

### Missing Initialization in Mainline/freedreno

1. **SQ_GPR_MANAGEMENT** - **FIXED by patch 0012**
2. **TP0_CHICKEN** - freedreno uses 0x2, KGSL uses 0x0 - investigate
3. **VSC binning** - Disabled in freedreno, may improve performance if enabled

### TouchPad-Specific Tuning

1. **RBBM_PM_OVERRIDE2 = 0x1a0** - Verify this is optimal for APQ8060
2. **GMEM size** - Verify 256KB is correctly detected
3. **Clock frequencies** - Verify 320 MHz core clock

### Binning Support

freedreno disables binning on A22X but KGSL supports it:
- `use_hw_binning()` returns false for non-A20X
- VSC registers are not initialized
- May be performance opportunity

---

## 13. Register Address Reference

### Complete A2XX Register Map

| Address | Name | A200 | A220 | Notes |
|---------|------|------|------|-------|
| 0x003B | RBBM_CNTL | Y | Y | |
| 0x003C | RBBM_SOFT_RESET | Y | Y | |
| 0x0040 | MH_MMU_CONFIG | Y | Y | |
| 0x00C0 | CP_PFP_UCODE_ADDR | Y | Y | |
| 0x00C1 | CP_PFP_UCODE_DATA | Y | Y | |
| 0x01C0 | CP_RB_BASE | Y | Y | |
| 0x01C1 | CP_RB_CNTL | Y | Y | |
| 0x01D5 | CP_QUEUE_THRESHOLDS | Y | Y | |
| 0x01F2 | CP_INT_CNTL | Y | Y | |
| 0x01F6 | CP_ME_CNTL | Y | Y | |
| 0x01F8 | CP_ME_RAM_WADDR | Y | Y | |
| 0x01FA | CP_ME_RAM_DATA | Y | Y | |
| 0x01FC | CP_DEBUG | Y | Y | |
| 0x039B | RBBM_DEBUG | Y | Y | |
| 0x039C | RBBM_PM_OVERRIDE1 | Y | Y | |
| 0x039D | RBBM_PM_OVERRIDE2 | Y | Y | Different values |
| 0x03B4 | RBBM_INT_CNTL | Y | Y | |
| 0x05D0 | RBBM_STATUS | Y | Y | Read-only |
| 0x0A40 | MH_ARBITER_CONFIG | Y | Y | |
| 0x0A42 | MH_INTERRUPT_MASK | Y | Y | |
| 0x0A54 | MH_CLNT_INTF_CTRL_CONFIG1 | N | Y | A22X only |
| 0x0C01 | A220_VSC_BIN_SIZE | N | Y | A22X only |
| 0x0C06-0x0C1D | VSC_PIPE_* | N | Y | A22X only |
| 0x0D00 | **SQ_GPR_MANAGEMENT** | Y | Y | **CRITICAL** |
| 0x0D01 | SQ_FLOW_CONTROL | N | Y | A225 only |
| 0x0D02 | SQ_INST_STORE_MANAGMENT | Y | Y | |
| 0x0D03 | SQ_RESOURCE_MANAGMENT | N | Y | Leia only |
| 0x0D0C | SQ_PIX_IN_CNTL | N | Y | Leia only |
| 0x0D34 | SQ_INT_CNTL | Y | Y | |
| 0x0E00 | TC_CNTL_STATUS | Y | Y | |
| 0x0E1E | TP0_CHICKEN | Y | Y | |
| 0x0F01 | RB_BC_CONTROL | Y | N | A20X only |
| 0x0F02 | RB_EDRAM_INFO | Y | Y | |
| 0x2000 | RB_SURFACE_INFO | Y | Y | |
| 0x2001 | RB_COLOR_INFO | Y | Y | |
| 0x2002 | RB_DEPTH_INFO | Y | Y | |
| 0x2100 | VGT_MAX_VTX_INDX | Y | Y | |
| 0x2102 | VGT_INDX_OFFSET | Y | Y | |
| 0x2104 | RB_COLOR_MASK | Y | Y | |
| 0x2180 | SQ_PROGRAM_CNTL | Y | Y | Per-shader |
| 0x2181 | SQ_CONTEXT_MISC | Y | Y | |
| 0x2182 | SQ_INTERPOLATOR_CNTL | Y | Y | |
| 0x2183 | SQ_WRAPPING_0 | Y | Y | |
| 0x2184 | SQ_WRAPPING_1 | Y | Y | |
| 0x21F6 | SQ_PS_PROGRAM | Y | Y | Per-shader |
| 0x21F7 | SQ_VS_PROGRAM | Y | Y | Per-shader |
| 0x2200 | RB_DEPTHCONTROL | Y | Y | |
| 0x2201 | RB_BLEND_CONTROL | Y | Y | |
| 0x2202 | RB_COLORCONTROL | Y | Y | |
| 0x2204 | PA_CL_CLIP_CNTL | Y | Y | |
| 0x2205 | PA_SU_SC_MODE_CNTL | Y | Y | |
| 0x2206 | PA_CL_VTE_CNTL | Y | Y | |
| 0x2208 | RB_MODECONTROL | Y | Y | Different values |
| 0x2209 | A220_RB_LRZ_VSC_CONTROL | N | Y | A22X only |
| 0x220A | RB_SAMPLE_POS | Y | Y | |
| 0x2210 | A220_GRAS_CONTROL | N | Y | A22X only |
| 0x2280 | PA_SU_POINT_SIZE | Y | Y | |
| 0x2293 | PA_SC_VIZ_QUERY | Y | Y | Different values |
| 0x2300 | PA_SC_LINE_CNTL | Y | Y | |
| 0x2301 | PA_SC_AA_CONFIG | Y | Y | |
| 0x2307 | SQ_VS_CONST | Y | Y | |
| 0x2308 | SQ_PS_CONST | Y | Y | |
| 0x2312 | PA_SC_AA_MASK | Y | Y | |
| 0x2316 | VGT_VERTEX_REUSE_BLOCK_CNTL | Y | Y | Different values |
| 0x2317 | VGT_OUT_DEALLOC_CNTL | Y | N | A20X only |
| 0x2318 | RB_COPY_CONTROL | Y | Y | |
| 0x231D | RB_DEPTH_CLEAR | Y | Y | |
| 0x2326 | RB_COLOR_DEST_MASK | Y | Y | |

---

## 14. Conclusion

The primary difference causing issues was **SQ_GPR_MANAGEMENT** not being initialized by the mainline kernel or freedreno (before patches). This has been fixed with patch 0012.

Key differences between TouchPad (webOS) and generic A220 implementations:
1. **RBBM_PM_OVERRIDE2 = 0x1a0** - APQ8060-specific clock gating
2. **KGSL_CHIPID_LEIA_REV470** - Specific chip revision detection
3. **VSC pipe initialization** - All pipes zeroed on TouchPad

The freedreno driver is now functionally equivalent to KGSL for the critical registers after applying patches 0011 and 0012.
