# Adreno 200/220 GPU Register Comparison

**Generated:** 2026-02-11
**Purpose:** Compare register initialization between Sony, HTC, webOS kernels, mainline kernel, and Mesa freedreno

## Executive Summary

This document compares GPU register initialization across 6 different driver implementations:

| Source | GPU Target | SoC | Type |
|--------|------------|-----|------|
| **Sony kernel** (msm7x27a) | A200/A203/A205/A220/A225 | MSM7x27A | Proprietary KGSL |
| **HTC kernel** (msm8660) | A220/A225 | MSM8660 | Proprietary KGSL |
| **webOS TouchPad kernel** | A220 (Leia) | APQ8060 | Proprietary KGSL |
| **Mainline kernel** | A200/A220/A225 | All | Open source DRM/MSM |
| **Mesa freedreno** | A200/A220/A225 | All | Open source Mesa |
| **Mesa freedreno + patches** | A220 (TouchPad) | APQ8060 | Patched Mesa |

### SoC Relationship
- **APQ8060** = Application processor only (TouchPad)
- **MSM8660** = APQ8060 + Modem (HTC devices like Sensation, EVO 3D)
- Both use identical Adreno 220 GPU silicon

### Key Findings

| Register | Sony KGSL | HTC KGSL | webOS KGSL | Mainline | freedreno | Issue |
|----------|-----------|----------|------------|----------|-----------|-------|
| **SQ_GPR_MANAGEMENT (0x0D00)** | **0x00040400** | **0x00040400** | **0x00040400** | NOT SET | **0x00040400** (patch 0012) | **ROOT CAUSE of faceted shading** |
| RBBM_PM_OVERRIDE2 (0x039D) | 0x80 (A22X) | **0x80** (A22X) | **0x1a0** (Leia) | 0x1a0 (A22X) | 0xfff (A22X) | **TouchPad uses unique value!** |
| A220_RB_LRZ_VSC_CONTROL (0x2209) | 0x00000000 | 0x00000000 | Context save | 0x00000000 | 0x00000000 | Matches |
| A220_GRAS_CONTROL (0x2210) | 0x00000000 | 0x00000000 | Context save | 0x00000000 | 0x00000000 | Matches |
| SQ_INTERPOLATOR_CNTL (0x2182) | 0xffffffff | 0xffffffff | 0xffffffff | NOT SET | 0xffffffff | Mesa correct |
| TP0_CHICKEN (0x0E1E) | 0x00000000 | 0x00000000 | 0x00000000 | NOT SET | 0x00000002 | Minor diff |
| SQ_FLOW_CONTROL (0x0D01) | 0x18000000 (A225) | 0x18000000 (A225) | N/A | 0x18000000 (A225) | N/A | A225 only |

### RBBM_PM_OVERRIDE2 Analysis

**Critical Finding**: TouchPad uses a unique value (0x1a0) not found in other A220 implementations:

| Source | Value | Binary | Notes |
|--------|-------|--------|-------|
| Sony (MSM7x27A) | 0x80 | 0000 0000 1000 0000 | Generic A22X |
| HTC (MSM8660) | 0x80 | 0000 0000 1000 0000 | Same as Sony |
| webOS (APQ8060) | **0x1a0** | 0000 0001 1010 0000 | **Palm-specific tuning** |
| Mainline | 0x1a0 | 0000 0001 1010 0000 | Copied from webOS |
| freedreno | 0xfff | 0000 1111 1111 1111 | All clocks enabled |

The 0x1a0 value may be Palm/HP-specific optimization for the APQ8060 or power tuning for tablet use case.

---

## GPU Variant Identification

### A200 Series (Yamato)
- **A200**: Original Adreno GPU
- **A203**: Lower power variant
- **A205**: Slightly enhanced A200

### A220 Series (Leia)
- **A220**: Enhanced A200 with binning support, larger GMEM
- **A225**: A220 with extended instruction store (used in MSM8960)

### SoC Comparison: APQ8060 vs MSM8660

| Aspect | APQ8060 (TouchPad) | MSM8660 (HTC) |
|--------|-------------------|---------------|
| CPU | Dual Scorpion 1.5GHz | Dual Scorpion 1.2-1.5GHz |
| GPU | Adreno 220 | Adreno 220 |
| Modem | **None** | Integrated |
| Manufacturer | HP/Palm | HTC |
| Devices | TouchPad | Sensation, EVO 3D, Vivid |
| RBBM_PM_OVERRIDE2 | **0x1a0** | 0x80 |

Both SoCs share identical GPU silicon, but the TouchPad kernel uses different power management settings.

### TouchPad Specifics
- **GPU**: Adreno 220 (code name "Leia")
- **Chip ID**: KGSL_CHIPID_LEIA_REV470
- **GMEM**: 256KB (vs 128KB on A200)
- **Binning**: VSC (Visibility Stream Controller) support
- **Clock**: 320 MHz core, 200 MHz MDP
- **Unique**: RBBM_PM_OVERRIDE2 = 0x1a0 (not found in other A220 devices)

### HTC MSM8660 Devices
- **GPU**: Adreno 220 (same as TouchPad)
- **GMEM**: 256KB
- **Clock**: Similar to TouchPad
- **Standard**: Uses generic A22X value (0x80) for RBBM_PM_OVERRIDE2

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

| Register | Value | Sony | HTC | Purpose |
|----------|-------|------|-----|---------|
| RBBM_PM_OVERRIDE2 | **0x1a0** | 0x80 | 0x80 | **Palm-specific clock gating** |
| SQ_GPR_MANAGEMENT | 0x00040400 | Same | Same | GPR allocation |
| VSC pipes | All zeroed | Not explicit | Not explicit | Explicit initialization |

### RBBM_PM_OVERRIDE2 Deep Dive

**This is the only register where TouchPad differs from all other A220 implementations.**

| Bit | 0x80 (Sony/HTC) | 0x1a0 (TouchPad) | Difference |
|-----|-----------------|------------------|------------|
| 7 | 1 | 1 | Same |
| 8 | 0 | **1** | TouchPad enables |
| 5 | 0 | **1** | TouchPad enables |

Possible explanations for 0x1a0:
1. **Tablet power optimization** - Different thermal/power profile vs phones
2. **APQ8060 vs MSM8660** - No modem means different power domains
3. **Palm engineering** - Custom tuning for webOS graphics stack
4. **Display timing** - 1024x768 @ 60Hz vs phone resolutions

**Recommendation**: Test with 0x80 on TouchPad to see if behavior changes.

### Registers with TouchPad-Specific Values

1. **RBBM_PM_OVERRIDE2 = 0x1a0**
   - Sony kernel uses 0x80 for generic A22X
   - HTC MSM8660 kernel uses 0x80 (same as Sony)
   - TouchPad uses 0x1a0 (unique!)
   - Mainline copied TouchPad value

2. **KGSL_CHIPID_LEIA_REV470**
   - TouchPad-specific chip revision
   - Triggers additional initialization (VSC pipes)

### HTC MSM8660 vs TouchPad APQ8060

| Aspect | HTC MSM8660 | TouchPad APQ8060 |
|--------|-------------|------------------|
| SoC | MSM8660 (with modem) | APQ8060 (no modem) |
| GPU Silicon | Identical A220 | Identical A220 |
| RBBM_PM_OVERRIDE2 | 0x80 | **0x1a0** |
| RBBM_PM_OVERRIDE1 | 0xfffffffe → 0x200 (8960) or 0x0 | 0xffffffff → 0x0 |
| SQ_GPR_MANAGEMENT | 0x00040400 | 0x00040400 |
| VSC init | Not explicit | All pipes zeroed |

### Leia-Specific Debug Registers

webOS kernel defines additional Leia debug registers not in Sony or HTC kernels:

| Register | Address | Purpose |
|----------|---------|---------|
| REG_LEIA_SQ_RESOURCE_MANAGMENT | 0x0D03 | Resource management |
| REG_LEIA_SQ_PIX_IN_CNTL | 0x0D0C | Pixel input control |
| REG_LEIA_CP_ME_STATUS | 0x01F7 | ME status |

---

## 10. Implementation Differences Summary

### Sony KGSL vs HTC KGSL vs webOS KGSL

| Aspect | Sony (MSM7x27A) | HTC (MSM8660) | webOS (APQ8060) |
|--------|-----------------|---------------|-----------------|
| A22X PM_OVERRIDE2 | 0x80 | 0x80 | **0x1a0** |
| PM_OVERRIDE1 final | 0x0 or 0x200 | 0x0 or 0x200 | 0x0 |
| VSC pipe init | Not explicit | Not explicit | All zeroed |
| Chip detection | Generic | Generic | LEIA_REV470 |
| SQ_GPR_MANAGEMENT | 0x00040400 | 0x00040400 | 0x00040400 |

### KGSL (All) vs Mainline Kernel

| Aspect | KGSL (Sony/HTC/webOS) | Mainline |
|--------|----------------------|----------|
| SQ_GPR_MANAGEMENT | **Set to 0x00040400** | **NOT SET** |
| SQ_INTERPOLATOR_CNTL | Set to 0xffffffff | Not set (Mesa does) |
| Context shadowing | 18KB shadow memory | No shadowing |
| VSC binning | Supported | Supported (A22X) |
| RBBM_PM_OVERRIDE2 | 0x80 (Sony/HTC) / 0x1a0 (webOS) | 0x1a0 |

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

### Key Findings

1. **SQ_GPR_MANAGEMENT (0x0D00) = 0x00040400**
   - Set by ALL KGSL implementations (Sony, HTC, webOS)
   - NOT set by mainline kernel
   - Now set by freedreno patch 0012
   - **ROOT CAUSE of intermittent faceted shading**

2. **RBBM_PM_OVERRIDE2 - TouchPad is UNIQUE**
   - Sony (MSM7x27A): 0x80
   - HTC (MSM8660): 0x80
   - **webOS (APQ8060): 0x1a0** ← Only TouchPad uses this!
   - Mainline: 0x1a0 (copied from webOS)
   - freedreno: 0xfff (all clocks on)

3. **VSC pipe initialization**
   - Only webOS explicitly zeros all VSC pipes
   - Sony and HTC rely on context save/restore

### Implications

The TouchPad's unique RBBM_PM_OVERRIDE2 value (0x1a0) may be:
- Palm-specific power optimization for tablet use case
- APQ8060-specific (no modem = different power domains)
- Tuning for 1024x768 display vs phone displays

Since HTC MSM8660 devices (which share identical GPU silicon) use 0x80, the 0x1a0 value appears to be a deliberate Palm engineering choice rather than a hardware requirement.

### Status

The freedreno driver is now functionally equivalent to KGSL for critical rendering registers after applying patches 0011 and 0012. The RBBM_PM_OVERRIDE2 difference (0xfff vs 0x1a0) keeps all GPU clocks enabled in freedreno, which may affect power consumption but ensures stability.
