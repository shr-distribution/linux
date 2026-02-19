# Leia (Adreno 220/225) SQ Register Analysis

## Overview

The HP TouchPad uses an Adreno 220 GPU (codename "Leia"). During investigation of
intermittent faceted shading issues, two Leia-specific registers were discovered
that are defined in KGSL but never used in kernel code:

- `REG_LEIA_SQ_RESOURCE_MANAGMENT` (0x0d03)
- `REG_LEIA_SQ_PIX_IN_CNTL` (0x0d0c)

## Register Details

### SQ Register Map (0x0d00 range)

| Address | Name | Status in KGSL | Status in Freedreno |
|---------|------|----------------|---------------------|
| 0x0d00 | SQ_GPR_MANAGEMENT | Saved/restored via shadow | Set in fd2_emit_restore() |
| 0x0d01 | SQ_FLOW_CONTROL | Not used | Not documented |
| 0x0d02 | SQ_INST_STORE_MANAGMENT | Saved/restored via shadow | Set in fd2_emit_restore() |
| 0x0d03 | SQ_RESOURCE_MANAGMENT | **Defined, never used** | **NOT DEFINED** |
| 0x0d04 | (Unknown) | Not defined | Not defined |
| 0x0d05 | SQ_DEBUG_MISC | Debug reads only | Not used |
| ... | | | |
| 0x0d0c | SQ_PIX_IN_CNTL | **Defined, never used** | **NOT DEFINED** |

### REG_LEIA_SQ_RESOURCE_MANAGMENT (0x0d03)

**Hypothesis based on naming and location:**
- Located between SQ_INST_STORE_MANAGMENT (0x0d02) and SQ_DEBUG_MISC (0x0d05)
- Name suggests it manages shader resource allocation
- May control:
  - Thread scheduling resources
  - Texture unit allocation
  - ALU/fetch unit assignment
  - Buffer/cache resources

**Possible Bitfield Structure (speculative):**
Similar to SQ_GPR_MANAGEMENT which has:
- Bits for vertex shader resources
- Bits for pixel shader resources
- Dynamic allocation mode flag

### REG_LEIA_SQ_PIX_IN_CNTL (0x0d0c)

**Hypothesis based on naming:**
- "Pixel Input Control" - controls pixel shader input handling
- May affect:
  - Varying interpolation input setup
  - Barycentric coordinate handling
  - Pixel coverage handling
  - Input register mapping

**Critical for interpolation:**
If this register controls how varying data is fed to the pixel shader,
incorrect settings could cause:
- Faceted shading (no smooth interpolation)
- Wrong varying values
- Missing texture coordinates

## Investigation Results

### KGSL Analysis

1. **leia_reg.h** - Both registers are defined:
   ```c
   #define REG_LEIA_SQ_RESOURCE_MANAGMENT 0xd03
   #define REG_LEIA_SQ_PIX_IN_CNTL 0xd0c
   ```

2. **Usage search** - Neither register is written anywhere in:
   - kgsl_drawctxt.c (context save/restore)
   - kgsl_yamato.c (GPU init)
   - kernel-3.0.5.txt (webOS patches)

3. **Context management** - These are NOT part of the context save/restore
   register ranges, unlike SQ_GPR_MANAGEMENT and SQ_INST_STORE_MANAGMENT.

### Freedreno Analysis

1. **a2xx.xml** - Neither register is documented in Mesa's register database
2. **fd2_emit.c** - No code references these registers
3. **Gap in register definitions** - 0x0d03-0x0d0b are undefined

## Possible Explanations

1. **Userspace driver handles them** - The proprietary libGLESv2.so may write
   these registers through PM4 command packets

2. **Hardware defaults sufficient** - These may have correct power-on defaults

3. **Leia-specific but optional** - May provide tuning capabilities not
   needed for basic operation

4. **Documentation artifacts** - Defined for completeness but never implemented

## Recommendation

Since these registers are undefined in freedreno and KGSL doesn't write them,
the proprietary driver likely sets them. Without access to the userspace
driver, we should:

1. Add these registers to the freedreno a2xx register database
2. Experiment with initialization values in fd2_emit_restore()
3. Test different values to see if they affect rendering

### Proposed Test Values

**SQ_RESOURCE_MANAGMENT (0x0d03):**
- Start with 0x00000000 (hardware default)
- Try values similar to SQ_GPR_MANAGEMENT structure

**SQ_PIX_IN_CNTL (0x0d0c):**
- Start with 0x00000000 (hardware default)
- Try enabling all inputs: 0xFFFFFFFF
- Try values that might enable interpolation

## Related Registers

For context, here's how similar registers are configured:

### SQ_GPR_MANAGEMENT (0x0d00)
```c
// KGSL sets: 0x00040400 (64 GPRs each for VS/PS)
// Freedreno sets: 0x00040401 (64 GPRs each + REG_DYNAMIC)
```

### SQ_INST_STORE_MANAGMENT (0x0d02)
```c
// Freedreno sets: 0x00000180
// Controls instruction store partitioning
```

### SQ_INTERPOLATOR_CNTL (0x2182)
```c
// Both KGSL and Freedreno set: 0xFFFFFFFF
// All 32 varying slots use smooth interpolation
```

## References

- KGSL source: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/`
- Freedreno registers: `src/freedreno/registers/adreno/a2xx.xml`
- Mesa freedreno driver: `src/gallium/drivers/freedreno/a2xx/`

---
*Document created: Feb 19, 2026*
*Author: Claude Code analysis*
