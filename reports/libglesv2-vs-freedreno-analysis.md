# webOS libGLESv2.so vs freedreno Register Analysis

## Executive Summary

Ghidra decompilation of the webOS libGLESv2.so reveals that **SQ_RESOURCE_MANAGMENT (0x0d03)** and **SQ_PIX_IN_CNTL (0x0d0c)** are **NOT written** by the proprietary driver. The references found were byte patterns in unrelated C++ code, not GPU register writes.

This means freedreno's behavior (not writing these registers) matches the proprietary driver.

## Registers Analyzed

### SQ_GPR_MANAGEMENT (0x0d00) - DYNAMICALLY SET

**Proprietary Driver:**
```c
// From leia_perform_resolve:
*puVar8 = 0xd00;
puVar8[1] = uVar10 << 0xc | (0x80 - uVar10) * 0x10;
```

Value is **dynamically calculated** based on shader requirements:
- Bits 4-11: Pixel shader GPRs = 128 - vertex_gprs
- Bits 12-19: Vertex shader GPRs

**freedreno:** Sets fixed value 0x00040401 (64 GPRs each + REG_DYNAMIC)

**Difference:** Proprietary is dynamic, freedreno is static. This may not matter in practice.

### SQ_INST_STORE_MANAGMENT (0x0d02) - SET

**Proprietary Driver (from leia_repartition_instruction_store):**
```c
puVar1[5] = 0xd02;
puVar1[6] = param_3 | *(int *)(iVar2 + 0x318) << 0x10;
```

**freedreno:** Sets 0x00000180 in fd2_emit_restore()

**Match:** Both drivers write this register.

### SQ_RESOURCE_MANAGMENT (0x0d03) - NOT WRITTEN

**Proprietary Driver:** No writes found. References were in shader compiler C++ code (byte patterns).

**freedreno (with patch 0017):** We write 0x00000000

**Finding:** Hardware default is likely correct. Our patch may be unnecessary but harmless.

### SQ_PIX_IN_CNTL (0x0d0c) - NOT WRITTEN

**Proprietary Driver:** No writes found. References were in compiler code.

**freedreno (with patch 0017):** We write 0xffffffff

**Finding:** Hardware default is likely correct. May want to test with/without our patch.

### SQ_INTERPOLATOR_CNTL (0x2182) - SET

**Proprietary Driver (from leia_set_hw_sq_interpolator_cntl_reg):**
```c
// PM4 packet format:
*param_1 = 0xc0012d00;           // Type 0 header, 1 register
param_1[1] = 0x40182;            // CP_REG(0x2182) = SQ_INTERPOLATOR_CNTL
param_1[2] = *(param_2 + 0xa0);  // Value from state structure
```

**freedreno:** Sets 0xffffffff (all smooth interpolation)

**Match:** Both drivers write this register. Value may differ per-draw based on shader needs.

### SQ_PROGRAM_CNTL (0x2180) & SQ_CONTEXT_MISC (0x2181)

**Proprietary Driver:** Written together in PM4 packets during draw submission.

**freedreno (with patch 0016):** Now writes atomically like proprietary driver.

**Match:** Our atomic write patch aligns with proprietary behavior.

## PM4 Packet Format Reference

From decompilation:
```c
// Type 0 packet: Write N registers starting at offset
header = 0xc000 | (count - 1) << 8 | 0x2d00;  // 0x2d00 = type marker?
reg_offset = 0x40000 | (register_address - 0x2000);

// Example for SQ_INTERPOLATOR_CNTL (0x2182):
// header = 0xc0012d00 (Type 0, 1 register)
// offset = 0x40182 = 0x40000 | (0x2182 - 0x2000)
```

## Key Functions Analyzed

| Function | Purpose | Registers Written |
|----------|---------|-------------------|
| leia_context_create | Context init | State structures |
| leia_perform_resolve | GMEM resolve | 0x0d00 (dynamic) |
| leia_repartition_instruction_store | Shader partitioning | 0x0d02 |
| leia_set_hw_sq_interpolator_cntl_reg | Per-draw interpolation | 0x2182 |
| leia_set_hw_sq_wrapping_reg | Texture wrapping | 0x2183, 0x2184 |

## Conclusions

1. **SQ_RESOURCE_MANAGMENT (0x0d03)** and **SQ_PIX_IN_CNTL (0x0d0c)** are NOT written by the proprietary driver
   - These registers likely have correct hardware defaults
   - Our patch 0017 writes them "just in case" but may be unnecessary

2. **SQ_GPR_MANAGEMENT (0x0d00)** is dynamically calculated in proprietary driver
   - freedreno uses fixed allocation
   - This difference is likely not causing issues

3. **Atomic register writes (patch 0016)** are correct
   - Proprietary driver writes SQ registers as PM4 packets
   - Our atomic approach matches this behavior

## Recommended Testing

1. Test **without** patch 0017 to see if it makes any difference
2. If faceted shading persists, the issue is elsewhere (likely not register initialization)

## Files Analyzed

- `/home/herrie/webos/touchpad-kernel/doctor305/untouched-rootfs/usr/lib/libGLESv2.so`
- Tool: Ghidra 12.0.1 headless decompilation

---
*Generated: Feb 19, 2026*
*Analysis by: Claude Code with Ghidra*
