# HTC Vendor EGL Binary Analysis for Adreno 220 (Leia)

## Summary

Ghidra decompilation analysis of HTC vendor EGL binaries from `ev_shooter-8.0.0-userbuild-2017.11.11` to identify register values and hardware initialization for Adreno 220 (Leia) GPU that may be missing in freedreno or kernel drivers.

**Key Finding: freedreno uses the wrong VGT_VERTEX_REUSE_BLOCK_CNTL value for Leia (A220)!**

## Critical Register Difference: VGT_VERTEX_REUSE_BLOCK_CNTL (0x2316)

| Driver/Binary | Value | Chip Target |
|---------------|-------|-------------|
| **HTC Vendor (Leia)** | `0x3b` | Adreno 220 specifically |
| **HTC Vendor (generic)** | `0x28f` | Other A2XX chips |
| **freedreno** | `0x28f` | ALL A2XX chips (BUG!) |
| **webOS kernel** | `0x02` | Leia specifically |

### Decompiled Code Evidence

From `FUN_0009f040` (line 114444-114451):
```c
if (param_4[2] == 0) {
    local_30[1] = 0x28f;      // Non-Leia A2XX
    local_30[0] = local_30[0] | 0x8000;
}
else {
    local_30[1] = 0x3b;       // Leia (A220)
}
FUN_0009e98c(puVar3, ..., 0x2316, local_30 + 1);  // Write to VGT_VERTEX_REUSE_BLOCK_CNTL
```

From `leia_init_hw` (line 109785-109787):
```c
local_30 = 0x3b;
uVar7 = FUN_00098118(..., 0x2316, &local_30, 1);  // VGT_VERTEX_REUSE_BLOCK_CNTL = 0x3b
```

### Analysis of 0x3b Value

`0x3b` = binary `00111011`:
- Bits 0-2 (VTX_REUSE_DEPTH): `011` = 3
- Bit 3: 1 (set)
- Bit 4: 1 (set)
- Bit 5: 1 (set)

Compared to freedreno's `0x28f` = binary `001010001111`:
- Bits 0-2 (VTX_REUSE_DEPTH): `111` = 7
- Bit 3: 1
- Bit 7: 1
- Bit 9: 1

## Leia Device ID Check

The HTC binary uses device ID `0xe1` to identify Adreno 220 (Leia) and applies different configurations:

```c
if (*(int *)(iVar + 0x20) == 0xe1) {
    // Leia-specific initialization
}
```

## Other Leia-Specific Register Values from leia_init_hw()

| Register | Address | Value | Description |
|----------|---------|-------|-------------|
| VGT_VERTEX_REUSE_BLOCK_CNTL | 0x2316 | 0x3b | Vertex reuse control |
| SC_SCISSOR | 0x2100 | 0xffffffff, 0 | Scissor test |
| RB_MODECONTROL | 0x2102 | 0x00 | Render backend mode |
| PA_SC_VIZ_QUERY | 0x2208 | 0x04 | Visibility query |
| PA_SC_LINE_CNTL | 0x2282 | 0x08 | Line control |
| PA_SU_POINT_SIZE | 0x2280 | 0x80008 | Point size |
| VGT_GS_MODE | 0x2301 | 0x00 | Geometry shader mode |
| PA_SC_??? | 0x2326 | 0xffffffff | Unknown register |
| ??? | 0x231b | 0x3c000 | Unknown register |

## Instruction Store Sizes (Leia-specific)

From leia_init_hw (line 109841-109852):
```c
if (device_id == 0xe1) {  // Leia
    *(iVar9 + 0x68c) = 0;
    *(iVar9 + 0x688) = 0x600;   // Larger instruction store
    uVar7 = 0x300;
}
else {
    *(iVar9 + 0x68c) = 0;
    *(iVar9 + 0x688) = 0x200;   // Smaller instruction store
    uVar7 = 0x180;
}
```

## State Structure Initialization (leia_init_hw)

Offsets from context structure at `(param_1 + 0x1684)`:
| Offset | Value | Description |
|--------|-------|-------------|
| 0x4b8 | 0x80000 | PA_SU_SC_MODE_CNTL default |
| 0x4b0 | 0x20 | Unknown |
| 0x4a0-0x4ac | 0x10001 | 4 consecutive registers |
| 0x4e8 | 0xffffffff | SQ_INTERPOLATOR_CNTL |
| 0x4d0 | 0x01 | Unknown |
| 0x4b4 | 0x00 | Unknown |
| 0x49c | 0x80 or 0x88 | Conditional on capabilities |
| 0x4bc | 0x43f | Unknown |
| 0x4c8 | 0xffff | Unknown |
| 0x4e4 | 0x04 | SQ_PROGRAM_CNTL |
| 0x4cc | 0x88888888 | Unknown |
| 0x48c-0x498 | 0x3f800000 | Floating point 1.0 (4 registers) |

## Recommendation

**Immediate fix for freedreno Mesa driver:**

In `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c` and `fd2_draw.c`, change VGT_VERTEX_REUSE_BLOCK_CNTL for Leia (A220) chips:

```c
// For Leia (A220) chips:
OUT_PKT3(ring, CP_SET_CONSTANT, 2);
OUT_RING(ring, CP_REG(REG_A2XX_VGT_VERTEX_REUSE_BLOCK_CNTL));
OUT_RING(ring, is_a220(screen) ? 0x0000003b : 0x0000028f);  // Leia uses 0x3b!
```

This matches the HTC vendor driver's behavior of using different values for Leia vs other A2XX chips.

## Files Analyzed

- `/home/herrie/Downloads/HTC/ev_shooter-8.0.0-userbuild-2017.11.11/Android_Image_Tools_v4/EXTRACTED_IMAGES/extracted_myimage/vendor/lib/egl/libGLESv2_adreno.so` (1MB, 3662 functions)
- Decompiled output: `/tmp/ghidra_analysis/decompiled_output.c` (155,106 lines)

## Key Functions Found

| Function | Address | Purpose |
|----------|---------|---------|
| leia_init_hw | 0x0009a33c | Main Leia hardware initialization |
| leia_set_hw_sq_interpolator_cntl_reg | 0x0009a030 | Set interpolation control |
| leia_set_hw_pa_su_sc_mode_cntl_reg | 0x0009a100 | Set polygon mode control |
| leia_preamble_init_register_state | 0x000a3dbc | Initialize register state |
| FUN_0009f040 | 0x0009f040 | Shader loading (sets VGT_VERTEX_REUSE) |
| leiaHwlInit | 0x000995b0 | Leia hardware layer init |

## Date

Analysis performed: 2026-04-13
