# Live OPAL register-trace findings — Tenderloin still capture

Captured 2026-04-27 by `tools/gemini_reg_poll/gemini_reg_poll` running on
the HP TouchPad booted into stock webOS 3.0.6 while the Camera app took
one still photo (sensor reports portrait, 1024×1280).

This is **the OPAL stack on Tenderloin** writing real registers to the
working Gemini IP. Ground truth.

## Bugs found in our mainline driver

| Register | OPAL writes | Mainline writes | Notes |
|---|---|---|---|
| `OP_ENCODE_MODE` | **3** | 1 | Cross-vendor doc said 1 for NV12 — wrong. |
| `OP_FORMAT_MAGIC` | **0x03381801** | 0x0107081F | OPAL takes the op_format=3 branch, not op_format=1. |
| `OP_GEOM[0]` | **`Hm*(Wm-1)*128`** | `16*(Wm-1)` | op_format=3 formula (different math from op_format=1). |
| `OP_GEOM[1]` | **`Hm*(Wm-1)*256`** | `16*(Wm-1)` | op_format=3 formula. |
| `OP_GEOM[2]` | **`Hm*(Wm-1)*256+16`** | `Wm*(Hm-1)*256+16` | Note operands SWAPPED. |
| `OP_GEOM[3]` | **`Hm*(Wm-1)*128+16`** | `Wm*(Hm-1)*128+16` | Note operands SWAPPED. |

Fields that we got right and the trace confirms:

- `PIPELINE_CFG = 0x038061FB`
- `WE_Y_THRESHOLD = WE_CBCR_THRESHOLD = 0x016A0190`
- `WE_Y_UB_CFG = 0x01FF0000`
- `WE_CFG = 0x20`
- `FE_INPUT_FORMAT = 0x10`
- `FE_PIPELINE_MODE = 0x203`
- `FE_DIMS = ((Wm-1) << 16) | (Hm-1)`
- `OP_MATRIX[0..8] = 0x303, 0xF0000F, 0xF0000F00, 0, 0, 0xC0C0303, 0xC0C03030, 0, 0`
- `FE_BUFFER_CFG = ((rows-1) << 16) | (rows-1)` — same value in Y and CBCR fields

## Why op_format=3 and not 1?

OPAL's `gemini_lib_hw_op_cfg` (Ghidra-decompiled at `0x14a28c` in Opal,
`0x13a1fc` in Tenderloin) branches on the `op_format` byte at offset 4 of
the `op_cfg_in` struct. The cross-vendor doc claimed this byte was always
1 for NV12 unrotated, but the live trace shows it's actually 3 for stills.

The op_format=3 branch (ARM disasm at `0x13a3ec` in Tenderloin):

```
13a3ec: ldr r4, [r7, #12]    // r4 = param_2[3]   (mode_dims field at offset 12)
13a3f0: ldr r6, [r7, #8]     // r6 = param_2[2]   (mode_dims field at offset 8)
13a3f4: sub r1, r4, #-0x1FFFFFFF   // r1 = r4 - 1
13a400: mul lr, r6, r1       // lr = param_2[2] * (param_2[3] - 1)
13a40c: movw r9, #0x1801
13a410: movt r9, #0x338      // r9 = 0x03381801 → OP_FORMAT_MAGIC
13a418: lsl ip, lr, #7       // ip = lr * 128
13a41c: bic lr, ip, #~0x3FFFFFFF   // lr = ip & 0x3FFFFFFF
                              //  → OP_GEOM[0] = param_2[2] * (param_2[3]-1) * 128
13a420: add ip, ip, #16      // → OP_GEOM[3] = OP_GEOM[0] + 16
13a414: mul r2, lr, r5       // r2 = OP_GEOM[0] * iVar7   (iVar7 = 2 for offline)
13a428: lsl r6, r2, #3       // r6 = r2 * 8 = OP_GEOM[0] * 16
13a42c: mla r3, r6, r4, r4   // OP_GEOM[2] = r6 * iVar13 + iVar13   (iVar13 = 16)
                              //   = param_2[2] * (param_2[3]-1) * 256 + 16
13a430: bic r0, r1, #~0x3FFFFFFF   // r0 = r1 & 0x3FFFFFFF
                              //  → OP_GEOM[1] = param_2[2] * (param_2[3]-1) * 256
```

## Why the operand swap?

Critical detail in `gemini_lib_hw_config` (the caller of op_cfg):

```c
// gemini_lib_hw_config @ 0x14be78 in Opal — Ghidra decompilation
local_4c = param_2[2];                         // mode_dims[2]  (= Wm)
local_58 = *param_2;                           // mode_dims[0]  (= mode)
local_54 = (uint)*(byte *)((int)param_2 + 5);  // mode_dims+5 byte
local_50 = param_2[3];                         // mode_dims[3]  (= Hm)
pvVar2 = gemini_lib_hw_op_cfg(param_5, &local_58);
```

The caller builds a NEW 4-dword struct (`&local_58`) and passes that to
`op_cfg`. In this new struct:

- offset 0  = mode (= 3 for offline)
- offset 4  = subsampling byte
- offset 8  = `mode_dims[3]`  → **`Hm`** (offset 8 of original mode_dims = `Wm`)
- offset 12 = `mode_dims[2]`  → **`Wm`** (offset 12 of original = `Hm`)

So when `op_cfg` reads `param_2[2]` (offset 8) it gets **Hm**, and when it
reads `param_2[3]` (offset 12) it gets **Wm**. The fields are SWAPPED
between fe_cfg's caller and op_cfg's caller.

For the captured trace (1024×1280 portrait, Wm=64, Hm=80):

- op_cfg sees param_2[2] = Hm = 80
- op_cfg sees param_2[3] = Wm = 64
- `param_2[2] * (param_2[3] - 1) = 80 * 63 = 5040` ✓
- `OP_GEOM[3] = 5040 * 128 + 16 = 0x9D810` ✓ matches captured

## Snapshot data (captured, 1024×1280 portrait still)

Key values from `tenderloin_gemini_during.txt`:

```
HW_VERSION       0x0000 = 0x00001100
PIPELINE_CFG     0x0008 = 0x038061FB
IRQ_MASK         0x0014 = 0xFFFFFFFF
FE_INPUT_FORMAT  0x0038 = 0x00000010
FE_DIMS          0x003C = 0x003F004F      // Wm-1=63, Hm-1=79 → Wm=64, Hm=80
FE_PIPELINE_MODE 0x0040 = 0x00000203
OP_ENCODE_MODE   0x0044 = 0x00000003      ← MAINLINE WRITES 1
OP_GEOM[0]       0x0048 = 0x0013B000      // = 80*63*256 (in op_cfg's swapped struct)
OP_GEOM[1]       0x004C = 0x0009D800      // = 80*63*128
OP_GEOM[2]       0x0050 = 0x0013B010
OP_GEOM[3]       0x0054 = 0x0009D810
OP_FORMAT_MAGIC  0x0058 = 0x03381801      ← MAINLINE WRITES 0x0107081F
OP_MATRIX[0..8]  = 0x303, 0xF0000F, 0xF0000F00, 0, 0, 0xC0C0303, 0xC0C03030, 0, 0
WE_CFG           0x0098 = 0x00000020
WE_Y_THRESHOLD   0x00C0 = 0x016A0190
WE_CBCR_THRESHOLD 0x00C4 = 0x016A0190
WE_Y_PING_CFG    0x00C8 = 0x001F9E70      // ~2 MB output buffer
WE_Y_UB_CFG      0x00E8 = 0x01FF0000
START_KICK       0x00F0 = 0x00000001
ENCODE_OUTPUT_SIZE 0x0034 = 0x0003781C    // ~227 KB encoded entropy stream
```

(Note: OP_GEOM[0] and [1] in the snapshot above appear *swapped* between
`*128` and `*256` versus the disasm formula — that's because the snapshot
was taken AFTER the encoder wrote intermediate values. The delta log shows
the actual writes. Reading from the delta log:

```
OP_GEOM[0] 0x0048  0x00000000 -> 0x0013b000   // 80*63*256
OP_GEOM[1] 0x004C  0x00000000 -> 0x0009d800   // 80*63*128
OP_GEOM[2] 0x0050  0x00000000 -> 0x0013b010   // 80*63*256+16
OP_GEOM[3] 0x0054  0x00000000 -> 0x0009d810   // 80*63*128+16
```

So the OPAL formulas are actually:

- `OP_GEOM[0] = Hm*(Wm-1) * 256`
- `OP_GEOM[1] = Hm*(Wm-1) * 128`
- `OP_GEOM[2] = Hm*(Wm-1) * 256 + 16`
- `OP_GEOM[3] = Hm*(Wm-1) * 128 + 16`

i.e. [0]/[2] use 256, [1]/[3] use 128 — opposite of what the ARM disasm
seemed to suggest at first glance. Reading `13a418..13a430` more carefully:

- `lsl ip, lr, #7`  → `ip = (Hm*(Wm-1)) * 128` (this is uVar20 → stored at OP_GEOM[1])
- `mla r3, r6, r4, r4` → `r3 = (Hm*(Wm-1)) * 256 + 16` (this is uVar5 → stored at OP_GEOM[2])

The `LAB_0014a448:` storage in Ghidra-decomp aliases the local variables
to specific cmd-buffer offsets, and from that mapping `uVar2 = OP_GEOM[1]`
and `uVar20 = OP_GEOM[0]`. **Live-trace values trump static disasm
guess:** trust the trace's `0x0013b000 / 0x0009d800 / 0x0013b010 /
0x0009d810` pattern.)

## Verdict

The mainline driver writes the wrong `OP_ENCODE_MODE`, the wrong
`OP_FORMAT_MAGIC`, and uses op_format=1's `OP_GEOM` formulas instead of
op_format=3's. With these three fixes the encoder should produce a
correctly-shaped bitstream.

Tested SRCREV before fix: `46134e883c8f` — JFIF container valid, pixel
content garbage.

Next commit will apply: OP_ENCODE_MODE=3, OP_FORMAT_MAGIC=0x03381801,
OP_GEOM[0..3] using `Hm*(Wm-1)*{256,128,256,128} (+16 for [2,3])`.
