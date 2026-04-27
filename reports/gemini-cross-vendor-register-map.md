# Qualcomm Gemini JPEG Hardware (MSM8660) — Cross-Vendor Register Reference

This document is the definitive cross-reference for programming the Qualcomm
Gemini JPEG IP found at MMIO `0x4600000` on MSM8660/APQ8060. All values were
recovered by disassembling four independent vendor camera HALs that link
against (or bundle) `libgemini.so`, then cross-checking the values each vendor
passes for an offline NV12 → JPEG encode.

The libgemini wire format is **identical across all four vendors** (verified
in two independent decompiles). What differs is the values the camera HAL
passes for the input structs.

## Source binaries

| Vendor | Binary | Decompile (where available) |
|---|---|---|
| **webOS Opal** (HP TouchPad) | `doctor305/untouched-rootfs/usr/lib/libqcameralib.so` (bundles libgemini logic) | `reports/opal-camera/opal_libqcameralib_decompiled.c` |
| **Samsung Quincy** | `reports/samsungjpeg/libgemini.so` + `libmmjpeg(3).so` | `reports/samsung-quincy-extra/quincy_libmmjpeg.c` |
| **HTC Shooter U** | `reports/htc/.../proprietary/{libgemini,libgemini2,libmmjpeg,libmmjpeg2}.so` | (none — disasm only) |
| **Sony Nozomi** | `reports/sony_nozomi/{liboemcamera,camera.msm8660,v4l2-qcamera-app}` | `reports/sony_nozomi/*_decompiled.c` |

## libgemini ABI

`gemini_lib_hw_config(fd, mode_dims_ptr, we_cfg_ptr, offline_struct_ptr, op_cfg_ptr)`
is the single entry point that emits the `MSM_JPEG_IOCTL_HW_CMDS` triples
covering everything except per-frame buffer addresses and the start kick.

**Wire format**: each command is a 12-byte triple `{u32 cmd:4|n:12|offset:16, u32 mask, u32 data}`.
Buffer prefixed by `u32 count`. `cmd=0x11` = WRITE, `cmd=0x12` = WRITE_OR_AND
(read-modify-write).

### Input struct layouts

`mode_dims` (16 bytes):
| Offset | Size | Meaning |
|---|---|---|
| +0 | u32 | `mode` ∈ {1, 2, 3} — 3 = offline encode |
| +4 | u8 | FE_INPUT_FORMAT bit 6 source (Y/CbCr swap) |
| +5 | u8 | FE_INPUT_FORMAT bits 5:4 source (subsampling code) |
| +6 | u8 | FE_INPUT_FORMAT bits 2:0 source (pixel-format code) |
| +7 | u8 | pad |
| +8 | u32 | `Wm` = `(W+15) >> 4` (width in MCU units) |
| +12 | u32 | `Hm` = `(H+15) >> 4` (height in MCU units) |

`we_cfg`:
| Offset | Size | Meaning |
|---|---|---|
| +0 | u8 | WE_CFG bits 5:4 source — burst-length code (always 2 for NV12) |
| +1 | u8 | WE_CFG bits 2:0 source — format code (always 0 for NV12 H2V2 offline) |

`offline_struct` (≥36 bytes; only fields used by hw_config):
| Offset | Size | Meaning |
|---|---|---|
| +0 | u16 | DRI restart-marker interval |
| +2 | u8 | `load_huffman_tables` flag (≠0 → call set_huffman_tables) |
| +5 | u8 | source of PIPELINE_CFG bit 10 (clk-gate enable). **Correction**: previous doc said "always 1 for offline" — actually OPAL's `gemini_lib_hw_config` passes 0 here, so bit 10 is **0** in the final PIPELINE_CFG word for offline encode on this exact silicon. Samsung's HAL may pass 1. |
| +20 | u32 | ptr to luma quant table (64 bytes natural order) |
| +24 | u32 | ptr to chroma quant table (64 bytes natural order) |
| +28 | u8 | filesize-control enable; if non-zero, programs FSC[0..4] regs |
| +32+ | — | filesize-control 5-dword struct |

`op_cfg`:
| Offset | Size | Meaning |
|---|---|---|
| +0 | u8 | branch select: 0 = realtime; ≠0 = offline (compute geometry) |
| +4 | u32 | `op_format` ∈ {0,1,2,3} — selects OP_GEOM formula and OP_FORMAT_MAGIC |

## Per-vendor input values for offline NV12 unrotated encode

| Field | webOS Opal | Samsung | HTC | Sony | Universal? |
|-------|-----------|---------|-----|------|-----------|
| `mode_dims.mode` | 3 | 3 (`local_cc=3`) | 3 | 3 (`mov lr,#3`) | **YES** |
| `mode_dims.Wm` | `(W+15)>>4` | `(W+15)>>4` | `(W+15)>>4` | `(W+15)>>4` | YES (formula) |
| `mode_dims.Hm` | `(H+15)>>4` | `(H+15)>>4` | `(H+15)>>4` | `(H+15)>>4` | YES (formula) |
| `mode_dims+5` (subsampling) | runtime | 1 (`local_c7=1`) | runtime | runtime | NV12 typically =1 |
| `mode_dims+6` (pixel-fmt) | runtime | 0 (`local_c6=0`) | runtime | runtime | NV12 typically =0 |
| `we_cfg+0` (burst code) | LUT (typ. 2) | 2 | 2 | 2 | **YES** for NV12 |
| `we_cfg+1` (format code) | LUT (typ. 0) | 0 | 0 | 0 | **YES** for NV12 |
| `op_cfg.byte[0]` (offline branch) | ≠0 | 1 (rot=0) | ≠0 | 3 | **YES** ≠0 |
| `op_cfg.dword[4]` (op_format) | 1 | 1 (rot=0) | 1 | runtime | **YES** =1 unrotated |
| `offline_struct.dri_interval` | 0 | 0 | 0 | 0 | **YES** =0 baseline |
| `offline_struct.load_huff` | 1 | 1 | 1 | 1 | **YES** — all four LOAD Huffman |
| `offline_struct.fsc_enable` | 0 | 0 | 0 | 0 | **YES** =0 baseline |

## Computed register values

| Register | Offset | Mask | Value (offline NV12 H2V2 unrotated, no DRI, no FSC) |
|----------|--------|------|------|
| HW_VERSION | 0x0000 | 0x000FFFFF | RO |
| RESET_CMD | 0x0004 | n/a | kernel-side: write `1` |
| **PIPELINE_CFG** | 0x0008 | 0x07F775FF | **`0x038061FB`** for unrotated NV12 H2V2 offline. **Corrected** — see "Settled disagreements" below. Earlier inferred `0x020065FB` was wrong. Decoded from OPAL `gemini_lib_hw_pipeline_cfg` @ 0x14a69c (libqcameralib bundled libgemini) and confirmed against Samsung libgemini ARM disasm @ 0x31a0. Formula: `0x61FB \| (offline ? 0x02000000 : 0) \| (p[5]&1)<<10 \| (p[1]&3)<<23 \| (p[2]&1)<<22 \| (p[3]&1)<<21 \| (p[4]&1)<<20 \| (p[6]&1)<<2`. OPAL's caller (`gemini_lib_hw_config`) fills the input struct: byte0=offline_struct.byte[0] (≠0 for offline), byte1=mode_dims.mode lower byte (=3 for offline), bytes2-6=0. So the offline value is `0x61FB \| 0x02000000 \| (3<<23) = 0x038061FB`. Bit 10 is **0** on OPAL (not 1 as earlier doc claimed). Bits 23-24 carrying the mode value were missed by earlier inference. Samsung's libgemini has an extra bit-9 source (p[7]) that OPAL's caller leaves uninitialized — does not change the OPAL output. |
| REALTIME_CMD | 0x000C | 0x3 | unused for offline |
| IRQ_MASK | 0x0014 | 0xFFFFFFFF | `0xFFFFFFFF` initially; narrow to `0x0C` (bits 2,3) at start |
| IRQ_CLEAR | 0x0018 | 0xFFFFFFFF | `0xFFFFFFFF` |
| IRQ_STATUS | 0x001C | RO | poll for FRAMEDONE / BUS_ERROR / VIOLATION |
| STOP_REQ | 0x0024 | 0x1 | `1` to request stop |
| STOP_STATUS | 0x0028 | 0x3 | poll for `==3` after STOP_REQ |
| ENCODE_OUTPUT_SIZE | 0x0034 | RO | read after FRAMEDONE for entropy length |
| **FE_INPUT_FORMAT** | 0x0038 | 0x000F0077 | **`0x10`** (subsampling=1 in bits 5:4) for NV12. May add bit 6 for NV21. |
| **FE_DIMS** | 0x003C | 0x01FF01FF | **`((Wm-1) << 16) \| (Hm-1)`** — for 640×480: `(39<<16) \| 29 = 0x0027001D` |
| **FE_PIPELINE_MODE** | 0x0040 | 0x000107FF | **`0x203`** for `mode==3` (offline encode); `0x101` for `mode==1` or `2` |
| **OP_ENCODE_MODE** | 0x0044 | 0x3 | **`1`** for unrotated NV12 (= `op_format & 3` with op_format=1) |
| **OP_GEOM[0]** (Y line stride - 1) | 0x0048 | 0x03FFFFFF | **`16 * (Wm-1)`** |
| **OP_GEOM[1]** (CbCr line stride - 1) | 0x004C | 0x03FFFFFF | **`16 * (Wm-1)`** |
| **OP_GEOM[2]** (Y frame bytes) | 0x0050 | 0x03FFFFFF | **`256 * Wm * (Hm-1) + 16`** |
| **OP_GEOM[3]** (CbCr frame bytes) | 0x0054 | 0x03FFFFFF | **`128 * Wm * (Hm-1) + 16`** |
| **OP_FORMAT_MAGIC** | 0x0058 | 0x033F1F1F | **`0x0107081F`** (op_format=1) |
| **OP_MATRIX[0]** | 0x005C | 0x0000FFFF | **`0x00000303`** (mode=3 row, sl=1) |
| **OP_MATRIX[1]** | 0x0060 | 0xFFFFFFFF | **`0x00F0000F`** |
| **OP_MATRIX[2]** | 0x0064 | 0xFFFFFFFF | **`0xF0000F00`** |
| **OP_MATRIX[3]** | 0x0068 | 0xFFFFFFFF | **`0x00000000`** |
| **OP_MATRIX[4]** | 0x006C | 0xFFFFFFFF | **`0x00000000`** |
| **OP_MATRIX[5]** | 0x0070 | 0xFFFFFFFF | **`0x0C0C0303`** |
| **OP_MATRIX[6]** | 0x0074 | 0xFFFFFFFF | **`0xC0C03030`** |
| **OP_MATRIX[7]** | 0x0078 | 0xFFFFFFFF | **`0x00000000`** |
| **OP_MATRIX[8]** | 0x007C | 0xFFFFFFFF | **`0x00000000`** |
| FE_BUFFER_CFG | 0x0080 | n/a | kernel programs MCU rows for FE pingpong |
| FE_Y_PING_ADDR | 0x0084 | n/a | per-buffer Y plane DMA addr |
| FE_Y_PONG_ADDR | 0x0088 | n/a | (pong unused in single-buffer M2M) |
| FE_CBCR_PING_ADDR | 0x008C | n/a | per-buffer CbCr plane DMA addr (= Y + W*H) |
| FE_CBCR_PONG_ADDR | 0x0090 | n/a | (unused) |
| **FE_CMD** | 0x0094 | 0x3 | `1` = buffer-reload pulse (per-frame); **`3` = offline-encode start** |
| **WE_CFG** | 0x0098 | 0x000F0F37 | **`0x20`** (we[0]=2 in bits 5:4) |
| WE_Y_THRESHOLD | 0x00C0 | 0x1FF01FF | offline thresholds, e.g. `0x016A0190` (legacy) or `0x00FF00FF` |
| WE_CBCR_THRESHOLD | 0x00C4 | 0x1FF01FF | similar |
| WE_Y_PING_CFG | 0x00C8 | 0x7FFFFF | output buffer length (≤ 23-bit) |
| WE_Y_PONG_CFG | 0x00CC | 0x7FFFFF | (unused) |
| WE_Y_PING_ADDR | 0x00D8 | 0xFFFFFFF8 | output DMA addr (8-byte aligned) |
| WE_Y_PONG_ADDR | 0x00DC | 0xFFFFFFF8 | (unused) |
| WE_Y_UB_CFG | 0x00E8 | n/a | `0x01FF0000` (encode UB allocation) |
| **START_KICK** | 0x00F0 | 0x1 | **`1`** before FE_CMD start |
| **DRI_INTERVAL** | 0x00F4 | 0x0001FFFF | low 16 bits = restart interval; bit 16 = enable. Baseline: `0` |
| FSC[0] (count) | 0x0110 | 0x0000011F | only if FSC enabled |
| FSC[1..4] (thresholds) | 0x0114..0x0120 | 0xFFFFFFFF | only if FSC enabled |
| **TABLE_SEL** | 0x0124 | 0x7 | `1`=quant luma, `2`=quant chroma, `5`=huff luma, `6`=huff chroma, `0`=deselect |
| **TABLE_INDEX** | 0x0128 | 0x3FF | reset to 0 before each table; HW auto-increments on each TABLE_DATA write |
| **TABLE_DATA** | 0x012C | 0xFFFFFFFF | per-entry payload (quant: reciprocal-of-Q; huff: packed (len-1, code) word) |

## Quant + Huffman table loading

**Quant tables** load in this exact sequence per the universal libgemini disasm:

```
TABLE_SEL = 1                  ; luma
TABLE_INDEX = 0
× 64: TABLE_DATA = q_luma[i]   ; auto-increment in HW
TABLE_SEL = 2                  ; chroma
TABLE_INDEX = 0
× 64: TABLE_DATA = q_chroma[i]
TABLE_SEL = 0                  ; deselect
```

Quant table byte order is **NATURAL** (row-major 8x8), not zigzag. The
`gemini_app_quant_zigzag_table_helper` HAL helper exists but rearranges
**before** filling `offline_struct[20]/[24]`; by the time bytes reach
TABLE_DATA they are in natural JPEG-spec order.

**Huffman tables**: same shape, different SEL values:

```
TABLE_SEL = 5                  ; huffman luma (DC + AC interleaved)
TABLE_INDEX = 0
× N: TABLE_DATA = ((len-1) << 16) | ((code << (16-len)) & 0xFFFF)
TABLE_SEL = 6                  ; huffman chroma
TABLE_INDEX = 0
× N: TABLE_DATA = ...
TABLE_SEL = 0
```

The (length, code) entries come from JPEG Annex C applied to BITS[16] +
HUFFVAL[] arrays. All four vendors load Annex K standard tables.

## Encode start sequence

After all configuration packets are submitted and per-buffer addresses
programmed via the per-frame buffer-enqueue ioctls, the start sequence is:

```
IRQ_MASK     ← 0xFFFFFFFF       ; unmask all
START_KICK   ← 0x00000001       ; bit 0 — encoder go
FE_CMD       ← 0x00000003       ; offline-encode start
```

No vendor inserts a settling delay or status-bit poll between the last
configuration packet and the start. No vendor writes to register `0xF0`
from userspace — the kernel-side ioctl handler for the start command
generates that write.

## Settled disagreements

The two earlier reverse-engineering passes disagreed on a few points; the
comprehensive cross-vendor compare resolved them:

| Question | Earlier confusion | Settled answer |
|---|---|---|
| `mode_dims.mode` for offline encode | "0 or 3?" | **3** universally; offline branch in fe_cfg/op_cfg requires this |
| `op_cfg.dword[4]` (op_format) for unrotated NV12 | "0 or 1?" | **1**; this is the value that goes to OP_ENCODE_MODE register |
| `OP_ENCODE_MODE` register value for unrotated | conflicting reports | **1**, NOT 0 |
| `OP_MATRIX` row selection | confused with realtime path | **sl=1 row** (mode=3 path), NOT sl=0 |
| `FE_DIMS` field order | rows-vs-cols inverted | **`(Wm-1) << 16 \| (Hm-1)`** — Wm in upper |
| `FE_INPUT_FORMAT` for NV12 | bits 0-2 vs bits 5-4 | **`0x10`** (subsampling code in bits 5:4) |
| `OP_GEOM` units | pixels or MCUs? | **MCUs** — pixels overflow the 26-bit mask |
| `PIPELINE_CFG` value for offline unrotated | this doc previously said `0x020065FB` (base + offline + clk-gate) | **`0x038061FB`** = base + offline + (mode==3)<<23. Bit 10 (clk-gate) is **0** on OPAL — earlier "always 1 for offline" claim was wrong. Bits 23-24 carry the mode_dims.mode value and were missing entirely from the earlier formula. Verified against OPAL `gemini_lib_hw_pipeline_cfg` C decompile + caller `gemini_lib_hw_config`, and against Samsung libgemini ARM disassembly (which agrees on the formula; Samsung's caller may set different bits but the OPAL caller does not). |
| Huffman tables for offline encode | "skip" or "load"? | **LOAD** universally |

## OP_MATRIX burst-mask preset table

Source: `.rodata` in libgemini (Samsung 0x4EBC, HTC 0x36CC, Sony 0x39CC,
Opal bundled). 84 bytes total = 28 dwords arranged as 9 columns × 3 rows
(actually padded to 9.33 rows in row-3 which duplicates row-1):

```
[0..8]   Row sl=0 (realtime path, mode_struct.dword[1] != 1):
         {0x00000002, 0x00000301, 0x00F0000F, 0, 0, 0, 0x0C0C0303, 0xC0C03030, 0}

[9..17]  Row sl=1 (offline path, mode_dims.mode > 1):
         {0x00000303, 0x00F0000F, 0xF0000F00, 0, 0, 0x0C0C0303, 0xC0C03030, 0, 0}

[18..26] Row sl=2 (padding — same bytes as sl=1):
         {0x00000303, 0x00F0000F, 0xF0000F00, 0, 0, 0x0C0C0303, 0xC0C03030, 0, 0}
```

For our offline NV12 encode use **Row sl=1**.

## Implementation status (Linux 6.18 kernel driver)

This driver is `drivers/media/platform/qcom/gemini/{gemini.c,gemini_hw.c,gemini_jpeg.c}`.

**As of commit `c1a25bc002fd`**, the driver programs all universal
register values listed above:

- ✅ All FE/OP/WE/PIPELINE register writes correct
- ✅ Quant tables loaded in natural order via TABLE_SEL=1/2 with
  IJG quality scaling
- ✅ Huffman tables loaded via TABLE_SEL=5/6 from JPEG Annex K standard
- ✅ JPEG marker preamble (SOI/JFIF/DQT/DHT/SOF0/SOS) emitted in
  software before hardware entropy stream
- ✅ EOI marker appended after FRAMEDONE
- ⏳ Awaits runtime verification — pending build/deploy/test cycle

## References

- `drivers/media/platform/qcom/gemini/gemini_hw.h` — register defines
- `drivers/media/platform/qcom/gemini/gemini_hw.c` — programming sequences
- `drivers/media/platform/qcom/gemini/gemini_jpeg.c` — JPEG header builder + standard tables
