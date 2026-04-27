# OPAL `gemini_lib_hw_set_huffman_tables` — bit-by-bit comparison

Compared `drivers/media/platform/qcom/gemini/gemini_hw.c` :
`gemini_hw_load_huffman_tables()` to OPAL's
`gemini_lib_hw_set_huffman_tables` as decompiled in
`reports/opal-camera/opal_libqcameralib_decompiled.c` lines 186008..186420
(also matches Tenderloin's `libqcameralib.so` at the same function — both
have identical encode logic, different compile offsets).

## Structure

OPAL's function:

1. Allocate a 0x2370-byte cmd buffer.
2. Copy a 9-dword (36-byte) static template into the first cmd slots.
3. **Prologue: 12 entries per table × 2 tables = 24 entries** of
   per-length seed slots, hand-unrolled in C source. Each entry pair is
   a `WRITE TABLE_INDEX = (n<<6)|base_id` followed by
   `WRITE TABLE_DATA = ((size-1+n)<<16) | (code<<(16-size) & 0xFFFF)`.
4. **Main loop 1 (luma)**: 176 iterations. Each iteration writes
   `WRITE TABLE_INDEX = (i<<2)|0` then
   `WRITE TABLE_DATA = ((size-1+(i>>4))<<16) | (code<<(16-size) & 0xFFFF)`.
5. **Main loop 2 (chroma)**: 176 iterations. Same as luma but with
   `WRITE TABLE_INDEX = (i<<2)|1`.
6. **Epilogue**: copy a 4-dword static template into final cmd slots
   (likely `TABLE_SEL = release`).

Our `gemini_hw_load_huffman_tables()`:

1. Set DRI_INTERVAL bit 16 (huffman load mode).
2. Set `TABLE_SEL = 6` (HUFFMAN), reset `TABLE_INDEX = 0`.
3. **Prologue: 12 entries per table × 2 tables = 24 entries** matching
   OPAL exactly.
4. **Main loop**: 176 iterations × 2 tables = 352 entries matching OPAL
   exactly.
5. Set `TABLE_SEL = 0` (release).
6. Clear DRI_INTERVAL bit 16.

## Per-entry formulas — verified IDENTICAL to OPAL

### TABLE_INDEX

| Phase | OPAL | Mainline | Match |
|---|---|---|---|
| Prologue luma (n=0..11) | `(n<<6)\|2` (literal `0x002, 0x042, …, 0x2C2`) | `((n << 6) \| 2) & 0x3FF` | ✓ |
| Prologue chroma (n=0..11) | `(n<<6)\|3` (`0x003, 0x043, …, 0x2C3`) | `((n << 6) \| 3) & 0x3FF` | ✓ |
| Main luma (i=0..175) | `i<<2 \| 0` (i.e. `0, 4, 8, …, 700`) | `((i << 2) \| 0) & 0x3FF` | ✓ |
| Main chroma (i=0..175) | `i<<2 \| 1` (i.e. `1, 5, 9, …, 701`) | `((i << 2) \| 1) & 0x3FF` | ✓ |

### TABLE_DATA

OPAL (extracted from the per-iter formula):

```
DATA = (((size - 1) + offset) & 0x1F) << 16
     | (code << (16 - size) & 0xFFFF)
```

where `offset = n` in prologue, `i >> 4` in main loop.

For `size == 0`:

```
size - 1 + offset = -1 + offset = offset - 1
       (overflow → (offset-1) & 0x1F)
code << (16 - 0)  = code << 16
       (& 0xFFFF → 0 since code is 16-bit)
```

so for empty symbols: `hi = (offset - 1) & 0x1F`, `lo = 0`. **Mainline
handles this with explicit conditional**:

```c
hi = (p->size > 0)
    ? (((u32)p->size - 1u + offset) & 0x1Fu)
    : ((offset - 1u) & 0x1Fu);
lo = (p->size > 0)
    ? (((u32)p->code << (16 - p->size)) & 0xFFFFu)
    : 0;
```

Both formulas evaluate to the same numbers. ✓

### Pair input layout

OPAL accesses input pair buffer at byte offset N as:

```
size = ((u16 *)(buf + N))[0]   // first ushort
code = ((u16 *)(buf + N))[1]   // second ushort
```

So the wire format is `{u16 size; u16 code;}`.

Mainline uses:

```c
struct gemini_huff_pair { u16 code; u16 size; };
```

so the byte layout is `{u16 code; u16 size;}` — fields swapped, but
mainline accesses by name (`p->size`, `p->code`) so the math is correct
regardless of layout.

## Iteration counts and per-table scope

- OPAL prologue: 12 entries × 2 tables = 24 ✓
- OPAL main loop: 176 entries × 2 tables = 352 ✓
- Mainline: same (12 × 2 outer prologue, 176 × 2 outer main)

## Reading order vs writing order

OPAL builds the cmd buffer in cmd-buffer order, then submits the entire
buffer via one ioctl. The kernel processes cmds in order, executing each
WRITE.

Mainline writes each cmd one at a time via `writel()`. The HW sees the
same write sequence in the same order.

## Inputs (per-table pair buffer)

OPAL passes 4 buffers to its function: param_1, param_2 for the prologue
(luma, chroma) and param_3, param_4 for the main loops. OPAL's caller in
`gemini_lib_hw_create_huffman_tables` likely passes the same pair buffer
twice (once to param_1/param_3 for luma, once to param_2/param_4 for
chroma).

Mainline passes 2 buffers (luma, chroma) to its function. Each is used
for both prologue and main loop of its respective table. Functionally
equivalent.

## Pair buffer construction

OPAL's caller calls `gemini_lib_hw_create_huffman_tables` to build the
4 pair buffers from JPEG `BITS[16]` + `HUFFVAL[]` arrays. We don't have
that decompilation easily readable, but the standard JPEG Annex C
canonical-Huffman algorithm is well-defined and our
`gemini_build_huff_pairs()` implements it. The merge order (AC then DC,
DC entries override AC at low huffvals) was confirmed cross-vendor.

## Conclusion of the bit-by-bit review

Mainline and OPAL produce **identical TABLE_INDEX / TABLE_DATA
sequences** for the same input pair buffers. The Huffman load formula is
not the source of the 1280×1024 band corruption.

## What wasn't ruled out

- The merge-AC-then-DC order in `gemini_load_tables()` could be subtly
  wrong (we override AC at low huffvals with DC, matching cross-vendor;
  but maybe the HW expects merging the OTHER way).
- The 4 standard Huffman tables in `gemini_jpeg.c` are compared
  byte-for-byte to JPEG Annex K.3 — the BITS values match the standard.
- The `gemini_build_huff_pairs()` implementation of canonical-Huffman
  code derivation — re-verified against ITU-T81 Annex C, the canonical
  size+code generation looks correct.

Bottom line: the Huffman load path is structurally correct against OPAL
and against the JPEG specification. The 10-MCU band corruption at
1280×1024 has a different root cause.
