# Gemini JPEG Encoder Debug — Status for Gemini AI (2026-04-28 update)

**Hardware:** Qualcomm MSM8660/APQ8060 Gemini JPEG IP at phys
`0x04600000`, HP TouchPad. Mainline kernel 6.18 V4L2 m2m driver at
`drivers/media/platform/qcom/gemini/`. Branch
`tenderloin/6.18/upstream-patches`. Latest HEAD: `1681f802b5a6`.

**Test:** `test_gemini` userspace tool feeds NV12 frame to `/dev/video6`,
gets JPEG out. Tested at 320×240, 640×480, 1280×1024 with synthetic
patterns: uniform Y (0/64/128/192/255 with UV=128) and XOR
(`p[y*W+x] = (uint8_t)(x^y)` with UV=128).

## Recent Breakthrough — DC encoding restored

**Bug:** Encoder was producing DC delta = 0 for every 8x8 block
regardless of input pixel value. Verified by parsing the entropy
stream: every flat-Y test JPEG started with `00…` (DC luma cat-0
Huffman code = delta 0). All flat inputs decoded to ~127 mid-gray. AC
content varied (XOR gave 68KB), but DC was zero everywhere.

**Root cause:** `OP_GEOM[0]` and `OP_GEOM[1]` had their `*128` and
`*256` multipliers swapped versus the live OPAL register trace at
`reports/opal-camera/tenderloin-live-traces/FINDINGS.md` (captured by
`tools/gemini_reg_poll/gemini_reg_poll` running on stock webOS during
a real still capture).

OPAL writes (verified at 1024×1280 portrait, Wm=64, Hm=80):
```
OP_GEOM[0] = Hm * (Wm-1) * 128
OP_GEOM[1] = Hm * (Wm-1) * 256
OP_GEOM[2] = Hm * (Wm-1) * 256 + 16
OP_GEOM[3] = Hm * (Wm-1) * 128 + 16
```

Mainline was writing `*256` for slot[0] and `*128` for slot[1]. Commit
`cc5a367969b9` swapped them back to OPAL ordering. After fix, file
sizes jumped from 5KB → 28KB at 640×480 for a flat input — actual
content is now being encoded. Live readback at 640×480 confirmed:
`OP_GEOM[0..3] = 0x00024900 0x00049200 0x00049210 0x00024910`.

## Residual Bug — "Bottom 20% is top 80% shifted right"

User-reported visual artifact, **identical at all three resolutions**:

> Bottom 20% of decoded image has the same data as top 80% rows, just
> shifted to the right.

Numerical analysis of the decoded XOR-pattern JPEG at 640×480:

- Top 80% (rows 0..383): ~330–380 of 640 cols are non-background per
  row, XOR pattern visible throughout the row.
- Anomaly at rows 352–360 (74–75% mark): ~580 active cols (more than
  the typical ~350).
- Bottom 20% (rows 384..479): per-row active region collapses to mostly
  cols 0..7, then mostly background (luma ≈ 79), with a few scattered
  short runs further right.

The split is sharp at exactly 80% / 20%:

- 320×240  → boundary at row 192 (= 0.8 × 240)
- 640×480  → boundary at row 384 (= 0.8 × 480)
- 1280×1024 → boundary at row 819 (≈ 0.8 × 1024)

That's an exact 1.25× ratio — the encoder is **consuming input data
1.25× too fast** somewhere, exhausting its read region by 80% of MCU
rows and producing garbage for the rest.

Color is also wrong for flat-Y inputs (chroma seems off — likely
related to or downstream of the same bug):

| Y_in | Decoded RGB (320, 640, 1280 all match) |
|------|---|
| 0    | (≈8, ≈245, ≈9) bright green |
| 64   | (≈234, ≈207, ≈244) light pink/lavender |
| 128  | (≈13, 0, ≈20) near-black |
| 192  | (0, ≈188, 0) pure green |
| 255  | (≈13, 0, ≈20) near-black (same as Y=128!) |

Note: Y=128 and Y=255 give **identical** decoded values — strongly
suggests sign-extension or 8-bit signed wrap somewhere in the
input/level-shift path.

## What we tried and ruled out

1. **`q_luma[0] = 1`** (force no DC quantization): bug persists.
   Confirms it's not a quant magnitude issue.
2. **`CBCR_MCU_ROWS = Hm/2 - 1`** (commit `5305bf4c8b1e`, reverted as
   `1681f802b5a6`): encoder hangs (FRAMEDONE never fires, select
   timeout) at every resolution. Confirms OPAL's "same value in Y and
   CBCR fields" rule: both `FE_BUFFER_CFG` fields must be `Hm-1`.
3. **`FE_PONG = PING`** (mirror, commit `a3fe99ab2620`): no effect on
   the 80/20 artifact. PONG = 0 (zeroed) gave the same result.
4. **OPAL has `gemini_hw_readback_quant_tables` after writing the quant
   table** — implemented and confirmed to fire (128 entries readback);
   no behavioural change.

## Confirmed register state during encode (640×480, current HEAD)

```
PIPELINE_CFG    = 0x038061fb     ← matches OPAL
FE_INPUT_FMT    = 0x00000010     ← matches OPAL ("0x10" = camera-path NV12 in)
FE_DIMS         = 0x0027001D     ← (Wm-1)<<16 | (Hm-1) = (39<<16)|29
FE_PIPELINE     = 0x00000203     ← matches OPAL
OP_ENC_MODE     = 0x00000003     ← matches OPAL
OP_GEOM[0..3]   = 0x24900 0x49200 0x49210 0x24910  ← matches OPAL
OP_FORMAT_MAGIC = 0x03381801     ← matches OPAL (H2V2 / 4:2:0)
OP_MATRIX[0..8] = 0x303 0xF0000F 0xF0000F00 0 0 0xC0C0303 0xC0C03030 0 0
                                  ← matches OPAL exactly
WE_CFG          = 0x00000020     ← matches OPAL
WE_Y_PING_ADDR  = 0x7c500270     ← dst buffer + JPEG header offset
WE_Y_PING_CFG   = 0x00095d8e     ← 613774 byte limit
WE_Y_UB_CFG     = 0x01ff0000     ← matches OPAL
WE_Y_THRESHOLD  = 0x016A0190     ← matches OPAL (low=0x16A=362, hi=0x190=400)
WE_CBCR_THRESH  = 0x016A0190     ← matches OPAL
FE_BUFFER_CFG   = 0x001D001D     ← (rows-1) in both Y and CBCR fields
FE_Y_PING       = 0x7c180000     ← src_y dma_addr
FE_CBCR_PING    = 0x7c1cb000     ← src_y + W*H = 0x7c180000 + 0x4b000
DRI_INTERVAL    = 0x00000000     ← no restart markers
```

Address arithmetic: `cbcr - y = 0x4b000 = 307200 = 640*480`. Correct
NV12 layout (Y plane then CbCr plane, single contiguous buffer).

## Theory space — what's left

**The 80/20 split is mathematically exact** — a 1.25× ratio. What
register or buffer-size mismatch produces 1.25× input consumption?

A. **Stride wrong by 25%** — encoder reads at stride `W * 1.25`,
   exhausting the `W*H` Y plane in `0.8 * H` rows. But what register
   controls input stride? FE_DIMS only encodes MCU dims; no explicit
   stride register has been identified in the decoded register set.

B. **Row count off by 25%** — encoder thinks there are `1.25 * Hm` MCU
   rows. But we wrote `Hm-1=29` in FE_DIMS lower 16 and FE_BUFFER_CFG
   both fields; live readback confirms.

C. **Chroma 4:2:2 vs 4:2:0 confusion** — for a 4:2:2 input with the
   same apparent W,H, the encoder reads 2× more chroma data. But
   `OP_FORMAT_MAGIC = 0x03381801` matches OPAL's H2V2 (4:2:0) magic;
   we're not in 4:2:2 mode.

D. **WE buffer threshold premature swap** — `WE_Y_THRESHOLD = 0x016A0190`
   means upper threshold = `0x190 = 400`, lower = `0x16A = 362`.
   Suspiciously close to 80% of 480, but these are byte-level FIFO
   thresholds, not row counts. Worth investigating whether they scale
   per encode size or are fixed.

E. **MCU column count interpretation** — for `Wm = 40` cols, encoder
   reads `40 * 16 = 640` Y bytes per row. If encoder mistakenly reads
   `40 * 16 + 40 * 4 = 800` bytes/row (adding 4 chroma bytes per MCU
   somehow), that's stride 1.25× — exhausts buffer at 80%! This is the
   leading hypothesis but requires understanding **why** the encoder
   would add 25%.

F. **`FE_INPUT_FORMAT = 0x10` semantics wrong** — bit 4 alone. Maybe
   bit 4 = "interleaved chroma" vs "planar chroma" toggled the wrong
   way for a `mmap`-fed test buffer. OPAL trace confirms `0x10` but
   OPAL was tracing the camera-VFE-output path, which may differ
   subtly from our `test_gemini` `mmap` path.

G. **PIPELINE_CFG bit field misinterpreted** — `0x038061FB` includes
   `0x02000000` (bit 25 = offline branch) and `0x01800000` (bits 23-24
   = mode value 3). Could one of the smaller bits in `0x000061FB`
   actually be selecting a 1.25×-stride mode that we leave on
   unconditionally?

## Files of interest

- `drivers/media/platform/qcom/gemini/gemini.c` — V4L2 m2m driver,
  device_run, IRQ handler
- `drivers/media/platform/qcom/gemini/gemini_hw.c` — Hardware
  programming helpers (`gemini_hw_set_fe_ping`,
  `gemini_hw_configure_encode_h2v2`, `gemini_hw_load_quant_table`,
  `gemini_hw_load_huffman_tables`, `gemini_hw_start_offline`)
- `drivers/media/platform/qcom/gemini/gemini_hw.h` — Register offsets,
  masks, magic constants
- `drivers/media/platform/qcom/gemini/gemini_jpeg.c` — JFIF builder,
  Annex K quant tables, standard Huffman tables
- `reports/opal-camera/tenderloin-live-traces/FINDINGS.md` — live OPAL
  register trace (ground truth)
- `tools/gemini_reg_poll/` — userspace register pollers (used to
  capture the OPAL trace on stock webOS)
- `test_gemini.c` (in repo root) — userspace V4L2 m2m test client

## Question to AI

What stride / buffer / row-count register or field in this
MSM8660/APQ8060 Gemini IP could plausibly cause **exactly 1.25× input
consumption rate** (producing the 80/20 wraparound)?

Specifically:

1. Is there a known register, format flag, or sequence on this IP where
   the encoder reads `W * 5/4` bytes per Y row instead of `W` — possibly
   because of a chroma-interleaving misinterpretation ("read NV12 Y
   plane plus interleaved chroma at +25% per row")?
2. Could it be that `FE_INPUT_FORMAT = 0x10` selects a packed
   "5-bytes-per-4-pixels" or "YYYYU" or similar non-standard
   subsampling where the buffer must be laid out at 1.25× stride?
3. Is the camera path (which is what OPAL traces) feeding the encoder
   directly from a VFE rotator output that happens to use a different
   in-memory format vs userspace `mmap` of plain NV12?
4. The fact that **Y=128 and Y=255 decode identically** points at a
   level-shift / sign-extension bug — does this IP have an "input is
   signed int8" mode that we may have left on, where unsigned `0x80`
   wraps to `-128 == 0x80` in level-shifted form?
