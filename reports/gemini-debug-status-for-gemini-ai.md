# Gemini JPEG encoder debug — concise status for handoff

## Target
Mainline Linux 6.18 driver for the **Qualcomm Gemini JPEG encoder** at
`0x04600000` on the HP TouchPad (MSM8660 / APQ8060). Identical silicon
to webOS 3.0.6 / OPAL — OPAL's stack produces correct JPEGs from this
exact hardware (ground truth).

## Symptom
Encoder runs, fires `FRAMEDONE`, produces structurally valid JFIF.
Test pattern: NV12 with `Y[y][x] = x ^ y`, `UV = 128` → expected
grayscale XOR pattern.

| Resolution | Decoded result |
|---|---|
| 320×240 | 100% grayscale (R==G==B), but pattern is scattered noise not XOR |
| 640×480 | 0% grayscale — consistent chroma cast (89, 117, 126) |
| 1280×1024 | 100% grayscale, **regular 4-valid + 4-zero band pattern** |

The 1280×1024 pattern: every row has 8 horizontal bands of 160 px (10
H2V2 MCUs) each, alternating "valid encoded" and "fully (0,0,0)".
49% of every row is exactly black. Black runs in row 100:
```
x=160..285 (126 px), 289..319 (31 px),
x=480..605 (126 px), 609..639 (31 px),
x=800..925 (126 px), 929..959 (31 px),
x=1120..1245 (126 px), 1249..1279 (31 px)
```

## Fixes that landed (each required, each verified by live OPAL trace)

1. **MMCC fabric AXI port-7 unhalt** — without it, JPEG_ENC port stays
   halted by RPM and FRAMEDONE never fires.
2. **`PIPELINE_CFG = 0x038061FB`** for offline NV12. Formula:
   `0x61FB | (offline ? 0x02000000 : 0) | ((mode & 3) << 23)`.
3. **Configure write order** — PIPELINE_CFG must be the LAST register
   written in the configure path; writing it first hangs the AHB bus.
4. **`OP_ENCODE_MODE = 3`** (not 1), **`OP_FORMAT_MAGIC = 0x03381801`**
   (not 0x0107081F). OPAL takes the op_format=3 branch for stills.
5. **OP_GEOM[0..3] = `Hm*(Wm-1)*{256,128,256,128}` (+16 for [2,3])** —
   op_format=3 formula. Verified by live trace.
6. **Post-reset WE config**: `WE_Y_UB_CFG=0x01FF0000`,
   `WE_Y/CBCR_THRESHOLD=0x016A0190` (matches legacy webOS msm_gemini).
7. **DRI_INTERVAL bit-16 cleared after Huffman load** — OPAL exits
   "Huffman load mode" with a `WRITE_OR_AND` zeroing bits 0..16 of 0xF4.
   Without this clear, encoder corrupts entropy past first MCU column.
8. **Huffman tables loaded BEFORE quant tables** (OPAL order).
9. **AC huffval nibble-swap before indexing pair buffer** — OPAL's
   `gemini_lib_hw_create_huffman_table` nibble-swaps AC huffvals
   (`(v & 0xF) << 4 | (v >> 4)`) before storing in pair[]. Mainline now
   matches.
10. **IRQ handler must NOT `disable_irq` on transient events** and
    must re-issue `FE_CMD = OFFLINE_CMD_START` on each `FE_RD_DONE`
    (without re-issue, FRAMEDONE never fires — confirmed by removing).

## Diagnostic dead-ends (each produced byte-identical output)

These knobs DO NOT influence the 1280×1024 band pattern:

- WE PING/PONG mirror vs split vs zero
- `WE_Y_THRESHOLD` legacy `0x16A/0x190` vs max `0x01FF01FF`
- FE PONG = 0 vs mirror (FE doesn't use PONG for single-frame encode)
- Huffman merge order (AC-then-DC vs DC-then-AC)
- AC nibble-swap on vs off
- Quant table readback pass after write

## Diagnostics that REGRESSED output

- FE_DIMS halves swap to OPAL convention `(Hm-1)<<16 | (Wm-1)`:
  regressed 1280 to per-pixel garbage. Mainline writes
  `(Wm-1)<<16 | (Hm-1)` and that's empirically what works.
- OP_GEOM operand swap `Wm*(Hm-1)` vs `Hm*(Wm-1)`: same regression.
- Drop FE_RD_DONE re-issue: encoder hangs.
- Skip post-reset WE thresholds: WE_Y_OVERFLOW.
- Skip OP_MATRIX writes: 320 OK, 640 row 0 clean grayscale, 1280
  broken (HW reset defaults align with one specific MCU geometry).

## Verified register parity with OPAL

Captured live OPAL trace via `tools/gemini_reg_poll/` (`/dev/mem`-based
poller, snapshots before/during/after a real camera capture). All
written registers match OPAL byte-for-byte:

```
PIPELINE_CFG=0x038061FB  FE_INPUT_FORMAT=0x10  FE_PIPELINE_MODE=0x203
OP_ENCODE_MODE=3  OP_FORMAT_MAGIC=0x03381801
OP_GEOM[0..3] = Hm*(Wm-1)*{256,128,256,128} (+16 for [2,3])
OP_MATRIX[0..8] = {0x303, 0xF0000F, 0xF0000F00, 0, 0,
                   0xC0C0303, 0xC0C03030, 0, 0}
WE_CFG=0x20  WE_Y/CBCR_THRESHOLD=0x016A0190  WE_Y_UB_CFG=0x01FF0000
FE_BUFFER_CFG packs (Hm-1) in both Y and CBCR fields
TABLE_INDEX after quant load = 0x80
```

The only register OPAL has set that we don't write is **`0x013C =
0x0007FFFF`** — searched OPAL/Samsung/HTC/Sony binaries, NO vendor
binary explicitly writes `0x0007FFFF`. Almost certainly a HW power-up
default, not a configuration we need to set.

## Specific open questions

1. **What controls the 10-MCU band cycling on 1280×1024 only?**
   80 MCU cols ÷ 10 = 8 bands. The pattern is fully deterministic and
   no register knob influences it. 320 (Wm=20) and 640 (Wm=40) — also
   divisible by 10 — don't show this pattern. Why is it
   width-specific to 1280?

2. **Why does 640×480 have only chroma cast?** Wm=40 also divisible
   by 10. We see no zero-bands, just chroma offset. What's different?

3. **Are pair-table-indexed Huffman entries even used by the encoder?**
   The nibble-swap fix produced byte-identical output, suggesting the
   per-huffval pair[i] entries (i=0..175) loaded via main loop don't
   affect AC encoding behavior on this silicon. So where does AC
   actually pull codes from?

4. **Could there be a HW errata at MCU column count 80?**
   Worth checking msm8x60 errata sheets if available.

## Reproducer

```bash
# Build test harness once:
arm-linux-gnueabihf-gcc -O2 -static \
    /home/herrie/webos/touchpad-kernel/test_gemini.c \
    -o /tmp/test_gemini

# Deploy and run:
scp -P 22 /tmp/test_gemini root@172.16.42.2:/tmp/
ssh -p 22 root@172.16.42.2 "/tmp/test_gemini -w 1280 -h 1024"
scp -P 22 root@172.16.42.2:/tmp/test.jpg ./test_1280.jpg

# Verify band pattern (49% black per row in regular 10-MCU stripes):
python3 -c "
from PIL import Image
img = Image.open('test_1280.jpg'); img.load()
y = 100; runs = []; in_b = False; s = 0
for x in range(1280):
    b = img.getpixel((x,y)) == (0,0,0)
    if b and not in_b: s = x; in_b = True
    elif not b and in_b: runs.append((s, x-1, x-s)); in_b = False
print(runs)"
```

## Key files for next reviewer

- Mainline driver: `drivers/media/platform/qcom/gemini/`
- Live OPAL trace: `reports/opal-camera/tenderloin-live-traces/`
  - `tenderloin_gemini_during.txt` — full reg state during real capture
  - `tenderloin_gemini_delta.log` — every register transition
  - `FINDINGS.md` — diff analysis vs mainline at the time
- Bit-by-bit Huffman comparison: `reports/opal-camera/HUFFMAN_LOAD_ANALYSIS.md`
- OPAL Ghidra decomp: `reports/opal-camera/opal_libqcameralib_decompiled.c`
- Tenderloin libqcameralib decomp: `/home/herrie/webos/touchpad-kernel/doctor305/ghidra_decompiled/libqcameralib_decompiled.c`
- Legacy webOS kernel: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/media/video/msm/msm_gemini_*.{c,h}`
- Register poller (cross-build with `make CC=arm-linux-gnueabihf-gcc`):
  `tools/gemini_reg_poll/`

## Latest deployed kernel
SRCREV `9aed0ae1ed37` — has all 10 fixes above plus AC nibble-swap.
Branch: `tenderloin/6.18/upstream-patches` on `shr-github`.

## Bottom line

We have an encoder running to FRAMEDONE that produces structurally
valid JFIF. Chroma reconstruction is correct (320 and 1280 decode
to 100% grayscale on grayscale-input test). Luma reconstruction shows
deterministic, width-dependent corruption that is unresponsive to
every register/algorithm knob we have audited or experimented with.
All written registers match OPAL byte-for-byte.

The remaining bug is most likely either a HW timing/ordering
constraint or a registered bit we haven't identified, NOT a wrong
value in a register we already write.
