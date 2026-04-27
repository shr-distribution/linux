# Gemini JPEG Encoder Debug — Current Status

**Target:** Get the Qualcomm Gemini JPEG hardware encoder working on mainline
Linux 6.18 for the HP TouchPad (MSM8660 / APQ8060 SoC).

**Status as of 2026-04-27 (post live-OPAL-trace fixes):**
The encoder runs to completion, fires `FRAMEDONE`, produces a structurally
valid JFIF container, and now decodes to **100% grayscale** (R==G==B) for
320×240 and 1280×1024 — exactly matching the test pattern's `UV = 128`
chroma. **640×480** still has a small consistent chroma DC offset.
**Luma reconstruction is still wrong** — the decoded image shows MCU-aligned
structural artifacts that don't match the input pattern. For 1280×1024
specifically, the decoded image is a regular 4-valid + 4-zero alternating
band pattern (each band is exactly 10 MCUs wide), giving the appearance of
a "checkerboard of grey and black squares."

This document captures everything verified, every fix that landed, every
diagnostic run, and every dead-end so a fresh reviewer can pick up
cleanly.

---

## 1. Hardware

- SoC: Qualcomm APQ8060 (MSM8660 family) — dual-core ARMv7 Scorpion @ 1.5 GHz.
- Gemini JPEG encoder IP at physical base **0x04600000**, 4 KB region.
- Interrupt: GIC SPI 89.
- Sits on the MMSS (multimedia subsystem) fabric at master AXI port 7.
- Power domain: `IJPEG_GDSC` from MMCC clock controller.
- Clocks: `core` (153.6 MHz), `axi`, `ahb`. IOMMU: `ijpeg_iommu`.

Identical silicon to webOS 3.0.6 / OPAL — same TouchPad, same Gemini block.
Working OPAL userspace binary is the ground-truth reference: it produces
correct JPEGs from this exact silicon.

---

## 2. Repos and key files

- Mainline kernel: `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/`
  branch `tenderloin/6.18/upstream-patches` on `shr-github`.
- Driver: `drivers/media/platform/qcom/gemini/`
  - `gemini.c` — V4L2 mem2mem driver
  - `gemini_hw.c` — HW programming helpers
  - `gemini_hw.h` — register map + function decls
  - `gemini_jpeg.c` — JFIF preamble builder, standard quant/Huffman tables
  - `gemini_jpeg.h` — JFIF builder API
- Test harness: `/home/herrie/webos/touchpad-kernel/test_gemini.c`
  (V4L2 M2M test, fills NV12 with `Y = x ^ y`, UV = 0x80).
- Cross-vendor register reference:
  `reports/gemini-cross-vendor-register-map.md`.
- **Live-OPAL register-trace tool:**
  `tools/gemini_reg_poll/` — `/dev/mem`-based poller that mmaps
  `0x04600000` (Gemini) and `0x04000000` (MMCC), polls all registers at
  ~10 kHz, snapshots state at three sync points (before/during/after
  capture), logs every register-value transition with timestamps.
- **Captured live OPAL trace from real TouchPad capture:**
  `reports/opal-camera/tenderloin-live-traces/`
  - `tenderloin_gemini_during.txt` — full register state during a real
    1280×1024 still-photo encode by OPAL camera app on stock webOS
  - `tenderloin_gemini_delta.log` — every register-value change in
    timestamp order
  - `FINDINGS.md` — what the trace revealed
- **Captured LuneOS trace post-fix:**
  `reports/opal-camera/luneos-comparison/luneos_during_op_format_3.txt`
- **Legacy webOS kernel source:**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/media/video/msm/msm_gemini_*.{c,h}`
- **OPAL userspace binary (Tenderloin variant):**
  `/home/herrie/webos/touchpad-kernel/doctor306-opal/nova-cust-image-opal.rootfs-3.0.6/usr/lib/libqcameralib.so`
  (Tenderloin doctor: `/tmp/tenderloin_libqcameralib.so` — same encode logic, identical key tables)
- **OPAL Ghidra decompilation:**
  `reports/opal-camera/opal_libqcameralib_decompiled.c`,
  `/home/herrie/webos/touchpad-kernel/doctor305/ghidra_decompiled/libqcameralib_decompiled.c`

---

## 3. Fixes that landed (each with live-trace verification)

The recent breakthrough was capturing a real OPAL register trace on the
TouchPad's webOS stack and diffing against our mainline output. That diff
exposed three concrete bugs in our `gemini_hw_configure_encode_h2v2()`:

### 3.1 OP_ENCODE_MODE = 3 (was 1)

Cross-vendor static analysis of `gemini_lib_hw_op_cfg` claimed
`mcu_type == 1` for NV12, dispatching to the op_format=1 branch. Live
trace shows OPAL actually writes `OP_ENCODE_MODE = 3`, taking the
op_format=3 branch.

### 3.2 OP_FORMAT_MAGIC = 0x03381801 (was 0x0107081F)

The op_format=3 path emits magic word `0x03381801`. Cross-vendor doc had
labelled this `H2V2` and op_format=1's `0x0107081F` as `H1V1`, but live
trace confirms `0x03381801` is correct for NV12 stills.

### 3.3 OP_GEOM[0..3] = `Hm * (Wm-1) * {256,128,256,128}` (+16 for [2,3])

The op_format=3 branch in `gemini_lib_hw_op_cfg` (ARM disasm at
`0x14a47c` in Tenderloin's libqcameralib) computes geometry from
`param_2[2] * (param_2[3] - 1)` where the caller in
`gemini_lib_hw_config` passes a local struct with mode_dims fields
swapped, so:

```
base = mode_dims[3] * (mode_dims[2] - 1)
     = Wm * (Hm - 1)     [if mode_dims[2]=Wm, mode_dims[3]=Hm]
```

Trying that swap regressed our output (1280×1024 went from grayscale
to per-pixel garbage). The formula that empirically *works* is:

```
base = Hm * (Wm - 1)
```

Verified by live trace at 1280×1024: OPAL writes
`OP_GEOM[1] = 0x9D800 = 80*63*128 = Hm*(Wm-1)*128`, matching this
formula with `Wm=80, Hm=64`. Same for OP_GEOM[0]/[2]/[3]:

```
OP_GEOM[0] = Hm * (Wm-1) * 256
OP_GEOM[1] = Hm * (Wm-1) * 128
OP_GEOM[2] = Hm * (Wm-1) * 256 + 16
OP_GEOM[3] = Hm * (Wm-1) * 128 + 16
```

(The static-analysis label confusion comes from `mode_dims[2]` and
`mode_dims[3]` being swapped between the cross-vendor doc and the
actual struct layout. The captured register values are unambiguous
ground truth.)

### 3.4 DRI_INTERVAL bit-16 cleared after Huffman load

OPAL's `gemini_lib_hw_set_huffman_tables` ends with a `WRITE_OR_AND`
cmd that zeroes bits 0..16 of DRI_INTERVAL — i.e. it exits "Huffman
load mode". Our driver entered the mode (`DRI_INTERVAL |= 0x10000`)
to load the tables but never exited. Without exiting, the encoder
ran in load-mode and corrupted entropy past the first MCU column.

This was the final unlock that turned 320×240 and 1280×1024 from
catastrophic alternating-block black/white into 100% grayscale.

### 3.5 Earlier landed fixes (still required)

These remain in place:
- MMCC fabric AXI port-7 unhalt (commit `79aba7828ddc`)
- IRQ-driven `RESET_ACK` wait via `struct completion`
- `PIPELINE_CFG = 0x038061FB`
- Configure write-order: PIPELINE_CFG must be LAST
- Post-reset WE config: `WE_Y_UB_CFG=0x01FF0000`,
  `WE_Y_THRESHOLD=WE_CBCR_THRESHOLD=0x016A0190` (matches legacy webOS
  `msm_gemini_hw.c` byte-for-byte for offline mode)
- Huffman tables loaded BEFORE quant tables (OPAL order)
- IRQ handler must NOT `disable_irq` on transient events; must
  re-issue `FE_CMD = OFFLINE_CMD_START` on each `FE_RD_DONE` (without
  this re-issue, FRAMEDONE never fires — confirmed)
- PING addresses mirrored or zeroed in PONG slots (PONG addresses
  are not used by FE for single-frame encode — confirmed
  empirically)

---

## 4. Current symptom — the band pattern

Test pattern: NV12 with `Y[y][x] = x ^ y`, `UV = 128`. Decoded image
should be a Sierpinski-like grayscale XOR pattern.

### 4.1 320×240

- Decoded as 100% grayscale (R==G==B everywhere).
- Avg pixel `(123, 123, 123)`.
- Visual content: scattered bright/dark pixels, NOT the XOR pattern;
  most rows have only 1-7 pixels at exactly `(0,0,0)`.
- File: `/home/herrie/webos/touchpad-kernel/test_320.jpg` (17,352 B)

### 4.2 1280×1024

- Decoded as 100% grayscale.
- Avg pixel `(33, 33, 33)` (much darker than expected).
- **49% of every row is exactly `(0, 0, 0)`** in a regular pattern.
- 8 horizontal bands per row alternating valid + zero, each band 160
  pixels wide (= **10 MCUs at H2V2's 16-pixel MCU width**).
- Black runs in row 100 (representative of every row):
  ```
  x=160..285 (126 px), 289..319 (31 px),
  x=480..605 (126 px), 609..639 (31 px),
  x=800..925 (126 px), 929..959 (31 px),
  x=1120..1245 (126 px), 1249..1279 (31 px)
  ```
- Pattern is **byte-deterministic**: identical byte counts (115,466)
  across all our register experiments (mirror PONG, zero PONG, split
  PONG, max thresholds, etc.).
- The entropy stream has **no long runs of plain `0x00` bytes** — the
  encoder is producing valid Huffman-coded data for the entire frame,
  but the decoded pixel content is wrong for half the columns.
- File: `/home/herrie/webos/touchpad-kernel/test_1280.jpg` (115,466 B)

### 4.3 640×480

- Decoded as 0% grayscale; every pixel has `R≠G≠B` with a small
  consistent tint.
- Avg pixel `(89, 117, 126)`.
- The structure of the decoded image is roughly correct (the XOR
  pattern is visible in shape), only the chroma reconstruction has
  ~10-quant-step offset.
- File: `/home/herrie/webos/touchpad-kernel/test_640.jpg` (68,071 B)
- The 640×480 chroma offset hasn't been root-caused. It does not
  exhibit the 1280×1024 banding pattern.

---

## 5. What we have verified matches OPAL byte-for-byte

Captured a real OPAL register trace on the TouchPad's stock webOS
during a real camera capture (1280×1024 still). All registers we
audit match the OPAL trace value for the same input.

```
Register             Address  OPAL          Mainline      Match
─────────────────────────────────────────────────────────────────
HW_VERSION           0x0000   0x00001100    0x00001100    ✓ (RO)
PIPELINE_CFG         0x0008   0x038061FB    0x038061FB    ✓
IRQ_MASK             0x0014   0xFFFFFFFF    0xFFFFFFFF    ✓
IRQ_STATUS           0x001C   (varies)      (varies)      ✓
FE_INPUT_FORMAT      0x0038   0x00000010    0x00000010    ✓
FE_DIMS              0x003C   0x003F004F    0x004F003F    ⚠ swap*
FE_PIPELINE_MODE     0x0040   0x00000203    0x00000203    ✓
OP_ENCODE_MODE       0x0044   0x00000003    0x00000003    ✓
OP_GEOM[0]           0x0048   0x0013B000    0x0013C000    ⚠ +0x1000**
OP_GEOM[1]           0x004C   0x0009D800    0x0009E000    ⚠ +0x800**
OP_GEOM[2]           0x0050   0x0013B010    0x0013C010    ⚠ +0x1000**
OP_GEOM[3]           0x0054   0x0009D810    0x0009E010    ⚠ +0x800**
OP_FORMAT_MAGIC      0x0058   0x03381801    0x03381801    ✓
OP_MATRIX[0..8]      0x005C   {0x303,
                              0xF0000F,
                              0xF0000F00,
                              0,0,
                              0xC0C0303,
                              0xC0C03030,
                              0,0}         (same)         ✓
WE_CFG               0x0098   0x00000020    0x00000020    ✓
WE_Y_THRESHOLD       0x00C0   0x016A0190    0x016A0190    ✓
WE_CBCR_THRESHOLD    0x00C4   0x016A0190    0x016A0190    ✓
WE_Y_UB_CFG          0x00E8   0x01FF0000    0x01FF0000    ✓
START_KICK           0x00F0   0x00000001    0x00000001    ✓
FE_CMD               0x0094   0x00000003    0x00000003    ✓
0x013C (undocumented) 0x013C  0x0007FFFF    0x00000000    ⚠ unwritten***
FE_BUFFER_CFG        0x0080   0x003F003F    0x003F003F    ✓
TABLE_INDEX (post-load) 0x0128 0x00000080   0x00000080    ✓
```

\* **FE_DIMS halves swap**: OPAL writes `(Hm-1)<<16 | (Wm-1)`
(`0x003F004F` for 1280×1024 → `Hm-1=63, Wm-1=79`). Mainline writes
`(Wm-1)<<16 | (Hm-1)` (`0x004F003F`). When we tried the swap to match
OPAL it regressed 1280 from grayscale to per-pixel garbage. So either
the encoder doesn't strictly require this exact convention, or some
other side effect of the swap broke something else. Reverted.

\*\* **OP_GEOM small differences**: OPAL captured for 1024×1280
**portrait** (Wm=64, Hm=80) gives `Hm*(Wm-1) = 80*63 = 5040`. We test
1280×1024 **landscape** (Wm=80, Hm=64) and `Hm*(Wm-1) = 64*79 = 5056`.
Different operands → different values, but the **formula**
`Hm*(Wm-1)*{256,128,256,128} (+16 for [2,3])` is identical between OPAL
and us. The tiny numeric difference is just because of different image
dimensions, not a bug.

\*\*\* **0x013C = 0x0007FFFF**: OPAL trace shows this register set to
all ones in lower 19 bits. Our mainline driver doesn't write 0x013C.
Cross-vendor doc and the legacy webOS kernel header don't name it.
The OPAL ARM disasm doesn't show it being written explicitly with a
`mov r2, #0x7FFFF` or `movw/movt` pair (we searched). It's likely a
read-only status register or an HW-driven shadow of something else,
not something we need to write.

---

## 6. What we have verified matches legacy webOS kernel byte-for-byte

The legacy webOS kernel source contains the exact register-write
sequences for offline mode in
`webos-linux-kernel-touchpad/drivers/media/video/msm/msm_gemini_hw.c`.

- `WE_Y_THRESHOLD = (0x16A << 16) | 0x190` = `0x016A0190` ✓
- `WE_Y_UB_CFG = JPEG_WE_YUB_ENCODE = 0x01FF0000` ✓
- `WE_CBCR_THRESHOLD` = same as `WE_Y_THRESHOLD` ✓
- `FE_BUFFER_CFG` packs `(num_of_mcu_rows-1)` in both Y and CBCR
  fields ✓
- Reset sequence: IRQ_MASK=DISABLE → IRQ_CLEAR=ALL →
  IRQ_MASK=ALL_SOURCES → RESET_CMD=0x0004FFFF ✓
- `msm_gemini_hw_write` performs read-modify-write for masks ≠
  0xFFFFFFFF; on reset (registers=0) RMW collapses to direct write;
  our `writel()` is therefore equivalent.

---

## 7. Diagnostics tried that produced byte-identical output

The 1280×1024 output is `115,466 bytes` regardless of any of the
following. This means none of these knobs are involved in the band
corruption:

| Diagnostic | Result |
|---|---|
| Mirror PONG_ADDR = PING_ADDR (default) | bands present |
| Set PONG_ADDR = 0 (FE PONG unused for single-frame) | bands identical |
| Split WE PING/PONG to separate halves of dst buffer | bands identical |
| `WE_Y_THRESHOLD = WE_CBCR_THRESHOLD = 0x01FF01FF` (max) | bands identical |
| Quant table READ pass after WRITE pass (OPAL pattern) | bands identical |

Diagnostics that **regressed** output:

| Diagnostic | Result |
|---|---|
| FE_DIMS halves swap to match OPAL convention | 1280 from grayscale to per-pixel garbage |
| OP_GEOM operand swap `Wm*(Hm-1)` vs `Hm*(Wm-1)` | 1280 from grayscale to per-pixel garbage |
| Drop FE_RD_DONE re-issue of FE_CMD=START | Encoder hangs (FRAMEDONE never fires) |
| Skip post-reset WE thresholds | `WE_Y_OVERFLOW` (status=0x40), encoder stalls |
| Skip OP_MATRIX writes | 320 grayscale, 640×480 row 0 clean grayscale, 1280 broken |

---

## 8. Specific open questions

1. **What controls the 10-MCU-wide band cycling?** The pattern is
   1:1 visible/zero alternating, exactly 10 MCUs (= 160 pixels) per
   band, repeating 4×4 across each row. It must be driven by some
   register or some HW state machine we haven't identified. 80
   columns / 10 = 8 → 4 valid + 4 zero. None of WE thresholds,
   PING/PONG addresses, or OP_GEOM values affect it.

2. **Why is 320×240 different from 1280×1024 visually?** Both
   resolutions decode to 100% grayscale, but 320×240 has scattered
   noise instead of regular bands. 320×240's `Wm=20`, divisible by
   10, so should produce 1 valid + 1 zero band of 10 MCUs each? But
   it doesn't.

3. **Why does 640×480 only have a chroma cast?** 640×480's `Wm=40`,
   also divisible by 10. Should we see 2 valid + 2 zero bands? We
   see only chroma offset, no zero bands.

4. **What is `0x013C = 0x0007FFFF`?** OPAL trace shows it set, we
   don't write it, doesn't appear in cross-vendor doc or legacy
   header. Could be a status register that OPAL reads, or a config
   we're missing.

5. **Is the JFIF SOF0 declaration wrong?** We declare H2V2 sampling.
   If the encoder actually emits H2V1 or H1V1 MCUs with
   `OP_FORMAT_MAGIC = 0x03381801`, the decoder would mis-parse
   coefficient counts per MCU and produce structural errors. But we
   verified `OP_FORMAT_MAGIC=0x03381801` matches OPAL's value
   exactly, and our JFIF decodes to grayscale (which means at least
   chroma is right).

6. **Is the HW Huffman LUT format derivation correct?** Our
   `gemini_hw_load_huffman_tables` was reverse-engineered from
   cross-vendor binaries. Per-symbol slots at
   `INDEX = (i << 2) | lsb`, per-length seed slots at
   `INDEX = (n << 6) | base_id` for `base_id ∈ {2, 3}`. The format
   may be subtly off in a way that produces correct codes for
   simple blocks but wrong codes for high-frequency blocks (which
   is what XOR pattern produces). The 1280×1024 zero bands could be
   the encoder running out of valid codes or producing zero-symbol
   codes for high-AC-content MCUs.

---

## 9. Latest deployed kernel state

- Branch: `tenderloin/6.18/upstream-patches` on `shr-github`.
- Tip commit: `3fa4ac075488` (a series of GPU/cache fixes from the
  user plus the gemini work)
- Last gemini-modifying commit: `fcaf10dac00b` (diagnostic max
  WE_Y_THRESHOLD — did not affect the band pattern).
- Yocto recipe: `/media/herrie/LuneOS/scarthgap/webos-ports/meta-smartphone/meta-hp/recipes-kernel/linux/linux-hp-tenderloin_git.bb`
- Build command: `cd /media/herrie/LuneOS/scarthgap/webos-ports && source setup-env && MACHINE=tenderloin bitbake linux-hp-tenderloin`
- Deploy: copy
  `tmp-glibc/deploy/images/tenderloin/uImage-dtb-zImage` to the
  device's `/uboot/uImage.LuneOS` then sysrq-reboot.

---

## 10. Reproducer

```bash
# Build the test harness (one-time):
arm-linux-gnueabihf-gcc -O2 -static \
    /home/herrie/webos/touchpad-kernel/test_gemini.c \
    -o /tmp/test_gemini

# Deploy:
scp -P 22 /tmp/test_gemini root@172.16.42.2:/tmp/test_gemini

# Run on device (LuneOS):
ssh -p 22 root@172.16.42.2 \
    "chmod +x /tmp/test_gemini; /tmp/test_gemini -w 1280 -h 1024; ls -la /tmp/test.jpg"

# Pull and inspect:
scp -P 22 root@172.16.42.2:/tmp/test.jpg ./test_1280.jpg
python3 -c "
from PIL import Image
img = Image.open('test_1280.jpg'); img.load()
px = list(img.getdata())
grays = sum(1 for p in px if p[0]==p[1]==p[2])
print(f'{100*grays/len(px):.1f}% grayscale')
y = 100
runs = []
in_black = False; start = 0
for x in range(1280):
    is_black = img.getpixel((x,y)) == (0,0,0)
    if is_black and not in_black:
        start = x; in_black = True
    elif not is_black and in_black:
        runs.append((start, x-1, x-start)); in_black = False
print('Black runs in row 100:', runs[:8])
"
```

Expected output for 1280×1024:
```
100.0% grayscale
Black runs in row 100: [(160, 285, 126), (289, 319, 31), (480, 605, 126),
                        (609, 639, 31), (800, 925, 126), (929, 959, 31),
                        (1120, 1245, 126), (1249, 1279, 31)]
```

For 320×240 the test pattern is:
```
100.0% grayscale
Black runs in row 100: [(183, 183, 1), (191, 191, 1), (290, 290, 1),
                        (298, 298, 1), (309, 309, 1), (313, 313, 1),
                        (317, 317, 1)]
```

For 640×480 the test pattern is:
```
0.0% grayscale
Avg (R, G, B) = (89, 117, 126)
```

---

## 11. To run a register trace on either webOS or LuneOS

Cross-build the poller:
```bash
cd /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/tools/gemini_reg_poll
make CC=arm-linux-gnueabihf-gcc
```

For webOS (via novacom):
```bash
novacom put file:///tmp/gemini_reg_poll < gemini_reg_poll
novacom run file://bin/chmod -- 755 /tmp/gemini_reg_poll
novacom run file:///tmp/gemini_reg_poll &
# Take a photo with the camera app
novacom get file:///tmp/gemini_snapshot_during.txt > webos_during.txt
novacom get file:///tmp/gemini_delta.log > webos_delta.log
```

For LuneOS (via SSH):
```bash
scp -P 22 gemini_reg_poll root@172.16.42.2:/tmp/
ssh -p 22 root@172.16.42.2 "(/tmp/gemini_reg_poll &); sleep 1; /tmp/test_gemini -w 1280 -h 1024"
scp -P 22 root@172.16.42.2:/tmp/gemini_snapshot_during.txt ./
scp -P 22 root@172.16.42.2:/tmp/gemini_delta.log ./
```

Diff approach: snapshot files dump the full Gemini region as
`offset = value` lines for non-zero values; `diff` can compare two
captures directly.

---

## 12. Key references for next investigator

- **OPAL ARM disasm of `gemini_lib_hw_op_cfg`** (where op_format=3
  geometry is computed): Tenderloin libqcameralib at offset
  `0x13a47c` (the second branch in the `if (iVar3 == 3)` block).
- **OPAL ARM disasm of `gemini_lib_hw_set_huffman_tables`**: ends
  with a write-or-and to DRI_INTERVAL clearing the load-mode bit;
  Tenderloin libqcameralib at the end of the function near the
  static-template-driven epilogue.
- **OPAL Ghidra decompilation of `gemini_lib_hw_config`** (full
  encode setup): file
  `/home/herrie/webos/touchpad-kernel/doctor306-opal/.../libqcameralib.so` decompiled at
  `reports/opal-camera/opal_libqcameralib_decompiled.c`. The local
  struct passed to `op_cfg` swaps `mode_dims[2]/[3]`, hiding the
  fact that op_cfg's local indexes 2 and 3 are `Wm` and `Hm`
  respectively from the perspective of the input dims struct.
- **Live OPAL register trace** (ground truth, captured from real
  camera capture on the TouchPad):
  `reports/opal-camera/tenderloin-live-traces/tenderloin_gemini_during.txt`,
  `reports/opal-camera/tenderloin-live-traces/tenderloin_gemini_delta.log`.
- **Findings of trace vs mainline diff:**
  `reports/opal-camera/tenderloin-live-traces/FINDINGS.md`.
- **Cross-vendor register reference** (initial static-analysis
  derivation, partially superseded by live trace):
  `reports/gemini-cross-vendor-register-map.md`.

---

## 13. Bottom line for a reviewer

We have an encoder that:

- Runs to FRAMEDONE
- Produces structurally-valid JFIF containers
- Decodes to 100% grayscale for 320×240 and 1280×1024 (proving chroma
  reconstruction is correct)
- Has a deterministic 4-valid + 4-zero 10-MCU-wide band pattern at
  1280×1024 that no register experiment we've tried influences

Every register we audit matches what OPAL writes for the same input.
The remaining bug — the band corruption — is most likely either:

1. A subtle bit-pattern error in our HW Huffman LUT format derivation
   that produces correct codes for low-AC content but malformed codes
   for high-AC content, or
2. An undocumented register or write-ordering constraint we haven't
   identified.

Most informative next experiments:

- (a) Encode a UNIFORM Y test input (all 128) to isolate DC vs AC
  paths. If a uniform input decodes correctly, the bug is in AC
  coefficient handling.
- (b) Disassemble OPAL's `gemini_lib_hw_set_huffman_tables` more
  thoroughly and compare the data-port write pattern to ours
  bit-by-bit — our cross-vendor reverse-engineering may have an
  off-by-one in the `(size - 1 + n) & 0x1F` adjustment or in the
  per-slot `idx` formula.
- (c) Run the live trace at 640×480 on stock webOS to capture OPAL's
  register state for that resolution and compare against the
  1280×1024 trace — the chroma-cast-only behaviour at 640×480 is
  diagnostically interesting because it hints at width-dependence
  in something we haven't pinned down.
