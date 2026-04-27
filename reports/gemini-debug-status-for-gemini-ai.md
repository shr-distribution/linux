# Gemini JPEG Encoder Debug — Current Status

**Target:** Get the Qualcomm Gemini JPEG hardware encoder working on mainline
Linux 6.18 for the HP TouchPad (MSM8660 / APQ8060 SoC).

**Status as of 2026-04-27:** Encoder programs cleanly, fires `FRAMEDONE`,
produces a structurally valid JFIF container of plausible size — but the
decoded pixel content is **wrong**. The corruption pattern is per-8x8-block
saturation: most pixels grayscale, scattered blocks fully `(0,0,~200)` or
`(255,255,255)`. Same garbage appears at every tested resolution.

This document captures everything we have verified, every fix that landed,
and every diagnostic we've run, so a fresh pair of eyes can pick up cleanly.

---

## 1. Hardware

- SoC: Qualcomm APQ8060 (MSM8660 family) — dual-core ARMv7 Scorpion @ 1.5 GHz.
- Gemini JPEG encoder IP at physical base **0x04600000**, 4 KB region.
- Interrupt: GIC SPI 89 (`interrupts = <0 89 IRQ_TYPE_LEVEL_HIGH>`).
- Sits on the MMSS (multimedia subsystem) fabric at master AXI port 7.
- Power domain: `IJPEG_GDSC` from MMCC clock controller.
- Clocks: `core` (153.6 MHz), `axi`, `ahb`. IOMMU: `ijpeg_iommu`.

Identical silicon to webOS 3.0.6 / OPAL — same TouchPad, same Gemini block.
Working OPAL userspace binary is the ground-truth reference: it produces
correct JPEGs from this exact silicon. Reverse-engineered cross-vendor
register map is in `reports/gemini-cross-vendor-register-map.md`.

---

## 2. Repos and key files

- Mainline kernel under development: `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/`
  branch `tenderloin/6.18/upstream-patches` on `shr-github`.
- Driver: `drivers/media/platform/qcom/gemini/`
  - `gemini.c` — V4L2 mem2mem driver
  - `gemini_hw.c` — HW programming helpers (reset / configure / table loaders)
  - `gemini_hw.h` — register map + function decls
  - `gemini_jpeg.c` — JFIF preamble builder, standard quant/Huffman tables
  - `gemini_jpeg.h` — JFIF builder API
- Test harness: `/home/herrie/webos/touchpad-kernel/test_gemini.c` (V4L2 M2M
  test, fills NV12 with `Y = x ^ y`, UV = 0x80, encodes one frame, writes
  result to `/tmp/test.jpg`).
- Cross-vendor register reference:
  `linux-6.18-tenderloin/reports/gemini-cross-vendor-register-map.md`
- **Legacy webOS kernel source (for ground-truth comparison):**
  `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/media/video/msm/msm_gemini_*.{c,h}`
- **OPAL userspace binary:**
  `/home/herrie/webos/touchpad-kernel/doctor306-opal/nova-cust-image-opal.rootfs-3.0.6/usr/lib/libqcameralib.so`
- **OPAL decompilation (Ghidra):**
  `linux-6.18-tenderloin/reports/opal-camera/opal_libqcameralib_decompiled.c`

---

## 3. What works (verified on hardware)

1. **MMSS fabric port-7 unhalt** — without it, the WE engine's bus traffic
   blocks indefinitely and `FRAMEDONE` never fires. Fix is `BIT(7)` added
   to `mmcc_msm8660_unhalt_mmss_ports()` halt mask.

2. **IRQ-driven `RESET_ACK` wait via `struct completion`** — replaces a
   broken poll-and-pray loop. `IRQ_MASK = RESET_ACK` armed before writing
   `RESET_CMD = 0x0004FFFF`; the IRQ handler `complete()`s the completion
   when `RESET_ACK` arrives.

3. **`PIPELINE_CFG = 0x038061FB`** for offline NV12 H2V2. Decoded from
   OPAL's `gemini_lib_hw_pipeline_cfg @ 0x14a69c`:
   `0x61FB | (offline ? 0x02000000 : 0) | ((mode_dims.mode & 3) << 23)`
   with `offline=true`, `mode=3` → `0x038061FB`. Earlier inferred value
   `0x020065FB` was wrong — caused a system hang on `FE_CMD = 3`.

4. **Configure write order: PIPELINE_CFG must be LAST**. Sequence is
   FE → OP → WE → PIPELINE_CFG → DRI. Writing PIPELINE_CFG first arms
   the offline branch with FE/OP/WE still zeroed and any subsequent
   register read hangs the AHB bus.

5. **Post-reset WE configuration** — three writes that must happen
   immediately after `RESET_ACK`, before any other configure register:
   - `WE_Y_UB_CFG = 0x01FF0000` (matches `JPEG_WE_YUB_ENCODE` in legacy
     webOS `msm_gemini_hw_reg.h`)
   - `WE_Y_THRESHOLD = 0x016A0190` (matches legacy: `(0x16A << 16) | 0x190`
     for offline mode from `GEMINI_WE_Y_THRESHOLD` table in
     `webos-linux-kernel-touchpad/.../msm_gemini_hw.c:239`)
   - `WE_CBCR_THRESHOLD = 0x016A0190` (same)

   Without these the WE engine fires `WE_Y_OVERFLOW` (`IRQ_STATUS = 0x40`)
   and stalls. **Confirmed needed.**

6. **Huffman tables loaded BEFORE quant tables** (matches OPAL's
   `gemini_lib_hw_config` order at `0x14be78`). Reversing causes worse
   corruption.

7. **PING addresses mirrored to PONG slots** — without it, the FE engine's
   ping-pong loop stalls reading uninitialised PONG (address 0) between
   IRQs on a single-buffer encode. Confirmed needed — without mirror, we
   get `BUS_ERROR`.

8. **IRQ handler must NOT `disable_irq` on transient events**, and must
   re-issue `FE_CMD = OFFLINE_CMD_START` on each `FE_RD_DONE` to advance
   the encoder. Without re-issue, `FRAMEDONE` never fires (encoder hangs
   waiting for FE to advance). **Confirmed needed.**

---

## 4. What's broken (the actual symptom)

`test_gemini -w 320 -h 240` produces `/tmp/test.jpg`:

- **Container is structurally valid JFIF.**
  - SOI, APP0/JFIF, DQT(luma), DQT(chroma), DHT×4, SOF0(H2V2), SOS, entropy
    stream, EOI — all markers present, all lengths consistent, no corruption
    in the parser.
  - Output sizes scale plausibly with input: 11648 B / 46020 B / 113610 B
    for 320×240 / 640×480 / 1280×1024 respectively.
  - `ENCODE_OUTPUT_SIZE` register reading matches the entropy-stream byte
    count to ±1.

- **Pixel content is wrong.**
  - Test input fills Y plane with `Y[y][x] = x ^ y` and UV plane with `0x80`
    (so the decoded image should be an XOR-pattern grayscale gradient with
    no color anywhere).
  - Decoded output decoded by PIL: most pixels are grayscale (R==G==B), but
    scattered 8×8 blocks decode to fully saturated `(0,0,~200)` or
    `(255,255,255)`. Mean `|R-G|` is 35–60 across the image — should be 0
    for pure grayscale input.
  - The pattern is **per-8x8-block saturation** — DCT coefficients are
    blowing up to ±1024 inside otherwise-correct rows. Classic signature of
    encoder/decoder MCU layout mismatch *or* per-block coefficient scaling
    error.

  Sample `test_320.jpg` row 0 first 8 pixels:
  ```
  (0, 0, 18), (0, 0, 18), (0, 0, 16), (0, 0, 14),
  (0, 0, 14), (0, 0, 11), (0, 0,  9), (0, 0,  5)
  ```
  vs expected (input Y=0..7, UV=128 → grayscale 0..7):
  ```
  (0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3),
  (4, 4, 4), (5, 5, 5), (6, 6, 6), (7, 7, 7)
  ```

---

## 5. Configuration we currently write (to the silicon, not via OPAL)

Per-frame, after `RESET_ACK`, in this exact order:

```
WE_Y_UB_CFG       (0x00E8) = 0x01FF0000
WE_Y_THRESHOLD    (0x00C0) = 0x016A0190
WE_CBCR_THRESHOLD (0x00C4) = 0x016A0190

FE_INPUT_FORMAT   (0x0038) = 0x10        // bits 5:4 = subsampling code 1 (NV12)
FE_DIMS           (0x003C) = (Wm-1)<<16 | (Hm-1)
FE_PIPELINE_MODE  (0x0040) = 0x203       // offline (mode=3)

OP_ENCODE_MODE    (0x0044) = 1           // mcu_type for NV12 (op_format & 3)
OP_GEOM[0]        (0x0048) = 16 * (Wm - 1)
OP_GEOM[1]        (0x004C) = 16 * (Wm - 1)
OP_GEOM[2]        (0x0050) = 256 * Wm * (Hm-1) + 16
OP_GEOM[3]        (0x0054) = 128 * Wm * (Hm-1) + 16
OP_FORMAT_MAGIC   (0x0058) = 0x0107081F  // op_format=1, NV12 H2V2 unrotated
OP_MATRIX[0..8]   (0x005C..0x007C) = 0x303, 0xF0000F, 0xF0000F00, 0, 0,
                                     0xC0C0303, 0xC0C03030, 0, 0
                  // verified at file offset 0x178FA4 in libqcameralib.so

WE_CFG            (0x0098) = 0x20        // burst code 2 for NV12

PIPELINE_CFG      (0x0008) = 0x038061FB  // arms the pipeline LAST
DRI_INTERVAL      (0x00F4) = 0
```

Then for each frame's `device_run`:

```
FE_BUFFER_CFG     (0x0080) = ((rows-1) << 16) | (rows-1)   // rows = (H+15)/16
FE_Y_PING_ADDR    (0x0084) = src_y_paddr
FE_CBCR_PING_ADDR (0x008C) = src_cbcr_paddr
FE_Y_PONG_ADDR    (0x0088) = src_y_paddr        // mirror
FE_CBCR_PONG_ADDR (0x0090) = src_cbcr_paddr     // mirror
FE_CMD            (0x0094) = 1                  // CMD_RELOAD

WE_Y_PING_ADDR    (0x00D8) = dst_paddr + hdr_aligned
WE_Y_PING_CFG     (0x00C8) = we_room
WE_Y_PONG_ADDR    (0x00DC) = dst_paddr + hdr_aligned   // mirror
WE_Y_PONG_CFG     (0x00CC) = we_room                   // mirror

IRQ_MASK          (0x0014) = FRAMEDONE | BUS_ERROR | VIOLATION

IRQ_MASK          (0x0014) = 0xFFFFFFFF              // start sequence
START_KICK        (0x00F0) = 1
FE_CMD            (0x0094) = 3                       // OFFLINE_CMD_START
```

Then per frame:
- Quant tables loaded via `TABLE_SEL=5` / `TABLE_INDEX=0` / 128 writes to
  `TABLE_DATA = 0x10000 / Q[i]` (16.0 reciprocal).
- Huffman tables loaded via `TABLE_SEL=6` with the canonical-Annex-C
  derivation cross-vendor analysis recovered (prologue: 12 per-length seed
  slots per table at `INDEX = (n<<6)|2` (luma) or `|3` (chroma); main
  loop: 176 per-symbol LUT slots per table at `INDEX = (i<<2)|0` (luma) or
  `|1` (chroma)).

JFIF header built in software, prepended to dst buffer; `EOI` appended
after the WE-produced entropy bytes.

---

## 6. Cross-checks we've run

### 6a. HW reset state

Register dump immediately after `RESET_ACK`, before any writes:

```
PIPELINE_CFG=0x00000000
FE_INPUT_FMT=0x00000000  FE_DIMS=0x00000000  FE_PIPELINE_MODE=0x00000000
OP_ENC_MODE=0x00000000   OP_FORMAT_MAGIC=0x00000000
OP_GEOM[0..3]=0x00000000 0x00000000 0x00000000 0x00000000
OP_MATRIX[0..8]=0,0,0,0,0,0,0,0,0
WE_CFG=0x00000000  WE_Y_UB_CFG=0x00000000  WE_Y_TH=0x00000000  WE_CBCR_TH=0x00000000
DRI_INTERVAL=0x00000000  IRQ_MASK=0x00000000  IRQ_STATUS=0x00000000
```

**Every register is 0x0 after reset.** No silent HW defaults.

### 6b. OPAL binary symbols — verified our values match

`arm-linux-gnueabihf-objdump -t libqcameralib.so` exposes:

```
00178f5c l  O .rodata 00000048  GEMINI_FE_RL_BURSTMASK
00178fa4 l  O .rodata 0000006c  GEMINI_FE_OL_BURSTMASK_64BB   ← we use this
001be868 g  O .data   00000003  hw_burstmask_table_index = [0x02, 0x03, 0x01]
```

OPAL's `gemini_lib_hw_op_cfg @ 0x14a28c`:

```c
r0 = mode_dims.mode;                         // = 3 for offline
r9 = byte_at(hw_burstmask_table_index, r0);  // OOB on a 3-byte table → 0
fp = OL_BURSTMASK_64BB + r9 * 4;             // = OL_BURSTMASK_64BB + 0
// OP_MATRIX[i] = *(fp + 0x48 + i*0xC)
// OP_FORMAT_MAGIC = *(fp + 0x48 - 0xC) = ... wait same row as OP_MATRIX
```

Reading `fp + 0x48` at file offset `0x178FA4` gives `0x00000303` =
**OP_MATRIX[0]** ✓. The full row matches our values byte-for-byte:
`0x303, 0xF0000F, 0xF0000F00, 0, 0, 0xC0C0303, 0xC0C03030, 0, 0`.

### 6c. Legacy webOS kernel — verified our threshold values

`webos-linux-kernel-touchpad/drivers/media/video/msm/msm_gemini_hw.c:239`:

```c
static const uint32_t GEMINI_WE_Y_THRESHOLD[2][2] = {
    { 0x00000190, 0x000001ff },   // [WE_ASSERT][offline=0, realtime=1]
    { 0x0000016a, 0x000001ff }    // [WE_DEASSERT][offline=0, realtime=1]
};
```

Combined for offline (`is_realtime=0`):
`(0x16A << 16) | 0x190 = 0x016A0190` ✓ matches our value.

`WE_Y_UB_CFG = JPEG_WE_YUB_ENCODE = 0x01FF0000` ✓ matches.

### 6d. Legacy `msm_gemini_hw_write` is read-modify-write

```c
if (mask == 0xFFFFFFFF) old_data = 0;
else { old_data = readl(); old_data &= ~mask; }
new_data = (data & mask) | old_data;
writel(new_data, paddr);
```

On reset (registers all zero), RMW collapses to a direct write of
`(data & mask)`. Our `writel(value)` writes 32 bits with no mask, but every
value we write is fully within its register's mask, so the result is
identical.

### 6e. PIPELINE_CFG formula confirmed

OPAL `gemini_lib_hw_pipeline_cfg @ 0x14a69c`:

```
data = 0x61FB
     | (offline ? 0x02000000 : 0)
     | ((p[1] & 3) << 23)   // p[1] = mode_dims.mode lower byte = 3 for offline
     | ((p[5] & 1) << 10)   // p[5] = 0 (OPAL caller passes 0)
     | ...                  // bits 20-22 also 0 in OPAL caller
```

Offline NV12 → `0x61FB | 0x02000000 | (3<<23) = 0x038061FB` ✓.

---

## 7. Diagnostics tried, none of which fixed the pixel content

| Experiment | Result |
|---|---|
| `OP_ENCODE_MODE=3 + OP_FORMAT_MAGIC=H2V2 (0x03381801)` | Different garbage; smoother gradient, strong color cast |
| `OP_ENCODE_MODE=1 + OP_FORMAT_MAGIC=H2V2` | Worse — alternating black/white again |
| Quant table READBACK pass after WRITE pass (mimics OPAL's `gemini_lib_hw_read_quant_tables`) | Zero observable change in output |
| Drop `FE_RD_DONE → FE_CMD=START` re-issue | Encoder hangs (FRAMEDONE never fires) |
| Drop post-reset WE thresholds | `WE_Y_OVERFLOW` (status=0x40) — WE thresholds required |
| Skip OP_MATRIX writes (= zeros) | **Interesting:** 640×480 row 0 decoded as clean grayscale `(155, 150, 140, 135, 142, 149, 143, 129)` — recognisable image content. 320 and 1280 still garbage. HW reset OP_MATRIX (0) happens to suit one specific MCU column count |

**The 640-with-zero-OP_MATRIX result is the loudest signal.** It says the
encoder *can* produce sensible bitstream from this silicon. Our cross-vendor
values cause output that's *less* sensible than no-op. Yet those values are
verifiably what OPAL writes.

Implication: there is something in OPAL's runtime sequence — a register
write we don't know about, an ordering constraint, or a state-machine
prerequisite — that our static analysis hasn't caught.

---

## 8. The plan: live trace from working webOS

Boot the TouchPad to webOS 3.0.6 (which has the working OPAL stack), take a
real photo with the Camera app, and snapshot the Gemini register region
during/around the encode. Compare register values vs ours.

### What we'll capture

A `/dev/mem`-based poller (`gemini_reg_poll.c`) that:

1. mmaps `/dev/mem` at `0x04600000` size 4 KB (Gemini base).
2. Polls every interesting register at ~10 kHz, timestamping each unique
   `(register, value)` pair to a log file.
3. Snapshots the FULL register state at three sync points:
   - "before" — before launching the camera app (idle)
   - "during" — while the camera is running and the user takes a photo
     (the binary detects activity by watching `IRQ_STATUS` / `FE_CMD` go
     non-zero)
   - "after" — after the photo is saved and the encoder has gone idle

Output is a delta log + 3 snapshots. We compare:
- snapshot during ≈ what OPAL writes ≡ ground truth.
- All registers we currently *don't* write but OPAL does → candidates for
  the missing piece.
- Any register OPAL writes with a value different from ours → candidate
  fix.

### Key registers the poller MUST cover

```
0x0000  HW_VERSION                  (RO, identifies silicon revision)
0x0004  RESET_CMD
0x0008  PIPELINE_CFG
0x000C  REALTIME_CMD
0x0014  IRQ_MASK
0x0018  IRQ_CLEAR
0x001C  IRQ_STATUS
0x0024  STOP_REQ
0x0028  STOP_STATUS
0x0034  ENCODE_OUTPUT_SIZE
0x0038  FE_INPUT_FORMAT
0x003C  FE_DIMS
0x0040  FE_PIPELINE_MODE
0x0044  OP_ENCODE_MODE
0x0048  OP_GEOM[0..3]    (4 dwords through 0x0054)
0x0058  OP_FORMAT_MAGIC
0x005C  OP_MATRIX[0..8]  (9 dwords through 0x007C)
0x0080  FE_BUFFER_CFG
0x0084  FE_Y_PING_ADDR
0x0088  FE_Y_PONG_ADDR
0x008C  FE_CBCR_PING_ADDR
0x0090  FE_CBCR_PONG_ADDR
0x0094  FE_CMD
0x0098  WE_CFG
0x00C0  WE_Y_THRESHOLD
0x00C4  WE_CBCR_THRESHOLD
0x00C8  WE_Y_PING_CFG
0x00CC  WE_Y_PONG_CFG
0x00D8  WE_Y_PING_ADDR
0x00DC  WE_Y_PONG_ADDR
0x00E8  WE_Y_UB_CFG
0x00F0  START_KICK
0x00F4  DRI_INTERVAL
0x0110  FSC_COUNT
0x0114  FSC_THRESHOLD[0..3]
0x0124  TABLE_SEL
0x0128  TABLE_INDEX
0x012C  TABLE_DATA      (poll: side-effect-free? maybe skip in tight poll)
```

Also worth dumping at sync points (less likely to change but if they do
it tells us something):
- The full 0x0000–0x0FFF region as one block.
- `MMCC` clock controller's IJPEG_GDSC + clock branches at `0x04000000+`
  (separate mmap).
- Optionally `vdd_dig` rail status from PMIC (less feasible from
  userspace).

---

## 9. Specific questions for whoever picks this up

1. **Does OPAL touch a register we haven't named?** If yes, that's almost
   certainly the bug. The cross-vendor register map covers 0x0000–0x012C
   plus the FSC range; anything OPAL pokes in 0x0130–0x0FFF or beyond is
   undocumented and likely the missing piece.

2. **Does OPAL set `OP_MATRIX[0..8]` to the values we set?** We've verified
   statically that OPAL *would* write those values for `mode=3`, but a
   live trace would confirm runtime behaviour (and reveal if there's
   conditional logic we missed).

3. **What does OPAL write to `WE_Y_THRESHOLD` / `WE_CBCR_THRESHOLD` /
   `WE_Y_UB_CFG` at the actual capture resolution?** Legacy webOS kernel
   sets the same constants we use, but if OPAL's userspace overrides them
   with width-dependent values, we'd never see that from the kernel
   source.

4. **What is the actual encode mode OPAL uses for stills?** We assumed
   offline (`mode_dims.mode = 3`). Confirm via the `FE_PIPELINE_MODE`
   register snapshot (`0x203` = offline, `0x101` = realtime).

5. **Do `TABLE_SEL` / `TABLE_INDEX` / `TABLE_DATA` get touched in a
   different ORDER than our prologue + 128 quant + 4×Huffman pattern?**
   Specifically, does OPAL switch `TABLE_SEL` between Huffman and Quant
   loads, or load both with `TABLE_SEL` left at one value?

6. **Quant table format:** we load `0x10000 / Q[i]` (16.0 fixed-point
   reciprocal). Does the live trace show that, or does it show direct `Q[i]`
   values — or some entirely different encoding (e.g. `(Q[i] << 8) | flag`)?

---

## 10. Latest deployed kernel

- Branch: `tenderloin/6.18/upstream-patches` on `shr-github`.
- Latest tested SRCREV: `38657f70082e` — has full register-state dump on
  `streamon` (so `dmesg` after a test cycle shows reset state and
  programmed state).
- Yocto recipe: `/media/herrie/LuneOS/scarthgap/webos-ports/meta-smartphone/meta-hp/recipes-kernel/linux/linux-hp-tenderloin_git.bb`.
- Build: `cd /media/herrie/LuneOS/scarthgap/webos-ports && source setup-env && MACHINE=tenderloin bitbake linux-hp-tenderloin`.
- Deploy: copy `tmp-glibc/deploy/images/tenderloin/uImage-dtb-zImage` to
  `/uboot/uImage.LuneOS` on the device, then sysrq-reboot.

---

## 11. Reproducer

```bash
# On the host, build the test harness:
arm-linux-gnueabihf-gcc -O2 -static \
    /home/herrie/webos/touchpad-kernel/test_gemini.c \
    -o /tmp/test_gemini

# Deploy:
scp -P 22 /tmp/test_gemini root@172.16.42.2:/tmp/test_gemini

# Run on the device (LuneOS, current kernel):
ssh -p 22 root@172.16.42.2 \
    "chmod +x /tmp/test_gemini; /tmp/test_gemini -w 320 -h 240; ls -la /tmp/test.jpg"

# Pull and inspect:
scp -P 22 root@172.16.42.2:/tmp/test.jpg ./test_320.jpg
python3 -c "
from PIL import Image
img = Image.open('test_320.jpg'); img.load()
for y in range(4):
    print([img.getpixel((x,y)) for x in range(8)])
"
```

Test pattern: `Y[y][x] = x ^ y`, `UV = 0x80`. Decoded image should be a pure
grayscale XOR pattern. Anything else means the encoder is wrong.

---

## 12. Bottom line for a reviewer

We have an encoder that runs to completion, fires `FRAMEDONE`, produces a
correctly-shaped JPEG container with a plausible-sized entropy stream, and
yet the entropy stream represents the wrong DCT coefficients. Every
configuration value we write has been independently verified against either
OPAL's binary or the legacy webOS kernel source. The HW reset state is
all-zero, so there are no hidden defaults masking a write we missed. The
read-modify-write semantics of the legacy kernel collapse to direct writes
on a zero baseline, so our `writel()` calls are equivalent.

The most informative next step is to live-trace OPAL on this exact silicon
and diff against what we write. The poller in
`tools/gemini_reg_poll/gemini_reg_poll.c` does this.
