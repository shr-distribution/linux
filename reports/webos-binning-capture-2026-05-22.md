# webOS legacy-kernel binning capture — ground truth (2026-05-22)

**How:** patched 2.6.35-palm webOS debug kernel (`uImage.webOSdebug`) with a `/d/kgsl/diag_binning`
debugfs toggle that, per ISSUEIBCMDS, walks IB1→IB2, tallies draw/bin opcodes, and (for state/shader
submits ≥512 dwords) idles the GPU and reads back the A220 VSC/LRZ/GPR register block + the
visibility-stream buffers. Captured ~1 s of the **webOS home-screen compositor** (LunaSysMgr, pid 2600).
Raw log: `webos-binning-capture-2026-05-22.log`.

## Headline

**webOS fully configures the A220 hardware binner and keeps it enabled — even for 2D UI compositing —
yet issues only plain `DRAW_INDX` (0x22), never `DRAW_INDX_BIN` (0x34).** freedreno leaves the entire
VSC block unprogrammed. The binner is always in the primitive path; webOS gives it real state, freedreno
gives it garbage → the period-8 tile-coverage cycle.

This **refines BINNER-CONFLICT and shrinks the fix**: freedreno does NOT need `DRAW_INDX_BIN` or a
separate binning-shader pass. It needs to **configure the VSC the way webOS does at context setup**.

## Ground-truth register state (steady-state readback, post-idle)

| Register | Value | Note |
|---|---|---|
| `LRZ_VSC_CONTROL` (0x2209) | **0x1** steady; **0x3** transiently | written 0x3 via `SET_CONSTANT` reg 0x209 during the configure block (`+0238 op=0x2d 00040209 00000003`), settles to 0x1. Phase-0 used 0x3 and stalled → 0x1 is the run value. |
| `VSC_BIN_SIZE` (0x0c01) | **0x108** | bin dimensions |
| `SQ_GPR_MANAGEMENT` (0x0d00) | **0x0007f010** | matches decompiled prelude (VTX=127, PIX=1) |
| `VSC[0x0c00]` master | 0x0 | |

**8 VSC pipes**, each a triple `(DATA_ADDRESS, DATA_LENGTH=0x40000 (256 KB), PIPE_CONFIG)`:

| pipe | DATA_ADDRESS | PIPE_CONFIG |
|---|---|---|
| 0 | 0x660b4080 / 0x660b5000 | 0x01200000 |
| 1 | 0x66cf5000 | 0x01100400 |
| 2 | 0x66d35000 | 0x01100401 |
| 3 | 0x66d75000 | 0x01200800 |
| 4 | 0x66db5000 | 0x01200002 |
| 5 | 0x66df5000 | 0x01100402 |
| 6 | 0x66e35000 | 0x01100403 |
| 7 | 0x66e75000 | 0x01200802 |

PIPE_CONFIG looks like bin (x,y): high nibble 0x011/0x012 = column, low field 0x400/0x800/0x002 = row.
All 8 DATA_ADDRESS values are **mapped** (kernel resolved + dumped them).

## Visibility streams are NON-EMPTY

Each pipe buffer holds real binner output, e.g. header `0000002c 00000008 00000008 00000018` followed by
dense per-primitive bin-assignment nibble streams (`13322113 21133221 32211332 …`). So the binner is
actively producing and consuming visibility data through these pipes — this is the thing Phase-0 lacked
(it enabled `LRZ_VSC_CONTROL=3` with no pipe buffers → empty stream → stall).

## Per-submit opcode profile (compositor)

`SET_CONSTANT (0x2d) ×253`, `DRAW_INDX (0x22) ×81`, `IM_LOAD_IMMEDIATE (0x2b) ×38`, `0x26 ×10`,
`0x52 ×4`, `0x46 ×4`. No `DRAW_INDX_BIN`/`SET_BIN_MASK/SELECT`. (Note: VSC is programmed via
`SET_CONSTANT`/TYPE-0, not `SET_BIN_*`, so the kernel's `bins=` opcode counter reads 0 — the decisive
evidence is the register readback, not the opcode tally.)

## Actionable fix for Fork C (revised, smaller scope)

At GLES context / first-render setup, freedreno must replicate the webOS VSC configuration — **without**
switching draws to `DRAW_INDX_BIN`:

1. Allocate **8 pipe BOs** (≥256 KB each; `DATA_LENGTH=0x40000`).
2. Program `VSC_BIN_SIZE=0x108`, the 8 `VSC_PIPE_{CONFIG,DATA_ADDRESS,DATA_LENGTH}` triples
   (config encodes bin x/y for the 2×3 layout), `SQ_GPR_MANAGEMENT=0x0007f010`.
3. Drive `LRZ_VSC_CONTROL`: 0x3 during configure, **0x1** for steady render (not 0x3).
4. Keep emitting normal `DRAW_INDX`.

Prediction: this collapses the period-8 cycle because the binner now routes through 8 valid, backed
pipes instead of garbage power-on state. If it stalls, recheck the pipe BO mapping/length (Phase-0
signature).

## Full-screen capture — VSC config is scene-INVARIANT (2026-05-22, 2nd run)

Log: `webos-binning-capture-3d-2026-05-22.log`. No native 3D GLES app is installed; webOS's only GLES
client is the LunaSysMgr compositor (pid 2600), so "3D" = heavy full-screen compositing. Drove it via
`luna-send` app launches (browser/deviceinfo/tasks full-screen cards + slide/scale animations). Result:

- Heavier workload confirmed: **draws=41** per submit (vs 27 idle), init submit 1400 dwords (vs 1152).
- **`VSC_BIN_SIZE` = 0x108 — identical.** All 8 **`PIPE_CONFIG` values — identical** to the idle capture.
- `LRZ_VSC_CONTROL` toggles **0↔1** (1 while a binning pass is active, 0 between); 3 only transiently
  during the configure block.
- vis-stream still non-empty; only the per-pipe written sizes change with the scene
  (idle header `…08 08 18`, full-screen `…0c 0c 1c`).

**Conclusion: the binner config (bin size + 8-pipe layout) is FIXED at GLES-context init, independent of
scene/surface size.** Only the visibility-stream *contents/sizes* vary. → freedreno can use one static VSC
configuration; no per-scene recompute needed.

## Register decode

**`VSC_BIN_SIZE` = 0x108 → 256 × 256 px bins.** (a2xx.xml: WIDTH=`(reg & 0x1f)<<5`, HEIGHT=`((reg>>5)&0x1f)<<5`;
both fields = 8 → 8<<5 = 256.) A 1024×768 surface = 4×3 = 12 bins; webOS uses the 8 hardware pipes
(reassigning across bins as needed). Note freedreno's SW tile-loop uses larger ~512×256 tiles (6 tiles) —
a different tiling than the HW binner's 256×256.

**8 `PIPE_CONFIG` words** (freedreno leaves this register opaque; raw values are what matter for
replication — they're fixed, so write them verbatim):

| pipe | reg | PIPE_CONFIG | DATA_ADDRESS | DATA_LENGTH | inferred (bin x,y) |
|---|---|---|---|---|---|
| 0 | 0x0c06 | 0x01200000 | 0x0c07=0x660b5000 | 0x0c08=0x40000 | (0,0) |
| 1 | 0x0c09 | 0x01100400 | 0x0c0a=0x66cf5000 | 0x40000 | (0,1) |
| 2 | 0x0c0c | 0x01100401 | 0x0c0d=0x66d35000 | 0x40000 | (1,1) |
| 3 | 0x0c0f | 0x01200800 | 0x0c10=0x66d75000 | 0x40000 | (0,2) |
| 4 | 0x0c12 | 0x01200002 | 0x0c13=0x66db5000 | 0x40000 | (2,0) |
| 5 | 0x0c15 | 0x01100402 | 0x0c16=0x66df5000 | 0x40000 | (2,1) |
| 6 | 0x0c18 | 0x01100403 | 0x0c19=0x66e35000 | 0x40000 | (3,1) |
| 7 | 0x0c1b | 0x01200802 | 0x0c1c=0x66e75000 | 0x40000 | (2,2) |

`VSC_SIZE_ADDRESS` = `0x0c02` = 0x660b4080 (binner writes per-pipe vis-stream byte counts here:
`0000002c 00000008 00000008 00000018 …`). High half of PIPE_CONFIG is const-ish (0x0110/0x0120 — likely
width/height-in-bins or format); low half looks like `(y<<10)|x` bin coordinates (inferred, not from xml —
treat as best-effort; replicate raw u32s, not the decode).

## Real 3D GLES app capture — config is PER-CONTEXT, not global (2026-05-22, 3rd run)

No native 3D app ships on the image, so built the PalmPDK `simple` sample (spinning perspective-projected
indexed-triangle mesh; GLES2 + SDL + PDL, full-screen) with `/opt/PalmPDK/arm-gcc` and ran it standalone
via `novacom run` (`/media/internal/simple`). Lowered the kernel diag's VSC-readback threshold 512→256
(rebuild #20) so the app's ≤300-dword submits trigger the readback. Log:
`webos-binning-capture-glapp-2026-05-22.log` (app = pid 3469).

**This corrects the "scene-invariant / fixed config" claim above.** It is invariant *within one GL context*
(compositor idle vs full-screen were identical), but **different GL contexts get different binner configs**:

| | compositor (2D) | `simple` 3D app |
|---|---|---|
| `VSC_BIN_SIZE` | 0x108 = **256×256 px** | 0x187 = **224×384 px** |
| 8× `PIPE_CONFIG` | 0x01200000,0x01100400,0x01100401,0x01200800,0x01200002,0x01100402,0x01100403,0x01200802 | 0x01100000,0x01100001,0x01200400,0x01100002,0x01100402,0x01100003,0x01100004,0x01200403 |
| `DATA_LENGTH` / pipe | 0x40000 (256 KB) | 0x40000 (256 KB) |
| `SQ_GPR_MANAGEMENT` | 0x0007f010 | 0x0007f010 (same) |
| draws | plain DRAW_INDX, no _BIN | plain DRAW_INDX, no _BIN |
| vis-stream | non-empty | non-empty |

**Why the bin size differs:** the driver sizes bins to fit a tile (color [+depth/stencil]) in the 512 KB
GMEM. The 2D compositor (color only) → 256×256×4 = 256 KB. The 3D app (color **+ depth**) needs more
bytes/pixel, so the driver shrinks the tile → 224×384 (224×384×4 = 336 KB, still ≤512 KB). PIPE_CONFIG low
half = `(y<<10)|x` bin coords: compositor 4×3 grid (256² over 1024×768), app 5×2 grid (224×384). 8 HW pipes
cover up to 8 of those bins.

## Corrected Fork-C guidance

freedreno must **compute** the bin grid from the render-target size **and** its GMEM byte budget (color +
depth/stencil), the way the proprietary driver does — NOT hardcode `0x108`. Then program `VSC_BIN_SIZE`,
the 8 `VSC_PIPE_{CONFIG,DATA_ADDRESS,DATA_LENGTH}` triples (config = `(y<<10)|x` per bin), allocate the
backing pipe BOs (256 KB each here), set `SQ_GPR_MANAGEMENT=0x0007f010`, drive `LRZ_VSC_CONTROL` 3→1, keep
plain `DRAW_INDX`. The captured compositor/app pairs give two worked examples to validate the computation
(2D no-depth vs 3D with-depth).

## Tooling notes
- PDK 3D test binary: `/home/herrie/webos/touchpad-kernel/simple-pdk/linux/Build_Device/simple`
  (deployed to device `/media/internal/simple`; survives reboot). Reusable for future captures / A-B.
- glmark2 / kmscube are NOT runnable on legacy webOS (no DRM/KMS; no x11/gbm/wayland EGL platform — webOS
  GLES is SDL+PDL). The PDK sample is the correct vehicle.
- Deployed kernel is now build #20 (VSC-readback threshold 256). `/boot` == `/uboot` == mmcblk0p13.
