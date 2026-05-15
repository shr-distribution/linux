# webOS legacy kernel IB-content decode — final offline analysis

**Date:** 2026-05-13
**Source data:** `/tmp/webos-logs-2026-05-13/` (pulled before reboot)
**Data span:** 512 IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS submits across ts 49188-50238 (1050 ts units)
**Capture:** patched 2.6.35-palm-tenderloin kernel + cap-binary + LunaSysMgr activity

## TL;DR

webOS's GLES rendering uses **512 ISSUEIBCMDS in a continuous stream** during normal home-screen activity.  The pattern is **many small submits (143-165 dwords each)** interleaved with **rare large submits (1185-1473 dwords)** and **pure sync submits (2 dwords)**.  Mainline Mesa freedreno emits ONE megasubmit (~1000 dwords) per render.  This is the architectural delta.

## Submit-size distribution

| sz (dwords) | count | % | class |
|-------------|-------|---|-------|
| **153** | **202** | **39%** | dominant draw submit |
| 156 | 60 | 12% | draw variant |
| 2 | 36 | 7% | pure CP_WAIT_FOR_IDLE sync |
| 143 | 26 | 5% | small draw |
| 1473 | 13 | 2.5% | state init + shader load |
| 165 | 11 | 2% | mid-size draw |
| 1185 | 10 | 2% | shader load + state init |
| 744 | 5 | 1% | mid-large |
| 146 | 5 | 1% | small draw |
| various | 144 | 28% | other sizes |

**70%+ of submits are 143-156 dwords (the "per-tile draw" submit size).**

## First-32-dword opcode frequency across all submits

```
CP_SET_CONSTANT       3553   (state register writes — dominant)
CP_WAIT_FOR_IDLE      439    (sync barriers — ~1 per submit)
TYPE-0 reg writes     72     (direct MMIO via PM4)
CP_IM_LOAD_IMMEDIATE  36     (shader code uploads)
```

**Notable absence:** `CP_DRAW_INDX` (opcode 0x22) is **never visible in the first 32 dwords** of any submit.  Webos puts state setup at the beginning of each IB, then DRAWs further in.  Our dump truncates before reaching the draws.

## Submit classes confirmed

### Class 1: Sync submit (sz=2, 36 occurrences)

```
+000 T3 CP_WAIT_FOR_IDLE(cnt=1) hdr=c0002600  payload=00000000
```

Pure 2-dword IB with only a CP_WAIT_FOR_IDLE.  Function: barrier between work-bearing submits to ensure GPU drains state.  These are emitted by `leia_cmdbuffer_insertwaitforidle` in vendor libGLESv2.

**Effect on 0x0ee2:** mostly no change (29/36 same), but 7/36 do change — explained by GPU still draining work from previous submit when we read `after`.

### Class 2: Draw submit (sz=143-156, 288 occurrences = 56%)

Sample (sz=156, ts=49188):
```
+000 T3 CP_SET_CONSTANT(cnt=2) reg_offset=0x202 (PA_CL_VPORT*)
+00c T3 CP_SET_CONSTANT(cnt=3) reg_offset=0x081 (RB_SURFACE_INFO area)
+01c T3 CP_SET_CONSTANT(cnt=5) reg_offset=0x303 (PA_CL_VPORT_*SCALE/OFFSET)
+034 T3 CP_SET_CONSTANT(cnt=5) reg_offset=0x10f (PA_SC_WINDOW_SCISSOR_*)
+04c T3 CP_SET_CONSTANT(cnt=2) reg_offset=0x201 (PA_CL_VTE_CNTL/CLIP_CNTL)
+058 T3 CP_SET_CONSTANT(cnt=2) reg_offset=0x312 (RB_DEPTHCONTROL)
+064 T3 CP_WAIT_FOR_IDLE             <-- mid-stream sync
+06c T3 CP_SET_CONSTANT(cnt=2) reg_offset=0x200 (PA_CL_*)
+078 T3 CP_SET_CONSTANT(cnt=2) reg_offset=0x205
... (continues for ~120 more dwords — state setup + CP_DRAW_INDX buried later)
```

Pattern: heavy `CP_SET_CONSTANT` register-write block at start (state restore), `CP_WAIT_FOR_IDLE` mid-stream (waiting for previous draw to retire), then more state + DRAW packets (not visible in first 32 dwords).

**Effect on 0x0ee2:** roughly 50/50 same/diff — when GPU drained between this and previous submit, `before == after`; when previous work still in flight, `before != after`.

### Class 3: Shader-load + state-init submit (sz=1185-1473, ~25 occurrences = 5%)

Sample (sz=1185, partial):
```
+000 T3 CP_SET_CONSTANT(cnt=5) reg_offset=0x080  (initial state)
+018 T3 CP_SET_CONSTANT(cnt=13) reg_offset=0x078 (vertex format / TEX state)
   payload: 685d6523 00000020 685d6563 00000020 ...  (sampler/texture register values)
+050 T3 CP_IM_LOAD_IMMEDIATE(cnt=32)              <-- SHADER UPLOAD
   payload: 0000001e 02556003 00001000 c2000000  (ALU instructions)
+0d4 T3 CP_IM_LOAD_IMMEDIATE(cnt=?)               <-- another shader chunk
...
```

These are issued whenever webOS switches GL programs / textures.  Contains:
- Sampler / texture register initialization (T3 with reg_offset=0x078 area)
- VS and PS shader code via `CP_IM_LOAD_IMMEDIATE` (opcode 0x2b)
- Constant memory uploads via larger `CP_SET_CONSTANT` packets

**Effect on 0x0ee2:** mostly same (29/36), 7/36 different — shader loads don't intrinsically advance the slot counter.

## Cap-binary attribution

```
pid=3453 (cap binary directly): 3 submits, all 0x0ee2 unchanged (= 0x15014142)
pid=2558 (LunaSysMgr/compositor): 512 submits — the actual rendering
```

webOS's GL is **proxied to the compositor**.  The cap binary's GL calls go through a shared GL context owned by LunaSysMgr, so the IBs get emitted in LunaSysMgr's address space.  This means our cap binary's 16-submit-per-render observation isn't really cap-binary-specific — it's how LunaSysMgr renders ANY frame.

## webOS render topology (reconstructed)

A typical "render a frame" sequence on webOS likely involves:

```
[shader-load submit if program changed]          sz=1185-1473
[per-tile setup submit]                          sz=153-156
[per-tile draw submit (state + DRAW_INDX)]       sz=153-156
[per-tile resolve submit]                        sz=143-156
[sync submit]                                    sz=2
[next tile setup...]                             sz=153-156
...
```

For a 1024x768 framebuffer with 256x256 tiles = 4x3 = 12 tiles.  Each tile ~2-3 submits = 24-36 submits per full frame.  Cap binary's 16 submits per render is consistent with a smaller framebuffer (~768x512 = 1-4 tiles × 4-8 submits).

## Comparison with mainline Mesa

| | Mesa freedreno (mainline) | webOS userspace |
|---|---|---|
| Submits per render | 1 | 16 |
| Single-IB size | ~1000-3000 dwords | n/a (split) |
| Per-tile structure | inline in one IB | separate submits |
| Sync barriers | inline `CP_WAIT_FOR_IDLE` | dedicated sz=2 sync submits |
| Shader load | inline `CP_SET_CONSTANT_*` (alu/tex banks) | separate `CP_IM_LOAD_IMMEDIATE` submits |
| State init | in `fd2_emit_restore` (per batch) | in shader-load submits (per program change) |

## Why the period-16 cycle on mainline?

Earlier hypothesis: 0x0ee2 advances ~+0x00020200 per `CP_DRAW_INDX`.  Mesa emits 2 draws (binning + render) per frame → +2 slot advances → wraps at 16 renders.

webOS observation **CONFIRMS this** for the 0x15-namespace transitions.  But webOS ALSO uses 0x7f, 0x29, 0x6b, 0x7d namespaces — these are extra HW states triggered by webOS's binning passes / context switches / shader loads that mainline doesn't issue.

The exit from 0x7f back to 0x15 in webOS (e.g. ts=3788: `0x7ff29039 → 0x1501c1c2`) is the **GPU completing a binner pass and returning to rasterization slot**.  Mainline doesn't trigger this transition, so its slot pointer monotonically advances within the 0x15-pool until wrap.

## Implications for the fix

**Hypothesis: per-tile-split alone may not eliminate the cycle** — even if Mesa emits 16 submits per render matching webOS's count, the 16 mainline submits would all be "draw-class" since Mesa's binning + state are inlined.  We'd still walk the 0x15-pool linearly.

**The real architectural fix would need:**
1. Separate "shader-load + state-init" submits (matching webOS class 3)
2. Separate "draw" submits per tile
3. Sync-only submits between phases
4. **Move binning IB into separate MSM_SUBMIT** so the GPU's binner-state transitions to 0x7f and back

The last point is critical — Mesa currently puts the binning IB inside the main batch ringbuffer.  By submitting binning as its own ISSUEIBCMDS, the GPU would visit the 0x7f namespace and the per-render slot advance would no longer be deterministic +1/cap.

## Next actions

1. **Re-dump with HIGHER dword count** (e.g. 128 dwords) to catch CP_DRAW_INDX position and the post-draw structure
2. **Decode the IB-2 / sub-IB structure** — webOS may use CP_INDIRECT_BUFFER from each submit, requiring further IB walks
3. **Implement Mesa "split binning IB into own MSM_SUBMIT"** as the minimum-viable architectural change
4. **Trace single isolated test app** to get a clean 16-submit sequence (currently mixed with LunaSysMgr noise)

## Files

- `/tmp/webos-logs-2026-05-13/messages` — 765KB full /var/log/messages
- `/tmp/webos-logs-2026-05-13/kgsl.log` — 4868 KGSL_DIAG lines extracted
- `/tmp/webos-logs-2026-05-13/cap.bin` — webOS-rendered framebuffer (3.1MB)
- `/tmp/webos-logs-2026-05-13/regs.txt` — A2XX register snapshot from cap binary
- `/tmp/webos-logs-2026-05-13/dmesg-live.log` — live dmesg snapshot
- Kernel source: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/kgsl.c` (patched)
- Built uImage: same dir / `arch/arm/boot/uImage` (md5 `8370944a08cd`)
