# A22X Hardware Binning Port Analysis (Phase 1)

**Date:** 2026-05-11
**Status:** Phase 0 done (LRZ_VSC_CONTROL=0x03 collapses cycle but hangs). Phase 1 decomp 95% complete. Implementation can begin once shader-variant strategy is decided.

## Phase 0 result (2026-05-11)

Mesa patch 0092 (LRZ_VSC_CONTROL=0x03 in `clear_state_restore`) tested twice — fresh boot and post-stress. **Result is reproducible:**
- Cycle collapsed from 8 hashes to 1 (`d1dd210d6b1312cb342b56d02bd5e651`)
- All-zero alpha framebuffer (binner stalls with nowhere to write)
- dmesg: hangcheck recover, "GPU still busy after 3s", ring rptr/wptr=60/2CE
- GPU recovers cleanly each iteration — lever is **safe** to keep iterating with

Confirms LRZ_VSC_CONTROL is the right lever to engage the A22X binner state machine, but the binner has no buffers/shaders/config to consume — so it stalls.

## Phase 1 update: full decompile findings (2026-05-11)

Disassembled `leia_configure_binning_pass` @ 0x124a50 (756 bytes), `leia_binning_grow_vis_stream_buffer` @ 0x11d1e0 (892 bytes), and `yamato_configure_binning_pass` @ 0x111150 (732 bytes) from webos_libGLESv2.so. Cross-referenced with KGSL `kgsl_drawctxt.c`.

### A22X binning prelude — exact register-write sequence

```
# State setup (mode-0 / binning cmdbuf)
SET_CONSTANT PA_SU_SC_MODE_CNTL = state.mode & ~0x10000     # disable BACK_FACE bit
WAIT_FOR_IDLE
SET_CONSTANT RB_DEPTHCONTROL    = (state.depth & ~1) | 8    # Z disable, set bit 3
# (only emitted if state cache shows these regs differ from cached value)

# Binning engage
PKT0 reg=0xC00 count=1                                       # ← A22X-SPECIFIC, undocumented
  value = 0x00000001                                         #    write 0xC00 = 1 (VSC enable?)
SET_CONSTANT A220_RB_LRZ_VSC_CONTROL = 0x00000003            # ← engages binner
WAIT_FOR_IDLE
PKT0 reg=0xD00 count=1                                       # SQ_GPR_MANAGEMENT
  value = 0x0007f010                                         #    REG_DYNAMIC=0, REG_SIZE_PIX=1, REG_SIZE_VTX=127

# Render-pass restore (mode-1 / rendering cmdbuf)
SET_CONSTANT PA_SU_SC_MODE_CNTL = state.mode (original)      # restore
WAIT_FOR_IDLE
SET_CONSTANT RB_DEPTHCONTROL    = state.depth (original)     # restore
SET_CONSTANT PA_SC_AA_CONFIG    = leia_msaa_config(surface)  # 0x2301
SET_CONSTANT A220_RB_SAMPLE_POS = state.sample_pos           # 0x220a
```

**Key A22X-specific registers (NOT in mesa's a2xx.xml):**
- `0x0C00` — undocumented. yamato (A20X) does NOT write this. Likely "VSC pipe master enable". Should be added to a2xx.xml as `A220_VSC_ENABLE` or similar.
- `0x2209` — already in xml as `A220_RB_LRZ_VSC_CONTROL`, bitfield semantics still undocumented. Value 3 = engage binner. Value 0x84 = clear-mode. Value 0 = disabled.

### `leia_binning_grow_vis_stream_buffer` decoded

Dynamically grows visibility-stream buffers (per-pipe BOs) in 256KB increments, capped at 1MB total per pipe.

```c
void grow(ctx, required_size) {
    required_size = min(required_size, 0x100000);  // cap 1MB
    new_size = state->buffer_size + 0x40000;       // try +256KB
    if (new_size < required_size)
        new_size = state->buffer_size + 0x80000 + round_up_256k(required_size);

    for (i = 0; i < state->num_pipes; i++) {       // num_pipes = 8
        gsl_memory_alloc_pure(new_size, FLAGS_0xC0900, &new_bo);  // alloc
        gsl_command_freememontimestamp_pure(..., old_bo, ts, TYPE=2);  // free old on next ts
        memcpy(&state->pipes[i].bo, &new_bo, 20);  // 20-byte BO descriptor
    }
    state->buffer_size = new_size;

    // Then emit 24-reg type-0 PM4 packet writing all 8 VSC_PIPE descriptors:
    PKT0 reg=0xC06 count=24
       PIPE[0].CONFIG       = (h<<24) | (w<<20) | (y<<10) | x   # 0xC06
       PIPE[0].DATA_ADDRESS = bo[0].gpu_iova                     # 0xC07
       PIPE[0].DATA_LENGTH  = bo[0].size                         # 0xC08
       ... (× 8 pipes) ...
}
```

### Per-pipe state struct (offsets from `state = ctx + 0x920`)

```c
state + 0x7c:  uint32_t buffer_size;     // current per-pipe buffer size (0 .. 1MB)
state + 0x80:  uint32_t num_pipes;       // = 8
state + 0x84:  pipe[8] (36 bytes each)
    + 0x00..0x13 (20B): BO descriptor — handle, gpu_iova, size, hostptr, flags
    + 0x14:  uint32_t tile_x;       // 10-bit
    + 0x18:  uint32_t tile_y;       // 10-bit
    + 0x1c:  uint32_t tile_width;   // 4-bit
    + 0x20:  uint32_t tile_height;  // 4-bit
```

VSC_PIPE_CONFIG bitfield (encoded from above fields): `(h<<24) | (w<<20) | (y<<10) | x` — **same layout as A20X**.

### A22X vs A20X delta (yamato cross-check)

Comparing `yamato_configure_binning_pass` to `leia_configure_binning_pass`:

| Reg/Action | A20X (yamato) | A22X (leia) |
|---|---|---|
| `0xC00 = 1` (VSC enable?) | NOT written | **WRITTEN** ← A22X-only |
| `LRZ_VSC_CONTROL = 3` | N/A (register A22X-only) | **WRITTEN** ← A22X-only |
| `SQ_GPR_MANAGEMENT = 0x7f010` | written | written (same) |
| `PA_SU_SC_MODE_CNTL` toggle | written | written |
| `RB_DEPTHCONTROL` toggle | written | written |
| `PA_SC_AA_CONFIG` | written | written |
| `A220_RB_SAMPLE_POS` (0x220a) | N/A (A22X-only) | written |
| Reg 0x239d write | written | NOT |
| Reg 0x2316 write | written | NOT |
| `CP_SET_DRAW_INIT_FLAGS` (opcode 0x4b) | written | NOT |

A22X drops some legacy A20X PM4 ops and replaces them with the LRZ_VSC_CONTROL=3 mechanism + sample-pos write.

### KGSL kernel side (context save/restore)

`kgsl_drawctxt.c`:
- **Save:** dumps regs 0xC01..0xC1D (VSC_BIN_SIZE + all 24 VSC_PIPE) to gpustate, plus SQ_GPR_MANAGEMENT
- **Restore:** writes 0 to all 29 VSC regs, **and writes LRZ_VSC_CONTROL = 0** on Leia (lines 827, 1050)
- **Never** writes `0xC00` (the master enable) — meaning userspace must write it itself

So the proprietary libGLESv2 owns the binning state setup entirely; KGSL only ensures clean zero state on context switch.

### Cross-kernel verification (2026-05-11)

Checked three other MSM-era kernels (newer KGSL with `a2xx_reg.h` + `adreno_a2xx.c`):
- **raden ampang-AOSP-mako-kernel** (android-msm-mako-3.4-kk-mr1): Nexus 4, MSM8064 / A320
- **LineageOS lge-kernel-mako** (cm-14.1): same hardware, more recent
- **LineageOS sony-kernel-msm8960** (jellybean): MSM8960 / A225 — REV471 sibling
- **LG G2 k2wl/g2_kernel** (kitkat-4.4.4): MSM8974 / A330

All four kernels agree:
- `0x0C00` is **NOT defined** in any `a2xx_reg.h` — the master VSC enable is purely a userspace-only knob across the entire KGSL kernel family
- `REG_A220_RB_LRZ_VSC_CONTROL = 0x2209` defined, but **no bitfield documentation** in any kernel
- All `adreno_a2xx.c` writes to LRZ_VSC_CONTROL use the value `0x00000000` (in GMEM save/restore command sequences)
- **No KGSL kernel ever writes the value `3`** to LRZ_VSC_CONTROL — only the closed-source userspace does

**Important defensive finding (Sony MSM8960 KGSL `adreno_a2xx.c:902`)** emits a `CP_SET_BIN_BASE_OFFSET` (PM4 opcode 0x4b) on context restore. We saw yamato (A20X) emit the same on touchpad webOS. BUT webOS `kgsl_drawctxt.c:1923-1924` has:

```c
cmds[0] = pm4_type3_packet(PM4_SET_BIN_BASE_OFFSET, 1);
cmds[1] = drawctxt->bin_base_offset;
if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
    kgsl_ringbuffer_issuecmds(device, 0, cmds, 2);
```

**Leia REV470 (our exact chip!) is EXPLICITLY excluded** from `CP_SET_BIN_BASE_OFFSET` emission. This confirms `bin_base_offset` is NOT part of the A22X REV470 binning pipeline — A22X uses the `LRZ_VSC_CONTROL=3` mechanism instead. We can ignore CP_SET_BIN_BASE_OFFSET entirely in any port effort.

**Net:** The cross-kernel survey produced no new documentation but provided strong negative evidence:
- 0xC00 enable is not kernel-tracked anywhere → must be a transient "engage" register, not a persistent one
- LRZ_VSC_CONTROL bitfields remain undocumented across ALL public Qualcomm KGSL code
- bin_base_offset is irrelevant for our chip (REV470) — Mesa port should NOT emit CP_SET_BIN_BASE_OFFSET

## Caveat (important)

**A20X ≠ A22X.** They are different chips with related but distinct binning hardware. Anything in this document marked "A20X reference" is comparative material only, not a template to be copied. A22X (Leia / REV470) has its own binning mechanism that the proprietary webOS stack engages via `leia_configure_binning_pass` and `leia_binning_grow_vis_stream_buffer` — these have NO A20X equivalents in the Mesa freedreno codebase.

## 1. What we know about A22X binning (concrete facts)

### A22X has VSC pipe infrastructure

From `leia_reg.h`:

```c
#define REG_LEIA_VSC_BIN_SIZE             0x0C01
#define REG_LEIA_VSC_PIPE_DATA_LENGTH_7   0x0C1D
#define REG_LEIA_RB_LRZ_VSC_CONTROL       0x2209
#define REG_LEIA_GRAS_CONTROL             0x2210
```

The 0x0C06..0x0C1D range encodes 8 VSC_PIPE × 3 regs (CONFIG / DATA_ADDRESS / DATA_LENGTH). Same offsets as A20X. **Same register layout doesn't mean same semantics** — the surrounding control logic may behave differently.

### A22X has Leia-specific debug registers

`REG_LEIA_SQ_DEBUG_INPUT_FSM (0x0dae)`, `REG_LEIA_SQ_DEBUG_EXP_ALLOC (0x0db3)`, `REG_LEIA_SQ_DEBUG_PTR_BUFF (0x0db4)`, `REG_LEIA_SQ_DEBUG_GPR_VTX (0x0db5)`, etc. These were NOT in our register-invariance proof (update 27). They may expose binner state changes across the cycle.

### A22X has Leia-only control registers absent from A20X

- `REG_LEIA_SQ_RESOURCE_MANAGMENT (0x0d03)`
- `REG_LEIA_SQ_PIX_IN_CNTL (0x0d0c)`
- `REG_A220_GRAS_CONTROL (0x2210)`

### Mesa freedreno A22X disables hw binning explicitly

`fd2_gmem.c:60-64`:

```c
/* only a20x hw binning is implement
 * a22x is more like a3xx, but perhaps the a20x works? (TODO)
 */
if (!is_a20x(batch->ctx->screen))
    return false;
```

"a22x is more like a3xx" — Rob Clark's comment. Suggests A22X binning is closer to the A3XX visibility-stream model than to A20X's shader-memexport-to-per-pipe-BOs model.

## 2. Proprietary stack mechanism (webOS libGLESv2.so decomp)

### `leia_configure_binning_pass` @ 0x00134a50 (size 756 bytes)

Sets up the binning pass. Critical PM4 emissions (decoded from raw bytes):

```c
// First path: not in binning mode yet (uVar4 & 0x40) == 0
puVar1[1] = 0x40205;  // CP_REG(PA_SU_SC_MODE_CNTL)
puVar1[2] = uVar5;    // = state->mode & 0xfffeffff (disables some bit)
// WAIT_FOR_IDLE
puVar1[1] = 0x40200;  // CP_REG(RB_DEPTHCONTROL)
puVar1[2] = uVar4;    // = state->depth_ctl | 8, & ~1

// New cmdbuf chunk
*puVar1 = 0xc00;       // CP_PKT3 type-0 register write at 0x000c (?)
puVar1[1] = 1;
puVar1[2] = 0xc0012d00; // CP_SET_CONSTANT
puVar1[3] = 0x40209;    // ← CP_REG(A220_RB_LRZ_VSC_CONTROL)
puVar1[4] = 3;          // ← VALUE = 3   *** KEY FINDING ***
// WAIT_FOR_IDLE
// Some shader-related setup

// Switch to rendering pass (mode 1 = different cmdbuf):
puVar1[1] = 0x40205;  // PA_SU_SC_MODE_CNTL — restore original
puVar1[2] = state->mode;
puVar1[1] = 0x40200;  // RB_DEPTHCONTROL — restore original
puVar1[2] = state->depth_ctl;
puVar1[4] = 0x40301;  // CP_REG of 0x2301 — SQ_INTERPOLATOR_CNTL area
puVar1[5] = msaa_config;
puVar1[7] = 0x4020a;  // CP_REG(RB_SAMPLE_POS)
puVar1[8] = state->sample_pos;
```

**KEY: `A220_RB_LRZ_VSC_CONTROL = 3` during the binning pass.** Mesa patch 0017 currently writes 0 ("disabled"). webOS writes 3 ("binning active" or similar — semantics unknown, register is undocumented in the freedreno XML).

### `leia_binning_grow_vis_stream_buffer` @ 0x0012d1e0 (size 872 bytes)

Allocates/grows the per-pipe visibility-stream buffers. Called from `leia_configure_binning_pass:8092`. Not fully decoded yet — too complex to summarise without further work. But it implies the proprietary stack maintains **growable per-pipe BOs** for visibility data, similar in concept to A20X's `vsc_pipe_bo[]` but presumably with A22X-specific layout.

### Two-pass rendering structure

Decomp shows proprietary stack calls `leia_configure_binning_pass` from:
- `0x8620` — at the start of state-flush before draw
- `0x9298` — somewhere in `leia_process_primitive_flags`
- `0x9508` — another draw-state path

Suggests rendering follows a **binning pass + render pass** model:
1. Binning pass: VS runs, writes visibility bits to per-pipe streams
2. Render pass: per tile, hardware uses visibility stream to clip vertices

## 3. Difference vs Mesa freedreno A20X hw binning

| Aspect | A20X (Mesa) | A22X (webOS) |
|---|---|---|
| Mechanism | Shader memexport to per-pipe BOs | Hardware visibility-stream writer driven by `LRZ_VSC_CONTROL=3` |
| Shader changes | VS appends N memexport CFs (one per pipe) patched at runtime | VS may have a separate "binning variant" — needs investigation |
| Per-pipe BO | `vsc_pipe_bo[8]`, fixed size | Dynamic grow via `leia_binning_grow_vis_stream_buffer` |
| Visibility consumption | Via `VGT_CURRENT_BIN_ID_MIN/MAX` per tile | Same registers but different semantic — interpretation TBD |
| Setup cost per submit | ~32 dwords constants + per-pipe BO refs | Larger — config_binning_pass alone is 756 bytes of cmd buffer emit |

## 4. What our investigation already proved

Through 19 falsifications today:

- ✅ `VGT_CURRENT_BIN_ID_MIN/MAX` writes (any encoding) have NO effect on the cycle when hw binning is disabled. This was the wrong lever — the BIN_ID register only matters when the binner is actively engaged.
- ✅ `VSC_PIPE[0..7]` zero-init (patch 0017) has no measurable effect on the cycle either.
- ✅ Register snapshots are invariant across the cycle.
- ✅ The cycle is firmware-independent — `CP_DRAW_INDX` bypasses the ME.
- ✅ The cycle visibly manifests as GMEM tile-coverage failures (8 different tile-subset patterns).

This is **consistent with**: the binner hardware on A22X is in some default-disabled-with-undefined-residue mode when Mesa doesn't engage it. Different tile-coverage patterns come from this undefined residue cycling through 8 visibility-bit configurations internally.

## 5. The minimal-effort step-0 experiment (BEFORE attempting the full port)

**Hypothesis: `A220_RB_LRZ_VSC_CONTROL = 0x3` (matching webOS binning-pass value) puts the binner into a known stable state, not the cycling-default mode.**

Current upstream Mesa writes three distinct LRZ_VSC_CONTROL values:
- `0x00` in `clear_state_restore` (post-clear "disabled")
- `0x84` in `clear_state` (during clear)
- `0x00` and `0x84` in various `fd2_gmem.c` paths

Bit decomposition:
- `0x00` = no bits
- `0x03` = bits 0 + 1 (webOS binning mode)
- `0x84` = bits 2 + 7 (Mesa clear mode)

Bitfield semantics are undocumented in `a2xx.xml`. The bits set by webOS are distinct from those Mesa uses for clears, supporting "different mode" interpretation.

Note: **patch 0017 is currently NOT in SRC_URI** (was dropped 2026-05-11 in v2 of patch 0090 — they were tested together as a falsification). So Phase 0 will be a NEW patch numbered 0092 (or similar), not a modification of 0017.

Concrete change: new minimal patch modifying `fd2_draw.c:clear_state_restore` to write `0x00000003` instead of `0x00000000` to `A220_RB_LRZ_VSC_CONTROL`. Single value change, ~5 lines of diff.

If this collapses the cycle: we win without porting full hw binning. The binner just needs to be "explicitly told mode-3" rather than left undefined.

If this changes the cycle but doesn't collapse it: more nuanced — we've moved the binner into a different mode. May enable other levers.

If this has no effect: the register write is not enough; full hw binning port is required.

If GPU hangs / faults: `3` activates the binner but downstream state isn't set up to consume it. Roll back, plan full port.

This is **30 minutes of work** (1 patch + 1 build + 1 deploy + 1 test). Should be Phase 0 of any port effort.

## 6. Full hw binning port plan (if Phase 0 inconclusive)

### Phase 1 — characterise A22X binning hardware
- Read the FULL decompile of `leia_configure_binning_pass` (all 756 bytes) and `leia_binning_grow_vis_stream_buffer` (872 bytes). We have 60% so far.
- Decode every PM4 packet, every register write, every conditional branch.
- Cross-reference with KGSL kernel-side `kgsl_drawctxt.c` for context save/restore of VSC state — confirms what registers matter.
- Identify A22X-specific PM4 packet opcodes (if any) that A20X doesn't use.
- Extend register-invariance proof to include LEIA_SQ_DEBUG_* registers — see if they DO change across the cycle, exposing the binner state.

### Phase 2 — design Mesa port
- Decide whether to graft onto Mesa's existing A20X hw_binning code (with A22X-conditional branches) or write a parallel A22X path.
- Plan the shader compiler changes. A22X may need a "binning shader variant" — running just the VS with simplified outputs.
- Plan VSC pipe BO management — A22X uses dynamic growth, Mesa's a20x uses fixed 64KB alloc.

### Phase 3 — implement
- Patch fd2_gmem.c to enable hw binning on A22X.
- Patch fd2_program.c / ir2 shader compiler for A22X binning shader variant.
- Implement the leia_configure_binning_pass equivalent.
- Implement visibility-stream allocation.

### Phase 4 — test
- Smoke test: does GL still work?
- 100-cap test: does cycle collapse?
- Performance regression check.

Estimated effort: 1-2 weeks of focused engineering AFTER Phase 0 + Phase 1.

## 7. Risk assessment

- **Phase 0 (LRZ_VSC_CONTROL=3 test)**: low risk, 30 min, 30% chance of useful result either way.
- **Phase 1 (deeper analysis)**: low risk, 1-2 days, primarily understanding.
- **Phase 2-4 (full port)**: medium risk (Mesa shader-compiler changes could introduce regressions in other code paths). 1-2 weeks. Outcome uncertainty: 30/50/20 (fix/improve/null).

## 8. Open questions (updated 2026-05-11)

1. ~~What does `A220_RB_LRZ_VSC_CONTROL` value 3 actually do? Undocumented in `a2xx.xml`. Bit decoding TBD.~~ → Phase 0 confirmed value 3 engages the binner (cycle collapsed to 1 hash + hang).
2. Does A22X have a binning shader variant separate from the rendering shader in the proprietary stack? **PARTIAL ANSWER:** the binning prelude writes `SQ_GPR_MANAGEMENT = 0x7f010` (REG_SIZE_VTX=127, REG_SIZE_PIX=1) — strongly suggests the binning pass uses a stripped-down VS-only shader. Need to find where the proprietary stack actually patches/compiles the binning variant — may be in `rb_gpuprogram_loadexecutable_internal` with a `mode=2` argument (vs. mode=0 nobinning, mode=1 normal).
3. ~~What's at `0x40301` (CP_REG of 0x2301)?~~ → `PA_SC_AA_CONFIG` (in mesa's a2xx.xml). Used to set MSAA mode for the render pass.
4. ~~What does `leia_binning_grow_vis_stream_buffer` do internally?~~ → Decoded. Allocates 8 per-pipe BOs, sized in 256KB increments capped at 1MB total, then writes all 24 VSC_PIPE registers in one type-0 PM4 packet.
5. Does the A22X binner use `VGT_CURRENT_BIN_ID` for tile filtering when properly enabled (LRZ_VSC_CONTROL=3)? — STILL OPEN. Phase 0 hang prevented observation. Phase 3 implementation will produce data here.
6. **NEW:** What is register `0xC00`? The undocumented A22X "VSC enable" master register. yamato (A20X) doesn't write it. KGSL never references it (not in save/restore). Likely a one-shot enable. Needs an XML entry like `A220_VSC_ENABLE`.
7. **NEW:** What does `gsl_memory_alloc_pure` flag `0xC0900` mean? Some combination of "GPU-only, no CPU mapping, cmd-buffer-visible". Need to compare to how Mesa's freedreno allocates vsc_pipe BOs for A20X.

## 9. Recommendation (updated 2026-05-11)

Phase 0 done. Phase 1 done (this document).

**Next step: Phase 2 design decision** — pick the implementation strategy before writing code. Three forks:

**Fork A (preferred):** Graft onto Mesa's existing A20X `use_hw_binning` path in `fd2_gmem.c` with `is_a22x` branches.
- Pros: minimal new code, reuses tile loop / BIN_ID emission, reuses ir2 hw_binning shader patching infra
- Cons: A20X binner is mem-export based; A22X is `LRZ_VSC_CONTROL=3` HW-driven — semantics differ, may end up with all-A22X branches anyway

**Fork B:** Write a parallel `use_a22x_hw_binning` path with its own tile loop and shader variant pipeline.
- Pros: clean separation, no risk of breaking A20X
- Cons: ~2x code duplication

**Fork C:** Take the absolute minimum subset — emit only the binning prelude (`0xC00=1`, `LRZ_VSC_CONTROL=3`, `SQ_GPR_MANAGEMENT=0x7f010`) plus per-pipe BO allocation, but **use the existing VS as both binning and render shader** (no separate binning variant). Just see if the binner can pull visibility data from a regular VS.
- Pros: 1-day experiment; if it works we skip the shader compiler work entirely
- Cons: may produce visible artefacts in the render pass (VS runs twice) or simply hang differently
- Risk: low — Phase 0 already showed the GPU recovers cleanly from this kind of hang

**Recommended sequencing:**
1. **Fork C first** as a Phase 1.5 experiment — minimum-viable-binning. ~1 day.
2. Based on result, choose Fork A or Fork B for the full port.

## 10. Required mesa changes summary (any fork)

### a2xx.xml additions
```xml
<reg32 offset="0x0c00" name="A220_VSC_ENABLE"/>   <!-- NEW, undocumented in xml -->
<!-- LRZ_VSC_CONTROL bitfield documentation -->
<reg32 offset="0x2209" name="A220_RB_LRZ_VSC_CONTROL">
   <bitfield name="ENABLE_BINNING" pos="0" type="boolean"/>   <!-- best-guess -->
   <bitfield name="ENABLE_VISBUF"  pos="1" type="boolean"/>   <!-- best-guess -->
   <bitfield name="CLEAR_MODE"     pos="2" type="boolean"/>   <!-- bit set by Mesa during fast clear -->
   <bitfield name="LRZ_INV"        pos="7" type="boolean"/>   <!-- bit set during clear, may relate to LRZ -->
</reg32>
```

### fd2_gmem.c
- Change `use_hw_binning()` to return true for A22X under conditions (no depth_export, no discard).
- In `fd2_emit_tile_init`, allocate 8 VSC pipe BOs (start 64KB each, growable).
- Compute VSC_BIN_SIZE = (tile_w << 0 | tile_h << 5) — width/height shifted by 5 per xml.
- Emit `A220_VSC_ENABLE = 1` and `VSC_BIN_SIZE` once per batch.

### fd2_draw.c
- In `clear_state_restore` keep LRZ_VSC_CONTROL=0 for clears (current behaviour).
- New `fd2_emit_binning_prelude()` called per binning pass: writes `A220_VSC_ENABLE=1, LRZ_VSC_CONTROL=3, SQ_GPR_MANAGEMENT=0x7f010`, then WFI.

### fd2_program.c / ir2 (only if Fork A or B)
- Detect binning pass at emit time.
- If binning, generate a binning shader variant that strips PS outputs (no fragment work) but keeps VS position outputs.
- This is the same pattern Mesa A20X already does via `prog->binning_variant`.

### fd2_resource / freedreno_resource
- Per-batch VSC pipe BOs need to track lifetime against batches and resize on growth signal.

## 11. Estimated effort (revised post-Phase-1)

- Fork C (minimum viable): 1-2 days build + 1 day testing
- Fork A or B full port: 1 week implementation + 1 week test/iterate
- Total to working hw binning: 2-3 weeks if no major surprises
