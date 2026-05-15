# A22X hw_binning Phase 2 — Full Implementation Plan

**Date:** 2026-05-11
**Status:** Drafting after Fork D + dump diagnostic confirmed:
- Binner can be engaged cleanly via prelude (0xC00=1, LRZ_VSC_CONTROL=3, SQ_GPR pulse)
- Engagement alone produces no visibility output (BOs stay all-zero)
- Period-8 cycle is in latched HW state, unreachable from any register pulse
- Real binning IB (VS-only draws while binner engaged) is required

## Discovery: Mesa already has most infrastructure

Surveying `fd2_program.c`, `fd2_draw.c`, `fd2_emit.c`, `fd2_gmem.c`:

**Already implemented (A20X path):**
- `batch->binning` — separate ringbuffer for VS-only binning pass (allocated in `freedreno_batch.c:100`)
- `fd2_program_emit()` — detects binning via `ring == batch->binning`, skips FS emission (line 154-175, 200). Variant 0 is the VS-only variant.
- `fd2_emit_state_binning()` — emits subset of state (program, VS constants, viewport, blend, PA_SU_SC_MODE_CNTL with FACE_KILL) into batch->binning
- `fd2_draw_vbo` — when binning is active, also emits per-draw cmds into batch->binning (fd2_draw.c:448, 456)
- `fd2_emit_vertex_bufs(batch->binning, ...)` — vertex buffer setup for binning pass
- `fd2_emit_ib(ring, batch->binning)` at fd2_gmem.c:939 — splices binning IB into gmem ring before tile loop
- `use_hw_binning()` returns true if conditions met — currently gates A22X with `if (!is_a20x) return false;`
- `patch_draws(USE_VISIBILITY)` — patches draws to use visibility data
- `CP_SET_DRAW_INIT_FLAGS` per tile — points hardware at the current tile's VSC pipe BO for visibility filtering (fd2_gmem.c:1015)

**A20X-specific (NOT needed for A22X):**
- Shader EXEC-CF patching for memexport (fd2_gmem.c:679-696) — A20X writes visibility via shader memexport; A22X uses hw binner instead
- Memexport address constants at 0x0000000C (fd2_gmem.c:701-726) — A20X-only
- Clip transform constants at 0x0000018C — may or may not be needed for A22X; check at impl time

**A22X-specific (NEW):**
- The prelude (currently in our Fork D patch 0093):
  - `OUT_PKT0 reg 0xC00 = 1` (VSC enable)
  - `LRZ_VSC_CONTROL = 3` (engage binner)
  - `SQ_GPR_MANAGEMENT = 0x7f010` pulse + restore
  - `VSC_BIN_SIZE` + `VSC_PIPE[0..7]` config
- Binner disengage at end: `LRZ_VSC_CONTROL = 0`

## Implementation plan

### Step 1: Move the prelude from gmem ring to binning ring

The current Fork D prelude lives in `fd2_emit_tile_init` which writes to `batch->gmem`. The binner needs to be engaged DURING the binning IB execution, not later.

Move the prelude emission to a new function called from inside the `use_hw_binning(batch)` block in `fd2_emit_tile_init`:

```c
if (use_hw_binning(batch)) {
    if (is_a22x(ctx->screen)) {
        /* Engage A22X hw binner before running binning IB */
        emit_a22x_binning_prelude(ring);    /* writes to gmem ring */
        fd2_emit_ib(ring, batch->binning);  /* run the binning pass */
        emit_a22x_binning_postlude(ring);   /* LRZ_VSC_CONTROL=0 to disengage */
    } else {
        /* existing A20X path */
        ...
    }
}
```

The prelude must run on `batch->gmem` (the main ring) just before `fd2_emit_ib(ring, batch->binning)` so the binner is engaged when the binning draws fire.

### Step 2: Remove the A20X-only gate in `use_hw_binning`

Replace:
```c
if (!is_a20x(batch->ctx->screen))
    return false;
```

With:
```c
if (!is_a20x(batch->ctx->screen) && !is_a22x(batch->ctx->screen))
    return false;
```

(Or just remove the gate entirely and rely on the A20X/A22X split inside the `if (use_hw_binning)` block to do the right thing per generation.)

### Step 3: Verify binning shader variant works for A22X

`fd2_program_emit` already emits VS-only when `ring == batch->binning`. The binning variant is created at `ir2_compile(vp, variant=1, fp)` for non-binning use; for binning, variant 0 (the VS with no FS-specific patches) is used.

A22X may need different VS register allocation than A20X for the binning pass. webOS sets `SQ_GPR_MANAGEMENT = 0x7f010` (VTX=127, PIX=1) specifically for binning. Mesa's existing `fd2_program_emit` doesn't change SQ_GPR_MANAGEMENT per pass — we override it in the prelude (Step 1) before binning IB fires, and restore it in the postlude.

**Risk:** the binning shader (variant 0) may have register usage that conflicts with the 0x7f010 partition (VTX=127 expects up to 127 GPRs available for VS). If the binning shader was compiled assuming the regular partition (64/64), allocating it 127 VTX GPRs should be fine (more is okay). The PIX=1 may cause issues if the shader still emits PS code — but `fd2_program_emit` skips PS emission for binning, so this should be safe.

### Step 4: Per-tile visibility filtering

The existing A20X path emits `CP_SET_DRAW_INIT_FLAGS` per tile at `fd2_gmem.c:1015`, pointing the rasterizer at the current tile's VSC pipe BO. This should work as-is for A22X — the binner wrote visibility data, the rasterizer reads it.

Need to confirm:
- A22X uses same `CP_SET_DRAW_INIT_FLAGS` opcode (0x4b)
- A22X visibility-stream format matches A20X (likely yes, given identical VSC_PIPE register layout)

### Step 5: Handle Mesa's existing A20X-specific blocks gracefully

The `use_hw_binning(batch)` block in `fd2_emit_tile_init` has:
- Shader EXEC-CF patching (lines 688-696) — A20X memexport patching, SKIP for A22X
- Memexport address constants (lines 701-726) — SKIP for A22X
- Clip transform constants (lines 728-757) — may need for A22X, check empirically
- VGT_VERTEX_REUSE disable/restore (lines 761-772) — A22X may also benefit, keep

Wrap the A20X-specific bits in `if (is_a20x(ctx->screen)) { ... }` so A22X skips them.

## Effort estimate

- Step 1 (prelude relocate): 2 hours
- Step 2 (gate change): 5 minutes
- Step 3 (shader variant test): 4 hours (mostly testing)
- Step 4 (per-tile DRAW_INIT_FLAGS): 2 hours (should work as-is from A20X path)
- Step 5 (skip A20X-only blocks): 1 hour
- Iteration / debugging: 1-2 days

**Total:** 2-3 days of focused work. The infrastructure is mostly there; we're adding an A22X-aware branch to existing code, not building from scratch.

## Risks

1. **Binning shader compile failure** — VS variant 0 may have been designed for A20X memexport and depend on it. If freedreno's ir2 emits memexport CFs into the binning variant regardless of A22X, those will likely hang or no-op. Need to verify via decomp or first build.

2. **Visibility stream format mismatch** — if A22X's hw binner writes visibility in a different format than A20X's shader memexport, the rasterizer reading via CP_SET_DRAW_INIT_FLAGS may produce wrong filtering. webOS proves the format is correct for A22X, so worst case we match webOS exactly.

3. **Need additional A22X-specific state** — the webOS render-pass cmdbuf (mode-1) includes PA_SC_AA_CONFIG, A220_RB_SAMPLE_POS — we may need to emit these too in our render path.

## Next deliverable

After current Fork D + disengage test:
- If single-render still produces deterministic period-8 (expected) → start Step 1+2 immediately
- If anything changes → re-evaluate

The disengage may actually fix the hang in double-render mode AND change single-render behavior (since the binner now starts each batch from a clean state). Worth running both single and double-render tests.
