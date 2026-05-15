# Vendor A2XX libGLESv2 cmdbuffer API reference

**Source:** Decompiled libGLESv2.so from HTC, Samsung, Xiaomi (all Adreno 220 / "Leia" / REV470) and webOS (HP TouchPad).
**Decomp tool:** Ghidra
**Decomp location:** `reports/ghidra-decomp/decomp-txt/`
**Date:** 2026-05-13

This document catalogues the **standard QCom A2XX usermode driver cmdbuffer API**.  All four vendor binaries share an identical helper set with effectively-identical implementations — strongly suggests one common QCom usermode reference codebase, lightly customised per OEM.  Mesa freedreno has none of these helpers; it emits raw PM4 packets directly via `OUT_PKT3`/`OUT_PKT0`/`OUT_RING`.

Vendor binary also contains `a4x_*` functions (Adreno 4xx variant) — the lib is a unified driver supporting multiple Adreno generations.

---

## Table of contents

1. [Function inventory by family](#1-function-inventory-by-family)
2. [Cmdbuffer-primitive helpers (full pseudo-code)](#2-cmdbuffer-primitive-helpers-full-pseudo-code)
3. [Register-setter helpers (yamato_set_hw_*)](#3-register-setter-helpers-yamato_set_hw_)
4. [Preamble/postamble state machine](#4-preambleambl-state-machine)
5. [Primitive helpers (draw/clear/resolve)](#5-primitive-helpers-drawclearresolve)
6. [Cmdbuffer building lifecycle](#6-cmdbuffer-building-lifecycle)
7. [Mesa equivalence table](#7-mesa-equivalence-table)
8. [Implications for 0x0ee2 cycle](#8-implications-for-0x0ee2-cycle)

---

## 1. Function inventory by family

### 1.1 `leia_cmdbuffer_*` — cmdbuffer primitive emitters (~12 functions, ALL present in all 4 vendors)

| Function | Size | Returns dwords | Purpose |
|----------|------|----------------|---------|
| `leia_cmdbuffer_insertnop(buf, n)` | 42 | n + 1 | CP_NOP × n |
| `leia_cmdbuffer_insertwaitforidle(ctx, buf)` | 110 | 2 or 6 | CP_WAIT_FOR_IDLE (stateful with fence-write barrier every 32 ops) |
| `leia_cmdbuffer_inserttexcacheinvalid(buf)` | 14 | 2 | PM4 type-0 write of 0x1 to TC_CNTL_STATUS (0x0e00) |
| `leia_cmdbuffer_insertindirectbuffer(buf, addr, size)` | 12 | 3 | CP_INDIRECT_BUFFER |
| `leia_cmdbuffer_insertgpumemwrite(buf, addr, data, n)` | 36 | 2 + n | CP_MEM_WRITE of n dwords |
| `leia_cmdbuffer_insertinterrupt(buf)` | 14 | 2 | CP_INTERRUPT (host notify) |
| `leia_cmdbuffer_indirectpreamble(ctx)` | 42 | varies | Pre-IB wrapper, emits gpu_spam(2,0) |
| `leia_cmdbuffer_indirectpostamble(ctx)` | 42 | varies | Post-IB wrapper, emits gpu_spam(2,1) |
| `leia_cmdbuffer_gpu_spam(ctx, type, arg)` | 354 | ~30-50 | The "magic" state-init block called by pre/postamble |
| `leia_cmdbuffer_presubmit()` | 2 | 0 | Stub on most vendors |
| `leia_cmdbuffer_reset(ctx, *flag)` | 32 | 0 | Reset preamble buffer tracking |
| `leia_cmdbuffer_size*(...)` | 4 | constant | Size predicate companions for each insert* |

### 1.2 `leia_preamble_*` — preamble buffer management (~10 functions)

| Function | Size | Purpose |
|----------|------|---------|
| `leia_preamble_init(ctx)` | 82 | Initialize preamble subsystem |
| `leia_preamble_init_full_state(ctx)` | 234 | Build full register state into preamble buffer |
| `leia_preamble_init_register_state(ctx)` | 116 | Initialize register tracking |
| `leia_preamble_allocate_state_resources(ctx)` | 208 | Allocate state BOs |
| `leia_preamble_free_state_resources(ctx)` | 112 | Free state BOs |
| `leia_preamble_destroy(ctx)` | 34 | Tear down preamble |
| `leia_preamble_calc_reg_ranges(ctx)` | 34 | Compute register range bookkeeping |
| `leia_preamble_fill_register_cmdbuffer(ctx)` | 100 | Emit register writes from cached state into cmdbuffer |
| `leia_preamble_update_state(ctx, mode, count, buf, ...)` | 186 | Update preamble's cached register state |
| `leia_preamble_update_contextregs(ctx)` | 112 | Push context register cache |
| `leia_preamble_update_globalregs(ctx)` | 76 | Push global register cache |
| `leia_preamble_reset()` | 60 | Reset preamble between submits |
| `leia_preamble_indirect_append(buf)` | 46 | Append indirect buffer's state to preamble |

**Key insight:** Vendors maintain an **explicit per-context preamble buffer** that caches GPU register state.  Each `CP_INDIRECT_BUFFER` call's register writes are also recorded into this preamble buffer, so subsequent IBs can be replayed without re-emitting state.  Mesa freedreno doesn't have an analogous structure.

### 1.3 `yamato_set_hw_*` — A2XX register setter helpers (26 functions)

| Function | Size | Register |
|----------|------|----------|
| `yamato_set_hw_pa_sc_window_scissor_reg` | 22 | PA_SC_WINDOW_SCISSOR_TL/BR |
| `yamato_set_hw_rb_color_mask_reg` | 20 | RB_COLOR_MASK |
| `yamato_set_hw_rb_blend_color_reg` | 30 | RB_BLEND_RED/GREEN/BLUE/ALPHA |
| `yamato_set_hw_rb_fog_color_reg` | 20 | RB_FOG_COLOR |
| `yamato_set_hw_rb_stencil_ref_mask_reg` | 22 | RB_STENCILREFMASK |
| `yamato_set_hw_rb_alpha_ref_reg` | 20 | RB_ALPHA_REF |
| `yamato_set_hw_pa_cl_viewport_xy_scale_offset_reg` | 30 | PA_CL_VPORT_X/YSCALE/OFFSET |
| `yamato_set_hw_pa_cl_viewport_z_scale_offset_reg` | 24 | PA_CL_VPORT_ZSCALE/OFFSET |
| `yamato_set_hw_rb_blend_control_reg` | 18 | RB_BLEND_CONTROL |
| `yamato_set_hw_rb_color_control_reg` | 20 | RB_COLORCONTROL |
| `yamato_set_hw_pa_su_sc_mode_cntl_reg` | 18 | PA_SU_SC_MODE_CNTL |
| `yamato_set_hw_pa_su_line_cntl_reg` | 22 | PA_SU_LINE_CNTL |
| `yamato_set_hw_pa_su_point_size_reg` | 18 | PA_SU_POINT_SIZE |
| `yamato_set_hw_pa_su_poly_offset_reg` | 40 | PA_SU_POLY_OFFSET (FRONT_SCALE/OFFSET, BACK_SCALE/OFFSET) |
| `yamato_set_hw_pa_su_vtx_cntl_reg` | 20 | PA_SU_VTX_CNTL |
| `yamato_set_hw_sq_interpolator_cntl_reg` | 22 | SQ_INTERPOLATOR_CNTL |
| `yamato_set_hw_pa_cl_clip_cntl_reg` | 18 | PA_CL_CLIP_CNTL |
| `yamato_set_hw_pa_cl_vte_cntl_reg` | 20 | PA_CL_VTE_CNTL |
| `yamato_set_hw_pa_sc_aa_mask_reg` | 20 | PA_SC_AA_MASK |
| `yamato_set_hw_pa_sc_line_cntl_reg` | 22 | PA_SC_LINE_STIPPLE |
| `yamato_set_hw_pa_cl_gb_clip_disc_adj_reg` | 30 | PA_CL_GB_VERT_CLIP/DISC_ADJ |
| `yamato_set_hw_rb_surface_regs` | 28 | RB_SURFACE_INFO etc. |
| `yamato_set_hw_rb_depth_control_reg` | 64 | RB_DEPTHCONTROL |

Plus `yamato_context_create`/`yamato_context_destroy`/`yamato_perform_resolve` (3488 bytes — tile resolve).

### 1.4 `leia_set_hw_*` — same family with Leia-specific helpers (~20 functions)

Effectively duplicates of `yamato_set_hw_*` with A22X-specific extensions.  Plus `leia_set_hw_gras_clipplane_dummy` (a "clipplane dummy" emission unique to A22X).

### 1.5 `leia_primitive_*` — high-level draw/clear/resolve

| Function | Size | Purpose |
|----------|------|---------|
| `leia_primitive_clear` | 2088 | glClear path |
| `leia_primitive_drawarrays` | 1100 | glDrawArrays |
| `leia_primitive_drawelements` | 1574 | glDrawElements |
| `leia_primitive_query_xform_buffer` | 4 | Transform feedback query |
| `leia_process_primitive_flags` | 876 | Pre-draw flag handling |

`leia_primitive_drawelements_uncached` (2208 bytes in webOS) is the actual draw implementation.

### 1.6 `leia_perform_resolve` (4504 bytes) — tile resolve

The "blob driver resolve" — what mesa calls `gmem2mem`.  Massive function with full state management.  References: `yamato_perform_resolve` (3488 bytes, simpler / A2XX-only).

### 1.7 `leia_init_hw` (1114 bytes) — boot-time HW init

Called once at GPU init.  Emits initial register state.  Likely the "all 16 slots get initialized" sequence we hypothesized.

### 1.8 `leia_gpuprogram_*` — shader loading

| Function | Size | Purpose |
|----------|------|---------|
| `leia_gpuprogram_loadconstants` | 476 | Upload uniform constants |
| `leia_gpuprogram_submitconstants` | 424 | Emit CP_LOAD_CONSTANT packets |
| `leia_gpuprogram_submitsamplers` | 228 | Emit sampler bindings |
| `leia_loadexecutable` | 894 | Load shader code into SQ inst-store |

### 1.9 `leia_binning_*` — hw binner config

| Function | Size | Purpose |
|----------|------|---------|
| `leia_binning_grow_vis_stream_buffer` | 160 | Dynamic VSC buffer resize |
| `leia_configure_binid_groups` | 554 | VGT bin-ID grouping |

### 1.10 Patches/format conversion helpers

| Function | Size | Purpose |
|----------|------|---------|
| `leia_patch_vertex_shader` | 950 | VS patching for A22X |
| `leia_patch_clear_resolve_shader` | 342 | Clear/resolve shader patching |
| `leia_patch_blt3d_shader` | 498 | 3D blit shader patching |
| `leia_patch_blt3d_sampler` | 522 | 3D blit sampler patching |
| `leia_fmt_to_*` (×8 functions) | ~20-130 each | Format conversion helpers |

### 1.11 a4x_* — Adreno 4xx variant (in same binary)

`a4x_cmdbuffer_*`, `a4x_set_hw_*`, `a4x_perform_resolve` — the unified driver also handles Adreno 4xx.  Mesa freedreno has separate per-gen code; QCom's lib is unified.

---

## 2. Cmdbuffer-primitive helpers (full pseudo-code)

### 2.1 `leia_cmdbuffer_insertnop(buf, n)` — CP_NOP packet

```c
uint *leia_cmdbuffer_insertnop(uint *buf, int n)
{
    /* Header: CP_TYPE3_PKT | (n-1)<<16 | (CP_NOP=0x10)<<8 */
    buf[0] = (n - 1) << 16 | 0xc0001000;
    for (int i = 1; i <= n; i++)
        buf[i] = 0;
    return buf + n + 1;
}
```

### 2.2 `leia_cmdbuffer_inserttexcacheinvalid(buf)` — TC_CNTL_STATUS=1

```c
undefined4 *leia_cmdbuffer_inserttexcacheinvalid(undefined4 *buf)
{
    /* PM4 type-0: write 1 dword to reg 0x0e00 (TC_CNTL_STATUS) */
    buf[0] = 0x00000e00;   /* CP_TYPE0_PKT | (count-1=0)<<16 | regidx=0x0e00 */
    buf[1] = 0x00000001;   /* A2XX_TC_CNTL_STATUS_L2_INVALIDATE bit 0 */
    return buf + 2;
}
```

**Mesa equivalent (doesn't exist on A22X path, testing in patch 0017):**
```c
OUT_PKT0(ring, REG_A2XX_TC_CNTL_STATUS, 1);
OUT_RING(ring, A2XX_TC_CNTL_STATUS_L2_INVALIDATE);
```

### 2.3 `leia_cmdbuffer_insertindirectbuffer(buf, addr, size)` — CP_INDIRECT_BUFFER

```c
undefined4 *leia_cmdbuffer_insertindirectbuffer(
    undefined4 *buf, uint addr, uint size_dwords)
{
    /* DAT_00091e50 = 0xc0023f00 = CP_TYPE3_PKT | (3-1)<<16 | CP_INDIRECT_BUFFER<<8 */
    buf[0] = 0xc0023f00;
    buf[1] = addr;
    buf[2] = size_dwords;
    return buf + 3;
}
```

Same encoding Mesa uses.

### 2.4 `leia_cmdbuffer_insertgpumemwrite(buf, addr, data, n)` — CP_MEM_WRITE

```c
uint *leia_cmdbuffer_insertgpumemwrite(
    uint *buf, uint addr, uint *data, int n)
{
    /* Header: CP_TYPE3_PKT | n<<16 | (CP_MEM_WRITE=0x3d)<<8 */
    buf[0] = n << 16 | 0xc0003d00;
    buf[1] = addr;
    memcpy(buf + 2, data, n * 4);
    return buf + 2 + n;
}
```

Mesa freedreno A2XX does **not** use CP_MEM_WRITE for sync — vendors use it for fence writes.

### 2.5 `leia_cmdbuffer_insertinterrupt(buf)` — CP_INTERRUPT

```c
undefined4 *leia_cmdbuffer_insertinterrupt(undefined4 *buf)
{
    /* DAT_00091eb4 = 0xc0004000 = CP_TYPE3_PKT | (1-1)<<16 | (CP_INTERRUPT=0x40)<<8 */
    buf[0] = 0xc0004000;
    buf[1] = 0x40000000;   /* Mask: trigger CP_INTERRUPT for context completion */
    return buf + 2;
}
```

Mesa freedreno does not emit CP_INTERRUPT.

### 2.6 `leia_cmdbuffer_insertwaitforidle(ctx, buf)` — STATEFUL wait

```c
undefined4 *leia_cmdbuffer_insertwaitforidle(int ctx, undefined4 *buf)
{
    cmdbuf_state_t *state = ctx->cmdbuf_state;  /* offset 0x1664 */
    
    if (!(ctx->flags & 4)) {                    /* offset 0xfa8, bit 2 */
        if (state->idle_counter > 0x20) {
            /* Every 32 idle ops: emit fence-write barrier instead */
            buf[0] = HDR_CP_MEM_WRITE_OR_EVENT_TS;  /* DAT_00091e1c */
            buf[1] = 0;
            buf[2] = 0x4000;                    /* flag */
            buf[3] = 0;
            buf[4] = fence_target_addr;         /* compute from state */
            buf[5] = 0;
            state->idle_counter = 0;
            return buf + 6;
        }
        state->idle_counter++;
    }
    
    /* Default: simple 2-dword form */
    buf[0] = 0xc0002600;   /* CP_TYPE3_PKT | (1-1)<<16 | CP_WAIT_FOR_IDLE<<8 */
    buf[1] = 0;
    return buf + 2;
}
```

**Critical:** vendors emit a **fence-write barrier every 32 idle operations**.  Mesa always emits the simple 2-dword `CP_WAIT_FOR_IDLE`.

### 2.7 `leia_cmdbuffer_indirectpreamble(ctx)`

```c
void leia_cmdbuffer_indirectpreamble(int ctx)
{
    cmdbuf_state_t *state = ctx->cmdbuf_state;
    
    /* If SQ inst-store partition has changed, repartition */
    if (state->cur_partition != state->prev_partition) {
        leia_repartition_instruction_store(ctx, 0);
    }
    
    /* Emit the BIG "magic" state-init block (preamble variant) */
    leia_cmdbuffer_gpu_spam(ctx, 2, 0);
}
```

### 2.8 `leia_cmdbuffer_indirectpostamble(ctx)`

```c
void leia_cmdbuffer_indirectpostamble(int ctx)
{
    cmdbuf_state_t *state = ctx->cmdbuf_state;
    
    /* Emit the BIG state-init block (postamble variant) */
    leia_cmdbuffer_gpu_spam(ctx, 2, 1);
    
    /* Snapshot current partition as prev */
    state->prev_partition = state->cur_partition;
    
    /* If a preamble buffer exists, append the IB state to it for replay */
    if (ctx->preamble_buf != NULL) {
        leia_preamble_indirect_append(ctx->preamble_buf);
    }
}
```

### 2.9 `leia_cmdbuffer_reset(ctx, *flag)`

```c
void leia_cmdbuffer_reset(int ctx, int *flag)
{
    cmdbuf_state_t *state = ctx->cmdbuf_state;
    state->prev_partition = state->cur_partition;
    
    if (ctx->preamble_buf != NULL && *flag == 1)
        leia_preamble_reset();
}
```

### 2.10 `leia_repartition_instruction_store(ctx, mode, partition)`

Emits 11 dwords across 3 register writes (each via CP_SET_CONSTANT):

```c
/* Reg 0x05d0 (REG_A2XX_SQ_INST_STORE_MANAGMENT) */
emit_set_constant(0x05d0, value0 = 0);
/* Reg 0x0d02 (unknown — perhaps SQ partition control) */
emit_set_constant(0x0d02, partition | (ctx->5ec << 16));
/* Reg 0x0300 (unknown — perhaps a binner / VSC register) */
emit_set_constant(0x0300, partition | (ctx->5ec << 16) | 0x80000000);
```

Then calls `leia_preamble_update_state` to cache the writes for replay.

### 2.11 `leia_cmdbuffer_gpu_spam(ctx, type, arg)` — full decode

**Conclusion (2026-05-13):** `gpu_spam` is **primarily diagnostic instrumentation**, NOT the magic cycle-fix block we hoped.  See breakdown below.

Full decompilation (xiaomi, all 354 bytes):

```c
void leia_cmdbuffer_gpu_spam(int ctx, undefined4 type, int arg, undefined4 p4)
{
    /* Read a global state struct (likely the GPU debug capture state).
     * inst_store_off = state->offset_2c (some flags / pointer)
     */
    int inst_store_off = *(int *)(**(int **)(GLOBAL_STATE) + 0x2c);
    int shifted = inst_store_off << 0x14;   /* shift bit 11 into MSB */
    
    /* Gate: only do work if bit 11 of inst_store_off is set (instrumentation
     * enabled).  When the debug capture system is off, gpu_spam is a no-op.
     */
    if (shifted >= 0)
        return;
    
    /* === Path 1: postamble vs preamble === */
    if (arg == 1) {
        /* POSTAMBLE: reserve 8 dwords in ringbuffer + emit waitforidle */
        uint4 ptr = rb_cmdbuffer_addcmds_immediate(ctx, 8, ...);
        leia_cmdbuffer_insertwaitforidle(ctx, ptr);
        /* The 8 dwords from addcmds_immediate are pre-zeroed; insertwaitforidle
         * writes 2 dwords (simple) or 6 dwords (fence-write barrier) at ptr.
         * Remaining dwords are zeros (= dead TYPE_0 single-dword packets).
         */
    } else {
        /* PREAMBLE (arg == 0): just bump a global counter, no cmd emission */
        GLOBAL_GPU_SPAM_PREAMBLE_COUNTER++;
    }
    
    /* === Path 2: ringbuffer wrap check === */
    if ((ctx->1710 + 7) * 4 > 0x10000) {
        ctx->1710 = 0;
        GLOBAL_WRAP_FLAG = 1;
    }
    
    /* Per-context buffer cursor */
    int cursor_x4 = ctx->1710 * 4;
    int buf_a = ctx->16f0;                  /* Log buffer base 1 */
    int buf_b = ctx->16f4;                  /* Log buffer base 2 */
    int target = buf_b + cursor_x4;
    
    /* === Path 3: optional debug logging === */
    if (GLOBAL_LOGGING_ENABLED == 1) {
        FILE *log = ctx->170c;
        /* Decode struct at (buf_a + cursor_x4) and fprintf it:
         *   [0]: u32 first value
         *   [1]: u32 kind enum {0,1,2,3,4 -> 5 different string formats}
         *   [2]: u32 sub-kind enum {0,1,2,others -> 4 string formats}
         *   [3], [4], [5], [6]: u32 four data fields
         */
        ... fprintf-with-format-from-DAT_* calls ...
    }
    
    /* === Path 4: emit 13-dword telemetry CP_NOP packet === */
    puVar3 = rb_cmdbuffer_addcmds_immediate(ctx, 0xd, ...);
    
    puVar3[0] = DAT_00092020;       /* PM4 header — likely CP_NOP w/ count=12
                                      = 0xc00c1000 (TYPE_3, CP_NOP, count-1=12) */
    puVar3[1] = target;              /* log buffer pointer */
    puVar3[2] = *(GLOBAL_TIMESTAMP_OR_FLAG);
    puVar3[3] = type;                /* always 2 from indirectpre/postamble */
    puVar3[4] = arg;                 /* 0 (preamble) or 1 (postamble) */
    puVar3[5] = os_process_getid();  /* PID */
    puVar3[6] = ctx;                 /* context pointer */
    puVar3[7] = DAT_00092024;        /* constant */
    puVar3[8] = 0x447;               /* possibly PM4 type-0 reg write to 0x447? */
    puVar3[9] = target + 0x14;       /* address */
    puVar3[10] = DAT_00092024;       /* same constant as [7] */
    puVar3[11] = 0x447;              /* same as [8] */
    puVar3[12] = target + 0x18;      /* address+4 */
    
    ctx->1710 += 7;                  /* advance cursor by 7 (not 13!) */
    
    return;
}
```

### What the 13 dwords actually contain

Read as a 13-dword PM4 packet starting with CP_NOP header (0xc00c1000):
- **[0]**: CP_NOP header (count=12)
- **[1..12]**: 12 dwords of PAYLOAD that the CP IGNORES.

So `gpu_spam` emits a **CP_NOP with 12 dwords of diagnostic payload**.  The CP firmware skips the payload entirely.  The data is meant to be CAPTURED by debugging infrastructure (kgsl kernel driver or rd_dump tooling) for post-mortem analysis.

Evidence the payload is telemetry, not GPU state:
- Contains `os_process_getid()` (PID) — purely software/CPU concept, GPU doesn't care
- Contains `ctx` pointer — software bookkeeping
- Contains `type`/`arg` from caller — software metadata
- 0x447 repeated twice with adjacent buffer pointers — looks like log entry indices

### Cursor advances by 7, not 13

`ctx->1710 += 7` — only 7 not 13.  Suggests the log buffer slots are 7-dword fixed-size records, but the 13-dword cmdstream encoding includes a header + tail overhead.

### gpu_spam summary

| Path | Triggered when | Cmdstream effect | Cycle-fix potential |
|------|----------------|------------------|---------------------|
| Path 1 (postamble) | `arg == 1` | 8 dwords (zeros + WAIT_FOR_IDLE) | Maybe — the WAIT could matter |
| Path 1 (preamble) | `arg == 0` | None (just counter) | None |
| Path 2 (wrap) | cursor near 0x10000 | None (just state reset) | None |
| Path 3 (logging) | `GLOBAL_LOGGING == 1` | None (fprintf only) | None |
| Path 4 (telemetry) | always (if gated bit set) | 13 dwords (CP_NOP) | None (payload ignored by CP) |

**Conclusion:** `gpu_spam` is dominated by **diagnostic telemetry**, not GPU state-fixing emission.  The only path with potential cycle relevance is the postamble-only `WAIT_FOR_IDLE` (Path 1, `arg == 1`), and Mesa's existing CP_WAIT_FOR_IDLE emissions are functionally equivalent.

**This falsifies the "gpu_spam is the missing 16-slot scrub" hypothesis.**  The real architectural difference must be elsewhere — likely in the `leia_preamble_*` cache subsystem or the stateful `WAIT_FOR_IDLE` with periodic fence-write barriers.

### Open follow-up

- The stateful `WAIT_FOR_IDLE` (vendors emit fence-write barrier every 32 idle calls) is still an untested candidate.  Mesa always emits plain CP_WAIT_FOR_IDLE.
- The `leia_preamble_indirect_append` call in `indirectpostamble` — this is the only architectural-level call left we haven't decoded.  It might be appending IB content to the preamble cache for replay.
- If `gpu_spam` is diagnostic-only, vendors must be doing the "16-slot init" elsewhere — likely in `leia_init_hw` (1114 bytes, called once at context create).  That's the next decode target.

---

## 3. Register-setter helpers (yamato_set_hw_*)

Pattern: every visible HW register has a dedicated setter that:
1. Reads current state from `ctx + offset`
2. Constructs the new value
3. Emits PM4 packet (`CP_SET_CONSTANT` for >= 0x2000, `OUT_PKT0` for < 0x2000)
4. Updates state cache via `leia_preamble_update_state`

Each setter is 18-40 bytes, very repetitive.  Total of 26 yamato setters + 20 leia setters covers the full A2XX register surface.

**Mesa freedreno** lumps all state into a single `fd2_emit_state` function — much less granular.

---

## 4. Preamble/postamble state machine

The preamble subsystem (`leia_preamble_*`) maintains a **persistent cached register state** across submits.  Lifecycle:

1. **Context create:** `leia_preamble_init` + `leia_preamble_allocate_state_resources` allocate BOs for state storage.
2. **Per submit:**
   - `leia_cmdbuffer_reset(ctx, &flush)` — reset tracking
   - Build cmdbuffer with state setters (each one calls `leia_preamble_update_state` to cache the write)
   - On `CP_INDIRECT_BUFFER` call: `leia_cmdbuffer_indirectpreamble` + IB + `leia_cmdbuffer_indirectpostamble`
   - The pre/post-amble wrap each IB with `gpu_spam` and append the IB's register writes to the preamble buffer.
3. **Effect:** subsequent IBs in the same submit can be "replayed" without re-emitting state.

Mesa freedreno has none of this — every batch re-emits full state via `fd_context_all_dirty`.

---

## 5. Primitive helpers (draw/clear/resolve)

### `leia_primitive_clear` (2088 bytes)

The "blob clear" — emits clear quad with proper RB_COLOR_INFO/RB_COPY_DEST_INFO setup.  Mesa's `fd2_clear` is much shorter (mostly delegates to gmem path).

### `leia_primitive_drawelements` (1574 bytes) + `_uncached` (2208 bytes)

The actual `glDrawElements` implementation.  Performs:
- Format conversion (indices)
- Vertex shader patching (`leia_patch_vertex_shader` — 950 bytes!)
- Constant/sampler/texture binding
- Calls into `leia_cmdbuffer_*` helpers to emit PM4
- Per-draw `leia_cmdbuffer_indirectpreamble + insert + indirectpostamble` wrapping

### `leia_perform_resolve` (4504 bytes)

GMEM → sysmem resolve.  Equivalent to Mesa's `fd2_emit_tile_gmem2mem` but much more elaborate.

---

## 6. Cmdbuffer building lifecycle (vendor model)

```
INIT (once at context create):
  leia_context_create(ctx)
    ├── leia_init_hw(ctx)                          /* 1114 bytes, boot-time state */
    ├── leia_preamble_init(ctx)
    ├── leia_preamble_allocate_state_resources(ctx)
    └── leia_preamble_init_full_state(ctx)         /* emit initial register state */

PER SUBMIT (per glFlush / present):
  leia_cmdbuffer_presubmit()                       /* stub on most vendors */
  leia_cmdbuffer_reset(ctx, &reset_flag)           /* roll back state tracking */
  
  /* Build cmdbuffer with state setters: */
  yamato_set_hw_pa_sc_window_scissor_reg(ctx, scissor)
  yamato_set_hw_rb_blend_color_reg(ctx, color)
  ... (~20 register setters typical for a frame) ...
  
  /* Per IB call (binning IB + tile IB + tile_store IB + ...): */
  leia_cmdbuffer_indirectpreamble(ctx)
    ├── (if partition changed) leia_repartition_instruction_store(ctx, 0)
    └── leia_cmdbuffer_gpu_spam(ctx, 2, 0)         /* MAGIC preamble */
  leia_cmdbuffer_insertindirectbuffer(buf, ib_addr, ib_size)
  leia_cmdbuffer_indirectpostamble(ctx)
    ├── leia_cmdbuffer_gpu_spam(ctx, 2, 1)         /* MAGIC postamble */
    └── leia_preamble_indirect_append(preamble_buf)
  
  /* Per WAIT_FOR_IDLE: */
  leia_cmdbuffer_insertwaitforidle(ctx, buf)
    └── (every 32 calls) emit fence-write barrier instead
  
  /* Per coherency point: */
  leia_cmdbuffer_inserttexcacheinvalid(buf)        /* PM4 0x0e00 = 1 */
  
  /* On end of frame: */
  leia_cmdbuffer_insertinterrupt(buf)              /* CP_INTERRUPT */
```

---

## 7. Mesa equivalence table

| Vendor helper | Mesa A22X equivalent | Status |
|---------------|---------------------|--------|
| `insertnop` | `OUT_PKT3(ring, CP_NOP, n)` | ✓ |
| `insertindirectbuffer` | `OUT_IB(ring, sub)` | ✓ same encoding |
| `insertgpumemwrite` | (none in A2XX path) | ❌ |
| `insertinterrupt` | (none) | ❌ |
| `inserttexcacheinvalid` | (none, testing in patch 0017 = `FD2_END_TC_INV`) | ❌ |
| `insertwaitforidle` (stateful) | always-simple `OUT_PKT3(ring, CP_WAIT_FOR_IDLE, 1); OUT_RING(0)` | ❌ Mesa lacks fence-write barrier variant |
| `indirectpreamble` / `indirectpostamble` | (none) | ❌ |
| `gpu_spam` | (none) | ❌ — likely the missing piece |
| `repartition_instruction_store` | Mesa patch 0080 (single emission at fd2_emit_restore) | ◇ partial |
| `preamble_*` cache subsystem | (none) | ❌ Mesa just re-emits state per batch |
| `yamato_set_hw_*` × 26 registers | Inlined into `fd2_emit_state` etc. | ◇ different decomposition |
| `leia_init_hw` (1114 bytes) | `fd2_emit_restore` + kernel `a2xx_hw_init` | ◇ partial — both incomplete |

---

## 8. Implications for 0x0ee2 cycle

The biggest gap is the **`indirectpreamble + gpu_spam + indirectpostamble`** wrapper around each `CP_INDIRECT_BUFFER`.  Mesa freedreno A22X emits multiple IBs per render (binning IB, tile IB, tile_store IB) without any wrapper.  If 0x0ee2 is advanced by IB transitions and only RESET by gpu_spam's state-emission, the gap perfectly explains the period-16 cycle.

**Decoding `leia_cmdbuffer_gpu_spam` is the next high-value reverse-engineering target.**  354 bytes — needs follow-up Ghidra session to expand `DAT_*` constants, trace `rb_cmdbuffer_addcmds_immediate` callsites, and identify which registers gpu_spam touches.

Secondary candidate: **emit the stateful WAIT_FOR_IDLE pattern** (fence-write barrier every 32 ops) instead of Mesa's always-simple form.  May allow slots to drain properly between renders.

---

## 9. References

- All decompiled binaries: `reports/ghidra-decomp/decomp-txt/{webos,htc,samsung,xiaomi}_libGLESv2*.so.decomp.txt`
- 0x0ee2 cycle counter memory: `~/.claude/projects/.../memory/project_cycle_register_0xee2.md`
- Gemini analysis chain: `reports/gemini-summary-2026-05-13-{round3,round4,round5}.md`
- Mesa patches incorporating findings:
  - 0014: end-of-tile-loop event emission (CONTEXT_DONE / FACENESS_FLUSH / VS-PS_DEALLOC) — all falsified
  - 0015: 3-way bisect (NO_CACHE_FLUSH_INV, START_NOP_COUNT, END_CACHE_FLUSH_INV_COUNT) — falsified
  - 0016: per-draw DEALLOC — pending test
  - 0017: inline TC_CNTL_STATUS=L2_INVALIDATE — pending test
