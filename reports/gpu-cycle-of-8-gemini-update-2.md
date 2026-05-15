# A22X cycle-of-8: update for Gemini after 0050 + 0046 cleanup

## Quick recap

The deterministic period-8 rendering nondeterminism on Adreno 220 / HP TouchPad
(mainline 6.18 + Mesa 26.1 freedreno) is unchanged: 100-cap test still produces
8 unique pixel hashes in the pattern `ABCDEFGH ABCDEFGH ABCDEFGH ...`, with only
1/8 (`5adc3160...`) being the correct gradient. The other 7/8 show channel-drop
(missing red OR blue, never green) plus stale GMEM tile artifacts.

Per your prior analysis: only `CP_DRAW_INDX` advances the SQ wavefront slot
allocation pointer; we exhausted kernel-side preamble experiments (CP_NOP,
CP_INVALIDATE_STATE 0x300/0x7fff, CP_LOAD_CONSTANT_CONTEXT, every cache-flush
event) and confirmed none advance it.

## What we tested since the last update

### 1. Mesa userspace dummy-draw scrub (patch 0050) — HANGS GPU

Implemented your recommended approach: 8x dummy `CP_DRAW_INDX` packets
(count=2, immediate-mode count=0 vertices, point primitive) at the END of
`fd2_emit_restore`, mirroring KGSL's `build_sys2gmem_cmds` for LEIA REV470:
`solid_prog` + `solid_vertexbuf` (offset 0x9c), `RB_COLOR_MASK=0xF`,
`RB_DEPTHCONTROL=0x8`, `RB_MODECONTROL=0x4`, `PA_CL_VTE_CNTL=0xb00`,
`PA_CL_CLIP_CNTL=0x10000`, `A220_RB_LRZ_VSC_CONTROL=0`, scissors set
0..0x1fff, viewport ZSCALE=0xbf800000, `VGT_MAX_VTX_INDX=3`, then
`CP_WAIT_FOR_IDLE` + `fd_context_all_dirty(ctx)`.

**Result**: GPU hard-hangs (CP packet parser desync, MMU fault on undefined
IOVA). Diagnosed cause: **no render target is bound at `fd2_emit_restore`
time** — that hook fires on per-batch state restore, before the user IB sets
up the framebuffer / RB color buffer. The dummy `DRAW_INDX` fires but the RB
back-end has no valid color destination, hangs the pipeline.

Set aside; the right place to emit the scrub is *after* the user has bound
their render target but *before* their first real draw. We'd need a different
hook (per-batch first-draw insertion, or per-context-creation in a context
that already has a render target — neither exists naturally in mainline
freedreno).

### 2. Cache-flush bisect (patch 0046) — fault root cause found

Original 0046 emitted three events at the start of `fd2_emit_restore`:
`CACHE_FLUSH_AND_INV_EVENT` (0x16), `VS_FETCH_DONE` (0x1b), `SC_WAIT_WC`
(0x09), all `CP_EVENT_WRITE` count=1 with no payload. Produced a deterministic
`0xc000428b` GPUMMU fault.

Hypothesis (now confirmed by elimination): `VS_FETCH_DONE` on A2XX uses CP
scratch registers `CP_ME_VS_FETCH_DONE_SRC/ADDR/DATA` (0x0612-0x0614) for an
internal write. If those scratch regs hold stale/zero values from a prior
context, firing the event makes the CP dereference garbage → IOMMU fault at
the cached address.

Replaced 0046 to emit only `CACHE_FLUSH_AND_INV_EVENT` (which is already used
in `fd2_draw.c` count=1 with no payload, so known-safe).

**Result**: MMU faults: 0. Boots cleanly, full functional rendering, all 100
captures complete. Cycle-of-8 frequency unchanged — 4 hashes at 13/100 + 4
hashes at 12/100, "good" hash at ~12.5% (1/8). **Confirms cache pollution is
not the cycle-of-8 mechanism**.

## Current state

Active Mesa patches: `0001` (is_a22x helper) + `0002` (scheduler instr limit)
+ `0040` (`CP_WAIT_FOR_IDLE` at top of `fd2_emit_restore`) + `0045` (clear
color via PS CONST[0]) + `0046` (cache flush+inv at batch start, A22X only).

This is the cleanest "minimal Mesa + safe sync primitives" baseline. Full
build + 100-cap result: `ABCDEFGH ABCDEFGH ABCDEFGH`, 1/8 correct, 0 MMU faults.

## Questions for you

1. **Where can dummy `DRAW_INDX` actually fire safely?**

   `fd2_emit_restore` is the only batch-start hook in mainline freedreno, but
   it runs before the user binds the render target. Options we've considered:
   - Insert right before the user's first `DRAW_INDX` in the IB (requires
     command-buffer scanning we don't currently do)
   - Emit in `fd_context_init` once at context creation, with a host-side
     dummy framebuffer bound via Mesa's `solid_prog` clear path
   - Emit at `do_blit_using_3d` time (the existing 3D-blit path already sets
     up a render target — would the wavefronts fire for blits the same way
     they do for user draws?)

   Which of these would you trust to actually advance the SQ slot allocator,
   and which is least likely to produce the same "missing render target"
   hang we hit at `fd2_emit_restore` time?

2. **Is the SQ slot allocator advanced by GMEM-tile clear/resolve `DRAW_INDX`?**

   Mesa's GMEM path emits `CP_DRAW_INDX` for tile clears and tile resolves
   (via `solid_prog`). If those count toward slot advancement, then a fresh
   batch on a fresh DRM context already does many `DRAW_INDX` operations
   before the user IB runs — and the slot pointer would already be scrubbed.
   If not, what distinguishes "real" user `DRAW_INDX` from "internal" Mesa
   `DRAW_INDX` from the SQ's perspective?

3. **Could the cycle-of-8 be a different mechanism entirely?**

   We've now ruled out:
   - All kernel-side CP preamble approaches (no `DRAW_INDX` reach the SQ)
   - Mesa cache pollution (cleanup via 0046 doesn't move the needle)
   - All 22 of our experimental Mesa patches except 0001/0002/0040/0045/0046
     (bisected to confirm none of them affect the 1/8 frequency)

   The remaining mechanisms we can think of:
   - SQ slot scheduler is the cause and only `DRAW_INDX` from a properly
     bound render target advances it (your prior analysis)
   - Period-8 timing alignment with some external clock (memory bus, ICC
     fabric, RPM tick, MDP refresh) — but the period-8 is *deterministic*
     across runs, so a free-running clock seems unlikely
   - GMEM tile address modulo-8 aliasing (some GMEM allocator state that
     lands the user IB on whichever of 8 starting offsets the previous
     client left)

   Anything we're missing? The deterministic-pattern + always-7/8-broken +
   slot-1-correct fingerprint is unusually clean — what hardware feature
   on A22X has 8 elements and no software-visible reset?

## Reference data we can pull on request

- Full PM4 cmdstream `.rd` dumps for "good" vs "bad" runs (already confirmed
  byte-identical — md5 match across 10 runs)
- Register dumps from KGSL "good" path (legacy webOS kernel snapshot)
- KGSL `build_sys2gmem_cmds` source for LEIA REV470
- Decompiled HTC vendor blob (libqcamera) for vendor-specific A22X init
- Full `auto-test.sh` pipeline + bisect history (~30 patches tested,
  verbatim output for each)

Let us know which of these would be most useful and we'll prep them.
