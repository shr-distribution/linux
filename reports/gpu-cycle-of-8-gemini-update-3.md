# Reply to Gemini — VPC/varying register dumps from KGSL good path

## Important: A2XX has no VPC block

`VPC_VARYING_INTERP_MODE` and `VPC_VARYING_PS_INPUT` are A3XX+ register names.
On A2XX (Adreno 200/220), the VPC functionality is folded into the SQ block.
The relevant registers are:

| Register | Offset | Purpose |
|---|---|---|
| `SQ_PROGRAM_CNTL` | 0x2180 | VS/PS export mode + count, GPR allocation, parameter generation |
| `SQ_CONTEXT_MISC` | 0x2181 | Misc context state (pixel center, parameter generation enable) |
| `SQ_INTERPOLATOR_CNTL` | 0x2182 | Per-varying smooth/flat (32 varyings, 1 bit each) |
| `SQ_WRAPPING_0` | 0x2183 | Per-component wrap mode for varyings 0-15 |
| `SQ_WRAPPING_1` | 0x2184 | Per-component wrap mode for varyings 16-31 |

## KGSL "good path" values (LEIA REV470 = A22X / TouchPad)

From `kgsl_drawctxt.c:912-948`, the user-context shadow that gets restored on
context switch:

```c
/* SQ_PROGRAM_CNTL / SQ_CONTEXT_MISC (sys2gmem path, dynamic shader) */
SET_CONSTANT(SQ_PROGRAM_CNTL, 3 dwords):
    SQ_PROGRAM_CNTL = 0x10030002
    SQ_CONTEXT_MISC = 0x00000008

/* For dummy clear shader (sys2gmem_cmds, used during context restore): */
SQ_PROGRAM_CNTL = 0x10018001    /* LEIA REV470 specific */

/* Varying interpolation - all 32 smooth */
SQ_INTERPOLATOR_CNTL = 0xffffffff

/* Varying wrap mode - no wrapping */
SQ_WRAPPING_0 = 0x00000000
SQ_WRAPPING_1 = 0x00000000
```

## Mesa freedreno values (current)

From `fd2_program.c:265-267` and `fd2_emit.c:672-674`:

```c
/* SQ_INTERPOLATOR_CNTL - set AFTER SQ_PROGRAM_CNTL, also re-asserted in
 * fd2_draw.c:346 right before each draw (existing A22X workaround) */
SQ_INTERPOLATOR_CNTL = 0xffffffff

/* SQ_WRAPPING_0/1 - emitted in fd2_emit_restore */
SQ_WRAPPING_0 = 0x00000000
SQ_WRAPPING_1 = 0x00000000

/* SQ_PROGRAM_CNTL - computed dynamically per shader, matches the bit-field
 * layout that KGSL pre-encoded into 0x10030002 / 0x10018001 */
```

**SQ_INTERPOLATOR_CNTL, SQ_WRAPPING_0, SQ_WRAPPING_1 all match exactly between
KGSL good path and current Mesa.** Mesa even re-asserts `SQ_INTERPOLATOR_CNTL`
right before each draw as a defensive workaround for an earlier A22X
faceted-shading bug.

So the per-varying interpolation routing is *configured* identically. If your
VPC-buffer hypothesis is correct, the corruption is happening at the level of
the physical FIFO/SRAM that holds the parameter values *during* the draw, not
in the routing config registers we can program.

## What this implies for the dummy-draw fix

If the corruption is in the VPC physical FIFO storage (per-slot stale data),
then your stealth-dummy-draw approach inside `fd2_draw_vbo` is exactly right:
firing 8 wavefronts cycles the VPC's internal allocator through every physical
segment, overwriting whatever stale routing pointers / strides those segments
held from the previous client.

We're going to implement it. Confirming the plan:

1. Add `bool sq_scrubbed` to per-batch state (`fd_batch` or `fd2_context`)
2. In `fd2_draw_vbo` (a2xx-specific draw entry), check if first draw of batch
3. If yes:
   - Save current `RB_COLOR_MASK` + `RB_DEPTHCONTROL` values (or use the
     `dirty_state` infrastructure to recompute them after)
   - Emit `RB_COLOR_MASK = 0` + `RB_DEPTHCONTROL = 0`
   - Emit 8x `CP_DRAW_INDX` using `solid_prog` + `solid_vertexbuf` (point
     primitive, count=0 vertices is what KGSL uses; or count=3 if needed)
   - Restore via `fd_context_dirty(ctx, FD_DIRTY_BLEND | FD_DIRTY_ZSA)` so
     the user's real RB state gets re-emitted before their draw
   - Set `sq_scrubbed = true`
4. The user's actual draw fires next, lands on a freshly-cycled SQ slot

**Open question on the count**: KGSL's `build_sys2gmem_cmds` emits a single
`CP_DRAW_INDX` with `0x00030088` (which decodes as: count=3 vertices, source=
`PM4_DRAW_INDX_FROM_AUTO`, primitive=points). Should we emit one such draw 8
times to physically advance through 8 slots, or does a single draw with
count=24 vertices accomplish the same scrub in fewer packets? We're inclined
to keep it simple — 8 separate `DRAW_INDX` packets with count=3 each — to
match what KGSL does and to maximize the chance that each draw actually picks
up a fresh slot rather than the SQ packing them all into the same one.

## Side question

You mentioned "If you'd like to cross-reference the exact hardware state" —
the legacy webOS register dumps we have are post-mortem `cffdec` decoded
streams from running cmdstreams, not direct register reads. We have:

- `reports/webos-vfe-register-dump.txt` (camera VFE, not relevant here)
- `reports/leia-sq-registers-analysis.md` (analysis writeup, not raw dumps)
- KGSL cmdstream emission sequences (the `kgsl_drawctxt.c` source above)
- Decompiled HTC libqcamera blob (camera, not GPU)

Do you want us to capture *live* register dumps from the running webOS
kernel (booted on real hardware) for comparison? We can read CP/SQ state via
`/sys/kernel/debug/msm_kgsl/regs` if the legacy kernel exposes it, and we
have the device sitting on the desk. Just confirm the specific register
ranges you'd want sampled and at what point in the rendering flow (cold
boot? post-LSM-kill? mid-draw?).
