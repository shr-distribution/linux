# Update 17 for Gemini: VSC pipe init falsified, KGSL "register-shadow" PM4_LOAD_CONSTANT_CONTEXT theory

## TL;DR

1. **Mesa patch 0017 (zero all 8 VSC_PIPE registers + VSC_BIN_SIZE +
   LRZ_VSC_CONTROL in `fd2_emit_restore`) was deployed and tested.
   The 8-cycle is unchanged.** Same 8 unique hashes, same 12-13
   distribution, `5adc3160` still at exactly 1/8. VSC pipes are not
   the cycle's mechanism.

2. **Mainline already has an 8x SRAM scrub loop**
   (`a2xx_emit_sanitizer_preamble`) that wraps ALU/TEX/Bool/Loop
   constants in 8 iterations with `CP_INVALIDATE_STATE 0x7fff` +
   `CACHE_FLUSH_AND_INV_EVENT` between them. The hope was that
   `INVALIDATE_STATE` would advance the SQ slot pointer, but the
   8-cycle exists with this preamble running on every test
   submission, so either INVALIDATE_STATE doesn't rotate slots, or
   the cycle's mechanism isn't slot-rotation of constant SRAMs.

3. **KGSL does one substantive thing mainline doesn't**: it builds
   a per-context "register shadow" using `PM4_LOAD_CONSTANT_CONTEXT
   type=4` (the **register-shadow** form, not the
   ALU/TEX/Bool/Loop types). This bulk-loads >100 a2xx state
   registers from a per-context shadow BO. Mesa freedreno doesn't
   use this — it writes individual registers via `CP_SET_CONSTANT`
   and individual shader bytes via `CP_IM_LOAD_IMMEDIATE`.
   Hypothesis: type-4 might be the mechanism that broadcasts state
   into all 8 hardware contexts at once via the GPU's "constant
   context" hardware path. We need your read on this.

## What we tested

Mesa patch 0017 from `meta-mainline/recipes-graphics/mesa/files/`
was added to the Yocto bbappend's active SRC_URI list, rebuilt,
deployed. Confirmed in deployed binary:
```
strings /usr/lib/libgallium-26.1.0-devel.so | grep VSC
A22X: VSC registers initialized (VSC_BIN_SIZE=0, LRZ_VSC_CONTROL=0, all pipes=0)
```

Test sequence (with a fresh `sysrq b` reboot in between to flush
shader cache):
```
=== sanity render ===
5adc3160         <- first frame, correct hash

=== 100-cap with patch 0017 (VSC pipe init) ===
unique: 8
MMU faults: 0
hangchecks: 0
freq:
  13 fb12cd4c
  13 c1bf109e
  13 7b6dc2d0
  13 50baa6c2
  12 c399f1f4
  12 5d1220b0
  12 5adc3160
  12 10fbbed0
cycle (first 24):
ABCDEFGH
ABCDEFGH
ABCDEFGH
```

Identical to baseline. 12-13 each, perfect ABCDEFGH, `5adc3160` at
12/100 = 1/8. VSC theory falsified.

## Comparison with mainline's existing scrub

Mainline `a2xx_emit_sanitizer_preamble` (~lines 283-444 of
`a2xx_gpu.c`) runs on every cross-DRM-client submit. For each of 8
iterations it:
- emits `CP_INVALIDATE_STATE 0x7fff` (Gemini's slot-rotation
  candidate from update 4)
- WAIT_FOR_IDLE
- bulk-loads ALU constants (2048 dwords)
- bulk-loads TEX constants (192 dwords)
- zero-fills Bool constants (8 dwords)
- zero-fills Loop constants (56 dwords)
- emits `CACHE_FLUSH_AND_INV_EVENT`
- WAIT_FOR_IDLE

Each test-binary launch creates a new DRM context, so this
preamble fires every time. But the 8-cycle persists. Either:
- `CP_INVALIDATE_STATE 0x7fff` doesn't actually rotate the SQ slot
  pointer (it's an immediate-mode invalidate, may not consume an
  SQ slot allocation)
- The cycle's source isn't ALU/TEX/Bool/Loop constants
- The cycle's source isn't slot-rotated SRAM at all

## What KGSL has that mainline doesn't

`kgsl_drawctxt.c::build_regrestore_cmds()` builds a per-context
register-restore IB. The *first* packet in that IB is:

```c
/* deferred pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, ???); */
cmd++;          /* <-- placeholder for header, filled later */
*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;

/* ... lots of reg_range() calls that emit (offset, value) pairs ...
 * RB_SURFACE_INFO..RB_DEPTH_INFO,
 * COHER_DEST_BASE_0..PA_SC_SCREEN_SCISSOR_BR,
 * PA_SC_WINDOW_OFFSET..PA_SC_WINDOW_SCISSOR_BR,
 * LEIA_PC_MAX_VTX_INDX..LEIA_PC_INDX_OFFSET,
 * RB_COLOR_MASK..RB_FOG_COLOR,
 * RB_STENCILREFMASK_BF..PA_CL_VPORT_ZOFFSET,
 * SQ_PROGRAM_CNTL..SQ_WRAPPING_1,        <-- 100+ regs total
 * ... [more] ...
 */

start[2] = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT,
                            (cmd - start) - 3);
start[4] |= (1 << 24) | (4 << 16);  /* shadow=1, type=4 (register) */
```

Key bits in the type-4 form:
- **type=4** = "register-context" (not ALU=0, not TEX=1, not
  Bool=2, not Loop=3)
- **shadow=1 (bit 24)** = enable shadowing — the GPU keeps a
  memory copy that gets reloaded automatically on certain events

Mesa freedreno's `fd2_emit_restore` writes the same registers, but
with `CP_SET_CONSTANT` (one register at a time, no shadow, no
register-context type). So the registers reach the GPU but never
go through the type-4 / shadow=1 path.

## Hypotheses on what type-4 / shadow=1 actually does

We suspect the type-4 register-shadow form is more than a bulk
write. Specifically:

1. **Shadow=1 may broadcast to all 8 hardware contexts.**  Each of
   the 8 SQ contexts has its own state. With shadow enabled, when
   the CP rotates to a new context, the hardware promotes the
   shadow values into that context's registers automatically. With
   shadow disabled (Mesa's path), only the currently-active
   context gets the new value; the other 7 keep their stale state.
2. **Shadow=1 may be the only way to reach certain "context-local"
   regs.** Some a2xx registers might be context-local SRAM that
   `CP_SET_CONSTANT` only writes to the active context, but
   `PM4_LOAD_CONSTANT_CONTEXT type=4 shadow=1` writes to all
   contexts.
3. **Or shadow=1 just writes once but with different barrier
   semantics**, in which case it shouldn't matter whether we use
   type=4 or one-by-one SET_CONSTANT.

Given that mainline's existing `a2xx_emit_sanitizer_preamble` does
8 explicit iterations with `CP_INVALIDATE_STATE` between and the
cycle still persists, hypothesis #1 (broadcast to all contexts) is
the most economical explanation: only type-4/shadow-1 actually
broadcasts; INVALIDATE_STATE doesn't rotate; SET_CONSTANT doesn't
broadcast.

## Direct asks

### 1. Is type-4 PM4_LOAD_CONSTANT_CONTEXT really an "all 8 contexts" broadcast?

If you can confirm or refute this from the Adreno docs / KGSL
header comments / hardware behavior memory, that's the diagnostic
single point of failure.

If yes: mainline's missing functionality is exactly
PM4_LOAD_CONSTANT_CONTEXT type=4 with shadow=1. The fix is to emit
exactly the same packet KGSL emits, with the same shadow BO setup
(per-context register-shadow buffer in GPU-mapped memory). We have
infrastructure for shadow BOs already (`a2xx_alloc_shadow`).

If no: we're back to the drawing board on the cycle's mechanism.

### 2. The "Maximum Contexts = 1" ME_INIT field

Both mainline and KGSL emit `ME_INIT` with field 11
("Maximum Contexts") = `0x00000001`. If "1" means "one shader
context active" (shaders share state), the 8-slot cycle should
NOT exist — there'd be only 1 slot. So either:
- "Maximum Contexts" is unrelated to SQ wavefront slots (likely:
  refs to context-rollover logic for state-block management)
- Or the value isn't actually being honoured by the hardware
- Or the 8 cycle slots are something other than wavefront slots

Can you clarify what ME_INIT field 11 actually controls?

### 3. Concrete test: emit type-4 PM4_LOAD_CONSTANT_CONTEXT in mainline preamble

We're prepared to write the kernel patch:
- Allocate a shadow BO (we already do for ALU/TEX banks)
- Pre-fill it with safe register defaults (e.g., zero, or values
  observed in a known-good webOS regdump)
- Emit `PM4_LOAD_CONSTANT_CONTEXT type=4 shadow=1` from
  `a2xx_emit_sanitizer_preamble` referencing that shadow BO

If this collapses the cycle, we have the fix. Worth doing? Or do
you want us to validate the broadcast hypothesis first via some
other less invasive test?

### 4. Update on shippable path

Independent of the type-4 hypothesis, we still have the option of
implementing the SQ-slot scrub via 8 dummy `CP_DRAW_INDX` packets
with high-GPR shader (your update-15 spec). This is Mesa-side and
known shippable if it works. We haven't tried it yet. If type-4
broadcast turns out to be the missing piece, we wouldn't need the
draw-based scrub.

## Per-boot hash drift confirmation

For completeness — across recent reboots, the cycle's *7 wrong
hashes* have varied per boot, but `5adc3160` (the channel-mean-
correct render) is invariant:

* Boot N:    `e3de0ab8 9da287bb 6ef9de51 626387c3 9bbe68da `**`5adc3160`**` 1ab7f47e 19e31a86`
* Boot N+1:  `fb12cd4c c1bf109e `**`5adc3160`**` 50baa6c2 c399f1f4 7b6dc2d0 5d1220b0 10fbbed0`
* Boot N+2:  same set as N+1 (the most recent two reboots gave
  identical wrong sets)

So warm reboots don't always reshuffle the wrong hashes — sometimes
they do, sometimes they don't. This is consistent with whatever
holds the dirty SRAM being only "loosely" cleared by reset
sequences (some reboots happen to scrub more, others less).

## Tip / SHA reference

Branch `tenderloin/6.18/upstream-patches`, kernel tip
`76b56fe33fab19f256dfa6a7c4375244fcae052b` (RBBM mask param).
Mesa: 26.1.0+git-e57fca6de2 with patch 0017 active.
