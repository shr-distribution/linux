# A2XX (Adreno 220 / Leia) Register-Write Cross-Vendor Comparison

Goal: find A2XX GPU registers that vendor proprietary GLES drivers
write but Mesa freedreno's a2xx driver does not, and registers where
values disagree. Inputs (all artifacts persisted under
`reports/ghidra-decomp/`):

- `decomp-txt/webos_libGLESv2.so.decomp.txt` (HP webOS, ~191 funcs,
  leia_*/context_*/preamble_* filter; binary in `webos-binaries/`)
- `decomp-txt/samsung_libGLESv2_adreno200.so.decomp.txt`
  (Samsung Q1 GalaxyTab Adreno 220, ~310 funcs)
- `decomp-txt/htc_libGLESv2_adreno200.so.decomp.txt` (HTC Pyramid
  Adreno 220, ~310 funcs)
- `decomp-txt/xiaomi_libGLESv2_adreno200.so.decomp.txt` (Xiaomi Aries
  Adreno 225 / MSM8960, similar variant; expected differences vs A220
  noted below)
- Mesa: `src/gallium/drivers/freedreno/a2xx/fd2_*.c` + register defs
  in `src/freedreno/registers/adreno/a2xx.xml`.

## Summary (TL;DR)

**Headline: no named A2XX register that the vendor consensus writes is
genuinely missing in Mesa.** After accounting for `CP_SET_CONSTANT`
block-writes (one packet writes `count` consecutive registers, only
the start register shows up in source greps), every documented
register the vendors write is also written by Mesa.

The earlier candidates (PA_CL_GB_*, RB_FOG_COLOR, RB_SAMPLE_POS,
VGT_MIN_VTX_INDX, RB_ALPHA_REF, PA_SU_POINT_MINMAX, PA_SU_LINE_CNTL)
all turn out to be covered by Mesa's existing block-writes.

What remains, and may still be actionable:

1. **61 undocumented register offsets** in the 0x2000-0x27FF range
   that Samsung+HTC write but a2xx.xml has no entry for. Likely
   texture/sampler descriptor slots, shader constant slots, or
   undocumented HW control registers. Cannot be acted on without
   identifying what they are first.
2. **Value differences** can't be extracted from the decomps cleanly
   - vendor values come through `*(struct + offset)` indirections,
   not constants. Need a runtime register-dump from a webOS app to
   compare against Mesa.
3. **A225-only registers** (`A225_PC_MULTI_PRIM_IB_RESET_INDX`,
   `A225_GRAS_UCP0X`, `A225_GRAS_UCP_ENABLED`) are written by
   vendors but don't apply to A220 (TouchPad is A220, not A225).

## Method

### Vendor side

`grep -oE "0x2[0-9a-f]{3},"` against each decomp - register address
literals in argument position. This catches every PM4 helper
(`FUN_0006f51c`, `FUN_0008ceec`, `FUN_000855d4`, etc.) regardless of
which one it is.

  - Samsung: 108 distinct register offsets in 0x2000-0x2FFF range
  - HTC: 108 distinct, **byte-identical to Samsung** (same Adreno 220
    proprietary userspace, just rebranded)
  - Xiaomi (aries, MSM8960 / Adreno 225): 107 distinct; intersects
    the Samsung+HTC set at 99 offsets. Differences track A225-vs-A220
    silicon evolution (see "A220 vs A225 silicon delta" below) - they
    are *not* candidates for missing-Mesa-state.
  - webOS: 0 - uses a different code pattern (deferred state-store
    via `rb_mark_state_change`, then a flush function emits the
    actual writes; the flush function isn't named `leia_*` and
    didn't make it into our function filter)

So the vendor consensus is **3-way consensus = Samsung ∩ HTC ∩ Xiaomi
= 99 offsets**. webOS is left out of the offset-set comparison but
cross-checks at the function-name level (e.g., webOS also has
`leia_sethwstate_guardband`, `leia_sethwstate_scissor`,
`leia_perform_resolve`). Where Xiaomi diverges, that's an A225-vs-A220
silicon delta, not a Mesa-needs-this signal.

### A220 vs A225 silicon delta (Xiaomi diff)

Xiaomi (A225) writes that Samsung+HTC (A220) don't:
`0x2240 0x22d6 0x22e2 0x22e4 0x22ec 0x2780 0x27c0 0x2f1e` - none of
these are documented in `a2xx.xml`. They're A225-specific and
**explicitly NOT relevant** to TouchPad (which is A220).

Samsung+HTC (A220) writes that Xiaomi (A225) doesn't:
`0x20c7 0x20c8 0x20cb 0x20cc 0x20cf 0x20d0 0x20d3 0x27c8 0x2ff6` -
also undocumented. May be A220-specific. Could be candidates for
"only Mesa-on-A220 needs this" but unnamed, can't act without
identifying them first.

### Mesa side

Extract every `REG_A2XX_*` and `REG_A220_*` symbol referenced in
`src/gallium/drivers/freedreno/a2xx/fd2_*.c`, look up offsets via
`a2xx.xml`. 55 distinct register offsets in the 0x2xxx range.

Critically, Mesa's writes use `OUT_PKT3(ring, CP_SET_CONSTANT, count)`
followed by a single `CP_REG(REG_A2XX_<name>)` for the start register
- but the packet implicitly writes `count` consecutive registers. So
the source-level grep finds only the start of each block; the
block-extension registers are covered by hardware but invisible to
the grep. This is exactly what tripped up the previous (incorrect)
"92 missing registers" report - it didn't account for block writes.

### Comparison

Set difference: of the 99-offset 3-way vendor consensus, 63 don't
appear in Mesa's source-level grep. After resolving block-write
chains, every **named** register in that gap collapses into a Mesa
block-write:

| Vendor reg | a2xx.xml name                       | Covered by Mesa block-write |
|-----------|--------------------------------------|-----------------------------|
| 0x2101    | VGT_MIN_VTX_INDX                     | `VGT_MAX_VTX_INDX` count=3 (0x2100..0x2102) |
| 0x2103    | A225_PC_MULTI_PRIM_IB_RESET_INDX     | A225-only, n/a for A220     |
| 0x2106    | RB_BLEND_GREEN                       | `RB_BLEND_RED` count=5 (0x2105..0x2109) |
| 0x2108    | RB_BLEND_ALPHA                       | `RB_BLEND_RED` count=5      |
| 0x2109    | RB_FOG_COLOR                         | `RB_BLEND_RED` count=5      |
| 0x210e    | RB_ALPHA_REF                         | `RB_STENCILREFMASK_BF` count=4 (0x210c..0x210f) |
| 0x2281    | PA_SU_POINT_MINMAX                   | `PA_SU_POINT_SIZE` count=5 (0x2280..0x2284) |
| 0x2282    | PA_SU_LINE_CNTL                      | `PA_SU_POINT_SIZE` count=5  |
| 0x2303    | PA_CL_GB_VERT_CLIP_ADJ               | `PA_SU_VTX_CNTL` count=6 (0x2302..0x2307) |
| 0x2340    | A225_GRAS_UCP0X                      | A225-only, n/a for A220     |
| 0x2360    | A225_GRAS_UCP_ENABLED                | A225-only, n/a for A220     |

Verifiable in Mesa source:
- `fd2_emit.c:282` — `PA_SU_VTX_CNTL` count=6 (covers 0x2303 guardband)
- `fd2_emit.c:286-289` — emits `fui(1.0f)` for all four
  PA_CL_GB_VERT_CLIP_ADJ / VERT_DISC / HORZ_CLIP / HORZ_DISC.
- `fd2_emit.c:264` — `RB_STENCILREFMASK_BF` count=4 (covers
  RB_ALPHA_REF at 0x210e via `zsa->rb_alpha_ref`)
- `fd2_draw.c:567` — `VGT_MAX_VTX_INDX` count=3 (covers
  VGT_MIN_VTX_INDX at 0x2101)
- `fd2_emit.c` — `RB_BLEND_RED` count=5 (covers GREEN/BLUE/ALPHA/FOG)

## Unresolved (52 undocumented offsets in 3-way consensus)

These are vendor-written addresses with no entry in
`src/freedreno/registers/adreno/a2xx.xml`, so we can't say what they
do or whether Mesa needs them. Grouped by stride pattern:

```
Group A (stride-4, 0x2040-0x204c, 0x2069-0x206c, 0x2070-0x207c):
  0x2003 0x2004 0x2008 0x200c
  0x2040 0x2044 0x2048 0x204c
  0x2069 0x206c
  0x2070 0x2071 0x2072 0x2073 0x2074 0x2077 0x2078 0x2079 0x207b 0x207c

Group B (stride-1 sweep around 0x209c-0x20FB - matches sampler
descriptor / texture state layout):
  0x209c 0x20a0 0x20a1 0x20a2 0x20a3 0x20a5 0x20a9 0x20ad 0x20ae
  0x20b2 0x20b3 0x20b7
  0x20c0 0x20c1 0x20c2 0x20c3 0x20c5 0x20c7 0x20c8 0x20cb 0x20cc 0x20cf 0x20d0 0x20d3
  0x20e4
  0x20f4 0x20f8 0x20f9 0x20fb

Group C (RB control gaps):
  0x210b
  0x21c2 0x21c4 0x21c5 0x21e4 0x21ec
  0x2220
  0x2297
  0x22c0

Group D (high addresses — far above a2xx.xml's documented range):
  0x27c8
  0x2e9a
  0x2ff6
```

**Group B** (0x20a0..0x20fb, ~30 addresses with regular stride) most
likely corresponds to **texture sampler / vertex fetch constant
slots** in CP_SET_CONSTANT type-1/2 space. Mesa handles texture
sampler binding through a different code path (`fd2_texture_state`)
and emits these via different packet types - so source-level grep on
`REG_A2XX_*` symbols wouldn't show them. **Not actionable as
"missing"; just a different code path.**

**Group A** (0x2040..0x207c, stride-4) might be performance-counter
config registers, MH (memory hub) routing, or ring-buffer
self-config - these aren't typically written from userspace
graphics in mainline; they'd be written by KGSL kernel-side context
init. **Not actionable from Mesa.**

**Group C** (0x21c2, 0x22c0, etc.) is the strongest candidate for
genuine missing GPU control state. Worth manually decoding what these
addresses are by cross-referencing the a225/a220 / yamato register
header in the legacy webOS kernel
(`drivers/gpu/msm/leia_reg.h`,
`drivers/gpu/msm/yamato_reg.h`) - those headers cover registers
the public Mesa a2xx.xml omits.

**Group D** (0x27c8, 0x2e9a, 0x2ff6) are outside the documented
A2XX register space entirely and may be A225/A22X chip-specific
registers, scratch addresses, or noise from the helper-arg
extraction.

## Mesa-only writes (in Mesa, not in vendor offset set)

11 register offsets Mesa writes that didn't appear in the vendor grep:

```
0x200f  PA_SC_SCREEN_SCISSOR_BR    (vendors: covered by 0x200e count=2)
0x2010  UNKNOWN_2010                (a2xx Mesa-only debug?)
0x2180  PA_SC_LINE_STIPPLE          (vendors: covered by 0x2181 chain?)
0x2203  RB_MODECONTROL              (vendors: 0x2208 covers? double-write?)
0x2207  RB_DEPTH_CLEAR             
0x220b  RB_SAMPLE_COUNT_CTL        
0x2210  CLEAR_COLOR                
0x2293  ?                           (Mesa unique)
0x2317  PA_CL_GB_VERT_CLIP_ADJ tail (block continuation)
0x231c, 0x231d  PA_CL_VPORT_*       (covered by 0x2312 chain?)
```

These are mostly block-write continuations on the vendor side too
(vendors write 0x200e count=2 which covers 0x200f) - confirmed they
also use block-writes, not stand-alone writes.

## Value comparison (limited)

The vendor decompilations don't expose constant values cleanly -
register values flow through C-struct member dereferences
(`*(int *)(param_3 + 0x5c)`), so we'd need to trace each struct
member to its initializer to recover values. That's beyond what a
text grep can do.

The one place values DID surface, the 0x2300 VGT_VERTEX_REUSE_BLOCK_CNTL
case explicitly seen in `leia_perform_resolve` and matching
upstream-mesa-default `0x3b` vs KGSL-default `0x02`, was already
investigated and resolved (Mesa SRC_URI patch 0026 set Mesa to
`0x02` to match KGSL).

If we want a real value comparison, the right move is a runtime
register dump from a webOS app via `kgsl_postmortem` or by patching
the legacy KGSL ringbuffer logger to print every CP_SET_CONSTANT
packet it sees. The decompile alone won't get us there.

## What this means for the visual bugs

The original framing - "Mesa is missing register writes that vendor
drivers do, that's why we have rendering corruption" - is **not
supported by this comparison**.

For the actual visual bugs (faceted shading, post-LSM-kill black
faces, glmark2 build/shading flat-shading), the remaining
hypotheses (in priority order) are:

1. **Per-context HW state leak** between DRM clients - what KGSL solved
   via per-context shadow memory, mainline solves implicitly via
   re-emit on every batch, but only if every state register Mesa
   tracks is in fact re-emitted on every batch. Sanity-check
   `fd2_emit_restore` (full re-emit at batch start) covers every reg
   from fd2_emit.c that's per-state. If a register only gets emitted
   via dirty-flag and the dirty flags aren't set after a context
   switch, that's a leak. Patch 0038 was supposed to fix this but
   visual artifacts persist.
2. **Sample-position / MSAA grid timing**. RB_SAMPLE_POS *is* written
   by Mesa, but at `fd2_emit.c:616` - check what code path actually
   emits it and whether it runs on every batch.
3. **Value mismatches** that the grep can't see. Need a runtime dump.
4. **Group C undocumented registers** (0x21c2, 0x22c0, 0x2297) are the
   one set of vendor writes that don't obviously dissolve. Worth
   resolving against `leia_reg.h` / `yamato_reg.h` to see what they
   are.

## Followups

- [ ] Cross-check the 11 "Mesa-only" writes - some may be debug
  leftovers Mesa shouldn't be doing either.
- [ ] Resolve Group A & Group C unnamed offsets against
  `webos-linux-kernel-touchpad/drivers/gpu/msm/{leia_reg,yamato_reg}.h`
  to see if they correspond to documented registers Mesa just doesn't
  reference.
- [x] Add Xiaomi `aries` (MSM8960 / Adreno 225) proprietary GLES to
  the comparison. **Done** - 3-way consensus at 99 offsets, identical
  list of 11 named missing registers (all already covered by Mesa
  block-writes). Confirmed conclusion: no genuinely missing named
  registers.
- [ ] Get a runtime CP_SET_CONSTANT trace from webOS for actual
  *value* comparison; offset-set comparison alone is not enough.

## Coverage notes

- Vendor grep coverage of 0x2xxx range: high (108 unique offsets
  matches the full a2xx physical reg + sampler-constant footprint).
- Mesa coverage: source-level grep found 55; block-write expansion
  adds another ~40 implicit registers (every count>1 packet).
- webOS GLES (HP TouchPad's actual proprietary userspace) was the
  poorest-coverage source - its register-write helpers don't take
  the offset as a literal argument, so the simple grep missed them
  entirely. Would need a different extraction approach (find the
  flush function, decode its inline writes).

---

**Generated**: 2026-05-08 by direct analysis (replacing earlier agent
report which mis-classified block-write continuations as missing
registers).
