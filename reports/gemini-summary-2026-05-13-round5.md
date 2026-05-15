# Gemini consultation: A22X period-16 — round 5, vendor binary architecture

**Continuation of:** `gemini-summary-2026-05-13-round4.md`
**Date:** 2026-05-13

## TL;DR

Deep-dive into the decompiled HTC/Samsung/Xiaomi/webOS libGLESv2 binaries revealed concrete architectural findings, including **one candidate fix that we have NOT yet tested in the right way**: vendors emit `OUT_PKT0(0x0e00, 1) + value 0x1` (TC_CNTL_STATUS = L2_INVALIDATE) **inline within the cmdstream**, while our earlier "TC_INV" test wrote the register out-of-band via debugfs (CPU MMIO write while GPU often clock-gated).

## Round-3-suggested per-draw DEALLOC is in flight

We implemented `FD2_PER_DRAW_DEALLOC=1` (Mesa patch 0016) — emit VS_DEALLOC + PS_DEALLOC after every `CP_DRAW_INDX` in the shared `fd_draw()` helper.  Yocto rebuild pending.  Will test on device next.

## Vendor binary deep dive

We dumped all `leia_*` and `yamato_*` cmdbuffer helper functions from each vendor's libGLESv2.so via Ghidra.  Per-vendor architecture is now visible.

### Key helper functions (consistent across HTC / Samsung / Xiaomi / webOS):

```
leia_cmdbuffer_insertnop                  (14 bytes, emits CP_NOP)
leia_cmdbuffer_insertwaitforidle          (110 bytes)
leia_cmdbuffer_inserttexcacheinvalid      (14-28 bytes)
leia_cmdbuffer_insertindirectbuffer       (12 bytes)
leia_cmdbuffer_insertgpumemwrite          (36 bytes)
leia_cmdbuffer_insertinterrupt            (14 bytes)
leia_cmdbuffer_indirectpreamble           (42 bytes)
leia_cmdbuffer_indirectpostamble          (42 bytes)
leia_cmdbuffer_gpu_spam                   (354 bytes) <- LARGE state setup
```

### `leia_cmdbuffer_inserttexcacheinvalid` decompiled (identical xiaomi/webos):

```c
undefined4 * leia_cmdbuffer_inserttexcacheinvalid(undefined4 *param_1)
{
  *param_1 = 0xe00;       /* PM4 type-0 header: write 1 dword to reg 0x0e00 */
  param_1[1] = 1;         /* value = 1 (L2_INVALIDATE bit 0) */
  return param_1 + 2;
}
```

That's **exactly** `OUT_PKT0(REG_A2XX_TC_CNTL_STATUS, 1); OUT_RING(0x1);` — i.e., **inline cmdstream write** of TC_CNTL_STATUS = L2_INVALIDATE.

### `leia_cmdbuffer_indirectpreamble` and `_indirectpostamble` (xiaomi):

```c
void leia_cmdbuffer_indirectpreamble(int param_1) {
    if (offset_0x5f8 != offset_0x5f0) {
        leia_repartition_instruction_store(param_1, 0);
    }
    leia_cmdbuffer_gpu_spam(param_1, 2, 0);
    return;
}

void leia_cmdbuffer_indirectpostamble(int param_1) {
    leia_cmdbuffer_gpu_spam(param_1, 2, 1);
    /* update preamble buffer state */
    if (preamble_buf != NULL) {
        leia_preamble_indirect_append(preamble_buf);
    }
}
```

Vendors wrap each indirect buffer call with pre/post-amble that emits "gpu_spam(type=2, arg=0|1)".  `gpu_spam` is 354 bytes — a substantial state setup block.  Mesa freedreno does NOT have anything like this around its `CP_INDIRECT_BUFFER` calls.

## What this means

1. **Vendor cmdstream is highly structured around indirect buffers**, with pre/post-amble wrappers.  Mesa just emits `CP_INDIRECT_BUFFER` raw.
2. **Vendors emit TC_CNTL_STATUS = L2_INVALIDATE INLINE in cmdstream** (executed by CP during render).  Mesa never does this.  Our earlier test wrote this register from CPU via debugfs out-of-band — fundamentally different code path.
3. **The `leia_cmdbuffer_gpu_spam(ctx, 2, 0|1)` function is large (354 bytes)** — probably emits 30-50 PM4 dwords of state initialization.  This may be the "16-slot scrub" that ensures all VPC/TP slots are initialized at preamble time.

## Candidate fixes (priority-ordered)

### Path 1: Inline TC_CNTL_STATUS = L2_INVALIDATE emission

Add to `fd2_emit_tile_fini` (or per-draw):

```c
OUT_PKT0(ring, REG_A2XX_TC_CNTL_STATUS, 1);
OUT_RING(ring, 0x1);  /* L2_INVALIDATE */
```

Cheap to test.  We already tested via debugfs without effect, but inline-in-cmdstream is a different code path and may have different timing semantics on the GPU.

### Path 2: Add a Mesa-side equivalent of `leia_cmdbuffer_indirectpreamble/postamble`

Wrap each `CP_INDIRECT_BUFFER` call (Mesa uses several — for binning IB, tile IB, etc.) with the same state-restoration block vendors emit.  Much more invasive — requires understanding `gpu_spam(type=2)` content.  Defer until Path 1 falsified.

### Path 3: Per-draw DEALLOC (already in flight)

Patch 0016 — testing whether emitting deallocation events AFTER each `CP_DRAW_INDX` (vs at end of tile loop) is the right position.

## Concrete questions for Gemini round 5

### Q1: Is inline TC_CNTL_STATUS = L2_INVALIDATE different from debugfs write?

We wrote `TC_CNTL_STATUS bit 0 = L2_INVALIDATE` via debugfs (CPU-side direct MMIO write while GPU might be clock-gated).  No effect on 0x0ee2 cycle.

If we emit the same write **inline in the cmdstream** (CP executes it during active render), would you expect a different outcome?  On Adreno, are there register writes whose semantics differ when issued by CP vs CPU?

### Q2: What does `leia_cmdbuffer_gpu_spam(ctx, 2, 0|1)` likely emit?

354 bytes / ~88 instructions for a "preamble" — this is huge.  Hypothesis: it emits a block of `CP_SET_CONSTANT` writes to restore all the register state that the GPU's internal slot allocator needs to be "consistent across slots".  Effectively a 16-slot warmup.

Does this match any A2XX architecture docs you know?  Specifically: **is there a known A2XX register block whose state, when consistent across all 16 internal slots, prevents the cycle**?

### Q3: HTC/Samsung/Xiaomi only have 5 `CP_SET_CONSTANT` (0x2d) literals while webOS has 195

That's a 40x difference.  HTC/Samsung/Xiaomi must construct PM4 headers dynamically (e.g., `header = (CP_TYPE3_PKT | (cnt << 16) | (opcode << 8))`).  Their entire cmdstream is built through helper functions; the static opcode literals appear only in the helpers, not at call sites.

webOS, being older code, has explicit `*ptr = 0xc002d_NN` assignments scattered throughout.

This means our earlier "vendor binary opcode survey" only captured the *helper functions' opcodes*, not the actual *number of calls* to those helpers.  We cannot easily count "how many draws does HTC issue per render" from the decomp.

Does this match your expectation of how qcom's binary blob was constructed across generations?

### Q4: Mesa's `CP_INDIRECT_BUFFER` topology

Mesa freedreno builds: binning IB + main ringbuffer + tile_store IB.  The main ringbuffer contains `CP_INDIRECT_BUFFER` calls to both.  Three IB1 → IB2 transitions per render.

Vendors wrap each IB call with `indirectpreamble` + `indirectpostamble` (their helpers, 42 bytes each = ~10 PM4 dwords).  Mesa does NOTHING around its `CP_INDIRECT_BUFFER` calls.

**Is the cycle counter advanced by the `CP_INDIRECT_BUFFER` transition itself, and is the pre/post-amble what cleans up after each transition?**  Round-3 v9 NOP-IB sweep showed adding NOPs inside an IB doesn't advance the counter, but we didn't test ADDING IB transitions around state setup.

## Repo references

- Mesa patch 0014/0015/0016: end-of-tile-loop / 3-way bisect / per-draw DEALLOC
- Vendor binary decomps: `reports/ghidra-decomp/decomp-txt/{webos,htc,samsung,xiaomi}_libGLESv2*.txt`
- This summary's data lives in same dir as round-3/round-4

## Most decisive next test

**Implement Path 1** (inline TC_CNTL_STATUS = L2_INVALIDATE).  3-line code change, env-gated for safety.  If it pins the cycle → we have the fix.  If not → we know the CPU-vs-CP register write distinction doesn't matter for this register.
