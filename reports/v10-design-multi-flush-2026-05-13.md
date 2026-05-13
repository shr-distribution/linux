# v10 design — Mesa-side multi-flush per render

**Date:** 2026-05-13
**Status:** Design draft, not yet implemented.  Follows v9 v2's falsification of the per-CP_INDIRECT_BUFFER hypothesis.

## Goal

Make mainline Mesa freedreno issue exactly 16 `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS`-equivalent submits per logical render on A22X, matching webOS's strace-confirmed cadence.  Each ioctl advances the GPU's internal cycle counter by 1; 16 advances per render wrap the counter to phase 0 → no visible cycle.

## Counter-advance model (confirmed by v9 v2 sweep)

- GPU per-IB cycle counter, period **16**
- Advances **+1 per `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS`** (= per Mesa `fd_submit_flush()`)
- Adding `CP_INDIRECT_BUFFER` calls INSIDE one submit's ringbuffer does **NOT** advance the counter (v9 v2 NOP_COUNT=0..30 sweep, identical cycles)
- Counter resets to phase 0 on **kernel boot** (first-cap-after-reboot = `5adc3160` correct)

## webOS strace reference (2026-05-13)

```
16x ioctl(19, 0xc0140910, ...)   IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS
```

webOS's libGLESv2 user-space driver submits the per-tile loop and resolve as **separate ioctls per tile** (~6 tile submits + binning + state setup + cleanup ≈ 16).  Mesa freedreno does the entire tile loop in ONE `fd_batch_flush()` → ONE ioctl.

## Why cap-binary `FD2_N_RENDERS=16` falsifier failed

Each render in the cap binary creates a fresh Mesa batch.  Each batch's `fd2_emit_tile_init` re-engages the binner via `LRZ_VSC_CONTROL=3` + `0xC00=1`.  Without explicit per-batch binner disengage (gated behind `FD2_BIN_FULL_DISENGAGE` in 0098 v7), the binner carries stale state forward and hangs partway through the 16-render sequence.

So the falsifier path is blocked by the binner-state-corruption issue from patch 0093.  Going around it requires either:
1. Force `FD2_BIN_FULL_DISENGAGE=1` for every test, or
2. Fix patch 0093 to always disengage cleanly between batches, or
3. Move the multi-flush INSIDE one Mesa batch (avoiding inter-batch binner state)

Option 3 is what v10 targets.

## v10 design — single-batch multi-flush via `fd_submit_flush` injection

**Approach:** Inside one `fd_batch_flush()` call, issue (N-1) dummy `fd_submit_flush()` calls before the real submit.  Each dummy = a tiny `fd_submit` containing a 2-dword `CP_NOP` ringbuffer.  The 16th submit is Mesa's real batch.  All 16 happen back-to-back from the kernel's perspective, but the GPU sees 16 ioctls = 16 counter advances = wrap to phase 0.

### Pseudocode

```c
/* In freedreno_batch.c batch_flush(), or new fd2-specific hook called
 * just before fd_gmem_render_tiles() */

static void
fd2_a22x_multi_flush_align(struct fd_batch *batch)
{
   if (!is_a22x(batch->ctx->screen))
      return;

   int multiplier = 16;  /* default match webOS */
   const char *e = getenv("FD2_FLUSH_MULTIPLIER");
   if (e)
      multiplier = atoi(e);
   if (multiplier <= 1)
      return;

   for (int i = 0; i < multiplier - 1; i++) {
      /* Build a minimal dummy fd_submit:
       *   - new fd_submit on the same pipe
       *   - new fd_ringbuffer with 2 dwords (CP_NOP + 1 payload zero)
       *   - fd_submit_flush(submit, -1, false)
       *   - free the submit
       *
       * Each fd_submit_flush() goes through MSM_SUBMIT ioctl which the
       * kernel translates to one ISSUEIBCMDS equivalent on the A2XX
       * ringbuffer.  GPU sees 1 IB advance per call.
       */
      struct fd_submit *dummy = fd_submit_new(batch->ctx->pipe);
      struct fd_ringbuffer *rb = fd_submit_new_ringbuffer(
          dummy, 8 /* sizedwords */, FD_RINGBUFFER_PRIMARY);
      OUT_PKT3(rb, CP_NOP, 0);
      OUT_RING(rb, 0x00000000);

      struct fd_fence *f = fd_submit_flush(dummy, -1, false);
      if (f)
         fd_pipe_fence_del(f);
      fd_submit_del(dummy);
   }
}
```

### Hook point

Best place: `freedreno_batch.c::batch_flush()` immediately BEFORE `fd_gmem_render_tiles(batch);` at line 379.  That way:
1. Mesa builds the real batch's ringbuffer fully (state setup + binning IB + per-tile commands)
2. We dispatch 15 dummy MSM_SUBMITs (each = 1 ISSUEIBCMDS-equivalent kernel-side)
3. Then `fd_gmem_render_tiles()` does the real submit (16th ioctl)
4. GPU counter advances 16 times → wraps to same phase as previous render → no visible cycle

### Concerns / risks

1. **Dummy fd_submit / fd_ringbuffer correctness.**  Need to ensure each dummy submit is a complete, self-contained submission that the kernel won't reject.  Minimum content: 1 valid PM4 packet (CP_NOP) + ringbuffer terminator.

2. **Fence ordering.**  The 15 dummies create fences we have to consume (or leak BOs).  Best practice: wait on each fence inline (`fd_pipe_fence_finish()`) before the next dummy?  Or just `_del` without waiting since the GPU work is trivial.

3. **Performance.**  15 extra ioctls + kernel processing per render.  Each ioctl is ~10-100 µs on this hardware.  Worst case: 1.5 ms extra per render = ~3 fps overhead at 60 fps.  Acceptable for a workaround.

4. **Interaction with patch 0093's prelude.**  The 15 dummies happen BEFORE the real batch starts.  At dummy-time, no binner engagement, no fork A/B prelude — just a bare ringbuffer with CP_NOP.  Should be safe.  The real batch then runs as usual.

5. **Counter ordering.**  We need the 15 dummies to advance the counter, then the real batch's render to land at the right phase.  If counter is "post-submit value", the real batch's render uses post-15th-dummy phase = base + 15.  Plus the real batch itself advances → +1.  Plus binning IB inside real batch is a CP_INDIRECT_BUFFER (no advance, confirmed by v9 v2).  So per render counter ends at base + 16.  Next render starts at base + 16 = same phase (mod 16) as previous = same render output.  ✓

6. **What if the counter is "pre-submit"?**  Then the render uses the BEFORE-dummies counter value = base.  The 15 dummies advance after, the real batch advances after.  Next render starts at base + 16 = same phase.  Same outcome.  ✓

Either ordering of counter increment relative to submit work produces the desired cycle alignment.

## Test plan once v10 is implemented

1. Default behaviour (no env): all renders should produce `5adc3160` ✓  (16/16 caps in default sweep)
2. `FD2_FLUSH_MULTIPLIER=1` → reverts to current period-16 cycle (control)
3. `FD2_FLUSH_MULTIPLIER=8` → period-2 cycle (every other render correct)
4. `FD2_FLUSH_MULTIPLIER=16` → cycle collapsed to single hash
5. `FD2_FLUSH_MULTIPLIER=32` → still single hash (2 full wraps)

If the bisect produces the expected periodicity, v10 is the structural fix.  Then we either ship `FD2_FLUSH_MULTIPLIER=16` as the A22X default or remove the env-var gate entirely.

## Alternative: per-tile separate submits

The cleaner architectural match to webOS is to split the per-tile loop in `fd_gmem_render_tiles()` into N separate `fd_batch_flush()` calls (one per tile).  This is much more invasive — affects core freedreno tile rendering for all GPUs.  Reserve for if v10 multi-flush works but has performance / correctness issues.

## Files / commits

- Mesa source (untouched yet): `/home/herrie/Documents/GitHub/mesa-latest/src/gallium/drivers/freedreno/freedreno_batch.c::batch_flush()` around line 379
- Yocto layer: `meta-mainline/recipes-graphics/mesa/files/0099-...patch` is **dropped** from SRC_URI (committed `b3b1e19b`)
- This document: kernel repo `reports/v10-design-multi-flush-2026-05-13.md`
