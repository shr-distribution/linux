# Update for Gemini: SQ-slot scheduler theory falsified by 5x test

## Summary

The 8-slot SQ wavefront scheduler hypothesis is **conclusively falsified**.
Two clear-amplification tests (8x and 5x), with mathematically distinct
predicted outcomes, produced **bit-for-bit identical** output to the
unmodified baseline. `CP_DRAW_INDX` does not advance any 8-slot pointer
in the way described, or — if it does — the period-8 cycle has nothing
to do with that pointer.

## What we tested

Patch 0050 v4: amplify the existing `fd_draw()` inside Mesa's clear path
(`clear_fast` and slow `fd2_clear`) by an integer multiple, gated on
`is_a22x()`. Two runs on the TouchPad (1024x768, A22X / Adreno 220):

### Run 1: 8x amplification (7 extra + 1 original = 8 total clears)
Per your math, 12 tiles × (8 + 1 + 1) = 120 draws/batch. 120 mod 8 = 0
→ predicted no shift. Confirmed: identical to baseline.

### Run 2: 5x amplification (4 extra + 1 original = 5 total clears)
Per your math, 12 tiles × (5 + 1 + 1) = 84 draws/batch. 84 mod 8 = 4
→ predicted **shift by 4 in the cycle phase**. **Got: zero shift.**
Identical to baseline AND to v4 8x.

## The data — same to the byte across 3 runs

```
unique samples: 8
MMU faults:     0
hash freq:
   13 fb0772c9...    13 f197cb26...    13 5adc3160... (correct)
   13 37242f10...
   12 aae50ced...    12 91666260...    12 4b895c7a...
   12 3584c308...
cycle pattern:
   ABCDEFGH ABCDEFGH ABCDEFGH ...
```

Same eight hashes. Same (13,13,13,13,12,12,12,12) frequency split. Same
ABCDEFGH ordering. The Mesa MD5 of the deployed binary differs across
all three builds, so the patch is reaching the device — it just has zero
observable effect on the rendering nondeterminism.

## What this rules out

* **The "8 SQ wavefront slots, advanced per CP_DRAW_INDX" model.** If it
  were correct, 84 mod 8 = 4 would have shifted the cycle. The cycle
  did not shift. Either the slot pointer doesn't advance per
  `CP_DRAW_INDX`, or it does but the 7 "broken" hashes aren't generated
  by the slot pointer landing on a poisoned slot.
* **GMEM tile-replication theory was the right math but the wrong
  conclusion.** The 5x test was specifically designed to escape mod-8
  invisibility; it didn't. So it's not "we just need a coprime count."
* **VFETCH / VGT cache poisoning being scrubbable by clear-amplification.**
  If clears merely advance the pointer but don't scrub, we'd see *some*
  difference; we see none.

## What we still know to be true

* The cycle is **deterministic**: same 8 hashes in the same `ABCDEFGH`
  order across hundreds of runs, fresh DRM contexts.
* The 7 "broken" outputs share a fingerprint: missing red OR blue
  channel, never green; plus stale-looking GMEM tile artifacts.
* The PM4 cmdstream is **byte-identical** across all 100 captures
  (FD_RD_DUMP md5 verified). Mesa is doing its job.
* The 1 "correct" hash (`5adc3160...`) is what a cold-boot GPU produces.
  All 7 broken hashes are what some kind of post-LSM-kill state
  contamination produces.

## Suspect: maybe it's not the GPU at all

A thought we want your opinion on: **is the period-8 actually coming
from the test harness or display path, not the GPU's internal
scheduler?**

Things in the system that have a mod-8 character we have NOT yet
investigated:
1. **MDP4 LCDC refresh phase** (60 Hz, fixed 16.67 ms cadence). If the
   capture timing is fast enough that 100 captures span ~8 different
   phase relationships with vsync, that could produce 8 outcomes.
2. **MDP4 DMA fetch buffers** — does the MDP allocate a rotating
   set of N=8 fetch contexts?
3. **The KMS/DRM scanout buffer rotation** in luna-surfacemanager
   (LSM, the compositor) — does it cycle through 8 buffers?
4. **The kernel-side fence/seqno path** modulo 8 — does anything in
   `msm/drm/freedreno`'s fence retire path act on the lower 3 bits?
5. **The test harness itself**: `gl-cap-and-regdump-mainline` runs 100
   captures. Is it 100 separate processes (each fresh DRM context),
   or 1 process taking 100 frames? If the latter, the period-8 might
   be intra-process, not the cross-context contamination we assumed.
6. **APQ8060 ICC fabric arbitration window** — RPM-managed bandwidth
   slots may rotate.

## What we'd like from you

1. **Is the slot-scheduler model genuinely wrong, or did our test
   miss something?** Can you suggest a different stimulus that would
   prove or disprove the slot pointer's existence? E.g., is there a
   register on A2XX we can read that exposes the SQ slot allocator
   state? `RBBM_STATUS` bits 21:22 mention `SQ_CNTX0_BUSY` /
   `SQ_CNTX17_BUSY` — that suggests 18 contexts not 8, but worth
   probing.
2. **Given the period-8 fingerprint stays identical regardless of
   amplification, what other 8-element resources exist on the
   APQ8060 graphics path** (GPU + MDP + ICC fabric + IOMMU)? We
   want to widen the search space beyond the SQ.
3. **Should we bisect the test harness?** I.e., establish whether the
   8 unique outputs come from 8 different process contexts or 8
   different frames within a single context.

## Investigation work we'll start in parallel

* Examine the test harness to determine whether it's 100 processes or
  1 process × 100 frames.
* Read `RBBM_STATUS` and the `SQ_CNTX*_BUSY` bits live during runs.
* Inspect MDP4 / LCDC refresh-phase tracking — see if there's any
  cmpositor frame counter exposed at /sys.

We are not married to the slot-pointer story. The data says it's wrong.
What do you want us to look at next?
