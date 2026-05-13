# v9 v2 FD2_NOP_IB_COUNT sweep — per-CP_INDIRECT_BUFFER hypothesis 100% FALSIFIED

**Date:** 2026-05-13
**Mesa libgallium md5:** `4f2b3b31107df238505a21e571c26644` (0099 v2 on top of 0098 v8)
**Test:** 16 caps each at FD2_NOP_IB_COUNT=0, 1, 2, 7, 13, 14, 15, 30.  Each NOP adds one `CP_INDIRECT_BUFFER_PFD` call to a 2-dword `CP_NOP` IB inside the same submit ringbuffer.

## The result

Every single NOP_COUNT produces the **same 16 hashes in the same order**, starting at the same phase:

```
phase  0:  03dee03a              phase  8:  1794228b
phase  1:  cb22c18c              phase  9:  c60a6009
phase  2:  e93d10aa              phase 10:  ebb5d66d
phase  3:  03aa0743              phase 11:  695afd48
phase  4:  d148b696              phase 12:  5adc3160 ✓ CORRECT
phase  5:  3c9c950b              phase 13:  ba4f2af7
phase  6:  9985d0bf              phase 14:  bebc09c6
phase  7:  6c10867f              phase 15:  2f1fa0a6
```

8 NOP_COUNT values × 16 caps = 128 captures.  All 8 runs trace the same path through the same 16-hash cycle.  Within each run, every successive cap advances exactly +1 in the cycle.

## Conclusion: GPU counter is per-ISSUEIBCMDS, not per-CP_INDIRECT_BUFFER

The 0099 v9 hypothesis was: "the GPU has a per-IB cycle counter advancing once per `CP_INDIRECT_BUFFER` call".  This sweep falsifies it utterly — adding 0, 14, or 30 NOP CP_INDIRECT_BUFFER calls inside the Mesa submit's ringbuffer produces **identical** cycle behaviour.

The cycle counter therefore advances **once per `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS` ioctl** (= per Mesa submit = per `fd_batch_flush`), independent of how many `CP_INDIRECT_BUFFER` calls the kernel command-processor sees inside one submission.

This matches the webOS strace exactly:
- webOS issues 16 ISSUEIBCMDS per render → 16 counter advances → wraps mod-16 → cycle completes within one render → no visible cycle.
- Mainline Mesa freedreno's Fork A/B path issues 1 ISSUEIBCMDS per render → 1 counter advance → cycle walks 1 phase per render → visible period-16 cycle of complete frames over 16 consecutive caps.

## Side observation: NOP IBs are FUNCTIONALLY INERT

Beyond not advancing the counter, the NOP IBs also don't perturb the cycle's hash pool (16 hashes are stable across all NOP_COUNTs).  This contradicts the earlier 5-cap-per-NOP sweep result which suggested NOPs introduced 9 "new" hashes.  Re-interpreted with the 16-cap data: the earlier impression of "new hashes" was an artefact of running too few caps per NOP_COUNT — the 5-cap windows weren't long enough to cover the full 16-phase cycle, so each NOP_COUNT's 5 caps just sampled a different 5-phase contiguous slice of the same cycle.

## What's next: 0099 should be reverted, v10 needs per-submit fix

The 0099 v9 v2 patch as-shipped is a safe no-op (no hangs, doesn't change behaviour).  It can stay in the build without harm, but it does nothing for the cycle.  Recommend reverting it from SRC_URI for cleanliness.

The actual fix requires Mesa to issue 15 extra `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS` per render to match webOS's 16-submit cadence.  Three options ranked by effort:

| approach | effort | risk | shippable |
|----------|--------|------|-----------|
| Cap-binary modification: render N times (extending the existing FD2_VSC_DUMP n_renders=2 logic) | 10 min cross-compile | low | no — only fixes synthetic test |
| Mesa freedreno: insert N extra `fd_batch_flush()` calls per real flush | 1-2 hr | medium | yes |
| Kernel drm/msm a2xx: synthesize N extra ringbuffer entries per `a2xx_submit()` | 4+ hr | high | yes (but invasive) |

**Recommended next step:** option 1 (cap-binary multi-render) as a falsifier — if doing 16 renders per process produces 100% `5adc3160` (only sampling the last render's output), the per-ISSUEIBCMDS hypothesis is doubly confirmed and we know option 2 (Mesa-side multi-flush) will work.  If 16-render cap still shows cycle, the counter is even higher-level (per-process, per-`open()`-of-kgsl, etc.) and option 3 is needed.

## Device state (post-test)

- mesa libgallium = v9 v2 (4f2b3b31...)
- GPU stable, 0 unexpected hangchecks during the entire 128-cap sweep
- 16 hangchecks listed in dmesg are leftover from the prior v9 v1 (1-dword NOP IB) failed test; not from this sweep
