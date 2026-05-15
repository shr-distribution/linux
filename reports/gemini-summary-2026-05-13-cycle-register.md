# Gemini consultation: A22X period-16 render cycle — found the register, can't reset it

**Date:** 2026-05-13
**Platform:** HP TouchPad, APQ8060, Adreno 220 (A22X / REV470), mainline Linux 6.18, Mesa freedreno
**Goal:** Fix deterministic period-16 cycle where only 1 in 16 hello-triangle renders is bit-exact correct (the other 15 show vertex-color rotation + per-tile mosaic corruption).

---

## TL;DR

We isolated the cycle to **MMIO word offset 0x0ee2** (byte 0x3b88), an undocumented TC/TP-area state register that advances deterministically per render. The boot value `0x15000002` produces the correct frame (`5adc3160`). We've burned 6 weeks chasing wrong hypotheses (binner state, SQ slots, GDSC retention, MSM_SUBMIT ioctl counter, kernel-side resets). All ruled out. Now we know which register but it appears **read-only from CPU MMIO** AND **not reset by RBBM_SOFT_RESET**. We need ideas on what GPU operation drives the advance.

---

## The cycle (10-cap diagnostic on fresh boot, FD2_FLUSH_MULTIPLIER=1)

```
boot:  15 00 00 02       (no cap yet)
cap 1: 15 02 02 02  ->  hash 5adc3160  (CORRECT frame, "phase 0")
cap 2: 15 04 04 02  ->  hash 0299c275
cap 3: 15 06 06 02  ->  hash bebc09c6
cap 4: 15 08 08 02  ->  hash f31c8916
cap 5: 15 01 c1 c2  ->  hash 03dee03a   (wrap, byte0 changes)
cap 6: 15 03 c3 c2  ->  hash 90986eb0
cap 7: 15 05 c5 c2  ->  hash 53fb83a8
cap 8: 15 07 c7 c2  ->  hash 03aa0743
cap 9: 15 01 81 82  ->  hash a8728c29   (second wrap)
cap 10: 15 03 83 82 -> hash 3c9c950b
...
```

Period is exactly 16. Every 16th render produces the correct `5adc3160` frame. Across all 10 caps, only register 0x0ee2 advances in a way that correlates with cap output (everything else is noise hashes).

High byte `0x15` is constant (chip-ID-like).  Low 24 bits encode the state machine. Within each "block of 4" the low byte stays constant (02, c2, 82, etc.) and bytes 1/2 advance by +0x0202.  At block boundaries byte 0 changes.  Total period: 16 distinct states.

## What we know about 0x0ee2

- **Location:** word offset 0x0ee2 = byte offset 0x3b88. In the undocumented region after `TCF_PERFCOUNTER` (last documented at 0x0e7d), before `SQ` group at 0x2000+.
- **Kernel debug-dump array `a220_registers[]`** explicitly includes `0x0EE0, 0x0EE2` — qcom devs knew it existed.
- **Mesa's `a2xx.xml` has no definition for it.**
- **`reg32 offset="0x0ee0..0x0ee2"` undocumented** but adjacent registers are `TC*_PERFCOUNTER*` and `TP0_*`. Strongly suggestive of **TC (Texture Cache) or TP (Texture Pipeline) internal state**.

## Falsifications (don't repeat these)

### Direct write — RO from MMIO
- Wrote `0xcafe0001` via kernel debugfs `regrw_value` (which calls `gpu_write()` while pm_runtime held active). Read back `0x15000002` — unchanged.
- Same kernel debugfs wrote `0xdeadbeef` to scratch reg `0x578` successfully. So the write path itself is correct; 0x0ee2 is just RO.

### RBBM_SOFT_RESET (full pulse) — no effect
- Wrote `0xffffffff` then `0` to `REG_A2XX_RBBM_SOFT_RESET` (word 0x003c). 0x0ee2 unchanged. So whatever block the register belongs to isn't reset by the "kill everything" pulse.
- Our kernel already does this pulse per-submit (legacy WPTR-poll workaround) — cycle persists across thousands of pulses.

### Multi-flush MSM_SUBMITs — counter NOT per-ioctl
- webOS strace shows 16 `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS` per render. Mainline Mesa freedreno does 1 ioctl per render.
- Hypothesis was: counter advances per MSM_SUBMIT → 16-per-render would wrap mod-16 → no cycle.
- Tested 5 variants (v1 pre-submit bare CP_NOP, v2 cache-flush body, v3 post-submit, v4 fence-wait, v5a zero-vertex DRAW_INDX). 
- v3 (no wait) produced "advance by ~1 per render regardless of multiplier" — meaning **15 of 15 dummies failed to advance 0x0ee2**.
- v4/v5a hung the GPU.
- **Conclusion: 0x0ee2 is NOT advanced per arbitrary MSM_SUBMIT**, only per "real" Mesa render with full state + draws + resolves.

### No vendor libGLESv2 writes 0x0ee2
- Decompiled webOS, HTC, Samsung, Xiaomi A22X libGLESv2.so binaries via Ghidra. Searched for 0x0ee2 / 0xee0 / 0x3b88 / nearby. **Zero references in any vendor driver.**
- So vendor drivers also don't explicitly reset it — they must be doing something else that keeps the state machine harmless.

### Many other red herrings already ruled out
- SQ wavefront slot scrub (POINT draws, GPR pulse, INVALIDATE_STATE reorder) — no effect on cycle, only minor shift
- LRZ_VSC_CONTROL / VSC_PIPE binner state — cycle is HW-binner-independent (Fork A/B test)
- VGT_CURRENT_BIN_ID — cycle invariant when binning disabled
- GFX3D GDSC force_collapse + retention clear — doesn't reset cycle (only adds noise)
- GPU clock frequency 96-320 MHz — cycle frequency-invariant
- CPU governor / affinity — invariant
- IOMMU domain / fault handling — invariant
- per-CP_INDIRECT_BUFFER NOP padding — invariant
- Per-batch state-dirty marking — invariant

## What works (1-frame-correct)

After a fresh kernel boot, the FIRST GLES render produces `5adc3160` — the bit-exact-correct triangle. After that we walk through the 16 states, only landing back on `5adc3160` every 16th render.

## Concrete questions for Gemini

1. **What GPU operation triggers 0x0ee2 to advance?**  Given (a) it's in the TP/TC undocumented region, (b) RBBM_SOFT_RESET doesn't reset it, (c) bare MSM_SUBMITs with cache-flush events don't advance it, and (d) actual Mesa renders DO advance it — what specific PM4 packet or GPU event in a normal Mesa render flow is the most likely trigger?  Candidates we suspect: texture sampler binding setup, the CP_DRAW_INDX of the resolve quad (RECTLIST), the per-tile gmem2mem copy, or some MEM_WRITE side-effect.  Which would you prioritize to test first?

2. **Is there a non-RBBM block reset we should try?**  A2XX has block-specific reset bits scattered across various registers (e.g., on Radeon-derived parts, TP_RESET / TC_RESET / SC_RESET were sometimes in `SOFT_RESET_TP_SEL` style registers).  Do you know of, or can you find, **any A2XX register that resets just the TC/TP block state** without resetting the whole GPU?

3. **Could 0x0ee2 be a "performance counter selector" survivor?**  Adjacent registers 0x0e** are perfcounter SELECT/HI/LOW.  Could 0x0ee2 be a perfcounter that ALWAYS counts something (like "draws retired" or "tiles resolved") regardless of selector?  If so, can we **disable counting** via a parent register write?

4. **What's the bit-encoding of the state machine?**  Looking at the sequence (`02 02 02`, `04 04 02`, ..., `01 c1 c2`, `03 c3 c2`, ..., `01 81 82`, `03 83 82`), do you see a clear pattern — e.g., **two bit-reversed counters** or **a Gray-code modulo something**?  If we can decode the state machine, maybe we can predict / force the "5adc3160 phase".

5. **Is the boot value `0x15000002` significant?**  High byte `0x15` is constant.  Low 24 bits `0x000002` at boot, `0x020202` after render 1.  Is `0x15` a literal "Adreno 220 r0p2" chip-rev encoding showing through, and the low bits a register that happens to be writable by GPU-internal but masked from CPU-MMIO writes?

6. **Webos approach hypothesis:** webOS sends 16 ioctls per render where Mesa sends 1.  Each webOS ioctl is a smaller piece of the per-tile work.  Could the extra ioctl boundaries be triggering some implicit `WAIT_FOR_IDLE` or `TC_INVALIDATE` event between tiles that we're missing?  Specifically: **does the A2XX have a CP_EVENT_WRITE event that resets / advances the TC state machine**, and would webOS naturally emit it 16x while Mesa emits it 1x?

7. **The "correct" frame is phase 0, not the average.**  Of 16 phases, ONE produces the correct triangle and 15 produce visibly corrupt mosaics.  This implies the state machine drives some "tile index" or "bucket selector" that's *only correct at offset 0*.  Does this fit your model of A2XX texture cache or pipeline arbitration?  If yes, the fix isn't "reset the counter" but **"saturate it so it never advances"** or **"only render from phase 0"** — what would saturate it look like?

8. **Sanity check:** is there any way the kernel driver could be missing an `MH_FLUSH` / `MH_INTERRUPT_CLEAR` / `TC_CNTL_STATUS_FLUSH` event that webOS / vendor KGSL would have emitted between submits?  Specifically: webOS kgsl writes `TC_CNTL_STATUS = INVALIDATE` between submits as part of its idle-handler — we don't do this.

## Repo references

- Cycle counter findings: `reports/v5-cycle-counter-2026-05-13.md` (this report's data)
- 10-cap register snapshots: `reports/fb-captures/mainline-newkernel-10runs/`
- Multi-flush falsification series: `reports/v9-nop-ib-sweep-2026-05-13.md`, `reports/v10-design-multi-flush-2026-05-13.md`
- Kernel debug-dump register array: `drivers/gpu/drm/msm/adreno/a2xx_gpu.c::a220_registers[]`
- Mesa patch stack: `meta-mainline/recipes-graphics/mesa/files/0001..0013-*.patch`

## What we DON'T need from Gemini

- Suggestions to try more multi-flush variants (we've burned 5 already, all dead-ends).
- Suggestions to power-cycle GDSC (force_collapse already does this between submits, didn't help).
- Suggestions to re-emit state every batch (we already do FD_DIRTY_ALL_BIT on every batch start).
- More CP_WAIT_FOR_IDLE / cache flush sprinkling (already at maximum, all events emitted).

We need **a concrete pointer to what 0x0ee2 is** (TC bucket selector? TP arbitration LFSR? perfcounter?), and/or **a specific register write or PM4 event that would either disable or saturate its advance**.

---

## Update (post-Gemini-round-1, 2026-05-13)

Gemini's round-1 response (16-slot allocator pool hypothesis) prompted three more falsifications:

1. **TC_CNTL_STATUS L2_INVALIDATE (bit 0 of word 0x0e00) does NOT reset 0x0ee2** — wrote 0x1 between every cap, register continued advancing at +1/cap rate. Cycle unchanged.
2. **RBBM_SOFT_RESET offset corrected** — confirmed word 0x003c (byte 0x00F0) is the right offset (mesa+kernel both agree, Gemini's 0x03E0 suggestion was wrong). Still no effect on 0x0ee2 from the full 0xffffffff pulse.
3. **Vendor binary event survey done**: webOS libGLESv2 emits CP_EVENT_WRITE 5x for events 0x06 (CACHE_FLUSH), 0x17 (PERFCOUNTER_START or IN_IB_PREFETCH_END), 0x18 (PERFCOUNTER_STOP), 0x1c (FACENESS_FLUSH). HTC/Samsung/Xiaomi A22X libGLESv2 emit ZERO CP_EVENT_WRITE — they take a different path entirely. **Mesa freedreno A2XX emits CACHE_FLUSH_AND_INV_EVENT (0x16) only, not 0x06 / 0x1c / 0x17 / 0x18.**

## Updated questions for Gemini round 2

9. **Given Gemini-round-1's "16-slot allocation pool" model is now stronger** (no CPU-side reset works), what GPU-internal operation deallocates a slot?  Standard hypothesis: VS_DEALLOC (event 0x00) + PS_DEALLOC (event 0x01) emitted via CP_EVENT_WRITE.  But searched webOS, HTC, Samsung, Xiaomi vendor libGLESv2.so — **none of them emit 0x00 or 0x01 events**.  Either (a) the events are auto-emitted by something else (CP firmware? VGT?) without explicit driver request, or (b) the slots are NOT released by these events.  Which is it?

10. **HTC/Samsung/Xiaomi vendors emit ZERO `CP_EVENT_WRITE` packets at all in their A22X libGLESv2**.  They produce correct output on the SAME hardware.  This implies the cycle is **a Mesa-specific bug** caused by something Mesa does that vendor drivers don't do — not something Mesa is missing.  Hypothesis: Mesa's IB1→IB2 split or batch->binning structure consumes slots in a way that doesn't deallocate, while vendor drivers' single-IB structure doesn't consume them in the first place.  Plausible?  Where in Mesa freedreno A2XX would the extra slot consumption come from?

11. **FACENESS_FLUSH (0x1c) is emitted only by webOS, not by HTC/Samsung/Xiaomi.**  webOS sometimes shows mosaic-free output, sometimes not — but we haven't decoded its full state machine.  Is FACENESS_FLUSH the only A2XX event that *deallocates* state from the per-tri faceness cache (which is one of the 16-entry pools)?

12. **PERFCOUNTER_START (0x17) / PERFCOUNTER_STOP (0x18) — used by webOS but not other vendors.**  Could starting/stopping a perfcounter have a side-effect of "tagging" a slot, allowing the GPU to clean it up on the next draw?  Or are these purely measurement?

13. **Last-ditch idea:** the high byte `0x15` in 0x0ee2 is invariant.  If we read 0x0ee0 + 0x0ee1 (the immediate neighbors that the kernel dump array includes), do we see correlated state machine entries?  Have we missed that 0x0ee0/0xee1 might be the "head/tail pointers" while 0x0ee2 is the "current slot encoding"?  Would testing those reveal a writable head/tail we can reset?
