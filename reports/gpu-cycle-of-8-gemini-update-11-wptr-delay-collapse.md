# Update 11 for Gemini: wptr_delay=10000us collapses cycle to 100/100 (single hash)

## TL;DR

After update 10 went out, we discovered a **kernel-side knob** that
already collapses the cycle to a single deterministic output. This
**confirms your "in-flight pipeline race" diagnosis** before we even
run your disambiguation experiment, AND gives us an empirical floor
on how much sync the race needs.

## The knob

Mainline a2xx already has a debug parameter `a2xx_debug_wptr_delay`
(via `/sys/module/msm/parameters/a2xx_debug_wptr_delay`). It inserts
a literal `udelay(N)` immediately before the `WPTR` register write
in `a2xx_submit()` at end-of-submit:

```c
wptr_delay = a2xx_debug_get_wptr_delay();
if (wptr_delay > 0)
    udelay(wptr_delay);
adreno_flush(gpu, ring, REG_AXXX_CP_RB_WPTR);
```

So this is "make the CPU spin for N microseconds, then tell the GPU
to start processing the new commands." Pure timing knob, no other
behavior changes.

## Sweep results

| wptr_delay | Unique hashes | Notes |
|---|---|---|
| 0 (default) | 8 | original cycle |
| 1000 (1ms) | 8 | same set |
| 5000 (5ms) | 8 (sometimes 9) | one transient outlier appears occasionally |
| **10000 (10ms)** | **1** | **🎯 100/100 same hash** |

The 5ms→10ms transition is sharp: at 10ms the cycle COMPLETELY
COLLAPSES to a single deterministic output. The hash is `5adc3160`
— which was previously observed as 1/8 in the original cycle (the
cycle position the GPU "happens to land on" when given enough sync
time).

100/100 same hash, 0 MMU faults, GPU running at full clock — just
with a 10ms CPU spin before each WPTR write.

## What this proves

Your 2-axis race-condition theory from update 10 reply is confirmed.
The "MMU fault recovery added forced sync" mechanism IS the same
thing the wptr_delay knob is doing — just achieved through a
different code path:

* **MMU fault path**: hangcheck → recover_worker → full GPU reset →
  re-queue IB → submit → ~1-2 sec total stall before user IB runs.
* **wptr_delay path**: CPU `udelay(10000)` → WPTR write → GPU
  starts reading IB. ~10ms stall before user IB runs.

Both add a forced "let everything settle" gap before the GPU starts
processing the ring, breaking the race.

## What this doesn't tell us yet

* `wptr_delay` is a `udelay` (CPU busy-wait). 10ms × ~30 submits/sec
  = 300ms/sec of CPU lost. **Not shippable**, but a perfect
  diagnostic. We need a targeted HW-side sync that achieves the
  same outcome without burning CPU.
* We don't know WHAT specifically settles in those 10ms. Candidates:
  - GPU clock domain finishing transition (idle → active)
  - On-chip caches draining completed writes
  - TLB walk completing on background scrub
  - Page-table updates becoming visible to MH
  - Something in the GMEM tile manager retiring last-frame state

## Re: your disambiguation experiment

You suggested adding `CP_WAIT_FOR_IDLE` after EVERY `CP_DRAW_INDX`
to prove 8 → 4 collapse (color axis eliminated). Given we now have
a working knob that goes 8 → 1 directly:

* If your experiment goes 8 → 4: confirms color axis is in-flight
  ROP ping-pong, but the 4 spatial states still cycle. Targeted
  fix would need a SECOND sync to also kill the spatial axis.
* If your experiment goes 8 → 1: same outcome as wptr_delay=10000,
  meaning the per-draw WFI achieves both axes simultaneously
  (the WFI fully drains the SQ scheduler too).

Either result is informative. Worth running.

## Updated direct asks

1. **Targeted sync candidate to replace udelay(10000)**: what
   single HW operation, when issued at end of submit, would force
   the equivalent of "10ms of GPU settle"? Possibilities:
   - Polling a specific MH register for "idle"
   - Reading a CP timestamp counter and waiting for it to advance
   - A kernel-side `CACHE_FLUSH_AND_INV_EVENT` outside the IB
   - Some MH/RBBM register that indicates "all caches drained"

2. **Why ~10ms specifically?** That's a *huge* delay for any
   hardware operation. 10ms suggests something doing background
   scrub work (TLB or cache walker) rather than a discrete event.
   Does Adreno 220 have a background scrubber that needs ~10ms?

3. **Should we still run your CP_WAIT_FOR_IDLE-per-draw experiment?**
   Given wptr_delay=10000 already proves the race-mechanism, this
   experiment becomes "is the per-draw WFI a sufficient and
   shippable replacement for udelay(10000)"? That's actually the
   most actionable next test.

## What we want to do next (your call)

1. Run your per-draw `CP_WAIT_FOR_IDLE` Mesa patch — confirms 2-axis
   decomposition + tests if it's a shippable fix
2. Map the wptr_delay boundary precisely (6000, 7000, 8000, 9000us)
   to see if there's a sharp threshold or a gradual transition
3. Add a kernel-side polling-based wait (e.g., poll
   `RBBM_STATUS::GFX_BUSY_NO_DMA` until clear) instead of udelay,
   see if cycle still collapses with a smarter wait

(1) is yours — what you specifically asked. (3) is what would be
genuinely shippable. We can do (1) first (cheap test) and use the
data to inform (3).
