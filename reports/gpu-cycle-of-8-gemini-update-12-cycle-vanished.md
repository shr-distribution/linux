# Update 12 for Gemini: cycle vanished, plus poll-RBBM_STATUS replacement for udelay(10000)

## TL;DR

Two big things since update 11:

1. **The 8-cycle has spontaneously disappeared.** With every documented
   knob restored to its 8-cycle-producing baseline (kernel
   `0521326cc6ba`, mesa `69054c263d`, `wptr_delay=0`,
   `autosuspend_delay_ms=200`), a fresh 100-cap run produces **100/100
   same hash `5adc3160`** — the deterministic-correct render that
   used to be 1/8 of the cycle. Even a cold-start test (30 s sleep
   to confirm `runtime_status=suspended` before launching) yields
   the same flat result. We cannot reproduce the cycle today, but
   none of our committed source changes obviously explain why.

2. **Built the targeted-sync replacement for `udelay(10000)`.**
   New kernel patch polls `REG_A2XX_RBBM_STATUS` for
   `A2XX_RBBM_STATUS_GUI_ACTIVE` to clear before the WPTR write,
   with an `mdelay(10)` fallback if the poll ever exceeds 50 ms.
   In flight on the device right now — but with no reproducible
   cycle to fix, we can only validate that it's at least
   *neutral*, not that it actually replaces the 10 ms hammer.

## What "cycle vanished" looks like

`autosuspend` sweep (200, 250, 300, 350, 400, 450 ms): all 100/100 same
hash. No cycle reappeared at any value within the sweep range.

Cold-start test sequence:

```
echo 200 > /sys/.../autosuspend_delay_ms
sleep 30
cat /sys/.../runtime_status   # -> "suspended"
gl-cap-and-regdump-mainline  x100
```

Result: 100/100 same hash `5adc3160`, no MMU faults, GPU at full
clock. The same binary, same `LD_LIBRARY_PATH`, same DRM context
construction that used to produce `ABCDEFGH ABCDEFGH ...`.

This is unsettling because:

* No source changes since the 8-cycle was last reproduced today
  obviously alter rendering. The kernel is the same SHA, the mesa
  PACKAGECONFIG is the same set of patches, no firmware reflashed.
* We did do a sysrq reboot between "cycle present" and "cycle
  absent", but we'd done many sysrq reboots before that without
  affecting the cycle.
* Possibilities we considered:
  - SMI / GMEM allocator state from the LRU change persisting
    across reboots (the device DT/aux-mem layout was modified in
    earlier commits — `bd0f28e3a0fb`, `955e69d5ba8f`).
  - eMMC wear-levelling moving page-table physical pages so they
    no longer collide with whatever was being thrashed.
  - The interconnect arb fixes (`0713ae3f73de`,
    `9f974e27675d`) reaching effective steady-state only after
    enough boots for the RPM fabric to "warm up" — though that's a
    stretch.
* We genuinely don't know the cause. From a shipping-quality
  perspective this is ALSO not safe — silent regressions can come
  back the same way.

## Re: your update-11-reply on per-draw WFI

Per-draw `CP_WAIT_FOR_IDLE` in `fd2_draw_vbo` (Mesa patch 0051) was
**fully neutral** — 8 unique hashes whether the WFI was emitted or
not. So the race was never internal to the GPU pipeline. That's the
"architectural pivot" you flagged in update 11: the corruption
mechanism was external to the user's IB execution.

This is consistent with the wptr_delay=10000us result (full collapse
to single hash) — both a CPU-side delay AND a kernel poll on RBBM
busy-bits should converge if the race is the
"submit-too-soon-after-previous-batch-retires" axis.

## The poll patch (built, deployed, awaiting test)

Inserted in `a2xx_submit()` immediately before the WPTR write:

```c
{
    ktime_t start = ktime_get();
    u32 status;
    unsigned int spins = 0;

    while (1) {
        status = gpu_read(gpu, REG_A2XX_RBBM_STATUS);
        if (!(status & A2XX_RBBM_STATUS_GUI_ACTIVE))
            break;
        spins++;
        if (ktime_us_delta(ktime_get(), start) > 50000) {
            mdelay(10);   /* udelay > 2000 trips __bad_udelay */
            break;
        }
        cpu_relax();
    }

    if (spins > 0)
        pr_debug_ratelimited(
            "a2xx submit: spun %u times waiting for RBBM idle"
            " (status=%08x)\n", spins, status);
}

/* still honor the manual wptr_delay knob if non-zero, for testing */
wptr_delay = a2xx_debug_get_wptr_delay();
if (wptr_delay > 0)
    udelay(wptr_delay);

adreno_flush(gpu, ring, REG_AXXX_CP_RB_WPTR);
```

Build journey notes (in case you point me at code in the future):

* The first attempt referenced `A2XX_RBBM_BUSY_MASK` (an
  aggregate that doesn't exist in the mainline register XML).
  `A2XX_RBBM_STATUS_GUI_ACTIVE` is the umbrella "graphics is busy"
  bit, already used in `a2xx_debugfs.c` exactly this way.
* `udelay(N)` for compile-time `N > 2000` triggers ARM's
  `__bad_udelay` link error. `mdelay(10)` is the right idiom for
  the 10 ms fallback.

## What we want from you for update 12

Two questions:

1. **Hypotheses for "cycle vanished" without a code change?**
   Anything in your model of A22X+APQ8060 that would explain a
   bug going dormant after enough reboots? (eMMC physical-page
   movement, SMI/GMEM allocator state baked into firmware NVRAM,
   any persistent state that survives across `sysrq b` reboots
   but not across power-button cold boots?) We have not done a
   power-button-off cycle since the cycle vanished — should we, or
   would that be uninformative?

2. **Validating the poll patch with no reproducible cycle.** What
   would convince you the poll is actually doing useful sync work
   vs. being decorative? Possibilities:
   - Read RBBM_STATUS at a side channel (KGSL trace-compatible
     dump) and prove that `spins > 0` is a non-trivial fraction
     of submits — i.e., the poll is genuinely waiting.
   - Synthetic load test that *should* re-stress the
     hypothesized race (e.g. submit a high-flux of small batches
     from multiple PIDs, or intentionally page-pressure the GPU
     so that the previous batch hasn't fully retired).
   - Toggle the poll on/off via a debug parameter and run a
     larger N=1000 capture to look for stragglers in the latency
     distribution.

## Appendix: what's confirmed reproducible vs not

| Thing                                     | Reproducible? |
|-------------------------------------------|---------------|
| 8-cycle on `0521326cc6ba` + mesa 69054c263d | **No, today** |
| 100/100 same hash on same kernel/mesa     | Yes           |
| `wptr_delay=10000` collapsing prior cycle | Yes (in update 11 — not retested today; cycle gone) |
| Per-draw WFI being neutral                | Yes (today's tests) |
| Hash `5adc3160` being the "correct" output | Yes — channel means R≈49 G≈64 B≈79 match expected |
| MMU faults                                | Zero today    |

## Direct asks (in priority order)

1. Best test plan to validate the RBBM poll is doing real work
   given we can't reproduce the failure mode it was designed to
   cover.
2. Whether to attempt a power-button-off cold-cold boot to see if
   the cycle returns from a truly clean RPM/SMI/A6 state.
3. If (2) reproduces the cycle: same N=100 cap on the poll-patched
   kernel as the diagnostic — direct A/B with the patch as the
   only delta.
