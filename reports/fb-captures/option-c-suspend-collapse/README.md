# Option C suspend-collapse test (auto-force_collapse on pm_runtime)

Kernel: tenderloin/6.18/upstream-patches @ `174a7b70af96`
Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046

Test: `a2xx_force_collapse_on_suspend=Y` (default) makes the GPU
driver explicitly drop the GFX3D rail on every `a2xx_pm_suspend`
(clear ENABLE+RETENTION via direct ioremap to MMCC GDSCR @ 0x4000188)
and pulse GFX3D_RESET (MMCC 0x4000210 bit 12) on every
`a2xx_pm_resume`. With knob=N the patch is a no-op (mainline
behaviour: GDSC stays warm in retention mode permanently).

## Phase A: knob=Y, 100 renders

* unique sample bins: **100/100**
* hangchecks: 0
* MMU faults: 0
* channel means: R≈34.2833-34.2844, G≈34.2872-34.2878, B≈34.2230-34.2239,
  A≈103.376-103.377 (variance at 4th-5th decimal place)

**All renders have visually-identical output** — channel means agree
to 4-5 decimals across all 100 samples. None of the 7 "broken" baseline
hashes (`c399f1f4` blue-only, `29fcdfdf` dark-blue, etc.) appear.

### But: per-pixel content still clusters into 4 distinct states

Pixel-diff measurements vs `sample-9f76b73f.png` (first in run order):

| Cluster | Sample count | Pixel diff vs ref |
|---------|--------------|-------------------|
| c0      |  28          |  22-29 pixels (essentially identical) |
| c1      |  44          | ~106 000 pixels                       |
| c2      |   4          | mixed (~106K-127K)                    |
| c3      |  23          | ~148 000 pixels                       |

So the 100 unique MD5s are not random noise — they fall into ~4 distinct
per-pixel "phases" of the same visual render. Within a cluster, runs
differ by ~25 pixels (rasterisation edge jitter). Between clusters,
runs differ by ~106K-148K pixels in a deterministic way.

Sequential pair diffs (consecutive runs in `hashes.txt` order):
```
run[0]  9f76b73f vs run[1]  18578a01 :     26   (same cluster)
run[5]  00d851d5 vs run[6]  d5c8dafd : 106 062  (cluster jump)
run[10] e0a8a169 vs run[11] 92e1d4ad : 106 066  (cluster jump)
run[50] fd331157 vs run[51] 328f571c : 148 959  (cluster jump)
run[95] 0f93048b vs run[96] 98fd4176 : 127 629  (cluster jump)
```

Cluster jumps happen frequently (more than half the consecutive pairs),
suggesting the underlying state machine has a small period (maybe 3-4)
that the GPU rotates through despite our reset pulse.

## Phase B: knob=N, expected: cycle reappears

Test was interrupted after 58 runs. All 58 produced `d1dd210d`
(all-zeros, channel mean A=0, RGB unparseable). **All 58 hung.**
59 hangchecks recorded in dmesg.

**Important caveat**: this is a hot-toggle artifact. Phase A's last
runtime-suspend dropped the rail (knob was Y). Then we set knob=N,
ran a render → the kernel called `a2xx_pm_resume` but skipped
`a2xx_force_gdsc_enable_and_reset` (because knob is now N), leaving
the rail collapsed. Result: all subsequent renders hang on a dead
rail.

To do a clean A/B between knob=Y and knob=N, the device needs to be
**rebooted** between knob-state changes. The knob isn't designed for
hot-toggling mid-session.

## Conclusion

The Option C patch:

1. **Eliminates the visual period-8 cycle** — no more
   `c399f1f4`/`29fcdfdf`/`fb12cd4c` etc. broken renders. All
   outputs have correct channel means (R=G=B=34, indicating the
   uniform-grey triangle).

2. **Still has a smaller residual cycle** — 4 distinct per-pixel
   states, all with the same channel means. Each state has ~25
   pixels of edge-jitter within it but ~106K-148K pixels of
   systematic difference vs other states.

The remaining 4-state period is much subtler than the original
8-cycle. Whether it matters depends on the use case:

* For 2D UI / desktop compositing: visually identical, no observable
  artifact.
* For pixel-perfect tests (game frame replays, compute kernels): the
  ~106K-pixel difference per cluster would show as a flicker.

The clustering implies some hardware state STILL persists across our
power-cycle. Candidates Gemini's earlier analysis flagged but we
didn't address:

* SQ scheduler scoreboard / ROP ping-pong queues (not in VPC SRAM)
* TC (texture cache) lookup tables (only varying-cache was reset by
  GFX3D_RESET pulse)
* Fixed-function gamma/dither LUTs

## Files in this directory

* `phase-A-on/sample-*.bin` + `.png` × 100 (knob=Y captures)
* `phase-A-on/hashes.txt` — run order
* `phase-B-off/sample-d1dd210d.bin` + `.png` (the stuck-buffer all-zeros)
* `phase-B-off/hashes.txt` — confirms 58/58 d1dd210d

## Branch / SHA reference

Branch: `tenderloin/6.18/upstream-patches`
Tip:    `174a7b70af96dc72af719002370bbdc4da0242ba`
Patch:  `drm/msm: a2xx: force GDSC collapse + core reset pulse on pm_runtime cycle`
