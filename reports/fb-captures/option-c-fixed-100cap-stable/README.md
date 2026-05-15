# Option C-fixed step 1: 100-cap stability test

Kernel: tenderloin/6.18/upstream-patches @ `6f24762bc238`
Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046
Params: `a2xx_force_collapse_on_suspend=Y` (default re-enabled), other knobs default.

## Run

100 invocations of `gl-cap-and-regdump-mainline` back-to-back, fresh DRM
context each time. No reboots between captures. tmpfs cleaned before
run.

## Result

* **unique hashes: 100/100** (every render produces a different MD5)
* **hangchecks: 0**
* **MMU faults: 0**
* **`5adc3160` (historical bit-exact correct hash) frequency: 0**
* **channel means across all samples: R≈34.28, G≈34.29, B≈34.22, A≈103.376**
  to 4-5 decimal places (matches the historical correct render exactly)

## What it looks like

Every render is a correctly-interpolated RGB-vertex triangle (red
bottom-left, green bottom-right, blue top, smooth gradient).

But every render has the same **tile-boundary noise overlay** we
saw in the earlier Option C tests:

* Vertical white stripe at x≈512 (GMEM tile column boundary on the
  2-col × 3-row 512×256 tile layout)
* White "spurs" along triangle silhouette (edge MSAA coverage)
* Random bright pixels in left margin (background should be the
  clear color but is uninitialised post-rail-drop)
* Text-like garbage near bottom-center (deterministic uninit memory
  pattern)

The specific noise PIXELS shift between runs (~106K pixel diff
between samples in different clusters, ~25 px diff within a cluster
from sub-pixel rasterisation jitter), but the noise PATTERN is the
same shape every time.

See `mosaic_9way.png` for visual comparison.

## Pixel-diff matrix (5 random samples)

```
  578a575b -> 5f01d43b : 106 066
  578a575b -> 687d3247 : 106 071
  578a575b -> 851f4db7 : 106 069
  578a575b -> bb7e15fb : 106 071
  5f01d43b -> 687d3247 :      21    <- same cluster
  5f01d43b -> 851f4db7 :      28
  5f01d43b -> bb7e15fb :      26
  687d3247 -> 851f4db7 :      30
  687d3247 -> bb7e15fb :      29
  851f4db7 -> bb7e15fb :      28
```

4 samples in one cluster (~25 px within-cluster), 1 sample in
a different cluster (~106K from cluster A). Consistent with the
earlier finding that captures fall into ~4 distinct noise phases.

## Comparison with baseline (no force_collapse)

Baseline 100-cap produces 8 deterministic hashes:
* `5adc3160` (correctly-interpolated triangle, white bg) - ~12.5%
* 7 other hashes with **visually broken** outputs:
  * `c399f1f4` (blue-only, R/G=0)
  * `29fcdfdf` (dark blue tinted)
  * `fb12cd4c` (faint with blue dominance)
  * etc.

So the tradeoff is:

| | Baseline | Option C-fixed step 1 |
|---|---|---|
| Bit-exact correct rate | 12.5% (1/8) | 0% over 100 |
| Visually-correct triangle | 12.5% | 100% |
| Visually-wrong / wrong colors | 87.5% | 0% |
| Noise overlay | None | Present (tile boundaries) |

For UI/compositing workloads, Option C-fixed is significantly better
(every frame approximately right) despite never being bit-exact. For
pixel-exact regression testing, neither is acceptable.

## Files

* `sample-*.bin` × 100 — raw 1024×768 RGBA captures
* `sample-*.png` × 100 — PNG conversions
* `hashes.txt` — full 100-line ordered list of hashes
* `mosaic_9way.png` — visual mosaic of 9 random samples
* This README

## Conclusion / next options

Option C-fixed step 1 alone doesn't reach 100/100 bit-exact correct.
Per Gemini update 26-27 analysis, the cycle's state lives in
non-register-accessible SRAM. Register-diff diagnostic confirmed
all register snapshots are bit-identical across runs producing
different output hashes.

Remaining paths:
1. **Option C-fixed step 2** — add explicit GMEM init via PM4
   sequence in `a2xx_hw_init`. Mesa would have to cooperate (likely)
   or kernel emits Mesa-equivalent clear draws.
2. **Option E** — Mesa-side dummy-draw scrub (8 draws with 32-GPR
   shader). Risk: another hang like patch 0070.
3. **Option F** — accept this as the shipping state (UI looks fine,
   pixel tests can use perceptual hashing).

Branch: `tenderloin/6.18/upstream-patches` tip `6f24762bc238`.
