# v6 T1 default — visual capture analysis

**Date:** 2026-05-12
**Mesa libgallium md5:** `cdd6187d8a71ebf964e1920243cc8eb7`
**Kernel:** `6.18.0-luneos-g1b49d399fb96`
**Env:** no env-vars (default Test 4 baseline path)
**Geometry under test:** GLES2 "Hello Triangle" — equilateral triangle with vertex colors
  - Bottom-left vertex → `(1, 0, 0, 1)` RED
  - Bottom-right vertex → `(0, 1, 0, 1)` GREEN
  - Top vertex          → `(0, 0, 1, 1)` BLUE
  - Interior interpolates as standard RGB barycentric gradient.

10/10 captures, 10 unique hashes (period >= 10 this run).  Same 1024x768 tile-binned resolution as prior baselines.

## The known-good output

**`5adc3160`** — clean RGB gradient triangle, vertex colors in correct corners. Matches the intended GLES output.

## What the "wrong" outputs reveal

Every other sample is some combination of two distinct corruption modes:

### Mode A — vertex-color rotation (whole-triangle)

Some captures render the entire triangle with the wrong assignment of vertex colors to corners — as if the vertex attribute array was read in a rotated order:

| sample | top vertex | bottom-left | bottom-right |
|---|---|---|---|
| `5adc3160` ✓ | blue | red | green |
| `e93d10aa`   | red  | green| yellow (red+green) |
| `4477e60a`   | red  | green| yellow |
| `3625b67f`   | red  | (per-tile garbage) | (per-tile garbage) |
| `e21b9529`   | red  | green| yellow |
| `03dee03a`   | (blue tile only at top) | green | orange/yellow |

When the top is RED, the bottom-right is YELLOW.  That's `R + G` mixed — consistent with the bottom-right vertex having BOTH the R and G channels driven (instead of just G).  So the vertex attribute swizzle is not just a rotation; some captures get color **vector components mixed across vertices**.

### Mode B — per-tile color patches (individual tiles drawn with different state)

Multiple captures show **rectangular patches** where one tile's worth of pixels were drawn with completely different vertex colors than the surrounding tiles:

- `e93d10aa` — bright green rectangle in bottom-left tile
- `4477e60a` — black/very dark patch in top-middle-right tile
- `bebc09c6` — horizontal split: top half correct, bottom half wrong assignment
- `3c9c950b` — yellow-green rectangle in bottom-right tile
- `202cfe9f` — bright orange rectangle in top-middle-left tile
- `3625b67f` — multiple distinct tile patches at bottom-left and bottom-right
- `9fb336c8` — at least 4 distinct tiles each with different vertex assignment
- `e21b9529` — dark red rectangle in middle tile

The patch boundaries align with the 1024x768 tile-binner partitioning (each rectangle is one of the 6 visible 384x384 tiles in the 2x3 tile layout).

## Interpretation

The combination of Mode A (whole-triangle wrong) and Mode B (per-tile wrong) suggests the corruption is in **per-tile vertex-attribute state**, not in coverage / visibility / depth.  Each tile picks its vertex-attribute snapshot from a small pool of internal states; the cycle of ~8 distinct states matches the Adreno 220 SQ wavefront slot count (8 slots).

When all tiles happen to pick the SAME correct state simultaneously, we get `5adc3160` (1 in 8 baseline rate).  Otherwise we get a mosaic of tile-local mis-assignments.

This is consistent with the long-running hypothesis that the period-8 cycle is **SQ wavefront-slot rotation** — the binner / wavefront scheduler picks a different slot per tile, and each slot has its own latched vertex attribute state, with only one of the 8 slots holding the correct interpolant constants for this draw.

## Files

| file | meaning |
|---|---|
| `5adc3160.png` | known-good baseline (label also as `KNOWN_GOOD-*`) |
| `<hash>.png`   | rendered framebuffer at that hash, converted from RGBA8 1024x768 |
| `sample-<hash>.bin` | raw 1024x768 RGBA8 framebuffer dump |

Hash capture order (run sequence): `5adc3160`, `4477e60a`, `bebc09c6`, `3625b67f`, `03dee03a`, `9fb336c8`, `e93d10aa`, `202cfe9f`, `e21b9529`, `3c9c950b`.
