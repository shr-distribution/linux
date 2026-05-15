# Update 23 for Gemini: honest visual assessment — Option C trades visible 8-cycle for tile-boundary noise overlay

## Correction to update 22

I led update 22 with "visual cycle ELIMINATED" because all 100
captures had matching channel means. **That was wrong.** The user
inspected the actual PNG files and pointed out the renders are
visibly broken in a different way. Channel means matching does NOT
mean visually identical — distributed RGB noise can preserve overall
means while introducing pixel-level artifacts.

## What the renders actually look like

### Baseline `5adc3160` (the historical 1-of-8 "correct" hash)

A clean RGB-vertex-colored triangle: red bottom-left, green
bottom-right, blue top, smooth gradient interpolation. Sharp edges.
**Background is white** — this is actually wrong (the test calls
`glClearColor(blue)`), but it's a pre-existing separate bug and we
treated it as "correct" because at least the triangle was right.

### Phase A captures with Option C (`a2xx_force_collapse_on_suspend=Y`)

The triangle interpolation IS correct (same red→green→blue gradient
as baseline). But there's a systematic noise overlay:

* **Vertical white stripe at x≈512** — this is exactly the GMEM
  tile boundary (1024 wide / 256 px tiles = 4 columns, boundaries
  at x=256, 512, 768).
* **White "spurs" sticking out from triangle edges** along all
  three slopes, looking like coverage-mask errors.
* **Random bright pixels in the left margin** outside the
  triangle, where the (already-wrong-white) background should be
  uniform.
* **Text-like garbage near bottom-center** of the framebuffer,
  consistent across all 100 captures.
* The garbage pattern's **specific pixels shift between runs**,
  driving the 4-cluster pixel-diff pattern from update 22 — but
  the macro distribution of noise is the same shape every time.

So the 100 unique MD5s are: triangle-correct + noise-with-shifting-
specific-pixels. The 4 clusters are 4 distinct noise instantiations
that happen to average to the same channel means.

## What Option C actually achieves vs costs

* **Eliminated**: the original 8-cycle's catastrophic visual
  failures (`c399f1f4` with B=62 R=G=0, `29fcdfdf` dark-blue, etc.).
  All 100 renders now have the correct triangle interpolation.

* **Introduced**: distributed noise overlay structured around GMEM
  tile boundaries, edge coverage, and apparently uninitialised
  off-triangle pixels.

* **Did not fix**: the missing-blue-background bug (predates
  Option C, all renders including baseline-correct show white
  background instead of `glClearColor(blue)` applied).

Net: from a visual-quality perspective, Phase A captures are
**worse than the 1-in-8 correct baseline render**, and **better
than the 7-in-8 wrong baseline renders**.

## Hypothesis: GMEM tile resolve state is being lost

The artefact distribution screams GMEM tile-binner / resolve:

* Tile-boundary stripe at x=512: this is where the tile-binner's
  resolve step copies internal tile-buffer contents back to the
  framebuffer BO. If the resolve gets corrupted state, the bytes
  written at tile-column boundaries are wrong.
* Triangle-edge "spurs": MSAA coverage-mask values that should be
  partial (along the slope) are coming out fully-on for some
  pixels, suggesting the coverage logic has stale state.
* Left-margin random pixels: areas the rasteriser shouldn't touch
  are getting written to. Could be tile boundaries again, where
  the tile that covers the empty left margin is leaking garbage.
* Bottom-center text-like garbage: consistent across all 100
  renders, so it's *deterministic* — some specific pixel range
  that gets the same wrong value every time. Could be a tile-
  binner pointer pointing at a fixed wrong place.

The GMEM tile-binner has internal SRAM that the GFX3D_RESET pulse
might not cover. Our pulse hits the core_clk reset (resetting SQ,
VPC, and probably ROP). The tile-binner state machine could be in
a separate clock domain or reset chain.

## What I'd ask of you

1. **Is the GMEM tile-binner reset reachable** through some other
   register / reset bit we're not pulsing? Candidates:
   * Some bit in MMCC 0x0210 we're not touching (we pulse
     bit 12 = GFX3D_RESET; mmcc-msm8660.c shows GFX3D_AHB_RESET
     at bit 10, but there may be others)
   * `RBBM_SOFT_RESET` written from inside the GPU
   * A clock-tree reset on `gmem_axi_clk` specifically

2. **Could the tile-resolve breakage be a TIMING issue** rather than
   missing reset? If the rail comes back too fast for the tile-
   binner's internal pipelines to fully repopulate from DRAM
   (binner-state IB, tile-pointer table, etc.), the first batch's
   resolve would catch them mid-fill. Possible fix: add an
   explicit "tile-binner re-init" packet at submit time, OR
   delay longer in `a2xx_force_gdsc_enable_and_reset`.

3. **Is there a way to disable GMEM tile-rendering entirely**
   (force "sysmem" / bypass-mode rendering) as a confirmation
   test? If sysmem-bypass on cold-rail produces a clean render,
   it confirms tile-binner state is the residual issue and we'd
   know to pursue tile-binner-specific re-init.

4. **The missing-blue-background bug**: separate from this thread,
   but worth flagging. The test calls `glClearColor(0, 0, 1, 1)`
   but renders come out with a white background. This has been
   present in every capture we've ever taken — including the
   baseline `5adc3160` we treated as "the correct render". Could
   be a Mesa fast-clear vs slow-clear path issue, or `CLEAR_COLOR`
   register vs `RB_COLOR_CLEAR_VALUE` register confusion specific
   to A22X. We haven't actively pursued this because the 8-cycle
   was more pressing, but flagging it for awareness — if you have
   a quick A22X-specific take it'd save a separate investigation
   cycle.

## Where this leaves the shippable fix

Option C is **NOT shippable as-is**. We need to either:

* **Fix the tile-resolve noise**: figure out the missing reset /
  re-init and add it to `a2xx_force_gdsc_enable_and_reset`, then
  re-test. If the noise disappears and we get bit-deterministic
  renders, we're done.

* **Revert to "no force_collapse"** and accept the period-8
  cycle: at least 1 in 8 renders is sharp+correctly-interpolated.
  Worse than Option C in average case but better in best case.

* **Pursue a different mechanism**: maybe a kernel-side scrub
  that's less destructive than full rail-collapse — e.g., just
  toggle `GFX3D_RESET` periodically without dropping the rail.
  That'd clear SQ/VPC state without disturbing GMEM tile-binner.

Asking for your read on which direction is most promising.

## Captures

`reports/fb-captures/option-c-suspend-collapse/`:

* `baseline-5adc3160.png` — sharp triangle, white bg (pre-Option-C
  "correct" reference)
* `phase-A-on/sample-9f76b73f.png` — typical Phase A: triangle +
  tile-boundary noise overlay (cluster c0)
* `phase-A-on/sample-80fa6e89.png` — cluster c1, ~106K pixels
  different from cluster c0 reference but visually nearly identical
* `phase-A-on/sample-0d9d8b47.png` — cluster c3, ~148K pixels diff
* `mosaic_4way.png` — side-by-side: baseline + 3 phase-A samples
* `zoom_mosaic_4way.png` — central crop of same 4
* 100× `sample-*.bin/png` pairs in `phase-A-on/`

Branch: `tenderloin/6.18/upstream-patches` tip
`174a7b70af96dc72af719002370bbdc4da0242ba`.
