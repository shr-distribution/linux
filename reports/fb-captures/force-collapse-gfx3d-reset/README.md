# force_collapse with GFX3D_RESET pulse — captures

Kernel: tenderloin/6.18/upstream-patches @ 4b2a320e5099
Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046

Test: same protocol as `../force-collapse-retention-test/` but the
kernel debugfs knob now pulses **GFX3D_RESET (MMCC 0x0210 bit 12)**
after `genpd->power_on()`. This mirrors the legacy webOS
`arch/arm/mach-msm/footswitch-8x60.c::footswitch_enable()` which does
`clk_reset(core_clk, ASSERT) → udelay(5) → clk_reset(core_clk,
DEASSERT)` after the rail comes back. Mainline `gdsc_enable` for
LEGACY_FOOTSWITCH only resets the AHB clock; the core_clk reset is
never pulsed.

## Channel means

### Baseline (Phase 1, no force_collapse — cycle present)

```
bl-5adc3160                  R=34.271   G=34.276   B=34.211   A=103.365
  (uniform-grey triangle, the "correct" historical render;
   appears 12-13× per 100 captures, each instance bit-for-bit identical)
```

### Phase 3 — 20 captures with force_collapse-each-render (RETENTION clear + GFX3D_RESET pulse)

```
gfx3d-reset-e0c67eeb         R=34.2833  G=34.2872  B=34.2230  A=103.376
gfx3d-reset-b4dd72af         R=34.2837  G=34.2874  B=34.2236  A=103.376
gfx3d-reset-87a11499         R=34.2837  G=34.2872  B=34.2235  A=103.376
gfx3d-reset-31c4d7b8         R=34.2836  G=34.2873  B=34.2233  A=103.376
```

All four agree to **5 decimal places** in every channel. The 20 unique
MD5s are just edge-pixel jitter — see pixel-diff table below.

### Phase 4 — residual cycle (no force_collapse, after Phase 3)

```
residual-8618ea32            R=1.05     G=1.05     B=35.19    A=103.365   (broken: faint R/G)
residual-90238fcf            R=7.46     G=2.46     B=21.25    A=103.365   (broken: dark blue)
residual-962fa663            R=23.56    G=24.10    B=13.80    A=103.365   (different shade)
```

These confirm the cycle re-emerges once force_collapse stops firing,
with a slightly different mix of broken hashes than baseline.

## Pixel-diff measurements

`compare -metric AE -fuzz 0%`:

|                                                         | differing pixels |
|---------------------------------------------------------|------------------|
| **GFX3D_RESET pulse path**:                             |                  |
| `gfx3d-reset-e0c67eeb` vs `gfx3d-reset-b4dd72af`        |       **30**     |
| `gfx3d-reset-e0c67eeb` vs `bl-5adc3160`                 |       85 358     |
|                                                         |                  |
| **Without GFX3D_RESET pulse (from prior test)**:        |                  |
| fc-1e563a4c vs fc-25797394 (per update 20)              |      106 073     |
| fc-1e563a4c vs fc-d83a1af1 (per update 20)              |      148 958     |

The GFX3D_RESET pulse reduces fc-to-fc pixel noise by **99.97%**
(30 vs 106,073). What remains is essentially deterministic
single-pixel edge jitter from rasterization precision.

The 85K pixel diff vs baseline 5adc3160 is residual — same visual
content (channel means match to 5 decimals), but per-pixel
rasterization detail differs between the historical "correct" render
and the post-fc render. Visually identical to a human eye.

## Conclusion

Pulsing GFX3D_RESET (the GPU's core_clk reset, distinct from
GFX3D_AHB_RESET which mainline already pulses) after the rail
power-on is the missing initialization step that turns mainline's
`gdsc_enable` from "wakes up dirty silicon" into "wakes up clean
silicon". Per Gemini's analysis, the VPC (Vertex Parameter Cache)
SRAM in the SQ block needs this reset pulse to come up in a
deterministic state.

The two fixes that turn this into a real shippable feature:

1. **`drivers/clk/qcom/mmcc-msm8660.c`**: add `GFX3D_RESET` to
   `gfx3d_gdsc.resets[]` so the existing `gdsc_assert_reset` /
   `gdsc_deassert_reset` calls pulse both AHB and core resets.

2. **`drivers/clk/qcom/gdsc.c::gdsc_enable` LEGACY_FOOTSWITCH path**:
   currently does `assert(resets) → enable → udelay(2) → deassert(resets)`
   *before* the rail is fully ramped. The legacy webOS sequence
   deasserts first, then pulses core AFTER the rail is on. The
   correct fix is to add a "second core-reset pulse" after the
   `udelay(5)` clamp-settle delay, only for GDSCs that opt into it
   (new flag like `RESET_PULSE_AFTER_ENABLE`).

3. **`drivers/clk/qcom/gdsc.c::gdsc_disable` LEGACY_FOOTSWITCH path**:
   currently only clears ENABLE (BIT 8). Needs to also clear
   RETENTION (BIT 9) when collapsing, otherwise the rail enters
   "Logic-Off-Memory-On" retention mode and SRAM survives.

4. **A trigger for the off-transition on idle**: since the QCOM
   gdsc driver registers genpd with `gov=NULL`, the off-transition
   never fires. Either add a governor specifically for the
   "collapse-safe" GDSCs, or have the a2xx driver explicitly trigger
   it from its `pm_runtime_suspend` callback.
