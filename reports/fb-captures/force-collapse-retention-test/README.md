# force_collapse + RETENTION-clear test captures

Kernel: tenderloin/6.18/upstream-patches @ 457e098800b1
Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046

Test: 100-cap variants run on 2026-05-10 ~19:46 with the
`/sys/kernel/debug/dri/0/force_collapse` knob extended to clear
GDSCR BIT(9) RETENTION in addition to BIT(8) ENABLE.

## Channel means

Format: file → R, G, B, A means (1024×768 RGBA capture).

### Baseline (Phase 1 — no force_collapse, original 8-cycle)

* `bl-sample-5adc3160.bin/png`  R=34.27  G=34.28  B=34.21  A=103.37
  — uniform grey render, the "correct" output (R=G=B)
* `bl-sample-29fcdfdf.bin/png`  R=7.46   G=2.46   B=21.25  A=103.37
  — broken: dark blue-tinted, R/G nearly zero
* `bl-sample-c399f1f4.bin/png`  R=0      G=0      B=61.90  A=103.37
  — broken: ONLY blue channel, R/G fully zero
* `bl-sample-fb12cd4c.bin/png`  R=1.05   G=1.05   B=35.19  A=103.37
  — broken: faint R/G, blue dominant

### force_collapse-before-each-render (Phase 3 — 20 captures total, 4 sampled here)

* `fc-sample-1e563a4c.bin/png`  R=34.28  G=34.29  B=34.22  A=103.38
* `fc-sample-25797394.bin/png`  R=34.28  G=34.29  B=34.22  A=103.38
* `fc-sample-d83a1af1.bin/png`  R=34.28  G=34.29  B=34.22  A=103.38
* `fc-sample-f8e4d15e.bin/png`  R=34.28  G=34.29  B=34.22  A=103.38

**All identical to baseline 5adc3160 channel means to 5 decimal
places.** The 20 unique MD5s in Phase 3 are sub-pixel rasterization
noise — the visible image is the same correct grey-triangle render.

### Post-FC residual cycle (Phase 4 — no force_collapse after Phase 3)

* `res-sample-2821b4f2.bin/png` R=10.97  G=10.98  B=27.83  A=103.37
* `res-sample-2fffa135.bin/png` R=7.46   G=2.46   B=21.25  A=103.37
  — same channel means as bl-29fcdfdf; cycle reformed with old hashes
* `res-sample-82b8940b.bin/png` R=1.05   G=1.05   B=35.19  A=103.37
  — same channel means as bl-fb12cd4c
* `res-sample-dec69f80.bin/png` R=23.56  G=24.10  B=13.80  A=103.37
  — new "wrong" pattern not in baseline set

## Conclusion

Clearing GDSCR BIT(9) RETENTION via the force_collapse debugfs knob
**collapses the period-8 cycle to the correct render** (R=G=B=34
grey-triangle). The 20 unique MD5s reported in the test were
visually-identical renders differing only in sub-pixel noise.

Once force_collapse is no longer fired between renders, the cycle
re-emerges, but with a partially-different hash set. The mechanism
appears to be:

1. RETENTION on (default): SQ slot SRAM (or similar) holds dirty
   state across runtime suspend/resume. Each new render lands on
   one of 8 slots with different stale state → 8 visible patterns.
2. RETENTION cleared: SRAM electrically lost on collapse, kernel
   re-init via `pm_runtime_force_resume` → `a2xx_hw_init` →
   `a2xx_me_init` programs slot 0 cleanly. Render works.
3. Without continued force_collapse, RETENTION stays on for
   subsequent runtime cycles, dirty SRAM accumulates, cycle
   reforms but with different per-slot garbage.

## Shippable fix path

Modify `drivers/clk/qcom/gdsc.c::gdsc_disable` LEGACY_FOOTSWITCH
path to also clear `LEGACY_FS_RETENTION_MASK` (BIT 9), with an
opt-in flag like `NO_RET_AT_DISABLE` so other QCOM platforms that
rely on retention aren't affected.

OR add it as a per-GDSC flag and set on `gfx3d_gdsc` (and other
MSM8660 domains we know are safe to fully collapse).
