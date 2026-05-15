# Footswitch / GDSC suspend handling on MSM8660 — investigation notes

## TL;DR

The infrastructure for actually power-collapsing the gfx3d GDSC and
clearing GPU SRAM is **already in place**. The legacy footswitch
disable path (`gdsc_disable` for `LEGACY_FOOTSWITCH | SW_RESET` flag
combo) is *exactly* what we need: it asserts AHB reset, clamps I/O,
and collapses the power rail. If genpd called it, we'd get a clean
SRAM scrub on every idle.

The blocker is that **genpd never calls it.** All 9 MMCC genpds on
this device report `total_idle_time = 0 ms` after >38 min uptime.

This is the same architectural hole we hit on the camera side
(VFE31, MT9M113 sensor MCU) — mainline assumes runtime PM hands
power-cycling to the genpd framework, but for legacy MSM8660
footswitches the framework is registered with `gov=NULL` and never
actually issues the off-transition.

## What `gdsc_disable` does for legacy footswitches

From `drivers/clk/qcom/gdsc.c:419`:

```c
if (sc->flags & LEGACY_FOOTSWITCH) {
    if (sc->flags & SW_RESET)
        gdsc_assert_reset(sc);             /* AHB reset asserted */

    legacy_fs_assert_clamp(sc);            /* I/O clamps applied */

    ret = gdsc_update_collapse_bit(sc, true); /* rail collapsed */
    if (ret)
        return ret;

    return 0;
}
```

For our `gfx3d_gdsc`:
- `flags = LEGACY_FOOTSWITCH | SW_RESET`
- `pwrsts = PWRSTS_OFF_ON` (full off allowed)
- `resets = { GFX3D_AHB_RESET }` — this is what gets asserted

So a single `gdsc_disable` call would: (1) assert the GFX3D AHB
reset, (2) clamp I/O, (3) collapse the rail. SQ slot SRAM gets
electrically wiped. On `gdsc_enable`, the rail comes back, reset is
deasserted, I/O unclamps. ~7 µs total enable time.

This matches what KGSL did via `internal_pwr_rail_ctl` —
fundamentally the same hardware sequence, just driven through SCM
secure-monitor rather than direct register writes.

## Why genpd never calls it

`gdsc_init` line 615:
```c
ret = pm_genpd_init(&sc->pd, NULL, !on);
                       /* ^^^ no governor */
```

All QCOM GDSCs register with `gov=NULL`. This is a deliberate choice
in the QCOM driver: governors that aggressively idle the domain were
problematic for some platforms (latency on resume during high-rate
UI work). But for our case it's a footgun — the legacy MSM8660
GDSCs have a clean off-path that we *want* to take on every runtime
suspend.

Empirical confirmation:
```
gfx2d0:  active=2288662 ms idle=0 ms
gfx2d1:  active=2288710 ms idle=0 ms
gfx3d:   active=2288758 ms idle=0 ms
ijpeg:   active=2288808 ms idle=0 ms
mdp:     active=2288863 ms idle=0 ms
rot:     active=2288919 ms idle=0 ms
ved:     active=2288975 ms idle=0 ms
vfe:     active=2289029 ms idle=0 ms
vpe:     active=2289097 ms idle=0 ms
```

All 9 domains have idle=0. Not a single off-transition, ever.

## Parallel with the camera issue

This is structurally identical to what we hit on VFE31:

* Camera path: VFE/CSI/CAMSS sub-blocks need explicit reset between
  format changes. Mainline assumed pm_runtime would handle it via
  the `vfe` genpd, but the genpd never collapses for the same
  `gov=NULL` reason. Symptom: MT9M113 sensor MCU lockup after
  sequential format changes (ZSL/RDI/preview), recoverable only by
  full sensor power cycle. We added explicit `pm_runtime_force_*`
  recovery in the driver.

* GPU path: SQ wavefront SRAM needs explicit reset between contexts.
  Mainline assumed pm_runtime would handle it via `gfx3d` genpd,
  same `gov=NULL` problem. Symptom: deterministic period-8 cycle of
  rendering nondeterminism.

The fix shape is the same: drive the footswitch off-transition
explicitly from the device driver, rather than waiting for the
generic genpd framework.

## Three concrete kernel-side options

### A. Add a per-GDSC opt-in flag `COLLAPSE_ON_IDLE`

Smallest blast radius. New flag in `drivers/clk/qcom/gdsc.h`. Set
on `gfx3d_gdsc`, `vfe_gdsc`, etc. — only the GDSCs we know are safe
to collapse on idle. In `gdsc_init`, when this flag is set, register
with a real governor (`simple_qos_governor` or a custom one). Other
GDSCs keep current `gov=NULL` behaviour.

Risk: governor selection is a global thing for the genpd subsystem;
custom per-domain governor support exists but adds complexity.

### B. Driver-side `pm_runtime_force_*` from a2xx

Don't touch the GDSC driver. From `a2xx_pm_runtime_suspend`,
explicitly trigger the genpd off-transition for the gfx3d domain.
Probably via `dev_pm_genpd_suspend(dev)` followed by
`pm_genpd_remove_device + add_device` or equivalent forced
collapse. Targeted to a2xx; doesn't affect other QCOM GDSCs.

Risk: layering violation (the GPU driver reaching into the clock
controller's genpd internals). But matches the camera
`pm_runtime_force_*` precedent.

### C. Debug knob first

Add a sysfs/debugfs knob in our existing a2xx module that, on
write, calls `gdsc_disable` directly via the genpd's `power_off`
callback. Use this to **verify the cycle collapses** when GFX3D is
actually power-collapsed, before committing to a real fix.

This is the cheapest validation step. If toggling the knob between
submits makes the cycle vanish, we have proof the SRAM theory is
right and either (A) or (B) is the engineering answer.

## What I'd recommend

1. Wait for the in-flight Mesa rebuild + 100-cap test of patch 0070
   (Gemini's 8-draw SQ scrub). If that collapses the cycle: fix is
   shippable as Mesa-only, no kernel surgery needed.
2. If 0070 doesn't collapse the cycle: option (C) above as the
   fastest validation. We add a debug knob, prove the theory, then
   commit to (A) or (B) for the actual fix.
3. The camera precedent suggests (B) is more practical — genpd
   subsystem changes are upstream-resistant for this kind of "this
   one machine needs special power management" pattern. A targeted
   driver-side pm_runtime sequence is what got merged for VFE31
   recovery, and the GPU side is structurally similar.

## Open question for Gemini (pending update)

If we go with (B), what's the right kernel API call to "force the
parent genpd to power-off-then-on" from a child driver's
pm_runtime_suspend / resume callback? The naive
`pm_genpd_remove_device + add_device` cycle smells wrong; there's
likely a cleaner primitive (`dev_pm_genpd_resume` /
`dev_pm_genpd_suspend`?). Also: is doing this on every
runtime_suspend/resume safe given the ~7 µs GDSC enable cost?

## Appendix: relevant code references

* `drivers/clk/qcom/gdsc.c:419-432` — legacy footswitch disable
  path (asserts SW_RESET, clamps, collapses rail)
* `drivers/clk/qcom/gdsc.c:326-353` — legacy footswitch enable path
  (uncollapses rail, deasserts reset, ~7µs total)
* `drivers/clk/qcom/gdsc.c:615` — `pm_genpd_init(&sc->pd, NULL, !on)`
  the gov=NULL registration that prevents idle transitions
* `drivers/clk/qcom/mmcc-msm8660.c::gfx3d_gdsc` — defines our GFX3D
  domain with `LEGACY_FOOTSWITCH | SW_RESET` flags, `PWRSTS_OFF_ON`
* `/sys/kernel/debug/pm_genpd/gfx3d/idle_states` — `0 ms`,
  empirical confirmation no transitions ever happen
* webOS KGSL `kgsl_pwrctrl.c::kgsl_pwrctrl_pwrrail` — calls
  `internal_pwr_rail_ctl` which goes through SCM to do the same
  hardware sequence
