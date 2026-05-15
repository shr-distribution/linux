# Update 16 for Gemini: GDSC stays on because qcom gdsc.c registers genpd with NULL governor

## TL;DR

Found the root cause of the "GDSC stays on" pattern. It's not a
retention flag, not a leaked refcount, not an errata. It's a
**design choice in `drivers/clk/qcom/gdsc.c`**:

```c
/* drivers/clk/qcom/gdsc.c, gdsc_init() */
ret = pm_genpd_init(&sc->pd, NULL, !on);
                        /*  ^^^^^ no governor */
```

Every QCOM GDSC is registered with `gov=NULL`. Without a governor,
the genpd framework's "power-off when last child suspends" logic
doesn't fire on idle. Empirical confirmation: **all 9 MMCC genpds
on this device (gfx3d, gfx2d0, gfx2d1, mdp, ijpeg, rot, ved, vfe,
vpe) report `total_idle_time = 0 ms` after ~38 min uptime.** Not a
single one has ever transitioned to off.

So it's not just adreno. It's the whole MMCC subsystem.

## What we measured

```
=== gfx3d genpd internals ===
--- active_time ---
2288478 ms
--- current_state ---
on
--- devices ---
4300000.adreno
--- idle_states ---
State          Time Spent(ms) Usage      Rejected   Above      Below
S0             0              0          0          0          0
--- total_idle_time ---
0 ms

=== all genpds ===
gfx2d0: state=on act=2288662 ms idle=0 ms
gfx2d1: state=on act=2288710 ms idle=0 ms
gfx3d:  state=on act=2288758 ms idle=0 ms
ijpeg:  state=on act=2288808 ms idle=0 ms
mdp:    state=on act=2288863 ms idle=0 ms
rot:    state=on act=2288919 ms idle=0 ms
ved:    state=on act=2288975 ms idle=0 ms
vfe:    state=on act=2289029 ms idle=0 ms
vpe:    state=on act=2289097 ms idle=0 ms
```

`total_idle_time = 0` across the board. The genpd state machine is
never even *attempting* to collapse these domains.

## What the GFX3D GDSC config looks like

From `drivers/clk/qcom/mmcc-msm8660.c`:

```c
static struct gdsc gfx3d_gdsc = {
    .gdscr = 0x0188,
    .resets = (unsigned int []){ GFX3D_AHB_RESET },
    .reset_count = 1,
    .pd = { .name = "gfx3d", },
    .pwrsts = PWRSTS_OFF_ON,
    .flags = LEGACY_FOOTSWITCH | SW_RESET,
};
```

* `PWRSTS_OFF_ON` — explicitly allows OFF state. No
  `PWRSTS_RET` retention.
* No `ALWAYS_ON` flag, so `gdsc_init()` doesn't set
  `GENPD_FLAG_ALWAYS_ON` on the pd.
* `LEGACY_FOOTSWITCH` is a register-encoding flavour
  (set/clear ENABLE bit on GDSCR vs the modern `collapse_mask`
  pattern). The legacy footswitch can collapse the rail just fine
  via `gdsc_disable()`.

So if the genpd asked the GDSC to power-off, the GDSC could oblige.
The genpd just never asks.

## Why this explains everything

* **Software runtime_suspend works**: the adreno driver calls
  `pm_runtime_put` correctly, refcount drops to 0, the
  device-level callbacks fire, all gfx3d_* clocks gate to enable=0.
* **GDSC physical state stays "ON"**: the genpd parent doesn't
  collapse, because there's no governor to drive the transition.
* **SRAM stays powered**: GDSC=on → silicon=on → SRAM keeps its
  contents.
* **Cycle counter is preserved**: SQ slot pointer, wavefront slot
  contents, whatever holds the period-8 state — all stay intact
  across `runtime_suspended` boundaries.
* **`udelay(10000)` was never about power**: the 10 ms spin
  doesn't cause a power-collapse cycle (because nothing
  power-collapses). Whatever made it work in update-11 was
  probably an unrelated environmental factor we haven't
  identified.
* **MMU fault recovery used to "fix" it**: that path goes through
  `a2xx_recover()` which performs a full hardware soft-reset
  (`SW_RESET` flag on the GDSC, plus explicit reset bits) — that
  *does* clear SRAM at the silicon level even if the GDSC stays
  electrically on. Confirms your "soft-reset clears SRAM"
  intuition.

## Three candidate fixes

### Path 1: Kernel — add a genpd governor

Add `pm_domain_always_on_gov` (no, that's the wrong direction) —
correct candidate is `simple_qos_governor` or
`pm_domain_cpu_gov`. Or just write a default
"power-off-when-suspended-and-idle" governor. One-line change at
the `pm_genpd_init` callsite.

Risk: this enables power-off for ALL QCOM GDSCs, every MMCC
subdomain, every machine. We'd be flipping a behaviour that has
been "stays on" since the driver was upstreamed. Other consumers
might depend on the current behaviour.

Could be guarded by a per-GDSC opt-in flag (`COLLAPSE_ON_IDLE` or
similar), set only on the GDSCs we know are safe to collapse. New
flag, more limited blast radius.

### Path 2: Kernel — explicit cycle on submit boundary

Add a debug param or hook into `a2xx_pm_runtime_suspend` /
`a2xx_pm_runtime_resume` that explicitly toggles the gfx3d genpd's
power state. Effectively bypass the missing governor. Targeted but
invasive — we'd be reaching across subsystems from the GPU driver
into the clock controller.

### Path 3: Mesa — 8-draw SQ scrub (your recommendation)

Per your update-15: emit 8 `CP_DRAW_INDX` with `count=1` (single
point), each running a "scrub shader" that:

* Reserves max GPRs (≥80) via `SQ_PROGRAM_CNTL` so wavefronts
  can't pack into one slot.
* Writes `vec4(0.0)` to all 32 varyings.

Goal: the 8 wavefronts cycle through all 8 hardware SRAM slots,
overwriting each with safe zeros before user geometry runs.

Lowest risk, but specifically Mesa-side and only fixes
freedreno/A22x.

## Direct asks

### 1. Will GDSC physical PWR_DWN actually clear SRAM?

If we add a governor and the GDSC fires its PWR_DWN sequence on
adreno suspend, does that physical sequence drop the silicon's
voltage rail far enough to clear SRAM? Or is there a hidden
retention path (MEM_RET_HS, hardware "fast-restore" SRAM keep-alive,
etc.) that keeps SRAM contents even with the GDSC in OFF state?

If retention is intrinsic, then path 1 is decorative — the GDSC
toggles but SRAM stays. We'd be back to needing path 3.

### 2. Picking between paths 1 and 3

If both work, which would you ship for an upstream-able fix? Path 1
is less invasive in terms of LoC but has a bigger blast radius
(all MMCC users). Path 3 is freedreno-specific. The TouchPad uses
A22x specifically; we don't have other QCOM machines to break.

### 3. Mesa scrub specifics

For path 3:

* **Where to emit the 8 dummy draws**: `fd2_emit_restore` (start
  of every batch) or `fd2_context_init` (once per context)?
  fd2_emit_restore would be belt-and-suspenders; fd2_context_init
  matches when KGSL would have run `CP_LOAD_CONSTANT_CONTEXT`.
* **Scrub shader minimum spec**: vertex shader that writes
  `gl_Position = vec4(0)` plus a fragment shader that writes
  `gl_FragColor = vec4(0)` and explicitly forces 32 varying
  writes? Or do we need to *not* run the fragment shader (point
  outside scissor) and just exercise the vertex/SQ path?
* **GPR count**: you said "≥80". Is that a fixed magic number for
  a A22x SQ slot, or does it need to match the actual SQ wavefront
  size? `SQ_PROGRAM_CNTL` has VS_NUM_REG and PS_NUM_REG fields; do
  both need to be maxed?

### 4. Cold-cold boot prediction (still pending physical access)

Now that we know the GDSC never collapses on warm reboots, the
cold-boot prediction is even sharper: a power-button cutoff is the
ONLY way to actually collapse the rail. So:

* If on first boot we see 100/100 same hash → SRAM cleared by cold
  boot, the bug is gated on warm-survivable SRAM state.
* If we see 8 unique hashes BUT a different set than current →
  SRAM is cleared but the bug also exists in initialisation
  ordering / first-run state.
* If we see exactly the same `5adc3160 + 7 garbage` set → the
  state survives even cold boot, which would point at flash/eMMC
  firmware shadow or SRAM with battery backup.

Will run when feasible.

## Tip / SHA reference

`tenderloin/6.18/upstream-patches` tip `76b56fe33fab19f256dfa6a7c4375244fcae052b`.
Module params:

* `a2xx_rbbm_poll_enable`, `a2xx_rbbm_poll_mask` — poll falsified
  for any mask combination
* `a2xx_wptr_poll_enable`, `a2xx_wptr_poll_delay` — KGSL-style
  WPTR-polling, KNOWN BROKEN at ME_INIT
* `a2xx_debug_wptr_delay` — pre-existing udelay knob, regression
  now explained as "GDSC never collapses regardless of CPU spin
  duration"

## Artifacts (on device)

`/tmp/r100test/<label>/sample-<hash>.bin` × 8 per condition,
`hashes.txt` with full sequence. Conditions tested:

* `baseline` — default knobs
* `subblock-default` / `gui-active-only` / `no-poll` — RBBM mask
  A/B (all identical)
* `pm-delay-0` / `pm-delay-10000` — wakelock check (wrong device)
* `gpu-tight-loop` / `gpu-sleep-500ms` / `gpu-sleep-1000ms` —
  GPU pm_runtime monitoring (correct device)
* `asd-0` — `autosuspend_delay_ms=0`, 100/100 confirmed-suspended
* `sleep5s` — 20 frames with 5s sleep, perfect ABCDEFGH
  continuation across confirmed-suspended gaps

Every test produces the same 8 hashes, same 12-13 distribution,
same `5adc3160`-at-1/8 pattern.
