# Update 15 for Gemini: shallow-suspend confirmed at the GDSC level

## TL;DR

You were right about shallow suspend. The smoking gun is in
`/sys/kernel/debug/pm_genpd/pm_genpd_summary`:

```
domain                  status     /device          runtime status   managed by
gfx3d                   on
    4300000.adreno         suspended                                  SW
```

The adreno device is software-`suspended`, all GPU clocks gated off
(clk_summary shows `gfx3d_*_clk` at enable=0/prepare=0/rate=0), but the
**`gfx3d` power domain (GDSC) remains `on`**. The silicon stays
powered. SRAM keeps its electrons. The 8-cycle survives.

Across multi-second confirmed-suspended periods, the cycle counter
advances linearly — `ABCD EFGH ABCD EFGH ABCD` — a perfect
deterministic period-8 across 5-second clocks-off windows.

## Update 14 correction

I reported "wakelock confirmed, GPU never suspends." That was wrong —
I was monitoring `/sys/class/drm/card0/device/power/runtime_status`
which is the **MDP display controller** (`5100000.mdp`), not the
GPU. Once corrected to
`/sys/devices/platform/soc/4300000.adreno/power/runtime_status`:

* GPU IS suspending properly.
* Cumulative time at uptime ~17min: ~5min active, ~12min suspended.
* During tight-loop 100-cap: 45% suspended / 53% active (sample
  cadence 25 ms).
* During sleep-1s 100-cap: 79% suspended.
* In all cases, **cycle remains 8 unique 12-13 each, including the
  textbook ABCDEFGH pattern**.

So pm_runtime is doing what it's supposed to. The bug is one layer
deeper: the suspend isn't deep enough.

## The autosuspend=0 + 5-second-sleep test

Per your "Force-0 + 30-second sleep" suggestion, ran a faster
variant first (`autosuspend_delay_ms=0` + 5-second sleep between
each of 20 frames):

```
=== test 2: 5s sleep between runs (deep idle) ===
post-sleep check: suspended=20 active=0     <- GPU confirmed suspended after each
freq:
  3 c399f1f4
  3 7b6dc2d0
  3 5d1220b0
  3 10fbbed0
  2 fb12cd4c
  2 c1bf109e
  2 5adc3160
  2 50baa6c2
cycle (4 chars per line):
ABCD
EFGH
ABCD
EFGH
ABCD
```

Perfect ABCDEFGH continuation across 5-second confirmed-suspended
gaps. The cycle counter doesn't reset.

We can run the 30-second sleep version next, but given the
genpd_summary finding, the prediction is: same result, because the
GDSC isn't dropping in either case.

## DT confirms the supply wiring

```
gpu: adreno@4300000 {
    compatible = "qcom,adreno-220.0", "qcom,adreno";
    ...
    power-domains = <&mmcc GFX3D_GDSC>;
    power-domain-names = "gfx3d";

    /* GPU voltage supply (vdd_dig) for DVFS. */
    gpu-supply = <&pm8058_s1>;
    ...
};
```

Both the GDSC and the regulator are wired. The regulator (`pm8058_s1`,
500-1350mV range) IS dropping its vote when the GPU suspends:

```
regulator_summary:
   s1   0   0   0   unknown   0mV    0mA    500mV   1350mV
       (no consumers active)
```

So the rail vote IS being released. The thing that *isn't* getting
to a "collapsed" state is the GDSC itself — it stays `on`.

(Mainline drivers that fully power-collapse, like A6xx, drive the
GDSC via the genpd framework into an actually-off state during
suspend. A2xx looks like it's only enabling/disabling clocks
through the GDSC rather than triggering its `*_PWR_DWN_*` writes.)

## Other RBBM mask data points (for completeness)

A/B'd three RBBM_STATUS poll masks back-to-back (same boot, no reset
between them):

| Mask                    | Bits                                        | unique | top freq    |
|-------------------------|---------------------------------------------|--------|-------------|
| `0x80000000`            | GUI_ACTIVE only                             | 8      | 13/13/13/13/12/12/12/12 |
| `0x580D0000` (default)  | CP_NRT \| MH \| MH_COH \| SQ_CNTX0/17 \| RB_CNTX | 8      | identical               |
| `0x00000000`            | Poll fully disabled                         | 8      | identical               |

Same hashes, same distribution. As discussed in update 14: cycle is
not pipeline-busy in any RBBM-tracked sense.

## Actionable hypotheses for the GDSC fix

### A. genpd flag on GFX3D
If the gfx3d genpd has `GENPD_FLAG_ALWAYS_ON` (or equivalent
upstream flag preventing collapse), removing it would let the
domain power down when no consumers are active. The other "on"
domains in the same dump (vpe, vfe, ved, rot, gfx2d0/1, ijpeg)
suggest this might be set across the board for this kernel.

### B. GDSC retention bits
On QCOM GDSCs, flags like `RETAIN_FF_ENABLE`,
`POLL_CFG_GDSCR`, or `HW_CTRL` configure how the domain
transitions. Some retain SRAM/flip-flop state during power-down
specifically to make subsequent power-on faster — at the cost of
not actually clearing the SRAM. If the gfx3d GDSC has a retention
flag, it would explain "GDSC reports off but SRAM keeps state",
or alternatively "GDSC never goes off because retention path is
mis-configured".

### C. Interconnect / sibling consumer
Something on the gfx3d power domain that we don't know about (e.g.
the gpu_iommu) might be holding a vote indirectly. The IOMMU
device for the GPU is `gpu_iommu`; if it has its own pm_runtime
that doesn't track the gpu's, it could keep the parent on.

### D. mainline a2xx driver doesn't drop the domain on suspend
The driver might have a missing `pm_runtime_put` for the genpd
parent, or a missing `dev_pm_domain_set` call. If the device
suspends but the genpd ref isn't dropped, the parent stays on.

## What we'd like input on

1. **Fastest path to verify "GDSC must actually drop"?**
   * Force-disable the gfx3d GDSC via debugfs and see if subsequent
     resume produces a fresh hash set?
   * Add a kernel patch that calls `pm_genpd_remove_device` and
     re-adds it on each submit to forcefully cycle the domain?
   * Custom: in `a2xx_pm_runtime_suspend`, explicitly call into
     the gdsc driver to force `*_PWR_DWN`?

2. **Is GFX3D_GDSC's retention flag the actual cause?**
   On `drivers/clk/qcom/mmcc-msm8660.c` (or similar), the gfx3d
   GDSC definition presumably has flags. If `RETAIN_FF_ENABLE` is
   set, the domain power-collapses but hardware retains SRAM. If
   it's *not* set but the domain still doesn't show as "off" in
   genpd_summary, then something else is keeping it on. Either way
   it points at a fix location.

3. **Cold-cold boot prediction holds?**
   Given GDSC stays on across `sysrq b`, the cycle's counter would
   indeed survive a warm boot (which is what we see). A power-button
   long-press should drop the rail entirely. Still pending physical
   access.

4. **Mesa-side "heavy scrub" as the fallback**
   If GDSC retention is intrinsic and we can't fix it in the kernel,
   your suggestion of an aggressive Mesa-side preamble that explicitly
   writes 8 wavefronts of safe-zero state to scrub the SQ slots is
   the shippable path. We already have an `a2xx_emit_sanitizer_preamble`
   path in the kernel (cross-context shadow-state restore). Would the
   right move be to extend it with explicit SQ-slot scrubbing dummy
   draws, or do this in Mesa? You suggested Mesa-side; what specifically
   should the dummy draw look like to force GPR/varying writes across
   all 8 slots?

## Tip / SHA reference

Branch `tenderloin/6.18/upstream-patches`, tip `76b56fe33fab`:

* `a2xx_rbbm_poll_enable` (bool) and `a2xx_rbbm_poll_mask` (uint)
  — the tunable RBBM poll
* `a2xx_wptr_poll_enable` (bool) — KGSL-style WPTR-polling, KNOWN
  BROKEN at ME_INIT (ring drain timeout)
* `a2xx_wptr_poll_delay` (uint) — alongside above
* `a2xx_debug_wptr_delay` (uint) — pre-existing udelay knob
