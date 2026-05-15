# Update 14 for Gemini: wakelock theory confirmed, RBBM mask falsified for all bit combos

## TL;DR

Two clean results from this session:

1. **Your wakelock-regression theory is right.** During a 100-cap with
   `wptr_delay=10000`, `runtime_status` was sampled every 50 ms (~600
   samples per run) and read **`active` 100 % of the time, with one
   transition (initial entry) per run**. The GPU never suspends. The
   10 ms udelay executes but doesn't power-collapse anything, so the
   toxic SRAM state Gemini hypothesised survives unchanged. Cycle
   persists.

2. **The "real idle" RBBM mask is also decorative.** A/B'd three poll
   masks across 100-cap runs:

   | Mask                                         | Distribution                |
   |----------------------------------------------|-----------------------------|
   | `0x80000000` (legacy GUI_ACTIVE)             | 8 unique, 12-13 each        |
   | `0x580D0000` (CP_NRT \| MH \| MH_COH \| SQ \| RB_CNTX) | 8 unique, 12-13 each        |
   | `0x00000000` (poll fully disabled)           | 8 unique, 12-13 each        |

   Same 8 hashes, same distribution, regardless of mask. Whatever
   the cycle's mechanism is, it's *upstream* or *orthogonal* to any
   bit RBBM_STATUS exposes.

## Methodology this round

We're now using the existing `scripts/auto-test.sh` pattern: 100 runs,
sample-per-unique-hash (8 sample bins ≈ 24 MB instead of 100 caps ≈
300 MB), `hashes.txt` with the full sequence. Artifacts at
`/tmp/r100test/<label>/` on device.

For the PM monitor: a background `while true` loop reading
`/sys/class/drm/card0/device/power/runtime_status` every 50 ms,
written with timestamp into `pm_status.txt`. Run alongside the
100-cap.

## RBBM mask falsification

Pre-WPTR poll loop now uses a tunable mask
(`a2xx_rbbm_poll_mask`, default = the union you suggested):
```c
while (gpu_read(REG_A2XX_RBBM_STATUS) & a2xx_rbbm_poll_mask)
    cpu_relax();
```

Three back-to-back runs, no reboot between:

```
=== subblock-default: mask=0x580d0000 rbbm_en=Y wptr_delay=0 ===
unique: 8 / MMU faults: 0 / hangchecks: 0
freq:
   13 fb12cd4c
   13 c1bf109e
   13 5adc3160
   13 50baa6c2
   12 c399f1f4
   12 7b6dc2d0
   12 5d1220b0
   12 10fbbed0
cycle: ABCDEFGH ABCDEFGH ABCDEFGH

=== gui-active-only: mask=0x80000000 ===
freq: same 8 hashes, 12-13 each
cycle: ABCDEFGH ABCDEFGH ABCDEFGH

=== no-poll: mask=0x0 ===
freq: same 8 hashes, 12-13 each
cycle: ABCDEFGH ABCDEFGH ABCDEFGH
```

Notes:
* No MMU faults, no hangchecks, GPU healthy throughout — these are
  *clean* baselines, not stuck-buffer artifacts.
* Per-run distribution is deterministic; the hashes simply rotate
  through their slots.

The conclusion is uncomfortable but clear: **the cycle is not
pipeline-busy in any sense RBBM_STATUS exposes**. It's not CP, not
MH, not SQ, not RB, not even GUI. So either:
* the relevant busy state is in a sub-block that doesn't have a
  status bit (something inside the SQ that the wavefront scheduler
  exposes only through internal counters), or
* the mechanism is *not* "wait for previous batch to finish" at all
  — the previous batch HAS finished by the time we submit, and the
  cycle counter advances per-submission regardless.

If your theory of "uninitialised SRAM crossed with PM races" holds,
this is consistent: the SRAM in question doesn't drive any RBBM
busy bit, it just *colours* the next batch's output.

## Wakelock confirmation

```
=== pm-delay-0: wptr_delay=0 ===
freq: 8 unique, 12-13 each (same 8 hashes)
PM samples: 605 entries
PM status distribution:
   605 active
transitions: 1 (initial entry only)

=== pm-delay-10000: wptr_delay=10000 ===
freq: 8 unique, 12-13 each (same 8 hashes)
PM samples: 593 entries
PM status distribution:
   593 active
transitions: 1 (initial entry only)
```

`runtime_status` never deviates from `active`. Confirmed: the 10 ms
spin no longer triggers an autosuspend cycle, even though the spin
itself is being executed (we've instrumented it; runs from a CPU
in the kernel's `udelay` path).

`autosuspend_delay_ms` reads as empty (just blank — investigating).
`control=auto` is set correctly.

This is a **clean reproduction of your update-13 wakelock theory**.

## Per-boot hash drift

Worth noting separately because it might be diagnostic:

* **Today's earlier baseline (boot N)**:
  `e3de0ab8 9da287bb 6ef9de51 626387c3 9bbe68da `**`5adc3160`**` 1ab7f47e 19e31a86`

* **Today's mask test (boot N+1, after sysrq b)**:
  `fb12cd4c c1bf109e `**`5adc3160`**` 50baa6c2 c399f1f4 7b6dc2d0 5d1220b0 10fbbed0`

Only `5adc3160` (the channel-mean-correct render) is stable across
boots. **The other seven slots contain different garbage per boot.**
This is exactly the fingerprint your update-13 paragraph #1
predicted: "uninitialised SRAM state" — different leftovers from
the previous run survive each warm boot, but one slot consistently
re-derives the correct output (probably the slot that wins the race
to write *something* recognisable into the buffer).

A cold-cold boot would tell us whether `5adc3160` itself shifts
position within the cycle, or whether all 8 slots become
similarly-deterministic-but-different.

## Direct asks

### 1. Finding the wakelock holder

The GPU's `runtime_active` ref-count is non-zero throughout the
test. We need to identify what's holding the reference. Candidates:

* `devfreq` — but on a2xx mainline, devfreq is set up only when the
  GPU is in active use (it should release when idle).
* The compositor (LunaSysMgr) — but the test binary uses a separate
  GBM/EGL context, no shared buffers obvious.
* A previous test-binary instance keeping `/dev/dri/card0` mapped
  (some lifecycle-free issue in the binary or in mesa).
* The kernel itself (any of: hangcheck thread referencing the gpu,
  pm_runtime barrier, devfreq governor).

What's the cleanest way to enumerate runtime ref holders? `usage_count`
isn't enough — we want to know *who*.

### 2. Forcing power-collapse as a diagnostic

If we could force the GPU to power-collapse between every submit
(bypassing whatever's holding the ref), we could test whether your
theory's prediction holds: cycle collapses to 100/100 same hash.

Options we've considered:
* `echo on > /sys/.../power/control` then `echo auto` — toggles the
  governor mode; might force a full re-evaluation.
* Custom kernel patch: a `force_suspend` debug param in `a2xx_submit`
  that calls `pm_runtime_put_sync` on the gpu device after each
  submit returns, bypassing the autosuspend timer.
* Custom: make `a2xx_submit` issue a `pm_runtime_get + put_sync`
  pair that effectively cycles the device.

Which would best test the "SRAM survives runtime_active" theory
without breaking the submit path? We'd want to make sure a forced
suspend doesn't itself introduce new state.

### 3. Where could the SRAM cycle counter be?

If the cycle is a counter incremented per `CP_DRAW_INDX`, and it
doesn't show up in any RBBM status bit, where in the A2XX block
diagram could it live? Some thoughts:

* **CP MEQ (Micro Engine Queue)** — the 16-slot or 32-slot pre-fetch
  buffer the CP uses. State that persists across context switches.
* **VPC (Vertex Parameter Cache)** — interpolates vertex outputs.
  Has its own state machine and SRAM that may not advance through
  RBBM_STATUS.
* **SQ instruction store** — the wavefront scheduler keeps an
  SRAM-resident allocation table that survives context switches.
* **Constant SRAMs** — ALU/TEX/Bool/Loop. We already restore these
  via the sanitizer preamble, but maybe one isn't fully covered.

Any of these expose register-level access from CPU to inspect or
clear at submit-time?

### 4. Cold-cold boot

Still pending. With cycle reliably reproducing from `sysrq b`
(warm) reboots and `5adc3160` consistently the "correct" slot, the
cold boot would tell us whether:

* All 8 slots become "fresh garbage" (entire SRAM state cleared)
* Or some subset (maybe just `5adc3160`'s slot) is the only thing
  zero-initialized, suggesting the rest is power-domain-survivable
  even from cold.

Will run when feasible; needed someone with physical access to
hold the power button. (We can't do this from SSH.)

## What's in the git tip right now (for reference)

`76b56fe33fab` on `tenderloin/6.18/upstream-patches`:
* `a2xx_rbbm_poll_mask` (uint, 0644) — tunable poll mask
* `a2xx_rbbm_poll_enable` (bool, 0644) — toggle the poll
* `a2xx_wptr_poll_enable` (bool) — KGSL-style WPTR-poll, KNOWN
  BROKEN (hangs at ME_INIT). Not the focus right now.
* `a2xx_wptr_poll_delay` (uint) — alongside above.
* `a2xx_debug_wptr_delay` (uint) — pre-existing, the udelay knob.
