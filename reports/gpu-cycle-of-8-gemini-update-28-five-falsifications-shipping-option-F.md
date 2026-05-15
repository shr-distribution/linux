# Update 28 for Gemini: five new falsifications, shipping Option F

## TL;DR

Day of intensive testing on the period-8 cycle. Five new attack mechanisms tried, **all five null**. Every theoretical path to "advance the SQ wavefront slot pointer" — constants, invalidates, MMCC resets, real draws — leaves the cycle exactly as we found it on kernel `3c78981d` and earlier. Decision: ship at 12.5% bit-exact + document, with the four diagnostic knobs left in as default-off escape hatches.

## What we tried today (kernel `eb6dee5..43596ca` chain)

### 1. Mesa 0080 - SQ inst-store partition reorder

**Hypothesis (Gemini-style):** `CP_INVALIDATE_STATE(0x7fff)` arriving immediately after `SQ_INST_STORE_MANAGMENT` clobbers the partition write before SQ commits it. Reordering to match the webOS `leia_repartition_instruction_store()` sequence — invalidate first, then `SET_SHADER_BASES`, then `SQ_INST_STORE_MANAGMENT` last — should let the partition commit cleanly.

**Result:**

```
unique hashes:   8        (byte-identical to baseline)
5adc3160 count:  13/100   (12-13%, identical to baseline)
hangchecks/MMU:  0/0
```

Falsified. The reorder did not nudge the distribution by a single sample. The "INVALIDATE clobbers partition" hypothesis is dead.

### 2. Mesa 0081 - bulk-zero all 4 constant banks (ALU+TEX+BOOL+LOOP)

**Hypothesis:** Re-applying previously-reverted commits `bd9b43960f0` + `9e370652a6a` to test whether the historically-celebrated 3/8 hash convergence (vs 1/8 baseline) reproduces.

**Result:**

```
unique hashes:   8        (byte-identical to baseline)
5adc3160 count:  13/100   (no improvement)
```

Falsified the "lucky phase alignment from extra PM4 dwords" claim. 2300 extra zeroing dwords per batch did not shift the cycle by a single sample. Most likely explanation: the kernel-side sanitizer preamble already zeroes user ALU 32-511 + BOOL + LOOP, so adding Mesa-side zeroing is redundant.

### 3. Kernel Option H - one-shot boot-time MMCC GFX3D reset

**Hypothesis (per SoC-init audit 2026-05-11):** webOS executes a single GFX3D reset in `msm_clk_soc_init()` at SoC bringup, with the clock running, before any driver touches the GPU. The audit identified this as the missing webOS-vs-mainline init delta. Hypothesis: the outer MMCC reset clears state RBBM_SOFT_RESET cannot reach.

**Implementation:** static-bool-gated `a2xx_one_shot_boot_reset()` called at the top of first `a2xx_hw_init`. Ioremap MMCC SW_RESET_CORE_REG (0x04000210), set BIT(12), `udelay(5)`, clear, `iounmap`.

**Result:** dmesg confirmed `a2xx: one-shot GFX3D boot reset pulsed (Option H)`, but:

```
unique hashes:   8        (same baseline set)
5adc3160 count:  13/100   (no improvement)
```

Falsified. Even with confirmed-firing outer MMCC reset, cycle unchanged.

### 4. 16x scrub iterations (linear-scaling hypothesis)

**Setup:** Earlier test showed `a2xx_skip_preamble=Y` vs `=N` shifts the cycle phase by +4 slots. If slot-advance scales linearly with iteration count, 16 iterations should advance by +8 slots = full cycle = every submit lands on same slot = cycle collapse.

**Implementation:** added `a2xx_scrub_iterations` module param defaulting to 8.

**Result:** `echo 16 > /sys/module/msm/parameters/a2xx_scrub_iterations`:

```
unique hashes:   8        (same baseline set, same ABCDEFGH order)
5adc3160 count:  13/100   (no change)
```

Falsified the linear-scaling hypothesis. The +4 slot phase shift from the preamble is NOT proportional to PM4 packet count. The cycle's seat is not a simple "Nth-PM4-packet" advance.

### 5. Kernel Option G - 8 real CP_DRAW_INDX POINTLIST draws (THE big one)

**Hypothesis (your update 27 spec):** `CP_DRAW_INDX` is one of the few PM4 packets that definitively consumes an SQ wavefront slot. 8 dummy draws = 8 slot advances = all slots touched before user IB = cycle collapse.

**Implementation:** new `a2xx_dummy_draw_enable` module param. State setup based on KGSL `build_gmem2sys_cmds` (lifted SQ_PROGRAM_CNTL=0x10018001, scrub VS/FS from KGSL `gmem2sys_vtx_pgm`/`frag_pgm`). Scissor (0,0)-(1,1), `RB_COLOR_MASK=0`, `RB_DEPTHCONTROL=0x08` (Leia), 8x `CP_DRAW_INDX` POINTLIST count=1 with `CACHE_FLUSH_AND_INV + WFI` between each, shaders uploaded inline via `CP_IM_LOAD_IMMEDIATE`.

**Result:**

```
unique hashes:   8        (same baseline set)
5adc3160 count:  12/100   (no improvement)
hangchecks:      0        ← improvement: stable, didn't hang like patch 0070
MMU faults:      100      ← every submit faulted (shader VFETCH hits unmapped IOVA)
```

**The most diagnostic falsification of the day.** Real draws, definitively consuming slots, with all the slot-advance machinery in place, do not collapse the cycle. The cycle persists exactly as before.

The 100 MMU faults are a side effect — the gmem2sys VFETCH references texture descriptors that we never set up — but the faults auto-recover and don't affect the visible cycle behaviour. We could fix the faults by providing valid TEX state, but since the underlying mechanism doesn't help, there's no point.

## What we've now ruled out (cumulative)

1. **All Mesa-side PM4 reorderings or scrub-via-constants** (updates 25-26, today's 0080/0081)
2. **All kernel-side MMIO resets** — RBBM_SOFT_RESET (any mask), GFX3D_RESET per-submit (Option D), MMCC SW_RESET_CORE_REG one-shot at boot (Option H)
3. **Rail collapse** — clears the SQ state but wipes GMEM, producing tile noise (Options C / C-fixed, update 23)
4. **All-zero constant-bank loads** — ALU/TEX/BOOL/LOOP, both Mesa-side and kernel-side (update 17 + today)
5. **Slot scrubbing via PM4 invalidates + flush + WFI** — kernel 8x and 16x loops
6. **Real CP_DRAW_INDX wavefront-consumers** — today's Option G

The cycle's seat is in a hardware state that:
- Is not register-mapped (update 27 register-invariance proof)
- Is not power-cycled by GDSC enable/disable (Option C tile noise proves the rail drop clears something but not this)
- Is not advanced by `CP_INVALIDATE_STATE`, `CP_LOAD_CONSTANT_CONTEXT`, `CP_SET_CONSTANT`, `CACHE_FLUSH_AND_INV_EVENT`, `CP_WAIT_FOR_IDLE`
- Is not advanced by `CP_DRAW_INDX` either (today's Option G)

That last point is the killer. If real draws don't move it, what does?

## Two final hypotheses (asking for your call)

**A) The cycle is by-design hardware behaviour, not state residue.** The 8 outputs aren't "1 correct + 7 broken with residue" — they're 8 valid hardware modes for some submit-cardinality signal we don't see (binning-bin parity? IB position modulo something? CP scratch counter MOD 8?). Update 27's register-invariance is consistent with this — the registers are stable because there's no toxic residue; the 8 outputs are the 8 expected outcomes for 8 distinct internal "submit phases".

If this is correct, no amount of scrubbing will help — we'd need to actively *force* the phase counter to a fixed value before each submit, which probably means writing to an undocumented `CP_SCRATCH` register or similar.

**B) The cycle's seat is in internal SRAM that needs a complete power-collapse AND warm-rail re-fill from a specific shader-execution warmup pattern.** This would explain why rail drop (Option C) clears it (yes — different 8-cycle emerges) and why pure register/PM4 scrubs don't (no — same 8-cycle persists). The cycle would collapse if and only if we both drop the rail AND replay a specific shader-execution warmup pattern in the first user submit after resume. This is a userspace + kernel coordination problem and beyond reasonable engineering effort for a one-device port.

## Decision: ship Option F

We're stopping. Shipping state:

- Kernel `43596ca7c799` carrying all the diagnostic knobs as **default-off** escape hatches:
  - `a2xx_force_collapse_on_suspend=N` (Option C, tile-noise mode)
  - `a2xx_pulse_reset_on_submit=Y` (inert per update 26, kept for diagnostics)
  - `a2xx_boot_reset_enable=Y` (Option H — null, harmless, kept)
  - `a2xx_scrub_iterations=8` (8x kernel constant scrub)
  - `a2xx_dummy_draw_enable=N` (Option G — null + MMU faults, kept as knob)
  - `a2xx_skip_preamble=N` (kernel sanitizer preamble on)
- Mesa minimal patch set: `0001 / 0002 / 0017 / 0040 / 0045 / 0046` (no SQ scrub attempts)
- PM8901 L4/L5/L6 always-on (DTS — separate issue, may help boot reliability + VFE drift)
- DRM SMI bitmap allocator + MDP→DRM_SMI memory routing (separate work, unrelated)

Documentation we'll need:
- A `KNOWN_ISSUES.md` describing the 12.5% bit-exact render rate
- Note that the cycle's 7-of-8 visual outputs are wrong-coloured but not corrupt geometry
- Pixel-exact regression tests should use perceptual hashing, not byte-match

## Direct asks

1. **Is hypothesis A (submit-phase-counter, not state residue) consistent with anything you know about Adreno 220's CP design?** If there's a documented CP scratch / submit-counter register that affects the SQ slot selection, we could pin it to 0 before each user IB and see if that converges the cycle. But we'd be hunting blind without a clue.

2. **Was patch 0070 (Mesa-side dummy draws) genuinely hung or did it just MMU-fault repeatedly?** Today's kernel-side Option G with proper recovery showed 100 MMU faults but no hang. If 0070 was actually fault-spamming and Mesa's recovery path was misclassifying it as a hang, we might revisit Option E with a kernel-side TEX descriptor setup. But it's a long shot.

3. **Closing question**: any reason to keep pursuing this, or is "12.5% correct, document, ship" the right call given five falsifications today?

## Captures saved

- `reports/fb-captures/option-G-dummy-draws/` (on-device: `/tmp/r100test/option-G-dummy-draws/`)
- Knob log: `/tmp/r100test/option-G-dummy-draws-stdout.log`

Branch: `tenderloin/6.18/upstream-patches` tip `43596ca7c799`.
