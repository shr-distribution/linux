# A220 patch audit — kernel a2xx + Mesa freedreno (2026-05-22)

Baselines: **kernel = `v6.18` tag**, **Mesa = `e57fca6`** (mesa-latest). Scope: a2xx GPU driver
+ Mesa freedreno a2xx only. MDP4 (display) is out of scope — separate block, unrelated to the binner.

## Why so much is droppable
Root cause is now known (see `webos-binning-capture-2026-05-22.md`): the A220 VSC tile-binner is always
in the primitive path; the fix is to **configure** it (compute bin grid from RT size + GMEM byte budget
incl. depth → program 8 VSC_PIPE config/addr/len with backing BOs → SQ_GPR_MANAGEMENT=0x7f010 →
LRZ_VSC_CONTROL 3→1 → **plain DRAW_INDX**, no separate binning pass). Every "scrub / reset / poll /
dummy-draw / multi-flush" experiment is therefore a falsified dead end. The tree is currently carrying
~40 such kernel commits and ~11 Mesa commits, several **active by default on every boot/submit**.

---
## PRIORITY 0 — remove now (active by default, degrading every render/boot)

**Kernel (`a2xx_gpu.c` / `adreno_gpu.c`):**
1. `a2xx_pulse_reset_on_submit=true` — RBBM_SOFT_RESET pulse every submit (e3ffca5 + tuning 26ec862/3f45f80/b0025a7/a99193b/6e40f49)
2. `a2xx_scratch_reset_enable=true` — scratch-pin in preamble (c148107)
3. Unconditional `CP_SCRATCH_REG2=0` every submit, not even gated (0521326/7fc9112)
4. `a2xx_rbbm_poll_enable=true` — RBBM_STATUS spin every submit (76b56fe/653b1b2)
5. `a2xx_boot_reset_enable=true` — MMCC GFX3D reset at first hw_init (16968d6)
6. `adreno_test_outer_sync=true` — outer_sync every flush (adreno_gpu.c)
7. The huge `pr_info` + 64-entry MH_DEBUG sweep on **every pm_resume** (e3767c5/1ea8d68/d372129/fb4b1b5) → gate behind debug

**Mesa:**
8. **`e7ae801` multi-flush v5a** — fires **15 dummy MSM_SUBMITs per A22X render** by default (`freedreno_gmem.c` mult=16). Single biggest live liability.
9. `30c43d3` CP_SCRATCH milestone markers — pollutes default cmdstream (writes SCRATCH0–7 every a22x submit)

---
## KERNEL — verdicts

**KEEP (genuine A220 correctness / clean infra):**
- `8f017aa` clk: msm8660/apq8060 split from msm8960
- `46d0e01` MH_MMU_CONFIG = BEH_TRAN_FLT (fault not range — matches webOS KGSL)
- `00ad56c` MH_ARBITER_CONFIG: clear IN_FLIGHT_LIMIT_ENABLE (matches webOS readback)
- `160f6d7` + adreno_device.c ICC-skip + a2xx_gpu.h `icc_path` — manual ICC bandwidth voting
- adreno_gpu.c: `MSM_PARAM_VA_START/SIZE` for shared-VM a2xx; OPP/autosuspend-from-DT robustness
- In-tree-verified hw_init writes: `RBBM_PM_OVERRIDE2=0x1a0` (a22x), `MH_CLNT_INTF_CTRL_CONFIG1=0x00032f07` (1K-boundary erratum), `SQ_INTERPOLATOR_CNTL=0xffffffff`
- Cleanup/reverts: `47b8a70` (drop dead scaffolding), `f2abc8b`/`8173fc3`/`1f47e5f` (experiment reverts)
- Fault/hang-only forensics (low cost): `80461fb`, `e8ca5d3`, `fee6b52` (gpummu iova→kvirt), `09533b9`

**REVISE (real fix entangled with experiment — split, don't drop wholesale):**
- `f4c8160` SQ_GPR_MANAGEMENT (a2xx_gpu.c:1262-63): keep `=0x00040400`, **delete prepended `=0x0007f010` pulse**
- `f3c1bbf`/`2ea5e33` VA-base = 0x66000000: **keep the VA-base** (matches webOS), strip bundled SQ-probe pr_info
- `12bdc907`(+b9a0d9f6) gpummu idle-wait before MH_MMU_INVALIDATE: unmap-path arguably defensible, map-path is experiment — trim
- "dump every hw_init" diagnostics + new files `a2xx_debug.c/.h`, `a2xx_debugfs.c`: keep behind debugfs but flip default-on submit-path knobs (`a2xx_debug_flush_cache`, `a2xx_debug_first_submit_sq`) OFF

**DROP (falsified experiments — full list):**
- Sanitizer-preamble scrub stack: 161cf4f3, 0ba88d20, 5c47bbed, 44ffd6e6, ce433474, fb2cdbb4, 36d78c83, ec097544, c402c659, 3ee58078, a2f05cc2, 86e5a24e + reverted pairs d75656a2/b311855a, 84de0025/7b261eb4
- Option C (force_collapse): 9f91120c, 457e0988, 4b2a3205, 174a7b70, 6f24762b, 3c78981d
- Option D (pulse-reset): e3ffca5 + 26ec862/3f45f80/b0025a7/a99193b/6e40f49 (P0 above)
- Option G (dummy draws): 43596ca, 4a4a662
- Option H (boot reset): 16968d6 (P0)
- Option I (scratch pin): c148107 (P0)
- WPTR-poll mode (self-documented BROKEN/hangs ME_INIT): 6cc4457, cb28e40, 8f3e367, 653b1b2
- Reverted experiments: 402950a, c2d955b, 45a8c11, speculative-CFG2 part of fb4b1b5

**Stale/contradictory to clean up:** `a2xx_force_collapse_on_suspend` DESC says "Default true — shippable" but default is false; `a2xx_wptr_poll` still compiled in despite "KNOWN BROKEN" comment.

---
## MESA — verdicts (19 commits)

**KEEP (5):**
- `c68927f` is_a22x() helper (infra)
- `d34a846` scheduler instruction-limit increase (fixes shader truncation / "faceted shading")
- `96e83a3` non-fast clear-color → PS CONST[0] (verified correctness)
- `319bb5b` WAIT_FOR_IDLE at fd2_emit_restore (consider a22x-gating)
- `717c734` cache flush+inv at batch start (keep event; strip the later FD2_NO_CACHE_FLUSH_INV bisect knob)

**DROP (11 + litter):**
- `.orig` files: `fd2_gmem.c.orig`, `fd2_draw.c.orig`, `ir2_nir.c.orig`
- `e7ae801` multi-flush (P0), `30c43d3` scratch markers (P0)
- `8d61035` FD2_VSC_DUMP, `52b9523` CP_SET_BIN_DATA/PIN_BIN_ID, `73283ff` end-of-tile DEALLOC,
  `b5e791e` 0x0ee2 bisects, `5820ad4` per-draw DEALLOC, `c7ba478` TC_CNTL_STATUS L2_INV,
  `12d6bc2`+`5320c2c` VSC reg-save parity (falsified), `ddedf00` split-binning (keep concept in notes only)

**REVISE / salvage toward the real fix (3):**
- `a11d5c4` Fork A/B prelude — **the salvage core**: VSC pipe BO alloc (256KB), `VSC_BIN_SIZE`,
  `VSC_PIPE_CONFIG[]`, partitioning enable, `LRZ_VSC_CONTROL`, `SQ_GPR_MANAGEMENT=0x7f010` are the correct
  pokes. **Un-gate from the separate-binning-pass model**; drive directly in `fd2_emit_tile_init` with plain DRAW_INDX.
- `19be2f2` Fork A/B defaults/env toggles — salvage prelude, delete the FD2_* toggles + wrong-model default.
- `19a96f1` per-tile VGT_CURRENT_BIN_ID — else-branch currently dead (use_hw_binning=true); re-evaluate vs webOS capture if binning-pass is removed.
- `02cd632` ir2_nir memexport skip — only needed because the separate binning pass is on; revisit when it's removed.

**Mesa structural fix direction:** `use_hw_binning()` returns true for A22X → triggers a **separate binning
shader pass** (`fd2_gmem.c:1363`) the webOS driver never does. The generic GMEM bin-sizing math
(`freedreno_gmem.c:164-197`, color+depth cpp into GMEM) and the VSC_PIPE register programming
(`fd2_gmem.c:1072-1140`) **already exist** — the real fix is to drive that prelude with **plain DRAW_INDX,
no separate pass**, and verify the bin math yields the captured `0x108` (2D) / `0x187` (3D) VSC_BIN_SIZE.
Also reconcile disengage `LRZ_VSC_CONTROL=0` (current) vs capture's steady `=1`.

---
## Headline
- **2 things hurting every frame today:** Mesa `e7ae801` (15 dummy submits/render) and the kernel
  default-on pulse-reset/poll/scratch family. Removing these alone should improve perf/stability immediately.
- **The real binner fix is ~half-scaffolded already** in Mesa (`a11d5c4` prelude + generic bin math) —
  it's buried under the wrong execution model, not missing. Salvage, don't restart.
