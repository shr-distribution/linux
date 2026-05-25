# CAMSS / VFE31 Mainline-Submission Audit — Master Synthesis

**Date:** 2026-05-25
**Target:** HP TouchPad (APQ8060 / MSM8660), Linux 6.18 mainline `camss`
**Reference (ground truth):** webOS 2.6.35 vendor driver (`msm_vfe31.c`, `msm_io_vfe31.c`,
`msm_isp.c`, `mt9m113_reg.c`) + live register dumps in `reports/webos-*-dump.txt`

This consolidates four parallel deep audits. Read the per-area reports for full tables:
- `reports/audit/vfe31-datapath-accuracy.md` — register/value/sequence correctness
- `reports/audit/vfe31-mainline-compliance.md` — upstream coding/style/framework
- `reports/audit/camss-csid-csiphy-sensor.md` — MIPI RX + sensor + **drift root cause**
- `reports/audit/vfe31-completeness-coverage.md` — feature matrix, over-builds, DT/Kconfig

---

## 1. Executive verdict

**Functionally:** the *capture-critical* datapath is largely correct. Almost every
register value the VFE backend emits (BUS_CFG, AXI mode, MODULE_CFG, CORE_CFG, DEMUX,
XBAR, IRQ masks, CAMIF window/EFS/subsample, ping-pong bit convention, halt) **matches
the live webOS dumps byte-for-byte.** The YUV422 PIX path (UYVY→NV12/NV16) is real and
on-device-proven.

**For upstream submission:** **NOT submittable as-is.** It is a bring-up/debug driver
wearing the camss directory layout. It would be rejected on first review for: 14 module
parameters, file-scope mutable per-device state, wholesale bypass of the gen1 framework
vtable, edits that bleed VFE31 specifics into shared headers/files, a cross-subdev CSIPHY→VFE
poke, ~187 `dev_info` calls (several per-frame), and 900-line functions. None of these are
deep — they are systematic cleanup + one real structural rewrite (use the gen1 vtable).

**Bonus:** the audit produced a **new, specific, testable root cause for the 640x480
drift** that finally reconciles the webOS paradox (see §2).

---

## 2. The 640x480 drift — CONFIRMED RESOLVED (2026-05-25)

> **On-device A/B confirms it: it is NOT a HW limit.** With FS/FE short packets OFF
> (`mt9m113_skip_short_pkt=1`, default) the image walks 16 lines over 12 frames; with them
> ON (`=0`) drift is 0 across all 12 frames (corr ~1.0). Test was a runtime sysfs toggle —
> no reboot. Metric/evidence: `reports/audit/drift_measure.py`, `drift_skip{0,1}_f*.png`.
> **Upstream fix:** unconditionally write R0x3404=0x0080 (FRAME_CNT_EN) in the sensor start
> path and delete the param. (The FRAME_LENGTH re-apply below is the webOS-faithful
> alternative, still untested but expected to also work.)

The drift doc (`vfe31-640x480-drift-for-gemini.md`) currently calls it a DMA-fetch HW
limitation. That is **falsified** and should be retired:

- Ping-pong selection is correct every frame (`buf_verify … MATCH`), and 6 address-commit
  variants gave identical drift → it is **not** buffer/address management.
- The CAMIF *register* state is **byte-identical** to the working webOS 640 preview dump
  (`CAMIF_STATUS=0`, EFS=0x40, FRAME_CFG=0, WINDOW=0x01E00500) → it is **not** the CAMIF config.
- The webOS preview dump **is** the 640×480 mode (WINDOW 0x01E00500 = 480 lines × **1280
  bytes = 640 px YUV422**). webOS ran the same CAMIF setup with zero drift.

**Lead root cause (sensor side):** `mt9m113_configure_sensor_context()`
(`drivers/media/i2c/mt9m113.c:451-528`) re-writes Context-A geometry (ROW/COL start/end,
READ_MODE) on **every** stream-on but **omits `FRAME_LENGTH_A` (0x271F) and
`LINE_LENGTH_PCK_A` (0x2721)** — the registers that set vertical/horizontal blanking and
therefore the **frame period**. Because the MT9M113 sends no MIPI Frame-Start/End short
packets (`mt9m113_skip_short_pkt=1`, R0x3404=0) and FRAME_CFG=0, the VFE CAMIF runs in APS
mode and **counts lines internally** to find the frame boundary. After a Context-B→A
switch (and the heavy per-stream STANDBY→REFRESH→RUN/CAPTURE churn), the live Context-A
blanking is left stale, so the CAMIF locks to the wrong period and the readout walks
~30 lines/frame. 1280 (Context B) is the freshly-programmed path, so it stays locked —
**this exactly matches the asymmetry.** webOS never re-writes context geometry at runtime,
so it never desyncs.

**Two cheap tests (no rebuild for the first):**
1. Boot `mt9m113.mt9m113_skip_short_pkt=0` → enables FS/FE short packets → CAMIF gets a hard
   per-frame boundary independent of blanking. Capture `CAMIF_STATUS` from the first-frame
   dump at 640 to confirm whether it reads nonzero today.
2. Add `FRAME_LENGTH`/`LINE_LENGTH_PCK` to `configure_sensor_context()` for both contexts
   (A: 814/1228, B: 1369/1826), **or** stop re-writing context geometry at runtime and match
   webOS (program once in the init table, issue only `SEQ_CMD` on transitions).

Secondary suspects (lower priority, in the CSID/sensor report): per-stream AE re-convergence
without matching FRAME_LENGTH; CSIPHY IRQ mask `0xFFFFFFFF` vs webOS `0xFFF7F3FF` (re-enables
DATA/CLK_CMM/ID error bits → IRQ flood on the noisy binned link); legacy 153.6 MHz CSI
bit-clock not reproduced.

---

## 3. Cross-report reconciliation (conflicts resolved)

- **WM stride `width*2` (datapath F1, flagged "BUG") — FALSE POSITIVE.** F1 assumed the
  webOS preview dump was a 1280-wide frame (→ stride should be `width`). The CSID/sensor
  report proves the dump is **640×480** (1280 bytes = 640 px YUV422, 480 lines, Context A
  binned). So the WM stride of 1280 **is** `width*2`, and mainline's `width*2` is **correct**.
  Corroboration: 1280×1024 and frame-1 at 640 render cleanly, which a stride bug could not
  produce. **Do not change the stride.** (Note in the datapath report: F1 should be downgraded.)

- **FRAME_CFG=0 (my earlier hypothesis) — not the cause.** Both webOS and mainline leave it 0;
  webOS doesn't drift. The frame-lock comes from sensor-side blanking + CAMIF line-counting,
  which §2 addresses directly.

- **CAMIF_ERROR restart (my earlier hypothesis) — not active in the streaming path.**
  CAMIF_ERROR (IRQ_MASK_1 bit0) is masked off during normal streaming (mask1 = RESET_ACK only),
  so that handler doesn't run. The restart path is dead weight (and a submission smell), not the
  drift cause.

---

## 4. Submission blockers (consolidated, severity-ordered)

| # | Blocker | Where | Fix |
|---|---------|-------|-----|
| B1 | **14 module parameters** (8 in vfe-3-1.c, 3 in shared camss-vfe.c, 3 in mt9m113.c) | vfe-3-1.c:47-340; camss-vfe.c:962-996; mt9m113.c:39-52 | Remove all. Hardcode the webOS-verified value; delete debug knobs. |
| B2 | **File-scope / function-static per-device state** (`recording_state`, `zsl_state`, `pix_wm_pending`; ISR `irq_count`/`last_ping_pong`/`camif_error_count`) | vfe-3-1.c:226-234, 2828-2831 | Move to `vfe_device`/`vfe_output`, or delete with the dropped feature. ISR counters are pure debug → delete. |
| B3 | **Framework bypass** — `comp_done`/`wm_done` are NOPs, `ops_gen1` never set; driver hand-rolls buffer rotation, queue, halt, disable | vfe-3-1.c:7124-7147 + local reimpls | Implement the gen1 vtable like `camss-vfe-4-1.c` (set `ops_gen1`, use `vfe_isr_ops_gen1`, `vfe_video_ops_gen1`, `vfe_isr_comp_done`, `vfe_gen1_halt`, `vfe_disable_output`). The biggest single task. |
| B4 | **Shared-file contamination** — VFE31-only line IDs (VIDEO/ZSL), ~20 `vfe_device` fields, testgen externs, `pix_stride_factor`, software-SOF op added to common code | camss-vfe.h, camss-vfe.c | Revert. Keep only genuinely-MSM8660 ops (`enable_pending_camif` may be defensible) out of shared structs. |
| B5 | **Cross-subdev poke** — CSIPHY ISR reaches into `csiphy->camss->vfe[0]` to fake SOF/REG_UPDATE on undocumented BIT(22) | camss-csiphy-8x60.c:954-1014 | Delete the software-SOF path entirely; rely on HW CAMIF SOF (works by default). |

---

## 5. Accuracy findings worth fixing (true, not blockers)

| Sev | Finding | Where |
|-----|---------|-------|
| BUG/seq | Global reset writes 0x3FF then a **blind delay**; legacy waits for RESET_ACK IRQ before applying defaults. Comments are stale/self-contradicting. | vfe-3-1.c:2330-2337, 2278-2290 |
| DISCREPANCY | CAMIF start order: legacy does **REG_UPDATE then CAMIF_CMD** as one sequence; mainline defers WM enable to a REG_UPDATE ISR — verify REG_UPDATE precedes CAMIF=1. | vs legacy `vfe31_start_common` |
| DISCREPANCY | `RAW_CROP_WIDTH` (0x200) left at 0; working device shows `0x3FFF3FFF`. | not written |
| DISCREPANCY | Halt omits legacy's `AXI_STATUS&0x1` busy-poll and post-halt `0x3ef` reset-upon-stop. | vfe-3-1.c:3135 |
| MAGIC | UB-depth formula (912 budget) matches webOS 0x12F only by NV12 coincidence; two divergent formulas, one dead. Name/derive it. | vfe-3-1.c:5126-5135 |
| MAGIC | Scaler phase consts 0x310000/0x320000 unverified vs a webOS phase dump. | vfe-3-1.c:4688-4734 |
| DOC | XBAR (0x1A1B) and CAMIF reg labels emit correct values but the comments contradict the dumps. | F4/F10 |

Everything else in the datapath (BUS_CFG, AXI, MODULE_CFG, CORE_CFG, DEMUX, IRQ masks,
COMP mask, EFS, SUBSAMPLE, BUS_CMD, hw_version, ping-pong convention) **matches and is OK.**

---

## 6. Scope: what to ship vs. drop for v1

**Ship (real, validated):** single PIX/YUV path — NV12 + NV16 (+ UYVY passthrough),
640×480 (Context A) and 1280×1024 (Context B). Omitting the entire legacy ISP-tuning +
3A-stats surface (107 commands) is **correct** — mainline camss has no kernel ABI for it,
and sibling backends omit it too.

**Drop (non-functional or unvalidated on this silicon):**
- RDI raw-bypass (AXI=0x60) — CAMIF filters non-YUV data types; confirmed dead.
- RAW-through-PIX "Y-plane hack" and the testgen generator (HW block doesn't exist on APQ8060).
- VIDEO + ZSL lines and their global state machines (reuse PIX WMs, unvalidated; no legacy ZSL).
- Dummy "bit-bucket" buffer and software-EOF/SOF workaround.
- RAW8/RAW10 Bayer entries in `formats_rdi_vfe31` (advertised but fail STREAMON).
- Drop `line_num` 6→4 once VIDEO/ZSL go.

---

## 7. DT / Kconfig / build readiness

- **Build wiring is correct** and follows upstream convention: `camss-vfe-3-1.o`,
  `camss-csid-8x60.o`, `camss-csiphy-8x60.o` under `CONFIG_VIDEO_QCOM_CAMSS`; no new Kconfig
  symbol needed. `CONFIG_VIDEO_MT9M113` present.
- **camss.c MSM8660 wiring complete** (resources, clocks, IRQs, single `VFE_GDSC`, unified
  CSIPHY+CSID reg space, no CSID IRQ). Open questions: `csi_pix`/`csi_rdi` clock necessity
  (webOS uses MISC_CC_REG=0); no `iommus` on the camss node (CMA-only — document as a limitation).
- **`aptina,mt9m113.yaml` binding exists.** **`qcom,msm8660-camss.yaml` does NOT — must be
  written**, describing the MSM8660 shape (unified CSIPHY+CSID reg space, no separate CSID
  reg/IRQ, clock-name set, single GDSC, interconnect `vfe-mem`).
- Remove the stray unused `mt9m114.c` to avoid confusion.

---

## 8. Suggested remediation order

1. **Confirm the drift cause first** (§2 test 1 — one boot, zero code). Then fix the sensor
   geometry re-apply (§2 test 2). A clean capture at 640 makes the rest of the cleanup honest.
2. **Strip the over-build** (§6) — deleting RDI/raw-pix/testgen/VIDEO/ZSL/software-SOF removes
   most module params, most globals, and most shared-file edits in one pass.
3. **Remove remaining module params + globals; revert shared-file edits** (B1, B2, B4, B5).
4. **Re-architect onto the gen1 vtable** (B3) — the one substantial structural task.
5. **Datapath fidelity fixes** (§5) + decompose the giant functions + kill debug spam.
6. **Write the camss binding doc; resolve csi_pix/csi_rdi clocks; document CMA/no-IOMMU.**
7. mt9m113 polish: drop the private context control (derive from set_fmt), demote logging,
   name MCU sequences, real author/SoB. The sensor is plausibly submittable independently.

The sensor driver and the build/DT wiring are close. The VFE backend needs a focused rewrite
of its *plumbing* (framework conformance + de-globalization), not of its *register logic*,
which is already faithful to the hardware.
