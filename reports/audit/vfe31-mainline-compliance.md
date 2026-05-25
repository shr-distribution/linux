# VFE31 Mainline Submission Compliance Audit

Date: 2026-05-25
Scope: upstream-submission readiness ("will linux-media / linux-arm-msm maintainers reject this?")
Files audited:
- `drivers/media/platform/qcom/camss/camss-vfe-3-1.c` (7280 lines)
- `drivers/media/i2c/mt9m113.c` (3257 lines)
- Contaminated shared framework: `camss-vfe.c`, `camss-vfe.h`, `camss-csiphy-8x60.c`
- Reference (clean): `camss-vfe-4-1.c`, `camss-vfe-gen1.c/.h`, `camss-vfe-17x.c`

Verdict: **NOT SUBMITTABLE.** This is a working bring-up/debug driver, not an upstream
candidate. It would be rejected on first review for module parameters, global mutable
state, framework bypass, and debug spam alone. A full rewrite of the data path against the
gen1 vtable is required, plus reverting all edits to shared framework files.

---

## Severity-ranked findings

| # | Severity | Area | Location | Finding |
|---|----------|------|----------|---------|
| 1 | BLOCKER | Module params | vfe-3-1.c:47-49,159-162,188-191,209-212,248-251,303-306,318-321,337-340 | 8 debug/experimentation module_params in the HW backend. All must be removed. |
| 2 | BLOCKER | Module params | camss-vfe.c:962-963,973-975,994-996 | 3 more module_params bolted into the **shared** framework file (`vfe31_use_testgen`, `vfe31_testgen_pixel_dims`, `software_eof_enable`), all `EXPORT_SYMBOL`. Touches every SoC. |
| 3 | BLOCKER | Module params | mt9m113.c:39-52 | 3 sensor tuning module_params (`mt9m113_pre_mipi_delay_ms`, `mt9m113_cont_mipi_clk`, `mt9m113_skip_short_pkt`). |
| 4 | BLOCKER | Global mutable state | vfe-3-1.c:226-227,234 | File-scope `vfe31_recording_state`, `vfe31_zsl_state`, `vfe31_pix_wm_pending` hold per-device runtime state. Non-reentrant, breaks multi-instance, racy from IRQ. |
| 5 | BLOCKER | Global mutable state | vfe-3-1.c:2828-2831 | Function-`static` `first_irq_time`, `irq_count`, `camif_error_count`, `last_ping_pong` inside the ISR. Per-device counters living in process-global storage; shared across all VFE instances; written without locking. |
| 6 | BLOCKER | Framework bypass | vfe-3-1.c:7124-7141 | `comp_done`/`wm_done` isr_ops are NOPs; the driver does buffer rotation itself in `vfe31_wm_done()` instead of using `vfe_isr_comp_done()`. `ops_gen1` is never set (NULL), so the entire gen1 layer (`vfe_video_ops_gen1`, `vfe_queue_buffer`, `vfe_gen1_halt`, `vfe_disable_output`) is reimplemented locally. 4-1.c does `vfe->ops_gen1 = &vfe_ops_gen1_4_1; vfe->isr_ops = vfe_isr_ops_gen1; vfe->video_ops = vfe_video_ops_gen1;` (4-1.c:989-991) — this driver wires none of that. |
| 6b| BLOCKER | Framework bypass | vfe.h:50-59,103-127,138-208,393-405 | The **shared** `camss-vfe.h` was edited: new line IDs `VFE_LINE_VIDEO`/`VFE_LINE_ZSL`, new hw_ops (`enable_pending_camif`, `vfe_cleanup`), ~20 VFE31-only `vfe_device` fields (shadow regs, dummy_buf, pending_*, raw_through_pix, camif_pending), testgen externs, `pix_stride_factor`. All bleed VFE31 specifics into the common struct seen by every backend. |
| 7 | BLOCKER | Framework bypass | camss-csiphy-8x60.c:954-1014 | CSIPHY calls into VFE (`vfe_trigger_software_sof`, `vfe_trigger_software_reg_update`) on undocumented `BIT(22)`. A subdev reaching across to poke another subdev's HW is an architectural non-starter upstream. |
| 8 | MAJOR | Debug spam | vfe-3-1.c (187 `dev_info`) | 187 `dev_info()` vs 1 in 4-1.c. Several are unconditional in the per-frame path: `vfe31_wm_done` line 2643 (every frame), 2659/2678/2684 (first-frame dumps with ~6 readl each), 2732 ("buf_verify" reads PING+PONG every frame). ISR also logs at 3031,3046,3055,3073,3086. Must be deleted or converted to tracepoints/`dev_dbg`. |
| 9 | MAJOR | Huge functions | vfe-3-1.c:3201-4105 | `vfe31_enable()` = ~904 lines. |
| 9b| MAJOR | Huge functions | vfe-3-1.c:4894-5809 | `vfe31_configure_pending_camif()` = ~915 lines. |
| 9c| MAJOR | Huge functions | vfe-3-1.c:6273-7021 | `vfe31_enable_pending_camif()` = ~748 lines. `vfe31_isr` = ~310 lines (2825-3135), `vfe31_wm_done` = ~229 lines. Far beyond reviewable size; must be decomposed. |
| 10| MAJOR | Locking — IRQ regpoke w/o lock | vfe-3-1.c:3012-3093 | The REG_UPDATE branch of the ISR writes WM CFG registers and REG_UPDATE_CMD driven by the global state machines **without** taking `output_lock`, concurrent with `vfe31_wm_done` (which does take it) and with `vfe31_queue_buffer`. Data race on `output->wm_idx`/state and on HW. |
| 11| MAJOR | Locking — duplicate dispatch | vfe-3-1.c:2896-2899 vs 3115-3127 | `IMAGE_COMPOSITE_DONE_n(1)` is handled twice in the same ISR pass (once as "RDI WM0", once for ZSL/RDI). Can double-complete a buffer / double-advance sequence. |
| 12| MAJOR | Dead/experimental code | vfe-3-1.c:164-191, raw_through_pix, vfe31_raw_pix_mode (10 uses), vfe31_rdi_efs_cfg (3), vfe31_rdi_force_16bpp (7) | RAW-through-PIX "Y-plane hack", RDI-emulation EFS/16bpp overrides, AXI-mode toggles — abandoned bring-up experiments threaded through the data path. `qcom,vfe31-raw-through-pix` is an undocumented vendor DT prop. |
| 13| MAJOR | Dead/experimental code | camss-vfe.c testgen + vfe-3-1.c:6070-6273 | Internal test-generator path (`vfe31_configure_testgen`, 203 lines) is debug-only and must not ship. |
| 14| MAJOR | Style — editorializing comments | vfe-3-1.c (183 "webOS" refs, 218 webos/hack/samsung/paradox/drift lines) | Comments narrate the reverse-engineering journey ("WebOS DISABLED CbCr...we're attempting what webOS never tested", "SRAM Overlap Paradox", "640x480 drift is HW limit"). Upstream wants terse technical comments, not a lab notebook. |
| 15| MAJOR | vb2/V4L2 — dummy buffer | vfe.h:198-205, vfe-3-1.c:7243-7253 | A `dma_alloc_coherent` "bit bucket" for unconfigured WMs masks misrouted XBAR config rather than configuring HW correctly. Upstream reviewers reject "point it at scratch so it doesn't crash". |
| 16| MAJOR | Reliability — CAMIF_ERROR as normal path | vfe-3-1.c:2950-3006 | ISR treats CAMIF_ERROR (missing MIPI FE) as the expected per-frame completion trigger and restarts CAMIF + fakes wm_done from inside the error handler. This is a sensor/CSI bring-up symptom, not a HW backend behavior. |
| 17| MINOR | Barriers — excess wmb | vfe-3-1.c (57 `wmb()`) | 57 `wmb()` against 245 `writel_relaxed`. Cargo-culted ordering. Use `writel()` where ordering vs the next MMIO matters; reserve `wmb()` for DMA-vs-MMIO ordering and justify each. |
| 18| MINOR | Style — magic constants | vfe-3-1.c:345-356 | `vfe31_get_bus_cfg()` returns bare `0x02AAA771`, `vfe31_get_bus_cmd_reload()` returns `0x7FFF` with only a "webOS dump" comment. Needs field-level #defines. |
| 19| MINOR | mt9m113 — custom V4L2 control | mt9m113.c:54-57,2737, 3101 | `V4L2_CID_MT9M113_CONTEXT` in `V4L2_CID_USER_BASE` range exposes A/B context as a private control. Context selection should derive from set_fmt resolution, not a vendor control knob. |
| 20| MINOR | mt9m113 — debug logging | mt9m113.c (34 dev_info, 20 dev_warn) | Excessive info/warn (e.g. 1405 "Forcing Context B"). Demote to dev_dbg. |
| 21| MINOR | mt9m113 — opaque MCU register blobs | mt9m113.c MCU indirect (0x098C/0x0990) sequences | Large indirect MCU variable writes ported from webOS `mt9m113_reg.c`; acceptable for sensors but need named #defines / documented sequences, not raw arrays. |
| 22| NIT | Copyright/attribution | vfe-3-1.c:6-9 | "Copyright (C) 2025 (based on Code Aurora Forum VFE31 driver)" — needs a real author/SoB; mt9m113.c:5 "Copyright (c) 2024 Linux Enthusiasts" is not an acceptable upstream author. |
| 23| NIT | Forward declarations | vfe-3-1.c:24-34 | Block of forward decls suggests poor ordering; reorder definitions to remove them. |

---

## Framework conformance detail (#3 expanded)

How 4-1.c (clean gen1 backend) plugs in vs how 3-1.c does it:

| Concern | gen1 / 4-1.c (correct) | vfe-3-1.c (divergent) |
|---------|------------------------|------------------------|
| isr_ops | `vfe_isr_ops_gen1` (comp_done=`vfe_isr_comp_done`, wm_done real) | `vfe31_isr_ops` with **NOP** comp_done/wm_done (7124-7141) |
| ops_gen1 | `vfe->ops_gen1 = &vfe_ops_gen1_4_1` (4-1.c:990) | **never assigned** (NULL) |
| video_ops | `vfe_video_ops_gen1` / `vfe_queue_buffer` | local `vfe31_queue_buffer` (7147) |
| buffer rotation | `vfe_isr_comp_done()` in gen1.c | hand-rolled in `vfe31_wm_done()` (2596) with per-frame readl verification |
| halt | `vfe_gen1_halt()` | local `vfe31_halt()` (3135) |
| disable | `vfe_disable_output()` (gen1.c:36) | local `vfe31_disable()` (4105) doing immediate CAMIF stop |
| reg_update/SOF | driven by gen1 state, lock-held | driven by file-scope state machines from ISR w/o lock (#10) |

The driver effectively forks the gen1 data path while still living in the gen1 directory and
struct. Upstream expectation: a VFE31 gen1 backend implements `vfe_hw_ops_gen1` (wm_enable,
wm_frame_based, wm_set_ping/pong, reg_update, etc.) and lets `camss-vfe-gen1.c` own buffer
lifecycle. None of that is done here.

---

## Must-fix-before-submission checklist

Blockers (driver is dead-on-arrival without these):
- [ ] Remove all 14 module_params (8 in vfe-3-1.c, 3 in camss-vfe.c, 3 in mt9m113.c). Replace any that encode a real HW decision with format/DT-driven logic; delete the rest.
- [ ] Eliminate all file-scope and function-`static` mutable runtime state (#4, #5). Move recording/zsl/pix-pending into `vfe_device`/`vfe_output`; move ISR counters out entirely (they are pure debug).
- [ ] Revert every edit to shared framework files: `camss-vfe.h`, `camss-vfe.c`, `camss-csiphy-8x60.c`. New line IDs, hw_ops additions, software-SOF/EOF hooks, testgen, pix_stride_factor, dummy-buf fields must not land in common code.
- [ ] Implement the gen1 vtable properly: set `ops_gen1`, use `vfe_isr_ops_gen1`, `vfe_video_ops_gen1`, `vfe_isr_comp_done`, `vfe_gen1_halt`, `vfe_disable_output`. Delete the local reimplementations.
- [ ] Remove the cross-subdev CSIPHY→VFE software-SOF/REG_UPDATE plumbing; solve frame sync within the sensor/CSID model or document a sanctioned mechanism.

Majors:
- [ ] Strip debug spam: 187→~handful of `dev_dbg`; zero unconditional logging in ISR/per-frame; convert diagnostics to tracepoints.
- [ ] Decompose `vfe31_enable` (904), `vfe31_configure_pending_camif` (915), `vfe31_enable_pending_camif` (748), `vfe31_isr` (310), `vfe31_wm_done` (229) into small reviewable functions.
- [ ] Fix ISR locking: take `output_lock` around the REG_UPDATE state-machine register writes (#10); remove the duplicate COMPOSITE_DONE_1 dispatch (#11).
- [ ] Remove RAW-through-PIX, testgen, RDI EFS/16bpp overrides, dummy-buffer, and the CAMIF_ERROR-as-frame-done path. Decide one real capture path and configure HW correctly.
- [ ] Replace narrative/editorializing comments with concise technical ones.

Minors/Nits:
- [ ] Audit the 57 `wmb()`; keep only justified DMA/MMIO-ordering ones.
- [ ] Name magic constants (BUS_CFG 0x02AAA771, BUS_CMD 0x7FFF, XBAR nibbles).
- [ ] mt9m113: drop the private context control (derive from set_fmt); demote logging; name MCU sequences; real author/copyright + SoB on both files.

---

## mt9m113.c assessment (positives)

It is structurally closer to clean than the VFE backend:
- Uses CCI regmap (`v4l2-cci.h`), proper `cci_write`/indirect MCU access (338-366).
- Proper `pm_runtime` with autosuspend (3179-3190, runtime_suspend/resume 2879-2908).
- Two-subdev PA/IFP model with separate ctrl handlers, standard pad ops
  (`v4l2_subdev_get_fmt`, enum_mbus_code, set_fmt), `v4l2-fwnode` endpoint parsing.
- Standard controls via `v4l2_ctrl_new_std*`, `MODULE_DEVICE_TABLE`, `aptina,mt9m113` compatible.

Remaining issues are the 3 module params, the custom USER_BASE context control, log
verbosity, opaque MCU register blobs, and author/copyright lines — all listed above. With
those fixed the sensor driver is plausibly submittable independently of the VFE work.
