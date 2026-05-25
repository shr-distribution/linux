# VFE31 Mainline Port — Feature Completeness & Scope Audit

**Date:** 2026-05-25
**Target:** HP TouchPad (APQ8060 / MSM8660), Linux 6.18 mainline camss
**Goal:** Assess upstream-readiness (v1) of the VFE 3.1 backend vs. webOS 2.6.35 vendor driver.

Files audited:
- `drivers/media/platform/qcom/camss/camss-vfe-3-1.c` (7280 lines)
- `camss-vfe.c`, `camss-vfe.h`, `camss-vfe-gen1.c`, `camss.c`, `Kconfig`, `Makefile`
- `drivers/media/i2c/mt9m113.c`
- Legacy: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.{c,h}`
- DT: `qcom-msm8660.dtsi`, `qcom-apq8060-tenderloin-common.dtsi`

---

## Executive Summary

The legacy `msm_vfe31.c` is a **full ISP control plane** (107 command IDs spanning AEC/AWB/AF/RS/CS/IHIST stats, full tuning pipeline — gamma, LA, color-correction, chroma suppression, MCE, SCE, ASF, demosaic/BPC/ABF, rolloff — sync/async timers, EPOCH, frame-skip, liveshot, JPEG enc). The mainline backend is a **capture-only bring-up driver**: CAMIF → DEMUX → scaler → XBAR → write-masters, producing YUV planar output. **Omitting the entire ISP tuning + stats surface is correct and expected for an upstream v1** — mainline camss has no in-kernel 3A/ISP-tuning userspace ABI, and other camss backends (4-1, 4-7, etc.) omit them too.

The real concerns for upstream are **not missing features** but **mainline over-build**: 11 module parameters, three experimental capture modes (RDI raw-bypass, RAW-through-PIX, testgen) that are **confirmed non-functional on this silicon**, global-static recording/ZSL state machines, and a `dummy buffer` / `software-EOF` workaround layer not present in any other camss backend. These need to be stripped or gated before submission.

**What actually works on-device (per reports + MEMORY):** YUV422 PIX capture (UYVY in → NV12/NV16 out) via the DEMUX path. `rawpix1280` (1280-wide RAW-through-PIX) works; pure RDI raw (AXI=0x60) does **not** — CAMIF silicon filters non-YUV MIPI data types. 640x480 has a known frame-drift HW-limit artifact.

---

## 1. Legacy Feature Inventory (feature matrix)

`needed-for-v1?` = needed for *correct capture bring-up upstream*, not for a full camera HAL.

| Feature (legacy V31_* / proc_general) | Legacy | Mainline | Needed v1? | Note |
|---|---|---|---|---|
| RESET / global reset | yes | **yes** (`vfe31_global_reset`) | yes | core |
| START / STOP / halt | yes | **yes** (`vfe_enable`/`disable`/`halt`) | yes | core |
| OPERATION_CFG (continuous/snapshot) | yes | **partial** | yes | mainline streams continuous only; snapshot/operation-mode register not driven |
| AXI_OUT_CFG (WM ping/pong, burst) | yes | **yes** | yes | per-WM addr/stride/UB applied |
| CAMIF_CFG (window, sync) | yes | **yes** | yes | APS mode (EFS_CFG=0) |
| AXI_INPUT_CFG (bus-read input) | yes | **absent** | no | offline/bus-read path; not needed for live capture |
| DEMUX_CFG (UYVY→Y/CbCr) | yes | **yes** | yes | 0xC9CA per webOS dump |
| BLACK_LEVEL_CFG | yes | **absent** | no | ISP tuning |
| ROLL_OFF_CFG (lens shading) | yes | **absent** | no | ISP tuning |
| DEMOSAIC_0 / BPC / ABF | yes | **absent** | no | Bayer ISP; sensor outputs YUV so N/A |
| FOV_CFG (crop) | yes | **absent** | no* | not needed unless cropping; could matter for exact sizing |
| MAIN_SCALER_CFG | yes | **partial** | yes | scaler used for NV12 vertical 480→240; not exposed as scaling control |
| WB / COLOR_COR / RGB_GAMMA / LA | yes | **absent** | no | ISP tuning |
| CHROMA_EN / CHROMA_SUP / CHROMA_SUBS | yes | **partial** | yes | CHROMA_SUBS driven (0x30 NV12 / 0x10 NV16); enhance/suppress omitted |
| MCE / SK_ENHAN (SCE) / ASF | yes | **absent** | no | ISP tuning |
| S2Y / S2CbCr (scaler2) | yes | **absent** | no | secondary scaler |
| OUT_CLAMP_CFG | yes | **absent** | no* | clamp; fine to omit, may affect levels |
| FRAME_SKIP_CFG / framedrop pattern | yes | **absent** | no | frame-rate control |
| Stats: AEC/AWB/AF (start/stop/enq/update) | yes | **absent** | no | no kernel 3A ABI upstream |
| Stats: RS/CS/IHIST | yes | **absent** | no | same |
| SYNC_TIMER / ASYNC_TIMER | yes | **absent** | no | flash/strobe timing |
| EPOCH1/2 ACK | yes | **absent** | no | mid-frame interrupt; not used |
| START/STOP_RECORDING (video path) | yes | **reimplemented** (state machine) | partial | see over-build §2 |
| CAPTURE (snapshot) | yes | **partial/absent** | no | single-shot snapshot not wired; streaming only |
| LIVESHOT | yes | **absent** | no | snapshot-during-video |
| JPEG_ENC_CFG / JPEG_OUT_BUF | yes | **absent** | no | offline JPEG; out of scope |
| RAW_OUT / RAW_IN buffer enq | yes | **absent** (RDI path instead) | no | see RDI over-build |
| GET_HW_VERSION | yes | **yes** (`vfe31_hw_version`) | yes | |
| GET_FRAME_SKIP_COUNTS | yes | **absent** | no | |
| REG_UPDATE handshake | yes | **yes** | yes | core; drives state machine |
| Violation / error IRQ handling | yes | **yes** (`vfe31_violation_read`) | yes | core |

\* "no*" = acceptable to omit for v1 but may cause minor correctness/levels/sizing differences worth a code comment.

**Verdict on §1:** Mainline correctly implements the capture-critical subset (reset/start/stop, CAMIF, DEMUX, AXI WM, scaler-for-420, chroma-subsample, reg-update, error IRQ). Everything absent is ISP-tuning or 3A-stats that has **no upstream kernel ABI** and is fine to omit for v1.

---

## 2. Mainline Over-Builds (features NOT in legacy / not justified)

| Over-build | Working HW? | Recommendation for upstream v1 |
|---|---|---|
| **RDI raw-bypass (AXI_OUT_MODE=0x60, MODULE_CFG=0)** | **NO** — `vfe31-rdi-investigation-summary.md`: CAMIF silicon filters non-YUV MIPI data types; RAW8 (dt=0x2A) → 0 pixels; AXI bypass path never transfers. Confirmed dead. | **DROP** or hide behind a clearly-marked experimental gate. The RDI *line* plumbing (RDI0–2 video nodes) can stay as camss convention, but the 0x60 raw mode and `vfe31_calc_rdi_config()` are non-functional on APQ8060. |
| **RAW-through-PIX "Y-plane hack"** (`vfe31_raw_pix_mode`) | partial — `rawpix1280` reportedly works as a 1280-byte capture trick (per MEMORY), but it is a debug hack feeding RAW bytes through the YUV DEMUX. | **DROP from v1.** Not a real format; produces mislabeled data. Pure debug instrumentation. |
| **Testgen pattern generator** (`vfe31_use_testgen`, `vfe31_testgen_pixel_dims`) | **NO** — driver's own comment (line ~2115): "testgen hardware block does not exist" on APQ8060; webOS only had a placeholder. | **DROP.** Dead code by the author's own admission. |
| **VIDEO line + recording state machine** (`vfe31_recording_state`, WM1/WM5) | speculative — driver notes VIDEO currently *reuses* PIX WMs; second output path not validated. | **DROP for v1** (single PIX output). The whole `enum vfe31_rec_state` + REG_UPDATE-driven enable/disable is a port of Samsung code that is not exercised on a single-output bring-up. |
| **ZSL line + ZSL state machine** (`vfe31_zsl_state`, WM2/WM6, VFE_LINE_ZSL) | speculative — no legacy ZSL command exists (legacy has LIVESHOT, not ZSL); recent commits still fixing ZSL UB/CbCr sizing. | **DROP for v1.** Not validated, no working capture proof. |
| **Dummy "bit-bucket" buffer** (`dummy_buf_*`, all 7 WMs pointed at it on reset) | workaround — driver comment: "webOS didn't use dummy buffers". | Acceptable as a safety net but should be justified in commit log; ideally replace with proper WM-disable (the webOS approach) for upstream cleanliness. |
| **Software-EOF / software-SOF** (`software_eof_enable`, `vfe_trigger_software_sof`) | workaround for "missing MIPI FE packet" symptom; reports show it triggers but doesn't fix capture (no pixel data). | **DROP / gate.** Adds a framework op (`vfe_trigger_software_sof`) and a global in `camss-vfe.c` that no other backend uses. Don't upstream a workaround that doesn't work. |
| **11 module parameters** (8 in vfe-3-1.c: axi_output_mode, xbar_cfg1, raw_pix_mode, dump_wm_regs, force_422, irq_comp_mask, rdi_efs_cfg, rdi_force_16bpp; 3 in vfe.c: use_testgen, testgen_pixel_dims, software_eof_enable) | debug knobs | **REMOVE all.** Upstream maintainers reject `module_param` tuning knobs. Pick the one correct value (the webOS-verified one) and hardcode it. `dump_wm_regs` → drop or `dev_dbg`. |

**Framework intrusions to note** (`camss-vfe.h`): the port added `VFE_LINE_VIDEO`, `VFE_LINE_ZSL`, `VFE_LINE_NUM_MAX=6`, and two new `vfe_hw_ops` callbacks (`enable_pending_camif`, `vfe_cleanup`) plus `vfe_trigger_software_sof`. These touch shared headers used by every other SoC's backend. For upstream, `enable_pending_camif`/`vfe_cleanup` may be defensible (deferred-CAMIF is a real MSM8660 quirk) but VIDEO/ZSL line additions and software-SOF should be reverted to keep the shared structs clean.

**Thread-safety red flag:** `vfe31_recording_state` and `vfe31_zsl_state` are **file-scope `static` globals**, not per-`vfe_device` state. Harmless with one VFE instance, but it is exactly the kind of non-reentrant global an upstream reviewer will reject.

---

## 3. Capture Modes Actually Wired

Formats exposed (`camss-vfe.c`):
- **PIX table** (`formats_pix_vfe31`): NV12, NV21, NV16, NV61 (from UYVY/VYUY/YUYV/YVYU 1X16 mbus) + passthrough UYVY/VYUY/YUYV/YVYU.
- **RDI table** (`formats_rdi_vfe31`): NV12/NV16 (RAW-through-PIX), UYVY/VYUY/YUYV/YVYU 16bpp, RAW8 Bayer (SBGGR8/SGBRG8/SGRBG8/SRGGB8), RAW10 packed Bayer.

What works on device (reports + MEMORY):
- **WORKS:** YUV422 PIX path. Sensor UYVY (16bpp) → DEMUX → NV12 (4:2:0, scaler does 480→240) or NV16 (4:2:2). This is the validated, real capture path.
- **WORKS (hack):** `rawpix1280` — 1280-wide RAW-through-PIX.
- **DOES NOT WORK:** Pure RDI RAW Bayer (RAW8/RAW10 via AXI=0x60). Silicon CAMIF data-type filtering blocks it. The RAW8/RAW10 entries in `formats_rdi_vfe31` are therefore **advertised but non-functional** — misleading to userspace.
- **Resolutions:** 1280x1024 (Context B) PIX works; 640x480 (Context A) works but exhibits a documented frame-drift artifact treated as a HW limit (`project_vfe31_dma_architecture` / `vfe31-640x480-drift`).

**v1 recommendation:** Expose only NV12/NV16 (+ raw UYVY passthrough) on the PIX line. Remove the RAW8/RAW10 Bayer RDI entries until the RDI path is proven (it can't be on this silicon), or they will fail VIDIOC_STREAMON for any client that selects them.

---

## 4. camss.c MSM8660 Wiring Completeness

`camss_subdev_resources` for 8x60 are present and the platform match is wired:
- `of_device_id`: `qcom,msm8660-camss` and `qcom,apq8060-camss` → `msm8660_resources` ✓
- `version = CAMSS_8x60` ✓ (enum present in `camss.h`)
- `csiphy_res_8x60[2]`, `csid_res_8x60[2]`, `vfe_res_8x60[1]`, `icc_res_8x60` all defined ✓
- VFE clocks: `vfe, vfe_axi, vfe_ahb, vfe_csi0, vfe_csi1, csi_pix, csi_rdi` ✓; clock_rate seeds VFE at 122.88/228.57/266.67 MHz ✓
- `reg = {"vfe0"}`, `interrupt = {"vfe0"}` ✓; CSIPHY reg/irq `csiphy0/1` ✓
- CSID shares CSIPHY reg space (`"csiphy0"`/`"csiphy1"`), **no CSID IRQ** (`.interrupt = {}`) — correct for the unified MSM8660 block, documented in comments.
- `has_pd` deliberately NOT set; single power-domain attached at platform probe (`VFE_GDSC`).

**Gaps / TODOs:**
- **`csi_pix`/`csi_rdi` clocks contested.** `CAMERA_STATUS_SUMMARY.md` flags that webOS does **not** use these (MISC_CC_REG=0) and they may be MSM8960+ artifacts. They are listed in both DT and `vfe_res_8x60`. For upstream, resolve whether they are genuinely required or vestigial.
- **No IOMMU wiring.** A `vfe_iommu` node exists in `qcom-msm8660.dtsi`, but the `camss@04500000` node has **no `iommus` property** and VFE buffers run on physical/CMA addresses. This matches MEMORY (VIDC/VFE "no IOMMU"). Acceptable for bring-up but should be a documented limitation.
- **`line_num = 6`** to cover PIX(3)/VIDEO(4)/ZSL(5). If VIDEO/ZSL are dropped (§2), this should drop to 4 (RDI0-2 + PIX).
- Comment noise: several multi-paragraph "webOS says / Samsung says" comments embedded in the resource table — fine internally, trim for upstream.

No hard stubs (no `return -ENOSYS`/`/* TODO */` in the hot path); wiring is structurally complete and matches the DT.

---

## 5. Kconfig / Makefile

- **Makefile:** `camss-vfe-3-1.o` is in `qcom-camss-objs` ✓. Also pulls in `camss-csid-8x60.o` and `camss-csiphy-8x60.o` (the new MSM8660 backends) ✓.
- **Kconfig:** There is **no per-backend symbol**. Everything builds under `CONFIG_VIDEO_QCOM_CAMSS`. This is consistent with how upstream camss handles all other VFE versions (no `VIDEO_QCOM_CAMSS_VFE_3_1` symbol exists or is expected). **No new Kconfig symbol needed** — but it does mean vfe-3-1 has **no compile-out option** and is always built when camss is enabled. That's the existing upstream convention, so it's fine.
- **Sensor:** `CONFIG_VIDEO_MT9M113` exists in `drivers/media/i2c/Kconfig` and `mt9m113.o` is wired in the i2c Makefile ✓. (Note both `mt9m113.c` and an older `mt9m114.c` exist in-tree; the DT binds `aptina,mt9m113` → `mt9m113.c`. The stray `mt9m114.c` should be removed if unused to avoid confusion — earlier status reports reference `mt9m114.c` as the sensor driver, now superseded.)

**Verdict:** Build wiring is correct and follows upstream camss conventions. No new Kconfig symbol required.

---

## 6. Device-Tree Binding Readiness

DT nodes present and well-formed:
- `camss@04500000` in `qcom-msm8660.dtsi`: `compatible = "qcom,msm8660-camss"`, 3 reg ranges (vfe0 + csiphy0/1), 3 IRQs, full clock list, `assigned-clock-rates`, `interconnects` (`vfe-mem`), `power-domains = <&mmcc VFE_GDSC>`, `ports` with CSI1 endpoint. `status = "disabled"` in SoC dtsi, enabled in board.
- Board (`tenderloin-common.dtsi`): `mt9m113@<addr>` `compatible = "aptina,mt9m113"` with endpoint → `camss_csi1_ep`; CSI1 endpoint wired back. Camera powerdown GPIO handled by sensor driver.

Binding docs:
- **`aptina,mt9m113.yaml` EXISTS** in `Documentation/devicetree/bindings/media/i2c/` ✓ — sensor binding ready.
- **NO `qcom,msm8660-camss.yaml` / `qcom,apq8060-camss.yaml`** exists. The closest documented binding is `qcom,msm8916-camss.yaml`. **A new binding doc is REQUIRED for upstream**, and it must describe the MSM8660-specific shape: unified CSIPHY+CSID reg space (only `vfe0`/`csiphy0`/`csiphy1` reg-names, no separate csid reg, no CSID IRQ), the `csi_pix`/`csi_rdi` clocks (pending §4 resolution), and the single `VFE_GDSC` power domain.

Custom/undocumented properties to formalize in the new binding: the reg-name set, clock-name set (esp. the `csi_pix`/`csi_rdi`/`csiN_phy` names), interrupt-name set, and the `interconnects`/`interconnect-names = "vfe-mem"` entry.

---

## Bottom-Line Recommendations for Upstream v1

1. **Ship the working PIX/YUV path only.** NV12 + NV16 (+ UYVY passthrough). Real, validated, on-device-proven.
2. **Strip the over-build:** remove RDI raw (0x60), RAW-through-PIX, testgen, VIDEO+ZSL lines & state machines, software-EOF/SOF. All are non-functional or unvalidated on APQ8060.
3. **Remove all 11 module parameters;** hardcode the webOS-verified values.
4. **De-globalize** `vfe31_recording_state`/`vfe31_zsl_state` (move per-device or delete with the feature).
5. **Trim the shared-header intrusions** (`VFE_LINE_VIDEO/ZSL`, `vfe_trigger_software_sof`) unless kept-features genuinely need them.
6. **Drop RAW8/RAW10 Bayer from the advertised format list** until RDI is proven (it can't be on this CAMIF).
7. **Resolve `csi_pix`/`csi_rdi` clock necessity.**
8. **Write `qcom,msm8660-camss.yaml`** binding doc (sensor binding already exists).
9. Consider replacing the dummy bit-bucket buffer with proper WM-disable (webOS approach).
