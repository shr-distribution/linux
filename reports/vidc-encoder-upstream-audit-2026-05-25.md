# VIDC H.264 Encoder — Pre-Upstream Audit (2026-05-25)

Audit of `drivers/media/platform/qcom/vidc/` encoder paths against the legacy
webOS 2.6.35 1080p-DDL reference (`webos-linux-kernel-touchpad/drivers/video/msm_pe/vidc/1080p/`).
Four parallel audits: buffers/addressing, protocol/IRQ/SHM, register values, upstream readiness.

Status: encoder is **functionally working + verified** (real-image round-trip ~50 dB PSNR,
10× stability, bit-deterministic) but is **bring-up-quality code**, not yet upstream-ready.

---

## Confirmed CORRECT vs legacy
- Address shift `VIDC_ADDR_SHIFT=11`; all encoder address writes are fw-relative `(dma-fw_dma_addr)>>11`.
- Work-buffer size formulas (mv/colz/md/pred/nbor) — identical to legacy `ddl_calc` (CABAC-safe superset).
- Work-buffer register offsets (0x600/0x604/0x608/0x610/0x720/0x740) — match.
- `mb_info=0` disabled — matches working webOS config.
- FRAME_DATA submit register sequence + ordering — matches `vidc_1080p_encode_frame_start_ch0`.
- Response dispatch: cmd5 FRAME_DONE branch-on-decoder, cmd7 ENC_COMPLETE=EOS — matches legacy.
- RISC2HOST/INTERRUPT/RETURNED_CH_INST_ID ack handshake + ordering — matches.
- Opcodes (FRAME_DATA 0x20000, SEQ_HEADER 0x10000), VOP timing res=fps*2, SEQ_HEADER size reg
  (0x2004=REG_845544), byte[6]=0xC0 patch index — all match.
- Profile=Baseline(2), no B-frames, PIC_TYPE_USE bit, frame format TILE(3), init-RC — match.
- Leak/locking: alloc error paths unwind; seq_hdr freed once; close clears curr_inst before
  cancel_work_sync (no UAF).

---

## FIXED THIS ROUND
- **[HIGH] Recon register offsets** (commit ad7fe2aa5a2a). Were fabricated `0x480+i*8`
  (offset 0x480 absent from HW map); real regs are non-contiguous
  0x61c/0x700 0x710/0x708 0x620/0x704 0x714/0x70c. Recon pointers were never actually
  programmed; firmware default was benign (hence 50 dB worked) but incorrect/fragile.
  NEEDS: rebuild + 10× retest to confirm no regression.

---

## OPEN — correctness/robustness (functional but divergent)
- **[HIGH] Per-frame SHM frame_tag (0x0004)** never written → input timestamp/tag not passed
  through; legacy sets it every frame (`ddl_vidc_encode_frame_run`) and reads it back.
- **[HIGH] Per-frame VOP frame_delta (0x0030)** written once at init w/ delta=0, never per-frame;
  legacy refreshes it each frame → affects RC pacing/PTS.
- **[HIGH] H.264 entropy register never written** — baseline needs CAVLC; relies on FW default.
  (Empirically OK — decoded stream was valid baseline — but should be explicit.)
- **[HIGH] Loop-filter (deblock) config never written** — legacy always writes db_config+offsets.
- **[MED] RC config divergence**: mb_rc bit off (legacy on for H264), reaction_coeff 0x14 vs
  legacy VBR default 0x1f4, frame_qp 26 vs 20, qp_max 40 vs 51.
- **[MED] Forced-IDR** not wired: CH0_INTRA_FRAME always 0; no FORCE_KEY_FRAME control.
- **[MED] Keyframe flag**: encoded frame type (reg 0x2010) not read; CAPTURE buffers never get
  V4L2_BUF_FLAG_KEYFRAME.
- **[LOW]** EXT_CTRL (0x0028) SHM, p_frame_ref_count, multi_slice, field_picture, circular-intra
  not written (rely on FW defaults; legacy writes all explicitly). EDFU (cmd16) no explicit case.
- **[MED]** Recon over-allocation: 4 slots programmed/allocated; baseline legacy uses 2.

---

## OPEN — upstream-readiness BLOCKERS (design work, not cleanup)
- **Debug purge**: ~20 `printk(KERN_EMERG "VIDC:...")` in vidc_enc.c start_streaming + ~30 more
  in vidc_core.c; per-frame/per-streamon `dev_info` spam. Must be deleted/downgraded to dev_dbg.
- **V4L2 controls are inert**: PROFILE/LEVEL/BITRATE registered with NO `s_ctrl` op — values
  hardcoded, controls do nothing. Missing BITRATE_MODE/GOP_SIZE/QP/FORCE_KEY_FRAME/HEADER_MODE.
- **Stateful encoder drain non-conformant**: ENC_CMD STOP doesn't drain; no V4L2_BUF_FLAG_LAST;
  EOS signalled immediately.
- **NV12MT (tiled) is the ONLY input format** — upstream expects linear NV12; needs a linear path
  or explicit maintainer sign-off.
- **SPS/PPS prepend hardcoded** instead of V4L2_CID_MPEG_VIDEO_HEADER_MODE.
- **result_size not clamped** to dst plane size (trusts FW reg → potential OOB on prepend path).
- **Permanent PM pin** ("Strategy 1" keep-resident, pm_runtime_get_noresume never released) +
  firmware recovery-mode (cmd=51) workarounds — maintainer will object; needs proper reset fix.
- **curr_inst** cleared without irqlock in vidc_enc_complete_work (data race vs IRQ reader).
- Magic numbers, webOS-bring-up comments, line-length — checkpatch cleanup.

---

## Suggested upstream roadmap (order)
1. Validate recon fix (rebuild + retest).  2. Per-frame frame_tag + VOP delta.
3. Entropy + loop-filter + RC parity.  4. Debug purge.  5. Real V4L2 control framework (s_ctrl).
6. Stateful drain/EOS/LAST + HEADER_MODE.  7. Linear NV12 input path.  8. PM pin / recovery-mode
resolution.  9. result_size clamp + curr_inst locking.  10. checkpatch/comment cleanup.

---
---

# VIDC H.264 DECODER — Pre-Upstream Audit (2026-05-25)

Same method: three parallel audits (DPB/buffers/addressing, protocol/IRQ/display/SHM,
config + V4L2 readiness) vs legacy 1080p-DDL decode path. Decoder is functionally
working (50/50 frames, fullscreen, repeatable since the cmd=51 keep-resident fix).

## Confirmed CORRECT vs legacy
- **No fabricated-register bug** (unlike encoder recon): all DPB address registers resolve in
  the legacy HWIO map and match the H.264 decode setter:
  LUMA 0x700, CHROMA 0x600, MV 0x780 (+i*4 stride), VERT_NB_MV 0x68c, NB_IP 0x690.
- fw-relative addressing (shift 11) on all DPB/MV/work writes.
- DPB luma/chroma/MV size formulas + work-buffer sizes (16K/32K) — identical to legacy.
- INIT_BUFFERS issuance + register-programming order — matches (decode DOES send INIT_BUFFERS).
- SHM allocated-size offsets (0x64/0x68/0x6c) — match.
- FRAME_DATA decode submit register set (STREAM_ADDR/SIZE/BUF_SIZE, DESC, DPB_RELEASE/CONFIG,
  SHM, CMD_SEQ_NUM, trigger-last) — present and matches.
- IRQ ack ordering; display addr absolute>>11; display_status low-nibble 4-way dispatch
  (decode-only/decode+display/display-only/dpb-empty) honored; cmd5 decoder routing; locking
  (curr_inst cleared before cancel_work_sync) — all correct.
- **Decode CONFIG is upstream-clean**: codec select(0), pcache disable(3), MPEG4/H263 PP filter,
  divx3-res clear, metadata. Legacy programs NO error-concealment/output-order registers for
  decode either — so nothing is missing there.

## OPEN — correctness/robustness
- **[HIGH] DPB_CONFIG flush bit (bit14) missing** — DPB_CONFIG written as count only; legacy ORs
  `dpb_flush<<14` on seek/STOP. Mid-stream seek/flush won't flush the firmware DPB.
- **[HIGH] Per-frame frame_tag SHM (0x0004) never written/read** — input→output timestamp
  correlation broken through B-frame reorder; we just bump sequence_cap.
- **[MED] DPB count has ZERO headroom** — we program exactly min_dpb; legacy always programs
  min_dpb + 4 (often +6/+9). Risk: B-frame/reorder streams can stall or recycle a slot still
  pending display → reference corruption. Fix: `dpb_count = min_dpb_count + 4` (keep 32 clamp).
- **[MED] resl_change bit ignored** — only low nibble of DISPLAY_STATUS read; mid-stream SPS
  resolution change → corrupt output instead of V4L2_EVENT_SOURCE_CHANGE (formats advertise
  DYN_RESOLUTION).
- **[MED] Cropping not applied** — visible-vs-coded rect (SHM CROP_INFO 0x20/0x24) never read;
  CAPTURE covers padded/coded area; G_SELECTION can't report visible rect.
- **[MED] No active EOS drain** — frame_done_work only calls m2m_job_finish; DISPLAY_ONLY/
  DPB_EMPTY frames aren't pumped (M2M needs both queues). VIDC_OP_LAST_FRAME defined but unused.
- **[LOW] start_byte_number SHM (0x18) not reset per frame; decode error/frame-type status
  (0x202c) not read (errored frames copied as good); CMD_SEQ_NUM written before DPB regs
  (benign reorder); metadata_status not read.**

## OPEN — upstream-readiness BLOCKERS (decoder-specific)
- **ZERO V4L2 controls** — `V4L2_CID_MIN_BUFFERS_FOR_CAPTURE` (required) absent; userspace can't
  size CAPTURE queue. queue_setup hardcodes max(n,8) instead of deriving from min_dpb.
- **DECODER_CMD START is a no-op; no TRY_DECODER_CMD; no V4L2_EVENT_EOS emitted** — drain
  (STOP→LAST flag→EOS→START) non-conformant.
- **No G_SELECTION/S_SELECTION** — no visible-rect cropping (coded height aligned to 32).
- **In-place mutation of the userspace OUTPUT bitstream buffer** in vidc_dec_seq_header_work_fn
  (memmove/truncate to strip AUD/slices) — illegal in V4L2 model, corrupts DMABUF imports.
  Must use a driver-private scratch copy. (correctness BLOCKER)
- **CAPTURE is NV12MT (deprecated tiled fourcc) only** — needs linear NV12 or modifier-based
  reporting + maintainer sign-off.
- **dev_info per-frame/per-session spam** (~9 sites) + 0xCC SMIPOOL sentinel debug instrumentation
  must be removed. (No KERN_EMERG in vidc_dec.c — better than encoder.)
- **SOURCE_CHANGE re-delivery hack** on subscribe (papers over a GStreamer race) — reviewers
  will flag.
- Minor: colorimetry fields unset, querycap bus_info empty, ctrl_handler leak on open error path,
  seq_header_work not cancelled in stop_streaming (race).

## SHARED with encoder (block both)
Permanent PM pin (fw_pinned keep-resident, never runtime-suspends) + firmware cmd=51
recovery-mode workarounds + IRQ-storm self-disable. Root cause: genpd VED power-collapse
produces a dirty reset. This is the single largest distance-to-upstream item and must be fixed
(proper VED reset) before either codec can be submitted — then the PM pin + recovery paths delete.

## Decoder verdict
Register/config layer is upstream-quality and faithful to legacy (no missing config writes, no
fabricated registers). The V4L2 stateful-decoder uAPI layer is pre-RFC: missing required control,
G_SELECTION cropping, conformant DECODER_CMD/EOS drain; plus one correctness blocker (userspace
buffer mutation). Highest-value safe fix: DPB +4 headroom.

## Combined upstream roadmap (both codecs)
0. [shared] Fix genpd VED reset → delete PM pin + cmd=51 recovery + IRQ-storm guard.
1. Encoder recon fix (done ad7fe2aa5a2a) — validate. Decoder DPB +4 headroom.
2. Per-frame SHM both codecs (frame_tag; enc VOP delta; dec start_byte/flush bit).
3. Enc entropy+loopfilter+RC parity. Dec resl_change→SOURCE_CHANGE, cropping/G_SELECTION.
4. Debug purge both files (~50 KERN_EMERG/dev_info).
5. Real V4L2 control framework (enc s_ctrl + full set; dec MIN_BUFFERS_FOR_CAPTURE + drain/EOS).
6. Dec: move bitstream stripping to scratch copy (correctness blocker).
7. Stateful drain/EOS/LAST + HEADER_MODE (enc) / DECODER_CMD (dec).
8. Linear NV12 path both codecs (tiled-only is an upstream blocker).
9. result_size clamp (enc) + curr_inst locking; misc compliance (colorimetry, bus_info).
10. checkpatch/comment cleanup.
