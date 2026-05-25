# VFE31 Datapath Register/Value/Sequence Accuracy Audit

Mainline file under audit:
`drivers/media/platform/qcom/camss/camss-vfe-3-1.c` (~7280 lines)

Ground truth:
- Legacy: `webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c` / `.h` / `msm_io_vfe31.c`
- Live dumps: `reports/webos-{preview,video,photo-capture,video-recording}-mode-dump.txt`

Authoritative per-WM register layout, decoded from `webos-preview-mode-dump.txt`
(frame = 1280x480, mode 0x01, XBAR 0x1A1B):

| Reg (WMn = base+0x18*n) | WM0 value | Meaning |
|---|---|---|
| 0x058 WR_ADDR_CFG  | `0x0000012F` | UB_start=0, UB_depth=0x12F=303 |
| 0x05C WR_UB_CFG    | `0x002701DF` | wpl_field=0x27=39, height-1=0x1DF=479 |
| 0x060 WR_IMAGE_SIZE| `0x00501DF2` | stride=0x50=80, ((479<<4)\|2) |
| WM4 0x0B8 ADDR_CFG | `0x01300097` | UB_start=0x130=304, depth=0x97=151 |
| WM4 0x0BC UB_CFG   | `0x002700EF` | wpl=39, cbcr_height-1=0xEF=239 |
| WM4 0x0C0 IMAGE    | `0x00500EF2` | stride=80, ((239<<4)\|2) |

Decoded constants: stride field 0x50=80 ⇒ **Y stride = 80*16 = 1280 bytes = width**.
UB wpl field 0x27=39 ⇒ (wpl/8 - 1)=39 ⇒ wpl=320 ⇒ **input_stride = 320*4 = 1280 bytes = width**.
So **webOS computes UB_CFG and IMAGE_SIZE stride from `width`, NOT `width*2`.**

---

## Findings (ranked by severity)

| # | Sev | Area | file:line | Finding |
|---|-----|------|-----------|---------|
| F1 | **BUG** | WM stride math | 5087, 5156, 5215, 5272 (`vfe31_configure_pending_camif`); also 553/680/688 (`vfe31_calc_pix_config`) | Mainline uses `input_stride = width*2` for IMAGE_SIZE and UB_CFG. webOS dump proves stride = **width** (0x50=80⇒1280, wpl 0x27=39⇒1280) for a 1280-wide frame. With width*2 mainline would emit stride field 0xA0=160 and wpl (640/8-1)=79=0x4F — neither matches the dump's 0x50/0x27. This over-sizes the UB allocation per line and mis-times the bus; classic cause of half/wrapped frames. The Y plane DMA actually writes compactly at `width` (mainline even acknowledges this at 589/5185 for the CbCr offset) yet still programs the WM stride at width*2. **Stride should be `width`, not `width*2`.** |
| F2 | **BUG** | WM ADDR_CFG depth (UB alloc) | 5126-5135 (`configure_pending_camif`) vs dump 0x012F=303 | The proportional formula `ub_depth = y_pixels*912/(y_pixels*3) - 1 = 303` happens to land on 303 *for NV12 specifically* (912/3-1). But it is a re-derivation that diverges for NV16 (`*4` ⇒ 227) and for any other geometry, where webOS/Opal HAL use the fixed 912-entry budget split. The webOS value 0x12F=303 is reproduced only by coincidence of the NV12 1.5x factor. The two independent code paths (this inline one and `vfe31_calc_pix_config` at 638-670) compute UB depth with **different formulas** (`*3`/`*4` here vs `*1.5`/`*2` there) and `vfe31_calc_pix_config` is dead for the streaming path — only `configure_pending_camif` runs. Magic-number 912 has no datasheet derivation, only "Opal HAL". |
| F3 | **BUG / sequence** | Global reset | 2330-2337 `vfe31_global_reset` | Mainline issues `GLOBAL_RESET=0x3FF` then a blind `usleep_range(2000,3000)`. Legacy `vfe31_reset()` (msm_vfe31.c:883) issues `VFE_RESET_UPON_RESET_CMD=0x3ff` and **waits for the RESET_ACK IRQ** (STATUS_1 bit22) before `vfe31_set_default_reg_values()`. Mainline applies all defaults after a fixed delay with no ACK check; if reset is slow the CGC/DEMUX/framedrop writes race the reset. Comments (2278-2281) even claim "the reset write hangs / skip it" — contradicted by the code which does write 0x3FF. Stale/contradictory comments. |
| F4 | **DISCREPANCY** | XBAR_CFG1 value | 806-832 `vfe31_calc_xbar`; default 156; legacy 717 | `vfe31_calc_xbar(pix=true,...)` returns `0x1A1B` (sets out1 byte 0x1A even when only PIX). The **live dump confirms 0x1A1B** (0x044), so the emitted value is correct. BUT the legacy V4L2 path (`msm_vfe31.c:717`) writes `0x1a03`, and mainline's own PIX-mode comment at 5017-5018 claims "0x1A03 correct, 0x1A1B causes Cb/Cr swap" — directly contradicting both the dump and `vfe31_calc_xbar`. The routing nibble derivations in the header (0x1B = Y→ch0+ch1, 0x1A=Y→ch1+CbCr→ch0) are guesses ("function unknown", 1456-1462). Value OK vs dump; documentation self-contradictory. |
| F5 | **DISCREPANCY** | Per-WM reconfig in WM-enable path | `vfe31_wm_enable`→`vfe31_configure_pending_camif` (5817, 4894) | The entire datapath (BUS_CFG, AXI mode, XBAR, all WM regs, CAMIF window/EFS/subsample, CORE_CFG, IRQ masks, CAMIF start) is programmed lazily inside `vfe31_wm_enable()` on the first WM enable. webOS programs AXI/WM via the one-shot `V31_AXI_OUT_CFG` memcpy (msm_io_vfe31.c table, offset 0x38 len 188) and CAMIF via `V31_CAMIF_CFG` once, then `vfe31_start_common()` writes IRQ_MASK_0 + REG_UPDATE + CAMIF_CMD. Folding all of this into the WM-enable callback is fragile ordering not present in legacy; correctness depends on `camif_pending` bookkeeping. Functionally reaches the same registers but is a large structural divergence. |
| F6 | MAGIC-NUMBER | BUS_CMD reload | 355 `vfe31_get_bus_cmd_reload`→0x7FFF; written 2422 | webOS writes `0x7FFF` to BUS_CMD (msm_vfe31.c:2350), dump shows readback 0x3FFF (bit14 self-clears). Matches legacy. Value correct; the "bit14 pingpong reload" rationale (2408-2418) is plausible but undocumented. **OK, matches legacy.** |
| F7 | OK-but-note | AXI_OUT_MODE | 564, 4935, 5043-5046; dump 0x040=0x01 | Default 0x01 emitted for PIX; matches dump and legacy OUTPUT_1_AND_3. RDI uses 0x60, ZSL path writes `0x101` (5043) which is NOT a webOS value (webOS never used ZSL) — speculative. 0x200/0x60 documented from legacy enum. PIX value OK. |
| F8 | OK-but-note | BUS_CFG | 345-348 →0x02AAA771; written 4986/5005/5042; dump 0x03C=0x02AAA771 | Exact match to dump and legacy. RAW-bpp variants (0x2aaa775/779) are HTC-derived, not webOS, but unused for PIX. **OK.** |
| F9 | OK | CAMIF EFS_CFG (0x1E4) | 5315; dump 0x1E4=0x40 | Writes BIT(6)=0x40 (`CAMIF2VFE`). Matches dump exactly. webOS supplies this via userspace V31_CAMIF_CFG; value identical. **OK.** |
| F10 | DISCREPANCY / doc | CAMIF window register semantics | header 1756-1764 vs code 5366-5377 vs dump | Header labels 0x1EC = "WINDOW_WIDTH lastPixel\|firstPixel" and 0x1E8 = "FRAME_CFG frame dims (linesPerFrame<<16\|pixelsPerLine)". Dump shows the **frame dims live at 0x1EC** (`0x01E00500` = 480<<16\|1280) and 0x1E8 = 0. Code at 5366 correctly writes `(height<<16)\|width_bytes` to 0x1EC and `width_bytes-1` to 0x1F0 — matching the dump. So **emitted values are correct** but the register-map comments mislabel 0x1E8/0x1EC/0x1F0. FRAME_CFG(0x1E8) left 0 is correct vs dump. |
| F11 | OK | CAMIF SUBSAMPLE_0/1 | 5384-5392; dump 0x1F4=0x1DF, 0x1F8=0xFFFFFFFF | 0x1F4 = height-1 (0x1DF=479), 0x1F8 = 0xFFFFFFFF. Both match dump. **OK.** EPOCH(0x1FC)=0 and RAW_CROP(0x200)=0x3FFF3FFF in dump are not written by mainline (left at reset value 0) — 0x200 mismatch (dump 0x3FFF3FFF vs reset 0). See F18. |
| F12 | OK | MODULE_CFG | 1283 `0x01C00C0C`; written 4523; dump 0x010=0x01C00C0C | Exact match to dump (CLAUDE.md cites same). **OK.** Note: `vfe31_global_reset` does NOT write MODULE_CFG (comment at 2290 claims "MODULE_CFG=0x3FF" but no such write exists — stale comment). MODULE_CFG only set in `vfe31_set_demux_cfg`. |
| F13 | OK | CORE_CFG | 5452-5457; dump 0x014=0x46 | UYVY → 0x6 (CBYCRY) \| BIT(6) = 0x46. Matches dump and CLAUDE.md. **OK.** |
| F14 | OK | DEMUX_CFG / EVEN / ODD | 4526-4635; dump 0x284=0x03, 0x290/0x294=0xC9CA | DEMUX_CFG period=3, EVEN=ODD=0xC9CA for UYVY. Matches dump, legacy, CLAUDE.md, cross-vendor. Gains 0x800080 match dump. **OK.** |
| F15 | OK | IRQ_MASK_0 at start | 5464 comment + actual write; legacy 993 `0x00EFE021`; dump 0x01C=0x00EFE021 | webOS start mask 0x00EFE021. Confirm the actual streaming write uses this exact value (see F19). Dump matches. |
| F16 | OK | IRQ_MASK_1 / COMP_MASK | dump 0x020=0x00400000 (RESET_ACK bit22), 0x034=0x00220011 | Mainline halt path uses RESET_ACK bit22 (3172). COMP mask 0x00220011 defined (296) and emitted via shadow. Matches dump. **OK.** |
| F17 | OK | Ping-pong bit convention | 2641 (`active_index=(pp>>wm)&1`, return `buf[!active_index]`) vs legacy macro 2468-2470 | Legacy: `(pp & (1<<chn))==0 ? PONG_addr : PING_addr` (bit0⇒completed buffer is PONG). Mainline: bit0⇒active_index=0⇒return buf[1]=PONG. **Conventions match.** Note mainline F17b: for wm_num==2 it switches to the CbCr WM's PP bit (2637-2640) — a heuristic ("Y bit never toggles") with no legacy basis; legacy always uses each channel's own bit. |
| F18 | DISCREPANCY | RAW_CROP_WIDTH (0x200) | not written by mainline; dump 0x200=0x3FFF3FFF | webOS leaves 0x200 at 0x3FFF3FFF (no crop). Mainline never writes 0x200, so it stays 0 after the 0x3FF reset. For PIX this is downstream of FOV/scaler and may be benign, but it differs from the working device. Worth setting 0x3FFF3FFF to match. |
| F19 | DISCREPANCY / sequence | CAMIF start vs REG_UPDATE order | legacy `vfe31_start_common` 1001-1002 | Legacy: write IRQ_MASK_0=0x00EFE021, then **REG_UPDATE_CMD=1, then CAMIF_CMD=1** (REG_UPDATE *before* CAMIF start). Mainline defers WM enable to a REG_UPDATE ISR (`vfe31_pix_wm_pending`, 234) and starts CAMIF inside `configure_pending_camif`; the strict legacy order (REG_UPDATE pulse immediately preceding CAMIF=1, in one place) is not reproduced as a single sequence. Verify the REG_UPDATE precedes CAMIF_CMD on the mainline path. |
| F20 | OK | Halt / stop sequence | 3135 `vfe31_halt` vs legacy `vfe31_stop` 515 | Mainline: MASK=0, CAMIF STOP_IMMEDIATELY(0x2), clear IRQs, IRQ_CMD=1, MASK_1=RESET_ACK, AXI_CMD=HALT, wait completion. Legacy adds: spin on `AXI_STATUS & 0x1` busy-poll *before* clearing halt, then `AXI_HALT_CLEAR`, then `GLOBAL_RESET=VFE_RESET_UPON_STOP_CMD (0x3ef)`. **Mainline omits the AXI_STATUS busy-poll and the post-halt 0x3ef reset-upon-stop.** Mainline relies on a HALT_ACK completion instead (different but defensible). Note: legacy uses 0x3ef (reset-upon-stop, register module preserved) vs reset path 0x3ff — mainline's global_reset always uses 0x3ff. |
| F21 | MAGIC-NUMBER | Scaler phase constants | 4688/4699/4716/4734 `0x00310000`, `0x00320000` | Phase mult 0x310000 (1:1) / 0x320000 (2:1) hardcoded. Dump (video) shows SCALE_Y_V_IMAGE=0x01E001E0 (480/480) consistent with 1:1, but the phase values themselves aren't in the grepped dump lines; they are HTC/Gemini-derived, not verified against a webOS phase dump. FOV_Y=width-1, FOV_CBCR=height-1 match video dump (0x360=0x27F, 0x364=0x1DF). Phase magic numbers unverified. |
| F22 | OK-but-note | CHROMA_SUBS_CFG | 4757-4764; 0x30 (NV12)/0x10 (NV16); dump (preview) shows 0x4F8 present | NV12⇒0x30, NV16⇒0x10. webOS dumps NV12 video so 0x30 expected. Derivation (bit4 enable, bit5 vsub) is HTC-binary, plausible. CHROMA_V_IMAGE = (v_out<<16)|height matches dump pattern (0x01E001E0 style). **OK for NV12.** |
| F23 | OK | hw_version | 2176-2187; dump 0x000=0x00030217 | Read-only; mainline just reads and logs. Value 0x00030217 in dump. No accuracy issue. |

---

## Summary of the load-bearing accuracy problems

1. **F1 (stride = width*2 instead of width)** is the most serious datapath bug:
   the live preview dump's WM0 IMAGE_SIZE stride field 0x50 (=1280) and UB wpl
   0x27 (=1280) prove webOS programs the Y/CbCr WM stride from **width**, while
   mainline uses **width*2** at lines 5087/5156/5215/5272 (and in the parallel
   dead path `vfe31_calc_pix_config`). This is the kind of mis-stride that
   produces wrapped/half frames.

2. **F2 (UB-depth re-derivation)** matches the webOS 0x12F=303 only by NV12
   coincidence; two divergent formulas exist and the cleaner one
   (`vfe31_calc_pix_config`) is not on the live path. The 912-entry budget is a
   HAL-derived magic number.

3. **F3/F19 (reset & start sequencing)** diverge from legacy's IRQ-ACK-gated
   ordering and carry stale, self-contradicting comments (claims of "skip reset"
   / "MODULE_CFG=0x3FF" that the code does not do).

4. **F4 (XBAR) and F10 (CAMIF reg labels)** emit values that *match the live
   dump* (0x1A1B, frame dims at 0x1EC), but the surrounding comments contradict
   the dump (0x1A03 "correct", 0x1E8 "frame dims"). Documentation is unreliable
   even where the code is right.

5. **F18 (RAW_CROP 0x200) and F20 (omitted AXI_STATUS poll + 0x3ef stop-reset)**
   are smaller fidelity gaps vs the known-good webOS sequence.

Items F6-F9, F11-F17, F22-F23 match the dumps/legacy and are correct.
