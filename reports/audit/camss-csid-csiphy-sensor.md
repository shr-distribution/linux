# CAMSS CSID / CSIPHY / MT9M113 audit — frame-sync focus

Scope: MSM8660/APQ8060 mainline camss MIPI RX path vs webOS 2.6.35 vendor driver,
with focus on the unsolved 640x480 vertical-drift bug (rolls ~30 lines/frame;
1280x1024 stable). Buffer/ping-pong selection is already proven correct
(`vfe31_wm_done` MATCHes every frame), so the drift is an **input-side
CAMIF frame-timing** problem, not a write-master addressing problem.

Reference register dumps used:
- `reports/webos-preview-mode-dump.txt` — this is the **640x480 preview** dump
  (WINDOW_WIDTH 0x01E00500 = height 480 << 16 | 1280 *bytes* = 640px YUV422)
- `reports/webos-video-mode-dump.txt`

## Severity table

| # | Area | Finding | Rank |
|---|------|---------|------|
| 1 | MT9M113 context re-apply | `mt9m113_configure_sensor_context()` re-writes ROW/COL start/end + READ_MODE on every stream-on but **omits FRAME_LENGTH (0x271F=814) and LINE_LENGTH_PCK (0x2721=1228)** which set the vertical/horizontal blanking. Legacy never re-writes context regs at all (writes them once in init table, then only issues SEQ_CMD). | **FRAME-SYNC-SUSPECT (lead)** |
| 2 | MT9M113 standby/refresh churn | Mainline does a heavy per-stream dance: write STANDBY 0x0028 + 50ms, MCU health read, clear stale SEQ_CMD, ctrl_handler_setup, double-buffer suspend/resume, **REFRESH**, then RUN. Legacy `mt9m113_set_sensor_mode` for preview is minimal: SEQ_CAP_MODE=0x0030, RUN, AE table. Extra REFRESH/standby cycles can leave AE/exposure re-converging → integration time ≠ frame period → vertical roll. | **FRAME-SYNC-SUSPECT** |
| 3 | CSIPHY cross-layer VFE poke | `csiphy_8x60_isr()` reaches into `csiphy->camss->vfe[0]` and calls `vfe_trigger_software_reg_update()` + `vfe_trigger_software_sof()` on a guessed frame boundary (FRAME_START bit, else undocumented BIT(22), else 200µs SOT gap heuristic). Gated by `software_sof_enable` (default false). | **DISCREPANCY / submission-blocker** |
| 4 | CSIPHY settle count | `timer_clk_rate` is 0 on 8x60 (no timer clock wired — see camss.c csiphy_res_8x60), so the settle-count formula branch never runs; falls back to `MSM8660_DEFAULT_SETTLE_CNT=0x14`. That equals legacy `settle_cnt=0x14`. Correct, but the elaborate formula + module params are dead/debug code. | OK (functionally) / DISCREPANCY (dead code) |
| 5 | CSID | `csid_8x60_*` are all stubs/no-ops (no CID, no SOF/EOF, pass-through). This is correct for the unified 8x60 CSIPHY+CSID — webOS configures nothing there either. The CSID does NOT and cannot generate SOF/EOF. | OK |
| 6 | CSIPHY register sequence | `csiphy_8x60_lanes_enable` matches legacy `msm_camio_enable`+`msm_camio_csi_config` byte-for-byte: PHY_CONTROL=0x4, SW_RST, PROTOCOL_CONTROL (LPHC|DECODE_ID|ECC|data_fmt), D0-3_CONTROL2 (settle<<24|0x0F<<16|LP_REC|ERR_SOT), CL_CONTROL=0x0F000004, CAMERA_CNTL=0xe404. data_format=0 (8-bit) for YUV matches `CSI_8BIT`. | OK |
| 7 | CAMIF register config | Mainline VFE-3-1 Step-3 writes EFS_CFG=0x40, FRAME_CFG untouched(=0), WINDOW_WIDTH=(h<<16)|width_bytes, WINDOW_HEIGHT=width_bytes-1, SUBSAMPLE_0=h-1, SUBSAMPLE_1=0xFFFFFFFF, EPOCH_CFG untouched(=0). **Byte-identical to the webOS 640 preview dump.** So CAMIF *register* state is not the cause. | OK |
| 8 | CSIPHY IRQ mask | Mainline sets `MIPI_INTERRUPT_MASK=0xFFFFFFFF`; legacy used 0xFFF7F3FF (masks ID_ERROR[19], DATA_CMM_ERR[11], CLK_CMM_ERR[10]). Enabling those "de-featured" error bits can flood/keep the CSIPHY IRQ asserted and steal CPU at exactly the rate that matters for frame timing. | DISCREPANCY |
| 9 | CSIPHY debug/calibration cruft | `calibration_mode` (0/1/2), `ecc_disable`, `hs_term_imp_override`, `debug_poll` workqueue with static globals, "per Gemini AI analysis" comments. Static `debug_poll_*` globals are not multi-instance safe. | DISCREPANCY / submission-blocker |
| 10 | Pixel clock / link freq | CSI src=384MHz (matches webOS msm_io_8x60). Legacy set CSI0_CLK=153.6MHz (CAMIO_CSI0_CLK) — mainline does not set a 153.6MHz rate anywhere; only 384MHz src. Sensor PLL identical (0x2145/0x0114/0x00F1...). MT9M113 link 96MHz. Not obviously wrong but the 153.6MHz CSI bit-clock rate from legacy is not reproduced. | FRAME-SYNC-SUSPECT (secondary) |
| 11 | Pipeline completeness | CSIPHY→CSID→VFE link is complete for PIX path (640 YUV uses AXI=0x01 PIX path, not RDI). SOF delivered to all lines; REG_UPDATE drives WM-enable state machines; ping-pong via COMPOSITE_DONE N+2. Functional. | OK |

## Frame-sync root-cause candidates (ranked)

The MT9M113 (R0x3404 short-packet disabled by default; `mt9m113_skip_short_pkt=1`)
sends **no MIPI Frame-Start/Frame-End short packets**. With FRAME_CFG=0 and EFS_CFG=0x40
the VFE CAMIF runs in **APS mode and counts lines internally** to find the frame
boundary. webOS got a stable 640 boundary with the *exact same* CAMIF register set,
so the boundary the CAMIF derives must depend on the **sensor-side line/frame
geometry being correct and stable**. That points the finger at the sensor, not the
CSIPHY/CSID.

1. **(LEAD) Missing FRAME_LENGTH_A / LINE_LENGTH_PCK_A re-apply on 640 (Context A).**
   `drivers/media/i2c/mt9m113.c:451-528` `mt9m113_configure_sensor_context()` re-writes
   ROW_START/COL_START/ROW_END/COL_END/ROW_SPEED/READ_MODE for the chosen context but
   does NOT re-write `0x271F MODE_SENSOR_FRAME_LENGTH_A (=0x032E/814)` nor
   `0x2721 MODE_SENSOR_LINE_LENGTH_PCK_A (=0x04CC/1228)`. These define vertical and
   horizontal blanking and therefore the **frame period** the CAMIF line-counter must
   lock to. The init table (`mt9m113.c:695-698`) writes them once, but Context B's
   FRAME_LENGTH_B/LINE_LENGTH_B (0x0559/0x0722) and the per-stream
   STANDBY/REFRESH/RUN/CAPTURE churn (esp. the "enter preview then CAPTURE" path for B,
   `mt9m113.c:1729-1804`) can leave the MCU's *live* Context-A blanking stale after a
   B→A switch. A frame period that is ~30 lines off from what CAMIF integrated against
   produces exactly the observed "rolls ~30 lines/frame". 1280 (Context B) is the
   "freshly programmed" path so it stays correct → matches the symptom asymmetry.
   webOS never switches blanking out from under Context A because it never re-writes
   context geometry at runtime at all.
   **Fix: add FRAME_LENGTH and LINE_LENGTH_PCK to `configure_sensor_context()` for both
   contexts (814/1228 for A, 1369/1826 for B), or stop re-writing context geometry and
   match webOS (write once in init, only SEQ_CMD at runtime).**

2. **AE / integration-time re-convergence per stream-on (#2).** The preview-mode
   readback shows IRQ activity and ping-pong toggling but the *content* rolls. If the
   sensor's coarse/fine integration time is being reprogrammed (AE table at
   `mt9m113.c:1308-1326`) without the matching FRAME_LENGTH, the exposure window can
   exceed/undershoot the frame period and the readout start row walks every frame —
   indistinguishable from a frame-lock failure. Legacy preview keeps AE simple and
   never touches FRAME_LENGTH.

3. **CSIPHY IRQ mask 0xFFFFFFFF (#8).** Enabling the DATA_CMM_ERR[11]/CLK_CMM_ERR[10]/
   ID_ERROR[19] bits that webOS deliberately masked makes the shared CSIPHY/CSID IRQ
   fire continuously on the noisy 640 binned link (the driver's own comments note ~50%
   ECC and "BIT(11) is the main activity"). Sustained IRQ pressure on this SoC has a
   history of disturbing tightly-timed DMA on this device. Restore webOS 0xFFF7F3FF.

4. **CSI bit-clock 153.6MHz not reproduced (#10).** Legacy explicitly set
   `CAMIO_CSI0_CLK = 153_600_000`. Mainline only sets `csi0_src=384MHz` and leaves the
   leaf `csi0`/`csi0_phy` rates at 0 (camss.c:114-122). If the actual CSI receive
   bit-clock differs from webOS, per-line sampling phase can slip across the binned
   frame. Lower priority because 1280 would likely also be affected, but worth
   confirming the leaf clock rates against webOS.

Items explicitly ruled OUT as the cause: CSID config (#5, correct stub), CAMIF register
values (#7, byte-identical to working webOS 640 dump), CSIPHY settle/PHY sequence (#6,
matches legacy), EPOCH_CFG (untouched =0 in both).

## Mainline-submission compliance issues (these files)

- **Cross-layer VFE poke from CSIPHY (#3):** `camss-csiphy-8x60.c` includes `camss-vfe.h`
  and calls `vfe_trigger_software_sof/reg_update/enable_pending_camif` directly on
  `csiphy->camss->vfe[0]`. CSIPHY must not drive VFE — unacceptable upstream. Frame sync
  must come from the HW CAMIF SOF (it does, by default — this path is `software_sof_enable`
  off). Delete the software-SOF path entirely.
- **Debug module params / "Gemini AI" comments / debug_poll globals (#9):** `calibration_mode`,
  `ecc_disable` (exported global), `hs_term_imp_override`, `settle_cnt_override`,
  `debug_poll_*` static globals (not multi-instance safe), and dozens of `dev_info`
  per-register logs. All must be stripped for submission; keep only the
  webOS-faithful Mode-2 (no-poll) calibration as the single code path.
- **`csid_8x60_hw_version` returns fake 0x8060**, `csid_8x60_reset` just completes —
  acceptable as documented stubs but should be commented as "unified with CSIPHY,
  no separate CSID HW" (already is).
- **MT9M113 per-stream STANDBY/REFRESH/MCU-recovery churn (#1/#2):** the runtime
  re-config is far heavier than any reference driver and is the most likely functional
  regression source; reduce to the webOS minimal sequence.

## Key file:line references

- `drivers/media/i2c/mt9m113.c:451-528` — `configure_sensor_context()` omits 0x271F/0x2721
- `drivers/media/i2c/mt9m113.c:695-698` / `:719-722` — init-table FRAME/LINE length A & B
- `drivers/media/i2c/mt9m113.c:49-51` — `mt9m113_skip_short_pkt=1` (no MIPI FS/FE packets)
- `drivers/media/i2c/mt9m113.c:1255-1261` — short-pkt enable only if `!skip_short_pkt`
- `drivers/media/i2c/mt9m113.c:1331-1804` — start_streaming standby/REFRESH/RUN/CAPTURE churn
- `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c:929-1045` — ISR cross-layer VFE poke + BIT(22)/200µs heuristic
- `camss-csiphy-8x60.c:669-671` — IRQ_MASK=0xFFFFFFFF (vs legacy 0xFFF7F3FF)
- `camss-csiphy-8x60.c:389-409,500-611` — PHY/cal sequence (matches legacy)
- `camss-csid-8x60.c:60-180` — CSID stubs (correct, no SOF/EOF)
- `camss-vfe-3-1.c:5285-5396` — CAMIF EFS/FRAME/WINDOW/SUBSAMPLE config (matches webOS 640 dump)
- `camss-vfe-3-1.c:3012-3103` — REG_UPDATE + CAMIF_SOF ISR handling (HW frame sync works)
- `camss.c:45-151` — 8x60 CSIPHY/CSID resource wiring (csi src 384MHz, no timer clk, leaf rates 0)
- legacy `msm_io_vfe31.c:777-868` `msm_camio_csi_config` — CSI ctrl reference
- legacy `mt9m113_reg.c:73-98` — FRAME_LENGTH_A=0x032E/LINE_A=0x04CC, FRAME_LENGTH_B=0x0559/LINE_B=0x0722
- legacy `mt9m113.c:536-659` `mt9m113_set_sensor_mode` — minimal preview/snapshot sequence
- webOS `reports/webos-preview-mode-dump.txt:74-100` — 640 preview CAMIF (CAMIF_STATUS=0)
