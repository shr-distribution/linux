// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 JPEG Encoder/Decoder (Gemini) - Hardware abstraction
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * Based on legacy msm_gemini driver from webOS kernel.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>

#include "gemini_hw.h"

/*
 * Diagnostic: FE_INPUT_FORMAT enum sweep. OPAL writes 0x10 (bit 4 set)
 * but its trace was for the camera-VFE-output path which may have used
 * a 10-bit packed format. For userspace mmap'd standard 8-bit NV12, the
 * correct value is unknown — try 0x00, 0x01, 0x02, 0x03, 0x04 with the
 * `gemini.fe_input_format=0xN` kernel command-line parameter.
 */
static unsigned int fe_input_format = 0x10;
module_param_named(fe_input_format, fe_input_format, uint, 0644);
MODULE_PARM_DESC(fe_input_format,
	"FE_INPUT_FORMAT register value (default 0x10 matches OPAL trace; try 0x00..0x07 for 8-bit NV12 enum sweep)");

/*
 * WE_Y/CBCR_THRESHOLD: the OPAL live register-write trace (offline JPEG path
 * on this exact device) shows BOTH thresholds written as 0x016A0190. Mako's
 * [mode][2] table that we initially mined for the layout uses {0x190, 0x1ff}
 * (realtime mode) / {0x16a, 0x1ff} (other mode), which led us to "correct"
 * 0x016A0190 to 0x01FF0190. But OPAL's working offline encode on this very
 * device writes 0x016A0190 directly, so trust the live trace over the mako
 * lookup-table reverse-engineering.
 */
static unsigned int we_y_threshold = 0x016A0190;
module_param_named(we_y_threshold, we_y_threshold, uint, 0644);
MODULE_PARM_DESC(we_y_threshold,
	"WE_Y_THRESHOLD register value (default 0x016A0190 matches OPAL live offline-encode trace)");

static unsigned int we_cbcr_threshold = 0x016A0190;
module_param_named(we_cbcr_threshold, we_cbcr_threshold, uint, 0644);
MODULE_PARM_DESC(we_cbcr_threshold,
	"WE_CBCR_THRESHOLD register value (default 0x016A0190 matches OPAL live offline-encode trace)");

static unsigned int we_y_ub_cfg = 0x01FF0000;
module_param_named(we_y_ub_cfg, we_y_ub_cfg, uint, 0644);
MODULE_PARM_DESC(we_y_ub_cfg,
	"WE_Y_UB_CFG register value (default 0x01FF0000 from OPAL)");

int gemini_hw_reset(void __iomem *base)
{
	/*
	 * Issue the soft reset. The RESET_ACK interrupt fires when the IP
	 * has finished resetting; the caller (gemini_reset() in gemini.c)
	 * arms IRQ_MASK with RESET_ACK before calling us and then waits on
	 * a completion that the IRQ handler signals.
	 */
	writel(GEMINI_RESET_DEFAULT, base + GEMINI_RESET_CMD);

	return 0;
}

void gemini_hw_enable_irq(void __iomem *base, u32 mask)
{
	writel(mask, base + GEMINI_IRQ_MASK);
}

void gemini_hw_disable_irq(void __iomem *base)
{
	writel(GEMINI_IRQ_DISABLE, base + GEMINI_IRQ_MASK);
}

void gemini_hw_clear_irq(void __iomem *base, u32 mask)
{
	writel(mask, base + GEMINI_IRQ_CLEAR);
}

u32 gemini_hw_get_irq_status(void __iomem *base)
{
	return readl(base + GEMINI_IRQ_STATUS);
}

u32 gemini_hw_get_output_size(void __iomem *base)
{
	return readl(base + GEMINI_ENCODE_OUTPUT_SIZE) & 0xFFFFFF;
}

void gemini_hw_set_fe_ping(void __iomem *base, dma_addr_t y_addr,
			   dma_addr_t cbcr_addr, u32 num_mcu_rows)
{
	u32 cfg, rows;

	pr_info("gemini fe_ping: B0 enter rows=%u y=0x%llx cbcr=0x%llx\n",
		num_mcu_rows, (u64)y_addr, (u64)cbcr_addr);

	rows = num_mcu_rows - 1;
	cfg = ((rows << GEMINI_FE_CBCR_MCU_ROWS_SHIFT) &
	       GEMINI_FE_CBCR_MCU_ROWS_MASK) |
	      ((rows << GEMINI_FE_Y_MCU_ROWS_SHIFT) &
	       GEMINI_FE_Y_MCU_ROWS_MASK);
	writel(cfg, base + GEMINI_FE_BUFFER_CFG);
	pr_info("gemini fe_ping: B1 FE_BUFFER_CFG done\n");

	writel(y_addr, base + GEMINI_FE_Y_PING_ADDR);
	pr_info("gemini fe_ping: B2 FE_Y_PING_ADDR done\n");

	writel(cbcr_addr, base + GEMINI_FE_CBCR_PING_ADDR);
	pr_info("gemini fe_ping: B3 FE_CBCR_PING_ADDR done\n");

	/*
	 * Mirror PING -> PONG so the encoder's pong-side reads (if any)
	 * land at the same physical pixel buffer. OPAL's libgemini does
	 * the equivalent of this for single-frame still encodes.
	 */
	writel(y_addr,    base + GEMINI_FE_Y_PONG_ADDR);
	writel(cbcr_addr, base + GEMINI_FE_CBCR_PONG_ADDR);

	writel(GEMINI_FE_CMD_RELOAD, base + GEMINI_FE_CMD);
	pr_info("gemini fe_ping: B4 FE_CMD=RELOAD done\n");
}

/*
 * WE buffer configuration that legacy webOS sets immediately after
 * RESET (in msm_gemini_core_reset → msm_gemini_hw_we_buffer_cfg). These
 * registers must be valid before PIPELINE_CFG arms the offline branch;
 * otherwise the WE engine starts with zero thresholds / no UB sizing
 * and stalls on first write.
 */
void gemini_hw_we_post_reset_cfg(void __iomem *base)
{
	pr_info("gemini post_reset: P0 WE buffer cfg ub=0x%08x y_th=0x%08x cbcr_th=0x%08x\n",
		we_y_ub_cfg, we_y_threshold, we_cbcr_threshold);
	writel(we_y_ub_cfg,       base + GEMINI_WE_Y_UB_CFG);
	writel(we_y_threshold,    base + GEMINI_WE_Y_THRESHOLD);
	writel(we_cbcr_threshold, base + GEMINI_WE_CBCR_THRESHOLD);
	pr_info("gemini post_reset: P1 WE thresholds done\n");
}

void gemini_hw_set_we_ping(void __iomem *base, dma_addr_t addr, u32 len)
{
	pr_info("gemini we_ping: C0 enter addr=0x%llx len=%u\n",
		(u64)addr, len);

	/*
	 * Write CFG (which carries the buffer length) BEFORE ADDR. Legacy
	 * msm_gemini_hw_we_buffer_update (hw_cmd_we_ping_update[]) emits the
	 * CFG write first, then the ADDR write -- the IP latches the address
	 * on CFG-write so writing them in the reverse order can discard the
	 * just-written ADDR.
	 */
	writel(len & GEMINI_WE_BUFFER_LEN_MASK, base + GEMINI_WE_Y_PING_CFG);
	writel(addr, base + GEMINI_WE_Y_PING_ADDR);
	pr_info("gemini we_ping: C1 WE_Y_PING CFG+ADDR done\n");

	/* Mirror to PONG (see set_fe_ping for rationale). */
	writel(len & GEMINI_WE_BUFFER_LEN_MASK, base + GEMINI_WE_Y_PONG_CFG);
	writel(addr, base + GEMINI_WE_Y_PONG_ADDR);
	pr_info("gemini we_ping: C2 WE_Y_PING+PONG done — exiting we_ping\n");
}

void gemini_hw_fe_reload(void __iomem *base)
{
	writel(GEMINI_FE_CMD_RELOAD, base + GEMINI_FE_CMD);
}

void gemini_hw_start_offline(void __iomem *base)
{
	int i;
	u32 status;

	pr_info("gemini start: D0 enter\n");

	/*
	 * The OPAL/Samsung hw_cmd_start template (decoded from the actual
	 * binary `.data` of libqcameralib's bundled libgemini and Samsung's
	 * libgemini.so) contains exactly three writes in this order:
	 *
	 *   IRQ_MASK    = 0xFFFFFFFF   (mask 0xFFFFFFFF)
	 *   START_KICK  = 1            (mask 0x1, offset 0xF0)
	 *   FE_CMD      = 3            (mask 0x3, OFFLINE_CMD_START)
	 *
	 * All three are required. We previously dropped IRQ_MASK widening
	 * and START_KICK chasing the hang, but the actual hang root cause
	 * was the wrong PIPELINE_CFG value (0x020065FB instead of 0x038061FB).
	 * Now that PIPELINE_CFG is correct, restore the OPAL-faithful start
	 * sequence.
	 */
	writel(GEMINI_IRQ_ALL, base + GEMINI_IRQ_MASK);
	pr_info("gemini start: D1 IRQ_MASK=0xFFFFFFFF done\n");

	writel(1, base + GEMINI_START_KICK);
	pr_info("gemini start: D2 START_KICK=1 done\n");

	writel(GEMINI_OFFLINE_CMD_START, base + GEMINI_FE_CMD);
	pr_info("gemini start: D3 FE_CMD=3 done\n");

	pr_info("gemini_hw_start: regs after start:\n"
		"  PIPELINE_CFG=0x%08x\n"
		"  FE_INPUT_FMT=0x%08x  FE_DIMS=0x%08x  FE_PIPELINE=0x%08x\n"
		"  OP_ENC_MODE=0x%08x  OP_GEOM[0..3]=0x%08x 0x%08x 0x%08x 0x%08x\n"
		"  OP_FORMAT_MAGIC=0x%08x\n"
		"  WE_CFG=0x%08x  WE_Y_PING_ADDR=0x%08x  WE_Y_PING_CFG=0x%08x  WE_UB_CFG=0x%08x\n"
		"  FE_BUFFER_CFG=0x%08x  FE_Y_PING=0x%08x  FE_CBCR_PING=0x%08x\n"
		"  IRQ_MASK=0x%08x  IRQ_STATUS=0x%08x  START_KICK=0x%08x  FE_CMD=0x%08x\n",
		readl(base + GEMINI_PIPELINE_CFG),
		readl(base + GEMINI_FE_INPUT_FORMAT),
		readl(base + GEMINI_FE_DIMS),
		readl(base + GEMINI_FE_PIPELINE_MODE),
		readl(base + GEMINI_OP_ENCODE_MODE),
		readl(base + GEMINI_OP_GEOM(0)),
		readl(base + GEMINI_OP_GEOM(1)),
		readl(base + GEMINI_OP_GEOM(2)),
		readl(base + GEMINI_OP_GEOM(3)),
		readl(base + GEMINI_OP_FORMAT_MAGIC),
		readl(base + GEMINI_WE_CFG),
		readl(base + GEMINI_WE_Y_PING_ADDR),
		readl(base + GEMINI_WE_Y_PING_CFG),
		readl(base + GEMINI_WE_Y_UB_CFG),
		readl(base + GEMINI_FE_BUFFER_CFG),
		readl(base + GEMINI_FE_Y_PING_ADDR),
		readl(base + GEMINI_FE_CBCR_PING_ADDR),
		readl(base + GEMINI_IRQ_MASK),
		readl(base + GEMINI_IRQ_STATUS),
		readl(base + GEMINI_START_KICK),
		readl(base + GEMINI_FE_CMD));

	/*
	 * Diagnostic poll. We watch:
	 *   - IRQ_STATUS for any bit firing (FRAMEDONE, BUS_ERROR, ...)
	 *   - ENCODE_OUTPUT_SIZE for evidence the WE has produced bytes
	 *   - FE_CMD self-clearing (legacy implies the FE drops the START
	 *     command from the cmd register once it's accepted)
	 *
	 * 50 ms is generous: legacy webOS sees FRAMEDONE in well under a
	 * millisecond on a 320×240 frame. If we still see status=0 and
	 * output_size=0 after 50 ms the encoder genuinely never started.
	 */
	for (i = 0; i < 500; i++) {
		u32 osize = readl(base + GEMINI_ENCODE_OUTPUT_SIZE);
		u32 fecmd = readl(base + GEMINI_FE_CMD);

		status = readl(base + GEMINI_IRQ_STATUS);
		if (status || osize) {
			pr_info("gemini_hw_start: status=0x%08x osize=0x%08x fecmd=0x%08x after %d * 100us\n",
				status, osize, fecmd, i);
			return;
		}
		udelay(100);
	}
	pr_info("gemini_hw_start: status=0 osize=0x%08x fecmd=0x%08x after 50 ms\n",
		readl(base + GEMINI_ENCODE_OUTPUT_SIZE),
		readl(base + GEMINI_FE_CMD));
}

/*
 * Program the encoder for NV12 -> JPEG offline encode.
 *
 * Cross-checked against Samsung's MMJPEG offline-encode trace (the
 * `FUN_00032450` entry in quincy_libmmjpeg.c calls gemini_lib_hw_config
 * with very specific values that differ markedly from what an isolated
 * read of libgemini.so suggests). Key findings:
 *   - op->mode (encode_mode) for offline is 3, not 0. That puts s8 = s12 = 2
 *     and s20 = 16 in op_cfg's geometry math.
 *   - op->mcu_type (and OP_ENCODE_MODE register) for NV12 is 1, not 3.
 *     Samsung's mcu_type==1 dispatches to the branch that emits the
 *     0x0107081F format-magic word — what an isolated reading of op_cfg
 *     would have called the "H1V1" path is actually the canonical NV12
 *     -> YUV420-output JPEG path.
 *   - PIPELINE_CFG carries pl[0]=1, pl[1]=3, pl[7]=1 in addition to the
 *     0x000061FB always-on bits, producing 0x380615FB after the 0x07F775FF
 *     write mask.
 *   - WE_CFG byte we[0]=2 -> register value 0x20.
 *   - MMJPEG explicitly skips Huffman table programming for offline
 *     encode and lets the encoder run with whatever the IP's RESET-time
 *     defaults are.
 *
 * Pixel dimensions are expected aligned to 16 by the V4L2 try_fmt path.
 */
void gemini_hw_configure_encode_h2v2(void __iomem *base, u32 w, u32 h)
{
	/*
	 * Minimal legacy-style programming: the webOS msm_gemini driver did
	 * not write any of PIPELINE_CFG, OP_ENCODE_MODE, OP_GEOM[0..3],
	 * OP_FORMAT_MAGIC, OP_MATRIX[0..8], WE_CFG, DRI_INTERVAL from kernel
	 * code — the encoder is presumed to have working RESET-time defaults
	 * for those, and userspace (libqcameralib.so) handled the
	 * vendor-specific tuning via the legacy MSM_JPEG_IOCTL_HW_CMDS
	 * pass-through. Our cross-vendor-derived values for those registers
	 * keep the encoder from firing FRAMEDONE on this exact MSM8660 IP
	 * even with all clock/reset/buffer prerequisites met. Strip them and
	 * trust the IP defaults.
	 *
	 * H2V2 MCUs are 16×16 pixels (one per 4 Y blocks + 1 Cb + 1 Cr).
	 */
	u32 mcu_rows = (h + 15) / 16;
	u32 mcu_cols = (w + 15) / 16;

	/*
	 * PIPELINE_CFG always-on bits (0x000061FB). Cross-vendor analysis
	 * marks these as the unconditional baseline emitted by libgemini for
	 * every encode mode, so the IP probably relies on them being set
	 * (clock-gate enables and pipeline routing). Legacy webOS kernel
	 * doesn't write this register because the bootloader leaves it
	 * already programmed; on our mainline boot we must set it ourselves.
	 */
	/*
	 * Register write ORDER matters. OPAL's gemini_lib_hw_config (the
	 * top-level config function in libqcameralib's bundled libgemini)
	 * programs everything in a very specific sequence:
	 *
	 *   1. fe_cfg     — FE_INPUT_FORMAT, FE_DIMS, FE_PIPELINE_MODE
	 *   2. op_cfg     — OP_ENCODE_MODE, OP_GEOM, OP_FORMAT_MAGIC, OP_MATRIX
	 *   3. we_cfg     — WE_CFG
	 *   4. pipeline_cfg — PIPELINE_CFG (LAST: arms offline branch + clk-gate)
	 *   5. restart_marker_set — DRI_INTERVAL
	 *
	 * Writing PIPELINE_CFG first (which we used to do) prematurely
	 * arms the offline-encode pipeline before FE/OP/WE registers are
	 * programmed; on this MSM8660 IP that puts the bus into a state
	 * where any subsequent register read hangs the CPU forever.
	 *
	 * Apply the same order, with the cross-vendor coherent recipe
	 * (mode==3 / op_format=1 / NV12 H2V2):
	 */
	u32 Wm = mcu_cols;
	u32 Hm = mcu_rows;

	pr_info("gemini cfg: A0 enter w=%u h=%u Wm=%u Hm=%u\n", w, h, Wm, Hm);

	/* 1. fe_cfg */
	writel(fe_input_format & 0x000F0077, base + GEMINI_FE_INPUT_FORMAT);
	pr_info("gemini cfg: A1 FE_INPUT_FORMAT=0x%08x done\n",
		fe_input_format & 0x000F0077);

	writel(((Wm - 1) & 0x1FF) << 16 | ((Hm - 1) & 0x1FF),
	       base + GEMINI_FE_DIMS);
	pr_info("gemini cfg: A2 FE_DIMS done\n");

	writel(0x203, base + GEMINI_FE_PIPELINE_MODE);
	pr_info("gemini cfg: A3 FE_PIPELINE_MODE=0x203 done\n");

	/* 2. op_cfg
	 *
	 * Live OPAL trace on the TouchPad (webOS 3.0.6) shows that for
	 * NV12 stills OPAL takes the op_format=3 branch of
	 * gemini_lib_hw_op_cfg, NOT the op_format=1 branch the
	 * cross-vendor doc had assumed. The op_format=3 branch:
	 *   - sets OP_ENCODE_MODE = 3
	 *   - sets OP_FORMAT_MAGIC = 0x03381801
	 *   - uses different OP_GEOM formulas, with Hm and Wm SWAPPED
	 *     in the operand position (op_cfg's caller passes a 4-dword
	 *     struct where slots [2] and [3] are Hm and Wm respectively,
	 *     opposite of the mode_dims layout).
	 *
	 * For our 320x240 test, captured live trace at 1024x1280
	 * (Wm=64, Hm=80) shows OP_GEOM[3] = 80*63*128 + 16 = 0x9D810
	 * exactly — i.e. the formula is Hm*(Wm-1)*128+16 in our naming.
	 *
	 *   GEOM[0] = Hm*(Wm-1) * 256
	 *   GEOM[1] = Hm*(Wm-1) * 128
	 *   GEOM[2] = Hm*(Wm-1) * 256 + 16
	 *   GEOM[3] = Hm*(Wm-1) * 128 + 16
	 */
	{
		u32 base_geom = Hm * (Wm - 1);

		writel(3, base + GEMINI_OP_ENCODE_MODE);
		/*
		 * Live webOS delta-log (1024x1280, base_geom=80*63=5040):
		 *   OP_GEOM[0]=0x13b000=*256  [1]=0x9d800=*128
		 *   OP_GEOM[2]=0x13b010=*256+16  [3]=0x9d810=*128+16
		 * i.e. {256,128,256,128}. The code previously had [0] and [1]
		 * swapped (128 and 256 exchanged), which gave the encoder an
		 * inconsistent output geometry so it read the frame but produced
		 * no entropy output (ENCODE_OUTPUT_SIZE stuck at 0, no FRAMEDONE).
		 */
		writel((base_geom * 256)      & 0x03FFFFFF, base + GEMINI_OP_GEOM(0));
		writel((base_geom * 128)      & 0x03FFFFFF, base + GEMINI_OP_GEOM(1));
		writel((base_geom * 256 + 16) & 0x03FFFFFF, base + GEMINI_OP_GEOM(2));
		writel((base_geom * 128 + 16) & 0x03FFFFFF, base + GEMINI_OP_GEOM(3));
		writel(GEMINI_OP_MAGIC_H2V2, base + GEMINI_OP_FORMAT_MAGIC);
	}

	writel(GEMINI_FE_BURST_OL_W0, base + GEMINI_OP_MATRIX(0));
	writel(GEMINI_FE_BURST_OL_W1, base + GEMINI_OP_MATRIX(1));
	writel(GEMINI_FE_BURST_OL_W2, base + GEMINI_OP_MATRIX(2));
	writel(GEMINI_FE_BURST_OL_W3, base + GEMINI_OP_MATRIX(3));
	writel(GEMINI_FE_BURST_OL_W4, base + GEMINI_OP_MATRIX(4));
	writel(GEMINI_FE_BURST_OL_W5, base + GEMINI_OP_MATRIX(5));
	writel(GEMINI_FE_BURST_OL_W6, base + GEMINI_OP_MATRIX(6));
	writel(GEMINI_FE_BURST_OL_W7, base + GEMINI_OP_MATRIX(7));
	writel(GEMINI_FE_BURST_OL_W8, base + GEMINI_OP_MATRIX(8));
	pr_info("gemini cfg: A4 OP_* done (verified via OPAL @0x178FA4)\n");

	/* 3. we_cfg */
	writel(0x20, base + GEMINI_WE_CFG);
	pr_info("gemini cfg: A5 WE_CFG done\n");

	/* 4. pipeline_cfg — LAST */
	writel(GEMINI_PIPELINE_OFFLINE, base + GEMINI_PIPELINE_CFG);
	pr_info("gemini cfg: A6 PIPELINE_CFG=0x020065FB done (last, arms pipeline)\n");

	/* 5. restart_marker_set */
	writel(0, base + GEMINI_DRI_INTERVAL);
	pr_info("gemini cfg: A7 DRI_INTERVAL done — exiting configure\n");
}

static void gemini_table_select(void __iomem *base, u32 sel)
{
	writel(sel, base + GEMINI_TABLE_SEL);
}

/*
 * Quantization tables: load luma at indices 0..63 and chroma at 64..127
 * in one contiguous stream. Match the userspace template ordering:
 *   1. write TABLE_INDEX = 0
 *   2. select QUANT
 *   3. push 128 "16.0 reciprocal-of-Q" entries to TABLE_DATA so the
 *      encoder can multiply instead of divide
 *   4. release SEL
 *
 * Recovered from the static .rodata template of
 * gemini_lib_hw_set_quant_tables() in libqcameralib.so.
 */
void gemini_hw_load_quant_table(void __iomem *base, bool chroma,
				const u16 q[64])
{
	int i;

	if (!chroma) {
		writel(0, base + GEMINI_TABLE_INDEX);
		gemini_table_select(base, GEMINI_TABLE_SEL_QUANT);
	}

	for (i = 0; i < 64; i++) {
		u32 d = (q[i] < 2) ? 0xFFFF : (0x10000 / q[i]);

		writel(d, base + GEMINI_TABLE_DATA);
	}

	if (chroma)
		gemini_table_select(base, GEMINI_TABLE_SEL_RELEASE);
}

/*
 * Read back the quant tables. OPAL's gemini_lib_hw_config calls
 * gemini_lib_hw_read_quant_tables() immediately after
 * gemini_lib_hw_set_quant_tables(); the read sequence at @ 0x14aa10
 * issues 128 READ commands (cmd=0x10) against the TABLE_DATA port,
 * matching the 128 WRITE commands of set_quant_tables.
 *
 * Hypothesis: the quant table RAM is double-buffered or ping-pong
 * latched; the WRITE side fills a shadow buffer, and the READ pass
 * commits the shadow into the active table the encoder uses. Without
 * this readback the encoder runs against whatever quant values were
 * loaded by the bootloader / previous encode session, producing
 * structurally valid but semantically wrong DCT coefficients.
 */
void gemini_hw_readback_quant_tables(void __iomem *base)
{
	int i;

	pr_info("gemini quant: Q0 readback enter\n");

	writel(0, base + GEMINI_TABLE_INDEX);
	gemini_table_select(base, GEMINI_TABLE_SEL_QUANT);

	for (i = 0; i < 128; i++)
		(void)readl(base + GEMINI_TABLE_DATA);

	gemini_table_select(base, GEMINI_TABLE_SEL_RELEASE);

	pr_info("gemini quant: Q1 readback done (128 entries)\n");
}

/*
 * Build the 256-entry per-symbol (size, code) pair buffer that the
 * Huffman hardware-loader expects, given JPEG BITS[16] + HUFFVAL[]
 * arrays. Buffer is keyed by huffval (symbol value 0..255); only the
 * symbols listed in vals[] get non-zero entries. Returns the number
 * of symbols actually written.
 *
 * Implements Annex C of JPEG ISO/IEC 10918-1.
 */
void gemini_build_huff_pairs(struct gemini_huff_pair *pairs,
			     const u8 bits[16], const u8 vals[],
			     unsigned int n_vals, bool is_ac)
{
	u8  huff_size[256];
	u16 huff_code[256];
	unsigned int p, l, i;
	u32 code;

	/* Single-purpose buffer (DC or AC): caller zeroes it; we only write
	 * the symbols listed in vals[]. Do NOT merge DC and AC tables. */

	/* Build (size) sequence per symbol in vals[] order */
	p = 0;
	for (l = 1; l <= 16; l++) {
		for (i = 0; i < bits[l - 1]; i++)
			huff_size[p++] = l;
	}
	if (p > n_vals)
		p = n_vals;

	/* Build canonical (code) sequence */
	code = 0;
	i = 0;
	l = (p > 0) ? huff_size[0] : 0;
	while (i < p) {
		while (i < p && huff_size[i] == l)
			huff_code[i++] = code++;
		if (i >= p)
			break;
		do {
			code <<= 1;
			l++;
		} while (huff_size[i] != l);
	}

	/* Lay out by huffval, with nibble-swap for AC tables. */
	for (i = 0; i < p; i++) {
		u8 v = vals[i];

		if (is_ac)
			v = ((v & 0x0F) << 4) | ((v & 0xF0) >> 4);

		pairs[v].size = huff_size[i];
		pairs[v].code = huff_code[i];
	}
}

/*
 * Load both Huffman tables (luma DC+AC merged, chroma DC+AC merged) into
 * the IP. Procedure verified against gemini_lib_hw_set_huffman_tables in
 * libqcameralib.so (ARM, 0x13ab84) and libgemini.so (Thumb, 0x2e24);
 * structure matches across all four surveyed vendor binaries.
 *
 * Two tables are loaded; each table holds two on-chip slot ranges:
 *   - per-length seed slots, addressed at INDEX = (n << 6) | (2|3) for
 *     n = 0..11 and table-LSB 0 or 1; populated from the first 12 (size,
 *     code) entries of the table's pair buffer
 *   - per-symbol LUT slots, addressed at INDEX = (i << 2) | (0|1) for
 *     i = 0..175 and table-LSB 0 or 1; populated from pair[i]
 *
 * The DATA word per slot encodes (length, code) plus a small per-slot
 * length adjustment that the hardware uses for run-length state:
 *   high 16 bits = ((size - 1) + offset) & 0x1F
 *   low  16 bits = (code << (16 - size)) & 0xFFFF
 * where offset = n in the prologue and offset = (i >> 4) in the main loop.
 *
 * Reg 0xF4 bit 16 must be set with mask 0x1FFFF to enter "huffman load
 * mode" before the table-port writes. TABLE_SEL = 6 covers both halves of
 * the load; SEL is released to 0 at the end.
 */
void gemini_hw_load_huffman_tables(void __iomem *base,
				   const struct gemini_huff_pair *luma_dc,
				   const struct gemini_huff_pair *luma_ac,
				   const struct gemini_huff_pair *chroma_dc,
				   const struct gemini_huff_pair *chroma_ac)
{
	/*
	 * Prologue (per-length seed slots) is fed from the DC tables; the
	 * main per-symbol LUT loop is fed from the AC tables. Matches OPAL's
	 * gemini_lib_hw_set_huffman_tables(dc_luma, dc_chroma, ac_luma,
	 * ac_chroma) — DC and AC are kept in separate buffers.
	 */
	const struct gemini_huff_pair *dc_tbls[2] = { luma_dc, chroma_dc };
	const struct gemini_huff_pair *ac_tbls[2] = { luma_ac, chroma_ac };
	u32 dri;
	int table, n, i;

	/* Enter huffman load mode: clear bits 0-15 of 0xF4, set bit 16 */
	dri = readl(base + GEMINI_DRI_INTERVAL) & ~0x0001FFFFu;
	writel(dri | 0x00010000u, base + GEMINI_DRI_INTERVAL);

	gemini_table_select(base, GEMINI_TABLE_SEL_HUFFMAN);
	writel(0, base + GEMINI_TABLE_INDEX);

	/* Prologue: per-length seed slots (DC tables) */
	for (table = 0; table < 2; table++) {
		u32 base_id = (table == 0) ? 2u : 3u;

		for (n = 0; n < 12; n++) {
			const struct gemini_huff_pair *p = &dc_tbls[table][n];
			u32 hi, lo, idx;

			idx = ((n << 6) | base_id) & 0x3FF;
			hi  = (p->size > 0)
			    ? (((u32)p->size - 1u + (u32)n) & 0x1Fu)
			    : (((u32)n - 1u) & 0x1Fu);
			lo  = (p->size > 0)
			    ? (((u32)p->code << (16 - p->size)) & 0xFFFFu)
			    : 0;
			writel(idx, base + GEMINI_TABLE_INDEX);
			writel((hi << 16) | lo, base + GEMINI_TABLE_DATA);
		}
	}

	/* Main: per-symbol LUT slots, 176 per table (AC tables) */
	for (table = 0; table < 2; table++) {
		u32 lsb = (u32)table;

		for (i = 0; i < 176; i++) {
			const struct gemini_huff_pair *p = &ac_tbls[table][i];
			u32 hi, lo, idx;

			idx = ((i << 2) | lsb) & 0x3FF;
			hi  = (p->size > 0)
			    ? (((u32)p->size - 1u + ((u32)i >> 4)) & 0x1Fu)
			    : ((((u32)i >> 4) - 1u) & 0x1Fu);
			lo  = (p->size > 0)
			    ? (((u32)p->code << (16 - p->size)) & 0xFFFFu)
			    : 0;
			writel(idx, base + GEMINI_TABLE_INDEX);
			writel((hi << 16) | lo, base + GEMINI_TABLE_DATA);
		}
	}

	gemini_table_select(base, GEMINI_TABLE_SEL_RELEASE);

	/*
	 * Exit huffman load mode: clear bit 16 of DRI_INTERVAL.
	 * OPAL's gemini_lib_hw_set_huffman_tables ends with a WRITE_OR_AND
	 * cmd that zeroes bits 0-16 of 0xF4. Without this clear, the
	 * encoder stays in "huffman load mode" during encode and the
	 * entropy coder produces wrong codes — symptom is per-MCU-column
	 * coefficient corruption with the first MCU column decoding
	 * correctly.
	 */
	dri = readl(base + GEMINI_DRI_INTERVAL) & ~0x0001FFFFu;
	writel(dri, base + GEMINI_DRI_INTERVAL);
}

/* Legacy single-table loader retained for symmetry; not used. */
void gemini_hw_load_huffman_table(void __iomem *base, u32 table_id,
				  const u8 bits[16], const u8 vals[],
				  unsigned int n_vals)
{
	(void)base;
	(void)table_id;
	(void)bits;
	(void)vals;
	(void)n_vals;
}
