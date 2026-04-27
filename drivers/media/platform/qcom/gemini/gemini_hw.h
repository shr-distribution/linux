/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm MSM8660 JPEG Encoder/Decoder (Gemini) driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_gemini driver from webOS kernel.
 */

#ifndef _GEMINI_HW_H_
#define _GEMINI_HW_H_

#include <linux/types.h>

/*
 * JPEG Register Offsets
 *
 * The legacy webOS kernel exposes only ~20 of these registers; the rest
 * (encoder mode/format, geometry, pipeline, quantization and Huffman
 * table ports, JPEG header DMA) were programmed by userspace via a
 * pass-through HW_CMD ioctl. The offsets below were recovered by
 * disassembling libqcameralib.so from the webOS rootfs and decoding the
 * static command-stream templates and the per-format derivation code in
 * gemini_lib_hw_{fe,op,we,pipeline}_cfg().
 */
#define GEMINI_HW_VERSION		0x0000	/* RO */
#define GEMINI_RESET_CMD		0x0004
#define GEMINI_PIPELINE_CFG		0x0008	/* mask 0x07F775FF */
#define GEMINI_REALTIME_CMD		0x000C	/* mask 0x3: 1=start, 3=stop */
#define GEMINI_IRQ_MASK			0x0014
#define GEMINI_IRQ_CLEAR		0x0018
#define GEMINI_IRQ_STATUS		0x001C
#define GEMINI_STOP_REQ			0x0024	/* bit 0 */
#define GEMINI_STOP_STATUS		0x0028	/* poll until == 3 */
#define GEMINI_ENCODE_OUTPUT_SIZE	0x0034	/* RO */

/* Fetch Engine (FE) input geometry/format */
#define GEMINI_FE_INPUT_FORMAT		0x0038	/* mask 0x000F0077 */
#define GEMINI_FE_DIMS			0x003C	/* (rows-1)<<16 | (cols-1), mask 0x01FF01FF */
#define GEMINI_FE_PIPELINE_MODE		0x0040	/* mask 0x000107FF: 0x101 or 0x203 */

/* Output (encoder) geometry/format/matrix */
#define GEMINI_OP_ENCODE_MODE		0x0044	/* mask 0x3 */
#define GEMINI_OP_GEOM(n)		(0x0048 + (n) * 4)	/* n in 0..3, mask 0x03FFFFFF */
#define GEMINI_OP_FORMAT_MAGIC		0x0058	/* mask 0x033F1F1F */
#define GEMINI_OP_MATRIX(n)		(0x005C + (n) * 4)	/* n in 0..8, FE burst-mask regs */

/* Fetch Engine (FE) - Input buffer registers */
#define GEMINI_FE_BUFFER_CFG		0x0080
#define GEMINI_FE_Y_PING_ADDR		0x0084
#define GEMINI_FE_Y_PONG_ADDR		0x0088
#define GEMINI_FE_CBCR_PING_ADDR	0x008C
#define GEMINI_FE_CBCR_PONG_ADDR	0x0090
#define GEMINI_FE_CMD			0x0094

/* Write Engine (WE) configuration + buffer registers */
#define GEMINI_WE_CFG			0x0098	/* mask 0x000F0F37 */
#define GEMINI_WE_Y_THRESHOLD		0x00C0
#define GEMINI_WE_CBCR_THRESHOLD	0x00C4
#define GEMINI_WE_Y_PING_CFG		0x00C8
#define GEMINI_WE_Y_PONG_CFG		0x00CC
#define GEMINI_WE_Y_PING_ADDR		0x00D8
#define GEMINI_WE_Y_PONG_ADDR		0x00DC
#define GEMINI_WE_Y_UB_CFG		0x00E8

/* Encode start kick + restart-marker interval */
#define GEMINI_START_KICK		0x00F0	/* bit 0, written =1 right before FE_CMD start */
#define GEMINI_DRI_INTERVAL		0x00F4	/* mask 0xFFFF; bit 0 doubles as huff-load enable (RMW) */

/* Filesize control (skippable for basic encode) */
#define GEMINI_FSC_COUNT		0x0110
#define GEMINI_FSC_THRESHOLD(n)		(0x0114 + (n) * 4)	/* n in 0..3 */

/* Quantization + Huffman table ports (shared) */
#define GEMINI_TABLE_SEL		0x0124	/* mask 0x7: 5=quant, 6=huff, 0=release */
#define GEMINI_TABLE_INDEX		0x0128	/* mask 0x3FF */
#define GEMINI_TABLE_DATA		0x012C	/* 32-bit, auto-increments INDEX */

/* TABLE_SEL values */
#define GEMINI_TABLE_SEL_QUANT		5
#define GEMINI_TABLE_SEL_HUFFMAN	6
#define GEMINI_TABLE_SEL_RELEASE	0

/* Command values */
#define GEMINI_RESET_DEFAULT		0x0004FFFF
#define GEMINI_FE_CMD_RELOAD		0x00000001
#define GEMINI_OFFLINE_CMD_START	0x00000003
#define GEMINI_REALTIME_CMD_START	0x00000001

/*
 * PIPELINE_CFG offline-encode value, derived from a careful re-read of
 * OPAL's gemini_lib_hw_pipeline_cfg (libqcameralib bundled libgemini).
 * The earlier cross-vendor doc was wrong: it claimed bit 10 = clk-gate
 * "always 1 for offline" and missed bits 23-24 entirely. OPAL's actual
 * formula:
 *
 *   PIPELINE_CFG = 0x61FB                               always-on
 *                | (offline ? (1<<25) : 0)              bit 25 = offline branch
 *                | ((mode_dims.mode & 3) << 23)         bits 23-24 = mode value
 *                | (p[5] & 1) << 10                     bit 10  (0 on Opal)
 *                | (p[1] & 3) << 23                     bits 23-24 (= mode bits 0-1)
 *                | (p[2] & 1) << 22 ... etc             all 0 on Opal
 *
 * For unrotated NV12 H2V2 offline encode (mode_dims.mode = 3) Opal sets
 * bit 25 (offline branch) AND bits 23-24 (mode value 3 = 0b11).
 *
 *   0x61FB | 0x02000000 | 0x01800000 = 0x038061FB
 *
 * Without bits 23-24 == 3, the IP's offline pipeline is in an
 * indeterminate state and FE_CMD = OFFLINE_CMD_START hangs the bus.
 */
#define GEMINI_PIPELINE_BASE		0x000061FB
#define GEMINI_PIPELINE_OFFLINE		0x038061FB

/* OP_FORMAT_MAGIC values per chroma subsampling */
#define GEMINI_OP_MAGIC_MONO		0x00000108
#define GEMINI_OP_MAGIC_H1V1		0x0107081F
#define GEMINI_OP_MAGIC_H2V1		0x023F1F18
#define GEMINI_OP_MAGIC_H2V2		0x03381801

/*
 * FE burst masks for offline (OL) encode at op->mode == 0.
 * Recovered from libqcameralib .rodata at GEMINI_FE_OL_BURST_TABLE
 * (file offset 0x178f14), strided 12 bytes, written to OP_MATRIX(0..8).
 */
#define GEMINI_FE_BURST_OL_W0		0x00000303
#define GEMINI_FE_BURST_OL_W1		0x00F0000F
#define GEMINI_FE_BURST_OL_W2		0xF0000F00
#define GEMINI_FE_BURST_OL_W3		0x00000000
#define GEMINI_FE_BURST_OL_W4		0x00000000
#define GEMINI_FE_BURST_OL_W5		0x0C0C0303
#define GEMINI_FE_BURST_OL_W6		0xC0C03030
#define GEMINI_FE_BURST_OL_W7		0x00000000
#define GEMINI_FE_BURST_OL_W8		0x00000000

/* IRQ status bits */
#define GEMINI_IRQ_FRAMEDONE		BIT(0)
#define GEMINI_IRQ_FE_RD_DONE		BIT(1)
#define GEMINI_IRQ_FE_RTOVF		BIT(2)
#define GEMINI_IRQ_FE_VFE_OVERFLOW	BIT(3)
#define GEMINI_IRQ_WE_Y_PINGPONG	BIT(4)
#define GEMINI_IRQ_WE_CBCR_PINGPONG	BIT(5)
#define GEMINI_IRQ_WE_Y_OVERFLOW	BIT(6)
#define GEMINI_IRQ_WE_CBCR_OVERFLOW	BIT(7)
#define GEMINI_IRQ_WE_CH0_OVERFLOW	BIT(8)
#define GEMINI_IRQ_WE_CH1_OVERFLOW	BIT(9)
#define GEMINI_IRQ_RESET_ACK		BIT(10)
#define GEMINI_IRQ_BUS_ERROR		BIT(11)
#define GEMINI_IRQ_VIOLATION		BIT(12)

#define GEMINI_IRQ_ALL			0xFFFFFFFF
#define GEMINI_IRQ_DISABLE		0x00000000

/* Buffer configuration masks */
#define GEMINI_FE_CBCR_MCU_ROWS_MASK	GENMASK(28, 16)
#define GEMINI_FE_CBCR_MCU_ROWS_SHIFT	16
#define GEMINI_FE_Y_MCU_ROWS_MASK	GENMASK(12, 0)
#define GEMINI_FE_Y_MCU_ROWS_SHIFT	0

#define GEMINI_WE_BUFFER_LEN_MASK	GENMASK(22, 0)

/* Hardware functions */
int gemini_hw_reset(void __iomem *base);
void gemini_hw_enable_irq(void __iomem *base, u32 mask);
void gemini_hw_disable_irq(void __iomem *base);
void gemini_hw_clear_irq(void __iomem *base, u32 mask);
u32 gemini_hw_get_irq_status(void __iomem *base);
u32 gemini_hw_get_output_size(void __iomem *base);

/* Buffer configuration */
void gemini_hw_we_post_reset_cfg(void __iomem *base);
void gemini_hw_set_fe_ping(void __iomem *base, dma_addr_t y_addr,
			   dma_addr_t cbcr_addr, u32 num_mcu_rows);
void gemini_hw_set_we_ping(void __iomem *base, dma_addr_t addr, u32 len);
void gemini_hw_fe_reload(void __iomem *base);
void gemini_hw_start_offline(void __iomem *base);

/* Encoder configuration (program once after reset, before per-frame data) */
void gemini_hw_configure_encode_h2v2(void __iomem *base, u32 width, u32 height);

/* Quantization + Huffman tables */
void gemini_hw_load_quant_table(void __iomem *base, bool chroma,
				const u16 q[64]);
void gemini_hw_readback_quant_tables(void __iomem *base);

/*
 * One huffman code per JPEG symbol value. Caller fills a 256-entry array
 * (one per huffval byte 0..255) from BITS[16] + HUFFVAL[] via Annex C;
 * unused symbols stay zero.
 */
struct gemini_huff_pair {
	u16 code;
	u16 size;
};

void gemini_hw_load_huffman_tables(void __iomem *base,
				   const struct gemini_huff_pair *luma,
				   const struct gemini_huff_pair *chroma);

/*
 * Build a per-huffval pair buffer from JPEG BITS[16] + HUFFVAL[].
 * Pass it twice to merge a DC + AC table into a single pair buffer
 * (call once for AC, then again for DC — DC at low huffvals overrides
 * AC at the same indices, matching the camera HAL convention).
 */
void gemini_build_huff_pairs(struct gemini_huff_pair *pairs,
			     const u8 bits[16], const u8 vals[],
			     unsigned int n_vals);

/* Legacy single-table loader (no-op stub retained for ABI symmetry). */
void gemini_hw_load_huffman_table(void __iomem *base, u32 table_id,
				  const u8 bits[16], const u8 vals[],
				  unsigned int n_vals);

#endif /* _GEMINI_HW_H_ */
