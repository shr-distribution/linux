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

/* JPEG Register Offsets */
#define GEMINI_RESET_CMD		0x0004
#define GEMINI_IRQ_MASK			0x0014
#define GEMINI_IRQ_CLEAR		0x0018
#define GEMINI_IRQ_STATUS		0x001C
#define GEMINI_ENCODE_OUTPUT_SIZE	0x0034

/* Fetch Engine (FE) - Input buffer registers */
#define GEMINI_FE_BUFFER_CFG		0x0080
#define GEMINI_FE_Y_PING_ADDR		0x0084
#define GEMINI_FE_Y_PONG_ADDR		0x0088
#define GEMINI_FE_CBCR_PING_ADDR	0x008C
#define GEMINI_FE_CBCR_PONG_ADDR	0x0090
#define GEMINI_FE_CMD			0x0094

/* Write Engine (WE) - Output buffer registers */
#define GEMINI_WE_Y_THRESHOLD		0x00C0
#define GEMINI_WE_CBCR_THRESHOLD	0x00C4
#define GEMINI_WE_Y_PING_CFG		0x00C8
#define GEMINI_WE_Y_PONG_CFG		0x00CC
#define GEMINI_WE_Y_PING_ADDR		0x00D8
#define GEMINI_WE_Y_PONG_ADDR		0x00DC
#define GEMINI_WE_Y_UB_CFG		0x00E8

/* Command values */
#define GEMINI_RESET_DEFAULT		0x0004FFFF
#define GEMINI_FE_CMD_RELOAD		0x00000001
#define GEMINI_OFFLINE_CMD_START	0x00000003
#define GEMINI_REALTIME_CMD_START	0x00000001

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
void gemini_hw_set_fe_ping(void __iomem *base, dma_addr_t y_addr,
			   dma_addr_t cbcr_addr, u32 y_rows, u32 cbcr_rows);
void gemini_hw_set_we_ping(void __iomem *base, dma_addr_t addr, u32 len);
void gemini_hw_fe_reload(void __iomem *base);
void gemini_hw_start_offline(void __iomem *base);

#endif /* _GEMINI_HW_H_ */
