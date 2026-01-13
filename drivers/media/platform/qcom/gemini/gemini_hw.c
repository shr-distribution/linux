// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 JPEG Encoder/Decoder (Gemini) - Hardware abstraction
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_gemini driver from webOS kernel.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include "gemini_hw.h"

int gemini_hw_reset(void __iomem *base)
{
	int timeout = 100;

	/* Clear any pending interrupts */
	writel(GEMINI_IRQ_ALL, base + GEMINI_IRQ_CLEAR);

	/* Issue reset command */
	writel(GEMINI_RESET_DEFAULT, base + GEMINI_RESET_CMD);

	/* Wait for reset acknowledgment */
	while (timeout > 0) {
		u32 status = readl(base + GEMINI_IRQ_STATUS);

		if (status & GEMINI_IRQ_RESET_ACK) {
			/* Clear the reset ack interrupt */
			writel(GEMINI_IRQ_RESET_ACK, base + GEMINI_IRQ_CLEAR);
			return 0;
		}
		udelay(100);
		timeout--;
	}

	return -ETIMEDOUT;
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
			   dma_addr_t cbcr_addr, u32 y_rows, u32 cbcr_rows)
{
	u32 cfg;

	/* Set buffer configuration (MCU rows) */
	cfg = ((cbcr_rows << GEMINI_FE_CBCR_MCU_ROWS_SHIFT) &
	       GEMINI_FE_CBCR_MCU_ROWS_MASK) |
	      ((y_rows << GEMINI_FE_Y_MCU_ROWS_SHIFT) &
	       GEMINI_FE_Y_MCU_ROWS_MASK);
	writel(cfg, base + GEMINI_FE_BUFFER_CFG);

	/* Set ping buffer addresses */
	writel(y_addr, base + GEMINI_FE_Y_PING_ADDR);
	writel(cbcr_addr, base + GEMINI_FE_CBCR_PING_ADDR);
}

void gemini_hw_set_we_ping(void __iomem *base, dma_addr_t addr, u32 len)
{
	/* Set output buffer address */
	writel(addr, base + GEMINI_WE_Y_PING_ADDR);

	/* Set output buffer length */
	writel(len & GEMINI_WE_BUFFER_LEN_MASK, base + GEMINI_WE_Y_PING_CFG);

	/* Set thresholds for flow control */
	writel(0x00FF00FF, base + GEMINI_WE_Y_THRESHOLD);
}

void gemini_hw_fe_reload(void __iomem *base)
{
	writel(GEMINI_FE_CMD_RELOAD, base + GEMINI_FE_CMD);
}

void gemini_hw_start_offline(void __iomem *base)
{
	/* For offline encoding, write start command to FE_CMD */
	writel(GEMINI_OFFLINE_CMD_START, base + GEMINI_FE_CMD);
}
