// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM QSD SPI Controller Driver
 *
 * Copyright (c) 2008-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2026, webOS Community
 *
 * This is the legacy SPI controller found on MSM7x30 and earlier Qualcomm SoCs.
 * It predates the QUP (Qualcomm Universal Peripheral) unified serial controller.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>

/* QSD SPI Register Offsets */
#define QSD_SPI_CONFIG			0x0000
#define QSD_SPI_IO_CONTROL		0x0004
#define QSD_SPI_IO_MODES		0x0008
#define QSD_SPI_SW_RESET		0x000c
#define QSD_SPI_TIME_OUT		0x0010
#define QSD_SPI_TIME_OUT_CURRENT	0x0014
#define QSD_SPI_MX_OUTPUT_COUNT		0x0018
#define QSD_SPI_MX_OUTPUT_CNT_CUR	0x001c
#define QSD_SPI_MX_INPUT_COUNT		0x0020
#define QSD_SPI_MX_INPUT_CNT_CUR	0x0024
#define QSD_SPI_MX_READ_COUNT		0x0028
#define QSD_SPI_MX_READ_CNT_CUR		0x002c
#define QSD_SPI_OPERATIONAL		0x0030
#define QSD_SPI_ERROR_FLAGS		0x0034
#define QSD_SPI_ERROR_FLAGS_EN		0x0038
#define QSD_SPI_DEASSERT_WAIT		0x003c
#define QSD_SPI_OUTPUT_DEBUG		0x0040
#define QSD_SPI_INPUT_DEBUG		0x0044
#define QSD_SPI_FIFO_WORD_CNT		0x0048
#define QSD_SPI_TEST_CTRL		0x004c
#define QSD_SPI_OUTPUT_FIFO		0x0100
#define QSD_SPI_INPUT_FIFO		0x0200

/* SPI_CONFIG bits */
#define SPI_CFG_N_MASK			0x0000001f
#define SPI_CFG_NO_OUTPUT		BIT(6)
#define SPI_CFG_NO_INPUT		BIT(7)
#define SPI_CFG_LOOPBACK		BIT(8)
#define SPI_CFG_INPUT_FIRST		BIT(9)
#define SPI_CFG_HS_MODE			BIT(10)

/* SPI_IO_CONTROL bits */
#define SPI_IO_C_NO_TRI_STATE		BIT(0)
#define SPI_IO_C_TRISTATE_CS		BIT(1)
#define SPI_IO_C_CS_N_POLARITY_SHIFT	4
#define SPI_IO_C_CS_N_POLARITY_MASK	0x000000f0
#define SPI_IO_C_CS_SELECT_SHIFT	8
#define SPI_IO_C_CS_SELECT_MASK		0x00000300
#define SPI_IO_C_CLK_IDLE_HIGH		BIT(10)
#define SPI_IO_C_FORCE_CS		BIT(11)

/* SPI_IO_MODES bits */
#define SPI_IO_M_OUTPUT_BIT_SHIFT_EN	BIT(15)
#define SPI_IO_M_PACK_EN		BIT(14)
#define SPI_IO_M_UNPACK_EN		BIT(13)
#define SPI_IO_M_INPUT_MODE_SHIFT	12
#define SPI_IO_M_OUTPUT_MODE_SHIFT	10
#define SPI_IO_M_MODE_MASK		0x03
#define SPI_IO_M_INPUT_FIFO_SIZE_SHIFT	7
#define SPI_IO_M_INPUT_FIFO_SIZE_MASK	(0x07 << 7)
#define SPI_IO_M_INPUT_BLOCK_SIZE_SHIFT	5
#define SPI_IO_M_INPUT_BLOCK_SIZE_MASK	(0x03 << 5)
#define SPI_IO_M_OUTPUT_FIFO_SIZE_SHIFT	2
#define SPI_IO_M_OUTPUT_FIFO_SIZE_MASK	(0x07 << 2)
#define SPI_IO_M_OUTPUT_BLOCK_SIZE_MASK	0x03

#define SPI_IO_M_MODE_FIFO		0
#define SPI_IO_M_MODE_BLOCK		1

/* SPI_OPERATIONAL bits */
#define SPI_OP_STATE_MASK		0x00000003
#define SPI_OP_STATE_RESET		0x00000000
#define SPI_OP_STATE_RUN		0x00000001
#define SPI_OP_STATE_PAUSE		0x00000003
#define SPI_OP_OUT_FIFO_NOT_EMPTY	BIT(4)
#define SPI_OP_IN_FIFO_NOT_EMPTY	BIT(5)
#define SPI_OP_OUT_FIFO_FULL		BIT(6)
#define SPI_OP_IN_FIFO_FULL		BIT(7)
#define SPI_OP_OUT_SERVICE_FLAG		BIT(8)
#define SPI_OP_IN_SERVICE_FLAG		BIT(9)
#define SPI_OP_MAX_OUTPUT_DONE		BIT(10)
#define SPI_OP_MAX_INPUT_DONE		BIT(11)

/* SPI_ERROR_FLAGS bits */
#define SPI_ERR_CLK_OVER_RUN		BIT(1)
#define SPI_ERR_CLK_UNDER_RUN		BIT(2)
#define SPI_ERR_OUTPUT_OVER_RUN		BIT(4)
#define SPI_ERR_OUTPUT_UNDER_RUN	BIT(5)
#define SPI_ERR_INPUT_OVER_RUN		BIT(6)
#define SPI_ERR_INPUT_UNDER_RUN		BIT(7)

/* Maximum transfer size per operation */
#define QSD_SPI_MAX_XFER_SIZE		(64 * 1024 - 1)

/* FIFO depth in words */
#define QSD_SPI_FIFO_DEPTH		8

/* Timeout for state transitions */
#define QSD_SPI_TIMEOUT_MS		100

struct qsd_spi {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct clk *pclk;
	int irq;

	struct completion done;

	/* Transfer state */
	const u8 *tx_buf;
	u8 *rx_buf;
	unsigned int tx_remaining;
	unsigned int rx_remaining;
	unsigned int bytes_per_word;

	/* Clock rate */
	unsigned long max_speed_hz;
};

static inline u32 qsd_spi_read(struct qsd_spi *qsd, u32 reg)
{
	return readl_relaxed(qsd->base + reg);
}

static inline void qsd_spi_write(struct qsd_spi *qsd, u32 reg, u32 val)
{
	writel_relaxed(val, qsd->base + reg);
}

static int qsd_spi_set_state(struct qsd_spi *qsd, u32 state)
{
	u32 val;
	unsigned long timeout;

	qsd_spi_write(qsd, QSD_SPI_OPERATIONAL, state);

	timeout = jiffies + msecs_to_jiffies(QSD_SPI_TIMEOUT_MS);
	do {
		val = qsd_spi_read(qsd, QSD_SPI_OPERATIONAL);
		if ((val & SPI_OP_STATE_MASK) == state)
			return 0;
		usleep_range(10, 50);
	} while (time_before(jiffies, timeout));

	dev_err(qsd->dev, "timeout setting state %u, current %u\n",
		state, val & SPI_OP_STATE_MASK);
	return -ETIMEDOUT;
}

static void qsd_spi_fifo_write(struct qsd_spi *qsd)
{
	u32 word;
	int i;

	while (qsd->tx_remaining &&
	       !(qsd_spi_read(qsd, QSD_SPI_OPERATIONAL) & SPI_OP_OUT_FIFO_FULL)) {
		word = 0;
		for (i = 0; i < qsd->bytes_per_word && qsd->tx_remaining; i++) {
			if (qsd->tx_buf) {
				word |= (*qsd->tx_buf++) << (i * 8);
			}
			qsd->tx_remaining--;
		}
		qsd_spi_write(qsd, QSD_SPI_OUTPUT_FIFO, word);
	}
}

static void qsd_spi_fifo_read(struct qsd_spi *qsd)
{
	u32 word;
	int i;

	while (qsd->rx_remaining &&
	       (qsd_spi_read(qsd, QSD_SPI_OPERATIONAL) & SPI_OP_IN_FIFO_NOT_EMPTY)) {
		word = qsd_spi_read(qsd, QSD_SPI_INPUT_FIFO);
		for (i = 0; i < qsd->bytes_per_word && qsd->rx_remaining; i++) {
			if (qsd->rx_buf) {
				*qsd->rx_buf++ = (word >> (i * 8)) & 0xff;
			}
			qsd->rx_remaining--;
		}
	}
}

static irqreturn_t qsd_spi_irq(int irq, void *data)
{
	struct qsd_spi *qsd = data;
	u32 op, err;

	op = qsd_spi_read(qsd, QSD_SPI_OPERATIONAL);
	err = qsd_spi_read(qsd, QSD_SPI_ERROR_FLAGS);

	/* Clear errors */
	if (err) {
		qsd_spi_write(qsd, QSD_SPI_ERROR_FLAGS, err);
		dev_err(qsd->dev, "SPI error: 0x%08x\n", err);
	}

	/* Handle output service */
	if (op & SPI_OP_OUT_SERVICE_FLAG) {
		qsd_spi_write(qsd, QSD_SPI_OPERATIONAL, SPI_OP_OUT_SERVICE_FLAG);
		qsd_spi_fifo_write(qsd);
	}

	/* Handle input service */
	if (op & SPI_OP_IN_SERVICE_FLAG) {
		qsd_spi_write(qsd, QSD_SPI_OPERATIONAL, SPI_OP_IN_SERVICE_FLAG);
		qsd_spi_fifo_read(qsd);
	}

	/* Check for completion */
	if ((op & SPI_OP_MAX_INPUT_DONE) && qsd->rx_remaining == 0) {
		complete(&qsd->done);
	}

	return IRQ_HANDLED;
}

static void qsd_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct qsd_spi *qsd = spi_controller_get_devdata(spi->controller);
	u32 val;

	val = qsd_spi_read(qsd, QSD_SPI_IO_CONTROL);
	val &= ~(SPI_IO_C_CS_SELECT_MASK | SPI_IO_C_FORCE_CS);

	/* Set chip select */
	val |= (spi_get_chipselect(spi, 0) << SPI_IO_C_CS_SELECT_SHIFT);

	/* Assert/deassert CS - note: enable is inverted (false = assert) */
	if (!enable)
		val |= SPI_IO_C_FORCE_CS;

	qsd_spi_write(qsd, QSD_SPI_IO_CONTROL, val);
}

static int qsd_spi_setup_transfer(struct qsd_spi *qsd, struct spi_device *spi,
				  struct spi_transfer *xfer)
{
	u32 config, io_ctrl, io_modes;
	unsigned int bits_per_word;
	unsigned int speed_hz;
	unsigned long clk_rate;
	int ret;

	bits_per_word = xfer->bits_per_word ?: spi->bits_per_word ?: 8;
	speed_hz = xfer->speed_hz ?: spi->max_speed_hz;

	/* Calculate bytes per word */
	qsd->bytes_per_word = DIV_ROUND_UP(bits_per_word, 8);

	/* Set clock rate */
	clk_rate = clk_get_rate(qsd->clk);
	if (clk_rate > speed_hz) {
		ret = clk_set_rate(qsd->clk, speed_hz);
		if (ret) {
			dev_err(qsd->dev, "failed to set clock rate: %d\n", ret);
			return ret;
		}
	}

	/* Configure SPI_CONFIG */
	config = (bits_per_word - 1) & SPI_CFG_N_MASK;
	if (spi->mode & SPI_CPHA)
		config |= SPI_CFG_INPUT_FIRST;
	if (!xfer->rx_buf)
		config |= SPI_CFG_NO_INPUT;
	if (!xfer->tx_buf)
		config |= SPI_CFG_NO_OUTPUT;
	if (spi->mode & SPI_LOOP)
		config |= SPI_CFG_LOOPBACK;

	qsd_spi_write(qsd, QSD_SPI_CONFIG, config);

	/* Configure IO control */
	io_ctrl = qsd_spi_read(qsd, QSD_SPI_IO_CONTROL);
	io_ctrl |= SPI_IO_C_NO_TRI_STATE;

	if (spi->mode & SPI_CPOL)
		io_ctrl |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		io_ctrl &= ~SPI_IO_C_CLK_IDLE_HIGH;

	if (spi->mode & SPI_CS_HIGH)
		io_ctrl |= (1 << (SPI_IO_C_CS_N_POLARITY_SHIFT +
				  spi_get_chipselect(spi, 0)));
	else
		io_ctrl &= ~(1 << (SPI_IO_C_CS_N_POLARITY_SHIFT +
				   spi_get_chipselect(spi, 0)));

	qsd_spi_write(qsd, QSD_SPI_IO_CONTROL, io_ctrl);

	/* Configure IO modes - use FIFO mode */
	io_modes = (SPI_IO_M_MODE_FIFO << SPI_IO_M_INPUT_MODE_SHIFT) |
		   (SPI_IO_M_MODE_FIFO << SPI_IO_M_OUTPUT_MODE_SHIFT) |
		   SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN;

	qsd_spi_write(qsd, QSD_SPI_IO_MODES, io_modes);

	return 0;
}

static int qsd_spi_transfer_one(struct spi_controller *ctlr,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct qsd_spi *qsd = spi_controller_get_devdata(ctlr);
	unsigned long timeout;
	int ret;

	if (xfer->len > QSD_SPI_MAX_XFER_SIZE) {
		dev_err(qsd->dev, "transfer too large: %u\n", xfer->len);
		return -EINVAL;
	}

	ret = qsd_spi_setup_transfer(qsd, spi, xfer);
	if (ret)
		return ret;

	/* Initialize transfer state */
	qsd->tx_buf = xfer->tx_buf;
	qsd->rx_buf = xfer->rx_buf;
	qsd->tx_remaining = xfer->len;
	qsd->rx_remaining = xfer->len;
	reinit_completion(&qsd->done);

	/* Set transfer counts */
	qsd_spi_write(qsd, QSD_SPI_MX_OUTPUT_COUNT, xfer->len);
	qsd_spi_write(qsd, QSD_SPI_MX_INPUT_COUNT, xfer->len);
	qsd_spi_write(qsd, QSD_SPI_MX_READ_COUNT, xfer->len);

	/* Enable error interrupts */
	qsd_spi_write(qsd, QSD_SPI_ERROR_FLAGS_EN,
		      SPI_ERR_CLK_OVER_RUN | SPI_ERR_CLK_UNDER_RUN |
		      SPI_ERR_OUTPUT_OVER_RUN | SPI_ERR_OUTPUT_UNDER_RUN |
		      SPI_ERR_INPUT_OVER_RUN | SPI_ERR_INPUT_UNDER_RUN);

	/* Put controller in reset state */
	ret = qsd_spi_set_state(qsd, SPI_OP_STATE_RESET);
	if (ret)
		return ret;

	/* Prime the TX FIFO */
	qsd_spi_fifo_write(qsd);

	/* Start the transfer */
	ret = qsd_spi_set_state(qsd, SPI_OP_STATE_RUN);
	if (ret)
		return ret;

	/* Wait for completion */
	timeout = wait_for_completion_timeout(&qsd->done,
					      msecs_to_jiffies(5000));
	if (!timeout) {
		dev_err(qsd->dev, "transfer timeout\n");
		qsd_spi_set_state(qsd, SPI_OP_STATE_RESET);
		return -ETIMEDOUT;
	}

	/* Put back in reset state */
	qsd_spi_set_state(qsd, SPI_OP_STATE_RESET);

	return 0;
}

static int qsd_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct qsd_spi *qsd;
	int ret;

	ctlr = spi_alloc_host(&pdev->dev, sizeof(*qsd));
	if (!ctlr)
		return -ENOMEM;

	qsd = spi_controller_get_devdata(ctlr);
	qsd->dev = &pdev->dev;

	platform_set_drvdata(pdev, ctlr);

	qsd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(qsd->base)) {
		ret = PTR_ERR(qsd->base);
		goto err_free;
	}

	qsd->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(qsd->clk)) {
		ret = PTR_ERR(qsd->clk);
		dev_err(&pdev->dev, "failed to get core clock: %d\n", ret);
		goto err_free;
	}

	qsd->pclk = devm_clk_get_optional(&pdev->dev, "iface");
	if (IS_ERR(qsd->pclk)) {
		ret = PTR_ERR(qsd->pclk);
		dev_err(&pdev->dev, "failed to get iface clock: %d\n", ret);
		goto err_free;
	}

	qsd->irq = platform_get_irq(pdev, 0);
	if (qsd->irq < 0) {
		ret = qsd->irq;
		goto err_free;
	}

	init_completion(&qsd->done);

	ret = devm_request_irq(&pdev->dev, qsd->irq, qsd_spi_irq, 0,
			       dev_name(&pdev->dev), qsd);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ: %d\n", ret);
		goto err_free;
	}

	/* Enable clocks */
	ret = clk_prepare_enable(qsd->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable core clock: %d\n", ret);
		goto err_free;
	}

	if (qsd->pclk) {
		ret = clk_prepare_enable(qsd->pclk);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable iface clock: %d\n",
				ret);
			goto err_clk;
		}
	}

	/* Reset the controller */
	qsd_spi_write(qsd, QSD_SPI_SW_RESET, 1);
	usleep_range(100, 200);

	/* Clear and disable errors */
	qsd_spi_write(qsd, QSD_SPI_ERROR_FLAGS,
		      qsd_spi_read(qsd, QSD_SPI_ERROR_FLAGS));
	qsd_spi_write(qsd, QSD_SPI_ERROR_FLAGS_EN, 0);

	/* Set initial state */
	qsd_spi_write(qsd, QSD_SPI_OPERATIONAL, SPI_OP_STATE_RESET);

	/* Configure controller */
	ctlr->bus_num = pdev->id;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LOOP;
	ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 32);
	ctlr->num_chipselect = 4;
	ctlr->max_speed_hz = 50000000;
	ctlr->set_cs = qsd_spi_set_cs;
	ctlr->transfer_one = qsd_spi_transfer_one;
	ctlr->auto_runtime_pm = true;
	ctlr->dev.of_node = pdev->dev.of_node;

	/* Get max speed from device tree */
	of_property_read_u32(pdev->dev.of_node, "spi-max-frequency",
			     &ctlr->max_speed_hz);
	qsd->max_speed_hz = ctlr->max_speed_hz;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = spi_register_controller(ctlr);
	if (ret) {
		dev_err(&pdev->dev, "failed to register SPI controller: %d\n",
			ret);
		goto err_pm;
	}

	dev_info(&pdev->dev, "Qualcomm QSD SPI controller initialized\n");
	return 0;

err_pm:
	pm_runtime_disable(&pdev->dev);
	if (qsd->pclk)
		clk_disable_unprepare(qsd->pclk);
err_clk:
	clk_disable_unprepare(qsd->clk);
err_free:
	spi_controller_put(ctlr);
	return ret;
}

static void qsd_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct qsd_spi *qsd = spi_controller_get_devdata(ctlr);

	pm_runtime_disable(&pdev->dev);
	spi_unregister_controller(ctlr);

	if (qsd->pclk)
		clk_disable_unprepare(qsd->pclk);
	clk_disable_unprepare(qsd->clk);
}

static int __maybe_unused qsd_spi_runtime_suspend(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct qsd_spi *qsd = spi_controller_get_devdata(ctlr);

	if (qsd->pclk)
		clk_disable_unprepare(qsd->pclk);
	clk_disable_unprepare(qsd->clk);

	return 0;
}

static int __maybe_unused qsd_spi_runtime_resume(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct qsd_spi *qsd = spi_controller_get_devdata(ctlr);
	int ret;

	ret = clk_prepare_enable(qsd->clk);
	if (ret)
		return ret;

	if (qsd->pclk) {
		ret = clk_prepare_enable(qsd->pclk);
		if (ret) {
			clk_disable_unprepare(qsd->clk);
			return ret;
		}
	}

	return 0;
}

static const struct dev_pm_ops qsd_spi_pm_ops = {
	SET_RUNTIME_PM_OPS(qsd_spi_runtime_suspend, qsd_spi_runtime_resume,
			   NULL)
};

static const struct of_device_id qsd_spi_dt_ids[] = {
	{ .compatible = "qcom,spi-qsd" },
	{ }
};
MODULE_DEVICE_TABLE(of, qsd_spi_dt_ids);

static struct platform_driver qsd_spi_driver = {
	.driver = {
		.name = "spi-qcom-qsd",
		.pm = &qsd_spi_pm_ops,
		.of_match_table = qsd_spi_dt_ids,
	},
	.probe = qsd_spi_probe,
	.remove = qsd_spi_remove,
};
module_platform_driver(qsd_spi_driver);

MODULE_DESCRIPTION("Qualcomm MSM QSD SPI Controller Driver");
MODULE_LICENSE("GPL");
