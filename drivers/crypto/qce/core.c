// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <crypto/algapi.h>
#include <crypto/internal/hash.h>
#include <linux/dma/qcom_adm.h>

#include "core.h"
#include "cipher.h"
#include "sha.h"
#include "aead.h"
#include "regs-ce2.h"

#define QCE_MAJOR_VERSION5	0x05
#define QCE_QUEUE_LENGTH	1

#define QCE_DEFAULT_MEM_BANDWIDTH	393600

/* Driver data for different CE versions */
struct qce_driver_data {
	enum qce_version version;
};

static const struct qce_algo_ops *qce_ops[] = {
#ifdef CONFIG_CRYPTO_DEV_QCE_SKCIPHER
	&skcipher_ops,
#endif
#ifdef CONFIG_CRYPTO_DEV_QCE_SHA
	&ahash_ops,
#endif
#ifdef CONFIG_CRYPTO_DEV_QCE_AEAD
	&aead_ops,
#endif
};

static void qce_unregister_algs(void *data)
{
	const struct qce_algo_ops *ops;
	struct qce_device *qce = data;
	int i;

	for (i = 0; i < ARRAY_SIZE(qce_ops); i++) {
		ops = qce_ops[i];
		ops->unregister_algs(qce);
	}
}

static int devm_qce_register_algs(struct qce_device *qce)
{
	const struct qce_algo_ops *ops;
	int i, j, ret = -ENODEV;

	for (i = 0; i < ARRAY_SIZE(qce_ops); i++) {
		ops = qce_ops[i];
		ret = ops->register_algs(qce);
		if (ret) {
			for (j = i - 1; j >= 0; j--)
				ops->unregister_algs(qce);
			return ret;
		}
	}

	return devm_add_action_or_reset(qce->dev, qce_unregister_algs, qce);
}

static int qce_handle_request(struct crypto_async_request *async_req)
{
	int ret = -EINVAL, i;
	const struct qce_algo_ops *ops;
	u32 type = crypto_tfm_alg_type(async_req->tfm);

	for (i = 0; i < ARRAY_SIZE(qce_ops); i++) {
		ops = qce_ops[i];
		if (type != ops->type)
			continue;
		ret = ops->async_req_handle(async_req);
		break;
	}

	return ret;
}

static int qce_handle_queue(struct qce_device *qce,
			    struct crypto_async_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	int ret = 0, err;

	scoped_guard(mutex, &qce->lock) {
		if (req)
			ret = crypto_enqueue_request(&qce->queue, req);

		/* busy, do not dequeue request */
		if (qce->req)
			return ret;

		backlog = crypto_get_backlog(&qce->queue);
		async_req = crypto_dequeue_request(&qce->queue);
		if (async_req)
			qce->req = async_req;
	}

	if (!async_req)
		return ret;

	if (backlog) {
		scoped_guard(mutex, &qce->lock)
			crypto_request_complete(backlog, -EINPROGRESS);
	}

	err = qce_handle_request(async_req);
	if (err) {
		qce->result = err;
		schedule_work(&qce->done_work);
	}

	return ret;
}

static void qce_req_done_work(struct work_struct *work)
{
	struct qce_device *qce = container_of(work, struct qce_device,
					      done_work);
	struct crypto_async_request *req;

	scoped_guard(mutex, &qce->lock) {
		req = qce->req;
		qce->req = NULL;
	}

	if (req)
		crypto_request_complete(req, qce->result);

	qce_handle_queue(qce, NULL);
}

static int qce_async_request_enqueue(struct qce_device *qce,
				     struct crypto_async_request *req)
{
	return qce_handle_queue(qce, req);
}

static void qce_async_request_done(struct qce_device *qce, int ret)
{
	qce->result = ret;
	schedule_work(&qce->done_work);
}

static int qce_check_version(struct qce_device *qce)
{
	u32 major, minor, step;

	if (qce->version == QCE_VERSION_CE2) {
		/*
		 * CE2 doesn't have a version register at offset 0x000.
		 * Version is read from STATUS register bits 31-28.
		 * We trust the device tree compatible string for version.
		 */
		qce->burst_size = CE2_ADM_BURST_SIZE;
		qce->pipe_pair_id = 0; /* CE2 uses ADM, not BAM pipe pairs */

		dev_info(qce->dev, "Crypto Engine 2 (CE2) found\n");
		return 0;
	}

	qce_get_version(qce, &major, &minor, &step);

	/*
	 * the driver does not support v5 with minor 0 because it has special
	 * alignment requirements.
	 */
	if (major == 5 && minor == 0)
		return -ENODEV;

	qce->version = QCE_VERSION_5;
	qce->burst_size = QCE_BAM_BURST_SIZE;

	/*
	 * Rx and tx pipes are treated as a pair inside CE.
	 * Pipe pair number depends on the actual BAM dma pipe
	 * that is used for transfers. The BAM dma pipes are passed
	 * from the device tree and used to derive the pipe pair
	 * id in the CE driver as follows.
	 * 	BAM dma pipes(rx, tx)		CE pipe pair id
	 *		0,1				0
	 *		2,3				1
	 *		4,5				2
	 *		6,7				3
	 *		...
	 */
	qce->pipe_pair_id = qce->dma.rxchan->chan_id >> 1;

	dev_dbg(qce->dev, "Crypto device found, version %d.%d.%d\n",
		major, minor, step);

	return 0;
}


static int qce_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct qce_driver_data *drvdata;
	struct qce_device *qce;
	int ret;

	qce = devm_kzalloc(dev, sizeof(*qce), GFP_KERNEL);
	if (!qce)
		return -ENOMEM;

	qce->dev = dev;
	platform_set_drvdata(pdev, qce);

	/* Get version from driver data if available */
	drvdata = of_device_get_match_data(dev);
	if (drvdata)
		qce->version = drvdata->version;

	{
		struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -ENODEV;
		qce->phys_base = res->start;
	}

	qce->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(qce->base))
		return PTR_ERR(qce->base);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret < 0)
		return ret;

	qce->core = devm_clk_get_optional_enabled(qce->dev, "core");
	if (IS_ERR(qce->core))
		return PTR_ERR(qce->core);

	qce->iface = devm_clk_get_optional_enabled(qce->dev, "iface");
	if (IS_ERR(qce->iface))
		return PTR_ERR(qce->iface);

	qce->bus = devm_clk_get_optional_enabled(qce->dev, "bus");
	if (IS_ERR(qce->bus))
		return PTR_ERR(qce->bus);

	qce->reset = devm_reset_control_get_optional_exclusive(qce->dev, "engine");
	if (IS_ERR(qce->reset))
		return PTR_ERR(qce->reset);

	/*
	 * CE2 Hardware Initialization (MSM8x60 family - MSM8260/MSM8660/APQ8060)
	 *
	 * On some devices (e.g., HP TouchPad), the bootloader OEMSBL/TrustZone
	 * stage is missing or incomplete, so CE2 peripheral is never properly
	 * initialized. This workaround verifies hardware responds to MMIO reads.
	 *
	 * The Clock Control Framework (CCF) handles CE2_P_CLK enable via
	 * the device tree clock bindings. This code only verifies the hardware
	 * is accessible after CCF enables the clocks.
	 *
	 * Verbose logging helps debug CE2 on other MSM8x60 platforms
	 * where bootloader initialization may differ.
	 *
	 * Reference: reports/ce2-investigation/CE2-BREAKTHROUGH-SUCCESS.md
	 */
	if (qce->version == QCE_VERSION_CE2) {
		u32 val, status;

		dev_dbg(dev, "CE2: Verifying hardware initialization\n");

		/*
		 * Verify CE2 MMIO is accessible. If all registers read as zero,
		 * the hardware may not have been initialized by the bootloader.
		 *
		 * On working systems, register 0x00 contains version/config info,
		 * 0x10 contains status, and 0x20 contains hardware capabilities.
		 */
		val = readl_relaxed(qce->base + 0x00);
		dev_dbg(dev, "CE2: Version register (0x00): 0x%08x\n", val);

		if (val == 0) {
			dev_dbg(dev, "CE2: Version register is zero, checking other registers...\n");

			val = readl_relaxed(qce->base + 0x10);
			dev_dbg(dev, "CE2: Status register (0x10): 0x%08x\n", val);

			val = readl_relaxed(qce->base + 0x20);
			dev_dbg(dev, "CE2: Capabilities register (0x20): 0x%08x\n", val);

			if (readl_relaxed(qce->base + 0x00) == 0 &&
			    readl_relaxed(qce->base + 0x10) == 0 &&
			    readl_relaxed(qce->base + 0x20) == 0) {
				dev_err(dev, "CE2: Hardware not responding - all registers read as zero\n");
				dev_err(dev, "CE2: This may indicate incomplete bootloader initialization\n");
				dev_err(dev, "CE2: or eFuse lockout. Crypto operations will likely fail.\n");
				return -EIO;
			}

			dev_dbg(dev, "CE2: Hardware accessible despite zero version register\n");
		}

		/*
		 * Initialize CE2 CONFIG register. Perform a proper SW_RST pulse
		 * to clear SW_ERR (STATUS bit 0) which is set at boot on TouchPad
		 * because the bootloader never initializes CE2.
		 *
		 * Without this cycle: SW_ERR stays set → CE2 never processes data
		 * → CRCI-CE_OUT never fires → ADM TX DMA hangs forever.
		 */
		/* Assert SW_RST (keep CLK_EN_N=0 so clock stays enabled) */
		writel_relaxed(BIT(CE2_SW_RST_SHIFT), qce->base + CE2_REG_CONFIG);
		usleep_range(10, 20);
		/* Deassert SW_RST */
		writel_relaxed(0, qce->base + CE2_REG_CONFIG);
		usleep_range(10, 20);

		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		dev_dbg(dev, "CE2: STATUS after SW_RST: 0x%08x (SW_ERR=%d)\n",
			status, !!(status & BIT(CE2_SW_ERR_SHIFT)));

		/* ENGINES_AVAIL @ +0x044: which crypto engines are physically
		 * implemented on this die. Log each engine so we know what's
		 * usable on this specific MSM8x60 silicon.
		 */
		val = readl_relaxed(qce->base + CE2_REG_ENGINES_AVAIL);
		dev_dbg(dev, "CE2: ENGINES_AVAIL=0x%08x:%s%s%s%s%s%s%s%s\n",
			 val,
			 (val & CE2_AES_SEL_MASK) == CE2_AES_SEL_FAST ? " AES(fast)" :
			   (val & CE2_AES_SEL_MASK) == CE2_AES_SEL_SLOW ? " AES(slow)" :
			   "",
			 (val & BIT(CE2_C2_SEL_SHIFT)) ? " C2" : "",
			 (val & BIT(CE2_DES_SEL_SHIFT)) ? " DES/3DES" : "",
			 (val & BIT(CE2_SHA_SEL_SHIFT)) ? " SHA1/SHA256" : "",
			 (val & BIT(CE2_SHA512_SEL_SHIFT)) ? " SHA384/SHA512" : "",
			 (val & BIT(CE2_HMAC_SEL_SHIFT)) ? " HMAC" : "",
			 (val & BIT(CE2_F8_SEL_SHIFT)) ? " F8" : "",
			 (val & BIT(CE2_F9_SEL_SHIFT)) ? " F9" : "");
	}

	/* Interconnect is optional - CE2 uses RPM for bus voting */
	qce->mem_path = devm_of_icc_get(qce->dev, "memory");
	if (IS_ERR(qce->mem_path)) {
		if (PTR_ERR(qce->mem_path) != -ENODATA)
			return PTR_ERR(qce->mem_path);
		qce->mem_path = NULL;
	}

	if (qce->mem_path) {
		ret = icc_set_bw(qce->mem_path, QCE_DEFAULT_MEM_BANDWIDTH,
				 QCE_DEFAULT_MEM_BANDWIDTH);
		if (ret)
			return ret;
	}

	ret = devm_qce_dma_request(qce, &qce->dma);
	if (ret)
		return ret;

	/*
	 * NOTE: On CE2 (MSM8x60) the per-channel CRCI burst size needs to
	 * be programmed at EE=0 before the first transfer (CRCI 4 = CE_IN,
	 * CRCI 5 = CE_OUT, burst = 32 bytes / 2). The original out-of-tree
	 * version of this driver called qcom_adm_program_crci_ee0() to do
	 * that, but no such helper exists in mainline qcom-adm.c. Wiring
	 * this up properly belongs in drivers/dma/qcom/qcom-adm.c (likely
	 * as a per-channel DT property the ADM driver applies in its own
	 * probe), not here. Left as a TODO for the qce CE2 follow-up
	 * series; the existing driver behaves correctly when the previous
	 * boot stage has left CRCI burst at its default.
	 */

	ret = qce_check_version(qce);
	if (ret)
		return ret;

	ret = devm_mutex_init(qce->dev, &qce->lock);
	if (ret)
		return ret;

	INIT_WORK(&qce->done_work, qce_req_done_work);
	crypto_init_queue(&qce->queue, QCE_QUEUE_LENGTH);

	qce->async_req_enqueue = qce_async_request_enqueue;
	qce->async_req_done = qce_async_request_done;

	return devm_qce_register_algs(qce);
}

static const struct qce_driver_data qce_ce2_data = {
	.version = QCE_VERSION_CE2,
};

static const struct qce_driver_data qce_v5_data = {
	.version = QCE_VERSION_5,
};

static const struct of_device_id qce_crypto_of_match[] = {
	{ .compatible = "qcom,msm8660-qce", .data = &qce_ce2_data },
	{ .compatible = "qcom,crypto-v5.1", .data = &qce_v5_data },
	{ .compatible = "qcom,crypto-v5.4", .data = &qce_v5_data },
	{ .compatible = "qcom,qce", .data = &qce_v5_data },
	{}
};
MODULE_DEVICE_TABLE(of, qce_crypto_of_match);

static struct platform_driver qce_crypto_driver = {
	.probe = qce_crypto_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = qce_crypto_of_match,
	},
};
module_platform_driver(qce_crypto_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm crypto engine driver");
MODULE_ALIAS("platform:" KBUILD_MODNAME);
MODULE_AUTHOR("The Linux Foundation");
