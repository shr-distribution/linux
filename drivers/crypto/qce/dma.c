// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma/qcom_adm.h>
#include <linux/of.h>
#include <crypto/scatterwalk.h>

#include "core.h"
#include "dma.h"
#include "regs-ce2.h"

static void qce_dma_release(void *data)
{
	struct qce_dma_data *dma = data;

	dma_release_channel(dma->txchan);
	dma_release_channel(dma->rxchan);
	kfree(dma->result_buf);
}

static int qce_dma_configure_crci(struct qce_device *qce, struct dma_chan *chan,
				  u32 crci)
{
	struct qcom_adm_peripheral_config periph_conf = {};
	/* CE2 FIFO operates in 16-byte chunks per CRCI handshake.  maxburst is
	 * in dword units, so 4 dwords = 16 bytes matches CE2's burst size. v5
	 * BAM hardware historically uses 8-dword bursts so keep that branch.
	 */
	unsigned int burst = (qce->version == QCE_VERSION_CE2) ? 4 : 8;
	struct dma_slave_config conf = {
		.device_fc = true,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = burst,
		.dst_maxburst = burst,
		/*
		 * ADM box descriptors carry the peripheral-side bus address
		 * even when CRCI flow control is on, so they must point at
		 * the correct QCE registers.
		 *
		 * CE2 (MSM8x60): Input data goes to DATA_SHADOW (0x8000),
		 * a large shadow register block that ADM writes to via CRCI.
		 * Hash results are read from AUTH_IV0 (0x100) after CRCI fires.
		 * Using DATA_IN/DATA_OUT (0x00/0x10) causes CE2 to ignore the
		 * data and never signal AUTH_DONE.
		 *
		 * v5 (BAM-based): Uses DATA_IN @ +0x00 and DATA_OUT @ +0x10.
		 */
		.src_addr = (qce->version == QCE_VERSION_CE2) ?
				qce->phys_base + CE2_REG_AUTH_IV0 :
				qce->phys_base + 0x10,
		.dst_addr = (qce->version == QCE_VERSION_CE2) ?
				qce->phys_base + CE2_REG_DATA_SHADOW0 :
				qce->phys_base + 0x00,
	};

	if (!crci)
		return 0;

	periph_conf.crci = crci;
	conf.peripheral_config = &periph_conf;
	conf.peripheral_size = sizeof(periph_conf);

	return dmaengine_slave_config(chan, &conf);
}

int devm_qce_dma_request(struct qce_device *qce, struct qce_dma_data *dma)
{
	struct device *dev = qce->dev;
	int ret;

	/* Read CRCI values for QCOM ADM DMA flow control */
	of_property_read_u32(dev->of_node, "qcom,rx-crci", &dma->rx_crci);
	of_property_read_u32(dev->of_node, "qcom,tx-crci", &dma->tx_crci);

	dma->txchan = dma_request_chan(dev, "tx");
	if (IS_ERR(dma->txchan))
		return dev_err_probe(dev, PTR_ERR(dma->txchan),
				     "Failed to get TX DMA channel\n");

	/* Configure TX channel with CRCI for ADM flow control */
	ret = qce_dma_configure_crci(qce, dma->txchan, dma->tx_crci);
	if (ret) {
		dev_err(dev, "Failed to configure TX CRCI: %d\n", ret);
		goto error_tx_config;
	}

	dma->rxchan = dma_request_chan(dev, "rx");
	if (IS_ERR(dma->rxchan)) {
		ret = dev_err_probe(dev, PTR_ERR(dma->rxchan),
				    "Failed to get RX DMA channel\n");
		goto error_rx;
	}

	/* Configure RX channel with CRCI for ADM flow control */
	ret = qce_dma_configure_crci(qce, dma->rxchan, dma->rx_crci);
	if (ret) {
		dev_err(dev, "Failed to configure RX CRCI: %d\n", ret);
		goto error_rx_config;
	}

	dma->result_buf = kmalloc(QCE_RESULT_BUF_SZ + QCE_IGNORE_BUF_SZ,
				  GFP_KERNEL);
	if (!dma->result_buf) {
		ret = -ENOMEM;
		goto error_nomem;
	}

	dma->ignore_buf = dma->result_buf + QCE_RESULT_BUF_SZ;

	return devm_add_action_or_reset(dev, qce_dma_release, dma);

error_nomem:
error_rx_config:
	dma_release_channel(dma->rxchan);
error_rx:
error_tx_config:
	dma_release_channel(dma->txchan);
	return ret;
}

struct scatterlist *
qce_sgtable_add(struct sg_table *sgt, struct scatterlist *new_sgl,
		unsigned int max_len)
{
	struct scatterlist *sg = sgt->sgl, *sg_last = NULL;
	unsigned int new_len;

	while (sg) {
		if (!sg_page(sg))
			break;
		sg = sg_next(sg);
	}

	if (!sg)
		return ERR_PTR(-EINVAL);

	while (new_sgl && sg && max_len) {
		new_len = new_sgl->length > max_len ? max_len : new_sgl->length;
		sg_set_page(sg, sg_page(new_sgl), new_len, new_sgl->offset);
		sg_last = sg;
		sg = sg_next(sg);
		new_sgl = sg_next(new_sgl);
		max_len -= new_len;
	}

	return sg_last;
}

static int qce_dma_prep_sg(struct dma_chan *chan, struct scatterlist *sg,
			   int nents, unsigned long flags,
			   enum dma_transfer_direction dir,
			   dma_async_tx_callback cb, void *cb_param,
			   dma_cookie_t *cookie_out)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;

	if (!sg || !nents)
		return -EINVAL;

	desc = dmaengine_prep_slave_sg(chan, sg, nents, dir, flags);
	if (!desc)
		return -EINVAL;

	desc->callback = cb;
	desc->callback_param = cb_param;
	cookie = dmaengine_submit(desc);
	if (cookie_out)
		*cookie_out = cookie;

	return dma_submit_error(cookie);
}

/*
 * rxchan completion. In the success path the engine waits to produce
 * output until rxchan has finished pushing input, so the caller's
 * callback runs from the tx completion below. In the error path
 * (ADM rejects the rxchan submission with err=1) the engine never
 * produces output, the txchan completion never fires, and the user
 * request would otherwise hang in wait_for_completion for hung-task
 * timeouts. Detect that case here and forward the failure to the
 * caller's callback so the request gets failed cleanly.
 */
static void qce_dma_rx_callback(void *data)
{
	struct qce_dma_data *dma = data;
	enum dma_status status;

	status = dma_async_is_tx_complete(dma->rxchan, dma->rx_cookie,
					  NULL, NULL);
	if (status != DMA_ERROR)
		return;

	if (atomic_xchg(&dma->completion_done, 1) != 0)
		return;

	dmaengine_terminate_async(dma->txchan);
	if (dma->user_cb)
		dma->user_cb(dma->user_cb_param);
}

static void qce_dma_tx_callback(void *data)
{
	struct qce_dma_data *dma = data;

	if (atomic_xchg(&dma->completion_done, 1) != 0)
		return;

	if (dma->user_cb)
		dma->user_cb(dma->user_cb_param);
}

int qce_dma_prep_sgs(struct qce_dma_data *dma, struct scatterlist *rx_sg,
		     int rx_nents, struct scatterlist *tx_sg, int tx_nents,
		     dma_async_tx_callback cb, void *cb_param)
{
	struct dma_chan *rxchan = dma->rxchan;
	struct dma_chan *txchan = dma->txchan;
	unsigned long flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	int ret;

	atomic_set(&dma->completion_done, 0);
	dma->user_cb = cb;
	dma->user_cb_param = cb_param;

	ret = qce_dma_prep_sg(rxchan, rx_sg, rx_nents, flags, DMA_MEM_TO_DEV,
			      qce_dma_rx_callback, dma, &dma->rx_cookie);
	if (ret)
		return ret;

	return qce_dma_prep_sg(txchan, tx_sg, tx_nents, flags, DMA_DEV_TO_MEM,
			       qce_dma_tx_callback, dma, NULL);
}

void qce_dma_issue_pending(struct qce_dma_data *dma)
{
	dma_async_issue_pending(dma->rxchan);
	dma_async_issue_pending(dma->txchan);
}

int qce_dma_terminate_all(struct qce_dma_data *dma)
{
	int ret;

	ret = dmaengine_terminate_all(dma->rxchan);
	return ret ?: dmaengine_terminate_all(dma->txchan);
}
