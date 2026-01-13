// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm VIDC 1080p Video Codec driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Linux-SHR Project
 *
 * This driver supports the VIDC 1080p video codec found in MSM8660/APQ8060
 * SoCs. Unlike newer Venus cores, VIDC 1080p uses direct register-based
 * HOST2RISC/RISC2HOST command interface.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vidc_core.h"
#include "vidc_dec.h"
#include "vidc_enc.h"

#define VIDC_FW_NAME		"qcom/vidc_1080p.fw"
#define VIDC_FW_SIZE_MAX	(512 * 1024)

#define VIDC_INIT_CH_INST_ID	0x0000ffff

/* Interconnect bandwidth for 1080p video (in bytes/sec) */
#define VIDC_BW_AVG		(245 * 1024 * 1024)	/* 245 MB/s average */
#define VIDC_BW_PEAK		(500 * 1024 * 1024)	/* 500 MB/s peak */

/* Clock rates in Hz */
static const unsigned long vidc_clk_rates[] = {
	27000000,
	48000000,
	96000000,
	133330000,
	200000000,
};

static int vidc_clk_enable(struct vidc_core *core)
{
	int ret;

	ret = clk_prepare_enable(core->iface_clk);
	if (ret) {
		dev_err(core->dev, "failed to enable iface clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(core->core_clk);
	if (ret) {
		dev_err(core->dev, "failed to enable core clock: %d\n", ret);
		goto err_iface_clk;
	}

	ret = clk_prepare_enable(core->axi_clk);
	if (ret) {
		dev_err(core->dev, "failed to enable axi clock: %d\n", ret);
		goto err_core_clk;
	}

	return 0;

err_core_clk:
	clk_disable_unprepare(core->core_clk);
err_iface_clk:
	clk_disable_unprepare(core->iface_clk);
	return ret;
}

static void vidc_clk_disable(struct vidc_core *core)
{
	clk_disable_unprepare(core->axi_clk);
	clk_disable_unprepare(core->core_clk);
	clk_disable_unprepare(core->iface_clk);
}

int vidc_hw_reset(struct vidc_core *core)
{
	u32 axi_status;
	int timeout = 100;

	/* Stage 1: Reset VI, RISC, VIDCCORE, DMX */
	vidc_write(core, VIDC_REG_SW_RESET,
		   VIDC_RESET_NONE & ~VIDC_RESET_VI);
	vidc_write(core, VIDC_REG_SW_RESET,
		   VIDC_RESET_NONE & ~(VIDC_RESET_VI | VIDC_RESET_RISC));
	vidc_write(core, VIDC_REG_SW_RESET,
		   VIDC_RESET_NONE & ~(VIDC_RESET_VI | VIDC_RESET_RISC |
				       VIDC_RESET_VIDCCORE | VIDC_RESET_DMX));

	/* Stage 2: Full reset then bring RISC out of reset */
	vidc_write(core, VIDC_REG_SW_RESET, VIDC_RESET_ALL);
	vidc_write(core, VIDC_REG_SW_RESET, VIDC_RESET_RISC);

	/* Halt AXI */
	vidc_write(core, VIDC_REG_AXI_CTRL, VIDC_AXI_HALT_REQ);

	/* Wait for AXI halt acknowledgment */
	do {
		axi_status = vidc_read(core, VIDC_REG_AXI_STATUS);
		axi_status = (axi_status & VIDC_AXI_HALT_ACK_MASK) >> 2;
		if (axi_status == 0x3)
			break;
		usleep_range(100, 200);
	} while (--timeout > 0);

	if (timeout == 0) {
		dev_err(core->dev, "AXI halt timeout\n");
		return -ETIMEDOUT;
	}

	/* Reset AXI */
	vidc_write(core, VIDC_REG_AXI_CTRL, VIDC_AXI_RESET);
	vidc_write(core, VIDC_REG_AXI_CTRL, 0);

	/* Configure burst sizes */
	vidc_write(core, VIDC_REG_BURST_CONFIG, (8 << 8) | 8);

	/* Initialize channel instance IDs */
	vidc_write(core, VIDC_REG_CH0_INST_ID, VIDC_INIT_CH_INST_ID);
	vidc_write(core, VIDC_REG_CH1_INST_ID, VIDC_INIT_CH_INST_ID);

	/* Clear command registers */
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);
	vidc_write(core, VIDC_REG_HOST2RISC_CMD, VIDC_CMD_EMPTY);

	/* Release reset */
	vidc_write(core, VIDC_REG_SW_RESET, VIDC_RESET_NONE);

	return 0;
}

int vidc_send_cmd(struct vidc_core *core, u32 cmd, u32 arg1, u32 arg2,
		  u32 arg3, u32 arg4)
{
	vidc_write(core, VIDC_REG_HOST2RISC_CMD, VIDC_CMD_EMPTY);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG1, arg1);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG2, arg2);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG3, arg3);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG4, arg4);
	vidc_write(core, VIDC_REG_HOST2RISC_CMD, cmd);

	return 0;
}

int vidc_get_response(struct vidc_core *core, u32 *cmd, u32 *arg1,
		      u32 *arg2, u32 *arg3, u32 *arg4)
{
	*cmd = vidc_read(core, VIDC_REG_RISC2HOST_CMD);
	*arg1 = vidc_read(core, VIDC_REG_RISC2HOST_ARG1);
	*arg2 = vidc_read(core, VIDC_REG_RISC2HOST_ARG2);
	*arg3 = vidc_read(core, VIDC_REG_RISC2HOST_ARG3);
	*arg4 = vidc_read(core, VIDC_REG_RISC2HOST_ARG4);

	return 0;
}

static void vidc_clear_interrupt(struct vidc_core *core)
{
	vidc_write(core, VIDC_REG_INTERRUPT, 0);
}

static void vidc_handle_frame_done(struct vidc_core *core,
				   struct vidc_inst *inst)
{
	u32 display_y, display_c;

	/* Read decoded frame addresses */
	display_y = vidc_read(core, VIDC_REG_DEC_DISPLAY_Y);
	display_c = vidc_read(core, VIDC_REG_DEC_DISPLAY_C);

	dev_dbg(core->dev, "Frame done: Y=0x%x C=0x%x\n",
		display_y << VIDC_ADDR_SHIFT,
		display_c << VIDC_ADDR_SHIFT);

	/* Read the decoded frame size */
	inst->result_size = vidc_read(core, VIDC_REG_SEQ_FRAME_SIZE);
}

static void vidc_handle_enc_complete(struct vidc_core *core,
				     struct vidc_inst *inst)
{
	/* Read encoded frame size */
	inst->result_size = vidc_read(core, VIDC_REG_ENC_FRAME_SIZE);

	dev_dbg(core->dev, "Encode complete: size=%u\n", inst->result_size);
}

static void vidc_handle_seq_done(struct vidc_core *core,
				 struct vidc_inst *inst)
{
	/* Read sequence header info */
	inst->seq_height = vidc_read(core, VIDC_REG_SEQ_IMG_SIZE_Y);
	inst->seq_width = vidc_read(core, VIDC_REG_SEQ_IMG_SIZE_X);
	inst->min_dpb_count = vidc_read(core, VIDC_REG_SEQ_MIN_DPB);

	dev_dbg(core->dev, "Sequence done: %ux%u, min_dpb=%u\n",
		inst->seq_width, inst->seq_height, inst->min_dpb_count);

	inst->state = VIDC_STATE_SEQ_PARSED;
}

static irqreturn_t vidc_isr(int irq, void *data)
{
	struct vidc_core *core = data;
	struct vidc_inst *inst;
	u32 cmd, arg1, arg2, arg3, arg4;
	unsigned long flags;

	spin_lock_irqsave(&core->irqlock, flags);

	vidc_clear_interrupt(core);
	vidc_get_response(core, &cmd, &arg1, &arg2, &arg3, &arg4);

	inst = core->curr_inst;

	dev_dbg(core->dev, "VIDC IRQ: cmd=%u arg1=0x%x arg2=0x%x inst=%p\n",
		cmd, arg1, arg2, inst);

	switch (cmd) {
	case VIDC_RESP_SYS_INIT:
		dev_info(core->dev, "Firmware initialized\n");
		break;

	case VIDC_RESP_OPEN_CH:
		dev_dbg(core->dev, "Channel opened\n");
		if (inst) {
			inst->state = VIDC_STATE_OPEN;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_CLOSE_CH:
		dev_dbg(core->dev, "Channel closed\n");
		if (inst) {
			inst->state = VIDC_STATE_IDLE;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_SEQ_DONE:
		if (inst) {
			vidc_handle_seq_done(core, inst);
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_FRAME_DONE:
		if (inst) {
			vidc_handle_frame_done(core, inst);
			inst->error = 0;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_ENC_COMPLETE:
		if (inst) {
			vidc_handle_enc_complete(core, inst);
			inst->error = 0;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_INIT_BUFFERS:
		dev_dbg(core->dev, "Buffers initialized\n");
		if (inst) {
			inst->state = VIDC_STATE_RUNNING;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_FLUSH_DONE:
		dev_dbg(core->dev, "Flush done\n");
		if (inst)
			complete(&inst->done);
		break;

	case VIDC_RESP_ERROR:
		dev_err(core->dev, "Firmware error: 0x%x\n", arg2);
		if (inst) {
			inst->error = -EIO;
			inst->state = VIDC_STATE_ERROR;
			complete(&inst->done);
		}
		break;

	default:
		break;
	}

	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);

	spin_unlock_irqrestore(&core->irqlock, flags);

	return IRQ_HANDLED;
}

int vidc_load_firmware(struct vidc_core *core)
{
	int ret;

	if (core->fw_loaded)
		return 0;

	ret = request_firmware(&core->fw, VIDC_FW_NAME, core->dev);
	if (ret) {
		dev_err(core->dev, "failed to load firmware %s: %d\n",
			VIDC_FW_NAME, ret);
		return ret;
	}

	if (core->fw->size > VIDC_FW_SIZE_MAX) {
		dev_err(core->dev, "firmware too large: %zu > %d\n",
			core->fw->size, VIDC_FW_SIZE_MAX);
		ret = -EINVAL;
		goto err_release_fw;
	}

	core->fw_vaddr = dma_alloc_coherent(core->dev, core->fw->size,
					    &core->fw_dma_addr, GFP_KERNEL);
	if (!core->fw_vaddr) {
		ret = -ENOMEM;
		goto err_release_fw;
	}

	memcpy(core->fw_vaddr, core->fw->data, core->fw->size);
	core->fw_size = core->fw->size;

	/* Initialize memory controller with firmware address */
	vidc_write(core, VIDC_REG_DRAM_BASE_A, core->fw_dma_addr >> 11);
	vidc_write(core, VIDC_REG_DRAM_BASE_B, core->fw_dma_addr >> 11);

	/* Send SYS_INIT command */
	ret = vidc_send_cmd(core, VIDC_CMD_SYS_INIT, 0, 0, 0, 0);
	if (ret) {
		dev_err(core->dev, "failed to send SYS_INIT\n");
		goto err_free_dma;
	}

	core->fw_loaded = true;
	core->fw_version = vidc_read(core, VIDC_REG_FW_VERSION);
	dev_info(core->dev, "Firmware loaded, version 0x%08x\n",
		 core->fw_version);

	return 0;

err_free_dma:
	dma_free_coherent(core->dev, core->fw_size, core->fw_vaddr,
			  core->fw_dma_addr);
err_release_fw:
	release_firmware(core->fw);
	core->fw = NULL;
	return ret;
}

void vidc_unload_firmware(struct vidc_core *core)
{
	if (!core->fw_loaded)
		return;

	if (core->fw_vaddr) {
		dma_free_coherent(core->dev, core->fw_size, core->fw_vaddr,
				  core->fw_dma_addr);
		core->fw_vaddr = NULL;
	}

	if (core->fw) {
		release_firmware(core->fw);
		core->fw = NULL;
	}

	core->fw_loaded = false;
}

int vidc_core_init(struct vidc_core *core)
{
	int ret;

	ret = vidc_clk_enable(core);
	if (ret)
		return ret;

	ret = vidc_hw_reset(core);
	if (ret) {
		vidc_clk_disable(core);
		return ret;
	}

	ret = vidc_load_firmware(core);
	if (ret) {
		vidc_clk_disable(core);
		return ret;
	}

	return 0;
}

void vidc_core_deinit(struct vidc_core *core)
{
	vidc_unload_firmware(core);
	vidc_clk_disable(core);
}

static int vidc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vidc_core *core;
	struct resource *res;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	mutex_init(&core->lock);
	spin_lock_init(&core->irqlock);
	INIT_LIST_HEAD(&core->instances);

	/* Map registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	core->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(core->base))
		return PTR_ERR(core->base);

	/* Get IRQ */
	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0)
		return core->irq;

	ret = devm_request_irq(dev, core->irq, vidc_isr, IRQF_TRIGGER_HIGH,
			       "vidc", core);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		return ret;
	}

	/* Get clocks */
	core->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(core->core_clk)) {
		dev_err(dev, "failed to get core clock\n");
		return PTR_ERR(core->core_clk);
	}

	core->iface_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(core->iface_clk)) {
		dev_err(dev, "failed to get iface clock\n");
		return PTR_ERR(core->iface_clk);
	}

	core->axi_clk = devm_clk_get(dev, "axi");
	if (IS_ERR(core->axi_clk)) {
		dev_err(dev, "failed to get axi clock\n");
		return PTR_ERR(core->axi_clk);
	}

	/* Set initial clock rate */
	ret = clk_set_rate(core->core_clk, vidc_clk_rates[2]); /* 96 MHz */
	if (ret) {
		dev_err(dev, "failed to set core clock rate: %d\n", ret);
		return ret;
	}

	/* Get optional power domain regulator */
	core->gdsc = devm_regulator_get_optional(dev, "gdsc");
	if (IS_ERR(core->gdsc)) {
		if (PTR_ERR(core->gdsc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		core->gdsc = NULL;
	}

	/* Get optional interconnect path */
	core->icc_path = devm_of_icc_get(dev, "video-mem");
	if (IS_ERR(core->icc_path)) {
		ret = PTR_ERR(core->icc_path);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_dbg(dev, "interconnect not available: %d\n", ret);
		core->icc_path = NULL;
	}

	/* Register V4L2 device */
	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register V4L2 device: %d\n", ret);
		return ret;
	}

	/* Register decoder video device */
	ret = vidc_dec_register(core);
	if (ret) {
		dev_err(dev, "failed to register decoder: %d\n", ret);
		goto err_v4l2_unregister;
	}

	/* Register encoder video device */
	ret = vidc_enc_register(core);
	if (ret) {
		dev_err(dev, "failed to register encoder: %d\n", ret);
		goto err_dec_unregister;
	}

	platform_set_drvdata(pdev, core);

	pm_runtime_enable(dev);

	dev_info(dev, "Qualcomm VIDC 1080p driver probed\n");

	return 0;

err_dec_unregister:
	vidc_dec_unregister(core);
err_v4l2_unregister:
	v4l2_device_unregister(&core->v4l2_dev);
	return ret;
}

static void vidc_remove(struct platform_device *pdev)
{
	struct vidc_core *core = platform_get_drvdata(pdev);

	pm_runtime_disable(core->dev);
	vidc_enc_unregister(core);
	vidc_dec_unregister(core);
	vidc_core_deinit(core);
	v4l2_device_unregister(&core->v4l2_dev);
}

static int vidc_runtime_suspend(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);

	vidc_clk_disable(core);

	if (core->icc_path)
		icc_set_bw(core->icc_path, 0, 0);

	if (core->gdsc)
		regulator_disable(core->gdsc);

	return 0;
}

static int vidc_runtime_resume(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);
	int ret;

	if (core->gdsc) {
		ret = regulator_enable(core->gdsc);
		if (ret)
			return ret;
	}

	if (core->icc_path) {
		ret = icc_set_bw(core->icc_path, VIDC_BW_AVG, VIDC_BW_PEAK);
		if (ret) {
			dev_err(dev, "failed to set interconnect bandwidth: %d\n",
				ret);
			goto err_gdsc;
		}
	}

	ret = vidc_clk_enable(core);
	if (ret)
		goto err_icc;

	return 0;

err_icc:
	if (core->icc_path)
		icc_set_bw(core->icc_path, 0, 0);
err_gdsc:
	if (core->gdsc)
		regulator_disable(core->gdsc);
	return ret;
}

static const struct dev_pm_ops vidc_pm_ops = {
	SET_RUNTIME_PM_OPS(vidc_runtime_suspend, vidc_runtime_resume, NULL)
};

static const struct of_device_id vidc_of_match[] = {
	{ .compatible = "qcom,msm8660-vidc" },
	{ .compatible = "qcom,apq8060-vidc" },
	{ },
};
MODULE_DEVICE_TABLE(of, vidc_of_match);

static struct platform_driver vidc_driver = {
	.probe = vidc_probe,
	.remove = vidc_remove,
	.driver = {
		.name = "qcom-vidc",
		.of_match_table = vidc_of_match,
		.pm = &vidc_pm_ops,
	},
};

module_platform_driver(vidc_driver);

MODULE_DESCRIPTION("Qualcomm VIDC 1080p Video Codec driver");
MODULE_LICENSE("GPL v2");
MODULE_FIRMWARE(VIDC_FW_NAME);
