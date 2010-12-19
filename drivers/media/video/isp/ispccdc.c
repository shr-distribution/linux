/*
 * ispccdc.c
 *
 * Driver Library for CCDC module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Pallavi Kulkarni <p-kulkarni@ti.com>
 *	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <media/v4l2-event.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"

static struct v4l2_mbus_framefmt *
__ccdc_get_format(struct isp_ccdc_device *ccdc, struct v4l2_subdev_fh *fh,
		  unsigned int pad, enum v4l2_subdev_format which);

/* Structure for saving/restoring CCDC module registers*/
static struct isp_reg ispccdc_reg_list[] = {
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HD_VD_WID, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PIX_LINES, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HORZ_INFO, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_START, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_LINES, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CULLING, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_DCSUB, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_COLPTN, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_BLKCMP, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC_ADDR, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VDINT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_REC656IF, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_HORZ, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_VERT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR2, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR3, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR4, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR5, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR6, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR7, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VP_OUT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_INITIAL, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_OFFSET, 0},
	{0, ISP_TOK_TERM, 0}
};

const static unsigned int ccdc_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
};

/*
 * ispccdc_save_context - Save values of the CCDC module registers
 * @dev: Device pointer specific to the OMAP3 ISP.
 */
void ispccdc_save_context(struct isp_device *isp)
{
	isp_save_context(isp, ispccdc_reg_list);
}

/*
 * ispccdc_restore_context - Restore values of the CCDC module registers
 * @dev: Pointer to ISP device
 */
void ispccdc_restore_context(struct isp_device *isp)
{
	isp_restore_context(isp, ispccdc_reg_list);
}

/*
 * ispccdc_print_status - Print current CCDC Module register values.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Also prints other debug information stored in the CCDC module.
 */
#define CCDC_PRINT_REGISTER(isp, name)\
	dev_dbg(isp->dev, "###CCDC " #name "=0x%08x\n", \
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_##name))

static void ispccdc_print_status(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	dev_dbg(isp->dev, "-------------CCDC Register dump-------------\n");

	CCDC_PRINT_REGISTER(isp, PCR);
	CCDC_PRINT_REGISTER(isp, SYN_MODE);
	CCDC_PRINT_REGISTER(isp, HD_VD_WID);
	CCDC_PRINT_REGISTER(isp, PIX_LINES);
	CCDC_PRINT_REGISTER(isp, HORZ_INFO);
	CCDC_PRINT_REGISTER(isp, VERT_START);
	CCDC_PRINT_REGISTER(isp, VERT_LINES);
	CCDC_PRINT_REGISTER(isp, CULLING);
	CCDC_PRINT_REGISTER(isp, HSIZE_OFF);
	CCDC_PRINT_REGISTER(isp, SDOFST);
	CCDC_PRINT_REGISTER(isp, SDR_ADDR);
	CCDC_PRINT_REGISTER(isp, CLAMP);
	CCDC_PRINT_REGISTER(isp, DCSUB);
	CCDC_PRINT_REGISTER(isp, COLPTN);
	CCDC_PRINT_REGISTER(isp, BLKCMP);
	CCDC_PRINT_REGISTER(isp, FPC);
	CCDC_PRINT_REGISTER(isp, FPC_ADDR);
	CCDC_PRINT_REGISTER(isp, VDINT);
	CCDC_PRINT_REGISTER(isp, ALAW);
	CCDC_PRINT_REGISTER(isp, REC656IF);
	CCDC_PRINT_REGISTER(isp, CFG);
	CCDC_PRINT_REGISTER(isp, FMTCFG);
	CCDC_PRINT_REGISTER(isp, FMT_HORZ);
	CCDC_PRINT_REGISTER(isp, FMT_VERT);
	CCDC_PRINT_REGISTER(isp, PRGEVEN0);
	CCDC_PRINT_REGISTER(isp, PRGEVEN1);
	CCDC_PRINT_REGISTER(isp, PRGODD0);
	CCDC_PRINT_REGISTER(isp, PRGODD1);
	CCDC_PRINT_REGISTER(isp, VP_OUT);
	CCDC_PRINT_REGISTER(isp, LSC_CONFIG);
	CCDC_PRINT_REGISTER(isp, LSC_INITIAL);
	CCDC_PRINT_REGISTER(isp, LSC_TABLE_BASE);
	CCDC_PRINT_REGISTER(isp, LSC_TABLE_OFFSET);

	dev_dbg(isp->dev, "--------------------------------------------\n");
}

/*
 * ispccdc_busy - Get busy state of the CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 */
int ispccdc_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR) &
		ISPCCDC_PCR_BUSY;
}

/* -----------------------------------------------------------------------------
 * Lens Shading Compensation
 */

/*
 * ispccdc_lsc_validate_config - Check that LSC configuration is valid.
 * @ccdc: Pointer to ISP CCDC device.
 * @lsc_cfg: the LSC configuration to check.
 *
 * Returns 0 if the LSC configuration is valid, or -EINVAL if invalid.
 */
static int ispccdc_lsc_validate_config(struct isp_ccdc_device *ccdc,
				       struct ispccdc_lsc_config *lsc_cfg)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct v4l2_mbus_framefmt *format;
	unsigned int paxel_width, paxel_height;
	unsigned int paxel_shift_x, paxel_shift_y;
	unsigned int min_width, min_height, min_size;
	unsigned int input_width, input_height;

	paxel_shift_x = lsc_cfg->gain_mode_m;
	paxel_shift_y = lsc_cfg->gain_mode_n;

	if ((paxel_shift_x < 2) || (paxel_shift_x > 6) ||
	    (paxel_shift_y < 2) || (paxel_shift_y > 6)) {
		dev_dbg(isp->dev, "CCDC: LSC: Invalid paxel size\n");
		return -EINVAL;
	}

	if (lsc_cfg->offset & 3) {
		dev_dbg(isp->dev, "CCDC: LSC: Offset must be a multiple of "
			"4\n");
		return -EINVAL;
	}

	if ((lsc_cfg->initial_x & 1) || (lsc_cfg->initial_y & 1)) {
		dev_dbg(isp->dev, "CCDC: LSC: initial_x and y must be even\n");
		return -EINVAL;
	}

	format = __ccdc_get_format(ccdc, NULL, CCDC_PAD_SINK,
				   V4L2_SUBDEV_FORMAT_ACTIVE);
	input_width = format->width;
	input_height = format->height;

	/* Calculate minimum bytesize for validation */
	paxel_width = 1 << paxel_shift_x;
	min_width = ((input_width + lsc_cfg->initial_x + paxel_width - 1)
		     >> paxel_shift_x) + 1;

	paxel_height = 1 << paxel_shift_y;
	min_height = ((input_height + lsc_cfg->initial_y + paxel_height - 1)
		     >> paxel_shift_y) + 1;

	min_size = 4 * min_width * min_height;
	if (min_size > lsc_cfg->size) {
		dev_dbg(isp->dev, "CCDC: LSC: too small table\n");
		return -EINVAL;
	}
	if (lsc_cfg->offset < (min_width * 4)) {
		dev_dbg(isp->dev, "CCDC: LSC: Offset is too small\n");
		return -EINVAL;
	}
	if ((lsc_cfg->size / lsc_cfg->offset) < min_height) {
		dev_dbg(isp->dev, "CCDC: LSC: Wrong size/offset combination\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * ispccdc_lsc_program_table - Program Lens Shading Compensation table address.
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ispccdc_lsc_program_table(struct isp_ccdc_device *ccdc, u32 addr)
{
	isp_reg_writel(to_isp_device(ccdc), addr,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE);
}

/*
 * ispccdc_lsc_setup_regs - Configures the lens shading compensation module
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ispccdc_lsc_setup_regs(struct isp_ccdc_device *ccdc,
				   struct ispccdc_lsc_config *cfg)
{
	struct isp_device *isp = to_isp_device(ccdc);
	int reg;

	isp_reg_writel(isp, cfg->offset, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_TABLE_OFFSET);

	reg = 0;
	reg |= cfg->gain_mode_n << ISPCCDC_LSC_GAIN_MODE_N_SHIFT;
	reg |= cfg->gain_mode_m << ISPCCDC_LSC_GAIN_MODE_M_SHIFT;
	reg |= cfg->gain_format << ISPCCDC_LSC_GAIN_FORMAT_SHIFT;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG);

	reg = 0;
	reg &= ~ISPCCDC_LSC_INITIAL_X_MASK;
	reg |= cfg->initial_x << ISPCCDC_LSC_INITIAL_X_SHIFT;
	reg &= ~ISPCCDC_LSC_INITIAL_Y_MASK;
	reg |= cfg->initial_y << ISPCCDC_LSC_INITIAL_Y_SHIFT;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_INITIAL);
}

static int ispccdc_lsc_wait_prefetch(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	unsigned int wait;

	isp_reg_writel(isp, IRQ0STATUS_CCDC_LSC_PREF_COMP_IRQ,
		       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);

	/* timeout 1 ms */
	for (wait = 0; wait < 1000; wait++) {
		if (isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS) &
				  IRQ0STATUS_CCDC_LSC_PREF_COMP_IRQ) {
			isp_reg_writel(isp, IRQ0STATUS_CCDC_LSC_PREF_COMP_IRQ,
				       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
			return 0;
		}

		rmb();
		udelay(1);
	}

	return -ETIMEDOUT;
}

/*
 * __ispccdc_lsc_enable - Enables/Disables the Lens Shading Compensation module.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LSC, 1 Enables LSC.
 */
static int __ispccdc_lsc_enable(struct isp_ccdc_device *ccdc, int enable)
{
	struct isp_device *isp = to_isp_device(ccdc);
	const struct v4l2_mbus_framefmt *format =
		__ccdc_get_format(ccdc, NULL, CCDC_PAD_SINK,
				  V4L2_SUBDEV_FORMAT_ACTIVE);

	if ((format->code != V4L2_MBUS_FMT_SGRBG10_1X10) &&
	    (format->code != V4L2_MBUS_FMT_SRGGB10_1X10) &&
	    (format->code != V4L2_MBUS_FMT_SBGGR10_1X10) &&
	    (format->code != V4L2_MBUS_FMT_SGBRG10_1X10))
		return -EINVAL;

	if (enable)
		isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_LSC_READ);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG,
		       ~ISPCCDC_LSC_ENABLE, enable ? ISPCCDC_LSC_ENABLE : 0);

	if (enable) {
		if (ispccdc_lsc_wait_prefetch(ccdc) < 0) {
			isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC,
				    ISPCCDC_LSC_CONFIG, ~ISPCCDC_LSC_ENABLE);
			ccdc->lsc.state = LSC_STATE_STOPPED;
			dev_warn(to_device(ccdc), "LSC prefecth timeout\n");
			return -ETIMEDOUT;
		}
		ccdc->lsc.state = LSC_STATE_RUNNING;
	} else {
		ccdc->lsc.state = LSC_STATE_STOPPING;
	}

	return 0;
}

static int ispccdc_lsc_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG) &
			     ISPCCDC_LSC_BUSY;
}

/* __ispccdc_lsc_configure - Configure LSC engine with new configuration
 * and/or table in interrupt context
 * @ccdc: Pointer to ISP CCDC device.
 *
 */
static int __ispccdc_lsc_configure(struct isp_ccdc_device *ccdc,
				   struct ispccdc_lsc_config_req *req)
{
	if (!req->enable)
		return 0;

	if (ispccdc_lsc_busy(ccdc))
		return -EBUSY;

	ispccdc_lsc_setup_regs(ccdc, &req->config);
	ispccdc_lsc_program_table(ccdc, req->table);
	return 0;
}

/*
 * ispccdc_lsc_error_handler - Handle LSC prefetch error scenario.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Disables LSC, and defers enablement to shadow registers update time.
 */
static void ispccdc_lsc_error_handler(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	/*
	 * From OMAP3 TRM: When this event is pending, the module
	 * goes into transparent mode (output =input). Normal
	 * operation can be resumed at the start of the next frame
	 * after:
	 *  1) Clearing this event
	 *  2) Disabling the LSC module
	 *  3) Enabling it
	 */
	isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC,
		    ISPCCDC_LSC_CONFIG, ~ISPCCDC_LSC_ENABLE);
	ccdc->lsc.state = LSC_STATE_STOPPED;
}

static int ispccdc_lsc_queue_req(struct isp_ccdc_device *ccdc,
				 struct ispccdc_lsc_config *cfg,
				 u32 table, int enable)
{
	struct ispccdc_lsc_config_req *req;
	unsigned long flags;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (req == NULL)
		return -ENOMEM;

	if (enable) {
		req->config = *cfg;
		req->table = table;
		req->enable = 1;
	}

	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	list_add_tail(&req->list, &ccdc->lsc.req_queue);
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
	return 0;
}

static struct ispccdc_lsc_config_req *
ispccdc_lsc_dequeue_req(struct isp_ccdc_device *ccdc, struct list_head *queue)
{
	if (list_empty(queue))
		return NULL;

	return list_first_entry(queue, struct ispccdc_lsc_config_req, list);
}

static void ispccdc_lsc_init_queue(struct isp_ccdc_device *ccdc)
{
	INIT_LIST_HEAD(&ccdc->lsc.req_queue);
	INIT_LIST_HEAD(&ccdc->lsc.free_queue);
}

static void ispccdc_lsc_free_queue(struct isp_ccdc_device *ccdc,
				   struct list_head *queue)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct ispccdc_lsc_config_req *req, *n;
	unsigned long flags;

	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	list_for_each_entry_safe(req, n, queue, list) {
		list_del(&req->list);
		spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
		if (req->table)
			iommu_vfree(isp->iommu, req->table);
		kfree(req);
		spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	}
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
}

static void ispccdc_lsc_free_table_work(struct work_struct *work)
{
	struct isp_ccdc_device *ccdc;
	struct ispccdc_lsc *lsc;

	lsc = container_of(work, struct ispccdc_lsc, table_work);
	ccdc = container_of(lsc, struct isp_ccdc_device, lsc);

	ispccdc_lsc_free_queue(ccdc, &lsc->free_queue);
}

static int ispccdc_lsc_config(struct isp_ccdc_device *ccdc,
			      struct ispccdc_update_config *config)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct ispccdc_lsc_config cfg;
	void *table_new_va;
	u32 table_new = 0;
	int enable = 0;
	u16 update;
	int ret;

	update = config->update & (ISP_ABS_CCDC_CONFIG_LSC | ISP_ABS_TBL_LSC);
	if (!update)
		return 0;

	if (update != (ISP_ABS_CCDC_CONFIG_LSC | ISP_ABS_TBL_LSC)) {
		dev_dbg(to_device(ccdc), "%s: Both LSC configuration and table "
			"need to be supplied\n", __func__);
		return -EINVAL;
	}

	if (config->flag & ISP_ABS_CCDC_CONFIG_LSC) {
		if (copy_from_user(&cfg, config->lsc_cfg, sizeof(cfg)))
			return -EFAULT;
		if (ispccdc_lsc_validate_config(ccdc, &cfg))
			return -EINVAL;
		enable = 1;
	} else {
		goto queue;
	}

	table_new = iommu_vmalloc(isp->iommu, 0, cfg.size, IOMMU_FLAG);
	if (IS_ERR_VALUE(table_new))
		return -ENOMEM;
	table_new_va = da_to_va(isp->iommu, table_new);
	if (copy_from_user(table_new_va, config->lsc, cfg.size)) {
		iommu_vfree(isp->iommu, table_new);
		return -EFAULT;
	}

queue:
	ret = ispccdc_lsc_queue_req(ccdc, &cfg, table_new, enable);
	if (ret < 0 && table_new)
		iommu_vfree(isp->iommu, table_new);
	return ret;
}

static inline int ispccdc_lsc_is_configured(struct isp_ccdc_device *ccdc)
{
	struct ispccdc_lsc_config_req *req;
	unsigned long flags;

	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	req = ccdc->lsc.active;
	if (req && req->enable) {
		spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
		return 1;
	}
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
	return 0;
}

/* -----------------------------------------------------------------------------
 * Parameters configuration
 */

/*
 * ispccdc_config_black_clamp - Configures the clamp parameters in CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 * @bclamp: Structure containing the optical black average gain, optical black
 *          sample length, sample lines, and the start pixel position of the
 *          samples w.r.t the HS pulse.
 *
 * Configures the clamp parameters in CCDC. Either if its being used the
 * optical black clamp, or the digital clamp. If its a digital clamp, then
 * assures to put a valid DC substraction level.
 *
 * Returns always 0 when completed.
 */
static int ispccdc_config_black_clamp(struct isp_ccdc_device *ccdc,
				      struct ispccdc_bclamp *bclamp)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 bclamp_val;

	if (ccdc->obclamp_en) {
		bclamp_val  = bclamp->obgain << ISPCCDC_CLAMP_OBGAIN_SHIFT;
		bclamp_val |= bclamp->oblen << ISPCCDC_CLAMP_OBSLEN_SHIFT;
		bclamp_val |= bclamp->oblines << ISPCCDC_CLAMP_OBSLN_SHIFT;
		bclamp_val |= bclamp->obstpixel << ISPCCDC_CLAMP_OBST_SHIFT;
		isp_reg_writel(isp, bclamp_val,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP);
	} else {
		isp_reg_writel(isp, bclamp->dcsubval,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_DCSUB);
	}
	return 0;
}

/*
 * ispccdc_enable_black_clamp - Enables/Disables the optical black clamp.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables optical black clamp, 1 Enables optical black clamp.
 *
 * Enables or disables the optical black clamp. When disabled, the digital
 * clamp operates.
 */
static void ispccdc_enable_black_clamp(struct isp_ccdc_device *ccdc,
				       u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP,
		       ~ISPCCDC_CLAMP_CLAMPEN,
		       enable ? ISPCCDC_CLAMP_CLAMPEN : 0);
	ccdc->obclamp_en = enable;
}

/*
 * ispccdc_config_fpc - Configures the Faulty Pixel Correction parameters.
 * @ccdc: Pointer to ISP CCDC device.
 * @fpc: Structure containing the number of faulty pixels corrected in the
 *       frame, address of the FPC table.
 *
 * Returns 0 if successful, or -EINVAL if FPC Address is not on the 64 byte
 * boundary.
 */
static int ispccdc_config_fpc(struct isp_ccdc_device *ccdc,
			      struct ispccdc_fpc *fpc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 fpc_val = 0;

	fpc_val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);

	isp_reg_writel(isp, fpc_val & (~ISPCCDC_FPC_FPCEN),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);
	isp_reg_writel(isp, fpc->fpcaddr,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC_ADDR);
	isp_reg_writel(isp, fpc_val | (fpc->fpnum << ISPCCDC_FPC_FPNUM_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);
	return 0;
}

/*
 * ispccdc_enable_fpc - Enable Faulty Pixel Correction.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables FPC, 1 Enables FPC.
 */
static void ispccdc_enable_fpc(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC,
		       ~ISPCCDC_FPC_FPCEN, enable ? ISPCCDC_FPC_FPCEN : 0);
}

/*
 * ispccdc_config_black_comp - Configure Black Level Compensation.
 * @ccdc: Pointer to ISP CCDC device.
 * @blcomp: Structure containing the black level compensation value for RGrGbB
 *          pixels. in 2's complement.
 */
static void ispccdc_config_black_comp(struct isp_ccdc_device *ccdc,
				      struct ispccdc_blcomp *blcomp)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 blcomp_val = 0;

	blcomp_val |= blcomp->b_mg << ISPCCDC_BLKCMP_B_MG_SHIFT;
	blcomp_val |= blcomp->gb_g << ISPCCDC_BLKCMP_GB_G_SHIFT;
	blcomp_val |= blcomp->gr_cy << ISPCCDC_BLKCMP_GR_CY_SHIFT;
	blcomp_val |= blcomp->r_ye << ISPCCDC_BLKCMP_R_YE_SHIFT;

	isp_reg_writel(isp, blcomp_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_BLKCMP);
}

/*
 * ispccdc_config_culling - Configure culling parameters.
 * @ccdc: Pointer to ISP CCDC device.
 * @cull: Structure containing the vertical culling pattern, and horizontal
 *        culling pattern for odd and even lines.
 */
static void ispccdc_config_culling(struct isp_ccdc_device *ccdc,
				   struct ispccdc_culling *cull)
{
	struct isp_device *isp = to_isp_device(ccdc);

	u32 culling_val = 0;

	culling_val |= cull->v_pattern << ISPCCDC_CULLING_CULV_SHIFT;
	culling_val |= cull->h_even << ISPCCDC_CULLING_CULHEVN_SHIFT;
	culling_val |= cull->h_odd << ISPCCDC_CULLING_CULHODD_SHIFT;

	isp_reg_writel(isp, culling_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_CULLING);
}

/*
 * ispccdc_enable_lpf - Enable Low-Pass Filter (LPF).
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LPF, 1 Enables LPF
 */
static void ispccdc_enable_lpf(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE,
		       ~ISPCCDC_SYN_MODE_LPF,
		       enable ? ISPCCDC_SYN_MODE_LPF : 0);
}

/*
 * ispccdc_config_alaw - Configure the input width for A-law compression.
 * @ccdc: Pointer to ISP CCDC device.
 * @ipwidth: Input width for A-law
 */
static void ispccdc_config_alaw(struct isp_ccdc_device *ccdc,
				enum alaw_ipwidth ipwidth)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, ipwidth << ISPCCDC_ALAW_GWDI_SHIFT,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW);
}

/*
 * ispccdc_enable_alaw - Enable A-law compression.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 - Disables A-law, 1 - Enables A-law
 */
static void ispccdc_enable_alaw(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW,
		       ~ISPCCDC_ALAW_CCDTBL,
		       enable ? ISPCCDC_ALAW_CCDTBL : 0);
}

/*
 * ispccdc_config_imgattr - Configure sensor image specific attributes.
 * @ccdc: Pointer to ISP CCDC device.
 * @colptn: Color pattern of the sensor.
 */
static void ispccdc_config_imgattr(struct isp_ccdc_device *ccdc, u32 colptn)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, colptn, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_COLPTN);
}

/*
 * ispccdc_config - Set CCDC configuration from userspace
 * @ccdc: Pointer to ISP CCDC device.
 * @userspace_add: Structure containing CCDC configuration sent from userspace.
 *
 * Returns 0 if successful, -EINVAL if the pointer to the configuration
 * structure is null, or the copy_from_user function fails to copy user space
 * memory to kernel space memory.
 */
static int ispccdc_config(struct isp_ccdc_device *ccdc,
			  struct ispccdc_update_config *ccdc_struct)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct ispccdc_bclamp bclamp_t;
	struct ispccdc_blcomp blcomp_t;
	struct ispccdc_culling cull_t;
	unsigned long flags;
	int ret = 0;

	if (ccdc_struct == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ccdc->lock, flags);
	ccdc->shadow_update = 1;
	spin_unlock_irqrestore(&ccdc->lock, flags);

	if (ISP_ABS_CCDC_ALAW & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_ALAW & ccdc_struct->update)
			ispccdc_config_alaw(ccdc, ccdc_struct->alawip);
		ispccdc_enable_alaw(ccdc, 1);
	} else if (ISP_ABS_CCDC_ALAW & ccdc_struct->update)
		ispccdc_enable_alaw(ccdc, 0);

	if (ISP_ABS_CCDC_LPF & ccdc_struct->flag)
		ispccdc_enable_lpf(ccdc, 1);
	else
		ispccdc_enable_lpf(ccdc, 0);

	if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) {
			if (copy_from_user(&bclamp_t, ccdc_struct->bclamp,
					   sizeof(struct ispccdc_bclamp))) {
				ret = -EFAULT;
				goto out;
			}

			ispccdc_enable_black_clamp(ccdc, 1);
			ispccdc_config_black_clamp(ccdc, &bclamp_t);
		} else
			ispccdc_enable_black_clamp(ccdc, 1);
	} else {
		if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) {
			if (copy_from_user(&bclamp_t, ccdc_struct->bclamp,
					   sizeof(struct ispccdc_bclamp))) {
				ret = -EFAULT;
				goto out;
			}

			ispccdc_enable_black_clamp(ccdc, 0);
			ispccdc_config_black_clamp(ccdc, &bclamp_t);
		}
	}

	if (ISP_ABS_CCDC_BCOMP & ccdc_struct->update) {
		if (copy_from_user(&blcomp_t, ccdc_struct->blcomp,
				   sizeof(blcomp_t))) {
				ret = -EFAULT;
				goto out;
			}

		ispccdc_config_black_comp(ccdc, &blcomp_t);
	}

	if (ISP_ABS_CCDC_FPC & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_FPC & ccdc_struct->update) {
			struct ispccdc_fpc fpc_t;
			u32 fpc_table_m;
			void *fpc_table;
			u32 fpc_table_old;
			u32 fpc_table_size;

			if (ccdc->state != ISP_PIPELINE_STREAM_STOPPED)
				return -EBUSY;

			if (copy_from_user(&fpc_t, ccdc_struct->fpc,
					   sizeof(fpc_t))) {
				ret = -EFAULT;
				goto out;
			}

			/*
			 * fpc_table_m must be 64-bytes aligned, but it's
			 * already done by iommu_vmalloc().
			 */
			fpc_table_size = fpc_t.fpnum * 4;
			fpc_table_m = iommu_vmalloc(isp->iommu, 0,
						    fpc_table_size, IOMMU_FLAG);
			if (IS_ERR_VALUE(fpc_table_m)) {
				ret = -ENOMEM;
				goto out;
			}
			fpc_table = da_to_va(isp->iommu, fpc_table_m);
			if (copy_from_user(fpc_table,
					   (__force void __user *)
					   fpc_t.fpcaddr,
					   fpc_table_size)) {
				iommu_vfree(isp->iommu, fpc_table_m);
				ret = -EFAULT;
				goto out;
			}
			fpc_t.fpcaddr = fpc_table_m;

			spin_lock_irqsave(&ccdc->lock, flags);
			fpc_table_old = ccdc->fpc_table_add_m;
			ccdc->fpc_table_add = fpc_table;
			ccdc->fpc_table_add_m = fpc_table_m;
			ispccdc_config_fpc(ccdc, &fpc_t);
			spin_unlock_irqrestore(&ccdc->lock, flags);

			if (fpc_table_old != 0)
				iommu_vfree(isp->iommu, fpc_table_old);
		}
		ispccdc_enable_fpc(ccdc, 1);
	} else if (ISP_ABS_CCDC_FPC & ccdc_struct->update)
		ispccdc_enable_fpc(ccdc, 0);

	if (ISP_ABS_CCDC_CULL & ccdc_struct->update) {
		if (copy_from_user(&cull_t, ccdc_struct->cull,
				   sizeof(cull_t))) {
			ret = -EFAULT;
			goto out;
		}
		ispccdc_config_culling(ccdc, &cull_t);
	}

	ret = ispccdc_lsc_config(ccdc, ccdc_struct);
	if (ret)
		goto out;

	if (ISP_ABS_CCDC_COLPTN & ccdc_struct->update)
		ispccdc_config_imgattr(ccdc, ccdc_struct->colptn);

out:
	if (ret == -EFAULT)
		dev_err(to_device(ccdc),
			"ccdc: user provided bad configuration data address");

	if (ret == -ENOMEM)
		dev_err(to_device(ccdc),
			"ccdc: can not allocate memory");

	ccdc->shadow_update = 0;
	return ret;
}

/* -----------------------------------------------------------------------------
 * Format- and pipeline-related configuration helpers
 */

/*
 * ispccdc_config_vp - Configure the Video Port.
 * @ccdc: Pointer to ISP CCDC device.
 * @vpcfg: Structure containing the Video Port input frequency, and the 10 bit
 *         format.
 */
static void ispccdc_config_vp(struct isp_ccdc_device *ccdc,
			      struct ispccdc_vp *vpcfg)
{
	struct isp_pipeline *pipe = to_isp_pipeline(&ccdc->subdev.entity);
	struct isp_device *isp = to_isp_device(ccdc);
	unsigned long l3_ick = pipe->l3_ick;
	unsigned int max_div = isp->revision == ISP_REVISION_15_0 ? 64 : 8;
	unsigned int div = 0;
	u32 fmtcfg_vp = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_FMTCFG);

	fmtcfg_vp &= ISPCCDC_FMTCFG_VPIN_MASK & ISPCCDC_FMTCFG_VPIF_FRQ_MASK;

	switch (vpcfg->bitshift_sel) {
	case BIT9_0:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_9_0;
		break;
	case BIT10_1:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_10_1;
		break;
	case BIT11_2:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_11_2;
		break;
	case BIT12_3:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_12_3;
		break;
	};

	if (pipe->input)
		div = DIV_ROUND_UP(l3_ick, pipe->max_rate);
	else if (vpcfg->pixelclk)
		div = l3_ick / vpcfg->pixelclk;

	div = clamp(div, 2U, max_div);
	fmtcfg_vp |= (div - 2) << ISPCCDC_FMTCFG_VPIF_FRQ_SHIFT;

	isp_reg_writel(isp, fmtcfg_vp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG);
}

/*
 * ispccdc_enable_vp - Enable Video Port.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables VP, 1 Enables VP
 *
 * This is needed for outputting image to Preview, H3A and HIST ISP submodules.
 */
static void ispccdc_enable_vp(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG,
		       ~ISPCCDC_FMTCFG_VPEN,
		       enable ? ISPCCDC_FMTCFG_VPEN : 0);
}

/*
 * ispccdc_config_outlineoffset - Configure memory saving output line offset
 * @ccdc: Pointer to ISP CCDC device.
 * @offset: Address offset to start a new line. Must be twice the
 *          Output width and aligned on 32 byte boundary
 * @oddeven: Specifies the odd/even line pattern to be chosen to store the
 *           output.
 * @numlines: Set the value 0-3 for +1-4lines, 4-7 for -1-4lines.
 *
 * - Configures the output line offset when stored in memory
 * - Sets the odd/even line pattern to store the output
 *    (EVENEVEN (1), ODDEVEN (2), EVENODD (3), ODDODD (4))
 * - Configures the number of even and odd line fields in case of rearranging
 * the lines.
 *
 * Returns 0 if successful, or -EINVAL if the offset is not in 32 byte
 * boundary.
 */
static int ispccdc_config_outlineoffset(struct isp_ccdc_device *ccdc,
					u32 offset, u8 oddeven, u8 numlines)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, offset & 0xffff,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF);

	isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
		    ~ISPCCDC_SDOFST_FINV);

	isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
		    ~ISPCCDC_SDOFST_FOFST_4L);

	switch (oddeven) {
	case EVENEVEN:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST0_SHIFT);
		break;
	case ODDEVEN:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST1_SHIFT);
		break;
	case EVENODD:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST2_SHIFT);
		break;
	case ODDODD:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST3_SHIFT);
		break;
	default:
		break;
	}
	return 0;
}

/*
 * ispccdc_set_outaddr - Set memory address to save output image
 * @ccdc: Pointer to ISP CCDC device.
 * @addr: ISP MMU Mapped 32-bit memory address aligned on 32 byte boundary.
 *
 * Sets the memory address where the output will be saved.
 */
static void ispccdc_set_outaddr(struct isp_ccdc_device *ccdc, u32 addr)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, addr, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR);
}

/*
 * ispccdc_config_sync_if - Set CCDC sync interface params between sensor and CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 * @syncif: Structure containing the sync parameters like field state, CCDC in
 *          master/slave mode, raw/yuv data, polarity of data, field, hs, vs
 *          signals.
 */
static void ispccdc_config_sync_if(struct isp_ccdc_device *ccdc,
				   struct ispccdc_syncif *syncif)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 syn_mode = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
				     ISPCCDC_SYN_MODE);

	syn_mode |= ISPCCDC_SYN_MODE_VDHDEN;

	if (syncif->fldstat)
		syn_mode |= ISPCCDC_SYN_MODE_FLDSTAT;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDSTAT;

	syn_mode &= ISPCCDC_SYN_MODE_DATSIZ_MASK;
	switch (syncif->datsz) {
	case 8:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_8;
		break;
	case 10:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_10;
		break;
	case 11:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_11;
		break;
	case 12:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_12;
		break;
	};

	if (syncif->fldmode)
		syn_mode |= ISPCCDC_SYN_MODE_FLDMODE;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDMODE;

	if (syncif->datapol)
		syn_mode |= ISPCCDC_SYN_MODE_DATAPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_DATAPOL;

	if (syncif->fldpol)
		syn_mode |= ISPCCDC_SYN_MODE_FLDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDPOL;

	if (syncif->hdpol)
		syn_mode |= ISPCCDC_SYN_MODE_HDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_HDPOL;

	if (syncif->vdpol)
		syn_mode |= ISPCCDC_SYN_MODE_VDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_VDPOL;

	if (syncif->ccdc_mastermode) {
		syn_mode |= ISPCCDC_SYN_MODE_FLDOUT | ISPCCDC_SYN_MODE_VDHDOUT;
		isp_reg_writel(isp,
			       syncif->hs_width << ISPCCDC_HD_VD_WID_HDW_SHIFT
			     | syncif->vs_width << ISPCCDC_HD_VD_WID_VDW_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_HD_VD_WID);

		isp_reg_writel(isp,
			       syncif->ppln << ISPCCDC_PIX_LINES_PPLN_SHIFT
			     | syncif->hlprf << ISPCCDC_PIX_LINES_HLPRF_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_PIX_LINES);
	} else
		syn_mode &= ~(ISPCCDC_SYN_MODE_FLDOUT |
			      ISPCCDC_SYN_MODE_VDHDOUT);

	isp_reg_writel(isp, syn_mode, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE);

	if (!syncif->bt_r656_en)
		isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC,
			    ISPCCDC_REC656IF, ~ISPCCDC_REC656IF_R656ON);
}

/* CCDC formats descriptions */
static const u32 ccdc_sgrbg_pattern =
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC0_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP0PLC1_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC2_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP0PLC3_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP1PLC0_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP1PLC1_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP1PLC2_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP1PLC3_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC0_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP2PLC1_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC2_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP2PLC3_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP3PLC0_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP3PLC1_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP3PLC2_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

static const u32 ccdc_srggb_pattern =
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP0PLC0_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC1_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP0PLC2_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC3_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP1PLC0_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP1PLC1_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP1PLC2_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP1PLC3_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP2PLC0_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC1_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP2PLC2_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC3_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP3PLC0_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP3PLC1_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP3PLC2_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

static const u32 ccdc_sbggr_pattern =
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP0PLC0_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP0PLC1_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP0PLC2_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP0PLC3_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP1PLC0_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP1PLC1_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP1PLC2_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP1PLC3_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP2PLC0_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP2PLC1_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP2PLC2_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP2PLC3_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP3PLC0_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP3PLC1_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP3PLC2_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

static const u32 ccdc_sgbrg_pattern =
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP0PLC0_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP0PLC1_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP0PLC2_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP0PLC3_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP1PLC0_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP1PLC1_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP1PLC2_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP1PLC3_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP2PLC0_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP2PLC1_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP2PLC2_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP2PLC3_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP3PLC0_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP3PLC1_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP3PLC2_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

static void ccdc_configure(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct isp_parallel_platform_data *pdata = NULL;
	struct media_entity_pad *pad;
	struct v4l2_subdev *sensor;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_pix_format pix;
	unsigned long flags;
	u32 syn_mode;
	u32 ccdc_pattern;

	if (ccdc->input == CCDC_INPUT_PARALLEL) {
		pad = media_entity_remote_pad(&ccdc->pads[CCDC_PAD_SINK]);
		sensor = media_entity_to_v4l2_subdev(pad->entity);
		pdata = &((struct isp_v4l2_subdevs_group *)sensor->host_priv)
			->bus.parallel;
	}

	isp_configure_bridge(isp, ccdc->input, pdata);

	syn_mode = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE);

	/* Use the raw, unprocessed data when writing to memory. The H3A and
	 * histogram modules are still fed with lens shading corrected data.
	 */
	syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;

	if (ccdc->output & CCDC_OUTPUT_MEMORY)
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;

	if (ccdc->output & CCDC_OUTPUT_RESIZER)
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;

	isp_reg_writel(isp, syn_mode, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE);

	/* CCDC_PAD_SINK */
	format = &ccdc->formats[CCDC_PAD_SINK];

	/* Mosaic filter */
	switch (format->code) {
	case V4L2_MBUS_FMT_SRGGB10_1X10:
		ccdc_pattern = ccdc_srggb_pattern;
		break;
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		ccdc_pattern = ccdc_sbggr_pattern;
		break;
	case V4L2_MBUS_FMT_SGBRG10_1X10:
		ccdc_pattern = ccdc_sgbrg_pattern;
		break;
	default:
		/* Use GRBG */
		ccdc_pattern = ccdc_sgrbg_pattern;
		break;
	}
	ispccdc_config_imgattr(ccdc, ccdc_pattern);

	/* Generate VD0 on the last line of the image and VD1 on the
	 * 2/3 height line.
	 */
	isp_reg_writel(isp, ((format->height - 2) << ISPCCDC_VDINT_0_SHIFT) |
		       ((format->height * 2 / 3) << ISPCCDC_VDINT_1_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VDINT);

	/* CCDC_PAD_SOURCE_OF */
	format = &ccdc->formats[CCDC_PAD_SOURCE_OF];

	isp_reg_writel(isp, (0 << ISPCCDC_HORZ_INFO_SPH_SHIFT) |
		       ((format->width - 1) << ISPCCDC_HORZ_INFO_NPH_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HORZ_INFO);
	isp_reg_writel(isp, 0 << ISPCCDC_VERT_START_SLV0_SHIFT,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_START);
	isp_reg_writel(isp, (format->height - 1)
			<< ISPCCDC_VERT_LINES_NLV_SHIFT,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_LINES);

	isp_video_mbus_to_pix(&ccdc->video_out, format, &pix);
	ispccdc_config_outlineoffset(ccdc, pix.bytesperline, 0, 0);

	/* CCDC_PAD_SOURCE_VP */
	format = &ccdc->formats[CCDC_PAD_SOURCE_VP];

	isp_reg_writel(isp, (0 << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT) |
		       (format->width << ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_HORZ);
	isp_reg_writel(isp, (0 << ISPCCDC_FMT_VERT_FMTSLV_SHIFT) |
		       ((format->height + 1) << ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_VERT);

	isp_reg_writel(isp, (format->width << ISPCCDC_VP_OUT_HORZ_NUM_SHIFT) |
		       (format->height << ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VP_OUT);

	/* Setup LSC. Disable it if not supported for the selected
	 * resolution.
	 */
	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	if (!list_empty(&ccdc->lsc.req_queue)) {
		struct ispccdc_lsc_config_req *req;

		req = ispccdc_lsc_dequeue_req(ccdc, &ccdc->lsc.req_queue);
		list_del(&req->list);
		if (ispccdc_lsc_validate_config(ccdc, &req->config) < 0) {
			list_add_tail(&req->list, &ccdc->lsc.free_queue);
			schedule_work(&ccdc->lsc.table_work);
		} else {
			WARN_ON(ccdc->lsc.active);
			ccdc->lsc.active = req;
			__ispccdc_lsc_configure(ccdc, req);
		}
	}
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);

	ispccdc_print_status(ccdc);
}

static void __ispccdc_enable(struct isp_ccdc_device *ccdc, int enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR,
		       ~ISPCCDC_PCR_EN, enable ? ISPCCDC_PCR_EN : 0);
}

static int ispccdc_disable(struct isp_ccdc_device *ccdc)
{
	unsigned long flags;
	int ret = 0;

	if (ccdc->state == ISP_PIPELINE_STREAM_SINGLESHOT)
		goto skip_wait;

	spin_lock_irqsave(&ccdc->lock, flags);
	ccdc->stopping = 1;
	spin_unlock_irqrestore(&ccdc->lock, flags);

	ret = wait_event_timeout(ccdc->wait, ccdc->stopping == 0,
				 msecs_to_jiffies(2000));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		ccdc->stopping = 0;
		dev_warn(to_device(ccdc), "CCDC stop timeout!\n");
	}

skip_wait:
	isp_sbl_disable(to_isp_device(ccdc), OMAP3_ISP_SBL_CCDC_LSC_READ);

	ispccdc_lsc_free_queue(ccdc, &ccdc->lsc.req_queue);
	ispccdc_lsc_free_queue(ccdc, &ccdc->lsc.free_queue);
	ispccdc_lsc_init_queue(ccdc);
	if (ccdc->lsc.active) {
		struct isp_device *isp = to_isp_device(ccdc);

		if (ccdc->lsc.active->table)
			iommu_vfree(isp->iommu, ccdc->lsc.active->table);
		kfree(ccdc->lsc.active);
		ccdc->lsc.active = NULL;
	}

	return ret > 0 ? 0 : ret;
}

static void ispccdc_enable(struct isp_ccdc_device *ccdc)
{
	if (ispccdc_lsc_is_configured(ccdc))
		__ispccdc_lsc_enable(ccdc, 1);
	__ispccdc_enable(ccdc, 1);
}

/* -----------------------------------------------------------------------------
 * Interrupt handling
 */

/*
 * ispccdc_sbl_busy - Poll idle state of CCDC and related SBL memory write bits
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Returns zero if the CCDC is idle and the image has been written to
 * memory, too.
 */
static int ispccdc_sbl_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return ispccdc_busy(ccdc)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_0) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_1) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_2) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_3) &
		   ISPSBL_CCDC_WR_0_DATA_READY);
}

/*
 * ispccdc_sbl_wait_idle - Wait until the CCDC and related SBL are idle
 * @ccdc: Pointer to ISP CCDC device.
 * @max_wait: Max retry count in us for wait for idle/busy transition.
 */
static int ispccdc_sbl_wait_idle(struct isp_ccdc_device *ccdc,
				 unsigned int max_wait)
{
	unsigned int wait = 0;

	if (max_wait == 0)
		max_wait = 10000; /* 10 ms */

	for (wait = 0; wait <= max_wait; wait++) {
		if (!ispccdc_sbl_busy(ccdc))
			return 0;

		rmb();
		udelay(1);
	}

	return -EBUSY;
}

static void ispccdc_hs_vs_isr(struct isp_ccdc_device *ccdc)
{
	struct video_device *vdev = &ccdc->subdev.devnode;
	struct v4l2_event event;

	memset(&event, 0, sizeof(event));
	event.type = V4L2_EVENT_OMAP3ISP_HS_VS;

	v4l2_event_queue(vdev, &event);
}

/*
 * ispccdc_lsc_isr - Handle LSC events
 * @ccdc: Pointer to ISP CCDC device.
 * @events: LSC events
 */
static void ispccdc_lsc_isr(struct isp_ccdc_device *ccdc, u32 events)
{
	struct ispccdc_lsc_config_req *req;
	unsigned long flags;

	if (events & IRQ0STATUS_CCDC_LSC_PREF_ERR_IRQ) {
		ispccdc_lsc_error_handler(ccdc);
		ccdc->error = 1;
		dev_dbg(to_device(ccdc), "lsc prefetch error\n");
	}

	if (!(events & IRQ0STATUS_CCDC_LSC_DONE_IRQ))
		return;

	/* This is an exception. Start of frame and LSC_DONE interrupt
	 * have been received on the same time. Skip this event and wait
	 * for better times.
	 */
	if ((events & IRQ0STATUS_HS_VS_IRQ) &&
	    (events & IRQ0STATUS_CCDC_LSC_DONE_IRQ)) {
		return;
	}

	/* LSC_DONE interrupt occur, there are two cases
	 * 1. stopping for reconfiguration
	 * 2. stopping because of STREAM OFF command
	 */
	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);

	if (ccdc->lsc.state == LSC_STATE_STOPPING) {
		ccdc->lsc.state = LSC_STATE_STOPPED;
		goto done;
	}

	if (ccdc->lsc.state != LSC_STATE_RECONFIG)
		goto done;

	/* LSC is in STOPPING state, change to the new state */
	ccdc->lsc.state = LSC_STATE_STOPPED;

	/* The LSC engine is stopped at this point. Get first entry from
	 * request queue without deleting it
	 */
	req = ispccdc_lsc_dequeue_req(ccdc, &ccdc->lsc.req_queue);
	if (req == NULL)
		goto done;

	/* We should have an old active configuration it's time to free it */
	if (ccdc->lsc.active)
		list_add_tail(&ccdc->lsc.active->list, &ccdc->lsc.free_queue);

	/* Modify active pointer to the current configuration entry and
	 * delete new request from request queue
	 */
	list_del(&req->list);
	ccdc->lsc.active = req;

	/* The user wants to disable LSC, nothing to do */
	if (!req->enable) {
		isp_sbl_disable(to_isp_device(ccdc),
				OMAP3_ISP_SBL_CCDC_LSC_READ);
		goto done;
	}

	if (__ispccdc_lsc_configure(ccdc, req) == 0)
		__ispccdc_lsc_enable(ccdc, 1);

	schedule_work(&ccdc->lsc.table_work);
done:
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
}

static int ispccdc_isr_buffer(struct isp_ccdc_device *ccdc)
{
	struct isp_pipeline *pipe = to_isp_pipeline(&ccdc->subdev.entity);
	struct isp_device *isp = to_isp_device(ccdc);
	struct isp_buffer *buffer;
	int restart = 0;

	/* The CCDC generates VD0 interrupts even when disabled (the datasheet
	 * doesn't explicitly state if that's supposed to happen or not, so it
	 * can be considered as a hardware bug or as a feature, but we have to
	 * deal with it anyway). Disabling the CCDC when no buffer is available
	 * would thus not be enough, we need to handle the situation explicitly.
	 */
	if (list_empty(&ccdc->video_out.dmaqueue))
		goto done;

	/* We're in continuous mode, and memory writes were disabled due to a
	 * buffer underrun. Reenable them now that we have a buffer. The buffer
	 * address has been set in ccdc_video_queue.
	 */
	if (ccdc->state == ISP_PIPELINE_STREAM_CONTINUOUS && ccdc->underrun) {
		restart = 1;
		ccdc->underrun = 0;
		goto done;
	}

	if (ispccdc_sbl_wait_idle(ccdc, 1000)) {
		dev_info(isp->dev, "CCDC won't become idle!\n");
		goto done;
	}

	buffer = isp_video_buffer_next(&ccdc->video_out, ccdc->error);
	if (buffer != NULL) {
		ispccdc_set_outaddr(ccdc, buffer->isp_addr);
		restart = 1;
	}

	pipe->state |= ISP_PIPELINE_IDLE_OUTPUT;

	if (ccdc->state == ISP_PIPELINE_STREAM_SINGLESHOT &&
	    isp_pipeline_ready(pipe))
		isp_pipeline_set_stream(pipe,
					ISP_PIPELINE_STREAM_SINGLESHOT);

done:
	ccdc->error = 0;
	return restart;
}

/*
 * ispccdc_vd0_isr - Handle VD0 event
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Executes LSC deferred enablement before next frame starts.
 */
static void ispccdc_vd0_isr(struct isp_ccdc_device *ccdc)
{
	unsigned long flags;
	int restart = 0;

	if (ccdc->output & CCDC_OUTPUT_MEMORY)
		restart = ispccdc_isr_buffer(ccdc);

	spin_lock_irqsave(&ccdc->lock, flags);
	if (ccdc->stopping) {
		ccdc->stopping = 0;
		spin_unlock_irqrestore(&ccdc->lock, flags);
		wake_up(&ccdc->wait);
		return;
	}
	spin_unlock_irqrestore(&ccdc->lock, flags);

	if (restart)
		ispccdc_enable(ccdc);
}

/*
 * ispccdc_vd1_isr - Handle VD1 event
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ispccdc_vd1_isr(struct isp_ccdc_device *ccdc)
{
	struct ispccdc_lsc_config_req *req;
	unsigned long flags;

	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);

	/* We are about to stop CCDC and/without LSC */
	if (ccdc->stopping || (ccdc->output & CCDC_OUTPUT_MEMORY) ||
	   (ccdc->state == ISP_PIPELINE_STREAM_SINGLESHOT)) {
		__ispccdc_lsc_enable(ccdc, 0);
		__ispccdc_enable(ccdc, 0);
		goto done;
	}

	/* Get first entry from queue but without deleting it */
	req = ispccdc_lsc_dequeue_req(ccdc, &ccdc->lsc.req_queue);
	if (req == NULL)
		goto done;

	/* Run time LSC update */
	switch (ccdc->lsc.state) {
	case LSC_STATE_RUNNING:
		/* LSC need to be reconfigured. Stop it here and on next
		 * LSC_DONE IRQ do the appropriate changes in registers
		 */
		__ispccdc_lsc_enable(ccdc, 0);
		ccdc->lsc.state = LSC_STATE_RECONFIG;
		break;

	case LSC_STATE_STOPPED:
		if (!req->enable)
			break;
		/* LSC has been in stopped state, enable it now */
		if (__ispccdc_lsc_configure(ccdc, req) == 0) {
			if (ccdc->lsc.active)
				list_add_tail(&ccdc->lsc.active->list,
					      &ccdc->lsc.free_queue);
			ccdc->lsc.active = req;
			list_del(&req->list);
			__ispccdc_lsc_enable(ccdc, 1);
			schedule_work(&ccdc->lsc.table_work);
		}
		break;

	case LSC_STATE_STOPPING:
	case LSC_STATE_RECONFIG:
		/* shouldn't happen */
		break;
	}

done:
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
}

/*
 * ispccdc_isr - Configure CCDC during interframe time.
 * @ccdc: Pointer to ISP CCDC device.
 * @events: CCDC events
 */
int ispccdc_isr(struct isp_ccdc_device *ccdc, u32 events)
{
	if (ccdc->state == ISP_PIPELINE_STREAM_STOPPED)
		return 0;

	if (events & IRQ0STATUS_CCDC_VD1_IRQ)
		ispccdc_vd1_isr(ccdc);

	ispccdc_lsc_isr(ccdc, events);

	if (events & IRQ0STATUS_CCDC_VD0_IRQ)
		ispccdc_vd0_isr(ccdc);

	if (events & IRQ0STATUS_HS_VS_IRQ)
		ispccdc_hs_vs_isr(ccdc);

	return 0;
}

/* -----------------------------------------------------------------------------
 * ISP video operations
 */

static int ccdc_video_queue(struct isp_video *video, struct isp_buffer *buffer)
{
	struct isp_ccdc_device *ccdc = &video->isp->isp_ccdc;

	if (!(ccdc->output & CCDC_OUTPUT_MEMORY))
		return -ENODEV;

	ispccdc_set_outaddr(ccdc, buffer->isp_addr);

	/* We now have a buffer queued on the output, restart the pipeline in
	 * on the next CCDC interrupt if running in continuous mode (or when
	 * starting the stream).
	 */
	ccdc->underrun = 1;

	return 0;
}

static const struct isp_video_operations ccdc_video_ops = {
	.queue = ccdc_video_queue,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

/*
 * ccdc_get_ctrl - V4L2 control get handler
 * @sd: ISP CCDC V4L2 subdevice
 * @ctrl: V4L2 control
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccdc_get_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return -EINVAL;
}

/*
 * ccdc_set_ctrl - V4L2 control set handler
 * @sd: ISP CCDC V4L2 subdevice
 * @ctrl: V4L2 control
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccdc_set_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return -EINVAL;
}

/*
 * ccdc_ioctl - CCDC module private ioctl's
 * @sd: ISP CCDC V4L2 subdevice
 * @cmd: ioctl command
 * @arg: ioctl argument
 *
 * Return 0 on success or a negative error code otherwise.
 */
static long ccdc_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	int ret;

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_CCDC_CFG:
		mutex_lock(&ccdc->ioctl_lock);
		ret = ispccdc_config(ccdc, arg);
		mutex_unlock(&ccdc->ioctl_lock);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

/*
 * ccdc_set_power - Power on/off the CCDC module
 * @sd: ISP CCDC V4L2 subdevice
 * @on: power on/off
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccdc_set_power(struct v4l2_subdev *sd, int on)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);

	if (on) {
		if (!isp_get(isp))
			return -EBUSY;
	} else {
		isp_put(isp);
	}

	return 0;
}

static int ccdc_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_OMAP3ISP_HS_VS)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub);
}

static int ccdc_unsubscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				  struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

/*
 * ccdc_set_stream - Enable/Disable streaming on the CCDC module
 * @sd: ISP CCDC V4L2 subdevice
 * @enable: Enable/disable stream
 *
 * When writing to memory, the CCDC hardware can't be enabled without a memory
 * buffer to write to. As the s_stream operation is called in response to a
 * STREAMON call without any buffer queued yet, just update the enabled field
 * and return immediately. The CCDC will be enabled in ccdc_isr_buffer().
 *
 * When not writing to memory enable the CCDC immediately.
 */
static int ccdc_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);
	int ret = 0;

	if (ccdc->state == ISP_PIPELINE_STREAM_STOPPED) {
		if (enable == ISP_PIPELINE_STREAM_STOPPED)
			return 0;

		isp_reg_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			   ISPCTRL_CCDC_RAM_EN | ISPCTRL_CCDC_CLK_EN);
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
			   ISPCCDC_CFG_VDLC);

		ccdc_configure(ccdc);

		/* TODO: Don't configure the video port if all of its output
		 * links are inactive.
		 */
		ispccdc_config_vp(ccdc, &ccdc->vpcfg);
		ispccdc_enable_vp(ccdc, 1);
	}

	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		if (ccdc->output & CCDC_OUTPUT_MEMORY)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_WRITE);

		if (ccdc->underrun || !(ccdc->output & CCDC_OUTPUT_MEMORY))
			ispccdc_enable(ccdc);

		ccdc->underrun = 0;
		break;

	case ISP_PIPELINE_STREAM_SINGLESHOT:
		if (ccdc->output & CCDC_OUTPUT_MEMORY &&
		    ccdc->state != ISP_PIPELINE_STREAM_SINGLESHOT)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_WRITE);

		ispccdc_enable(ccdc);
		break;

	case ISP_PIPELINE_STREAM_STOPPED:
		ret = ispccdc_disable(ccdc);
		if (ccdc->output & CCDC_OUTPUT_MEMORY)
			isp_sbl_disable(isp, OMAP3_ISP_SBL_CCDC_WRITE);
		isp_reg_and(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			    ~(ISPCTRL_CCDC_CLK_EN | ISPCTRL_CCDC_RAM_EN));
		ccdc->underrun = 0;
		break;
	}

	ccdc->state = enable;
	return ret;
}

static struct v4l2_mbus_framefmt *
__ccdc_get_format(struct isp_ccdc_device *ccdc, struct v4l2_subdev_fh *fh,
		  unsigned int pad, enum v4l2_subdev_format which)
{
	if (which == V4L2_SUBDEV_FORMAT_PROBE)
		return v4l2_subdev_get_probe_format(fh, pad);
	else
		return &ccdc->formats[pad];
}

/*
 * ccdc_try_format - Try video format on a pad
 * @ccdc: ISP CCDC device
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 */
static void
ccdc_try_format(struct isp_ccdc_device *ccdc, struct v4l2_subdev_fh *fh,
		unsigned int pad, struct v4l2_mbus_framefmt *fmt,
		enum v4l2_subdev_format which)
{
	struct v4l2_mbus_framefmt *format;
	unsigned int width = fmt->width;
	unsigned int height = fmt->height;
	unsigned int i;

	switch (pad) {
	case CCDC_PAD_SINK:
		/* TODO: If the CCDC output formatter pad is connected directly
		 * to the resizer, only YUV formats can be used.
		 */
		for (i = 0; i < ARRAY_SIZE(ccdc_fmts); i++) {
			if (fmt->code == ccdc_fmts[i])
				break;
		}

		/* If not found, use SGRBG10 as default */
		if (i >= ARRAY_SIZE(ccdc_fmts))
			fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

		/* Clamp the input size. */
		fmt->width = clamp_t(u32, width, 32, 4096);
		fmt->height = clamp_t(u32, height, 32, 4096);
		break;

	case CCDC_PAD_SOURCE_OF:
		format = __ccdc_get_format(ccdc, fh, CCDC_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		/* The data formatter truncates the number of horizontal output
		 * pixels to a multiple of 16. To avoid clipping data, allow
		 * callers to request an output size bigger than the input size
		 * up to the nearest multiple of 16.
		 */
		fmt->width = clamp_t(u32, width, 32, (fmt->width + 15) & ~15);
		fmt->width &= ~15;
		fmt->height = clamp_t(u32, height, 32, fmt->height);
		break;

	case CCDC_PAD_SOURCE_VP:
		format = __ccdc_get_format(ccdc, fh, CCDC_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		/* The number of lines that can be clocked out from the video
		 * port output must be at least one line less than the number
		 * of input lines.
		 */
		fmt->width = clamp_t(u32, width, 32, fmt->width);
		fmt->height = clamp_t(u32, height, 32, fmt->height - 1);
		break;
	}

	/* Data is written to memory unpacked, each 10-bit pixel is stored on
	 * 2 bytes.
	 */
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
}

/*
 * ccdc_enum_mbus_code - Handle pixel format enumeration
 * @sd     : pointer to v4l2 subdev structure
 * @fh : V4L2 subdev file handle
 * @code   : pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int ccdc_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_fh *fh,
			       struct v4l2_subdev_pad_mbus_code_enum *code)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	switch (code->pad) {
	case CCDC_PAD_SINK:
		if (code->index >= ARRAY_SIZE(ccdc_fmts))
			return -EINVAL;

		code->code = ccdc_fmts[code->index];
		break;

	case CCDC_PAD_SOURCE_OF:
	case CCDC_PAD_SOURCE_VP:
		/* No format conversion inside CCDC */
		if (code->index != 0)
			return -EINVAL;

		format = __ccdc_get_format(ccdc, fh, CCDC_PAD_SINK,
					   V4L2_SUBDEV_FORMAT_PROBE);

		code->code = format->code;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ccdc_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ccdc_try_format(ccdc, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_PROBE);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	ccdc_try_format(ccdc, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_PROBE);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * ccdc_get_format - Retrieve the video format on a pad
 * @sd : ISP CCDC V4L2 subdevice
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int ccdc_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   unsigned int pad, struct v4l2_mbus_framefmt *fmt,
			   enum v4l2_subdev_format which)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __ccdc_get_format(ccdc, fh, pad, which);
	if (format == NULL)
		return -EINVAL;

	memcpy(fmt, format, sizeof(*fmt));
	return 0;
}

/*
 * ccdc_set_format - Set the video format on a pad
 * @sd : ISP CCDC V4L2 subdevice
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int ccdc_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   unsigned int pad, struct v4l2_mbus_framefmt *fmt,
			   enum v4l2_subdev_format which)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __ccdc_get_format(ccdc, fh, pad, which);
	if (format == NULL)
		return -EINVAL;

	ccdc_try_format(ccdc, fh, pad, fmt, which);
	memcpy(format, fmt, sizeof(*format));

	/* Propagate the format from sink to source */
	if (pad == CCDC_PAD_SINK) {
		format = __ccdc_get_format(ccdc, fh, CCDC_PAD_SOURCE_OF, which);
		memcpy(format, fmt, sizeof(*format));
		ccdc_try_format(ccdc, fh, CCDC_PAD_SOURCE_OF, format, which);

		format = __ccdc_get_format(ccdc, fh, CCDC_PAD_SOURCE_VP, which);
		memcpy(format, fmt, sizeof(*format));
		ccdc_try_format(ccdc, fh, CCDC_PAD_SOURCE_VP, format, which);
	}

	return 0;
}

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops ccdc_v4l2_core_ops = {
	.g_ctrl = ccdc_get_ctrl,
	.s_ctrl = ccdc_set_ctrl,
	.ioctl = ccdc_ioctl,
	.s_power = ccdc_set_power,
	.subscribe_event = ccdc_subscribe_event,
	.unsubscribe_event = ccdc_unsubscribe_event,
};

/* V4L2 subdev video operations */
static const struct v4l2_subdev_video_ops ccdc_v4l2_video_ops = {
	.s_stream = ccdc_set_stream,
};

/* V4L2 subdev pad operations */
static const struct v4l2_subdev_pad_ops ccdc_v4l2_pad_ops = {
	.enum_mbus_code = ccdc_enum_mbus_code,
	.enum_frame_size = ccdc_enum_frame_size,
	.get_fmt = ccdc_get_format,
	.set_fmt = ccdc_set_format,
};

/* V4L2 subdev operations */
static const struct v4l2_subdev_ops ccdc_v4l2_ops = {
	.core = &ccdc_v4l2_core_ops,
	.video = &ccdc_v4l2_video_ops,
	.pad = &ccdc_v4l2_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/*
 * ccdc_link_setup - Setup CCDC connections
 * @entity: CCDC media entity
 * @local: Pad at the local end of the link
 * @remote: Pad at the remote end of the link
 * @flags: Link flags
 *
 * return -EINVAL or zero on success
 */
static int ccdc_link_setup(struct media_entity *entity,
			   const struct media_entity_pad *local,
			   const struct media_entity_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);

	switch (local->index | (remote->entity->type << 16)) {
	case CCDC_PAD_SINK | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* Read from the sensor (parallel interface), CCP2, CSI2a or
		 * CSI2c.
		 */
		if (!(flags & MEDIA_LINK_FLAG_ACTIVE)) {
			ccdc->input = CCDC_INPUT_NONE;
			break;
		}

		if (ccdc->input != CCDC_INPUT_NONE)
			return -EBUSY;

		if (remote->entity == &isp->isp_ccp2.subdev.entity)
			ccdc->input = CCDC_INPUT_CCP2B;
		else if (remote->entity == &isp->isp_csi2a.subdev.entity)
			ccdc->input = CCDC_INPUT_CSI2A;
		else if (remote->entity == &isp->isp_csi2c.subdev.entity)
			ccdc->input = CCDC_INPUT_CSI2C;
		else
			ccdc->input = CCDC_INPUT_PARALLEL;

		break;

	case CCDC_PAD_SOURCE_VP | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* Write to preview engine, histogram and H3A. When none of
		 * those links are active, the video port can be disabled.
		 */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			ccdc->output |= CCDC_OUTPUT_PREVIEW;
		else
			ccdc->output &= ~CCDC_OUTPUT_PREVIEW;
		break;

	case CCDC_PAD_SOURCE_OF | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* Write to memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			ccdc->output |= CCDC_OUTPUT_MEMORY;
		else
			ccdc->output &= ~CCDC_OUTPUT_MEMORY;
		break;

	case CCDC_PAD_SOURCE_OF | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* Write to resizer */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			ccdc->output |= CCDC_OUTPUT_RESIZER;
		else
			ccdc->output &= ~CCDC_OUTPUT_RESIZER;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* media operations */
static const struct media_entity_operations ccdc_media_ops = {
	.link_setup = ccdc_link_setup,
	.set_power = v4l2_subdev_set_power,
};

/*
 * isp_ccdc_init_entities - Initialize V4L2 subdev and media entity
 * @ccdc: ISP CCDC module
 *
 * Return 0 on success and a negative error code on failure.
 */
static int isp_ccdc_init_entities(struct isp_ccdc_device *ccdc)
{
	struct v4l2_subdev *sd = &ccdc->subdev;
	struct media_entity_pad *pads = ccdc->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	ccdc->input = CCDC_INPUT_NONE;

	v4l2_subdev_init(sd, &ccdc_v4l2_ops);
	strlcpy(sd->name, "OMAP3 ISP CCDC", sizeof(sd->name));
	sd->grp_id = 1 << 16;	/* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, ccdc);
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->nevents = OMAP3ISP_CCDC_NEVENTS;

	pads[CCDC_PAD_SINK].type = MEDIA_PAD_TYPE_INPUT;
	pads[CCDC_PAD_SOURCE_VP].type = MEDIA_PAD_TYPE_OUTPUT;
	pads[CCDC_PAD_SOURCE_OF].type = MEDIA_PAD_TYPE_OUTPUT;

	me->ops = &ccdc_media_ops;
	ret = media_entity_init(me, CCDC_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	ccdc->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ccdc->video_out.ops = &ccdc_video_ops;
	ccdc->video_out.isp = to_isp_device(ccdc);
	ccdc->video_out.capture_mem = PAGE_ALIGN(4096 * 4096) * 3;
	ccdc->video_out.alignment = 32;

	ret = isp_video_init(&ccdc->video_out, "CCDC");
	if (ret < 0)
		return ret;

	/* Connect the CCDC subdev to the video node. */
	ret = media_entity_create_link(&ccdc->subdev.entity, CCDC_PAD_SOURCE_OF,
			&ccdc->video_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void isp_ccdc_unregister_entities(struct isp_ccdc_device *ccdc)
{
	media_entity_cleanup(&ccdc->subdev.entity);

	v4l2_device_unregister_subdev(&ccdc->subdev);
	isp_video_unregister(&ccdc->video_out);
}

int isp_ccdc_register_entities(struct isp_ccdc_device *ccdc,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video node. */
	ret = v4l2_device_register_subdev(vdev, &ccdc->subdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&ccdc->video_out, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	isp_ccdc_unregister_entities(ccdc);
	return ret;
}

/* -----------------------------------------------------------------------------
 * ISP CCDC initialisation and cleanup
 */

/*
 * isp_ccdc_init - CCDC module initialization.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * TODO: Get the initialisation values from platform data.
 *
 * Return 0 on success or a negative error code otherwise.
 */
int isp_ccdc_init(struct isp_device *isp)
{
	struct isp_ccdc_device *ccdc = &isp->isp_ccdc;

	spin_lock_init(&ccdc->lock);
	init_waitqueue_head(&ccdc->wait);
	mutex_init(&ccdc->ioctl_lock);

	ccdc->shadow_update = 0;

	INIT_WORK(&ccdc->lsc.table_work, ispccdc_lsc_free_table_work);
	ccdc->lsc.state = LSC_STATE_STOPPED;
	ispccdc_lsc_init_queue(ccdc);
	spin_lock_init(&ccdc->lsc.req_lock);

	ccdc->fpc_table_add_m = 0;

	ccdc->syncif.ccdc_mastermode = 0;
	ccdc->syncif.datapol = 0;
	ccdc->syncif.datsz = 10;
	ccdc->syncif.fldmode = 0;
	ccdc->syncif.fldout = 0;
	ccdc->syncif.fldpol = 0;
	ccdc->syncif.fldstat = 0;
	ccdc->syncif.hdpol = 0;
	ccdc->syncif.vdpol = 0;

	ccdc->blkcfg.oblen = 0;
	ccdc->blkcfg.dcsubval = 64;

	ccdc->vpcfg.bitshift_sel = BIT9_0;
	ccdc->vpcfg.pixelclk = 0;

	ispccdc_config_sync_if(ccdc, &ccdc->syncif);
	ispccdc_config_black_clamp(ccdc, &ccdc->blkcfg);

	return isp_ccdc_init_entities(ccdc);
}

/*
 * isp_ccdc_cleanup - CCDC module cleanup.
 * @dev: Device pointer specific to the OMAP3 ISP.
 */
void isp_ccdc_cleanup(struct isp_device *isp)
{
	struct isp_ccdc_device *ccdc = &isp->isp_ccdc;

	/* Free lsc old table */
	flush_work(&ccdc->lsc.table_work);

	if (ccdc->fpc_table_add_m != 0)
		iommu_vfree(isp->iommu, ccdc->fpc_table_add_m);
}
