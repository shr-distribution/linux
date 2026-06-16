// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Author: Bjorn Andersson <bjorn.andersson@sonymobile.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/mfd/qcom_rpm.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/clk.h>

#include <dt-bindings/mfd/qcom-rpm.h>

struct qcom_rpm_resource {
	unsigned target_id;
	unsigned status_id;
	unsigned select_id;
	unsigned size;
};

struct qcom_rpm_data {
	u32 version;
	const struct qcom_rpm_resource *resource_table;
	unsigned int n_resources;
	unsigned int req_ctx_off;
	unsigned int req_sel_off;
	unsigned int ack_ctx_off;
	unsigned int ack_sel_off;
	unsigned int req_sel_size;
	unsigned int ack_sel_size;
};

struct qcom_rpm {
	struct device *dev;
	struct regmap *ipc_regmap;
	unsigned ipc_offset;
	unsigned ipc_bit;
	struct clk *ramclk;

	struct completion ack;
	struct mutex lock;

	void __iomem *status_regs;
	void __iomem *ctrl_regs;
	void __iomem *req_regs;

	u32 ack_status;

	/*
	 * Set by qcom_rpm_suspend_late(), cleared by qcom_rpm_resume_early().
	 * While true, qcom_rpm_write() returns -EAGAIN immediately instead of
	 * grabbing rpm->lock and blocking 5*HZ on rpm->ack -- after this point
	 * the ack IRQ will be masked at the GIC by suspend_device_irqs() (or
	 * is masked by the time the matching genpd power_off cascade reaches
	 * us at .suspend_noirq), so the completion would never fire and the
	 * call would time out. Worse, because the lock is held across the
	 * whole wait, every subsequent caller would serialise behind it
	 * (22 RPM consumers x 5.04s = ~110s of resume-window cascade on
	 * MSM8x60). The guard turns that into an instant -EAGAIN return;
	 * callers can either retry later or treat their operation as a best-
	 * effort no-op for this PM cycle.
	 *
	 * Arming this at .suspend_late (not .suspend_noirq) matters: noirq
	 * callbacks run AFTER suspend_device_irqs() has masked the IRQ, AND
	 * the PM core walks noirq callbacks in reverse probe order, so the
	 * MMCC genpd footswitch power_off cascade (qcom_rpm's child consumers)
	 * fires its own qcom_rpm_write attempts BEFORE qcom_rpm's own
	 * .suspend_noirq would set the flag. By the suspend_late phase we
	 * are guaranteed to run before both suspend_device_irqs() and the
	 * genpd cascade.
	 */
	bool suspended;

	const struct qcom_rpm_data *data;
};

#define RPM_STATUS_REG(rpm, i)	((rpm)->status_regs + (i) * 4)
#define RPM_CTRL_REG(rpm, i)	((rpm)->ctrl_regs + (i) * 4)
#define RPM_REQ_REG(rpm, i)	((rpm)->req_regs + (i) * 4)

#define RPM_REQUEST_TIMEOUT	(5 * HZ)

#define RPM_MAX_SEL_SIZE	7

#define RPM_NOTIFICATION	BIT(30)
#define RPM_REJECTED		BIT(31)

/*
 * Bounds for qcom_rpm_write_sync() status polling.
 *
 * RPM_SYNC_TIMEOUT_US is the upper bound on how long we will wait
 * for the per-resource status register to reflect a vote we just
 * queued.  100 ms covers the worst observed apply latency on
 * MSM8x60-class hardware (fabric clock ramps) with a generous
 * safety margin, and is small enough that a stuck RPM does not
 * brick boot.
 *
 * RPM_SYNC_POLL_US is the inter-poll interval; 10 µs is short
 * enough that a typical 200-500 µs apply latency only costs a few
 * dozen reads and long enough that we are not pounding the MMIO
 * bus.
 */
#define RPM_SYNC_TIMEOUT_US	100000
#define RPM_SYNC_POLL_US	10

static const struct qcom_rpm_resource apq8064_rpm_resource_table[] = {
	[QCOM_RPM_CXO_CLK] =			{ 25, 9, 5, 1 },
	[QCOM_RPM_PXO_CLK] =			{ 26, 10, 6, 1 },
	[QCOM_RPM_APPS_FABRIC_CLK] =		{ 27, 11, 8, 1 },
	[QCOM_RPM_SYS_FABRIC_CLK] =		{ 28, 12, 9, 1 },
	[QCOM_RPM_MM_FABRIC_CLK] =		{ 29, 13, 10, 1 },
	[QCOM_RPM_DAYTONA_FABRIC_CLK] =		{ 30, 14, 11, 1 },
	[QCOM_RPM_SFPB_CLK] =			{ 31, 15, 12, 1 },
	[QCOM_RPM_CFPB_CLK] =			{ 32, 16, 13, 1 },
	[QCOM_RPM_MMFPB_CLK] =			{ 33, 17, 14, 1 },
	[QCOM_RPM_EBI1_CLK] =			{ 34, 18, 16, 1 },
	[QCOM_RPM_APPS_FABRIC_HALT] =		{ 35, 19, 18, 1 },
	[QCOM_RPM_APPS_FABRIC_MODE] =		{ 37, 20, 19, 1 },
	[QCOM_RPM_APPS_FABRIC_IOCTL] =		{ 40, 21, 20, 1 },
	[QCOM_RPM_APPS_FABRIC_ARB] =		{ 41, 22, 21, 12 },
	[QCOM_RPM_SYS_FABRIC_HALT] =		{ 53, 23, 22, 1 },
	[QCOM_RPM_SYS_FABRIC_MODE] =		{ 55, 24, 23, 1 },
	[QCOM_RPM_SYS_FABRIC_IOCTL] =		{ 58, 25, 24, 1 },
	[QCOM_RPM_SYS_FABRIC_ARB] =		{ 59, 26, 25, 30 },
	[QCOM_RPM_MM_FABRIC_HALT] =		{ 89, 27, 26, 1 },
	[QCOM_RPM_MM_FABRIC_MODE] =		{ 91, 28, 27, 1 },
	[QCOM_RPM_MM_FABRIC_IOCTL] =		{ 94, 29, 28, 1 },
	[QCOM_RPM_MM_FABRIC_ARB] =		{ 95, 30, 29, 21 },
	[QCOM_RPM_PM8921_SMPS1] =		{ 116, 31, 30, 2 },
	[QCOM_RPM_PM8921_SMPS2] =		{ 118, 33, 31, 2 },
	[QCOM_RPM_PM8921_SMPS3] =		{ 120, 35, 32, 2 },
	[QCOM_RPM_PM8921_SMPS4] =		{ 122, 37, 33, 2 },
	[QCOM_RPM_PM8921_SMPS5] =		{ 124, 39, 34, 2 },
	[QCOM_RPM_PM8921_SMPS6] =		{ 126, 41, 35, 2 },
	[QCOM_RPM_PM8921_SMPS7] =		{ 128, 43, 36, 2 },
	[QCOM_RPM_PM8921_SMPS8] =		{ 130, 45, 37, 2 },
	[QCOM_RPM_PM8921_LDO1] =		{ 132, 47, 38, 2 },
	[QCOM_RPM_PM8921_LDO2] =		{ 134, 49, 39, 2 },
	[QCOM_RPM_PM8921_LDO3] =		{ 136, 51, 40, 2 },
	[QCOM_RPM_PM8921_LDO4] =		{ 138, 53, 41, 2 },
	[QCOM_RPM_PM8921_LDO5] =		{ 140, 55, 42, 2 },
	[QCOM_RPM_PM8921_LDO6] =		{ 142, 57, 43, 2 },
	[QCOM_RPM_PM8921_LDO7] =		{ 144, 59, 44, 2 },
	[QCOM_RPM_PM8921_LDO8] =		{ 146, 61, 45, 2 },
	[QCOM_RPM_PM8921_LDO9] =		{ 148, 63, 46, 2 },
	[QCOM_RPM_PM8921_LDO10] =		{ 150, 65, 47, 2 },
	[QCOM_RPM_PM8921_LDO11] =		{ 152, 67, 48, 2 },
	[QCOM_RPM_PM8921_LDO12] =		{ 154, 69, 49, 2 },
	[QCOM_RPM_PM8921_LDO13] =		{ 156, 71, 50, 2 },
	[QCOM_RPM_PM8921_LDO14] =		{ 158, 73, 51, 2 },
	[QCOM_RPM_PM8921_LDO15] =		{ 160, 75, 52, 2 },
	[QCOM_RPM_PM8921_LDO16] =		{ 162, 77, 53, 2 },
	[QCOM_RPM_PM8921_LDO17] =		{ 164, 79, 54, 2 },
	[QCOM_RPM_PM8921_LDO18] =		{ 166, 81, 55, 2 },
	[QCOM_RPM_PM8921_LDO19] =		{ 168, 83, 56, 2 },
	[QCOM_RPM_PM8921_LDO20] =		{ 170, 85, 57, 2 },
	[QCOM_RPM_PM8921_LDO21] =		{ 172, 87, 58, 2 },
	[QCOM_RPM_PM8921_LDO22] =		{ 174, 89, 59, 2 },
	[QCOM_RPM_PM8921_LDO23] =		{ 176, 91, 60, 2 },
	[QCOM_RPM_PM8921_LDO24] =		{ 178, 93, 61, 2 },
	[QCOM_RPM_PM8921_LDO25] =		{ 180, 95, 62, 2 },
	[QCOM_RPM_PM8921_LDO26] =		{ 182, 97, 63, 2 },
	[QCOM_RPM_PM8921_LDO27] =		{ 184, 99, 64, 2 },
	[QCOM_RPM_PM8921_LDO28] =		{ 186, 101, 65, 2 },
	[QCOM_RPM_PM8921_LDO29] =		{ 188, 103, 66, 2 },
	[QCOM_RPM_PM8921_CLK1] =		{ 190, 105, 67, 2 },
	[QCOM_RPM_PM8921_CLK2] =		{ 192, 107, 68, 2 },
	[QCOM_RPM_PM8921_LVS1] =		{ 194, 109, 69, 1 },
	[QCOM_RPM_PM8921_LVS2] =		{ 195, 110, 70, 1 },
	[QCOM_RPM_PM8921_LVS3] =		{ 196, 111, 71, 1 },
	[QCOM_RPM_PM8921_LVS4] =		{ 197, 112, 72, 1 },
	[QCOM_RPM_PM8921_LVS5] =		{ 198, 113, 73, 1 },
	[QCOM_RPM_PM8921_LVS6] =		{ 199, 114, 74, 1 },
	[QCOM_RPM_PM8921_LVS7] =		{ 200, 115, 75, 1 },
	[QCOM_RPM_PM8821_SMPS1] =		{ 201, 116, 76, 2 },
	[QCOM_RPM_PM8821_SMPS2] =		{ 203, 118, 77, 2 },
	[QCOM_RPM_PM8821_LDO1] =		{ 205, 120, 78, 2 },
	[QCOM_RPM_PM8921_NCP] =			{ 207, 122, 80, 2 },
	[QCOM_RPM_CXO_BUFFERS] =		{ 209, 124, 81, 1 },
	[QCOM_RPM_USB_OTG_SWITCH] =		{ 210, 125, 82, 1 },
	[QCOM_RPM_HDMI_SWITCH] =		{ 211, 126, 83, 1 },
	[QCOM_RPM_DDR_DMM] =			{ 212, 127, 84, 2 },
	[QCOM_RPM_QDSS_CLK] =			{ 214, ~0, 7, 1 },
	[QCOM_RPM_VDDMIN_GPIO] =		{ 215, 131, 89, 1 },
};

static const struct qcom_rpm_data apq8064_template = {
	.version = 3,
	.resource_table = apq8064_rpm_resource_table,
	.n_resources = ARRAY_SIZE(apq8064_rpm_resource_table),
	.req_ctx_off = 3,
	.req_sel_off = 11,
	.ack_ctx_off = 15,
	.ack_sel_off = 23,
	.req_sel_size = 4,
	.ack_sel_size = 7,
};

static const struct qcom_rpm_resource msm8660_rpm_resource_table[] = {
	[QCOM_RPM_CXO_CLK] =			{ 32, 12, 5, 1 },
	[QCOM_RPM_PXO_CLK] =			{ 33, 13, 6, 1 },
	[QCOM_RPM_PLL_4] =			{ 34, 14, 7, 1 },
	[QCOM_RPM_APPS_FABRIC_CLK] =		{ 35, 15, 8, 1 },
	[QCOM_RPM_SYS_FABRIC_CLK] =		{ 36, 16, 9, 1 },
	[QCOM_RPM_MM_FABRIC_CLK] =		{ 37, 17, 10, 1 },
	[QCOM_RPM_DAYTONA_FABRIC_CLK] =		{ 38, 18, 11, 1 },
	[QCOM_RPM_SFPB_CLK] =			{ 39, 19, 12, 1 },
	[QCOM_RPM_CFPB_CLK] =			{ 40, 20, 13, 1 },
	[QCOM_RPM_MMFPB_CLK] =			{ 41, 21, 14, 1 },
	[QCOM_RPM_SMI_CLK] =			{ 42, 22, 15, 1 },
	[QCOM_RPM_EBI1_CLK] =			{ 43, 23, 16, 1 },
	[QCOM_RPM_APPS_L2_CACHE_CTL] =		{ 44, 24, 17, 1 },
	[QCOM_RPM_APPS_FABRIC_HALT] =		{ 45, 25, 18, 2 },
	[QCOM_RPM_APPS_FABRIC_MODE] =		{ 47, 26, 19, 3 },
	[QCOM_RPM_APPS_FABRIC_ARB] =		{ 51, 28, 21, 6 },
	[QCOM_RPM_SYS_FABRIC_HALT] =		{ 63, 29, 22, 2 },
	[QCOM_RPM_SYS_FABRIC_MODE] =		{ 65, 30, 23, 3 },
	[QCOM_RPM_SYS_FABRIC_ARB] =		{ 69, 32, 25, 22 },
	[QCOM_RPM_MM_FABRIC_HALT] =		{ 105, 33, 26, 2 },
	[QCOM_RPM_MM_FABRIC_MODE] =		{ 107, 34, 27, 3 },
	[QCOM_RPM_MM_FABRIC_ARB] =		{ 111, 36, 29, 23 },
	[QCOM_RPM_PM8901_SMPS0] =		{ 134, 37, 30, 2 },
	[QCOM_RPM_PM8901_SMPS1] =		{ 136, 39, 31, 2 },
	[QCOM_RPM_PM8901_SMPS2] =		{ 138, 41, 32, 2 },
	[QCOM_RPM_PM8901_SMPS3] =		{ 140, 43, 33, 2 },
	[QCOM_RPM_PM8901_SMPS4] =		{ 142, 45, 34, 2 },
	[QCOM_RPM_PM8901_LDO0] =		{ 144, 47, 35, 2 },
	[QCOM_RPM_PM8901_LDO1] =		{ 146, 49, 36, 2 },
	[QCOM_RPM_PM8901_LDO2] =		{ 148, 51, 37, 2 },
	[QCOM_RPM_PM8901_LDO3] =		{ 150, 53, 38, 2 },
	[QCOM_RPM_PM8901_LDO4] =		{ 152, 55, 39, 2 },
	[QCOM_RPM_PM8901_LDO5] =		{ 154, 57, 40, 2 },
	[QCOM_RPM_PM8901_LDO6] =		{ 156, 59, 41, 2 },
	[QCOM_RPM_PM8901_LVS0] =		{ 158, 61, 42, 1 },
	[QCOM_RPM_PM8901_LVS1] =		{ 159, 62, 43, 1 },
	[QCOM_RPM_PM8901_LVS2] =		{ 160, 63, 44, 1 },
	[QCOM_RPM_PM8901_LVS3] =		{ 161, 64, 45, 1 },
	[QCOM_RPM_PM8901_MVS] =			{ 162, 65, 46, 1 },
	[QCOM_RPM_PM8058_SMPS0] =		{ 163, 66, 47, 2 },
	[QCOM_RPM_PM8058_SMPS1] =		{ 165, 68, 48, 2 },
	[QCOM_RPM_PM8058_SMPS2] =		{ 167, 70, 49, 2 },
	[QCOM_RPM_PM8058_SMPS3] =		{ 169, 72, 50, 2 },
	[QCOM_RPM_PM8058_SMPS4] =		{ 171, 74, 51, 2 },
	[QCOM_RPM_PM8058_LDO0] =		{ 173, 76, 52, 2 },
	[QCOM_RPM_PM8058_LDO1] =		{ 175, 78, 53, 2 },
	[QCOM_RPM_PM8058_LDO2] =		{ 177, 80, 54, 2 },
	[QCOM_RPM_PM8058_LDO3] =		{ 179, 82, 55, 2 },
	[QCOM_RPM_PM8058_LDO4] =		{ 181, 84, 56, 2 },
	[QCOM_RPM_PM8058_LDO5] =		{ 183, 86, 57, 2 },
	[QCOM_RPM_PM8058_LDO6] =		{ 185, 88, 58, 2 },
	[QCOM_RPM_PM8058_LDO7] =		{ 187, 90, 59, 2 },
	[QCOM_RPM_PM8058_LDO8] =		{ 189, 92, 60, 2 },
	[QCOM_RPM_PM8058_LDO9] =		{ 191, 94, 61, 2 },
	[QCOM_RPM_PM8058_LDO10] =		{ 193, 96, 62, 2 },
	[QCOM_RPM_PM8058_LDO11] =		{ 195, 98, 63, 2 },
	[QCOM_RPM_PM8058_LDO12] =		{ 197, 100, 64, 2 },
	[QCOM_RPM_PM8058_LDO13] =		{ 199, 102, 65, 2 },
	[QCOM_RPM_PM8058_LDO14] =		{ 201, 104, 66, 2 },
	[QCOM_RPM_PM8058_LDO15] =		{ 203, 106, 67, 2 },
	[QCOM_RPM_PM8058_LDO16] =		{ 205, 108, 68, 2 },
	[QCOM_RPM_PM8058_LDO17] =		{ 207, 110, 69, 2 },
	[QCOM_RPM_PM8058_LDO18] =		{ 209, 112, 70, 2 },
	[QCOM_RPM_PM8058_LDO19] =		{ 211, 114, 71, 2 },
	[QCOM_RPM_PM8058_LDO20] =		{ 213, 116, 72, 2 },
	[QCOM_RPM_PM8058_LDO21] =		{ 215, 118, 73, 2 },
	[QCOM_RPM_PM8058_LDO22] =		{ 217, 120, 74, 2 },
	[QCOM_RPM_PM8058_LDO23] =		{ 219, 122, 75, 2 },
	[QCOM_RPM_PM8058_LDO24] =		{ 221, 124, 76, 2 },
	[QCOM_RPM_PM8058_LDO25] =		{ 223, 126, 77, 2 },
	[QCOM_RPM_PM8058_LVS0] =		{ 225, 128, 78, 1 },
	[QCOM_RPM_PM8058_LVS1] =		{ 226, 129, 79, 1 },
	[QCOM_RPM_PM8058_NCP] =			{ 227, 130, 80, 2 },
	[QCOM_RPM_CXO_BUFFERS] =		{ 229, 132, 81, 1 },
};

static const struct qcom_rpm_data msm8660_template = {
	.version = 2,
	.resource_table = msm8660_rpm_resource_table,
	.n_resources = ARRAY_SIZE(msm8660_rpm_resource_table),
	.req_ctx_off = 3,
	.req_sel_off = 11,
	.ack_ctx_off = 19,
	.ack_sel_off = 27,
	.req_sel_size = 7,
	.ack_sel_size = 7,
};

static const struct qcom_rpm_resource msm8960_rpm_resource_table[] = {
	[QCOM_RPM_CXO_CLK] =			{ 25, 9, 5, 1 },
	[QCOM_RPM_PXO_CLK] =			{ 26, 10, 6, 1 },
	[QCOM_RPM_APPS_FABRIC_CLK] =		{ 27, 11, 8, 1 },
	[QCOM_RPM_SYS_FABRIC_CLK] =		{ 28, 12, 9, 1 },
	[QCOM_RPM_MM_FABRIC_CLK] =		{ 29, 13, 10, 1 },
	[QCOM_RPM_DAYTONA_FABRIC_CLK] =		{ 30, 14, 11, 1 },
	[QCOM_RPM_SFPB_CLK] =			{ 31, 15, 12, 1 },
	[QCOM_RPM_CFPB_CLK] =			{ 32, 16, 13, 1 },
	[QCOM_RPM_MMFPB_CLK] =			{ 33, 17, 14, 1 },
	[QCOM_RPM_EBI1_CLK] =			{ 34, 18, 16, 1 },
	[QCOM_RPM_APPS_FABRIC_HALT] =		{ 35, 19, 18, 1 },
	[QCOM_RPM_APPS_FABRIC_MODE] =		{ 37, 20, 19, 1 },
	[QCOM_RPM_APPS_FABRIC_IOCTL] =		{ 40, 21, 20, 1 },
	[QCOM_RPM_APPS_FABRIC_ARB] =		{ 41, 22, 21, 12 },
	[QCOM_RPM_SYS_FABRIC_HALT] =		{ 53, 23, 22, 1 },
	[QCOM_RPM_SYS_FABRIC_MODE] =		{ 55, 24, 23, 1 },
	[QCOM_RPM_SYS_FABRIC_IOCTL] =		{ 58, 25, 24, 1 },
	[QCOM_RPM_SYS_FABRIC_ARB] =		{ 59, 26, 25, 29 },
	[QCOM_RPM_MM_FABRIC_HALT] =		{ 88, 27, 26, 1 },
	[QCOM_RPM_MM_FABRIC_MODE] =		{ 90, 28, 27, 1 },
	[QCOM_RPM_MM_FABRIC_IOCTL] =		{ 93, 29, 28, 1 },
	[QCOM_RPM_MM_FABRIC_ARB] =		{ 94, 30, 29, 23 },
	[QCOM_RPM_PM8921_SMPS1] =		{ 117, 31, 30, 2 },
	[QCOM_RPM_PM8921_SMPS2] =		{ 119, 33, 31, 2 },
	[QCOM_RPM_PM8921_SMPS3] =		{ 121, 35, 32, 2 },
	[QCOM_RPM_PM8921_SMPS4] =		{ 123, 37, 33, 2 },
	[QCOM_RPM_PM8921_SMPS5] =		{ 125, 39, 34, 2 },
	[QCOM_RPM_PM8921_SMPS6] =		{ 127, 41, 35, 2 },
	[QCOM_RPM_PM8921_SMPS7] =		{ 129, 43, 36, 2 },
	[QCOM_RPM_PM8921_SMPS8] =		{ 131, 45, 37, 2 },
	[QCOM_RPM_PM8921_LDO1] =		{ 133, 47, 38, 2 },
	[QCOM_RPM_PM8921_LDO2] =		{ 135, 49, 39, 2 },
	[QCOM_RPM_PM8921_LDO3] =		{ 137, 51, 40, 2 },
	[QCOM_RPM_PM8921_LDO4] =		{ 139, 53, 41, 2 },
	[QCOM_RPM_PM8921_LDO5] =		{ 141, 55, 42, 2 },
	[QCOM_RPM_PM8921_LDO6] =		{ 143, 57, 43, 2 },
	[QCOM_RPM_PM8921_LDO7] =		{ 145, 59, 44, 2 },
	[QCOM_RPM_PM8921_LDO8] =		{ 147, 61, 45, 2 },
	[QCOM_RPM_PM8921_LDO9] =		{ 149, 63, 46, 2 },
	[QCOM_RPM_PM8921_LDO10] =		{ 151, 65, 47, 2 },
	[QCOM_RPM_PM8921_LDO11] =		{ 153, 67, 48, 2 },
	[QCOM_RPM_PM8921_LDO12] =		{ 155, 69, 49, 2 },
	[QCOM_RPM_PM8921_LDO13] =		{ 157, 71, 50, 2 },
	[QCOM_RPM_PM8921_LDO14] =		{ 159, 73, 51, 2 },
	[QCOM_RPM_PM8921_LDO15] =		{ 161, 75, 52, 2 },
	[QCOM_RPM_PM8921_LDO16] =		{ 163, 77, 53, 2 },
	[QCOM_RPM_PM8921_LDO17] =		{ 165, 79, 54, 2 },
	[QCOM_RPM_PM8921_LDO18] =		{ 167, 81, 55, 2 },
	[QCOM_RPM_PM8921_LDO19] =		{ 169, 83, 56, 2 },
	[QCOM_RPM_PM8921_LDO20] =		{ 171, 85, 57, 2 },
	[QCOM_RPM_PM8921_LDO21] =		{ 173, 87, 58, 2 },
	[QCOM_RPM_PM8921_LDO22] =		{ 175, 89, 59, 2 },
	[QCOM_RPM_PM8921_LDO23] =		{ 177, 91, 60, 2 },
	[QCOM_RPM_PM8921_LDO24] =		{ 179, 93, 61, 2 },
	[QCOM_RPM_PM8921_LDO25] =		{ 181, 95, 62, 2 },
	[QCOM_RPM_PM8921_LDO26] =		{ 183, 97, 63, 2 },
	[QCOM_RPM_PM8921_LDO27] =		{ 185, 99, 64, 2 },
	[QCOM_RPM_PM8921_LDO28] =		{ 187, 101, 65, 2 },
	[QCOM_RPM_PM8921_LDO29] =		{ 189, 103, 66, 2 },
	[QCOM_RPM_PM8921_CLK1] =		{ 191, 105, 67, 2 },
	[QCOM_RPM_PM8921_CLK2] =		{ 193, 107, 68, 2 },
	[QCOM_RPM_PM8921_LVS1] =		{ 195, 109, 69, 1 },
	[QCOM_RPM_PM8921_LVS2] =		{ 196, 110, 70, 1 },
	[QCOM_RPM_PM8921_LVS3] =		{ 197, 111, 71, 1 },
	[QCOM_RPM_PM8921_LVS4] =		{ 198, 112, 72, 1 },
	[QCOM_RPM_PM8921_LVS5] =		{ 199, 113, 73, 1 },
	[QCOM_RPM_PM8921_LVS6] =		{ 200, 114, 74, 1 },
	[QCOM_RPM_PM8921_LVS7] =		{ 201, 115, 75, 1 },
	[QCOM_RPM_PM8921_NCP] =			{ 202, 116, 80, 2 },
	[QCOM_RPM_CXO_BUFFERS] =		{ 204, 118, 81, 1 },
	[QCOM_RPM_USB_OTG_SWITCH] =		{ 205, 119, 82, 1 },
	[QCOM_RPM_HDMI_SWITCH] =		{ 206, 120, 83, 1 },
	[QCOM_RPM_DDR_DMM] =			{ 207, 121, 84, 2 },
};

static const struct qcom_rpm_data msm8960_template = {
	.version = 3,
	.resource_table = msm8960_rpm_resource_table,
	.n_resources = ARRAY_SIZE(msm8960_rpm_resource_table),
	.req_ctx_off = 3,
	.req_sel_off = 11,
	.ack_ctx_off = 15,
	.ack_sel_off = 23,
	.req_sel_size = 4,
	.ack_sel_size = 7,
};

static const struct qcom_rpm_resource ipq806x_rpm_resource_table[] = {
	[QCOM_RPM_CXO_CLK] =			{ 25, 9, 5, 1 },
	[QCOM_RPM_PXO_CLK] =			{ 26, 10, 6, 1 },
	[QCOM_RPM_APPS_FABRIC_CLK] =		{ 27, 11, 8, 1 },
	[QCOM_RPM_SYS_FABRIC_CLK] =		{ 28, 12, 9, 1 },
	[QCOM_RPM_NSS_FABRIC_0_CLK] =		{ 29, 13, 10, 1 },
	[QCOM_RPM_DAYTONA_FABRIC_CLK] =		{ 30, 14, 11, 1 },
	[QCOM_RPM_SFPB_CLK] =			{ 31, 15, 12, 1 },
	[QCOM_RPM_CFPB_CLK] =			{ 32, 16, 13, 1 },
	[QCOM_RPM_NSS_FABRIC_1_CLK] =		{ 33, 17, 14, 1 },
	[QCOM_RPM_EBI1_CLK] =			{ 34, 18, 16, 1 },
	[QCOM_RPM_APPS_FABRIC_HALT] =		{ 35, 19, 18, 2 },
	[QCOM_RPM_APPS_FABRIC_MODE] =		{ 37, 20, 19, 3 },
	[QCOM_RPM_APPS_FABRIC_IOCTL] =		{ 40, 21, 20, 1 },
	[QCOM_RPM_APPS_FABRIC_ARB] =		{ 41, 22, 21, 12 },
	[QCOM_RPM_SYS_FABRIC_HALT] =		{ 53, 23, 22, 2 },
	[QCOM_RPM_SYS_FABRIC_MODE] =		{ 55, 24, 23, 3 },
	[QCOM_RPM_SYS_FABRIC_IOCTL] =		{ 58, 25, 24, 1 },
	[QCOM_RPM_SYS_FABRIC_ARB] =		{ 59, 26, 25, 30 },
	[QCOM_RPM_MM_FABRIC_HALT] =		{ 89, 27, 26, 2 },
	[QCOM_RPM_MM_FABRIC_MODE] =		{ 91, 28, 27, 3 },
	[QCOM_RPM_MM_FABRIC_IOCTL] =		{ 94, 29, 28, 1 },
	[QCOM_RPM_MM_FABRIC_ARB] =		{ 95, 30, 29, 2 },
	[QCOM_RPM_CXO_BUFFERS] =		{ 209, 33, 31, 1 },
	[QCOM_RPM_USB_OTG_SWITCH] =		{ 210, 34, 32, 1 },
	[QCOM_RPM_HDMI_SWITCH] =		{ 211, 35, 33, 1 },
	[QCOM_RPM_DDR_DMM] =			{ 212, 36, 34, 2 },
	[QCOM_RPM_VDDMIN_GPIO] =		{ 215, 40, 39, 1 },
	[QCOM_RPM_SMB208_S1a] =			{ 216, 41, 90, 2 },
	[QCOM_RPM_SMB208_S1b] =			{ 218, 43, 91, 2 },
	[QCOM_RPM_SMB208_S2a] =			{ 220, 45, 92, 2 },
	[QCOM_RPM_SMB208_S2b] =			{ 222, 47, 93, 2 },
};

static const struct qcom_rpm_data ipq806x_template = {
	.version = 3,
	.resource_table = ipq806x_rpm_resource_table,
	.n_resources = ARRAY_SIZE(ipq806x_rpm_resource_table),
	.req_ctx_off = 3,
	.req_sel_off = 11,
	.ack_ctx_off = 15,
	.ack_sel_off = 23,
	.req_sel_size = 4,
	.ack_sel_size = 7,
};

static const struct qcom_rpm_resource mdm9615_rpm_resource_table[] = {
	[QCOM_RPM_CXO_CLK] =			{ 25, 9, 5, 1 },
	[QCOM_RPM_SYS_FABRIC_CLK] =		{ 26, 10, 9, 1 },
	[QCOM_RPM_DAYTONA_FABRIC_CLK] =		{ 27, 11, 11, 1 },
	[QCOM_RPM_SFPB_CLK] =			{ 28, 12, 12, 1 },
	[QCOM_RPM_CFPB_CLK] =			{ 29, 13, 13, 1 },
	[QCOM_RPM_EBI1_CLK] =			{ 30, 14, 16, 1 },
	[QCOM_RPM_APPS_FABRIC_HALT] =		{ 31, 15, 22, 2 },
	[QCOM_RPM_APPS_FABRIC_MODE] =		{ 33, 16, 23, 3 },
	[QCOM_RPM_APPS_FABRIC_IOCTL] =		{ 36, 17, 24, 1 },
	[QCOM_RPM_APPS_FABRIC_ARB] =		{ 37, 18, 25, 27 },
	[QCOM_RPM_PM8018_SMPS1] =		{ 64, 19, 30, 2 },
	[QCOM_RPM_PM8018_SMPS2] =		{ 66, 21, 31, 2 },
	[QCOM_RPM_PM8018_SMPS3] =		{ 68, 23, 32, 2 },
	[QCOM_RPM_PM8018_SMPS4] =		{ 70, 25, 33, 2 },
	[QCOM_RPM_PM8018_SMPS5] =		{ 72, 27, 34, 2 },
	[QCOM_RPM_PM8018_LDO1] =		{ 74, 29, 35, 2 },
	[QCOM_RPM_PM8018_LDO2] =		{ 76, 31, 36, 2 },
	[QCOM_RPM_PM8018_LDO3] =		{ 78, 33, 37, 2 },
	[QCOM_RPM_PM8018_LDO4] =		{ 80, 35, 38, 2 },
	[QCOM_RPM_PM8018_LDO5] =		{ 82, 37, 39, 2 },
	[QCOM_RPM_PM8018_LDO6] =		{ 84, 39, 40, 2 },
	[QCOM_RPM_PM8018_LDO7] =		{ 86, 41, 41, 2 },
	[QCOM_RPM_PM8018_LDO8] =		{ 88, 43, 42, 2 },
	[QCOM_RPM_PM8018_LDO9] =		{ 90, 45, 43, 2 },
	[QCOM_RPM_PM8018_LDO10] =		{ 92, 47, 44, 2 },
	[QCOM_RPM_PM8018_LDO11] =		{ 94, 49, 45, 2 },
	[QCOM_RPM_PM8018_LDO12] =		{ 96, 51, 46, 2 },
	[QCOM_RPM_PM8018_LDO13] =		{ 98, 53, 47, 2 },
	[QCOM_RPM_PM8018_LDO14] =		{ 100, 55, 48, 2 },
	[QCOM_RPM_PM8018_LVS1] =		{ 102, 57, 49, 1 },
	[QCOM_RPM_PM8018_NCP] =			{ 103, 58, 80, 2 },
	[QCOM_RPM_CXO_BUFFERS] =		{ 105, 60, 81, 1 },
	[QCOM_RPM_USB_OTG_SWITCH] =		{ 106, 61, 82, 1 },
	[QCOM_RPM_HDMI_SWITCH] =		{ 107, 62, 83, 1 },
	[QCOM_RPM_VOLTAGE_CORNER] =		{ 109, 64, 87, 1 },
};

static const struct qcom_rpm_data mdm9615_template = {
	.version = 3,
	.resource_table = mdm9615_rpm_resource_table,
	.n_resources = ARRAY_SIZE(mdm9615_rpm_resource_table),
	.req_ctx_off = 3,
	.req_sel_off = 11,
	.ack_ctx_off = 15,
	.ack_sel_off = 23,
	.req_sel_size = 4,
	.ack_sel_size = 7,
};

static const struct of_device_id qcom_rpm_of_match[] = {
	{ .compatible = "qcom,rpm-apq8064", .data = &apq8064_template },
	{ .compatible = "qcom,rpm-msm8660", .data = &msm8660_template },
	{ .compatible = "qcom,rpm-msm8960", .data = &msm8960_template },
	{ .compatible = "qcom,rpm-ipq8064", .data = &ipq806x_template },
	{ .compatible = "qcom,rpm-mdm9615", .data = &mdm9615_template },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_rpm_of_match);

int qcom_rpm_write(struct qcom_rpm *rpm,
		   int state,
		   int resource,
		   u32 *buf, size_t count)
{
	const struct qcom_rpm_resource *res;
	const struct qcom_rpm_data *data = rpm->data;
	u32 sel_mask[RPM_MAX_SEL_SIZE] = { 0 };
	int left;
	int ret = 0;
	int i;

	if (WARN_ON(resource < 0 || resource >= data->n_resources))
		return -EINVAL;

	res = &data->resource_table[resource];
	if (WARN_ON(res->size != count))
		return -EINVAL;

	/*
	 * Reject writes once .suspend_late has armed the suspend guard.
	 * Beyond that point dpm_suspend_noirq() will mask the ack IRQ at
	 * the GIC, so wait_for_completion_timeout() below would spin a
	 * full 5*HZ and return -ETIMEDOUT -- with rpm->lock held, dragging
	 * every other RPM caller into the same wait. Fail fast instead.
	 * See the struct qcom_rpm.suspended comment for context.
	 */
	if (READ_ONCE(rpm->suspended))
		return -EAGAIN;

	mutex_lock(&rpm->lock);

	for (i = 0; i < res->size; i++)
		writel_relaxed(buf[i], RPM_REQ_REG(rpm, res->target_id + i));

	bitmap_set((unsigned long *)sel_mask, res->select_id, 1);
	for (i = 0; i < rpm->data->req_sel_size; i++) {
		writel_relaxed(sel_mask[i],
			       RPM_CTRL_REG(rpm, rpm->data->req_sel_off + i));
	}

	writel_relaxed(BIT(state), RPM_CTRL_REG(rpm, rpm->data->req_ctx_off));

	reinit_completion(&rpm->ack);
	regmap_write(rpm->ipc_regmap, rpm->ipc_offset, BIT(rpm->ipc_bit));

	left = wait_for_completion_timeout(&rpm->ack, RPM_REQUEST_TIMEOUT);
	if (!left)
		ret = -ETIMEDOUT;
	else if (rpm->ack_status & RPM_REJECTED)
		ret = -EIO;

	mutex_unlock(&rpm->lock);

	return ret;
}
EXPORT_SYMBOL(qcom_rpm_write);

/**
 * qcom_rpm_write_sync() - vote and (best-effort) wait for the RPM
 *                         firmware to apply it.
 * @rpm:      RPM handle.
 * @state:    QCOM_RPM_ACTIVE_STATE or QCOM_RPM_SLEEP_STATE.
 * @resource: resource ID (index into the per-SoC resource table).
 * @buf:      values to write (length must match resource->size).
 * @count:    number of u32 words; must match resource->size, like
 *            qcom_rpm_write().
 *
 * qcom_rpm_write() submits a vote and waits for the RPM ack IRQ.  The
 * ack only acknowledges that the request was queued in the shared-memory
 * request region; it does not mean the RPM firmware has applied the new
 * value to the physical resource yet.  Consumers that vote a higher
 * clock/bandwidth rate and then immediately drive hardware traffic at
 * that rate (e.g. icc consumers handing off to a DMA submit) can race
 * the ramp.
 *
 * This helper queues the vote via qcom_rpm_write() and then polls the
 * per-resource status register until the applied value is at least the
 * value we voted for, or RPM_SYNC_TIMEOUT_US elapses.  RPM aggregates
 * concurrent votes by max() for clock / regulator / fabric resources,
 * so "applied >= requested" is the correct success criterion even when
 * other consumers are voting higher.
 *
 * Returns 0 in all of:
 *
 *   - status converged (vote was applied);
 *   - timeout (status never reached the requested value): logged once
 *     per resource as a WARN with the observed values, then treated as
 *     fire-and-forget.  Some SoC families' status registers do not
 *     reflect the requested unit (e.g. report MHz when we vote kHz, or
 *     report a coarse-grained state token rather than the rate), making
 *     polling pointless; callers should not propagate the timeout as
 *     an error in those cases;
 *   - SLEEP_STATE votes (status register only reflects sleep-context
 *     values during cluster sleep);
 *   - resources without a readable status register (status_id = ~0,
 *     only QCOM_RPM_QDSS_CLK today).
 *
 * Returns whatever qcom_rpm_write() returns on the queue step
 * (-EIO on RPM rejection, -ETIMEDOUT on ack timeout, -EAGAIN if
 * suspended).
 *
 * Diagnostic logging: the first call per (resource, state) pair
 * captures the status register before and immediately after the queue
 * write, and the final outcome (convergence latency or final stuck
 * value).  These data points let the caller observe whether the
 * status register on a given SoC actually tracks the requested vote
 * value.
 */
int qcom_rpm_write_sync(struct qcom_rpm *rpm,
			int state,
			int resource,
			u32 *buf, size_t count)
{
	const struct qcom_rpm_resource *res;
	u32 status_before = 0, status_after_queue = 0;
	ktime_t t_queue, t_done, deadline;
	bool diag;
	int ret;

	if (WARN_ON(resource < 0 || resource >= rpm->data->n_resources))
		return -EINVAL;

	res = &rpm->data->resource_table[resource];
	if (WARN_ON(res->size != count))
		return -EINVAL;

	/*
	 * One-shot diagnostic per (resource, state) pair, gated on a
	 * static bitmap so we get exactly one sample per call site
	 * across the lifetime of the driver.  Two states (ACTIVE,
	 * SLEEP) per resource: index = resource * 2 + state.  Sized
	 * for the largest resource table on any supported SoC.
	 */
#define RPM_DIAG_MAX_RESOURCES	256
	{
		static DECLARE_BITMAP(diag_done, RPM_DIAG_MAX_RESOURCES * 2);
		unsigned int bit = (unsigned int)resource * 2 + (state & 1);

		diag = (bit < RPM_DIAG_MAX_RESOURCES * 2) &&
		       !test_and_set_bit(bit, diag_done);
	}

	if (diag && state == QCOM_RPM_ACTIVE_STATE && res->status_id != ~0u)
		status_before = readl(RPM_STATUS_REG(rpm, res->status_id));

	t_queue = ktime_get();
	ret = qcom_rpm_write(rpm, state, resource, buf, count);
	if (ret) {
		if (diag)
			dev_warn(rpm->dev,
				 "rpm_sync diag: res=%d state=%d voted=%u kHz queue write failed: %d\n",
				 resource, state, buf[0], ret);
		return ret;
	}

	/*
	 * Sleep votes only land during cluster-sleep transitions, so the
	 * status register cannot be polled against them while the SoC
	 * is running.  Treat a queued sleep vote as committed.
	 */
	if (state != QCOM_RPM_ACTIVE_STATE) {
		if (diag)
			dev_info(rpm->dev,
				 "rpm_sync diag: res=%d state=SLEEP voted=%u kHz queue=%lld us (no status poll)\n",
				 resource, buf[0],
				 ktime_us_delta(ktime_get(), t_queue));
		return 0;
	}

	/*
	 * Resources without a readable status register cannot be polled
	 * (only QCOM_RPM_QDSS_CLK on apq8064 today, status_id = ~0).
	 * Behave like qcom_rpm_write() for those.
	 */
	if (res->status_id == ~0u) {
		if (diag)
			dev_info(rpm->dev,
				 "rpm_sync diag: res=%d state=ACTIVE voted=%u kHz queue=%lld us (no status reg)\n",
				 resource, buf[0],
				 ktime_us_delta(ktime_get(), t_queue));
		return 0;
	}

	if (diag)
		status_after_queue = readl(RPM_STATUS_REG(rpm, res->status_id));

	deadline = ktime_add_us(ktime_get(), RPM_SYNC_TIMEOUT_US);
	for (;;) {
		bool all_ok = true;
		int i;

		for (i = 0; i < count; i++) {
			u32 applied = readl(RPM_STATUS_REG(rpm,
							   res->status_id + i));

			if (applied < buf[i]) {
				all_ok = false;
				break;
			}
		}

		if (all_ok) {
			t_done = ktime_get();
			if (diag)
				dev_info(rpm->dev,
					 "rpm_sync diag: res=%d state=ACTIVE voted=%u kHz status[before]=%u [after-queue]=%u CONVERGED in %lld us (queue=%lld us, poll=%lld us)\n",
					 resource, buf[0],
					 status_before, status_after_queue,
					 ktime_us_delta(t_done, t_queue),
					 ktime_us_delta(deadline, t_queue) - RPM_SYNC_TIMEOUT_US + ktime_us_delta(t_done, t_queue),
					 ktime_us_delta(t_done, t_queue));
			return 0;
		}

		if (!ktime_before(ktime_get(), deadline)) {
			u32 final = readl(RPM_STATUS_REG(rpm, res->status_id));

			/*
			 * Status never converged.  Log a one-shot diag if
			 * this is the first sample for this (resource, state),
			 * and always emit a WARN once globally so a stuck-status
			 * SoC is noticed without spamming.  Then fall through
			 * to fire-and-forget: the queue write itself succeeded.
			 */
			if (diag)
				dev_warn(rpm->dev,
					 "rpm_sync diag: res=%d state=ACTIVE voted=%u kHz status[before]=%u [after-queue]=%u TIMEOUT after %d us final_status=%u (treating as fire-and-forget)\n",
					 resource, buf[0],
					 status_before, status_after_queue,
					 RPM_SYNC_TIMEOUT_US, final);
			WARN_ONCE(1,
				  "qcom_rpm_write_sync: status register did not converge on this SoC; treating sync writes as fire-and-forget\n");
			return 0;
		}

		udelay(RPM_SYNC_POLL_US);
	}
}
EXPORT_SYMBOL_GPL(qcom_rpm_write_sync);

static irqreturn_t qcom_rpm_ack_interrupt(int irq, void *dev)
{
	struct qcom_rpm *rpm = dev;
	u32 ack;
	int i;

	ack = readl_relaxed(RPM_CTRL_REG(rpm, rpm->data->ack_ctx_off));
	for (i = 0; i < rpm->data->ack_sel_size; i++)
		writel_relaxed(0,
			RPM_CTRL_REG(rpm, rpm->data->ack_sel_off + i));
	writel(0, RPM_CTRL_REG(rpm, rpm->data->ack_ctx_off));

	if (ack & RPM_NOTIFICATION) {
		dev_warn(rpm->dev, "ignoring notification!\n");
	} else {
		rpm->ack_status = ack;
		complete(&rpm->ack);
	}

	return IRQ_HANDLED;
}

static irqreturn_t qcom_rpm_err_interrupt(int irq, void *dev)
{
	struct qcom_rpm *rpm = dev;

	regmap_write(rpm->ipc_regmap, rpm->ipc_offset, BIT(rpm->ipc_bit));
	dev_err(rpm->dev, "RPM triggered fatal error\n");

	return IRQ_HANDLED;
}

static irqreturn_t qcom_rpm_wakeup_interrupt(int irq, void *dev)
{
	return IRQ_HANDLED;
}

static int qcom_rpm_probe(struct platform_device *pdev)
{
	struct device_node *syscon_np;
	struct qcom_rpm *rpm;
	u32 fw_version[3];
	int irq_wakeup;
	int irq_ack;
	int irq_err;
	int ret;

	rpm = devm_kzalloc(&pdev->dev, sizeof(*rpm), GFP_KERNEL);
	if (!rpm)
		return -ENOMEM;

	rpm->dev = &pdev->dev;
	mutex_init(&rpm->lock);
	init_completion(&rpm->ack);

	/* Enable message RAM clock */
	rpm->ramclk = devm_clk_get_enabled(&pdev->dev, "ram");
	if (IS_ERR(rpm->ramclk)) {
		ret = PTR_ERR(rpm->ramclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		/*
		 * Fall through in all other cases, as the clock is
		 * optional. (Does not exist on all platforms.)
		 */
		rpm->ramclk = NULL;
	}

	irq_ack = platform_get_irq_byname(pdev, "ack");
	if (irq_ack < 0)
		return irq_ack;

	irq_err = platform_get_irq_byname(pdev, "err");
	if (irq_err < 0)
		return irq_err;

	irq_wakeup = platform_get_irq_byname(pdev, "wakeup");
	if (irq_wakeup < 0)
		return irq_wakeup;

	rpm->data = device_get_match_data(&pdev->dev);
	if (!rpm->data)
		return -ENODEV;

	rpm->status_regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(rpm->status_regs))
		return PTR_ERR(rpm->status_regs);
	rpm->ctrl_regs = rpm->status_regs + 0x400;
	rpm->req_regs = rpm->status_regs + 0x600;

	syscon_np = of_parse_phandle(pdev->dev.of_node, "qcom,ipc", 0);
	if (!syscon_np) {
		dev_err(&pdev->dev, "no qcom,ipc node\n");
		return -ENODEV;
	}

	rpm->ipc_regmap = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);
	if (IS_ERR(rpm->ipc_regmap))
		return PTR_ERR(rpm->ipc_regmap);

	ret = of_property_read_u32_index(pdev->dev.of_node, "qcom,ipc", 1,
					 &rpm->ipc_offset);
	if (ret < 0) {
		dev_err(&pdev->dev, "no offset in qcom,ipc\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_index(pdev->dev.of_node, "qcom,ipc", 2,
					 &rpm->ipc_bit);
	if (ret < 0) {
		dev_err(&pdev->dev, "no bit in qcom,ipc\n");
		return -EINVAL;
	}

	dev_set_drvdata(&pdev->dev, rpm);

	fw_version[0] = readl(RPM_STATUS_REG(rpm, 0));
	fw_version[1] = readl(RPM_STATUS_REG(rpm, 1));
	fw_version[2] = readl(RPM_STATUS_REG(rpm, 2));
	if (fw_version[0] != rpm->data->version) {
		dev_err(&pdev->dev,
			"RPM version %u.%u.%u incompatible with driver version %u",
			fw_version[0],
			fw_version[1],
			fw_version[2],
			rpm->data->version);
		return -EFAULT;
	}

	writel(fw_version[0], RPM_CTRL_REG(rpm, 0));
	writel(fw_version[1], RPM_CTRL_REG(rpm, 1));
	writel(fw_version[2], RPM_CTRL_REG(rpm, 2));

	dev_info(&pdev->dev, "RPM firmware %u.%u.%u\n", fw_version[0],
							fw_version[1],
							fw_version[2]);

	ret = devm_request_irq(&pdev->dev,
			       irq_ack,
			       qcom_rpm_ack_interrupt,
			       IRQF_TRIGGER_RISING,
			       "qcom_rpm_ack",
			       rpm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request ack interrupt\n");
		return ret;
	}

	ret = irq_set_irq_wake(irq_ack, 1);
	if (ret)
		dev_warn(&pdev->dev, "failed to mark ack irq as wakeup\n");

	ret = devm_request_irq(&pdev->dev,
			       irq_err,
			       qcom_rpm_err_interrupt,
			       IRQF_TRIGGER_RISING,
			       "qcom_rpm_err",
			       rpm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request err interrupt\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev,
			       irq_wakeup,
			       qcom_rpm_wakeup_interrupt,
			       IRQF_TRIGGER_RISING,
			       "qcom_rpm_wakeup",
			       rpm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request wakeup interrupt\n");
		return ret;
	}

	ret = irq_set_irq_wake(irq_wakeup, 1);
	if (ret)
		dev_warn(&pdev->dev, "failed to mark wakeup irq as wakeup\n");

	return devm_of_platform_populate(&pdev->dev);
}

static int __maybe_unused qcom_rpm_suspend_late(struct device *dev)
{
	struct qcom_rpm *rpm = dev_get_drvdata(dev);

	/*
	 * .suspend_late runs after every device's .suspend has completed
	 * (so consumers that need RPM votes during their own suspend have
	 * already issued them) but before dpm_suspend_noirq() calls
	 * suspend_device_irqs(). Arming the guard here ensures both that
	 * (a) the MMCC footswitch power_off cascade reached from
	 * genpd_finish_suspend at .suspend_noirq sees the flag and skips
	 * its qcom_rpm_write attempts, and (b) any other late RPM caller
	 * (regulator set_load, qnoc ARB write) also fails fast instead of
	 * stalling on the masked ack IRQ.
	 *
	 * WRITE_ONCE for visibility; full SMP synchronisation isn't needed
	 * because by .suspend_late the PM core has already serialised
	 * device suspends via dpm_list_mtx + the suspend_test_finish flush.
	 */
	WRITE_ONCE(rpm->suspended, true);
	return 0;
}

static int __maybe_unused qcom_rpm_resume_early(struct device *dev)
{
	struct qcom_rpm *rpm = dev_get_drvdata(dev);

	/*
	 * .resume_early runs after dpm_resume_noirq() has called
	 * resume_device_irqs() to re-enable our ack IRQ at the GIC. Clear
	 * the guard so the first .resume_early caller that needs RPM
	 * (e.g. drm/msm/mdp4 issuing its NoC port unhalt) can issue an
	 * IPC normally.
	 */
	WRITE_ONCE(rpm->suspended, false);
	return 0;
}

static const struct dev_pm_ops qcom_rpm_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(qcom_rpm_suspend_late,
				     qcom_rpm_resume_early)
};

static struct platform_driver qcom_rpm_driver = {
	.probe = qcom_rpm_probe,
	.driver  = {
		.name  = "qcom_rpm",
		.of_match_table = qcom_rpm_of_match,
		.pm	= &qcom_rpm_pm_ops,
	},
};

static int __init qcom_rpm_init(void)
{
	return platform_driver_register(&qcom_rpm_driver);
}
arch_initcall(qcom_rpm_init);

static void __exit qcom_rpm_exit(void)
{
	platform_driver_unregister(&qcom_rpm_driver);
}
module_exit(qcom_rpm_exit)

MODULE_DESCRIPTION("Qualcomm Resource Power Manager driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
