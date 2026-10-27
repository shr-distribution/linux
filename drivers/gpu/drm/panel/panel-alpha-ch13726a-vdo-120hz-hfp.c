// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "../../../misc/mediatek/gate_ic/gate_i2c.h"

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
//#include "lcm_i2c.h"

//#define LCM_POWER_CTL

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *avdd_en_gpio;
//	struct gpio_desc *ocp2133_fd_gpio;
	struct gpio_desc *disp_pwm_gpio;
//	struct regulator *vdd2v8;
#ifdef LCM_POWER_CTL
	struct regulator *avdd;
	struct regulator *iovdd;
#endif
	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif
static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}



static void lcm_panel_init(struct lcm *ctx)
{
//	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(20);
	gpiod_set_value(ctx->avdd_en_gpio, 1);
//	devm_gpiod_put(ctx->dev, ctx->avdd_en_gpio);
	msleep(10);
//	gpiod_set_value(ctx->ocp2133_fd_gpio, 1);
//	devm_gpiod_put(ctx->dev, ctx->ocp2133_fd_gpio);
//	udelay(10 * 1000);
	gpiod_set_value(ctx->disp_pwm_gpio, 1);
//	devm_gpiod_put(ctx->dev, ctx->disp_pwm_gpio);
	//-------------  Display Initial Code Setting  -------------------------	
lcm_dcs_write_seq_static(ctx,0x2a,0x00,0x00,0x04,0x37);
lcm_dcs_write_seq_static(ctx,0x2b,0x00,0x00,0x04,0xd7);
lcm_dcs_write_seq_static(ctx,0x8f,0x04,0x88,0x77);
//lcm_dcs_write_seq_static(ctx,0x51,0xff);
lcm_dcs_write_seq_static(ctx,0x51,0x80);
//lcm_dcs_write_seq_static(ctx,0x53,0x00);
lcm_dcs_write_seq_static(ctx,0x35,0x00);
lcm_dcs_write_seq_static(ctx,0x60,0x80);
lcm_dcs_write_seq_static(ctx,0x44,0x02,0x6c);
lcm_dcs_write_seq_static(ctx,0xf0,0x50);
//lcm_dcs_write_seq_static(ctx,0xe0,0xaa);
lcm_dcs_write_seq_static(ctx,0xb0,0x0e,0x0e);
lcm_dcs_write_seq_static(ctx,0xb2,0x80);
lcm_dcs_write_seq_static(ctx,0xb3,0x00,0x1e);
lcm_dcs_write_seq_static(ctx,0xb4,0x92,0x18,0x92);
lcm_dcs_write_seq_static(ctx,0xb6,0x00,0x3f,0x00);
lcm_dcs_write_seq_static(ctx,0xb7,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xb8,0x02,0x62);
lcm_dcs_write_seq_static(ctx,0xb9,0x11);
lcm_dcs_write_seq_static(ctx,0xbb,0x08,0x08);
lcm_dcs_write_seq_static(ctx,0xc0,0x80);
lcm_dcs_write_seq_static(ctx,0xc7,0x40);
lcm_dcs_write_seq_static(ctx,0xc9,0x55,0xf8,0xf8);
lcm_dcs_write_seq_static(ctx,0xe8,0x58);
lcm_dcs_write_seq_static(ctx,0xec,0x08);
lcm_dcs_write_seq_static(ctx,0xf0,0x51);
lcm_dcs_write_seq_static(ctx,0xb0,0x28);
lcm_dcs_write_seq_static(ctx,0xb1,0x28);
lcm_dcs_write_seq_static(ctx,0xb2,0x1c);
lcm_dcs_write_seq_static(ctx,0xb3,0x1c);
lcm_dcs_write_seq_static(ctx,0xc0,0x2d);
lcm_dcs_write_seq_static(ctx,0xc1,0x18,0x18);
lcm_dcs_write_seq_static(ctx,0xc3,0x47,0x04);
lcm_dcs_write_seq_static(ctx,0xc4,0x28);
lcm_dcs_write_seq_static(ctx,0xc9,0x1b);
lcm_dcs_write_seq_static(ctx,0xd1,0x4a,0x4a,0x80,0x4f,0x69);
lcm_dcs_write_seq_static(ctx,0xd2,0x05);
lcm_dcs_write_seq_static(ctx,0xd3,0x11,0x00,0x00,0x64);
lcm_dcs_write_seq_static(ctx,0xd7,0xa8,0x5a,0xaa,0xaa);
lcm_dcs_write_seq_static(ctx,0xd8,0xa8,0x5a,0xaa,0xaa);
lcm_dcs_write_seq_static(ctx,0xdd,0xaa,0xaa,0xaa,0xaa);
lcm_dcs_write_seq_static(ctx,0xf0,0x52);
lcm_dcs_write_seq_static(ctx,0xb0,0x21,0x94,0x69);
lcm_dcs_write_seq_static(ctx,0xba,0xff,0x35,0x43,0x52,0x61,0xff,0x71,0x79,0x7d,0x80,0x0f,0x85,0x87);
lcm_dcs_write_seq_static(ctx,0xb9,0xaa,0x21,0x45,0x67,0x86,0xaa,0x95,0xa4,0xc0,0xde,0xfe,0xfa,0x09,0x18,0x26);
lcm_dcs_write_seq_static(ctx,0xb8,0x00,0x00,0xda,0xed,0xf8,0x55,0x29,0x51,0x6a,0x7b,0x55,0xa1,0xc2,0xdf,0xf7);
lcm_dcs_write_seq_static(ctx,0xb7,0xfa,0xec,0xfa,0x07,0x16,0xff,0x24,0x2b,0x2e,0x31,0x0f,0x36,0x37);
lcm_dcs_write_seq_static(ctx,0xb6,0xa9,0xec,0x0f,0x2d,0x4a,0xaa,0x58,0x66,0x81,0x9c,0xaa,0xb6,0xc3,0xd0,0xde);
lcm_dcs_write_seq_static(ctx,0xb5,0x00,0x00,0xd8,0xe3,0xea,0x55,0x03,0x22,0x3b,0x51,0x55,0x76,0x95,0xaf,0xc5);
lcm_dcs_write_seq_static(ctx,0xb4,0xff,0x14,0x23,0x32,0x41,0xff,0x51,0x59,0x5d,0x61,0x0f,0x65,0x67);
lcm_dcs_write_seq_static(ctx,0xb3,0xaa,0x05,0x28,0x48,0x67,0xaa,0x76,0x84,0xa0,0xbc,0xea,0xd9,0xe8,0xf7,0x05);
lcm_dcs_write_seq_static(ctx,0xb2,0x00,0x00,0xc0,0xcd,0xcf,0x55,0x15,0x39,0x52,0x69,0x55,0x8f,0xad,0xc6,0xdd);
lcm_dcs_write_seq_static(ctx,0xf0,0x53);
lcm_dcs_write_seq_static(ctx,0xb0,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xb3,0xa8,0x10,0x3c,0x0a,0x03,0x77,0x77);
lcm_dcs_write_seq_static(ctx,0xb4,0x26,0x00,0x00,0x05,0x00,0x77,0x77);
lcm_dcs_write_seq_static(ctx,0xb9,0x24,0x90,0x00,0x77,0x17,0x31,0x22,0x90,0x00,0x77,0x17,0x31);
lcm_dcs_write_seq_static(ctx,0xba,0x20,0x90,0x00,0x77,0x03,0x10,0x21,0x90,0x00,0x77,0x03,0x10);
lcm_dcs_write_seq_static(ctx,0xbe,0x20,0x09,0x00,0x00,0x2c,0x20,0x46,0x00,0x00,0x2c          );
lcm_dcs_write_seq_static(ctx,0xc1,0x13,0x12,0x0b,0x0a,0x02,0x09,0x08,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xc3,0x00,0x00,0x00,0x00,0x00,0x08,0x09,0x03,0x0a,0x0b,0x12,0x13);
lcm_dcs_write_seq_static(ctx,0xf0,0x55);
lcm_dcs_write_seq_static(ctx,0xb1,0x80,0x00);
lcm_dcs_write_seq_static(ctx,0xb3,0x7f,0x7f,0x00,0x00,0x77,0x87,0x32,0xaa,0x21);
lcm_dcs_write_seq_static(ctx,0xb4,0x2a,0xa9,0x2a,0x21,0xaa,0x32,0x00,0x87,0x77);
lcm_dcs_write_seq_static(ctx,0xb5,0x32,0xaa,0x21,0x32,0xaa,0x21,0x77,0x87,0x00);
lcm_dcs_write_seq_static(ctx,0xb6,0x00,0x87,0x77,0x87,0x77,0x00,0x21,0xaa,0x32);
lcm_dcs_write_seq_static(ctx,0xc3,0x7f,0x7f,0x00,0x00,0x47,0x51,0x32,0xaa,0x21);
lcm_dcs_write_seq_static(ctx,0xc4,0x2a,0xa9,0x2a,0x21,0x66,0x1e,0x00,0x87,0x77);
lcm_dcs_write_seq_static(ctx,0xc5,0x1e,0x66,0x14,0x32,0xaa,0x21,0x77,0x87,0x00);
lcm_dcs_write_seq_static(ctx,0xc6,0x00,0x51,0x47,0x87,0x77,0x00,0x21,0xaa,0x32);
lcm_dcs_write_seq_static(ctx,0xc7,0x7f,0x7f,0x00,0x00,0x77,0x87,0x32,0xaa,0x21);
lcm_dcs_write_seq_static(ctx,0xc8,0x32,0xaa,0x21,0x32,0xaa,0x21,0x77,0x87,0x00);
lcm_dcs_write_seq_static(ctx,0xc4,0x2a,0xa9,0x2a,0x21,0xaa,0x32,0x00,0x87,0x77);
lcm_dcs_write_seq_static(ctx,0xb6,0x00,0x87,0x77,0x87,0x77,0x00,0x21,0xaa,0x32);
lcm_dcs_write_seq_static(ctx,0xf0,0x58);
lcm_dcs_write_seq_static(ctx,0xc0,0x83,0xad,0x52,0x00);
lcm_dcs_write_seq_static(ctx,0xc1,0xbf,0x7f,0x3f,0x2f,0x1f,0x0f,0x07,0x00);
lcm_dcs_write_seq_static(ctx,0xc2,0xff);
lcm_dcs_write_seq_static(ctx,0xc3,0x7f,0x05);
lcm_dcs_write_seq_static(ctx,0xc4,0x09);
lcm_dcs_write_seq_static(ctx,0xc5,0x00,0x09,0x09,0x09,0x09,0x00,0x09,0x09,0x09,0x09);
lcm_dcs_write_seq_static(ctx,0xc6,0xff,0xff);
lcm_dcs_write_seq_static(ctx,0xc7,0x5a);
lcm_dcs_write_seq_static(ctx,0xc9,0x7f,0x3f,0x2f,0x1f,0x0f,0x07,0x05,0x04,0x03,0x02,0x01,0x00);
lcm_dcs_write_seq_static(ctx,0xca,0xff,0x8f);
lcm_dcs_write_seq_static(ctx,0xcb,0x67,0x44,0x34,0x01,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xcc,0x5a,0xe9,0x1f,0xdc,0x8b,0x01,0x1b,0xc8,0xac,0x9c,0x00,0x89,0x70,0x52,0x00);
lcm_dcs_write_seq_static(ctx,0x59,0x0e);
lcm_dcs_write_seq_static(ctx,0xf0,0x59);
lcm_dcs_write_seq_static(ctx,0xb0,0x13,0x04,0xc4,0x00,0x3c,0x3b);
lcm_dcs_write_seq_static(ctx,0xb2,0x07,0x73,0xda,0xfe,0x13,0xe8,0x13,0xf9,0x52,0x3e,0x90,0x2f,0xb0,0x02,0x1b,0x29);
lcm_dcs_write_seq_static(ctx,0xb3,0xf5,0x12,0x1e,0x29,0xf1,0x81,0x02,0x1e,0x13,0x17,0x1a,0x1d,0x1e,0x0f,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xc2,0xf1,0xe1,0xd1,0xa1,0x71,0x31,0x02,0x1e,0x28,0xf1,0x91,0x12,0x2e,0xf5,0x91,0x02);
lcm_dcs_write_seq_static(ctx,0xc3,0x2b,0xb0,0x03,0xf9,0x52,0x3e,0x91,0x3f,0x81,0x7e,0x30,0xa7,0xed,0x0f,0x00,0x00);
	

#if 0
//////////bist/////////                           
	lcm_dcs_write_seq_static(ctx,0xF0,0x50);
	lcm_dcs_write_seq_static(ctx,0xe0,0xaa);
#endif
//	lcm_dcs_write_seq_static(ctx,0x53,0x00);

	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(140);
	lcm_dcs_write_seq_static(ctx, 0x29);
	msleep(20);
 
//-------------  Display Initial Setting end -------------------------

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
//	int ret;
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s:ch13726a +\n", __func__);
	if (!ctx->prepared)
		return 0;
	
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(140);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x4f,0x01);//deep sleep
	if (!IS_ERR_OR_NULL(ctx->disp_pwm_gpio)) {
		gpiod_set_value(ctx->disp_pwm_gpio, 0);
	}
//	msleep(140);
	if (!IS_ERR_OR_NULL(ctx->avdd_en_gpio)) {
		gpiod_set_value(ctx->avdd_en_gpio, 0);
	}
#if 0	//enter deep sleep(for vdd2.8 vio1.8 is always on,so enter deep sleep for power saving),rst should keep hight
	if (!IS_ERR_OR_NULL(ctx->reset_gpio)) {
		gpiod_set_value(ctx->reset_gpio, 0);
	}
#endif
#if 0
	if (!IS_ERR_OR_NULL(ctx->vdd2v8)) {
		ret = regulator_set_voltage(ctx->vdd2v8, 2800000, 2800000);
//		pr_info("%s:ch13726a  set vdd2v8 ret=%d \n", __func__,ret);
		ret =regulator_disable(ctx->vdd2v8);
//		pr_info("%s:ch13726a  disable vdd2v8 ret=%d \n", __func__,ret);
	}
#endif
#ifdef LCM_POWER_CTL
	ctx->iovdd = devm_regulator_get(ctx->dev, "iovdd");
	if (!IS_ERR_OR_NULL(ctx->iovdd)) {
		pr_info("%s:ch13726a  get iovdd ok \n", __func__);
		regulator_set_voltage(ctx->iovdd, 1800000, 1800000);
		regulator_disable(ctx->iovdd);
		devm_regulator_put(ctx->iovdd);
	}
	else
		pr_info("%s:ch13726a  get iovdd error \n", __func__);

	ctx->avdd = devm_regulator_get(ctx->dev, "avdd");
	if (!IS_ERR_OR_NULL(ctx->avdd)) {
		pr_info("%s:ch13726a  get avdd ok \n", __func__);
		regulator_set_voltage(ctx->avdd, 2800000, 2800000);
		regulator_disable(ctx->avdd);
		devm_regulator_put(ctx->avdd);
	}
	else
		pr_info("%s:ch13726a  get avdd error \n", __func__);
#endif	
	//udelay(1000);

	ctx->error = 0;
	ctx->prepared = false;

	pr_info("%s:ch13726a -\n", __func__);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s:ch13726a +\n", __func__);
	if (ctx->prepared)
		return 0;

#if 0
	if (!IS_ERR_OR_NULL(ctx->vdd2v8)) {
		ret = regulator_set_voltage(ctx->vdd2v8, 2800000, 2800000);
//		pr_info("%s:ch13726a  set vdd2v8 ret=%d \n", __func__,ret);
		ret =regulator_enable(ctx->vdd2v8);
//		pr_info("%s:ch13726a  enable vdd2v8 ret=%d \n", __func__,ret);
	}
#endif
#ifdef LCM_POWER_CTL
	ctx->iovdd = devm_regulator_get(ctx->dev, "iovdd");
	if (!IS_ERR_OR_NULL(ctx->iovdd)) {
		pr_info("%s:ch13726a  get iovdd ok \n", __func__);
		regulator_set_voltage(ctx->iovdd, 1800000, 1800000);
		regulator_enable(ctx->iovdd);
		devm_regulator_put(ctx->iovdd);
	}
	else
		pr_info("%s:ch13726a  get iovdd error \n", __func__);

	ctx->avdd = devm_regulator_get(ctx->dev, "avdd");
	if (!IS_ERR_OR_NULL(ctx->avdd)) {
		pr_info("%s:ch13726a  get avdd ok \n", __func__);
		regulator_set_voltage(ctx->avdd, 2800000, 2800000);
		regulator_enable(ctx->avdd);
		devm_regulator_put(ctx->avdd);
	}
	else
		pr_info("%s:ch13726a  get avdd error \n", __func__);
#endif

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	pr_info("%s:ch13726a -\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (80)
#define HSA (80)
#define HBP (80)
#define VFP (90)
#define VSA (90)
#define VBP (90)
#define VAC (1240)
#define HAC (1080)

static struct drm_display_mode default_mode = {
	.clock = 119592,//84000,//164297,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

//	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
//	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

//	pr_info("%s:ch13726a : setbacklight(0x51),level=%d\n", __func__,level);
	return 0;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 372,//358,//218,//264,//530,
	//.vfp_low_power = 840,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *	   become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *	  display the first valid frame after starting to receive
	 *	  video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *	   turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		 to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 64;
	connector->display_info.height_mm = 129;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;

	pr_info("%s:ch13726a  +++111 \n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s:ch13726a + skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	pr_info("%s:ch13726a  +++222 \n", __func__);

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;

	pr_info("%s:ch13726a  +++333 \n", __func__);
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	pr_info("%s:ch13726a  +++444 \n", __func__);
	
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
//	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->avdd_en_gpio = devm_gpiod_get(dev, "avdd-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->avdd_en_gpio)) {
		dev_err(dev, "cannot get avdd-en-gpios %ld\n",
			PTR_ERR(ctx->avdd_en_gpio));
		return PTR_ERR(ctx->avdd_en_gpio);
	}
//	devm_gpiod_put(dev, ctx->avdd_en_gpio);
#if 0	
	ctx->ocp2133_fd_gpio = devm_gpiod_get(dev, "ocp2133-fd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->ocp2133_fd_gpio)) {
		dev_err(dev, "cannot get ocp2133-fd-gpios %ld\n",
			PTR_ERR(ctx->ocp2133_fd_gpio));
		return PTR_ERR(ctx->ocp2133_fd_gpio);
	}
	devm_gpiod_put(dev, ctx->ocp2133_fd_gpio);
#endif
	ctx->disp_pwm_gpio = devm_gpiod_get(dev, "disp-pwm", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->disp_pwm_gpio)) {
		dev_err(dev, "cannot get disp-pwm-gpios %ld\n",
			PTR_ERR(ctx->disp_pwm_gpio));
		return PTR_ERR(ctx->disp_pwm_gpio);
	}
//	devm_gpiod_put(dev, ctx->disp_pwm_gpio);

#if 0
	ctx->vdd2v8 = devm_regulator_get(dev, "vdd2v8");
	if (IS_ERR_OR_NULL(ctx->vdd2v8)) {
		ret = PTR_ERR(ctx->vdd2v8);
		dev_err(dev,"Failed to get regulator vdd2v8:%d", ret);
		ctx->vdd2v8 = NULL;
		return ret;
	}else{
		ret = regulator_set_voltage(ctx->vdd2v8, 2800000, 2800000);
//		pr_info("%s:ch13726a  set vdd2v8 ret=%d \n", __func__,ret);
		regulator_enable(ctx->vdd2v8);
//		pr_info("%s:ch13726a  enable vdd2v8 ret=%d \n", __func__,ret);
	}
#endif
#if 0//def LCM_POWER_CTL
	ctx->iovdd = devm_regulator_get(dev, "iovdd");
	if (IS_ERR(ctx->iovdd)) {
		pr_info("%s:ch13726a  set iovdd error \n", __func__);
		ret = PTR_ERR(ctx->iovdd);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request iovdd regulator: %d\n", ret);
		return ret;
	}
	regulator_set_voltage(ctx->iovdd, 1800000, 1800000);
	regulator_enable(ctx->iovdd);
	
	ctx->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(ctx->avdd)) {
		pr_info("%s:ch13726a  set avdd error \n", __func__);
		ret = PTR_ERR(ctx->avdd);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request avdd regulator: %d\n", ret);
		
		return ret;
	}
	regulator_set_voltage(ctx->avdd, 2800000, 2800000);
	regulator_enable(ctx->avdd);
	
#endif

	ctx->prepared = true;
	ctx->enabled = true;

	pr_info("%s:ch13726a  +++555 \n", __func__);
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	pr_info("%s:ch13726a  +++666 \n", __func__);
	drm_panel_add(&ctx->panel);

	pr_info("%s:ch13726a  +++777 \n", __func__);
	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	pr_info("%s:ch13726a  +++888 \n", __func__);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	pr_info("%s:ch13726a -\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "ch13726a,boe,mipi4,hdp,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-ch13726a-boe-mipi4-hdp-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("maliejun <maliejun@agenewtech.com>");
MODULE_DESCRIPTION("hongzan jd9365d VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");