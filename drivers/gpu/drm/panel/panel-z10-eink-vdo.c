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


#define FRAME_WIDTH    (800)
#define HFP (20)
#define HSA (16)
#define HBP (20)
#define HTOTAL         (FRAME_WIDTH + HFP + HSA + HBP)
#define FRAME_HEIGHT   (600)
#define VFP (54)
#define VSA (10)
#define VBP (10)
#define VTOTAL         (FRAME_HEIGHT + VFP + VSA + VBP)
#define FRAME_TOTAL    (HTOTAL * VTOTAL)
#define CLK_FPS60_X10 ((FRAME_TOTAL * 60) / 100)
#define CLK_FPS60_DEF		(((CLK_FPS60_X10 % 10) != 0) ?             \
			(CLK_FPS60_X10 / 10 + 1) : (CLK_FPS60_X10 / 10))
			
static struct drm_panel *g_panel = NULL;
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	bool prepared;
	bool enabled;

	//fpgv power set
	struct gpio_desc *v12_en;
	struct gpio_desc *v18_en;
	struct gpio_desc *v33_en;

	unsigned int gate_ic;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
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
	u8 buffer[3] = { 0 };
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if 0
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
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("%s\n", __func__);
	
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}

#if 0
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 11 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#endif	
	usleep_range(10 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 15 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

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
int fpga_power_off(void)
{
	struct lcm *ctx = panel_to_lcm(g_panel);
	printk("%s enter\n", __func__);
	if (!ctx->prepared)
		return 0;

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	 
	//set fpga/cpld power off
	ctx->v18_en = devm_gpiod_get(ctx->dev, "v18_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v18_en)) {
		dev_err(ctx->dev, "%s: cannot get v18_en %ld\n",
			__func__, PTR_ERR(ctx->v18_en));
		return PTR_ERR(ctx->v18_en);
	}
	ctx->v33_en = devm_gpiod_get(ctx->dev, "v33_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v33_en)) {
		dev_err(ctx->dev, "%s: cannot get v33_en %ld\n",
			__func__, PTR_ERR(ctx->v33_en));
		return PTR_ERR(ctx->v33_en);
	}
	ctx->v12_en = devm_gpiod_get(ctx->dev, "v12_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v12_en)) {
		dev_err(ctx->dev, "%s: cannot get v12_en %ld\n",
			__func__, PTR_ERR(ctx->v12_en));
		return PTR_ERR(ctx->v12_en);
	}	
	
	gpiod_set_value(ctx->v18_en, 0);
	devm_gpiod_put(ctx->dev, ctx->v18_en);

	gpiod_set_value(ctx->v33_en, 0);
	devm_gpiod_put(ctx->dev, ctx->v33_en);	

	gpiod_set_value(ctx->v12_en, 0);
	devm_gpiod_put(ctx->dev, ctx->v12_en);
	
	//ctx->error = 0;
	//ctx->prepared = false;
	printk("%s leave\n", __func__);
	return 0;
}
EXPORT_SYMBOL(fpga_power_off);
int fpga_power_on(void)
{
	struct lcm *ctx = panel_to_lcm(g_panel);
	printk("%s enter\n", __func__);
	if (!ctx->prepared)
		return 0;
	
	ctx->v12_en = devm_gpiod_get(ctx->dev, "v12_en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v12_en, 1);
	devm_gpiod_put(ctx->dev, ctx->v12_en);

	ctx->v33_en = devm_gpiod_get(ctx->dev, "v33_en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v33_en, 1);
	devm_gpiod_put(ctx->dev, ctx->v33_en);	
	
	ctx->v18_en = devm_gpiod_get(ctx->dev, "v18_en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v18_en, 1);
	devm_gpiod_put(ctx->dev, ctx->v18_en);
	printk("%s leave\n", __func__);
	return 0;
}
EXPORT_SYMBOL(fpga_power_on);
static int lcm_unprepare(struct drm_panel *panel)
{

	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;

	//lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	//msleep(50);
	//lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	//msleep(150);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	 
	//set fpga/cpld power off
	ctx->v18_en = devm_gpiod_get(ctx->dev, "v18_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v18_en)) {
		dev_err(ctx->dev, "%s: cannot get v18_en %ld\n",
			__func__, PTR_ERR(ctx->v18_en));
		return PTR_ERR(ctx->v18_en);
	}
	ctx->v33_en = devm_gpiod_get(ctx->dev, "v33_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v33_en)) {
		dev_err(ctx->dev, "%s: cannot get v33_en %ld\n",
			__func__, PTR_ERR(ctx->v33_en));
		return PTR_ERR(ctx->v33_en);
	}
	ctx->v12_en = devm_gpiod_get(ctx->dev, "v12_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v12_en)) {
		dev_err(ctx->dev, "%s: cannot get v12_en %ld\n",
			__func__, PTR_ERR(ctx->v12_en));
		return PTR_ERR(ctx->v12_en);
	}	
	
	gpiod_set_value(ctx->v18_en, 0);
	devm_gpiod_put(ctx->dev, ctx->v18_en);

	gpiod_set_value(ctx->v33_en, 0);
	devm_gpiod_put(ctx->dev, ctx->v33_en);	

	gpiod_set_value(ctx->v12_en, 0);
	devm_gpiod_put(ctx->dev, ctx->v12_en);
	

#if 0	 
	if (ctx->gate_ic == 0) {
		ctx->bias_neg =
			devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(2000, 2001);

		ctx->bias_pos =
			devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
	} else if (ctx->gate_ic == 4831) {
		_gate_ic_i2c_panel_bias_enable(0);
		_gate_ic_Power_off();
	}
#endif	
	
	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;
	
	ctx->v12_en = devm_gpiod_get(ctx->dev, "v12_en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v12_en, 1);
	devm_gpiod_put(ctx->dev, ctx->v12_en);

	ctx->v33_en = devm_gpiod_get(ctx->dev, "v33_en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v33_en, 1);
	devm_gpiod_put(ctx->dev, ctx->v33_en);	
	
	ctx->v18_en = devm_gpiod_get(ctx->dev, "v18_en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v18_en, 1);
	devm_gpiod_put(ctx->dev, ctx->v18_en);
	
#if 0	
	if (ctx->gate_ic == 0) {
		ctx->bias_pos =
			devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);

		usleep_range(2000, 2001);
		ctx->bias_neg =
			devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);
	} else if (ctx->gate_ic == 4831) {
		_gate_ic_Power_on();
		_gate_ic_i2c_panel_bias_enable(1);
	}
#endif	

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

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

static const struct drm_display_mode default_mode = {
	.clock = CLK_FPS60_DEF,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP,
	.vsync_end = FRAME_HEIGHT + VFP + VSA,
	.vtotal = FRAME_HEIGHT + VFP + VSA + VBP,
};


#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	if (level > 255)
		level = 255;
	pr_info("%s backlight = -%d\n", __func__, level);
	bl_tb0[1] = (u8)level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static struct mtk_panel_params ext_params = {
	//.pll_clk = 450,
	//.data_rate = 1190,
	//.vfp_low_power = 840,
	//.cust_esd_check = 1,
	//.esd_check_enable = 1,
	.rotate = 2,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	
	.lane_swap_en = 1,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,

};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
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

	connector->display_info.width_mm = 87; //70;
	connector->display_info.height_mm = 65; //152;

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
	unsigned int value;
	int ret;

	pr_info("%s+\n", __func__);

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
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;			//MIPI_DSI_FMT_RGB888  MIPI_DSI_FMT_RGB565
	
#if 0
	//command mode
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;
#else
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;
#endif		


	ret = of_property_read_u32(dev->of_node, "gate-ic", &value);
	if (ret < 0)
		value = 0;
	else
		ctx->gate_ic = value;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	
	//eink fpga/cpld power must be default on, otherwise would not display when enter into kernel from lk
	ctx->v12_en = devm_gpiod_get(dev, "v12_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v12_en)) {
		dev_err(dev, "moshaoxi cannot get v12_en-gpios %ld\n",
			 PTR_ERR(ctx->v12_en));
		return PTR_ERR(ctx->v12_en);
	}
	devm_gpiod_put(dev, ctx->v12_en);


	ctx->v18_en = devm_gpiod_get(dev, "v18_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v18_en)) {
		dev_err(dev, "moshaoxi cannot get v18_en-gpios %ld\n",
			 PTR_ERR(ctx->v18_en));
		return PTR_ERR(ctx->v18_en);
	}
	devm_gpiod_put(dev, ctx->v18_en);


	ctx->v33_en = devm_gpiod_get(dev, "v33_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v33_en)) {
		dev_err(dev, "moshaoxi cannot get v33_en-gpios %ld\n",
			 PTR_ERR(ctx->v33_en));
		return PTR_ERR(ctx->v33_en);
	}
	devm_gpiod_put(dev, ctx->v33_en);
	
#if 0
	if (ctx->gate_ic == 0) {
		ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
				__func__, PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
		devm_gpiod_put(dev, ctx->bias_pos);

		ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
				__func__, PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		devm_gpiod_put(dev, ctx->bias_neg);
	}
#endif	

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif

	pr_info("%s-\n", __func__);
	g_panel = &ctx->panel;
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
	{
	    .compatible = "z10,eink,vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-z10-eink-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

//module_mipi_dsi_driver(lcm_driver);
		
static int __init lcm_init(void)
{
	mipi_dsi_driver_register(&lcm_driver);
	return 0;
}
late_initcall(lcm_init);

static void __exit lcm_exit(void)
{
	mipi_dsi_driver_unregister(&lcm_driver);
}
module_exit(lcm_exit);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("Z10 EINK VDO Panel Driver");
MODULE_LICENSE("GPL v2");
