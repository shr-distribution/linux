// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2019, Michael Srba

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct s6e88a0_ams452ef01 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	struct backlight_device *bl_dev;
	bool bl_blanked;
	int bl_aor_idx;
	int bl_want_idx;
	bool bl_on;
	struct mutex bl_lock;
	struct delayed_work bl_work;
};

static inline struct
s6e88a0_ams452ef01 *to_s6e88a0_ams452ef01(struct drm_panel *panel)
{
	return container_of(panel, struct s6e88a0_ams452ef01, panel);
}

/*
 * The fixed gamma operating point. The enable sequence writes this, and so does
 * every brightness update: 0xf7 is the gamma/AOR update and the panel only
 * latches a new AOR when gamma is rewritten in the same transaction. Sending
 * AOR plus 0xf7 alone is accepted without error and simply does not take
 * effect until the next enable - which presents as brightness that only
 * changes after the screen has been off.
 */
static const u8 s6e88a0_ams452ef01_gamma[] = {
	0xca,
	0x01, 0x00, 0x01, 0x00, 0x01, 0x00,	/* V255 RR,GG,BB */
	0x80, 0x80, 0x80,			/* V203 R,G,B */
	0x80, 0x80, 0x80,			/* V151 R,G,B */
	0x80, 0x80, 0x80,			/* V87  R,G,B */
	0x80, 0x80, 0x80,			/* V51  R,G,B */
	0x80, 0x80, 0x80,			/* V35  R,G,B */
	0x80, 0x80, 0x80,			/* V23  R,G,B */
	0x80, 0x80, 0x80,			/* V11  R,G,B */
	0x6b, 0x68, 0x71,			/* V3   R,G,B */
	0x00, 0x00, 0x00,			/* V1   R,G,B */
};

/*
 * AOR (Amoled Off Ratio) per brightness step, taken verbatim from Samsung's
 * samsung,aid_tx_cmds_revA in
 * drivers/video/msm/mdss/samsung/S6E88A0_AMS452EF01/
 *     dsi_panel_S6E88A0_AMS452EF01_qhd_octa_video.dtsi
 *
 * AOR is the duty ratio the panel blanks itself for, so it is what actually
 * dims an AMOLED of this generation - the vendor varies it across 41 steps
 * spanning roughly 5 to 360 cd/m2. The last entry, 0x000a, is the value this
 * driver used to write unconditionally, i.e. the brightest step.
 *
 * Gamma is NOT varied with it. Samsung generates gamma per step by "smart
 * dimming" from calibration data read out of each individual panel
 * (ss_dsi_smart_dimming_S6E88A0_AMS452EF01.c), which cannot be reduced to a
 * static table the way AOR can. Holding gamma at the fixed operating point the
 * enable sequence sets costs some tone accuracy at the dim end but keeps every
 * value written here a vendor one.
 */
/*
 * The vendor's brightness ladder, verbatim from the Samsung panel dtsi for this
 * revision: samsung,aid_map_table_revA and samsung,smart_acl_elvss_map_table_revA
 * select, per nominal cd/m2 level, one entry from samsung,aid_tx_cmds_revA (AOR,
 * register 0xb2) and one from samsung,smart_acl_elvss_tx_cmds_revA (ELVSS,
 * register 0xb6).
 *
 * Both are needed. Across the middle of the range - roughly 64 to 162 cd/m2 -
 * AOR sits still at index 33 and the vendor varies only ELVSS, so driving AOR
 * alone leaves that whole band flat no matter what the slider does.
 */
static const struct {
	u16 cd;		/* nominal luminance, for reference only */
	u8 aor_hi, aor_lo;
	u8 elvss;
} s6e88a0_ams452ef01_levels[] = {
	{   5, 0x03, 0xac, 0x17 },
	{   6, 0x03, 0xa5, 0x17 },
	{   7, 0x03, 0x9b, 0x17 },
	{   8, 0x03, 0x93, 0x17 },
	{   9, 0x03, 0x89, 0x17 },
	{  10, 0x03, 0x80, 0x17 },
	{  11, 0x03, 0x77, 0x17 },
	{  12, 0x03, 0x6f, 0x17 },
	{  13, 0x03, 0x65, 0x17 },
	{  14, 0x03, 0x5c, 0x17 },
	{  15, 0x03, 0x52, 0x17 },
	{  16, 0x03, 0x4a, 0x17 },
	{  17, 0x03, 0x41, 0x17 },
	{  19, 0x03, 0x2f, 0x17 },
	{  20, 0x03, 0x25, 0x17 },
	{  21, 0x03, 0x1d, 0x17 },
	{  22, 0x03, 0x12, 0x17 },
	{  24, 0x03, 0x01, 0x17 },
	{  25, 0x02, 0xf7, 0x17 },
	{  27, 0x02, 0xe5, 0x17 },
	{  29, 0x02, 0xd3, 0x17 },
	{  30, 0x02, 0xc8, 0x17 },
	{  32, 0x02, 0xb7, 0x17 },
	{  34, 0x02, 0xa2, 0x17 },
	{  37, 0x02, 0x87, 0x17 },
	{  39, 0x02, 0x74, 0x17 },
	{  41, 0x02, 0x5f, 0x17 },
	{  44, 0x02, 0x42, 0x17 },
	{  47, 0x02, 0x22, 0x17 },
	{  50, 0x02, 0x05, 0x17 },
	{  53, 0x01, 0xe5, 0x17 },
	{  56, 0x01, 0xc7, 0x17 },
	{  60, 0x01, 0x9e, 0x17 },
	{  64, 0x01, 0x78, 0x17 },
	{  68, 0x01, 0x78, 0x17 },
	{  72, 0x01, 0x78, 0x17 },
	{  77, 0x01, 0x78, 0x17 },
	{  82, 0x01, 0x78, 0x16 },
	{  87, 0x01, 0x78, 0x16 },
	{  93, 0x01, 0x78, 0x15 },
	{  98, 0x01, 0x78, 0x15 },
	{ 105, 0x01, 0x78, 0x15 },
	{ 111, 0x01, 0x78, 0x14 },
	{ 119, 0x01, 0x78, 0x14 },
	{ 126, 0x01, 0x78, 0x13 },
	{ 134, 0x01, 0x78, 0x13 },
	{ 143, 0x01, 0x78, 0x12 },
	{ 152, 0x01, 0x78, 0x12 },
	{ 162, 0x01, 0x78, 0x11 },
	{ 172, 0x01, 0x4f, 0x11 },
	{ 183, 0x01, 0x22, 0x10 },
	{ 195, 0x00, 0xef, 0x10 },
	{ 207, 0x00, 0xbd, 0x10 },
	{ 220, 0x00, 0x85, 0x10 },
	{ 234, 0x00, 0x49, 0x0f },
	{ 249, 0x00, 0x0a, 0x0f },
	{ 265, 0x00, 0x0a, 0x0f },
	{ 282, 0x00, 0x0a, 0x0e },
	{ 300, 0x00, 0x0a, 0x0d },
	{ 316, 0x00, 0x0a, 0x0c },
	{ 333, 0x00, 0x0a, 0x0c },
	{ 360, 0x00, 0x0a, 0x0b },
};

/*
 * Brightness and blanking.
 *
 * Without a backlight device nothing can blank or unblank the screen at all:
 * userspace driving /sys/class/backlight has nothing to write to, so the
 * display turns off on an idle timeout and never comes back. Blanking goes
 * through the panel's own DCS display on/off, which this driver already issues
 * on enable and disable.
 *
 * Brightness varies AOR only - see the table above for why gamma is left at the
 * fixed operating point the enable sequence establishes.
 */
/*
 * Write one AOR step out to the panel. Caller holds bl_lock.
 *
 * The DSI is already in low-power mode: s6e88a0_ams452ef01_on() sets
 * MIPI_DSI_MODE_LPM and only _off() clears it, so there is nothing to do about
 * the transfer mode here.
 */
static int s6e88a0_ams452ef01_write_aor(struct s6e88a0_ams452ef01 *ctx, int idx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u8 aor[] = { 0xb2, 0x40, 0x0a, 0x17, 0x00, 0x0a };
	u8 elvss[] = { 0xb6, 0x2c, 0x0b };

	aor[4] = s6e88a0_ams452ef01_levels[idx].aor_hi;
	aor[5] = s6e88a0_ams452ef01_levels[idx].aor_lo;
	elvss[2] = s6e88a0_ams452ef01_levels[idx].elvss;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a); /* level 2 key on */
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, aor, sizeof(aor));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e88a0_ams452ef01_gamma,
					sizeof(s6e88a0_ams452ef01_gamma));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, elvss, sizeof(elvss));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x03);	  /* gamma/aor update */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5); /* level 2 key off */

	if (!dsi_ctx.accum_err)
		ctx->bl_aor_idx = idx;

	return dsi_ctx.accum_err;
}

/*
 * Applying an AOR step means retransmitting gamma alongside it, and that whole
 * transaction lands in the middle of a running video stream. One of them is a
 * brief visible glitch; the stream of them a slider drag produces is constant
 * flicker and tearing. Coalesce instead: update_status only records the wanted
 * step and rearms this work, so a drag collapses into a single write once the
 * value stops moving.
 */
static void s6e88a0_ams452ef01_bl_work(struct work_struct *work)
{
	struct s6e88a0_ams452ef01 *ctx = container_of(work,
			struct s6e88a0_ams452ef01, bl_work.work);

	mutex_lock(&ctx->bl_lock);

	if (ctx->bl_on && !ctx->bl_blanked && ctx->bl_want_idx != ctx->bl_aor_idx)
		s6e88a0_ams452ef01_write_aor(ctx, ctx->bl_want_idx);

	mutex_unlock(&ctx->bl_lock);
}

/* Long enough to swallow a slider drag, short enough not to feel laggy. */
#define S6E88A0_BL_SETTLE_MS 80

static int s6e88a0_ams452ef01_set_brightness(struct backlight_device *bd)
{
	struct s6e88a0_ams452ef01 *ctx = bl_get_data(bd);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret = 0;
	bool blank;
	int idx;

	/*
	 * Brightness 0 means off, not "on but dark". With no graduated control
	 * the value carries no other meaning, and userspace that dims a panel
	 * to nothing expects it to go dark - LuneOS' display manager blanks by
	 * writing brightness, not bl_power, so honouring only backlight_is_blank()
	 * here left the screen lit through every blank request.
	 */
	blank = backlight_is_blank(bd) || bd->props.brightness == 0;

	/* Map the 1..max range onto the vendor's AOR steps. */
	idx = (bd->props.brightness - 1) * (ARRAY_SIZE(s6e88a0_ams452ef01_levels) - 1) /
	      max_t(int, bd->props.max_brightness - 1, 1);
	idx = clamp_t(int, idx, 0, (int)ARRAY_SIZE(s6e88a0_ams452ef01_levels) - 1);

	mutex_lock(&ctx->bl_lock);

	ctx->bl_want_idx = idx;

	/*
	 * Blanking is what the power key and the idle timeout go through, so it
	 * has to take effect now rather than after a settling delay.
	 */
	if (blank != ctx->bl_blanked) {
		if (blank)
			mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
		else
			mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

		ret = dsi_ctx.accum_err;
		if (ret)
			goto out;

		ctx->bl_blanked = blank;
	}

	if (!blank && ctx->bl_on && idx != ctx->bl_aor_idx)
		mod_delayed_work(system_dfl_wq, &ctx->bl_work,
				 msecs_to_jiffies(S6E88A0_BL_SETTLE_MS));

out:
	mutex_unlock(&ctx->bl_lock);

	return ret;
}

static const struct backlight_ops s6e88a0_ams452ef01_bl_ops = {
	.update_status = s6e88a0_ams452ef01_set_brightness,
};

static int s6e88a0_ams452ef01_backlight_register(struct s6e88a0_ams452ef01 *ctx)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};
	struct device *dev = &ctx->dsi->dev;

	ctx->bl_dev = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
						     &s6e88a0_ams452ef01_bl_ops,
						     &props);
	if (IS_ERR(ctx->bl_dev))
		return dev_err_probe(dev, PTR_ERR(ctx->bl_dev),
				     "error registering backlight device\n");

	return 0;
}

static void s6e88a0_ams452ef01_reset(struct s6e88a0_ams452ef01 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
}

static int s6e88a0_ams452ef01_on(struct s6e88a0_ams452ef01 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a); // enable LEVEL2 commands
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x4c); // set Pixel Clock Divider polarity

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	// set default brightness/gama
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e88a0_ams452ef01_gamma,
					sizeof(s6e88a0_ams452ef01_gamma));
	// set default Amoled Off Ratio
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x40, 0x0a, 0x17, 0x00, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0x2c, 0x0b); // set default elvss voltage
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x03); // gamma/aor update
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5); // disable LEVEL2 commands

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	/*
	 * The sequence above restored the hardcoded AOR, so whatever brightness
	 * was last programmed no longer reflects the panel. Force the next
	 * update_status to write it out again.
	 */
	ctx->bl_aor_idx = -1;
	ctx->bl_blanked = false;
	ctx->bl_on = true;

	return dsi_ctx.accum_err;
}

static void s6e88a0_ams452ef01_off(struct s6e88a0_ams452ef01 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi};

	/*
	 * Make sure no pending brightness update can run against a panel that
	 * is being powered down. bl_lock is not held across this because the
	 * worker takes it itself.
	 */
	mutex_lock(&ctx->bl_lock);
	ctx->bl_on = false;
	mutex_unlock(&ctx->bl_lock);
	cancel_delayed_work_sync(&ctx->bl_work);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 35);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
}

static int s6e88a0_ams452ef01_prepare(struct drm_panel *panel)
{
	struct s6e88a0_ams452ef01 *ctx = to_s6e88a0_ams452ef01(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	s6e88a0_ams452ef01_reset(ctx);

	ret = s6e88a0_ams452ef01_on(ctx);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies),
				       ctx->supplies);
		return ret;
	}

	return 0;
}

static int s6e88a0_ams452ef01_unprepare(struct drm_panel *panel)
{
	struct s6e88a0_ams452ef01 *ctx = to_s6e88a0_ams452ef01(panel);

	s6e88a0_ams452ef01_off(ctx);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

static const struct drm_display_mode s6e88a0_ams452ef01_mode = {
	.clock = (540 + 88 + 4 + 20) * (960 + 14 + 2 + 8) * 60 / 1000,
	.hdisplay = 540,
	.hsync_start = 540 + 88,
	.hsync_end = 540 + 88 + 4,
	.htotal = 540 + 88 + 4 + 20,
	.vdisplay = 960,
	.vsync_start = 960 + 14,
	.vsync_end = 960 + 14 + 2,
	.vtotal = 960 + 14 + 2 + 8,
	.width_mm = 56,
	.height_mm = 100,
};

static int s6e88a0_ams452ef01_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &s6e88a0_ams452ef01_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs s6e88a0_ams452ef01_panel_funcs = {
	.unprepare = s6e88a0_ams452ef01_unprepare,
	.prepare = s6e88a0_ams452ef01_prepare,
	.get_modes = s6e88a0_ams452ef01_get_modes,
};

static int s6e88a0_ams452ef01_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e88a0_ams452ef01 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e88a0_ams452ef01, panel,
				   &s6e88a0_ams452ef01_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supplies[0].supply = "vdd3";
	ctx->supplies[1].supply = "vci";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to get regulators: %d\n", ret);
		return ret;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "Failed to get reset-gpios: %d\n", ret);
		return ret;
	}

	ctx->dsi = dsi;
	ctx->bl_aor_idx = -1;	/* unknown until the first update_status */
	ctx->bl_want_idx = -1;
	ret = devm_mutex_init(dev, &ctx->bl_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&ctx->bl_work, s6e88a0_ams452ef01_bl_work);
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	ret = s6e88a0_ams452ef01_backlight_register(ctx);
	if (ret < 0) {
		mipi_dsi_detach(dsi);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void s6e88a0_ams452ef01_remove(struct mipi_dsi_device *dsi)
{
	struct s6e88a0_ams452ef01 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	cancel_delayed_work_sync(&ctx->bl_work);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e88a0_ams452ef01_of_match[] = {
	{ .compatible = "samsung,s6e88a0-ams452ef01" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, s6e88a0_ams452ef01_of_match);

static struct mipi_dsi_driver s6e88a0_ams452ef01_driver = {
	.probe = s6e88a0_ams452ef01_probe,
	.remove = s6e88a0_ams452ef01_remove,
	.driver = {
		.name = "panel-s6e88a0-ams452ef01",
		.of_match_table = s6e88a0_ams452ef01_of_match,
	},
};
module_mipi_dsi_driver(s6e88a0_ams452ef01_driver);

MODULE_AUTHOR("Michael Srba <Michael.Srba@seznam.cz>");
MODULE_DESCRIPTION("MIPI-DSI based Panel Driver for AMS452EF01 AMOLED LCD with a S6E88A0 controller");
MODULE_LICENSE("GPL v2");
