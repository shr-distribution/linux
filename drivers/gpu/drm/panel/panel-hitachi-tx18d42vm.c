// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hitachi TX18D42VM 7" 1024x768 LVDS panel
 *
 * Used by the HP TouchPad Go (Opal / SHORTLOIN). The panel is driven over
 * LVDS by the MSM8660/APQ8060 MDP4 LCDC encoder; a separate SPI control
 * channel (GSBI5 QUP SPI, 16-bit words, SPI mode 1) carries the power-on
 * register sequence that takes the timing controller out of standby and
 * issues DISPLAY ON. Without that SPI sequence the panel stays dark.
 *
 * Register sequence and timings derived from the legacy webOS driver
 * drivers/video/msm_pe/lcdc_hitachi_xga.c (board-shortloin). Two panel
 * revisions exist (WS1 / WS2); production Opal hardware reports WS2 and
 * that is the default here. Each SPI word is { addr<<2, data }, MSB first.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct tx18d42vm {
	struct drm_panel panel;
	struct spi_device *spi;
	struct regulator *power;	/* VDD_LVDS (pm8058_l10), optional */
	struct gpio_desc *enable_gpio;	/* optional panel/LVDS enable */
	bool ws1;			/* false = WS2 (production default) */
};

static inline struct tx18d42vm *to_tx18d42vm(struct drm_panel *panel)
{
	return container_of(panel, struct tx18d42vm, panel);
}

/* One control word: high byte = (addr << 2), low byte = data. */
struct tx18d42vm_cmd {
	u8 addr;
	u8 data;
};

/* WS1 first-silicon sequence (legacy init_settings_ws1). */
static const struct tx18d42vm_cmd tx18d42vm_init_ws1[] = {
	{ 0x00 << 2, 0x29 },	/* RESET */
	{ 0x00 << 2, 0x25 },	/* STANDBY */
	{ 0x02 << 2, 0x40 },	/* Normally Black */
	{ 0x01 << 2, 0x32 },	/* FRC / Dither */
	{ 0x03 << 2, 0x06 },	/* Gate-on sequence: reverse "Z" */
	{ 0x0e << 2, 0x5f },	/* Test mode (1) */
	{ 0x0f << 2, 0xa4 },	/* Test mode (2) */
	{ 0x0d << 2, 0x05 },	/* Enable SDRRS, enlarge OE width */
};

/* WS2 production sequence (legacy init_settings_ws2). */
static const struct tx18d42vm_cmd tx18d42vm_init_ws2[] = {
	{ 0x00 << 2, 0x29 },	/* RESET */
	{ 0x00 << 2, 0x25 },	/* STANDBY */
	{ 0x02 << 2, 0x40 },	/* Normally Black */
	{ 0x01 << 2, 0x32 },	/* FRC / Dither */
	{ 0x0e << 2, 0x5f },	/* Test mode (1) */
	{ 0x0f << 2, 0xa4 },	/* Test mode (2) */
	{ 0x0d << 2, 0x05 },	/* Enable SDRRS, enlarge OE width */
	{ 0x10 << 2, 0x41 },	/* Adopt 2 Line / 1 Dot */
};

#define TX18D42VM_DISPLAY_ON	0xad
#define TX18D42VM_STANDBY_ON	0xa5

static int tx18d42vm_write(struct tx18d42vm *ctx, u8 addr, u8 data)
{
	u8 buf[2] = { addr, data };

	/* bits_per_word = 16 => a single MSB-first word, addr then data */
	return spi_write(ctx->spi, buf, sizeof(buf));
}

/*
 * 1024x768 @ 60 Hz, pixel clock 69.3 MHz.
 * Legacy lcdc_hitachi_xga.c:
 *   h_back=90  h_front=216  h_pulse=70
 *   v_back=13  v_front=34   v_pulse=10
 */
static const struct drm_display_mode tx18d42vm_mode = {
	.clock = 69300,
	.hdisplay = 1024,
	.hsync_start = 1024 + 216,
	.hsync_end = 1024 + 216 + 70,
	.htotal = 1024 + 216 + 70 + 90,
	.vdisplay = 768,
	.vsync_start = 768 + 34,
	.vsync_end = 768 + 34 + 10,
	.vtotal = 768 + 34 + 10 + 13,
	.width_mm = 152,
	.height_mm = 114,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static int tx18d42vm_prepare(struct drm_panel *panel)
{
	struct tx18d42vm *ctx = to_tx18d42vm(panel);
	const struct tx18d42vm_cmd *seq;
	unsigned int i, n;
	int ret;

	if (ctx->power) {
		ret = regulator_enable(ctx->power);
		if (ret)
			return ret;
	}

	gpiod_set_value_cansleep(ctx->enable_gpio, 1);
	/* Let the LVDS rail / panel logic settle before SPI traffic. */
	msleep(20);

	if (ctx->ws1) {
		seq = tx18d42vm_init_ws1;
		n = ARRAY_SIZE(tx18d42vm_init_ws1);
	} else {
		seq = tx18d42vm_init_ws2;
		n = ARRAY_SIZE(tx18d42vm_init_ws2);
	}

	for (i = 0; i < n; i++) {
		ret = tx18d42vm_write(ctx, seq[i].addr, seq[i].data);
		if (ret) {
			dev_err(&ctx->spi->dev,
				"panel init write %u failed: %d\n", i, ret);
			goto err;
		}
		/* Wait for the controller to settle after RESET. */
		if (i == 0)
			usleep_range(4000, 5000);
	}

	/* Datasheet/legacy: settle before turning the display on. */
	msleep(100);

	ret = tx18d42vm_write(ctx, 0x00 << 2, TX18D42VM_DISPLAY_ON);
	if (ret)
		goto err;

	return 0;

err:
	gpiod_set_value_cansleep(ctx->enable_gpio, 0);
	if (ctx->power)
		regulator_disable(ctx->power);
	return ret;
}

static int tx18d42vm_unprepare(struct drm_panel *panel)
{
	struct tx18d42vm *ctx = to_tx18d42vm(panel);

	tx18d42vm_write(ctx, 0x00 << 2, TX18D42VM_STANDBY_ON);

	gpiod_set_value_cansleep(ctx->enable_gpio, 0);
	if (ctx->power)
		regulator_disable(ctx->power);

	return 0;
}

static int tx18d42vm_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &tx18d42vm_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bpc = 8;	/* 24bpp panel */

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs tx18d42vm_funcs = {
	.prepare = tx18d42vm_prepare,
	.unprepare = tx18d42vm_unprepare,
	.get_modes = tx18d42vm_get_modes,
};

static int tx18d42vm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tx18d42vm *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct tx18d42vm, panel,
				   &tx18d42vm_funcs,
				   DRM_MODE_CONNECTOR_LVDS);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->spi = spi;
	spi_set_drvdata(spi, ctx);

	/* WS1 vs WS2 is read from the "lcdxpanel" EDID i2c on the real
	 * hardware; production Opal is WS2. Allow a DT override for the rare
	 * first-silicon WS1 units.
	 */
	ctx->ws1 = of_property_read_bool(dev->of_node, "hitachi,panel-ws1");

	ctx->power = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(ctx->power)) {
		ret = PTR_ERR(ctx->power);
		if (ret == -EPROBE_DEFER)
			return ret;
		ctx->power = NULL;	/* rail may be always-on */
	}

	ctx->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio),
				     "failed to get enable GPIO\n");

	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_1;
	ret = spi_setup(spi);
	if (ret < 0)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	return 0;
}

static void tx18d42vm_remove(struct spi_device *spi)
{
	struct tx18d42vm *ctx = spi_get_drvdata(spi);

	drm_panel_remove(&ctx->panel);
	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
}

static const struct of_device_id tx18d42vm_of_match[] = {
	{ .compatible = "hitachi,tx18d42vm" },
	{ }
};
MODULE_DEVICE_TABLE(of, tx18d42vm_of_match);

static const struct spi_device_id tx18d42vm_ids[] = {
	{ "tx18d42vm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, tx18d42vm_ids);

static struct spi_driver tx18d42vm_driver = {
	.probe = tx18d42vm_probe,
	.remove = tx18d42vm_remove,
	.id_table = tx18d42vm_ids,
	.driver = {
		.name = "panel-hitachi-tx18d42vm",
		.of_match_table = tx18d42vm_of_match,
	},
};
module_spi_driver(tx18d42vm_driver);

MODULE_DESCRIPTION("Hitachi TX18D42VM LVDS panel driver (HP TouchPad Go / Opal)");
MODULE_LICENSE("GPL");
