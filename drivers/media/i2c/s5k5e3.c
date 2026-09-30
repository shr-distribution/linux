// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5K5E3YX 5 Mpx camera sensor
 *
 * The sensor exposes the standard SMIA/MIPI register layout - mode select at
 * 0x0100, coarse exposure at 0x0202, analogue gain at 0x0204, frame and line
 * length at 0x0340 and 0x0342, mirroring at 0x0101 - so most of it looks like
 * any other sensor of that generation. The initialisation table and the mode
 * geometry below come from Samsung's vendor driver as shipped in the msm8916
 * and pxa19xx trees (drivers/media/i2c/b52_camera/s5k5e3.[ch]), which is the
 * only published description of the sensor's private 0x3xxx registers.
 *
 * Clocking, derived from the init table and confirmed against the vendor's own
 * comment that the pixel clock is ~179 MHz: EXTCLK is 26 MHz (0x0136), the
 * pre-PLL divider is 6 and the PLL multiplier 204 (0x0305, 0x0306), giving
 * 884 Mbps per lane (0x0820). With two lanes (0x0114) and 10 bits per pixel
 * that is a 176.8 Mpx/s pixel rate and a 442 MHz link frequency.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define S5K5E3_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K5E3_CHIP_ID			0x5e30

#define S5K5E3_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K5E3_MODE_STANDBY		0x00
#define S5K5E3_MODE_STREAMING		0x01

#define S5K5E3_REG_IMAGE_ORIENTATION	CCI_REG8(0x0101)
#define S5K5E3_HFLIP			BIT(0)
#define S5K5E3_VFLIP			BIT(1)

#define S5K5E3_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K5E3_EXPOSURE_MIN		4
#define S5K5E3_EXPOSURE_MARGIN		4
#define S5K5E3_EXPOSURE_STEP		1
#define S5K5E3_EXPOSURE_DEFAULT		0x04e2

#define S5K5E3_REG_ANALOGUE_GAIN	CCI_REG16(0x0204)
#define S5K5E3_ANA_GAIN_MIN		32
#define S5K5E3_ANA_GAIN_MAX		512
#define S5K5E3_ANA_GAIN_STEP		1
#define S5K5E3_ANA_GAIN_DEFAULT		0x20

#define S5K5E3_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define S5K5E3_REG_LINE_LENGTH		CCI_REG16(0x0342)
#define S5K5E3_FRAME_LENGTH_MAX		0xffff
#define S5K5E3_LINE_LENGTH_MAX		0xffff

/* Fixed by the PLL configuration in the init table. */
#define S5K5E3_XCLK_FREQ		26000000
#define S5K5E3_PIXEL_RATE		176800000
#define S5K5E3_LINK_FREQ		442000000

#define S5K5E3_NATIVE_WIDTH		2576
#define S5K5E3_NATIVE_HEIGHT		1932

struct s5k5e3_mode {
	unsigned int width;
	unsigned int height;
	unsigned int line_length;
	unsigned int frame_length;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct cci_reg_sequence s5k5e3_init_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x3000), 0x04 },
	{ CCI_REG8(0x3002), 0x03 },
	{ CCI_REG8(0x3003), 0x04 },
	{ CCI_REG8(0x3004), 0x05 },
	{ CCI_REG8(0x3005), 0x00 },
	{ CCI_REG8(0x3006), 0x10 },
	{ CCI_REG8(0x3007), 0x0a },
	{ CCI_REG8(0x3008), 0x55 },
	{ CCI_REG8(0x3039), 0x00 },
	{ CCI_REG8(0x303a), 0x00 },
	{ CCI_REG8(0x303b), 0x00 },
	{ CCI_REG8(0x3009), 0x05 },
	{ CCI_REG8(0x300a), 0x55 },
	{ CCI_REG8(0x300b), 0x38 },
	{ CCI_REG8(0x300c), 0x10 },
	{ CCI_REG8(0x3012), 0x14 },
	{ CCI_REG8(0x3013), 0x00 },
	{ CCI_REG8(0x3014), 0x22 },
	{ CCI_REG8(0x300e), 0x79 },
	{ CCI_REG8(0x3010), 0x68 },
	{ CCI_REG8(0x3019), 0x03 },
	{ CCI_REG8(0x301a), 0x00 },
	{ CCI_REG8(0x301b), 0x06 },
	{ CCI_REG8(0x301c), 0x00 },
	{ CCI_REG8(0x301d), 0x22 },
	{ CCI_REG8(0x301e), 0x00 },
	{ CCI_REG8(0x301f), 0x10 },
	{ CCI_REG8(0x3020), 0x00 },
	{ CCI_REG8(0x3021), 0x00 },
	{ CCI_REG8(0x3022), 0x0a },
	{ CCI_REG8(0x3023), 0x1e },
	{ CCI_REG8(0x3024), 0x00 },
	{ CCI_REG8(0x3025), 0x00 },
	{ CCI_REG8(0x3026), 0x00 },
	{ CCI_REG8(0x3027), 0x00 },
	{ CCI_REG8(0x3028), 0x1a },
	{ CCI_REG8(0x3015), 0x00 },
	{ CCI_REG8(0x3016), 0x84 },
	{ CCI_REG8(0x3017), 0x00 },
	{ CCI_REG8(0x3018), 0xa0 },
	{ CCI_REG8(0x302b), 0x10 },
	{ CCI_REG8(0x302c), 0x0a },
	{ CCI_REG8(0x302d), 0x06 },
	{ CCI_REG8(0x302e), 0x05 },
	{ CCI_REG8(0x302f), 0x0e },
	{ CCI_REG8(0x3030), 0x2f },
	{ CCI_REG8(0x3031), 0x08 },
	{ CCI_REG8(0x3032), 0x05 },
	{ CCI_REG8(0x3033), 0x09 },
	{ CCI_REG8(0x3034), 0x05 },
	{ CCI_REG8(0x3035), 0x00 },
	{ CCI_REG8(0x3036), 0x00 },
	{ CCI_REG8(0x3037), 0x00 },
	{ CCI_REG8(0x3038), 0x00 },
	{ CCI_REG8(0x3088), 0x06 },
	{ CCI_REG8(0x308a), 0x08 },
	{ CCI_REG8(0x308c), 0x05 },
	{ CCI_REG8(0x308e), 0x07 },
	{ CCI_REG8(0x3090), 0x06 },
	{ CCI_REG8(0x3092), 0x08 },
	{ CCI_REG8(0x3094), 0x05 },
	{ CCI_REG8(0x3096), 0x21 },
	{ CCI_REG8(0x3055), 0x9e },
	{ CCI_REG8(0x3099), 0x06 },
	{ CCI_REG8(0x3070), 0x10 },
	{ CCI_REG8(0x3085), 0x31 },
	{ CCI_REG8(0x3086), 0x01 },
	{ CCI_REG8(0x3064), 0x00 },
	{ CCI_REG8(0x3062), 0x08 },
	{ CCI_REG8(0x3061), 0x15 },
	{ CCI_REG8(0x307b), 0x20 },
	{ CCI_REG8(0x3068), 0x01 },
	{ CCI_REG8(0x3074), 0x00 },
	{ CCI_REG8(0x307d), 0x05 },
	{ CCI_REG8(0x3045), 0x01 },
	{ CCI_REG8(0x3046), 0x05 },
	{ CCI_REG8(0x3047), 0x78 },
	{ CCI_REG8(0x307f), 0xb1 },
	{ CCI_REG8(0x3098), 0x01 },
	{ CCI_REG8(0x305c), 0xf6 },
	{ CCI_REG8(0x3063), 0x2f },
	{ CCI_REG8(0x3400), 0x01 },
	{ CCI_REG8(0x3235), 0x49 },
	{ CCI_REG8(0x3233), 0x00 },
	{ CCI_REG8(0x3234), 0x00 },
	{ CCI_REG8(0x3300), 0x0c },
	{ CCI_REG8(0x3320), 0x02 },
	{ CCI_REG8(0x3203), 0x45 },
	{ CCI_REG8(0x3205), 0x4d },
	{ CCI_REG8(0x320b), 0x40 },
	{ CCI_REG8(0x320c), 0x06 },
	{ CCI_REG8(0x320d), 0xc0 },
	{ CCI_REG8(0x3244), 0x00 },
	{ CCI_REG8(0x3245), 0x00 },
	{ CCI_REG8(0x3246), 0x01 },
	{ CCI_REG8(0x3247), 0x00 },
	{ CCI_REG8(0x3268), 0x88 },
	{ CCI_REG8(0x3269), 0x01 },
	{ CCI_REG8(0x0136), 0x1a },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xcc },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x74 },
	{ CCI_REG8(0x3c1c), 0x58 },
	{ CCI_REG8(0x0114), 0x01 },
	{ CCI_REG8(0x0340), 0x07 },
	{ CCI_REG8(0x0341), 0xce },
	{ CCI_REG8(0x0342), 0x0b },
	{ CCI_REG8(0x0343), 0x86 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x02 },
	{ CCI_REG8(0x0348), 0x0a },
	{ CCI_REG8(0x0349), 0x0f },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x034b), 0x8d },
	{ CCI_REG8(0x034c), 0x0a },
	{ CCI_REG8(0x034d), 0x10 },
	{ CCI_REG8(0x034e), 0x07 },
	{ CCI_REG8(0x034f), 0x8c },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x3941), 0x08 },
	{ CCI_REG8(0x3942), 0xa1 },
	{ CCI_REG8(0x3924), 0x56 },
	{ CCI_REG8(0x3925), 0x48 },
	{ CCI_REG8(0x3c31), 0x70 },
	{ CCI_REG8(0x3c32), 0x2b },
	{ CCI_REG8(0x3c08), 0x7b },
	{ CCI_REG8(0x3c09), 0x62 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x20 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x00 },
	{ CCI_REG8(0x0200), 0x04 },
	{ CCI_REG8(0x0201), 0x98 },
};

static const struct cci_reg_sequence s5k5e3_mode_1280x960_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xe0 },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x80 },
	{ CCI_REG8(0x3c1c), 0x58 },
	{ CCI_REG8(0x0114), 0x01 },
	{ CCI_REG8(0x0340), 0x03 },
	{ CCI_REG8(0x0341), 0xf4 },
	{ CCI_REG8(0x0342), 0x0b },
	{ CCI_REG8(0x0343), 0x86 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0a },
	{ CCI_REG8(0x0349), 0x07 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x034b), 0x87 },
	{ CCI_REG8(0x034c), 0x05 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x03 },
	{ CCI_REG8(0x034f), 0xc0 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x20 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x00 },
	{ CCI_REG8(0x0200), 0x04 },
	{ CCI_REG8(0x0100), 0x01 },
};


static const struct s5k5e3_mode s5k5e3_modes[] = {
	{
		/* Full resolution, configured by the init table itself. */
		.width = S5K5E3_NATIVE_WIDTH,
		.height = S5K5E3_NATIVE_HEIGHT,
		.line_length = 2950,
		.frame_length = 1998,
		.regs = NULL,
		.num_regs = 0,
	}, {
		.width = 1280,
		.height = 960,
		.line_length = 2950,
		.frame_length = 1012,
		.regs = s5k5e3_mode_1280x960_regs,
		.num_regs = ARRAY_SIZE(s5k5e3_mode_1280x960_regs),
	},
};

/* Only the Bayer order the sensor actually produces. */
static const u32 s5k5e3_mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10;

static const char * const s5k5e3_supply_names[] = {
	"vana",		/* 2.8 V analogue */
	"vdig",		/* 1.2 V digital core */
	"vio",		/* 1.8 V interface */
};

struct s5k5e3 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k5e3_supply_names)];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

static inline struct s5k5e3 *to_s5k5e3(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5e3, sd);
}

static const struct s5k5e3_mode *s5k5e3_current_mode(struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5k5e3_modes); i++)
		if (s5k5e3_modes[i].width == fmt->width &&
		    s5k5e3_modes[i].height == fmt->height)
			return &s5k5e3_modes[i];

	return &s5k5e3_modes[0];
}

static int s5k5e3_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e3 *sensor =
		container_of(ctrl->handler, struct s5k5e3, ctrl_handler);
	struct v4l2_subdev_state *state;
	const struct s5k5e3_mode *mode;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	mode = s5k5e3_current_mode(state);

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max = mode->height + ctrl->val - S5K5E3_EXPOSURE_MARGIN;

		__v4l2_ctrl_modify_range(sensor->exposure, S5K5E3_EXPOSURE_MIN,
					 exposure_max, S5K5E3_EXPOSURE_STEP,
					 min(sensor->exposure->val, exposure_max));
	}

	/*
	 * Applying a control to a powered-down sensor is pointless: the values
	 * are pushed again from s5k5e3_enable_streams().
	 */
	if (!pm_runtime_get_if_in_use(sensor->sd.dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, S5K5E3_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, S5K5E3_REG_ANALOGUE_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, S5K5E3_REG_FRAME_LENGTH,
			  mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HBLANK:
		cci_write(sensor->regmap, S5K5E3_REG_LINE_LENGTH,
			  mode->width + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* Both mirror bits live in one register, so write them together. */
		cci_update_bits(sensor->regmap, S5K5E3_REG_IMAGE_ORIENTATION,
				S5K5E3_HFLIP | S5K5E3_VFLIP,
				(sensor->hflip->val ? S5K5E3_HFLIP : 0) |
				(sensor->vflip->val ? S5K5E3_VFLIP : 0),
				&ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->sd.dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k5e3_ctrl_ops = {
	.s_ctrl = s5k5e3_set_ctrl,
};

static int s5k5e3_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = s5k5e3_mbus_code;

	return 0;
}

static int s5k5e3_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k5e3_modes) ||
	    fse->code != s5k5e3_mbus_code)
		return -EINVAL;

	fse->min_width = fse->max_width = s5k5e3_modes[fse->index].width;
	fse->min_height = fse->max_height = s5k5e3_modes[fse->index].height;

	return 0;
}

static void s5k5e3_update_pad_format(const struct s5k5e3_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = s5k5e3_mbus_code;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k5e3_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k5e3 *sensor = to_s5k5e3(sd);
	const struct s5k5e3_mode *mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	int exposure_max;

	mode = v4l2_find_nearest_size(s5k5e3_modes, ARRAY_SIZE(s5k5e3_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k5e3_update_pad_format(mode, &fmt->format);

	format = v4l2_subdev_state_get_format(state, 0);
	*format = fmt->format;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = S5K5E3_NATIVE_WIDTH;
	crop->height = S5K5E3_NATIVE_HEIGHT;

	if (fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	/* The blanking limits and the exposure range follow the mode. */
	__v4l2_ctrl_modify_range(sensor->vblank, 4,
				 S5K5E3_FRAME_LENGTH_MAX - mode->height, 1,
				 mode->frame_length - mode->height);
	__v4l2_ctrl_s_ctrl(sensor->vblank, mode->frame_length - mode->height);

	__v4l2_ctrl_modify_range(sensor->hblank,
				 mode->line_length - mode->width,
				 S5K5E3_LINE_LENGTH_MAX - mode->width, 1,
				 mode->line_length - mode->width);

	exposure_max = mode->frame_length - S5K5E3_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(sensor->exposure, S5K5E3_EXPOSURE_MIN,
				 exposure_max, S5K5E3_EXPOSURE_STEP,
				 min(sensor->exposure->val, exposure_max));

	return 0;
}

static int s5k5e3_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K5E3_NATIVE_WIDTH;
		sel->r.height = S5K5E3_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5k5e3_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = S5K5E3_NATIVE_WIDTH,
			.height = S5K5E3_NATIVE_HEIGHT,
		},
	};

	return s5k5e3_set_pad_format(sd, state, &fmt);
}

static int s5k5e3_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k5e3 *sensor = to_s5k5e3(sd);
	const struct s5k5e3_mode *mode = s5k5e3_current_mode(state);
	int ret;

	ret = pm_runtime_resume_and_get(sd->dev);
	if (ret < 0)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, s5k5e3_init_regs,
				  ARRAY_SIZE(s5k5e3_init_regs), NULL);
	if (ret) {
		dev_err(sd->dev, "Failed to write the init sequence\n");
		goto err_rpm_put;
	}

	if (mode->num_regs) {
		ret = cci_multi_reg_write(sensor->regmap, mode->regs,
					  mode->num_regs, NULL);
		if (ret) {
			dev_err(sd->dev, "Failed to write the mode registers\n");
			goto err_rpm_put;
		}
	}

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(sensor->regmap, S5K5E3_REG_MODE_SELECT,
			S5K5E3_MODE_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	pm_runtime_mark_last_busy(sd->dev);
	pm_runtime_put_autosuspend(sd->dev);

	return ret;
}

static int s5k5e3_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k5e3 *sensor = to_s5k5e3(sd);
	int ret;

	ret = cci_write(sensor->regmap, S5K5E3_REG_MODE_SELECT,
			S5K5E3_MODE_STANDBY, NULL);

	pm_runtime_mark_last_busy(sd->dev);
	pm_runtime_put_autosuspend(sd->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops s5k5e3_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k5e3_pad_ops = {
	.enum_mbus_code = s5k5e3_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k5e3_set_pad_format,
	.get_selection = s5k5e3_get_selection,
	.enum_frame_size = s5k5e3_enum_frame_size,
	.enable_streams = s5k5e3_enable_streams,
	.disable_streams = s5k5e3_disable_streams,
};

static const struct v4l2_subdev_ops s5k5e3_subdev_ops = {
	.video = &s5k5e3_video_ops,
	.pad = &s5k5e3_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5e3_internal_ops = {
	.init_state = s5k5e3_init_state,
};

static int s5k5e3_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e3 *sensor = to_s5k5e3(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
		return ret;
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	/*
	 * The sensor needs the external clock running for a short while after
	 * reset is released before it will answer on i2c.
	 */
	fsleep(5000);

	return 0;
}

static int s5k5e3_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e3 *sensor = to_s5k5e3(sd);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	clk_disable_unprepare(sensor->xclk);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);

	return 0;
}

static int s5k5e3_identify(struct s5k5e3 *sensor)
{
	u64 id;
	int ret;

	ret = cci_read(sensor->regmap, S5K5E3_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(sensor->sd.dev, ret,
				     "Failed to read the chip id\n");

	if (id != S5K5E3_CHIP_ID)
		return dev_err_probe(sensor->sd.dev, -ENODEV,
				     "Unexpected chip id %04llx, expected %04x\n",
				     id, S5K5E3_CHIP_ID);

	return 0;
}

static int s5k5e3_check_bus_config(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -ENXIO, "No endpoint found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to parse the endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		ret = dev_err_probe(dev, -EINVAL,
				    "Only 2 data lanes are supported, got %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done;
	}

	if (bus_cfg.nr_of_link_frequencies != 1 ||
	    bus_cfg.link_frequencies[0] != S5K5E3_LINK_FREQ) {
		ret = dev_err_probe(dev, -EINVAL,
				    "Only a %u Hz link frequency is supported\n",
				    S5K5E3_LINK_FREQ);
		goto done;
	}

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k5e3_init_controls(struct s5k5e3 *sensor)
{
	static const s64 link_freq = S5K5E3_LINK_FREQ;
	const struct s5k5e3_mode *mode = &s5k5e3_modes[0];
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 9);
	if (ret)
		return ret;

	ctrl = v4l2_ctrl_new_int_menu(hdl, &s5k5e3_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, &link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops, V4L2_CID_PIXEL_RATE,
				 S5K5E3_PIXEL_RATE, S5K5E3_PIXEL_RATE, 1,
				 S5K5E3_PIXEL_RATE);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops,
					   V4L2_CID_VBLANK, 4,
					   S5K5E3_FRAME_LENGTH_MAX - mode->height,
					   1, mode->frame_length - mode->height);

	/*
	 * Horizontal blanking is writable rather than pinned to the mode's line
	 * length: it is half of what sets the frame duration, and libcamera
	 * warns that the driver needs fixing when it cannot be written.
	 */
	sensor->hblank = v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->line_length - mode->width,
					   S5K5E3_LINE_LENGTH_MAX - mode->width, 1,
					   mode->line_length - mode->width);

	sensor->exposure = v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K5E3_EXPOSURE_MIN,
					     mode->frame_length - S5K5E3_EXPOSURE_MARGIN,
					     S5K5E3_EXPOSURE_STEP,
					     S5K5E3_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K5E3_ANA_GAIN_MIN, S5K5E3_ANA_GAIN_MAX,
			  S5K5E3_ANA_GAIN_STEP, S5K5E3_ANA_GAIN_DEFAULT);

	/*
	 * Which way the sensor faces and how it is mounted. userspace reads
	 * these as V4L2 controls rather than out of the device tree itself, so
	 * a driver that does not publish them leaves libcamera reporting
	 * "Rotation control not available" and guessing 0 degrees.
	 */
	ret = v4l2_fwnode_device_parse(sensor->sd.dev, &props);
	if (ret) {
		dev_err(sensor->sd.dev, "Failed to parse the device properties\n");
		goto err_free;
	}

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &s5k5e3_ctrl_ops, &props);
	if (ret)
		goto err_free;

	sensor->hflip = v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(hdl, &s5k5e3_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	/*
	 * Mirroring changes the Bayer order the sensor emits, so the format has
	 * to be re-read after either flip changes.
	 */
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (hdl->error) {
		ret = hdl->error;
		dev_err(sensor->sd.dev, "Failed to add controls: %d\n", ret);
		goto err_free;
	}

	sensor->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);

	return ret;
}

static int s5k5e3_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k5e3 *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&sensor->sd, client, &s5k5e3_subdev_ops);
	sensor->sd.internal_ops = &s5k5e3_internal_ops;

	ret = s5k5e3_check_bus_config(dev);
	if (ret)
		return ret;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "Failed to initialise the register map\n");

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(dev, PTR_ERR(sensor->xclk),
				     "Failed to get the external clock\n");

	ret = clk_set_rate(sensor->xclk, S5K5E3_XCLK_FREQ);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set the external clock to %u Hz\n",
				     S5K5E3_XCLK_FREQ);

	for (i = 0; i < ARRAY_SIZE(s5k5e3_supply_names); i++)
		sensor->supplies[i].supply = s5k5e3_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get the regulators\n");

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "Failed to get the reset gpio\n");

	ret = s5k5e3_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to power the sensor on\n");

	ret = s5k5e3_identify(sensor);
	if (ret)
		goto err_power_off;

	ret = s5k5e3_init_controls(sensor);
	if (ret)
		goto err_power_off;

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_free_ctrls;

	sensor->sd.state_lock = sensor->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_media_cleanup;

	/*
	 * Hand the sensor over to runtime PM powered up, so the first stream
	 * start does not pay for a full power-up, then allow it to suspend.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register the subdev\n");
		goto err_pm_disable;
	}

	pm_runtime_idle(dev);

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sensor->sd);
err_media_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
err_power_off:
	s5k5e3_power_off(dev);

	return ret;
}

static void s5k5e3_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e3 *sensor = to_s5k5e3(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		s5k5e3_power_off(dev);
	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k5e3_pm_ops, s5k5e3_power_off,
				 s5k5e3_power_on, NULL);

static const struct of_device_id s5k5e3_of_match[] = {
	{ .compatible = "samsung,s5k5e3" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k5e3_of_match);

static struct i2c_driver s5k5e3_driver = {
	.driver = {
		.name = "s5k5e3",
		.of_match_table = s5k5e3_of_match,
		.pm = pm_ptr(&s5k5e3_pm_ops),
	},
	.probe = s5k5e3_probe,
	.remove = s5k5e3_remove,
};
module_i2c_driver(s5k5e3_driver);

MODULE_DESCRIPTION("Samsung S5K5E3YX camera sensor driver");
MODULE_LICENSE("GPL");
