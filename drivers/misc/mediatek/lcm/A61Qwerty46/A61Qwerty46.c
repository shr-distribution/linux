#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#include <linux/gpio.h>
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

/* AGold hack which calls mtk_gpio_set on pctl->chip in pinctrl-mtk-common */
extern void agold_gpio_set(unsigned offset, int value);

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
        lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)

/* static unsigned char lcd_id_pins_value = 0xFF; */
//static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									1
#define FRAME_WIDTH                                     (1440)
#define FRAME_HEIGHT                                    (1440)

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[30];
};

static struct LCM_setting_table init_setting[] = {
    {0x53, 1, {0x2c}},
    {0x2a, 4, {0x00, 0x00, 0x05, 0x9f}},
    {0x2b, 4, {0x00, 0x00, 0x05, 0x9f}},
    {0x35, 1, {}},
    {0x36, 1, {}},
    {0x3a, 1, {0x77}},
    {0x51, 1, {0xff}},
    {0x44, 2, {0x03, 0xe8}},
    {0x55, 1, {0x01}},
    {0x5e, 1, {}},
    {0x29, 1, {}},
    {REGFLAG_DELAY, 20, {}},
    {0x11, 1, {}},
    {REGFLAG_DELAY, 20, {}},
    {0x2a, 4, {0x00, 0x00, 0x04, 0x38}},
    {0x2b, 4, {0x00, 0x00, 0x05, 0x9f}},
    {0x51, 1, {0xff}},
    {REGFLAG_END_OF_TABLE, 0, {}}
};

static int agold_power_setting[] = {
	0, 1, 175, -1, -1, -1
};

static struct regulator *lcm_ldo;
static int regulator_inited;

int lcd_ldo_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	lcm_ldo = regulator_get(NULL, "mt6370_ldo");
	if (IS_ERR(lcm_ldo)) { /* handle return value */
		ret = PTR_ERR(lcm_ldo);
		pr_err("failed to get mt6370_ldo LDO, %d\n", ret);
		return ret;
	}

	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_ldo);
	pr_warn("lcm LDO voltage = %d in LK stage\n", ret);

	regulator_inited = 1;
	return ret; /* must be 0 */
}

int lcd_ldo_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcd_ldo_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(lcm_ldo, 2800000, 2800000);
	if (ret < 0)
		pr_err("set voltage lcm_ldo fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(lcm_ldo);
	if (ret < 0)
		pr_err("enable regulator lcm_ldo fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

int lcd_ldo_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcd_ldo_regulator_init();

	ret = regulator_disable(lcm_ldo);
	if (ret < 0)
		pr_err("disable regulator lcm_ldo fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
	params->physical_width = 81;
	params->physical_height = 81;
    // enable tearing-free
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.io_driving_current = 2;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    //params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577

	params->dsi.vertical_sync_active				= 4;	//10
	params->dsi.vertical_backporch = 2;
	params->dsi.vertical_frontporch = 10;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 25;
	params->dsi.horizontal_frontporch				= 25;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK=430;
	params->dsi.ssc_disable = 1;
}

static void lcm_power_on(void)
{
	LCM_LOGI("lcm_power_on start\n");
	if (!agold_power_setting[0])
		agold_gpio_set(agold_power_setting[2], 1);

	if (agold_power_setting[1] == 1) {
		lcd_ldo_enable();
	} else if (!agold_power_setting[1]) {
		agold_gpio_set(agold_power_setting[3], 1u);
	}

	if (agold_power_setting[4] != -1 && agold_power_setting[5] != -1)
		display_bias_enable();

	LCM_LOGI("lcm_power_on end\n");
}

static void lcm_init_power(void)
{
	LCM_LOGD("lcm_init_power enter\n");
	lcm_power_on();
}

static void lcm_suspend_power(void)
{
	LCM_LOGD("lcm_suspend_power enter\n");
}

static void lcm_resume_power(void)
{
	LCM_LOGD("lcm_resume_power enter\n");
	lcm_power_on();
}

static void lcm_init(void)
{
	LCM_LOGD("[Kernel]enter lcm_init\n");
	MDELAY(1);
	agold_gpio_set(157, 1);
	MDELAY(5);
	agold_gpio_set(153, 1);
	MDELAY(5);
	agold_gpio_set(18, 1);
	MDELAY(5);
	agold_gpio_set(18, 0);
	MDELAY(5);
	agold_gpio_set(18, 1);
	MDELAY(5);
	agold_gpio_set(45, 1);
	MDELAY(5);
	push_table(0,init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int data;

	LCM_LOGD("lcm_suspend enter\n");

	data = 0x280500;
	dsi_set_cmdq(&data, 1, 1);
	data = 0x100500;
	dsi_set_cmdq(&data, 1, 1);
	MDELAY(120);
	agold_gpio_set(45, 0);
	SET_RESET_PIN(0);
	MDELAY(10);
	agold_gpio_set(157, 0);
	MDELAY(10);

	LCM_LOGD("lcm_suspend end\n");
}

static void lcm_resume(void)
{
	LCM_LOGD("enter lcm_resume\n");
	lcm_init();
	LCM_LOGD("lcm_resume end\n");
}

static void lcm_update(unsigned int x, unsigned int y,
		       unsigned int width, unsigned int height)
{
#if (LCM_DSI_CMD_MODE)
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
#endif
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

struct LCM_DRIVER A61Qwerty46_lcm_drv =
{
    .name			= "A61Qwerty46",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.resume         = lcm_resume,
	.suspend        = lcm_suspend,
	.update		= lcm_update,
};
