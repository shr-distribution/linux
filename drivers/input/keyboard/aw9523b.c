/*
 * aw9523b.c   aw9523b led and key
 *
 * Version: V1.11.0
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 *  Author: <shiqiang@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/leds.h>
#include <linux/fb.h>
#include <stddef.h>
#include "aw9523b.h"

#define HRTIMER_FRAME	20
#define AW9523B_DRIVER_VERSION	"V1.12.0"

//msx-add FB-suspend
#include "mtk_disp_notify.h"
#include <linux/notifier.h>
#include <linux/fb.h>
static int aw9523b_suspend(struct device *dev);  
static int aw9523b_resume(struct device *dev);
int suspend_flag = 0;
//static int aw9523b_fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data);
//end

/*register list */
#define P0_INPUT		0x00
#define P1_INPUT		0x01
#define P0_OUTPUT		0x02
#define P1_OUTPUT		0x03
#define P0_CONFIG		0x04
#define P1_CONFIG		0x05
#define P0_INT			0x06
#define P1_INT			0x07
#define ID_REG			0x10
#define CTL_REG			0x11
#define P0_LED_MODE		0x12
#define P1_LED_MODE		0x13
#define P1_0_DIM0		0x20
#define P1_1_DIM0		0x21
#define P1_2_DIM0		0x22
#define P1_3_DIM0		0x23
#define P0_0_DIM0		0x24
#define P0_1_DIM0		0x25
#define P0_2_DIM0		0x26
#define P0_3_DIM0		0x27
#define P0_4_DIM0		0x28
#define P0_5_DIM0		0x29
#define P0_6_DIM0		0x2A
#define P0_7_DIM0		0x2B
#define P1_4_DIM0		0x2C
#define P1_5_DIM0		0x2D
#define P1_6_DIM0		0x2E
#define P1_7_DIM0		0x2F
#define SW_RSTN			0x7F


static KEY_STATE key_map[] = {
/*	name		code				val */
	{"Space",	KEY_SPACE,			0},					//			/*0*/
	{"FN",		KEY_FN,				0},								/*1*/
	{"SYM",		KEY_SYM,			0},					//			/*2*/	
	{"EM",		KEY_EM,				0},								/*3*/
	{"LEFTSHIFT",	KEY_LEFTSHIFT,	0},					//			/*4*/
	{"RIGHTSHIFT",	KEY_RIGHTSHIFT,	0},					//			/*5*/
	{"ALT",		KEY_LEFTALT,		0},					//			/*6*/
	{"Z",		KEY_Z,				0},					//			/*7*/

	{"S",		KEY_S,				0},					//			/*8*/
	{"A",		KEY_A,				0},					//			/*9*/
	{"Q",		KEY_Q,				0},					//			/*10*/
	{"W",		KEY_W,				0},					//			/*11*/
	{"X",		KEY_X,				0},					//			/*12*/
	{"C",		KEY_C,				0},					//			/*13*/
	{"F",		KEY_F,				0},					//			/*14*/
	{"D",		KEY_D,				0},					//			/*15*/

	{"E",		KEY_E,				0},					//			/*16*/
	{"R",		KEY_R,				0},					//			/*17*/
	{"V",		KEY_V,				0},					//			/*18*/
	{"B",		KEY_B,				0},					//			/*19*/
	{"H",		KEY_H,				0},					//			/*20*/
	{"G",		KEY_G,				0},					//			/*21*/
	{"T",		KEY_T,				0},					//			/*22*/
	{"Y",		KEY_Y,				0},					//			/*23*/

	{"N",		KEY_N,				0},					//			/*24*/
	{"M",		KEY_M,				0},					//			/*25*/
	{"K",		KEY_K,				0},					//			/*26*/
	{"J",		KEY_J,				0},					//			/*27*/
	{"U",		KEY_U,				0},					//			/*28*/
	{"I",		KEY_I,				0},					//			/*29*/
	{"MIC",		KEY_MIC,		0},					//			/*30*/
	{"ENTER",	KEY_ENTER,			0},					//			/*31*/

	{"Backspace",	KEY_BACKSPACE,	0},					//			/*32*/
	{"L",		KEY_L,				0},					//			/*33*/
	{"O",		KEY_O,				0},					//			/*34*/
	{"P",		KEY_P,				0},					//			/*35*/
	{"V",		KEY_V,				0},
	{"B",		KEY_B,				0},
	{"F9",		KEY_F9,				0},
	{"F10",		KEY_F10,			0},

	{"N",		KEY_N,				0},
	{"M",		KEY_M,				0},
	{"Backspace",	KEY_BACKSPACE,	0},
	{"Down",	KEY_DOWN,			0},
	{"Up",		KEY_UP,				0},
	{"ENTER",	KEY_ENTER,			0},
	{"F11",		KEY_F11,			0},
	{"F12",		KEY_F12,			0},

	{"ALT",		KEY_ALTERASE,			0},
	{".",		KEY_DOT,			0},
	{"Left",	KEY_LEFT,			0},
	{"Right",	KEY_RIGHT,			0},
	{"Space",	KEY_SPACE,			0},
	{"Volup",	KEY_VOLUMEUP,			0},
	{"KEY_STOP",	KEY_STOP,			0},
	{"KEY_AGAIN",	KEY_AGAIN,			0},

	{"PROPS",	KEY_PROPS,			0},
	{"UNDO",	KEY_UNDO,			0},
	{"FRONT",	KEY_FRONT,			0},
	{"COPY",	KEY_COPY,			0},
	{"OPEN",	KEY_OPEN,			0},
	{"PASTE",	KEY_PASTE,			0},
	{"FIND",	KEY_FIND,			0},
	{"CUT",		KEY_CUT,			0},
};


//msx-add fb-suspend
#if 0
static struct notifier_block aw9523b_fb_notifier = {
	.notifier_call = aw9523b_fb_notifier_callback,
};

static int aw9523b_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)					
{
   	struct fb_event *evdata = data;
	int blank;

	AW_DEBUG("aw9523b_fb_notifier_callback enter\n");

	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)evdata->data;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		AW_DEBUG("%s(%d) SCREEN ON\n", __func__, __LINE__);
		break;
	case FB_BLANK_POWERDOWN:
		AW_DEBUG("%s(%d) SCREEN OFF\n", __func__, __LINE__);
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int aw9523b_drm_notifier_callback(struct notifier_block *nb,
		unsigned long value, void *v)
{
	struct aw9523b *aw9523v_dev =
			container_of(nb, struct aw9523b, disp_notifier);
	int *data = (int *)v;
  
	AW_DEBUG("%s enter: value:0x%x, *data:0x%x in\n", __func__, value, *data);
	if (aw9523v_dev && v) {
		if (value == MTK_DISP_EVENT_BLANK) {
			if (*data == MTK_DISP_BLANK_UNBLANK && suspend_flag) {
				AW_DEBUG("%s: resume in\n", __func__);
				aw9523b_resume(aw9523v_dev->dev);
				AW_DEBUG("%s: resume out\n", __func__);
			} else if (*data == MTK_DISP_BLANK_POWERDOWN) {
					AW_DEBUG("%s: suspend in\n", __func__);
					suspend_flag = 1;
					aw9523b_suspend(aw9523v_dev->dev);
					AW_DEBUG("%s: suspend out\n", __func__);
			}
		} else if (value == MTK_DISP_EARLY_EVENT_BLANK) {
				if (*data == MTK_DISP_BLANK_POWERDOWN) {
					AW_DEBUG("%s: ealy suspend in\n", __func__);
					suspend_flag = 1;
					//aw9523b_suspend(aw9523v_dev->dev);
					AW_DEBUG("%s: ealy suspend out\n", __func__);
			}
		} else {
			AW_DEBUG("%s: aw9523b IC can not suspend or resume\n", __func__);
				return -1;
		}
	}
	return 0;
}
//end


/*********************************************************
 *
 * awxxxx i2c write/read byte
 *
 ********************************************************/
static unsigned int i2c_write_byte(struct i2c_client *client,
					unsigned char reg_addr,
					unsigned char reg_data)
{
	int ret = 0;
	unsigned char wdbuf[2] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags 	= 0,
			.len 	= 2,
			.buf 	= wdbuf,
		},
	};

	if (client == NULL) {
		pr_err("msg %s i2c client is NULL\n", __func__);
		return -ENODEV;
	}
	wdbuf[0] = reg_addr;
	wdbuf[1] = reg_data;

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c write error: %d\n", __func__, ret);

	return ret;

}

static unsigned int i2c_read_byte(struct i2c_client *client,
				unsigned char reg_addr,
				unsigned char *val)
{
	int ret = 0;
	unsigned char rdbuf[2] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr 	= client->addr,
			.flags	 = 0,
			.len 	= 1,
			.buf 	= &reg_addr,
		},
		{
			.addr 	= client->addr,
			.flags 	= I2C_M_RD,
			.len 	= 1,
			.buf 	= rdbuf,
		},
	};

	if (client == NULL) {
		pr_err("msg %s i2c client is NULL\n", __func__);
		return -ENODEV;
	}

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	*val = rdbuf[0];

	return ret;
}


/*********************************************************
 *
 * awxxxx i2c write bytes
 *
 ********************************************************/
static int i2c_write_multi_byte(struct i2c_client *client,
				unsigned char reg_addr,
				unsigned char *buf, unsigned int len)
{
	int ret = 0;
	unsigned char *wdbuf;

	wdbuf = kmalloc(len + 1, GFP_KERNEL);
	if (wdbuf == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	wdbuf[0] = reg_addr;
	memcpy(&wdbuf[1], buf, len);

	ret = i2c_master_send(client, wdbuf, len + 1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(wdbuf);

	return ret;
}

static unsigned int i2c_read_multi_byte(struct i2c_client *client,
					unsigned char reg_addr,
					unsigned char *buf,
					unsigned int len)
{
	int ret = 0;
	unsigned char *rdbuf = NULL;

	struct i2c_msg msgs[] = {
		{
			.addr 	= client->addr,
			.flags 	= 0,
			.len 	= 1,
			.buf 	= &reg_addr,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
		},
	};

	rdbuf = kmalloc(len, GFP_KERNEL);
	if (rdbuf == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return  -ENOMEM;
	}

	msgs[1].buf = rdbuf;

	if (client == NULL) {
		pr_err("msg %s i2c client is NULL\n", __func__);
		return -ENODEV;
		kfree(rdbuf);
	}

	ret = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	if (buf != NULL)
		memcpy(buf, rdbuf, len);
	kfree(rdbuf);
	return ret;
}
/******************************************************
 *
 * aw9523b chip ctrol interface
 *
 ******************************************************/
 /* val 0 -> led, gpio -> 1 */
static void aw9523b_set_port_mode_by_mask(struct aw9523b *p_aw9523b,
					unsigned int mask,
					unsigned int val)
{
	unsigned char reg_val[2] = {0};
	int i = 0;

	/* p0_0 - P1_7 */
	i2c_read_multi_byte(p_aw9523b->client, P0_LED_MODE, &reg_val[0],
					sizeof(reg_val) / sizeof(reg_val[0]));
	for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
		if (mask & (0x01 << i)) {
			if (val) {
				if (i < 8)
					reg_val[0] |= 0x1 << i;
				else
					reg_val[1] |= 0x1 << (i - 8);
			} else {
				if (i < 8)
					reg_val[0] &= ~(0x1 << i);
				else
					reg_val[1] &= ~(0x1 << (i - 8));
			}
		}
	}
	i2c_write_multi_byte(p_aw9523b->client, P0_LED_MODE, reg_val,
					sizeof(reg_val) / sizeof(reg_val[0]));
}

/* val 0 -> OD 1-> push pull */
static void aw9523b_set_port_output_mode(struct aw9523b *p_aw9523b,
							unsigned char val)
{
	unsigned char reg_val[1] = {0};

	i2c_read_byte(p_aw9523b->client, CTL_REG, &reg_val[0]); /* p0_0 -- p0_7 */
	if (val)
		reg_val[0] |= 0x01 << 4;
	else
		reg_val[0] &= ~(0x01 << 4);
	i2c_write_byte(p_aw9523b->client, CTL_REG, reg_val[0]);
}


/* val 0-> output  1->input */
static void aw9523b_set_port_direction_by_mask(struct aw9523b *p_aw9523b,
						unsigned int mask,
						int val)
{
	unsigned char reg_val[2] = {0};
	int i = 0;
	struct aw9523b_key *p_key_data = p_aw9523b->key_data;

	// kcm modify by phf, 20250123
	#if 1
	reg_val[0] = p_key_data->port_dir_config & 0xff;
	reg_val[1] = p_key_data->port_dir_config >> 8;
	#else
	i2c_read_multi_byte(p_aw9523b->client, P0_CONFIG, reg_val,
					sizeof(reg_val) / sizeof(reg_val[0]));
	#endif
	
	/* 0-output 1-input */
	for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
		if (mask & (0x01 << i)) {
			if (val) {
				if (i < 8)
					reg_val[0] |= 0x1 << i;
				else
					reg_val[1] |= 0x1 << (i - 8);
			} else {
				if (i < 8)
					reg_val[0] &= ~(0x1 << i);
				else
					reg_val[1] &= ~(0x1 << (i - 8));
			}
		}
	}
	i2c_write_multi_byte(p_aw9523b->client, P0_CONFIG, reg_val,
					sizeof(reg_val)/sizeof(reg_val[0]));
					
	// kcm added by phf, 20250123
	p_key_data->port_dir_config = (reg_val[1]<<8) |	reg_val[0];			
}


/* val 1-> output high  0->output low */
static void aw9523b_set_port_output_state_by_mask(struct aw9523b *p_aw9523b,
						unsigned int mask, int val)
{
	unsigned char reg_val[2] = {0};
	int i = 0;

	i2c_read_multi_byte(p_aw9523b->client, P0_OUTPUT, reg_val,
					sizeof(reg_val)/sizeof(reg_val[0]));
	for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
		if (mask & (0x01 << i)) {
			if (val) {
				if (i < 8)
					reg_val[0] |= 0x1 << i;
				else
					reg_val[1] |= 0x1 << (i - 8);
			} else {
				if (i < 8)
					reg_val[0] &= ~(0x1 << i);
				else
					reg_val[1] &= ~(0x1 << (i - 8));
			}
		}
	}
	i2c_write_multi_byte(p_aw9523b->client, P0_OUTPUT, reg_val,
					sizeof(reg_val) / sizeof(reg_val[0]));
}

#if 0

/* val 1-> output high  0->output low */
static unsigned int aw9523b_get_port_output_state_by_mask(struct aw9523b *p_aw9523b,
						unsigned int mask, int val)
{
	unsigned char reg_val[2] = {0};

	i2c_write_byte(p_aw9523b->client, P0_OUTPUT, reg_val[0]);
	i2c_write_byte(p_aw9523b->client, P1_OUTPUT, reg_val[1]);
	return (reg_val[0] & 0x0f) | ((reg_val[1] << 8) & 0xFF00);
}

/* val 1-> output high  0->output low */
static unsigned int aw9523b_get_port_input_state(struct aw9523b *p_aw9523b)
{
	unsigned char reg_val[2] = {0};

	i2c_read_multi_byte(p_aw9523b->client, P0_INPUT, reg_val,
					sizeof(reg_val) / sizeof(reg_val[0]));
	return reg_val[0] | (reg_val[1] << 8);
}

#endif

/* val 1 -> enable 0 - > disable */
static void aw9523b_enbale_interrupt_by_mask(struct aw9523b *p_aw9523b,
						unsigned int mask,
						unsigned int val)
{
	unsigned char reg_val[2] = {0};
	int i = 0;

	i2c_read_multi_byte(p_aw9523b->client, P0_INT, reg_val,
					sizeof(reg_val) / sizeof(reg_val[0]));
	/* 0 enable 1 disable */
	for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
		if (mask & (0x01<<i)) {
			if (!val) {
				if (i < 8)
					reg_val[0] |= 0x1 << i;
				else
					reg_val[1] |= 0x1 << (i - 8);
			} else {
				if (i < 8)
					reg_val[0] &= ~(0x1 << i);
				else
					reg_val[1] &= ~(0x1 << (i - 8));
			}
		}
	}
	i2c_write_multi_byte(p_aw9523b->client, P0_INT, reg_val,
					sizeof(reg_val) / sizeof(reg_val[0]));
}

/* Must be read single-byte */
static void aw9523b_clear_interrupt(struct aw9523b *p_aw9523b)
{
	unsigned char reg_val[2] = {0};

	i2c_read_byte(p_aw9523b->client, P0_INPUT, &reg_val[0]);
	i2c_read_byte(p_aw9523b->client, P1_INPUT, &reg_val[1]);
}

/* val 0x00-0x03 */
static void aw9523b_set_led_imax(struct aw9523b *p_aw9523b, unsigned char val)
{
	unsigned char reg_val[1] = {0};

	i2c_read_byte(p_aw9523b->client, CTL_REG, &reg_val[0]);
	reg_val[0] &= ~0x03;
	reg_val[0] |= (val & 0x03);
	i2c_write_byte(p_aw9523b->client, CTL_REG, reg_val[0]);
}

static void aw9523b_set_dim_by_mask(struct aw9523b *p_aw9523b, unsigned int mask,
								unsigned int val)
{
	int i = 0;

	AW_DEBUG("aw9523b_set_dim_by_mask value is %d\n",val);

	for (i = 0; i < AW9523B_LED_MAX_NUMS; i++) {
		if (mask & (0x1 << i)) {
			if (i < 8) {
				i2c_write_byte(p_aw9523b->client,
							P0_0_DIM0 + i, val);
			} else if (i > 11) {
				i2c_write_byte(p_aw9523b->client,
							P1_4_DIM0 + (i - 12), val);
			} else {
				i2c_write_byte(p_aw9523b->client,
							P1_0_DIM0 + (i - 8), val);
			}
		}
	}
}
static int aw9523b_reset(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	struct device_node *node = p_aw9523b->dev->of_node;

	p_aw9523b->rst_gpio = of_get_named_gpio(node, "reset-gpio", 0);

	if ((!gpio_is_valid(p_aw9523b->rst_gpio))) {
		dev_err(p_aw9523b->dev, "%s: dts don't provide reset-gpio\n",
			__func__);
		return -EINVAL;
	}

	ret = gpio_request(p_aw9523b->rst_gpio, "aw9523b-reset");
	if (ret) {
		dev_err(&p_aw9523b->client->dev,
			"%s: unable to request gpio [%d]\n", __func__,
			p_aw9523b->rst_gpio);
		return ret;
	}

	ret = gpio_direction_output(p_aw9523b->rst_gpio, 1);
	if (ret) {
		gpio_free(p_aw9523b->rst_gpio);
		dev_err(p_aw9523b->dev, "%s: unable to set direction of gpio\n",
			__func__);
		return ret;
	}
	//gpio_set_value(p_aw9523b->rst_gpio, 1);
	//mdelay(1);
	gpio_set_value(p_aw9523b->rst_gpio, 0);
	mdelay(1);
	gpio_set_value(p_aw9523b->rst_gpio, 1);
	mdelay(5);
	return ret;
}


static int aw9523b_parse_dt_for_feature_enable(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	unsigned int val = 0;
	struct device_node *np = NULL;

	if (p_aw9523b == NULL || p_aw9523b->dev == NULL) {
		dev_err(p_aw9523b->dev, "%s: error p_aw9523b is NULL\n", __func__);
		return -EINVAL;
	}
	np = p_aw9523b->dev->of_node;

	p_aw9523b->single_key_enable = false;
	ret = of_property_read_u32(np, "aw9523b,single_key_enable", &val);
	if (ret) {
		dev_err(p_aw9523b->dev,
			"%s: no aw9523b,single_key_enable provided,The default function is not key\n",
			__func__);
	} else {
		AW_DEBUG("%s:aw9523b,single_key_enable provided ok, val = %d\n",
			__func__, val);
		if (val == 1)
			p_aw9523b->single_key_enable = true;
	}

	p_aw9523b->matrix_key_enable = false;
	ret = of_property_read_u32(np, "aw9523b,matrix_key_enable", &val);
	if (ret) {
		dev_err(p_aw9523b->dev, "%s: no aw9523b,key_enable provided, The default function is not key\n",
			__func__);
	} else {
		AW_DEBUG("%s:aw9523b,key_enable provided ok, val = %d\n",
			__func__, val);
		if (val == 1)
			p_aw9523b->matrix_key_enable = true;
	}

	p_aw9523b->led_feature_enable = false;
	ret = of_property_read_u32(np, "aw9523b,led_enable", &val);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s: no aw9523b,led_enable provided, The default function is not LED\n",
			__func__);
	} else {
		AW_DEBUG("%s:aw9523b,led_enable provided ok, val = %d\n",
			__func__, val);
		if (val == 1)
			p_aw9523b->led_feature_enable = true;
	}

	p_aw9523b->gpio_feature_enable = false;
	ret = of_property_read_u32(np, "aw9523b,gpio_enable", &val);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s: aw9523b,gpio_enable provided, The default function is not GPIO\n",
			__func__);
	} else {
		AW_DEBUG("%s:aw9523b,gpio_enable provided ok, val = %d\n",
			__func__, val);
		if (val == 1)
			p_aw9523b->gpio_feature_enable = true;
	}
	return ret;
}


/*********************************************************
 *
 * aw9523b reg
 *
 ********************************************************/
static ssize_t aw9523b_get_reg(struct device *cd,
			    struct device_attribute *attr,
			    char *buf)
{
	unsigned char reg_val = 0;
	unsigned char i = 0;
	int j = 0;
	ssize_t len = 0;
	struct aw9523b *p_aw9523b = dev_get_drvdata(cd);
	struct i2c_client *client = p_aw9523b->client;

	for (j = 0; j < 0x30; j++) {
		i2c_read_byte(client, j, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg%2x = 0x%2x\n",
				j, reg_val);
		switch (j) {
		case P0_LED_MODE:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
						PAGE_SIZE - len,
						"P0_%d gpio mode\n", i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d led mode\n", i);
				}
			}
			break;
		case P1_LED_MODE:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d gpio mode\n", i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d led mode\n", i);
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "\r\n");
			break;
		case P0_CONFIG:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d input mode \n", i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d output mode\n",
							i);
				}
			}
			break;
		case P1_CONFIG:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
						PAGE_SIZE - len,
						"P1_%d input mode\n", i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d output mode\n",
							i);
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "\r\n");
			break;
		case P0_INT:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d interrrupt disable\n",
							i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d interrrupt enable\n",
							i);
				}
			}
			break;
		case P1_INT:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d interrrupt disable\n",
							i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d interrrupt enable\n",
							i);
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "\r\n");
			break;
		case P0_OUTPUT:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d output high\n",
							i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d output low\n",
							i);
				}
			}
			break;
		case P1_OUTPUT:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
						PAGE_SIZE - len,
						"P1_%d output high\n",
						i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d output low\n",
							i);
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "\r\n");
			break;

		case P0_INPUT:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
						PAGE_SIZE - len,
						"P0_%d input high\n",
						i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P0_%d intpu low\n",
							i);
				}
			}
			break;
		case P1_INPUT:
			for (i = 0; i < 8; i++) {
				if (reg_val & (0x01 << i)) {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d input high\n",
							i);
				} else {
					len += snprintf(buf + len,
							PAGE_SIZE - len,
							"P1_%d input low\n",
							i);
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "\r\n");
			break;
		case CTL_REG:
			if (reg_val & (0x1 << 7)) {
				len += snprintf(buf + len, PAGE_SIZE - len,
						"gpio Pull-Push mode\n");
			} else {
				len += snprintf(buf + len, PAGE_SIZE - len,
						"gpio OD mode\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "\r\n");
			break;
		default:
			break;
		}

	}

	return len;
}

static ssize_t aw9523b_set_reg(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	unsigned int databuf[2];
	struct aw9523b *p_aw9523b = dev_get_drvdata(dev);
	struct i2c_client *client = p_aw9523b->client;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		i2c_write_byte(client, databuf[0], databuf[1]);
	return len;
}

static DEVICE_ATTR(reg, 0660, aw9523b_get_reg, aw9523b_set_reg);
static int aw9523b_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	return err;
}

/*********************************************************
 *
 * aw9523b check chipid
 *
 ********************************************************/
static int aw9523b_read_chipid(struct i2c_client *client)
{
	unsigned char val;
	unsigned char cnt;

	for (cnt = 5; cnt > 0; cnt--) {
		i2c_read_byte(client, ID_REG, &val);

		/* 0x80:aw9523ba,0x23:aw9523b */
		AW_DEBUG("aw9523b_read_chipid val =0x%x\n", val);
		if (val == AW9523B_ID)
			return 0;
		mdelay(5);
	}
	return -EINVAL;
}

/*********************************************************
 *
 * aw9523b key feature
 *
 ********************************************************/
static int aw9523b_parse_dt_for_key(struct aw9523b_key *p_key_data,
					struct device_node *np)
{
	int ret = 0;
	int i = 0;
	unsigned int val = 0;
	struct aw9523b *p_aw9523b = p_key_data->priv;

	p_key_data->wake_up_enable = false;
	ret = of_property_read_u32(np, "aw9523b,wake_up_enable", &val);
	if (ret) {
		dev_err(p_aw9523b->dev, "%s: no aw9523b,wake_up_enable, abort\n",
			__func__);
		return ret;
	} else {
		AW_DEBUG("%s:aw9523b,wake_up_enable provided ok, val = %d\n",
			__func__, val);
		if (val == 1)
			p_key_data->wake_up_enable = true;
	}

	ret = of_property_read_u32(np, "aw9523b,input_port_mask",
				&p_key_data->input_port_mask);
	if (ret) {
		dev_err(p_aw9523b->dev, "%s: no aw9523b,input_port_mask, abort\n",
			__func__);
		return ret;
	}

	p_key_data->input_port_nums = 0;
	for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
		if (p_key_data->input_port_mask & (0x01 << i))
			p_key_data->input_port_nums++;
	}

	if (p_aw9523b->single_key_enable) {
		/*
		*AW_DEBUG("aw9523b key input input_port_mask = %d\r\n", p_key_data->input_port_nums);
		*/
		p_key_data->key_mask =  p_key_data->input_port_mask;
		AW_DEBUG("aw9523b key input_port_mask = 0x%x, input_num	= %d\n",
						p_key_data->input_port_mask,
						p_key_data->input_port_nums);
						
		p_key_data->port_dir_config = 0;				
	}
	if (p_aw9523b->matrix_key_enable) {
		ret = of_property_read_u32(np, "aw9523b,output_port_mask",
					&p_key_data->output_port_mask);
		if (ret) {
			dev_err(p_aw9523b->dev,
				"%s, no aw9523b,output_port_mask, abort\n",
				__func__);
			return ret;
		}
		p_key_data->output_port_nums = 0;
		for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
			if (p_key_data->output_port_mask & (0x01 << i))
				p_key_data->output_port_nums++;
		}
		p_key_data->key_mask
		= p_key_data->input_port_mask | p_key_data->output_port_mask;
		AW_DEBUG("aw9523b key output_port_mask = 0x%x, output_nmu = %d\n",
						p_key_data->output_port_mask,
						p_key_data->output_port_nums);
						
		p_key_data->port_dir_config = 0;
	}

	return 0;
}

irqreturn_t aw9523b_irq_func(int irq, void *key_data)
{
	struct aw9523b_key *p_key_data = (struct aw9523b_key *)key_data;

	disable_irq_nosync(p_key_data->priv->irq_num);
	/* disable aw9523b input interrupt */
	
	//schedule_delayed_work(&p_key_data->int_work,
	//		      msecs_to_jiffies(p_key_data->debounce_delay));	
	schedule_work(&p_key_data->key_work);
	
	return 0;
}

static int aw9523b_irq_register(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	struct device_node *np = p_aw9523b->dev->of_node;

	p_aw9523b->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (p_aw9523b->irq_gpio < 0) {
		dev_err(p_aw9523b->dev, "%s: get irq gpio failed\r\n", __func__);
		return -EINVAL;
	}
	ret = devm_gpio_request(p_aw9523b->dev, p_aw9523b->irq_gpio,
				"aw9523b irq gpio");
	if (ret) {
		dev_err(p_aw9523b->dev,
			"%s: devm_gpio_request irq gpio failed\r\n", __func__);
		return -EBUSY;
	}
	gpio_direction_input(p_aw9523b->irq_gpio);
	p_aw9523b->irq_num = gpio_to_irq(p_aw9523b->irq_gpio);
	if (p_aw9523b->irq_num < 0) {
		ret = p_aw9523b->irq_num;
		dev_err(p_aw9523b->dev, "%s gpio to irq failed\r\n", __func__);
		goto err;
	}

	AW_DEBUG("aw9523b irq num=%d\n", p_aw9523b->irq_num);
	//ret = devm_request_threaded_irq(p_aw9523b->dev, p_aw9523b->irq_num, NULL,
	ret = devm_request_irq(p_aw9523b->dev, p_aw9523b->irq_num,
					aw9523b_irq_func,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"aw9523b_irq", p_aw9523b->key_data);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s register irq failed\r\n", __func__);
		goto err;
	}
	/* enable_irq_wake(p_aw9523b->irq_num); */
	device_init_wakeup(p_aw9523b->dev, 1);
	return 0;
err:
	devm_gpio_free(p_aw9523b->dev, p_aw9523b->irq_gpio);
	return ret;
}

static int aw9523b_input_dev_register(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	int i = 0;
	struct aw9523b_key *p_key_data = p_aw9523b->key_data;

	p_key_data->input = input_allocate_device();

	if (p_key_data->input == NULL) {
		ret = -ENOMEM;
		dev_err(p_aw9523b->dev, "%s: failed to allocate input device\n",
					__func__);
		return ret;
	}
	p_key_data->input->name = "aw9523b-key";
	p_key_data->input->dev.parent = p_aw9523b->dev;
	p_key_data->keymap_len = sizeof(key_map) / sizeof(KEY_STATE);
	p_key_data->keymap = (KEY_STATE *)&key_map;
	input_set_drvdata(p_key_data->input, p_key_data);

	__set_bit(EV_KEY, p_key_data->input->evbit);
	__set_bit(EV_SYN, p_key_data->input->evbit);
	for (i = 0; i < p_key_data->keymap_len; i++) {
		__set_bit(p_key_data->keymap[i].key_code & KEY_MAX,
						p_key_data->input->keybit);
	}

	ret = input_register_device(p_key_data->input);
	if (ret) {
		input_free_device(p_key_data->input);
		dev_err(p_aw9523b->dev, "%s: failed to allocate input device\n",
					__func__);
		return ret;
	}
	return 0;
}

// kcm modify by phf,20250121
#if 0
void aw9523b_int_work(struct work_struct *work)
{
	struct delayed_work *p_delayed_work = container_of(work,
							struct delayed_work,
							work);
	struct aw9523b_key *p_key_data = container_of(p_delayed_work,
						    struct aw9523b_key,
						    int_work);
	AW_DEBUG("%s enter, %d\r\n", __func__, __LINE__);
	schedule_work(&p_key_data->key_work);
}
#endif

/* val 1-> output high  0->output low */
static unsigned int aw9523b_get_port_input_state(struct aw9523b *p_aw9523b)
{
	unsigned char reg_val[2] = {0};

	i2c_read_byte(p_aw9523b->client, P0_INPUT, &reg_val[0]);
	i2c_read_byte(p_aw9523b->client, P1_INPUT, &reg_val[1]);
	return reg_val[0] | (reg_val[1] << 8);
}

static void aw9523b_key_work(struct work_struct *work)
{
	int i = 0;
	int j = 0;
	int real_idx = 0;
	int real_row = 0;
	int real_col = 0;
	int key_code = 0;
	int key_val = 0;
	int key_num = 0;
	unsigned int retry_state;
	unsigned int input_val = 0;
	struct aw9523b_key *p_key_data = container_of(work, struct aw9523b_key,
						key_work);
	struct aw9523b *p_aw9523b = p_key_data->priv;

#if 1
	unsigned int key_row_status, key_col_status;
	unsigned int port_dir_config;
#endif
	unsigned int *new_state = p_key_data->new_output_state;
	unsigned int *old_state = p_key_data->old_output_state;
	unsigned char reg_val[2] = {0};
/*
	if wake_up_enable set to 1,the key fuction can be use in suspend mode.
	if wake_up_enable set to 0,the key fuction will be banned in suspend mode.
*/
	if ((!p_key_data->wake_up_enable) && (!p_aw9523b->screen_state)) {
		
		AW_DEBUG("%s line=%d\n", __func__, __LINE__);		
		
		enable_irq(p_aw9523b->irq_num);
		aw9523b_enbale_interrupt_by_mask(p_aw9523b, p_key_data->input_port_mask, 1);
		aw9523b_clear_interrupt(p_aw9523b);
		return;
	}

	if (p_aw9523b->matrix_key_enable) {
		
	#if 1 // fix key may be not detect bug
		//key raw input state
		input_val = aw9523b_get_port_input_state(p_aw9523b) ;
		key_row_status = input_val & p_key_data->input_port_mask;
		//set output and input line to input, 
		port_dir_config = p_key_data->output_port_mask | p_key_data->input_port_mask;
		reg_val[0] = port_dir_config & 0xff;
		reg_val[1] = port_dir_config >> 8;
		i2c_write_multi_byte(p_aw9523b->client, P0_CONFIG,
				reg_val,
				sizeof(reg_val) / sizeof(reg_val[0]));		
		
		udelay(5);		// 10
		//pressed key col would be high level
		input_val = aw9523b_get_port_input_state(p_aw9523b);
		key_col_status = (~input_val) & p_key_data->output_port_mask;
		
		#if 0
		AW_DEBUG("aw9523b key_row_status=0x%x, key_col_status=0x%x", key_row_status,key_col_status);
		#endif
		
		/* key state change */
		if( p_key_data->old_key_row_status != key_row_status || p_key_data->old_key_col_status != key_col_status )
		{
			int press_col_num = 0;
			// get press col number
			for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
				if (p_key_data->output_port_mask & (0x01 << i)) {
					if((key_col_status & (0x01 << i)) ==0)
						press_col_num++;
				}
			}
			// if press col num == 1
			if(press_col_num <= 1)
			{				
				for (i = 0, real_idx = 0; i < AW9523B_KEY_PORT_MAX; i++) {
					if (p_key_data->output_port_mask & (0x01 << i)) {
						if((key_col_status & (0x01 << i)) ==0)
							new_state[real_idx] = key_row_status;
						else
							new_state[real_idx] = p_key_data->input_port_mask;
						
						real_idx++;
					}
				}
			}
			else //if press col num > 1, need scan pressed col
			{
				//key scan, set press col to low level, one by one
				for (i = 0, real_idx = 0; i < AW9523B_KEY_PORT_MAX; i++) {
					if (p_key_data->output_port_mask & (0x01 << i)) {
						
						if((key_col_status & (0x01 << i)) ==0)
						{
							port_dir_config = p_key_data->output_port_mask | p_key_data->input_port_mask;	
							port_dir_config &= ~(0x01 << i);

							reg_val[0] = port_dir_config & 0xff;
							reg_val[1] = (port_dir_config >> 8) & 0xff;

							i2c_write_multi_byte(p_aw9523b->client, P0_CONFIG,
									reg_val,
									sizeof(reg_val) / sizeof(reg_val[0]));
									
							// delay 10 us 		
							udelay(5);		// 10

							//read P0,P1 input state
							//i2c_read_multi_byte(p_aw9523b->client, P0_INPUT,
							//		reg_val,
							//		sizeof(reg_val)/sizeof(reg_val[0]));
							//new_state[real_idx] = (reg_val[0] | (reg_val[1] << 8)) & p_key_data->input_port_mask;
							input_val = aw9523b_get_port_input_state(p_aw9523b);
							new_state[real_idx] = input_val & p_key_data->input_port_mask;
						}
						else {
							new_state[real_idx] = p_key_data->input_port_mask;
						}
						
						real_idx++;
					}	
				}
			}
			
#if 0
			/* debug info */
			for (i = 0, real_idx = 0; i < AW9523B_KEY_PORT_MAX; i++) {
				if (p_key_data->output_port_mask & (0x01 << i)) {
					AW_DEBUG("new-state[%d] = 0x%x, old_state[%d] = 0x%x\r\n",
						real_idx, new_state[real_idx], real_idx,
						old_state[real_idx]);
					real_idx++;
				}
			}
#endif			
			//check key
			for (i = 0, real_col = 0; i < AW9523B_KEY_PORT_MAX; i++) {
				if (p_key_data->output_port_mask & (0x01 << i)) {
					if (new_state[real_col] != old_state[real_col]) {
						for (j = 0, real_row = 0; j < AW9523B_KEY_PORT_MAX; j++) {
							if (p_key_data->input_port_mask & (0x01 << j)) {
								if ((new_state[real_col] & (0x01 << j)) != (old_state[real_col] & (0x01 << j))) {
									key_code = p_key_data->keymap[real_row * p_key_data->output_port_nums + real_col].key_code;
									key_val = (old_state[real_col] & (0x01 << j)) ? 1 : 0;/* press or release */
									AW_DEBUG("aw9523b report: key_code = %d, key_val = %d\n", key_code, key_val);
									AW_DEBUG("aw9523b real_row = %d, real_col = %d\n", real_row, real_col);
									input_report_key(p_key_data->input, key_code, key_val);
									input_sync(p_key_data->input);
								}
								real_row++;
							}
						}
					}
					real_col++;
				}
			}
			
			//save state
			memcpy(&old_state[0], &new_state[0],
				p_key_data->output_port_nums * sizeof(unsigned int));
			p_key_data->old_key_row_status = key_row_status;
			p_key_data->old_key_col_status = key_col_status;
			
		}
		

		//set port direction, reinit output port and input port
		//port_dir_config = p_key_data->port_dir_config;
		port_dir_config = (~p_key_data->output_port_mask) &  p_key_data->input_port_mask ;
		reg_val[0] = port_dir_config & 0xff;
		reg_val[1] = port_dir_config >> 8;
		i2c_write_multi_byte(p_aw9523b->client, P0_CONFIG,
				reg_val,
				sizeof(reg_val) / sizeof(reg_val[0]));
					
		/* all key release */
		if ( (key_row_status & p_key_data->input_port_mask) == p_key_data->input_port_mask 
			&& (key_col_status & p_key_data->output_port_mask) == p_key_data->output_port_mask) {
							
			AW_DEBUG("all key release. re-check...\n");				
			//re-check if all key release
					
			udelay(5);		// 10
			retry_state = aw9523b_get_port_input_state(p_aw9523b);
			if (( retry_state & p_key_data->input_port_mask ) == p_key_data->input_port_mask) {
				AW_DEBUG("%s enter, all key release.\n", __func__);
				
				//aw9523b_enbale_interrupt_by_mask(p_aw9523b, p_key_data->input_port_mask, 1); /* enable input interrupt */
				/* clear inputerrupt */
				//	(p_aw9523b);
				enable_irq(p_aw9523b->irq_num);
				
				return;
			}
		}	
	
	#else
		
		//key scan,
		for (i = 0, real_idx = 0; i < AW9523B_KEY_PORT_MAX; i++) {
			if (p_key_data->output_port_mask & (0x01 << i)) {
					
				/*key scan, set current output line to output mode, default out put low level. 
				set others output line to input mode */
				// kcm modify by phf, 20250123
				#if 1
				reg_val[0] = p_key_data->port_dir_config & 0xff;
				reg_val[1] = p_key_data->port_dir_config >> 8;
				#else
				i2c_read_multi_byte(p_aw9523b->client, P0_CONFIG,
						reg_val,
						sizeof(reg_val)/sizeof(reg_val[0]));
				#endif

				input_val = reg_val[0] | (reg_val[1] << 8);
				input_val |= p_key_data->output_port_mask;
				input_val &= ~(0x01 << i);

				reg_val[0] = input_val & 0xff;
				reg_val[1] = (input_val >> 8) & 0xff;

				i2c_write_multi_byte(p_aw9523b->client, P0_CONFIG,
						reg_val,
						sizeof(reg_val) / sizeof(reg_val[0]));
				
				// kcm added by phf, 20250123
				p_key_data->port_dir_config = (reg_val[1]<<8) |	reg_val[0];
				
				// delay 10 us 		
				udelay(10);		// 10

				//read P0,P1 input state
				//i2c_read_multi_byte(p_aw9523b->client, P0_INPUT,
				//		reg_val,
				//		sizeof(reg_val)/sizeof(reg_val[0]));
				//new_state[real_idx] = (reg_val[0] | (reg_val[1] << 8)) & p_key_data->input_port_mask;
				input_val = aw9523b_get_port_input_state(p_aw9523b);
				new_state[real_idx] = input_val & p_key_data->input_port_mask;
				real_idx++;
			}
		}

#if 0
		/* debug info */
		for (i = 0, real_idx = 0; i < AW9523B_KEY_PORT_MAX; i++) {
			if (p_key_data->output_port_mask & (0x01 << i)) {
				AW_DEBUG("new-state[%d] = 0x%x, old_state[%d] = 0x%x\r\n",
					real_idx, new_state[real_idx], real_idx,
					old_state[real_idx]);
				real_idx++;
			}
		}
#endif

		/* key state change */
		if (memcmp(&new_state[0], &old_state[0], p_key_data->output_port_nums * sizeof(unsigned int))) {
			/* stage changed */
			for (i = 0, real_col = 0; i < AW9523B_KEY_PORT_MAX; i++) {
				if (p_key_data->output_port_mask & (0x01 << i)) {
					if (new_state[real_col] != old_state[real_col]) {
						for (j = 0, real_row = 0; j < AW9523B_KEY_PORT_MAX; j++) {
							if (p_key_data->input_port_mask & (0x01 << j)) {
								if ((new_state[real_col] & (0x01 << j)) != (old_state[real_col] & (0x01 << j))) {
									key_code = p_key_data->keymap[real_row * p_key_data->output_port_nums + real_col].key_code;
									key_val = (old_state[real_col] & (0x01 << j)) ? 1 : 0;/* press or release */
									AW_DEBUG("aw9523b report: key_code = %d, key_val = %d\n", key_code, key_val);
									AW_DEBUG("aw9523b real_row = %d, real_col = %d\n", real_row, real_col);
									input_report_key(p_key_data->input, key_code, key_val);
									input_sync(p_key_data->input);
								}
								real_row++;
							}
						}
					}
					real_col++;
				}
			}
		}

		memcpy(&old_state[0], &new_state[0],
				p_key_data->output_port_nums * sizeof(unsigned int));

		/* all key release */
		if ( memcmp(&new_state[0], &p_key_data->def_output_state[0],
			p_key_data->output_port_nums * sizeof(unsigned int)) == 0 ) {
							
			//re-check if all key release
			//aw9523b_set_port_output_state_by_mask(p_aw9523b, p_key_data->output_port_mask, 0);/* set output low */
			aw9523b_set_port_direction_by_mask(p_aw9523b, p_key_data->output_port_mask, 0); /* set output mode */
			udelay(10);		// 10
			retry_state = aw9523b_get_port_input_state(p_aw9523b);
			if (( retry_state & p_key_data->input_port_mask ) == p_key_data->input_port_mask) {
				AW_DEBUG("%s enter, all key release.\n", __func__);
				
				aw9523b_enbale_interrupt_by_mask(p_aw9523b, p_key_data->input_port_mask, 1); /* enable input interrupt */
				/* clear inputerrupt */
				//	(p_aw9523b);
				enable_irq(p_aw9523b->irq_num);
				
				return;
			}
		}
		
	#endif
	
		//AW_DEBUG("%s Have key down, start timer...\n", __func__);
		hrtimer_start(&p_key_data->key_timer, ktime_set(0, (1000*1000 / HRTIMER_FRAME) * 1000), HRTIMER_MODE_REL);
		
	}
	if (p_aw9523b->single_key_enable) {
		p_key_data->new_input_state = aw9523b_get_port_input_state(p_aw9523b);
		p_key_data->new_input_state &= p_key_data->input_port_mask;
		real_idx = 0;
		if (p_key_data->new_input_state != p_key_data->old_input_state) {
			for (i = 0; i < AW9523B_KEY_PORT_MAX; i++) {
				if (p_key_data->input_port_mask & (0x01 << i)) {
					if ((p_key_data->new_input_state & (0x01 << i)) != (p_key_data->old_input_state & (0x01 << i))) {
						key_code = p_key_data->keymap[real_idx].key_code;
						key_val = (p_key_data->old_input_state & 0x01 << i) ? 1 : 0; /* press or release */

						key_num = aw9523b_separate_key_data[real_idx];
						if (key_val == 1)
							AW_DEBUG("%s, key%d pressed\n", __func__, key_num);
						else
							AW_DEBUG("%s, key%d release\n", __func__, key_num);
						/* AW_DEBUG("%s, key_code = 0x%x, key_val = 0x%x\r\n", __func__, key_code, key_val); */
						input_report_key(p_key_data->input, key_code, key_val);
						input_sync(p_key_data->input);
					}
					real_idx++;
				}
			}
		}
		/* AW_DEBUG("%s,new input state = 0x%x, old input state = 0x%x\r\n",
					__func__, p_key_data->new_input_state,
					p_key_data->old_input_state);
		*/
		p_key_data->old_input_state = p_key_data->new_input_state;
		aw9523b_clear_interrupt(p_aw9523b);
		enable_irq(p_aw9523b->irq_num);
		/* enable input interrupt */
		aw9523b_enbale_interrupt_by_mask(p_aw9523b,
						p_key_data->input_port_mask, 1);
		return;
	}
}

static enum hrtimer_restart aw9523b_timer_func(struct hrtimer *p_hrtimer)
{
	struct aw9523b_key *p_key_data = container_of(p_hrtimer, struct aw9523b_key, key_timer);

	schedule_work(&p_key_data->key_work);
	return HRTIMER_NORESTART;
}

static void aw9523b_key_chip_init(struct aw9523b_key *p_key_data)
{
	unsigned int all_mask = 0;
	struct aw9523b *p_aw9523b = p_key_data->priv;

	AW_DEBUG("%s enter\n", __func__);

	disable_irq_nosync(p_key_data->priv->irq_num);
	aw9523b_enbale_interrupt_by_mask(p_aw9523b, AW9523B_INT_MASK, 0); /* disale p0 p1 interrupt */
	all_mask = p_key_data->input_port_mask | p_key_data->output_port_mask;
	aw9523b_set_port_mode_by_mask(p_aw9523b, all_mask, 1); /* set  gpio mode */
	
		// kcm added by phf, 20250123
	p_key_data->port_dir_config = 0;
	
	aw9523b_set_port_direction_by_mask(p_aw9523b, p_key_data->input_port_mask, 1); /* input mode */
	aw9523b_set_port_direction_by_mask(p_aw9523b, p_key_data->output_port_mask, 0); /* output mode */
	aw9523b_set_port_output_mode(p_aw9523b, 1); /* set output port pull push mode */

	aw9523b_set_port_output_state_by_mask(p_aw9523b, p_key_data->output_port_mask, 0);/* set output low */

	aw9523b_enbale_interrupt_by_mask(p_aw9523b, p_key_data->input_port_mask, 1); /* enable input interrupt */
	/* clear inputerrupt */
	aw9523b_clear_interrupt(p_aw9523b);
	enable_irq(p_key_data->priv->irq_num);
}

static void aw9523b_key_free_all_resource(struct aw9523b *p_aw9523b)
{	
	if (p_aw9523b->matrix_key_enable) {
		devm_kfree(p_aw9523b->dev, p_aw9523b->key_data->new_output_state);
		devm_kfree(p_aw9523b->dev, p_aw9523b->key_data->old_output_state);
		devm_kfree(p_aw9523b->dev, p_aw9523b->key_data->def_output_state);
		input_unregister_device(p_aw9523b->key_data->input);
		input_free_device(p_aw9523b->key_data->input);
		devm_gpio_free(p_aw9523b->dev, p_aw9523b->irq_gpio);
		devm_kfree(p_aw9523b->dev, p_aw9523b->key_data);
	}
}

static void aw9523b_single_key_chip_init(struct aw9523b_key *p_key_data)
{
	unsigned int all_mask = 0;
	struct aw9523b *p_aw9523b = p_key_data->priv;

	AW_DEBUG("%s enter\n", __func__);

	disable_irq_nosync(p_key_data->priv->irq_num);
	aw9523b_enbale_interrupt_by_mask(p_aw9523b, AW9523B_INT_MASK, 0); /* disale p0 p1 interrupt */
	all_mask = p_key_data->input_port_mask;
	p_key_data->old_input_state = p_key_data->input_port_mask;
	aw9523b_set_port_mode_by_mask(p_aw9523b, all_mask, 1); /* set  gpio mode */
	aw9523b_set_port_direction_by_mask(p_aw9523b, p_key_data->input_port_mask, 1); /* input mode */
	aw9523b_set_port_output_mode(p_aw9523b, 1); /* set output port pull push mode */

	aw9523b_enbale_interrupt_by_mask(p_aw9523b, p_key_data->input_port_mask, 1);/* enable input interrupt */
	/* clear inputerrupt */
	aw9523b_clear_interrupt(p_aw9523b);
	enable_irq(p_key_data->priv->irq_num);
}

static int aw9523b_key_feature_init(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	int i = 0;
	struct aw9523b_key *p_key_data = NULL;
	struct device_node *key_node = NULL;

	AW_DEBUG("%s enter\r\n", __func__);

	p_key_data = devm_kzalloc(p_aw9523b->dev, sizeof(struct aw9523b_key), GFP_KERNEL);
	if (p_key_data == NULL) {
		dev_err(p_aw9523b->dev, "%s: malloc memory failed\n", __func__);
		return -ENOMEM;
	}
	p_aw9523b->key_data = p_key_data;
	p_key_data->priv = p_aw9523b;
	key_node = of_find_node_by_name(p_aw9523b->dev->of_node, "aw9523b,key");
	if (key_node == NULL) {
		dev_err(p_aw9523b->dev, "%s: can't find aw9523b,key node return failed\n", __func__);
		ret = -EINVAL;
		goto err_id;
	}
	ret = aw9523b_parse_dt_for_key(p_key_data, key_node);
	if (ret) {
		dev_err(p_aw9523b->dev, "aw9523b_parse_dt_for_key failed, check dts\n");
		goto err_id;
	}
	p_key_data->old_output_state = devm_kzalloc(p_aw9523b->dev,
						    sizeof(unsigned int) * p_key_data->output_port_nums,
						    GFP_KERNEL);
	if (p_key_data->old_output_state == NULL) {
		dev_err(p_aw9523b->dev, "%s:p_key_data->old_output_state malloc memory failed\r\n", __func__);
		goto err_id;
	}
	p_key_data->new_output_state = devm_kzalloc(p_aw9523b->dev,
						    sizeof(unsigned int) * p_key_data->output_port_nums,
						    GFP_KERNEL);
	if (p_key_data->new_output_state == NULL) {
		dev_err(p_aw9523b->dev, "%s:p_key_data->new_output_state malloc memory failed\r\n", __func__);
		goto free_old_output;
	}

	p_key_data->def_output_state = devm_kzalloc(p_aw9523b->dev,
						    sizeof(unsigned int) * p_key_data->output_port_nums, GFP_KERNEL);
	if (p_key_data->def_output_state  == NULL) {
		dev_err(p_aw9523b->dev, "%s:p_key_data->def_output_state malloc memory failed\r\n", __func__);
		ret = -ENOMEM;
		goto free_new_output;
	}

	for (i = 0; i < p_key_data->output_port_nums; i++) {
		p_key_data->new_output_state[i] = p_key_data->input_port_mask;
		p_key_data->old_output_state[i] = p_key_data->input_port_mask;
		p_key_data->def_output_state[i] = p_key_data->input_port_mask;
	}
	
	// kcm added by phf, 20250123
	p_key_data->old_key_row_status = p_key_data->input_port_mask;
	p_key_data->old_key_col_status = p_key_data->output_port_mask;

	ret = aw9523b_input_dev_register(p_aw9523b);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s input dev register failed\r\n", __func__);
		goto free_def_output;
	}

	//kcm modify by phf, 20250123
	//p_key_data->debounce_delay = 1;
	//INIT_DELAYED_WORK(&p_key_data->int_work, aw9523b_int_work);
	INIT_WORK(&p_key_data->key_work, aw9523b_key_work);
	
	hrtimer_init(&p_key_data->key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p_key_data->key_timer.function = aw9523b_timer_func;
	ret = aw9523b_irq_register(p_aw9523b);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s irq register failed\r\n", __func__);
		goto input_dev;
	}

	if (p_aw9523b->matrix_key_enable)
		aw9523b_key_chip_init(p_key_data);

	if (p_aw9523b->single_key_enable)
		aw9523b_single_key_chip_init(p_key_data);
	return 0;
input_dev:
	input_unregister_device(p_key_data->input);
free_def_output:
	devm_kfree(p_aw9523b->dev, p_key_data->def_output_state);
free_new_output:
	devm_kfree(p_aw9523b->dev, p_key_data->new_output_state);
free_old_output:
	devm_kfree(p_aw9523b->dev, p_key_data->old_output_state);
err_id:
	devm_kfree(p_aw9523b->dev, p_aw9523b->key_data);
	return ret;
}


/******************************************************
 *
 * led feature start
 *
 ******************************************************/
/* led sys node */
/* echo imax > blink_param */
static ssize_t aw9523b_imax_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	unsigned int databuf[1] = {0};
	struct aw9523b *p_aw9523b = dev_get_drvdata(dev);
	struct aw9523b_led *p_led_data = p_aw9523b->led_data;

	sscanf(buf, "%x", &databuf[0]);
	p_led_data->imax = databuf[0] & 0x03;
	aw9523b_set_led_imax(p_aw9523b, p_led_data->imax);
	return len;
}

static ssize_t aw9523b_imax_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "echo imax> blink_param\n");
	return len;
}

static DEVICE_ATTR(imax, S_IWUSR | S_IRUGO, aw9523b_imax_show, aw9523b_imax_store);
static struct attribute *aw9523b_led_attributes[] = {
	&dev_attr_imax.attr,
	NULL
};

static struct attribute_group aw9523b_led_attribute_group = {
	.attrs = aw9523b_led_attributes
};

static int aw9523b_parse_dt_for_led_global_param(struct device_node *led_node,
						struct aw9523b *p_aw9523b,
						struct aw9523b_led *p_led_data)
{
	int ret = 0;

	ret = of_property_read_u32(led_node, "aw9523b,default_imax",
					      &p_led_data->imax);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s: no aw9523b,default_imax, abort\n",
					__func__);
		return ret;
	}
	return 0;
}

static void aw9523b_brightness_work(struct work_struct *work)
{
	struct aw9523b_single_led *p_single_led_data =
		container_of(work, struct aw9523b_single_led, brightness_work);
	struct aw9523b *p_aw9523b = p_single_led_data->priv;

	AW_DEBUG("%s enter, %d 0x%x\r\n", __func__, __LINE__,
		p_single_led_data->idx);

	//printk("aw9523b_brightness_set brightness value is %d\n",p_single_led_data->cur_brightness);

	aw9523b_set_dim_by_mask(p_aw9523b, 0x01,
			       p_single_led_data->cur_brightness);
	aw9523b_set_dim_by_mask(p_aw9523b, 0x02,
			       p_single_led_data->cur_brightness);
	aw9523b_set_dim_by_mask(p_aw9523b, 0x100,
			       p_single_led_data->cur_brightness);
	aw9523b_set_dim_by_mask(p_aw9523b, 0x200,
			       p_single_led_data->cur_brightness);				   				   				   
}

void aw9523b_brightness_set(struct led_classdev *led_cdev,
			   enum led_brightness brightness)
{
	struct aw9523b_single_led *p_single_led_data =
		container_of(led_cdev, struct aw9523b_single_led, cdev);

	AW_DEBUG("aw9523b_brightness_set enter , brightness value is %d\n",brightness);

	if (brightness > p_single_led_data->max_brightness)
		brightness = p_single_led_data->max_brightness;
	p_single_led_data->cdev.brightness = brightness;
	p_single_led_data->cur_brightness = brightness;
	schedule_work(&p_single_led_data->brightness_work);
}

enum led_brightness aw9523b_brightness_get(struct led_classdev *led_cdev)
{
	struct aw9523b_single_led *p_single_led_data =
		container_of(led_cdev, struct aw9523b_single_led, cdev);
	return p_single_led_data->cur_brightness;
}

static int aw9523b_parse_for_single_led(struct device_node *led_node,
				       struct aw9523b *p_aw9523b,
				       struct aw9523b_led *p_led_data)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	struct device_node *temp = NULL;
	unsigned int idx_count = 0;
	unsigned int *idx_array = NULL;
	struct aw9523b_single_led *p_single_led_data = p_led_data->single_led_data;

	AW_DEBUG("%s enter, %d\r\n", __func__, __LINE__);
	if ((p_led_data == NULL) || (p_led_data->single_led_data == NULL))
		return -EINVAL;
	for_each_child_of_node(led_node, temp) {
		ret = of_property_read_string(temp, "aw9523b,name",
					      &p_single_led_data[i].cdev.name);
		if (ret) {
			dev_err(p_aw9523b->dev,
				"%s:dts parse aw9523b,name failed, idx = %d\n",
				__func__, i);
			goto err_id;
		}
		ret = of_property_read_u32(temp, "aw9523b,idx_count", &idx_count);
		if (ret < 0) {
			dev_err(p_aw9523b->dev,
				"%s:dts parse aw9523b,idx_count, idx = %d\n",
				__func__, i);
			goto err_id;
		}
		idx_array = kzalloc(sizeof(unsigned int) * idx_count, GFP_KERNEL);
		if (idx_array == NULL) {
			dev_err(p_aw9523b->dev, "%s:kmalloc failed\r\n", __func__);
			ret = -ENOMEM;
			goto err_id;
		}
		ret = of_property_read_u32_array(temp, "aw9523b,idx", idx_array, idx_count);
		if (ret < 0) {
			dev_err(p_aw9523b->dev,
				"%s:dts parse aw9523b,idx failed, idx = %d\n",
				__func__, i);
			kfree(idx_array);
			goto err_id;
		}
		for (j = 0; j < idx_count; j++) {
			if (idx_array[j] < AW9523B_LED_MAX_NUMS) {
				p_single_led_data[i].idx |= (0x01 << idx_array[j]);
			} else {
				kfree(idx_array);
				dev_err(p_aw9523b->dev,
					"%s: dts err: aw9523b,idx parameter unavailable, idx =  %d\n",
					__func__, i);
				goto err_id;
			}
		}
		kfree(idx_array);
		ret = of_property_read_u32(temp, "aw9523b,default_brightness",
					  &p_single_led_data[i].cur_brightness);
		if (ret < 0) {
			dev_err(p_aw9523b->dev,
				"%s:dts parse aw9523b,default_brightness failed, idx = %d\n",
				__func__, i);
			goto err_id;
		}
		ret = of_property_read_u32(temp, "aw9523b,max_brightness",
					  &p_single_led_data[i].max_brightness);
		if (ret < 0) {
			dev_err(p_aw9523b->dev,
				"%s:dts parse aw9523b,idx failed, idx = %d\n",
				__func__, i);
			goto err_id;
		}
		p_led_data->led_mask |= p_single_led_data[i].idx;
		AW_DEBUG("aw9523b single led: i = %d, name = %s, idx_count = %d, idx = 0x%x\n",
					i,
					p_single_led_data[i].cdev.name,
					idx_count,
					p_single_led_data[i].idx);
		i++;
	}

	AW_DEBUG("aw9523b led_maks = 0x%x\n", p_led_data->led_mask);

	/* register led classdev */
	for (i = 0; i < p_led_data->led_nums; i++) {
		p_single_led_data[i].cdev.max_brightness = p_single_led_data[i].max_brightness;
		p_single_led_data[i].cdev.brightness_set = aw9523b_brightness_set;
		p_single_led_data[i].cdev.brightness_get = aw9523b_brightness_get;
		INIT_WORK(&p_single_led_data[i].brightness_work, aw9523b_brightness_work);
		p_single_led_data[i].priv = p_aw9523b;
		ret = led_classdev_register(p_aw9523b->dev, &p_single_led_data[i].cdev);
		if (ret) {
			dev_err(p_aw9523b->dev, "led classdev register failed, idx = %d\r\n", i);
			goto free_led_classdev;
		}
	}
	return 0;
free_led_classdev:
	for (i--; i > 0 || i == 0; i--)
		led_classdev_unregister(&p_single_led_data[i].cdev);
err_id:
	return ret;
}

static int aw9523b_led_chip_init(struct aw9523b *p_aw9523b, struct aw9523b_led *p_led_data)
{
	int i = 0;

	aw9523b_set_port_mode_by_mask(p_aw9523b, p_led_data->led_mask, 0); /* set led mode */
	aw9523b_set_led_imax(p_aw9523b, p_led_data->imax); /* set led breath level range */
	
	for (i = 0; i < p_led_data->led_nums; i++) {
		aw9523b_set_dim_by_mask(p_aw9523b,
				       p_led_data->single_led_data[i].idx,
					   0x00);
				       //p_led_data->single_led_data[i].cur_brightness);
	}
	
	return 0;
}

static void aw9523b_led_free_all_resource(struct aw9523b *p_aw9523b)
{
	int i = 0;

	if (p_aw9523b->led_feature_enable) {
		for (i = 0; i < p_aw9523b->led_data->led_nums; i++)
			led_classdev_unregister(&p_aw9523b->led_data->single_led_data[i].cdev);
		devm_kfree(p_aw9523b->dev, p_aw9523b->led_data);
	}
}

static int aw9523b_led_feature_init(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	int i = 0;
	struct device_node *led_node = NULL;
	struct aw9523b_led *p_led_data = NULL;

	AW_DEBUG("%s enter\r\n", __func__);
	p_led_data = devm_kzalloc(p_aw9523b->dev, sizeof(struct aw9523b_led), GFP_KERNEL);
	if (p_led_data == NULL) {
		dev_err(p_aw9523b->dev, "%s: malloc memory failed\r\n", __func__);
		return -ENOMEM;
	}
	p_aw9523b->led_data = p_led_data;
	led_node = of_find_node_by_name(p_aw9523b->dev->of_node, "aw9523b,led");
	if (led_node == NULL) {
		dev_err(p_aw9523b->dev,
			"%s: can't find aw9523b,led node return failed\r\n",
			__func__);
		ret = -EINVAL;
		goto err_id;
	}
	ret = aw9523b_parse_dt_for_led_global_param(led_node, p_aw9523b, p_led_data);
	if (ret) {
		dev_err(p_aw9523b->dev,
			"%s parse dt for led golbal param failed\r\n",
			__func__);
		goto err_id;
	}
	p_led_data->led_nums = of_get_child_count(led_node);
	AW_DEBUG("%s enter, %d led nums = %d\r\n",
					__func__, __LINE__,
					p_led_data->led_nums);

	p_led_data->single_led_data = devm_kzalloc(p_aw9523b->dev,
						   sizeof(struct aw9523b_single_led) * p_led_data->led_nums,
						   GFP_KERNEL);
	if (p_led_data->single_led_data == NULL) {
		dev_err(p_aw9523b->dev, "%s: malloc memory failed\r\n", __func__);
		ret = -ENOMEM;
		goto err_id;
	}
	ret = aw9523b_parse_for_single_led(led_node, p_aw9523b, p_led_data);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "aw9523b_parse_single_led failed\r\n");
		goto free_single_led;
	}
	ret = sysfs_create_group(&p_aw9523b->dev->kobj, &aw9523b_led_attribute_group);
	if (ret) {
		dev_err(p_aw9523b->dev, "led sysfs failed ret: %d\n", ret);
		goto free_led_class;
	}
	aw9523b_led_chip_init(p_aw9523b, p_led_data);
	return 0;
free_led_class:
	for (i = 0; i < p_led_data->led_nums; i++)
		led_classdev_unregister(&p_led_data->single_led_data[i].cdev);
free_single_led:
	devm_kfree(p_aw9523b->dev, p_led_data->single_led_data);
err_id:
	devm_kfree(p_aw9523b->dev, p_led_data);
	return ret;
}

/******************************************************
 *
 * gpio feature start
 *
 ******************************************************/
 /* gpio sys node */

/* echo gpio_idx dirction state > aw9523b_gpio */
static ssize_t aw9523b_gpio_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t len)
{
	unsigned int databuf[3];
	int i = 0;
	struct aw9523b *p_aw9523b = dev_get_drvdata(dev);
	struct aw9523b_gpio *p_gpio_data = p_aw9523b->gpio_data;
	struct aw9523b_singel_gpio *p_single_gpio_data = p_gpio_data->single_gpio_data;

	sscanf(buf, "%d %d %d", &databuf[0], &databuf[1], &databuf[2]);
	AW_DEBUG("aw9523b gpio cmd param: %d %d %d\n", databuf[0], databuf[1], databuf[2]);
	if (p_gpio_data->gpio_mask & (0x01 << databuf[0])) {
		for (i = 0; i < p_gpio_data->gpio_num; i++) {
			if (p_single_gpio_data[i].gpio_idx == databuf[0]) {
				if (p_single_gpio_data[i].gpio_direction != databuf[1]) {
					p_single_gpio_data[i].gpio_direction = databuf[1];
					aw9523b_set_port_direction_by_mask(p_aw9523b, 0x1 << p_single_gpio_data[i].gpio_idx,
								  p_single_gpio_data[i].gpio_direction);
				}
				if (p_single_gpio_data[i].gpio_direction == 0x01) { /* output */
					if (p_single_gpio_data[i].state != databuf[2]) {
						p_single_gpio_data[i].state = databuf[2];
						aw9523b_set_port_output_state_by_mask(p_aw9523b,
										 0x1 << p_single_gpio_data[i].gpio_idx,
										 p_single_gpio_data[i].state);
					}
				}
			}
		}
	}
	return len;
}

static ssize_t aw9523b_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	struct aw9523b *p_aw9523b = dev_get_drvdata(dev);
	struct aw9523b_singel_gpio *p_single_gpio_data = p_aw9523b->gpio_data->single_gpio_data;

	len += snprintf(buf+len, PAGE_SIZE-len, "Uasge: echo gpio_idx dirction state > aw9523b_gpio\n");
	for (i = 0; i < p_aw9523b->gpio_data->gpio_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "aw9523b gpio idx = %d, dir = %d, state = %d",
				p_single_gpio_data[i].gpio_idx,
				p_single_gpio_data[i].gpio_direction,
				p_single_gpio_data[i].state);
	}
	return len;
}


static DEVICE_ATTR(aw9523b_gpio, S_IWUSR | S_IRUGO, aw9523b_gpio_show, aw9523b_gpio_store);
static struct attribute *aw9523b_gpio_attributes[] = {
	 &dev_attr_aw9523b_gpio.attr,
	 NULL
};
static struct attribute_group aw9523b_gpio_attribute_group = {
	.attrs = aw9523b_gpio_attributes
};

static int aw9523b_parse_for_single_gpio(struct device_node *gpio_node,
					 struct aw9523b *p_aw9523b,
					 struct aw9523b_gpio *p_gpio_data)
{
	int ret = 0;
	struct device_node *temp = NULL;
	int i = 0;
	struct aw9523b_singel_gpio *p_single_gpio_data = p_gpio_data->single_gpio_data;

	if ((p_gpio_data == NULL) || (p_gpio_data->single_gpio_data == NULL))
		return -EINVAL;
	 for_each_child_of_node(gpio_node, temp) {
		ret = of_property_read_u32(temp, "aw9523b,gpio_idx", &p_single_gpio_data[i].gpio_idx);
		if (ret < 0) {
			dev_err(p_aw9523b->dev, "%s: no aw9523b,gpio_idx, abort\n", __func__);
			goto err_id;
		}
		ret = of_property_read_u32(temp, "aw9523b,gpio_dir", &p_single_gpio_data[i].gpio_direction);
		if (ret < 0) {
			dev_err(p_aw9523b->dev, "%s, no aw9523b,gpio_dir\n", __func__);
			goto err_id;
		}
		ret = of_property_read_u32(temp, "aw9523b,gpio_default_val", &p_single_gpio_data[i].state);
		if (ret < 0) {
			dev_err(p_aw9523b->dev, "%s, no aw9523b,gpio_default_val, abort\n", __func__);
			goto err_id;
		}
		p_gpio_data->gpio_mask |= 0x01 << p_single_gpio_data[i].gpio_idx;
		AW_DEBUG("idx = %d, aw9523b,gpio_idx = %d\n", i, p_single_gpio_data[i].gpio_idx);
		i++;
	}
	AW_DEBUG("%s gpio_mask = 0x%x\r\n", __func__, p_gpio_data->gpio_mask);
	return 0;
err_id:
	return ret;
}

static void aw9523b_gpio_chip_init(struct aw9523b *p_aw9523b, struct aw9523b_gpio *p_gpio_data)
{
	int i = 0;
	struct aw9523b_singel_gpio *p_single_gpio_data = p_gpio_data->single_gpio_data;

	aw9523b_set_port_mode_by_mask(p_aw9523b, p_gpio_data->gpio_mask, 1); /* gpio mode */
	aw9523b_set_port_output_mode(p_aw9523b, p_gpio_data->output_mode); /* OD or pull push mode */
	for (i = 0; i < p_gpio_data->gpio_num; i++) {
		aw9523b_set_port_direction_by_mask(p_aw9523b,
					0x1 << p_single_gpio_data[i].gpio_idx,
					p_single_gpio_data[i].gpio_direction);
		if (p_single_gpio_data[i].gpio_direction) { /* output */
			aw9523b_set_port_output_state_by_mask(p_aw9523b,
					0x1<<p_single_gpio_data[i].gpio_idx,
					p_single_gpio_data[i].state);
		}
	}
}

static void aw9523b_gpio_free_all_resource(struct aw9523b *p_aw9523b)
{
	if (p_aw9523b->gpio_feature_enable) {
		devm_kfree(p_aw9523b->dev, p_aw9523b->gpio_data->single_gpio_data);
		devm_kfree(p_aw9523b->dev, p_aw9523b->gpio_data);
	}
}

static int aw9523b_gpio_feature_init(struct aw9523b *p_aw9523b)
{
	int ret = 0;
	int i = 0;
	struct device_node *gpio_node = NULL;
	struct aw9523b_gpio *p_gpio_data = NULL;
	int gpio_num = 0;

	p_gpio_data = devm_kzalloc(p_aw9523b->dev, sizeof(struct aw9523b_gpio),
						  GFP_KERNEL);
	if (p_gpio_data == NULL) {
		dev_err(p_aw9523b->dev, "%s: malloc memory failed\r\n", __func__);
		return -ENOMEM;
	}
	p_aw9523b->gpio_data = p_gpio_data;

	gpio_node = of_find_node_by_name(p_aw9523b->dev->of_node, "aw9523b,gpio");
	if (gpio_node == NULL) {
		dev_err(p_aw9523b->dev,
				"%s: can't find aw9523b,gpio return failed\r\n",
				__func__);
		ret = -1;
		goto err_id;
	}
	ret = of_property_read_u32(gpio_node, "aw9523b,gpio_mode",
					      &p_gpio_data->output_mode);
	if (ret < 0) {
		dev_err(p_aw9523b->dev, "%s: no aw9523b,gpio_mode, abort\n", __func__);
		goto err_id;
	}

	gpio_num = of_get_child_count(gpio_node);
	p_gpio_data->gpio_num = gpio_num;
	p_gpio_data->single_gpio_data = devm_kzalloc(p_aw9523b->dev,
					sizeof(struct aw9523b_singel_gpio) * gpio_num,
					GFP_KERNEL);
	if (p_gpio_data->single_gpio_data == NULL) {
		dev_err(p_aw9523b->dev, "%s: malloc memory failed\r\n", __func__);
		ret = -ENOMEM;
		goto err_id;
	}
	ret = aw9523b_parse_for_single_gpio(gpio_node, p_aw9523b, p_gpio_data);
	if (ret) {
		dev_err(p_aw9523b->dev, "aw9523b_parse_single_led failed\r\n");
		goto free_mem;
	}
	for (i = 0; i < gpio_num; i++)
		p_gpio_data->single_gpio_data[i].priv = p_aw9523b;
	ret = sysfs_create_group(&p_aw9523b->dev->kobj,
			&aw9523b_gpio_attribute_group);
	if (ret) {
		dev_err(p_aw9523b->dev, "led sysfs failed ret: %d\n", ret);
		goto free_mem;
	}
	aw9523b_gpio_chip_init(p_aw9523b, p_gpio_data);
	return 0;
free_mem:
	devm_kfree(p_aw9523b->dev, p_gpio_data->single_gpio_data);
	devm_kfree(p_aw9523b->dev, p_aw9523b->gpio_data);
err_id:
	return ret;

}

static void aw9523b_set_gpio_low_output(struct aw9523b *p_aw9523b)
{
	i2c_write_byte(p_aw9523b->client, P0_OUTPUT, 0);
	i2c_write_byte(p_aw9523b->client, P1_OUTPUT, 0);
}

/*********************************************************
 *
 * aw9523b driver
 *
 ********************************************************/
static int aw9523b_key_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct aw9523b *p_aw9523b = NULL;
	int ret = 0;

	AW_DEBUG("%s enter, %s\n", __func__, AW9523B_DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C transfer not Supported\n");
		return -EIO;
	}

	p_aw9523b = devm_kzalloc(&client->dev, sizeof(struct aw9523b), GFP_KERNEL);
	if (!p_aw9523b) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	p_aw9523b->dev = &client->dev;
	p_aw9523b->client = client;
	i2c_set_clientdata(client, p_aw9523b);

#if 0
	/* power supply */
	p_aw9523b->power_supply = regulator_get(&client->dev, "aw9523b_power");
	if (IS_ERR(p_aw9523b->power_supply)) {
		ret = PTR_ERR(p_aw9523b->power_supply);
		dev_err(&client->dev, "Regulator get failed ret=%d\n", ret);
		goto err_free_mem;
	}

	ret = regulator_enable(p_aw9523b->power_supply);
	if (ret) {
		dev_err(&client->dev, "Regulator power_supply enable failed ret=%d\n", ret);
		ret = regulator_disable(p_aw9523b->power_supply);
		goto err_free_mem;
	} else {
		dev_info(&client->dev, "Regulator power_supply enable success!\n");
	}
#endif
	ret = aw9523b_reset(p_aw9523b);
	if (ret)
		goto err_free_mem;
	if (aw9523b_read_chipid(client)) {
		dev_err(&client->dev, "%s: read_chipid error\n", __func__);
		goto err_free_rst;
	}

	aw9523b_set_gpio_low_output(p_aw9523b);

	/* create debug dev node - reg */
	aw9523b_create_sysfs(client);

	aw9523b_parse_dt_for_feature_enable(p_aw9523b);

	if (p_aw9523b->single_key_enable) {
		/* single key init */
		ret = aw9523b_key_feature_init(p_aw9523b);
		if (ret) {
			dev_err(p_aw9523b->dev, "aw9523b single key feature init failed \r\n");
			goto err_free_rst;
		}
	}

	if (p_aw9523b->matrix_key_enable) {
		/* key init */
		ret = aw9523b_key_feature_init(p_aw9523b);
		if (ret) {
			dev_err(p_aw9523b->dev, "aw9523b key feature init failed \r\n");
			goto err_free_rst;
		}
	}

	if (p_aw9523b->led_feature_enable) {
		/* led init */
		ret = aw9523b_led_feature_init(p_aw9523b);
		if (ret) {
			dev_err(p_aw9523b->dev, "aw9523b led feature init failed \r\n");
			goto free_key;
		}
	}

	if (p_aw9523b->gpio_feature_enable) {
		/* gpio init */
		ret = aw9523b_gpio_feature_init(p_aw9523b);
		if (ret) {
			dev_err(p_aw9523b->dev, "aw9523b gpio feature init failed \r\n");
			goto free_led;
		}
	}
	p_aw9523b->screen_state = true;

	//msx-add fb-suspend
	//p_aw9523b->fb_notif.notifier_call = fb_notifier_callback;
    //ret = fb_register_client(&p_aw9523b->fb_notif);
	#if 0
	ret = fb_register_client(&aw9523b_fb_notifier);
    if (ret) {
        AW_DEBUG("[FB]Unable to register fb_notifier: %d\n", ret);
    }
	#endif

	p_aw9523b->disp_notifier.notifier_call = aw9523b_drm_notifier_callback;
	if (mtk_disp_notifier_register("aw9523b_drm", &p_aw9523b->disp_notifier))
		AW_DEBUG("Failed to register aw9523b_fb disp notifier client");			
	//end

	AW_DEBUG("aw9523b_key_i2c_probe success!\n");
	return 0;
free_led:
	aw9523b_led_free_all_resource(p_aw9523b);
free_key:
	aw9523b_key_free_all_resource(p_aw9523b);
err_free_rst:
	gpio_free(p_aw9523b->rst_gpio);
err_free_mem:
	devm_kfree(&client->dev, p_aw9523b);
	AW_DEBUG("aw9523b_key_i2c_probe fail!\n");
	return ret;
}

static int aw9523b_key_i2c_remove(struct i2c_client *client)
{
	struct aw9523b *p_aw9523b = i2c_get_clientdata(client);

	aw9523b_key_free_all_resource(p_aw9523b);
	aw9523b_led_free_all_resource(p_aw9523b);
	aw9523b_gpio_free_all_resource(p_aw9523b);

	//msx-add fb-suspend
	//fb_unregister_client(&aw9523b_fb_notifier);
	mtk_disp_notifier_unregister(&p_aw9523b->disp_notifier);
	//end

	devm_kfree(p_aw9523b->dev, p_aw9523b);
	return 0;
}

static int aw9523b_suspend(struct device *dev)
{
	struct aw9523b *p_aw9523b = dev_get_drvdata(dev);
	struct aw9523b_key *p_key_data = p_aw9523b->key_data;

	AW_DEBUG("%s enter\n", __func__);
	if (!p_aw9523b) {
		AW_DEBUG("%s p_aw9523b is null\n", __func__);
		return -ENOMEM;
	}
	p_aw9523b->screen_state = false;

	if (!p_key_data) {
		AW_DEBUG("%s p_key_data is null\n", __func__);
		return -ENOMEM;
	}
	aw9523b_key_chip_init(p_key_data);

	//gpio_set_value(p_aw9523b->rst_gpio, 0);
	//mdelay(3);

	return 0;
}

static int aw9523b_resume(struct device *dev)
{
	struct aw9523b *p_aw9523b = dev_get_drvdata(dev);
	struct aw9523b_key *p_key_data = p_aw9523b->key_data;
	int i = 0;

	AW_DEBUG("%s enter\n", __func__);

	/*
	gpio_set_value(p_aw9523b->rst_gpio, 0);
	mdelay(1);
	gpio_set_value(p_aw9523b->rst_gpio, 1);
	mdelay(1);
	gpio_set_value(p_aw9523b->rst_gpio, 0);
	mdelay(1);
	gpio_set_value(p_aw9523b->rst_gpio, 1);
	mdelay(10);
	*/


	if (!p_aw9523b) {
		AW_DEBUG("%s p_aw9523b is null\n", __func__);
		return -ENOMEM;
	}

	p_aw9523b->screen_state = true;

	if (!p_key_data) {
		AW_DEBUG("%s p_key_data is null\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < p_key_data->output_port_nums; i++) {
		p_key_data->new_output_state[i] = p_key_data->input_port_mask;
		p_key_data->old_output_state[i] = p_key_data->input_port_mask;
		p_key_data->def_output_state[i] = p_key_data->input_port_mask;
	}
	
	// kcm added by phf, 20250123
	p_key_data->old_key_row_status = p_key_data->input_port_mask;
	p_key_data->old_key_col_status = p_key_data->output_port_mask;

	aw9523b_key_chip_init(p_key_data);

	return 0;
}

static const struct dev_pm_ops aw9523b_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(aw9523b_suspend, aw9523b_resume)
};

static const struct of_device_id aw9523b_keypad_of_match[] = {
	{ .compatible = "awinic,aw9523b",},
	{},
};

static const struct i2c_device_id aw9523b_key_i2c_id[] = {
	{"awinic,aw9523b", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, aw9523b_key_i2c_id);

static struct i2c_driver aw9523b_key_i2c_driver = {
	.driver = {
		.name = "aw9523b-key",
		.owner = THIS_MODULE,
		.of_match_table = aw9523b_keypad_of_match,
		.pm = &aw9523b_pm_ops,
	},
	.probe = aw9523b_key_i2c_probe,
	.remove = aw9523b_key_i2c_remove,
	.id_table = aw9523b_key_i2c_id,
};

static int __init aw9523b_key_i2c_init(void)
{
	int ret = 0;

	AW_DEBUG("%s enter\n", __func__);

	ret = i2c_add_driver(&aw9523b_key_i2c_driver);
	if (ret) {
		pr_err("fail to add aw9523b device into i2c\n");
		return ret;
	}

	AW_DEBUG("%s exit\n", __func__);

	return 0;
}
module_init(aw9523b_key_i2c_init);

static void __exit aw9523b_key_i2c_exit(void)
{
	AW_DEBUG("%s enter\n", __func__);

	i2c_del_driver(&aw9523b_key_i2c_driver);

	AW_DEBUG("%s exit\n", __func__);
}
module_exit(aw9523b_key_i2c_exit);

MODULE_AUTHOR("shiqiang@awinic.com.cn");
MODULE_DESCRIPTION("aw9523b driver");
MODULE_LICENSE("GPL");
