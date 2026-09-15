// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef OCP81375_DTNAME_I2C
#define OCP81375_DTNAME_I2C "mediatek,strobe_main"
#endif
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef OCP81375_DTNAME
#define OCP81375_DTNAME "mediatek,flashlights_ocp81375"
#endif

#define OCP81375_NAME "flashlights-ocp81375"

/* define registers */
#define OCP81375_REG_ENABLE (0x01)
#define OCP81375_REG_FLASH_LEVEL_LED1 (0x03)
#define OCP81375_REG_TORCH_LEVEL_LED1 (0x04)
#define OCP81375_REG_FLAG1 (0x05)
#define OCP81375_REG_FLASH_LEVEL_LED2 (0x04)
#define OCP81375_REG_TORCH_LEVEL_LED2 (0x06)
#define OCP81375_REG_FLAG2 (0x0B)
#define OCP81375_REG_BOOST_CONFIG	(0x07)
#define OCP81375_REG_TIMING_CONFIG	(0x08)
#define OCP81375_ENABLE_LED1_TORCH (0x02)
#define OCP81375_ENABLE_LED1_FLASH (0x03)
#define OCP81375_ENABLE_LED1 (0x04)



#define OCP81375_MASK_ENABLE_LED1 (0x01)
#define OCP81375_MASK_ENABLE_LED2 (0x02)
#define OCP81375_DISABLE (0x00)
#define OCP81375_ENABLE_LED2 (0x02)
#define OCP81375_ENABLE_LED2_TORCH (0x0A)
#define OCP81375_ENABLE_LED2_FLASH (0x0E)
#define OCP81375_REG_TIMING_CONF (0x08)
#define OCP81375_TORCH_RAMP_TIME (0x00)
#define OCP81375_FLASH_TIMEOUT   (0x0F)


/* define channel, level */
#define OCP81375_CHANNEL_NUM 2
#define OCP81375_CHANNEL_CH1 0
#define OCP81375_CHANNEL_CH2 1

#define OCP81375_LEVEL_NUM 128
#define OCP81375_LEVEL_TORCH 52

#define OCP81375_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(ocp81375_mutex);
static struct work_struct ocp81375_work_ch1;
static struct work_struct ocp81375_work_ch2;

/* define pinctrl */
#define OCP81375_PINCTRL_PIN_HWEN 0
#define OCP81375_PINCTRL_PINSTATE_LOW 0
#define OCP81375_PINCTRL_PINSTATE_HIGH 1
#define OCP81375_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define OCP81375_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *ocp81375_pinctrl;
static struct pinctrl_state *ocp81375_hwen_high;
static struct pinctrl_state *ocp81375_hwen_low;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *ocp81375_i2c_client;

/* platform data */
struct ocp81375_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* ocp81375 chip data */
struct ocp81375_chip_data {
	struct i2c_client *client;
	struct ocp81375_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int ocp81375_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	ocp81375_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ocp81375_pinctrl)) {
		printk("ocp81375 Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(ocp81375_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	ocp81375_hwen_high = pinctrl_lookup_state(
			ocp81375_pinctrl, OCP81375_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(ocp81375_hwen_high)) {
		printk("ocp81375 Failed to init (%s)\n",
			OCP81375_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(ocp81375_hwen_high);
	}
	ocp81375_hwen_low = pinctrl_lookup_state(
			ocp81375_pinctrl, OCP81375_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(ocp81375_hwen_low)) {
		printk("ocp81375 Failed to init (%s)\n", OCP81375_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(ocp81375_hwen_low);
	}

	return ret;
}

static int ocp81375_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(ocp81375_pinctrl)) {
		pr_info("ocp81375 pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case OCP81375_PINCTRL_PIN_HWEN:
		if (state == OCP81375_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(ocp81375_hwen_low))
			pinctrl_select_state(ocp81375_pinctrl, ocp81375_hwen_low);
		else if (state == OCP81375_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(ocp81375_hwen_high))
			pinctrl_select_state(ocp81375_pinctrl, ocp81375_hwen_high);
		else
			pr_info("ocp81375 set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("ocp81375 set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("ocp81375 pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * ocp81375 operations
 *****************************************************************************/
#if 0
static const int ocp81375_current[OCP81375_LEVEL_NUM] = {
	 2,  63,  184,  276,  297, 368, 450, 460, 480, 490,
	500, 510, 520, 530,  550, 597, 656, 703, 750, 796,
	855, 1001, 1101, 1201, 1301, 1480
};
#endif

static const unsigned char ocp81375_torch_level[OCP81375_LEVEL_NUM] = {
	0x00, 0x2B, 0x7F, 0xBF, 0xCD, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char ocp81375_flash_level[OCP81375_LEVEL_NUM] = {
	0x00, 0x15, 0x3F, 0x57, 0x66, 0x7F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static unsigned char ocp81375_reg_enable;
static int ocp81375_level_ch1 = -1;
static int ocp81375_level_ch2 = -1;

static int ocp81375_is_torch(int level)
{
	if (level >= OCP81375_LEVEL_TORCH)
		return -1;

	return 0;
}

static int ocp81375_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= OCP81375_LEVEL_NUM)
		level = OCP81375_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int ocp81375_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct ocp81375_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		printk("ocp81375 failed writing at 0x%02x\n", reg);

	return ret;
}

static int ocp81375_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct ocp81375_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

/* flashlight enable function */
static int ocp81375_enable_ch1(void)
{
	//unsigned char reg, val;
	unsigned char reg;

	reg = OCP81375_REG_ENABLE;
	printk("ocp81375  ocp81375_enable_ch1 enter\n");
	if (!ocp81375_is_torch(ocp81375_level_ch1)) {
		/* torch mode */
		printk("ocp81375  ocp81375_enable_ch1 torch\n");

		//ocp81375_write_reg(ocp81375_i2c_client,
		//	OCP81375_REG_TORCH_LEVEL_LED1,0xCD);
		ocp81375_write_reg(ocp81375_i2c_client,
					OCP81375_REG_ENABLE, 0x06);	

	} else {
		/* flash mode */
		printk("ocp81375  ocp81375_enable_ch1 flash\n");

		//ocp81375_write_reg(ocp81375_i2c_client,
		//	OCP81375_REG_FLASH_LEVEL_LED1,0x3f);
		ocp81375_write_reg(ocp81375_i2c_client,
			OCP81375_REG_ENABLE, 0x07);
	}

	return 0;
}

static int ocp81375_enable_ch2(void)
{
	unsigned char reg, val;

	reg = OCP81375_REG_ENABLE;
	if (!ocp81375_is_torch(ocp81375_level_ch2)) {
		/* torch mode */
		ocp81375_reg_enable |= OCP81375_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		ocp81375_reg_enable |= OCP81375_ENABLE_LED2_FLASH;
	}
	val = ocp81375_reg_enable;

	return ocp81375_write_reg(ocp81375_i2c_client, reg, val);
}

static int ocp81375_enable(int channel)
{
	printk("ocp81375  ocp81375_enable enter\n");
	if (channel == OCP81375_CHANNEL_CH1)
	{
		printk("ocp81375  ocp81375_enable_ch1\n");
		ocp81375_enable_ch1();
	}
	else if (channel == OCP81375_CHANNEL_CH2)
	{
		printk("ocp81375  ocp81375_enable_ch2\n");
		ocp81375_enable_ch2();
	}
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int ocp81375_disable_ch1(void)
{
	/*
	unsigned char reg, val;

	reg = OCP81375_REG_ENABLE;
	if (ocp81375_reg_enable & OCP81375_MASK_ENABLE_LED2) {
		// if LED 2 is enable, disable LED 1 
		ocp81375_reg_enable &= (~OCP81375_ENABLE_LED1);
	} else {
		// if LED 2 is disable, disable LED 1 and clear mode 
		ocp81375_reg_enable &= (~OCP81375_ENABLE_LED1_FLASH);
	}
	val = ocp81375_reg_enable;

	return ocp81375_write_reg(ocp81375_i2c_client, reg, val);
	*/

	
	printk("ocp81375_disable_ch1  222222  enter\n");

	ocp81375_write_reg(ocp81375_i2c_client,
					OCP81375_REG_ENABLE, 0x00);	
	return 0;
}

static int ocp81375_disable_ch2(void)
{
	unsigned char reg, val;

	reg = OCP81375_REG_ENABLE;
	if (ocp81375_reg_enable & OCP81375_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		ocp81375_reg_enable &= (~OCP81375_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		ocp81375_reg_enable &= (~OCP81375_ENABLE_LED2_FLASH);
	}
	val = ocp81375_reg_enable;

	return ocp81375_write_reg(ocp81375_i2c_client, reg, val);
}

static int ocp81375_disable(int channel)
{
	printk("ocp81375_disable enter\n");
	if (channel == OCP81375_CHANNEL_CH1)
	{
		printk("ocp81375_disable_ch 11111 enter\n");
		ocp81375_disable_ch1();
	}
	else if (channel == OCP81375_CHANNEL_CH2)
	{
		printk("ocp81375_disable_ch2 enter\n");
		ocp81375_disable_ch2();
	}
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int ocp81375_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;
	int current_val = 0;
	int current_level = 0;

	printk("moshaoxi ocp81375_set_level_ch1 enter,level is %d\n",level);

	level = ocp81375_verify_level(level);

	//current_val = 1.4 * level;
	current_val = 3 * level;

	/* set torch brightness level */
	reg = OCP81375_REG_TORCH_LEVEL_LED1;			//(0x04)

	//torch
	//if(current_val<63)
	if(current_val<126)
		current_level = 1;	
	//else if(current_val<184 && current_val>=63)
	else if(current_val<368 && current_val>=126)
		current_level = 2;
	//else if(current_val<276 && current_val>=184)
	else if(current_val<552 && current_val>=368)
		current_level = 3;	
	//else if(current_val<297 && current_val>=276)
	else if(current_val<594 && current_val>=552)
		current_level = 4;		
	//else if(current_val<368 && current_val>=297)
	else if(current_val<736 && current_val>=594)
		current_level = 5;	

	printk("moshaoxi ocp81375_verify_level ,final level is %d\n",current_level);		

	val = ocp81375_torch_level[current_level];

	printk("moshaoxi ocp81375_torch_level ,val is %x\n",val);
	//val = 0xCD;

	ret = ocp81375_write_reg(ocp81375_i2c_client, reg, val);


	//flash

	current_val = 7 * level;

	if(current_val<257)
		current_level = 1;
	else if(current_val<750 && current_val>=257)
		current_level = 2;
	else if(current_val<1030 && current_val>=750)
		current_level = 3;	
	else if(current_val<1200 && current_val>=1030)
		current_level = 4;		
	else if(current_val<1500 && current_val>=1200)
		current_level = 5;	


	ocp81375_level_ch1 = level;

	/* set flash brightness level */
	reg = OCP81375_REG_FLASH_LEVEL_LED1;			//(0x03)

	printk("moshaoxi OCP81375_REG_FLASH_LEVEL_LED1 ,final level is %d\n",current_level);

	val = ocp81375_flash_level[current_level];
	//val = 0x3f;

	printk("moshaoxi ocp81375_flash_level ,val is %x\n",val);

	ret = ocp81375_write_reg(ocp81375_i2c_client, reg, val);

	return ret;
}

static int ocp81375_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = ocp81375_verify_level(level);

	/* set torch brightness level */
	reg = OCP81375_REG_TORCH_LEVEL_LED2;
	val = ocp81375_torch_level[level];
	ret = ocp81375_write_reg(ocp81375_i2c_client, reg, val);

	ocp81375_level_ch2 = level;

	/* set flash brightness level */
	reg = OCP81375_REG_FLASH_LEVEL_LED2;
	val = ocp81375_flash_level[level];
	ret = ocp81375_write_reg(ocp81375_i2c_client, reg, val);

	return ret;
}

static int ocp81375_set_level(int channel, int level)
{
	if (channel == OCP81375_CHANNEL_CH1)
		ocp81375_set_level_ch1(level);
	else if (channel == OCP81375_CHANNEL_CH2)
		ocp81375_set_level_ch2(level);
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

static int ocp81375_get_flag(int num)
{
	if (num == 1)
		return ocp81375_read_reg(ocp81375_i2c_client, OCP81375_REG_FLAG1);
	else if (num == 2)
		return ocp81375_read_reg(ocp81375_i2c_client, OCP81375_REG_FLAG2);

	pr_info("Error num\n");
	return 0;
}

/* flashlight init */
int ocp81375_init(void)
{
	int ret;
	unsigned char reg, val;

	ocp81375_pinctrl_set(
			OCP81375_PINCTRL_PIN_HWEN, OCP81375_PINCTRL_PINSTATE_HIGH);
	msleep(20);

	/* clear enable register */
	reg = OCP81375_REG_ENABLE;
	val = OCP81375_DISABLE;
	ret = ocp81375_write_reg(ocp81375_i2c_client, reg, val);

	ocp81375_reg_enable = val;


	// set torch current ramp time and flash timeout 
	//reg = OCP81375_REG_TIMING_CONFIG;
	//val = OCP81375_TORCH_RAMP_TIME | OCP81375_FLASH_TIMEOUT;
	//ret = ocp81375_write_reg(ocp81375_i2c_client, reg, val);


	//ocp81375_write_reg(ocp81375_i2c_client, OCP81375_REG_BOOST_CONFIG, 0x09);
	//ocp81375_write_reg(ocp81375_i2c_client, OCP81375_REG_TIMING_CONFIG, 0x1f);
	

	return ret;
}

/* flashlight uninit */
int ocp81375_uninit(void)
{
	ocp81375_disable(OCP81375_CHANNEL_CH1);
	ocp81375_disable(OCP81375_CHANNEL_CH2);
	ocp81375_pinctrl_set(
			OCP81375_PINCTRL_PIN_HWEN, OCP81375_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer ocp81375_timer_ch1;
static struct hrtimer ocp81375_timer_ch2;
static unsigned int ocp81375_timeout_ms[OCP81375_CHANNEL_NUM];

static void ocp81375_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ocp81375 ht work queue callback\n");
	ocp81375_disable_ch1();
}

static void ocp81375_work_disable_ch2(struct work_struct *data)
{
	pr_debug("ocp81375 lt work queue callback\n");
	ocp81375_disable_ch2();
}

static enum hrtimer_restart ocp81375_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&ocp81375_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart ocp81375_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&ocp81375_work_ch2);
	return HRTIMER_NORESTART;
}


static int ocp81375_timer_start(int channel, ktime_t ktime)
{
	if (channel == OCP81375_CHANNEL_CH1)
		hrtimer_start(&ocp81375_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == OCP81375_CHANNEL_CH2)
		hrtimer_start(&ocp81375_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}


static int ocp81375_timer_cancel(int channel)
{
	if (channel == OCP81375_CHANNEL_CH1)
		hrtimer_cancel(&ocp81375_timer_ch1);
	else if (channel == OCP81375_CHANNEL_CH2)
		hrtimer_cancel(&ocp81375_timer_ch2);
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int ocp81375_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= OCP81375_CHANNEL_NUM) {
		printk("ocp81375 Failed with error channel\n");
		return -EINVAL;
	}

	printk("ocp81375 ocp81375_ioctl enter\n");

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("ocp81375 FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81375_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		printk("ocp81375 FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81375_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("ocp81375 FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			
			if (ocp81375_timeout_ms[channel]) {
				s = ocp81375_timeout_ms[channel] / 1000;
				ns = ocp81375_timeout_ms[channel] % 1000
					* 1000000;
				ktime = ktime_set(s, ns);
				ocp81375_timer_start(channel, ktime);
			}
			ocp81375_enable(channel);
		} else {
			ocp81375_disable(channel);
			ocp81375_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		printk("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = OCP81375_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		printk("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = OCP81375_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = ocp81375_verify_level(fl_arg->arg);
		printk("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		//fl_arg->arg = ocp81375_current[fl_arg->arg];
		fl_arg->arg = 7 * (int)fl_arg->arg;
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		printk("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = OCP81375_HW_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		printk("FLASH_IOC_GET_HW_FAULT(%d)\n", channel);
		fl_arg->arg = ocp81375_get_flag(1);
		break;

	case FLASH_IOC_GET_HW_FAULT2:
		printk("FLASH_IOC_GET_HW_FAULT2(%d)\n", channel);
		fl_arg->arg = ocp81375_get_flag(2);
		break;

	default:
		printk("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int ocp81375_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int ocp81375_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int ocp81375_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&ocp81375_mutex);
	if (set) {
		if (!use_count)
			ret = ocp81375_init();
		use_count++;
		printk("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = ocp81375_uninit();
		if (use_count < 0)
			use_count = 0;
		printk("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&ocp81375_mutex);

	return ret;
}

static ssize_t ocp81375_strobe_store(struct flashlight_arg arg)
{

	printk("ocp81375_strobe_store enter\n");
	ocp81375_set_driver(1);
	ocp81375_set_level(arg.channel, arg.level);
	ocp81375_timeout_ms[arg.channel] = 0;
	ocp81375_enable(arg.channel);
	msleep(arg.dur);
	ocp81375_disable(arg.channel);
	ocp81375_set_driver(0);

	return 0;
}

static struct flashlight_operations ocp81375_ops = {
	ocp81375_open,
	ocp81375_release,
	ocp81375_ioctl,
	ocp81375_strobe_store,
	ocp81375_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int ocp81375_chip_init(struct ocp81375_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * ocp81375_init();
	 */

	return 0;
}

static int ocp81375_parse_dt(struct device *dev,
		struct ocp81375_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		printk("moshaoxi Parse no dt, node.\n");
		return 0;
	}
	printk("moshaoxi Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		printk("moshaoxi Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				OCP81375_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		printk("moshaoxi Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int ocp81375_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ocp81375_chip_data *chip;
	int err;

	printk("moshaoxi i2c probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct ocp81375_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	client->addr = 0x64;
	chip->client = client;

	i2c_set_clientdata(client, chip);
	ocp81375_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	ocp81375_chip_init(chip);

	printk("moshaoxi i2c probe done.\n");

	return 0;

err_out:
	return err;
}

static int ocp81375_i2c_remove(struct i2c_client *client)
{
	struct ocp81375_chip_data *chip = i2c_get_clientdata(client);

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id ocp81375_i2c_id[] = {
	{OCP81375_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id ocp81375_i2c_of_match[] = {
	{.compatible = OCP81375_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver ocp81375_i2c_driver = {
	.driver = {
		.name = OCP81375_NAME,
#ifdef CONFIG_OF
		.of_match_table = ocp81375_i2c_of_match,
#endif
	},
	.probe = ocp81375_i2c_probe,
	.remove = ocp81375_i2c_remove,
	.id_table = ocp81375_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int ocp81375_probe(struct platform_device *pdev)
{
	struct ocp81375_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct ocp81375_chip_data *chip = NULL;
	int err;
	int i;

	printk("ocp81375 Probe start.\n");

	/* init pinctrl */
	if (ocp81375_pinctrl_init(pdev)) {
		printk("ocp81375 Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&ocp81375_i2c_driver)) {
		printk("ocp81375 Failed to add i2c driver.\n");
		return -1;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		pdev->dev.platform_data = pdata;
		err = ocp81375_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_free;
	}

	/* init work queue */
	INIT_WORK(&ocp81375_work_ch1, ocp81375_work_disable_ch1);
	INIT_WORK(&ocp81375_work_ch2, ocp81375_work_disable_ch2);

	/* init timer */
	hrtimer_init(&ocp81375_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ocp81375_timer_ch1.function = ocp81375_timer_func_ch1;
	hrtimer_init(&ocp81375_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ocp81375_timer_ch2.function = ocp81375_timer_func_ch2;
	ocp81375_timeout_ms[OCP81375_CHANNEL_CH1] = 100;
	ocp81375_timeout_ms[OCP81375_CHANNEL_CH2] = 100;

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
					&pdata->dev_id[i],
					&ocp81375_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(OCP81375_NAME, &ocp81375_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	printk("ocp81375 Probe done.\n");

	return 0;
err_free:
	chip = i2c_get_clientdata(ocp81375_i2c_client);
	i2c_set_clientdata(ocp81375_i2c_client, NULL);
	kfree(chip);
	return err;
}

static int ocp81375_remove(struct platform_device *pdev)
{
	struct ocp81375_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	i2c_del_driver(&ocp81375_i2c_driver);

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(OCP81375_NAME);

	/* flush work queue */
	flush_work(&ocp81375_work_ch1);
	flush_work(&ocp81375_work_ch2);

	pr_debug("Remove done.\n");

	return 0;
}

/*
#ifdef CONFIG_OF
static const struct of_device_id ocp81375_of_match[] = {
	{.compatible = OCP81375_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, ocp81375_of_match);
#else
static struct platform_device ocp81375_platform_device[] = {
	{
		.name = OCP81375_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, ocp81375_platform_device);
#endif
*/


static const struct of_device_id ocp81375_of_match[] = {
	{.compatible = OCP81375_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, ocp81375_of_match);


static struct platform_driver ocp81375_platform_driver = {
	.probe = ocp81375_probe,
	.remove = ocp81375_remove,
	.driver = {
		.name = OCP81375_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ocp81375_of_match,
	},
};

static int __init flashlight_ocp81375_init(void)
{
	int ret;

	printk("moshaoxi flashlight_ocp81375_init Init start.\n");

#ifndef CONFIG_OF
	printk("moshaoxi ifndef CONFIG_OF enter\n");
	ret = platform_device_register(&ocp81375_platform_device);
	if (ret) {
		printk("moshaoxi Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&ocp81375_platform_driver);
	if (ret) {
		printk("moshaoxi Failed to register platform driver\n");
		return ret;
	}

	printk("moshaoxi flashlight_ocp81375_init done.\n");

	return 0;
}

static void __exit flashlight_ocp81375_exit(void)
{
	printk("Exit start.\n");

	platform_driver_unregister(&ocp81375_platform_driver);

	printk("Exit done.\n");
}

module_init(flashlight_ocp81375_init);
module_exit(flashlight_ocp81375_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight OCP81375 Driver");

