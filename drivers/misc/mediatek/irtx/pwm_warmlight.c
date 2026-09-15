// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <media/rc-core.h>
#include <linux/leds.h>
#include <mt-plat/mtk_pwm.h>
#include <mt-plat/mtk_pwm_hal.h>

#define DRIVER_NAME	"mtk-pwm-warmlight"
void mt_warmlight_set_pwm(u32 level);

int maxlevel = 255;			//最大亮度等级

struct warmlight_led {
	struct led_classdev cdev;
	unsigned int cur_brightness;
	unsigned int max_brightness;
	//unsigned int idx;
	//struct aw9523b *priv;
	//struct work_struct brightness_work;
};

char *led_name[1] = {
	"led-warmlight",
};

struct warmlight_led warmlight_dev;

static struct pwm_spec_config pwm_setting = {

	//频率:clk_src/clk_div/DATA_WIDTH
	//占空比:THRESH/DATA_WIDTH
	.pwm_no = 1,
	.mode = PWM_MODE_OLD,
	.clk_div = CLK_DIV1,
	.clk_src = PWM_CLK_OLD_MODE_BLOCK,
	.PWM_MODE_OLD_REGS.IDLE_VALUE = IDLE_FALSE,
	.PWM_MODE_OLD_REGS.GUARD_VALUE = GUARD_FALSE,
	.PWM_MODE_OLD_REGS.GDURATION = 0,
	.PWM_MODE_OLD_REGS.WAVE_NUM = 0,
	//.PWM_MODE_OLD_REGS.DATA_WIDTH = 9,
	//.PWM_MODE_OLD_REGS.THRESH = 4,					
	
	.pmic_pad = 0,
	//.PWM_MODE_OLD_REGS.STOP_BITPOS_VALUE = 31,
	/* 1 microseconds, assume clock source is 26M */
	//.PWM_MODE_OLD_REGS.HDURATION = 229,
	//.PWM_MODE_OLD_REGS.LDURATION = 229,
};


static void led_brightness_set(struct led_classdev * led_cdev, enum led_brightness brightness)
{                   
	if(brightness > maxlevel)
	{
		brightness = maxlevel;
		printk("brightness > maxlevel,brightness is %d\n",brightness);
	}                     
	warmlight_dev.cur_brightness = brightness;  
	pr_info("[LED]led_brightness_set,name=%s\n",led_cdev->name);	
	if (strcmp(led_cdev->name, "led-warmlight") == 0)
	{	
		mt_warmlight_set_pwm(brightness);
	}
}  

static enum led_brightness led_brightness_get(struct led_classdev * led_dev)
{
	return warmlight_dev.cur_brightness;
}


int led_init(void)
{
	int ret;
	warmlight_dev.max_brightness = maxlevel;
	warmlight_dev.cdev.name = led_name[0]; 
	warmlight_dev.cdev.brightness_set = led_brightness_set;
	warmlight_dev.cdev.brightness_get = led_brightness_get;
	
	ret = led_classdev_register(NULL, &warmlight_dev.cdev);
	if(ret)
	{
		printk("led_classdev_register fail,and unregister\n");
		led_classdev_unregister(&warmlight_dev.cdev);
	}
	return ret;
}



void mt_warmlight_set_pwm(u32 level)
{

	printk("level is %d\n",level);

	if(level > maxlevel)
	{
		level = maxlevel;
		printk("level > maxlevel,level is %d\n",level);
	}

	/* 256 level */
	//pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 255;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = maxlevel;
	pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;

	//pr_debug("[LEDS]backlight_set_pwm:duty is %d/%d\n",
	//		level,pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
	//pr_debug("[LEDS]backlight_set_pwm:clk_src/div is %d%d\n",
	//		pwm_setting.clk_src,pwm_setting.clk_div);
	
	if (level > 0 && level < 256) {
		pwm_set_spec_config(&pwm_setting);
		printk("[LEDS]old mode: level/maxlevel is %d/%d\n",
			pwm_setting.PWM_MODE_OLD_REGS.THRESH,
			pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
	} else {
		printk("[LEDS]Error level in backlight\n");
		mt_pwm_disable(pwm_setting.pwm_no,false);
	}
		
}


static int mtk_pwm_warmlight_probe(struct platform_device *pdev)
{
	int ret;
	printk("mtk_pwm_warmlight_probe enter\n");
	ret = led_init();

	if (ret < 0) {
		printk("led_init fail, ret: %d\n", ret);
	}

	return 0;
}

static const struct of_device_id mtk_pwm_warmlight_of_match[] = {
	{.compatible = "mediatek,warmlight-pwm",},
	{}
};

static struct platform_driver pwm_warmlight_driver = {
	.probe = mtk_pwm_warmlight_probe,
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(mtk_pwm_warmlight_of_match),
	},
};

module_platform_driver(pwm_warmlight_driver);

MODULE_DESCRIPTION("MTK PWM IR Transmitter");
MODULE_AUTHOR("Chang-An Chen <chang-an.chen@mediatek.com>");
MODULE_LICENSE("GPL");
