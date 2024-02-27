/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
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
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef LED_GPIO_DTNAME
#define LED_GPIO_DTNAME "mediatek,flashlights_led_gpio"
#endif

/* TODO: define driver name */
#define LED_GPIO_NAME "flashlights_led_gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(led_gpio_mutex);
static struct work_struct led_gpio_work;

/* define pinctrl */
/* TODO: define pinctrl */
static struct pinctrl *flashlight_gpio = NULL;
static struct pinctrl_state *flashlight_gpio_mode_default = NULL;
static struct pinctrl_state *flashlight_gpio_en_l = NULL;
static struct pinctrl_state *flashlight_gpio_en_h = NULL;
static struct pinctrl_state *flashlight_gpio_touch_l = NULL;
static struct pinctrl_state *flashlight_gpio_touch_h = NULL;

//static struct pinctrl_state *flashlight_sub_gpio_en_l = NULL;
//static struct pinctrl_state *flashlight_sub_gpio_en_h = NULL;

/* define usage count */
static int use_count;
static int g_duty = -1;
//static int g_flash_channel_idx;

/* platform data */
struct led_gpio_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int led_gpio_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	flashlight_gpio = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flashlight_gpio)) {
		ret = PTR_ERR(flashlight_gpio);
		pr_err("[ERROR] Cannot find flashlight_gpio %d!\n", ret);
		return ret;
	}
	
	flashlight_gpio_mode_default = pinctrl_lookup_state(flashlight_gpio, "pin_default");
	if (IS_ERR(flashlight_gpio_mode_default)) {
		ret = PTR_ERR(flashlight_gpio_mode_default);
		pr_err( "[ERROR] Cannot find flashlight_gpio_mode_default %d!\n", ret);
	
	}
	
	flashlight_gpio_en_h = pinctrl_lookup_state(flashlight_gpio, "pin_fl_en1");
	if (IS_ERR(flashlight_gpio_en_h)) {
		ret = PTR_ERR(flashlight_gpio_en_h);
		pr_err( "[ERROR] Cannot find flashlight_gpio_en_h:%d!\n", ret);
		return ret;
	}

	flashlight_gpio_en_l = pinctrl_lookup_state(flashlight_gpio, "pin_fl_en0");
	if (IS_ERR(flashlight_gpio_en_l)) {
		ret = PTR_ERR(flashlight_gpio_en_l);
		pr_err( "[ERROR] Cannot find flashlight_gpio_en_l:%d!\n", ret);
		return ret;
	}


	flashlight_gpio_touch_h = pinctrl_lookup_state(flashlight_gpio, "pin_fl_torch1");
	if (IS_ERR(flashlight_gpio_touch_h)) {
		ret = PTR_ERR(flashlight_gpio_touch_h);
		pr_err( "[ERROR] Cannot find flashlight_gpio_touch_h:%d!\n", ret);
		return ret;
	}

	flashlight_gpio_touch_l = pinctrl_lookup_state(flashlight_gpio, "pin_fl_torch0");
	if (IS_ERR(flashlight_gpio_touch_l)) {
		ret = PTR_ERR(flashlight_gpio_touch_l);
		pr_err( "[ERROR] Cannot find flashlight_gpio_touch_l:%d!\n", ret);
		return ret;
	}	
	/*

	flashlight_sub_gpio_en_h = pinctrl_lookup_state(flashlight_gpio, "pin_fl_sub_en1");
	if (IS_ERR(flashlight_sub_gpio_en_h)) {
		ret = PTR_ERR(flashlight_sub_gpio_en_h);
		pr_err( "[ERROR] Cannot find flashlight_sub_gpio_en_h:%d!\n", ret);
		return ret;
	}

	flashlight_sub_gpio_en_l = pinctrl_lookup_state(flashlight_gpio, "pin_fl_sub_en0");
	if (IS_ERR(flashlight_sub_gpio_en_l)) {
		ret = PTR_ERR(flashlight_sub_gpio_en_l);
		pr_err( "[ERROR] Cannot find flashlight_sub_gpio_en_l:%d!\n", ret);
		return ret;
	} */

	return 0;

}


/******************************************************************************
 * led_gpio operations
 *****************************************************************************/
/* flashlight enable function */
static int led_gpio_enable(void)
{
	if (g_duty < 0)
		g_duty = 0;
	else if (g_duty > 16)
		g_duty = 16;
	if (g_duty <= 2) {
	//	pr_debug(" FL_Enable line=%d,g_duty = %d\n", __LINE__,g_duty);
	  // if (g_flash_channel_idx == 0)
	  // 	{
           pinctrl_select_state(flashlight_gpio, flashlight_gpio_en_h);
		   pinctrl_select_state(flashlight_gpio, flashlight_gpio_touch_l);
	  // 	}
	  // else ;
           //pinctrl_select_state(flashlight_gpio, flashlight_sub_gpio_en_h);   
		
	} else {
	//	pr_debug(" FL_Enable line=%d,g_duty = %d\n", __LINE__,g_duty);
	//	if (g_flash_channel_idx == 0)
	//	{
           pinctrl_select_state(flashlight_gpio, flashlight_gpio_en_h);
           pinctrl_select_state(flashlight_gpio, flashlight_gpio_touch_h);
	//	}
	 //  else ;
           //pinctrl_select_state(flashlight_gpio, flashlight_sub_gpio_en_h); 
		
	}
	pr_debug(" FL_Enable line=%d\n", __LINE__);
	return 0;
}

/* flashlight disable function */
static int led_gpio_disable(void)
{
	//if (g_flash_channel_idx == 0)
	//{
		pinctrl_select_state(flashlight_gpio, flashlight_gpio_en_l);
		pinctrl_select_state(flashlight_gpio, flashlight_gpio_touch_l);
	//}
	//else ;
	    //pinctrl_select_state(flashlight_gpio, flashlight_sub_gpio_en_l);
	pr_debug(" FL_Disable line=%d\n", __LINE__);
	return 0;
}

/* set flashlight level */
static int led_gpio_set_level(int level)
{
    g_duty = level;
	return 0;
}

/* flashlight init */
static int led_gpio_init(void)
{
	led_gpio_disable();

	return 0;
}

/* flashlight uninit */
static int led_gpio_uninit(void)
{

	led_gpio_disable();

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer led_gpio_timer;
static unsigned int led_gpio_timeout_ms;

static void led_gpio_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	led_gpio_disable();
}

static enum hrtimer_restart led_gpio_timer_func(struct hrtimer *timer)
{
	schedule_work(&led_gpio_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int led_gpio_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;
	//g_flash_channel_idx = channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		led_gpio_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		led_gpio_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (led_gpio_timeout_ms) {
				ktime = ktime_set(led_gpio_timeout_ms / 1000,
						(led_gpio_timeout_ms % 1000) * 1000000);
				hrtimer_start(&led_gpio_timer, ktime, HRTIMER_MODE_REL);
			}
			led_gpio_enable();
		} else {
			led_gpio_disable();
			hrtimer_cancel(&led_gpio_timer);
		}
		break;
	default:
		pr_debug("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int led_gpio_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int led_gpio_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int led_gpio_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&led_gpio_mutex);
	if (set) {
		if (!use_count)
			ret = led_gpio_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = led_gpio_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&led_gpio_mutex);

	return ret;
}

static ssize_t led_gpio_strobe_store(struct flashlight_arg arg)
{
	led_gpio_set_driver(1);
	led_gpio_set_level(arg.level);
	led_gpio_timeout_ms = 0;
	led_gpio_enable();
	msleep(arg.dur);
	led_gpio_disable();
	led_gpio_set_driver(0);

	return 0;
}

static struct flashlight_operations led_gpio_ops = {
	led_gpio_open,
	led_gpio_release,
	led_gpio_ioctl,
	led_gpio_strobe_store,
	led_gpio_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int led_gpio_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * led_gpio_init();
	 */

	return 0;
}

static int led_gpio_parse_dt(struct device *dev,
		struct led_gpio_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
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
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, LED_GPIO_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int led_gpio_probe(struct platform_device *pdev)
{
	struct led_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (led_gpio_pinctrl_init(pdev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = led_gpio_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&led_gpio_work, led_gpio_work_disable);

	/* init timer */
	hrtimer_init(&led_gpio_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	led_gpio_timer.function = led_gpio_timer_func;
	led_gpio_timeout_ms = 100;

	/* init chip hw */
	led_gpio_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &led_gpio_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(LED_GPIO_NAME, &led_gpio_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int led_gpio_remove(struct platform_device *pdev)
{
	struct led_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(LED_GPIO_NAME);

	/* flush work queue */
	flush_work(&led_gpio_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id led_gpio_of_match[] = {
	{.compatible = LED_GPIO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, led_gpio_of_match);
#else
static struct platform_device led_gpio_platform_device[] = {
	{
		.name = LED_GPIO_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, led_gpio_platform_device);
#endif

static struct platform_driver led_gpio_platform_driver = {
	.probe = led_gpio_probe,
	.remove = led_gpio_remove,
	.driver = {
		.name = LED_GPIO_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = led_gpio_of_match,
#endif
	},
};

static int __init flashlight_led_gpio_init(void)
{
	int ret;

	pr_debug("flashlight_led_gpio_init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&led_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&led_gpio_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("flashlight_led_gpio_init done.\n");

	return 0;
}

static void __exit flashlight_led_gpio_exit(void)
{
	pr_debug("flashlight_led_gpio_init start.\n");

	platform_driver_unregister(&led_gpio_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_led_gpio_init);
module_exit(flashlight_led_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight led_gpio GPIO Driver");

