/*
 * Copyright (C) 2023 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#include "pad.h"
#include "pad_debug.h"
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#ifdef CONFIG_MTK_MT6306_GPIO_SUPPORT
#include <mtk_6306_gpio.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/* for magnify velocity******************************************** */
#define TOUCH_IOC_MAGIC 'A'

#define PAD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC, 0)
#define PAD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC, 1)
#define PAD_GET_FILTER_PARA _IOWR(TOUCH_IOC_MAGIC, 2, struct pad_filter_t)
#ifdef CONFIG_COMPAT
#define COMPAT_PAD_GET_FILTER_PARA _IOWR(TOUCH_IOC_MAGIC, \
						2, struct pad_filter_t)
#endif

struct pad_filter_t pad_filter;

const struct of_device_id pad_of_match[] = {
	{ .compatible = "mediatek,touch_pad", },
	{},
};

/* Global variables */
unsigned long PAD_RES_X = 0;
unsigned long PAD_RES_Y = 0;
int pad_load_status = 0;      /* 0: failed, 1: success */
int pad_mode = 0;
int pad_mode_axis = 0;
int pad_mode_min = 0;
int pad_mode_max = 0;
int pad_mode_keypad_tolerance = 0;

int PAD_DO_WARP = 0;
int pad_v_magnify_x = 0;
int pad_v_magnify_y = 0;

/* Debug variables */
int pad_em_debounce_time = 0;
int pad_em_debounce_time0 = 0;
int pad_em_debounce_time1 = 0;
int pad_em_asamp = 0;
int pad_em_auto_time_interval = 0;
int pad_em_sample_cnt = 0;
int pad_em_pressure_threshold = 0;
int pad_em_spl_num = 0;
DEFINE_MUTEX(pad_em_mutex);

static int pad_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int pad_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

#ifdef CONFIG_COMPAT
static long pad_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	switch (cmd) {
	case COMPAT_PAD_GET_FILTER_PARA:
		if (arg32 == NULL) {
			pr_info("invalid argument.");
			return -EINVAL;
		}
		ret = file->f_op->unlocked_ioctl(file, PAD_GET_FILTER_PARA,
					   (unsigned long)arg32);
		if (ret) {
			pr_info("PAD_GET_FILTER_PARA unlocked_ioctl failed.");
			return ret;
		}
		break;
	default:
		pr_info("pad: unknown IOCTL: 0x%08x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static long pad_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* char strbuf[256]; */
	void __user *data;
	long err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) {
		pr_info("pad: access error: %08X, (%2d, %2d)\n",
			cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case PAD_GET_VELOCITY_CUSTOM_X:
		data = (void __user *)arg;

		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &pad_v_magnify_x, sizeof(pad_v_magnify_x))) {
			err = -EFAULT;
			break;
		}
		break;

	case PAD_GET_VELOCITY_CUSTOM_Y:
		data = (void __user *)arg;

		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &pad_v_magnify_y, sizeof(pad_v_magnify_y))) {
			err = -EFAULT;
			break;
		}
		break;

	case PAD_GET_FILTER_PARA:
		data = (void __user *) arg;

		if (data == NULL) {
			err = -EINVAL;
			PAD_DMESG("GET_FILTER_PARA: data is null\n");
			break;
		}

		if (copy_to_user(data, &pad_filter, sizeof(struct pad_filter_t))) {
			PAD_DMESG("GET_FILTER_PARA: copy data error\n");
			err = -EFAULT;
			break;
		}
		break;

	default:
		pr_info("pad: unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

static struct work_struct touch_resume_work;
static struct workqueue_struct *touch_resume_workqueue;

static const struct file_operations pad_fops = {
/* .owner = THIS_MODULE, */
	.open = pad_misc_open,
	.release = pad_misc_release,
	.unlocked_ioctl = pad_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pad_compat_ioctl,
#endif
};

static struct miscdevice pad_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pad_touch",
	.fops = &pad_fops,
};

/* Function definitions */
static int __init pad_device_init(void);
static void __exit pad_device_exit(void);
static int pad_probe(struct platform_device *pdev);
static int pad_remove(struct platform_device *pdev);
static struct work_struct pad_init_work;
static struct workqueue_struct *pad_init_workqueue;
static int pad_suspend_flag;
int pad_register_flag;

/* Global variable definitions */
struct pad_device *pad;
static struct pad_driver_t pad_driver_list[PAD_DRV_MAX_COUNT];

struct platform_device pad_device = {
	.name = PAD_DEVICE,
	.id = -1,
};

const struct dev_pm_ops pad_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};

static struct platform_driver pad_driver = {
	.remove = pad_remove,
	.shutdown = NULL,
	.probe = pad_probe,
	.driver = {
		.name = PAD_DEVICE,
		.pm = &pad_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = pad_of_match,
	},
};

static struct pad_driver_t *g_pad_drv;

/* Use fb_notifier */
static struct notifier_block pad_fb_notifier;

/* Callback for touch resume workqueue */
static void touch_resume_workqueue_callback(struct work_struct *work)
{
	PAD_DEBUG("PAD %s\n", __func__);
	g_pad_drv->resume(NULL);
	pad_suspend_flag = 0;
}

/* FB notifier callback for touchpad */
static int pad_fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct fb_event *evdata = NULL;
	int blank;
	int err = 0;

	PAD_DEBUG("%s\n", __func__);

	evdata = data;
	/* If we aren't interested in this event, skip it immediately... */
	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)evdata->data;
	PAD_DMESG("fb_notify(blank=%d)\n", blank);
	switch (blank) {
	case FB_BLANK_UNBLANK:
		PAD_DMESG("LCD ON Notify\n");
		if (g_pad_drv && pad_suspend_flag) {
			err = queue_work(touch_resume_workqueue, &touch_resume_work);
			if (!err) {
				PAD_DMESG("start resume_workqueue failed\n");
				return err;
			}
		}
		break;
	case FB_BLANK_POWERDOWN:
		PAD_DMESG("LCD OFF Notify\n");
		if (g_pad_drv && !pad_suspend_flag) {
			err = cancel_work_sync(&touch_resume_work);
			if (!err)
				PAD_DMESG("cancel resume_workqueue failed\n");
			g_pad_drv->suspend(NULL);
		}
		pad_suspend_flag = 1;
		break;
	default:
		break;
	}
	return 0;
}

/* Add driver: if find PAD_TYPE_CAPACITIVE driver successfully, loading it */
int pad_driver_add(struct pad_driver_t *pad_drv)
{
	int i;

	if (g_pad_drv != NULL) {
		PAD_DMESG("pad driver exist\n");
		return -1;
	}
	/* check parameter */
	if (pad_drv == NULL)
		return -1;
	pad_drv->pad_have_button = 0;
	/* R-touch */
	if (strcmp(pad_drv->pad_device_name, "generic") == 0) {
		pad_driver_list[0].pad_device_name = pad_drv->pad_device_name;
		pad_driver_list[0].pad_local_init = pad_drv->pad_local_init;
		pad_driver_list[0].suspend = pad_drv->suspend;
		pad_driver_list[0].resume = pad_drv->resume;
		pad_driver_list[0].pad_have_button = pad_drv->pad_have_button;
		return 0;
	}
	for (i = 1; i < PAD_DRV_MAX_COUNT; i++) {
		/* add pad driver into list */
		if (pad_driver_list[i].pad_device_name == NULL) {
			pad_driver_list[i].pad_device_name = pad_drv->pad_device_name;
			pad_driver_list[i].pad_local_init = pad_drv->pad_local_init;
			pad_driver_list[i].suspend = pad_drv->suspend;
			pad_driver_list[i].resume = pad_drv->resume;
			pad_driver_list[i].pad_have_button = pad_drv->pad_have_button;
			pad_driver_list[i].attrs = pad_drv->attrs;
			break;
		}
		if (strcmp(pad_driver_list[i].pad_device_name, pad_drv->pad_device_name) == 0)
			return 1;	/* driver exist */
	}

	return 0;
}

int pad_driver_remove(struct pad_driver_t *pad_drv)
{
	int i = 0;
	/* check parameter */
	if (pad_drv == NULL)
		return -1;
	for (i = 0; i < PAD_DRV_MAX_COUNT; i++) {
		/* find it */
		if (strcmp(pad_driver_list[i].pad_device_name, pad_drv->pad_device_name) == 0) {
			memset(&pad_driver_list[i], 0, sizeof(struct pad_driver_t));
			break;
		}
	}
	return 0;
}

static void pad_create_attributes(struct device *dev, struct pad_attrs *attrs)
{
	int num = attrs->num;

	for (; num > 0;) {
		if (device_create_file(dev, attrs->attr[--num]))
			pr_info("mtk_pad: pad create attributes file failed\n");
	}
}

/* Touch panel probe */
static int pad_probe(struct platform_device *pdev)
{
	int touch_type = 1;	/* 0:R-touch, 1: Cap-touch */
	int i = 0;
	int ret = 0;

	PAD_DMESG("enter %s, %d\n", "pad_probe", 384);

	if (misc_register(&pad_misc_device))
		PAD_DMESG("pad_misc_device register failed\n");

	pad = kmalloc(sizeof(struct pad_device), GFP_KERNEL);
	if (pad == NULL)
		return -ENOMEM;
	memset(pad, 0, sizeof(struct pad_device));

	/* allocate input device */
	pad->dev = input_allocate_device();
	if (pad->dev == NULL) {
		kfree(pad);
		return -ENOMEM;
	}

	/* Set screen resolution info */
#if defined(CONFIG_MTK_FB)
	PAD_RES_X = 1440; // DISP_GetScreenWidth();
	PAD_RES_Y = 720; // DISP_GetScreenHeight();
#endif

	/* Adjust resolution if needed */
	if (PAD_RES_X == 2560)
		PAD_RES_X = 2048;
	if (PAD_RES_Y == 1600)
		PAD_RES_Y = 1536;

	PAD_DEBUG("PAD_RES_X = %lu, PAD_RES_Y = %lu\n", PAD_RES_X, PAD_RES_Y);

	/* Configure pad modes and settings */
	pad_mode = PAD_MODE_NORMAL;
	pad_mode_axis = 0;
	pad_mode_min = PAD_RES_Y / 2;
	pad_mode_max = PAD_RES_Y;
	pad_mode_keypad_tolerance = PAD_RES_X * PAD_RES_X / 64;

	/* Configure input device properties */
	pad->dev->name = PAD_DEVICE;
	set_bit(EV_ABS, pad->dev->evbit);
	set_bit(EV_KEY, pad->dev->evbit);
	set_bit(BTN_TOUCH, pad->dev->keybit);
	set_bit(INPUT_PROP_DIRECT, pad->dev->propbit);
	set_bit(ABS_X, pad->dev->absbit);
	set_bit(ABS_Y, pad->dev->absbit);
	set_bit(ABS_PRESSURE, pad->dev->absbit);
	set_bit(KEY_HOMEPAGE, pad->dev->keybit);
	set_bit(KEY_BACK, pad->dev->keybit);

	/* Save dev for regulator_get() before pad_local_init() */
	pad->pad_dev = &pdev->dev;

	/* Initialize touchpad device */
	for (i = 1; i < PAD_DRV_MAX_COUNT; i++) {
		if (pad_driver_list[i].pad_device_name != NULL) {
			pad_driver_list[i].pad_local_init();
			if (pad_load_status == 1) {
                PAD_DMESG("%s, pad_driver_name=%s\n", __func__,
                    pad_driver_list[i].pad_device_name);
                g_pad_drv = &pad_driver_list[i];
				break;
			}
		}
	}

	if (g_pad_drv == NULL) {
		if (pad_driver_list[0].pad_device_name != NULL) {
			g_pad_drv = &pad_driver_list[0];
			/* touch_type:0: r-touch, 1: C-touch */
			touch_type = 0;
			g_pad_drv->pad_local_init();
			PAD_DMESG("Generic touchpad panel driver\n");
		} else {
			PAD_DMESG("cap touchpad and Generic touchpad both are not loaded!!\n");
			return 0;
		}
	}

	/* Create resume workqueue */
	touch_resume_workqueue = create_singlethread_workqueue("pad_touch_resume");
	INIT_WORK(&touch_resume_work, touch_resume_workqueue_callback);

	/* Register FB notifier */
	pad_fb_notifier.notifier_call = pad_fb_notifier_callback;
	if (fb_register_client(&pad_fb_notifier))
		PAD_DMESG("register fb_notifier fail!\n");

	/* Configure capacitive touchpad settings based on decompiled assembly */
	if (touch_type) {
		/* Multitouch capabilities */
		set_bit(ABS_MT_TRACKING_ID, pad->dev->absbit);
		set_bit(ABS_MT_TOUCH_MAJOR, pad->dev->absbit);
		set_bit(ABS_MT_TOUCH_MINOR, pad->dev->absbit);
		set_bit(ABS_MT_POSITION_X, pad->dev->absbit);
		set_bit(ABS_MT_POSITION_Y, pad->dev->absbit);

		/* Setting parameters for multitouch events with fixed resolutions */
		input_set_abs_params(pad->dev, ABS_MT_POSITION_X, 0, 1440, 0, 0);
		input_set_abs_params(pad->dev, ABS_MT_POSITION_Y, 0, 720, 0, 0);
		input_set_abs_params(pad->dev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
		input_set_abs_params(pad->dev, ABS_MT_TOUCH_MINOR, 0, 100, 0, 0);

		PAD_DMESG("Cap touchpad panel driver\n");
	}

	/* Single touch capabilities with fixed resolution values from decompiled code */
	input_set_abs_params(pad->dev, ABS_X, 0, 1440, 0, 0);
	input_set_abs_params(pad->dev, ABS_Y, 0, 720, 0, 0);

	/* Set resolution for abs info */
	input_abs_set_res(pad->dev, ABS_X, 1440);
	input_abs_set_res(pad->dev, ABS_Y, 720);

	/* Pressure and tracking ID parameters */
	input_set_abs_params(pad->dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(pad->dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

	if ((ret = input_register_device(pad->dev)))
		PAD_DMESG("input_register_device failed.(pad)\n");
	else
		pad_register_flag = 1;

	/* Create device attributes */
	if (g_pad_drv->attrs.num) {
		pad_create_attributes(&pdev->dev, &g_pad_drv->attrs);
	}

	return 0;
}

static int pad_remove(struct platform_device *pdev)
{
	input_unregister_device(pad->dev);
	return 0;
}

/* Called when loaded into kernel */
static void pad_init_work_callback(struct work_struct *work)
{
	PAD_DEBUG("MediaTek touchpad panel driver init\n");
	if (platform_driver_register(&pad_driver) != 0)
		PAD_DMESG("unable to register touchpad panel driver.\n");
}

extern void synaptics_rmi4_init(void);

static int __init pad_device_init(void)
{
	int res = 0;

	pad_init_workqueue = create_singlethread_workqueue("mtk-pad");
	INIT_WORK(&pad_init_work, pad_init_work_callback);

	res = queue_work(pad_init_workqueue, &pad_init_work);
	if (!res)
		pr_info("pad : touchpad device init failed res:%d\n", res);
	return 0;
}

/* Should never be called */
static void __exit pad_device_exit(void)
{
	PAD_DMESG("MediaTek touchpad panel driver exit\n");
	/* input_unregister_device(pad->dev); */
	platform_driver_unregister(&pad_driver);
}

late_initcall(pad_device_init);
module_exit(pad_device_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek touchpad panel driver");
MODULE_AUTHOR("MediaTek Inc.");