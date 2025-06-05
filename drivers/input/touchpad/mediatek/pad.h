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

#ifndef __PAD_H
#define __PAD_H
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <generated/autoconf.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>

/*debug macros */
#define PAD_DEBUG
#define PAD_DEBUG_CODE
/* #define PAD_DEBUG_TRACK */
#define PAD_DMESG(a, arg...) \
	pr_info(PAD_DEVICE ":[%s:%d] " a, __func__, __LINE__, ##arg)
#if defined(PAD_DEBUG)
#undef PAD_DEBUG
#define PAD_DEBUG(a, arg...) \
	pr_info(PAD_DEVICE ":[%s:%d] " a, __func__, __LINE__, ##arg)
#else
#define PAD_DEBUG(arg...)
#endif
#define SPLIT ", "

/* register, address, configurations */
#define PAD_DEVICE            "mtk-pad"
#define PAD_X                  0
#define PAD_Y                  1
#define PAD_Z1                 2
#define PAD_Z2                 3
#define PAD_DELAY             (2*HZ/100)
#define PAD_DRV_MAX_COUNT      (20)
#define PAD_WARP_CNT          (4)
#define PAD_VIRTUAL_KEY_MAX   (10)

/* various mode */
#define PAD_MODE_NORMAL        0
#define PAD_MODE_KEYPAD        1
#define PAD_MODE_SW            2
#define PAD_MODE_FAV_SW        3
#define PAD_MODE_FAV_HW        4
#define PAD_MODE_RAW_DATA      5

extern unsigned long PAD_RES_X;
extern unsigned long PAD_RES_Y;
extern int pad_load_status;     /* 0: failed, 1: success */
extern int pad_mode;
extern int pad_mode_axis;
extern int pad_mode_min;
extern int pad_mode_max;
extern int pad_mode_keypad_tolerance;

extern int PAD_DO_WARP;
extern int pad_v_magnify_x;
extern int pad_v_magnify_y;

struct pad_device {
    struct device *pad_dev;
    struct regulator *reg;
    struct regulator *io_reg;
    struct input_dev *dev;
    struct input_dev *kpd;
    struct timer_list timer;
    struct tasklet_struct tasklet;
    int btn_state;
};

struct pad_filter_t {
    int enable; /*0: disable, 1: enable*/
    int pixel_density; /*XXX pixel/cm*/
    int W_W[3][4];/*filter custom setting parameters*/
    unsigned int VECLOCITY_THRESHOLD[3];/*filter speed custom settings*/
};

struct pad_attrs {
    struct device_attribute **attr;
    int num;
};

struct pad_driver_t {
    char *pad_device_name;
    int (*pad_local_init)(void);
    void (*suspend)(struct device *h);
    void (*resume)(struct device *h);
    int pad_have_button;
    struct pad_attrs attrs;
};

extern const struct of_device_id pad_of_match[];

/* Function declarations */
int pad_driver_add(struct pad_driver_t *pad_drv);
int pad_driver_remove(struct pad_driver_t *pad_drv);

#endif /* __PAD_H */