// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include "camss-vfe.h"
#include "camss.h"

#define MSM_VFE_NAME "msm_vfe"

/* VFE reset timeout */
#define VFE_RESET_TIMEOUT_MS 50

#define SCALER_RATIO_MAX 16

#define VFE_HW_VERSION		0x0
#define		HW_VERSION_STEPPING	0
#define		HW_VERSION_REVISION	16
#define		HW_VERSION_GENERATION	28

static const struct camss_format_info formats_rdi_8x16[] = {
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_UYVY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_VYUY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_YUYV, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_YVYU, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, 8, V4L2_PIX_FMT_SBGGR8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, 8, V4L2_PIX_FMT_SGBRG8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, 8, V4L2_PIX_FMT_SGRBG8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, 8, V4L2_PIX_FMT_SRGGB8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, 10, V4L2_PIX_FMT_SBGGR10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, 10, V4L2_PIX_FMT_SGBRG10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, 10, V4L2_PIX_FMT_SGRBG10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, 10, V4L2_PIX_FMT_SRGGB10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, 12, V4L2_PIX_FMT_SBGGR12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, 12, V4L2_PIX_FMT_SGBRG12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, 12, V4L2_PIX_FMT_SGRBG12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, 12, V4L2_PIX_FMT_SRGGB12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_Y10_1X10, 10, V4L2_PIX_FMT_Y10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
};

static const struct camss_format_info formats_rdi_8x96[] = {
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_UYVY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_VYUY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_YUYV, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_YVYU, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, 8, V4L2_PIX_FMT_SBGGR8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, 8, V4L2_PIX_FMT_SGBRG8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, 8, V4L2_PIX_FMT_SGRBG8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, 8, V4L2_PIX_FMT_SRGGB8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, 10, V4L2_PIX_FMT_SBGGR10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, 10, V4L2_PIX_FMT_SGBRG10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, 10, V4L2_PIX_FMT_SGRBG10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, 10, V4L2_PIX_FMT_SRGGB10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE, 16, V4L2_PIX_FMT_SBGGR10, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, 12, V4L2_PIX_FMT_SBGGR12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, 12, V4L2_PIX_FMT_SGBRG12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, 12, V4L2_PIX_FMT_SGRBG12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, 12, V4L2_PIX_FMT_SRGGB12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SBGGR14_1X14, 14, V4L2_PIX_FMT_SBGGR14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_SGBRG14_1X14, 14, V4L2_PIX_FMT_SGBRG14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_SGRBG14_1X14, 14, V4L2_PIX_FMT_SGRBG14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_SRGGB14_1X14, 14, V4L2_PIX_FMT_SRGGB14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_Y10_1X10, 10, V4L2_PIX_FMT_Y10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_Y10_2X8_PADHI_LE, 16, V4L2_PIX_FMT_Y10, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
};

static const struct camss_format_info formats_rdi_845[] = {
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_UYVY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_VYUY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_YUYV, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_YVYU, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, 8, V4L2_PIX_FMT_SBGGR8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, 8, V4L2_PIX_FMT_SGBRG8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, 8, V4L2_PIX_FMT_SGRBG8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, 8, V4L2_PIX_FMT_SRGGB8, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, 10, V4L2_PIX_FMT_SBGGR10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, 10, V4L2_PIX_FMT_SGBRG10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, 10, V4L2_PIX_FMT_SGRBG10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, 10, V4L2_PIX_FMT_SRGGB10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE, 16, V4L2_PIX_FMT_SBGGR10, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, 12, V4L2_PIX_FMT_SBGGR12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, 12, V4L2_PIX_FMT_SGBRG12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, 12, V4L2_PIX_FMT_SGRBG12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, 12, V4L2_PIX_FMT_SRGGB12P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 12) },
	{ MEDIA_BUS_FMT_SBGGR14_1X14, 14, V4L2_PIX_FMT_SBGGR14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_SGBRG14_1X14, 14, V4L2_PIX_FMT_SGBRG14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_SGRBG14_1X14, 14, V4L2_PIX_FMT_SGRBG14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_SRGGB14_1X14, 14, V4L2_PIX_FMT_SRGGB14P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 14) },
	{ MEDIA_BUS_FMT_Y8_1X8, 8, V4L2_PIX_FMT_GREY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 8) },
	{ MEDIA_BUS_FMT_Y10_1X10, 10, V4L2_PIX_FMT_Y10P, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 10) },
	{ MEDIA_BUS_FMT_Y10_2X8_PADHI_LE, 16, V4L2_PIX_FMT_Y10, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
};

static const struct camss_format_info formats_pix_8x16[] = {
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	/* 2X8 formats for parallel camera interface (CAMIF) */
	{ MEDIA_BUS_FMT_YUYV8_2X8, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YVYU8_2X8, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_2X8, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_VYUY8_2X8, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YUYV8_2X8, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YVYU8_2X8, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_2X8, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_VYUY8_2X8, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	/* Packed YUV 4:2:2 formats - for raw CAMIF passthrough (MSM8660/VFE31) */
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_UYVY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_VYUY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_YUYV, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_YVYU, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_UYVY8_2X8, 8, V4L2_PIX_FMT_UYVY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_VYUY8_2X8, 8, V4L2_PIX_FMT_VYUY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YUYV8_2X8, 8, V4L2_PIX_FMT_YUYV, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YVYU8_2X8, 8, V4L2_PIX_FMT_YVYU, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
};

static const struct camss_format_info formats_pix_8x96[] = {
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, 8, V4L2_PIX_FMT_NV12, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, 8, V4L2_PIX_FMT_NV21, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 3, 8) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_NV16, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_NV61, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 2, 8) },
	{ MEDIA_BUS_FMT_UYVY8_1X16, 8, V4L2_PIX_FMT_UYVY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_VYUY8_1X16, 8, V4L2_PIX_FMT_VYUY, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YUYV8_1X16, 8, V4L2_PIX_FMT_YUYV, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
	{ MEDIA_BUS_FMT_YVYU8_1X16, 8, V4L2_PIX_FMT_YVYU, 1,
	  PER_PLANE_DATA(0, 1, 1, 1, 1, 16) },
};

const struct camss_formats vfe_formats_rdi_8x16 = {
	.nformats = ARRAY_SIZE(formats_rdi_8x16),
	.formats = formats_rdi_8x16
};

const struct camss_formats vfe_formats_pix_8x16 = {
	.nformats = ARRAY_SIZE(formats_pix_8x16),
	.formats = formats_pix_8x16
};

const struct camss_formats vfe_formats_rdi_8x96 = {
	.nformats = ARRAY_SIZE(formats_rdi_8x96),
	.formats = formats_rdi_8x96
};

const struct camss_formats vfe_formats_pix_8x96 = {
	.nformats = ARRAY_SIZE(formats_pix_8x96),
	.formats = formats_pix_8x96
};

const struct camss_formats vfe_formats_rdi_845 = {
	.nformats = ARRAY_SIZE(formats_rdi_845),
	.formats = formats_rdi_845
};

/* TODO: Replace with pix formats */
const struct camss_formats vfe_formats_pix_845 = {
	.nformats = ARRAY_SIZE(formats_rdi_845),
	.formats = formats_rdi_845
};

static u32 vfe_src_pad_code(struct vfe_line *line, u32 sink_code,
			    unsigned int index, u32 src_req_code)
{
	struct vfe_device *vfe = to_vfe(line);

	switch (vfe->camss->res->version) {
	case CAMSS_8x16:
	case CAMSS_8x53:
	case CAMSS_8x60:
		switch (sink_code) {
		case MEDIA_BUS_FMT_YUYV8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_YUYV8_1X16,
				MEDIA_BUS_FMT_YUYV8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		case MEDIA_BUS_FMT_YVYU8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_YVYU8_1X16,
				MEDIA_BUS_FMT_YVYU8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		case MEDIA_BUS_FMT_UYVY8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_UYVY8_1X16,
				MEDIA_BUS_FMT_UYVY8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		case MEDIA_BUS_FMT_VYUY8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_VYUY8_1X16,
				MEDIA_BUS_FMT_VYUY8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		default:
			if (index > 0)
				return 0;

			return sink_code;
		}
		break;
	case CAMSS_660:
	case CAMSS_2290:
	case CAMSS_7280:
	case CAMSS_8x96:
	case CAMSS_8250:
	case CAMSS_8280XP:
	case CAMSS_8300:
	case CAMSS_845:
	case CAMSS_8550:
	case CAMSS_8775P:
	case CAMSS_X1E80100:
		switch (sink_code) {
		case MEDIA_BUS_FMT_YUYV8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_YUYV8_1X16,
				MEDIA_BUS_FMT_YVYU8_1X16,
				MEDIA_BUS_FMT_UYVY8_1X16,
				MEDIA_BUS_FMT_VYUY8_1X16,
				MEDIA_BUS_FMT_YUYV8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		case MEDIA_BUS_FMT_YVYU8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_YVYU8_1X16,
				MEDIA_BUS_FMT_YUYV8_1X16,
				MEDIA_BUS_FMT_UYVY8_1X16,
				MEDIA_BUS_FMT_VYUY8_1X16,
				MEDIA_BUS_FMT_YVYU8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		case MEDIA_BUS_FMT_UYVY8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_UYVY8_1X16,
				MEDIA_BUS_FMT_YUYV8_1X16,
				MEDIA_BUS_FMT_YVYU8_1X16,
				MEDIA_BUS_FMT_VYUY8_1X16,
				MEDIA_BUS_FMT_UYVY8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		case MEDIA_BUS_FMT_VYUY8_1X16:
		{
			u32 src_code[] = {
				MEDIA_BUS_FMT_VYUY8_1X16,
				MEDIA_BUS_FMT_YUYV8_1X16,
				MEDIA_BUS_FMT_YVYU8_1X16,
				MEDIA_BUS_FMT_UYVY8_1X16,
				MEDIA_BUS_FMT_VYUY8_1_5X8,
			};

			return camss_format_find_code(src_code, ARRAY_SIZE(src_code),
						      index, src_req_code);
		}
		default:
			if (index > 0)
				return 0;

			return sink_code;
		}
		break;
	default:
		WARN(1, "Unsupported HW version: %x\n",
		     vfe->camss->res->version);
		break;
	}
	return 0;
}

/*
 * vfe_hw_version - Process write master done interrupt
 * @vfe: VFE Device
 *
 * Return vfe hw version
 */
u32 vfe_hw_version(struct vfe_device *vfe)
{
	u32 hw_version = readl_relaxed(vfe->base + VFE_HW_VERSION);

	u32 gen = (hw_version >> HW_VERSION_GENERATION) & 0xF;
	u32 rev = (hw_version >> HW_VERSION_REVISION) & 0xFFF;
	u32 step = (hw_version >> HW_VERSION_STEPPING) & 0xFFFF;

	dev_dbg(vfe->camss->dev, "VFE:%d HW Version = %u.%u.%u\n",
		vfe->id, gen, rev, step);

	return hw_version;
}

/*
 * vfe_buf_done - Process write master done interrupt
 * @vfe: VFE Device
 * @wm: Write master id
 */
void vfe_buf_done(struct vfe_device *vfe, int wm)
{
	struct vfe_line *line = &vfe->line[vfe->wm_output_map[wm]];
	const struct vfe_hw_ops *ops = vfe->res->hw_ops;
	struct camss_buffer *ready_buf;
	struct vfe_output *output;
	unsigned long flags;
	u32 index;
	u64 ts = ktime_get_ns();

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm] == VFE_LINE_NONE) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	ready_buf = output->buf[0];
	if (!ready_buf) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d!\n", output->state);
		goto out_unlock;
	}

	ready_buf->vb.vb2_buf.timestamp = ts;
	ready_buf->vb.sequence = output->sequence++;

	index = 0;
	output->buf[0] = output->buf[1];
	if (output->buf[0])
		index = 1;

	output->buf[index] = vfe_buf_get_pending(output);

	if (output->buf[index]) {
		ops->vfe_wm_update(vfe, output->wm_idx[0],
				   output->buf[index]->addr[0],
				   line);
		ops->reg_update(vfe, line->id);
	} else {
		output->gen2.active_num--;
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	vb2_buffer_done(&ready_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

int vfe_enable_output_v2(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	const struct vfe_hw_ops *ops = vfe->res->hw_ops;
	struct media_pad *sensor_pad;
	unsigned long flags;
	unsigned int frame_skip = 0;
	unsigned int i;

	sensor_pad = camss_find_sensor_pad(&line->subdev.entity);
	if (sensor_pad) {
		struct v4l2_subdev *subdev =
			media_entity_to_v4l2_subdev(sensor_pad->entity);

		v4l2_subdev_call(subdev, sensor, g_skip_frames, &frame_skip);
		/* Max frame skip is 29 frames */
		if (frame_skip > VFE_FRAME_DROP_VAL - 1)
			frame_skip = VFE_FRAME_DROP_VAL - 1;
	}

	spin_lock_irqsave(&vfe->output_lock, flags);

	ops->reg_update_clear(vfe, line->id);

	if (output->state > VFE_OUTPUT_RESERVED) {
		dev_err(vfe->camss->dev,
			"Output is not in reserved state %d\n",
			output->state);
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return -EINVAL;
	}

	WARN_ON(output->gen2.active_num);

	output->state = VFE_OUTPUT_ON;

	output->sequence = 0;
	output->wait_reg_update = 0;
	reinit_completion(&output->reg_update);

	ops->vfe_wm_start(vfe, output->wm_idx[0], line);

	for (i = 0; i < 2; i++) {
		output->buf[i] = vfe_buf_get_pending(output);
		if (!output->buf[i])
			break;
		output->gen2.active_num++;
		ops->vfe_wm_update(vfe, output->wm_idx[0],
				   output->buf[i]->addr[0], line);
		ops->reg_update(vfe, line->id);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

/*
 * vfe_queue_buffer_v2 - Add empty buffer
 * @vid: Video device structure
 * @buf: Buffer to be enqueued
 *
 * Add an empty buffer - depending on the current number of buffers it will be
 * put in pending buffer queue or directly given to the hardware to be filled.
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_queue_buffer_v2(struct camss_video *vid,
			struct camss_buffer *buf)
{
	struct vfe_line *line = container_of(vid, struct vfe_line, video_out);
	struct vfe_device *vfe = to_vfe(line);
	const struct vfe_hw_ops *ops = vfe->res->hw_ops;
	struct vfe_output *output;
	unsigned long flags;

	output = &line->output;

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (output->state == VFE_OUTPUT_ON &&
	    output->gen2.active_num < 2) {
		output->buf[output->gen2.active_num++] = buf;
		ops->vfe_wm_update(vfe, output->wm_idx[0],
				   buf->addr[0], line);
		ops->reg_update(vfe, line->id);
	} else {
		vfe_buf_add_pending(output, buf);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

/*
 * vfe_enable_v2 - Enable streaming on VFE line
 * @line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_enable_v2(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	const struct vfe_hw_ops *ops = vfe->res->hw_ops;
	int ret;

	mutex_lock(&vfe->stream_lock);

	if (vfe->res->hw_ops->enable_irq)
		ops->enable_irq(vfe);

	vfe->stream_count++;

	mutex_unlock(&vfe->stream_lock);

	ret = vfe_get_output_v2(line);
	if (ret < 0)
		goto error_get_output;

	ret = vfe_enable_output_v2(line);
	if (ret < 0)
		goto error_enable_output;

	vfe->was_streaming = 1;

	return 0;

error_enable_output:
	vfe_put_output(line);

error_get_output:
	mutex_lock(&vfe->stream_lock);

	vfe->stream_count--;

	mutex_unlock(&vfe->stream_lock);

	return ret;
}

/*
 * vfe_get_output_v2 - Get vfe output port for corresponding VFE line
 * @line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_get_output_v2(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output;
	unsigned long flags;

	spin_lock_irqsave(&vfe->output_lock, flags);

	output = &line->output;
	if (output->state > VFE_OUTPUT_RESERVED) {
		dev_err(vfe->camss->dev, "Output is running\n");
		goto error;
	}

	output->wm_num = 1;

	/* Correspondence between VFE line number and WM number.
	 * line 0 -> RDI 0, line 1 -> RDI1, line 2 -> RDI2, line 3 -> PIX/RDI3
	 * Note this 1:1 mapping will not work for PIX streams.
	 */
	output->wm_idx[0] = line->id;
	vfe->wm_output_map[line->id] = line->id;

	output->drop_update_idx = 0;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;

error:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
	output->state = VFE_OUTPUT_OFF;

	return -EINVAL;
}

int vfe_reset(struct vfe_device *vfe)
{
	unsigned long time;

	reinit_completion(&vfe->reset_complete);
	vfe->vfe31_reset_done = false;

	vfe->res->hw_ops->global_reset(vfe);

	/*
	 * VFE31 on MSM8660: The reset IRQ doesn't fire. The global_reset
	 * function uses a delay and sets vfe31_reset_done directly.
	 */
	if (vfe->vfe31_reset_done)
		return 0;

	time = wait_for_completion_timeout(&vfe->reset_complete,
		msecs_to_jiffies(VFE_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(vfe->camss->dev, "VFE reset timeout\n");
		return -EIO;
	}

	return 0;
}

/*
 * VFE31 register definitions for deferred CAMIF configuration.
 * These are duplicated here for the MSM8660 workaround.
 */
#define VFE31_CORE_CFG			0x014
#define VFE31_CORE_CFG_PIXEL_YCBYCR	0x4
#define VFE31_CORE_CFG_PIXEL_YCRYCB	0x5
#define VFE31_CORE_CFG_PIXEL_CBYCRY	0x6
#define VFE31_CORE_CFG_PIXEL_CRYCBY	0x7

/*
 * Note: VFE31 does NOT have a separate VFE_CFG register for CAMIF_TO_BUS.
 * The 0x01C offset is actually VFE_IRQ_MASK_0 in VFE31!
 * CAMIF to bus routing is controlled by the AXI output mode at 0x40.
 * Setting AXI output mode to 0x60 (CAMIF_TO_AXI_VIA_OUTPUT_2) automatically
 * configures the internal routing for raw passthrough mode.
 */

/*
 * Note: 0x03C is NOT a BUS_CFG register on VFE31!
 * It's part of the 188-byte AXI output config block (ao[1]).
 * webOS leaves this at 0. We only read it for debug purposes.
 */
#define VFE31_AXI_CFG_1			0x03C  /* ao[1] - must be 0 */

/*
 * MSM8660 External Register Addresses for Debug
 *
 * MMCC (Multimedia Clock Controller) base: 0x04000000
 *   VFE_CC_REG at offset 0x0104 controls CSI-to-VFE bridge clocks:
 *   - Bit 10: vfe_csi1_clk enable (CSI1 -> VFE bridge)
 *   - Bit 12: vfe_csi0_clk enable (CSI0 -> VFE bridge)
 *
 * TCSR (Top-Level CSR) base: 0x16B00000
 *   May contain CSI mux registers that route CSIPHY to VFE.
 *   Exact layout unknown - dumping first 32 bytes for analysis.
 */
#define MSM8660_MMCC_BASE		0x04000000
#define MSM8660_MMCC_SIZE		0x1000
#define MSM8660_CSI_CC_REG_OFFSET	0x0040
#define MSM8660_CSI_CC_REG_CSI0_EN	BIT(0)
#define MSM8660_CSI_CC_REG_CSI1_EN	BIT(7)
#define MSM8660_CSI_CC_REG_CSI0_PHY_EN	BIT(8)
#define MSM8660_CSI_CC_REG_CSI1_PHY_EN	BIT(9)
#define MSM8660_VFE_CC_REG_OFFSET	0x0104
#define MSM8660_VFE_CC_REG_CSI1_VFE_EN	BIT(10)
#define MSM8660_VFE_CC_REG_CSI0_VFE_EN	BIT(12)

#define MSM8660_TCSR_BASE		0x16B00000
#define MSM8660_TCSR_SIZE		0x1000

/*
 * Debug option: Use EFS sync mode instead of APS for MIPI CSI-2.
 * Set to 1 to enable EFS mode (Gemini's suggestion).
 * APS (Active Pixel Sync) = 0: Uses external sync signals
 * EFS (Embedded Frame Sync) = 1: Uses embedded sync codes in data stream
 *
 * MIPI CSI-2 sends Frame Start/End as short packets. Some sensors may
 * require EFS mode if the CSIPHY strips the short packets and embeds
 * the sync info in the pixel stream.
 */
static int vfe31_use_efs_sync = 0;
module_param(vfe31_use_efs_sync, int, 0644);
MODULE_PARM_DESC(vfe31_use_efs_sync, "VFE31: Use EFS sync mode instead of APS (0=APS, 1=EFS)");

/*
 * vfe31_debug_dump_external_regs - Dump MMCC and TCSR registers for debug
 * @dev: Device for logging
 *
 * This function temporarily maps external registers (clock controller, TCSR)
 * to read their values during VFE initialization. This helps debug CSI-to-VFE
 * data path issues by verifying clock enables and potential CSI mux settings.
 */
static void vfe31_debug_dump_external_regs(struct device *dev)
{
	void __iomem *mmcc_base, *tcsr_base;
	u32 vfe_cc_reg;
	int i;

	/* Map and read MMCC registers */
	mmcc_base = ioremap(MSM8660_MMCC_BASE, MSM8660_MMCC_SIZE);
	if (mmcc_base) {
		u32 csi_cc_reg;

		/* CSI_CC_REG at 0x0040 - controls CSI core clocks */
		csi_cc_reg = readl_relaxed(mmcc_base + MSM8660_CSI_CC_REG_OFFSET);
		dev_info(dev, "VFE DEBUG: MMCC CSI_CC_REG (0x%08x) = 0x%08x\n",
			 MSM8660_MMCC_BASE + MSM8660_CSI_CC_REG_OFFSET, csi_cc_reg);
		dev_info(dev, "VFE DEBUG:   CSI0_CLK (bit 0): %s\n",
			 (csi_cc_reg & MSM8660_CSI_CC_REG_CSI0_EN) ? "ENABLED" : "disabled");
		dev_info(dev, "VFE DEBUG:   CSI1_CLK (bit 7): %s\n",
			 (csi_cc_reg & MSM8660_CSI_CC_REG_CSI1_EN) ? "ENABLED" : "disabled");
		dev_info(dev, "VFE DEBUG:   CSI0_PHY_CLK (bit 8): %s\n",
			 (csi_cc_reg & MSM8660_CSI_CC_REG_CSI0_PHY_EN) ? "ENABLED" : "disabled");
		dev_info(dev, "VFE DEBUG:   CSI1_PHY_CLK (bit 9): %s\n",
			 (csi_cc_reg & MSM8660_CSI_CC_REG_CSI1_PHY_EN) ? "ENABLED" : "disabled");

		/* VFE_CC_REG at 0x0104 - controls CSI-to-VFE bridge clocks */
		vfe_cc_reg = readl_relaxed(mmcc_base + MSM8660_VFE_CC_REG_OFFSET);
		dev_info(dev, "VFE DEBUG: MMCC VFE_CC_REG (0x%08x) = 0x%08x\n",
			 MSM8660_MMCC_BASE + MSM8660_VFE_CC_REG_OFFSET, vfe_cc_reg);
		dev_info(dev, "VFE DEBUG:   CSI0_VFE_CLK (bit 12): %s\n",
			 (vfe_cc_reg & MSM8660_VFE_CC_REG_CSI0_VFE_EN) ? "ENABLED" : "disabled");
		dev_info(dev, "VFE DEBUG:   CSI1_VFE_CLK (bit 10): %s\n",
			 (vfe_cc_reg & MSM8660_VFE_CC_REG_CSI1_VFE_EN) ? "ENABLED" : "disabled");

		/* If any CSI1 clocks are not enabled, warn */
		if (!(csi_cc_reg & MSM8660_CSI_CC_REG_CSI1_EN)) {
			dev_warn(dev, "VFE DEBUG: CSI1_CLK (bit 7) not enabled!\n");
		}
		if (!(csi_cc_reg & MSM8660_CSI_CC_REG_CSI1_PHY_EN)) {
			dev_warn(dev, "VFE DEBUG: CSI1_PHY_CLK (bit 9) not enabled!\n");
		}
		if (!(vfe_cc_reg & MSM8660_VFE_CC_REG_CSI1_VFE_EN)) {
			dev_warn(dev, "VFE DEBUG: CSI1_VFE_CLK not enabled! Attempting manual enable...\n");
			writel_relaxed(vfe_cc_reg | MSM8660_VFE_CC_REG_CSI1_VFE_EN,
				       mmcc_base + MSM8660_VFE_CC_REG_OFFSET);
			wmb();
			vfe_cc_reg = readl_relaxed(mmcc_base + MSM8660_VFE_CC_REG_OFFSET);
			dev_info(dev, "VFE DEBUG: After manual enable: VFE_CC_REG = 0x%08x\n", vfe_cc_reg);
		}

		iounmap(mmcc_base);
	} else {
		dev_err(dev, "VFE DEBUG: Failed to map MMCC base 0x%08x\n", MSM8660_MMCC_BASE);
	}

	/* Map and dump TCSR registers */
	tcsr_base = ioremap(MSM8660_TCSR_BASE, MSM8660_TCSR_SIZE);
	if (tcsr_base) {
		dev_info(dev, "VFE DEBUG: TCSR register dump (base 0x%08x):\n", MSM8660_TCSR_BASE);
		for (i = 0; i < 32; i += 4) {
			u32 val = readl_relaxed(tcsr_base + i);
			if (val != 0)  /* Only print non-zero registers */
				dev_info(dev, "VFE DEBUG:   TCSR[0x%03x] = 0x%08x\n", i, val);
		}
		/* Also check offsets 0x100-0x120 where CSI mux might be */
		dev_info(dev, "VFE DEBUG: TCSR potential CSI mux area (0x100-0x120):\n");
		for (i = 0x100; i < 0x120; i += 4) {
			u32 val = readl_relaxed(tcsr_base + i);
			dev_info(dev, "VFE DEBUG:   TCSR[0x%03x] = 0x%08x\n", i, val);
		}
		iounmap(tcsr_base);
	} else {
		dev_err(dev, "VFE DEBUG: Failed to map TCSR base 0x%08x\n", MSM8660_TCSR_BASE);
	}

	/*
	 * Dump CSIPHY register space to verify which PHY is active.
	 * CSIPHY0 at 0x04800000, CSIPHY1 at 0x04900000.
	 * MT9M113 uses CSIPHY1 (0x04900000).
	 */
	{
		void __iomem *csi0_base, *csi1_base;

		csi0_base = ioremap(0x04800000, 0x100);
		csi1_base = ioremap(0x04900000, 0x100);

		if (csi0_base && csi1_base) {
			u32 csi0_protocol = readl_relaxed(csi0_base + 0x04);
			u32 csi0_d1_ctrl = readl_relaxed(csi0_base + 0x20);
			u32 csi0_camera = readl_relaxed(csi0_base + 0x24);
			u32 csi1_protocol = readl_relaxed(csi1_base + 0x04);
			u32 csi1_d1_ctrl = readl_relaxed(csi1_base + 0x20);
			u32 csi1_camera = readl_relaxed(csi1_base + 0x24);

			dev_info(dev, "VFE DEBUG: CSIPHY0 (0x04800000): PROTOCOL=0x%08x D1_CTRL=0x%08x CAMERA_CNTL=0x%08x\n",
				 csi0_protocol, csi0_d1_ctrl, csi0_camera);
			dev_info(dev, "VFE DEBUG: CSIPHY1 (0x04900000): PROTOCOL=0x%08x D1_CTRL=0x%08x CAMERA_CNTL=0x%08x\n",
				 csi1_protocol, csi1_d1_ctrl, csi1_camera);
			dev_info(dev, "VFE DEBUG:   MT9M113 should use CSIPHY1 with D1_CTRL=0x300 (PHY enabled)\n");
		}

		if (csi0_base)
			iounmap(csi0_base);
		if (csi1_base)
			iounmap(csi1_base);
	}
}

/*
 * VFE31 AXI output mode configuration at 0x40.
 * This register controls which output paths are enabled.
 *
 * From webOS msm_vfe31.c vfe31_config_axi():
 *   OUTPUT_2 (preview mode):         0x200 - data flows through VFE ISP pipeline
 *   CAMIF_TO_AXI_VIA_OUTPUT_2 (raw): 0x60  - data bypasses VFE, goes direct to memory
 *
 * For PIX mode (with VFE processing), use 0x200.
 * For RDI mode (raw capture), use 0x60.
 */
#define VFE31_AXI_OUT_MODE_CFG		0x040
#define VFE31_AXI_OUT_MODE_PREVIEW	0x200	/* OUTPUT_2: preview w/ VFE ISP */
#define VFE31_AXI_OUT_MODE_RAW_SNAPSHOT	0x60	/* CAMIF_TO_AXI: raw bypass */

/*
 * VFE31 CAMIF register block layout (32 bytes at 0x1E4-0x203):
 *
 * IMPORTANT: VFE31 is DIFFERENT from VFE8x!
 *
 * The VFE8x VFE_CAMIFConfigType structure (at 0x114) has camif2vfeEnable (bit 8)
 * and camif2busEnable (bit 10) for data routing. VFE31 does NOT have these bits!
 *
 * The webOS libqcameralib reverse engineering report incorrectly claimed these
 * routing bits exist at 0x1E4. This was a misinterpretation - the actual webOS
 * kernel source (msm_vfe8x_proc.h) shows:
 *
 *   - VFE8x uses struct VFE_CAMIFConfigType at 0x114 with camif2vfeEnable/camif2busEnable
 *   - VFE31 uses struct vfe_camifcfg at 0x1E4 with EFS codes (NO routing bits)
 *
 * VFE31 0x1E4 (EFS_CFG) layout from vfe_camifcfg struct:
 *   [7:0]   efsEndOfLine   - EFS sync code (0 for APS mode)
 *   [15:8]  efsStartOfLine - EFS sync code (0 for APS mode)
 *   [23:16] efsEndOfFrame  - EFS sync code (0 for APS mode)
 *   [31:24] efsStartOfFrame- EFS sync code (0 for APS mode)
 *
 * For MIPI CSI-2 (APS mode), write 0 to EFS_CFG.
 *
 * Data routing on VFE31 is controlled ONLY via AXI output mode at 0x040:
 *   - 0x200: OUTPUT_2 (preview with VFE ISP processing)
 *   - 0x60:  CAMIF_TO_AXI_VIA_OUTPUT_2 (raw bypass to memory)
 *
 * 0x1E8: FRAME_CFG - frame dimensions
 *        [13:0]  pixelsPerLine (total bytes per line for YUV)
 *        [29:16] linesPerFrame
 * 0x1EC: WINDOW_WIDTH_CFG - horizontal capture window
 *        [13:0]  lastPixel (0-indexed)
 *        [29:16] firstPixel (usually 0)
 * 0x1F0: WINDOW_HEIGHT_CFG - vertical capture window
 *        [13:0]  lastLine (0-indexed)
 *        [29:16] firstLine (usually 0)
 * 0x1F4: SUBSAMPLE_CFG_0 - pixel/line subsampling
 * 0x1F8: SUBSAMPLE_CFG_1 - frame subsampling
 * 0x1FC: EPOCH_CFG - epoch interrupt lines
 */
#define VFE31_CAMIF_EFS_CFG		0x1E4	/* EFS codes (0 for APS mode) */
#define VFE31_CAMIF_FRAME_CFG		0x1E8	/* Frame dimensions */
#define VFE31_CAMIF_WINDOW_WIDTH_CFG	0x1EC	/* Horizontal window */
#define VFE31_CAMIF_WINDOW_HEIGHT_CFG	0x1F0	/* Vertical window */
#define VFE31_CAMIF_SUBSAMPLE_CFG_0	0x1F4	/* Subsample config */
#define VFE31_CAMIF_SUBSAMPLE_CFG_1	0x1F8	/* Frame subsample */
#define VFE31_CAMIF_EPOCH_CFG		0x1FC	/* Epoch interrupt */
#define VFE31_CAMIF_STATUS		0x204	/* Read status */
#define VFE31_CAMIF_CMD			0x1E0	/* Write commands */
#define VFE31_CAMIF_CMD_CLEAR_STATUS	BIT(2)
#define VFE31_CAMIF_CMD_STOP_IMMEDIATELY BIT(1)
/*
 * VFE31 CAMIF START command = 1 (BIT(0) only)
 *
 * NOTE: The webOS msm_vfe31.h header defines CAMIF_COMMAND_START = 0x5,
 * but the actual VFE31 code in vfe31_start_common() writes 1, NOT 0x5:
 *   msm_io_w(1, vfe31_ctrl->vfebase + VFE_CAMIF_COMMAND);
 *
 * The 0x5 macro is defined but NEVER USED in VFE31. It appears to be
 * legacy from VFE8x where it IS used. For VFE31, only BIT(0) is needed.
 *
 * Reference: webOS msm_vfe31.c line 1002
 */
/*
 * CAMIF_CMD_START: webOS msm_vfe31.h defines 0x5, but actual code at
 * vfe31_start_common() line 1002 writes 1 (just bit 0).
 * - Bit 0: Enable image data capture at frame boundary
 * - Bit 2: Clear CAMIF_STATUS register (NOT used in actual webOS start)
 */
#define VFE31_CAMIF_CMD_START		BIT(0)
#define VFE31_REG_UPDATE_CMD		0x260

/*
 * VFE31 IRQ registers and bit definitions
 *
 * IRQ_MASK_0 bit map (from webOS msm_vfe31.h):
 *   Bit 0:  CAMIF_SOF              - Start of Frame from CAMIF
 *   Bit 1:  CAMIF_EOF              - End of Frame (webOS does NOT enable this)
 *   Bit 5:  REG_UPDATE             - Register update acknowledged
 *   Bit 8-12: WM0-WM4 ping/pong    - Individual write master IRQs (not used by webOS)
 *   Bit 13: STATS_AEC              - Auto exposure stats ready
 *   Bit 14: STATS_AF               - Auto focus stats ready
 *   Bit 15: STATS_AWB              - Auto white balance stats ready
 *   Bit 16: STATS_RS               - Row sum stats ready
 *   Bit 17: STATS_CS               - Column sum stats ready
 *   Bit 18: STATS_IHIST            - Image histogram stats ready
 *   Bit 19: STATS_SKIN             - Skin detection stats ready
 *   Bit 21: IMAGE_COMPOSIT_DONE0   - Output path 0 frame complete
 *   Bit 22: IMAGE_COMPOSIT_DONE1   - Output path 1 frame complete
 *   Bit 23: IMAGE_COMPOSIT_DONE2   - Output path 2 frame complete
 *   Bit 24: STATS_COMPOSIT         - Stats composite done
 *   Bit 25-27: SYNC_TIMER0-2       - Sync timer IRQs
 *   Bit 28-31: ASYNC_TIMER0-3      - Async timer IRQs
 *
 * webOS vfe31_start_common() uses: 0x00EFE021
 *   = CAMIF_SOF | REG_UPDATE | all STATS | IMAGE_COMPOSIT_DONE0/1/2
 *
 * IRQ_MASK_1 bit map:
 *   Bit 0-21: Error IRQs (CAMIF_ERROR, overflow, violation, etc.)
 *   Bit 22: RESET_ACK              - Hardware reset complete
 *   Bit 23: AXI_HALT_ACK           - AXI bus halt complete
 *
 * webOS uses VFE_IMASK_WHILE_STOPPING_1 = 0x00400000 (bit 22 only)
 */
#define VFE31_IRQ_MASK_0		0x01C
#define VFE31_IRQ_MASK_1		0x020

/* IRQ_MASK_0 individual bits */
#define VFE31_IRQ_MASK_0_CAMIF_SOF		BIT(0)
#define VFE31_IRQ_MASK_0_CAMIF_EOF		BIT(1)
#define VFE31_IRQ_MASK_0_REG_UPDATE		BIT(5)
#define VFE31_IRQ_MASK_0_PING_PONG_WM(n)	BIT((n) + 8)
#define VFE31_IRQ_MASK_0_STATS_AEC		BIT(13)
#define VFE31_IRQ_MASK_0_STATS_AF		BIT(14)
#define VFE31_IRQ_MASK_0_STATS_AWB		BIT(15)
#define VFE31_IRQ_MASK_0_STATS_RS		BIT(16)
#define VFE31_IRQ_MASK_0_STATS_CS		BIT(17)
#define VFE31_IRQ_MASK_0_STATS_IHIST		BIT(18)
#define VFE31_IRQ_MASK_0_STATS_SKIN		BIT(19)
#define VFE31_IRQ_MASK_0_IMAGE_COMPOSIT_DONE0	BIT(21)
#define VFE31_IRQ_MASK_0_IMAGE_COMPOSIT_DONE1	BIT(22)
#define VFE31_IRQ_MASK_0_IMAGE_COMPOSIT_DONE2	BIT(23)
#define VFE31_IRQ_MASK_0_STATS_COMPOSIT		BIT(24)

/* IRQ_MASK_1 individual bits */
#define VFE31_IRQ_MASK_1_RESET_ACK		BIT(22)
#define VFE31_IRQ_MASK_1_AXI_HALT_ACK		BIT(23)

/* VFE31 bus ping-pong status (to verify AXI data flow) */
#define VFE31_BUS_PING_PONG_STATUS	0x180
/* Error bits are 0-21, used for VFE31_IMASK_ERROR_ONLY_1 */

/*
 * webOS IRQ mask values - use these exact values for compatibility!
 * From vfe31_start_common() in msm_vfe31.c:
 *   msm_io_w(0x00EFE021, vfe31_ctrl->vfebase + VFE_IRQ_MASK_0);
 *   msm_io_w(VFE_IMASK_WHILE_STOPPING_1, ...) = 0x00400000
 */
#define VFE31_IRQ_MASK_0_WEBOS			0x00EFE021
#define VFE31_IRQ_MASK_1_WEBOS			0x00400000

/*
 * VFE31 configuration registers
 * VFE_CFG_OFF (0x14): Pixel pattern and input source selection
 * VFE_MODULE_CFG (0x10): Module enable bits
 */
#define VFE31_CFG_OFF			0x014
#define VFE31_MODULE_CFG		0x010

/*
 * VFE_CFG register bit layout - Official Qualcomm documentation
 *
 * From Android kernel msm_vfe32.h and webOS msm_vfe8x_proc.h:
 *
 *   struct vfe_cfg {
 *       uint32_t pixelPattern:3;      // bits 0-2
 *       uint32_t reserved:13;         // bits 3-15 (see note below)
 *       uint32_t inputSource:2;       // bits 16-17
 *       uint32_t reserved:14;         // bits 18-31 (see note below)
 *   };
 *
 * pixelPattern values (bits 0-2) - enum VFE_START_PIXEL_PATTERN:
 *   0 = VFE_BAYER_RGRGRG
 *   1 = VFE_BAYER_GRGRGR
 *   2 = VFE_BAYER_BGBGBG
 *   3 = VFE_BAYER_GBGBGB
 *   4 = VFE_YUV_YCbYCr
 *   5 = VFE_YUV_YCrYCb
 *   6 = VFE_YUV_CbYCrY (UYVY format)
 *   7 = VFE_YUV_CrYCbY
 *
 * inputSource values (bits 16-17):
 *   0 = CAMIF (parallel camera interface)
 *   1 = TESTGEN (internal test pattern generator)
 *   2 = AXI (for MIPI CSI input via AXI bus)
 *
 * NOTE: The "reserved" bits (3-15, 18-31) are documented as reserved in
 * VFE8x/VFE32 headers, but webOS libqcameralib sets them for VFE31.
 * These appear to control internal VFE31 pipeline/mux configuration.
 * See VFE31_CFG_WEBOS_BASE below for the webOS-proven values.
 */

/* VFE_CFG_OFF pixel pattern values (bits 0-2) */
#define VFE31_PIXEL_PATTERN_BAYER_RGRGRG	0
#define VFE31_PIXEL_PATTERN_BAYER_GRGRGR	1
#define VFE31_PIXEL_PATTERN_BAYER_BGBGBG	2
#define VFE31_PIXEL_PATTERN_BAYER_GBGBGB	3
#define VFE31_PIXEL_PATTERN_YUV_YCbYCr		4
#define VFE31_PIXEL_PATTERN_YUV_YCrYCb		5
#define VFE31_PIXEL_PATTERN_YUV_CbYCrY		6  /* UYVY format */
#define VFE31_PIXEL_PATTERN_YUV_CrYCbY		7

/* VFE_CFG_OFF input source (bits 16-17) */
#define VFE31_INPUT_SOURCE_CAMIF	(0 << 16)
#define VFE31_INPUT_SOURCE_TESTGEN	(1 << 16)
#define VFE31_INPUT_SOURCE_AXI		(2 << 16)

/*
 * VFE31 VFE_CFG_OFF (0x014) - CRITICAL FINDING
 *
 * IMPORTANT: VFE31 on MSM8660 does NOT have inputSource bits at 16-17!
 *
 * Testing shows that only bits 0-7 and bit 21 persist when writing to this
 * register. Bits 8-20 and 22-31 are read-only/reserved on VFE31.
 *
 * - Wrote: 0x02a9a776
 * - Read back: 0x00a10076
 * - Only bits 0-7 (0x76) and bit 21 persisted
 *
 * The webOS libqcameralib value 0x2aaa771 was for userspace config that gets
 * filtered by the kernel driver - the kernel itself only writes pixel pattern.
 *
 * For VFE31, we should ONLY set the pixel pattern bits (0-2). Data routing
 * from CSIPHY to VFE is automatic once the CSI-VFE bridge clocks are enabled
 * and CAMIF is started. There is no inputSource selection register.
 *
 * The VFE8x "inputSource" concept (CAMIF vs CSI vs AXI) does not apply to
 * VFE31 - the hardware architecture is fundamentally different.
 */

/* VFE31 global reset and IRQ clear registers */
#define VFE31_GLOBAL_RESET_CMD		0x004
#define VFE31_IRQ_CMD			0x018
#define VFE31_IRQ_CLEAR_0		0x024
#define VFE31_IRQ_CLEAR_1		0x028

/* VFE31 Write Master registers - used for DMA buffer addresses */
#define VFE31_WM_WR_PING_ADDR(n)	(0x050 + 0x18 * (n))
#define VFE31_WM_WR_PONG_ADDR(n)	(0x054 + 0x18 * (n))
#define VFE31_WM_WR_ADDR_CFG(n)		(0x058 + 0x18 * (n))
#define VFE31_WM_WR_UB_CFG(n)		(0x05C + 0x18 * (n))
#define VFE31_WM_WR_IMAGE_SIZE(n)	(0x060 + 0x18 * (n))

/*
 * vfe31_reg_update_poll - Issue REG_UPDATE and poll until bit 0 clears
 * @vfe: VFE device
 * @timeout_us: Timeout in microseconds
 *
 * VFE31 shadow registers require REG_UPDATE to latch new values. This function
 * writes 1 to VFE_REG_UPDATE_CMD and polls until bit 0 auto-clears, indicating
 * the update completed. Without proper REG_UPDATE completion, configuration
 * writes to VFE_CFG_OFF and other shadow registers won't take effect.
 *
 * Returns 0 on success, -ETIMEDOUT if polling times out.
 */
static int vfe31_reg_update_poll(struct vfe_device *vfe, unsigned int timeout_us)
{
	u32 val;
	int ret;

	/* Write 1 to trigger register update */
	writel_relaxed(1, vfe->base + VFE31_REG_UPDATE_CMD);
	wmb();

	/* Poll until bit 0 clears (update complete) */
	ret = readl_relaxed_poll_timeout(vfe->base + VFE31_REG_UPDATE_CMD,
					 val, !(val & 1), 10, timeout_us);
	if (ret) {
		dev_warn(vfe->camss->dev,
			 "VFE31: REG_UPDATE timeout (val=0x%08x after %u us)\n",
			 val, timeout_us);
	} else {
		dev_dbg(vfe->camss->dev, "VFE31: REG_UPDATE completed\n");
	}

	return ret;
}

/*
 * vfe31_cold_reset - Perform a cold reset of VFE31 to unstick registers
 * @vfe: VFE device
 *
 * If VFE shadow registers are stuck (writes don't take effect), a full reset
 * can clear the internal state machine. This follows the webOS reset sequence:
 * 1. Disable all IRQs
 * 2. Clear all pending IRQs
 * 3. Issue global reset (0x3FF to reset all modules)
 * 4. Wait for reset completion
 * 5. Re-enable clock gates
 *
 * Note: This is a heavyweight operation - use only if REG_UPDATE polling fails.
 */
static void vfe31_cold_reset(struct vfe_device *vfe)
{
	dev_info(vfe->camss->dev, "VFE31: Performing cold reset to unstick registers\n");

	/* Step 1: Disable all IRQs */
	writel_relaxed(0x0, vfe->base + VFE31_IRQ_MASK_0);
	writel_relaxed(0x0, vfe->base + VFE31_IRQ_MASK_1);
	wmb();

	/* Step 2: Clear all pending IRQs */
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE31_IRQ_CLEAR_0);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE31_IRQ_CLEAR_1);
	writel_relaxed(1, vfe->base + VFE31_IRQ_CMD);  /* Acknowledge clear */
	wmb();

	/* Step 3: Issue global reset - 0x3FF resets all VFE modules */
	writel_relaxed(0x3FF, vfe->base + VFE31_GLOBAL_RESET_CMD);
	wmb();

	/* Step 4: Wait for reset to complete */
	usleep_range(2000, 3000);

	/* Step 5: Re-enable all clock gates */
	writel_relaxed(0xFFFFFFFF, vfe->base + 0x00C);  /* CGC_OVERRIDE */
	wmb();

	dev_info(vfe->camss->dev, "VFE31: Cold reset complete\n");
}

/*
 * vfe_enable_pending_camif - Configure and enable deferred CAMIF
 * @vfe: VFE device
 *
 * MSM8660 workaround: ALL CAMIF configuration must be deferred until after
 * CSIPHY is configured. Writing to VFE CAMIF registers before CSIPHY is
 * ready blocks CSI register access. This matches the legacy webOS kernel
 * behavior where CSI is initialized before VFE CAMIF.
 */
void vfe_enable_pending_camif(struct vfe_device *vfe)
{
	struct vfe_line *line;
	ktime_t start_time;
	u32 val;
	u32 vfe_cfg_val;  /* Save VFE_CFG value for re-application after CAMIF start */

	if (!vfe->camif_pending) {
		dev_dbg(vfe->camss->dev, "VFE: no pending CAMIF config\n");
		return;
	}

	start_time = ktime_get();
	line = &vfe->line[vfe->camif_pending_line_id];

	dev_info(vfe->camss->dev,
		 "[TIMING] VFE CAMIF CONFIG START at %lld ns - WM%d RDI%d\n",
		 ktime_to_ns(start_time), vfe->camif_pending_wm, vfe->camif_pending_line_id);

	/*
	 * Debug: Dump external registers (MMCC VFE_CC_REG, TCSR)
	 * This helps verify CSI-to-VFE clock enables and potential CSI mux settings.
	 */
	vfe31_debug_dump_external_regs(vfe->camss->dev);

	/*
	 * Debug: Dump pre-CAMIF VFE state
	 * These register reads help verify VFE power/clock state.
	 */
	dev_info(vfe->camss->dev,
		 "VFE: PRE-CONFIG state: HW_VERSION=0x%08x CGC=0x%08x\n",
		 readl_relaxed(vfe->base + 0x000),  /* VFE_HW_VERSION */
		 readl_relaxed(vfe->base + 0x00C)); /* VFE_CGC_OVERRIDE */

	/*
	 * Step 0: VBIF + Clock "Brute Force" Initialization
	 *
	 * Based on Gemini analysis: The VFE31 has an internal sub-module called
	 * VBIF (VFE Bus Interface) which is the AXI master/slave logic. If VBIF
	 * is halted or its clock gate isn't open, VFE_CFG_OFF writes may fail.
	 *
	 * Sequence:
	 * 1. Force ALL VFE clocks via CGC_OVERRIDE (including VBIF)
	 * 2. Wake up VBIF via 0x400 (VBIF_CLK_ON) and 0x404 (VBIF_AXI_CFG)
	 * 3. Clear any pending resets
	 * 4. Then configure VFE_CFG_OFF with inputSource=CSI for live camera
	 */

	/* Debug: Read VBIF state before configuration */
	dev_info(vfe->camss->dev,
		 "VFE: VBIF PRE-CONFIG: 0x400=0x%08x 0x404=0x%08x 0x408=0x%08x\n",
		 readl_relaxed(vfe->base + 0x400),
		 readl_relaxed(vfe->base + 0x404),
		 readl_relaxed(vfe->base + 0x408));

	/* Step 0a: Force ALL VFE clocks on (full 32-bit mask) */
	writel_relaxed(0xFFFFFFFF, vfe->base + 0x00C);  /* VFE_CGC_OVERRIDE */
	wmb();
	dev_info(vfe->camss->dev, "VFE: CGC_OVERRIDE=0xFFFFFFFF (all clocks forced on)\n");

	/* Step 0b: Wake up VBIF (VFE Bus Interface) */
	writel_relaxed(0x00000001, vfe->base + 0x400);  /* VBIF_CLK_ON */
	wmb();
	writel_relaxed(0x00000110, vfe->base + 0x404);  /* VBIF_AXI_CFG (standard priority) */
	wmb();
	dev_info(vfe->camss->dev,
		 "VFE: VBIF configured: 0x400=0x1, 0x404=0x110\n");

	/* Step 0c: Clear any pending resets */
	writel_relaxed(0, vfe->base + 0x00C);
	wmb();
	udelay(10);
	/* Re-enable CGC after reset clear */
	writel_relaxed(0xFFFFFFFF, vfe->base + 0x00C);
	wmb();

	/* Debug: Read VBIF state after configuration */
	dev_info(vfe->camss->dev,
		 "VFE: VBIF POST-CONFIG: 0x400=0x%08x 0x404=0x%08x 0x408=0x%08x\n",
		 readl_relaxed(vfe->base + 0x400),
		 readl_relaxed(vfe->base + 0x404),
		 readl_relaxed(vfe->base + 0x408));

	/* Set DEMUX gains to passthrough (required for YUV input) */
	writel_relaxed(0x800080, vfe->base + 0x288); /* VFE_DEMUX_GAIN_0 */
	writel_relaxed(0x800080, vfe->base + 0x28C); /* VFE_DEMUX_GAIN_1 */

	/* Set clamp values for output */
	writel_relaxed(0x00ffffff, vfe->base + 0x524); /* VFE_CLAMP_ENC_MAX_CFG */
	writel_relaxed(0x0, vfe->base + 0x528);        /* VFE_CLAMP_ENC_MIN_CFG */

	/*
	 * Step 0d: Enable modules BEFORE writing VFE_CFG_OFF
	 *
	 * VFE_MODULE_CFG bits:
	 *   Bit 2: demuxEnable - required for YUV data demuxing
	 *   Bit 3: chromaUpsampleEnable - needed for YUV422 to YUV444
	 *
	 * Note: We don't poll REG_UPDATE here because it requires VSYNC to
	 * complete, and VSYNC requires active data flow which we don't have
	 * yet. The config will latch on the first SOF after CAMIF starts.
	 */
	dev_info(vfe->camss->dev, "VFE: Step 0d - Enable modules\n");
	writel_relaxed(0x0C, vfe->base + VFE31_MODULE_CFG);  /* DEMUX + CHROMA_UPSAMPLE */
	wmb();
	dev_info(vfe->camss->dev, "VFE: VFE_MODULE_CFG=0x0C (DEMUX+CHROMA enabled)\n");

	/*
	 * Step 0b: Write VFE_CFG_OFF with pixel pattern ONLY
	 *
	 * CRITICAL: VFE31 does NOT have inputSource bits (16-17)!
	 * Only the pixel pattern (bits 0-2) is writable. Data routing from
	 * CSIPHY to VFE is automatic once clocks are enabled and CAMIF starts.
	 *
	 * Do NOT use VFE31_CFG_WEBOS_BASE - those bits don't persist on VFE31.
	 */
	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		val = VFE31_PIXEL_PATTERN_YUV_YCbYCr;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		val = VFE31_PIXEL_PATTERN_YUV_YCrYCb;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	default:
		val = VFE31_PIXEL_PATTERN_YUV_CbYCrY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		val = VFE31_PIXEL_PATTERN_YUV_CrYCbY;
		break;
	}
	vfe_cfg_val = val;
	dev_info(vfe->camss->dev,
		 "VFE: Step 0b - VFE_CFG_OFF=0x%08x (pixel pattern only)\n", val);
	writel(val, vfe->base + VFE31_CFG_OFF);
	wmb();

	/*
	 * Skip REG_UPDATE polling for initial config.
	 *
	 * VFE31 REG_UPDATE requires VSYNC to latch, but CAMIF hasn't started
	 * yet so there's no VSYNC. The config will latch on the first SOF
	 * after CAMIF_START. This matches webOS behavior - it doesn't poll
	 * REG_UPDATE during initial config.
	 */
	dev_info(vfe->camss->dev,
		 "VFE: VFE_CFG_OFF set (will latch on first SOF after CAMIF start)\n");

	wmb();

	/* Step 1: Configure CORE_CFG pixel pattern */
	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		val = VFE31_CORE_CFG_PIXEL_YCBYCR;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		val = VFE31_CORE_CFG_PIXEL_YCRYCB;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	default:
		val = VFE31_CORE_CFG_PIXEL_CBYCRY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		val = VFE31_CORE_CFG_PIXEL_CRYCBY;
		break;
	}
	writel_relaxed(val, vfe->base + VFE31_CORE_CFG);

	/*
	 * Step 2: Configure EFS_CFG at 0x1E4
	 *
	 * VFE31 CAMIF Register Layout Analysis (based on webOS msm_vfe31.h/vfe8x_proc.h):
	 *
	 * IMPORTANT: VFE31 is DIFFERENT from VFE8x!
	 *
	 * VFE8x has: CAMIF_CONFIG at 0x114 with camif2vfeEnable (bit 8) and
	 *            camif2busEnable (bit 10) for data routing.
	 *
	 * VFE31 does NOT have those routing bits. Instead:
	 * - 0x1E0: CAMIF_COMMAND (start=1, stop=2, clear=4)
	 * - 0x1E4: EFS_CFG (32-byte config block starts here)
	 *          Bits 7:0   = efsEndOfLine (for EFS sync mode)
	 *          Bits 15:8  = efsStartOfLine
	 *          Bits 23:16 = efsEndOfFrame
	 *          Bits 31:24 = efsStartOfFrame
	 * - 0x1E8: FRAME_CFG
	 * - 0x1EC: WINDOW_WIDTH_CFG
	 * - 0x1F0: WINDOW_HEIGHT_CFG
	 * - 0x1F4: SUBSAMPLE_CFG_0
	 * - 0x1F8: SUBSAMPLE_CFG_1
	 * - 0x1FC: EPOCH_CFG
	 * - 0x204: CAMIF_STATUS
	 *
	 * For VFE31, data routing is controlled via:
	 * - AXI output mode at 0x040 (0x60 for raw, 0x200 for preview)
	 * - Write master configuration
	 *
	 * For APS mode (Active Pixel Sync, used with MIPI CSI-2), EFS_CFG should
	 * be 0 since we don't use embedded sync codes.
	 */
	dev_info(vfe->camss->dev, "VFE: EFS_CFG=0x0 at 0x1E4 (APS mode, no embedded sync)\n");
	writel_relaxed(0, vfe->base + VFE31_CAMIF_EFS_CFG);

	/* Step 3: Configure CAMIF frame dimensions at 0x1E8 */
	val = line->fmt[MSM_VFE_PAD_SINK].width * 2;
	val |= line->fmt[MSM_VFE_PAD_SINK].height << 16;
	dev_info(vfe->camss->dev, "VFE: FRAME_CFG=0x%08x at 0x%03x\n",
		 val, VFE31_CAMIF_FRAME_CFG);
	writel_relaxed(val, vfe->base + VFE31_CAMIF_FRAME_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].width * 2 - 1;
	writel_relaxed(val, vfe->base + VFE31_CAMIF_WINDOW_WIDTH_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].height - 1;
	writel_relaxed(val, vfe->base + VFE31_CAMIF_WINDOW_HEIGHT_CFG);

	writel_relaxed(0xffffffff, vfe->base + VFE31_CAMIF_SUBSAMPLE_CFG_0);
	writel_relaxed(0xffffffff, vfe->base + VFE31_CAMIF_SUBSAMPLE_CFG_1);

	/*
	 * EPOCH_CFG at 0x200 - epoch interrupt timing.
	 * webOS vfe_camifcfg structure shows epoch1Line and epoch2Line fields.
	 * Set to reasonable defaults (line counts for timing interrupts).
	 * Use height/2 for epoch1 and height-1 for epoch2.
	 */
	val = ((line->fmt[MSM_VFE_PAD_SINK].height / 2) & 0x3FFF) |       /* epoch1Line */
	      (((line->fmt[MSM_VFE_PAD_SINK].height - 1) & 0x3FFF) << 16); /* epoch2Line */
	dev_info(vfe->camss->dev, "VFE: EPOCH_CFG=0x%08x (epoch1=%d epoch2=%d)\n",
		 val, line->fmt[MSM_VFE_PAD_SINK].height / 2,
		 line->fmt[MSM_VFE_PAD_SINK].height - 1);
	writel_relaxed(val, vfe->base + VFE31_CAMIF_EPOCH_CFG);

	/*
	 * Step 2b: Enable write masters via VFE_BUS_CMD (0x38)
	 * webOS writes 0x7FFF here to "reload all write masters (frame & line)"
	 * This prepares write masters for receiving data from CAMIF.
	 */
#define VFE31_BUS_CMD			0x038
#define VFE31_BUS_CMD_RELOAD_WM		0x7FFF
	dev_info(vfe->camss->dev,
		 "VFE: BUS_CMD writing 0x%04x (reload write masters)\n",
		 VFE31_BUS_CMD_RELOAD_WM);
	writel_relaxed(VFE31_BUS_CMD_RELOAD_WM, vfe->base + VFE31_BUS_CMD);
	wmb();

	/*
	 * Step 3: Configure AXI output mode based on PIX vs RDI mode.
	 *
	 * NOTE: Do NOT write to 0x03C! On VFE31, the 188-byte AXI config block
	 * starts at 0x038, and 0x03C (ao[1]) must be left at 0.
	 * The Ghidra analysis of webOS libqcameralib.so confirmed this:
	 * - ao[0] at 0x038: AXI output base config
	 * - ao[1] at 0x03C: Must be 0 (webOS never writes here)
	 * - ao[2] at 0x040: AXI output mode (0x200 for preview, 0x60 for raw)
	 *
	 * AXI output mode values:
	 *   PIX mode (preview):   0x200 (OUTPUT_2) - data flows through VFE ISP
	 *   RDI mode (raw):       0x60  (CAMIF_TO_AXI) - data bypasses VFE ISP
	 */
	if (vfe->camif_pending_line_id == VFE_LINE_PIX) {
		dev_info(vfe->camss->dev,
			 "VFE: AXI_OUT_MODE (PIX) = 0x%03x (OUTPUT_2/preview)\n",
			 VFE31_AXI_OUT_MODE_PREVIEW);
		writel_relaxed(VFE31_AXI_OUT_MODE_PREVIEW,
			       vfe->base + VFE31_AXI_OUT_MODE_CFG);
	} else {
		dev_info(vfe->camss->dev,
			 "VFE: AXI_OUT_MODE (RDI) = 0x%03x (CAMIF_TO_AXI/raw)\n",
			 VFE31_AXI_OUT_MODE_RAW_SNAPSHOT);
		writel_relaxed(VFE31_AXI_OUT_MODE_RAW_SNAPSHOT,
			       vfe->base + VFE31_AXI_OUT_MODE_CFG);
	}
	wmb();

	/*
	 * NOTE: WM buffer addresses (ping/pong, image_size, addr_cfg, ub_cfg)
	 * are already configured by vfe31_enable() BEFORE this function runs.
	 * Do NOT overwrite them here - vfe31_enable() gets addresses from
	 * output->buf[0/1]->addr[0] which are the correct DMA buffer addresses.
	 */

	/*
	 * Step 4: Clear CAMIF status before configuring
	 * This ensures CAMIF is in a clean state to accept new configuration.
	 *
	 * NOTE: VFE31 does NOT have camif2vfeEnable/camif2busEnable routing bits
	 * at 0x1E4. Those bits exist in VFE8x at 0x114, not in VFE31.
	 * Data routing on VFE31 is controlled solely via AXI_OUT_MODE at 0x040.
	 */
	writel_relaxed(VFE31_CAMIF_CMD_CLEAR_STATUS, vfe->base + VFE31_CAMIF_CMD);
	wmb();
	udelay(10);

	dev_info(vfe->camss->dev,
		 "VFE: AXI_OUT_MODE=0x%08x, EFS_CFG=0x%08x (routing via AXI mode only)\n",
		 readl_relaxed(vfe->base + VFE31_AXI_OUT_MODE_CFG),
		 readl_relaxed(vfe->base + VFE31_CAMIF_EFS_CFG));

	/*
	 * Step 5c: Enable IRQs - CRITICAL!
	 * Without proper IRQ mask, VFE cannot signal frame completion.
	 *
	 * Use EXACT webOS IRQ mask values from vfe31_start_common():
	 *
	 * IRQ_MASK_0 = 0x00EFE021:
	 *   Bit 0:  CAMIF_SOF              - Start of frame
	 *   Bit 5:  REG_UPDATE             - Register update done
	 *   Bit 13-19: STATS_*             - Statistics IRQs (AEC,AF,AWB,RS,CS,IHIST,SKIN)
	 *   Bit 21-23: IMAGE_COMPOSIT_DONE - Frame complete for output paths 0/1/2
	 *
	 * IRQ_MASK_1 = 0x00400000:
	 *   Bit 22: RESET_ACK              - Hardware reset complete
	 *
	 * NOTE: webOS does NOT enable CAMIF_EOF (bit 1) or individual WM ping/pong
	 * IRQs (bits 8-12). It uses IMAGE_COMPOSIT_DONE for frame completion instead.
	 */
	vfe->irq_mask0_shadow = VFE31_IRQ_MASK_0_WEBOS;
	vfe->irq_mask1_shadow = VFE31_IRQ_MASK_1_WEBOS;

	dev_info(vfe->camss->dev,
		 "VFE: Enabling IRQs: MASK_0=0x%08x MASK_1=0x%08x\n",
		 vfe->irq_mask0_shadow, vfe->irq_mask1_shadow);

	writel_relaxed(vfe->irq_mask0_shadow, vfe->base + VFE31_IRQ_MASK_0);
	writel_relaxed(vfe->irq_mask1_shadow, vfe->base + VFE31_IRQ_MASK_1);
	wmb();

	/*
	 * Step 6: REG_UPDATE then START CAMIF
	 * From webOS vfe31_start_common(): write 1 to REG_UPDATE_CMD with barrier,
	 * then write 1 to CAMIF_COMMAND. No separate CLEAR step.
	 *
	 * IMPORTANT: VFE31 uses shadow registers. All configuration writes go to
	 * shadow registers first. Writing to REG_UPDATE_CMD latches the shadow
	 * register values into the active configuration. The VFE will set
	 * IRQ_STATUS_0 bit 5 (REG_UPDATE) when the latch is complete.
	 */
	dev_info(vfe->camss->dev,
		 "VFE: CAMIF_STATUS before REG_UPDATE: 0x%08x\n",
		 readl_relaxed(vfe->base + VFE31_CAMIF_STATUS));

	/*
	 * Step 6: REG_UPDATE + CAMIF_START (webOS style)
	 *
	 * From webOS vfe31_start_common(): issues REG_UPDATE_CMD immediately
	 * followed by CAMIF_COMMAND=START, with no polling in between.
	 * The REG_UPDATE IRQ will fire later once CAMIF is running and
	 * processing data.
	 *
	 * Previous code polled for REG_UPDATE completion before starting
	 * CAMIF, but this doesn't match webOS behavior and may cause timeouts
	 * since the VFE might need active data flow to complete REG_UPDATE.
	 */
	dev_info(vfe->camss->dev, "VFE: Issuing REG_UPDATE + CAMIF_START (webOS style)\n");

	/* REG_UPDATE to latch shadow registers */
	writel(1, vfe->base + VFE31_REG_UPDATE_CMD);
	wmb();

	/* Immediately start CAMIF (no waiting for REG_UPDATE IRQ) */
	writel(VFE31_CAMIF_CMD_START, vfe->base + VFE31_CAMIF_CMD);
	wmb();

	/* Small delay then check status */
	udelay(100);
	dev_info(vfe->camss->dev,
		 "VFE: After START: CAMIF_STATUS=0x%08x IRQ_STATUS0=0x%08x IRQ_STATUS1=0x%08x\n",
		 readl_relaxed(vfe->base + VFE31_CAMIF_STATUS),
		 readl_relaxed(vfe->base + 0x02C),  /* VFE_IRQ_STATUS_0 */
		 readl_relaxed(vfe->base + 0x030)); /* VFE_IRQ_STATUS_1 */

	/*
	 * VFE31: Re-apply pixel pattern after CAMIF start.
	 * This ensures the shadow register latches with the first SOF.
	 *
	 * Note: VFE31 does NOT have inputSource bits - data routing is
	 * automatic via the CSI-VFE bridge clocks. We only need to ensure
	 * the pixel pattern is set correctly.
	 */
	{
		u32 cfg_before = readl(vfe->base + VFE31_CFG_OFF);
		writel(vfe_cfg_val, vfe->base + VFE31_CFG_OFF);
		wmb();

		dev_info(vfe->camss->dev,
			 "VFE: Re-applied VFE_CFG_OFF: before=0x%08x after=0x%08x (pixel=%d)\n",
			 cfg_before, readl(vfe->base + VFE31_CFG_OFF), vfe_cfg_val & 0x7);
	}

	vfe->camif_pending = false;

	/* Complete streaming setup - set output state */
	line->output.state = VFE_OUTPUT_ON;
	/* Note: stream_count is already incremented in vfe31_enable() */

	dev_info(vfe->camss->dev,
		 "VFE: CAMIF configured and streaming started (stream_count=%d)\n",
		 vfe->stream_count);
	dev_info(vfe->camss->dev,
		 "VFE: status=0x%08x efs_cfg=0x%08x frame=0x%08x epoch=0x%08x ping_pong=0x%08x\n",
		 readl_relaxed(vfe->base + VFE31_CAMIF_STATUS),
		 readl_relaxed(vfe->base + VFE31_CAMIF_EFS_CFG),
		 readl_relaxed(vfe->base + VFE31_CAMIF_FRAME_CFG),
		 readl_relaxed(vfe->base + VFE31_CAMIF_EPOCH_CFG),
		 readl_relaxed(vfe->base + VFE31_BUS_PING_PONG_STATUS));
	dev_info(vfe->camss->dev,
		 "VFE: axi_mode=0x%08x subsamp0=0x%08x subsamp1=0x%08x\n",
		 readl_relaxed(vfe->base + VFE31_AXI_OUT_MODE_CFG),
		 readl_relaxed(vfe->base + VFE31_CAMIF_SUBSAMPLE_CFG_0),
		 readl_relaxed(vfe->base + VFE31_CAMIF_SUBSAMPLE_CFG_1));
	dev_info(vfe->camss->dev,
		 "VFE: vfe_cfg=0x%08x module_cfg=0x%08x axi_cfg_1=0x%08x\n",
		 readl_relaxed(vfe->base + VFE31_CFG_OFF),
		 readl_relaxed(vfe->base + VFE31_MODULE_CFG),
		 readl_relaxed(vfe->base + VFE31_AXI_CFG_1));
	/*
	 * Note: VFE31 IRQ_MASK_0/1 are write-only registers - do NOT read them!
	 * Use shadow values instead.
	 */
	dev_info(vfe->camss->dev,
		 "VFE: IRQ_MASK_0=0x%08x (shadow) IRQ_MASK_1=0x%08x (shadow)\n",
		 vfe->irq_mask0_shadow, vfe->irq_mask1_shadow);

	dev_info(vfe->camss->dev,
		 "[TIMING] VFE CAMIF CONFIG DONE elapsed=%lld ns - VFE is now ready for data!\n",
		 ktime_to_ns(ktime_get()) - ktime_to_ns(start_time));
}

static void vfe_init_outputs(struct vfe_device *vfe)
{
	int i;

	for (i = 0; i < vfe->res->line_num; i++) {
		struct vfe_output *output = &vfe->line[i].output;

		output->state = VFE_OUTPUT_OFF;
		output->buf[0] = NULL;
		output->buf[1] = NULL;
		INIT_LIST_HEAD(&output->pending_bufs);
	}
}

static void vfe_reset_output_maps(struct vfe_device *vfe)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++)
		vfe->wm_output_map[i] = VFE_LINE_NONE;
}

int vfe_reserve_wm(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	int ret = -EBUSY;
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++) {
		if (vfe->wm_output_map[i] == VFE_LINE_NONE) {
			vfe->wm_output_map[i] = line_id;
			ret = i;
			break;
		}
	}

	return ret;
}

int vfe_release_wm(struct vfe_device *vfe, u8 wm)
{
	if (wm >= ARRAY_SIZE(vfe->wm_output_map))
		return -EINVAL;

	vfe->wm_output_map[wm] = VFE_LINE_NONE;

	return 0;
}

struct camss_buffer *vfe_buf_get_pending(struct vfe_output *output)
{
	struct camss_buffer *buffer = NULL;

	if (!list_empty(&output->pending_bufs)) {
		buffer = list_first_entry(&output->pending_bufs,
					  struct camss_buffer,
					  queue);
		list_del(&buffer->queue);
	}

	return buffer;
}

void vfe_buf_add_pending(struct vfe_output *output,
			 struct camss_buffer *buffer)
{
	INIT_LIST_HEAD(&buffer->queue);
	list_add_tail(&buffer->queue, &output->pending_bufs);
}

/*
 * vfe_buf_flush_pending - Flush all pending buffers.
 * @output: VFE output
 * @state: vb2 buffer state
 */
static void vfe_buf_flush_pending(struct vfe_output *output,
				  enum vb2_buffer_state state)
{
	struct camss_buffer *buf;
	struct camss_buffer *t;

	list_for_each_entry_safe(buf, t, &output->pending_bufs, queue) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->queue);
	}
}

int vfe_put_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&vfe->output_lock, flags);

	for (i = 0; i < output->wm_num; i++)
		vfe_release_wm(vfe, output->wm_idx[i]);

	output->state = VFE_OUTPUT_OFF;

	spin_unlock_irqrestore(&vfe->output_lock, flags);
	return 0;
}

static int vfe_disable_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&vfe->output_lock, flags);
	for (i = 0; i < output->wm_num; i++)
		vfe->res->hw_ops->vfe_wm_stop(vfe, output->wm_idx[i]);
	output->gen2.active_num = 0;
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return vfe_reset(vfe);
}

/*
 * vfe_disable - Disable streaming on VFE line
 * @line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_disable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	int ret;

	ret = vfe_disable_output(line);
	if (ret)
		goto error;

	vfe_put_output(line);

	mutex_lock(&vfe->stream_lock);

	vfe->stream_count--;

	mutex_unlock(&vfe->stream_lock);

error:
	return ret;
}

/**
 * vfe_isr_comp_done() - Process composite image done interrupt
 * @vfe: VFE Device
 * @comp: Composite image id
 */
void vfe_isr_comp_done(struct vfe_device *vfe, u8 comp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++)
		if (vfe->wm_output_map[i] == VFE_LINE_PIX) {
			vfe->isr_ops.wm_done(vfe, i);
			break;
		}
}

void vfe_isr_reset_ack(struct vfe_device *vfe)
{
	complete(&vfe->reset_complete);
}

/*
 * vfe_trigger_software_sof - Trigger software-generated SOF for VFE
 * @vfe: VFE device
 * @line_id: VFE line to send SOF to (usually VFE_LINE_PIX for raw)
 *
 * MSM8660 workaround: Some sensors (like MT9M113) don't send MIPI Frame
 * Start/End short packets, so VFE never receives CAMIF_SOF interrupts.
 * This function allows CSIPHY to trigger software SOF when it detects
 * frame boundaries through other means (e.g., SOT after idle gap).
 */
void vfe_trigger_software_sof(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	if (!vfe || line_id >= VFE_LINE_NUM_MAX)
		return;

	/*
	 * Call the VFE's SOF handler directly. This will update frame
	 * counters and notify any waiting clients of the frame start.
	 */
	if (vfe->isr_ops.sof)
		vfe->isr_ops.sof(vfe, line_id);
}
EXPORT_SYMBOL_GPL(vfe_trigger_software_sof);

/*
 * vfe_pm_domain_off - Disable power domains specific to this VFE.
 * @vfe: VFE Device
 */
void vfe_pm_domain_off(struct vfe_device *vfe)
{
	if (!vfe->genpd)
		return;

	device_link_del(vfe->genpd_link);
	vfe->genpd_link = NULL;
}

/*
 * vfe_pm_domain_on - Enable power domains specific to this VFE.
 * @vfe: VFE Device
 */
int vfe_pm_domain_on(struct vfe_device *vfe)
{
	struct camss *camss = vfe->camss;

	if (!vfe->genpd)
		return 0;

	vfe->genpd_link = device_link_add(camss->dev, vfe->genpd,
					  DL_FLAG_STATELESS |
					  DL_FLAG_PM_RUNTIME |
					  DL_FLAG_RPM_ACTIVE);
	if (!vfe->genpd_link)
		return -EINVAL;

	return 0;
}

static int vfe_match_clock_names(struct vfe_device *vfe,
				 struct camss_clock *clock)
{
	char vfe_name[7]; /* vfeXXX\0 */
	char vfe_lite_name[12]; /* vfe_liteXXX\0 */

	snprintf(vfe_name, sizeof(vfe_name), "vfe%d", vfe->id);
	snprintf(vfe_lite_name, sizeof(vfe_lite_name), "vfe_lite%d", vfe->id);

	return (!strcmp(clock->name, vfe_name) ||
		!strcmp(clock->name, vfe_lite_name) ||
		!strcmp(clock->name, "vfe_lite") ||
		!strcmp(clock->name, "vfe") ||		/* MSM8660 uses "vfe" not "vfe0" */
		!strcmp(clock->name, "camnoc_axi"));
}

/*
 * vfe_check_clock_levels - Calculate and set clock rates on VFE module
 * @clock: clocks data
 *
 * Return false if there is no non-zero clock level and true otherwise.
 */
static bool vfe_check_clock_levels(struct camss_clock *clock)
{
	int i;

	for (i = 0; i < clock->nfreqs; i++)
		if (clock->freq[i])
			return true;
	return false;
}

/*
 * vfe_set_clock_rates - Calculate and set clock rates on VFE module
 * @vfe: VFE device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_set_clock_rates(struct vfe_device *vfe)
{
	struct device *dev = vfe->camss->dev;
	u64 pixel_clock[VFE_LINE_NUM_MAX];
	int i, j;
	int ret;

	for (i = VFE_LINE_RDI0; i < vfe->res->line_num; i++) {
		ret = camss_get_pixel_clock(&vfe->line[i].subdev.entity,
					    &pixel_clock[i]);
		if (ret)
			pixel_clock[i] = 0;
	}

	for (i = 0; i < vfe->nclocks; i++) {
		struct camss_clock *clock = &vfe->clock[i];

		if (vfe_match_clock_names(vfe, clock) && vfe_check_clock_levels(clock)) {
			u64 min_rate = 0;
			long rate;

			for (j = VFE_LINE_RDI0; j < vfe->res->line_num; j++) {
				u32 tmp;
				u8 bpp;

				if (j == VFE_LINE_PIX) {
					tmp = pixel_clock[j];
				} else {
					struct vfe_line *l = &vfe->line[j];

					bpp = camss_format_get_bpp(l->formats,
								   l->nformats,
								   l->fmt[MSM_VFE_PAD_SINK].code);
					tmp = pixel_clock[j] * bpp / 64;
				}

				if (min_rate < tmp)
					min_rate = tmp;
			}

			camss_add_clock_margin(&min_rate);

			for (j = 0; j < clock->nfreqs; j++)
				if (min_rate < clock->freq[j])
					break;

			if (j == clock->nfreqs) {
				dev_err(dev,
					"Pixel clock is too high for VFE");
				return -EINVAL;
			}

			/*
			 * If sensor pixel clock is not available, or for VFE31
			 * (MSM8660) which requires high clock for raw passthrough,
			 * use the highest possible VFE clock rate.
			 *
			 * VFE31 on MSM8660 always uses 228.57 MHz per webOS kernel.
			 * The dynamic calculation doesn't work well for raw mode.
			 */
			if (min_rate == 0 || vfe->res->hw_ops == &vfe_ops_3_1)
				j = clock->nfreqs - 1;

			dev_info(dev, "VFE clock %s: min_rate=%llu j=%d freq[j]=%u nfreqs=%d\n",
				 clock->name, min_rate, j, clock->freq[j], clock->nfreqs);

			rate = clk_round_rate(clock->clk, clock->freq[j]);
			if (rate < 0) {
				dev_err(dev, "clk round rate failed: %ld\n",
					rate);
				return -EINVAL;
			}

			dev_info(dev, "VFE clock %s: requested=%u rounded=%ld\n",
				 clock->name, clock->freq[j], rate);

			/*
			 * MSM8660 workaround: Skip clk_set_rate if current
			 * rate already matches. MSM8660 hangs if you write
			 * to MMCC registers while the clock is enabled.
			 * CSIPHY may have already set the rate before enabling.
			 */
			if (clk_get_rate(clock->clk) == rate) {
				dev_info(dev,
					 "VFE clock %s: already at %ld Hz, skipping set_rate\n",
					 clock->name, rate);
			} else {
				ret = clk_set_rate(clock->clk, rate);
				if (ret < 0) {
					dev_err(dev, "clk set rate failed: %d\n",
						ret);
					return ret;
				}
				dev_info(dev, "VFE clock %s: set to %ld Hz\n",
					 clock->name, clk_get_rate(clock->clk));
			}
		}
	}

	return 0;
}

/*
 * vfe_check_clock_rates - Check current clock rates on VFE module
 * @vfe: VFE device
 *
 * Return 0 if current clock rates are suitable for a new pipeline
 * or a negative error code otherwise
 */
static int vfe_check_clock_rates(struct vfe_device *vfe)
{
	u64 pixel_clock[VFE_LINE_NUM_MAX];
	int i, j;
	int ret;

	for (i = VFE_LINE_RDI0; i < vfe->res->line_num; i++) {
		ret = camss_get_pixel_clock(&vfe->line[i].subdev.entity,
					    &pixel_clock[i]);
		if (ret)
			pixel_clock[i] = 0;
	}

	for (i = 0; i < vfe->nclocks; i++) {
		struct camss_clock *clock = &vfe->clock[i];

		if (vfe_match_clock_names(vfe, clock) && vfe_check_clock_levels(clock)) {
			u64 min_rate = 0;
			unsigned long rate;

			for (j = VFE_LINE_RDI0; j < vfe->res->line_num; j++) {
				u32 tmp;
				u8 bpp;

				if (j == VFE_LINE_PIX) {
					tmp = pixel_clock[j];
				} else {
					struct vfe_line *l = &vfe->line[j];

					bpp = camss_format_get_bpp(l->formats,
								   l->nformats,
								   l->fmt[MSM_VFE_PAD_SINK].code);
					tmp = pixel_clock[j] * bpp / 64;
				}

				if (min_rate < tmp)
					min_rate = tmp;
			}

			camss_add_clock_margin(&min_rate);

			rate = clk_get_rate(clock->clk);
			if (rate < min_rate)
				return -EBUSY;
		}
	}

	return 0;
}

/*
 * vfe_get - Power up and reset VFE module
 * @vfe: VFE Device
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_get(struct vfe_device *vfe)
{
	int ret;

	mutex_lock(&vfe->power_lock);

	if (vfe->power_count == 0) {
		ret = vfe->res->hw_ops->pm_domain_on(vfe);
		if (ret < 0)
			goto error_pm_domain;

		ret = pm_runtime_resume_and_get(vfe->camss->dev);
		if (ret < 0)
			goto error_domain_off;

		/* Enable clocks before setting rates - QCOM clock framework
		 * requires clocks to be enabled for rate changes to take effect
		 */
		ret = camss_enable_clocks(vfe->nclocks, vfe->clock,
					  vfe->camss->dev);
		if (ret < 0)
			goto error_pm_runtime_get;

		/* Debug: Check MMCC state immediately after clock enable */
		if (vfe->camss->res->version == CAMSS_8x60) {
			void __iomem *mmcc_base;
			struct clk *csi1_clk;

			/*
			 * MSM8660: Set csi_pix and csi_rdi clock parents to CSI1.
			 *
			 * The MT9M113 front camera is connected to CSIPHY1/CSI1.
			 * By default, csi_pix and csi_rdi clock muxes select CSI0
			 * as their parent. We need to switch them to CSI1.
			 *
			 * Clock indices in MSM8660 VFE config:
			 *   5 = csi_rdi (Raw Data Interface path)
			 *   6 = csi_pix (Pixel Interface path)
			 */
			csi1_clk = devm_clk_get(vfe->camss->dev, "csi1");
			if (!IS_ERR(csi1_clk)) {
				/*
				 * MSM8660 CSI mux parent setting:
				 *
				 * The csi_pix and csi_rdi clocks are already enabled by
				 * camss_enable_clocks(). When we call clk_set_parent()
				 * on an enabled clock, the clock framework should handle
				 * the parent migration. However, to ensure the new parent
				 * is properly enabled, we cycle the clock after parent change.
				 */

				/* Set csi_rdi parent to CSI1 (index 5) */
				if (vfe->nclocks > 5 && vfe->clock[5].clk) {
					ret = clk_set_parent(vfe->clock[5].clk, csi1_clk);
					if (ret) {
						dev_warn(vfe->camss->dev,
							 "VFE: Failed to set csi_rdi parent to CSI1: %d\n",
							 ret);
					} else {
						dev_info(vfe->camss->dev,
							 "VFE: csi_rdi parent set to CSI1\n");
						/* Verify parent was actually set */
						if (clk_get_parent(vfe->clock[5].clk) != csi1_clk) {
							dev_err(vfe->camss->dev,
								"VFE: ERROR - csi_rdi parent mismatch after set!\n");
						}
					}
				}
				/* Set csi_pix parent to CSI1 (index 6) */
				if (vfe->nclocks > 6 && vfe->clock[6].clk) {
					ret = clk_set_parent(vfe->clock[6].clk, csi1_clk);
					if (ret) {
						dev_warn(vfe->camss->dev,
							 "VFE: Failed to set csi_pix parent to CSI1: %d\n",
							 ret);
					} else {
						dev_info(vfe->camss->dev,
							 "VFE: csi_pix parent set to CSI1\n");
						/* Verify parent was actually set */
						if (clk_get_parent(vfe->clock[6].clk) != csi1_clk) {
							dev_err(vfe->camss->dev,
								"VFE: ERROR - csi_pix parent mismatch after set!\n");
						}
					}
				}

				/*
				 * Ensure CSI1 clock is explicitly enabled.
				 * The parent clock should be enabled when child is enabled,
				 * but let's be explicit about it.
				 */
				ret = clk_prepare_enable(csi1_clk);
				if (ret) {
					dev_warn(vfe->camss->dev,
						 "VFE: Failed to enable CSI1 clock: %d\n", ret);
				} else {
					dev_info(vfe->camss->dev,
						 "VFE: CSI1 clock explicitly enabled\n");
				}
			} else {
				dev_warn(vfe->camss->dev,
					 "VFE: Could not get csi1 clock for parent setting: %ld\n",
					 PTR_ERR(csi1_clk));
			}

			mmcc_base = ioremap(0x04000000, 0x1000);
			if (mmcc_base) {
				u32 vfe_cc = readl_relaxed(mmcc_base + 0x0104);
				u32 misc_cc = readl_relaxed(mmcc_base + 0x0058);
				dev_info(vfe->camss->dev,
					 "VFE: MMCC VFE_CC_REG after clk enable: 0x%08x "
					 "(CSI0_VFE=%s, CSI1_VFE=%s)\n",
					 vfe_cc,
					 (vfe_cc & BIT(12)) ? "ON" : "off",
					 (vfe_cc & BIT(10)) ? "ON" : "off");
				dev_info(vfe->camss->dev,
					 "VFE: MMCC MISC_CC_REG: 0x%08x "
					 "(csi_pix_sel=%s, csi_pix_en=%s, csi_rdi_sel=%s, csi_rdi_en=%s)\n",
					 misc_cc,
					 (misc_cc & BIT(25)) ? "CSI1" : "CSI0",
					 (misc_cc & BIT(26)) ? "ON" : "off",
					 (misc_cc & BIT(12)) ? "CSI1" : "CSI0",
					 (misc_cc & BIT(13)) ? "ON" : "off");

				/* Verify CSI1 data path is fully enabled */
				if (!(vfe_cc & BIT(10))) {
					dev_err(vfe->camss->dev,
						"VFE: ERROR - CSI1_VFE_CLK not enabled! Data path blocked.\n");
				}
				if (!(misc_cc & BIT(25))) {
					dev_err(vfe->camss->dev,
						"VFE: ERROR - csi_pix_sel not set to CSI1! Using CSI0 instead.\n");
				}
				if (!(misc_cc & BIT(26))) {
					dev_err(vfe->camss->dev,
						"VFE: ERROR - csi_pix_clk not enabled! Data path blocked.\n");
				}

				iounmap(mmcc_base);
			}
		}

		ret = vfe_set_clock_rates(vfe);
		if (ret < 0)
			goto error_reset;

		ret = vfe_reset(vfe);
		if (ret < 0)
			goto error_reset;

		dev_info(vfe->camss->dev, "VFE get: reset done, setting camif_pending=false\n");

		/* Ensure clean CAMIF state for MSM8660 deferred enable */
		vfe->camif_pending = false;

		dev_info(vfe->camss->dev, "VFE get: calling vfe_reset_output_maps\n");
		vfe_reset_output_maps(vfe);

		dev_info(vfe->camss->dev, "VFE get: calling vfe_init_outputs\n");
		vfe_init_outputs(vfe);

		dev_info(vfe->camss->dev, "VFE get: calling hw_version\n");
		vfe->res->hw_ops->hw_version(vfe);

		dev_info(vfe->camss->dev, "VFE get: all init complete\n");
	} else {
		ret = vfe_check_clock_rates(vfe);
		if (ret < 0)
			goto error_pm_domain;
	}
	vfe->power_count++;

	mutex_unlock(&vfe->power_lock);

	return 0;

error_reset:
	camss_disable_clocks(vfe->nclocks, vfe->clock);

error_pm_runtime_get:
	pm_runtime_put_sync(vfe->camss->dev);
error_domain_off:
	vfe->res->hw_ops->pm_domain_off(vfe);

error_pm_domain:
	mutex_unlock(&vfe->power_lock);

	return ret;
}

/*
 * vfe_put - Power down VFE module
 * @vfe: VFE Device
 */
void vfe_put(struct vfe_device *vfe)
{
	mutex_lock(&vfe->power_lock);

	if (vfe->power_count == 0) {
		dev_err(vfe->camss->dev, "vfe power off on power_count == 0\n");
		goto exit;
	} else if (vfe->power_count == 1) {
		if (vfe->was_streaming) {
			vfe->was_streaming = 0;
			vfe->res->hw_ops->vfe_halt(vfe);
		}
		/*
		 * MSM8660 workaround: Disable CAMIF to ensure clean state.
		 * Stale CAMIF state can block CSIPHY register access on
		 * subsequent camera sessions.
		 *
		 * Note: VFE31 does not have CAMIF_CFG, so we clear EFS_CFG
		 * and stop CAMIF via the command register.
		 */
		writel_relaxed(0, vfe->base + VFE31_CAMIF_CMD);
		writel_relaxed(0, vfe->base + VFE31_CAMIF_EFS_CFG);
		vfe->camif_pending = false;

		/*
		 * Always perform a global reset before disabling clocks.
		 * This ensures the VFE hardware is in a clean state even if
		 * halt timed out or the pipeline failed during startup.
		 * Without this reset, the vfe_clk may refuse to turn off
		 * because the hardware is still active.
		 */
		vfe_reset(vfe);

		camss_disable_clocks(vfe->nclocks, vfe->clock);
		pm_runtime_put_sync(vfe->camss->dev);
		vfe->res->hw_ops->pm_domain_off(vfe);
	}

	vfe->power_count--;

exit:
	mutex_unlock(&vfe->power_lock);
}

/*
 * vfe_flush_buffers - Return all vb2 buffers
 * @vid: Video device structure
 * @state: vb2 buffer state of the returned buffers
 *
 * Return all buffers to vb2. This includes queued pending buffers (still
 * unused) and any buffers given to the hardware but again still not used.
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_flush_buffers(struct camss_video *vid,
		      enum vb2_buffer_state state)
{
	struct vfe_line *line = container_of(vid, struct vfe_line, video_out);
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output;
	unsigned long flags;

	output = &line->output;

	spin_lock_irqsave(&vfe->output_lock, flags);

	vfe_buf_flush_pending(output, state);

	if (output->buf[0]) {
		vb2_buffer_done(&output->buf[0]->vb.vb2_buf, state);
		output->buf[0] = NULL;
	}

	if (output->buf[1]) {
		vb2_buffer_done(&output->buf[1]->vb.vb2_buf, state);
		output->buf[1] = NULL;
	}

	if (output->last_buffer) {
		vb2_buffer_done(&output->last_buffer->vb.vb2_buf, state);
		output->last_buffer = NULL;
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

/*
 * vfe_set_power - Power on/off VFE module
 * @sd: VFE V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_set_power(struct v4l2_subdev *sd, int on)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct vfe_device *vfe = to_vfe(line);
	int ret;

	if (on) {
		ret = vfe_get(vfe);
		if (ret < 0)
			return ret;
	} else {
		vfe_put(vfe);
	}

	return 0;
}

/*
 * vfe_set_stream - Enable/disable streaming on VFE module
 * @sd: VFE V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of VFE module is triggered here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct vfe_device *vfe = to_vfe(line);
	ktime_t start_time;
	int ret;

	start_time = ktime_get();
	dev_info(vfe->camss->dev,
		 "[TIMING] VFE set_stream: enable=%d line_id=%d START at %lld ns\n",
		 enable, line->id, ktime_to_ns(start_time));

	if (enable) {
		line->output.state = VFE_OUTPUT_RESERVED;
		ret = vfe->res->hw_ops->vfe_enable(line);
		if (ret < 0)
			dev_err(vfe->camss->dev,
				"Failed to enable vfe outputs\n");

		/*
		 * After VFE enable completes, dump critical status registers
		 * to verify VFE is ready to receive data from CSIPHY.
		 */
		dev_info(vfe->camss->dev,
			 "[TIMING] VFE READY CHECK after enable: IRQ_STATUS_0=0x%08x IRQ_STATUS_1=0x%08x\n",
			 readl_relaxed(vfe->base + 0x02C),  /* VFE_IRQ_STATUS_0 */
			 readl_relaxed(vfe->base + 0x030)); /* VFE_IRQ_STATUS_1 */
		dev_info(vfe->camss->dev,
			 "[TIMING] VFE READY CHECK: CAMIF_STATUS=0x%08x CAMIF_CFG=0x%08x\n",
			 readl_relaxed(vfe->base + 0x204),  /* VFE_CAMIF_STATUS */
			 readl_relaxed(vfe->base + 0x1E4)); /* VFE_CAMIF_CFG */
	} else {
		ret = vfe->res->hw_ops->vfe_disable(line);
		if (ret < 0)
			dev_err(vfe->camss->dev,
				"Failed to disable vfe outputs\n");
	}

	dev_info(vfe->camss->dev,
		 "[TIMING] VFE set_stream DONE ret=%d elapsed=%lld ns\n",
		 ret, ktime_to_ns(ktime_get()) - ktime_to_ns(start_time));
	return ret;
}

/*
 * __vfe_get_format - Get pointer to format structure
 * @line: VFE line
 * @sd_state: V4L2 subdev state
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__vfe_get_format(struct vfe_line *line,
		 struct v4l2_subdev_state *sd_state,
		 unsigned int pad,
		 enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_format(sd_state, pad);

	return &line->fmt[pad];
}

/*
 * __vfe_get_compose - Get pointer to compose selection structure
 * @line: VFE line
 * @sd_state: V4L2 subdev state
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE compose rectangle structure
 */
static struct v4l2_rect *
__vfe_get_compose(struct vfe_line *line,
		  struct v4l2_subdev_state *sd_state,
		  enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_compose(sd_state,
						     MSM_VFE_PAD_SINK);

	return &line->compose;
}

/*
 * __vfe_get_crop - Get pointer to crop selection structure
 * @line: VFE line
 * @sd_state: V4L2 subdev state
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE crop rectangle structure
 */
static struct v4l2_rect *
__vfe_get_crop(struct vfe_line *line,
	       struct v4l2_subdev_state *sd_state,
	       enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_crop(sd_state, MSM_VFE_PAD_SRC);

	return &line->crop;
}

/*
 * vfe_try_format - Handle try format by pad subdev method
 * @line: VFE line
 * @sd_state: V4L2 subdev state
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void vfe_try_format(struct vfe_line *line,
			   struct v4l2_subdev_state *sd_state,
			   unsigned int pad,
			   struct v4l2_mbus_framefmt *fmt,
			   enum v4l2_subdev_format_whence which)
{
	unsigned int i;
	u32 code;

	switch (pad) {
	case MSM_VFE_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < line->nformats; i++)
			if (fmt->code == line->formats[i].code)
				break;

		/* If not found, use UYVY as default */
		if (i >= line->nformats)
			fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		fmt->field = V4L2_FIELD_NONE;
		fmt->colorspace = V4L2_COLORSPACE_SRGB;

		break;

	case MSM_VFE_PAD_SRC:
		/* Set and return a format same as sink pad */
		code = fmt->code;

		*fmt = *__vfe_get_format(line, sd_state, MSM_VFE_PAD_SINK,
					 which);

		fmt->code = vfe_src_pad_code(line, fmt->code, 0, code);

		if (line->id == VFE_LINE_PIX) {
			struct v4l2_rect *rect;

			rect = __vfe_get_crop(line, sd_state, which);

			fmt->width = rect->width;
			fmt->height = rect->height;
		}

		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * vfe_try_compose - Handle try compose selection by pad subdev method
 * @line: VFE line
 * @sd_state: V4L2 subdev state
 * @rect: pointer to v4l2 rect structure
 * @which: wanted subdev format
 */
static void vfe_try_compose(struct vfe_line *line,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_rect *rect,
			    enum v4l2_subdev_format_whence which)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = __vfe_get_format(line, sd_state, MSM_VFE_PAD_SINK, which);

	if (rect->width > fmt->width)
		rect->width = fmt->width;

	if (rect->height > fmt->height)
		rect->height = fmt->height;

	if (fmt->width > rect->width * SCALER_RATIO_MAX)
		rect->width = (fmt->width + SCALER_RATIO_MAX - 1) /
							SCALER_RATIO_MAX;

	rect->width &= ~0x1;

	if (fmt->height > rect->height * SCALER_RATIO_MAX)
		rect->height = (fmt->height + SCALER_RATIO_MAX - 1) /
							SCALER_RATIO_MAX;

	if (rect->width < 16)
		rect->width = 16;

	if (rect->height < 4)
		rect->height = 4;
}

/*
 * vfe_try_crop - Handle try crop selection by pad subdev method
 * @line: VFE line
 * @sd_state: V4L2 subdev state
 * @rect: pointer to v4l2 rect structure
 * @which: wanted subdev format
 */
static void vfe_try_crop(struct vfe_line *line,
			 struct v4l2_subdev_state *sd_state,
			 struct v4l2_rect *rect,
			 enum v4l2_subdev_format_whence which)
{
	struct v4l2_rect *compose;

	compose = __vfe_get_compose(line, sd_state, which);

	if (rect->width > compose->width)
		rect->width = compose->width;

	if (rect->width + rect->left > compose->width)
		rect->left = compose->width - rect->width;

	if (rect->height > compose->height)
		rect->height = compose->height;

	if (rect->height + rect->top > compose->height)
		rect->top = compose->height - rect->height;

	/* wm in line based mode writes multiple of 16 horizontally */
	rect->left += (rect->width & 0xf) >> 1;
	rect->width &= ~0xf;

	if (rect->width < 16) {
		rect->left = 0;
		rect->width = 16;
	}

	if (rect->height < 4) {
		rect->top = 0;
		rect->height = 4;
	}
}

/*
 * vfe_enum_mbus_code - Handle pixel format enumeration
 * @sd: VFE V4L2 subdevice
 * @sd_state: V4L2 subdev state
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 *
 * return -EINVAL or zero on success
 */
static int vfe_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);

	if (code->pad == MSM_VFE_PAD_SINK) {
		if (code->index >= line->nformats)
			return -EINVAL;

		code->code = line->formats[code->index].code;
	} else {
		struct v4l2_mbus_framefmt *sink_fmt;

		sink_fmt = __vfe_get_format(line, sd_state, MSM_VFE_PAD_SINK,
					    code->which);

		code->code = vfe_src_pad_code(line, sink_fmt->code,
					      code->index, 0);
		if (!code->code)
			return -EINVAL;
	}

	return 0;
}

/*
 * vfe_enum_frame_size - Handle frame size enumeration
 * @sd: VFE V4L2 subdevice
 * @sd_state: V4L2 subdev state
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	vfe_try_format(line, sd_state, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	vfe_try_format(line, sd_state, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * vfe_get_format - Handle get format by pads subdev method
 * @sd: VFE V4L2 subdevice
 * @sd_state: V4L2 subdev state
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_get_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __vfe_get_format(line, sd_state, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int vfe_set_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_selection *sel);

/*
 * vfe_set_format - Handle set format by pads subdev method
 * @sd: VFE V4L2 subdevice
 * @sd_state: V4L2 subdev state
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_set_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __vfe_get_format(line, sd_state, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	vfe_try_format(line, sd_state, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	if (fmt->pad == MSM_VFE_PAD_SINK) {
		struct v4l2_subdev_selection sel = { 0 };
		int ret;

		if (line->id == VFE_LINE_PIX) {
			/*
			 * Reset compose/crop selection BEFORE propagating
			 * format to source pad. vfe_try_format for source pad
			 * uses the crop rectangle dimensions, so crop must be
			 * updated first to avoid using stale values.
			 */
			sel.which = fmt->which;
			sel.pad = MSM_VFE_PAD_SINK;
			sel.target = V4L2_SEL_TGT_COMPOSE;
			sel.r.width = fmt->format.width;
			sel.r.height = fmt->format.height;
			ret = vfe_set_selection(sd, sd_state, &sel);
			if (ret < 0)
				return ret;
		}

		/* Propagate the format from sink to source */
		format = __vfe_get_format(line, sd_state, MSM_VFE_PAD_SRC,
					  fmt->which);

		*format = fmt->format;
		vfe_try_format(line, sd_state, MSM_VFE_PAD_SRC, format,
			       fmt->which);
	}

	return 0;
}

/*
 * vfe_get_selection - Handle get selection by pads subdev method
 * @sd: VFE V4L2 subdevice
 * @sd_state: V4L2 subdev state
 * @sel: pointer to v4l2 subdev selection structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_selection *sel)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_format fmt = { 0 };
	struct v4l2_rect *rect;
	int ret;

	if (line->id != VFE_LINE_PIX)
		return -EINVAL;

	if (sel->pad == MSM_VFE_PAD_SINK)
		switch (sel->target) {
		case V4L2_SEL_TGT_COMPOSE_BOUNDS:
			fmt.pad = sel->pad;
			fmt.which = sel->which;
			ret = vfe_get_format(sd, sd_state, &fmt);
			if (ret < 0)
				return ret;

			sel->r.left = 0;
			sel->r.top = 0;
			sel->r.width = fmt.format.width;
			sel->r.height = fmt.format.height;
			break;
		case V4L2_SEL_TGT_COMPOSE:
			rect = __vfe_get_compose(line, sd_state, sel->which);
			if (rect == NULL)
				return -EINVAL;

			sel->r = *rect;
			break;
		default:
			return -EINVAL;
		}
	else if (sel->pad == MSM_VFE_PAD_SRC)
		switch (sel->target) {
		case V4L2_SEL_TGT_CROP_BOUNDS:
			rect = __vfe_get_compose(line, sd_state, sel->which);
			if (rect == NULL)
				return -EINVAL;

			sel->r.left = rect->left;
			sel->r.top = rect->top;
			sel->r.width = rect->width;
			sel->r.height = rect->height;
			break;
		case V4L2_SEL_TGT_CROP:
			rect = __vfe_get_crop(line, sd_state, sel->which);
			if (rect == NULL)
				return -EINVAL;

			sel->r = *rect;
			break;
		default:
			return -EINVAL;
		}

	return 0;
}

/*
 * vfe_set_selection - Handle set selection by pads subdev method
 * @sd: VFE V4L2 subdevice
 * @sd_state: V4L2 subdev state
 * @sel: pointer to v4l2 subdev selection structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_set_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_selection *sel)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_rect *rect;
	int ret;

	if (line->id != VFE_LINE_PIX)
		return -EINVAL;

	if (sel->target == V4L2_SEL_TGT_COMPOSE &&
		sel->pad == MSM_VFE_PAD_SINK) {
		struct v4l2_subdev_selection crop = { 0 };

		rect = __vfe_get_compose(line, sd_state, sel->which);
		if (rect == NULL)
			return -EINVAL;

		vfe_try_compose(line, sd_state, &sel->r, sel->which);
		*rect = sel->r;

		/* Reset source crop selection */
		crop.which = sel->which;
		crop.pad = MSM_VFE_PAD_SRC;
		crop.target = V4L2_SEL_TGT_CROP;
		crop.r = *rect;
		ret = vfe_set_selection(sd, sd_state, &crop);
	} else if (sel->target == V4L2_SEL_TGT_CROP &&
		sel->pad == MSM_VFE_PAD_SRC) {
		struct v4l2_subdev_format fmt = { 0 };

		rect = __vfe_get_crop(line, sd_state, sel->which);
		if (rect == NULL)
			return -EINVAL;

		vfe_try_crop(line, sd_state, &sel->r, sel->which);
		*rect = sel->r;

		/* Reset source pad format width and height */
		fmt.which = sel->which;
		fmt.pad = MSM_VFE_PAD_SRC;
		ret = vfe_get_format(sd, sd_state, &fmt);
		if (ret < 0)
			return ret;

		fmt.format.width = rect->width;
		fmt.format.height = rect->height;
		ret = vfe_set_format(sd, sd_state, &fmt);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/*
 * vfe_init_formats - Initialize formats on all pads
 * @sd: VFE V4L2 subdevice
 * @fh: V4L2 subdev file handle
 *
 * Initialize all pad formats with default values.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_init_formats(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct vfe_device *vfe = to_vfe(line);
	struct v4l2_subdev_format format = {
		.pad = MSM_VFE_PAD_SINK,
		.which = fh ? V4L2_SUBDEV_FORMAT_TRY :
			      V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.code = MEDIA_BUS_FMT_UYVY8_1X16,
			.width = 1920,
			.height = 1080
		}
	};

	/*
	 * VFE 3.1 uses parallel camera interface (CAMIF) which requires
	 * 2X8 media bus formats instead of 1X16. The resolution matches
	 * MT9M114 IFP output: 1288x968 (pixel array 1296x976 minus 8 border).
	 */
	if (vfe->res->hw_ops == &vfe_ops_3_1) {
		format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
		format.format.width = 1288;
		format.format.height = 968;
		dev_info(vfe->camss->dev,
			 "VFE 3.1 init format: %ux%u code=0x%04x\n",
			 format.format.width, format.format.height,
			 format.format.code);
	}

	return vfe_set_format(sd, fh ? fh->state : NULL, &format);
}

/*
 * msm_vfe_subdev_init - Initialize VFE device structure and resources
 * @vfe: VFE device
 * @res: VFE module resources table
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_vfe_subdev_init(struct camss *camss, struct vfe_device *vfe,
			const struct camss_subdev_resources *res, u8 id)
{
	struct device *dev = camss->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int i, j;
	int ret;

	if (!res->vfe.line_num)
		return -EINVAL;

	vfe->res = &res->vfe;
	vfe->res->hw_ops->subdev_init(dev, vfe);

	/* Power domain */

	if (res->vfe.pd_name) {
		vfe->genpd = dev_pm_domain_attach_by_name(camss->dev,
							  res->vfe.pd_name);
		if (IS_ERR(vfe->genpd)) {
			ret = PTR_ERR(vfe->genpd);
			return ret;
		}
	}

	if (!vfe->genpd && res->vfe.has_pd) {
		/*
		 * Legacy magic index.
		 * Requires
		 * power-domain = <VFE_X>,
		 *                <VFE_Y>,
		 *                <TITAN_TOP>
		 * id must correspondng to the index of the VFE which must
		 * come before the TOP GDSC. VFE Lite has no individually
		 * collapasible domain which is why id < vfe_num is a valid
		 * check.
		 */
		vfe->genpd = dev_pm_domain_attach_by_id(camss->dev, id);
		if (IS_ERR(vfe->genpd))
			return PTR_ERR(vfe->genpd);
	}

	/* Memory */

	vfe->base = devm_platform_ioremap_resource_byname(pdev, res->reg[0]);
	if (IS_ERR(vfe->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(vfe->base);
	}

	/* Interrupt */

	ret = platform_get_irq_byname(pdev, res->interrupt[0]);
	if (ret < 0)
		return ret;

	vfe->irq = ret;
	snprintf(vfe->irq_name, sizeof(vfe->irq_name), "%s_%s%d",
		 dev_name(dev), MSM_VFE_NAME, id);
	ret = devm_request_irq(dev, vfe->irq, vfe->res->hw_ops->isr,
			       IRQF_TRIGGER_RISING, vfe->irq_name, vfe);
	if (ret < 0) {
		dev_err(dev, "request_irq failed: %d\n", ret);
		return ret;
	}

	/* Clocks */

	vfe->nclocks = 0;
	while (res->clock[vfe->nclocks])
		vfe->nclocks++;

	vfe->clock = devm_kcalloc(dev, vfe->nclocks, sizeof(*vfe->clock),
				  GFP_KERNEL);
	if (!vfe->clock)
		return -ENOMEM;

	for (i = 0; i < vfe->nclocks; i++) {
		struct camss_clock *clock = &vfe->clock[i];

		clock->clk = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(clock->clk))
			return PTR_ERR(clock->clk);

		clock->name = res->clock[i];

		clock->nfreqs = 0;
		while (res->clock_rate[i][clock->nfreqs])
			clock->nfreqs++;

		if (!clock->nfreqs) {
			clock->freq = NULL;
			continue;
		}

		clock->freq = devm_kcalloc(dev,
					   clock->nfreqs,
					   sizeof(*clock->freq),
					   GFP_KERNEL);
		if (!clock->freq)
			return -ENOMEM;

		for (j = 0; j < clock->nfreqs; j++)
			clock->freq[j] = res->clock_rate[i][j];
	}

	mutex_init(&vfe->power_lock);
	vfe->power_count = 0;

	mutex_init(&vfe->stream_lock);
	vfe->stream_count = 0;

	spin_lock_init(&vfe->output_lock);

	vfe->camss = camss;
	vfe->id = id;
	vfe->reg_update = 0;

	for (i = VFE_LINE_RDI0; i < vfe->res->line_num; i++) {
		struct vfe_line *l = &vfe->line[i];

		l->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		l->video_out.camss = camss;
		l->id = i;
		init_completion(&l->output.sof);
		init_completion(&l->output.reg_update);

		if (i == VFE_LINE_PIX) {
			l->nformats = res->vfe.formats_pix->nformats;
			l->formats = res->vfe.formats_pix->formats;
		} else {
			l->nformats = res->vfe.formats_rdi->nformats;
			l->formats = res->vfe.formats_rdi->formats;
		}
	}

	init_completion(&vfe->reset_complete);
	init_completion(&vfe->halt_complete);

	return 0;
}

/*
 * msm_vfe_genpd_cleanup - Cleanup VFE genpd linkages
 * @vfe: VFE device
 */
void msm_vfe_genpd_cleanup(struct vfe_device *vfe)
{
	if (vfe->genpd_link)
		device_link_del(vfe->genpd_link);

	if (vfe->genpd)
		dev_pm_domain_detach(vfe->genpd, true);
}

/*
 * vfe_link_setup - Setup VFE connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Return 0 on success
 */
static int vfe_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	if (flags & MEDIA_LNK_FL_ENABLED)
		if (media_pad_remote_pad_first(local))
			return -EBUSY;

	return 0;
}

static const struct v4l2_subdev_core_ops vfe_core_ops = {
	.s_power = vfe_set_power,
};

static const struct v4l2_subdev_video_ops vfe_video_ops = {
	.s_stream = vfe_set_stream,
};

static const struct v4l2_subdev_pad_ops vfe_pad_ops = {
	.enum_mbus_code = vfe_enum_mbus_code,
	.enum_frame_size = vfe_enum_frame_size,
	.get_fmt = vfe_get_format,
	.set_fmt = vfe_set_format,
	.get_selection = vfe_get_selection,
	.set_selection = vfe_set_selection,
};

static const struct v4l2_subdev_ops vfe_v4l2_ops = {
	.core = &vfe_core_ops,
	.video = &vfe_video_ops,
	.pad = &vfe_pad_ops,
};

static const struct v4l2_subdev_internal_ops vfe_v4l2_internal_ops = {
	.open = vfe_init_formats,
};

static const struct media_entity_operations vfe_media_ops = {
	.link_setup = vfe_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static int vfe_bpl_align(struct vfe_device *vfe)
{
	int ret = 8;

	switch (vfe->camss->res->version) {
	case CAMSS_7280:
	case CAMSS_8250:
	case CAMSS_8280XP:
	case CAMSS_8300:
	case CAMSS_845:
	case CAMSS_8550:
	case CAMSS_8775P:
	case CAMSS_X1E80100:
		ret = 16;
		break;
	default:
		break;
	}

	return ret;
}

/*
 * msm_vfe_register_entities - Register subdev node for VFE module
 * @vfe: VFE device
 * @v4l2_dev: V4L2 device
 *
 * Initialize and register a subdev node for the VFE module. Then
 * call msm_video_register() to register the video device node which
 * will be connected to this subdev node. Then actually create the
 * media link between them.
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_vfe_register_entities(struct vfe_device *vfe,
			      struct v4l2_device *v4l2_dev)
{
	struct device *dev = vfe->camss->dev;
	struct v4l2_subdev *sd;
	struct media_pad *pads;
	struct camss_video *video_out;
	int ret;
	int i;

	for (i = 0; i < vfe->res->line_num; i++) {
		char name[32];

		sd = &vfe->line[i].subdev;
		pads = vfe->line[i].pads;
		video_out = &vfe->line[i].video_out;

		v4l2_subdev_init(sd, &vfe_v4l2_ops);
		sd->internal_ops = &vfe_v4l2_internal_ops;
		sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		if (i == VFE_LINE_PIX)
			snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d_%s",
				 MSM_VFE_NAME, vfe->id, "pix");
		else
			snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d_%s%d",
				 MSM_VFE_NAME, vfe->id, "rdi", i);

		v4l2_set_subdevdata(sd, &vfe->line[i]);

		ret = vfe_init_formats(sd, NULL);
		if (ret < 0) {
			dev_err(dev, "Failed to init format: %d\n", ret);
			goto error_init;
		}

		pads[MSM_VFE_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
		pads[MSM_VFE_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

		sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
		sd->entity.ops = &vfe_media_ops;
		ret = media_entity_pads_init(&sd->entity, MSM_VFE_PADS_NUM,
					     pads);
		if (ret < 0) {
			dev_err(dev, "Failed to init media entity: %d\n", ret);
			goto error_init;
		}

		ret = v4l2_device_register_subdev(v4l2_dev, sd);
		if (ret < 0) {
			dev_err(dev, "Failed to register subdev: %d\n", ret);
			goto error_reg_subdev;
		}

		video_out->ops = &vfe->video_ops;
		video_out->bpl_alignment = vfe_bpl_align(vfe);
		video_out->line_based = 0;
		if (i == VFE_LINE_PIX) {
			video_out->bpl_alignment = 16;
			video_out->line_based = 1;
		}

		video_out->nformats = vfe->line[i].nformats;
		video_out->formats = vfe->line[i].formats;

		/*
		 * Set the video device's default resolution to match the VFE
		 * subdev source pad format. This ensures the video device
		 * initializes with a format compatible with the upstream sensor.
		 */
		video_out->default_width = vfe->line[i].fmt[MSM_VFE_PAD_SRC].width;
		video_out->default_height = vfe->line[i].fmt[MSM_VFE_PAD_SRC].height;

		dev_info(dev, "VFE line %d: default video format %ux%u\n",
			 i, video_out->default_width, video_out->default_height);

		snprintf(name, ARRAY_SIZE(name), "%s%d_%s%d",
			 MSM_VFE_NAME, vfe->id, "video", i);
		ret = msm_video_register(video_out, v4l2_dev, name);
		if (ret < 0) {
			dev_err(dev, "Failed to register video node: %d\n",
				ret);
			goto error_reg_video;
		}

		ret = media_create_pad_link(
				&sd->entity, MSM_VFE_PAD_SRC,
				&video_out->vdev.entity, 0,
				MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
		if (ret < 0) {
			dev_err(dev, "Failed to link %s->%s entities: %d\n",
				sd->entity.name, video_out->vdev.entity.name,
				ret);
			goto error_link;
		}
		dev_info(dev, "Created link: %s pad %d -> %s pad 0 (IMMUTABLE|ENABLED)\n",
			 sd->entity.name, MSM_VFE_PAD_SRC, video_out->vdev.entity.name);
	}

	return 0;

error_link:
	msm_video_unregister(video_out);

error_reg_video:
	v4l2_device_unregister_subdev(sd);

error_reg_subdev:
	media_entity_cleanup(&sd->entity);

error_init:
	for (i--; i >= 0; i--) {
		sd = &vfe->line[i].subdev;
		video_out = &vfe->line[i].video_out;

		msm_video_unregister(video_out);
		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}

	return ret;
}

/*
 * msm_vfe_unregister_entities - Unregister VFE module subdev node
 * @vfe: VFE device
 */
void msm_vfe_unregister_entities(struct vfe_device *vfe)
{
	int i;

	mutex_destroy(&vfe->power_lock);
	mutex_destroy(&vfe->stream_lock);

	for (i = 0; i < vfe->res->line_num; i++) {
		struct v4l2_subdev *sd = &vfe->line[i].subdev;
		struct camss_video *video_out = &vfe->line[i].video_out;

		msm_video_unregister(video_out);
		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
}

bool vfe_is_lite(struct vfe_device *vfe)
{
	return vfe->camss->res->vfe_res[vfe->id].vfe.is_lite;
}
