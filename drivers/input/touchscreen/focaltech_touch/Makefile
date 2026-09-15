# Makefile for the focaltech touchscreen drivers.

ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/

obj-$(CONFIG_TOUCHSCREEN_FTS) += focaltech_touch.o
focaltech_touch-y := \
	    focaltech_core.o \
		focaltech_ex_fun.o \
		focaltech_ex_mode.o \
		focaltech_gesture.o \
		focaltech_esdcheck.o \
		focaltech_point_report_check.o \
		focaltech_flash.o \
		focaltech_i2c.o \
		focaltech_upgrade_ft5422.o \
		focaltech_upgrade_ft5422u.o \
		focaltech_upgrade_ft5446.o
