# Makefile for the focaltech touchscreen drivers.


ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/
obj-m+= ft3519.o
ft3519-objs := focaltech_core.o \
		focaltech_ex_fun.o \
		focaltech_ex_mode.o \
		focaltech_gesture.o \
		focaltech_esdcheck.o \
		focaltech_point_report_check.o \
		focaltech_proximity.o



ft3519-objs += focaltech_i2c.o






ft3519-objs += focaltech_flash.o
ft3519-objs += focaltech_flash/focaltech_upgrade_ft3519.o