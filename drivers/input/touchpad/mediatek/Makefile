#
# Makefile for the MediaTek touchpad driver.
#

obj-$(CONFIG_MTK_TOUCHPAD)		+= pad.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/

obj-$(CONFIG_MTK_TOUCHPAD_DEBUG)	+= pad_debug.o

# Device-specific drivers
obj-$(CONFIG_MTK_TOUCHPAD_SYNAPTICS)	+=  synaptics_dsx_pad/
