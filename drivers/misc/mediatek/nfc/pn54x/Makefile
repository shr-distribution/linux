#
# Makefile for nfc devices
#

ccflags-$(CONFIG_NFC_CHIP_NXP54X) += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
obj-$(CONFIG_NFC_CHIP_NXP54X)		+= pn553.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG

