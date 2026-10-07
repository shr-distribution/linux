#
# Copyright (C) 2017 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#
ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/inc \

obj-y		+= regulator/
obj-y		+= gpio/
obj-y		+= mclk/
obj-y		+= imgsensor_cfg_table.o

# Titan Pocket (g66): rear camera AVDD is a GPIO-switched LDO, DVDD the PMIC
ifneq (,$(filter "g66v71c2k_dfl_eea" "g66v71c2k_dfl_tee",$(CONFIG_ARCH_MTK_PROJECT)))
ccflags-y += -DTITAN_POCKET_MAIN_AVDD_GPIO=1
endif
#obj-y           += imgsensor_oc.o
