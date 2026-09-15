# Copyright (c) 2015-2018 TrustKernel Incorporated
# All Rights Reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

obj-$(CONFIG_TRUSTKERNEL_TEE_SUPPORT) += tkcore_drv.o

M ?= $(srctree)/drivers/tee/tkcore

ifneq ($(filter y m, $(CONFIG_TRUSTKERNEL_TEE_RPMB_SUPPORT)),)
ccflags-y += -DCONFIG_TRUSTKERNEL_TEE_RPMB_SUPPORT
endif

ccflags-y += \
	-I$(M)/include/arm_common \
	-I$(M)/include/linux \
	-I$(M)/include \
	-I$(M)/core

tkcore_drv-y := \
	tee_tz_drv.o \
	tee_smc_xfer.o \
	tee_mem.o \
	handle.o
