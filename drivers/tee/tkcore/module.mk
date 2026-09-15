LOCAL_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

#KDIR := /path/to/aos-kernel-sdk/arm64/ti/general/aos-kernel/sysroot/usr/lib/modules/5.10.0-1.h1.AOS3.2.aarch64/build

ifeq ($(KDIR),)
$(error KDIR must be provided)
endif

CROSS_COMPILE ?= aarch64-linux-gnu-

export CONFIG_TRUSTKERNEL_TEE_SUPPORT := m
export CONFIG_TRUSTKERNEL_TEE_RPMB_SUPPORT := y

include $(LOCAL_PATH)/Makefile

.PHONY: all tkcore clean

all: tkcore

clean:
	make ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(LOCAL_PATH) clean

tkcore:
	make ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(LOCAL_PATH) modules
