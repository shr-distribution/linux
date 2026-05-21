KDIR ?= /home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad
CROSS ?= /opt/PalmPDK/arm-gcc/bin/arm-none-linux-gnueabi-

obj-m := legacy_adm_probe.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS) clean
