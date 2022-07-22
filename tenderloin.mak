#CROSS_COMPILE	?= arm-webos-linux-gnueabi-
#PATCHDIR	:= ${HOME}/linux-stericsson
build_dir       := $(CURDIR)/build-apq8060
output_dir	:= $(HOME)
rootfs		:= $(HOME)/rootfs-msm8660.cpio
rootfsbase	:= $(shell basename $(rootfs))
install_dir     := $(build_dir)/install
config_file     := $(build_dir)/.config
#dtb		:= $(build_dir)/arch/arm/boot/dts/qcom-msm8660-surf.dtb
dtb		:= $(build_dir)/arch/arm/boot/dts/qcom-apq8060-tenderloin.dtb
makejobs	:= $(shell grep '^processor' /proc/cpuinfo | sort -u | wc -l)
makethreads	:= $(shell dc -e "$(makejobs) 1 + p")


make_options := -f Makefile \
		-j$(makethreads) -l$(makejobs) \
		ARCH=arm \
		CROSS_COMPILE=$(CROSS_COMPILE) \
		KBUILD_OUTPUT=$(build_dir)

.PHONY: help
help:
	@echo "****  Common Makefile  ****"
	@echo "make config [PLATFORM=foo] - configure for platform \"foo\""
	@echo "make build - build the kernel and produce a RAMdisk image"
	@echo
	@echo "example:"
	@echo "make -f apq8060.mak config"
	@echo "make -f apq8060.mak build"

.PHONY: have-rootfs
have-rootfs:
	@if [ ! -f $(rootfs) ] ; then \
	     echo "ERROR: no rootfs at $(rootfs)" ; \
	     echo "This is needed to boot the system." ; \
	     echo "ABORTING." ; \
	     exit 1 ; \
	else \
	     echo "Rootfs available at $(rootfs)" ; \
	fi
	@if [ ! -r $(rootfs) ] ; then \
	     echo "ERROR: rootfs at $(rootfs) not readable" ; \
	     echo "ABORTING." ; \
	     exit 1 ; \
	fi

.PHONY: have-crosscompiler
have-crosscompiler:
	@echo -n "Check that $(CROSS_COMPILE)gcc is available..."
	@which $(CROSS_COMPILE)gcc > /dev/null ; \
	if [ ! $$? -eq 0 ] ; then \
	   echo "ERROR: cross-compiler $(CROSS_COMPILE)gcc not in PATH=$$PATH!" ; \
	   echo "ABORTING." ; \
	   exit 1 ; \
	else \
	   echo "OK" ;\
	fi

config-base: FORCE
	@mkdir -p $(build_dir)

	if [ -f arch/arm/configs/qcom_defconfig ] ; then \
	   $(MAKE) $(make_options) qcom_defconfig ; \
	else \
	   $(MAKE) $(make_options) msm_defconfig ; \
	fi

config-initramfs: config-base
	# Configure in the initramfs
	$(CURDIR)/scripts/config --file $(config_file) \
	--enable BLK_DEV_INITRD \
	--set-str INITRAMFS_SOURCE $(rootfsbase) \
	--enable RD_GZIP \
	--enable INITRAMFS_COMPRESSION_GZIP

# For early printk
config-earlydebug: config-base
	$(CURDIR)/scripts/config --file $(config_file) \
	--enable DEBUG_LL \
	--enable EARLY_PRINTK \
	--set-val DEBUG_UART_PHYS 0x19c40000 \
	--set-val DEBUG_UART_VIRT 0xf0040000 \
	--enable DEBUG_UNCOMPRESS \
	--enable DEBUG_EARLY_MAPPINGS \
	--set-str CMDLINE "console=ttyMSM0,115200n8 earlyprintk"

config-kasan: config-base
	$(CURDIR)/scripts/config --file $(config_file) \
	--enable SLUB \
	--enable SLUB_DEBUG \
	--enable SLUB_DEBUG_ON \
	--enable KASAN \
	--enable KASAN_OUTLINE \
	--enable STACKTRACE \
	--enable TEST_KASAN

config-mainlined-features: config-base
	# Then reconfigure various stuff
	$(CURDIR)/scripts/config --file $(config_file) \
	--enable ARCH_MSM8X60 \
	--enable MACH_MSM8X60_SURF \
	--enable PINCTRL_MSM8660 \
	--disable GPIO_MSM_V1 \
	--enable GPIO_MSM_V2 \
	--enable MMC \
	--enable MMC_ARMMMCI \
	--enable NEW_LEDS \
	--enable LEDS_CLASS \
	--enable LEDS_GPIO \
	--enable LEDS_TRIGGERS \
	--enable LEDS_TRIGGER_HEARTBEAT \
	--enable LEDS_PM8058 \
	--enable QCOM_EBI2 \
	--enable ETHERNET \
	--enable NET_VENDOR_SMSC \
	--enable SMSC911X \
	--enable IIO \
	--enable IIO_BUFFER \
	--enable IIO_BUFFER_CB \
	--enable IIO_KFIFO_BUF \
	--enable IIO_TRIGGERED_BUFFER \
	--enable IIO_CONFIGFS \
	--enable IIO_TRIGGER \
	--enable IIO_SW_TRIGGER \
	--enable IIO_HRTIMER_TRIGGER \
	--enable AK8975 \
	--disable INPUT_MPU3050 \
	--enable MPU3050 \
	--enable MPU3050_I2C \
	--enable KXSD9 \
	--enable KXSD9_I2C \
	--enable KXSD9_SPI \
	--enable BMP280 \
	--enable CM3605 \
	--enable QCOM_PM8XXX_XOADC \
	--enable PM \
	--enable HWMON \
	--enable SENSORS_IIO_HWMON \
	--enable EEPROM_AT24 \
	--enable SND_SOC \
	--enable SND_SOC_QCOM \
	--enable SND_SOC_WM8903 \
	--enable SND_SOC_MSM8660_WM8903 \
	--enable MSM_LCC_8660 \
	--enable MSM_MMCC_8660 \
	--enable DRM_MSM \
	--enable DRM_MSM_DSI \
	--enable DRM_MSM_DSI_PLL \
	--enable CMA \
	--enable DMA_CMA \
	--enable DRM_PANEL \
	--enable DRM_PANEL_AUO_H361VL01

config-bluetooth: config-base config-mainlined-features
	# Then reconfigure various stuff
	$(CURDIR)/scripts/config --file $(config_file) \
	--enable MFD_QCOM_MARIMBA \
	--enable SERIAL_DEV_BUS \
	--enable SERIAL_DEV_CTRL_TTYPORT \
	--enable BT \
	--enable BT_HCIUART \
	--enable BT_HCIUART_SERDEV \
	--enable BT_HCIUART_WCN2243

config: config-base config-mainlined-features config-bluetooth config-initramfs config-kasan config-earlydebug FORCE
	# Reconfigure a bit
	$(CURDIR)/scripts/config --file $(build_dir)/.config \
	--enable ARCH_MSM8X60
	yes "" | make $(make_options) oldconfig

menuconfig: FORCE
	if [ ! -d $(build_dir) ] ; then \
	   echo "no build dir" ; \
	   exit 1 ; \
        fi
	$(MAKE) $(make_options) menuconfig

saveconfig: config-base config-mainlined-features FORCE
	yes "" | make $(make_options) oldconfig
	$(MAKE) $(make_options) savedefconfig
	cp $(build_dir)/defconfig arch/arm/configs/qcom_defconfig

build-zimage: have-crosscompiler
	$(MAKE) $(make_options) zImage CONFIG_DEBUG_SECTION_MISMATCH=y

build-dtbs: FORCE
	$(MAKE) $(make_options) dtbs

build: have-rootfs build-zimage build-dtbs FORCE
	@echo "Copy zImage to $(output_dir)/zImage"
	@cp -f $(build_dir)/arch/arm/boot/zImage $(output_dir)/zImage
	@if [ ! -r $(dtb) ] ; then \
	   echo "NO DTB in $(dtb)!" ; \
	   exit 1 ; \
	fi
	@echo "Catenate DTB onto zImage $(output_dir)/zImage..."
	cat $(dtb) >> $(output_dir)/zImage

clean:
	$(MAKE) -f Makefile clean
	rm -rf $(build_dir)

# Rules without commands or prerequisites that do not match a file name
# are considered to always change when make runs. This means that any rule
# that depends on FORCE will always be remade also.
FORCE:
