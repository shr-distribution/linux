# Sunwave fingerprint driver v2.2.38-2018-12-25, as published in
# Teracube-Inc/kernel_teracube_2e-4.9 (drivers/input/fingerprint/sunwave_fp_tee).
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)

# TrustKernel TEE
ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/core
ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/include

obj-$(CONFIG_SUNWAVE_FP) += sf_ctl.o sf_hw.o
