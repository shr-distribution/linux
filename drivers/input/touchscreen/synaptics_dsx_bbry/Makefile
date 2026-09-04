#
# Makefile for the Synaptics DSX touchscreen driver.
#

# Prefer the local synaptics_dsx.h (which carries the BBRY-specific
# board_data fields like resolution_x/y and num_of_{rx,tx}_electrodes)
# over include/linux/input/synaptics_dsx.h which is the upstream one.
ccflags-y += -I$(srctree)/$(src)

# 4.19 + newer compilers are stricter than 4.4 was. Relax a few Werrors the
# original BlackBerry code trips (VLAs, typedef redefinitions, unused
# vars, NULL/0 non-literal conversions, -Wfortify-source).
#
# Wrapped in cc-option: vla-extension, typedef-redefinition,
# non-literal-null-conversion and fortify-source are clang spellings. GCC does
# not know them and rejects the whole -Wno-error=<x> outright
# ("cc1: error: '-Wno-error=vla-extension': no option '-Wvla-extension'"),
# which broke the GCC build of every file in this directory.
ccflags-y += $(call cc-option,-Wno-error=vla-extension)
ccflags-y += $(call cc-option,-Wno-error=vla)
ccflags-y += $(call cc-option,-Wno-error=typedef-redefinition)
ccflags-y += $(call cc-option,-Wno-error=unused-variable)
ccflags-y += $(call cc-option,-Wno-error=non-literal-null-conversion)
ccflags-y += $(call cc-option,-Wno-error=fortify-source)

# Each configuration option enables a list of files.
obj-y += synaptics_dsx_i2c.o
obj-y += synaptics_dsx_core.o
obj-y += synaptics_dsx_slide.o
obj-y += synaptics_dsx_wakeup.o
#obj-y += synaptics_dsx_ddt.o
obj-y += synaptics_dsx_rmi_dev.o
obj-y += synaptics_dsx_fw_update.o
obj-y += synaptics_dsx_fw_update.o
obj-y += synaptics_dsx_test_reporting.o
#obj-y += synaptics_dsx_proximity.o
#obj-y += synaptics_dsx_active_pen.o


obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C) += synaptics_dsx_i2c.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SPI) += synaptics_dsx_spi.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE) += synaptics_dsx_core.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE) += synaptics_dsx_slide.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE) += synaptics_dsx_wakeup.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT) += synaptics_dsx_ddt.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_DEV) += synaptics_dsx_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) += synaptics_dsx_fw_update.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_BBRY_DSX_FW_UPDATE) += synaptics_dsx_fw_update.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING) += synaptics_dsx_test_reporting.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_PROXIMITY) += synaptics_dsx_proximity.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ACTIVE_PEN) += synaptics_dsx_active_pen.o
