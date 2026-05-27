// SPDX-License-Identifier: GPL-2.0+
/*
 * novacom.c -- Novacom USB Gadget Driver
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Copyright (C) 2024 Modernized for Linux 6.x
 *
 * This is a legacy composite gadget driver that provides the novacom
 * USB function for webOS device communication with host computers.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

#include "u_novacom.h"

#define DRIVER_DESC		"Novacom Gadget"
#define DRIVER_VERSION		"2.0"

/*-------------------------------------------------------------------------*/

USB_GADGET_COMPOSITE_OPTIONS();

/* Palm/HP Vendor and Product IDs */
#define NOVACOM_VENDOR_ID	0x0830	/* HP */
#define NOVACOM_PRODUCT_ID	0x8002	/* TouchPad */

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof(device_desc),
	.bDescriptorType =	USB_DT_DEVICE,
	/* .bcdUSB = DYNAMIC */
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */
	.idVendor =		cpu_to_le16(NOVACOM_VENDOR_ID),
	.idProduct =		cpu_to_le16(NOVACOM_PRODUCT_ID),
	.bcdDevice =		cpu_to_le16(0x0100),
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	.bNumConfigurations =	1,
};

static const struct usb_descriptor_header *otg_desc[2];

/*-------------------------------------------------------------------------*/

/* String IDs are assigned dynamically */
#define STRING_DESCRIPTION_IDX	USB_GADGET_FIRST_AVAIL_IDX

static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "HP",
	[USB_GADGET_PRODUCT_IDX].s = "webOS Device",
	[USB_GADGET_SERIAL_IDX].s = "",
	[STRING_DESCRIPTION_IDX].s = "Novacom Configuration",
	{ } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

/*-------------------------------------------------------------------------*/

static struct usb_configuration novacom_config_driver = {
	.label			= "novacom",
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

static struct usb_function_instance *fi_novacom;
static struct usb_function *f_novacom;

static int novacom_do_config(struct usb_configuration *c)
{
	int ret;

	fi_novacom = usb_get_function_instance("novacom");
	if (IS_ERR(fi_novacom))
		return PTR_ERR(fi_novacom);

	f_novacom = usb_get_function(fi_novacom);
	if (IS_ERR(f_novacom)) {
		ret = PTR_ERR(f_novacom);
		goto err_get_func;
	}

	ret = usb_add_function(c, f_novacom);
	if (ret)
		goto err_add_func;

	return 0;

err_add_func:
	usb_put_function(f_novacom);
err_get_func:
	usb_put_function_instance(fi_novacom);
	return ret;
}

static int novacom_bind(struct usb_composite_dev *cdev)
{
	int status;

	/* Allocate string IDs */
	status = usb_string_ids_tab(cdev, strings_dev);
	if (status < 0)
		return status;

	device_desc.iManufacturer = strings_dev[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;
	novacom_config_driver.iConfiguration =
		strings_dev[STRING_DESCRIPTION_IDX].id;

	/* Handle OTG descriptor if needed */
	if (gadget_is_otg(cdev->gadget)) {
		struct usb_descriptor_header *usb_desc;

		usb_desc = usb_otg_descriptor_alloc(cdev->gadget);
		if (!usb_desc)
			return -ENOMEM;

		usb_otg_descriptor_init(cdev->gadget, usb_desc);
		otg_desc[0] = usb_desc;
		otg_desc[1] = NULL;

		novacom_config_driver.descriptors = otg_desc;
		novacom_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	/* Add configuration */
	status = usb_add_config(cdev, &novacom_config_driver, novacom_do_config);
	if (status < 0)
		goto fail;

	usb_composite_overwrite_options(cdev, &coverwrite);

	dev_info(&cdev->gadget->dev, "%s, version: %s\n",
		 DRIVER_DESC, DRIVER_VERSION);

	return 0;

fail:
	kfree(otg_desc[0]);
	otg_desc[0] = NULL;
	return status;
}

static int novacom_unbind(struct usb_composite_dev *cdev)
{
	if (f_novacom) {
		usb_put_function(f_novacom);
		f_novacom = NULL;
	}
	if (fi_novacom) {
		usb_put_function_instance(fi_novacom);
		fi_novacom = NULL;
	}

	kfree(otg_desc[0]);
	otg_desc[0] = NULL;

	return 0;
}

static struct usb_composite_driver novacom_driver = {
	.name		= "g_novacom",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_SUPER,
	.bind		= novacom_bind,
	.unbind		= novacom_unbind,
};

module_usb_composite_driver(novacom_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Palm, Inc.");
MODULE_LICENSE("GPL");
