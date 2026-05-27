/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * u_novacom.h - interface to USB gadget novacom function
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Copyright (C) 2024 Modernized for Linux 6.x
 */

#ifndef __U_NOVACOM_H
#define __U_NOVACOM_H

#include <linux/usb/composite.h>

/**
 * struct f_novacom_opts - novacom function options for ConfigFS
 * @func_inst: USB function instance
 * @lock: protects configuration data
 * @refcnt: reference counter
 */
struct f_novacom_opts {
	struct usb_function_instance	func_inst;
	struct mutex			lock;
	int				refcnt;
};

#endif /* __U_NOVACOM_H */
