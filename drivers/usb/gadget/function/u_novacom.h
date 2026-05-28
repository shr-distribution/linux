/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * u_novacom.h - interface to USB gadget novacom function
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Copyright (C) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 */

#ifndef __U_NOVACOM_H
#define __U_NOVACOM_H

#include <linux/usb/composite.h>

/**
 * struct f_novacom_opts - novacom function options for ConfigFS
 * @func_inst: USB function instance
 *
 * The novacom function takes no configurable parameters; the structure
 * exists solely so the function can be instantiated via ConfigFS.
 */
struct f_novacom_opts {
	struct usb_function_instance	func_inst;
};

#endif /* __U_NOVACOM_H */
