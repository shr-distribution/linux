/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * linux/include/linux/a6.h
 *
 * Driver for the A6 Battery Controller (HP TouchPad)
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 * Author: Raj Mojumder <raj.mojumder@palm.com>
 *
 * Modernized for device tree and modern kernel APIs
 */

#ifndef _A6_H
#define _A6_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Device names */
#define  A6_DEVICE_0   "a6_0"
#define  A6_DEVICE_1   "a6_1"
#define  A6_DRIVER     "a6"

#define  A6_DEVICE     A6_DEVICE_0

/* IOCTLs for firmware update */
#define A6_IOCTL_SET_FW_DATA		_IOW('c', 0x01, int)
#define A6_IOCTL_VERIFY_FW_DATA		_IOW('c', 0x02, int)

/*
 * Legacy platform data structure - DEPRECATED
 * Modern drivers should use device tree with GPIO descriptors
 * This is kept only for reference and backward compatibility
 */
struct a6_platform_data {
	char	*dev_name;   /* device name */
	int	pwr_gpio;    /* deprecated: use DT interrupts property */
	int	sbw_tck_gpio;  /* deprecated: use DT tck-gpios property */
	int	sbw_tdio_gpio; /* deprecated: use DT tdio-gpios property */
	int	sbw_wkup_gpio; /* deprecated: use DT wakeup-gpios property */
	void	*sbw_ops;      /* deprecated: handled internally */
	void	*wake_ops;     /* deprecated: handled internally */

	void	*sbw_init_gpio_config;      /* deprecated */
	int	sbw_init_gpio_config_size;  /* deprecated */
	void	*sbw_deinit_gpio_config;    /* deprecated */
	int	sbw_deinit_gpio_config_size; /* deprecated */

	int	(*sbw_init)(struct a6_platform_data *);   /* deprecated */
	int	(*sbw_deinit)(struct a6_platform_data *); /* deprecated */

	int	pwr_gpio_wakeup_cap;  /* deprecated */
};

/*
 * Wake operation callbacks - used internally by driver
 * Applications should use sysfs power management interfaces
 */
struct a6_wake_ops {
	void	*data;

	/* external periodic sleep/wake interface */
	int	(*enable_periodic_wake)(void *);
	int	(*disable_periodic_wake)(void *);

	/* internal sleep/wake interface */
	int	(*internal_wake_enable_state)(void *);
	int	(*internal_wake_period)(void *);

	/* force sleep/wake interface */
	int	(*force_wake)(void *);
	int	(*force_sleep)(void *);
};

#endif /* _A6_H */
