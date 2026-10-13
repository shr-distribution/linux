/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_POWER_OZ8806_BATTERY_H
#define __LINUX_POWER_OZ8806_BATTERY_H

#include <linux/errno.h>
#include <linux/kconfig.h>

/* One reading of the OZ8806 fuel gauge, for the supply that reports it. */
struct oz8806_reading {
	int capacity;		/* percent, 0..100 */
	int charge_now_mah;
	int charge_full_mah;
};

#if IS_ENABLED(CONFIG_BATTERY_OZ8806_MTK_FEED)
/* 0 with *r filled in, or a negative errno while there is no valid reading. */
int oz8806_battery_read(struct oz8806_reading *r);
#else
static inline int oz8806_battery_read(struct oz8806_reading *r)
{
	return -ENODEV;
}
#endif

#endif
