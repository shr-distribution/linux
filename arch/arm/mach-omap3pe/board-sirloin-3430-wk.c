/*
 * linux/arch/arm/mach-omap3pe/board-sirloin-3430-wk.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/arch/mux.h>
#include <asm/arch/control.h>

#include <asm/arch/pm.h>

#include "board-sirloin-3430-wk.h"

/*************************************
 * Wakeup sources.
 ************************************/

struct wakeup_source {
	const char *name;
	const char *pin;
};

static struct wakeup_source omap3_wakeup_source[] = {
 { "RTC_WAKE",          "AF26_SYS_NREQ" },
 { "MODEM_WAKE_UART",   "AE1_3430_GPIO152" },
 { "MODEM_WAKE_USB",    "AC1_3430_GPIO155" },
 { "CORE_NAVI_WAKE",    "AE13_3430_GPIO17_CORE_NAVI" },
 { "BT_WAKE",           "AB1_BT_HOST_WAKE" },
 { "WIFI_WAKE",         "AF3_WL_IRQ" },
};

static u32 omap3_wakeup_source_info;

/** 
 * @brief This must be called from prcm_pwr.c before the wakeup
 *        source registers get clobbered.
 */
void omap3_wakeup_sources_save(void)
{
	u16 mask = 0x8000;
	u16 addr, val;
	int i;

	omap3_wakeup_source_info = 0;

	for (i = 0; i < ARRAY_SIZE(omap3_wakeup_source); i++) {
		addr = omap_get_mux_reg(omap3_wakeup_source[i].pin);

		if (addr) {
			val = omap_ctrl_readw(addr);

			if (val & mask) {
				omap3_wakeup_source_info |= (1 << i);
			}
		}
	}
}

static ssize_t omap3_wakeup_sources_show(struct kset *subsys, char *buf)
{
	int len = 0;
	int i;
	for (i = 0; i < ARRAY_SIZE(omap3_wakeup_source); i++) {
		
		struct wakeup_source *source = &omap3_wakeup_source[i];
		if (omap3_wakeup_source_info & (1 << i)) {
			len += sprintf(buf + len, "%s ", source->name);
		}
	}

	len += sprintf(buf + len, "\n");
	return len;
}

int omap3_wakeup_sources_get(void)
{
	return omap3_wakeup_source_info;
}
EXPORT_SYMBOL(omap3_wakeup_sources_get);

void omap3_wakeup_sources_clear(void)
{
	omap3_wakeup_source_info = 0;
}

/**
 * Init
 */

static struct subsys_attribute wakeup_sources = {
	.attr = {
		.name = __stringify(wakeup_sources),
		.mode = 0644,
	},
	.show  = omap3_wakeup_sources_show,
	.store = NULL,
};

static int __init wakeup_source_info_init(void)
{
	int retval;

	omap3_wakeup_source_info = 0;

	retval = subsys_create_file(&power_subsys, &wakeup_sources);
	if (retval) {
		printk(KERN_ERR
			"ERROR creating sysfs entry for 'wakeup_source': %d\n", retval);
	}

	return retval;
}

module_init(wakeup_source_info_init);
