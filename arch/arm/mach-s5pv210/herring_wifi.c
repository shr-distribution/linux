/*
 * platform driver for herring_wifi device
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (C) 2008 Google Inc
 */

#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>

static struct wifi_platform_data *wifi_control_data = NULL;
static struct resource *wifi_irqres = NULL;

int wifi_set_carddetect(int on)
{
        printk("%s = %d\n", __FUNCTION__, on);
        if (wifi_control_data && wifi_control_data->set_carddetect) {
                wifi_control_data->set_carddetect(on);
        }
        return 0;
}

int wifi_set_power(int on, unsigned long msec)
{
        printk("%s = %d\n", __FUNCTION__, on);
        if (wifi_control_data && wifi_control_data->set_power) {
                wifi_control_data->set_power(on);
        }
        if (msec)
                mdelay(msec);
        return 0;
}

int wifi_set_reset(int on, unsigned long msec)
{
        if (wifi_control_data && wifi_control_data->set_reset) {
                wifi_control_data->set_reset(on);
        }
        if (msec)
                mdelay(msec);
        return 0;
}



static int wifi_probe(struct platform_device *pdev)
{
        struct wifi_platform_data *wifi_ctrl =
                (struct wifi_platform_data *)(pdev->dev.platform_data);

        wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "bcm4329_wlan_irq");
        wifi_control_data = wifi_ctrl;

        wifi_set_power(1, 0);   /* Power On */
        wifi_set_carddetect(1); /* CardDetect (0->1) */

        return 0;
}

static int wifi_remove(struct platform_device *pdev)
{
        struct wifi_platform_data *wifi_ctrl =
                (struct wifi_platform_data *)(pdev->dev.platform_data);

        wifi_control_data = wifi_ctrl;

        wifi_set_power(0, 0);   /* Power Off */
        wifi_set_carddetect(0); /* CardDetect (1->0) */

        return 0;
}

static struct platform_driver wifi_device = {
	.probe		= wifi_probe,
	.remove		= wifi_remove,
	.driver		= {
		.name   = "bcm4329_wlan",
	},
};

static int __init brcm_wifi_sdio_init(void)
{
	return platform_driver_register(&wifi_device);
}

static void __exit brcm_wifi_sdio_exit(void)
{
	platform_driver_unregister(&wifi_device);
}

module_init(brcm_wifi_sdio_init);
module_exit(brcm_wifi_sdio_exit);
MODULE_LICENSE("GPL");
