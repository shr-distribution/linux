/*
 * linux/arch/arm/mach-omap3pe/board-sirloin-3430-wifi.c
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

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include "prcm-regs.h"

#define WIFI_RESET_SET		0x7f

unsigned int wifi_irq_count = 0;

static int  sirloin_wifi_reset_and_rescan(void)
{
	/* TODO maybe we should use a timer to restart the reset procedure if the device is
	 * not available after reset. */

	omap_set_gpio_dataout(WIFI_RESET_SET, 1);
	udelay(5);
	omap_set_gpio_dataout(WIFI_RESET_SET, 0);
	udelay(2);
	omap_set_gpio_dataout(WIFI_RESET_SET, 1);

	return board_rescan_wifi_slot();
}

static irqreturn_t wifi_irq_handler(int irq, void *data)
{
	/* Simply count how often the irq occured */
	wifi_irq_count++;

	printk(KERN_INFO "WIFI_GPIO_INT triggered: count = %i\n", wifi_irq_count);

	return IRQ_HANDLED;
}

static int __init sirloin_wifi_sdio_init(void)
{
	printk(KERN_INFO "%s: Power enable = 1 for WiFi chip.\n", __FUNCTION__);
	board_wl_vcc_enable(1);
	board_wl_rf_vcc_enable(1);

	printk(KERN_INFO "%s: Taing WiFi Chip out of reset, rescan sdio slot and loading\n", __FUNCTION__);

	/* Request GPIO and register irq handler */

	/* TODO Why we need to request the 0x8a GPIO and use later only the 0x8b one? */
	omap_request_gpio(0x8a);
	omap_set_gpio_direction(0x8a, 1);

	if(request_irq(0x12a, wifi_irq_handler, IRQF_TRIGGER_FALLING, "WIFI_GPIO_INT", NULL) < 0)
	{
		printk(KERN_ERR "%s: Could not register WiFi IRQ handler function\n", __FUNCTION__);
		goto err;
	}

	if (sirloin_wifi_reset_and_rescan() < 0)
	{
		printk(KERN_ERR "%s: Failed to find wifi chip on mmc slot.\n", __FUNCTION__);
		goto err;
	}
	
	return 0;
err:
	return -1;
}
static void __exit sirloin_wifi_sdio_exit(void)
{
	printk(KERN_INFO "%s: Unloading WiFi Driver and Keeping WiFi Chip in reset\n", __FUNCTION__);

	/* Put WiFi chip in reset mode and rescan mmc slots */
	omap_set_gpio_dataout(WIFI_RESET_SET, 0);
	udelay(5);
	board_rescan_wifi_slot();
	
	/* Free IRQ and GPIO */
	free_irq(0x12a, NULL);
	omap_free_gpio(0x8a);

	printk(KERN_INFO "%s: Power enable = 0 for WiFi chip.\n", __FUNCTION__);
	board_wl_vcc_enable(0);
	board_wl_rf_vcc_enable(0);
}

module_init(sirloin_wifi_sdio_init);
module_exit(sirloin_wifi_sdio_exit);
MODULE_LICENSE("GPL");
