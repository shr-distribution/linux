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
#include <asm/arch/board.h>
#include <asm/arch/gpio.h>

static int __init sirloin_wifi_sdio_init(void)
{
        board_wl_vcc_enable(1);
        board_wl_rf_vcc_enable(1);
	board_rescan_wifi_slot();
	return 0;

}
static void __exit sirloin_wifi_sdio_exit(void)
{
        board_wl_vcc_enable(0);
        board_wl_rf_vcc_enable(0);
	board_rescan_wifi_slot();

}

module_init(sirloin_wifi_sdio_init);
module_exit(sirloin_wifi_sdio_exit);
MODULE_LICENSE("GPL");
