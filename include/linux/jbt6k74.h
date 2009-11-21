#ifndef __JBT6K74_H__
#define __JBT6K74_H__

#include <linux/spi/spi.h>

enum jbt_resolution {
	JBT_RESOLUTION_VGA,
	JBT_RESOLUTION_QVGA,
};

enum jbt_power_mode {
	JBT_POWER_MODE_OFF,
	JBT_POWER_MODE_STANDBY,
	JBT_POWER_MODE_NORMAL,
};

extern void jbt6k74_setpower(enum jbt_power_mode new_power);
extern int jbt6k74_prepare_resolutionchange(enum jbt_resolution new_resolution);
extern int jbt6k74_finish_resolutionchange(enum jbt_resolution new_resolution);


/*
 *  struct jbt6k74_platform_data - Platform data for jbt6k74 driver
 *  @probe_completed: Callback to be called when the driver has been
 *  successfully probed.
 *  @enable_pixel_clock: Callback to enable or disable the pixelclock of the
 *  gpu.
 *  @gpio_reset: Reset gpio pin number.
 */
struct jbt6k74_platform_data {
	void (*probe_completed)(struct device *dev);

	int gpio_reset;
};

#endif
