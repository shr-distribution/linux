#ifndef __LINUX_HDQ_H__
#define __LINUX_HDQ_H__

#include <linux/device.h>

#define HDQ_SAMPLE_PERIOD_US	10

/* platform data */

struct hdq_platform_data {
	void (*gpio_dir_out)(void);
	void (*gpio_dir_in)(void);
	void (*gpio_set)(int);
	int (*gpio_get)(void);

	int (*enable_fiq)(void);
	void (*disable_fiq)(void);
	void (*kick_fiq)(void);
};

int hdq_read(struct device *dev, unsigned int address);

#endif
