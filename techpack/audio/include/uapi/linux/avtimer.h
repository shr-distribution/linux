#ifndef _UAPI_AVTIMER_H
#define _UAPI_AVTIMER_H

#include <linux/ioctl.h>

#define MAJOR_NUM 100

#define IOCTL_GET_AVTIMER_TICK _IOR(MAJOR_NUM, 0, uint64_t)

struct avtimer_fptr_t {
	int (*fptr_avtimer_open)(void);
	int (*fptr_avtimer_enable)(int enable);
	int (*fptr_avtimer_get_time)(uint64_t *avtimer_tick);
};

void msm_isp_set_avtimer_fptr(struct avtimer_fptr_t avtimer);

#endif
