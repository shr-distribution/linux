/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm MSM8660 / APQ8060 MPM wake-source consumer interface.
 *
 * Consumer drivers (e.g. mmci for SDC4 WiFi wake) obtain a handle via
 * msm8660_mpm_get() and use the helpers below to register wake sources
 * with the MPM hardware. The MPM driver itself lives in
 * drivers/soc/qcom/msm8660-mpm.c.
 */

#ifndef __SOC_QCOM_MSM8660_MPM_H__
#define __SOC_QCOM_MSM8660_MPM_H__

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/irq.h>

#define MSM8660_MPM_NR_PINS	64

/* Well-known wake-source pin indices (from the legacy 2.6.35-palm tree). */
#define MSM8660_MPM_PIN_SDC3_DAT1	21
#define MSM8660_MPM_PIN_SDC3_DAT3	22
#define MSM8660_MPM_PIN_SDC4_DAT1	23
#define MSM8660_MPM_PIN_SDC4_DAT3	24

struct device_node;
struct msm8660_mpm;

#if IS_ENABLED(CONFIG_QCOM_MSM8660_MPM)

struct msm8660_mpm *msm8660_mpm_get(struct device_node *np,
				    const char *propname);

int msm8660_mpm_set_pin_wake(struct msm8660_mpm *mpm, unsigned int pin,
			     bool on);
int msm8660_mpm_enable_pin(struct msm8660_mpm *mpm, unsigned int pin,
			   bool enable);
int msm8660_mpm_set_pin_type(struct msm8660_mpm *mpm, unsigned int pin,
			     unsigned int flow_type);

void msm8660_mpm_enter_sleep(struct msm8660_mpm *mpm, bool from_idle);
void msm8660_mpm_exit_sleep(struct msm8660_mpm *mpm, bool from_idle);

#else /* !CONFIG_QCOM_MSM8660_MPM */

static inline struct msm8660_mpm *
msm8660_mpm_get(struct device_node *np, const char *propname)
{
	return ERR_PTR(-ENODEV);
}

static inline int msm8660_mpm_set_pin_wake(struct msm8660_mpm *mpm,
					   unsigned int pin, bool on)
{
	return -ENODEV;
}

static inline int msm8660_mpm_enable_pin(struct msm8660_mpm *mpm,
					 unsigned int pin, bool enable)
{
	return -ENODEV;
}

static inline int msm8660_mpm_set_pin_type(struct msm8660_mpm *mpm,
					   unsigned int pin,
					   unsigned int flow_type)
{
	return -ENODEV;
}

static inline void msm8660_mpm_enter_sleep(struct msm8660_mpm *mpm,
					   bool from_idle) { }
static inline void msm8660_mpm_exit_sleep(struct msm8660_mpm *mpm,
					  bool from_idle) { }

#endif /* CONFIG_QCOM_MSM8660_MPM */

#endif /* __SOC_QCOM_MSM8660_MPM_H__ */
