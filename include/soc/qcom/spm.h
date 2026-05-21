/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 */

#ifndef __SPM_H__
#define __SPM_H__

enum pm_sleep_mode {
	PM_SLEEP_MODE_STBY,
	PM_SLEEP_MODE_RET,
	PM_SLEEP_MODE_SPC,
	PM_SLEEP_MODE_PC,
	PM_SLEEP_MODE_NR,
};

struct spm_driver_data;

struct spm_driver_data *spm_get_drv_by_cpu(unsigned int cpu);
void spm_set_low_power_mode(struct spm_driver_data *drv,
			    enum pm_sleep_mode mode);

/**
 * spm_collapse_via_scm() - does this SPM use the SCM TERMINATE_PC call?
 * @drv: SPM driver instance
 *
 * Returns true if the platform delegates CPU power-collapse to TrustZone
 * via SCM_BOOT/TERMINATE_PC (the typical mainline path). Returns false on
 * SoCs where the SCM call is not implemented by the TZ firmware and the
 * collapse must be driven by the SPM hardware sequence triggered from a
 * raw WFI instead -- currently MSM8660 / MSM8260 / APQ8060 (SAW v1.0).
 */
bool spm_collapse_via_scm(struct spm_driver_data *drv);

#endif /* __SPM_H__ */
