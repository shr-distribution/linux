/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm MSM8x60-family MMSS NoC master port halt helper.
 *
 * Wraps the QCOM_RPM_MM_FABRIC_HALT IPC so multiple consumers (the MMCC
 * footswitch power_off path, plus per-subsystem .suspend_late hooks on
 * mdp4 / adreno / camss-vfe / vidc) can independently request a port
 * halt without fighting each other. The helper refcounts per port and
 * only emits an RPM IPC on a transition.
 *
 * Copyright (c) 2026, Herman van Hazendonk <github.com@herrie.org>
 */
#ifndef __SOC_QCOM_MMSS_PORTHALT_H__
#define __SOC_QCOM_MMSS_PORTHALT_H__

#include <linux/errno.h>
#include <linux/types.h>

struct qcom_rpm;

#if IS_ENABLED(CONFIG_QCOM_MMSS_PORT_HALT)
int qcom_mmss_port_halt(struct qcom_rpm *rpm, u32 port_mask, bool halt);
#else
static inline int qcom_mmss_port_halt(struct qcom_rpm *rpm, u32 port_mask,
				      bool halt)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_QCOM_MMSS_PORTHALT_H__ */
