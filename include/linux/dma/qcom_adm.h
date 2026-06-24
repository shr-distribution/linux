// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_DMA_QCOM_ADM_H
#define __LINUX_DMA_QCOM_ADM_H

#include <linux/types.h>

struct qcom_adm_peripheral_config {
	u32 crci;
	u32 mux;

	/*
	 * Optional pre-submit hook. If set, the ADM driver calls
	 * exec_func(exec_user) immediately before writing the channel's
	 * CMD_PTR register, so the peripheral can commit its own MMIO
	 * (e.g. an SDCC's DATACTRL + data command) atomically with the ADM
	 * start. Required by flow-controlled peripherals whose hardware
	 * latches the CRCI state at CMD_PTR time and must be armed in the
	 * same window. Called in atomic context; must not sleep and must
	 * complete promptly (a few register writes).
	 */
	void (*exec_func)(void *exec_user);
	void *exec_user;
};

#endif /* __LINUX_DMA_QCOM_ADM_H */
