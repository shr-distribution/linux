// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_DMA_QCOM_ADM_H
#define __LINUX_DMA_QCOM_ADM_H

#include <linux/types.h>

struct dma_chan;

struct qcom_adm_peripheral_config {
	u32 crci;
	u32 mux;
};

/**
 * qcom_adm_program_crci_ee0 - one-shot CRCI_CTL programmer for EE=0 window
 * @chan:     ADM DMA channel handle (from dma_request_chan)
 * @crci_val: value to write — typically (mux_sel | blk_size)
 *
 * On APQ8060 / MSM8660 (Tenderloin) the live CRCI_CTL register lives at
 * EE=0. The mainline driver writes CRCI_CTL at adev->ee (=1 per DT), but
 * those writes are silently dropped on this SoC. The bootloader pre-programs
 * EE=0 for peripherals it enables (eMMC, NAND, SDC); peripherals that the
 * bootloader never touched (QCE crypto: CRCI 4=CE_IN, 5=CE_OUT) need the
 * consumer driver to populate EE=0 once at probe while the channel is idle.
 *
 * MUST be called only when the channel is idle (not mid-transfer). Writing
 * to a live CRCI_CTL register corrupts the in-flight burst.
 *
 * Returns 0 on success, -EINVAL if chan is not an ADM channel with a CRCI.
 */
int qcom_adm_program_crci_ee0(struct dma_chan *chan, u32 crci_val);

#endif /* __LINUX_DMA_QCOM_ADM_H */
