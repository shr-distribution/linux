// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_DMA_QCOM_ADM_H
#define __LINUX_DMA_QCOM_ADM_H

#include <linux/types.h>

struct qcom_adm_peripheral_config {
	u32 crci;
	u32 mux;
};

struct dma_chan;

/*
 * Tenderloin (APQ8060 / Palm) quirk helper: pre-program a channel's
 * CRCI_CTL register in the EE=0 window. On this SoC the EE=1 CRCI_CTL
 * writes that mainline qcom_adm.c emits per-descriptor are silently
 * dropped — the live state lives at EE=0. The bootloader pre-programs
 * EE=0 for the peripherals it enables (MMC's CRCI=1); peripherals
 * brought up only under mainline (e.g. QCE crypto, CRCIs 4 and 5)
 * need the consumer driver to populate EE=0 once at probe.
 *
 * MUST be called only when the channel is idle. Writes during an
 * active flow corrupt in-flight bursts.
 *
 * Returns 0 on success, -EINVAL if the channel has no CRCI assigned.
 * No-op on non-Tenderloin builds (compile-time guarded by the caller).
 */
int qcom_adm_program_crci_ee0(struct dma_chan *chan, u32 crci_val);

#endif /* __LINUX_DMA_QCOM_ADM_H */
