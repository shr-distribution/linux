// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_DMA_QCOM_ADM_H
#define __LINUX_DMA_QCOM_ADM_H

#include <linux/bits.h>
#include <linux/types.h>

struct dma_chan;

/*
 * Optional cmd_flags bits.  When set, the ADM hardware applies the
 * corresponding byte/short/word reorder to data flowing between memory
 * and the peripheral port, in-flight, per CRCI handshake.  The flags are
 * direction-agnostic in this API: the driver applies SRC_SWAP_* on
 * DMA_DEV_TO_MEM transfers (reorder data leaving the peripheral) and
 * DST_SWAP_* on DMA_MEM_TO_DEV transfers (reorder data entering the
 * peripheral).  In both cases the swap happens on the peripheral side
 * -- "swap whichever byte order the peripheral disagrees with my memory
 * layout on".
 *
 * Combined SWAP_BYTES | SWAP_SHORTS produces a full 32-bit byte reverse
 * (equivalent to cpu_to_be32 between LE memory and a BE peripheral).
 *
 * Set on the peripheral_config passed to dmaengine_slave_config().
 */
#define QCOM_ADM_CMD_FLAG_SWAP_BYTES	BIT(0)
#define QCOM_ADM_CMD_FLAG_SWAP_SHORTS	BIT(1)
#define QCOM_ADM_CMD_FLAG_SWAP_WORDS	BIT(2)

/*
 * Use the ADM controller's scatter-gather command mode instead of the
 * default BOX/SINGLE descriptors.  In SG mode the controller walks two
 * separate descriptor lists (one for each side of the transfer) per
 * CRCI handshake, which is what legacy webOS qce.c uses for the CE2
 * crypto engine -- BOX-mode multi-row internal walking doesn't pace
 * correctly through the CE2 engine's per-block CRCI flow control past
 * the first 1-2 blocks.
 *
 * When this flag is set, the peripheral provides a scatterlist via
 * dmaengine_prep_slave_sg() for the memory side; the ADM driver emits a
 * single SG command pointing at that scatterlist plus a single-entry
 * descriptor at the peripheral port (slave.{src,dst}_addr).
 */
#define QCOM_ADM_CMD_FLAG_MODE_SG	BIT(3)

struct qcom_adm_peripheral_config {
	u32 crci;
	u32 mux;

	/*
	 * Bitmask of QCOM_ADM_CMD_FLAG_* values OR'd into every BOX/SINGLE
	 * ADM command emitted on this channel.  Zero (unset) preserves
	 * existing behaviour (no in-flight swap).
	 */
	u32 cmd_flags;

	/*
	 * Optional pre-submit hook. If set, the ADM driver calls
	 * exec_func(exec_user) inside its per-controller submit lock,
	 * immediately before writing the channel's CMD_PTR register.
	 * Matches the legacy webOS msm_dmov "exec_func" pattern: the
	 * peripheral can write its own MMIO registers (e.g. SDCC
	 * DATACTRL + CMD) atomically with the ADM start, with all other
	 * channels on the same ADM controller blocked.
	 *
	 * Constraints:
	 *  - Called in atomic context with submit_lock held + IRQs off.
	 *  - Must not sleep, must not take other locks that nest above
	 *    submit_lock, must complete promptly (target: a handful of
	 *    register writes, microseconds).
	 *  - Called once per submission; the peripheral driver owns
	 *    lifetimes of exec_user.
	 */
	void (*exec_func)(void *exec_user);
	void *exec_user;
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
