// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_DMA_QCOM_ADM_H
#define __LINUX_DMA_QCOM_ADM_H

#include <linux/types.h>

struct dma_chan;

struct qcom_adm_peripheral_config {
	u32 crci;
	u32 mux;

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

	/*
	 * Per-word byteswap of the data the ADM moves on this channel.
	 * Match Qualcomm's downstream ADM command-list bits:
	 *   swap_bytes  - exchange bytes inside each 16-bit half-word
	 *   swap_shorts - exchange the two 16-bit halves of each 32-bit word
	 * Both together yield a 32-bit big-endian / little-endian byteswap
	 * per dword (ntohl-equivalent). For DMA_MEM_TO_DEV channels these
	 * become DST_SWAP_* (bits 14/15); for DMA_DEV_TO_MEM they become
	 * SRC_SWAP_* (bits 11/12).
	 *
	 * Required by the CE2 (MSM8x60) Crypto Engine 2: the engine consumes
	 * data in big-endian-packed integer form at DATA_SHADOW0 and emits
	 * results the same way. Without ADM-side swap the engine produces
	 * deterministically wrong AES output past the first 4 blocks per
	 * GOPROC (engine's CRCI handshake is tied to ADM's per-word swap).
	 */
	bool swap_bytes;
	bool swap_shorts;

	/*
	 * Optional peripheral-side state dumper invoked by the ADM
	 * driver's per-channel watchdog when a transfer wedges. Lets
	 * the consumer driver (mmci-pl18x for SDCC, etc.) snapshot its
	 * own MMIO state (DATACTRL, MMCISTATUS, ...) at the EXACT moment
	 * the ADM channel determined it didn't get RSLT_VALID within
	 * the watchdog window. Output goes via dev_warn/dev_info on
	 * the consumer's device (already throttled by dev_*_ratelimited
	 * at the call site).
	 *
	 * Constraints:
	 *  - Called from timer softirq context, achan->vc.lock held,
	 *    IRQs off on the local CPU.
	 *  - Must not sleep, must not take locks that nest above vc.lock.
	 *  - Should restrict itself to single readl_relaxed of the
	 *    relevant peripheral registers and one printk.
	 */
	void (*dump_state)(void *dump_user);
	void *dump_user;
};

/**
 * qcom_adm_program_crci_ee0 - one-shot CRCI_CTL programmer for EE=0 window
 * @chan:     any ADM DMA channel on the target controller — only used to
 *            resolve the adm_device pointer; the CRCI to program does NOT
 *            need to be this channel's own CRCI. QCE uses two channels
 *            (rx/tx) but needs three CRCIs programmed (CE_IN, CE_OUT,
 *            CE_HASH); pass any one channel for all three calls.
 * @crci:     CRCI number to program (1..15; CRCI 0 is "no CRCI").
 * @crci_val: value to write — typically (mux_sel | blk_size).
 *
 * On APQ8060 / MSM8660 (Tenderloin) the live CRCI_CTL register lives at
 * EE=0. The mainline driver writes CRCI_CTL at adev->ee (=1 per DT), but
 * those writes are silently dropped on this SoC. The bootloader pre-programs
 * EE=0 for peripherals it enables (eMMC, NAND, SDC); peripherals that the
 * bootloader never touched (QCE crypto: CRCI 4=CE_IN, 5=CE_OUT, CE_HASH)
 * need the consumer driver to populate EE=0 once at probe while the channel
 * is idle.
 *
 * MUST be called only when the channel is idle (not mid-transfer). Writing
 * to a live CRCI_CTL register corrupts the in-flight burst.
 *
 * Returns 0 on success, -EINVAL if @chan is not an ADM channel or @crci is
 * out of range.
 */
int qcom_adm_program_crci_ee0(struct dma_chan *chan, u32 crci, u32 crci_val);

#endif /* __LINUX_DMA_QCOM_ADM_H */
