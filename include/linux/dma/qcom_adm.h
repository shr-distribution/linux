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
};

#endif /* __LINUX_DMA_QCOM_ADM_H */
