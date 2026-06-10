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

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/types.h>

struct qcom_rpm;

/*
 * MMSS NoC master port IDs (subset of MSM_BUS_MASTER_* enum from the
 * downstream msm_bus_board_8660.c). These are the bit positions in the
 * QCOM_RPM_MM_FABRIC_HALT mask that the helper accepts.
 *
 * Exposed in this header (not the qcom_mmss_porthalt.c .c file) so SoC-
 * agnostic consumer drivers (drivers/gpu/drm/msm/disp/mdp4/,
 * drivers/gpu/drm/msm/adreno/, drivers/media/platform/qcom/camss/, ...)
 * can request halts for the master ports they own without depending on
 * MSM8x60-specific clock-driver headers.
 */
#define QCOM_MMSS_PORT_MDP0	BIT(0)
#define QCOM_MMSS_PORT_MDP1	BIT(1)
#define QCOM_MMSS_PORT_GFX3D	BIT(4)

#if IS_ENABLED(CONFIG_QCOM_MMSS_PORT_HALT)
/**
 * qcom_mmss_porthalt_register_rpm() - Bind an RPM handle to the helper.
 * @rpm:	qcom_rpm handle that will service QCOM_RPM_MM_FABRIC_HALT.
 *		Pass NULL to unregister (e.g. from a provider .remove path).
 *
 * Called once by the MMCC provider (drivers/clk/qcom/mmcc-msm8660.c) after
 * it has resolved and pinned the RPM supplier via device_link_add(). The
 * helper caches a single handle; SoC-agnostic consumer drivers call
 * qcom_mmss_port_halt() without needing to know how the handle was
 * obtained.
 */
void qcom_mmss_porthalt_register_rpm(struct qcom_rpm *rpm);

/**
 * qcom_mmss_port_halt() - Halt or unhalt MMSS NoC master ports.
 * @port_mask:	bitmap of QCOM_MMSS_PORT_* IDs to act on.
 * @halt:	true to halt (quiesce AXI), false to unhalt.
 *
 * See the kerneldoc in drivers/soc/qcom/qcom_mmss_porthalt.c for full
 * semantics. Returns 0 / -EAGAIN / -ENODEV / -EINVAL / RPM IPC errno;
 * callers that treat halt as best-effort can ignore everything but 0.
 */
int qcom_mmss_port_halt(u32 port_mask, bool halt);
#else
static inline void qcom_mmss_porthalt_register_rpm(struct qcom_rpm *rpm) { }
static inline int qcom_mmss_port_halt(u32 port_mask, bool halt)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_QCOM_MMSS_PORTHALT_H__ */
