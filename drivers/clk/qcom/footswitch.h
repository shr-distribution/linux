/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026, Herman van Hazendonk <github.com@herrie.org>
 *
 * Qualcomm MSM8x60-family legacy footswitch (GFS) framework.
 *
 * The GFS block predates the modern qcom GDSC programming model:
 *   - CLAMP, ENABLE and RETENTION bits all live in a single GDSCR
 *     register (no separate clamp_io_ctrl);
 *   - no power-status bit, so software has to gate progress on fixed
 *     udelay() instead of polling completion;
 *   - ENABLE is positive-logic (set to power up, clear to collapse)
 *     instead of the modern inverted SW_COLLAPSE_MASK;
 *   - none of the modern wait-time / HW-trigger / SW-override fields
 *     exist, so the modern wait-config programming block does not
 *     apply.
 *
 * Keep this independent of struct gdsc / drivers/clk/qcom/gdsc.c so
 * neither code path has to dispatch on a flag; the modern path stays
 * clean and the legacy path can be reused by other Qualcomm SoCs that
 * still ship GFS-style power domains (e.g. MSM8960, APQ8064).
 */
#ifndef __QCOM_FOOTSWITCH_H__
#define __QCOM_FOOTSWITCH_H__

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/pm_domain.h>

struct regmap;
struct regulator;
struct reset_controller_dev;

/* Footswitch flags (per-domain capability bits). */
#define FOOTSWITCH_SW_RESET		BIT(0)
#define FOOTSWITCH_ALWAYS_ON		BIT(1)
#define FOOTSWITCH_RPM_ALWAYS_ON	BIT(2)

/*
 * Footswitch power states. Legacy footswitches only support
 * full-off and full-on (no retention or HW-trigger modes);
 * a third pseudo-state lets a domain declare itself ON-only
 * (must stay powered, must never collapse) for blocks the
 * downstream vendor driver also pinned ON.
 */
enum footswitch_pwr_state {
	FOOTSWITCH_PWRSTS_OFF_ON = 0,
	FOOTSWITCH_PWRSTS_ON,
};

/**
 * struct footswitch - per-domain legacy footswitch descriptor
 * @pd:		generic_pm_domain backing this footswitch
 * @gdscr:	address of the per-domain GDSCR (single register that
 *		holds ENABLE / CLAMP / RETENTION bits)
 * @regmap:	regmap covering @gdscr (set by footswitch_register())
 * @rsupply:	optional parent regulator (resolved during register())
 * @supply:	devicetree supply name for @rsupply; NULL means "no parent"
 * @rcdev:	reset controller backing @resets
 * @resets:	array of reset IDs to assert/deassert around enable
 * @reset_count:number of entries in @resets
 * @flags:	FOOTSWITCH_SW_RESET / _ALWAYS_ON / _RPM_ALWAYS_ON
 * @pwrsts:	power-state capability of this domain
 * @port_mask:	bitmap of NoC master ports owned by this power domain.
 *		Zero (the default) means the domain has no associated NoC
 *		master and the @port_halt callback is not invoked.
 * @port_halt:	optional callback that halts (@halt=true) or unhalts
 *		(@halt=false) the @port_mask ports at the NoC fabric.
 *		Called from footswitch_power_off() before clamping the
 *		rail and from footswitch_power_on() after unclamping it,
 *		matching the downstream footswitch-8x60.c pairing of
 *		msm_bus_axi_porthalt/portunhalt with rail clamp/unclamp.
 *
 *		PM-phase contract: the callback runs at whatever PM phase
 *		genpd_power_on/off fires the footswitch_power_on/off
 *		callback. For runtime PM that is runtime-suspend context
 *		(IRQs enabled). For system suspend genpd hard-wires
 *		power_off to .suspend_noirq (drivers/pmdomain/core.c) --
 *		after dpm_suspend_noirq() has called suspend_device_irqs()
 *		-- so any callback that relies on IRQ-driven IPC (notably
 *		qcom_rpm_write, which waits 5*HZ for the RPM ack IRQ) will
 *		time out and return -ETIMEDOUT. The MSM8x60 MMCC provider
 *		uses qcom_mmss_port_halt() under the hood; that helper
 *		refcounts per port, so a system-suspend collapse where the
 *		consumer's .suspend_late hook already halted the port sees
 *		no transition here and returns 0 immediately. Errors are
 *		logged but do not abort the power transition (best-effort:
 *		the rail still gets clamped / unclamped even if the NoC
 *		write fails or is skipped).
 */
struct footswitch {
	struct generic_pm_domain	pd;
	unsigned int			gdscr;
	struct regmap			*regmap;
	struct regulator		*rsupply;
	const char			*supply;
	struct reset_controller_dev	*rcdev;
	unsigned int			*resets;
	unsigned int			reset_count;
	u32				flags;
	enum footswitch_pwr_state	pwrsts;
	u32				port_mask;
	int				(*port_halt)(u32 port_mask, bool halt);
};

/**
 * struct footswitch_desc - per-provider footswitch table
 * @dev:	provider device (regulator parent, devres anchor)
 * @fs:		array of @num footswitches owned by this provider
 * @num:	number of entries in @fs (may include NULL slots,
 *		which are skipped)
 * @pd_list:	allocated by footswitch_register(), torn down by
 *		footswitch_unregister()
 */
struct footswitch_desc {
	struct device			*dev;
	struct footswitch		**fs;
	size_t				num;
	struct dev_pm_domain_list	*pd_list;
};

#ifdef CONFIG_QCOM_FOOTSWITCH
int footswitch_register(struct footswitch_desc *desc,
			struct reset_controller_dev *rcdev,
			struct regmap *regmap);
void footswitch_unregister(struct footswitch_desc *desc);
#else
static inline int footswitch_register(struct footswitch_desc *desc,
				      struct reset_controller_dev *rcdev,
				      struct regmap *regmap)
{
	return -ENOSYS;
}

static inline void footswitch_unregister(struct footswitch_desc *desc) {}
#endif /* CONFIG_QCOM_FOOTSWITCH */

#endif /* __QCOM_FOOTSWITCH_H__ */
