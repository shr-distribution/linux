// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026, Herman van Hazendonk <github.com@herrie.org>
 *
 * Qualcomm MSM8x60-family legacy footswitch (GFS) driver.
 *
 * Standalone power-domain implementation for the older single-register
 * footswitch block used by the multimedia clocks on the MSM8x60 family
 * (MSM8260 / MSM8660 / APQ8060). The GFS block predates the modern
 * GDSC programming model in gdsc.c -- a dedicated file keeps the
 * legacy register layout, the fixed-delay sequencing and the
 * positive-logic ENABLE bit from cluttering the modern path with
 * if (sc->flags & LEGACY_FOOTSWITCH) branches throughout.
 *
 * The same block is found on the multimedia clock controllers of
 * MSM8960 and APQ8064, which is why the API lives in a generic
 * "footswitch" namespace rather than carrying the msm8660 prefix.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>

#include "footswitch.h"

/*
 * Legacy GDSCR bit assignments (positive-logic ENABLE):
 *   bit 5  CLAMP_MASK     - set to clamp I/O at safe values
 *   bit 8  ENABLE_MASK    - set to power the rail up
 *   bit 9  RETENTION_MASK - set to retain memory state across collapse
 *                           (cleared during teardown so peripherals
 *                            do not draw retention current at full off)
 */
#define LEGACY_FS_CLAMP_MASK		BIT(5)
#define LEGACY_FS_ENABLE_MASK		BIT(8)
#define LEGACY_FS_RETENTION_MASK	BIT(9)

/* Settle delays after the matching write. */
#define LEGACY_FS_RAIL_CHARGE_US	2
#define LEGACY_FS_CLAMP_RELEASE_US	5
#define LEGACY_FS_RESET_DELAY_US	1

#define pd_to_footswitch(_pd) container_of(_pd, struct footswitch, pd)

/*
 * Internal status return for footswitch_check_status():
 *   1  fully ON  (ENABLE=1, CLAMP=0)
 *   0  fully OFF (ENABLE=0)
 *  -EAGAIN  indeterminate (ENABLE=1, CLAMP=1 - bootloader handoff
 *           left the rail powered but isolated; gdsc-style boolean
 *           callers cannot represent this state, so we surface it
 *           as a distinct error)
 *  <0  hard error from the regmap read
 */
static int footswitch_check_status_raw(struct footswitch *fs)
{
	u32 val;
	int ret;

	ret = regmap_read(fs->regmap, fs->gdscr, &val);
	if (ret)
		return ret;

	if (!(val & LEGACY_FS_ENABLE_MASK))
		return 0;
	if (val & LEGACY_FS_CLAMP_MASK)
		return -EAGAIN;
	return 1;
}

static int footswitch_assert_clamp(struct footswitch *fs)
{
	return regmap_update_bits(fs->regmap, fs->gdscr,
				  LEGACY_FS_CLAMP_MASK, LEGACY_FS_CLAMP_MASK);
}

static int footswitch_deassert_clamp(struct footswitch *fs)
{
	return regmap_update_bits(fs->regmap, fs->gdscr,
				  LEGACY_FS_CLAMP_MASK, 0);
}

static int footswitch_set_enable(struct footswitch *fs, bool on)
{
	return regmap_update_bits(fs->regmap, fs->gdscr,
				  LEGACY_FS_ENABLE_MASK,
				  on ? LEGACY_FS_ENABLE_MASK : 0);
}

static inline int footswitch_deassert_reset(struct footswitch *fs)
{
	int i;

	for (i = 0; i < fs->reset_count; i++)
		fs->rcdev->ops->deassert(fs->rcdev, fs->resets[i]);
	return 0;
}

static inline int footswitch_assert_reset(struct footswitch *fs)
{
	int i;

	for (i = 0; i < fs->reset_count; i++)
		fs->rcdev->ops->assert(fs->rcdev, fs->resets[i]);
	return 0;
}

/*
 * Legacy enable sequence:
 *   0. take the regulator vote (if any)
 *   1. assert per-block resets (if SW_RESET)
 *   2. set ENABLE in GDSCR to power up the rail
 *   3. wait LEGACY_FS_RAIL_CHARGE_US for the rail to charge
 *   4. clear CLAMP in GDSCR to release the I/O clamp
 *   5. deassert resets (after clamp release, matching the vendor order)
 *   6. wait LEGACY_FS_CLAMP_RELEASE_US for clamps and signals to settle
 *
 * The block has no completion status bit, so the fixed delays above
 * are the only safe synchronisation point.
 *
 * Any rollback path leaves the rail collapsed AND the resets ASSERTED,
 * matching the steady-state of footswitch_disable() so a follow-up
 * enable always starts from a known state.
 */
static int footswitch_power_on(struct generic_pm_domain *domain)
{
	struct footswitch *fs = pd_to_footswitch(domain);
	int ret, rc;

	if (fs->rsupply) {
		ret = regulator_enable(fs->rsupply);
		if (ret)
			return ret;
	}

	if (fs->flags & FOOTSWITCH_SW_RESET) {
		footswitch_assert_reset(fs);
		udelay(LEGACY_FS_RESET_DELAY_US);
	}

	ret = footswitch_set_enable(fs, true);
	if (ret) {
		/*
		 * Rail did not power up. Leave resets asserted (matches
		 * footswitch_power_off()'s steady-state) and drop the
		 * regulator vote.
		 */
		if (fs->rsupply) {
			rc = regulator_disable(fs->rsupply);
			if (rc)
				pr_err("%s: regulator_disable failed (%d) after ENABLE-set failure (%d)\n",
				       fs->pd.name, rc, ret);
		}
		return ret;
	}

	udelay(LEGACY_FS_RAIL_CHARGE_US);

	ret = footswitch_deassert_clamp(fs);
	if (ret) {
		/*
		 * Rail is powered but I/O is clamped. Collapse the rail
		 * to a defined-off state, keep resets asserted, drop the
		 * regulator vote. Errors from these rollback steps are
		 * logged but do not override the original error; the
		 * caller already has indeterminate hardware state and
		 * the vote MUST be released regardless.
		 */
		rc = footswitch_set_enable(fs, false);
		if (rc) {
			/*
			 * Collapse failed: rail is still ON, drawing from
			 * the supply. Do NOT regulator_disable() -- cutting
			 * the supply while ENABLE is set risks hardware
			 * damage. Accept the vote leak (visible via the
			 * regulator sysfs use_count) over silicon damage.
			 */
			pr_err("%s: rail collapse rollback failed (%d) after clamp-release failure (%d); rail may be ON, regulator vote leaked\n",
			       fs->pd.name, rc, ret);
			return ret;
		}
		if (fs->rsupply) {
			rc = regulator_disable(fs->rsupply);
			if (rc)
				pr_err("%s: regulator_disable failed (%d) in clamp-release rollback\n",
				       fs->pd.name, rc);
		}
		return ret;
	}

	/* Deassert resets only after the clamp is released. */
	if (fs->flags & FOOTSWITCH_SW_RESET)
		footswitch_deassert_reset(fs);

	udelay(LEGACY_FS_CLAMP_RELEASE_US);

	/*
	 * Unhalt the NoC master ports owned by this domain (paired with
	 * the clamp release above; matches downstream footswitch-8x60.c
	 * msm_bus_axi_portunhalt() in footswitch_enable()). Best-effort:
	 * a failed unhalt leaves the rail powered + I/O unclamped, which
	 * is still safer than tearing the rail back down -- the consumer
	 * driver will hit a downstream bus error rather than indeterminate
	 * silicon state.
	 */
	if (fs->port_halt && fs->port_mask) {
		ret = fs->port_halt(fs->port_mask, false);
		if (ret)
			pr_warn("%s: NoC port unhalt (mask 0x%x) failed (%d); continuing\n",
				fs->pd.name, fs->port_mask, ret);
	}

	return 0;
}

/*
 * Legacy disable sequence:
 *   1. assert per-block resets (if SW_RESET) so the block is held
 *      while I/O is clamped
 *   2. set CLAMP in GDSCR to hold I/O at safe values across collapse
 *   3. clear ENABLE in GDSCR to collapse the rail
 *   4. drop the regulator vote (if any)
 *
 * If regulator_disable() fails after the rail is already collapsed,
 * the error is logged and SWALLOWED rather than returned to genpd.
 * Returning an error here would leave genpd thinking the domain is
 * still ON while the silicon is in fact off; the next consumer enable
 * would then be no-op'd by genpd and hit dead hardware. Better to
 * leak the regulator vote (visible via the regulator sysfs use_count)
 * than to corrupt genpd state.
 */
static int footswitch_power_off(struct generic_pm_domain *domain)
{
	struct footswitch *fs = pd_to_footswitch(domain);
	int ret;

	/*
	 * FOOTSWITCH_PWRSTS_ON domains do not support OFF. Honour the
	 * flag here so callers (genpd, or our own init path) cannot
	 * accidentally collapse the rail.
	 */
	if (fs->pwrsts == FOOTSWITCH_PWRSTS_ON)
		return 0;

	if (fs->flags & FOOTSWITCH_SW_RESET)
		footswitch_assert_reset(fs);

	/*
	 * Halt the NoC master ports owned by this domain BEFORE clamping
	 * the I/O. With the master port halted no new AXI traffic can be
	 * issued; the bus quiesces, the rail clamp below is safe, and any
	 * subsequent clock-halt-status poll on AXI clocks in this domain
	 * succeeds. Pairs with footswitch_power_on()'s portunhalt; matches
	 * downstream footswitch-8x60.c msm_bus_axi_porthalt() in
	 * footswitch_disable(). Best-effort: a failed halt does NOT abort
	 * the power-off -- proceeding to clamp + collapse is safer than
	 * leaving the rail powered with the halt request hanging.
	 */
	if (fs->port_halt && fs->port_mask) {
		ret = fs->port_halt(fs->port_mask, true);
		if (ret)
			pr_warn("%s: NoC port halt (mask 0x%x) failed (%d); continuing collapse\n",
				fs->pd.name, fs->port_mask, ret);
	}

	ret = footswitch_assert_clamp(fs);
	if (ret)
		return ret;

	ret = footswitch_set_enable(fs, false);
	if (ret)
		return ret;

	if (fs->rsupply) {
		ret = regulator_disable(fs->rsupply);
		if (ret) {
			pr_err("%s: regulator_disable failed (%d) after rail collapse; vote leaked, genpd state kept consistent with silicon\n",
			       fs->pd.name, ret);
			/* swallow */
		}
	}

	/* Clear RETENTION so the block does not draw retention current. */
	(void)regmap_update_bits(fs->regmap, fs->gdscr,
				 LEGACY_FS_RETENTION_MASK, 0);

	return 0;
}

/*
 * Per-footswitch init. Called from footswitch_register() for each
 * configured domain. Inspects the silicon state, reconciles the
 * regulator vote and genpd registration so they always agree with
 * what the bootloader actually left behind.
 */
static int footswitch_init(struct footswitch *fs)
{
	int initial_on = footswitch_check_status_raw(fs);
	bool on;
	int ret;

	if (initial_on < 0 && initial_on != -EAGAIN)
		return initial_on;

	/*
	 * Bootloader-clamped state (ENABLE=1, CLAMP=1) is forced on by
	 * fully running the enable sequence below -- safer than leaving
	 * the block half-configured where the regulator framework might
	 * later cut the parent supply while ENABLE is still set.
	 */
	if (initial_on == -EAGAIN) {
		ret = footswitch_power_on(&fs->pd);
		if (ret)
			return ret;
		on = true;
	} else {
		on = initial_on == 1;
	}

	/*
	 * FOOTSWITCH_PWRSTS_ON domains must be ON at registration time
	 * (the framework rejects GENPD_FLAG_ALWAYS_ON / RPM_ALWAYS_ON
	 * registration otherwise). Bring them up if needed -- but only
	 * if they are not already on, so a bootloader-held block
	 * (e.g. a splash framebuffer) is not cold-rebooted underneath
	 * us.
	 */
	if (fs->pwrsts == FOOTSWITCH_PWRSTS_ON && !on) {
		ret = footswitch_power_on(&fs->pd);
		if (ret)
			return ret;
		on = true;
	} else if (fs->flags & (FOOTSWITCH_ALWAYS_ON | FOOTSWITCH_RPM_ALWAYS_ON) && !on) {
		ret = footswitch_power_on(&fs->pd);
		if (ret)
			return ret;
		on = true;
	}

	/*
	 * Sync the kernel regulator state only if the silicon was ON at
	 * probe AND we did not just bring it up ourselves -- footswitch_
	 * power_on() already took the vote in that case, and re-voting
	 * here would leak a reference.
	 */
	if (on && initial_on == 1 && fs->rsupply) {
		ret = regulator_enable(fs->rsupply);
		if (ret)
			return ret;
	}

	fs->pd.power_off = footswitch_power_off;
	fs->pd.power_on = footswitch_power_on;
	if (fs->flags & FOOTSWITCH_ALWAYS_ON)
		fs->pd.flags |= GENPD_FLAG_ALWAYS_ON;
	if (fs->flags & FOOTSWITCH_RPM_ALWAYS_ON)
		fs->pd.flags |= GENPD_FLAG_RPM_ALWAYS_ON;

	ret = pm_genpd_init(&fs->pd, NULL, !on);
	if (ret)
		goto err_collapse;

	return 0;

err_collapse:
	/*
	 * pm_genpd_init() rejected our domain. The rail is currently ON
	 * (either bootloader-handoff or we just enabled it). Properly
	 * collapse it -- which also releases the regulator vote --
	 * BEFORE returning. Dropping the vote without collapsing would
	 * cut the supply while ENABLE is still set.
	 */
	if (on) {
		int rc = footswitch_power_off(&fs->pd);

		if (rc)
			pr_err("%s: failed to collapse rail (%d) after pm_genpd_init err (%d); rail may still be on\n",
			       fs->pd.name, rc, ret);
	}
	return ret;
}

/**
 * footswitch_register() - register a table of legacy footswitch domains
 * @desc:	descriptor describing this provider's footswitches
 * @rcdev:	reset controller backing the per-domain @resets arrays
 * @regmap:	regmap covering each footswitch's @gdscr offset
 *
 * Walks @desc->fs[], reconciles silicon state with regulator vote,
 * registers each as a generic_pm_domain, then publishes the
 * one-cell genpd provider for consumers (DT phandles).
 *
 * On failure, any domains already registered are torn down via
 * pm_genpd_remove() so the global gpd_list does not retain dangling
 * references to devm-managed footswitch storage.
 */
int footswitch_register(struct footswitch_desc *desc,
			struct reset_controller_dev *rcdev,
			struct regmap *regmap)
{
	struct genpd_onecell_data *data;
	struct device *dev = desc->dev;
	struct footswitch **fs = desc->fs;
	size_t num = desc->num;
	int i, ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->domains = devm_kcalloc(dev, num, sizeof(*data->domains),
				     GFP_KERNEL);
	if (!data->domains)
		return -ENOMEM;
	data->num_domains = num;

	for (i = 0; i < num; i++) {
		if (!fs[i] || !fs[i]->supply)
			continue;

		fs[i]->rsupply = devm_regulator_get_optional(dev, fs[i]->supply);
		if (IS_ERR(fs[i]->rsupply)) {
			ret = PTR_ERR(fs[i]->rsupply);
			if (ret != -ENODEV)
				return ret;
			fs[i]->rsupply = NULL;
		}
	}

	for (i = 0; i < num; i++) {
		if (!fs[i])
			continue;
		fs[i]->regmap = regmap;
		fs[i]->rcdev = rcdev;
		ret = footswitch_init(fs[i]);
		if (ret)
			goto err_genpd_remove;
		data->domains[i] = &fs[i]->pd;
	}

	ret = of_genpd_add_provider_onecell(dev->of_node, data);
	if (ret)
		goto err_genpd_remove;

	return 0;

err_genpd_remove:
	while (--i >= 0) {
		if (!fs[i])
			continue;
		pm_genpd_remove(&fs[i]->pd);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(footswitch_register);

/**
 * footswitch_unregister() - tear down a footswitch provider
 *
 * Removes the OF provider first so a consumer cannot grab a phandle
 * mid-teardown, then removes each domain from the global gpd_list.
 * The regulator vote and devres-managed allocations are released
 * when the underlying device is unbound.
 */
void footswitch_unregister(struct footswitch_desc *desc)
{
	struct footswitch **fs = desc->fs;
	size_t num = desc->num;
	size_t i;

	of_genpd_del_provider(desc->dev->of_node);

	for (i = 0; i < num; i++) {
		if (!fs[i])
			continue;
		pm_genpd_remove(&fs[i]->pd);
	}
}
EXPORT_SYMBOL_GPL(footswitch_unregister);

MODULE_DESCRIPTION("Qualcomm MSM8x60-family legacy footswitch (GFS) driver");
MODULE_LICENSE("GPL");
