// SPDX-License-Identifier: GPL-2.0-only
/*
 * Restart and power-off support for Qualcomm PM8058 (SSBI) based boards,
 * e.g. the APQ8060/MSM8660 HP TouchPad.
 *
 * The PM8058 decides whether dropping the SoC PS_HOLD line results in a
 * warm reset or a power-off based on the PON_CNTL_1.WD_EN latch.  This
 * driver programs that latch (and, for power-off, the extra rail/SMPL
 * sequence used by the vendor kernel) and then drops PS_HOLD via a syscon
 * mapping of the TLMM register that holds it.
 *
 * A PS_HOLD-triggered PMIC reset re-runs the PON sequence and power-cycles
 * the regulator rails, which gives downstream devices (notably the eMMC on
 * vmmc = pm8901_l5) a clean power-on reset.  This mirrors the legacy webOS
 * reset path (arch/arm/mach-msm/restart.c + drivers/mfd/pmic8058.c) and
 * avoids the warm-boot eMMC wedge seen when the SoC is reset by the
 * watchdog alone (which leaves the PMIC, and therefore the eMMC, powered).
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

/* PM8058 SSBI register map (1 byte each). */
#define PM8058_PON_CNTL_1	0x01c
#define  PM8058_PON_WD_EN	BIT(3)	/* 1 = reset, 0 = power-off on PS_HOLD */
#define  PM8058_PON_PUP_MASK	0xf0	/* enable all power-up pullups */
#define PM8058_SLEEP_CNTL	0x02b
#define  PM8058_SLEEP_SMPL_EN	BIT(2)	/* SMPL auto-restart */
#define PM8058_L22_CTRL		0x121
#define  PM8058_L22_PD		BIT(6)	/* pull-down: preserve current state */
#define  PM8058_L22_1V225_HPM	0x93	/* L22 = 1.225 V, high-power mode */

struct pm8058_poweroff {
	struct regmap *regmap;		/* PM8058 SSBI regmap (MFD parent) */
	struct regmap *ps_hold;		/* TLMM PS_HOLD syscon */
	u32 ps_hold_offset;
};

static void pm8058_poweroff_set_latch(struct pm8058_poweroff *po, bool reset)
{
	/* Force L22 to 1.225 V HPM, preserving only the pull-down bit. */
	regmap_update_bits(po->regmap, PM8058_L22_CTRL,
			   (u8)~PM8058_L22_PD, PM8058_L22_1V225_HPM);

	/* Power-off only: disable SMPL so the PMIC does not auto-restart. */
	if (!reset)
		regmap_update_bits(po->regmap, PM8058_SLEEP_CNTL,
				   PM8058_SLEEP_SMPL_EN, 0);

	/* Select reset vs power-off, and enable all pullups. */
	regmap_update_bits(po->regmap, PM8058_PON_CNTL_1,
			   PM8058_PON_WD_EN | PM8058_PON_PUP_MASK,
			   (reset ? PM8058_PON_WD_EN : 0) | PM8058_PON_PUP_MASK);
}

static void pm8058_poweroff_drop_ps_hold(struct pm8058_poweroff *po)
{
	/* Dropping PS_HOLD lets the PMIC act on the latch configured above. */
	regmap_write(po->ps_hold, po->ps_hold_offset, 0);
	mdelay(10000);
}

static int pm8058_restart(struct sys_off_data *data)
{
	struct pm8058_poweroff *po = data->cb_data;

	pm8058_poweroff_set_latch(po, true);
	pm8058_poweroff_drop_ps_hold(po);

	return NOTIFY_DONE;
}

static int pm8058_poweroff(struct sys_off_data *data)
{
	struct pm8058_poweroff *po = data->cb_data;

	pm8058_poweroff_set_latch(po, false);
	pm8058_poweroff_drop_ps_hold(po);

	return NOTIFY_DONE;
}

static int pm8058_poweroff_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pm8058_poweroff *po;
	int ret;

	po = devm_kzalloc(dev, sizeof(*po), GFP_KERNEL);
	if (!po)
		return -ENOMEM;

	po->regmap = dev_get_regmap(dev->parent, NULL);
	if (!po->regmap)
		return dev_err_probe(dev, -ENODEV,
				     "no regmap on parent PM8058\n");

	po->ps_hold = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "qcom,ps-hold");
	if (IS_ERR(po->ps_hold))
		return dev_err_probe(dev, PTR_ERR(po->ps_hold),
				     "failed to look up PS_HOLD syscon\n");

	of_property_read_u32(dev->of_node, "qcom,ps-hold-offset",
			     &po->ps_hold_offset);

	/*
	 * Run the restart handler ahead of the watchdog-bite restart
	 * (registered at default priority): a PMIC reset power-cycles the
	 * rails, whereas the watchdog only resets the apps subsystem.
	 */
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART,
					    SYS_OFF_PRIO_HIGH,
					    pm8058_restart, po);
	if (ret)
		return ret;

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
					    SYS_OFF_PRIO_DEFAULT,
					    pm8058_poweroff, po);
	if (ret)
		return ret;

	dev_info(dev, "PM8058 PS_HOLD restart/power-off registered\n");

	return 0;
}

static const struct of_device_id pm8058_poweroff_of_match[] = {
	{ .compatible = "qcom,pm8058-poweroff" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8058_poweroff_of_match);

static struct platform_driver pm8058_poweroff_driver = {
	.probe = pm8058_poweroff_probe,
	.driver = {
		.name = "pm8058-poweroff",
		.of_match_table = pm8058_poweroff_of_match,
	},
};
module_platform_driver(pm8058_poweroff_driver);

MODULE_DESCRIPTION("Qualcomm PM8058 PS_HOLD restart/power-off driver");
MODULE_LICENSE("GPL");
