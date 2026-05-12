// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm RPM sleep-set programmer for MSM8660 / APQ8060.
 *
 * Programs the Resource Power Manager's SLEEP-context resource votes
 * so that when all CPU masters signal sleep (via SPM master_stat
 * bits with rpm_bypass=0), the RPM drops shared resources to their
 * deepest safe levels. This is the missing piece on top of the
 * "PM_SLEEP_MODE_PC" cluster idle state — without it the RPM keeps
 * resources at their active-set values even while the cluster is
 * power-collapsed, which negates most of the standby power saving.
 *
 * On legacy webOS the equivalent voting was done by an aggregator
 * (arch/arm/mach-msm/rpm_resources.c) that combined votes from the
 * regulator, clock, XO and cpuidle subsystems. For the initial port
 * a static minimum sleep-set is sufficient: the RPM applies these
 * values only during the cluster-sleep window, so they don't affect
 * runtime operation.
 *
 * Resources voted in the SLEEP context (all values = 0):
 *   PXO_CLK            27 MHz crystal oscillator off
 *   CXO_CLK            19.2 MHz crystal oscillator off
 *   EBI1_CLK           DDR memory bus clock off
 *   SMI_CLK            SMI (vdd_mem proxy) clock off
 *   APPS_L2_CACHE_CTL  L2 in HSFS_OPEN (deepest L2 sleep)
 *
 * Zero means "I have no vote for any clock rate" — the RPM then
 * aggregates all votes and uses the highest. With nothing voting,
 * the RPM is free to drop the resource entirely.
 *
 * The driver binds as a child of the qcom,rpm-msm8660 MFD node and
 * reaches the rpm handle via dev_get_drvdata(parent).
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/qcom_rpm.h>

#include <dt-bindings/mfd/qcom-rpm.h>

struct rpm_sleep_resource {
	const char *name;
	int resource_id;
};

/*
 * Order matters only for log readability — RPM does not care.
 * APPS_L2_CACHE_CTL is written last so that if any earlier write
 * fails the L2 stays in its current active-set state (safe).
 */
static const struct rpm_sleep_resource msm8660_sleep_resources[] = {
	{ "PXO_CLK",           QCOM_RPM_PXO_CLK },
	{ "CXO_CLK",           QCOM_RPM_CXO_CLK },
	{ "EBI1_CLK",          QCOM_RPM_EBI1_CLK },
	{ "SMI_CLK",           QCOM_RPM_SMI_CLK },
	{ "APPS_L2_CACHE_CTL", QCOM_RPM_APPS_L2_CACHE_CTL },
};

static int rpm_sleep_set_probe(struct platform_device *pdev)
{
	struct qcom_rpm *rpm;
	u32 zero = 0;
	int i, ret;
	int failures = 0;

	rpm = dev_get_drvdata(pdev->dev.parent);
	if (!rpm) {
		dev_err(&pdev->dev, "no parent rpm handle\n");
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(msm8660_sleep_resources); i++) {
		const struct rpm_sleep_resource *r = &msm8660_sleep_resources[i];

		ret = qcom_rpm_write(rpm, QCOM_RPM_SLEEP_STATE,
				     r->resource_id, &zero, 1);
		if (ret) {
			/*
			 * A failure here means the RPM rejected the
			 * vote — usually because another master has a
			 * conflicting active-set vote that the RPM
			 * cannot reconcile with a 0 sleep-set. Log and
			 * continue: even partial sleep-set programming
			 * gives some standby benefit.
			 */
			dev_warn(&pdev->dev,
				 "sleep-set vote for %s failed: %d\n",
				 r->name, ret);
			failures++;
			continue;
		}

		dev_dbg(&pdev->dev, "sleep-set %s = 0 OK\n", r->name);
	}

	if (failures == ARRAY_SIZE(msm8660_sleep_resources)) {
		dev_err(&pdev->dev,
			"all sleep-set votes failed — RPM not reachable?\n");
		return -EIO;
	}

	dev_info(&pdev->dev,
		 "RPM sleep-set programmed (%d/%d resources)\n",
		 (int)ARRAY_SIZE(msm8660_sleep_resources) - failures,
		 (int)ARRAY_SIZE(msm8660_sleep_resources));

	return 0;
}

static const struct of_device_id rpm_sleep_set_of_match[] = {
	{ .compatible = "qcom,rpm-sleep-set-msm8660" },
	{ }
};
MODULE_DEVICE_TABLE(of, rpm_sleep_set_of_match);

static struct platform_driver rpm_sleep_set_driver = {
	.probe = rpm_sleep_set_probe,
	.driver = {
		.name = "qcom-rpm-sleep-set-msm8660",
		.of_match_table = rpm_sleep_set_of_match,
	},
};
module_platform_driver(rpm_sleep_set_driver);

MODULE_DESCRIPTION("Qualcomm RPM sleep-set programmer for MSM8660/APQ8060");
MODULE_LICENSE("GPL");
