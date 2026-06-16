// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016 Linaro Ltd
 */
#include <linux/module.h>
#include <linux/ulpi/driver.h>
#include <linux/ulpi/regs.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>
#include <linux/extcon.h>
#include <linux/notifier.h>

#define ULPI_PWR_CLK_MNG_REG		0x88
# define ULPI_PWR_OTG_COMP_DISABLE	BIT(0)

#define ULPI_MISC_A			0x96
# define ULPI_MISC_A_VBUSVLDEXTSEL	BIT(1)
# define ULPI_MISC_A_VBUSVLDEXT		BIT(0)


struct ulpi_seq {
	u8 addr;
	u8 val;
};

struct qcom_usb_hs_phy {
	struct ulpi *ulpi;
	struct phy *phy;
	struct clk *ref_clk;
	struct clk *sleep_clk;
	struct regulator *v1p8;
	struct regulator *v3p3;
	struct reset_control *reset;
	struct ulpi_seq *init_seq;
	/*
	 * Optional sequence of writes targeting raw ULPI addresses (no
	 * ULPI_EXT_VENDOR_SPECIFIC base added). Populated from the
	 * "qcom,vendor-init-seq" DT property. Applied after PHY reset
	 * so the values survive the reset that follows init_seq writes.
	 */
	struct ulpi_seq *vendor_init_seq;
	struct extcon_dev *vbus_edev;
	struct notifier_block vbus_notify;
};

static int qcom_usb_hs_phy_set_mode(struct phy *phy,
				    enum phy_mode mode, int submode)
{
	struct qcom_usb_hs_phy *uphy = phy_get_drvdata(phy);
	u8 addr;
	int ret;

	if (!uphy->vbus_edev) {
		u8 val = 0;

		switch (mode) {
		case PHY_MODE_USB_OTG:
		case PHY_MODE_USB_HOST:
			val |= ULPI_INT_IDGRD;
			fallthrough;
		case PHY_MODE_USB_DEVICE:
			val |= ULPI_INT_SESS_VALID;
			break;
		default:
			break;
		}

		ret = ulpi_write(uphy->ulpi, ULPI_USB_INT_EN_RISE, val);
		if (ret)
			return ret;
		ret = ulpi_write(uphy->ulpi, ULPI_USB_INT_EN_FALL, val);
	} else {
		switch (mode) {
		case PHY_MODE_USB_OTG:
		case PHY_MODE_USB_DEVICE:
			addr = ULPI_SET(ULPI_MISC_A);
			break;
		case PHY_MODE_USB_HOST:
			addr = ULPI_CLR(ULPI_MISC_A);
			break;
		default:
			return -EINVAL;
		}

		ret = ulpi_write(uphy->ulpi, ULPI_SET(ULPI_PWR_CLK_MNG_REG),
				 ULPI_PWR_OTG_COMP_DISABLE);
		if (ret)
			return ret;
		ret = ulpi_write(uphy->ulpi, addr, ULPI_MISC_A_VBUSVLDEXTSEL);
	}

	return ret;
}

static int
qcom_usb_hs_phy_vbus_notifier(struct notifier_block *nb, unsigned long event,
			      void *ptr)
{
	struct qcom_usb_hs_phy *uphy;
	u8 addr;

	uphy = container_of(nb, struct qcom_usb_hs_phy, vbus_notify);

	if (event)
		addr = ULPI_SET(ULPI_MISC_A);
	else
		addr = ULPI_CLR(ULPI_MISC_A);

	return ulpi_write(uphy->ulpi, addr, ULPI_MISC_A_VBUSVLDEXT);
}

static int qcom_usb_hs_phy_power_on(struct phy *phy)
{
	struct qcom_usb_hs_phy *uphy = phy_get_drvdata(phy);
	struct ulpi *ulpi = uphy->ulpi;
	const struct ulpi_seq *seq;
	int ret, state;

	ret = clk_prepare_enable(uphy->ref_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(uphy->sleep_clk);
	if (ret)
		goto err_sleep;

	ret = regulator_set_load(uphy->v1p8, 50000);
	if (ret < 0)
		goto err_1p8;

	ret = regulator_enable(uphy->v1p8);
	if (ret)
		goto err_1p8;

	ret = regulator_set_voltage_triplet(uphy->v3p3, 3050000, 3300000,
					    3300000);
	if (ret)
		goto err_3p3;

	ret = regulator_set_load(uphy->v3p3, 50000);
	if (ret < 0)
		goto err_3p3;

	ret = regulator_enable(uphy->v3p3);
	if (ret)
		goto err_3p3;

	for (seq = uphy->init_seq; seq->addr; seq++) {
		ret = ulpi_write(ulpi, ULPI_EXT_VENDOR_SPECIFIC + seq->addr,
				 seq->val);
		if (ret)
			goto err_ulpi;
	}

	if (uphy->reset) {
		ret = reset_control_reset(uphy->reset);
		if (ret)
			goto err_ulpi;
	}

	/*
	 * Apply board-specific raw-address ULPI writes after the PHY reset
	 * so they survive register restore. Used to reach the standard
	 * vendor range 0x30-0x3F which qcom,init-seq (above) cannot —
	 * pre-emphasis level / HS driver slope / CDR auto-reset etc. live
	 * there on MSM8660-class hardware.
	 */
	for (seq = uphy->vendor_init_seq; seq->addr; seq++) {
		ret = ulpi_write(ulpi, seq->addr, seq->val);
		if (ret)
			goto err_ulpi;
	}

	if (uphy->vbus_edev) {
		state = extcon_get_state(uphy->vbus_edev, EXTCON_USB);
		/* setup initial state */
		qcom_usb_hs_phy_vbus_notifier(&uphy->vbus_notify, state,
					      uphy->vbus_edev);
		ret = extcon_register_notifier(uphy->vbus_edev, EXTCON_USB,
					       &uphy->vbus_notify);
		if (ret)
			goto err_ulpi;
	}

	return 0;
err_ulpi:
	regulator_disable(uphy->v3p3);
err_3p3:
	regulator_disable(uphy->v1p8);
err_1p8:
	clk_disable_unprepare(uphy->sleep_clk);
err_sleep:
	clk_disable_unprepare(uphy->ref_clk);
	return ret;
}

static int qcom_usb_hs_phy_power_off(struct phy *phy)
{
	struct qcom_usb_hs_phy *uphy = phy_get_drvdata(phy);

	if (uphy->vbus_edev)
		extcon_unregister_notifier(uphy->vbus_edev, EXTCON_USB,
					   &uphy->vbus_notify);
	regulator_disable(uphy->v3p3);
	regulator_disable(uphy->v1p8);
	clk_disable_unprepare(uphy->sleep_clk);
	clk_disable_unprepare(uphy->ref_clk);

	return 0;
}

static const struct phy_ops qcom_usb_hs_phy_ops = {
	.power_on = qcom_usb_hs_phy_power_on,
	.power_off = qcom_usb_hs_phy_power_off,
	.set_mode = qcom_usb_hs_phy_set_mode,
	.owner = THIS_MODULE,
};

/*
 * The binding caps both qcom,init-seq and qcom,vendor-init-seq at
 * maxItems: 32 (addr, val) pairs, i.e. 64 bytes total. Enforce that
 * limit here so a malformed DT cannot drive an unbounded
 * devm_kmalloc_array() and so the misconfiguration is visible at
 * probe time instead of silently truncated.
 */
#define QCOM_USB_HS_PHY_INIT_SEQ_MAX_PAIRS	32
#define QCOM_USB_HS_PHY_INIT_SEQ_MAX_BYTES	\
	(QCOM_USB_HS_PHY_INIT_SEQ_MAX_PAIRS * 2)

static int qcom_usb_hs_phy_parse_init_seq(struct ulpi *ulpi,
					  const char *propname,
					  struct ulpi_seq **out)
{
	struct ulpi_seq *seq;
	int size;

	size = of_property_count_u8_elems(ulpi->dev.of_node, propname);
	if (size < 0)
		size = 0;
	if (size > QCOM_USB_HS_PHY_INIT_SEQ_MAX_BYTES) {
		dev_err(&ulpi->dev,
			"%s: %d bytes exceeds %d-byte maximum\n",
			propname, size, QCOM_USB_HS_PHY_INIT_SEQ_MAX_BYTES);
		return -EINVAL;
	}
	if (size % 2) {
		dev_err(&ulpi->dev,
			"%s: %d bytes is not a whole number of (addr, val) pairs\n",
			propname, size);
		return -EINVAL;
	}

	seq = devm_kmalloc_array(&ulpi->dev, (size / 2) + 1, sizeof(*seq),
				 GFP_KERNEL);
	if (!seq)
		return -ENOMEM;

	if (size) {
		int ret = of_property_read_u8_array(ulpi->dev.of_node,
						    propname, (u8 *)seq, size);
		if (ret)
			return ret;
	}
	/* NUL-terminate so the power_on loop's seq->addr-as-sentinel works. */
	seq[size / 2].addr = 0;
	seq[size / 2].val = 0;

	*out = seq;
	return 0;
}

static int qcom_usb_hs_phy_probe(struct ulpi *ulpi)
{
	struct qcom_usb_hs_phy *uphy;
	struct phy_provider *p;
	struct clk *clk;
	struct regulator *reg;
	struct reset_control *reset;
	int ret;

	uphy = devm_kzalloc(&ulpi->dev, sizeof(*uphy), GFP_KERNEL);
	if (!uphy)
		return -ENOMEM;
	ulpi_set_drvdata(ulpi, uphy);
	uphy->ulpi = ulpi;

	ret = qcom_usb_hs_phy_parse_init_seq(ulpi, "qcom,init-seq",
					     &uphy->init_seq);
	if (ret)
		return ret;
	/*
	 * Optional raw-address vendor init sequence — same encoding as
	 * qcom,init-seq (u8 addr/val pairs) but each pair is written to
	 * the raw ULPI address rather than to ULPI_EXT_VENDOR_SPECIFIC +
	 * addr. Lets boards reach the standard vendor range 0x30-0x3F.
	 */
	ret = qcom_usb_hs_phy_parse_init_seq(ulpi, "qcom,vendor-init-seq",
					     &uphy->vendor_init_seq);
	if (ret)
		return ret;

	uphy->ref_clk = clk = devm_clk_get(&ulpi->dev, "ref");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	uphy->sleep_clk = clk = devm_clk_get(&ulpi->dev, "sleep");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	uphy->v1p8 = reg = devm_regulator_get(&ulpi->dev, "v1p8");
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	uphy->v3p3 = reg = devm_regulator_get(&ulpi->dev, "v3p3");
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	uphy->reset = reset = devm_reset_control_get_optional_exclusive(&ulpi->dev, "por");
	if (IS_ERR(reset))
		return dev_err_probe(&ulpi->dev, PTR_ERR(reset),
				     "failed to get reset control\n");

	uphy->phy = devm_phy_create(&ulpi->dev, ulpi->dev.of_node,
				    &qcom_usb_hs_phy_ops);
	if (IS_ERR(uphy->phy))
		return PTR_ERR(uphy->phy);

	uphy->vbus_edev = extcon_get_edev_by_phandle(&ulpi->dev, 0);
	if (IS_ERR(uphy->vbus_edev)) {
		if (PTR_ERR(uphy->vbus_edev) != -ENODEV)
			return PTR_ERR(uphy->vbus_edev);
		uphy->vbus_edev = NULL;
	}

	uphy->vbus_notify.notifier_call = qcom_usb_hs_phy_vbus_notifier;
	phy_set_drvdata(uphy->phy, uphy);

	p = devm_of_phy_provider_register(&ulpi->dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(p);
}

static const struct of_device_id qcom_usb_hs_phy_match[] = {
	{ .compatible = "qcom,usb-hs-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_usb_hs_phy_match);

static struct ulpi_driver qcom_usb_hs_phy_driver = {
	.probe = qcom_usb_hs_phy_probe,
	.driver = {
		.name = "qcom_usb_hs_phy",
		.of_match_table = qcom_usb_hs_phy_match,
	},
};
module_ulpi_driver(qcom_usb_hs_phy_driver);

MODULE_DESCRIPTION("Qualcomm USB HS phy");
MODULE_LICENSE("GPL v2");
