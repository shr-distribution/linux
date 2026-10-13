// SPDX-License-Identifier: GPL-2.0
/*
 * Board glue for the Unihertz Titan Pocket keyboard touchpad.
 *
 * The touch surface on the keyboard is a Synaptics RMI4 controller
 * (F01/F12/F34/F54/F55) at i2c6 address 0x20. The stock dtbo only describes
 * it loosely (a "mediatek,cap_tkpd" node at 0x21 and a "mediatek,touch_pad"
 * node carrying the interrupt), and the stock Agold driver hard-codes the
 * reset line. This driver reproduces the stock power-up, then hands the chip
 * to the mainline RMI4 driver as a touchpad (indirect pointing device).
 *
 * Recovered from stock synaptics_rmi4_probe(): GPIO 9 is reset (active low),
 * pulsed high 200 ms, low 20 ms, high 200 ms; interrupt is EINT 2, level low.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/rmi.h>

#define TP_I2C_BUS	6
#define TP_I2C_ADDR	0x20
#define TP_RESET_GPIO	9	/* SoC pin number, driven via agold_gpio_set() */

/* drivers/pinctrl/mediatek/pinctrl-paris.c */
extern void agold_gpio_set(unsigned gpio, int value);

struct titan_tp {
	struct i2c_client *client;
	struct rmi_device_platform_data pdata;
};

static void titan_tp_reset(void)
{
	agold_gpio_set(TP_RESET_GPIO, 1);
	msleep(200);
	agold_gpio_set(TP_RESET_GPIO, 0);
	msleep(20);
	agold_gpio_set(TP_RESET_GPIO, 1);
	msleep(200);
}

static int titan_tp_probe(struct platform_device *pdev)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("rmi4_i2c", TP_I2C_ADDR),
	};
	struct i2c_adapter *adap;
	struct titan_tp *tp;
	int irq;

	tp = devm_kzalloc(&pdev->dev, sizeof(*tp), GFP_KERNEL);
	if (!tp)
		return -ENOMEM;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no interrupt in touch_pad node\n");
		return -EINVAL;
	}

	adap = i2c_get_adapter(TP_I2C_BUS);
	if (!adap)
		return -EPROBE_DEFER;

	titan_tp_reset();

	/* Indirect pointer, so userspace treats it as a touchpad */
	tp->pdata.sensor_pdata.sensor_type = rmi_sensor_touchpad;
	info.irq = irq;
	info.platform_data = &tp->pdata;

	tp->client = i2c_new_device(adap, &info);
	i2c_put_adapter(adap);
	if (!tp->client) {
		dev_err(&pdev->dev, "failed to create rmi4_i2c device\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, tp);
	dev_info(&pdev->dev, "touchpad at %d-%04x, irq %d\n",
		 TP_I2C_BUS, TP_I2C_ADDR, irq);
	return 0;
}

static int titan_tp_remove(struct platform_device *pdev)
{
	struct titan_tp *tp = platform_get_drvdata(pdev);

	i2c_unregister_device(tp->client);
	return 0;
}

static const struct of_device_id titan_tp_of_match[] = {
	{ .compatible = "mediatek,touch_pad" },	/* stock dtbo node */
	{ }
};
MODULE_DEVICE_TABLE(of, titan_tp_of_match);

static struct platform_driver titan_tp_driver = {
	.driver = {
		.name = "titanpocket-touchpad",
		.of_match_table = titan_tp_of_match,
		/* the reset pulse sleeps ~420 ms; don't hold up boot for it */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = titan_tp_probe,
	.remove = titan_tp_remove,
};
module_platform_driver(titan_tp_driver);

MODULE_DESCRIPTION("Unihertz Titan Pocket keyboard touchpad glue (RMI4)");
MODULE_LICENSE("GPL v2");
