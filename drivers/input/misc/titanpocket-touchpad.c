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
 *
 * The surface is the keyboard: every key press pushes the sensor in. The
 * stock key handler therefore puts the sensor to sleep on each key press and
 * wakes it once all keys are up (agold_touchpad_resume_suspend(), called
 * from aw9523_key_work(); stock synaptics_rmi4_suspend()/_resume()). Left
 * scanning through key presses, the controller's baseline drifts: it then
 * reports a contact that never lifts, and every touch as a full-pressure
 * blob pulled into a corner, until it is reset. The same entry point is
 * provided here, on top of the mainline RMI4 suspend/resume.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/rmi.h>

#include "../rmi4/rmi_bus.h"

#define TP_I2C_BUS	6
#define TP_I2C_ADDR	0x20
#define TP_RESET_GPIO	9	/* SoC pin number, driven via agold_gpio_set() */

/* drivers/pinctrl/mediatek/pinctrl-paris.c */
extern void agold_gpio_set(unsigned gpio, int value);

int agold_touchpad_resume_suspend(int suspend);

struct titan_tp {
	struct i2c_client *client;
	struct rmi_device_platform_data pdata;
};

/* The one touchpad, for the keyboard driver's calls; NULL while unbound. */
static struct titan_tp *titan_tp;
static DEFINE_MUTEX(titan_tp_lock);

static int titan_tp_match(struct device *dev, void *data)
{
	return rmi_is_physical_device(dev) &&
	       to_rmi_device(dev)->xport->dev == data;
}

/* What the stock driver reports on suspend: every contact lifted. A finger
 * that was on the surface when a key went down would otherwise stay down
 * until the sensor wakes. The interrupt is off by now, so the 2D sensor
 * code cannot report at the same time. */
static void titan_tp_release_all(struct input_dev *input)
{
	int i;

	if (!input || !input->mt)
		return;
	for (i = 0; i < input->mt->num_slots; i++) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
	}
	input_mt_sync_frame(input);
	input_sync(input);
}

/**
 * agold_touchpad_resume_suspend() - put the sensor to sleep, or wake it
 * @suspend: non-zero to sleep (a key went down), 0 to wake (all keys up)
 *
 * Called from the keyboard driver's work item, so it may sleep. Sleeping an
 * already sleeping sensor, or waking an awake one, is harmless: the RMI4
 * core only toggles its interrupt when it is in the other state, and the
 * F01 sleep-mode write is the same either way.
 *
 * Return: 0, or a negative errno from the RMI4 core.
 */
int agold_touchpad_resume_suspend(int suspend)
{
	struct rmi_driver_data *data;
	struct rmi_device *rmi_dev;
	struct device *dev;
	int ret = 0;

	mutex_lock(&titan_tp_lock);
	if (!titan_tp || !titan_tp->client) {
		mutex_unlock(&titan_tp_lock);
		return 0;
	}
	dev = bus_find_device(&rmi_bus_type, NULL, &titan_tp->client->dev,
			      titan_tp_match);	/* takes a reference */
	mutex_unlock(&titan_tp_lock);
	if (!dev)
		return 0;	/* rmi4_i2c not bound (yet): nothing to do */

	/* Hold off an unbind (a manual rebind) while the device is used. */
	device_lock(dev);
	rmi_dev = to_rmi_device(dev);
	data = dev_get_drvdata(dev);
	if (dev->driver && data) {
		if (suspend) {
			ret = rmi_driver_suspend(rmi_dev, false);
			titan_tp_release_all(data->input);
		} else {
			ret = rmi_driver_resume(rmi_dev, false);
		}
	}
	device_unlock(dev);
	put_device(dev);
	return ret;
}
EXPORT_SYMBOL_GPL(agold_touchpad_resume_suspend);

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
	mutex_lock(&titan_tp_lock);
	titan_tp = tp;
	mutex_unlock(&titan_tp_lock);
	dev_info(&pdev->dev, "touchpad at %d-%04x, irq %d\n",
		 TP_I2C_BUS, TP_I2C_ADDR, irq);
	return 0;
}

static int titan_tp_remove(struct platform_device *pdev)
{
	struct titan_tp *tp = platform_get_drvdata(pdev);

	mutex_lock(&titan_tp_lock);
	titan_tp = NULL;
	mutex_unlock(&titan_tp_lock);
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
