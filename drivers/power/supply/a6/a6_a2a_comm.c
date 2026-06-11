// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 legacy A2A communication interface (Tap-to-Share)
 *
 * Downstream-only auxiliary-bus driver that re-exposes the legacy
 * /dev/a6_N character device and /sys/class/misc/a6_N/regs/ sysfs
 * attributes that the original webOS `tap2shared` userspace consumes
 * for Palm/HP Tap-to-Share between devices.
 *
 * This driver intentionally lives outside the upstream A6 battery
 * driver: modern LuneOS / mainline userspace consumes battery state
 * via the standard power_supply class and has no use for any of the
 * raw register access surface this module creates. The upstream
 * a6-battery driver carries no userspace ABI beyond power_supply.
 *
 * Binds to the auxiliary device "a6_battery.a2a-comm.N" created by
 * the base a6-battery i2c driver. All register I/O is performed
 * through the read_regs / write_regs callbacks in struct a6_aux_dev,
 * which take the base driver's dev_mutex internally so this module
 * never races the power_supply class against itself.
 *
 * Copyright (C) 2008-2011 Palm, Inc. (original legacy a6 driver)
 * Copyright (C) 2010 Hewlett-Packard Co.
 * Copyright (C) 2026 Herman van Hazendonk <github.com@herrie.org>
 */

#include <linux/atomic.h>
#include <linux/auxiliary_bus.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "palm-a6-aux.h"

/*
 * A6 register IDs. Duplicated here (not pulled from a6_internal.h)
 * because that header is private to the base driver and this module
 * must stay loosely coupled. The addresses are firmware-stable: they
 * are part of the A6 MSP430 protocol contract.
 */
#define TS2_I2C_INT_MASK_3		0x0003
#define TS2_I2C_INT_STATUS_3		0x0007
#define TS2_I2C_ID			0x0700
#define TS2_I2C_FLAGS_2			0x0703
#define TS2_I2C_COMM_STATUS		0x0200
#define TS2_I2C_COMM_STATUS_RX_FULL	0x02
#define TS2_I2C_COMM_STATUS_TX_EMPTY	0x01
#define TS2_I2C_COMM_TXDATA_RXDATA	0x0203
#define TS2_I2C_WAKEUP_PERIOD		0x07c1
#define TS2_I2C_COMMAND			0x1000

/* page 0x04 - accessory data (local) */
#define TS2_I2C_ENUM_ACCE_0		0x0428
/* page 0x05 - accessory data (remote) */
#define TS2_I2C_ENUM_REMOTE_ACCE_0	0x0528

#define A6_ACC_DATA_COUNT		16

/* TX/RX bounce buffer size for /dev/a6_N. Matches the legacy driver. */
#define A2A_RW_BUF_SIZE			(4 * 1024)

/*
 * Per-byte poll cap for /dev/a6_N read/write. The A6 firmware updates
 * COMM_STATUS opportunistically on every i2c transaction; the legacy
 * driver used a 300-iteration miss count with no sleep. Sleep 1ms
 * between iterations and cap total wait per byte at 5 seconds (5000
 * iterations) which is generous given a real T2S session moves <2 KB
 * total.
 */
#define A2A_PER_BYTE_TIMEOUT_MS		5000

struct a6_a2a {
	struct a6_aux_dev *aux;
	struct miscdevice mdev;
	char mdev_name[8];	/* "a6_N" */

	/* one-opener-at-a-time enforcement for /dev/a6_N */
	atomic_t opened;

	/* bounce buffers; allocated once at probe */
	u8 *rx_buf;
	u8 *tx_buf;

	bool sysfs_linked;
};

/* ---------- low-level helpers over the aux ops table ----------------- */

static int a6_a2a_read(struct a6_a2a *priv, u16 id, u8 *out)
{
	return priv->aux->read_regs(priv->aux->client, &id, 1, out);
}

static int a6_a2a_write(struct a6_a2a *priv, u16 id, u8 val)
{
	return priv->aux->write_regs(priv->aux->client, &id, 1, &val);
}

static int a6_a2a_read_range(struct a6_a2a *priv, u16 base_id, u32 num,
			     u8 *out)
{
	u16 ids[A6_ACC_DATA_COUNT];
	u32 i;

	if (num > ARRAY_SIZE(ids))
		return -EINVAL;
	for (i = 0; i < num; i++)
		ids[i] = base_id + i;
	return priv->aux->read_regs(priv->aux->client, ids, num, out);
}

static int a6_a2a_write_range(struct a6_a2a *priv, u16 base_id, u32 num,
			      const u8 *in)
{
	u16 ids[A6_ACC_DATA_COUNT];
	u32 i;

	if (num > ARRAY_SIZE(ids))
		return -EINVAL;
	for (i = 0; i < num; i++)
		ids[i] = base_id + i;
	return priv->aux->write_regs(priv->aux->client, ids, num, in);
}

/* ---------- shared sysfs parse helper ------------------------------- */

/*
 * Parse a whitespace-delimited list of decimal/hex byte values from a
 * sysfs write. Returns the count parsed, or negative errno. Each value
 * must fit in u8.
 */
static int a6_a2a_parse_bytes(const char *buf, size_t count, u8 *out,
			      size_t max)
{
	char *copy, *cursor, *tok;
	size_t n = 0;
	int ret = 0;
	u8 val;

	copy = kstrndup(buf, count, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	cursor = copy;
	while ((tok = strsep(&cursor, " \t\n\r\v\f")) != NULL) {
		if (*tok == '\0')
			continue;
		if (n >= max) {
			ret = -EINVAL;
			goto out;
		}
		ret = kstrtou8(tok, 0, &val);
		if (ret < 0)
			goto out;
		out[n++] = val;
	}

	ret = n ? (int)n : -EINVAL;
out:
	kfree(copy);
	return ret;
}

/* ---------- single-byte sysfs attributes ---------------------------- */

struct a6_single_reg_attr {
	struct device_attribute dattr;
	u16 reg;
};

#define to_single_reg_attr(_a)	container_of(_a, struct a6_single_reg_attr, dattr)

static ssize_t a6_single_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_single_reg_attr *sattr = to_single_reg_attr(attr);
	u8 val = 0;
	int ret;

	ret = a6_a2a_read(priv, sattr->reg, &val);
	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", (int)(s8)val);
}

static ssize_t a6_single_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_single_reg_attr *sattr = to_single_reg_attr(attr);
	u8 val;
	int ret;

	ret = a6_a2a_parse_bytes(buf, count, &val, 1);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EINVAL;
	ret = a6_a2a_write(priv, sattr->reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/*
 * `command` is the write side of TS2_I2C_COMMAND (0x1000). The legacy
 * userspace path used it to send glow / glow-off pulses to the puck;
 * a malicious local writer could in principle bootloader-reset the
 * controller through TS2_I2C_COMMAND_RESET_HOST. Gate behind
 * CAP_SYS_ADMIN to match the spirit of the original 0220 mode.
 */
static ssize_t a6_command_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	return a6_single_store(dev, attr, buf, count);
}

/* ---------- multi-byte accessory-data combo attributes -------------- */

struct a6_combo_attr {
	struct device_attribute dattr;
	u16 base_reg;	/* contiguous reg page start */
	bool writable;
};

#define to_combo_attr(_a)	container_of(_a, struct a6_combo_attr, dattr)

static ssize_t a6_combo_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_combo_attr *cattr = to_combo_attr(attr);
	u8 vals[A6_ACC_DATA_COUNT];
	ssize_t len = 0;
	int ret, i;

	ret = a6_a2a_read_range(priv, cattr->base_reg, A6_ACC_DATA_COUNT,
				vals);
	if (ret < 0)
		return ret;

	for (i = 0; i < A6_ACC_DATA_COUNT; i++)
		len += sysfs_emit_at(buf, len, "%d ", (int)(s8)vals[i]);
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t a6_combo_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_combo_attr *cattr = to_combo_attr(attr);
	u8 vals[A6_ACC_DATA_COUNT];
	int ret;

	if (!cattr->writable)
		return -EPERM;

	ret = a6_a2a_parse_bytes(buf, count, vals, A6_ACC_DATA_COUNT);
	if (ret < 0)
		return ret;
	if (ret != A6_ACC_DATA_COUNT)
		return -EINVAL;

	ret = a6_a2a_write_range(priv, cattr->base_reg, A6_ACC_DATA_COUNT,
				 vals);
	if (ret < 0)
		return ret;
	return count;
}

/* ---------- attribute table ----------------------------------------- */

/* single-byte read+write (mode 0640) */
#define A6_SINGLE_RW_ATTR(_name, _reg)				\
static struct a6_single_reg_attr a6_attr_##_name = {		\
	.dattr = __ATTR(_name, 0640, a6_single_show,		\
			a6_single_store),			\
	.reg = (_reg),						\
}

/* single-byte read-only (mode 0440) */
#define A6_SINGLE_RO_ATTR(_name, _reg)				\
static struct a6_single_reg_attr a6_attr_##_name = {		\
	.dattr = __ATTR(_name, 0440, a6_single_show, NULL),	\
	.reg = (_reg),						\
}

A6_SINGLE_RO_ATTR(id,			TS2_I2C_ID);
A6_SINGLE_RW_ATTR(int_mask3,		TS2_I2C_INT_MASK_3);
A6_SINGLE_RW_ATTR(int_status3,		TS2_I2C_INT_STATUS_3);
A6_SINGLE_RO_ATTR(charger,		TS2_I2C_FLAGS_2);
A6_SINGLE_RW_ATTR(comm_txdata_rx_data,	TS2_I2C_COMM_TXDATA_RXDATA);
A6_SINGLE_RO_ATTR(get_comm_status,	TS2_I2C_COMM_STATUS);
A6_SINGLE_RW_ATTR(periodic_wake_bit_params, TS2_I2C_WAKEUP_PERIOD);

/* write-only, CAP_SYS_ADMIN-gated command register */
static struct a6_single_reg_attr a6_attr_command = {
	.dattr = __ATTR(command, 0220, NULL, a6_command_store),
	.reg = TS2_I2C_COMMAND,
};

/* accessory data combo: 16 contiguous bytes, R/W (local) or RO (remote) */
static struct a6_combo_attr a6_attr_acc_data_combo = {
	.dattr = __ATTR(acc_data_combo, 0640, a6_combo_show, a6_combo_store),
	.base_reg = TS2_I2C_ENUM_ACCE_0,
	.writable = true,
};

static struct a6_combo_attr a6_attr_remote_acc_data_combo = {
	.dattr = __ATTR(remote_acc_data_combo, 0440,
			a6_combo_show, NULL),
	.base_reg = TS2_I2C_ENUM_REMOTE_ACCE_0,
	.writable = false,
};

/* Individual acc_data_0..15 (R/W) and remote_acc_data_0..15 (RO) */
static struct a6_single_reg_attr a6_acc_data_attrs[A6_ACC_DATA_COUNT];
static struct a6_single_reg_attr a6_remote_acc_data_attrs[A6_ACC_DATA_COUNT];

/*
 * a6_init_indexed_attrs() - fill in the per-slot acc_data attribute
 * descriptors at module load. The 32 attribute structs are static
 * (one set per module-instance is fine because the names are
 * identical across miscdevices: "a6_0/regs/acc_data_0" and
 * "a6_1/regs/acc_data_0" both reference the same kobj_attribute
 * struct, which is allowed because device_attribute is read-only at
 * the bus core).
 */
static void a6_init_indexed_attrs(void)
{
	static const char * const acc_names[] = {
		"acc_data_0",  "acc_data_1",  "acc_data_2",  "acc_data_3",
		"acc_data_4",  "acc_data_5",  "acc_data_6",  "acc_data_7",
		"acc_data_8",  "acc_data_9",  "acc_data_10", "acc_data_11",
		"acc_data_12", "acc_data_13", "acc_data_14", "acc_data_15",
	};
	static const char * const remote_names[] = {
		"remote_acc_data_0",  "remote_acc_data_1",
		"remote_acc_data_2",  "remote_acc_data_3",
		"remote_acc_data_4",  "remote_acc_data_5",
		"remote_acc_data_6",  "remote_acc_data_7",
		"remote_acc_data_8",  "remote_acc_data_9",
		"remote_acc_data_10", "remote_acc_data_11",
		"remote_acc_data_12", "remote_acc_data_13",
		"remote_acc_data_14", "remote_acc_data_15",
	};
	int i;

	for (i = 0; i < A6_ACC_DATA_COUNT; i++) {
		a6_acc_data_attrs[i].dattr.attr.name = acc_names[i];
		a6_acc_data_attrs[i].dattr.attr.mode = 0640;
		a6_acc_data_attrs[i].dattr.show = a6_single_show;
		a6_acc_data_attrs[i].dattr.store = a6_single_store;
		a6_acc_data_attrs[i].reg = TS2_I2C_ENUM_ACCE_0 + i;

		a6_remote_acc_data_attrs[i].dattr.attr.name = remote_names[i];
		a6_remote_acc_data_attrs[i].dattr.attr.mode = 0440;
		a6_remote_acc_data_attrs[i].dattr.show = a6_single_show;
		a6_remote_acc_data_attrs[i].dattr.store = NULL;
		a6_remote_acc_data_attrs[i].reg =
			TS2_I2C_ENUM_REMOTE_ACCE_0 + i;
	}
}

/*
 * Build the attribute group dynamically because struct attribute_group
 * needs an array of `struct attribute *` and we have a mix of single
 * and combo attrs plus 32 indexed slots. Total = 8 singles + 2 combos
 * + 16 acc + 16 remote_acc + NULL terminator = 43.
 */
#define A6_REGS_ATTR_COUNT	(8 + 2 + A6_ACC_DATA_COUNT * 2)

static struct attribute *a6_regs_attrs[A6_REGS_ATTR_COUNT + 1];

static void a6_build_regs_attrs(void)
{
	int i, n = 0;

	a6_regs_attrs[n++] = &a6_attr_id.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_int_mask3.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_int_status3.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_charger.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_comm_txdata_rx_data.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_get_comm_status.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_periodic_wake_bit_params.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_command.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_acc_data_combo.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_remote_acc_data_combo.dattr.attr;
	for (i = 0; i < A6_ACC_DATA_COUNT; i++)
		a6_regs_attrs[n++] = &a6_acc_data_attrs[i].dattr.attr;
	for (i = 0; i < A6_ACC_DATA_COUNT; i++)
		a6_regs_attrs[n++] = &a6_remote_acc_data_attrs[i].dattr.attr;
	a6_regs_attrs[n] = NULL;
}

static const struct attribute_group a6_regs_group = {
	.name = "regs",
	.attrs = a6_regs_attrs,
};

/* ---------- /dev/a6_N character device ------------------------------ */

static int a6_a2a_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);

	/*
	 * Allow only one opener at a time. The legacy driver tracked
	 * READ_ACTIVE/WRITE_ACTIVE flags per-call; serialising at the
	 * file-descriptor layer is both simpler and matches how
	 * tap2shared actually uses /dev/a6_N (single dedicated worker
	 * thread per device).
	 */
	if (atomic_xchg(&priv->opened, 1))
		return -EBUSY;

	file->private_data = priv;
	return 0;
}

static int a6_a2a_release(struct inode *inode, struct file *file)
{
	struct a6_a2a *priv = file->private_data;

	atomic_set(&priv->opened, 0);
	return 0;
}

/*
 * Wait for a COMM_STATUS bit (RX_FULL when reading, TX_EMPTY when
 * writing) to be asserted, polling every millisecond. Returns 0 on
 * success, -ETIMEDOUT after A2A_PER_BYTE_TIMEOUT_MS, or the i2c error
 * on bus failure. -ERESTARTSYS if interrupted.
 */
static int a6_a2a_wait_status(struct a6_a2a *priv, u8 want_bit)
{
	unsigned long deadline = jiffies +
		msecs_to_jiffies(A2A_PER_BYTE_TIMEOUT_MS);
	u8 status;
	int ret;

	for (;;) {
		ret = a6_a2a_read(priv, TS2_I2C_COMM_STATUS, &status);
		if (ret < 0)
			return ret;
		if (status & want_bit)
			return 0;
		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;
		if (msleep_interruptible(1))
			return -ERESTARTSYS;
	}
}

static ssize_t a6_a2a_dev_read(struct file *file, char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct a6_a2a *priv = file->private_data;
	size_t done = 0;
	int ret;

	if (!count)
		return 0;
	if (count > A2A_RW_BUF_SIZE)
		count = A2A_RW_BUF_SIZE;

	while (done < count) {
		u8 byte;

		ret = a6_a2a_wait_status(priv, TS2_I2C_COMM_STATUS_RX_FULL);
		if (ret < 0) {
			if (done)
				break;
			return ret;
		}

		ret = a6_a2a_read(priv, TS2_I2C_COMM_TXDATA_RXDATA, &byte);
		if (ret < 0) {
			if (done)
				break;
			return ret;
		}
		priv->rx_buf[done++] = byte;
	}

	if (copy_to_user(ubuf, priv->rx_buf, done))
		return -EFAULT;
	return done;
}

static ssize_t a6_a2a_dev_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct a6_a2a *priv = file->private_data;
	size_t done = 0;
	int ret;

	if (!count)
		return 0;
	if (count > A2A_RW_BUF_SIZE)
		count = A2A_RW_BUF_SIZE;

	if (copy_from_user(priv->tx_buf, ubuf, count))
		return -EFAULT;

	while (done < count) {
		ret = a6_a2a_wait_status(priv, TS2_I2C_COMM_STATUS_TX_EMPTY);
		if (ret < 0) {
			if (done)
				break;
			return ret;
		}

		ret = a6_a2a_write(priv, TS2_I2C_COMM_TXDATA_RXDATA,
				   priv->tx_buf[done]);
		if (ret < 0) {
			if (done)
				break;
			return ret;
		}
		done++;
	}

	return done;
}

static const struct file_operations a6_a2a_fops = {
	.owner		= THIS_MODULE,
	.open		= a6_a2a_open,
	.release	= a6_a2a_release,
	.read		= a6_a2a_dev_read,
	.write		= a6_a2a_dev_write,
};

/* ---------- auxiliary_driver bind/unbind --------------------------- */

static void a6_a2a_remove_sysfs(void *data)
{
	struct a6_a2a *priv = data;

	if (priv->sysfs_linked) {
		sysfs_remove_group(&priv->mdev.this_device->kobj,
				   &a6_regs_group);
		priv->sysfs_linked = false;
	}
}

static void a6_a2a_misc_deregister(void *data)
{
	struct a6_a2a *priv = data;

	misc_deregister(&priv->mdev);
}

static int a6_a2a_probe(struct auxiliary_device *adev,
			const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct a6_aux_dev *aux = to_a6_aux_dev(adev);
	struct a6_a2a *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->aux = aux;
	atomic_set(&priv->opened, 0);

	priv->rx_buf = devm_kzalloc(dev, A2A_RW_BUF_SIZE, GFP_KERNEL);
	if (!priv->rx_buf)
		return -ENOMEM;
	priv->tx_buf = devm_kzalloc(dev, A2A_RW_BUF_SIZE, GFP_KERNEL);
	if (!priv->tx_buf)
		return -ENOMEM;

	snprintf(priv->mdev_name, sizeof(priv->mdev_name), "a6_%d",
		 aux->device_index);

	priv->mdev.minor = MISC_DYNAMIC_MINOR;
	priv->mdev.name = priv->mdev_name;
	priv->mdev.fops = &a6_a2a_fops;
	/*
	 * Parenting the misc device to the auxiliary device gives the
	 * /sys/class/misc/a6_N/device symlink a meaningful target and
	 * keeps the lifetime tied to the base i2c_client (which is the
	 * grandparent via the auxiliary bus).
	 */
	priv->mdev.parent = dev;

	ret = misc_register(&priv->mdev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register %s\n",
				     priv->mdev_name);
	dev_set_drvdata(priv->mdev.this_device, &priv->mdev);

	ret = devm_add_action_or_reset(dev, a6_a2a_misc_deregister, priv);
	if (ret)
		return ret;

	ret = sysfs_create_group(&priv->mdev.this_device->kobj,
				 &a6_regs_group);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to create regs sysfs group\n");
	priv->sysfs_linked = true;

	ret = devm_add_action_or_reset(dev, a6_a2a_remove_sysfs, priv);
	if (ret)
		return ret;

	auxiliary_set_drvdata(adev, priv);
	dev_info(dev, "Palm A6 A2A comm interface ready on /dev/%s\n",
		 priv->mdev_name);
	return 0;
}

static void a6_a2a_remove(struct auxiliary_device *adev)
{
	/* devm teardown handles sysfs_remove_group and misc_deregister */
}

static const struct auxiliary_device_id a6_a2a_id_table[] = {
	{ .name = "a6_battery.a2a-comm" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, a6_a2a_id_table);

static struct auxiliary_driver a6_a2a_driver = {
	.name		= "a2a-comm",
	.probe		= a6_a2a_probe,
	.remove		= a6_a2a_remove,
	.id_table	= a6_a2a_id_table,
};

static int __init a6_a2a_init(void)
{
	a6_init_indexed_attrs();
	a6_build_regs_attrs();
	return auxiliary_driver_register(&a6_a2a_driver);
}
module_init(a6_a2a_init);

static void __exit a6_a2a_exit(void)
{
	auxiliary_driver_unregister(&a6_a2a_driver);
}
module_exit(a6_a2a_exit);

MODULE_DESCRIPTION("Palm A6 legacy A2A comm interface for Tap-to-Share");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
