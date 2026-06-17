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
#define TS2_I2C_FLAGS_0			0x0701
#define TS2_I2C_FLAGS_2			0x0703
#define TS2_I2C_COMM_STATUS		0x0200
#define TS2_I2C_COMM_STATUS_RX_FULL	0x02
#define TS2_I2C_COMM_STATUS_TX_EMPTY	0x01
#define TS2_I2C_COMM_TXDATA_RXDATA	0x0203
#define TS2_I2C_WAKEUP_PERIOD		0x07c1
#define TS2_I2C_V_OFFSET		0x07c0
#define TS2_I2C_COMMAND			0x1000

/* battery diagnostic registers consumed by powerd (legacy sysfs ABI) */
#define TS2_I2C_BAT_STATUS		0x0100
#define TS2_I2C_BAT_COULOMB_MSB		0x010b
#define TS2_I2C_BAT_COULOMB_LSB		0x010c
#define TS2_I2C_BAT_AS			0x010d
#define TS2_I2C_BAT_SACR_MSB		0x0115
#define TS2_I2C_BAT_SACR_LSB		0x0116
#define TS2_I2C_BAT_ASL			0x0117
#define TS2_I2C_BAT_FAC_MSB		0x0118
#define TS2_I2C_BAT_FAC_LSB		0x0119
#define TS2_I2C_BAT_RSNSP		0x0112
#define TS2_I2C_BAT_TEMP_LOW_MSB	0x0180
#define TS2_I2C_BAT_TEMP_LOW_LSB	0x0181
#define TS2_I2C_BAT_TEMP_HIGH_MSB	0x0182
#define TS2_I2C_BAT_TEMP_HIGH_LSB	0x0183
#define TS2_I2C_BAT_VOLT_LOW_MSB	0x0184
#define TS2_I2C_BAT_VOLT_LOW_LSB	0x0185
#define TS2_I2C_BAT_RARC_CRIT		0x0186

/* page 0x04 - accessory data (local) */
#define TS2_I2C_ENUM_ACCE_0		0x0428
/* page 0x05 - accessory data (remote) + remote enumeration */
#define TS2_I2C_ENUM_REMOTE_STRUCT_VER	0x0500
#define TS2_I2C_ENUM_REMOTE_SERNO_7	0x0507
#define TS2_I2C_ENUM_REMOTE_SERNO_6	0x0508
#define TS2_I2C_ENUM_REMOTE_SERNO_5	0x0509
#define TS2_I2C_ENUM_REMOTE_SERNO_4	0x050a
#define TS2_I2C_ENUM_REMOTE_SERNO_3	0x050b
#define TS2_I2C_ENUM_REMOTE_SERNO_2	0x050c
#define TS2_I2C_ENUM_REMOTE_SERNO_1	0x050d
#define TS2_I2C_ENUM_REMOTE_SERNO_0	0x050e
#define TS2_I2C_ENUM_REMOTE_VNODE_MAX_HI 0x0515
#define TS2_I2C_ENUM_REMOTE_VNODE_MAX_LO 0x0516
#define TS2_I2C_ENUM_REMOTE_INODE_MAX_HI 0x0519
#define TS2_I2C_ENUM_REMOTE_INODE_MAX_LO 0x051a
#define TS2_I2C_ENUM_REMOTE_POWER_MAX	0x051d
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

	/*
	 * Format: "0x%02x " per byte. Userspace tap2shared parses this
	 * via uint64_from_bytestream() mode 4 (ASCII hex with optional
	 * "0x" prefix). v1 originally emitted "%d " (signed decimal),
	 * which the mode-4 parser silently misinterprets as hex (e.g.
	 * "18" -> 0x18 = 24, not 18), corrupting every TLV burst and
	 * surfacing as "bad phase 1 data" aborts in tap_connected.
	 * The legacy 2.6.35 webOS driver emitted the hex form; restore
	 * compatibility.
	 */
	for (i = 0; i < A6_ACC_DATA_COUNT; i++)
		len += sysfs_emit_at(buf, len, "0x%02x ", vals[i]);
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

/* ---------- battery-diagnostic attributes consumed by powerd -------- */

/*
 * Mirror the legacy /sys/class/misc/a6_N/regs/<name> read/write format
 * for the diagnostic registers powerd reads (battery health, factory
 * data, charging thresholds). Each helper formats the chip output
 * exactly the same way the legacy driver's per-attribute format_*()
 * callbacks did, so the userspace parser in powerd Just Works.
 *
 * Two register-layout quirks come from the A6 firmware contract:
 *
 *   - 16-bit values live in MSB:LSB on the wire but the kernel
 *     register-ID list is given as {LSB_addr, MSB_addr}; the base
 *     driver swaps so the resulting byte buffer is {low, high}.
 *
 *   - Coulomb-style readings scale by 1/Rsense (mOhm). Rsense lives
 *     in TS2_I2C_BAT_RSNSP and is read once at probe and cached in
 *     priv->cached_rsense to avoid a chip transaction per show().
 */

static int a6_a2a_read_pair(struct a6_a2a *priv, u16 reg_lsb, u16 reg_msb,
			    u8 out[2])
{
	u16 ids[2] = { reg_lsb, reg_msb };

	return priv->aux->read_regs(priv->aux->client, ids, 2, out);
}

static int a6_a2a_write_pair(struct a6_a2a *priv, u16 reg_lsb, u16 reg_msb,
			     const u8 in[2])
{
	u16 ids[2] = { reg_lsb, reg_msb };

	return priv->aux->write_regs(priv->aux->client, ids, 2, in);
}

/* Read N specific (possibly non-contiguous) register IDs. */
static int a6_a2a_read_ids(struct a6_a2a *priv, const u16 *ids, u32 num,
			   u8 *out)
{
	return priv->aux->read_regs(priv->aux->client, (u16 *)ids, num, out);
}

/* A pair register pair (LSB, MSB) reads two consecutive entries of a
 * dual-byte chip register. The .reg field of a6_pair_reg_attr holds
 * the LSB address; the MSB is assumed at LSB - 1 (legacy MSB:LSB
 * neighbour layout). */
struct a6_pair_reg_attr {
	struct device_attribute dattr;
	u16 reg_lsb;
	u16 reg_msb;
};

#define to_pair_reg_attr(_a)	container_of(_a, struct a6_pair_reg_attr, dattr)

/* Multi-byte attribute (3+ regs). Stores up to 8 explicit register IDs. */
struct a6_multi_reg_attr {
	struct device_attribute dattr;
	u16 ids[8];
	u8 num_ids;
};

#define to_multi_reg_attr(_a)	container_of(_a, struct a6_multi_reg_attr, dattr)

/* ----- status (TS2_I2C_BAT_STATUS, 1 byte, bit-decoded) ----- */

static const char * const a6_bat_status_bits[8] = {
	NULL,
	"power-on-reset",
	"undervoltage",
	NULL,
	"learn",
	"standby-empty",
	"active-empty",
	"charge-termination",
};

static ssize_t a6_status_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_single_reg_attr *sattr = to_single_reg_attr(attr);
	ssize_t len = 0;
	u8 val = 0;
	int ret, i;

	ret = a6_a2a_read(priv, sattr->reg, &val);
	if (ret < 0)
		return ret;
	for (i = 0; i < 8; i++) {
		if ((val & BIT(i)) && a6_bat_status_bits[i])
			len += sysfs_emit_at(buf, len, "%s\n",
					     a6_bat_status_bits[i]);
	}
	return len;
}

/* ----- unsigned-byte show ("%u\n" — legacy format_raw_unsigned) ----- */

static ssize_t a6_unsigned_show(struct device *dev,
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
	return sysfs_emit(buf, "%u\n", (unsigned int)val);
}

static ssize_t a6_unsigned_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_single_reg_attr *sattr = to_single_reg_attr(attr);
	u8 val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret < 0)
		return ret;
	ret = a6_a2a_write(priv, sattr->reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/* ----- temperature pair (MSB-only signed degrees C) ----- */

static ssize_t a6_temp_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_pair_reg_attr *pattr = to_pair_reg_attr(attr);
	u8 raw[2];
	int ret;

	ret = a6_a2a_read_pair(priv, pattr->reg_lsb, pattr->reg_msb, raw);
	if (ret < 0)
		return ret;
	/* raw[0] = LSB (fractional, ignored); raw[1] = MSB (signed deg C). */
	return sysfs_emit(buf, "%d\n", (int)(s8)raw[1]);
}

static ssize_t a6_temp_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_pair_reg_attr *pattr = to_pair_reg_attr(attr);
	long val;
	u8 raw[2];
	int ret;

	ret = kstrtol(buf, 0, &val);
	if (ret < 0)
		return ret;
	if (val < S8_MIN || val > S8_MAX)
		return -ERANGE;
	raw[0] = 0;			/* fractional zero */
	raw[1] = (u8)(s8)val;		/* integer deg C */
	ret = a6_a2a_write_pair(priv, pattr->reg_lsb, pattr->reg_msb, raw);
	if (ret < 0)
		return ret;
	return count;
}

/* ----- voltage pair (11-bit signed, unit = 4880 uV) ----- */

static ssize_t a6_voltage_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_pair_reg_attr *pattr = to_pair_reg_attr(attr);
	u8 raw[2];
	s16 v;
	int ret;

	ret = a6_a2a_read_pair(priv, pattr->reg_lsb, pattr->reg_msb, raw);
	if (ret < 0)
		return ret;
	v = (s16)((raw[1] << 8) | raw[0]);
	return sysfs_emit(buf, "%d\n", ((int)v >> 5) * 4880);
}

static ssize_t a6_voltage_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_pair_reg_attr *pattr = to_pair_reg_attr(attr);
	long uv;
	s16 v;
	u8 raw[2];
	int ret;

	ret = kstrtol(buf, 0, &uv);
	if (ret < 0)
		return ret;
	v = (s16)(uv / 4880) << 5;	/* inverse of show */
	raw[0] = v & 0xff;
	raw[1] = (v >> 8) & 0xff;
	ret = a6_a2a_write_pair(priv, pattr->reg_lsb, pattr->reg_msb, raw);
	if (ret < 0)
		return ret;
	return count;
}

/* ----- raw coulomb (16-bit signed * 6250 / Rsense, mOhm) ----- */

static ssize_t a6_rawcoulomb_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_pair_reg_attr *pattr = to_pair_reg_attr(attr);
	u8 raw[2], rsense = 0;
	s16 v;
	int ret, uah;

	ret = a6_a2a_read_pair(priv, pattr->reg_lsb, pattr->reg_msb, raw);
	if (ret < 0)
		return ret;
	ret = a6_a2a_read(priv, TS2_I2C_BAT_RSNSP, &rsense);
	if (ret < 0 || rsense == 0)
		return ret < 0 ? ret : -EIO;

	v = (s16)((raw[1] << 8) | raw[0]);
	uah = ((int)v * 6250) / rsense;
	return sysfs_emit(buf, "%d.%03d\n",
			  uah / 1000,
			  (uah >= 0 ? uah : -uah) % 1000);
}

/* ----- multi-byte hex serial number (6 or 8 bytes) ----- */

static ssize_t a6_serno_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_multi_reg_attr *mattr = to_multi_reg_attr(attr);
	u8 raw[8];
	ssize_t len = 0;
	int ret, i;

	ret = a6_a2a_read_ids(priv, mattr->ids, mattr->num_ids, raw);
	if (ret < 0)
		return ret;
	for (i = 0; i < mattr->num_ids; i++)
		len += sysfs_emit_at(buf, len, "%02x", raw[i]);
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

/* ----- max power available (vnode_max * inode_max * power / 2560) ----- */

static ssize_t a6_maxpower_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct miscdevice *mdev = dev_get_drvdata(dev);
	struct a6_a2a *priv = container_of(mdev, struct a6_a2a, mdev);
	struct a6_multi_reg_attr *mattr = to_multi_reg_attr(attr);
	u8 raw[5];
	u32 vnode, inode, power;
	int ret;

	if (mattr->num_ids != 5)
		return -EINVAL;
	ret = a6_a2a_read_ids(priv, mattr->ids, mattr->num_ids, raw);
	if (ret < 0)
		return ret;
	vnode = (raw[1] << 8) | raw[0];
	inode = (raw[3] << 8) | raw[2];
	power = raw[4];
	return sysfs_emit(buf, "%u\n", (vnode * inode * power) / 2560);
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

/* powerd diagnostic attrs -- mirror legacy /sys/class/misc/a6_N/regs/ ABI */

/* status: bit-decoded battery status */
static struct a6_single_reg_attr a6_attr_status = {
	.dattr = __ATTR(status, 0440, a6_status_show, NULL),
	.reg = TS2_I2C_BAT_STATUS,
};

/* single-byte unsigned ("%u\n") -- legacy format_raw_unsigned / format_v_offset */
#define A6_UNSIGNED_RO(_name, _reg)					\
static struct a6_single_reg_attr a6_attr_##_name = {			\
	.dattr = __ATTR(_name, 0440, a6_unsigned_show, NULL),		\
	.reg = (_reg),							\
}

#define A6_UNSIGNED_RW(_name, _reg)					\
static struct a6_single_reg_attr a6_attr_##_name = {			\
	.dattr = __ATTR(_name, 0640, a6_unsigned_show,			\
			a6_unsigned_store),				\
	.reg = (_reg),							\
}

A6_UNSIGNED_RO(getasl,			TS2_I2C_BAT_ASL);
A6_UNSIGNED_RO(getrawas,		TS2_I2C_BAT_AS);
A6_UNSIGNED_RO(remote_struct_ver,	TS2_I2C_ENUM_REMOTE_STRUCT_VER);
A6_UNSIGNED_RW(puck_priority,		TS2_I2C_FLAGS_0);
A6_UNSIGNED_RW(rarc_crit,		TS2_I2C_BAT_RARC_CRIT);
A6_UNSIGNED_RW(v_offset,		TS2_I2C_V_OFFSET);

/* paired 16-bit registers -- temperature, voltage thresholds, coulomb */
#define A6_PAIR_ATTR(_name, _show, _store, _mode, _lsb, _msb)		\
static struct a6_pair_reg_attr a6_attr_##_name = {			\
	.dattr = __ATTR(_name, (_mode), (_show), (_store)),		\
	.reg_lsb = (_lsb),						\
	.reg_msb = (_msb),						\
}

A6_PAIR_ATTR(temp_high,	  a6_temp_show,	     a6_temp_store,    0640,
	     TS2_I2C_BAT_TEMP_HIGH_LSB, TS2_I2C_BAT_TEMP_HIGH_MSB);
A6_PAIR_ATTR(temp_low,	  a6_temp_show,	     a6_temp_store,    0640,
	     TS2_I2C_BAT_TEMP_LOW_LSB,	TS2_I2C_BAT_TEMP_LOW_MSB);
A6_PAIR_ATTR(volt_low,	  a6_voltage_show,   a6_voltage_store, 0640,
	     TS2_I2C_BAT_VOLT_LOW_LSB,	TS2_I2C_BAT_VOLT_LOW_MSB);
A6_PAIR_ATTR(getrawcoulomb, a6_rawcoulomb_show, NULL,	       0440,
	     TS2_I2C_BAT_COULOMB_LSB,	TS2_I2C_BAT_COULOMB_MSB);
A6_PAIR_ATTR(getsacr,	  a6_rawcoulomb_show, NULL,	       0440,
	     TS2_I2C_BAT_SACR_LSB,	TS2_I2C_BAT_SACR_MSB);
A6_PAIR_ATTR(getfac,	  a6_rawcoulomb_show, NULL,	       0440,
	     TS2_I2C_BAT_FAC_LSB,	TS2_I2C_BAT_FAC_MSB);

/* multi-register: remote serial-numbers and max-power-available */
static struct a6_multi_reg_attr a6_attr_remote_serno_v1 = {
	.dattr = __ATTR(remote_serno_v1, 0440, a6_serno_show, NULL),
	.ids = {
		TS2_I2C_ENUM_REMOTE_SERNO_0, TS2_I2C_ENUM_REMOTE_SERNO_1,
		TS2_I2C_ENUM_REMOTE_SERNO_2, TS2_I2C_ENUM_REMOTE_SERNO_3,
		TS2_I2C_ENUM_REMOTE_SERNO_4, TS2_I2C_ENUM_REMOTE_SERNO_5,
	},
	.num_ids = 6,
};

static struct a6_multi_reg_attr a6_attr_remote_serno_v2 = {
	.dattr = __ATTR(remote_serno_v2, 0440, a6_serno_show, NULL),
	.ids = {
		TS2_I2C_ENUM_REMOTE_SERNO_0, TS2_I2C_ENUM_REMOTE_SERNO_1,
		TS2_I2C_ENUM_REMOTE_SERNO_2, TS2_I2C_ENUM_REMOTE_SERNO_3,
		TS2_I2C_ENUM_REMOTE_SERNO_4, TS2_I2C_ENUM_REMOTE_SERNO_5,
		TS2_I2C_ENUM_REMOTE_SERNO_6, TS2_I2C_ENUM_REMOTE_SERNO_7,
	},
	.num_ids = 8,
};

static struct a6_multi_reg_attr a6_attr_getmaxpoweravail = {
	.dattr = __ATTR(getmaxpoweravail, 0440, a6_maxpower_show, NULL),
	.ids = {
		TS2_I2C_ENUM_REMOTE_VNODE_MAX_LO,
		TS2_I2C_ENUM_REMOTE_VNODE_MAX_HI,
		TS2_I2C_ENUM_REMOTE_INODE_MAX_LO,
		TS2_I2C_ENUM_REMOTE_INODE_MAX_HI,
		TS2_I2C_ENUM_REMOTE_POWER_MAX,
	},
	.num_ids = 5,
};

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
 * and combo attrs plus 32 indexed slots. Total = 8 base singles +
 * 2 combos + 16 powerd diagnostic attrs + 16 acc + 16 remote_acc
 * + NULL terminator = 59.
 */
#define A6_POWERD_ATTR_COUNT	16
#define A6_REGS_ATTR_COUNT	(8 + 2 + A6_POWERD_ATTR_COUNT + A6_ACC_DATA_COUNT * 2)

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

	/* powerd diagnostic surface (16 attrs) */
	a6_regs_attrs[n++] = &a6_attr_status.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_getasl.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_getrawas.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_remote_struct_ver.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_puck_priority.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_rarc_crit.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_v_offset.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_temp_high.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_temp_low.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_volt_low.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_getrawcoulomb.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_getsacr.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_getfac.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_remote_serno_v1.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_remote_serno_v2.dattr.attr;
	a6_regs_attrs[n++] = &a6_attr_getmaxpoweravail.dattr.attr;
	/* (a6_diag and validate_cksum -- the remaining 2 of powerd's 19
	 * reads -- need the SBW (Spy-Bi-Wire) JTAG driver to be ported
	 * before we can implement them. Deferred.)
	 */

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
	 * tap2shared opens /dev/a6_N concurrently from two worker threads:
	 * receive_TLV_worker_thread (O_RDONLY) and send_TLV_worker_thread
	 * (O_WRONLY) run side-by-side on every tap. The original single-open
	 * guard returned EBUSY for the second opener and broke phase-2 of
	 * tap-to-share end-to-end. Chip-level I/O is already serialised by
	 * the base driver's dev_mutex through a6_a2a_read/write, so the
	 * file-descriptor-layer lock isn't load-bearing -- drop it.
	 */
	file->private_data = priv;
	return 0;
}

static int a6_a2a_release(struct inode *inode, struct file *file)
{
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
