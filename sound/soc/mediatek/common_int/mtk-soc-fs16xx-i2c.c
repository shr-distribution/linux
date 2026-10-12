// SPDX-License-Identifier: GPL-2.0
/*
 * /dev/smartpa_i2c for a FourSemi FS16xx smart amplifier on the
 * "mediatek,speaker_amp" node.
 *
 * The amplifier is programmed entirely from user space: the vendor audio
 * HAL's libfs1860.so opens /dev/smartpa_i2c, selects the chip with the
 * i2c-dev I2C_SLAVE ioctl, and then reads and writes registers with plain
 * read() and write(), loading its presets from /vendor/etc/smartpa_params.
 * Without the node it logs "device not initialized" on every speaker on and
 * the speaker stays silent.
 *
 * The node is restricted to the bus the amplifier sits on and to the four
 * addresses an FS16xx can strap to, and it only appears once the chip has
 * answered with a FourSemi device ID.
 */

#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mtk-soc-fs16xx-i2c.h"

#define FS16XX_REG_ID		0x03
#define FS16XX_ADDR_FIRST	0x34
#define FS16XX_ADDR_LAST	0x37
/* The same cap i2c-dev puts on one read() or write(). */
#define FS16XX_XFER_MAX		8192
#define FS16XX_ID_TRIES		3

static struct i2c_client *fs16xx_client;

static bool fs16xx_known_id(u8 id)
{
	switch (id) {
	case 0x03:	/* FS1601S */
	case 0x05:	/* FS1603 */
	case 0x06:	/* FS1818 */
	case 0x07:	/* FS1860 */
		return true;
	default:
		return false;
	}
}

static int fs16xx_misc_open(struct inode *inode, struct file *file)
{
	const struct i2c_client *client = READ_ONCE(fs16xx_client);

	if (!client)
		return -ENODEV;

	/* Until I2C_SLAVE says otherwise, talk to the chip the node is for. */
	file->private_data = (void *)(uintptr_t)client->addr;
	return 0;
}

static u16 fs16xx_file_addr(const struct file *file)
{
	return (u16)(uintptr_t)file->private_data;
}

static ssize_t fs16xx_misc_read(struct file *file, char __user *buf,
				size_t count, loff_t *offset)
{
	struct i2c_client *client = READ_ONCE(fs16xx_client);
	struct i2c_msg msg;
	u8 *tmp;
	int ret;

	if (!client)
		return -ENODEV;
	if (!count)
		return 0;
	if (count > FS16XX_XFER_MAX)
		count = FS16XX_XFER_MAX;

	tmp = kmalloc(count, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	msg.addr = fs16xx_file_addr(file);
	msg.flags = I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : (int)count;
	else if (ret >= 0)
		ret = -EIO;

	kfree(tmp);
	return ret;
}

static ssize_t fs16xx_misc_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct i2c_client *client = READ_ONCE(fs16xx_client);
	struct i2c_msg msg;
	u8 *tmp;
	int ret;

	if (!client)
		return -ENODEV;
	if (!count)
		return 0;
	if (count > FS16XX_XFER_MAX)
		count = FS16XX_XFER_MAX;

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	msg.addr = fs16xx_file_addr(file);
	msg.flags = 0;
	msg.len = count;
	msg.buf = tmp;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		ret = count;
	else if (ret >= 0)
		ret = -EIO;

	kfree(tmp);
	return ret;
}

static long fs16xx_misc_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		if (arg < FS16XX_ADDR_FIRST || arg > FS16XX_ADDR_LAST)
			return -EINVAL;
		file->private_data = (void *)(uintptr_t)arg;
		return 0;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations fs16xx_misc_fops = {
	.owner = THIS_MODULE,
	.open = fs16xx_misc_open,
	.read = fs16xx_misc_read,
	.write = fs16xx_misc_write,
	.unlocked_ioctl = fs16xx_misc_ioctl,
	/* Every command takes its argument by value. */
	.compat_ioctl = fs16xx_misc_ioctl,
	.llseek = no_llseek,
};

static struct miscdevice fs16xx_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smartpa_i2c",
	.fops = &fs16xx_misc_fops,
};

int mtk_fs16xx_i2c_attach(struct i2c_client *client)
{
	int i, ret = -ENODEV;
	u8 id;

	if (fs16xx_client)
		return -EBUSY;
	if (client->addr < FS16XX_ADDR_FIRST || client->addr > FS16XX_ADDR_LAST)
		return -ENODEV;

	for (i = 0; i < FS16XX_ID_TRIES; i++) {
		ret = i2c_smbus_read_word_swapped(client, FS16XX_REG_ID);
		if (ret >= 0)
			break;
		usleep_range(5000, 6000);
	}
	if (ret < 0) {
		dev_info(&client->dev, "no FourSemi amplifier: ID read failed, %d\n",
			 ret);
		return ret;
	}

	id = (ret >> 8) & 0xff;
	if (!fs16xx_known_id(id)) {
		dev_info(&client->dev, "no FourSemi amplifier: ID register 0x%04x\n",
			 ret);
		return -ENODEV;
	}

	fs16xx_client = client;
	ret = misc_register(&fs16xx_misc);
	if (ret) {
		fs16xx_client = NULL;
		dev_err(&client->dev, "cannot register /dev/%s, %d\n",
			fs16xx_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "FourSemi FS16xx, device ID 0x%02x, /dev/%s\n",
		 id, fs16xx_misc.name);
	return 0;
}

void mtk_fs16xx_i2c_detach(const struct i2c_client *client)
{
	if (!fs16xx_client || fs16xx_client != client)
		return;

	misc_deregister(&fs16xx_misc);
	WRITE_ONCE(fs16xx_client, NULL);
}
