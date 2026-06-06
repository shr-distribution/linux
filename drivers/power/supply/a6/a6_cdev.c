// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - char device + firmware update
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Exposes the per-device misc nodes (/dev/a6_N + /dev/a6_N_diag) used by
 * userspace tooling: A2A datagram read/write, the ioctl that flashes /
 * verifies the A6 MSP430 firmware over SBW, and the pmem extract device used
 * to pull out the last-known controller state image.
 */

#include <linux/cdev.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/hex.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "a6_internal.h"
#include "high_level_funcs.h"

enum a6_pgm_thread_op {
	A6_PROGAM_AND_VERIFY_FW = 1,
	A6_VERIFY_FW
};

struct a6_pgm_thread_params {
	struct a6_sbw_interface *sbw_ops;
	uint32_t buffer_p;
	int32_t ret_code;
	enum a6_pgm_thread_op op;
	struct completion a6_flash_thread_nice;
	struct completion a6_flash_thread_exit;
};

static int a6_pgm_thread_fn(void *param)
{
	int32_t ret_val;

	struct a6_pgm_thread_params *t_p = (struct a6_pgm_thread_params *)param;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: buffer_p val: 0x%x\n", __func__, t_p->buffer_p);

	// wait for parent re-nice completion...
	ret_val = wait_for_completion_interruptible(&t_p->a6_flash_thread_nice);
	if (-ERESTARTSYS == ret_val) {
		pr_err("A6: waiting for parent re-nice completion interrupted.\n");
		t_p->ret_code = -EINTR;
		goto err0;
	}

	if (t_p->op == A6_PROGAM_AND_VERIFY_FW) {
		ret_val = program_device_sbw(t_p->sbw_ops, t_p->buffer_p);
	} else {
		ret_val = verify_device_sbw(t_p->sbw_ops, t_p->buffer_p);
		// ignore error returns for the moment as the verification fn
		// does not differentiate between code and r/w data sections.
		// also, the fw section allocation changes fairly dynamically
		// between fw drops and we don't yet support parameterizing the
		// section map for the verification code.
		ret_val = 0;
	}
	if (ret_val)
		t_p->ret_code = -EINVAL;

	// signal thread completion
	complete(&t_p->a6_flash_thread_exit);

err0:
	return 0;
}

static long a6_ioctl(struct file *file, unsigned int cmd, unsigned long args)
{
	int32_t rc = 0;
	struct a6_device_state *state = file->private_data;
	void *usr_ptr   = (void *)args;
	//uint32_t usr_bytes = _IOC_SIZE(cmd);
	//uint32_t usr_val ;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: cmd: %d\n", __func__, _IOC_NR(cmd));

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 3 seconds: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq,
						      !test_bit(DEVICE_BUSY_BIT, state->flags), 3*HZ);
		if (!rc) {
			pr_err("%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			pr_err("%s: mutex_lock interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	// bootload request: set flag in critsec
	if (A6_IOCTL_SET_FW_DATA == cmd || A6_IOCTL_VERIFY_FW_DATA == cmd) {
		int32_t ret_val;

		ret_val = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
		ASSERT(!ret_val);
	}

	// we're done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	switch (cmd) {
	case A6_IOCTL_SET_FW_DATA:
	case A6_IOCTL_VERIFY_FW_DATA:
	{
			uint8_t *buffer_p = NULL;
			uint32_t payload_size = 0;
			struct a6_sbw_interface *sbw_ops =
					(struct a6_sbw_interface *)state->plat_data->sbw_ops;
			uint8_t *a6_fw_buffer;
			struct task_struct *pgm_worker_task;
			struct a6_pgm_thread_params t_params;

			// reset force-wake state to always force wake on first i2c txn
			// after flashing/verification
			timer_delete(&state->a6_force_wake_timer);
			mutex_lock(&state->a6_force_wake_mutex);
			if (test_bit(CAP_PERIODIC_WAKE, state->flags))
				clear_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
			mutex_unlock(&state->a6_force_wake_mutex);

			// copy payload ptr from ioctl parameter
			if (copy_from_user(&buffer_p, usr_ptr, sizeof(buffer_p))) {
				rc = -EFAULT;
				break;
			}
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "buffer_p val: 0x%p\n", buffer_p);

			// copy payload size from ioctl parameter
			if (copy_from_user(&payload_size, (uint8_t *)usr_ptr + sizeof(uint32_t), sizeof(payload_size))) {
				rc = -EFAULT;
				break;
			}

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "payload_size: %d\n", payload_size);

			// alloc kernel buffer for payload
			a6_fw_buffer = kmalloc(payload_size, GFP_KERNEL);
			if (!a6_fw_buffer) {
				rc = -ENOMEM;
				break;
			}
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "fw buffer ptr_val: 0x%p\n", a6_fw_buffer);

			// copy-in payload data
			if (copy_from_user(a6_fw_buffer, buffer_p, payload_size)) {
				rc = -EFAULT;
				break;
			}

			pr_err("A6: Starting flashing sequence.\n");
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6: parent task nice value: %d\n",
				   task_nice(current));

			// initialize worker task params
			init_completion(&t_params.a6_flash_thread_nice);
			init_completion(&t_params.a6_flash_thread_exit);
			t_params.sbw_ops = sbw_ops;
			t_params.buffer_p = (uint32_t)a6_fw_buffer;
			t_params.ret_code = 0;
			t_params.op = (cmd == A6_IOCTL_SET_FW_DATA) ?
					A6_PROGAM_AND_VERIFY_FW : A6_VERIFY_FW;

			// create worker task...
			pgm_worker_task = kthread_run(a6_pgm_thread_fn, &t_params,
						      "a6_pgm_%s", state->plat_data->dev_name);
			if (IS_ERR(pgm_worker_task)) {
				pr_err("A6: failed to create pgm worker task.\n");
				rc = PTR_ERR(pgm_worker_task);
				pgm_worker_task = NULL;
				break;
			}
			// re-nice worker task
			//set_user_nice(pgm_worker_task, 10);
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6: pgm worker task nice value: %d\n",
				   task_nice(pgm_worker_task));

			/* hold cpu at max freq before signaling worker thread to commence sbw */
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_TICKLE
			CPUFREQ_HOLD_CHECK(&state->cpufreq_hold_flag);
#endif
			// signal re-nice completion...
			complete(&t_params.a6_flash_thread_nice);

			// wait for worker task-end completion...
			rc = wait_for_completion_interruptible(&t_params.a6_flash_thread_exit);
			/* unhold cpu */
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_TICKLE
			CPUFREQ_UNHOLD_CHECK(&state->cpufreq_hold_flag);
#endif
			kfree(a6_fw_buffer);

			if (-ERESTARTSYS == rc) {
				pr_err("A6: waiting for pgm worker start interrupted.\n");
				break;
			}

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "A6: pgm worker task exit code: %d\n",
				   t_params.ret_code);
			if (t_params.ret_code) {
				pr_err("A6: Failed to completed flashing sequence. ret: %d\n",
				       t_params.ret_code);
				// propagate error to caller...
				rc = t_params.ret_code;
			} else {
				pr_err("A6: Completed flashing sequence.\n");
				// wait for the A6 to boot...
				msleep(3000);
				// - flashing fw forces a device power-up sequence: re-init state
				// - if init fails: treat as fw flash failure by returning rc
				rc = a6_init_state(state->i2c_dev);
				if (rc < 0)
					pr_err("%s: failed to initialize, err: %d\n", A6_DRIVER, rc);
			}

			mutex_lock(&state->dev_mutex);
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
			mutex_unlock(&state->dev_mutex);
	}
	break;

	default:
	{
		rc = -EINVAL;
	}
	break;
	}


//Done:
	mutex_lock(&state->dev_mutex);
	if (rc) {
		if (test_bit(BOOTLOAD_ACTIVE_BIT, state->flags))
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	} else if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	}
	mutex_unlock(&state->dev_mutex);

	wake_up_interruptible(&state->dev_busyq);

	return rc;
}

static int a6_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct a6_device_state *state;

	/* get device from miscdevice */
	state = container_of(mdev, struct a6_device_state, mdev);

	/* attach private data */
	file->private_data = state;

	return 0;
}

static int a6_close(struct inode *inode, struct file *file)
{
	(void)file;

	return 0;
}

#define A2A_DGRAM_PREAMBLE (0x5AC35AC3)
struct a2a_dgram_hdr {
	uint32_t preamble;
	uint16_t len;
	uint16_t cksum;
};

static ssize_t a6_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
# define A2A_RX_MISS_THRESHOLD (300)

	ssize_t rc = 0;
	struct a6_device_state *state;
	struct a6_register_desc *reg_desc_comm_status, *reg_desc_comm_rxtx;
	uint8_t vals[id_size];
	int32_t miss_count, rd_count;
	//struct a2a_dgram_hdr hdr;
	uint32_t start_time;
	uint8_t elt_val = 0, prev_byte;


	start_time = jiffies;
	/* input validations */
	if (!count)
		return -EINVAL;
	else if (count > A2A_RD_BUFF_SIZE)
		return -EFBIG;

	/* get state */
	state = file->private_data;

	// acquire critsec
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted.\n", __func__);
		return -ERESTARTSYS;
	}

	/* not connected? exit */
	if (!test_bit(A2A_CONNECTED, state->flags)) {
		mutex_unlock(&state->dev_mutex);
		pr_err("%s: no a2a connection detected.\n", __func__);
		return -EINVAL;
	}

	// busy?
	if (test_and_set_bit(READ_ACTIVE_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);
		pr_err("%s: re-entrant call disallowed.\n", __func__);
		return -EINVAL;
	}

	mutex_unlock(&state->dev_mutex);

	/* init a2a read ptr */
	state->a2a_rp = state->a2a_rd_buf;

	rd_count = 0;
	miss_count = 0;
	prev_byte = 0;
	do {
		/* read comm status */
		reg_desc_comm_status = &a6_register_desc_arr[63];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc_comm_status->id,
				     reg_desc_comm_status->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error reading reg: %s, id: 0x%x\n",
			       __func__, reg_desc_comm_status->debug_name, reg_desc_comm_status->id[0]);
			goto err0;
		}

		if (!(vals[0] & TS2_I2C_COMM_STATUS_RX_FULL)) {
			if (++miss_count > A2A_RX_MISS_THRESHOLD) {
				if (test_bit(A2A_CONNECTED, state->flags))
					continue;
				else
					break;
			}
			continue;
		}

		/* read rx data */
		memset(vals, 0, sizeof(vals));
		reg_desc_comm_rxtx = &a6_register_desc_arr[64];
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc_comm_rxtx->id,
				      reg_desc_comm_rxtx->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			       __func__, reg_desc_comm_rxtx->debug_name, reg_desc_comm_rxtx->id[0]);
			rc = -EIO;
			goto err0;
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: read byte: %d successfully.\n",
			   __func__, rd_count);

		if (a6_t2s_dup_correct) {
			if ((prev_byte & 0x80) ^ (vals[0] & 0x80)) {
				prev_byte = vals[0];
				if (vals[0] & 0x80) {
					elt_val = _convert_hex_char_to_decimal(vals[0] & 0x7f) << 4;
					continue;
				} else {
					elt_val |= (_convert_hex_char_to_decimal(vals[0]) & 0x0f);
				}
			} else {
				pr_err("%s: t2s duplicate detected; char: 0x%02x.\n",
				       __func__, vals[0]);
				continue;
			}
		} else {
			elt_val = vals[0];
		}

		*state->a2a_rp = elt_val;
		state->a2a_rp++;
		rd_count++;
	} while (rd_count < count);

	if (!rd_count) {
		pr_err("%s: rx failed; A2A connection terminated.\n", __func__);
		rc = -EINVAL;
	} else {
		long diff_time;

		if (copy_to_user(buf, state->a2a_rd_buf, rd_count)) {
			rc = -EFAULT;
			goto err0;
		}
		rc = rd_count;
		diff_time = (long)jiffies - (long)start_time;
		pr_err("%s: elapsed time: %ld ms; count: %u\n",
		       __func__, diff_time * 1000/HZ, rd_count);
	}


err0:
	clear_bit(READ_ACTIVE_BIT, state->flags);
	return rc;
}

static ssize_t a6_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
# define A2A_TX_MISS_THRESHOLD (300)

	ssize_t rc = 0;
	struct a6_device_state *state;
	struct a6_register_desc *reg_desc_comm_status, *reg_desc_comm_rxtx;
	uint8_t vals[id_size];
	int32_t miss_count, wr_count;
	//struct a2a_dgram_hdr hdr;
	uint32_t start_time;
	uint8_t elt_buf[2], *elt_bufp = NULL;
	int elt_bufsize;


	start_time = jiffies;
	/* input validations */
	if (!count)
		return -EINVAL;
	else if (count > A2A_WR_BUFF_SIZE)
		return -EFBIG;

	/* get state */
	state = file->private_data;

	// acquire critsec
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted.\n", __func__);
		return -ERESTARTSYS;
	}

	/* not connected? exit */
	if (!test_bit(A2A_CONNECTED, state->flags)) {
		mutex_unlock(&state->dev_mutex);
		pr_err("%s: no a2a connection detected.\n", __func__);
		return -EINVAL;
	}

	// busy?
	if (test_and_set_bit(WRITE_ACTIVE_BIT, state->flags)) {
		mutex_unlock(&state->dev_mutex);
		pr_err("%s: re-entrant call disallowed.\n", __func__);
		return -EINVAL;
	}

	mutex_unlock(&state->dev_mutex);

	/* copy to kernel buffer */
	//hdr.preamble = A2A_DGRAM_PREAMBLE;
	//hdr.len = count;
	//hdr.cksum = 0;
	///* header */
	//if (copy_from_user(state->a2a_wr_buf, &hdr, sizeof(hdr))) {
	//	rc = -EFAULT;
	//	mutex_unlock(&state->dev_mutex);
	//	goto err0;
	//}
	/* data */
	if (copy_from_user(state->a2a_wr_buf/*+sizeof(hdr)*/, buf, count)) {
		rc = -EFAULT;
		goto err0;
	}
	/* init a2a write ptr */
	state->a2a_wp = state->a2a_wr_buf;

	wr_count = 0;
	miss_count = 0;
	elt_bufsize = 0;
	do {
		/* read comm status */
		reg_desc_comm_status = &a6_register_desc_arr[63];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc_comm_status->id,
				      reg_desc_comm_status->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error reading reg: %s, id: 0x%x\n",
			__func__, reg_desc_comm_status->debug_name, reg_desc_comm_status->id[0]);
			goto err0;
		}

		if (!(vals[0] & TS2_I2C_COMM_STATUS_TX_EMPTY)) {
			if (++miss_count > A2A_TX_MISS_THRESHOLD) {
				if (test_bit(A2A_CONNECTED, state->flags))
					continue;
				else
					break;
			}
			continue;
		}

		/* write tx data */
		reg_desc_comm_rxtx = &a6_register_desc_arr[64];
		if (!elt_bufsize) {
			if (a6_t2s_dup_correct) {
				elt_bufsize = 2;
				/*
				 * The A6 protocol encodes one register value as
				 * two ASCII-hex digits with the sync bit (0x80)
				 * OR'd into the first digit. Use hex_byte_pack()
				 * instead of sprintf("%02x") because elt_buf is
				 * exactly 2 bytes -- sprintf would write a third
				 * NUL terminator past the end of the buffer.
				 */
				hex_byte_pack(elt_buf, *state->a2a_wp);
				elt_buf[0] |= 0x80;
			} else {
				elt_bufsize = 1;
				elt_buf[0] = *state->a2a_wp;
			}
			elt_bufp = elt_buf;
		}
		rc = a6_i2c_write_reg(state->i2c_dev, reg_desc_comm_rxtx->id,
				       reg_desc_comm_rxtx->num_ids, elt_bufp);
		if (rc < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc_comm_rxtx->debug_name, reg_desc_comm_rxtx->id[0]);
			rc = -EIO;
			goto err0;
		}
		elt_bufp++;
		elt_bufsize--;

		if (!elt_bufsize) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: written byte: %d successfully.\n",
				   __func__, wr_count);
			state->a2a_wp++;
			wr_count++;
		}
	} while (wr_count < count);

	if (!wr_count) {
		pr_err("%s: tx failed; A2A connection terminated.\n",
		       __func__);
		rc = -EIO;
		goto err0;
	} else {
		long diff_time;

		rc = wr_count;
		diff_time = (long)jiffies - (long)start_time;
		pr_err("%s: elapsed time: %ld ms; count: %u\n", __func__,
		       diff_time * 1000/HZ, wr_count);
	}


err0:
	clear_bit(WRITE_ACTIVE_BIT, state->flags);
	return rc;
}

static unsigned int a6_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int  mask = 0;

	return mask;
}

const struct file_operations a6_fops = {
	.owner   = THIS_MODULE,
	.read    = a6_read,
	.write    = a6_write,
	.poll    = a6_poll,
	.unlocked_ioctl = a6_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = a6_ioctl,
#endif
	.open    = a6_open,
	.release = a6_close,
};

static int a6_pmem_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct a6_device_state *state;

	/* get device from miscdevice */
	state = container_of(mdev, struct a6_device_state, pmem_mdev);

	/* Allow only read. */
	if ((file->f_mode & (FMODE_READ|FMODE_WRITE)) != FMODE_READ)
		return -EINVAL;

	/* check if it is in use */
	if (test_and_set_bit(IS_OPENED, state->flags))
		return -EBUSY;

	/* attach private data */
	file->private_data = state;
	return 0;
}

static int a6_pmem_close(struct inode *inode, struct file *file)
{
	struct a6_device_state *state = (struct  a6_device_state *) file->private_data;

	/* mark it as unused */
	clear_bit(IS_OPENED, state->flags);
	return 0;
}

static ssize_t a6_pmem_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	if (!count)
		return -EINVAL;

	return ttf_image_read(buf, count, ppos);
}

const struct file_operations a6_pmem_fops = {
	.owner   = THIS_MODULE,
	.read    = a6_pmem_read,
	.open    = a6_pmem_open,
	.release = a6_pmem_close,
};
