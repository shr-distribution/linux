/*
 * Copyright (c) 2015-2018 TrustKernel Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <asm/barrier.h>

#include <linux/tee_core.h>
#include <linux/tee_kernel_lowlevel_api.h>

#include <arm_common/teesmc_st.h>
#include <arm_common/teesmc.h>

#include "tee_procfs.h"
#include "tee_core_priv.h"
#include <version.h>

#define TEE_LOG_TIMEOUT_MS	(500)
#define ENABLE_PRINTK_TEE_LOG

struct proc_dir_entry *tee_proc_dir;

struct proc_dir_entry *tee_proc_log_file;
struct proc_dir_entry *tee_proc_drv_version;
struct proc_dir_entry *tee_proc_tee_version;
struct proc_dir_entry *tee_proc_teed_version;

union tee_log_ctrl {
	struct {
		unsigned int tee_buf_addr;
		unsigned int tee_buf_size;
		unsigned int tee_write_pos;
		unsigned int tee_read_pos;

		unsigned int tee_buf_unread_size;

		unsigned int tee_irq_count;
		unsigned int tee_reader_alive;

		unsigned int tee_write_seq;
		unsigned int tee_read_seq;
	} info;
	unsigned char data[TEE_LOG_CTL_BUF_SIZE];
};

struct klog {
	/* shm for log ctl */
	union tee_log_ctrl *log_ctl;

	/* tee ringbuffer for log */
	char *tee_rb;
	/* tee ring buffer length */
	uint32_t tee_rb_len;

	/*
	 * whether write_pos
	 * has restarted from
	 * one
	 */
	bool overwrite;
	/*
	 * served as notifier
	 * when there's new
	 * log
	 */
	wait_queue_head_t wq;

	struct task_struct *ts;

	/*
	 * irq for tee to notify
	 * nsdrv to fetch log
	 */
	int notify_irq;
};

struct ulog {
	/*
	 * local read sequence,
	 * only updated by user
	 * apps open this proc
	 * entry
	 */
	uint32_t rseq;

	/*
	 * buffer containing
	 * temporary str to
	 * pass to CA
	 */
	const char *tmpbuf;

	/*
	 * ptr to global
	 * klog
	 */
	struct klog *klog;

};

static struct klog klog;

static inline bool rb_overrun(uint32_t rseq,
							uint32_t wseq,
							uint32_t rb_len)
{
	return wseq - rseq > rb_len;
}

static size_t ulog_rb(struct klog *klog,
					struct ulog *ulog,
					char __user *buf,
					size_t count)
{
	size_t len = 0;
	uint32_t wseq;

	union tee_log_ctrl *log_ctl = klog->log_ctl;

	static const char ulog_flag_intr[] =
		"------ interrupted\n";

	wseq = READ_ONCE(log_ctl->info.tee_write_seq);

	while ((len != count) && ((ulog->rseq != wseq) ||
		(ulog->tmpbuf && ulog->tmpbuf[0] != '\0'))) {
		size_t copy_len;
		unsigned long n;

		if (ulog->tmpbuf == NULL) {
			if (rb_overrun(ulog->rseq, wseq,
						klog->tee_rb_len)) {
				ulog->tmpbuf = ulog_flag_intr;
			}
		} else if (ulog->tmpbuf[0] == '\0') {
			if (klog->overwrite ||
				wseq >= klog->tee_rb_len) {
				ulog->rseq = wseq - klog->tee_rb_len;
			} else {
				ulog->rseq = 0;
			}
			ulog->tmpbuf = NULL;
		}

		if (ulog->tmpbuf) {
			size_t tmpbuf_len;
			const char *tmpbuf;

			tmpbuf = ulog->tmpbuf;
			tmpbuf_len = strlen(tmpbuf);

			copy_len = (uint32_t) min(tmpbuf_len, count - len);
			n = copy_to_user(&buf[len], tmpbuf, copy_len);

			if (copy_len == n) {
				pr_warn("tkcoredrv: failed to copy flag to user");
				return len;
			}

			ulog->tmpbuf = &tmpbuf[copy_len - n];
			len += (copy_len - n);
		} else {
			copy_len = min((uint32_t) (count - len),
					min(klog->tee_rb_len -
						ulog->rseq % klog->tee_rb_len,
						wseq - ulog->rseq));
			n = copy_to_user(&buf[len],
				&klog->tee_rb[ulog->rseq % klog->tee_rb_len],
				copy_len);
			if (copy_len == n) {
				pr_warn("tkcoredrv: failed to copy klog to user\n");
				return len;
			}

			ulog->rseq += (copy_len - n);
			len += (copy_len - n);
		}

		wseq = READ_ONCE(log_ctl->info.tee_write_seq);
	}

	return len;
}

static ssize_t tee_log_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	ssize_t ret;
	struct ulog *ulog;
	struct klog *klog;

	if (file == NULL || buf == NULL || pos == NULL)
		return -EINVAL;

	ulog = (struct ulog *) file->private_data;
	if (ulog == NULL) {
		pr_warn("tkcoredrv: file not open correctly\n");
		return -EINVAL;
	}

	klog = ulog->klog;

	if (file->f_flags & O_NONBLOCK) {
		/*
		 * currently nonblock file is
		 * not supported, since
		 * we might need to enter
		 * wait queue
		 */
		return -EAGAIN;
	}

	if (ulog->tmpbuf == NULL) {
		long r;

		do {
			r = wait_event_interruptible_timeout(klog->wq,
				ulog->rseq !=
				READ_ONCE(klog->log_ctl->info.tee_write_seq),
				msecs_to_jiffies(TEE_LOG_TIMEOUT_MS));

		} while (!r);

		if (r < 0) {
			/*
			 * woke up due to signal, e.g.
			 * calling program terminated
			 * by CTRL-C
			 */
			return -EINTR;
		}
	}

	ret = ulog_rb(klog, ulog, buf, count);
	*pos += ret;

	return ret;
}

int tee_log_open(struct inode *inode, struct file *file)
{
	int ret;
	struct ulog *ulog;

	static const char ulog_flag_begin[] =
		"------ beginning of tee\n";

	ulog = kmalloc(
			sizeof(struct ulog),
			GFP_KERNEL);
	if (ulog == NULL)
		return -ENOMEM;

	if (klog.log_ctl == NULL)
		return -EIO;

	ulog->tmpbuf = ulog_flag_begin;
	ulog->klog = &klog;

	ret = nonseekable_open(inode, file);
	if (unlikely(ret)) {
		kfree(ulog);

		pr_warn("tkcoredrv: open file failed with %d\n", ret);
		return ret;
	}

	file->private_data = (void *) ulog;

	return 0;
}

int tee_log_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static const struct proc_ops log_tee_ops = {
	.proc_read = tee_log_read,
	.proc_open = tee_log_open,
	.proc_release = tee_log_release,
};

static ssize_t copy_to_user_str(char __user *buf, ssize_t count, loff_t *pos,
				const char *version)
{
	ssize_t r;
	size_t cnt;
	loff_t __pos;

	__pos = *pos;
	if (__pos > strlen(version) + 1) {
		pr_warn("invalid pos: %lld len: %zu\n",
			__pos, strlen(version));
		return -EINVAL;
	}

	cnt = count < strlen(version) + 1 - __pos ?
		count : strlen(version) + 1 - __pos;

	r = copy_to_user(buf, version + __pos, cnt);

	if (r < 0)
		return r;

	*pos += cnt - r;

	return cnt - r;
}

static ssize_t tee_version_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	struct tee *tee;
	char tee_version[32];

	if (buf == NULL || pos == NULL)
		return -EINVAL;

	tee = (struct tee *) PDE_DATA(file_inode(file));

	snprintf(tee_version, sizeof(tee_version),
		"%u.%u.%u-gp\n",
		tee->version.maj, tee->version.mid, tee->version.min);

	return copy_to_user_str(buf, count, pos, tee_version);
}

static const struct proc_ops tee_version_ops = {
	.proc_read = tee_version_read,
};

static ssize_t drv_version_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	if (buf == NULL || pos == NULL)
		return -EINVAL;

	return copy_to_user_str(buf, count, pos, tkcore_nsdrv_version);
}

static const struct proc_ops drv_version_ops = {
	.proc_read = drv_version_read,
};

#define TEED_VERSION_SIZE 50
char teed_version[TEED_VERSION_SIZE + 1] = "unknown\n";

static ssize_t teed_version_read(struct file *file, char __user *buf,
				 size_t count, loff_t *pos)
{
	if (buf == NULL || pos == NULL)
		return -EINVAL;

	return copy_to_user_str(buf, count, pos, teed_version);
}

static ssize_t teed_version_write(struct file *filp, const char __user *buf,
				  size_t count, loff_t *pos)
{
	ssize_t r;

	if (count > TEED_VERSION_SIZE)
		return -ENOMEM;

	r = copy_from_user(teed_version, buf, count);
	if (r < 0)
		return r;

	teed_version[count + 1] = '\0';

	return count;
}

static const struct proc_ops teed_version_ops = {
	.proc_read = teed_version_read,
	.proc_write = teed_version_write,
};

static void remove_entry(void)
{
	proc_remove(tee_proc_dir);

	tee_proc_dir = NULL;
	tee_proc_log_file = NULL;
	tee_proc_drv_version = NULL;
	tee_proc_tee_version = NULL;
}

static int create_entry(struct tee *tee)
{

	tee_proc_dir = proc_mkdir("tkcore", NULL);
	if (tee_proc_dir == NULL) {
		pr_err("proc_mkdir tkcore failed\n");
		return -1;
	}

	tee_proc_log_file = proc_create_data("tkcore_log",
		0444, tee_proc_dir, &log_tee_ops, (void *) tee);

	if (tee_proc_log_file == NULL) {
		pr_err("proc_create failed\n");
		goto err;
	}

	tee_proc_drv_version = proc_create_data(
			"tkcore_drv_version",
			0444, tee_proc_dir,
			&drv_version_ops, (void *) tee);

	if (tee_proc_drv_version == NULL) {
		pr_err("proc_create tkcore_drv_version failed\n");
		goto err;
	}

	tee_proc_tee_version = proc_create_data("tkcore_os_version",
			0444, tee_proc_dir,
			&tee_version_ops, (void *) tee);

	if (tee_proc_tee_version == NULL) {
		pr_err("proc_create tkcore_os_version failed\n");
		goto err;
	}

	tee_proc_teed_version = proc_create_data("tkcore_teed_version",
			0666, tee_proc_dir, &teed_version_ops, (void *) tee);

	if (tee_proc_teed_version == NULL) {
		pr_err("proc_create tkcore_teed_version failed\n");
		goto err;
	}

	return 0;

err:
	remove_entry();
	return -1;
}

static irqreturn_t tkcore_log_irq_handler(int irq, void *dev_id)
{
	wake_up_all(&(klog.wq));
	return IRQ_HANDLED;
}

#define LINE_LENGTH	120U

static void kprint_line(const char *logline)
{
#if defined(ENABLE_PRINTK_TEE_LOG)
	pr_info("%s\n", logline);
#else
	(void) logline;
#endif
}

static void log_rb(struct klog *klog)
{
	char *p;
	uint32_t rseq, wseq;

	const char tail[] = "<...>";
	char line[LINE_LENGTH + sizeof(tail) + 1];

	union tee_log_ctrl *log_ctl = klog->log_ctl;
	char *rb = klog->tee_rb;

	strcpy(&line[LINE_LENGTH], tail);

	rseq = log_ctl->info.tee_read_seq;
	wseq = READ_ONCE(log_ctl->info.tee_write_seq);

	p = line;

	do {
		uint32_t copy_len, i, k;

		if (rb_overrun(rseq, wseq, klog->tee_rb_len)) {
			kprint_line("---- interrupted");
			rseq = log_ctl->info.tee_read_seq =
				wseq - klog->tee_rb_len;
		}

		k = rseq % klog->tee_rb_len;

		copy_len = min(LINE_LENGTH - (uint32_t) (p - line),
				min(wseq - rseq, klog->tee_rb_len - k));

		for (i = 0; i < copy_len &&
				rb[k + i] != '\n' && rb[k + i] != '\0';
				i++, p++) {
			*p = rb[k + i];
		}

		rseq += i;

		if (i != copy_len) {
			/*
			 * find an '\n' in buffer
			 * we skip it
			 */
			++rseq;
			*p = '\0';
		}

		if (((i == copy_len) && (p - line == LINE_LENGTH))
				|| (i != copy_len)) {
			kprint_line(line);
			p = line;
			log_ctl->info.tee_read_seq = rseq;
		}

		wseq = READ_ONCE(log_ctl->info.tee_write_seq);
		if (wseq >= klog->tee_rb_len)
			klog->overwrite = true;
	} while (rseq != wseq);
}

static int logd(void *args)
{
	struct klog *klog = (struct klog *) args;
	union tee_log_ctrl *log_ctl = klog->log_ctl;

	++log_ctl->info.tee_reader_alive;

	while (!kthread_should_stop()) {
		/*
		 * a memory barrier is implied by
		 * wait_event_interruptible_timeout(..)
		 */
		if (wait_event_interruptible_timeout(klog->wq,
			log_ctl->info.tee_read_seq !=
			READ_ONCE(log_ctl->info.tee_write_seq),
			msecs_to_jiffies(TEE_LOG_TIMEOUT_MS)) <= 0) {
			/*
			 * interrupted /
			 * timeout and condition
			 * evaluated to false
			 */

			continue;
		}

		log_rb(klog);
	}

	--log_ctl->info.tee_reader_alive;
	return 0;
}

static int register_klog_irq(struct tee *tee, struct klog *klog)
{
	int r;

	if (tee->log.irq == 0) {
		pr_warn("tkcoredrv: unknown tee log irq\n");
		return 0;
	}

	pr_info("tkcoredrv: tee log irq = %d\n",
			tee->log.irq);

	r = request_irq(tee->log.irq,
		(irq_handler_t) tkcore_log_irq_handler,
		IRQF_TRIGGER_RISING,
		"tee_log_irq", NULL);

	if (r != 0) {
		pr_err("tkcoredrv: failed to register klog_irq with %d\n",
			r);
		return -1;
	}

	klog->notify_irq = tee->log.irq;

	return 0;
}

static int init_klog_shm(struct klog *klog,
						void *shm_va,
						size_t shm_len)
{
	char *rb;
	uint32_t rb_len;
	union tee_log_ctrl *log_ctl;

	if (shm_len <= TEE_LOG_CTL_BUF_SIZE) {
		pr_err("tkcoredrv: init_klog_shm: shm_len %zu too small\n",
				shm_len);
		return -1;
	}

	log_ctl = (union tee_log_ctrl *) shm_va;

	rb = (char *) log_ctl + TEE_LOG_CTL_BUF_SIZE;
	rb_len = log_ctl->info.tee_buf_size;

	if (rb_len != shm_len - TEE_LOG_CTL_BUF_SIZE) {
		pr_err("tkcoredrv:Unexpected shm length: %u\n",
				rb_len);
		return -1;
	}

	log_ctl->info.tee_reader_alive = 0;

	klog->log_ctl = log_ctl;

	klog->tee_rb = rb;
	klog->tee_rb_len = rb_len;

	return 0;
}

static int init_klog(struct klog *klog, struct tee *tee)
{
	/* sanity of tee is verified in tee_init_procfs */

	BUILD_BUG_ON(sizeof(union tee_log_ctrl)
			!= TEE_LOG_CTL_BUF_SIZE);

	memset(klog, 0, sizeof(*klog));

	if (tee->log.buffer == NULL ||
			tee->log.length == 0) {
		/* tee log setup fails */
		return 0;
	}

	if (init_klog_shm(klog,
					tee->log.buffer,
					tee->log.length) < 0) {
		return -1;
	}

	klog->overwrite = false;

	init_waitqueue_head(&klog->wq);

	if (register_klog_irq(tee, klog) < 0)
		goto err_unmap_shm;

	klog->ts = kthread_run(logd,
				(void *) klog, "tee-log");
	if (klog->ts == NULL) {
		pr_err("tkcoredrv: Failed to create kthread\n");
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	if (klog->notify_irq > 0)
		free_irq(klog->notify_irq, NULL);

err_unmap_shm:

	memset(klog, 0, sizeof(*klog));
	/*
	 * set notify_irq to
	 * un-initialized state
	 */
	klog->notify_irq = 0;

	return -1;
}

/* TODO wait for kthread logwq
 * to exit
 */
static void free_klog(struct klog *klog)
{
	if (klog->notify_irq > 0) {
		free_irq(klog->notify_irq, NULL);
		klog->notify_irq = 0;
	}

	kthread_stop(klog->ts);
	klog->ts = NULL;

	/*
	 * wake up all waiters
	 * in wq, since we're
	 * about to leave
	 */
	wake_up_all(&(klog->wq));

	/*
	 * we don't unmap klog->tee_rb,
	 * because it's not
	 * quite easy to check whether
	 * all user process using
	 * procfs has finished
	 */
}

int tee_init_procfs(struct tee *tee)
{
	if (WARN_ON(!tee || !tee->priv))
		return -1;

	if (create_entry(tee) < 0)
		return -1;

	if (init_klog(&klog, tee) < 0)
		goto out_remove_entry;

	return 0;

out_remove_entry:
	remove_entry();

	return -1;
}

void tee_exit_procfs(void)
{
	remove_entry();
	free_klog(&klog);
}
