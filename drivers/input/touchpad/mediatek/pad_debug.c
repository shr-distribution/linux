/*
 * Copyright (C) 2023 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "pad.h"
#include "pad_debug.h"

#ifdef PAD_DEBUG_CODE
int pad_fail_count;
int pad_trial_count;

int pad_debug_nr;
module_param(pad_debug_nr, int, 0644);
module_param(pad_fail_count, int, 0644);
module_param(pad_trial_count, int, 0644);

int pad_debug_time;
long pad_debug_last_2_int_time[2] = { 0 };

long pad_last_down_time;
int pad_start_profiling;

int pad_down_status;
module_param(pad_debug_time, int, 0644);

void pad_debug_set_time(int time)
{
	struct timeval t;

	if (!pad_debug_time && !pad_em_log)
		return;

	do_gettimeofday(&t);
	pad_debug_last_2_int_time[0] = pad_debug_last_2_int_time[1];
	pad_debug_last_2_int_time[1] = (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;

	/*  */
	/* Start profiling while receive touch DOWN event */
	/* Stop profiling while the first frame is upadted */
	/*  */
	if (!pad_down_status) {
		pad_start_profiling = 1;
		pad_last_down_time = pad_debug_last_2_int_time[1];
	}
}

#define BUFFER_SIZE 128

int pad_em_log;
module_param(pad_em_log, int, 0664);

void pad_enable_em_log(int enable)
{
	if (enable)
		pad_em_log = 1;
	else
		pad_em_log = 0;
}
EXPORT_SYMBOL(pad_enable_em_log);

int pad_em_log_to_fs;
module_param(pad_em_log_to_fs, int, 0664);

int pad_em_log_first = 1;

struct pad_em_log_struct {
	struct list_head list;
	char data[BUFFER_SIZE];
};
static LIST_HEAD(pad_em_log_list);
int pad_log_line_buffer = 128;	/* per line 128 bytes */
int pad_log_line_cnt = 1024 * 10;
module_param(pad_log_line_buffer, int, 0664);
module_param(pad_log_line_cnt, int, 0664);

struct pad_debug_log_buf {
	unsigned int head;
	unsigned int tail;
	spinlock_t buffer_lock;
	unsigned int cnt;
	unsigned char *buffer;
};
struct pad_debug_log_buf pad_buf;

static int pad_debug_log_open(struct inode *inode, struct file *file)
{
	if (pad_buf.buffer == NULL)
		pad_buf.buffer =
			vmalloc(pad_log_line_cnt * pad_log_line_buffer);
	if (pad_buf.buffer == NULL) {
		pr_info("pad_log: nomem for pad_buf->buffer\n");
		return -ENOMEM;
	}
	spin_lock(&pad_buf.buffer_lock);
	pad_buf.head = pad_buf.tail = 0;
	pad_buf.cnt = 0;
	spin_unlock(&pad_buf.buffer_lock);

	file->private_data = &pad_buf;
	pr_debug("[pad_em_log]: open log file\n");
	return 0;
}

static int pad_debug_log_release(struct inode *inode, struct file *file)
{
	unsigned char *tmp_buffer = NULL;

	pr_debug("[pad_em_log]: close log file\n");
	spin_lock(&pad_buf.buffer_lock);
	tmp_buffer = pad_buf.buffer;
	pad_buf.buffer = NULL;
	spin_unlock(&pad_buf.buffer_lock);
	if (tmp_buffer)
		vfree(tmp_buffer);
	return 0;
}

static ssize_t pad_debug_log_write(struct file *file, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t pad_debug_log_read(struct file *file, char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct pad_debug_log_buf *pad_buf =
		(struct pad_debug_log_buf *)file->private_data;
	unsigned int retval = 0, unit = pad_log_line_buffer;
	unsigned char *tmp_buf = NULL;

	if (pad_buf == NULL)
		return -ENOMEM;

	if (pad_buf->head == pad_buf->tail && (file->f_flags & O_NONBLOCK))
		return -EAGAIN;

	while ((count - retval) >= unit) {
		spin_lock_irq(&pad_buf->buffer_lock);
		if (pad_buf->head != pad_buf->tail) {
			tmp_buf = &pad_buf->buffer[pad_buf->tail++ * unit];
			pad_buf->tail &= pad_log_line_cnt - 1;
		} else {
			spin_unlock_irq(&pad_buf->buffer_lock);
			break;
		}
		spin_unlock_irq(&pad_buf->buffer_lock);
		/* printk("%s, tmp_buf:0x%x\n", tmp_buf, tmp_buf); */
		if (copy_to_user(buffer + retval, tmp_buf, unit))
			return -EFAULT;

		retval += unit;
	}
	return retval;
}

static unsigned char *pad_log_find_buffer(void)
{
	unsigned char *buffer = NULL;
	unsigned int unit = pad_log_line_buffer;

	if (pad_buf.buffer == NULL) {
		pr_info("[pad_em_log] :pad_buf.buffer is NULL\n");
		return NULL;
	}
	spin_lock(&pad_buf.buffer_lock);
	buffer = &pad_buf.buffer[pad_buf.head++ * unit];
	pad_buf.head &= pad_log_line_cnt - 1;
	spin_unlock(&pad_buf.buffer_lock);
	if (pad_buf.head == pad_buf.tail) {
		snprintf(buffer, unit, "[pad_em_log] overlay !!!!!\n");
		return NULL;
	}
	memset(buffer, 0, unit);
	return buffer;
}

static const struct file_operations pad_debug_log_fops = {
	.owner = THIS_MODULE,
	.read = pad_debug_log_read,
	.write = pad_debug_log_write,
	.open = pad_debug_log_open,
	.release = pad_debug_log_release,
};

static struct miscdevice pad_debug_log_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pad_em_log",
	.fops = &pad_debug_log_fops,
};

/* External MET touchpad function */
int MET_touchpad(int type, int single_num, int diff_x, int diff_y)
{
	struct timeval t;

	do_gettimeofday(&t);
	if ((pad_down_status == 0 && type == 1) ||
		(pad_down_status == 1 && type == 0)) {
		pad_down_status = !pad_down_status;
	}
	return 0;
}

void pad_em_log_output(int raw_x, int raw_y,
			int cal_x, int cal_y, int p, int down)
{
	if (down == PAD_TYPE_INT_DOWN) {
		pr_debug("[pad_em_log] int trigger down\n");
	} else if (down == PAD_TYPE_INT_UP) {
		pr_debug("[pad_em_log] int trigger up\n");
	} else if (down == PAD_TYPE_TIMER) {
		pr_debug("[pad_em_log] timer trigger\n");
	} else if (down == PAD_TYPE_RAW_DATA) {
		if (pad_em_log == PAD_TYPE_RAW_DATA) {
			pr_debug("[pad_em_log] rx=%d,ry=%d,rz1=%d"
				SPLIT "rz2=%d,p=%d,r\n",
			       raw_x, raw_y, cal_x, cal_y, p);
		}
	} else if (down == PAD_TYPE_REJECT1) {
		pr_debug("[pad_em_log] the first or last point is rejected\n");
	} else if (down == PAD_TYPE_REJECT2) {
		pr_debug
		    ("[pad_em_log] pressure(%d) > NICE_PRESSURE(%d)"
		    SPLIT "debounce debt0:%d ms, debt1:%d ms, spl_num:%d\n",
		     raw_x, raw_y, cal_x, cal_y, p);
	} else if (down == PAD_TYPE_FIST_LATENCY) {
		pr_debug("[pad_em_log] The first touch latency is %d ms\n",
			raw_x / 1000);
	} else if (down && pad_down_status == 0) {
		pr_debug("[pad_em_log] rx=%d,ry=%d,cx=%d"
			SPLIT "cy=%d,p=%d,d(+%ld ms)\n",
		       raw_x, raw_y, cal_x, cal_y, p,
		       (pad_debug_last_2_int_time[1] -
				pad_debug_last_2_int_time[0]) / 1000);
	} else if (down && pad_down_status != 0) {
		pr_debug("[pad_em_log] rx=%d,ry=%d,cx=%d"
			SPLIT "cy=%d,p=%d,m(+%ld ms)\n",
		       raw_x, raw_y, cal_x, cal_y, p,
		       (pad_debug_last_2_int_time[1] -
				pad_debug_last_2_int_time[0]) / 1000);
	} else {
		pr_debug("[pad_em_log] rx=%d,ry=%d,cx=%d"
			SPLIT "cy=%d,p=%d,u(+%ld ms)\n",
		       raw_x, raw_y, cal_x, cal_y, p,
		       (pad_debug_last_2_int_time[1] -
				pad_debug_last_2_int_time[0]) / 1000);
	}
}

void pad_em_log_store(int raw_x, int raw_y,
			int cal_x, int cal_y, int p, int down)
{
	struct timeval t;
	unsigned char *buffer = NULL;

	buffer = pad_log_find_buffer();
	if (buffer == NULL) {
		pr_info("not buffer\n");
		return;
	}
	do_gettimeofday(&t);

	if (down == PAD_TYPE_INT_DOWN) {
		snprintf(buffer, pad_log_line_buffer,
			"[%5lu.%06lu][pad_em_log] int trigger down\n",
			 (t.tv_sec & 0xFFF), t.tv_usec);
	} else if (down == PAD_TYPE_INT_UP) {
		snprintf(buffer, pad_log_line_buffer,
			"[%5lu.%06lu][pad_em_log] int trigger up\n",
			 (t.tv_sec & 0xFFF), t.tv_usec);
	} else if (down == PAD_TYPE_TIMER) {
		snprintf(buffer, pad_log_line_buffer,
			"[%5lu.%06lu][pad_em_log] timer trigger\n",
			 (t.tv_sec & 0xFFF), t.tv_usec);
	} else if (down == PAD_TYPE_RAW_DATA) {
		if (pad_em_log == PAD_TYPE_RAW_DATA) {
			snprintf(buffer, pad_log_line_buffer,
				 "[%5lu.%06lu][pad_em_log]"
				 SPLIT "rx=%d,ry=%d,rz1=%d,rz2=%d,p=%d,r\n",
				 (t.tv_sec & 0xFFF), t.tv_usec,
				 raw_x, raw_y, cal_x, cal_y, p);
		}
	} else if (down == PAD_TYPE_REJECT1) {
		snprintf(buffer, pad_log_line_buffer,
			 "[%5lu.%06lu][pad_em_log]"
			 SPLIT "the first or last point is rejected\n",
			 (t.tv_sec & 0xFFF), t.tv_usec);
	} else if (down == PAD_TYPE_REJECT2) {
		snprintf(buffer, pad_log_line_buffer,
			 "[%5lu.%06lu][pad_em_log]"
			 SPLIT "pressure(%d) > NICE_PRESSURE(%d), "
			 SPLIT "debounce debt0:%d, debt1:%d, spl_num:%d\n",
			 (t.tv_sec & 0xFFF), t.tv_usec,
			 raw_x, raw_y, cal_x, cal_y, p);
	} else if (down == PAD_TYPE_FIST_LATENCY) {
		snprintf(buffer, pad_log_line_buffer,
			 "[%5lu.%06lu][pad_em_log]"
			 SPLIT "The first touch latency is %d ms\n",
			 (t.tv_sec & 0xFFF), t.tv_usec, raw_x / 1000);
	} else if (down && pad_down_status == 0) {
		snprintf(buffer, pad_log_line_buffer,
			 "[%5lu.%06lu][pad_em_log]"
			 SPLIT "rx=%d,ry=%d,cx=%d,cy=%d,p=%d,d(+%ld ms)\n",
			 (t.tv_sec & 0xFFF), t.tv_usec,
			 raw_x, raw_y, cal_x, cal_y, p,
			 (pad_debug_last_2_int_time[1] -
				pad_debug_last_2_int_time[0]) / 1000);
	} else if (down && pad_down_status != 0) {
		snprintf(buffer, pad_log_line_buffer,
			 "[%5lu.%06lu][pad_em_log]"
			 SPLIT "rx=%d,ry=%d,cx=%d,cy=%d,p=%d,d(+%ld ms)\n",
			 (t.tv_sec & 0xFFF), t.tv_usec,
			 raw_x, raw_y, cal_x, cal_y, p,
			 (pad_debug_last_2_int_time[1] -
				pad_debug_last_2_int_time[0]) / 1000);
	} else {
		snprintf(buffer, pad_log_line_buffer,
			 "[%5lu.%06lu][pad_em_log]"
			 SPLIT "rx=%d,ry=%d,cx=%d,cy=%d,p=%d,u(+%ld ms)\n",
			 (t.tv_sec & 0xFFF), t.tv_usec,
			 raw_x, raw_y, cal_x, cal_y, p,
			 (pad_debug_last_2_int_time[1] -
				pad_debug_last_2_int_time[0]) / 1000);
	}
}

void pad_em_log_release(void)
{
	struct pad_em_log_struct *p, *p_tmp;

	if (!pad_em_log_first) {
		remove_proc_entry("pad_em_log", NULL);

		list_for_each_entry_safe(p, p_tmp, &pad_em_log_list, list) {
			list_del(&p->list);
			kfree(p);
		}

		pad_em_log_first = 1;
		pad_em_log_to_fs = 0;
	}
}

int pad_debug_no_response(int enable)
{
	return 0;
}

static int __init pad_log_init(void)
{
	if (misc_register(&pad_debug_log_dev) < 0) {
		pr_info("[pad_em_log] :register device failed\n");
		return -1;
	}
	pr_info("[pad_em_log] :register device successfully\n");
	spin_lock_init(&pad_buf.buffer_lock);
	return 0;
}

int pad_debuglog;
module_param(pad_debuglog, int, 0664);
module_init(pad_log_init);
#endif /* PAD_DEBUG_CODE */