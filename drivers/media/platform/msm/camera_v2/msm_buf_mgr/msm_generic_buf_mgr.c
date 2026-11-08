/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "msm_generic_buf_mgr.h"

static struct msm_buf_mngr_device *msm_buf_mngr_dev;

struct v4l2_subdev *msm_buf_mngr_get_subdev(void)
{
	return &msm_buf_mngr_dev->subdev.sd;
}

static int msm_buf_mngr_get_buf(struct msm_buf_mngr_device *buf_mngr_dev,
	void __user *argp)
{
	unsigned long flags;
	struct msm_buf_mngr_info *buf_info =
		(struct msm_buf_mngr_info *)argp;
	struct msm_get_bufs *bufs, *save;
	struct msm_get_bufs *new_entry =
		kzalloc(sizeof(struct msm_get_bufs), GFP_KERNEL);

	if (!new_entry) {
		pr_err("%s:No mem\n", __func__);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&new_entry->entry);
	new_entry->vb2_buf = buf_mngr_dev->vb2_ops.get_buf(buf_info->session_id,
		buf_info->stream_id);
	if (!new_entry->vb2_buf) {
		pr_debug("%s:Get buf is null\n", __func__);
		kfree(new_entry);
		return -EINVAL;
	}
	new_entry->session_id = buf_info->session_id;
	new_entry->stream_id = buf_info->stream_id;
	new_entry->index = new_entry->vb2_buf->v4l2_buf.index;
	spin_lock_irqsave(&buf_mngr_dev->buf_q_spinlock, flags);
	/*
	 * get_buf only hands out a buffer nobody holds, so an entry that is
	 * still listed for the same session, stream and index is stale: a
	 * buffer that was never done or put before its stream went away (the
	 * camera daemon leaks one when a bus message races a metadata stream
	 * off). Its vb2_buf may be freed, and buf_done would match it before
	 * the new entry and leave the real buffer undelivered.
	 */
	list_for_each_entry_safe(bufs, save, &buf_mngr_dev->buf_qhead, entry) {
		if (bufs->session_id == new_entry->session_id &&
			bufs->stream_id == new_entry->stream_id &&
			bufs->index == new_entry->index) {
			pr_warn("%s: dropping stale entry %u/%u/%u\n", __func__,
				bufs->session_id, bufs->stream_id, bufs->index);
			list_del_init(&bufs->entry);
			kfree(bufs);
		}
	}
	list_add_tail(&new_entry->entry, &buf_mngr_dev->buf_qhead);
	spin_unlock_irqrestore(&buf_mngr_dev->buf_q_spinlock, flags);
	buf_info->index = new_entry->index;
	return 0;
}

/*
 * Whether an entry still names a buffer of its stream's current vb2 queue. A
 * stream that was torn down and created again has new buffers, and the
 * entry's vb2_buf may be freed: compare pointers, never dereference it.
 */
static bool msm_buf_mngr_entry_live(struct msm_buf_mngr_device *buf_mngr_dev,
	struct msm_get_bufs *bufs)
{
	struct vb2_queue *q;

	if (!buf_mngr_dev->vb2_ops.get_vb2_queue)
		return true;
	q = buf_mngr_dev->vb2_ops.get_vb2_queue(bufs->session_id,
		bufs->stream_id);
	return q && bufs->index < q->num_buffers &&
		q->bufs[bufs->index] == bufs->vb2_buf;
}

static int msm_buf_mngr_buf_done(struct msm_buf_mngr_device *buf_mngr_dev,
	struct msm_buf_mngr_info *buf_info)
{
	unsigned long flags;
	struct msm_get_bufs *bufs, *save;
	int ret = -EINVAL;

	spin_lock_irqsave(&buf_mngr_dev->buf_q_spinlock, flags);
	list_for_each_entry_safe(bufs, save, &buf_mngr_dev->buf_qhead, entry) {
		if ((bufs->session_id == buf_info->session_id) &&
			(bufs->stream_id == buf_info->stream_id) &&
			(bufs->index == buf_info->index)) {
			if (!msm_buf_mngr_entry_live(buf_mngr_dev, bufs)) {
				pr_warn("%s: dropping stale entry %u/%u/%u\n",
					__func__, bufs->session_id,
					bufs->stream_id, bufs->index);
				list_del_init(&bufs->entry);
				kfree(bufs);
				continue;
			}
			bufs->vb2_buf->v4l2_buf.sequence  = buf_info->frame_id;
			bufs->vb2_buf->v4l2_buf.timestamp = buf_info->timestamp;
			ret = buf_mngr_dev->vb2_ops.buf_done
					(bufs->vb2_buf,
						buf_info->session_id,
						buf_info->stream_id);
			list_del_init(&bufs->entry);
			kfree(bufs);
			break;
		}
	}
	spin_unlock_irqrestore(&buf_mngr_dev->buf_q_spinlock, flags);
	return ret;
}


static int msm_buf_mngr_put_buf(struct msm_buf_mngr_device *buf_mngr_dev,
	struct msm_buf_mngr_info *buf_info)
{
	unsigned long flags;
	struct msm_get_bufs *bufs, *save;
	int ret = -EINVAL;

	spin_lock_irqsave(&buf_mngr_dev->buf_q_spinlock, flags);
	list_for_each_entry_safe(bufs, save, &buf_mngr_dev->buf_qhead, entry) {
		if ((bufs->session_id == buf_info->session_id) &&
			(bufs->stream_id == buf_info->stream_id) &&
			(bufs->index == buf_info->index)) {
			if (!msm_buf_mngr_entry_live(buf_mngr_dev, bufs)) {
				pr_warn("%s: dropping stale entry %u/%u/%u\n",
					__func__, bufs->session_id,
					bufs->stream_id, bufs->index);
				list_del_init(&bufs->entry);
				kfree(bufs);
				continue;
			}
			ret = buf_mngr_dev->vb2_ops.put_buf(bufs->vb2_buf,
				buf_info->session_id, buf_info->stream_id);
			list_del_init(&bufs->entry);
			kfree(bufs);
			break;
		}
	}
	spin_unlock_irqrestore(&buf_mngr_dev->buf_q_spinlock, flags);
	return ret;
}

/*
 * msm_close(): the camera daemon closed the msm config node, which it holds
 * open for its whole life, so every session is being destroyed and no buffer
 * handed out by get_buf will be done or put any more. Free all entries rather
 * than carry them, with their vb2_buf pointers, into the next daemon's
 * sessions, which reuse the same session and stream ids. The close ioctl's
 * argument is not filled in by msm_close() and is not read.
 */
static void msm_buf_mngr_sd_shutdown(struct msm_buf_mngr_device *buf_mngr_dev)
{
	unsigned long flags;
	struct msm_get_bufs *bufs, *save;

	spin_lock_irqsave(&buf_mngr_dev->buf_q_spinlock, flags);
	list_for_each_entry_safe(bufs, save, &buf_mngr_dev->buf_qhead, entry) {
		pr_debug("%s: freeing %u/%u/%u\n", __func__, bufs->session_id,
			bufs->stream_id, bufs->index);
		list_del_init(&bufs->entry);
		kfree(bufs);
	}
	spin_unlock_irqrestore(&buf_mngr_dev->buf_q_spinlock, flags);
}

static long msm_buf_mngr_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct msm_buf_mngr_device *buf_mngr_dev = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;

	if (!buf_mngr_dev) {
		pr_err("%s buf manager device NULL\n", __func__);
		rc = -ENOMEM;
		return rc;
	}

	switch (cmd) {
	case VIDIOC_MSM_BUF_MNGR_GET_BUF:
		rc = msm_buf_mngr_get_buf(buf_mngr_dev, argp);
		break;
	case VIDIOC_MSM_BUF_MNGR_BUF_DONE:
		rc = msm_buf_mngr_buf_done(buf_mngr_dev, argp);
		break;
	case VIDIOC_MSM_BUF_MNGR_PUT_BUF:
		rc = msm_buf_mngr_put_buf(buf_mngr_dev, argp);
		break;
	case MSM_SD_SHUTDOWN:
		msm_buf_mngr_sd_shutdown(buf_mngr_dev);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return rc;
}

static struct v4l2_subdev_core_ops msm_buf_mngr_subdev_core_ops = {
	.ioctl = msm_buf_mngr_subdev_ioctl,
};

static const struct v4l2_subdev_ops msm_buf_mngr_subdev_ops = {
	.core = &msm_buf_mngr_subdev_core_ops,
};

static const struct of_device_id msm_buf_mngr_dt_match[] = {
	{.compatible = "qcom,msm_buf_mngr"},
};

static int __init msm_buf_mngr_init(void)
{
	int rc = 0;
	msm_buf_mngr_dev = kzalloc(sizeof(*msm_buf_mngr_dev),
		GFP_KERNEL);
	if (WARN_ON(!msm_buf_mngr_dev)) {
		pr_err("%s: not enough memory", __func__);
		return -ENOMEM;
	}
	/* Sub-dev */
	v4l2_subdev_init(&msm_buf_mngr_dev->subdev.sd,
		&msm_buf_mngr_subdev_ops);
	snprintf(msm_buf_mngr_dev->subdev.sd.name,
		ARRAY_SIZE(msm_buf_mngr_dev->subdev.sd.name), "msm_buf_mngr");
	msm_buf_mngr_dev->subdev.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	v4l2_set_subdevdata(&msm_buf_mngr_dev->subdev.sd, msm_buf_mngr_dev);

	media_entity_init(&msm_buf_mngr_dev->subdev.sd.entity, 0, NULL, 0);
	msm_buf_mngr_dev->subdev.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_buf_mngr_dev->subdev.sd.entity.group_id =
		MSM_CAMERA_SUBDEV_BUF_MNGR;
	msm_buf_mngr_dev->subdev.close_seq = MSM_SD_CLOSE_4TH_CATEGORY;
	rc = msm_sd_register(&msm_buf_mngr_dev->subdev);
	if (rc != 0) {
		pr_err("%s: msm_sd_register error = %d\n", __func__, rc);
		goto end;
	}

	v4l2_subdev_notify(&msm_buf_mngr_dev->subdev.sd, MSM_SD_NOTIFY_REQ_CB,
		&msm_buf_mngr_dev->vb2_ops);

	INIT_LIST_HEAD(&msm_buf_mngr_dev->buf_qhead);
	spin_lock_init(&msm_buf_mngr_dev->buf_q_spinlock);
end:
	return rc;
}

static void __exit msm_buf_mngr_exit(void)
{
	kfree(msm_buf_mngr_dev);
}

module_init(msm_buf_mngr_init);
module_exit(msm_buf_mngr_exit);
MODULE_DESCRIPTION("MSM Buffer Manager");
MODULE_LICENSE("GPL v2");

