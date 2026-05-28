// SPDX-License-Identifier: GPL-2.0+
/*
 * f_novacom.c -- novacom USB gadget function driver
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Copyright (C) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * Vendor-specific USB interface for the host-side novacomd daemon used by
 * webOS devices (Palm Pre family, HP TouchPad). The kernel side is a thin
 * pipe; all framing lives in userspace. Three character devices are
 * exposed:
 *   - /dev/novacom_ep0    : Control/event endpoint (gadgetfs-style events)
 *   - /dev/novacom_ep_in  : Bulk IN  (device -> host)
 *   - /dev/novacom_ep_out : Bulk OUT (host   -> device)
 *
 * See Documentation/usb/gadget_novacom.rst for the userspace ABI.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/uaccess.h>

#include <linux/usb/gadgetfs.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include "u_novacom.h"

#define NOVACOM_DEVNAME_EP0	"novacom_ep0"
#define NOVACOM_DEVNAME_IN	"novacom_ep_in"
#define NOVACOM_DEVNAME_OUT	"novacom_ep_out"

/* Palm-specific USB identifiers */
#define NOVACOM_INTERFACE_CLASS		USB_CLASS_VENDOR_SPEC
#define NOVACOM_INTERFACE_SUBCLASS	0x47
#define NOVACOM_INTERFACE_PROTOCOL	0x11

/*
 * Maximum single-shot transfer size. Userspace requests larger than this are
 * silently truncated to this value and must be re-issued for the remainder.
 * Caps a single kmalloc; chosen to keep allocations within the kmalloc-safe
 * region while still letting novacomd push large frames in one syscall.
 */
#define NOVACOM_MAX_XFER	(256 * 1024)

/*-------------------------------------------------------------------------*/

enum novacom_conn_state {
	NOVACOM_DISCONNECTED,
	NOVACOM_CONNECTED,
};

enum novacom_dev_state {
	NOVACOM_DEV_UNBOUND = 0,
	NOVACOM_DEV_CLOSED,
	NOVACOM_DEV_OPENED,
};

enum novacom_ep_state {
	NOVACOM_EP_DISABLED = 0,
	NOVACOM_EP_ENABLED,
};

#define N_EVENT		5

static const char novacom_shortname[] = "novacom";

struct novacom_ep {
	struct mutex			lock;
	enum novacom_ep_state		state;
	struct f_novacom		*novacom;
	struct usb_ep			*ep;
	struct usb_request		*req;
	ssize_t				status;
	char				name[16];
	wait_queue_head_t		wait;
};

struct f_novacom {
	struct usb_function		function;
	u8				data_id;

	struct novacom_ep		ep_in;
	struct novacom_ep		ep_out;

	/* ep0 control */
	spinlock_t			lock;
	enum novacom_dev_state		state;
	enum novacom_conn_state		connect_state;
	struct usb_gadgetfs_event	event[N_EVENT];
	unsigned			ev_next;
	struct fasync_struct		*fasync;
	wait_queue_head_t		ep0_wait;
};

static inline struct f_novacom *func_to_novacom(struct usb_function *f)
{
	return container_of(f, struct f_novacom, function);
}

/*
 * The misc devices are static (NOVACOM_DEVNAME_*) so only one function
 * instance can be live at a time. ConfigFS will return -EBUSY on a second
 * mkdir; @the_novacom is the back-pointer used by the misc fops. Reads of
 * @the_novacom from fops must be serialized with bind/unbind via
 * @novacom_lock.
 */
static struct f_novacom *the_novacom;
static DEFINE_MUTEX(novacom_lock);

/*-------------------------------------------------------------------------*/
/* USB Descriptors                                                          */
/*-------------------------------------------------------------------------*/

static struct usb_interface_descriptor novacom_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	NOVACOM_INTERFACE_CLASS,
	.bInterfaceSubClass =	NOVACOM_INTERFACE_SUBCLASS,
	.bInterfaceProtocol =	NOVACOM_INTERFACE_PROTOCOL,
	/* .iInterface = DYNAMIC */
};

/* Full-speed descriptors */
static struct usb_endpoint_descriptor novacom_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by stack for full-speed */
};

static struct usb_endpoint_descriptor novacom_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by stack for full-speed */
};

static struct usb_descriptor_header *novacom_fs_function[] = {
	(struct usb_descriptor_header *) &novacom_interface_desc,
	(struct usb_descriptor_header *) &novacom_fs_in_desc,
	(struct usb_descriptor_header *) &novacom_fs_out_desc,
	NULL,
};

/* High-speed descriptors */
static struct usb_endpoint_descriptor novacom_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor novacom_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *novacom_hs_function[] = {
	(struct usb_descriptor_header *) &novacom_interface_desc,
	(struct usb_descriptor_header *) &novacom_hs_in_desc,
	(struct usb_descriptor_header *) &novacom_hs_out_desc,
	NULL,
};

/*
 * SuperSpeed is intentionally not advertised: the only known consumer of this
 * function (TI/Qualcomm-based webOS hardware) is high-speed, and SS bulk
 * needs proper bMaxBurst tuning + on-hardware testing before being claimed.
 */

/* String descriptors */
static struct usb_string novacom_string_defs[] = {
	[0].s = "Novacom",
	{ } /* end of list */
};

static struct usb_gadget_strings novacom_string_table = {
	.language =	0x0409,	/* en-us */
	.strings =	novacom_string_defs,
};

static struct usb_gadget_strings *novacom_strings[] = {
	&novacom_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/
/* Endpoint Helper Functions                                                */
/*-------------------------------------------------------------------------*/

static int novacom_enable_ep(struct novacom_ep *nep)
{
	if (nep->state == NOVACOM_EP_DISABLED) {
		dev_dbg(&nep->novacom->function.config->cdev->gadget->dev,
			"novacom: %s: NOVACOM_EP_ENABLED\n", nep->name);
		nep->state = NOVACOM_EP_ENABLED;
	}
	return 0;
}

static void novacom_disable_ep(struct novacom_ep *nep)
{
	if (nep->state == NOVACOM_EP_ENABLED) {
		dev_dbg(&nep->novacom->function.config->cdev->gadget->dev,
			"novacom: %s: NOVACOM_EP_DISABLED\n", nep->name);
		nep->state = NOVACOM_EP_DISABLED;
	}
}

static void novacom_epio_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct completion *done = req->context;
	struct f_novacom *novacom = the_novacom;
	struct novacom_ep *nep;

	if (!done) {
		dev_warn(&novacom->function.config->cdev->gadget->dev,
			 "novacom: completion has no context\n");
		return;
	}

	/* Determine which endpoint this is */
	if (ep == novacom->ep_in.ep)
		nep = &novacom->ep_in;
	else
		nep = &novacom->ep_out;

	if (req->status)
		nep->status = req->status;
	else
		nep->status = req->actual;

	complete(done);
}

/* Lock endpoint and wait until it's ready */
static int novacom_get_ready_ep(unsigned int f_flags, struct novacom_ep *nep)
{
	int val;

	if (f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&nep->lock))
			return -EAGAIN;
		if (nep->state != NOVACOM_EP_ENABLED) {
			mutex_unlock(&nep->lock);
			return -EAGAIN;
		}
		return 0;
	}

	if (mutex_lock_interruptible(&nep->lock))
		return -EINTR;

	switch (nep->state) {
	case NOVACOM_EP_ENABLED:
		val = 0;
		break;
	case NOVACOM_EP_DISABLED:
	default:
		dev_dbg(&nep->novacom->function.config->cdev->gadget->dev,
			"novacom: ep %p not available, state %d\n",
			nep, nep->state);
		val = -ENODEV;
		mutex_unlock(&nep->lock);
	}
	return val;
}

/*
 * Issue a single bulk transfer of at most one MPS worth of data
 * (a 0-length transfer is a ZLP on IN). Caller must hold nep->lock.
 *
 * Returns bytes transferred (>= 0) or a negative errno.
 */
static ssize_t novacom_ep_io_one(struct novacom_ep *nep, void *buf,
				 unsigned int len)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct f_novacom *novacom = nep->novacom;
	struct usb_composite_dev *cdev = novacom->function.config->cdev;
	unsigned long flags;
	int value;

	spin_lock_irqsave(&novacom->lock, flags);
	if (likely(nep->ep != NULL)) {
		struct usb_request *req = nep->req;

		req->context = &done;
		req->complete = novacom_epio_complete;
		req->buf = buf;
		req->length = len;
		req->zero = 0;
		value = usb_ep_queue(nep->ep, req, GFP_ATOMIC);
	} else {
		value = -ENODEV;
	}
	spin_unlock_irqrestore(&novacom->lock, flags);

	if (likely(value == 0)) {
		value = wait_for_completion_interruptible(&done);
		if (value != 0) {
			spin_lock_irqsave(&novacom->lock, flags);
			if (likely(nep->ep != NULL)) {
				dev_warn(&cdev->gadget->dev,
					 "novacom: %s i/o interrupted\n",
					 nep->name);
				usb_ep_dequeue(nep->ep, nep->req);
				spin_unlock_irqrestore(&novacom->lock, flags);

				/* Wait for dequeue completion with timeout */
				if (!wait_for_completion_timeout(&done,
						msecs_to_jiffies(1000)))
					dev_err(&cdev->gadget->dev,
						"novacom: i/o completion timeout\n");
				if (nep->status == -ECONNRESET)
					nep->status = -EINTR;
			} else {
				spin_unlock_irqrestore(&novacom->lock, flags);
				dev_err(&cdev->gadget->dev,
					"novacom: endpoint gone\n");
				nep->status = -ENODEV;
			}
		}
		return nep->status;
	}
	return value;
}

/*
 * Loop sub-MPS transfers until the user request is satisfied. Splitting
 * the request keeps each on-the-wire URB <= MPS so the chipidea (and any
 * other) UDC always sees short-packet termination cleanly. Without this,
 * a single large OUT URB sits ACTIVE in the dTD chain when the host
 * sends a short packet partway in.
 *
 * OUT (read): stop on the first short packet -- that signals end of
 * transfer from the host's point of view.
 * IN  (write): send all bytes, and append a ZLP if the total is an exact
 * multiple of MPS so the host's BULK IN URB terminates.
 */
static ssize_t novacom_ep_io(struct novacom_ep *nep, void *buf,
			     unsigned int len, int dir)
{
	struct f_novacom *novacom = nep->novacom;
	struct usb_composite_dev *cdev = novacom->function.config->cdev;
	unsigned int mps;
	ssize_t total = 0;
	ssize_t n;

	dev_dbg(&cdev->gadget->dev, "novacom: %s: len=%u dir=%s\n", __func__,
		len, dir == USB_DIR_IN ? "IN" : "OUT");

	if (!nep->ep || !nep->ep->desc)
		return -ENODEV;
	mps = usb_endpoint_maxp(nep->ep->desc);
	if (!mps)
		return -EIO;

	while (total < len) {
		unsigned int chunk = min_t(unsigned int, len - total, mps);

		n = novacom_ep_io_one(nep, buf + total, chunk);
		if (n < 0)
			return total > 0 ? total : n;
		total += n;

		/* OUT: a short packet ends the transfer. */
		if (dir == USB_DIR_OUT && n < chunk)
			break;
	}

	/* IN: terminate an exact-MPS-multiple stream with a ZLP so the
	 * host's BULK IN URB sees end-of-transfer. */
	if (dir == USB_DIR_IN && total > 0 && (total % mps) == 0)
		(void)novacom_ep_io_one(nep, buf, 0);

	return total;
}

/*-------------------------------------------------------------------------*/
/* Data Endpoint File Operations                                            */
/*-------------------------------------------------------------------------*/

static ssize_t novacom_ep_read(struct file *fd, char __user *buf,
			       size_t len, loff_t *ptr)
{
	struct novacom_ep *nep = fd->private_data;
	struct f_novacom *novacom = nep->novacom;
	struct usb_composite_dev *cdev = novacom->function.config->cdev;
	unsigned long flags;
	void *kbuf;
	ssize_t value;

	if (len == 0)
		return 0;
	len = min_t(size_t, len, NOVACOM_MAX_XFER);

	value = novacom_get_ready_ep(fd->f_flags, nep);
	if (value < 0)
		return value;

	/* Check for wrong-direction I/O (reading from IN endpoint) */
	if (nep == &novacom->ep_in) {
		dev_info(&cdev->gadget->dev, "novacom: %s halt (wrong dir)\n",
			 nep->name);
		spin_lock_irqsave(&novacom->lock, flags);
		if (likely(nep->ep != NULL))
			usb_ep_set_halt(nep->ep);
		spin_unlock_irqrestore(&novacom->lock, flags);
		mutex_unlock(&nep->lock);
		return -EBADMSG;
	}

	kbuf = kmalloc(len, GFP_KERNEL);
	if (unlikely(!kbuf)) {
		mutex_unlock(&nep->lock);
		return -ENOMEM;
	}

	value = novacom_ep_io(nep, kbuf, len, USB_DIR_OUT);
	if (value >= 0 && copy_to_user(buf, kbuf, value))
		value = -EFAULT;

	mutex_unlock(&nep->lock);
	kfree(kbuf);
	return value;
}

static ssize_t novacom_ep_write(struct file *fd, const char __user *buf,
				size_t len, loff_t *ptr)
{
	struct novacom_ep *nep = fd->private_data;
	struct f_novacom *novacom = nep->novacom;
	struct usb_composite_dev *cdev = novacom->function.config->cdev;
	unsigned long flags;
	void *kbuf;
	ssize_t value;

	if (len == 0)
		return 0;
	len = min_t(size_t, len, NOVACOM_MAX_XFER);

	value = novacom_get_ready_ep(fd->f_flags, nep);
	if (value < 0)
		return value;

	/* Check for wrong-direction I/O (writing to OUT endpoint) */
	if (nep == &novacom->ep_out) {
		dev_info(&cdev->gadget->dev, "novacom: %s halt (wrong dir)\n",
			 nep->name);
		spin_lock_irqsave(&novacom->lock, flags);
		if (likely(nep->ep != NULL))
			usb_ep_set_halt(nep->ep);
		spin_unlock_irqrestore(&novacom->lock, flags);
		mutex_unlock(&nep->lock);
		return -EBADMSG;
	}

	kbuf = kmalloc(len, GFP_KERNEL);
	if (!kbuf) {
		mutex_unlock(&nep->lock);
		return -ENOMEM;
	}

	if (copy_from_user(kbuf, buf, len)) {
		value = -EFAULT;
		goto out;
	}

	value = novacom_ep_io(nep, kbuf, len, USB_DIR_IN);

out:
	mutex_unlock(&nep->lock);
	kfree(kbuf);
	return value;
}

static int novacom_ep_release(struct inode *inode, struct file *fd)
{
	return 0;
}

static int novacom_ep_open(struct inode *inode, struct file *fd,
			   struct novacom_ep *nep)
{
	struct f_novacom *novacom;
	unsigned long flags;
	int value = -EBUSY;

	mutex_lock(&novacom_lock);
	novacom = the_novacom;
	if (!novacom) {
		mutex_unlock(&novacom_lock);
		return -ENOENT;
	}

	if (mutex_lock_interruptible(&nep->lock)) {
		mutex_unlock(&novacom_lock);
		return -EINTR;
	}

	spin_lock_irqsave(&novacom->lock, flags);
	if (nep->state == NOVACOM_EP_ENABLED) {
		value = 0;
		fd->private_data = nep;
		dev_dbg(&novacom->function.config->cdev->gadget->dev,
			"novacom: %s opened\n", nep->name);
	} else {
		dev_warn(&novacom->function.config->cdev->gadget->dev,
			 "novacom: %s busy (state=%d)\n", nep->name, nep->state);
	}
	spin_unlock_irqrestore(&novacom->lock, flags);
	mutex_unlock(&nep->lock);
	mutex_unlock(&novacom_lock);

	return value;
}

static int novacom_ep_in_open(struct inode *inode, struct file *fd)
{
	struct f_novacom *novacom = the_novacom;

	if (!novacom)
		return -ENOENT;
	return novacom_ep_open(inode, fd, &novacom->ep_in);
}

static int novacom_ep_out_open(struct inode *inode, struct file *fd)
{
	struct f_novacom *novacom = the_novacom;

	if (!novacom)
		return -ENOENT;
	return novacom_ep_open(inode, fd, &novacom->ep_out);
}

static const struct file_operations novacom_ep_in_fops = {
	.owner =	THIS_MODULE,
	.llseek =	noop_llseek,
	.open =		novacom_ep_in_open,
	.read =		novacom_ep_read,
	.write =	novacom_ep_write,
	.release =	novacom_ep_release,
};

static const struct file_operations novacom_ep_out_fops = {
	.owner =	THIS_MODULE,
	.llseek =	noop_llseek,
	.open =		novacom_ep_out_open,
	.read =		novacom_ep_read,
	.write =	novacom_ep_write,
	.release =	novacom_ep_release,
};

static struct miscdevice novacom_ep_in_device = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		NOVACOM_DEVNAME_IN,
	.fops =		&novacom_ep_in_fops,
};

static struct miscdevice novacom_ep_out_device = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		NOVACOM_DEVNAME_OUT,
	.fops =		&novacom_ep_out_fops,
};

/*-------------------------------------------------------------------------*/
/* EP0 (Control) File Operations                                            */
/*-------------------------------------------------------------------------*/

static struct usb_gadgetfs_event *
novacom_next_event(struct f_novacom *novacom, enum usb_gadgetfs_event_type type)
{
	struct usb_gadgetfs_event *event;
	unsigned int i;

	switch (type) {
	case GADGETFS_DISCONNECT:
	case GADGETFS_CONNECT:
		/* These events purge the queue */
		novacom->ev_next = 0;
		break;
	case GADGETFS_SETUP:
	case GADGETFS_SUSPEND:
		/* Remove duplicate events */
		for (i = 0; i != novacom->ev_next; i++) {
			if (novacom->event[i].type != type)
				continue;
			novacom->ev_next--;
			if (i == novacom->ev_next)
				break;
			memmove(&novacom->event[i], &novacom->event[i + 1],
				sizeof(struct usb_gadgetfs_event) *
				(novacom->ev_next - i));
		}
		break;
	default:
		WARN_ONCE(1, "novacom: unsupported event type %d\n", type);
		return NULL;
	}

	if (WARN_ON_ONCE(novacom->ev_next >= N_EVENT))
		return NULL;

	event = &novacom->event[novacom->ev_next++];
	memset(event, 0, sizeof(*event));
	event->type = type;
	return event;
}

static inline void novacom_ep0_send_event(struct f_novacom *novacom, int value)
{
	struct usb_gadgetfs_event *event;

	if (value > 0) {
		/* Configuration set */
		event = novacom_next_event(novacom, GADGETFS_SETUP);
		if (event) {
			event->u.setup.bRequest = USB_REQ_SET_CONFIGURATION;
			event->u.setup.wValue = cpu_to_le16(value);
		}
	} else {
		/* Disconnection */
		novacom_next_event(novacom, GADGETFS_DISCONNECT);
	}
	wake_up(&novacom->ep0_wait);
	kill_fasync(&novacom->fasync, SIGIO, POLL_IN);
}

static ssize_t novacom_ep0_read(struct file *fd, char __user *buf,
				size_t len, loff_t *ptr)
{
	struct f_novacom *novacom;
	enum novacom_conn_state connect_state;
	unsigned long flags;
	ssize_t retval;

	mutex_lock(&novacom_lock);
	novacom = the_novacom;
	if (!novacom) {
		mutex_unlock(&novacom_lock);
		return -ENOENT;
	}
	mutex_unlock(&novacom_lock);

	spin_lock_irqsave(&novacom->lock, flags);
	connect_state = novacom->connect_state;

	if (len < sizeof(novacom->event[0])) {
		retval = -EINVAL;
		goto done;
	}
	len -= len % sizeof(struct usb_gadgetfs_event);

scan:
	/* Return queued events immediately */
	if (novacom->ev_next != 0) {
		unsigned int n;

		n = len / sizeof(struct usb_gadgetfs_event);
		if (novacom->ev_next < n)
			n = novacom->ev_next;

		spin_unlock_irqrestore(&novacom->lock, flags);
		len = n * sizeof(struct usb_gadgetfs_event);
		if (copy_to_user(buf, &novacom->event, len))
			retval = -EFAULT;
		else
			retval = len;

		if (len > 0) {
			spin_lock_irqsave(&novacom->lock, flags);
			if (novacom->ev_next > n) {
				memmove(&novacom->event[0], &novacom->event[n],
					sizeof(struct usb_gadgetfs_event) *
					(novacom->ev_next - n));
			}
			novacom->ev_next -= n;
			spin_unlock_irqrestore(&novacom->lock, flags);
		}
		return retval;
	}

	if (fd->f_flags & O_NONBLOCK) {
		retval = -EAGAIN;
		goto done;
	}

	switch (connect_state) {
	case NOVACOM_DISCONNECTED:
	case NOVACOM_CONNECTED:
		spin_unlock_irqrestore(&novacom->lock, flags);
		/* Wait for events */
		retval = wait_event_interruptible(novacom->ep0_wait,
						  novacom->ev_next != 0);
		if (retval < 0)
			return retval;
		spin_lock_irqsave(&novacom->lock, flags);
		goto scan;
	default:
		retval = -ESRCH;
		break;
	}

done:
	spin_unlock_irqrestore(&novacom->lock, flags);
	return retval;
}

static int novacom_ep0_fasync(int f, struct file *fd, int on)
{
	struct f_novacom *novacom;

	mutex_lock(&novacom_lock);
	novacom = the_novacom;
	mutex_unlock(&novacom_lock);

	if (!novacom)
		return -ENOENT;

	return fasync_helper(f, fd, on, &novacom->fasync);
}

static int novacom_ep0_open(struct inode *inode, struct file *fd)
{
	struct f_novacom *novacom;
	unsigned long flags;
	int value = -EBUSY;

	mutex_lock(&novacom_lock);
	novacom = the_novacom;
	if (!novacom) {
		mutex_unlock(&novacom_lock);
		return -ENOENT;
	}

	spin_lock_irqsave(&novacom->lock, flags);
	if (novacom->state == NOVACOM_DEV_CLOSED) {
		novacom->state = NOVACOM_DEV_OPENED;
		value = 0;

		if (novacom->connect_state == NOVACOM_CONNECTED) {
			novacom_enable_ep(&novacom->ep_in);
			novacom_enable_ep(&novacom->ep_out);
			novacom_ep0_send_event(novacom, 1);
		}
	}
	spin_unlock_irqrestore(&novacom->lock, flags);
	mutex_unlock(&novacom_lock);

	return value;
}

static int novacom_ep0_release(struct inode *inode, struct file *fd)
{
	struct f_novacom *novacom;
	unsigned long flags;

	mutex_lock(&novacom_lock);
	novacom = the_novacom;
	if (!novacom) {
		mutex_unlock(&novacom_lock);
		return -ENOENT;
	}

	novacom_disable_ep(&novacom->ep_in);
	novacom_disable_ep(&novacom->ep_out);

	fasync_helper(-1, fd, 0, &novacom->fasync);

	spin_lock_irqsave(&novacom->lock, flags);
	novacom->state = NOVACOM_DEV_CLOSED;
	spin_unlock_irqrestore(&novacom->lock, flags);
	mutex_unlock(&novacom_lock);

	return 0;
}

static __poll_t novacom_ep0_poll(struct file *fd, poll_table *wait)
{
	struct f_novacom *novacom;
	unsigned long flags;
	__poll_t mask = 0;

	mutex_lock(&novacom_lock);
	novacom = the_novacom;
	mutex_unlock(&novacom_lock);

	if (!novacom)
		return EPOLLERR;

	poll_wait(fd, &novacom->ep0_wait, wait);

	spin_lock_irqsave(&novacom->lock, flags);
	if (novacom->ev_next != 0)
		mask = EPOLLIN | EPOLLRDNORM;
	spin_unlock_irqrestore(&novacom->lock, flags);

	return mask;
}

static const struct file_operations novacom_ep0_fops = {
	.owner =	THIS_MODULE,
	.llseek =	noop_llseek,
	.open =		novacom_ep0_open,
	.read =		novacom_ep0_read,
	.fasync =	novacom_ep0_fasync,
	.poll =		novacom_ep0_poll,
	.release =	novacom_ep0_release,
};

static struct miscdevice novacom_ep0_device = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		NOVACOM_DEVNAME_EP0,
	.fops =		&novacom_ep0_fops,
};

/*-------------------------------------------------------------------------*/
/* Connection Management                                                    */
/*-------------------------------------------------------------------------*/

static int novacom_connect(struct f_novacom *novacom)
{
	unsigned long flags;
	int status;

	status = novacom_enable_ep(&novacom->ep_in);
	if (status)
		return status;

	status = novacom_enable_ep(&novacom->ep_out);
	if (status)
		return status;

	spin_lock_irqsave(&novacom->lock, flags);
	novacom->connect_state = NOVACOM_CONNECTED;
	if (novacom->state == NOVACOM_DEV_OPENED)
		novacom_ep0_send_event(novacom, 1);
	spin_unlock_irqrestore(&novacom->lock, flags);

	return 0;
}

static void novacom_disconnect(struct f_novacom *novacom)
{
	unsigned long flags;

	spin_lock_irqsave(&novacom->lock, flags);
	if (novacom->connect_state != NOVACOM_DISCONNECTED) {
		novacom->connect_state = NOVACOM_DISCONNECTED;
		novacom_ep0_send_event(novacom, 0);
	}
	spin_unlock_irqrestore(&novacom->lock, flags);

	novacom_disable_ep(&novacom->ep_out);
	novacom_disable_ep(&novacom->ep_in);
}

/*-------------------------------------------------------------------------*/
/* USB Function Operations                                                  */
/*-------------------------------------------------------------------------*/

static int novacom_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_novacom *novacom = func_to_novacom(f);
	struct usb_string *us;
	struct usb_ep *ep;
	int status;
	int id;

	/* Allocate device-global string IDs and patch iInterface. */
	us = usb_gstrings_attach(cdev, novacom_strings,
				 ARRAY_SIZE(novacom_string_defs));
	if (IS_ERR(us))
		return PTR_ERR(us);
	novacom_interface_desc.iInterface = us[0].id;

	/* Allocate interface ID */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	novacom->data_id = id;
	novacom_interface_desc.bInterfaceNumber = id;

	/* Allocate endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &novacom_fs_in_desc);
	if (!ep) {
		dev_err(&cdev->gadget->dev,
			"novacom: can't autoconfig IN endpoint\n");
		return -ENODEV;
	}
	novacom->ep_in.ep = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &novacom_fs_out_desc);
	if (!ep) {
		dev_err(&cdev->gadget->dev,
			"novacom: can't autoconfig OUT endpoint\n");
		return -ENODEV;
	}
	novacom->ep_out.ep = ep;

	/* Copy endpoint addresses to high-speed descriptors */
	novacom_hs_in_desc.bEndpointAddress =
		novacom_fs_in_desc.bEndpointAddress;
	novacom_hs_out_desc.bEndpointAddress =
		novacom_fs_out_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, novacom_fs_function,
					novacom_hs_function,
					NULL, NULL);
	if (status)
		return status;

	/* Allocate USB requests */
	novacom->ep_in.req = usb_ep_alloc_request(novacom->ep_in.ep, GFP_KERNEL);
	if (!novacom->ep_in.req) {
		dev_err(&cdev->gadget->dev,
			"novacom: can't allocate IN request\n");
		status = -ENOMEM;
		goto fail;
	}

	novacom->ep_out.req = usb_ep_alloc_request(novacom->ep_out.ep, GFP_KERNEL);
	if (!novacom->ep_out.req) {
		dev_err(&cdev->gadget->dev,
			"novacom: can't allocate OUT request\n");
		status = -ENOMEM;
		goto fail;
	}

	novacom->state = NOVACOM_DEV_CLOSED;

	dev_info(&cdev->gadget->dev,
		 "novacom: IN/%s OUT/%s\n",
		 novacom->ep_in.ep->name, novacom->ep_out.ep->name);

	return 0;

fail:
	if (novacom->ep_out.req) {
		usb_ep_free_request(novacom->ep_out.ep, novacom->ep_out.req);
		novacom->ep_out.req = NULL;
	}
	if (novacom->ep_in.req) {
		usb_ep_free_request(novacom->ep_in.ep, novacom->ep_in.req);
		novacom->ep_in.req = NULL;
	}
	usb_free_all_descriptors(f);
	return status;
}

static void novacom_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_novacom *novacom = func_to_novacom(f);
	unsigned long flags;

	spin_lock_irqsave(&novacom->lock, flags);

	if (novacom->ep_out.req) {
		usb_ep_free_request(novacom->ep_out.ep, novacom->ep_out.req);
		novacom->ep_out.req = NULL;
	}
	if (novacom->ep_in.req) {
		usb_ep_free_request(novacom->ep_in.ep, novacom->ep_in.req);
		novacom->ep_in.req = NULL;
	}

	/*
	 * NULL the ep pointers so any racing fop sees a dead function.
	 * misc_deregister() in novacom_free() is the real barrier, but this
	 * keeps the nep->ep != NULL checks honest if the function is left
	 * unbound in ConfigFS with the misc devices still open.
	 */
	novacom->ep_out.ep = NULL;
	novacom->ep_in.ep = NULL;

	novacom->state = NOVACOM_DEV_UNBOUND;

	spin_unlock_irqrestore(&novacom->lock, flags);

	usb_free_all_descriptors(f);
}

static int novacom_set_alt(struct usb_function *f, unsigned int intf,
			   unsigned int alt)
{
	struct f_novacom *novacom = func_to_novacom(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int value;

	/* The novacom interface has no alternate settings. */
	if (alt != 0)
		return -EINVAL;

	/* Activation or reset */
	if (novacom->ep_in.ep->enabled) {
		dev_dbg(&cdev->gadget->dev, "novacom: reset\n");
		novacom_disconnect(novacom);
		usb_ep_disable(novacom->ep_in.ep);
		usb_ep_disable(novacom->ep_out.ep);
	} else {
		dev_dbg(&cdev->gadget->dev, "novacom: activate\n");
	}

	/* Configure endpoints by speed */
	if (!novacom->ep_in.ep->desc) {
		value = config_ep_by_speed(cdev->gadget, f, novacom->ep_in.ep);
		if (value) {
			dev_err(&cdev->gadget->dev,
				"novacom: can't configure IN ep\n");
			return value;
		}
	}
	if (!novacom->ep_out.ep->desc) {
		value = config_ep_by_speed(cdev->gadget, f, novacom->ep_out.ep);
		if (value) {
			dev_err(&cdev->gadget->dev,
				"novacom: can't configure OUT ep\n");
			return value;
		}
	}

	value = usb_ep_enable(novacom->ep_in.ep);
	if (value) {
		dev_err(&cdev->gadget->dev,
			"novacom: can't enable IN ep: %d\n", value);
		return value;
	}

	value = usb_ep_enable(novacom->ep_out.ep);
	if (value) {
		usb_ep_disable(novacom->ep_in.ep);
		dev_err(&cdev->gadget->dev,
			"novacom: can't enable OUT ep: %d\n", value);
		return value;
	}

	novacom_connect(novacom);
	return 0;
}

static void novacom_disable(struct usb_function *f)
{
	struct f_novacom *novacom = func_to_novacom(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	dev_dbg(&cdev->gadget->dev, "novacom: deactivated\n");
	novacom_disconnect(novacom);
	usb_ep_disable(novacom->ep_in.ep);
	usb_ep_disable(novacom->ep_out.ep);
}

/*-------------------------------------------------------------------------*/
/* ConfigFS Interface                                                       */
/*-------------------------------------------------------------------------*/

static inline struct f_novacom_opts *to_f_novacom_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_novacom_opts,
			    func_inst.group);
}

static void novacom_attr_release(struct config_item *item)
{
	struct f_novacom_opts *opts = to_f_novacom_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations novacom_item_ops = {
	.release = novacom_attr_release,
};

static const struct config_item_type novacom_func_type = {
	.ct_item_ops = &novacom_item_ops,
	.ct_owner = THIS_MODULE,
};

static void novacom_free_inst(struct usb_function_instance *f)
{
	struct f_novacom_opts *opts;

	opts = container_of(f, struct f_novacom_opts, func_inst);
	kfree(opts);
}

static struct usb_function_instance *novacom_alloc_inst(void)
{
	struct f_novacom_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	opts->func_inst.free_func_inst = novacom_free_inst;

	config_group_init_type_name(&opts->func_inst.group, "",
				    &novacom_func_type);

	return &opts->func_inst;
}

static void novacom_free(struct usb_function *f)
{
	struct f_novacom *novacom = func_to_novacom(f);

	mutex_lock(&novacom_lock);
	if (the_novacom == novacom) {
		misc_deregister(&novacom_ep_out_device);
		misc_deregister(&novacom_ep_in_device);
		misc_deregister(&novacom_ep0_device);
		the_novacom = NULL;
	}
	mutex_unlock(&novacom_lock);

	kfree(novacom);
}

static struct usb_function *novacom_alloc(struct usb_function_instance *fi)
{
	struct f_novacom *novacom;
	int status;

	novacom = kzalloc(sizeof(*novacom), GFP_KERNEL);
	if (!novacom)
		return ERR_PTR(-ENOMEM);

	/* Initialize endpoints */
	strscpy(novacom->ep_in.name, "ep_in", sizeof(novacom->ep_in.name));
	novacom->ep_in.state = NOVACOM_EP_DISABLED;
	novacom->ep_in.novacom = novacom;
	mutex_init(&novacom->ep_in.lock);
	init_waitqueue_head(&novacom->ep_in.wait);

	strscpy(novacom->ep_out.name, "ep_out", sizeof(novacom->ep_out.name));
	novacom->ep_out.state = NOVACOM_EP_DISABLED;
	novacom->ep_out.novacom = novacom;
	mutex_init(&novacom->ep_out.lock);
	init_waitqueue_head(&novacom->ep_out.wait);

	/* Initialize control state */
	novacom->state = NOVACOM_DEV_UNBOUND;
	novacom->connect_state = NOVACOM_DISCONNECTED;
	spin_lock_init(&novacom->lock);
	init_waitqueue_head(&novacom->ep0_wait);

	/* Set up function */
	novacom->function.name = "novacom";
	novacom->function.strings = novacom_strings;
	novacom->function.bind = novacom_bind;
	novacom->function.unbind = novacom_unbind;
	novacom->function.set_alt = novacom_set_alt;
	novacom->function.disable = novacom_disable;
	novacom->function.free_func = novacom_free;

	/* Register misc devices */
	mutex_lock(&novacom_lock);
	if (the_novacom) {
		mutex_unlock(&novacom_lock);
		kfree(novacom);
		return ERR_PTR(-EBUSY);
	}

	status = misc_register(&novacom_ep0_device);
	if (status) {
		mutex_unlock(&novacom_lock);
		kfree(novacom);
		return ERR_PTR(status);
	}

	status = misc_register(&novacom_ep_in_device);
	if (status) {
		misc_deregister(&novacom_ep0_device);
		mutex_unlock(&novacom_lock);
		kfree(novacom);
		return ERR_PTR(status);
	}

	status = misc_register(&novacom_ep_out_device);
	if (status) {
		misc_deregister(&novacom_ep_in_device);
		misc_deregister(&novacom_ep0_device);
		mutex_unlock(&novacom_lock);
		kfree(novacom);
		return ERR_PTR(status);
	}

	the_novacom = novacom;
	mutex_unlock(&novacom_lock);

	return &novacom->function;
}

DECLARE_USB_FUNCTION_INIT(novacom, novacom_alloc_inst, novacom_alloc);

MODULE_DESCRIPTION("Novacom USB Gadget Function Driver");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
