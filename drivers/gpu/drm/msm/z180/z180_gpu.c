// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024 Herrie <herrie@herrie.org>
 *
 * Z180 2D Graphics Engine driver for Qualcomm MSM8660/APQ8060
 *
 * This driver provides support for the Z180 2D graphics accelerator
 * including command stream submission, ringbuffer management, and
 * synchronization primitives.
 *
 * Based on legacy kgsl_g12 driver from webOS kernel.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "z180_gpu.h"

/*
 * Ringbuffer structure for command stream management
 */
struct z180_ringbuffer {
	void *hostptr;			/* CPU address of ringbuffer */
	dma_addr_t gpuaddr;		/* GPU/DMA address of ringbuffer */
	size_t size;			/* Size of ringbuffer in bytes */
	unsigned int prevctx;		/* Previous context ID */
};

/*
 * Memstore for timestamp storage
 */
struct z180_memstore {
	void *hostptr;			/* CPU address */
	dma_addr_t gpuaddr;		/* GPU/DMA address */
	size_t size;			/* Size in bytes */
};

struct z180_device {
	struct device *dev;
	void __iomem *mmio;
	struct clk *core_clk;
	struct clk *iface_clk;
	struct icc_path *icc_path;
	int irq;
	int id;				/* Device ID (0 or 1) */

	/* Hardware state */
	spinlock_t cmdwin_lock;
	struct mutex mutex;		/* Protects command submission */
	u32 timestamp;			/* Completed timestamp from IRQ */
	u32 current_timestamp;		/* Next timestamp to assign */

	/* Ringbuffer and memstore */
	struct z180_ringbuffer ringbuffer;
	struct z180_memstore memstore;

	/* Wait queue for timestamp completion */
	wait_queue_head_t timestamp_wq;

	/* Miscdevice for userspace access */
	struct miscdevice miscdev;
	bool started;			/* Command stream started */
};

/* Device tracking for miscdevice lookup */
static DEFINE_MUTEX(z180_dev_lock);
static struct z180_device *z180_devices[2];
static int z180_dev_count;

static inline void z180_write(struct z180_device *z180, u32 reg, u32 data)
{
	writel(data, z180->mmio + (reg << 2));
}

static inline u32 z180_read(struct z180_device *z180, u32 reg)
{
	return readl(z180->mmio + (reg << 2));
}

/*
 * The MH registers must be accessed through a 2-step write/read process
 * via the command window. These registers may be accessed from interrupt
 * context, so we use a spinlock.
 */
static void z180_regwrite_mh(struct z180_device *z180, u32 offsetwords, u32 value)
{
	unsigned long flags;
	u32 cmdwinaddr;

	cmdwinaddr = ((Z180_CMDWINDOW_MMU << Z180_CMDWINDOW_TARGET_SHIFT) &
		      Z180_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((offsetwords << Z180_CMDWINDOW_ADDR_SHIFT) &
		       Z180_CMDWINDOW_ADDR_MASK);

	spin_lock_irqsave(&z180->cmdwin_lock, flags);
	z180_write(z180, Z180_VGC_MMUCOMMANDSTREAM >> 2, cmdwinaddr);
	z180_write(z180, Z180_VGC_MMUCOMMANDSTREAM >> 2, value);
	spin_unlock_irqrestore(&z180->cmdwin_lock, flags);
}

static void z180_regread_mh(struct z180_device *z180, u32 offsetwords, u32 *value)
{
	unsigned long flags;

	spin_lock_irqsave(&z180->cmdwin_lock, flags);
	z180_write(z180, Z180_VGC_MH_READ_ADDR >> 2, offsetwords);
	*value = z180_read(z180, Z180_VGC_MH_DATA_ADDR >> 2);
	spin_unlock_irqrestore(&z180->cmdwin_lock, flags);
}

/*
 * Write to VGV3 (2D) registers via command window
 */
static void z180_cmdwindow_write(struct z180_device *z180, u32 addr, u32 data)
{
	unsigned long flags;
	u32 cmdwinaddr;

	cmdwinaddr = ((Z180_CMDWINDOW_2D << Z180_CMDWINDOW_TARGET_SHIFT) &
		      Z180_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((addr << Z180_CMDWINDOW_ADDR_SHIFT) &
		       Z180_CMDWINDOW_ADDR_MASK);

	spin_lock_irqsave(&z180->cmdwin_lock, flags);
	z180_write(z180, Z180_VGC_COMMANDSTREAM >> 2, cmdwinaddr);
	z180_write(z180, Z180_VGC_COMMANDSTREAM >> 2, data);
	spin_unlock_irqrestore(&z180->cmdwin_lock, flags);
}

/*
 * Calculate ringbuffer offset for a given packet index
 */
static inline unsigned int rb_offset(unsigned int index)
{
	return index * sizeof(u32) * Z180_PACKET_SIZE;
}

/*
 * Add a marker packet to the ringbuffer at the given index.
 * The marker packet consists of stream packets that set up the
 * VGV3_LAST register to signal completion.
 */
static void z180_addmarker(struct z180_device *z180, unsigned int index)
{
	u32 *p = (u32 *)((char *)z180->ringbuffer.hostptr + rb_offset(index));

	/* First stream packet - marker command */
	*p++ = Z180_STREAM_PACKET;
	*p++ = Z180_MARKER_CMD | 5;
	*p++ = Z180_VGV3_LAST << 24;
	*p++ = Z180_VGV3_LAST << 24;
	*p++ = Z180_VGV3_LAST << 24;

	/* Second stream packet - continuation */
	*p++ = Z180_STREAM_PACKET;
	*p++ = 5;
	*p++ = Z180_VGV3_LAST << 24;
	*p++ = Z180_VGV3_LAST << 24;
	*p++ = Z180_VGV3_LAST << 24;
}

/*
 * Add a command packet to the ringbuffer at the given index.
 * This follows the marker section and contains the actual command
 * buffer address to execute.
 */
static void z180_addcmd(struct z180_device *z180, unsigned int index,
			u32 cmd, u32 nextcnt)
{
	u32 *p = (u32 *)((char *)z180->ringbuffer.hostptr +
			 rb_offset(index) + (Z180_MARKER_SIZE * sizeof(u32)));

	*p++ = Z180_STREAM_PACKET_CALL;
	*p++ = cmd;
	*p++ = Z180_CALL_CMD | nextcnt;
	*p++ = Z180_VGV3_LAST << 24;
	*p++ = Z180_VGV3_LAST << 24;
}

/*
 * Check if there's room in the ringbuffer for a new submission
 */
static bool z180_room_in_rb(struct z180_device *z180)
{
	int ts_diff;

	ts_diff = z180->current_timestamp - z180->timestamp;
	return ts_diff < Z180_PACKET_COUNT;
}

/*
 * Initialize the command stream (allocate ringbuffer and memstore)
 */
static int z180_cmdstream_init(struct z180_device *z180)
{
	/* Allocate ringbuffer */
	z180->ringbuffer.size = Z180_RB_SIZE;
	z180->ringbuffer.hostptr = dma_alloc_coherent(z180->dev,
						      z180->ringbuffer.size,
						      &z180->ringbuffer.gpuaddr,
						      GFP_KERNEL);
	if (!z180->ringbuffer.hostptr) {
		dev_err(z180->dev, "Failed to allocate ringbuffer\n");
		return -ENOMEM;
	}

	memset(z180->ringbuffer.hostptr, 0, z180->ringbuffer.size);
	z180->ringbuffer.prevctx = Z180_INVALID_CONTEXT;

	/* Allocate memstore for timestamp storage */
	z180->memstore.size = Z180_MEMSTORE_SIZE;
	z180->memstore.hostptr = dma_alloc_coherent(z180->dev,
						    z180->memstore.size,
						    &z180->memstore.gpuaddr,
						    GFP_KERNEL);
	if (!z180->memstore.hostptr) {
		dev_err(z180->dev, "Failed to allocate memstore\n");
		dma_free_coherent(z180->dev, z180->ringbuffer.size,
				  z180->ringbuffer.hostptr,
				  z180->ringbuffer.gpuaddr);
		return -ENOMEM;
	}

	memset(z180->memstore.hostptr, 0, z180->memstore.size);

	dev_info(z180->dev, "Command stream initialized: rb@0x%llx mem@0x%llx\n",
		 (u64)z180->ringbuffer.gpuaddr, (u64)z180->memstore.gpuaddr);

	return 0;
}

/*
 * Start the command stream engine
 */
static int z180_cmdstream_start(struct z180_device *z180)
{
	u32 cmd;

	if (z180->started)
		return 0;

	z180->timestamp = 0;
	z180->current_timestamp = 0;

	/* Add initial marker at index 0 */
	z180_addmarker(z180, 0);

	/* Set VGV3 mode */
	z180_cmdwindow_write(z180, Z180_VGV3_MODE, 4);

	/* Set ringbuffer address */
	z180_cmdwindow_write(z180, Z180_VGV3_NEXTADDR,
			     z180->ringbuffer.gpuaddr);

	/* Configure jump command */
	cmd = Z180_VGV3_NEXTCMD_JUMP << Z180_VGV3_NEXTCMD_FSHIFT;
	z180_cmdwindow_write(z180, Z180_VGV3_NEXTCMD, cmd | 5);

	/* Set memstore address for timestamp writes */
	z180_cmdwindow_write(z180, Z180_VGV3_WRITEADDR, z180->memstore.gpuaddr);

	/* Trigger initial marker */
	cmd = (1 & Z180_VGV3_CONTROL_MARKADD_FMASK) << Z180_VGV3_CONTROL_MARKADD_FSHIFT;
	z180_cmdwindow_write(z180, Z180_VGV3_CONTROL, cmd);
	z180_cmdwindow_write(z180, Z180_VGV3_CONTROL, 0);

	z180->started = true;
	dev_info(z180->dev, "Command stream started\n");

	return 0;
}

/*
 * Close the command stream and free resources
 */
static void z180_cmdstream_close(struct z180_device *z180)
{
	z180->started = false;

	if (z180->memstore.hostptr) {
		dma_free_coherent(z180->dev, z180->memstore.size,
				  z180->memstore.hostptr,
				  z180->memstore.gpuaddr);
		z180->memstore.hostptr = NULL;
	}

	if (z180->ringbuffer.hostptr) {
		dma_free_coherent(z180->dev, z180->ringbuffer.size,
				  z180->ringbuffer.hostptr,
				  z180->ringbuffer.gpuaddr);
		z180->ringbuffer.hostptr = NULL;
	}
}

/*
 * Wait for a timestamp to complete
 */
static int z180_wait_timestamp(struct z180_device *z180, u32 timestamp,
			       unsigned long timeout_ms)
{
	int ret;

	ret = wait_event_interruptible_timeout(z180->timestamp_wq,
					       (int)(z180->timestamp - timestamp) >= 0,
					       msecs_to_jiffies(timeout_ms));
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

/*
 * Submit a command buffer for execution
 */
static int z180_submit_cmd(struct z180_device *z180, u32 gpuaddr,
			   u32 sizedwords, u32 flags, u32 *timestamp)
{
	unsigned int index, nextindex;
	u32 cmd;
	u32 cnt = 5;
	u32 ofs = Z180_PACKETSIZE_STATESTREAM * sizeof(u32);
	int ret;

	(void)sizedwords; /* Used for chaining in full implementation */

	mutex_lock(&z180->mutex);

	/* Ensure command stream is started */
	if (!z180->started) {
		ret = z180_cmdstream_start(z180);
		if (ret) {
			mutex_unlock(&z180->mutex);
			return ret;
		}
	}

	/* Check for context switch */
	if (flags & Z180_SUBMIT_FLAGS_CTX_SWITCH) {
		cnt = Z180_PACKETSIZE_STATESTREAM;
		ofs = 0;
	}

	/* Wait for room in ringbuffer */
	ret = wait_event_interruptible_timeout(z180->timestamp_wq,
					       z180_room_in_rb(z180),
					       msecs_to_jiffies(Z180_WAIT_TIMEOUT_MS));
	if (ret < 0) {
		mutex_unlock(&z180->mutex);
		return ret;
	}
	if (ret == 0) {
		mutex_unlock(&z180->mutex);
		return -ETIMEDOUT;
	}

	/* Calculate indices */
	index = z180->current_timestamp % Z180_PACKET_COUNT;
	z180->current_timestamp++;
	nextindex = z180->current_timestamp % Z180_PACKET_COUNT;
	*timestamp = z180->current_timestamp;

	/* Add command to ringbuffer */
	z180_addcmd(z180, index, gpuaddr + ofs, cnt);

	/* Add marker for next slot */
	z180_addmarker(z180, nextindex);

	/*
	 * Note: In a full implementation with user buffer chaining, we would
	 * write the next ringbuffer address to the end of the user's command
	 * buffer. For now, we rely on the marker/call mechanism.
	 */

	/* Memory barrier before triggering hardware */
	wmb();

	/* Trigger the hardware to process commands */
	cmd = (2 & Z180_VGV3_CONTROL_MARKADD_FMASK) << Z180_VGV3_CONTROL_MARKADD_FSHIFT;
	z180_cmdwindow_write(z180, Z180_VGV3_CONTROL, cmd);
	z180_cmdwindow_write(z180, Z180_VGV3_CONTROL, 0);

	mutex_unlock(&z180->mutex);

	return 0;
}

static irqreturn_t z180_irq_handler(int irq, void *data)
{
	struct z180_device *z180 = data;
	u32 status;
	int count;

	status = z180_read(z180, Z180_VGC_IRQSTATUS >> 2);

	if (!(status & (Z180_IRQ_MH_MASK | Z180_IRQ_G2D_MASK | Z180_IRQ_FIFO_MASK)))
		return IRQ_NONE;

	/* Acknowledge interrupts */
	z180_write(z180, Z180_VGC_IRQSTATUS >> 2,
		   status & (Z180_IRQ_MH_MASK | Z180_IRQ_G2D_MASK | Z180_IRQ_FIFO_MASK));

	if (status & Z180_IRQ_FIFO_MASK)
		dev_err(z180->dev, "Z180 FIFO interrupt\n");

	if (status & Z180_IRQ_MH_MASK) {
		u32 mh_status;

		z180_regread_mh(z180, Z180_MH_INTERRUPT_STATUS, &mh_status);

		if (mh_status & Z180_MH_INT_AXI_READ_ERROR)
			dev_err(z180->dev, "Z180 AXI read error\n");
		if (mh_status & Z180_MH_INT_AXI_WRITE_ERROR)
			dev_err(z180->dev, "Z180 AXI write error\n");
		if (mh_status & Z180_MH_INT_MMU_PAGE_FAULT)
			dev_err(z180->dev, "Z180 MMU page fault\n");

		z180_regwrite_mh(z180, Z180_MH_INTERRUPT_CLEAR, mh_status);
	}

	if (status & Z180_IRQ_G2D_MASK) {
		/* Read active count register */
		count = z180_read(z180, Z180_VGC_IRQ_ACTIVE_CNT >> 2);
		count = (count >> 8) & 0xff;
		z180->timestamp += count;

		/* Wake up waiters */
		wake_up_interruptible(&z180->timestamp_wq);
	}

	return IRQ_HANDLED;
}

static int z180_hw_init(struct z180_device *z180)
{
	/* Set up MH arbiter */
	z180_regwrite_mh(z180, Z180_MH_ARBITER_CONFIG, Z180_CFG_MHARB);
	z180_regwrite_mh(z180, Z180_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030F27);
	z180_regwrite_mh(z180, Z180_MH_CLNT_INTF_CTRL_CONFIG2, 0x004B274F);

	/* Enable interrupts */
	z180_write(z180, Z180_VGC_IRQENABLE >> 2, 0x3);

	dev_dbg(z180->dev, "Z180 hardware initialized\n");

	return 0;
}

/*
 * Miscdevice file operations for userspace access
 */
static int z180_open(struct inode *inode, struct file *file)
{
	struct z180_device *z180 = container_of(file->private_data,
						struct z180_device, miscdev);

	pm_runtime_get_sync(z180->dev);
	file->private_data = z180;

	return 0;
}

static int z180_release(struct inode *inode, struct file *file)
{
	struct z180_device *z180 = file->private_data;

	pm_runtime_put(z180->dev);

	return 0;
}

static long z180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct z180_device *z180 = file->private_data;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	switch (cmd) {
	case Z180_IOCTL_SUBMIT: {
		struct z180_submit_cmd submit;

		if (copy_from_user(&submit, argp, sizeof(submit)))
			return -EFAULT;

		ret = z180_submit_cmd(z180, submit.gpuaddr, submit.sizedwords,
				      submit.flags, &submit.timestamp);
		if (ret)
			return ret;

		if (copy_to_user(argp, &submit, sizeof(submit)))
			return -EFAULT;

		break;
	}

	case Z180_IOCTL_WAIT: {
		struct z180_wait_timestamp wait;

		if (copy_from_user(&wait, argp, sizeof(wait)))
			return -EFAULT;

		ret = z180_wait_timestamp(z180, wait.timestamp, wait.timeout_ms);
		break;
	}

	case Z180_IOCTL_INFO: {
		struct z180_device_info info;

		info.device_id = z180->id;
		info.chip_id = 0; /* Could read from hardware if available */
		info.timestamp = z180->timestamp;
		info.pending = z180->current_timestamp - z180->timestamp;

		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;

		break;
	}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations z180_fops = {
	.owner = THIS_MODULE,
	.open = z180_open,
	.release = z180_release,
	.unlocked_ioctl = z180_ioctl,
	.compat_ioctl = z180_ioctl,
};

static int z180_runtime_suspend(struct device *dev)
{
	struct z180_device *z180 = dev_get_drvdata(dev);

	/* Disable interrupts */
	z180_write(z180, Z180_VGC_IRQENABLE >> 2, 0);

	clk_disable_unprepare(z180->core_clk);
	clk_disable_unprepare(z180->iface_clk);

	/* Release interconnect bandwidth */
	icc_set_bw(z180->icc_path, 0, 0);

	return 0;
}

static int z180_runtime_resume(struct device *dev)
{
	struct z180_device *z180 = dev_get_drvdata(dev);
	int ret;

	/* Request interconnect bandwidth (128 MB/s average, 256 MB/s peak) */
	ret = icc_set_bw(z180->icc_path, 128000, 256000);
	if (ret) {
		dev_err(dev, "Failed to set interconnect bandwidth: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(z180->iface_clk);
	if (ret) {
		dev_err(dev, "Failed to enable iface clock: %d\n", ret);
		icc_set_bw(z180->icc_path, 0, 0);
		return ret;
	}

	ret = clk_prepare_enable(z180->core_clk);
	if (ret) {
		dev_err(dev, "Failed to enable core clock: %d\n", ret);
		clk_disable_unprepare(z180->iface_clk);
		icc_set_bw(z180->icc_path, 0, 0);
		return ret;
	}

	ret = z180_hw_init(z180);
	if (ret) {
		clk_disable_unprepare(z180->core_clk);
		clk_disable_unprepare(z180->iface_clk);
		icc_set_bw(z180->icc_path, 0, 0);
		return ret;
	}

	/* Restart command stream if it was running */
	if (z180->started) {
		z180->started = false;
		z180_cmdstream_start(z180);
	}

	return 0;
}

static int z180_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct z180_device *z180;
	char name[16];
	int ret;

	z180 = devm_kzalloc(dev, sizeof(*z180), GFP_KERNEL);
	if (!z180)
		return -ENOMEM;

	z180->dev = dev;
	spin_lock_init(&z180->cmdwin_lock);
	mutex_init(&z180->mutex);
	init_waitqueue_head(&z180->timestamp_wq);
	platform_set_drvdata(pdev, z180);

	/* Assign device ID */
	mutex_lock(&z180_dev_lock);
	if (z180_dev_count >= 2) {
		mutex_unlock(&z180_dev_lock);
		return -ENODEV;
	}
	z180->id = z180_dev_count;
	z180_devices[z180_dev_count++] = z180;
	mutex_unlock(&z180_dev_lock);

	/* Map registers */
	z180->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(z180->mmio)) {
		ret = PTR_ERR(z180->mmio);
		goto err_dev;
	}

	/* Get clocks */
	z180->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(z180->core_clk)) {
		dev_err(dev, "Failed to get core clock\n");
		ret = PTR_ERR(z180->core_clk);
		goto err_dev;
	}

	z180->iface_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(z180->iface_clk)) {
		dev_err(dev, "Failed to get iface clock\n");
		ret = PTR_ERR(z180->iface_clk);
		goto err_dev;
	}

	/* Get interconnect path (optional) */
	z180->icc_path = devm_of_icc_get(dev, "gfx-mem");
	if (IS_ERR(z180->icc_path)) {
		ret = PTR_ERR(z180->icc_path);
		if (ret != -ENODATA) {
			dev_err(dev, "Failed to get interconnect path: %d\n", ret);
			goto err_dev;
		}
		/* No interconnect specified, continue without it */
		z180->icc_path = NULL;
	}

	/* Get IRQ */
	z180->irq = platform_get_irq(pdev, 0);
	if (z180->irq < 0) {
		ret = z180->irq;
		goto err_dev;
	}

	ret = devm_request_irq(dev, z180->irq, z180_irq_handler,
			       IRQF_TRIGGER_HIGH, "z180", z180);
	if (ret) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		goto err_dev;
	}

	/* Set DMA mask */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Failed to set DMA mask: %d\n", ret);
		goto err_dev;
	}

	/* Initialize command stream */
	ret = z180_cmdstream_init(z180);
	if (ret)
		goto err_dev;

	/* Enable runtime PM */
	pm_runtime_enable(dev);

	/* Initialize hardware */
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err(dev, "Failed to resume device: %d\n", ret);
		goto err_cmdstream;
	}

	/* Register miscdevice */
	snprintf(name, sizeof(name), "z180-%d", z180->id);
	z180->miscdev.minor = MISC_DYNAMIC_MINOR;
	z180->miscdev.name = devm_kstrdup(dev, name, GFP_KERNEL);
	z180->miscdev.fops = &z180_fops;
	z180->miscdev.parent = dev;

	ret = misc_register(&z180->miscdev);
	if (ret) {
		dev_err(dev, "Failed to register miscdevice: %d\n", ret);
		pm_runtime_put(dev);
		goto err_cmdstream;
	}

	dev_info(dev, "Z180 2D GPU %d probed: /dev/%s\n", z180->id, name);

	pm_runtime_put(dev);

	return 0;

err_cmdstream:
	pm_runtime_disable(dev);
	z180_cmdstream_close(z180);
err_dev:
	mutex_lock(&z180_dev_lock);
	z180_devices[z180->id] = NULL;
	z180_dev_count--;
	mutex_unlock(&z180_dev_lock);
	return ret;
}

static void z180_remove(struct platform_device *pdev)
{
	struct z180_device *z180 = platform_get_drvdata(pdev);

	misc_deregister(&z180->miscdev);
	pm_runtime_disable(&pdev->dev);
	z180_cmdstream_close(z180);

	mutex_lock(&z180_dev_lock);
	z180_devices[z180->id] = NULL;
	z180_dev_count--;
	mutex_unlock(&z180_dev_lock);
}

static const struct dev_pm_ops z180_pm_ops = {
	SET_RUNTIME_PM_OPS(z180_runtime_suspend, z180_runtime_resume, NULL)
};

static const struct of_device_id z180_of_match[] = {
	{ .compatible = "qcom,msm8660-z180" },
	{ }
};
MODULE_DEVICE_TABLE(of, z180_of_match);

static struct platform_driver z180_driver = {
	.probe = z180_probe,
	.remove = z180_remove,
	.driver = {
		.name = "qcom-z180",
		.pm = &z180_pm_ops,
		.of_match_table = z180_of_match,
	},
};

module_platform_driver(z180_driver);

MODULE_DESCRIPTION("Qualcomm Z180 2D Graphics Engine Driver");
MODULE_LICENSE("GPL");
