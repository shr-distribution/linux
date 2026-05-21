// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM8660 / APQ8060 MPM (MSM Power Manager) wake-source driver
 *
 * The MSM Power Manager is an always-on hardware block that keeps a small
 * set of wake sources alive while the application processor is powered
 * down for cpuidle Power Collapse or suspend-to-RAM. On MSM8660 / APQ8060
 * the vMPM (virtual MPM) registers live INSIDE the RPM's 4 KB control
 * block at:
 *
 *   request (control) regs: RPM_BASE + 0x9d8   (ENABLE, DETECT_CTL,
 *                                                POLARITY, CLEAR)
 *   status regs:            RPM_BASE + 0xdf8   (PENDING)
 *
 * with each "register" being a window of two 32-bit slots (since MPM
 * exposes 64 wake pins / IRQs).
 *
 * The mainline qcom-mpm driver (drivers/irqchip/irq-qcom-mpm.c) assumes:
 *   - a dedicated MPM SRAM region separate from RPM;
 *   - a mailbox controller (IPCC) for waking MPM;
 *   - early init via IRQCHIP_DECLARE.
 *
 * None of those hold on MSM8660:
 *   - vMPM regs live inside RPM's control block;
 *   - wake notification is a raw MMIO write to GCC + 0x008 (bit 1);
 *   - IRQCHIP_DECLARE runs before platform devices exist, so
 *     of_find_device_by_node() returns NULL and the init silently hangs.
 *
 * This driver instead replicates the 2.6.35-palm `arch/arm/mach-msm/mpm.c`
 * mechanism as a regular platform driver: probes after platform infra is
 * ready, accesses RPM via a syscon phandle (non-exclusive), and triggers
 * the MPM via a second syscon phandle to the GCC block. No DT resource
 * conflict with the RPM driver.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <soc/qcom/msm8660-mpm.h>

#define MPM_REG_ENABLE		0
#define MPM_REG_DETECT_CTL	1
#define MPM_REG_POLARITY	2
#define MPM_REG_CLEAR		3

#define MPM_REG_STATUS_PENDING	0

/* Each MPM register is a window of REG_STRIDE 32-bit slots. */
#define MPM_REG_STRIDE		2	/* DIV_ROUND_UP(64 pins, 32) */

struct msm8660_mpm {
	struct device *dev;

	/* Regmap to RPM control block; vMPM lives at known offsets within. */
	struct regmap *rpm_regmap;
	u32 request_offset;	/* offset to control regs (typically 0x9d8) */
	u32 status_offset;	/* offset to status regs (typically 0xdf8)  */

	/* IPC trigger: syscon + offset + bit (matches qcom,ipc binding). */
	struct regmap *ipc_regmap;
	u32 ipc_offset;
	u32 ipc_bit;

	/* IPC IRQ raised by MPM when a wake source fires while CPU asleep. */
	int ipc_irq;

	/* In-memory shadow of the four control regs (2x u32 each = 64 pins). */
	u32 enabled_mask[MPM_REG_STRIDE];
	u32 wake_mask[MPM_REG_STRIDE];
	u32 detect_ctl[MPM_REG_STRIDE];
	u32 polarity[MPM_REG_STRIDE];

	raw_spinlock_t lock;
};

/*
 * Global pointer for cross-driver consumers (e.g. the mmci SDC4 wakeup
 * shim). Single MPM instance per SoC; we don't expect more than one.
 */
static struct msm8660_mpm *msm8660_mpm_global;

static u32 mpm_request_reg_offset(struct msm8660_mpm *mpm,
				  unsigned int reg, unsigned int subreg)
{
	return mpm->request_offset + (reg * MPM_REG_STRIDE + subreg) * 4;
}

static u32 mpm_status_reg_offset(struct msm8660_mpm *mpm,
				 unsigned int reg, unsigned int subreg)
{
	return mpm->status_offset + (reg * MPM_REG_STRIDE + subreg) * 4;
}

static int mpm_write_reg(struct msm8660_mpm *mpm,
			 unsigned int reg, unsigned int subreg, u32 val)
{
	return regmap_write(mpm->rpm_regmap,
			    mpm_request_reg_offset(mpm, reg, subreg), val);
}

static int mpm_read_status(struct msm8660_mpm *mpm,
			   unsigned int reg, unsigned int subreg, u32 *val)
{
	return regmap_read(mpm->rpm_regmap,
			   mpm_status_reg_offset(mpm, reg, subreg), val);
}

static void mpm_trigger_ipc(struct msm8660_mpm *mpm)
{
	/*
	 * The MPM watches a single bit in the GCC block. Writing the bit
	 * once is enough to nudge MPM to re-read its request registers
	 * (it's an edge-triggered handshake — RPM/MPM clears the bit
	 * internally after acting).
	 */
	regmap_write(mpm->ipc_regmap, mpm->ipc_offset, BIT(mpm->ipc_bit));
}

/*
 * Push the current shadow state into MPM hardware and ping the IPC bit.
 * Called from suspend / cpuidle PC entry paths.
 *
 * @wake_only: if true, write the wake_mask (used during suspend); if
 *             false, write the full enabled_mask (used during cpuidle
 *             PC entry, where all enabled IRQs should remain active).
 */
static void msm8660_mpm_commit(struct msm8660_mpm *mpm, bool wake_only)
{
	const u32 *mask = wake_only ? mpm->wake_mask : mpm->enabled_mask;
	u32 readback;
	int i;

	for (i = 0; i < MPM_REG_STRIDE; i++) {
		mpm_write_reg(mpm, MPM_REG_ENABLE,     i, mask[i]);
		mpm_write_reg(mpm, MPM_REG_DETECT_CTL, i, mpm->detect_ctl[i]);
		mpm_write_reg(mpm, MPM_REG_POLARITY,   i, mpm->polarity[i]);
		mpm_write_reg(mpm, MPM_REG_CLEAR,      i, 0xffffffff);
	}

	/* Posted-write flush: read any MPM register before triggering IPC. */
	regmap_read(mpm->rpm_regmap,
		    mpm_request_reg_offset(mpm, MPM_REG_ENABLE, 0), &readback);

	mpm_trigger_ipc(mpm);
}

/**
 * msm8660_mpm_set_pin_wake() - mark an MPM pin as wake-capable
 * @mpm: handle from msm8660_mpm_get()
 * @pin: MPM pin index (0..63); pre-defined wake pins on tenderloin are
 *       SDC4_DAT1 = 23, SDC4_DAT3 = 24.
 * @on:  true to mark wake, false to clear.
 *
 * Updates the in-memory mask only; the actual MPM hardware is programmed
 * on suspend entry via msm8660_mpm_enter_sleep().
 */
int msm8660_mpm_set_pin_wake(struct msm8660_mpm *mpm, unsigned int pin,
			     bool on)
{
	unsigned int idx, shift;
	unsigned long flags;

	if (!mpm || pin >= MSM8660_MPM_NR_PINS)
		return -EINVAL;

	idx = pin / 32;
	shift = pin % 32;

	raw_spin_lock_irqsave(&mpm->lock, flags);
	if (on)
		mpm->wake_mask[idx] |= BIT(shift);
	else
		mpm->wake_mask[idx] &= ~BIT(shift);
	raw_spin_unlock_irqrestore(&mpm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_set_pin_wake);

/**
 * msm8660_mpm_enable_pin() - enable/disable an MPM pin for monitoring
 * @mpm: handle from msm8660_mpm_get()
 * @pin: MPM pin index
 * @enable: true to monitor pin during PC entry, false to ignore.
 */
int msm8660_mpm_enable_pin(struct msm8660_mpm *mpm, unsigned int pin,
			   bool enable)
{
	unsigned int idx, shift;
	unsigned long flags;

	if (!mpm || pin >= MSM8660_MPM_NR_PINS)
		return -EINVAL;

	idx = pin / 32;
	shift = pin % 32;

	raw_spin_lock_irqsave(&mpm->lock, flags);
	if (enable)
		mpm->enabled_mask[idx] |= BIT(shift);
	else
		mpm->enabled_mask[idx] &= ~BIT(shift);
	raw_spin_unlock_irqrestore(&mpm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_enable_pin);

/**
 * msm8660_mpm_set_pin_type() - set trigger type for an MPM pin
 * @mpm: handle from msm8660_mpm_get()
 * @pin: MPM pin index
 * @flow_type: standard IRQ_TYPE_* constants
 *
 * On MSM8660 the trigger config is split across DETECT_CTL (edge vs
 * level) and POLARITY (rising/high vs falling/low).
 */
int msm8660_mpm_set_pin_type(struct msm8660_mpm *mpm, unsigned int pin,
			     unsigned int flow_type)
{
	unsigned int idx, shift;
	unsigned long flags;
	bool edge, polarity;

	if (!mpm || pin >= MSM8660_MPM_NR_PINS)
		return -EINVAL;

	idx = pin / 32;
	shift = pin % 32;

	edge = !!(flow_type & IRQ_TYPE_EDGE_BOTH);
	polarity = !!(flow_type & (IRQ_TYPE_EDGE_RISING |
				   IRQ_TYPE_LEVEL_HIGH));

	raw_spin_lock_irqsave(&mpm->lock, flags);

	if (edge)
		mpm->detect_ctl[idx] |= BIT(shift);
	else
		mpm->detect_ctl[idx] &= ~BIT(shift);

	if (polarity)
		mpm->polarity[idx] |= BIT(shift);
	else
		mpm->polarity[idx] &= ~BIT(shift);

	raw_spin_unlock_irqrestore(&mpm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_set_pin_type);

/**
 * msm8660_mpm_enter_sleep() - prepare MPM for low-power state
 * @mpm: handle
 * @from_idle: true for cpuidle PC entry (use enabled_mask),
 *             false for suspend-to-RAM (use wake_mask only).
 *
 * Must be called with IRQs disabled by the caller (idle/suspend path).
 */
void msm8660_mpm_enter_sleep(struct msm8660_mpm *mpm, bool from_idle)
{
	if (!mpm)
		return;

	msm8660_mpm_commit(mpm, !from_idle);
}
EXPORT_SYMBOL_GPL(msm8660_mpm_enter_sleep);

/**
 * msm8660_mpm_exit_sleep() - re-arm MPM after resume
 * @mpm: handle
 * @from_idle: true for cpuidle PC exit, false for suspend-to-RAM resume.
 *
 * Reads pending status (so MPM doesn't hold a stale wake), then re-arms
 * MPM with the enabled_mask (cpuidle) or zero (suspend-to-RAM, until
 * userspace re-arms wakeup sources).
 */
void msm8660_mpm_exit_sleep(struct msm8660_mpm *mpm, bool from_idle)
{
	u32 pending;
	int i;

	if (!mpm)
		return;

	for (i = 0; i < MPM_REG_STRIDE; i++) {
		if (mpm_read_status(mpm, MPM_REG_STATUS_PENDING, i, &pending) == 0)
			dev_dbg(mpm->dev, "exit_sleep: pending[%d]=0x%08x\n",
				i, pending);
	}

	msm8660_mpm_commit(mpm, !from_idle);
}
EXPORT_SYMBOL_GPL(msm8660_mpm_exit_sleep);

/**
 * msm8660_mpm_get() - get a handle to the MPM device from a phandle
 * @np:  device node containing the phandle property
 * @propname: phandle property name (e.g. "qcom,mpm-wake")
 *
 * Returns the singleton MPM handle on success, or an ERR_PTR.
 * Consumer drivers (e.g. mmci) call this in their probe to obtain the
 * handle, then use msm8660_mpm_set_pin_wake() etc.
 */
struct msm8660_mpm *msm8660_mpm_get(struct device_node *np,
				    const char *propname)
{
	struct device_node *mpm_np;

	if (!msm8660_mpm_global)
		return ERR_PTR(-EPROBE_DEFER);

	if (np && propname) {
		mpm_np = of_parse_phandle(np, propname, 0);
		if (!mpm_np)
			return ERR_PTR(-ENOENT);
		of_node_put(mpm_np);
		/*
		 * We only support a single MPM instance per SoC; any
		 * phandle that lands here is treated as a request for
		 * that one. Verifying the node matches is left to
		 * consumers that care.
		 */
	}

	return msm8660_mpm_global;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_get);

/*
 * IPC IRQ handler. MPM raises this IRQ when one of the enabled wake
 * sources fires while we're in PC. The actual GIC IRQ for the originating
 * source will fire as well once the CPU comes back up, so we don't need
 * to dispatch handlers here — we just need to clear the pending bits
 * so MPM doesn't keep nagging.
 */
static irqreturn_t msm8660_mpm_ipc_irq(int irq, void *data)
{
	struct msm8660_mpm *mpm = data;
	u32 pending;
	int i;

	for (i = 0; i < MPM_REG_STRIDE; i++) {
		if (mpm_read_status(mpm, MPM_REG_STATUS_PENDING, i, &pending) == 0
		    && pending) {
			dev_dbg(mpm->dev, "ipc_irq: pending[%d]=0x%08x\n",
				i, pending);
			mpm_write_reg(mpm, MPM_REG_CLEAR, i, pending);
		}
	}

	mpm_trigger_ipc(mpm);

	return IRQ_HANDLED;
}

static int msm8660_mpm_parse_ipc(struct msm8660_mpm *mpm,
				 struct device_node *np)
{
	struct of_phandle_args args;
	int ret;

	/*
	 * qcom,ipc = <&syscon offset bit>
	 * Same format the existing legacy RPM and SMSM bindings use, so
	 * we parse it identically.
	 */
	ret = of_parse_phandle_with_fixed_args(np, "qcom,ipc", 2, 0, &args);
	if (ret) {
		dev_err(mpm->dev, "missing or malformed qcom,ipc\n");
		return ret;
	}

	mpm->ipc_regmap = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(mpm->ipc_regmap)) {
		dev_err(mpm->dev, "qcom,ipc syscon not found: %ld\n",
			PTR_ERR(mpm->ipc_regmap));
		return PTR_ERR(mpm->ipc_regmap);
	}

	mpm->ipc_offset = args.args[0];
	mpm->ipc_bit = args.args[1];

	return 0;
}

static int msm8660_mpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct msm8660_mpm *mpm;
	int ret;

	if (msm8660_mpm_global) {
		dev_err(dev, "only one MPM instance is supported\n");
		return -EBUSY;
	}

	mpm = devm_kzalloc(dev, sizeof(*mpm), GFP_KERNEL);
	if (!mpm)
		return -ENOMEM;

	mpm->dev = dev;
	raw_spin_lock_init(&mpm->lock);

	mpm->rpm_regmap = syscon_regmap_lookup_by_phandle(np, "qcom,rpm-syscon");
	if (IS_ERR(mpm->rpm_regmap)) {
		ret = PTR_ERR(mpm->rpm_regmap);
		dev_err(dev, "qcom,rpm-syscon lookup failed: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "qcom,mpm-request-offset",
				   &mpm->request_offset);
	if (ret) {
		dev_err(dev, "missing qcom,mpm-request-offset: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "qcom,mpm-status-offset",
				   &mpm->status_offset);
	if (ret) {
		dev_err(dev, "missing qcom,mpm-status-offset: %d\n", ret);
		return ret;
	}

	ret = msm8660_mpm_parse_ipc(mpm, np);
	if (ret)
		return ret;

	mpm->ipc_irq = platform_get_irq(pdev, 0);
	if (mpm->ipc_irq < 0)
		return mpm->ipc_irq;

	ret = devm_request_irq(dev, mpm->ipc_irq, msm8660_mpm_ipc_irq,
			       IRQF_TRIGGER_RISING, "msm8660-mpm", mpm);
	if (ret) {
		dev_err(dev, "failed to request IPC IRQ %d: %d\n",
			mpm->ipc_irq, ret);
		return ret;
	}

	ret = enable_irq_wake(mpm->ipc_irq);
	if (ret) {
		dev_warn(dev, "could not mark IPC IRQ as wakeup: %d\n", ret);
		/* not fatal; the SoC can still wake via other paths */
	}

	platform_set_drvdata(pdev, mpm);
	msm8660_mpm_global = mpm;

	dev_info(dev,
		 "ready: rpm-syscon req@0x%x sts@0x%x ipc-bit=%u ipc-irq=%d\n",
		 mpm->request_offset, mpm->status_offset, mpm->ipc_bit,
		 mpm->ipc_irq);

	return 0;
}

static void msm8660_mpm_remove(struct platform_device *pdev)
{
	struct msm8660_mpm *mpm = platform_get_drvdata(pdev);

	disable_irq_wake(mpm->ipc_irq);
	msm8660_mpm_global = NULL;
}

static const struct of_device_id msm8660_mpm_of_match[] = {
	{ .compatible = "qcom,msm8660-mpm" },
	{ }
};
MODULE_DEVICE_TABLE(of, msm8660_mpm_of_match);

static struct platform_driver msm8660_mpm_driver = {
	.driver = {
		.name = "msm8660-mpm",
		.of_match_table = msm8660_mpm_of_match,
	},
	.probe = msm8660_mpm_probe,
	.remove = msm8660_mpm_remove,
};
module_platform_driver(msm8660_mpm_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660 MPM wake-source driver");
MODULE_LICENSE("GPL");
