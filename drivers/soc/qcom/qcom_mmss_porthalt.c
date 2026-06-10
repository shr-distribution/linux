// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8x60-family MMSS NoC master port halt helper.
 *
 * The MMSS (multimedia subsystem) on MSM8260/MSM8660/APQ8060 sits behind
 * a NoC fabric whose master ports must be quiesced before the rail that
 * backs each master collapses. The only mechanism the SoC exposes is an
 * RPM IPC (QCOM_RPM_MM_FABRIC_HALT) that flips a 14-bit port-halt mask.
 * Later silicon (MSM8960+) moved AXI quiescence in-band via the GDSC
 * power-down-complete state machine; on MSM8x60 the kernel has to drive
 * the halt explicitly.
 *
 * Multiple consumers want to assert a port halt:
 *   - The MMCC footswitch power_off path on cold-collapse (runtime PM)
 *   - The per-subsystem .suspend_late hooks on mdp4 / adreno / camss-vfe
 *     / vidc that have to halt the port BEFORE the genpd suspend_noirq
 *     cascade (because qcom_rpm_write needs IRQs enabled — see context
 *     comment on qcom_mmss_port_halt())
 *
 * This helper refcounts per-port-bit so back-to-back asserts from
 * different consumers are idempotent at the silicon side; only refcount
 * transitions (0->1 on halt, 1->0 on unhalt) translate into an RPM IPC.
 *
 * Copyright (c) 2026, Herman van Hazendonk <github.com@herrie.org>
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/hardirq.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/mfd/qcom_rpm.h>
#include <linux/soc/qcom/qcom_mmss_porthalt.h>

#include <dt-bindings/mfd/qcom-rpm.h>

/*
 * MSM8x60 MMSS fabric has 14 master ports (MSM_BUS_MASTER_* enum in the
 * downstream msm_bus_board_8660.c). The QCOM_RPM_MM_FABRIC_HALT mask is
 * 14 bits wide.
 */
#define MMSS_FABRIC_NR_PORTS	14
#define MMSS_FABRIC_PORT_MASK	GENMASK(MMSS_FABRIC_NR_PORTS - 1, 0)

static atomic_t port_halt_count[MMSS_FABRIC_NR_PORTS];

/*
 * Cached qcom_rpm handle. Set once by qcom_mmss_porthalt_register_rpm()
 * from the MMCC provider's probe path (drivers/clk/qcom/mmcc-msm8660.c)
 * after it has resolved the RPM supplier with device_link_add(). The
 * supplier stays bound for the consumer's lifetime so the pointer is
 * valid until the consumer (mmcc) goes away.
 *
 * SoC-agnostic consumer drivers (drm/msm/mdp4, drm/msm/adreno,
 * camss-vfe, vidc) call qcom_mmss_port_halt() without supplying the
 * handle themselves; on SoCs other than MSM8x60 the cache stays NULL
 * (no provider registered) and the helper returns -ENODEV, making the
 * port_halt call a harmless no-op there.
 *
 * READ_ONCE / WRITE_ONCE for visibility; full SMP synchronisation isn't
 * needed because (a) register happens once during the MMCC provider's
 * probe with no concurrent halt callers, and (b) callers that race with
 * a NULL-to-non-NULL transition simply get -ENODEV on the first call
 * and succeed on the next, which is the same as if they had arrived
 * one cycle later.
 */
static struct qcom_rpm *cached_rpm;

void qcom_mmss_porthalt_register_rpm(struct qcom_rpm *rpm)
{
	WRITE_ONCE(cached_rpm, rpm);
}
EXPORT_SYMBOL_GPL(qcom_mmss_porthalt_register_rpm);

/*
 * Serializes the refcount-transition -> RPM-write window. atomic_t per
 * port keeps individual port accounting correct across concurrent
 * callers; this mutex prevents two threads from issuing overlapping
 * QCOM_RPM_MM_FABRIC_HALT writes that the RPM would process in arrival
 * order (out-of-order delivery would leave stale halt-state bits if the
 * two writes covered the same port_mask bits with different halt
 * directions).
 */
static DEFINE_MUTEX(port_halt_lock);

/**
 * qcom_mmss_port_halt() - halt or unhalt MMSS NoC master ports
 * @port_mask:	bitmap of MMSS NoC master port IDs to act on
 *		(QCOM_MMSS_PORT_* defines, max 14 bits)
 * @halt:	true to halt the ports (quiesce AXI traffic), false to
 *		unhalt them (admit traffic again)
 *
 * Halting an MMSS NoC master port via QCOM_RPM_MM_FABRIC_HALT pairs with
 * collapsing the rail that backs that master, mirroring the downstream
 * MSM8x60 footswitch-8x60.c msm_bus_axi_porthalt() / portunhalt() calls.
 * The function refcounts per port so independent consumers can request a
 * halt without fighting each other; only refcount transitions trigger an
 * actual RPM IPC.
 *
 * Context: schedulable, IRQs enabled. The underlying qcom_rpm_write()
 * blocks on wait_for_completion_timeout() for the RPM ack IRQ. After
 * dpm_suspend_noirq() runs suspend_device_irqs() the ack IRQ is masked,
 * the completion never fires, and the call times out at 5*HZ. Therefore
 * this helper MUST NOT be invoked from:
 *
 *   - genpd power_off callbacks (which fire at .suspend_noirq)
 *   - syscore_ops.suspend (skipped entirely on s2idle anyway)
 *   - any .suspend_noirq / .resume_noirq driver callback
 *
 * The correct system-suspend callsite is .suspend_late on the consumer
 * device (mdp4 / adreno / camss-vfe / ...), which runs BEFORE
 * dpm_suspend_noirq. Runtime-PM-driven collapses are also fine since
 * they run with IRQs enabled by definition. The helper actively rejects
 * IRQ-disabled invocations with -EAGAIN rather than blocking for 5s and
 * returning -ETIMEDOUT.
 *
 * Return:
 * - 0 on success, or on no-op (the requested transition was already in
 *   the cumulative refcounted state).
 * - -ENODEV if no MMCC provider has registered an RPM handle yet
 *   (qcom_mmss_porthalt_register_rpm() hasn't run, or the running SoC
 *   simply has no MMSS NoC HALT facility -- e.g. mainline MSM8960+ where
 *   AXI quiescence moved in-band into the GDSC FSM). Callers can treat
 *   this as "halt unsupported on this platform, continue without it".
 * - -EAGAIN if called from atomic / IRQ-disabled context. The cumulative
 *   refcount is left unchanged; the caller should either retry in a
 *   schedulable context or accept the missed halt (the underlying rail
 *   collapse / restore still proceeds, just without the NoC quiesce
 *   pairing).
 * - -EINVAL on bad inputs.
 * - Whatever qcom_rpm_write() returns on RPM IPC failure. The refcount
 *   transitions are rolled back so the next call re-attempts the IPC,
 *   keeping the helper's view of silicon state consistent.
 */
int qcom_mmss_port_halt(u32 port_mask, bool halt)
{
	struct qcom_rpm *rpm = READ_ONCE(cached_rpm);
	unsigned long mask = port_mask;
	u32 transition_mask = 0;
	u32 cmd[2];
	int bit, ret;

	if (port_mask & ~MMSS_FABRIC_PORT_MASK)
		return -EINVAL;
	if (!port_mask)
		return 0;
	if (!rpm)
		return -ENODEV;

	/*
	 * Reject IRQ-disabled callers up front (see context note above).
	 * irqs_disabled() catches local_irq_disable, spin_lock_irq* and
	 * the dpm_suspend_noirq window; in_atomic() catches preempt-off
	 * regions. Either condition would make qcom_rpm_write block for
	 * 5*HZ and return -ETIMEDOUT.
	 */
	if (irqs_disabled() || in_atomic())
		return -EAGAIN;

	mutex_lock(&port_halt_lock);

	for_each_set_bit(bit, &mask, MMSS_FABRIC_NR_PORTS) {
		if (halt) {
			if (atomic_inc_return(&port_halt_count[bit]) == 1)
				transition_mask |= BIT(bit);
		} else {
			/*
			 * atomic_dec_if_positive returns the new value, or
			 * -1 if the counter was already 0 (consumer
			 * unbalanced its halt/unhalt). The latter is a
			 * caller bug; ignore the silicon-side write but do
			 * not WARN -- the helper's job is to be robust to
			 * imperfect consumer accounting.
			 */
			if (atomic_dec_if_positive(&port_halt_count[bit]) == 0)
				transition_mask |= BIT(bit);
		}
	}

	if (!transition_mask) {
		ret = 0;
		goto out_unlock;
	}

	/*
	 * QCOM_RPM_MM_FABRIC_HALT command format:
	 *   cmd[0] = halt state bits (1=halt, 0=unhalt) for the ports
	 *            named in cmd[1]
	 *   cmd[1] = port mask (which bits cmd[0] applies to)
	 */
	cmd[0] = halt ? transition_mask : 0;
	cmd[1] = transition_mask;

	ret = qcom_rpm_write(rpm, QCOM_RPM_ACTIVE_STATE,
			     QCOM_RPM_MM_FABRIC_HALT, cmd, 2);
	if (ret) {
		unsigned long roll = transition_mask;

		/*
		 * Roll back the transitions: silicon state did not change,
		 * but our refcount did. Without rollback the next matching
		 * call would see a non-transition and skip the IPC, leaving
		 * the silicon and our view permanently out of sync.
		 */
		for_each_set_bit(bit, &roll, MMSS_FABRIC_NR_PORTS) {
			if (halt)
				atomic_dec(&port_halt_count[bit]);
			else
				atomic_inc(&port_halt_count[bit]);
		}
	}

out_unlock:
	mutex_unlock(&port_halt_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_mmss_port_halt);

MODULE_DESCRIPTION("Qualcomm MSM8x60 MMSS NoC master port halt helper");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
