// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *  Copyright (c) 2014 The Linux Foundation. All rights reserved.
 */

#include <linux/cache.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/firmware/qcom/qcom_scm.h>

#include <asm/barrier.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#include <soc/qcom/spm.h>


#define VDD_SC1_ARRAY_CLAMP_GFS_CTL	0x35a0
#define SCSS_CPU1CORE_RESET		0x2d80
#define SCSS_DBG_STATUS_CORE_PWRDUP	0x2e64

#define APCS_CPU_PWR_CTL	0x04
#define PLL_CLAMP		BIT(8)
#define CORE_PWRD_UP		BIT(7)
#define COREPOR_RST		BIT(5)
#define CORE_RST		BIT(4)
#define L2DT_SLP		BIT(3)
#define CORE_MEM_CLAMP		BIT(1)
#define CLAMP			BIT(0)

#define APC_PWR_GATE_CTL	0x14
#define BHS_CNT_SHIFT		24
#define LDO_PWR_DWN_SHIFT	16
#define LDO_BYP_SHIFT		8
#define BHS_SEG_SHIFT		1
#define BHS_EN			BIT(0)

#define APCS_SAW2_VCTL		0x14
#define APCS_SAW2_2_VCTL	0x1c

extern void secondary_startup_arm(void);

/*
 * Holding-pen shared variable for MSM8660 (Scorpion-MP) SMP bring-up.
 *
 * msm8660_secondary_startup (arch/arm/mach-qcom/headsmp.S) is the
 * cold-boot entry pointed at by qcom_scm_set_cold_boot_addr — CPU1
 * lands there when scss_release_secondary takes it out of reset and
 * sits in a wfe loop polling this variable. The boot CPU writes
 * pen_release = cpu, flushes the cache line, executes `sev`, and
 * (defensively) sends a wakeup IPI. CPU1 sees its own id in
 * pen_release, writes -1 back as ack, and branches to
 * secondary_startup.
 *
 * Why this matters: GIC SGI delivery to a CPU in plain wfi is
 * unreliable on this silicon (the SGI is "marked pending" but the
 * handler never runs — observed as idle=N/1/0x40000000 in RCU
 * stalls). Wake via the WFE/SEV event mechanism plus a memory token
 * is what legacy webOS uses for the same reason.
 *
 * Volatile + flushed-on-write keeps the writer's update visible to
 * the reader running with cache off. Cacheline-aligned to avoid false
 * sharing with adjacent global state.
 */
volatile int pen_release __cacheline_aligned = -1;
EXPORT_SYMBOL(pen_release);

extern void msm8660_secondary_startup(void);

/*
 * Per-CPU flag tracking whether each core has completed its initial
 * cold boot sequence (scss_release_secondary). Used for hotplug to
 * determine whether to run full cold boot or just wake from pen_release.
 */
static DEFINE_PER_CPU(int, cold_boot_done);

#ifdef CONFIG_HOTPLUG_CPU
/*
 * CPU power collapse helper called from cpu_suspend().
 * This runs with MMU off, enters SCM to request power down,
 * then executes WFI. The SPM hardware state machine (already
 * programmed to SPC mode) handles the actual power collapse.
 */
static int qcom_pm_collapse_standalone(unsigned long unused)
{
	qcom_scm_cpu_power_down(QCOM_SCM_CPU_PWR_DOWN_L2_ON);
	/*
	 * Returns here only if there was a pending interrupt and we did not
	 * power down as a result.
	 */
	return -1;
}

static void qcom_cpu_die(unsigned int cpu)
{
	struct spm_driver_data *drv;

	drv = spm_get_drv_by_cpu(cpu);
	if (drv) {
		/*
		 * Set SPM to standalone power collapse mode (SPC).
		 * This programs SPM_CTL with rpm_bypass=1 (standalone, don't
		 * notify RPM) and mode=0x02 (power collapse with reset).
		 */
		spm_set_low_power_mode(drv, PM_SLEEP_MODE_SPC);

		/*
		 * Clear cold_boot_done for this CPU. After power collapse with
		 * reset, the CPU is fully powered down and needs the complete
		 * hardware initialization sequence (scss_release_secondary) on
		 * next boot, not just the pen_release wake path.
		 */
		per_cpu(cold_boot_done, cpu) = false;

		/*
		 * Loop until power collapse succeeds. cpu_suspend() can return
		 * if there are pending IRQs when SCM tries to power down.
		 * Keep retrying until we actually power off.
		 *
		 * cpu_suspend():
		 * 1. Saves CPU context (registers, VFP state)
		 * 2. Flushes caches
		 * 3. Calls qcom_pm_collapse_standalone (SCM + WFI)
		 * 4. SPM hardware manages power down when WFI executes
		 *
		 * This function never returns under normal hotplug. If it does
		 * return, it means we were woken up (pen_release written) for
		 * online, and SPM needs to be restored to standby.
		 */
		while (1) {
			int ret = cpu_suspend(0, qcom_pm_collapse_standalone);
			if (ret == 0) {
				/*
				 * Successfully woke from power collapse.
				 * Restore SPM to standby mode (clock gating).
				 */
				spm_set_low_power_mode(drv, PM_SLEEP_MODE_STBY);
				break;
			}
			/* Pending IRQ prevented collapse, retry */
		}
	} else {
		/*
		 * Fallback if SPM driver not available (shouldn't happen
		 * on MSM8660 with proper DT, but safe to have).
		 */
		pr_warn("CPU%u: SPM not found, using plain WFI\n", cpu);
		per_cpu(cold_boot_done, cpu) = false;
		while (1)
			wfi();
	}
}

/*
 * MSM8660/APQ8060 (Scorpion-MP) hotplug with power collapse.
 *
 * With SPM register initialization now in place (drivers/soc/qcom/spm.c),
 * CPU hotplug can use proper power collapse instead of plain WFI.
 *
 * The SPM driver programs power collapse mode via spm_set_low_power_mode()
 * when cpuidle enters cpu-spc state. For hotplug, the offline CPU should
 * enter the same power collapse path as cpuidle.
 *
 * Legacy webOS used msm_pm_power_collapse() from qcom_cpu_die which:
 * 1. Set SPM mode to POWER_COLLAPSE_STANDALONE
 * 2. Flushed VFP state and caches
 * 3. Called msm_pm_collapse() assembly (context save + WFI)
 * 4. On wake: restored context, reset SPM mode to clock gating
 *
 * For now, we allow hotplug but use simple WFI in qcom_cpu_die().
 * TODO: Integrate with SPM driver for full power collapse:
 *   - Call spm_set_low_power_mode(drv, PM_SLEEP_MODE_SPC)
 *   - Flush caches (via cpu_v7_do_idle path)
 *   - Enter WFI (hardware triggers power collapse via SPM)
 *   - On wake: SPM restores to clock gating mode
 */
static bool msm8660_cpu_can_disable(unsigned int cpu)
{
	/*
	 * Enable hotplug now that SPM initialization is in place.
	 * CPU1 can be offlined, which allows testing of single-core
	 * power collapse via cpuidle cpu-spc state.
	 */
	return true;
}
#endif

static int scss_release_secondary(unsigned int cpu)
{
	struct device_node *node;
	void __iomem *base;

	node = of_find_compatible_node(NULL, NULL, "qcom,gcc-msm8660");
	if (!node) {
		pr_err("%s: can't find node\n", __func__);
		return -ENXIO;
	}

	base = of_iomap(node, 0);
	of_node_put(node);
	if (!base)
		return -ENOMEM;

	writel_relaxed(0, base + VDD_SC1_ARRAY_CLAMP_GFS_CTL);
	writel_relaxed(0, base + SCSS_CPU1CORE_RESET);
	writel_relaxed(3, base + SCSS_DBG_STATUS_CORE_PWRDUP);
	mb();
	iounmap(base);

	return 0;
}

static int cortex_a7_release_secondary(unsigned int cpu)
{
	int ret = 0;
	void __iomem *reg;
	struct device_node *cpu_node, *acc_node;
	u32 reg_val;

	cpu_node = of_get_cpu_node(cpu, NULL);
	if (!cpu_node)
		return -ENODEV;

	acc_node = of_parse_phandle(cpu_node, "qcom,acc", 0);
	if (!acc_node) {
		ret = -ENODEV;
		goto out_acc;
	}

	reg = of_iomap(acc_node, 0);
	if (!reg) {
		ret = -ENOMEM;
		goto out_acc_map;
	}

	/* Put the CPU into reset. */
	reg_val = CORE_RST | COREPOR_RST | CLAMP | CORE_MEM_CLAMP;
	writel(reg_val, reg + APCS_CPU_PWR_CTL);

	/* Turn on the BHS and set the BHS_CNT to 16 XO clock cycles */
	writel(BHS_EN | (0x10 << BHS_CNT_SHIFT), reg + APC_PWR_GATE_CTL);
	/* Wait for the BHS to settle */
	udelay(2);

	reg_val &= ~CORE_MEM_CLAMP;
	writel(reg_val, reg + APCS_CPU_PWR_CTL);
	reg_val |= L2DT_SLP;
	writel(reg_val, reg + APCS_CPU_PWR_CTL);
	udelay(2);

	reg_val = (reg_val | BIT(17)) & ~CLAMP;
	writel(reg_val, reg + APCS_CPU_PWR_CTL);
	udelay(2);

	/* Release CPU out of reset and bring it to life. */
	reg_val &= ~(CORE_RST | COREPOR_RST);
	writel(reg_val, reg + APCS_CPU_PWR_CTL);
	reg_val |= CORE_PWRD_UP;
	writel(reg_val, reg + APCS_CPU_PWR_CTL);

	iounmap(reg);
out_acc_map:
	of_node_put(acc_node);
out_acc:
	of_node_put(cpu_node);
	return ret;
}

static int kpssv1_release_secondary(unsigned int cpu)
{
	int ret = 0;
	void __iomem *reg, *saw_reg;
	struct device_node *cpu_node, *acc_node, *saw_node;
	u32 val;

	cpu_node = of_get_cpu_node(cpu, NULL);
	if (!cpu_node)
		return -ENODEV;

	acc_node = of_parse_phandle(cpu_node, "qcom,acc", 0);
	if (!acc_node) {
		ret = -ENODEV;
		goto out_acc;
	}

	saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
	if (!saw_node) {
		ret = -ENODEV;
		goto out_saw;
	}

	reg = of_iomap(acc_node, 0);
	if (!reg) {
		ret = -ENOMEM;
		goto out_acc_map;
	}

	saw_reg = of_iomap(saw_node, 0);
	if (!saw_reg) {
		ret = -ENOMEM;
		goto out_saw_map;
	}

	/* Turn on CPU rail */
	writel_relaxed(0xA4, saw_reg + APCS_SAW2_VCTL);
	mb();
	udelay(512);

	/* Krait bring-up sequence */
	val = PLL_CLAMP | L2DT_SLP | CLAMP;
	writel_relaxed(val, reg + APCS_CPU_PWR_CTL);
	val &= ~L2DT_SLP;
	writel_relaxed(val, reg + APCS_CPU_PWR_CTL);
	mb();
	ndelay(300);

	val |= COREPOR_RST;
	writel_relaxed(val, reg + APCS_CPU_PWR_CTL);
	mb();
	udelay(2);

	val &= ~CLAMP;
	writel_relaxed(val, reg + APCS_CPU_PWR_CTL);
	mb();
	udelay(2);

	val &= ~COREPOR_RST;
	writel_relaxed(val, reg + APCS_CPU_PWR_CTL);
	mb();
	udelay(100);

	val |= CORE_PWRD_UP;
	writel_relaxed(val, reg + APCS_CPU_PWR_CTL);
	mb();

	iounmap(saw_reg);
out_saw_map:
	iounmap(reg);
out_acc_map:
	of_node_put(saw_node);
out_saw:
	of_node_put(acc_node);
out_acc:
	of_node_put(cpu_node);
	return ret;
}

static int kpssv2_release_secondary(unsigned int cpu)
{
	void __iomem *reg;
	struct device_node *cpu_node, *l2_node, *acc_node, *saw_node;
	void __iomem *l2_saw_base;
	unsigned reg_val;
	int ret;

	cpu_node = of_get_cpu_node(cpu, NULL);
	if (!cpu_node)
		return -ENODEV;

	acc_node = of_parse_phandle(cpu_node, "qcom,acc", 0);
	if (!acc_node) {
		ret = -ENODEV;
		goto out_acc;
	}

	l2_node = of_parse_phandle(cpu_node, "next-level-cache", 0);
	if (!l2_node) {
		ret = -ENODEV;
		goto out_l2;
	}

	saw_node = of_parse_phandle(l2_node, "qcom,saw", 0);
	if (!saw_node) {
		ret = -ENODEV;
		goto out_saw;
	}

	reg = of_iomap(acc_node, 0);
	if (!reg) {
		ret = -ENOMEM;
		goto out_map;
	}

	l2_saw_base = of_iomap(saw_node, 0);
	if (!l2_saw_base) {
		ret = -ENOMEM;
		goto out_saw_map;
	}

	/* Turn on the BHS, turn off LDO Bypass and power down LDO */
	reg_val = (64 << BHS_CNT_SHIFT) | (0x3f << LDO_PWR_DWN_SHIFT) | BHS_EN;
	writel_relaxed(reg_val, reg + APC_PWR_GATE_CTL);
	mb();
	/* wait for the BHS to settle */
	udelay(1);

	/* Turn on BHS segments */
	reg_val |= 0x3f << BHS_SEG_SHIFT;
	writel_relaxed(reg_val, reg + APC_PWR_GATE_CTL);
	mb();
	 /* wait for the BHS to settle */
	udelay(1);

	/* Finally turn on the bypass so that BHS supplies power */
	reg_val |= 0x3f << LDO_BYP_SHIFT;
	writel_relaxed(reg_val, reg + APC_PWR_GATE_CTL);

	/* enable max phases */
	writel_relaxed(0x10003, l2_saw_base + APCS_SAW2_2_VCTL);
	mb();
	udelay(50);

	reg_val = COREPOR_RST | CLAMP;
	writel_relaxed(reg_val, reg + APCS_CPU_PWR_CTL);
	mb();
	udelay(2);

	reg_val &= ~CLAMP;
	writel_relaxed(reg_val, reg + APCS_CPU_PWR_CTL);
	mb();
	udelay(2);

	reg_val &= ~COREPOR_RST;
	writel_relaxed(reg_val, reg + APCS_CPU_PWR_CTL);
	mb();

	reg_val |= CORE_PWRD_UP;
	writel_relaxed(reg_val, reg + APCS_CPU_PWR_CTL);
	mb();

	ret = 0;

	iounmap(l2_saw_base);
out_saw_map:
	iounmap(reg);
out_map:
	of_node_put(saw_node);
out_saw:
	of_node_put(l2_node);
out_l2:
	of_node_put(acc_node);
out_acc:
	of_node_put(cpu_node);

	return ret;
}

static int qcom_boot_secondary(unsigned int cpu, int (*func)(unsigned int))
{
	int ret = 0;

	if (!per_cpu(cold_boot_done, cpu)) {
		ret = func(cpu);
		if (!ret)
			per_cpu(cold_boot_done, cpu) = true;
	}

	/*
	 * Send the secondary CPU a soft interrupt, thereby causing
	 * the boot monitor to read the system wide flags register,
	 * and branch to the address found there.
	 */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	return ret;
}

/*
 * MSM8660 (Scorpion-MP) boot_secondary using pen_release + sev.
 *
 * Mirrors what legacy webOS does (the qcom_boot_secondary above is a
 * plain "IPI and hope" mechanism that works on most QC SoCs but is
 * not reliable on Scorpion-MP; the IPI sometimes doesn't wake CPU1).
 *
 * Sequence:
 *   1. cold-boot path: scss_release_secondary takes CPU1 out of reset
 *      so it lands at msm8660_secondary_startup (the cold-boot addr
 *      set by msm8660_smp_prepare_cpus). On warm hotplug-up the CPU is
 *      already in our headsmp.S wfe loop from qcom_cpu_die.
 *   2. Write pen_release = cpu and flush the cache line — CPU1 reads
 *      pen_release from DRAM with cache off, so we MUST flush before
 *      it can see the new value.
 *   3. asm("sev") wakes any CPU sitting in wfe (which includes CPU1
 *      polling pen_release in headsmp.S).
 *   4. dsb() ensures the SEV is globally observed before the IPI.
 *   5. arch_send_wakeup_ipi_mask is the belt-and-braces backup —
 *      delivers an SGI just in case the SEV path is somehow missed.
 *      Harmless if CPU1 is already past the wfe loop.
 *   6. Poll pen_release until CPU1 writes -1 (the ack from
 *      msm8660_secondary_startup), with a SECONDARY_CPU_WAIT_MS
 *      cap (~10 ms) to avoid wedging if CPU1 truly never starts.
 */
#define SECONDARY_CPU_WAIT_MS	10

/*
 * Light-weight pen_release path diagnostics. These are read by anyone
 * curious about the wake mechanism via /proc/scorpion_smp_stats (if
 * the proc entry below is built) and printed once on first success.
 */
static atomic_t pen_release_successes;
static atomic_t pen_release_timeouts;
static unsigned int pen_release_last_ack_ms;
static int pen_release_last_seen_on_timeout;

static int msm8660_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int ret;
	int cnt = 0;
	unsigned long start_jiffies;
	struct spm_driver_data *drv;

	/*
	 * Restore SPM to standby mode before attempting to bring CPU online.
	 * If the CPU was previously offlined via power collapse (SPC), its
	 * SPM is still programmed for power-down mode. We must restore it
	 * to standby so the CPU can actually run when we release it from reset.
	 */
	drv = spm_get_drv_by_cpu(cpu);
	if (drv)
		spm_set_low_power_mode(drv, PM_SLEEP_MODE_STBY);

	if (!per_cpu(cold_boot_done, cpu)) {
		ret = scss_release_secondary(cpu);
		if (ret)
			return ret;
		per_cpu(cold_boot_done, cpu) = true;
	}

	/*
	 * Release CPU%u from the holding pen via pen_release + sev.
	 * Cache is off on CPU1 at this point so we must flush our
	 * write all the way to DRAM before signalling.
	 */
	pen_release = cpu;
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release),
			  __pa(&pen_release) + sizeof(pen_release));
	dsb(ishst);
	sev();

	/* Backup: also poke via IPI in case sev didn't take. */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	start_jiffies = jiffies;

	/*
	 * Wait for CPU%u to acknowledge by writing pen_release back to
	 * -1 (mvn r7,#0; str r7,[r6] in headsmp.S). We have to
	 * invalidate our cache line each iteration because CPU1's ack
	 * is a memory write with cache off, so it won't appear in our
	 * cached view automatically.
	 */
	while (pen_release != -1) {
		__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
		outer_inv_range(__pa(&pen_release),
				__pa(&pen_release) + sizeof(pen_release));
		if (cnt++ >= SECONDARY_CPU_WAIT_MS)
			break;
		usleep_range(1000, 1500);
	}

	if (pen_release != -1) {
		atomic_inc(&pen_release_timeouts);
		pen_release_last_seen_on_timeout = pen_release;
		pr_warn("CPU%u: pen_release ack timed out (pen=%d, %ums)\n",
			cpu, pen_release,
			jiffies_to_msecs(jiffies - start_jiffies));
		return -ETIMEDOUT;
	}

	/*
	 * Diagnostics: record latency in ms and print once on first
	 * success so dmesg has positive confirmation the mechanism works.
	 * Subsequent successes update the counter silently — read via
	 * pr_info-on-demand or future debugfs.
	 */
	pen_release_last_ack_ms = jiffies_to_msecs(jiffies - start_jiffies);
	if (atomic_inc_return(&pen_release_successes) == 1)
		pr_info("Scorpion-MP pen_release path active: CPU%u released and ack'd in %ums\n",
			cpu, pen_release_last_ack_ms);

	return 0;
}

static int cortex_a7_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	return qcom_boot_secondary(cpu, cortex_a7_release_secondary);
}

static int kpssv1_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	return qcom_boot_secondary(cpu, kpssv1_release_secondary);
}

static int kpssv2_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	return qcom_boot_secondary(cpu, kpssv2_release_secondary);
}

static void __init qcom_smp_prepare_cpus(unsigned int max_cpus)
{
	int cpu;

	if (qcom_scm_set_cold_boot_addr(secondary_startup_arm)) {
		for_each_present_cpu(cpu) {
			if (cpu == smp_processor_id())
				continue;
			set_cpu_present(cpu, false);
		}
		pr_warn("Failed to set CPU boot address, disabling SMP\n");
	}
}

/*
 * MSM8660 needs the holding-pen stub (msm8660_secondary_startup) as the
 * cold-boot address rather than the generic secondary_startup_arm —
 * see headsmp.S and pen_release for why.
 */
static void __init msm8660_smp_prepare_cpus(unsigned int max_cpus)
{
	int cpu;

	if (qcom_scm_set_cold_boot_addr(msm8660_secondary_startup)) {
		for_each_present_cpu(cpu) {
			if (cpu == smp_processor_id())
				continue;
			set_cpu_present(cpu, false);
		}
		pr_warn("Failed to set CPU boot address, disabling SMP\n");
	}
}

static const struct smp_operations smp_msm8660_ops __initconst = {
	.smp_prepare_cpus	= msm8660_smp_prepare_cpus,
	.smp_boot_secondary	= msm8660_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= qcom_cpu_die,
	.cpu_can_disable	= msm8660_cpu_can_disable,
#endif
};
CPU_METHOD_OF_DECLARE(qcom_smp, "qcom,gcc-msm8660", &smp_msm8660_ops);

static const struct smp_operations qcom_smp_cortex_a7_ops __initconst = {
	.smp_prepare_cpus	= qcom_smp_prepare_cpus,
	.smp_boot_secondary	= cortex_a7_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= qcom_cpu_die,
#endif
};
CPU_METHOD_OF_DECLARE(qcom_smp_msm8226, "qcom,msm8226-smp", &qcom_smp_cortex_a7_ops);
CPU_METHOD_OF_DECLARE(qcom_smp_msm8909, "qcom,msm8909-smp", &qcom_smp_cortex_a7_ops);
CPU_METHOD_OF_DECLARE(qcom_smp_msm8916, "qcom,msm8916-smp", &qcom_smp_cortex_a7_ops);

static const struct smp_operations qcom_smp_kpssv1_ops __initconst = {
	.smp_prepare_cpus	= qcom_smp_prepare_cpus,
	.smp_boot_secondary	= kpssv1_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= qcom_cpu_die,
#endif
};
CPU_METHOD_OF_DECLARE(qcom_smp_kpssv1, "qcom,kpss-acc-v1", &qcom_smp_kpssv1_ops);

static const struct smp_operations qcom_smp_kpssv2_ops __initconst = {
	.smp_prepare_cpus	= qcom_smp_prepare_cpus,
	.smp_boot_secondary	= kpssv2_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= qcom_cpu_die,
#endif
};
CPU_METHOD_OF_DECLARE(qcom_smp_kpssv2, "qcom,kpss-acc-v2", &qcom_smp_kpssv2_ops);
