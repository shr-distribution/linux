// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright (c) 2011, The Linux Foundation. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/bitops.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/dma/qcom_adm.h>
#include "mmci.h"

/* Registers */
#define DML_CONFIG			0x00
#define PRODUCER_CRCI_MSK		GENMASK(1, 0)
#define PRODUCER_CRCI_DISABLE		0
#define PRODUCER_CRCI_X_SEL		BIT(0)
#define PRODUCER_CRCI_Y_SEL		BIT(1)
#define CONSUMER_CRCI_MSK		GENMASK(3, 2)
#define CONSUMER_CRCI_DISABLE		0
#define CONSUMER_CRCI_X_SEL		BIT(2)
#define CONSUMER_CRCI_Y_SEL		BIT(3)
#define PRODUCER_TRANS_END_EN		BIT(4)
#define BYPASS				BIT(16)
#define DIRECT_MODE			BIT(17)
#define INFINITE_CONS_TRANS		BIT(18)

#define DML_SW_RESET			0x08
#define DML_PRODUCER_START		0x0c
#define DML_CONSUMER_START		0x10
#define DML_PRODUCER_PIPE_LOGICAL_SIZE	0x14
#define DML_CONSUMER_PIPE_LOGICAL_SIZE	0x18
#define DML_PIPE_ID			0x1c
#define PRODUCER_PIPE_ID_SHFT		0
#define PRODUCER_PIPE_ID_MSK		GENMASK(4, 0)
#define CONSUMER_PIPE_ID_SHFT		16
#define CONSUMER_PIPE_ID_MSK		GENMASK(20, 16)

#define DML_PRODUCER_BAM_BLOCK_SIZE	0x24
#define DML_PRODUCER_BAM_TRANS_SIZE	0x28

/* other definitions */
#define PRODUCER_PIPE_LOGICAL_SIZE	4096
#define CONSUMER_PIPE_LOGICAL_SIZE	4096

#define DML_OFFSET			0x800

/*
 * Check if the DMA controller is ADM (Application Data Mover).
 * ADM is used on older Qualcomm SoCs (msm8x60 and earlier) and
 * does not have the DML (Data Mover Layer) block.
 */
static bool qcom_dma_is_adm(struct device_node *np)
{
	struct of_phandle_args dma_spec;
	struct device_node *dma_node;
	bool is_adm = false;

	if (of_parse_phandle_with_args(np, "dmas", "#dma-cells", 0, &dma_spec))
		return false;

	dma_node = dma_spec.np;
	if (dma_node) {
		struct property *prop;
		const char *compat;

		/*
		 * The qcom_adm driver binds the bare "qcom,adm" AND several
		 * per-SoC compatibles ("qcom,adm-apq8060", "qcom,adm-msm8660",
		 * "qcom,adm-ipq8064", ...). The original exact match on
		 * "qcom,adm" missed all the SoC-specific strings, so a node
		 * like tenderloin's "qcom,adm-apq8060" was NOT recognised as
		 * ADM: qcom_dma_start() then fell through to the BAM path
		 * (mmci_dmae_start = submit + issue immediately) instead of the
		 * ADM submit-only path. That issued the channel during
		 * dma_start — before mmci stashed DATACTRL/ARG/CMD for the
		 * atomic-submit exec_func — so exec_func fired with an empty
		 * stash, the SDCC was never armed, and eMMC reads wedged
		 * (ADM ch2 waiting on a CRCI that never came). Match any
		 * "qcom,adm" / "qcom,adm-*" compatible by prefix.
		 */
		of_property_for_each_string(dma_node, "compatible", prop, compat) {
			if (!strncmp(compat, "qcom,adm", strlen("qcom,adm"))) {
				is_adm = true;
				break;
			}
		}
		of_node_put(dma_node);
	}

	return is_adm;
}

/*
 * exec_func callback invoked by the qcom_adm driver from inside its
 * per-controller submit_lock, immediately before the channel's CMD_PTR
 * register is written. Writes the SDCC DATACTRL + CMD ARG + CMD REG
 * atomically with the ADM start, replicating the legacy msm_sdcc
 * msmsdcc_dma_exec_func() pattern.
 *
 * Constraints (per <linux/dma/qcom_adm.h>):
 *  - Called in atomic context with submit_lock held + IRQs off.
 *  - Must not sleep, must complete promptly.
 *
 * Closes a documented mainline-vs-legacy gap: without this, mmci writes
 * DATACTRL/ARG/CMD outside any cross-channel lock, so under AHB-fabric
 * contention (e.g. eMMC + WiFi DMA both bursting on adm_dma1) the gap
 * between those writes can be split by another ADM channel's CMD_PTR
 * write, breaking the SDCC's atomic command+data setup.
 */
void mmci_qcom_atomic_exec_func(void *exec_user)
{
	struct mmci_host *host = exec_user;
	void __iomem *base = host->base;
	unsigned int delay_us;

	/*
	 * Use the same clock-dependent delay formula as legacy msm_sdcc's
	 * msmsdcc_delay(): 1 + 3000000 / clk_rate (≈1 µs @ 48 MHz, ≈8.5 µs
	 * @ 400 kHz init clock).  Commit 32870e59b8ad added this formula to
	 * mmci_start_command, but mmci_start_command is a no-op stash when
	 * atomic-submit is active — the actual ARG/CMD writes are HERE.
	 * For the EXT_CSD READ (which happens at 400 kHz init clock with
	 * atomic-submit active since reads are gated through this path),
	 * the previous hardcoded udelay(1) was too short — Samsung PRV=0x90
	 * occasionally returned an OTP-only EXT_CSD (capacity = 0 B) because
	 * the SDCC sampled DATACTRL/ARG before the card's internal SRAM mux
	 * had switched from the OTP block to the dynamic EXT_CSD block.
	 */
	delay_us = (host->variant->qcom_datactrl_delay) ?
		mmci_qcom_settle_us(host) : 0;

	/*
	 * Match legacy webOS msmsdcc_dma_exec_func ordering:
	 *
	 *   MMCIDATATIMER  -> MMCIDATALENGTH  -> delay
	 *   -> MMCIDATACTRL -> delay
	 *   -> MMCIARGUMENT -> delay
	 *   -> MMCICOMMAND
	 *
	 * DATATIMER + DATALENGTH MUST be written inside this atomic window
	 * (ADM has the descriptor queued; we're inside the ADM submit_lock
	 * just before CMD_PTR fires).  Writing them earlier in mmci_start_data
	 * left DPSM with its byte-count loaded tens of microseconds before
	 * CMD53 reached the chip on the wire, which on tenderloin/AR6003
	 * corrupted single-CMD53 128 B HTC mailbox reads (root cause of
	 * the A3 PIO split workaround at ath6kl/sdio.c:228-250).
	 */
	/*
	 * DEBUG one-shot per host: dump the exact stash values this
	 * exec_func is about to program. Captures DATACTRL / CMD at the
	 * instant of the atomic submit, with no CPSM/DPSM auto-clear
	 * ambiguity (the +500ms watchdog readback can't distinguish
	 * "never written" from "written then auto-cleared on completion").
	 * If datactrl/cmd_reg read 0 here, the stash was never populated
	 * and SDCC was never told to transfer -> ADM waits for a CRCI
	 * that can't come.
	 */
	if (!host->atomic_exec_logged) {
		host->atomic_exec_logged = 1;
		dev_info(mmc_dev(host->mmc),
			 "atomic_exec mmc%u: datactrl=0x%08x cmd_reg=0x%08x cmd_arg=0x%08x datalen=0x%08x datatimer=0x%08x\n",
			 host->mmc->index,
			 host->atomic_submit.datactrl,
			 host->atomic_submit.cmd_reg,
			 host->atomic_submit.cmd_arg,
			 host->atomic_submit.datalen,
			 host->atomic_submit.datatimer);
	}

	writel_relaxed(host->atomic_submit.datatimer, base + MMCIDATATIMER);
	writel_relaxed(host->atomic_submit.datalen, base + MMCIDATALENGTH);
	if (delay_us)
		udelay(delay_us);

	writel_relaxed(host->atomic_submit.datactrl, base + MMCIDATACTRL);
	if (delay_us)
		udelay(delay_us);

	writel_relaxed(host->atomic_submit.cmd_arg, base + MMCIARGUMENT);
	if (delay_us)
		udelay(delay_us);

	writel(host->atomic_submit.cmd_reg, base + MMCICOMMAND);
}

/*
 * mmci_qcom_dump_state - peripheral state snapshot for ADM watchdog
 *
 * Called from qcom_adm's adm_watchdog_timeout when an ADM-side transfer
 * wedges (no RSLT_VALID within 500 ms). Reads the SDCC controller's
 * MMIO state at THAT instant — DATACTRL, MMCISTATUS, DATACNT, etc — so
 * we can correlate which side stopped pumping first. Output goes to
 * dev_warn on the mmci device.
 *
 * Constraints (per qcom_adm.h):
 *   - timer softirq, achan->vc.lock held, IRQs off on local CPU
 *   - must not sleep, must not nest locks above vc.lock
 *   - one readl per register, one printk
 */
void mmci_qcom_dump_state(void *dump_user)
{
	struct mmci_host *host = dump_user;
	void __iomem *base = host->base;

	dev_warn(mmc_dev(host->mmc),
		"ADM-WATCHDOG mmc%u snapshot: POWER=0x%08x CLOCK=0x%08x CMD=0x%08x CMDARG=0x%08x DLEN=0x%08x DCTL=0x%08x DCNT=0x%08x STAT=0x%08x MASK0=0x%08x FIFOCNT=0x%08x\n",
		host->mmc->index,
		readl_relaxed(base + MMCIPOWER),
		readl_relaxed(base + MMCICLOCK),
		readl_relaxed(base + MMCICOMMAND),
		readl_relaxed(base + MMCIARGUMENT),
		readl_relaxed(base + MMCIDATALENGTH),
		readl_relaxed(base + MMCIDATACTRL),
		readl_relaxed(base + MMCIDATACNT),
		readl_relaxed(base + MMCISTATUS),
		readl_relaxed(base + MMCIMASK0),
		readl_relaxed(base + MMCIFIFOCNT));
}

static int qcom_dma_start(struct mmci_host *host, unsigned int *datactrl)
{
	u32 config;
	void __iomem *base = host->base + DML_OFFSET;
	struct mmc_data *data = host->data;
	struct device_node *np = host->mmc->parent->of_node;
	int ret;

	if (qcom_dma_is_adm(np)) {
		/*
		 * For ADM DMA: submit descriptor but don't issue pending.
		 *
		 * Two paths from here:
		 *  - atomic_submit.active = true (variant->qcom_dml_atomic_submit
		 *    + qualifying write request): mmci_dma_start stashes
		 *    DATACTRL on host->atomic_submit; mmci_start_command stashes
		 *    ARG + CMD reg; mmci_request kicks dma_issue_pending after
		 *    stashing, and the ADM driver fires our exec_func from
		 *    inside its submit_lock to do all three writes atomically
		 *    with the ADM CMD_PTR.
		 *  - otherwise: legacy mainline behaviour — mmci_dma_start
		 *    writes DATACTRL, mmci_start_command writes ARG+CMD,
		 *    cmd_irq later fires dma_issue_pending.
		 */
		ret = mmci_dmae_submit(host, datactrl);
		return ret;
	}

	/* BAM DMA path: submit + issue, then configure DML */
	ret = mmci_dmae_start(host, datactrl);
	if (ret)
		return ret;

	if (data->flags & MMC_DATA_READ) {
		/* Read operation: configure DML for producer operation */
		/* Set producer CRCI-x and disable consumer CRCI */
		config = readl_relaxed(base + DML_CONFIG);
		config = (config & ~PRODUCER_CRCI_MSK) | PRODUCER_CRCI_X_SEL;
		config = (config & ~CONSUMER_CRCI_MSK) | CONSUMER_CRCI_DISABLE;
		writel_relaxed(config, base + DML_CONFIG);

		/* Set the Producer BAM block size */
		writel_relaxed(data->blksz, base + DML_PRODUCER_BAM_BLOCK_SIZE);

		/* Set Producer BAM Transaction size */
		writel_relaxed(data->blocks * data->blksz,
			       base + DML_PRODUCER_BAM_TRANS_SIZE);
		/* Set Producer Transaction End bit */
		config = readl_relaxed(base + DML_CONFIG);
		config |= PRODUCER_TRANS_END_EN;
		writel_relaxed(config, base + DML_CONFIG);
		/* Trigger producer */
		writel_relaxed(1, base + DML_PRODUCER_START);
	} else {
		/* Write operation: configure DML for consumer operation */
		/* Set consumer CRCI-x and disable producer CRCI*/
		config = readl_relaxed(base + DML_CONFIG);
		config = (config & ~CONSUMER_CRCI_MSK) | CONSUMER_CRCI_X_SEL;
		config = (config & ~PRODUCER_CRCI_MSK) | PRODUCER_CRCI_DISABLE;
		writel_relaxed(config, base + DML_CONFIG);
		/* Clear Producer Transaction End bit */
		config = readl_relaxed(base + DML_CONFIG);
		config &= ~PRODUCER_TRANS_END_EN;
		writel_relaxed(config, base + DML_CONFIG);
		/* Trigger consumer */
		writel_relaxed(1, base + DML_CONSUMER_START);
	}

	/* make sure the dml is configured before dma is triggered */
	wmb();
	return 0;
}

static int of_get_dml_pipe_index(struct device_node *np, const char *name)
{
	int index;
	struct of_phandle_args	dma_spec;

	index = of_property_match_string(np, "dma-names", name);

	if (index < 0)
		return -ENODEV;

	if (of_parse_phandle_with_args(np, "dmas", "#dma-cells", index,
				       &dma_spec))
		return -ENODEV;

	of_node_put(dma_spec.np);
	if (dma_spec.args_count)
		return dma_spec.args[0];

	return -ENODEV;
}

/* Initialize the dml hardware connected to SD Card controller */
static int qcom_dma_setup(struct mmci_host *host)
{
	u32 config;
	void __iomem *base;
	int consumer_id, producer_id, ret;
	struct device_node *np = host->mmc->parent->of_node;
	bool use_adm;

	ret = mmci_dmae_setup(host);
	if (ret) {
		/*
		 * Propagate ALL errors up (Sashiko Critical #1).  The
		 * previous 'return 0' on non-EPROBE_DEFER errors silently
		 * swallowed -ENOMEM where mmci_dmae_setup failed before
		 * allocating struct mmci_dmae_priv -- leaving host->dma_priv
		 * NULL while mmci_dma_setup would still set host->use_dma
		 * to true, and any subsequent code path touching dma_priv
		 * (mmci_dmae_prep_data, mmci_dmae_submit, etc.) hit a NULL
		 * dereference.
		 *
		 * Returning the error here lets mmci_dma_setup leave
		 * host->use_dma = false; the host operates in PIO mode for
		 * non-deferred failures as the previous comment intended,
		 * but without the half-initialised DMA state landmine.
		 */
		dev_dbg(host->mmc->parent,
			"DMA setup failed (%d), falling back to PIO mode\n",
			ret);
		return ret;
	}

	/*
	 * ADM (Application Data Mover) is used on older Qualcomm SoCs
	 * like msm8x60. ADM doesn't use DML, so skip DML setup and just
	 * return success - DMA will work directly via ADM.
	 */
	use_adm = qcom_dma_is_adm(np);
	if (use_adm) {
		return 0;
	}

	consumer_id = of_get_dml_pipe_index(np, "tx");
	producer_id = of_get_dml_pipe_index(np, "rx");

	if (producer_id < 0 || consumer_id < 0) {
		mmci_dmae_release(host);
		return -EINVAL;
	}

	base = host->base + DML_OFFSET;

	/* Reset the DML block */
	writel_relaxed(1, base + DML_SW_RESET);

	/* Disable the producer and consumer CRCI */
	config = (PRODUCER_CRCI_DISABLE | CONSUMER_CRCI_DISABLE);
	/*
	 * Disable the bypass mode. Bypass mode will only be used
	 * if data transfer is to happen in PIO mode and don't
	 * want the BAM interface to connect with SDCC-DML.
	 */
	config &= ~BYPASS;
	/*
	 * Disable direct mode as we don't DML to MASTER the AHB bus.
	 * BAM connected with DML should MASTER the AHB bus.
	 */
	config &= ~DIRECT_MODE;
	/*
	 * Disable infinite mode transfer as we won't be doing any
	 * infinite size data transfers. All data transfer will be
	 * of finite data size.
	 */
	config &= ~INFINITE_CONS_TRANS;
	writel_relaxed(config, base + DML_CONFIG);

	/*
	 * Initialize the logical BAM pipe size for producer
	 * and consumer.
	 */
	writel_relaxed(PRODUCER_PIPE_LOGICAL_SIZE,
		       base + DML_PRODUCER_PIPE_LOGICAL_SIZE);
	writel_relaxed(CONSUMER_PIPE_LOGICAL_SIZE,
		       base + DML_CONSUMER_PIPE_LOGICAL_SIZE);

	/* Initialize Producer/consumer pipe id */
	writel_relaxed(producer_id | (consumer_id << CONSUMER_PIPE_ID_SHFT),
		       base + DML_PIPE_ID);

	/* Make sure dml initialization is finished */
	mb();

	return 0;
}

static u32 qcom_get_dctrl_cfg(struct mmci_host *host)
{
	return MCI_DPSM_ENABLE | (host->data->blksz << 4);
}

/*
 * Issue DMA pending for ADM. Called after DATACTRL is written.
 * For BAM DMA, issue_pending was already called in qcom_dma_start(),
 * so this is only needed for ADM.
 */
static void qcom_dma_issue_pending(struct mmci_host *host)
{
	struct device_node *np = host->mmc->parent->of_node;

	if (qcom_dma_is_adm(np))
		mmci_dmae_issue_pending(host);
}

static struct mmci_host_ops qcom_variant_ops = {
	.prep_data = mmci_dmae_prep_data,
	.unprep_data = mmci_dmae_unprep_data,
	.get_datactrl_cfg = qcom_get_dctrl_cfg,
	.get_next_data = mmci_dmae_get_next_data,
	.dma_setup = qcom_dma_setup,
	.dma_release = mmci_dmae_release,
	.dma_start = qcom_dma_start,
	.dma_issue_pending = qcom_dma_issue_pending,
	.dma_finalize = mmci_dmae_finalize,
	.dma_error = mmci_dmae_error,
};

void qcom_variant_init(struct mmci_host *host)
{
	host->ops = &qcom_variant_ops;
}
