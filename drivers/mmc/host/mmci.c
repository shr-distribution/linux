// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/drivers/mmc/host/mmci.c - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (C) 2010 ST-Ericsson SA
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/string_choices.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/pm.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dma/qcom_adm.h>
#include <linux/amba/mmci.h>
#include <linux/interconnect.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>

#include <asm/div64.h>
#include <asm/io.h>

#include "mmci.h"

#define DRIVER_NAME "mmci-pl18x"

/*
 * Hot-path bring-up diagnostics for the Qualcomm variant.  trace_printk()
 * prints into the ftrace ring buffer (no console latency) but pollutes
 * the buffer and prints a one-shot WARN at boot, so gate the calls
 * behind a Kconfig that defaults to N (Sashiko Medium #49,
 * submit/mmci-qcom-tenderloin, 2026-06-14).
 */
#ifdef CONFIG_MMC_ARMMMCI_QCOM_DEBUG
#define mmci_qcom_diag(fmt, ...)	trace_printk(fmt, ##__VA_ARGS__)
#else
#define mmci_qcom_diag(fmt, ...)	do { } while (0)
#endif

static void mmci_variant_init(struct mmci_host *host);
static void ux500_variant_init(struct mmci_host *host);
static void ux500v2_variant_init(struct mmci_host *host);

static unsigned int fmax = 515633;

/*
 * Runtime knob for the mmc1 (sdcc4/WiFi) SDIO DMA threshold (qcom variant).
 * Default 256 keeps the historical AR6003 mailbox-WR workaround: writes
 * below 256 B go through PIO.  Lower to 64 (= variant->fifosize) to match
 * legacy webOS msm_sdcc validate_dma() and let 128 B HTC mailbox writes
 * flow via ADM DMA.  Sysfs path on built-in:
 *   /sys/module/mmci_pl180/parameters/mmc1_wr_dma_min
 */
static unsigned int mmci_mmc1_wr_dma_min = 256;

/*
 * Active-transfer ICC bandwidth vote for the qcom eMMC (mmc0), in the icc
 * units mmci passes to icc_set_bw (~kBps; 512000 -> ~64 MHz DFAB at the
 * 8-byte DFAB-SDC node width).  The idle vote stays 512000 (the
 * webOS-measured ~64 MHz DFAB idle rate); this is voted by
 * mmci_qcom_icc_bump_active() for the duration of a data transfer so the
 * dynamic fabric scaling (interconnect/qcom/msm8660.c) ramps DFAB up while
 * the eMMC drains.  Flat active==idle==512000 pinned DFAB at 64 MHz, which
 * is enough raw bandwidth (512 MB/s) but leaves no latency headroom for the
 * ADM<->SDCC drain handshake -- the first multi-block read graceful-flushes
 * (STATE0=0x8000c003, RXOVERRUN).  The pre-v5 fabric ran higher and masked
 * this.  Default ~200 MHz of DFAB headroom; tunable on the kernel cmdline
 * (mmci_pl18x.qcom_emmc_active_bw=...) to sweep the threshold without a
 * rebuild.  0 falls back to 512000 (no active ramp).
 */
static unsigned int mmci_qcom_emmc_active_bw = 1600000;

/*
 * DEBUG instrumentation: cap the per-request size on the qcom eMMC so the
 * transfer-size threshold where the ADM ch2 drain breaks can be swept.
 * 512 B reads always succeed; 4 KB reads fail.  Set bytes via
 * armmmci.qcom_emmc_max_bytes=N (e.g. 512, 1024, 2048, 3072, 4096) to bisect
 * the breaking size: if it breaks just above the 64 B FIFO / a fixed size it
 * is a FIFO/burst-depth limit; if it scales with DFAB rate it is bandwidth.
 * 0 = no cap.  A small working cap also doubles as a reliable-boot workaround.
 */
static unsigned int mmci_qcom_emmc_max_bytes;

/*
 * Debug knobs for the qcom ADM write-throughput investigation.
 *  - adm_sample: periodically log the SDCC FIFO/DATACNT cadence during
 *    large ADM transfers so the per-burst rate is visible before any
 *    watchdog wedge (see mmci_adm_sample_work).
 *  - qcom_pwrsave: gate MCI_CLK_PWRSAVE on the qcom SDCC. Default on
 *    (matches legacy webOS CLKREG=0x9f00). Boot with
 *    mmci_pl18x.qcom_pwrsave=0 to A/B test whether per-burst clock
 *    gating is throttling writes.
 */
static bool adm_sample = true;
static bool qcom_pwrsave = true;
static void mmci_adm_sample_arm(struct mmci_host *host, struct mmc_data *data);

static struct variant_data variant_arm = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datalength_bits	= 16,
	.datactrl_blocksz	= 11,
	.pwrreg_powerup		= MCI_PWR_UP,
	.f_max			= 100000000,
	.reversed_irq_handling	= true,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_ROD,
	.init			= mmci_variant_init,
};

static struct variant_data variant_arm_extended_fifo = {
	.fifosize		= 128 * 4,
	.fifohalfsize		= 64 * 4,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datalength_bits	= 16,
	.datactrl_blocksz	= 11,
	.pwrreg_powerup		= MCI_PWR_UP,
	.f_max			= 100000000,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_ROD,
	.init			= mmci_variant_init,
};

static struct variant_data variant_arm_extended_fifo_hwfc = {
	.fifosize		= 128 * 4,
	.fifohalfsize		= 64 * 4,
	.clkreg_enable		= MCI_ARM_HWFCEN,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datalength_bits	= 16,
	.datactrl_blocksz	= 11,
	.pwrreg_powerup		= MCI_PWR_UP,
	.f_max			= 100000000,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_ROD,
	.init			= mmci_variant_init,
};

static struct variant_data variant_u300 = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg_enable		= MCI_ST_U300_HWFCEN,
	.clkreg_8bit_bus_enable = MCI_ST_8BIT_BUS,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datalength_bits	= 16,
	.datactrl_blocksz	= 11,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.st_sdio			= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.f_max			= 100000000,
	.signal_direction	= true,
	.pwrreg_clkgate		= true,
	.pwrreg_nopower		= true,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_OD,
	.init			= mmci_variant_init,
};

static struct variant_data variant_nomadik = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_8bit_bus_enable = MCI_ST_8BIT_BUS,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datalength_bits	= 24,
	.datactrl_blocksz	= 11,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.st_sdio		= true,
	.st_clkdiv		= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.f_max			= 100000000,
	.signal_direction	= true,
	.pwrreg_clkgate		= true,
	.pwrreg_nopower		= true,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_OD,
	.init			= mmci_variant_init,
};

static struct variant_data variant_ux500 = {
	.fifosize		= 30 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_enable		= MCI_ST_UX500_HWFCEN,
	.clkreg_8bit_bus_enable = MCI_ST_8BIT_BUS,
	.clkreg_neg_edge_enable	= MCI_ST_UX500_NEG_EDGE,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datalength_bits	= 24,
	.datactrl_blocksz	= 11,
	.datactrl_any_blocksz	= true,
	.dma_power_of_2		= true,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.st_sdio		= true,
	.st_clkdiv		= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.f_max			= 100000000,
	.signal_direction	= true,
	.pwrreg_clkgate		= true,
	.busy_detect		= true,
	.busy_dpsm_flag		= MCI_DPSM_ST_BUSYMODE,
	.busy_detect_flag	= MCI_ST_CARDBUSY,
	.busy_detect_mask	= MCI_ST_BUSYENDMASK,
	.pwrreg_nopower		= true,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_OD,
	.init			= ux500_variant_init,
};

static struct variant_data variant_ux500v2 = {
	.fifosize		= 30 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_enable		= MCI_ST_UX500_HWFCEN,
	.clkreg_8bit_bus_enable = MCI_ST_8BIT_BUS,
	.clkreg_neg_edge_enable	= MCI_ST_UX500_NEG_EDGE,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.datactrl_mask_ddrmode	= MCI_DPSM_ST_DDRMODE,
	.datalength_bits	= 24,
	.datactrl_blocksz	= 11,
	.datactrl_any_blocksz	= true,
	.dma_power_of_2		= true,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.st_sdio		= true,
	.st_clkdiv		= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.f_max			= 100000000,
	.signal_direction	= true,
	.pwrreg_clkgate		= true,
	.busy_detect		= true,
	.busy_dpsm_flag		= MCI_DPSM_ST_BUSYMODE,
	.busy_detect_flag	= MCI_ST_CARDBUSY,
	.busy_detect_mask	= MCI_ST_BUSYENDMASK,
	.pwrreg_nopower		= true,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_OD,
	.init			= ux500v2_variant_init,
};

static struct variant_data variant_stm32 = {
	.fifosize		= 32 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_enable		= MCI_ST_UX500_HWFCEN,
	.clkreg_8bit_bus_enable = MCI_ST_8BIT_BUS,
	.clkreg_neg_edge_enable	= MCI_ST_UX500_NEG_EDGE,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.datalength_bits	= 24,
	.datactrl_blocksz	= 11,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.st_sdio		= true,
	.st_clkdiv		= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.f_max			= 48000000,
	.pwrreg_clkgate		= true,
	.pwrreg_nopower		= true,
	.dma_flow_controller	= true,
	.init			= mmci_variant_init,
};

static struct variant_data variant_stm32_sdmmc = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.f_max			= 208000000,
	.stm32_clkdiv		= true,
	.cmdreg_cpsm_enable	= MCI_CPSM_STM32_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_STM32_LRSP_CRC,
	.cmdreg_srsp_crc	= MCI_CPSM_STM32_SRSP_CRC,
	.cmdreg_srsp		= MCI_CPSM_STM32_SRSP,
	.cmdreg_stop		= MCI_CPSM_STM32_CMDSTOP,
	.data_cmd_enable	= MCI_CPSM_STM32_CMDTRANS,
	.irq_pio_mask		= MCI_IRQ_PIO_STM32_MASK,
	.datactrl_first		= true,
	.datacnt_useless	= true,
	.datalength_bits	= 25,
	.datactrl_blocksz	= 14,
	.datactrl_any_blocksz	= true,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.stm32_idmabsize_mask	= GENMASK(12, 5),
	.stm32_idmabsize_align	= BIT(5),
	.supports_sdio_irq	= true,
	.busy_timeout		= true,
	.busy_detect		= true,
	.busy_detect_flag	= MCI_STM32_BUSYD0,
	.busy_detect_mask	= MCI_STM32_BUSYD0ENDMASK,
	.init			= sdmmc_variant_init,
};

static struct variant_data variant_stm32_sdmmcv2 = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.f_max			= 267000000,
	.stm32_clkdiv		= true,
	.cmdreg_cpsm_enable	= MCI_CPSM_STM32_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_STM32_LRSP_CRC,
	.cmdreg_srsp_crc	= MCI_CPSM_STM32_SRSP_CRC,
	.cmdreg_srsp		= MCI_CPSM_STM32_SRSP,
	.cmdreg_stop		= MCI_CPSM_STM32_CMDSTOP,
	.data_cmd_enable	= MCI_CPSM_STM32_CMDTRANS,
	.irq_pio_mask		= MCI_IRQ_PIO_STM32_MASK,
	.datactrl_first		= true,
	.datacnt_useless	= true,
	.datalength_bits	= 25,
	.datactrl_blocksz	= 14,
	.datactrl_any_blocksz	= true,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.stm32_idmabsize_mask	= GENMASK(16, 5),
	.stm32_idmabsize_align	= BIT(5),
	.supports_sdio_irq	= true,
	.dma_lli		= true,
	.busy_timeout		= true,
	.busy_detect		= true,
	.busy_detect_flag	= MCI_STM32_BUSYD0,
	.busy_detect_mask	= MCI_STM32_BUSYD0ENDMASK,
	.init			= sdmmc_variant_init,
};

static struct variant_data variant_stm32_sdmmcv3 = {
	.fifosize		= 256 * 4,
	.fifohalfsize		= 128 * 4,
	.f_max			= 267000000,
	.stm32_clkdiv		= true,
	.cmdreg_cpsm_enable	= MCI_CPSM_STM32_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_STM32_LRSP_CRC,
	.cmdreg_srsp_crc	= MCI_CPSM_STM32_SRSP_CRC,
	.cmdreg_srsp		= MCI_CPSM_STM32_SRSP,
	.cmdreg_stop		= MCI_CPSM_STM32_CMDSTOP,
	.data_cmd_enable	= MCI_CPSM_STM32_CMDTRANS,
	.irq_pio_mask		= MCI_IRQ_PIO_STM32_MASK,
	.datactrl_first		= true,
	.datacnt_useless	= true,
	.datalength_bits	= 25,
	.datactrl_blocksz	= 14,
	.datactrl_any_blocksz	= true,
	.datactrl_mask_sdio	= MCI_DPSM_ST_SDIOEN,
	.stm32_idmabsize_mask	= GENMASK(16, 6),
	.stm32_idmabsize_align	= BIT(6),
	.supports_sdio_irq	= true,
	.dma_lli		= true,
	.busy_timeout		= true,
	.busy_detect		= true,
	.busy_detect_flag	= MCI_STM32_BUSYD0,
	.busy_detect_mask	= MCI_STM32_BUSYD0ENDMASK,
	.init			= sdmmc_variant_init,
};

static struct variant_data variant_qcom = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_enable		= MCI_QCOM_CLK_FLOWENA |
				  MCI_QCOM_CLK_SELECT_IN_FBCLK,
	.clkreg_8bit_bus_enable = MCI_QCOM_CLK_WIDEBUS_8,
	.datactrl_mask_ddrmode	= MCI_QCOM_CLK_SELECT_IN_DDR_MODE,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.data_cmd_enable	= MCI_CPSM_QCOM_DATCMD,
	.datalength_bits	= 24,
	.datactrl_blocksz	= 11,
	.datactrl_any_blocksz	= true,
	.pwrreg_powerup		= MCI_PWR_UP,
	.f_max			= 208000000,
	.explicit_mclk_control	= true,
	.qcom_fifo		= true,
	.qcom_dml		= true,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	.start_err		= MCI_STARTBITERR,
	.opendrain		= MCI_ROD,
	.init			= qcom_variant_init,
};

/*
 * MSM8660-specific variant.  Layered on top of variant_qcom to keep all
 * known-to-work behavior for modern Qualcomm SDCC instances (MSM8916,
 * MSM8974, SDM845, ...) untouched.  Selected only when the DT node
 * carries the "qcom,msm8660-sdcc-v4" compatible (see mmci_probe), which
 * is exclusive to the v4 PrimeCell wired to MSM8660/APQ8060.  Sashiko
 * High #44 on submit/mmci-qcom-tenderloin (2026-06-14): the original
 * code mutated variant_qcom in place, which would propagate to every
 * Qualcomm SoC sharing the amba_id 0x00051180 entry.
 *
 * Differences vs variant_qcom:
 *  - qcom_datactrl_delay:    250 us settle after every DATACTRL write
 *                            because the v4 SDCC needs the FBCLK
 *                            domain to re-sync before CPSM kick.
 *  - qcom_data_timeout_2x:   v4 SDCC undercounts DATALENGTH-derived
 *                            timeout vs actual MCLK; doubling matches
 *                            legacy msm_sdcc behaviour.
 *  - qcom_dml_atomic_submit: ADM exec_func atomic-submit hook for
 *                            datactrl_first hosts.  Sub-gated on
 *                            host->datactrl_first (per-host DT flag)
 *                            inside mmci_should_atomic_submit().
 *  - dma_flow_controller:    qcom_adm is the flow controller on v4.
 *  - dma_threshold:          32 B floor; anything above the half-FIFO
 *                            goes through ADM.  Avoids the PIO IRQ
 *                            storm during AR6003 firmware download
 *                            that delays sdcc1 mmci CMD/RESP processing
 *                            on the same CPU and trips DATACRCFAIL.
 *  - supports_sdio_irq:      legacy msm_sdcc routes the in-band SDIO
 *                            DAT1 IRQ -- required for AR6003 CMD53
 *                            RX scheduling.
 *  - start_err cleared:      MCI_STARTBITERR is a false positive on v4
 *                            (legacy msm_sdcc never enables it).
 *                            Enabling it produces spurious -ECOMM on
 *                            otherwise-good SDIO commands.
 */
static struct variant_data variant_qcom_msm8660 = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_enable		= MCI_QCOM_CLK_FLOWENA |
				  MCI_QCOM_CLK_SELECT_IN_FBCLK,
	.clkreg_8bit_bus_enable = MCI_QCOM_CLK_WIDEBUS_8,
	.datactrl_mask_ddrmode	= MCI_QCOM_CLK_SELECT_IN_DDR_MODE,
	.cmdreg_cpsm_enable	= MCI_CPSM_ENABLE,
	.cmdreg_lrsp_crc	= MCI_CPSM_RESPONSE | MCI_CPSM_LONGRSP,
	.cmdreg_srsp_crc	= MCI_CPSM_RESPONSE,
	.cmdreg_srsp		= MCI_CPSM_RESPONSE,
	.data_cmd_enable	= MCI_CPSM_QCOM_DATCMD,
	.datalength_bits	= 24,
	.datactrl_blocksz	= 11,
	.datactrl_any_blocksz	= true,
	.pwrreg_powerup		= MCI_PWR_UP,
	.f_max			= 208000000,
	.explicit_mclk_control	= true,
	.qcom_fifo		= true,
	.qcom_dml		= true,
	.qcom_datactrl_delay	= true,
	.qcom_data_timeout_2x	= true,
	.qcom_dml_atomic_submit	= true,
	.dma_flow_controller	= true,
	.dma_threshold		= 32,
	.mmcimask1		= true,
	.irq_pio_mask		= MCI_IRQ_PIO_MASK,
	/* No .start_err -- see above. */
	.opendrain		= MCI_ROD,
	.supports_sdio_irq	= true,
	.init			= qcom_variant_init,
};

/* Busy detection for the ST Micro variant */
static int mmci_card_busy(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;
	int busy = 0;

	spin_lock_irqsave(&host->lock, flags);
	if (readl(host->base + MMCISTATUS) & host->variant->busy_detect_flag)
		busy = 1;
	spin_unlock_irqrestore(&host->lock, flags);

	return busy;
}

static void mmci_reg_delay(struct mmci_host *host)
{
	/*
	 * According to the spec, at least three feedback clock cycles
	 * of max 52 MHz must pass between two writes to the MMCICLOCK reg.
	 * Three MCLK clock cycles must pass between two MMCIPOWER reg writes.
	 * Worst delay time during card init is at 100 kHz => 30 us.
	 * Worst delay time when up and running is at 25 MHz => 120 ns.
	 */
	if (host->cclk < 25000000)
		udelay(30);
	else
		ndelay(120);
}

/*
 * This must be called with host->lock held
 */
void mmci_write_clkreg(struct mmci_host *host, u32 clk)
{
	if (host->clk_reg != clk) {
		host->clk_reg = clk;
		writel(clk, host->base + MMCICLOCK);
	}
}

/*
 * This must be called with host->lock held
 */
void mmci_write_pwrreg(struct mmci_host *host, u32 pwr)
{
	if (host->pwr_reg != pwr) {
		host->pwr_reg = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}
}

/*
 * This must be called with host->lock held
 */
static void mmci_write_datactrlreg(struct mmci_host *host, u32 datactrl)
{
	/* Keep busy mode in DPSM and SDIO mask if enabled */
	datactrl |= host->datactrl_reg & (host->variant->busy_dpsm_flag |
					  host->variant->datactrl_mask_sdio);

	if (host->datactrl_reg != datactrl) {
		host->datactrl_reg = datactrl;
		writel(datactrl, host->base + MMCIDATACTRL);
	}
}

/*
 * This must be called with host->lock held
 */
static void mmci_set_clkreg(struct mmci_host *host, unsigned int desired)
{
	struct variant_data *variant = host->variant;
	u32 clk = variant->clkreg;

	/* Make sure cclk reflects the current calculated clock */
	host->cclk = 0;

	if (desired) {
		if (variant->explicit_mclk_control) {
			host->cclk = host->mclk;
		} else if (desired >= host->mclk) {
			clk = MCI_CLK_BYPASS;
			if (variant->st_clkdiv)
				clk |= MCI_ST_UX500_NEG_EDGE;
			host->cclk = host->mclk;
		} else if (variant->st_clkdiv) {
			/*
			 * DB8500 TRM says f = mclk / (clkdiv + 2)
			 * => clkdiv = (mclk / f) - 2
			 * Round the divider up so we don't exceed the max
			 * frequency
			 */
			clk = DIV_ROUND_UP(host->mclk, desired) - 2;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (clk + 2);
		} else {
			/*
			 * PL180 TRM says f = mclk / (2 * (clkdiv + 1))
			 * => clkdiv = mclk / (2 * f) - 1
			 */
			clk = host->mclk / (2 * desired) - 1;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (2 * (clk + 1));
		}

		clk |= variant->clkreg_enable;
		clk |= MCI_CLK_ENABLE;
		/*
		 * MCI_CLK_PWRSAVE auto-gates the SDC bus clock between
		 * transfers. Legacy webOS enables it on both eMMC (SDCC1
		 * CLKREG=0x9f00) and WiFi (SDCC4 CLKREG=0x9b00) in steady
		 * state. We match that here: PWRSAVE on by default once past
		 * 400 kHz identification.
		 *
		 * Earlier attempts to enable PWRSAVE on SDIO regressed BMI
		 * with DATACRCFAIL because the SDCC's data-path state
		 * machine does not fully close after a CMD53 WRITE; once
		 * SCLK gates the next CMD on the bus sees residual state.
		 * The "qcom,dummy52-required" workaround in this driver
		 * inserts a CMD52 between CMD53 writes and the next request
		 * to drain that state, making PWRSAVE-on-SDIO safe again.
		 */
		if (variant->qcom_datactrl_delay && desired > 400000 &&
		    qcom_pwrsave)
			clk |= MCI_CLK_PWRSAVE;
	}

	/* Set actual clock for debug */
	host->mmc->actual_clock = host->cclk;

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		clk |= MCI_4BIT_BUS;
	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_8)
		clk |= variant->clkreg_8bit_bus_enable;

	if (host->mmc->ios.timing == MMC_TIMING_UHS_DDR50 ||
	    host->mmc->ios.timing == MMC_TIMING_MMC_DDR52)
		clk |= variant->clkreg_neg_edge_enable;

	/*
	 * Diagnostic: observe what clkreg + state we end up with for each
	 * controller instance. Gated on qcom_datactrl_delay so only the qcom
	 * variant emits this. dev_dbg (off by default, enable via dynamic
	 * debug) when chasing eMMC vs SDIO regressions where the per-instance
	 * PWRSAVE / datactrl_first / card-type combination matters.
	 */
	if (variant->qcom_datactrl_delay)
		dev_dbg(mmc_dev(host->mmc),
			"set_clkreg: desired=%u clk_reg=0x%08x datactrl_first=%d card_type=%d pwrsave=%s\n",
			desired, clk,
			host->datactrl_first,
			host->mmc->card ? host->mmc->card->type : -1,
			str_on_off(clk & MCI_CLK_PWRSAVE));

	mmci_write_clkreg(host, clk);

	/*
	 * Legacy webOS msm_sdcc waits 50 us after every MMCICLOCK write
	 * (msm_sdcc.c:1173 udelay(50)) before issuing further register
	 * writes or starting a data transfer. The generic mmci_reg_delay
	 * called by other callers only waits ndelay(120) once cclk is
	 * above 25 MHz — 400x shorter than legacy — which leaves the
	 * SDCC controller's clock-divider re-lock window straddling the
	 * subsequent DPSM start. On the Tenderloin SanDisk SEM32G that's
	 * enough to push the first large multi-block READ at HS rates
	 * past the data-line setup margin: DATACRCFAIL on the first
	 * 256 KB transfer after probe, recovery CMD6 SWITCH then times
	 * out and the bus falls back to 1-bit / 5.4 MB/s for the rest
	 * of the session.
	 *
	 * Match legacy's 50 us settle here so callers don't need to
	 * remember an extra delay. Gated on qcom_datactrl_delay so other
	 * mmci variants (PL180, ST, STM32, ux500) keep their lower
	 * ndelay path.
	 */
	if (variant->qcom_datactrl_delay)
		udelay(50);
}

static void mmci_dma_release(struct mmci_host *host)
{
	if (host->ops && host->ops->dma_release)
		host->ops->dma_release(host);

	host->use_dma = false;
}

static int mmci_dma_setup(struct mmci_host *host)
{
	int ret;

	if (!host->ops || !host->ops->dma_setup)
		return 0;

	ret = host->ops->dma_setup(host);
	if (ret)
		return ret;

	/* initialize pre request cookie */
	host->next_cookie = 1;

	host->use_dma = true;
	return 0;
}

/*
 * Validate mmc prerequisites
 */
static int mmci_validate_data(struct mmci_host *host,
			      struct mmc_data *data)
{
	struct variant_data *variant = host->variant;

	if (!data)
		return 0;
	if (!is_power_of_2(data->blksz) && !variant->datactrl_any_blocksz) {
		dev_err(mmc_dev(host->mmc),
			"unsupported block size (%d bytes)\n", data->blksz);
		return -EINVAL;
	}

	if (host->ops && host->ops->validate_data)
		return host->ops->validate_data(host, data);

	return 0;
}

static int mmci_prep_data(struct mmci_host *host, struct mmc_data *data, bool next)
{
	int err;

	if (!host->ops || !host->ops->prep_data)
		return 0;

	err = host->ops->prep_data(host, data, next);

	if (next && !err)
		data->host_cookie = ++host->next_cookie < 0 ?
			1 : host->next_cookie;

	return err;
}

static void mmci_unprep_data(struct mmci_host *host, struct mmc_data *data,
		      int err)
{
	if (host->ops && host->ops->unprep_data)
		host->ops->unprep_data(host, data, err);

	data->host_cookie = 0;
}

static void mmci_get_next_data(struct mmci_host *host, struct mmc_data *data)
{
	WARN_ON(data->host_cookie && data->host_cookie != host->next_cookie);

	if (host->ops && host->ops->get_next_data)
		host->ops->get_next_data(host, data);
}

/*
 * Decide whether a given (data) request qualifies for the
 * atomic-submission path.
 *
 * Legacy msm_sdcc uses TWO distinct structural sequences depending on
 * direction:
 *
 *   READ : DATACTRL + ARG + CMD all written inside msm_dmov exec_func,
 *          immediately before the channel's CMD_PTR write.  SDCC is
 *          armed (RX direction) BEFORE the card receives the read
 *          command, so by the time the card responds with data the
 *          DPSM is ready and ADM is pulling.
 *
 *   WRITE: CMD is written first (mmci_start_command), the card
 *          processes the command + responds, THEN start_data sets up
 *          DPSM (TX direction) inside exec_func with NO CMD (data
 *          arrives only after card is in receive state).  Trying to
 *          arm the WRITE-direction DPSM before the card has accepted
 *          the command causes CRC errors on the data the SDCC sends
 *          (AR6003 reports -84 EILSEQ on WRITE-data CRC check).
 *
 * Match that asymmetry: atomic-submit only for READS.  WRITES fall
 * through to the conventional deferred-DMA path which already mirrors
 * the legacy CMD-first write sequence.
 */
static inline bool mmci_should_atomic_submit(struct mmci_host *host,
					     struct mmc_data *data)
{
	return host->variant->qcom_dml_atomic_submit &&
	       host->datactrl_first &&
	       (data->flags & MMC_DATA_READ);
}

/*
 * Graded DMA->PIO recovery bookkeeping for the eMMC (mmc0) on the qcom ADM
 * variant. Counts consecutive data errors; after the second consecutive
 * flush, arm a one-shot PIO retry (consumed in mmci_dma_start()) so a read
 * that keeps overrunning the ADM drain completes in PIO instead of failing
 * the mount. A clean transfer resets the counter, so DMA stays the norm and
 * full-speed runtime throughput is preserved. Scoped to mmc0 on the ADM
 * variant (qcom_datactrl_delay) so SDIO (mmc1) and non-qcom hosts are
 * untouched.
 */
static void mmci_qcom_grade_recovery(struct mmci_host *host,
				     struct mmc_data *data)
{
	if (!host->variant->qcom_datactrl_delay || host->mmc->index != 0)
		return;

	if (data->error) {
		if (++host->qcom_flush_count >= 2) {
			host->qcom_force_pio = true;
			host->qcom_flush_count = 0;
		}
	} else {
		host->qcom_flush_count = 0;
	}
}

static int mmci_dma_start(struct mmci_host *host, unsigned int datactrl)
{
	struct mmc_data *data = host->data;
	int ret;

	/*
	 * One-shot PIO fallback: a previous read flushed the ADM drain
	 * repeatedly, so retry this single transfer in PIO (which never
	 * touches the ADM). Cleared here so the very next transfer is back
	 * on DMA -- we do NOT call mmci_dma_release(), which would kill DMA
	 * for the rest of the session.
	 */
	if (host->qcom_force_pio) {
		host->qcom_force_pio = false;
		dev_warn_ratelimited(mmc_dev(host->mmc),
			"eMMC: PIO fallback for this transfer after repeated ADM drain flush\n");
		return -EINVAL;
	}

	if (!host->use_dma)
		return -EINVAL;

	ret = mmci_prep_data(host, data, false);
	if (ret)
		return ret;

	if (!host->ops || !host->ops->dma_start)
		return -EINVAL;

	/* Okay, go for it. */
	dev_vdbg(mmc_dev(host->mmc),
		 "Submit MMCI DMA job, sglen %d blksz %04x blks %04x flags %08x\n",
		 data->sg_len, data->blksz, data->blocks, data->flags);

	ret = host->ops->dma_start(host, &datactrl);
	if (ret)
		return ret;

	/*
	 * Commit to the atomic-submission path only AFTER dma_start
	 * succeeded. If dma_start had failed we return above, so the PIO
	 * fallback in mmci_start_data + mmci_start_command keeps the
	 * conventional write path (active stays false).
	 *
	 * Gate on mmci_should_atomic_submit() rather than the host->
	 * atomic_submit.armed flag: armed is set during prep but cleared at
	 * __mmci_start_request() entry, and on the EXT_CSD/eMMC read path it
	 * was not reliably re-set before this latch — so active never became
	 * true, mmci_start_command() skipped stashing ARG/CMD, and
	 * mmci_qcom_atomic_exec_func() fired with an empty stash
	 * (datactrl=0/cmd=0) leaving the SDCC unarmed and the ADM waiting on
	 * a CRCI forever. should_atomic_submit() is recomputed here from
	 * stable inputs (variant->qcom_dml_atomic_submit + host->
	 * datactrl_first + READ) and cannot be lost. It is BAM-safe:
	 * qcom_dml_atomic_submit is set only on the ADM variant
	 * (variant_qcom_msm8660); BAM hosts use variant_qcom and never reach
	 * here. The exec_func itself is wired in _mmci_dmae_prep_data only
	 * when dmae->crci is present, which every datactrl-first host on this
	 * SoC has.
	 */
	if (mmci_should_atomic_submit(host, data))
		host->atomic_submit.active = true;

	if (host->atomic_submit.active) {
		/*
		 * Atomic-submission path (variant->qcom_dml_atomic_submit +
		 * qualifying write). Don't write DATACTRL here; stash it
		 * for the qcom_adm exec_func, which will write
		 * DATACTRL + ARG + CMD atomically with the ADM CMD_PTR.
		 * mmci_request will dma_issue_pending after the matching
		 * mmci_start_command has stashed its ARG/CMD values too.
		 */
		host->atomic_submit.datactrl = datactrl;
	} else {
		bool defer_for_adm_write =
			host->ops && host->ops->dma_issue_pending &&
			host->datactrl_first &&
			!(data->flags & MMC_DATA_READ) &&
			host->variant->qcom_dml_atomic_submit;

		if (defer_for_adm_write) {
			/*
			 * ADM WRITES must follow legacy webOS msm_sdcc's
			 * CMD-first order, for BOTH SDIO (sdcc4/WiFi CMD53) and
			 * eMMC (sdcc1 CMD25): the data command is written first
			 * (mmci_start_command), and only on the CMDRESPEND IRQ —
			 * once the card has accepted the command and entered the
			 * receive state — does mmci_cmd_irq write DATACTRL and
			 * kick the ADM (dma_issue_pending). The DPSM is therefore
			 * never armed in WRITE direction before the card is ready.
			 *
			 * Mainline historically wrote DATACTRL here and issued the
			 * ADM immediately, BEFORE mmci_start_command sent the data
			 * command. That arms the write-direction DPSM + DMAENABLE
			 * and starts the ADM filling the FIFO microseconds before
			 * the command reaches the card. On SDIO/AR6003 the chip
			 * NAKed CMD53 in that window; on eMMC the write wedged —
			 * ADM ch2 fills the FIFO and waits on a CRCI the SDCC never
			 * asserts (card not yet receiving), no RSLT_VALID, 500 ms
			 * watchdog (ADM-WATCHDOG xfer: WRITE blocks=16). This is
			 * the write-direction twin of the read arming gap the
			 * atomic-submit path fixes; writes have no atomic path
			 * (mmci_should_atomic_submit is read-only), so they use
			 * this deferral instead.
			 *
			 * Stash DATACTRL here; mmci_cmd_irq writes it just before
			 * dma_issue_pending() on CMDRESPEND -- matching legacy
			 * exec_func: card ACKs CMD -> DATACTRL -> udelay -> ADM
			 * kicks -> CRCI handshake -> data on wire.
			 */
			host->deferred_datactrl = datactrl;
			host->deferred_datactrl_pending = true;
			host->dma_issue_deferred = true;
		} else {
			/* Trigger the DMA transfer */
			mmci_write_datactrlreg(host, datactrl);

			/*
			 * Qualcomm SDCC requires a delay after writing DATACTRL
			 * to allow the Data Path State Machine (DPSM) to
			 * initialize before DMA starts pushing data into the
			 * FIFO. The legacy msm_sdcc driver uses writel_delay()
			 * with 1us after DATACTRL, followed by CMD register
			 * writes which add additional delay before DMA data
			 * flow.
			 */
			if (host->variant->qcom_datactrl_delay) {
				wmb();
				udelay(1);
			}

			/*
			 * Defer the ADM CMD_PTR kick to the CMDRESPEND IRQ for
			 * ADM-flow-controlled READS on this variant. The MSM8660
			 * ADM latches CRCI state when CMD_PTR is written: if we
			 * kick it HERE (before mmci_start_command sends the data
			 * command), the channel arms while CRCI is idle and the
			 * card hasn't streamed yet — and it never re-samples when
			 * the card later fills the SDCC FIFO and asserts CRCI. The
			 * channel sits with the command queued (STATUS CMD_COUNT=1)
			 * forever; the SDCC FIFO fills to RXFIFOFULL and stalls,
			 * never drained. Proven on eMMC ch2 by a live FIFO poll
			 * (datactrl=0x200b, STATUS RXFIFOFULL+RXDATAAVLBL, ADM
			 * transferred 0 rows) vs the webOS fingerprint where the
			 * same channel cycles normally.
			 *
			 * Legacy webOS arch/arm/mach-msm/dma.c issues CMD_PTR from
			 * msm_dmov_enqueue_cmd_ext AFTER msmsdcc_dma_exec_func has
			 * already written DATACTRL + the data command — i.e. the
			 * card is already streaming before CMD_PTR. DATACTRL stays
			 * written inline here (DPSM armed before the command, as
			 * legacy does); only the CMD_PTR kick is deferred to
			 * CMDRESPEND so it lands after the command, matching
			 * legacy ordering. SDIO writes use the separate
			 * defer_for_sdio_write path above (which also defers
			 * DATACTRL); this read path keeps DATACTRL inline.
			 */
			if (host->variant->qcom_dml_atomic_submit &&
			    (data->flags & MMC_DATA_READ) &&
			    host->ops && host->ops->dma_issue_pending) {
				host->dma_issue_deferred = true;
				host->deferred_datactrl_pending = false;
			} else if (host->ops && host->ops->dma_issue_pending) {
				host->ops->dma_issue_pending(host);
			}
		}
	}

	/*
	 * Let the MMCI say when the data is ended and it's time
	 * to fire next DMA request. When that happens, MMCI will
	 * call mmci_data_end()
	 *
	 * Legacy webOS msm_sdcc parity (tenderloin AR6003/eMMC concurrent
	 * stress fix): ALSO enable the FIFO half-full/half-empty mask even
	 * on the DMA success path.  Legacy webOS sets pio_irqmask to
	 * MCI_RXFIFOHALFFULLMASK (reads) or MCI_TXFIFOHALFEMPTYMASK (writes)
	 * before kicking the DMA, giving SDCC a safety net: if the ADM
	 * stalls mid-burst (FLUSH_STATE5 = drain-stage-3, see
	 * project_adm_flush_state_decode_2026_06_14), the FIFO IRQ fires
	 * with the channel pipeline frozen, and the host gets a chance to
	 * notice and react rather than wait for DATATIMER to expire (which
	 * is what we observed: DATACRCFAIL with errbits "(none latched)
	 * (RXOVERRUN=fabric/ADM drain starvation)").
	 *
	 * Live /dev/mem dump on running webOS confirms: SDC1 MCIMASK0 =
	 * 0x008001ff (low 9 bits + bit 15 RXFIFOHALFFULLMASK + bit 23
	 * CEATAENDMASK).  Mainline's DIAG[DATACRCFAIL] capture at the same
	 * stall instant: MASK0 = 0x000001ff (bit 15 NOT set).
	 *
	 * The CEATA mask is irrelevant on eMMC; the load-bearing bit is
	 * RXFIFOHALFFULLMASK / TXFIFOHALFEMPTYMASK for the safety net.
	 *
	 * mmci_pio_irq() bails out early when host->dma_in_progress is set
	 * (see that handler), so this extra mask cannot drive the PIO drain
	 * path into an uninitialised sg_miter.  The cost is one extra IRQ
	 * per FIFO half-transition during normal DMA reads (~100 kHz on a
	 * 4-bit 48 MHz bus with 64 B FIFO -- fast, but harmless: each IRQ
	 * is just an ACK that returns IRQ_HANDLED immediately).
	 *
	 * Gated on qcom_dml_atomic_submit so non-qcom variants are unaffected.
	 */
	if (host->variant->qcom_dml_atomic_submit) {
		u32 extra_mask;

		if (data->flags & MMC_DATA_READ)
			extra_mask = MCI_RXFIFOHALFFULLMASK;
		else
			extra_mask = MCI_TXFIFOHALFEMPTYMASK;

		writel(readl(host->base + MMCIMASK0) | MCI_DATAENDMASK |
				extra_mask,
		       host->base + MMCIMASK0);
	} else {
		writel(readl(host->base + MMCIMASK0) | MCI_DATAENDMASK,
		       host->base + MMCIMASK0);
	}

	/* Trace mask setup for WiFi SDCC4 debugging */
	if (host->mmc->index == 1)
		mmci_qcom_diag("MMCI-DMA-START: MASK0=0x%08x (enabled DATAENDMASK) blksz=%u blocks=%u\n",
			     readl(host->base + MMCIMASK0), data->blksz, data->blocks);

	return 0;
}

static void mmci_dma_finalize(struct mmci_host *host, struct mmc_data *data)
{
	if (!host->use_dma)
		return;

	if (host->ops && host->ops->dma_finalize)
		host->ops->dma_finalize(host, data);
}

static void mmci_dma_error(struct mmci_host *host)
{
	if (!host->use_dma)
		return;

	if (host->ops && host->ops->dma_error)
		host->ops->dma_error(host);
}

/* Forward declaration: used by mmci_cmd_irq dummy52-completion path. */
static void __mmci_start_request(struct mmci_host *host,
				 struct mmc_request *mrq);

/* Forward declaration: used by mmci_qcom_dma_complete() for data->stop. */
static void mmci_start_command(struct mmci_host *host,
			       struct mmc_command *cmd, u32 c);

static void
mmci_request_end(struct mmci_host *host, struct mmc_request *mrq)
{
	/* Trace request end for WiFi SDCC4 debugging */
	if (host->mmc->index == 1 && mrq && mrq->data)
		mmci_qcom_diag("MMCI-REQ-END: cmd=%u blksz=%u blocks=%u\n",
			     mrq->cmd->opcode, mrq->data->blksz, mrq->data->blocks);

	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	/*
	 * Qualcomm SDCC dummy CMD52 errata: arm the follow-up if the just
	 * completed request was a CMD53 with data. The next call to
	 * mmci_request will insert a CMD52 before the real command so the
	 * SDCC data-path state machine drains cleanly.
	 *
	 * WiFi (mmc1): arm after DMA READ only (>= 128B, which uses DMA via
	 * validate_dma). The SDCC DPSM retains residual state after ADM DMA
	 * read completion (via callback) that causes the next CMD53 WRITE to
	 * CMDTIMEOUT (observed after 6 HTC 128B service-connect reads →
	 * htc_start write). NOT arming after writes keeps the dummy52 out of
	 * the BMI firmware upload path (all writes). Small PIO reads (< 128B,
	 * like BMI register reads) don't need dummy52 — only DMA reads trigger
	 * the DPSM issue.
	 * eMMC (mmc0): arm after WRITE only (original behavior).
	 */
	if (host->dummy52_required && mrq && mrq->cmd &&
	    mrq->data && mrq->cmd->opcode == SD_IO_RW_EXTENDED) {
		unsigned int len = mrq->data->blksz * mrq->data->blocks;
		bool arm = (host->mmc->index == 1 &&
			    (mrq->data->flags & MMC_DATA_READ) && len >= 128) ||
			   (host->mmc->index == 0 &&
			    (mrq->data->flags & MMC_DATA_WRITE));
		if (arm) {
			host->dummy52_needed = true;
			dev_dbg(mmc_dev(host->mmc),
				"dummy52: armed after CMD53 %s %u bytes\n",
				(mrq->data->flags & MMC_DATA_WRITE) ?
				"WRITE" : "READ", len);
		}
	}

	mmc_request_done(host->mmc, mrq);
}

static void mmci_set_mask1(struct mmci_host *host, unsigned int mask)
{
	void __iomem *base = host->base;
	struct variant_data *variant = host->variant;

	if (host->singleirq) {
		unsigned int mask0 = readl(base + MMCIMASK0);

		mask0 &= ~variant->irq_pio_mask;
		mask0 |= mask;

		writel(mask0, base + MMCIMASK0);
	}

	if (variant->mmcimask1)
		writel(mask, base + MMCIMASK1);

	host->mask1_reg = mask;
}

static void mmci_stop_data(struct mmci_host *host)
{
	mmci_write_datactrlreg(host, 0);
	mmci_set_mask1(host, 0);
	host->data = NULL;
}

static void mmci_init_sg(struct mmci_host *host, struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
}

static u32 mmci_get_dctrl_cfg(struct mmci_host *host)
{
	return MCI_DPSM_ENABLE | mmci_dctrl_blksz(host);
}

static u32 ux500v2_get_dctrl_cfg(struct mmci_host *host)
{
	return MCI_DPSM_ENABLE | (host->data->blksz << 16);
}

static void ux500_busy_clear_mask_done(struct mmci_host *host)
{
	void __iomem *base = host->base;

	writel(host->variant->busy_detect_mask, base + MMCICLEAR);
	writel(readl(base + MMCIMASK0) &
	       ~host->variant->busy_detect_mask, base + MMCIMASK0);
	host->busy_state = MMCI_BUSY_DONE;
	host->busy_status = 0;
}

/*
 * ux500_busy_complete() - this will wait until the busy status
 * goes off, saving any status that occur in the meantime into
 * host->busy_status until we know the card is not busy any more.
 * The function returns true when the busy detection is ended
 * and we should continue processing the command.
 *
 * The Ux500 typically fires two IRQs over a busy cycle like this:
 *
 *  DAT0 busy          +-----------------+
 *                     |                 |
 *  DAT0 not busy  ----+                 +--------
 *
 *                     ^                 ^
 *                     |                 |
 *                    IRQ1              IRQ2
 */
static bool ux500_busy_complete(struct mmci_host *host, struct mmc_command *cmd,
				u32 status, u32 err_msk)
{
	void __iomem *base = host->base;
	int retries = 10;

	if (status & err_msk) {
		/* Stop any ongoing busy detection if an error occurs */
		ux500_busy_clear_mask_done(host);
		goto out_ret_state;
	}

	/*
	 * The state transitions are encoded in a state machine crossing
	 * the edges in this switch statement.
	 */
	switch (host->busy_state) {

	/*
	 * Before unmasking for the busy end IRQ, confirm that the
	 * command was sent successfully. To keep track of having a
	 * command in-progress, waiting for busy signaling to end,
	 * store the status in host->busy_status.
	 *
	 * Note that, the card may need a couple of clock cycles before
	 * it starts signaling busy on DAT0, hence re-read the
	 * MMCISTATUS register here, to allow the busy bit to be set.
	 */
	case MMCI_BUSY_DONE:
		/*
		 * Save the first status register read to be sure to catch
		 * all bits that may be lost will retrying. If the command
		 * is still busy this will result in assigning 0 to
		 * host->busy_status, which is what it should be in IDLE.
		 */
		host->busy_status = status & (MCI_CMDSENT | MCI_CMDRESPEND);
		while (retries) {
			status = readl(base + MMCISTATUS);
			/* Keep accumulating status bits */
			host->busy_status |= status & (MCI_CMDSENT | MCI_CMDRESPEND);
			if (status & host->variant->busy_detect_flag) {
				writel(readl(base + MMCIMASK0) |
				       host->variant->busy_detect_mask,
				       base + MMCIMASK0);
				host->busy_state = MMCI_BUSY_WAITING_FOR_START_IRQ;
				schedule_delayed_work(&host->ux500_busy_timeout_work,
				      msecs_to_jiffies(cmd->busy_timeout));
				goto out_ret_state;
			}
			retries--;
		}
		dev_dbg(mmc_dev(host->mmc),
			"no busy signalling in time CMD%02x\n", cmd->opcode);
		ux500_busy_clear_mask_done(host);
		break;

	/*
	 * If there is a command in-progress that has been successfully
	 * sent, then bail out if busy status is set and wait for the
	 * busy end IRQ.
	 *
	 * Note that, the HW triggers an IRQ on both edges while
	 * monitoring DAT0 for busy completion, but there is only one
	 * status bit in MMCISTATUS for the busy state. Therefore
	 * both the start and the end interrupts needs to be cleared,
	 * one after the other. So, clear the busy start IRQ here.
	 */
	case MMCI_BUSY_WAITING_FOR_START_IRQ:
		if (status & host->variant->busy_detect_flag) {
			host->busy_status |= status & (MCI_CMDSENT | MCI_CMDRESPEND);
			writel(host->variant->busy_detect_mask, base + MMCICLEAR);
			host->busy_state = MMCI_BUSY_WAITING_FOR_END_IRQ;
		} else {
			dev_dbg(mmc_dev(host->mmc),
				"lost busy status when waiting for busy start IRQ CMD%02x\n",
				cmd->opcode);
			cancel_delayed_work(&host->ux500_busy_timeout_work);
			ux500_busy_clear_mask_done(host);
		}
		break;

	case MMCI_BUSY_WAITING_FOR_END_IRQ:
		if (!(status & host->variant->busy_detect_flag)) {
			host->busy_status |= status & (MCI_CMDSENT | MCI_CMDRESPEND);
			writel(host->variant->busy_detect_mask, base + MMCICLEAR);
			cancel_delayed_work(&host->ux500_busy_timeout_work);
			ux500_busy_clear_mask_done(host);
		} else {
			dev_dbg(mmc_dev(host->mmc),
				"busy status still asserted when handling busy end IRQ - will keep waiting CMD%02x\n",
				cmd->opcode);
		}
		break;

	default:
		dev_dbg(mmc_dev(host->mmc), "fell through on state %d, CMD%02x\n",
			host->busy_state, cmd->opcode);
		break;
	}

out_ret_state:
	return (host->busy_state == MMCI_BUSY_DONE);
}

/*
 * All the DMA operation mode stuff goes inside this ifdef.
 * This assumes that you have a generic DMA device interface,
 * no custom DMA interfaces are supported.
 */
#ifdef CONFIG_DMA_ENGINE
struct mmci_dmae_next {
	struct dma_async_tx_descriptor *desc;
	struct dma_chan	*chan;
};

struct mmci_dmae_priv {
	struct dma_chan	*cur;
	struct dma_chan	*rx_channel;
	struct dma_chan	*tx_channel;
	struct dma_async_tx_descriptor	*desc_current;
	/*
	 * Cookie of the currently in-flight descriptor, stashed by
	 * mmci_dmae_submit() because the descriptor pointer at
	 * desc_current is not guaranteed to carry the original cookie
	 * by the time the dmaengine callback fires -- the qcom_adm pool
	 * path (adm_desc_get -> memset &desc->vd) recycles descriptor
	 * structs across submissions and clears the cookie field.
	 * Queried by mmci_qcom_dma_complete() via dmaengine_tx_status()
	 * to detect ADM-internal RSLT_ERR that would otherwise leave
	 * MMCISTATUS clean and look like a successful transfer.
	 */
	dma_cookie_t curr_cookie;
	struct mmci_dmae_next next_data;
	u32 crci;	/* CRCI value for QCOM ADM DMA */
};

int mmci_dmae_setup(struct mmci_host *host)
{
	const char *rxname, *txname;
	struct mmci_dmae_priv *dmae;

	dmae = devm_kzalloc(mmc_dev(host->mmc), sizeof(*dmae), GFP_KERNEL);
	if (!dmae)
		return -ENOMEM;

	host->dma_priv = dmae;

	/* Read CRCI value for QCOM ADM DMA flow control */
	of_property_read_u32(mmc_dev(host->mmc)->of_node, "qcom,sdcc-crci",
			     &dmae->crci);

	dmae->rx_channel = dma_request_chan(mmc_dev(host->mmc), "rx");
	if (IS_ERR(dmae->rx_channel)) {
		int ret = PTR_ERR(dmae->rx_channel);
		dmae->rx_channel = NULL;
		return ret;
	}

	dmae->tx_channel = dma_request_chan(mmc_dev(host->mmc), "tx");
	if (IS_ERR(dmae->tx_channel)) {
		if (PTR_ERR(dmae->tx_channel) == -EPROBE_DEFER)
			dev_warn(mmc_dev(host->mmc),
				 "Deferred probe for TX channel ignored\n");
		dmae->tx_channel = NULL;
	}

	/*
	 * If only an RX channel is specified, the driver will
	 * attempt to use it bidirectionally, however if it
	 * is specified but cannot be located, DMA will be disabled.
	 */
	if (dmae->rx_channel && !dmae->tx_channel)
		dmae->tx_channel = dmae->rx_channel;

	if (dmae->rx_channel)
		rxname = dma_chan_name(dmae->rx_channel);
	else
		rxname = "none";

	if (dmae->tx_channel)
		txname = dma_chan_name(dmae->tx_channel);
	else
		txname = "none";

	dev_info(mmc_dev(host->mmc), "DMA channels RX %s, TX %s, CRCI %u\n",
		 rxname, txname, dmae->crci);

	/*
	 * Limit the maximum segment size in any SG entry according to
	 * the parameters of the DMA engine device.
	 */
	if (dmae->tx_channel) {
		struct device *dev = dmae->tx_channel->device->dev;
		unsigned int max_seg_size = dma_get_max_seg_size(dev);

		if (max_seg_size < host->mmc->max_seg_size)
			host->mmc->max_seg_size = max_seg_size;
	}
	if (dmae->rx_channel) {
		struct device *dev = dmae->rx_channel->device->dev;
		unsigned int max_seg_size = dma_get_max_seg_size(dev);

		if (max_seg_size < host->mmc->max_seg_size)
			host->mmc->max_seg_size = max_seg_size;
	}

	if (!dmae->tx_channel || !dmae->rx_channel) {
		mmci_dmae_release(host);
		return -EINVAL;
	}

	return 0;
}

/*
 * This is used in or so inline it
 * so it can be discarded.
 */
void mmci_dmae_release(struct mmci_host *host)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;

	if (dmae->rx_channel)
		dma_release_channel(dmae->rx_channel);
	if (dmae->tx_channel)
		dma_release_channel(dmae->tx_channel);
	dmae->rx_channel = dmae->tx_channel = NULL;
}

static void mmci_dma_unmap(struct mmci_host *host, struct mmc_data *data)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	struct dma_chan *chan;

	if (data->flags & MMC_DATA_READ)
		chan = dmae->rx_channel;
	else
		chan = dmae->tx_channel;

	dma_unmap_sg(chan->device->dev, data->sg, data->sg_len,
		     mmc_get_dma_dir(data));
}

void mmci_dmae_error(struct mmci_host *host)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	struct mmci_dmae_next *next = &dmae->next_data;
	struct dma_chan *chan = dmae->cur;

	if (!dma_inprogress(host))
		return;

	dev_err(mmc_dev(host->mmc), "error during DMA transfer!\n");
	dmaengine_terminate_all(chan);
	host->dma_in_progress = false;
	dmae->cur = NULL;
	dmae->desc_current = NULL;
	host->data->host_cookie = 0;

	/*
	 * Clear next_data if it was prepared on the same channel that we
	 * just terminated. dmaengine_terminate_all() frees all pending
	 * descriptors on the channel, so next->desc would be a stale
	 * pointer if next->chan matches the terminated channel.
	 *
	 * If next->chan is a different channel (e.g., RX vs TX), the
	 * descriptor may still be valid and should not be cleared.
	 */
	if (next->chan == chan) {
		next->desc = NULL;
		next->chan = NULL;
	}

	mmci_dma_unmap(host, host->data);
}

void mmci_dmae_finalize(struct mmci_host *host, struct mmc_data *data)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	u32 status;
	int i;

	if (!dma_inprogress(host))
		return;

	/*
	 * Skip the post-DMA RX-FIFO drain check for the qcom variant.
	 *
	 * The original check is a heuristic for PrimeCell-class DMA glue that
	 * can't monitor DMALBREQ/DMALSREQ: it polls MMCISTATUS for up to 1 ms
	 * waiting for MCI_RXDATAAVLBL to clear, and on failure calls
	 * mmci_dma_release() which disables DMA *for the rest of the session*.
	 *
	 * On MSM8660 / APQ8060 paired with the qcom_adm DMA engine, a SDIO
	 * byte-mode FIXED-address CMD53 read at the AR6003 mailbox (mbox
	 * 0x800, 128 B, blksz=128 blkcnt=1) finishes with status 0x0062a400 —
	 * MCI_RXDATAAVLBL stays asserted because SDCC clocks more bytes into
	 * the RX FIFO than ADM was asked to consume; the residue never drains
	 * within 1 ms and is also harmless (the next CMD53 setup clears the
	 * data path). Determinism: 15/15 modprobe iterations hit this exact
	 * status, then mmci_dma_release() permanently disables DMA, which
	 * fails the very next ath6kl HTC read with -EIO -> -ENOMEM.
	 *
	 * mmci_qcom_dma_complete() — the ADM dmaengine callback — already
	 * samples MMCISTATUS and honors any real SDCC errors (DATACRCFAIL,
	 * DATATIMEOUT, RXOVERRUN, STARTBITERR). The dmaengine RESULT path is
	 * authoritative that the requested bytes are in memory, so we don't
	 * lose error coverage by skipping this check.
	 *
	 * eMMC traffic on the same variant is unaffected: SDCC block-mode
	 * multi-block reads always finish with RXDATAAVLBL already clear, so
	 * the original loop would exit at i == 0 and the buggy-DMA branch
	 * would never trigger. We are removing a check that was already
	 * inactive on the eMMC path.
	 */
	if (host->variant->qcom_dml) {
		if (!data->host_cookie)
			mmci_dma_unmap(host, data);
		goto out;
	}

	/* Wait up to 1ms for the DMA to complete */
	for (i = 0; ; i++) {
		status = readl(host->base + MMCISTATUS);
		if (!(status & MCI_RXDATAAVLBLMASK) || i >= 100)
			break;
		udelay(10);
	}

	/*
	 * Check to see whether we still have some data left in the FIFO -
	 * this catches DMA controllers which are unable to monitor the
	 * DMALBREQ and DMALSREQ signals while allowing us to DMA to non-
	 * contiguous buffers.  On TX, we'll get a FIFO underrun error.
	 */
	if (status & MCI_RXDATAAVLBLMASK) {
		mmci_dma_error(host);
		if (!data->error)
			data->error = -EIO;
	} else if (!data->host_cookie) {
		mmci_dma_unmap(host, data);
	}

	/*
	 * Use of DMA with scatter-gather is impossible.
	 * Give up with DMA and switch back to PIO mode.
	 */
	if (status & MCI_RXDATAAVLBLMASK) {
		dev_err(mmc_dev(host->mmc),
			"buggy DMA detected: status=0x%08x after %d iters, dir=%s, blksz=%u, blkcnt=%u — disabling DMA\n",
			status, i,
			str_read_write(data->flags & MMC_DATA_READ),
			data->blksz, data->blocks);
		mmci_dma_release(host);
	}

out:
	host->dma_in_progress = false;
	dmae->cur = NULL;
	dmae->desc_current = NULL;
}

/*
 * Qualcomm SDCC ADM-DMA completion callback (reads AND writes).
 *
 * On MSM8660/APQ8060 (Tenderloin) the SDCC Data Path State Machine does
 * NOT reliably raise MCI_DATAEND after a DMA transfer. For reads only
 * MCI_DATABLOCKEND fires (masked out of MMCIMASK0); mmci_data_irq()
 * completes a transfer solely on MCI_DATAEND, so a missed DATAEND means
 * the request never completes. Reads then hang forever in
 * mmc_wait_for_req_done() (AR6003 HTC 128-byte read, modprobe >600 s);
 * writes get force-terminated with -ETIMEDOUT by qcom_dma_timeout_work,
 * which TEARS a write mid-flight and corrupts the eMMC filesystem (the
 * recurring rootfs/uboot corruption after a clean webOS reflash).
 *
 * Legacy webOS msm_sdcc completes EVERY DMA transfer — read and write —
 * from the msm_dmov complete_func, not from a SDCC DATAEND alone. We
 * replicate that here: when the ADM signals the descriptor done
 * (RESULT=success ⇒ all bytes moved), finish the request.
 *
 * Safe for writes because these transfers are CRCI flow-controlled
 * (device_fc=1): the ADM descriptor only completes once CRCI pacing has
 * drained every byte through the SDCC FIFO to the card, so DMA-done means
 * the data reached the card. The card's internal programming-busy is then
 * handled by the mmc core's normal CMD13 ready-poll before the next
 * request — mmci_stop_data() here does not truncate it.
 *
 * The qcom_adm IRQ handler invokes this from hardirq with achan->vc.lock
 * dropped, so taking host->lock is safe. The MCI_DATAEND path in
 * mmci_data_irq() still runs for transfers that DO raise it; whichever
 * path grabs host->lock and finds host->data first wins, the loser sees
 * host->data == NULL and bails, so there is no double-completion. The
 * watchdog remains armed as a last resort for a genuine DMA failure where
 * neither DATAEND nor this callback ever fires.
 */
static void mmci_qcom_dma_complete(void *param)
{
	struct mmci_host *host = param;
	struct mmci_dmae_priv *dmae = host->dma_priv;
	unsigned long flags;
	struct mmc_data *data;
	u32 status, status_err;
	bool write;

	spin_lock_irqsave(&host->lock, flags);

	data = host->data;
	if (!data) {
		/* Already completed (DATAEND raced in, or error path ran) */
		mmci_qcom_diag("MMCI-DMA-CB: mmc%u callback LATE (DATAEND already completed)\n",
			     host->mmc->index);
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	write = !(data->flags & MMC_DATA_READ);

	/*
	 * Consult the dmaengine BEFORE trusting MMCISTATUS.  An ADM-internal
	 * RSLT_ERR (channel pipeline aborted, no bytes moved into memory)
	 * does NOT raise an SDCC status_err -- the failure is upstream of
	 * the SDCC data path.  MMCISTATUS reads clean and the success branch
	 * below would charge blksz * blocks of unread memory to userspace
	 * (silent LVM / ext4 corruption observed on apq8060 / HP TouchPad:
	 * ADM ch2 result=0x80000008, MMCISTATUS=0x0, mmc block layer
	 * accepting zeros as the rootfs).
	 *
	 * Mark data->error here and fall through -- the rest of the
	 * function (status_err handling, mmc1 PROGDONE poll, bytes
	 * accounting, mmci_dma_finalize) is gated on !data->error and
	 * becomes a no-op.  Do NOT call mmci_dma_error() inline: it would
	 * dmaengine_terminate_all() from this dmaengine callback context
	 * (per qcom_adm.c hardirq comment), whose vc.desc_free path can
	 * reach dma_free_coherent() for non-pool descriptors and trip
	 * WARN_ON in kernel/dma/mapping.c.  The SDCC-side reset
	 * (reset_control_assert/deassert at mmci_data_irq()) fires when
	 * the SDCC raises its own data error -- which it does eventually
	 * for a stalled transfer -- or, if SDCC stays clean (ADM-only
	 * failure), the per-channel error_cookie path in qcom_adm.c
	 * surfaces DMA_ERROR on every subsequent submission and the MMC
	 * block layer's retry exhaustion terminates the request with
	 * -EIO instead of stranding it.
	 */
	if (dmae->cur && dmae->curr_cookie) {
		enum dma_status ds = dmaengine_tx_status(dmae->cur,
							 dmae->curr_cookie,
							 NULL);

		if (ds == DMA_ERROR && !data->error) {
			data->error = -EIO;
			/*
			 * Full SDCC snapshot at the moment dmaengine reports
			 * DMA_ERROR for this cookie (typically the ADM
			 * watchdog completed the wedged cookie with error).
			 * Paired with the ADM-side dump in adm_watchdog_timeout
			 * this gives us the canonical "who stopped first":
			 *   - SDCC TXACTIVE/RXACTIVE + non-empty FIFO →
			 *     SDCC was still pumping; ADM stopped getting CRCI.
			 *   - SDCC FIFO empty + DATAEND pending →
			 *     SDCC reached end-of-data; ADM didn't see RSLT.
			 *   - SDCC DATACRCFAIL/DATATIMEOUT/STARTBITERR set →
			 *     card side aborted the transfer.
			 *   - DATACNT mid-range with no error bits →
			 *     SDCC half-completed; data path stalled.
			 */
			dev_warn_ratelimited(mmc_dev(host->mmc),
				"DMA_ERROR mmc%u %s blksz=%u blocks=%u: STATUS=0x%08x DATACTRL=0x%08x DATACNT=%u DATALEN=%u FIFOCNT=%u CLOCK=0x%08x MASK0=0x%08x\n",
				host->mmc->index,
				write ? "WRITE" : "read",
				data->blksz, data->blocks,
				readl(host->base + MMCISTATUS),
				readl(host->base + MMCIDATACTRL),
				readl(host->base + MMCIDATACNT),
				readl(host->base + MMCIDATALENGTH),
				readl(host->base + MMCIFIFOCNT),
				readl(host->base + MMCICLOCK),
				readl(host->base + MMCIMASK0));
		}
	}

	/*
	 * The ADM moving all bytes does NOT mean the SDCC considered the
	 * transfer clean: it can still latch DATACRCFAIL / RXOVERRUN /
	 * DATATIMEOUT / STARTBITERR. Completing purely on the DMA-done edge
	 * would bypass that check and could report success with CRC-bad data
	 * (a real risk on this SoC's FIXED-address mailbox reads). Sample
	 * MMCISTATUS here and honor any pending data error so the request
	 * fails cleanly instead of returning corrupt data. (We hold
	 * host->lock, so this is serialized against mmci_irq's error path;
	 * whichever runs first completes, the other sees host->data == NULL.)
	 */
	status = readl(host->base + MMCISTATUS);
	status_err = status & (host->variant->start_err |
			       MCI_DATACRCFAIL | MCI_DATATIMEOUT |
			       MCI_TXUNDERRUN | MCI_RXOVERRUN);

	mmci_qcom_diag("MMCI-DMA-CB: mmc%u callback COMPLETES %s (no DATAEND) blksz=%u blocks=%u status=0x%08x err=0x%08x\n",
		     host->mmc->index, write ? "WRITE" : "read",
		     data->blksz, data->blocks, status, status_err);

	if (host->variant->qcom_datactrl_delay)
		cancel_delayed_work(&host->qcom_dma_timeout_work);

	if (status_err && !data->error) {
		/* Clear the latched error bits we are consuming. */
		writel(status_err, host->base + MMCICLEAR);

		if (status_err & MCI_DATACRCFAIL)
			data->error = -EILSEQ;
		else if (status_err & MCI_DATATIMEOUT)
			data->error = -ETIMEDOUT;
		else if (status_err & MCI_STARTBITERR)
			data->error = -ECOMM;
		else if (status_err & (MCI_TXUNDERRUN | MCI_RXOVERRUN))
			data->error = -EIO;

		/* Tear down the DMA cleanly on a hardware data error. */
		mmci_dma_error(host);
	}

	/*
	 * WRITES on mmc1: poll for MCI_QCOM_PROGDONE before tearing down
	 * the data path.  ADM-done means all bytes left the SDCC TX FIFO,
	 * but the card's internal programming may still be in flight.
	 * Without this wait mmci_stop_data() would truncate the write and
	 * the next CMD would race PROG_DONE.  PROG_DONE arrives within
	 * microseconds for a 128 B mailbox write under healthy fabric.
	 *
	 * Bound at 200 us so we do not busy-spin in hardirq for
	 * milliseconds while holding host->lock with IRQs off (Sashiko
	 * High #3 on submit/mmci-qcom-tenderloin, 2026-06-14).  If
	 * PROG_DONE never asserts within that window we ack and continue
	 * -- the next CMD will fail visibly with a CMDTIMEOUT and the
	 * existing recovery path handles it, which is preferable to
	 * starving every other IRQ on the system.
	 */
	if (write && host->mmc->index == 1 && !data->error) {
		u32 status;
		int ret;

		ret = readl_poll_timeout_atomic(host->base + MMCISTATUS,
						status,
						status & MCI_QCOM_PROGDONE,
						1, 200);
		if (ret)
			dev_warn_ratelimited(mmc_dev(host->mmc),
				"PROG_DONE not asserted within 200 us; CMD-path recovery will fire if next CMD fails\n");
		writel(MCI_QCOM_PROGDONE, host->base + MMCICLEAR);
	}

	/* ADM RESULT=success ⇒ full transfer moved (to memory, or to card) */
	mmci_dma_finalize(host, data);
	mmci_stop_data(host);

	if (!data->error)
		data->bytes_xfered = data->blksz * data->blocks;

	/* Graded DMA->PIO recovery: arm a one-shot PIO retry if eMMC keeps flushing. */
	mmci_qcom_grade_recovery(host, data);

	if (!data->stop) {
		mmci_request_end(host, data->mrq);
	} else if (host->mrq->sbc && !data->error) {
		mmci_request_end(host, data->mrq);
	} else {
		/*
		 * Clear atomic_submit before dispatching the stop CMD.
		 * mmci_start_command() stashes ARG/CMD into atomic_submit
		 * instead of writing MMCICOMMAND when .active is set; if
		 * we leave .active set from the data-phase submit, the
		 * CMD12 stop is silently swallowed.  (Sashiko Critical #3
		 * on submit/mmci-qcom-tenderloin, 2026-06-14.)
		 */
		host->atomic_submit.armed = false;
		host->atomic_submit.active = false;
		mmci_start_command(host, data->stop, 0);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

/* prepares DMA channel and DMA descriptor, returns non-zero on failure */
static int _mmci_dmae_prep_data(struct mmci_host *host, struct mmc_data *data,
				struct dma_chan **dma_chan,
				struct dma_async_tx_descriptor **dma_desc)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	struct variant_data *variant = host->variant;
	struct qcom_adm_peripheral_config periph_conf = {};
	/*
	 * For the qcom variant with CRCI flow-control via ADM, the box
	 * descriptor's "row" must match the full SDC FIFO size (64 bytes
	 * on MSM8660). The CRCI is asserted only when the FIFO is half-
	 * empty — and the ADM doesn't start the next row until CRCI
	 * fires again, so each row transfers an entire FIFO-full of data
	 * between CRCI assertions. If the row is only half the FIFO
	 * (the default for non-qcom mmci variants), boundary conditions
	 * during boot-time CPU/bus transitions intermittently corrupt
	 * data → DATACRCFAIL ~1/3 of boots.
	 *
	 * Legacy webOS msm_sdcc.c uses MCI_FIFOSIZE (= full fifosize)
	 * as box row size and is reliable. Match that for qcom variant.
	 *
	 * Standard mmci variants keep the original behaviour (half-FIFO
	 * burst, matching the half-empty trigger semantics of generic
	 * ARM PrimeCell).
	 */
	unsigned int burst_words = variant->qcom_fifo ? (variant->fifosize >> 2)
						      : (variant->fifohalfsize >> 2);
	struct dma_slave_config conf = {
		.src_addr = host->phybase + MMCIFIFO,
		.dst_addr = host->phybase + MMCIFIFO,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = burst_words,
		.dst_maxburst = burst_words,
		.device_fc = variant->dma_flow_controller,
	};
	struct dma_chan *chan;
	struct dma_device *device;
	struct dma_async_tx_descriptor *desc;
	int nr_sg;
	unsigned long flags = DMA_CTRL_ACK;

	/*
	 * Pass CRCI + (when applicable) atomic-submit exec_func to the
	 * Qualcomm ADM DMA controller. Both fields live in struct
	 * qcom_adm_peripheral_config, so the peripheral_config block
	 * is only valid when the channel is actually an ADM channel.
	 *
	 * dmae->crci is set from the "qcom,sdcc-crci" DT property which
	 * is only present on ADM-DMA hosts (BAM hosts use a different
	 * peripheral_config layout). Gating on dmae->crci therefore
	 * implicitly restricts this block to ADM channels.
	 */
	if (dmae->crci) {
		periph_conf.crci = dmae->crci;

		/*
		 * Atomic-submission opt-in: when the variant supports the
		 * peripheral exec_func hook and this request qualifies
		 * (write + datactrl_first + SDIO_IRQ host -- the same
		 * predicate as the previous "deferred via cmd_irq"
		 * fallback that this replaces), wire up the callback. The
		 * exec_func is invoked by qcom_adm from inside its
		 * per-controller submit_lock right before the channel's
		 * CMD_PTR write, performing SDCC DATACTRL + ARG + CMD
		 * writes atomically with the ADM start.
		 *
		 * host->atomic_submit.active is set later by mmci_dma_start
		 * only after ops->dma_start() has succeeded, so a PIO
		 * fallback keeps the conventional write path even if
		 * exec_func was configured here.
		 */
		if (mmci_should_atomic_submit(host, data)) {
			periph_conf.exec_func = mmci_qcom_atomic_exec_func;
			periph_conf.exec_user = host;
			host->atomic_submit.armed = true;
		}

		/*
		 * Always wire dump_state so the ADM watchdog can snapshot
		 * the SDCC side when it sees a wedge on this host's channel.
		 */
		periph_conf.dump_state = mmci_qcom_dump_state;
		periph_conf.dump_user  = host;

		conf.peripheral_config = &periph_conf;
		conf.peripheral_size = sizeof(periph_conf);
	}

	if (data->flags & MMC_DATA_READ) {
		conf.direction = DMA_DEV_TO_MEM;
		chan = dmae->rx_channel;
	} else {
		conf.direction = DMA_MEM_TO_DEV;
		chan = dmae->tx_channel;
	}

	/* If there's no DMA channel, fall back to PIO */
	if (!chan)
		return -EINVAL;

	/*
	 * Qualcomm SDCC + ADM: replicate legacy webOS msm_sdcc validate_dma()
	 * EXACTLY. The vendor driver only hands a transfer to the ADM when it
	 * is >= one FIFO (64 B) AND an exact multiple of the FIFO size; every
	 * other transfer (sub-FIFO, or non-FIFO-aligned) goes through PIO.
	 *
	 * This matters because the ADM box/CRCI handshake is built around
	 * whole-FIFO rows. Feeding it a sub-FIFO transfer (e.g. the 52/44-byte
	 * AR6003 BMI control transfers) builds a single non-box descriptor
	 * whose CRCI pacing does not drain the FIFO on a row boundary, leaving
	 * the SDCC data path in a state that makes the NEXT DMA transfer (e.g.
	 * the 128-byte HTC mailbox read) latch RXOVERRUN/DATACRCFAIL —
	 * "error during DMA transfer". The earlier dma_threshold=32 override
	 * (to "route BMI through DMA") diverged from the vendor driver, which
	 * in fact PIOs those small transfers. DMA still carries the bulk path
	 * (large, FIFO-aligned data frames) exactly as webOS does.
	 */
	if (host->variant->qcom_dml && host->mmc->index == 1) {
		/* WiFi (sdcc4) ONLY — see note above. */
		unsigned int len = data->blksz * data->blocks;

		/*
		 * mmc1 (sdcc4 / WiFi) SDIO WR DMA threshold.
		 *
		 * Default 256 is the historical AR6003 mailbox-WR workaround:
		 * 128 B mailbox WRITEs at chip 0xF80 used to CMDTIMEOUT via
		 * DMA (arg=0x941f0080, DATACTRL=0x809) and we forced them
		 * through PIO.  Since the mmci atomic exec_func DATATIMER +
		 * DATALENGTH fix (41f975cdc56c + 734a0002e6ed) and the
		 * defer-DATACTRL-for-writes path (daafc06017e5), the host
		 * arms DPSM in lockstep with CMD53 and the 128 B WR @ 0xF80
		 * race should be gone -- letting us match legacy webOS
		 * msm_sdcc validate_dma() which only gates on
		 *   (len >= fifosize) && (len % fifosize == 0).
		 *
		 * Runtime knob mmci_mmc1_wr_dma_min (default 256) lets us
		 * A/B test: set to 64 to use the legacy-equivalent threshold.
		 */
		if (len < mmci_mmc1_wr_dma_min || (len % variant->fifosize))
			return -EINVAL;
	} else if (data->blksz * data->blocks <=
		   (variant->dma_threshold ?: variant->fifosize)) {
		/*
		 * eMMC and non-qcom variants: original behaviour — PIO at/below
		 * the DMA threshold (defaults to fifosize, overridable per
		 * variant). eMMC is left exactly as it was before the WiFi work.
		 */
		return -EINVAL;
	}

	/*
	 * This is necessary to get SDIO working on the Ux500. We do not yet
	 * know if this is a bug in:
	 * - The Ux500 DMA controller (DMA40)
	 * - The MMCI DMA interface on the Ux500
	 * some power of two blocks (such as 64 bytes) are sent regularly
	 * during SDIO traffic and those work fine so for these we enable DMA
	 * transfers.
	 */
	if (host->variant->dma_power_of_2 && !is_power_of_2(data->blksz))
		return -EINVAL;

	device = chan->device;
	nr_sg = dma_map_sg(device->dev, data->sg, data->sg_len,
			   mmc_get_dma_dir(data));
	if (nr_sg == 0)
		return -EINVAL;

	if (host->variant->qcom_dml)
		flags |= DMA_PREP_INTERRUPT;

	dmaengine_slave_config(chan, &conf);
	desc = dmaengine_prep_slave_sg(chan, data->sg, nr_sg,
					    conf.direction, flags);
	if (!desc)
		goto unmap_exit;

	/*
	 * WiFi (sdcc4 = mmc1) READS ONLY: the AR6003 SDIO read path misses
	 * MCI_DATAEND from the SDCC DPSM, so reads cannot depend on
	 * mmci_data_irq() to complete. Wire a dmaengine completion callback
	 * (invoked by qcom_adm from its IRQ handler) to finish the request
	 * when the ADM reports the descriptor done.
	 *
	 * Deliberately NOT applied to:
	 *  - eMMC (mmc0): gets DATAEND reliably; completing on the DMA-done
	 *    edge raced the SDCC and destabilised the eMMC data path.
	 *  - WRITES (any controller): for a write, ADM descriptor-done only
	 *    means the data was pushed to the FIFO/card — the card still owes
	 *    its CRC status token (PROG_DONE), which is exactly what
	 *    MCI_DATAEND waits for. Completing on the DMA-done edge calls
	 *    mmci_stop_data() before PROG_DONE, truncating the write and
	 *    wedging the SDCC data path, so the NEXT command (e.g. the
	 *    WMI-CONTROL connect's follow-up reg-table read) fails. WiFi
	 *    writes DO get DATAEND, so they complete correctly via
	 *    mmci_data_irq() with no callback needed.
	 */
	/*
	 * Wire the ADM-done completion callback for ALL mmc1 DMA transfers,
	 * not just reads.  The original gate restricted this to reads because
	 * "WiFi writes DO get DATAEND" — but that is only true for BMI-phase
	 * writes and post-HTC-ready steady-state writes.  The first post-BMI
	 * WRITE (HTC SERVICE CONNECT) on AR6003 does NOT raise DATAEND: the
	 * SDCC stays at TXACTIVE+DATABLOCKEND+TXFIFOEMPTY indefinitely
	 * (status=0x00445400) and the 500 ms qcom_dma_data_timeout_work is
	 * the only thing that clears it — see memory
	 * project_wifi_post_bmi_dma_write_hang.  Wiring the callback for
	 * writes routes completion through the ADM-done edge, matching
	 * legacy webOS msm_sdcc.c's two-edge synchronisation
	 * (got_dataend || dma_busy=0).  The callback handles PROG_DONE for
	 * writes; if DATAEND does fire first on a normal write, the IRQ-side
	 * path completes and the late callback sees host->data == NULL and
	 * bails — no double-completion.
	 */
	if (host->variant->qcom_dml && host->mmc->index == 1) {
		desc->callback = mmci_qcom_dma_complete;
		desc->callback_param = host;
	}

	*dma_chan = chan;
	*dma_desc = desc;

	return 0;

 unmap_exit:
	dma_unmap_sg(device->dev, data->sg, data->sg_len,
		     mmc_get_dma_dir(data));
	return -ENOMEM;
}

int mmci_dmae_prep_data(struct mmci_host *host,
			struct mmc_data *data,
			bool next)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	struct mmci_dmae_next *nd = &dmae->next_data;

	if (!host->use_dma)
		return -EINVAL;

	if (next)
		return _mmci_dmae_prep_data(host, data, &nd->chan, &nd->desc);
	/* Check if next job is already prepared. */
	if (dmae->cur && dmae->desc_current) {
		/*
		 * mmci_pre_request → _mmci_dmae_prep_data already set
		 * host->atomic_submit.armed = true, but __mmci_start_request
		 * cleared it on its way in.  Re-arm here when this data
		 * qualifies, matching what a non-pre-prepared path would do.
		 */
		if (mmci_should_atomic_submit(host, data))
			host->atomic_submit.armed = true;
		return 0;
	}

	/* No job were prepared thus do it now. */
	return _mmci_dmae_prep_data(host, data, &dmae->cur,
				    &dmae->desc_current);
}

/*
 * Submit DMA descriptor without issuing pending.
 * Used by Qualcomm ADM DMA where DATACTRL must be written before DMA starts.
 */
int mmci_dmae_submit(struct mmci_host *host, unsigned int *datactrl)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	dma_cookie_t cookie;
	int ret;

	if (!dmae->desc_current || !dmae->cur) {
		dev_dbg(mmc_dev(host->mmc),
			"DMA submit without descriptor, falling back to PIO\n");
		return -EINVAL;
	}

	host->dma_in_progress = true;
	cookie = dmaengine_submit(dmae->desc_current);
	ret = dma_submit_error(cookie);
	if (ret < 0) {
		host->dma_in_progress = false;
		return ret;
	}
	/*
	 * Stash the cookie -- desc_current may be recycled by the qcom_adm
	 * pool path before mmci_qcom_dma_complete() reads its cookie back.
	 * Querying dmaengine_tx_status() via the stashed value is the
	 * race-safe way to see DMA_ERROR (cookies are monotonic per
	 * channel and unaffected by descriptor struct reuse).
	 */
	dmae->curr_cookie = cookie;

	/* Debug: start the FIFO/DATACNT cadence sampler for large transfers. */
	mmci_adm_sample_arm(host, host->data);

	*datactrl |= MCI_DPSM_DMAENABLE;

	/*
	 * One-shot diagnostic: log the first successful DMA submit so we
	 * can confirm in dmesg that DMA actually engages for eMMC transfers
	 * (vs. silently falling back to PIO). Subsequent submits are silent.
	 */
	if (!host->dma_engaged_once) {
		host->dma_engaged_once = true;
		dev_info(mmc_dev(host->mmc),
			 "DMA submit OK (first transfer) — driver is using DMA path\n");
	}

	return 0;
}

void mmci_dmae_issue_pending(struct mmci_host *host)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;

	if (dmae && dmae->cur)
		dma_async_issue_pending(dmae->cur);
}

int mmci_dmae_start(struct mmci_host *host, unsigned int *datactrl)
{
	int ret = mmci_dmae_submit(host, datactrl);

	if (ret)
		return ret;

	mmci_dmae_issue_pending(host);

	return 0;
}

void mmci_dmae_get_next_data(struct mmci_host *host, struct mmc_data *data)
{
	struct mmci_dmae_priv *dmae = host->dma_priv;
	struct mmci_dmae_next *next = &dmae->next_data;

	if (!host->use_dma)
		return;

	/*
	 * Only use the pre-prepared "next" descriptor if this request
	 * was actually pre-prepared (has a host_cookie). If host_cookie
	 * is 0, this request wasn't pre-prepared via mmci_pre_request(),
	 * so any descriptor in next->desc belongs to a different request.
	 *
	 * This can happen during MMC hardware reset recovery, where new
	 * internal commands (CMD8, CMD13, etc.) come through mmci_request()
	 * without going through mmci_pre_request() first.
	 *
	 * Key fix: We only clear next_data when we actually consume it.
	 * If we don't use it (host_cookie=0), leave it intact for when
	 * the actual pre-prepared request is issued later. Setting
	 * desc_current to NULL will trigger inline DMA preparation via
	 * mmci_dmae_prep_data().
	 */
	if (data->host_cookie) {
		/* Request was pre-prepared, use and consume the descriptor */
		dmae->desc_current = next->desc;
		dmae->cur = next->chan;
		next->desc = NULL;
		next->chan = NULL;
	} else {
		/* Request not pre-prepared, prepare DMA inline */
		dmae->desc_current = NULL;
		dmae->cur = NULL;
		/* Leave next_data intact for its actual owner */
	}
}

void mmci_dmae_unprep_data(struct mmci_host *host,
			   struct mmc_data *data, int err)

{
	struct mmci_dmae_priv *dmae = host->dma_priv;

	if (!host->use_dma)
		return;

	mmci_dma_unmap(host, data);

	if (err) {
		struct mmci_dmae_next *next = &dmae->next_data;
		struct dma_chan *chan;
		if (data->flags & MMC_DATA_READ)
			chan = dmae->rx_channel;
		else
			chan = dmae->tx_channel;
		dmaengine_terminate_all(chan);

		if (dmae->desc_current == next->desc)
			dmae->desc_current = NULL;

		if (dmae->cur == next->chan) {
			host->dma_in_progress = false;
			dmae->cur = NULL;
		}

		next->desc = NULL;
		next->chan = NULL;
	}
}

static struct mmci_host_ops mmci_variant_ops = {
	.prep_data = mmci_dmae_prep_data,
	.unprep_data = mmci_dmae_unprep_data,
	.get_datactrl_cfg = mmci_get_dctrl_cfg,
	.get_next_data = mmci_dmae_get_next_data,
	.dma_setup = mmci_dmae_setup,
	.dma_release = mmci_dmae_release,
	.dma_start = mmci_dmae_start,
	.dma_finalize = mmci_dmae_finalize,
	.dma_error = mmci_dmae_error,
};
#else
static struct mmci_host_ops mmci_variant_ops = {
	.get_datactrl_cfg = mmci_get_dctrl_cfg,
};
#endif

static void mmci_variant_init(struct mmci_host *host)
{
	host->ops = &mmci_variant_ops;
}

static void ux500_variant_init(struct mmci_host *host)
{
	host->ops = &mmci_variant_ops;
	host->ops->busy_complete = ux500_busy_complete;
}

static void ux500v2_variant_init(struct mmci_host *host)
{
	host->ops = &mmci_variant_ops;
	host->ops->busy_complete = ux500_busy_complete;
	host->ops->get_datactrl_cfg = ux500v2_get_dctrl_cfg;
}

/*
 * Dynamic ICC bandwidth voting (Qualcomm variant).  Called from
 * mmci_pre_request when a data transfer is about to be dispatched.
 * Bumps the SDC ICC vote to the active level and records last activity
 * jiffies so the idle worker keeps the vote held while traffic is
 * sustained.  Always runs in process context (blk-mq dispatch), so
 * icc_set_bw is safe.
 */
static void mmci_qcom_icc_bump_active(struct mmci_host *host)
{
	bool need_set;
	unsigned long flags;

	if (!host->icc_path || !host->icc_active_bw)
		return;

	spin_lock_irqsave(&host->icc_vote_lock, flags);
	host->icc_last_active = jiffies;
	need_set = !host->icc_voted_active;
	if (need_set)
		host->icc_voted_active = true;
	spin_unlock_irqrestore(&host->icc_vote_lock, flags);

	if (need_set)
		icc_set_bw(host->icc_path,
			   host->icc_active_bw, host->icc_active_bw);

	/* Re-arm the idle worker so it fires icc_idle_ms after this point. */
	mod_delayed_work(system_wq, &host->icc_idle_work,
			 msecs_to_jiffies(host->icc_idle_ms));
}

/*
 * Idle worker: if no traffic has touched icc_last_active within the
 * hysteresis window, drop the SDC ICC vote back to the keep-alive
 * level.  Re-armed by mmci_qcom_icc_bump_active on every bump so a
 * sustained burst keeps the vote held until quiet.
 */
/* --- ADM throughput instrumentation (debug) ---------------------------- */
#define ADM_SAMPLE_PERIOD_MS	40
#define ADM_SAMPLE_MAX		32
#define ADM_SAMPLE_MIN_BYTES	(64 * 1024)

/*
 * Periodically log the SDCC FIFO occupancy and residual byte count while an
 * ADM data transfer is in flight. DATACNT counts down from DATALENGTH, so
 * the delta between ticks is the bytes actually moved -> instantaneous rate.
 * FIFOCNT localises the bottleneck: near full (16 words) => the SDCC->card
 * drain is the limiter (flow-control / card busy); near empty => the
 * ADM->FIFO fill (memory/fabric side) is. Self-terminates when the transfer
 * ends or after ADM_SAMPLE_MAX ticks.
 */
static void mmci_adm_sample_work(struct work_struct *work)
{
	struct mmci_host *host = container_of(work, struct mmci_host,
					      adm_sample_work.work);
	void __iomem *base = host->base;
	u32 fifocnt, status, dcnt, dlen, moved;
	unsigned int elapsed_ms, kbs;
	unsigned long flags;
	bool in_flight;

	spin_lock_irqsave(&host->lock, flags);
	in_flight = host->data && host->dma_in_progress;
	spin_unlock_irqrestore(&host->lock, flags);
	if (!in_flight)
		return;

	fifocnt = readl_relaxed(base + MMCIFIFOCNT);
	status  = readl_relaxed(base + MMCISTATUS);
	dcnt    = readl_relaxed(base + MMCIDATACNT);
	dlen    = readl_relaxed(base + MMCIDATALENGTH);
	elapsed_ms = jiffies_to_msecs(jiffies - host->adm_sample_t0);
	moved = host->adm_sample_last_dcnt > dcnt ?
		host->adm_sample_last_dcnt - dcnt : 0;
	kbs = (moved * 1000U / 1024U) / ADM_SAMPLE_PERIOD_MS;

	dev_info(mmc_dev(host->mmc),
		 "ADM-SAMPLE mmc%u %s #%u t=%ums FIFOCNT=%u STATUS=0x%08x DCNT=0x%08x/0x%08x moved=%uB (~%u KB/s)\n",
		 host->mmc->index, host->adm_sample_write ? "WR" : "RD",
		 host->adm_sample_n, elapsed_ms, fifocnt, status, dcnt, dlen,
		 moved, kbs);

	host->adm_sample_last_dcnt = dcnt;
	if (++host->adm_sample_n < ADM_SAMPLE_MAX)
		mod_delayed_work(system_wq, &host->adm_sample_work,
				 msecs_to_jiffies(ADM_SAMPLE_PERIOD_MS));
}

static void mmci_adm_sample_arm(struct mmci_host *host, struct mmc_data *data)
{
	if (!adm_sample || !host->variant->qcom_dml_atomic_submit || !data)
		return;
	if ((size_t)data->blksz * data->blocks < ADM_SAMPLE_MIN_BYTES)
		return;
	host->adm_sample_t0 = jiffies;
	host->adm_sample_last_dcnt = readl_relaxed(host->base + MMCIDATACNT);
	host->adm_sample_n = 0;
	host->adm_sample_write = !(data->flags & MMC_DATA_READ);
	mod_delayed_work(system_wq, &host->adm_sample_work,
			 msecs_to_jiffies(ADM_SAMPLE_PERIOD_MS));
}

static void mmci_qcom_icc_idle_work(struct work_struct *work)
{
	struct mmci_host *host = container_of(work, struct mmci_host,
					      icc_idle_work.work);
	unsigned long flags, deadline;
	bool need_lower = false;

	if (!host->icc_path)
		return;

	spin_lock_irqsave(&host->icc_vote_lock, flags);
	deadline = host->icc_last_active +
		   msecs_to_jiffies(host->icc_idle_ms);
	if (time_before(jiffies, deadline)) {
		/*
		 * Activity bumped us between schedule and fire.  Re-arm
		 * for the residual interval.
		 */
		spin_unlock_irqrestore(&host->icc_vote_lock, flags);
		mod_delayed_work(system_wq, &host->icc_idle_work,
				 deadline - jiffies);
		return;
	}
	/*
	 * Never drop the DFAB vote while a request/DMA is still in flight.
	 * icc_last_active is only refreshed in pre_request, so a single
	 * transfer longer than icc_idle_ms (e.g. a multi-MB eMMC write) would
	 * otherwise have the fabric lowered out from under it: the ADM can no
	 * longer drain the SDCC FIFO to EBI, starves mid-transfer and the
	 * channel wedges (500 ms watchdog). Legacy webOS holds dfab_sdc_clk at
	 * 64 MHz for the whole active period (persistent clk_enable); mirror
	 * that by keeping the vote until the controller is genuinely idle.
	 */
	if (host->mrq || host->data || host->dma_in_progress) {
		spin_unlock_irqrestore(&host->icc_vote_lock, flags);
		mod_delayed_work(system_wq, &host->icc_idle_work,
				 msecs_to_jiffies(host->icc_idle_ms));
		return;
	}
	if (host->icc_voted_active) {
		host->icc_voted_active = false;
		need_lower = true;
	}
	spin_unlock_irqrestore(&host->icc_vote_lock, flags);

	if (need_lower)
		icc_set_bw(host->icc_path,
			   host->icc_idle_bw, host->icc_idle_bw);
}

static void mmci_pre_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	WARN_ON(data->host_cookie);

	if (mmci_validate_data(host, data))
		return;

	mmci_prep_data(host, data, true);

	/*
	 * Bump ICC vote AFTER prep validation -- if the prep failed we have
	 * nothing to bump for, and we want to avoid bumping for cmd-only
	 * requests (already filtered above by !data).
	 */
	if (host->variant->qcom_dml)
		mmci_qcom_icc_bump_active(host);
}

static void mmci_post_request(struct mmc_host *mmc, struct mmc_request *mrq,
			      int err)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data || !data->host_cookie)
		return;

	mmci_unprep_data(host, data, err);
}

static void mmci_start_data(struct mmci_host *host, struct mmc_data *data)
{
	struct variant_data *variant = host->variant;
	unsigned int datactrl, timeout, irqmask;
	unsigned long long clks;
	void __iomem *base;

	dev_dbg(mmc_dev(host->mmc), "blksz %04x blks %04x flags %08x\n",
		data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz * data->blocks;
	data->bytes_xfered = 0;

	/*
	 * WiFi (mmc1): clear any stale PROG_DONE latch before each transfer so
	 * that, when a WRITE completes, a set PROG_DONE reflects THIS write's
	 * card programming. The DMA-write completion path polls for it (see
	 * mmci_data_irq) to avoid issuing the next CMD53 while the AR6003 is
	 * still programming the just-written mailbox.
	 *
	 * Also snapshot + clear any stale data-error latches (STARTBITERR,
	 * DATATIMEOUT, DATACRCFAIL, TX/RX FIFO overflow). MMCISTATUS error bits
	 * are sticky on the qcom SDCC variant; a residue from a prior glitched
	 * transfer would otherwise feed straight into this transfer's IRQ path
	 * and look like a fresh error.  Observed on tenderloin/AR6003 as
	 * post-fw-boot CMD53 wedges: the chip's pin-drive misconfiguration left
	 * STARTBITERR (bit 9) latched, the very next set_addrwin_reg CMD53 then
	 * timed out -110.  We log the residue ratelimited so a real recurring
	 * problem is still visible.
	 */
	if (host->mmc->index == 1) {
		u32 stale;
		u32 stale_mask = MCI_STARTBITERR | MCI_DATATIMEOUT |
				 MCI_DATACRCFAIL | MCI_TXUNDERRUN |
				 MCI_RXOVERRUN;

		stale = readl(host->base + MMCISTATUS) & stale_mask;
		if (stale) {
			static DEFINE_RATELIMIT_STATE(rs, HZ, 5);

			if (__ratelimit(&rs))
				dev_warn(mmc_dev(host->mmc),
					 "clearing stale data-error residue 0x%08x before CMD53 (DATACNT=%u DATACTRL=0x%08x)\n",
					 stale,
					 readl(host->base + MMCIDATACNT),
					 readl(host->base + MMCIDATACTRL));
			writel(stale, host->base + MMCICLEAR);
		}
	}

	/*
	 * Clear any stale PROG_DONE before arming the data path.
	 *
	 * Legacy webOS msm_sdcc clears MCI_PROGDONE every cycle (it is in
	 * MCI_CLR_MASK) on *every* SDCC.  Our driver previously cleared it for
	 * mmc1 (WiFi) only, so the eMMC (mmc0) carried PROG_DONE latched-set
	 * from the previous write.  On the qcom SDCC the data-path FSM
	 * consults PROG_DONE, and a stale-set bit gates it from asserting the
	 * write CRCI partway through a large multi-block write: the ADM stalls
	 * on DST_CRCI with the card not busy (MCI_ST_CARDBUSY clear) and the
	 * TX FIFO holding data (MCI_TXDATAAVLBL set) while DATACNT freezes,
	 * until the 500 ms ADM ch2 watchdog fires.  Live ADM-SAMPLE capture on
	 * tenderloin showed exactly this: STATUS=0x00901400 (PROG_DONE set,
	 * CARDBUSY clear) during both the ~250 ms start stall and the tail
	 * stall.  Clear it for every qcom ADM host so each transfer arms with
	 * a clean FSM, matching legacy.
	 *
	 * Gate on qcom_dml_atomic_submit (set only on variant_qcom_msm8660,
	 * the ADM SDCC variant used by both sdcc1/eMMC and sdcc4/WiFi) rather
	 * than qcom_dml: bit 23 is PROG_DONE only on the ADM SDCC; on BAM
	 * qcom SoCs (variant_qcom) it is CE-ATA and must not be cleared here.
	 */
	if (host->variant->qcom_dml_atomic_submit)
		writel(MCI_QCOM_PROGDONE, host->base + MMCICLEAR);

	clks = (unsigned long long)data->timeout_ns * host->cclk;
	do_div(clks, NSEC_PER_SEC);

	/*
	 * Qualcomm SDCC requires doubling the calculated data timeout.
	 * The legacy msm_sdcc driver uses clks*2 for the data timeout.
	 * Without this, SDIO operations can timeout prematurely.
	 */
	if (variant->qcom_data_timeout_2x)
		clks *= 2;

	timeout = data->timeout_clks + (unsigned int)clks;

	base = host->base;
	writel(timeout, base + MMCIDATATIMER);
	writel(host->size, base + MMCIDATALENGTH);

	/*
	 * Also stash for the atomic-submit path so mmci_qcom_atomic_exec_func
	 * can re-write DATATIMER + DATALENGTH inside the IRQs-off window
	 * just before DATACTRL arms DPSM, matching legacy webOS
	 * msmsdcc_dma_exec_func ordering exactly.  Eager writes here are
	 * NOT suppressed because the same code path serves PIO transfers
	 * on sdcc4 (4 B BMI register reads, etc.) which never reach
	 * exec_func; suppressing them would leave PIO with zero/stale
	 * DATALENGTH and hang the request indefinitely.  Re-writing the
	 * same values inside exec_func is a no-op cost MMIO write on the
	 * DMA path and keeps PIO working.
	 */
	if (mmci_should_atomic_submit(host, data)) {
		host->atomic_submit.datatimer = timeout;
		host->atomic_submit.datalen = host->size;
	}

	datactrl = host->ops->get_datactrl_cfg(host);
	datactrl |= host->data->flags & MMC_DATA_READ ? MCI_DPSM_DIRECTION : 0;

	if (host->mmc->card && mmc_card_sdio(host->mmc->card)) {
		u32 clk;

		datactrl |= variant->datactrl_mask_sdio;

		/*
		 * The ST Micro variant for SDIO small write transfers
		 * needs to have clock H/W flow control disabled,
		 * otherwise the transfer will not start. The threshold
		 * depends on the rate of MCLK.
		 */
		if (variant->st_sdio && data->flags & MMC_DATA_WRITE &&
		    (host->size < 8 ||
		     (host->size <= 8 && host->mclk > 50000000)))
			clk = host->clk_reg & ~variant->clkreg_enable;
		else
			clk = host->clk_reg | variant->clkreg_enable;

		mmci_write_clkreg(host, clk);
	}

	if (host->mmc->ios.timing == MMC_TIMING_UHS_DDR50 ||
	    host->mmc->ios.timing == MMC_TIMING_MMC_DDR52)
		datactrl |= variant->datactrl_mask_ddrmode;

	/*
	 * Attempt to use DMA operation mode, if this
	 * should fail, fall back to PIO mode
	 */
	if (!mmci_dma_start(host, datactrl))
		return;

	/* IRQ mode, map the SG list for CPU reading/writing */
	mmci_init_sg(host, data);

	if (data->flags & MMC_DATA_READ) {
		irqmask = MCI_RXFIFOHALFFULLMASK;

		/*
		 * If we have less than the fifo 'half-full' threshold to
		 * transfer, trigger a PIO interrupt as soon as any data
		 * is available.
		 */
		if (host->size < variant->fifohalfsize)
			irqmask |= MCI_RXDATAAVLBLMASK;
	} else {
		/*
		 * We don't actually need to include "FIFO empty" here
		 * since its implicit in "FIFO half empty".
		 */
		irqmask = MCI_TXFIFOHALFEMPTYMASK;
	}

	/*
	 * Qualcomm SDCC requires a delay after writing DATATIMER/DATALENGTH
	 * before writing DATACTRL. Without this delay, SDIO data transfers
	 * can fail with CRC or start bit errors.
	 */
	if (host->variant->qcom_datactrl_delay) {
		/* Ensure data parameters are applied before DATACTRL */
		wmb();
		udelay(5);
	}

	/* Debug: print DATACTRL value for Qualcomm SDIO PIO fallback debugging */
	if (host->variant->qcom_datactrl_delay)
		dev_dbg(mmc_dev(host->mmc),
			"PIO datactrl=0x%x blksz=%u size=%u\n",
			datactrl, data->blksz, host->size);

	mmci_write_datactrlreg(host, datactrl);

	/* Another delay after DATACTRL for Qualcomm */
	if (host->variant->qcom_datactrl_delay) {
		wmb();
		udelay(5);
	}

	writel(readl(base + MMCIMASK0) & ~MCI_DATAENDMASK, base + MMCIMASK0);
	mmci_set_mask1(host, irqmask);
}

static void
mmci_start_command(struct mmci_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;
	bool busy_resp = cmd->flags & MMC_RSP_BUSY;
	unsigned long long clks;

	dev_dbg(mmc_dev(host->mmc), "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & host->variant->cmdreg_cpsm_enable) {
		writel(0, base + MMCICOMMAND);
		mmci_reg_delay(host);
	}

	if (host->variant->cmdreg_stop &&
	    cmd->opcode == MMC_STOP_TRANSMISSION)
		c |= host->variant->cmdreg_stop;

	c |= cmd->opcode | host->variant->cmdreg_cpsm_enable;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= host->variant->cmdreg_lrsp_crc;
		else if (cmd->flags & MMC_RSP_CRC)
			c |= host->variant->cmdreg_srsp_crc;
		else
			c |= host->variant->cmdreg_srsp;
	}

	host->busy_status = 0;
	host->busy_state = MMCI_BUSY_DONE;

	/* Assign a default timeout if the core does not provide one */
	if (busy_resp && !cmd->busy_timeout)
		cmd->busy_timeout = 10 * MSEC_PER_SEC;

	if (busy_resp && host->variant->busy_timeout) {
		if (cmd->busy_timeout > host->mmc->max_busy_timeout)
			clks = (unsigned long long)host->mmc->max_busy_timeout * host->cclk;
		else
			clks = (unsigned long long)cmd->busy_timeout * host->cclk;

		do_div(clks, MSEC_PER_SEC);
		writel_relaxed(clks, host->base + MMCIDATATIMER);
	}

	if (host->ops->pre_sig_volt_switch && cmd->opcode == SD_SWITCH_VOLTAGE)
		host->ops->pre_sig_volt_switch(host);

	if (/*interrupt*/0)
		c |= MCI_CPSM_INTERRUPT;

	if (mmc_cmd_type(cmd) == MMC_CMD_ADTC)
		c |= host->variant->data_cmd_enable;

	host->cmd = cmd;

	if (host->atomic_submit.active) {
		/*
		 * Atomic-submission path: stash ARG/CMD for the qcom_adm
		 * exec_func instead of writing them directly. exec_func
		 * will perform the DATACTRL + ARG + CMD writes inside the
		 * ADM submit_lock, atomically with the channel's CMD_PTR
		 * write. mmci_request will issue the DMA pending right
		 * after this returns.
		 */
		host->atomic_submit.cmd_arg = cmd->arg;
		host->atomic_submit.cmd_reg = c;
		return;
	}

	writel(cmd->arg, base + MMCIARGUMENT);

	/*
	 * Qualcomm SDCC requires a clock-dependent delay between ARGUMENT and
	 * COMMAND register writes.  Legacy msm_sdcc uses msmsdcc_delay():
	 *   1 + 3000000 / clk_rate  (≈1 µs @ 48 MHz, ≈8.5 µs @ 400 kHz)
	 * The short udelay(1) here was enough for most eMMC firmware, but
	 * Samsung SEM32G fw-9.0 (PRV=0x90) returns an OTP-only EXT_CSD
	 * (capacity = 0) with the 1 µs gap at the 400 kHz init clock.
	 * Using the full formula restores the timing the hardware expects.
	 */
	if (host->variant->qcom_datactrl_delay)
		udelay(mmci_qcom_settle_us(host));

	writel(c, base + MMCICOMMAND);
}

static void mmci_stop_command(struct mmci_host *host)
{
	host->stop_abort.error = 0;
	mmci_start_command(host, &host->stop_abort, 0);
}

/*
 * Diagnostic helper to dump SDCC + ADM state at the moment a data
 * IRQ error fires. Used for investigating the heavy-concurrent-DMA
 * failure mode (legacy passes the same workload; mainline doesn't).
 *
 * Ratelimited so the recovery cascade (cmd12/cmd13 retries each
 * generating their own CMDTIMEOUTs) doesn't drown the log.
 */
static void mmci_diag_dump_state(struct mmci_host *host, const char *reason)
{
	static DEFINE_RATELIMIT_STATE(rs, HZ, 4);
	void __iomem *base = host->base;
	u32 cmd_reg, arg_reg, resp0, dctrl_reg, dlen_reg, dcnt_reg;
	u32 mask0_reg, fifocnt_reg, status_reg;
	struct mmc_data *data = host->data;
	struct mmc_command *cmd = host->cmd;
	const char *dma_state;

	if (!__ratelimit(&rs))
		return;

	cmd_reg     = readl(base + MMCICOMMAND);
	arg_reg     = readl(base + MMCIARGUMENT);
	resp0       = readl(base + MMCIRESPONSE0);
	dctrl_reg   = readl(base + MMCIDATACTRL);
	dlen_reg    = readl(base + MMCIDATALENGTH);
	dcnt_reg    = readl(base + MMCIDATACNT);
	mask0_reg   = readl(base + MMCIMASK0);
	status_reg  = readl(base + MMCISTATUS);
	fifocnt_reg = host->variant->qcom_fifo ? readl(base + 0x44) : 0;

	/* Compare against legacy webOS reference (verified via /dev/mem
	 * dump of running 2.6.35-palm):
	 *   SDCC1 (eMMC):  CLKREG = 0x00009f00  (PWRSAVE+FLOWENA+FBCLK set)
	 *   SDCC4 (WiFi):  CLKREG = 0x00009b00  (PWRSAVE+FLOWENA+FBCLK set)
	 * Difference from mainline = bit 9 (MCI_CLK_PWRSAVE).
	 */
	dev_warn(mmc_dev(host->mmc),
		 "DIAG[%s]: CLKREG=0x%08x (legacy=0x9f00 sdcc1 / 0x9b00 sdcc4)\n",
		 reason, readl(base + MMCICLOCK));

	if (host->dma_in_progress)
		dma_state = "DMA_IN_PROGRESS";
	else if (host->dma_issue_deferred)
		dma_state = "DMA_DEFERRED";
	else if (host->atomic_submit.active)
		dma_state = "ATOMIC_SUBMIT";
	else
		dma_state = "PIO_OR_IDLE";

	dev_warn(mmc_dev(host->mmc),
		 "DIAG[%s]: STATUS=0x%08x DATACTRL=0x%08x DATALEN=%u DATACNT=%u MASK0=0x%08x FIFOCNT=%u\n",
		 reason, status_reg, dctrl_reg, dlen_reg, dcnt_reg,
		 mask0_reg, fifocnt_reg);
	/*
	 * Decode the latched error bits by name. The disambiguation we care
	 * about for the eMMC-under-WiFi fabric-starvation hypothesis:
	 *   RXOVERRUN set  -> the ADM stopped draining the RX FIFO in time
	 *                     (FIFO overflowed) => ADM/fabric drain starvation.
	 *   DATACRCFAIL only (no RXOVERRUN) -> trailing CRC16 mismatch from a
	 *                     byte-shift, also consistent with a transient
	 *                     overrun the controller swallowed, but check
	 *                     CRCI pacing / bus error too.
	 *   DATATIMEOUT/STARTBITERR -> card/link side, not fabric.
	 * Pair this with the ADM-side ADM_CH_RSLT / FLUSH_STATE0 dump
	 * (qcom_adm.c logs them for CRCI 1/5 on flush/error) by timestamp.
	 */
	dev_warn(mmc_dev(host->mmc),
		 "DIAG[%s]: errbits:%s%s%s%s%s%s  (RXOVERRUN=fabric/ADM drain starvation)\n",
		 reason,
		 (status_reg & MCI_RXOVERRUN)   ? " RXOVERRUN"   : "",
		 (status_reg & MCI_TXUNDERRUN)  ? " TXUNDERRUN"  : "",
		 (status_reg & MCI_DATACRCFAIL) ? " DATACRCFAIL" : "",
		 (status_reg & MCI_DATATIMEOUT) ? " DATATIMEOUT" : "",
		 (status_reg & MCI_STARTBITERR) ? " STARTBITERR" : "",
		 (status_reg & (MCI_RXOVERRUN | MCI_TXUNDERRUN | MCI_DATACRCFAIL |
				MCI_DATATIMEOUT | MCI_STARTBITERR)) ? "" : " (none latched)");
	dev_warn(mmc_dev(host->mmc),
		 "DIAG[%s]: cur_cmd=%p CMD_reg=0x%08x ARG=0x%08x RESP0=0x%08x dma=%s\n",
		 reason, cmd, cmd_reg, arg_reg, resp0, dma_state);
	if (data) {
		dev_warn(mmc_dev(host->mmc),
			 "DIAG[%s]: data: blksz=%u blocks=%u flags=0x%x sg_len=%u host->size=%u bytes_xfered=%u\n",
			 reason, data->blksz, data->blocks, data->flags,
			 data->sg_len, host->size, data->bytes_xfered);
	}
	if (cmd) {
		dev_warn(mmc_dev(host->mmc),
			 "DIAG[%s]: cmd: opcode=%u arg=0x%08x flags=0x%x\n",
			 reason, cmd->opcode, cmd->arg, cmd->flags);
	}
}

/*
 * Qualcomm SDCC controller reset, mirroring legacy msm_sdcc's
 * msmsdcc_reset_and_restore(): clk_reset the SDCC IP to wipe stale
 * CPSM/DPSM state from a wedged/half-finished transfer, then restore the
 * clock / power / interrupt-mask registers identically.  It is 2 MMIO
 * writes + udelay(2) and the qcom reset backend is regmap-mmio
 * (atomic-safe), so it is callable from both the data and command IRQ
 * paths.  No-op on variants without a reset control.
 */
static void mmci_qcom_reset_restore(struct mmci_host *host)
{
	if (!host->rst)
		return;

	reset_control_assert(host->rst);
	udelay(2);
	reset_control_deassert(host->rst);
	writel(host->clk_reg, host->base + MMCICLOCK);
	writel(host->pwr_reg, host->base + MMCIPOWER);
	writel(MCI_IRQENABLE | host->variant->start_err,
	       host->base + MMCIMASK0);
}

static void
mmci_data_irq(struct mmci_host *host, struct mmc_data *data,
	      unsigned int status)
{
	unsigned int status_err;

	/* Make sure we have data to handle */
	if (!data)
		return;

	/* Trace data IRQ for WiFi SDCC4 debugging */
	if (host->mmc->index == 1)
		mmci_qcom_diag("MMCI-DATA-IRQ: status=0x%08x blksz=%u blocks=%u\n",
			     status, data->blksz, data->blocks);

	/* First check for errors */
	status_err = status & (host->variant->start_err |
			       MCI_DATACRCFAIL | MCI_DATATIMEOUT |
			       MCI_TXUNDERRUN | MCI_RXOVERRUN);

	if (status_err) {
		u32 remain, success;

		/* Terminate the DMA transfer */
		mmci_dma_error(host);

		/*
		 * Calculate how far we are into the transfer.  Note that
		 * the data counter gives the number of bytes transferred
		 * on the MMC bus, not on the host side.  On reads, this
		 * can be as much as a FIFO-worth of data ahead.  This
		 * matters for FIFO overruns only.
		 */
		if (!host->variant->datacnt_useless) {
			remain = readl(host->base + MMCIDATACNT);
			success = data->blksz * data->blocks - remain;
		} else {
			success = 0;
		}

		dev_err(mmc_dev(host->mmc), "MCI ERROR IRQ, status 0x%08x at 0x%08x\n",
			status_err, success);
		if (status_err & MCI_DATACRCFAIL) {
			dev_err(mmc_dev(host->mmc), "DATACRCFAIL: blksz=%d blocks=%d flags=0x%x\n",
				data->blksz, data->blocks, data->flags);
			mmci_diag_dump_state(host, "DATACRCFAIL");
			/* Last block was not successful */
			success -= 1;
			data->error = -EILSEQ;
		} else if (status_err & MCI_DATATIMEOUT) {
			dev_err(mmc_dev(host->mmc), "DATATIMEOUT: blksz=%d blocks=%d flags=0x%x\n",
				data->blksz, data->blocks, data->flags);
			mmci_diag_dump_state(host, "DATATIMEOUT");
			data->error = -ETIMEDOUT;
		} else if (status_err & MCI_STARTBITERR) {
			data->error = -ECOMM;
		} else if (status_err & MCI_TXUNDERRUN) {
			data->error = -EIO;
		} else if (status_err & MCI_RXOVERRUN) {
			if (success > host->variant->fifosize)
				success -= host->variant->fifosize;
			else
				success = 0;
			data->error = -EIO;
		}
		data->bytes_xfered = round_down(success, data->blksz);
	}

	if (status & MCI_DATABLOCKEND)
		dev_err(mmc_dev(host->mmc), "stray MCI_DATABLOCKEND interrupt\n");

	if (status & MCI_DATAEND || data->error) {
		/*
		 * WiFi (mmc1) DMA READ: do NOT complete on DATAEND. On this SoC
		 * the SDCC raises DATAEND while the ADM is still draining the
		 * CRCI handshake — the dmaengine completion callback fires LATE,
		 * after DATAEND. Tearing down the DPSM here (mmci_stop_data) while
		 * the ADM is mid-transfer leaves the SDCC CPSM stuck in the
		 * data-state response window, so the NEXT command never completes
		 * (confirmed: the dummy52 CMD52 issued after a 128-byte HTC read
		 * never raises a completion IRQ → modprobe hangs in D-state).
		 *
		 * Defer to mmci_qcom_dma_complete, which runs from the ADM's own
		 * IRQ when the descriptor is truly done and the data path is
		 * clean. This matches legacy webOS msm_sdcc, which completes reads
		 * only from the msm_dmov complete_func, never from a SDCC DATAEND.
		 * The callback is wired for every mmc1 read (see _mmci_dmae_prep_
		 * data), so it is guaranteed to fire; the DATAEND bit was already
		 * cleared by mmci_irq, and qcom_dma_timeout_work is the backstop.
		 * Errors (DATACRCFAIL/RXOVERRUN/TIMEOUT) still complete here so the
		 * failure is handled promptly. Writes are unaffected (PROG_DONE
		 * path below). eMMC (mmc0) is unaffected.
		 */
		if (host->mmc->index == 1 && host->dma_in_progress &&
		    (data->flags & MMC_DATA_READ) && !data->error) {
			mmci_qcom_diag("MMCI-DATAEND: mmc1 read DATAEND IGNORED, defer to ADM callback blksz=%u blocks=%u\n",
				     data->blksz, data->blocks);
			return;
		}

		/*
		 * Tag DATAEND-driven completion of DMA transfers (low volume) so
		 * we can contrast against the mmci_qcom_dma_complete callback path
		 * and see, per direction and per controller (mmc0=eMMC,
		 * mmc1=WiFi), which transfers get a real DATAEND vs rely on the
		 * DMA-done callback.
		 */
		if (host->dma_in_progress)
			mmci_qcom_diag("MMCI-DATAEND: mmc%u completes %s via DATAEND status=0x%08x blksz=%u blocks=%u err=%d\n",
				     host->mmc->index,
				     (data->flags & MMC_DATA_READ) ? "read" : "WRITE",
				     status, data->blksz, data->blocks, data->error);

		/*
		 * WiFi (mmc1) DMA WRITE: wait for the card to finish programming
		 * (PROG_DONE, bit 23) before completing the request, so the next
		 * CMD53 is not issued while the AR6003 is still busy. A DMA
		 * write's DATAEND fires fast enough that the follow-up reg-table
		 * READ's CMD53 otherwise races ahead of PROG_DONE and CMDTIMEOUTs
		 * — the WMI-CONTROL connect failure (DIAG[CMD53-TO] showed
		 * PROG_DONE set only after the read had already been sent). PIO
		 * writes are slow enough to avoid this, so only the DMA path
		 * needs the wait. PROG_DONE arrives within microseconds for a
		 * 128-byte mailbox write; the bound only guards an error case.
		 */
		if (host->mmc->index == 1 && host->dma_in_progress &&
		    !(data->flags & MMC_DATA_READ) && !data->error) {
			u32 status;
			int ret;

			/* See companion comment in mmci_qcom_dma_complete:
			 * bound to 200 us to avoid millisecond-scale busy-spin
			 * in hardirq (Sashiko High #3).
			 */
			ret = readl_poll_timeout_atomic(host->base + MMCISTATUS,
							status,
							status & MCI_QCOM_PROGDONE,
							1, 200);
			if (ret)
				dev_warn_ratelimited(mmc_dev(host->mmc),
					"PROG_DONE not asserted within 200 us at data IRQ; CMD-path recovery will fire if next CMD fails\n");
			writel(MCI_QCOM_PROGDONE, host->base + MMCICLEAR);
		}

		if (host->variant->qcom_datactrl_delay)
			cancel_delayed_work(&host->qcom_dma_timeout_work);

		mmci_dma_finalize(host, data);

		mmci_stop_data(host);

		/*
		 * Qualcomm SDCC: on data CRC/timeout, mirror legacy
		 * msm_sdcc's msmsdcc_reset_and_restore() — clk_reset the
		 * SDCC IP (wipes stale DPSM/CPSM state from the
		 * half-finished transfer) then restore MMCICLOCK / MMCIPOWER
		 * / MMCIMASK0 identically (same 8-bit/48 MHz settings).
		 * Without this, the upcoming CMD12 / data->stop typically
		 * CMDTIMEOUTs (CPSM is still in DATA-state response window),
		 * mmc-core cascades through mmc_blk_reset → mmc_power_cycle
		 * → mmc_init_card → mmc_select_bus_width's verification path
		 * → second read CRC-fails under fabric contention → card
		 * falls back to 1-bit irreversibly.
		 *
		 * Reset is 2 MMIO writes + udelay(2); cheap inside the data
		 * IRQ path, and crucial that it happens BEFORE CMD12 below.
		 */
		if (data->error && host->variant->qcom_datactrl_delay)
			mmci_qcom_reset_restore(host);

		if (!data->error)
			/* The error clause is handled above, success! */
			data->bytes_xfered = data->blksz * data->blocks;

		/* Graded DMA->PIO recovery: arm a one-shot PIO retry if eMMC keeps flushing. */
		mmci_qcom_grade_recovery(host, data);

		if (!data->stop) {
			if (host->variant->cmdreg_stop && data->error)
				mmci_stop_command(host);
			else
				mmci_request_end(host, data->mrq);
		} else if (host->mrq->sbc && !data->error) {
			mmci_request_end(host, data->mrq);
		} else {
			/* See same-block comment in mmci_qcom_dma_complete:
			 * clear atomic_submit so the stop CMD12 is actually
			 * issued instead of being swallowed by the stash path.
			 */
			host->atomic_submit.armed = false;
			host->atomic_submit.active = false;
			mmci_start_command(host, data->stop, 0);
		}
	}
}

static void
mmci_cmd_irq(struct mmci_host *host, struct mmc_command *cmd,
	     unsigned int status)
{
	u32 err_msk = MCI_CMDCRCFAIL | MCI_CMDTIMEOUT;
	void __iomem *base = host->base;
	bool sbc, busy_resp;

	if (!cmd)
		return;

	/*
	 * Dummy CMD52 completion path: the response content is ignored
	 * (any of CMDSENT / CMDRESPEND / CMDCRCFAIL / CMDTIMEOUT means
	 * the CPSM has finished running and the residual SDCC data-path
	 * state has been drained). Clear in-progress, dispatch the real
	 * request that was stashed at request entry.
	 */
	if (host->dummy52_in_progress && cmd == &host->dummy52_cmd) {
		if (!(status & (MCI_CMDSENT | MCI_CMDRESPEND |
				MCI_CMDCRCFAIL | MCI_CMDTIMEOUT)))
			return;

		mmci_qcom_diag("dummy52: CMD52 completed, status=0x%08x\n", status);
		host->cmd = NULL;
		host->dummy52_in_progress = false;
		cancel_delayed_work(&host->qcom_dummy52_watchdog);

		if (host->pending_mrq) {
			struct mmc_request *real_mrq = host->pending_mrq;

			host->pending_mrq = NULL;
			mmci_qcom_diag("dummy52: dispatching pending request cmd=%u\n",
				     real_mrq->cmd->opcode);
			__mmci_start_request(host, real_mrq);
		}
		return;
	}

	sbc = (cmd == host->mrq->sbc);
	busy_resp = !!(cmd->flags & MMC_RSP_BUSY);

	/*
	 * We need to be one of these interrupts to be considered worth
	 * handling. Note that we tag on any latent IRQs postponed
	 * due to waiting for busy status.
	 */
	if (host->variant->busy_timeout && busy_resp)
		err_msk |= MCI_DATATIMEOUT;

	if (!((status | host->busy_status) &
	      (err_msk | MCI_CMDSENT | MCI_CMDRESPEND)))
		return;

	/* Handle busy detection on DAT0 if the variant supports it. */
	if (busy_resp && host->variant->busy_detect)
		if (!host->ops->busy_complete(host, cmd, status, err_msk))
			return;

	host->cmd = NULL;

	if (status & MCI_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT;
		if (host->variant->qcom_datactrl_delay)
			dev_err(mmc_dev(host->mmc),
				"CMDTIMEOUT: cmd%d arg=0x%08x status=0x%08x data=%s\n",
				cmd->opcode, cmd->arg, status,
				str_yes_no(host->data));
		/*
		 * WiFi (mmc1) CMD53 timeout diag: dump the CPSM/DPSM state to
		 * understand why the command after a DMA write gets no response
		 * (WMI WRITE->READ CMDTIMEOUT). datactrl_first means DATACTRL is
		 * armed before the command; a stuck DPSM from the prior write
		 * would show here.
		 */
		if (host->mmc->index == 1 && cmd->opcode == SD_IO_RW_EXTENDED)
			dev_err(mmc_dev(host->mmc),
				"DIAG[CMD53-TO]: STATUS=0x%08x DATACTRL=0x%08x DATACNT=%u CLK=0x%08x MASK0=0x%08x CMD=0x%08x atomic=%d defer=%d\n",
				readl(host->base + MMCISTATUS),
				readl(host->base + MMCIDATACTRL),
				readl(host->base + MMCIDATACNT),
				readl(host->base + MMCICLOCK),
				readl(host->base + MMCIMASK0),
				readl(host->base + MMCICOMMAND),
				host->atomic_submit.active,
				host->dma_issue_deferred);

		/*
		 * DEBUG TLB-2026-06-13: eMMC (mmc0) CMDTIMEOUT diag.  When the
		 * chip silently drops a CMD on the cmd line (no response at
		 * all -- not even a CRC error), the controller logs CMDTIMEOUT
		 * but we don't know whether the cmd line clock was even
		 * driving correctly at the moment of failure.  Dump the
		 * actual clk_get_rate() on host->clk plus the MMCICLOCK
		 * divider register and the MMCISTATUS bits so we can correlate
		 * with msm_serial's OPP-rate transition timestamps for
		 * 16540000.serial (the BT UART, which retunes through PLL8 --
		 * the same PLL parent as sdc1_clk -- during BCSP WARM_RESET +
		 * baud switch).  If both happen in the same ~tens-of-ms window
		 * the PLL-relock-glitch theory is confirmed.
		 *
		 * One-shot per mmci instance to keep the log small (only the
		 * first CMDTIMEOUT carries meaningful clock-state context --
		 * subsequent ones are cmd12/cmd13 recovery cascade triggered
		 * by the original timeout, so their clock readings are after
		 * the chip has already wedged).
		 */
		/*
		 * Filter out the SDIO-card-detection probe commands that the mmc
		 * core legitimately issues to every host at boot, including the
		 * eMMC host (mmc0).  The eMMC chip only responds to MMC-class
		 * commands (CMD1, CMD2, ..., CMD23 for SET_BLOCK_COUNT) so these
		 * SDIO discovery cmds always cmdtimeout -- they're not the real
		 * failure we are chasing, and they would consume the one-shot
		 * cmdto_diag_seen flag before the real cmd23 cascade fires.
		 *
		 * Excluded opcodes:
		 *   CMD5  (SD_IO_SEND_OP_COND, opcode 5)  -- SDIO init
		 *   CMD8  (SEND_IF_COND, opcode 8)        -- SD interface check
		 *   CMD52 (SD_IO_RW_DIRECT, opcode 52)    -- SDIO IO_RW
		 *   CMD53 (SD_IO_RW_EXTENDED, opcode 53)  -- SDIO IO_RW (handled
		 *                                            by the mmc1-specific
		 *                                            CMD53 diag above)
		 *   CMD55 (APP_CMD, opcode 55)            -- SD-specific app-cmd
		 *                                            prefix
		 */
		if (host->mmc->index == 0 &&
		    cmd->opcode != SD_IO_SEND_OP_COND &&
		    cmd->opcode != SD_SEND_IF_COND &&
		    cmd->opcode != SD_IO_RW_DIRECT &&
		    cmd->opcode != SD_IO_RW_EXTENDED &&
		    cmd->opcode != MMC_APP_CMD &&
		    !host->cmdto_diag_seen) {
			unsigned long hclk_hz = host->clk ?
						 clk_get_rate(host->clk) : 0UL;

			dev_err(mmc_dev(host->mmc),
				"DIAG[eMMC-CMDTO]: cmd%d arg=0x%08x  clk_get_rate=%lu Hz  MMCICLOCK=0x%08x  STATUS=0x%08x  DATACTRL=0x%08x  DATACNT=%u  jiffies=%lu\n",
				cmd->opcode, cmd->arg, hclk_hz,
				readl(host->base + MMCICLOCK),
				readl(host->base + MMCISTATUS),
				readl(host->base + MMCIDATACTRL),
				readl(host->base + MMCIDATACNT),
				jiffies);
			host->cmdto_diag_seen = true;
		}

		/*
		 * Qualcomm SDCC: a command-response timeout on a busy (R1b)
		 * command -- e.g. the eMMC CMD6 SWITCH issued during init or a
		 * partition switch -- leaves the CPSM wedged so every following
		 * CMD also times out.  This is the intermittent warm-boot eMMC
		 * failure: a storm of CMD6 CMDTIMEOUTs -> I/O error -> rootfs
		 * mount fails.  Mirror legacy msm_sdcc's
		 * msmsdcc_reset_and_restore() and reset the controller now so the
		 * mmc-core retry hits a clean command engine.  Gated on busy_resp
		 * so the benign SD/SDIO probe timeouts (CMD5/8/52/55, not R1b)
		 * and the WiFi data path are left untouched.
		 */
		if (host->variant->qcom_datactrl_delay && busy_resp)
			mmci_qcom_reset_restore(host);
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ;
	} else if (host->variant->busy_timeout && busy_resp &&
		   status & MCI_DATATIMEOUT) {
		cmd->error = -ETIMEDOUT;
		/*
		 * This will wake up mmci_irq_thread() which will issue
		 * a hardware reset of the MMCI block.
		 */
		host->irq_action = IRQ_WAKE_THREAD;
	} else {
		cmd->resp[0] = readl(base + MMCIRESPONSE0);
		cmd->resp[1] = readl(base + MMCIRESPONSE1);
		cmd->resp[2] = readl(base + MMCIRESPONSE2);
		cmd->resp[3] = readl(base + MMCIRESPONSE3);
	}

	if ((!sbc && !cmd->data) || cmd->error) {
		if (host->data) {
			/* Terminate the DMA transfer */
			mmci_dma_error(host);

			mmci_stop_data(host);
			if (host->variant->cmdreg_stop && cmd->error) {
				/*
				 * Clear deferred DMA flag - the DMA was
				 * terminated above, don't issue it.  Drop
				 * any stashed DATACTRL too; the chip didn't
				 * see CMD53 so DPSM never needs arming.
				 */
				host->dma_issue_deferred = false;
				host->deferred_datactrl_pending = false;
				mmci_stop_command(host);
				return;
			}
		}

		/* Clear deferred DMA flag on error/no-data path */
		host->dma_issue_deferred = false;
		host->deferred_datactrl_pending = false;

		if (host->irq_action != IRQ_WAKE_THREAD)
			mmci_request_end(host, host->mrq);

	} else if (sbc) {
		mmci_start_command(host, host->mrq->cmd, 0);
	} else if (!host->datactrl_first &&
		   !(cmd->data->flags & MMC_DATA_READ)) {
		mmci_start_data(host, cmd->data);
	}

	/*
	 * Qualcomm ADM DMA with datactrl_first writes: issue deferred
	 * DMA pending now that CMD has completed.
	 *
	 * For SDIO WRITES we also defer the DATACTRL write itself
	 * (see mmci_dma_start defer_for_sdio_write branch).  The
	 * matching write happens here, before dma_issue_pending,
	 * so the order on the bus is exactly the legacy webOS
	 * msm_sdcc msmsdcc_dma_exec_func sequence:
	 *
	 *   chip ACKs CMD53 -> DATACTRL -> udelay -> ADM kicks
	 *   CMD_PTR_LIST -> CRCI handshake -> data on wire.
	 *
	 * The DPSM is never armed before CMD53 reaches the chip,
	 * which avoids the AR6003 "silent CMD53 NAK" failure mode
	 * that historically forced <256 B mmc1 SDIO writes through
	 * PIO (mmci.c:1614).
	 */
	if (host->dma_issue_deferred) {
		host->dma_issue_deferred = false;
		if (host->deferred_datactrl_pending) {
			host->deferred_datactrl_pending = false;
			mmci_write_datactrlreg(host, host->deferred_datactrl);
			if (host->variant->qcom_datactrl_delay) {
				wmb();
				udelay(1);
			}
		}
		if (host->ops && host->ops->dma_issue_pending) {
			host->ops->dma_issue_pending(host);
			/*
			 * 500 ms is enough for any real DMA — typical SDIO
			 * transfers complete in microseconds. The previous 3 s
			 * timeout was wide enough to coincide with eMMC's
			 * natural multi-block-write busy windows, falsely
			 * triggering a forced terminate that cascaded into
			 * eMMC journal abort. Only legitimate WiFi-chip hangs
			 * (waiting on response that never comes) need this
			 * watchdog now and 500 ms catches those just fine.
			 */
			schedule_delayed_work(
				&host->qcom_dma_timeout_work,
				msecs_to_jiffies(500));
		}
	}
}

static char *ux500_state_str(struct mmci_host *host)
{
	switch (host->busy_state) {
	case MMCI_BUSY_WAITING_FOR_START_IRQ:
		return "waiting for start IRQ";
	case MMCI_BUSY_WAITING_FOR_END_IRQ:
		return "waiting for end IRQ";
	case MMCI_BUSY_DONE:
		return "not waiting for IRQs";
	default:
		return "unknown";
	}
}

/*
 * This busy timeout worker is used to "kick" the command IRQ if a
 * busy detect IRQ fails to appear in reasonable time. Only used on
 * variants with busy detection IRQ delivery.
 */
static void ux500_busy_timeout_work(struct work_struct *work)
{
	struct mmci_host *host = container_of(work, struct mmci_host,
					ux500_busy_timeout_work.work);
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&host->lock, flags);

	if (host->cmd) {
		/* If we are still busy let's tag on a cmd-timeout error. */
		status = readl(host->base + MMCISTATUS);
		if (status & host->variant->busy_detect_flag) {
			status |= MCI_CMDTIMEOUT;
			dev_err(mmc_dev(host->mmc),
				"timeout in state %s still busy with CMD%02x\n",
				ux500_state_str(host), host->cmd->opcode);
		} else {
			dev_err(mmc_dev(host->mmc),
				"timeout in state %s waiting for busy CMD%02x\n",
				ux500_state_str(host), host->cmd->opcode);
		}

		mmci_cmd_irq(host, host->cmd, status);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Qualcomm SDCC DMA data timeout worker.
 *
 * On Qualcomm SDCC with ADM DMA using deferred DMA issue, the hardware
 * occasionally fails to generate DATAEND or DATATIMEOUT after a DMA write
 * transfer completes. The ADM DMA engine reports success but the SDCC
 * Data Path State Machine (DPSM) gets stuck, causing the request to hang
 * indefinitely in mmc_wait_for_req_done().
 *
 * This watchdog detects the hang by firing a few seconds after deferred
 * DMA is issued. If the data transfer is still pending, it dumps the
 * SDCC register state for debugging and force-terminates the transfer
 * with -ETIMEDOUT so the SDIO layer can retry.
 */
static void qcom_dma_data_timeout_work(struct work_struct *work)
{
	struct mmci_host *host = container_of(work, struct mmci_host,
					qcom_dma_timeout_work.work);
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&host->lock, flags);

	if (!host->data) {
		/* Transfer already completed normally */
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	status = readl(host->base + MMCISTATUS);
	dev_err(mmc_dev(host->mmc),
		"DMA data timeout! status=0x%08x datactrl=0x%08x "
		"datalength=0x%08x mask0=0x%08x\n",
		status,
		readl(host->base + MMCIDATACTRL),
		readl(host->base + MMCIDATALENGTH),
		readl(host->base + MMCIMASK0));

	/* Terminate the DMA transfer */
	mmci_dma_error(host);

	host->data->error = -ETIMEDOUT;
	host->data->bytes_xfered = 0;

	mmci_stop_data(host);

	if (host->mrq)
		mmci_request_end(host, host->mrq);

	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Qualcomm SDCC dummy-CMD52 watchdog.
 *
 * The dummy CMD52 errata workaround injects a CCCR-reg-0 read between a
 * preceding CMD53 and the real request that mmc-core handed us, so the
 * controller's residual DPSM state gets drained.  CMD52 carries no data
 * phase so it normally completes within microseconds.  If the CPSM
 * wedges (no CMDSENT / CMDRESPEND / CMDTIMEOUT), the in-progress
 * stashed mmc_request never reaches mmc_request_done and the caller
 * hangs in D-state until mmc-core's own ~1 s timeout kicks in.
 *
 * Catch it at 100 ms here, dump SDCC state, and force-recover by
 * dispatching the pending real request with -ETIMEDOUT so the SDIO
 * stack can issue its own retry / reset.
 */
static void qcom_dummy52_watchdog_work(struct work_struct *work)
{
	struct mmci_host *host = container_of(work, struct mmci_host,
					      qcom_dummy52_watchdog.work);
	struct mmc_request *mrq;
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&host->lock, flags);

	if (!host->dummy52_in_progress) {
		/* Normal completion already fired -- nothing to do. */
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	status = readl(host->base + MMCISTATUS);
	dev_err(mmc_dev(host->mmc),
		"dummy52 watchdog: CPSM wedged, status=0x%08x command=0x%08x\n",
		status, readl(host->base + MMCICOMMAND));

	host->dummy52_in_progress = false;
	host->cmd = NULL;
	mrq = host->pending_mrq;
	host->pending_mrq = NULL;

	if (mrq) {
		/*
		 * Fail the real request rather than re-dispatching: the CPSM
		 * is in an unknown state and starting another command would
		 * likely wedge again.  Returning -ETIMEDOUT lets the SDIO
		 * stack run its own recovery (which may reset the host).
		 */
		mrq->cmd->error = -ETIMEDOUT;
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(host->mmc, mrq);
		return;
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_get_rx_fifocnt(struct mmci_host *host, u32 status, int remain)
{
	return remain - (readl(host->base + MMCIFIFOCNT) << 2);
}

static int mmci_qcom_get_rx_fifocnt(struct mmci_host *host, u32 status, int r)
{
	/*
	 * on qcom SDCC4 only 8 words are used in each burst so only 8 addresses
	 * from the fifo range should be used
	 */
	if (status & MCI_RXFIFOHALFFULL)
		return host->variant->fifohalfsize;
	else if (status & MCI_RXDATAAVLBL)
		return 4;

	return 0;
}

static int mmci_pio_read(struct mmci_host *host, char *buffer, unsigned int remain)
{
	void __iomem *base = host->base;
	char *ptr = buffer;
	u32 status = readl(host->base + MMCISTATUS);
	int host_remain = host->size;

	do {
		int count = host->get_rx_fifocnt(host, status, host_remain);

		if (count > remain)
			count = remain;

		if (count <= 0)
			break;

		/*
		 * SDIO especially may want to send something that is
		 * not divisible by 4 (as opposed to card sectors
		 * etc). Therefore make sure to always read the last bytes
		 * while only doing full 32-bit reads towards the FIFO.
		 */
		if (unlikely(count & 0x3)) {
			if (count < 4) {
				unsigned char buf[4];
				ioread32_rep(base + MMCIFIFO, buf, 1);
				memcpy(ptr, buf, count);
			} else {
				ioread32_rep(base + MMCIFIFO, ptr, count >> 2);
				count &= ~0x3;
			}
		} else {
			ioread32_rep(base + MMCIFIFO, ptr, count >> 2);
		}

		ptr += count;
		remain -= count;
		host_remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_RXDATAAVLBL);

	return ptr - buffer;
}

static int mmci_pio_write(struct mmci_host *host, char *buffer, unsigned int remain, u32 status)
{
	struct variant_data *variant = host->variant;
	void __iomem *base = host->base;
	char *ptr = buffer;

	do {
		unsigned int count, maxcnt;

		maxcnt = status & MCI_TXFIFOEMPTY ?
			 variant->fifosize : variant->fifohalfsize;
		count = min(remain, maxcnt);

		/*
		 * SDIO especially may want to send something that is
		 * not divisible by 4 (as opposed to card sectors
		 * etc), and the FIFO only accept full 32-bit writes.
		 * So compensate by adding +3 on the count, a single
		 * byte become a 32bit write, 7 bytes will be two
		 * 32bit writes etc.
		 */
		iowrite32_rep(base + MMCIFIFO, ptr, (count + 3) >> 2);

		ptr += count;
		remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_TXFIFOHALFEMPTY);

	return ptr - buffer;
}

/*
 * PIO data transfer IRQ handler.
 */
static irqreturn_t mmci_pio_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	struct variant_data *variant = host->variant;
	void __iomem *base = host->base;
	u32 status;

	/*
	 * Legacy webOS parity safety net: when the qcom variant arms the
	 * RX/TX FIFO half-full/half-empty mask alongside DMA (see
	 * mmci_dma_start, gated on qcom_dml_atomic_submit), the FIFO IRQ
	 * will fire during a normal DMA transfer too.  The PIO drain path
	 * below uses host->sg_miter which is only initialised by
	 * mmci_init_sg() on the PIO fallback path -- entering it while DMA
	 * is in flight would dereference an uninitialised iterator.
	 *
	 * Just ACK the IRQ and return: ADM is draining (or the DATAEND /
	 * DATACRCFAIL / DATATIMEOUT path will fire next if it isn't).  The
	 * value of arming the mask at all is the BACKSTOP behaviour: if the
	 * ADM truly stalls mid-burst the FIFO stays full, this IRQ keeps
	 * firing harmlessly, and the next DATATIMEOUT / DATACRCFAIL still
	 * arrives on schedule via the existing mmci_data_irq path.
	 *
	 * Use READ_ONCE: the field is written under host->lock elsewhere
	 * but read here without the lock; the qualifier prevents the
	 * compiler from re-loading / tearing the access.
	 */
	if (READ_ONCE(host->dma_in_progress)) {
		/*
		 * DEBUG one-shot per host: prove the dma_in_progress bail-out
		 * isn't accidentally firing for an actual PIO host.
		 */
		if (!host->pio_dma_bailed_logged) {
			host->pio_dma_bailed_logged = 1;
			dev_warn(mmc_dev(host->mmc),
				 "mmci_pio_irq: bailed out (dma_in_progress=1) — DEBUG marker\n");
		}
		return IRQ_HANDLED;
	}

	status = readl(base + MMCISTATUS);

	/*
	 * DEBUG one-shot per host: prove mmci_pio_irq is being entered at
	 * all. If we never see this on a wedged boot, mmci_irq is not
	 * dispatching to us - the FIFO IRQ isn't firing in MMCISTATUS or
	 * the dispatch gate (status & mask1_reg) didn't match.
	 */
	if (!host->pio_entered_logged) {
		host->pio_entered_logged = 1;
		dev_info(mmc_dev(host->mmc),
			 "mmci_pio_irq: ENTERED first time; status=0x%08x mask1=0x%08x\n",
			 status, host->mask1_reg);
	}

	dev_dbg(mmc_dev(host->mmc), "irq1 (pio) %08x\n", status);

	do {
		unsigned int remain, len;
		char *buffer;

		/*
		 * For write, we only need to test the half-empty flag
		 * here - if the FIFO is completely empty, then by
		 * definition it is more than half empty.
		 *
		 * For read, check for data available.
		 */
		if (!(status & (MCI_TXFIFOHALFEMPTY|MCI_RXDATAAVLBL)))
			break;

		if (!sg_miter_next(sg_miter))
			break;

		buffer = sg_miter->addr;
		remain = sg_miter->length;

		len = 0;
		if (status & MCI_RXACTIVE)
			len = mmci_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = mmci_pio_write(host, buffer, remain, status);

		sg_miter->consumed = len;

		host->size -= len;
		remain -= len;

		if (remain)
			break;

		status = readl(base + MMCISTATUS);
	} while (1);

	sg_miter_stop(sg_miter);

	/*
	 * If we have less than the fifo 'half-full' threshold to transfer,
	 * trigger a PIO interrupt as soon as any data is available.
	 */
	if (status & MCI_RXACTIVE && host->size < variant->fifohalfsize)
		mmci_set_mask1(host, MCI_RXDATAAVLBLMASK);

	/*
	 * If we run out of data, disable the data IRQs; this
	 * prevents a race where the FIFO becomes empty before
	 * the chip itself has disabled the data path, and
	 * stops us racing with our data end IRQ.
	 */
	if (host->size == 0) {
		mmci_set_mask1(host, 0);
		writel(readl(base + MMCIMASK0) | MCI_DATAENDMASK, base + MMCIMASK0);
	}

	return IRQ_HANDLED;
}

static void mmci_write_sdio_irq_bit(struct mmci_host *host, int enable)
{
	void __iomem *base = host->base;
	u32 mask = readl_relaxed(base + MMCIMASK0);

	if (enable) {
		writel_relaxed(mask | MCI_ST_SDIOITMASK, base + MMCIMASK0);
	} else {
		writel_relaxed(mask & ~MCI_ST_SDIOITMASK, base + MMCIMASK0);
	}
}

static void mmci_signal_sdio_irq(struct mmci_host *host, u32 status)
{
	if (status & MCI_ST_SDIOIT) {
		dev_dbg(mmc_dev(host->mmc),
			"SDIO IRQ received, status=0x%08x\n", status);
		mmci_write_sdio_irq_bit(host, 0);
		sdio_signal_irq(host->mmc);
	}
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t mmci_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	u32 status;

	spin_lock(&host->lock);
	host->irq_action = IRQ_HANDLED;

	do {
		status = readl(host->base + MMCISTATUS);

		/* Trace IRQ entry for WiFi SDCC4 debugging */
		if (host->mmc->index == 1 && status)
			mmci_qcom_diag("MMCI-IRQ: status=0x%08x mask0=0x%08x\n",
				     status, readl(host->base + MMCIMASK0));

		if (!status)
			break;

		if (host->singleirq) {
			if (status & host->mask1_reg)
				mmci_pio_irq(irq, dev_id);

			status &= ~host->variant->irq_pio_mask;
		}

		/*
		 * Busy detection is managed by mmci_cmd_irq(), including to
		 * clear the corresponding IRQ.
		 */
		status &= readl(host->base + MMCIMASK0);

		if (host->variant->busy_detect)
			writel(status & ~host->variant->busy_detect_mask,
			       host->base + MMCICLEAR);
		else
			writel(status, host->base + MMCICLEAR);

		dev_dbg(mmc_dev(host->mmc), "irq0 (data+cmd) %08x\n", status);

		if (host->variant->reversed_irq_handling) {
			mmci_data_irq(host, host->data, status);
			mmci_cmd_irq(host, host->cmd, status);
		} else {
			mmci_cmd_irq(host, host->cmd, status);
			mmci_data_irq(host, host->data, status);
		}

		if (host->variant->supports_sdio_irq)
			mmci_signal_sdio_irq(host, status);

		/*
		 * Busy detection has been handled by mmci_cmd_irq() above.
		 * Clear the status bit to prevent polling in IRQ context.
		 */
		if (host->variant->busy_detect_flag)
			status &= ~host->variant->busy_detect_flag;

	} while (status);

	spin_unlock(&host->lock);

	return host->irq_action;
}

/*
 * mmci_irq_thread() - A threaded IRQ handler that manages a reset of the HW.
 *
 * A reset is needed for some variants, where a datatimeout for a R1B request
 * causes the DPSM to stay busy (non-functional).
 */
static irqreturn_t mmci_irq_thread(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	unsigned long flags;

	if (host->rst) {
		reset_control_assert(host->rst);
		udelay(2);
		reset_control_deassert(host->rst);
	}

	spin_lock_irqsave(&host->lock, flags);
	writel(host->clk_reg, host->base + MMCICLOCK);
	writel(host->pwr_reg, host->base + MMCIPOWER);
	writel(MCI_IRQENABLE | host->variant->start_err,
	       host->base + MMCIMASK0);

	host->irq_action = IRQ_HANDLED;
	mmci_request_end(host, host->mrq);
	spin_unlock_irqrestore(&host->lock, flags);

	return host->irq_action;
}

/*
 * Locked helper: start a real mmc_request. Assumes host->lock is held
 * and host->mrq is NULL.
 *
 * Factored out so it can be invoked both from mmci_request() (after
 * lock acquisition) and from the dummy52-completion path in
 * mmci_cmd_irq() where the lock is already held by the IRQ handler.
 */
static void __mmci_start_request(struct mmci_host *host,
				 struct mmc_request *mrq)
{
	host->mrq = mrq;
	host->atomic_submit.armed = false;
	host->atomic_submit.active = false;
	host->deferred_datactrl_pending = false;

	if (mrq->data)
		mmci_get_next_data(host, mrq->data);

	if (mrq->data &&
	    (host->datactrl_first || mrq->data->flags & MMC_DATA_READ)) {
		mmci_start_data(host, mrq->data);
		/*
		 * Clock-dependent data-to-cmd settle.  Legacy msm_sdcc uses
		 * msmsdcc_delay() = 1 + 3000000/clk_rate (≈1 µs @ 48 MHz,
		 * ≈8.5 µs @ 400 kHz init clock).  The previous hardcoded
		 * udelay(1) was too short at init clock — this is the path
		 * that issues the EXT_CSD READ on Samsung PRV=0x90.
		 */
		if (host->variant->qcom_datactrl_delay)
			udelay(mmci_qcom_settle_us(host));
	}

	if (mrq->sbc)
		mmci_start_command(host, mrq->sbc, 0);
	else
		mmci_start_command(host, mrq->cmd, 0);

	if (host->atomic_submit.active && host->ops &&
	    host->ops->dma_issue_pending) {
		host->ops->dma_issue_pending(host);
	}

}

static void mmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;

	WARN_ON(host->mrq != NULL);

	mrq->cmd->error = mmci_validate_data(host, mrq->data);
	if (mrq->cmd->error) {
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Qualcomm SDCC dummy CMD52 errata: after a CMD53 data transfer, the
	 * SDCC DPSM retains residual state. Drain it by issuing a CMD52
	 * (read of CCCR reg 0, function 0) before the triggering command,
	 * then dispatch the real request from mmci_cmd_irq.
	 *
	 * Drain direction per controller:
	 *  - eMMC (mmc0): WRITE->WRITE hits DATACRCFAIL → drain before WRITE.
	 *  - WiFi (mmc1): drain before WRITE. This covers:
	 *    (a) WRITE->WRITE (original errata)
	 *    (b) READ->WRITE (DMA read callback leaves DPSM dirty → next
	 *        CMD53 WRITE CMDTIMEOUTs; confirmed on HTC START after 6
	 *        service-connect reads)
	 *    Never drain before READ: read→read chains work fine, and a
	 *    drain before a read during BMI/HTC poll causes 800 ms timeout.
	 */
	if (host->dummy52_required && host->dummy52_needed) {
		host->dummy52_needed = false;

		if (mrq->cmd->opcode == SD_IO_RW_EXTENDED && mrq->data) {
			bool is_write = mrq->data->flags & MMC_DATA_WRITE;

			if (is_write) {
				host->dummy52_in_progress = true;
				host->pending_mrq = mrq;
				/*
				 * CRITICAL: clear the atomic-submit state before
				 * issuing the dummy CMD52. mmci_should_atomic_submit
				 * sets atomic_submit.active=true for the preceding WiFi
				 * DMA READ, and it is only cleared at the top of
				 * __mmci_start_request — which we bypass here. If left
				 * set, mmci_start_command STASHES the CMD52 (ARG/CMD
				 * saved for the ADM exec_func) instead of writing
				 * MMCICOMMAND, so the command is never issued, no
				 * completion IRQ fires, and modprobe hangs forever in
				 * D-state. The dummy CMD52 carries no data and must go
				 * through the conventional command path.
				 */
				host->atomic_submit.armed = false;
				host->atomic_submit.active = false;
				dev_dbg(mmc_dev(mmc),
					"dummy52: dispatching before CMD53 WRITE\n");
				mmci_start_command(host, &host->dummy52_cmd, 0);
				/*
				 * 100 ms backstop -- CMD52 normally completes
				 * in microseconds; we only need to catch a
				 * wedged CPSM (Sashiko Medium #46).
				 */
				schedule_delayed_work(
					&host->qcom_dummy52_watchdog,
					msecs_to_jiffies(100));
				spin_unlock_irqrestore(&host->lock, flags);
				return;
			}
		}
	}

	__mmci_start_request(host, mrq);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void mmci_set_max_busy_timeout(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	u32 max_busy_timeout = 0;

	if (!host->variant->busy_detect)
		return;

	if (host->variant->busy_timeout && mmc->actual_clock)
		max_busy_timeout = U32_MAX / DIV_ROUND_UP(mmc->actual_clock,
							  MSEC_PER_SEC);

	mmc->max_busy_timeout = max_busy_timeout;
}

static void mmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct variant_data *variant = host->variant;
	u32 pwr = 0;
	unsigned long flags;
	int ret;

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		/*
		 * Set low load on qcom variant before power off.
		 *
		 * SDIO consumers managed via mmc-pwrseq (e.g. AR6003 WiFi on
		 * APQ8060/tenderloin) keep their chip alive across host
		 * power-off and only resume cleanly if the I/O supply (vqmmc)
		 * stays in LPM rather than dropping the consumer's load vote
		 * to zero.  Legacy webOS tenderloin_wifi_power() sets 10000
		 * uA (MIN_LOAD) on disable; matching that here prevents the
		 * AR6003 SDIO state machine from desyncing across host
		 * power-cycle.  Non-pwrseq hosts retain the historical
		 * full-zero vote.
		 */
		if (variant->qcom_fifo && (mmc->caps & MMC_CAP_SDIO_IRQ)) {
			u32 vqmmc_load = mmc->pwrseq ? 10000 : 0;

			if (!IS_ERR(mmc->supply.vqmmc))
				regulator_set_load(mmc->supply.vqmmc, vqmmc_load);
			if (!IS_ERR(mmc->supply.vmmc))
				regulator_set_load(mmc->supply.vmmc, 0);
		}

		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

		if (!IS_ERR(mmc->supply.vqmmc) && host->vqmmc_enabled) {
			regulator_disable(mmc->supply.vqmmc);
			host->vqmmc_enabled = false;
		}

		break;
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);

		/*
		 * The ST Micro variant doesn't have the PL180s MCI_PWR_UP
		 * and instead uses MCI_PWR_ON so apply whatever value is
		 * configured in the variant data.
		 */
		pwr |= variant->pwrreg_powerup;

		break;
	case MMC_POWER_ON:
		if (!IS_ERR(mmc->supply.vqmmc) && !host->vqmmc_enabled) {
			ret = regulator_enable(mmc->supply.vqmmc);
			if (ret < 0)
				dev_err(mmc_dev(mmc),
					"failed to enable vqmmc regulator\n");
			else
				host->vqmmc_enabled = true;
		}

		/* Set high load for SDIO WiFi on qcom variant */
		if (variant->qcom_fifo && (mmc->caps & MMC_CAP_SDIO_IRQ)) {
			if (!IS_ERR(mmc->supply.vqmmc))
				regulator_set_load(mmc->supply.vqmmc, 100000);
			if (!IS_ERR(mmc->supply.vmmc))
				regulator_set_load(mmc->supply.vmmc, 100000);
		}

		pwr |= MCI_PWR_ON;
		break;
	}

	if (variant->signal_direction && ios->power_mode != MMC_POWER_OFF) {
		/*
		 * The ST Micro variant has some additional bits
		 * indicating signal direction for the signals in
		 * the SD/MMC bus and feedback-clock usage.
		 */
		pwr |= host->pwr_reg_add;

		if (ios->bus_width == MMC_BUS_WIDTH_4)
			pwr &= ~MCI_ST_DATA74DIREN;
		else if (ios->bus_width == MMC_BUS_WIDTH_1)
			pwr &= (~MCI_ST_DATA74DIREN &
				~MCI_ST_DATA31DIREN &
				~MCI_ST_DATA2DIREN);
	}

	if (variant->opendrain) {
		if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
			pwr |= variant->opendrain;
	} else {
		/*
		 * If the variant cannot configure the pads by its own, then we
		 * expect the pinctrl to be able to do that for us
		 */
		if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
			pinctrl_select_state(host->pinctrl, host->pins_opendrain);
		else
			pinctrl_select_default_state(mmc_dev(mmc));
	}

	/*
	 * If clock = 0 and the variant requires the MMCIPOWER to be used for
	 * gating the clock, the MCI_PWR_ON bit is cleared.
	 */
	if (!ios->clock && variant->pwrreg_clkgate)
		pwr &= ~MCI_PWR_ON;

	/*
	 * Legacy msm_sdcc snaps any (fmid, fmax) = (24, 48) MHz request down
	 * to 24 MHz. gcc-msm8660's clk_tbl_sdc[] has only discrete steps
	 * {400 kHz, 16, 17.07, 20.21, 24, 48} MHz — nothing between 24 and
	 * 48. If the MMC core asks for, e.g., 26 MHz (legacy MMC HS or the
	 * intermediate of a DDR50 probe), the clock framework may round UP
	 * to 48 MHz before the controller is configured for HS mode. Signal
	 * sampling at the wrong edge then produces DATACRCFAIL on the first
	 * transfer at that rate. Mirror legacy's defensive clamp.
	 *
	 * clock_cache tracks the applied (post-snap) rate so re-requests of
	 * the same in-between rate hit the cache and skip clk_set_rate.
	 */
	{
		unsigned int set_rate = ios->clock;

		if (host->variant->qcom_datactrl_delay && set_rate &&
		    set_rate < mmc->f_max && set_rate > 24000000)
			set_rate = 24000000;

		if (host->variant->explicit_mclk_control &&
		    set_rate != host->clock_cache) {
			ret = clk_set_rate(host->clk, set_rate);
			if (ret < 0)
				dev_err(mmc_dev(host->mmc),
					"Error setting clock rate (%d)\n", ret);
			else
				host->mclk = clk_get_rate(host->clk);
		}
		host->clock_cache = set_rate;

		/*
		 * Legacy msm_sdcc waits "at least 2 MCLK cycles" after
		 * clk_set_rate before reprogramming MMCICLOCK, to let the
		 * SDC core resync to the new RCG rate. Without this,
		 * MMCICLOCK can be sampled mid-transition and the controller
		 * lands in an indeterminate state — observed as intermittent
		 * DATACRCFAIL on the first transfer after a rate change.
		 * Formula 1 + 3000000/clock matches legacy msmsdcc_delay():
		 * 1 us at 48 MHz, 8 us at 400 kHz.
		 */
		if (host->variant->qcom_datactrl_delay && set_rate)
			udelay(1 + 3000000 / set_rate);
	}

	spin_lock_irqsave(&host->lock, flags);

	if (host->ops && host->ops->set_clkreg)
		host->ops->set_clkreg(host, ios->clock);
	else
		mmci_set_clkreg(host, ios->clock);

	mmci_set_max_busy_timeout(mmc);

	/*
	 * Legacy msm_sdcc waits 50 us between MMCICLOCK and MMCIPOWER writes.
	 * This is during the card-init MCI_PWR_OFF -> MCI_PWR_UP -> MCI_PWR_ON
	 * power-state machine, where the controller needs MMCICLOCK to have
	 * stabilised before the power register changes. mmci_reg_delay() runs
	 * *after* both writes, which doesn't guarantee a stable clock when
	 * MMCIPOWER samples the bus. Add the gap legacy explicitly waits for.
	 */
	if (host->variant->qcom_datactrl_delay)
		udelay(50);

	if (host->ops && host->ops->set_pwrreg)
		host->ops->set_pwrreg(host, pwr);
	else
		mmci_write_pwrreg(host, pwr);

	mmci_reg_delay(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_get_cd(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct mmci_platform_data *plat = host->plat;
	unsigned int status = mmc_gpio_get_cd(mmc);

	if (status == -ENOSYS) {
		if (!plat->status)
			return 1; /* Assume always present */

		status = plat->status(mmc_dev(host->mmc));
	}
	return status;
}

static int mmci_sig_volt_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmci_host *host = mmc_priv(mmc);
	int ret;

	ret = mmc_regulator_set_vqmmc(mmc, ios);

	if (!ret && host->ops && host->ops->post_sig_volt_switch)
		ret = host->ops->post_sig_volt_switch(host, ios);
	else if (ret)
		ret = 0;

	if (ret < 0)
		dev_warn(mmc_dev(mmc), "Voltage switch failed\n");

	return ret;
}

static void mmci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;

	if (enable)
		/* Keep the SDIO mode bit if SDIO irqs are enabled */
		pm_runtime_get_sync(mmc_dev(mmc));

	spin_lock_irqsave(&host->lock, flags);
	mmci_write_sdio_irq_bit(host, enable);
	spin_unlock_irqrestore(&host->lock, flags);

	if (!enable) {
		pm_runtime_put_autosuspend(mmc_dev(mmc));
	}
}

static void mmci_ack_sdio_irq(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	mmci_write_sdio_irq_bit(host, 1);
	spin_unlock_irqrestore(&host->lock, flags);
}

static struct mmc_host_ops mmci_ops = {
	.request	= mmci_request,
	.pre_req	= mmci_pre_request,
	.post_req	= mmci_post_request,
	.set_ios	= mmci_set_ios,
	.get_ro		= mmc_gpio_get_ro,
	.get_cd		= mmci_get_cd,
	.start_signal_voltage_switch = mmci_sig_volt_switch,
};

static void mmci_probe_level_translator(struct mmc_host *mmc)
{
	struct device *dev = mmc_dev(mmc);
	struct mmci_host *host = mmc_priv(mmc);
	struct gpio_desc *cmd_gpio;
	struct gpio_desc *ck_gpio;
	struct gpio_desc *ckin_gpio;
	int clk_hi, clk_lo;

	/*
	 * Assume the level translator is present if st,use-ckin is set.
	 * This is to cater for DTs which do not implement this test.
	 */
	host->clk_reg_add |= MCI_STM32_CLK_SELCKIN;

	cmd_gpio = gpiod_get(dev, "st,cmd", GPIOD_OUT_HIGH);
	if (IS_ERR(cmd_gpio))
		goto exit_cmd;

	ck_gpio = gpiod_get(dev, "st,ck", GPIOD_OUT_HIGH);
	if (IS_ERR(ck_gpio))
		goto exit_ck;

	ckin_gpio = gpiod_get(dev, "st,ckin", GPIOD_IN);
	if (IS_ERR(ckin_gpio))
		goto exit_ckin;

	/* All GPIOs are valid, test whether level translator works */

	/* Sample CKIN */
	clk_hi = !!gpiod_get_value(ckin_gpio);

	/* Set CK low */
	gpiod_set_value(ck_gpio, 0);

	/* Sample CKIN */
	clk_lo = !!gpiod_get_value(ckin_gpio);

	/* Tristate all */
	gpiod_direction_input(cmd_gpio);
	gpiod_direction_input(ck_gpio);

	/* Level translator is present if CK signal is propagated to CKIN */
	if (!clk_hi || clk_lo) {
		host->clk_reg_add &= ~MCI_STM32_CLK_SELCKIN;
		dev_warn(dev,
			 "Level translator inoperable, CK signal not detected on CKIN, disabling.\n");
	}

	gpiod_put(ckin_gpio);

exit_ckin:
	gpiod_put(ck_gpio);
exit_ck:
	gpiod_put(cmd_gpio);
exit_cmd:
	pinctrl_select_default_state(dev);
}

static int mmci_of_parse(struct device_node *np, struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	int ret = mmc_of_parse(mmc);

	if (ret)
		return ret;

	if (of_property_read_bool(np, "st,sig-dir-dat0"))
		host->pwr_reg_add |= MCI_ST_DATA0DIREN;
	if (of_property_read_bool(np, "st,sig-dir-dat2"))
		host->pwr_reg_add |= MCI_ST_DATA2DIREN;
	if (of_property_read_bool(np, "st,sig-dir-dat31"))
		host->pwr_reg_add |= MCI_ST_DATA31DIREN;
	if (of_property_read_bool(np, "st,sig-dir-dat74"))
		host->pwr_reg_add |= MCI_ST_DATA74DIREN;
	if (of_property_read_bool(np, "st,sig-dir-cmd"))
		host->pwr_reg_add |= MCI_ST_CMDDIREN;
	if (of_property_read_bool(np, "st,sig-pin-fbclk"))
		host->pwr_reg_add |= MCI_ST_FBCLKEN;
	if (of_property_read_bool(np, "st,sig-dir"))
		host->pwr_reg_add |= MCI_STM32_DIRPOL;
	if (of_property_read_bool(np, "st,neg-edge"))
		host->clk_reg_add |= MCI_STM32_CLK_NEGEDGE;
	if (of_property_read_bool(np, "st,use-ckin"))
		mmci_probe_level_translator(mmc);

	if (of_property_read_bool(np, "mmc-cap-mmc-highspeed"))
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED;
	if (of_property_read_bool(np, "mmc-cap-sd-highspeed"))
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;

	return 0;
}

static int mmci_probe(struct amba_device *dev,
	const struct amba_id *id)
{
	struct mmci_platform_data *plat = dev->dev.platform_data;
	struct device_node *np = dev->dev.of_node;
	struct variant_data *variant = id->data;
	struct mmci_host *host;
	struct mmc_host *mmc;
	int ret;

	/* Must have platform data or Device Tree. */
	if (!plat && !np) {
		dev_err(&dev->dev, "No plat data or DT found\n");
		return -EINVAL;
	}

	/*
	 * Qualcomm SDCC v4 (MSM8660/APQ8060) needs a different variant from
	 * the generic Qualcomm SDCC because of several MSM8660-only
	 * behaviours (ADM exec_func atomic submit, DATACTRL settle delay,
	 * 2x data timeout, no STARTBITERR IRQ, supports_sdio_irq).  The
	 * amba_id (0x00051180) is shared with all Qualcomm PrimeCell SDCC
	 * revisions, so DT compatible is the only way to disambiguate
	 * without affecting MSM8916/MSM8974/SDM845/...  Sashiko High #44.
	 */
	if (variant == &variant_qcom && np &&
	    of_device_is_compatible(np, "qcom,msm8660-sdcc-v4"))
		variant = &variant_qcom_msm8660;

	if (!plat) {
		plat = devm_kzalloc(&dev->dev, sizeof(*plat), GFP_KERNEL);
		if (!plat)
			return -ENOMEM;
	}

	mmc = devm_mmc_alloc_host(&dev->dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->mmc_ops = &mmci_ops;
	mmc->ops = &mmci_ops;

	ret = mmci_of_parse(np, mmc);
	if (ret)
		return ret;

	/*
	 * Some variant (STM32) doesn't have opendrain bit, nevertheless
	 * pins can be set accordingly using pinctrl
	 */
	if (!variant->opendrain) {
		host->pinctrl = devm_pinctrl_get(&dev->dev);
		if (IS_ERR(host->pinctrl))
			return dev_err_probe(&dev->dev, PTR_ERR(host->pinctrl),
					     "failed to get pinctrl\n");

		host->pins_opendrain = pinctrl_lookup_state(host->pinctrl,
							    MMCI_PINCTRL_STATE_OPENDRAIN);
		if (IS_ERR(host->pins_opendrain))
			return dev_err_probe(&dev->dev, PTR_ERR(host->pins_opendrain),
					     "Can't select opendrain pins\n");
	}

	host->hw_designer = amba_manf(dev);
	host->hw_revision = amba_rev(dev);
	dev_dbg(mmc_dev(mmc), "designer ID = 0x%02x\n", host->hw_designer);
	dev_dbg(mmc_dev(mmc), "revision = 0x%01x\n", host->hw_revision);

	host->clk = devm_clk_get(&dev->dev, NULL);
	if (IS_ERR(host->clk))
		return PTR_ERR(host->clk);

	ret = clk_prepare_enable(host->clk);
	if (ret)
		return ret;

	/*
	 * Interconnect path for SD card memory access (optional).
	 *
	 * The legacy webOS kernel used dfab_sdc_clk clock voters to keep
	 * the Daytona Fabric (DFAB) active during SD card operations.
	 * The interconnect framework provides equivalent functionality.
	 */
	host->icc_path = devm_of_icc_get(&dev->dev, "sdc");
	if (IS_ERR(host->icc_path)) {
		ret = PTR_ERR(host->icc_path);
		if (ret != -ENODATA && ret != -ENOENT) {
			dev_err(&dev->dev, "failed to get interconnect: %d\n", ret);
			goto clk_disable;
		}
		/* No interconnect in DT - optional for backwards compat */
		host->icc_path = NULL;
	}

	if (host->icc_path) {
		/*
		 * Vote for DFAB bandwidth to keep the fabric active.
		 *
		 * Legacy webOS msm_sdcc force-votes dfab_sdc_clk=64MHz on
		 * every active SDCC with pclk_src_dfab=1 (board-tenderloin sets
		 * it on both SDC1/eMMC and SDC4/WiFi; msm_sdcc.c does
		 * clk_set_rate(dfab_pclk, 64000000) + a persistent clk_enable).
		 * On the 64-bit DFAB that is 64e6 * 8 = 512 MB/s, and it is held
		 * SUSTAINED (clk_enable), not a transient ceiling. The mainline
		 * msm8660 ICC provider uses buswidth=8 and rate = bw/8, so
		 * 512000 kBps maps to exactly 64 MHz DFAB.
		 *
		 * IMPORTANT: vote it as avg_bw (sustained), not just peak. The
		 * earlier (0, 512000) vote put 512 MB/s only in the peak slot;
		 * while max(avg,peak) yields 64 MHz in the ACTIVE state, avg=0
		 * lets the floor lapse across RPM active/sleep context changes.
		 * Under sustained concurrent eMMC+WiFi DMA the ADM must drain
		 * the SDCC FIFO to EBI continuously; if DFAB drops to the RPM
		 * minimum the ADM starves mid-transfer and the SDCC latches
		 * DATACRCFAIL/RXOVERRUN (the recurring eMMC-under-WiFi failure).
		 * avg=peak=512000 replicates legacy's persistent 64 MHz hold.
		 */
		/*
		 * Dynamic vote (Qualcomm variant): start at idle, bump in
		 * pre_request, lower again via delayed_work after icc_idle_ms
		 * of inactivity.  Matches legacy webOS per-transaction
		 * msm_bus_scale_client_update_request semantics.  For
		 * non-Qualcomm variants we just hold the historic 512 MB/s
		 * sustained vote.
		 *
		 * Numbers (calibrated against legacy webOS reference, on-device
		 * /proc/interrupts measured 2026-06-14):
		 *   idle  =  64 MBps  -- DFAB keep-alive (legacy minimum hold)
		 *   active = 1024 MBps -- 128 MHz on the 8-byte DFAB, headroom
		 *                         for sustained concurrent eMMC+WiFi
		 *                         DMA without ADM ch2 drain starvation.
		 *   hysteresis = 200 ms -- generous enough that bursty SDIO
		 *                         I/O does not bounce votes; tight
		 *                         enough that DFAB drops back to keep-
		 *                         alive between activity bursts.
		 */
		spin_lock_init(&host->icc_vote_lock);
		INIT_DELAYED_WORK(&host->icc_idle_work, mmci_qcom_icc_idle_work);
		INIT_DELAYED_WORK(&host->adm_sample_work, mmci_adm_sample_work);
		/*
		 * Hold the SDCC Daytona Fabric at a constant 64 MHz, the value
		 * every MSM8660 reference kernel uses: legacy webOS, Samsung and
		 * HTC msm_sdcc.c all do clk_set_rate(dfab_sdc_clk, 64000000) +
		 * clk_enable (persistent, never scaled), and SPS/BAM does the
		 * same. Confirmed live on stock webOS 3.0.5 via novacom:
		 * dfab_clk / dfab_sdc1_clk read 64 MHz at idle, during a
		 * sustained eMMC transfer, and during concurrent eMMC+WiFi DMA --
		 * always, never higher, never lower (only EBI/AFAB scale).
		 *
		 * 512000 kBps = 64 MHz at buswidth 8. Both the idle floor and the
		 * active vote are 64 MHz so the fabric is held constant like the
		 * legacy clk_enable; there is no 128 MHz "burst" tier (no
		 * reference kernel uses one and 64 MHz sustains every workload).
		 * The ADM fills/drains the SDCC FIFO to EBI continuously; if DFAB
		 * drops below 64 MHz it starves mid-transfer (a 1.5 MB write
		 * crawls to the 500 ms ch2 watchdog and wedges the eMMC). The
		 * earlier 64000 kBps (8 MHz) idle floor + dynamic bump regressed
		 * large writes once the interconnect started scaling the bus
		 * clock dynamically.
		 */
		host->icc_idle_bw = 512000;
		host->icc_active_bw = mmci_qcom_emmc_active_bw ?: 512000;
		host->icc_idle_ms = 200;
		host->icc_voted_active = false;

		ret = icc_set_bw(host->icc_path,
				 variant->qcom_dml ? host->icc_idle_bw : 512000,
				 variant->qcom_dml ? host->icc_idle_bw : 512000);
		if (ret) {
			dev_err(&dev->dev, "failed to set interconnect bw: %d\n", ret);
			goto clk_disable;
		}
		dev_dbg(&dev->dev,
			"interconnect bandwidth voting enabled (idle=%u active=%u kBps)\n",
			host->icc_idle_bw, host->icc_active_bw);
	}

	if (variant->qcom_fifo)
		host->get_rx_fifocnt = mmci_qcom_get_rx_fifocnt;
	else
		host->get_rx_fifocnt = mmci_get_rx_fifocnt;

	host->plat = plat;
	host->variant = variant;
	/*
	 * Initialize datactrl_first from variant default, then allow
	 * device tree to override. This enables per-controller tuning
	 * for SDIO vs eMMC which may have different timing requirements.
	 */
	host->datactrl_first = variant->datactrl_first;
	if (np && of_property_read_bool(np, "qcom,datactrl-first"))
		host->datactrl_first = true;

	/*
	 * Qualcomm SDCC dummy CMD52 errata. Opt-in per controller via
	 * "qcom,dummy52-required" so boards can enable it only on SDCC
	 * instances that actually need the workaround (typically the one
	 * wired to an SDIO function device like AR6003 WiFi).
	 */
	if (np && of_property_read_bool(np, "qcom,dummy52-required")) {
		host->dummy52_required = true;
		host->dummy52_cmd.opcode = SD_IO_RW_DIRECT;
		host->dummy52_cmd.arg = 0;
		host->dummy52_cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
		host->dummy52_cmd.data = NULL;
		dev_info(mmc_dev(mmc),
			 "qcom,dummy52-required enabled (CMD52 after CMD53 WRITE)\n");
	}
	host->mclk = clk_get_rate(host->clk);
	/*
	 * According to the spec, mclk is max 100 MHz,
	 * so we try to adjust the clock down to this,
	 * (if possible).
	 */
	if (host->mclk > variant->f_max) {
		ret = clk_set_rate(host->clk, variant->f_max);
		if (ret < 0)
			goto clk_disable;
		host->mclk = clk_get_rate(host->clk);
		dev_dbg(mmc_dev(mmc), "eventual mclk rate: %u Hz\n",
			host->mclk);
	}

	host->phybase = dev->res.start;
	host->base = devm_ioremap_resource(&dev->dev, &dev->res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto clk_disable;
	}

	if (variant->init)
		variant->init(host);

	/*
	 * The ARM and ST versions of the block have slightly different
	 * clock divider equations which means that the minimum divider
	 * differs too.
	 * on Qualcomm like controllers get the nearest minimum clock to 100Khz
	 */
	if (variant->st_clkdiv)
		mmc->f_min = DIV_ROUND_UP(host->mclk, 257);
	else if (variant->stm32_clkdiv)
		mmc->f_min = DIV_ROUND_UP(host->mclk, 2046);
	else if (variant->explicit_mclk_control)
		mmc->f_min = clk_round_rate(host->clk, 100000);
	else
		mmc->f_min = DIV_ROUND_UP(host->mclk, 512);
	/*
	 * If no maximum operating frequency is supplied, fall back to use
	 * the module parameter, which has a (low) default value in case it
	 * is not specified. Either value must not exceed the clock rate into
	 * the block, of course.
	 */
	if (mmc->f_max)
		mmc->f_max = variant->explicit_mclk_control ?
				min(variant->f_max, mmc->f_max) :
				min(host->mclk, mmc->f_max);
	else
		mmc->f_max = variant->explicit_mclk_control ?
				fmax : min(host->mclk, fmax);


	dev_dbg(mmc_dev(mmc), "clocking block at %u Hz\n", mmc->f_max);

	host->rst = devm_reset_control_get_optional_exclusive(&dev->dev, NULL);
	if (IS_ERR(host->rst)) {
		ret = PTR_ERR(host->rst);
		goto clk_disable;
	}
	ret = reset_control_deassert(host->rst);
	if (ret)
		dev_err(mmc_dev(mmc), "failed to de-assert reset\n");

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto clk_disable;

	if (!mmc->ocr_avail)
		mmc->ocr_avail = plat->ocr_mask;
	else if (plat->ocr_mask)
		dev_warn(mmc_dev(mmc), "Platform OCR mask is ignored\n");

	/* We support these capabilities. */
	mmc->caps |= MMC_CAP_CMD23;

	/*
	 * Qualcomm ADM atomic-submit DMA (qcom_dml_atomic_submit +
	 * host->datactrl_first) requires the data command to be available at
	 * DMA-submit time so mmci_qcom_atomic_exec_func can write
	 * DATACTRL + ARG + CMD atomically with the ADM CMD_PTR (closing the
	 * inline-path CRCI arming gap that wedges eMMC reads — see
	 * adm-dma reports/emmc-dma-wedge-rootcause.md).
	 *
	 * CMD23 (SBC) breaks that: __mmci_start_request issues the SBC first
	 * and the real data command (CMD18) only later, from the SBC's
	 * CMDRESPEND IRQ — so the data command is NOT available when the DMA
	 * is submitted, and the atomic stash captures CMD23 instead. Drop
	 * CMD23 on these hosts so every read is single-command (CMD18 + CMD12
	 * stop), matching legacy webOS msm_sdcc, and the atomic path works
	 * uniformly. (CMD23 only applies to eMMC/SD memory cards, never the
	 * SDIO WiFi host, so this is a no-op there.)
	 */
	if (host->variant->qcom_dml_atomic_submit && host->datactrl_first)
		mmc->caps &= ~MMC_CAP_CMD23;

	/*
	 * Attempted MMC_CAP_BUS_WIDTH_TEST on qcom variants — backed out:
	 * the Samsung SEM32G fitted to tenderloin doesn't respond to
	 * CMD19 BUSTEST_R, so enabling the cap caused eMMC re-init to
	 * DATATIMEOUT on every bus-width verification step.  Fix A (SDCC
	 * IP clk_reset on data error) is enough on its own — by the time
	 * the bus-width ladder is reached we've already avoided the
	 * cascade that re-init was triggering.
	 */

	/*
	 * Enable busy detection.
	 */
	if (variant->busy_detect) {
		mmci_ops.card_busy = mmci_card_busy;
		/*
		 * Not all variants have a flag to enable busy detection
		 * in the DPSM, but if they do, set it here.
		 */
		if (variant->busy_dpsm_flag)
			mmci_write_datactrlreg(host,
					       host->variant->busy_dpsm_flag);
		mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;
	}

	if (variant->supports_sdio_irq && host->mmc->caps & MMC_CAP_SDIO_IRQ) {
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;

		mmci_ops.enable_sdio_irq = mmci_enable_sdio_irq;
		mmci_ops.ack_sdio_irq	= mmci_ack_sdio_irq;

		mmci_write_datactrlreg(host,
				       host->variant->datactrl_mask_sdio);
	}

	/* Variants with mandatory busy timeout in HW needs R1B responses. */
	if (variant->busy_timeout)
		mmc->caps |= MMC_CAP_NEED_RSP_BUSY;

	/* Prepare a CMD12 - needed to clear the DPSM on some variants. */
	host->stop_abort.opcode = MMC_STOP_TRANSMISSION;
	host->stop_abort.arg = 0;
	host->stop_abort.flags = MMC_RSP_R1B | MMC_CMD_AC;

	/* We support these PM capabilities. */
	mmc->pm_caps |= MMC_PM_KEEP_POWER;

	/*
	 * We can do SGIO
	 */
	mmc->max_segs = NR_SG;

	/*
	 * Since only a certain number of bits are valid in the data length
	 * register, we must ensure that we don't exceed 2^num-1 bytes in a
	 * single request.
	 */
	mmc->max_req_size = (1 << variant->datalength_bits) - 1;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 1 << variant->datactrl_blocksz;

	/*
	 * Limit the number of blocks transferred so that we don't overflow
	 * the maximum request size.
	 */
	mmc->max_blk_count = mmc->max_req_size >> variant->datactrl_blocksz;

	/*
	 * Optional DT-driven cap on per-request size, used to mitigate
	 * AHB-fabric contention between two ADM channels on shared bus.
	 *
	 * On APQ8060/MSM8660 (Tenderloin) adm_dma1 is shared between sdcc1
	 * (eMMC) and sdcc4 (WiFi/AR6003). When both channels burst on the
	 * fabric simultaneously, the AHB bridge arbiter saturates and the
	 * non-bursting controller's CPU register accesses stall; sdcc1
	 * sees multi-hundred-block DATACRCFAIL and the rootfs goes RO.
	 *
	 * The mitigation (per Gemini diagnostic): clamp the per-request size
	 * on the bulk-storage controller so the ADM periodically relinquishes
	 * the fabric, letting the other controller's command path break
	 * through. 16 KB is the suggested starting value -- small enough to
	 * give the fabric breathing room every ~340 us at peak SDCC clock,
	 * still large enough for sustained throughput.
	 *
	 * Property is opt-in: absent or zero leaves the defaults alone.
	 */
	{
		u32 max_req_kb = 0;
		struct device_node *np = mmc_dev(mmc)->of_node;

		if (np && !of_property_read_u32(np, "qcom,max-req-kb",
						&max_req_kb) &&
		    max_req_kb > 0) {
			u32 cap = max_req_kb * 1024;

			if (cap < mmc->max_req_size) {
				mmc->max_req_size = cap;
				mmc->max_seg_size = cap;
				/*
				 * Do NOT clamp max_blk_count here -- the MMC
				 * core enforces min(max_req_size,
				 * blksize * max_blk_count). Capping
				 * max_blk_count using datactrl_blocksz
				 * (max block size = 2048) makes the cap
				 * pessimistic at the typical 512 B block
				 * size used for eMMC.
				 */
				dev_info(mmc_dev(mmc),
					 "qcom,max-req-kb=%u -> max_req=%u (max_blk_count=%u unchanged)\n",
					 max_req_kb, mmc->max_req_size,
					 mmc->max_blk_count);
			}
		}
	}

	/*
	 * DEBUG: cmdline-tunable per-request size cap for the qcom eMMC, to
	 * sweep the transfer-size threshold where the ADM ch2 drain breaks.
	 * armmmci.qcom_emmc_max_bytes=N; 0 = no cap.
	 */
	if (mmci_qcom_emmc_max_bytes && host->variant->qcom_datactrl_delay &&
	    mmci_qcom_emmc_max_bytes < mmc->max_req_size) {
		mmc->max_req_size = mmci_qcom_emmc_max_bytes;
		mmc->max_seg_size = mmci_qcom_emmc_max_bytes;
		dev_info(mmc_dev(mmc),
			 "DEBUG qcom_emmc_max_bytes=%u -> max_req=%u max_seg=%u\n",
			 mmci_qcom_emmc_max_bytes, mmc->max_req_size,
			 mmc->max_seg_size);
	}

	spin_lock_init(&host->lock);

	writel(0, host->base + MMCIMASK0);

	if (variant->mmcimask1)
		writel(0, host->base + MMCIMASK1);

	writel(0xfff, host->base + MMCICLEAR);

	/*
	 * If:
	 * - not using DT but using a descriptor table, or
	 * - using a table of descriptors ALONGSIDE DT, or
	 * look up these descriptors named "cd" and "wp" right here, fail
	 * silently of these do not exist
	 */
	if (!np) {
		ret = mmc_gpiod_request_cd(mmc, "cd", 0, false, 0);
		if (ret == -EPROBE_DEFER)
			goto clk_disable;

		ret = mmc_gpiod_request_ro(mmc, "wp", 0, 0);
		if (ret == -EPROBE_DEFER)
			goto clk_disable;
	}

	ret = devm_request_threaded_irq(&dev->dev, dev->irq[0], mmci_irq,
					mmci_irq_thread, IRQF_SHARED,
					DRIVER_NAME " (cmd)", host);
	if (ret)
		goto clk_disable;

	if (!dev->irq[1])
		host->singleirq = true;
	else {
		ret = devm_request_irq(&dev->dev, dev->irq[1], mmci_pio_irq,
				IRQF_SHARED, DRIVER_NAME " (pio)", host);
		if (ret)
			goto clk_disable;
	}

	if (host->variant->busy_detect)
		INIT_DELAYED_WORK(&host->ux500_busy_timeout_work,
				  ux500_busy_timeout_work);

	if (host->variant->qcom_dml)
		INIT_DELAYED_WORK(&host->qcom_dma_timeout_work,
				  qcom_dma_data_timeout_work);

	if (host->dummy52_required)
		INIT_DELAYED_WORK(&host->qcom_dummy52_watchdog,
				  qcom_dummy52_watchdog_work);

	writel(MCI_IRQENABLE | variant->start_err, host->base + MMCIMASK0);

	amba_set_drvdata(dev, mmc);

	dev_info(&dev->dev, "%s: PL%03x manf %x rev%u at 0x%08llx irq %d,%d (pio)\n",
		 mmc_hostname(mmc), amba_part(dev), amba_manf(dev),
		 amba_rev(dev), (unsigned long long)dev->res.start,
		 dev->irq[0], dev->irq[1]);

	ret = mmci_dma_setup(host);
	if (ret == -EPROBE_DEFER)
		goto clk_disable;

	/*
	 * Per-variant override: leave runtime PM in "always-active" state
	 * by NOT calling pm_runtime_use_autosuspend + not dropping the
	 * pm_runtime_get_sync reference that probe took. The host then
	 * never autosuspends, its clocks stay enabled, and its MMIO is
	 * always reachable — needed for the ADM watchdog dump_state
	 * callback to read SDCC registers from atomic context without
	 * risking a bus hang on a clock-gated controller.
	 */
	if (!variant->no_pm_autosuspend) {
		pm_runtime_set_autosuspend_delay(&dev->dev, 50);
		pm_runtime_use_autosuspend(&dev->dev);
	}

	ret = mmc_add_host(mmc);
	if (ret)
		goto clk_disable;

	if (!variant->no_pm_autosuspend)
		pm_runtime_put(&dev->dev);
	return 0;

 clk_disable:
	clk_disable_unprepare(host->clk);
	return ret;
}

static void mmci_remove(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);
		struct variant_data *variant = host->variant;

		/*
		 * Undo pm_runtime_put() in probe.  We use the _sync
		 * version here so that we can access the primecell.
		 */
		pm_runtime_get_sync(&dev->dev);

		mmc_remove_host(mmc);

		/*
		 * Order of operations matters here (Sashiko High #5).  We
		 * must mask all hardware IRQ sources BEFORE
		 * cancel_delayed_work_sync() so that no remaining
		 * mmci_data_irq / mmci_qcom_dma_complete can re-arm
		 * qcom_dma_timeout_work between the cancel_sync and the
		 * IRQ teardown.  Cancelling first would leave a window
		 * where the still-armed hardware fires, the IRQ handler
		 * re-schedules the work, and the work executes after
		 * mmci_dma_release()/clk_disable have torn down the host.
		 */
		writel(0, host->base + MMCIMASK0);

		if (variant->mmcimask1)
			writel(0, host->base + MMCIMASK1);

		writel(0, host->base + MMCICOMMAND);
		writel(0, host->base + MMCIDATACTRL);

		if (variant->qcom_dml)
			cancel_delayed_work_sync(&host->qcom_dma_timeout_work);

		if (host->dummy52_required)
			cancel_delayed_work_sync(&host->qcom_dummy52_watchdog);

		if (host->icc_path && host->variant->qcom_dml) {
			cancel_delayed_work_sync(&host->icc_idle_work);
			cancel_delayed_work_sync(&host->adm_sample_work);
		}

		mmci_dma_release(host);
		clk_disable_unprepare(host->clk);
	}
}

static void mmci_save(struct mmci_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	writel(0, host->base + MMCIMASK0);
	if (host->variant->pwrreg_nopower) {
		writel(0, host->base + MMCIDATACTRL);
		writel(0, host->base + MMCIPOWER);
		writel(0, host->base + MMCICLOCK);
	}
	mmci_reg_delay(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void mmci_restore(struct mmci_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (host->variant->pwrreg_nopower) {
		writel(host->clk_reg, host->base + MMCICLOCK);
		writel(host->datactrl_reg, host->base + MMCIDATACTRL);
		writel(host->pwr_reg, host->base + MMCIPOWER);
	}
	writel(MCI_IRQENABLE | host->variant->start_err,
	       host->base + MMCIMASK0);
	mmci_reg_delay(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_runtime_suspend(struct device *dev)
{
	struct amba_device *adev = to_amba_device(dev);
	struct mmc_host *mmc = amba_get_drvdata(adev);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);
		pinctrl_pm_select_sleep_state(dev);
		mmci_save(host);
		clk_disable_unprepare(host->clk);
	}

	return 0;
}

static int mmci_runtime_resume(struct device *dev)
{
	struct amba_device *adev = to_amba_device(dev);
	struct mmc_host *mmc = amba_get_drvdata(adev);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);
		clk_prepare_enable(host->clk);
		mmci_restore(host);
		pinctrl_select_default_state(dev);
	}

	return 0;
}

static const struct dev_pm_ops mmci_dev_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(mmci_runtime_suspend, mmci_runtime_resume, NULL)
};

static const struct amba_id mmci_ids[] = {
	{
		.id	= 0x00041180,
		.mask	= 0xff0fffff,
		.data	= &variant_arm,
	},
	{
		.id	= 0x01041180,
		.mask	= 0xff0fffff,
		.data	= &variant_arm_extended_fifo,
	},
	{
		.id	= 0x02041180,
		.mask	= 0xff0fffff,
		.data	= &variant_arm_extended_fifo_hwfc,
	},
	{
		.id	= 0x00041181,
		.mask	= 0x000fffff,
		.data	= &variant_arm,
	},
	/* ST Micro variants */
	{
		.id     = 0x00180180,
		.mask   = 0x00ffffff,
		.data	= &variant_u300,
	},
	{
		.id     = 0x10180180,
		.mask   = 0xf0ffffff,
		.data	= &variant_nomadik,
	},
	{
		.id     = 0x00280180,
		.mask   = 0x00ffffff,
		.data	= &variant_nomadik,
	},
	{
		.id     = 0x00480180,
		.mask   = 0xf0ffffff,
		.data	= &variant_ux500,
	},
	{
		.id     = 0x10480180,
		.mask   = 0xf0ffffff,
		.data	= &variant_ux500v2,
	},
	{
		.id     = 0x00880180,
		.mask   = 0x00ffffff,
		.data	= &variant_stm32,
	},
	{
		.id     = 0x10153180,
		.mask	= 0xf0ffffff,
		.data	= &variant_stm32_sdmmc,
	},
	{
		.id     = 0x00253180,
		.mask	= 0xf0ffffff,
		.data	= &variant_stm32_sdmmcv2,
	},
	{
		.id     = 0x20253180,
		.mask	= 0xf0ffffff,
		.data	= &variant_stm32_sdmmcv2,
	},
	{
		.id     = 0x00353180,
		.mask	= 0xf0ffffff,
		.data	= &variant_stm32_sdmmcv3,
	},
	/* Qualcomm variants */
	{
		.id     = 0x00051180,
		.mask	= 0x000fffff,
		.data	= &variant_qcom,
	},
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(amba, mmci_ids);

static struct amba_driver mmci_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
		.pm	= pm_ptr(&mmci_dev_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= mmci_probe,
	.remove		= mmci_remove,
	.id_table	= mmci_ids,
};

module_amba_driver(mmci_driver);

module_param(fmax, uint, 0444);
module_param_named(mmc1_wr_dma_min, mmci_mmc1_wr_dma_min, uint, 0644);
MODULE_PARM_DESC(mmc1_wr_dma_min,
		 "Min mmc1 SDIO WR length routed through DMA on qcom variant (default 256, set 64 for legacy-equivalent)");
module_param_named(qcom_emmc_active_bw, mmci_qcom_emmc_active_bw, uint, 0644);
MODULE_PARM_DESC(qcom_emmc_active_bw,
		 "Active-transfer ICC bw vote for qcom eMMC so dynamic fabric scaling ramps DFAB during transfers (default 1600000 ~200MHz DFAB; idle stays 512000 ~64MHz; 0 = no ramp)");
module_param_named(qcom_emmc_max_bytes, mmci_qcom_emmc_max_bytes, uint, 0644);
MODULE_PARM_DESC(qcom_emmc_max_bytes,
		 "DEBUG: cap qcom eMMC per-request size to sweep the ADM-drain breaking threshold (e.g. 512/1024/2048/4096; 0 = no cap)");
module_param(adm_sample, bool, 0644);
MODULE_PARM_DESC(adm_sample,
		 "Debug: log qcom ADM SDCC FIFO/DATACNT cadence during large transfers (default on)");
module_param(qcom_pwrsave, bool, 0644);
MODULE_PARM_DESC(qcom_pwrsave,
		 "Enable MCI_CLK_PWRSAVE clock-gating on the qcom SDCC (default on = legacy; boot =0 to A/B test write throttling)");

MODULE_DESCRIPTION("ARM PrimeCell PL180/181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
