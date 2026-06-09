// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for msm7k serial device and console
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Robert Love <rlove@google.com>
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/dma/qcom_adm.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/moduleparam.h>

/*
 * Debug: trace BT (GSBI6, mapbase 0x16540000) UART TX bursts to detect
 * inter-byte/wire gaps without a scope (Gemini 2026-05-25). Logs per burst:
 * requested vs written bytes, feed time, on-wire floor, gap since previous
 * burst. A SYNC_RSP fed in one burst (wrote==req) drains contiguously = no
 * gap; a split burst (TX_READY dropped) = FIFO refill = potential wire gap.
 * Enable: echo 1 > /sys/module/msm_serial/parameters/bt_tx_trace
 */
static bool bt_tx_trace;
module_param(bt_tx_trace, bool, 0644);
MODULE_PARM_DESC(bt_tx_trace, "Trace GSBI6 BT UART TX bursts for inter-byte gap detection");

/*
 * TouchPad BT TX diagnostics (no oscilloscope required). See
 * reports/bt-trace/bt-bringup-summary-2026-05-27.md. These probe the
 * analog/timing suspects for the "chip only-SYNCs, ignores our TX" blocker:
 *
 *  - bt_diag: dump the TLMM pad config (drive strength / pull / output-enable /
 *    idle line level) for the BT UART pads gpio53-56 at port open, to diff the
 *    *programmed* pad state against the known-good webOS setup (8 mA, no pull).
 *  - bt_tx_pio_bursts / bt_tx_pio_underruns (read-only): count PIO TX bursts and
 *    how many drained the TX FIFO mid-frame -> inter-byte WIRE GAPs at the pad
 *    that do not show in a "bytes queued" trace.
 *  - bt_diag_loopback (write 1): internal-loopback self-test (UARTDM MR2 bit 7),
 *    transmits a BCSP SYNC-RSP payload and reads it back through the controller,
 *    isolating "digital framing wrong" from "analog/pad wrong".
 */
static bool bt_diag;
module_param(bt_diag, bool, 0644);
MODULE_PARM_DESC(bt_diag, "BT UART: dump TLMM pad config (gpio53-56) at port open");

static unsigned int bt_tx_pio_bursts;
module_param(bt_tx_pio_bursts, uint, 0444);
MODULE_PARM_DESC(bt_tx_pio_bursts, "BT UART: PIO TX burst count (read-only diag)");

static unsigned int bt_tx_pio_underruns;
module_param(bt_tx_pio_underruns, uint, 0444);
MODULE_PARM_DESC(bt_tx_pio_underruns,
	"BT UART: PIO TX bursts where TX_READY dropped mid-frame = potential wire gap (read-only diag)");

/*
 * bt_force_8e1: force even parity (8E1) on the BT UART, to test the
 * "BCSP uses even parity" claim on-device. See msm_set_termios. Default off.
 */
static bool bt_force_8e1;
module_param(bt_force_8e1, bool, 0644);
MODULE_PARM_DESC(bt_force_8e1, "BT UART: force even parity 8E1 (diag; default 8N1)");

/*
 * bt_tx_bytegap_us: forced inter-byte gap experiment. When >0, the BT UART
 * PIO TX sends one byte at a time, waits for the transmitter to fully drain
 * (SR TX_EMPTY), then idles the line for this many microseconds before the
 * next byte. This intentionally starves the UARTDM to put real idle gaps
 * BETWEEN bytes on gpio53 -- the one thing a single gapless FIFO burst cannot
 * produce. If the CSR chip suddenly answers, it relies on inter-byte timing.
 * INVASIVE / slow (busy-waits with IRQs off); BT port only; default 0 = off.
 */
static unsigned int bt_tx_bytegap_us;
module_param(bt_tx_bytegap_us, uint, 0644);
MODULE_PARM_DESC(bt_tx_bytegap_us,
	"BT UART: force N us idle gap between TX bytes on the wire (diag; 0=off)");

/*
 * BT UART IPR / IMR overrides. Live rdmem capture (2026-06-08) found two
 * persistent register-level deltas between mainline link-est and webOS
 * operational state:
 *
 *   IPR: mainline 0x06, webOS 0x200 (rxstale-timeout ~20x shorter on us)
 *   IMR: mainline 0x1aa7, webOS 0x203 (mainline enables bits 1/2/7/11/12
 *        not present in mainline's IMR bit map — possible QC-specific bits)
 *
 * These knobs let us A/B-test the deltas at runtime without rebuilding:
 *
 *   echo 0x200 > /sys/module/msm_serial/parameters/bt_ipr_override
 *   echo 0x203 > /sys/module/msm_serial/parameters/bt_imr_override   # SEE WARNING
 *
 * Zero = don't override (use the driver-computed value).  Override applies
 * only to the BT UART port (port->mapbase == MSM_BT_UART_MAPBASE).  IPR is
 * re-applied when the param is written; IMR override takes effect on the
 * next IRQ-handler IMR restore (which happens many times a second under
 * any traffic, so updates are near-immediate).
 *
 * WARNING — bt_imr_override is a foot-gun.  Mainline's computed IMR
 * (0x1aa7) enables five bits (2, 5, 7, 11, 12) that mainline's IMR bit
 * map doesn't define publicly but that the driver's IRQ-handler dispatch
 * + DMA-RX completion plumbing clearly depend on for forward progress.
 * Forcing IMR to the webOS value 0x203 (only bits 0, 1, 9) masks those
 * private bits; on a live BT port that hangs the kernel — first the BCSP
 * RX path stops getting completion events, then CPU0 has no work to wake
 * for and stalls in cpuidle while CPU1 spins waiting for an event that
 * will never come.  Validated 2026-06-08: a ~3 minute soft-lockup
 * followed every `echo 0x203 > bt_imr_override` test, requiring a power
 * cycle.  Use this knob only with values that are a SUPERSET of the
 * driver-computed IMR (e.g. 0x1aa7 | extra_bits), never a subset.
 * IPR override is safe at any value — it tunes the RX stale timer only
 * and doesn't affect IRQ dispatch.
 */
static unsigned int bt_ipr_override;
static unsigned int bt_imr_override;
static int bt_ipr_override_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops bt_ipr_override_ops = {
	.set = bt_ipr_override_set,
	.get = param_get_uint,
};
module_param_cb(bt_ipr_override, &bt_ipr_override_ops, &bt_ipr_override, 0644);
MODULE_PARM_DESC(bt_ipr_override,
	"BT UART: override IPR (rxstale timeout) — 0=use computed (default). Safe at any value.");
module_param(bt_imr_override, uint, 0644);
MODULE_PARM_DESC(bt_imr_override,
	"BT UART: override IMR (interrupt mask) — 0=use computed (default). DANGEROUS: value must be a SUPERSET of mainline's 0x1aa7, else soft-lockup. Use 0 unless you really know what you're doing.");
#include <linux/wait.h>

#define MSM_UART_MR1			0x0000

#define MSM_UART_MR1_AUTO_RFR_LEVEL0	0x3F
#define MSM_UART_MR1_AUTO_RFR_LEVEL1	0x3FF00
#define MSM_UART_DM_MR1_AUTO_RFR_LEVEL1	0xFFFFFF00
#define MSM_UART_MR1_RX_RDY_CTL		BIT(7)
#define MSM_UART_MR1_CTS_CTL		BIT(6)

#define MSM_UART_MR2			0x0004
#define MSM_UART_MR2_ERROR_MODE		BIT(6)
#define MSM_UART_MR2_BITS_PER_CHAR	0x30
#define MSM_UART_MR2_BITS_PER_CHAR_5	(0x0 << 4)
#define MSM_UART_MR2_BITS_PER_CHAR_6	(0x1 << 4)
#define MSM_UART_MR2_BITS_PER_CHAR_7	(0x2 << 4)
#define MSM_UART_MR2_BITS_PER_CHAR_8	(0x3 << 4)
#define MSM_UART_MR2_STOP_BIT_LEN_ONE	(0x1 << 2)
#define MSM_UART_MR2_STOP_BIT_LEN_TWO	(0x3 << 2)
#define MSM_UART_MR2_PARITY_MODE_NONE	0x0
#define MSM_UART_MR2_PARITY_MODE_ODD	0x1
#define MSM_UART_MR2_PARITY_MODE_EVEN	0x2
#define MSM_UART_MR2_PARITY_MODE_SPACE	0x3
#define MSM_UART_MR2_PARITY_MODE	0x3

#define MSM_UART_CSR			0x0008

#define MSM_UART_TF			0x000C
#define UARTDM_TF			0x0070

#define MSM_UART_CR				0x0010
#define MSM_UART_CR_CMD_NULL			(0 << 4)
#define MSM_UART_CR_CMD_RESET_RX		(1 << 4)
#define MSM_UART_CR_CMD_RESET_TX		(2 << 4)
#define MSM_UART_CR_CMD_RESET_ERR		(3 << 4)
#define MSM_UART_CR_CMD_RESET_BREAK_INT		(4 << 4)
#define MSM_UART_CR_CMD_START_BREAK		(5 << 4)
#define MSM_UART_CR_CMD_STOP_BREAK		(6 << 4)
#define MSM_UART_CR_CMD_RESET_CTS		(7 << 4)
#define MSM_UART_CR_CMD_RESET_STALE_INT		(8 << 4)
#define MSM_UART_CR_CMD_PACKET_MODE		(9 << 4)
#define MSM_UART_CR_CMD_MODE_RESET		(12 << 4)
#define MSM_UART_CR_CMD_SET_RFR			(13 << 4)
#define MSM_UART_CR_CMD_RESET_RFR		(14 << 4)
#define MSM_UART_CR_CMD_PROTECTION_EN		(16 << 4)
#define MSM_UART_CR_CMD_STALE_EVENT_DISABLE	(6 << 8)
#define MSM_UART_CR_CMD_STALE_EVENT_ENABLE	(80 << 4)
#define MSM_UART_CR_CMD_FORCE_STALE		(4 << 8)
#define MSM_UART_CR_CMD_RESET_TX_READY		(3 << 8)
#define MSM_UART_CR_TX_DISABLE			BIT(3)
#define MSM_UART_CR_TX_ENABLE			BIT(2)
#define MSM_UART_CR_RX_DISABLE			BIT(1)
#define MSM_UART_CR_RX_ENABLE			BIT(0)
#define MSM_UART_CR_CMD_RESET_RXBREAK_START	((1 << 11) | (2 << 4))

#define MSM_UART_IMR			0x0014
#define MSM_UART_IMR_TXLEV		BIT(0)
#define MSM_UART_IMR_RXSTALE		BIT(3)
#define MSM_UART_IMR_RXLEV		BIT(4)
#define MSM_UART_IMR_DELTA_CTS		BIT(5)
#define MSM_UART_IMR_CURRENT_CTS	BIT(6)
#define MSM_UART_IMR_RXBREAK_START	BIT(10)

#define MSM_UART_IPR_RXSTALE_LAST		0x20
#define MSM_UART_IPR_STALE_LSB			0x1F
#define MSM_UART_IPR_STALE_TIMEOUT_MSB		0x3FF80
#define MSM_UART_DM_IPR_STALE_TIMEOUT_MSB	0xFFFFFF80

#define MSM_UART_IPR			0x0018
#define MSM_UART_TFWR			0x001C
#define MSM_UART_RFWR			0x0020
#define MSM_UART_HCR			0x0024

#define MSM_UART_MREG			0x0028
#define MSM_UART_NREG			0x002C
#define MSM_UART_DREG			0x0030
#define MSM_UART_MNDREG			0x0034
#define MSM_UART_IRDA			0x0038
#define MSM_UART_MISR_MODE		0x0040
#define MSM_UART_MISR_RESET		0x0044
#define MSM_UART_MISR_EXPORT		0x0048
#define MSM_UART_MISR_VAL		0x004C
#define MSM_UART_TEST_CTRL		0x0050

#define MSM_UART_SR			0x0008
#define MSM_UART_SR_HUNT_CHAR		BIT(7)
#define MSM_UART_SR_RX_BREAK		BIT(6)
#define MSM_UART_SR_PAR_FRAME_ERR	BIT(5)
#define MSM_UART_SR_OVERRUN		BIT(4)
#define MSM_UART_SR_TX_EMPTY		BIT(3)
#define MSM_UART_SR_TX_READY		BIT(2)
#define MSM_UART_SR_RX_FULL		BIT(1)
#define MSM_UART_SR_RX_READY		BIT(0)

#define MSM_UART_RF			0x000C
#define UARTDM_RF			0x0070
#define MSM_UART_MISR			0x0010
#define MSM_UART_ISR			0x0014
#define MSM_UART_ISR_TX_READY		BIT(7)

#define UARTDM_RXFS			0x50
#define UARTDM_RXFS_BUF_SHIFT		0x7
#define UARTDM_RXFS_BUF_MASK		0x7

#define UARTDM_DMEN			0x3C
#define UARTDM_DMEN_RX_SC_ENABLE	BIT(5)
#define UARTDM_DMEN_TX_SC_ENABLE	BIT(4)

#define UARTDM_DMEN_TX_BAM_ENABLE	BIT(2)	/* UARTDM_1P4 */
#define UARTDM_DMEN_TX_DM_ENABLE	BIT(0)	/* < UARTDM_1P4 */

#define UARTDM_DMEN_RX_BAM_ENABLE	BIT(3)	/* UARTDM_1P4 */
#define UARTDM_DMEN_RX_DM_ENABLE	BIT(1)	/* < UARTDM_1P4 */

#define UARTDM_DMRX			0x34
#define UARTDM_NCF_TX			0x40
#define UARTDM_RX_TOTAL_SNAP		0x38

#define UARTDM_BURST_SIZE		16   /* in bytes */
#define UARTDM_TX_AIGN(x)		((x) & ~0x3) /* valid for > 1p3 */
#define UARTDM_TX_MAX			256   /* in bytes, valid for <= 1p3 */
#define UARTDM_RX_SIZE			(UART_XMIT_SIZE / 4)

enum {
	UARTDM_1P1 = 1,
	UARTDM_1P2,
	UARTDM_1P3,
	UARTDM_1P4,
};

struct msm_dma {
	struct dma_chan		*chan;
	enum dma_data_direction dir;
	union {
		struct {
			dma_addr_t		phys;
			unsigned char		*virt;
			unsigned int		count;
		} rx;
		struct scatterlist tx_sg;
	};
	dma_cookie_t		cookie;
	u32			enable_bit;
	struct dma_async_tx_descriptor	*desc;
};

struct msm_port {
	struct uart_port	uart;
	char			name[16];
	struct clk		*clk;
	struct clk		*pclk;
	unsigned int		imr;
	int			is_uartdm;
	unsigned int		old_snap_state;
	bool			break_detected;
	struct msm_dma		tx_dma;
	struct msm_dma		rx_dma;
	/*
	 * Optional startup pin-mux glitch (webOS btuart_pin_mux dance): briefly
	 * flip the UART pins to a GPIO state and back at port-open, to wake a
	 * power-gated peer UART RX (TouchPad CSR BlueCore BT). Gated on the DT
	 * bool "qcom,startup-mux-glitch"; needs a "gpio" pinctrl state.
	 */
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_default;
	struct pinctrl_state	*pinctrl_gpio;
	/*
	 * RX-safe wake glitch: flips ONLY TX(gpio53)+RFR/RTS(gpio56) — the lines
	 * we drive toward the chip — leaving RX/CTS in UART mode so an inbound
	 * frame is never clipped. Preferred over pinctrl_gpio for the BT wake.
	 */
	struct pinctrl_state	*pinctrl_gpio_txrts;
	bool			startup_mux_glitch;
	/*
	 * TouchPad BT (H2): re-arm the wake glitch during BCSP link
	 * establishment. bt_linkest is true from port-open until we leave the
	 * 115200 link-est baud (see msm_set_termios) or shut down.
	 */
	struct delayed_work	bt_wake_work;
	bool			bt_is_bt_uart;
	bool			bt_linkest;
};

static inline struct msm_port *to_msm_port(struct uart_port *up)
{
	return container_of(up, struct msm_port, uart);
}

static
void msm_write(struct uart_port *port, unsigned int val, unsigned int off)
{
	writel_relaxed(val, port->membase + off);
}

static
unsigned int msm_read(struct uart_port *port, unsigned int off)
{
	return readl_relaxed(port->membase + off);
}

/*
 * Setup the MND registers to use the TCXO clock.
 */
static void msm_serial_set_mnd_regs_tcxo(struct uart_port *port)
{
	msm_write(port, 0x06, MSM_UART_MREG);
	msm_write(port, 0xF1, MSM_UART_NREG);
	msm_write(port, 0x0F, MSM_UART_DREG);
	msm_write(port, 0x1A, MSM_UART_MNDREG);
	port->uartclk = 1843200;
}

/*
 * Setup the MND registers to use the TCXO clock divided by 4.
 */
static void msm_serial_set_mnd_regs_tcxoby4(struct uart_port *port)
{
	msm_write(port, 0x18, MSM_UART_MREG);
	msm_write(port, 0xF6, MSM_UART_NREG);
	msm_write(port, 0x0F, MSM_UART_DREG);
	msm_write(port, 0x0A, MSM_UART_MNDREG);
	port->uartclk = 1843200;
}

static void msm_serial_set_mnd_regs(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);

	/*
	 * These registers don't exist so we change the clk input rate
	 * on uartdm hardware instead
	 */
	if (msm_port->is_uartdm)
		return;

	if (port->uartclk == 19200000)
		msm_serial_set_mnd_regs_tcxo(port);
	else if (port->uartclk == 4800000)
		msm_serial_set_mnd_regs_tcxoby4(port);
}

static void msm_handle_tx(struct uart_port *port);
void msm_serial_bt_wake_glitch(void);	/* exported; called by hci_bcsp */
void msm_serial_bt_dance(void);		/* exported; called by hci_bcsp at probe */
void msm_serial_bt_force_rfr(bool assert_low);	/* exported; called by hci_bcsp */
void msm_serial_bt_send_break(unsigned int us);	/* exported; called by hci_bcsp */
static inline unsigned int msm_apply_bt_imr_override(struct msm_port *msm_port,
						     unsigned int imr);
static inline void msm_write_imr(struct uart_port *port, unsigned int imr);
static void msm_start_rx_dma(struct msm_port *msm_port);

static void msm_stop_dma(struct uart_port *port, struct msm_dma *dma)
{
	struct device *dev = port->dev;
	unsigned int mapped;
	u32 val;

	if (dma->dir == DMA_TO_DEVICE) {
		mapped = sg_dma_len(&dma->tx_sg);
	} else {
		mapped = dma->rx.count;
		dma->rx.count = 0;
	}

	dmaengine_terminate_all(dma->chan);

	/*
	 * DMA Stall happens if enqueue and flush command happens concurrently.
	 * For example before changing the baud rate/protocol configuration and
	 * sending flush command to ADM, disable the channel of UARTDM.
	 * Note: should not reset the receiver here immediately as it is not
	 * suggested to do disable/reset or reset/disable at the same time.
	 */
	val = msm_read(port, UARTDM_DMEN);
	val &= ~dma->enable_bit;
	msm_write(port, val, UARTDM_DMEN);

	/*
	 * RX uses a coherent buffer allocated once in msm_request_rx_dma(), so
	 * there is nothing to unmap here -- only the streaming TX scatterlist.
	 */
	if (mapped && dma->dir == DMA_TO_DEVICE) {
		dma_unmap_sg(dev, &dma->tx_sg, 1, dma->dir);
		sg_init_table(&dma->tx_sg, 1);
	}
}

static void msm_release_dma(struct msm_port *msm_port)
{
	struct msm_dma *dma;

	dma = &msm_port->tx_dma;
	if (dma->chan) {
		msm_stop_dma(&msm_port->uart, dma);
		dma_release_channel(dma->chan);
	}

	memset(dma, 0, sizeof(*dma));

	dma = &msm_port->rx_dma;
	if (dma->chan) {
		msm_stop_dma(&msm_port->uart, dma);
		dma_release_channel(dma->chan);
		dma_free_coherent(msm_port->uart.dev, UARTDM_RX_SIZE,
				  dma->rx.virt, dma->rx.phys);
	}

	memset(dma, 0, sizeof(*dma));
}

static void msm_request_tx_dma(struct msm_port *msm_port, resource_size_t base)
{
	struct device *dev = msm_port->uart.dev;
	struct dma_slave_config conf;
	struct qcom_adm_peripheral_config periph_conf = {};
	struct msm_dma *dma;
	u32 crci = 0;
	int ret;

	dma = &msm_port->tx_dma;

	/* allocate DMA resources, if available */
	dma->chan = dma_request_chan(dev, "tx");
	if (IS_ERR(dma->chan)) {
		dev_err(dev, "Failed to get TX DMA channel: %ld\n", PTR_ERR(dma->chan));
		goto no_tx;
	}

	of_property_read_u32(dev->of_node, "qcom,tx-crci", &crci);

	memset(&conf, 0, sizeof(conf));
	conf.direction = DMA_MEM_TO_DEV;
	conf.device_fc = true;
	conf.dst_addr = base + UARTDM_TF;
	conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	conf.dst_maxburst = UARTDM_BURST_SIZE;
	if (crci) {
		conf.peripheral_config = &periph_conf;
		conf.peripheral_size = sizeof(periph_conf);
		periph_conf.crci = crci;
	}

	ret = dmaengine_slave_config(dma->chan, &conf);
	if (ret)
		goto rel_tx;

	dev_info(dev, "TX DMA channel acquired (CRCI %u)\n", crci);
	dma->dir = DMA_TO_DEVICE;

	if (msm_port->is_uartdm < UARTDM_1P4)
		dma->enable_bit = UARTDM_DMEN_TX_DM_ENABLE;
	else
		dma->enable_bit = UARTDM_DMEN_TX_BAM_ENABLE;

	return;

rel_tx:
	dma_release_channel(dma->chan);
no_tx:
	memset(dma, 0, sizeof(*dma));
}

static void msm_request_rx_dma(struct msm_port *msm_port, resource_size_t base)
{
	struct device *dev = msm_port->uart.dev;
	struct dma_slave_config conf;
	struct qcom_adm_peripheral_config periph_conf = {};
	struct msm_dma *dma;
	u32 crci = 0;
	int ret;

	dma = &msm_port->rx_dma;

	/* allocate DMA resources, if available */
	dma->chan = dma_request_chan(dev, "rx");
	if (IS_ERR(dma->chan)) {
		dev_err(dev, "Failed to get RX DMA channel: %ld\n", PTR_ERR(dma->chan));
		goto no_rx;
	}

	of_property_read_u32(dev->of_node, "qcom,rx-crci", &crci);

	/*
	 * Use a coherent (uncached) RX buffer, like the legacy webOS hsuart
	 * driver. With a streaming kzalloc buffer + dma_map/unmap_single, the
	 * cache invalidate can win the race against the ADM3 graceful-flush
	 * write retiring to DRAM: the CPU then reads back the original zeros
	 * while UARTDM_RX_TOTAL_SNAP (an independent HW counter) reports the
	 * correct byte count -- the "correct count, zero data" symptom seen on
	 * the TouchPad BT (GSBI6) RX. A coherent buffer has no cache line to
	 * race, so ADM writes are always visible, and its phys address is
	 * stable for the channel's lifetime (no per-cycle map/unmap).
	 */
	dma->rx.virt = dma_alloc_coherent(dev, UARTDM_RX_SIZE, &dma->rx.phys,
					  GFP_KERNEL);
	if (!dma->rx.virt)
		goto rel_rx;

	memset(&conf, 0, sizeof(conf));
	conf.direction = DMA_DEV_TO_MEM;
	conf.device_fc = true;
	conf.src_addr = base + UARTDM_RF;
	conf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	conf.src_maxburst = UARTDM_BURST_SIZE;
	if (crci) {
		conf.peripheral_config = &periph_conf;
		conf.peripheral_size = sizeof(periph_conf);
		periph_conf.crci = crci;
	}

	ret = dmaengine_slave_config(dma->chan, &conf);
	if (ret)
		goto err;

	dev_info(dev, "RX DMA channel acquired (CRCI %u)\n", crci);
	dma->dir = DMA_FROM_DEVICE;

	if (msm_port->is_uartdm < UARTDM_1P4)
		dma->enable_bit = UARTDM_DMEN_RX_DM_ENABLE;
	else
		dma->enable_bit = UARTDM_DMEN_RX_BAM_ENABLE;

	return;
err:
	dma_free_coherent(dev, UARTDM_RX_SIZE, dma->rx.virt, dma->rx.phys);
rel_rx:
	dma_release_channel(dma->chan);
no_rx:
	memset(dma, 0, sizeof(*dma));
}

static inline void msm_wait_for_xmitr(struct uart_port *port)
{
	unsigned int timeout = 500000;

	while (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_EMPTY)) {
		if (msm_read(port, MSM_UART_ISR) & MSM_UART_ISR_TX_READY)
			break;
		udelay(1);
		if (!timeout--)
			break;
	}
	msm_write(port, MSM_UART_CR_CMD_RESET_TX_READY, MSM_UART_CR);
}

static void msm_stop_tx(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);

	msm_port->imr &= ~MSM_UART_IMR_TXLEV;
	msm_write_imr(port, msm_port->imr);
}

static void msm_start_tx(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	struct msm_dma *dma = &msm_port->tx_dma;

	/* Already started in DMA mode */
	if (sg_dma_len(&dma->tx_sg))
		return;

	msm_port->imr |= MSM_UART_IMR_TXLEV;
	msm_write_imr(port, msm_port->imr);
}

static void msm_reset_dm_count(struct uart_port *port, int count)
{
	msm_wait_for_xmitr(port);
	msm_write(port, count, UARTDM_NCF_TX);
	msm_read(port, UARTDM_NCF_TX);
}

static void msm_complete_tx_dma(void *args)
{
	struct msm_port *msm_port = args;
	struct uart_port *port = &msm_port->uart;
	struct tty_port *tport = &port->state->port;
	struct msm_dma *dma = &msm_port->tx_dma;
	struct dma_tx_state state;
	unsigned long flags;
	unsigned int count;
	u32 val;

	uart_port_lock_irqsave(port, &flags);

	/* Already stopped */
	if (!sg_dma_len(&dma->tx_sg))
		goto done;

	dmaengine_tx_status(dma->chan, dma->cookie, &state);

	dma_unmap_sg(port->dev, &dma->tx_sg, 1, dma->dir);

	val = msm_read(port, UARTDM_DMEN);
	val &= ~dma->enable_bit;
	msm_write(port, val, UARTDM_DMEN);

	if (msm_port->is_uartdm > UARTDM_1P3) {
		msm_write(port, MSM_UART_CR_CMD_RESET_TX, MSM_UART_CR);
		msm_write(port, MSM_UART_CR_TX_ENABLE, MSM_UART_CR);
	}

	count = sg_dma_len(&dma->tx_sg) - state.residue;
	uart_xmit_advance(port, count);
	sg_init_table(&dma->tx_sg, 1);

	/* Restore "Tx FIFO below watermark" interrupt */
	msm_port->imr |= MSM_UART_IMR_TXLEV;
	msm_write_imr(port, msm_port->imr);

	if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	msm_handle_tx(port);
done:
	uart_port_unlock_irqrestore(port, flags);
}

static int msm_handle_tx_dma(struct msm_port *msm_port, unsigned int count)
{
	struct uart_port *port = &msm_port->uart;
	struct tty_port *tport = &port->state->port;
	struct msm_dma *dma = &msm_port->tx_dma;
	unsigned int mapped;
	int ret;
	u32 val;

	sg_init_table(&dma->tx_sg, 1);
	kfifo_dma_out_prepare(&tport->xmit_fifo, &dma->tx_sg, 1, count);

	mapped = dma_map_sg(port->dev, &dma->tx_sg, 1, dma->dir);
	if (!mapped) {
		ret = -EIO;
		goto zero_sg;
	}

	dma->desc = dmaengine_prep_slave_sg(dma->chan, &dma->tx_sg, 1,
						DMA_MEM_TO_DEV,
						DMA_PREP_INTERRUPT |
						DMA_PREP_FENCE);
	if (!dma->desc) {
		ret = -EIO;
		goto unmap;
	}

	dma->desc->callback = msm_complete_tx_dma;
	dma->desc->callback_param = msm_port;

	dma->cookie = dmaengine_submit(dma->desc);
	ret = dma_submit_error(dma->cookie);
	if (ret)
		goto unmap;

	/*
	 * Using DMA complete for Tx FIFO reload, no need for
	 * "Tx FIFO below watermark" one, disable it
	 */
	msm_port->imr &= ~MSM_UART_IMR_TXLEV;
	msm_write_imr(port, msm_port->imr);

	val = msm_read(port, UARTDM_DMEN);
	val |= dma->enable_bit;

	if (msm_port->is_uartdm < UARTDM_1P4)
		msm_write(port, val, UARTDM_DMEN);

	msm_reset_dm_count(port, count);

	if (msm_port->is_uartdm > UARTDM_1P3)
		msm_write(port, val, UARTDM_DMEN);

	dma_async_issue_pending(dma->chan);
	return 0;
unmap:
	dma_unmap_sg(port->dev, &dma->tx_sg, 1, dma->dir);
zero_sg:
	sg_init_table(&dma->tx_sg, 1);
	return ret;
}

static void msm_complete_rx_dma(void *args)
{
	struct msm_port *msm_port = args;
	struct uart_port *port = &msm_port->uart;
	struct tty_port *tport = &port->state->port;
	struct msm_dma *dma = &msm_port->rx_dma;
	int count = 0, i, sysrq;
	unsigned long flags;
	u32 val;

	uart_port_lock_irqsave(port, &flags);

	/* Already stopped */
	if (!dma->rx.count)
		goto done;

	val = msm_read(port, UARTDM_DMEN);
	val &= ~dma->enable_bit;
	msm_write(port, val, UARTDM_DMEN);

	if (msm_read(port, MSM_UART_SR) & MSM_UART_SR_OVERRUN) {
		port->icount.overrun++;
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
		msm_write(port, MSM_UART_CR_CMD_RESET_ERR, MSM_UART_CR);
	}

	count = msm_read(port, UARTDM_RX_TOTAL_SNAP);

	port->icount.rx += count;

	dma->rx.count = 0;

	/*
	 * Coherent buffer: no unmap needed. Order the CPU reads after the ADM
	 * completion IRQ so the flushed RX bytes are guaranteed visible.
	 */
	dma_rmb();

	for (i = 0; i < count; i++) {
		char flag = TTY_NORMAL;

		if (msm_port->break_detected && dma->rx.virt[i] == 0) {
			port->icount.brk++;
			flag = TTY_BREAK;
			msm_port->break_detected = false;
			if (uart_handle_break(port))
				continue;
		}

		if (!(port->read_status_mask & MSM_UART_SR_RX_BREAK))
			flag = TTY_NORMAL;

		sysrq = uart_prepare_sysrq_char(port, dma->rx.virt[i]);
		if (!sysrq)
			tty_insert_flip_char(tport, dma->rx.virt[i], flag);
	}

	msm_start_rx_dma(msm_port);
done:
	uart_unlock_and_check_sysrq_irqrestore(port, flags);

	if (count)
		tty_flip_buffer_push(tport);
}

static void msm_start_rx_dma(struct msm_port *msm_port)
{
	struct msm_dma *dma = &msm_port->rx_dma;
	struct uart_port *uart = &msm_port->uart;
	u32 val;
	int ret;
	bool bt = uart->mapbase == 0x16540000;

	if (IS_ENABLED(CONFIG_CONSOLE_POLL)) {
		if (bt) dev_info(uart->dev, "rx_dma: bail CONSOLE_POLL=y\n");
		return;
	}

	if (!dma->chan) {
		if (bt) dev_info(uart->dev, "rx_dma: bail dma->chan=NULL\n");
		return;
	}

	if (bt) dev_info(uart->dev, "rx_dma: prep_slave_single phys=%pad size=%u\n",
			 &dma->rx.phys, UARTDM_RX_SIZE);
	/* Coherent RX buffer: phys is stable, no per-cycle mapping needed. */
	dma->desc = dmaengine_prep_slave_single(dma->chan, dma->rx.phys,
						UARTDM_RX_SIZE, DMA_DEV_TO_MEM,
						DMA_PREP_INTERRUPT);
	if (!dma->desc) {
		if (bt) dev_info(uart->dev, "rx_dma: prep returned NULL -> sw_mode\n");
		goto sw_mode;
	}

	dma->desc->callback = msm_complete_rx_dma;
	dma->desc->callback_param = msm_port;

	dma->cookie = dmaengine_submit(dma->desc);
	ret = dma_submit_error(dma->cookie);
	if (ret) {
		if (bt) dev_info(uart->dev, "rx_dma: submit err=%d -> sw_mode\n", ret);
		goto sw_mode;
	}
	/*
	 * Using DMA for FIFO off-load, no need for "Rx FIFO over
	 * watermark" or "stale" interrupts, disable them
	 */
	msm_port->imr &= ~(MSM_UART_IMR_RXLEV | MSM_UART_IMR_RXSTALE);

	/*
	 * Well, when DMA is ADM3 engine(implied by <= UARTDM v1.3),
	 * we need RXSTALE to flush input DMA fifo to memory
	 */
	if (msm_port->is_uartdm < UARTDM_1P4)
		msm_port->imr |= MSM_UART_IMR_RXSTALE;

	msm_write_imr(uart, msm_port->imr);

	dma->rx.count = UARTDM_RX_SIZE;

	dma_async_issue_pending(dma->chan);

	msm_write(uart, MSM_UART_CR_CMD_RESET_STALE_INT, MSM_UART_CR);
	msm_write(uart, MSM_UART_CR_CMD_STALE_EVENT_ENABLE, MSM_UART_CR);

	val = msm_read(uart, UARTDM_DMEN);
	val |= dma->enable_bit;

	if (msm_port->is_uartdm < UARTDM_1P4)
		msm_write(uart, val, UARTDM_DMEN);

	msm_write(uart, UARTDM_RX_SIZE, UARTDM_DMRX);

	if (msm_port->is_uartdm > UARTDM_1P3)
		msm_write(uart, val, UARTDM_DMEN);

	if (bt) {
		u32 dmen_rb = msm_read(uart, UARTDM_DMEN);
		u32 imr_rb  = msm_read(uart, MSM_UART_IMR);
		dev_info(uart->dev,
			 "rx_dma: armed, enable_bit=0x%02x DMEN_w=0x%02x DMEN_r=0x%02x IMR=0x%x\n",
			 dma->enable_bit, val, dmen_rb, imr_rb);
	}
	return;

sw_mode:
	/*
	 * Switch from DMA to SW/FIFO mode. After clearing Rx BAM (UARTDM_DMEN),
	 * receiver must be reset.
	 */
	msm_write(uart, MSM_UART_CR_CMD_RESET_RX, MSM_UART_CR);
	msm_write(uart, MSM_UART_CR_RX_ENABLE, MSM_UART_CR);

	msm_write(uart, MSM_UART_CR_CMD_RESET_STALE_INT, MSM_UART_CR);
	msm_write(uart, 0xFFFFFF, UARTDM_DMRX);
	msm_write(uart, MSM_UART_CR_CMD_STALE_EVENT_ENABLE, MSM_UART_CR);

	/* Re-enable RX interrupts */
	msm_port->imr |= MSM_UART_IMR_RXLEV | MSM_UART_IMR_RXSTALE;
	msm_write_imr(uart, msm_port->imr);

	if (bt) {
		u32 imr_rb = msm_read(uart, MSM_UART_IMR);
		dev_info(uart->dev, "rx_dma: sw_mode IMR=0x%x\n", imr_rb);
	}
}

static void msm_stop_rx(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	struct msm_dma *dma = &msm_port->rx_dma;

	msm_port->imr &= ~(MSM_UART_IMR_RXLEV | MSM_UART_IMR_RXSTALE);
	msm_write_imr(port, msm_port->imr);

	if (dma->chan)
		msm_stop_dma(port, dma);
}

static void msm_enable_ms(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);

	msm_port->imr |= MSM_UART_IMR_DELTA_CTS;
	msm_write_imr(port, msm_port->imr);
}

static void msm_handle_rx_dm(struct uart_port *port, unsigned int misr)
	__must_hold(&port->lock)
{
	struct tty_port *tport = &port->state->port;
	unsigned int sr;
	int count = 0;
	struct msm_port *msm_port = to_msm_port(port);

	if ((msm_read(port, MSM_UART_SR) & MSM_UART_SR_OVERRUN)) {
		port->icount.overrun++;
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
		msm_write(port, MSM_UART_CR_CMD_RESET_ERR, MSM_UART_CR);
	}

	if (misr & MSM_UART_IMR_RXSTALE) {
		count = msm_read(port, UARTDM_RX_TOTAL_SNAP) -
			msm_port->old_snap_state;
		msm_port->old_snap_state = 0;
	} else {
		count = 4 * (msm_read(port, MSM_UART_RFWR));
		msm_port->old_snap_state += count;
	}

	/* TODO: Precise error reporting */

	port->icount.rx += count;

	while (count > 0) {
		unsigned char buf[4];
		int sysrq, r_count, i;

		sr = msm_read(port, MSM_UART_SR);
		if ((sr & MSM_UART_SR_RX_READY) == 0) {
			msm_port->old_snap_state -= count;
			break;
		}

		ioread32_rep(port->membase + UARTDM_RF, buf, 1);
		r_count = min_t(int, count, sizeof(buf));

		for (i = 0; i < r_count; i++) {
			char flag = TTY_NORMAL;

			if (msm_port->break_detected && buf[i] == 0) {
				port->icount.brk++;
				flag = TTY_BREAK;
				msm_port->break_detected = false;
				if (uart_handle_break(port))
					continue;
			}

			if (!(port->read_status_mask & MSM_UART_SR_RX_BREAK))
				flag = TTY_NORMAL;

			sysrq = uart_prepare_sysrq_char(port, buf[i]);
			if (!sysrq)
				tty_insert_flip_char(tport, buf[i], flag);
		}
		count -= r_count;
	}

	tty_flip_buffer_push(tport);

	if (misr & (MSM_UART_IMR_RXSTALE))
		msm_write(port, MSM_UART_CR_CMD_RESET_STALE_INT, MSM_UART_CR);
	msm_write(port, 0xFFFFFF, UARTDM_DMRX);
	msm_write(port, MSM_UART_CR_CMD_STALE_EVENT_ENABLE, MSM_UART_CR);

	/* Try to use DMA */
	msm_start_rx_dma(msm_port);
}

static void msm_handle_rx(struct uart_port *port)
	__must_hold(&port->lock)
{
	struct tty_port *tport = &port->state->port;
	unsigned int sr;

	/*
	 * Handle overrun. My understanding of the hardware is that overrun
	 * is not tied to the RX buffer, so we handle the case out of band.
	 */
	if ((msm_read(port, MSM_UART_SR) & MSM_UART_SR_OVERRUN)) {
		port->icount.overrun++;
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
		msm_write(port, MSM_UART_CR_CMD_RESET_ERR, MSM_UART_CR);
	}

	/* and now the main RX loop */
	while ((sr = msm_read(port, MSM_UART_SR)) & MSM_UART_SR_RX_READY) {
		unsigned int c;
		char flag = TTY_NORMAL;
		int sysrq;

		c = msm_read(port, MSM_UART_RF);

		if (sr & MSM_UART_SR_RX_BREAK) {
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		} else if (sr & MSM_UART_SR_PAR_FRAME_ERR) {
			port->icount.frame++;
		} else {
			port->icount.rx++;
		}

		/* Mask conditions we're ignoring. */
		sr &= port->read_status_mask;

		if (sr & MSM_UART_SR_RX_BREAK)
			flag = TTY_BREAK;
		else if (sr & MSM_UART_SR_PAR_FRAME_ERR)
			flag = TTY_FRAME;

		sysrq = uart_prepare_sysrq_char(port, c);
		if (!sysrq)
			tty_insert_flip_char(tport, c, flag);
	}

	tty_flip_buffer_push(tport);
}

static void msm_handle_tx_pio(struct uart_port *port, unsigned int tx_count)
{
	struct msm_port *msm_port = to_msm_port(port);
	struct tty_port *tport = &port->state->port;
	unsigned int num_chars;
	unsigned int tf_pointer = 0;
	void __iomem *tf;
	/* BT TX gap trace (bt_tx_trace param + GSBI6): detect inter-byte gaps. */
	bool trace = bt_tx_trace && port->mapbase == 0x16540000;
	ktime_t t0 = trace ? ktime_get() : 0;
	u32 sr_enter = trace ? msm_read(port, MSM_UART_SR) : 0;

	if (msm_port->is_uartdm)
		tf = port->membase + UARTDM_TF;
	else
		tf = port->membase + MSM_UART_TF;

	/*
	 * Forced inter-byte gap experiment (bt_tx_bytegap_us, BT port only): send
	 * one byte per UARTDM transfer, wait for the transmitter to fully drain
	 * (SR TX_EMPTY), then idle the line for the configured gap before the next
	 * byte. Produces real gaps BETWEEN bytes on gpio53 that a single gapless
	 * FIFO burst never can. Busy-waits with IRQs off -> diag only.
	 */
	if (bt_tx_bytegap_us && port->mapbase == 0x16540000 && msm_port->is_uartdm) {
		while (tf_pointer < tx_count) {
			unsigned char buf[4] = { 0 };
			int g;

			if (uart_fifo_out(port, buf, 1) != 1)
				break;
			msm_reset_dm_count(port, 1);
			for (g = 0; g < 50000 && !(msm_read(port, MSM_UART_SR) &
						   MSM_UART_SR_TX_READY); g++)
				cpu_relax();
			iowrite32_rep(tf, buf, 1);
			tf_pointer++;
			for (g = 0; g < 100000 && !(msm_read(port, MSM_UART_SR) &
						    MSM_UART_SR_TX_EMPTY); g++)
				cpu_relax();
			udelay(bt_tx_bytegap_us);
		}
		bt_tx_pio_bursts++;
		goto tx_done;
	}

	if (tx_count && msm_port->is_uartdm)
		msm_reset_dm_count(port, tx_count);

	while (tf_pointer < tx_count) {
		unsigned char buf[4] = { 0 };

		if (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_READY))
			break;

		if (msm_port->is_uartdm)
			num_chars = min(tx_count - tf_pointer,
					(unsigned int)sizeof(buf));
		else
			num_chars = 1;

		num_chars = uart_fifo_out(port, buf, num_chars);
		iowrite32_rep(tf, buf, 1);
		tf_pointer += num_chars;
	}

	/*
	 * BT UART PIO underrun diag: a mid-frame TX_READY drop (tf_pointer <
	 * tx_count above) means the TX FIFO emptied before the burst finished,
	 * so the shift register can idle the line between bytes = an inter-byte
	 * WIRE GAP the CSR BCSP receiver may reject. Counted unconditionally for
	 * GSBI6; read via /sys/module/msm_serial/parameters/bt_tx_pio_*.
	 */
	if (port->mapbase == 0x16540000 && tx_count) {
		bt_tx_pio_bursts++;
		if (tf_pointer < tx_count)
			bt_tx_pio_underruns++;
	}

	if (trace) {
		static ktime_t prev_end;
		ktime_t t1 = ktime_get();
		s64 feed_us = ktime_us_delta(t1, t0);
		s64 gap_us = prev_end ? ktime_us_delta(t0, prev_end) : -1;
		/* on-wire time floor for this burst: bytes * 10 bits / baud */
		unsigned int floor_us =
			tx_count ? (tx_count * 10u * 1000000u) / 115200u : 0;
		dev_info(port->dev,
			 "BTTX: req=%u wrote=%u feed=%lldus floor=%uus gap_since_prev=%lldus SR_in=0x%x SR_out=0x%x%s\n",
			 tx_count, tf_pointer, feed_us, floor_us, gap_us,
			 sr_enter, msm_read(port, MSM_UART_SR),
			 (tf_pointer < tx_count) ?
			 " [TX_READY dropped mid-frame -> FIFO refill needed -> potential WIRE GAP]" :
			 " [whole burst fed -> drains contiguously, no intra-burst gap]");
		prev_end = t1;
	}

tx_done:
	/* disable tx interrupts if nothing more to send */
	if (kfifo_is_empty(&tport->xmit_fifo))
		msm_stop_tx(port);

	if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void msm_handle_tx(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	struct tty_port *tport = &port->state->port;
	struct msm_dma *dma = &msm_port->tx_dma;
	unsigned int pio_count, dma_count, dma_min;
	char buf[4] = { 0 };
	void __iomem *tf;
	int err = 0;

	if (port->x_char) {
		if (msm_port->is_uartdm)
			tf = port->membase + UARTDM_TF;
		else
			tf = port->membase + MSM_UART_TF;

		buf[0] = port->x_char;

		if (msm_port->is_uartdm)
			msm_reset_dm_count(port, 1);

		iowrite32_rep(tf, buf, 1);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (kfifo_is_empty(&tport->xmit_fifo) || uart_tx_stopped(port)) {
		msm_stop_tx(port);
		return;
	}

	dma_count = pio_count = kfifo_out_linear(&tport->xmit_fifo, NULL,
			UART_XMIT_SIZE);

	dma_min = 1;	/* Always DMA */
	if (msm_port->is_uartdm > UARTDM_1P3) {
		dma_count = UARTDM_TX_AIGN(dma_count);
		dma_min = UARTDM_BURST_SIZE;
	} else {
		if (dma_count > UARTDM_TX_MAX)
			dma_count = UARTDM_TX_MAX;
	}

	if (pio_count > port->fifosize)
		pio_count = port->fifosize;

	if (!dma->chan || dma_count < dma_min)
		msm_handle_tx_pio(port, pio_count);
	else
		err = msm_handle_tx_dma(msm_port, dma_count);

	if (err)	/* fall back to PIO mode */
		msm_handle_tx_pio(port, pio_count);
}

static void msm_handle_delta_cts(struct uart_port *port)
{
	msm_write(port, MSM_UART_CR_CMD_RESET_CTS, MSM_UART_CR);
	port->icount.cts++;
	wake_up_interruptible(&port->state->port.delta_msr_wait);
}

static irqreturn_t msm_uart_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct msm_port *msm_port = to_msm_port(port);
	struct msm_dma *dma = &msm_port->rx_dma;
	unsigned int misr;
	u32 val;

	uart_port_lock(port);
	misr = msm_read(port, MSM_UART_MISR);
	msm_write(port, 0, MSM_UART_IMR); /* disable interrupt */

	if (misr & MSM_UART_IMR_RXBREAK_START) {
		msm_port->break_detected = true;
		msm_write(port, MSM_UART_CR_CMD_RESET_RXBREAK_START, MSM_UART_CR);
	}

	if (misr & (MSM_UART_IMR_RXLEV | MSM_UART_IMR_RXSTALE)) {
		if (dma->rx.count) {
			val = MSM_UART_CR_CMD_STALE_EVENT_DISABLE;
			msm_write(port, val, MSM_UART_CR);
			val = MSM_UART_CR_CMD_RESET_STALE_INT;
			msm_write(port, val, MSM_UART_CR);
			/*
			 * Flush DMA input fifo to memory, this will also
			 * trigger DMA RX completion
			 */
			dmaengine_terminate_all(dma->chan);
		} else if (msm_port->is_uartdm) {
			msm_handle_rx_dm(port, misr);
		} else {
			msm_handle_rx(port);
		}
	}
	if (misr & MSM_UART_IMR_TXLEV)
		msm_handle_tx(port);
	if (misr & MSM_UART_IMR_DELTA_CTS)
		msm_handle_delta_cts(port);

	msm_write_imr(port, msm_port->imr); /* restore interrupt */
	uart_unlock_and_check_sysrq(port);

	return IRQ_HANDLED;
}

static unsigned int msm_tx_empty(struct uart_port *port)
{
	return (msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int msm_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR | TIOCM_RTS;
}

static void msm_reset(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	unsigned int mr;

	/* reset everything */
	msm_write(port, MSM_UART_CR_CMD_RESET_RX, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_TX, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_ERR, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_BREAK_INT, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_CTS, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_RFR, MSM_UART_CR);
	mr = msm_read(port, MSM_UART_MR1);
	mr &= ~MSM_UART_MR1_RX_RDY_CTL;
	msm_write(port, mr, MSM_UART_MR1);

	/* Disable DM modes */
	if (msm_port->is_uartdm)
		msm_write(port, 0, UARTDM_DMEN);
}

static void msm_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned int mr;

	mr = msm_read(port, MSM_UART_MR1);

	if (!(mctrl & TIOCM_RTS)) {
		mr &= ~MSM_UART_MR1_RX_RDY_CTL;
		msm_write(port, mr, MSM_UART_MR1);
		msm_write(port, MSM_UART_CR_CMD_RESET_RFR, MSM_UART_CR);
	} else {
		mr |= MSM_UART_MR1_RX_RDY_CTL;
		msm_write(port, mr, MSM_UART_MR1);
	}
}

static void msm_break_ctl(struct uart_port *port, int break_ctl)
{
	if (break_ctl)
		msm_write(port, MSM_UART_CR_CMD_START_BREAK, MSM_UART_CR);
	else
		msm_write(port, MSM_UART_CR_CMD_STOP_BREAK, MSM_UART_CR);
}

struct msm_baud_map {
	u16	divisor;
	u8	code;
	u8	rxstale;
};

static const struct msm_baud_map *
msm_find_best_baud(struct uart_port *port, unsigned int baud,
		   unsigned long *rate)
{
	struct msm_port *msm_port = to_msm_port(port);
	unsigned int divisor, result, clk_mult = 1;
	unsigned long target, old, best_rate = 0, diff, best_diff = ULONG_MAX;
	const struct msm_baud_map *entry, *end, *best;
	static const struct msm_baud_map table[] = {
		{    1, 0xff, 31 },
		{    2, 0xee, 16 },
		{    3, 0xdd,  8 },
		{    4, 0xcc,  6 },
		{    6, 0xbb,  6 },
		{    8, 0xaa,  6 },
		{   12, 0x99,  6 },
		{   16, 0x88,  1 },
		{   24, 0x77,  1 },
		{   32, 0x66,  1 },
		{   48, 0x55,  1 },
		{   96, 0x44,  1 },
		{  192, 0x33,  1 },
		{  384, 0x22,  1 },
		{  768, 0x11,  1 },
		{ 1536, 0x00,  1 },
	};

	/*
	 * GSBI6 (phys 0x16540000) is the TouchPad BT (CSR BlueCore) UART. The
	 * CSR chip decodes webOS's TX but not ours despite identical MR1/MR2/
	 * 8N1/115200. The one remaining config difference: webOS clocks this
	 * UART from a higher fundamental rate and divides via CSR (fund_clk
	 * 7372800 / CSR DIV_4 for 115200) instead of fund_clk = 16*baud / CSR
	 * DIV_1. Mirror that here for low baud on the BT UART: target a 4x
	 * higher clock so the selector picks a CSR /4 divider from a cleaner
	 * higher source clock (mainline gcc-msm8660 ftbl supports 7372800).
	 */
	if (port->mapbase == 0x16540000 && baud && baud <= 460800)
		clk_mult = 4;

	best = table; /* Default to smallest divider */
	target = clk_round_rate(msm_port->clk, clk_mult * 16 * baud);
	divisor = DIV_ROUND_CLOSEST(target, 16 * baud);

	end = table + ARRAY_SIZE(table);
	entry = table;
	while (entry < end) {
		if (entry->divisor <= divisor) {
			result = target / entry->divisor / 16;
			diff = abs(result - baud);

			/* Keep track of best entry */
			if (diff < best_diff) {
				best_diff = diff;
				best = entry;
				best_rate = target;
			}

			if (result == baud)
				break;
		} else {
			old = target;
			target = clk_round_rate(msm_port->clk, old + 1);
			/*
			 * The rate didn't get any faster so we can't do
			 * better at dividing it down
			 */
			if (target == old)
				break;

			/* Start the divisor search over at this new rate */
			entry = table;
			divisor = DIV_ROUND_CLOSEST(target, 16 * baud);
			continue;
		}
		entry++;
	}

	*rate = best_rate;
	return best;
}

static int msm_set_baud_rate(struct uart_port *port, unsigned int baud,
			     unsigned long *saved_flags)
	__must_hold(&port->lock)
{
	unsigned int rxstale, watermark, mask;
	struct msm_port *msm_port = to_msm_port(port);
	const struct msm_baud_map *entry;
	unsigned long flags, rate;

	flags = *saved_flags;
	uart_port_unlock_irqrestore(port, flags);

	entry = msm_find_best_baud(port, baud, &rate);
	dev_pm_opp_set_rate(port->dev, rate);
	baud = rate / 16 / entry->divisor;

	uart_port_lock_irqsave(port, &flags);
	*saved_flags = flags;
	port->uartclk = rate;

	msm_write(port, entry->code, MSM_UART_CSR);

	/*
	 * BT UART diagnostic: confirm what msm_set_baud_rate actually set,
	 * including whether MSM_UART_CSR (0x0008) is even read-back-able on
	 * this UART block.  Live tests 2026-06-08 showed devmem reads of
	 * 0x16540008 returning 0x0c (TX-DIV=0xc nibble, RX-DIV=0x0 nibble)
	 * even after the driver wrote 0xff for divisor=1 — strongly suggesting
	 * the offset is write-only and reads return unrelated FIFO/state.
	 * This dev_info prints the rate, computed divisor, written code, the
	 * post-write CSR readback, and the resolved baud so we can compare.
	 */
	if (port->mapbase == 0x16540000) {
		unsigned int csr_rb = msm_read(port, MSM_UART_CSR);

		dev_info(port->dev,
			 "msm_set_baud_rate: rate=%lu divisor=%u CSR_w=0x%02x CSR_r=0x%08x baud=%u\n",
			 rate, entry->divisor, entry->code, csr_rb, baud);
	}

	/* RX stale watermark */
	rxstale = entry->rxstale;
	watermark = MSM_UART_IPR_STALE_LSB & rxstale;
	if (msm_port->is_uartdm) {
		mask = MSM_UART_DM_IPR_STALE_TIMEOUT_MSB;
	} else {
		watermark |= MSM_UART_IPR_RXSTALE_LAST;
		mask = MSM_UART_IPR_STALE_TIMEOUT_MSB;
	}

	watermark |= mask & (rxstale << 2);

	/* Apply IPR override (BT port only) if /sys param is set. */
	if (msm_port->bt_is_bt_uart && bt_ipr_override) {
		msm_write(port, bt_ipr_override, MSM_UART_IPR);
		dev_info(port->dev, "BT IPR overridden to 0x%x (post-baud)\n",
			 bt_ipr_override);
	} else {
		msm_write(port, watermark, MSM_UART_IPR);
	}

	/* set RX watermark */
	watermark = (port->fifosize * 3) / 4;
	msm_write(port, watermark, MSM_UART_RFWR);

	/*
	 * Set TX watermark (level at which the TX-FIFO "need more data" IRQ
	 * fires). Mainline default is 10 = refill late = little headroom =
	 * prone to TX-FIFO underrun -> inter-byte gaps when the ISR is slow.
	 * The legacy webOS hsuart uses 32 on the BT UART (msm_uart_dm.c:1641)
	 * = refill early = no underrun. The CSR BlueCore chip's RX flushes on
	 * a mid-frame gap, so match webOS=32 on GSBI6 (BT) to keep our SYNC_RSP
	 * gap-free. Other UARTs keep 10 (work fine).
	 */
	msm_write(port, (port->mapbase == 0x16540000) ? 32 : 10, MSM_UART_TFWR);

	msm_write(port, MSM_UART_CR_CMD_PROTECTION_EN, MSM_UART_CR);
	msm_reset(port);

	/* Enable RX and TX */
	msm_write(port, MSM_UART_CR_TX_ENABLE | MSM_UART_CR_RX_ENABLE, MSM_UART_CR);

	/* turn on RX and CTS interrupts */
	msm_port->imr = MSM_UART_IMR_RXLEV | MSM_UART_IMR_RXSTALE |
			MSM_UART_IMR_CURRENT_CTS | MSM_UART_IMR_RXBREAK_START;

	msm_write_imr(port, msm_port->imr);

	if (msm_port->is_uartdm) {
		msm_write(port, MSM_UART_CR_CMD_RESET_STALE_INT, MSM_UART_CR);
		msm_write(port, 0xFFFFFF, UARTDM_DMRX);
		msm_write(port, MSM_UART_CR_CMD_STALE_EVENT_ENABLE, MSM_UART_CR);
	}

	return baud;
}

static void msm_init_clock(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);

	dev_pm_opp_set_rate(port->dev, port->uartclk);
	clk_prepare_enable(msm_port->clk);
	clk_prepare_enable(msm_port->pclk);
	msm_serial_set_mnd_regs(port);
}

/*
 * TouchPad Bluetooth bring-up (H2). The CSR BlueCore's UART RX is power-gated
 * and the legacy webOS driver re-ran a pin-mux wake "glitch" right before every
 * BCSP link-establishment TX. We approximate that by re-running the glitch
 * periodically (only while the TX line is idle, so a frame in flight is never
 * corrupted) while the BT UART sits at the 115200 link-est baud. 0 disables.
 * Tune on-device against the chip's RX re-gate timeout.
 */
/*
 * Default OFF: the periodic (asynchronous) glitch was found to clip inbound
 * frames on the TouchPad. The faithful wake is now done synchronously right
 * before each TX from the BCSP driver via msm_serial_bt_wake_glitch(). Set
 * >0 only to also re-arm the (now RX-safe, TX/RTS-only) glitch periodically.
 */
static int bt_wake_period_ms;	/* 0 = off */
module_param(bt_wake_period_ms, int, 0644);
MODULE_PARM_DESC(bt_wake_period_ms,
	"TouchPad BT: re-arm UART pin-mux wake glitch every N ms during link-est (0=off, default 0)");

#define MSM_BT_UART_MAPBASE	0x16540000

/*
 * The single BT UART port + a lock to serialize wake glitches (the synchronous
 * pre-TX glitch from the BCSP driver, the periodic work, and startup can all
 * race on the shared pinctrl). Set in probe for mapbase 0x16540000.
 */
static struct uart_port *msm_bt_wake_port;
static DEFINE_MUTEX(msm_bt_wake_lock);

/*
 * If bt_imr_override is set and this port is the BT UART, return the
 * override value to write into IMR instead of the driver-computed one.
 * Used by the IRQ handler's IMR restore (msm_uart_irq) and by every other
 * site that writes msm_port->imr -> MSM_UART_IMR.
 */
static inline unsigned int msm_apply_bt_imr_override(struct msm_port *msm_port,
						     unsigned int imr)
{
	if (msm_port->bt_is_bt_uart && bt_imr_override)
		return bt_imr_override;
	return imr;
}

/*
 * Universal IMR-write helper.  ALL writes to MSM_UART_IMR that pass a
 * non-zero value (i.e. enabling/restoring interrupts) must go through this
 * to honour bt_imr_override consistently.  The bare zero-writes (disable
 * IRQs during processing) still use msm_write() directly so we don't
 * accidentally re-enable the override IMR mid-handler.
 *
 * Without this wrapper the override applied only at the IRQ-handler
 * restore site, while ~10 other writers (set_mctrl, set_termios, startup,
 * shutdown, set_baud_rate, console, console_write etc.) overwrote it with
 * the driver-computed `msm_port->imr` on every config touch.  That left
 * IMR flapping between the override and the driver value, and during a
 * `rmmod hci_uart` storm the partial override masked RXLEV/RXSTALE long
 * enough for the BCSP layer to wedge with the port lock held -> CPU0
 * stuck in cpuidle, CPU1 spinning in the unload path -> soft lockup.
 */
static inline void msm_write_imr(struct uart_port *port, unsigned int imr)
{
	struct msm_port *msm_port = to_msm_port(port);

	msm_write(port, msm_apply_bt_imr_override(msm_port, imr), MSM_UART_IMR);
}

/*
 * Writable-param setter for bt_ipr_override.  When the user echoes a value
 * to /sys/module/msm_serial/parameters/bt_ipr_override, apply it to the BT
 * port's IPR register immediately (don't wait for the next baud-rate set).
 */
static int bt_ipr_override_set(const char *val, const struct kernel_param *kp)
{
	struct uart_port *port;
	unsigned long flags;
	int ret;

	ret = param_set_uint(val, kp);
	if (ret)
		return ret;

	port = READ_ONCE(msm_bt_wake_port);
	if (port && bt_ipr_override) {
		uart_port_lock_irqsave(port, &flags);
		msm_write(port, bt_ipr_override, MSM_UART_IPR);
		uart_port_unlock_irqrestore(port, flags);
		dev_info(port->dev, "BT IPR overridden to 0x%x\n",
			 bt_ipr_override);
	}
	return 0;
}

/* ---- TouchPad BT TX diagnostics (no scope required) ---------------------- */

/*
 * MSM8660 TLMM pad config, used to diff the BT UART pads (TX gpio53, RX gpio54,
 * CTS gpio55, RFR/RTS gpio56) against the known-good webOS setup WITHOUT a
 * scope. ctl_reg encodes mux/drive/pull/output-enable; io_reg holds the live
 * IN/OUT line level (idle polarity). Register layout + bit positions are from
 * drivers/pinctrl/qcom/pinctrl-msm8660.c (ctl=0x1000+0x10*id, io=ctl+4;
 * mux@2, drv@6, pull@0, oe@9; io in@0, out@1). TLMM base = phys 0x800000.
 */
#define MSM_TLMM_PHYS		0x800000
#define MSM_TLMM_GPIO_CTL(n)	(0x1000 + 0x10 * (n))

static void msm_bt_diag_dump_pads(struct uart_port *port, const char *when)
{
	static const struct { int gpio; const char *name; } pads[] = {
		{ 53, "TX" }, { 54, "RX" }, { 55, "CTS" }, { 56, "RFR/RTS" },
	};
	static const unsigned int drv_ma[8] = { 2, 4, 6, 8, 10, 12, 14, 16 };
	static const char * const pull_s[4] = {
		"no-pull", "pull-down", "keeper", "pull-up"
	};
	void __iomem *tlmm;
	int i;

	/* Map a small window covering gpio53..56 ctl/io (4 pins * 0x10). */
	tlmm = ioremap(MSM_TLMM_PHYS + MSM_TLMM_GPIO_CTL(53), 0x40);
	if (!tlmm) {
		dev_warn(port->dev, "BT-DIAG: TLMM ioremap failed\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(pads); i++) {
		unsigned int off = MSM_TLMM_GPIO_CTL(pads[i].gpio) -
				   MSM_TLMM_GPIO_CTL(53);
		u32 ctl = readl(tlmm + off);
		u32 io  = readl(tlmm + off + 4);

		dev_info(port->dev,
			 "BT-DIAG[%s] gpio%d %-7s func=%u drive=%umA %s oe=%u | idle line in=%u out=%u (ctl=0x%08x io=0x%08x)\n",
			 when, pads[i].gpio, pads[i].name,
			 (ctl >> 2) & 7, drv_ma[(ctl >> 6) & 7], pull_s[ctl & 3],
			 (ctl >> 9) & 1, io & 1, (io >> 1) & 1, ctl, io);
	}
	iounmap(tlmm);
}

/*
 * Internal-loopback self-test (UARTDM MR2 LOOP_MODE = BIT(7), confirmed against
 * MSM8660 msm_serial_hs_hwreg.h). Routes TX->RX inside the controller, sends a
 * BCSP SYNC-RSP payload, and reads it back. A clean readback proves the digital
 * framing/divisor/start-stop path is internally consistent, isolating the
 * remaining suspect to the pad/analog (or the chip RX being gated). This
 * BYPASSES the gpio pad, so it does NOT validate the wire itself.
 *
 * INVASIVE: masks IRQs and resets TX/RX, which tears down any armed RX DMA.
 * Run as a one-shot check (BT interface up so clocks are on), then re-open the
 * BT interface. Trigger: echo 1 > /sys/module/msm_serial/parameters/bt_diag_loopback
 */
static int msm_bt_diag_loopback_run(void)
{
	struct uart_port *port = READ_ONCE(msm_bt_wake_port);
	static const u8 pat[4] = { 0xac, 0xaf, 0xef, 0xee }; /* BCSP SYNC-RSP */
	struct msm_port *msm_port;
	u8 got[4] = { 0 };
	u32 save_mr1, save_mr2, save_imr, sr, err_sr = 0;
	unsigned long flags;
	int i, n = 0, ret;

	if (!port)
		return -ENODEV;
	msm_port = to_msm_port(port);
	if (!msm_port->is_uartdm)
		return -EOPNOTSUPP;

	msm_bt_diag_dump_pads(port, "loopback-pre");

	uart_port_lock_irqsave(port, &flags);

	save_imr = msm_port->imr;
	save_mr1 = msm_read(port, MSM_UART_MR1);
	save_mr2 = msm_read(port, MSM_UART_MR2);

	/* Mask all UART IRQs so the normal (serdev) RX path can't eat our bytes. */
	msm_write(port, 0, MSM_UART_IMR);

	msm_write(port, MSM_UART_CR_CMD_RESET_RX, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_TX, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_ERR, MSM_UART_CR);

	/* Enable internal loopback. */
	msm_write(port, save_mr2 | 0x80, MSM_UART_MR2);
	msm_write(port, MSM_UART_CR_TX_ENABLE | MSM_UART_CR_RX_ENABLE,
		  MSM_UART_CR);

	/* UARTDM needs the TX char count programmed before the TF writes. */
	msm_reset_dm_count(port, sizeof(pat));
	for (i = 0; i < 20000 &&
		    !(msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_READY); i++)
		cpu_relax();
	iowrite32_rep(port->membase + UARTDM_TF, pat, 1);

	/* Poll the RX FIFO for the looped-back bytes (bounded). */
	for (i = 0; i < 20000 && n < (int)sizeof(pat); i++) {
		sr = msm_read(port, MSM_UART_SR);
		err_sr |= sr & (MSM_UART_SR_PAR_FRAME_ERR | MSM_UART_SR_OVERRUN);
		if (sr & MSM_UART_SR_RX_READY) {
			u32 w = msm_read(port, UARTDM_RF);
			int j;

			for (j = 0; j < 4 && n < (int)sizeof(pat); j++)
				got[n++] = (w >> (8 * j)) & 0xff;
		}
		cpu_relax();
	}

	/* Restore: disable loopback, reset, re-enable, unmask IRQs. */
	msm_write(port, MSM_UART_CR_CMD_RESET_RX, MSM_UART_CR);
	msm_write(port, MSM_UART_CR_CMD_RESET_TX, MSM_UART_CR);
	msm_write(port, save_mr2, MSM_UART_MR2);
	msm_write(port, save_mr1, MSM_UART_MR1);
	msm_write(port, MSM_UART_CR_TX_ENABLE | MSM_UART_CR_RX_ENABLE,
		  MSM_UART_CR);
	msm_port->imr = save_imr;
	msm_write_imr(port, save_imr);

	uart_port_unlock_irqrestore(port, flags);

	ret = (n == (int)sizeof(pat) && !memcmp(pat, got, sizeof(pat)) &&
	       !err_sr) ? 0 : -EIO;

	dev_info(port->dev,
		 "BT-DIAG loopback: sent %zu got %d [%*ph] err_sr=0x%x -> %s\n",
		 sizeof(pat), n, n, got, err_sr,
		 ret ? "MISMATCH/FRAMING (digital path suspect)" :
		       "OK (controller TX->RX digital path clean -> suspect pad/analog or chip RX)");
	dev_info(port->dev,
		 "BT-DIAG loopback was invasive (RX reset) -> re-open the BT interface\n");
	return ret;
}

static int msm_bt_diag_loopback_set(const char *val,
				    const struct kernel_param *kp)
{
	bool run;
	int ret = kstrtobool(val, &run);

	if (ret)
		return ret;
	if (run)
		return msm_bt_diag_loopback_run();
	return 0;
}

static const struct kernel_param_ops bt_diag_loopback_ops = {
	.set = msm_bt_diag_loopback_set,
};
module_param_cb(bt_diag_loopback, &bt_diag_loopback_ops, NULL, 0220);
MODULE_PARM_DESC(bt_diag_loopback,
	"BT UART: write 1 to run internal-loopback TX->RX self-test (invasive; re-open BT after)");

/*
 * Flip the chip-facing UART pins (TX gpio53 + RFR/RTS gpio56) to GPIO-high and
 * back, to wake the power-gated CSR BlueCore UART RX — the webOS btuart_pin_mux
 * off->on dance. Uses the RX-safe "gpio-txrts" state when present so an inbound
 * frame is never clipped, falling back to the full "gpio" state. Sleeps; call
 * from process context only.
 */
static void msm_bt_wake_glitch(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	struct pinctrl_state *gpio;

	if (!msm_port->startup_mux_glitch || !msm_port->pinctrl)
		return;

	gpio = msm_port->pinctrl_gpio_txrts ?: msm_port->pinctrl_gpio;

	mutex_lock(&msm_bt_wake_lock);
	pinctrl_select_state(msm_port->pinctrl, gpio);
	usleep_range(500, 1000);
	pinctrl_select_state(msm_port->pinctrl, msm_port->pinctrl_default);
	usleep_range(500, 1000);
	mutex_unlock(&msm_bt_wake_lock);
}

/*
 * Exported wake glitch for the BT UART, callable by the BCSP serdev driver
 * (hci_bcsp) synchronously right before a link-establishment TX. No-op until
 * the BT port has probed. Process context only (it sleeps).
 */
void msm_serial_bt_wake_glitch(void)
{
	struct uart_port *port = READ_ONCE(msm_bt_wake_port);

	if (port)
		msm_bt_wake_glitch(port);
}
EXPORT_SYMBOL_GPL(msm_serial_bt_wake_glitch);

/*
 * Full BT bring-up dance matching legacy webOS hsuart timing.
 *
 * Live capture from /var/log/messages on a working webOS BT bring-up:
 *
 *   T+0      btuart_pin_mux: on                   (FUNC_1 UART active)
 *   T+4ms    btuart_deassert_rts: 0(get)         (RFR LOW via FUNC_1 OUT_LOW)
 *   T+10ms   btuart_deassert_rts: 1(put)         (RFR HIGH via FUNC_GPIO OUT_HIGH)
 *   T+19ms   btuart_pin_mux: off                  (release TX/RX/CTS pins)
 *   T+23ms   btuart_pin_mux: on                   (reacquire UART)
 *   T+26ms   btuart_deassert_rts: 1(put)
 *   T+30ms   btuart_deassert_rts: 1(put)
 *   T+33ms   btuart_deassert_rts: 1(put)
 *   T+40ms   btuart_deassert_rts: 0(get)         (final ASSERT LOW)
 *
 * Total: ~40 ms with ms-level gaps between operations.
 *
 * Our pre-existing msm_bt_wake_glitch is ~1 ms total with us-level gaps and
 * does NOT toggle RFR through pinctrl states — it only switches TX/RFR pins
 * to GPIO and back.  Our msm_serial_bt_force_rfr touches RFR via UART CR
 * commands (SET_RFR / RESET_RFR), which keeps the pin in FUNC_1 the whole
 * time and doesn't reproduce the GPIO-mode HIGH transitions the CSR BlueCore
 * apparently needs to wake its RX UART.  This helper does the full webOS
 * pattern: pinctrl-based RFR toggles + ms-level holds.
 *
 * Tune via bt_dance_step_ms module param (default 4); set to 0 to disable.
 * Process context only — sleeps.
 */
static unsigned int bt_dance_step_ms = 4;
module_param(bt_dance_step_ms, uint, 0644);
MODULE_PARM_DESC(bt_dance_step_ms,
		 "Per-step delay (ms) for the legacy-webOS BT bring-up dance (default 4, 0=disable dance)");

void msm_serial_bt_dance(void)
{
	struct uart_port *port = READ_ONCE(msm_bt_wake_port);
	struct msm_port *msm_port;
	struct pinctrl_state *uart_state, *gpio_state;
	unsigned int step;

	if (!port)
		return;
	msm_port = to_msm_port(port);
	if (!msm_port->pinctrl || !msm_port->pinctrl_default)
		return;

	step = bt_dance_step_ms;
	if (!step)
		return;

	uart_state = msm_port->pinctrl_default;
	/* "gpio_txrts" state drives only RFR(gpio56)+TX(gpio53) to GPIO OUT_HIGH
	 * which is the closest mainline equivalent of the legacy SUSPENDED config
	 * for these pins.  Falls back to the broader "gpio" state if not present. */
	gpio_state = msm_port->pinctrl_gpio_txrts ?: msm_port->pinctrl_gpio;
	if (!gpio_state)
		return;

	mutex_lock(&msm_bt_wake_lock);

	/* T+0:  pin_mux ON (UART active) — already the case in steady state. */
	pinctrl_select_state(msm_port->pinctrl, uart_state);
	msleep(step);

	/* T+4ms: ASSERT RFR LOW via UART CR.  Pin stays in FUNC_1, line goes LOW. */
	msm_write(port,
		  msm_read(port, MSM_UART_MR1) & ~MSM_UART_MR1_RX_RDY_CTL,
		  MSM_UART_MR1);
	msm_write(port, MSM_UART_CR_CMD_SET_RFR, MSM_UART_CR);
	msleep(step + 2);	/* ~6 ms hold LOW */

	/* T+10ms: DEASSERT HIGH via pin_mux to GPIO (RFR+TX driven HIGH at 2 mA).
	 * The pinctrl gpio_txrts state should set these pins to OUT_HIGH per the
	 * device-tree binding. */
	pinctrl_select_state(msm_port->pinctrl, gpio_state);
	msleep(step + 1);	/* ~5 ms HIGH */

	/* T+19ms: pin_mux OFF — same gpio state, the legacy log shows two events
	 * here (off then on); for us a single pinctrl reselect plus a hold is the
	 * equivalent of the pin tri-state moment legacy creates. */
	msleep(step);

	/* T+23ms: pin_mux ON — back to UART function with RFR HIGH-stored. */
	pinctrl_select_state(msm_port->pinctrl, uart_state);
	msleep(step);

	/* T+26-33ms: deassert HIGH x3 — replicated as three short HIGH pulses. */
	msm_write(port, MSM_UART_CR_CMD_RESET_RFR, MSM_UART_CR);
	msleep(step);
	msm_write(port, MSM_UART_CR_CMD_RESET_RFR, MSM_UART_CR);
	msleep(step);
	msm_write(port, MSM_UART_CR_CMD_RESET_RFR, MSM_UART_CR);
	msleep(step);

	/* T+40ms: final ASSERT LOW. */
	msm_write(port, MSM_UART_CR_CMD_SET_RFR, MSM_UART_CR);

	mutex_unlock(&msm_bt_wake_lock);

	dev_info(port->dev,
		 "BT bring-up dance complete (step=%u ms, total ~%u ms)\n",
		 step, step * 10 + 2);
}
EXPORT_SYMBOL_GPL(msm_serial_bt_dance);

/*
 * Manually drive the BT UART RFR (host-side "ready for receiving") signal
 * via CR commands, WITHOUT enabling auto-RFR hardware mode. This mirrors
 * the legacy webOS msm_uart_dm.c __msm_uartdm_set_rx_flow() pattern (flow_ctl
 * = 0, flow_state = 1): clear MR1 RX_RDY_CTL (bit 7), then issue SET_RFR
 * to drive the line LOW or RESET_RFR to drive it HIGH.
 *
 * Why this exists: the BCSP serdev driver (hci_bcsp) previously called
 * serdev_device_set_tiocm(TIOCM_RTS) to assert RFR low. That routes through
 * msm_set_mctrl(), which sets MR1 RX_RDY_CTL = auto-RFR mode. Auto-RFR is
 * documented hardware behavior (deassert RFR when RX FIFO crosses
 * AUTO_RFR_LEVEL0) but it isn't what BCSP link-establishment wants — the
 * CSR chip expects RFR held statically LOW for the duration of the
 * handshake. Live rdmem capture from a working webOS boot confirms link-est
 * MR1 has bit 7 clear and RFR is driven via the CR command path, not via
 * auto-RFR. Our mainline live capture (`reports/bt-trace/
 * mainline_link_est_capture_*.txt`) showed MR1 = 0xb4 (bit 7 set) where
 * webOS would have 0x34 — this helper closes that gap.
 */
void msm_serial_bt_force_rfr(bool assert_low)
{
	struct uart_port *port = READ_ONCE(msm_bt_wake_port);
	unsigned long flags;
	unsigned int mr1;

	if (!port)
		return;

	uart_port_lock_irqsave(port, &flags);
	mr1 = msm_read(port, MSM_UART_MR1);
	mr1 &= ~MSM_UART_MR1_RX_RDY_CTL;
	msm_write(port, mr1, MSM_UART_MR1);
	msm_write(port,
		  assert_low ? MSM_UART_CR_CMD_SET_RFR : MSM_UART_CR_CMD_RESET_RFR,
		  MSM_UART_CR);
	uart_port_unlock_irqrestore(port, flags);

	dev_info(port->dev, "BT RFR forced %s (manual, no auto-RFR)\n",
		 assert_low ? "LOW (asserted)" : "HIGH (deasserted)");
}
EXPORT_SYMBOL_GPL(msm_serial_bt_force_rfr);

/*
 * Drive a BREAK condition on the BT UART TX line for N microseconds, then
 * release.  Used by the BCSP serdev driver as an experimental "wake-from-
 * deep-sleep" gesture for CSR BlueCore chips per CSR app-notes: a BREAK
 * pulse on the host TX line is one documented way to nudge the chip's
 * RX UART out of a power-gated state without toggling BT_WAKE.  Process
 * context only (sleeps).  No-op until the BT port has probed.
 */
void msm_serial_bt_send_break(unsigned int us)
{
	struct uart_port *port = READ_ONCE(msm_bt_wake_port);
	unsigned long flags;

	if (!port || !us)
		return;

	uart_port_lock_irqsave(port, &flags);
	msm_write(port, MSM_UART_CR_CMD_START_BREAK, MSM_UART_CR);
	uart_port_unlock_irqrestore(port, flags);

	/* BREAK held outside the spinlock — usleep_range can sleep. */
	usleep_range(us, us + 200);

	uart_port_lock_irqsave(port, &flags);
	msm_write(port, MSM_UART_CR_CMD_STOP_BREAK, MSM_UART_CR);
	uart_port_unlock_irqrestore(port, flags);

	dev_info(port->dev, "BT BREAK pulsed for %u us\n", us);
}
EXPORT_SYMBOL_GPL(msm_serial_bt_send_break);

static void msm_bt_wake_work(struct work_struct *w)
{
	struct msm_port *msm_port =
		container_of(to_delayed_work(w), struct msm_port, bt_wake_work);
	struct uart_port *port = &msm_port->uart;

	if (!msm_port->bt_linkest || bt_wake_period_ms <= 0)
		return;

	/* Only glitch when TX is idle, so we never corrupt a frame in flight. */
	if (msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_EMPTY)
		msm_bt_wake_glitch(port);

	schedule_delayed_work(&msm_port->bt_wake_work,
			      msecs_to_jiffies(bt_wake_period_ms));
}

static int msm_startup(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	unsigned int data, rfr_level, mask;
	int ret;

	snprintf(msm_port->name, sizeof(msm_port->name),
		 "msm_serial%d", port->line);

	msm_init_clock(port);

	/*
	 * webOS btuart_pin_mux "dance": briefly flip the UART pins to a GPIO
	 * state (TX/RTS driven high, 2 mA) and back to the UART function, once
	 * at port-open before any TX. On the TouchPad this is what wakes the
	 * CSR BlueCore BT chip's power-gated UART RX so it will accept our
	 * (byte-perfect) SYNC_RSP. No-op unless qcom,startup-mux-glitch + a
	 * "gpio" pinctrl state are present in DT.
	 */
	if (msm_port->startup_mux_glitch && msm_port->pinctrl) {
		msm_bt_wake_glitch(port);
		dev_info(port->dev, "startup pin-mux glitch applied (BT wake)\n");

		/*
		 * Keep re-arming the wake glitch through BCSP link establishment
		 * (H2). Cleared when we switch to the operational baud (>115200)
		 * in msm_set_termios, or at shutdown.
		 */
		if (msm_port->bt_is_bt_uart && bt_wake_period_ms > 0) {
			msm_port->bt_linkest = true;
			schedule_delayed_work(&msm_port->bt_wake_work,
					      msecs_to_jiffies(bt_wake_period_ms));
		}
	}

	/* BT diag: dump the BT UART pad config to diff against webOS (8 mA, no pull). */
	if (msm_port->bt_is_bt_uart && bt_diag) {
		msm_bt_diag_dump_pads(port, "startup");
		/*
		 * Ensure the IrDA encoder is OFF. UARTDM IrDA would pulse the
		 * line at 3/16 bit-width instead of standard NRZ levels; internal
		 * loopback would still pass (RX decodes it) but the CSR chip on
		 * the wire would see garbage. MSM_UART_IRDA (0x38) is write-only
		 * (reads return RX_TOTAL_SNAP), so enforce 0 rather than read-check.
		 */
		msm_write(port, 0, MSM_UART_IRDA);
		dev_info(port->dev, "BT-DIAG: IRDA encoder forced off (NRZ levels)\n");
	}

	if (likely(port->fifosize > 12))
		rfr_level = port->fifosize - 12;
	else
		rfr_level = port->fifosize;

	/* set automatic RFR level */
	data = msm_read(port, MSM_UART_MR1);

	if (msm_port->is_uartdm)
		mask = MSM_UART_DM_MR1_AUTO_RFR_LEVEL1;
	else
		mask = MSM_UART_MR1_AUTO_RFR_LEVEL1;

	data &= ~mask;
	data &= ~MSM_UART_MR1_AUTO_RFR_LEVEL0;
	data |= mask & (rfr_level << 2);
	data |= MSM_UART_MR1_AUTO_RFR_LEVEL0 & rfr_level;
	msm_write(port, data, MSM_UART_MR1);

	if (msm_port->is_uartdm) {
		msm_request_tx_dma(msm_port, msm_port->uart.mapbase);
		msm_request_rx_dma(msm_port, msm_port->uart.mapbase);
	}

	ret = request_irq(port->irq, msm_uart_irq, IRQF_TRIGGER_HIGH,
			  msm_port->name, port);
	if (unlikely(ret))
		goto err_irq;

	return 0;

err_irq:
	if (msm_port->is_uartdm)
		msm_release_dma(msm_port);

	clk_disable_unprepare(msm_port->pclk);
	clk_disable_unprepare(msm_port->clk);
	dev_pm_opp_set_rate(port->dev, 0);

	return ret;
}

static void msm_shutdown(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);

	/* TouchPad BT (H2): stop the link-est wake glitch (process context). */
	msm_port->bt_linkest = false;
	cancel_delayed_work_sync(&msm_port->bt_wake_work);

	msm_port->imr = 0;
	msm_write(port, 0, MSM_UART_IMR); /* disable interrupts */

	if (msm_port->is_uartdm)
		msm_release_dma(msm_port);

	clk_disable_unprepare(msm_port->clk);
	dev_pm_opp_set_rate(port->dev, 0);

	free_irq(port->irq, port);
}

static void msm_set_termios(struct uart_port *port, struct ktermios *termios,
			    const struct ktermios *old)
{
	struct msm_port *msm_port = to_msm_port(port);
	struct msm_dma *dma = &msm_port->rx_dma;
	unsigned long flags;
	unsigned int baud, mr;

	uart_port_lock_irqsave(port, &flags);

	if (dma->chan) /* Terminate if any */
		msm_stop_dma(port, dma);

	/* calculate and set baud rate */
	baud = uart_get_baud_rate(port, termios, old, 300, 4000000);
	baud = msm_set_baud_rate(port, baud, &flags);
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

	/*
	 * TouchPad BT (H2): once we leave the 115200 link-est baud the BCSP
	 * link is up, so stop re-arming the wake glitch. cancel_delayed_work()
	 * (async) is safe under the port lock held here.
	 */
	if (msm_port->bt_is_bt_uart && baud > 115200 && msm_port->bt_linkest) {
		msm_port->bt_linkest = false;
		cancel_delayed_work(&msm_port->bt_wake_work);
	}

	/* calculate parity */
	mr = msm_read(port, MSM_UART_MR2);
	mr &= ~MSM_UART_MR2_PARITY_MODE;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			mr |= MSM_UART_MR2_PARITY_MODE_ODD;
		else if (termios->c_cflag & CMSPAR)
			mr |= MSM_UART_MR2_PARITY_MODE_SPACE;
		else
			mr |= MSM_UART_MR2_PARITY_MODE_EVEN;
	}

	/*
	 * BT diag (bt_force_8e1): force even parity on the BT UART regardless of
	 * the requested termios, to test the HP "BCSP uses 8E1" claim on-device.
	 * Our extracted PSKEY uartConfigBcsp=0x082e DOES set bit2 (even parity) —
	 * but that applies only to the post-warm-reset 3.6864M steady-state link;
	 * the 115200 ROM-default link-establishment phase is 8N1 (the chip SYNCs
	 * to us cleanly at 8N1, which is impossible if it required parity we omit).
	 * Expect forcing 8E1 here to LOSE the chip's SYNC (RX parity/framing
	 * errors) -> a live on-device demonstration that LE is 8N1, not 8E1.
	 */
	if (msm_port->bt_is_bt_uart && bt_force_8e1) {
		mr &= ~MSM_UART_MR2_PARITY_MODE;
		mr |= MSM_UART_MR2_PARITY_MODE_EVEN;
		dev_info(port->dev,
			 "BT-DIAG: forcing EVEN parity (8E1) on BT UART (diag)\n");
	}

	/* calculate bits per char */
	mr &= ~MSM_UART_MR2_BITS_PER_CHAR;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mr |= MSM_UART_MR2_BITS_PER_CHAR_5;
		break;
	case CS6:
		mr |= MSM_UART_MR2_BITS_PER_CHAR_6;
		break;
	case CS7:
		mr |= MSM_UART_MR2_BITS_PER_CHAR_7;
		break;
	case CS8:
	default:
		mr |= MSM_UART_MR2_BITS_PER_CHAR_8;
		break;
	}

	/* calculate stop bits */
	mr &= ~(MSM_UART_MR2_STOP_BIT_LEN_ONE | MSM_UART_MR2_STOP_BIT_LEN_TWO);
	if (termios->c_cflag & CSTOPB)
		mr |= MSM_UART_MR2_STOP_BIT_LEN_TWO;
	else
		mr |= MSM_UART_MR2_STOP_BIT_LEN_ONE;

	/* set parity, bits per char, and stop bit */
	msm_write(port, mr, MSM_UART_MR2);

	/* calculate and set hardware flow control */
	mr = msm_read(port, MSM_UART_MR1);
	mr &= ~(MSM_UART_MR1_CTS_CTL | MSM_UART_MR1_RX_RDY_CTL);
	if (termios->c_cflag & CRTSCTS) {
		mr |= MSM_UART_MR1_CTS_CTL;
		mr |= MSM_UART_MR1_RX_RDY_CTL;
	}
	msm_write(port, mr, MSM_UART_MR1);

	/* Configure status bits to ignore based on termio flags. */
	port->read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= MSM_UART_SR_PAR_FRAME_ERR;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= MSM_UART_SR_RX_BREAK;

	uart_update_timeout(port, termios->c_cflag, baud);

	/* Try to use DMA */
	msm_start_rx_dma(msm_port);

	uart_port_unlock_irqrestore(port, flags);
}

static const char *msm_type(struct uart_port *port)
{
	return "MSM";
}

static void msm_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	resource_size_t size;

	uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!uart_resource))
		return;
	size = resource_size(uart_resource);

	release_mem_region(port->mapbase, size);
	iounmap(port->membase);
	port->membase = NULL;
}

static int msm_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	resource_size_t size;
	int ret;

	uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!uart_resource))
		return -ENXIO;

	size = resource_size(uart_resource);

	if (!request_mem_region(port->mapbase, size, "msm_serial"))
		return -EBUSY;

	port->membase = ioremap(port->mapbase, size);
	if (!port->membase) {
		ret = -EBUSY;
		goto fail_release_port;
	}

	return 0;

fail_release_port:
	release_mem_region(port->mapbase, size);
	return ret;
}

static void msm_config_port(struct uart_port *port, int flags)
{
	int ret;

	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_MSM;
		ret = msm_request_port(port);
		if (ret)
			return;
	}
}

static int msm_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_MSM))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

static void msm_power(struct uart_port *port, unsigned int state,
		      unsigned int oldstate)
{
	struct msm_port *msm_port = to_msm_port(port);

	switch (state) {
	case 0:
		dev_pm_opp_set_rate(port->dev, port->uartclk);
		clk_prepare_enable(msm_port->clk);
		clk_prepare_enable(msm_port->pclk);
		break;
	case 3:
		clk_disable_unprepare(msm_port->clk);
		dev_pm_opp_set_rate(port->dev, 0);
		clk_disable_unprepare(msm_port->pclk);
		break;
	default:
		pr_err("msm_serial: Unknown PM state %d\n", state);
	}
}

#ifdef CONFIG_CONSOLE_POLL
static int msm_poll_get_char_single(struct uart_port *port)
{
	struct msm_port *msm_port = to_msm_port(port);
	unsigned int rf_reg = msm_port->is_uartdm ? UARTDM_RF : MSM_UART_RF;

	if (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_RX_READY))
		return NO_POLL_CHAR;

	return msm_read(port, rf_reg) & 0xff;
}

static int msm_poll_get_char_dm(struct uart_port *port)
{
	int c;
	static u32 slop;
	static int count;
	unsigned char *sp = (unsigned char *)&slop;

	/* Check if a previous read had more than one char */
	if (count) {
		c = sp[sizeof(slop) - count];
		count--;
	/* Or if FIFO is empty */
	} else if (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_RX_READY)) {
		/*
		 * If RX packing buffer has less than a word, force stale to
		 * push contents into RX FIFO
		 */
		count = msm_read(port, UARTDM_RXFS);
		count = (count >> UARTDM_RXFS_BUF_SHIFT) & UARTDM_RXFS_BUF_MASK;
		if (count) {
			msm_write(port, MSM_UART_CR_CMD_FORCE_STALE, MSM_UART_CR);
			slop = msm_read(port, UARTDM_RF);
			c = sp[0];
			count--;
			msm_write(port, MSM_UART_CR_CMD_RESET_STALE_INT, MSM_UART_CR);
			msm_write(port, 0xFFFFFF, UARTDM_DMRX);
			msm_write(port, MSM_UART_CR_CMD_STALE_EVENT_ENABLE, MSM_UART_CR);
		} else {
			c = NO_POLL_CHAR;
		}
	/* FIFO has a word */
	} else {
		slop = msm_read(port, UARTDM_RF);
		c = sp[0];
		count = sizeof(slop) - 1;
	}

	return c;
}

static int msm_poll_get_char(struct uart_port *port)
{
	u32 imr;
	int c;
	struct msm_port *msm_port = to_msm_port(port);

	/* Disable all interrupts */
	imr = msm_read(port, MSM_UART_IMR);
	msm_write(port, 0, MSM_UART_IMR);

	if (msm_port->is_uartdm)
		c = msm_poll_get_char_dm(port);
	else
		c = msm_poll_get_char_single(port);

	/* Enable interrupts */
	msm_write_imr(port, imr);

	return c;
}

static void msm_poll_put_char(struct uart_port *port, unsigned char c)
{
	u32 imr;
	struct msm_port *msm_port = to_msm_port(port);

	/* Disable all interrupts */
	imr = msm_read(port, MSM_UART_IMR);
	msm_write(port, 0, MSM_UART_IMR);

	if (msm_port->is_uartdm)
		msm_reset_dm_count(port, 1);

	/* Wait until FIFO is empty */
	while (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_READY))
		cpu_relax();

	/* Write a character */
	msm_write(port, c, msm_port->is_uartdm ? UARTDM_TF : MSM_UART_TF);

	/* Wait until FIFO is empty */
	while (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_READY))
		cpu_relax();

	/* Enable interrupts */
	msm_write_imr(port, imr);
}
#endif

static const struct uart_ops msm_uart_pops = {
	.tx_empty = msm_tx_empty,
	.set_mctrl = msm_set_mctrl,
	.get_mctrl = msm_get_mctrl,
	.stop_tx = msm_stop_tx,
	.start_tx = msm_start_tx,
	.stop_rx = msm_stop_rx,
	.enable_ms = msm_enable_ms,
	.break_ctl = msm_break_ctl,
	.startup = msm_startup,
	.shutdown = msm_shutdown,
	.set_termios = msm_set_termios,
	.type = msm_type,
	.release_port = msm_release_port,
	.request_port = msm_request_port,
	.config_port = msm_config_port,
	.verify_port = msm_verify_port,
	.pm = msm_power,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= msm_poll_get_char,
	.poll_put_char	= msm_poll_put_char,
#endif
};

static struct msm_port msm_uart_ports[] = {
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 0,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 1,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 2,
		},
	},
};

#define MSM_UART_NR	ARRAY_SIZE(msm_uart_ports)

static inline struct uart_port *msm_get_port_from_line(unsigned int line)
{
	return &msm_uart_ports[line].uart;
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static void __msm_console_write(struct uart_port *port, const char *s,
				unsigned int count, bool is_uartdm)
{
	unsigned long flags;
	int i;
	int num_newlines = 0;
	bool replaced = false;
	void __iomem *tf;
	int locked = 1;

	if (is_uartdm)
		tf = port->membase + UARTDM_TF;
	else
		tf = port->membase + MSM_UART_TF;

	/* Account for newlines that will get a carriage return added */
	for (i = 0; i < count; i++)
		if (s[i] == '\n')
			num_newlines++;
	count += num_newlines;

	if (oops_in_progress)
		locked = uart_port_trylock_irqsave(port, &flags);
	else
		uart_port_lock_irqsave(port, &flags);

	if (is_uartdm)
		msm_reset_dm_count(port, count);

	i = 0;
	while (i < count) {
		int j;
		unsigned int num_chars;
		char buf[4] = { 0 };

		if (is_uartdm)
			num_chars = min(count - i, (unsigned int)sizeof(buf));
		else
			num_chars = 1;

		for (j = 0; j < num_chars; j++) {
			char c = *s;

			if (c == '\n' && !replaced) {
				buf[j] = '\r';
				j++;
				replaced = true;
			}
			if (j < num_chars) {
				buf[j] = c;
				s++;
				replaced = false;
			}
		}

		while (!(msm_read(port, MSM_UART_SR) & MSM_UART_SR_TX_READY))
			cpu_relax();

		iowrite32_rep(tf, buf, 1);
		i += num_chars;
	}

	if (locked)
		uart_port_unlock_irqrestore(port, flags);
}

static void msm_console_write(struct console *co, const char *s,
			      unsigned int count)
{
	struct uart_port *port;
	struct msm_port *msm_port;

	BUG_ON(co->index < 0 || co->index >= MSM_UART_NR);

	port = msm_get_port_from_line(co->index);
	msm_port = to_msm_port(port);

	__msm_console_write(port, s, count, msm_port->is_uartdm);
}

static int msm_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (unlikely(co->index >= MSM_UART_NR || co->index < 0))
		return -ENXIO;

	port = msm_get_port_from_line(co->index);

	if (unlikely(!port->membase))
		return -ENXIO;

	msm_init_clock(port);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	pr_info("msm_serial: console setup on port #%d\n", port->line);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static void
msm_serial_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	__msm_console_write(&dev->port, s, n, false);
}

static int __init
msm_serial_early_console_setup(struct earlycon_device *device, const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = msm_serial_early_write;
	return 0;
}
OF_EARLYCON_DECLARE(msm_serial, "qcom,msm-uart",
		    msm_serial_early_console_setup);

static void
msm_serial_early_write_dm(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	__msm_console_write(&dev->port, s, n, true);
}

static int __init
msm_serial_early_console_setup_dm(struct earlycon_device *device,
				  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	/* Disable DM / single-character modes */
	msm_write(&device->port, 0, UARTDM_DMEN);
	msm_write(&device->port, MSM_UART_CR_CMD_RESET_RX, MSM_UART_CR);
	msm_write(&device->port, MSM_UART_CR_CMD_RESET_TX, MSM_UART_CR);
	msm_write(&device->port, MSM_UART_CR_TX_ENABLE, MSM_UART_CR);

	device->con->write = msm_serial_early_write_dm;
	return 0;
}
OF_EARLYCON_DECLARE(msm_serial_dm, "qcom,msm-uartdm",
		    msm_serial_early_console_setup_dm);

static struct uart_driver msm_uart_driver;

static struct console msm_console = {
	.name = "ttyMSM",
	.write = msm_console_write,
	.device = uart_console_device,
	.setup = msm_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &msm_uart_driver,
};

#define MSM_CONSOLE	(&msm_console)

#else
#define MSM_CONSOLE	NULL
#endif

static struct uart_driver msm_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_serial",
	.dev_name = "ttyMSM",
	.nr = MSM_UART_NR,
	.cons = MSM_CONSOLE,
};

static atomic_t msm_uart_next_id = ATOMIC_INIT(0);

static const struct of_device_id msm_uartdm_table[] = {
	{ .compatible = "qcom,msm-uartdm-v1.1", .data = (void *)UARTDM_1P1 },
	{ .compatible = "qcom,msm-uartdm-v1.2", .data = (void *)UARTDM_1P2 },
	{ .compatible = "qcom,msm-uartdm-v1.3", .data = (void *)UARTDM_1P3 },
	{ .compatible = "qcom,msm-uartdm-v1.4", .data = (void *)UARTDM_1P4 },
	{ }
};

static int msm_serial_probe(struct platform_device *pdev)
{
	struct msm_port *msm_port;
	struct resource *resource;
	struct uart_port *port;
	const struct of_device_id *id;
	int irq, line, ret;

	if (pdev->dev.of_node)
		line = of_alias_get_id(pdev->dev.of_node, "serial");
	else
		line = pdev->id;

	if (line < 0)
		line = atomic_inc_return(&msm_uart_next_id) - 1;

	if (unlikely(line < 0 || line >= MSM_UART_NR))
		return -ENXIO;

	dev_info(&pdev->dev, "msm_serial: detected port #%d\n", line);

	port = msm_get_port_from_line(line);
	port->dev = &pdev->dev;
	msm_port = to_msm_port(port);

	id = of_match_device(msm_uartdm_table, &pdev->dev);
	if (id)
		msm_port->is_uartdm = (unsigned long)id->data;
	else
		msm_port->is_uartdm = 0;

	msm_port->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(msm_port->clk))
		return PTR_ERR(msm_port->clk);

	if (msm_port->is_uartdm) {
		msm_port->pclk = devm_clk_get(&pdev->dev, "iface");
		if (IS_ERR(msm_port->pclk))
			return PTR_ERR(msm_port->pclk);
	}

	ret = devm_pm_opp_set_clkname(&pdev->dev, "core");
	if (ret)
		return ret;

	/* OPP table is optional */
	ret = devm_pm_opp_of_add_table(&pdev->dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(&pdev->dev, ret, "invalid OPP table\n");

	port->uartclk = clk_get_rate(msm_port->clk);
	dev_info(&pdev->dev, "uartclk = %d\n", port->uartclk);

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return -ENXIO;
	port->mapbase = resource->start;

	irq = platform_get_irq(pdev, 0);
	if (unlikely(irq < 0))
		return -ENXIO;
	port->irq = irq;
	port->has_sysrq = IS_ENABLED(CONFIG_SERIAL_MSM_CONSOLE);

	/*
	 * Optional webOS-style startup pin-mux glitch (BT wake). Look up an
	 * explicit pinctrl handle with "default" and "gpio" states; the core
	 * already applies "default" at probe, this handle is only used to
	 * briefly select "gpio" and back in msm_startup(). All optional.
	 */
	msm_port->startup_mux_glitch =
		of_property_read_bool(pdev->dev.of_node, "qcom,startup-mux-glitch");
	if (msm_port->startup_mux_glitch) {
		msm_port->pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(msm_port->pinctrl)) {
			msm_port->pinctrl = NULL;
		} else {
			msm_port->pinctrl_default =
				pinctrl_lookup_state(msm_port->pinctrl, "default");
			msm_port->pinctrl_gpio =
				pinctrl_lookup_state(msm_port->pinctrl, "gpio");
			if (IS_ERR(msm_port->pinctrl_default) ||
			    IS_ERR(msm_port->pinctrl_gpio)) {
				dev_warn(&pdev->dev,
					 "startup-mux-glitch: missing default/gpio pinctrl state, disabled\n");
				msm_port->startup_mux_glitch = false;
			}
			/* RX-safe TX/RTS-only glitch state (optional). */
			msm_port->pinctrl_gpio_txrts =
				pinctrl_lookup_state(msm_port->pinctrl, "gpio-txrts");
			if (IS_ERR(msm_port->pinctrl_gpio_txrts))
				msm_port->pinctrl_gpio_txrts = NULL;
		}
	}

	/* TouchPad BT (H2): periodic wake-glitch re-arm during link establishment. */
	INIT_DELAYED_WORK(&msm_port->bt_wake_work, msm_bt_wake_work);
	msm_port->bt_is_bt_uart = (port->mapbase == MSM_BT_UART_MAPBASE);
	if (msm_port->bt_is_bt_uart)
		WRITE_ONCE(msm_bt_wake_port, port);

	platform_set_drvdata(pdev, port);

	return uart_add_one_port(&msm_uart_driver, port);
}

static void msm_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	uart_remove_one_port(&msm_uart_driver, port);
}

static const struct of_device_id msm_match_table[] = {
	{ .compatible = "qcom,msm-uart" },
	{ .compatible = "qcom,msm-uartdm" },
	{}
};
MODULE_DEVICE_TABLE(of, msm_match_table);

static int __maybe_unused msm_serial_suspend(struct device *dev)
{
	struct msm_port *port = dev_get_drvdata(dev);

	uart_suspend_port(&msm_uart_driver, &port->uart);

	return 0;
}

static int __maybe_unused msm_serial_resume(struct device *dev)
{
	struct msm_port *port = dev_get_drvdata(dev);

	uart_resume_port(&msm_uart_driver, &port->uart);

	return 0;
}

static const struct dev_pm_ops msm_serial_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_serial_suspend, msm_serial_resume)
};

static struct platform_driver msm_platform_driver = {
	.remove = msm_serial_remove,
	.probe = msm_serial_probe,
	.driver = {
		.name = "msm_serial",
		.pm = &msm_serial_dev_pm_ops,
		.of_match_table = msm_match_table,
	},
};

static int __init msm_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&msm_uart_driver);
	if (unlikely(ret))
		return ret;

	ret = platform_driver_register(&msm_platform_driver);
	if (unlikely(ret))
		uart_unregister_driver(&msm_uart_driver);

	pr_info("msm_serial: driver initialized\n");

	return ret;
}

static void __exit msm_serial_exit(void)
{
	platform_driver_unregister(&msm_platform_driver);
	uart_unregister_driver(&msm_uart_driver);
}

module_init(msm_serial_init);
module_exit(msm_serial_exit);

MODULE_AUTHOR("Robert Love <rlove@google.com>");
MODULE_DESCRIPTION("Driver for msm7x serial device");
MODULE_LICENSE("GPL");
