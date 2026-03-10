// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  Bluetooth HCI UART driver for BCSP protocol
 *
 *  Copyright (C) 2002-2003  Fabrizio Gennari <fabrizio.gennari@philips.com>
 *  Copyright (C) 2004-2005  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2024       HP TouchPad serdev support
 *
 *  This driver supports both line discipline and serdev modes:
 *  - Line discipline: Used with hciattach for legacy compatibility
 *  - Serdev: Direct device tree integration with GPIO power control
 *
 *  For HP TouchPad (BCM4329), serdev mode enables proper two-phase
 *  initialization: PSKEYs + WARM_RESET, then reconnect with configured chip.
 */

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <linux/bitrev.h>
#include <linux/unaligned.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

static bool txcrc = true;
static bool hciextn = true;
static char *bdaddr;
static bool skip_pskeys;  /* Skip PSKEY/WARM_RESET for debugging */

#define BCSP_TXWINSIZE	4

#define BCSP_ACK_PKT	0x05
#define BCSP_LE_PKT	0x06
#define BCSP_BCCMD_PKT	0x07

/* BCCMD constants */
#define BCCMD_SETREQ		0x0002
#define BCCMD_VARID_PS		0x7003
#define BCCMD_VARID_WARM_RESET	0x4002

/*
 * PSKEY definitions for CSR/BCM chip configuration
 *
 * Based on webOS libPmBtBsaif.so analysis:
 * - All PSKEYs use stores=8 (PSRAM/volatile), not stores=4 (PSI/persistent)
 * - PSKEYs are applied via BCCMD SETREQ (0x0002) with VARID_PS (0x7003)
 * - WARM_RESET (0x4002) must be sent after PSKEYs to apply config
 */

/* Common PSKEYs (sent for all BlueCore devices) */
#define PSKEY_BDADDR			0x0001	/* Bluetooth device address */
#define PSKEY_ENC_KEY_LMIN		0x000E	/* Min encryption key length */
#define PSKEY_ANA_FREQ			0x0011	/* Crystal frequency (26 MHz = 26000) */
#define PSKEY_ANA_FTRIM			0x0013	/* Crystal fine trim (0x19) */
#define PSKEY_LC_MAX_TX_POWER		0x0017	/* Maximum TX power level */
#define PSKEY_LC_DEFAULT_TX_POWER	0x0021	/* Default TX power level */
#define PSKEY_TX_POWER_LEVEL		0x0031	/* TX power level table */
#define PSKEY_H_HC_FC_MAX_ACL		0x01AB	/* HCI FC max ACL packets */
#define PSKEY_H_HC_FC_MAX_SCO		0x01B0	/* HCI FC max SCO packets */
#define PSKEY_PCM_SAMPLE_SIZE		0x01B9	/* PCM sample size / UART config */
#define PSKEY_PCM_MIN_CPU_CLOCK		0x01BE	/* Deep sleep config */
#define PSKEY_BAUDRATE_CONFIG		0x01F6	/* UART baudrate */
#define PSKEY_UNKNOWN_01F9		0x01F9	/* Unknown (0x0001) */
#define PSKEY_HOST_INTERFACE		0x01FE	/* Host interface config */
#define PSKEY_LC_MAX_TX_POWER_NO_RSSI	0x024D	/* Max TX power without RSSI */
#define PSKEY_LC_DEFAULT_TX_POWER_NO_RSSI 0x025D /* Default TX power without RSSI */

/* Palm Platform PSKEYs (12 entries from webOS) */
#define PSKEY_PALM_01B3			0x01B3	/* Palm config (4 words) */
#define PSKEY_PALM_01B6			0x01B6	/* Palm config (2 words) */
#define PSKEY_PALM_01BF			0x01BF	/* Palm config (2 words) */
#define PSKEY_PALM_01BA			0x01BA	/* Palm config (4 words) */
#define PSKEY_PALM_01C7			0x01C7	/* Palm config (8 words) */
#define PSKEY_PALM_01CA			0x01CA	/* Palm config (2 words) */
#define PSKEY_PALM_001D			0x001D	/* Palm config (2 words) */

/*
 * TouchPad-Specific PSKEYs (LMP subversion 0x12E9)
 * These contain RF calibration data extracted from webOS libPmBtBsaif.so
 */
#define PSKEY_TOUCHPAD_00F6		0x00F6	/* Configuration (1 word) */
#define PSKEY_TOUCHPAD_0203		0x0203	/* Configuration (32 words) */
#define PSKEY_TOUCHPAD_0394		0x0394	/* Configuration (1 word) */
#define PSKEY_TOUCHPAD_03AA		0x03AA	/* Configuration (12 words) */
#define PSKEY_TOUCHPAD_03AB		0x03AB	/* Configuration (12 words) */
#define PSKEY_TOUCHPAD_03D4		0x03D4	/* Configuration (1 word) */
#define PSKEY_TOUCHPAD_212C		0x212C	/* RF calibration (20 words) */
#define PSKEY_TOUCHPAD_212D		0x212D	/* RF calibration (16 words) */
#define PSKEY_TOUCHPAD_212E		0x212E	/* RF calibration (34 words) */
#define PSKEY_TOUCHPAD_212F		0x212F	/* RF calibration (46 words) */
#define PSKEY_TOUCHPAD_2130		0x2130	/* RF calibration (15 words) */
#define PSKEY_TOUCHPAD_2131		0x2131	/* RF calibration (21 words) */
#define PSKEY_TOUCHPAD_2132		0x2132	/* RF calibration (28 words) */
#define PSKEY_TOUCHPAD_2133		0x2133	/* RF calibration (39 words) */
#define PSKEY_TOUCHPAD_2134		0x2134	/* RF calibration (40 words) */
#define PSKEY_TOUCHPAD_2135		0x2135	/* RF calibration (24 words) */
#define PSKEY_TOUCHPAD_2136		0x2136	/* RF calibration (31 words) */
#define PSKEY_TOUCHPAD_2137		0x2137	/* RF calibration (29 words) */
#define PSKEY_TOUCHPAD_2138		0x2138	/* RF calibration (13 words) */
#define PSKEY_TOUCHPAD_2139		0x2139	/* RF calibration (43 words) */
#define PSKEY_TOUCHPAD_213A		0x213A	/* RF calibration (4 words) */
#define PSKEY_TOUCHPAD_21E1		0x21E1	/* RF calibration (16 words) */
#define PSKEY_TOUCHPAD_2215		0x2215	/* RF calibration (18 words) */
#define PSKEY_TOUCHPAD_2216		0x2216	/* RF calibration (28 words) */
#define PSKEY_TOUCHPAD_2227		0x2227	/* RF calibration (63 words) */
#define PSKEY_TOUCHPAD_2228		0x2228	/* RF calibration (62 words) */
#define PSKEY_TOUCHPAD_2229		0x2229	/* RF calibration (29 words) */
#define PSKEY_TOUCHPAD_222A		0x222A	/* RF calibration (58 words) */
#define PSKEY_TOUCHPAD_222B		0x222B	/* RF calibration (26 words) */

/* Number of TouchPad-specific PSKEYs */
#define TOUCHPAD_PSKEY_COUNT		29

/* PSKEY storage types */
#define PSKEY_STORES_DEFAULT		0x00	/* Default store */
#define PSKEY_STORES_PSI		0x04	/* Persistent Store Implementation */
#define PSKEY_STORES_PSRAM		0x08	/* RAM (volatile) - webOS uses this */

/* Maximum size of TX power table from device tree (in u16 words) */
#define BCSP_TX_POWER_TABLE_MAX		128

/*
 * Palm Platform PSKEY Data (extracted from webOS PmBtStack)
 * These are the exact values sent by webOS for BCM4329 on HP TouchPad.
 */

/* palmPlatformCommonPskeys */
static const u16 palm_pskey_01b3[] = { 0x08a0, 0x0016, 0x0060, 0x082e };
static const u16 palm_pskey_01b6[] = { 0x0060, 0x082e };
static const u16 palm_pskey_01bf[] = { 0x082e, 0x0000 };

/* palmPlatformSpecificPskeys */
static const u16 palm_pskey_01ba[] = { 0x1002, 0x0177, 0x0001, 0x03e8 };
static const u16 palm_pskey_01c7[] = { 0x0001, 0x03e8, 0x000a, 0x0064,
				       0x0031, 0x0004, 0x0004, 0x2200 };
static const u16 palm_pskey_01ca[] = { 0x0031, 0x0004 };
static const u16 palm_pskey_001d[] = { 0x2410, 0x08a0 };

/* TX Power Table (PSKEY 0x0031) - 30 entries for RF calibration */
static const u16 palm_tx_power_table[] = {
	0x2200, 0x0050, 0x2600, 0x0050, 0xf000,  /* Entry 0 */
	0x2800, 0x0050, 0x2d00, 0x0050, 0xf400,  /* Entry 1 */
	0x2500, 0x0040, 0x2a00, 0x0040, 0xf800,  /* Entry 2 */
	0x2200, 0x0020, 0x2700, 0x0020, 0xfc00,  /* Entry 3 */
	0x2600, 0x0010, 0x2c00, 0x0010, 0x0000,  /* Entry 4 */
	0x2c00, 0x0000, 0x3a00, 0x0000, 0x0400   /* Entry 5 */
};

/*
 * Serdev device structure for BCM4329 with BCSP protocol
 *
 * This structure is used when the driver is instantiated via device tree
 * (serdev path) rather than via hciattach (line discipline path).
 *
 * Serdev mode provides:
 * - Direct GPIO control for power management
 * - Proper two-phase initialization (PSKEYs + WARM_RESET + reconnect)
 * - No dependency on hciattach
 */
struct bcsp_serdev {
	/* Must be first member - hci_serdev.c expects this */
	struct hci_uart		serdev_hu;

	struct device		*dev;

	/* GPIO descriptors for power control */
	struct gpio_desc	*shutdown_gpio;	/* BT_REG_ON / BT_POWER */
	struct gpio_desc	*device_wakeup;	/* BT_WAKE */
	struct gpio_desc	*reset_gpio;	/* BT_RST_N (active low) */

	/* Initialization state */
	enum {
		BCSP_SERDEV_INIT_POWER_OFF,	/* Chip powered off */
		BCSP_SERDEV_INIT_PHASE1,	/* Sending PSKEYs + WARM_RESET */
		BCSP_SERDEV_INIT_RECONNECTING,	/* Reconnecting after reset */
		BCSP_SERDEV_INIT_DONE		/* Initialization complete */
	} init_state;

	/* UART settings */
	u32			init_speed;
	u32			oper_speed;
};

struct bcsp_struct {
	struct sk_buff_head unack;	/* Unack'ed packets queue */
	struct sk_buff_head rel;	/* Reliable packets queue */
	struct sk_buff_head unrel;	/* Unreliable packets queue */

	unsigned long rx_count;
	struct	sk_buff *rx_skb;
	u8	rxseq_txack;		/* rxseq == txack. */
	u8	rxack;			/* Last packet sent by us that the peer ack'ed */
	struct	timer_list tbcsp;
	struct	hci_uart *hu;

	enum {
		BCSP_W4_PKT_DELIMITER,
		BCSP_W4_PKT_START,
		BCSP_W4_BCSP_HDR,
		BCSP_W4_DATA,
		BCSP_W4_CRC
	} rx_state;

	enum {
		BCSP_ESCSTATE_NOESC,
		BCSP_ESCSTATE_ESC
	} rx_esc_state;

	/* Link establishment state machine */
	enum {
		BCSP_LINK_UNINIT,	/* Initial: sending sync, waiting for sync_rsp */
		BCSP_LINK_INIT,		/* Got sync_rsp: sending conf, waiting for conf_rsp */
		BCSP_LINK_ACTIVE	/* Got conf_rsp: link established */
	} link_state;

	u8	use_crc;
	u16	message_crc;
	u8	txack_req;		/* Do we need to send ack's to the peer? */

	/* Reliable packet sequence number - used to assign seq to each rel pkt. */
	u8	msgq_txseq;

	/* BD address configuration state machine */
	enum {
		BCSP_BDADDR_NONE,	/* No BD address to configure */
		BCSP_BDADDR_PENDING,	/* Waiting for first link establishment */
		BCSP_BDADDR_SENT,	/* BCCMD sent, waiting for chip reset */
		BCSP_BDADDR_DONE	/* Configuration complete */
	} bdaddr_state;
	bdaddr_t bdaddr;		/* BD address to set */
	bool	bdaddr_from_dt;		/* BD address was read from device tree */

	/* TX power table from device tree (for RF calibration) */
	u16	*tx_power_table;	/* Dynamically allocated from DT */
	int	tx_power_table_len;	/* Length in u16 words, 0 if not available */

	/*
	 * PSKEYs from device tree (with Palm defaults as fallback)
	 * All values match webOS PmBtStack for HP TouchPad BCM4329
	 */
	bool	pskeys_from_dt;		/* True if PSKEYs loaded from DT */

	/* Common PSKEYs */
	u16	pskey_ana_freq;		/* 0x0011: Crystal frequency (default: 26000) */
	u16	pskey_ana_ftrim;	/* 0x0013: Crystal fine trim (default: 0x19) */
	u16	pskey_host_interface;	/* 0x01FE: Host interface (default: 0x0001) */
	u16	pskey_deep_sleep;	/* 0x01BE: Deep sleep config (default: 0x0000) */
	u16	pskey_hci_max_acl;	/* 0x01AB: HCI FC max ACL (default: 0x03ec) */
	u16	pskey_hci_max_sco;	/* 0x01B0: HCI FC max SCO (default: 0x0000) */
	u16	pskey_uart_baudrate;	/* 0x01B9: UART baudrate (default: 0x01d8) */
	u16	pskey_enc_key_min;	/* 0x000E: Enc key min len (default: 0x0001) */
	u16	pskey_unknown_01f9;	/* 0x01F9: Unknown (default: 0x0001) */
	u16	pskey_max_tx_power_no_rssi;	/* 0x024D (default: 0x0004) */
	u16	pskey_default_tx_power_no_rssi;	/* 0x025D (default: 0x0001) */
	u16	pskey_max_tx_power;	/* 0x0017: Max TX power (default: 0x0004) */
	u16	pskey_default_tx_power;	/* 0x0021: Default TX power (default: 0x0004) */

	/* Palm Platform PSKEYs (multi-word, stored as pointers) */
	const u16 *pskey_palm_01b3;	/* 4 words */
	int	pskey_palm_01b3_len;
	const u16 *pskey_palm_01b6;	/* 2 words */
	int	pskey_palm_01b6_len;
	const u16 *pskey_palm_01bf;	/* 2 words */
	int	pskey_palm_01bf_len;
	const u16 *pskey_palm_01ba;	/* 4 words */
	int	pskey_palm_01ba_len;
	const u16 *pskey_palm_01c7;	/* 8 words */
	int	pskey_palm_01c7_len;
	const u16 *pskey_palm_01ca;	/* 2 words */
	int	pskey_palm_01ca_len;
	const u16 *pskey_palm_001d;	/* 2 words */
	int	pskey_palm_001d_len;

	/*
	 * TouchPad-Specific PSKEYs (LMP subversion 0x12E9)
	 * Loaded from device tree, NULL if not present.
	 * Contains RF calibration data for optimal Bluetooth performance.
	 */
	bool	touchpad_pskeys_present;  /* True if TouchPad PSKEYs loaded */
	u16	*touchpad_pskey_00f6;
	int	touchpad_pskey_00f6_len;
	u16	*touchpad_pskey_0203;
	int	touchpad_pskey_0203_len;
	u16	*touchpad_pskey_0394;
	int	touchpad_pskey_0394_len;
	u16	*touchpad_pskey_03aa;
	int	touchpad_pskey_03aa_len;
	u16	*touchpad_pskey_03ab;
	int	touchpad_pskey_03ab_len;
	u16	*touchpad_pskey_03d4;
	int	touchpad_pskey_03d4_len;
	u16	*touchpad_pskey_212c;
	int	touchpad_pskey_212c_len;
	u16	*touchpad_pskey_212d;
	int	touchpad_pskey_212d_len;
	u16	*touchpad_pskey_212e;
	int	touchpad_pskey_212e_len;
	u16	*touchpad_pskey_212f;
	int	touchpad_pskey_212f_len;
	u16	*touchpad_pskey_2130;
	int	touchpad_pskey_2130_len;
	u16	*touchpad_pskey_2131;
	int	touchpad_pskey_2131_len;
	u16	*touchpad_pskey_2132;
	int	touchpad_pskey_2132_len;
	u16	*touchpad_pskey_2133;
	int	touchpad_pskey_2133_len;
	u16	*touchpad_pskey_2134;
	int	touchpad_pskey_2134_len;
	u16	*touchpad_pskey_2135;
	int	touchpad_pskey_2135_len;
	u16	*touchpad_pskey_2136;
	int	touchpad_pskey_2136_len;
	u16	*touchpad_pskey_2137;
	int	touchpad_pskey_2137_len;
	u16	*touchpad_pskey_2138;
	int	touchpad_pskey_2138_len;
	u16	*touchpad_pskey_2139;
	int	touchpad_pskey_2139_len;
	u16	*touchpad_pskey_213a;
	int	touchpad_pskey_213a_len;
	u16	*touchpad_pskey_21e1;
	int	touchpad_pskey_21e1_len;
	u16	*touchpad_pskey_2215;
	int	touchpad_pskey_2215_len;
	u16	*touchpad_pskey_2216;
	int	touchpad_pskey_2216_len;
	u16	*touchpad_pskey_2227;
	int	touchpad_pskey_2227_len;
	u16	*touchpad_pskey_2228;
	int	touchpad_pskey_2228_len;
	u16	*touchpad_pskey_2229;
	int	touchpad_pskey_2229_len;
	u16	*touchpad_pskey_222a;
	int	touchpad_pskey_222a_len;
	u16	*touchpad_pskey_222b;
	int	touchpad_pskey_222b_len;

	/*
	 * Serdev mode state tracking
	 * When operating in serdev mode (device tree instantiated), we have
	 * access to GPIO power control and can properly handle WARM_RESET
	 * by power cycling the chip for a clean restart.
	 */
	bool	is_serdev;		/* True if operating in serdev mode */
	bool	warm_reset_sent;	/* WARM_RESET sent, expecting chip restart */
	void	*serdev_bdev;		/* Pointer to bcsp_serdev for GPIO access */

	/* Link establishment completion for serdev mode */
	struct completion link_up;	/* Signaled when BCSP link is established */
	bool	link_established;	/* True after first successful handshake */
};

/* Forward declaration for serdev power cycle */
#ifdef CONFIG_SERIAL_DEV_BUS
struct bcsp_serdev;
static int bcsp_serdev_power_cycle(struct bcsp_serdev *bdev);
#endif

/* ---- BCSP CRC calculation ---- */

/* Table for calculating CRC for polynomial 0x1021, LSB processed first,
 * initial value 0xffff, bits shifted in reverse order.
 */

static const u16 crc_table[] = {
	0x0000, 0x1081, 0x2102, 0x3183,
	0x4204, 0x5285, 0x6306, 0x7387,
	0x8408, 0x9489, 0xa50a, 0xb58b,
	0xc60c, 0xd68d, 0xe70e, 0xf78f
};

/* Initialise the crc calculator */
#define BCSP_CRC_INIT(x) x = 0xffff

/* Update crc with next data byte
 *
 * Implementation note
 *     The data byte is treated as two nibbles.  The crc is generated
 *     in reverse, i.e., bits are fed into the register from the top.
 */
static void bcsp_crc_update(u16 *crc, u8 d)
{
	u16 reg = *crc;

	reg = (reg >> 4) ^ crc_table[(reg ^ d) & 0x000f];
	reg = (reg >> 4) ^ crc_table[(reg ^ (d >> 4)) & 0x000f];

	*crc = reg;
}

/* ---- BCSP core ---- */

static void bcsp_slip_msgdelim(struct sk_buff *skb)
{
	const char pkt_delim = 0xc0;

	skb_put_data(skb, &pkt_delim, 1);
}

static void bcsp_slip_one_byte(struct sk_buff *skb, u8 c)
{
	const char esc_c0[2] = { 0xdb, 0xdc };
	const char esc_db[2] = { 0xdb, 0xdd };

	switch (c) {
	case 0xc0:
		skb_put_data(skb, &esc_c0, 2);
		break;
	case 0xdb:
		skb_put_data(skb, &esc_db, 2);
		break;
	default:
		skb_put_data(skb, &c, 1);
	}
}

static int bcsp_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct bcsp_struct *bcsp = hu->priv;

	if (skb->len > 0xFFF) {
		BT_ERR("Packet too long");
		kfree_skb(skb);
		return 0;
	}

	/*
	 * Reject HCI packets while chip is resetting after WARM_RESET.
	 * The chip needs to re-establish the BCSP link first.
	 * Also block during the re-establishment phase (BDADDR_SENT state).
	 */
	if (bcsp->warm_reset_sent || bcsp->bdaddr_state == BCSP_BDADDR_SENT) {
		BT_DBG("BCSP: Dropping packet during chip reset/re-establishment");
		kfree_skb(skb);
		return 0;
	}

	switch (hci_skb_pkt_type(skb)) {
	case HCI_ACLDATA_PKT:
	case HCI_COMMAND_PKT:
		skb_queue_tail(&bcsp->rel, skb);
		break;

	case HCI_SCODATA_PKT:
		skb_queue_tail(&bcsp->unrel, skb);
		break;

	default:
		BT_ERR("Unknown packet type");
		kfree_skb(skb);
		break;
	}

	return 0;
}

static struct sk_buff *bcsp_prepare_pkt(struct bcsp_struct *bcsp, u8 *data,
					int len, int pkt_type)
{
	struct sk_buff *nskb;
	u8 hdr[4], chan;
	u16 BCSP_CRC_INIT(bcsp_txmsg_crc);
	int rel, i;

	switch (pkt_type) {
	case HCI_ACLDATA_PKT:
		chan = 6;	/* BCSP ACL channel */
		rel = 1;	/* reliable channel */
		break;
	case HCI_COMMAND_PKT:
		chan = 5;	/* BCSP cmd/evt channel */
		rel = 1;	/* reliable channel */
		break;
	case HCI_SCODATA_PKT:
		chan = 7;	/* BCSP SCO channel */
		rel = 0;	/* unreliable channel */
		break;
	case BCSP_LE_PKT:
		chan = 1;	/* BCSP LE channel */
		rel = 0;	/* unreliable channel */
		break;
	case BCSP_ACK_PKT:
		chan = 0;	/* BCSP internal channel */
		rel = 0;	/* unreliable channel */
		break;
	case BCSP_BCCMD_PKT:
		chan = 2;	/* BCCMD channel */
		rel = 0;	/* unreliable channel */
		break;
	default:
		BT_ERR("Unknown packet type");
		return NULL;
	}

	if (hciextn && chan == 5) {
		__le16 opcode = ((struct hci_command_hdr *)data)->opcode;

		/* Vendor specific commands */
		if (hci_opcode_ogf(__le16_to_cpu(opcode)) == 0x3f) {
			u8 desc = *(data + HCI_COMMAND_HDR_SIZE);

			if ((desc & 0xf0) == 0xc0) {
				data += HCI_COMMAND_HDR_SIZE + 1;
				len  -= HCI_COMMAND_HDR_SIZE + 1;
				chan = desc & 0x0f;
			}
		}
	}

	/* Max len of packet: (original len +4(bcsp hdr) +2(crc))*2
	 * (because bytes 0xc0 and 0xdb are escaped, worst case is
	 * when the packet is all made of 0xc0 and 0xdb :) )
	 * + 2 (0xc0 delimiters at start and end).
	 */

	nskb = alloc_skb((len + 6) * 2 + 2, GFP_ATOMIC);
	if (!nskb)
		return NULL;

	hci_skb_pkt_type(nskb) = pkt_type;

	bcsp_slip_msgdelim(nskb);

	hdr[0] = bcsp->rxseq_txack << 3;
	bcsp->txack_req = 0;
	BT_DBG("We request packet no %u to card", bcsp->rxseq_txack);

	if (rel) {
		hdr[0] |= 0x80 + bcsp->msgq_txseq;
		BT_DBG("Sending packet with seqno %u", bcsp->msgq_txseq);
		bcsp->msgq_txseq = (bcsp->msgq_txseq + 1) & 0x07;
	}

	/*
	 * CRC handling for BCSP packets:
	 * - Link Establishment (channel 1): NO CRC - hciattach sends LE packets
	 *   with header byte 0 = 0x00 (no CRC). The chip may send LE packets
	 *   WITH CRC, but expects to receive them WITHOUT CRC.
	 * - All other channels: Use CRC if enabled (default true via txcrc)
	 */
	bool pkt_crc = bcsp->use_crc && chan != 1;

	if (pkt_crc)
		hdr[0] |= 0x40;

	hdr[1] = ((len << 4) & 0xff) | chan;
	hdr[2] = len >> 4;
	hdr[3] = ~(hdr[0] + hdr[1] + hdr[2]);

	/* Put BCSP header */
	for (i = 0; i < 4; i++) {
		bcsp_slip_one_byte(nskb, hdr[i]);

		if (pkt_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, hdr[i]);
	}

	/* Put payload */
	for (i = 0; i < len; i++) {
		bcsp_slip_one_byte(nskb, data[i]);

		if (pkt_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, data[i]);
	}

	/* Put CRC */
	if (pkt_crc) {
		bcsp_txmsg_crc = bitrev16(bcsp_txmsg_crc);
		bcsp_slip_one_byte(nskb, (u8)((bcsp_txmsg_crc >> 8) & 0x00ff));
		bcsp_slip_one_byte(nskb, (u8)(bcsp_txmsg_crc & 0x00ff));
	}

	bcsp_slip_msgdelim(nskb);
	return nskb;
}

/* This is a rewrite of pkt_avail in ABCSP */
static struct sk_buff *bcsp_dequeue(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	unsigned long flags;
	struct sk_buff *skb;

	/* First of all, check for unreliable messages in the queue,
	 * since they have priority
	 */

	skb = skb_dequeue(&bcsp->unrel);
	if (skb != NULL) {
		struct sk_buff *nskb;

		BT_INFO("BCSP: dequeuing unrel pkt type %d len %d",
			hci_skb_pkt_type(skb), skb->len);
		nskb = bcsp_prepare_pkt(bcsp, skb->data, skb->len,
					hci_skb_pkt_type(skb));
		if (nskb) {
			/* Debug: show first bytes of prepared packet */
			if (hci_skb_pkt_type(skb) == BCSP_LE_PKT && nskb->len <= 16)
				BT_INFO("BCSP: TX LE pkt: %*ph", nskb->len, nskb->data);
			kfree_skb(skb);
			return nskb;
		} else {
			skb_queue_head(&bcsp->unrel, skb);
			BT_ERR("Could not dequeue pkt because alloc_skb failed");
		}
	}

	/* Now, try to send a reliable pkt. We can only send a
	 * reliable packet if the number of packets sent but not yet ack'ed
	 * is < than the winsize
	 */

	spin_lock_irqsave_nested(&bcsp->unack.lock, flags, SINGLE_DEPTH_NESTING);

	if (bcsp->unack.qlen < BCSP_TXWINSIZE) {
		skb = skb_dequeue(&bcsp->rel);
		if (skb != NULL) {
			struct sk_buff *nskb;

			nskb = bcsp_prepare_pkt(bcsp, skb->data, skb->len,
						hci_skb_pkt_type(skb));
			if (nskb) {
				__skb_queue_tail(&bcsp->unack, skb);
				mod_timer(&bcsp->tbcsp, jiffies + HZ / 4);
				spin_unlock_irqrestore(&bcsp->unack.lock, flags);
				return nskb;
			} else {
				skb_queue_head(&bcsp->rel, skb);
				BT_ERR("Could not dequeue pkt because alloc_skb failed");
			}
		}
	}

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	/* We could not send a reliable packet, either because there are
	 * none or because there are too many unack'ed pkts. Did we receive
	 * any packets we have not acknowledged yet ?
	 */

	if (bcsp->txack_req) {
		/* if so, craft an empty ACK pkt and send it on BCSP unreliable
		 * channel 0
		 */
		struct sk_buff *nskb = bcsp_prepare_pkt(bcsp, NULL, 0, BCSP_ACK_PKT);
		return nskb;
	}

	/* We have nothing to send */
	return NULL;
}

static int bcsp_flush(struct hci_uart *hu)
{
	BT_DBG("hu %p", hu);
	return 0;
}

/* Remove ack'ed packets */
static void bcsp_pkt_cull(struct bcsp_struct *bcsp)
{
	struct sk_buff *skb, *tmp;
	unsigned long flags;
	int i, pkts_to_be_removed;
	u8 seqno;

	spin_lock_irqsave(&bcsp->unack.lock, flags);

	pkts_to_be_removed = skb_queue_len(&bcsp->unack);
	seqno = bcsp->msgq_txseq;

	while (pkts_to_be_removed) {
		if (bcsp->rxack == seqno)
			break;
		pkts_to_be_removed--;
		seqno = (seqno - 1) & 0x07;
	}

	if (bcsp->rxack != seqno)
		BT_ERR("Peer acked invalid packet");

	BT_DBG("Removing %u pkts out of %u, up to seqno %u",
	       pkts_to_be_removed, skb_queue_len(&bcsp->unack),
	       (seqno - 1) & 0x07);

	i = 0;
	skb_queue_walk_safe(&bcsp->unack, skb, tmp) {
		if (i >= pkts_to_be_removed)
			break;
		i++;

		__skb_unlink(skb, &bcsp->unack);
		dev_kfree_skb_irq(skb);
	}

	/*
	 * Only delete timer if link is active and no retransmissions pending.
	 * During link establishment (UNINIT/INIT states), the timer is used
	 * to send sync/conf packets, so we must keep it running.
	 */
	if (skb_queue_empty(&bcsp->unack) &&
	    bcsp->link_state == BCSP_LINK_ACTIVE)
		timer_delete(&bcsp->tbcsp);

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	if (i != pkts_to_be_removed)
		BT_ERR("Removed only %u out of %u pkts", i, pkts_to_be_removed);
}

/* Forward declarations for BCCMD functions used in link establishment */
static int bcsp_send_bdaddr_bccmd(struct hci_uart *hu, bdaddr_t *addr);
static int bcsp_send_warm_reset(struct hci_uart *hu);
static int bcsp_send_pskey_word(struct hci_uart *hu, u16 pskey, u16 value);
static int bcsp_send_pskey_data(struct hci_uart *hu, u16 pskey,
				const u16 *data, u16 len_words);

/* Handle BCSP link-establishment packets.
 * This implements the full BCSP link establishment state machine:
 * - sync: device is asking to establish link, we reply with sync_rsp
 * - sync_rsp: device acknowledged our sync, we send conf
 * - conf: device sends config, we reply with conf_rsp
 * - conf_rsp: link is established
 */
static void bcsp_handle_le_pkt(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	u8 sync_pkt[4]     = { 0xda, 0xdc, 0xed, 0xed };
	u8 sync_rsp_pkt[4] = { 0xac, 0xaf, 0xef, 0xee };
	u8 conf_pkt[4]     = { 0xad, 0xef, 0xac, 0xed };
	u8 conf_rsp_pkt[4] = { 0xde, 0xad, 0xd0, 0xd0 };
	u8 len_nibble, len_high;

	/* Debug: log LE packet reception */
	len_nibble = bcsp->rx_skb->data[1] >> 4;
	len_high = bcsp->rx_skb->data[2];

	/* Check packet has 4-byte payload (link establishment packets) */
	if (len_nibble != 4 || len_high != 0) {
		BT_DBG("BCSP LE pkt: not a 4-byte LE pkt, ignoring");
		return;
	}

	/* Log raw bytes of LE packet for debugging */
	BT_INFO("BCSP: RX LE payload: %02x %02x %02x %02x",
		bcsp->rx_skb->data[4], bcsp->rx_skb->data[5],
		bcsp->rx_skb->data[6], bcsp->rx_skb->data[7]);

	/* Handle sync packet - device is starting link establishment */
	if (!memcmp(&bcsp->rx_skb->data[4], sync_pkt, 4)) {
		struct sk_buff *nskb;

		BT_INFO("BCSP: sync received%s",
			bcsp->warm_reset_sent ? " (after WARM_RESET)" : "");

		/*
		 * In serdev mode after WARM_RESET: The chip has restarted
		 * and is sending sync to re-establish the link. We can power
		 * cycle for a completely clean state, since we have GPIO control.
		 */
#ifdef CONFIG_SERIAL_DEV_BUS
		if (bcsp->is_serdev && bcsp->warm_reset_sent && bcsp->serdev_bdev) {
			struct bcsp_serdev *bdev = bcsp->serdev_bdev;

			BT_INFO("BCSP: Serdev mode - power cycling for clean restart");

			/*
			 * Power cycle the chip. This gives us a completely
			 * fresh start with the newly configured PSKEYs.
			 */
			bcsp_serdev_power_cycle(bdev);

			/* Clear the warm reset flag */
			bcsp->warm_reset_sent = false;

			/*
			 * Reset ALL state for clean link establishment.
			 * After power cycle, we're starting completely fresh.
			 */

			/* Reset sequence numbers */
			bcsp->rxseq_txack = 0;
			bcsp->msgq_txseq = 0;

			/* Purge all queues */
			skb_queue_purge(&bcsp->unack);
			skb_queue_purge(&bcsp->rel);
			skb_queue_purge(&bcsp->unrel);

			/* Reset RX state machine - chip may send garbage on boot */
			if (bcsp->rx_skb) {
				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
			}
			bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
			bcsp->rx_count = 0;
			bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;

			/* The chip will send a new sync after power cycle */
			BT_INFO("BCSP: Waiting for chip to send sync after power cycle...");
			return;
		}
#endif

		/*
		 * Reset sequence numbers - the chip has reset (e.g., after
		 * WARM_RESET) and is starting fresh. We must reset our
		 * sequence state to match.
		 */
		bcsp->rxseq_txack = 0;
		bcsp->msgq_txseq = 0;

		/* Purge all queues for clean state */
		skb_queue_purge(&bcsp->unack);
		skb_queue_purge(&bcsp->rel);
		skb_queue_purge(&bcsp->unrel);

		/* Clear warm reset flag */
		bcsp->warm_reset_sent = false;

		nskb = alloc_skb(4, GFP_ATOMIC);
		if (!nskb)
			return;
		skb_put_data(nskb, sync_rsp_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		BT_INFO("BCSP: sending sync_rsp");
		skb_queue_head(&bcsp->unrel, nskb);  /* Head for immediate response */
		hci_uart_tx_wakeup(hu);
	}
	/* Handle sync_rsp packet - device acknowledged our sync, send conf */
	else if (!memcmp(&bcsp->rx_skb->data[4], sync_rsp_pkt, 4)) {
		struct sk_buff *nskb = alloc_skb(4, GFP_ATOMIC);

		BT_INFO("BCSP: sync_rsp received, moving to INIT state");

		/* Transition to INIT state - will send conf via timer */
		bcsp->link_state = BCSP_LINK_INIT;

		if (!nskb)
			return;
		skb_put_data(nskb, conf_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		skb_queue_tail(&bcsp->unrel, nskb);
		hci_uart_tx_wakeup(hu);
	}
	/* Handle conf packet - device sent config, reply with conf_rsp */
	else if (!memcmp(&bcsp->rx_skb->data[4], conf_pkt, 4)) {
		struct sk_buff *nskb = alloc_skb(4, GFP_ATOMIC);

		BT_INFO("BCSP: conf received, responding with conf_rsp (bdaddr_state=%d)",
			bcsp->bdaddr_state);

		/* Transition to ACTIVE state */
		bcsp->link_state = BCSP_LINK_ACTIVE;

		if (!nskb)
			return;
		skb_put_data(nskb, conf_rsp_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		skb_queue_head(&bcsp->unrel, nskb);  /* Head for immediate response */
		hci_uart_tx_wakeup(hu);

		/*
		 * After sending conf_rsp, the link is established.
		 * Signal any waiters (e.g., bcsp_setup waiting for link).
		 */
		if (!bcsp->link_established) {
			BT_INFO("BCSP: Link established (first time)");
			bcsp->link_established = true;
			complete(&bcsp->link_up);
		}

		/*
		 * If we were waiting for re-establishment after BD address
		 * config, mark it as complete now.
		 */
		if (bcsp->bdaddr_state == BCSP_BDADDR_SENT) {
			BT_INFO("BCSP: Link re-established after BD address config");
			bcsp->bdaddr_state = BCSP_BDADDR_DONE;
		}
	}
	/* Handle conf_rsp packet - link establishment complete */
	else if (!memcmp(&bcsp->rx_skb->data[4], conf_rsp_pkt, 4)) {
		BT_INFO("BCSP: conf_rsp received, link established (bdaddr_state=%d, is_serdev=%d)",
			bcsp->bdaddr_state, bcsp->is_serdev);

		/* Transition to ACTIVE state */
		bcsp->link_state = BCSP_LINK_ACTIVE;

		/* Signal link establishment completion */
		if (!bcsp->link_established) {
			BT_INFO("BCSP: Link established via conf_rsp");
			bcsp->link_established = true;
			complete(&bcsp->link_up);
		}

		if (bcsp->bdaddr_state == BCSP_BDADDR_PENDING && bcsp->is_serdev) {
			/*
			 * Serdev mode: Link is now established, send full
			 * PSKEY sequence + WARM_RESET. The chip will reset
			 * and we'll power cycle it via GPIO for clean restart.
			 */
			BT_INFO("BCSP: Serdev link up, sending full PSKEYs + WARM_RESET");

			/* === Common PSKEYs === */
			bcsp_send_pskey_word(hu, PSKEY_ANA_FREQ,
					     bcsp->pskey_ana_freq);
			bcsp_send_pskey_word(hu, PSKEY_ANA_FTRIM,
					     bcsp->pskey_ana_ftrim);
			bcsp_send_pskey_word(hu, PSKEY_HOST_INTERFACE,
					     bcsp->pskey_host_interface);
			bcsp_send_pskey_word(hu, PSKEY_PCM_MIN_CPU_CLOCK,
					     bcsp->pskey_deep_sleep);
			bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_ACL,
					     bcsp->pskey_hci_max_acl);
			bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_SCO,
					     bcsp->pskey_hci_max_sco);
			bcsp_send_pskey_word(hu, PSKEY_PCM_SAMPLE_SIZE,
					     bcsp->pskey_uart_baudrate);
			bcsp_send_pskey_word(hu, PSKEY_ENC_KEY_LMIN,
					     bcsp->pskey_enc_key_min);
			bcsp_send_pskey_word(hu, PSKEY_UNKNOWN_01F9,
					     bcsp->pskey_unknown_01f9);
			bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER_NO_RSSI,
					     bcsp->pskey_max_tx_power_no_rssi);
			bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER_NO_RSSI,
					     bcsp->pskey_default_tx_power_no_rssi);

			/* === Palm Platform PSKEYs === */
			bcsp_send_pskey_data(hu, PSKEY_PALM_01B3,
					     bcsp->pskey_palm_01b3,
					     bcsp->pskey_palm_01b3_len);
			bcsp_send_pskey_data(hu, PSKEY_PALM_01B6,
					     bcsp->pskey_palm_01b6,
					     bcsp->pskey_palm_01b6_len);
			bcsp_send_pskey_data(hu, PSKEY_PALM_01BF,
					     bcsp->pskey_palm_01bf,
					     bcsp->pskey_palm_01bf_len);
			bcsp_send_pskey_data(hu, PSKEY_PALM_01BA,
					     bcsp->pskey_palm_01ba,
					     bcsp->pskey_palm_01ba_len);
			bcsp_send_pskey_data(hu, PSKEY_PALM_01C7,
					     bcsp->pskey_palm_01c7,
					     bcsp->pskey_palm_01c7_len);
			bcsp_send_pskey_data(hu, PSKEY_PALM_01CA,
					     bcsp->pskey_palm_01ca,
					     bcsp->pskey_palm_01ca_len);
			bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER,
					     bcsp->pskey_default_tx_power);
			bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER,
					     bcsp->pskey_max_tx_power);
			bcsp_send_pskey_data(hu, PSKEY_PALM_001D,
					     bcsp->pskey_palm_001d,
					     bcsp->pskey_palm_001d_len);

			/* TX Power Table */
			if (bcsp->tx_power_table && bcsp->tx_power_table_len > 0) {
				bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
						     bcsp->tx_power_table,
						     bcsp->tx_power_table_len);
			} else {
				bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
						     palm_tx_power_table,
						     ARRAY_SIZE(palm_tx_power_table));
			}

			/* WARM_RESET to apply PSKEYs */
			bcsp_send_warm_reset(hu);
			bcsp->warm_reset_sent = true;

			BT_INFO("BCSP: PSKEYs + WARM_RESET sent, chip will reset");
			bcsp->bdaddr_state = BCSP_BDADDR_SENT;
		} else if (bcsp->bdaddr_state == BCSP_BDADDR_PENDING) {
			BT_INFO("BCSP: First link up, sending RF PSKEYs (no reset)");

			/*
			 * Line discipline mode: Send critical RF PSKEYs only.
			 */
			bcsp_send_pskey_word(hu, PSKEY_ANA_FREQ,
					     bcsp->pskey_ana_freq);
			bcsp_send_pskey_word(hu, PSKEY_ANA_FTRIM,
					     bcsp->pskey_ana_ftrim);
			bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER,
					     bcsp->pskey_max_tx_power);
			bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER,
					     bcsp->pskey_default_tx_power);

			if (bcsp->tx_power_table && bcsp->tx_power_table_len > 0) {
				bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
						     bcsp->tx_power_table,
						     bcsp->tx_power_table_len);
			} else {
				bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
						     palm_tx_power_table,
						     ARRAY_SIZE(palm_tx_power_table));
			}

			bcsp->bdaddr_state = BCSP_BDADDR_DONE;
			BT_INFO("BCSP: RF PSKEYs sent (no BD addr, no reset)");
		}
	}
}

static inline void bcsp_unslip_one_byte(struct bcsp_struct *bcsp, unsigned char byte)
{
	const u8 c0 = 0xc0, db = 0xdb;

	switch (bcsp->rx_esc_state) {
	case BCSP_ESCSTATE_NOESC:
		switch (byte) {
		case 0xdb:
			bcsp->rx_esc_state = BCSP_ESCSTATE_ESC;
			break;
		default:
			skb_put_data(bcsp->rx_skb, &byte, 1);
			if ((bcsp->rx_skb->data[0] & 0x40) != 0 &&
			    bcsp->rx_state != BCSP_W4_CRC)
				bcsp_crc_update(&bcsp->message_crc, byte);
			bcsp->rx_count--;
		}
		break;

	case BCSP_ESCSTATE_ESC:
		switch (byte) {
		case 0xdc:
			skb_put_data(bcsp->rx_skb, &c0, 1);
			if ((bcsp->rx_skb->data[0] & 0x40) != 0 &&
			    bcsp->rx_state != BCSP_W4_CRC)
				bcsp_crc_update(&bcsp->message_crc, 0xc0);
			bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;
			bcsp->rx_count--;
			break;

		case 0xdd:
			skb_put_data(bcsp->rx_skb, &db, 1);
			if ((bcsp->rx_skb->data[0] & 0x40) != 0 &&
			    bcsp->rx_state != BCSP_W4_CRC)
				bcsp_crc_update(&bcsp->message_crc, 0xdb);
			bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;
			bcsp->rx_count--;
			break;

		default:
			BT_ERR("Invalid byte %02x after esc byte", byte);
			kfree_skb(bcsp->rx_skb);
			bcsp->rx_skb = NULL;
			bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
			bcsp->rx_count = 0;
		}
	}
}

static void bcsp_complete_rx_pkt(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	int pass_up = 0;

	if (bcsp->rx_skb->data[0] & 0x80) {	/* reliable pkt */
		BT_DBG("Received seqno %u from card", bcsp->rxseq_txack);

		/* check the rx sequence number is as expected */
		if ((bcsp->rx_skb->data[0] & 0x07) == bcsp->rxseq_txack) {
			bcsp->rxseq_txack++;
			bcsp->rxseq_txack %= 0x8;
		} else {
			/* handle re-transmitted packet or
			 * when packet was missed
			 */
			BT_ERR("Out-of-order packet arrived, got %u expected %u",
			       bcsp->rx_skb->data[0] & 0x07, bcsp->rxseq_txack);

			/* do not process out-of-order packet payload */
			pass_up = 2;
		}

		/* send current txack value to all received reliable packets */
		bcsp->txack_req = 1;

		/* If needed, transmit an ack pkt */
		hci_uart_tx_wakeup(hu);
	}

	bcsp->rxack = (bcsp->rx_skb->data[0] >> 3) & 0x07;
	BT_DBG("Request for pkt %u from card", bcsp->rxack);

	/* handle received ACK indications,
	 * including those from out-of-order packets
	 */
	bcsp_pkt_cull(bcsp);

	if (pass_up != 2) {
		if ((bcsp->rx_skb->data[1] & 0x0f) == 6 &&
		    (bcsp->rx_skb->data[0] & 0x80)) {
			hci_skb_pkt_type(bcsp->rx_skb) = HCI_ACLDATA_PKT;
			pass_up = 1;
		} else if ((bcsp->rx_skb->data[1] & 0x0f) == 5 &&
			   (bcsp->rx_skb->data[0] & 0x80)) {
			hci_skb_pkt_type(bcsp->rx_skb) = HCI_EVENT_PKT;
			pass_up = 1;
		} else if ((bcsp->rx_skb->data[1] & 0x0f) == 7) {
			hci_skb_pkt_type(bcsp->rx_skb) = HCI_SCODATA_PKT;
			pass_up = 1;
		} else if ((bcsp->rx_skb->data[1] & 0x0f) == 1 &&
			   !(bcsp->rx_skb->data[0] & 0x80)) {
			BT_INFO("BCSP: RX LE hdr: %02x %02x %02x %02x",
				bcsp->rx_skb->data[0], bcsp->rx_skb->data[1],
				bcsp->rx_skb->data[2], bcsp->rx_skb->data[3]);
			bcsp_handle_le_pkt(hu);
			/*
			 * bcsp_handle_le_pkt may free rx_skb (e.g., during
			 * power cycle). Check before continuing.
			 */
			if (!bcsp->rx_skb)
				return;
			pass_up = 0;
		} else {
			BT_DBG("BCSP: unknown pkt chan=%d rel=%d",
			       bcsp->rx_skb->data[1] & 0x0f,
			       (bcsp->rx_skb->data[0] & 0x80) ? 1 : 0);
			pass_up = 0;
		}
	}

	if (pass_up == 0) {
		struct hci_event_hdr hdr;
		u8 desc = (bcsp->rx_skb->data[1] & 0x0f);

		if (desc != 0 && desc != 1) {
			if (hciextn) {
				desc |= 0xc0;
				skb_pull(bcsp->rx_skb, 4);
				memcpy(skb_push(bcsp->rx_skb, 1), &desc, 1);

				hdr.evt = 0xff;
				hdr.plen = bcsp->rx_skb->len;
				memcpy(skb_push(bcsp->rx_skb, HCI_EVENT_HDR_SIZE), &hdr, HCI_EVENT_HDR_SIZE);
				hci_skb_pkt_type(bcsp->rx_skb) = HCI_EVENT_PKT;

				hci_recv_frame(hu->hdev, bcsp->rx_skb);
			} else {
				BT_ERR("Packet for unknown channel (%u %s)",
				       bcsp->rx_skb->data[1] & 0x0f,
				       bcsp->rx_skb->data[0] & 0x80 ?
				       "reliable" : "unreliable");
				kfree_skb(bcsp->rx_skb);
			}
		} else
			kfree_skb(bcsp->rx_skb);
	} else if (pass_up == 1) {
		/* Pull out BCSP hdr */
		skb_pull(bcsp->rx_skb, 4);

		hci_recv_frame(hu->hdev, bcsp->rx_skb);
	} else {
		/* ignore packet payload of already ACKed re-transmitted
		 * packets or when a packet was missed in the BCSP window
		 */
		kfree_skb(bcsp->rx_skb);
	}

	bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
	bcsp->rx_skb = NULL;
}

static u16 bscp_get_crc(struct bcsp_struct *bcsp)
{
	return get_unaligned_be16(&bcsp->rx_skb->data[bcsp->rx_skb->len - 2]);
}

/* Recv data */
static int bcsp_recv(struct hci_uart *hu, const void *data, int count)
{
	struct bcsp_struct *bcsp = hu->priv;
	const unsigned char *ptr;

	if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
		return -EUNATCH;

	BT_DBG("hu %p count %d rx_state %d rx_count %ld",
	       hu, count, bcsp->rx_state, bcsp->rx_count);

	ptr = data;
	while (count) {
		if (bcsp->rx_count) {
			if (*ptr == 0xc0) {
				BT_ERR("Short BCSP packet");
				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
				bcsp->rx_state = BCSP_W4_PKT_START;
				bcsp->rx_count = 0;
			} else
				bcsp_unslip_one_byte(bcsp, *ptr);

			ptr++; count--;
			continue;
		}

		switch (bcsp->rx_state) {
		case BCSP_W4_BCSP_HDR:
			if ((0xff & (u8)~(bcsp->rx_skb->data[0] + bcsp->rx_skb->data[1] +
			    bcsp->rx_skb->data[2])) != bcsp->rx_skb->data[3]) {
				BT_ERR("Error in BCSP hdr checksum");
				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
				bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
				bcsp->rx_count = 0;
				continue;
			}
			bcsp->rx_state = BCSP_W4_DATA;
			bcsp->rx_count = (bcsp->rx_skb->data[1] >> 4) +
					(bcsp->rx_skb->data[2] << 4);	/* May be 0 */
			continue;

		case BCSP_W4_DATA:
			if (bcsp->rx_skb->data[0] & 0x40) {	/* pkt with crc */
				bcsp->rx_state = BCSP_W4_CRC;
				bcsp->rx_count = 2;
			} else
				bcsp_complete_rx_pkt(hu);
			continue;

		case BCSP_W4_CRC:
			if (bitrev16(bcsp->message_crc) != bscp_get_crc(bcsp)) {
				BT_ERR("Checksum failed: computed %04x received %04x",
				       bitrev16(bcsp->message_crc),
				       bscp_get_crc(bcsp));

				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
				bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
				bcsp->rx_count = 0;
				continue;
			}
			skb_trim(bcsp->rx_skb, bcsp->rx_skb->len - 2);
			bcsp_complete_rx_pkt(hu);
			continue;

		case BCSP_W4_PKT_DELIMITER:
			switch (*ptr) {
			case 0xc0:
				bcsp->rx_state = BCSP_W4_PKT_START;
				break;
			default:
				/*BT_ERR("Ignoring byte %02x", *ptr);*/
				break;
			}
			ptr++; count--;
			break;

		case BCSP_W4_PKT_START:
			switch (*ptr) {
			case 0xc0:
				ptr++; count--;
				break;

			default:
				bcsp->rx_state = BCSP_W4_BCSP_HDR;
				bcsp->rx_count = 4;
				bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;
				BCSP_CRC_INIT(bcsp->message_crc);

				/* Do not increment ptr or decrement count
				 * Allocate packet. Max len of a BCSP pkt=
				 * 0xFFF (payload) +4 (header) +2 (crc)
				 */

				bcsp->rx_skb = bt_skb_alloc(0x1005, GFP_ATOMIC);
				if (!bcsp->rx_skb) {
					BT_ERR("Can't allocate mem for new packet");
					bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
					bcsp->rx_count = 0;
					return 0;
				}
				break;
			}
			break;
		}
	}
	return count;
}

/* ---- BCCMD support for CSR chip initialization ---- */

/*
 * Send a single-word PSKEY via BCCMD
 * Used for simple configuration values like ANA_FREQ, TX power, etc.
 */
static int bcsp_send_pskey_word(struct hci_uart *hu, u16 pskey, u16 value)
{
	struct bcsp_struct *bcsp = hu->priv;
	struct sk_buff *skb;
	u8 bccmd[18];

	memset(bccmd, 0, sizeof(bccmd));

	/* BCCMD header */
	bccmd[0] = BCCMD_SETREQ & 0xff;
	bccmd[1] = (BCCMD_SETREQ >> 8) & 0xff;
	bccmd[2] = 0x09;	/* Length: 9 words (18 bytes) */
	bccmd[3] = 0x00;
	bccmd[4] = 0x00;	/* SeqNo */
	bccmd[5] = 0x00;
	bccmd[6] = BCCMD_VARID_PS & 0xff;
	bccmd[7] = (BCCMD_VARID_PS >> 8) & 0xff;
	bccmd[8] = 0x00;	/* Status */
	bccmd[9] = 0x00;

	/* PS payload */
	bccmd[10] = pskey & 0xff;	/* PSKey (low) */
	bccmd[11] = (pskey >> 8) & 0xff;	/* PSKey (high) */
	bccmd[12] = 0x01;		/* Length: 1 word (2 bytes) */
	bccmd[13] = 0x00;
	bccmd[14] = PSKEY_STORES_PSRAM;	/* Stores: PSRAM (volatile) - matches webOS */
	bccmd[15] = 0x00;
	bccmd[16] = value & 0xff;	/* Value (low) */
	bccmd[17] = (value >> 8) & 0xff;	/* Value (high) */

	skb = alloc_skb(sizeof(bccmd), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, bccmd, sizeof(bccmd));
	hci_skb_pkt_type(skb) = BCSP_BCCMD_PKT;

	skb_queue_tail(&bcsp->unrel, skb);
	hci_uart_tx_wakeup(hu);

	BT_INFO("BCSP: Set PSKEY 0x%04x = 0x%04x", pskey, value);

	return 0;
}

/*
 * Send a multi-word PSKEY via BCCMD
 * Used for power tables and other large configuration data.
 */
static int bcsp_send_pskey_data(struct hci_uart *hu, u16 pskey,
				const u16 *data, u16 len_words)
{
	struct bcsp_struct *bcsp = hu->priv;
	struct sk_buff *skb;
	u8 *bccmd;
	int bccmd_len;
	int payload_len;
	int i;

	/*
	 * BCCMD format:
	 * - Header: 10 bytes (Type, Length, SeqNo, VarID, Status)
	 * - PS header: 6 bytes (PSKey, Length, Stores)
	 * - Data: len_words * 2 bytes
	 *
	 * Total length in words must be set in header
	 */
	payload_len = 6 + (len_words * 2);  /* PS header + data */
	bccmd_len = 10 + payload_len;       /* BCCMD header + payload */

	bccmd = kmalloc(bccmd_len, GFP_KERNEL);
	if (!bccmd)
		return -ENOMEM;

	memset(bccmd, 0, bccmd_len);

	/* BCCMD header */
	bccmd[0] = BCCMD_SETREQ & 0xff;
	bccmd[1] = (BCCMD_SETREQ >> 8) & 0xff;
	/* Length in words (including header, excluding type) */
	bccmd[2] = ((bccmd_len / 2) - 1) & 0xff;
	bccmd[3] = (((bccmd_len / 2) - 1) >> 8) & 0xff;
	bccmd[4] = 0x00;	/* SeqNo */
	bccmd[5] = 0x00;
	bccmd[6] = BCCMD_VARID_PS & 0xff;
	bccmd[7] = (BCCMD_VARID_PS >> 8) & 0xff;
	bccmd[8] = 0x00;	/* Status */
	bccmd[9] = 0x00;

	/* PS payload header */
	bccmd[10] = pskey & 0xff;
	bccmd[11] = (pskey >> 8) & 0xff;
	bccmd[12] = len_words & 0xff;
	bccmd[13] = (len_words >> 8) & 0xff;
	bccmd[14] = PSKEY_STORES_PSRAM;	/* Stores: PSRAM (volatile) - matches webOS */
	bccmd[15] = 0x00;

	/* Copy data (little-endian u16 values) */
	for (i = 0; i < len_words; i++) {
		bccmd[16 + i * 2] = data[i] & 0xff;
		bccmd[16 + i * 2 + 1] = (data[i] >> 8) & 0xff;
	}

	skb = alloc_skb(bccmd_len, GFP_KERNEL);
	if (!skb) {
		kfree(bccmd);
		return -ENOMEM;
	}

	skb_put_data(skb, bccmd, bccmd_len);
	hci_skb_pkt_type(skb) = BCSP_BCCMD_PKT;

	skb_queue_tail(&bcsp->unrel, skb);
	hci_uart_tx_wakeup(hu);

	BT_INFO("BCSP: Set PSKEY 0x%04x with %d words", pskey, len_words);

	kfree(bccmd);
	return 0;
}

/*
 * Send WARM_RESET BCCMD to apply PSKEY changes
 * Format matches bcattach: 9 words (18 bytes) with 8 bytes padding
 */
static int bcsp_send_warm_reset(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	struct sk_buff *skb;
	u8 bccmd[18];

	memset(bccmd, 0, sizeof(bccmd));

	/* BCCMD header */
	bccmd[0] = BCCMD_SETREQ & 0xff;
	bccmd[1] = (BCCMD_SETREQ >> 8) & 0xff;
	bccmd[2] = 0x09;	/* Length: 9 words (matches bcattach) */
	bccmd[3] = 0x00;
	bccmd[4] = 0x00;	/* SeqNo */
	bccmd[5] = 0x00;
	bccmd[6] = BCCMD_VARID_WARM_RESET & 0xff;
	bccmd[7] = (BCCMD_VARID_WARM_RESET >> 8) & 0xff;
	bccmd[8] = 0x00;	/* Status */
	bccmd[9] = 0x00;
	/* bytes 10-17 are zero padding (already cleared by memset) */

	skb = alloc_skb(sizeof(bccmd), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, bccmd, sizeof(bccmd));
	hci_skb_pkt_type(skb) = BCSP_BCCMD_PKT;

	skb_queue_tail(&bcsp->unrel, skb);
	hci_uart_tx_wakeup(hu);

	BT_INFO("BCSP: Sent WARM_RESET to apply PSKEY changes");

	return 0;
}

/*
 * Send TouchPad-specific PSKEYs (RF calibration data)
 * These are sent only when touchpad_pskeys_present is true (loaded from DT)
 */
static void bcsp_send_touchpad_pskeys(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;

	if (!bcsp->touchpad_pskeys_present)
		return;

	BT_INFO("BCSP: Sending TouchPad RF calibration PSKEYs...");

#define SEND_TOUCHPAD_PSKEY(pskey_id, field) \
	do { \
		if (bcsp->touchpad_pskey_##field && \
		    bcsp->touchpad_pskey_##field##_len > 0) { \
			bcsp_send_pskey_data(hu, pskey_id, \
				bcsp->touchpad_pskey_##field, \
				bcsp->touchpad_pskey_##field##_len); \
			msleep(20); \
		} \
	} while (0)

	/* Configuration PSKEYs */
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_00F6, 00f6);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_0203, 0203);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_0394, 0394);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_03AA, 03aa);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_03AB, 03ab);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_03D4, 03d4);

	/* RF Calibration PSKEYs */
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_212C, 212c);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_212D, 212d);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_212E, 212e);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_212F, 212f);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2130, 2130);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2131, 2131);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2132, 2132);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2133, 2133);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2134, 2134);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2135, 2135);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2136, 2136);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2137, 2137);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2138, 2138);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2139, 2139);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_213A, 213a);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_21E1, 21e1);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2215, 2215);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2216, 2216);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2227, 2227);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2228, 2228);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_2229, 2229);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_222A, 222a);
	SEND_TOUCHPAD_PSKEY(PSKEY_TOUCHPAD_222B, 222b);

#undef SEND_TOUCHPAD_PSKEY

	BT_INFO("BCSP: TouchPad RF calibration PSKEYs sent");
}

/*
 * Parse BD address string "XX:XX:XX:XX:XX:XX" into bdaddr_t
 * Returns 0 on success, -1 on error
 */
static int bcsp_parse_bdaddr(const char *str, bdaddr_t *addr)
{
	unsigned int b[6];
	int i;

	if (!str || strlen(str) != 17)
		return -1;

	if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
		   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
		return -1;

	for (i = 0; i < 6; i++)
		addr->b[5 - i] = b[i];  /* bdaddr_t is in reverse order */

	return 0;
}

/*
 * Send BCCMD to set PSKEY_BDADDR via BCSP channel 2
 *
 * BCCMD packet structure:
 *   Type (2) | Length (2) | SeqNo (2) | VarID (2) | Status (2) | Payload
 *
 * For PSKEY_BDADDR:
 *   Type   = 0x0002 (SETREQ)
 *   Length = 0x000C (12 words)
 *   VarID  = 0x7003 (CSR_VARID_PS)
 *   Payload: PSKey (2) | Stores (2) | Length (2) | Data (8)
 */
static int bcsp_send_bdaddr_bccmd(struct hci_uart *hu, bdaddr_t *addr)
{
	struct bcsp_struct *bcsp = hu->priv;
	struct sk_buff *skb;
	u8 bccmd[24];

	/* BCCMD header */
	bccmd[0] = 0x02;	/* Type: SETREQ (low byte) */
	bccmd[1] = 0x00;	/* Type: SETREQ (high byte) */
	bccmd[2] = 0x0c;	/* Length: 12 words (low byte) */
	bccmd[3] = 0x00;	/* Length: 12 words (high byte) */
	bccmd[4] = 0x00;	/* SeqNo (low byte) */
	bccmd[5] = 0x00;	/* SeqNo (high byte) */
	bccmd[6] = 0x03;	/* VarID: 0x7003 PS (low byte) */
	bccmd[7] = 0x70;	/* VarID: 0x7003 PS (high byte) */
	bccmd[8] = 0x00;	/* Status (low byte) */
	bccmd[9] = 0x00;	/* Status (high byte) */

	/*
	 * PS payload format (from BlueZ bccmd.c):
	 *   PSKey  (2 bytes) - which PSKEY to set
	 *   Length (2 bytes) - data length in words
	 *   Stores (2 bytes) - which store to use
	 *   Data   (length * 2 bytes)
	 */
	bccmd[10] = 0x01;	/* PSKey: 0x0001 BDADDR (low byte) */
	bccmd[11] = 0x00;	/* PSKey: 0x0001 BDADDR (high byte) */
	bccmd[12] = 0x04;	/* Length: 4 words = 8 bytes (low byte) */
	bccmd[13] = 0x00;	/* Length: 4 words (high byte) */
	bccmd[14] = PSKEY_STORES_PSRAM;	/* Stores: PSRAM (volatile) - matches webOS */
	bccmd[15] = 0x00;

	/*
	 * BD Address in CSR PSKEY format:
	 *   Word 0: LAP[15:8] | (LAP[23:16] << 8)
	 *   Word 1: LAP[7:0] with padding
	 *   Word 2: UAP with padding
	 *   Word 3: NAP (little endian)
	 *
	 * bdaddr_t.b[] stores address in little endian:
	 *   b[0]=LAP[7:0], b[1]=LAP[15:8], b[2]=LAP[23:16],
	 *   b[3]=UAP, b[4]=NAP[7:0], b[5]=NAP[15:8]
	 */
	bccmd[16] = addr->b[1];	/* Word 0 low: LAP[15:8] */
	bccmd[17] = addr->b[2];	/* Word 0 high: LAP[23:16] */
	bccmd[18] = addr->b[0];	/* Word 1 low: LAP[7:0] */
	bccmd[19] = 0x00;	/* Word 1 high: padding */
	bccmd[20] = addr->b[3];	/* Word 2 low: UAP */
	bccmd[21] = 0x00;	/* Word 2 high: padding */
	bccmd[22] = addr->b[4];	/* Word 3 low: NAP[7:0] */
	bccmd[23] = addr->b[5];	/* Word 3 high: NAP[15:8] */

	skb = alloc_skb(sizeof(bccmd), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, bccmd, sizeof(bccmd));
	hci_skb_pkt_type(skb) = BCSP_BCCMD_PKT;

	skb_queue_tail(&bcsp->unrel, skb);
	hci_uart_tx_wakeup(hu);

	BT_INFO("BCSP: Sent BCCMD to set BD address %pMR", addr);

	return 0;
}

static int bcsp_setup(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	int i;

	/*
	 * BD address configuration for CSR chips.
	 *
	 * Note: hciattach performs BCSP link establishment in userspace before
	 * setting the line discipline. By the time bcsp_setup() is called, the
	 * link is already established. We cannot intercept conf_rsp packets.
	 *
	 * Instead, we send BCCMD commands here after link establishment:
	 * 1. Send PSKEY_BDADDR to set the BD address
	 * 2. Send WARM_RESET to apply the change
	 * 3. Wait for chip to reset and re-establish link
	 * 4. Continue with HCI initialization
	 */
	if (bcsp->bdaddr_state == BCSP_BDADDR_PENDING) {
		if (skip_pskeys) {
			BT_INFO("BCSP: Skipping PSKEY configuration (skip_pskeys=1)");
			bcsp->bdaddr_state = BCSP_BDADDR_DONE;
			return 0;
		}

		/*
		 * In serdev mode, wait for link establishment before
		 * sending PSKEYs.
		 */
		if (bcsp->is_serdev && !bcsp->link_established) {
			unsigned long timeout;

			BT_INFO("BCSP: Serdev mode - waiting for link establishment");

			timeout = wait_for_completion_timeout(&bcsp->link_up,
							      msecs_to_jiffies(5000));
			if (!timeout) {
				BT_ERR("BCSP: Timeout waiting for link establishment");
				return -ETIMEDOUT;
			}

			BT_INFO("BCSP: Link established, proceeding with PSKEY config");
		}

		BT_INFO("BCSP: Configuring chip with webOS PSKEYs + WARM_RESET");

		/*
		 * Send PSKEYs in webOS sequence (extracted from PmBtStack):
		 * 1. Common PSKEYs (crystal freq, ftrim, etc.)
		 * 2. Palm Platform Common PSKEYs (5 entries)
		 * 3. Palm Platform Specific PSKEYs (7 entries incl power table)
		 * 4. BD address
		 * 5. WARM_RESET to apply all PSKEYs
		 *
		 * All PSKEYs use stores=8 (PSRAM/volatile).
		 */

		/* === Common PSKEYs (from DT or Palm defaults) === */

		/* Crystal frequency - CRITICAL for RF */
		bcsp_send_pskey_word(hu, PSKEY_ANA_FREQ, bcsp->pskey_ana_freq);
		msleep(20);

		/* Crystal fine trim */
		bcsp_send_pskey_word(hu, PSKEY_ANA_FTRIM, bcsp->pskey_ana_ftrim);
		msleep(20);

		/* Host interface - BCSP mode */
		bcsp_send_pskey_word(hu, PSKEY_HOST_INTERFACE,
				     bcsp->pskey_host_interface);
		msleep(20);

		/* Deep sleep config */
		bcsp_send_pskey_word(hu, PSKEY_PCM_MIN_CPU_CLOCK,
				     bcsp->pskey_deep_sleep);
		msleep(20);

		/* HCI FC max ACL packets */
		bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_ACL,
				     bcsp->pskey_hci_max_acl);
		msleep(20);

		/* HCI FC max SCO packets */
		bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_SCO,
				     bcsp->pskey_hci_max_sco);
		msleep(20);

		/* UART baudrate divisor */
		bcsp_send_pskey_word(hu, PSKEY_PCM_SAMPLE_SIZE,
				     bcsp->pskey_uart_baudrate);
		msleep(20);

		/* Encryption key min length */
		bcsp_send_pskey_word(hu, PSKEY_ENC_KEY_LMIN,
				     bcsp->pskey_enc_key_min);
		msleep(20);

		/* Unknown 0x01F9 */
		bcsp_send_pskey_word(hu, PSKEY_UNKNOWN_01F9,
				     bcsp->pskey_unknown_01f9);
		msleep(20);

		/* Max TX power without RSSI */
		bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER_NO_RSSI,
				     bcsp->pskey_max_tx_power_no_rssi);
		msleep(20);

		/* Default TX power without RSSI */
		bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER_NO_RSSI,
				     bcsp->pskey_default_tx_power_no_rssi);
		msleep(20);

		/* === Palm Platform Common PSKEYs === */

		bcsp_send_pskey_data(hu, PSKEY_PALM_01B3,
				     bcsp->pskey_palm_01b3,
				     bcsp->pskey_palm_01b3_len);
		msleep(20);

		bcsp_send_pskey_data(hu, PSKEY_PALM_01B6,
				     bcsp->pskey_palm_01b6,
				     bcsp->pskey_palm_01b6_len);
		msleep(20);

		bcsp_send_pskey_data(hu, PSKEY_PALM_01BF,
				     bcsp->pskey_palm_01bf,
				     bcsp->pskey_palm_01bf_len);
		msleep(20);

		/* === Palm Platform Specific PSKEYs === */

		bcsp_send_pskey_data(hu, PSKEY_PALM_01BA,
				     bcsp->pskey_palm_01ba,
				     bcsp->pskey_palm_01ba_len);
		msleep(20);

		bcsp_send_pskey_data(hu, PSKEY_PALM_01C7,
				     bcsp->pskey_palm_01c7,
				     bcsp->pskey_palm_01c7_len);
		msleep(20);

		bcsp_send_pskey_data(hu, PSKEY_PALM_01CA,
				     bcsp->pskey_palm_01ca,
				     bcsp->pskey_palm_01ca_len);
		msleep(20);

		/* Default TX Power (PSKEY 0x0021) */
		bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER,
				     bcsp->pskey_default_tx_power);
		msleep(20);

		/* Max TX Power (PSKEY 0x0017) */
		bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER,
				     bcsp->pskey_max_tx_power);
		msleep(20);

		/* Palm config 0x001D */
		bcsp_send_pskey_data(hu, PSKEY_PALM_001D,
				     bcsp->pskey_palm_001d,
				     bcsp->pskey_palm_001d_len);
		msleep(20);

		/* TouchPad-specific RF calibration PSKEYs (if present) */
		bcsp_send_touchpad_pskeys(hu);

		/*
		 * TX Power Level Table (PSKEY 0x0031) - CRITICAL FOR RF
		 * Use DT-provided table if available, otherwise use Palm defaults.
		 */
		if (bcsp->tx_power_table && bcsp->tx_power_table_len > 0) {
			bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
					     bcsp->tx_power_table,
					     bcsp->tx_power_table_len);
		} else {
			bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
					     palm_tx_power_table,
					     ARRAY_SIZE(palm_tx_power_table));
		}
		msleep(50);

		/* BD address */
		BT_INFO("BCSP: Setting BD address %pMR", &bcsp->bdaddr);
		bcsp_send_bdaddr_bccmd(hu, &bcsp->bdaddr);
		msleep(50);

		/* WARM_RESET to apply all PSKEYs */
		bcsp_send_warm_reset(hu);
		bcsp->warm_reset_sent = true;
		msleep(50);

		/* Give TX time to send the packets */
		for (i = 0; i < 10; i++) {
			if (skb_queue_empty(&bcsp->unrel))
				break;
			msleep(50);
			hci_uart_tx_wakeup(hu);
		}

		BT_INFO("BCSP: WARM_RESET sent, chip will reset...");

		/*
		 * After WARM_RESET, the chip resets and the BCSP link breaks.
		 *
		 * In serdev mode: We have GPIO control and can power cycle the chip.
		 * The sync packet handler will detect the chip restart and trigger
		 * a power cycle for clean re-initialization.
		 *
		 * In line discipline mode: Let hciattach restart with a fresh
		 * connection to the now-configured chip.
		 */
		if (bcsp->is_serdev) {
			BT_INFO("BCSP: Serdev mode - waiting for chip restart...");
			/* Wait longer in serdev mode to allow sync detection */
			msleep(1000);
		} else {
			msleep(500);
			BT_INFO("BCSP: PSKEYs + BDADDR applied. Restart hciattach for fresh connection.");
		}

		bcsp->bdaddr_state = BCSP_BDADDR_DONE;
	} else if (bcsp->bdaddr_state == BCSP_BDADDR_NONE) {
		/*
		 * Fast init mode - no BD address to configure, but we still
		 * need to send RF calibration PSKEYs and WARM_RESET for the
		 * radio to work. The crystal frequency (ANA_FREQ) in particular
		 * requires a reset to take effect.
		 */
		if (skip_pskeys) {
			BT_INFO("BCSP: Skipping PSKEY configuration (skip_pskeys=1)");
			return 0;
		}

		/*
		 * In serdev mode, bcsp_setup() is called before the BCSP link
		 * is established. We must wait for link establishment before
		 * sending PSKEYs. The conf handler will signal link_up when
		 * the handshake completes.
		 *
		 * Note: Check hu->serdev directly since bcsp->is_serdev isn't
		 * set until after hci_uart_register_device() returns.
		 */
		if (hu->serdev) {
			unsigned long timeout;

			BT_INFO("BCSP: Serdev mode - waiting for link establishment");

			/* Wait up to 5 seconds for link to be established */
			timeout = wait_for_completion_timeout(&bcsp->link_up,
							      msecs_to_jiffies(5000));
			if (!timeout) {
				BT_ERR("BCSP: Timeout waiting for link establishment");
				return -ETIMEDOUT;
			}

			BT_INFO("BCSP: Link established, proceeding with PSKEY config");
		}

		BT_INFO("BCSP: Sending PSKEYs + WARM_RESET (no BD addr)");

		/*
		 * Send PSKEYs in webOS sequence - same as BCSP_BDADDR_PENDING
		 * but without BD address configuration.
		 * Uses values from DT or Palm defaults.
		 */

		/* === Common PSKEYs === */
		bcsp_send_pskey_word(hu, PSKEY_ANA_FREQ, bcsp->pskey_ana_freq);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_ANA_FTRIM, bcsp->pskey_ana_ftrim);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_HOST_INTERFACE,
				     bcsp->pskey_host_interface);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_PCM_MIN_CPU_CLOCK,
				     bcsp->pskey_deep_sleep);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_ACL,
				     bcsp->pskey_hci_max_acl);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_SCO,
				     bcsp->pskey_hci_max_sco);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_PCM_SAMPLE_SIZE,
				     bcsp->pskey_uart_baudrate);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_ENC_KEY_LMIN,
				     bcsp->pskey_enc_key_min);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_UNKNOWN_01F9,
				     bcsp->pskey_unknown_01f9);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER_NO_RSSI,
				     bcsp->pskey_max_tx_power_no_rssi);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER_NO_RSSI,
				     bcsp->pskey_default_tx_power_no_rssi);
		msleep(20);

		/* === Palm Platform Common PSKEYs === */
		bcsp_send_pskey_data(hu, PSKEY_PALM_01B3,
				     bcsp->pskey_palm_01b3,
				     bcsp->pskey_palm_01b3_len);
		msleep(20);
		bcsp_send_pskey_data(hu, PSKEY_PALM_01B6,
				     bcsp->pskey_palm_01b6,
				     bcsp->pskey_palm_01b6_len);
		msleep(20);
		bcsp_send_pskey_data(hu, PSKEY_PALM_01BF,
				     bcsp->pskey_palm_01bf,
				     bcsp->pskey_palm_01bf_len);
		msleep(20);

		/* === Palm Platform Specific PSKEYs === */
		bcsp_send_pskey_data(hu, PSKEY_PALM_01BA,
				     bcsp->pskey_palm_01ba,
				     bcsp->pskey_palm_01ba_len);
		msleep(20);
		bcsp_send_pskey_data(hu, PSKEY_PALM_01C7,
				     bcsp->pskey_palm_01c7,
				     bcsp->pskey_palm_01c7_len);
		msleep(20);
		bcsp_send_pskey_data(hu, PSKEY_PALM_01CA,
				     bcsp->pskey_palm_01ca,
				     bcsp->pskey_palm_01ca_len);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER,
				     bcsp->pskey_default_tx_power);
		msleep(20);
		bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER,
				     bcsp->pskey_max_tx_power);
		msleep(20);
		bcsp_send_pskey_data(hu, PSKEY_PALM_001D,
				     bcsp->pskey_palm_001d,
				     bcsp->pskey_palm_001d_len);
		msleep(20);

		/* TouchPad-specific RF calibration PSKEYs (if present) */
		bcsp_send_touchpad_pskeys(hu);

		/* TX Power Table - use DT if available, else Palm defaults */
		if (bcsp->tx_power_table && bcsp->tx_power_table_len > 0) {
			bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
					     bcsp->tx_power_table,
					     bcsp->tx_power_table_len);
		} else {
			bcsp_send_pskey_data(hu, PSKEY_TX_POWER_LEVEL,
					     palm_tx_power_table,
					     ARRAY_SIZE(palm_tx_power_table));
		}
		msleep(50);

		/* WARM_RESET to apply PSKEYs */
		bcsp_send_warm_reset(hu);
		bcsp->warm_reset_sent = true;
		msleep(50);

		/* Wait for packets to be sent */
		for (i = 0; i < 10; i++) {
			if (skb_queue_empty(&bcsp->unrel))
				break;
			msleep(50);
			hci_uart_tx_wakeup(hu);
		}

		BT_INFO("BCSP: WARM_RESET sent, chip will reset...");

		/*
		 * After WARM_RESET, the chip resets and the BCSP link breaks.
		 *
		 * In serdev mode: We have GPIO control and can power cycle the chip.
		 * The sync packet handler will detect the chip restart and trigger
		 * a power cycle for clean re-initialization.
		 *
		 * In line discipline mode: Let hciattach restart with a fresh
		 * connection to the now-configured chip.
		 */
		if (bcsp->is_serdev) {
			BT_INFO("BCSP: Serdev mode - waiting for chip restart...");
			msleep(1000);
		} else {
			msleep(500);
			BT_INFO("BCSP: PSKEYs applied. Restart hciattach for fresh connection.");
		}

		bcsp->bdaddr_state = BCSP_BDADDR_DONE;
	}

	return 0;
}

/*
 * Send a link establishment packet (sync or conf).
 */
static void bcsp_send_link_pkt(struct bcsp_struct *bcsp, const u8 *data, size_t len)
{
	struct sk_buff *skb;

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb)
		return;

	skb_put_data(skb, data, len);
	hci_skb_pkt_type(skb) = BCSP_LE_PKT;
	skb_queue_tail(&bcsp->unrel, skb);
}

/*
 * Timer callback for link establishment and retransmission.
 * - In UNINIT state: send sync packets to initiate link
 * - In INIT state: send conf packets to complete handshake
 * - In ACTIVE state: handle reliable packet retransmission
 */
static void bcsp_timed_event(struct timer_list *t)
{
	static const u8 sync_pkt[4] = { 0xda, 0xdc, 0xed, 0xed };
	static const u8 conf_pkt[4] = { 0xad, 0xef, 0xac, 0xed };
	struct bcsp_struct *bcsp = timer_container_of(bcsp, t, tbcsp);
	struct hci_uart *hu = bcsp->hu;
	struct sk_buff *skb;
	unsigned long flags;

	BT_DBG("hu %p link_state %d unack %u", hu, bcsp->link_state, bcsp->unack.qlen);

	/*
	 * BCSP Link Establishment requires both sides to send sync.
	 * When the chip receives our sync, it responds with sync_rsp.
	 * When we receive sync_rsp, we move to INIT state.
	 * Meanwhile, we also respond to chip's sync with our sync_rsp.
	 */
	if (bcsp->link_state == BCSP_LINK_UNINIT) {
		BT_INFO("BCSP: timer sending sync (link_state=%d)", bcsp->link_state);
		bcsp_send_link_pkt(bcsp, sync_pkt, sizeof(sync_pkt));
	}

	/* Send conf packets in INIT state */
	if (bcsp->link_state == BCSP_LINK_INIT) {
		BT_INFO("BCSP: timer sending conf (link_state=%d)", bcsp->link_state);
		bcsp_send_link_pkt(bcsp, conf_pkt, sizeof(conf_pkt));
	}

	/* Re-arm timer if link not yet active */
	BT_INFO("BCSP: timer check link_state=%d (ACTIVE=%d)",
		bcsp->link_state, BCSP_LINK_ACTIVE);
	if (bcsp->link_state != BCSP_LINK_ACTIVE) {
		mod_timer(&bcsp->tbcsp, jiffies + HZ / 4);
		BT_INFO("BCSP: timer re-armed");
	}

	/* Handle retransmission of reliable packets */
	spin_lock_irqsave_nested(&bcsp->unack.lock, flags, SINGLE_DEPTH_NESTING);

	while ((skb = __skb_dequeue_tail(&bcsp->unack)) != NULL) {
		bcsp->msgq_txseq = (bcsp->msgq_txseq - 1) & 0x07;
		skb_queue_head(&bcsp->rel, skb);
	}

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	hci_uart_tx_wakeup(hu);
}

/*
 * Initialize PSKEY values with Palm/webOS defaults for HP TouchPad BCM4329.
 * These can be overridden by device tree properties.
 */
static void bcsp_init_pskey_defaults(struct bcsp_struct *bcsp)
{
	/* Common PSKEYs - webOS defaults */
	bcsp->pskey_ana_freq = 26000;		/* 26 MHz crystal */
	bcsp->pskey_ana_ftrim = 0x19;		/* Crystal fine trim */
	bcsp->pskey_host_interface = 0x0001;	/* BCSP mode */
	bcsp->pskey_deep_sleep = 0x0000;	/* Deep sleep config */
	bcsp->pskey_hci_max_acl = 0x03ec;	/* HCI FC max ACL */
	bcsp->pskey_hci_max_sco = 0x0000;	/* HCI FC max SCO */
	bcsp->pskey_uart_baudrate = 0x01d8;	/* 115200 baud */
	bcsp->pskey_enc_key_min = 0x0001;	/* Encryption key min */
	bcsp->pskey_unknown_01f9 = 0x0001;	/* Unknown */
	bcsp->pskey_max_tx_power_no_rssi = 0x0004;
	bcsp->pskey_default_tx_power_no_rssi = 0x0001;
	bcsp->pskey_max_tx_power = 0x0004;	/* Max TX power */
	bcsp->pskey_default_tx_power = 0x0004;	/* Default TX power */

	/* Palm Platform PSKEYs - use static defaults */
	bcsp->pskey_palm_01b3 = palm_pskey_01b3;
	bcsp->pskey_palm_01b3_len = ARRAY_SIZE(palm_pskey_01b3);
	bcsp->pskey_palm_01b6 = palm_pskey_01b6;
	bcsp->pskey_palm_01b6_len = ARRAY_SIZE(palm_pskey_01b6);
	bcsp->pskey_palm_01bf = palm_pskey_01bf;
	bcsp->pskey_palm_01bf_len = ARRAY_SIZE(palm_pskey_01bf);
	bcsp->pskey_palm_01ba = palm_pskey_01ba;
	bcsp->pskey_palm_01ba_len = ARRAY_SIZE(palm_pskey_01ba);
	bcsp->pskey_palm_01c7 = palm_pskey_01c7;
	bcsp->pskey_palm_01c7_len = ARRAY_SIZE(palm_pskey_01c7);
	bcsp->pskey_palm_01ca = palm_pskey_01ca;
	bcsp->pskey_palm_01ca_len = ARRAY_SIZE(palm_pskey_01ca);
	bcsp->pskey_palm_001d = palm_pskey_001d;
	bcsp->pskey_palm_001d_len = ARRAY_SIZE(palm_pskey_001d);

	/* TX power table - use Palm default */
	bcsp->tx_power_table = NULL;  /* Will use palm_tx_power_table */
	bcsp->tx_power_table_len = 0;
}

/*
 * Read PSKEY configuration from device tree.
 * PSKEYs are stored in properties of a compatible = "brcm,bcm4329-bt" node.
 * Values from DT override the Palm defaults.
 */
static void bcsp_read_pskeys_from_dt(struct bcsp_struct *bcsp)
{
	struct device_node *np;
	int len, ret;
	int pskeys_read = 0;

	/* Initialize with Palm/webOS defaults first */
	bcsp_init_pskey_defaults(bcsp);
	bcsp->pskeys_from_dt = false;

	np = of_find_compatible_node(NULL, NULL, "brcm,bcm4329-bt");
	if (!np) {
		BT_INFO("BCSP: No DT node found, using Palm defaults");
		return;
	}

	/* Read TX power table from DT (overrides Palm default) */
	if (of_find_property(np, "brcm,tx-power-table", &len)) {
		len = len / sizeof(u16);
		if (len > 0 && len <= BCSP_TX_POWER_TABLE_MAX) {
			bcsp->tx_power_table = kmalloc_array(len, sizeof(u16),
							     GFP_KERNEL);
			if (bcsp->tx_power_table) {
				ret = of_property_read_u16_array(np,
					"brcm,tx-power-table",
					bcsp->tx_power_table, len);
				if (ret) {
					kfree(bcsp->tx_power_table);
					bcsp->tx_power_table = NULL;
				} else {
					bcsp->tx_power_table_len = len;
					pskeys_read++;
				}
			}
		}
	}

	/* Read individual PSKEYs from DT (override defaults) */
	if (!of_property_read_u16(np, "brcm,ana-freq",
				  &bcsp->pskey_ana_freq))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,ana-ftrim",
				  &bcsp->pskey_ana_ftrim))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,host-interface",
				  &bcsp->pskey_host_interface))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,deep-sleep",
				  &bcsp->pskey_deep_sleep))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,hci-max-acl",
				  &bcsp->pskey_hci_max_acl))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,hci-max-sco",
				  &bcsp->pskey_hci_max_sco))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,uart-baudrate",
				  &bcsp->pskey_uart_baudrate))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,enc-key-min",
				  &bcsp->pskey_enc_key_min))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,max-tx-power",
				  &bcsp->pskey_max_tx_power))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,default-tx-power",
				  &bcsp->pskey_default_tx_power))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,max-tx-power-no-rssi",
				  &bcsp->pskey_max_tx_power_no_rssi))
		pskeys_read++;
	if (!of_property_read_u16(np, "brcm,default-tx-power-no-rssi",
				  &bcsp->pskey_default_tx_power_no_rssi))
		pskeys_read++;

	if (pskeys_read > 0) {
		bcsp->pskeys_from_dt = true;
		BT_INFO("BCSP: %d PSKEYs overridden from DT", pskeys_read);
	} else {
		BT_INFO("BCSP: Using Palm defaults (no DT overrides)");
	}

	/*
	 * Read TouchPad-specific PSKEYs (RF calibration data)
	 * These are only present on HP TouchPad (LMP subversion 0x12E9)
	 */
	bcsp->touchpad_pskeys_present = false;

#define READ_TOUCHPAD_PSKEY(name, field) \
	do { \
		if (of_find_property(np, "brcm,pskey-" name, &len)) { \
			len = len / sizeof(u16); \
			if (len > 0) { \
				bcsp->touchpad_pskey_##field = \
					kmalloc_array(len, sizeof(u16), GFP_KERNEL); \
				if (bcsp->touchpad_pskey_##field) { \
					ret = of_property_read_u16_array(np, \
						"brcm,pskey-" name, \
						bcsp->touchpad_pskey_##field, len); \
					if (ret) { \
						kfree(bcsp->touchpad_pskey_##field); \
						bcsp->touchpad_pskey_##field = NULL; \
					} else { \
						bcsp->touchpad_pskey_##field##_len = len; \
						bcsp->touchpad_pskeys_present = true; \
					} \
				} \
			} \
		} \
	} while (0)

	/* Configuration PSKEYs */
	READ_TOUCHPAD_PSKEY("00f6", 00f6);
	READ_TOUCHPAD_PSKEY("0203", 0203);
	READ_TOUCHPAD_PSKEY("0394", 0394);
	READ_TOUCHPAD_PSKEY("03aa", 03aa);
	READ_TOUCHPAD_PSKEY("03ab", 03ab);
	READ_TOUCHPAD_PSKEY("03d4", 03d4);

	/* RF Calibration PSKEYs */
	READ_TOUCHPAD_PSKEY("212c", 212c);
	READ_TOUCHPAD_PSKEY("212d", 212d);
	READ_TOUCHPAD_PSKEY("212e", 212e);
	READ_TOUCHPAD_PSKEY("212f", 212f);
	READ_TOUCHPAD_PSKEY("2130", 2130);
	READ_TOUCHPAD_PSKEY("2131", 2131);
	READ_TOUCHPAD_PSKEY("2132", 2132);
	READ_TOUCHPAD_PSKEY("2133", 2133);
	READ_TOUCHPAD_PSKEY("2134", 2134);
	READ_TOUCHPAD_PSKEY("2135", 2135);
	READ_TOUCHPAD_PSKEY("2136", 2136);
	READ_TOUCHPAD_PSKEY("2137", 2137);
	READ_TOUCHPAD_PSKEY("2138", 2138);
	READ_TOUCHPAD_PSKEY("2139", 2139);
	READ_TOUCHPAD_PSKEY("213a", 213a);
	READ_TOUCHPAD_PSKEY("21e1", 21e1);
	READ_TOUCHPAD_PSKEY("2215", 2215);
	READ_TOUCHPAD_PSKEY("2216", 2216);
	READ_TOUCHPAD_PSKEY("2227", 2227);
	READ_TOUCHPAD_PSKEY("2228", 2228);
	READ_TOUCHPAD_PSKEY("2229", 2229);
	READ_TOUCHPAD_PSKEY("222a", 222a);
	READ_TOUCHPAD_PSKEY("222b", 222b);

#undef READ_TOUCHPAD_PSKEY

	if (bcsp->touchpad_pskeys_present)
		BT_INFO("BCSP: TouchPad RF calibration PSKEYs loaded from DT");

	/*
	 * Read local-bd-address from device tree.
	 * Standard Bluetooth DT binding uses little-endian format:
	 * For address 00:1D:FE:85:64:A9, the DT value is [A9 64 85 FE 1D 00]
	 */
	{
		u8 bd_addr_le[6];

		if (!of_property_read_u8_array(np, "local-bd-address",
					       bd_addr_le, 6)) {
			/* Convert from DT little-endian to bdaddr_t format */
			bcsp->bdaddr.b[0] = bd_addr_le[0];
			bcsp->bdaddr.b[1] = bd_addr_le[1];
			bcsp->bdaddr.b[2] = bd_addr_le[2];
			bcsp->bdaddr.b[3] = bd_addr_le[3];
			bcsp->bdaddr.b[4] = bd_addr_le[4];
			bcsp->bdaddr.b[5] = bd_addr_le[5];
			bcsp->bdaddr_from_dt = true;
			BT_INFO("BCSP: BD address from DT: %pMR", &bcsp->bdaddr);
			pskeys_read++;
		}
	}

	of_node_put(np);
}

static int bcsp_open(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp;

	BT_DBG("hu %p", hu);

	bcsp = kzalloc(sizeof(*bcsp), GFP_KERNEL);
	if (!bcsp)
		return -ENOMEM;

	hu->priv = bcsp;
	bcsp->hu = hu;
	skb_queue_head_init(&bcsp->unack);
	skb_queue_head_init(&bcsp->rel);
	skb_queue_head_init(&bcsp->unrel);

	/*
	 * Enable hardware flow control for serdev devices.
	 * hci_serdev.c disables flow control before calling open(),
	 * but BCSP needs it for proper link establishment with BCM4329.
	 * WebOS used UART_WITH_FLOW_CONTROL for GSBI6 (BT UART).
	 */
	if (hu->serdev) {
		serdev_device_set_flow_control(hu->serdev, true);
		BT_INFO("BCSP: Enabled hardware flow control for serdev");
	}

	timer_setup(&bcsp->tbcsp, bcsp_timed_event, 0);

	bcsp->rx_state = BCSP_W4_PKT_DELIMITER;

	/* Initialize link establishment state machine */
	bcsp->link_state = BCSP_LINK_UNINIT;
	init_completion(&bcsp->link_up);
	bcsp->link_established = false;

	/*
	 * Send one initial sync packet to wake the chip.
	 * The chip will respond with sync, and we'll respond with sync_rsp.
	 */
	{
		static const u8 sync_pkt[4] = { 0xda, 0xdc, 0xed, 0xed };
		struct sk_buff *skb = alloc_skb(4, GFP_KERNEL);
		if (skb) {
			skb_put_data(skb, sync_pkt, 4);
			hci_skb_pkt_type(skb) = BCSP_LE_PKT;
			skb_queue_tail(&bcsp->unrel, skb);
			BT_INFO("BCSP: Sent initial sync to wake chip");
		}
	}

	/* Start timer for retransmissions and conf sending */
	mod_timer(&bcsp->tbcsp, jiffies + HZ / 4);

	if (txcrc)
		bcsp->use_crc = 1;

	/* Load TX power table from device tree */
	bcsp_read_pskeys_from_dt(bcsp);

	/*
	 * Initialize BD address configuration state.
	 * Priority: 1) Device tree, 2) Module parameter
	 */
	bcsp->bdaddr_state = BCSP_BDADDR_NONE;
	if (bcsp->bdaddr_from_dt) {
		/* BD address already loaded from device tree */
		bcsp->bdaddr_state = BCSP_BDADDR_PENDING;
		BT_INFO("BCSP: Will configure BD address from DT on link up");
	} else if (bdaddr && bdaddr[0]) {
		if (bcsp_parse_bdaddr(bdaddr, &bcsp->bdaddr) == 0) {
			bcsp->bdaddr_state = BCSP_BDADDR_PENDING;
			BT_INFO("BCSP: Will configure BD address %s on link up", bdaddr);
		} else {
			BT_ERR("BCSP: Invalid bdaddr parameter: %s", bdaddr);
		}
	}

	return 0;
}

static int bcsp_close(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;

	timer_shutdown_sync(&bcsp->tbcsp);

	hu->priv = NULL;

	BT_DBG("hu %p", hu);

	skb_queue_purge(&bcsp->unack);
	skb_queue_purge(&bcsp->rel);
	skb_queue_purge(&bcsp->unrel);

	if (bcsp->rx_skb) {
		kfree_skb(bcsp->rx_skb);
		bcsp->rx_skb = NULL;
	}

	kfree(bcsp->tx_power_table);

	/* Free TouchPad PSKEYs */
	kfree(bcsp->touchpad_pskey_00f6);
	kfree(bcsp->touchpad_pskey_0203);
	kfree(bcsp->touchpad_pskey_0394);
	kfree(bcsp->touchpad_pskey_03aa);
	kfree(bcsp->touchpad_pskey_03ab);
	kfree(bcsp->touchpad_pskey_03d4);
	kfree(bcsp->touchpad_pskey_212c);
	kfree(bcsp->touchpad_pskey_212d);
	kfree(bcsp->touchpad_pskey_212e);
	kfree(bcsp->touchpad_pskey_212f);
	kfree(bcsp->touchpad_pskey_2130);
	kfree(bcsp->touchpad_pskey_2131);
	kfree(bcsp->touchpad_pskey_2132);
	kfree(bcsp->touchpad_pskey_2133);
	kfree(bcsp->touchpad_pskey_2134);
	kfree(bcsp->touchpad_pskey_2135);
	kfree(bcsp->touchpad_pskey_2136);
	kfree(bcsp->touchpad_pskey_2137);
	kfree(bcsp->touchpad_pskey_2138);
	kfree(bcsp->touchpad_pskey_2139);
	kfree(bcsp->touchpad_pskey_213a);
	kfree(bcsp->touchpad_pskey_21e1);
	kfree(bcsp->touchpad_pskey_2215);
	kfree(bcsp->touchpad_pskey_2216);
	kfree(bcsp->touchpad_pskey_2227);
	kfree(bcsp->touchpad_pskey_2228);
	kfree(bcsp->touchpad_pskey_2229);
	kfree(bcsp->touchpad_pskey_222a);
	kfree(bcsp->touchpad_pskey_222b);

	kfree(bcsp);
	return 0;
}

static const struct hci_uart_proto bcsp = {
	.id		= HCI_UART_BCSP,
	.name		= "BCSP",
	.open		= bcsp_open,
	.close		= bcsp_close,
	.setup		= bcsp_setup,
	.enqueue	= bcsp_enqueue,
	.dequeue	= bcsp_dequeue,
	.recv		= bcsp_recv,
	.flush		= bcsp_flush
};

/* ---- Serdev support for BCM4329 with BCSP protocol ---- */

#ifdef CONFIG_SERIAL_DEV_BUS

/*
 * Power control for BCM4329 Bluetooth chip
 *
 * GPIO signals (active high unless noted):
 * - shutdown_gpio: BT_REG_ON / BT_POWER - main power enable
 * - device_wakeup: BT_WAKE - wake signal to chip
 * - reset_gpio: BT_RST_N - active low reset
 */
static int bcsp_serdev_set_power(struct bcsp_serdev *bdev, bool powered)
{
	if (powered) {
		/* Power on sequence */
		if (bdev->shutdown_gpio)
			gpiod_set_value_cansleep(bdev->shutdown_gpio, 1);

		if (bdev->reset_gpio)
			gpiod_set_value_cansleep(bdev->reset_gpio, 0); /* Deassert reset */

		if (bdev->device_wakeup)
			gpiod_set_value_cansleep(bdev->device_wakeup, 1);

		/* Wait for chip to stabilize */
		msleep(100);
	} else {
		/* Power off sequence */
		if (bdev->device_wakeup)
			gpiod_set_value_cansleep(bdev->device_wakeup, 0);

		if (bdev->reset_gpio)
			gpiod_set_value_cansleep(bdev->reset_gpio, 1); /* Assert reset */

		if (bdev->shutdown_gpio)
			gpiod_set_value_cansleep(bdev->shutdown_gpio, 0);

		msleep(10);
	}

	return 0;
}

/*
 * Power cycle the chip - used after WARM_RESET to cleanly restart
 *
 * After power cycle, the chip boots at its default baud rate (115200).
 * We must reset our UART to init_speed to match, otherwise we get
 * baud mismatch and communication fails.
 */
static int bcsp_serdev_power_cycle(struct bcsp_serdev *bdev)
{
	dev_info(bdev->dev, "Power cycling Bluetooth chip...\n");

	/*
	 * Cancel any pending TX work and wait for completion.
	 * Clear TX state bits to ensure clean restart.
	 */
	cancel_work_sync(&bdev->serdev_hu.write_work);
	clear_bit(HCI_UART_SENDING, &bdev->serdev_hu.tx_state);
	clear_bit(HCI_UART_TX_WAKEUP, &bdev->serdev_hu.tx_state);

	bcsp_serdev_set_power(bdev, false);
	msleep(100);

	/*
	 * Flush TX buffer and reset baud rate.
	 * Note: We do NOT close/reopen serdev as that breaks TX path
	 * (serdev_device_write_buf returns 0 after reopen).
	 */
	serdev_device_write_flush(bdev->serdev_hu.serdev);
	serdev_device_set_baudrate(bdev->serdev_hu.serdev, bdev->init_speed);
	dev_info(bdev->dev, "Reset UART to %u baud\n", bdev->init_speed);

	bcsp_serdev_set_power(bdev, true);

	/*
	 * Wait for chip to fully initialize after power on.
	 * WebOS waits ~2-3 seconds here before expecting SYNC.
	 */
	msleep(2000);

	return 0;
}

static int bcsp_serdev_probe(struct serdev_device *serdev)
{
	struct bcsp_serdev *bdev;
	struct device *dev = &serdev->dev;
	int err;

	dev_info(dev, "BCSP serdev probe starting\n");

	bdev = devm_kzalloc(dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;

	bdev->dev = dev;
	bdev->serdev_hu.serdev = serdev;
	bdev->init_state = BCSP_SERDEV_INIT_POWER_OFF;
	serdev_device_set_drvdata(serdev, bdev);

	/* Get GPIO descriptors */
	bdev->shutdown_gpio = devm_gpiod_get_optional(dev, "shutdown",
						       GPIOD_OUT_LOW);
	if (IS_ERR(bdev->shutdown_gpio)) {
		dev_err(dev, "Failed to get shutdown GPIO: %ld\n",
			PTR_ERR(bdev->shutdown_gpio));
		return PTR_ERR(bdev->shutdown_gpio);
	}

	bdev->device_wakeup = devm_gpiod_get_optional(dev, "device-wakeup",
						       GPIOD_OUT_LOW);
	if (IS_ERR(bdev->device_wakeup)) {
		dev_err(dev, "Failed to get device-wakeup GPIO: %ld\n",
			PTR_ERR(bdev->device_wakeup));
		return PTR_ERR(bdev->device_wakeup);
	}

	bdev->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(bdev->reset_gpio)) {
		dev_err(dev, "Failed to get reset GPIO: %ld\n",
			PTR_ERR(bdev->reset_gpio));
		return PTR_ERR(bdev->reset_gpio);
	}

	/* Get UART speeds from DT */
	device_property_read_u32(dev, "init-speed", &bdev->init_speed);
	device_property_read_u32(dev, "max-speed", &bdev->oper_speed);

	if (!bdev->init_speed)
		bdev->init_speed = 115200;

	dev_info(dev, "BCM4329 BCSP Bluetooth, init-speed=%u\n", bdev->init_speed);

	/* Power on the chip */
	err = bcsp_serdev_set_power(bdev, true);
	if (err) {
		dev_err(dev, "Failed to power on\n");
		return err;
	}

	dev_info(dev, "Registering HCI UART device\n");

	/* Set init_speed so hci_uart_register_device_priv() can configure baud rate */
	bdev->serdev_hu.init_speed = bdev->init_speed;

	/* Register with HCI UART core */
	err = hci_uart_register_device(&bdev->serdev_hu, &bcsp);
	if (err) {
		dev_err(dev, "Failed to register HCI UART device: %d\n", err);
		bcsp_serdev_set_power(bdev, false);
		return err;
	}

	dev_info(dev, "HCI UART device registered successfully\n");

	/*
	 * Set up serdev tracking in bcsp_struct.
	 * This enables WARM_RESET handling with GPIO power cycling.
	 */
	if (bdev->serdev_hu.priv) {
		struct bcsp_struct *bcsp_priv = bdev->serdev_hu.priv;
		bcsp_priv->is_serdev = true;
		bcsp_priv->serdev_bdev = bdev;
		dev_info(dev, "BCSP serdev tracking initialized\n");
	}

	bdev->init_state = BCSP_SERDEV_INIT_PHASE1;

	return 0;
}

static void bcsp_serdev_remove(struct serdev_device *serdev)
{
	struct bcsp_serdev *bdev = serdev_device_get_drvdata(serdev);

	hci_uart_unregister_device(&bdev->serdev_hu);
	bcsp_serdev_set_power(bdev, false);
}

#ifdef CONFIG_OF
static const struct of_device_id bcsp_bluetooth_of_match[] = {
	/*
	 * Use palm-specific compatible to avoid conflict with hci_bcm
	 * which also matches "brcm,bcm4329-bt" but uses H4 protocol.
	 * The BCM4329 on HP TouchPad requires BCSP protocol.
	 */
	{ .compatible = "palm,bcm4329-bcsp" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcsp_bluetooth_of_match);
#endif

static struct serdev_device_driver bcsp_serdev_driver = {
	.probe = bcsp_serdev_probe,
	.remove = bcsp_serdev_remove,
	.driver = {
		.name = "hci_uart_bcsp",
		.of_match_table = of_match_ptr(bcsp_bluetooth_of_match),
	},
};

#endif /* CONFIG_SERIAL_DEV_BUS */

int __init bcsp_init(void)
{
	int err;

	err = hci_uart_register_proto(&bcsp);
	if (err)
		return err;

#ifdef CONFIG_SERIAL_DEV_BUS
	serdev_device_driver_register(&bcsp_serdev_driver);
#endif

	return 0;
}

int __exit bcsp_deinit(void)
{
#ifdef CONFIG_SERIAL_DEV_BUS
	serdev_device_driver_unregister(&bcsp_serdev_driver);
#endif

	return hci_uart_unregister_proto(&bcsp);
}

module_param(txcrc, bool, 0644);
MODULE_PARM_DESC(txcrc, "Transmit CRC with every BCSP packet");

module_param(hciextn, bool, 0644);
MODULE_PARM_DESC(hciextn, "Convert HCI Extensions into BCSP packets");

module_param(bdaddr, charp, 0444);
MODULE_PARM_DESC(bdaddr, "Bluetooth device address (XX:XX:XX:XX:XX:XX)");

module_param(skip_pskeys, bool, 0644);
MODULE_PARM_DESC(skip_pskeys, "Skip PSKEY configuration (for debugging)");
