// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  Bluetooth HCI UART driver
 *
 *  Copyright (C) 2002-2003  Fabrizio Gennari <fabrizio.gennari@philips.com>
 *  Copyright (C) 2004-2005  Marcel Holtmann <marcel@holtmann.org>
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

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

static bool txcrc = true;
static bool hciextn = true;
static char *bdaddr;

#define BCSP_TXWINSIZE	4

#define BCSP_ACK_PKT	0x05
#define BCSP_LE_PKT	0x06
#define BCSP_BCCMD_PKT	0x07

/* BCCMD constants */
#define BCCMD_SETREQ		0x0002
#define BCCMD_VARID_PS		0x7003
#define BCCMD_VARID_WARM_RESET	0x4002

/* PSKEY definitions for CSR chip configuration */
#define PSKEY_BDADDR			0x0001
#define PSKEY_ENC_KEY_LMIN		0x000E	/* Min encryption key length */
#define PSKEY_LC_MAX_TX_POWER		0x0011	/* Maximum TX power level */
#define PSKEY_LC_DEFAULT_TX_POWER	0x0013	/* Default TX power level */
#define PSKEY_HOST_INTERFACE		0x01FE	/* Host interface config */
#define PSKEY_PCM_MIN_CPU_CLOCK		0x01BE	/* PCM minimum CPU clock */
#define PSKEY_H_HC_FC_MAX_ACL		0x01AB	/* HCI FC max ACL packets */
#define PSKEY_H_HC_FC_MAX_SCO		0x01B0	/* HCI FC max SCO packets */
#define PSKEY_PCM_SAMPLE_SIZE		0x01B9	/* PCM sample size */
#define PSKEY_ANA_FREQ			0x01F6	/* External crystal frequency */
#define PSKEY_XTAL_FTRIM		0x01F9	/* Crystal fine trim */
#define PSKEY_LC_MAX_TX_POWER_NO_RSSI	0x024D	/* Max TX power without RSSI */
#define PSKEY_LC_DEFAULT_TX_POWER_NO_RSSI 0x025D /* Default TX power without RSSI */

/* Crystal frequency value for 26MHz external crystal */
#define ANA_FREQ_26MHZ			0x0019

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
};

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

	if (bcsp->use_crc)
		hdr[0] |= 0x40;

	hdr[1] = ((len << 4) & 0xff) | chan;
	hdr[2] = len >> 4;
	hdr[3] = ~(hdr[0] + hdr[1] + hdr[2]);

	/* Put BCSP header */
	for (i = 0; i < 4; i++) {
		bcsp_slip_one_byte(nskb, hdr[i]);

		if (bcsp->use_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, hdr[i]);
	}

	/* Put payload */
	for (i = 0; i < len; i++) {
		bcsp_slip_one_byte(nskb, data[i]);

		if (bcsp->use_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, data[i]);
	}

	/* Put CRC */
	if (bcsp->use_crc) {
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

		nskb = bcsp_prepare_pkt(bcsp, skb->data, skb->len,
					hci_skb_pkt_type(skb));
		if (nskb) {
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

	if (skb_queue_empty(&bcsp->unack))
		timer_delete(&bcsp->tbcsp);

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	if (i != pkts_to_be_removed)
		BT_ERR("Removed only %u out of %u pkts", i, pkts_to_be_removed);
}

/* Forward declarations for BCCMD functions used in link establishment */
static int bcsp_send_bdaddr_bccmd(struct hci_uart *hu, bdaddr_t *addr);
static int bcsp_send_warm_reset(struct hci_uart *hu);
static int bcsp_send_pskey_word(struct hci_uart *hu, u16 pskey, u16 value);
static void bcsp_reset_link_state(struct bcsp_struct *bcsp);

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
	BT_DBG("BCSP LE pkt: hdr[0]=%02x hdr[1]=%02x hdr[2]=%02x len=%d",
	       bcsp->rx_skb->data[0], bcsp->rx_skb->data[1],
	       bcsp->rx_skb->data[2], (len_high << 4) | len_nibble);

	/* Check packet has 4-byte payload (link establishment packets) */
	if (len_nibble != 4 || len_high != 0) {
		BT_DBG("BCSP LE pkt: not a 4-byte LE pkt, ignoring");
		return;
	}

	/* Handle sync packet - device is starting link establishment */
	if (!memcmp(&bcsp->rx_skb->data[4], sync_pkt, 4)) {
		struct sk_buff *nskb = alloc_skb(4, GFP_ATOMIC);

		BT_INFO("BCSP: sync received, responding with sync_rsp");
		if (!nskb)
			return;
		skb_put_data(nskb, sync_rsp_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		skb_queue_head(&bcsp->unrel, nskb);
		hci_uart_tx_wakeup(hu);
	}
	/* Handle sync_rsp packet - device acknowledged our sync, send conf */
	else if (!memcmp(&bcsp->rx_skb->data[4], sync_rsp_pkt, 4)) {
		struct sk_buff *nskb = alloc_skb(4, GFP_ATOMIC);

		BT_INFO("BCSP: sync_rsp received, sending conf");
		if (!nskb)
			return;
		skb_put_data(nskb, conf_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		skb_queue_head(&bcsp->unrel, nskb);
		hci_uart_tx_wakeup(hu);
	}
	/* Handle conf packet - device sent config, reply with conf_rsp */
	else if (!memcmp(&bcsp->rx_skb->data[4], conf_pkt, 4)) {
		struct sk_buff *nskb = alloc_skb(4, GFP_ATOMIC);

		BT_INFO("BCSP: conf received, responding with conf_rsp (bdaddr_state=%d)",
			bcsp->bdaddr_state);
		if (!nskb)
			return;
		skb_put_data(nskb, conf_rsp_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		skb_queue_head(&bcsp->unrel, nskb);
		hci_uart_tx_wakeup(hu);

		/*
		 * After sending conf_rsp, the link is established.
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
		BT_INFO("BCSP: conf_rsp received, link established (bdaddr_state=%d)",
			bcsp->bdaddr_state);

		if (bcsp->bdaddr_state == BCSP_BDADDR_PENDING) {
			BT_INFO("BCSP: First link up, sending RF PSKEYs (no reset)");

			/* Send critical RF configuration PSKEYs */
			bcsp_send_pskey_word(hu, PSKEY_ANA_FREQ, ANA_FREQ_26MHZ);
			bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER, 0x0154);
			bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER, 0x000B);

			/* Skip BD address and WARM_RESET - test if RF works without */
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
			BT_INFO("BCSP: LE packet on channel 1, calling handler");
			bcsp_handle_le_pkt(hu);
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
	bccmd[14] = 0x08;		/* Stores: PSRAM */
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
	bccmd[14] = 0x08;	/* Stores: 0x0008 PSRAM (low byte) */
	bccmd[15] = 0x00;	/* Stores: 0x0008 PSRAM (high byte) */

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

/*
 * Reset BCSP link state to prepare for re-establishment
 * Note: Does NOT purge unrel queue as it may contain outgoing BCCMDs
 */
static void bcsp_reset_link_state(struct bcsp_struct *bcsp)
{
	/* Clear reliable packet queues (no ACKs will come after reset) */
	skb_queue_purge(&bcsp->unack);
	skb_queue_purge(&bcsp->rel);
	/* Do NOT purge unrel - it may contain pending BCCMDs we need to send */

	/* Reset sequence numbers */
	bcsp->rxseq_txack = 0;
	bcsp->rxack = 0;
	bcsp->msgq_txseq = 0;

	/* Reset other state */
	bcsp->txack_req = 0;
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
		BT_INFO("BCSP: Configuring chip with PSKEYs + WARM_RESET");

		/*
		 * Send all 12 critical PSKEYs and BD address, then WARM_RESET.
		 * The chip will reset and come back with new settings.
		 * Based on bcattach analysis from webOS-ports/utilities.
		 */

		/* 1. Host interface - BCSP mode configuration */
		bcsp_send_pskey_word(hu, PSKEY_HOST_INTERFACE, 0x6590);
		msleep(50);

		/* 2. PCM minimum CPU clock */
		bcsp_send_pskey_word(hu, PSKEY_PCM_MIN_CPU_CLOCK, 0x3AFC);
		msleep(50);

		/* 3. HCI flow control - max ACL packets */
		bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_ACL, 0x0001);
		msleep(50);

		/* 4. HCI flow control - max SCO packets */
		bcsp_send_pskey_word(hu, PSKEY_H_HC_FC_MAX_SCO, 0x0001);
		msleep(50);

		/* 5. PCM sample size */
		bcsp_send_pskey_word(hu, PSKEY_PCM_SAMPLE_SIZE, 0x0008);
		msleep(50);

		/* 6. PSKEY_ANA_FREQ = 25 (0x19) for 26MHz crystal - CRITICAL */
		bcsp_send_pskey_word(hu, PSKEY_ANA_FREQ, ANA_FREQ_26MHZ);
		msleep(50);

		/* 7. Max TX power level */
		bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER, 0x0154);
		msleep(50);

		/* 8. Default TX power level */
		bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER, 0x000B);
		msleep(50);

		/* 9. Max TX power without RSSI */
		bcsp_send_pskey_word(hu, PSKEY_LC_MAX_TX_POWER_NO_RSSI, 0x0000);
		msleep(50);

		/* 10. Minimum encryption key length */
		bcsp_send_pskey_word(hu, PSKEY_ENC_KEY_LMIN, 0x0001);
		msleep(50);

		/* 11. Crystal fine trim - important for frequency accuracy */
		bcsp_send_pskey_word(hu, PSKEY_XTAL_FTRIM, 0x0001);
		msleep(50);

		/* 12. Default TX power without RSSI */
		bcsp_send_pskey_word(hu, PSKEY_LC_DEFAULT_TX_POWER_NO_RSSI, 0x0001);
		msleep(50);

		/* BD address */
		BT_INFO("BCSP: Setting BD address %pMR", &bcsp->bdaddr);
		bcsp_send_bdaddr_bccmd(hu, &bcsp->bdaddr);
		msleep(50);

		/* WARM_RESET to apply all PSKEYs */
		bcsp_send_warm_reset(hu);
		msleep(50);

		/* Give TX time to send the packets */
		for (i = 0; i < 10; i++) {
			if (skb_queue_empty(&bcsp->unrel))
				break;
			msleep(50);
			hci_uart_tx_wakeup(hu);
		}

		BT_INFO("BCSP: WARM_RESET sent, waiting for chip to reset...");

		/*
		 * Wait for chip to reset. Testing shows the BCM4329 takes
		 * approximately 21 seconds from WARM_RESET to sending sync.
		 * Use 15s initial wait + 10s polling loop = 25s total.
		 */
		msleep(15000);

		/* Reset our link state for re-establishment */
		bcsp_reset_link_state(bcsp);

		BT_INFO("BCSP: Waiting for link re-establishment...");
		bcsp->bdaddr_state = BCSP_BDADDR_SENT;

		for (i = 0; i < 100; i++) {
			msleep(100);
			if (bcsp->bdaddr_state == BCSP_BDADDR_DONE) {
				BT_INFO("BCSP: Link re-established");
				break;
			}
		}

		if (bcsp->bdaddr_state != BCSP_BDADDR_DONE) {
			BT_WARN("BCSP: Link timeout, continuing anyway");
			bcsp->bdaddr_state = BCSP_BDADDR_DONE;
		}

		/* Extra stabilization delay */
		msleep(2000);
		BT_INFO("BCSP: PSKEY configuration complete");
	}

	return 0;
}

	/* Arrange to retransmit all messages in the relq. */
static void bcsp_timed_event(struct timer_list *t)
{
	struct bcsp_struct *bcsp = timer_container_of(bcsp, t, tbcsp);
	struct hci_uart *hu = bcsp->hu;
	struct sk_buff *skb;
	unsigned long flags;

	BT_DBG("hu %p retransmitting %u pkts", hu, bcsp->unack.qlen);

	spin_lock_irqsave_nested(&bcsp->unack.lock, flags, SINGLE_DEPTH_NESTING);

	while ((skb = __skb_dequeue_tail(&bcsp->unack)) != NULL) {
		bcsp->msgq_txseq = (bcsp->msgq_txseq - 1) & 0x07;
		skb_queue_head(&bcsp->rel, skb);
	}

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	hci_uart_tx_wakeup(hu);
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

	timer_setup(&bcsp->tbcsp, bcsp_timed_event, 0);

	bcsp->rx_state = BCSP_W4_PKT_DELIMITER;

	if (txcrc)
		bcsp->use_crc = 1;

	/* Initialize BD address configuration state */
	bcsp->bdaddr_state = BCSP_BDADDR_NONE;
	if (bdaddr && bdaddr[0]) {
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

int __init bcsp_init(void)
{
	return hci_uart_register_proto(&bcsp);
}

int __exit bcsp_deinit(void)
{
	return hci_uart_unregister_proto(&bcsp);
}

module_param(txcrc, bool, 0644);
MODULE_PARM_DESC(txcrc, "Transmit CRC with every BCSP packet");

module_param(hciextn, bool, 0644);
MODULE_PARM_DESC(hciextn, "Convert HCI Extensions into BCSP packets");

module_param(bdaddr, charp, 0444);
MODULE_PARM_DESC(bdaddr, "Bluetooth device address (XX:XX:XX:XX:XX:XX)");
