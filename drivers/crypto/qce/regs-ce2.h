/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2026, Herrie <pherrie@gmail.com>
 *
 * Register definitions for Qualcomm Crypto Engine 2 (CE2)
 * Found on MSM8660/APQ8060 and similar SoCs
 */

#ifndef _REGS_CE2_H_
#define _REGS_CE2_H_

#include <linux/bitops.h>

/* CE2 Register offsets - different from CE v5 */
#define CE2_REG_DATA_IN			0x000
#define CE2_REG_DATA_OUT		0x010
#define CE2_REG_STATUS			0x020
#define CE2_REG_CONFIG			0x024
#define CE2_REG_DEBUG			0x028
#define CE2_REG_REGISTER_LOCK		0x02C
#define CE2_REG_SEG_CFG			0x030
#define CE2_REG_ENCR_SEG_CFG		0x034
#define CE2_REG_AUTH_SEG_CFG		0x038
#define CE2_REG_SEG_SIZE		0x03C
#define CE2_REG_GOPROC			0x040
#define CE2_REG_ENGINES_AVAIL		0x044

/* DES key registers */
#define CE2_REG_DES_KEY0		0x050
#define CE2_REG_DES_KEY1		0x054
#define CE2_REG_DES_KEY2		0x058
#define CE2_REG_DES_KEY3		0x05C
#define CE2_REG_DES_KEY4		0x060
#define CE2_REG_DES_KEY5		0x064

/* Counter/IV registers */
#define CE2_REG_CNTR0_IV0		0x070
#define CE2_REG_CNTR1_IV1		0x074
#define CE2_REG_CNTR2_IV2		0x078
#define CE2_REG_CNTR3_IV3		0x07C
#define CE2_REG_CNTR_MASK		0x080

/* Auth byte count registers */
#define CE2_REG_AUTH_BYTECNT0		0x090
#define CE2_REG_AUTH_BYTECNT1		0x094
#define CE2_REG_AUTH_BYTECNT2		0x098
#define CE2_REG_AUTH_BYTECNT3		0x09C

/* Auth IV registers */
#define CE2_REG_AUTH_IV0		0x100
#define CE2_REG_AUTH_IV1		0x104
#define CE2_REG_AUTH_IV2		0x108
#define CE2_REG_AUTH_IV3		0x10C
#define CE2_REG_AUTH_IV4		0x110
#define CE2_REG_AUTH_IV5		0x114
#define CE2_REG_AUTH_IV6		0x118
#define CE2_REG_AUTH_IV7		0x11C
#define CE2_REG_AUTH_IV8		0x120
#define CE2_REG_AUTH_IV9		0x124
#define CE2_REG_AUTH_IV10		0x128
#define CE2_REG_AUTH_IV11		0x12C
#define CE2_REG_AUTH_IV12		0x130
#define CE2_REG_AUTH_IV13		0x134
#define CE2_REG_AUTH_IV14		0x138
#define CE2_REG_AUTH_IV15		0x13C

/* AES round key registers (60 words) */
#define CE2_REG_AES_RNDKEY0		0x200
#define CE2_REG_AES_RNDKEY_LAST		0x2EC

/* Data shadow registers */
#define CE2_REG_DATA_SHADOW0		0x8000
#define CE2_REG_DATA_SHADOW_LAST	0x8FFC

/* Register bits - CE2_REG_STATUS */
#define CE2_CORE_REV_SHIFT		28
#define CE2_CORE_REV_MASK		GENMASK(31, 28)
#define CE2_DOUT_SIZE_AVAIL_SHIFT	22
#define CE2_DOUT_SIZE_AVAIL_MASK	GENMASK(24, 22)
#define CE2_DIN_SIZE_AVAIL_SHIFT	19
#define CE2_DIN_SIZE_AVAIL_MASK		GENMASK(21, 19)
#define CE2_ACCESS_VIOL_SHIFT		18
#define CE2_SEG_CHNG_ERR_SHIFT		17
#define CE2_CFG_CHNG_ERR_SHIFT		16
#define CE2_DOUT_ERR_SHIFT		15
#define CE2_DIN_ERR_SHIFT		14
#define CE2_LOCKED_SHIFT		13
#define CE2_CRYPTO_STATE_SHIFT		10
#define CE2_CRYPTO_STATE_MASK		GENMASK(12, 10)
#define CE2_ENCR_BUSY_SHIFT		9
#define CE2_AUTH_BUSY_SHIFT		8
#define CE2_DOUT_INTR_SHIFT		7
#define CE2_DIN_INTR_SHIFT		6
#define CE2_AUTH_DONE_INTR_SHIFT	5
#define CE2_ERR_INTR_SHIFT		4
#define CE2_DOUT_RDY_SHIFT		3
#define CE2_DIN_RDY_SHIFT		2
#define CE2_AUTH_DONE_SHIFT		1
#define CE2_SW_ERR_SHIFT		0

/* Crypto state values */
#define CE2_CRYPTO_STATE_IDLE		0
#define CE2_CRYPTO_STATE_LOCKED		1
#define CE2_CRYPTO_STATE_GO		3
#define CE2_CRYPTO_STATE_PROCESSING	4
#define CE2_CRYPTO_STATE_FINAL_READ	5
#define CE2_CRYPTO_STATE_CTXT_CLEARING	6
#define CE2_CRYPTO_STATE_UNLOCKING	7

/* Register bits - CE2_REG_CONFIG */
#define CE2_HIGH_SPD_HASH_EN_N_SHIFT	15
#define CE2_HIGH_SPD_OUT_EN_N_SHIFT	14
#define CE2_HIGH_SPD_IN_EN_N_SHIFT	13
#define CE2_DBG_EN_SHIFT		12
#define CE2_DBG_SEL_SHIFT		7
#define CE2_DBG_SEL_MASK		GENMASK(11, 7)
#define CE2_MASK_DOUT_INTR_SHIFT	6
#define CE2_MASK_DIN_INTR_SHIFT		5
#define CE2_MASK_AUTH_DONE_INTR_SHIFT	4
#define CE2_MASK_ERR_INTR_SHIFT		3
#define CE2_AUTO_SHUTDOWN_EN_SHIFT	2
#define CE2_CLK_EN_N_SHIFT		1
#define CE2_SW_RST_SHIFT		0

/* Register bits - CE2_REG_SEG_CFG (combined segment config) */
#define CE2_F8_KEYSTREAM_ENABLE_SHIFT	25
#define CE2_F9_DIRECTION_SHIFT		24
#define CE2_F8_DIRECTION_SHIFT		23
#define CE2_USE_HW_KEY_SHIFT		22
#define CE2_CNTR_ALG_SHIFT		20
#define CE2_CNTR_ALG_MASK		GENMASK(21, 20)
#define CE2_CLR_CNTXT_SHIFT		19
#define CE2_LAST_SHIFT			18
#define CE2_FIRST_SHIFT			17
#define CE2_ENCODE_SHIFT		16
#define CE2_AUTH_POS_SHIFT		14
#define CE2_AUTH_POS_MASK		GENMASK(15, 14)
#define CE2_AUTH_SIZE_SHIFT		11
#define CE2_AUTH_SIZE_MASK		GENMASK(13, 11)
#define CE2_AUTH_ALG_SHIFT		9
#define CE2_AUTH_ALG_MASK		GENMASK(10, 9)
#define CE2_ENCR_MODE_SHIFT		6
#define CE2_ENCR_MODE_MASK		GENMASK(8, 6)
#define CE2_ENCR_KEY_SZ_SHIFT		3
#define CE2_ENCR_KEY_SZ_MASK		GENMASK(5, 3)
#define CE2_ENCR_ALG_SHIFT		0
#define CE2_ENCR_ALG_MASK		GENMASK(2, 0)

/* Counter algorithm values */
#define CE2_CNTR_ALG_NIST		0
#define CE2_CNTR_ALG_UMB		1
#define CE2_CNTR_ALG_VAR2		2

/* Auth position values */
#define CE2_AUTH_POS_BEFORE		0
#define CE2_AUTH_POS_AFTER		1

/* Auth size values for SHA */
#define CE2_AUTH_SIZE_SHA1		0
#define CE2_AUTH_SIZE_SHA256		1
#define CE2_AUTH_SIZE_SHA384		2
#define CE2_AUTH_SIZE_SHA512		3
#define CE2_AUTH_SIZE_HMAC_SHA1		4

/* Auth algorithm values */
#define CE2_AUTH_ALG_NONE		0
#define CE2_AUTH_ALG_SHA		1
#define CE2_AUTH_ALG_F9			2

/* Encryption mode values */
#define CE2_ENCR_MODE_ECB		0
#define CE2_ENCR_MODE_CBC		1
#define CE2_ENCR_MODE_CTR		2

/* Encryption key size values for DES */
#define CE2_ENCR_KEY_SZ_DES		0
#define CE2_ENCR_KEY_SZ_3DES		1

/* Encryption key size values for AES */
#define CE2_ENCR_KEY_SZ_AES128		0
#define CE2_ENCR_KEY_SZ_AES192		1
#define CE2_ENCR_KEY_SZ_AES256		2

/* Encryption algorithm values */
#define CE2_ENCR_ALG_NONE		0
#define CE2_ENCR_ALG_DES		1
#define CE2_ENCR_ALG_AES		2
#define CE2_ENCR_ALG_C2			3
#define CE2_ENCR_ALG_F8			4

/* Register bits - CE2_REG_ENCR_SEG_CFG */
#define CE2_ENCR_SEG_SIZE_SHIFT		16
#define CE2_ENCR_SEG_SIZE_MASK		GENMASK(31, 16)
#define CE2_ENCR_START_SHIFT		0
#define CE2_ENCR_START_MASK		GENMASK(15, 0)

/* Register bits - CE2_REG_AUTH_SEG_CFG */
#define CE2_AUTH_SEG_SIZE_SHIFT		16
#define CE2_AUTH_SEG_SIZE_MASK		GENMASK(31, 16)
#define CE2_AUTH_START_SHIFT		0
#define CE2_AUTH_START_MASK		GENMASK(15, 0)

/* Register bits - CE2_REG_SEG_SIZE */
#define CE2_SEG_SIZE_SHIFT		0
#define CE2_SEG_SIZE_MASK		GENMASK(15, 0)

/* Register bits - CE2_REG_GOPROC */
#define CE2_GO_SHIFT			0

/* Register bits - CE2_REG_ENGINES_AVAIL */
#define CE2_F9_SEL_SHIFT		8
#define CE2_F8_SEL_SHIFT		7
#define CE2_HMAC_SEL_SHIFT		6
#define CE2_SHA512_SEL_SHIFT		5
#define CE2_SHA_SEL_SHIFT		4
#define CE2_DES_SEL_SHIFT		3
#define CE2_C2_SEL_SHIFT		2
#define CE2_AES_SEL_SHIFT		0
#define CE2_AES_SEL_MASK		GENMASK(1, 0)
#define CE2_AES_SEL_NO			0
#define CE2_AES_SEL_SLOW		1
#define CE2_AES_SEL_FAST		2

/* Misc constants */
#define CE2_AES_RNDKEYS			60

/* CE2 burst size - ADM uses smaller bursts than BAM */
#define CE2_ADM_BURST_SIZE		64

#endif /* _REGS_CE2_H_ */
