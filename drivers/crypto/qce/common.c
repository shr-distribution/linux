// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 */

#include <crypto/internal/hash.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <crypto/aes.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>

#include <linux/dma/qcom_adm.h>

#include "cipher.h"
#include "common.h"
#include "core.h"
#include "dma.h"
#include "regs-v5.h"
#include "regs-ce2.h"
#include "sha.h"
#include "aead.h"

/*
 * CE2 register offset translation
 * CE2 has different register layout than v5
 */
static inline u32 qce_reg_status(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_STATUS : REG_STATUS;
}

static inline u32 qce_reg_config(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_CONFIG : REG_CONFIG;
}

static inline u32 qce_reg_goproc(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_GOPROC : REG_GOPROC;
}

static inline u32 qce_reg_seg_size(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_SEG_SIZE : REG_SEG_SIZE;
}

static inline u32 qce_reg_encr_seg_cfg(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_ENCR_SEG_CFG : REG_ENCR_SEG_CFG;
}

static inline u32 qce_reg_encr_seg_size(struct qce_device *qce)
{
	/* CE2 combines size and start in ENCR_SEG_CFG */
	return qce_is_ce2(qce) ? CE2_REG_ENCR_SEG_CFG : REG_ENCR_SEG_SIZE;
}

static inline u32 qce_reg_auth_seg_cfg(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_AUTH_SEG_CFG : REG_AUTH_SEG_CFG;
}

static inline u32 qce_reg_auth_seg_size(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_AUTH_SEG_CFG : REG_AUTH_SEG_SIZE;
}

static inline u32 qce_reg_cntr0_iv0(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_CNTR0_IV0 : REG_CNTR0_IV0;
}

static inline u32 qce_reg_cntr_mask(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_CNTR_MASK : REG_CNTR_MASK;
}

static inline u32 qce_reg_auth_iv0(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_AUTH_IV0 : REG_AUTH_IV0;
}

static inline u32 qce_reg_auth_bytecnt0(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_AUTH_BYTECNT0 : REG_AUTH_BYTECNT0;
}

/*
 * CE2 uses AES round key registers at 0x200 for encryption keys.
 * The key needs to be expanded before being written.
 * For simplicity, we use the DES key registers for DES/3DES
 * and round key registers for AES.
 */
static inline u32 qce_reg_encr_key0(struct qce_device *qce)
{
	/* For CE2, AES keys go to round key registers */
	return qce_is_ce2(qce) ? CE2_REG_AES_RNDKEY0 : REG_ENCR_KEY0;
}

static inline u32 qce_reg_des_key0(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_DES_KEY0 : REG_ENCR_KEY0;
}

/* CE2 doesn't have separate auth key registers - uses auth IV */
static inline u32 qce_reg_auth_key0(struct qce_device *qce)
{
	return qce_is_ce2(qce) ? CE2_REG_AUTH_IV0 : REG_AUTH_KEY0;
}

static inline u32 qce_read(struct qce_device *qce, u32 offset)
{
	return readl(qce->base + offset);
}

static inline void qce_write(struct qce_device *qce, u32 offset, u32 val)
{
	writel(val, qce->base + offset);
}

static inline void qce_write_array(struct qce_device *qce, u32 offset,
				   const u32 *val, unsigned int len)
{
	int i;

	for (i = 0; i < len; i++)
		qce_write(qce, offset + i * sizeof(u32), val[i]);
}

static inline void
qce_clear_array(struct qce_device *qce, u32 offset, unsigned int len)
{
	int i;

	for (i = 0; i < len; i++)
		qce_write(qce, offset + i * sizeof(u32), 0);
}

static u32 qce_config_reg(struct qce_device *qce, int little)
{
	u32 config;

	if (qce_is_ce2(qce)) {
		/*
		 * CE2 config register has different bit layout:
		 * - No pipe pair select (uses ADM, not BAM)
		 * - No request size (ADM handles this)
		 * - Mask interrupts in config register
		 *
		 * CRITICAL: Read current CONFIG to preserve CLK_EN_N (bit 1)
		 * and SW_RST (bit 0) clear state from initialization.
		 */
		config = qce_read(qce, qce_reg_config(qce));
		/* Set interrupt masks */
		config |= BIT(CE2_MASK_DOUT_INTR_SHIFT) |
			  BIT(CE2_MASK_DIN_INTR_SHIFT) |
			  BIT(CE2_MASK_AUTH_DONE_INTR_SHIFT) |
			  BIT(CE2_MASK_ERR_INTR_SHIFT);
		/* Enable high speed mode */
		config &= ~(BIT(CE2_HIGH_SPD_IN_EN_N_SHIFT) |
			    BIT(CE2_HIGH_SPD_OUT_EN_N_SHIFT) |
			    BIT(CE2_HIGH_SPD_HASH_EN_N_SHIFT));
		/* Ensure CLK_EN_N (bit 1) and SW_RST (bit 0) stay cleared */
		config &= ~(BIT(1) | BIT(0));
		/* Note: CE2 doesn't have little endian mode bit */
	} else {
		u32 beats = (qce->burst_size >> 3) - 1;
		u32 pipe_pair = qce->pipe_pair_id;

		config = (beats << REQ_SIZE_SHIFT) & REQ_SIZE_MASK;
		config |= BIT(MASK_DOUT_INTR_SHIFT) | BIT(MASK_DIN_INTR_SHIFT) |
			  BIT(MASK_OP_DONE_INTR_SHIFT) | BIT(MASK_ERR_INTR_SHIFT);
		config |= (pipe_pair << PIPE_SET_SELECT_SHIFT) & PIPE_SET_SELECT_MASK;
		config &= ~HIGH_SPD_EN_N_SHIFT;

		if (little)
			config |= BIT(LITTLE_ENDIAN_MODE_SHIFT);
	}

	return config;
}

void qce_cpu_to_be32p_array(__be32 *dst, const u8 *src, unsigned int len)
{
	__be32 *d = dst;
	const u8 *s = src;
	unsigned int n;

	n = len / sizeof(u32);
	for (; n > 0; n--) {
		*d = cpu_to_be32p((const __u32 *) s);
		s += sizeof(__u32);
		d++;
	}
}

static void qce_setup_config(struct qce_device *qce)
{
	u32 config;

	/* get big endianness */
	config = qce_config_reg(qce, 0);

	/* clear status */
	qce_write(qce, qce_reg_status(qce), 0);
	qce_write(qce, qce_reg_config(qce), config);
}

static inline void qce_crypto_go(struct qce_device *qce, bool result_dump)
{
	u32 val;

	if (qce_is_ce2(qce)) {
		/* CE2 just has a GO bit at position 0 */
		qce_write(qce, qce_reg_goproc(qce), BIT(CE2_GO_SHIFT));
	} else {
		if (result_dump)
			val = BIT(GO_SHIFT) | BIT(RESULTS_DUMP_SHIFT);
		else
			val = BIT(GO_SHIFT);
		qce_write(qce, qce_reg_goproc(qce), val);
	}
}

#if defined(CONFIG_CRYPTO_DEV_QCE_SHA) || defined(CONFIG_CRYPTO_DEV_QCE_AEAD)
/*
 * CE2 SEG_CFG bit layout differs from v5 (see regs-ce2.h):
 *   AUTH_ALG at bit 9-10 (v5: 0-2)
 *   AUTH_SIZE at bit 11-13 (v5: 9-13)
 *   AUTH_POS at bit 14-15 (same)
 *   FIRST at bit 17 (same)
 *   LAST at bit 18 (v5: 16)
 * Using v5 constants on CE2 silently maps SHA256 size to bit 9 where CE2
 * reads only AUTH_ALG -> SHA256 collapses to SHA1 (value 0 at bit 11-13).
 */
static u32 qce_auth_cfg_ce2(unsigned long flags)
{
	u32 cfg = 0;

	cfg |= CE2_AUTH_ALG_SHA << CE2_AUTH_ALG_SHIFT;

	if (IS_SHA1(flags) || IS_SHA1_HMAC(flags))
		cfg |= CE2_AUTH_SIZE_SHA1 << CE2_AUTH_SIZE_SHIFT;
	else if (IS_SHA256(flags) || IS_SHA256_HMAC(flags))
		cfg |= CE2_AUTH_SIZE_SHA256 << CE2_AUTH_SIZE_SHIFT;

	if (IS_SHA(flags) || IS_SHA_HMAC(flags))
		cfg |= CE2_AUTH_POS_BEFORE << CE2_AUTH_POS_SHIFT;

	return cfg;
}

static u32 qce_auth_cfg(struct qce_device *qce, unsigned long flags,
			u32 key_size, u32 auth_size)
{
	u32 cfg = 0;

	if (qce_is_ce2(qce))
		return qce_auth_cfg_ce2(flags);

	if (IS_CCM(flags) || IS_CMAC(flags))
		cfg |= AUTH_ALG_AES << AUTH_ALG_SHIFT;
	else
		cfg |= AUTH_ALG_SHA << AUTH_ALG_SHIFT;

	if (IS_CCM(flags) || IS_CMAC(flags)) {
		if (key_size == AES_KEYSIZE_128)
			cfg |= AUTH_KEY_SZ_AES128 << AUTH_KEY_SIZE_SHIFT;
		else if (key_size == AES_KEYSIZE_256)
			cfg |= AUTH_KEY_SZ_AES256 << AUTH_KEY_SIZE_SHIFT;
	}

	if (IS_SHA1(flags) || IS_SHA1_HMAC(flags))
		cfg |= AUTH_SIZE_SHA1 << AUTH_SIZE_SHIFT;
	else if (IS_SHA256(flags) || IS_SHA256_HMAC(flags))
		cfg |= AUTH_SIZE_SHA256 << AUTH_SIZE_SHIFT;
	else if (IS_CMAC(flags))
		cfg |= AUTH_SIZE_ENUM_16_BYTES << AUTH_SIZE_SHIFT;
	else if (IS_CCM(flags))
		cfg |= (auth_size - 1) << AUTH_SIZE_SHIFT;

	if (IS_SHA1(flags) || IS_SHA256(flags))
		cfg |= AUTH_MODE_HASH << AUTH_MODE_SHIFT;
	else if (IS_SHA1_HMAC(flags) || IS_SHA256_HMAC(flags))
		cfg |= AUTH_MODE_HMAC << AUTH_MODE_SHIFT;
	else if (IS_CCM(flags))
		cfg |= AUTH_MODE_CCM << AUTH_MODE_SHIFT;
	else if (IS_CMAC(flags))
		cfg |= AUTH_MODE_CMAC << AUTH_MODE_SHIFT;

	if (IS_SHA(flags) || IS_SHA_HMAC(flags))
		cfg |= AUTH_POS_BEFORE << AUTH_POS_SHIFT;

	if (IS_CCM(flags))
		cfg |= QCE_MAX_NONCE_WORDS << AUTH_NONCE_NUM_WORDS_SHIFT;

	return cfg;
}
#endif

#ifdef CONFIG_CRYPTO_DEV_QCE_SHA
static int qce_setup_regs_ahash(struct crypto_async_request *async_req)
{
	struct ahash_request *req = ahash_request_cast(async_req);
	struct crypto_ahash *ahash = __crypto_ahash_cast(async_req->tfm);
	struct qce_sha_reqctx *rctx = ahash_request_ctx_dma(req);
	struct qce_alg_template *tmpl = to_ahash_tmpl(async_req->tfm);
	struct qce_device *qce = tmpl->qce;
	unsigned int digestsize = crypto_ahash_digestsize(ahash);
	unsigned int blocksize = crypto_tfm_alg_blocksize(async_req->tfm);
	__be32 auth[SHA256_DIGEST_SIZE / sizeof(__be32)] = {0};
	__be32 mackey[QCE_SHA_HMAC_KEY_SIZE / sizeof(__be32)] = {0};
	u32 auth_cfg = 0, config;
	unsigned int iv_words;

	/* if not the last, the size has to be on the block boundary */
	if (!rctx->last_blk && req->nbytes % blocksize)
		return -EINVAL;

	qce_setup_config(qce);

	if (IS_CMAC(rctx->flags)) {
		qce_write(qce, REG_AUTH_SEG_CFG, 0);
		qce_write(qce, REG_ENCR_SEG_CFG, 0);
		qce_write(qce, REG_ENCR_SEG_SIZE, 0);
		qce_clear_array(qce, REG_AUTH_IV0, 16);
		qce_clear_array(qce, REG_AUTH_KEY0, 16);
		qce_clear_array(qce, REG_AUTH_BYTECNT0, 4);

		auth_cfg = qce_auth_cfg(qce, rctx->flags, rctx->authklen,
					digestsize);
	}

	if (IS_SHA_HMAC(rctx->flags) || IS_CMAC(rctx->flags)) {
		u32 authkey_words = rctx->authklen / sizeof(u32);

		qce_cpu_to_be32p_array(mackey, rctx->authkey, rctx->authklen);
		qce_write_array(qce, qce_reg_auth_key0(qce), (u32 *)mackey,
				authkey_words);
	}

	if (IS_CMAC(rctx->flags))
		goto go_proc;

	if (rctx->first_blk)
		memcpy(auth, rctx->digest, digestsize);
	else
		qce_cpu_to_be32p_array(auth, rctx->digest, digestsize);

	iv_words = (IS_SHA1(rctx->flags) || IS_SHA1_HMAC(rctx->flags)) ? 5 : 8;
	qce_write_array(qce, qce_reg_auth_iv0(qce), (u32 *)auth, iv_words);

	if (rctx->first_blk)
		qce_clear_array(qce, qce_reg_auth_bytecnt0(qce), 4);
	else
		qce_write_array(qce, qce_reg_auth_bytecnt0(qce),
				(u32 *)rctx->byte_count, 2);

	auth_cfg = qce_auth_cfg(qce, rctx->flags, 0, digestsize);

	if (qce_is_ce2(qce)) {
		if (rctx->last_blk)
			auth_cfg |= BIT(CE2_LAST_SHIFT);
		if (rctx->first_blk)
			auth_cfg |= BIT(CE2_FIRST_SHIFT);
		/* CLR_CNTXT on the first block prevents state leakage from any
		 * prior operation (the engine retains AUTH_IV until cleared).
		 */
		if (rctx->first_blk)
			auth_cfg |= BIT(CE2_CLR_CNTXT_SHIFT);
	} else {
		if (rctx->last_blk)
			auth_cfg |= BIT(AUTH_LAST_SHIFT);
		else
			auth_cfg &= ~BIT(AUTH_LAST_SHIFT);

		if (rctx->first_blk)
			auth_cfg |= BIT(AUTH_FIRST_SHIFT);
		else
			auth_cfg &= ~BIT(AUTH_FIRST_SHIFT);
	}

go_proc:
	if (qce_is_ce2(qce)) {
		/*
		 * CE2 uses different register layout for hash operations.
		 * SEG_CFG at 0x030 holds the combined config (set above).
		 * AUTH_SEG_CFG at 0x038 holds auth size/start.
		 * SEG_SIZE at 0x03C holds the total segment size.
		 * No separate ENCR_SEG_CFG write needed for hash-only.
		 */
		qce_write(qce, CE2_REG_SEG_CFG, auth_cfg);
		qce_write(qce, CE2_REG_AUTH_SEG_CFG,
			  req->nbytes << CE2_AUTH_SEG_SIZE_SHIFT);
		qce_write(qce, CE2_REG_SEG_SIZE, req->nbytes);
	} else {
		qce_write(qce, REG_AUTH_SEG_CFG, auth_cfg);
		qce_write(qce, REG_AUTH_SEG_SIZE, req->nbytes);
		qce_write(qce, REG_AUTH_SEG_START, 0);
		qce_write(qce, REG_ENCR_SEG_CFG, 0);
		qce_write(qce, REG_SEG_SIZE, req->nbytes);
	}

	/* get little endianness */
	config = qce_config_reg(qce, 1);
	qce_write(qce, qce_reg_config(qce), config);

	qce_crypto_go(qce, true);

	return 0;
}
#endif

#if defined(CONFIG_CRYPTO_DEV_QCE_SKCIPHER) || defined(CONFIG_CRYPTO_DEV_QCE_AEAD)
/*
 * Build encryption configuration for CE2
 * CE2 uses a combined SEG_CFG register at offset 0x030
 */
static u32 qce_encr_cfg_ce2(unsigned long flags, u32 aes_key_size)
{
	u32 cfg = 0;

	if (IS_AES(flags)) {
		cfg |= CE2_ENCR_ALG_AES << CE2_ENCR_ALG_SHIFT;
		if (aes_key_size == AES_KEYSIZE_128)
			cfg |= CE2_ENCR_KEY_SZ_AES128 << CE2_ENCR_KEY_SZ_SHIFT;
		else if (aes_key_size == AES_KEYSIZE_192)
			cfg |= CE2_ENCR_KEY_SZ_AES192 << CE2_ENCR_KEY_SZ_SHIFT;
		else if (aes_key_size == AES_KEYSIZE_256)
			cfg |= CE2_ENCR_KEY_SZ_AES256 << CE2_ENCR_KEY_SZ_SHIFT;
	} else if (IS_DES(flags)) {
		cfg |= CE2_ENCR_ALG_DES << CE2_ENCR_ALG_SHIFT;
		cfg |= CE2_ENCR_KEY_SZ_DES << CE2_ENCR_KEY_SZ_SHIFT;
	} else if (IS_3DES(flags)) {
		cfg |= CE2_ENCR_ALG_DES << CE2_ENCR_ALG_SHIFT;
		cfg |= CE2_ENCR_KEY_SZ_3DES << CE2_ENCR_KEY_SZ_SHIFT;
	}

	switch (flags & QCE_MODE_MASK) {
	case QCE_MODE_ECB:
		cfg |= CE2_ENCR_MODE_ECB << CE2_ENCR_MODE_SHIFT;
		break;
	case QCE_MODE_CBC:
		cfg |= CE2_ENCR_MODE_CBC << CE2_ENCR_MODE_SHIFT;
		break;
	case QCE_MODE_CTR:
		cfg |= CE2_ENCR_MODE_CTR << CE2_ENCR_MODE_SHIFT;
		cfg |= CE2_CNTR_ALG_NIST << CE2_CNTR_ALG_SHIFT;
		break;
	case QCE_MODE_XTS:
		/* CE2 doesn't support XTS mode */
		return ~0;
	case QCE_MODE_CCM:
		/* CE2 doesn't support CCM mode */
		return ~0;
	default:
		return ~0;
	}

	return cfg;
}

/* Build encryption configuration for v5 */
static u32 qce_encr_cfg_v5(unsigned long flags, u32 aes_key_size)
{
	u32 cfg = 0;

	if (IS_AES(flags)) {
		if (aes_key_size == AES_KEYSIZE_128)
			cfg |= ENCR_KEY_SZ_AES128 << ENCR_KEY_SZ_SHIFT;
		else if (aes_key_size == AES_KEYSIZE_256)
			cfg |= ENCR_KEY_SZ_AES256 << ENCR_KEY_SZ_SHIFT;
	}

	if (IS_AES(flags))
		cfg |= ENCR_ALG_AES << ENCR_ALG_SHIFT;
	else if (IS_DES(flags) || IS_3DES(flags))
		cfg |= ENCR_ALG_DES << ENCR_ALG_SHIFT;

	if (IS_DES(flags))
		cfg |= ENCR_KEY_SZ_DES << ENCR_KEY_SZ_SHIFT;

	if (IS_3DES(flags))
		cfg |= ENCR_KEY_SZ_3DES << ENCR_KEY_SZ_SHIFT;

	switch (flags & QCE_MODE_MASK) {
	case QCE_MODE_ECB:
		cfg |= ENCR_MODE_ECB << ENCR_MODE_SHIFT;
		break;
	case QCE_MODE_CBC:
		cfg |= ENCR_MODE_CBC << ENCR_MODE_SHIFT;
		break;
	case QCE_MODE_CTR:
		cfg |= ENCR_MODE_CTR << ENCR_MODE_SHIFT;
		break;
	case QCE_MODE_XTS:
		cfg |= ENCR_MODE_XTS << ENCR_MODE_SHIFT;
		break;
	case QCE_MODE_CCM:
		cfg |= ENCR_MODE_CCM << ENCR_MODE_SHIFT;
		cfg |= LAST_CCM_XFR << LAST_CCM_SHIFT;
		break;
	default:
		return ~0;
	}

	return cfg;
}

static u32 qce_encr_cfg(struct qce_device *qce, unsigned long flags, u32 aes_key_size)
{
	if (qce_is_ce2(qce))
		return qce_encr_cfg_ce2(flags, aes_key_size);
	else
		return qce_encr_cfg_v5(flags, aes_key_size);
}
#endif

#ifdef CONFIG_CRYPTO_DEV_QCE_SKCIPHER
static void qce_xts_swapiv(__be32 *dst, const u8 *src, unsigned int ivsize)
{
	u8 swap[QCE_AES_IV_LENGTH];
	u32 i, j;

	if (ivsize > QCE_AES_IV_LENGTH)
		return;

	memset(swap, 0, QCE_AES_IV_LENGTH);

	for (i = (QCE_AES_IV_LENGTH - ivsize), j = ivsize - 1;
	     i < QCE_AES_IV_LENGTH; i++, j--)
		swap[i] = src[j];

	qce_cpu_to_be32p_array(dst, swap, QCE_AES_IV_LENGTH);
}

static void qce_xtskey(struct qce_device *qce, const u8 *enckey,
		       unsigned int enckeylen, unsigned int cryptlen)
{
	u32 xtskey[QCE_MAX_CIPHER_KEY_SIZE / sizeof(u32)] = {0};
	unsigned int xtsklen = enckeylen / (2 * sizeof(u32));

	qce_cpu_to_be32p_array((__be32 *)xtskey, enckey + enckeylen / 2,
			       enckeylen / 2);
	qce_write_array(qce, REG_ENCR_XTS_KEY0, xtskey, xtsklen);

	/* Set data unit size to cryptlen. Anything else causes
	 * crypto engine to return back incorrect results.
	 */
	qce_write(qce, REG_ENCR_XTS_DU_SIZE, cryptlen);
}

static int qce_setup_regs_skcipher(struct crypto_async_request *async_req)
{
	struct skcipher_request *req = skcipher_request_cast(async_req);
	struct qce_cipher_reqctx *rctx = skcipher_request_ctx(req);
	struct qce_cipher_ctx *ctx = crypto_tfm_ctx(async_req->tfm);
	struct qce_alg_template *tmpl = to_cipher_tmpl(crypto_skcipher_reqtfm(req));
	struct qce_device *qce = tmpl->qce;
	__be32 enckey[QCE_MAX_CIPHER_KEY_SIZE / sizeof(__be32)] = {0};
	__be32 enciv[QCE_MAX_IV_SIZE / sizeof(__be32)] = {0};
	unsigned int enckey_words, enciv_words;
	unsigned int keylen;
	u32 encr_cfg = 0, auth_cfg = 0, config;
	unsigned int ivsize = rctx->ivsize;
	unsigned long flags = rctx->flags;

	qce_setup_config(qce);

	if (IS_XTS(flags))
		keylen = ctx->enc_keylen / 2;
	else
		keylen = ctx->enc_keylen;

	qce_cpu_to_be32p_array(enckey, ctx->enc_key, keylen);
	enckey_words = keylen / sizeof(u32);

	/* Clear auth config for encryption-only operation */
	if (qce_is_ce2(qce))
		qce_write(qce, CE2_REG_AUTH_SEG_CFG, auth_cfg);
	else
		qce_write(qce, REG_AUTH_SEG_CFG, auth_cfg);

	encr_cfg = qce_encr_cfg(qce, flags, keylen);

	if (IS_DES(flags)) {
		enciv_words = 2;
		enckey_words = 2;
	} else if (IS_3DES(flags)) {
		enciv_words = 2;
		enckey_words = 6;
	} else if (IS_AES(flags)) {
		if (IS_XTS(flags)) {
			if (qce_is_ce2(qce))
				return -EINVAL; /* CE2 doesn't support XTS */
			qce_xtskey(qce, ctx->enc_key, ctx->enc_keylen,
				   rctx->cryptlen);
		}
		enciv_words = 4;
	} else {
		return -EINVAL;
	}

	/* Write encryption key to appropriate register */
	if (qce_is_ce2(qce)) {
		if (IS_DES(flags) || IS_3DES(flags))
			qce_write_array(qce, qce_reg_des_key0(qce),
					(u32 *)enckey, enckey_words);
		else
			qce_write_array(qce, qce_reg_encr_key0(qce),
					(u32 *)enckey, enckey_words);
	} else {
		qce_write_array(qce, REG_ENCR_KEY0, (u32 *)enckey, enckey_words);
	}

	if (!IS_ECB(flags)) {
		if (IS_XTS(flags))
			qce_xts_swapiv(enciv, rctx->iv, ivsize);
		else
			qce_cpu_to_be32p_array(enciv, rctx->iv, ivsize);

		qce_write_array(qce, qce_reg_cntr0_iv0(qce), (u32 *)enciv, enciv_words);
	}

	if (IS_ENCRYPT(flags)) {
		if (qce_is_ce2(qce))
			encr_cfg |= BIT(CE2_ENCODE_SHIFT);
		else
			encr_cfg |= BIT(ENCODE_SHIFT);
	}

	if (qce_is_ce2(qce)) {
		/*
		 * CE2 uses combined SEG_CFG register for mode/algorithm
		 * and separate ENCR_SEG_CFG for size/start.
		 * Add FIRST and LAST bits for single-shot operation.
		 */
		encr_cfg |= BIT(CE2_FIRST_SHIFT) | BIT(CE2_LAST_SHIFT);
		qce_write(qce, CE2_REG_SEG_CFG, encr_cfg);
		/* ENCR_SEG_CFG has size in upper 16 bits, start in lower 16 */
		qce_write(qce, CE2_REG_ENCR_SEG_CFG,
			  (rctx->cryptlen << CE2_ENCR_SEG_SIZE_SHIFT));
	} else {
		qce_write(qce, REG_ENCR_SEG_CFG, encr_cfg);
		qce_write(qce, REG_ENCR_SEG_SIZE, rctx->cryptlen);
		qce_write(qce, REG_ENCR_SEG_START, 0);
	}

	if (IS_CTR(flags)) {
		qce_write(qce, qce_reg_cntr_mask(qce), ~0);
		if (!qce_is_ce2(qce)) {
			/* v5 has additional mask registers */
			qce_write(qce, REG_CNTR_MASK0, ~0);
			qce_write(qce, REG_CNTR_MASK1, ~0);
			qce_write(qce, REG_CNTR_MASK2, ~0);
		}
	}

	qce_write(qce, qce_reg_seg_size(qce), rctx->cryptlen);

	/* get little endianness */
	config = qce_config_reg(qce, 1);
	qce_write(qce, qce_reg_config(qce), config);

	qce_crypto_go(qce, true);

	return 0;
}
#endif

#ifdef CONFIG_CRYPTO_DEV_QCE_AEAD
static const u32 std_iv_sha1[SHA256_DIGEST_SIZE / sizeof(u32)] = {
	SHA1_H0, SHA1_H1, SHA1_H2, SHA1_H3, SHA1_H4, 0, 0, 0
};

static const u32 std_iv_sha256[SHA256_DIGEST_SIZE / sizeof(u32)] = {
	SHA256_H0, SHA256_H1, SHA256_H2, SHA256_H3,
	SHA256_H4, SHA256_H5, SHA256_H6, SHA256_H7
};

static unsigned int qce_be32_to_cpu_array(u32 *dst, const u8 *src, unsigned int len)
{
	u32 *d = dst;
	const u8 *s = src;
	unsigned int n;

	n = len / sizeof(u32);
	for (; n > 0; n--) {
		*d = be32_to_cpup((const __be32 *)s);
		s += sizeof(u32);
		d++;
	}
	return DIV_ROUND_UP(len, sizeof(u32));
}

static int qce_setup_regs_aead(struct crypto_async_request *async_req)
{
	struct aead_request *req = aead_request_cast(async_req);
	struct qce_aead_reqctx *rctx = aead_request_ctx_dma(req);
	struct qce_aead_ctx *ctx = crypto_tfm_ctx(async_req->tfm);
	struct qce_alg_template *tmpl = to_aead_tmpl(crypto_aead_reqtfm(req));
	struct qce_device *qce = tmpl->qce;
	u32 enckey[QCE_MAX_CIPHER_KEY_SIZE / sizeof(u32)] = {0};
	u32 enciv[QCE_MAX_IV_SIZE / sizeof(u32)] = {0};
	u32 authkey[QCE_SHA_HMAC_KEY_SIZE / sizeof(u32)] = {0};
	u32 authiv[SHA256_DIGEST_SIZE / sizeof(u32)] = {0};
	u32 authnonce[QCE_MAX_NONCE / sizeof(u32)] = {0};
	unsigned int enc_keylen = ctx->enc_keylen;
	unsigned int auth_keylen = ctx->auth_keylen;
	unsigned int enc_ivsize = rctx->ivsize;
	unsigned int auth_ivsize = 0;
	unsigned int enckey_words, enciv_words;
	unsigned int authkey_words, authiv_words, authnonce_words;
	unsigned long flags = rctx->flags;
	u32 encr_cfg, auth_cfg, config, totallen;
	u32 iv_last_word;

	qce_setup_config(qce);

	/* Write encryption key */
	enckey_words = qce_be32_to_cpu_array(enckey, ctx->enc_key, enc_keylen);
	qce_write_array(qce, REG_ENCR_KEY0, enckey, enckey_words);

	/* Write encryption iv */
	enciv_words = qce_be32_to_cpu_array(enciv, rctx->iv, enc_ivsize);
	qce_write_array(qce, REG_CNTR0_IV0, enciv, enciv_words);

	if (IS_CCM(rctx->flags)) {
		iv_last_word = enciv[enciv_words - 1];
		qce_write(qce, REG_CNTR3_IV3, iv_last_word + 1);
		qce_write_array(qce, REG_ENCR_CCM_INT_CNTR0, (u32 *)enciv, enciv_words);
		qce_write(qce, REG_CNTR_MASK, ~0);
		qce_write(qce, REG_CNTR_MASK0, ~0);
		qce_write(qce, REG_CNTR_MASK1, ~0);
		qce_write(qce, REG_CNTR_MASK2, ~0);
	}

	/* Clear authentication IV and KEY registers of previous values */
	qce_clear_array(qce, REG_AUTH_IV0, 16);
	qce_clear_array(qce, REG_AUTH_KEY0, 16);

	/* Clear byte count */
	qce_clear_array(qce, REG_AUTH_BYTECNT0, 4);

	/* Write authentication key */
	authkey_words = qce_be32_to_cpu_array(authkey, ctx->auth_key, auth_keylen);
	qce_write_array(qce, REG_AUTH_KEY0, (u32 *)authkey, authkey_words);

	/* Write initial authentication IV only for HMAC algorithms */
	if (IS_SHA_HMAC(rctx->flags)) {
		/* Write default authentication iv */
		if (IS_SHA1_HMAC(rctx->flags)) {
			auth_ivsize = SHA1_DIGEST_SIZE;
			memcpy(authiv, std_iv_sha1, auth_ivsize);
		} else if (IS_SHA256_HMAC(rctx->flags)) {
			auth_ivsize = SHA256_DIGEST_SIZE;
			memcpy(authiv, std_iv_sha256, auth_ivsize);
		}
		authiv_words = auth_ivsize / sizeof(u32);
		qce_write_array(qce, REG_AUTH_IV0, (u32 *)authiv, authiv_words);
	} else if (IS_CCM(rctx->flags)) {
		/* Write nonce for CCM algorithms */
		authnonce_words = qce_be32_to_cpu_array(authnonce, rctx->ccm_nonce, QCE_MAX_NONCE);
		qce_write_array(qce, REG_AUTH_INFO_NONCE0, authnonce, authnonce_words);
	}

	/* Set up ENCR_SEG_CFG - CE2 doesn't support AEAD */
	if (qce_is_ce2(qce))
		return -EINVAL;

	encr_cfg = qce_encr_cfg(qce, flags, enc_keylen);
	if (IS_ENCRYPT(flags))
		encr_cfg |= BIT(ENCODE_SHIFT);
	qce_write(qce, REG_ENCR_SEG_CFG, encr_cfg);

	/* Set up AUTH_SEG_CFG (AEAD path -- CE2 returns -EINVAL earlier, so qce is always v5 here) */
	auth_cfg = qce_auth_cfg(qce, rctx->flags, auth_keylen, ctx->authsize);
	auth_cfg |= BIT(AUTH_LAST_SHIFT);
	auth_cfg |= BIT(AUTH_FIRST_SHIFT);
	if (IS_ENCRYPT(flags)) {
		if (IS_CCM(rctx->flags))
			auth_cfg |= AUTH_POS_BEFORE << AUTH_POS_SHIFT;
		else
			auth_cfg |= AUTH_POS_AFTER << AUTH_POS_SHIFT;
	} else {
		if (IS_CCM(rctx->flags))
			auth_cfg |= AUTH_POS_AFTER << AUTH_POS_SHIFT;
		else
			auth_cfg |= AUTH_POS_BEFORE << AUTH_POS_SHIFT;
	}
	qce_write(qce, REG_AUTH_SEG_CFG, auth_cfg);

	totallen = rctx->cryptlen + rctx->assoclen;

	/* Set the encryption size and start offset */
	if (IS_CCM(rctx->flags) && IS_DECRYPT(rctx->flags))
		qce_write(qce, REG_ENCR_SEG_SIZE, rctx->cryptlen + ctx->authsize);
	else
		qce_write(qce, REG_ENCR_SEG_SIZE, rctx->cryptlen);
	qce_write(qce, REG_ENCR_SEG_START, rctx->assoclen & 0xffff);

	/* Set the authentication size and start offset */
	qce_write(qce, REG_AUTH_SEG_SIZE, totallen);
	qce_write(qce, REG_AUTH_SEG_START, 0);

	/* Write total length */
	if (IS_CCM(rctx->flags) && IS_DECRYPT(rctx->flags))
		qce_write(qce, REG_SEG_SIZE, totallen + ctx->authsize);
	else
		qce_write(qce, REG_SEG_SIZE, totallen);

	/* get little endianness */
	config = qce_config_reg(qce, 1);
	qce_write(qce, REG_CONFIG, config);

	/* Start the process */
	qce_crypto_go(qce, !IS_CCM(flags));

	return 0;
}
#endif

int qce_start(struct crypto_async_request *async_req, u32 type)
{
	switch (type) {
#ifdef CONFIG_CRYPTO_DEV_QCE_SKCIPHER
	case CRYPTO_ALG_TYPE_SKCIPHER:
		return qce_setup_regs_skcipher(async_req);
#endif
#ifdef CONFIG_CRYPTO_DEV_QCE_SHA
	case CRYPTO_ALG_TYPE_AHASH:
		return qce_setup_regs_ahash(async_req);
#endif
#ifdef CONFIG_CRYPTO_DEV_QCE_AEAD
	case CRYPTO_ALG_TYPE_AEAD:
		return qce_setup_regs_aead(async_req);
#endif
	default:
		return -EINVAL;
	}
}

/* Status error bits for v5 */
#define STATUS_ERRORS_V5	\
		(BIT(SW_ERR_SHIFT) | BIT(AXI_ERR_SHIFT) | BIT(HSD_ERR_SHIFT))

/* Status error bits for CE2 */
#define STATUS_ERRORS_CE2	\
		(BIT(CE2_SW_ERR_SHIFT) | BIT(CE2_DIN_ERR_SHIFT) | \
		 BIT(CE2_DOUT_ERR_SHIFT) | BIT(CE2_ACCESS_VIOL_SHIFT))

#ifdef CONFIG_CRYPTO_DEV_QCE_SHA
static void qce_ce2_dma_done(void *param)
{
	complete(param);
}

/*
 * Chain input data feed + digest readout on a single ADM channel.
 * Mirrors the webOS qce.c::_setup_cmd_template pattern:
 *
 *   cmd 1 (BOX, MEM_TO_DEV): input data -> DATA_SHADOW0 (+0x8000),
 *                            CRCI 4 (CE_IN) handshake gates writes
 *   cmd 2 (SINGLE, DEV_TO_MEM, CMD_LC): AUTH_IV0..N -> result_buf,
 *                                       CRCI 15 (CE_HASH) handshake
 *
 * Both descriptors execute back-to-back on the SAME channel (rxchan).
 * The qcom_adm driver doesn't natively support per-descriptor CRCI
 * override, so we slave_config between preps -- the prep functions
 * capture src/dst/CRCI into the box_desc / single_desc at prep time,
 * subsequent slave_config calls only affect future preps.
 *
 * Critically: NO dmaengine_terminate_sync() between the two
 * descriptors.  Earlier attempts that used separate rxchan + txchan
 * with a terminate between them gave 4/6 (wedge after 1-2 SHA256
 * ops) because the engine sees the channel teardown as an
 * abnormal end-of-transaction, leaving internal context in a state
 * the next op can't recover from.  Letting ADM walk both descriptors
 * within a single submit-issue-pending transaction matches webOS
 * exactly.
 */
static int qce_ce2_dma_chain_input_digest(struct qce_device *qce,
					  struct scatterlist *src,
					  unsigned int nbytes,
					  void *digest, unsigned int digestsize)
{
	struct dma_chan *chan = qce->dma.rxchan;
	struct qcom_adm_peripheral_config in_periph = {
		.crci = qce->dma.rx_crci,	/* CRCI 4 = CE_IN */
	};
	struct qcom_adm_peripheral_config out_periph = {
		.crci = qce->dma.tx_crci,	/* CRCI 15 = CE_HASH */
	};
	struct dma_slave_config in_conf = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr = qce->phys_base + CE2_REG_DATA_SHADOW0,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_maxburst = 4,	/* 16 byte burst, CE2 FIFO chunk */
		.peripheral_config = &in_periph,
		.peripheral_size = sizeof(in_periph),
	};
	struct dma_slave_config out_conf = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr = qce->phys_base + CE2_REG_AUTH_IV0,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 8,	/* 32 byte burst, fits SHA256 in 1 row */
		.peripheral_config = &out_periph,
		.peripheral_size = sizeof(out_periph),
	};
	struct dma_async_tx_descriptor *in_desc, *out_desc;
	struct completion done;
	dma_cookie_t in_cookie, out_cookie;
	dma_addr_t in_dma, out_dma;
	u8 *in_buf;
	__le32 *in_words;
	__le32 *out_buf;
	enum dma_status status;
	unsigned int in_total, in_dwords, words = digestsize / sizeof(u32);
	unsigned int i;
	u8 *src_copy;
	int ret;

	/* Input buffer: round up to 16-byte FIFO chunk, byte-swap to BE */
	in_total = round_up(nbytes, 16);
	if (in_total == 0)
		in_total = 16;

	src_copy = kzalloc(in_total, GFP_KERNEL);
	if (!src_copy)
		return -ENOMEM;
	if (nbytes)
		sg_copy_to_buffer(src, sg_nents(src), src_copy, nbytes);

	in_buf = dma_alloc_coherent(qce->dev, in_total, &in_dma, GFP_KERNEL);
	if (!in_buf) {
		ret = -ENOMEM;
		goto out_free_src;
	}

	out_buf = dma_alloc_coherent(qce->dev, digestsize, &out_dma, GFP_KERNEL);
	if (!out_buf) {
		ret = -ENOMEM;
		goto out_free_in;
	}

	/* Byte-swap input dwords to BE: CE2 reads each beat MSB-first */
	in_dwords = in_total / 4;
	in_words = (__le32 *)in_buf;
	for (i = 0; i < in_dwords; i++) {
		u32 w = ((u32)src_copy[i * 4 + 0] << 24) |
			((u32)src_copy[i * 4 + 1] << 16) |
			((u32)src_copy[i * 4 + 2] <<  8) |
			((u32)src_copy[i * 4 + 3] <<  0);
		in_words[i] = cpu_to_le32(w);
	}

	/* Prep descriptor 1: input MEM_TO_DEV with CRCI 4 */
	ret = dmaengine_slave_config(chan, &in_conf);
	if (ret) {
		dev_err(qce->dev, "CE2 hash: input slave_config failed: %d\n", ret);
		goto out_free_out_only;
	}

	in_desc = dmaengine_prep_slave_single(chan, in_dma, in_total,
					      DMA_MEM_TO_DEV, DMA_CTRL_ACK);
	if (!in_desc) {
		dev_err(qce->dev, "CE2 hash: input prep failed\n");
		ret = -ENOMEM;
		goto out_terminate;
	}

	/* Prep descriptor 2: digest DEV_TO_MEM with CRCI 15.  Reconfigure
	 * slave config -- the in_desc above has already captured its CRCI=4
	 * and dst_addr into the box descriptor, so changing the channel's
	 * slave config here only affects this next prep.
	 */
	ret = dmaengine_slave_config(chan, &out_conf);
	if (ret) {
		dev_err(qce->dev, "CE2 hash: digest slave_config failed: %d\n", ret);
		goto out_terminate;
	}

	out_desc = dmaengine_prep_slave_single(chan, out_dma, digestsize,
					       DMA_DEV_TO_MEM,
					       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!out_desc) {
		dev_err(qce->dev, "CE2 hash: digest prep failed\n");
		ret = -ENOMEM;
		goto out_terminate;
	}

	init_completion(&done);
	out_desc->callback = qce_ce2_dma_done;
	out_desc->callback_param = &done;

	in_cookie = dmaengine_submit(in_desc);
	if (dma_submit_error(in_cookie)) {
		dev_err(qce->dev, "CE2 hash: input submit failed\n");
		ret = -EIO;
		goto out_terminate;
	}
	out_cookie = dmaengine_submit(out_desc);
	if (dma_submit_error(out_cookie)) {
		dev_err(qce->dev, "CE2 hash: digest submit failed\n");
		ret = -EIO;
		goto out_terminate;
	}

	dma_async_issue_pending(chan);

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(1000))) {
		dev_err(qce->dev,
			"CE2 hash: chained DMA timeout (input+digest)\n");
		ret = -ETIMEDOUT;
		goto out_terminate;
	}

	status = dma_async_is_tx_complete(chan, out_cookie, NULL, NULL);
	if (status != DMA_COMPLETE) {
		dev_err(qce->dev, "CE2 hash: chained DMA incomplete (%d)\n",
			status);
		ret = -EIO;
		goto out_terminate;
	}

	/* Convert digest words from raw LE u32 (ADM-read) to BE bytes (SHA spec) */
	{
		__be32 *out_be = digest;

		for (i = 0; i < words; i++)
			out_be[i] = cpu_to_be32(le32_to_cpu(out_buf[i]));
	}
	ret = 0;

out_terminate:
	dmaengine_terminate_sync(chan);
out_free_out_only:
	dma_free_coherent(qce->dev, digestsize, out_buf, out_dma);
out_free_in:
	dma_free_coherent(qce->dev, in_total, in_buf, in_dma);
out_free_src:
	kfree(src_copy);
	return ret;
}

/*
 * Read CE2 hash digest via ADM DMA from AUTH_IV0 with CRCI 15
 * (CE_HASH_CRCI) handshake.  The CRCI 15 handshake is what drives
 * the engine's FINAL_READ -> CTXT_CLEARING -> UNLOCKING -> IDLE
 * state-machine transition.  A PIO readl() returns correct bytes
 * but does NOT fire the handshake -- after a couple of SHA256 ops
 * the engine wedges in PROCESSING.
 *
 * Key configuration to make this work:
 *
 *   src_maxburst = 8 dwords = 32 bytes.  This is enough to fit the
 *   SHA256 digest (32 B) in a single BOX row, OR to fall through to
 *   the SINGLE-descriptor path for SHA1 (20 B).  In both cases the
 *   ADM peripheral source address auto-increments WITHIN the
 *   descriptor (qcom_adm BOX hardware walks src linearly within a
 *   row; SINGLE walks both src and dst linearly).  An earlier
 *   attempt with src_maxburst=4 (16 B burst) emitted TWO BOX rows
 *   for SHA256, and the second row re-read AUTH_IV0..3 because
 *   row_offset only advances destination across rows.
 *
 *   We dmaengine_terminate_sync() on every exit path (including
 *   success) to flush the channel state; without this the second
 *   transfer on the same channel times out.
 *
 *   ADM delivers AUTH_IV bytes to memory as raw little-endian u32s.
 *   The SHA spec digest is big-endian, so we cpu_to_be32() each
 *   word back into the result buffer (same swap PIO did).
 */
static int qce_ce2_dma_read_digest(struct qce_device *qce, void *digest,
				   unsigned int digestsize)
{
	struct dma_chan *chan = qce->dma.txchan;
	struct qcom_adm_peripheral_config periph = { .crci = qce->dma.tx_crci };
	struct dma_slave_config conf = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr = qce->phys_base + CE2_REG_AUTH_IV0,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		/* 8 beats * 4 bytes = 32 byte burst -- exactly SHA256 size */
		.src_maxburst = 8,
		.peripheral_config = &periph,
		.peripheral_size = sizeof(periph),
	};
	struct dma_async_tx_descriptor *desc;
	struct completion done;
	dma_cookie_t cookie;
	dma_addr_t dma_addr;
	__le32 *coherent_buf;
	enum dma_status status;
	unsigned int words = digestsize / sizeof(u32);
	unsigned int i;
	int ret;

	coherent_buf = dma_alloc_coherent(qce->dev, digestsize, &dma_addr,
					  GFP_KERNEL);
	if (!coherent_buf)
		return -ENOMEM;

	ret = dmaengine_slave_config(chan, &conf);
	if (ret) {
		dev_err(qce->dev, "CE2 hash: slave_config failed: %d\n", ret);
		goto out_free;
	}

	desc = dmaengine_prep_slave_single(chan, dma_addr, digestsize,
					   DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(qce->dev, "CE2 hash: DMA prep failed\n");
		ret = -ENOMEM;
		goto out_terminate;
	}

	init_completion(&done);
	desc->callback = qce_ce2_dma_done;
	desc->callback_param = &done;

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		dev_err(qce->dev, "CE2 hash: DMA submit failed\n");
		ret = -EIO;
		goto out_terminate;
	}
	dma_async_issue_pending(chan);

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(500))) {
		dev_err(qce->dev, "CE2 hash: DMA digest read timeout\n");
		ret = -ETIMEDOUT;
		goto out_terminate;
	}

	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
	if (status != DMA_COMPLETE) {
		dev_err(qce->dev, "CE2 hash: DMA digest read incomplete (%d)\n",
			status);
		ret = -EIO;
		goto out_terminate;
	}

	/*
	 * coherent_buf holds raw 32-bit words as the ADM read them from
	 * AUTH_IV registers.  Convert each word from CPU u32 to BE bytes,
	 * which matches the SHA spec's digest byte order.
	 */
	{
		__be32 *out = digest;

		for (i = 0; i < words; i++)
			out[i] = cpu_to_be32(le32_to_cpu(coherent_buf[i]));
	}
	ret = 0;

out_terminate:
	/*
	 * Always terminate the channel after the transfer to flush ADM
	 * state.  Without this, a second DMA on the same channel times
	 * out -- the first transfer leaves the channel in some half-
	 * processed state that blocks subsequent prep_slave_single().
	 */
	dmaengine_terminate_sync(chan);
out_free:
	dma_free_coherent(qce->dev, digestsize, coherent_buf, dma_addr);
	return ret;
}

/*
 * CE2 hash PIO data path.
 *
 * Mirrors the standalone CE2 PIO diagnostic that returns correct
 * SHA1("test") = a94a8fe5...: write registers in this exact order, fire
 * GOPROC, then feed data in 16-byte chunks while polling DIN_RDY.
 *
 * Doing the register programming inside this function (instead of via
 * qce_setup_regs_ahash) was required because the qce_setup_regs_ahash
 * path writes STATUS=0 + CONFIG twice + AUTH_IV before SEG_CFG, which
 * appears to leave CE2 in a state where AUTH_DONE fires with garbage
 * AUTH_IV output. The diagnostic PIO test inside core.c does it in the
 * order seen in HTC's sbl3 bootloader: SEG_CFG -> sizes -> IVs ->
 * BYTECNT -> CONFIG -> GOPROC -> data, and that works.
 */
int qce_ce2_pio_run_hash(struct crypto_async_request *async_req)
{
	struct ahash_request *req = ahash_request_cast(async_req);
	struct crypto_ahash *ahash = __crypto_ahash_cast(async_req->tfm);
	struct qce_sha_reqctx *rctx = ahash_request_ctx_dma(req);
	struct qce_alg_template *tmpl = to_ahash_tmpl(async_req->tfm);
	struct qce_device *qce = tmpl->qce;
	unsigned int digestsize = crypto_ahash_digestsize(ahash);
	unsigned int blocksize = crypto_tfm_alg_blocksize(async_req->tfm);
	__be32 auth[SHA256_DIGEST_SIZE / sizeof(__be32)] = {0};
	unsigned int iv_words;
	u32 auth_cfg, config, status;
	int timeout, ret = 0;

	/* If not the last block, size must be on the block boundary
	 * (matches qce_setup_regs_ahash precondition).
	 */
	if (!rctx->last_blk && req->nbytes % blocksize)
		return -EINVAL;

	/*
	 * Per-op engine hardware reset.  Empirically the CE2 engine on
	 * APQ8060 has a hard limit of ~5 hash ops per power-on before it
	 * silently stops processing (returns AUTH_IV unchanged).  Clock
	 * gating alone was tried and did NOT lift the limit -- engine
	 * register state is preserved across a clock gate; only an actual
	 * hardware reset clears the internal counter / state.
	 *
	 * GCC_CE2_RESET (DT: reset-names = "clk") asserts the engine's
	 * reset line.  10 us is enough for the engine to fully reset (the
	 * probe-time SW_RST sequence in core.c uses similar timing).
	 * After deassert the engine is back to power-on state -- our
	 * per-op SEG_CFG/AUTH_IV/AUTH_BYTECNT/CONFIG/GOPROC writes below
	 * re-program it from scratch.
	 */
	if (qce->reset) {
		reset_control_assert(qce->reset);
		udelay(10);
		reset_control_deassert(qce->reset);
		udelay(10);
	}

	/*
	 * Wait for CE2 to be fully IDLE before configuring. After AUTH_DONE
	 * on a previous op, CE2 transitions through:
	 *   PROCESSING -> FINAL_READ -> CTXT_CLEARING -> UNLOCKING -> IDLE
	 *
	 * On webOS / mako, this transition is driven by an ADM DMA read of
	 * AUTH_IV0 with CRCI 15 (CE_HASH_CRCI) handshake -- the hardware
	 * handshake is what tells the engine "host consumed the digest, you
	 * can clear context now".  Our PIO readl() of AUTH_IV does NOT fire
	 * CRCI 15, so the engine takes considerably longer to reach IDLE
	 * (or, after multiple ops, fails to fully clear internal state at
	 * all -- this is the SHA256 back-to-back wedge).
	 *
	 * Wait up to 100 ms (vs the previous 10 ms) to give the engine the
	 * full state-machine settling time.
	 */
	for (timeout = 10000; timeout > 0; timeout--) {
		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if ((status & CE2_CRYPTO_STATE_MASK) == 0)
			break;
		udelay(10);
	}

	/* Exact mirror of the working qce_test_pio_mode() diagnostic in
	 * core.c. Uses writel_relaxed throughout (no memory barriers
	 * between writes). The order is:
	 *   SEG_CFG -> AUTH_SEG_CFG -> SEG_SIZE -> AUTH_IV -> AUTH_BYTECNT
	 *   -> CONFIG (read-modify-write) -> GOPROC
	 */
	auth_cfg = qce_auth_cfg_ce2(rctx->flags);
	if (rctx->first_blk)
		auth_cfg |= BIT(CE2_FIRST_SHIFT) | BIT(CE2_CLR_CNTXT_SHIFT);
	if (rctx->last_blk)
		auth_cfg |= BIT(CE2_LAST_SHIFT);

	if (!IS_SHA1(rctx->flags) && !IS_SHA1_HMAC(rctx->flags))
		iv_words = 8;
	else
		iv_words = 5;

	if (rctx->first_blk)
		memcpy(auth, rctx->digest, digestsize);
	else
		qce_cpu_to_be32p_array(auth, rctx->digest, digestsize);

	/*
	 * Use writel() (with implicit dmb/dsb barriers) instead of
	 * writel_relaxed() for every register write below.  Samsung's MSM8960
	 * QCE3 TrustZone (reverse-engineered from tz.mbn) issues a `dsb sy`
	 * after every CE register access without exception -- they don't trust
	 * the engine to see writes in program order without explicit barriers.
	 * On QCE2 silicon with two ADM channels sharing the engine, in-flight
	 * writes can reorder relative to the GOPROC trigger, contributing to
	 * race-window state corruption that manifests as the SHA256 back-to-
	 * back wedge.  writel()'s wmb() before each I/O matches the Samsung
	 * pattern.
	 */

	/* SEG_CFG / AUTH_SEG_CFG / SEG_SIZE first */
	writel(auth_cfg, qce->base + CE2_REG_SEG_CFG);
	writel(req->nbytes << CE2_AUTH_SEG_SIZE_SHIFT,
	       qce->base + CE2_REG_AUTH_SEG_CFG);
	writel(req->nbytes, qce->base + CE2_REG_SEG_SIZE);

	/*
	 * AUTH_IV0..7: zero-prime the full SHA256 IV register block, then
	 * write the algorithm IV.  Samsung MSM8960 TZ explicitly zeros
	 * AUTH_IV[0/1] before every op rather than trusting hardware
	 * auto-init.  Doing this for all 8 SHA256 IV slots ensures any
	 * residual digest words from a previous op (which the engine left
	 * in AUTH_IV0..7 after AUTH_DONE) are cleared before the new op
	 * primes its initial vector.
	 *
	 * For SHA1 we only need 5 IV words -- the upper 3 (IV5..7) get
	 * zeroed and stay zero, no harm.  For SHA256 the same zero-prime
	 * gets immediately overwritten with the algorithm's initial vector
	 * in the next loop.
	 */
	{
		unsigned int j;
		u32 *iv32 = (u32 *)auth;

		for (j = 0; j < 8; j++)
			writel(0, qce->base +
				CE2_REG_AUTH_IV0 + j * sizeof(u32));

		for (j = 0; j < iv_words; j++)
			writel(iv32[j], qce->base +
				CE2_REG_AUTH_IV0 + j * sizeof(u32));
	}

	/*
	 * AUTH_BYTECNT0/1 only.  webOS / mako only program 64 bits of
	 * counter (BYTECNT0/1).  BYTECNT2/3 are not used by SHA1/SHA256.
	 */
	if (rctx->first_blk) {
		writel(0, qce->base + CE2_REG_AUTH_BYTECNT0);
		writel(0, qce->base + CE2_REG_AUTH_BYTECNT1);
	} else {
		writel((__force u32)rctx->byte_count[0],
		       qce->base + CE2_REG_AUTH_BYTECNT0);
		writel((__force u32)rctx->byte_count[1],
		       qce->base + CE2_REG_AUTH_BYTECNT1);
	}

	/* CONFIG: read-modify-write matching PIO diagnostic (set interrupt
	 * masks, enable high-speed paths, ensure CLK_EN_N=0 and SW_RST=0)
	 */
	config = readl(qce->base + CE2_REG_CONFIG);
	config |= BIT(CE2_MASK_DOUT_INTR_SHIFT) |
		  BIT(CE2_MASK_DIN_INTR_SHIFT) |
		  BIT(CE2_MASK_AUTH_DONE_INTR_SHIFT) |
		  BIT(CE2_MASK_ERR_INTR_SHIFT);
	config &= ~(BIT(CE2_HIGH_SPD_IN_EN_N_SHIFT) |
		    BIT(CE2_HIGH_SPD_OUT_EN_N_SHIFT) |
		    BIT(CE2_HIGH_SPD_HASH_EN_N_SHIFT));
	config &= ~(BIT(CE2_CLK_EN_N_SHIFT) | BIT(CE2_SW_RST_SHIFT));
	writel(config, qce->base + CE2_REG_CONFIG);

	/* Diagnostic */
	dev_info(qce->dev,
		 "CE2 hash pre-GO: SEG_CFG=0x%08x SEG_SIZE=%u CONFIG=0x%08x first=%d last=%d\n",
		 auth_cfg, req->nbytes, config,
		 rctx->first_blk, rctx->last_blk);

	/* GOPROC -- use writel() for the barrier; the engine must observe
	 * all setup writes above before it starts processing.
	 */
	writel(BIT(CE2_GO_SHIFT), qce->base + CE2_REG_GOPROC);

	/* 8) Wait for engine to leave IDLE (transition to LOCKED) */
	for (timeout = 1000; timeout > 0; timeout--) {
		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if (status & CE2_CRYPTO_STATE_MASK)
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		dev_err(qce->dev,
			"CE2 hash: engine never left IDLE; STATUS=0x%08x\n",
			status);
		return -ETIMEDOUT;
	}

	/*
	 * 9) Chained DMA: input feed + digest readout on the SAME ADM
	 *    channel as two back-to-back descriptors.  ADM gates the input
	 *    on CRCI 4 (CE_IN) and the digest on CRCI 15 (CE_HASH).  The
	 *    engine's AUTH_DONE -> CRCI 15 firing serves as the synchroniz-
	 *    ation point between the two descriptors; no PIO AUTH_DONE poll
	 *    needed.
	 *
	 *    This matches the webOS _setup_cmd_template chain pattern --
	 *    the structural difference between our previous attempts (which
	 *    hit 4/6 ceiling) and the working webOS path.
	 */
	{
		__be32 result[SHA256_DIGEST_SIZE / sizeof(__be32)];

		ret = qce_ce2_dma_chain_input_digest(qce, req->src, req->nbytes,
						     result, digestsize);
		if (ret)
			goto out;

		memcpy(rctx->digest, result, digestsize);
		if (req->result && rctx->last_blk)
			memcpy(req->result, result, digestsize);
	}

	/* Capture byte count for streaming-hash continuation */
	rctx->byte_count[0] = cpu_to_be32(readl(
		qce->base + CE2_REG_AUTH_BYTECNT0));
	rctx->byte_count[1] = cpu_to_be32(readl(
		qce->base + CE2_REG_AUTH_BYTECNT1));

out:
	return ret;
}
#endif

#ifdef CONFIG_CRYPTO_DEV_QCE_SKCIPHER
static void qce_ce2_skc_dma_done(void *param)
{
	complete(param);
}

/* GOPROC fire callback invoked inside qce_ce2_dma_inout_cipher() AFTER
 * all CPU prep work (memcpy / byte-swap / dmaengine_prep + submit) is
 * done but BEFORE the dma_async_issue_pending() calls that start the
 * ADM bursts.  Closes the GOPROC-to-DIN-streaming window the original
 * layout left open: hundreds of microseconds during which the engine
 * sat in LOCKED waiting for input -- and occasionally aborted the op,
 * producing a no-op chunk (caught separately by the DOUT_SIZE_AVAIL
 * post-DMA check).
 */
static void qce_ce2_skc_fire_goproc(struct qce_device *qce, void *arg)
{
	u32 status;
	int timeout;

	writel(BIT(CE2_GO_SHIFT), qce->base + CE2_REG_GOPROC);

	/* Engine leaves IDLE within microseconds; ADM issue_pending
	 * fires immediately after.
	 */
	for (timeout = 1000; timeout > 0; timeout--) {
		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if (status & CE2_CRYPTO_STATE_MASK)
			break;
		udelay(10);
	}
}

/*
 * Dual-channel ADM DMA for CE2 cipher I/O.
 *
 * rxchan (CRCI 4 = CE_IN):  memory -> DATA_SHADOW0 (peripheral)
 * txchan (CRCI 5 = CE_OUT): DATA_SHADOW0 -> memory
 *
 * Mirrors webOS cmd_list_ce_in / cmd_list_ce_out:
 *   cmd_list_ce_in:  CMD_DST_CRCI(crci_in)  src=memory  dst=DATA_SHADOW0
 *   cmd_list_ce_out: CMD_SRC_CRCI(crci_out) src=DATA_SHADOW0 dst=memory
 *
 * The CE2 engine pipes plaintext bytes through DATA_SHADOW0 in,
 * pumps them through the ENCR block, and emits ciphertext bytes
 * back via DATA_SHADOW0 out -- ADM gates each direction on its
 * own CRCI handshake.
 *
 * Byte-swap notes (same as hash):
 *   CE2 reads each input dword MSB-first as 4 message bytes, and
 *   writes output dwords MSB-first.  ADM moves raw u32 words to/
 *   from memory in CPU endianness, so we swap input bytes -> BE
 *   before DMA, and swap output dwords from raw u32 -> BE bytes
 *   after DMA.  Matches CMD_DST_SWAP_BYTES/SHORTS the webOS ADM
 *   command list set on both directions.
 */
/*
 * Bounce buffer set for chunked AES.  Allocated ONCE in
 * qce_ce2_pio_run_skcipher (sized to the worst-case chunk that the
 * cipher path can pass in), reused across every chunk so we avoid
 * hammering the dma-coherent pool in a tight loop -- per-chunk
 * dma_alloc_coherent / dma_free_coherent churn was a suspect for
 * the cumulative >2 KB failure pattern.
 */
struct qce_ce2_bounce {
	u8 *src_copy;
	u8 *dst_copy;
	__le32 *in_buf;
	__le32 *out_buf;
	dma_addr_t in_dma;
	dma_addr_t out_dma;
	size_t size;
};

/*
 * Configure ADM rx/tx channels for the CE2 cipher data path.  Called
 * ONCE per skcipher request before the chunk loop in
 * qce_ce2_pio_run_skcipher; the per-chunk path then only preps and
 * submits descriptors, skipping the slave-config write.
 *
 * Earlier hash ops may have left rxchan reconfigured for the digest
 * readback (CRCI 15, src=AUTH_IV0); we re-establish the cipher
 * configuration here unconditionally.
 */
static int qce_ce2_dma_setup_cipher_chans(struct qce_device *qce,
					  unsigned int block_dwords)
{
	struct qcom_adm_peripheral_config in_periph = {
		.crci = qce->dma.rx_crci,	/* CRCI 4 = CE_IN */
	};
	struct qcom_adm_peripheral_config out_periph = {
		.crci = 5,			/* CRCI 5 = CE_OUT */
	};
	struct dma_slave_config in_conf = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr = qce->phys_base + CE2_REG_DATA_SHADOW0,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_maxburst = block_dwords,
		.peripheral_config = &in_periph,
		.peripheral_size = sizeof(in_periph),
	};
	struct dma_slave_config out_conf = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr = qce->phys_base + CE2_REG_DATA_SHADOW0,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = block_dwords,
		.peripheral_config = &out_periph,
		.peripheral_size = sizeof(out_periph),
	};
	int ret;

	ret = dmaengine_slave_config(qce->dma.rxchan, &in_conf);
	if (ret) {
		dev_err(qce->dev,
			"CE2 cipher: in slave_config failed: %d\n", ret);
		return ret;
	}
	ret = dmaengine_slave_config(qce->dma.txchan, &out_conf);
	if (ret) {
		dev_err(qce->dev,
			"CE2 cipher: out slave_config failed: %d\n", ret);
		return ret;
	}
	return 0;
}

typedef void (*qce_ce2_pre_issue_fn)(struct qce_device *qce, void *arg);

static int qce_ce2_dma_inout_cipher(struct qce_device *qce,
				    struct scatterlist *src,
				    struct scatterlist *dst,
				    unsigned int sg_offset,
				    unsigned int nbytes,
				    unsigned int block_dwords,
				    struct qce_ce2_bounce *b,
				    qce_ce2_pre_issue_fn pre_issue,
				    void *pre_issue_arg)
{
	struct dma_chan *rx = qce->dma.rxchan;
	struct dma_chan *tx = qce->dma.txchan;
	struct dma_async_tx_descriptor *in_desc, *out_desc;
	struct completion done;
	dma_cookie_t in_cookie, out_cookie;
	dma_addr_t in_dma = b->in_dma, out_dma = b->out_dma;
	__le32 *in_buf = b->in_buf, *out_buf = b->out_buf;
	u8 *src_copy = b->src_copy, *dst_copy = b->dst_copy;
	enum dma_status status;
	unsigned int burst_bytes = block_dwords * 4;
	unsigned int padded = round_up(nbytes, burst_bytes);
	unsigned int dwords;
	unsigned int i;
	int ret;

	if (!padded || padded > b->size)
		return -EINVAL;
	dwords = padded / 4;

	memset(src_copy, 0, padded);
	memset(dst_copy, 0, padded);
	sg_pcopy_to_buffer(src, sg_nents(src), src_copy, nbytes, sg_offset);

	/* Byte-swap input dwords to BE (CE2 reads MSB-first) */
	for (i = 0; i < dwords; i++) {
		u32 w = ((u32)src_copy[i * 4 + 0] << 24) |
			((u32)src_copy[i * 4 + 1] << 16) |
			((u32)src_copy[i * 4 + 2] <<  8) |
			((u32)src_copy[i * 4 + 3] <<  0);
		in_buf[i] = cpu_to_le32(w);
	}

	/* slave_config for both channels is done ONCE by
	 * qce_ce2_dma_setup_cipher_chans() before the chunk loop -- no
	 * per-chunk reconfig here.
	 */
	in_desc = dmaengine_prep_slave_single(rx, in_dma, padded,
					      DMA_MEM_TO_DEV, DMA_CTRL_ACK);
	if (!in_desc) {
		dev_err(qce->dev, "CE2 cipher: in prep failed\n");
		ret = -ENOMEM;
		goto out_terminate_rx;
	}

	out_desc = dmaengine_prep_slave_single(tx, out_dma, padded,
					       DMA_DEV_TO_MEM,
					       DMA_PREP_INTERRUPT |
					       DMA_CTRL_ACK);
	if (!out_desc) {
		dev_err(qce->dev, "CE2 cipher: out prep failed\n");
		ret = -ENOMEM;
		goto out_terminate_rx;
	}

	init_completion(&done);
	out_desc->callback = qce_ce2_skc_dma_done;
	out_desc->callback_param = &done;

	in_cookie = dmaengine_submit(in_desc);
	if (dma_submit_error(in_cookie)) {
		dev_err(qce->dev, "CE2 cipher: in submit failed\n");
		ret = -EIO;
		goto out_terminate;
	}
	out_cookie = dmaengine_submit(out_desc);
	if (dma_submit_error(out_cookie)) {
		dev_err(qce->dev, "CE2 cipher: out submit failed\n");
		ret = -EIO;
		goto out_terminate;
	}

	/* GOPROC fires here, AFTER all the CPU work (memcpy, byte-swap,
	 * prep, submit) is complete but BEFORE the ADM channels are
	 * issued.  This minimises the gap between "engine kicked" and
	 * "ADM streaming DIN data": the engine sees DIN_RDY data within
	 * a few microseconds of leaving IDLE, instead of the hundreds of
	 * microseconds the old layout produced when GOPROC fired BEFORE
	 * the prep+submit work.  Stochastic "no-op chunk" failures
	 * correlate with that gap -- closing it should make them
	 * disappear.
	 */
	if (pre_issue)
		pre_issue(qce, pre_issue_arg);

	dma_async_issue_pending(rx);
	dma_async_issue_pending(tx);

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(1000))) {
		dev_err(qce->dev, "CE2 cipher: DMA timeout (%u bytes)\n",
			padded);
		ret = -ETIMEDOUT;
		goto out_terminate;
	}

	status = dma_async_is_tx_complete(tx, out_cookie, NULL, NULL);
	if (status != DMA_COMPLETE) {
		dev_err(qce->dev, "CE2 cipher: DMA incomplete (%d)\n", status);
		ret = -EIO;
		goto out_terminate;
	}

	/* Byte-swap output dwords from raw LE u32 -> BE bytes */
	for (i = 0; i < dwords; i++) {
		u32 w = le32_to_cpu(out_buf[i]);

		dst_copy[i * 4 + 0] = (w >> 24) & 0xff;
		dst_copy[i * 4 + 1] = (w >> 16) & 0xff;
		dst_copy[i * 4 + 2] = (w >>  8) & 0xff;
		dst_copy[i * 4 + 3] = (w >>  0) & 0xff;
	}
	sg_pcopy_from_buffer(dst, sg_nents(dst), dst_copy, nbytes, sg_offset);

	ret = 0;

out_terminate:
	dmaengine_terminate_sync(tx);
out_terminate_rx:
	dmaengine_terminate_sync(rx);
	return ret;
}

/* Allocate the bounce-buffer set for AES chunked path.  Sized once
 * at the start of qce_ce2_pio_run_skcipher and reused across all
 * chunks; freed at the end.
 */
static int qce_ce2_alloc_bounce(struct qce_device *qce,
				struct qce_ce2_bounce *b, size_t size)
{
	memset(b, 0, sizeof(*b));
	b->size = size;
	b->src_copy = kzalloc(size, GFP_KERNEL);
	if (!b->src_copy)
		return -ENOMEM;
	b->dst_copy = kzalloc(size, GFP_KERNEL);
	if (!b->dst_copy)
		goto free_src;
	b->in_buf = dma_alloc_coherent(qce->dev, size, &b->in_dma,
				       GFP_KERNEL);
	if (!b->in_buf)
		goto free_dst;
	b->out_buf = dma_alloc_coherent(qce->dev, size, &b->out_dma,
					GFP_KERNEL);
	if (!b->out_buf)
		goto free_in;
	return 0;

free_in:
	dma_free_coherent(qce->dev, size, b->in_buf, b->in_dma);
free_dst:
	kfree(b->dst_copy);
free_src:
	kfree(b->src_copy);
	memset(b, 0, sizeof(*b));
	return -ENOMEM;
}

static void qce_ce2_free_bounce(struct qce_device *qce,
				struct qce_ce2_bounce *b)
{
	if (!b->size)
		return;
	dma_free_coherent(qce->dev, b->size, b->out_buf, b->out_dma);
	dma_free_coherent(qce->dev, b->size, b->in_buf, b->in_dma);
	kfree(b->dst_copy);
	kfree(b->src_copy);
	memset(b, 0, sizeof(*b));
}

/*
 * CE2 cipher PIO+DMA path.  Mirror of qce_ce2_pio_run_hash():
 *
 *   1. Per-op GCC_CE2_RESET (otherwise the engine wedges after ~5 ops)
 *   2. Wait for IDLE
 *   3. Write keys, IV, CNTR_MASK, SEG_CFG, ENCR_SEG_CFG, SEG_SIZE,
 *      CONFIG in the order proven by webOS _ce_setup
 *   4. GOPROC, wait engine to leave IDLE
 *   5. Dual-channel ADM DMA for input + output via DATA_SHADOW0
 *   6. Read back CNTR0..3 for CBC/CTR IV chaining
 *
 * Bypasses qce_setup_regs_skcipher() + qce_dma_prep_sgs() because the
 * v5 path has the wrong register-write order for CE2 (STATUS=0 +
 * CONFIG before SEG_CFG yields garbage on CE2) and the qce_dma_*
 * helpers configure both channels for the hash digest readback at
 * probe time.
 */
int qce_ce2_pio_run_skcipher(struct crypto_async_request *async_req)
{
	struct skcipher_request *req = skcipher_request_cast(async_req);
	struct qce_cipher_reqctx *rctx = skcipher_request_ctx(req);
	struct qce_cipher_ctx *ctx = crypto_tfm_ctx(async_req->tfm);
	struct qce_alg_template *tmpl =
		to_cipher_tmpl(crypto_skcipher_reqtfm(req));
	struct qce_device *qce = tmpl->qce;
	unsigned long flags = rctx->flags;
	unsigned int keylen = ctx->enc_keylen;
	__be32 enckey[QCE_MAX_CIPHER_KEY_SIZE / sizeof(__be32)] = {0};
	__be32 enciv[QCE_MAX_IV_SIZE / sizeof(__be32)] = {0};
	unsigned int enckey_words = keylen / sizeof(u32);
	unsigned int enciv_words = 0;
	u32 encr_cfg, config, status;
	int timeout, ret;
	unsigned int k;

	if (rctx->cryptlen == 0)
		return 0;

	/* Per-op engine reset: the CE2 power-on op counter is shared
	 * between AUTH and ENCR blocks -- skipping the cipher reset
	 * causes the same ~5-op wedge that hash hits.  We need the
	 * reset.
	 *
	 * The first-op-after-reset AES-256-CBC encrypt block-2 passthrough
	 * symptom is timing-related: the AES core needs more settling
	 * time after deassert before the round-key expansion is stable
	 * for the longer (14-round) AES-256 schedule.  100 us after
	 * deassert (vs 10 us in hash path) gives the core enough time
	 * for AES-256; AES-128/DES/3DES are faster and tolerate either.
	 */
	if (qce->reset) {
		reset_control_assert(qce->reset);
		usleep_range(1000, 1500);
		reset_control_deassert(qce->reset);
		usleep_range(1000, 1500);
	}

	/* Additional SW_RST pulse via CONFIG register.  GCC_CE2_RESET
	 * resets the clock domain; SW_RST resets the engine's internal
	 * state machine.  Probe does both at init time; the per-op path
	 * needs SW_RST too for 3DES-CBC decrypt cold-start, which
	 * otherwise returns stale DATA_SHADOW0 bytes (engine not
	 * processing at all).  Mirrors the probe-time SW_RST pulse
	 * timing (10 us assert, 10 us deassert).
	 */
	writel_relaxed(BIT(CE2_SW_RST_SHIFT), qce->base + CE2_REG_CONFIG);
	usleep_range(10, 20);
	writel_relaxed(0, qce->base + CE2_REG_CONFIG);
	usleep_range(10, 20);

	/* Wait for IDLE before configuring */
	for (timeout = 10000; timeout > 0; timeout--) {
		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if ((status & CE2_CRYPTO_STATE_MASK) == 0)
			break;
		udelay(10);
	}

	encr_cfg = qce_encr_cfg_ce2(flags, keylen);
	if (encr_cfg == ~0U) {
		dev_err(qce->dev,
			"CE2 cipher: unsupported alg/mode flags=0x%lx keylen=%u\n",
			flags, keylen);
		return -EINVAL;
	}
	/* CTR mode is symmetric (P xor keystream = C; C xor keystream = P).
	 * On CE2, ENCODE=0 in CTR mode appears to flip the engine to
	 * AES_decrypt(input) xor counter instead of input xor AES_encrypt(
	 * counter) -- empirically CTR encrypt works but CTR decrypt returns
	 * garbage.  Force ENCODE=1 + AUTH_POS=1 for CTR regardless of the
	 * crypto-API direction so the engine always runs the symmetric
	 * encrypt-counter path.
	 */
	if (IS_ENCRYPT(flags) || IS_CTR(flags)) {
		encr_cfg |= BIT(CE2_ENCODE_SHIFT);
		/* webOS _ce_setup sets AUTH_POS (bit 14) for encrypt
		 * direction only.  On a pure-cipher op the bit nominally
		 * controls auth ordering, but CE2 also uses it as part of
		 * the cipher direction state machine; without it, decrypt
		 * after encrypt leaves the engine confused.
		 */
		encr_cfg |= BIT(CE2_AUTH_POS_SHIFT);
	}
	/* FIRST | LAST get set per-chunk inside the loop below.  Keep the
	 * base encr_cfg without them so the chunk loop can OR them in.
	 *
	 * The webOS SW_RST + GCC_CE2_RESET above handles context clearing
	 * at op start; CLR_CNTXT inside SEG_CFG is not needed and caps AES
	 * at 4 blocks per op if set.
	 */

	/* No auth segment for skcipher-only */
	writel(0, qce->base + CE2_REG_AUTH_SEG_CFG);

	/* Key writes: DES_KEY0..5 for DES/3DES, AES_RNDKEY0..59 for AES.
	 *
	 * History: an earlier raw-key fast-path (webOS pce_dev->fastaes=1
	 * style) wrote just the 4-8 user key words to RNDKEY0..N and let
	 * the engine expand internally.  That path passed all NIST vectors
	 * <= 32 B but capped AES at 4 blocks/op on real workloads -- the
	 * engine's internal fast-expander appears to wedge after the first
	 * 4 blocks on this silicon.
	 *
	 * Samsung's MSM8660 qce.ko (decompiled at SGH-I727 system.img.ext4)
	 * always writes a fully-expanded FIPS-197 round-key schedule
	 * regardless of the AES_SEL_FAST bit.  Match that pattern.
	 *
	 * Subtle seeding bug to avoid: aes_expandkey() reads input bytes
	 * via get_unaligned_le32(), so bytes [b0,b1,b2,b3] become word
	 * 0xb3b2b1b0.  Samsung (and webOS slow-path) seed from BE-packed
	 * words: bytes [b0,b1,b2,b3] -> word 0xb0b1b2b3.  Different seed
	 * -> different FIPS-197 expansion -> wrong schedule beyond word 0.
	 * Workaround: BE-pack the user key into enckey first, then pass
	 * (u8 *)enckey to aes_expandkey().  On LE ARM the BE word is
	 * stored byte-reversed in memory, so the LE-read inside reconstitutes
	 * the original BE-packed word -- matching Samsung's seed.
	 */
	qce_cpu_to_be32p_array(enckey, ctx->enc_key, keylen);
	if (IS_DES(flags) || IS_3DES(flags)) {
		for (k = 0; k < enckey_words; k++)
			writel((__force u32)enckey[k],
			       qce->base + CE2_REG_DES_KEY0 + k * 4);
	} else {	/* AES */
		struct crypto_aes_ctx aes_ctx = {0};
		int aerr;

		aerr = aes_expandkey(&aes_ctx, (const u8 *)enckey, keylen);
		if (aerr) {
			dev_err(qce->dev,
				"CE2 cipher: aes_expandkey failed (%d)\n",
				aerr);
			return aerr;
		}
		/* Write all 60 expanded dwords.  For AES-128 only the first
		 * 44 are populated by FIPS-197; aes_ctx was zero-initialised
		 * so the remaining 16 stay 0.  Mirrors Samsung's _ce_setup
		 * auStack_160 buffer layout.
		 */
		for (k = 0; k < CE2_AES_RNDKEYS; k++)
			writel(aes_ctx.key_enc[k],
			       qce->base + CE2_REG_AES_RNDKEY0 + k * 4);
		memzero_explicit(&aes_ctx, sizeof(aes_ctx));
	}

	/* IV: CNTR0..3.  Skip for ECB (no IV) */
	if (!IS_ECB(flags) && rctx->iv && rctx->ivsize) {
		qce_cpu_to_be32p_array(enciv, rctx->iv, rctx->ivsize);
		enciv_words = rctx->ivsize / sizeof(u32);
		for (k = 0; k < enciv_words; k++)
			writel((__force u32)enciv[k],
			       qce->base + CE2_REG_CNTR0_IV0 + k * 4);
	}

	if (IS_CTR(flags))
		writel(0xffff, qce->base + CE2_REG_CNTR_MASK);

	/* CONFIG value computed once.  Engine expects CONFIG written
	 * AFTER SEG_CFG/SEG_SIZE (webOS order), so the actual writel
	 * happens inside the loop body before GOPROC.
	 */
	config = readl(qce->base + CE2_REG_CONFIG);
	config |= BIT(CE2_MASK_DOUT_INTR_SHIFT) |
		  BIT(CE2_MASK_DIN_INTR_SHIFT) |
		  BIT(CE2_MASK_AUTH_DONE_INTR_SHIFT) |
		  BIT(CE2_MASK_ERR_INTR_SHIFT);
	config &= ~(BIT(CE2_HIGH_SPD_IN_EN_N_SHIFT) |
		    BIT(CE2_HIGH_SPD_OUT_EN_N_SHIFT) |
		    BIT(CE2_HIGH_SPD_HASH_EN_N_SHIFT));
	config &= ~(BIT(CE2_CLK_EN_N_SHIFT) | BIT(CE2_SW_RST_SHIFT));

	/*
	 * Chunked DMA loop.  CE2 AES engine has a ~64 B internal FIFO; ops
	 * larger than 4 AES blocks (64 B) overflow the FIFO and produce
	 * silent corruption past block 4.  Split AES > 64 B into 64-byte
	 * chunks, each driven as an independent FIRST+LAST single-shot op
	 * with explicit IV chaining via CNTR0..3 between chunks.
	 *
	 * Empirically the engine does NOT preserve CNTR state across
	 * chunks even when SEG_CFG.LAST=0 on the prior chunk -- the
	 * CTXT_CLEARING stage at op end wipes it regardless.  So we read
	 * back CNTR0..3 after each chunk (engine has updated them with the
	 * last ciphertext block for CBC / incremented counter for CTR) and
	 * write them back as the IV for the next chunk before its GOPROC.
	 *
	 * Each chunk uses FIRST=1, LAST=1 (single-shot SEG_CFG); the
	 * software IV-chaining via CNTR readback/restore is what makes the
	 * stream concatenate correctly.
	 *
	 * DES/3DES don't appear to hit the FIFO ceiling -- skip chunking,
	 * one shot up to SEG_SIZE max (65535 B).
	 */
	{
		/* AES chunk size = 48 B (3 blocks).  The silicon FIFO-cap
		 * symptom is a stochastic ~3-5% per-chunk corruption of the
		 * LAST AES block when a chunk hits the 4-block (64 B) ceiling
		 * dead-on.  Empirically observed: with max_chunk=64 we see
		 * scattered failures at "byte 48 of chunk N" -- exactly the
		 * 4th block.  Backing off to 3 blocks per chunk keeps the
		 * engine a safe distance below the cap.  Cost: 33% more
		 * chunks per stream (more CNTR readback + reset overhead),
		 * benefit: eliminates the block-3 failure mode.
		 *
		 * Burst stays at 64 dw (256 B) so the bounce buffer is sized
		 * for round_up(48, 256) = 256 B per chunk -- same allocation
		 * as before.
		 */
		unsigned int max_chunk = IS_AES(flags) ? 48 : 65535;
		unsigned int burst = (IS_DES(flags) || IS_3DES(flags)) ? 8 : 64;
		unsigned int sg_off = 0;
		unsigned int total = rctx->cryptlen;
		u32 chunk_cfg = encr_cfg | BIT(CE2_FIRST_SHIFT) |
				BIT(CE2_LAST_SHIFT);
		struct qce_ce2_bounce bounce;
		unsigned int bounce_sz;

		/* Bounce sized to worst case: AES = burst-rounded chunk (256
		 * B for burst=64dw); DES/3DES = full cryptlen padded up to
		 * the burst.  Allocate ONCE here; reuse for every chunk.
		 */
		bounce_sz = IS_AES(flags) ?
			    round_up(max_chunk, burst * 4) :
			    round_up(total, burst * 4);
		ret = qce_ce2_alloc_bounce(qce, &bounce, bounce_sz);
		if (ret) {
			dev_err(qce->dev,
				"CE2 skc: bounce alloc (%u B) failed: %d\n",
				bounce_sz, ret);
			return ret;
		}

		/* Configure ADM rx/tx slave channels ONCE for this op.
		 * The per-chunk path inside qce_ce2_dma_inout_cipher now
		 * only preps + submits descriptors; slave_config is constant
		 * across chunks (same CRCIs, same burst, same shadow-port
		 * address), so we eliminate ~N redundant slave_config writes
		 * for an N-chunk transfer.
		 */
		ret = qce_ce2_dma_setup_cipher_chans(qce, burst);
		if (ret) {
			qce_ce2_free_bounce(qce, &bounce);
			return ret;
		}

		while (sg_off < total) {
			unsigned int chunk_len =
				(total - sg_off > max_chunk) ?
				max_chunk : (total - sg_off);
			unsigned int chunk_retries = 0;

retry_chunk:
			/*
			 * Between chunks, the CE2 engine's DOUT FIFO retains
			 * stale data from the prior chunk -- on next GOPROC
			 * the engine flushes it as the first 2 blocks of
			 * "output" without processing the new input.  This
			 * was confirmed via a deterministic all-zero
			 * plaintext test: chunk 1 output came back as
			 * literally blocks 3-4 of chunk 0 repeated.
			 *
			 * Re-running the full per-op reset between chunks
			 * (GCC_CE2_RESET + SW_RST pulse + wait-IDLE) clears
			 * the FIFO.  Keys + CONFIG survive the reset since
			 * they live in different register banks; only the
			 * engine state machine resets.  But we must re-write
			 * the keys after reset (the register bank IS wiped
			 * by the reset on this silicon, even though the
			 * CONFIG bits aren't).
			 *
			 * Cost: ~2 ms per reset.  At 4 AES blocks per chunk
			 * the effective HW throughput is ~30 KB/s -- slow,
			 * but cra_priority on the qce CE2 cipher is 50
			 * (below aes-generic's 100), so the kernel uses
			 * software AES by default; HW path is for explicit-
			 * driver-name testing.
			 */
			if (sg_off != 0 || chunk_retries > 0) {
				/* Inter-chunk reset: cycle the engine core
				 * clock (clk_disable / clk_enable) PLUS the
				 * SW_RST pulse.  webOS qce.c and Samsung
				 * qce.ko both fully cycle the clock between
				 * ops -- on this silicon neither
				 * reset_control_assert (GCC_CE2_RESET) nor
				 * SW_RST alone clears enough state to keep
				 * the engine stable across many chunks
				 * (~5% per-chunk failure rate, always fails
				 * past ~64 chunks).  The clock cycle
				 * presumably drains some internal state
				 * machine that reset signals don't touch.
				 *
				 * Note: qce->core was obtained via
				 * devm_clk_get_optional_enabled at probe,
				 * which enables the clock and prevents
				 * auto-disable on remove.  Manual
				 * clk_disable / clk_enable cycling is still
				 * permitted -- the devm guard is about
				 * teardown not runtime usage.
				 */
				if (qce->core) {
					clk_disable(qce->core);
					udelay(10);
					clk_enable(qce->core);
					udelay(10);
				}
				writel_relaxed(BIT(CE2_SW_RST_SHIFT),
					       qce->base + CE2_REG_CONFIG);
				usleep_range(10, 20);
				writel_relaxed(0, qce->base + CE2_REG_CONFIG);
				usleep_range(10, 20);
				for (timeout = 10000; timeout > 0; timeout--) {
					status = readl_relaxed(qce->base +
							       CE2_REG_STATUS);
					if ((status & CE2_CRYPTO_STATE_MASK) == 0)
						break;
					udelay(10);
				}

				/* Re-write AUTH_SEG_CFG (cleared by SW_RST). */
				writel(0, qce->base + CE2_REG_AUTH_SEG_CFG);

				/* Re-write keys (cleared by SW_RST). */
				if (IS_DES(flags) || IS_3DES(flags)) {
					for (k = 0; k < enckey_words; k++)
						writel((__force u32)enckey[k],
						       qce->base +
						       CE2_REG_DES_KEY0 + k * 4);
				} else {
					struct crypto_aes_ctx aes_ctx = {0};

					if (!aes_expandkey(&aes_ctx,
							   (const u8 *)enckey,
							   keylen)) {
						for (k = 0; k < CE2_AES_RNDKEYS; k++)
							writel(aes_ctx.key_enc[k],
							       qce->base +
							       CE2_REG_AES_RNDKEY0 +
							       k * 4);
					}
					memzero_explicit(&aes_ctx,
							 sizeof(aes_ctx));
				}

				if (IS_CTR(flags))
					writel(0xffff, qce->base + CE2_REG_CNTR_MASK);
			}

			/* Re-install IV (CNTR0..3) for non-first chunks --
			 * holds the previous chunk's tail block / next CTR.
			 * IV for first chunk was already written above.
			 * Also re-install on retry (engine state was just
			 * reset, CNTR0..3 wiped).
			 */
			if ((sg_off != 0 || chunk_retries > 0) &&
			    !IS_ECB(flags) && enciv_words) {
				for (k = 0; k < enciv_words; k++)
					writel((__force u32)enciv[k],
					       qce->base + CE2_REG_CNTR0_IV0 +
					       k * 4);
			}

			/* webOS register write order: SEG_CFG ->
			 * ENCR_SEG_CFG -> SEG_SIZE -> CONFIG -> GOPROC.
			 * Writing CONFIG before SEG_CFG empirically breaks
			 * 3DES-CBC decrypt (engine returns input unchanged).
			 */
			writel(chunk_cfg, qce->base + CE2_REG_SEG_CFG);
			writel(chunk_len << CE2_ENCR_SEG_SIZE_SHIFT,
			       qce->base + CE2_REG_ENCR_SEG_CFG);
			writel(chunk_len, qce->base + CE2_REG_SEG_SIZE);
			writel(config, qce->base + CE2_REG_CONFIG);

			dev_dbg(qce->dev, "CE2 skc chunk off=%u len=%u\n",
				sg_off, chunk_len);

			/* GOPROC now fires inside qce_ce2_dma_inout_cipher
			 * via the pre_issue callback -- AFTER memcpy /
			 * byte-swap / prep / submit, but BEFORE
			 * dma_async_issue_pending.  That keeps the engine
			 * "kicked" only microseconds before ADM starts
			 * streaming DIN data, instead of the hundreds of
			 * microseconds the original layout produced when
			 * GOPROC fired in the cipher loop and the CPU then
			 * did the DMA prep work.  Closing that window
			 * should eliminate the ~1.5%-per-chunk "engine
			 * no-op" stochastic failure.
			 */
			ret = qce_ce2_dma_inout_cipher(qce, req->src, req->dst,
						       sg_off, chunk_len,
						       burst, &bounce,
						       qce_ce2_skc_fire_goproc,
						       NULL);
			if (ret) {
				dev_err(qce->dev,
					"CE2 skc DMA failed at off=%u: ret=%d STATUS=0x%08x\n",
					sg_off, ret,
					readl_relaxed(qce->base + CE2_REG_STATUS));
				qce_ce2_free_bounce(qce, &bounce);
				return ret;
			}

			{
				u32 st_post_dma = readl_relaxed(qce->base +
								CE2_REG_STATUS);
				unsigned int dout_avail =
					(st_post_dma & CE2_DOUT_SIZE_AVAIL_MASK) >>
					CE2_DOUT_SIZE_AVAIL_SHIFT;

				/* Engine no-op detection: on healthy chunks
				 * the engine has produced all output and DOUT
				 * FIFO is drained (DOUT_SIZE_AVAIL=0).  On the
				 * ~1.5% stochastic no-op chunks, the engine
				 * never produced anything; DMA in non-FC BOX
				 * mode pulled 12 stale DATA_SHADOW0 dwords and
				 * the engine still has 4 dwords sitting in
				 * its DOUT FIFO (DOUT_SIZE_AVAIL=4).  Use that
				 * as the retry trigger -- the heavy
				 * inter-chunk reset (clk cycle + SW_RST + key
				 * re-write) above clears the wedge state and
				 * a re-run normally succeeds.
				 *
				 * Retry up to a small bound; if we exceed it
				 * the engine is in a deeper wedge and the
				 * outer DMA layer or a higher-level reset
				 * needs to recover.
				 */
				if (dout_avail != 0 &&
				    chunk_retries < 4) {
					chunk_retries++;
					dev_warn_ratelimited(qce->dev,
						"CE2 skc no-op chunk at off=%u (DOUT_AVAIL=%u STATUS=0x%08x), retry %u\n",
						sg_off, dout_avail,
						st_post_dma, chunk_retries);
					goto retry_chunk;
				}
				if (dout_avail != 0) {
					dev_err(qce->dev,
						"CE2 skc chunk wedged after %u retries at off=%u STATUS=0x%08x\n",
						chunk_retries, sg_off,
						st_post_dma);
					qce_ce2_free_bounce(qce, &bounce);
					return -EIO;
				}
			}

			/* Wait for engine to drop back to IDLE -- but ONLY if
			 * the next-chunk IV comes from a CNTR readback (i.e.
			 * NOT CBC; CBC derives the next IV from dst_copy now).
			 *
			 * On this silicon the engine never actually returns
			 * to IDLE after a chunk -- it sits in PROCESSING with
			 * DIN_ERR/DOUT_ERR/SW_ERR set indefinitely.  The
			 * inter-chunk reset (clk cycle + SW_RST) brings it
			 * back to IDLE for the next chunk; we don't need the
			 * engine to self-settle here.
			 *
			 * For CBC the wait was pure overhead -- 1000 polls *
			 * 10 us = ~10 ms per chunk burned waiting for an
			 * event that never fires (~83 % of total time at
			 * 256 KB).  Skip it.
			 *
			 * For CTR (and any future mode that reads CNTR back),
			 * the wait + udelay(5) still matters: STATUS goes
			 * "IDLE" one cycle before the engine's final write to
			 * CNTR commits, so an immediate readl can return
			 * stale state.  Short timeout (50 polls = 500 us) is
			 * enough; the inter-chunk reset covers the long tail.
			 */
			if (!IS_ECB(flags) && !IS_CBC(flags) && enciv_words) {
				for (timeout = 50; timeout > 0; timeout--) {
					status = readl_relaxed(qce->base +
							       CE2_REG_STATUS);
					if ((status & CE2_CRYPTO_STATE_MASK) == 0)
						break;
					udelay(10);
				}
				udelay(5);
			}

			/* Drain any residual DOUT FIFO dwords.  If even one
			 * dword leaks across the chunk boundary, the engine's
			 * internal block pointer slips relative to the data
			 * it sees on the next chunk, producing the random
			 * mid-stream divergence we observe past ~64 chunks.
			 *
			 * DATA_OUT (0x010) is the direct PIO drain port --
			 * distinct from DATA_SHADOW0 (0x8000) which bypasses
			 * the FIFO pointer logic.  Cap the drain count at 8
			 * since DOUT_SIZE_AVAIL is a 3-bit field (max 7).
			 */
			{
				int drain;

				for (drain = 0; drain < 8; drain++) {
					u32 s = readl_relaxed(qce->base +
							      CE2_REG_STATUS);
					u32 avail = (s >> CE2_DOUT_SIZE_AVAIL_SHIFT)
						    & 0x7;
					if (avail == 0)
						break;
					readl_relaxed(qce->base +
						      CE2_REG_DATA_OUT);
				}
				if (drain > 0)
					dev_dbg(qce->dev,
						"CE2 skc DOUT drained %d dword(s) post-chunk off=%u\n",
						drain, sg_off);
			}

			/* Capture next-chunk IV.
			 *
			 * For CBC: the IV for chunk N+1 is mathematically the
			 * last cipher block of chunk N -- we already have that
			 * sitting in bounce.dst_copy from the byte-swap step
			 * inside qce_ce2_dma_inout_cipher().  Use the buffer
			 * directly instead of reading the engine's CNTR0..3
			 * registers.  This eliminates a delicate timing
			 * window: STATUS goes IDLE one cycle before the
			 * engine's final write to CNTR fully commits to the
			 * readable bank on this silicon, so an occasional
			 * CNTR read returns stale state and corrupts block 0
			 * of the next chunk.  Empirically observed:
			 *   - 256 B chunk 4 block 0 failure (byte 192)
			 *   - 8 KB chunk 26 block 0 failure (byte 1248)
			 *   - 32 KB chunks 149 & 465 block 0 failures
			 * All exactly at chunk boundary block 0 -- the
			 * signature of stale CNTR readback.
			 *
			 * For CTR: the counter increments inside the engine
			 * per AES block and the post-chunk value is NOT
			 * derivable from ciphertext, so we still need the
			 * CNTR readback (and tolerate the rare timing race
			 * for CTR mode -- not used by the kernel CBC paths
			 * we care about).
			 *
			 * dst_copy holds post-byte-swap output in big-endian
			 * byte order, which is the same byte order CNTR
			 * readback would produce (cast to __be32).  memcpy
			 * preserves byte order; the next chunk's writel
			 * loop above does the same cast we used originally.
			 */
			if (!IS_ECB(flags) && enciv_words) {
				if (IS_CBC(flags)) {
					/* dst_copy holds BE-byte ciphertext.
					 * Chip's CNTR0..3 are u32 with the
					 * cipher block's first byte at MSB
					 * (BE-packed); the existing readback
					 * stores chip's u32 verbatim into
					 * enciv[] (raw u32 on LE memory).
					 * Reproduce that layout: read each
					 * 4-byte BE chunk of dst_copy as a
					 * u32 and store it as the BE-packed
					 * value chip would have produced.
					 */
					const u8 *p = bounce.dst_copy +
						      chunk_len -
						      enciv_words * 4;
					for (k = 0; k < enciv_words; k++)
						enciv[k] = (__force __be32)
							get_unaligned_be32(
								p + k * 4);
				} else {
					for (k = 0; k < enciv_words; k++)
						enciv[k] = (__force __be32)readl(
							qce->base +
							CE2_REG_CNTR0_IV0 +
							k * 4);
				}
			}

			sg_off += chunk_len;
		}

		qce_ce2_free_bounce(qce, &bounce);
	}

	/* Final IV writeback to caller for chained mode.
	 *
	 * enciv[] in both paths (CBC via dst_copy or CTR via CNTR readl)
	 * holds chip-format BE-packed u32 values stored as raw u32 in
	 * LE memory.  Convert each u32 to BE byte order for the caller's
	 * rctx->iv (which uses the standard FIPS-byte AES convention).
	 */
	if (!IS_ECB(flags) && rctx->iv && enciv_words) {
		for (k = 0; k < enciv_words; k++)
			put_unaligned_be32((__force u32)enciv[k],
					   rctx->iv + k * 4);
	}

	return 0;
}
#endif	/* CONFIG_CRYPTO_DEV_QCE_SKCIPHER */

int qce_check_status(struct qce_device *qce, u32 *status)
{
	int ret = 0;

	*status = qce_read(qce, qce_reg_status(qce));

	if (qce_is_ce2(qce)) {
		/*
		 * CE2 status register has different bit layout.
		 * Check for errors and auth done status.
		 */
		if (*status & STATUS_ERRORS_CE2)
			ret = -ENXIO;
		else if (!(*status & BIT(CE2_AUTH_DONE_SHIFT)))
			ret = -ENXIO;
	} else {
		/*
		 * Don't use result dump status. The operation may not be complete.
		 * Instead, use the status we just read from device. In case, we need to
		 * use result_status from result dump the result_status needs to be byte
		 * swapped, since we set the device to little endian.
		 */
		if (*status & STATUS_ERRORS_V5 || !(*status & BIT(OPERATION_DONE_SHIFT)))
			ret = -ENXIO;
		else if (*status & BIT(MAC_FAILED_SHIFT))
			ret = -EBADMSG;
	}

	return ret;
}

void qce_get_version(struct qce_device *qce, u32 *major, u32 *minor, u32 *step)
{
	u32 val;

	if (qce_is_ce2(qce)) {
		/*
		 * CE2 doesn't have a version register at offset 0x000.
		 * Version info is in STATUS register bits 31-28.
		 */
		val = qce_read(qce, CE2_REG_STATUS);
		*major = (val & CE2_CORE_REV_MASK) >> CE2_CORE_REV_SHIFT;
		*minor = 0;
		*step = 0;
	} else {
		val = qce_read(qce, REG_VERSION);
		*major = (val & CORE_MAJOR_REV_MASK) >> CORE_MAJOR_REV_SHIFT;
		*minor = (val & CORE_MINOR_REV_MASK) >> CORE_MINOR_REV_SHIFT;
		*step = (val & CORE_STEP_REV_MASK) >> CORE_STEP_REV_SHIFT;
	}
}
