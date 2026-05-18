// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 */

#include <crypto/internal/hash.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/types.h>
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
static int qce_ce2_dma_inout_cipher(struct qce_device *qce,
				    struct scatterlist *src,
				    struct scatterlist *dst,
				    unsigned int nbytes,
				    unsigned int block_dwords)
{
	struct dma_chan *rx = qce->dma.rxchan;
	struct dma_chan *tx = qce->dma.txchan;
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
		/* burst = cipher block size: ADM walks one block per CRCI
		 * handshake so the engine can fully produce/consume a block
		 * before the next handshake fires.  AES = 4 dw (16 B);
		 * DES/3DES = 2 dw (8 B).  Using 4 dw for 3DES caused ADM to
		 * read 16 bytes while engine only produced 8 valid bytes
		 * (other 8 read back as zero from DATA_SHADOW0).
		 */
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
	struct dma_async_tx_descriptor *in_desc, *out_desc;
	struct completion done;
	dma_cookie_t in_cookie, out_cookie;
	dma_addr_t in_dma, out_dma;
	__le32 *in_buf, *out_buf;
	u8 *src_copy = NULL, *dst_copy = NULL;
	enum dma_status status;
	unsigned int burst_bytes = block_dwords * 4;
	unsigned int padded = round_up(nbytes, burst_bytes);
	unsigned int dwords;
	unsigned int i;
	int ret;

	if (!padded)
		return -EINVAL;
	dwords = padded / 4;

	src_copy = kzalloc(padded, GFP_KERNEL);
	if (!src_copy)
		return -ENOMEM;
	dst_copy = kzalloc(padded, GFP_KERNEL);
	if (!dst_copy) {
		ret = -ENOMEM;
		goto out_free_src_copy;
	}
	sg_copy_to_buffer(src, sg_nents(src), src_copy, nbytes);

	in_buf = dma_alloc_coherent(qce->dev, padded, &in_dma, GFP_KERNEL);
	if (!in_buf) {
		ret = -ENOMEM;
		goto out_free_dst_copy;
	}
	out_buf = dma_alloc_coherent(qce->dev, padded, &out_dma, GFP_KERNEL);
	if (!out_buf) {
		ret = -ENOMEM;
		goto out_free_in;
	}

	/* Byte-swap input dwords to BE (CE2 reads MSB-first) */
	for (i = 0; i < dwords; i++) {
		u32 w = ((u32)src_copy[i * 4 + 0] << 24) |
			((u32)src_copy[i * 4 + 1] << 16) |
			((u32)src_copy[i * 4 + 2] <<  8) |
			((u32)src_copy[i * 4 + 3] <<  0);
		in_buf[i] = cpu_to_le32(w);
	}

	/* Configure rxchan for input (CRCI 4 -> DATA_SHADOW0).  Earlier
	 * hash ops may have left rxchan reconfigured for digest readback
	 * (CRCI 15, src=AUTH_IV0) so we re-establish the cipher input
	 * config here unconditionally.
	 */
	ret = dmaengine_slave_config(rx, &in_conf);
	if (ret) {
		dev_err(qce->dev,
			"CE2 cipher: in slave_config failed: %d\n", ret);
		goto out_free_out;
	}
	in_desc = dmaengine_prep_slave_single(rx, in_dma, padded,
					      DMA_MEM_TO_DEV, DMA_CTRL_ACK);
	if (!in_desc) {
		dev_err(qce->dev, "CE2 cipher: in prep failed\n");
		ret = -ENOMEM;
		goto out_terminate_rx;
	}

	/* Configure txchan for output (CRCI 5 <- DATA_SHADOW0).  Default
	 * config from probe points txchan at AUTH_IV0 with CRCI 15 for
	 * the hash digest path; override here for cipher output.
	 */
	ret = dmaengine_slave_config(tx, &out_conf);
	if (ret) {
		dev_err(qce->dev,
			"CE2 cipher: out slave_config failed: %d\n", ret);
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
	sg_copy_from_buffer(dst, sg_nents(dst), dst_copy, nbytes);

	ret = 0;

out_terminate:
	dmaengine_terminate_sync(tx);
out_terminate_rx:
	dmaengine_terminate_sync(rx);
out_free_out:
	dma_free_coherent(qce->dev, padded, out_buf, out_dma);
out_free_in:
	dma_free_coherent(qce->dev, padded, in_buf, in_dma);
out_free_dst_copy:
	kfree(dst_copy);
out_free_src_copy:
	kfree(src_copy);
	return ret;
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
		/* longer assert + deassert hold than hash path.
		 * 1ms post-deassert alone fixed AES-256-CBC encrypt at
		 * cold-start but decrypt still failed block-2 passthrough.
		 * Try a longer assert too -- maybe the AES core's decrypt
		 * pipeline needs a fuller reset cycle than the simple
		 * 10us hash settle covers.
		 */
		usleep_range(1000, 1500);
		reset_control_deassert(qce->reset);
		usleep_range(1000, 1500);
	}

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
	encr_cfg |= BIT(CE2_FIRST_SHIFT) | BIT(CE2_LAST_SHIFT) |
		    BIT(CE2_CLR_CNTXT_SHIFT);

	/* No auth segment for skcipher-only */
	writel(0, qce->base + CE2_REG_AUTH_SEG_CFG);

	/* Key writes: DES_KEY0..5 for DES/3DES, AES_RNDKEY0..N for AES.
	 * APQ8060 CE2 reports CRYPTO_AES_SEL_FAST in ENGINES_AVAIL, so
	 * the raw key is written and the engine expands the round-key
	 * schedule internally (webOS pce_dev->fastaes=1 path).
	 */
	qce_cpu_to_be32p_array(enckey, ctx->enc_key, keylen);
	if (IS_DES(flags) || IS_3DES(flags)) {
		for (k = 0; k < enckey_words; k++)
			writel((__force u32)enckey[k],
			       qce->base + CE2_REG_DES_KEY0 + k * 4);
	} else {	/* AES */
		for (k = 0; k < enckey_words; k++)
			writel((__force u32)enckey[k],
			       qce->base + CE2_REG_AES_RNDKEY0 + k * 4);
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

	/* SEG_CFG -> ENCR_SEG_CFG -> SEG_SIZE (webOS order) */
	writel(encr_cfg, qce->base + CE2_REG_SEG_CFG);
	writel(rctx->cryptlen << CE2_ENCR_SEG_SIZE_SHIFT,
	       qce->base + CE2_REG_ENCR_SEG_CFG);
	writel(rctx->cryptlen, qce->base + CE2_REG_SEG_SIZE);

	/* CONFIG: high-speed enable + interrupt masks (same as hash) */
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

	dev_info(qce->dev,
		 "CE2 skc pre-GO: SEG_CFG=0x%08x len=%u CONFIG=0x%08x flags=0x%lx keylen=%u\n",
		 encr_cfg, rctx->cryptlen, config, flags, keylen);

	/* GOPROC */
	writel(BIT(CE2_GO_SHIFT), qce->base + CE2_REG_GOPROC);

	/* Wait for engine to leave IDLE (transition to LOCKED/PROCESSING) */
	for (timeout = 1000; timeout > 0; timeout--) {
		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if (status & CE2_CRYPTO_STATE_MASK)
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		dev_err(qce->dev,
			"CE2 skc: engine never left IDLE; STATUS=0x%08x\n",
			status);
		return -ETIMEDOUT;
	}

	/* Dual-channel DMA: input -> DATA_SHADOW0, output <- DATA_SHADOW0.
	 * DES/3DES use 8-byte blocks (2 dwords); AES uses 16-byte (4 dwords).
	 * The ADM burst must match the engine's block size so the per-block
	 * CRCI handshake aligns with what the engine produces/consumes.
	 */
	ret = qce_ce2_dma_inout_cipher(qce, req->src, req->dst, rctx->cryptlen,
				       (IS_DES(flags) || IS_3DES(flags)) ? 2 : 4);
	dev_info(qce->dev,
		 "CE2 skc post-DMA: ret=%d STATUS=0x%08x\n", ret,
		 readl_relaxed(qce->base + CE2_REG_STATUS));
	if (ret)
		return ret;

	/* Read back IV from CNTR0..3 for CBC/CTR chaining.  ADM has done
	 * the actual encryption already; the engine has updated CNTR
	 * registers with either the last ciphertext block (CBC) or the
	 * incremented counter (CTR).  Convert from raw u32 to BE bytes
	 * before handing back to caller.
	 */
	if (!IS_ECB(flags) && rctx->iv && enciv_words) {
		for (k = 0; k < enciv_words; k++) {
			u32 w = readl(qce->base +
				      CE2_REG_CNTR0_IV0 + k * 4);
			__be32 be = cpu_to_be32(w);

			memcpy(rctx->iv + k * 4, &be, sizeof(be));
		}
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
