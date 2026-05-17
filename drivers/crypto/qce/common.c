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
	unsigned int iv_words, total, dwords, i;
	u32 auth_cfg, config, status, *p;
	u8 *buf;
	int timeout, ret = 0;

	/* If not the last block, size must be on the block boundary
	 * (matches qce_setup_regs_ahash precondition).
	 */
	if (!rctx->last_blk && req->nbytes % blocksize)
		return -EINVAL;

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

	/* 9) Feed input in 16-byte chunks. CE2 FIFO consumes 16 bytes per
	 *    CRCI handshake; trailing bytes past SEG_SIZE are ignored.
	 */
	total = round_up(req->nbytes, 16);
	if (total == 0)
		total = 16;

	buf = kzalloc(total, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (req->nbytes)
		sg_copy_to_buffer(req->src, sg_nents(req->src), buf,
				  req->nbytes);

	/* Byte-swap each 4-byte chunk to big-endian for DATA_IN. CE2 interprets
	 * each 32-bit DATA_IN write MSB-first as the next four message bytes.
	 *
	 * Example: input bytes "test" = 74 65 73 74 must be presented to CE2
	 * as u32 0x74657374 (MSB-first reads as 0x74 'e' 's' 't'). But buf
	 * holds bytes in CPU-native order; on ARM LE, *(u32 *)buf would give
	 * 0x74736574 -- which CE2 reads as bytes "tset" and hashes that.
	 * The probe-time PIO test in core.c hardcoded test_data=0x74657374,
	 * which is why it produced correct SHA1("test") -- it skipped this
	 * conversion since the constant was already in BE form.
	 */
	dwords = total / 4;
	dev_info(qce->dev,
		 "CE2 hash: feeding %u dwords (BE-swapped from buf)\n", dwords);
	for (i = 0; i < dwords; i++) {
		u32 word = ((u32)buf[i * 4 + 0] << 24) |
			   ((u32)buf[i * 4 + 1] << 16) |
			   ((u32)buf[i * 4 + 2] <<  8) |
			   ((u32)buf[i * 4 + 3] <<  0);

		for (timeout = 10000; timeout > 0; timeout--) {
			status = readl_relaxed(qce->base + CE2_REG_STATUS);
			if (status & BIT(CE2_DIN_RDY_SHIFT))
				break;
			udelay(10);
		}
		if (timeout <= 0) {
			dev_err(qce->dev,
				"CE2 hash: DIN_RDY stuck after %u/%u dwords; STATUS=0x%08x\n",
				i, dwords, status);
			ret = -ETIMEDOUT;
			goto out;
		}
		/*
		 * writel() (not writel_relaxed) so each DATA_IN word fully
		 * commits before we poll DIN_RDY for the next.  Samsung's TZ
		 * does a dsb after every DATA_IN write; this is the Linux
		 * equivalent.
		 */
		writel(word, qce->base + CE2_REG_DATA_IN);
	}

	/* 10) Wait for AUTH_DONE */
	timeout = max(50000U, dwords * 100U);
	for (; timeout > 0; timeout--) {
		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if (status & BIT(CE2_AUTH_DONE_SHIFT))
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		dev_err(qce->dev, "CE2 hash: AUTH_DONE timeout; STATUS=0x%08x\n",
			status);
		ret = -ETIMEDOUT;
		goto out;
	}

	if (status & BIT(CE2_SW_ERR_SHIFT)) {
		dev_err(qce->dev, "CE2 hash: SW_ERR set; STATUS=0x%08x\n",
			status);
		ret = -EIO;
		goto out;
	}

	/*
	 * Read digest via ADM DMA from AUTH_IV0 with CRCI 15 handshake.
	 * This is what drives the engine through FINAL_READ ->
	 * CTXT_CLEARING -> UNLOCKING -> IDLE so that the next op can run.
	 * Plain readl() returns correct bytes but doesn't fire the
	 * handshake, leading to a wedge after ~2 SHA256 ops.
	 */
	{
		__be32 result[SHA256_DIGEST_SIZE / sizeof(__be32)];

		ret = qce_ce2_dma_read_digest(qce, result, digestsize);
		if (ret)
			goto out;

		memcpy(rctx->digest, result, digestsize);
		if (req->result && rctx->last_blk)
			memcpy(req->result, result, digestsize);
	}

	/* Capture byte count for streaming-hash continuation */
	rctx->byte_count[0] = cpu_to_be32(readl_relaxed(
		qce->base + CE2_REG_AUTH_BYTECNT0));
	rctx->byte_count[1] = cpu_to_be32(readl_relaxed(
		qce->base + CE2_REG_AUTH_BYTECNT1));

out:
	kfree(buf);
	return ret;
}
#endif

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
