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
#include <linux/iopoll.h>
#include <linux/reset.h>
#include <linux/sizes.h>
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>

/*
 * Per-op AES debug instrumentation. Default off; toggle at runtime via
 *   echo 1 > /sys/module/qcrypto/parameters/aes_debug
 * Used to capture the full byte journey through key/IV/PT writes,
 * register readbacks, and output byteswaps so we can isolate where
 * the AES path diverges from NIST-expected bytes.
 */
static bool qce_aes_debug;
module_param_named(aes_debug, qce_aes_debug, bool, 0644);
MODULE_PARM_DESC(aes_debug,
		 "Enable per-op AES register/buffer dumps (debug only)");

#define QCE_DBG_BUF(qce, label, buf, len) do {				\
	if (qce_aes_debug)						\
		print_hex_dump(KERN_INFO, "qce-dbg " label ": ",	\
			       DUMP_PREFIX_OFFSET, 16, 1, (buf), (len), false); \
} while (0)

#define QCE_DBG(qce, fmt, ...) do {					\
	if (qce_aes_debug)						\
		dev_info((qce)->dev, "qce-dbg: " fmt, ##__VA_ARGS__);	\
} while (0)

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
/*
 * v5 (BAM-based) hash register setup. CE2 (MSM8x60) callers bypass
 * this entirely via qce_ce2_pio_run_hash() before qce_start(), so any
 * qce_is_ce2() branch here would be dead code on every platform.
 */
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
		qce_write_array(qce, REG_AUTH_KEY0, (u32 *)mackey,
				authkey_words);
	}

	if (IS_CMAC(rctx->flags))
		goto go_proc;

	if (rctx->first_blk)
		memcpy(auth, rctx->digest, digestsize);
	else
		qce_cpu_to_be32p_array(auth, rctx->digest, digestsize);

	iv_words = (IS_SHA1(rctx->flags) || IS_SHA1_HMAC(rctx->flags)) ? 5 : 8;
	qce_write_array(qce, REG_AUTH_IV0, (u32 *)auth, iv_words);

	if (rctx->first_blk)
		qce_clear_array(qce, REG_AUTH_BYTECNT0, 4);
	else
		qce_write_array(qce, REG_AUTH_BYTECNT0,
				(u32 *)rctx->byte_count, 2);

	auth_cfg = qce_auth_cfg(qce, rctx->flags, 0, digestsize);

	if (rctx->last_blk)
		auth_cfg |= BIT(AUTH_LAST_SHIFT);
	else
		auth_cfg &= ~BIT(AUTH_LAST_SHIFT);

	if (rctx->first_blk)
		auth_cfg |= BIT(AUTH_FIRST_SHIFT);
	else
		auth_cfg &= ~BIT(AUTH_FIRST_SHIFT);

go_proc:
	qce_write(qce, REG_AUTH_SEG_CFG, auth_cfg);
	qce_write(qce, REG_AUTH_SEG_SIZE, req->nbytes);
	qce_write(qce, REG_AUTH_SEG_START, 0);
	qce_write(qce, REG_ENCR_SEG_CFG, 0);
	qce_write(qce, REG_SEG_SIZE, req->nbytes);

	/* get little endianness */
	config = qce_config_reg(qce, 1);
	qce_write(qce, REG_CONFIG, config);

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
	u8 swap[QCE_AES_IV_LENGTH] = {0};
	unsigned int i, offset;

	if (ivsize > QCE_AES_IV_LENGTH)
		return;

	offset = QCE_AES_IV_LENGTH - ivsize;

	/* Reverse and right-align IV bytes. */
	for (i = 0; i < ivsize; i++)
		swap[offset + i] = src[ivsize - 1 - i];

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

/*
 * v5 (BAM-based) cipher register setup. CE2 (MSM8x60) callers bypass
 * this entirely via qce_ce2_pio_run_skcipher() before qce_start(), so
 * any qce_is_ce2() branch here would be dead code on every platform.
 */
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

	qce_write(qce, REG_AUTH_SEG_CFG, auth_cfg);

	encr_cfg = qce_encr_cfg(qce, flags, keylen);

	if (IS_DES(flags)) {
		enciv_words = 2;
		enckey_words = 2;
	} else if (IS_3DES(flags)) {
		enciv_words = 2;
		enckey_words = 6;
	} else if (IS_AES(flags)) {
		if (IS_XTS(flags))
			qce_xtskey(qce, ctx->enc_key, ctx->enc_keylen,
				   rctx->cryptlen);
		enciv_words = 4;
	} else {
		return -EINVAL;
	}

	qce_write_array(qce, REG_ENCR_KEY0, (u32 *)enckey, enckey_words);

	if (!IS_ECB(flags)) {
		if (IS_XTS(flags))
			qce_xts_swapiv(enciv, rctx->iv, ivsize);
		else
			qce_cpu_to_be32p_array(enciv, rctx->iv, ivsize);

		qce_write_array(qce, REG_CNTR0_IV0, (u32 *)enciv, enciv_words);
	}

	if (IS_ENCRYPT(flags))
		encr_cfg |= BIT(ENCODE_SHIFT);

	qce_write(qce, REG_ENCR_SEG_CFG, encr_cfg);
	qce_write(qce, REG_ENCR_SEG_SIZE, rctx->cryptlen);
	qce_write(qce, REG_ENCR_SEG_START, 0);

	if (IS_CTR(flags)) {
		qce_write(qce, REG_CNTR_MASK, ~0);
		qce_write(qce, REG_CNTR_MASK0, ~0);
		qce_write(qce, REG_CNTR_MASK1, ~0);
		qce_write(qce, REG_CNTR_MASK2, ~0);
	}

	qce_write(qce, REG_SEG_SIZE, rctx->cryptlen);

	/* get little endianness */
	config = qce_config_reg(qce, 1);
	qce_write(qce, REG_CONFIG, config);

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
		.crci = qce->adm_crci_in,
	};
	struct qcom_adm_peripheral_config out_periph = {
		.crci = qce->adm_crci_hash,
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

	/*
	 * Hardware cap: CE2_REG_SEG_SIZE and CE2_AUTH_SEG_SIZE are both
	 * 16-bit fields (bits 15:0 of SEG_SIZE; bits 31:16 of AUTH_SEG_CFG).
	 * The engine cannot accept more than 65535 B per GOPROC. Round down
	 * to a 16-byte input-FIFO boundary (65520) so round_up(nbytes, 16)
	 * below cannot overflow into the field's high bits and silently
	 * truncate. Earlier code allowed up to SZ_1M; anything > 65535
	 * silently truncated AUTH_SEG_SIZE to the low 16 bits and the
	 * engine hashed only the first ~64 KiB while reporting success.
	 *
	 * Return -EMSGSIZE so AHASH layer callers can detect the limit and
	 * chunk at the update boundary. Each AHASH update accumulates into
	 * the same partial digest, so transparent caller-side chunking is
	 * lossless. dm-integrity, fscrypt and IPsec large-payload callers
	 * that pass multi-MB single updates need to either chunk or fall
	 * back to software SHA.
	 *
	 * Transparent internal chunking is a separate follow-up; it would
	 * manage first/last flags and AUTH_IV0..N carry-over across
	 * per-chunk GOPROC.
	 */
	if (nbytes > 65520)
		return -EMSGSIZE;

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

	/*
	 * Stage the input dwords so the bytes in memory are in the order
	 * the CE2 engine consumes them. The engine reads each 32-bit beat
	 * MSB-first, so the memory layout must be big-endian:
	 *     src_copy[i*4+0] -> high byte of beat i
	 *     src_copy[i*4+3] -> low  byte of beat i
	 * Build the value in CPU byte order then store via cpu_to_be32()
	 * so the resulting memory bytes are [b0, b1, b2, b3] (BE) on any
	 * host. cpu_to_le32() would be wrong on a little-endian host (the
	 * no-op store leaves bytes in [b3, b2, b1, b0] order, the opposite
	 * of what the engine consumes).
	 */
	in_dwords = in_total / 4;
	in_words = (__le32 *)in_buf;
	for (i = 0; i < in_dwords; i++) {
		u32 w = ((u32)src_copy[i * 4 + 0] << 24) |
			((u32)src_copy[i * 4 + 1] << 16) |
			((u32)src_copy[i * 4 + 2] <<  8) |
			((u32)src_copy[i * 4 + 3] <<  0);
		in_words[i] = (__force __le32)cpu_to_be32(w);
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
	 * GCC_CE2_RESET (DT: reset-names = "engine") asserts the engine's
	 * reset line.  10 us is enough for the engine to fully reset (the
	 * probe-time SW_RST sequence in core.c uses similar timing).
	 * After deassert the engine is back to power-on state -- our
	 * per-op SEG_CFG/AUTH_IV/AUTH_BYTECNT/CONFIG/GOPROC writes below
	 * re-program it from scratch.
	 *
	 * udelay (not usleep_range) on both flanks: GCC reset propagation
	 * is a hardware-level timing requirement on the order of single-
	 * digit microseconds and cannot tolerate scheduler latency. Total
	 * cost per op: 2 × ~10 us = ~20 us. The other udelay() sites in
	 * this file are similarly short bounded waits (post-GOPROC IDLE
	 * leave: max 1000 × 10 us = 10 ms; CTR IV settle / AUTH_DONE poll:
	 * max 50 × 10 us = 500 us) — they were previously open-coded as
	 * 10000 × udelay(10) busy-spins (~100 ms) and are now bounded to
	 * the actual hardware latency envelope.
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
	/*
	 * Yield-friendly equivalent of the previous open-coded
	 * 10000 × udelay(10) busy-spin (~100 ms hard CPU monopolise per
	 * call). readl_poll_timeout uses usleep_range(sleep_us, 2*sleep_us)
	 * between reads, which yields the CPU. The total wall-time bound
	 * is the same (10 µs × 10 000 = 100 ms), but other tasks can now
	 * run, preventing soft-lockup / RCU-stall warnings under
	 * sustained AF_ALG load.
	 */
	readl_poll_timeout(qce->base + CE2_REG_STATUS, status,
			   (status & CE2_CRYPTO_STATE_MASK) == 0,
			   10, 100000);

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

	dev_dbg(qce->dev,
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
		u32 hash_status;

		ret = qce_ce2_dma_chain_input_digest(qce, req->src, req->nbytes,
						     result, digestsize);
		if (ret)
			goto out;

		/*
		 * Check CE2 STATUS for hardware-side error bits before
		 * trusting the AUTH_IV0..N bytes that qce_ce2_dma_chain_input_digest
		 * just copied into `result`. The cipher path validates via
		 * qce_check_status() (STATUS_ERRORS_CE2) but the hash path
		 * bypasses that helper for performance — without this check
		 * an engine that raised SW_ERR / DIN_ERR / DOUT_ERR /
		 * ACCESS_VIOL mid-operation would silently produce a stale
		 * or zero digest, and the caller (kernel crypto API, AF_ALG)
		 * has no way to detect it.
		 */
		hash_status = readl_relaxed(qce->base + CE2_REG_STATUS);
		if (hash_status & STATUS_ERRORS_CE2) {
			dev_dbg(qce->dev,
				"CE2 hash: post-DMA STATUS=0x%08x has error bits — discarding digest\n",
				hash_status);
			ret = -ENXIO;
			goto out;
		}

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
	/* Wipe raw digest bytes from the kernel stack — memzero_explicit
	 * survives compiler dead-store elimination so an attacker cannot
	 * read intermediate hash state from re-used stack frames.
	 */
	memzero_explicit(auth, sizeof(auth));
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
	u32 status_before, status_after;
	int timeout;

	status_before = readl_relaxed(qce->base + CE2_REG_STATUS);
	writel(BIT(CE2_GO_SHIFT), qce->base + CE2_REG_GOPROC);

	/* Engine leaves IDLE within microseconds; ADM issue_pending
	 * fires immediately after.
	 */
	for (timeout = 1000; timeout > 0; timeout--) {
		status_after = readl_relaxed(qce->base + CE2_REG_STATUS);
		if (status_after & CE2_CRYPTO_STATE_MASK)
			break;
		udelay(10);
	}

	dev_info(qce->dev,
		 "CE2 GOPROC: STATUS pre=0x%08x post=0x%08x iter_remaining=%d (%s)\n",
		 status_before, status_after, timeout,
		 timeout > 0 ? "left IDLE" : "STUCK IN IDLE after 10ms");
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
	/*
	 * Per-word byteswap in the ADM controller (CMD_DST_SWAP_BYTES |
	 * CMD_DST_SWAP_SHORTS for input, CMD_SRC_SWAP_BYTES |
	 * CMD_SRC_SWAP_SHORTS for output) — matches Samsung's vendor
	 * driver chain command setup, removes the previous software-side
	 * byteswap in qce_ce2_dma_inout_cipher input/output prep, and
	 * (hypothesis under test) lifts the engine's apparent 4-block
	 * limit per GOPROC that the software-swap path hit.
	 */
	struct qcom_adm_peripheral_config in_periph = {
		.crci = qce->adm_crci_in,
		.swap_bytes = true,
		.swap_shorts = true,
	};
	struct qcom_adm_peripheral_config out_periph = {
		.crci = qce->adm_crci_out,
		.swap_bytes = true,
		.swap_shorts = true,
	};
	struct dma_slave_config in_conf = {
		.direction = DMA_MEM_TO_DEV,
		.device_fc = true,	/* use CRCI flow control */
		.dst_addr = qce->phys_base + CE2_REG_DATA_SHADOW0,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_maxburst = block_dwords,
		.peripheral_config = &in_periph,
		.peripheral_size = sizeof(in_periph),
	};
	struct dma_slave_config out_conf = {
		.direction = DMA_DEV_TO_MEM,
		.device_fc = true,	/* use CRCI flow control */
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
	int ret;

	if (!padded || padded > b->size)
		return -EINVAL;

	dev_info(qce->dev,
		 "CE2 dma_inout entry: nbytes=%u burst_dwords=%u padded=%u in_dma=%pad out_dma=%pad\n",
		 nbytes, block_dwords, padded, &in_dma, &out_dma);

	memset(src_copy, 0, padded);
	memset(dst_copy, 0, padded);
	sg_pcopy_to_buffer(src, sg_nents(src), src_copy, nbytes, sg_offset);

	QCE_DBG_BUF(qce, "src_copy raw PT (first 32 B)",
		    src_copy, min_t(unsigned, 32U, padded));

	/*
	 * No software byteswap here. The ADM does per-word byteswap in
	 * hardware via CMD_DST_SWAP_BYTES | CMD_DST_SWAP_SHORTS (set
	 * via qcom_adm_peripheral_config.swap_bytes/swap_shorts in
	 * qce_ce2_dma_setup_cipher_chans). Pass the user bytes through
	 * to DMA verbatim; ADM swaps them into the byte-order the
	 * engine wants when it writes to DATA_SHADOW0.
	 */
	memcpy(in_buf, src_copy, padded);

	QCE_DBG_BUF(qce, "in_buf (ADM byteswaps; first 32 B, to engine)",
		    (u8 *)in_buf, min_t(unsigned, 32U, padded));

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
	dev_info(qce->dev,
		 "CE2 dma_inout: rx+tx submitted (in_cookie=%d out_cookie=%d), STATUS=0x%08x — calling pre_issue (GOPROC)\n",
		 in_cookie, out_cookie,
		 readl_relaxed(qce->base + CE2_REG_STATUS));

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

	dev_info(qce->dev,
		 "CE2 dma_inout: GOPROC fired, STATUS=0x%08x — issuing ADM rx then tx\n",
		 readl_relaxed(qce->base + CE2_REG_STATUS));
	dma_async_issue_pending(rx);
	dev_info(qce->dev,
		 "CE2 dma_inout: rx issued, STATUS=0x%08x\n",
		 readl_relaxed(qce->base + CE2_REG_STATUS));
	dma_async_issue_pending(tx);
	dev_info(qce->dev,
		 "CE2 dma_inout: tx issued, STATUS=0x%08x — waiting for completion (1s timeout)\n",
		 readl_relaxed(qce->base + CE2_REG_STATUS));

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

	QCE_DBG(qce,
		"post-DMA: STATUS=0x%08x SEG_CFG=0x%08x ENCR_SEG_CFG=0x%08x SEG_SIZE=0x%08x\n",
		readl(qce->base + CE2_REG_STATUS),
		readl(qce->base + CE2_REG_SEG_CFG),
		readl(qce->base + CE2_REG_ENCR_SEG_CFG),
		readl(qce->base + CE2_REG_SEG_SIZE));
	QCE_DBG_BUF(qce, "out_buf raw engine output (first 32 B, ADM-swapped)",
		    (u8 *)out_buf, min_t(unsigned, 32U, padded));

	/*
	 * No software byteswap here either. ADM SRC_SWAP on the output
	 * channel has already byteswapped the engine's BE-packed integer
	 * output into user byte order in out_buf. memcpy out → user dst
	 * directly.
	 */
	memcpy(dst_copy, out_buf, padded);

	QCE_DBG_BUF(qce, "dst_copy (first 32 B, to user)",
		    dst_copy, min_t(unsigned, 32U, padded));

	sg_pcopy_from_buffer(dst, sg_nents(dst), dst_copy, nbytes, sg_offset);

	ret = 0;

out_terminate:
	/*
	 * DEBUG (throwaway): dump the wedge state BEFORE the engine reset
	 * below clobbers it back to post-reset (0x10200004 + friends). This
	 * is the only place we can observe the real stuck state of the
	 * engine + ADM channels at timeout.
	 */
	{
		u32 status, seg_cfg, encr_seg_cfg, seg_size, config;
		unsigned int avail;

		status = readl_relaxed(qce->base + CE2_REG_STATUS);
		seg_cfg = readl_relaxed(qce->base + CE2_REG_SEG_CFG);
		encr_seg_cfg = readl_relaxed(qce->base + CE2_REG_ENCR_SEG_CFG);
		seg_size = readl_relaxed(qce->base + CE2_REG_SEG_SIZE);
		config = readl_relaxed(qce->base + CE2_REG_CONFIG);
		avail = (status & CE2_DOUT_SIZE_AVAIL_MASK) >>
			CE2_DOUT_SIZE_AVAIL_SHIFT;

		dev_info(qce->dev,
			 "CE2 PRE-RESET WEDGE DUMP (ret=%d nbytes=%u padded=%u):\n"
			 "  STATUS       = 0x%08x  (DOUT_AVAIL=%u)\n"
			 "  CONFIG       = 0x%08x\n"
			 "  SEG_CFG      = 0x%08x\n"
			 "  ENCR_SEG_CFG = 0x%08x\n"
			 "  SEG_SIZE     = 0x%08x\n",
			 ret, nbytes, padded,
			 status, avail, config, seg_cfg, encr_seg_cfg, seg_size);
	}

	/*
	 * Hard-reset the engine BEFORE terminating the ADM channels.
	 *
	 * qcom_adm's terminate_all takes the controller submit_lock and
	 * waits for the in-flight box descriptor to drain. If the engine
	 * has wedged mid-op (DOUT FIFO holding output the engine refuses
	 * to release; DIN FIFO waiting for input the engine refuses to
	 * accept) it has stopped firing CRCI handshakes — the ADM
	 * descriptor never drains, terminate_sync never returns, and
	 * the CPU that called us deadlocks waiting for an ADM IRQ that
	 * can never run (because it needs the same submit_lock we now
	 * indirectly hold via terminate_sync). On the device this
	 * cascades into RCU stalls and full system lockup within ~20 s.
	 *
	 * GCC_CE2_RESET releases the engine's CRCI lines: after
	 * deassert the engine no longer signals "ready" so ADM stops
	 * waiting for handshakes and the in-flight descriptor either
	 * completes (any remaining queued bytes flushed) or is cancelled
	 * cleanly. terminate_sync below then drains the now-quiescent
	 * channel without blocking.
	 *
	 * Skip if qce->reset is NULL (DT didn't wire reset-names =
	 * "engine") — older deployments without the reset line have to
	 * accept the deadlock risk, but mainline tenderloin DT does
	 * provide it.
	 */
	if (qce->reset) {
		reset_control_assert(qce->reset);
		udelay(10);
		reset_control_deassert(qce->reset);
		udelay(10);
	}
	dmaengine_terminate_sync(tx);
out_terminate_rx:
	dmaengine_terminate_sync(rx);
	return ret;
}

/*
 * CE2 max single-GOPROC = 65280 B (16-bit SEG_SIZE rounded down to a
 * 256-byte / burst*4 boundary). The per-op path will -EMSGSIZE any
 * request larger than this and let the skcipher / AF_ALG layer split.
 */
#define QCE_CE2_BOUNCE_MAX	65280U

/*
 * Pre-allocate the bounce-buffer set ONCE at probe (4 × 65280 B =
 * ~256 KB resident). The per-op fast path hands out the pre-allocated
 * pointers; qce->lock serialises ops on the engine, so no extra
 * locking. Replaces per-op dma_alloc_coherent (CMA migration + RCU
 * sync) which dominated the cipher path overhead (~3-5 ms / op,
 * capping throughput at ~7 MiB/s on long streams).
 */
static void qce_ce2_bounce_destroy_action(void *data)
{
	qce_ce2_bounce_destroy(data);
}

int qce_ce2_bounce_init(struct qce_device *qce)
{
	struct qce_ce2_bounce *b;
	int ret;

	if (!qce_is_ce2(qce))
		return 0;

	b = devm_kzalloc(qce->dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->size = QCE_CE2_BOUNCE_MAX;
	b->src_copy = devm_kzalloc(qce->dev, b->size, GFP_KERNEL);
	if (!b->src_copy)
		return -ENOMEM;
	b->dst_copy = devm_kzalloc(qce->dev, b->size, GFP_KERNEL);
	if (!b->dst_copy)
		return -ENOMEM;
	b->in_buf = dma_alloc_coherent(qce->dev, b->size, &b->in_dma,
				       GFP_KERNEL);
	if (!b->in_buf)
		return -ENOMEM;
	b->out_buf = dma_alloc_coherent(qce->dev, b->size, &b->out_dma,
					GFP_KERNEL);
	if (!b->out_buf) {
		dma_free_coherent(qce->dev, b->size, b->in_buf, b->in_dma);
		return -ENOMEM;
	}

	qce->ce2_bounce = b;

	ret = devm_add_action_or_reset(qce->dev,
				       qce_ce2_bounce_destroy_action, qce);
	if (ret)
		return ret;

	return 0;
}

void qce_ce2_bounce_destroy(struct qce_device *qce)
{
	struct qce_ce2_bounce *b = qce->ce2_bounce;

	if (!b)
		return;
	dma_free_coherent(qce->dev, b->size, b->out_buf, b->out_dma);
	dma_free_coherent(qce->dev, b->size, b->in_buf, b->in_dma);
	/* src_copy + dst_copy + b itself are devm-managed */
	qce->ce2_bounce = NULL;
}

/*
 * Hot-path: hand out the pre-allocated bounce. No allocation; no CMA.
 * Returns 0 on success; -EMSGSIZE if the caller wants more than we
 * pre-allocated (caller should chunk above us).
 */
static int qce_ce2_alloc_bounce(struct qce_device *qce,
				struct qce_ce2_bounce *b, size_t size)
{
	struct qce_ce2_bounce *src = qce->ce2_bounce;

	if (WARN_ON_ONCE(!src))
		return -ENOMEM;
	if (size > src->size)
		return -EMSGSIZE;
	*b = *src;
	b->size = size;
	return 0;
}

static void qce_ce2_free_bounce(struct qce_device *qce,
				struct qce_ce2_bounce *b)
{
	/* Pre-allocated pool — nothing to free per op. */
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
	u32 encr_cfg, status;
	int timeout, ret;
	unsigned int k;

	if (rctx->cryptlen == 0)
		return 0;

	dev_info(qce->dev,
		 "CE2 skc op entry: flags=0x%lx keylen=%u cryptlen=%u ivsize=%u\n",
		 flags, keylen, rctx->cryptlen, rctx->ivsize);

	/*
	 * The engine is reset ONCE at probe (qce_check_version() in core.c
	 * issues SW_RST + deassert). With the CRCI-5 blk_size bug fixed,
	 * engine state no longer drifts between ops, so the per-op SW_RST
	 * + MASK_INTR restore that used to live here is unnecessary
	 * overhead (~50 µs/op + IDLE poll). Match Samsung's vendor
	 * _ce_setup pattern: configure-once at init, per-op only writes
	 * SEG_CFG / ENCR_SEG_CFG / SEG_SIZE / keys / IV / GOPROC. The
	 * IDLE poll still runs below in case a previous op's STATUS hasn't
	 * settled — cheap (one or two register reads) and a useful gate.
	 */

	/* Wait for IDLE before configuring */
	/*
	 * Yield-friendly equivalent of the previous open-coded
	 * 10000 × udelay(10) busy-spin (~100 ms hard CPU monopolise per
	 * call). readl_poll_timeout uses usleep_range(sleep_us, 2*sleep_us)
	 * between reads, which yields the CPU. The total wall-time bound
	 * is the same (10 µs × 10 000 = 100 ms), but other tasks can now
	 * run, preventing soft-lockup / RCU-stall warnings under
	 * sustained AF_ALG load.
	 */
	readl_poll_timeout(qce->base + CE2_REG_STATUS, status,
			   (status & CE2_CRYPTO_STATE_MASK) == 0,
			   10, 100000);

	/*
	 * Drain any residual DOUT FIFO dwords from a prior op. On this
	 * silicon neither GCC_CE2_RESET nor SW_RST reliably empties the
	 * DOUT FIFO — empirically ~15% of cold-start ops produced silently
	 * wrong ciphertext (same wrong digest every time, matching the
	 * signature of stale FIFO data being pulled into the output ahead
	 * of the engine's real result). DATA_OUT (0x010) is the direct PIO
	 * drain port. DOUT_SIZE_AVAIL is a 3-bit field (max 7) so 8 reads
	 * are sufficient.
	 */
	{
		int drain;

		for (drain = 0; drain < 8; drain++) {
			u32 s = readl_relaxed(qce->base + CE2_REG_STATUS);
			u32 avail = (s >> CE2_DOUT_SIZE_AVAIL_SHIFT) & 0x7;

			if (avail == 0)
				break;
			readl_relaxed(qce->base + CE2_REG_DATA_OUT);
		}
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
	QCE_DBG(qce, "skc op flags=0x%lx keylen=%u cryptlen=%u ivsize=%u\n",
		flags, keylen, rctx->cryptlen, rctx->ivsize);
	QCE_DBG_BUF(qce, "raw user key (ctx->enc_key)", ctx->enc_key, keylen);

	qce_cpu_to_be32p_array(enckey, ctx->enc_key, keylen);
	QCE_DBG_BUF(qce, "enckey[] after qce_cpu_to_be32p_array",
		    (u8 *)enckey, keylen);

	if (IS_DES(flags) || IS_3DES(flags)) {
		for (k = 0; k < enckey_words; k++)
			writel((__force u32)enckey[k],
			       qce->base + CE2_REG_DES_KEY0 + k * 4);
	} else {	/* AES */
		/*
		 * AES_SEL_FAST path: ENGINES_AVAIL reports AES = FAST on
		 * this silicon, and the prior comment block notes "an
		 * earlier raw-key fast-path ... passed all NIST vectors
		 * <= 32 B". The slow path (software-expanded 60-dword
		 * round-key schedule) currently produces deterministic
		 * non-NIST AES output for varied-byte keys — confirmed by
		 * qce-nist with the NIST FIPS-197 vector
		 *   K=2b7e..., PT=6bc1bee2... → expected 3ad77bb4...
		 *   got ae5e647c... (off by full AES math).
		 *
		 * Switch to fast path: write ONLY the raw user key (4 dwords
		 * for AES-128, 6 for AES-192, 8 for AES-256) and let the
		 * engine expand internally. enckey[] is the user key already
		 * BE-packed by qce_cpu_to_be32p_array() above; writel on LE
		 * places bus bytes so the engine reads the user-key bytes
		 * in original order at RNDKEY0..N.
		 *
		 * If this path also wedges past ~64 bytes (as the older
		 * comment warned), we'll cap there and fix the slow-path
		 * byte order separately.
		 */
		/*
		 * Key write — NO byteswap. The engine reads RNDKEY0..N
		 * as 32-bit integers matching the writel value: enckey[k]
		 * is the user key as a BE-packed integer (via
		 * qce_cpu_to_be32p_array), writel encodes that integer onto
		 * the LE bus, engine reads back the same integer, and
		 * interprets it as the FIPS-197 BE-packed K[0..3] word for
		 * that round-0 column. Triangulation against software
		 * AES (see commit message) showed engine_K = NIST_K under
		 * this write pattern.
		 */
		for (k = 0; k < enckey_words; k++)
			writel((__force u32)enckey[k],
			       qce->base + CE2_REG_AES_RNDKEY0 + k * 4);
	}

	/* IV: CNTR0..3.  Skip for ECB (no IV) */
	if (!IS_ECB(flags) && rctx->iv && rctx->ivsize) {
		QCE_DBG_BUF(qce, "raw user IV", rctx->iv, rctx->ivsize);
		qce_cpu_to_be32p_array(enciv, rctx->iv, rctx->ivsize);
		QCE_DBG_BUF(qce, "enciv[] after qce_cpu_to_be32p_array",
			    (u8 *)enciv, rctx->ivsize);
		enciv_words = rctx->ivsize / sizeof(u32);
		for (k = 0; k < enciv_words; k++)
			writel((__force u32)enciv[k],
			       qce->base + CE2_REG_CNTR0_IV0 + k * 4);
		if (qce_aes_debug) {
			u32 rb[4];

			for (k = 0; k < enciv_words; k++)
				rb[k] = readl(qce->base +
					      CE2_REG_CNTR0_IV0 + k * 4);
			QCE_DBG_BUF(qce, "CNTR0..N readback",
				    (u8 *)rb, enciv_words * 4);
		}
	}

	/*
	 * Write CNTR_MASK = 0xffff for ALL AES modes (matches Samsung's
	 * _ce_setup, which writes it unconditionally in the AES branch
	 * before any cfg/keys). Previous scratch-driver iteration only
	 * wrote this for CTR; ECB/CBC may rely on CNTR_MASK initial state
	 * even though they don't "use" the counter, and Samsung's
	 * reference always writes it.
	 */
	if (IS_AES(flags))
		writel(0xffff, qce->base + CE2_REG_CNTR_MASK);

	/*
	 * HIGH_SPD_*_EN_N bits: 0 = high-speed enabled. The SW_RST pulse +
	 * MASK_INTR write above leaves these bits cleared (= enabled),
	 * which is what we want. Samsung's vendor driver never explicitly
	 * touches HIGH_SPD per op — dropping our per-op read-modify-write
	 * matches their pattern and avoids re-arming MASK_INTR state.
	 */

	/*
	 * CE2 cipher path: ONE GOPROC for the entire op (matches the
	 * Qualcomm downstream / HTC / Samsung vendor drivers for MSM8x60
	 * — see _ce_setup() in their drivers/crypto/msm/qce.c).
	 *
	 * Earlier iterations split AES into 48-byte chunks because the
	 * engine appeared to corrupt block 4 of long ops; that symptom was
	 * an artefact of chunking with FIRST|LAST set on every chunk: the
	 * engine treated each chunk as a standalone op and leaked DOUT
	 * FIFO state across "ops". Programming full cryptlen into
	 * SEG_SIZE, firing GOPROC once, and letting ADM stream the whole
	 * payload as a single descriptor pair avoids the FIFO leak
	 * entirely and gives DES/3DES-class throughput for AES too.
	 *
	 * SEG_SIZE is a 16-bit field; cap cryptlen at 65280 (65535 rounded
	 * down to a burst*4 = 256 boundary). The skcipher / AF_ALG layer
	 * above splits longer requests; we never see more than the cap in
	 * a single call.
	 */
	{
		/*
		 * burst = ADM CRCI handshake granularity in dwords. The
		 * CE2 engine signals CRCI per 16-byte (4-dword) block for
		 * AES and per 8-byte (2-dword) block for DES/3DES — same
		 * value qce_dma_configure_crci uses at probe time.
		 *
		 * The earlier 64 dwords (256 B) here was a leftover from
		 * the chunked iteration when device_fc was effectively
		 * bypassed; ADM blasted the whole burst without CRCI
		 * handshakes so the mismatch was invisible. Now that
		 * device_fc=true routes us through the CRCI-flow-controlled
		 * path, ADM expects to pull `burst` bytes per engine
		 * handshake — a 64-dword burst would wait for handshakes
		 * the engine doesn't fire and stall.
		 */
		/*
		 * burst = 4 dwords (16 B) for ALL ciphers, not per-cipher.
		 *
		 * Earlier this was 2 dwords (8 B) for DES/3DES on the theory
		 * that the DMA burst should match the cipher block size. But
		 * the engine signals CE_IN / CE_OUT CRCI at its internal FIFO
		 * granularity (16 B), independent of cipher block size.
		 * Setting burst = 8 B made adm_get_blksize() map to
		 * blk_size=1 (= 32 B CRCI handshake) — the engine then
		 * signalled every 16 B but ADM expected 32 B per handshake,
		 * exactly the same drift class that the CRCI-5 SDCC override
		 * bug caused for AES output. DES produced wrong / flaky
		 * output for cryptlen < 32 B and wedged the engine at large
		 * cryptlen. Samsung's vendor driver never switches burst
		 * per-cipher either; the ADM channel CRCI burst is set once
		 * at probe.
		 *
		 * Engine FIFO granularity is 16 B, so DES inputs that aren't
		 * a multiple of 16 must be padded to one before the engine
		 * runs (engine sees padded length; we copy back only the
		 * caller-visible length). For AES, cryptlen is always a
		 * multiple of the AES block (16 B) so the pad is a no-op.
		 */
		unsigned int burst = 4;
		unsigned int total = rctx->cryptlen;
		unsigned int eng_total = round_up(total, burst * 4);
		u32 seg_cfg = encr_cfg | BIT(CE2_FIRST_SHIFT) |
			      BIT(CE2_LAST_SHIFT);
		struct qce_ce2_bounce bounce;
		unsigned int bounce_sz;

		/*
		 * Cap cryptlen at the engine's 16-bit SEG_SIZE limit rounded
		 * down to a burst-aligned boundary (65535 → 65280). Returning
		 * -EMSGSIZE for larger inputs lets the skcipher / AF_ALG
		 * layer above split the request — same pattern as the hash
		 * path uses for inputs over SZ_1M.
		 */
		if (total > QCE_CE2_BOUNCE_MAX)
			return -EMSGSIZE;

		bounce_sz = round_up(total, burst * 4);
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

		/*
		 * Samsung vendor register order: ENCR_SEG_CFG -> SEG_CFG ->
		 * SEG_SIZE -> GOPROC.  CONFIG is NOT touched per op.
		 *
		 * Engine sees eng_total (pad-rounded), not the caller's
		 * cryptlen — the ADM transfers eng_total bytes and the
		 * engine must agree, or DOUT FIFO drifts vs ADM expectations
		 * and wedges.  Caller-visible cryptlen is restored when we
		 * sg_pcopy_from_buffer() only `total` bytes back into the
		 * user's dst sgl, and when the CBC IV-chain capture below
		 * indexes by `total` (not eng_total) so the IV for op N+1
		 * comes from the real last cipher block, not the pad's.
		 */
		writel(eng_total << CE2_ENCR_SEG_SIZE_SHIFT,
		       qce->base + CE2_REG_ENCR_SEG_CFG);
		writel(seg_cfg, qce->base + CE2_REG_SEG_CFG);
		writel(eng_total, qce->base + CE2_REG_SEG_SIZE);

		dev_info(qce->dev, "CE2 skc op len=%u eng=%u burst=%u seg_cfg_wrote=0x%08x encr_cfg=0x%08x\n",
			 total, eng_total, burst, seg_cfg, encr_cfg);
		dev_info(qce->dev,
			"CE2 pre-GOPROC readback: SEG_CFG=0x%08x ENCR_SEG_CFG=0x%08x SEG_SIZE=0x%08x CONFIG=0x%08x STATUS=0x%08x\n",
			readl(qce->base + CE2_REG_SEG_CFG),
			readl(qce->base + CE2_REG_ENCR_SEG_CFG),
			readl(qce->base + CE2_REG_SEG_SIZE),
			readl(qce->base + CE2_REG_CONFIG),
			readl(qce->base + CE2_REG_STATUS));
		{
			u32 rb[4];
			int kk;

			for (kk = 0; kk < 4; kk++)
				rb[kk] = readl_relaxed(qce->base +
						       CE2_REG_AES_RNDKEY0 + kk * 4);
			dev_info(qce->dev,
				 "CE2 key readback RNDKEY0..3: 0x%08x 0x%08x 0x%08x 0x%08x\n",
				 rb[0], rb[1], rb[2], rb[3]);
			if (!IS_ECB(flags) && enciv_words) {
				for (kk = 0; kk < (int)enciv_words; kk++)
					rb[kk] = readl_relaxed(qce->base +
							       CE2_REG_CNTR0_IV0 + kk * 4);
				dev_info(qce->dev,
					 "CE2 IV readback CNTR0..N (%u words): 0x%08x 0x%08x 0x%08x 0x%08x\n",
					 enciv_words, rb[0], rb[1],
					 enciv_words > 2 ? rb[2] : 0,
					 enciv_words > 3 ? rb[3] : 0);
			}
		}

		/*
		 * GOPROC fires inside qce_ce2_dma_inout_cipher via the
		 * pre_issue callback — after CPU prep (memcpy / byte-swap /
		 * descriptor submit) but before dma_async_issue_pending().
		 * That keeps the engine "kicked" only microseconds before
		 * ADM starts streaming DIN data.
		 */
		ret = qce_ce2_dma_inout_cipher(qce, req->src, req->dst,
					       0, total,
					       burst, &bounce,
					       qce_ce2_skc_fire_goproc,
					       NULL);
		/* inout_cipher's internal `padded = round_up(nbytes, burst_bytes)`
		 * matches our eng_total — both pad to 16 B. sg_pcopy in/out is
		 * sized to `nbytes` (= total) so the user sees only the
		 * caller-visible bytes; pad bytes encrypt and decrypt as
		 * throwaway.
		 */
		if (ret) {
			dev_err(qce->dev,
				"CE2 skc DMA failed: ret=%d STATUS=0x%08x\n",
				ret,
				readl_relaxed(qce->base + CE2_REG_STATUS));
			qce_ce2_free_bounce(qce, &bounce);
			return ret;
		}

		/*
		 * Post-GOPROC sanity: on healthy ops the engine has produced
		 * all output and DOUT_SIZE_AVAIL=0. A non-zero residue means
		 * the engine produced fewer dwords than ADM pulled, and ADM
		 * pulled stale DATA_SHADOW0 dwords into our output buffer —
		 * the signature of the FIFO-leak corruption that prior chunked
		 * iterations of this driver tried to mask with per-chunk
		 * resets. Fail the op explicitly so the caller sees -EIO
		 * instead of silently wrong ciphertext.
		 */
		{
			u32 st = readl_relaxed(qce->base + CE2_REG_STATUS);
			unsigned int avail = (st & CE2_DOUT_SIZE_AVAIL_MASK) >>
					     CE2_DOUT_SIZE_AVAIL_SHIFT;

			if (avail != 0) {
				dev_err_ratelimited(qce->dev,
					"CE2 skc engine no-op (DOUT_AVAIL=%u STATUS=0x%08x len=%u)\n",
					avail, st, total);
				qce_ce2_free_bounce(qce, &bounce);
				return -EIO;
			}
		}

		/*
		 * Capture next-op IV for chained-mode callers.
		 *
		 * For CBC: the IV for op N+1 is mathematically the last
		 * cipher block of op N — already sitting in bounce.dst_copy
		 * from the byte-swap step inside qce_ce2_dma_inout_cipher().
		 * Use that directly instead of reading the engine's CNTR0..3
		 * registers, which on this silicon update one cycle before
		 * STATUS goes IDLE and can return stale state.
		 *
		 * For CTR: the counter increments inside the engine per AES
		 * block and the post-op value is NOT derivable from
		 * ciphertext, so read CNTR0..3 back (with a short wait for
		 * the register-bank update window).
		 */
		if (!IS_ECB(flags) && enciv_words) {
			if (IS_CBC(flags)) {
				const u8 *p = bounce.dst_copy + total -
					      enciv_words * 4;
				for (k = 0; k < enciv_words; k++)
					enciv[k] = (__force __be32)
						get_unaligned_be32(p + k * 4);
			} else {
				for (timeout = 50; timeout > 0; timeout--) {
					status = readl_relaxed(qce->base +
							       CE2_REG_STATUS);
					if ((status & CE2_CRYPTO_STATE_MASK) == 0)
						break;
					udelay(10);
				}
				udelay(5);
				for (k = 0; k < enciv_words; k++)
					enciv[k] = (__force __be32)readl(
						qce->base + CE2_REG_CNTR0_IV0 +
						k * 4);
			}
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

	/* Wipe raw cipher key and IV bytes from the kernel stack —
	 * memzero_explicit survives compiler dead-store elimination so
	 * an attacker cannot read leftover key material from re-used
	 * stack frames. The expanded aes_ctx is wiped separately at
	 * the AES key-expansion site above.
	 *
	 * Error returns earlier in the function also leave (possibly
	 * partially-populated) key state on the stack, but they only
	 * fire on engine wedge/timeout — which already requires a
	 * reset before the next op — so the practical leak window is
	 * narrow. A follow-up that converts those early returns to
	 * `goto out;` will close that residual exposure.
	 */
	memzero_explicit(enckey, sizeof(enckey));
	memzero_explicit(enciv, sizeof(enciv));
	return 0;
}
#endif	/* CONFIG_CRYPTO_DEV_QCE_SKCIPHER */

int qce_check_status(struct qce_device *qce, u32 *status, bool wait_auth_done)
{
	int ret = 0;

	if (qce_is_ce2(qce)) {
		int timeout;

		/*
		 * CE2 (MSM8x60) STATUS bits transition one cycle before the
		 * engine's final write to its register bank commits — the
		 * same characteristic the cipher chunk loop already mitigates
		 * with poll-and-udelay for CTR-mode CNTR readbacks (see the
		 * comments at common.c:~2110 and the CBC dst_copy bypass at
		 * ~2165).
		 *
		 * The AEAD completion path is affected because the ADM/CRCI
		 * DMA controller signals descriptor completion as soon as it
		 * has moved the last byte, but AUTH_DONE in CE2_REG_STATUS
		 * can lag the DMA callback by tens of µs while the engine
		 * finalises the MAC. Without this poll the first STATUS read
		 * sees AUTH_DONE=0 and we mis-classify a perfectly successful
		 * AEAD operation as -ENXIO with a status word like 0x10200004
		 * (DOUT_SIZE_AVAIL=4, DIN_SIZE_AVAIL=1, DIN_RDY=1, every
		 * error bit clear) — observed on tenderloin every ~95 s under
		 * normal TLS/AEAD traffic.
		 *
		 * Bounded poll: 50 iterations × 10 µs upper bound, matching
		 * the existing pattern in the cipher chunk loop. In the
		 * common case AUTH_DONE is already asserted on the first
		 * read and the loop exits with zero added latency.
		 *
		 * The AUTH_DONE bit is literal — only hash and AEAD operations
		 * on CE2 ever set it. A pure skcipher op never sets AUTH_DONE;
		 * waiting for it on a cipher completion would always time out
		 * and return -ENXIO. Callers select the right completion
		 * condition via @wait_auth_done:
		 *
		 *   wait_auth_done = true  (hash, AEAD): poll AUTH_DONE
		 *   wait_auth_done = false (skcipher):   poll engine IDLE
		 *
		 * Today the CE2 cipher/hash async_req_handle entry points
		 * bypass this function (dispatching direct to
		 * qce_ce2_pio_run_{skcipher,hash} which do their own
		 * register polling), and AEAD is not registered on CE2 (see
		 * qce_aead_register), so this branch is currently unreached
		 * on CE2 in practice. The split below keeps it
		 * structurally-safe for any future caller that does route
		 * through here.
		 */
		for (timeout = 50; timeout > 0; timeout--) {
			*status = qce_read(qce, qce_reg_status(qce));
			if (*status & STATUS_ERRORS_CE2)
				return -ENXIO;
			if (wait_auth_done) {
				if (*status & BIT(CE2_AUTH_DONE_SHIFT))
					return 0;
			} else {
				if ((*status & CE2_CRYPTO_STATE_MASK) == 0)
					return 0;
			}
			udelay(10);
		}
		return -ENXIO;
	}

	*status = qce_read(qce, qce_reg_status(qce));

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
