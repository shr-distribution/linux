// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HP TouchPad token-partition nvmem provider.
 *
 * The TouchPad's "tokens" eMMC partition (mmcblk0p12 in the moboot layout)
 * holds the unique-per-device factory tokens: BToADDR, WIFIoADDR, ProductSN,
 * ProductSKU, HWoRev, etc.  webOS reads them as ASCII strings from
 * /dev/tokens/<name>; this driver parses the same format and re-exposes
 * each token as an nvmem cell that mainline drivers can consume via the
 * standard nvmem-cells DT binding.
 *
 * Storage layout:
 *
 *   offset 0x0000: NVRM magic + flags
 *   offset 0x0020..0x4FFF: TOC1 entries (32 bytes each: TOC1 magic,
 *                          offset, size, flags, name)
 *   offset 0x5000+:        tokens region — series of TOKN TLV records:
 *
 *                          0x00 'TOKN' (4 bytes)
 *                          0x04  u32  version (1)
 *                          0x08  u32  data length (bytes)
 *                          0x0C  u32  type (1=binary, 2=ASCII)
 *                          0x10  u32  hash/CRC
 *                          0x14  char name[16] (null-padded)
 *                          0x24  u8   data[length]
 *
 * BToADDR specifically is stored as ASCII "00:1D:FE:85:64:A9".  Cells
 * declared with `read-post-process = "macaddr"` get the ASCII parsed
 * into 6 raw bytes (Bluetooth LE order: LSB first).
 *
 * The driver waits for the backing block device to be ready (typically
 * deferring probe until the mmc host has enumerated mmcblk0) then reads
 * the partition once and caches the parsed token data.
 */
#include <linux/blk_types.h>
#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

#define NVRM_MAGIC		"NVRM"
#define TOC_MAGIC		"TOC1"
#define TOKEN_MAGIC		"TOKN"
#define TOC_ENTRY_SIZE		32
#define TOKEN_HEADER_SIZE	36
#define TOKEN_NAME_SIZE		16
#define TOKEN_TYPE_BINARY	1
#define TOKEN_TYPE_ASCII	2

/* Max raw partition bytes we keep in RAM for parsing.  TouchPad's tokens
 * region is ~10 KB; cap at 64 KB defensively.
 */
#define TOKENS_PARTITION_MAX	(64 * 1024)

struct touchpad_token {
	char	name[TOKEN_NAME_SIZE + 1];
	u32	type;
	u32	length;
	const u8 *data;		/* pointer into priv->raw */
};

struct touchpad_tokens_priv {
	struct device	*dev;
	struct nvmem_device *nvmem;
	void		*raw;		/* cached partition bytes */
	size_t		raw_size;

	/* Each cell is given a linear synthetic offset into the nvmem space;
	 * the driver translates that back to its underlying token data via
	 * this offset table.  Cell 0 starts at offset 0, cell 1 starts at
	 * offset cell0.len, etc.
	 */
	struct touchpad_token *tokens;
	unsigned int	num_tokens;

	struct nvmem_cell_info *cell_info;
};

/* Parse the NVRM TOC and find the token-region offset+size. */
static int touchpad_tokens_find_region(struct touchpad_tokens_priv *priv,
				       size_t *out_off, size_t *out_len)
{
	const u8 *p = priv->raw;
	size_t i;

	if (priv->raw_size < 0x20)
		return -EINVAL;
	if (memcmp(p, NVRM_MAGIC, 4))
		return -EINVAL;

	for (i = 0x20; i + TOC_ENTRY_SIZE <= priv->raw_size; i += TOC_ENTRY_SIZE) {
		const u8 *e = p + i;
		u32 off, sz;
		const char *name;

		if (memcmp(e, TOC_MAGIC, 4))
			continue;
		off  = get_unaligned_le32(e + 4);
		sz   = get_unaligned_le32(e + 8);
		name = (const char *)(e + 16);

		if (!strncmp(name, "tokens", 6)) {
			if (off + sz > priv->raw_size)
				return -EINVAL;
			*out_off = off;
			*out_len = sz;
			return 0;
		}
	}
	return -ENOENT;
}

/* Walk the tokens region and stash each TOKN record's metadata. */
static int touchpad_tokens_parse(struct touchpad_tokens_priv *priv,
				 size_t tokens_off, size_t tokens_len)
{
	const u8 *base = priv->raw + tokens_off;
	const u8 *end  = base + tokens_len;
	const u8 *p    = base;
	unsigned int n = 0;

	/* First pass: count valid records so we can kcalloc the array. */
	while (p + TOKEN_HEADER_SIZE <= end) {
		u32 len;

		if (memcmp(p, TOKEN_MAGIC, 4))
			break;
		len = get_unaligned_le32(p + 8);
		if (p + TOKEN_HEADER_SIZE + len > end)
			break;
		n++;
		p += TOKEN_HEADER_SIZE + round_up(len, 4);
	}
	if (!n)
		return -ENOENT;

	priv->tokens = devm_kcalloc(priv->dev, n, sizeof(*priv->tokens),
				    GFP_KERNEL);
	if (!priv->tokens)
		return -ENOMEM;

	/* Second pass: fill in. */
	p = base;
	for (priv->num_tokens = 0; priv->num_tokens < n; priv->num_tokens++) {
		struct touchpad_token *t = &priv->tokens[priv->num_tokens];
		u32 len = get_unaligned_le32(p + 8);

		t->type   = get_unaligned_le32(p + 12);
		t->length = len;
		t->data   = p + TOKEN_HEADER_SIZE;
		memcpy(t->name, p + 20, TOKEN_NAME_SIZE);
		t->name[TOKEN_NAME_SIZE] = '\0';
		p += TOKEN_HEADER_SIZE + round_up(len, 4);
	}

	dev_info(priv->dev, "parsed %u tokens\n", priv->num_tokens);
	return 0;
}

/* Convert "XX:XX:XX:XX:XX:XX" ASCII (NUL-terminated, possibly with
 * trailing junk) into a 6-byte raw MAC in Bluetooth LE order (LSB first
 * = standard `local-bd-address` cell format).
 */
static int parse_macaddr_ascii_le(const char *ascii, size_t len, u8 *out)
{
	unsigned int v[6];
	int r;

	if (len < 17)
		return -EINVAL;
	r = sscanf(ascii, "%02x:%02x:%02x:%02x:%02x:%02x",
		   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
	if (r != 6)
		return -EINVAL;

	/* Bluetooth `local-bd-address` cell expects LE order. */
	out[0] = v[5];
	out[1] = v[4];
	out[2] = v[3];
	out[3] = v[2];
	out[4] = v[1];
	out[5] = v[0];
	return 0;
}

/* nvmem-cell `read-post-process = "macaddr"` callback. */
static int touchpad_tokens_post_process_macaddr(void *context, const char *id,
						int index, unsigned int offset,
						void *buf, size_t bytes)
{
	struct touchpad_tokens_priv *priv = context;
	unsigned int i;

	if (bytes != 6)
		return -EINVAL;
	/* The cell's underlying token must have come back as ASCII; find it
	 * by name (cell name == token name) and parse.
	 */
	for (i = 0; i < priv->num_tokens; i++) {
		if (strcmp(priv->tokens[i].name, id))
			continue;
		if (priv->tokens[i].type != TOKEN_TYPE_ASCII)
			return -EINVAL;
		return parse_macaddr_ascii_le((const char *)priv->tokens[i].data,
					      priv->tokens[i].length, buf);
	}
	return -ENOENT;
}

/* reg_read for the synthetic flat nvmem space: each token gets a fixed
 * contiguous range starting at the offset we recorded in cell_info.
 */
static int touchpad_tokens_read(void *context, unsigned int offset,
				void *val, size_t bytes)
{
	struct touchpad_tokens_priv *priv = context;
	struct nvmem_cell_info *ci;
	unsigned int i;

	for (i = 0; i < priv->num_tokens; i++) {
		ci = &priv->cell_info[i];
		if (offset >= ci->offset && offset < ci->offset + ci->bytes) {
			unsigned int in_cell = offset - ci->offset;
			unsigned int copy = min_t(unsigned int, bytes,
						  ci->bytes - in_cell);

			if (priv->tokens[i].length < in_cell + copy)
				return -EINVAL;
			memcpy(val, priv->tokens[i].data + in_cell, copy);
			return 0;
		}
	}
	return -EINVAL;
}

/* Read the backing block device into priv->raw.  Uses kernel-side VFS
 * filp_open so we don't have to invent a custom block-dev API. Defers
 * probe if the device isn't enumerated yet (e.g., mmc host probe hasn't
 * completed).
 */
static int touchpad_tokens_load_storage(struct touchpad_tokens_priv *priv,
					const char *path)
{
	struct file *fp;
	loff_t pos = 0;
	ssize_t r;

	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		long e = PTR_ERR(fp);

		if (e == -ENOENT)
			return -EPROBE_DEFER;
		return e;
	}

	priv->raw_size = TOKENS_PARTITION_MAX;
	priv->raw = devm_kzalloc(priv->dev, priv->raw_size, GFP_KERNEL);
	if (!priv->raw) {
		filp_close(fp, NULL);
		return -ENOMEM;
	}

	r = kernel_read(fp, priv->raw, priv->raw_size, &pos);
	filp_close(fp, NULL);
	if (r < 0)
		return r;
	priv->raw_size = r;
	dev_info(priv->dev, "read %zu bytes from %s\n", priv->raw_size, path);
	return 0;
}

static int touchpad_tokens_probe(struct platform_device *pdev)
{
	struct touchpad_tokens_priv *priv;
	struct nvmem_config cfg = {};
	const char *path;
	size_t tokens_off, tokens_len;
	unsigned int i, total = 0;
	int err;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;

	err = device_property_read_string(&pdev->dev, "storage-device-path",
					  &path);
	if (err) {
		dev_err(&pdev->dev, "missing storage-device-path DT property\n");
		return err;
	}

	err = touchpad_tokens_load_storage(priv, path);
	if (err)
		return err;

	err = touchpad_tokens_find_region(priv, &tokens_off, &tokens_len);
	if (err) {
		dev_err(&pdev->dev, "no 'tokens' TOC entry found\n");
		return err;
	}

	err = touchpad_tokens_parse(priv, tokens_off, tokens_len);
	if (err)
		return err;

	/* Build cell_info array.  Token names become cell names; each token
	 * gets a contiguous offset range.  Consumers reference cells by
	 * <name> via the nvmem-cell-names DT binding.
	 */
	priv->cell_info = devm_kcalloc(&pdev->dev, priv->num_tokens,
				       sizeof(*priv->cell_info), GFP_KERNEL);
	if (!priv->cell_info)
		return -ENOMEM;
	for (i = 0; i < priv->num_tokens; i++) {
		priv->cell_info[i].name   = priv->tokens[i].name;
		priv->cell_info[i].offset = total;
		priv->cell_info[i].bytes  = priv->tokens[i].length;
		/* For ASCII MAC tokens, attach the macaddr post-processor —
		 * cells declared with `nvmem-cell-names = "local-bd-address";`
		 * pick this up automatically per the standard binding.
		 */
		if (priv->tokens[i].type == TOKEN_TYPE_ASCII &&
		    (!strcmp(priv->tokens[i].name, "BToADDR") ||
		     !strcmp(priv->tokens[i].name, "WIFIoADDR"))) {
			priv->cell_info[i].read_post_process =
				touchpad_tokens_post_process_macaddr;
			/* Override exposed length: post-processor produces 6
			 * raw bytes regardless of the ASCII source length.
			 */
			priv->cell_info[i].bytes = 6;
		}
		total += priv->cell_info[i].bytes;
	}

	cfg.dev      = &pdev->dev;
	cfg.name     = "touchpad-tokens";
	cfg.size     = total;
	cfg.word_size = 1;
	cfg.stride   = 1;
	cfg.read_only = true;
	cfg.priv     = priv;
	cfg.reg_read = touchpad_tokens_read;
	cfg.cells    = priv->cell_info;
	cfg.ncells   = priv->num_tokens;

	priv->nvmem = devm_nvmem_register(&pdev->dev, &cfg);
	if (IS_ERR(priv->nvmem))
		return PTR_ERR(priv->nvmem);

	platform_set_drvdata(pdev, priv);
	return 0;
}

static const struct of_device_id touchpad_tokens_of_match[] = {
	{ .compatible = "hp,touchpad-nvrm-tokens" },
	{}
};
MODULE_DEVICE_TABLE(of, touchpad_tokens_of_match);

static struct platform_driver touchpad_tokens_driver = {
	.driver = {
		.name = "touchpad-tokens",
		.of_match_table = touchpad_tokens_of_match,
	},
	.probe = touchpad_tokens_probe,
};
module_platform_driver(touchpad_tokens_driver);

MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_DESCRIPTION("HP TouchPad NVRM/TOC tokens partition nvmem provider");
MODULE_LICENSE("GPL");
