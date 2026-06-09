// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HP TouchPad token-partition nvmem provider.
 *
 * The TouchPad's "tokens" eMMC partition (mmcblk0p12 in the moboot layout)
 * holds the unique-per-device factory tokens: BToADDR, WIFIoADDR, ProductSN,
 * ProductSKU, HWoRev, etc.  webOS reads them as ASCII strings from
 * /dev/tokens/<name>; this driver parses the same format and re-exposes
 * each token requested via DT-declared cells.
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
 * Cell layout model:
 *
 *   Each DT child node of touchpad-tokens declares one cell.  The child
 *   node's name (e.g. "BToADDR") is matched against the token name in the
 *   partition.  The child's `reg = <offset length>` declares where the
 *   cell sits inside the synthetic nvmem address space exposed to consumers.
 *
 *   At probe we allocate a synthetic buffer sized to fit all the declared
 *   cells, then for each DT child find the matching token and copy (or
 *   parse) its data into the buffer at the declared offset.  reg_read is
 *   then a flat memcpy out of that buffer — no cell-info merge surprises,
 *   no offset/length mismatch between programmatic cells and DT cells.
 *
 *   Special case: a cell with declared length 6 against an ASCII token is
 *   treated as "XX:XX:XX:XX:XX:XX" and parsed into 6 raw LE bytes — the
 *   format the standard `local-bd-address` consumer binding expects.
 */
#include <linux/device.h>
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

#define TOKENS_PARTITION_MAX	(64 * 1024)

struct touchpad_token {
	char	name[TOKEN_NAME_SIZE + 1];
	u32	type;
	u32	length;
	const u8 *data;
};

struct touchpad_tokens_priv {
	struct device	*dev;
	struct nvmem_device *nvmem;

	void		*raw;
	size_t		raw_size;

	struct touchpad_token *tokens;
	unsigned int	num_tokens;

	/* Synthetic linear buffer the nvmem framework reads from.  Sized to
	 * cover the highest-addressed DT cell; reg_read is memcpy from here.
	 */
	u8		*synth;
	size_t		synth_size;
};

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

static int touchpad_tokens_parse(struct touchpad_tokens_priv *priv,
				 size_t tokens_off, size_t tokens_len)
{
	const u8 *base = priv->raw + tokens_off;
	const u8 *end  = base + tokens_len;
	const u8 *p    = base;
	unsigned int n = 0;

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

static struct touchpad_token *find_token(struct touchpad_tokens_priv *priv,
					 const char *name)
{
	unsigned int i;

	for (i = 0; i < priv->num_tokens; i++)
		if (!strcmp(priv->tokens[i].name, name))
			return &priv->tokens[i];
	return NULL;
}

/* "XX:XX:XX:XX:XX:XX" ASCII → 6 raw bytes in Bluetooth-LE order. */
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

	out[0] = v[5];
	out[1] = v[4];
	out[2] = v[3];
	out[3] = v[2];
	out[4] = v[1];
	out[5] = v[0];
	return 0;
}

/* Walk DT children, allocate a synthetic buffer big enough to cover them
 * all, and populate it.  Each child node's name picks the token; reg picks
 * where in the synthetic space it lands.
 */
static int touchpad_tokens_layout(struct touchpad_tokens_priv *priv,
				  struct device_node *np)
{
	struct device_node *child;
	size_t needed = 0;

	for_each_child_of_node(np, child) {
		u32 reg[2];

		if (of_property_read_u32_array(child, "reg", reg, 2))
			continue;
		if (reg[0] + reg[1] > needed)
			needed = reg[0] + reg[1];
	}
	if (!needed) {
		dev_err(priv->dev, "no DT cells declared — nothing to expose\n");
		return -EINVAL;
	}

	priv->synth_size = needed;
	priv->synth = devm_kzalloc(priv->dev, needed, GFP_KERNEL);
	if (!priv->synth)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		u32 reg[2];
		struct touchpad_token *t;
		const char *cell_name = child->name;

		if (of_property_read_u32_array(child, "reg", reg, 2))
			continue;

		t = find_token(priv, cell_name);
		if (!t) {
			dev_warn(priv->dev,
				 "cell '%s' has no matching token — leaving zero\n",
				 cell_name);
			continue;
		}

		/* 6-byte cell against an ASCII token: treat as "XX:XX:..." MAC
		 * and parse into 6 raw LE bytes — what `local-bd-address`
		 * consumers want.
		 */
		if (reg[1] == 6 && t->type == TOKEN_TYPE_ASCII) {
			u8 mac[6];

			if (!parse_macaddr_ascii_le((const char *)t->data,
						    t->length, mac)) {
				memcpy(priv->synth + reg[0], mac, 6);
				dev_info(priv->dev,
					 "cell '%s' parsed MAC %pM (LE bytes %*phN)\n",
					 cell_name, mac, 6, mac);
				continue;
			}
			dev_warn(priv->dev,
				 "cell '%s': MAC parse of '%.*s' failed, raw copy fallback\n",
				 cell_name, (int)min_t(size_t, t->length, 24),
				 t->data);
		}

		{
			size_t copy = min_t(size_t, reg[1], t->length);

			memcpy(priv->synth + reg[0], t->data, copy);
			dev_dbg(priv->dev, "cell '%s' raw copy %zu bytes\n",
				cell_name, copy);
		}
	}
	return 0;
}

static int touchpad_tokens_read(void *context, unsigned int offset,
				void *val, size_t bytes)
{
	struct touchpad_tokens_priv *priv = context;

	if (offset + bytes > priv->synth_size)
		return -EINVAL;
	memcpy(val, priv->synth + offset, bytes);
	return 0;
}

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

	err = touchpad_tokens_layout(priv, pdev->dev.of_node);
	if (err)
		return err;

	cfg.dev      = &pdev->dev;
	cfg.name     = "touchpad-tokens";
	cfg.size     = priv->synth_size;
	cfg.word_size = 1;
	cfg.stride   = 1;
	cfg.read_only = true;
	cfg.priv     = priv;
	cfg.reg_read = touchpad_tokens_read;
	cfg.add_legacy_fixed_of_cells = true;

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
