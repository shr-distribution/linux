/*
 * legacy_adm_probe.ko - sample ADM CMD_PTR + chain N times during sustained dd
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>

#define ADM1_PHYS  0x18420000UL
#define ADM_SIZE   0x10000UL

#define ADM_CH_CONF(ch)     (0x240 + (ch) * 4)
#define ADM_CH_CMD_PTR(ch)  (0x800 + (ch) * 4)
#define ADM_CH_RSLT(ch)     (0x840 + (ch) * 4)
#define ADM_CH_STATUS(ch)   (0xA00 + (ch) * 4)
#define ADM_CH_RSLT_CONF(ch) (0xB00 + (ch) * 4)

#define CPLE_LP    (1U << 31)
#define CPLE_LIST  (1U << 29)
#define CMD_LC     (1U << 31)
#define CMD_TYPE_BOX 0x3

#define SAMPLES 30   /* sample 30 times across ~300 ms */
#define SAMPLE_MS 10

static void *adm_io;

struct chain_summary {
	u32 cmd_ptr;
	u32 first_box_bytes;
	int box_count;
	u32 total_bytes;
};

static void sample_chain(int ch, struct chain_summary *cs)
{
	u32 cmd_ptr = readl(adm_io + ADM_CH_CMD_PTR(ch));
	void *list_io, *desc_io;
	u32 list_phys;
	int i;

	cs->cmd_ptr = cmd_ptr;
	cs->first_box_bytes = 0;
	cs->box_count = 0;
	cs->total_bytes = 0;

	if (!cmd_ptr || !(cmd_ptr & CPLE_LIST))
		return;

	list_phys = (cmd_ptr & 0x1FFFFFFF) << 3;
	list_io = ioremap_nocache(list_phys, 256);
	if (!list_io)
		return;

	for (i = 0; i < 32; i++) {
		u32 e = readl(list_io + i * 4);
		u32 e_addr = (e & 0x1FFFFFFF) << 3;
		if (e_addr) {
			desc_io = ioremap_nocache(e_addr, 32);
			if (desc_io) {
				u32 dcmd = readl(desc_io + 0);
				u32 rl   = readl(desc_io + 12);
				u32 nr   = readl(desc_io + 16);
				if ((dcmd & 0x7) == CMD_TYPE_BOX) {
					u32 bytes = (rl & 0xFFFF) * (nr & 0xFFFF);
					if (cs->box_count == 0)
						cs->first_box_bytes = bytes;
					cs->box_count++;
					cs->total_bytes += bytes;
				}
				iounmap(desc_io);
			}
		}
		if (e & CPLE_LP)
			break;
	}
	iounmap(list_io);
}

static int __init legacy_adm_probe_init(void)
{
	int i;
	struct chain_summary cs2, cs5;
	u32 last_cmd2 = 0, last_cmd5 = 0;
	int distinct2 = 0, distinct5 = 0;

	adm_io = ioremap_nocache(ADM1_PHYS, ADM_SIZE);
	if (!adm_io) return -ENOMEM;

	pr_info("legacy_adm_probe: %d samples at %d ms interval\n",
		SAMPLES, SAMPLE_MS);
	pr_info("legacy_adm_probe: idx | ch2 cmd_ptr | boxes/total_bytes | ch5 cmd_ptr | boxes/total_bytes\n");
	for (i = 0; i < SAMPLES; i++) {
		sample_chain(2, &cs2);
		sample_chain(5, &cs5);
		if (cs2.cmd_ptr != last_cmd2 && cs2.cmd_ptr) { distinct2++; last_cmd2 = cs2.cmd_ptr; }
		if (cs5.cmd_ptr != last_cmd5 && cs5.cmd_ptr) { distinct5++; last_cmd5 = cs5.cmd_ptr; }
		pr_info("[%2d] ch2 0x%08x %dx %u B | ch5 0x%08x %dx %u B\n",
			i, cs2.cmd_ptr, cs2.box_count, cs2.total_bytes,
			cs5.cmd_ptr, cs5.box_count, cs5.total_bytes);
		mdelay(SAMPLE_MS);
	}
	pr_info("legacy_adm_probe: distinct ch2 cmd_ptr values seen = %d / %d samples\n",
		distinct2, SAMPLES);
	pr_info("legacy_adm_probe: distinct ch5 cmd_ptr values seen = %d / %d samples\n",
		distinct5, SAMPLES);

	return 0;
}

static void __exit legacy_adm_probe_exit(void)
{
	iounmap(adm_io);
}

module_init(legacy_adm_probe_init);
module_exit(legacy_adm_probe_exit);
MODULE_LICENSE("GPL");
