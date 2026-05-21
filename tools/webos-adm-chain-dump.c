/*
 * webos-adm-chain-dump — read the ADM descriptor chain via /dev/mem
 * and decode it.
 *
 * Run after webos-adm-sampler captured a non-zero CMD_PTR value.
 *
 * Usage:
 *   /tmp/webos-adm-chain-dump <hex-cmd-ptr>
 * e.g.
 *   /tmp/webos-adm-chain-dump 0x2fd57c60
 *
 * CMD_PTR encoding (legacy mach/dma.h):
 *   bit 31     = LP   (last pointer in chain)
 *   bit 29     = LIST (1: this is a CMD_PTR_LIST entry, points to a chain
 *                      of pointers; 0: this is a direct descriptor pointer)
 *   bits 28-0  = address >> 3
 *
 * Descriptor format (BOX):
 *   u32 cmd        // bit 31 = LC (last command), bits 7-10 = DST_CRCI,
 *                  // bits 3-6 = SRC_CRCI
 *   u32 src_addr
 *   u32 dst_addr
 *   u32 row_len
 *   u32 num_rows
 *   u32 row_offset
 *
 * Descriptor format (SINGLE):
 *   u32 cmd
 *   u32 src_addr
 *   u32 dst_addr
 *   u32 len
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>

#define PAGE_SIZE 4096UL
#define PAGE_MASK (~(PAGE_SIZE - 1))

#define CPLE_LP        (1U << 31)
#define CPLE_LIST      (1U << 29)
#define CMD_LC         (1U << 31)
#define CMD_TYPE_MASK  0x7
#define CMD_TYPE_SINGLE 0x0
#define CMD_MODE_SG    0x1
#define CMD_TYPE_BOX   0x3

static int memfd = -1;

static int read_phys(uint32_t phys, void *buf, size_t len)
{
	off_t page = phys & PAGE_MASK;
	size_t off = phys - page;
	size_t mapsize = ((off + len + PAGE_SIZE - 1) & PAGE_MASK);
	void *map = mmap(NULL, mapsize, PROT_READ, MAP_SHARED, memfd, page);
	if (map == MAP_FAILED) {
		fprintf(stderr, "mmap 0x%08x: %s\n", phys, strerror(errno));
		return -errno;
	}
	memcpy(buf, (char *)map + off, len);
	munmap(map, mapsize);
	return 0;
}

static void decode_cmd_type(uint32_t cmd)
{
	uint32_t t = cmd & CMD_TYPE_MASK;
	const char *name = "?";
	if (t == CMD_TYPE_SINGLE) name = "SINGLE";
	else if (t == CMD_MODE_SG) name = "SG";
	else if (t == CMD_TYPE_BOX) name = "BOX";
	printf("  cmd_type=%s%s",
	       name, (cmd & CMD_LC) ? " (LAST)" : "");
	printf(" src_crci=%u dst_crci=%u\n",
	       (cmd >> 3) & 0xf, (cmd >> 7) & 0xf);
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <hex-cmd-ptr-from-sampler>\n", argv[0]);
		return 1;
	}

	uint32_t cmd_ptr_raw = strtoul(argv[1], NULL, 0);
	int lp = !!(cmd_ptr_raw & CPLE_LP);
	int list_mode = !!(cmd_ptr_raw & CPLE_LIST);
	uint32_t list_phys = (cmd_ptr_raw & 0x1fffffff) << 3;
	uint32_t direct_phys = cmd_ptr_raw & ~(CPLE_LP | CPLE_LIST);

	printf("CMD_PTR raw  = 0x%08x\n", cmd_ptr_raw);
	printf("  LP   = %d (last in chain)\n", lp);
	printf("  LIST = %d (1=list of pointers, 0=direct descriptor)\n", list_mode);
	printf("  if LIST, list_phys (<<3 from low bits) = 0x%08x\n", list_phys);
	printf("  if not LIST, raw addr would be 0x%08x (suspicious if low bits set)\n", direct_phys);
	printf("\n");

	memfd = open("/dev/mem", O_RDONLY | O_SYNC);
	if (memfd < 0) { perror("open /dev/mem"); return 1; }

	/*
	 * If LIST mode: read a list of u32 entries from list_phys, each is
	 * itself a descriptor pointer (with LP+LIST flags).
	 * If not LIST mode: descriptor is at the address directly (with the
	 * legacy "addr >> 3" encoding — but we'll just try both).
	 */
	uint32_t base = list_mode ? list_phys : direct_phys;

	printf("=== Reading descriptor chain starting at 0x%08x ===\n", base);
	printf("(if LIST mode, this is the list of pointers)\n\n");

	if (list_mode) {
		/* Read up to 32 entries from the list */
		uint32_t list[32];
		if (read_phys(base, list, sizeof(list)) < 0) return 1;

		printf("List entries (each is a CPLE = u32 with flags):\n");
		int total_boxes = 0;
		uint64_t total_bytes = 0;
		for (int i = 0; i < 32; i++) {
			uint32_t e = list[i];
			int e_lp   = !!(e & CPLE_LP);
			int e_list = !!(e & CPLE_LIST);
			uint32_t e_addr = (e & 0x1fffffff) << 3;
			printf("  [%2d] = 0x%08x  LP=%d LIST=%d  desc_addr=0x%08x\n",
			       i, e, e_lp, e_list, e_addr);

			/* Read the BOX descriptor at e_addr */
			uint32_t desc[6];
			if (read_phys(e_addr, desc, sizeof(desc)) == 0) {
				printf("       cmd=0x%08x src=0x%08x dst=0x%08x\n",
				       desc[0], desc[1], desc[2]);
				printf("       row_len=%u num_rows=%u row_off=0x%08x  -> bytes=%u\n",
				       desc[3], desc[4], desc[5], desc[3] * desc[4]);
				decode_cmd_type(desc[0]);
				total_boxes++;
				total_bytes += (uint64_t)desc[3] * desc[4];
			}

			if (e_lp) {
				printf("  [%2d] -- LP set, end of list (after %d entries)\n",
				       i + 1, i + 1);
				break;
			}
		}
		printf("\n=== Chain summary ===\n");
		printf("Total BOX descriptors decoded: %d\n", total_boxes);
		printf("Total bytes covered: %lu (%.2f KB / %.2f MB)\n",
		       (unsigned long)total_bytes,
		       (double)total_bytes / 1024.0,
		       (double)total_bytes / (1024.0 * 1024.0));
		printf("Bytes per BOX: %lu\n",
		       total_boxes ? (unsigned long)total_bytes / total_boxes : 0);
	} else {
		/* Direct descriptor */
		uint32_t desc[6];
		if (read_phys(base, desc, sizeof(desc)) < 0) return 1;
		printf("Direct descriptor:\n");
		printf("  cmd=0x%08x src=0x%08x dst=0x%08x\n",
		       desc[0], desc[1], desc[2]);
		printf("  row_len=%u num_rows=%u row_off=0x%08x  -> bytes=%u\n",
		       desc[3], desc[4], desc[5], desc[3] * desc[4]);
		decode_cmd_type(desc[0]);
	}

	close(memfd);
	return 0;
}
