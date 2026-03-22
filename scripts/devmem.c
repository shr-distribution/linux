/*
 * Simple devmem tool for reading physical memory
 * Cross-compile: arm-linux-gnueabihf-gcc -static -o devmem devmem.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

int main(int argc, char *argv[])
{
    int fd;
    void *map;
    uint32_t addr, value;
    off_t page_base;
    size_t page_offset;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <address> [value]\n", argv[0]);
        fprintf(stderr, "  Read:  %s 0x04000104\n", argv[0]);
        fprintf(stderr, "  Write: %s 0x04000104 0x1234\n", argv[0]);
        return 1;
    }

    addr = strtoul(argv[1], NULL, 0);
    page_base = addr & ~0xFFF;
    page_offset = addr & 0xFFF;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    if (map == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    if (argc > 2) {
        /* Write */
        value = strtoul(argv[2], NULL, 0);
        *(volatile uint32_t *)(map + page_offset) = value;
        printf("Wrote 0x%08X to 0x%08X\n", value, addr);
    } else {
        /* Read */
        value = *(volatile uint32_t *)(map + page_offset);
        printf("0x%08X\n", value);
    }

    munmap(map, 4096);
    close(fd);
    return 0;
}
