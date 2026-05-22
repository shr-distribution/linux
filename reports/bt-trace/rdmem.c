/* Freestanding ARM /dev/mem register dumper (no libc -> runs on 2.6.35).
 * mmap()s each 4K page (read() of MMIO fails on /dev/mem, mmap works) and
 * prints "addr=value" for a fixed list of physical addresses.
 * Build: arm-linux-gnueabihf-gcc -nostdlib -static -O2 -marm -o rdmem rdmem.c
 */

/* ARM EABI syscall: number in r7, args r0..r5, ret in r0 */
static long sc(long n, long a, long b, long c, long d, long e, long f)
{
	register long r7 __asm__("r7") = n;
	register long r0 __asm__("r0") = a;
	register long r1 __asm__("r1") = b;
	register long r2 __asm__("r2") = c;
	register long r3 __asm__("r3") = d;
	register long r4 __asm__("r4") = e;
	register long r5 __asm__("r5") = f;
	__asm__ volatile("svc 0"
			 : "+r"(r0)
			 : "r"(r7), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5)
			 : "memory");
	return r0;
}

#define SYS_exit   1
#define SYS_write  4
#define SYS_open   5
#define SYS_close  6
#define SYS_mmap2  192

static int slen(const char *s){ int n=0; while(s[n]) n++; return n; }
static void wr(const char *s, int n){ sc(SYS_write,1,(long)s,n,0,0,0); }
static void wrs(const char *s){ wr(s, slen(s)); }

static void wrhex(unsigned long v)
{
	char buf[11]; const char *h="0123456789abcdef";
	int i; buf[0]='0'; buf[1]='x';
	for(i=0;i<8;i++) buf[2+i]=h[(v>>((7-i)*4))&0xf];
	buf[10]='\n'; wr(buf,11);
}

static const unsigned long addrs[] = {
	0x16540000, 0x16540004, 0x16540008, 0x16540010, 0x16540014, /* UART MR1/MR2/SR/CR/IMR */
	0x801824, 0x801834, 0x8018a8, 0x801814,                     /* GPIO 130/131/138/129 io */
	0x801350, 0x801360, 0x801370, 0x801380,                     /* GPIO 53/54/55/56 ctl */
};

void run(void)
{
	int fd = sc(SYS_open, (long)"/dev/mem", 2 /*O_RDWR*/, 0, 0,0,0);
	unsigned i;
	if (fd < 0) { wrs("open /dev/mem failed\n"); sc(SYS_exit,1,0,0,0,0,0); }
	for (i = 0; i < sizeof(addrs)/sizeof(addrs[0]); i++) {
		unsigned long a = addrs[i];
		unsigned long page = a & ~0xfffUL;
		long m = sc(SYS_mmap2, 0, 4096, 1 /*PROT_READ*/, 1 /*MAP_SHARED*/,
			    fd, page >> 12);
		wrhex(a);
		if (m < 0 && m > -4096) {
			wrs("  mmap failed\n");
			continue;
		}
		{
			volatile unsigned long *p =
				(volatile unsigned long *)(m + (a & 0xfff));
			wrs("  = ");
			wrhex(*p);
		}
	}
	sc(SYS_close, fd, 0,0,0,0,0);
	sc(SYS_exit, 0, 0,0,0,0,0);
}

void _start(void){ run(); }
