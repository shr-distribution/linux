/* Freestanding ARM /dev/mem dumper with BT-specific addresses. */

static long sc(long n, long a, long b, long c, long d, long e, long f)
{
	register long r7 __asm__("r7") = n;
	register long r0 __asm__("r0") = a;
	register long r1 __asm__("r1") = b;
	register long r2 __asm__("r2") = c;
	register long r3 __asm__("r3") = d;
	register long r4 __asm__("r4") = e;
	register long r5 __asm__("r5") = f;
	__asm__ volatile("svc 0" : "+r"(r0) : "r"(r7), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5) : "memory");
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

struct entry { unsigned long addr; const char *name; };

static const struct entry addrs[] = {
	{ 0x00902A70, "UART_APPS6_MD" },
	{ 0x00902A74, "UART_APPS6_NS" },
	{ 0x16500000, "GSBI6_CTRL" },
	{ 0x16540000, "UART_MR1" },
	{ 0x16540004, "UART_MR2" },
	{ 0x16540008, "UART_SR (rd) / CSR (wr)" },
	{ 0x16540014, "UART_IMR" },
	{ 0x16540018, "UART_IPR" },
	{ 0x1654001C, "UART_TFWR" },
	{ 0x16540020, "UART_RFWR" },
	{ 0x1654003C, "UART_DMEN" },
	{ 0x16540040, "UART_NCF_TX" },
	{ 0x1654006C, "UART_RXFS" },
	/* GPIO_CFG[gpio]: TLMM base 0x801000 + 0x10*gpio. */
	{ 0x801330, "GPIO51_CFG" },
	{ 0x801340, "GPIO52_CFG" },
	{ 0x801350, "GPIO53_CFG (UART RFR)" },
	{ 0x801360, "GPIO54_CFG (UART CTS)" },
	{ 0x801370, "GPIO55_CFG (UART RX)" },
	{ 0x801380, "GPIO56_CFG (UART TX)" },
	{ 0x801810, "GPIO129_CFG" },
	{ 0x801820, "GPIO130_CFG (?BT)" },
	{ 0x801830, "GPIO131_CFG (BT_WAKE)" },
	{ 0x801840, "GPIO132_CFG (?BT)" },
	/* GPIO_IN/OUT regs at 0x800000 base, packed 32 gpio per word:
	 * GPIO_IN_OUT[g] = 0x800004 + 4*(g/32), bit (g%32). */
	{ 0x800010, "GPIO_IN_OUT[128-159]" },
	/* TCSR ADM CRCI mux. */
	{ 0x16b00078, "TCSR_ADM1_CRCI_MUX_A" },
	{ 0x16b0007c, "TCSR_ADM1_CRCI_MUX_B" },
};

void run(void)
{
	int fd = sc(SYS_open, (long)"/dev/mem", 2, 0, 0,0,0);
	unsigned i;
	if (fd < 0) { wrs("open /dev/mem failed\n"); sc(SYS_exit,1,0,0,0,0,0); }
	for (i = 0; i < sizeof(addrs)/sizeof(addrs[0]); i++) {
		unsigned long a = addrs[i].addr;
		unsigned long page = a & ~0xfffUL;
		long m = sc(SYS_mmap2, 0, 4096, 1, 1, fd, page >> 12);
		wrhex(a);
		wrs("  ");
		wrs(addrs[i].name);
		wrs(" = ");
		if (m < 0 && m > -4096) { wrs("MMAP_FAIL\n"); continue; }
		{
			volatile unsigned long *p = (volatile unsigned long *)(m + (a & 0xfff));
			wrhex(*p);
		}
	}
	sc(SYS_close, fd, 0,0,0,0,0);
	sc(SYS_exit, 0, 0,0,0,0,0);
}

void _start(void){ run(); }
