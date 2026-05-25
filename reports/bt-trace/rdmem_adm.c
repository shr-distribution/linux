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
	/* ADM1 CONF (0x240+ch*4) EE0 then EE1 : ch2=eMMC ch5=WiFi ch6=BTtx ch7=BTrx */
	0x18420248, 0x18420A48,   /* ch2 eMMC CONF EE0/EE1 */
	0x18420254, 0x18420A54,   /* ch5 WiFi CONF EE0/EE1 */
	0x18420258, 0x18420A58,   /* ch6 BT-TX CONF EE0/EE1 */
	0x1842025C, 0x18420A5C,   /* ch7 BT-RX CONF EE0/EE1 */
	/* ADM1 CMD_PTR (0x000+ch*4) ch2 eMMC EE0/EE1 (live transfer ptr) */
	0x18420008, 0x18420808,
	/* ADM1 CRCI_CTL (0x400+crci*4) EE0 then EE1 : crci1=eMMC 5=WiFi 8=BTtx 9=BTrx */
	0x18420404, 0x18420C04,   /* crci1 eMMC EE0/EE1 */
	0x18420414, 0x18420C14,   /* crci5 WiFi EE0/EE1 */
	0x18420420, 0x18420C20,   /* crci8 BT-TX EE0/EE1 */
	0x18420424, 0x18420C24,   /* crci9 BT-RX EE0/EE1 */
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
