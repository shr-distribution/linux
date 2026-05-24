/* Minimal static hciattach+hciconfig-up replacement for tenderloin BCSP.
 * Bypasses the bad-eMMC bluez binaries by running from tmpfs, no lib deps.
 *
 * Usage:
 *   btup attach <tty> [baud]   -> set raw <baud> (default 115200), RESET_ON_INIT
 *                                 flag, N_HCI ldisc, BCSP proto; hold fd open.
 *   btup attachup <tty> [baud] -> attach, then HCIDEVUP, then hold.
 *   btup up                    -> HCIDEVUP on hci0.
 *   btup break <tty> [ms]      -> hold UART_RX low <ms> (default 1200) to reset
 *                                 the CSR chip via PSKEY_HOSTIO_UART_RESET_TIMEOUT
 *                                 (BC63B239A datasheet 9.2), then exit.
 *   btup klog                  -> dump kernel ring buffer.
 *
 * baud may be ANY value (termios2/BOTHER), e.g. 113000 114000 115200 116000
 * 117000 for the CSR ">=1% RX tolerance" sweep (datasheet Table 9.2): our SoC
 * RX is lenient enough to keep decoding the chip's 115200 SYNC across the
 * sweep, so a TX rate that lands inside the chip's tight +/-1% window should
 * finally get a SYNC_RSP back.
 *
 * BT_WAKE: asserts gpio643 (TLMM base 512 + BT_WAKE 131) high before touching
 * the tty. Override with env BT_WAKE_GPIO=<n>, or BT_WAKE_GPIO=0 to skip.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/klog.h>

#define N_HCI            15
#define HCIUARTSETPROTO  _IOW('U', 200, int)
#define HCIUARTSETFLAGS  _IOW('U', 203, int)
#define HCI_UART_BCSP    1
#define HCI_UART_RESET_ON_INIT_BIT 1

#define AF_BLUETOOTH 31
#define BTPROTO_HCI  1
#define HCIDEVUP   _IOW('H', 201, int)
#define HCIDEVDOWN _IOW('H', 202, int)

/*
 * termios2 / BOTHER for arbitrary baud. Defined locally to avoid the
 * <asm/termbits.h> vs <termios.h> conflict. Generic-termbits values (ARM).
 */
#ifndef BOTHER
#define BOTHER 0010000
#endif
#ifndef CBAUD
#define CBAUD 0010017
#endif
#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif
struct termios2 {
	tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
	cc_t c_line;
	cc_t c_cc[19];
	speed_t c_ispeed;
	speed_t c_ospeed;
};

/* Assert BT_WAKE (chip PIO[1], host->chip wake). Non-fatal. */
static void bt_wake_assert(void)
{
	const char *env = getenv("BT_WAKE_GPIO");
	int gpio = env ? atoi(env) : 643;
	char path[64]; int f; char buf[8]; int len;
	if (gpio <= 0) return;
	f = open("/sys/class/gpio/export", O_WRONLY);
	if (f >= 0) { len = snprintf(buf,sizeof buf,"%d",gpio); write(f,buf,len); close(f); }
	snprintf(path,sizeof path,"/sys/class/gpio/gpio%d/direction",gpio);
	f = open(path, O_WRONLY);
	if (f < 0) { printf("BT_WAKE gpio%d: open failed (%s) — assert by hand\n", gpio, strerror(errno)); return; }
	write(f,"out",3); close(f);
	snprintf(path,sizeof path,"/sys/class/gpio/gpio%d/value",gpio);
	f = open(path, O_WRONLY);
	if (f >= 0) { write(f,"1",1); close(f); printf("BT_WAKE gpio%d asserted HIGH\n", gpio); }
}

/* Raw 8N1, no flow control, exact (possibly non-standard) baud. */
static int set_raw(int fd, unsigned int baud)
{
	struct termios ti;
	struct termios2 t2;

	if (tcgetattr(fd, &ti) < 0) return -1;
	cfmakeraw(&ti);
	ti.c_cflag |= CLOCAL | CREAD;
	ti.c_cflag &= ~CRTSCTS;        /* no hardware flow control (webOS) */
	cfsetospeed(&ti, B115200);     /* placeholder; exact baud set below */
	cfsetispeed(&ti, B115200);
	tcflush(fd, TCIOFLUSH);
	if (tcsetattr(fd, TCSANOW, &ti) < 0) return -1;

	/* Apply exact baud via TCSETS2 + BOTHER (handles 113000, 116000, ...). */
	if (ioctl(fd, TCGETS2, &t2) == 0) {
		t2.c_cflag &= ~CBAUD;
		t2.c_cflag |= BOTHER;
		t2.c_ospeed = baud;
		t2.c_ispeed = baud;
		if (ioctl(fd, TCSETS2, &t2) < 0)
			perror("TCSETS2 (custom baud, non-fatal)");
	} else {
		perror("TCGETS2 (non-fatal)");
	}
	tcflush(fd, TCIOFLUSH);
	printf("set_raw: requested baud=%u (8N1, no flow)\n", baud);
	fflush(stdout);
	return 0;
}

static void assert_rts(int fd)
{
	int mbits = 0;
	ioctl(fd, TIOCMGET, &mbits);
	printf("modem before RTS-assert = 0x%x\n", mbits);
	mbits = TIOCM_RTS;
	if (ioctl(fd, TIOCMBIS, &mbits) < 0)
		perror("TIOCMBIS RTS");
	mbits = 0;
	ioctl(fd, TIOCMGET, &mbits);
	printf("modem after  RTS-assert = 0x%x\n", mbits);
}

static int do_attach(const char *tty, unsigned int baud)
{
	int fd, ld = N_HCI, proto = HCI_UART_BCSP;
	unsigned long flags = (1UL << HCI_UART_RESET_ON_INIT_BIT);
	bt_wake_assert();
	fd = open(tty, O_RDWR | O_NOCTTY);
	if (fd < 0) { perror("open tty"); return -1; }
	if (set_raw(fd, baud) < 0) { perror("termios"); return -1; }
	if (ioctl(fd, TIOCSETD, &ld) < 0) { perror("TIOCSETD N_HCI"); return -1; }
	if (ioctl(fd, HCIUARTSETFLAGS, flags) < 0)
		perror("HCIUARTSETFLAGS (non-fatal)");
	if (ioctl(fd, HCIUARTSETPROTO, proto) < 0) { perror("HCIUARTSETPROTO"); return -1; }
	assert_rts(fd);
	printf("attach: BCSP ldisc on %s, baud=%u (RESET_ON_INIT, RTS asserted)\n", tty, baud);
	fflush(stdout);
	return fd;   /* caller must keep this open */
}

/* Long UART break: hold TX low > PSKEY_HOSTIO_UART_RESET_TIMEOUT to reset the
 * CSR chip to a known state (BC63B239A datasheet 9.2, Figure 9.2). */
static int do_break(const char *tty, unsigned int baud, int ms)
{
	int fd;
	bt_wake_assert();
	fd = open(tty, O_RDWR | O_NOCTTY);
	if (fd < 0) { perror("open tty"); return -1; }
	set_raw(fd, baud);
	assert_rts(fd);
	printf("break: holding UART_RX low %d ms to reset the chip...\n", ms);
	fflush(stdout);
	if (ioctl(fd, TIOCSBRK) < 0) perror("TIOCSBRK");
	usleep((useconds_t)ms * 1000);
	if (ioctl(fd, TIOCCBRK) < 0) perror("TIOCCBRK");
	tcflush(fd, TCIOFLUSH);
	printf("break: released; chip should re-init and stream fresh SYNC\n");
	fflush(stdout);
	close(fd);
	return 0;
}

static int do_up(void)
{
	int s = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
	if (s < 0) { perror("socket AF_BLUETOOTH"); return -1; }
	if (ioctl(s, HCIDEVUP, 0) < 0) {
		printf("HCIDEVUP hci0: %s (errno %d)\n", strerror(errno), errno);
		fflush(stdout);
		close(s);
		return -1;
	}
	printf("HCIDEVUP hci0: OK\n");
	fflush(stdout);
	close(s);
	return 0;
}

/* Dump the kernel ring buffer via klogctl (no eMMC binary needed). */
static int do_klog(void)
{
	static char buf[256 * 1024];
	int n = klogctl(3 /* SYSLOG_ACTION_READ_ALL */, buf, sizeof(buf) - 1);
	if (n < 0) { perror("klogctl"); return -1; }
	buf[n] = 0;
	fwrite(buf, 1, n, stdout);
	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: btup attach|attachup <tty> [baud] | break <tty> [ms] | up | klog\n");
		return 2;
	}

	if (!strcmp(argv[1], "klog"))
		return do_klog() == 0 ? 0 : 1;

	if (!strcmp(argv[1], "up"))
		return do_up() == 0 ? 0 : 1;

	if (argc < 3) { fprintf(stderr, "need <tty>\n"); return 2; }

	if (!strcmp(argv[1], "break")) {
		int ms = (argc > 3) ? atoi(argv[3]) : 1200;
		return do_break(argv[2], 115200, ms) == 0 ? 0 : 1;
	}

	{
		unsigned int baud = (argc > 3) ? (unsigned int)strtoul(argv[3], NULL, 0) : 115200;
		int fd = do_attach(argv[2], baud);
		if (fd < 0) return 1;

		if (!strcmp(argv[1], "attachup")) {
			sleep(2);          /* let BCSP link establish */
			do_up();
		}

		/* hold the ldisc open until killed */
		for (;;) pause();
	}
	return 0;
}
