/* Minimal static hciattach+hciconfig-up replacement for tenderloin BCSP.
 * Bypasses the bad-eMMC bluez binaries by running from tmpfs, no lib deps.
 *
 * Usage:
 *   btup attach <tty>     -> set raw 115200, RESET_ON_INIT flag, N_HCI ldisc,
 *                            BCSP proto; then hold the fd open (ldisc stays
 *                            attached) until killed.
 *   btup up               -> HCIDEVUP on hci0 (triggers hci_dev_open ->
 *                            bcsp_setup -> HCI init w/ Reset-first).
 *   btup attachup <tty>   -> attach, then HCIDEVUP, then hold.
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

static int set_raw_115200(int fd)
{
	struct termios ti;
	if (tcgetattr(fd, &ti) < 0) return -1;
	cfmakeraw(&ti);
	ti.c_cflag |= CLOCAL | CREAD;
	ti.c_cflag &= ~CRTSCTS;        /* no hardware flow control (webOS) */
	cfsetospeed(&ti, B115200);
	cfsetispeed(&ti, B115200);
	tcflush(fd, TCIOFLUSH);
	if (tcsetattr(fd, TCSANOW, &ti) < 0) return -1;
	tcflush(fd, TCIOFLUSH);
	return 0;
}

static int do_attach(const char *tty)
{
	int fd = open(tty, O_RDWR | O_NOCTTY);
	int ld = N_HCI, proto = HCI_UART_BCSP;
	unsigned long flags = (1UL << HCI_UART_RESET_ON_INIT_BIT);
	if (fd < 0) { perror("open tty"); return -1; }
	if (set_raw_115200(fd) < 0) { perror("termios"); return -1; }
	if (ioctl(fd, TIOCSETD, &ld) < 0) { perror("TIOCSETD N_HCI"); return -1; }
	if (ioctl(fd, HCIUARTSETFLAGS, flags) < 0)
		perror("HCIUARTSETFLAGS (non-fatal)");
	if (ioctl(fd, HCIUARTSETPROTO, proto) < 0) { perror("HCIUARTSETPROTO"); return -1; }
	/*
	 * Assert RTS (RFR/gpio56) — webOS drives it low (asserted) during the
	 * BCSP handshake; RFR is the chip's CTS input and the chip won't send
	 * CONF/SYNC-RSP unless it sees CTS asserted.
	 */
	{
		int mbits = 0;
		ioctl(fd, 0x5415 /*TIOCMGET*/, &mbits);
		printf("attach: modem before RTS-assert = 0x%x\n", mbits);
		mbits = 0x004; /* TIOCM_RTS */
		if (ioctl(fd, 0x5416 /*TIOCMBIS*/, &mbits) < 0)
			perror("TIOCMBIS RTS");
		mbits = 0;
		ioctl(fd, 0x5415 /*TIOCMGET*/, &mbits);
		printf("attach: modem after  RTS-assert = 0x%x\n", mbits);
	}
	printf("attach: BCSP ldisc set on %s (RESET_ON_INIT flag=%lu, RTS asserted)\n", tty, flags);
	fflush(stdout);
	return fd;   /* caller must keep this open */
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
	if (argc < 2) { fprintf(stderr, "usage: btup attach|up|attachup|klog <tty>\n"); return 2; }

	if (!strcmp(argv[1], "klog"))
		return do_klog() == 0 ? 0 : 1;

	if (!strcmp(argv[1], "up"))
		return do_up() == 0 ? 0 : 1;

	if (argc < 3) { fprintf(stderr, "need <tty>\n"); return 2; }

	int fd = do_attach(argv[2]);
	if (fd < 0) return 1;

	if (!strcmp(argv[1], "attachup")) {
		sleep(2);          /* let BCSP link establish */
		do_up();
	}

	/* hold the ldisc open until killed */
	for (;;) pause();
	return 0;
}
