/* Raw userspace BCSP link establishment over a tty (like webOS bcattach).
 * Bypasses the kernel hci_bcsp/ldisc entirely: opens the tty, drives the
 * SYNC->SYNC-RSP->CONF->CONF-RSP handshake by hand, prints all RX.
 * Purpose: isolate kernel-BCSP-driver vs msm_serial TX. If the chip
 * advances (sends CONF) here, the kernel driver was the issue.
 *
 * Build (mainline LuneOS, glibc): arm-linux-gnueabihf-gcc -static -O2 -o btbcsp btbcsp.c
 * Usage: btbcsp /dev/ttyMSM1 [bt_wake_gpio]
 *   bt_wake_gpio defaults to 643 (TLMM base 512 + BT_WAKE 131); pass 0 to skip.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <time.h>

/* on-wire BCSP packets (chan 1, CRC), byte-identical to webOS */
static const unsigned char SYNC[]    = {0xc0,0x40,0x41,0x00,0x7e,0xda,0xdc,0xed,0xed,0xa9,0x7a,0xc0};
static const unsigned char SYNCRSP[] = {0xc0,0x40,0x41,0x00,0x7e,0xac,0xaf,0xef,0xee,0xbb,0x84,0xc0};
static const unsigned char CONF[]    = {0xc0,0x40,0x41,0x00,0x7e,0xad,0xef,0xac,0xed,0xa1,0xa6,0xc0};
static const unsigned char CONFRSP[] = {0xc0,0x40,0x41,0x00,0x7e,0xde,0xad,0xd0,0xd0,0x83,0x58,0xc0};

/* magic payloads to detect in RX */
static const unsigned char P_SYNC[]    = {0xda,0xdc,0xed,0xed};
static const unsigned char P_SYNCRSP[] = {0xac,0xaf,0xef,0xee};
static const unsigned char P_CONF[]    = {0xad,0xef,0xac,0xed};
static const unsigned char P_CONFRSP[] = {0xde,0xad,0xd0,0xd0};

static long now_ms(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec*1000+t.tv_nsec/1000000; }

/* Assert BT_WAKE (chip PIO[1], host->chip wake). On mainline the TLMM gpiochip
 * base is 512, so BT_WAKE gpio131 = sysfs 643. The serdev driver drives this in
 * set_power, but ldisc/userspace tests bypass it — without it the CSR chip can
 * keep its UART RX gated (deep sleep) while still TXing SYNC. Non-fatal. */
static void bt_wake_assert(int gpio)
{
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

static int contains(const unsigned char *buf, int n, const unsigned char *pat){
	int i; for(i=0;i+4<=n;i++) if(!memcmp(buf+i,pat,4)) return 1; return 0;
}

int main(int argc, char **argv)
{
	struct termios ti;
	int fd, mbits, st = 0; /* 0 shy, 1 curious, 2 up */
	long start, last_tx = 0;
	unsigned char rx[512];
	const char *tty = argc>1?argv[1]:"/dev/ttyMSM1";
	int wake_gpio = argc>2?atoi(argv[2]):643;

	bt_wake_assert(wake_gpio);
	fd = open(tty, O_RDWR|O_NOCTTY|O_NONBLOCK);
	if (fd<0){ perror("open"); return 1; }
	tcgetattr(fd,&ti); cfmakeraw(&ti);
	ti.c_cflag |= CLOCAL|CREAD; ti.c_cflag &= ~(CRTSCTS|PARENB|CSTOPB);
	ti.c_cflag = (ti.c_cflag & ~CSIZE) | CS8;
	cfsetospeed(&ti,B115200); cfsetispeed(&ti,B115200);
	tcflush(fd,TCIOFLUSH); tcsetattr(fd,TCSANOW,&ti); tcflush(fd,TCIOFLUSH);
	mbits = TIOCM_RTS|TIOCM_DTR; ioctl(fd,TIOCMBIS,&mbits);  /* assert RTS */
	ioctl(fd,TIOCMGET,&mbits); printf("opened %s, modem=0x%x, raw 8N1 115200, RTS asserted\n", tty, mbits);

	start = now_ms();
	while (now_ms()-start < 8000 && st < 2) {
		long t = now_ms();
		/* periodic TX of our current-state packet */
		if (t - last_tx > 250) {
			if (st==0){ write(fd,SYNC,sizeof SYNC); }
			else if (st==1){ write(fd,CONF,sizeof CONF); }
			last_tx = t;
		}
		usleep(40000);
		int n = read(fd, rx, sizeof rx);
		if (n > 0) {
			int i; printf("[%4ld] RX(%d):", t-start, n);
			for(i=0;i<n && i<40;i++) printf(" %02x", rx[i]);
			printf("\n"); fflush(stdout);
			if (contains(rx,n,P_SYNC))    { write(fd,SYNCRSP,sizeof SYNCRSP); printf("  -> got SYNC, sent SYNC-RSP\n"); }
			if (contains(rx,n,P_SYNCRSP)) { if(st==0){st=1; printf("  -> got SYNC-RSP -> CURIOUS, sending CONF\n"); write(fd,CONF,sizeof CONF);} }
			if (contains(rx,n,P_CONF))    { write(fd,CONFRSP,sizeof CONFRSP); printf("  -> got CONF, sent CONF-RSP\n"); }
			if (contains(rx,n,P_CONFRSP)) { st=2; printf("  -> got CONF-RSP -> LINK UP!\n"); }
		}
	}
	printf(st>=2 ? "RESULT: LINK ESTABLISHED (chip responded to raw userspace BCSP)\n"
		     : "RESULT: no link (chip only-SYNC; msm_serial TX is the wall)\n");
	close(fd);
	return st>=2?0:2;
}
