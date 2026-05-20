/*
 * Stream a file through AF_ALG AES-128-CBC.
 *
 * Usage:  qce-streamenc <alg-name> <input-file> <output-file>
 *
 * Reads input in 64KB chunks, sends each through the same op_fd
 * (IV state is preserved across sendmsg/read pairs).  Uses
 * MSG_MORE on all but the last sendmsg so the kernel doesn't
 * finalize the cipher state until end-of-stream.
 *
 * Compile: arm-linux-gnueabihf-gcc -static -O2 -o qce-streamenc qce-streamenc.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/socket.h>
#include <linux/if_alg.h>

#ifndef SOL_ALG
#define SOL_ALG 279
#endif
#ifndef ALG_SET_KEY
#define ALG_SET_KEY 1
#endif
#ifndef ALG_OP_ENCRYPT
#define ALG_OP_ENCRYPT 1
#endif
#ifndef ALG_SET_IV
#define ALG_SET_IV 2
#endif
#ifndef ALG_SET_OP
#define ALG_SET_OP 3
#endif

/* CE2 SEG_SIZE is a 16-bit field (max 65535); 64K wraps to 0 and
 * the engine errors out (SW_ERR).  32K is safely below the limit
 * and matches a common cryptdev block boundary.
 */
#define CHUNK (32 * 1024)

static double now_sec(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char **argv)
{
	struct sockaddr_alg sa = {
		.salg_family = AF_ALG,
		.salg_type   = "skcipher",
	};
	int sock, op_fd, in_fd, out_fd;
	unsigned char key[16], iv[16];
	unsigned char *buf;
	struct msghdr msg = { 0 };
	struct cmsghdr *cmsg;
	struct iovec iov;
	unsigned int cmsglen;
	void *cbuf;
	struct af_alg_iv *aiv;
	__u32 *op;
	ssize_t n;
	size_t total = 0;
	double t0, t1;

	if (argc != 4) {
		fprintf(stderr,
			"Usage: %s <alg> <input> <output>\n  e.g. %s cbc-aes-qce in.bin out.bin\n",
			argv[0], argv[0]);
		return 1;
	}

	memset(key, 0x42, sizeof(key));
	memset(iv, 0x17, sizeof(iv));

	strncpy((char *)sa.salg_name, argv[1], sizeof(sa.salg_name) - 1);

	in_fd = open(argv[2], O_RDONLY);
	if (in_fd < 0) { perror("open input"); return 1; }
	out_fd = open(argv[3], O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (out_fd < 0) { perror("open output"); close(in_fd); return 1; }

	sock = socket(AF_ALG, SOCK_SEQPACKET, 0);
	if (sock < 0) { perror("socket"); return 1; }
	if (bind(sock, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		fprintf(stderr, "bind(%s) failed: %s\n", argv[1], strerror(errno));
		close(sock); return 1;
	}
	unsigned int klen = 16;	/* default AES-128 */
	if (strstr(argv[1], "3des"))
		klen = 24;
	else if (strstr(argv[1], "des"))
		klen = 8;
	if (setsockopt(sock, SOL_ALG, ALG_SET_KEY, key, klen) < 0) {
		fprintf(stderr, "setkey(%u) failed: %s\n", klen, strerror(errno));
		close(sock); return 1;
	}

	op_fd = accept(sock, NULL, NULL);
	if (op_fd < 0) { perror("accept"); close(sock); return 1; }

	unsigned int ivl = (klen == 8 || klen == 24) ? 8 : 16;
	cmsglen = CMSG_SPACE(sizeof(__u32)) +
		  CMSG_SPACE(sizeof(struct af_alg_iv) + ivl);
	cbuf = calloc(1, cmsglen);
	if (!cbuf) { close(op_fd); close(sock); return 1; }
	msg.msg_control = cbuf;
	msg.msg_controllen = cmsglen;

	cmsg = CMSG_FIRSTHDR(&msg);
	cmsg->cmsg_level = SOL_ALG;
	cmsg->cmsg_type = ALG_SET_OP;
	cmsg->cmsg_len = CMSG_LEN(sizeof(__u32));
	op = (__u32 *)CMSG_DATA(cmsg);
	*op = ALG_OP_ENCRYPT;

	cmsg = CMSG_NXTHDR(&msg, cmsg);
	cmsg->cmsg_level = SOL_ALG;
	cmsg->cmsg_type = ALG_SET_IV;
	cmsg->cmsg_len = CMSG_LEN(sizeof(struct af_alg_iv) + ivl);
	aiv = (struct af_alg_iv *)CMSG_DATA(cmsg);
	aiv->ivlen = ivl;
	memcpy(aiv->iv, iv, ivl);

	buf = malloc(CHUNK);
	if (!buf) return 1;

	t0 = now_sec();

	while ((n = read(in_fd, buf, CHUNK)) > 0) {
		iov.iov_base = buf;
		iov.iov_len = n;
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;
		/* MSG_MORE on every chunk so the kernel keeps streaming.
		 * We never finalize the cipher; this is fine for CBC where
		 * each block depends only on the previous ciphertext.
		 */
		if (sendmsg(op_fd, &msg, MSG_MORE) != n) {
			fprintf(stderr, "sendmsg(%zd) failed: %s\n",
				n, strerror(errno));
			break;
		}
		/* After first sendmsg, drop the op+IV cmsg (kernel only
		 * needs them once per op_fd).
		 */
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
		if (read(op_fd, buf, n) != n) {
			fprintf(stderr, "read(%zd) failed: %s\n",
				n, strerror(errno));
			break;
		}
		if (write(out_fd, buf, n) != n) {
			perror("write");
			break;
		}
		total += n;
	}

	t1 = now_sec();

	free(buf);
	free(cbuf);
	close(op_fd);
	close(sock);
	close(in_fd);
	close(out_fd);

	printf("alg=%s bytes=%zu time=%.3f s  throughput=%.2f MB/s\n",
	       argv[1], total, t1 - t0,
	       total / ((t1 - t0) * 1024.0 * 1024.0));
	return 0;
}
