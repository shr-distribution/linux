# SPDX-License-Identifier: GPL-2.0-only
#
# Cross-build the Gemini register poller for ARMv7. Produces a static
# binary that drops onto either the LuneOS or the webOS rootfs without
# library deps.

CC ?= arm-linux-gnueabihf-gcc
CFLAGS ?= -O2 -Wall -Wextra -static

gemini_reg_poll: gemini_reg_poll.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f gemini_reg_poll
