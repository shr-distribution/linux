CC ?= arm-linux-gnueabihf-gcc
CFLAGS := -static

all: test-vidc

test-vidc: test-vidc.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f test-vidc
