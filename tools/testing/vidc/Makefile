CC ?= arm-linux-gnueabihf-gcc
CFLAGS := -static

all: test-vidc test-vidc-enc

test-vidc: test-vidc.c
	$(CC) $(CFLAGS) -o $@ $<

test-vidc-enc: test-vidc-enc.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f test-vidc test-vidc-enc
