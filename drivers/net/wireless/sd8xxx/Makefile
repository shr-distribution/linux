CONFIG_DEBUG=0

# Proc in /proc/net/wlan
CONFIG_PROC_DEBUG=y

# Re-association in driver
CONFIG_REASSOCIATION=y

# Manufacturing firmware support
CONFIG_MFG_CMD_SUPPORT=y

ifeq ($(CONFIG_DEBUG),1)
	EXTRA_CFLAGS += -DDEBUG_LEVEL1
endif

ifeq ($(CONFIG_DEBUG),2)
	EXTRA_CFLAGS += -DDEBUG_LEVEL1
	EXTRA_CFLAGS += -DDEBUG_LEVEL2
	DBG=	-dbg
endif

ifeq ($(CONFIG_DEBUG),3)
	EXTRA_CFLAGS += -DDEBUG_LEVEL1
	EXTRA_CFLAGS += -DDEBUG_LEVEL2
	EXTRA_CFLAGS += -DDEBUG_LEVEL3
	DBG=	-dbg
endif

ifeq ($(CONFIG_DEBUG),4)
	EXTRA_CFLAGS += -DDEBUG_LEVEL1
	EXTRA_CFLAGS += -DDEBUG_LEVEL2
	EXTRA_CFLAGS += -DDEBUG_LEVEL3
	EXTRA_CFLAGS += -DDEBUG_LEVEL4
	DBG=	-dbg
endif

ifeq ($(CONFIG_PROC_DEBUG),y)
	EXTRA_CFLAGS += -DPROC_DEBUG
	export CONFIG_PROC_DEBUG
endif

ifeq ($(CONFIG_REASSOCIATION),y)
	EXTRA_CFLAGS += -DREASSOCIATION
endif

ifeq ($(CONFIG_MFG_CMD_SUPPORT),y)
	EXTRA_CFLAGS += -DMFG_CMD_SUPPORT
endif

sd8686-objs =	wlan/wlan_main.o wlan/wlan_fw.o wlan/wlan_wext.o \
				wlan/wlan_rx.o wlan/wlan_tx.o \
				wlan/wlan_cmd.o wlan/wlan_cmdresp.o \
				wlan/wlan_proc.o wlan/wlan_scan.o wlan/wlan_join.o \
				wlan/wlan_wmm.o wlan/wlan_11d.o \
				if/if_sdio/if_sdio.o

ifeq ($(CONFIG_PROC_DEBUG), y)
sd8686-objs += wlan/wlan_debug.o
endif

obj-m += sd8686.o

EXTRA_CFLAGS += -DFPNUM='"13"'
EXTRA_CFLAGS += -DFB
EXTRA_CFLAGS += -I$(src)/os/linux -I$(src)/wlan

ifdef CONFIG_MMC_DEBUG
#	EXTRA_CFLAGS += -DDEBUG_LEVEL1
#	EXTRA_CFLAGS += -DDEBUG_LEVEL2
#	EXTRA_CFLAGS += -DDEBUG_LEVEL3
#	EXTRA_CFLAGS += -DDEBUG_LEVEL4
#	DBG=    -dbg
endif

