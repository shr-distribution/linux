/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024 Herrie <herrie@herrie.org>
 *
 * Z180 2D Graphics Engine driver for Qualcomm MSM8660/APQ8060
 *
 * Register definitions based on legacy kgsl_g12 driver.
 */

#ifndef __Z180_GPU_H__
#define __Z180_GPU_H__

#include <linux/types.h>

/* VGC (Vector Graphics Controller) registers */
#define Z180_VGC_COMMANDSTREAM		0x0000
#define Z180_VGC_MMUCOMMANDSTREAM	0x03FC
#define Z180_VGC_IRQSTATUS		0x0418
#define Z180_VGC_IRQENABLE		0x0438
#define Z180_VGC_IRQ_ACTIVE_CNT		0x04E0
#define Z180_VGC_MH_READ_ADDR		0x0510
#define Z180_VGC_MH_DATA_ADDR		0x0518

/* VGV3 (Vector Graphics V3 core) registers */
#define Z180_VGV3_CONTROL		0x0070
#define Z180_VGV3_MODE			0x0071
#define Z180_VGV3_WRITEADDR		0x0072
#define Z180_VGV3_NEXTADDR		0x0075
#define Z180_VGV3_NEXTCMD		0x0076
#define Z180_VGV3_LAST			0x007F

/* MH (Memory Hub) registers - accessed via command window */
#define Z180_MH_MMU_CONFIG		0x0040
#define Z180_MH_MMU_VA_RANGE		0x0041
#define Z180_MH_MMU_PT_BASE		0x0042
#define Z180_MH_MMU_PAGE_FAULT		0x0043
#define Z180_MH_MMU_TRAN_ERROR		0x0044
#define Z180_MH_MMU_INVALIDATE		0x0045
#define Z180_MH_MMU_MPU_BASE		0x0046
#define Z180_MH_MMU_MPU_END		0x0047

#define Z180_MH_ARBITER_CONFIG		0x0A40
#define Z180_MH_INTERRUPT_MASK		0x0A42
#define Z180_MH_INTERRUPT_STATUS	0x0A43
#define Z180_MH_INTERRUPT_CLEAR		0x0A44
#define Z180_MH_AXI_ERROR		0x0A45
#define Z180_MH_AXI_HALT_CONTROL	0x0A50
#define Z180_MH_CLNT_INTF_CTRL_CONFIG1	0x0A54
#define Z180_MH_CLNT_INTF_CTRL_CONFIG2	0x0A55

/* IRQ status bits */
#define Z180_IRQ_MH_MASK		0x00000001
#define Z180_IRQ_G2D_MASK		0x00000002
#define Z180_IRQ_FIFO_MASK		0x00000004

/* MH interrupt bits */
#define Z180_MH_INT_AXI_READ_ERROR	0x00000001
#define Z180_MH_INT_AXI_WRITE_ERROR	0x00000002
#define Z180_MH_INT_MMU_PAGE_FAULT	0x00000004

/* Command stream packet types */
#define Z180_STREAM_PACKET		0x7C000176
#define Z180_STREAM_PACKET_CALL		0x7C000275
#define Z180_MARKER_CMD			0x00008000
#define Z180_CALL_CMD			0x00001000
#define Z180_STREAM_END_CMD		0x00009000

/* VGV3 control register bits */
#define Z180_VGV3_NEXTCMD_JUMP		0x01
#define Z180_VGV3_NEXTCMD_FSHIFT	12
#define Z180_VGV3_NEXTCMD_FMASK		0x7

#define Z180_VGV3_CONTROL_MARKADD_FSHIFT 0
#define Z180_VGV3_CONTROL_MARKADD_FMASK  0xfff

/* Command window targets */
#define Z180_CMDWINDOW_2D		0x00
#define Z180_CMDWINDOW_MMU		0x03

#define Z180_CMDWINDOW_TARGET_SHIFT	0
#define Z180_CMDWINDOW_TARGET_MASK	0x000000FF
#define Z180_CMDWINDOW_ADDR_SHIFT	8
#define Z180_CMDWINDOW_ADDR_MASK	0x00FFFF00

/* MH arbiter config bits */
#define Z180_MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT	0x06
#define Z180_MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT		0x07
#define Z180_MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT	0x08
#define Z180_MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT		0x09
#define Z180_MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT		0x0A
#define Z180_MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT	0x0D
#define Z180_MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT	0x0E
#define Z180_MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT	0x0F
#define Z180_MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT		0x10
#define Z180_MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT		0x16
#define Z180_MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT		0x17
#define Z180_MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT		0x18
#define Z180_MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT		0x19
#define Z180_MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT		0x1A

/* Default MH arbiter configuration */
#define Z180_CFG_MHARB						\
	(0x10							\
	| (0 << Z180_MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT)	\
	| (1 << Z180_MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT)	\
	| (0 << Z180_MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT)	\
	| (1 << Z180_MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT)	\
	| (0 << Z180_MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT)	\
	| (0x8 << Z180_MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT)	\
	| (1 << Z180_MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT)		\
	| (1 << Z180_MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

/*
 * Ringbuffer configuration
 *
 * The Z180 uses a simple ringbuffer with fixed-size packets.
 * Each packet consists of a marker section followed by a command section.
 */
#define Z180_PACKET_SIZE	15	/* words per packet */
#define Z180_MARKER_SIZE	10	/* words for marker */
#define Z180_PACKET_COUNT	8	/* number of packets in ringbuffer */
#define Z180_RB_SIZE		(Z180_PACKET_SIZE * Z180_PACKET_COUNT * sizeof(u32))

#define Z180_INVALID_CONTEXT	UINT_MAX

/* Timestamp storage size (for memstore) */
#define Z180_MEMSTORE_SIZE	PAGE_SIZE

/* Timeouts */
#define Z180_IDLE_TIMEOUT_MS	2000
#define Z180_WAIT_TIMEOUT_MS	10000

/*
 * State stream packet sizes for context switching
 * These define the size of GPU state that needs to be saved/restored
 */
#define Z180_NUMTEXUNITS	4
#define Z180_TEXUNITREGCOUNT	25
#define Z180_VG_REGCOUNT	0x39

#define Z180_PACKETSIZE_BEGIN		3
#define Z180_PACKETSIZE_G2DCOLOR	2
#define Z180_PACKETSIZE_TEXUNIT		(Z180_TEXUNITREGCOUNT * 2)
#define Z180_PACKETSIZE_REG		(Z180_VG_REGCOUNT * 2)
#define Z180_PACKETSIZE_STATE		(Z180_PACKETSIZE_TEXUNIT * Z180_NUMTEXUNITS + \
					 Z180_PACKETSIZE_REG + Z180_PACKETSIZE_BEGIN + \
					 Z180_PACKETSIZE_G2DCOLOR)
#define Z180_PACKETSIZE_STATESTREAM	(ALIGN(Z180_PACKETSIZE_STATE * sizeof(u32), 32) / \
					 sizeof(u32))

/*
 * IOCTL definitions for userspace interface
 */
#define Z180_IOCTL_BASE		'Z'

/* Submit a command buffer for execution */
struct z180_submit_cmd {
	__u32 gpuaddr;		/* GPU address of command buffer */
	__u32 sizedwords;	/* Size of command buffer in dwords */
	__u32 timestamp;	/* Returned timestamp for this submission */
	__u32 flags;		/* Submission flags */
};

#define Z180_SUBMIT_FLAGS_CTX_SWITCH	BIT(0)

#define Z180_IOCTL_SUBMIT	_IOWR(Z180_IOCTL_BASE, 1, struct z180_submit_cmd)

/* Wait for a timestamp to complete */
struct z180_wait_timestamp {
	__u32 timestamp;	/* Timestamp to wait for */
	__u32 timeout_ms;	/* Timeout in milliseconds */
};

#define Z180_IOCTL_WAIT		_IOW(Z180_IOCTL_BASE, 2, struct z180_wait_timestamp)

/* Get device info */
struct z180_device_info {
	__u32 device_id;	/* Device ID (0 or 1) */
	__u32 chip_id;		/* Chip revision */
	__u32 timestamp;	/* Current completed timestamp */
	__u32 pending;		/* Number of pending submissions */
};

#define Z180_IOCTL_INFO		_IOR(Z180_IOCTL_BASE, 3, struct z180_device_info)

#endif /* __Z180_GPU_H__ */
