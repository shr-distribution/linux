/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * arch/arm/include/debug/msm-uart.S
 *
 * Debugging macro for Qualcomm MSM simple (non-DM) UART
 * Used for MSM7x30 UART1/2/3 (not UARTDM)
 *
 * Copyright (c) 2024, Linux community
 */

/* MSM UART register offsets */
#define MSM_UART_SR	0x0008	/* Status Register */
#define MSM_UART_TF	0x000c	/* Transmit FIFO */

/* Status Register bits */
#define MSM_UART_SR_TX_READY	(1 << 2)
#define MSM_UART_SR_TX_EMPTY	(1 << 3)

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =CONFIG_DEBUG_UART_PHYS
	ldr	\rv, =CONFIG_DEBUG_UART_VIRT
	.endm

	.macro	senduart, rd, rx
	strb	\rd, [\rx, #MSM_UART_TF]
	.endm

	.macro	waituartcts, rd, rx
	.endm

	.macro	waituarttxrdy, rd, rx
1001:	ldr	\rd, [\rx, #MSM_UART_SR]
	tst	\rd, #MSM_UART_SR_TX_READY
	beq	1001b
	.endm

	.macro	busyuart, rd, rx
1002:	ldr	\rd, [\rx, #MSM_UART_SR]
	tst	\rd, #MSM_UART_SR_TX_EMPTY
	beq	1002b
	.endm
