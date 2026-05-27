/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * linux/include/linux/a6_sbw_interface.h
 *
 * Spy-Bi-Wire (SBW) protocol interface for MSP430 programming
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 * Author: Raj Mojumder <raj.mojumder@palm.com>
 *
 * DEPRECATED: This interface is kept for compatibility during driver
 * modernization. New code should use GPIO descriptors directly within
 * the driver rather than using board-specific callback functions.
 */

#ifndef _A6_SBW_INTERFACE_H_
#define _A6_SBW_INTERFACE_H_

#include <linux/types.h>

/*
 * Legacy SBW callback interface - DEPRECATED
 *
 * Modern drivers should use GPIO descriptors (struct gpio_desc) obtained
 * from device tree and call gpiod_* functions directly instead of using
 * these callbacks.
 *
 * This structure is kept temporarily during the transition to device tree.
 */
struct a6_sbw_interface {
	/* per-A6-device interface (separate instantiation for every a6 device) */
	struct {
		uint16_t (*SetSBWTCK)(void);
		uint16_t (*ClrSBWTCK)(void);
		uint16_t (*SetSBWTDIO)(void);
		uint16_t (*ClrSBWTDIO)(void);
		uint16_t (*SetInSBWTDIO)(void);
		uint16_t (*SetOutSBWTDIO)(void);
		uint16_t (*GetSBWTDIO)(void);
		uint16_t (*SetSBWAKEUP)(void);
		uint16_t (*ClrSBWAKEUP)(void);
	} a6_per_device_interface;

	/* per-target interface (separate instantiation for every board) */
	struct {
		void (*delay)(uint32_t delay_us);
	} a6_per_target_interface;
};

/*
 * Interrupt control macros - DEPRECATED
 *
 * These macros use the deprecated local_irq_save/restore API.
 * Modern code should use:
 * - spin_lock_irqsave/spin_unlock_irqrestore for general locking
 * - disable_irq/enable_irq for disabling specific interrupt lines
 *
 * For SBW timing-critical sections, consider using:
 * - gpiod_set_value_cansleep() for non-atomic contexts
 * - gpiod_set_value() for atomic contexts (if needed)
 * - Proper spinlocks for critical sections
 */
#ifdef __linux__
#include <linux/irqflags.h>
#define a6_disable_interrupts(flags) do { (void)(flags); local_irq_save(flags); } while (0)
#define a6_enable_interrupts(flags)  do { local_irq_restore(flags); } while (0)
#else
#define a6_disable_interrupts(flags) do { i_need_definition(); } while (0)
#define a6_enable_interrupts(flags)  do { i_need_definition(); } while (0)
#endif

#endif /* _A6_SBW_INTERFACE_H_ */
