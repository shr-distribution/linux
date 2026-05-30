/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TI LM8502 Multi-Function Device
 *
 * The LM8502 is a combo LED + haptic controller exposed over I2C. The
 * MFD core handles the i2c_client, regmap, regulator and chip-wide
 * initialization; per-subsystem child platform drivers in drivers/leds/
 * and drivers/input/misc/ bind to the spawned children and use the
 * parent's regmap.
 *
 * Children retrieve the parent's lm8502 state via
 *     struct lm8502 *chip = dev_get_drvdata(pdev->dev.parent);
 * and serialise register access on chip->lock.
 */
#ifndef __LINUX_MFD_LM8502_H
#define __LINUX_MFD_LM8502_H

#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/types.h>

/* --- Register map --------------------------------------------------- */

#define LM8502_ENGINE_CNTRL1		0x00
#define LM8502_ENGINE_CNTRL2		0x01
#define LM8502_GROUP_FADING1		0x02
#define LM8502_GROUP_FADING2		0x03
#define LM8502_GROUP_FADING3		0x04

#define LM8502_D1_CONTROL		0x06
#define LM8502_D2_CONTROL		0x07
#define LM8502_D3_CONTROL		0x08
#define LM8502_D4_CONTROL		0x09
#define LM8502_D5_CONTROL		0x0A
#define LM8502_D6_CONTROL		0x0B
#define LM8502_D7_CONTROL		0x0C
#define LM8502_D8_CONTROL		0x0D
#define LM8502_D9_CONTROL		0x0E
#define LM8502_D10_CONTROL		0x0F

#define LM8502_HAPTIC_CONTROL		0x10
#define LM8502_HAPTIC_FEEDBACK_CTRL	0x21
#define LM8502_HAPTIC_PWM_DUTY_CYCLE	0x22

#define LM8502_D1_CURRENT_CTRL		0x26
#define LM8502_D2_CURRENT_CTRL		0x27
#define LM8502_D3_CURRENT_CTRL		0x28
#define LM8502_D4_CURRENT_CTRL		0x29
#define LM8502_D5_CURRENT_CTRL		0x2A
#define LM8502_D6_CURRENT_CTRL		0x2B
#define LM8502_D7_CURRENT_CTRL		0x2C
#define LM8502_D8_CURRENT_CTRL		0x2D
#define LM8502_D9_CURRENT_CTRL		0x2E
#define LM8502_D10_CURRENT_CTRL		0x2F

#define LM8502_MISC			0x36
#define LM8502_ENGINE1_PC		0x37
#define LM8502_ENGINE2_PC		0x38
#define LM8502_STATUS			0x3A
#define LM8502_INT			0x3B
#define LM8502_RESET			0x3D

/* ENGINE_CNTRL1 bits */
#define LM8502_CHIP_EN			BIT(6)

/* ENGINE_CNTRL2 — value applied at chip_init: charge-pump CP_MODE = 1x.
 * Bit 5 selects the boost-converter compare mode; the value the legacy
 * legacy vendor driver writes matches the LP55xx-family ENGINE_CNTRL2 "all
 * engines in HOLD" default plus CP_MODE_1X.
 */
#define LM8502_ENGINE_CNTRL2_INIT	0x20

/* MISC (0x36) bits applied at chip_init */
#define LM8502_MISC_PWM_INPUT		BIT(1)	/* PWM input mux on D10 */
/* BOOST_EN: internal boost converter -- REQUIRED for LED current. */
#define LM8502_MISC_BOOST_EN		BIT(3)
#define LM8502_MISC_POWER_SAVE		BIT(5)	/* clock gate idle blocks */
#define LM8502_MISC_INIT		(LM8502_MISC_POWER_SAVE | \
					 LM8502_MISC_BOOST_EN | \
					 LM8502_MISC_PWM_INPUT)

/* D<n>_CONTROL bits */
#define LM8502_LED_MAPPING_MASK		0x03
#define LM8502_LED_MAPPING_DIRECT	0x00	/* drive from current reg */
#define LM8502_LED_MAX_CURRENT_MASK	0x18
#define LM8502_LED_MAX_CURRENT_SHIFT	3
#define LM8502_LED_MAX_CURRENT_9MA	0x02	/* field = 2 -> 9 mA cap */
/*
 * Initial D<n>_CONTROL value: MAX_CURRENT field = 2 (9 mA), DIRECT
 * mapping. Programmed for every channel during chip_init so a child
 * brightness write to D<n>_CURRENT_CTRL is honoured immediately.
 */
#define LM8502_LED_CONTROL_INIT		(LM8502_LED_MAX_CURRENT_9MA << \
					 LM8502_LED_MAX_CURRENT_SHIFT)

/* HAPTIC_FEEDBACK_CTRL bits */
#define LM8502_HAPTIC_FB_INVERT_DIR	BIT(0)	/* H-bridge polarity flip */
#define LM8502_HAPTIC_FB_ENABLE		BIT(1)	/* drive the motor */

#define LM8502_MAX_LEDS			10

/* RESET (0x3D) magic write value (LP55xx-family software reset). */
#define LM8502_RESET_MAGIC		0xFF

/* --- Shared parent state -------------------------------------------- */

/**
 * struct lm8502 - parent (MFD core) state shared with child drivers
 * @dev:        the I2C client's struct device (parent of every child pdev)
 * @regmap:     the chip's 8-bit register map (no cache; every read goes
 *              to the bus)
 * @lock:       serialises register sequences across children and against
 *              the parent's PM callbacks
 * @suspended:  set true while the chip is power-gated by parent suspend;
 *              children must check it under @lock before touching
 *              registers
 *
 * Chip readiness is implicit: child platform devices only probe after
 * the MFD parent's probe has completed lm8502_chip_init() successfully,
 * so by the time a child runs the chip is fully configured and ACKing
 * I2C. There is no separate "initialized" flag.
 */
struct lm8502 {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	bool suspended;
};

#endif /* __LINUX_MFD_LM8502_H */
