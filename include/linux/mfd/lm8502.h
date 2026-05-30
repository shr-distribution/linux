/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TI LM8502 Multi-Function Device
 *
 * The LM8502 is a combo LED + haptic + camera-flash controller exposed
 * over I2C. The MFD core handles the i2c_client, regmap, regulator and
 * chip-wide initialization; per-subsystem child platform drivers in
 * drivers/leds/ and drivers/input/misc/ bind to the spawned children
 * and use the parent's regmap.
 *
 * Children retrieve the parent's lm8502 state via
 *     struct lm8502 *chip = dev_get_drvdata(pdev->dev.parent);
 * and gate all register writes on lm8502_chip_ready(chip) so they
 * cooperate with the deferred chip_init pattern documented in
 * drivers/mfd/lm8502.c.
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
#define LM8502_CHIP_EN			0x40

#define LM8502_MAX_LEDS			10

/* --- Shared parent state -------------------------------------------- */

/**
 * struct lm8502 - parent (MFD core) state shared with child drivers
 * @dev:        the I2C client's struct device (parent of every child pdev)
 * @regmap:     the chip's 8-bit register map (volatile, no cache)
 * @lock:       serializes shared register operations between children
 *              (regmap has its own internal lock; @lock additionally
 *              protects @initialized / @suspended transitions)
 * @initialized: deferred chip_init has run successfully; child drivers
 *              must not issue dependent register writes before this
 *              flips true (use lm8502_chip_ready())
 * @suspended:  chip is currently powered down by parent suspend
 */
struct lm8502 {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	bool initialized;
	bool suspended;
};

/**
 * lm8502_chip_ready() - is the chip ready for register writes?
 *
 * Returns true once the deferred chip_init in drivers/mfd/lm8502.c has
 * completed and the chip is not currently suspended. Child drivers
 * should check this before issuing register writes; pattern is:
 *
 *     if (!lm8502_chip_ready(chip))
 *         return -EAGAIN;     // for sync callbacks
 *         // or just bail from a work handler
 */
static inline bool lm8502_chip_ready(struct lm8502 *chip)
{
	return chip->initialized && !chip->suspended;
}

/**
 * lm8502_write_retry() - regmap_write that retries once on -ENXIO
 *
 * The GSBI8/QUP bus on the HP TouchPad is shared with other I2C
 * clients (A6 batteries) and occasionally drops the first transaction
 * after a settling window with -ENXIO; the chip is also briefly
 * unresponsive in the post-reset window. Children should use this
 * helper for writes during initialization paths where dropping the
 * first byte is plausible. Steady-state brightness/haptic writes can
 * use plain regmap_write() since the chip is fully awake by then.
 *
 * Defined out-of-line in drivers/mfd/lm8502.c so all subsystem child
 * drivers share the same retry policy.
 */
int lm8502_write_retry(struct lm8502 *chip, unsigned int reg, unsigned int val);

#endif /* __LINUX_MFD_LM8502_H */
