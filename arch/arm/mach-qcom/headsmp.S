/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Scorpion-MP (MSM8660 / APQ8060) secondary-CPU "holding pen" entry.
 *
 * Legacy webOS arch/arm/mach-msm/headsmp.S used this pattern to work
 * around an IPI-from-WFI wake-up quirk on Scorpion-MP: a GIC SGI
 * delivered to a CPU sitting in plain `wfi` is unreliable on some
 * boots. Replacing `wfi` with `wfe` and using a shared-memory token
 * (`pen_release`) plus an explicit `sev` from the boot CPU yields a
 * deterministic wake.
 *
 * Mainline qcom's platsmp.c previously pointed cold-boot at
 * `secondary_startup_arm` directly. After scss_release_secondary takes
 * CPU1 out of reset, CPU1 lands at that address and tries to enter the
 * generic SMP init. There's no holding pen, no pen_release polling,
 * and no SEV-triggered wake. Works most of the time but not reliably,
 * particularly on warm hotplug-up.
 *
 * This file provides msm8660_secondary_startup: a tiny ARM-asm stub
 * that waits in a wfe-loop, polls the pen_release variable, and only
 * proceeds to secondary_startup when its own CPU id has been written
 * to pen_release. After consuming the release, it writes -1 back so
 * the boot CPU sees the ack.
 *
 * MPIDR aff0 is masked to 4 bits — sufficient for the dual-core part.
 */

#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>

ENTRY(msm8660_secondary_startup)
ARM_BE8(setend	be)
	mrc	p15, 0, r0, c0, c0, 5		@ MPIDR
	and	r0, r0, #15			@ this CPU's aff0 (0..3)

	/*
	 * Compute the physical address of pen_release. The label `1:`
	 * is at our current pc; subtracting that from the linker-time
	 * address of `1:` (loaded into r5) gives our virt/phys
	 * displacement, which we add to the linker-time address of
	 * pen_release (r6) to produce the runtime physical address.
	 *
	 * Cache is off here, so we read pen_release directly from DRAM.
	 * The boot CPU must dcache_flush before sev/IPI for our load to
	 * observe the new value.
	 */
	adr	r4, 2f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4

1:
	wfe						@ wait for boot CPU's SEV
	dsb						@ subsequent load after wake
	ldr	r7, [r6]
	cmp	r7, r0
	bne	1b					@ not us — back to wfe

	/*
	 * Released. Write -1 back to pen_release so the boot CPU's
	 * poll loop sees the ack. Use mvn to materialise -1 without
	 * a literal pool entry (we may still be at a low boot
	 * address with no .rodata mapping).
	 */
	mvn	r7, #0				@ r7 = -1 (0xFFFFFFFF)
	str	r7, [r6]
	dsb					@ ack visible to boot CPU
	b	secondary_startup

	.align	2
2:	.long	.
	.long	pen_release
ENDPROC(msm8660_secondary_startup)
