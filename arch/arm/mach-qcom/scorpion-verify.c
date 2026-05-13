// SPDX-License-Identifier: GPL-2.0-only
/*
 * Boot-time verification of __v7_scorpion_setup register writes.
 *
 * The Scorpion-specific procinfo entry (__v7_scorpion_proc_info in
 * arch/arm/mm/proc-v7.S) overrides several CP15 registers vs. the
 * generic v7 path:
 *
 *   NMRR  = 0x40e080e0 (Scorpion)   vs  0x40e040e0 (generic)
 *   L2CR1 = 0x100                   (DBB disabled for SMP stability)
 *   L2CR0 = 0xC0050F0F              (OoO bus attrs, error reporting)
 *   BPCR  = 0x01FF01FF              (full branch history)
 *
 * If MIDR matches Scorpion but NMRR is the generic value, the
 * Scorpion procinfo entry was not selected at boot — that's the
 * audit gap flagged as P0 #4 (Scorpion proc-v7.S NMRR path
 * verification).
 *
 * Read the registers back at arch_initcall time (after MMU and
 * printk are up) and pr_info the result so we can confirm the
 * Scorpion setup ran on every boot.
 */

#include <linux/init.h>
#include <linux/printk.h>

#define SCORPION_MIDR_MATCH	0x510f02d0	/* per proc-v7.S:858 */
#define SCORPION_MIDR_MASK	0xff0ff0f0	/* per proc-v7.S:859 */
#define SCORPION_NMRR_EXPECT	0x40e080e0
#define GENERIC_V7_NMRR		0x40e040e0

static int __init scorpion_verify_init(void)
{
	u32 midr, nmrr, prrr, actlr;
	u32 l2cr0, l2cr1, bpcr;
	u32 l2cpucr, spcr;

	asm volatile("mrc p15, 0, %0, c0, c0, 0" : "=r" (midr));

	if ((midr & SCORPION_MIDR_MASK) !=
	    (SCORPION_MIDR_MATCH & SCORPION_MIDR_MASK))
		return 0;

	asm volatile("mrc p15, 0, %0, c10, c2, 1" : "=r" (nmrr));
	asm volatile("mrc p15, 0, %0, c10, c2, 0" : "=r" (prrr));
	asm volatile("mrc p15, 0, %0, c1,  c0, 1" : "=r" (actlr));
	asm volatile("mrc p15, 3, %0, c15, c0, 1" : "=r" (l2cr0));
	asm volatile("mrc p15, 3, %0, c15, c0, 3" : "=r" (l2cr1));
	asm volatile("mrc p15, 7, %0, c15, c0, 2" : "=r" (bpcr));

	/*
	 * Additional Scorpion-implementation-defined registers that the
	 * legacy webOS kernel programs (proc-v7.S "optimal MP" path) or
	 * that the bootloader (arch-init-scorpion.S) initialises.
	 *
	 * L2CPUCR (p15,3,c15,c0,2): legacy bootloader sets 0xe0 (L2 parity
	 *   error reporting); legacy proc-v7 SCORPIONMP path further ORs
	 *   bit 21 ("optimal setting for Scorpion MP"). Our mainline
	 *   __v7_scorpion_setup does NOT touch this register. If readback
	 *   shows 0, TZ wiped the bootloader setting and we should restore
	 *   it. If it shows 0xe0, TZ preserved the bootloader value but
	 *   bit 21 is still missing.
	 *
	 * SPCR (p15,0,c9,c7,0): legacy bootloader writes 0x0F (error
	 *   reporting umbrella). Comment in legacy source notes the reset
	 *   value is unpredictable so it MUST be written. We don't write
	 *   it. If readback shows non-zero, bootloader/TZ left something
	 *   sane. If zero, we have unpredictable error reporting.
	 */
	asm volatile("mrc p15, 3, %0, c15, c0, 2" : "=r" (l2cpucr));
	asm volatile("mrc p15, 0, %0, c9,  c7, 0" : "=r" (spcr));

	pr_info("scorpion: MIDR=0x%08x NMRR=0x%08x PRRR=0x%08x\n",
		midr, nmrr, prrr);
	pr_info("scorpion: ACTLR=0x%08x (err_rep=%s mp=%s smp_nAMP=%s)\n",
		actlr,
		(actlr & 0x37) == 0x37 ? "on" : "off",
		(actlr & (1U << 24))   ? "on" : "off",
		(actlr & (1U <<  6))   ? "on" : "off");
	pr_info("scorpion: L2CR0=0x%08x L2CR1=0x%08x BPCR=0x%08x\n",
		l2cr0, l2cr1, bpcr);
	pr_info("scorpion: L2CPUCR=0x%08x (parity_err=%s mp_bit21=%s) SPCR=0x%08x\n",
		l2cpucr,
		(l2cpucr & 0xe0) == 0xe0 ? "on" : "off",
		(l2cpucr & (1U << 21))   ? "on" : "off",
		spcr);

	if (nmrr == SCORPION_NMRR_EXPECT)
		pr_info("scorpion: __v7_scorpion_setup confirmed (Scorpion NMRR applied)\n");
	else if (nmrr == GENERIC_V7_NMRR)
		pr_warn("scorpion: NMRR is generic v7 value — Scorpion proc_info NOT selected!\n");
	else
		pr_warn("scorpion: NMRR unexpected (0x%08x) — investigate\n", nmrr);

	return 0;
}
arch_initcall(scorpion_verify_init);
