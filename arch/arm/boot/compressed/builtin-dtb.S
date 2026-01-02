/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Built-in DTB for bootloaders that cannot pass DTB and reject appended DTBs.
 * Used by Palm bootie on HP Pre3 (mantaray).
 */
	.section .builtin_dtb, "a"
	.balign 8
	.incbin "arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dtb"
