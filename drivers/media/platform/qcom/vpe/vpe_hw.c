// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 Video Processing Engine (VPE) - Hardware abstraction
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_vpe1 driver from webOS kernel.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include "vpe_hw.h"

/*
 * VPE polyphase scale coefficient tables, indexed by phase_step (= input/output
 * ratio): 0.2-0.4 / 0.4-0.6 / 0.6-0.8 / 0.8-20.0, covering ~5x upscale through
 * 20x downscale. 4 taps, 32 phases; each phase is two words packed exactly as
 * the hardware wants: LSB = (tap0<<16)|tap1, MSB = (tap3<<16)|tap2 (each tap a
 * 10-bit signed coefficient, taps sum to 512 = unity gain).
 *
 * Recovered from the TouchPad's own (unstripped) webOS libqcameralib.so
 * (vpe_scale_0p*_C0..C3 + vpe_init_scale_table). The previous placeholder
 * tables summed to ~0 (a high-pass filter), so the scaler emitted only edges.
 */
static const u32 vpe_scale_coeff_0p2_0p4[] = {
	0x0083008d, 0x006b0084, 0x0083008c, 0x006c0084,
	0x0082008c, 0x006d0084, 0x0081008c, 0x006d0085,
	0x0080008c, 0x006e0085, 0x007f008b, 0x006f0086,
	0x007f008a, 0x00700086, 0x007e008a, 0x00710086,
	0x007d008a, 0x00720086, 0x007d0089, 0x00730086,
	0x007c0089, 0x00730087, 0x007b0089, 0x00740087,
	0x007b0088, 0x00750087, 0x00790089, 0x00750088,
	0x00780089, 0x00760088, 0x00770089, 0x00770088,
	0x00770088, 0x00770089, 0x00760088, 0x00780089,
	0x00750088, 0x00790089, 0x00750087, 0x007b0088,
	0x00740087, 0x007b0089, 0x00730087, 0x007c0089,
	0x00730086, 0x007d0089, 0x00720086, 0x007d008a,
	0x00710086, 0x007e008a, 0x00700086, 0x007f008a,
	0x006f0086, 0x007f008b, 0x006e0085, 0x0080008c,
	0x006d0085, 0x0081008c, 0x006d0084, 0x0082008c,
	0x006c0084, 0x0083008c, 0x006b0084, 0x0083008d,
};

static const u32 vpe_scale_coeff_0p4_0p6[] = {
	0x008800ce, 0x001d008c, 0x008400cd, 0x0020008e,
	0x008000cc, 0x00210092, 0x007b00cc, 0x00240094,
	0x007700c9, 0x00270098, 0x007300c8, 0x0029009b,
	0x006f00c7, 0x002c009d, 0x006b00c5, 0x002f00a0,
	0x006700c4, 0x003200a2, 0x006200c2, 0x003600a5,
	0x005f00bf, 0x003900a8, 0x005b00bf, 0x003b00aa,
	0x005700bd, 0x003e00ad, 0x005400b9, 0x004200b0,
	0x005000b8, 0x004500b2, 0x004c00b6, 0x004900b4,
	0x004900b4, 0x004c00b6, 0x004500b2, 0x005000b8,
	0x004200b0, 0x005400b9, 0x003e00ad, 0x005700bd,
	0x003b00aa, 0x005b00bf, 0x003900a8, 0x005f00bf,
	0x003600a5, 0x006200c2, 0x003200a2, 0x006700c4,
	0x002f00a0, 0x006b00c5, 0x002c009d, 0x006f00c7,
	0x0029009b, 0x007300c8, 0x00270098, 0x007700c9,
	0x00240094, 0x007b00cc, 0x00210092, 0x008000cc,
	0x0020008e, 0x008400cd, 0x001d008c, 0x008800ce,
};

static const u32 vpe_scale_coeff_0p6_0p8[] = {
	0x0068012f, 0x03f80070, 0x0060012f, 0x03f80078,
	0x0059012e, 0x03f80080, 0x0052012c, 0x03f80089,
	0x004b012a, 0x03f90091, 0x00440128, 0x03f9009a,
	0x003d0125, 0x03fa00a3, 0x00370121, 0x03fb00ac,
	0x0031011e, 0x03fc00b4, 0x002b0119, 0x03fe00bd,
	0x00260114, 0x000000c5, 0x0021010e, 0x000200ce,
	0x001c0109, 0x000400d6, 0x00180102, 0x000600df,
	0x001400fc, 0x000900e6, 0x001000f5, 0x000c00ee,
	0x000c00ee, 0x001000f5, 0x000900e6, 0x001400fc,
	0x000600df, 0x00180102, 0x000400d6, 0x001c0109,
	0x000200ce, 0x0021010e, 0x000000c5, 0x00260114,
	0x03fe00bd, 0x002b0119, 0x03fc00b4, 0x0031011e,
	0x03fb00ac, 0x00370121, 0x03fa00a3, 0x003d0125,
	0x03f9009a, 0x00440128, 0x03f90091, 0x004b012a,
	0x03f80089, 0x0052012c, 0x03f80080, 0x0059012e,
	0x03f80078, 0x0060012f, 0x03f80070, 0x0068012f,
};

static const u32 vpe_scale_coeff_0p8_20p0[] = {
	0x000001ff, 0x00000000, 0x03f901fb, 0x03fe000d,
	0x03f301f5, 0x03fb001c, 0x03ed01ee, 0x03f9002b,
	0x03e801e5, 0x03f6003c, 0x03e401db, 0x03f3004d,
	0x03e001cf, 0x03f1005f, 0x03de01c2, 0x03ee0071,
	0x03db01b4, 0x03eb0085, 0x03d901a6, 0x03e80098,
	0x03d80195, 0x03e600ac, 0x03d70184, 0x03e300c1,
	0x03d70172, 0x03e100d5, 0x03d70160, 0x03df00e9,
	0x03d8014d, 0x03dd00fd, 0x03d8013a, 0x03db0112,
	0x03da0125, 0x03da0126, 0x03db0112, 0x03d8013a,
	0x03dd00fd, 0x03d8014d, 0x03df00e9, 0x03d70160,
	0x03e100d5, 0x03d70172, 0x03e300c1, 0x03d70184,
	0x03e600ac, 0x03d80195, 0x03e80098, 0x03d901a6,
	0x03eb0085, 0x03db01b4, 0x03ee0071, 0x03de01c2,
	0x03f1005f, 0x03e001cf, 0x03f3004d, 0x03e401db,
	0x03f6003c, 0x03e801e5, 0x03f9002b, 0x03ed01ee,
	0x03fb001c, 0x03f301f5, 0x03fe000d, 0x03f901fb,
};

int vpe_hw_reset(void __iomem *base)
{
	int timeout = 100;

	/* Issue software reset */
	writel(VPE_SW_RESET_VALUE, base + VPE_SW_RESET);

	/* Wait for reset to complete */
	while (timeout > 0) {
		if (!(readl(base + VPE_SW_RESET) & VPE_SW_RESET_VALUE))
			break;
		udelay(10);
		timeout--;
	}

	if (timeout == 0)
		return -ETIMEDOUT;

	/* Enable all clock gates */
	writel(VPE_CGC_ENABLE_VALUE, base + VPE_CGC_EN);

	/* Configure AXI read arbitration */
	writel(VPE_AXI_RD_ARB_CONFIG_VALUE, base + VPE_AXI_RD_ARB_CONFIG);

	/*
	 * Select command (display-list) mode ONCE here, before any frame
	 * registers are programmed -- this matches legacy msm_vpe1, which sets
	 * CMD_MODE at init and never per-frame. Setting it per-kick after the
	 * frame programming (as we did) left the engine consuming the kick but
	 * producing no output and never raising DONE.
	 */
	writel(VPE_CMD_MODE_VALUE, base + VPE_CMD_MODE);

	/* Set default operation mode */
	writel(VPE_DEFAULT_OP_MODE_VALUE, base + VPE_OP_MODE);

	/* Set default scale config */
	writel(VPE_DEFAULT_SCALE_CONFIG, base + VPE_SCALE_CONFIG);

	return 0;
}

void vpe_hw_enable_irq(void __iomem *base)
{
	writel(VPE_INTR_STATUS_DONE, base + VPE_INTR_ENABLE);
}

void vpe_hw_disable_irq(void __iomem *base)
{
	writel(0, base + VPE_INTR_ENABLE);
}

void vpe_hw_clear_irq(void __iomem *base)
{
	writel(VPE_INTR_STATUS_DONE, base + VPE_INTR_CLEAR);
}

u32 vpe_hw_get_irq_status(void __iomem *base)
{
	return readl(base + VPE_INTR_STATUS);
}

u32 vpe_hw_get_version(void __iomem *base)
{
	return readl(base + VPE_HW_VERSION);
}

void vpe_hw_start(void __iomem *base)
{
	/*
	 * CMD_MODE is established once in vpe_hw_reset (matching legacy
	 * msm_vpe1); here we only trigger the operation, as legacy vpe_start
	 * does (INTR_ENABLE is set by vpe_hw_enable_irq just before this).
	 */
	writel(1, base + VPE_DL0_START);
}

void vpe_hw_set_src_addr(void __iomem *base, dma_addr_t y_addr, dma_addr_t cbcr_addr)
{
	writel(y_addr, base + VPE_SRCP0_ADDR);
	writel(cbcr_addr, base + VPE_SRCP1_ADDR);
}

void vpe_hw_set_dst_addr(void __iomem *base, dma_addr_t y_addr, dma_addr_t cbcr_addr)
{
	writel(y_addr, base + VPE_OUTP0_ADDR);
	writel(cbcr_addr, base + VPE_OUTP1_ADDR);
}

void vpe_hw_set_src_size(void __iomem *base, u32 img_w, u32 img_h,
			 u32 crop_w, u32 crop_h, u32 stride)
{
	/* SRC_SIZE is the ROI (crop) being read; IMAGE_SIZE is the full frame */
	writel(((crop_h & 0xfff) << 16) | (crop_w & 0xfff), base + VPE_SRC_SIZE);
	writel(((img_h & 0xfff) << 16) | (img_w & 0xfff),
	       base + VPE_SRC_IMAGE_SIZE);
	/*
	 * YSTRIDE1 packs TWO strides: Y (luma) in bits [15:0] and CbCr (chroma)
	 * in bits [31:16] -- the camera HAL's mm_vpe_set_input_plane writes both
	 * (struct offsets 12 and 14). For NV12 the chroma plane has the same row
	 * stride as luma. Writing only the low half left chroma stride at 0, so
	 * the chroma DMA stalled: empty chroma plane and no DONE on a plain copy.
	 */
	writel((stride & 0xffff) | (stride << 16), base + VPE_SRC_YSTRIDE1);

	/*
	 * Source format + unpack pattern for NV12 (pseudo-planar Y + CbCr).
	 * These are the exact words the msm camera HAL programs for the VPE
	 * input plane (confirmed identical across the Sony nozomi, HTC and
	 * Samsung msm8660 liboemcamera.so / mm_vpe_set_input_plane). Our prior
	 * placeholder 0x2 / 0x03020100 left the engine unable to interpret the
	 * surface, so it consumed the kick but produced no output.
	 */
	writel(VPE_SRC_FORMAT_NV12, base + VPE_SRC_FORMAT);
	writel(VPE_PACK_PATTERN_NV12, base + VPE_SRC_UNPACK_PATTERN1);
}

void vpe_hw_set_dst_size(void __iomem *base, u32 width, u32 height, u32 stride)
{
	u32 size = ((height & 0xfff) << 16) | (width & 0xfff);

	writel(size, base + VPE_OUT_SIZE);
	/* Y stride [15:0] + chroma stride [31:16], same as the source plane. */
	writel((stride & 0xffff) | (stride << 16), base + VPE_OUT_YSTRIDE1);

	/* Output format + pack pattern for NV12, from the camera HAL's VPE
	 * output-plane config (mm_vpe_set_output_plane). Note the format word
	 * differs from the source one (byte2 = 0x09 vs 0x12). */
	writel(VPE_OUT_FORMAT_NV12, base + VPE_OUT_FORMAT);
	writel(VPE_PACK_PATTERN_NV12, base + VPE_OUT_PACK_PATTERN1);
}

static void vpe_hw_load_scale_coeff(void __iomem *base, u32 phase_step)
{
	const u32 *coeff;
	int i;

	/*
	 * Select coefficient table by phase_step (input/output ratio) in the
	 * 3.29 fixed-point the scaler uses (1.0 = 0x20000000): the camera HAL
	 * splits the range into <0.4 / 0.4-0.6 / 0.6-0.8 / >=0.8 (the last band
	 * covers 1:1 and all downscale).
	 */
	if (phase_step < 0x0ccccccd)		/* < 0.4  : ~2.5x-5x upscale */
		coeff = vpe_scale_coeff_0p2_0p4;
	else if (phase_step < 0x13333333)	/* 0.4-0.6: ~1.7x-2.5x upscale */
		coeff = vpe_scale_coeff_0p4_0p6;
	else if (phase_step < 0x1999999a)	/* 0.6-0.8: ~1.25x-1.7x upscale */
		coeff = vpe_scale_coeff_0p6_0p8;
	else					/* >= 0.8 : 1:1 .. 20x downscale */
		coeff = vpe_scale_coeff_0p8_20p0;

	/* Load coefficients */
	for (i = 0; i < VPE_SCALE_COEFF_NUM; i++) {
		writel(coeff[i * 2], base + VPE_SCALE_COEFF_LSBn(i));
		writel(coeff[i * 2 + 1], base + VPE_SCALE_COEFF_MSBn(i));
	}
}

void vpe_hw_set_scale(void __iomem *base, u32 src_w, u32 src_h, u32 dst_w, u32 dst_h)
{
	u64 phase_step_x, phase_step_y;
	u32 step_x, step_y;
	u32 op_mode;

	/* Calculate phase step values (fixed point 3.29 format) */
	phase_step_x = ((u64)src_w << SCALER_PHASE_BITS) + (dst_w - 1);
	do_div(phase_step_x, dst_w);
	step_x = (u32)phase_step_x;

	phase_step_y = ((u64)src_h << SCALER_PHASE_BITS) + (dst_h - 1);
	do_div(phase_step_y, dst_h);
	step_y = (u32)phase_step_y;

	/*
	 * Enable/disable the scaler block in OP_MODE[1:0]. The phase steps
	 * and coefficients below are inert unless the scaler is enabled here:
	 * the legacy msm_vpe1 driver sets OP_MODE |= 0x3 when zooming and
	 * clears it for a 1:1 passthrough. Without this the VPE ignores the
	 * requested scale entirely.
	 */
	op_mode = readl(base + VPE_OP_MODE) & ~VPE_OP_MODE_SCALE_MASK;
	if (src_w != dst_w || src_h != dst_h)
		op_mode |= VPE_OP_MODE_SCALE_EN;
	writel(op_mode, base + VPE_OP_MODE);

	/* Load scale coefficients based on maximum phase step */
	vpe_hw_load_scale_coeff(base, max(step_x, step_y));

	/* Set phase init to 0 */
	writel(0, base + VPE_SCALE_PHASEX_INIT);
	writel(0, base + VPE_SCALE_PHASEY_INIT);

	/* Set phase step values */
	writel(step_x, base + VPE_SCALE_PHASEX_STEP);
	writel(step_y, base + VPE_SCALE_PHASEY_STEP);
}

void vpe_hw_set_roi(void __iomem *base, u32 src_x, u32 src_y,
		    u32 dst_x, u32 dst_y)
{
	/* Source crop offset and output placement offset */
	writel(((src_y & 0xfff) << 16) | (src_x & 0xfff), base + VPE_SRC_XY);
	writel(((dst_y & 0xfff) << 16) | (dst_x & 0xfff), base + VPE_OUT_XY);
}

void vpe_hw_set_rotation(void __iomem *base, int rotation)
{
	u32 op_mode = readl(base + VPE_OP_MODE);

	/* Clear rotation bits */
	op_mode &= ~VPE_OP_MODE_ROTATION_MASK;

	/* Set new rotation */
	switch (rotation) {
	case 90:
		op_mode |= VPE_OP_MODE_ROT_90;
		break;
	case 180:
		op_mode |= VPE_OP_MODE_ROT_180;
		break;
	case 270:
		op_mode |= VPE_OP_MODE_ROT_270;
		break;
	default:
		op_mode |= VPE_OP_MODE_ROT_0;
		break;
	}

	writel(op_mode, base + VPE_OP_MODE);
}

void vpe_hw_set_op_mode(void __iomem *base, u32 mode)
{
	writel(mode, base + VPE_OP_MODE);
}
