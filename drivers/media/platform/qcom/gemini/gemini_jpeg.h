/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm MSM8660 JPEG (Gemini) - JPEG marker preamble + standard tables
 *
 * Copyright (c) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 */

#ifndef _GEMINI_JPEG_H_
#define _GEMINI_JPEG_H_

#include <linux/types.h>

/*
 * Upper bound on the JPEG marker preamble we emit:
 *   SOI               2
 *   APP0/JFIF        18
 *   DQT (luma)       69
 *   DQT (chroma)     69
 *   DHT (DC luma)    33  (5 + 16 + 12)
 *   DHT (DC chroma)  33
 *   DHT (AC luma)   183  (5 + 16 + 162)
 *   DHT (AC chroma) 183
 *   SOF0             19
 *   SOS              14
 *  --------------------
 *   total           623
 *
 * Round up to 640 (multiple of 8 for the WE buffer alignment requirement).
 */
#define GEMINI_JPEG_HDR_MAX_LEN	640

/*
 * Huffman table descriptor: BITS[16] (codes-per-length histogram) plus
 * HUFFVAL[] (symbol values), per JPEG Annex C.
 */
struct gemini_huff_table {
	const u8 *bits;
	const u8 *vals;
	unsigned int n_vals;
};

/*
 * Index into gemini_huff_tables[]:
 *   0 = DC luma, 1 = DC chroma, 2 = AC luma, 3 = AC chroma.
 *
 * The hardware table-loader function expects this same ordering when
 * pumping the four tables through the TABLE_INDEX/TABLE_DATA window.
 */
extern const struct gemini_huff_table gemini_huff_tables[4];

void gemini_scale_quant_luma(u16 out[64], int quality);
void gemini_scale_quant_chroma(u16 out[64], int quality);

size_t gemini_build_jpeg_header(u8 *dst, u32 width, u32 height,
				const u16 q_luma[64],
				const u16 q_chroma[64]);

#endif /* _GEMINI_JPEG_H_ */
