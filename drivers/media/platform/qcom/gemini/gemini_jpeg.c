// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 JPEG Encoder/Decoder (Gemini) - JPEG header generation
 *
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * The Gemini hardware writes only the entropy-coded segment of the JPEG
 * stream. The driver builds the SOI/JFIF/DQT/DHT/SOF0/SOS marker preamble
 * in software, prepends it to the destination buffer, and appends EOI
 * after the hardware reports FRAMEDONE.
 *
 * Quantization tables come from JPEG Annex K.1 scaled by the IJG
 * jpeg_quality_scaling formula (used by libjpeg and most JPEG software);
 * Huffman tables are JPEG Annex K.3 verbatim.
 */

#include <linux/array_size.h>
#include <linux/string.h>
#include <linux/types.h>

#include "gemini_jpeg.h"

/* Annex K.1 luminance quantization table */
static const u8 gemini_std_qtbl_luma[64] = {
	16, 11, 10, 16,  24,  40,  51,  61,
	12, 12, 14, 19,  26,  58,  60,  55,
	14, 13, 16, 24,  40,  57,  69,  56,
	14, 17, 22, 29,  51,  87,  80,  62,
	18, 22, 37, 56,  68, 109, 103,  77,
	24, 35, 55, 64,  81, 104, 113,  92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103,  99,
};

/* Annex K.1 chrominance quantization table */
static const u8 gemini_std_qtbl_chroma[64] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
};

/* JPEG Annex K.3 zigzag scan order */
static const u8 gemini_zigzag[64] = {
	 0,  1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
};

/* Annex K.3 standard Huffman tables */
static const u8 gemini_dc_luma_bits[16] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
};
static const u8 gemini_dc_luma_vals[12] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 gemini_dc_chroma_bits[16] = {
	0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
};
static const u8 gemini_dc_chroma_vals[12] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 gemini_ac_luma_bits[16] = {
	0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
};
static const u8 gemini_ac_luma_vals[162] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u8 gemini_ac_chroma_bits[16] = {
	0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
};
static const u8 gemini_ac_chroma_vals[162] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

const struct gemini_huff_table gemini_huff_tables[4] = {
	{ gemini_dc_luma_bits,   gemini_dc_luma_vals,   12 },
	{ gemini_dc_chroma_bits, gemini_dc_chroma_vals, 12 },
	{ gemini_ac_luma_bits,   gemini_ac_luma_vals,  162 },
	{ gemini_ac_chroma_bits, gemini_ac_chroma_vals, 162 },
};

/*
 * IJG-style quality scaling: maps 1..100 quality to a scale factor and
 * applies it to the standard Annex K.1 tables. quality == 50 leaves the
 * tables unchanged. Output entries are clamped to 1..255 and stored in
 * scan order matching the input table (NOT zigzag — that ordering is
 * only used when emitting the DQT marker).
 */
static void gemini_scale_quant_table(u16 out[64], const u8 std[64], int quality)
{
	int scale, i, q;

	if (quality <= 0)
		quality = 1;
	if (quality > 100)
		quality = 100;

	if (quality < 50)
		scale = 5000 / quality;
	else
		scale = 200 - quality * 2;

	for (i = 0; i < 64; i++) {
		q = (std[i] * scale + 50) / 100;
		if (q < 1)
			q = 1;
		if (q > 255)
			q = 255;
		out[i] = q;
	}
}

void gemini_scale_quant_luma(u16 out[64], int quality)
{
	gemini_scale_quant_table(out, gemini_std_qtbl_luma, quality);
	/*
	 * Diagnostic: force DC luma quantizer to 1 to rule out DC quant
	 * bug. Decoded JPEG will use q[0]=1 in DQT, encoder programs
	 * reciprocal 0xFFFF for q[0]. If Y=0 input then decodes to black,
	 * the DC path is correct and previous bug was quant-magnitude.
	 */
	out[0] = 1;
}

void gemini_scale_quant_chroma(u16 out[64], int quality)
{
	gemini_scale_quant_table(out, gemini_std_qtbl_chroma, quality);
}

/*
 * Build the JPEG marker preamble for an NV12 -> YUV420 H2V2 encode at
 * the given pixel dimensions and quality. Writes into `dst` and returns
 * the number of bytes written. Caller must ensure dst has at least
 * GEMINI_JPEG_HDR_MAX_LEN bytes available.
 */
size_t gemini_build_jpeg_header(u8 *dst, u32 width, u32 height,
				const u16 q_luma[64],
				const u16 q_chroma[64])
{
	u8 *p = dst;
	int i, t;

	/* SOI */
	*p++ = 0xFF; *p++ = 0xD8;

	/* APP0 / JFIF */
	*p++ = 0xFF; *p++ = 0xE0;
	*p++ = 0x00; *p++ = 0x10;	/* length 16 */
	*p++ = 'J';  *p++ = 'F';  *p++ = 'I';  *p++ = 'F';  *p++ = 0x00;
	*p++ = 0x01; *p++ = 0x01;	/* version 1.1 */
	*p++ = 0x00;			/* aspect ratio units = none */
	*p++ = 0x00; *p++ = 0x01;	/* X density */
	*p++ = 0x00; *p++ = 0x01;	/* Y density */
	*p++ = 0x00;			/* X thumbnail */
	*p++ = 0x00;			/* Y thumbnail */

	/* DQT (luma) */
	*p++ = 0xFF; *p++ = 0xDB;
	*p++ = 0x00; *p++ = 0x43;	/* length 67 */
	*p++ = 0x00;			/* Pq=0 (8-bit) | Tq=0 (luma) */
	for (i = 0; i < 64; i++)
		*p++ = (u8)q_luma[gemini_zigzag[i]];

	/* DQT (chroma) */
	*p++ = 0xFF; *p++ = 0xDB;
	*p++ = 0x00; *p++ = 0x43;
	*p++ = 0x01;			/* Pq=0 | Tq=1 (chroma) */
	for (i = 0; i < 64; i++)
		*p++ = (u8)q_chroma[gemini_zigzag[i]];

	/* DHT x 4: DC luma, AC luma, DC chroma, AC chroma */
	for (t = 0; t < 4; t++) {
		const struct gemini_huff_table *h = &gemini_huff_tables[t];
		u16 len = 2 + 1 + 16 + h->n_vals;
		/*
		 * Tc/Th byte: high nibble = class (0=DC, 1=AC),
		 * low nibble = destination id (0=luma, 1=chroma).
		 * Order: t=0 DC luma, t=1 DC chroma, t=2 AC luma, t=3 AC chroma.
		 */
		u8 tc = (t >= 2) ? 1 : 0;
		u8 th = (t & 1) ? 1 : 0;

		*p++ = 0xFF; *p++ = 0xC4;
		*p++ = (len >> 8) & 0xFF;
		*p++ = len & 0xFF;
		*p++ = (tc << 4) | th;
		memcpy(p, h->bits, 16); p += 16;
		memcpy(p, h->vals, h->n_vals); p += h->n_vals;
	}

	/* SOF0 (baseline) */
	*p++ = 0xFF; *p++ = 0xC0;
	*p++ = 0x00; *p++ = 0x11;	/* length 17 */
	*p++ = 0x08;			/* sample precision */
	*p++ = (height >> 8) & 0xFF; *p++ = height & 0xFF;
	*p++ = (width  >> 8) & 0xFF; *p++ = width  & 0xFF;
	*p++ = 0x03;			/* number of components */
	*p++ = 0x01; *p++ = 0x22; *p++ = 0x00;	/* Y:  H=2, V=2, Tq=0 */
	*p++ = 0x02; *p++ = 0x11; *p++ = 0x01;	/* Cb: H=1, V=1, Tq=1 */
	*p++ = 0x03; *p++ = 0x11; *p++ = 0x01;	/* Cr: H=1, V=1, Tq=1 */

	/* SOS */
	*p++ = 0xFF; *p++ = 0xDA;
	*p++ = 0x00; *p++ = 0x0C;	/* length 12 */
	*p++ = 0x03;			/* number of components */
	*p++ = 0x01; *p++ = 0x00;	/* Y:  Td=0, Ta=0 */
	*p++ = 0x02; *p++ = 0x11;	/* Cb: Td=1, Ta=1 */
	*p++ = 0x03; *p++ = 0x11;	/* Cr: Td=1, Ta=1 */
	*p++ = 0x00;			/* Ss=0 */
	*p++ = 0x3F;			/* Se=63 */
	*p++ = 0x00;			/* Ah=0, Al=0 */

	return p - dst;
}
