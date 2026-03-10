// SPDX-License-Identifier: GPL-2.0-only
#include "a6_host_adapter.h"
#include "low_level_funcs.h"

u8 tdo_bit;		/* holds the value of TDO-bit */
u8 TCLK_saved = 1;	/* holds the last value of TCLK before entering a JTAG sequence */

/*
 * Function declarations which have to be programmed by the user for use
 * with hosts other than the MSP430F149.
 *
 * The following MSP430F149-specific code can be used as a reference as to
 * how to implement the required JTAG communication on additional hosts.
 */

/* combinations of sbw-cycles (TMS, TDI, TDO) */
void TMSL_TDIL(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSL;
	TDIL;
	TDOsbw;
	EnableInterrupts(flags);
}

void TMSH_TDIL(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSH;
	TDIL;
	TDOsbw;
	EnableInterrupts(flags);
}

void TMSL_TDIH(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSL;
	TDIH;
	TDOsbw;
	EnableInterrupts(flags);
}

void TMSH_TDIH(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSH;
	TDIH;
	TDOsbw;
	EnableInterrupts(flags);
}

void TMSL_TDIH_TDOrd(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSL;
	TDIH;
	TDO_RD;
	EnableInterrupts(flags);
}

void TMSL_TDIL_TDOrd(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSL;
	TDIL;
	TDO_RD;
	EnableInterrupts(flags);
}

void TMSH_TDIH_TDOrd(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSH;
	TDIH;
	TDO_RD;
	EnableInterrupts(flags);
}

void TMSH_TDIL_TDOrd(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	TMSH;
	TDIL;
	TDO_RD;
	EnableInterrupts(flags);
}

/* enters with TCLK_saved and exits with TCLK = 0 */
void ClrTCLK_sbw(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	if (TCLK_saved)
		TMSLDH;
	else
		TMSL;

	ClrSBWTDIO();
	TDIL;
	TDOsbw;
	TCLK_saved = 0;
	EnableInterrupts(flags);
}

/* enters with TCLK_saved and exits with TCLK = 1 */
void SetTCLK_sbw(void)
{
	unsigned long flags = 0;

	DisableInterrupts(flags);
	if (TCLK_saved)
		TMSLDH;
	else
		TMSL;

	SetSBWTDIO();
	TDIH;
	TDOsbw;
	TCLK_saved = 1;
	EnableInterrupts(flags);
}

/*
 * Shift a value into TDI (MSB first) and simultaneously shift out a value
 * from TDO (MSB first).
 * Arguments: word Format (number of bits shifted, 8 (F_BYTE), 16 (F_WORD),
 *            20 (F_ADDR) or 32 (F_LONG))
 *            unsigned long Data (data to be shifted into TDI)
 * Result:    unsigned long (scanned TDO value)
 */
unsigned long AllShifts(u16 Format, unsigned long Data)
{
	unsigned long TDOword = 0x00000000;
	unsigned long MSB = 0x00000000;
	u16 i;

	switch (Format) {
	case F_BYTE:
		MSB = 0x00000080;
		break;
	case F_WORD:
		MSB = 0x00008000;
		break;
	case F_ADDR:
		MSB = 0x00080000;
		break;
	case F_LONG:
		MSB = 0x80000000;
		break;
	default:
		/* unsupported format, function will just return 0 */
		return TDOword;
	}

	/* shift in bits */
	for (i = Format; i > 0; i--) {
		/* last bit requires TMS=1; TDO one bit before TDI */
		if (i == 1) {
			if ((Data & MSB) == 0)
				TMSH_TDIL_TDOrd();
			else
				TMSH_TDIH_TDOrd();
		} else {
			if ((Data & MSB) == 0)
				TMSL_TDIL_TDOrd();
			else
				TMSL_TDIH_TDOrd();
		}
		Data <<= 1;
		if (tdo_bit)
			TDOword++;
		if (i > 1)
			TDOword <<= 1;
	}

	TMSH_TDIH();	/* update IR */
	if (TCLK_saved)
		TMSL_TDIH();
	else
		TMSL_TDIL();

	/* de-scramble bits on a 20bit shift */
	if (Format == F_ADDR)
		TDOword = ((TDOword << 16) + (TDOword >> 4)) & 0x000FFFFF;

	return TDOword;
}

void DrvSignals(void)
{
	SetSBWTDIO();
	ClrSBWTCK();
}

void RlsSignals(void)
{
	SetSBWTDIO();
	ClrSBWTCK();
}

void InitTarget(void)
{
	DrvSignals();
}

void ReleaseTarget(void)
{
	RlsSignals();
}
