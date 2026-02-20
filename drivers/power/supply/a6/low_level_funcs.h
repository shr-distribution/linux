/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LOW_LEVEL_FUNCS_H_
#define _LOW_LEVEL_FUNCS_H_

#include <linux/types.h>

/*
 * Macros and Pin-to-Signal assignments which have to be programmed
 * by the user. This implementation assumes use of an MSP430F149 as the host
 * controller and the corresponding hardware given in the application
 * report TBD Appendix A.
 *
 * The following MSP430 example acts as a hint of how to generally
 * implement a micro-controller programmer solution for the MSP430 flash-
 * based devices.
 */

/* Pin-to-Signal Assignments */

#define TMSH	do { SetSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; SetSBWTCK(); } while (0)
#define TMSL	do { ClrSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; SetSBWTCK(); } while (0)
#define TMSLDH	do { ClrSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; SetSBWTDIO(); SetSBWTCK(); } while (0)
#define TDIH	do { SetSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; SetSBWTCK(); } while (0)
#define TDIL	do { ClrSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; SetSBWTCK(); } while (0)
#define TDOsbw	do { SetSBWTDIO(); SetInSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; SetSBWTCK(); SetOutSBWTDIO(); } while (0)
#define TDO_RD	do { SetSBWTDIO(); SetInSBWTDIO(); nNOPS; ClrSBWTCK(); nNOPS; tdo_bit = GetSBWTDIO(); SetSBWTCK(); SetOutSBWTDIO(); } while (0)

void ClrTCLK_sbw(void);
void SetTCLK_sbw(void);

#define ClrTCLK()	ClrTCLK_sbw()
#define SetTCLK()	SetTCLK_sbw()

#define SetRST()	SetSBWTDIO()
#define ClrRST()	ClrSBWTDIO()
#define ReleaseRST()	do { } while (0)
#define SetTST()	SetSBWTCK()
#define ClrTST()	ClrSBWTCK()

/* Definition of global variables */
extern u8 TCLK_saved;

/* Function pointers for SBW operations - set by platform code */
extern uint16_t (*SetSBWTCK)(void);
extern uint16_t (*ClrSBWTCK)(void);
extern uint16_t (*SetSBWTDIO)(void);
extern uint16_t (*ClrSBWTDIO)(void);
extern uint16_t (*SetInSBWTDIO)(void);
extern uint16_t (*SetOutSBWTDIO)(void);
extern uint16_t (*GetSBWTDIO)(void);
extern uint16_t (*SetSBWAKEUP)(void);
extern uint16_t (*ClrSBWAKEUP)(void);
extern void (*delay)(uint32_t delay_us);

/* Low Level function prototypes */

void TMSL_TDIL(void);
void TMSH_TDIL(void);
void TMSL_TDIH(void);
void TMSH_TDIH(void);
void TMSL_TDIH_TDOrd(void);
void TMSL_TDIL_TDOrd(void);
void TMSH_TDIH_TDOrd(void);
void TMSH_TDIL_TDOrd(void);

unsigned long AllShifts(u16 Format, unsigned long Data);
void DrvSignals(void);
void RlsSignals(void);
void InitTarget(void);
void ReleaseTarget(void);

#endif
