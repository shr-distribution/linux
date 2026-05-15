# HTC TrustZone CE2 Comprehensive Analysis
## Date: 2026-05-15 (Deep Dive #2)

## Objective
Complete analysis of HTC TZ firmware to identify ALL initialization steps for CE2 and related peripherals (ADM DMA, clocks, interrupts).

## GCC Clock Register Writes Found

### Location: 0xd120 - Generic Clock Enable Function

```asm
d120:  40 f0 01 00   orr.w   r0, r0, #1      ; Set bit 0 (root enable)
d124:  00 90         str     r0, [sp, #0]
d126:  00 98         ldr     r0, [sp, #0]
d128:  40 f0 02 00   orr.w   r0, r0, #2      ; Set bit 1 (branch enable)
d12c:  00 90         str     r0, [sp, #0]
d12e:  00 98         ldr     r0, [sp, #0]
d130:  20 f0 18 00   bic.w   r0, r0, #24     ; Clear bits 3-4
d134:  08 30         adds    r0, #8          ; Set bit 3
d136:  00 90         str     r0, [sp, #0]
d13a:  20 f0 20 00   bic.w   r0, r0, #32     ; Clear bit 5
d142:  20 f0 40 00   bic.w   r0, r0, #64     ; Clear bit 6
```

**Pattern**: 
- Sets bits: 0, 1, 3 (enable flags)
- Clears bits: 4, 5, 6 (disable/reset flags)

**Note**: This does NOT match CE2_HCLK_CTL pattern (bit 4=enable). This is likely for DIFFERENT clocks.

### GCC Base Address References

Found at offset 0x6cf0:
```
d9 f8 00 00 00 90 00 28    ; Load 0x00900000 (GCC base)
```

## QCE Peripheral Init Sequence

### Location: 0x13248 - QCE Status Polling

```asm
13248:  f8d9 0020    ldr.w   r0, [r9, #32]      ; r9 = QCE base (0x18500000)
1324c:  0740         lsls    r0, r0, #29        ; Test bit 3
1324e:  d5fb         bpl.n   0x13248            ; Loop if clear
```

Polls **QCE+0x20** (STATUS register) for **bit 3** (DOUT_RDY).

### Location: 0x1325c - QCE Config Write

```asm
13250:  2204         movs    r2, #4             ; 4 bytes
13252:  4629         mov     r1, r5             ; Source = data table
13254:  4668         mov     r0, sp             ; Dest = stack
13256:  f000 f97d    bl      0x13554            ; memcpy
1325a:  9800         ldr     r0, [sp, #0]       ; Load value
1325c:  f8c9 0000    str.w   r0, [r9]           ; Write to QCE+0x00
```

Writes configuration value from table to **QCE+0x00** (DATA_IN).

### Location: 0x1326a - QCE Completion Polling

```asm
1326a:  f8d9 0020    ldr.w   r0, [r9, #32]      ; Load QCE+0x20
1326e:  0700         lsls    r0, r0, #28        ; Test bit 4
13270:  d5fb         bpl.n   0x1326a            ; Loop if clear
```

Polls **QCE+0x20** for **bit 4** (ERR_INTR or completion).

### Location: 0x13272 - QCE Result Read

```asm
13272:  f8d9 0010    ldr.w   r0, [r9, #16]      ; Read QCE+0x10
```

Reads result from **QCE+0x10** (DATA_OUT).

## Critical Finding: This is a CRYPTO TEST, not init!

The HTC TZ sequence is **testing** the crypto engine by:
1. Writing test data to QCE
2. Waiting for processing  
3. Reading result

This is NOT required initialization - it's validation!

## What We're Actually Missing

The CE2_HCLK_CTL clock enable (bit 4) is NOT done by this generic function at 0xd120.
It must be done elsewhere OR by a different mechanism.

Let me search for the actual CE2_HCLK_CTL offset (0x2740)...


## KEY FINDING: No CE2_HCLK_CTL Write Found

Searched entire HTC TZ firmware for offset 0x2740 - **NOT FOUND**.

**Conclusion**: HTC TrustZone does NOT manually enable CE2 clock via GCC register write.
The clock must be enabled by:
1. Earlier bootloader stage (OEMSBL/SBL)
2. Hardware default state
3. Device tree / Linux CCF

## What HTC TZ Actually Does

1. **Clock Management** (0xd120): Generic function for OTHER peripherals, not CE2
2. **QCE Test Sequence** (0x13248): Validates crypto engine works, not initialization
3. **No ADM DMA Init**: No references to ADM base addresses found
4. **No CRCI Configuration**: No flow control setup

## TouchPad vs HTC Difference

**HTC Devices**: Have complete bootloader chain (OEMSBL → APPSBL → TZ → Linux)
**TouchPad**: Missing OEMSBL stage (partition is empty), goes directly APPSBL → Linux

**Impact**: Whatever OEMSBL does for CE2/ADM is MISSING on TouchPad!

## What We Need To Find

1. OEMSBL-level CE2 initialization
2. ADM DMA controller setup
3. Interrupt routing/enabling  
4. CRCI mux configuration

The TZ firmware assumes these are already done.

