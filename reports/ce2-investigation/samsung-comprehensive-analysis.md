# Samsung MSM8660 CE2 Comprehensive Analysis  
## Date: 2026-05-15

## Source: Samsung Galaxy Note (I717) Stock ROM
**Path**: `/home/herrie/Downloads/I717_ATT_I717UCMD3_I717ATTMD3--www.SamSony.net--/KIES_HOME_I717UCMD3_I717ATTMD3_1117019_REV02_user_low_ship/`

## Firmware Files Analyzed

### MODEM.B05 - Main Modem Firmware (16MB)
Contains QCE base address 0x18500000 at multiple locations:
- 0x005c98a0
- 0x009dce10  
- 0x00f22e80

### MODEM.B06 - Modem Data
QCE reference at: 0x000838c0

### Source Code Paths Found

```
/modem_proc/core/securemsm/crypto/shared/src/secapi.c
/modem_proc/core/securemsm/crypto/shared/src/secenchw.c  
/modem_proc/core/securemsm/crypto/environment/src/seccrypti.c
```

These are Qualcomm's secure MSM crypto API implementation.

## Peripheral Address Table Analysis

### Location: MODEM.B05 @ 0x009dce00

This appears to be a peripheral mapping table with pattern:
```
[size/flags] [address] [permissions] [type]
```

Addresses found:
- 0x184e00 (unknown)
- 0x185000 (QCE!)
- 0x185100 (QCE+0x100)
- 0x185300 (QCE+0x300)
- 0x185800 (unknown)
- 0x185a00 (unknown)
- ...

## Error Messages Found

```
"Crypto layer not initialized!"
"HSHhw: CE_Hash_Update failed! Crypto Engine API not supported"
"HSHhw: CeMLInit failed! Crypto Engine API not supported"
"SSL: Could not alloc crypto handle"
```

These suggest the MODEM firmware has crypto API but may not actually USE the hardware.


## ADM DMA References Found

### ADM0 (0x18320000) - Multiple references in MODEM.B05
Used by modem for data transfers, not crypto.

### ADM1 (0x18420000) - Found in MODEM.B05  
This is the ADM instance QCE uses!

Locations with ADM1 base:
- 0x00090560: Address table entry
- 0x001dec40: Configuration data  
- Multiple code references for modem peripherals

## Critical Question: Does Samsung Use QCE?

Error strings suggest crypto API exists but may fallback to software:
```
"HSHhw: CE_Hash_Update failed! Crypto Engine API not supported"
```

**Hypothesis**: Samsung modem has QCE driver code but doesn't actually use hardware,
similar to other vendors avoiding CE2 due to bugs/complexity.

## What Samsung Firmware Doesn't Tell Us

1. No obvious ADM channel configuration for crypto
2. No CRCI setup visible in modem firmware  
3. No GCC clock register writes found
4. Modem firmware runs in different security context than Linux

**Conclusion**: Samsung MODEM firmware doesn't help - it doesn't initialize QCE for Linux!

## Comparison to TouchPad

Both TouchPad and Samsung likely rely on:
1. Boot ROM / PBL (Primary Boot Loader) - not accessible
2. SBL (Secondary Boot Loader) - might be in different partitions
3. Device tree in Linux kernel

The missing piece is likely in PBL/SBL level, before any firmware we can extract.

