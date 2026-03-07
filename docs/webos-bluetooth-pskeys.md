# WebOS Bluetooth PSKEYs Documentation

## Overview

This document details all the PSKEY (Persistent Store Key) configurations used by webOS
for the BCM4329 Bluetooth chip on the HP TouchPad. Analysis based on decompiled
`libPmBtBsaif.so` from webOS 3.0.5.

**CRITICAL FINDING: WebOS uses `stores=8` (PSRAM/volatile) for ALL PSKEYs.**

This differs from the current driver which uses `stores=4` (PSI/persistent).

## PSKEY Storage Types

| Value | Name    | Description |
|-------|---------|-------------|
| 0x00  | DEFAULT | Default store |
| 0x01  | PS_I    | Persistent store (implementation) |
| 0x02  | PS_F    | Persistent store (factory) |
| 0x04  | PSI     | Persistent Store Implementation |
| 0x08  | PSRAM   | RAM (volatile) - **WebOS uses this** |
| 0x10  | PSNV    | Non-volatile |

## Initialization Sequence

WebOS uses the following initialization flow:
1. PmBtStart script reads BT address from `/dev/tokens/BToADDR`
2. PmBtStack launches with parameters: `-C /dev/ttyMSM1 -X <btaddr> -F 26000 -U 115200`
3. CsrBtPalmInitBootstrap sets frequency=26000 and ftrim=0x19
4. CsrTmBlueCoreGetBootstrap builds all PSKEY commands
5. All PSKEYs sent with stores=8 (PSRAM)
6. WARM_RESET (0x4002) sent to apply settings

## Common PSKEYs (All Devices)

These PSKEYs are sent for all BlueCore devices:

| PSKEY ID | Size (uint16) | Name/Purpose | Notes |
|----------|---------------|--------------|-------|
| 0x01FE   | 1             | Host interface config | Related to UART/BCSP |
| 0x01BE   | 1             | Deep sleep config | Conditional on interface type 1 or 7 |
| 0x01AB   | 1             | UART config | |
| 0x01B0   | 1             | UART config | |
| 0x01B9   | 1             | UART config | |
| 0x01F6   | 1             | UART baudrate | |
| 0x0011   | 1             | Ana freq (MHz) | Crystal frequency |
| 0x0013   | 1             | Ana ftrim | Crystal trim value |
| 0x024D   | 1             | Unknown | |
| 0x000E   | 1             | Unknown | |
| 0x01F9   | 1             | Unknown | |
| 0x025D   | 1             | Unknown | |
| 0x0001   | 4             | BD_ADDR | Bluetooth device address |

## TouchPad-Specific PSKEYs (LMP Subversion 0x12E9)

These additional PSKEYs are sent only when the LMP subversion is 0x12E9 (TouchPad):

### Configuration PSKEYs

| PSKEY ID | Size (uint16) | Name/Purpose |
|----------|---------------|--------------|
| 0x00F6   | 1             | Unknown config |
| 0x0203   | 32 (0x20)     | Unknown config |
| 0x0394   | 1             | Unknown config |
| 0x03AA   | 12 (0x0C)     | Unknown config |
| 0x03AB   | 12 (0x0C)     | Unknown config |
| 0x03D4   | 1             | Unknown config |

### RF Calibration PSKEYs

These PSKEYs contain RF calibration data critical for radio operation:

| PSKEY ID | Size (uint16) | Name/Purpose |
|----------|---------------|--------------|
| 0x212C   | 20 (0x14)     | RF calibration data |
| 0x212D   | 16 (0x10)     | RF calibration data |
| 0x212E   | 34 (0x22)     | RF calibration data |
| 0x212F   | 46 (0x2E)     | RF calibration data |
| 0x2130   | 15 (0x0F)     | RF calibration data |
| 0x2131   | 21 (0x15)     | RF calibration data |
| 0x2132   | 28 (0x1C)     | RF calibration data |
| 0x2133   | 39 (0x27)     | RF calibration data |
| 0x2134   | 40 (0x28)     | RF calibration data |
| 0x2135   | 24 (0x18)     | RF calibration data |
| 0x2136   | 31 (0x1F)     | RF calibration data |
| 0x2137   | 29 (0x1D)     | RF calibration data |
| 0x2138   | 13 (0x0D)     | RF calibration data |
| 0x2139   | 43 (0x2B)     | RF calibration data |
| 0x213A   | 4             | RF calibration data |
| 0x21E1   | 16 (0x10)     | RF calibration data |
| 0x2215   | 18 (0x12)     | RF calibration data |
| 0x2216   | 28 (0x1C)     | RF calibration data |
| 0x2227   | 63 (0x3F)     | RF calibration data |
| 0x2228   | 62 (0x3E)     | RF calibration data |
| 0x2229   | 29 (0x1D)     | RF calibration data |
| 0x222A   | 58 (0x3A)     | RF calibration data |
| 0x222B   | 26 (0x1A)     | RF calibration data |

**Total RF calibration data size: ~800 bytes**

## Palm Platform PSKEYs

These 12 additional PSKEYs are returned by `CsrBtPalmGetBootstrapKey()`:

The function returns PSKEYs from two arrays:
- `palmPlatformCommonPskeys` (60 bytes, 5 entries × 12 bytes each)
- `palmPlatformSpecificPskeys` (84 bytes, 7 entries × 12 bytes each)

Each entry has: PSKEY ID (2 bytes), data pointer (4 bytes), data length (2 bytes), reserved (4 bytes)

## BCCMD Commands

| Command | Value | Purpose |
|---------|-------|---------|
| SETREQ  | 0x02  | Set request |
| PS_KEY  | 0x7003 | Set PS key |
| WARM_RESET | 0x4002 | Warm reset (applies PSRAM config) |

## Key Functions in libPmBtBsaif.so

| Function | Address | Purpose |
|----------|---------|---------|
| CsrTmBlueCoreBuildBccmdPsSetMsg | 0x000d47d8 | Build PSKEY set message |
| CsrTmBlueCoreBuildBccmdSetMsg | 0x000d489c | Build BCCMD set message |
| CsrBuildPsKeyCommand | 0x000d4bf8 | Build PS key command |
| CsrTmBlueCoreGetBootstrap | 0x0007960c | Main bootstrap function |
| CsrBtPalmInitBootstrap | 0x0007a30c | Initialize bootstrap (freq=26000, ftrim=0x19) |
| CsrBtPalmGetBootstrapNumKeys | 0x0007a298 | Returns 12 (Palm-specific PSKEYs) |
| CsrBtPalmGetBootstrapKey | 0x0007a2a0 | Get Palm-specific PSKEY |

## Initialization Flow

```
CsrBtPalmInitBootstrap()
    ├── setBootstrapFrequency(26000)    // PSKEY 0x0011 = 26000
    └── setBootstrapFtrim(0x19)         // PSKEY 0x0013 = 0x19

CsrTmBlueCoreGetBootstrap(lmp_subversion, &num_keys)
    ├── PalmImportCsrBootstrap()        // Try to read ./csr.bt.bootstrap.import
    │
    ├── // Common PSKEYs (all devices)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x01fe, 8, 1, ...)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x01be, 8, 1, ...)  // conditional
    ├── ... (0x01ab, 0x01b0, 0x01b9, 0x01f6, 0x0011, 0x0013, 0x024d, 0x000e, 0x01f9, 0x025d)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x0001, 8, 4, BD_ADDR)
    │
    ├── // TouchPad-specific (if lmp_subversion == 0x12e9)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x00f6, 8, 1, ...)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x0203, 8, 0x20, ...)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x0394, 8, 1, ...)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x03aa, 8, 0x0c, ...)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x03ab, 8, 0x0c, ...)
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x03d4, 8, 1, ...)
    ├── // RF Calibration PSKEYs
    ├── CsrTmBlueCoreBuildBccmdPsSetMsg(0x212c-0x222b, 8, ..., calibration_data)
    │
    ├── // Palm platform PSKEYs (12 keys)
    ├── for each CsrBtPalmGetBootstrapKey():
    │       CsrTmBlueCoreBuildBccmdPsSetMsg(pskey, 8, len, data)
    │
    └── CsrTmBlueCoreBuildBccmdSetMsg(0x4002, 0, 0)  // WARM_RESET
```

## Total PSKEY Count

- Common PSKEYs: ~13
- TouchPad-specific config: 6
- RF Calibration: 23
- Palm platform: 12
- **Total: ~54 PSKEYs + WARM_RESET**

## Issue: Missing RF Calibration Data

The RF calibration data (PSKEYs 0x212c-0x222b) is compiled into the libPmBtBsaif.so binary.
This data is essential for proper RF operation. Without it, Bluetooth may initialize but
RF scanning/discovery will not work.

The calibration data is stored at a data offset referenced in `CsrTmBlueCoreGetBootstrap()`
at `iVar12 = iVar8 + DAT_0007a148`. The exact values need to be extracted from the binary
or captured during webOS Bluetooth initialization.

## Driver Implementation Notes

To match webOS behavior, the Linux driver should:

1. **Use stores=8 (PSRAM)** instead of stores=4 (PSI)
2. Send ALL required PSKEYs, not just a subset
3. Include the RF calibration data for TouchPad (PSKEYs 0x212c-0x222b)
4. Set crystal frequency to 26000 MHz (PSKEY 0x0011)
5. Set ftrim to 0x19 (PSKEY 0x0013)
6. Send BD_ADDR as PSKEY 0x0001 (4 uint16 values)
7. End with WARM_RESET (0x4002) command

## Known PSKEY Values

### Crystal Configuration
- PSKEY 0x0011 (ANA_FREQ): 26000 (0x6590 in hex, but as decimal 26000)
- PSKEY 0x0013 (ANA_FTRIM): 0x0019

### Baudrate Configuration (in setBootstrapBaudrate)
| Baudrate | PSKEY value |
|----------|-------------|
| 9600     | 0x009D |
| 19200    | 0x004F |
| 38400    | 0x0027 |
| 57600    | 0x00EC |
| 115200   | 0x01D8 |
| 230400   | 0x03B0 |
| 460800   | 0x075F |
| 921600   | 0x0EBF |
| 1382400  | 0x161E |
| 2000000  | 0x20C5 |
| 3000000  | 0x3000 |
| 3072000  | 0x3127 |
| 3686400  | 0x3AFC |
| 3750000  | 0x3C00 |

## References

- webOS 3.0.5 rootfs: `/usr/lib/libPmBtBsaif.so`
- webOS Bluetooth startup: `/usr/bin/PmBtStart`
- Token database: partition 12, offset 0x50b0
- BToADDR token: "00:1D:FE:85:64:A9" (Palm-assigned)
