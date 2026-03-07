# WebOS Bluetooth PSKEYs Documentation

## Overview

This document details all the PSKEY (Persistent Store Key) configurations used by webOS
for the BCM4329 Bluetooth chip on the HP TouchPad. Analysis based on decompiled
`libPmBtBsaif.so` and `PmBtStack` from webOS 3.0.5.

**CRITICAL FINDING: WebOS uses `stores=8` (PSRAM/volatile) for ALL PSKEYs.**

## PSKEY Storage Types

| Value | Name    | Description |
|-------|---------|-------------|
| 0x00  | DEFAULT | Default store |
| 0x04  | PSI     | Persistent Store Implementation |
| 0x08  | PSRAM   | RAM (volatile) - **WebOS uses this** |

## Complete PSKEY List

### Section 1: Common PSKEYs (All Devices)

These PSKEYs are sent for all BlueCore devices:

| PSKEY ID | Size | Value | Description |
|----------|------|-------|-------------|
| 0x0001   | 4    | BD_ADDR | Bluetooth device address (from tokens) |
| 0x000E   | 1    | 0x0001 | Unknown |
| 0x0011   | 1    | 26000 (0x6590) | ANA_FREQ - Crystal frequency (26 MHz) |
| 0x0013   | 1    | 0x0019 | ANA_FTRIM - Crystal trim |
| 0x01AB   | 1    | 0x03ec (1004) | UART config |
| 0x01B0   | 1    | 0x0000 | UART config |
| 0x01B9   | 1    | 0x01d8 (472) | UART baudrate divisor (115200) |
| 0x01BE   | 1    | 0x0000 | Deep sleep config (conditional) |
| 0x01F6   | 1    | varies | UART baudrate |
| 0x01F9   | 1    | 0x0001 | Unknown |
| 0x01FE   | 1    | 0x0001 | Host interface config |
| 0x024D   | 1    | varies | Unknown |
| 0x025D   | 1    | 0x0001 | Unknown |

### Section 2: Palm Platform PSKEYs (12 entries)

#### palmPlatformCommonPskeys (5 entries)

| PSKEY ID | Size | Data (uint16 values) |
|----------|------|----------------------|
| 0x01B3   | 4    | `{ 0x08a0, 0x0016, 0x0060, 0x082e }` |
| 0x01B6   | 2    | `{ 0x0060, 0x082e }` |
| 0x01BF   | 2    | `{ 0x082e, 0x0000 }` |
| 0x01F7   | 4    | (runtime data) |
| 0x01F8   | 4    | (runtime data) |

#### palmPlatformSpecificPskeys (7 entries)

| PSKEY ID | Size | Data (uint16 values) | Description |
|----------|------|----------------------|-------------|
| 0x01BA   | 4    | `{ 0x1002, 0x0177, 0x0001, 0x03e8 }` | Config |
| 0x01C7   | 8    | `{ 0x0001, 0x03e8, 0x000a, 0x0064, 0x0031, 0x0004, 0x0004, 0x2200 }` | Config |
| 0x01CA   | 2    | `{ 0x0031, 0x0004 }` | Config |
| 0x0021   | 2    | `{ 0x0004, 0x0004 }` | Default TX Power |
| 0x0017   | 2    | `{ 0x0004, 0x2200 }` | Max TX Power |
| 0x0031   | 30   | See Power Table below | Power Table |
| 0x001D   | 2    | `{ 0x2410, 0x08a0 }` | Config |

**Power Table (PSKEY 0x0031) - 30 uint16 values:**
```c
static const u16 palm_power_table[] = {
    /* Entry 0: power=0x22, freq_off=0x50, ... */
    0x2200, 0x0050, 0x2600, 0x0050, 0xf000,
    /* Entry 1 */
    0x2800, 0x0050, 0x2d00, 0x0050, 0xf400,
    /* Entry 2 */
    0x2500, 0x0040, 0x2a00, 0x0040, 0xf800,
    /* Entry 3 */
    0x2200, 0x0020, 0x2700, 0x0020, 0xfc00,
    /* Entry 4 */
    0x2600, 0x0010, 0x2c00, 0x0010, 0x0000,
    /* Entry 5 */
    0x2c00, 0x0000, 0x3a00, 0x0000, 0x0400
};
```

### Section 3: TouchPad-Specific PSKEYs (LMP Subversion 0x12E9)

These PSKEYs are sent only when LMP subversion is 0x12E9 (TouchPad):

#### Configuration PSKEYs

| PSKEY ID | Size | Description |
|----------|------|-------------|
| 0x00F6   | 1    | Unknown config |
| 0x0203   | 32   | Unknown config (64 bytes) |
| 0x0394   | 1    | Unknown config |
| 0x03AA   | 12   | Unknown config (24 bytes) |
| 0x03AB   | 12   | Unknown config (24 bytes) |
| 0x03D4   | 1    | Unknown config |

#### RF Calibration PSKEYs (0x212C-0x222B)

These contain device-specific RF calibration data:

| PSKEY ID | Size (uint16) | Size (bytes) |
|----------|---------------|--------------|
| 0x212C   | 20            | 40           |
| 0x212D   | 16            | 32           |
| 0x212E   | 34            | 68           |
| 0x212F   | 46            | 92           |
| 0x2130   | 15            | 30           |
| 0x2131   | 21            | 42           |
| 0x2132   | 28            | 56           |
| 0x2133   | 39            | 78           |
| 0x2134   | 40            | 80           |
| 0x2135   | 24            | 48           |
| 0x2136   | 31            | 62           |
| 0x2137   | 29            | 58           |
| 0x2138   | 13            | 26           |
| 0x2139   | 43            | 86           |
| 0x213A   | 4             | 8            |
| 0x21E1   | 16            | 32           |
| 0x2215   | 18            | 36           |
| 0x2216   | 28            | 56           |
| 0x2227   | 63            | 126          |
| 0x2228   | 62            | 124          |
| 0x2229   | 29            | 58           |
| 0x222A   | 58            | 116          |
| 0x222B   | 26            | 52           |

**Total RF calibration data: ~1500 bytes**

**STATUS:** Values embedded in libPmBtBsaif.so, need runtime capture.

### Section 4: BD_ADDR (PSKEY 0x0001)

Read from token database at runtime:
- Token file: `/dev/tokens/BToADDR`
- Partition: 12, Offset: 0x50b0
- Example: `00:1D:FE:85:64:A9`

Format for PSKEY 0x0001 (4 uint16 values):
```
{ LAP[15:0], UAP<<8 | LAP[23:16], NAP[7:0], NAP[15:8] }
```

For 00:1D:FE:85:64:A9:
```c
static const u16 bd_addr[] = { 0x64A9, 0xFE85, 0x001D, 0x0000 };
```

## BCCMD Commands

| Command | Value  | Description |
|---------|--------|-------------|
| SETREQ  | 0x02   | Set request |
| PS_KEY  | 0x7003 | Set PS key |
| WARM_RESET | 0x4002 | Apply PSRAM config |

## Initialization Flow

```
CsrBtPalmInitBootstrap()
    ├── setBootstrapFrequency(26000)    // PSKEY 0x0011 = 26000
    └── setBootstrapFtrim(0x19)         // PSKEY 0x0013 = 0x19

CsrTmBlueCoreGetBootstrap(lmp_subversion, &num_keys)
    ├── // Common PSKEYs (stores=8)
    ├── PSKEY 0x01FE, 0x01BE, 0x01AB, 0x01B0, 0x01B9, 0x01F6
    ├── PSKEY 0x0011 (freq), 0x0013 (ftrim)
    ├── PSKEY 0x024D, 0x000E, 0x01F9, 0x025D
    ├── PSKEY 0x0001 (BD_ADDR)
    │
    ├── // TouchPad-specific (if lmp_subversion == 0x12e9)
    ├── PSKEY 0x00F6, 0x0203, 0x0394, 0x03AA, 0x03AB, 0x03D4
    ├── RF Calibration: 0x212C through 0x222B
    │
    ├── // Palm platform PSKEYs (12 keys)
    ├── palmPlatformCommonPskeys (5)
    ├── palmPlatformSpecificPskeys (7)
    │
    └── BCCMD 0x4002 (WARM_RESET)
```

## Total PSKEY Count

- Common PSKEYs: 13
- Palm Common: 5
- Palm Specific: 7
- TouchPad Config: 6
- RF Calibration: 23
- **Total: ~54 PSKEYs + WARM_RESET**

## Extracting RF Calibration Data

The RF calibration data is embedded in libPmBtBsaif.so using position-independent
code. To extract the actual values:

1. Boot into webOS
2. Check for `./csr.bt.bootstrap.export` after BT init
3. Or intercept BCCMD packets on /dev/ttyMSM1

## Driver Implementation

To match webOS behavior, the Linux driver needs:

1. **Use PSRAM storage (stores=8)**
2. **Set crystal config:**
   ```c
   PSKEY 0x0011 = 26000  // 26 MHz
   PSKEY 0x0013 = 0x19   // ftrim
   ```
3. **Send BD_ADDR as PSKEY 0x0001**
4. **Include Palm platform PSKEYs**
5. **Send WARM_RESET (0x4002) at end**

### C Code for Extracted PSKEYs

```c
/* Palm Platform Common PSKEYs */
static const u16 pskey_01b3[] = { 0x08a0, 0x0016, 0x0060, 0x082e };
static const u16 pskey_01b6[] = { 0x0060, 0x082e };
static const u16 pskey_01bf[] = { 0x082e, 0x0000 };

/* Palm Platform Specific PSKEYs */
static const u16 pskey_01ba[] = { 0x1002, 0x0177, 0x0001, 0x03e8 };
static const u16 pskey_01c7[] = { 0x0001, 0x03e8, 0x000a, 0x0064,
                                  0x0031, 0x0004, 0x0004, 0x2200 };
static const u16 pskey_01ca[] = { 0x0031, 0x0004 };
static const u16 pskey_0021[] = { 0x0004 };  /* Default TX Power */
static const u16 pskey_0017[] = { 0x0004 };  /* Max TX Power */
static const u16 pskey_001d[] = { 0x2410, 0x08a0 };

/* Power Table (PSKEY 0x0031) */
static const u16 pskey_0031[] = {
    0x2200, 0x0050, 0x2600, 0x0050, 0xf000,
    0x2800, 0x0050, 0x2d00, 0x0050, 0xf400,
    0x2500, 0x0040, 0x2a00, 0x0040, 0xf800,
    0x2200, 0x0020, 0x2700, 0x0020, 0xfc00,
    0x2600, 0x0010, 0x2c00, 0x0010, 0x0000,
    0x2c00, 0x0000, 0x3a00, 0x0000, 0x0400
};
```

## References

- webOS 3.0.5 rootfs: `/usr/lib/libPmBtBsaif.so`
- webOS Bluetooth binary: `/usr/bin/PmBtStack`
- webOS Bluetooth startup: `/usr/bin/PmBtStart`
- Token database: partition 12, offset 0x50b0
- BToADDR token: "00:1D:FE:85:64:A9"
- LMP Subversion for TouchPad: 0x12E9
