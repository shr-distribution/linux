# HP TouchPad RF Calibration Data Extraction

## Status: EXTRACTED

The TouchPad-specific RF calibration PSKEYs (0x212C-0x222B) have been successfully
extracted from `libPmBtBsaif.so` using PIC (position-independent code) analysis.

**Extraction Method:**
1. Located CsrTmBlueCoreGetBootstrap function at 0x6960c
2. Traced literal pool references to find PC-relative offsets
3. Calculated data addresses: iVar8 = 0x69620 + 0x7a09c = 0xE36BC
4. TouchPad data at: iVar8 + (-54292) = 0xD62A8
5. Extracted all 29 PSKEYs totaling ~1500 bytes

## TouchPad-Specific PSKEYs (LMP subversion 0x12E9)

These PSKEYs are only sent when the Bluetooth chip reports LMP subversion 0x12E9:

### Configuration PSKEYs

| PSKEY ID | Length (words) | Description |
|----------|----------------|-------------|
| 0x00F6   | 1              | Unknown config |
| 0x0203   | 32             | Unknown config (64 bytes) |
| 0x0394   | 1              | Unknown config |
| 0x03AA   | 12             | Unknown config (24 bytes) |
| 0x03AB   | 12             | Unknown config (24 bytes) |
| 0x03D4   | 1              | Unknown config |

### RF Calibration PSKEYs

| PSKEY ID | Length (words) | Description |
|----------|----------------|-------------|
| 0x212C   | 20             | RF calibration |
| 0x212D   | 16             | RF calibration |
| 0x212E   | 34             | RF calibration |
| 0x212F   | 46             | RF calibration |
| 0x2130   | 15             | RF calibration |
| 0x2131   | 21             | RF calibration |
| 0x2132   | 28             | RF calibration |
| 0x2133   | 39             | RF calibration |
| 0x2134   | 40             | RF calibration |
| 0x2135   | 24             | RF calibration |
| 0x2136   | 31             | RF calibration |
| 0x2137   | 29             | RF calibration |
| 0x2138   | 13             | RF calibration |
| 0x2139   | 43             | RF calibration |
| 0x213A   | 4              | RF calibration |
| 0x21E1   | 16             | RF calibration |
| 0x2215   | 18             | RF calibration |
| 0x2216   | 28             | RF calibration |
| 0x2227   | 63             | RF calibration |
| 0x2228   | 62             | RF calibration |
| 0x2229   | 29             | RF calibration |
| 0x222A   | 58             | RF calibration |
| 0x222B   | 26             | RF calibration |

**Total RF calibration data: ~1500 bytes (750 uint16 values)**

## Extracted Data Location

The extracted PSKEY data is available in:
- `drivers/bluetooth/hci_bcsp_touchpad_pskeys.h`

This header file contains:
- All 29 TouchPad-specific PSKEYs as static const u16 arrays
- A structured table for easy iteration during BCCMD transmission
- Proper SPDX license header

## Extraction Technical Details

The webOS binary uses ARM position-independent code (PIC):

```
69614:  ldr r2, [pc, #2832]     @ literal pool at 0x6a12c
69618:  add r2, pc, r2          @ r2 = base pointer for data
```

Literal pool values:
- 0x6a12c: 0x0007a09c (base offset)
- 0x6a148: 0xffff2bec (-54292, TouchPad data offset)

Address calculation:
```
base = PC(0x69620) + 0x7a09c = 0xE36BC
touchpad_data = 0xE36BC + (-54292) = 0xD62A8
```

The TouchPad data block starts at file offset 0xD62A8 in libPmBtBsaif.so.

## Current Driver Status

The Linux hci_bcsp driver now has access to:
- Common PSKEYs (ANA_FREQ=26000, ANA_FTRIM=0x19, etc.)
- Palm Platform PSKEYs (0x01B3, 0x01B6, 0x01BF, 0x01BA, 0x01C7, 0x01CA, 0x001D)
- TX Power Table (30 words)
- BD Address
- **TouchPad-specific PSKEYs (29 entries)** - NEW
- WARM_RESET

The TouchPad RF calibration PSKEYs should improve:
- RF sensitivity
- TX power accuracy
- Frequency stability

## Integration Notes

To use the TouchPad PSKEYs in the driver:

1. Include the header:
   ```c
   #include "hci_bcsp_touchpad_pskeys.h"
   ```

2. Check LMP subversion during initialization:
   ```c
   if (lmp_subversion == 0x12E9) {
       /* Send TouchPad-specific PSKEYs */
       for (i = 0; i < TOUCHPAD_PSKEYS_COUNT; i++) {
           bcsp_send_pskey(bcsp,
               touchpad_pskeys[i].pskey_id,
               touchpad_pskeys[i].stores,
               touchpad_pskeys[i].length,
               touchpad_pskeys[i].data);
       }
   }
   ```

## References

- Decompiled code: `/home/herrie/webos/touchpad-kernel/doctor305/bt_decompiled.c`
- webOS binary: `/usr/lib/libPmBtBsaif.so`
- PSKEY documentation: `docs/webos-bluetooth-pskeys.md`
