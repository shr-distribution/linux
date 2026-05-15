# TouchPad Partition Layout Verification
## Official webOS Doctor topaz.xml vs Extracted Partitions

## Official Partition Table (from topaz.xml)

| Partition | Type | Size | Description | File in webOS Doctor |
|-----------|------|------|-------------|---------------------|
| p1 | FAT (0x0c) | 1024 KB | Boot FAT | fat.bin |
| p2 | CFG_DATA (0x4d) | 500 KB | RPM Secondary Bootloader | rpmsbl.mbn |
| p3 | SPBL (0x51) | 1500 KB | Secondary Program Boot Loader | spbl.mbn |
| **Extended partition p4** | - | - | - | - |
| p5 | APPSSBL (0x47) | 500 KB | RPM firmware | rpm.mbn |
| p6 | QCSBL (0x45) | 500 KB | Qualcomm Secondary Bootloader | ssbl.mbn |
| p7 | EFS2 (0x4e) | 65024 KB (~63.5 MB) | EFS2 filesystem | - |
| p8 | FOTA (0x4c) | 2500 KB | APPSBL bootloader | emmc_appsboot.mbn |
| p9 | APPS (0x48) | 10240 KB (10 MB) | Android boot.img | boot.img |
| p10 | OEMSBL (0x46) | 500 KB | **TrustZone** | tz.mbn |
| p11 | MODEM_ST1 (0x4a) | 3072 KB (3 MB) | Modem state 1 | - |
| p12 | MODEM_ST2 (0x4b) | 3072 KB (3 MB) | Modem state 2 | - |
| p13 | NVRAM | 4 MB | moboot environment + logos | - |
| p14 | ext3 | 32 MB | /boot mount | - |
| p15 | LVM PV | Rest of disk | LVM physical volume | - |

## Partition Type Codes

From topaz.xml partition type codes:
- `0x0c` = FAT32 LBA
- `0x4d` = CFG_DATA (Custom)
- `0x51` = SPBL (Custom)
- `0x47` = APPSSBL/RPM (Custom)
- `0x45` = QCSBL (Custom)
- `0x4e` = EFS2 (Custom)
- `0x4c` = FOTA (Custom)
- `0x48` = APPS (Custom)
- `0x46` = OEMSBL/TZ (Custom)
- `0x4a` = MODEM_ST1 (Custom)
- `0x4b` = MODEM_ST2 (Custom)

## Our Extraction vs Official Layout

### What We Extracted Previously

```bash
/tmp/touchpad-p2.bin  # CFG_DATA (500KB) ✅
/tmp/touchpad-p3.bin  # SPBL (1.5MB) ✅
/tmp/touchpad-p5.bin  # RPM (500KB) ✅
/tmp/touchpad-p6.bin  # QCSBL (500KB) ✅
/tmp/touchpad-p7.bin  # APPSBL (2.5MB) ✅
/tmp/touchpad-p9.bin  # APPS (10MB) ✅
/tmp/tz-touchpad.mbn  # TZ (500KB) ✅ (but EMPTY!)
```

### Corrections Needed

**p7 confusion**: We extracted `/tmp/touchpad-p7.bin` thinking it was a bootloader, but:
- Official layout shows p7 = EFS2 (63.5 MB filesystem)
- Official layout shows p8 = FOTA/APPSBL (2.5 MB bootloader)

**We likely extracted p8, not p7!**

Let me verify actual partition numbers on the device.

## Bootloader File Name Mapping

| webOS Doctor File | Description | Size | Goes to Partition |
|------------------|-------------|------|------------------|
| `rpmsbl.mbn` | RPM Secondary Bootloader | 500 KB | p2 (CFG_DATA) |
| `spbl.mbn` | Secondary Program Boot Loader | 1500 KB | p3 (SPBL) |
| `rpm.mbn` | RPM firmware | 500 KB | p5 (APPSSBL) |
| `ssbl.mbn` | Qualcomm Secondary Bootloader | 500 KB | p6 (QCSBL) |
| `emmc_appsboot.mbn` | APPSBL (Bootie) | 2500 KB | p8 (FOTA) |
| `tz.mbn` | TrustZone | 500 KB | p10 (OEMSBL) |
| `boot.img` | Android-style boot image | 10 MB | p9 (APPS) |

## Key Insights

### 1. Partition Naming Confusion

HP/Palm used confusing partition type names:
- **CFG_DATA (p2)** contains `rpmsbl.mbn` (RPM Secondary Bootloader)
- **APPSSBL (p5)** contains `rpm.mbn` (RPM firmware)
- **QCSBL (p6)** contains `ssbl.mbn` (Secondary Bootloader)
- **FOTA (p8)** contains `emmc_appsboot.mbn` (APPSBL = Bootie)
- **OEMSBL (p10)** contains `tz.mbn` (TrustZone)

The partition type names don't always match what they contain!

### 2. Boot Chain Sequence

```
ROM → SPBL (p3) → RPMSBL (p2) → RPM (p5) → QCSBL (p6) → APPSBL (p8) → TZ (p10) → Boot.img (p9)
                                                                           ↑
                                                                     EMPTY ON TOUCHPAD!
```

Actually, looking at this more carefully, the boot sequence should be:

```
ROM → SPBL (p3, spbl.mbn) → 
      RPMSBL (p2, rpmsbl.mbn) → 
      RPM (p5, rpm.mbn) → 
      QCSBL (p6, ssbl.mbn) → 
      APPSBL (p8, emmc_appsboot.mbn = Bootie) → 
      TZ (p10, tz.mbn) [EMPTY!] → 
      Kernel (p9, boot.img or p14 /boot)
```

### 3. TrustZone Location Confirmed

The official layout confirms:
- **p10** is OEMSBL partition (500 KB)
- Contains `tz.mbn` file
- This is the TrustZone kernel
- **We confirmed this is COMPLETELY EMPTY on TouchPad**

### 4. What We're Missing

Looking at the official layout, we never extracted:
- **p1** (FAT boot partition, 1 MB) - contains `fat.bin`
- **p7** (EFS2, 63.5 MB) - Encrypted File System (modem/radio data)
- **p11** (MODEM_ST1, 3 MB) - Modem state 1
- **p12** (MODEM_ST2, 3 MB) - Modem state 2
- **p13** (NVRAM, 4 MB) - moboot environment and boot logos

None of these should have peripheral initialization code (they're filesystems or modem data), but p13 has the moboot environment which might be interesting.

### 5. Bootie is APPSBL

The file `emmc_appsboot.mbn` (2.5 MB) goes to p8 (FOTA partition).

This is Bootie (the HP bootloader), which is the APPSBL (Applications Processor Secondary Boot Loader).

So when we extracted `/tmp/touchpad-p7.bin` at 2.5 MB, we likely got p8 (Bootie/APPSBL), not p7 (EFS2 which is 63.5 MB).

## Comparison with HTC

HTC boot sequence (MSM8960):
```
ROM → sbl1.img → sbl2.img → sbl3.img → rpm.img → tz.img (106 KB, WORKING) → boot.img
```

TouchPad boot sequence (APQ8060):
```
ROM → spbl.mbn → rpmsbl.mbn → rpm.mbn → ssbl.mbn → emmc_appsboot.mbn → tz.mbn (500 KB, EMPTY) → boot.img
```

**Key difference**: HTC has working TrustZone (106 KB with code), TouchPad has empty TrustZone (500 KB of zeros).

## Where CE2 Init Should Have Been

Based on HTC TrustZone analysis, the CE2 initialization code should be in:
- **p10 (tz.mbn)** - TrustZone kernel
- But this is **EMPTY** on TouchPad

This is why CE2 (and potentially PMIC, clocks, other secure peripherals) was never initialized.

## Verification Commands

To verify actual partition numbers on the device:

```bash
# Show partition table
fdisk -l /dev/mmcblk0

# Show partition types
gdisk -l /dev/mmcblk0

# Check p7 size (should be 63.5 MB if EFS2)
blockdev --getsize64 /dev/mmcblk0p7

# Check p8 size (should be 2.5 MB if APPSBL)
blockdev --getsize64 /dev/mmcblk0p8
```

## Action Items

1. ✅ We correctly identified TrustZone (p10) is empty
2. ✅ We correctly extracted bootloader stages (even if we misnamed some)
3. ✅ We correctly identified CE2 init missing from TZ
4. ✅ We fixed CE2 in kernel (bypassing missing TZ init)
5. ⏭️ Next: Apply same approach to PMIC/regulator init (highest priority from comparison)
6. ⏭️ Consider extracting p13 (NVRAM) to see moboot environment

## Conclusion

The topaz.xml confirms our findings:
- TrustZone (p10, tz.mbn) is the missing initialization stage
- This should be 500 KB with peripheral init code
- On TouchPad, it's 500 KB of zeros
- This explains CE2, and likely explains PMIC/regulator issues too

Our extraction was mostly correct, we just need to be more careful about partition numbering vs partition roles.

---

**Date**: 2026-05-15
**Source**: `/home/herrie/Downloads/webosdoctorp305hstnhwifi/resources/topaz.xml`
**Analysis**: Herman van Hazendonk + Claude Code
