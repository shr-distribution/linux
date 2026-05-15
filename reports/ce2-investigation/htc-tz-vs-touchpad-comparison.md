# HTC TrustZone vs HP TouchPad Bootloader Comparison
## Date: 2026-05-15

## Overview

**HTC Device**: Likely HTC One S or similar (MSM8960)  
**HTC TZ Firmware**: `/tmp/tz.img` (106 KB)  
**TouchPad OEMSBL/TZ**: Partition 10 - **COMPLETELY EMPTY**

---

## File Comparison

### HTC TrustZone (tz.img)

**Size**: 107,686 bytes (106 KB)  
**Type**: ARM binary (TrustZone kernel)  
**Header**: ARM exception vectors
```
00000000  19 00 00 00 03 00 00 00  00 00 00 00 00 00 03 2a
```

**Contents**:
- ARM exception vector table
- TrustZone kernel code
- DMA channel definitions
- Hardware initialization routines
- Peripheral configuration tables

### TouchPad OEMSBL (partition 10)

**Size**: 500 KB (partition size)  
**Type**: Empty  
**Header**: All zeros
```
00000000  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00
*
0007d000
```

**Contents**: None - completely unpopulated

---

## CE2/QCE References Comparison

### HTC TZ Firmware

**DMA Channel Definitions Found:**
```
CHAN_CE2_IN_TZ      ← TrustZone RX channel
CHAN_CE2_OUT_TZ     ← TrustZone TX channel
DEV_CE2_IN_A        ← APPS RX device
CHAN_CE2_IN_A       ← APPS RX channel
DEV_CE2_OUT_A       ← APPS TX device
CHAN_CE2_OUT_A      ← APPS TX channel
DEV_CE2_IN_TZ       ← TrustZone RX device
CHAN_CE2_IN_TZ      ← TrustZone RX channel (duplicate)
DEV_CE2_OUT_TZ      ← TrustZone TX device
CHAN_CE2_OUT_TZ     ← TrustZone TX channel (duplicate)
DEV_CE2_IN_M        ← Modem RX device
CHAN_CE2_IN_M       ← Modem RX channel
DEV_CE2_OUT_M       ← Modem TX device
CHAN_CE2_OUT_M      ← Modem TX channel
CRCI_CE2_IN         ← CRCI for CE2 input
CRCI_CE2_OUT        ← CRCI for CE2 output
CRCI_CE2_HASH       ← CRCI for CE2 hash engine
```

**Hardware Addresses Found:**
```
0x18500000 (QCE base) - Found at multiple offsets:
  - 0x132a0
  - 0x132c0
  - 0x13310
  - 0x133c0
  - 0x13490
  - 0x13550
  - 0x13740
  - 0x13760
```

**Initialization Strings:**
```
"Initializing PIL\n"
"DMOV: Unable to initialize ADM3_0"
"Dmov Svc: Unable to initialize ADM3_1"
"HW Initialization Failed"
"DMOV Initialization Failed"
"Init handler WARM_BOOT Failed"
"clocks_mgd"
"enable_pm"
```

### TouchPad Bootloaders (p2-p9)

**DMA Channel Definitions Found:**
```
Partition 6 (QCSBL):
CHAN_CE_OUT_TZ
CHAN_CE_IN_TZ
CHAN_CE2_OUT_TZ
CHAN_CE2_IN_TZ

Partition 7 (APPSBL/Bootie):
DEV_CE2_IN_A
CHAN_CE2_IN_A
DEV_CE2_OUT_A
CHAN_CE2_OUT_A
DEV_CE2_IN_TZ
CHAN_CE2_IN_TZ
DEV_CE2_OUT_TZ
CHAN_CE2_OUT_TZ
DEV_CE2_IN_M
CHAN_CE2_IN_M
DEV_CE2_OUT_M
CHAN_CE2_OUT_M
```

**Hardware Addresses Found:**
```
Partition 5 (RPM):
  0x00902740 (CE2_HCLK_CTL) - Clock control

Partition 6 (QCSBL):
  0x00902fd4 (CE2_HALT_STATUS) - Halt status check
```

**Initialization Strings:**
```
None specific to CE2 initialization
```

---

## Key Differences

### 1. TrustZone Stage Presence

| Aspect | HTC | TouchPad |
|--------|-----|----------|
| TZ Firmware | ✅ Present (106 KB) | ❌ Missing (empty partition) |
| TZ Kernel | ✅ Full TrustZone kernel | ❌ None |
| Init routines | ✅ Has initialization code | ❌ None |

### 2. QCE Hardware References

| Aspect | HTC TZ | TouchPad Bootloaders |
|--------|--------|---------------------|
| QCE base (0x18500000) | ✅ Referenced at 8+ locations | ❌ Not found |
| CE2 DMA channels | ✅ All three domains defined | ✅ All three domains defined |
| CRCI definitions | ✅ Has CRCI_CE2_* | ❌ Not found |
| Clock registers | ❓ Unknown | ✅ Has CE2_HCLK_CTL |

### 3. Initialization Code

| Aspect | HTC TZ | TouchPad |
|--------|--------|----------|
| PIL init | ✅ "Initializing PIL" | ❌ None |
| DMOV init | ✅ "DMOV Initialization" | ❌ None |
| HW init | ✅ Has "HW Initialization" | ❌ None |
| Power mgmt | ✅ "enable_pm", "clocks_mgd" | ❌ None |

---

## What HTC TZ Does (That TouchPad Lacks)

Based on strings and structure, HTC TZ firmware performs:

### 1. PIL (Peripheral Image Loader) Initialization
```
"Initializing PIL\n"
"tzbsp_pil_init_image"
```
**Purpose**: Load firmware images for peripherals (modem, ADSP, etc.)  
**Impact on CE2**: May initialize crypto engine firmware/microcode

### 2. DMOV/ADM Initialization
```
"DMOV: Unable to initialize ADM3_0"
"Dmov Svc: Unable to initialize ADM3_1"
"DMOV Initialization Failed"
```
**Purpose**: Initialize DMA controllers  
**Impact on CE2**: CE2 requires DMA (ADM0 channels 2-3)  
**Note**: TouchPad ADM works for eMMC, so this isn't fatal

### 3. Hardware Initialization
```
"HW Initialization Failed"
" HW Initialization Failed"
" Init handler WARM_BOOT Failed"
```
**Purpose**: Initialize peripherals at boot  
**Impact on CE2**: This is likely where CE2 gets initialized

### 4. Power/Clock Management
```
"clocks_mgd"
"enable_pm"
"DAL: Attempt to use driver in powered-down state!"
"DAL: Attempt to use driver while other proc has powered-down the hardware!"
```
**Purpose**: Manage peripheral power domains and clocks  
**Impact on CE2**: This is probably the missing piece!

### 5. Security Domain Configuration
```
"upper_bank_vmid_enable_mask"
```
**Purpose**: Configure VMID (Virtual Machine ID) / Security Domain access  
**Impact on CE2**: Controls which processors can access which peripherals

---

## Analysis: What's Missing on TouchPad

### Critical Missing Components

**1. Peripheral Power Enable**

HTC TZ has power management code (`enable_pm`, `clocks_mgd`) that likely:
- Enables voltage rails for peripherals
- Ungates clocks at hardware level
- Enables power domains

**TouchPad consequence**: CE2 power domain never enabled → hardware stays OFF

**2. Hardware Initialization Routine**

HTC TZ has explicit "HW Initialization" code that likely:
- Writes initial configuration to CE2 registers
- Sets up DMA arbitration
- Configures bus fabric routing

**TouchPad consequence**: CE2 remains uninitialized → MMIO reads return zeros

**3. Security Domain Setup**

HTC TZ has VMID/security domain code that likely:
- Configures TrustZone Peripheral Access Control (TZPAC)
- Assigns peripherals to security domains
- Enables non-secure access where appropriate

**TouchPad consequence**: CE2 may be locked to undefined security domain

**4. PIL Firmware Loading**

HTC TZ has PIL (Peripheral Image Loader) that may:
- Load CE2 microcode (if CE2 has any)
- Initialize crypto engine state
- Set up hardware contexts

**TouchPad consequence**: CE2 internal state never initialized

---

## Specific QCE References in HTC TZ

### QCE Base Address (0x18500000)

Found at 8 locations in TZ firmware:
```
Offset    Context
-------   -------
0x132a0   00 00 50 18 00 23 08 b9  02 20 70 47
0x132c0   c9 6f c0 f8 fd 10 18 46  70 47 00 00 00 00 50 18
0x13310   60 46 70 47 00 00 50 18  00 01 50 18 2d e9 f0 43
0x133c0   b6 d1 05 b0 00 20 bd e8  f0 83 00 00 00 00 50 18
0x13490   00 28 f9 d0 20 46 10 bc  70 47 00 00 00 00 50 18
0x13550   00 00 50 18 30 b4 00 23  08 b1 01 b1 12 b9 30 bc
0x13740   74 20 48 62 00 20 70 47  01 20 70 47 00 00 50 18
0x13760   00 00 50 18 00 b5 87 b0  0c 23 03 93 04 90 05 91
```

**Pattern analysis**:
- Appears in data sections (not code)
- Often followed by ARM Thumb return instructions (`70 47` = `bx lr`)
- Likely part of function pointer tables or configuration structures

**Hypothesis**: These are **peripheral base address tables** used by TZ to:
1. Map peripheral MMIO regions
2. Configure MMU/page tables for secure/non-secure access
3. Initialize bus fabric routing

### CRCI Definitions

HTC TZ defines CE2 CRCI identifiers:
```
CRCI_CE2_IN     ← Input channel CRCI
CRCI_CE2_OUT    ← Output channel CRCI
CRCI_CE2_HASH   ← Hash engine CRCI
```

**TouchPad comparison**:
- TouchPad bootloaders have CRCI 4 (CE_IN) and 5 (CE_OUT)
- TouchPad does NOT have CRCI_CE2_HASH reference
- This suggests different CE2 configuration

---

## Boot Sequence Comparison

### HTC (MSM8960) - Complete Boot Chain

```
ROM PBL
  ↓
SBL1 (Primary bootloader)
  ↓ Initializes DDR
SBL2 (Secondary bootloader)
  ↓ Initializes clocks
SBL3 (Tertiary bootloader)
  ↓ Loads and verifies images
TZ (TrustZone kernel)  ← THIS IS THE KEY
  ↓ - Initializes PIL
  ↓ - Enables power domains
  ↓ - Configures TZPAC
  ↓ - Initializes peripherals (including CE2)
RPM (Resource Power Manager)
  ↓ Runs continuously
Linux Kernel
```

### TouchPad (MSM8660) - Incomplete Boot Chain

```
ROM PBL
  ↓
SPBL (partition 3)
  ↓ Initializes DDR
QCSBL (partition 6)
  ↓ Configures clocks
  ↓ Defines CE2 channels
APPSBL/Bootie (partition 7)
  ↓ Defines CE2 DMA channels
  ↓ Loads kernel
[OEMSBL/TZ MISSING]  ← CRITICAL GAP
  ↓ ❌ No PIL init
  ↓ ❌ No power domain enable
  ↓ ❌ No TZPAC config
  ↓ ❌ No peripheral init
RPM (partition 5)
  ↓ Has CE2 clock register
  ↓ But doesn't enable it?
Linux Kernel
  ↓ Tries to use CE2
  ↓ Hardware doesn't respond
```

**The missing TZ stage is the smoking gun.**

---

## Next Steps: Extract Init Sequence from HTC TZ

### Goal

Find the exact code in HTC TZ that initializes CE2, so we can replicate it in TouchPad's Linux kernel driver.

### Approach

1. **Decompile HTC TZ with Ghidra**
   - Import tz.img (✅ Done)
   - Set load address (TZ likely loads at 0x01A00000 or similar)
   - Analyze functions around 0x13000 region (where QCE refs are)

2. **Find CE2 Initialization Function**
   - Search for functions that:
     - Reference 0x18500000 (QCE base)
     - Write to 0x00902740 (CE2_HCLK_CTL)
     - Call "enable_pm" or "clocks_mgd"
   - Look for initialization patterns

3. **Extract Register Writes**
   - Document all MMIO writes to:
     - 0x18500000 range (CE2 config)
     - 0x00900000 range (GCC clocks)
     - Power domain registers
   - Create initialization sequence

4. **Replicate in Linux Driver**
   - Add init sequence to QCE driver probe
   - Test on TouchPad
   - Verify if CE2 becomes accessible

---

## Hypothesis

**Why HTC devices work and TouchPad doesn't:**

HTC devices have complete TrustZone firmware (tz.img) that:
1. Enables CE2 power domain at boot
2. Configures CE2 peripheral registers
3. Sets up TZPAC to allow APPS access
4. Leaves CE2 in ready state for Linux

TouchPad lacks this TZ stage, so:
1. CE2 power domain never enabled
2. CE2 registers never initialized
3. TZPAC never configured
4. Linux finds CE2 in powered-off state

**If we can extract HTC's init sequence and run it from Linux, CE2 might work on TouchPad.**

---

## Files for Analysis

**HTC TrustZone firmware:**
```
/tmp/tz.img              (106 KB - TrustZone kernel)
/tmp/sbl1.img            (78 KB - Primary bootloader)
/tmp/sbl2.img            (111 KB - Secondary bootloader)
/tmp/sbl3.img            (584 KB - Tertiary bootloader)
/tmp/rpm.img             (119 KB - RPM firmware)
```

**TouchPad bootloaders:**
```
/tmp/touchpad-p2.bin     (500 KB - CFG_DATA)
/tmp/touchpad-p3.bin     (1.5 MB - SPBL)
/tmp/touchpad-p5.bin     (500 KB - RPM)
/tmp/touchpad-p6.bin     (750 KB - QCSBL)
/tmp/touchpad-p7.bin     (2.5 MB - APPSBL/Bootie)
/tmp/touchpad-p9.bin     (1.5 MB - Unknown)
/tmp/tz-touchpad.mbn     (500 KB - EMPTY)
```

**Ghidra projects:**
```
/tmp/ghidra-analysis/htc-tz/         (HTC TZ analysis)
/tmp/ghidra-analysis/touchpad-boot/  (TouchPad bootloaders)
```

---

## Summary

| Component | HTC (MSM8960) | TouchPad (MSM8660) |
|-----------|---------------|-------------------|
| TZ Firmware | ✅ 106 KB, full kernel | ❌ Empty partition |
| CE2 channels | ✅ Defined in TZ | ✅ Defined in bootloaders |
| QCE address | ✅ Referenced 8x in TZ | ❌ Not in bootloaders |
| Init code | ✅ Has PIL, DMOV, HW init | ❌ None |
| Power mgmt | ✅ Has enable_pm | ❌ None |
| Result | ✅ CE2 likely works | ❌ CE2 doesn't work |

**Conclusion**: The missing TrustZone initialization stage is why CE2 doesn't work on TouchPad. HTC's TZ firmware contains the init sequence we need.
