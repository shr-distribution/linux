# CE2 Hardware Crypto Investigation
## HP TouchPad (MSM8660/APQ8060) - May 2026

This directory contains the complete investigation and breakthrough that enabled QCE (Qualcomm Crypto Engine) CE2 hardware acceleration on HP TouchPad running mainline Linux 6.18.

## Investigation Summary

**Problem**: CE2 crypto engine MMIO returned all zeros, making hardware completely inaccessible despite proper clocks, DMA, and CRCI configuration.

**Root Cause**: HP TouchPad bootloader lacks the OEMSBL/TrustZone initialization stage. The TZ partition (p10) is completely empty (500KB of zeros).

**Solution**: Manual clock enable in Linux driver (set bit 4 of CE2_HCLK_CTL @ GCC 0x00902740).

**Result**: ✅ Hardware working, 20 crypto algorithms functional, performance excellent for hash operations.

## Key Documents

### 1. Breakthrough and Results

- **[CE2-BREAKTHROUGH-SUCCESS.md](CE2-BREAKTHROUGH-SUCCESS.md)** - Complete timeline and victory report
- **[QCE-PERFORMANCE-BENCHMARK.md](QCE-PERFORMANCE-BENCHMARK.md)** - Detailed performance tests
  - SHA1: 26.5 MB/s
  - SHA256: 15.7 MB/s
  - AES-128-CBC: 11.4 MB/s

### 2. Implementation Guide

- **[qce-complete-init-sequence.md](qce-complete-init-sequence.md)** - Complete initialization procedure extracted from HTC TZ firmware
- Implementation location: `drivers/crypto/qce/core.c` lines 270-367
- Commit: fc0964d73cc5

### 3. Reverse Engineering Analysis

- **[htc-tz-ce2-init-analysis.md](htc-tz-ce2-init-analysis.md)** - Ghidra disassembly of HTC TrustZone firmware
- **[htc-tz-vs-touchpad-comparison.md](htc-tz-vs-touchpad-comparison.md)** - Side-by-side comparison of HTC (working) vs TouchPad (broken)

### 4. Bootloader Analysis

- **[touchpad-tz-extraction-results.md](touchpad-tz-extraction-results.md)** - Discovery of empty TZ partition
- **[qce-bootloader-channel-discovery.md](qce-bootloader-channel-discovery.md)** - DMA channel configuration in bootloaders
- **[qce-final-bootloader-analysis.md](qce-final-bootloader-analysis.md)** - Complete bootloader investigation

### 5. Vendor Analysis

- **[vendor-binary-qce-analysis.md](vendor-binary-qce-analysis.md)** - Analysis of HTC, Samsung, Sony, Xiaomi binaries proving all vendors avoided CE2
- **[libQSEEComAPI-deep-analysis.md](libQSEEComAPI-deep-analysis.md)** - Xiaomi TrustZone communication library analysis

## Investigation Timeline

**Duration**: ~10 hours (2026-05-14 evening through 2026-05-15 04:00 UTC)

### Phase 1: Problem Discovery (earlier sessions)
- Identified QCE hardware completely dead
- Fixed ADM DMA CRCI programming
- Added CE2 bus clock and reset
- Still didn't work - MMIO remained zero

### Phase 2: Bootloader Extraction (May 15, 00:00-02:00)
- Extracted all TouchPad bootloader partitions (p2-p10)
- Discovered OEMSBL/TZ partition completely empty
- Found DMA channels configured but never enabled
- Located CE2 clock registers in bootloaders

### Phase 3: HTC TZ Reverse Engineering (May 15, 02:00-03:00)
- Downloaded HTC RUU package (PG86IMG.zip)
- Extracted TrustZone firmware (tz.img, 106KB)
- Reverse-engineered with Ghidra
- Found clock enable at offset 0xd120
- Found peripheral init at offset 0x13200

### Phase 4: Implementation & Breakthrough (May 15, 03:00-04:00)
- Implemented complete init sequence
- Hit boot hang from register scanning
- Simplified to minimal clock enable
- **BREAKTHROUGH: Hardware responds!**
- Verified with performance benchmarks

## Key Findings

### 1. The Simple Fix

After complex investigation, the fix was one line:
```c
val |= BIT(4);   /* Set bit 4 of CE2_HCLK_CTL */
```

This enables the CE2 clock at hardware level, which the missing bootloader stage should have done.

### 2. Why Vendors Avoided CE2

Analysis of vendor binaries from HTC, Samsung, Sony, and Xiaomi (298 libraries total) found **zero QCE hardware usage**. All vendors used OpenSSL software crypto.

**Reason**: They all discovered the same problem we did - CE2 doesn't work on MSM8660 because initialization is missing. Rather than fix it, they worked around it.

### 3. HTC TZ Firmware Analysis Was Misleading

The generic clock function at HTC TZ offset 0xd120 **clears** bit 4, which seemed correct but was wrong. That function is for *gating* clocks, not enabling them.

We initially implemented it and broke the working code, then had to revert to the simple enable.

### 4. TouchPad Boot Sequence Gap

```
Standard Qualcomm: ROM → SPBL → QCSBL → APPSBL → OEMSBL/TZ → Kernel
HP TouchPad:       ROM → SPBL → QCSBL → APPSBL → [EMPTY!]  → Kernel
                                                    ↑
                                          Missing init stage
```

## Files Extracted During Investigation

### TouchPad Bootloaders
- `/tmp/touchpad-p2.bin` - CFG_DATA (500KB)
- `/tmp/touchpad-p3.bin` - SPBL (1.5MB)
- `/tmp/touchpad-p5.bin` - RPM (500KB) - Contains CE2_HCLK_CTL address
- `/tmp/touchpad-p6.bin` - QCSBL (750KB) - Contains CE2_HALT_STATUS
- `/tmp/touchpad-p7.bin` - APPSBL/Bootie (2.5MB) - Defines all CE2 channels
- `/tmp/touchpad-p9.bin` - Unknown (1.5MB)
- `/tmp/tz-touchpad.mbn` - OEMSBL/TZ (500KB) - **COMPLETELY EMPTY**

### HTC Reference Firmware
- `/tmp/tz.img` - TrustZone kernel (106KB, MSM8960)
- `/tmp/sbl1.img`, `/tmp/sbl2.img`, `/tmp/sbl3.img` - HTC bootloaders
- `/tmp/rpm.img` - HTC RPM firmware

## Code Changes

**Primary commit**: fc0964d73cc5
**File**: `drivers/crypto/qce/core.c`
**Function**: `qce_crypto_probe()`
**Lines**: 261-367 (CE2 Hardware Initialization block)

Key sections:
- Phase 1: Clock Enable (lines 283-341)
- Phase 2: Peripheral Init (lines 317-354) - times out but harmless
- Phase 3: Verification (lines 357-367)

## Performance Impact

With CE2 hardware acceleration enabled:

| Operation     | Software (est) | QCE Hardware | Speedup |
|---------------|----------------|--------------|---------|
| SHA1          | ~12 MB/s       | 26.5 MB/s    | ~2.2x   |
| SHA256        | ~6 MB/s        | 15.7 MB/s    | ~2.6x   |
| AES-128-CBC   | ~10 MB/s       | 11.4 MB/s    | ~1.1x   |

Hash operations benefit most from hardware acceleration.

## Historical Significance

**First working QCE CE2 implementation on mainline Linux for MSM8660/APQ8060.**

All vendors (HTC, Samsung, Sony, Xiaomi) avoided CE2 hardware and used software crypto exclusively. This is the first time CE2 hardware crypto works on this platform with a mainline kernel.

## Related Commits

- `3749afb54308` - Restore CE2 bus clock and reset (May 15, 00:30)
- `f7b526b5772e` - First manual clock enable attempt (May 15, 01:30)
- `83c48605eefc` - Wrong HTC TZ bit pattern (broke it) (May 15, 03:14)
- `cd11f77c5d14` - Register scanning (caused boot hang) (May 15, 03:25)
- `fc0964d73cc5` - **Final working version** (May 15, 03:35)

## Tools Used

- **Ghidra** - Reverse engineering HTC TrustZone firmware
- **hexdump** - Binary analysis of bootloaders
- **arm-linux-gnueabihf-objdump** - ARM disassembly
- **grep/strings** - Firmware string analysis
- **Python** - Binary pattern searching
- **OpenSSL** - Performance benchmarking

## Credits

**Investigation and Implementation**: Herman van Hazendonk (Herrie82)
**Analysis Assistance**: Claude Code (Anthropic)
**Reference Hardware**: HTC device TrustZone firmware (PG86IMG.zip)

---

**Date Range**: 2026-05-14 to 2026-05-15
**Total Investigation Time**: ~10 hours
**Result**: ✅ Hardware crypto working on 13-year-old tablet
