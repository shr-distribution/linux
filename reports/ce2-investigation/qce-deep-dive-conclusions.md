# QCE CE2 Deep Dive Conclusions
## Date: 2026-05-15

## Firmware Analysis Summary

### HTC TrustZone (tz.img - 106KB)
- ✅ Contains QCE test sequence (validation, not init)
- ❌ No CE2_HCLK_CTL clock enable  
- ❌ No ADM DMA initialization
- ❌ No CRCI configuration
- **Conclusion**: TZ assumes bootloader already did init

### Samsung Modem (MODEM.B05 - 16MB)
- ✅ Has QCE and ADM address tables
- ✅ Contains Qualcomm crypto API code
- ❌ Error messages suggest hardware not actually used
- ❌ No Linux-accessible initialization found
- **Conclusion**: Modem firmware doesn't help Linux driver

## What We Successfully Did

1. ✅ **Phase 1: Clock Enable** - Manual GCC CE2_HCLK_CTL bit 4 write
   - Hardware is now accessible (MMIO reads work)
   - All 20 crypto algorithms register
   - Self-tests pass

2. ✅ **CRCI Programming** - Set CRCI_CTL for channels 4 & 5
   - Value 0x1 = 32-byte burst size
   - Done at EE=0 during probe

3. ✅ **Device Tree Configuration** - Proper clocks, resets, DMA channels

## What's Still Broken

**Symptom**: Crypto operations hang indefinitely in 'D' state (uninterruptible sleep)

**Root Cause**: DMA completion never arrives - process waits forever for:
```c
wait_for_completion_timeout(&qce->dma_completion, ...);
```

## Likely Missing Pieces

### 1. ADM Interrupt Routing
The ADM DMA controller generates interrupts when transfers complete, but they may not be:
- Enabled in ADM registers
- Routed through interrupt controller (GIC)  
- Properly registered in Linux IRQ subsystem

**Check**: Does `cat /proc/interrupts` show ADM interrupts? Are they firing?

### 2. ADM Channel State Machine
ADM channels have complex state:
- IDLE → ACTIVE → COMPLETE cycle
- May need explicit "enable" command beyond just queuing descriptors
- CRCI handshaking might need additional setup

**Check**: Read ADM channel status registers during hung operation

### 3. QCE-to-ADM Flow Control
CRCI lines connect QCE to ADM for flow control:
- CE_IN (CRCI 4): QCE ready to receive data from ADM
- CE_OUT (CRCI 5): QCE has data to send to ADM

**Check**: Are CRCI signals actually toggling? Might need logic analyzer.

### 4. Security/TrustZone Lock
CE2 might be locked to TrustZone security domain:
- TZPAC (TrustZone Protection and Access Control) configuration
- SCM (Secure Channel Manager) calls needed to unlock

**Check**: Does writing to QCE config registers actually work, or silently fail?

## Comparison to Working Platforms

### Why Other MSM8660 Devices Work (HTC, etc.)

They have **complete bootloader chain**:
```
PBL → OEMSBL → APPSBL → TZ → Linux
      ^^^^^^^
      This does CE2/ADM init!
```

### Why TouchPad Doesn't Work

Missing OEMSBL stage (partition empty):
```  
PBL → APPSBL → Linux
      (skips CE2/ADM init)
```

**Whatever OEMSBL does, we need to replicate in Linux driver!**

## Next Steps to Fix QCE

### Approach 1: Add Missing ADM Init (Most Likely)

1. Check `/proc/interrupts` - are ADM interrupts registered?
2. Read ADM channel status during hang - what state are channels in?
3. Add explicit ADM channel enable after queueing descriptors
4. Verify CRCI signals are actually connected (device tree)

### Approach 2: Bypass DMA (Debugging Only)

Implement polled/PIO mode for QCE:
- Write data directly to QCE_DATA_IN
- Poll QCE_STATUS for completion
- Read result from QCE_DATA_OUT

This would prove QCE hardware works, isolating problem to DMA.

### Approach 3: Extract More Bootloader Stages

Look for OEMSBL or SBL in:
- TouchPad bootloader partitions (might be packed differently)
- HTC device firmware (if OEMSBL can be found)
- Qualcomm reference bootloader source (if available)

## Why We're Stuck

The **critical init sequence** happens in:
1. Boot ROM (PBL) - burned into hardware, can't access
2. OEMSBL - missing on TouchPad  
3. Before any firmware we can analyze

We need to either:
- **Find** OEMSBL code that does init
- **Reverse engineer** what OEMSBL does by testing hardware
- **Give up** and use software crypto (what Samsung/HTC likely do)

## Recommendation

**Focus on VIDC firmware loading** (where we're making progress) rather than continuing 
to chase QCE without access to the bootloader code that actually initializes it.

QCE is a "nice to have" for crypto acceleration.
VIDC is "must have" for video encoding/decoding.

