# CE2 Diagnostic Implementation Summary
## Date: 2026-05-15
## Commit: 3b8bb4669958

---

## What Was Implemented

Comprehensive CE2 diagnostic suite testing Gemini AI's "Security Domain Conflict" theory.

**Theory:** ADM channels 4 & 5 are locked to Security Domain 3 (TrustZone) at EE=0, preventing Linux (non-secure, EE=1) from using them for DMA transfers. This would explain why DMA completion never arrives.

---

## Code Changes

### File: `drivers/crypto/qce/core.c`

**1. Module Parameters (4 new)**
```c
module_param_named(test_adm_domain, qce_test_adm_domain, bool, 0644);
module_param_named(fix_adm_domain, qce_fix_adm_domain, bool, 0644);
module_param_named(use_pio_mode, qce_use_pio_mode, bool, 0644);
module_param_named(try_scm_unlock, qce_try_scm_unlock, bool, 0644);
```

**2. Diagnostic Functions (3 new)**

#### `qce_test_adm_domain_conflict()`
- Maps ADM0/ADM1 at EE=0 (0x18320000, 0x18420000) and EE=1 (+0x800)
- Reads CH_CONF registers for channels 2, 4, 5
- Extracts security domain from bits 27-28
- Reports if locked to Domain 3 (secure world)
- Returns -EACCES if conflict found

#### `qce_fix_adm_domain_conflict()`
- Reads current CH_CONF values at EE=0
- Attempts to clear bits 27-28 (set domain to 0)
- Toggles GCC BCR bit 7 to clear XPU error state
- Reads back to verify write succeeded
- Returns 0 if fix worked, -EPERM if hardware locked

#### `qce_test_pio_mode()`
- Bypasses DMA completely
- Writes test data (0x12345678) to CE2_REG_DATA_IN (0x000)
- Polls CE2_REG_STATUS (0x020) for DOUT_RDY bit (bit 3)
- Reads result from CE2_REG_DATA_OUT (0x010)
- Timeout 10ms (1000 iterations @ 10µs)
- Returns -ETIMEDOUT if DOUT_RDY never sets, -EIO if SW_ERR sets

**3. Probe Function Integration**
- Diagnostics run after CRCI programming (line ~590)
- Only executes if module params are set
- Fails probe if unfixable conflict found

---

## Memory Map Reference

### ADM Controllers
```
ADM0 EE=0 (Secure):     0x18320000
ADM0 EE=1 (Non-Secure): 0x18320800
ADM1 EE=0 (Secure):     0x18420000  ← QCE uses this
ADM1 EE=1 (Non-Secure): 0x18420800
```

### ADM Channel Registers (per channel)
```
CH_CMD:       base + 0x000 + (ch * 0x80)
CH_STATUS:    base + 0x004 + (ch * 0x80)
CH_CONF:      base + 0x008 + (ch * 0x80)  ← Domain bits here
CH_RSLT_CONF: base + 0x00C + (ch * 0x80)
CRCI_CTL_n:   base + 0x400 + (ch * 0x04)
```

### QCE Channels on ADM1
```
Channel 4: CE_IN  (RX) - offset 0x200
Channel 5: CE_OUT (TX) - offset 0x280
```

### CE2 Registers
```
DATA_IN:  0x000  ← Write data here for PIO mode
DATA_OUT: 0x010  ← Read result here
STATUS:   0x020  ← Poll for DOUT_RDY (bit 3)
CONFIG:   0x024
GOPROC:   0x040  ← Trigger processing
```

---

## Register Format: CH_CONF

```
Bits 31-28: Reserved
Bits 27-28: Security Domain ← KEY FIELD
            00 = Domain 0 (Non-Secure, Linux can use)
            01 = Domain 1 (Reserved)
            10 = Domain 2 (Reserved)
            11 = Domain 3 (Secure World, Linux BLOCKED)
Bits 26-12: Various config (burst size, etc.)
Bits 11-0:  More config
```

**Expected value if locked:** 0x180008d5 (Domain=3, bits 27-28 = 11b)
**Expected value if unlocked:** 0x000008d5 (Domain=0, bits 27-28 = 00b)

---

## Register Format: CE2_REG_STATUS

```
Bit 31-28: CORE_REV (CE2 core revision)
Bit 24-22: DOUT_SIZE_AVAIL
Bit 21-19: DIN_SIZE_AVAIL
Bit 18:    ACCESS_VIOL
Bit 17:    SEG_CHNG_ERR
Bit 16:    CFG_CHNG_ERR
Bit 15:    DOUT_ERR
Bit 14:    DIN_ERR
Bit 13:    LOCKED
Bit 12-10: CRYPTO_STATE
Bit 9:     ENCR_BUSY
Bit 8:     AUTH_BUSY
Bit 7:     DOUT_INTR
Bit 6:     DIN_INTR
Bit 5:     AUTH_DONE_INTR
Bit 4:     ERR_INTR
Bit 3:     DOUT_RDY ← Check this in PIO mode
Bit 2:     DIN_RDY  ← Check this before writing
Bit 1:     AUTH_DONE
Bit 0:     SW_ERR   ← Error flag
```

---

## Test Scenarios

### Scenario 1: Domain Conflict + Fix Works
```
test_adm_domain=1 → Conflict found (Domain=3)
fix_adm_domain=1  → Write succeeds, Domain=0
[reload with fix] → sha256sum works
Result: CE2 functional! ✓
```

### Scenario 2: Domain Conflict + Fix Fails + PIO Works
```
test_adm_domain=1 → Conflict found (Domain=3)
fix_adm_domain=1  → Write ignored, still Domain=3
use_pio_mode=1    → DOUT_RDY sets, data returned
Result: QCE OK, DMA locked. Abandon CE2. ✗
```

### Scenario 3: Domain Conflict + Fix Fails + PIO Fails
```
test_adm_domain=1 → Conflict found (Domain=3)
fix_adm_domain=1  → Write ignored, still Domain=3
use_pio_mode=1    → Timeout, DOUT_RDY never sets
Result: Entire CE2 locked to secure world. Abandon. ✗
```

### Scenario 4: No Domain Conflict + PIO Works
```
test_adm_domain=1 → No conflict (Domain=0)
use_pio_mode=1    → DOUT_RDY sets, data returned
Result: QCE OK, problem is elsewhere (CRCI/interrupts). Debug more. ?
```

### Scenario 5: No Domain Conflict + PIO Fails
```
test_adm_domain=1 → No conflict (Domain=0)
use_pio_mode=1    → Timeout, DOUT_RDY never sets
Result: QCE locked despite no domain conflict. Abandon. ✗
```

---

## Why Each Test Matters

### Test 1: Domain Conflict Detection
**Purpose:** Confirms or refutes Gemini's theory.
**If conflict found:** Validates theory, proceed to fix.
**If no conflict:** Theory wrong, but PIO test still valuable.

### Test 2: Fix Attempt
**Purpose:** Test if XPU/PAC lock is software or hardware (eFuse).
**If fix works:** Channels can be unlocked by Linux.
**If fix fails:** Hardware permanently fused to secure world.

### Test 3: PIO Mode
**Purpose:** Isolate whether problem is in QCE hardware or ADM DMA path.
**If PIO works:** QCE itself is functional, DMA path is broken.
**If PIO fails:** QCE hardware itself is locked/non-functional.

### Test 4: Actual Crypto
**Purpose:** Confirm end-to-end functionality after fix.
**If works:** Total success, CE2 is usable.
**If hangs:** Fix wasn't sufficient, need SCM unlock or other mechanism.

---

## Expected Test Time

| Test | Time | Condition |
|------|------|-----------|
| Build & Deploy | 4 min | Always |
| Test 1: Domain | 2 min | Always |
| Test 2: Fix | 2 min | If conflict found |
| Test 3: PIO | 2 min | If fix fails OR no conflict |
| Test 4: Crypto | 2 min | If fix succeeds |
| **Total** | **7-12 min** | Depends on path |

---

## Decision Points

```
START
  │
  ├─ Test 1: Domain Conflict?
  │   │
  │   ├─ YES (Domain=3)
  │   │   └─ Test 2: Can fix?
  │   │       │
  │   │       ├─ YES → Test 4: Crypto works? → SUCCESS or NEED_SCM
  │   │       └─ NO → Test 3: PIO works? → QCE_OK_DMA_LOCKED or ALL_LOCKED
  │   │
  │   └─ NO (Domain=0)
  │       └─ Test 3: PIO works? → DEBUG_MORE or QCE_LOCKED
```

**Outcome paths:**
1. SUCCESS - Make fix permanent, use CE2
2. NEED_SCM - Need SCM unlock call, abandon for now
3. QCE_OK_DMA_LOCKED - QCE works but DMA locked, abandon
4. ALL_LOCKED - Everything locked, abandon
5. DEBUG_MORE - No conflict but QCE works, investigate further
6. QCE_LOCKED - QCE locked despite no conflict, abandon

Paths 2-6 all lead to: **Abandon CE2, use software crypto.**

---

## Why This Approach is Valuable

### If Theory is Correct:
- Explains all symptoms (DMA never completes, vendors avoid CE2)
- Potentially fixable if not eFuse-locked
- Definitive test in 12 minutes

### If Theory is Wrong:
- PIO test still isolates QCE vs DMA issues
- Rules out entire class of problems
- Provides data for next debugging step

### Either Way:
- Clear go/no-go decision point
- Stops endless debugging of unfixable hardware
- Allows focus on VIDC (more critical)

---

## Next Steps After Testing

### If Any Test Path → Abandon:

1. Remove crypto node from device tree:
   ```dts
   // arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi
   // crypto@18500000 { status = "disabled"; };
   ```

2. Update CLAUDE.md:
   ```markdown
   ## QCE Crypto Engine Status
   CE2 hardware is permanently locked to TrustZone secure world.
   Cannot be used by Linux. Use software crypto (OpenSSL).
   
   Investigation: reports/ce2-investigation/FINAL-VERDICT.md
   ```

3. Focus on VIDC video codec (actually critical for device functionality)

### If Test Path → Success:

1. Make fix permanent (remove module param):
   ```c
   if (qce->version == QCE_VERSION_CE2) {
       // Always unlock ADM channels
       ret = qce_fix_adm_domain_conflict(dev);
       if (ret) return ret;
   }
   ```

2. Test all crypto algorithms extensively

3. Write upstream patch series:
   - Patch 1: Add domain conflict detection
   - Patch 2: Add domain unlock in probe
   - Patch 3: Documentation

4. Submit to linux-crypto@vger.kernel.org

---

## Files Created

### Reports
- `GEMINI-DIAGNOSTIC-PATCH.md` - Full implementation documentation
- `QUICK-TEST-GUIDE.md` - Step-by-step testing instructions
- `IMPLEMENTATION-SUMMARY.md` - This file
- `QCE-GEMINI-ANALYSIS-REQUEST.md` - Original context sent to Gemini
- `qce-deep-dive-conclusions.md` - Investigation conclusions
- `htc-tz-comprehensive-analysis.md` - HTC TrustZone analysis
- `samsung-comprehensive-analysis.md` - Samsung modem analysis
- `SONY-SAMSUNG-FINAL-ANALYSIS.md` - Sony/Samsung findings

### Code
- `drivers/crypto/qce/core.c` - Modified with diagnostics

---

## Commit Details

**Commit:** 3b8bb4669958
**Branch:** tenderloin/6.18/upstream-patches
**Remote:** shr-github
**Date:** 2026-05-15

**Files changed:** 12
**Lines added:** ~2100 (mostly documentation)
**Lines changed in code:** ~200 (diagnostic functions + module params)

---

## Success Criteria

**Diagnostic is successful if:**
- We get a definitive answer (use CE2 or abandon it)
- We learn whether Gemini's theory was correct
- We isolate QCE vs DMA issues via PIO test
- We complete testing in < 15 minutes

**CE2 is successful if:**
- Domain conflict found AND
- Fix succeeds (Domain bits clear) AND
- `echo "test" | sha256sum` returns hash immediately

**Abandon CE2 if:**
- Fix fails (hardware eFuse-locked) OR
- PIO fails (QCE hardware locked) OR
- sha256sum still hangs after fix

---

## Historical Context

**Days spent on CE2:** ~3 days
**Firmware analyzed:** HTC TrustZone (106KB), Samsung modem (16MB), Sony bootloader
**Previous theories tested:**
- Clock enable (✓ works)
- CRCI programming (✓ works)
- Missing bootloader init (✗ couldn't find)
- Power domain (? untested)
- Interrupt routing (? untested)

**This diagnostic:** Tests entirely new angle suggested by AI

---

## Conclusion

Comprehensive diagnostic suite implementing Gemini's domain conflict theory.

**Time to answer:** 12 minutes
**Possible outcomes:** 6 (1 success, 5 abandon)
**Value:** Definitive go/no-go decision on CE2 hardware crypto

The PIO mode test alone is worth implementing - it definitively answers whether
QCE hardware works at all, regardless of domain conflict theory.

If tests confirm CE2 is locked, we can confidently abandon it and focus on VIDC,
which is far more critical for video playback on the TouchPad.
