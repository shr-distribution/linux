# CE2 Gemini Diagnostic Patch
## Date: 2026-05-15
## Implements: ADM Domain Conflict Testing, Fix Attempt, PIO Mode, SCM Unlock

---

## Background

After exhaustive firmware analysis (HTC, Samsung, Sony), Gemini AI suggested a new theory:
**ADM channels may be locked to Security Domain 3 (TrustZone) at EE=0, blocking Linux from using them.**

This would explain:
- Why DMA completion never arrives
- Why all vendors avoid CE2 despite having hardware
- Why writes from Linux (EE=1) are ignored/blocked by XPU/PAC hardware

---

## What This Patch Does

Adds **four diagnostic modes** controlled by module parameters:

### 1. Test ADM Domain Conflict (`test_adm_domain=1`)

**What it does:**
- Maps ADM0 and ADM1 at both EE=0 (secure) and EE=1 (non-secure)
- Reads `CH_CONF` registers for channels 4 & 5 (QCE channels)
- Checks bits 27-28 (security domain field)
- Reports if channels are locked to Domain 3 (secure world)

**Expected output if theory is correct:**
```
ADM1 Channel 4 (CE_IN) register dump:
  EE=0 CH_CONF: 0x180008d5 (Domain=3)
  *** Channel 4 locked to Domain 3 (Secure World) ***
  EE=1 CH_CONF: 0x00000000
ADM1 Channel 5 (CE_OUT) register dump:
  EE=0 CH_CONF: 0x180008d5 (Domain=3)
  *** Channel 5 locked to Domain 3 (Secure World) ***
  EE=1 CH_CONF: 0x00000000
*** DOMAIN CONFLICT DETECTED ***
```

**Memory map:**
- ADM0 EE=0: 0x18320000
- ADM0 EE=1: 0x18320800
- ADM1 EE=0: 0x18420000
- ADM1 EE=1: 0x18420800
- CH_CONF offset: 0x08 + (channel * 0x80)

### 2. Fix ADM Domain Conflict (`fix_adm_domain=1`)

**What it does:**
- Reads current `CH_CONF` values at EE=0
- Attempts to clear bits 27-28 (set domain to 0 = non-secure)
- Also toggles BCR bit 7 to clear XPU error state
- Reads back to verify if write succeeded

**Two possible outcomes:**

**Success (hardware not fused):**
```
Before: CH4=0x180008d5 CH5=0x180008d5
After:  CH4=0x000008d5 CH5=0x000008d5
*** SUCCESS: Domain bits cleared, channels unlocked! ***
```

**Failure (hardware fused to secure world):**
```
Before: CH4=0x180008d5 CH5=0x180008d5
After:  CH4=0x180008d5 CH5=0x180008d5
*** FAILED: Hardware refused write, permanently locked ***
CE2 is fused to secure world, cannot be used by Linux.
```

### 3. PIO Mode Test (`use_pio_mode=1`)

**What it does:**
- Bypasses ADM DMA entirely
- Writes test data (0x12345678) directly to `CE2_REG_DATA_IN`
- Polls `CE2_REG_STATUS` for `DOUT_RDY` bit
- Reads result from `CE2_REG_DATA_OUT`

**Purpose:** Isolate whether QCE hardware itself works, or if problem is ADM-specific.

**Two possible outcomes:**

**QCE works (DMA is the problem):**
```
Initial STATUS: 0x1120800d
Writing test data 0x12345678 to DATA_IN
Read result 0x12345678 from DATA_OUT
*** PIO MODE WORKS: QCE hardware is functional! ***
Problem is isolated to ADM DMA path, not QCE itself.
```

**QCE locked (hardware itself is broken):**
```
Initial STATUS: 0x1120800d
Writing test data 0x12345678 to DATA_IN
*** TIMEOUT: DOUT_RDY never set ***
QCE hardware itself is locked/broken, not just DMA.
```

### 4. SCM Unlock Attempt (`try_scm_unlock=1`)

**What it does:**
- Currently just documents the SCM call parameters needed
- TODO: Implement actual `qcom_scm_set_remote_state()` call

**Parameters for SCM call:**
- Service ID: `0x01` (SCM_SVC_BOOT)
- Command ID: `0x01` (Set Peripheral Access)
- Resource ID: `0x14` (CE2)
- Permission: `0x01` (Non-Secure/Linux)

---

## How to Use

### Step 1: Test for Domain Conflict
```bash
cd /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin
make M=drivers/crypto/qce modules

# Deploy to device
scp drivers/crypto/qce/qce.ko root@172.16.42.2:/tmp/

# On device
ssh root@172.16.42.2
rmmod qce
insmod /tmp/qce.ko test_adm_domain=1
dmesg | tail -30
```

**Look for:** "DOMAIN CONFLICT DETECTED" message

### Step 2: Try to Fix (if conflict found)
```bash
rmmod qce
insmod /tmp/qce.ko test_adm_domain=1 fix_adm_domain=1
dmesg | tail -20
```

**Look for:** "SUCCESS: Domain bits cleared" or "FAILED: Hardware refused write"

### Step 3: Test PIO Mode (isolate QCE vs DMA)
```bash
rmmod qce
insmod /tmp/qce.ko use_pio_mode=1
dmesg | tail -20
```

**Look for:** "PIO MODE WORKS" or "TIMEOUT: DOUT_RDY never set"

### Step 4: Test Crypto After Fix
```bash
# If fix succeeded, test actual crypto operation
echo "test" | sha256sum
# Should return: 9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08

# Check if it hangs or completes
```

---

## Decision Tree

```
1. Run test_adm_domain=1
   ├─ No conflict found
   │  └─ Problem is elsewhere (interrupts, CRCI, power domain)
   │     └─ ABANDON QCE, use software crypto
   │
   └─ Conflict found (Domain 3)
      └─ Run fix_adm_domain=1
         ├─ Fix succeeds
         │  ├─ Run use_pio_mode=1 → works
         │  │  └─ Test sha256sum → works
         │  │     └─ VICTORY! CE2 now functional!
         │  │
         │  └─ Test sha256sum → still hangs
         │     └─ Need SCM unlock or other fix
         │
         └─ Fix fails (hardware locked)
            └─ Run use_pio_mode=1
               ├─ PIO works
               │  └─ QCE itself OK, but ADM permanently locked
               │     └─ ABANDON CE2, use software crypto
               │
               └─ PIO fails
                  └─ Entire CE2 subsystem locked to secure world
                     └─ ABANDON CE2, use software crypto
```

---

## Expected Timeline

**Quick path (15 minutes):**
1. Test domain conflict - 5 min
2. Attempt fix - 5 min
3. Test PIO mode - 5 min
4. Make decision (continue or abandon)

**If fix works:** Continue with crypto testing
**If fix fails:** Abandon CE2, document findings, focus on VIDC

---

## Code Changes

### File: `drivers/crypto/qce/core.c`

**Added module parameters:**
```c
static bool qce_test_adm_domain = false;
module_param_named(test_adm_domain, qce_test_adm_domain, bool, 0644);

static bool qce_fix_adm_domain = false;
module_param_named(fix_adm_domain, qce_fix_adm_domain, bool, 0644);

static bool qce_use_pio_mode = false;
module_param_named(use_pio_mode, qce_use_pio_mode, bool, 0644);

static bool qce_try_scm_unlock = false;
module_param_named(try_scm_unlock, qce_try_scm_unlock, bool, 0644);
```

**Added diagnostic functions:**
- `qce_test_adm_domain_conflict()` - ~80 lines
- `qce_fix_adm_domain_conflict()` - ~60 lines
- `qce_test_pio_mode()` - ~50 lines

**Modified probe function:**
- Calls diagnostics after CRCI programming
- Only runs if module parameters are set
- Fails probe if unfixable conflict found

---

## Register Reference

### ADM CH_CONF Register Format
```
Bits 31-28: Reserved
Bits 27-28: Security Domain
            00 = Domain 0 (Non-Secure, Linux)
            01 = Domain 1 (Reserved)
            10 = Domain 2 (Reserved)
            11 = Domain 3 (Secure World, TrustZone)
Bits 0-26:  Other channel config
```

### CE2 STATUS Register (0x20)
```
Bit 7: DOUT_INTR    - Output interrupt
Bit 6: DIN_INTR     - Input interrupt
Bit 5: AUTH_DONE    - Auth complete interrupt
Bit 4: ERR_INTR     - Error interrupt
Bit 3: DOUT_RDY     - Output data ready
Bit 2: DIN_RDY      - Input data ready
Bit 1: AUTH_DONE    - Auth complete (non-interrupt)
Bit 0: SW_ERR       - Software error
```

---

## Why This Matters

If Gemini's theory is correct:
1. **Explains everything** - why DMA never completes, why vendors avoid CE2
2. **Testable in 15 minutes** - clear yes/no answer
3. **Potentially fixable** - if not fused, can unlock channels
4. **Definitive answer** - if hardware is fused, we know to abandon CE2

If theory is wrong:
- Still learn valuable info about ADM configuration
- PIO test isolates QCE vs DMA issues
- Can confidently move to other theories or abandon

---

## Next Steps After Testing

### If Fix Works:
1. Document exact sequence that worked
2. Make fix permanent (remove module param, always apply)
3. Test all crypto algorithms
4. Submit patch upstream
5. Write success report

### If Fix Fails:
1. Document exact failure mode
2. Update investigation reports with findings
3. **Officially abandon CE2 hardware crypto**
4. Remove CE2 support from device tree
5. Focus on VIDC video codec (more critical)
6. Add to CLAUDE.md: "CE2 permanently locked to secure world, use software crypto"

---

## Files Modified

- `drivers/crypto/qce/core.c` - Added diagnostics and module params

## Files Created

- `reports/ce2-investigation/GEMINI-DIAGNOSTIC-PATCH.md` (this file)

---

## Commit Message

```
crypto: qce: Add CE2 diagnostic modes for domain conflict testing

Implements Gemini AI theory that ADM channels may be locked to Security
Domain 3 (TrustZone) at EE=0, preventing Linux from using them.

Adds four module parameters:
- test_adm_domain: Test for domain conflict
- fix_adm_domain: Attempt to clear domain bits
- use_pio_mode: Test PIO mode (bypass DMA)
- try_scm_unlock: Document SCM call parameters

This allows quick testing (15 min) to either:
1. Fix CE2 if channels are unlockable
2. Definitively abandon CE2 if hardware is fused

Based on analysis that all MSM8660 vendors (HTC, Samsung, Sony) avoid
CE2 hardware despite having it available, suggesting security lockout.

Reference: reports/ce2-investigation/GEMINI-DIAGNOSTIC-PATCH.md
```

---

## Testing Checklist

- [ ] Compile kernel module
- [ ] Deploy to TouchPad
- [ ] Test 1: Domain conflict detection
- [ ] Test 2: Domain fix attempt
- [ ] Test 3: PIO mode test
- [ ] Test 4: Crypto operation after fix
- [ ] Document results in follow-up report
- [ ] Make go/no-go decision on CE2

---

## Conclusion

This patch implements a comprehensive diagnostic suite to test Gemini's domain
conflict theory. Within 15 minutes of testing, we'll have a definitive answer
on whether CE2 can be fixed or must be abandoned.

The PIO mode test is particularly valuable - if that works, we know QCE hardware
itself is functional and the problem is purely in the ADM DMA security lockout.

**Time investment:** 15 minutes of testing to answer a question that's consumed
days of firmware analysis.

**Worst case:** Confirms CE2 is permanently locked, allowing us to confidently
abandon it and focus on VIDC.

**Best case:** Unlocks CE2, enabling hardware crypto acceleration on TouchPad.
