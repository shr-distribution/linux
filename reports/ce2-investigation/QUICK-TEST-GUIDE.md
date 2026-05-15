# CE2 Quick Test Guide - 15 Minutes to Answer
## Date: 2026-05-15

---

## TL;DR

Test Gemini's domain conflict theory in 15 minutes. Three commands, three outcomes, one decision.

---

## Prerequisites

- TouchPad connected via USB (LuneOS running)
- SSH access: `ssh root@172.16.42.2`
- Kernel source at: `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin`

---

## Build & Deploy (5 minutes)

```bash
# On host
cd /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin
export ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
make M=drivers/crypto/qce clean
make M=drivers/crypto/qce modules

# Deploy
scp drivers/crypto/qce/qce.ko root@172.16.42.2:/tmp/
```

---

## Test 1: Domain Conflict Detection (5 minutes)

```bash
ssh root@172.16.42.2

# Unload current module
rmmod qce

# Load with domain test enabled
insmod /tmp/qce.ko test_adm_domain=1

# Check results
dmesg | tail -30
```

**Look for one of these:**

### Outcome A: Conflict Found ✗
```
ADM1 Channel 4 (CE_IN) register dump:
  EE=0 CH_CONF: 0x180008d5 (Domain=3)
  *** Channel 4 locked to Domain 3 (Secure World) ***
ADM1 Channel 5 (CE_OUT) register dump:
  EE=0 CH_CONF: 0x180008d5 (Domain=3)
  *** Channel 5 locked to Domain 3 (Secure World) ***
*** DOMAIN CONFLICT DETECTED ***
```

→ **Gemini was RIGHT! Proceed to Test 2.**

### Outcome B: No Conflict ✓
```
ADM1 Channel 4 (CE_IN) register dump:
  EE=0 CH_CONF: 0x000008d5 (Domain=0)
ADM1 Channel 5 (CE_OUT) register dump:
  EE=0 CH_CONF: 0x000008d5 (Domain=0)
No domain conflict detected, channels accessible to Linux.
```

→ **Gemini was wrong. Problem is elsewhere. Skip to Test 3.**

---

## Test 2: Fix Attempt (3 minutes)

**Only run if Test 1 found conflict!**

```bash
rmmod qce
insmod /tmp/qce.ko test_adm_domain=1 fix_adm_domain=1
dmesg | tail -25
```

**Look for one of these:**

### Outcome A: Fix Succeeded ✓
```
Before: CH4=0x180008d5 CH5=0x180008d5
After:  CH4=0x000008d5 CH5=0x000008d5
*** SUCCESS: Domain bits cleared, channels unlocked! ***
```

→ **VICTORY PATH! Proceed to crypto test.**

### Outcome B: Fix Failed ✗
```
Before: CH4=0x180008d5 CH5=0x180008d5
After:  CH4=0x180008d5 CH5=0x180008d5
*** FAILED: Hardware refused write, permanently locked ***
```

→ **Hardware is eFuse-locked. Proceed to Test 3 to confirm.**

---

## Test 3: PIO Mode (Isolate QCE vs DMA) (3 minutes)

```bash
rmmod qce
insmod /tmp/qce.ko use_pio_mode=1
dmesg | tail -20
```

**Look for one of these:**

### Outcome A: PIO Works ✓
```
Initial STATUS: 0x1120800d
Writing test data 0x12345678 to DATA_IN
Read result 0x12345678 from DATA_OUT
*** PIO MODE WORKS: QCE hardware is functional! ***
Problem is isolated to ADM DMA path, not QCE itself.
```

→ **QCE itself OK, but DMA is locked. If fix failed, CE2 is unusable.**

### Outcome B: PIO Fails ✗
```
Initial STATUS: 0x1120800d
Writing test data 0x12345678 to DATA_IN
*** TIMEOUT: DOUT_RDY never set ***
QCE hardware itself is locked/broken, not just DMA.
```

→ **Entire CE2 subsystem locked. Definitely abandon.**

---

## Test 4: Actual Crypto (2 minutes)

**Only run if Test 2 fix succeeded!**

```bash
# Reload with fix enabled
rmmod qce
insmod /tmp/qce.ko fix_adm_domain=1

# Test SHA256
echo "test" | sha256sum
```

**Expected:** `9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0f822cd15d6c15b0f00a08`

### Outcome A: Works Immediately ✓
```
9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0f822cd15d6c15b0f00a08  -
```

→ **TOTAL VICTORY! CE2 is now functional! Write success report.**

### Outcome B: Hangs ✗
```
[process hangs in 'D' state]
```

→ **Fix wasn't enough. Need SCM unlock or other mechanism. Abandon for now.**

---

## Decision Matrix

| Test 1 | Test 2 | Test 3 | Test 4 | Decision |
|--------|--------|--------|--------|----------|
| Conflict | Fix Success | - | Works | **Use CE2!** Make fix permanent. |
| Conflict | Fix Success | - | Hangs | Need SCM unlock. Abandon for now. |
| Conflict | Fix Failed | PIO Works | - | QCE OK, DMA locked. **Abandon CE2.** |
| Conflict | Fix Failed | PIO Fails | - | All locked. **Abandon CE2.** |
| No Conflict | - | PIO Works | - | QCE OK, problem elsewhere. Debug more. |
| No Conflict | - | PIO Fails | - | QCE locked despite no domain conflict. **Abandon CE2.** |

---

## What to Do After Testing

### If CE2 Works (Test 4 = success):

```bash
# Document exact working sequence
echo "CE2 working!" > /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/ce2-investigation/SUCCESS.md

# Make fix permanent (remove module param, always apply)
# Edit drivers/crypto/qce/core.c to always call qce_fix_adm_domain_conflict()

# Test all algorithms
for alg in md5 sha1 sha256 sha512; do
  echo "test" | ${alg}sum
done

# Write upstream patch
```

### If CE2 Fails (any path leads to abandon):

```bash
# Document failure mode
cat > /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/ce2-investigation/FINAL-VERDICT.md <<EOF
# CE2 Final Verdict - ABANDONED

## Date: $(date +%Y-%m-%d)

## Test Results
- Domain Conflict: [YES/NO]
- Fix Attempt: [SUCCESS/FAILED/SKIPPED]
- PIO Mode: [WORKS/FAILS]
- Crypto Test: [WORKS/HANGS/SKIPPED]

## Conclusion
CE2 is [permanently locked to secure world / broken / other].
Cannot be used by Linux on TouchPad.

## Action
- Remove crypto@18500000 from device tree
- Use software crypto (OpenSSL)
- Focus on VIDC video codec (more critical)
EOF

# Update CLAUDE.md
echo "" >> /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/CLAUDE.md
echo "## QCE Crypto Engine Status" >> /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/CLAUDE.md
echo "CE2 hardware is permanently locked to secure world. Use software crypto." >> /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/CLAUDE.md
```

---

## Quick Copy-Paste Script

Save this to `test-ce2.sh` on the device:

```bash
#!/bin/bash
set -e

echo "=== CE2 Quick Test Script ==="
echo ""

# Test 1: Domain conflict
echo "Test 1: Checking for domain conflict..."
rmmod qce 2>/dev/null || true
insmod /tmp/qce.ko test_adm_domain=1
if dmesg | tail -30 | grep -q "DOMAIN CONFLICT DETECTED"; then
    echo "✗ Conflict found! Attempting fix..."
    
    # Test 2: Fix attempt
    rmmod qce
    insmod /tmp/qce.ko test_adm_domain=1 fix_adm_domain=1
    if dmesg | tail -20 | grep -q "SUCCESS"; then
        echo "✓ Fix succeeded! Testing crypto..."
        
        # Test 4: Crypto
        rmmod qce
        insmod /tmp/qce.ko fix_adm_domain=1
        timeout 5 bash -c 'echo "test" | sha256sum' && echo "✓ CE2 WORKS!" || echo "✗ Still hangs"
    else
        echo "✗ Fix failed. Testing PIO mode..."
        
        # Test 3: PIO
        rmmod qce
        insmod /tmp/qce.ko use_pio_mode=1
        dmesg | tail -15 | grep -q "PIO MODE WORKS" && echo "✓ QCE OK, DMA locked" || echo "✗ QCE also locked"
    fi
else
    echo "✓ No domain conflict. Testing PIO mode..."
    
    # Test 3: PIO
    rmmod qce
    insmod /tmp/qce.ko use_pio_mode=1
    dmesg | tail -15 | grep -q "PIO MODE WORKS" && echo "✓ QCE works, problem elsewhere" || echo "✗ QCE locked"
fi

echo ""
echo "=== Test Complete ==="
echo "Review dmesg for full diagnostics"
```

---

## Time Budget

- Build: 3 min
- Deploy: 1 min
- Test 1: 2 min
- Test 2: 2 min (if needed)
- Test 3: 2 min
- Test 4: 2 min (if applicable)
- Decision: 1 min

**Total: 13 minutes worst case, 7 minutes best case**

---

## One-Liner Summary

```bash
# Full test sequence (if you're confident)
make M=drivers/crypto/qce modules && \
scp drivers/crypto/qce/qce.ko root@172.16.42.2:/tmp/ && \
ssh root@172.16.42.2 'rmmod qce; insmod /tmp/qce.ko test_adm_domain=1 fix_adm_domain=1 use_pio_mode=1; dmesg | tail -50'
```

---

## Expected Outcome

**Most likely:** Domain conflict found, fix fails, PIO works → CE2 locked but functional → **Abandon**

**Best case:** Domain conflict found, fix works, crypto works → **Use CE2!**

**Worst case:** All tests fail → **Abandon immediately**

Either way, we'll know within 15 minutes whether to continue or move on.
