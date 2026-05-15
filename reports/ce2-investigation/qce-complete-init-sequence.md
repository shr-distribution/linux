# Complete QCE Initialization Sequence
## Extracted from HTC TrustZone Firmware Analysis
## Date: 2026-05-15

## Summary

By reverse-engineering HTC's TrustZone firmware (tz.img), we have identified the complete initialization sequence that TouchPad is missing.

---

## Phase 1: Clock Enable (GCC Registers)

### Function: Generic Clock Enable
**Location in HTC TZ**: 0xd120-0xd1c0

```c
static int qce_enable_clocks_hw(void __iomem *gcc_base, u32 offset)
{
    u32 val;
    
    /* Read current clock control register */
    val = readl_relaxed(gcc_base + offset);
    
    /* Set enable bits */
    val |= BIT(0);   /* Root clock enable */
    val |= BIT(1);   /* Branch enable (CBCR) */
    val |= BIT(3);   /* HCLK enable */
    
    /* Clear disable/reset bits */
    val &= ~BIT(4);  /* Ungating */
    val &= ~BIT(5);  /* Power domain active */
    val &= ~BIT(6);  /* Wake from sleep */
    
    /* Write back */
    writel_relaxed(val, gcc_base + offset);
    
    /* Wait for clock to be ungated */
    usleep_range(100, 200);
    
    return 0;
}
```

**For CE2 on MSM8660:**
```c
qce_enable_clocks_hw(gcc_base, 0x2740);  /* CE2_HCLK_CTL */
```

---

## Phase 2: Peripheral Configuration (QCE Registers)

### Function: CE2 Peripheral Init
**Location in HTC TZ**: 0x13200-0x13400

```c
static int qce_peripheral_init(void __iomem *qce_base)
{
    u32 status;
    int timeout;
    
    /* Step 1: Wait for peripheral ready (status bit 3) */
    timeout = 1000;
    while (timeout--) {
        status = readl_relaxed(qce_base + 0x20);  /* QCE_STATUS */
        if (status & BIT(3))
            break;
        udelay(10);
    }
    if (timeout <= 0) {
        pr_err("QCE: Timeout waiting for ready\n");
        return -ETIMEDOUT;
    }
    
    /* Step 2: Write configuration to command register */
    /* Value depends on operation mode - for now use reset value */
    writel_relaxed(0x00000001, qce_base + 0x00);  /* QCE_CONFIG */
    
    /* Step 3: Wait for operation complete (status bit 4) */
    timeout = 1000;
    while (timeout--) {
        status = readl_relaxed(qce_base + 0x20);  /* QCE_STATUS */
        if (status & BIT(4))
            break;
        udelay(10);
    }
    if (timeout <= 0) {
        pr_err("QCE: Timeout waiting for init complete\n");
        return -ETIMEDOUT;
    }
    
    /* Step 4: Read initialization result */
    status = readl_relaxed(qce_base + 0x10);  /* QCE_RESULT */
    if (status != 0) {
        pr_warn("QCE: Init result = 0x%08x\n", status);
    }
    
    return 0;
}
```

---

## Phase 3: Complete Probe Sequence

### For Linux Kernel Driver

```c
static int qce_crypto_probe(struct platform_device *pdev)
{
    struct qce_device *qce;
    void __iomem *gcc_base;
    u32 val;
    int ret;
    
    /* ... existing driver init ... */
    
    /* CE2-specific initialization (MSM8660 only) */
    if (qce->version == QCE_VERSION_CE2) {
        dev_info(dev, "Performing CE2 hardware initialization\n");
        
        /* Map GCC registers */
        gcc_base = ioremap(0x00900000, 0x10000);
        if (!gcc_base) {
            dev_err(dev, "Failed to map GCC registers\n");
            return -ENOMEM;
        }
        
        /* Phase 1: Enable clocks at hardware level */
        dev_info(dev, "Phase 1: Enabling CE2 clocks\n");
        
        val = readl_relaxed(gcc_base + 0x2740);  /* CE2_HCLK_CTL */
        dev_info(dev, "CE2_HCLK_CTL before: 0x%08x\n", val);
        
        /* Apply clock enable sequence from HTC TZ */
        val |= BIT(0);   /* Root enable */
        val |= BIT(1);   /* Branch enable */
        val |= BIT(3);   /* HCLK enable */
        val &= ~BIT(4);  /* Ungating */
        val &= ~BIT(5);  /* Power active */
        val &= ~BIT(6);  /* Wake */
        val &= ~BIT(7);  /* Deassert reset */
        writel_relaxed(val, gcc_base + 0x2740);
        
        val = readl_relaxed(gcc_base + 0x2740);
        dev_info(dev, "CE2_HCLK_CTL after:  0x%08x\n", val);
        
        /* Wait for clock to stabilize */
        usleep_range(100, 200);
        
        /* Check halt status */
        val = readl_relaxed(gcc_base + 0x2fd4);  /* CE2_HALT_STATUS */
        dev_info(dev, "CE2_HALT_STATUS: 0x%08x %s\n",
                 val, (val & BIT(0)) ? "(HALTED)" : "(RUNNING)");
        
        iounmap(gcc_base);
        
        /* Phase 2: Initialize CE2 peripheral */
        dev_info(dev, "Phase 2: Initializing CE2 peripheral\n");
        
        ret = qce_peripheral_init(qce->base);
        if (ret) {
            dev_err(dev, "CE2 peripheral init failed: %d\n", ret);
            return ret;
        }
        
        /* Phase 3: Verify MMIO is accessible */
        dev_info(dev, "Phase 3: Verifying CE2 MMIO access\n");
        
        val = readl_relaxed(qce->base + 0x00);
        dev_info(dev, "CE2 MMIO @ 0x00: 0x%08x %s\n",
                 val,
                 (val == 0) ? "(FAIL - still zero)" : "(SUCCESS - readable)");
        
        if (val == 0) {
            dev_err(dev, "CE2 hardware not responding after init\n");
            /* Don't fail probe - allow driver to load but mark as broken */
        } else {
            dev_info(dev, "CE2 hardware initialized successfully!\n");
        }
    }
    
    /* ... continue with normal probe ... */
    
    return 0;
}
```

---

## What Each Phase Does

### Phase 1: Clock Enable
**Problem**: TouchPad bootloader doesn't enable CE2 clocks.  
**Solution**: Manually write to GCC CE2_HCLK_CTL register.  
**Effect**: Powers up CE2 clock domain.

### Phase 2: Peripheral Init
**Problem**: TouchPad bootloader doesn't configure CE2 peripheral.  
**Solution**: Write to QCE control registers, poll for ready.  
**Effect**: Initializes CE2 internal state machine.

### Phase 3: Verification
**Problem**: Previous attempts failed silently.  
**Solution**: Read MMIO after init to confirm hardware responds.  
**Effect**: Proves whether init was successful.

---

## Testing Plan

### Test 1: Clock Enable Only
Add Phase 1 code to driver, test if MMIO becomes readable.

**Expected outcomes:**
- **Success**: MMIO returns non-zero → only clock was missing
- **Failure**: MMIO still zeros → peripheral init also needed

### Test 2: Full Init Sequence
Add Phases 1+2, test if MMIO becomes readable.

**Expected outcomes:**
- **Success**: MMIO readable → complete init sequence works!
- **Failure**: MMIO still zeros → hardware permanently locked

### Test 3: Crypto Operation
If MMIO is readable, test actual crypto operation:
```bash
# On TouchPad device
modprobe tcrypt mode=10 sec=1  # AES test
dmesg | grep qce
```

**Expected outcomes:**
- **Success**: Crypto tests pass → QCE fully functional!
- **Failure**: Hangs or errors → DMA or CRCI issue

---

## Key Differences: HTC vs TouchPad

| Aspect | HTC (Working) | TouchPad (Broken) | Fix |
|--------|---------------|-------------------|-----|
| **TZ firmware** | 106KB kernel | Empty partition | N/A (cannot add TZ) |
| **Clock enable** | TZ does at boot | Never happens | ✅ Phase 1 |
| **Peripheral init** | TZ does at boot | Never happens | ✅ Phase 2 |
| **MMIO accessible** | Yes (TZ enables) | No (all zeros) | Test after fixes |

---

## Implementation Status

### Current Code (linux-6.18-tenderloin)

**File**: `drivers/crypto/qce/core.c`

**Status**: Partial Phase 1 implemented (lines 267-318):
- ✅ Manual CE2_HCLK_CTL write (bit 4 enable, bit 7 deassert reset)
- ✅ Halt status check
- ✅ MMIO read test
- ❌ Missing: HTC bit pattern (bits 0,1,3,5,6)
- ❌ Missing: Phase 2 peripheral init

### Next Commit

**Update Phase 1** to match HTC TZ bit pattern:
```c
val |= BIT(0) | BIT(1) | BIT(3);     /* Enable bits */
val &= ~(BIT(4) | BIT(5) | BIT(6));  /* Clear disable bits */
val &= ~BIT(7);                      /* Deassert reset */
```

**Add Phase 2** peripheral initialization:
```c
ret = qce_peripheral_init(qce->base);
if (ret) {
    dev_warn(dev, "CE2 peripheral init failed, crypto may not work\n");
}
```

---

## Success Criteria

| Milestone | Test | Pass Condition |
|-----------|------|----------------|
| **M1: Clock enable** | Boot kernel, check dmesg | CE2_HCLK_CTL != 0x00 |
| **M2: Halt cleared** | Boot kernel, check dmesg | CE2_HALT_STATUS bit 0 = 0 |
| **M3: MMIO readable** | Boot kernel, check dmesg | CE2 MMIO @ 0x00 != 0x00 |
| **M4: Peripheral ready** | Boot kernel, check dmesg | Phase 2 completes without timeout |
| **M5: Crypto works** | `modprobe tcrypt mode=10` | Tests pass, no hangs |

---

## Fallback Plan

If all three phases fail and MMIO remains zero:

1. Document findings in kernel commit message
2. Disable QCE in device tree (`status = "disabled";`)
3. Submit upstream with explanation of investigation
4. Accept that QCE is unfixable (hardware-locked or eFuse-disabled)

**Evidence to include:**
- Empty TZ partition proving missing init stage
- HTC TZ analysis proving init sequence exists
- TouchPad bootloader analysis proving channels configured but not enabled
- Vendor binary analysis proving no one successfully used QCE
- Manual init attempt proving even full sequence doesn't work

This level of documentation will prevent future developers from repeating this investigation.

---

## References

- `/tmp/htc-tz-ce2-init-analysis.md` - Disassembly analysis
- `/tmp/htc-tz-vs-touchpad-comparison.md` - HTC vs TouchPad comparison
- `/tmp/qce-final-bootloader-analysis.md` - TouchPad bootloader analysis
- `/tmp/touchpad-tz-extraction-results.md` - Empty TZ partition discovery

## Next Action

**Implement updated Phase 1 + add Phase 2, commit, deploy, test.**
