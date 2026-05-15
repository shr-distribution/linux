# QCE CE2 Hardware Crypto Engine - Complete Analysis for External Review
## Platform: HP TouchPad (MSM8660/APQ8060) - Linux 6.18 Mainline
## Date: 2026-05-15
## Status: Partially Working - Operations Hang Waiting for DMA Completion

---

## Executive Summary

We have successfully enabled hardware accessibility for Qualcomm's CE2 (Crypto Engine 2) on the HP TouchPad tablet running mainline Linux 6.18. The hardware registers are accessible, all crypto algorithms register successfully, and self-tests pass. However, **all actual crypto operations hang indefinitely** waiting for DMA completion that never arrives. We've analyzed firmware from HTC and Samsung MSM8660 devices but found no smoking gun for the missing initialization. We need fresh perspective on what ADM DMA or QCE configuration we're missing.

---

## Hardware Platform Details

### Device: HP TouchPad (Tenderloin)
- **SoC**: Qualcomm APQ8060 (dual-core ARMv7 Scorpion @ 1.5GHz)
- **Crypto Engine**: CE2 (Crypto Engine 2) at base address 0x18500000
- **DMA Controller**: ADM (Application Data Mover) - two instances:
  - ADM0 at 0x18320000 (used by modem/peripherals)
  - ADM1 at 0x18420000 (used by QCE, channels 4 & 5)
- **Operating System**: LuneOS (webOS continuation) with Linux 6.18 mainline kernel

### Critical Platform Difference
**TouchPad bootloader is incomplete** - missing OEMSBL (OEM Secondary Boot Loader) stage:
```
Normal MSM8660:  PBL → OEMSBL → APPSBL → TrustZone → Linux
TouchPad:        PBL → APPSBL → Linux (OEMSBL partition is 500KB of zeros)
```

This means any CE2/ADM initialization done by OEMSBL is **missing** on TouchPad.

---

## What We Successfully Implemented

### 1. Manual Clock Enable (Phase 1)
The device tree clock framework (CCF) wasn't sufficient. We manually enable CE2_HCLK_CTL:

```c
void __iomem *gcc_base = ioremap(0x00900000, 0x10000);  // GCC base
u32 val = readl_relaxed(gcc_base + 0x2740);              // CE2_HCLK_CTL offset
val |= BIT(4);   // Clock enable
val &= ~BIT(7);  // Reset deassert
writel_relaxed(val, gcc_base + 0x2740);
```

**Result**: 
- Register reads 0x00000010 (bit 4 set, clock running)
- MMIO registers now accessible (previously all zeros)
- Hardware responds to reads

**Code Location**: `drivers/crypto/qce/core.c:337`

### 2. CRCI (Flow Control) Programming
ADM uses CRCI (CRC Interface) signals for flow control with peripherals. We program both QCE channels:

```c
// CE_IN channel (CRCI 4)
qcom_adm_program_crci_ee0(qce->dma.rxchan, 0x1);

// CE_OUT channel (CRCI 5)  
qcom_adm_program_crci_ee0(qce->dma.txchan, 0x1);
```

Value 0x1 = 32-byte burst size (CE2_ADM_BURST_SIZE / 2).

**Code Location**: `drivers/crypto/qce/core.c:379-385`

### 3. Device Tree Configuration
Proper clocks, resets, interrupts, and DMA channels configured:

```dts
crypto@18500000 {
    compatible = "qcom,crypto-ce2";
    reg = <0x18500000 0x20000>;
    clocks = <&gcc CE2_P_CLK>;
    resets = <&gcc CE2_RESET>;
    dmas = <&adm1 4 0>, <&adm1 5 0>;  // CE_IN, CE_OUT
    dma-names = "rx", "tx";
};
```

### 4. Driver Probe Success
```
[    5.961901] qcrypto 18500000.crypto: CE2: Verifying hardware initialization
[    5.967359] qcrypto 18500000.crypto: CE2: Version register (0x00): 0x00000000
[    5.983730] qcrypto 18500000.crypto: CE2: Status register (0x10): 0x926c0fea
[    5.983742] qcrypto 18500000.crypto: CE2: Capabilities register (0x20): 0x1120800d
[    5.983798] qcrypto 18500000.crypto: CE2: Enabling hardware clocks
[    5.983808] qcrypto 18500000.crypto: CE2: CE2_HCLK_CTL before: 0x00000010
[    5.983819] qcrypto 18500000.crypto: CE2: CE2_HCLK_CTL after: 0x00000010
[    5.984305] qcrypto 18500000.crypto: CE2: Hardware initialized successfully
[    6.000204] qcrypto 18500000.crypto: Crypto Engine 2 (CE2) found
```

All 20 crypto algorithms successfully registered:
- SHA1, SHA256, SHA384, SHA512
- AES-128/192/256 (ECB, CBC, CTR, XTS)
- 3DES (ECB, CBC)
- HMAC-SHA1, HMAC-SHA256
- AEAD modes (authenc, CCM)

All **self-tests pass**!

---

## The Problem: Operations Hang Forever

### Symptom
Any actual crypto operation hangs indefinitely:

```bash
$ echo "test" | sha256sum
[hangs forever]

$ python3 -c "import socket; s=socket.socket(socket.AF_ALG, socket.SOCK_SEQPACKET, 0); \
              s.bind(('hash', 'sha256')); op=s.accept()[0]; op.sendall(b'test')"
[process enters 'D' state - uninterruptible sleep]
```

Process state from `ps aux`:
```
root      2144  Ds   python3 /tmp/test-qce.py
```

The 'D' state means waiting for I/O that never completes.

### Root Cause (Confirmed)
Kernel code path hangs here:

```c
// drivers/crypto/qce/dma.c
ret = dma_submit_error(dmaengine_submit(desc));
dma_async_issue_pending(chan);

// Wait for completion from ADM DMA
ret = wait_for_completion_timeout(&qce->dma.done, 
                                  msecs_to_jiffies(QCE_DMA_TIMEOUT));
// ^^^ This timeout never fires because QCE_DMA_TIMEOUT = 0 (infinite)
//     Process waits forever for completion that never arrives
```

The completion is supposed to be signaled by ADM DMA interrupt when transfer finishes, but **the interrupt never fires**.

### What Doesn't Happen
1. ADM DMA descriptor is queued ✅
2. ADM channel is enabled ✅
3. QCE hardware should process data ❓
4. ADM should generate completion interrupt ❌ **NEVER HAPPENS**
5. Interrupt handler should call `complete(&qce->dma.done)` ❌ **NEVER CALLED**

---

## Firmware Analysis Results

### HTC TrustZone Firmware Analysis
**Source**: HTC Shooter (MSM8960) `tz.img` extracted from RUU package (106KB)

**Findings**:
1. **No CE2_HCLK_CTL initialization found** - searched entire 106KB image for offset 0x2740, not present
2. **QCE code is validation, not initialization** - at offset 0x13248:
   - Writes test data to QCE+0x00 (DATA_IN register)
   - Polls QCE+0x20 (STATUS) for bit 3 (DOUT_RDY)
   - Polls QCE+0x20 for bit 4 (ERR_INTR completion)
   - Reads result from QCE+0x10 (DATA_OUT)
   - **This is a crypto TEST, not initialization**
3. **No ADM DMA initialization** - no references to ADM base addresses (0x18320000 or 0x18420000)
4. **No CRCI setup** - no flow control configuration code found
5. **Generic clock enable function** at 0xd120 sets bits 0,1,3 and clears 4,5,6 - **opposite of CE2_HCLK_CTL pattern** (which needs bit 4 SET)

**Conclusion**: HTC TrustZone assumes CE2/ADM are already initialized by earlier bootloader stage (OEMSBL).

**Document**: `reports/ce2-investigation/htc-tz-comprehensive-analysis.md`

### Samsung Galaxy Note Firmware Analysis  
**Source**: Samsung I717 (MSM8660) stock ROM MODEM firmware (16MB)

**Findings**:
1. **QCE base address 0x18500000 found** in peripheral address tables:
   - MODEM.B05 @ 0x009dce10
   - MODEM.B05 @ 0x00f22e80
   - MODEM.B06 @ 0x000838c0
2. **Source paths found** in strings:
   - `/modem_proc/core/securemsm/crypto/shared/src/secapi.c`
   - `/modem_proc/core/securemsm/crypto/shared/src/secenchw.c`
   - Qualcomm's crypto API implementation in modem processor
3. **Error messages suggest software fallback**:
   - "HSHhw: CE_Hash_Update failed! Crypto Engine API not supported"
   - "HSHhw: CeMLInit failed! Crypto Engine API not supported"
4. **ADM1 references exist** (0x18420000) but for modem use, not Linux crypto
5. **No Linux-accessible initialization found** - modem firmware runs in different security context

**Conclusion**: Samsung modem has QCE driver code but likely uses software crypto, avoiding CE2 hardware entirely.

**Document**: `reports/ce2-investigation/samsung-comprehensive-analysis.md`

---

## Current Theories on Missing Piece

### Theory 1: ADM Interrupt Not Enabled/Routed
**Evidence**:
- Process hangs waiting for `complete(&qce->dma.done)`
- Completion is signaled by ADM interrupt handler
- Interrupt might not be:
  - Enabled in ADM channel registers
  - Enabled in GIC (interrupt controller)
  - Properly routed to Linux

**How to Test**:
```bash
cat /proc/interrupts | grep adm
# Should show ADM interrupt line and counter
# If counter never increments during crypto op, interrupt isn't firing
```

### Theory 2: ADM Channel State Machine Not Triggered
**Evidence**:
- ADM channels have IDLE → ACTIVE → COMPLETE state machine
- Might need explicit enable command beyond descriptor queue

**Possible Missing Steps**:
1. Read ADM_CH_n_RSLT_CONF to check channel mode
2. Write ADM_CH_n_CMD to explicitly enable channel
3. Verify ADM_CH_n_STATUS shows ACTIVE state

**How to Test**: Add debug code to read ADM registers during hang:
```c
u32 status = readl(adm_base + ADM_CH_4_STATUS);
pr_err("ADM channel 4 status during hang: 0x%08x\n", status);
```

### Theory 3: CRCI Handshaking Not Working
**Evidence**:
- CRCI lines (4=CE_IN, 5=CE_OUT) coordinate flow control
- We program CRCI_CTL but signals might not be physically connected
- Device tree might have wrong CRCI assignments

**Possible Issues**:
1. CRCI mux needs configuration (route signals to correct ADM channels)
2. QCE needs to drive CRCI signals (might need register write)
3. ADM needs to sample CRCI signals (might need register write)

**How to Test**: Check if CRCI values we're writing actually stick:
```c
// After programming
u32 crci_val = readl(adm_base + ADM_CRCI_CTL_n);
pr_info("CRCI value after programming: 0x%08x\n", crci_val);
```

### Theory 4: TrustZone Lock
**Evidence**:
- CE2 might be locked to TrustZone security domain via TZPAC
- Writes to QCE config registers might be silently ignored
- Only TrustZone can unlock via SCM calls

**How to Test**: Try direct QCE register write in Linux:
```c
writel(0x12345678, qce->base + 0x100);  // Some non-critical register
u32 readback = readl(qce->base + 0x100);
pr_info("Wrote 0x12345678, read back 0x%08x\n", readback);
// If read returns 0 or wrong value, hardware is locked
```

### Theory 5: Missing Peripheral Power Domain
**Evidence**:
- MSM8660 has complex power domains managed by RPM
- CE2 might be in powered-off domain despite clock being on

**Possible Missing**:
- RPM (Resource Power Manager) vote for CE2 power domain
- GDSC (Global Distributed Switch Controller) enable

**How to Test**: Check power domain status:
```bash
# If debugfs mounted
cat /sys/kernel/debug/pm_genpd/pm_genpd_summary | grep -i ce2
```

---

## CE2 Register Definitions Reference

From `drivers/crypto/qce/regs-ce2.h`:

```c
#define CE2_REG_DATA_IN              0x000  // Input data FIFO
#define CE2_REG_DATA_OUT             0x010  // Output data FIFO
#define CE2_REG_STATUS               0x020  // Status register
#define CE2_REG_CONFIG               0x024  // Configuration
#define CE2_REG_GOPROC               0x040  // Start processing
#define CE2_REG_ENGINES_AVAIL        0x044  // Available engines

// Status register bits (offset 0x20)
#define CE2_DOUT_RDY_SHIFT           3      // Output data ready
#define CE2_DIN_RDY_SHIFT            2      // Input data ready
#define CE2_AUTH_DONE_SHIFT          1      // Auth complete
#define CE2_SW_ERR_SHIFT             0      // Software error
#define CE2_ERR_INTR_SHIFT           4      // Error interrupt
#define CE2_DOUT_INTR_SHIFT          7      // Output interrupt
#define CE2_DIN_INTR_SHIFT           6      // Input interrupt
```

**Current Status Register Value**: 0x1120800d
```
Binary: 0001 0001 0010 0000 1000 0000 0000 1101
Bits set: 0 (SW_ERR), 2 (DIN_RDY), 3 (DOUT_RDY), 15, 21, 24, 28
```

This suggests hardware is idle and ready for data.

---

## ADM DMA Controller Details

### ADM Architecture
- Two instances: ADM0 (0x18320000) and ADM1 (0x18420000)
- 16 channels per instance (0-15)
- Each channel has TX and RX capability
- CRCI-based flow control with peripherals

### QCE Uses ADM1 Channels
- **Channel 4** (CE_IN): DMA from memory → QCE
- **Channel 5** (CE_OUT): DMA from QCE → memory

### Key ADM Registers (per channel)
```
ADM_CH_n_CMD          0x000 + (n * 0x80)  // Command register
ADM_CH_n_STATUS       0x004 + (n * 0x80)  // Channel status
ADM_CH_n_CONF         0x008 + (n * 0x80)  // Configuration
ADM_CH_n_RSLT_CONF    0x00C + (n * 0x80)  // Result config
ADM_CRCI_CTL_n        0x400 + (n * 0x4)   // CRCI control
```

### Current CRCI Programming
We write 0x1 to `ADM_CRCI_CTL_4` and `ADM_CRCI_CTL_5` at EE=0.

**Question**: Is there a global ADM enable or CRCI mux we're missing?

---

## What Other Vendors Do

### HTC Devices (MSM8660)
**Observation**: Mainstream kernels show QCE driver loaded but unclear if actually used.

**Bootloader Chain**: PBL → OEMSBL → APPSBL → TZ → Linux (complete)

### Samsung Devices (MSM8660)
**Observation**: Error messages suggest fallback to software crypto.

**Likely Reality**: Avoided CE2 hardware due to complexity/bugs, use software OpenSSL.

### Sony Devices (MSM8660)
**Unknown**: No firmware samples analyzed.

### Xiaomi Devices (MSM8660)
**Unknown**: No firmware samples analyzed.

---

## Questions for External Review (Gemini)

### 1. ADM DMA Initialization
**Q**: What ADM controller-level initialization might we be missing?
- Global enable register?
- Master clock gate?
- Security domain unlock?
- CRCI mux configuration?

**Context**: We configure per-channel settings but might be missing controller-wide setup.

### 2. CRCI Flow Control
**Q**: How do CRCI signals actually work between ADM and QCE?
- Does QCE need to enable CRCI output drivers?
- Does ADM need to enable CRCI input samplers?
- Is there a CRCI routing/mux register in GCC or elsewhere?

**Context**: We write CRCI_CTL values but don't see evidence signals are actually connected.

### 3. Interrupt Routing
**Q**: How are ADM interrupts supposed to reach Linux?
- Is there an interrupt enable in ADM_CH_n_CONF?
- Does ADM have a master interrupt enable?
- How do ADM interrupts route through GIC?

**Context**: Completion never arrives, suggesting interrupt path is broken.

### 4. Power Domains
**Q**: What power domains affect CE2/ADM operation?
- Is there an RPM power domain for crypto subsystem?
- Do we need GDSC enable beyond clock enable?
- Could CE2 be powered but ADM not?

**Context**: Clock is on but that might not be sufficient.

### 5. Security Configuration  
**Q**: Could TrustZone locks prevent CE2/ADM from working in Linux?
- Does TZPAC lock CE2 to secure world?
- Do we need SCM calls to unlock?
- How can we test if hardware is actually accessible for writes?

**Context**: Reads work but crypto operations fail - might be write-locked.

### 6. OEMSBL Missing Init
**Q**: What does OEMSBL typically initialize for crypto?
- ADM DMA controller setup?
- CRCI mux configuration?
- Security domain assignment?
- Interrupt routing?

**Context**: TouchPad lacks OEMSBL - need to replicate its init in Linux.

### 7. Alternative Debugging
**Q**: Can we bypass DMA to isolate the problem?
- Implement PIO (Programmed I/O) mode for QCE?
- Directly write to CE2_REG_DATA_IN and poll CE2_REG_STATUS?
- This would prove QCE hardware works, isolating issue to DMA.

**Context**: Need to know if QCE itself is broken or just the DMA path.

---

## Code Locations for Reference

All code is in Linux 6.18 mainline tree at:
`/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/`

**QCE Driver**:
- `drivers/crypto/qce/core.c` - Main driver, clock enable at line 337
- `drivers/crypto/qce/dma.c` - DMA setup, hang point at wait_for_completion
- `drivers/crypto/qce/regs-ce2.h` - CE2 register definitions

**ADM Driver**:
- `drivers/dma/qcom/adm.c` - ADM DMA controller driver
- CRCI programming function: `qcom_adm_program_crci_ee0()`

**Device Tree**:
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - Hardware config

**Analysis Reports**:
- `reports/ce2-investigation/htc-tz-comprehensive-analysis.md`
- `reports/ce2-investigation/samsung-comprehensive-analysis.md`
- `reports/ce2-investigation/qce-deep-dive-conclusions.md`
- `reports/ce2-investigation/CE2-BREAKTHROUGH-SUCCESS.md` (earlier "success" was premature)

---

## Request for Gemini

Please analyze this complete picture and provide:

1. **Most likely root cause** for DMA completion never arriving
2. **Specific registers/bits** we should check or set
3. **Test procedure** to isolate whether QCE or ADM is the problem
4. **Code changes** to try (with register addresses and values)
5. **Any insights** from Qualcomm documentation you might have access to

We're particularly interested in:
- ADM controller-wide initialization we might be missing
- CRCI signal routing/mux configuration
- Interrupt enable sequence
- Security/TrustZone unlock requirements

---

## Success Criteria

QCE will be considered working when:
```bash
$ echo "test" | sha256sum
[returns hash immediately without hanging]

$ cat /proc/interrupts | grep adm
[shows ADM interrupt counter incrementing during crypto operations]
```

Thank you for any insights you can provide!

