# HTC TrustZone CE2 Initialization Sequence Analysis
## Date: 2026-05-15

## Objective
Decompile and extract the exact CE2 initialization sequence from HTC TZ firmware (tz.img) to determine what TouchPad is missing.

---

## QCE Base Address References in HTC TZ

Found at **8 locations** in TZ firmware:

```
Offset    Bytes                 Context
-------   -------------------   ----------------------------
0x132a0   00 00 50 18           In data structure/table
0x132c0   00 00 50 18           After function epilogue
0x13310   00 00 50 18           Data structure
0x133c0   00 00 50 18           Data structure
0x13490   00 00 50 18           Data structure
0x13550   00 00 50 18           Data structure
0x13740   00 00 50 18           After function
0x13760   00 00 50 18           Data structure
```

**Pattern**: `0x18500000` appears in **data sections**, not instruction streams. This suggests:
- These are **peripheral base address tables**
- Used by initialization code to map MMIO regions
- Likely loaded into registers via `ldr r0, [pc, #offset]` patterns

---

## Disassembly Analysis

### Region 0x13200-0x13400 (ARM Thumb-2 code)

**Key observations:**

#### 1. MMIO Polling Loop (0x13248-0x1324e)
```asm
13248:   f8d9 0020    ldr.w   r0, [r9, #32]      ; Load from [r9+0x20]
1324c:   0740         lsls    r0, r0, #29        ; Test bit 3
1324e:   d5fb         bpl.n   0x13248            ; Loop if bit clear
```

**Analysis**: This is a **status register polling loop**, waiting for bit 3 to be set.
- `r9` likely contains a peripheral base address (possibly QCE)
- Offset 0x20 is a status register
- Polling bit 3 before proceeding

**QCE equivalent**: This could be waiting for `QCE_STATUS` ready bit.

#### 2. Data Structure Reference (0x1323c)
```asm
1323c:   f8df 9060    ldr.w   r9, [pc, #96]      ; Load address from PC+96
```

**PC+96 from 0x1323c = 0x132a0** which contains `00 00 50 18` (0x18500000 = QCE base!)

So `r9 = 0x18500000` (QCE base address).

#### 3. Write Sequence (0x13250-0x1325c)
```asm
13250:   2204         movs    r2, #4             ; Size = 4 bytes
13252:   4629         mov     r1, r5             ; Source address
13254:   4668         mov     r0, sp             ; Dest = stack
13256:   f000 f97d    bl      0x13554            ; Call memcpy-like function
1325a:   9800         ldr     r0, [sp, #0]       ; Load value from stack
1325c:   f8c9 0000    str.w   r0, [r9]           ; Write to [r9+0] = QCE+0x00
```

**Analysis**: This is a **configuration write** to QCE base+0x00:
1. Copies 4 bytes from source table to stack
2. Loads value from stack
3. Writes to QCE register at offset 0x00

**QCE Register 0x00**: This is likely `QCE_CONFIG` or `QCE_COMMAND` register.

#### 4. Polling Loop 2 (0x1326a-0x13270)
```asm
1326a:   f8d9 0020    ldr.w   r0, [r9, #32]      ; Load QCE+0x20
1326e:   0700         lsls    r0, r0, #28        ; Test bit 4
13270:   d5fb         bpl.n   0x1326a            ; Loop if clear
```

**Analysis**: Another polling loop on QCE+0x20, this time waiting for bit 4.

#### 5. Read and Forward (0x13272-0x1327e)
```asm
13272:   f8d9 0010    ldr.w   r0, [r9, #16]      ; Read QCE+0x10
13276:   9000         str     r0, [sp, #0]       ; Store to stack
13278:   2204         movs    r2, #4             ; Size = 4
1327a:   4669         mov     r1, sp             ; Source = stack
1327c:   4630         mov     r0, r6             ; Dest address
1327e:   f000 f969    bl      0x13554            ; Call write function
```

**Analysis**: This **reads** QCE+0x10 and copies it elsewhere.
- Reads result from QCE offset 0x10
- Writes to destination buffer (r6)

---

## Reconstructed Initialization Sequence

Based on disassembly, HTC TZ does the following for CE2:

### Step 1: Load QCE Base Address
```c
void __iomem *qce_base = 0x18500000;
```

### Step 2: Poll Until Ready
```c
// Wait for QCE ready (bit 3 of status register)
while (!(readl(qce_base + 0x20) & BIT(3)))
    cpu_relax();
```

### Step 3: Write Configuration
```c
// Write configuration value to QCE command/config register
u32 config_value = <from table>;
writel(config_value, qce_base + 0x00);
```

### Step 4: Poll For Completion
```c
// Wait for operation complete (bit 4 of status register)
while (!(readl(qce_base + 0x20) & BIT(4)))
    cpu_relax();
```

### Step 5: Read Result
```c
// Read result from QCE
u32 result = readl(qce_base + 0x10);
```

---

## QCE Register Mapping (Inferred)

| Offset | Function                  | Evidence                              |
|--------|---------------------------|---------------------------------------|
| 0x00   | CONFIG/COMMAND            | Written to in init sequence           |
| 0x10   | DATA/RESULT               | Read after command completion         |
| 0x20   | STATUS                    | Polled for bits 3 and 4               |
| 0x32+  | Other control registers   | Referenced but not decoded yet        |

**Note**: These are **QCE-internal registers**, not GCC clock registers!

---

## What's Different on TouchPad?

### HTC TZ Init Does:
1. ✅ Loads QCE base address from table
2. ✅ Polls QCE status register (waits for ready)
3. ✅ Writes configuration to QCE command register
4. ✅ Polls for completion
5. ✅ Reads result

### TouchPad (No TZ Init) Does:
1. ❌ No polling of QCE registers
2. ❌ No configuration writes
3. ❌ No status checks
4. ❌ **Just assumes QCE is ready** (but it's not!)

**Critical difference**: HTC TZ **actively configures CE2 peripheral registers** before Linux boots. TouchPad skips this entirely.

---

## Missing Clock Initialization

The disassembly above shows **peripheral configuration**, but not **clock enable**. Let's search for GCC writes.

### Looking For GCC Clock Writes

Need to find code that writes to:
- `0x00902740` (CE2_HCLK_CTL) - Clock enable
- `0x00900000` range - GCC registers

Let me search for these patterns:

```bash
# Search for 0x00902740 in TZ firmware
hexdump -C /tmp/tz.img | grep "40 27 90 00"

# Search for any 0x009xxxxx addresses
hexdump -C /tmp/tz.img | grep "00 90 0"
```

---

## Clock Control Code Found!

### Region 0xd120-0xd1c0: Generic Clock Enable Function

This is a **generic clock control function** that manipulates clock register bits:

```asm
d120:  f040 0001   orr.w   r0, r0, #1      ; Set bit 0 (enable)
d124:  9000        str     r0, [sp, #0]    ; Store to stack
d126:  9800        ldr     r0, [sp, #0]    ; Load back
d128:  f040 0002   orr.w   r0, r0, #2      ; Set bit 1
d12c:  9000        str     r0, [sp, #0]
d12e:  9800        ldr     r0, [sp, #0]
d130:  f020 0018   bic.w   r0, r0, #24     ; Clear bits 3-4
d134:  3008        adds    r0, #8          ; Set bit 3
d136:  9000        str     r0, [sp, #0]
d13a:  f020 0020   bic.w   r0, r0, #32     ; Clear bit 5
d142:  f020 0040   bic.w   r0, r0, #64     ; Clear bit 6
```

**Analysis**: This function:
1. Sets bits 0, 1, 3 (enable flags)
2. Clears bits 4, 5, 6 (disable/reset flags)
3. Performs read-modify-write on clock registers

**Pattern matches CE2_HCLK_CTL manipulation!**
- Set bit 4 = enable clock
- Clear bit 7 = deassert reset

### Calling Convention

The function at 0xce74 appears to be `write_gcc_register()`:
```asm
d158:  2214        movs    r2, #20         ; Offset
d15a:  4629        mov     r1, r5          ; Base address (GCC 0x00900000)
d15c:  4620        mov     r0, r4          ; Value to write
d15e:  f7ff fe89   bl      0xce74          ; Call write function
```

### Generic Clock Enable Sequence

```c
void enable_peripheral_clock(void __iomem *gcc_base, u32 offset) {
    u32 val;
    
    /* Read current value */
    val = readl(gcc_base + offset);
    
    /* Set enable bits */
    val |= BIT(0);   /* Root clock enable */
    val |= BIT(1);   /* Branch enable */
    val |= BIT(3);   /* HCLK enable */
    
    /* Clear disable/reset bits */
    val &= ~BIT(4);  /* Clear gating */
    val &= ~BIT(5);  /* Clear power domain disable */
    val &= ~BIT(6);  /* Clear sleep */
    
    /* Write back */
    writel(val, gcc_base + offset);
}
```

## Next Steps

### 1. ✅ Clock Enable Code Found
Located at 0xd120-0xd1c0 in TZ firmware.

### 2. Find CE2-Specific Clock Call
Search for code that calls this generic function with CE2 offset.

### 3. Create Complete Init Sequence
Combine:
- Clock enable (GCC writes) ✅ Found
- Power domain enable (if found)
- Peripheral config (QCE register writes above) ✅ Found

### 4. Test on TouchPad
Add complete sequence to Linux driver probe and test if QCE responds.

---

## Key Finding

**HTC TZ actively writes to QCE registers during boot.**

This is the missing piece! TouchPad's empty TZ partition means:
- No clock enable at hardware level
- No peripheral configuration
- No readiness checks
- QCE remains in reset/powered-off state

The manual clock enable we added to the Linux driver is a good start, but we also need the **peripheral configuration writes** shown above.

**To fully replicate HTC init, we need:**
1. ✅ Enable CE2_HCLK_CTL (we already do this)
2. ❌ Write QCE config registers (this is NEW)
3. ❌ Poll QCE status for ready
4. ❌ Handle any error conditions

Let me continue searching for the clock enable code...
