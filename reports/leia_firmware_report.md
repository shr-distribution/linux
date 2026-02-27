# Adreno Leia (A220) GPU Firmware Analysis Report

## Overview

The Adreno A220 GPU uses two microcode firmware files:
- **PM4 (leia_pm4_470.fw)**: Main Command Processor Micro-Engine firmware (9220 bytes, 2305 instructions)
- **PFP (leia_pfp_470.fw)**: Pre-Fetch Parser firmware (1156 bytes, 289 instructions)

## PM4 Firmware Structure

### Header (0x0000-0x0007)
```
0x0000: 0x00000000  ; Reserved/Magic
0x0004: 0x00220014  ; Version word
         GPU ID: 0x220 (Adreno 220/Leia)
         FW Version: 0x14 (20)
```

**CRITICAL**: If version word is 0, driver logs "Legacy firmware detected, disabling protection support"

### Instruction Statistics
| Type | Count | Percentage | Description |
|------|-------|------------|-------------|
| NOP | 995 | 43.2% | Zero/padding instructions |
| IMM16 | 359 | 15.6% | 16-bit immediate values/offsets |
| DATA | 346 | 15.0% | Data words/constants |
| REG_REF | 200 | 8.7% | Register references ($r17.68, etc.) |
| PM4_T3 | 89 | 3.9% | PM4 Type 3 command templates |
| CTRL | 66 | 2.9% | Control flow instructions |
| BRANCH | 58 | 2.5% | Conditional branches |
| REG_OP | 35 | 1.5% | Register operations |
| MASK | 18 | 0.8% | Bitmask operations |
| WRITE_IMM | 18 | 0.8% | Write immediate to register |
| JUMP | 16 | 0.7% | Unconditional jumps |
| LOAD | 9 | 0.4% | Load operations |
| CJUMP | 8 | 0.3% | Conditional jumps |

### Key Code Sections

#### Initialization (0x0008-0x00FF)
- PM4 command template setup (op=0x04, 0x0a, 0x46)
- Register initialization
- Interrupt handlers

#### Command Dispatch (0x0100-0x01FF)
- PM4 packet parsing (Type 3 commands)
- Control flow for different opcodes
- Common patterns: `002f0222` (control), `0ce00000` (branch)

#### Interrupt Handlers (0x0200-0x08FF)
- Multiple interrupt service routines
- Conditional execution paths
- Register state save/restore

#### Magic Values Found
```
0x2140: 0xcafebabe  ; Sentinel/marker value
0x2218: 0xdeadbeef  ; Debug/sentinel value
```
These are commonly used as:
- Memory corruption detection
- Uninitialized memory markers
- Debug break points

#### Jump/Dispatch Table (0x22B4-0x23FF)
The firmware ends with a dispatch table containing packed 16-bit offsets:
```
0x22b4: 0x00020158  ; Entry: opcode 0x00 -> offset 0x158, opcode 0x01 -> 0x02
0x22b8: 0x00020002  ; Padding/default entries
...
0x2344: 0x00020156  ; More dispatch entries
```

### Register References
The firmware frequently references these register patterns:
- `$r17.68` (0x00204411) - 78 occurrences - likely GPR or scratch register
- `$r17.72` (0x00204811) - 35 occurrences - paired with above
- `$r3.72` (0x00204803) - state register
- `0x0029462c` - register operation (35 occurrences)

### PM4 Command Templates
Most common PM4 Type 3 opcodes referenced in microcode:
| Opcode | Name | Count |
|--------|------|-------|
| 0x04 | CP_UNKNOWN_04 | Multiple |
| 0x08 | CP_FETCH_DMA | Several |
| 0x0A | CP_UNKNOWN_0A | Several |
| 0x0C | CP_UNKNOWN_0C | Several |
| 0x10 | CP_ME_INIT | Several |
| 0x46 | CP_REG_TO_SCRATCH | Several |
| 0x48 | CP_COND_REG_EXEC | Several |

## PFP Firmware Structure

### Header
```
0x0000: 0x00000000  ; Reserved
0x0004: 0x00c60400  ; Version (different encoding)
```

### Statistics
| Type | Count | Percentage |
|------|-------|------------|
| DATA | 272 | 94.1% |
| NOP | 10 | 3.5% |
| IMM16 | 6 | 2.1% |
| ADDR | 1 | 0.3% |

The PFP uses a more compact instruction encoding with most instructions in the DATA category (0x00xxxxxx pattern).

### Dispatch Table (0x0404-0x0480)
```
0x0404: 0x000100bf  ; Opcode 0x01 -> handler at 0xbf
0x0408: 0x000200c4  ; Opcode 0x02 -> handler at 0xc4
0x040c: 0x000300c9  ; Opcode 0x03 -> handler at 0xc9
...
```

## Comparison: Linux-firmware vs WebOS

| Aspect | Linux-firmware | WebOS |
|--------|---------------|-------|
| Size | 9220 bytes | 9220 bytes |
| Version Word | 0x00220014 | 0x00000000 |
| Different Words | - | 1436 (62.3%) |
| Small Deltas (<100) | - | 235 |
| Large Deltas (≥100) | - | 1201 |

The WebOS firmware is a completely different build with:
- Missing version identification (triggers legacy mode)
- Different instruction sequences
- Different jump targets and offsets

## Debugging Applications

### Graphics Issues
When debugging Adreno A220 graphics issues, check:

1. **Firmware Loading**: Verify correct firmware is loaded
   ```bash
   dmesg | grep -i "leia\|adreno\|firmware"
   ```

2. **Version Detection**: Should NOT see "Legacy firmware detected"
   ```bash
   dmesg | grep -i legacy
   ```

3. **Command Processor Hangs**: Look for:
   - `CP_INTERRUPT` (0x35) handler issues
   - `CP_WAIT_FOR_IDLE` (0x16) timeout
   - Ring buffer issues (CP_RB_* registers)

4. **Register Dump**: Key registers for debugging
   - `RBBM_STATUS` (0x0000) - Overall GPU status
   - `CP_ME_STATUS` (0x0145) - ME status
   - `CP_STAT` (0x0156) - CP statistics
   - `SCRATCH_REG0-7` (0x0440-0x0447) - Debug scratch

### Common Failure Modes

1. **GPU Hang**: Usually CP waiting for condition that never occurs
   - Check `CP_WAIT_REG_*` conditions
   - Look for interrupt handlers

2. **Rendering Corruption**: Often related to:
   - `CP_LOAD_STATE` (0x3a) issues
   - Register write ordering
   - Cache flush timing

3. **Display Freeze**: Check:
   - MDP4 LCDC timing
   - VSYNC interrupt handling
   - Framebuffer flip commands

## Files

- `/lib/firmware/qcom/leia_pm4_470.fw.zst` - Compressed PM4 firmware
- `/lib/firmware/qcom/leia_pfp_470.fw.zst` - Compressed PFP firmware
- `/sys/kernel/debug/dri/0/` - DRM debug interface (when available)
