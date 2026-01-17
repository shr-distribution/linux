# MDP Pixel Clock Comparison: webOS vs Mainline

**Date:** 2026-01-17
**Purpose:** Document differences in MDP pixel clock configuration between webOS kernel and mainline Linux 6.18

## Executive Summary

**Critical Finding:** The mainline kernel has a **register address mismatch** for the MDP pixel clock NS register:
- webOS: `0x00DC`
- Mainline: `0x00e0`

This 4-byte difference could explain why adding 96MHz to the clock table breaks boot.

---

## 1. Register Address Comparison

| Register | webOS (clock-8x60.c) | Mainline (mmcc-msm8960.c) | Match? |
|----------|---------------------|---------------------------|--------|
| NS_REG   | 0x00DC              | 0x00e0                    | **NO** |
| MD_REG   | 0x00D8              | 0x00d8                    | YES    |
| CC_REG   | 0x00D4              | 0x00d4 (enable_reg)       | YES    |

### Analysis
The NS (namespace/control) register offset differs by 4 bytes. This register controls:
- Clock source selection
- Pre-divider configuration
- M/N counter enable

Writing to the wrong register offset when setting 96MHz could corrupt adjacent registers or cause undefined behavior.

---

## 2. Frequency Table Comparison

### webOS Kernel (clk_tbl_pixel_mdp)
```c
F_PIXEL_MDP(        0, MM_GND,   1,   0,    0, NONE),
F_PIXEL_MDP( 25600000, MM_GPERF, 3,   1,    5, LOW),
F_PIXEL_MDP( 42667000, MM_GPERF, 1,   1,    9, LOW),
F_PIXEL_MDP( 43192000, MM_GPERF, 1,  64,  569, LOW),
F_PIXEL_MDP( 48000000, MM_GPERF, 4,   1,    2, LOW),
F_PIXEL_MDP( 53990000, MM_GPERF, 2, 169,  601, LOW),
F_PIXEL_MDP( 64000000, MM_GPERF, 2,   1,    3, LOW),
F_PIXEL_MDP( 69300000, MM_GPERF, 1, 231, 1280, LOW),
F_PIXEL_MDP( 76800000, MM_GPERF, 1,   1,    5, LOW),   // <-- Correct!
F_PIXEL_MDP( 85333000, MM_GPERF, 1,   2,    9, LOW),
F_PIXEL_MDP( 96000000, MM_GPERF, 4,   0,    0, LOW),   // <-- 96 MHz for TouchPad
F_PIXEL_MDP(100030000, MM_GPERF, 2, 211,  405, LOW),
F_PIXEL_MDP(106500000, MM_GPERF, 1,  71,  256, NOMINAL),
F_PIXEL_MDP(109714000, MM_GPERF, 1,   2,    7, NOMINAL),
```

### Mainline Kernel (clk_tbl_mdp_pixel)
```c
{  25200000, P_PLL8, 1, 33, 502 },
{  27000000, P_PXO,  1,  0,   0 },
{  40000000, P_PLL8, 1,  5,  48 },
{  46000000, P_PLL8, 1, 23, 192 },
{  50000000, P_PLL8, 1,  1,   8 },
{  65000000, P_PLL8, 1, 13,  76 },
{  74250000, P_PLL8, 1, 99, 512 },
{  83950000, P_PLL8, 1,  1,   5 },   // <-- WRONG! Should be 76800000
// NO 96 MHz entry!
```

### SHR Kernel (Working on older kernel)
```c
{  76800000, P_PLL8, 1, 1, 5 },  // 384 * 1/5 = 76.8 MHz
{  96000000, P_PLL8, 1, 1, 4 },  // 384 * 1/4 = 96 MHz
```

---

## 3. Frequency Calculation Analysis

### PLL8 = 384 MHz (confirmed on device via debugfs)

| Target (webOS) | pre_div | M | N | Calculated | Matches? |
|----------------|---------|---|---|------------|----------|
| 25,600,000     | 3       | 1 | 5 | 384/3*1/5 = 25.6M | YES |
| 42,667,000     | 1       | 1 | 9 | 384*1/9 = 42.67M | YES |
| 48,000,000     | 4       | 1 | 2 | 384/4*1/2 = 48M | YES |
| 64,000,000     | 2       | 1 | 3 | 384/2*1/3 = 64M | YES |
| **76,800,000** | 1       | 1 | 5 | 384*1/5 = 76.8M | YES |
| 85,333,000     | 1       | 2 | 9 | 384*2/9 = 85.33M | YES |
| **96,000,000** | 4       | 0 | 0 | 384/4 = 96M | YES |

| Target (Mainline) | pre_div | M | N | Calculated | Matches? |
|-------------------|---------|---|---|------------|----------|
| 25,200,000        | 1       | 33| 502| 384*33/502 = 25.2M | YES |
| 50,000,000        | 1       | 1 | 8 | 384*1/8 = 48M | ~NO |
| 65,000,000        | 1       | 13| 76| 384*13/76 = 65.7M | ~YES |
| **83,950,000**    | 1       | 1 | 5 | 384*1/5 = **76.8M** | **NO!** |

---

## 4. Clock Source Comparison

| Kernel | Source Name | Actual PLL | Frequency |
|--------|-------------|------------|-----------|
| webOS  | MM_GPERF    | PLL8       | 384 MHz   |
| Mainline | P_PLL8    | PLL8       | 384 MHz   |

Both use PLL8 at 384 MHz - this matches.

---

## 5. M/N Divider Configuration

### webOS
```c
#define F_PIXEL_MDP(f, s, d, m, n, v) \
    { \
        .freq_hz = f, \
        .src = SRC_##s, \
        .md_val = MD16(m, n), \
        .ns_val = NS_MM(31, 16, n, m, 15, 14, d, 2, 0, s), \
        .cc_val = CC(6, n), \
        .mnd_en_mask = BIT(5) * !!(n), \
        .sys_vdd = v \
    }
```

### Mainline
```c
static struct clk_rcg mdp_pixel_src = {
    .ns_reg = 0x00e0,      // <-- DIFFERENT from webOS!
    .md_reg = 0x00d8,
    .mn = {
        .mnctr_en_bit = 5,
        .mnctr_reset_bit = 7,
        .mnctr_mode_shift = 6,
        .n_val_shift = 16,
        .m_val_shift = 8,
        .width = 8,
    },
    .p = {
        .pre_div_shift = 12,
        .pre_div_width = 4,
    },
    .s = {
        .src_sel_shift = 0,
        .parent_map = mmcc_pxo_pll8_pll2_map,
    },
    ...
};
```

---

## 6. Observations and Hypotheses

### Why 96MHz Breaks Boot

1. **Register Address Mismatch**: The NS register is at 0x00DC in webOS but 0x00e0 in mainline. When the clock driver tries to set 96MHz, it writes to the wrong register, potentially corrupting adjacent clock configurations.

2. **Early Boot Clock Access**: The MMCC driver initializes early. If something queries or sets the pixel clock during init, using wrong register addresses could crash the system before USB gadget initializes.

3. **Frequency Table Mismatch**: The mainline table's frequency values don't match their divisor settings. This suggests the table may have been designed for a different SoC variant (MSM8960) with different PLL8 configuration.

### Why 83.95MHz "Works" (but doesn't give display)

The 83.95MHz entry with m=1, n=5 actually produces 76.8MHz. But since:
- The panel requests 96MHz
- clk_round_rate returns 83.95MHz (the table lookup, not actual rate)
- Mode validation fails (96M != 83.95M)
- Display init is skipped entirely

The display code never actually tries to SET the clock, so the wrong register address is never accessed during display init.

---

## 7. Recommended Fixes

### Option A: Fix NS Register Address
Change mainline `mdp_pixel_src.ns_reg` from `0x00e0` to `0x00DC` to match webOS.

### Option B: Use webOS-Style Frequency Table
Replace the entire frequency table with webOS values that are known to work.

### Option C: Verify Register Layout
Dump MMCC registers on running webOS vs LuneOS to confirm actual hardware register layout.

---

## 8. Register Dump Commands

To verify on device:
```bash
# On webOS:
novacom run file://bin/cat -- /sys/kernel/debug/clk/mmcc/registers

# On LuneOS:
cat /sys/kernel/debug/clk/clk_summary | grep pixel
devmem2 0x04000000 + offset
```

---

## 9. Files to Modify

| File | Change Needed |
|------|---------------|
| `drivers/clk/qcom/mmcc-msm8960.c` | Fix NS register address (0x00e0 → 0x00DC) |
| `drivers/clk/qcom/mmcc-msm8960.c` | Fix frequency table values |
| `drivers/clk/qcom/mmcc-msm8960.c` | Add 96MHz entry after fixing registers |

---

## 10. Appendix: Full webOS Clock Table

```c
// webOS: arch/arm/mach-msm/clock-8x60.c
static struct clk_freq_tbl clk_tbl_pixel_mdp[] = {
    F_PIXEL_MDP(        0, MM_GND,   1,   0,    0, NONE),
    F_PIXEL_MDP( 25600000, MM_GPERF, 3,   1,    5, LOW),
    F_PIXEL_MDP( 42667000, MM_GPERF, 1,   1,    9, LOW),
    F_PIXEL_MDP( 43192000, MM_GPERF, 1,  64,  569, LOW),
    F_PIXEL_MDP( 48000000, MM_GPERF, 4,   1,    2, LOW),
    F_PIXEL_MDP( 53990000, MM_GPERF, 2, 169,  601, LOW),
    F_PIXEL_MDP( 64000000, MM_GPERF, 2,   1,    3, LOW),
    F_PIXEL_MDP( 69300000, MM_GPERF, 1, 231, 1280, LOW),
    F_PIXEL_MDP( 76800000, MM_GPERF, 1,   1,    5, LOW),
    F_PIXEL_MDP( 85333000, MM_GPERF, 1,   2,    9, LOW),
    F_PIXEL_MDP( 96000000, MM_GPERF, 4,   0,    0, LOW),
    F_PIXEL_MDP(100030000, MM_GPERF, 2, 211,  405, LOW),
    F_PIXEL_MDP(106500000, MM_GPERF, 1,  71,  256, NOMINAL),
    F_PIXEL_MDP(109714000, MM_GPERF, 1,   2,    7, NOMINAL),
    F_END,
};
```
