# MMCC Clock Comparison: webOS 2.6 vs Mainline Linux 6.18

**Date:** 2026-01-17 (Updated)
**Purpose:** Comprehensive comparison of Multimedia Clock Controller (MMCC) configuration between webOS 2.6 kernel (clock-8x60.c) and mainline Linux 6.18 (mmcc-msm8960.c) for HP TouchPad (APQ8060/MSM8660)

## Executive Summary

### Critical Findings

1. **NS Register Address Mismatch** (FIXED in MSM8660-specific structures)
   - webOS PIXEL_NS_REG: `0x00DC`
   - Original mainline mdp_pixel_src: `0x00e0`
   - Fixed mdp_pixel_src_msm8660: `0x00dc` ✓

2. **M Value Shift Error** (FIXED)
   - webOS MD16 format: M at bits 31:16, requires `m_val_shift = 16`
   - Original mainline: `m_val_shift = 8` (WRONG - causes M/N overlap)
   - Fixed mdp_pixel_src_msm8660: `m_val_shift = 16` ✓

3. **MN Counter Width Difference**
   - webOS PIXEL_MDP: 16-bit M/N counters (MD16 format)
   - Original mainline: 8-bit M/N counters
   - Fixed mdp_pixel_src_msm8660: `width = 16` ✓

4. **Pre-divider Configuration Difference**
   - webOS: `pre_div_shift = 14`, `pre_div_width = 2`
   - MSM8960 mainline: `pre_div_shift = 12`, `pre_div_width = 4`
   - Fixed mdp_pixel_src_msm8660: matches webOS ✓

5. **LCDC Halt Bit Difference**
   - webOS: LCDC halt bit = 21
   - MSM8960 mainline: LCDC halt bit = 24
   - Fixed mdp_lcdc_clk_msm8660: `halt_bit = 21` ✓

---

## 1. Register Address Comparison

### 1.1 MDP Pixel Clock Registers

| Register | webOS (0x00XX) | MSM8960 Mainline | MSM8660 Fixed | Match webOS? |
|----------|----------------|------------------|---------------|--------------|
| NS_REG   | 0x00DC         | 0x00e0           | 0x00dc        | ✓ Fixed      |
| MD_REG   | 0x00D8         | 0x00d8           | 0x00d8        | ✓            |
| CC_REG   | 0x00D4         | 0x00d4           | 0x00d4        | ✓            |

### 1.2 MDP Core Registers

| Register | webOS (0x00XX) | Mainline | Match? |
|----------|----------------|----------|--------|
| MDP_MD0_REG | 0x00C4      | 0x00c4   | ✓      |
| MDP_MD1_REG | 0x00C8      | 0x00c8   | ✓      |
| MDP_NS_REG  | 0x00D0      | 0x00d0   | ✓      |

### 1.3 GFX2D Registers

| Register | webOS (0x00XX) | Mainline | Match? |
|----------|----------------|----------|--------|
| GFX2D0_MD0_REG | 0x0064   | 0x0064   | ✓      |
| GFX2D0_MD1_REG | 0x0068   | 0x0068   | ✓      |
| GFX2D0_NS_REG  | 0x0070   | 0x0070   | ✓      |

### 1.4 GFX3D Registers

| Register | webOS (0x00XX) | Mainline | Match? |
|----------|----------------|----------|--------|
| GFX3D_MD0_REG | 0x0084    | 0x0084   | ✓      |
| GFX3D_MD1_REG | 0x0088    | 0x0088   | ✓      |
| GFX3D_NS_REG  | 0x008C    | 0x008c   | ✓      |

### 1.5 TV Clock Registers

| Register | webOS (0x00XX) | Mainline | Match? |
|----------|----------------|----------|--------|
| TV_CC_REG  | 0x00EC       | 0x00ec   | ✓      |
| TV_CC2_REG | 0x0124       | 0x0124   | ✓      |
| TV_NS_REG  | 0x00F4       | 0x00f4   | ✓      |

### 1.6 PLL Registers

| PLL | webOS Name | webOS Offset | Mainline Name | Mainline Offset | Match? |
|-----|------------|--------------|---------------|-----------------|--------|
| PLL0 | MM_PLL0_MODE | 0x0300    | (not used)    | -               | -      |
| PLL1 | MM_PLL1_MODE | 0x031C    | pll2.mode_reg | 0x31c           | ✓ (different naming) |
| PLL2 | MM_PLL2_MODE | 0x0338    | pll15.mode_reg| 0x338           | ✓ (different naming) |

**Note:** Mainline calls the PLL at 0x31c "pll2" while webOS calls it "MM_PLL1". This is a naming convention difference, not a bug.

---

## 2. MN Counter Configuration Comparison

### 2.1 PIXEL/MDP Clock (MD16 Format)

| Parameter | webOS MD16 | MSM8960 Mainline | MSM8660 Fixed | Notes |
|-----------|-----------|------------------|---------------|-------|
| M position in MD | bits 31:16 | bits 15:8 (m_val_shift=8) | bits 31:16 (m_val_shift=16) | **FIXED** |
| N position in MD | bits 15:0 (~N) | bits 7:0 | bits 15:0 (~N) | **FIXED** |
| Width | 16-bit | 8-bit | 16-bit | **FIXED** |
| mnctr_en_bit | 5 | 5 | 5 | ✓ |
| mnctr_mode_shift | 6 | 6 | 6 | ✓ |
| mnctr_reset_bit | 7 | 7 | 7 | ✓ |
| n_val_shift (in NS) | 16 | 16 | 16 | ✓ |

### 2.2 Pre-divider Configuration

| Parameter | webOS PIXEL_MDP | MSM8960 Mainline | MSM8660 Fixed | Notes |
|-----------|-----------------|------------------|---------------|-------|
| pre_div_shift | 14 | 12 | 14 | **FIXED** |
| pre_div_width | 2 | 4 | 2 | **FIXED** |

### 2.3 TV/HDMI Clock (MD8 Format)

| Parameter | webOS MD8 | Mainline | Match? |
|-----------|----------|----------|--------|
| M position in MD | bits 15:8 | bits 15:8 (m_val_shift=8) | ✓ |
| N position in MD | bits 7:0 (~N) | bits 7:0 | ✓ |
| Width | 8-bit | 8-bit | ✓ |
| mnctr_en_bit | 5 | 5 | ✓ |

---

## 3. Halt Bit Comparison

### 3.1 Display-Related Clocks

| Clock | webOS Halt Bit | MSM8960 Mainline | MSM8660 Fixed | Notes |
|-------|----------------|------------------|---------------|-------|
| MDP_PIXEL_CLK | 23 | 23 | 23 | ✓ |
| MDP_LCDC_CLK | **21** | 24 | **21** | **FIXED** |
| MDP_CLK | 10 | 10 | 10 | ✓ |

### 3.2 Halt Register

| Clock Domain | webOS | Mainline | Match? |
|--------------|-------|----------|--------|
| MDP halts | 0x01d0 | 0x01d0 | ✓ |
| GFX halts | 0x01c8 | 0x01c8 | ✓ |
| AXI halts | 0x01d8 | 0x01d8 | ✓ |
| AHB halts | 0x01dc | 0x01dc | ✓ |

---

## 4. Frequency Table Comparison

### 4.1 webOS PIXEL_MDP Frequencies

```c
// webOS: arch/arm/mach-msm/clock-8x60.c
static struct clk_freq_tbl clk_tbl_pixel_mdp[] = {
    F_PIXEL_MDP(        0, MM_GND,   1,   0,    0, NONE),
    F_PIXEL_MDP( 25600000, MM_GPERF, 3,   1,    5, LOW),   // 384/3/5 = 25.6M
    F_PIXEL_MDP( 42667000, MM_GPERF, 1,   1,    9, LOW),   // 384/9 = 42.67M
    F_PIXEL_MDP( 43192000, MM_GPERF, 1,  64,  569, LOW),
    F_PIXEL_MDP( 48000000, MM_GPERF, 4,   1,    2, LOW),   // 384/4/2 = 48M
    F_PIXEL_MDP( 53990000, MM_GPERF, 2, 169,  601, LOW),
    F_PIXEL_MDP( 64000000, MM_GPERF, 2,   1,    3, LOW),   // 384/2/3 = 64M
    F_PIXEL_MDP( 69300000, MM_GPERF, 1, 231, 1280, LOW),
    F_PIXEL_MDP( 76800000, MM_GPERF, 1,   1,    5, LOW),   // 384/5 = 76.8M
    F_PIXEL_MDP( 85333000, MM_GPERF, 1,   2,    9, LOW),   // 384*2/9 = 85.33M
    F_PIXEL_MDP( 96000000, MM_GPERF, 4,   0,    0, LOW),   // 384/4 = 96M (pre-div only)
    F_PIXEL_MDP(100030000, MM_GPERF, 2, 211,  405, LOW),
    F_PIXEL_MDP(106500000, MM_GPERF, 1,  71,  256, NOMINAL),
    F_PIXEL_MDP(109714000, MM_GPERF, 1,   2,    7, NOMINAL),
    F_END,
};
```

### 4.2 MSM8960 Mainline Frequencies (Original)

```c
// Mainline: drivers/clk/qcom/mmcc-msm8960.c (for DSI panels)
static const struct freq_tbl clk_tbl_mdp_pixel[] = {
    {  25200000, P_PLL8, 1, 33, 502 },  // 384*33/502 ≈ 25.2M
    {  27000000, P_PXO,  1,  0,   0 },  // 27M from PXO
    {  40000000, P_PLL8, 1,  5,  48 },  // 384*5/48 = 40M
    {  46000000, P_PLL8, 1, 23, 192 },  // 384*23/192 ≈ 46M
    {  50000000, P_PLL8, 1,  1,   8 },  // 384/8 = 48M (mislabeled!)
    {  65000000, P_PLL8, 1, 13,  76 },  // 384*13/76 ≈ 65.7M
    {  74250000, P_PLL8, 1, 99, 512 },  // 384*99/512 ≈ 74.25M
    {  83950000, P_PLL8, 1,  1,   5 },  // 384/5 = 76.8M (mislabeled!)
    { }
};
```

### 4.3 MSM8660 Fixed Frequencies

```c
// Fixed for MSM8660/APQ8060 LCDC panel
static const struct freq_tbl clk_tbl_mdp_pixel_msm8660[] = {
    {  76800000, P_PLL8, 1, 1, 5 },  // 384/5 = 76.8M
    {  96000000, P_PLL8, 1, 1, 4 },  // 384/4 = 96M (TouchPad needs this!)
    { }
};
```

**Note:** The webOS 96MHz entry uses `(4, 0, 0)` meaning pre_div=4 with no MN divider, while our fix uses `(1, 1, 4)` meaning pre_div=1 with M=1, N=4. Both produce 96MHz from PLL8 (384MHz).

---

## 5. MD Register Format Analysis

### 5.1 webOS MD16 Macro

```c
#define MD16(m, n) \
    (BVAL(31, 16, m) | BVAL(15, 0, ~(n)))
```

- **M value**: bits 31:16 (upper 16 bits)
- **~N value**: bits 15:0 (lower 16 bits, inverted)

### 5.2 webOS MD8 Macro (for TV clocks)

```c
#define MD8(m_lsb, m, n_lsb, n) \
    (BVAL((m_lsb+7), m_lsb, m) | BVAL((n_lsb+7), n_lsb, ~(n)))
```

- **M value**: 8-bit field at m_lsb
- **~N value**: 8-bit field at n_lsb (inverted)

### 5.3 Mainline clk-rcg.c Calculation

```c
// From drivers/clk/qcom/clk-rcg.c
static u32 mn_to_md(struct mn *mn, u32 m, u32 n, u32 md)
{
    u32 mask, mask_w;
    mask_w = BIT(mn->width) - 1;
    mask = (mask_w << mn->m_val_shift) | mask_w;
    md &= ~mask;
    if (n) {
        m <<= mn->m_val_shift;  // Shift M to correct position
        md |= m;
        md |= ~n & mask_w;      // Store ~N in lower bits
    }
    return md;
}
```

For MD16 format (width=16):
- `mask_w = 0xFFFF`
- With `m_val_shift = 16`: M goes to bits 31:16, ~N to bits 15:0 ✓
- With `m_val_shift = 8` (bug): M goes to bits 23:8, overlapping with N!

---

## 6. NS Register Format Analysis

### 6.1 webOS NS_MM Macro

```c
#define NS_MM(n_msb, n_lsb, n, m, d_msb, d_lsb, d, s_msb, s_lsb, s) \
    (BVAL(n_msb, n_lsb, ~(n-m)) | BVAL(d_msb, d_lsb, (d-1)) \
    | BVAL(s_msb, s_lsb, SRC_SEL_##s))

// For PIXEL_MDP: NS_MM(31, 16, n, m, 15, 14, d, 2, 0, s)
```

- **~(N-M)**: bits 31:16
- **pre_div-1**: bits 15:14 (2-bit field)
- **source select**: bits 2:0

### 6.2 Mainline NS Register Layout

For MSM8660 fixed structure:
- **n_val_shift = 16**: ~(N-M) at bits 31:16 ✓
- **pre_div_shift = 14, width = 2**: pre_div at bits 15:14 ✓
- **src_sel_shift = 0**: source at bits 2:0 ✓

---

## 7. Clock Enable Configuration

### 7.1 Pixel Clock Enable Bits

| Clock | Enable Register | Enable Bit | webOS | Mainline |
|-------|-----------------|------------|-------|----------|
| MDP_PIXEL_SRC | 0x00d4 | BIT(2) | CC bit 2 | ✓ |
| MDP_PIXEL_CLK | 0x00d4 | BIT(0) | - | ✓ |
| MDP_LCDC_CLK | 0x00d4 | BIT(8) | - | ✓ |

---

## 8. Current MSM8660 Clock Array

The minimal clock array for MSM8660/APQ8060:

```c
static struct clk_regmap *mmcc_msm8660_clks[] = {
    [MDP_AHB_CLK] = &mdp_ahb_clk.clkr,
    [MDP_AXI_CLK] = &mdp_axi_clk.clkr,
    [MDP_SRC] = &mdp_src.clkr,
    [MDP_CLK] = &mdp_clk.clkr,
    [HDMI_TV_CLK] = &hdmi_tv_clk.clkr,
    [GFX2D0_SRC] = &gfx2d0_src.clkr,
    [GFX2D0_CLK] = &gfx2d0_clk.clkr,
    [GMEM_AXI_CLK] = &gmem_axi_clk.clkr,
    [GFX3D_AHB_CLK] = &gfx3d_ahb_clk.clkr,
    [GFX3D_SRC] = &gfx3d_src.clkr,
    [GFX3D_CLK] = &gfx3d_clk.clkr,
    [PLL2] = &pll2.clkr,
    [MDP_PIXEL_SRC] = &mdp_pixel_src_msm8660.clkr,  // MSM8660-specific
    [MDP_PIXEL_CLK] = &mdp_pixel_clk_msm8660.clkr,  // MSM8660-specific
    [MDP_LCDC_CLK] = &mdp_lcdc_clk_msm8660.clkr,    // MSM8660-specific
};
```

---

## 9. Potential Issues Still Under Investigation

### 9.1 PLL2 Enable at Probe

```c
// In mmcc_msm8960_probe():
if (desc == &mmcc_msm8660_desc)
    regmap_update_bits(regmap, 0x31c, BIT(7), BIT(7));
```

This writes BIT(7) to register 0x31c during probe. Standard PLL mode register bits are:
- BIT(0): OUTCTRL (output enable)
- BIT(1): BYPASSNL
- BIT(2): RESET_N

BIT(7) is not a standard PLL enable bit. This may need investigation if boot issues persist.

### 9.2 Clocks Using MSM8960 Structures

The following clocks in the MSM8660 array still use MSM8960 structures and may need MSM8660-specific versions if they have different register layouts:

| Clock | Uses Structure | Verified Against webOS? |
|-------|---------------|------------------------|
| MDP_AHB_CLK | mdp_ahb_clk | Register only, no MN |
| MDP_AXI_CLK | mdp_axi_clk | Register only, no MN |
| MDP_SRC | mdp_src | Registers match ✓ |
| MDP_CLK | mdp_clk | Branch clock |
| GFX2D0_SRC | gfx2d0_src | Registers match ✓ |
| GFX3D_SRC | gfx3d_src | Registers match ✓ |

---

## 10. Summary of Fixes Applied

| Issue | Original Value | Fixed Value | Status |
|-------|---------------|-------------|--------|
| NS register offset | 0x00e0 | 0x00dc | ✓ Fixed |
| m_val_shift | 8 | 16 | ✓ Fixed |
| MN width | 8 | 16 | ✓ Fixed |
| pre_div_shift | 12 | 14 | ✓ Fixed |
| pre_div_width | 4 | 2 | ✓ Fixed |
| LCDC halt_bit | 24 | 21 | ✓ Fixed |
| Pixel halt_bit | Already 23 | 23 | ✓ OK |
| 96MHz frequency | Missing | Added | ✓ Fixed |

---

## 11. Reference Sources

- **webOS 2.6 Kernel**: https://github.com/panda-z/android_kernel_hp_mantaray/blob/master/arch/arm/mach-msm/clock-8x60.c
- **SHR Linux (working)**: https://github.com/Tofee/shr-linux/commits/tenderloin/6.13/mainline-for-upstream
- **Mainline Linux 6.18**: drivers/clk/qcom/mmcc-msm8960.c

---

## 12. Test Status

| Test | Result | Date |
|------|--------|------|
| USB with 96MHz (m_val_shift=8) | FAIL - USB not up | 2026-01-17 |
| USB with 96MHz (m_val_shift=16) | PENDING | - |
| Display at 96MHz | PENDING | - |
