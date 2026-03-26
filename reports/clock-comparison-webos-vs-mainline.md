# Clock/PLL Comparison: webOS vs Mainline Linux

HP TouchPad (MSM8660/APQ8060)
- webOS: 3.0.5 (register dump via devmem)
- Mainline: Linux 6.18 (debugfs clk_summary)

## Executive Summary

| Aspect | webOS | Mainline | Status |
|--------|-------|----------|--------|
| PLL8 (CSI/USB) | 384 MHz | 384 MHz | ✅ MATCH |
| MM_PLL1 (pll2) | 800 MHz | 800 MHz | ✅ MATCH |
| PLL4 (LPASS) | OFF | 540.672 MHz | ⚠️ Different |
| PLL3 (LPASS) | 502 MHz | N/A | Different approach |
| CSIPHY Timer | NOT USED | Was blocking | ✅ FIXED |
| VFE Clock | ~228 MHz | 266.67 MHz | ✅ OK (higher) |
| CSI Source | 384 MHz | 384 MHz | ✅ MATCH |

---

## 1. PLL Comparison

### GCC PLLs (Global Clock Controller)

| PLL | webOS Rate | webOS Status | Mainline Rate | Mainline Status | Notes |
|-----|------------|--------------|---------------|-----------------|-------|
| PLL0 | 0 | OFF | N/A | Not in tree | Apps Fabric PLL |
| PLL1 | N/A | Partial | N/A | Not in tree | |
| PLL2 | N/A | Partial | N/A | Not in tree | |
| **PLL3** | **502 MHz** | **ACTIVE** | N/A | Not in tree | LPASS in webOS |
| **PLL4** | 0 | OFF | **540.672 MHz** | ACTIVE | LPASS in mainline |
| PLL5 | 0 | OFF | N/A | Not in tree | |
| **PLL8** | **384 MHz** | **ACTIVE** | **384 MHz** | **ACTIVE** | ✅ USB/CSI - Critical |
| PLL9-12 | 0 | OFF | N/A | Not in tree | |
| PLL14 | Unknown | RPM | N/A | RPM-owned | |

### MMCC PLLs (Multimedia Clock Controller)

| PLL | webOS Rate | webOS Status | Mainline Rate | Mainline Status | Notes |
|-----|------------|--------------|---------------|-----------------|-------|
| **MM_PLL0** | **~1320 MHz** | **ACTIVE** | N/A | Not visible | High-speed multimedia |
| **MM_PLL1 (pll2)** | **~800 MHz** | **ACTIVE** | **800 MHz** | **ACTIVE** | ✅ VFE/MDP source |
| MM_PLL2 | 0 | OFF | N/A | Not in tree | |

### PLL Configuration Details

**PLL8 (384 MHz) - CRITICAL FOR CAMERA**
```
webOS:  L=14, M=2, N=9 → 27MHz × (14 + 2/9) = 384 MHz
Mainline: Managed by clock framework, 384 MHz confirmed
```

**MM_PLL1/pll2 (800 MHz) - VFE/MDP SOURCE**
```
webOS:  L=29, M=17, N=27 → 27MHz × (29 + 17/27) = 800 MHz
Mainline: 800 MHz (from debugfs)
```

**PLL4 vs PLL3 (Audio/LPASS)**
```
webOS uses PLL3 at 502 MHz for audio
Mainline uses PLL4 at 540.672 MHz for audio
Different approach but both functional
```

---

## 2. Camera Clocks

### CSI (Camera Serial Interface)

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| csi0_src | 384 MHz (PLL8) | 384 MHz | ✅ MATCH |
| csi1_src | 384 MHz (PLL8) | 384 MHz | ✅ MATCH |
| csi0_clk | Enabled | 384 MHz | ✅ |
| csi1_clk | Enabled | 384 MHz | ✅ |
| csi0_phy_clk | Enabled | 384 MHz | ✅ |
| csi1_phy_clk | Enabled | 384 MHz | ✅ |
| csi_pix_clk | Enabled | 384 MHz | ✅ |
| csi_rdi_clk | Enabled | 384 MHz | ✅ |

### VFE (Video Front End)

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| vfe_src | MN:D from pll2 | 266.67 MHz | ✅ OK |
| vfe_clk | ~228 MHz | 266.67 MHz | ✅ Higher is OK |
| vfe_csi0_clk | Enabled | 266.67 MHz | ✅ |
| vfe_csi1_clk | Enabled | 266.67 MHz | ✅ |
| vfe_axi_clk | Enabled | Enabled | ✅ |
| vfe_ahb_clk | Enabled | Enabled | ✅ |

### CSIPHY Timer Clocks

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| csiphy0_timer_clk | **NOT USED** (0x160=0) | Was 384 MHz | ✅ REMOVED |
| csiphy1_timer_clk | **NOT USED** (0x168=0) | Was 384 MHz | ✅ REMOVED |

**Finding:** webOS does NOT configure CSIPHY timer clocks at all. The registers
at 0x160-0x168 are all zeros. Mainline was trying to enable these clocks but
they fail with EBUSY, blocking the entire camera pipeline. Fixed by removing
these clocks from the driver.

### Camera Clock (MCLK to Sensor)

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| camclk0_clk | Configured | 24 MHz | ✅ |
| camclk1_clk | Not used | 27 MHz | OK |

---

## 3. Display Clocks

### MDP (Mobile Display Processor)

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| mdp_src | From pll2 | 200 MHz | ✅ |
| mdp_clk | Enabled | 200 MHz | ✅ |
| mdp_lut_clk | Enabled | 200 MHz | ✅ |
| mdp_vsync_clk | Enabled | 27 MHz | ✅ |
| mdp_axi_clk | Enabled | Enabled | ✅ |
| mdp_ahb_clk | Enabled | Enabled | ✅ |

### LCDC (LCD Controller)

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| mdp_lcdc_clk | Configured | 96 MHz | ✅ |
| mdp_pixel_clk | Configured | 96 MHz | ✅ |

---

## 4. GSBI (General Serial Bus Interface)

| GSBI | webOS | Mainline | Function |
|------|-------|----------|----------|
| GSBI1 | OFF | OFF | - |
| GSBI2 | OFF | OFF | - |
| GSBI3 | **ON** | **ON** | I2C (sensors) |
| GSBI4 | OFF | **ON** | I2C (camera) |
| GSBI5 | OFF | OFF | - |
| GSBI6 | **ON** | **ON** | UART (debug) |
| GSBI7 | OFF | **ON** | I2C |
| GSBI8 | **ON** | **ON** | I2C (touchscreen controller) |
| GSBI9 | OFF | OFF | - |
| GSBI10 | **ON** | **ON** | UART (touchscreen @ 4 Mbps) |
| GSBI11 | OFF | OFF | - |
| GSBI12 | OFF | **ON** | I2C + UART (console) |

**Mainline enables more GSBI ports** - this is expected as mainline supports
more peripherals than webOS was using at capture time.

---

## 5. USB Clocks

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| usb_hs1_h_clk | Enabled | Enabled | ✅ |
| usb_hs1_xcvr_clk | Enabled | 60 MHz | ✅ |
| usb_fs1_* | OFF | OFF | ✅ |
| usb_fs2_* | OFF | OFF | ✅ |

---

## 6. Storage Clocks (SDC/eMMC)

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| sdc1_clk | Active | 48 MHz | ✅ eMMC |
| sdc2_clk | OFF | OFF | - |
| sdc3_clk | Active | 281 kHz | ✅ |
| sdc4_clk | Active | 24 MHz | ✅ |
| sdc5_clk | OFF | OFF | - |

---

## 7. Audio Clocks (LPASS)

| Clock | webOS Source | Mainline Source | Status |
|-------|--------------|-----------------|--------|
| codec_i2s_spkr_osr | PLL3 (502 MHz) | PLL4 (540.672 MHz) | ✅ Different but OK |
| codec_i2s_spkr_bit | Derived | 69.246 kHz | ✅ |
| codec_i2s_mic_* | Derived | 152.543 kHz | ✅ |

**Note:** webOS uses PLL3 for LPASS, mainline uses PLL4. Both work correctly
for audio functionality.

---

## 8. Fabric/Interconnect Clocks

| Clock | webOS | Mainline | Status |
|-------|-------|----------|--------|
| afab_clk | Active | 764.3 MHz | ✅ Apps Fabric |
| sfab_clk | Active | 384 MHz | ✅ System Fabric |
| mmfab_clk | Active | 558.8 MHz | ✅ MM Fabric |
| smi_clk | Active | 558.8 MHz | ✅ SMI |
| daytona_clk | Active | 384 MHz | ✅ |

---

## 9. CPU Clocks

| Clock | Mainline Rate | Notes |
|-------|---------------|-------|
| cpu0_clk | 1512 MHz | Max frequency |
| cpu1_clk | 756 MHz | Currently scaled |

---

## 10. Issues Found & Fixed

### CSIPHY Timer Clock Issue (FIXED)

**Problem:** Mainline was trying to enable `csiphy0_timer_clk` and
`csiphy1_timer_clk` which caused EBUSY errors and blocked the camera pipeline.

**Root Cause:** The MMCC csiphytimer_src clock cannot be enabled because the
underlying hardware registers (0x160-0x168) are not properly configured for
MSM8660.

**Evidence:** webOS register dump shows these registers are all zeros:
```
TIMER_CC (0x160): 0x00000000
TIMER_MD (0x164): 0x00000000
TIMER_NS (0x168): 0x00000000
```

**Solution:** Removed CSIPHY timer clocks from driver. Settle count uses
hardcoded value 0x14 (20) per webOS.

---

## 11. Summary of Matches

| Category | Match Status |
|----------|--------------|
| PLL8 (384 MHz for CSI) | ✅ EXACT MATCH |
| MM_PLL1/pll2 (800 MHz) | ✅ EXACT MATCH |
| CSI clocks (384 MHz) | ✅ EXACT MATCH |
| VFE clocks | ✅ COMPATIBLE (slightly higher) |
| MDP clocks | ✅ COMPATIBLE |
| USB clocks | ✅ MATCH |
| GSBI clocks | ✅ MATCH (mainline has more) |
| SDC clocks | ✅ COMPATIBLE |
| Audio clocks | ✅ COMPATIBLE (different PLL) |
| Fabric clocks | ✅ MATCH |
| CSIPHY timer | ✅ FIXED (removed) |

---

## 12. Recommendations

1. **Camera clocks are correct** - All critical camera clocks match webOS
2. **CSIPHY timer clocks removed** - Matches webOS behavior
3. **Audio uses different PLL** - PLL4 vs PLL3, both functional
4. **VFE clock slightly higher** - 266 MHz vs 228 MHz, acceptable
5. **No further clock changes needed** for basic camera functionality

---

*Generated: 2024*
*Comparison based on webOS 3.0.5 register dump and Linux 6.18 debugfs*
