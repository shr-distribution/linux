# Camera Debug Summary for Gemini Analysis
## HP TouchPad (APQ8060/MSM8660) - March 22, 2026

## Problem Statement
VFE31 CAMIF never generates SOF (Start of Frame) interrupt despite CSIPHY receiving valid MIPI data from the MT9M113 sensor. Capture times out after ~10 seconds.

## What's Working
1. **Sensor**: MT9M113 initializes correctly, outputs MIPI CSI-2 data
2. **CSIPHY**: Receives MIPI data - confirmed by SOT (Start of Transmission) and ECC interrupts
3. **Pipeline**: Full media pipeline starts correctly (sensor → CSIPHY → CSID → VFE)
4. **Clocks**: vfe_csi1_clk not halted (DBG_BUS_VEC_B bit 8 = 0)

## What's Broken
- VFE CAMIF never receives pixel data
- IRQ_STATUS_0 stays 0x00000000 (no SOF, no frame interrupts)
- CAMIF_STATUS transitions from 0x80000000 (halted) to 0x00000000 (started) but no data arrives

## Register Comparison: webOS vs Mainline

### Registers We Fixed (Now Match webOS)
| Register | Address | webOS | Mainline (OLD) | Mainline (NEW) |
|----------|---------|-------|----------------|----------------|
| AXI_OUT_MODE | VFE+0x040 | 0x00000001 | 0x60 or 0x200 | **0x00000001** ✓ |
| EFS_CFG | VFE+0x1E4 | 0x00000040 | 0x00000000 | **0x00000040** ✓ |

### Registers That Still Differ
| Register | MMCC Address | webOS | Mainline | Notes |
|----------|--------------|-------|----------|-------|
| **MISC_CC_REG** | 0x04000058 | **0x00000400** | 0x06003000 | **CRITICAL DIFFERENCE** |
| CSI_CC_REG | 0x04000040 | 0x00000085 | 0x00000000 | Clock config |
| VFE_CC_REG | 0x04000104 | 0x80ff14a5 | varies | Clock enables |
| VFE_NS_REG | 0x04000014 | 0x04a48c71 | varies | Clock dividers |
| FRAME_CFG | VFE+0x1E8 | 0x00000000 | 0x03c80508 | webOS doesn't use this |

## MISC_CC_REG Analysis (Most Likely Root Cause)

**webOS value: 0x00000400**
- Only bit 10 is set
- Meaning of bit 10 is unknown (not documented in our driver)

**Mainline value: 0x06003000**
- bit 12: csi_rdi_sel = CSI1
- bit 13: csi_rdi_clk enable
- bit 25: csi_pix_sel = CSI1
- bit 26: csi_pix_clk enable

**Hypothesis**: The Linux clock framework configures csi_pix and csi_rdi mux/enables, but webOS doesn't use these paths. webOS may use a different, simpler data routing that only requires bit 10.

## CSIPHY IRQ Evidence (Data IS Arriving)
```
CSIPHY1: IRQ status=0x00400830 [SOT ECC] sof_count=0
CSIPHY1: IRQ status=0x00000030 [SOT ECC] sof_count=0
```
- SOT (Start of Transmission) interrupts confirm MIPI data packets arriving
- ECC (Error Correction) interrupts confirm valid packet headers
- sof_count=0 confirms VFE is NOT seeing frames

## VFE State During Capture Attempt
```
VFE: EFS_CFG=0x40 at 0x1E4 (matches webOS)
VFE: AXI_OUT_MODE=0x00000001, EFS_CFG=0x00000040
VFE: CAMIF_STATUS before REG_UPDATE: 0x80000000
VFE: After START: CAMIF_STATUS=0x00000000 IRQ_STATUS0=0x00000000
[10 seconds later]
VFE sof timeout
VFE reg update timeout
```

## Hardware Data Path (MSM8660)
```
MT9M113 Sensor (MIPI CSI-2, 1 lane, UYVY 8-bit)
     │
     ▼
CSIPHY1 (0x04900000) ─── SOT+ECC IRQs confirm data here ✓
     │
     ▼
??? ─── DATA DISAPPEARS HERE ───
     │
     ▼
VFE31 CAMIF (0x04500000 + 0x1E0) ─── Never sees SOF ✗
     │
     ▼
AXI Write Master → DDR Memory
```

## Questions for Gemini

1. **MISC_CC_REG bit 10**: What does bit 10 at MMCC offset 0x058 control on MSM8660? webOS only sets this bit, not the csi_pix/csi_rdi mux bits.

2. **Data routing architecture**: Does MSM8660 have a direct CSIPHY→VFE path that bypasses the csi_pix/csi_rdi clock muxes? If so, how is it configured?

3. **FRAME_CFG register**: webOS has FRAME_CFG=0x00000000, but our driver writes actual frame dimensions (0x03c80508 = 968 lines | 1288 pixels). Does VFE31 ignore FRAME_CFG and use WINDOW_WIDTH/WINDOW_HEIGHT instead?

4. **Clock dependencies**: Could the different VFE_CC_REG/VFE_NS_REG values between webOS and mainline cause the CSIPHY→VFE data path to fail even though clocks appear enabled?

5. **CSI_CC_REG**: webOS has 0x00000085, mainline has 0x00000000. What do these bits control and could this affect data flow?

## Relevant webOS Register Dump (Full)
```
=== MMCC Clock Registers ===
CLK_HALT_STATEC (0x104):  0x80ff14a5
DBG_BUS_VEC_B (0x114):    0x00000000  (vfe_csi1_clk NOT halted)
CSI_CC_REG (0x040):       0x00000085
CSI1_CC_REG (0x044):      0x00000000
MISC_CC_REG (0x058):      0x00000400  <-- KEY DIFFERENCE
VFE_CC_REG (0x104):       0x80ff14a5
VFE_NS_REG (0x014):       0x04a48c71

=== VFE31 Registers ===
VFE_HW_VERSION (0x000):   0x00030217
VFE_MODULE_CFG (0x010):   0x01c00c0c
VFE_CFG_OFF (0x014):      0x00000046  (pixelPattern=6=UYVY)
AXI_OUT_MODE (0x040):     0x00000001
EFS_CFG (0x1E4):          0x00000040
FRAME_CFG (0x1E8):        0x00000000  (not used by webOS)
WINDOW_WIDTH (0x1EC):     0x01e00500
WINDOW_HEIGHT (0x1F0):    0x000004ff
CAMIF_STATUS (0x204):     0x00000000

=== CSIPHY1 Registers ===
PHY_CONTROL (0x00):       0x00000004
PROTOCOL_CONTROL (0x04):  0x00260000
IRQ_STATUS (0x08):        0x00400830  (SOT+ECC active)
D1_CONTROL (0x20):        0x00000300
```

## Files Modified
- `drivers/media/platform/qcom/camss/camss-vfe-3-1.c` - AXI_OUT_MODE default, EFS_CFG
- `drivers/media/platform/qcom/camss/camss-vfe.c` - EFS_CFG writes

## Commits
- `aa5c9dde4b53` - media: camss: VFE31: Match webOS AXI_OUT_MODE and EFS_CFG values
- `cb8398111f34` - docs: Add webOS VFE register dump for reference
