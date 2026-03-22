# HP TouchPad Camera Debugging Status

**Last Updated:** 2026-03-22
**Kernel:** Linux 6.18-tenderloin
**SoC:** Qualcomm APQ8060 (MSM8660 variant)
**Sensor:** MT9M113 front camera (MIPI CSI-2, 1 lane, UYVY 8-bit)

---

## Executive Summary for AI Analysis

### The Problem
CSIPHY receives MIPI data from sensor (confirmed by SOT+ECC interrupts) but VFE CAMIF never generates CAMIF_SOF. The data path between CSIPHY decoded output and VFE CAMIF input is broken despite all visible configuration matching webOS.

### Key Architecture Finding: VFE31 vs VFE8x

**CONCLUSIVE from GitHub research:**

| Feature | VFE8x (QSD8x50) | VFE31 (MSM8x60/APQ8060) |
|---------|-----------------|-------------------------|
| `struct vfe_cfg` with bit fields | YES | NO |
| VFE_CFG inputSource bits 16-17 | YES (functional) | NO (enum defined but NEVER used in driver) |
| VFE_CFG pixelPattern bits 0-2 | YES | YES (only bits 0-7 readable) |
| Test generator handler | YES | NO (V31_TEST_GEN_START defined but no handler) |

**Source:** [Evervolv msm_vfe_8x60_ZSL.c](https://github.com/Evervolv/android_kernel_htc_pyramid/blob/295c047ddfd55b1aa4890e9382510de5c0db7414/drivers/media/video/msm/msm_vfe_8x60_ZSL.c)
- `vfe31_operation_config()` writes raw value to VFE_CFG without bit manipulation
- Only `& 0x7` mask ever used to read pixelPattern - no bits 16-17 access
- `VFE_START_INPUT_SOURCE` enum defined but never referenced in code

**Implication:** Data routing from CSIPHY→VFE is automatic on MSM8660. There is no input source mux register.

### What Works
- ✅ Sensor streaming (CSIPHY SOT+ECC interrupts confirm MIPI packets arriving)
- ✅ Sensor OUTPUT_CONTROL = 0x7A08 (MIPI enabled, parallel disabled)
- ✅ CSIPHY registers match webOS exactly (all 9 registers verified)
- ✅ VFE clocks enabled (all 7: vfe, vfe_axi, vfe_ahb, vfe_csi0, vfe_csi1, csi_pix, csi_rdi)
- ✅ MISC_CC_REG = 0x06003440 (csi_pix/csi_rdi enabled, CSI1 selected)
- ✅ VFE CAMIF/AXI configuration matches webOS values

### What Fails
- ❌ VFE CAMIF never generates CAMIF_SOF
- ❌ VFE reg update timeout (needs SOF to latch)
- ❌ Captured frames = 0 bytes

### Theories to Investigate
1. **Clock HALT status:** Are vfe_csi1_clk or csi_pix/csi_rdi clocks actually halted despite enable bits?
   - Check DBG_BUS_VEC_B_REG (0x04000114) for halt status bits
2. **Async FIFO stall:** Clock domain crossing FIFO between CSIPHY and VFE may be stalling
3. **Undocumented enable bit:** There may be a VFE input enable we haven't found
4. **Format mismatch:** CSIPHY/VFE pixel format configuration mismatch causing CAMIF to ignore data

### Questions for AI
1. On MSM8660, since VFE31 has no inputSource mux, how does data routing from CSIPHY to VFE actually work?
2. Is there an async FIFO or CDC (clock domain crossing) block between CSIPHY and VFE that needs explicit enable?
3. What registers in MMCC (0x04000000-0x040001FF) control the VFE input path beyond MISC_CC_REG?
4. Does VFE CAMIF on VFE31 require specific MODULE_CFG bits to be enabled before it will recognize input data?

---

## Current Issue

**Problem:** VFE (Video Front End) never receives pixel data from the MIPI CSI camera chain, resulting in "VFE sof timeout" and "VFE reg update timeout" errors.

**Symptoms:**
- Capture attempts result in 0-byte output files
- VFE timeout errors in dmesg
- CSIPHY receives MIPI data (confirmed by IRQ activity)
- But VFE CAMIF never receives Start-of-Frame or pixel data

---

## MSM8660 Camera Architecture (Critical Understanding)

### Key Difference from Later Qualcomm SoCs

MSM8660 has a **unified CSIPHY+CSID architecture** where data routing from CSIPHY to VFE is **automatic**:

| Feature | MSM8660 | MSM8974+ |
|---------|---------|----------|
| CSIPHY/CSID | **Unified block** | Separate blocks |
| ISPIF | **Not present** | Required for routing |
| Data path | CSIPHY → VFE (direct) | CSIPHY → CSID → ISPIF → VFE |
| CID configuration | **Not needed** | Required per-stream |

**WebOS confirms this:** The legacy kernel does NOT configure any CSID CID registers. It simply:
1. Configures CSIPHY (PHY_CONTROL, PROTOCOL_CONTROL, CALIBRATION_CONTROL, lane config)
2. Configures VFE CAMIF
3. Data flows automatically

### Hardware Data Path

```
MT9M113 Sensor (I2C: 0x3c)
     │
     ▼ (MIPI CSI-2, 1 lane, UYVY 8-bit)
┌─────────────────────────────────────────────────┐
│ CSIPHY1 @ 0x04900000                            │
│ - PHY_CONTROL, PROTOCOL_CONTROL                 │
│ - CALIBRATION_CONTROL, CAMERA_CNTL              │
│ - IRQ: SOT, ECC, Frame Start/End                │
│                                                 │
│ NOTE: MSM8660 has UNIFIED CSIPHY+CSID           │
│       No separate CSID block exists!            │
│       No ISPIF exists on MSM8660!               │
└─────────────────────────────────────────────────┘
     │
     │  ← Data routing is AUTOMATIC (no ISPIF)
     │  ← Requires clock bridge: vfe_csi1_clk OR csi_pix/csi_rdi
     │
     ▼
┌─────────────────────────────────────────────────┐
│ VFE31 @ 0x04500000                              │
│ CAMIF block (0x1E0-0x204):                      │
│   - EFS_CFG (0x1E4): Sync mode selection        │
│   - FRAME_CFG (0x1E8): width/height             │
│   - WINDOW_CFG: first/last pixel/line           │
│   - CAMIF_CMD (0x1E0): start/stop               │
│                                                 │
│ AXI block (0x040):                              │
│   - AXI_OUT_MODE: 0x60 (raw) or 0x200 (pix)     │
│   - Write masters for DMA to memory             │
│                                                 │
│ IRQ status:                                     │
│   - STATUS0 BIT(0): CAMIF_SOF                   │
│   - STATUS0 BIT(2): REG_UPDATE_ACK              │
│   - STATUS1 BIT(22): RESET_ACK                  │
└─────────────────────────────────────────────────┘
     │
     ▼
AXI → DDR Memory → /dev/video3 (V4L2)
```

---

## Clock Bridge Configuration

Two paths exist for CSIPHY→VFE data routing on MSM8660:

### Path 1: Modern Mux (via csi_pix/csi_rdi clocks) - CURRENTLY USING

Controlled by **MISC_CC_REG** (0x04000058):
```
BIT(26): csi_pix_clk enable
BIT(25): csi_pix_clk CSI1 select (0=CSI0, 1=CSI1)
BIT(13): csi_rdi_clk enable
BIT(12): csi_rdi_clk CSI1 select (0=CSI0, 1=CSI1)
```

For MT9M113 on CSI1: **Current value 0x06003440** means:
- BIT(26)=1, BIT(25)=1: csi_pix enabled, CSI1 selected ✓
- BIT(13)=1, BIT(12)=1: csi_rdi enabled, CSI1 selected ✓
- Additional bits: AXI clock gates

### Path 2: Legacy (via vfe_csi0/vfe_csi1 clocks only)

Controlled by **VFE_CC_REG** (0x04000104):
```
BIT(12): vfe_csi0_clk enable
BIT(10): vfe_csi1_clk enable
```

**Note:** Legacy mode (MISC_CC_REG=0x0) was tested and is BROKEN. Modern mux is required.

---

## Current Configuration Status

### ✅ VERIFIED WORKING

| Component | Register/Value | Status |
|-----------|----------------|--------|
| MT9M113 OUTPUT_CONTROL | 0x3400 = 0x7A08 | MIPI output enabled (commit 3942033534e0 fixed MCU clearing BIT(3)) |
| MT9M113 RESET_REGISTER | 0x301A = 0x120C | Streaming mode enabled |
| MT9M113 SEQ_CMD | 0xA103 = 0x0001 | MCU running in preview mode |
| MISC_CC_REG | 0x04000058 = 0x06003440 | csi_pix/csi_rdi enabled, CSI1 selected |
| CSIPHY1 D1_CONTROL | 0x20 = 0x300 | PHY enabled (bits 8,9 set) |
| CSIPHY1 CAMERA_CNTL | 0x24 = 0xe404 | 1 lane configured correctly |
| CSIPHY1 PROTOCOL_CONTROL | 0x04 = 0x00260000 | ECC+DECODE_ID+LONG_PKT_CAPTURE |
| CSIPHY1 IRQ status | BIT(4)+BIT(5) | SOT+ECC interrupts firing = sensor sending data |
| VFE EFS_CFG | 0x1E4 = 0x00 | APS mode (not EFS sync) |
| VFE FRAME_CFG | 0x1E8 = correct | width/height configured |
| VFE AXI_OUT_MODE | 0x040 = 0x60 | CAMIF_TO_AXI (raw bypass) |
| VFE clocks | all 7 present | vfe, vfe_axi, vfe_ahb, vfe_csi0, vfe_csi1, csi_pix, csi_rdi |

### ❌ FAILING

| Symptom | Error | Notes |
|---------|-------|-------|
| VFE SOF timeout | "VFE sof timeout" | CAMIF never receives frame start |
| VFE reg update timeout | "VFE reg update timeout" | REG_UPDATE needs SOF to latch |
| Pipeline stop failed | -515 (ENOIOCTLCMD) | Harmless - pixel array subdev has no s_stream |
| Captured file empty | 0 bytes | No frames written to DMA buffer |

### Key Observation

**CSIPHY receives data (SOT+ECC interrupts) but VFE CAMIF never generates CAMIF_SOF.**

This means the data path between CSIPHY decoded output and VFE CAMIF input is not working, despite all clocks being enabled.

---

## Recent Commits (This Session)

| Commit | Description |
|--------|-------------|
| cbad6db12844 | Fix swapped csi_pix/csi_rdi clock index comments |
| 3942033534e0 | Fix MT9M113 MCU clearing OUTPUT_CONTROL BIT(3) when SEQ_CMD=RUN |

---

## Things We've Ruled Out

1. **Sensor not streaming:** CSIPHY receives SOT/ECC = sensor is sending MIPI packets
2. **OUTPUT_CONTROL disabled:** Now correctly shows 0x7A08 after SEQ_CMD (MCU fix)
3. **Wrong CSI selected:** MISC_CC_REG shows CSI1 selected (BIT 25, 12 set)
4. **Clocks not enabled:** All 7 VFE clocks enabled including csi_pix and csi_rdi
5. **VFE configuration wrong:** CAMIF registers and AXI_OUT_MODE match webOS values
6. **CSIPHY PHY disabled:** D1_CONTROL = 0x300 (PHY enabled)
7. **Legacy mode works:** Tested - MISC_CC_REG=0x0 is BROKEN, modern mux required
8. **AXI_OUT_MODE selection:** Tested both 0x60 (raw) and 0x200 (pix) - both fail

---

## CSIPHY Configuration Verified MATCHING webOS

| Register | Offset | Our Value | webOS Value | Status |
|----------|--------|-----------|-------------|--------|
| MIPI_PHY_CONTROL | 0x00 | 0x04 | 0x04 | ✓ MATCH |
| MIPI_PROTOCOL_CONTROL | 0x04 | 0x00260000 | 0x00260000 | ✓ MATCH |
| MIPI_CALIBRATION_CONTROL | 0x18 | 0x00E00080 | 0x00E00080 | ✓ MATCH |
| MIPI_PHY_D0_CONTROL | 0x34 | 0x00000000 | 0x00000000 | ✓ MATCH |
| MIPI_PHY_D0_CONTROL2 | 0x38 | 0x140F0018 | 0x140F0018 | ✓ MATCH |
| MIPI_PHY_D1_CONTROL | 0x20 | 0x00000300 | 0x00000300 | ✓ MATCH |
| MIPI_PHY_CL_CONTROL | 0x48 | 0x0F000004 | 0x0F000004 | ✓ MATCH |
| MIPI_CAMERA_CNTL | 0x24 | 0x0000E404 | 0x0000E404 | ✓ MATCH |
| MIPI_INTERRUPT_MASK | 0x0C | 0xFFF7F3FF | 0xFFF7F3FF | ✓ MATCH |

**PROTOCOL_CONTROL breakdown (0x00260000):**
- LONG_PACKET_HEADER_CAPTURE (bit 21): 1
- DATA_FORMAT (bits 19-20): 0 (raw/YUV422-8bit)
- DECODE_ID (bit 18): 1
- ECC_EN (bit 17): 1

---

## Open Questions

### Q1: Is MIPI_PROTOCOL_CONTROL DATA_FORMAT=0 correct?

Current value: 0x00260000 with DATA_FORMAT bits [20:19] = 0x0

MT9M113 sends UYVY which is YUV422-8bit (MIPI data type 0x1E). DATA_FORMAT=0 should be correct for this, as it means "decode based on CSI packet header data type."

### Q2: Does VFE CAMIF require something to see the data?

VFE CAMIF can operate in:
- **APS mode (EFS_CFG=0):** Asynchronous Pixel Stream - internal sync
- **EFS mode (EFS_CFG!=0):** External Frame Sync - waits for external VSYNC

We use APS mode. CAMIF should generate CAMIF_SOF internally based on pixel flow.

### Q3: Is there a missing VFE input select register?

We've configured AXI_OUT_MODE for output routing, but is there an **input** mux/select that chooses between CSI0 and CSI1 as the pixel source?

### Q4: Are we missing a register write in MMCC?

WebOS may write additional MMCC registers we haven't identified:
- Additional routing in MISC_CC_REG or nearby
- VFE input enable bit somewhere
- Clock domain crossing enable

### Q5: Timing/ordering issue?

Could be a race:
- VFE CAMIF started before CSIPHY fully locked
- Need CSIPHY receiving frames before VFE starts
- Some specific init order required

---

## webOS Reference Sequences

### msm_camio_csi_config() (msm_io_8x60.c):

```c
1. writel(0x4, csibase + MIPI_PHY_CONTROL);
2. writel(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK, csibase + MIPI_PROTOCOL_CONTROL);
3. writel(0x00260000, csibase + MIPI_PROTOCOL_CONTROL);
4. writel(0x00E00080, csibase + MIPI_CALIBRATION_CONTROL);
5. writel(0x140F0018, csibase + MIPI_PHY_D0_CONTROL2);  // all 4 lanes
6. writel(0x0F000004, csibase + MIPI_PHY_CL_CONTROL);
7. writel(0, csibase + MIPI_PHY_D0_CONTROL);
8. writel(0x00000300, csibase + MIPI_PHY_D1_CONTROL);  // PHY enable
9. writel(0, csibase + MIPI_PHY_D2_CONTROL);
10. writel(0, csibase + MIPI_PHY_D3_CONTROL);
11. writel(0xE404, csibase + MIPI_CAMERA_CNTL);  // 1 lane
12. writel(0xFFF7F3FF, csibase + MIPI_INTERRUPT_MASK);
```

**Our driver matches this sequence exactly.**

### VFE start sequence (msm_vfe31.c):

```c
1. vfe31_reset()
2. Configure AXI output config (188 bytes at offset 0x38)
3. vfe31_start_common():
   - IRQ_MASK_0 = 0x00EFE021
   - IRQ_MASK_1 = 0x00400000
   - REG_UPDATE_CMD = 1
   - CAMIF_COMMAND = 1
4. Wait for IRQ
```

**Our driver does this too.**

---

## Hypothesis: Missing Data Path Enable

There may be a register we haven't found that enables the actual data path from CSIPHY decoded output to VFE CAMIF input. Possibilities:

1. **In MMCC block (0x04000000):** Additional clock gating or routing register beyond MISC_CC_REG
2. **In VFE (0x04500000):** Input select register for CSI0 vs CSI1 that we haven't identified
3. **In TCSR:** MSM8660-specific "chicken bits" or hidden enables

WebOS sets up camera via qcameralib HAL, which may write registers through kernel ioctls we haven't fully traced.

---

## Diagnostic Commands

```bash
# On device:
./test-camera.sh pix     # Full pipeline test (uses video3)
./test-camera.sh legacy  # Legacy routing mode test (KNOWN BROKEN)

# Check dmesg for errors:
dmesg | grep -E "VFE|CSIPHY|mt9m|camss" | tail -100

# Check clock register values:
devmem2 0x04000058 w  # MISC_CC_REG (expect 0x06003xxx with CSI1 selected)
devmem2 0x04000104 w  # VFE_CC_REG

# Check CSIPHY interrupt count:
cat /proc/interrupts | grep csiphy
```

---

## dmesg Excerpt (Typical Capture Attempt)

```
MT9M113: OUTPUT_CONTROL verification: 0x7A08 (MIPI enabled)
CSIPHY1: lanes_enable complete
CSIPHY1: READBACK: PHY_CONTROL=0x04 PROTOCOL=0x00260000
CSIPHY1: CAMERA_CNTL(0x24)=0x0000e404
CSIPHY1: D1_CTRL(0x20)=0x00000300 (PHY enabled)
VFE DEBUG: MMCC MISC_CC_REG = 0x06003440
VFE DEBUG:   csi_pix_en (bit 26): 1, csi_pix_sel (bit 25): CSI1
VFE DEBUG:   csi_rdi_en (bit 13): 1, csi_rdi_sel (bit 12): CSI1
CSIPHY1: IRQ #1 status=0x00000030 [SOT ECC] sof_count=0
CSIPHY1: IRQ #2 status=0x00000030 [SOT ECC] sof_count=1
...
VFE: AXI_OUT_MODE set to 0x00000060
VFE: Writing CAMIF_CMD=1 to start capture
...
[500ms later]
VFE: sof timeout
VFE: reg update timeout
Video pipeline stop failed: -515
```

---

## Summary

**Working:** Sensor streaming (OUTPUT_CONTROL=0x7A08), MIPI PHY receiving data (SOT/ECC IRQs), all clocks enabled (MISC_CC_REG=0x06003440 with CSI1), CSIPHY configuration matches webOS.

**Not working:** Pixel data not reaching VFE CAMIF despite all visible configuration being correct. VFE never generates CAMIF_SOF interrupt.

**Root cause hypothesis:** Unknown register or configuration that enables the CSIPHY→VFE data path on MSM8660. The data is decoded (ECC IRQs prove this) but not flowing to CAMIF.

---

## Gemini Analysis (2026-03-22)

### Key Insight: VFE as MIPI Receiver

Because MSM8660 lacks a standalone CSID, **VFE itself must act as the MIPI receiver**. Mainline `camss` assumes CSID handles Virtual Channel (VC) and Data Type (DT) filtering. If mainline treats 8x60 CSID as pass-through without programming VFE's internal CSI-2 decode, VFE won't know to grab VC=0/DT=0x1E data.

**However:** Investigation shows VFE31 does NOT have VC/DT configuration registers. The CSIPHY handles all MIPI decode and is supposed to pass decoded pixels automatically.

### Clock HALT Status

Even with clocks enabled, need to verify HALT status bits:
- **DBG_BUS_VEC_B_REG:** Contains halt bits for CSI and VFE clocks
- If downstream block holds clock in reset, data won't flow

### Async FIFO Stall Theory

An async FIFO sits between MIPI byte clock and VFE core clock:
- If VFE input formatter is misconfigured (expects 10-bit raw but receives 8-bit YUV)
- The byte-packing logic stalls
- FIFO never reaches "ready" threshold
- CAMIF_SOF never fires

**Action:** Compare webOS vs mainline values for:
- EPOCH_CFG
- FRAME_CFG
- Any registers with PACK/UNPACK in name

### Clock Domain Crossing (CDC) Issue

Gemini suggested: "If webOS left MISC_CC_REG at 0x00000000, it implies hardware defaults to hardwired CDC path between CSIPHY1 and VFE. By setting 0x06003440, mainline might be severing the default bridge."

**Status:** We tested MISC_CC_REG=0 (legacy mode) and it also failed. So this isn't the issue.

### VFE31 inputSource Bits - CONCLUSIVE FINDING

**GitHub Research Results:**

Examined multiple kernel sources to compare VFE8x vs VFE31 architecture:

| Feature | VFE8x (QSD8x50) | VFE31 (MSM8x60) |
|---------|-----------------|-----------------|
| `struct vfe_cfg` with bit fields | YES | NO |
| inputSource bits 16-17 | YES (used) | NO (enum defined but NEVER used) |
| pixelPattern bits 0-2 | YES | YES (only bits 0-7 read) |

**Evidence from [Evervolv/android_kernel_htc_pyramid](https://github.com/Evervolv/android_kernel_htc_pyramid/blob/295c047ddfd55b1aa4890e9382510de5c0db7414/drivers/media/video/msm/msm_vfe_8x60_ZSL.c):**

1. `vfe31_operation_config()` writes raw value to VFE_CFG_OFF without bit manipulation
2. Only `& 0x7` mask is ever used to read pixelPattern - no bits 16-17 access
3. `VFE_START_INPUT_SOURCE` enum is defined in header but NEVER used in driver code
4. `V31_TEST_GEN_START` command exists in table but has NO handler implementation

**VFE8x struct vfe_cfg (from webOS msm_vfe8x_proc.h:547-552):**
```c
struct vfe_cfg {
    uint32_t pixelPattern:3;   // bits 0-2
    uint32_t reserved:13;      // bits 3-15
    uint32_t inputSource:2;    // bits 16-17  ← EXISTS IN VFE8x ONLY
    uint32_t reserved:14;      // bits 18-31
};
```

**CONCLUSION:** VFE31 removed inputSource hardware bits. Data routing is automatic.

---

## Proposed Test: VFE Test Generator

**FINDING:** GitHub research shows V31_TEST_GEN_START command is defined in the command table but has **NO handler implementation** in webOS VFE31 driver. This suggests:
- Test generator may not be supported on VFE31
- Or it requires different enabling mechanism than VFE8x

We can still try our HW_TESTGEN register approach:
1. Enable VFE test pattern generator via module parameter:
   ```bash
   echo 1 > /sys/module/qcom_camss/parameters/vfe31_use_testgen
   ./test-camera.sh pix
   ```

2. This will:
   - Configure HW_TESTGEN registers at 0x36C-0x39C
   - Write VFE_TEST_GEN_GO (0x01) to HW_TESTGEN_CMD
   - **NOTE:** inputSource bits don't exist on VFE31, so this relies on direct HW_TESTGEN activation

3. Expected results:
   - If test generator works → VFE can process internally generated data
   - If test generator fails → VFE31 may not support test generator at all

**Implementation details:**
- HW_TESTGEN_CMD (0x36C): VFE_TEST_GEN_GO=0x01, VFE_TEST_GEN_STOP=0x02
- HW_TESTGEN_CFG (0x370): Frame count, pixel size, sync edges
- HW_TESTGEN_IMAGE_CFG (0x374): width | (height << 16)
- HW_TESTGEN_COLOR_BARS (0x398): Color bar pattern config

**Alternative approach:** Focus on CSIPHY→VFE path debugging since test generator may not be viable.

---

## Key webOS VFE31 Implementation Details

From analyzing webOS `msm_vfe31.c` and `msm_vfe31.h`:

### vfe31_operation_config() - VFE_CFG Setup
```c
// Line 887-900 of msm_vfe31.c
static int vfe31_operation_config(uint32_t *cmd)
{
    uint32_t *p = cmd;
    vfe31_ctrl->operation_mode = *p;         // Word 0: operation mode
    vfe31_ctrl->stats_comp = *(++p);         // Word 1: stats comp
    msm_io_w(*(++p), ... + VFE_CFG_OFF);     // Word 2 → VFE_CFG (0x14)
    msm_io_w(*(++p), ... + VFE_MODULE_CFG);  // Word 3 → VFE_MODULE_CFG (0x10)
    // ... more registers
}
```

### vfe31_start_common() - Start Sequence
```c
// Line 987-1009 of msm_vfe31.c
msm_io_w(0x00EFE021, ... + VFE_IRQ_MASK_0);
msm_io_w(VFE_IMASK_WHILE_STOPPING_1, ... + VFE_IRQ_MASK_1);
msm_io_w_mb(1, ... + VFE_REG_UPDATE_CMD);
msm_io_w(1, ... + VFE_CAMIF_COMMAND);  // NOTE: Uses 1, not 0x5
wmb();
```

### Key Observations
- VFE_CFG is passed from userspace via V31_OPERATION_CFG command (28 bytes)
- inputSource and pixelPattern are set by qcameralib, not kernel driver
- CAMIF_COMMAND uses value 1 at runtime (despite header defining 0x5)

---

## Questions for AI Analysis

1. **VFE_CFG inputSource value during normal camera operation:**
   - What value does qcameralib set for inputSource (bits 16-17)?
   - Is inputSource=0 (CAMIF) the correct value for CSI input?
   - We're currently NOT setting inputSource at all - default is unknown

2. **V31_OPERATION_CFG command content:**
   - webOS passes 28 bytes (7 words) via this command
   - We need to understand what values qcameralib passes
   - Specifically the VFE_CFG word which contains inputSource

3. **VFE MODULE_CFG (0x10) register:**
   - This register enables/disables ISP modules
   - What bits need to be set for basic CAMIF capture?
   - Are we missing an enable bit?

4. **Data format consideration:**
   - MIPI CSI-2 YUV422-8 (data type 0x1E) - pixelPattern should be VFE_YUV_CbYCrY (6)
   - Is there a format mismatch causing CAMIF to not recognize valid data?

5. **CSIPHY→VFE path on MSM8660:**
   - Since we confirmed inputSource bits exist, how does VFE know to select CSI0 vs CSI1?
   - Is there a CSI select register separate from inputSource?
