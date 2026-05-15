# VFE31 RDI/RAW Mode Investigation Summary

## Platform
- **SoC**: Qualcomm APQ8060 (MSM8660 variant)
- **Camera ISP**: VFE31 (Video Front End version 3.1)
- **Camera Sensor**: Aptina MT9M113 (1.26MP, supports YUV422 and RAW Bayer)
- **MIPI Interface**: 1-lane CSI-2 via integrated CSIPHY/CSID (no separate ISPIF)
- **Kernel**: Linux 6.18 mainline with custom camss driver

## Problem Statement
We're trying to implement RAW Bayer capture on HP TouchPad using VFE31's RDI (Raw Dump Interface) mode. YUV capture via PIX mode works perfectly, but RAW capture via RDI mode fails completely.

## Working Configuration (PIX Mode - YUV Capture)
```
Sensor Output:      UYVY (YUV422, 16 bits/pixel)
MIPI Data Type:     0x1E (YUV422 8-bit)
VFE AXI_OUT_MODE:   0x01 (OUTPUT_1_AND_3)
VFE MODULE_CFG:     0x01C00C0C (DEMUX + processing enabled)
Write Masters:      WM0 (Y plane), WM4 (CbCr plane)
Result:             SUCCESS - frames captured correctly
```

## Failing Configuration (RDI Mode - RAW Capture)
```
Sensor Output:      RAW8 Bayer GRBG (8 bits/pixel)
MIPI Data Type:     0x2A (RAW8)
VFE AXI_OUT_MODE:   0x60 (CAMIF_TO_AXI_VIA_OUTPUT_2)
VFE MODULE_CFG:     0x00 (all processing disabled - raw bypass)
Write Masters:      WM0 only (single plane raw)
Result:             FAIL - 0 bytes captured, continuous VIOLATION IRQs
```

## Diagnostic Tests Performed

### Test 1: Standard RDI Mode with RAW8 (dt=0x2A)
**Configuration**: Sensor outputs RAW8, MIPI data type 0x2A
**CAMIF Status**: `0x80000000` - 0 pixels, 0 lines received
**Result**: FAIL - VFE CAMIF receives no data despite sensor streaming

### Test 2: "Fake YUV" Trick (RAW pixels with YUV data type)
**Configuration**: Sensor outputs RAW8 pixels but MIPI data type set to 0x1E (YUV)
**CAMIF Status**: `0x01680500` - 1280 pixels, 360 lines received
**Result**: FAIL - Data reaches CAMIF but doesn't transfer to memory

### Test 3: PIX Mode with Fake YUV
**Configuration**: Fake YUV through PIX/DEMUX path
**Result**: Data captured but colors corrupted (DEMUX interprets RAW as YUV)

## Key Findings

### 1. VFE31 CAMIF Has MIPI Data Type Filtering
The CAMIF input stage filters packets based on MIPI data type:
- **RAW8 (dt=0x2A)**: Packets IGNORED - CAMIF reports 0 pixels, 0 lines
- **YUV (dt=0x1E)**: Packets ACCEPTED - CAMIF reports 1280 pixels, 360 lines

This filtering appears to be hardcoded in silicon - we found no register to disable it.

### 2. CAMIF_TO_AXI Raw Bypass Path is Non-Functional
Even when data reaches CAMIF (using fake YUV trick):
- AXI_STATUS remains 0x00000000
- BUS_OPERATION_STATUS remains 0x00000000
- PING_PONG_STATUS shows no buffer switching
- Continuous VIOLATION IRQs fire (status=0, spurious)
- No data appears in DMA buffers

### 3. MSM8660 CSID Lacks Data Type Configuration
Unlike newer Qualcomm SoCs (MSM8974+) that have separate CSID with CID_LUT registers for per-virtual-channel data type filtering, the MSM8660 has an integrated CSIPHY/CSID that's essentially pass-through with no data type configuration capability.

### 4. WebOS Never Used RAW Mode
Analysis of webOS kernel sources confirms:
- The `CAMIF_TO_AXI_VIA_OUTPUT_2` code path existed but was never production-tested
- WebOS only captured YUV via PIX mode
- The RAW snapshot code was likely copy-pasted from reference code but never validated

## Register Comparison

| Register | PIX Mode (working) | RDI Mode (failing) |
|----------|-------------------|-------------------|
| MODULE_CFG (0x010) | 0x01C00C0C | 0x00000000 |
| CORE_CFG (0x014) | 0x00000046 | 0x00000040 |
| AXI_OUT_MODE (0x040) | 0x00000001 | 0x00000060 |
| XBAR_CFG1 (0x044) | 0x00001A03 | 0x00000000 |
| IRQ_COMPOSITE_MASK | 0x00000011 | 0x00000100 |

## Questions for Further Investigation

1. **Is there an undocumented register** that controls CAMIF data type filtering?

2. **Is the AXI_OUT_MODE 0x60 path actually implemented** in VFE31 silicon, or is it a placeholder from a reference design?

3. **Are there alternative approaches** to capture raw data on VFE31?
   - Could we configure DEMUX as pass-through?
   - Could we abuse the test pattern generator path?
   - Is there a different AXI mode that works?

4. **Do other MSM8660/APQ8060 devices** (e.g., HTC phones from that era) have working RAW capture, or is this a known limitation?

5. **Is there Qualcomm errata** documenting VFE31 limitations on MSM8660?

## Code References
- VFE31 Driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- CSIPHY Driver: `drivers/media/platform/qcom/camss/camss-csiphy-8x60.c`
- Sensor Driver: `drivers/media/i2c/mt9m114.c`

## Conclusion

The VFE31 CAMIF_TO_AXI raw bypass path (AXI_OUT_MODE 0x60) does not appear to be functional on APQ8060. The hardware exhibits two distinct issues:

1. **Input filtering**: CAMIF ignores RAW8 MIPI packets (only accepts YUV)
2. **Output path**: Even when data reaches CAMIF, the raw bypass DMA path doesn't transfer data to memory

This may be a silicon limitation, an unvalidated feature, or require undocumented register configuration that we haven't discovered.

Any insights into VFE31 architecture, MSM8660 camera subsystem quirks, or alternative approaches would be appreciated.
