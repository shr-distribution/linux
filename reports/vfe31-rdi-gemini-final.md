# VFE31 AXI=0x60 Raw Bypass - Final Investigation Summary for Gemini

## The Mystery

We cannot get the CAMIF_TO_AXI raw bypass path (AXI_OUT_MODE=0x60) to work
on APQ8060 VFE 3.1 (HW version 0x00030217), despite matching EVERY register
value from Samsung, HTC, Opal, and webOS vendor kernels and HALs.

Samsung and HTC kernels have `CAMIF_TO_AXI_VIA_OUTPUT_2` with `*p = 0x60`
in their `vfe31_config_axi()` function. HTC's decompiled HAL
(`liboemcamera.so`) has a complete `axi_raw_snapshot_config` function that
configures the AXI blob with 0x60 and sends it via ioctl. The code is
reachable via `camera_ops_start(case 0xe)`. This is NOT dead code.

Yet on our HP TouchPad (APQ8060, same silicon as MSM8660), the CAMIF shows
"active" but counts ZERO lines/pixels, and ZERO VFE IRQs fire during
streaming with AXI=0x60.

## What DOES Work

**RAW-through-PIX**: Setting the sensor to RAW10 output while keeping the
VFE in PIX mode (AXI=0x01) captures real, recognizable images at 3.4fps.
The CAMIF counts pixels, VFE IRQs fire, PP_STATUS toggles, 5 frames of
1280x1024 data captured. The Y plane contains valid Bayer RAW data.

This proves:
- Sensor outputs valid RAW10 over MIPI ✓
- CSIPHY receives it (2000+ data IRQs) ✓
- CAMIF can process RAW10 packets through PIX path ✓
- WM0 DMA to DDR works ✓

## Every Register We've Matched

| Register | Offset | Our Value | Samsung Value | Match |
|----------|--------|-----------|---------------|-------|
| CORE_CFG | 0x014 | 0x01 | 0x01 (pattern, no INPUT_MUX) | ✓ |
| MODULE_CFG | 0x010 | 0x00400C04 | 0x00000404+ (DEMUX+path gates) | ✓ |
| IRQ_MASK_0 | 0x01C | 0x00E00021 | 0x00E00021 (Samsung exact) | ✓ |
| IRQ_MASK_1 | 0x020 | 0x00400000 | 0x00400000 (RESET_ACK only) | ✓ |
| IRQ_COMP_MASK | 0x034 | 0x00000100 | 0x00000100 (WM0 group 1) | ✓ |
| BUS_CMD | 0x038 | 0x7FFF | 0x3FFF (reload all WMs) | ✓ |
| BUS_CFG | 0x03C | 0x02AAA775 | 0x02AAA775 (10-bit raw) | ✓ |
| AXI_OUT_MODE | 0x040 | 0x60 | 0x60 (CAMIF_TO_AXI) | ✓ |
| XBAR_CFG1 | 0x044 | not written | not written for raw | ✓ |
| WM0_WR_CFG | 0x04C | 0x01 | 0x01 (enabled) | ✓ |
| WM0_PING | 0x050 | valid DMA addr | valid DMA addr | ✓ |
| WM0_PONG | 0x054 | valid DMA addr | valid DMA addr | ✓ |
| WM0_ADDR_CFG | 0x058 | 0x0000038F | 0x00000397 (UB depth ~911) | ~✓ |
| WM0_UB_CFG | 0x05C | depth+height | depth+height | ✓ |
| WM0_IMG_SIZE | 0x060 | stride+height | stride+height | ✓ |
| CAMIF_CMD | 0x1E0 | 0x01 | 0x01 (start, CODE value) | ✓ |
| CAMIF_CFG | 0x1E4 | 0x10 | 0x10 (bit 4, raw mode) | ✓ |
| FRAME_CFG | 0x1E8 | 0x00 | 0x00 (zeroed) | ✓ |
| WINDOW_WIDTH | 0x1EC | height\|width*2 | height\|width*2 | ✓ |
| CLAMP_MAX | 0x524 | 0xFFFFFF | 0xFFFFFF | ✓ |
| REALIGN_BUF | 0x52C | 0x00 | 0x00 (zeroed) | ✓ |
| VFE_AXI_CFG | 0x600 | 0x80000000 | 0x80000000 (out-of-order) | ✓ |
| BUS_PM_CFG | 0x18C | 1 | 1 | ✓ |
| BUS_PM_CMD | 0x188 | 1 | 1 | ✓ |
| REG_UPDATE | 0x260 | 1 | 1 | ✓ |

## CSIPHY Configuration

| Setting | Value | Match |
|---------|-------|-------|
| DATA_FORMAT | 1 (10-bit for RAW10) | ✓ (dynamic detection) |
| PROTOCOL_CONTROL | 0x002E0000 | ✓ (ECC+DECODE_ID+LONG_PKT+DATA_FMT) |
| Settle count | 0x14 | ✓ (webOS value) |
| Preserved across stream_on SW_RST | Yes | ✓ |

## Additional Things We've Tried

1. **Hot-switch from PIX to AXI=0x60**: Primed VFE with PIX mode for 2
   frames, stopped CAMIF, switched to 0x60, restarted. PIX priming worked
   (PP_STATUS toggled) but 0x60 immediately dead after switch.

2. **Samsung's 0x214101 extended RAW mode**: Hardware masked it to 0x60
   (register readback confirmed). It's a HAL command format, not a register
   value.

3. **BUS_CMD=0x3FFF reload with AXI=0x60**: No effect.

4. **CAMIF_CFG=0x50 (bits 4+6)**: No effect.

5. **REG_UPDATE=0x7 (bits 0+1+2)**: No effect.

6. **GDSC CLAMP bit**: Already clear (0x300 = ENABLE+RETENTION, no CLAMP).

7. **RPM MM_FABRIC_HALT unhalt**: Sent unhalt for all 11 MMSS ports via
   `qcom_rpm_write(QCOM_RPM_MM_FABRIC_HALT)`, rc=0. No effect.

8. **MODULE_CFG=0**: Changed to 0x00400C04 (Opal value with DEMUX and path
   gates). Confirmed all vendors keep DEMUX enabled for raw. No effect on
   AXI=0x60 (but correct fix to keep).

9. **VFE_AXI_CFG=0x80000000**: Added per Samsung/Mako "out of order option".
   Confirmed across Samsung, HTC, Mako kernels. No effect on AXI=0x60.

10. **IRQ_MASK_0=0x00E00021**: Matched Samsung's exact minimal mask (no
    individual WM/stats bits). No effect.

11. **CAMIF_CMD=1 vs 5**: Samsung header defines 5 but code writes 1.
    Tried both. No effect.

## Live Register Readback During AXI=0x60 Streaming

Read via devmem while sensor was actively streaming (2254 CSIPHY IRQs):

```
CAMIF_STATUS:  0x00000000  (active but ZERO lines, ZERO pixels!)
MODULE_CFG:    0x00400C04  ✓
CORE_CFG:      0x00000001  ✓
AXI_OUT_MODE:  0x00000060  ✓
CAMIF_CFG:     0x00000010  ✓
BUS_CFG:       0x02AAA775  ✓
PP_STATUS:     0x00010000  (bit 16 set, no toggle = no DMA)
WM0_CFG:       0x00000001  ✓ (enabled)
WM0_PING:      0x7C500000  ✓ (valid address)
WM0_PONG:      0x7C700000  ✓ (valid address)
IRQ_COMP_MASK: 0x00000100  ✓ (WM0 in group 1)
CAMIF_WINDOW:  0x04000A00  ✓ (1024 lines, 2560 width)
FRAME_CFG:     0x00000000  ✓ (zeroed)
```

**The smoking gun**: CAMIF_STATUS stays at 0x00000000 with zero line and
pixel counters. The CAMIF is "active" (bit 31 cleared) but never counts
any incoming data when AXI_OUT_MODE=0x60.

When AXI_OUT_MODE=0x01 (PIX mode) with the EXACT SAME sensor output, the
CAMIF counts pixels normally and data flows.

## Power Domain Comparison

| Vendor | Footswitch | AXI Port Unhalt | VFE Power State |
|--------|-----------|-----------------|-----------------|
| Samsung | `regulator_enable(fs_vfe)` | `msm_bus_axi_portunhalt(VFE)` | Cycled per session |
| HTC | Same as Samsung | Same as Samsung | Cycled per session |
| webOS/Opal | `#define HP_DISABLE` (skip) | Never halted | Always on from boot |
| Our kernel | GDSC via `pm_runtime` | RPM unhalt added (rc=0) | Cycled per session |

## The Question

What else could prevent the CAMIF from counting pixels when AXI_OUT_MODE
changes from 0x01 to 0x60, when:
- Every discoverable VFE register matches vendor kernels
- The sensor outputs valid data (confirmed via PIX path)
- CSIPHY receives data (2000+ IRQs)
- GDSC is powered, clocks running, AXI port unhalted
- All internal path gates enabled (MODULE_CFG)
- AXI out-of-order enabled (0x600)

Is there a register we haven't found? A timing dependency? A bus
configuration outside the VFE address space? Something in the CSID or
CSIPHY that gates data flow based on VFE's AXI mode?

## Hardware Details
- SoC: APQ8060 (Qualcomm, same silicon die as MSM8660)
- VFE: Version 3.1, HW version 0x00030217
- Sensor: Aptina MT9M113 1.3MP, MIPI CSI-2, 1 data lane
- Device: HP TouchPad (Topaz), running Linux 6.18
- VFE base: 0x04500000
- CSIPHY1 base: 0x04900000
- MMSS_CLK_CTL base: 0x04000000
