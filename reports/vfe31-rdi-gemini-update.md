# VFE31 RDI Investigation Update - For Gemini

## Results from Your Three Hypotheses

### Hypothesis 1: MIPI Data Type Filter - CONFIRMED BUG, FIXED, NOT SUFFICIENT
You were right - the CSIPHY MIPI_PROTOCOL_CONTROL register (offset 0x04)
DATA_FORMAT field at bits [20:19] was hardcoded to 0 (8-bit). We fixed it to
dynamically detect the sensor format and set 1 for RAW10. We also fixed
`stream_on` which was overwriting DATA_FORMAT back to 0 after SW_RST.

**However**: Fixing DATA_FORMAT alone did NOT fix RDI. AND the RAW-through-PIX
test (see below) worked even WITH DATA_FORMAT=0! So this field affects
unpacking but doesn't gate packet reception at the CAMIF level.

### Hypothesis 2: Shadow Register / REG_UPDATE Sequencing
We verified our sequence: WM config → AXI=0x60 → REG_UPDATE → CAMIF_CMD=0x5.
This matches the Samsung/Opal sequence. REG_UPDATE IS issued before CAMIF_CMD.

### Hypothesis 3: The "Warm Start" / Hot-Switch Test - CRITICAL RESULT

**We implemented the kernel-internal hot-switch test.** Here's what happened:

1. For RDI mode, first configure as PIX (AXI=0x01, CORE_CFG=0x41, CAMIF_CFG=0x40)
2. Start CAMIF, wait 150ms (~2 frames at 14fps)
3. **PIX priming succeeded**: CAMIF_STATUS=0x00000000 (active), PP_STATUS=0x00010000 (data flowing!)
4. Stop CAMIF immediately
5. Switch to AXI=0x60, CORE_CFG=0x01, CAMIF_CFG=0x10, MODULE_CFG=0
6. REG_UPDATE to latch
7. Restart CAMIF

**Result after switch to AXI=0x60**: CAMIF_STATUS=0x00000000 (active), but ZERO VFE IRQs, ZERO data. The CAMIF went blind immediately after switching to 0x60.

**This proves the RDI path failure is NOT a clock/state priming issue.** The PIX path is fully functional and the CAMIF sees RAW10 data through it. But the moment AXI_OUT_MODE changes to 0x60, data stops flowing to WM0.

## The RAW-through-PIX Diagnostic - BREAKTHROUGH

We set the sensor to RAW10 output but configured the VFE as normal PIX mode:
- Sensor: SGRBG10_1X10 1280x1024 (RAW10 Bayer)
- CSIPHY: UYVY8_2X8 (lie about format)
- VFE: AXI=0x01, CORE_CFG=0x46, CAMIF_CFG=0x40, MODULE_CFG=webOS value

**Result: 5 frames captured at 3.4fps!** The CAMIF counts pixels, VFE IRQs fire,
PP_STATUS toggles. The Y plane contains recognizable Bayer RAW image data
(tested with good lighting - clear image of a person with colored tape rolls).

The DEMUX interprets the RAW10 data as UYVY and splits it into Y+CbCr planes.
The Y channel contains the raw Bayer pixel values (every other pixel from the
raw stream). The image is usable after Bayer demosaicing in userspace.

## Key Register Values (All Match Samsung/Opal)

For AXI=0x60 (FAILS):
```
CORE_CFG     = 0x01 (pattern only, no INPUT_MUX_ENABLE)
MODULE_CFG   = 0x00 (all ISP disabled)
CAMIF_CFG    = 0x10 (bit 4, raw mode per Opal HAL)
FRAME_CFG    = 0x00 (zeroed per Samsung/Opal)
WINDOW_WIDTH = 0x04000A00 (height=1024, width=2560 = width*2)
AXI_OUT_MODE = 0x60 (CAMIF_TO_AXI)
BUS_CFG      = 0x02AAA775 (10-bit raw)
CAMIF_CMD    = 0x05 (START + CLEAR)
```

For AXI=0x01 (WORKS with RAW10 sensor):
```
CORE_CFG     = 0x46 (UYVY pattern + INPUT_MUX_ENABLE)
MODULE_CFG   = 0x01C00C0C (ISP modules enabled)
CAMIF_CFG    = 0x40 (bit 6, camif2vfe)
AXI_OUT_MODE = 0x01 (OUTPUT_1_AND_3)
```

## The Question

The AXI=0x60 CAMIF_TO_AXI_VIA_OUTPUT_2 path appears completely non-functional
on this APQ8060 VFE 3.1 (HW version 0x00030217). Even after:
- Matching ALL vendor register values
- Fixing CSIPHY DATA_FORMAT for RAW10
- Trying hot-switch from active PIX streaming
- Confirming CAMIF CAN see RAW10 through PIX path

**What else could gate the AXI=0x60 path?**

Possibilities we haven't explored:
1. Is there an AXI bus arbiter or bridge configuration outside VFE that gates
   the CAMIF_TO_AXI DMA path?
2. Could AXI=0x60 require a specific BUS_CMD (0x038) write that differs from
   the normal 0x7FFF reload?
3. Is there a VFE micro-controller or firmware that needs to be initialized
   for the raw bypass path?
4. Could the CAMIF_TO_AXI path be physically disconnected in the APQ8060
   revision (test silicon vs production)?
5. Samsung/HTC had different camera sensors (8MP+ main cameras). Could
   CAMIF_TO_AXI only work with certain MIPI configurations (2+ lanes,
   specific data rates)?
6. Is there a different AXI clock domain or bus port that needs enabling
   for the raw bypass path vs the ISP pipeline?

## Our Current Plan

If AXI=0x60 truly cannot work, implement RAW-through-PIX as production:
- RDI lines use AXI=0x01 with MODULE_CFG=0 (DEMUX disabled)
- Raw Bayer data flows through ISP pipeline as transparent wire
- WM0 captures raw data, userspace applies Bayer demosaicing

This is proven to work but adds ISP pipeline latency and the DEMUX
(even when "disabled") may still affect the data path.

Any other ideas before we accept AXI=0x60 as broken?
