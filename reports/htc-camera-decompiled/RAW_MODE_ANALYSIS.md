# RAW Mode Analysis from HTC Camera Binaries

## Decompiled Sources
- liboemcamera.so (991 functions, primary VFE control library)
- camera.vendor.msm8660.so (camera HAL)
- libhtccamera.so (HTC camera extensions)
- libcamerapp.so (camera application interface)
- libmmcamera_rawchipproc.so (DxO rawchip ISP - external to VFE)

## RAW Snapshot Configuration (from liboemcamera.so)

### AXI Output Mode Values
From `axi_raw_snapshot_config` function around line 36244:

```c
local_f4 = 0x60;  // AXI_OUT_MODE for raw snapshot (CAMIF_TO_AXI_VIA_OUTPUT_2)
```

This matches webOS kernel `*p = 0x60` for raw snapshot mode.

### RAW Depth Configuration
Different RAW depths use different burst configurations:

| Depth Value | Burst Divisor | Config Value |
|------------|---------------|--------------|
| 0 (8-bit)  | 8             | 0x2aaa771    |
| 1 (10-bit) | 6             | 0x2aaa775    |
| 2 (12-bit) | 5             | 0x2aaa779    |

### RAW Stride Calculation (line 36266)
```c
sVar4 = __aeabi_uidiv(iVar16 * 2 + -1 + uVar8);  // burst calculation
local_d6 = local_d6 & 0xfc00 | sVar4 - 1U & 0x3ff;
uVar5 = __aeabi_uidiv(iVar16 + -1 + uVar8, iVar16);
```

### Frame Drop Configuration
```c
local_fc = 0x3fff;  // Frame drop pattern (no drops)
```

## VFE Module Initialization Sequence

From `vfe_modules_init` (lines 46842-47034):

1. `vfe_util_write_hw_cmd` with cmd 0x42 (GET_HW_VERSION)
2. Conditional module initialization based on flags at param_2[0x29f2]:
   - bit 0: black_level_init
   - bit 4: mce_init
   - bit 5: chroma_suppression_init
   - bit 6: chroma_enhan_init (color enhancement)
   - bit 7: frame_skip_init
   - bit 8: clamp_init
   - bit 10: chroma_subsample_init
   - bit 11: demosaic_bpc_init
   - bit 12: demosaic_init
   - bit 13: sce_init (skin color enhancement)
   - bit 14: la_init (luma adaptation)
   - bit 15: rolloff_init
   - bit 16: gamma_init
   - bit 17: abf_init (auto bayer filter)
   - bit 18: color_correct_init
   - bit 19: demux_init
   - bit 20: asf_init
   - bit 21: wb_init

## VFE Command Codes (from vfe_util_write_hw_cmd calls)

| Command | Value | Description |
|---------|-------|-------------|
| GET_HW_VERSION | 0x42 | Get VFE hardware version |
| MODULE_CFG | 0x71 | Module configuration |
| BLACK_LEVEL | 0x09 | Black level config |
| BLACK_LEVEL_UPDATE | 0x33 | Black level update |
| DEMUX | 0x0B | Demux config |
| DEMUX_UPDATE | 0x21 | Demux update |
| DEMOSAIC | 0x68 | Demosaic config |
| WB | 0x0E | White balance config |
| WB_UPDATE | 0x24 | White balance update |
| COLOR_CORRECT | 0x0F | Color correction config |
| COLOR_CORRECT_UPDATE | 0x25 | Color correction update |
| GAMMA | 0x10 | Gamma config |
| GAMMA_UPDATE | 0x26 | Gamma update |
| LA | 0x11 | Luma adaptation config |
| LA_UPDATE | 0x27 | Luma adaptation update |
| COLOR_CONVERSION | 0x12 | Color conversion config |
| COLOR_CONVERSION_UPDATE | 0x28 | Color conversion update |
| CHROMA_SUPP | 0x13 | Chroma suppression config |
| CHROMA_SUPP_UPDATE | 0x29 | Chroma suppression update |
| MCE | 0x14 | Memory color enhancement |
| SCE | 0x15 | Skin color enhancement |
| CHROMA_SS | 0x19 | Chroma subsample config |
| STATS_AEC | 0x56 | AEC stats config |
| STATS_AF | 0x54 | AF stats config |
| STATS_HIST | 0x60 | Histogram stats |
| ABF | 0x74 | Auto bayer filter |
| ABF2 | 0x6C | Auto bayer filter 2 |
| ROLLOFF | 0x78, 0x79 | Lens rolloff correction |

## Chroma Subsample Configuration (from vfe_chroma_subsample_config)

Line 59431:
```c
iVar4 = vfe_util_write_hw_cmd(*param_2, 0, param_1, 0xc, 0x19, param_2);
```

Writes 12 bytes (0xc) of chroma subsample configuration with command 0x19.

Configuration bits (line 59425-59426):
- bit 2: NV12/NV21 vs NV16/NV61 selection
- bit 4: always set
- bit 5: vertical subsampling enable
- bits 0-1: cleared

## Other Output Mode Values

From axi_vfe_config around line 36350:

| Format Mask | Output Mode | Description |
|-------------|-------------|-------------|
| 0x86        | 0x200       | YUV420 (NV12/NV21) |
| 0x41        | 0x1a00      | YUV422 (NV16/NV61) |
| 0x20        | 0x204000    | Other format |

## Key Insights for Driver Development

1. **RAW mode uses 0x60 AXI output mode** - confirms webOS CAMIF_TO_AXI_VIA_OUTPUT_2

2. **Module enable is controlled via bit flags** - similar to our MODULE_CFG register

3. **Chroma subsample writes 12 bytes** - our CHROMA_SUBS_CFG register block

4. **VFE command IDs** - can be used to identify ioctl commands

5. **Frame drop pattern 0x3fff** - no frame dropping during raw capture

## DxO Rawchip (libmmcamera_rawchipproc.so)

This is a **separate** hardware ISP from DxO Labs, not part of VFE31:
- Device node: `/dev/rawchip0`
- Used for external image processing
- Has its own AF parameters per sensor (s5k3h2yx, imx175, s5k4e5yx, ov5693)
- Not relevant to VFE31 raw mode implementation

## Recommendations

1. For RAW mode implementation in camss driver:
   - Use AXI_OUT_MODE = 0x60 for CAMIF_TO_AXI
   - Use WM0 only (single write master)
   - Configure burst based on RAW bit depth

2. For YUV mode (current implementation):
   - AXI_OUT_MODE values from table above
   - WM0 for Y, WM1 for CbCr (or WM4/WM5 depending on output path)

3. Chroma subsample config is 12 bytes written with command 0x19

---

## Additional Analysis from camera.vendor.msm8660.so (Camera HAL)

### Camera Channel Types
From `cam_ops_ch_set_attr` and operation action calls:

| Channel ID | Purpose |
|------------|---------|
| 0 | Preview stream |
| 1 | Recording stream |
| 2 | Snapshot/JPEG stream |
| 3 | RAW snapshot stream |
| 5 | ZSL stream |

### Camera Operation Modes
From `cam_ops_action` calls:

| Action ID | Action Value | Operation |
|-----------|--------------|-----------|
| 0 | 0 | Stop Preview |
| 1 | 0 | Start Preview |
| 0 | 1 | Stop Recording |
| 1 | 1 | Start Recording |
| 1 | 2 | Snapshot related |
| 1 | 3 | Snapshot (JPEG) |
| 1 | 4 | Snapshot (JPEG) |
| 1 | 5 | ZSL operation |
| 1 | 6 | Get buffered frame |
| 0 | 7 | Cancel autofocus |
| 1 | 7 | Start autofocus |

### MM_CAMERA_PARM Configuration Codes
From `cam_config_set_parm` and `cam_config_get_parm` calls:

| Code | Parameter Name |
|------|----------------|
| 0x03 | DIMENSION |
| 0x24 | FLASH_STATE |
| 0x25 | FOCUS_DISTANCES |
| 0x2b | ZOOM_RATIO |
| 0x2c | MIN_FOCUS_DISTANCE |
| 0x2d | MAX_FOCUS_DISTANCE |
| 0x36 | PREVIEW_FORMAT |
| 0x3c | CHANNEL_FORMAT (stream image format) |
| 0x3d | CHANNEL_ATTRIBUTE |
| 0x41 | SENSOR_INFO |
| 0x44 | ZOOM_STATUS |
| 0x45 | MAX_PICTURE_SIZE |
| 0x46 | MAX_PREVIEW_SIZE |
| 0x51 | SENSOR_PARAMS |
| 0x52 | VIDEO_SIZE_TABLE |
| 0x53-0x56 | SIZE_TABLE entries |
| 0x57-0x5a | SIZE_TABLE_SIZE entries |
| 0x5c | VFE_OUTPUT_ENABLE |
| 0x5d | FOCUS_RECT |
| 0x5e | AE_RECT |
| 0x5f | HFR_FRAME_SKIP |
| 0x60 | ZOOM_VALUE |
| 0x66 | AWB_INFO |
| 0x68 | ISO_VALUE (snapshot) |
| 0x69 | ISO_VALUE (preview) |
| 0x6a | AWB_GAINS |
| 0x6d-0x6e | FOCUS_INFO |
| 0x6f | (unknown - set during CAF) |
| 0x76-0x78 | STATS_INFO |
| 0x7b | CAF_SENSOR_DATA |
| 0x80 | HISTOGRAM |
| 0x83 | EXIF_INFO |
| 0x84 | SNAP_EXPOSURE_TIME |
| 0x85 | CURR_EXPOSURE_TIME |
| 0x87 | SENSOR_METADATA |
| 0x88 | FLASH_INFO |
| 0x90-0x92 | CAPABILITIES |
| 0x96 | FOCUS_RANGE |
| 0x97 | ZOOM_INFO |

### Camera Operation Modes (MM_CAMERA_OP_MODE)

| Mode | Description |
|------|-------------|
| MM_CAMERA_OP_MODE_VIDEO | Video recording mode |
| MM_CAMERA_OP_MODE_ZSL | Zero Shutter Lag mode |
| MM_CAMERA_OP_MODE_CAPTURE | Still image capture |

### Buffer Initialization Parameters

From `initHeapMem` function (line 16191):
```
Init Heap: pmem_type, num_of_buf, buf_len, cbcr_off
```

Buffer types (pmem_type):
- 3: Preview/Video buffers (uses local_3c = 2)
- 4-5: Snapshot buffers (uses local_3c = 4)
- 0x17: Special histogram buffers

### Raw Snapshot Channel Configuration

From `initRawSnapshotChannel` (line 26200):
```c
cam_ops_ch_set_attr(camera_handle, 3, attributes);  // Channel 3 = RAW
cam_config_set_parm(camera_handle, 0x3c, format);   // Set channel format
```

Format structure for RAW:
- `local_2c = 3` (channel type)
- `local_28 = 3` (format type)
- `local_24 = raw_width`
- `local_20 = raw_height`

### Snapshot Format Configuration

From `initSnapshotFormat` (line 25741):
```c
local_2c = 2;  // Channel type for JPEG snapshot
local_18 = postview_width;
local_20 = picture_height;
local_28 = picture_format;
local_1c = thumbnail_format;
local_24 = picture_width;
local_14 = thumbnail_height;
```

### Default Postview Size
From `configSnapshotDimension` (line 22917-22918):
```c
if (postview_width == 0 && postview_height == 0) {
    postview_width = 0x200;   // 512
    postview_height = 0x180;  // 384
}
```

### Dimension Ratio Calculation

The camera HAL uses `getDimensionRatio` to match aspect ratios between preview, picture, and postview sizes. This ensures proper scaling without cropping artifacts.

### ZSL (Zero Shutter Lag) Support

The camera supports ZSL mode via:
- `MM_CAMERA_OPS_ZSL` operation (action 5)
- `MM_CAMERA_OPS_GET_BUFFERED_FRAME` (action 6)
- `setZSLChannelAttribute` function

### Key Insights for VFE31 Driver

1. **Channel 3 is dedicated to RAW snapshot** - separate from JPEG channel (2)

2. **Format code 0x3c is critical** - used to set stream/channel image format

3. **Dimension parameter 0x03** - primary way to configure capture dimensions

4. **VFE_OUTPUT_ENABLE (0x5c)** - controls which VFE outputs are active

5. **Buffer allocation uses ION memory** with fallback to ashmem

6. **Multiple size tables** - separate tables for preview, picture, and video sizes

7. **Histogram uses special buffer type 0x17**
