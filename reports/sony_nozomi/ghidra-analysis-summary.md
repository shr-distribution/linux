# Sony Nozomi Camera Library Ghidra Analysis Summary

Analysis performed: 2026-04-17
Target: Sony Xperia S (Nozomi) MSM8660 camera libraries
Purpose: Extract VFE31 configuration values and patterns

## Binaries Analyzed

| Binary | Size | Functions Exported | Notes |
|--------|------|-------------------|-------|
| camera.msm8660.so | 91KB | 340/382 | High-level HAL, burst shot features |
| liboemcamera.so | 826KB | 1059/1219 | **Main VFE configuration** |
| libmmcamera_statsproc31.so | 83KB | 195/226 | AEC/AWB/AF statistics processing |
| v4l2-qcamera-app | 34KB | 87/142 | V4L2 client application |

## Critical Finding: DEMUX Configuration

**Function:** `vfe_demux_set_cfg_parms` @ 0x7bf9c in liboemcamera.so

This function configures the DEMUX module based on input pixel format:

### Bayer Formats (DEMUX_CFG period = 1)

| Case | Pattern | EVEN_CFG | ODD_CFG | Description |
|------|---------|----------|---------|-------------|
| 0 | GRBG | 0xAC | 0xC9 | Green-Red / Blue-Green |
| 1 | RGGB | 0xCA | 0x9C | Red-Green / Green-Blue |
| 2 | BGGR | 0x9C | 0xCA | Blue-Green / Green-Red |
| 3 | GBRG | 0xC9 | 0xAC | Green-Blue / Red-Green |

### YUV Formats (DEMUX_CFG period = 3)

| Case | Format | EVEN_CFG | ODD_CFG | Notes |
|------|--------|----------|---------|-------|
| 4 | UYVY | 0x9CAC | 0x9CAC | U-Y-V-Y byte order |
| 5 | VYUY | 0xAC9C | 0xAC9C | V-Y-U-Y byte order |
| 6 | YUYV | 0xC9CA | 0xC9CA | **webOS uses this** |
| 7 | YVYU | 0xCAC9 | 0xCAC9 | Y-V-Y-U byte order |

### Register Structure Offsets
```
param_1[0x00]: DEMUX_CFG (period in bits 0-2: 1=Bayer, 3=YUV)
param_1[0x0C]: DEMUX_EVEN_CFG
param_1[0x10]: DEMUX_ODD_CFG
```

### Key Insight
The DEMUX period field controls how many pixels are processed together:
- Period 1 (& 0xf8 | 1): Bayer pattern, process every 2 pixels
- Period 3 (& 0xf8 | 3): YUV pattern, process every 4 pixels

This matches webOS DEMUX_CFG = 0x03 for YUV processing.

## VFE Function Categories

### ISP Parameter Update Functions
All follow the pattern: `semc_vfe_interface_update_param_<module>()`
- `semc_vfe_interface_update_param_blacklevel`
- `semc_vfe_interface_update_param_defectpixelcorrection`
- `semc_vfe_interface_update_param_lensshadingcorrection`
- `semc_vfe_interface_update_param_noisereduction`
- `semc_vfe_interface_update_param_wbgain`
- `semc_vfe_interface_update_param_colorcorrection`
- `semc_vfe_interface_update_param_gamma`
- `semc_vfe_interface_update_param_colorconversion`
- `semc_vfe_interface_update_param_asf` (Adaptive Spatial Filter)
- `semc_vfe_interface_update_param_mce` (Memory Color Enhancement)
- `semc_vfe_interface_update_param_sce` (Skin Color Enhancement)
- `semc_vfe_interface_update_param_chromasupp`

### VFE Client API
```c
VFE_client_open()    @ 0x78270 - Opens VFE client instance
VFE_comp_create()    @ 0x78334 - Creates VFE component
VFE_comp_destroy()   @ 0x78358 - Destroys VFE component
```

### Debug Functions (Stubs - return immediately)
Many debug functions are defined but are empty stubs:
- `vfe_demux_debug`
- `vfe_demux_gain_debug`
- `vfe_wb_debug`
- `vfe_color_correct_debug`
- `vfe_chroma_enhan_config_debug`
- `vfe_abf_debug`
- `vfe_awb_stats_debug`
- `vfe_black_level_debug`
- `vfe_demosaic_debug`
- `vfe_chrom_supp_config_debug`

## Statistics Processing (libmmcamera_statsproc31.so)

### Statistics Types
- AEC (Auto Exposure Control)
- AF (Auto Focus)
- AWB (Auto White Balance)

### Key Functions
```c
stats_proc_client_process()    - Main stats processing
stats_proc_client_set_params() - Set stats parameters
stats_proc_client_get_params() - Get stats parameters
STATSPROC_client_open()        - Open stats client
aec_fast_conv_config()         - AEC fast convergence
aec_set_antibanding_status()   - Set anti-banding
awb_init()                     - Initialize AWB
af_process()                   - Process AF data
```

## Comparison with webOS

| Setting | Sony Nozomi | webOS | Match |
|---------|-------------|-------|-------|
| DEMUX_EVEN_CFG for YUYV | 0xC9CA | 0xC9CA | YES |
| DEMUX_ODD_CFG for YUYV | 0xC9CA | 0xC9CA | YES |
| DEMUX_CFG period | 0x03 | 0x03 | YES |

The Sony Nozomi uses the same DEMUX configuration values as webOS for YUV processing, confirming these are correct VFE31 values.

## Files Generated

All decompiled output saved to:
`reports/sony_nozomi/decompiled/`

- `liboemcamera.so_decompiled.c` (1.5MB) - Main analysis source
- `liboemcamera.so_symbols.txt` - Symbol table
- `liboemcamera.so_strings.txt` - String references
- `camera.msm8660.so_decompiled.c`
- `libmmcamera_statsproc31.so_decompiled.c`
- `v4l2-qcamera-app_decompiled.c`

## Implications for Mainline Driver

1. **DEMUX Configuration Confirmed**: The mainline driver should use 0xC9CA for both DEMUX_EVEN_CFG and DEMUX_ODD_CFG when processing YUYV input.

2. **Period Setting**: DEMUX_CFG period should be 3 (bits 0-2 = 0x03) for YUV formats.

3. **Module Pattern**: Sony uses a layered approach where `semc_camif_vfe_*` functions wrap `semc_vfe_interface_*` calls, which in turn configure hardware registers.

4. **Statistics Processing**: The VFE31 stats processing is a separate library (statsproc31), indicating stats can be handled independently from image path configuration.

## Next Steps

1. Verify DEMUX configuration in mainline driver matches these values
2. Cross-reference with webOS kernel source for complete picture
3. Look for AXI/XBAR configuration in remaining binaries if available
