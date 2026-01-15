# Boot Timing Optimization Report

This report documents the boot timing analysis and optimizations for the HP TouchPad running Linux 6.18.

## Current Boot Timing (Kernel #23)

| Component | Time | Initcall Level | Status |
|-----------|------|----------------|--------|
| ulpi_init | 16.5s | subsys_initcall | Base |
| ci_hdrc_platform_register | 17.3s | subsys_initcall | Optimized |
| gadget_cfs_init | 17.4s | subsys_initcall | Optimized |
| ci_hdrc_msm_driver_init | 19.9s | subsys_initcall_sync | Optimized |
| mmc0 (eMMC) detected | 23.8s | - | Good |
| mmcblk0 partitions | 24s | - | Good |
| ecmmod_init | 28.3s | fs_initcall | Optimized |
| eth_driver_init (g_ether) | 28.4s | fs_initcall | Optimized |
| qcom_usb_hs_phy_driver_init | 32.3s | module_init | **Bottleneck** |
| qcom_usb_hs_phy_probe | 52s | deferred probe | **Bottleneck** |
| **g_ether ready** | **55.4s** | - | Target for improvement |
| mmc1 (WiFi SDIO) | 71.3s | - | Late |
| **Run /init** | **96.3s** | - | |

## Optimizations Applied

### 1. USB ChipIdea Core (drivers/usb/chipidea/core.c)
- Changed from `module_init` to `subsys_initcall`
- Moved from ~59s to ~17s

### 2. USB ChipIdea MSM (drivers/usb/chipidea/ci_hdrc_msm.c)
- Changed from `module_platform_driver` to `subsys_initcall_sync`
- Moved from ~59s to ~20s

### 3. USB Gadget ConfigFS (drivers/usb/gadget/configfs.c)
- Changed from `module_init` to `subsys_initcall`
- Moved from ~64s to ~17s

### 4. USB Function Drivers (include/linux/usb/composite.h)
- Changed `DECLARE_USB_FUNCTION_INIT` macro from `module_init` to `fs_initcall`
- Moved ecm/rndis/geth drivers from ~63s to ~28s

### 5. g_ether Driver (drivers/usb/gadget/legacy/ether.c)
- Changed from `module_usb_composite_driver` to explicit `fs_initcall`
- Moved from ~64s to ~28s

### 6. QCOM RNG/PRNG (drivers/crypto/qcom-rng.c)
- Changed from `module_platform_driver` to `subsys_initcall`
- Enabled hwrng_support for faster entropy
- Reduced init_encrypted wait from ~6.7s to ~0.5s

## Remaining Bottleneck: USB HS PHY

The main remaining bottleneck is the Qualcomm USB HS PHY driver:

```
qcom_usb_hs_phy_driver_init: 32.3s (module_init)
qcom_usb_hs_phy_probe success: 52s (20 second gap!)
```

### Investigation Findings

1. **Regulators ready early**: regulator.18 and regulator.19 (PM8058 LDOs for USB PHY) are registered at 16s
2. **Driver registers late**: qcom_usb_hs_phy_driver_init at 32s (module_init)
3. **Probe defers repeatedly**: ci_hdrc.0 returns -EPROBE_DEFER multiple times from 20s-29s
4. **Long deferred probe gap**: 20 second gap between driver init (32s) and successful probe (52s)
5. **Devlink creation**: regulator-to-ULPI devlinks only created at 52s

### Failed Optimization Attempts

Moving qcom_usb_hs_phy to earlier initcalls caused USB to stop working:
- `subsys_initcall`: USB completely broken (PHY registered before ULPI bus)
- `fs_initcall`: Caused race condition, g_ether at 71s instead of 55s

The issue appears to be complex dependency ordering between:
- ULPI bus driver
- PHY core subsystem
- Reset controller
- Regulators
- Deferred probe workqueue timing

## Boot Timeline Visualization

```
0s     10s    20s    30s    40s    50s    60s    70s    80s    90s   100s
|------|------|------|------|------|------|------|------|------|------|
       [ULPI/CI_HDRC init]
              [eMMC ready]
                     [PHY driver]
                                          [PHY probe OK]
                                                [g_ether]
                                                       [WiFi]
                                                                    [init]
```

## Commits

1. `736b8053869b` - usb: Use earlier initcall levels for faster USB gadget availability
2. `a01e3c66b101` - crypto: qcom-rng: Enable hwrng and early init for faster boot

## Next Steps

1. Investigate deferred_probe_timeout kernel parameter
2. Analyze why there's a 20s gap between PHY driver init and probe success
3. Consider device tree dependency annotations
4. Profile the deferred probe workqueue behavior

## Historical Comparison

| Metric | Before Optimization | After Optimization | Improvement |
|--------|--------------------|--------------------|-------------|
| g_ether ready | ~63s | ~55s | 8s faster |
| init_encrypted | ~6.7s | ~0.5s | 6.2s faster |
| CI HDRC init | ~59s | ~17-20s | 40s faster |
| Run /init | ~100s | ~96s | 4s faster |

Note: The deferred probe mechanism adds variability to boot times.
