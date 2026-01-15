# Boot Timing Optimization Report

This report documents the boot timing analysis and optimizations for the HP TouchPad running Linux 6.18.

## Current Boot Timing (Kernel #3 - CONFIG_BLK_DEV_RAM disabled)

| Component | Time | Initcall Level | Status |
|-----------|------|----------------|--------|
| ulpi_init | 15.5s | subsys_initcall | Base |
| ci_hdrc_platform_register | 15.8s | subsys_initcall | Optimized |
| gadget_cfs_init | 15.8s | subsys_initcall | Optimized |
| ci_hdrc_msm_driver_init | 17.4s | subsys_initcall_sync | Optimized |
| g_ether driver registers | 25.4s | fs_initcall | No UDC yet |
| qcom_usb_hs_phy_driver_init | 31.5s | module_init | Optimized |
| qcom_usb_hs_phy_probe | 38.6s | deferred probe | **Improved!** |
| **g_ether ready** | **40.7s** | - | **14.7s faster** |
| loop_init | 48.6s-52s | device_initcall | 3.4s (could disable) |
| **Run /init** | **93.9s** | - | |

## Previous Boot Timing (Kernel #23)

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
| qcom_usb_hs_phy_driver_init | 32.3s | module_init | Bottleneck |
| qcom_usb_hs_phy_probe | 52s | deferred probe | Bottleneck |
| g_ether ready | 55.4s | - | - |
| mmc1 (WiFi SDIO) | 71.3s | - | Late |
| Run /init | 96.3s | - | |

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

### 7. Disabled CONFIG_BLK_DEV_RAM (tenderloin_debug_defconfig)
- RAM disk driver was taking 3.5s during brd_init
- Removing it allowed deferred probe to run earlier
- **Result: PHY probe improved from 52s to 38.6s (13.4s faster)**
- **g_ether ready improved from 55.4s to 40.7s (14.7s faster)**

## Remaining Slow Initcalls

After disabling BLK_DEV_RAM, the remaining slow initcalls are:

| Initcall | Duration | Notes |
|----------|----------|-------|
| param_sysfs_builtin_init | 13.1s | Module param sysfs |
| gsbi_driver_init | 9.4s | I2C bus registration |
| chr_dev_init | 5.6s | Character devices |
| ssbi_driver_init | 4.1s | SSBI bus |
| loop_init | 3.4s | Loop devices (could disable) |
| msm_serial_init | 2.6s | Serial console |
| init_encrypted | 2.5s | Encryption init |

### Notes on USB HS PHY

The PHY probe delay has been significantly reduced by removing the RAM disk driver.
The gap between driver init and probe success went from 20s down to ~7s.

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

### Current (After BLK_DEV_RAM disabled)
```
0s     10s    20s    30s    40s    50s    60s    70s    80s    90s   100s
|------|------|------|------|------|------|------|------|------|------|
       [ULPI/CI init]
              [g_ether reg]
                     [PHY init][PHY probe]
                                    [g_ether ready!]
                                                                 [init]
```

### Previous (Kernel #23)
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

1. ~~Investigate deferred_probe_timeout kernel parameter~~ (already set to 5s)
2. ~~Analyze why there's a 20s gap between PHY driver init and probe success~~ (Fixed by disabling BLK_DEV_RAM)
3. Consider disabling CONFIG_BLK_DEV_LOOP if not needed (saves 3.4s)
4. Investigate param_sysfs_builtin_init taking 13s
5. Consider optimizing gsbi_driver_init (9.4s for I2C bus registration)

## Historical Comparison

| Metric | Original | After USB Opts | After BLK_DEV_RAM | Total Improvement |
|--------|----------|----------------|-------------------|-------------------|
| g_ether ready | ~63s | ~55s | **~40.7s** | **22.3s faster** |
| PHY probe | - | ~52s | **~38.6s** | **13.4s faster** |
| init_encrypted | ~6.7s | ~0.5s | ~2.5s | 4.2s faster |
| CI HDRC init | ~59s | ~17-20s | ~15-17s | 42s faster |
| Run /init | ~100s | ~96s | **~93.9s** | **6.1s faster** |

Note: The deferred probe mechanism adds variability to boot times.
Disabling BLK_DEV_RAM had a much larger impact than expected due to how it
allowed the deferred probe workqueue to run earlier during boot.
