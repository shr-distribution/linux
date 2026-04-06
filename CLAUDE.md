# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Linux 6.18 kernel port for the HP TouchPad tablet. The TouchPad uses a Qualcomm APQ8060 SoC (dual-core ARMv7 Scorpion @ 1.5GHz) and runs LuneOS, the open-source continuation of webOS.

**Hardware Variants:**
- Topaz: 9.7" display (WiFi and 3G variants)
- Opal: 7" display (WiFi and 3G variants, unreleased)

## Build Commands

```bash
# Set environment (can also use arm-linux-gnueabi-)
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

# Full build
make -j$(nproc) zImage modules dtbs

# Build specific module
make M=drivers/misc/a6 modules

# Pack kernel for moboot bootloader (creates ../build-output/uImage.LuneOS)
./scripts/pack-uimage.sh topaz

# Deploy to device (auto-detects webOS/novacom or LuneOS/telnet)
./scripts/deploy-to-touchpad.sh [--reboot]
```

## Key Architecture

### Device Tree Structure

The hardware configuration is defined in device trees, not board files:
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - Main hardware config (~2000 lines)
- `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi` - Base SoC peripherals
- `arch/arm/boot/dts/qcom/qcom-apq8060-topaz.dts` - Topaz WiFi variant (default)

### Custom/Modified Drivers

**Touchscreen** (`drivers/input/touchscreen/cy8ctma395*.c`):
- UART serdev driver at 4 Mbps via GSBI10
- Processes proprietary binary touch data packets
- Uses weighted centroid algorithm for coordinate calculation

**Battery Controller** (`drivers/misc/a6/`):
- Palm A6 power management IC driver
- Supports dual battery system
- JTAG firmware update support

**Display** (`drivers/gpu/drm/msm/`):
- MDP4 display controller with LCDC output
- Z180 2D graphics engine (Adreno 200)

### Build Output Layout

```
../build-output/
├── uImage.LuneOS          # Final deployable image
├── moboot.next            # Boot selection file
├── arch/arm/boot/         # Intermediate build files
└── archive/               # Historical builds with commit info
```

### Initramfs

The initramfs is external to this repo at `../initramfs-uImage.bin`. It provides:
- USB gadget network (ECM at 172.16.42.2)
- telnetd for debug access
- Hardware test mode loop

## Deployment

**Via webOS (device running stock OS):**
```bash
novacom run file://bin/mount -- -o remount,rw /boot
novacom put file:///boot/uImage.LuneOS < ../build-output/uImage.LuneOS
echo "LuneOS" | novacom put file:///boot/moboot.next
novacom run file://sbin/tellbootie  # reboot
```

**Via LuneOS (device already booted):**
```bash
# Configure host USB network first
sudo ip addr add 172.16.42.1/24 dev <usb-interface>
telnet 172.16.42.2
```

## moboot Requirements

The bootloader has specific uImage format requirements:
1. Multi-file uImage with load/entry address `0x00000000`
2. Kernel Image 0 with load/entry `0x40208000`
3. Initramfs Image 1 must have compression header `none` (even if gzip data)

The `pack-uimage.sh` script handles all of this automatically.

## Upstream Status

See `UPSTREAM_PATCH_PLAN.md` for the 28-patch submission plan organized into 10 series:
- Series 1-2: Clock infrastructure, DRM fixes
- Series 3-4: Touchscreen drivers
- Series 5-7: Media, camera, remoteproc
- Series 8-9: Device trees
- Series 10: Defconfig

## Reference Documentation

- `docs/touchpad-connection-guide.md` - Build, deploy, and connect instructions
- `reports/hp_touchpad_mainline_status_report.md` - Hardware verification status
- `reports/DEVICE_TREE_CROSSCHECK_REPORT.md` - DT vs webOS kernel comparison
- `../schematics/` - Hardware schematics PDFs

## VFE31 Reference Kernels (Camera Driver)

The VFE31 (Video Front End) camera driver is used on MSM8660/APQ8060. Reference implementations:

**Local webOS kernel (primary reference):**
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c`
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.h`

**GitHub reference kernels:**
- Mako (Nexus 4, MSM8960): https://github.com/raden/ampang-AOSP-mako-kernel
  - `drivers/media/video/msm/vfe/msm_vfe31.c` and `msm_vfe31.h`
- LineageOS Mako: https://github.com/LineageOS/lge-kernel-mako
- Sony MSM8960: https://github.com/LineageOS/sony-kernel-msm8960
- LG G2 VFE31: https://gitlab.com/k2wl/g2_kernel/-/blob/master/drivers/media/video/msm/msm_vfe31.c
  - Contains raw snapshot mode (CAMIF_TO_AXI_VIA_OUTPUT_2) useful for RDI implementation

**Key VFE31 register values from webOS dumps:**
- DEMUX_EVEN_CFG (0x290) = 0xC9CA (16-bit combined, same to both EVEN and ODD)
- DEMUX_ODD_CFG (0x294) = 0xC9CA
- AXI_OUT_MODE (0x040) = 0x01 (OUTPUT_1_AND_3)
- XBAR_CFG1 (0x044) = 0x1A1B (VIDEO_MODE routing)
- MODULE_CFG (0x010) = 0x01C00C0C

**Local register dumps:**
- `reports/webos-preview-mode-dump.txt` - Live preview mode registers
- `reports/webos-video-mode-dump.txt` - Video recording registers
- `reports/webos-vfe-register-dump.txt` - General VFE dump

## Hardware Access

**From host (webOS running):** `novacom run file://bin/sh`
**From host (LuneOS debug):** `telnet 172.16.42.2`
**From host (LuneOS full):** `ssh root@172.16.42.2`

**Important:**
- Always use port 22 for SSH to the device (172.16.42.2:22), not port 2221. Port 2221 is the default on the host machine, not the TouchPad.
- Use `scp` to copy files to the device, not base64 encoding or other workarounds. Example: `scp file.txt root@172.16.42.2:/path/`

## Git Remotes

- `origin` - kernel.org stable
- `github` - Personal fork (Herrie82)
- `shr-github` - SHR distribution (shr-distribution/linux)
- `shr-linux` - Local SHR repo (/home/herrie/Documents/GitHub/linux-shr)

**Always push to `shr-github`** for this project (not `github` which may have access issues).
