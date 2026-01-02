=======================================
HP Pre3 (mantaray) Boot Debugging Notes
=======================================

:Author: webOS Ports Project
:Date: January 2026

Overview
========

This document captures the debugging efforts to bring up mainline Linux
(6.x kernel) on the HP Pre3 smartphone (codename "mantaray", board "rib")
with Qualcomm MSM7230 SoC.

Hardware Summary
================

- **SoC**: Qualcomm MSM7230 (MSM7x30 family)
- **CPU**: Single-core Scorpion ARMv7 @ ~1GHz
- **Memory**: 461MB total in TWO separate banks
  - Bank 1: 206 MB at 0x00000000 - 0x0cdfffff
  - Bank 2: 255 MB at 0x40000000 - 0x4fefffff
- **Interrupt Controller**: VIC (Vectored Interrupt Controller), NOT GIC
  - 4 banks of 32 interrupts (128 total)
- **PMIC**: PM8058 via SSBI bus
- **Display**: MDDI interface

Boot Environment
================

The HP Pre3 uses Palm's "bootie" bootloader which:

1. Loads kernel from USB or NAND
2. Passes ATAGs (not device tree) to kernel
3. **Rejects appended DTB** - silently fails if DTB is appended to zImage
4. Uses custom uImage format (nova-installer-image)

USB Device IDs::

    0830:8051 - bootie mode (ready for kernel load)
    0830:8054 - webOS device mode
    0830:8056 - alternative webOS device mode

Boot Findings and Fixes
=======================

1. Appended DTB Rejection
-------------------------

**Problem**: Palm bootie rejects kernels with appended DTB.

**Solution**: Use ``CONFIG_ARM_BUILTIN_DTB=y`` to embed DTB into the
compressed kernel image. Required fixing position-independent addressing
in ``arch/arm/boot/compressed/head.S``::

    /* Original (broken - uses link-time address): */
    ldr r8, =__builtin_dtb_begin

    /* Fixed (position-independent): */
    adr     lr, .L__builtin_dtb_offset
    ldr     r8, [lr]
    add     r8, r8, lr

2. Memory Layout Discovery
--------------------------

**Problem**: Initial DT claimed 512MB contiguous RAM at 0x0, which is wrong.

**Discovery**: Reading ``/proc/iomem`` on legacy 2.6.32 kernel revealed::

    Memory: 206MB 255MB = 461MB total

    System RAM at:
      0x00200000 - 0x0cdfffff (Bank 1, ~206MB usable after modem)
      0x40000000 - 0x4fefffff (Bank 2, ~255MB at 1GB offset)

**Note**: First 2MB (0x0 - 0x1fffff) reserved for modem/AMSS firmware.

3. TEXT_OFFSET for Modem Reservation
------------------------------------

**Problem**: Kernel must not load into modem memory area (first 2MB).

**Solution**: Added to ``arch/arm/Makefile``::

    textofs-$(CONFIG_ARCH_MSM7X30) := 0x00208000

This ensures kernel text starts at 0x00208000, after modem reservation.

4. VIC Interrupt Controller
---------------------------

**Problem**: MSM7x30 uses VIC, not GIC. Initial driver had 2 banks.

**Solution**:
- Fixed to 4 banks of 32 interrupts (128 total)
- Changed initialization to write 0 to EN register directly
  (instead of 0xFFFFFFFF to ENCLEAR)

5. Missing Base Oscillator Clocks
---------------------------------

**Problem**: Clock driver missing TCXO and LPXO definitions.

**Solution**: Added to ``drivers/clk/qcom/gcc-msm7x30.c``::

    TCXO: 19.2 MHz - base oscillator for many peripherals
    LPXO: 24.576 MHz - timer uses LPXO/4 = 6.144 MHz for DGT

6. CPU Clock Reference
----------------------

**Problem**: DT referenced ``<&gcc ACPU_CLK>`` but this clock isn't
available in the mainline clock driver (requires ACPU clock controller).

**Solution**: Removed CPU clock reference from ``qcom-msm7x30.dtsi``.

Current Status (January 2026)
=============================

**Status**: Kernel still crashes early in boot.

**Symptoms**:
- HP logo appears briefly
- Device falls back to bootie, then boots legacy kernel from NAND
- No USB enumeration as mainline kernel
- No serial output visible (DEBUG_LL configured but not verified working)

**What Works**:
- DTB is properly embedded and detected (verified via hexdump)
- Kernel loads and starts decompression
- Early boot code runs (HP logo appears)

**What's Unknown**:
- Whether VIC initialization succeeds
- Whether timer starts correctly
- Whether memory configuration is accepted
- Exact crash point

Next Steps to Investigate
=========================

1. **Framebuffer Console**: Enable early framebuffer output for debugging
   - MSM7x30 has MDP4 display controller
   - Could show kernel messages on screen

2. **Machine Descriptor**: Verify DT_MACHINE_START handling for qcom,msm7x30
   - Modern multi-platform kernels may need specific setup
   - Legacy kernel had extensive early IO mappings in map_io()

3. **Early IO Mappings**: MSM7x30 may need early device mappings for:
   - VIC at 0xc0080000
   - Timer at 0xc0100000
   - Clock controller at 0xab800000

4. **ATAG Compatibility**: Verify ARM_ATAG_DTB_COMPAT works correctly
   - Bootie passes ATAGs, kernel should merge with DTB

Legacy Kernel Reference
=======================

Key files from webOS 2.6.32 kernel:

- ``arch/arm/mach-msm/board-rib.c`` - Board initialization
- ``arch/arm/mach-msm/io.c`` - Early IO mappings
- ``arch/arm/mach-msm/devices-msm7x30.c`` - Device definitions

Important legacy values::

    msm_shared_ram_phys = 0x0FF00000  (Pre3 specific)
    boot_params = 0x00200100

Test Kernel Versions
====================

Multiple kernel iterations were tested during debugging:

- v1-v6: Various fixes for DTB, VIC, clocks
- v7: TEXT_OFFSET fix
- v8: Position-independent DTB addressing
- v9: Two-bank memory configuration
- v10: Single-bank (206MB) for simplified testing

Commands Used
=============

Boot device into bootie mode::

    novacom -w run file:///sbin/tellbootie recover

Load kernel via novacom::

    novacom boot mem:// < nova-installer-image.uImage

Check device mode::

    lsusb | grep -i palm

Build commands::

    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- mantaray_defconfig
    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc) zImage dtbs

    # Force DTB rebuild if needed:
    rm -f arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dtb
