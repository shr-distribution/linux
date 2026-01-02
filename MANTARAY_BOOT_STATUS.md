# HP Pre3 (Mantaray) Mainline Linux Boot Status

## Date: 2026-01-02 (Updated)

## Summary

We are working on booting mainline Linux kernel on the HP Pre3 (codename "mantaray")
with Qualcomm MSM7230 SoC. The device uses Palm's "bootie" bootloader which loads
kernels via novacom USB protocol.

## Critical Finding: No DTB Support in Bootie

The webOS kernel (Linux 2.6.32) **predates device tree support** on ARM. Device tree
was only added to ARM Linux around version 3.x. This means:

- **Bootie uses ATAGS**, not device tree
- **Machine ID 3076** (RIB) is passed to the kernel
- **ATAGS location**: 0x00200100
- Bootie validates zImage format and **rejects any appended data**

## What Works

### Kernel Image Format
The correct format for bootie is a **multi-file uImage**:

```bash
# Step 1: Create kernel uImage from zImage
mkimage -A arm -O linux -T kernel -C none -a 0x00208000 -e 0x00208000 \
    -n "Linux-kernel" -d arch/arm/boot/zImage uKernel

# Step 2: Create ramdisk uImage (MUST use -C none)
mkimage -A arm -O linux -T ramdisk -C none -n "Initrd" \
    -d initrd.cpio.gz uRamdisk

# Step 3: Combine into multi-file image
mkimage -A arm -O linux -T multi -C none -n "Linux-mantaray" \
    -d uKernel:uRamdisk uMulti
```

### Kernel Execution
- The kernel (without DTB) IS accepted by bootie
- The kernel DOES execute (device leaves bootie mode, USB disconnects)
- The kernel then crashes/panics and device reboots to webOS
- This confirms the image format is correct

## What Doesn't Work

### Device Tree (DTB) Appending - CONFIRMED BLOCKED

Bootie specifically validates the zImage against its internal `_magic_end` marker
at offset 0x2C. Any data appended after this point causes rejection.

**Tested and rejected:**
1. Simple DTB append: `cat zImage dtb > zImage-with-dtb` - REJECTED
2. Minimal 300-byte DTB append - REJECTED
3. Patching zImage `_magic_end` to include DTB - REJECTED
4. DTB before zImage - REJECTED (and would break boot anyway)

**Conclusion**: Bootie does NOT support any modification to the zImage format.

### Current Crash Without DTB
Without DTB, the kernel crashes because:
- Modern DT-based drivers (VIC, timer, clock, pinctrl) cannot probe
- No hardware information available
- Kernel panics in early boot

## webOS Kernel Analysis

From `/home/herrie/webos/Pre3/webos-linux-kernel-pre3/arch/arm/mach-msm/board-rib.c`:

```c
MACHINE_START(RIB, "Rib")
    .boot_params = 0x00200100,    // ATAGS location
    .fixup = rib_fixup,           // Fix memory reporting bug
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
```

The webOS kernel uses:
- **Machine ID 3076** (MACH_RIB)
- **Platform data** structures for hardware (no DT)
- **Board file** with hardcoded hardware definitions

## Proposed Solutions

### Solution 1: Compile DTB into Kernel (Recommended)

Embed the DTB as a C array in the kernel's .init section:

1. Convert DTB to C: `xxd -i dtb > builtin_dtb.h`
2. Create kernel code that provides this DTB when none is passed
3. Modify `arch/arm/boot/compressed/head.S` to use builtin DTB
4. Kernel sees DTB, drivers can probe

**Pros**: Clean solution, works with existing drivers
**Cons**: Requires kernel modification, DTB changes need rebuild

### Solution 2: Add Machine ID 3076 Support

Create a board file for machine ID 3076 that:
1. Registers with MACHINE_START(RIB, ...)
2. Provides a pointer to compiled-in DTB
3. Sets up the fixup for memory (first 2MB hack)

**Pros**: Follows ARM boot conventions
**Cons**: Complex integration with DT-based drivers

### Solution 3: Platform Data Fallback

Modify drivers to work with platform data when no DT:
1. Add platform_device registrations for VIC, timer, clocks
2. Use machine ID 3076 to trigger platform setup
3. Keep DT support but add non-DT fallback

**Pros**: Works without DT modification
**Cons**: Significant driver changes, not upstreamable

## Hardware Details

- **SoC**: Qualcomm MSM7230 (MSM7x30 family)
- **CPU**: Single-core Scorpion ARMv7 @ ~1GHz
- **Machine ID**: 3076 (RIB)
- **ATAGS**: 0x00200100
- **Interrupt Controller**: VIC (128 interrupts, 4 banks)
- **Memory**: First 2MB reserved for modem, RAM starts at 0x00200000
- **Shared RAM**: 0x0FF00000 (modem communication)

## Drivers Created (DT-based)

1. **Clock driver** (`drivers/clk/qcom/gcc-msm7x30.c`) - ~53 clocks
2. **Pinctrl driver** (`drivers/pinctrl/qcom/pinctrl-msm7x30.c`) - 182 GPIOs
3. **VIC interrupt controller** (`drivers/irqchip/irq-qcom-vic.c`)
4. **Timer driver** (`drivers/clocksource/timer-msm.c`)
5. **Device tree** (`arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dts`)

## Files

- **Kernel config**: `.config` (based on multi_v7_defconfig)
- **Device tree**: `arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dts`
- **Working uImage**: `/tmp/uMulti` (executes but crashes)
- **webOS board file**: `webos-linux-kernel-pre3/arch/arm/mach-msm/board-rib.c`

## Commands Reference

```bash
# Enter bootie mode from webOS
novacom run file:///sbin/tellbootie recover

# Check device status
novacom -l
# mantaray-bootie = bootie mode
# mantaray-linux = webOS running

# Boot kernel
novacom boot mem:// < uMulti

# Check USB device type
lsusb | grep -i palm
# 0830:8051 = bootie mode
# 0830:8054/8056 = webOS running
```

## Next Immediate Step

Implement **Solution 1** (Compile DTB into Kernel):

1. Modify `arch/arm/boot/compressed/head.S` to check for a built-in DTB
   when no appended DTB is found
2. Create `arch/arm/boot/compressed/builtin-dtb.S` with the DTB as data
3. Update the Makefile to include it when `CONFIG_ARM_BUILTIN_DTB=y`
4. Rebuild and test
