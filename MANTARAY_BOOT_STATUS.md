# HP Pre3 (Mantaray) Mainline Linux Boot Status

## Date: 2026-01-02

## Summary

We are working on booting mainline Linux kernel on the HP Pre3 (codename "mantaray")
with Qualcomm MSM7230 SoC. The device uses Palm's "bootie" bootloader which loads
kernels via novacom USB protocol.

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

Key points:
- Load address: 0x00208000 (same as webOS kernel)
- Entry point: 0x00208000
- Ramdisk MUST use `-C none` or bootie rejects it
- Multi-file format required for bootie to use correct bootargs

### Kernel Execution
- The kernel (without DTB) IS accepted by bootie
- The kernel DOES execute (device leaves bootie mode)
- The kernel then crashes/panics and device reboots to webOS
- This confirms the image format is correct

## What Doesn't Work

### Device Tree (DTB) Appending
When we append the DTB to zImage:
```bash
cat zImage qcom-msm7230-hp-pre3.dtb > zImage-with-dtb
```

Bootie **rejects** this image and stays in bootie mode (or hangs).

Possible reasons:
1. **zImage header validation**: The zImage has an "end" marker at offset 0x2C
   that bootie may validate against file size
2. **Size limit**: The DTB adds ~17KB, possibly exceeding some limit
3. **Unknown validation**: Bootie may check CRC or other fields

### Current Crash
Without DTB, the kernel crashes immediately because:
- No device tree = no hardware information
- VIC interrupt controller, timer, clocks, etc. cannot initialize
- Kernel panics early in boot

## Hardware Details

- **SoC**: Qualcomm MSM7230 (MSM7x30 family)
- **CPU**: Single-core Scorpion ARMv7 @ ~1GHz
- **Interrupt Controller**: VIC (not GIC)
- **Memory**: Shared RAM at 0x0FF00000 (modem communication)
- **USB**: Via novacom protocol for kernel loading

## Drivers Created

1. **Clock driver** (`drivers/clk/qcom/gcc-msm7x30.c`) - ~53 clocks
2. **Pinctrl driver** (`drivers/pinctrl/qcom/pinctrl-msm7x30.c`) - 182 GPIOs
3. **VIC interrupt controller** (`drivers/irqchip/irq-qcom-vic.c`)
4. **Timer driver** (`drivers/clocksource/timer-msm.c`)
5. **Device tree** (`arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dts`)

## Next Steps to Try

### Option 1: Disable Device Tree Requirement
Modify kernel config to work with ATAGS only:
- Disable `CONFIG_USE_OF` or make DTB optional
- Use machine ID matching instead of device tree
- This is a regression but might allow initial boot

### Option 2: Built-in DTB
Some architectures support building DTB into the kernel:
- Check for `CONFIG_BUILTIN_DTB` or similar
- May require kernel changes

### Option 3: Investigate zImage Header
- Understand exactly what bootie validates
- Possibly patch the zImage end marker after appending DTB
- Risky as it might break kernel's DTB detection

### Option 4: Use Initramfs with DTB
- Embed DTB in initramfs
- Have init script extract and pass DTB to kernel
- Requires kernel support for late DTB loading (unlikely)

### Option 5: Analyze Bootie Behavior
- Use webOS's bootie source if available
- Reverse engineer the validation logic
- Create image that passes validation

## Files

- **Kernel config**: `.config` (based on multi_v7_defconfig)
- **Device tree**: `arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dts`
- **Working uImage**: `/tmp/uMulti` (executes but crashes)
- **Rejected uImage**: `/tmp/uMulti-with-dtb` (with DTB, rejected by bootie)

## Commands Reference

```bash
# Enter bootie mode from webOS
novacom run file:///sbin/tellbootie recover

# Check device status
novacom -l

# Boot kernel
novacom boot mem:// < uMulti

# Check USB device type
lsusb | grep -i palm
# 0830:8051 = bootie mode
# 0830:8054/8056 = webOS
```
