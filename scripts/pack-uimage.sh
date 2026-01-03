#!/bin/bash
# Pack kernel and DTB into uImage for HP TouchPad
#
# Usage: ./scripts/pack-uimage.sh [dtb-variant]
#   dtb-variant: topaz (default), topaz-3g, opal, opal-3g
#
# Output: ../uImage-dtb-zImage

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"

VARIANT="${1:-topaz}"
ZIMAGE="$KERNEL_DIR/arch/arm/boot/zImage"
DTB="$KERNEL_DIR/arch/arm/boot/dts/qcom/qcom-apq8060-${VARIANT}.dtb"
INITRAMFS="$PARENT_DIR/initramfs-uImage.bin"
OUTPUT="$PARENT_DIR/uImage.LuneOS"

# Validate inputs
if [ ! -f "$ZIMAGE" ]; then
    echo "Error: zImage not found at $ZIMAGE"
    echo "Run 'make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage' first"
    exit 1
fi

if [ ! -f "$DTB" ]; then
    echo "Error: DTB not found at $DTB"
    echo "Available DTBs:"
    ls -1 "$KERNEL_DIR/arch/arm/boot/dts/qcom/qcom-apq8060-"*.dtb 2>/dev/null || echo "  (none)"
    exit 1
fi

if [ ! -f "$INITRAMFS" ]; then
    echo "Error: initramfs uImage not found at $INITRAMFS"
    echo "Extract it from an existing uImage-dtb-zImage first"
    exit 1
fi

# Check for mkimage
if ! command -v mkimage &> /dev/null; then
    echo "Error: mkimage not found. Install u-boot-tools package."
    exit 1
fi

TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

echo "Packing uImage for HP TouchPad ($VARIANT)..."
echo "  Kernel: $ZIMAGE"
echo "  DTB:    $DTB"

# Create zImage with appended DTB
cat "$ZIMAGE" "$DTB" > "$TMPDIR/zImage-dtb"

# Create kernel uImage
mkimage -A arm -O linux -T kernel -C none -a 0x40208000 -e 0x40208000 \
    -n "LuneOS/6.18+git/tenderloin" -d "$TMPDIR/zImage-dtb" "$TMPDIR/uImage-kernel" \
    > /dev/null

# Create multi-file uImage
mkimage -A arm -O linux -T multi -C none -a 0x00000000 -e 0x00000000 \
    -n "HP Touchpad boot" -d "$TMPDIR/uImage-kernel:$INITRAMFS" "$OUTPUT" \
    > /dev/null

echo ""
echo "Created: $OUTPUT"
mkimage -l "$OUTPUT"
echo ""
ls -lh "$OUTPUT"
