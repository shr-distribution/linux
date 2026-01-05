#!/bin/bash
# Pack kernel and DTB into uImage for HP TouchPad
#
# Usage: ./scripts/pack-uimage.sh [dtb-variant]
#   dtb-variant: topaz (default), topaz-3g, opal, opal-3g
#
# Output: ../build-output/uImage.LuneOS
#
# IMPORTANT: moboot requires specific uImage format:
#   - Multi-file uImage with load/entry address 0x00000000
#   - Kernel uImage (Image 0) with load/entry 0x40208000
#   - Initramfs uImage (Image 1) with compression type "none" in header
#     (even if the data is gzip compressed - moboot checks the header)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"
BUILD_OUTPUT="$PARENT_DIR/build-output"

VARIANT="${1:-topaz}"
ZIMAGE="$KERNEL_DIR/arch/arm/boot/zImage"
DTB="$KERNEL_DIR/arch/arm/boot/dts/qcom/qcom-apq8060-${VARIANT}.dtb"
INITRAMFS_INPUT="$PARENT_DIR/initramfs-uImage.bin"
OUTPUT="$BUILD_OUTPUT/uImage.LuneOS"

# Create output directory
mkdir -p "$BUILD_OUTPUT"

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

if [ ! -f "$INITRAMFS_INPUT" ]; then
    echo "Error: initramfs not found at $INITRAMFS_INPUT"
    echo "Provide either:"
    echo "  - A uImage containing the initramfs"
    echo "  - A raw cpio.gz initramfs archive"
    exit 1
fi

# Check for required tools
if ! command -v mkimage &> /dev/null; then
    echo "Error: mkimage not found. Install u-boot-tools package."
    exit 1
fi

if ! command -v dumpimage &> /dev/null; then
    echo "Error: dumpimage not found. Install u-boot-tools package."
    exit 1
fi

TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

echo "Packing uImage for HP TouchPad ($VARIANT)..."
echo "  Kernel:    $ZIMAGE"
echo "  DTB:       $DTB"
echo "  Initramfs: $INITRAMFS_INPUT"

# Prepare initramfs - ensure it has correct uImage header with -C none
# moboot checks the compression type in the header and rejects "gzip"
INITRAMFS_TYPE=$(file "$INITRAMFS_INPUT")
if echo "$INITRAMFS_TYPE" | grep -q "u-boot legacy uImage"; then
    echo "  Initramfs is a uImage, checking compression header..."

    # Check if it has the wrong compression type (gzip instead of none)
    if echo "$INITRAMFS_TYPE" | grep -q "(gzip)"; then
        echo "  WARNING: Initramfs uImage has gzip compression header"
        echo "  Repacking with -C none header (required by moboot)..."

        # Extract raw data from uImage
        dumpimage -T ramdisk -p 0 -o "$TMPDIR/initramfs.cpio.gz" "$INITRAMFS_INPUT"

        # Repack with -C none header
        mkimage -A arm -O linux -T ramdisk -C none -a 0x00000000 -e 0x00000000 \
            -n "initramfs" -d "$TMPDIR/initramfs.cpio.gz" "$TMPDIR/initramfs-uImage.bin" \
            > /dev/null
        INITRAMFS="$TMPDIR/initramfs-uImage.bin"
    else
        echo "  Initramfs uImage header OK"
        INITRAMFS="$INITRAMFS_INPUT"
    fi
elif echo "$INITRAMFS_TYPE" | grep -q "gzip compressed"; then
    echo "  Initramfs is raw cpio.gz, creating uImage..."
    mkimage -A arm -O linux -T ramdisk -C none -a 0x00000000 -e 0x00000000 \
        -n "initramfs" -d "$INITRAMFS_INPUT" "$TMPDIR/initramfs-uImage.bin" \
        > /dev/null
    INITRAMFS="$TMPDIR/initramfs-uImage.bin"
else
    echo "Error: Unknown initramfs format: $INITRAMFS_TYPE"
    exit 1
fi

# Create zImage with appended DTB
cat "$ZIMAGE" "$DTB" > "$TMPDIR/zImage-dtb"

# Create kernel uImage with correct load/entry addresses
echo "  Creating kernel uImage..."
mkimage -A arm -O linux -T kernel -C none -a 0x40208000 -e 0x40208000 \
    -n "LuneOS/6.18+git/tenderloin" -d "$TMPDIR/zImage-dtb" "$TMPDIR/uImage-kernel" \
    > /dev/null

# Create multi-file uImage
# CRITICAL: Load/entry addresses MUST be 0x00000000 for moboot
echo "  Creating multi-file uImage..."
mkimage -A arm -O linux -T multi -C none -a 0x00000000 -e 0x00000000 \
    -n "HP Touchpad boot" -d "$TMPDIR/uImage-kernel:$INITRAMFS" "$OUTPUT" \
    > /dev/null

echo ""
echo "Created: $OUTPUT"
mkimage -l "$OUTPUT"
echo ""
ls -lh "$OUTPUT"

# Also create moboot.next for auto-boot
echo "LuneOS" > "$BUILD_OUTPUT/moboot.next"
echo "Created: $BUILD_OUTPUT/moboot.next"

# Archive the build for historical record
ARCHIVE_DIR="$BUILD_OUTPUT/archive"
mkdir -p "$ARCHIVE_DIR"

# Get commit info
if command -v git &> /dev/null && git rev-parse --git-dir > /dev/null 2>&1; then
    COMMIT_SHORT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
    COMMIT_FULL=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
    COMMIT_MSG=$(git log -1 --format="%s" 2>/dev/null || echo "unknown")
    COMMIT_DATE=$(git log -1 --format="%ci" 2>/dev/null || echo "unknown")
else
    COMMIT_SHORT="unknown"
    COMMIT_FULL="unknown"
    COMMIT_MSG="unknown"
    COMMIT_DATE="unknown"
fi

TIMESTAMP=$(date +%Y%m%d-%H%M%S)
ARCHIVE_NAME="${TIMESTAMP}_${COMMIT_SHORT}"
ARCHIVE_PATH="$ARCHIVE_DIR/$ARCHIVE_NAME"

mkdir -p "$ARCHIVE_PATH"
cp "$OUTPUT" "$ARCHIVE_PATH/"

# Save build info
cat > "$ARCHIVE_PATH/build-info.txt" << BUILDINFO
Build Timestamp: $(date '+%Y-%m-%d %H:%M:%S')
Commit: $COMMIT_FULL
Commit Date: $COMMIT_DATE
Subject: $COMMIT_MSG
Variant: $VARIANT
Kernel: $ZIMAGE
DTB: $DTB
Initramfs: $INITRAMFS_INPUT
BUILDINFO

echo ""
echo "Archived to: $ARCHIVE_PATH"
