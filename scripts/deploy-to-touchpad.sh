#!/bin/bash
# Deploy kernel to HP TouchPad via novacom
#
# This script:
# 1. Mounts /boot as read-write
# 2. Pushes uImage.LuneOS to /boot
# 3. Creates/updates moboot.default to boot LuneOS by default
#
# Prerequisites:
# - Device connected via USB with novacom access
# - uImage.LuneOS built (run scripts/pack-uimage.sh first)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"

UIMAGE="$PARENT_DIR/uImage.LuneOS"
BOOT_IMAGE_NAME="LuneOS"

# Check for novacom
if ! command -v novacom &> /dev/null; then
    echo "Error: novacom not found"
    exit 1
fi

# Check device connection
if ! novacom run file:///bin/true 2>/dev/null; then
    echo "Error: No device connected via novacom"
    exit 1
fi

# Check for uImage
if [ ! -f "$UIMAGE" ]; then
    echo "Error: uImage.LuneOS not found at $UIMAGE"
    echo "Run './scripts/pack-uimage.sh' first to create it"
    exit 1
fi

echo "=== Deploying kernel to HP TouchPad ==="
echo ""

# Step 1: Remount /boot as read-write
echo "Step 1: Remounting /boot as read-write..."
novacom run "file:///bin/mount" -- -o remount,rw /boot
echo "  Done."

# Step 2: Push uImage.LuneOS
echo ""
echo "Step 2: Pushing uImage.LuneOS to /boot..."
novacom put file:///boot/uImage.LuneOS < "$UIMAGE"
echo "  Done."

# Step 3: Set moboot.default to webOS (safe default)
echo ""
echo "Step 3: Setting default boot to webOS..."
echo "webOS" | novacom put file:///boot/moboot.default
echo "  Done."

# Step 3b: Set moboot.next to LuneOS (one-time boot)
echo ""
echo "Step 3b: Setting next boot to LuneOS..."
echo "$BOOT_IMAGE_NAME" | novacom put file:///boot/moboot.next
echo "  Done."

# Step 4: Verify
echo ""
echo "Step 4: Verifying deployment..."
echo "  /boot contents:"
novacom run file:///bin/ls /boot/uImage.LuneOS /boot/moboot.default /boot/moboot.next
echo ""
echo "  moboot.default (safe default):"
novacom run file:///bin/cat /boot/moboot.default
echo ""
echo "  moboot.next (one-time boot):"
novacom run file:///bin/cat /boot/moboot.next

# Step 5: Remount /boot as read-only (optional, for safety)
echo ""
echo "Step 5: Remounting /boot as read-only..."
novacom run "file:///bin/mount" -- -o remount,ro /boot
echo "  Done."

echo ""
echo "=== Deployment complete ==="
echo ""
echo "Next reboot: LuneOS (one-time via moboot.next)"
echo "After that:  webOS (default via moboot.default)"
echo ""
echo "To reboot now, run: novacom run file:///sbin/reboot"
