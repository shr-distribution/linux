#!/bin/bash
# Deploy kernel to HP TouchPad via novacom
#
# This script:
# 1. Mounts /boot as read-write
# 2. Pushes uImage.LuneOS to /boot
# 3. Creates moboot.next for one-time boot to LuneOS
#    (moboot.next is deleted by initramfs on successful boot)
#
# Usage: ./scripts/deploy-to-touchpad.sh [--reboot]
#
# Prerequisites:
# - Device connected via USB with novacom access (webOS booted)
# - uImage.LuneOS built (run scripts/pack-uimage.sh first)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"
BUILD_OUTPUT="$PARENT_DIR/build-output"

UIMAGE="$BUILD_OUTPUT/uImage.LuneOS"
MOBOOT_NEXT="$BUILD_OUTPUT/moboot.next"
BOOT_IMAGE_NAME="LuneOS"

DO_REBOOT=false
[ "$1" = "--reboot" ] && DO_REBOOT=true

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

# Check for moboot.next
if [ ! -f "$MOBOOT_NEXT" ]; then
    echo "Error: moboot.next not found at $MOBOOT_NEXT"
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
echo "Step 2: Pushing uImage.LuneOS to /boot ($(du -h "$UIMAGE" | cut -f1))..."
novacom put file:///boot/uImage.LuneOS < "$UIMAGE"
echo "  Done."

# Step 3: Set moboot.next to LuneOS (one-time boot)
# Note: moboot.next is deleted by the LuneOS initramfs on successful boot
echo ""
echo "Step 3: Setting next boot to LuneOS (via moboot.next)..."
novacom put file:///boot/moboot.next < "$MOBOOT_NEXT"
echo "  Done."

# Step 4: Sync
echo ""
echo "Step 4: Syncing filesystem..."
novacom run file:///bin/sync
echo "  Done."

echo ""
echo "=== Deployment complete ==="
echo ""
echo "Next reboot: LuneOS (one-time via moboot.next)"
echo "After that:  webOS (moboot.next deleted by initramfs on successful boot)"
echo ""

if [ "$DO_REBOOT" = true ]; then
    echo "Rebooting device..."
    novacom run file:///sbin/tellbootie -- reboot 2>/dev/null || true
    echo ""
    echo "Device is rebooting. After boot, run:"
    echo "  ./scripts/test-touchpad-hardware.sh"
else
    echo "To reboot and test:"
    echo "  novacom run file:///sbin/tellbootie -- reboot"
    echo "  ./scripts/test-touchpad-hardware.sh"
fi
