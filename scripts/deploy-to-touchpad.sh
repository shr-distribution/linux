#!/bin/bash
# Deploy kernel to HP TouchPad
#
# This script deploys uImage.LuneOS to the TouchPad boot partition.
# It auto-detects if device is running webOS (novacom) or LuneOS (telnet).
#
# Usage: ./scripts/deploy-to-touchpad.sh [--reboot]
#
# Deployment Methods:
#   webOS:  Uses novacom to push files directly
#   LuneOS: Starts HTTP server and uses wget on device via telnet
#
# Boot Partition Layout:
#   /dev/mmcblk0p13 is mounted at:
#     - /boot in webOS
#     - /mnt/boot in LuneOS initramfs (NOT /boot which is tmpfs)
#
# moboot.next:
#   - Contains "LuneOS" to boot LuneOS on next reboot
#   - Deleted by LuneOS initramfs on successful boot
#   - Must be recreated before each reboot from LuneOS
#
# Prerequisites:
#   - uImage.LuneOS built (run scripts/pack-uimage.sh first)
#   - For webOS: novacom installed and device connected
#   - For LuneOS: Device accessible at 172.16.42.2 via USB network

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"
BUILD_OUTPUT="$PARENT_DIR/build-output"

UIMAGE="$BUILD_OUTPUT/uImage.LuneOS"
HOST_IP="172.16.42.1"
DEVICE_IP="172.16.42.2"
HTTP_PORT="8080"

DO_REBOOT=false
[ "$1" = "--reboot" ] && DO_REBOOT=true

# Check for uImage
if [ ! -f "$UIMAGE" ]; then
    echo "Error: uImage.LuneOS not found at $UIMAGE"
    echo "Run './scripts/pack-uimage.sh' first to create it"
    exit 1
fi

echo "=== Deploying kernel to HP TouchPad ==="
echo "Image: $UIMAGE ($(du -h "$UIMAGE" | cut -f1))"
echo ""

# Detect which mode the device is in
deploy_via_novacom() {
    echo "Detected: webOS (novacom)"
    echo ""

    echo "Step 1: Remounting /boot as read-write..."
    novacom run "file:///bin/mount" -- -o remount,rw /boot

    echo "Step 2: Pushing uImage.LuneOS to /boot..."
    novacom put file:///boot/uImage.LuneOS < "$UIMAGE"

    echo "Step 3: Setting moboot.next to LuneOS..."
    echo "LuneOS" | novacom put file:///boot/moboot.next

    echo "Step 4: Syncing filesystem..."
    novacom run file:///bin/sync

    echo ""
    echo "=== Deployment complete (webOS) ==="

    if [ "$DO_REBOOT" = true ]; then
        echo "Rebooting device..."
        novacom run file:///sbin/reboot 2>/dev/null || true
    fi
}

deploy_via_telnet() {
    echo "Detected: LuneOS (telnet at $DEVICE_IP)"
    echo ""

    # Start HTTP server in background
    echo "Step 1: Starting HTTP server on port $HTTP_PORT..."
    cd "$BUILD_OUTPUT"

    # Kill any existing server
    pkill -f "http.server $HTTP_PORT" 2>/dev/null || true
    sleep 1

    # Start new server
    python3 -m http.server $HTTP_PORT &>/dev/null &
    HTTP_PID=$!
    sleep 2

    # Verify server is running
    if ! curl -s -o /dev/null "http://localhost:$HTTP_PORT/uImage.LuneOS"; then
        echo "Error: Failed to start HTTP server"
        exit 1
    fi
    echo "  HTTP server running (PID: $HTTP_PID)"

    echo ""
    echo "Step 2: Deploying via wget on device..."
    echo "  NOTE: Boot partition is at /mnt/boot in LuneOS (not /boot)"

    # Deploy via netcat/telnet
    # IMPORTANT: Use /mnt/boot, NOT /boot (which is tmpfs in initramfs)
    timeout 90 nc "$DEVICE_IP" 23 <<EOF || true
rm -f /mnt/boot/uImage.LuneOS
wget -O /mnt/boot/uImage.LuneOS http://${HOST_IP}:${HTTP_PORT}/uImage.LuneOS
echo "LuneOS" > /mnt/boot/moboot.next
sync
echo "=== Deployment complete ==="
ls -la /mnt/boot/uImage.LuneOS
cat /mnt/boot/moboot.next
exit
EOF

    # Kill HTTP server
    kill $HTTP_PID 2>/dev/null || true

    echo ""
    echo "=== Deployment complete (LuneOS) ==="
    echo ""
    echo "IMPORTANT: moboot.next will be deleted on next boot."
    echo "To reboot now, run:  telnet $DEVICE_IP  then: reboot"

    if [ "$DO_REBOOT" = true ]; then
        echo ""
        echo "Rebooting device..."
        # Use sysrq trigger for reliable reboot
        timeout 5 nc "$DEVICE_IP" 23 <<EOF || true
sync
echo b > /proc/sysrq-trigger
EOF
    fi
}

# Try to detect device mode
if command -v novacom &>/dev/null && novacom run file:///bin/true 2>/dev/null; then
    deploy_via_novacom
elif ping -c 1 -W 2 "$DEVICE_IP" &>/dev/null; then
    deploy_via_telnet
else
    echo "Error: No device detected"
    echo ""
    echo "For webOS: Connect via USB and ensure novacom is working"
    echo "For LuneOS: Ensure USB network is configured:"
    echo "  sudo ip addr add ${HOST_IP}/24 dev <interface>"
    echo "  ping $DEVICE_IP"
    exit 1
fi
