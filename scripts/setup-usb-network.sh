#!/bin/bash
# Setup USB network for HP TouchPad
#
# This script configures the host-side USB network interface for
# communicating with the TouchPad running LuneOS.
#
# Usage: sudo ./scripts/setup-usb-network.sh
#
# The script will:
#   1. Auto-detect the USB gadget network interface
#   2. Disable NetworkManager management (prevents interference)
#   3. Configure IP address 172.16.42.1/24
#   4. Wait for carrier (link) to come up
#
# After running this script, you can:
#   - ping 172.16.42.2
#   - telnet 172.16.42.2
#   - ./scripts/deploy-to-touchpad.sh

set -e

HOST_IP="172.16.42.1"
DEVICE_IP="172.16.42.2"
TIMEOUT=60

# Check for root
if [ "$EUID" -ne 0 ]; then
    echo "This script requires root privileges."
    echo "Usage: sudo $0"
    exit 1
fi

echo "=== HP TouchPad USB Network Setup ==="
echo ""

# Find USB gadget interface (RNDIS/CDC Ethernet)
# Look for interfaces named enx* or usb* that correspond to USB gadgets
find_gadget_interface() {
    # Method 1: Look for interface with specific USB vendor/product
    for iface in /sys/class/net/enx* /sys/class/net/usb*; do
        [ -e "$iface" ] || continue
        iface_name=$(basename "$iface")

        # Check if it's a USB device
        if [ -e "$iface/device" ]; then
            # Get the USB device path
            usb_path=$(readlink -f "$iface/device")
            if echo "$usb_path" | grep -q "usb"; then
                # Check for RNDIS gadget (0525:a4a2) or similar
                if [ -e "$iface/device/../idVendor" ]; then
                    vendor=$(cat "$iface/device/../idVendor" 2>/dev/null || true)
                    product=$(cat "$iface/device/../idProduct" 2>/dev/null || true)
                    if [ "$vendor" = "0525" ]; then
                        echo "$iface_name"
                        return 0
                    fi
                fi
                # Fallback: any USB network interface
                echo "$iface_name"
                return 0
            fi
        fi
    done
    return 1
}

# Wait for USB device to appear
echo "Step 1: Looking for USB gadget interface..."
IFACE=""
for i in $(seq 1 30); do
    IFACE=$(find_gadget_interface 2>/dev/null) && break
    printf "."
    sleep 1
done
echo ""

if [ -z "$IFACE" ]; then
    echo "Error: No USB gadget interface found"
    echo ""
    echo "Make sure:"
    echo "  1. TouchPad is connected via USB"
    echo "  2. TouchPad is booted into LuneOS (not webOS)"
    echo "  3. USB gadget driver is loaded on TouchPad"
    echo ""
    echo "Current USB devices:"
    lsusb | grep -iE "palm|hp|gadget|rndis|cdc|0525" || echo "  (no relevant devices found)"
    exit 1
fi

echo "  Found interface: $IFACE"

# Disable NetworkManager management
echo ""
echo "Step 2: Disabling NetworkManager management..."
if command -v nmcli &>/dev/null; then
    nmcli device set "$IFACE" managed no 2>/dev/null || true
    echo "  NetworkManager: $IFACE set to unmanaged"
else
    echo "  NetworkManager not found, skipping"
fi

# Configure IP address
echo ""
echo "Step 3: Configuring IP address..."
# Remove any existing IP
ip addr flush dev "$IFACE" 2>/dev/null || true
# Add our IP
ip addr add "${HOST_IP}/24" dev "$IFACE" 2>/dev/null || true
ip link set "$IFACE" up
echo "  IP address: $HOST_IP/24"

# Wait for carrier
echo ""
echo "Step 4: Waiting for carrier (up to ${TIMEOUT}s)..."
for i in $(seq 1 $TIMEOUT); do
    if ip link show "$IFACE" | grep -q "LOWER_UP"; then
        echo ""
        echo "  Carrier detected after ${i}s!"
        break
    fi
    printf "."
    sleep 1
done
echo ""

# Check final status
if ip link show "$IFACE" | grep -q "LOWER_UP"; then
    echo ""
    echo "=== USB Network Ready ==="
    echo ""
    echo "Interface: $IFACE"
    echo "Host IP:   $HOST_IP"
    echo "Device IP: $DEVICE_IP"
    echo ""

    # Try to ping device
    if ping -c 1 -W 2 "$DEVICE_IP" &>/dev/null; then
        echo "Device is reachable!"
        echo ""
        echo "You can now:"
        echo "  telnet $DEVICE_IP"
        echo "  ./scripts/deploy-to-touchpad.sh"
    else
        echo "Warning: Device not responding to ping yet"
        echo "Try: ping $DEVICE_IP"
    fi
else
    echo ""
    echo "Warning: No carrier detected after ${TIMEOUT}s"
    echo ""
    echo "The interface is configured but link is not up."
    echo "This usually means the TouchPad hasn't finished booting."
    echo ""
    echo "Try waiting a bit longer, then run:"
    echo "  ip link show $IFACE"
    echo "  ping $DEVICE_IP"
fi
