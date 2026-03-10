#!/bin/bash
# Deploy Bluetooth modules and firmware to HP TouchPad
#
# This script deploys kernel modules (bluetooth.ko, btbcm.ko, hci_uart.ko)
# and optionally loads them on the device for Bluetooth testing.
#
# Usage: ./scripts/deploy-bt.sh [--load] [--device IP]
#
# Options:
#   --load      Load modules on device after deployment
#   --device    Specify device IP (default: 172.16.42.2)
#
# Prerequisites:
#   - Kernel modules built (in Scarthgap build or local)
#   - Device accessible at 172.16.42.2 via USB network

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"

# Scarthgap build paths
SCARTHGAP="/media/herrie/LuneOS/scarthgap/webos-ports/tmp-glibc"
BT_BUILD="$SCARTHGAP/work/tenderloin-webos-linux-gnueabi/linux-hp-tenderloin/6.18+git/linux-tenderloin-standard-build"

# Network config
HOST_IP="172.16.42.1"
DEVICE_IP="172.16.42.2"
HTTP_PORT="8889"

DO_LOAD=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --load)
            DO_LOAD=true
            shift
            ;;
        --device)
            DEVICE_IP="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--load] [--device IP]"
            exit 1
            ;;
    esac
done

TMPDIR="/tmp/bt-deploy-$$"
cleanup() {
    rm -rf "$TMPDIR"
    pkill -f "http.server $HTTP_PORT" 2>/dev/null || true
}
trap cleanup EXIT

mkdir -p "$TMPDIR/modules"

echo "=== Deploying Bluetooth to HP TouchPad ==="
echo ""

# Find and copy kernel modules
echo "Step 1: Gathering kernel modules..."

# Check Scarthgap build first, then local build
if [ -f "$BT_BUILD/net/bluetooth/bluetooth.ko" ]; then
    echo "  Using Scarthgap build modules"
    cp "$BT_BUILD/net/bluetooth/bluetooth.ko" "$TMPDIR/modules/"
    cp "$BT_BUILD/drivers/bluetooth/btbcm.ko" "$TMPDIR/modules/"
    cp "$BT_BUILD/drivers/bluetooth/hci_uart.ko" "$TMPDIR/modules/"
elif [ -f "$KERNEL_DIR/net/bluetooth/bluetooth.ko" ]; then
    echo "  Using local build modules"
    cp "$KERNEL_DIR/net/bluetooth/bluetooth.ko" "$TMPDIR/modules/"
    cp "$KERNEL_DIR/drivers/bluetooth/btbcm.ko" "$TMPDIR/modules/"
    cp "$KERNEL_DIR/drivers/bluetooth/hci_uart.ko" "$TMPDIR/modules/"
else
    echo "ERROR: Bluetooth modules not found!"
    echo "Build with: make -j\$(nproc) modules"
    exit 1
fi

echo "  Modules collected:"
ls -lh "$TMPDIR/modules/"

# Create tarball
echo ""
echo "Step 2: Creating deployment archive..."
cd "$TMPDIR"
tar czf bt-deploy.tar.gz modules
echo "  Created bt-deploy.tar.gz ($(du -h bt-deploy.tar.gz | cut -f1))"

# Deploy function for LuneOS (telnet)
deploy_via_telnet() {
    echo ""
    echo "Step 3: Deploying via telnet to $DEVICE_IP..."

    # Start HTTP server
    cd "$TMPDIR"
    pkill -f "http.server $HTTP_PORT" 2>/dev/null || true
    sleep 1
    python3 -m http.server $HTTP_PORT &>/dev/null &
    HTTP_PID=$!
    sleep 2

    if ! curl -s -o /dev/null "http://localhost:$HTTP_PORT/bt-deploy.tar.gz"; then
        echo "ERROR: Failed to start HTTP server"
        exit 1
    fi
    echo "  HTTP server running (PID: $HTTP_PID)"

    # Deploy and optionally load modules
    if [ "$DO_LOAD" = true ]; then
        LOAD_CMDS="
# Unload existing modules (ignore errors)
rmmod hci_uart 2>/dev/null || true
rmmod btbcm 2>/dev/null || true
rmmod bluetooth 2>/dev/null || true

# Load modules
insmod /tmp/bt-modules/modules/bluetooth.ko
insmod /tmp/bt-modules/modules/btbcm.ko
insmod /tmp/bt-modules/modules/hci_uart.ko
echo 'Modules loaded:'
lsmod | grep -E 'bluetooth|btbcm|hci_uart'
echo ''
echo 'BT devices:'
hciconfig -a 2>/dev/null || echo '(no hci devices yet - run hciattach)'
echo ''
echo 'To attach BT UART: hciattach /dev/ttyMSM1 bcm43xx 3000000 flow'
"
    else
        LOAD_CMDS="
echo 'Modules extracted. To load:'
echo '  insmod /tmp/bt-modules/modules/bluetooth.ko'
echo '  insmod /tmp/bt-modules/modules/btbcm.ko'
echo '  insmod /tmp/bt-modules/modules/hci_uart.ko'
echo '  hciattach /dev/ttyMSM1 bcm43xx 3000000 flow'
"
    fi

    REMOTE_OUTPUT=$(timeout 60 nc "$DEVICE_IP" 23 <<EOF || true
rm -rf /tmp/bt-modules /tmp/bt-deploy.tar.gz
mkdir -p /tmp/bt-modules
cd /tmp/bt-modules
wget -q http://${HOST_IP}:${HTTP_PORT}/bt-deploy.tar.gz
tar xzf bt-deploy.tar.gz
echo "Extracted:"
ls -la /tmp/bt-modules/modules/
$LOAD_CMDS
exit
EOF
)
    echo "$REMOTE_OUTPUT"

    kill $HTTP_PID 2>/dev/null || true
}

# Check device connection
if ping -c 1 -W 2 "$DEVICE_IP" &>/dev/null; then
    deploy_via_telnet
else
    echo ""
    echo "ERROR: Device not reachable at $DEVICE_IP"
    echo ""
    echo "For LuneOS initramfs, configure USB network:"
    echo "  sudo ip addr add ${HOST_IP}/24 dev <usb-interface>"
    echo "  ping $DEVICE_IP"
    echo ""
    echo "Archive ready at: $TMPDIR/bt-deploy.tar.gz"
    echo "Copy to device manually if needed."
    exit 1
fi

echo ""
echo "=== Bluetooth deployment complete ==="
if [ "$DO_LOAD" = true ]; then
    echo ""
    echo "Next steps on device:"
    echo "  hciattach /dev/ttyMSM1 bcm43xx 3000000 flow"
    echo "  hciconfig hci0 up"
    echo "  hcitool scan"
else
    echo ""
    echo "Run with --load to auto-load modules, or manually:"
    echo "  telnet $DEVICE_IP"
    echo "  cd /tmp/bt-modules"
    echo "  insmod modules/bluetooth.ko"
    echo "  insmod modules/btbcm.ko"
    echo "  insmod modules/hci_uart.ko"
    echo "  hciattach /dev/ttyMSM1 bcm43xx 3000000 flow"
fi
