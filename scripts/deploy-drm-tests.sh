#!/bin/bash
# Deploy DRM/KMS test tools to HP TouchPad
#
# This script deploys the drm-tests package to the device and optionally
# runs the tests.
#
# Usage: ./scripts/deploy-drm-tests.sh [--run [mode]]
#
# Options:
#   --run         Run tests after deployment
#   --run quick   Run quick test mode (default)
#   --run full    Run full test mode
#   --run benchmark  Run benchmark mode
#
# Prerequisites:
#   - drm-tests.tar.gz created by pack-drm-tests.sh
#   - Device accessible at 172.16.42.2 via telnet

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"
BUILD_OUTPUT="$PARENT_DIR/build-output"
PACKAGE_TAR="$BUILD_OUTPUT/drm-tests.tar.gz"

HOST_IP="172.16.42.1"
DEVICE_IP="172.16.42.2"
HTTP_PORT="8080"
DEVICE_DIR="/tmp/drm-tests"

RUN_TESTS=false
TEST_MODE="quick"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

status() { echo -e "${GREEN}[*]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
error() { echo -e "${RED}[X]${NC} $1"; }

# Parse arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --run)
            RUN_TESTS=true
            if [ -n "$2" ] && [[ ! "$2" =~ ^-- ]]; then
                TEST_MODE="$2"
                shift
            fi
            ;;
        *)
            error "Unknown option: $1"
            echo "Usage: $0 [--run [quick|full|benchmark]]"
            exit 1
            ;;
    esac
    shift
done

# Check for package
if [ ! -f "$PACKAGE_TAR" ]; then
    error "Package not found: $PACKAGE_TAR"
    echo ""
    echo "Please run './scripts/pack-drm-tests.sh' first to create the package."
    exit 1
fi

# Check device connectivity
status "Checking device connectivity..."
if ! ping -c 1 -W 2 "$DEVICE_IP" &>/dev/null; then
    error "Cannot reach device at $DEVICE_IP"
    echo ""
    echo "Make sure the device is connected and USB network is configured:"
    echo "  sudo ip addr add ${HOST_IP}/24 dev <usb-interface>"
    exit 1
fi
status "Device reachable at $DEVICE_IP"

echo ""
echo "=========================================="
echo "  Deploy DRM Test Tools to HP TouchPad"
echo "=========================================="
echo ""

# Start HTTP server
status "Starting HTTP server on port $HTTP_PORT..."
cd "$BUILD_OUTPUT"

# Kill any existing server
pkill -f "http.server $HTTP_PORT" 2>/dev/null || true
sleep 1

# Start new server
python3 -m http.server $HTTP_PORT &>/dev/null &
HTTP_PID=$!
sleep 2

# Verify server is running
if ! curl -s -o /dev/null "http://localhost:$HTTP_PORT/drm-tests.tar.gz"; then
    error "Failed to start HTTP server"
    exit 1
fi
status "HTTP server running (PID: $HTTP_PID)"

# Deploy via telnet
status "Deploying to device..."

REMOTE_OUTPUT=$(timeout 60 nc "$DEVICE_IP" 23 <<EOF || true
# Clean up old installation
rm -rf $DEVICE_DIR

# Download package
cd /tmp
rm -f drm-tests.tar.gz
wget -O drm-tests.tar.gz http://${HOST_IP}:${HTTP_PORT}/drm-tests.tar.gz

# Extract (tarball contains drm-tests/ directory)
tar xzf drm-tests.tar.gz
rm -f drm-tests.tar.gz

# Verify
ls -la $DEVICE_DIR/
echo "DEPLOY_SUCCESS"
exit
EOF
)

# Kill HTTP server
kill $HTTP_PID 2>/dev/null || true

# Check for success
if echo "$REMOTE_OUTPUT" | grep -q "DEPLOY_SUCCESS"; then
    status "Deployment successful!"
    echo ""
    status "Package deployed to: $DEVICE_DIR"
else
    error "Deployment may have failed. Output:"
    echo "$REMOTE_OUTPUT"
    exit 1
fi

# Run tests if requested
if [ "$RUN_TESTS" = true ]; then
    echo ""
    status "Running DRM tests (mode: $TEST_MODE)..."
    echo ""
    echo "=========================================="
    echo "          Test Output"
    echo "=========================================="
    echo ""

    # Run tests with longer timeout for benchmark mode
    TIMEOUT_SECS=120
    [ "$TEST_MODE" = "benchmark" ] && TIMEOUT_SECS=300

    timeout $TIMEOUT_SECS nc "$DEVICE_IP" 23 <<EOF || true
cd $DEVICE_DIR
chmod +x run-drm-tests.sh
./run-drm-tests.sh $TEST_MODE
exit
EOF

    echo ""
    echo "=========================================="
fi

echo ""
status "Done!"
echo ""
status "To run tests manually on device:"
status "  telnet $DEVICE_IP"
status "  cd $DEVICE_DIR"
status "  ./run-drm-tests.sh [quick|full|benchmark]"
echo ""
