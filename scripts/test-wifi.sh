#!/bin/bash
#
# test-wifi.sh - Automated WiFi testing script for HP TouchPad
#
# This script:
# 1. Ensures HTTP server is running (reuses if already running)
# 2. Uploads ath6kl modules to device
# 3. Creates firmware symlinks
# 4. Loads modules and shows results
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${KERNEL_DIR}/../build-output"
DEVICE_IP="172.16.42.2"
HTTP_PORT="8000"
HOST_IP="172.16.42.1"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info() { echo -e "${GREEN}==>${NC} $*"; }
warn() { echo -e "${YELLOW}WARNING:${NC} $*"; }
error() { echo -e "${RED}ERROR:${NC} $*" >&2; exit 1; }

# Check device connectivity
check_device() {
    if ! ping -c 1 -W 2 "$DEVICE_IP" &>/dev/null; then
        error "Device not reachable at $DEVICE_IP"
    fi
    info "Device is reachable"
}

# Start HTTP server if not already running
ensure_http_server() {
    if curl -s -o /dev/null -w "%{http_code}" "http://localhost:${HTTP_PORT}/" 2>/dev/null | grep -q "200"; then
        info "HTTP server already running on port ${HTTP_PORT}"
        return 0
    fi

    info "Starting HTTP server on port ${HTTP_PORT}..."
    cd "$BUILD_DIR"
    python3 -m http.server "$HTTP_PORT" &>/dev/null &
    HTTP_PID=$!
    sleep 2

    if ! curl -s -o /dev/null "http://localhost:${HTTP_PORT}/" 2>/dev/null; then
        error "Failed to start HTTP server"
    fi
    info "HTTP server started (PID: $HTTP_PID)"
}

# Run commands on device via telnet
run_on_device() {
    local timeout="${2:-10}"
    (echo "$1"; sleep "$timeout") | telnet "$DEVICE_IP" 2>&1 | grep -v "^Trying\|^Connected\|^Escape\|^$\|^LuneOS"
}

# Main test function
test_wifi() {
    info "Testing WiFi on HP TouchPad"

    check_device
    ensure_http_server

    info "Step 1: Uploading ath6kl modules..."
    run_on_device "cd /tmp && rm -f ath6kl*.ko && wget -q http://${HOST_IP}:${HTTP_PORT}/drivers/net/wireless/ath/ath6kl/ath6kl_core.ko && wget -q http://${HOST_IP}:${HTTP_PORT}/drivers/net/wireless/ath/ath6kl/ath6kl_sdio.ko && ls -la ath6kl*.ko" 10

    info "Step 2: Creating firmware symlinks and deploying regulatory.db..."
    run_on_device "cd /lib/firmware/ath6k/AR6003/hw2.1.1 && ln -sf fw-3.bin fw-4.bin && ln -sf fw-3.bin fw-5.bin && ls -la fw-*.bin" 5
    run_on_device "wget -q -O /lib/firmware/regulatory.db http://${HOST_IP}:${HTTP_PORT}/regulatory.db 2>/dev/null; wget -q -O /lib/firmware/regulatory.db.p7s http://${HOST_IP}:${HTTP_PORT}/regulatory.db.p7s 2>/dev/null; ls -la /lib/firmware/regulatory.db* 2>/dev/null || echo 'regulatory.db not available on host'" 5

    info "Step 3: Unloading any existing modules..."
    run_on_device "rmmod ath6kl_sdio 2>/dev/null; rmmod ath6kl_core 2>/dev/null; echo 'Modules unloaded'" 3

    info "Step 4: Loading ath6kl modules..."
    run_on_device "cd /tmp && insmod ath6kl_core.ko && insmod ath6kl_sdio.ko && echo 'Modules loaded, waiting...'" 5

    info "Step 5: Checking results (dmesg)..."
    sleep 3
    echo ""
    echo "=== dmesg output (ath6kl related) ==="
    run_on_device "dmesg | grep -i 'ath6kl\|mmc1.*ath\|unable\|error\|failed' | tail -25" 5

    echo ""
    echo "=== Network interfaces ==="
    run_on_device "ip link show 2>/dev/null | grep -A1 'wlan'" 3

    echo ""
    info "Test complete. Check output above for errors."
}

# Parse arguments
case "${1:-test}" in
    test)
        test_wifi
        ;;
    upload)
        check_device
        ensure_http_server
        info "Uploading modules..."
        run_on_device "cd /tmp && rm -f ath6kl*.ko && wget -q http://${HOST_IP}:${HTTP_PORT}/drivers/net/wireless/ath/ath6kl/ath6kl_core.ko && wget -q http://${HOST_IP}:${HTTP_PORT}/drivers/net/wireless/ath/ath6kl/ath6kl_sdio.ko && ls -la ath6kl*.ko" 10
        ;;
    load)
        check_device
        info "Loading modules..."
        run_on_device "cd /tmp && rmmod ath6kl_sdio 2>/dev/null; rmmod ath6kl_core 2>/dev/null; insmod ath6kl_core.ko && insmod ath6kl_sdio.ko" 5
        sleep 3
        run_on_device "dmesg | tail -30" 5
        ;;
    dmesg)
        check_device
        run_on_device "dmesg | grep -i 'ath6kl\|mmc1\|sdio\|mmci' | tail -40" 5
        ;;
    *)
        echo "Usage: $0 [test|upload|load|dmesg]"
        echo "  test   - Full test (upload, symlinks, load, check)"
        echo "  upload - Just upload modules"
        echo "  load   - Just load modules (assumes already uploaded)"
        echo "  dmesg  - Show relevant dmesg"
        ;;
esac
