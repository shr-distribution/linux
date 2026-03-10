#!/bin/bash
#
# bt-test.sh - Bluetooth testing script for HP TouchPad
#
# Deploys Bluetooth modules and tests serdev-based BCSP driver.
# The BCM4329 chip uses BCSP protocol via serdev (device tree binding).
#
# With serdev, the driver automatically:
#   - Probes when hci_uart module is loaded
#   - Controls GPIOs for power management
#   - Sends PSKEYs and handles WARM_RESET
#   - Creates hci0 device when ready
#
# Usage: ./scripts/bt-test.sh [options]
#
# Options:
#   --deploy-only    Only deploy modules, don't test
#   --test-only      Only test (modules already deployed)
#   --scan           Perform device scan after init
#   --monitor        Continuous monitoring mode
#   --diag           Run diagnostics
#   --help           Show this help
#

set -e

# Configuration
DEVICE_IP="172.16.42.2"
DEVICE_PORT="22"
DEVICE_USER="root"
SSH_OPTS="-o ConnectTimeout=10 -o StrictHostKeyChecking=no"

# Build directory for modules
BUILD_BASE="/media/herrie/LuneOS/scarthgap/webos-ports/tmp-glibc/work/tenderloin-webos-linux-gnueabi/linux-hp-tenderloin/6.18+git/linux-tenderloin-standard-build"

# Module paths
MODULES=(
    "net/bluetooth/bluetooth.ko"
    "drivers/bluetooth/btbcm.ko"
    "drivers/bluetooth/hci_uart.ko"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
info() { echo -e "${BLUE}[INFO]${NC} $*"; }
success() { echo -e "${GREEN}[OK]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

ssh_cmd() {
    ssh -p "$DEVICE_PORT" $SSH_OPTS "$DEVICE_USER@$DEVICE_IP" "$@"
}

scp_cmd() {
    scp -P "$DEVICE_PORT" $SSH_OPTS "$@"
}

check_device() {
    info "Checking device connectivity..."
    if ! ssh_cmd "echo ok" &>/dev/null; then
        error "Cannot connect to device at $DEVICE_IP:$DEVICE_PORT"
        echo "Make sure:"
        echo "  1. Device is booted and USB cable connected"
        echo "  2. USB network interface is configured:"
        echo "     sudo ip addr add 172.16.42.1/24 dev <usb-iface>"
        exit 1
    fi
    success "Device is reachable"
}

get_kernel_version() {
    ssh_cmd "uname -r"
}

check_modules_exist() {
    info "Checking if modules exist in build directory..."
    local missing=0
    for mod in "${MODULES[@]}"; do
        local path="$BUILD_BASE/$mod"
        if [[ ! -f "$path" ]]; then
            error "Module not found: $path"
            missing=1
        fi
    done
    if [[ $missing -eq 1 ]]; then
        error "Some modules are missing. Please build the kernel first."
        exit 1
    fi
    success "All modules found"
}

deploy_modules() {
    info "Deploying Bluetooth modules to device..."

    # Ensure /tmp has space
    local free_space=$(ssh_cmd "df /tmp | tail -1 | awk '{print \$4}'")
    if [[ $free_space -lt 20000 ]]; then
        error "Not enough space in /tmp (need ~20MB)"
        exit 1
    fi

    # Unload existing modules first
    info "Unloading existing Bluetooth modules..."
    ssh_cmd "rmmod hci_uart btbcm bluetooth 2>/dev/null || true"
    sleep 1

    # Copy modules
    for mod in "${MODULES[@]}"; do
        local src="$BUILD_BASE/$mod"
        local name=$(basename "$mod")
        info "  Copying $name..."
        scp_cmd "$src" "$DEVICE_USER@$DEVICE_IP:/tmp/$name"
    done

    success "Modules deployed to /tmp/"
}

unload_modules() {
    info "Unloading Bluetooth modules..."
    ssh_cmd "rmmod hci_uart 2>/dev/null; rmmod btbcm 2>/dev/null; rmmod bluetooth 2>/dev/null; true"
    sleep 1
}

load_modules() {
    info "Loading Bluetooth modules..."

    # Load in dependency order
    ssh_cmd "insmod /tmp/bluetooth.ko" || {
        if ssh_cmd "lsmod | grep -q '^bluetooth'"; then
            warn "bluetooth.ko already loaded"
        else
            error "Failed to load bluetooth.ko"
            return 1
        fi
    }

    ssh_cmd "insmod /tmp/btbcm.ko" || {
        if ssh_cmd "lsmod | grep -q '^btbcm'"; then
            warn "btbcm.ko already loaded"
        else
            error "Failed to load btbcm.ko"
            return 1
        fi
    }

    ssh_cmd "insmod /tmp/hci_uart.ko" || {
        if ssh_cmd "lsmod | grep -q '^hci_uart'"; then
            warn "hci_uart.ko already loaded"
        else
            error "Failed to load hci_uart.ko"
            return 1
        fi
    }

    success "Modules loaded"
}

check_serdev_probe() {
    info "Checking serdev probe status..."

    # Check if driver is bound to device
    local driver_link=$(ssh_cmd "ls -la /sys/bus/serial/devices/serial0-0/driver 2>/dev/null" || echo "")

    if echo "$driver_link" | grep -q "hci_uart_bcsp"; then
        success "Serdev driver bound: hci_uart_bcsp"
        return 0
    else
        # Check dmesg for probe result
        local probe_result=$(ssh_cmd "dmesg | grep 'hci_uart_bcsp serial0-0' | tail -1")
        if echo "$probe_result" | grep -q "failed"; then
            local err_code=$(echo "$probe_result" | grep -oE '\-[0-9]+' | tail -1)
            error "Serdev probe failed with error $err_code"

            case "$err_code" in
                -16)
                    echo "  Error -16 (EBUSY): Resource busy"
                    echo "  Possible causes:"
                    echo "    - GPIO already claimed by another driver"
                    echo "    - Serial port already in use"
                    echo "    - Check 'dmesg | grep GPIO' for conflicts"
                    ;;
                -19)
                    echo "  Error -19 (ENODEV): No such device"
                    echo "  Check device tree and compatible string"
                    ;;
                -22)
                    echo "  Error -22 (EINVAL): Invalid argument"
                    echo "  Check device tree properties"
                    ;;
            esac
            return 1
        else
            warn "Driver not bound (may not have probed yet)"
            return 1
        fi
    fi
}

wait_for_hci_device() {
    info "Waiting for HCI device (serdev auto-creates it)..."

    local attempts=0
    local max_attempts=10

    while [[ $attempts -lt $max_attempts ]]; do
        if ssh_cmd "hciconfig hci0 2>/dev/null" | grep -q "hci0"; then
            success "HCI device hci0 appeared"
            return 0
        fi
        attempts=$((attempts + 1))
        echo -n "."
        sleep 1
    done

    echo ""
    warn "HCI device did not appear after ${max_attempts}s"
    return 1
}

check_hci_device() {
    info "Checking HCI device..."

    local hci_info=$(ssh_cmd "hciconfig -a 2>&1")

    if echo "$hci_info" | grep -q "hci0"; then
        success "HCI device found: hci0"
        echo "$hci_info" | head -20
        return 0
    else
        error "No HCI device found"
        return 1
    fi
}

bring_up_hci() {
    info "Bringing up HCI device..."

    # First unblock rfkill if blocked
    ssh_cmd "rfkill unblock bluetooth 2>/dev/null" || true

    local result=$(ssh_cmd "hciconfig hci0 up 2>&1")
    local rc=$?

    if [[ $rc -eq 0 ]]; then
        success "HCI device is UP"
        return 0
    else
        error "Failed to bring up HCI device"
        echo "$result"
        return 1
    fi
}

check_bluetooth_status() {
    info "Bluetooth status:"
    echo "----------------------------------------"

    # HCI config
    echo -e "${BLUE}HCI Configuration:${NC}"
    ssh_cmd "hciconfig -a 2>/dev/null" || echo "No HCI device"
    echo ""

    # Loaded modules
    echo -e "${BLUE}Loaded modules:${NC}"
    ssh_cmd "lsmod | grep -E 'bluetooth|btbcm|hci'"
    echo ""

    # Serial device status
    echo -e "${BLUE}Serdev device:${NC}"
    ssh_cmd "cat /sys/bus/serial/devices/serial0-0/modalias 2>/dev/null" || echo "Not found"
    local driver=$(ssh_cmd "basename \$(readlink /sys/bus/serial/devices/serial0-0/driver 2>/dev/null) 2>/dev/null" || echo "none")
    echo "Driver: $driver"
    echo ""

    # Recent kernel messages
    echo -e "${BLUE}Recent Bluetooth kernel messages:${NC}"
    ssh_cmd "dmesg | grep -i -E 'bcsp|hci_uart|bluetooth' | tail -15"
    echo "----------------------------------------"
}

scan_devices() {
    info "Scanning for Bluetooth devices (10 seconds)..."

    # Enable scanning
    ssh_cmd "hciconfig hci0 piscan" || {
        warn "Could not enable scan mode"
    }

    # Run inquiry
    echo "Scanning..."
    ssh_cmd "timeout 10 hcitool scan 2>&1" || true

    # Show found devices
    echo ""
    info "Discovered devices:"
    ssh_cmd "hcitool dev"
}

monitor_mode() {
    info "Entering monitor mode (Ctrl+C to exit)..."
    echo "Watching kernel messages..."
    echo ""

    ssh_cmd "dmesg -w 2>/dev/null | grep --line-buffered -i -E 'bcsp|hci|bluetooth'" || {
        warn "Live dmesg not available, polling every 2 seconds..."
        while true; do
            ssh_cmd "dmesg | grep -i -E 'bcsp|hci|bluetooth' | tail -5"
            sleep 2
        done
    }
}

run_diagnostics() {
    info "Running Bluetooth diagnostics..."
    echo ""

    # Device tree info
    echo -e "${BLUE}Device tree Bluetooth node:${NC}"
    ssh_cmd "ls /sys/firmware/devicetree/base/soc/gsbi@16500000/serial@16540000/bluetooth/ 2>/dev/null" || echo "DT node not found"
    echo ""

    echo -e "${BLUE}Compatible string:${NC}"
    ssh_cmd "cat /sys/firmware/devicetree/base/soc/gsbi@16500000/serial@16540000/bluetooth/compatible 2>/dev/null" || echo "Not found"
    echo ""

    echo -e "${BLUE}Status:${NC}"
    ssh_cmd "cat /sys/firmware/devicetree/base/soc/gsbi@16500000/serial@16540000/bluetooth/status 2>/dev/null" || echo "Not found"
    echo ""

    # Serdev info
    echo -e "${BLUE}Serdev device modalias:${NC}"
    ssh_cmd "cat /sys/bus/serial/devices/serial0-0/modalias 2>/dev/null" || echo "Not found"
    echo ""

    echo -e "${BLUE}Available serdev drivers:${NC}"
    ssh_cmd "ls /sys/bus/serial/drivers/ 2>/dev/null" || echo "None"
    echo ""

    # GPIO states for BT pins (130=POWER, 131=WAKE, 138=RST)
    echo -e "${BLUE}Bluetooth GPIOs (130=POWER, 131=WAKE, 138=RST):${NC}"
    ssh_cmd "cat /sys/kernel/debug/gpio 2>/dev/null | grep -E 'gpio13[018]'" || echo "GPIO debug not available"
    echo ""

    # Probe messages
    echo -e "${BLUE}Probe messages:${NC}"
    ssh_cmd "dmesg | grep -E 'hci_uart_bcsp|serial0-0|BCSP serdev' | tail -10"
    echo ""

    # UART status
    echo -e "${BLUE}UART status:${NC}"
    ssh_cmd "cat /proc/tty/driver/msm_serial 2>/dev/null | head -5" || echo "UART info not available"
}

show_help() {
    head -24 "$0" | tail -22
    exit 0
}

# Main
main() {
    local deploy=1
    local test=1
    local monitor=0
    local scan=0
    local diag=0

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --deploy-only)
                test=0
                ;;
            --test-only)
                deploy=0
                ;;
            --monitor)
                monitor=1
                ;;
            --scan)
                scan=1
                ;;
            --diag)
                diag=1
                ;;
            --help|-h)
                show_help
                ;;
            *)
                error "Unknown option: $1"
                show_help
                ;;
        esac
        shift
    done

    echo "========================================"
    echo "  HP TouchPad Bluetooth Test Script"
    echo "        (Serdev BCSP Driver)"
    echo "========================================"
    echo ""

    check_device

    local kver=$(get_kernel_version)
    info "Device kernel: $kver"
    echo ""

    if [[ $diag -eq 1 ]]; then
        run_diagnostics
        exit 0
    fi

    if [[ $deploy -eq 1 ]]; then
        check_modules_exist
        deploy_modules
        echo ""

        unload_modules
        load_modules
        echo ""

        # Check serdev probe status
        check_serdev_probe
        echo ""
    fi

    if [[ $test -eq 1 ]]; then
        # With serdev, HCI device appears automatically after successful probe
        if wait_for_hci_device; then
            echo ""
            check_hci_device
            echo ""

            if bring_up_hci; then
                success "Bluetooth initialization successful!"
                echo ""
                check_bluetooth_status

                if [[ $scan -eq 1 ]]; then
                    echo ""
                    scan_devices
                fi
            else
                error "Failed to bring up HCI device"
                echo ""
                run_diagnostics
                exit 1
            fi
        else
            error "HCI device did not appear - serdev probe likely failed"
            echo ""
            run_diagnostics
            echo ""
            info "Check kernel messages for details:"
            ssh_cmd "dmesg | grep -i -E 'bcsp|hci_uart|serial0' | tail -20"
            exit 1
        fi
    fi

    if [[ $monitor -eq 1 ]]; then
        echo ""
        monitor_mode
    fi

    echo ""
    success "Done!"
}

main "$@"
