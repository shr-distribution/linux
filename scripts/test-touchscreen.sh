#!/bin/bash
# HP TouchPad Cypress CY8CTMA395 Touchscreen Test Script
#
# This script performs comprehensive testing of the touchscreen driver,
# checking hardware initialization, UART communication, and touch events.
#
# Usage: ./scripts/test-touchscreen.sh [--device IP] [--verbose]
#
# Options:
#   --device IP    Device IP address (default: 172.16.42.2)
#   --verbose      Enable verbose output
#   --help         Show this help
#
# Prerequisites:
#   - Device running LuneOS with the cy8ctma395-ts driver
#   - USB network connection to device
#   - nc (netcat) installed on host

set -e

# Default configuration
DEVICE_IP="172.16.42.2"
VERBOSE=false
TIMEOUT=10
PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --device)
            DEVICE_IP="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --help)
            head -20 "$0" | tail -15
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASS_COUNT++))
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAIL_COUNT++))
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    ((WARN_COUNT++))
}

log_verbose() {
    if [ "$VERBOSE" = true ]; then
        echo -e "${BLUE}[DEBUG]${NC} $1"
    fi
}

# Run command on device
run_on_device() {
    local cmd="$1"
    local result
    result=$(echo -e "$cmd\nexit" | timeout "$TIMEOUT" nc "$DEVICE_IP" 23 2>&1 | grep -v "^$" | tail -n +3 | head -n -1)
    echo "$result"
}

# Check device connectivity
check_connectivity() {
    log_info "Checking device connectivity at $DEVICE_IP..."
    if ping -c 1 -W 2 "$DEVICE_IP" &>/dev/null; then
        log_pass "Device is reachable"
        return 0
    else
        log_fail "Device is not reachable at $DEVICE_IP"
        return 1
    fi
}

# Test 1: Check driver is loaded
test_driver_loaded() {
    log_info "Test 1: Checking if cy8ctma395-ts driver is loaded..."

    local result
    result=$(run_on_device "dmesg | grep -c 'cy8ctma395-ts.*initialized' 2>/dev/null || echo 0")

    if [[ "$result" =~ [1-9] ]]; then
        log_pass "Driver is loaded and initialized"
        return 0
    else
        log_fail "Driver not found or not initialized"
        log_verbose "Checking dmesg for driver messages..."
        run_on_device "dmesg | grep -i cy8ctma395 | tail -5"
        return 1
    fi
}

# Test 2: Check input device registered
test_input_device() {
    log_info "Test 2: Checking input device registration..."

    local result
    result=$(run_on_device "cat /proc/bus/input/devices | grep -A5 'cy8ctma395'")

    if echo "$result" | grep -q "HP TouchPad Touchscreen"; then
        log_pass "Input device registered as 'HP TouchPad Touchscreen'"

        # Extract event device
        local event_dev
        event_dev=$(echo "$result" | grep "Handlers=" | grep -o "event[0-9]*")
        log_verbose "Event device: /dev/input/$event_dev"
        return 0
    else
        log_fail "Input device not registered"
        return 1
    fi
}

# Test 3: Check GPIO configuration
test_gpio_config() {
    log_info "Test 3: Checking GPIO configuration..."

    local gpio_status
    gpio_status=$(run_on_device "cat /sys/kernel/debug/gpio 2>/dev/null || mount -t debugfs debugfs /sys/kernel/debug && cat /sys/kernel/debug/gpio")

    # Check reset GPIO (gpio70)
    if echo "$gpio_status" | grep -q "gpio70.*out.*high"; then
        log_pass "Reset GPIO (70) configured correctly (out, high = deasserted)"
    else
        local gpio70_state
        gpio70_state=$(echo "$gpio_status" | grep "gpio70")
        log_warn "Reset GPIO (70) state: $gpio70_state"
    fi

    # Check wake GPIO (gpio123)
    if echo "$gpio_status" | grep -q "gpio123.*out.*high"; then
        log_pass "Wake GPIO (123) configured correctly (out, high = streaming)"
    else
        local gpio123_state
        gpio123_state=$(echo "$gpio_status" | grep "gpio123")
        log_warn "Wake GPIO (123) state: $gpio123_state"
    fi
}

# Test 4: Check regulator status
test_regulator() {
    log_info "Test 4: Checking VDD regulator (L10)..."

    local reg_status
    reg_status=$(run_on_device "cat /sys/kernel/debug/regulator/regulator_summary 2>/dev/null | grep -A1 'l10'")

    if echo "$reg_status" | grep -q "serial1-0-vdd"; then
        log_pass "VDD regulator (L10) is enabled for touchscreen"
        log_verbose "$reg_status"
    else
        log_warn "Could not verify VDD regulator status"
    fi
}

# Test 5: Check I2C initialization
test_i2c_init() {
    log_info "Test 5: Checking I2C initialization..."

    local dmesg_i2c
    dmesg_i2c=$(run_on_device "dmesg | grep -i 'cy8ctma395.*I2C'")

    if echo "$dmesg_i2c" | grep -q "ret=1"; then
        log_pass "I2C initialization commands succeeded"
    elif echo "$dmesg_i2c" | grep -q "ret=-"; then
        log_fail "I2C initialization failed"
        log_verbose "$dmesg_i2c"
    else
        log_warn "Could not verify I2C initialization (no debug output)"
    fi
}

# Test 6: Check UART baud rate
test_uart_baud() {
    log_info "Test 6: Checking UART baud rate configuration..."

    local baud_info
    baud_info=$(run_on_device "dmesg | grep -i 'baud'")

    if echo "$baud_info" | grep -q "actual baud 4000000"; then
        log_pass "UART baud rate set to 4000000 (4 Mbps)"
    elif echo "$baud_info" | grep -q "actual baud"; then
        local actual
        actual=$(echo "$baud_info" | grep -o "actual baud [0-9]*" | head -1)
        log_warn "UART baud rate: $actual (expected 4000000)"
    else
        log_warn "Could not verify UART baud rate (no debug output)"
    fi
}

# Test 7: Check UART data reception
test_uart_data() {
    log_info "Test 7: Checking UART data reception..."

    local irq_count
    irq_count=$(run_on_device "cat /proc/interrupts | grep msm_serial2 | awk '{print \$2}'")

    if [ -z "$irq_count" ]; then
        log_fail "Could not find msm_serial2 interrupt"
        return 1
    fi

    log_verbose "Current interrupt count: $irq_count"

    # Wait 2 seconds and check if count increases
    sleep 2
    local irq_count2
    irq_count2=$(run_on_device "cat /proc/interrupts | grep msm_serial2 | awk '{print \$2}'")

    if [ "$irq_count2" -gt "$irq_count" ]; then
        local diff=$((irq_count2 - irq_count))
        log_pass "UART receiving data ($diff interrupts in 2 seconds)"
    else
        log_warn "No UART data received (interrupt count unchanged: $irq_count)"
        log_verbose "This may indicate the touchscreen is not streaming data"
    fi
}

# Test 8: Check for driver errors
test_driver_errors() {
    log_info "Test 8: Checking for driver errors..."

    local errors
    errors=$(run_on_device "dmesg | grep -i 'cy8ctma395' | grep -iE 'error|fail|warn' | grep -v kobject")

    if [ -z "$errors" ]; then
        log_pass "No driver errors found"
    else
        log_warn "Driver warnings/errors found:"
        echo "$errors" | while read -r line; do
            log_verbose "  $line"
        done
    fi
}

# Test 9: Check serdev binding
test_serdev_binding() {
    log_info "Test 9: Checking serdev binding..."

    local binding
    binding=$(run_on_device "cat /sys/bus/serial/devices/serial1-0/uevent 2>/dev/null")

    if echo "$binding" | grep -q "DRIVER=cy8ctma395-ts"; then
        log_pass "serdev device bound to cy8ctma395-ts driver"
    else
        log_fail "serdev device not bound correctly"
        log_verbose "$binding"
    fi
}

# Test 10: Check for UART RX data (if debug enabled)
test_uart_rx_stats() {
    log_info "Test 10: Checking UART RX statistics..."

    local rx_stats
    rx_stats=$(run_on_device "dmesg | grep 'UART RX' | tail -1")

    if echo "$rx_stats" | grep -q "bytes total"; then
        log_pass "UART receiving data: $rx_stats"
    else
        log_warn "No UART RX statistics available (driver debug may not be enabled)"
    fi
}

# Test 11: Check input event capabilities
test_input_capabilities() {
    log_info "Test 11: Checking input device capabilities..."

    local caps
    caps=$(run_on_device "cat /proc/bus/input/devices | grep -A10 'cy8ctma395'")

    # Check for multitouch support (ABS_MT_*)
    if echo "$caps" | grep -q "B: ABS="; then
        log_pass "Input device has ABS (absolute) capabilities"

        # Check for KEY events (BTN_TOUCH)
        if echo "$caps" | grep -q "B: KEY="; then
            log_pass "Input device has KEY capabilities (BTN_TOUCH)"
        fi
    else
        log_fail "Input device missing required capabilities"
    fi
}

# Test 12: Interactive touch test (optional)
test_touch_interactive() {
    log_info "Test 12: Interactive touch test..."
    echo ""
    echo "This test requires physical interaction with the touchscreen."
    echo "Touch the screen and watch for events in dmesg."
    echo ""

    # Show real-time dmesg for touch events
    log_info "Monitoring for touch events (5 seconds)..."
    run_on_device "dmesg -w 2>/dev/null &; sleep 5; killall dmesg 2>/dev/null" || true
}

# Summary
print_summary() {
    echo ""
    echo "========================================"
    echo "       TOUCHSCREEN TEST SUMMARY"
    echo "========================================"
    echo -e "  ${GREEN}PASSED${NC}: $PASS_COUNT"
    echo -e "  ${RED}FAILED${NC}: $FAIL_COUNT"
    echo -e "  ${YELLOW}WARNINGS${NC}: $WARN_COUNT"
    echo "========================================"

    if [ $FAIL_COUNT -gt 0 ]; then
        echo ""
        echo "Some tests failed. Check the output above for details."
        echo ""
        echo "Common issues:"
        echo "  - UART not receiving: Check baud rate, GPIO wake state"
        echo "  - I2C failures: Check regulator, reset GPIO timing"
        echo "  - No input events: Verify driver probe succeeded"
        return 1
    elif [ $WARN_COUNT -gt 0 ]; then
        echo ""
        echo "Tests passed with warnings. Touchscreen may work but needs verification."
        return 0
    else
        echo ""
        echo "All tests passed! Touchscreen should be functional."
        return 0
    fi
}

# Main test sequence
main() {
    echo "========================================"
    echo "  HP TouchPad Touchscreen Test Suite"
    echo "    Cypress CY8CTMA395 Driver Tests"
    echo "========================================"
    echo ""

    # Check connectivity first
    if ! check_connectivity; then
        echo ""
        echo "Cannot reach device. Ensure USB network is configured:"
        echo "  sudo nmcli dev set usb0 managed no"
        echo "  sudo ip addr add 172.16.42.1/24 dev usb0"
        exit 1
    fi

    echo ""

    # Run all tests
    test_driver_loaded
    test_input_device
    test_serdev_binding
    test_gpio_config
    test_regulator
    test_i2c_init
    test_uart_baud
    test_uart_data
    test_uart_rx_stats
    test_driver_errors
    test_input_capabilities

    # Print summary
    print_summary
}

# Run main
main "$@"
