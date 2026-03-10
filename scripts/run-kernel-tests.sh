#!/bin/sh
# HP TouchPad Kernel Test Script
# Runs automated tests and collects results
# Usage: Run via telnet to 172.16.42.2

RESULTS_FILE="/tmp/kernel_test_results.txt"
DMESG_FILE="/tmp/dmesg_full.txt"

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

passed=0
failed=0
skipped=0

log() {
    echo "$1" | tee -a "$RESULTS_FILE"
}

test_pass() {
    passed=$((passed + 1))
    log "[PASS] $1"
}

test_fail() {
    failed=$((failed + 1))
    log "[FAIL] $1"
}

test_skip() {
    skipped=$((skipped + 1))
    log "[SKIP] $1"
}

# Initialize results file
echo "=== HP TouchPad Kernel Test Results ===" > "$RESULTS_FILE"
echo "Date: $(date)" >> "$RESULTS_FILE"
echo "Kernel: $(uname -r)" >> "$RESULTS_FILE"
echo "" >> "$RESULTS_FILE"

# Save full dmesg
dmesg > "$DMESG_FILE"

log "=== 1. BOOT AND BASIC SYSTEM ==="

# Test 1.1: Kernel version
if uname -r | grep -q "6.18"; then
    test_pass "Kernel version is 6.18.x: $(uname -r)"
else
    test_fail "Kernel version: $(uname -r)"
fi

# Test 1.2: Device tree model
MODEL=$(cat /proc/device-tree/model 2>/dev/null)
if echo "$MODEL" | grep -q "TouchPad"; then
    test_pass "Device tree model: $MODEL"
else
    test_fail "Device tree model not detected"
fi

# Test 1.3: CPU detection
CPUS=$(grep -c "^processor" /proc/cpuinfo)
if [ "$CPUS" = "2" ]; then
    test_pass "Dual CPU cores detected"
else
    test_fail "Expected 2 CPUs, found $CPUS"
fi

# Test 1.4: CPU type
CPU_TYPE=$(grep "CPU part" /proc/cpuinfo | head -1 | awk '{print $4}')
if [ "$CPU_TYPE" = "0x02d" ]; then
    test_pass "CPU type: Scorpion (0x02d)"
else
    test_fail "CPU type: $CPU_TYPE (expected Scorpion 0x02d)"
fi

# Test 1.5: Memory
MEM_TOTAL=$(grep MemTotal /proc/meminfo | awk '{print $2}')
MEM_MB=$((MEM_TOTAL / 1024))
if [ "$MEM_MB" -gt 800 ]; then
    test_pass "Memory: ${MEM_MB}MB detected"
else
    test_fail "Memory: ${MEM_MB}MB (expected >800MB)"
fi

# Test 1.6: Check for kernel errors
ERRORS=$(grep -ci "error" "$DMESG_FILE" || echo "0")
CRITICAL=$(grep -Ei "panic|oops|bug" "$DMESG_FILE" | wc -l)
if [ "$CRITICAL" = "0" ]; then
    test_pass "No critical kernel errors (panic/oops/bug)"
else
    test_fail "Found $CRITICAL critical errors"
fi
log "  Note: $ERRORS total error messages in dmesg"

log ""
log "=== 2. DRIVER LOADING ==="

# Test 2.1: Pinctrl
if grep -q "800000.pinctrl returned 0" "$DMESG_FILE"; then
    test_pass "Pinctrl driver loaded"
else
    test_fail "Pinctrl driver"
fi

# Test 2.2: Clock controller
if grep -q "clock-controller returned 0" "$DMESG_FILE"; then
    test_pass "Clock controller loaded"
else
    test_fail "Clock controller"
fi

# Test 2.3: RPM
if grep -q "rpm returned 0" "$DMESG_FILE"; then
    test_pass "RPM driver loaded"
else
    test_fail "RPM driver"
fi

# Test 2.4: I2C buses
I2C_BUSES=$(ls -d /sys/bus/i2c/devices/i2c-* 2>/dev/null | wc -l)
if [ "$I2C_BUSES" -gt 0 ]; then
    test_pass "I2C buses detected: $I2C_BUSES"
else
    test_fail "No I2C buses found"
fi

# Test 2.5: USB gadget
if ls /sys/class/udc/ 2>/dev/null | grep -q .; then
    UDC=$(ls /sys/class/udc/)
    test_pass "USB UDC present: $UDC"
else
    test_fail "USB UDC not found"
fi

# Test 2.6: USB gadget ethernet
if grep -q "g_ether" "$DMESG_FILE"; then
    test_pass "USB Ethernet gadget active"
else
    test_fail "USB Ethernet gadget"
fi

log ""
log "=== 3. DISPLAY SUBSYSTEM ==="

# Test 3.1: DRM device
if ls /sys/class/drm/card0 2>/dev/null; then
    test_pass "DRM card0 present"
else
    test_fail "DRM card0 not found"
fi

# Test 3.2: Framebuffer
if ls /dev/fb0 2>/dev/null; then
    test_pass "Framebuffer /dev/fb0 present"
else
    test_skip "Framebuffer not found (may use DRM only)"
fi

# Test 3.3: Backlight
BL_DEV=$(ls /sys/class/backlight/ 2>/dev/null | head -1)
if [ -n "$BL_DEV" ]; then
    test_pass "Backlight device: $BL_DEV"
else
    test_skip "Backlight device not found"
fi

log ""
log "=== 4. INPUT DEVICES ==="

# Test 4.1: Input devices
INPUT_COUNT=$(ls /dev/input/event* 2>/dev/null | wc -l)
if [ "$INPUT_COUNT" -gt 0 ]; then
    test_pass "Input devices found: $INPUT_COUNT"
else
    test_fail "No input devices"
fi

# Test 4.2: Touchscreen
if grep -qi "touch" /proc/bus/input/devices 2>/dev/null; then
    test_pass "Touchscreen input device detected"
else
    test_skip "Touchscreen not detected"
fi

log ""
log "=== 5. STORAGE ==="

# Test 5.1: eMMC
if ls /dev/mmcblk0 2>/dev/null; then
    test_pass "eMMC device present (mmcblk0)"
else
    test_fail "eMMC not found"
fi

# Test 5.2: Partitions
PARTS=$(ls /dev/mmcblk0p* 2>/dev/null | wc -l)
if [ "$PARTS" -gt 10 ]; then
    test_pass "eMMC partitions found: $PARTS"
else
    test_fail "Expected >10 partitions, found $PARTS"
fi

log ""
log "=== 6. NETWORK ==="

# Test 6.1: USB network interface
USB_NET=$(ip link show | grep -E "usb|eth" | head -1 | awk -F: '{print $2}' | tr -d ' ')
if [ -n "$USB_NET" ]; then
    test_pass "USB network interface: $USB_NET"
else
    test_fail "USB network interface not found"
fi

# Test 6.2: IP configured
IP_ADDR=$(ip addr show | grep "172.16.42.2" | head -1)
if [ -n "$IP_ADDR" ]; then
    test_pass "IP address configured: 172.16.42.2"
else
    test_fail "IP address not configured"
fi

log ""
log "=== 7. POWER SUPPLY ==="

# Test 7.1: Power supply devices
PS_DEVICES=$(ls /sys/class/power_supply/ 2>/dev/null)
if [ -n "$PS_DEVICES" ]; then
    test_pass "Power supply devices: $PS_DEVICES"
else
    test_skip "No power supply devices (A6 driver may not be loaded)"
fi

log ""
log "=== 8. SENSORS (IIO) ==="

# Test 8.1: IIO devices
IIO_COUNT=$(ls -d /sys/bus/iio/devices/iio:device* 2>/dev/null | wc -l)
if [ "$IIO_COUNT" -gt 0 ]; then
    test_pass "IIO devices found: $IIO_COUNT"
    for dev in /sys/bus/iio/devices/iio:device*/name; do
        if [ -f "$dev" ]; then
            log "  - $(cat $dev)"
        fi
    done
else
    test_skip "No IIO devices (sensor drivers may not be loaded)"
fi

log ""
log "=== 9. REGULATORS ==="

# Test 9.1: Regulator devices
REG_COUNT=$(ls /sys/class/regulator/ 2>/dev/null | wc -l)
if [ "$REG_COUNT" -gt 10 ]; then
    test_pass "Regulators found: $REG_COUNT"
else
    test_fail "Expected >10 regulators, found $REG_COUNT"
fi

log ""
log "=== 10. LEDS ==="

# Test 10.1: LED devices
LED_DEVICES=$(ls /sys/class/leds/ 2>/dev/null)
if [ -n "$LED_DEVICES" ]; then
    test_pass "LED devices: $LED_DEVICES"
else
    test_skip "No LED devices found"
fi

log ""
log "=== 11. THERMAL ==="

# Test 11.1: Thermal zones
THERMAL_ZONES=$(ls -d /sys/class/thermal/thermal_zone* 2>/dev/null | wc -l)
if [ "$THERMAL_ZONES" -gt 0 ]; then
    test_pass "Thermal zones found: $THERMAL_ZONES"
    for tz in /sys/class/thermal/thermal_zone*/type; do
        if [ -f "$tz" ]; then
            ZONE_DIR=$(dirname "$tz")
            TYPE=$(cat "$tz")
            TEMP=$(cat "$ZONE_DIR/temp" 2>/dev/null || echo "N/A")
            log "  - $TYPE: ${TEMP}mC"
        fi
    done
else
    test_skip "No thermal zones"
fi

log ""
log "=== 12. ERROR SUMMARY ==="

# Summarize errors from dmesg
log "Errors in dmesg:"
grep -i "error" "$DMESG_FILE" | head -n 20 | while read line; do
    log "  $line"
done

log ""
log "Warnings in dmesg:"
grep -i "warn" "$DMESG_FILE" | grep -v "initcall" | head -n 10 | while read line; do
    log "  $line"
done

log ""
log "=== TEST SUMMARY ==="
log "Passed:  $passed"
log "Failed:  $failed"
log "Skipped: $skipped"
log "Total:   $((passed + failed + skipped))"
log ""
log "Full results saved to: $RESULTS_FILE"
log "Full dmesg saved to: $DMESG_FILE"
log ""

# Print summary
echo ""
echo "==================================="
echo "  PASSED:  $passed"
echo "  FAILED:  $failed"
echo "  SKIPPED: $skipped"
echo "==================================="
echo ""
echo "To view full results: cat $RESULTS_FILE"
