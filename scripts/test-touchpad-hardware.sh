#!/bin/bash
# HP TouchPad Hardware Test Script
#
# This script:
# 1. Waits for the USB gadget to enumerate
# 2. Configures host network (requires sudo)
# 3. Connects to the device and runs comprehensive hardware tests
# 4. Generates a detailed report with test results
#
# Usage: ./scripts/test-touchpad-hardware.sh [report-name]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"
REPORT_DIR="$KERNEL_DIR/reports"
BUILD_OUTPUT="$PARENT_DIR/build-output"
ARCHIVE_DIR="$BUILD_OUTPUT/archive"
REPORT_NAME="${1:-hardware-test-$(date +%Y%m%d-%H%M%S)}"
REPORT_FILE="$REPORT_DIR/${REPORT_NAME}.md"

DEVICE_IP="172.16.42.2"
HOST_IP="172.16.42.1"
NETMASK="16"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

mkdir -p "$REPORT_DIR"

# Expected hardware for HP TouchPad
declare -A EXPECTED
EXPECTED[cpu_cores]="2"
EXPECTED[cpu_type]="Scorpion"
EXPECTED[ram_mb]="1024"
EXPECTED[emmc_device]="mmcblk0"
EXPECTED[display_width]="1024"
EXPECTED[display_height]="768"
EXPECTED[touchscreen]="cy8ctma395"
EXPECTED[backlight]="lm3528"
EXPECTED[wifi_chip]="ath6kl"
EXPECTED[bluetooth]="hci0"
EXPECTED[audio_codec]="wm8994"
EXPECTED[accelerometer]="mpu3050"
EXPECTED[led_driver]="lm8502"
EXPECTED[battery]="bq27541"
EXPECTED[usb_gadget]="ci_hdrc"

# Function to print status
status() {
    echo -e "${GREEN}[*]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[!]${NC} $1"
}

error() {
    echo -e "${RED}[X]${NC} $1"
}

info() {
    echo -e "${BLUE}[i]${NC} $1"
}

# Function to run command on device via netcat
run_on_device() {
    local cmd="$1"
    local timeout="${2:-5}"
    echo "$cmd" | timeout "$timeout" nc -q 1 "$DEVICE_IP" 23 2>/dev/null | tail -n +4 | head -n -1
}

# Function to run command and capture full output
run_on_device_full() {
    local cmd="$1"
    local timeout="${2:-10}"
    (echo "$cmd"; sleep 1) | timeout "$timeout" nc "$DEVICE_IP" 23 2>/dev/null
}

# Wait for USB gadget
wait_for_device() {
    status "Waiting for USB gadget (0525:a4a2)..."
    local count=0
    local max_wait=60

    while [ $count -lt $max_wait ]; do
        if lsusb 2>/dev/null | grep -qE "0525:a4a2"; then
            echo ""
            status "USB gadget detected after ${count}s"
            sleep 2
            return 0
        fi
        sleep 1
        count=$((count + 1))
        printf "."
    done

    echo ""
    error "Timeout waiting for USB gadget"
    return 1
}

# Find and configure network interface
configure_network() {
    status "Looking for USB network interface..."

    # Find the interface
    local iface=""
    for i in {1..10}; do
        iface=$(ip -o link show 2>/dev/null | grep -oP 'enx[a-f0-9]+' | head -1)
        [ -n "$iface" ] && break
        sleep 1
    done

    if [ -z "$iface" ]; then
        error "No USB network interface found"
        return 1
    fi

    status "Found interface: $iface"

    # Check if already configured
    if ip addr show "$iface" 2>/dev/null | grep -q "$HOST_IP"; then
        status "Interface already configured"
    else
        status "Configuring interface (requires sudo)..."
        sudo ip addr add "$HOST_IP/$NETMASK" dev "$iface" 2>/dev/null || true
        sudo ip link set "$iface" up
    fi

    # Wait for link
    status "Waiting for link..."
    for i in {1..10}; do
        if ip link show "$iface" 2>/dev/null | grep -q "LOWER_UP"; then
            status "Link is up"
            break
        fi
        sleep 1
    done

    # Test connectivity
    status "Testing connectivity..."
    if ping -c 2 -W 2 "$DEVICE_IP" > /dev/null 2>&1; then
        status "Device is reachable at $DEVICE_IP"
        return 0
    else
        error "Cannot reach device at $DEVICE_IP"
        return 1
    fi
}

# Initialize report
init_report() {
    cat > "$REPORT_FILE" << EOF
# HP TouchPad Hardware Test Report

**Date:** $(date '+%Y-%m-%d %H:%M:%S')
**Host:** $(hostname)
**Test Script Version:** 2.0

---

## Test Environment

| Property | Value |
|----------|-------|
| Device IP | $DEVICE_IP |
| Host IP | $HOST_IP |
| Report File | \`${REPORT_NAME}.md\` |

---

EOF
}

# Add section to report
add_section() {
    local title="$1"
    echo "" >> "$REPORT_FILE"
    echo "## $title" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
}

# Add subsection to report
add_subsection() {
    local title="$1"
    echo "" >> "$REPORT_FILE"
    echo "### $title" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
}

# Add detailed test result to report
add_test_detailed() {
    local name="$1"
    local status="$2"
    local expected="$3"
    local actual="$4"
    local details="$5"
    local notes="$6"

    local icon="❓"
    local status_color=""
    case "$status" in
        PASS) icon="✅"; status_color="**PASS**" ;;
        FAIL) icon="❌"; status_color="**FAIL**" ;;
        PARTIAL) icon="⚠️"; status_color="**PARTIAL**" ;;
        SKIP) icon="⏭️"; status_color="**SKIP**" ;;
        INFO) icon="ℹ️"; status_color="**INFO**" ;;
    esac

    echo "### $icon $name" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
    echo "| Property | Value |" >> "$REPORT_FILE"
    echo "|----------|-------|" >> "$REPORT_FILE"
    echo "| **Status** | $status_color |" >> "$REPORT_FILE"

    if [ -n "$expected" ]; then
        echo "| **Expected** | $expected |" >> "$REPORT_FILE"
    fi
    if [ -n "$actual" ]; then
        echo "| **Actual** | $actual |" >> "$REPORT_FILE"
    fi
    echo "" >> "$REPORT_FILE"

    if [ -n "$notes" ]; then
        echo "> $notes" >> "$REPORT_FILE"
        echo "" >> "$REPORT_FILE"
    fi

    if [ -n "$details" ]; then
        echo "<details>" >> "$REPORT_FILE"
        echo "<summary>Raw output (click to expand)</summary>" >> "$REPORT_FILE"
        echo "" >> "$REPORT_FILE"
        echo '```' >> "$REPORT_FILE"
        echo "$details" >> "$REPORT_FILE"
        echo '```' >> "$REPORT_FILE"
        echo "</details>" >> "$REPORT_FILE"
    fi
    echo "" >> "$REPORT_FILE"
}

# Simple test result (backward compatible)
add_test() {
    local name="$1"
    local status="$2"
    local details="$3"
    add_test_detailed "$name" "$status" "" "" "$details" ""
}

# Run all hardware tests
run_tests() {
    local pass=0
    local fail=0
    local partial=0

    #==========================================================================
    add_section "System Information"
    #==========================================================================

    # Kernel version
    status "Testing: Kernel version..."
    local kernel=$(run_on_device "uname -a")
    local kernel_ver=$(echo "$kernel" | grep -oP 'Linux \S+ \S+' | head -1)
    if echo "$kernel" | grep -q "Linux"; then
        add_test_detailed "Kernel Version" "PASS" \
            "Linux 6.18.x for ARM" \
            "$kernel_ver" \
            "$kernel" \
            "Kernel booted successfully"
        ((pass++))
    else
        add_test_detailed "Kernel Version" "FAIL" \
            "Linux 6.18.x" \
            "Could not retrieve" \
            "$kernel" \
            "Unable to get kernel version - connection issue?"
        ((fail++))
    fi

    # CPU info
    status "Testing: CPU..."
    local cpu_full=$(run_on_device "cat /proc/cpuinfo")
    local cpu_model=$(echo "$cpu_full" | grep -i "model name\|Processor\|Hardware" | head -3)
    local cpu_cores=$(echo "$cpu_full" | grep -c "^processor" || echo "0")
    local cpu_bogomips=$(echo "$cpu_full" | grep -i "bogomips" | head -1 | awk '{print $3}')

    if [ "$cpu_cores" -ge 2 ]; then
        add_test_detailed "CPU" "PASS" \
            "${EXPECTED[cpu_cores]} cores (${EXPECTED[cpu_type]})" \
            "$cpu_cores cores, $cpu_bogomips BogoMIPS" \
            "$cpu_model" \
            "Dual-core Qualcomm Scorpion detected"
        ((pass++))
    elif [ "$cpu_cores" -ge 1 ]; then
        add_test_detailed "CPU" "PARTIAL" \
            "${EXPECTED[cpu_cores]} cores" \
            "$cpu_cores core(s) detected" \
            "$cpu_full" \
            "Expected 2 cores but found $cpu_cores - SMP may not be enabled"
        ((partial++))
    else
        add_test_detailed "CPU" "FAIL" \
            "${EXPECTED[cpu_cores]} cores" \
            "Could not detect" \
            "$cpu_full" \
            "Unable to read CPU information"
        ((fail++))
    fi

    # Memory
    status "Testing: Memory..."
    local mem_full=$(run_on_device "cat /proc/meminfo")
    local mem_total_kb=$(echo "$mem_full" | grep "MemTotal" | awk '{print $2}')
    local mem_total_mb=$((mem_total_kb / 1024))
    local mem_free_kb=$(echo "$mem_full" | grep "MemFree" | awk '{print $2}')
    local mem_free_mb=$((mem_free_kb / 1024))
    local mem_avail_kb=$(echo "$mem_full" | grep "MemAvailable" | awk '{print $2}')
    local mem_avail_mb=$((mem_avail_kb / 1024))

    if [ "$mem_total_mb" -ge 900 ]; then
        add_test_detailed "Memory" "PASS" \
            "${EXPECTED[ram_mb]} MB" \
            "${mem_total_mb} MB total, ${mem_avail_mb} MB available" \
            "$(echo "$mem_full" | head -10)" \
            "1GB RAM detected as expected"
        ((pass++))
    elif [ "$mem_total_mb" -ge 100 ]; then
        add_test_detailed "Memory" "PARTIAL" \
            "${EXPECTED[ram_mb]} MB" \
            "${mem_total_mb} MB detected" \
            "$(echo "$mem_full" | head -10)" \
            "Less memory than expected - kernel memory reservation?"
        ((partial++))
    else
        add_test_detailed "Memory" "FAIL" \
            "${EXPECTED[ram_mb]} MB" \
            "Could not detect" \
            "$mem_full" \
            "Unable to read memory information"
        ((fail++))
    fi

    #==========================================================================
    add_section "Storage"
    #==========================================================================

    # eMMC device
    status "Testing: eMMC..."
    local emmc_devices=$(run_on_device "ls -la /dev/mmcblk* 2>&1" 10)
    local emmc_size=$(run_on_device "cat /sys/block/mmcblk0/size 2>/dev/null" 5)
    local emmc_size_gb=""
    if [ -n "$emmc_size" ] && [ "$emmc_size" -gt 0 ] 2>/dev/null; then
        emmc_size_gb="$((emmc_size * 512 / 1024 / 1024 / 1024)) GB"
    fi

    if echo "$emmc_devices" | grep -q "mmcblk0"; then
        add_test_detailed "eMMC Device" "PASS" \
            "/dev/${EXPECTED[emmc_device]}" \
            "/dev/mmcblk0 ($emmc_size_gb)" \
            "$emmc_devices" \
            "eMMC storage detected and accessible"
        ((pass++))
    else
        add_test_detailed "eMMC Device" "FAIL" \
            "/dev/${EXPECTED[emmc_device]}" \
            "Not found" \
            "$emmc_devices" \
            "eMMC not detected - check MMC driver and device tree"
        ((fail++))
    fi

    # eMMC partitions
    status "Testing: eMMC partitions..."
    local parts=$(run_on_device "cat /proc/partitions | grep mmcblk")
    local part_count=$(echo "$parts" | grep -c "mmcblk0p" || echo "0")

    if [ "$part_count" -ge 10 ]; then
        add_test_detailed "eMMC Partitions" "PASS" \
            "Multiple partitions (webOS layout)" \
            "$part_count partitions found" \
            "$parts" \
            "Standard webOS partition layout detected"
        ((pass++))
    elif [ "$part_count" -ge 1 ]; then
        add_test_detailed "eMMC Partitions" "PARTIAL" \
            "Multiple partitions" \
            "$part_count partition(s)" \
            "$parts" \
            "Fewer partitions than expected"
        ((partial++))
    else
        add_test_detailed "eMMC Partitions" "FAIL" \
            "Partitions" \
            "None found" \
            "$parts" \
            "No partitions detected"
        ((fail++))
    fi

    #==========================================================================
    add_section "Display Subsystem"
    #==========================================================================

    # Framebuffer
    status "Testing: Framebuffer..."
    local fb_dev=$(run_on_device "ls -la /dev/fb* 2>&1")
    local fb_name=$(run_on_device "cat /sys/class/graphics/fb0/name 2>&1")
    local fb_modes=$(run_on_device "cat /sys/class/graphics/fb0/modes 2>&1")
    local fb_size=$(run_on_device "cat /sys/class/graphics/fb0/virtual_size 2>&1")
    local fb_bpp=$(run_on_device "cat /sys/class/graphics/fb0/bits_per_pixel 2>&1")

    if echo "$fb_dev" | grep -q "/dev/fb0"; then
        local resolution="${fb_size:-unknown}"
        add_test_detailed "Framebuffer" "PASS" \
            "${EXPECTED[display_width]}x${EXPECTED[display_height]}" \
            "$resolution @ ${fb_bpp}bpp" \
            "Device: $fb_dev
Name: $fb_name
Modes: $fb_modes
Size: $fb_size
BPP: $fb_bpp" \
            "Framebuffer device available - display driver working"
        ((pass++))
    else
        add_test_detailed "Framebuffer" "FAIL" \
            "/dev/fb0" \
            "Not found" \
            "$fb_dev" \
            "No framebuffer device - check display driver (mdp4/mdss)"
        ((fail++))
    fi

    # Backlight
    status "Testing: Backlight..."
    local bl_devices=$(run_on_device "ls /sys/class/backlight/ 2>&1")
    local bl_brightness=$(run_on_device "cat /sys/class/backlight/*/brightness 2>&1" | head -1)
    local bl_max=$(run_on_device "cat /sys/class/backlight/*/max_brightness 2>&1" | head -1)
    local bl_driver=$(run_on_device "basename \$(readlink /sys/class/backlight/*/device/driver 2>/dev/null) 2>/dev/null" | head -1)

    if [ -n "$bl_brightness" ] && [ "$bl_brightness" != "" ]; then
        add_test_detailed "Backlight Control" "PASS" \
            "${EXPECTED[backlight]} driver" \
            "Brightness: $bl_brightness / $bl_max" \
            "Devices: $bl_devices
Current brightness: $bl_brightness
Max brightness: $bl_max
Driver: $bl_driver" \
            "Backlight controllable via sysfs"
        ((pass++))
    else
        add_test_detailed "Backlight Control" "FAIL" \
            "${EXPECTED[backlight]} driver" \
            "No backlight control found" \
            "$bl_devices" \
            "Backlight driver not loaded or not working"
        ((fail++))
    fi

    #==========================================================================
    add_section "Input Devices"
    #==========================================================================

    # Touchscreen
    status "Testing: Touchscreen..."
    local input_devices=$(run_on_device "cat /proc/bus/input/devices" 10)
    local ts_info=$(echo "$input_devices" | grep -A10 -i "touch")
    local ts_name=$(echo "$ts_info" | grep "Name=" | sed 's/.*Name="\([^"]*\)".*/\1/')
    local ts_events=$(run_on_device "ls /dev/input/event* 2>&1")
    local ts_i2c=$(run_on_device "ls /sys/bus/i2c/devices/*/name 2>&1 | xargs -I{} sh -c 'echo \"{}: \$(cat {})\"' | grep -i touch")

    if echo "$input_devices" | grep -qi "touch"; then
        add_test_detailed "Touchscreen" "PASS" \
            "${EXPECTED[touchscreen]}" \
            "$ts_name" \
            "$ts_info" \
            "Touchscreen registered as input device"
        ((pass++))
    else
        add_test_detailed "Touchscreen" "FAIL" \
            "${EXPECTED[touchscreen]}" \
            "Not detected" \
            "Input devices:
$input_devices

I2C devices:
$ts_i2c" \
            "Touchscreen not registered - check I2C driver and device tree"
        ((fail++))
    fi

    # Power button and other buttons
    status "Testing: Buttons..."
    local buttons=$(echo "$input_devices" | grep -B2 -A5 -iE "button|gpio-keys|power")
    local button_names=$(echo "$buttons" | grep "Name=" | sed 's/.*Name="\([^"]*\)".*/\1/' | tr '\n' ', ')

    if [ -n "$button_names" ]; then
        add_test_detailed "Hardware Buttons" "PASS" \
            "Power, Volume buttons" \
            "$button_names" \
            "$buttons" \
            "GPIO-based buttons registered"
        ((pass++))
    else
        add_test_detailed "Hardware Buttons" "PARTIAL" \
            "Power, Volume buttons" \
            "Not found as input devices" \
            "$input_devices" \
            "Buttons may use different input method"
        ((partial++))
    fi

    #==========================================================================
    add_section "I2C Bus"
    #==========================================================================

    status "Testing: I2C buses..."
    local i2c_devs=$(run_on_device "ls -la /dev/i2c-* 2>&1")
    local i2c_count=$(echo "$i2c_devs" | grep -c "i2c-" || echo "0")
    local i2c_devices=$(run_on_device "ls /sys/bus/i2c/devices/ 2>&1")
    local i2c_drivers=$(run_on_device "ls /sys/bus/i2c/drivers/ 2>&1" | head -20)

    if [ "$i2c_count" -ge 1 ]; then
        add_test_detailed "I2C Buses" "PASS" \
            "Multiple I2C buses" \
            "$i2c_count I2C bus(es) available" \
            "Buses: $i2c_devs

Devices:
$i2c_devices

Loaded drivers:
$i2c_drivers" \
            "I2C subsystem operational"
        ((pass++))
    else
        add_test_detailed "I2C Buses" "FAIL" \
            "I2C buses" \
            "None found" \
            "$i2c_devs" \
            "I2C controller driver may not be loaded"
        ((fail++))
    fi

    #==========================================================================
    add_section "LEDs"
    #==========================================================================

    status "Testing: LED subsystem..."
    local led_list=$(run_on_device "ls /sys/class/leds/ 2>&1")
    local led_count=$(echo "$led_list" | wc -w)
    local led_details=$(run_on_device "for led in /sys/class/leds/*; do name=\$(basename \$led); brightness=\$(cat \$led/brightness 2>/dev/null); max=\$(cat \$led/max_brightness 2>/dev/null); trigger=\$(cat \$led/trigger 2>/dev/null | grep -o '\[[^]]*\]'); echo \"\$name: \$brightness/\$max [\$trigger]\"; done" 10)

    if [ "$led_count" -ge 1 ] && [ "$led_list" != "" ]; then
        add_test_detailed "LED Controller" "PASS" \
            "${EXPECTED[led_driver]} driver" \
            "$led_count LED(s) registered" \
            "$led_details" \
            "LED class devices available"
        ((pass++))
    else
        add_test_detailed "LED Controller" "FAIL" \
            "${EXPECTED[led_driver]} driver" \
            "No LEDs found" \
            "$led_list" \
            "LED driver not loaded or no LEDs defined in device tree"
        ((fail++))
    fi

    #==========================================================================
    add_section "Power Management"
    #==========================================================================

    status "Testing: Power supply..."
    local psu_list=$(run_on_device "ls /sys/class/power_supply/ 2>&1")
    local psu_details=""
    for psu in $psu_list; do
        local psu_type=$(run_on_device "cat /sys/class/power_supply/$psu/type 2>/dev/null")
        local psu_status=$(run_on_device "cat /sys/class/power_supply/$psu/status 2>/dev/null")
        local psu_capacity=$(run_on_device "cat /sys/class/power_supply/$psu/capacity 2>/dev/null")
        local psu_voltage=$(run_on_device "cat /sys/class/power_supply/$psu/voltage_now 2>/dev/null")
        psu_details="$psu_details
=== $psu ===
Type: $psu_type
Status: $psu_status
Capacity: $psu_capacity%
Voltage: $psu_voltage uV"
    done

    local battery_found=$(echo "$psu_list" | grep -ci "battery\|bq27")
    if [ "$battery_found" -ge 1 ]; then
        local bat_capacity=$(run_on_device "cat /sys/class/power_supply/*/capacity 2>/dev/null" | head -1)
        add_test_detailed "Battery" "PASS" \
            "${EXPECTED[battery]} fuel gauge" \
            "Battery at ${bat_capacity}%" \
            "$psu_details" \
            "Battery fuel gauge operational"
        ((pass++))
    elif [ -n "$psu_list" ] && [ "$psu_list" != "" ]; then
        add_test_detailed "Battery" "PARTIAL" \
            "${EXPECTED[battery]} fuel gauge" \
            "Power supplies found but no battery" \
            "$psu_details" \
            "USB power detected but battery gauge not working"
        ((partial++))
    else
        add_test_detailed "Battery" "FAIL" \
            "${EXPECTED[battery]} fuel gauge" \
            "No power supplies found" \
            "$psu_list" \
            "Power supply subsystem not working"
        ((fail++))
    fi

    #==========================================================================
    add_section "Wireless"
    #==========================================================================

    # WiFi
    status "Testing: WiFi..."
    local net_devices=$(run_on_device "ls /sys/class/net/ 2>&1")
    local wlan_dev=$(echo "$net_devices" | grep -E "wlan|wifi" | head -1)
    local phy_devices=$(run_on_device "ls /sys/class/ieee80211/ 2>&1")
    local wifi_driver=$(run_on_device "basename \$(readlink /sys/class/net/wlan*/device/driver 2>/dev/null) 2>/dev/null" | head -1)
    local wifi_mac=$(run_on_device "cat /sys/class/net/wlan*/address 2>/dev/null" | head -1)

    if [ -n "$wlan_dev" ]; then
        add_test_detailed "WiFi" "PASS" \
            "${EXPECTED[wifi_chip]} driver" \
            "$wlan_dev (driver: $wifi_driver)" \
            "Network devices: $net_devices
PHY devices: $phy_devices
MAC: $wifi_mac
Driver: $wifi_driver" \
            "WiFi interface registered"
        ((pass++))
    elif [ -n "$phy_devices" ] && [ "$phy_devices" != "" ]; then
        add_test_detailed "WiFi" "PARTIAL" \
            "${EXPECTED[wifi_chip]}" \
            "PHY found but no wlan interface" \
            "PHY: $phy_devices
Net: $net_devices" \
            "WiFi PHY detected but interface not created - firmware issue?"
        ((partial++))
    else
        add_test_detailed "WiFi" "FAIL" \
            "${EXPECTED[wifi_chip]} driver" \
            "Not detected" \
            "Net: $net_devices" \
            "WiFi not working - check ath6kl driver and firmware"
        ((fail++))
    fi

    # Bluetooth
    status "Testing: Bluetooth..."
    local bt_devices=$(run_on_device "ls /sys/class/bluetooth/ 2>&1")
    local bt_hci=$(run_on_device "hciconfig 2>&1")
    local bt_rfkill=$(run_on_device "cat /sys/class/rfkill/*/type 2>&1 | grep -n bluetooth")

    if echo "$bt_devices" | grep -qi "hci"; then
        add_test_detailed "Bluetooth" "PASS" \
            "${EXPECTED[bluetooth]}" \
            "$(echo "$bt_devices" | head -1)" \
            "Devices: $bt_devices
HCI: $bt_hci
RFKill: $bt_rfkill" \
            "Bluetooth HCI registered"
        ((pass++))
    elif [ -n "$bt_rfkill" ]; then
        add_test_detailed "Bluetooth" "PARTIAL" \
            "${EXPECTED[bluetooth]}" \
            "RFKill entry found but no HCI" \
            "$bt_rfkill" \
            "Bluetooth hardware detected but HCI not initialized"
        ((partial++))
    else
        add_test_detailed "Bluetooth" "FAIL" \
            "${EXPECTED[bluetooth]}" \
            "Not detected" \
            "$bt_devices" \
            "Bluetooth not working"
        ((fail++))
    fi

    #==========================================================================
    add_section "Audio"
    #==========================================================================

    status "Testing: Audio..."
    local snd_devices=$(run_on_device "ls /dev/snd/ 2>&1")
    local snd_cards=$(run_on_device "cat /proc/asound/cards 2>&1")
    local snd_pcm=$(run_on_device "cat /proc/asound/pcm 2>&1")
    local snd_controls=$(run_on_device "cat /proc/asound/card0/codec* 2>&1 | head -20" 10)

    if echo "$snd_devices" | grep -q "control"; then
        local card_name=$(echo "$snd_cards" | grep -oP '\[.*\]' | head -1)
        add_test_detailed "Audio Codec" "PASS" \
            "${EXPECTED[audio_codec]}" \
            "$card_name" \
            "Devices: $snd_devices
Cards: $snd_cards
PCM: $snd_pcm" \
            "ALSA sound card registered"
        ((pass++))
    else
        add_test_detailed "Audio Codec" "FAIL" \
            "${EXPECTED[audio_codec]}" \
            "No sound card" \
            "Devices: $snd_devices
Cards: $snd_cards" \
            "Audio codec not working - check ASoC driver"
        ((fail++))
    fi

    #==========================================================================
    add_section "Sensors"
    #==========================================================================

    status "Testing: IIO sensors..."
    local iio_devices=$(run_on_device "ls /sys/bus/iio/devices/ 2>&1")
    local iio_details=""
    for dev in $iio_devices; do
        local iio_name=$(run_on_device "cat /sys/bus/iio/devices/$dev/name 2>/dev/null")
        iio_details="$iio_details
$dev: $iio_name"
    done

    local accel_found=$(echo "$iio_details" | grep -ci "accel\|mpu\|gyro\|imu")
    local light_found=$(run_on_device "ls /sys/class/leds/*/als_enable 2>&1" | grep -c als || echo "0")

    if [ "$accel_found" -ge 1 ]; then
        add_test_detailed "Accelerometer/Gyro" "PASS" \
            "${EXPECTED[accelerometer]}" \
            "IIO sensor(s) found" \
            "$iio_details" \
            "Motion sensors available via IIO subsystem"
        ((pass++))
    else
        add_test_detailed "Accelerometer/Gyro" "FAIL" \
            "${EXPECTED[accelerometer]}" \
            "Not detected" \
            "IIO devices: $iio_devices
Details: $iio_details" \
            "Motion sensors not working - check MPU3050/BMA150 drivers"
        ((fail++))
    fi

    #==========================================================================
    add_section "USB Gadget"
    #==========================================================================

    status "Testing: USB gadget..."
    local udc_list=$(run_on_device "ls /sys/class/udc/ 2>&1")
    local udc_state=$(run_on_device "cat /sys/class/udc/*/state 2>&1")
    local gadget_config=$(run_on_device "ls /config/usb_gadget/*/configs/*/. 2>&1")
    local gadget_functions=$(run_on_device "ls /config/usb_gadget/*/functions/ 2>&1")

    if echo "$udc_state" | grep -qi "configured"; then
        add_test_detailed "USB Device Controller" "PASS" \
            "${EXPECTED[usb_gadget]}" \
            "Configured and connected" \
            "UDC: $udc_list
State: $udc_state
Functions: $gadget_functions" \
            "USB gadget operational (we're connected through it!)"
        ((pass++))
    elif [ -n "$udc_list" ]; then
        add_test_detailed "USB Device Controller" "PARTIAL" \
            "${EXPECTED[usb_gadget]}" \
            "UDC present but state: $udc_state" \
            "UDC: $udc_list" \
            "USB controller found but not fully configured"
        ((partial++))
    else
        add_test_detailed "USB Device Controller" "FAIL" \
            "${EXPECTED[usb_gadget]}" \
            "Not found" \
            "$udc_list" \
            "USB device controller not working"
        ((fail++))
    fi

    #==========================================================================
    add_section "Clocks and Regulators"
    #==========================================================================

    status "Testing: Clock tree..."
    local clk_summary=$(run_on_device "cat /sys/kernel/debug/clk/clk_summary 2>&1 | head -60" 15)
    local clk_count=$(echo "$clk_summary" | wc -l)

    if [ "$clk_count" -gt 10 ]; then
        add_test_detailed "Clock Tree" "PASS" \
            "GCC/MMCC clocks" \
            "$clk_count clocks registered" \
            "$(echo "$clk_summary" | head -40)" \
            "Clock framework operational"
        ((pass++))
    else
        add_test_detailed "Clock Tree" "PARTIAL" \
            "GCC/MMCC clocks" \
            "Limited clock info" \
            "$clk_summary" \
            "debugfs may not be mounted or clock debug not enabled"
        ((partial++))
    fi

    status "Testing: Regulators..."
    local reg_list=$(run_on_device "ls /sys/class/regulator/ 2>&1")
    local reg_count=$(echo "$reg_list" | wc -w)
    local reg_details=$(run_on_device "for r in /sys/class/regulator/regulator.*; do name=\$(cat \$r/name 2>/dev/null); state=\$(cat \$r/state 2>/dev/null); echo \"\$(basename \$r): \$name [\$state]\"; done 2>&1 | head -20" 10)

    if [ "$reg_count" -ge 5 ]; then
        add_test_detailed "Voltage Regulators" "PASS" \
            "PM8921/PM8058 regulators" \
            "$reg_count regulators registered" \
            "$reg_details" \
            "PMIC regulator framework operational"
        ((pass++))
    else
        add_test_detailed "Voltage Regulators" "PARTIAL" \
            "PM8921/PM8058 regulators" \
            "$reg_count regulator(s)" \
            "$reg_list" \
            "Fewer regulators than expected"
        ((partial++))
    fi

    #==========================================================================
    add_section "Device Tree Verification"
    #==========================================================================

    status "Testing: Device Tree..."
    local dt_model=$(run_on_device "cat /sys/firmware/devicetree/base/model 2>&1" | tr -d '\0')
    local dt_compat=$(run_on_device "cat /sys/firmware/devicetree/base/compatible 2>&1" | tr '\0' ', ')
    local dt_soc=$(run_on_device "ls /sys/firmware/devicetree/base/soc*/ 2>&1 | head -20")

    if echo "$dt_compat" | grep -qi "tenderloin\|topaz"; then
        add_test_detailed "Device Tree" "PASS" \
            "HP TouchPad (tenderloin/topaz)" \
            "$dt_model" \
            "Compatible: $dt_compat

SoC nodes:
$dt_soc" \
            "Correct device tree loaded"
        ((pass++))
    else
        add_test_detailed "Device Tree" "PARTIAL" \
            "HP TouchPad DT" \
            "$dt_model" \
            "$dt_compat" \
            "Device tree loaded but may not be TouchPad-specific"
        ((partial++))
    fi

    #==========================================================================
    add_section "Kernel Messages (Errors/Warnings)"
    #==========================================================================

    status "Collecting: Kernel messages..."
    local dmesg_errors=$(run_on_device "dmesg | grep -iE 'error|fail|unable|cannot' | tail -30" 15)
    local dmesg_warnings=$(run_on_device "dmesg | grep -iE 'warn|timeout|retry' | tail -20" 15)
    local dmesg_probes=$(run_on_device "dmesg | grep -iE 'probe|register|found|detected' | tail -30" 15)

    add_test_detailed "Kernel Errors" "INFO" \
        "" \
        "$(echo "$dmesg_errors" | wc -l) error messages" \
        "$dmesg_errors" \
        "Review these for driver issues"

    add_test_detailed "Kernel Warnings" "INFO" \
        "" \
        "$(echo "$dmesg_warnings" | wc -l) warning messages" \
        "$dmesg_warnings" \
        "Warnings may indicate minor issues"

    add_test_detailed "Driver Probes" "INFO" \
        "" \
        "Driver initialization messages" \
        "$dmesg_probes" \
        "Shows which drivers loaded successfully"

    #==========================================================================
    add_section "Summary"
    #==========================================================================

    local total=$((pass + fail + partial))
    local pass_pct=$((pass * 100 / total))

    cat >> "$REPORT_FILE" << EOF

### Test Statistics

| Category | Count | Percentage |
|----------|-------|------------|
| ✅ **PASS** | $pass | ${pass_pct}% |
| ❌ **FAIL** | $fail | $((fail * 100 / total))% |
| ⚠️ **PARTIAL** | $partial | $((partial * 100 / total))% |
| **Total Tests** | $total | 100% |

### Hardware Readiness

EOF

    if [ $fail -eq 0 ]; then
        echo "🎉 **All critical hardware tests passed!** The kernel appears fully functional." >> "$REPORT_FILE"
    elif [ $fail -le 2 ]; then
        echo "⚠️ **Most hardware working.** $fail component(s) need attention." >> "$REPORT_FILE"
    else
        echo "🔧 **Multiple hardware issues detected.** $fail component(s) failing." >> "$REPORT_FILE"
    fi

    cat >> "$REPORT_FILE" << EOF

### Next Steps

EOF
    if [ $fail -gt 0 ]; then
        echo "1. Review FAIL items above and check corresponding drivers" >> "$REPORT_FILE"
        echo "2. Check dmesg errors for clues about failures" >> "$REPORT_FILE"
        echo "3. Verify device tree bindings for failing components" >> "$REPORT_FILE"
    fi
    if [ $partial -gt 0 ]; then
        echo "- PARTIAL items may need driver updates or firmware" >> "$REPORT_FILE"
    fi
    echo "" >> "$REPORT_FILE"
    echo "---" >> "$REPORT_FILE"
    echo "*Report generated by test-touchpad-hardware.sh v2.0*" >> "$REPORT_FILE"

    echo ""
    status "======================================"
    status "Test Results: $pass PASS, $fail FAIL, $partial PARTIAL (${pass_pct}%)"
    status "Report saved to: $REPORT_FILE"
    status "======================================"
}

# Main
main() {
    echo ""
    echo "=========================================="
    echo "  HP TouchPad Hardware Test Script"
    echo "=========================================="
    echo ""

    # Wait for device
    if ! wait_for_device; then
        error "Device not found. Make sure TouchPad is booted with LuneOS kernel."
        exit 1
    fi

    # Configure network
    if ! configure_network; then
        error "Failed to configure network"
        exit 1
    fi

    # Initialize report
    init_report

    # Run tests
    status "Starting hardware tests..."
    run_tests

    # Copy report to latest archive if it exists
    if [ -d "$ARCHIVE_DIR" ]; then
        LATEST_ARCHIVE=$(ls -td "$ARCHIVE_DIR"/*/ 2>/dev/null | head -1)
        if [ -n "$LATEST_ARCHIVE" ] && [ -d "$LATEST_ARCHIVE" ]; then
            cp "$REPORT_FILE" "$LATEST_ARCHIVE/hardware-test.md"
            status "Report also saved to: $LATEST_ARCHIVE/hardware-test.md"
        fi
    fi

    echo ""
    status "Done! View report with: cat $REPORT_FILE"
}

main "$@"
