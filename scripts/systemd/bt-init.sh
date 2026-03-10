#!/bin/bash
#
# bt-init.sh - HP TouchPad Bluetooth initialization script
#
# BCM4329 Bluetooth chip requires BCSP protocol.
# This script handles GPIO power control and hciattach.
#
# BD Address: The chip defaults to 00:02:5B:00:A5:A5. The Palm-assigned
# address from device tokens can be set via the hci_uart bdaddr module
# parameter, which sends a BCCMD during BCSP initialization.
#
# See reports/bcm4329-bluetooth-analysis.md for details.
#

set -e

# GPIO numbers (TLMM base 512 + GPIO number)
GPIO_BASE=512
BT_HOST_WAKE=$((GPIO_BASE + 129))  # Input: chip signals host
BT_PWR=$((GPIO_BASE + 130))        # Output: power enable
BT_WAKE=$((GPIO_BASE + 131))       # Output: wake chip
BT_RST=$((GPIO_BASE + 138))        # Output: reset (active low)

# UART device
UART_DEV="/dev/ttyMSM1"
BAUD_RATE="115200"
PROTOCOL="bcsp"

# PID file for hciattach
PID_FILE="/run/hciattach.pid"

# Token partition for BD address
TOKEN_PARTITION="/dev/mmcblk0p12"

log() {
    logger -t "bt-init" "$@"
    echo "bt-init: $@"
}

read_bdaddr_from_tokens() {
    # Read BToADDR from device tokens partition
    if [ -b "$TOKEN_PARTITION" ]; then
        local addr=$(dd if="$TOKEN_PARTITION" bs=4096 count=256 2>/dev/null | \
                     strings -n 6 | grep -A1 '^BToADDR$' | tail -1)
        if echo "$addr" | grep -qE '^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$'; then
            echo "$addr"
            return 0
        fi
    fi
    # Return empty if not found
    echo ""
    return 1
}

export_gpio() {
    local gpio=$1
    if [ ! -d "/sys/class/gpio/gpio${gpio}" ]; then
        echo "$gpio" > /sys/class/gpio/export 2>/dev/null || true
    fi
}

gpio_direction() {
    local gpio=$1
    local dir=$2
    echo "$dir" > "/sys/class/gpio/gpio${gpio}/direction" 2>/dev/null || true
}

gpio_set() {
    local gpio=$1
    local val=$2
    echo "$val" > "/sys/class/gpio/gpio${gpio}/value"
}

gpio_get() {
    local gpio=$1
    cat "/sys/class/gpio/gpio${gpio}/value"
}

power_on() {
    log "Powering on Bluetooth chip..."

    # Export GPIOs
    export_gpio $BT_PWR
    export_gpio $BT_WAKE
    export_gpio $BT_RST
    export_gpio $BT_HOST_WAKE

    # Set directions
    gpio_direction $BT_PWR "out"
    gpio_direction $BT_WAKE "out"
    gpio_direction $BT_RST "out"
    gpio_direction $BT_HOST_WAKE "in"

    # Power sequence
    gpio_set $BT_PWR 1      # Power on
    gpio_set $BT_WAKE 1     # Wake chip
    gpio_set $BT_RST 0      # Assert reset
    sleep 0.1
    gpio_set $BT_RST 1      # Release reset

    # Wait for chip to be ready (HOST_WAKE should go high)
    sleep 0.2
    local host_wake=$(gpio_get $BT_HOST_WAKE)
    if [ "$host_wake" = "1" ]; then
        log "Bluetooth chip ready (HOST_WAKE=1)"
    else
        log "Warning: HOST_WAKE=$host_wake (expected 1)"
    fi
}

power_off() {
    log "Powering off Bluetooth chip..."

    # Assert reset and power off
    gpio_set $BT_RST 0 2>/dev/null || true
    gpio_set $BT_WAKE 0 2>/dev/null || true
    gpio_set $BT_PWR 0 2>/dev/null || true
}

start() {
    log "Starting Bluetooth..."

    # Check if already running
    if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
        log "hciattach already running"
        return 0
    fi

    # Check UART device exists
    if [ ! -c "$UART_DEV" ]; then
        log "Error: $UART_DEV not found"
        return 1
    fi

    # Read BD address from device tokens
    local bdaddr=$(read_bdaddr_from_tokens)

    # Load bluetooth module
    modprobe -q bluetooth || true

    # Load hci_uart with bdaddr parameter if we have one
    # This sends BCCMD to set BD address during BCSP initialization
    if [ -n "$bdaddr" ]; then
        log "Setting BD address via BCCMD: $bdaddr"
        # Unload first in case it's already loaded without the parameter
        rmmod hci_uart 2>/dev/null || true
        modprobe hci_uart bdaddr="$bdaddr"
    else
        log "No BD address in tokens, using default: 00:02:5B:00:A5:A5"
        modprobe -q hci_uart || true
    fi

    # Power on the chip
    power_on

    # Attach UART with BCSP protocol
    log "Attaching $UART_DEV with $PROTOCOL at $BAUD_RATE baud..."
    hciattach "$UART_DEV" "$PROTOCOL" "$BAUD_RATE" &
    local pid=$!
    echo "$pid" > "$PID_FILE"

    # Wait for HCI device to appear
    sleep 2
    if hciconfig hci0 2>/dev/null | grep -q "hci0"; then
        log "HCI device created"

        # Unblock rfkill and bring up
        rfkill unblock bluetooth 2>/dev/null || true
        hciconfig hci0 up

        # Show final BD address
        local final_addr=$(hciconfig hci0 2>/dev/null | grep "BD Address" | awk '{print $3}')
        if [ -n "$final_addr" ]; then
            log "BD Address: $final_addr"
        fi

        if hciconfig hci0 | grep -q "UP RUNNING"; then
            log "Bluetooth is UP and RUNNING"
            return 0
        else
            log "Warning: HCI device not fully up"
            return 0
        fi
    else
        log "Error: HCI device not created"
        return 1
    fi
}

stop() {
    log "Stopping Bluetooth..."

    # Bring down HCI
    hciconfig hci0 down 2>/dev/null || true

    # Kill hciattach
    if [ -f "$PID_FILE" ]; then
        local pid=$(cat "$PID_FILE")
        kill "$pid" 2>/dev/null || true
        rm -f "$PID_FILE"
    fi

    # Also kill any stray hciattach processes
    killall hciattach 2>/dev/null || true

    # Power off chip
    power_off

    log "Bluetooth stopped"
}

status() {
    echo "=== Bluetooth Status ==="
    echo ""
    echo "GPIOs:"
    echo "  BT_PWR ($BT_PWR):       $(gpio_get $BT_PWR 2>/dev/null || echo 'N/A')"
    echo "  BT_WAKE ($BT_WAKE):     $(gpio_get $BT_WAKE 2>/dev/null || echo 'N/A')"
    echo "  BT_RST ($BT_RST):       $(gpio_get $BT_RST 2>/dev/null || echo 'N/A')"
    echo "  BT_HOST_WAKE ($BT_HOST_WAKE): $(gpio_get $BT_HOST_WAKE 2>/dev/null || echo 'N/A')"
    echo ""
    echo "HCI:"
    hciconfig -a 2>/dev/null || echo "  No HCI device"
    echo ""
    echo "Modules:"
    lsmod | grep -E "bluetooth|hci" || echo "  No BT modules loaded"
    echo ""
    echo "rfkill:"
    rfkill list bluetooth 2>/dev/null || echo "  rfkill not available"
}

case "$1" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop
        sleep 1
        start
        ;;
    status)
        status
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
