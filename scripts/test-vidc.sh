#!/bin/bash
# VIDC (Video Codec) test script for HP TouchPad
# Tests firmware loading and checks for IRQ storms

set -e

DEVICE_IP="172.16.42.2"
TELNET_TIMEOUT=10

echo "=== VIDC Test Script ==="
echo "Checking device connectivity..."

if ! ping -c 1 -W 2 "$DEVICE_IP" > /dev/null 2>&1; then
    echo "ERROR: Device at $DEVICE_IP is not reachable"
    exit 1
fi

echo "Device is reachable"
echo ""

# Function to run command via telnet
run_remote() {
    local cmd="$1"
    (
        sleep 1
        echo "$cmd"
        sleep 2
        echo "exit"
    ) | telnet "$DEVICE_IP" 2>&1 | grep -v "^Trying\|^Connected\|^Escape\|^LuneOS\|^sh-"
}

echo "=== 1. Check VIDC driver probe ==="
run_remote "dmesg | grep -i vidc"
echo ""

echo "=== 2. Check video device nodes ==="
run_remote "ls -la /dev/video* | grep -E 'video[67]'"
echo ""

echo "=== 3. Check IRQ count (should be 0 or low) ==="
run_remote "cat /proc/interrupts | grep vidc"
echo ""

echo "=== 4. Check for IRQ storm messages ==="
STORM_CHECK=$(run_remote "dmesg | grep -i 'irq.*storm'")
if [ -z "$STORM_CHECK" ]; then
    echo "✓ No IRQ storm detected"
else
    echo "✗ IRQ STORM DETECTED:"
    echo "$STORM_CHECK"
fi
echo ""

echo "=== 5. Check firmware files ==="
FW_CHECK=$(run_remote "find /lib/firmware -name '*vidc*' -o -name 'vidc_1080p.fw' 2>&1")
if [ -z "$FW_CHECK" ]; then
    echo "✗ No firmware files found!"
else
    echo "✓ Firmware found:"
    run_remote "ls -lh /lib/firmware/qcom/vidc_1080p.fw"
fi
echo ""

echo "=== 6. Check encoder device info ==="
run_remote "v4l2-ctl -d /dev/video7 --info"
echo ""

echo "=== 7. Check supported formats ==="
run_remote "v4l2-ctl -d /dev/video7 --list-formats"
run_remote "v4l2-ctl -d /dev/video7 --list-formats-out"
echo ""

echo "=== 8. Enable debug logging ==="
run_remote "echo 'module qcom_vidc +p' > /sys/kernel/debug/dynamic_debug/control"
echo "✓ Debug logging enabled"
echo ""

echo "=== 9. Try to query encoder (may trigger firmware load) ==="
run_remote "dmesg -C"
run_remote "v4l2-ctl -d /dev/video7 --all 2>&1 | head -20"
sleep 2
echo ""

echo "=== 10. Check for new kernel messages ==="
DMESG_OUTPUT=$(run_remote "dmesg")
if [ -z "$DMESG_OUTPUT" ]; then
    echo "✓ No errors or warnings"
else
    echo "Kernel messages:"
    echo "$DMESG_OUTPUT"
fi
echo ""

echo "=== 11. Test firmware loading via stream trigger ==="
echo "Starting stream to trigger firmware load (this opens a channel)..."
(
    sleep 1
    echo "dmesg -C"
    sleep 1
    echo "timeout 5 v4l2-ctl -d /dev/video7 --stream-mmap --stream-count=0 2>&1 &"
    sleep 1
    echo "jobs"
    sleep 5
    echo "killall v4l2-ctl 2>/dev/null || true"
    sleep 1
    echo "echo '--- Firmware load messages: ---'"
    echo "dmesg | head -80"
    sleep 2
    echo "exit"
) | telnet "$DEVICE_IP" 2>&1 | grep -v "^Trying\|^Connected\|^Escape" | tail -100
echo ""

echo "=== 12. Final IRQ count check ==="
run_remote "cat /proc/interrupts | grep vidc"
echo ""

echo "=== 13. Check for any IRQ storm after firmware load ==="
STORM_CHECK2=$(run_remote "dmesg | grep -i 'irq.*storm'")
if [ -z "$STORM_CHECK2" ]; then
    echo "✓ No IRQ storm after firmware load"
else
    echo "✗ IRQ STORM DETECTED AFTER FW LOAD:"
    echo "$STORM_CHECK2"
fi
echo ""

echo "=== Test complete ==="
