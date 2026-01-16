#!/bin/bash
# Test eMMC DMA vs PIO performance on HP TouchPad
#
# Usage: ./scripts/test-emmc-dma.sh [device]
#   device: block device to test (default: /dev/mmcblk0)
#
# This script:
# 1. Detects if DMA or PIO mode is being used
# 2. Checks ADM DMA interrupt counts
# 3. Runs read benchmarks
# 4. Reports results

set -e

DEVICE="${1:-/dev/mmcblk0}"
TEST_SIZE_MB=64
TEST_COUNT=3

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=============================================="
echo "  eMMC DMA/PIO Test Script for HP TouchPad"
echo "=============================================="
echo ""

# Check if running as root
if [ "$(id -u)" != "0" ]; then
    echo -e "${YELLOW}Warning: Not running as root. Some tests may fail.${NC}"
    echo ""
fi

# Check if device exists
if [ ! -b "$DEVICE" ]; then
    echo -e "${RED}Error: Device $DEVICE not found${NC}"
    echo ""
    echo "Available block devices:"
    lsblk -d 2>/dev/null || ls /dev/mmcblk* /dev/sd* 2>/dev/null
    exit 1
fi

echo "Test device: $DEVICE"
echo ""

# =============================================================================
# 1. Check DMA mode
# =============================================================================
echo "--- DMA Mode Detection ---"

# Check kernel log for DMA mode messages
if dmesg | grep -q "ADM DMA: enabled"; then
    echo -e "${GREEN}DMA Mode: ADM DMA enabled${NC}"
    DMA_MODE="ADM"
elif dmesg | grep -q "ADM DMA: using PIO mode"; then
    echo -e "${YELLOW}DMA Mode: PIO (ADM DMA disabled)${NC}"
    DMA_MODE="PIO"
elif dmesg | grep -q "DMA setup failed"; then
    echo -e "${YELLOW}DMA Mode: PIO (DMA setup failed)${NC}"
    DMA_MODE="PIO"
else
    echo "DMA Mode: Unknown (check dmesg for details)"
    DMA_MODE="UNKNOWN"
fi

# Check for MMCI/PL180 driver messages
echo ""
echo "MMCI driver messages:"
dmesg | grep -i "mmci\|mmc0\|mmc1" | tail -10

echo ""

# =============================================================================
# 2. Check ADM DMA interrupts
# =============================================================================
echo "--- ADM DMA Interrupt Status ---"

ADM_IRQ_BEFORE=0
if [ -f /proc/interrupts ]; then
    # Look for ADM interrupts (IRQ 199 for ADM1 on APQ8060)
    ADM_LINE=$(grep -E "adm|167|199" /proc/interrupts 2>/dev/null | head -1)
    if [ -n "$ADM_LINE" ]; then
        echo "ADM IRQ line: $ADM_LINE"
        ADM_IRQ_BEFORE=$(echo "$ADM_LINE" | awk '{sum=0; for(i=2;i<=NF-2;i++) sum+=$i; print sum}')
        echo "Total ADM interrupts (before test): $ADM_IRQ_BEFORE"
    else
        echo "ADM interrupt line not found in /proc/interrupts"
        echo "Available interrupt lines:"
        grep -E "dma|adm|mmc" /proc/interrupts 2>/dev/null || echo "  (none matching)"
    fi
fi

echo ""

# =============================================================================
# 3. Check DMA channel status (if available)
# =============================================================================
echo "--- DMA Channel Info ---"

if [ -d /sys/class/dma ]; then
    echo "DMA channels:"
    ls /sys/class/dma/ 2>/dev/null | head -10
else
    echo "No DMA sysfs entries found"
fi

echo ""

# =============================================================================
# 4. Run read benchmark
# =============================================================================
echo "--- Read Performance Test ---"
echo "Reading ${TEST_SIZE_MB}MB from $DEVICE ($TEST_COUNT iterations)"
echo ""

# Drop caches before test
if [ -w /proc/sys/vm/drop_caches ]; then
    sync
    echo 3 > /proc/sys/vm/drop_caches 2>/dev/null || true
fi

TOTAL_SPEED=0
for i in $(seq 1 $TEST_COUNT); do
    echo -n "  Test $i: "

    # Use dd to read from device
    RESULT=$(dd if=$DEVICE of=/dev/null bs=1M count=$TEST_SIZE_MB 2>&1)

    # Extract speed from dd output
    SPEED=$(echo "$RESULT" | grep -oE '[0-9.]+ [MG]B/s' | tail -1)
    if [ -z "$SPEED" ]; then
        SPEED=$(echo "$RESULT" | grep -oE '[0-9.]+ MB/s' | tail -1)
    fi

    if [ -n "$SPEED" ]; then
        echo "$SPEED"
        # Extract numeric value for averaging
        SPEED_NUM=$(echo "$SPEED" | grep -oE '[0-9.]+')
        TOTAL_SPEED=$(echo "$TOTAL_SPEED + $SPEED_NUM" | bc 2>/dev/null || echo "$TOTAL_SPEED")
    else
        echo "Could not parse speed"
        echo "  Raw output: $RESULT"
    fi

    # Drop caches between tests
    if [ -w /proc/sys/vm/drop_caches ]; then
        sync
        echo 3 > /proc/sys/vm/drop_caches 2>/dev/null || true
    fi

    sleep 1
done

# Calculate average
if [ "$TEST_COUNT" -gt 0 ] && [ -n "$TOTAL_SPEED" ] && [ "$TOTAL_SPEED" != "0" ]; then
    AVG_SPEED=$(echo "scale=2; $TOTAL_SPEED / $TEST_COUNT" | bc 2>/dev/null || echo "N/A")
    echo ""
    echo -e "  ${GREEN}Average read speed: ${AVG_SPEED} MB/s${NC}"
fi

echo ""

# =============================================================================
# 5. Check ADM interrupts after test
# =============================================================================
echo "--- ADM DMA Interrupt Status (After Test) ---"

if [ -f /proc/interrupts ]; then
    ADM_LINE=$(grep -E "adm|167|199" /proc/interrupts 2>/dev/null | head -1)
    if [ -n "$ADM_LINE" ]; then
        ADM_IRQ_AFTER=$(echo "$ADM_LINE" | awk '{sum=0; for(i=2;i<=NF-2;i++) sum+=$i; print sum}')
        echo "Total ADM interrupts (after test): $ADM_IRQ_AFTER"

        IRQ_DIFF=$((ADM_IRQ_AFTER - ADM_IRQ_BEFORE))
        if [ "$IRQ_DIFF" -gt 0 ]; then
            echo -e "${GREEN}DMA interrupts during test: $IRQ_DIFF (DMA is working!)${NC}"
        else
            echo -e "${YELLOW}DMA interrupts during test: $IRQ_DIFF (DMA not firing)${NC}"
        fi
    fi
fi

echo ""

# =============================================================================
# 6. Summary
# =============================================================================
echo "=============================================="
echo "  Summary"
echo "=============================================="
echo ""
echo "Device: $DEVICE"
echo "DMA Mode: $DMA_MODE"

if [ "$DMA_MODE" = "ADM" ]; then
    if [ "$IRQ_DIFF" -gt 0 ] 2>/dev/null; then
        echo -e "${GREEN}Status: ADM DMA is working${NC}"
    else
        echo -e "${RED}Status: ADM DMA enabled but no interrupts (broken)${NC}"
    fi
elif [ "$DMA_MODE" = "PIO" ]; then
    echo -e "${YELLOW}Status: Running in PIO mode (slower but working)${NC}"
fi

echo ""
echo "Expected performance:"
echo "  PIO mode:  ~8-15 MB/s"
echo "  DMA mode:  ~25-40 MB/s"
echo ""

# =============================================================================
# 7. Kernel log snippet
# =============================================================================
echo "--- Recent kernel messages (last 20 mmc/dma related) ---"
dmesg | grep -iE "mmc|dma|adm|sdcc|mmci" | tail -20

echo ""
echo "Done."
