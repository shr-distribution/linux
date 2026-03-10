#!/bin/sh
# eMMC Speed Benchmark for HP TouchPad
#
# Usage: ./scripts/benchmark-emmc.sh [iterations] [test_size_mb]
#   iterations:   number of test iterations (default: 20)
#   test_size_mb: size of each test in MB (default: 100)
#
# This script:
# 1. Sets CPU to max frequency (performance governor)
# 2. Tests eMMC sequential read with different block sizes
# 3. Tests eMMC sequential read from different offsets
# 4. Drops caches between runs for accurate results
# 5. Reports statistics (min, max, average, median)
#
# Works on both webOS (BusyBox) and LuneOS
#
# Test device: /dev/mmcblk0 (eMMC)
# Safe read-only tests - does not write to eMMC

ITERATIONS="${1:-20}"
TEST_SIZE_MB="${2:-100}"
EMMC_DEV="/dev/mmcblk0"

# Block sizes to test (in bytes)
BS_SMALL=65536      # 64K
BS_MEDIUM=1048576   # 1M
BS_LARGE=4194304    # 4M

# =============================================================================
# Helper: Get time from /proc/uptime (works on BusyBox)
# =============================================================================
get_uptime_ms() {
    awk '{printf "%.0f", $1 * 1000}' /proc/uptime
}

# =============================================================================
# Helper: Drop filesystem caches
# =============================================================================
drop_caches() {
    sync
    echo 3 > /proc/sys/vm/drop_caches 2>/dev/null
}

# =============================================================================
# Helper: Calculate statistics from space-separated values
# =============================================================================
calc_stats() {
    echo "$1" | awk -v size="$2" '
    {
        n = NF
        if (n == 0) exit

        # Collect values
        for (i = 1; i <= NF; i++) {
            vals[i] = $i
        }

        # Sort for median
        for (i = 1; i <= n; i++) {
            for (j = i + 1; j <= n; j++) {
                if (vals[i] > vals[j]) {
                    tmp = vals[i]
                    vals[i] = vals[j]
                    vals[j] = tmp
                }
            }
        }

        min = vals[1]
        max = vals[n]
        sum = 0
        for (i = 1; i <= n; i++) {
            sum += vals[i]
        }
        avg = sum / n

        # Median
        if (n % 2 == 1) {
            median = vals[int(n/2) + 1]
        } else {
            median = (vals[n/2] + vals[n/2 + 1]) / 2
        }

        # Calculate speeds (MB/s)
        min_speed = size / max
        max_speed = size / min
        avg_speed = size / avg
        median_speed = size / median

        printf "  Min:    %6.3f s  (%6.1f MB/s)\n", min, max_speed
        printf "  Max:    %6.3f s  (%6.1f MB/s)\n", max, min_speed
        printf "  Avg:    %6.3f s  (%6.1f MB/s)\n", avg, avg_speed
        printf "  Median: %6.3f s  (%6.1f MB/s)\n", median, median_speed
    }'
}

# =============================================================================
# Helper: Run a single read test
# =============================================================================
run_read_test() {
    local bs="$1"
    local count="$2"
    local skip="$3"
    local bs_label="$4"

    drop_caches

    START=$(get_uptime_ms)
    dd if="$EMMC_DEV" of=/dev/null bs="$bs" count="$count" skip="$skip" 2>/dev/null
    END=$(get_uptime_ms)

    ELAPSED_MS=$((END - START))
    echo "$ELAPSED_MS"
}

# =============================================================================
# Header
# =============================================================================
echo "=============================================="
echo "  eMMC Speed Benchmark for HP TouchPad"
echo "=============================================="
echo ""
echo "Iterations:    $ITERATIONS"
echo "Test size:     ${TEST_SIZE_MB} MB per run"
echo "Device:        $EMMC_DEV"
echo ""

# Check if device exists
if [ ! -b "$EMMC_DEV" ]; then
    echo "ERROR: $EMMC_DEV not found!"
    exit 1
fi

# =============================================================================
# System Info
# =============================================================================
echo "--- System Info ---"
echo "Kernel: $(uname -r)"
if [ -f /proc/meminfo ]; then
    grep -E "MemTotal|MemFree" /proc/meminfo 2>/dev/null
fi

# Get eMMC info if available
if [ -f /sys/class/mmc_host/mmc0/mmc0:0001/name ]; then
    EMMC_NAME=$(cat /sys/class/mmc_host/mmc0/mmc0:0001/name 2>/dev/null)
    echo "eMMC: $EMMC_NAME"
fi
if [ -f /sys/class/mmc_host/mmc0/mmc0:0001/rev ]; then
    EMMC_REV=$(cat /sys/class/mmc_host/mmc0/mmc0:0001/rev 2>/dev/null)
    echo "eMMC Rev: $EMMC_REV"
fi
echo ""

# =============================================================================
# Set CPU to max frequency
# =============================================================================
echo "--- Setting CPU to Max Frequency ---"

CPU0_GOV="/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
CPU1_GOV="/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor"
CPU0_FREQ="/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"

if [ -f "$CPU0_GOV" ]; then
    OLD_GOV=$(cat "$CPU0_GOV" 2>/dev/null)
    echo "Current governor: $OLD_GOV"

    # Set performance governor
    echo "performance" > "$CPU0_GOV" 2>/dev/null
    if [ -f "$CPU1_GOV" ]; then
        echo "performance" > "$CPU1_GOV" 2>/dev/null
    fi

    NEW_GOV=$(cat "$CPU0_GOV" 2>/dev/null)
    echo "New governor: $NEW_GOV"

    if [ -f "$CPU0_FREQ" ]; then
        FREQ=$(cat "$CPU0_FREQ" 2>/dev/null)
        FREQ_MHZ=$((FREQ / 1000))
        echo "CPU frequency: ${FREQ_MHZ} MHz"
    fi
else
    echo "Warning: cpufreq not available"
    OLD_GOV=""
fi
echo ""

# =============================================================================
# Test 1: Sequential Read with 1MB blocks (standard benchmark)
# =============================================================================
echo "=============================================="
echo "  Test 1: Sequential Read (bs=1M)"
echo "=============================================="
echo ""

COUNT_1M=$TEST_SIZE_MB
TIMES_1M=""

for i in $(seq 1 $ITERATIONS); do
    # Vary skip offset to avoid any caching effects
    SKIP_OFFSET=$((i * 100))

    ELAPSED_MS=$(run_read_test "$BS_MEDIUM" "$COUNT_1M" "$SKIP_OFFSET" "1M")
    ELAPSED=$(awk -v ms="$ELAPSED_MS" 'BEGIN {printf "%.3f", ms / 1000}')
    SPEED=$(awk -v ms="$ELAPSED_MS" -v size="$TEST_SIZE_MB" 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

    printf "  Run %2d: %6.3f s  (%6.1f MB/s)\n" "$i" "$ELAPSED" "$SPEED"

    if [ -z "$TIMES_1M" ]; then
        TIMES_1M="$ELAPSED"
    else
        TIMES_1M="$TIMES_1M $ELAPSED"
    fi
done

echo ""
echo "  --- Statistics (bs=1M) ---"
calc_stats "$TIMES_1M" "$TEST_SIZE_MB"
echo ""

# =============================================================================
# Test 2: Sequential Read with 64K blocks (more I/O operations)
# =============================================================================
echo "=============================================="
echo "  Test 2: Sequential Read (bs=64K)"
echo "=============================================="
echo ""

# For 64K blocks, use same total size
COUNT_64K=$((TEST_SIZE_MB * 16))  # 16 * 64K = 1M
TIMES_64K=""

for i in $(seq 1 $ITERATIONS); do
    SKIP_OFFSET=$((i * 1600))  # Skip in 64K blocks

    ELAPSED_MS=$(run_read_test "$BS_SMALL" "$COUNT_64K" "$SKIP_OFFSET" "64K")
    ELAPSED=$(awk -v ms="$ELAPSED_MS" 'BEGIN {printf "%.3f", ms / 1000}')
    SPEED=$(awk -v ms="$ELAPSED_MS" -v size="$TEST_SIZE_MB" 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

    printf "  Run %2d: %6.3f s  (%6.1f MB/s)\n" "$i" "$ELAPSED" "$SPEED"

    if [ -z "$TIMES_64K" ]; then
        TIMES_64K="$ELAPSED"
    else
        TIMES_64K="$TIMES_64K $ELAPSED"
    fi
done

echo ""
echo "  --- Statistics (bs=64K) ---"
calc_stats "$TIMES_64K" "$TEST_SIZE_MB"
echo ""

# =============================================================================
# Test 3: Sequential Read with 4M blocks (large sequential)
# =============================================================================
echo "=============================================="
echo "  Test 3: Sequential Read (bs=4M)"
echo "=============================================="
echo ""

COUNT_4M=$((TEST_SIZE_MB / 4))
if [ "$COUNT_4M" -lt 1 ]; then
    COUNT_4M=1
fi
ACTUAL_SIZE_4M=$((COUNT_4M * 4))
TIMES_4M=""

for i in $(seq 1 $ITERATIONS); do
    SKIP_OFFSET=$((i * 25))  # Skip in 4M blocks

    ELAPSED_MS=$(run_read_test "$BS_LARGE" "$COUNT_4M" "$SKIP_OFFSET" "4M")
    ELAPSED=$(awk -v ms="$ELAPSED_MS" 'BEGIN {printf "%.3f", ms / 1000}')
    SPEED=$(awk -v ms="$ELAPSED_MS" -v size="$ACTUAL_SIZE_4M" 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

    printf "  Run %2d: %6.3f s  (%6.1f MB/s)\n" "$i" "$ELAPSED" "$SPEED"

    if [ -z "$TIMES_4M" ]; then
        TIMES_4M="$ELAPSED"
    else
        TIMES_4M="$TIMES_4M $ELAPSED"
    fi
done

echo ""
echo "  --- Statistics (bs=4M) ---"
calc_stats "$TIMES_4M" "$ACTUAL_SIZE_4M"
echo ""

# =============================================================================
# Test 4: Random offset reads (to test seek performance)
# =============================================================================
echo "=============================================="
echo "  Test 4: Varied Offset Reads (bs=1M)"
echo "=============================================="
echo "  (Tests read from different disk regions)"
echo ""

# Get approximate disk size in MB (use 29000 MB as fallback for 32GB)
DISK_SIZE_MB=29000
TIMES_VAR=""

for i in $(seq 1 $ITERATIONS); do
    # Calculate a pseudo-random offset based on iteration
    # This spreads reads across different parts of the disk
    OFFSET_MB=$(( (i * 1337) % (DISK_SIZE_MB - TEST_SIZE_MB - 1000) + 500 ))

    ELAPSED_MS=$(run_read_test "$BS_MEDIUM" "$TEST_SIZE_MB" "$OFFSET_MB" "1M")
    ELAPSED=$(awk -v ms="$ELAPSED_MS" 'BEGIN {printf "%.3f", ms / 1000}')
    SPEED=$(awk -v ms="$ELAPSED_MS" -v size="$TEST_SIZE_MB" 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

    printf "  Run %2d: %6.3f s  (%6.1f MB/s)  [offset: %d MB]\n" "$i" "$ELAPSED" "$SPEED" "$OFFSET_MB"

    if [ -z "$TIMES_VAR" ]; then
        TIMES_VAR="$ELAPSED"
    else
        TIMES_VAR="$TIMES_VAR $ELAPSED"
    fi
done

echo ""
echo "  --- Statistics (varied offset) ---"
calc_stats "$TIMES_VAR" "$TEST_SIZE_MB"
echo ""

# =============================================================================
# Cleanup
# =============================================================================
echo "--- Cleanup ---"

# Restore governor if we changed it
if [ -n "$OLD_GOV" ] && [ "$OLD_GOV" != "performance" ]; then
    echo "Restoring governor to $OLD_GOV"
    echo "$OLD_GOV" > "$CPU0_GOV" 2>/dev/null
    if [ -f "$CPU1_GOV" ]; then
        echo "$OLD_GOV" > "$CPU1_GOV" 2>/dev/null
    fi
else
    echo "Governor unchanged"
fi
echo ""

# =============================================================================
# Summary
# =============================================================================
echo "=============================================="
echo "  Summary"
echo "=============================================="
echo ""

# Extract median speeds for summary
MEDIAN_1M=$(echo "$TIMES_1M" | awk -v size="$TEST_SIZE_MB" '
{
    n = NF
    for (i = 1; i <= NF; i++) vals[i] = $i
    for (i = 1; i <= n; i++) {
        for (j = i + 1; j <= n; j++) {
            if (vals[i] > vals[j]) { tmp = vals[i]; vals[i] = vals[j]; vals[j] = tmp }
        }
    }
    if (n % 2 == 1) median = vals[int(n/2) + 1]
    else median = (vals[n/2] + vals[n/2 + 1]) / 2
    printf "%.1f", size / median
}')

MEDIAN_64K=$(echo "$TIMES_64K" | awk -v size="$TEST_SIZE_MB" '
{
    n = NF
    for (i = 1; i <= NF; i++) vals[i] = $i
    for (i = 1; i <= n; i++) {
        for (j = i + 1; j <= n; j++) {
            if (vals[i] > vals[j]) { tmp = vals[i]; vals[i] = vals[j]; vals[j] = tmp }
        }
    }
    if (n % 2 == 1) median = vals[int(n/2) + 1]
    else median = (vals[n/2] + vals[n/2 + 1]) / 2
    printf "%.1f", size / median
}')

MEDIAN_4M=$(echo "$TIMES_4M" | awk -v size="$ACTUAL_SIZE_4M" '
{
    n = NF
    for (i = 1; i <= NF; i++) vals[i] = $i
    for (i = 1; i <= n; i++) {
        for (j = i + 1; j <= n; j++) {
            if (vals[i] > vals[j]) { tmp = vals[i]; vals[i] = vals[j]; vals[j] = tmp }
        }
    }
    if (n % 2 == 1) median = vals[int(n/2) + 1]
    else median = (vals[n/2] + vals[n/2 + 1]) / 2
    printf "%.1f", size / median
}')

MEDIAN_VAR=$(echo "$TIMES_VAR" | awk -v size="$TEST_SIZE_MB" '
{
    n = NF
    for (i = 1; i <= NF; i++) vals[i] = $i
    for (i = 1; i <= n; i++) {
        for (j = i + 1; j <= n; j++) {
            if (vals[i] > vals[j]) { tmp = vals[i]; vals[i] = vals[j]; vals[j] = tmp }
        }
    }
    if (n % 2 == 1) median = vals[int(n/2) + 1]
    else median = (vals[n/2] + vals[n/2 + 1]) / 2
    printf "%.1f", size / median
}')

echo "  Median Read Speeds:"
echo "    bs=64K:         ${MEDIAN_64K} MB/s"
echo "    bs=1M:          ${MEDIAN_1M} MB/s"
echo "    bs=4M:          ${MEDIAN_4M} MB/s"
echo "    Varied offset:  ${MEDIAN_VAR} MB/s"
echo ""
echo "=============================================="
echo "  Benchmark Complete"
echo "=============================================="
