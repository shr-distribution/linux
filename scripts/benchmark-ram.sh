#!/bin/sh
# RAM Speed Benchmark for HP TouchPad
#
# Usage: ./scripts/benchmark-ram.sh [iterations] [busy_wait]
#   iterations: number of test iterations (default: 20)
#   busy_wait:  set to 1 to keep CPU busy between iterations (default: 0)
#
# This script:
# 1. Sets CPU to max frequency (performance governor)
# 2. Monitors fabric clock rates (AFAB, SFAB, MMFAB) for bimodality detection
# 3. Runs memory bandwidth tests (zero -> null)
# 4. Runs tmpfs write/read tests
# 5. Reports statistics (min, max, average) and clock rate changes
#
# Test parameters from reports/ram-speed-benchmark.md:
# - Memory bandwidth: dd if=/dev/zero of=/dev/null bs=1M count=512
# - tmpfs write: dd if=/dev/zero of=/tmp/ramtest bs=1M count=128
# - tmpfs read: dd if=/tmp/ramtest of=/dev/null bs=1M

ITERATIONS="${1:-20}"
BUSY_WAIT="${2:-0}"  # Set to 1 to enable CPU busy-wait between iterations
MEM_SIZE_MB=512
TMPFS_SIZE_MB=128
TMPFS_FILE="/tmp/ramtest"

# Fabric clock paths (debugfs)
CLK_SUMMARY="/sys/kernel/debug/clk/clk_summary"
AFAB_CLK="/sys/kernel/debug/clk/afab_clk/clk_rate"
SFAB_CLK="/sys/kernel/debug/clk/sfab_clk/clk_rate"
MMFAB_CLK="/sys/kernel/debug/clk/mmfab_clk/clk_rate"

# Track clock rate changes for bimodality detection
CLOCK_CHANGES=0
CLOCK_RATES_SEEN=""

# =============================================================================
# Helper: Keep CPU busy for ~100ms to prevent idle state transitions
# =============================================================================
cpu_busy_wait() {
    if [ "$BUSY_WAIT" = "1" ]; then
        # Busy loop: increment counter ~100k times (roughly 50-100ms on Scorpion)
        i=0
        while [ $i -lt 100000 ]; do
            i=$((i + 1))
        done
    fi
}

# =============================================================================
# Helper: Read fabric clock rate (returns rate in Hz, or "N/A" if unavailable)
# =============================================================================
get_fabric_clock() {
    local clk_name="$1"
    local clk_path="/sys/kernel/debug/clk/${clk_name}/clk_rate"

    if [ -f "$clk_path" ]; then
        cat "$clk_path" 2>/dev/null
    elif [ -f "$CLK_SUMMARY" ]; then
        # Fall back to parsing clk_summary
        grep -E "^\s*${clk_name}\s" "$CLK_SUMMARY" 2>/dev/null | awk '{print $4}'
    else
        echo "N/A"
    fi
}

# =============================================================================
# Helper: Get all fabric clocks as "afab:rate sfab:rate mmfab:rate"
# =============================================================================
get_all_fabric_clocks() {
    local afab=$(get_fabric_clock "afab_clk")
    local sfab=$(get_fabric_clock "sfab_clk")
    local mmfab=$(get_fabric_clock "mmfab_clk")

    # Convert to MHz for readability
    local afab_mhz="N/A"
    local sfab_mhz="N/A"
    local mmfab_mhz="N/A"

    if [ "$afab" != "N/A" ] && [ -n "$afab" ]; then
        afab_mhz=$((afab / 1000000))
    fi
    if [ "$sfab" != "N/A" ] && [ -n "$sfab" ]; then
        sfab_mhz=$((sfab / 1000000))
    fi
    if [ "$mmfab" != "N/A" ] && [ -n "$mmfab" ]; then
        mmfab_mhz=$((mmfab / 1000000))
    fi

    echo "${afab_mhz}:${sfab_mhz}:${mmfab_mhz}"
}

# =============================================================================
# Helper: Track clock rate and detect changes
# =============================================================================
track_clock_rate() {
    local rate="$1"
    local found=0

    # Check if we've seen this rate before
    for seen in $CLOCK_RATES_SEEN; do
        if [ "$seen" = "$rate" ]; then
            found=1
            break
        fi
    done

    if [ "$found" = "0" ]; then
        CLOCK_RATES_SEEN="$CLOCK_RATES_SEEN $rate"
    fi
}

# =============================================================================
# Helper: Check if fabric clocks are available
# =============================================================================
check_fabric_clocks() {
    if [ -f "$AFAB_CLK" ] || [ -f "$CLK_SUMMARY" ]; then
        echo "1"
    else
        echo "0"
    fi
}

echo "=============================================="
echo "  RAM Speed Benchmark for HP TouchPad"
echo "=============================================="
echo ""
echo "Iterations: $ITERATIONS"
echo "Memory test size: ${MEM_SIZE_MB} MB"
echo "tmpfs test size: ${TMPFS_SIZE_MB} MB"
echo "CPU busy-wait: $([ "$BUSY_WAIT" = "1" ] && echo "ENABLED" || echo "disabled")"

# Check for fabric clock monitoring
FABRIC_CLK_AVAIL=$(check_fabric_clocks)
if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
    echo "Fabric clock monitoring: ENABLED"
    INITIAL_CLOCKS=$(get_all_fabric_clocks)
    echo "Initial fabric clocks (AFAB:SFAB:MMFAB MHz): $INITIAL_CLOCKS"
else
    echo "Fabric clock monitoring: unavailable (debugfs not mounted?)"
fi
echo ""

# =============================================================================
# Helper: Get time from /proc/uptime (works on BusyBox)
# =============================================================================
get_uptime_ms() {
    awk '{printf "%.0f", $1 * 1000}' /proc/uptime
}

# =============================================================================
# 1. Set CPU to max frequency
# =============================================================================
echo "--- Setting CPU to Max Frequency ---"

CPU0_GOV="/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
CPU1_GOV="/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor"
CPU0_FREQ="/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
CPU0_MAX="/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"

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

    if [ -f "$CPU0_MAX" ]; then
        MAX_FREQ=$(cat "$CPU0_MAX" 2>/dev/null)
        MAX_FREQ_MHZ=$((MAX_FREQ / 1000))
        echo "Max frequency: ${MAX_FREQ_MHZ} MHz"
    fi
else
    echo "Warning: cpufreq not available"
fi

echo ""

# =============================================================================
# 2. Show system info
# =============================================================================
echo "--- System Info ---"
uname -r
if [ -f /proc/meminfo ]; then
    grep -E "MemTotal|MemFree|MemAvailable" /proc/meminfo
fi
echo ""

# =============================================================================
# 3. Memory bandwidth test (zero -> null)
# =============================================================================
echo "--- Memory Bandwidth Test (${MEM_SIZE_MB} MB, $ITERATIONS iterations) ---"
if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
    echo "    (showing AFAB clock rate before->after each run)"
fi
echo ""

MEM_TIMES=""
MEM_CLOCKS=""
for i in $(seq 1 $ITERATIONS); do
    # Keep CPU busy to prevent idle state transitions
    cpu_busy_wait

    # Capture clock rate before test
    if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
        CLK_BEFORE=$(get_all_fabric_clocks)
    fi

    # Sync and drop caches
    sync
    echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

    # Run test and capture time using /proc/uptime
    START=$(get_uptime_ms)
    dd if=/dev/zero of=/dev/null bs=1M count=$MEM_SIZE_MB 2>/dev/null
    END=$(get_uptime_ms)

    # Capture clock rate after test
    if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
        CLK_AFTER=$(get_all_fabric_clocks)
        track_clock_rate "$CLK_BEFORE"
        track_clock_rate "$CLK_AFTER"

        # Check for clock change during test
        if [ "$CLK_BEFORE" != "$CLK_AFTER" ]; then
            CLK_CHANGED="*"
            CLOCK_CHANGES=$((CLOCK_CHANGES + 1))
        else
            CLK_CHANGED=""
        fi
    fi

    # Calculate elapsed time in seconds
    ELAPSED_MS=$((END - START))
    ELAPSED=$(awk -v ms=$ELAPSED_MS 'BEGIN {printf "%.3f", ms / 1000}')
    SPEED=$(awk -v ms=$ELAPSED_MS -v size=$MEM_SIZE_MB 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

    if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
        # Extract just AFAB rate for compact display
        AFAB_BEFORE=$(echo "$CLK_BEFORE" | cut -d: -f1)
        AFAB_AFTER=$(echo "$CLK_AFTER" | cut -d: -f1)
        printf "  Run %2d: %s s (%s MB/s)  AFAB: %s->%s MHz%s\n" "$i" "$ELAPSED" "$SPEED" "$AFAB_BEFORE" "$AFAB_AFTER" "$CLK_CHANGED"
    else
        printf "  Run %2d: %s s (%s MB/s)\n" "$i" "$ELAPSED" "$SPEED"
    fi

    if [ -z "$MEM_TIMES" ]; then
        MEM_TIMES="$ELAPSED"
    else
        MEM_TIMES="$MEM_TIMES $ELAPSED"
    fi
done

# Calculate statistics
echo ""
echo "$MEM_TIMES" | awk -v size=$MEM_SIZE_MB '
{
    n = NF
    min = $1; max = $1; sum = 0
    for (i = 1; i <= NF; i++) {
        if ($i < min) min = $i
        if ($i > max) max = $i
        sum += $i
    }
    avg = sum / n

    # Calculate speeds
    min_speed = size / max
    max_speed = size / min
    avg_speed = size / avg

    printf "  Min time:  %.3f s (%.1f MB/s)\n", min, max_speed
    printf "  Max time:  %.3f s (%.1f MB/s)\n", max, min_speed
    printf "  Avg time:  %.3f s (%.1f MB/s)\n", avg, avg_speed
}'

echo ""

# =============================================================================
# 4. tmpfs write test
# =============================================================================
echo "--- tmpfs Write Test (${TMPFS_SIZE_MB} MB, $ITERATIONS iterations) ---"
echo ""

# Check tmpfs space
if [ -d /tmp ]; then
    TMPFS_AVAIL=$(df /tmp 2>/dev/null | tail -1 | awk '{print int($4/1024)}')
    echo "  tmpfs available: ${TMPFS_AVAIL} MB"
    if [ "$TMPFS_AVAIL" -lt "$TMPFS_SIZE_MB" ]; then
        echo "  Warning: Not enough tmpfs space, reducing test size"
        TMPFS_SIZE_MB=$((TMPFS_AVAIL - 10))
        if [ "$TMPFS_SIZE_MB" -lt 10 ]; then
            echo "  Error: tmpfs too small for test"
            TMPFS_SIZE_MB=0
        fi
    fi
fi
echo ""

WRITE_TIMES=""
if [ "$TMPFS_SIZE_MB" -gt 0 ]; then
    for i in $(seq 1 $ITERATIONS); do
        # Keep CPU busy to prevent idle state transitions
        cpu_busy_wait

        # Capture clock rate before test
        if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
            CLK_BEFORE=$(get_all_fabric_clocks)
        fi

        # Clean up and sync
        rm -f "$TMPFS_FILE" 2>/dev/null
        sync
        echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

        # Run test
        START=$(get_uptime_ms)
        dd if=/dev/zero of="$TMPFS_FILE" bs=1M count=$TMPFS_SIZE_MB 2>/dev/null
        sync
        END=$(get_uptime_ms)

        # Capture clock rate after test
        if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
            CLK_AFTER=$(get_all_fabric_clocks)
            track_clock_rate "$CLK_BEFORE"
            track_clock_rate "$CLK_AFTER"

            if [ "$CLK_BEFORE" != "$CLK_AFTER" ]; then
                CLK_CHANGED="*"
                CLOCK_CHANGES=$((CLOCK_CHANGES + 1))
            else
                CLK_CHANGED=""
            fi
        fi

        ELAPSED_MS=$((END - START))
        ELAPSED=$(awk -v ms=$ELAPSED_MS 'BEGIN {printf "%.3f", ms / 1000}')
        SPEED=$(awk -v ms=$ELAPSED_MS -v size=$TMPFS_SIZE_MB 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

        if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
            AFAB_BEFORE=$(echo "$CLK_BEFORE" | cut -d: -f1)
            AFAB_AFTER=$(echo "$CLK_AFTER" | cut -d: -f1)
            printf "  Run %2d: %s s (%s MB/s)  AFAB: %s->%s MHz%s\n" "$i" "$ELAPSED" "$SPEED" "$AFAB_BEFORE" "$AFAB_AFTER" "$CLK_CHANGED"
        else
            printf "  Run %2d: %s s (%s MB/s)\n" "$i" "$ELAPSED" "$SPEED"
        fi

        if [ -z "$WRITE_TIMES" ]; then
            WRITE_TIMES="$ELAPSED"
        else
            WRITE_TIMES="$WRITE_TIMES $ELAPSED"
        fi
    done

    echo ""
    echo "$WRITE_TIMES" | awk -v size=$TMPFS_SIZE_MB '
    {
        n = NF
        min = $1; max = $1; sum = 0
        for (i = 1; i <= NF; i++) {
            if ($i < min) min = $i
            if ($i > max) max = $i
            sum += $i
        }
        avg = sum / n

        min_speed = size / max
        max_speed = size / min
        avg_speed = size / avg

        printf "  Min time:  %.3f s (%.1f MB/s)\n", min, max_speed
        printf "  Max time:  %.3f s (%.1f MB/s)\n", max, min_speed
        printf "  Avg time:  %.3f s (%.1f MB/s)\n", avg, avg_speed
    }'
fi

echo ""

# =============================================================================
# 5. tmpfs read test
# =============================================================================
echo "--- tmpfs Read Test (${TMPFS_SIZE_MB} MB, $ITERATIONS iterations) ---"
echo ""

READ_TIMES=""
if [ "$TMPFS_SIZE_MB" -gt 0 ] && [ -f "$TMPFS_FILE" ]; then
    for i in $(seq 1 $ITERATIONS); do
        # Keep CPU busy to prevent idle state transitions
        cpu_busy_wait

        # Capture clock rate before test
        if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
            CLK_BEFORE=$(get_all_fabric_clocks)
        fi

        # Sync and drop caches
        sync
        echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

        # Run test
        START=$(get_uptime_ms)
        dd if="$TMPFS_FILE" of=/dev/null bs=1M 2>/dev/null
        END=$(get_uptime_ms)

        # Capture clock rate after test
        if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
            CLK_AFTER=$(get_all_fabric_clocks)
            track_clock_rate "$CLK_BEFORE"
            track_clock_rate "$CLK_AFTER"

            if [ "$CLK_BEFORE" != "$CLK_AFTER" ]; then
                CLK_CHANGED="*"
                CLOCK_CHANGES=$((CLOCK_CHANGES + 1))
            else
                CLK_CHANGED=""
            fi
        fi

        ELAPSED_MS=$((END - START))
        ELAPSED=$(awk -v ms=$ELAPSED_MS 'BEGIN {printf "%.3f", ms / 1000}')
        SPEED=$(awk -v ms=$ELAPSED_MS -v size=$TMPFS_SIZE_MB 'BEGIN {if (ms > 0) printf "%.1f", size * 1000 / ms; else print "N/A"}')

        if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
            AFAB_BEFORE=$(echo "$CLK_BEFORE" | cut -d: -f1)
            AFAB_AFTER=$(echo "$CLK_AFTER" | cut -d: -f1)
            printf "  Run %2d: %s s (%s MB/s)  AFAB: %s->%s MHz%s\n" "$i" "$ELAPSED" "$SPEED" "$AFAB_BEFORE" "$AFAB_AFTER" "$CLK_CHANGED"
        else
            printf "  Run %2d: %s s (%s MB/s)\n" "$i" "$ELAPSED" "$SPEED"
        fi

        if [ -z "$READ_TIMES" ]; then
            READ_TIMES="$ELAPSED"
        else
            READ_TIMES="$READ_TIMES $ELAPSED"
        fi
    done

    echo ""
    echo "$READ_TIMES" | awk -v size=$TMPFS_SIZE_MB '
    {
        n = NF
        min = $1; max = $1; sum = 0
        for (i = 1; i <= NF; i++) {
            if ($i < min) min = $i
            if ($i > max) max = $i
            sum += $i
        }
        avg = sum / n

        min_speed = size / max
        max_speed = size / min
        avg_speed = size / avg

        printf "  Min time:  %.3f s (%.1f MB/s)\n", min, max_speed
        printf "  Max time:  %.3f s (%.1f MB/s)\n", max, min_speed
        printf "  Avg time:  %.3f s (%.1f MB/s)\n", avg, avg_speed
    }'
else
    echo "  Skipped (no tmpfs file available)"
fi

echo ""

# =============================================================================
# 6. Cleanup
# =============================================================================
echo "--- Cleanup ---"
rm -f "$TMPFS_FILE" 2>/dev/null
echo "Removed $TMPFS_FILE"

# Restore governor if we changed it
if [ -n "$OLD_GOV" ] && [ "$OLD_GOV" != "performance" ]; then
    echo "Restoring governor to $OLD_GOV"
    echo "$OLD_GOV" > "$CPU0_GOV" 2>/dev/null
    if [ -f "$CPU1_GOV" ]; then
        echo "$OLD_GOV" > "$CPU1_GOV" 2>/dev/null
    fi
fi

# =============================================================================
# 7. Fabric Clock Analysis
# =============================================================================
if [ "$FABRIC_CLK_AVAIL" = "1" ]; then
    echo ""
    echo "--- Fabric Clock Analysis ---"
    FINAL_CLOCKS=$(get_all_fabric_clocks)
    echo "Final fabric clocks (AFAB:SFAB:MMFAB MHz): $FINAL_CLOCKS"
    echo "Clock changes detected during tests: $CLOCK_CHANGES"

    # Count unique clock rates seen
    UNIQUE_RATES=$(echo "$CLOCK_RATES_SEEN" | tr ' ' '\n' | sort -u | grep -v '^$' | wc -l)
    echo "Unique clock rate combinations seen: $UNIQUE_RATES"

    if [ "$UNIQUE_RATES" -gt 1 ]; then
        echo ""
        echo "*** BIMODALITY DETECTED ***"
        echo "Multiple fabric clock rates observed during testing."
        echo "Clock rates seen (AFAB:SFAB:MMFAB MHz):"
        echo "$CLOCK_RATES_SEEN" | tr ' ' '\n' | sort -u | grep -v '^$' | while read rate; do
            echo "  - $rate"
        done
        echo ""
        echo "This suggests the interconnect driver is dynamically scaling"
        echo "fabric clocks, which may cause bimodal memory performance."
        echo "See: drivers/interconnect/qcom/msm8660.c"
    else
        echo "Clock rates stable - fabric clocks are not causing bimodality."
    fi
fi

echo ""
echo "=============================================="
echo "  Benchmark Complete"
echo "=============================================="
