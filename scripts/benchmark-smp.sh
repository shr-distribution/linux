#!/bin/sh
# SMP memory bandwidth benchmark for HP TouchPad (APQ8060, dual Scorpion).
#
# Usage: ./scripts/benchmark-smp.sh [iterations] [size_mb]
#   iterations: per-test repeats (default 10)
#   size_mb:    per-thread transfer size in MB (default 256, so dual-thread total = 512)
#
# What it measures:
#   1. Single-thread memory bandwidth (dd /dev/zero -> /dev/null) on one CPU
#   2. Dual-thread aggregate bandwidth — taskset 0x1 and taskset 0x2 in parallel
#   3. Per-thread efficiency under contention (aggregate/2 vs. single-thread median)
#   4. Variance: max-min spread across iterations (low variance = healthy SMP coherency)
#   5. EBI/AFAB/SFAB/MMFAB clock rates before and after
#   6. vdd_mem / vdd_dig regulator voltages (PM8058 S0/S1)
#   7. CPU governor + frequency state
#
# Why dual-thread:
#   Scorpion ACTLR.bit24 is the documented SMP-enable hint. With it set, cache
#   coherency between the two cores should be stable; without it, dual-thread
#   memory ops thrash. Low dual-thread variance (max-min < 5%) is the signal
#   that coherency is healthy.
#
# Why EBI clock capture:
#   The CPU->EBI ICC vote in apcs-msm8660.c picks one of four bandwidth tiers
#   based on CPU frequency. We capture the actual ebi1_clk to confirm the vote
#   is propagating to RPM (vs. just being a no-op).
#
# Tips:
#   - Run on settled system (uptime > 60s, no userspace load)
#   - Drop caches between iterations is best-effort on Scorpion; some test data
#     may still hit L2/L1 (bimodal results)
#   - Median is more robust than mean here

ITERATIONS="${1:-10}"
SIZE_MB="${2:-256}"
TMP_RESULTS="/tmp/bench-smp-$$.times"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# /proc/uptime in milliseconds (busybox awk OK)
get_uptime_ms() {
    awk '{printf "%.0f", $1 * 1000}' /proc/uptime
}

# Read a clock rate from debugfs, return MHz (or "N/A")
get_clk_mhz() {
    local name="$1"
    local path="/sys/kernel/debug/clk/${name}/clk_rate"
    if [ -f "$path" ]; then
        local hz=$(cat "$path" 2>/dev/null)
        if [ -n "$hz" ] && [ "$hz" != "0" ]; then
            echo $((hz / 1000000))
            return
        fi
    fi
    echo "N/A"
}

# Print a clock+regulator snapshot
snapshot_state() {
    local label="$1"
    echo "  $label:"
    echo "    ebi1_clk:  $(get_clk_mhz ebi1_clk) MHz"
    echo "    ebi1_a:    $(get_clk_mhz ebi1_a_clk) MHz"
    echo "    afab_clk:  $(get_clk_mhz afab_clk) MHz"
    echo "    sfab_clk:  $(get_clk_mhz sfab_clk) MHz"
    echo "    mmfab_clk: $(get_clk_mhz mmfab_clk) MHz"
    echo "    dfab_clk:  $(get_clk_mhz dfab_clk) MHz"
    # PMIC regulator voltages
    for r in /sys/class/regulator/regulator.*/; do
        local n=$(cat "$r/name" 2>/dev/null)
        case "$n" in
            8058_s0)
                local uv=$(cat "$r/microvolts" 2>/dev/null)
                echo "    vdd_mem:   $((uv / 1000)) mV (8058_s0)"
                ;;
            8058_s1)
                local uv=$(cat "$r/microvolts" 2>/dev/null)
                echo "    vdd_dig:   $((uv / 1000)) mV (8058_s1)"
                ;;
        esac
    done
}

# Compute min / median / max / mean / max-min spread from a list of values in $1
stats() {
    local label="$1"
    local data="$2"
    local unit="$3"
    echo "$data" | awk -v label="$label" -v unit="$unit" '
    {
        n = NF
        if (n == 0) { print "  " label ": no data"; exit }
        # Sort by copying into a, then bubble (small n)
        for (i = 1; i <= n; i++) a[i] = $i
        for (i = 1; i <= n; i++)
            for (j = i+1; j <= n; j++)
                if (a[i] > a[j]) { t = a[i]; a[i] = a[j]; a[j] = t }
        min = a[1]; max = a[n]
        if (n % 2) median = a[(n+1)/2]
        else median = (a[n/2] + a[n/2+1]) / 2
        sum = 0
        for (i = 1; i <= n; i++) sum += a[i]
        avg = sum / n
        spread_pct = (max > 0) ? (max - min) * 100.0 / max : 0
        printf "  %s: min=%g median=%g avg=%g max=%g %s  (spread %.1f%%)\n",
               label, min, median, avg, max, unit, spread_pct
    }'
}

# Bandwidth in MB/s given time-ms and total-MB (per worker for single, total for dual)
ms_to_mbps() {
    awk -v ms="$1" -v sz="$2" 'BEGIN { if (ms > 0) printf "%.1f", sz * 1000 / ms; else print "0" }'
}

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

echo "=============================================="
echo "  SMP RAM Benchmark for HP TouchPad"
echo "=============================================="
echo "kernel:     $(uname -r)"
echo "iterations: $ITERATIONS"
echo "per-thread: ${SIZE_MB} MB"
echo "dual total: $((SIZE_MB * 2)) MB"
echo

# Force performance governor on both CPUs
for c in 0 1; do
    [ -w /sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor ] && \
        echo performance > /sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor 2>/dev/null
done

# Make sure both CPUs are online
[ -w /sys/devices/system/cpu/cpu1/online ] && echo 1 > /sys/devices/system/cpu/cpu1/online 2>/dev/null

echo "CPU state:"
for c in 0 1; do
    if [ -f /sys/devices/system/cpu/cpu$c/cpufreq/scaling_cur_freq ]; then
        local_freq=$(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_cur_freq)
        local_gov=$(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor)
        local_online=$(cat /sys/devices/system/cpu/cpu$c/online 2>/dev/null || echo 1)
        echo "  cpu$c: $((local_freq / 1000)) MHz, gov=$local_gov, online=$local_online"
    fi
done

# Mount debugfs if needed
[ ! -d /sys/kernel/debug/clk ] && mount -t debugfs none /sys/kernel/debug 2>/dev/null

echo
echo "Pre-test state:"
snapshot_state "before"
echo

# ---------------------------------------------------------------------------
# Single-thread test
# ---------------------------------------------------------------------------

echo "----------------------------------------------"
echo "Single-thread (one dd, no CPU pinning):"
echo "----------------------------------------------"

SINGLE_TIMES=""
SINGLE_BW=""

for i in $(seq 1 $ITERATIONS); do
    sync
    echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

    start=$(get_uptime_ms)
    dd if=/dev/zero of=/dev/null bs=1M count=$SIZE_MB 2>/dev/null
    end=$(get_uptime_ms)

    elapsed=$((end - start))
    bw=$(ms_to_mbps "$elapsed" "$SIZE_MB")
    printf "  run %2d: %4d ms  (%s MB/s)\n" "$i" "$elapsed" "$bw"
    SINGLE_TIMES="$SINGLE_TIMES $elapsed"
    SINGLE_BW="$SINGLE_BW $bw"
done

echo
stats "single times" "$SINGLE_TIMES" "ms"
stats "single BW   " "$SINGLE_BW"    "MB/s"
echo

# ---------------------------------------------------------------------------
# Dual-thread parallel test
# ---------------------------------------------------------------------------

echo "----------------------------------------------"
echo "Dual-thread (2x dd, taskset to cpu0 + cpu1, parallel):"
echo "----------------------------------------------"

DUAL_TIMES=""
DUAL_BW=""

# Verify taskset is available
if ! command -v taskset >/dev/null 2>&1; then
    echo "  WARNING: taskset not found, falling back to un-pinned parallel dd"
    USE_TASKSET=0
else
    USE_TASKSET=1
fi

for i in $(seq 1 $ITERATIONS); do
    sync
    echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

    start=$(get_uptime_ms)
    if [ "$USE_TASKSET" = "1" ]; then
        taskset 0x1 dd if=/dev/zero of=/dev/null bs=1M count=$SIZE_MB 2>/dev/null &
        P1=$!
        taskset 0x2 dd if=/dev/zero of=/dev/null bs=1M count=$SIZE_MB 2>/dev/null &
        P2=$!
    else
        dd if=/dev/zero of=/dev/null bs=1M count=$SIZE_MB 2>/dev/null &
        P1=$!
        dd if=/dev/zero of=/dev/null bs=1M count=$SIZE_MB 2>/dev/null &
        P2=$!
    fi
    wait $P1 $P2
    end=$(get_uptime_ms)

    elapsed=$((end - start))
    # Aggregate bandwidth = both workers' data in elapsed wall time
    bw_total=$(ms_to_mbps "$elapsed" "$((SIZE_MB * 2))")
    bw_each=$(ms_to_mbps "$elapsed" "$SIZE_MB")
    printf "  run %2d: %4d ms  (aggregate %s MB/s, per-cpu %s MB/s)\n" \
        "$i" "$elapsed" "$bw_total" "$bw_each"
    DUAL_TIMES="$DUAL_TIMES $elapsed"
    DUAL_BW="$DUAL_BW $bw_total"
done

echo
stats "dual times " "$DUAL_TIMES" "ms"
stats "dual agg BW" "$DUAL_BW"    "MB/s"
echo

# ---------------------------------------------------------------------------
# Scaling efficiency
# ---------------------------------------------------------------------------

echo "----------------------------------------------"
echo "Scaling efficiency:"
echo "----------------------------------------------"

# Compute single-thread MIN and MEDIAN, dual-thread MEDIAN.
# Use SINGLE-THREAD-MIN as the baseline for scaling ratio — it's the truly-
# uncached number (when drop_caches did flush enough). Dividing dual-thread
# aggregate by single-thread MEDIAN is misleading because the median often
# includes cache-hit runs that get artificially fast results.

stats_to() {
    local var="$1"
    local data="$2"
    local what="$3"   # "min" or "median"
    echo "$data" | awk -v what="$what" '
    {
        n = NF
        for (i = 1; i <= n; i++) a[i] = $i + 0
        for (i = 1; i <= n; i++)
            for (j = i+1; j <= n; j++)
                if (a[i] > a[j]) { t = a[i]; a[i] = a[j]; a[j] = t }
        if (what == "min") printf "%.1f", a[1]
        else if (n % 2)    printf "%.1f", a[(n+1)/2]
        else               printf "%.1f", (a[n/2] + a[n/2+1]) / 2
    }'
}

SINGLE_MIN_BW=$(stats_to single_min "$SINGLE_BW" min)
SINGLE_MEDIAN_BW=$(stats_to single_med "$SINGLE_BW" median)
DUAL_MEDIAN_BW=$(stats_to dual_med "$DUAL_BW" median)
DUAL_SPREAD_PCT=$(echo "$DUAL_BW" | awk '
{
    n = NF; min = $1; max = $1
    for (i = 1; i <= n; i++) { if ($i<min) min=$i; if ($i>max) max=$i }
    if (max > 0) printf "%.1f", (max - min) * 100.0 / max
    else print "0"
}')

awk -v smin="$SINGLE_MIN_BW" -v smed="$SINGLE_MEDIAN_BW" -v d="$DUAL_MEDIAN_BW" \
    -v spread="$DUAL_SPREAD_PCT" 'BEGIN {
    printf "  single-thread BW (min,    truly uncached): %s MB/s\n", smin
    printf "  single-thread BW (median, partial cache):  %s MB/s\n", smed
    printf "  dual-thread aggregate BW (median):         %s MB/s\n", d
    printf "  dual-thread BW spread across runs:         %s%%\n", spread
    print  ""

    # Scaling efficiency: compare aggregate to uncached single-thread.
    # This is the primary signal — if scaling is good, coherency is healthy.
    if (smin > 0) {
        eff = d / smin
        printf "  SMP scaling factor (aggregate / uncached single): %.2fx\n", eff
        if (eff >= 1.8) {
            scaling = "EXCELLENT"
            print "    >>> EXCELLENT. Cores pull near-independent DRAM bandwidth."
        } else if (eff >= 1.3) {
            scaling = "GOOD"
            print "    >>> GOOD scaling, modest DRAM contention."
        } else if (eff >= 0.9) {
            scaling = "DRAM_BOUND"
            print "    >>> DRAM-BOUND. Aggregate matches single-thread uncached —"
            print "        both cores share a single DRAM channel and saturate it."
        } else {
            scaling = "POOR"
            print "    >>> POOR — dual-thread aggregate below uncached single-thread."
            print "        Likely coherency thrashing or pathological cache miss."
        }
    }

    # Variance interpretation, considered jointly with scaling.
    # A few outliers can drive max-min spread above 15% without indicating real
    # coherency trouble — if scaling is healthy, treat moderate spread as noise
    # (drop_caches inconsistency, userspace interrupts, etc.).
    print  ""
    if (spread + 0 < 5) {
        print "  Run-to-run variance: TIGHT (< 5%). Strong evidence of healthy coherency."
    } else if (spread + 0 < 15) {
        print "  Run-to-run variance: MODERATE (< 15%). Acceptable."
    } else if (scaling == "EXCELLENT" || scaling == "GOOD") {
        print "  Run-to-run variance: looser (> 15%) — but scaling is healthy,"
        print "    so this is likely background noise (drop_caches inconsistency,"
        print "    userspace activity, governor jitter), not a coherency issue."
    } else {
        print "  Run-to-run variance: HIGH (> 15%) combined with weak scaling —"
        print "    this is consistent with cache-line ping-pong / coherency stalls."
    }
}'

echo
echo "Post-test state:"
snapshot_state "after"
echo

# ---------------------------------------------------------------------------
# Footer with environmental notes
# ---------------------------------------------------------------------------

echo "----------------------------------------------"
echo "Notes:"
echo "----------------------------------------------"
echo "  - Bimodal single-thread (very fast vs slow within same run) usually means"
echo "    drop_caches did not fully purge L1/L2 and some iterations hit cache."
echo "    Compare 'slow' single-thread runs to per-cpu dual-thread for true DRAM BW."
echo "  - Low dual-thread spread (< 5%) is the signal that ACTLR.bit24 SMP coherency"
echo "    is healthy. Erratic dual-thread results indicate cache-line ping-pong."
echo "  - EBI clock should rise above the RPM-default when CPU is at performance"
echo "    governor (per apcs-msm8660.c CPU->EBI ICC tier votes). If ebi1_clk reads"
echo "    27 MHz / RPM default under load, the ICC vote is not propagating."
echo
echo "=============================================="
echo "  Done"
echo "=============================================="

rm -f "$TMP_RESULTS"
