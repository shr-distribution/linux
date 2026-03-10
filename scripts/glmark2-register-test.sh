#!/bin/bash
#
# glmark2 Register Comparison Test Script (Build-only version)
#
# Runs 25 iterations of glmark2 build benchmark and dumps A2XX GPU
# registers before and after each run. Results are saved immediately
# to persistent storage after each iteration.
#
# Usage: ./glmark2-register-test.sh [output_dir]

ITERATIONS=25
OUTPUT_DIR="${1:-/media/internal/regs-test}"
DEBUGFS_REGS="/sys/kernel/debug/a2xx_regs"
BENCHMARK="build"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "============================================"
echo "glmark2 A2XX Register Test (Build only)"
echo "============================================"
echo "Iterations: $ITERATIONS"
echo "Benchmark:  $BENCHMARK"
echo "Output dir: $OUTPUT_DIR"
echo "Timestamp:  $TIMESTAMP"
echo "============================================"

# Check requirements
if [ ! -f "$DEBUGFS_REGS" ]; then
    echo "Error: Debugfs register file not found: $DEBUGFS_REGS"
    echo "Make sure kernel has A2XX debugfs support and debugfs is mounted"
    exit 1
fi

if ! command -v glmark2-es2-drm &> /dev/null; then
    echo "Error: glmark2-es2-drm not found"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Initialize result files with headers
REGLOG="$OUTPUT_DIR/registers_${TIMESTAMP}.log"
RESULTS="$OUTPUT_DIR/results_${TIMESTAMP}.csv"
SUMMARY="$OUTPUT_DIR/summary_${TIMESTAMP}.txt"

echo "# Register log started at $(date)" > "$REGLOG"
echo "iteration,benchmark,fps,visual_status,sq_gpr_mgmt,sq_interpolator,rbbm_pm_ovr1,rbbm_pm_ovr2,rbbm_status" > "$RESULTS"

# Function to dump registers and extract values
dump_regs() {
    local iteration="$1"
    local phase="$2"
    local regfile="$OUTPUT_DIR/regs_iter${iteration}_${phase}.txt"

    # Read registers
    cat "$DEBUGFS_REGS" > "$regfile" 2>&1

    # Extract key values
    local sq_gpr=$(grep "SQ_GPR_MANAGEMENT" "$regfile" | awk '{print $NF}')
    local sq_interp=$(grep "SQ_INTERPOLATOR" "$regfile" 2>/dev/null | awk '{print $NF}')
    local pm_ovr1=$(grep "RBBM_PM_OVERRIDE1" "$regfile" | awk '{print $NF}')
    local pm_ovr2=$(grep "RBBM_PM_OVERRIDE2" "$regfile" | awk '{print $NF}')
    local status=$(grep "RBBM_STATUS" "$regfile" | awk '{print $NF}')

    # Log to register log
    echo "=== Iter $iteration $phase $(date) ===" >> "$REGLOG"
    cat "$regfile" >> "$REGLOG"
    echo "" >> "$REGLOG"

    # Sync to disk immediately
    sync

    # Return values for later use
    echo "$sq_gpr,$sq_interp,$pm_ovr1,$pm_ovr2,$status"
}

# Main test loop
echo ""
echo "Starting test at $(date)"
echo ""

for i in $(seq 1 $ITERATIONS); do
    echo "============================================"
    echo "Iteration $i of $ITERATIONS - $BENCHMARK benchmark"
    echo "============================================"

    # Dump registers BEFORE
    echo "Dumping registers before run..."
    regs_before=$(dump_regs "$i" "before")

    # Run the benchmark
    echo "Running glmark2 $BENCHMARK..."
    result_file="$OUTPUT_DIR/glmark_iter${i}.txt"

    timeout 120 glmark2-es2-drm --benchmark "$BENCHMARK" 2>&1 | tee "$result_file"
    exit_code=$?

    # Dump registers AFTER
    echo "Dumping registers after run..."
    regs_after=$(dump_regs "$i" "after")

    # Extract FPS
    fps=$(grep '\[build\]' "$result_file" | grep 'FPS:' | sed 's/.*FPS: *\([0-9]*\).*/\1/' | tail -1)
    if [ -z "$fps" ]; then
        fps="FAIL"
    fi

    # Ask user for visual status (or default to unknown)
    echo ""
    echo ">>> Iteration $i completed: FPS=$fps"
    echo ">>> Was the visual output GOOD (no artifacts) or BAD (artifacts)?"
    echo ">>> Press 'g' for GOOD, 'b' for BAD, or Enter to skip: "

    # Read with timeout
    read -t 10 -n 1 visual_input
    echo ""

    case "$visual_input" in
        g|G) visual_status="GOOD" ;;
        b|B) visual_status="BAD" ;;
        *) visual_status="UNKNOWN" ;;
    esac

    # Log result immediately
    echo "$i,$BENCHMARK,$fps,$visual_status,$regs_after" >> "$RESULTS"
    sync

    echo "Result: FPS=$fps Visual=$visual_status"
    echo "Registers: $regs_after"
    echo ""

    # Small delay between iterations
    sleep 2
done

# Generate summary
echo "============================================" | tee "$SUMMARY"
echo "Test Summary" | tee -a "$SUMMARY"
echo "============================================" | tee -a "$SUMMARY"
echo "Completed: $ITERATIONS iterations" | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"

echo "Results by visual status:" | tee -a "$SUMMARY"
echo "GOOD runs:" | tee -a "$SUMMARY"
grep ",GOOD," "$RESULTS" | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"
echo "BAD runs:" | tee -a "$SUMMARY"
grep ",BAD," "$RESULTS" | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"

echo "Register value frequency:" | tee -a "$SUMMARY"
echo "SQ_GPR_MANAGEMENT values:" | tee -a "$SUMMARY"
cut -d',' -f5 "$RESULTS" | tail -n +2 | sort | uniq -c | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"

echo "============================================" | tee -a "$SUMMARY"
echo "Test completed at $(date)" | tee -a "$SUMMARY"
echo "============================================" | tee -a "$SUMMARY"

sync
echo ""
echo "Results saved to $OUTPUT_DIR"
echo "To analyze: cat $RESULTS"
