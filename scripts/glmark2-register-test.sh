#!/bin/bash
#
# glmark2 Register Comparison Test Script
#
# Runs 25 iterations of glmark2 benchmarks (build, texture, shading)
# and dumps A2XX GPU registers before and after each run.
# This helps identify register value differences between successful
# renders and renders with artifacts.
#
# Usage: ./glmark2-register-test.sh [output_dir]
#
# Requirements:
# - Kernel with debugfs A2XX register dump support
# - glmark2-es2-drm installed
# - Root access for debugfs

ITERATIONS=25
OUTPUT_DIR="${1:-/tmp/glmark2-regs}"
DEBUGFS_REGS="/sys/kernel/debug/dri/0/a2xx_regs"
BENCHMARKS="build texture shading"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "============================================"
echo "glmark2 A2XX Register Comparison Test"
echo "============================================"
echo "Iterations: $ITERATIONS"
echo "Benchmarks: $BENCHMARKS"
echo "Output dir: $OUTPUT_DIR"
echo "Timestamp:  $TIMESTAMP"
echo "============================================"

# Check requirements
if [ ! -f "$DEBUGFS_REGS" ]; then
    echo -e "${RED}Error: Debugfs register file not found: $DEBUGFS_REGS${NC}"
    echo "Make sure kernel has A2XX debugfs support and debugfs is mounted"
    exit 1
fi

if ! command -v glmark2-es2-drm &> /dev/null; then
    echo -e "${RED}Error: glmark2-es2-drm not found${NC}"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"
LOGFILE="$OUTPUT_DIR/test_${TIMESTAMP}.log"
SUMMARY="$OUTPUT_DIR/summary_${TIMESTAMP}.txt"

# Function to dump registers
dump_regs() {
    local prefix="$1"
    local iteration="$2"
    local benchmark="$3"
    local phase="$4"
    local outfile="$OUTPUT_DIR/${prefix}_iter${iteration}_${benchmark}_${phase}.txt"

    echo "Dumping registers to: $outfile"
    cat "$DEBUGFS_REGS" > "$outfile" 2>&1

    # Also extract key values for quick comparison
    local sq_gpr=$(grep "SQ_GPR_MANAGEMENT" "$outfile" | awk '{print $NF}')
    local sq_inst=$(grep "SQ_INST_STORE_MANAGMENT" "$outfile" | awk '{print $NF}')
    local pm_ovr1=$(grep "RBBM_PM_OVERRIDE1" "$outfile" | awk '{print $NF}')
    local pm_ovr2=$(grep "RBBM_PM_OVERRIDE2" "$outfile" | awk '{print $NF}')
    local rbbm_status=$(grep "RBBM_STATUS" "$outfile" | awk '{print $NF}')

    echo "$iteration,$benchmark,$phase,$sq_gpr,$sq_inst,$pm_ovr1,$pm_ovr2,$rbbm_status" >> "$OUTPUT_DIR/registers_${TIMESTAMP}.csv"
}

# Function to run a single benchmark
run_benchmark() {
    local iteration="$1"
    local benchmark="$2"
    local result_file="$OUTPUT_DIR/glmark_iter${iteration}_${benchmark}.txt"

    echo -e "${YELLOW}Running benchmark: $benchmark (iteration $iteration)${NC}"

    # Dump registers BEFORE
    dump_regs "regs" "$iteration" "$benchmark" "before"

    # Run the benchmark
    # Use --run-forever with timeout to get consistent behavior
    timeout 60 glmark2-es2-drm \
        --benchmark "$benchmark" \
        --off-screen \
        2>&1 | tee "$result_file"

    local exit_code=$?

    # Dump registers AFTER
    dump_regs "regs" "$iteration" "$benchmark" "after"

    # Extract FPS and check for success
    local fps=$(grep -oP 'FPS:\s*\K[0-9]+' "$result_file" | tail -1)
    local score=$(grep -oP 'Score:\s*\K[0-9]+' "$result_file" | tail -1)

    if [ -z "$fps" ]; then
        fps="FAIL"
    fi

    echo "$iteration,$benchmark,$fps,$score,$exit_code" >> "$OUTPUT_DIR/results_${TIMESTAMP}.csv"

    # Visual indicator
    if [ "$fps" != "FAIL" ] && [ "$fps" -gt 5 ]; then
        echo -e "${GREEN}  Result: FPS=$fps Score=$score${NC}"
        return 0
    else
        echo -e "${RED}  Result: FPS=$fps Score=$score (possible artifacts)${NC}"
        return 1
    fi
}

# Initialize CSV headers
echo "iteration,benchmark,phase,sq_gpr_mgmt,sq_inst_store,pm_override1,pm_override2,rbbm_status" > "$OUTPUT_DIR/registers_${TIMESTAMP}.csv"
echo "iteration,benchmark,fps,score,exit_code" > "$OUTPUT_DIR/results_${TIMESTAMP}.csv"

# Main test loop
success_count=0
fail_count=0

echo "" | tee -a "$LOGFILE"
echo "Starting test at $(date)" | tee -a "$LOGFILE"
echo "" | tee -a "$LOGFILE"

for i in $(seq 1 $ITERATIONS); do
    echo "============================================" | tee -a "$LOGFILE"
    echo "Iteration $i of $ITERATIONS" | tee -a "$LOGFILE"
    echo "============================================" | tee -a "$LOGFILE"

    for bench in $BENCHMARKS; do
        if run_benchmark "$i" "$bench" 2>&1 | tee -a "$LOGFILE"; then
            ((success_count++))
        else
            ((fail_count++))
        fi

        # Small delay between benchmarks
        sleep 2
    done

    echo "" | tee -a "$LOGFILE"
done

# Generate summary
echo "============================================" | tee "$SUMMARY"
echo "Test Summary" | tee -a "$SUMMARY"
echo "============================================" | tee -a "$SUMMARY"
echo "Total runs: $((ITERATIONS * 3))" | tee -a "$SUMMARY"
echo "Successful: $success_count" | tee -a "$SUMMARY"
echo "Failed/Artifacts: $fail_count" | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"
echo "Output files:" | tee -a "$SUMMARY"
echo "  Register dumps: $OUTPUT_DIR/regs_iter*_*.txt" | tee -a "$SUMMARY"
echo "  Register CSV:   $OUTPUT_DIR/registers_${TIMESTAMP}.csv" | tee -a "$SUMMARY"
echo "  Results CSV:    $OUTPUT_DIR/results_${TIMESTAMP}.csv" | tee -a "$SUMMARY"
echo "  Full log:       $LOGFILE" | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"

# Analyze register differences
echo "============================================" | tee -a "$SUMMARY"
echo "Register Value Analysis" | tee -a "$SUMMARY"
echo "============================================" | tee -a "$SUMMARY"

# Extract unique SQ_GPR_MANAGEMENT values
echo "Unique SQ_GPR_MANAGEMENT values:" | tee -a "$SUMMARY"
cut -d',' -f4 "$OUTPUT_DIR/registers_${TIMESTAMP}.csv" | tail -n +2 | sort -u | tee -a "$SUMMARY"

echo "" | tee -a "$SUMMARY"
echo "Unique SQ_INST_STORE_MANAGMENT values:" | tee -a "$SUMMARY"
cut -d',' -f5 "$OUTPUT_DIR/registers_${TIMESTAMP}.csv" | tail -n +2 | sort -u | tee -a "$SUMMARY"

echo "" | tee -a "$SUMMARY"
echo "Unique RBBM_PM_OVERRIDE1 values:" | tee -a "$SUMMARY"
cut -d',' -f6 "$OUTPUT_DIR/registers_${TIMESTAMP}.csv" | tail -n +2 | sort -u | tee -a "$SUMMARY"

echo "" | tee -a "$SUMMARY"
echo "Unique RBBM_PM_OVERRIDE2 values:" | tee -a "$SUMMARY"
cut -d',' -f7 "$OUTPUT_DIR/registers_${TIMESTAMP}.csv" | tail -n +2 | sort -u | tee -a "$SUMMARY"

echo "" | tee -a "$SUMMARY"
echo "============================================" | tee -a "$SUMMARY"
echo "Test completed at $(date)" | tee -a "$SUMMARY"
echo "============================================" | tee -a "$SUMMARY"

# Create analysis helper script
cat > "$OUTPUT_DIR/analyze.sh" << 'ANALYZE_EOF'
#!/bin/bash
# Quick analysis of register dumps
# Usage: ./analyze.sh

DIR="$(dirname "$0")"
echo "Comparing register values between successful and failed runs..."
echo ""

# Find latest CSV files
REGS_CSV=$(ls -t "$DIR"/registers_*.csv | head -1)
RESULTS_CSV=$(ls -t "$DIR"/results_*.csv | head -1)

if [ -z "$REGS_CSV" ] || [ -z "$RESULTS_CSV" ]; then
    echo "No CSV files found"
    exit 1
fi

echo "Register CSV: $REGS_CSV"
echo "Results CSV: $RESULTS_CSV"
echo ""

# Join and analyze
echo "=== Runs with FPS < 5 (potential artifacts) ==="
awk -F',' 'NR>1 && $3<5 {print "Iteration "$1" benchmark "$2": FPS="$3}' "$RESULTS_CSV"

echo ""
echo "=== Runs with FPS >= 5 (likely successful) ==="
awk -F',' 'NR>1 && $3>=5 {print "Iteration "$1" benchmark "$2": FPS="$3}' "$RESULTS_CSV"

echo ""
echo "=== Register value frequency ==="
echo "SQ_GPR_MANAGEMENT:"
cut -d',' -f4 "$REGS_CSV" | tail -n +2 | sort | uniq -c | sort -rn

echo ""
echo "RBBM_PM_OVERRIDE2:"
cut -d',' -f7 "$REGS_CSV" | tail -n +2 | sort | uniq -c | sort -rn
ANALYZE_EOF

chmod +x "$OUTPUT_DIR/analyze.sh"

echo ""
echo "To analyze results, run: $OUTPUT_DIR/analyze.sh"
echo "Or copy $OUTPUT_DIR to your host for detailed analysis"
