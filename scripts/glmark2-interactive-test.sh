#!/bin/bash
#
# glmark2 Interactive Test Script
#
# Runs glmark2 benchmark iterations with immediate visual feedback.
# You can press keys DURING the benchmark to provide feedback:
#   s = SMOOTH - mark as success and continue to next
#   g = FACETED (log) - mark as failure but let it finish for full logs
#   f = FACETED (skip) - mark as failure and IMMEDIATELY skip to next
#   q = quit and show summary
#
# Usage: ./glmark2-interactive-test.sh [options] [benchmark] [iterations]
#
# Options:
#   -c, --cmdstream    Enable command stream tracing (FD_A22X_CMDSTREAM=1)
#   -t, --ftrace       Enable kernel ftrace for GPU events
#   -r, --regsample    Enable register sampling during render (can slow down)
#   -d, --rd           Enable kernel ring dump capture (cmdstream via debugfs)
#   -h, --help         Show this help
#
# Benchmarks: build, texture, shading, bump, effect2d, pulsar, desktop,
#             buffer, ideas, jellyfish, terrain, shadow, refract, conditionals,
#             function, loop, clear
#
# Examples:
#   ./glmark2-interactive-test.sh build 10
#   ./glmark2-interactive-test.sh -c build 5        # with command stream tracing
#   ./glmark2-interactive-test.sh -c -t shading 3   # with cmdstream + ftrace
#   ./glmark2-interactive-test.sh -r build 5        # with register sampling
#   ./glmark2-interactive-test.sh -d build 5        # with kernel ring dump capture
#   ./glmark2-interactive-test.sh -r -d -t build 5  # full debug (regsample + rd + ftrace)

# Parse options
CMDSTREAM_TRACE=0
FTRACE_ENABLED=0
REGSAMPLE_ENABLED=0
RD_ENABLED=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--cmdstream)
            CMDSTREAM_TRACE=1
            shift
            ;;
        -t|--ftrace)
            FTRACE_ENABLED=1
            shift
            ;;
        -r|--regsample)
            REGSAMPLE_ENABLED=1
            shift
            ;;
        -d|--rd)
            RD_ENABLED=1
            shift
            ;;
        -h|--help)
            head -25 "$0" | tail -20
            exit 0
            ;;
        -*)
            echo "Unknown option: $1"
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

BENCHMARK="${1:-build}"
ITERATIONS="${2:-10}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Use persistent storage instead of /tmp (survives hangs/reboots)
OUTPUT_BASE="/media/internal/glmark2-tests"
mkdir -p "$OUTPUT_BASE"

RESULTS_FILE="${OUTPUT_BASE}/glmark2_${BENCHMARK}_${TIMESTAMP}.txt"
GPU_DUMP_DIR="${OUTPUT_BASE}/gpu_dumps_${TIMESTAMP}"
MESA_LOG="${OUTPUT_BASE}/mesa_debug_${TIMESTAMP}.log"
CMDSTREAM_LOG="${OUTPUT_BASE}/cmdstream_${TIMESTAMP}.log"
FTRACE_LOG="${OUTPUT_BASE}/ftrace_${TIMESTAMP}.log"

# Debugfs paths
DEBUGFS_BASE="/sys/kernel/debug"
DRI_DEBUGFS="/sys/kernel/debug/dri"
GPU_DEBUGFS=""  # Will be detected

# Counters
smooth_count=0
faceted_count=0
unknown_count=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to setup ftrace for GPU events
setup_ftrace() {
    local trace_dir="/sys/kernel/debug/tracing"

    if [ ! -d "$trace_dir" ]; then
        echo -e "${RED}Ftrace not available${NC}"
        return 1
    fi

    echo -e "${YELLOW}Setting up ftrace for GPU events...${NC}"

    # Clear existing trace
    echo > "$trace_dir/trace"

    # Enable DRM events if available
    if [ -f "$trace_dir/events/drm/enable" ]; then
        echo 1 > "$trace_dir/events/drm/enable"
        echo -e "  ${GREEN}DRM events enabled${NC}"
    fi

    # Enable MSM GPU events (Adreno-specific: submit, flush, retired, freq)
    if [ -f "$trace_dir/events/drm_msm_gpu/enable" ]; then
        echo 1 > "$trace_dir/events/drm_msm_gpu/enable"
        echo -e "  ${GREEN}MSM GPU events enabled${NC}"
    fi

    # Enable DMA fence events (tracks GPU completion)
    if [ -f "$trace_dir/events/dma_fence/enable" ]; then
        echo 1 > "$trace_dir/events/dma_fence/enable"
        echo -e "  ${GREEN}DMA fence events enabled${NC}"
    fi

    # Enable tracing
    echo 1 > "$trace_dir/tracing_on"

    echo -e "${GREEN}Ftrace enabled${NC}"
    return 0
}

# Function to capture ftrace output
capture_ftrace() {
    local output_file="$1"
    local trace_dir="/sys/kernel/debug/tracing"

    if [ -f "$trace_dir/trace" ]; then
        echo "=== Ftrace capture: $(date '+%Y-%m-%d %H:%M:%S') ===" > "$output_file"
        cat "$trace_dir/trace" >> "$output_file"
        # Clear trace for next iteration
        echo > "$trace_dir/trace"
    fi
}

# Function to stop ftrace
stop_ftrace() {
    local trace_dir="/sys/kernel/debug/tracing"
    if [ -f "$trace_dir/tracing_on" ]; then
        echo 0 > "$trace_dir/tracing_on" 2>/dev/null
    fi
    if [ -f "$trace_dir/events/drm/enable" ]; then
        echo 0 > "$trace_dir/events/drm/enable" 2>/dev/null
    fi
    if [ -f "$trace_dir/events/drm_msm_gpu/enable" ]; then
        echo 0 > "$trace_dir/events/drm_msm_gpu/enable" 2>/dev/null
    fi
    if [ -f "$trace_dir/events/dma_fence/enable" ]; then
        echo 0 > "$trace_dir/events/dma_fence/enable" 2>/dev/null
    fi
}

# Function to mount debugfs if needed
setup_debugfs() {
    # Check if debugfs is mounted
    if ! mountpoint -q "$DEBUGFS_BASE" 2>/dev/null; then
        echo -e "${YELLOW}Mounting debugfs...${NC}"
        mount -t debugfs none "$DEBUGFS_BASE" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Debugfs mounted successfully${NC}"
        else
            echo -e "${RED}Failed to mount debugfs (need root?)${NC}"
            return 1
        fi
    fi

    # Find DRI device directory (could be 5100000.mdp or similar)
    if [ -d "$DRI_DEBUGFS" ]; then
        for dir in "$DRI_DEBUGFS"/*/; do
            if [ -f "${dir}gpu" ]; then
                GPU_DEBUGFS="${dir}gpu"
                DRI_DIR="$dir"
                echo -e "${GREEN}Found GPU debugfs: $GPU_DEBUGFS${NC}"
                break
            fi
        done
    fi

    if [ -z "$GPU_DEBUGFS" ]; then
        echo -e "${YELLOW}Warning: GPU debugfs not found${NC}"
        return 1
    fi
    return 0
}

# Function to capture all debugfs state
capture_debugfs_state() {
    local prefix="$1"  # e.g., "pre" or "post"
    local iteration="$2"
    local result="$3"
    local dump_dir="$4"

    local state_file="${dump_dir}/debugfs_${BENCHMARK}_${iteration}_${prefix}_${result}.txt"

    echo "=== Debugfs capture: $prefix iteration $iteration ($result) ===" > "$state_file"
    echo "Timestamp: $(date '+%Y-%m-%d %H:%M:%S.%N')" >> "$state_file"
    echo "" >> "$state_file"

    # Capture GPU state
    if [ -f "$GPU_DEBUGFS" ]; then
        echo "=== GPU STATE ===" >> "$state_file"
        cat "$GPU_DEBUGFS" >> "$state_file" 2>/dev/null
        echo "" >> "$state_file"
    fi

    # Capture A2XX-specific debugfs files (new registers for faceted debugging)
    if [ -n "$DRI_DIR" ]; then
        # A2XX Summary (critical registers for faceted debug)
        if [ -f "${DRI_DIR}summary" ]; then
            echo "=== A2XX SUMMARY ===" >> "$state_file"
            cat "${DRI_DIR}summary" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # VGT (Vertex/Geometry) state
        if [ -f "${DRI_DIR}vgt" ]; then
            echo "=== A2XX VGT ===" >> "$state_file"
            cat "${DRI_DIR}vgt" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # TP/TC (Texture Pipe/Cache) state
        if [ -f "${DRI_DIR}tp" ]; then
            echo "=== A2XX TP/TC ===" >> "$state_file"
            cat "${DRI_DIR}tp" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # SQ (Shader Sequencer) state
        if [ -f "${DRI_DIR}sq" ]; then
            echo "=== A2XX SQ ===" >> "$state_file"
            cat "${DRI_DIR}sq" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # RB (Render Backend) state
        if [ -f "${DRI_DIR}rb" ]; then
            echo "=== A2XX RB ===" >> "$state_file"
            cat "${DRI_DIR}rb" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # PA (Primitive Assembly) state
        if [ -f "${DRI_DIR}pa" ]; then
            echo "=== A2XX PA ===" >> "$state_file"
            cat "${DRI_DIR}pa" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi
    fi

    # Capture other DRI debugfs files if available
    if [ -n "$DRI_DIR" ]; then
        # KMS state
        if [ -f "${DRI_DIR}kms" ]; then
            echo "=== KMS STATE ===" >> "$state_file"
            cat "${DRI_DIR}kms" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # Framebuffer info
        if [ -f "${DRI_DIR}fb" ]; then
            echo "=== FRAMEBUFFER ===" >> "$state_file"
            cat "${DRI_DIR}fb" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # GEM objects
        if [ -f "${DRI_DIR}gem" ]; then
            echo "=== GEM OBJECTS ===" >> "$state_file"
            cat "${DRI_DIR}gem" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi

        # Memory manager
        if [ -f "${DRI_DIR}mm" ]; then
            echo "=== MEMORY MANAGER ===" >> "$state_file"
            cat "${DRI_DIR}mm" >> "$state_file" 2>/dev/null
            echo "" >> "$state_file"
        fi
    fi

    # Capture recent dmesg (last 50 lines with GPU/DRM mentions)
    echo "=== RECENT DMESG (GPU/DRM) ===" >> "$state_file"
    dmesg | grep -iE 'gpu|drm|msm|adreno|a2xx|kgsl|fence|hang' | tail -50 >> "$state_file" 2>/dev/null
    echo "" >> "$state_file"

    # Capture interrupts related to GPU
    echo "=== GPU INTERRUPTS ===" >> "$state_file"
    grep -E 'mdp|gpu|kgsl' /proc/interrupts >> "$state_file" 2>/dev/null
    echo "" >> "$state_file"

    echo "$state_file"
}

echo "============================================"
echo "glmark2 Interactive Test"
echo "============================================"
echo "Benchmark:  $BENCHMARK"
echo "Iterations: $ITERATIONS"
echo "Output dir: $OUTPUT_BASE"
echo "Results:    $RESULTS_FILE"
echo "GPU Dumps:  $GPU_DUMP_DIR"
echo "Mesa Log:   $MESA_LOG"
if [ $CMDSTREAM_TRACE -eq 1 ]; then
    echo -e "Cmdstream:  ${GREEN}ENABLED${NC} -> $CMDSTREAM_LOG"
fi
if [ $FTRACE_ENABLED -eq 1 ]; then
    echo -e "Ftrace:     ${GREEN}ENABLED${NC} -> $FTRACE_LOG"
fi
if [ $REGSAMPLE_ENABLED -eq 1 ]; then
    echo -e "RegSample:  ${GREEN}ENABLED${NC} (500ms interval)"
fi
if [ $RD_ENABLED -eq 1 ]; then
    echo -e "Ring Dump:  ${GREEN}ENABLED${NC} (kernel cmdstream capture)"
fi
echo ""

# Setup debugfs
echo "Setting up debugfs..."
setup_debugfs
DEBUGFS_AVAILABLE=$?

# Setup ftrace if requested
if [ $FTRACE_ENABLED -eq 1 ]; then
    setup_ftrace
fi

echo ""
echo -e "${CYAN}Controls (press DURING benchmark):${NC}"
echo -e "  ${GREEN}s${NC} = SMOOTH (success) - continue to next"
echo -e "  ${RED}g${NC} = FACETED (log) - mark failed, but finish for full logs"
echo -e "  ${RED}f${NC} = FACETED (skip) - mark failed and ABORT immediately"
echo -e "  ${YELLOW}q${NC} = quit and show summary"
echo "============================================"
echo ""

# Initialize results file
echo "# glmark2 Interactive Test Results - $(date)" > "$RESULTS_FILE"
echo "# benchmark: $BENCHMARK" >> "$RESULTS_FILE"
echo "# iteration,result,fps" >> "$RESULTS_FILE"

# Create GPU dump directory
mkdir -p "$GPU_DUMP_DIR"
echo "# GPU dumps saved to: $GPU_DUMP_DIR" >> "$RESULTS_FILE"
echo "# Debugfs available: $DEBUGFS_AVAILABLE (0=yes, 1=no)" >> "$RESULTS_FILE"

# Function to show current stats
show_stats() {
    local total=$((smooth_count + faceted_count + unknown_count))
    local rate=0
    if [ $total -gt 0 ]; then
        rate=$((smooth_count * 100 / total))
    fi
    echo -e "${GREEN}SMOOTH: $smooth_count${NC} | ${RED}FACETED: $faceted_count${NC} | ${YELLOW}UNKNOWN: $unknown_count${NC} | Success rate: ${rate}%"
}

# Cleanup function
cleanup() {
    # Kill any remaining glmark2 processes
    pkill -9 glmark2-es2-drm 2>/dev/null
    # Stop ftrace if it was enabled
    if [ $FTRACE_ENABLED -eq 1 ]; then
        stop_ftrace
    fi
    # Restore terminal settings
    stty sane 2>/dev/null
}
trap cleanup EXIT

# Main test loop
for i in $(seq 1 $ITERATIONS); do
    echo ""
    echo "============================================"
    echo "[$BENCHMARK] Iteration $i of $ITERATIONS"
    show_stats
    echo "============================================"
    echo -e "${CYAN}Press s=smooth, g=faceted(log), f=faceted(skip), q=quit${NC}"
    echo ""

    # Capture PRE-iteration debugfs state
    if [ $DEBUGFS_AVAILABLE -eq 0 ]; then
        pre_state=$(capture_debugfs_state "pre" "$i" "PENDING" "$GPU_DUMP_DIR")
        echo -e "${CYAN}Pre-iteration state saved${NC}"
    fi

    # Run glmark2 in background, capture Mesa debug output
    ITERATION_LOG="${OUTPUT_BASE}/iteration_${i}_${TIMESTAMP}.log"
    CMDSTREAM_ITER_LOG="${OUTPUT_BASE}/cmdstream_iter_${i}_${TIMESTAMP}.log"
    echo "=== Iteration $i started at $(date) ===" >> "$MESA_LOG"

    # Build environment variables
    MESA_ENV="FD_MESA_DEBUG=msgs MESA_DEBUG=1"
    if [ $CMDSTREAM_TRACE -eq 1 ]; then
        MESA_ENV="$MESA_ENV FD_A22X_CMDSTREAM=1"
        echo "=== Iteration $i command stream ===" > "$CMDSTREAM_ITER_LOG"
    fi

    # Start kernel ring dump capture if enabled
    # Note: rd capture is heavyweight, so we only capture for 2 seconds
    # after glmark2 starts rendering (not the whole iteration)
    rd_pid=""
    RD_TEMP_FILE="${GPU_DUMP_DIR}/rd_${BENCHMARK}_${i}_PENDING.rd"

    # Run glmark2 with appropriate environment
    if [ $CMDSTREAM_TRACE -eq 1 ]; then
        FD_MESA_DEBUG=msgs MESA_DEBUG=1 FD_A22X_CMDSTREAM=1 glmark2-es2-drm --benchmark "$BENCHMARK" > "$ITERATION_LOG" 2>&1 &
    else
        FD_MESA_DEBUG=msgs MESA_DEBUG=1 glmark2-es2-drm --benchmark "$BENCHMARK" > "$ITERATION_LOG" 2>&1 &
    fi
    glmark_pid=$!

    # Start brief rd capture after glmark2 begins rendering (2 second window)
    # This avoids the severe performance hit of continuous rd capture
    if [ $RD_ENABLED -eq 1 ] && [ -f "/sys/kernel/debug/dri/0/rd" ]; then
        (
            sleep 0.5  # Let glmark2 start rendering
            timeout 2 cat /sys/kernel/debug/dri/0/rd > "$RD_TEMP_FILE" 2>/dev/null
        ) &
        rd_pid=$!
    fi

    # Start background register sampling during rendering (optional, can cause slowdown)
    sampler_pid=""
    REGSAMPLE_LOG="${GPU_DUMP_DIR}/regsample_${BENCHMARK}_${i}.txt"
    if [ "${REGSAMPLE_ENABLED:-0}" -eq 1 ] && [ -n "$DRI_DIR" ] && [ -f "${DRI_DIR}summary" ]; then
        (
            sample_num=0
            echo "=== Register sampling started: $(date '+%H:%M:%S.%N') ===" > "$REGSAMPLE_LOG"
            while kill -0 $glmark_pid 2>/dev/null; do
                echo "--- Sample $sample_num @ $(date '+%H:%M:%S.%N') ---" >> "$REGSAMPLE_LOG"
                # Capture critical registers only
                cat "${DRI_DIR}summary" 2>/dev/null | grep -E "SQ_|VGT_|TP0_|TC_CNTL|RB_|PA_|PC_DEBUG|GRAS_|RBBM_STATUS|VSC_|LRZ_" >> "$REGSAMPLE_LOG"
                ((sample_num++))
                sleep 0.5  # Sample every 500ms (less aggressive)
            done
            echo "=== Sampling ended: $(date '+%H:%M:%S.%N'), $sample_num samples ===" >> "$REGSAMPLE_LOG"
        ) </dev/null &
        sampler_pid=$!
    fi

    result="UNKNOWN"
    fps="N/A"
    user_quit=0

    # Set terminal to raw mode for immediate key detection
    stty -echo -icanon min 0 time 1 2>/dev/null

    # Monitor for user input while glmark2 runs
    while kill -0 $glmark_pid 2>/dev/null; do
        # Try to read a character (non-blocking due to stty settings)
        char=$(dd bs=1 count=1 2>/dev/null)

        case "$char" in
            s|S)
                result="SMOOTH"
                ((smooth_count++))
                echo -e "\n${GREEN}>>> SMOOTH - waiting for benchmark to finish...${NC}"
                # Let it finish naturally
                ;;
            g|G)
                result="FACETED"
                ((faceted_count++))
                echo -e "\n${RED}>>> FACETED - waiting for benchmark to finish (capturing logs)...${NC}"
                # Let it finish naturally to capture full logs
                ;;
            f|F)
                result="FACETED"
                ((faceted_count++))
                echo -e "\n${RED}>>> FACETED - aborting${NC}"
                kill -9 $glmark_pid 2>/dev/null
                wait $glmark_pid 2>/dev/null
                break
                ;;
            q|Q)
                result="QUIT"
                user_quit=1
                echo -e "\n${YELLOW}>>> QUIT${NC}"
                kill -9 $glmark_pid 2>/dev/null
                wait $glmark_pid 2>/dev/null
                break
                ;;
        esac

        # Small sleep to prevent CPU spinning
        sleep 0.1
    done

    # Restore terminal
    stty sane 2>/dev/null

    # Wait for glmark2 to finish if still running
    wait $glmark_pid 2>/dev/null

    # Stop register sampler
    if [ -n "$sampler_pid" ]; then
        kill $sampler_pid 2>/dev/null
        wait $sampler_pid 2>/dev/null
    fi

    # Stop rd capture
    if [ -n "$rd_pid" ]; then
        kill $rd_pid 2>/dev/null
        wait $rd_pid 2>/dev/null
    fi

    # Extract FPS from output
    if [ -f "$ITERATION_LOG" ]; then
        # FPS is on its own line: " FPS: 45 FrameTime: 22.621 ms"
        fps=$(grep -E '^\s*FPS:' "$ITERATION_LOG" | sed 's/.*FPS: *\([0-9]*\).*/\1/' | tail -1)
        [ -z "$fps" ] && fps="N/A"
        # Append Mesa debug output to main log
        cat "$ITERATION_LOG" >> "$MESA_LOG" 2>/dev/null
        echo "=== Iteration $i ended: $result ===" >> "$MESA_LOG"
    fi

    # If user didn't provide input, ask now
    if [ "$result" = "UNKNOWN" ] && [ $user_quit -eq 0 ]; then
        echo ""
        echo -e ">>> FPS: $fps - Was it ${GREEN}[s]mooth${NC} or ${RED}[f]aceted${NC}? "
        read -n 1 input
        echo ""
        case "$input" in
            s|S)
                result="SMOOTH"
                ((smooth_count++))
                ;;
            f|F|g|G)
                result="FACETED"
                ((faceted_count++))
                ;;
            q|Q)
                user_quit=1
                ;;
            *)
                ((unknown_count++))
                ;;
        esac
    fi

    # Capture POST-iteration debugfs state (comprehensive)
    if [ $DEBUGFS_AVAILABLE -eq 0 ] && [ "$result" != "QUIT" ]; then
        post_state=$(capture_debugfs_state "post" "$i" "$result" "$GPU_DUMP_DIR")
        echo -e "${CYAN}Post-iteration state saved: $(basename "$post_state")${NC}"

        # Also save simple GPU dump for quick comparison
        if [ -f "$GPU_DEBUGFS" ]; then
            gpu_dump_file="$GPU_DUMP_DIR/gpu_${BENCHMARK}_${i}_${result}.txt"
            cat "$GPU_DEBUGFS" > "$gpu_dump_file" 2>/dev/null
        fi

        # Rename pre-state file with actual result
        if [ -f "$pre_state" ]; then
            new_pre_state="${GPU_DUMP_DIR}/debugfs_${BENCHMARK}_${i}_pre_${result}.txt"
            mv "$pre_state" "$new_pre_state" 2>/dev/null
        fi
    fi

    # Rename rd capture file with actual result
    if [ $RD_ENABLED -eq 1 ] && [ -f "$RD_TEMP_FILE" ]; then
        RD_FINAL_FILE="${GPU_DUMP_DIR}/rd_${BENCHMARK}_${i}_${result}.rd"
        mv "$RD_TEMP_FILE" "$RD_FINAL_FILE" 2>/dev/null
        rd_size=$(stat -c%s "$RD_FINAL_FILE" 2>/dev/null || echo 0)
        echo -e "${CYAN}Ring dump saved: $(basename "$RD_FINAL_FILE") (${rd_size} bytes)${NC}"
    fi

    # Capture command stream log for this iteration
    if [ $CMDSTREAM_TRACE -eq 1 ] && [ -f "$ITERATION_LOG" ]; then
        echo "" >> "$CMDSTREAM_LOG"
        echo "=== Iteration $i ($result) ===" >> "$CMDSTREAM_LOG"
        # Extract command stream lines (look for CMDSTREAM or ring buffer output)
        grep -E 'CMDSTREAM|RING\[|PM4|CP_|OUT_RING|dwords' "$ITERATION_LOG" >> "$CMDSTREAM_LOG" 2>/dev/null
    fi

    # Capture ftrace for this iteration
    if [ $FTRACE_ENABLED -eq 1 ]; then
        FTRACE_ITER="${GPU_DUMP_DIR}/ftrace_${BENCHMARK}_${i}_${result}.txt"
        capture_ftrace "$FTRACE_ITER"
        # Also append to main ftrace log
        if [ -f "$FTRACE_ITER" ]; then
            echo "" >> "$FTRACE_LOG"
            echo "=== Iteration $i ($result) ===" >> "$FTRACE_LOG"
            cat "$FTRACE_ITER" >> "$FTRACE_LOG"
        fi
    fi

    # Log result
    echo "$i,$result,$fps" >> "$RESULTS_FILE"
    echo -e "Recorded: $result (FPS: $fps)"

    # Sync to disk immediately (survives hangs)
    sync

    # Check if user wants to quit
    if [ $user_quit -eq 1 ]; then
        echo "Quitting..."
        break
    fi

    # Brief pause before next iteration
    sleep 0.5
done

# Final summary
echo ""
echo "============================================"
echo "            FINAL SUMMARY"
echo "============================================"
echo ""
show_stats
echo ""
echo "Detailed results:"
echo "----------------------------------------"
cat "$RESULTS_FILE"
echo "----------------------------------------"
echo ""
echo "Results saved to: $RESULTS_FILE"

# Calculate and display success rate
total=$((smooth_count + faceted_count))
if [ $total -gt 0 ]; then
    rate=$((smooth_count * 100 / total))
    echo ""
    echo "Success rate: $smooth_count / $total = ${rate}%"
fi

# GPU dump comparison helper
echo ""
echo "============================================"
echo "       GPU DUMP ANALYSIS"
echo "============================================"
echo ""

# Find first SMOOTH and first FACETED dump for this benchmark
smooth_dump=$(ls "$GPU_DUMP_DIR"/gpu_${BENCHMARK}_*_SMOOTH.txt 2>/dev/null | head -1)
faceted_dump=$(ls "$GPU_DUMP_DIR"/gpu_${BENCHMARK}_*_FACETED.txt 2>/dev/null | head -1)

if [ -n "$smooth_dump" ] && [ -n "$faceted_dump" ]; then
    echo "Comparing first SMOOTH vs first FACETED:"
    echo "  SMOOTH:  $(basename "$smooth_dump")"
    echo "  FACETED: $(basename "$faceted_dump")"
    echo ""

    # Count differences
    diff_count=$(diff "$smooth_dump" "$faceted_dump" | grep "^[<>]" | wc -l)
    echo "Register differences: $((diff_count / 2)) lines"
    echo ""

    # Show first few differences
    echo "First 20 differences:"
    diff "$smooth_dump" "$faceted_dump" | grep "^[<>]" | head -20
    echo ""
    echo "Full diff: diff $smooth_dump $faceted_dump"
else
    if [ -z "$smooth_dump" ]; then
        echo "No SMOOTH GPU dumps captured"
    fi
    if [ -z "$faceted_dump" ]; then
        echo "No FACETED GPU dumps captured"
    fi
fi

echo ""
echo "GPU dumps directory: $GPU_DUMP_DIR"
echo "To analyze: ls -la $GPU_DUMP_DIR"

# Additional analysis: compare pre vs post state for each iteration
echo ""
echo "============================================"
echo "       PRE vs POST STATE ANALYSIS"
echo "============================================"
echo ""

for pre_file in "$GPU_DUMP_DIR"/debugfs_${BENCHMARK}_*_pre_*.txt; do
    [ -f "$pre_file" ] || continue
    basename_pre=$(basename "$pre_file")
    # Extract iteration number and result (format: debugfs_BENCHMARK_NUM_pre_RESULT.txt)
    iter_num=$(echo "$basename_pre" | sed "s/debugfs_${BENCHMARK}_\([0-9]*\)_pre_.*/\1/")
    result_type=$(echo "$basename_pre" | sed "s/debugfs_${BENCHMARK}_[0-9]*_pre_\(.*\)\.txt/\1/")
    post_file="$GPU_DUMP_DIR/debugfs_${BENCHMARK}_${iter_num}_post_${result_type}.txt"

    if [ -f "$post_file" ]; then
        echo "Iteration $iter_num (${result_type}):"
        # Count differences in GPU STATE section only
        pre_gpu=$(sed -n '/=== GPU STATE ===/,/=== /p' "$pre_file" 2>/dev/null | head -100)
        post_gpu=$(sed -n '/=== GPU STATE ===/,/=== /p' "$post_file" 2>/dev/null | head -100)
        if [ "$pre_gpu" = "$post_gpu" ]; then
            echo "  GPU state: IDENTICAL (pre vs post)"
        else
            diff_lines=$(diff <(echo "$pre_gpu") <(echo "$post_gpu") | grep "^[<>]" | wc -l)
            echo "  GPU state: $((diff_lines/2)) differences (pre vs post)"
        fi
    fi
done

echo ""
echo "Full debugfs captures available in: $GPU_DUMP_DIR"
echo "Files: debugfs_<iteration>_<pre|post>_<SMOOTH|FACETED>.txt"

# Command stream summary
if [ $CMDSTREAM_TRACE -eq 1 ]; then
    echo ""
    echo "============================================"
    echo "       COMMAND STREAM TRACE"
    echo "============================================"
    echo ""
    if [ -f "$CMDSTREAM_LOG" ] && [ -s "$CMDSTREAM_LOG" ]; then
        line_count=$(wc -l < "$CMDSTREAM_LOG")
        echo "Command stream log: $CMDSTREAM_LOG ($line_count lines)"
        echo ""
        echo "Sample (last 30 lines):"
        echo "----------------------------------------"
        tail -30 "$CMDSTREAM_LOG"
        echo "----------------------------------------"
    else
        echo "No command stream data captured."
        echo "Note: FD_A22X_CMDSTREAM requires Mesa patch 0019 with cmdstream tracing."
    fi
fi

# Ftrace summary
if [ $FTRACE_ENABLED -eq 1 ]; then
    echo ""
    echo "============================================"
    echo "       KERNEL FTRACE LOG"
    echo "============================================"
    echo ""
    if [ -f "$FTRACE_LOG" ] && [ -s "$FTRACE_LOG" ]; then
        line_count=$(wc -l < "$FTRACE_LOG")
        echo "Ftrace log: $FTRACE_LOG ($line_count lines)"
        echo ""
        echo "Per-iteration ftrace files in: $GPU_DUMP_DIR/ftrace_*"
    else
        echo "No ftrace data captured."
    fi
fi

# Register sampling analysis
echo ""
echo "============================================"
echo "       REGISTER SAMPLING ANALYSIS"
echo "============================================"
echo ""

# Analyze register samples for each iteration
for regsample in "$GPU_DUMP_DIR"/regsample_${BENCHMARK}_*.txt; do
    [ -f "$regsample" ] || continue
    iter_num=$(basename "$regsample" | sed "s/regsample_${BENCHMARK}_\([0-9]*\)\.txt/\1/")
    sample_count=$(grep -c "^--- Sample" "$regsample" 2>/dev/null || echo 0)

    # Get result for this iteration from results file
    iter_result=$(grep "^$iter_num," "$RESULTS_FILE" | cut -d, -f2)

    echo "Iteration $iter_num ($iter_result): $sample_count samples"

    # Check for any variation in SQ_INTERPOLATOR_CNTL (should always be ffffffff)
    interp_values=$(grep "SQ_INTERPOLATOR_CNTL" "$regsample" | awk '{print $2}' | sort -u)
    interp_count=$(echo "$interp_values" | wc -l)
    if [ "$interp_count" -gt 1 ]; then
        echo "  WARNING: SQ_INTERPOLATOR_CNTL varied: $interp_values"
    else
        echo "  SQ_INTERPOLATOR_CNTL: stable ($interp_values)"
    fi

    # Check RBBM_STATUS for busy states
    busy_count=$(grep "RBBM_STATUS" "$regsample" | grep -v "IDLE" | wc -l)
    idle_count=$(grep "RBBM_STATUS" "$regsample" | grep "IDLE" | wc -l)
    echo "  RBBM_STATUS: $idle_count idle, $busy_count busy"
done

echo ""
echo "Register sample files: $GPU_DUMP_DIR/regsample_${BENCHMARK}_*.txt"

# Ring dump analysis
if [ $RD_ENABLED -eq 1 ]; then
    echo ""
    echo "============================================"
    echo "       RING DUMP (CMDSTREAM) CAPTURE"
    echo "============================================"
    echo ""

    # Find rd files for this benchmark
    smooth_rd=$(ls "$GPU_DUMP_DIR"/rd_${BENCHMARK}_*_SMOOTH.rd 2>/dev/null | head -1)
    faceted_rd=$(ls "$GPU_DUMP_DIR"/rd_${BENCHMARK}_*_FACETED.rd 2>/dev/null | head -1)

    if [ -n "$smooth_rd" ] || [ -n "$faceted_rd" ]; then
        echo "Ring dump files captured:"
        for rd_file in "$GPU_DUMP_DIR"/rd_${BENCHMARK}_*.rd; do
            [ -f "$rd_file" ] || continue
            rd_size=$(stat -c%s "$rd_file" 2>/dev/null || echo 0)
            echo "  $(basename "$rd_file"): $rd_size bytes"
        done
        echo ""

        if [ -n "$smooth_rd" ] && [ -n "$faceted_rd" ]; then
            smooth_size=$(stat -c%s "$smooth_rd" 2>/dev/null || echo 0)
            faceted_size=$(stat -c%s "$faceted_rd" 2>/dev/null || echo 0)
            echo "SMOOTH vs FACETED comparison:"
            echo "  SMOOTH:  $(basename "$smooth_rd") ($smooth_size bytes)"
            echo "  FACETED: $(basename "$faceted_rd") ($faceted_size bytes)"
            echo ""
            echo "To analyze with cffdump (on host):"
            echo "  cffdump $smooth_rd > smooth_cmds.txt"
            echo "  cffdump $faceted_rd > faceted_cmds.txt"
            echo "  diff smooth_cmds.txt faceted_cmds.txt"
        fi
    else
        echo "No ring dump files captured."
        echo "Note: Ring dump requires GPU activity during capture."
    fi

    echo ""
    echo "Ring dump files: $GPU_DUMP_DIR/rd_${BENCHMARK}_*.rd"
fi

echo ""
echo "============================================"
echo "All logs saved to: $OUTPUT_BASE"
echo "============================================"
