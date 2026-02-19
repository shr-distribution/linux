#!/bin/bash
#
# glmark2 Interactive Test Script
#
# Runs glmark2 benchmark iterations with immediate visual feedback.
# You can press keys DURING the benchmark to provide feedback:
#   s = SMOOTH - mark as success and continue to next
#   f = FACETED - mark as failure and IMMEDIATELY skip to next
#   q = quit and show summary
#
# Usage: ./glmark2-interactive-test.sh [benchmark] [iterations]
#
# Benchmarks: build, texture, shading, bump, effect2d, pulsar, desktop,
#             buffer, ideas, jellyfish, terrain, shadow, refract, conditionals,
#             function, loop, clear
#
# Examples:
#   ./glmark2-interactive-test.sh build 10
#   ./glmark2-interactive-test.sh shading 5
#   ./glmark2-interactive-test.sh texture

BENCHMARK="${1:-build}"
ITERATIONS="${2:-10}"
RESULTS_FILE="/tmp/glmark2_${BENCHMARK}_$(date +%Y%m%d_%H%M%S).txt"

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

echo "============================================"
echo "glmark2 Interactive Test"
echo "============================================"
echo "Benchmark:  $BENCHMARK"
echo "Iterations: $ITERATIONS"
echo "Results:    $RESULTS_FILE"
echo ""
echo -e "${CYAN}Controls (press DURING benchmark):${NC}"
echo -e "  ${GREEN}s${NC} = SMOOTH (success) - continue to next"
echo -e "  ${RED}f${NC} = FACETED (failure) - ABORT and skip to next"
echo -e "  ${YELLOW}q${NC} = quit and show summary"
echo "============================================"
echo ""

# Initialize results file
echo "# glmark2 Interactive Test Results - $(date)" > "$RESULTS_FILE"
echo "# benchmark: $BENCHMARK" >> "$RESULTS_FILE"
echo "# iteration,result,fps" >> "$RESULTS_FILE"

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
    echo -e "${CYAN}Press s=smooth, f=faceted (abort), q=quit${NC}"
    echo ""

    # Run glmark2 in background
    FD_MESA_DEBUG=msgs glmark2-es2-drm --benchmark "$BENCHMARK" > /tmp/glmark_output_$i.txt 2>&1 &
    glmark_pid=$!

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
            f|F)
                result="FACETED"
                ((faceted_count++))
                echo -e "\n${RED}>>> FACETED - killing and moving on${NC}"
                # Kill immediately, don't wait for cleanup
                kill -9 $glmark_pid 2>/dev/null
                # Disown so we don't wait for it
                disown $glmark_pid 2>/dev/null
                break
                ;;
            q|Q)
                result="QUIT"
                user_quit=1
                echo -e "\n${YELLOW}>>> QUIT - killing and exiting${NC}"
                kill -9 $glmark_pid 2>/dev/null
                disown $glmark_pid 2>/dev/null
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

    # Extract FPS from output
    if [ -f /tmp/glmark_output_$i.txt ]; then
        fps=$(grep "\[$BENCHMARK\]" /tmp/glmark_output_$i.txt | grep 'FPS:' | sed 's/.*FPS: *\([0-9]*\).*/\1/' | tail -1)
        [ -z "$fps" ] && fps="N/A"
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
            f|F)
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

    # Log result
    echo "$i,$result,$fps" >> "$RESULTS_FILE"
    echo -e "Recorded: $result (FPS: $fps)"

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
