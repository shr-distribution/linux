#!/bin/bash
# Pack DRM/KMS test tools (modetest, kmscube) and dependencies for HP TouchPad
#
# This script creates a deployment package containing modetest, kmscube,
# and all required shared libraries for running DRM/KMS tests on the device.
#
# Usage: ./scripts/pack-drm-tests.sh [sysroot-path]
#
# If sysroot-path is not specified, the script will look for:
#   1. ../luneos-sysroot (LuneOS cross-compilation sysroot)
#   2. /opt/luneos-sysroot
#   3. Environment variable LUNEOS_SYSROOT
#
# Output: ../build-output/drm-tests.tar.gz

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(dirname "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$KERNEL_DIR")"
BUILD_OUTPUT="$PARENT_DIR/build-output"
PACKAGE_DIR="$BUILD_OUTPUT/drm-tests"
PACKAGE_TAR="$BUILD_OUTPUT/drm-tests.tar.gz"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

status() { echo -e "${GREEN}[*]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
error() { echo -e "${RED}[X]${NC} $1"; }

# Known build locations for TouchPad DRM/graphics stack
LIBDRM_BUILD="$PARENT_DIR/libdrm/build-arm"
MESA_BUILD="$PARENT_DIR/mesa/build-arm"
KMSCUBE_BUILD="$PARENT_DIR/kmscube/build-arm"

# Find sysroot (for libc and other system libs)
find_sysroot() {
    if [ -n "$1" ] && [ -d "$1" ]; then
        echo "$1"
        return 0
    fi

    if [ -n "$LUNEOS_SYSROOT" ] && [ -d "$LUNEOS_SYSROOT" ]; then
        echo "$LUNEOS_SYSROOT"
        return 0
    fi

    local candidates=(
        "$PARENT_DIR/luneos-sysroot"
        "/opt/luneos-sysroot"
        "$HOME/luneos-sysroot"
        "/usr/arm-linux-gnueabihf"
    )

    for candidate in "${candidates[@]}"; do
        if [ -d "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done

    return 1
}

# Find a binary in sysroot
find_binary() {
    local sysroot="$1"
    local name="$2"
    local paths=(
        "$sysroot/usr/bin/$name"
        "$sysroot/usr/local/bin/$name"
        "$sysroot/bin/$name"
    )

    for path in "${paths[@]}"; do
        if [ -f "$path" ] && [ -x "$path" ]; then
            echo "$path"
            return 0
        fi
    done

    return 1
}

# Find required shared libraries for a binary
find_libs() {
    local binary="$1"
    local sysroot="$2"
    local cross_prefix="${CROSS_COMPILE:-arm-linux-gnueabihf-}"
    local readelf="${cross_prefix}readelf"

    # Check if we have cross readelf
    if ! command -v "$readelf" &>/dev/null; then
        readelf="readelf"
    fi

    # Get list of needed libraries
    $readelf -d "$binary" 2>/dev/null | grep NEEDED | sed 's/.*\[\(.*\)\]/\1/'
}

# Find library file in sysroot
find_lib_file() {
    local sysroot="$1"
    local libname="$2"
    local paths=(
        "$sysroot/usr/lib/$libname"
        "$sysroot/usr/lib/arm-linux-gnueabihf/$libname"
        "$sysroot/lib/$libname"
        "$sysroot/lib/arm-linux-gnueabihf/$libname"
    )

    for path in "${paths[@]}"; do
        if [ -f "$path" ]; then
            echo "$path"
            return 0
        fi
        # Also check for symlinks and versioned libraries
        for f in "$path"*; do
            if [ -f "$f" ] 2>/dev/null; then
                echo "$f"
                return 0
            fi
        done
    done

    return 1
}

# Recursively collect all library dependencies
collect_libs() {
    local binary="$1"
    local sysroot="$2"
    local dest_dir="$3"
    local collected_file="$4"

    local libs=$(find_libs "$binary" "$sysroot")
    for lib in $libs; do
        # Skip already collected
        if grep -q "^$lib$" "$collected_file" 2>/dev/null; then
            continue
        fi
        echo "$lib" >> "$collected_file"

        local lib_path=$(find_lib_file "$sysroot" "$lib")
        if [ -n "$lib_path" ]; then
            # Copy the library and any symlinks
            local lib_basename=$(basename "$lib_path")
            if [ ! -f "$dest_dir/$lib_basename" ]; then
                cp -P "$lib_path"* "$dest_dir/" 2>/dev/null || cp "$lib_path" "$dest_dir/"
                status "  Copied: $lib_basename"
                # Recurse for this library's dependencies
                collect_libs "$lib_path" "$sysroot" "$dest_dir" "$collected_file"
            fi
        else
            warn "  Library not found: $lib"
        fi
    done
}

# Create the test runner script for on-device use
create_test_runner() {
    local dest="$1"

    cat > "$dest/run-drm-tests.sh" << 'SCRIPT_EOF'
#!/bin/sh
# DRM/KMS Test Runner for HP TouchPad
#
# This script sets CPU and GPU to maximum performance,
# runs modetest and kmscube tests, and monitors frequencies.
#
# Logs are written to /mnt/boot for persistence across reboots
# in case the GPU locks up or kills USB.
#
# Usage: ./run-drm-tests.sh [--quick|--full|--benchmark]

set -e

# Paths
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:$LD_LIBRARY_PATH"
export GBM_BACKENDS_PATH="$SCRIPT_DIR/lib/gbm"
export PATH="$SCRIPT_DIR/bin:$PATH"

# Persistent log directory (survives reboot)
# /mnt/boot is the boot partition in LuneOS initramfs
LOG_DIR="/mnt/boot"
LOG_PREFIX="drm-test"
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
DMESG_LOG="$LOG_DIR/${LOG_PREFIX}-dmesg-${TIMESTAMP}.log"
TEST_LOG="$LOG_DIR/${LOG_PREFIX}-${TIMESTAMP}.log"
LOGGER_PID=""

# Colors (may not work on all terminals)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Test mode
MODE="${1:-quick}"

log() { echo "${GREEN}[*]${NC} $1"; log_to_file "[*] $1"; }
warn() { echo "${YELLOW}[!]${NC} $1"; log_to_file "[!] $1"; }
err() { echo "${RED}[X]${NC} $1"; log_to_file "[X] $1"; }
info() { echo "${BLUE}[i]${NC} $1"; log_to_file "[i] $1"; }

# Write to persistent log file
log_to_file() {
    if [ -n "$TEST_LOG" ] && [ -d "$LOG_DIR" ]; then
        echo "[$(date '+%H:%M:%S')] $1" >> "$TEST_LOG" 2>/dev/null || true
    fi
}

# Save dmesg to persistent storage with a label
save_dmesg() {
    local label="$1"
    if [ -d "$LOG_DIR" ]; then
        echo "" >> "$DMESG_LOG"
        echo "========== DMESG $label [$(date '+%Y-%m-%d %H:%M:%S')] ==========" >> "$DMESG_LOG"
        dmesg >> "$DMESG_LOG" 2>/dev/null || true
        sync
    fi
}

# Background dmesg logger - uses dmesg -w for real-time capture
start_dmesg_logger() {
    local interval="${1:-1}"
    # Try dmesg -w first (real-time follow), fall back to polling
    if dmesg -w --help >/dev/null 2>&1; then
        log "Using dmesg -w for real-time kernel log capture"
        dmesg -w >> "$DMESG_LOG" 2>/dev/null &
        LOGGER_PID=$!
    else
        log "Using polling mode for kernel log capture (every ${interval}s)"
        (
            count=0
            last_lines=0
            while true; do
                sleep $interval
                count=$((count + 1))
                # Get new dmesg lines since last check
                current_lines=$(dmesg | wc -l)
                if [ "$current_lines" -gt "$last_lines" ]; then
                    echo "=== CHECKPOINT $count [$(date '+%H:%M:%S.%N' | cut -c1-12)] ===" >> "$DMESG_LOG"
                    dmesg | tail -n $((current_lines - last_lines)) >> "$DMESG_LOG" 2>/dev/null || true
                    sync
                    last_lines=$current_lines
                fi
            done
        ) &
        LOGGER_PID=$!
    fi
    log "Started background dmesg logger (PID: $LOGGER_PID)"
}

stop_dmesg_logger() {
    if [ -n "$LOGGER_PID" ]; then
        kill $LOGGER_PID 2>/dev/null || true
        wait $LOGGER_PID 2>/dev/null || true
        LOGGER_PID=""
        log "Stopped background dmesg logger"
    fi
}

# Store original settings for restoration
ORIG_CPU0_GOV=""
ORIG_CPU1_GOV=""
ORIG_GPU_GOV=""

# CPU frequency paths
CPU0_GOV="/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
CPU1_GOV="/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor"
CPU0_FREQ="/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
CPU1_FREQ="/sys/devices/system/cpu/cpu1/cpufreq/scaling_cur_freq"
CPU0_MAX="/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"
CPU0_AVAIL_GOV="/sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors"

# GPU frequency paths (Adreno via devfreq or debugfs)
GPU_DEVFREQ="/sys/class/devfreq/4300000.adreno"
GPU_DEBUGFS="/sys/kernel/debug/dri/0"
GPU_CLK_RATE="/sys/kernel/debug/clk/gfx3d_clk/clk_rate"

cleanup() {
    log "Restoring original settings..."

    # Stop background logger
    stop_dmesg_logger

    # Restore CPU governors
    if [ -n "$ORIG_CPU0_GOV" ] && [ -f "$CPU0_GOV" ]; then
        echo "$ORIG_CPU0_GOV" > "$CPU0_GOV" 2>/dev/null || true
    fi
    if [ -n "$ORIG_CPU1_GOV" ] && [ -f "$CPU1_GOV" ]; then
        echo "$ORIG_CPU1_GOV" > "$CPU1_GOV" 2>/dev/null || true
    fi

    # Restore GPU governor if using devfreq
    if [ -n "$ORIG_GPU_GOV" ] && [ -f "$GPU_DEVFREQ/governor" ]; then
        echo "$ORIG_GPU_GOV" > "$GPU_DEVFREQ/governor" 2>/dev/null || true
    fi

    # Final dmesg save
    save_dmesg "CLEANUP/EXIT"

    log "Settings restored."
    log "Logs saved to: $LOG_DIR"
    log "  Test log:  $TEST_LOG"
    log "  Dmesg log: $DMESG_LOG"
}

trap cleanup EXIT

show_frequencies() {
    echo ""
    info "=== Current Frequencies ==="

    # CPU frequencies
    if [ -f "$CPU0_FREQ" ]; then
        local cpu0_mhz=$(($(cat "$CPU0_FREQ") / 1000))
        info "CPU0: ${cpu0_mhz} MHz (governor: $(cat "$CPU0_GOV" 2>/dev/null || echo 'N/A'))"
    fi
    if [ -f "$CPU1_FREQ" ]; then
        local cpu1_mhz=$(($(cat "$CPU1_FREQ") / 1000))
        info "CPU1: ${cpu1_mhz} MHz (governor: $(cat "$CPU1_GOV" 2>/dev/null || echo 'N/A'))"
    fi

    # GPU frequency
    if [ -f "$GPU_DEVFREQ/cur_freq" ]; then
        local gpu_mhz=$(($(cat "$GPU_DEVFREQ/cur_freq") / 1000000))
        info "GPU:  ${gpu_mhz} MHz (governor: $(cat "$GPU_DEVFREQ/governor" 2>/dev/null || echo 'N/A'))"
    elif [ -f "$GPU_CLK_RATE" ]; then
        local gpu_mhz=$(($(cat "$GPU_CLK_RATE") / 1000000))
        info "GPU:  ${gpu_mhz} MHz (via clock debugfs)"
    else
        info "GPU:  frequency not available via sysfs"
    fi
    echo ""
}

set_max_performance() {
    log "Setting CPU and GPU to maximum performance..."

    # Save and set CPU0 governor
    if [ -f "$CPU0_GOV" ]; then
        ORIG_CPU0_GOV=$(cat "$CPU0_GOV")
        log "CPU0 original governor: $ORIG_CPU0_GOV"

        # Check if performance governor is available
        if grep -q "performance" "$CPU0_AVAIL_GOV" 2>/dev/null; then
            echo "performance" > "$CPU0_GOV"
            log "CPU0 set to: performance"
        else
            warn "Performance governor not available, available: $(cat "$CPU0_AVAIL_GOV")"
        fi
    else
        warn "CPU0 cpufreq not available"
    fi

    # Save and set CPU1 governor
    if [ -f "$CPU1_GOV" ]; then
        ORIG_CPU1_GOV=$(cat "$CPU1_GOV")
        log "CPU1 original governor: $ORIG_CPU1_GOV"
        echo "performance" > "$CPU1_GOV" 2>/dev/null || true
        log "CPU1 set to: performance"
    fi

    # GPU - try devfreq first, then direct clock control
    if [ -d "$GPU_DEVFREQ" ]; then
        if [ -f "$GPU_DEVFREQ/governor" ]; then
            ORIG_GPU_GOV=$(cat "$GPU_DEVFREQ/governor")
            log "GPU original governor: $ORIG_GPU_GOV"

            # Set to performance or highest frequency
            if echo "performance" > "$GPU_DEVFREQ/governor" 2>/dev/null; then
                log "GPU set to: performance governor"
            elif [ -f "$GPU_DEVFREQ/max_freq" ]; then
                local max_freq=$(cat "$GPU_DEVFREQ/max_freq")
                echo "$max_freq" > "$GPU_DEVFREQ/min_freq" 2>/dev/null || true
                log "GPU min_freq locked to max: $((max_freq / 1000000)) MHz"
            fi
        fi
    else
        info "GPU devfreq not available - GPU may run at fixed frequency"
    fi

    # Wait for frequencies to stabilize
    sleep 1

    # Verify the change
    show_frequencies
}

verify_drm() {
    log "Verifying DRM subsystem..."

    if [ ! -e /dev/dri/card0 ]; then
        err "No DRM device found at /dev/dri/card0"
        return 1
    fi

    if [ ! -r /dev/dri/card0 ]; then
        warn "/dev/dri/card0 not readable, trying to fix permissions..."
        chmod 666 /dev/dri/card0 2>/dev/null || true
    fi

    log "DRM device: $(ls -la /dev/dri/)"
    return 0
}

run_modetest_info() {
    log "=== Running modetest info ==="

    if ! command -v modetest &>/dev/null; then
        err "modetest not found in PATH"
        return 1
    fi

    echo ""
    log "--- Connectors ---"
    modetest -c 2>&1 || warn "modetest -c failed"

    echo ""
    log "--- Encoders ---"
    modetest -e 2>&1 || warn "modetest -e failed"

    echo ""
    log "--- CRTCs ---"
    modetest -r 2>&1 || warn "modetest -r failed"

    echo ""
    log "--- Planes ---"
    modetest -p 2>&1 || warn "modetest -p failed"

    echo ""
    log "--- Framebuffers ---"
    modetest -f 2>&1 || warn "modetest -f failed"

    return 0
}

run_modetest_patterns() {
    log "=== Running modetest pattern tests ==="

    # Find the active connector
    local connector=$(modetest -c 2>/dev/null | grep -E "connected" | head -1 | awk '{print $1}')
    if [ -z "$connector" ]; then
        warn "No connected display found"
        return 1
    fi

    log "Using connector: $connector"

    # Find the preferred mode
    local mode=$(modetest -c 2>/dev/null | grep -A20 "^$connector" | grep -E "^\s+[0-9]+x[0-9]+" | head -1 | awk '{print $1}')
    if [ -z "$mode" ]; then
        mode="1024x768"
        warn "Could not detect mode, using default: $mode"
    fi

    log "Using mode: $mode"

    # Test patterns
    local patterns="smpte tiles"
    for pattern in $patterns; do
        log "Testing pattern: $pattern"
        timeout 3 modetest -s "$connector:$mode@XR24" -P "$pattern" 2>&1 || true
        sleep 1
    done

    return 0
}

run_kmscube() {
    log "=== Running kmscube ==="

    if ! command -v kmscube &>/dev/null; then
        warn "kmscube not found in PATH"
        return 1
    fi

    local duration=5
    case "$MODE" in
        quick) duration=3 ;;
        full) duration=10 ;;
        benchmark) duration=30 ;;
    esac

    log "Running kmscube for ${duration}s..."
    show_frequencies

    # Sync and mark log before GPU operation (crash recovery point)
    log_to_file ">>> STARTING KMSCUBE (duration=${duration}s) <<<"
    sync

    # Run kmscube and capture FPS output
    timeout $duration kmscube 2>&1 | tee /tmp/kmscube-output.txt || true

    # Mark completion
    log_to_file ">>> KMSCUBE COMPLETED <<<"
    sync

    # Show final frequencies
    log "Post-test frequencies:"
    show_frequencies

    # Extract and display FPS statistics
    if [ -f /tmp/kmscube-output.txt ]; then
        local fps_lines=$(grep -E "fps" /tmp/kmscube-output.txt | tail -5)
        if [ -n "$fps_lines" ]; then
            log "FPS samples:"
            echo "$fps_lines"
            # Also save to log file
            echo "$fps_lines" >> "$TEST_LOG" 2>/dev/null || true
        fi
    fi

    return 0
}

run_kmscube_atomic() {
    log "=== Running kmscube with atomic modesetting ==="

    if ! command -v kmscube &>/dev/null; then
        warn "kmscube not found"
        return 1
    fi

    local duration=5
    case "$MODE" in
        benchmark) duration=15 ;;
    esac

    log "Running kmscube --atomic for ${duration}s..."

    # Sync and mark log before GPU operation (crash recovery point)
    log_to_file ">>> STARTING KMSCUBE ATOMIC (duration=${duration}s) <<<"
    sync

    timeout $duration kmscube --atomic 2>&1 || warn "Atomic mode may not be supported"

    log_to_file ">>> KMSCUBE ATOMIC COMPLETED <<<"
    sync

    return 0
}

run_stress_test() {
    log "=== Running GPU stress test ==="

    log "Initial frequencies:"
    show_frequencies

    # Run multiple kmscube instances in sequence
    for i in 1 2 3; do
        log "Stress iteration $i/3..."
        log_to_file ">>> STRESS TEST ITERATION $i/3 <<<"
        sync
        timeout 10 kmscube 2>&1 | grep -E "fps" | tail -1 || true
        show_frequencies
        sync
    done

    log_to_file ">>> STRESS TEST COMPLETED <<<"
    return 0
}

monitor_frequencies() {
    log "=== Monitoring frequencies during load ==="

    log_to_file ">>> STARTING FREQUENCY MONITOR WITH BACKGROUND KMSCUBE <<<"
    sync

    # Start kmscube in background
    kmscube &
    local kmscube_pid=$!

    # Monitor for 10 seconds
    for i in $(seq 1 10); do
        sleep 1
        echo "=== Second $i ==="
        show_frequencies
    done

    # Kill kmscube
    kill $kmscube_pid 2>/dev/null || true
    wait $kmscube_pid 2>/dev/null || true

    log_to_file ">>> FREQUENCY MONITOR COMPLETED <<<"
    sync

    return 0
}

print_summary() {
    echo ""
    log "=========================================="
    log "         DRM/KMS Test Summary"
    log "=========================================="

    # System info
    info "Kernel: $(uname -r)"
    info "Device: $(cat /sys/firmware/devicetree/base/model 2>/dev/null | tr -d '\0' || echo 'Unknown')"

    # DRM info
    if [ -d /sys/class/drm ]; then
        info "DRM cards: $(ls /sys/class/drm/ | grep -E '^card[0-9]+$' | tr '\n' ' ')"
    fi

    # GPU driver
    if [ -f /sys/class/drm/card0/device/driver/module/drivers/platform:adreno ]; then
        info "GPU driver: adreno"
    elif [ -L /sys/class/drm/card0/device/driver ]; then
        info "GPU driver: $(basename $(readlink /sys/class/drm/card0/device/driver))"
    fi

    show_frequencies

    log "=========================================="
    log "Tests completed!"
    log "=========================================="
}

# Initialize logging
init_logging() {
    # Check if log directory exists and is writable
    if [ ! -d "$LOG_DIR" ]; then
        warn "Log directory $LOG_DIR not found, trying /tmp"
        LOG_DIR="/tmp"
        DMESG_LOG="$LOG_DIR/${LOG_PREFIX}-dmesg-${TIMESTAMP}.log"
        TEST_LOG="$LOG_DIR/${LOG_PREFIX}-${TIMESTAMP}.log"
    fi

    # Try to make boot partition writable if needed
    if [ "$LOG_DIR" = "/mnt/boot" ]; then
        mount -o remount,rw /mnt/boot 2>/dev/null || true
    fi

    # Initialize log files
    echo "=========================================" > "$TEST_LOG"
    echo "HP TouchPad DRM/KMS Test Log" >> "$TEST_LOG"
    echo "Started: $(date)" >> "$TEST_LOG"
    echo "Mode: $MODE" >> "$TEST_LOG"
    echo "Kernel: $(uname -r)" >> "$TEST_LOG"
    echo "=========================================" >> "$TEST_LOG"

    echo "=========================================" > "$DMESG_LOG"
    echo "HP TouchPad DRM/KMS Dmesg Log" >> "$DMESG_LOG"
    echo "Started: $(date)" >> "$DMESG_LOG"
    echo "=========================================" >> "$DMESG_LOG"

    sync

    log "Logging initialized"
    log "  Test log:  $TEST_LOG"
    log "  Dmesg log: $DMESG_LOG"
}

# Main
main() {
    echo ""
    log "=========================================="
    log "  HP TouchPad DRM/KMS Test Suite"
    log "=========================================="
    log "Mode: $MODE"
    echo ""

    # Initialize persistent logging
    init_logging

    # Save initial dmesg state
    save_dmesg "PRE-TEST (initial state)"

    # Start background dmesg logger (saves every second)
    start_dmesg_logger 1

    # Verify DRM is available
    if ! verify_drm; then
        err "DRM not available. Check that the kernel has DRM support enabled."
        save_dmesg "DRM VERIFICATION FAILED"
        exit 1
    fi

    save_dmesg "POST-DRM-VERIFY"

    # Set max performance
    set_max_performance
    save_dmesg "POST-FREQ-SETUP"

    # Run tests based on mode
    case "$MODE" in
        quick)
            run_modetest_info
            save_dmesg "POST-MODETEST-INFO"
            run_kmscube
            save_dmesg "POST-KMSCUBE"
            ;;
        full)
            run_modetest_info
            save_dmesg "POST-MODETEST-INFO"
            run_modetest_patterns
            save_dmesg "POST-MODETEST-PATTERNS"
            run_kmscube
            save_dmesg "POST-KMSCUBE"
            run_kmscube_atomic
            save_dmesg "POST-KMSCUBE-ATOMIC"
            ;;
        benchmark)
            run_modetest_info
            save_dmesg "POST-MODETEST-INFO"
            run_kmscube
            save_dmesg "POST-KMSCUBE"
            run_stress_test
            save_dmesg "POST-STRESS-TEST"
            monitor_frequencies
            save_dmesg "POST-MONITOR"
            ;;
        *)
            warn "Unknown mode: $MODE"
            warn "Available modes: quick, full, benchmark"
            exit 1
            ;;
    esac

    save_dmesg "TESTS-COMPLETE"
    print_summary
}

main "$@"
SCRIPT_EOF

    chmod +x "$dest/run-drm-tests.sh"
    status "Created run-drm-tests.sh"
}

# Copy library and its symlinks
copy_lib() {
    local src="$1"
    local dest_dir="$2"
    local basename=$(basename "$src")

    # Copy the actual file
    cp -P "$src" "$dest_dir/" 2>/dev/null || true

    # If it's a symlink, also copy what it points to
    if [ -L "$src" ]; then
        local target=$(readlink -f "$src")
        if [ -f "$target" ]; then
            cp "$target" "$dest_dir/"
        fi
    fi

    # Copy versioned variants (e.g., libdrm.so.2, libdrm.so.2.131.0)
    local dir=$(dirname "$src")
    local base=${basename%.so*}
    for f in "$dir/$base.so"*; do
        [ -e "$f" ] && cp -P "$f" "$dest_dir/" 2>/dev/null || true
    done
}

# Main
main() {
    echo ""
    echo "=========================================="
    echo "  Pack DRM Test Tools for HP TouchPad"
    echo "=========================================="
    echo ""

    # Check for required build directories
    local missing=0

    if [ ! -d "$LIBDRM_BUILD" ]; then
        error "libdrm build not found at: $LIBDRM_BUILD"
        missing=1
    fi

    if [ ! -d "$MESA_BUILD" ]; then
        error "Mesa build not found at: $MESA_BUILD"
        missing=1
    fi

    if [ ! -d "$KMSCUBE_BUILD" ]; then
        error "kmscube build not found at: $KMSCUBE_BUILD"
        missing=1
    fi

    if [ $missing -eq 1 ]; then
        echo ""
        echo "Expected build directories:"
        echo "  libdrm:  $LIBDRM_BUILD"
        echo "  Mesa:    $MESA_BUILD"
        echo "  kmscube: $KMSCUBE_BUILD"
        exit 1
    fi

    status "Using libdrm build: $LIBDRM_BUILD"
    status "Using Mesa build:   $MESA_BUILD"
    status "Using kmscube build: $KMSCUBE_BUILD"

    # Create package directory
    rm -rf "$PACKAGE_DIR"
    mkdir -p "$PACKAGE_DIR/bin"
    mkdir -p "$PACKAGE_DIR/lib"
    mkdir -p "$BUILD_OUTPUT"

    # Copy modetest
    local modetest_bin="$LIBDRM_BUILD/tests/modetest/modetest"
    if [ -f "$modetest_bin" ]; then
        cp "$modetest_bin" "$PACKAGE_DIR/bin/"
        status "Copied modetest"
    else
        warn "modetest not found at: $modetest_bin"
    fi

    # Copy kmscube
    local kmscube_bin="$KMSCUBE_BUILD/kmscube"
    if [ -f "$kmscube_bin" ]; then
        cp "$kmscube_bin" "$PACKAGE_DIR/bin/"
        status "Copied kmscube"
    else
        warn "kmscube not found at: $kmscube_bin"
    fi

    # Check if we have at least one tool
    if [ ! -f "$PACKAGE_DIR/bin/modetest" ] && [ ! -f "$PACKAGE_DIR/bin/kmscube" ]; then
        error "Neither modetest nor kmscube found. Cannot create package."
        exit 1
    fi

    # Copy libraries from build directories
    status "Copying libraries..."

    # libdrm
    copy_lib "$LIBDRM_BUILD/libdrm.so" "$PACKAGE_DIR/lib"
    status "  libdrm.so"

    # Mesa libraries
    copy_lib "$MESA_BUILD/src/gbm/libgbm.so" "$PACKAGE_DIR/lib"
    status "  libgbm.so"

    copy_lib "$MESA_BUILD/src/egl/libEGL.so" "$PACKAGE_DIR/lib"
    status "  libEGL.so"

    copy_lib "$MESA_BUILD/src/mesa/glapi/es2api/libGLESv2.so" "$PACKAGE_DIR/lib"
    status "  libGLESv2.so"

    # Check for additional Mesa libs that might be needed
    if [ -f "$MESA_BUILD/src/mesa/glapi/libglapi.so" ]; then
        copy_lib "$MESA_BUILD/src/mesa/glapi/libglapi.so" "$PACKAGE_DIR/lib"
        status "  libglapi.so"
    fi

    # libgallium (Mesa's gallium driver, required by libEGL)
    if [ -f "$MESA_BUILD/src/gallium/targets/dri/libgallium-26.0.0-devel.so" ]; then
        cp "$MESA_BUILD/src/gallium/targets/dri/libgallium-26.0.0-devel.so" "$PACKAGE_DIR/lib/"
        status "  libgallium-26.0.0-devel.so"
    fi

    # GBM DRI backend (required for GBM to work)
    mkdir -p "$PACKAGE_DIR/lib/gbm"
    if [ -f "$MESA_BUILD/src/gbm/backends/dri/dri_gbm.so" ]; then
        cp "$MESA_BUILD/src/gbm/backends/dri/dri_gbm.so" "$PACKAGE_DIR/lib/gbm/"
        status "  gbm/dri_gbm.so (GBM DRI backend)"
    fi

    # libz (from Mesa's zlib subproject)
    if [ -d "$MESA_BUILD/subprojects/zlib-1.3.1" ]; then
        copy_lib "$MESA_BUILD/subprojects/zlib-1.3.1/libz.so" "$PACKAGE_DIR/lib"
        status "  libz.so (from Mesa subproject)"
    fi

    # System libraries from ARM toolchain
    TOOLCHAIN_LIB="/usr/arm-linux-gnueabihf/lib"
    if [ -d "$TOOLCHAIN_LIB" ]; then
        # libgcc_s (required by Mesa libs)
        if [ -f "$TOOLCHAIN_LIB/libgcc_s.so.1" ]; then
            cp "$TOOLCHAIN_LIB/libgcc_s.so.1" "$PACKAGE_DIR/lib/"
            status "  libgcc_s.so.1"
        fi

        # libstdc++ (required by libgallium)
        if [ -f "$TOOLCHAIN_LIB/libstdc++.so.6" ]; then
            cp -P "$TOOLCHAIN_LIB/libstdc++.so.6"* "$PACKAGE_DIR/lib/"
            status "  libstdc++.so.6"
        fi
    fi

    # List what we packaged
    echo ""
    status "Libraries packaged:"
    ls -la "$PACKAGE_DIR/lib/" | grep -v "^total" | grep -v "^d"

    # Create the test runner script
    create_test_runner "$PACKAGE_DIR"

    # Create debug script for kmscube with aggressive logging
    cat > "$PACKAGE_DIR/debug-kmscube.sh" << 'DEBUG_EOF'
#!/bin/sh
# Debug script for kmscube with aggressive logging
# Captures kernel messages, interconnect, GPU freq, memory in real-time
# All logs written continuously to /mnt/boot to survive USB crashes

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:$LD_LIBRARY_PATH"
export GBM_BACKENDS_PATH="$SCRIPT_DIR/lib/gbm"
export PATH="$SCRIPT_DIR/bin:$PATH"

# Log files on boot partition (survives crash)
LOG_DIR="/mnt/boot"
TS=$(date +%Y%m%d-%H%M%S)
DMESG_LOG="$LOG_DIR/debug-dmesg-$TS.log"
TRACE_LOG="$LOG_DIR/debug-trace-$TS.log"
MONITOR_LOG="$LOG_DIR/debug-monitor-$TS.log"

# Timestamp function (BusyBox compatible - no %N support)
ts() {
    date '+%H:%M:%S'
}

# Make boot partition writable early
mount -o remount,rw /mnt/boot 2>/dev/null || true

echo "=== Debug kmscube script ===" | tee "$TRACE_LOG"
echo "Started at: $(date)" | tee -a "$TRACE_LOG"
echo "Logs will be saved to: $LOG_DIR" | tee -a "$TRACE_LOG"
echo "  TRACE:   $TRACE_LOG" | tee -a "$TRACE_LOG"
echo "  DMESG:   $DMESG_LOG" | tee -a "$TRACE_LOG"
echo "  MONITOR: $MONITOR_LOG" | tee -a "$TRACE_LOG"
sync

# Save initial dmesg
echo ">>> INITIAL DMESG at $(date) <<<" > "$DMESG_LOG"
dmesg >> "$DMESG_LOG"
sync
echo "[$(ts)] Saved initial dmesg" | tee -a "$TRACE_LOG"
sync

# Start continuous dmesg logging (every 0.5s)
echo "[$(ts)] Starting continuous dmesg capture..." | tee -a "$TRACE_LOG"
sync
(
    while true; do
        dmesg -c 2>/dev/null | while IFS= read -r line; do
            echo "[$(ts)] $line" >> "$DMESG_LOG"
        done
        sync
        sleep 1
    done
) &
DMESG_PID=$!
echo "[$(ts)] dmesg logger PID: $DMESG_PID" | tee -a "$TRACE_LOG"
sync

# Function to capture system state
capture_state() {
    local tag="$1"
    echo ""
    echo "=== $tag at $(ts) ==="

    # Memory info
    echo "-- Memory --"
    grep -E 'MemFree|MemAvailable|Buffers|Cached|Shmem' /proc/meminfo 2>/dev/null
    # Show memory usage summary
    free 2>/dev/null || true

    # GPU/Z180 clock frequency (check both debugfs and sysfs)
    echo "-- GPU Clocks --"
    # Try debugfs first
    for clk in /sys/kernel/debug/clk/gfx3d_clk/clk_rate \
               /sys/kernel/debug/clk/gfx2d0_clk/clk_rate \
               /sys/kernel/debug/clk/gfx2d1_clk/clk_rate \
               /sys/kernel/debug/clk/z180_clk/clk_rate; do
        if [ -f "$clk" ]; then
            clkname=$(dirname "$clk")
            clkname=$(basename "$clkname")
            echo "  $clkname: $(cat "$clk" 2>/dev/null) Hz"
        fi
    done
    # Try OPP framework
    for opp in /sys/class/devfreq/*/cur_freq; do
        if [ -f "$opp" ]; then
            devname=$(dirname "$opp")
            devname=$(basename "$devname")
            freq=$(cat "$opp" 2>/dev/null)
            echo "  $devname: $freq Hz"
        fi
    done
    # If no clocks found
    if [ ! -d /sys/kernel/debug/clk ] && [ ! -d /sys/class/devfreq ]; then
        echo "  (no clock debug info available)"
    fi

    # Devfreq state (GPU frequency scaling)
    echo "-- Devfreq --"
    if ls /sys/class/devfreq/* >/dev/null 2>&1; then
        for df in /sys/class/devfreq/*; do
            if [ -d "$df" ]; then
                name=$(basename "$df")
                cur=$(cat "$df/cur_freq" 2>/dev/null || echo "N/A")
                gov=$(cat "$df/governor" 2>/dev/null || echo "N/A")
                min=$(cat "$df/min_freq" 2>/dev/null || echo "?")
                max=$(cat "$df/max_freq" 2>/dev/null || echo "?")
                echo "  $name: $cur Hz [$min-$max] (gov: $gov)"
            fi
        done
    else
        echo "  (no devfreq devices)"
    fi

    # Interconnect bandwidth summary
    echo "-- Interconnect --"
    if [ -f /sys/kernel/debug/interconnect/interconnect_summary ]; then
        cat /sys/kernel/debug/interconnect/interconnect_summary 2>/dev/null
    elif [ -d /sys/kernel/debug/interconnect ]; then
        echo "  Interconnect nodes:"
        ls /sys/kernel/debug/interconnect/ 2>/dev/null
    else
        echo "  (debugfs not available - build with CONFIG_DEBUG_FS=y)"
    fi

    # USB gadget state - detailed
    echo "-- USB Gadget --"
    for udc in /sys/class/udc/*; do
        if [ -d "$udc" ]; then
            name=$(basename "$udc")
            state=$(cat "$udc/state" 2>/dev/null || echo "N/A")
            speed=$(cat "$udc/current_speed" 2>/dev/null || echo "?")
            echo "  $name: state=$state speed=$speed"
        fi
    done
    # Check USB host controllers too
    echo "  USB host controllers:"
    for hc in /sys/bus/usb/devices/usb*; do
        if [ -d "$hc" ]; then
            prod=$(cat "$hc/product" 2>/dev/null || echo "unknown")
            speed=$(cat "$hc/speed" 2>/dev/null || echo "?")
            echo "    $(basename $hc): $prod ($speed Mbps)"
        fi
    done

    # DRM info from sysfs (works without debugfs)
    echo "-- DRM Info --"
    for card in /sys/class/drm/card*; do
        if [ -d "$card" ] && [ ! -L "$card" ]; then
            cardname=$(basename "$card")
            echo "  $cardname:"
            # Show connectors
            for conn in "$card"/*-*/status; do
                if [ -f "$conn" ]; then
                    conndir=$(dirname "$conn")
                    connname=$(basename "$conndir")
                    status=$(cat "$conn" 2>/dev/null)
                    echo "    $connname: $status"
                fi
            done
        fi
    done

    # DRM framebuffer info (if debugfs available)
    if [ -f /sys/kernel/debug/dri/0/framebuffer ]; then
        echo "-- DRM Framebuffers (debugfs) --"
        cat /sys/kernel/debug/dri/0/framebuffer 2>/dev/null
    fi

    # Memory mappings for display hardware
    echo "-- Display Memory Regions (iomem) --"
    grep -iE 'mdp|lcdc|display|hdmi|dsi|gpu|kgsl|z180|smi|mmss' /proc/iomem 2>/dev/null || echo "  (none found)"

    # Interrupts related to display/USB
    echo "-- Relevant IRQs --"
    grep -iE 'mdp|lcdc|usb|ci_hdrc|dwc|msm_otg' /proc/interrupts 2>/dev/null || echo "  (none found)"
}

# Start continuous monitoring (captures state every 2 seconds)
echo "[$(ts)] Starting continuous monitoring..." | tee -a "$TRACE_LOG"
sync
echo ">>> CONTINUOUS MONITOR LOG <<<" > "$MONITOR_LOG"
(
    count=0
    while true; do
        capture_state "SAMPLE $count" >> "$MONITOR_LOG" 2>&1
        sync
        count=$((count + 1))
        sleep 2
    done
) &
MONITOR_PID=$!
echo "[$(ts)] monitor logger PID: $MONITOR_PID" | tee -a "$TRACE_LOG"
sync

# Trap to save logs on exit
cleanup() {
    echo "[$(ts)] CLEANUP - saving final state..." | tee -a "$TRACE_LOG"
    kill $DMESG_PID 2>/dev/null
    kill $MONITOR_PID 2>/dev/null
    echo ">>> FINAL DMESG <<<" >> "$DMESG_LOG"
    dmesg >> "$DMESG_LOG"
    echo ">>> FINAL STATE <<<" >> "$MONITOR_LOG"
    capture_state "FINAL" >> "$MONITOR_LOG" 2>&1
    sync
    echo "[$(ts)] Logs saved to $LOG_DIR" | tee -a "$TRACE_LOG"
}
trap cleanup EXIT INT TERM

echo "[$(ts)] === INITIAL SYSTEM STATE ===" | tee -a "$TRACE_LOG"
capture_state "INITIAL" | tee -a "$TRACE_LOG"
sync

# Check DRM state
echo "[$(ts)] === DRM DEVICES ===" | tee -a "$TRACE_LOG"
ls -la /dev/dri/ 2>&1 | tee -a "$TRACE_LOG"
sync

# Check if debugfs is mounted
echo "[$(ts)] === DEBUGFS CHECK ===" | tee -a "$TRACE_LOG"
if [ ! -d /sys/kernel/debug ]; then
    echo "[$(ts)] Creating /sys/kernel/debug..." | tee -a "$TRACE_LOG"
    mkdir -p /sys/kernel/debug 2>/dev/null || true
fi
if [ ! -f /sys/kernel/debug/interconnect/interconnect_summary ]; then
    echo "[$(ts)] Mounting debugfs..." | tee -a "$TRACE_LOG"
    mount -t debugfs debugfs /sys/kernel/debug 2>&1 | tee -a "$TRACE_LOG"
fi
if [ -d /sys/kernel/debug ]; then
    echo "[$(ts)] Debugfs contents:" | tee -a "$TRACE_LOG"
    ls /sys/kernel/debug/ 2>&1 | tee -a "$TRACE_LOG"
else
    echo "[$(ts)] Debugfs not available (CONFIG_DEBUG_FS not enabled?)" | tee -a "$TRACE_LOG"
fi
sync

# Run modetest info first (safe, no mode change)
echo "[$(ts)] === MODETEST INFO ===" | tee -a "$TRACE_LOG"
sync
$SCRIPT_DIR/bin/modetest -M msm -c 2>&1 | head -n 30 | tee -a "$TRACE_LOG"
sync

# Show connector/encoder/crtc IDs
echo "[$(ts)] === MODETEST PLANES ===" | tee -a "$TRACE_LOG"
$SCRIPT_DIR/bin/modetest -M msm -p 2>&1 | head -n 50 | tee -a "$TRACE_LOG"
sync

echo "[$(ts)] === STARTING KMSCUBE ===" | tee -a "$TRACE_LOG"
echo "[$(ts)] Will run for 10 seconds then exit cleanly" | tee -a "$TRACE_LOG"
sync

# Run kmscube in background with manual timeout (BusyBox compatible)
echo "[$(ts)] Launching kmscube..." | tee -a "$TRACE_LOG"
sync

$SCRIPT_DIR/bin/kmscube 2>&1 &
KMSCUBE_PID=$!
echo "[$(ts)] kmscube PID: $KMSCUBE_PID" | tee -a "$TRACE_LOG"
sync

# Monitor during kmscube run
for i in 1 2 3 4 5 6 7 8 9 10; do
    sleep 1
    memfree=$(awk '/MemFree/ {print $2}' /proc/meminfo 2>/dev/null)
    usbstate=$(cat /sys/class/udc/ci_hdrc.0/state 2>/dev/null || echo "?")
    echo "[$(ts)] kmscube ($i/10): MemFree=${memfree}kB USB=$usbstate" | tee -a "$TRACE_LOG"
    sync
done

echo "[$(ts)] Sending SIGTERM to kmscube..." | tee -a "$TRACE_LOG"
sync
kill $KMSCUBE_PID 2>/dev/null
sleep 1
echo "[$(ts)] Sending SIGKILL to kmscube..." | tee -a "$TRACE_LOG"
sync
kill -9 $KMSCUBE_PID 2>/dev/null
wait $KMSCUBE_PID 2>/dev/null
KMSCUBE_EXIT=$?

echo "[$(ts)] kmscube exited with code: $KMSCUBE_EXIT" | tee -a "$TRACE_LOG"
sync

echo "[$(ts)] === POST-KMSCUBE STATE ===" | tee -a "$TRACE_LOG"
capture_state "POST-KMSCUBE" | tee -a "$TRACE_LOG"
sync

echo "[$(ts)] Waiting 3 seconds for system to settle..." | tee -a "$TRACE_LOG"
sleep 3
sync

echo "[$(ts)] === FINAL STATE ===" | tee -a "$TRACE_LOG"
capture_state "FINAL" | tee -a "$TRACE_LOG"
sync

echo "[$(ts)] === TEST COMPLETE ===" | tee -a "$TRACE_LOG"
echo "[$(ts)] If you see this, USB survived!" | tee -a "$TRACE_LOG"
sync

echo ""
echo "============================================"
echo "Logs saved to:"
echo "  $TRACE_LOG   (main trace)"
echo "  $DMESG_LOG   (kernel messages)"
echo "  $MONITOR_LOG (continuous monitoring)"
echo "============================================"
DEBUG_EOF
    chmod +x "$PACKAGE_DIR/debug-kmscube.sh"
    status "Created debug-kmscube.sh"

    # Create simpler diagnostic script to isolate USB crash cause
    cat > "$PACKAGE_DIR/diagnose-drm.sh" << 'DIAG_EOF'
#!/bin/sh
# Diagnostic script to isolate USB crash cause
# Tests DRM operations one by one with aggressive logging

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:$LD_LIBRARY_PATH"
export GBM_BACKENDS_PATH="$SCRIPT_DIR/lib/gbm"
export PATH="$SCRIPT_DIR/bin:$PATH"

# Mesa/Gallium debugging - check which driver is actually used
export MESA_DEBUG=1
export LIBGL_DEBUG=verbose

# Uncomment to force specific driver:
# export GALLIUM_DRIVER=freedreno   # Force freedreno HW driver
# export GALLIUM_DRIVER=llvmpipe    # Force software rendering
# export LIBGL_ALWAYS_SOFTWARE=1    # Force software rendering

LOG_DIR="/mnt/boot"
mount -o remount,rw /mnt/boot 2>/dev/null || true

log() {
    echo "[$(date '+%H:%M:%S')] $*"
    echo "[$(date '+%H:%M:%S')] $*" >> "$LOG_DIR/diagnose.log"
    sync
}

test_passed() {
    log "TEST PASSED: $1"
    sleep 2
}

test_failed() {
    log "TEST FAILED: $1"
}

echo "" > "$LOG_DIR/diagnose.log"
log "=== DRM USB CRASH DIAGNOSTICS ==="
log "Testing each operation separately to find the crash trigger"
log ""

case "$1" in
    1|info)
        log "TEST 1: modetest -c (info only, no mode changes)"
        $SCRIPT_DIR/bin/modetest -M msm -c
        test_passed "modetest info"
        ;;
    2|pattern)
        log "TEST 2: modetest pattern for 3 seconds"
        log "Running modetest -P..."
        $SCRIPT_DIR/bin/modetest -M msm -s 44@39:1024x768 -P 40@39:1024x768@XR24 &
        MTEST_PID=$!
        log "modetest PID: $MTEST_PID"
        sleep 3
        log "Killing modetest..."
        kill $MTEST_PID 2>/dev/null
        sleep 1
        kill -9 $MTEST_PID 2>/dev/null
        test_passed "modetest pattern"
        ;;
    3|open)
        log "TEST 3: Open/close DRM device multiple times"
        for i in 1 2 3 4 5; do
            log "Open/close iteration $i..."
            $SCRIPT_DIR/bin/modetest -M msm -c > /dev/null 2>&1
            sync
        done
        test_passed "DRM open/close"
        ;;
    4|cma)
        log "TEST 4: CMA memory state"
        log "Before:"
        grep -i cma /proc/meminfo
        log "After modetest pattern:"
        $SCRIPT_DIR/bin/modetest -M msm -s 44@39:1024x768 &
        sleep 2
        grep -i cma /proc/meminfo
        kill %1 2>/dev/null
        sleep 1
        log "After cleanup:"
        grep -i cma /proc/meminfo
        test_passed "CMA memory"
        ;;
    5|kmscube)
        log "TEST 5: kmscube for 5 seconds"
        log "Starting kmscube..."
        $SCRIPT_DIR/bin/kmscube 2>&1 &
        KC_PID=$!
        log "kmscube PID: $KC_PID"
        sleep 5
        log "Sending SIGTERM to kmscube..."
        kill $KC_PID 2>/dev/null
        sleep 1
        log "Sending SIGKILL to kmscube..."
        kill -9 $KC_PID 2>/dev/null
        wait $KC_PID 2>/dev/null
        log "kmscube exit code: $?"
        test_passed "kmscube"
        ;;
    7|renderer)
        log "TEST 7: Check renderer info"
        log "Running kmscube briefly to check renderer..."
        # Capture kmscube output which shows renderer info
        $SCRIPT_DIR/bin/kmscube 2>&1 | head -n 20 &
        KC_PID=$!
        sleep 3
        kill $KC_PID 2>/dev/null
        log ""
        log "To force hardware rendering, set:"
        log "  export GALLIUM_DRIVER=freedreno"
        log ""
        log "To force software rendering (for comparison):"
        log "  export LIBGL_ALWAYS_SOFTWARE=1"
        test_passed "renderer check"
        ;;
    8|hw)
        log "TEST 8: Force HARDWARE rendering (freedreno)"
        export GALLIUM_DRIVER=freedreno
        log "GALLIUM_DRIVER=freedreno"
        log "Starting kmscube with hardware acceleration..."
        $SCRIPT_DIR/bin/kmscube 2>&1 &
        KC_PID=$!
        log "kmscube PID: $KC_PID"
        sleep 5
        log "Killing kmscube..."
        kill $KC_PID 2>/dev/null
        sleep 1
        kill -9 $KC_PID 2>/dev/null
        test_passed "hardware rendering"
        ;;
    9|sw)
        log "TEST 9: Force SOFTWARE rendering (llvmpipe)"
        export LIBGL_ALWAYS_SOFTWARE=1
        log "LIBGL_ALWAYS_SOFTWARE=1"
        log "Starting kmscube with software rendering..."
        $SCRIPT_DIR/bin/kmscube 2>&1 &
        KC_PID=$!
        log "kmscube PID: $KC_PID"
        sleep 5
        log "Killing kmscube..."
        kill $KC_PID 2>/dev/null
        sleep 1
        kill -9 $KC_PID 2>/dev/null
        test_passed "software rendering"
        ;;
    6|modeset)
        log "TEST 6: Mode setting only (no pattern)"
        log "Setting mode..."
        $SCRIPT_DIR/bin/modetest -M msm -s 44@39:1024x768 &
        MT_PID=$!
        log "modetest PID: $MT_PID"
        sleep 3
        log "Killing modetest..."
        kill $MT_PID 2>/dev/null
        sleep 1
        kill -9 $MT_PID 2>/dev/null
        test_passed "mode setting"
        ;;
    all)
        log "Running all tests in sequence..."
        log ""
        $0 1 && $0 2 && $0 3 && $0 4 && $0 6 && $0 5
        log "=== ALL TESTS COMPLETE ==="
        ;;
    *)
        echo "Usage: $0 <test>"
        echo ""
        echo "Tests (USB crash isolation):"
        echo "  1|info    - modetest info (no mode changes)"
        echo "  2|pattern - modetest with pattern (3 sec)"
        echo "  3|open    - Open/close DRM device 5 times"
        echo "  4|cma     - Check CMA memory"
        echo "  5|kmscube - Run kmscube (5 sec)"
        echo "  6|modeset - Mode setting without pattern"
        echo ""
        echo "Tests (renderer):"
        echo "  7|renderer - Check which renderer kmscube uses"
        echo "  8|hw       - Force hardware rendering (freedreno)"
        echo "  9|sw       - Force software rendering (llvmpipe)"
        echo ""
        echo "  all       - Run crash isolation tests in order"
        echo ""
        echo "Run tests one by one to find which causes USB crash"
        echo "Logs saved to: $LOG_DIR/diagnose.log"
        ;;
esac
DIAG_EOF
    chmod +x "$PACKAGE_DIR/diagnose-drm.sh"
    status "Created diagnose-drm.sh"

    # Create a simple README
    cat > "$PACKAGE_DIR/README.txt" << 'EOF'
HP TouchPad DRM/KMS Test Suite
==============================

Contents:
  bin/       - Test binaries (modetest, kmscube)
  lib/       - Required shared libraries
  run-drm-tests.sh - Test runner script

Installation:
  1. Extract to /tmp or /home/root on the device
  2. Run: ./run-drm-tests.sh [quick|full|benchmark]

Modes:
  quick     - Basic info and short kmscube test (default)
  full      - All modetest info + patterns + kmscube tests
  benchmark - Extended tests with frequency monitoring

The script will:
  - Set CPU governor to 'performance' (max speed)
  - Set GPU to maximum frequency if devfreq is available
  - Run the requested tests
  - Monitor frequency changes
  - Restore original settings on exit

Requirements:
  - DRM kernel driver loaded (adreno/msm)
  - /dev/dri/card0 present and accessible
  - Root access (for cpufreq/devfreq control)
  - GPU firmware in initramfs (included in kernel image)
EOF

    # Create tarball
    status "Creating package tarball..."
    cd "$BUILD_OUTPUT"
    tar czf drm-tests.tar.gz drm-tests/

    echo ""
    status "=========================================="
    status "Package created: $PACKAGE_TAR"
    status "Size: $(du -h "$PACKAGE_TAR" | cut -f1)"
    status "=========================================="
    echo ""
    status "To deploy to device, run:"
    status "  ./scripts/deploy-drm-tests.sh"
    echo ""
}

main "$@"
