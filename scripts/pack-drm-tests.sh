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

# GPU firmware location (from webOS doctor)
FIRMWARE_DIR="$PARENT_DIR/doctor305/untouched-rootfs/lib/firmware"

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

# Background dmesg logger - saves every N seconds
start_dmesg_logger() {
    local interval="${1:-5}"
    (
        count=0
        while true; do
            sleep $interval
            count=$((count + 1))
            echo "" >> "$DMESG_LOG"
            echo "========== DMESG CHECKPOINT $count [$(date '+%Y-%m-%d %H:%M:%S')] ==========" >> "$DMESG_LOG"
            dmesg | tail -100 >> "$DMESG_LOG" 2>/dev/null || true
            sync
        done
    ) &
    LOGGER_PID=$!
    log "Started background dmesg logger (PID: $LOGGER_PID, interval: ${interval}s)"
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

install_firmware() {
    log "Installing GPU firmware..."

    # Firmware destination
    local fw_dest="/lib/firmware"

    # Create directory if needed
    mkdir -p "$fw_dest" 2>/dev/null || true

    # Check if we have firmware in our package
    if [ ! -d "$SCRIPT_DIR/firmware" ]; then
        warn "No firmware directory in package"
        return 1
    fi

    # List of required firmware files
    local fw_files="yamato_pfp.fw yamato_pm4.fw leia_pfp_470.fw leia_pm4_470.fw"
    local installed=0
    local skipped=0

    for fw in $fw_files; do
        if [ -f "$SCRIPT_DIR/firmware/$fw" ]; then
            if [ -f "$fw_dest/$fw" ]; then
                # Check if different
                if cmp -s "$SCRIPT_DIR/firmware/$fw" "$fw_dest/$fw"; then
                    info "  $fw (already installed)"
                    skipped=$((skipped + 1))
                else
                    cp "$SCRIPT_DIR/firmware/$fw" "$fw_dest/"
                    log "  $fw (updated)"
                    installed=$((installed + 1))
                fi
            else
                cp "$SCRIPT_DIR/firmware/$fw" "$fw_dest/"
                log "  $fw (installed)"
                installed=$((installed + 1))
            fi
        else
            warn "  $fw (not in package)"
        fi
    done

    log "Firmware: $installed installed, $skipped already present"

    # Sync to ensure firmware is on disk
    sync

    return 0
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

    # Install GPU firmware (required for Adreno/kgsl)
    install_firmware
    save_dmesg "POST-FIRMWARE-INSTALL"

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

    # List what we packaged
    echo ""
    status "Libraries packaged:"
    ls -la "$PACKAGE_DIR/lib/" | grep -v "^total" | grep -v "^d"

    # Copy GPU firmware files
    status "Copying GPU firmware..."
    mkdir -p "$PACKAGE_DIR/firmware"

    local fw_files="yamato_pfp.fw yamato_pm4.fw leia_pfp_470.fw leia_pm4_470.fw"
    for fw in $fw_files; do
        if [ -f "$FIRMWARE_DIR/$fw" ]; then
            cp "$FIRMWARE_DIR/$fw" "$PACKAGE_DIR/firmware/"
            status "  $fw"
        else
            warn "  $fw not found"
        fi
    done

    echo ""
    status "Firmware packaged:"
    ls -la "$PACKAGE_DIR/firmware/" | grep -v "^total" | grep -v "^d"

    # Create the test runner script
    create_test_runner "$PACKAGE_DIR"

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
  2. Run: ./run-drm-tests.sh [--quick|--full|--benchmark]

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
