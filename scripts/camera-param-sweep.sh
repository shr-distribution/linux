#!/bin/bash
#
# Camera Parameter Sweep Test Script
# Systematically tests various CSIPHY/VFE parameter combinations
# and records results in a markdown table for comparison
#
# Usage: ./scripts/camera-param-sweep.sh [--quick] [--full]
#

set -e

DEVICE_IP="172.16.42.2"
SSH_PORT="22"
TIMEOUT=10
RESULTS_FILE="/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/docs/camera-sweep-results.md"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_step() { echo -e "${GREEN}[STEP]${NC} $1"; }
log_info() { echo -e "${YELLOW}[INFO]${NC} $1"; }
log_result() { echo -e "${CYAN}[RESULT]${NC} $1"; }

# Run command on device via SSH
run_on_device() {
    local cmd="$1"
    ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=$TIMEOUT -p $SSH_PORT root@$DEVICE_IP "$cmd" 2>/dev/null
}

# Check device connectivity
check_device() {
    log_step "Checking device connectivity..."
    if timeout 3 bash -c "echo > /dev/tcp/$DEVICE_IP/$SSH_PORT" 2>/dev/null; then
        log_info "Device is reachable"
        return 0
    else
        echo "Cannot connect to device"
        return 1
    fi
}

# Set module parameters
set_params() {
    local hs_term="$1"
    local settle="$2"
    local ecc_disable="$3"
    local debug_poll="$4"

    run_on_device "
        # HS termination impedance (-1 = use default 0x0F)
        if [ -f /sys/module/qcom_camss/parameters/hs_term_imp_override ]; then
            echo $hs_term > /sys/module/qcom_camss/parameters/hs_term_imp_override
        fi

        # Settle count (-1 = use default 0x14)
        if [ -f /sys/module/qcom_camss/parameters/settle_cnt_override ]; then
            echo $settle > /sys/module/qcom_camss/parameters/settle_cnt_override
        fi

        # ECC disable (0=enabled, 1=disabled)
        if [ -f /sys/module/qcom_camss/parameters/csiphy_ecc_disable ]; then
            echo $ecc_disable > /sys/module/qcom_camss/parameters/csiphy_ecc_disable
        fi

        # Debug polling (0=off, 1=on)
        if [ -f /sys/module/qcom_camss/parameters/debug_poll_enable ]; then
            echo $debug_poll > /sys/module/qcom_camss/parameters/debug_poll_enable
        fi
    "
}

# Run single test and capture results
run_single_test() {
    local mode="$1"
    local test_name="$2"
    local timeout_sec=15

    # Clear dmesg before test
    run_on_device "dmesg -c > /dev/null 2>&1" || true

    # Run the test based on mode
    local result=""
    case $mode in
        pix)
            result=$(run_on_device "
                # Setup media pipeline for PIX mode
                media-ctl -r 2>/dev/null || true
                media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || true
                media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || true
                media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
                media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
                media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
                media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
                media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
                media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
                v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=1024,pixelformat=UYVY 2>&1 || true

                # Run capture with timeout
                timeout $timeout_sec v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=5 --stream-to=/tmp/test.raw 2>&1
                echo \"EXIT_CODE=\$?\"
            ")
            ;;
        raw)
            result=$(run_on_device "
                # Setup media pipeline for RDI/RAW mode
                media-ctl -r 2>/dev/null || true
                media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || true
                media-ctl -l '\"msm_csid1\":1->\"msm_vfe0_rdi0\":0[1]' 2>&1 || true
                media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/640x480]' 2>&1 || true
                media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/640x480]' 2>&1 || true
                media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/640x480]' 2>&1 || true
                media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/640x480]' 2>&1 || true
                media-ctl -V '\"msm_csid1\":1[fmt:UYVY8_1X16/640x480]' 2>&1 || true
                v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=UYVY 2>&1 || true

                # Run capture with timeout
                timeout $timeout_sec v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=5 --stream-to=/tmp/test.raw 2>&1
                echo \"EXIT_CODE=\$?\"
            ")
            ;;
    esac

    # Get dmesg output after test
    local dmesg_output=$(run_on_device "dmesg | tail -100")

    # Parse results
    local exit_code=$(echo "$result" | grep "EXIT_CODE=" | sed 's/EXIT_CODE=//')

    # Count various IRQ types from dmesg
    local sot_count=$(echo "$dmesg_output" | grep -c "SOT" 2>/dev/null || echo 0)
    local ecc_count=$(echo "$dmesg_output" | grep -c "ECC" 2>/dev/null || echo 0)
    local data_count=$(echo "$dmesg_output" | grep -c "DATA" 2>/dev/null || echo 0)
    local sof_count=$(echo "$dmesg_output" | grep -c "CAMIF_SOF\|SOF" 2>/dev/null || echo 0)
    local reg_upd=$(echo "$dmesg_output" | grep -c "REG_UPDATE" 2>/dev/null || echo 0)
    local timeout_err=$(echo "$dmesg_output" | grep -c "timeout" 2>/dev/null || echo 0)

    # Check for ping_pong changes (indicates frames written)
    local pp_changed=$(echo "$dmesg_output" | grep -c "PING_PONG changed" 2>/dev/null || echo 0)

    # Check capture file size
    local file_size=$(run_on_device "stat -c%s /tmp/test.raw 2>/dev/null || echo 0")

    # Determine success
    local status="FAIL"
    if [ "$exit_code" = "0" ] && [ "$file_size" -gt 0 ]; then
        status="OK"
    elif [ "$sof_count" -gt 0 ]; then
        status="SOF"
    elif [ "$sot_count" -gt 0 ]; then
        status="SOT"
    fi

    echo "$status|$sot_count|$ecc_count|$data_count|$sof_count|$reg_upd|$pp_changed|$timeout_err|$file_size"
}

# Main sweep test
run_sweep() {
    local quick="$1"

    log_step "Starting parameter sweep test..."

    # Create/update results file header
    cat > "$RESULTS_FILE" << 'EOF'
# Camera Parameter Sweep Results

## Test Environment
- Kernel: linux-6.18-tenderloin
- Device: HP TouchPad (APQ8060)
- Sensor: MT9M113 front camera
- Resolution: 1280x1024 UYVY (PIX mode), 640x480 UYVY (RAW mode)

## Parameter Descriptions
- **HS_TERM_IMP**: HS termination impedance (0x00-0x0F, -1=default 0x0F)
- **SETTLE_CNT**: MIPI settle count (0x00-0xFF, -1=default 0x14)
- **ECC**: ECC checking (0=enabled, 1=disabled)
- **Mode**: pix (VFE ISP) or raw (RDI bypass)

## Results Key
- **Status**: OK=success, SOF=got SOF no frames, SOT=got SOT only, FAIL=no activity
- **SOT**: Start of Transmission IRQ count
- **ECC**: ECC error IRQ count
- **DATA**: Data lane activity count
- **SOF**: VFE CAMIF_SOF count
- **REG**: REG_UPDATE completion count
- **PP**: Ping-pong toggle count (indicates frames written)
- **TO**: Timeout errors
- **Size**: Captured data size (bytes)

## Results Table

EOF
    echo "| HS_TERM | SETTLE | ECC | Mode | Status | SOT | ECC | DATA | SOF | REG | PP | TO | Size |" >> "$RESULTS_FILE"
    echo "|---------|--------|-----|------|--------|-----|-----|------|-----|-----|----|----|------|" >> "$RESULTS_FILE"

    # Define parameter sets to test
    if [ "$quick" = "quick" ]; then
        # Quick test - just a few key combinations
        hs_values=(-1 0x00 0x07)
        settle_values=(-1 0x28)
        ecc_values=(0 1)
        modes=(pix)
    else
        # Full test - comprehensive sweep
        hs_values=(-1 0x00 0x03 0x07 0x0B 0x0F)
        settle_values=(-1 0x14 0x20 0x28 0x40)
        ecc_values=(0 1)
        modes=(pix raw)
    fi

    local total_tests=0
    local passed_tests=0

    for mode in "${modes[@]}"; do
        for hs in "${hs_values[@]}"; do
            for settle in "${settle_values[@]}"; do
                for ecc in "${ecc_values[@]}"; do
                    total_tests=$((total_tests + 1))

                    local test_name="hs=${hs} settle=${settle} ecc=${ecc} mode=${mode}"
                    log_info "Test $total_tests: $test_name"

                    # Set parameters
                    set_params "$hs" "$settle" "$ecc" "0"

                    # Small delay for parameters to take effect
                    sleep 1

                    # Run test
                    local result=$(run_single_test "$mode" "$test_name")

                    # Parse result
                    IFS='|' read -r status sot ecc_cnt data sof reg pp to size <<< "$result"

                    # Format for display
                    local hs_disp=$([ "$hs" = "-1" ] && echo "def" || echo "$hs")
                    local settle_disp=$([ "$settle" = "-1" ] && echo "def" || echo "$settle")
                    local ecc_disp=$([ "$ecc" = "0" ] && echo "on" || echo "off")

                    # Log to console
                    log_result "  Status=$status SOT=$sot ECC=$ecc_cnt SOF=$sof PP=$pp Size=$size"

                    # Write to results file
                    echo "| $hs_disp | $settle_disp | $ecc_disp | $mode | $status | $sot | $ecc_cnt | $data | $sof | $reg | $pp | $to | $size |" >> "$RESULTS_FILE"

                    if [ "$status" = "OK" ]; then
                        passed_tests=$((passed_tests + 1))
                    fi

                    # Small delay between tests
                    sleep 2
                done
            done
        done
    done

    # Add summary
    cat >> "$RESULTS_FILE" << EOF

## Summary

- Total tests: $total_tests
- Passed: $passed_tests
- Failed: $((total_tests - passed_tests))

## Test Date
$(date -Iseconds)

## Git Commit
$(git -C /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin rev-parse --short HEAD 2>/dev/null || echo "unknown")
EOF

    log_step "Sweep complete: $passed_tests/$total_tests tests passed"
    log_info "Results saved to: $RESULTS_FILE"
}

# Reset parameters to defaults
reset_params() {
    log_step "Resetting parameters to defaults..."
    set_params "-1" "-1" "0" "0"
}

# Main
main() {
    local mode="full"

    for arg in "$@"; do
        case $arg in
            --quick)
                mode="quick"
                ;;
            --full)
                mode="full"
                ;;
            --reset)
                check_device || exit 1
                reset_params
                exit 0
                ;;
            --help|-h)
                echo "Usage: $0 [--quick] [--full] [--reset]"
                echo ""
                echo "Options:"
                echo "  --quick   Run quick sweep (fewer combinations)"
                echo "  --full    Run full comprehensive sweep (default)"
                echo "  --reset   Reset all parameters to defaults"
                exit 0
                ;;
        esac
    done

    check_device || exit 1
    run_sweep "$mode"
    reset_params
}

main "$@"
