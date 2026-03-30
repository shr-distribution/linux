#!/bin/bash
# Test camera workarounds for CAMIF_EOF issue
#
# The VFE CAMIF never receives EOF because:
# - ECC errors corrupt the MIPI data stream
# - Only ~240 of 968 lines arrive before timeout
# - MT9M113 doesn't send MIPI Frame Start/End short packets
#
# This script tests two workarounds:
# 1. software_sof_enable - triggers software SOF+REG_UPDATE at frame boundaries
# 2. ecc_disable - disables ECC checking to allow corrupted packets through

set -e

echo "=== Camera Workaround Test Script ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Warning: Some operations may need root. Run with sudo if needed."
fi

# Default test parameters
MODE="${1:-both}"
OUTPUT_DIR="${2:-/tmp/camera-test}"
FRAME_COUNT="${3:-5}"

mkdir -p "$OUTPUT_DIR"

echo "Mode: $MODE"
echo "Output: $OUTPUT_DIR"
echo "Frames: $FRAME_COUNT"
echo ""

# Function to configure pipeline
configure_pipeline() {
    echo "Configuring media pipeline..."

    # Reset all links (except IMMUTABLE ones)
    media-ctl -d /dev/media0 -r || true

    # Set up MT9M113 -> CSIPHY1 -> CSID1 -> VFE PIX pipeline
    # Format: UYVY8_1X16 640x480 (Context A preview)
    # NOTE: Sensor -> CSIPHY1 link is IMMUTABLE

    media-ctl -d /dev/media0 -l '"msm_csiphy1":1 -> "msm_csid1":0[1]'
    media-ctl -d /dev/media0 -l '"msm_csid1":4 -> "msm_vfe0_pix":0[1]'
    media-ctl -d /dev/media0 -l '"msm_vfe0_pix":1 -> "msm_vfe0_video3":0[1]'

    # Set formats on all pads (640x480 preview mode)
    media-ctl -d /dev/media0 --set-v4l2 '"mt9m114 ifp 4-003c":1[fmt:UYVY8_1X16/640x480]'
    media-ctl -d /dev/media0 --set-v4l2 '"msm_csiphy1":0[fmt:UYVY8_1X16/640x480]'
    media-ctl -d /dev/media0 --set-v4l2 '"msm_csiphy1":1[fmt:UYVY8_1X16/640x480]'
    media-ctl -d /dev/media0 --set-v4l2 '"msm_csid1":0[fmt:UYVY8_1X16/640x480]'
    media-ctl -d /dev/media0 --set-v4l2 '"msm_csid1":4[fmt:UYVY8_2X8/640x480]'
    media-ctl -d /dev/media0 --set-v4l2 '"msm_vfe0_pix":0[fmt:UYVY8_2X8/640x480]'
    media-ctl -d /dev/media0 --set-v4l2 '"msm_vfe0_pix":1[fmt:UYVY8_2X8/640x480]'

    echo "Pipeline configured."
}

# Function to capture frames
capture_frames() {
    local prefix="$1"
    local outfile="$OUTPUT_DIR/${prefix}_capture.raw"

    echo "Capturing $FRAME_COUNT frames to $outfile..."

    # 640x480 NV16 = 640 * 480 * 2 = 614400 bytes per frame (Y + UV planes)
    # VFE31 PIX mode outputs NV16/NV61 (YUV 4:2:2 semi-planar)
    timeout 10 v4l2-ctl -d /dev/video3 \
        --set-fmt-video=width=640,height=480,pixelformat=NV16 \
        --stream-mmap --stream-count=$FRAME_COUNT \
        --stream-to="$outfile" 2>&1 || true

    if [ -f "$outfile" ]; then
        local size=$(stat -c%s "$outfile" 2>/dev/null || echo 0)
        local expected=$((640 * 480 * 2 * FRAME_COUNT))
        echo "  Captured: $size bytes (expected: $expected)"

        # Check for non-zero data
        local nonzero=$(od -An -tx1 "$outfile" | head -1 | tr -d ' 0\n')
        if [ -n "$nonzero" ]; then
            echo "  Data: NON-ZERO (some valid data!)"
            hexdump -C "$outfile" | head -5
        else
            echo "  Data: ALL ZEROS"
        fi
    else
        echo "  No output file created"
    fi
}

# Function to show current module parameters
show_params() {
    echo ""
    echo "Current module parameters:"
    for param in software_sof_enable ecc_disable hs_term_imp_override settle_cnt_override calibration_mode; do
        local path="/sys/module/qcom_camss/parameters/$param"
        if [ -f "$path" ]; then
            echo "  $param = $(cat $path)"
        fi
    done
    # mt9m114 module params
    for param in mt9m113_pre_mipi_delay_ms; do
        local path="/sys/module/mt9m114/parameters/$param"
        if [ -f "$path" ]; then
            echo "  $param = $(cat $path)"
        fi
    done
}

# Test 1: Baseline (current settings)
test_baseline() {
    echo ""
    echo "=== Test 1: Baseline (current settings) ==="
    show_params
    configure_pipeline
    capture_frames "baseline"
    dmesg | tail -50 | grep -E "VFE|CSIPHY|CAMIF|IRQ" || true
}

# Test 2: With software SOF enabled
test_software_sof() {
    echo ""
    echo "=== Test 2: software_sof_enable=1 ==="
    echo 1 > /sys/module/qcom_camss/parameters/software_sof_enable
    echo 0 > /sys/module/qcom_camss/parameters/ecc_disable
    show_params
    configure_pipeline
    capture_frames "software_sof"
    dmesg | tail -50 | grep -E "VFE|CSIPHY|CAMIF|IRQ|SOF" || true
}

# Test 3: With ECC disabled
test_ecc_disable() {
    echo ""
    echo "=== Test 3: ecc_disable=1 ==="
    echo 0 > /sys/module/qcom_camss/parameters/software_sof_enable
    echo 1 > /sys/module/qcom_camss/parameters/ecc_disable
    show_params
    configure_pipeline
    capture_frames "ecc_disable"
    dmesg | tail -50 | grep -E "VFE|CSIPHY|CAMIF|IRQ|ECC" || true
}

# Test 4: With both workarounds
test_both() {
    echo ""
    echo "=== Test 4: software_sof_enable=1 + ecc_disable=1 ==="
    echo 1 > /sys/module/qcom_camss/parameters/software_sof_enable
    echo 1 > /sys/module/qcom_camss/parameters/ecc_disable
    show_params
    configure_pipeline
    capture_frames "both_workarounds"
    dmesg | tail -50 | grep -E "VFE|CSIPHY|CAMIF|IRQ" || true
}

# Reset to default
reset_params() {
    echo ""
    echo "=== Resetting to defaults ==="
    echo 0 > /sys/module/qcom_camss/parameters/software_sof_enable 2>/dev/null || true
    echo 0 > /sys/module/qcom_camss/parameters/ecc_disable 2>/dev/null || true
    show_params
}

# Run tests based on mode
case "$MODE" in
    baseline)
        test_baseline
        ;;
    sof)
        test_software_sof
        reset_params
        ;;
    ecc)
        test_ecc_disable
        reset_params
        ;;
    both)
        test_both
        reset_params
        ;;
    all)
        test_baseline
        test_software_sof
        test_ecc_disable
        test_both
        reset_params
        ;;
    *)
        echo "Usage: $0 [baseline|sof|ecc|both|all] [output_dir] [frame_count]"
        echo ""
        echo "Modes:"
        echo "  baseline - Test with current settings"
        echo "  sof      - Test with software_sof_enable=1"
        echo "  ecc      - Test with ecc_disable=1"
        echo "  both     - Test with both workarounds"
        echo "  all      - Run all tests sequentially"
        exit 1
        ;;
esac

echo ""
echo "=== Test complete ==="
echo "Output files in: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR"/*.raw 2>/dev/null || echo "No capture files"
