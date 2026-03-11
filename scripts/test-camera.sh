#!/bin/bash
#
# Camera Test Script for HP TouchPad
# Tests the CAMSS (Camera Subsystem) with MT9M114 front camera sensor
#
# Usage: ./scripts/test-camera.sh [--info] [--setup] [--capture]
#
# Requirements:
# - Device accessible at 172.16.42.2 via USB network
# - Camera drivers loaded (camss, mt9m114)
#

set -e

DEVICE_IP="172.16.42.2"
SSH_PORT="22"
TIMEOUT=10

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

log_step() {
    echo -e "${GREEN}[STEP]${NC} $1"
}

log_info() {
    echo -e "${YELLOW}[INFO]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_result() {
    echo -e "${GREEN}[RESULT]${NC} $1"
}

log_debug() {
    echo -e "${CYAN}[DEBUG]${NC} $1"
}

# Run command on device via SSH
run_on_device() {
    local cmd="$1"
    ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=$TIMEOUT -p $SSH_PORT root@$DEVICE_IP "$cmd" 2>/dev/null
}

# Check device connectivity
check_device() {
    log_step "Checking device connectivity..."
    if timeout 3 bash -c "echo > /dev/tcp/$DEVICE_IP/$SSH_PORT" 2>/dev/null; then
        log_info "Device is reachable at $DEVICE_IP (SSH port $SSH_PORT)"
        return 0
    else
        log_error "Cannot connect to device at $DEVICE_IP port $SSH_PORT"
        return 1
    fi
}

# Check and ensure mt9m114 sensor is powered and bound
ensure_camera_ready() {
    log_step "Checking camera sensor status..."

    run_on_device "cat <<'SCRIPT' > /tmp/ensure_camera.sh
#!/bin/sh

echo '=== Checking mt9m114 sensor ==='

# Check if sensor is bound
SENSOR_PATH='/sys/bus/i2c/devices/4-003c'
DRIVER_PATH='/sys/bus/i2c/drivers/mt9m114'

if [ ! -d \"\$SENSOR_PATH\" ]; then
    echo 'ERROR: Sensor device not found at 4-003c'
    exit 1
fi

# Check if driver is bound
if [ -L \"\$SENSOR_PATH/driver\" ]; then
    BOUND_DRIVER=\$(basename \$(readlink \$SENSOR_PATH/driver))
    echo \"Sensor bound to driver: \$BOUND_DRIVER\"
else
    echo 'WARNING: Sensor not bound to any driver'
    echo 'Attempting to bind mt9m114 driver...'
    echo '4-003c' > \$DRIVER_PATH/bind 2>/dev/null
    sleep 2
    if [ -L \"\$SENSOR_PATH/driver\" ]; then
        echo 'SUCCESS: Sensor bound to mt9m114 driver'
    else
        echo 'ERROR: Failed to bind sensor'
        exit 1
    fi
fi

# Check runtime PM status
if [ -f \"\$SENSOR_PATH/power/runtime_status\" ]; then
    STATUS=\$(cat \$SENSOR_PATH/power/runtime_status)
    echo \"Runtime PM status: \$STATUS\"
    if [ \"\$STATUS\" = \"suspended\" ]; then
        echo 'Sensor is suspended, will wake on use'
    fi
fi

# Verify sensor appears in media topology
if command -v media-ctl >/dev/null 2>&1; then
    if media-ctl -p 2>/dev/null | grep -q 'mt9m114'; then
        echo 'Sensor found in media topology'
    else
        echo 'WARNING: Sensor not in media topology - may need reboot'
    fi
fi

echo ''
echo '=== Sensor check complete ==='
SCRIPT
chmod +x /tmp/ensure_camera.sh
/tmp/ensure_camera.sh"
}

# Show camera module info
show_camera_info() {
    log_step "Camera driver information..."

    log_info "Loaded camera modules:"
    run_on_device "lsmod | grep -E 'camss|mt9m114|v4l2' || echo 'No camera modules loaded'"

    log_info "Video devices:"
    run_on_device "ls -la /dev/video* 2>/dev/null || echo 'No video devices found'"

    log_info "Media devices:"
    run_on_device "ls -la /dev/media* 2>/dev/null || echo 'No media devices found'"

    log_info "V4L2 subdevices:"
    run_on_device "ls -la /dev/v4l-subdev* 2>/dev/null || echo 'No v4l2 subdevices found'"
}

# Show media topology
show_media_topology() {
    log_step "Media topology..."

    run_on_device "cat <<'SCRIPT' > /tmp/show_media.sh
#!/bin/sh
# Check if media-ctl is available
if command -v media-ctl >/dev/null 2>&1; then
    echo '=== Media Controller Topology ==='
    media-ctl -p 2>/dev/null || echo 'media-ctl -p failed'
    echo ''
    echo '=== Entity Names ==='
    media-ctl -p 2>/dev/null | grep 'entity' | head -20
else
    echo 'media-ctl not available, checking sysfs...'
    if [ -d /sys/bus/media/devices ]; then
        for dev in /sys/bus/media/devices/media*; do
            echo \"Device: \$(basename \$dev)\"
            cat \$dev/model 2>/dev/null || echo 'No model info'
        done
    fi
fi
SCRIPT
chmod +x /tmp/show_media.sh
/tmp/show_media.sh"
}

# Show V4L2 device capabilities
show_v4l2_info() {
    log_step "V4L2 device capabilities..."

    run_on_device "cat <<'SCRIPT' > /tmp/v4l2_info.sh
#!/bin/sh
for dev in /dev/video*; do
    if [ -e \"\$dev\" ]; then
        echo \"\"
        echo \"=== \$dev ===\"

        # Check if v4l2-ctl is available
        if command -v v4l2-ctl >/dev/null 2>&1; then
            echo '--- Device Info ---'
            v4l2-ctl -d \$dev --info 2>/dev/null | head -20

            echo ''
            echo '--- Supported Formats ---'
            v4l2-ctl -d \$dev --list-formats-ext 2>/dev/null | head -40

            echo ''
            echo '--- Current Format ---'
            v4l2-ctl -d \$dev --get-fmt-video 2>/dev/null
        else
            # Fallback to reading sysfs
            echo 'v4l2-ctl not available'
            devnum=\$(basename \$dev | sed 's/video//')
            if [ -d /sys/class/video4linux/video\$devnum ]; then
                echo \"Name: \$(cat /sys/class/video4linux/video\$devnum/name 2>/dev/null)\"
            fi
        fi
    fi
done
SCRIPT
chmod +x /tmp/v4l2_info.sh
/tmp/v4l2_info.sh"
}

# Setup media pipeline for capture
setup_media_pipeline() {
    log_step "Setting up media pipeline..."

    # MT9M114 outputs 1280x960 UYVY (or 1288x968 with blanking)
    # Sensor -> CSIPHY1 -> CSID1 -> VFE -> video node

    run_on_device "cat <<'SCRIPT' > /tmp/setup_pipeline.sh
#!/bin/sh

# Check if media-ctl is available
if ! command -v media-ctl >/dev/null 2>&1; then
    echo 'ERROR: media-ctl is required for pipeline setup'
    exit 1
fi

echo '=== Setting up camera pipeline ==='

# Find the media device
MEDIA_DEV=\$(ls /dev/media* 2>/dev/null | head -1)
if [ -z \"\$MEDIA_DEV\" ]; then
    echo 'ERROR: No media device found'
    exit 1
fi
echo \"Using media device: \$MEDIA_DEV\"

# Reset all links first
echo 'Resetting links...'
media-ctl -d \$MEDIA_DEV -r 2>/dev/null || true

# Get entity names from topology
echo ''
echo 'Discovering entities...'
SENSOR=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -o 'mt9m114[^\"]*' | head -1)
echo \"Sensor: \$SENSOR\"

# Common entity patterns for MSM8660 CAMSS
# Format: sensor -> csiphy -> csid -> ispif -> vfe -> video

# Try to find CSIPHY1 (front camera uses CSI1)
CSIPHY=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_csiphy[0-9]' | grep '1' | head -1)
if [ -z \"\$CSIPHY\" ]; then
    CSIPHY=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_csiphy[0-9]' | head -1)
fi
echo \"CSIPHY: \$CSIPHY\"

# Try to find CSID1 (matching CSIPHY1 for front camera)
CSID=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_csid[0-9]' | grep '1' | head -1)
if [ -z \"\$CSID\" ]; then
    CSID=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_csid[0-9]' | head -1)
fi
echo \"CSID: \$CSID\"

# VFE has multiple pads - we need the RDI or PIX output
VFE=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_vfe[0-9]' | head -1)
echo \"VFE: \$VFE\"

# VFE PIX is a separate entity for pixel (CAMIF) path
VFE_PIX=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_vfe[0-9]_pix' | head -1)
echo \"VFE_PIX: \$VFE_PIX\"

if [ -z \"\$SENSOR\" ] || [ -z \"\$CSIPHY\" ]; then
    echo 'ERROR: Could not find required entities'
    echo 'Current topology:'
    media-ctl -d \$MEDIA_DEV -p 2>/dev/null
    exit 1
fi

# Set format on sensor (source pad 0)
# MT9M114 native resolution is 1288x968 UYVY
# For MSM8660 CAMIF PIX mode, use 2X8 format (8-bit data on 16-bit bus)
echo ''
echo 'Setting sensor format (1288x968 UYVY8_2X8)...'
media-ctl -d \$MEDIA_DEV -V \"'\$SENSOR':0[fmt:UYVY8_2X8/1288x968]\" 2>&1 || \
media-ctl -d \$MEDIA_DEV -V \"'\$SENSOR':0[fmt:UYVY8_1X16/1288x968]\" 2>&1 || \
echo 'Sensor format set may have failed'

# Set CSIPHY format (sink and source)
echo 'Setting CSIPHY format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':0[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':1[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true

# Set CSID format (sink=0, PIX source=4)
echo 'Setting CSID format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':0[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':4[fmt:UYVY8_2X8/1280x968]\" 2>&1 || true

# Set VFE PIX format (without blanking pixels)
echo 'Setting VFE format...'
if [ -n \"\$VFE_PIX\" ]; then
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE_PIX':0[fmt:UYVY8_2X8/1280x968]\" 2>&1 || true
else
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE':0[fmt:UYVY8_2X8/1280x968]\" 2>&1 || true
fi

# Enable links: sensor -> csiphy -> csid -> vfe
echo ''
echo 'Enabling links...'
media-ctl -d \$MEDIA_DEV -l \"'\$SENSOR':0->'\$CSIPHY':0[1]\" 2>&1 || echo 'sensor->csiphy link failed'
media-ctl -d \$MEDIA_DEV -l \"'\$CSIPHY':1->'\$CSID':0[1]\" 2>&1 || echo 'csiphy->csid link failed'

# For VFE, enable CSID->VFE PIX link
# MSM8660 has no ISPIF, CSID connects directly to VFE
# CSID pad 4 = PIX line (MSM_CSID_PAD_FIRST_SRC + VFE_LINE_PIX = 1 + 3 = 4)
# VFE_PIX sink pad = 0
if [ -n \"\$VFE_PIX\" ]; then
    media-ctl -d \$MEDIA_DEV -l \"'\$CSID':4->'\$VFE_PIX':0[1]\" 2>&1 || echo 'csid->vfe_pix link failed'
else
    # Fallback: try generic VFE entity with pad 4->0
    media-ctl -d \$MEDIA_DEV -l \"'\$CSID':4->'\$VFE':0[1]\" 2>&1 || echo 'csid->vfe link failed'
fi

echo ''
echo '=== Final Pipeline Configuration ==='
media-ctl -d \$MEDIA_DEV -p 2>/dev/null

echo ''
echo 'Pipeline setup complete'
SCRIPT
chmod +x /tmp/setup_pipeline.sh
/tmp/setup_pipeline.sh"
}

# Test video capture
test_capture() {
    log_step "Testing video capture..."

    run_on_device "cat <<'SCRIPT' > /tmp/test_capture.sh
#!/bin/sh

VIDEO_DEV='/dev/video0'
FRAMES=5
OUTPUT='/tmp/camera_test.raw'

# Find first available video device
for dev in /dev/video*; do
    if [ -e \"\$dev\" ]; then
        VIDEO_DEV=\$dev
        break
    fi
done

echo \"Using video device: \$VIDEO_DEV\"

# Method 1: Try v4l2-ctl capture
if command -v v4l2-ctl >/dev/null 2>&1; then
    echo ''
    echo '=== Testing with v4l2-ctl ==='

    # Set format to match mt9m114 sensor output (1288x968 UYVY)
    echo 'Setting video format to 1288x968 UYVY (mt9m114 native)...'
    v4l2-ctl -d \$VIDEO_DEV --set-fmt-video=width=1288,height=968,pixelformat=UYVY 2>&1

    echo 'Current format:'
    v4l2-ctl -d \$VIDEO_DEV --get-fmt-video 2>&1

    echo ''
    echo \"Capturing \$FRAMES frames to \$OUTPUT...\"
    v4l2-ctl -d \$VIDEO_DEV --stream-mmap --stream-count=\$FRAMES --stream-to=\$OUTPUT 2>&1

    if [ -f \$OUTPUT ]; then
        SIZE=\$(stat -c %s \$OUTPUT 2>/dev/null || echo 0)
        echo \"Captured file size: \$SIZE bytes\"
        if [ \"\$SIZE\" -gt 0 ]; then
            echo 'SUCCESS: Captured frame data!'
            # Expected size: 1288 * 968 * 2 bytes/pixel * 5 frames = 12468160 bytes
            EXPECTED=\$((1288 * 968 * 2 * FRAMES))
            echo \"Expected size: \$EXPECTED bytes\"
        else
            echo 'WARNING: Captured file is empty'
        fi
    else
        echo 'ERROR: No output file created'
    fi
fi

# Method 2: Try GStreamer (if available)
if command -v gst-launch-1.0 >/dev/null 2>&1; then
    echo ''
    echo '=== Testing with GStreamer ==='

    # Test with fakesink first (no actual output, just test pipeline)
    # IMPORTANT: Use 1288x968 UYVY - the exact mt9m114 sensor resolution
    echo 'Testing pipeline with fakesink (1288x968 UYVY)...'
    timeout 10 gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=5 ! \
        'video/x-raw,format=UYVY,width=1288,height=968' ! \
        fakesink 2>&1

    # If that works, try saving a frame
    if [ \$? -eq 0 ]; then
        echo ''
        echo 'Saving single frame as PPM...'
        timeout 10 gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=1 ! \
            'video/x-raw,format=UYVY,width=1288,height=968' ! \
            videoconvert ! \
            pnmenc ! \
            filesink location=/tmp/frame.ppm 2>&1
        ls -la /tmp/frame.ppm 2>/dev/null && echo 'Frame saved!'
    fi
fi

# Method 3: Raw read test (last resort)
if [ ! -f \$OUTPUT ] || [ \"\$(stat -c %s \$OUTPUT 2>/dev/null)\" = '0' ]; then
    echo ''
    echo '=== Testing raw device read ==='
    # Try to read directly from device
    timeout 5 dd if=\$VIDEO_DEV of=/tmp/raw_test.bin bs=1024 count=100 2>&1 || \
        echo 'Raw read failed (expected if no capture active)'
fi
SCRIPT
chmod +x /tmp/test_capture.sh
/tmp/test_capture.sh"
}

# Check kernel messages for camera errors
check_dmesg() {
    log_step "Checking kernel messages..."

    log_info "Recent camera-related messages:"
    run_on_device "dmesg | grep -iE 'camss|csiphy|csid|vfe|mt9m114|video|format' | tail -30"

    log_info "Any errors:"
    run_on_device "dmesg | grep -iE 'error|fail|timeout' | grep -iE 'camss|csiphy|csid|vfe|video' | tail -10"
}

# Quick capture test (no media-ctl setup)
quick_capture_test() {
    log_step "Quick capture test..."

    log_info "Testing capture with correct mt9m114 format (1288x968 UYVY)..."
    run_on_device "
        if command -v gst-launch-1.0 >/dev/null 2>&1; then
            # MT9M114 native resolution is 1288x968 UYVY
            # IMPORTANT: Must specify exact format to avoid wrong negotiation
            echo '=== Test: 1288x968 UYVY (native mt9m114 resolution) ==='
            timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                'video/x-raw,format=UYVY,width=1288,height=968' ! \\
                fakesink 2>&1

            if [ \$? -eq 0 ]; then
                echo ''
                echo 'SUCCESS: Capture completed!'
                echo ''
                echo '=== Saving test frame to /tmp/camera_frame.raw ==='
                timeout 10 gst-launch-1.0 v4l2src device=/dev/video0 num-buffers=1 ! \\
                    'video/x-raw,format=UYVY,width=1288,height=968' ! \\
                    filesink location=/tmp/camera_frame.raw 2>&1
                ls -la /tmp/camera_frame.raw 2>/dev/null && echo 'Frame saved!'
            else
                echo ''
                echo 'Native resolution failed, trying alternatives...'

                echo ''
                echo '=== Test: 1280x960 UYVY ==='
                timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                    'video/x-raw,format=UYVY,width=1280,height=960' ! \\
                    fakesink 2>&1 || echo 'Test failed'

                echo ''
                echo '=== Test: 640x480 UYVY (VGA) ==='
                timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                    'video/x-raw,format=UYVY,width=640,height=480' ! \\
                    fakesink 2>&1 || echo 'Test failed'
            fi
        else
            echo 'GStreamer not available, using dd test'
            # Raw device read test
            timeout 5 dd if=/dev/video0 of=/tmp/raw_test.bin bs=1024 count=100 2>&1 || \\
                echo 'Raw read failed (expected if streaming not started)'
        fi
    "
}

# Test RAW passthrough mode (CAMIF -> memory, no ISP)
# Uses /dev/video0 (msm_vfe0_rdi0)
test_raw_mode() {
    log_step "Testing RAW passthrough mode..."
    log_info "Path: Sensor -> CSIPHY -> CSID -> VFE RDI0 -> /dev/video0"
    log_info "Data goes directly to memory, bypassing ISP"

    run_on_device "
        echo '=== RAW Mode Test (video0 via RDI0) ==='
        echo ''
        echo 'Testing capture with 1288x968 UYVY...'
        timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=1288,height=968,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: RAW capture completed!'
        else
            echo ''
            echo 'FAILED: RAW capture did not complete'
            echo 'Check dmesg for errors'
        fi
    "
}

# Test PIX/CAMIF mode (through ISP processing)
# Uses /dev/video3 (msm_vfe0_pix)
test_pix_mode() {
    log_step "Testing PIX/CAMIF mode (ISP processing)..."
    log_info "Path: Sensor -> CSIPHY -> CSID -> VFE PIX -> /dev/video3"
    log_info "Data goes through VFE ISP for processing"

    run_on_device "
        echo '=== PIX Mode Test (video3 via VFE PIX) ==='
        echo ''

        # Setup media pipeline: CSID1 -> VFE PIX link
        # VFE PIX mode expects UYVY8_2X8 format at 1280x968 (without blanking)
        # The VFE extracts 8-bit data from the 16-bit MIPI bus
        echo 'Setting up media pipeline...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        echo 'Pipeline configured'

        echo ''
        echo 'Testing capture with 1280x968 UYVY...'
        timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=1280,height=968,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: PIX capture completed!'
        else
            echo ''
            echo 'FAILED: PIX capture did not complete'
            echo 'Check dmesg for errors'
        fi
    "
}

# Main
main() {
    echo "=============================================="
    echo "  HP TouchPad Camera Test Script"
    echo "=============================================="
    echo ""

    MODE="full"

    # Parse arguments
    for arg in "$@"; do
        case $arg in
            --info)
                MODE="info"
                ;;
            --setup)
                MODE="setup"
                ;;
            --capture)
                MODE="capture"
                ;;
            --quick)
                MODE="quick"
                ;;
            raw)
                MODE="raw"
                ;;
            pix)
                MODE="pix"
                ;;
            --help|-h)
                echo "Usage: $0 [MODE]"
                echo ""
                echo "Modes:"
                echo "  raw        Test RAW passthrough (CAMIF->memory via RDI, no ISP)"
                echo "  pix        Test PIX mode (through VFE ISP processing)"
                echo "  --info     Show camera device information only"
                echo "  --setup    Set up media pipeline only"
                echo "  --capture  Test capture only (assumes pipeline is set up)"
                echo "  --quick    Quick capture test without media-ctl setup"
                echo "  (no args)  Run full test sequence"
                exit 0
                ;;
        esac
    done

    check_device || exit 1

    case $MODE in
        info)
            show_camera_info
            show_v4l2_info
            show_media_topology
            ;;
        setup)
            setup_media_pipeline
            ;;
        capture)
            ensure_camera_ready
            test_capture
            check_dmesg
            ;;
        quick)
            show_camera_info
            ensure_camera_ready
            quick_capture_test
            check_dmesg
            ;;
        raw)
            show_camera_info
            ensure_camera_ready
            test_raw_mode
            check_dmesg
            ;;
        pix)
            show_camera_info
            ensure_camera_ready
            test_pix_mode
            check_dmesg
            ;;
        full)
            show_camera_info
            ensure_camera_ready
            show_v4l2_info
            show_media_topology
            setup_media_pipeline
            test_capture
            check_dmesg
            ;;
    esac

    echo ""
    echo "=============================================="
    echo "  Test Complete"
    echo "=============================================="
}

main "$@"
