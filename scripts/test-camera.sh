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

CSID=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_csid[0-9]' | head -1)
echo \"CSID: \$CSID\"

# VFE has multiple pads - we need the RDI or PIX output
VFE=\$(media-ctl -d \$MEDIA_DEV -p 2>/dev/null | grep -oE 'msm_vfe[0-9]' | head -1)
echo \"VFE: \$VFE\"

if [ -z \"\$SENSOR\" ] || [ -z \"\$CSIPHY\" ]; then
    echo 'ERROR: Could not find required entities'
    echo 'Current topology:'
    media-ctl -d \$MEDIA_DEV -p 2>/dev/null
    exit 1
fi

# Set format on sensor (source pad 0)
# MT9M114 native formats: 1280x960 UYVY or 1288x968 with blanking
echo ''
echo 'Setting sensor format...'
media-ctl -d \$MEDIA_DEV -V \"'\$SENSOR':0[fmt:UYVY8_2X8/1280x960]\" 2>&1 || \
media-ctl -d \$MEDIA_DEV -V \"'\$SENSOR':0[fmt:UYVY8_1X16/1280x960]\" 2>&1 || \
echo 'Sensor format set may have failed'

# Set CSIPHY format
echo 'Setting CSIPHY format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':0[fmt:UYVY8_2X8/1280x960]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':1[fmt:UYVY8_2X8/1280x960]\" 2>&1 || true

# Set CSID format
echo 'Setting CSID format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':0[fmt:UYVY8_2X8/1280x960]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':1[fmt:UYVY8_2X8/1280x960]\" 2>&1 || true

# Set VFE format
echo 'Setting VFE format...'
media-ctl -d \$MEDIA_DEV -V \"'\$VFE':0[fmt:UYVY8_2X8/1280x960]\" 2>&1 || true

# Enable links: sensor -> csiphy -> csid -> vfe
echo ''
echo 'Enabling links...'
media-ctl -d \$MEDIA_DEV -l \"'\$SENSOR':0->'\$CSIPHY':0[1]\" 2>&1 || echo 'sensor->csiphy link failed'
media-ctl -d \$MEDIA_DEV -l \"'\$CSIPHY':1->'\$CSID':0[1]\" 2>&1 || echo 'csiphy->csid link failed'

# For VFE, need to find the right pads
# RDI path: csid -> vfe_rdi
# PIX path: csid -> vfe_pix
media-ctl -d \$MEDIA_DEV -l \"'\$CSID':1->'\$VFE':0[1]\" 2>&1 || echo 'csid->vfe link failed'

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

    # Set format to match sensor output
    echo 'Setting video format to 1280x960 UYVY...'
    v4l2-ctl -d \$VIDEO_DEV --set-fmt-video=width=1280,height=960,pixelformat=UYVY 2>&1

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
            # Expected size: 1280 * 960 * 2 bytes/pixel * 5 frames = 12288000 bytes
            EXPECTED=\$((1280 * 960 * 2 * FRAMES))
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
    echo 'Testing pipeline with fakesink...'
    timeout 5 gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=5 ! \
        'video/x-raw,format=UYVY,width=1280,height=960' ! \
        fakesink 2>&1

    # If that works, try saving a frame
    if [ \$? -eq 0 ]; then
        echo ''
        echo 'Saving single frame as PPM...'
        gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=1 ! \
            'video/x-raw,format=UYVY,width=1280,height=960' ! \
            videoconvert ! \
            pnmenc ! \
            filesink location=/tmp/frame.ppm 2>&1
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

    log_info "Testing v4l2src with explicit format..."
    run_on_device "
        if command -v gst-launch-1.0 >/dev/null 2>&1; then
            # Try different format combinations
            echo 'Test 1: 1280x960 UYVY'
            timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                'video/x-raw,format=UYVY,width=1280,height=960' ! \\
                fakesink 2>&1 || echo 'Test 1 failed'

            echo ''
            echo 'Test 2: 1280x720 UYVY (720p)'
            timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                'video/x-raw,format=UYVY,width=1280,height=720' ! \\
                fakesink 2>&1 || echo 'Test 2 failed'

            echo ''
            echo 'Test 3: 640x480 UYVY (VGA)'
            timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                'video/x-raw,format=UYVY,width=640,height=480' ! \\
                fakesink 2>&1 || echo 'Test 3 failed'

            echo ''
            echo 'Test 4: Auto-negotiate (let v4l2src choose)'
            timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                videoconvert ! \\
                fakesink 2>&1 || echo 'Test 4 failed'
        else
            echo 'GStreamer not available'
            # Try v4l2-ctl if available
            if command -v v4l2-ctl >/dev/null 2>&1; then
                v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=960,pixelformat=UYVY
                v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=3 --stream-to=/tmp/test.raw
                ls -la /tmp/test.raw
            fi
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
            --help|-h)
                echo "Usage: $0 [--info] [--setup] [--capture] [--quick]"
                echo ""
                echo "Options:"
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
            test_capture
            check_dmesg
            ;;
        quick)
            show_camera_info
            quick_capture_test
            check_dmesg
            ;;
        full)
            show_camera_info
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
