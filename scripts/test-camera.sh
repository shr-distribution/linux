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

# Set CSIPHY format (UYVY8_1X16 to match sensor output)
echo 'Setting CSIPHY format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':0[fmt:UYVY8_1X16/1288x968]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':1[fmt:UYVY8_1X16/1288x968]\" 2>&1 || true

# Set CSID format (sink uses 1X16 to match CSIPHY, pad 4 uses 2X8 to match VFE)
echo 'Setting CSID format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':0[fmt:UYVY8_1X16/1288x968]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':4[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true

# Set VFE PIX format (UYVY8_2X8 is VFE internal format, use 1288x968 to match CSID)
echo 'Setting VFE format...'
if [ -n \"\$VFE_PIX\" ]; then
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE_PIX':0[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE_PIX':1[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true
else
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE':0[fmt:UYVY8_2X8/1288x968]\" 2>&1 || true
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

# HP TouchPad camera paths:
# /dev/video0-2 = RDI paths (raw data, requires manual link setup)
# /dev/video3 = PIX path (ISP processed, has IMMUTABLE links)
#
# PIX mode (/dev/video3) is preferred because:
# - Links are pre-configured (IMMUTABLE)
# - Uses VFE ISP for color processing
# - Output is 1280x968 UYVY (sensor 1288 cropped to 1280)

VIDEO_DEV='/dev/video3'  # PIX mode - ISP processed output
FRAMES=5
OUTPUT='/tmp/camera_test.raw'

# Allow override via argument
if [ -n \"\$1\" ]; then
    VIDEO_DEV=\"\$1\"
fi

echo \"Using video device: \$VIDEO_DEV\"

# For PIX mode (video3), use 1280x968 (cropped from sensor's 1288x968)
# For RDI mode (video0-2), use 1288x968 (raw sensor output)
case \$VIDEO_DEV in
    *video3*)
        WIDTH=1280
        HEIGHT=968
        echo 'PIX mode: Using 1280x968 (VFE ISP cropped)'
        ;;
    *)
        WIDTH=1288
        HEIGHT=968
        echo 'RDI mode: Using 1288x968 (raw sensor output)'
        ;;
esac

# Method 1: Try v4l2-ctl capture
if command -v v4l2-ctl >/dev/null 2>&1; then
    echo ''
    echo '=== Testing with v4l2-ctl ==='

    # Set format
    echo \"Setting video format to \${WIDTH}x\${HEIGHT} UYVY...\"
    v4l2-ctl -d \$VIDEO_DEV --set-fmt-video=width=\$WIDTH,height=\$HEIGHT,pixelformat=UYVY 2>&1

    echo 'Current format:'
    v4l2-ctl -d \$VIDEO_DEV --get-fmt-video 2>&1

    echo ''
    echo \"Capturing \$FRAMES frames to \$OUTPUT...\"
    timeout 30 v4l2-ctl -d \$VIDEO_DEV --stream-mmap --stream-count=\$FRAMES --stream-to=\$OUTPUT 2>&1

    if [ -f \$OUTPUT ]; then
        SIZE=\$(stat -c %s \$OUTPUT 2>/dev/null || echo 0)
        echo \"Captured file size: \$SIZE bytes\"
        if [ \"\$SIZE\" -gt 0 ]; then
            echo 'SUCCESS: Captured frame data!'
            EXPECTED=\$((\$WIDTH * \$HEIGHT * 2 * FRAMES))
            echo \"Expected size: \$EXPECTED bytes\"
        else
            echo 'WARNING: Captured file is empty'
        fi
    else
        echo 'ERROR: No output file created (capture timed out)'
    fi
fi

# Method 2: Try GStreamer (if available)
if command -v gst-launch-1.0 >/dev/null 2>&1; then
    echo ''
    echo '=== Testing with GStreamer ==='

    # Test with fakesink first (no actual output, just test pipeline)
    echo \"Testing pipeline with fakesink (\${WIDTH}x\${HEIGHT} UYVY)...\"
    timeout 15 gst-launch-1.0 -v v4l2src device=\$VIDEO_DEV num-buffers=5 ! \
        \"video/x-raw,format=UYVY,width=\$WIDTH,height=\$HEIGHT\" ! \
        fakesink 2>&1

    # If that works, try saving a frame
    if [ \$? -eq 0 ]; then
        echo ''
        echo 'Saving single frame as PPM...'
        timeout 15 gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=1 ! \
            \"video/x-raw,format=UYVY,width=\$WIDTH,height=\$HEIGHT\" ! \
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

# Quick capture test - uses PIX mode by default (proper IMMUTABLE links)
quick_capture_test() {
    log_step "Quick capture test (PIX mode)..."

    log_info "Testing PIX mode capture: 1280x968 UYVY via /dev/video3..."
    log_info "This path has IMMUTABLE links and uses VFE ISP"

    run_on_device "
        if command -v gst-launch-1.0 >/dev/null 2>&1; then
            # PIX mode uses /dev/video3 with 1280x968 (cropped from sensor's 1288x968)
            # This path has IMMUTABLE links: sensor -> csiphy1 -> csid1:4 -> vfe_pix -> video3
            echo '=== PIX Mode Test: 1280x968 UYVY via /dev/video3 ==='
            timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=5 ! \\
                'video/x-raw,format=UYVY,width=1280,height=968' ! \\
                fakesink 2>&1

            if [ \$? -eq 0 ]; then
                echo ''
                echo 'SUCCESS: PIX capture completed!'
                echo ''
                echo '=== Saving test frame to /tmp/camera_frame.raw ==='
                timeout 15 gst-launch-1.0 v4l2src device=/dev/video3 num-buffers=1 ! \\
                    'video/x-raw,format=UYVY,width=1280,height=968' ! \\
                    filesink location=/tmp/camera_frame.raw 2>&1
                ls -la /tmp/camera_frame.raw 2>/dev/null && echo 'Frame saved!'
            else
                echo ''
                echo 'PIX mode failed. Trying RDI mode fallback...'

                echo ''
                echo '=== RDI Mode Test: 1288x968 UYVY via /dev/video0 ==='
                # First enable the RDI path links
                media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>/dev/null
                media-ctl -l '\"msm_csid1\":1->\"msm_vfe0_rdi0\":0[1]' 2>/dev/null
                timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=5 ! \\
                    'video/x-raw,format=UYVY,width=1288,height=968' ! \\
                    fakesink 2>&1 || echo 'RDI mode also failed'
            fi
        else
            echo 'GStreamer not available, using v4l2-ctl'
            timeout 30 v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=968,pixelformat=UYVY \\
                --stream-mmap --stream-count=3 --stream-to=/tmp/camera_test.raw 2>&1
            ls -la /tmp/camera_test.raw 2>/dev/null
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

        # Reset media links first to avoid conflicts from previous tests
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Setup full RDI path: CSIPHY1 -> CSID1 -> VFE RDI0
        # The sensor->CSIPHY link is IMMUTABLE and always enabled
        echo 'Setting up RDI0 media pipeline...'
        # Enable CSIPHY1 -> CSID1 link
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1
        # Enable CSID1 -> VFE RDI0 link (pad 1 = RDI0 source)
        media-ctl -l '\"msm_csid1\":1->\"msm_vfe0_rdi0\":0[1]' 2>&1
        # Set formats along the path
        # CRITICAL: Must configure sensor FIRST, then propagate downstream
        echo 'Configuring sensor for 1288x968...'
        media-ctl -V '\"mt9m114 pixel array 4-003c\":0[fmt:SGRBG10_1X10/1296x976 crop:(0,0)/1296x976]' 2>/dev/null
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[fmt:SGRBG10_1X10/1296x976 crop:(4,4)/1288x968]' 2>/dev/null
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        echo 'Configuring downstream pipeline...'
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_rdi0\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        echo 'RDI0 pipeline configured'

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

        # Reset media links first to avoid conflicts from previous tests
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Setup full PIX path: CSIPHY1 -> CSID1 -> VFE PIX
        # The sensor->CSIPHY link is IMMUTABLE and always enabled
        echo 'Setting up PIX media pipeline...'
        # Enable CSIPHY1 -> CSID1 link
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1
        # Enable CSID1 -> VFE PIX link (pad 4 = PIX source)
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1
        # Set formats along the path (PIX uses 1280x968 without blanking pixels)
        # CRITICAL: Must configure sensor FIRST, then propagate downstream
        # MT9M113 Context B (full res) = 1296x976 -> crop to 1288x968
        echo 'Configuring sensor for 1288x968...'
        media-ctl -V '\"mt9m114 pixel array 4-003c\":0[fmt:SGRBG10_1X10/1296x976 crop:(0,0)/1296x976]' 2>/dev/null
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[fmt:SGRBG10_1X10/1296x976 crop:(4,4)/1288x968]' 2>/dev/null
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        echo 'Configuring downstream pipeline...'
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        echo 'PIX pipeline configured'

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

# Test with sensor test pattern enabled
# This helps diagnose if data path works by using known pattern
test_sensor_pattern() {
    log_step "Testing with sensor test pattern..."
    log_info "This enables MT9M114's built-in color bar generator"
    log_info "If capture works with test pattern -> data path is OK"
    log_info "If capture fails even with test pattern -> VFE path broken"

    run_on_device "
        echo '=== Sensor Test Pattern Mode ==='
        echo ''

        # Find the MT9M114 IFP subdev (where test pattern control lives)
        echo 'Looking for MT9M114 IFP subdev...'
        IFP_DEV=''
        for dev in /dev/v4l-subdev*; do
            if v4l2-ctl -d \$dev --list-ctrls 2>/dev/null | grep -q 'test_pattern'; then
                IFP_DEV=\$dev
                echo \"Found test pattern control on: \$dev\"
                break
            fi
        done

        if [ -z \"\$IFP_DEV\" ]; then
            echo 'ERROR: No subdev with test_pattern control found'
            echo ''
            echo 'Available subdevs and their controls:'
            for dev in /dev/v4l-subdev*; do
                echo \"--- \$dev ---\"
                v4l2-ctl -d \$dev --list-ctrls 2>/dev/null | head -10
            done
            exit 1
        fi

        # Show available test patterns
        echo ''
        echo 'Available test patterns:'
        v4l2-ctl -d \$IFP_DEV --list-ctrls 2>/dev/null | grep -A10 test_pattern

        # Enable color bars test pattern (pattern 2 = 100% Color Bars)
        echo ''
        echo 'Enabling 100% Color Bars test pattern (pattern=2)...'
        v4l2-ctl -d \$IFP_DEV --set-ctrl=test_pattern=2 2>&1

        # Verify it's set
        echo ''
        echo 'Verifying test pattern is enabled:'
        v4l2-ctl -d \$IFP_DEV --get-ctrl=test_pattern 2>&1

        # Setup media pipeline for PIX mode
        echo ''
        echo 'Setting up media pipeline...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x968]' 2>/dev/null

        # Try to capture with test pattern
        echo ''
        echo 'Attempting capture with test pattern enabled...'
        timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=5 ! \\
            'video/x-raw,format=UYVY,width=1280,height=968,framerate=30/1' ! \\
            fakesink 2>&1
        RESULT=\$?

        # If capture worked, save a frame
        if [ \$RESULT -eq 0 ]; then
            echo ''
            echo 'SUCCESS: Capture with test pattern worked!'
            echo 'Saving test frame to /tmp/testpattern.raw...'
            gst-launch-1.0 v4l2src device=/dev/video3 num-buffers=1 ! \\
                'video/x-raw,format=UYVY,width=1280,height=968' ! \\
                filesink location=/tmp/testpattern.raw 2>&1
            ls -la /tmp/testpattern.raw 2>/dev/null

            # Check if file contains valid data (should be 2.4MB for 1280x968 UYVY)
            SIZE=\$(stat -c %s /tmp/testpattern.raw 2>/dev/null || echo 0)
            EXPECTED=\$((1280 * 968 * 2))
            echo \"File size: \$SIZE bytes (expected: \$EXPECTED)\"

            # Quick pattern analysis
            if [ \$SIZE -gt 0 ]; then
                echo ''
                echo 'First 64 bytes (hex):'
                xxd /tmp/testpattern.raw | head -4
                echo ''
                echo 'Unique byte patterns in first 1KB:'
                head -c 1024 /tmp/testpattern.raw | xxd -p | fold -w2 | sort | uniq -c | sort -rn | head -10
            fi
        else
            echo ''
            echo 'FAILED: Capture did not work even with test pattern'
            echo 'This indicates a VFE/CAMIF data path issue'
        fi

        # Disable test pattern
        echo ''
        echo 'Disabling test pattern...'
        v4l2-ctl -d \$IFP_DEV --set-ctrl=test_pattern=0 2>&1
    "
}

# Analyze captured frame data
analyze_frame() {
    log_step "Analyzing captured frame..."

    run_on_device "
        echo '=== Frame Analysis ==='
        echo ''

        # Check for captured files
        for file in /tmp/testpattern.raw /tmp/camera_frame.raw /tmp/camera_test.raw; do
            if [ -f \"\$file\" ]; then
                echo \"Found: \$file\"
                SIZE=\$(stat -c %s \$file)
                echo \"  Size: \$SIZE bytes\"

                # Check if file is all zeros
                ZEROS=\$(head -c 1024 \$file | xxd -p | tr -d '0' | wc -c)
                if [ \$ZEROS -eq 0 ]; then
                    echo '  WARNING: First 1KB is all zeros - no valid data captured!'
                else
                    echo '  Data appears valid (non-zero)'

                    # Show byte distribution
                    echo ''
                    echo '  Byte value distribution (first 4KB):'
                    head -c 4096 \$file | xxd -p | fold -w2 | sort | uniq -c | sort -rn | head -5

                    # For UYVY, we expect Y (luma) values spread across range
                    # and U/V (chroma) centered around 128 for gray/neutral
                    echo ''
                    echo '  First 32 bytes (UYVY interleaved):'
                    echo '  Format: U0 Y0 V0 Y1 U2 Y2 V2 Y3 ...'
                    head -c 32 \$file | xxd
                fi
                echo ''
            fi
        done

        if [ ! -f /tmp/testpattern.raw ] && [ ! -f /tmp/camera_frame.raw ]; then
            echo 'No captured frame files found'
            echo 'Run \"testpattern\" or \"pix\" mode first to capture data'
        fi
    "
}

# Test PREVIEW mode (640x480) - matches MT9M113 Context A output
# The sensor defaults to Context A (preview) mode which outputs 640x480
# This is the correct resolution to match what the sensor actually streams
test_preview_mode() {
    log_step "Testing PREVIEW mode (640x480)..."
    log_info "Path: Sensor -> CSIPHY -> CSID -> VFE PIX -> /dev/video3"
    log_info "Resolution: 640x480 (MT9M113 Context A / Preview mode)"
    log_info "Link frequency: 96 MHz (correct for this resolution)"

    run_on_device "
        echo '=== PREVIEW Mode Test (640x480 via VFE PIX) ==='
        echo ''
        echo 'NOTE: MT9M113 streams Context A (640x480) by default.'
        echo 'SEQ_CAP_MODE=0x0030 selects preview mode.'
        echo 'Link frequency 96 MHz is correct for 640x480 output.'
        echo ''

        # Reset media links first to avoid conflicts from previous tests
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Setup full PIX path: CSIPHY1 -> CSID1 -> VFE PIX
        echo 'Setting up PIX media pipeline for 640x480...'
        # Enable CSIPHY1 -> CSID1 link
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1
        # Enable CSID1 -> VFE PIX link (pad 4 = PIX source)
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1

        # Set formats along the path for 640x480
        # CRITICAL: Must configure sensor FIRST, then propagate downstream
        # MT9M113 Context A (preview) = 640x480 output
        echo 'Configuring sensor for 640x480...'
        media-ctl -V '\"mt9m114 pixel array 4-003c\":0[fmt:SGRBG10_1X10/648x488 crop:(0,0)/648x488]' 2>/dev/null
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[fmt:SGRBG10_1X10/648x488 crop:(4,4)/640x480]' 2>/dev/null
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/640x480]' 2>/dev/null
        echo 'Configuring downstream pipeline...'
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/640x480]' 2>/dev/null
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/640x480]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/640x480]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/640x480]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/640x480]' 2>/dev/null
        echo 'PIX pipeline configured for 640x480'

        echo ''
        echo 'Testing capture with 640x480 UYVY...'
        timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=640,height=480,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: PREVIEW capture completed!'
            echo ''
            echo 'Saving frame to /tmp/preview_frame.raw...'
            timeout 10 gst-launch-1.0 v4l2src device=/dev/video3 num-buffers=1 ! \\
                'video/x-raw,format=UYVY,width=640,height=480' ! \\
                filesink location=/tmp/preview_frame.raw 2>&1
            ls -la /tmp/preview_frame.raw 2>/dev/null
            SIZE=\$(stat -c %s /tmp/preview_frame.raw 2>/dev/null || echo 0)
            EXPECTED=\$((640 * 480 * 2))
            echo \"File size: \$SIZE bytes (expected: \$EXPECTED)\"
        else
            echo ''
            echo 'FAILED: PREVIEW capture did not complete'
            echo 'Check dmesg for errors'
        fi
    "
}

# Test with EFS sync mode (instead of APS)
# EFS = Embedded Frame Sync - uses embedded sync codes in data stream
# APS = Active Pixel Sync - uses external sync signals from CSIPHY (default)
test_efs_mode() {
    log_step "Testing with EFS sync mode enabled..."
    log_info "This changes CAMIF sync mode from APS (default) to EFS"
    log_info "Some MIPI sensors may require EFS mode for proper framing"

    run_on_device "
        echo '=== EFS Sync Mode Test ==='
        echo ''

        # Check current setting
        echo 'Current vfe31_use_efs_sync setting:'
        cat /sys/module/qcom_camss/parameters/vfe31_use_efs_sync 2>/dev/null || echo 'Parameter not found (module not loaded?)'

        # Enable EFS mode
        echo ''
        echo 'Enabling EFS sync mode (vfe31_use_efs_sync=1)...'
        echo 1 > /sys/module/qcom_camss/parameters/vfe31_use_efs_sync 2>/dev/null
        if [ \$? -eq 0 ]; then
            echo 'EFS mode enabled'
        else
            echo 'ERROR: Failed to set EFS mode - is qcom_camss module loaded?'
            exit 1
        fi

        # Verify setting
        echo 'Verifying: vfe31_use_efs_sync='
        cat /sys/module/qcom_camss/parameters/vfe31_use_efs_sync

        # Clear dmesg to see fresh output
        dmesg -C

        # Setup media pipeline (same as pix mode)
        echo ''
        echo 'Setting up PIX media pipeline...'
        media-ctl -r 2>/dev/null || true
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        echo 'Pipeline configured'

        # Attempt capture
        echo ''
        echo 'Testing capture with EFS sync mode...'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=1280,height=968,framerate=30/1' ! \\
            fakesink 2>&1
        RESULT=\$?

        # Show relevant dmesg
        echo ''
        echo '=== Relevant dmesg output ==='
        dmesg | grep -iE 'vfe.*efs|vfe.*aps|camif_cfg|sync.*mode' | tail -20

        # Disable EFS mode (back to APS default)
        echo ''
        echo 'Disabling EFS mode (back to APS default)...'
        echo 0 > /sys/module/qcom_camss/parameters/vfe31_use_efs_sync

        if [ \$RESULT -eq 0 ]; then
            echo ''
            echo 'SUCCESS: Capture with EFS mode completed!'
            echo 'EFS mode may be the correct setting for this sensor.'
        else
            echo ''
            echo 'FAILED: Capture with EFS mode did not work'
            echo 'Try APS mode (default) or check other VFE configuration.'
        fi
    "
}

# Full debug capture mode with clock and register dumps
test_debug_capture() {
    log_step "DEBUG MODE: Full diagnostic capture with clock/register dumps..."
    log_info "This mode captures comprehensive debug info for VFE troubleshooting"

    run_on_device "
        echo '=============================================='
        echo '  CAMSS DEBUG CAPTURE MODE'
        echo '=============================================='
        echo ''

        # Clear dmesg to start fresh
        echo 'Clearing dmesg...'
        dmesg -C

        # Step 1: Show initial state
        echo ''
        echo '=== Step 1: Initial System State ==='
        echo 'Clock debugfs (if available):'
        if [ -d /sys/kernel/debug/clk ]; then
            echo '  VFE clocks:'
            for clk in vfe vfe_axi vfe_ahb vfe_csi0 vfe_csi1 csi_rdi csi_pix csi1 csi1_phy; do
                if [ -f /sys/kernel/debug/clk/\$clk/clk_enable_count ]; then
                    echo \"    \$clk: enable_count=\$(cat /sys/kernel/debug/clk/\$clk/clk_enable_count)\"
                fi
            done
        else
            echo '  debugfs/clk not available - check dmesg for clock enables'
        fi

        # Step 2: Power on sensor
        echo ''
        echo '=== Step 2: Powering on Sensor ==='
        # Touch the subdev to trigger runtime PM
        v4l2-ctl -d /dev/v4l-subdev3 --get-ctrl=test_pattern 2>/dev/null || true
        sleep 1

        # Step 3: Show clock state after sensor power on
        echo ''
        echo '=== Step 3: Clock State After Sensor Power On ==='
        if [ -d /sys/kernel/debug/clk ]; then
            echo '  VFE clocks:'
            for clk in vfe vfe_axi vfe_ahb vfe_csi0 vfe_csi1 csi_rdi csi_pix csi1 csi1_phy; do
                if [ -f /sys/kernel/debug/clk/\$clk/clk_enable_count ]; then
                    echo \"    \$clk: enable_count=\$(cat /sys/kernel/debug/clk/\$clk/clk_enable_count)\"
                fi
            done
        fi

        # Step 4: Setup media pipeline
        echo ''
        echo '=== Step 4: Setting Up Media Pipeline ==='
        # Reset media links first
        media-ctl -r 2>/dev/null || true
        # Enable full PIX path: CSIPHY1 -> CSID1 -> VFE PIX
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1
        # Set formats along the path
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1288x968]' 2>/dev/null
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x968]' 2>/dev/null
        echo 'PIX pipeline configured: CSIPHY1 -> CSID1 -> VFE_PIX'

        # Step 5: Show clock state after pipeline setup
        echo ''
        echo '=== Step 5: Clock State After Pipeline Setup ==='
        if [ -d /sys/kernel/debug/clk ]; then
            for clk in vfe vfe_axi vfe_ahb vfe_csi0 vfe_csi1 csi_rdi csi_pix csi1 csi1_phy; do
                if [ -f /sys/kernel/debug/clk/\$clk/clk_enable_count ]; then
                    echo \"    \$clk: enable_count=\$(cat /sys/kernel/debug/clk/\$clk/clk_enable_count)\"
                fi
            done
        fi

        # Step 6: Attempt capture (will likely fail but generate debug output)
        echo ''
        echo '=== Step 6: Attempting Capture ==='
        echo 'Starting gst-launch (expect VFE SOF timeout)...'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=5 ! \\
            'video/x-raw,format=UYVY,width=1280,height=968' ! \\
            fakesink 2>&1 || echo 'Capture failed as expected'

        # Step 7: Dump full dmesg for analysis
        echo ''
        echo '=== Step 7: Full DMESG Output ==='
        echo '(Focus on clock enables, VFE registers, CSIPHY status)'
        echo ''
        dmesg | grep -iE 'camss|csiphy|csid|vfe|clock|enable' | tail -100

        # Step 8: Show CSIPHY interrupt status
        echo ''
        echo '=== Step 8: CSIPHY Interrupt Summary ==='
        dmesg | grep -i 'csiphy.*sof_count\\|csiphy.*irq\\|csiphy.*status' | tail -20

        # Step 9: Show VFE register state
        echo ''
        echo '=== Step 9: VFE Register Dumps ==='
        dmesg | grep -iE 'vfe31:|core_cfg|camif_cfg|axi_out|irq_status' | tail -30

        # Step 10: Final summary
        echo ''
        echo '=== Step 10: Debug Summary ==='
        echo 'Key things to check:'
        echo '  1. Are vfe_csi0 and vfe_csi1 clocks enabled? (look for enable_clocks output)'
        echo '  2. Does CSIPHY1 show sof_count > 0? (CSIPHY receiving frames)'
        echo '  3. Does VFE IRQ_STATUS0 show any interrupts? (VFE receiving data)'
        echo '  4. What are CAMIF_CFG and CORE_CFG values?'
        echo ''
        echo 'If CSIPHY gets frames but VFE does not:'
        echo '  -> Check vfe_csi1 clock enable (CSI1 to VFE data path)'
        echo '  -> Check VFE CAMIF configuration'
        echo ''
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
            testpattern)
                MODE="testpattern"
                ;;
            analyze)
                MODE="analyze"
                ;;
            debug)
                MODE="debug"
                ;;
            efs)
                MODE="efs"
                ;;
            preview|vga|640)
                MODE="preview"
                ;;
            --help|-h)
                echo "Usage: $0 [MODE]"
                echo ""
                echo "Modes:"
                echo "  preview     Test PREVIEW mode (640x480) - matches MT9M113 Context A"
                echo "  raw         Test RAW passthrough (CAMIF->memory via RDI, no ISP)"
                echo "  pix         Test PIX mode (1280x968, through VFE ISP processing)"
                echo "  efs         Test PIX mode with EFS sync (instead of default APS)"
                echo "  testpattern Enable sensor test pattern and capture (debug data path)"
                echo "  analyze     Analyze previously captured frame data"
                echo "  debug       Full debug capture with clock and register dumps"
                echo "  --info      Show camera device information only"
                echo "  --setup     Set up media pipeline only"
                echo "  --capture   Test capture only (assumes pipeline is set up)"
                echo "  --quick     Quick capture test without media-ctl setup"
                echo "  (no args)   Run full test sequence"
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
        preview)
            show_camera_info
            ensure_camera_ready
            test_preview_mode
            check_dmesg
            ;;
        testpattern)
            show_camera_info
            ensure_camera_ready
            test_sensor_pattern
            check_dmesg
            ;;
        efs)
            show_camera_info
            ensure_camera_ready
            test_efs_mode
            check_dmesg
            ;;
        analyze)
            analyze_frame
            ;;
        debug)
            test_debug_capture
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
