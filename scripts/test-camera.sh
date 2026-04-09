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

    # MT9M113 supported resolutions (from webOS init table):
    # - Context A: 640x480 (preview)
    # - Context B: 1280x1024 (capture/full resolution)
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
# MT9M113 supported: 640x480 (preview), 1280x1024 (capture)
# Using 1280x1024 for full capture mode testing
echo ''
echo 'Setting sensor format (1280x1024 UYVY8_2X8)...'
media-ctl -d \$MEDIA_DEV -V \"'\$SENSOR':0[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || \
media-ctl -d \$MEDIA_DEV -V \"'\$SENSOR':0[fmt:UYVY8_1X16/1280x1024]\" 2>&1 || \
echo 'Sensor format set may have failed'

# Set CSIPHY format (sink and source)
echo 'Setting CSIPHY format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':0[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSIPHY':1[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || true

# Set CSID format (sink=0, PIX source=4)
echo 'Setting CSID format...'
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':0[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || true
media-ctl -d \$MEDIA_DEV -V \"'\$CSID':4[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || true

# Set VFE PIX format
echo 'Setting VFE format...'
if [ -n \"\$VFE_PIX\" ]; then
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE_PIX':0[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || true
else
    media-ctl -d \$MEDIA_DEV -V \"'\$VFE':0[fmt:UYVY8_2X8/1280x1024]\" 2>&1 || true
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

    # MT9M113 supported: 640x480 (preview), 1280x1024 (capture)
    echo 'Setting video format to 1280x1024 UYVY (MT9M113 capture mode)...'
    v4l2-ctl -d \$VIDEO_DEV --set-fmt-video=width=1280,height=1024,pixelformat=UYVY 2>&1

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
            # Expected size: 1280 * 1024 * 2 bytes/pixel * 5 frames = 13107200 bytes
            EXPECTED=\$((1280 * 1024 * 2 * FRAMES))
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

    # MT9M113 supported: 640x480 (preview), 1280x1024 (capture)
    echo 'Testing pipeline with fakesink (1280x1024 UYVY)...'
    timeout 10 gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=5 ! \
        'video/x-raw,format=UYVY,width=1280,height=1024' ! \
        fakesink 2>&1

    # If that works, try saving a frame
    if [ \$? -eq 0 ]; then
        echo ''
        echo 'Saving single frame as PPM...'
        timeout 10 gst-launch-1.0 v4l2src device=\$VIDEO_DEV num-buffers=1 ! \
            'video/x-raw,format=UYVY,width=1280,height=1024' ! \
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

    log_info "Testing capture with MT9M113 supported format..."
    run_on_device "
        if command -v gst-launch-1.0 >/dev/null 2>&1; then
            # MT9M113 supported: 640x480 (preview), 1280x1024 (capture)
            echo '=== Test: 1280x1024 UYVY (MT9M113 capture mode) ==='
            timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=3 ! \\
                'video/x-raw,format=UYVY,width=1280,height=1024' ! \\
                fakesink 2>&1

            if [ \$? -eq 0 ]; then
                echo ''
                echo 'SUCCESS: Capture completed!'
                echo ''
                echo '=== Saving test frame to /tmp/camera_frame.raw ==='
                timeout 10 gst-launch-1.0 v4l2src device=/dev/video0 num-buffers=1 ! \\
                    'video/x-raw,format=UYVY,width=1280,height=1024' ! \\
                    filesink location=/tmp/camera_frame.raw 2>&1
                ls -la /tmp/camera_frame.raw 2>/dev/null && echo 'Frame saved!'
            else
                echo ''
                echo 'Full resolution failed, trying preview mode...'

                echo ''
                echo '=== Test: 640x480 UYVY (MT9M113 preview mode) ==='
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

# Test RAW passthrough mode (RDI -> memory, no ISP)
# Uses /dev/video0 (msm_vfe0_rdi0)
# CSID pad 4 -> VFE RDI0 (all CSID sources use pad 4 on MSM8660)
test_raw_mode() {
    log_step "Testing RAW passthrough mode..."
    log_info "Path: Sensor -> CSIPHY -> CSID:4 -> VFE RDI0 -> /dev/video0"
    log_info "Data goes directly to memory, bypassing ISP"
    log_info "Using 1280x1024 (Context B - full resolution)"

    # Set AXI output mode to raw/RDI (0x60)
    set_axi_output_mode "0x60"

    run_on_device "
        echo '=== RAW Mode Test (video0 via RDI0) ==='
        echo ''

        # Setup media pipeline for RDI mode
        # All CSID sources use pad 4 on MSM8660 (no separate RDI pads)
        # RDI vs PIX is determined by VFE AXI output mode, not CSID pad
        echo 'Setting up RDI media pipeline...'

        # Reset all links first
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        # Note: sensor -> csiphy link is IMMUTABLE (always enabled)
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable RDI link: CSID pad 4 -> VFE RDI0 pad 0
        echo 'Enabling RDI link (CSID:4 -> VFE RDI0)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_rdi0\":0[1]' 2>&1 || echo '  csid:4->vfe_rdi0 link failed'

        # Set formats on the RDI path (1280x1024 = MT9M113 Context B full capture)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        echo 'Setting formats (1280x1024 Context B capture mode)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_rdi0\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_rdi0\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true

        # Set V4L2 video device format
        v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=1024,pixelformat=UYVY 2>&1 || true

        echo ''
        echo 'Current pipeline:'
        media-ctl -p 2>/dev/null | grep -E 'entity|pad|->|<-' | head -40

        echo ''
        echo 'Testing capture with 1280x1024 UYVY...'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: RAW capture completed!'
            echo ''
            echo 'Saving frame to /tmp/camera_raw.raw...'
            rm -f /tmp/camera_raw.raw
            timeout 10 v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=1 --stream-to=/tmp/camera_raw.raw 2>&1
            if [ -s /tmp/camera_raw.raw ]; then
                SIZE=\$(stat -c%s /tmp/camera_raw.raw 2>/dev/null || echo 0)
                echo \"Frame saved: \$SIZE bytes (expected 1966080 for 1280x1024 UYVY)\"
            fi
        else
            echo ''
            echo 'FAILED: RAW capture did not complete'
            echo 'Check dmesg for errors'
        fi
    "
}

# Set VFE31 AXI output mode
# 0x01  = PIX/Preview mode (through ISP processing) - webOS default
# 0x60  = Raw/RDI mode (CAMIF_TO_AXI bypassing ISP)
# 0x200 = OUTPUT_2 mode (was incorrectly used, webOS uses 0x01)
set_axi_output_mode() {
    local mode="$1"
    log_step "Setting VFE31 AXI output mode to: $mode"

    run_on_device "
        PARAM='/sys/module/qcom_camss/parameters/vfe31_axi_output_mode'
        if [ -f \"\$PARAM\" ]; then
            echo $mode > \$PARAM
            echo \"vfe31_axi_output_mode set to: \$(cat \$PARAM)\"
        else
            echo 'WARNING: vfe31_axi_output_mode parameter not found'
            echo 'Module may not be loaded or parameter not available'
        fi
    "
}

# Enable/disable VFE test generator mode
set_testgen_mode() {
    local enable="$1"
    log_step "Setting VFE test generator mode to: $enable"

    run_on_device "
        PARAM='/sys/module/qcom_camss/parameters/vfe31_use_testgen'
        if [ -f \"\$PARAM\" ]; then
            echo $enable > \$PARAM
            echo \"vfe31_use_testgen set to: \$(cat \$PARAM)\"
        else
            echo 'WARNING: vfe31_use_testgen parameter not found'
            echo 'Module may not be loaded or parameter not available'
        fi
    "
}

# Test VFE internal test generator mode
# Bypasses camera sensor entirely to verify VFE pipeline
test_testgen_mode() {
    log_step "Testing VFE internal test generator..."
    log_info "This bypasses the camera sensor entirely"
    log_info "Useful for verifying VFE pipeline independently"

    # Set AXI output mode to PIX/preview (0x01 = webOS value)
    # Note: 0x200 was incorrect - webOS uses 0x01 for ISP processing path
    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== VFE Test Generator Mode ==='
        echo ''

        # Check if parameter exists
        PARAM='/sys/module/qcom_camss/parameters/vfe31_use_testgen'
        if [ ! -f \"\$PARAM\" ]; then
            echo 'ERROR: vfe31_use_testgen parameter not found'
            echo 'Make sure qcom_camss module is loaded with testgen support'
            exit 1
        fi

        # Enable test generator
        echo '1. Enabling VFE test generator...'
        echo 1 > \$PARAM
        echo \"   vfe31_use_testgen = \$(cat \$PARAM)\"

        # Clear dmesg to see fresh output
        dmesg -c > /dev/null 2>&1

        # Reset all links first
        echo ''
        echo '2. Setting up media pipeline...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links (even for testgen, pipeline must be valid)
        echo 'Enabling upstream links...'
        media-ctl -l '\"mt9m114 4-003c\":0->\"msm_csiphy1\":0[1]' 2>&1 || echo '  sensor->csiphy link failed'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable PIX link: CSID pad 4 (PIX) -> VFE PIX pad 0
        echo 'Enabling PIX link (CSID:4 -> VFE PIX)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || echo '  csid:4->vfe_pix link failed'

        # Set formats on entire pipeline (1280x1024)
        echo 'Setting formats (1280x1024 UYVY8_2X8)...'
        media-ctl -V '\"mt9m114 4-003c\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true

        # Try capture - this should use test generator internally
        echo ''
        echo '3. Testing capture with test generator...'
        timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1
        RESULT=\$?

        # Show VFE debug output
        echo ''
        echo '4. VFE debug messages:'
        dmesg | grep -E 'VFE|TESTGEN|CAMIF|CLOCK' | tail -50

        # Disable test generator
        echo ''
        echo '5. Disabling VFE test generator...'
        echo 0 > \$PARAM
        echo \"   vfe31_use_testgen = \$(cat \$PARAM)\"

        if [ \$RESULT -eq 0 ]; then
            echo ''
            echo 'SUCCESS: Test generator capture completed!'
        else
            echo ''
            echo 'FAILED: Test generator capture did not complete'
        fi
    "
}

# Test PIX/CAMIF mode (through ISP processing)
# Uses /dev/video3 (msm_vfe0_pix)
# CSID pad 4 (PIX line) -> VFE PIX (line 3)
test_pix_mode() {
    log_step "Testing PIX/CAMIF mode (ISP processing)..."
    log_info "Path: Sensor -> CSIPHY -> CSID:4 -> VFE PIX -> /dev/video3"
    log_info "Data goes through VFE ISP for processing"
    log_info "Using 1280x1024 (Context B - full resolution)"

    # Set AXI output mode to PIX/preview (0x01 = webOS value for ISP path)
    # Note: 0x200 is raw/CAMIF_TO_AXI bypass, 0x01 is ISP processing path
    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== PIX Mode Test (video3 via VFE PIX) ==='
        echo ''

        # MT9M113 supported resolutions (from webOS init table):
        # - Context A: 640x480 (preview)
        # - Context B: 1280x1024 (capture/full resolution)
        # Using Context B for FULL RESOLUTION capture

        # Reset all links first
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        # Note: sensor -> csiphy link is IMMUTABLE (always enabled)
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable PIX link: CSID pad 4 (PIX) -> VFE PIX pad 0
        echo 'Enabling PIX link (CSID:4 -> VFE PIX)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || echo '  csid:4->vfe_pix link failed'

        # Set formats on entire pipeline (1280x1024 = MT9M113 Context B full capture)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        echo 'Setting formats (1280x1024 Context B capture mode)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":1[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true

        # Set V4L2 video device format to NV12 (native VFE31 output)
        v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=1024,pixelformat=NV12 2>&1 || true

        echo ''
        echo 'Current pipeline configuration:'
        media-ctl -p 2>/dev/null | grep -E 'entity|pad|->|<-' | head -40

        echo ''
        echo 'Testing capture with 1280x1024 NV16 (MT9M113 Context B)...'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=NV12,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: PIX capture completed!'
            echo ''
            echo 'Saving frame to /tmp/camera_pix.raw...'
            rm -f /tmp/camera_pix.raw
            timeout 10 v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=1 --stream-to=/tmp/camera_pix.raw 2>&1
            if [ -s /tmp/camera_pix.raw ]; then
                SIZE=\$(stat -c%s /tmp/camera_pix.raw 2>/dev/null || echo 0)
                echo \"Frame saved: \$SIZE bytes (expected 1966080 for 1280x1024 NV12)\"
            fi
        else
            echo ''
            echo 'FAILED: PIX capture did not complete'
            echo ''
            echo 'Check dmesg for errors'
        fi
    "
}

# Test with v4l2-ctl directly (bypasses GStreamer)
# This is useful for debugging as it has fewer layers
test_v4l2_mode() {
    log_step "Testing PIX mode with v4l2-ctl (direct capture)..."
    log_info "Path: Sensor -> CSIPHY -> CSID:4 -> VFE PIX -> /dev/video3"
    log_info "Using v4l2-ctl instead of GStreamer for direct capture"

    # Set AXI output mode to PIX/preview (0x01 = webOS value)
    # Note: 0x200 was incorrect - webOS uses 0x01 for ISP processing path
    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== v4l2-ctl Direct Capture Test (video3 via VFE PIX) ==='
        echo ''

        # Reset all links first
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        # Note: sensor -> csiphy link is IMMUTABLE (always enabled)
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable PIX link: CSID pad 4 (PIX) -> VFE PIX pad 0
        echo 'Enabling PIX link (CSID:4 -> VFE PIX)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || echo '  csid:4->vfe_pix link failed'

        # Set formats on entire pipeline (1280x1024 = MT9M113 Context B)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        # The sensor output format on pad 1 is derived from the compose size
        echo 'Setting formats (1280x1024)...'
        echo 'Setting compose rectangle to 1280x1024 (triggers Context B)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true

        echo ''
        echo 'Current pipeline configuration:'
        media-ctl -p 2>/dev/null | grep -E 'entity|pad|->|<-' | head -40

        echo ''
        echo 'Setting video format on /dev/video3...'
        v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=1024,pixelformat=UYVY 2>&1
        echo ''
        echo 'Current format:'
        v4l2-ctl -d /dev/video3 --get-fmt-video 2>&1

        echo ''
        echo 'Capturing 5 frames with v4l2-ctl...'
        rm -f /tmp/frame.raw
        timeout 10 v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=5 --stream-to=/tmp/frame.raw 2>&1

        if [ \$? -eq 0 ] && [ -s /tmp/frame.raw ]; then
            SIZE=\$(stat -c%s /tmp/frame.raw 2>/dev/null || echo 0)
            EXPECTED=\$((1280 * 1024 * 2 * 5))  # 5 frames of UYVY
            echo ''
            echo \"SUCCESS: Captured \$SIZE bytes (expected ~\$EXPECTED for 5 frames)\"
            echo 'Frame saved to /tmp/frame.raw'
        else
            echo ''
            echo 'FAILED: v4l2-ctl capture did not complete'
            echo ''
            echo 'Trying 640x480 mode...'
            media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/640x480]' 2>&1 || true
            media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/640x480]' 2>&1 || true
            media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/640x480]' 2>&1 || true
            media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/640x480]' 2>&1 || true
            media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/640x480]' 2>&1 || true
            media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/640x480]' 2>&1 || true
            v4l2-ctl -d /dev/video3 --set-fmt-video=width=640,height=480,pixelformat=UYVY 2>&1
            rm -f /tmp/frame.raw
            timeout 10 v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=5 --stream-to=/tmp/frame.raw 2>&1
            if [ \$? -eq 0 ] && [ -s /tmp/frame.raw ]; then
                SIZE=\$(stat -c%s /tmp/frame.raw 2>/dev/null || echo 0)
                echo \"SUCCESS: 640x480 capture - \$SIZE bytes\"
            fi
        fi
    "
}

# Test VIDEO mode (preview + video output via WM4/WM5)
# Uses AXI mode 0x01 with XBAR_CFG1 = 0x1a1b
# This configures both preview (WM0/WM1) and video (WM4/WM5) outputs
test_video_mode() {
    log_step "Testing VIDEO mode (preview + video output)..."
    log_info "Path: Sensor -> CSIPHY -> CSID:4 -> VFE PIX -> /dev/video3"
    log_info "AXI mode 0x01 enables both preview (WM0/1) and video (WM4/5) paths"
    log_info "Using 1280x1024 (Context B - full resolution)"

    # Set AXI output mode to PIX+Video (0x01)
    set_axi_output_mode "0x01"

    # Enable video output
    set_video_output "1"

    run_on_device "
        echo '=== VIDEO Mode Test (preview + video output) ==='
        echo ''
        echo 'AXI mode 0x01 = preview + video'
        echo 'XBAR_CFG1 = 0x1a1b (routes to WM0/1 + WM4/5)'
        echo ''

        # Reset all links first
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable PIX link: CSID pad 4 (PIX) -> VFE PIX pad 0
        echo 'Enabling PIX link (CSID:4 -> VFE PIX)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || echo '  csid:4->vfe_pix link failed'

        # Set formats on entire pipeline (1280x1024 = MT9M113 Context B full capture)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        echo 'Setting formats (1280x1024 Context B capture mode)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":1[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true

        # Set V4L2 video device format to NV12 (native VFE31 output)
        v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=1024,pixelformat=NV12 2>&1 || true

        echo ''
        echo 'Current module parameters:'
        echo \"  vfe31_axi_output_mode = \$(cat /sys/module/qcom_camss/parameters/vfe31_axi_output_mode 2>/dev/null)\"
        echo \"  vfe31_video_output_enable = \$(cat /sys/module/qcom_camss/parameters/vfe31_video_output_enable 2>/dev/null)\"

        echo ''
        echo 'Testing capture with 1280x1024 NV16 (VIDEO mode)...'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=NV12,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: VIDEO mode capture completed!'
            echo ''
            echo 'Saving frame to /tmp/camera_video.raw...'
            rm -f /tmp/camera_video.raw
            timeout 10 v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=1 --stream-to=/tmp/camera_video.raw 2>&1
            if [ -s /tmp/camera_video.raw ]; then
                SIZE=\$(stat -c%s /tmp/camera_video.raw 2>/dev/null || echo 0)
                echo \"Frame saved: \$SIZE bytes (expected 1966080 for 1280x1024 NV12)\"
            fi
        else
            echo ''
            echo 'FAILED: VIDEO mode capture did not complete'
            echo 'Check dmesg for errors'
        fi
    "

    # Disable video output after test
    set_video_output "0"
}

# Enable/disable VFE31 video output (WM4/WM5)
set_video_output() {
    local enable="$1"
    log_step "Setting VFE31 video output enable to: $enable"

    run_on_device "
        PARAM='/sys/module/qcom_camss/parameters/vfe31_video_output_enable'
        if [ -f \"\$PARAM\" ]; then
            echo $enable > \$PARAM
            echo \"vfe31_video_output_enable set to: \$(cat \$PARAM)\"
        else
            echo 'WARNING: vfe31_video_output_enable parameter not found'
            echo 'Module may not be loaded or parameter not available'
        fi
    "
}

# Test VIDEO4 mode - dual output capture via WM4/WM5
# Uses /dev/video4 (msm_vfe0_video) for high-quality capture
# This is the RECOMMENDED mode for capture as it uses dedicated WMs
# CSID pad 4 (shared with PIX) -> VFE VIDEO (line 4)
# Note: VFE_LINE_VIDEO shares CSID pad with VFE_LINE_PIX - both use pad 4
test_video4_mode() {
    log_step "Testing VIDEO4 mode (capture via WM4/WM5)..."
    log_info "Path: Sensor -> CSIPHY -> CSID:4 -> VFE VIDEO -> /dev/video4"
    log_info "Uses WM4 (Y) + WM5 (CbCr) for dedicated capture output"
    log_info "This is the RECOMMENDED mode for FULL RESOLUTION capture"

    # Set AXI output mode to PIX/preview (0x01 = webOS value)
    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== VIDEO4 Mode Test (video4 via VFE VIDEO with WM4/WM5) ==='
        echo ''
        echo 'VIDEO4 uses dedicated write masters for capture:'
        echo '  - WM4: Y plane (luma)'
        echo '  - WM5: CbCr plane (interleaved chroma)'
        echo ''
        echo 'This leaves WM0/WM1 free for preview output'
        echo 'Using FULL RESOLUTION 1280x1024 (MT9M113 Context B capture mode)'
        echo ''

        # Reset all links first
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable VIDEO link: CSID pad 4 -> VFE VIDEO pad 0
        # VFE_LINE_VIDEO shares CSID pad 4 with VFE_LINE_PIX (both from CAMIF/DEMUX)
        echo 'Enabling VIDEO link (CSID:4 -> VFE VIDEO)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_video\":0[1]' 2>&1 || echo '  csid:4->vfe_video link failed'

        # Set formats on entire pipeline (1280x1024 = MT9M113 Context B full capture)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        # The sensor output format on pad 1 is derived from the compose size
        echo 'Setting formats (1280x1024 capture mode)...'
        echo 'Setting compose rectangle to 1280x1024 (triggers Context B)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_video\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_video\":1[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true

        # Set V4L2 video device format to NV12 (native VFE31 output)
        echo ''
        echo 'Setting V4L2 format to NV16 1280x1024 on /dev/video4...'
        v4l2-ctl -d /dev/video4 --set-fmt-video=width=1280,height=1024,pixelformat=NV12 2>&1 || true
        echo ''
        echo 'Current format:'
        v4l2-ctl -d /dev/video4 --get-fmt-video 2>&1

        echo ''
        echo 'Current pipeline configuration:'
        media-ctl -p 2>/dev/null | grep -E 'entity|pad|->|<-' | head -50

        echo ''
        echo 'Current module parameters:'
        echo \"  vfe31_axi_output_mode = \$(cat /sys/module/qcom_camss/parameters/vfe31_axi_output_mode 2>/dev/null)\"
        echo \"  vfe31_video_output_enable = \$(cat /sys/module/qcom_camss/parameters/vfe31_video_output_enable 2>/dev/null)\"
        echo \"  vfe31_xbar_cfg1 = \$(cat /sys/module/qcom_camss/parameters/vfe31_xbar_cfg1 2>/dev/null)\"

        echo ''
        echo 'Testing capture with 1280x1024 NV16 on VIDEO4...'
        echo 'Expected size: 1280*1024 (Y) + 1280*1024 (CbCr) = 2621440 bytes per frame'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video4 num-buffers=10 ! \\
            'video/x-raw,format=NV12,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: VIDEO4 capture completed!'
            echo ''
            echo 'Saving frame to /tmp/capture_video4.raw...'
            rm -f /tmp/capture_video4.raw
            timeout 10 v4l2-ctl -d /dev/video4 --stream-mmap --stream-count=1 --stream-to=/tmp/capture_video4.raw 2>&1
            if [ -s /tmp/capture_video4.raw ]; then
                SIZE=\$(stat -c%s /tmp/capture_video4.raw 2>/dev/null || echo 0)
                echo \"Frame saved: \$SIZE bytes (expected 1966080 for 1280x1024 NV12)\"
            fi
        else
            echo ''
            echo 'FAILED: VIDEO4 capture did not complete'
            echo ''
            echo 'Trying v4l2-ctl direct capture...'
            rm -f /tmp/capture_video4.raw
            timeout 10 v4l2-ctl -d /dev/video4 --stream-mmap --stream-count=1 --stream-to=/tmp/capture_video4.raw 2>&1
            if [ -s /tmp/capture_video4.raw ]; then
                SIZE=\$(stat -c%s /tmp/capture_video4.raw 2>/dev/null || echo 0)
                echo \"SUCCESS with v4l2-ctl: \$SIZE bytes captured\"
            fi
        fi
    "
}

# Test NV16 semi-planar format (native VFE31 DEMUX output)
# VFE31 DEMUX outputs Y to WM0 and CbCr to WM1 in NV16 format
# This is the NATIVE output format - no conversion needed
test_nv16_mode() {
    log_step "Testing NV16 semi-planar format (native VFE31 output)..."
    log_info "Path: Sensor -> CSIPHY -> CSID:4 -> VFE PIX -> /dev/video3"
    log_info "VFE31 DEMUX outputs: Y plane to WM0, CbCr plane to WM1"
    log_info "NV16 = Y plane followed by interleaved CbCr plane"
    log_info "Using 1280x1024 (Context B - full resolution)"

    # Set AXI output mode to PIX/preview (0x01 = webOS value)
    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== NV16 Mode Test (video3 via VFE PIX) ==='
        echo ''
        echo 'NV16 is the NATIVE VFE31 DEMUX output format:'
        echo '  - WM0: Y plane (luma)'
        echo '  - WM1: CbCr plane (interleaved chroma)'
        echo ''

        # Reset all links first
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable PIX link: CSID pad 4 (PIX) -> VFE PIX pad 0
        echo 'Enabling PIX link (CSID:4 -> VFE PIX)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || echo '  csid:4->vfe_pix link failed'

        # Set formats on entire pipeline (1280x1024 = MT9M113 Context B full capture)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        echo 'Setting formats (1280x1024 Context B capture mode)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":1[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true

        # Set V4L2 video device format to NV16
        echo ''
        echo 'Setting V4L2 format to NV16 1280x1024...'
        v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=1024,pixelformat=NV12 2>&1 || true
        echo ''
        echo 'Current format:'
        v4l2-ctl -d /dev/video3 --get-fmt-video 2>&1

        echo ''
        echo 'Current pipeline configuration:'
        media-ctl -p 2>/dev/null | grep -E 'entity|pad|->|<-' | head -40

        echo ''
        echo 'Testing capture with 1280x1024 NV16...'
        echo 'Expected size: 1280*1024 (Y) + 1280*1024 (CbCr) = 2621440 bytes per frame'
        timeout 20 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=NV12,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1

        if [ \$? -eq 0 ]; then
            echo ''
            echo 'SUCCESS: NV16 capture completed!'
            echo ''
            echo 'Saving frame to /tmp/camera_nv16.raw...'
            timeout 10 gst-launch-1.0 v4l2src device=/dev/video3 num-buffers=1 ! \\
                'video/x-raw,format=NV12,width=1280,height=1024' ! \\
                filesink location=/tmp/camera_nv16.raw 2>&1
            if [ -f /tmp/camera_nv16.raw ]; then
                SIZE=\$(stat -c%s /tmp/camera_nv16.raw 2>/dev/null || echo 0)
                echo \"Frame saved: \$SIZE bytes (expected 1966080 for 1280x1024 NV12)\"
            fi
        else
            echo ''
            echo 'FAILED: NV16 capture did not complete'
            echo ''
            echo 'Trying v4l2-ctl direct capture...'
            rm -f /tmp/camera_nv16.raw
            timeout 10 v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=1 --stream-to=/tmp/camera_nv16.raw 2>&1
            if [ -s /tmp/camera_nv16.raw ]; then
                SIZE=\$(stat -c%s /tmp/camera_nv16.raw 2>/dev/null || echo 0)
                echo \"SUCCESS with v4l2-ctl: \$SIZE bytes captured\"
            fi
        fi
    "
}

# Capture frames in specified format and fetch to host
# Usage: capture_and_fetch <mode> <format> <output_file>
capture_frames() {
    local mode="$1"
    local format="${2:-NV16}"
    local width="${3:-1280}"
    local height="${4:-1024}"
    local frames="${5:-1}"

    log_step "Capturing $frames frame(s) in $format format ($width x $height)..."

    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== Capture Mode: $format $width x $height ==='

        # Reset and setup pipeline
        media-ctl -r 2>/dev/null || true
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || true
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || true

        # Set formats - set compose first to trigger Context B for 1280x1024
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/'$width'x'$height']' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":1[fmt:UYVY8_2X8/'$width'x'$height']' 2>&1 || true

        # Set V4L2 format
        v4l2-ctl -d /dev/video3 --set-fmt-video=width=$width,height=$height,pixelformat=$format 2>&1 || true

        # Capture
        rm -f /tmp/capture_$format.raw
        timeout 15 v4l2-ctl -d /dev/video3 --stream-mmap --stream-count=$frames --stream-to=/tmp/capture_$format.raw 2>&1

        if [ -s /tmp/capture_$format.raw ]; then
            SIZE=\$(stat -c%s /tmp/capture_$format.raw 2>/dev/null || echo 0)
            echo \"SUCCESS: Captured \$SIZE bytes to /tmp/capture_$format.raw\"
        else
            echo 'FAILED: No capture data'
        fi
    "
}

# Fetch captures from device
fetch_captures() {
    local dest="${1:-.}"
    log_step "Fetching captures from device to $dest..."

    # Get list of captured files
    local files=$(run_on_device "ls -1 /tmp/camera_*.raw /tmp/capture_*.raw 2>/dev/null" || true)

    if [ -z "$files" ]; then
        log_error "No capture files found on device"
        return 1
    fi

    log_info "Found captures:"
    echo "$files"

    for f in $files; do
        local basename=$(basename "$f")
        log_info "Fetching $basename..."
        scp -P $SSH_PORT -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            root@$DEVICE_IP:"$f" "$dest/$basename" 2>/dev/null
        if [ -f "$dest/$basename" ]; then
            log_result "Saved: $dest/$basename ($(stat -c%s "$dest/$basename") bytes)"
        fi
    done
}

# Test legacy routing mode (webOS-style MISC_CC=0x0400)
# This is now the DEFAULT and RECOMMENDED mode for MSM8660/VFE31
# Uses direct CSI1_VFE_CLK routing like webOS (MISC_CC bit 10 only)
test_legacy_mode() {
    log_step "Testing LEGACY routing mode (webOS-style MISC_CC=0x0400)..."
    log_info "This is the RECOMMENDED mode for MSM8660 (matches webOS)"
    log_info "Data routes directly via CSI1_VFE_CLK"

    # Set AXI output mode to PIX/preview (0x01 = webOS value)
    set_axi_output_mode "0x01"

    run_on_device "
        echo '=== LEGACY Routing Mode Test ==='
        echo ''
        echo 'This mode writes MISC_CC_REG=0x0 like webOS'
        echo 'Data routes: CSIPHY1 -> CSI1_VFE_CLK -> VFE CAMIF'
        echo ''

        # Check current module parameters
        echo 'Current module parameters:'
        cat /sys/module/qcom_camss/parameters/vfe31_axi_output_mode 2>/dev/null || echo '  (not found)'
        echo ''

        # Reset all links first
        echo 'Resetting media links...'
        media-ctl -r 2>/dev/null || true

        # Enable upstream links: csiphy -> csid
        # Note: sensor -> csiphy link is IMMUTABLE (always enabled)
        echo 'Enabling upstream links...'
        media-ctl -l '\"msm_csiphy1\":1->\"msm_csid1\":0[1]' 2>&1 || echo '  csiphy->csid link failed'

        # Enable PIX link: CSID pad 4 (PIX) -> VFE PIX pad 0
        echo 'Enabling PIX link (CSID:4 -> VFE PIX)...'
        media-ctl -l '\"msm_csid1\":4->\"msm_vfe0_pix\":0[1]' 2>&1 || echo '  csid:4->vfe_pix link failed'

        # Set formats on entire pipeline (1280x1024 = MT9M113 Context B)
        # IMPORTANT: Set compose rectangle on IFP pad 0 to trigger Context B
        # The sensor output format on pad 1 is derived from the compose size
        echo 'Setting formats (1280x1024)...'
        echo 'Setting compose rectangle to 1280x1024 (triggers Context B)...'
        media-ctl -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/1280x1024]' 2>&1 || true
        media-ctl -V '\"mt9m114 ifp 4-003c\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csiphy1\":1[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":0[fmt:UYVY8_1X16/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_csid1\":4[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        media-ctl -V '\"msm_vfe0_pix\":0[fmt:UYVY8_2X8/1280x1024]' 2>&1 || true
        echo 'Pipeline configured'

        echo ''
        echo 'Testing capture with 1280x1024 UYVY (LEGACY mode)...'
        timeout 15 gst-launch-1.0 -v v4l2src device=/dev/video3 num-buffers=10 ! \\
            'video/x-raw,format=UYVY,width=1280,height=1024,framerate=30/1' ! \\
            fakesink 2>&1

        RESULT=\$?

        echo ''
        echo 'Checking MISC_CC_REG state after test...'
        # Need devmem to check, but dmesg should show it
        dmesg | grep -E 'MISC_CC|LEGACY' | tail -10

        if [ \$RESULT -eq 0 ]; then
            echo ''
            echo 'SUCCESS: LEGACY mode capture completed!'
        else
            echo ''
            echo 'FAILED: LEGACY mode capture did not complete'
            echo 'Check dmesg for errors'
        fi
    "
    # Note: Keep legacy routing enabled - it's the correct mode for MSM8660
}

# Comprehensive test at specific resolution
# Usage: test_resolution <width> <height> <mode> <video_dev>
# mode: pix, rdi, video, testgen
# video_dev: video0-4
test_at_resolution() {
    local width="$1"
    local height="$2"
    local mode="$3"
    local video_dev="${4:-video3}"
    local csid="${5:-msm_csid1}"
    local csiphy="${6:-msm_csiphy1}"
    local format="${7:-NV12}"  # Default to NV12, can be NV16

    log_info "Testing $mode at ${width}x${height} on /dev/$video_dev (format=$format)"

    run_on_device "
        # Kill any stuck capture processes first
        pkill -9 v4l2-ctl 2>/dev/null || true
        pkill -9 gst-launch 2>/dev/null || true
        sleep 1

        # Reset pipeline completely (this disables all non-immutable links)
        media-ctl -d /dev/media0 -r 2>/dev/null || true
        sleep 0.5

        # Mode-specific format setup (links will be enabled AFTER formats are set)
        # NOTE: CSID output format is always UYVY8_1X16 (from sensor via MIPI)
        # VFE PIX/VIDEO input accepts UYVY8_1X16 and converts internally via DEMUX
        # Format passed from caller (default NV12 for semi-planar modes)
        PIXFMT='$format'

        case '$mode' in
            pix)
                # PIX mode: CSID pad 4 -> VFE PIX
                # Supports NV12 (4:2:0) and NV16 (4:2:2) output formats
                CSID_PAD=4
                VFE_ENTITY='msm_vfe0_pix'
                VFE_FMT='UYVY8_1X16'
                # For NV16, also set hardware mode override
                if [ \"\$PIXFMT\" = \"NV16\" ] || [ \"\$PIXFMT\" = \"NV61\" ]; then
                    echo 2 > /sys/module/qcom_camss/parameters/vfe31_force_422 2>/dev/null || true
                else
                    echo 0 > /sys/module/qcom_camss/parameters/vfe31_force_422 2>/dev/null || true
                fi
                ;;
            rdi)
                # RDI mode: Raw bypass through CAMIF (AXI=0x60)
                # On MSM8660/VFE31, there's no hardware RDI path - all data goes
                # through CAMIF. RDI is emulated via AXI output mode 0x60.
                #
                # MT9M113 outputs 10-bit Bayer in 8+2 format (2 bytes per pixel).
                # Use UYVY format for stride calculation (2 bytes/pixel) since
                # VFE doesn't support unpacked 10-bit Bayer. The actual data
                # is 10-bit Bayer but we treat it as raw bytes for transport.
                CSID_PAD=4
                VFE_ENTITY='msm_vfe0_rdi0'
                VFE_FMT='UYVY8_1X16'
                PIXFMT='UYVY'
                USE_RAW_FORMAT=1
                ;;
            video)
                # VIDEO mode: CSID pad 4 -> VFE VIDEO
                # Supports NV12 (4:2:0) and NV16 (4:2:2) output formats
                CSID_PAD=4
                VFE_ENTITY='msm_vfe0_video'
                VFE_FMT='UYVY8_1X16'
                # For NV16, also set hardware mode override
                if [ \"\$PIXFMT\" = \"NV16\" ] || [ \"\$PIXFMT\" = \"NV61\" ]; then
                    echo 2 > /sys/module/qcom_camss/parameters/vfe31_force_422 2>/dev/null || true
                else
                    echo 0 > /sys/module/qcom_camss/parameters/vfe31_force_422 2>/dev/null || true
                fi
                ;;
            testgen)
                # TESTGEN mode: same as PIX but with testgen enabled
                CSID_PAD=4
                VFE_ENTITY='msm_vfe0_pix'
                VFE_FMT='UYVY8_1X16'
                echo 1 > /sys/module/qcom_camss/parameters/vfe31_use_testgen 2>/dev/null || true
                # For NV16, also set hardware mode override
                if [ \"\$PIXFMT\" = \"NV16\" ] || [ \"\$PIXFMT\" = \"NV61\" ]; then
                    echo 2 > /sys/module/qcom_camss/parameters/vfe31_force_422 2>/dev/null || true
                else
                    echo 0 > /sys/module/qcom_camss/parameters/vfe31_force_422 2>/dev/null || true
                fi
                ;;
        esac

        # Set sensor output format first
        # For RDI mode, we use UYVY format for consistent 2 bytes/pixel stride.
        # MT9M113 outputs 10-bit Bayer in 8+2 format (2 bytes/pixel), but
        # since the VFE doesn't support unpacked 10-bit, we use UYVY as a
        # transport format. The actual data will be 10-bit Bayer.
        if [ -n \"\$USE_RAW_FORMAT\" ]; then
            # Use UYVY for RDI to get 2 bytes/pixel layout
            SENSOR_FMT='UYVY8_1X16'
            echo 'Configuring sensor for RDI mode (UYVY transport, 2 bytes/pixel)...'
        else
            SENSOR_FMT='UYVY8_1X16'
        fi

        media-ctl -d /dev/media0 -V '\"mt9m114 ifp 4-003c\":0[compose:(0,0)/'$width'x'$height']' 2>&1 || true
        media-ctl -d /dev/media0 -V \"\\\"mt9m114 ifp 4-003c\\\":1[fmt:\${SENSOR_FMT}/${width}x${height}]\" 2>&1 || true

        # Set CSIPHY format
        media-ctl -d /dev/media0 -V \"\\\"$csiphy\\\":0[fmt:\${SENSOR_FMT}/${width}x${height}]\" 2>&1 || true
        media-ctl -d /dev/media0 -V \"\\\"$csiphy\\\":1[fmt:\${SENSOR_FMT}/${width}x${height}]\" 2>&1 || true

        # Set CSID formats (input and output)
        media-ctl -d /dev/media0 -V \"\\\"$csid\\\":0[fmt:\${SENSOR_FMT}/${width}x${height}]\" 2>&1 || true
        media-ctl -d /dev/media0 -V \"\\\"$csid\\\":\${CSID_PAD}[fmt:\${SENSOR_FMT}/${width}x${height}]\" 2>&1 || true

        # Set VFE entity format - must match CSID output
        media-ctl -d /dev/media0 -V \"\\\"\${VFE_ENTITY}\\\":0[fmt:\${VFE_FMT}/${width}x${height}]\" 2>&1 || true

        # Enable all pipeline links AFTER setting formats (format changes can disable links)
        # Link order: CSIPHY -> CSID -> VFE
        media-ctl -d /dev/media0 -l \"\\\"$csiphy\\\":1->\\\"$csid\\\":0[1]\" 2>&1 || true
        media-ctl -d /dev/media0 -l \"\\\"$csid\\\":\${CSID_PAD}->\\\"\${VFE_ENTITY}\\\":0[1]\" 2>&1 || true

        # Verify links are enabled
        echo \"Verifying pipeline links...\"
        media-ctl -d /dev/media0 -p 2>&1 | grep -E \"\${VFE_ENTITY}.*ENABLED\" || echo 'WARNING: VFE link not enabled!'

        # Set V4L2 device format
        v4l2-ctl -d /dev/$video_dev --set-fmt-video=width=$width,height=$height,pixelformat=\$PIXFMT 2>&1 || true

        # Show configured format
        echo \"Pipeline configured for $mode mode (format=\$PIXFMT):\"
        v4l2-ctl -d /dev/$video_dev --get-fmt-video 2>&1 | grep -E 'Width|Pixel'

        # Capture with register dump during capture
        # Include format in filename for NV12 vs NV16 comparison
        OUTPUT=\"/tmp/test_${mode}_${width}x${height}_\${PIXFMT}.raw\"
        REGDUMP=\"/tmp/test_${mode}_${width}x${height}_\${PIXFMT}_regs.txt\"
        rm -f \"\$OUTPUT\" \"\$REGDUMP\"

        # Start capture in background
        timeout 15 v4l2-ctl -d /dev/$video_dev --stream-mmap --stream-count=3 --stream-to=\"\$OUTPUT\" &
        CAP_PID=\$!

        # Wait for capture to start
        sleep 0.5

        # Dump VFE31 registers during capture
        VFE=0x04500000
        {
            echo \"=== VFE31 Registers for $mode ${width}x${height} ===\"
            echo \"MODULE_CFG:   \$(devmem \$((VFE + 0x010)) 2>/dev/null)\"
            echo \"CORE_CFG:     \$(devmem \$((VFE + 0x014)) 2>/dev/null)\"
            echo \"IRQ_COMP:     \$(devmem \$((VFE + 0x034)) 2>/dev/null)\"
            echo \"BUS_CFG:      \$(devmem \$((VFE + 0x03C)) 2>/dev/null)\"
            echo \"AXI_OUT:      \$(devmem \$((VFE + 0x040)) 2>/dev/null)\"
            echo \"XBAR_CFG1:    \$(devmem \$((VFE + 0x044)) 2>/dev/null)\"
            echo \"WM0_CFG:      \$(devmem \$((VFE + 0x04C)) 2>/dev/null)\"
            echo \"WM0_ADDR_CFG: \$(devmem \$((VFE + 0x058)) 2>/dev/null)\"
            echo \"WM0_UB_CFG:   \$(devmem \$((VFE + 0x05C)) 2>/dev/null)\"
            echo \"WM0_IMG_SIZE: \$(devmem \$((VFE + 0x060)) 2>/dev/null)\"
            echo \"WM1_CFG:      \$(devmem \$((VFE + 0x064)) 2>/dev/null)\"
            echo \"WM1_ADDR_CFG: \$(devmem \$((VFE + 0x070)) 2>/dev/null)\"
            echo \"WM1_UB_CFG:   \$(devmem \$((VFE + 0x074)) 2>/dev/null)\"
            echo \"WM1_IMG_SIZE: \$(devmem \$((VFE + 0x078)) 2>/dev/null)\"
            echo \"WM4_CFG:      \$(devmem \$((VFE + 0x0AC)) 2>/dev/null)\"
            echo \"WM4_ADDR_CFG: \$(devmem \$((VFE + 0x0B8)) 2>/dev/null)\"
            echo \"WM4_UB_CFG:   \$(devmem \$((VFE + 0x0BC)) 2>/dev/null)\"
            echo \"WM4_IMG_SIZE: \$(devmem \$((VFE + 0x0C0)) 2>/dev/null)\"
            echo \"CAMIF_WIN_W:  \$(devmem \$((VFE + 0x1EC)) 2>/dev/null)\"
            echo \"CAMIF_WIN_H:  \$(devmem \$((VFE + 0x1F0)) 2>/dev/null)\"
            echo \"CAMIF_SUBS0:  \$(devmem \$((VFE + 0x1F4)) 2>/dev/null)\"
            echo \"FOV_Y:        \$(devmem \$((VFE + 0x360)) 2>/dev/null)\"
            echo \"FOV_CBCR:     \$(devmem \$((VFE + 0x364)) 2>/dev/null)\"
            echo \"SCALE_Y_V:    \$(devmem \$((VFE + 0x378)) 2>/dev/null)\"
            echo \"CHROMA_V:     \$(devmem \$((VFE + 0x4F0)) 2>/dev/null)\"
            echo \"DEMUX_EVEN:   \$(devmem \$((VFE + 0x290)) 2>/dev/null)\"
            echo \"DEMUX_ODD:    \$(devmem \$((VFE + 0x294)) 2>/dev/null)\"
        } > \"\$REGDUMP\" 2>&1

        # Wait for capture to complete
        wait \$CAP_PID 2>/dev/null || true

        # Show register dump
        echo \"VFE31 Registers:\"
        cat \"\$REGDUMP\" | head -20

        # Disable testgen if it was enabled
        if [ '$mode' = 'testgen' ]; then
            echo 0 > /sys/module/qcom_camss/parameters/vfe31_use_testgen 2>/dev/null || true
        fi

        # Clear RAW format flag (driver auto-detects AXI mode based on line type)
        USE_RAW_FORMAT=

        # Check result
        if [ -s \"\$OUTPUT\" ]; then
            SIZE=\$(stat -c%s \"\$OUTPUT\" 2>/dev/null || echo 0)
            echo \"PASS: $mode ${width}x${height} - \$SIZE bytes captured\"
        else
            echo \"FAIL: $mode ${width}x${height} - no data captured\"
        fi
    "
}

# Convert raw frames to PNG with timestamped filenames
# Usage: convert_frames <mode> <width> <height> <raw_file_on_device>
# Output: captures/mode_YYYYMMDD_HHMMSS_fN.png
convert_frames() {
    local mode="$1"
    local width="$2"
    local height="$3"
    local remote_raw="${4:-/tmp/test_${mode}_${width}x${height}.raw}"
    local timestamp=$(date +%Y%m%d_%H%M%S)
    local local_raw="/tmp/${mode}${width}_${timestamp}.raw"
    local output_dir="captures"

    # Ensure output directory exists
    mkdir -p "$output_dir"
    local frame_size
    local num_frames
    local pixfmt

    # Determine frame size and pixel format based on mode
    case "$mode" in
        rdi)
            # RAW8 Bayer - 1 byte per pixel
            frame_size=$((width * height))
            pixfmt="gray"  # Use gray for Bayer, demosaic separately
            ;;
        pix|video|testgen)
            # NV16 - Y plane + UV plane = 2 bytes per pixel
            frame_size=$((width * height * 2))
            pixfmt="nv16"
            ;;
        *)
            log_error "Unknown mode: $mode"
            return 1
            ;;
    esac

    # Fetch raw file from device
    log_info "Fetching $remote_raw from device..."
    scp -P $SSH_PORT root@$DEVICE_IP:"$remote_raw" "$local_raw" 2>/dev/null
    if [ ! -f "$local_raw" ]; then
        log_error "Failed to fetch raw file"
        return 1
    fi

    local file_size=$(stat -c%s "$local_raw" 2>/dev/null || echo 0)
    num_frames=$((file_size / frame_size))

    if [ "$num_frames" -eq 0 ]; then
        log_error "Raw file too small or empty"
        rm -f "$local_raw"
        return 1
    fi

    log_info "Converting $num_frames frames (${width}x${height} $pixfmt)..."

    # Extract and convert each frame
    for i in $(seq 1 $num_frames); do
        local frame_raw="/tmp/${mode}${width}_f${i}.raw"
        local frame_png="${output_dir}/${mode}${width}_${timestamp}_f${i}.png"

        # Extract frame using dd with efficient block size
        dd if="$local_raw" bs=$frame_size skip=$((i-1)) count=1 of="$frame_raw" 2>/dev/null

        # Convert to PNG using ffmpeg
        if [ "$pixfmt" = "gray" ]; then
            # For RAW Bayer, just save as grayscale (proper demosaic needs more work)
            ffmpeg -y -f rawvideo -pix_fmt gray -s ${width}x${height} \
                   -i "$frame_raw" "$frame_png" 2>/dev/null
        else
            # For NV16 semi-planar
            ffmpeg -y -f rawvideo -pix_fmt nv16 -s ${width}x${height} \
                   -i "$frame_raw" "$frame_png" 2>/dev/null
        fi

        rm -f "$frame_raw"

        if [ -f "$frame_png" ]; then
            local png_size=$(stat -c%s "$frame_png" 2>/dev/null)
            echo "  Created: $frame_png ($png_size bytes)"
        fi
    done

    rm -f "$local_raw"
    log_info "Conversion complete: ${output_dir}/${mode}${width}_${timestamp}_f[1-${num_frames}].png"
}

# Comprehensive test - all modes at both resolutions
test_comprehensive() {
    log_step "Running comprehensive camera tests..."
    echo ""
    echo "=============================================="
    echo "  Comprehensive Camera Test"
    echo "  Resolutions: 640x480, 1280x1024"
    echo "  Modes: PIX, RDI, VIDEO, TESTGEN"
    echo "=============================================="
    echo ""

    # Results array
    local results=""

    # Test PIX mode (video3) at both resolutions
    log_step "=== PIX Mode Tests (video3) ==="
    echo ""
    test_at_resolution 640 480 pix video3 msm_csid1 msm_csiphy1
    echo ""
    test_at_resolution 1280 1024 pix video3 msm_csid1 msm_csiphy1
    echo ""

    # Test RDI mode (video0) at both resolutions
    log_step "=== RDI Mode Tests (video0) ==="
    echo ""
    test_at_resolution 640 480 rdi video0 msm_csid1 msm_csiphy1
    echo ""
    test_at_resolution 1280 1024 rdi video0 msm_csid1 msm_csiphy1
    echo ""

    # Test VIDEO mode (video4) at both resolutions
    log_step "=== VIDEO Mode Tests (video4) ==="
    echo ""
    test_at_resolution 640 480 video video4 msm_csid1 msm_csiphy1
    echo ""
    test_at_resolution 1280 1024 video video4 msm_csid1 msm_csiphy1
    echo ""

    # Test TESTGEN mode (video3) at both resolutions
    log_step "=== TESTGEN Mode Tests (video3) ==="
    echo ""
    test_at_resolution 640 480 testgen video3 msm_csid1 msm_csiphy1
    echo ""
    test_at_resolution 1280 1024 testgen video3 msm_csid1 msm_csiphy1
    echo ""

    # Summary
    log_step "=== Test Summary ==="
    run_on_device "
        echo ''
        echo 'Captured files:'
        ls -la /tmp/test_*.raw 2>/dev/null || echo '  (none)'
        echo ''
        echo 'Results:'
        for f in /tmp/test_*.raw; do
            if [ -f \"\$f\" ]; then
                SIZE=\$(stat -c%s \"\$f\" 2>/dev/null || echo 0)
                NAME=\$(basename \"\$f\" .raw)
                if [ \"\$SIZE\" -gt 0 ]; then
                    echo \"  PASS: \$NAME (\$SIZE bytes)\"
                else
                    echo \"  FAIL: \$NAME (empty)\"
                fi
            fi
        done 2>/dev/null || echo '  No test files found'
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
            testgen)
                MODE="testgen"
                ;;
            legacy)
                MODE="legacy"
                ;;
            video)
                MODE="video"
                ;;
            video4)
                MODE="video4"
                ;;
            v4l2)
                MODE="v4l2"
                ;;
            nv16)
                MODE="nv16"
                ;;
            fetch)
                MODE="fetch"
                ;;
            capture-uyvy)
                MODE="capture-uyvy"
                ;;
            capture-nv16)
                MODE="capture-nv16"
                ;;
            all)
                MODE="all"
                ;;
            comprehensive|comp)
                MODE="comprehensive"
                ;;
            pix640)
                MODE="pix640"
                ;;
            pix1280)
                MODE="pix1280"
                ;;
            pix640-nv16)
                MODE="pix640-nv16"
                ;;
            pix1280-nv16)
                MODE="pix1280-nv16"
                ;;
            rdi640)
                MODE="rdi640"
                ;;
            rdi1280)
                MODE="rdi1280"
                ;;
            video640)
                MODE="video640"
                ;;
            video1280)
                MODE="video1280"
                ;;
            video640-nv16)
                MODE="video640-nv16"
                ;;
            video1280-nv16)
                MODE="video1280-nv16"
                ;;
            testgen640)
                MODE="testgen640"
                ;;
            testgen1280)
                MODE="testgen1280"
                ;;
            testgen640-nv16)
                MODE="testgen640-nv16"
                ;;
            testgen1280-nv16)
                MODE="testgen1280-nv16"
                ;;
            convert-video640)
                MODE="convert-video640"
                ;;
            convert-video1280)
                MODE="convert-video1280"
                ;;
            convert-pix640)
                MODE="convert-pix640"
                ;;
            convert-pix1280)
                MODE="convert-pix1280"
                ;;
            convert-rdi640)
                MODE="convert-rdi640"
                ;;
            convert-rdi1280)
                MODE="convert-rdi1280"
                ;;
            convert-testgen640)
                MODE="convert-testgen640"
                ;;
            convert-testgen1280)
                MODE="convert-testgen1280"
                ;;
            --help|-h)
                echo "Usage: $0 [MODE]"
                echo ""
                echo "Modes:"
                echo "  raw        Test RAW passthrough (CAMIF->memory via RDI, no ISP)"
                echo "  pix        Test PIX mode (through VFE ISP, uses GStreamer)"
                echo "  nv16       Test NV16 semi-planar format (native VFE31 output)"
                echo "  video      Test VIDEO mode (preview+video via WM4/WM5, AXI mode 0x01)"
                echo "  video4     Test VIDEO4 mode (WM4/WM5 capture, RECOMMENDED for full resolution)"
                echo "  v4l2       Test PIX mode with v4l2-ctl (direct, no GStreamer)"
                echo "  testgen    Test VFE internal test generator (bypasses camera)"
                echo "  legacy     Test with webOS-style legacy routing (MISC_CC=0)"
                echo "  capture-uyvy  Capture frame in UYVY format"
                echo "  capture-nv16  Capture frame in NV16 format (CbCr order)"
                echo "  capture-nv61  Capture frame in NV61 format (CrCb order, for color test)"
                echo "  fetch      Fetch all captures from device to current directory"
                echo "  convert-MODE-RES  Convert captured frames to timestamped PNGs"
                echo "              (e.g., convert-video640, convert-pix1280, convert-rdi640)"
                echo "  all        Run all test modes sequentially"
                echo "  comprehensive  Test ALL modes at 640x480 AND 1280x1024 (RECOMMENDED)"
                echo ""
                echo "Resolution-specific modes (NV12 - 4:2:0 default):"
                echo "  pix640     PIX mode at 640x480"
                echo "  pix1280    PIX mode at 1280x1024"
                echo "  rdi640     RDI mode at 640x480 (UYVY raw)"
                echo "  rdi1280    RDI mode at 1280x1024 (UYVY raw)"
                echo "  video640   VIDEO mode at 640x480"
                echo "  video1280  VIDEO mode at 1280x1024"
                echo "  testgen640   TESTGEN mode at 640x480"
                echo "  testgen1280  TESTGEN mode at 1280x1024"
                echo ""
                echo "NV16 (4:2:2) modes - full chroma height:"
                echo "  pix640-nv16     PIX mode at 640x480 (NV16)"
                echo "  pix1280-nv16    PIX mode at 1280x1024 (NV16)"
                echo "  video640-nv16   VIDEO mode at 640x480 (NV16)"
                echo "  video1280-nv16  VIDEO mode at 1280x1024 (NV16)"
                echo "  testgen640-nv16   TESTGEN mode at 640x480 (NV16)"
                echo "  testgen1280-nv16  TESTGEN mode at 1280x1024 (NV16)"
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
        v4l2)
            show_camera_info
            ensure_camera_ready
            test_v4l2_mode
            check_dmesg
            ;;
        nv16)
            show_camera_info
            ensure_camera_ready
            test_nv16_mode
            check_dmesg
            ;;
        testgen)
            show_camera_info
            test_testgen_mode
            ;;
        legacy)
            show_camera_info
            ensure_camera_ready
            test_legacy_mode
            check_dmesg
            ;;
        video)
            show_camera_info
            ensure_camera_ready
            test_video_mode
            check_dmesg
            ;;
        video4)
            show_camera_info
            ensure_camera_ready
            test_video4_mode
            check_dmesg
            ;;
        capture-uyvy)
            ensure_camera_ready
            capture_frames "pix" "UYVY" 1280 1024 1
            check_dmesg
            ;;
        capture-nv16)
            ensure_camera_ready
            capture_frames "pix" "NV16" 1280 1024 1
            check_dmesg
            ;;
        capture-nv61)
            # Test NV61 (CrCb order) to see if UV channels are swapped
            ensure_camera_ready
            capture_frames "pix" "NV61" 1280 1024 1
            check_dmesg
            ;;
        fetch)
            fetch_captures "."
            ;;
        convert-video640)
            convert_frames video 640 480
            ;;
        convert-video1280)
            convert_frames video 1280 1024
            ;;
        convert-pix640)
            convert_frames pix 640 480
            ;;
        convert-pix1280)
            convert_frames pix 1280 1024
            ;;
        convert-rdi640)
            convert_frames rdi 640 480
            ;;
        convert-rdi1280)
            convert_frames rdi 1280 1024
            ;;
        convert-testgen640)
            convert_frames testgen 640 480
            ;;
        convert-testgen1280)
            convert_frames testgen 1280 1024
            ;;
        comprehensive)
            show_camera_info
            ensure_camera_ready
            test_comprehensive
            check_dmesg
            ;;
        pix640)
            ensure_camera_ready
            test_at_resolution 640 480 pix video3 msm_csid1 msm_csiphy1 NV12
            check_dmesg
            ;;
        pix1280)
            ensure_camera_ready
            test_at_resolution 1280 1024 pix video3 msm_csid1 msm_csiphy1 NV12
            check_dmesg
            ;;
        pix640-nv16)
            ensure_camera_ready
            test_at_resolution 640 480 pix video3 msm_csid1 msm_csiphy1 NV16
            check_dmesg
            ;;
        pix1280-nv16)
            ensure_camera_ready
            test_at_resolution 1280 1024 pix video3 msm_csid1 msm_csiphy1 NV16
            check_dmesg
            ;;
        rdi640)
            ensure_camera_ready
            test_at_resolution 640 480 rdi video0 msm_csid1 msm_csiphy1 UYVY
            check_dmesg
            ;;
        rdi1280)
            ensure_camera_ready
            test_at_resolution 1280 1024 rdi video0 msm_csid1 msm_csiphy1 UYVY
            check_dmesg
            ;;
        video640)
            ensure_camera_ready
            test_at_resolution 640 480 video video4 msm_csid1 msm_csiphy1 NV12
            check_dmesg
            ;;
        video1280)
            ensure_camera_ready
            test_at_resolution 1280 1024 video video4 msm_csid1 msm_csiphy1 NV12
            check_dmesg
            ;;
        video640-nv16)
            ensure_camera_ready
            test_at_resolution 640 480 video video4 msm_csid1 msm_csiphy1 NV16
            check_dmesg
            ;;
        video1280-nv16)
            ensure_camera_ready
            test_at_resolution 1280 1024 video video4 msm_csid1 msm_csiphy1 NV16
            check_dmesg
            ;;
        testgen640)
            ensure_camera_ready
            test_at_resolution 640 480 testgen video3 msm_csid1 msm_csiphy1 NV12
            check_dmesg
            ;;
        testgen1280)
            ensure_camera_ready
            test_at_resolution 1280 1024 testgen video3 msm_csid1 msm_csiphy1 NV12
            check_dmesg
            ;;
        testgen640-nv16)
            ensure_camera_ready
            test_at_resolution 640 480 testgen video3 msm_csid1 msm_csiphy1 NV16
            check_dmesg
            ;;
        testgen1280-nv16)
            ensure_camera_ready
            test_at_resolution 1280 1024 testgen video3 msm_csid1 msm_csiphy1 NV16
            check_dmesg
            ;;
        all)
            show_camera_info
            ensure_camera_ready
            log_step "Running all test modes..."
            echo ""
            test_pix_mode
            echo ""
            test_nv16_mode
            echo ""
            test_video_mode
            echo ""
            test_video4_mode
            echo ""
            test_raw_mode
            echo ""
            check_dmesg
            log_step "Fetching all captures..."
            fetch_captures "."
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
