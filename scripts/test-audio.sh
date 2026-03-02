#!/bin/bash
#
# Audio Test Script for HP TouchPad
# Deploys firmware, tools, configures mixer, and tests playback
#
# Usage: ./scripts/test-audio.sh [--skip-transfer]
#
# Requirements:
# - Device accessible at 172.16.42.2 via USB network
# - HTTP server files in /tmp on host (or use --skip-transfer if already deployed)
#

set -e

DEVICE_IP="172.16.42.2"
SSH_PORT="22"
TIMEOUT=10

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

# Run command on device via SSH
run_on_device() {
    local cmd="$1"
    ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=$TIMEOUT -p $SSH_PORT root@$DEVICE_IP "$cmd" 2>/dev/null
}

# Transfer file to device via SCP
scp_to_device() {
    local src="$1"
    local dst="$2"
    scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -P $SSH_PORT "$src" root@$DEVICE_IP:"$dst" 2>/dev/null
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

# Prepare host files
prepare_host_files() {
    log_step "Preparing host files in /tmp..."

    # Q6 firmware
    if [ ! -f /tmp/q6-firmware.tar.gz ]; then
        log_info "Creating Q6 firmware tarball..."
        FIRMWARE_DIR="/home/herrie/webos/touchpad-kernel/doctor305/untouched-rootfs/lib/firmware"
        if [ -d "$FIRMWARE_DIR" ]; then
            tar czf /tmp/q6-firmware.tar.gz -C "$FIRMWARE_DIR" q6.mdt q6.b00 q6.b01 q6.b02 q6.b03 q6.b04 q6.b05 q6.b06 q6.b07 2>/dev/null || true
        fi
    fi

    # WM8958 firmware (if available)
    if [ ! -f /tmp/wm8958-firmware.tar.gz ]; then
        WM_DIR="/home/herrie/webos/touchpad-kernel/doctor305/untouched-rootfs/lib/firmware"
        if ls "$WM_DIR"/wm8958*.wfw >/dev/null 2>&1; then
            log_info "Creating WM8958 firmware tarball..."
            tar czf /tmp/wm8958-firmware.tar.gz -C "$WM_DIR" wm8958_mbc.wfw wm8958_mbc_vss.wfw wm8958_enh_eq.wfw 2>/dev/null || true
        fi
    fi

    # TinyALSA tools
    TINYALSA_DIR="/home/herrie/webos/touchpad-kernel/tinyalsa/utils"
    if [ -f "$TINYALSA_DIR/tinyplay" ]; then
        cp "$TINYALSA_DIR/tinyplay" "$TINYALSA_DIR/tinymix" "$TINYALSA_DIR/tinycap" "$TINYALSA_DIR/tinypcminfo" /tmp/ 2>/dev/null || true
    fi

    # Test audio files - use 48kHz versions (Q6 LPASS requires 48kHz)
    # Convert original files if 48kHz versions don't exist
    SOUNDS_DIR="/home/herrie/webos/touchpad-kernel/doctor305/untouched-rootfs/usr/palm/sounds"
    if [ ! -f /tmp/notification_48k.wav ] && [ -f "$SOUNDS_DIR/notification.wav" ]; then
        log_info "Converting notification.wav to 48kHz..."
        ffmpeg -y -i "$SOUNDS_DIR/notification.wav" -ar 48000 /tmp/notification_48k.wav 2>/dev/null || \
            sox "$SOUNDS_DIR/notification.wav" -r 48000 /tmp/notification_48k.wav 2>/dev/null || true
    fi
    if [ ! -f /tmp/phone_48k.wav ] && [ -f "$SOUNDS_DIR/phone.wav" ]; then
        log_info "Converting phone.wav to 48kHz..."
        ffmpeg -y -i "$SOUNDS_DIR/phone.wav" -ar 48000 /tmp/phone_48k.wav 2>/dev/null || \
            sox "$SOUNDS_DIR/phone.wav" -r 48000 /tmp/phone_48k.wav 2>/dev/null || true
    fi

    # List prepared files
    log_info "Files prepared:"
    ls -la /tmp/q6-firmware.tar.gz /tmp/tinyplay /tmp/tinymix /tmp/notification_48k.wav /tmp/phone_48k.wav 2>/dev/null || true
}

# Transfer files to device via SCP
transfer_files() {
    log_step "Transferring files to device via SCP..."

    # Transfer Q6 firmware
    log_info "Transferring Q6 firmware..."
    scp_to_device /tmp/q6-firmware.tar.gz /tmp/
    run_on_device "cd /tmp && tar xzf q6-firmware.tar.gz && ls q6.mdt"

    # Transfer WM8958 firmware (optional)
    if [ -f /tmp/wm8958-firmware.tar.gz ]; then
        log_info "Transferring WM8958 firmware..."
        scp_to_device /tmp/wm8958-firmware.tar.gz /tmp/
        run_on_device "cd /tmp && tar xzf wm8958-firmware.tar.gz 2>/dev/null" || true
    fi

    # Transfer TinyALSA tools
    log_info "Transferring tinyplay..."
    scp_to_device /tmp/tinyplay /tmp/

    log_info "Transferring tinymix..."
    scp_to_device /tmp/tinymix /tmp/

    log_info "Transferring notification_48k.wav..."
    scp_to_device /tmp/notification_48k.wav /tmp/

    log_info "Transferring phone_48k.wav (longer test file)..."
    scp_to_device /tmp/phone_48k.wav /tmp/

    # Make executables
    run_on_device "chmod +x /tmp/tinyplay /tmp/tinymix"

    # Verify
    log_info "Verifying transfers..."
    run_on_device "ls -la /tmp/tinyplay /tmp/tinymix /tmp/notification_48k.wav /tmp/phone_48k.wav /tmp/q6.mdt"
}

# Deploy firmware to /lib/firmware
deploy_firmware() {
    log_step "Deploying firmware to /lib/firmware..."

    run_on_device "mkdir -p /lib/firmware && cp /tmp/q6.* /lib/firmware/"

    # WM8958 firmware (optional)
    run_on_device "cp /tmp/wm8958*.wfw /lib/firmware/ 2>/dev/null || true"

    log_info "Firmware deployed:"
    run_on_device "ls /lib/firmware/"
}

# Start LPASS
start_lpass() {
    log_step "Starting LPASS (Q6 DSP)..."

    local state=$(run_on_device "cat /sys/class/remoteproc/remoteproc0/state")
    if echo "$state" | grep -q "running"; then
        log_info "LPASS already running"
        return 0
    fi

    run_on_device "echo start > /sys/class/remoteproc/remoteproc0/state"
    sleep 2

    state=$(run_on_device "cat /sys/class/remoteproc/remoteproc0/state")
    if echo "$state" | grep -q "running"; then
        log_result "LPASS started successfully"
    else
        log_error "LPASS failed to start: $state"
        return 1
    fi
}

# Check sound card
check_sound_card() {
    log_step "Checking sound card..."

    local cards=$(run_on_device "cat /proc/asound/cards")
    echo "$cards"

    if echo "$cards" | grep -q "HPTouchPad"; then
        log_result "Sound card detected: HP TouchPad"
    else
        log_error "Sound card not detected"
        return 1
    fi
}

# Configure mixer routing
configure_mixer() {
    log_step "Configuring audio mixer routing..."

    # Enable LPASS to codec routing (Q6 DSP -> Secondary MI2S -> Codec)
    # Note: DT uses SECONDARY_MI2S for the codec link, so we need SEC_MI2S_RX
    log_info "Enabling SEC_MI2S_RX Audio Mixer..."
    run_on_device "cd /tmp && ./tinymix set 'SEC_MI2S_RX Audio Mixer MultiMedia1' 1"

    # Enable DAC paths (AIF1 -> DAC1)
    log_info "Enabling DAC mixer paths..."
    run_on_device "cd /tmp && ./tinymix set 'DAC1L Mixer AIF1.1 Switch' 1"
    run_on_device "cd /tmp && ./tinymix set 'DAC1R Mixer AIF1.1 Switch' 1"
    run_on_device "cd /tmp && ./tinymix set 'DAC1 Switch' 1 1"  # Both channels!

    # Enable output mixers (DAC -> Output Mixer)
    log_info "Enabling output mixers..."
    run_on_device "cd /tmp && ./tinymix set 'Left Output Mixer DAC Switch' 1"
    run_on_device "cd /tmp && ./tinymix set 'Right Output Mixer DAC Switch' 1"

    # Enable SPKOUT paths (Output Mixer -> Speaker Mixer -> Speaker Driver -> SPKOUT)
    # NOTE: TouchPad speakers are connected to SPKOUT pins, NOT LINEOUT!
    # LINEOUT is not connected on this board per webOS board-tenderloin.c
    log_info "Enabling speaker mixer paths (SPKOUT)..."
    run_on_device "cd /tmp && ./tinymix set 'SPKL DAC1 Switch' 1"
    run_on_device "cd /tmp && ./tinymix set 'SPKR DAC1 Switch' 1"

    # Enable speaker output switches (critical for audio output!)
    log_info "Enabling speaker output switches..."
    run_on_device "cd /tmp && ./tinymix set 'SPKL Output Switch' 1"
    run_on_device "cd /tmp && ./tinymix set 'SPKR Output Switch' 1"

    # Set volumes to maximum
    log_info "Setting volumes..."
    run_on_device "cd /tmp && ./tinymix set 'Speaker Volume' 63 63"
    run_on_device "cd /tmp && ./tinymix set 'Speaker Boost Volume' 7 7"
    run_on_device "cd /tmp && ./tinymix set 'Speaker Mixer Volume' 3 3"
    run_on_device "cd /tmp && ./tinymix set 'Output Volume' 63 63"
    run_on_device "cd /tmp && ./tinymix set 'AIF1DAC1 Volume' 96 96"
    run_on_device "cd /tmp && ./tinymix set 'DAC1 Volume' 96 96"

    log_result "Mixer configured for SPKOUT path"
}

# Test audio playback
test_playback() {
    log_step "Testing audio playback..."

    # Use large buffers to avoid ALSA timeout (-p 16384 -n 8 = 128KB buffer)
    # Default buffers (~8KB) cause timeout before DSP processes data

    log_info "Playing notification_48k.wav (short, 48kHz)..."
    local result=$(run_on_device "cd /tmp && ./tinyplay notification_48k.wav -p 16384 -n 8 2>&1")
    echo "$result"

    if echo "$result" | grep -q "Played.*bytes"; then
        log_result "Notification playback completed"
    fi

    sleep 2

    log_info "Playing phone_48k.wav (long, 48kHz)..."
    result=$(run_on_device "cd /tmp && ./tinyplay phone_48k.wav -p 16384 -n 8 2>&1")
    echo "$result"

    if echo "$result" | grep -q "error\|Error\|cannot"; then
        log_error "Playback encountered errors"

        # Get dmesg errors
        log_info "Checking kernel log for errors..."
        run_on_device "dmesg | grep -i 'error\|asm\|afe' | tail -10"
        return 1
    else
        log_result "Phone playback completed"
    fi
}

# Check dmesg for ASM/AFE status
check_dmesg() {
    log_step "Checking kernel messages..."

    log_info "Legacy protocol status:"
    run_on_device "dmesg | grep -i 'legacy' | tail -5"

    log_info "Recent audio errors:"
    run_on_device "dmesg | grep -iE 'error.*0x|returned error|asm|afe' | tail -10"

    log_info "APR header debug (if available):"
    run_on_device "dmesg | grep -i 'src_port\|dest_port\|open_write' | tail -5"
}

# Main
main() {
    echo "=============================================="
    echo "  HP TouchPad Audio Test Script"
    echo "=============================================="
    echo ""

    SKIP_TRANSFER=0
    if [ "$1" = "--skip-transfer" ]; then
        SKIP_TRANSFER=1
        log_info "Skipping file transfer (using existing files)"
    fi

    check_device || exit 1

    if [ $SKIP_TRANSFER -eq 0 ]; then
        prepare_host_files
        transfer_files
    fi

    deploy_firmware
    start_lpass
    sleep 2
    check_sound_card || exit 1
    configure_mixer
    test_playback
    check_dmesg

    echo ""
    echo "=============================================="
    echo "  Test Complete"
    echo "=============================================="
}

main "$@"
