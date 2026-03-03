#!/bin/bash
#
# Audio Test Script for HP TouchPad
# Deploys firmware, tools, configures mixer, and tests playback
#
# Usage: ./scripts/test-audio.sh [--skip-transfer] [--dump-regs] [--dapm]
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

    # Use default buffer parameters - large buffers (-p 16384) fail hw_params constraints

    # Dump registers before playback
    dump_codec_registers "Codec registers BEFORE playback"

    # Check DAPM widget states before playback
    check_dapm_widgets

    log_info "Playing notification_48k.wav (short, 48kHz)..."
    local result=$(run_on_device "cd /tmp && ./tinyplay notification_48k.wav 2>&1")
    echo "$result"

    if echo "$result" | grep -q "Played.*bytes"; then
        log_result "Notification playback completed"
    fi

    sleep 1

    # Start register monitor before longer playback
    start_register_monitor

    sleep 1

    log_info "Playing phone_48k.wav (long, 48kHz)..."
    result=$(run_on_device "cd /tmp && ./tinyplay phone_48k.wav 2>&1")
    echo "$result"

    # Wait for monitor to finish collecting data
    sleep 2

    if echo "$result" | grep -q "error\|Error\|cannot"; then
        log_error "Playback encountered errors"

        # Get dmesg errors
        log_info "Checking kernel log for errors..."
        run_on_device "dmesg | grep -i 'error\|asm\|afe' | tail -10"
    else
        log_result "Phone playback completed"
    fi

    # Show register monitoring results
    show_register_monitor_results

    # Dump registers after playback
    dump_codec_registers "Codec registers AFTER playback"

    # Check DAPM widget states after playback
    check_dapm_widgets
}

# Dump key WM8958 codec registers
dump_codec_registers() {
    local label="${1:-Codec Register Dump}"
    log_step "$label"

    # Path to regmap debugfs
    local REGMAP="/sys/kernel/debug/regmap/wm8994/registers"

    run_on_device "cat <<'SCRIPT' > /tmp/dump_regs.sh
#!/bin/sh
REGMAP='/sys/kernel/debug/regmap/1-001a/registers'
if [ ! -f \"\$REGMAP\" ]; then
    echo 'Regmap not available'
    exit 1
fi

# Read all registers once
REGS=\$(cat \$REGMAP)

# Function to get register value
get_reg() {
    echo \"\$REGS\" | grep -i \"^\$1:\" | awk '{print \$2}'
}

echo '=== Power Management ==='
printf 'PM1 (0x01):     0x%s  (VMID, bias, outputs)\\n' \"\$(get_reg 1)\"
printf 'PM3 (0x03):     0x%s  (mixers, spk drivers)\\n' \"\$(get_reg 3)\"
printf 'PM5 (0x05):     0x%s  (DAC enables)\\n' \"\$(get_reg 5)\"

echo ''
echo '=== Output Path ==='
printf 'SPKOUT Mux (0x24):    0x%s  (speaker output select)\\n' \"\$(get_reg 24)\"
printf 'Out Mixer1 (0x2d):    0x%s  (left output mixer)\\n' \"\$(get_reg 2d)\"
printf 'Out Mixer2 (0x2e):    0x%s  (right output mixer)\\n' \"\$(get_reg 2e)\"
printf 'Spk Mixer (0x36):     0x%s  (DAC->speaker mixer)\\n' \"\$(get_reg 36)\"

echo ''
echo '=== DAC/AIF Path ==='
printf 'DAC1L Mixer (0x601):  0x%s  (AIF->DAC1L select)\\n' \"\$(get_reg 601)\"
printf 'DAC1R Mixer (0x602):  0x%s  (AIF->DAC1R select)\\n' \"\$(get_reg 602)\"

echo ''
echo '=== Clocking ==='
printf 'AIF1CLK (0x200):      0x%s  (AIF1 clock source)\\n' \"\$(get_reg 200)\"
printf 'AIF1 Rate (0x210):    0x%s  (sample rate)\\n' \"\$(get_reg 210)\"
printf 'FLL1 Ctrl1 (0x220):   0x%s  (FLL enable)\\n' \"\$(get_reg 220)\"
printf 'FLL1 Ctrl5 (0x224):   0x%s  (FLL source)\\n' \"\$(get_reg 224)\"

echo ''
echo '=== Status ==='
printf 'Int Status2 (0x731):  0x%s  (FLL lock: bit5=1 is locked)\\n' \"\$(get_reg 731)\"
printf 'GPIO1 (0x700):        0x%s  (ext amp ctrl: 0x41=enabled)\\n' \"\$(get_reg 700)\"

echo ''
echo '=== Volume (check unmuted) ==='
printf 'DAC1L Vol (0x610):    0x%s  (bit9=0 unmuted)\\n' \"\$(get_reg 610)\"
printf 'DAC1R Vol (0x611):    0x%s  (bit9=0 unmuted)\\n' \"\$(get_reg 611)\"
printf 'Speaker L (0x26):     0x%s  (bit6=mute)\\n' \"\$(get_reg 26)\"
printf 'Speaker R (0x27):     0x%s  (bit6=mute)\\n' \"\$(get_reg 27)\"
SCRIPT
chmod +x /tmp/dump_regs.sh
/tmp/dump_regs.sh"
}

# Monitor registers during playback (runs in background on device)
start_register_monitor() {
    log_info "Starting register monitor in background..."

    run_on_device "cat <<'SCRIPT' > /tmp/monitor_regs.sh
#!/bin/sh
REGMAP='/sys/kernel/debug/regmap/1-001a/registers'
OUTFILE='/tmp/reg_monitor.log'
echo 'Register Monitor Started' > \$OUTFILE
echo '========================' >> \$OUTFILE

# Monitor key registers every 0.5 seconds for 10 seconds
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
    REGS=\$(cat \$REGMAP 2>/dev/null)
    if [ -z \"\$REGS\" ]; then
        echo 'Regmap read failed' >> \$OUTFILE
        break
    fi

    # Extract key registers
    PM1=\$(echo \"\$REGS\" | grep -i '^1:' | awk '{print \$2}')
    PM3=\$(echo \"\$REGS\" | grep -i '^3:' | awk '{print \$2}')
    PM5=\$(echo \"\$REGS\" | grep -i '^5:' | awk '{print \$2}')
    FLL=\$(echo \"\$REGS\" | grep -i '^731:' | awk '{print \$2}')
    DAC1L=\$(echo \"\$REGS\" | grep -i '^601:' | awk '{print \$2}')
    GPIO=\$(echo \"\$REGS\" | grep -i '^700:' | awk '{print \$2}')
    SPKMIX=\$(echo \"\$REGS\" | grep -i '^36:' | awk '{print \$2}')

    echo \"[\$(date +%H:%M:%S.\$((i*50)))] PM1=\$PM1 PM3=\$PM3 PM5=\$PM5 FLL731=\$FLL DAC601=\$DAC1L SPKMIX=\$SPKMIX GPIO=\$GPIO\" >> \$OUTFILE
    sleep 0.5
done
echo 'Monitor Complete' >> \$OUTFILE
SCRIPT
chmod +x /tmp/monitor_regs.sh
nohup /tmp/monitor_regs.sh > /dev/null 2>&1 &
echo \$!"
}

# Show register monitor results
show_register_monitor_results() {
    log_step "Register monitor results:"
    run_on_device "cat /tmp/reg_monitor.log 2>/dev/null || echo 'No monitor log found'"
}

# Check DAPM widget states
check_dapm_widgets() {
    log_step "DAPM widget states..."

    run_on_device "cat <<'SCRIPT' > /tmp/check_dapm.sh
#!/bin/sh
DAPM_PATH='/sys/kernel/debug/asoc/HP-TouchPad/wm8994-codec/dapm'
if [ ! -d \"\$DAPM_PATH\" ]; then
    echo 'DAPM debugfs not available'
    exit 1
fi

echo '=== Clock Widgets ==='
for w in AIF1CLK CLK_SYS; do
    state=\$(cat \"\$DAPM_PATH/\$w\" 2>/dev/null | head -1)
    printf '%-20s: %s\\n' \"\$w\" \"\$state\"
done

echo ''
echo '=== AIF/DAC Path Widgets ==='
for w in 'AIF1DAC1L' 'AIF1DAC1R' 'DAC1L Mixer' 'DAC1R Mixer' 'DAC1L' 'DAC1R'; do
    state=\$(cat \"\$DAPM_PATH/\$w\" 2>/dev/null | head -1)
    printf '%-20s: %s\\n' \"\$w\" \"\$state\"
done

echo ''
echo '=== Output Mixer/PGA Widgets ==='
for w in 'Left Output Mixer' 'Right Output Mixer' 'Left Output PGA' 'Right Output PGA'; do
    state=\$(cat \"\$DAPM_PATH/\$w\" 2>/dev/null | head -1)
    printf '%-20s: %s\\n' \"\$w\" \"\$state\"
done

echo ''
echo '=== Speaker Widgets ==='
for w in 'SPKL' 'SPKR' 'SPKL Boost' 'SPKR Boost' 'SPKL Driver' 'SPKR Driver'; do
    state=\$(cat \"\$DAPM_PATH/\$w\" 2>/dev/null | head -1)
    printf '%-20s: %s\\n' \"\$w\" \"\$state\"
done
SCRIPT
chmod +x /tmp/check_dapm.sh
/tmp/check_dapm.sh"
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
    DUMP_REGS_ONLY=0
    DAPM_ONLY=0

    # Parse arguments
    for arg in "$@"; do
        case $arg in
            --skip-transfer)
                SKIP_TRANSFER=1
                log_info "Skipping file transfer (using existing files)"
                ;;
            --dump-regs)
                DUMP_REGS_ONLY=1
                log_info "Dump registers mode"
                ;;
            --dapm)
                DAPM_ONLY=1
                log_info "DAPM widgets mode"
                ;;
        esac
    done

    check_device || exit 1

    # Quick modes for debugging
    if [ $DUMP_REGS_ONLY -eq 1 ]; then
        dump_codec_registers "Current codec register state"
        exit 0
    fi

    if [ $DAPM_ONLY -eq 1 ]; then
        check_dapm_widgets
        exit 0
    fi

    # Full test flow
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
