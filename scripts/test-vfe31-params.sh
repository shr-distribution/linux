#!/bin/bash
# VFE31 Parameter Testing Script
# Quickly test different VFE31 configurations

DEVICE="root@172.16.42.2"
SSH="ssh -p 22 $DEVICE"

FORMAT="nv12"  # Default format for analysis

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --ping-only N      Set vfe31_ping_only (0=off, 1=on, 2=static)"
    echo "  --single-buffer N  Set vfe31_single_buffer (0=off, 1=on)"
    echo "  --swap N           Set vfe31_swap_pingpong (0=off, 1=on)"
    echo "  --invert-pp N      Set vfe31_invert_pp (0=off, 1=on)"
    echo "  --y-height N       Set vfe31_pix_y_img_height"
    echo "  --y-lines N        Set vfe31_pix_y_lines"
    echo "  --y-ub-height N    Set vfe31_pix_y_ub_height"
    echo "  --stride N         Set vfe31_image_stride"
    echo "  --stride-fix N     Set vfe31_nv12_stride_fix (0=off, 1=on)"
    echo "  --reset            Reset all params to defaults"
    echo "  --show             Show current params"
    echo "  --test MODE        Run test (pix640, pix1280, video640, video1280)"
    echo "  --format FMT       Set format for analyze (nv12, nv16)"
    echo "  --analyze          Analyze last capture"
    echo ""
    echo "Examples:"
    echo "  $0 --reset --test pix640 --analyze"
    echo "  $0 --ping-only 1 --test pix640 --analyze"
    echo "  $0 --format nv16 --analyze"
}

set_param() {
    local param=$1
    local value=$2
    echo "Setting $param = $value"
    $SSH "echo $value > /sys/module/qcom_camss/parameters/$param"
}

reset_params() {
    echo "=== Resetting all VFE31 params to defaults ==="
    set_param vfe31_ping_only 0
    set_param vfe31_single_buffer 0
    set_param vfe31_swap_pingpong 0
    set_param vfe31_invert_pp 0
    set_param vfe31_nv12_stride_fix 1
    set_param vfe31_pix_y_img_height -1
    set_param vfe31_pix_y_lines -1
    set_param vfe31_pix_y_ub_height -1
    set_param vfe31_pix_cbcr_img_height -1
    set_param vfe31_pix_cbcr_lines -1
    set_param vfe31_image_stride 0
    set_param vfe31_bytesperline 0
}

show_params() {
    echo "=== Current VFE31 Parameters ==="
    $SSH "
        echo 'ping_only:' \$(cat /sys/module/qcom_camss/parameters/vfe31_ping_only)
        echo 'single_buffer:' \$(cat /sys/module/qcom_camss/parameters/vfe31_single_buffer)
        echo 'swap_pingpong:' \$(cat /sys/module/qcom_camss/parameters/vfe31_swap_pingpong)
        echo 'invert_pp:' \$(cat /sys/module/qcom_camss/parameters/vfe31_invert_pp)
        echo 'nv12_stride_fix:' \$(cat /sys/module/qcom_camss/parameters/vfe31_nv12_stride_fix)
        echo 'pix_y_img_height:' \$(cat /sys/module/qcom_camss/parameters/vfe31_pix_y_img_height)
        echo 'pix_y_lines:' \$(cat /sys/module/qcom_camss/parameters/vfe31_pix_y_lines)
        echo 'pix_y_ub_height:' \$(cat /sys/module/qcom_camss/parameters/vfe31_pix_y_ub_height)
        echo 'image_stride:' \$(cat /sys/module/qcom_camss/parameters/vfe31_image_stride)
    "
}

run_test() {
    local mode=$1
    echo "=== Running test: $mode ==="
    $SSH "dmesg -C && cd /home/root && ./test-camera.sh $mode 2>&1" | tail -10
}

analyze_capture() {
    echo "=== Analyzing capture ==="
    local file="/tmp/test_pix_640x480.raw"

    # Check frame data on device
    $SSH "
        FRAME_SIZE=460800
        FILE=$file
        if [ ! -f \$FILE ]; then
            echo 'No capture file found'
            exit 1
        fi
        FILE_SIZE=\$(stat -c%s \$FILE)
        NUM_FRAMES=\$((FILE_SIZE / FRAME_SIZE))
        echo 'Frames:' \$NUM_FRAMES

        echo ''
        echo '=== Frame data (first 8 bytes) ==='
        for i in \$(seq 0 \$((NUM_FRAMES - 1))); do
            OFFSET=\$((i * FRAME_SIZE))
            DATA=\$(dd if=\$FILE bs=1 skip=\$OFFSET count=8 2>/dev/null | od -An -tx1)
            if echo \"\$DATA\" | grep -q '00 00 00 00 00 00 00 00'; then
                echo \"Frame \$i: EMPTY\"
            else
                echo \"Frame \$i: \$DATA\"
            fi
        done

        echo ''
        echo '=== Y plane coverage (lines with data) ==='
        VALID_LINES=0
        for line in 0 60 120 180 239 240 300 360 420 479; do
            OFFSET=\$((line * 640))
            DATA=\$(dd if=\$FILE bs=1 skip=\$OFFSET count=4 2>/dev/null | od -An -tx1)
            if echo \"\$DATA\" | grep -qv '00 00 00 00'; then
                echo \"  Line \$line: data\"
                VALID_LINES=\$((VALID_LINES + 1))
            else
                echo \"  Line \$line: empty\"
            fi
        done

        echo ''
        Y_SIZE=307200
        if [ \"$FORMAT\" = \"nv16\" ]; then
            echo '=== CbCr plane coverage (NV16: full height, starts at Y_size=307200) ==='
            for line in 0 60 120 180 239 240 300 360 420 479; do
                OFFSET=\$((Y_SIZE + line * 640))
                DATA=\$(dd if=\$FILE bs=1 skip=\$OFFSET count=4 2>/dev/null | od -An -tx1)
                if echo \"\$DATA\" | grep -qv '00 00 00 00'; then
                    echo \"  CbCr line \$line: \$DATA\"
                else
                    echo \"  CbCr line \$line: empty\"
                fi
            done
        else
            echo '=== CbCr plane coverage (NV12: half height, starts at Y_size=307200) ==='
            for line in 0 60 120 180 239; do
                OFFSET=\$((Y_SIZE + line * 640))
                DATA=\$(dd if=\$FILE bs=1 skip=\$OFFSET count=4 2>/dev/null | od -An -tx1)
                if echo \"\$DATA\" | grep -qv '00 00 00 00'; then
                    echo \"  CbCr line \$line: \$DATA\"
                else
                    echo \"  CbCr line \$line: empty\"
                fi
            done
        fi
    "
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --ping-only)
            set_param vfe31_ping_only "$2"
            shift 2
            ;;
        --single-buffer)
            set_param vfe31_single_buffer "$2"
            shift 2
            ;;
        --swap)
            set_param vfe31_swap_pingpong "$2"
            shift 2
            ;;
        --invert-pp)
            set_param vfe31_invert_pp "$2"
            shift 2
            ;;
        --y-height)
            set_param vfe31_pix_y_img_height "$2"
            shift 2
            ;;
        --y-lines)
            set_param vfe31_pix_y_lines "$2"
            shift 2
            ;;
        --y-ub-height)
            set_param vfe31_pix_y_ub_height "$2"
            shift 2
            ;;
        --stride)
            set_param vfe31_image_stride "$2"
            shift 2
            ;;
        --stride-fix)
            set_param vfe31_nv12_stride_fix "$2"
            shift 2
            ;;
        --format)
            FORMAT="$2"
            shift 2
            ;;
        --reset)
            reset_params
            shift
            ;;
        --show)
            show_params
            shift
            ;;
        --test)
            run_test "$2"
            shift 2
            ;;
        --analyze)
            analyze_capture
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ $# -eq 0 ]] && [[ ! "$*" =~ "--" ]]; then
    usage
fi
