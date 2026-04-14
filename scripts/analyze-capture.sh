#!/bin/bash
#
# Camera Capture Analysis Script
# Analyzes raw captures from VFE31 VIDEO/PIX modes
#
# Usage: ./scripts/analyze-capture.sh <raw_file> <width> <height> [format] [prefix]
#
# Formats: NV12, NV16
# Buffer handling: Auto-detects VFE31 stride_factor=2 buffers
# Output naming: <prefix>_YYYYMMDD_HHMMSS_<type>.png

set -e

RAW_FILE="$1"
WIDTH="${2:-640}"
HEIGHT="${3:-480}"
FORMAT="${4:-NV12}"
PREFIX="$5"

# Auto-detect prefix from filename if not provided
if [ -z "$PREFIX" ]; then
    PREFIX=$(basename "$RAW_FILE" .raw)
fi

if [ ! -f "$RAW_FILE" ]; then
    echo "Usage: $0 <raw_file> <width> <height> [format] [prefix]"
    echo "  raw_file: Path to raw capture file"
    echo "  width:    Frame width (default: 640)"
    echo "  height:   Frame height (default: 480)"
    echo "  format:   NV12, NV16 (default: NV12)"
    echo "  prefix:   Output prefix (default: auto from filename)"
    echo ""
    echo "Formats:"
    echo "  NV12   - Y + UV at half height (1.5 bytes/pixel, 4:2:0)"
    echo "  NV16   - Y + UV at full height (2 bytes/pixel, 4:2:2)"
    echo ""
    echo "Note: VFE31 stride_factor=2 buffers are auto-detected."
    echo "      Data is contiguous; buffer padding is ignored."
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTDIR="captures"
mkdir -p "$OUTDIR"

# Calculate sizes based on format
Y_SIZE=$((WIDTH * HEIGHT))
case "$FORMAT" in
    NV16|nv16)
        DATA_SIZE=$((WIDTH * HEIGHT * 2))
        UV_HEIGHT=$HEIGHT
        FORMAT_DESC="NV16 (2 bytes/pixel, 4:2:2)"
        FFMPEG_FMT="nv16"
        ;;
    NV12|nv12)
        DATA_SIZE=$((WIDTH * HEIGHT * 3 / 2))
        UV_HEIGHT=$((HEIGHT / 2))
        FORMAT_DESC="NV12 (1.5 bytes/pixel, 4:2:0)"
        FFMPEG_FMT="nv12"
        ;;
    *)
        echo "Unknown format: $FORMAT (use NV12 or NV16)"
        exit 1
        ;;
esac

FILE_SIZE=$(stat -c%s "$RAW_FILE")

# Auto-detect buffer size (VFE31 may use stride_factor=2)
# VFE31 stride_factor=2: stride = width * 2 = 1280 for 640 width
# Buffer sizes are power-of-2 aligned (1MB for 640x480, 4MB for 1280x1024)
#
# For NV12 640x480 with stride=1280:
#   Y plane: 1280 * 480 = 614400 bytes
#   CbCr: 1280 * 240 = 307200 bytes
#   Total strided data: 921600 bytes -> buffer rounds to 1MB (1048576)
#
# For NV16 640x480 with stride=1280:
#   Y plane: 1280 * 480 = 614400 bytes
#   CbCr: 1280 * 480 = 614400 bytes
#   Total strided data: 1228800 bytes -> buffer rounds to 2MB (2097152)

STRIDE=$((WIDTH * 2))
case "$FORMAT" in
    NV12|nv12)
        STRIDED_DATA=$((STRIDE * HEIGHT + STRIDE * HEIGHT / 2))  # Y + CbCr (half height)
        ;;
    NV16|nv16)
        STRIDED_DATA=$((STRIDE * HEIGHT * 2))  # Y + CbCr (full height)
        ;;
esac

# Round up to power of 2 for buffer size
if [ $STRIDED_DATA -le 1048576 ]; then
    BUFFER_STRIDE2=1048576   # 1MB
elif [ $STRIDED_DATA -le 2097152 ]; then
    BUFFER_STRIDE2=2097152   # 2MB
elif [ $STRIDED_DATA -le 4194304 ]; then
    BUFFER_STRIDE2=4194304   # 4MB
else
    BUFFER_STRIDE2=$((STRIDED_DATA + 1048575 & ~1048575))  # Round to 1MB
fi

# Try stride_factor=2 buffer first (more common with VFE31)
if [ $((FILE_SIZE % BUFFER_STRIDE2)) -eq 0 ]; then
    BUFFER_SIZE=$BUFFER_STRIDE2
    STRIDE_FACTOR=2
elif [ $((FILE_SIZE % DATA_SIZE)) -eq 0 ]; then
    BUFFER_SIZE=$DATA_SIZE
    STRIDE_FACTOR=1
else
    # Fallback: try to find a reasonable buffer size
    echo "WARNING: File size $FILE_SIZE not evenly divisible by expected frame/buffer size"
    echo "  Data size: $DATA_SIZE, Buffer (stride=2): $BUFFER_STRIDE2"
    BUFFER_SIZE=$DATA_SIZE
    STRIDE_FACTOR=1
fi

FRAME_SIZE=$BUFFER_SIZE
NUM_FRAMES=$((FILE_SIZE / BUFFER_SIZE))

echo "=============================================="
echo "  Camera Capture Analysis"
echo "=============================================="
echo "Input:      $RAW_FILE"
echo "Size:       $FILE_SIZE bytes"
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "Format:     $FORMAT_DESC"
echo "Data size:  $DATA_SIZE bytes/frame"
if [ $STRIDE_FACTOR -gt 1 ]; then
echo "Buffer:     $BUFFER_SIZE bytes (stride_factor=$STRIDE_FACTOR detected)"
fi
echo "Frames:     $NUM_FRAMES"
echo "Output:     $OUTDIR/${PREFIX}_${TIMESTAMP}_*.png"
echo ""

if [ $NUM_FRAMES -eq 0 ]; then
    echo "ERROR: File too small for even one frame"
    exit 1
fi

# Extract individual frames (with de-striding if stride_factor > 1)
echo "=== Extracting frames ==="
for i in $(seq 1 $NUM_FRAMES); do
    BUFFER_OFFSET=$(( (i-1) * BUFFER_SIZE ))

    if [ $STRIDE_FACTOR -gt 1 ]; then
        # Extract strided data and de-stride it
        # VFE31 stride_factor=2 means stride = width * 2
        STRIDE=$((WIDTH * STRIDE_FACTOR))

        echo "  Frame $i: de-striding from buffer at offset $BUFFER_OFFSET (stride=$STRIDE)"

        # Extract raw buffer for this frame
        tail -c +$((BUFFER_OFFSET + 1)) "$RAW_FILE" | head -c $BUFFER_SIZE > "/tmp/frame_${i}_strided.raw"

        # De-stride Y plane: extract WIDTH bytes from each STRIDE-byte row
        > "/tmp/frame_${i}_y.raw"
        for row in $(seq 0 $((HEIGHT - 1))); do
            ROW_OFFSET=$((row * STRIDE))
            dd if="/tmp/frame_${i}_strided.raw" bs=1 skip=$ROW_OFFSET count=$WIDTH 2>/dev/null >> "/tmp/frame_${i}_y.raw"
        done

        # De-stride CbCr plane: starts at HEIGHT * STRIDE
        CBCR_START=$((HEIGHT * STRIDE))
        > "/tmp/frame_${i}_cbcr.raw"
        for row in $(seq 0 $((UV_HEIGHT - 1))); do
            ROW_OFFSET=$((CBCR_START + row * STRIDE))
            dd if="/tmp/frame_${i}_strided.raw" bs=1 skip=$ROW_OFFSET count=$WIDTH 2>/dev/null >> "/tmp/frame_${i}_cbcr.raw"
        done

        # Combine Y and CbCr
        cat "/tmp/frame_${i}_y.raw" "/tmp/frame_${i}_cbcr.raw" > "/tmp/frame_${i}.raw"
        rm -f "/tmp/frame_${i}_strided.raw" "/tmp/frame_${i}_cbcr.raw"
    else
        # No striding - extract contiguous data
        tail -c +$((BUFFER_OFFSET + 1)) "$RAW_FILE" | head -c $DATA_SIZE > "/tmp/frame_${i}.raw"
        echo "  Frame $i: extracted $DATA_SIZE bytes at offset $BUFFER_OFFSET"
    fi
done

# Analyze first frame data pattern
echo ""
echo "=== Data pattern analysis (Frame 1) ==="
echo "First 32 bytes:"
xxd "/tmp/frame_1.raw" | head -2
echo ""
echo "At Y/UV boundary (offset $Y_SIZE):"
xxd -s $Y_SIZE "/tmp/frame_1.raw" | head -2

# Convert using different format interpretations
echo ""
echo "=== Converting frames ==="

for i in $(seq 1 $NUM_FRAMES); do
    FRAME="/tmp/frame_${i}.raw"
    OUT_BASE="${OUTDIR}/${PREFIX}_${TIMESTAMP}_f${i}"

    # Convert using detected format
    ffmpeg -y -f rawvideo -pix_fmt $FFMPEG_FMT -s ${WIDTH}x${HEIGHT} \
           -i "$FRAME" "${OUT_BASE}_${FORMAT,,}.png" 2>/dev/null && \
           echo "  Frame $i: ${FORMAT} -> ${OUT_BASE}_${FORMAT,,}.png"

    # Y-only grayscale (always useful for debugging)
    head -c $Y_SIZE "$FRAME" > "/tmp/frame_${i}_y.raw"
    ffmpeg -y -f rawvideo -pix_fmt gray -s ${WIDTH}x${HEIGHT} \
           -i "/tmp/frame_${i}_y.raw" "${OUT_BASE}_gray.png" 2>/dev/null && \
           echo "  Frame $i: Y-only gray -> ${OUT_BASE}_gray.png"
done

# Summary
echo ""
echo "=== Output files ==="
ls -la ${OUTDIR}/${PREFIX}_${TIMESTAMP}_*.png 2>/dev/null | awk '{print "  " $9 " (" $5 " bytes)"}'

echo ""
echo "=== Analysis complete ==="

# Cleanup
rm -f /tmp/frame_*.raw /tmp/combined_*.raw
