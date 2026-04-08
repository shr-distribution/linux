#!/bin/bash
#
# Camera Capture Analysis Script
# Analyzes raw captures from VFE31 VIDEO/PIX modes
#
# Usage: ./scripts/analyze-capture.sh <raw_file> <width> <height> [format] [prefix]
#
# Formats: NV16, NV12, NV12x2 (NV12 with stride_factor=2)
# Output naming: <prefix>_YYYYMMDD_HHMMSS_<type>.png

set -e

RAW_FILE="$1"
WIDTH="${2:-640}"
HEIGHT="${3:-480}"
FORMAT="${4:-NV16}"
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
    echo "  format:   NV16, NV12, NV12x2 (default: NV16)"
    echo "  prefix:   Output prefix (default: auto from filename)"
    echo ""
    echo "Formats:"
    echo "  NV16   - Y + UV at full height (2 bytes/pixel)"
    echo "  NV12   - Y + UV at half height (1.5 bytes/pixel)"
    echo "  NV12x2 - NV12 with VFE31 stride_factor=2 (3 bytes/pixel buffer)"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTDIR="captures"
mkdir -p "$OUTDIR"

# Calculate sizes based on format
Y_SIZE=$((WIDTH * HEIGHT))
case "$FORMAT" in
    NV16|nv16|UYVY|uyvy)
        FRAME_SIZE=$((WIDTH * HEIGHT * 2))
        UV_HEIGHT=$HEIGHT
        FORMAT_DESC="NV16/UYVY (2 bytes/pixel)"
        ;;
    NV12|nv12)
        FRAME_SIZE=$((WIDTH * HEIGHT * 3 / 2))
        UV_HEIGHT=$((HEIGHT / 2))
        FORMAT_DESC="NV12 (1.5 bytes/pixel)"
        ;;
    NV12x2|nv12x2|NV12X2)
        # VFE31 stride_factor=2: buffer is 2x NV12 size
        # Data layout: Y at stride=width*2, UV at stride=width*2
        FRAME_SIZE=$((WIDTH * HEIGHT * 3))
        UV_HEIGHT=$((HEIGHT / 2))
        FORMAT_DESC="NV12x2 (VFE31 doubled buffer, 3 bytes/pixel)"
        ;;
    *)
        echo "Unknown format: $FORMAT"
        exit 1
        ;;
esac

FILE_SIZE=$(stat -c%s "$RAW_FILE")
NUM_FRAMES=$((FILE_SIZE / FRAME_SIZE))

echo "=============================================="
echo "  Camera Capture Analysis"
echo "=============================================="
echo "Input:      $RAW_FILE"
echo "Size:       $FILE_SIZE bytes"
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "Format:     $FORMAT_DESC"
echo "Frame size: $FRAME_SIZE bytes"
echo "Frames:     $NUM_FRAMES"
echo "Output:     $OUTDIR/${PREFIX}_${TIMESTAMP}_*.png"
echo ""

if [ $NUM_FRAMES -eq 0 ]; then
    echo "ERROR: File too small for even one frame"
    exit 1
fi

# Extract individual frames
echo "=== Extracting frames ==="
for i in $(seq 1 $NUM_FRAMES); do
    OFFSET=$(( (i-1) * FRAME_SIZE ))
    tail -c +$((OFFSET + 1)) "$RAW_FILE" | head -c $FRAME_SIZE > "/tmp/frame_${i}.raw"
    echo "  Frame $i: extracted ($FRAME_SIZE bytes at offset $OFFSET)"
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

    case "$FORMAT" in
        NV16|nv16)
            # NV16 semi-planar
            ffmpeg -y -f rawvideo -pix_fmt nv16 -s ${WIDTH}x${HEIGHT} \
                   -i "$FRAME" "${OUT_BASE}_nv16.png" 2>/dev/null && \
                   echo "  Frame $i: NV16 -> ${OUT_BASE}_nv16.png"
            ;;
        NV12|nv12)
            # NV12 semi-planar
            ffmpeg -y -f rawvideo -pix_fmt nv12 -s ${WIDTH}x${HEIGHT} \
                   -i "$FRAME" "${OUT_BASE}_nv12.png" 2>/dev/null && \
                   echo "  Frame $i: NV12 -> ${OUT_BASE}_nv12.png"
            ;;
        NV12x2|nv12x2|NV12X2)
            # VFE31 NV12 with doubled stride - data has padding
            # Y plane: width*2 bytes per line, height lines
            # UV plane: width*2 bytes per line, height/2 lines
            INPUT_STRIDE=$((WIDTH * 2))

            # Extract Y plane (strided) and UV plane (strided)
            python3 << EOF
import sys
width = $WIDTH
height = $HEIGHT
input_stride = $INPUT_STRIDE  # VFE31 writes at UYVY stride
uv_height = height // 2

with open('/tmp/frame_${i}.raw', 'rb') as f:
    data = f.read()

# Y plane: extract width bytes from each input_stride line
y_data = bytearray()
for line in range(height):
    offset = line * input_stride
    y_data.extend(data[offset:offset + width])

# UV plane: starts after Y data in buffer
uv_start = height * input_stride
uv_data = bytearray()
for line in range(uv_height):
    offset = uv_start + line * input_stride
    uv_data.extend(data[offset:offset + width])

# Write de-strided NV12
with open('/tmp/frame_${i}_destrided.raw', 'wb') as f:
    f.write(y_data)
    f.write(uv_data)

print(f"  Frame $i: De-strided {len(data)} -> {len(y_data) + len(uv_data)} bytes")
EOF

            ffmpeg -y -f rawvideo -pix_fmt nv12 -s ${WIDTH}x${HEIGHT} \
                   -i "/tmp/frame_${i}_destrided.raw" "${OUT_BASE}_nv12.png" 2>/dev/null && \
                   echo "  Frame $i: NV12 (de-strided) -> ${OUT_BASE}_nv12.png"
            ;;
    esac

    # Y-only grayscale (always useful)
    if [ "$FORMAT" = "NV12x2" ] || [ "$FORMAT" = "nv12x2" ]; then
        # For NV12x2, Y is at doubled stride
        python3 << EOF
width = $WIDTH
height = $HEIGHT
input_stride = width * 2
with open('/tmp/frame_${i}.raw', 'rb') as f:
    data = f.read()
y_data = bytearray()
for line in range(height):
    offset = line * input_stride
    y_data.extend(data[offset:offset + width])
with open('/tmp/frame_${i}_y.raw', 'wb') as f:
    f.write(y_data)
EOF
    else
        head -c $Y_SIZE "/tmp/frame_${i}.raw" > "/tmp/frame_${i}_y.raw"
    fi

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
