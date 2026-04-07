#!/bin/bash
#
# Camera Capture Analysis Script
# Analyzes raw captures from VFE31 VIDEO/PIX modes
#
# Usage: ./scripts/analyze-capture.sh <raw_file> <width> <height> [output_prefix]
#
# Output naming: <prefix>_YYYYMMDD_HHMMSS_<type>.png

set -e

RAW_FILE="$1"
WIDTH="${2:-640}"
HEIGHT="${3:-480}"

# Auto-detect prefix from filename if not provided
# e.g., pix640_stridefix.raw -> pix640_stridefix
if [ -n "$4" ]; then
    PREFIX="$4"
else
    # Extract basename without extension
    PREFIX=$(basename "$RAW_FILE" .raw)
fi

if [ ! -f "$RAW_FILE" ]; then
    echo "Usage: $0 <raw_file> <width> <height> [output_prefix]"
    echo "  raw_file: Path to raw capture file"
    echo "  width:    Frame width (default: 640)"
    echo "  height:   Frame height (default: 480)"
    echo "  prefix:   Output prefix (default: auto from filename)"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTDIR="captures"
mkdir -p "$OUTDIR"

# Calculate sizes
FRAME_SIZE_NV16=$((WIDTH * HEIGHT * 2))   # NV16: Y + UV planes
FRAME_SIZE_UYVY=$((WIDTH * HEIGHT * 2))   # UYVY: packed
Y_SIZE=$((WIDTH * HEIGHT))
LINE_SIZE=$((WIDTH * 2))                  # UYVY line size

FILE_SIZE=$(stat -c%s "$RAW_FILE")
NUM_FRAMES=$((FILE_SIZE / FRAME_SIZE_NV16))

echo "=============================================="
echo "  Camera Capture Analysis"
echo "=============================================="
echo "Input:      $RAW_FILE"
echo "Size:       $FILE_SIZE bytes"
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "Frames:     $NUM_FRAMES (assuming NV16/UYVY)"
echo "Output:     $OUTDIR/${PREFIX}_${TIMESTAMP}_*.png"
echo ""

# Extract individual frames
echo "=== Extracting frames ==="
for i in $(seq 1 $NUM_FRAMES); do
    OFFSET=$(( (i-1) * FRAME_SIZE_NV16 ))
    tail -c +$((OFFSET + 1)) "$RAW_FILE" | head -c $FRAME_SIZE_NV16 > "/tmp/frame_${i}.raw"
    echo "  Frame $i: extracted"
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

    # Method 1: NV16 semi-planar (what we requested)
    ffmpeg -y -f rawvideo -pix_fmt nv16 -s ${WIDTH}x${HEIGHT} \
           -i "$FRAME" "${OUT_BASE}_nv16.png" 2>/dev/null && \
           echo "  Frame $i: NV16 -> ${OUT_BASE}_nv16.png"

    # Method 2: UYVY packed (what VFE might actually output)
    ffmpeg -y -f rawvideo -pix_fmt uyvy422 -s ${WIDTH}x${HEIGHT} \
           -i "$FRAME" "${OUT_BASE}_uyvy.png" 2>/dev/null && \
           echo "  Frame $i: UYVY -> ${OUT_BASE}_uyvy.png"

    # Method 3: Half-height UYVY (if only top half has data)
    HALF_SIZE=$((FRAME_SIZE_NV16 / 2))
    head -c $HALF_SIZE "$FRAME" > "/tmp/frame_${i}_half.raw"
    ffmpeg -y -f rawvideo -pix_fmt uyvy422 -s ${WIDTH}x$((HEIGHT/2)) \
           -i "/tmp/frame_${i}_half.raw" "${OUT_BASE}_uyvy_half.png" 2>/dev/null && \
           echo "  Frame $i: UYVY half-height -> ${OUT_BASE}_uyvy_half.png"

    # Method 4: Y-only grayscale
    head -c $Y_SIZE "$FRAME" > "/tmp/frame_${i}_y.raw"
    ffmpeg -y -f rawvideo -pix_fmt gray -s ${WIDTH}x${HEIGHT} \
           -i "/tmp/frame_${i}_y.raw" "${OUT_BASE}_gray.png" 2>/dev/null && \
           echo "  Frame $i: Y-only gray -> ${OUT_BASE}_gray.png"
done

# Try combining consecutive frames as interlaced fields
if [ $NUM_FRAMES -ge 2 ]; then
    echo ""
    echo "=== Combining frames as interlaced fields ==="

    python3 << EOF
import os

width = $WIDTH
height = $HEIGHT
line_size = width * 2  # UYVY
half_height = height // 2

# Combine F1 and F2 as interlaced (using half-height data from each)
with open('/tmp/frame_1_half.raw', 'rb') as f1, open('/tmp/frame_2_half.raw', 'rb') as f2:
    f1_data = f1.read()
    f2_data = f2.read()

    # F1 odd lines, F2 even lines
    with open('/tmp/combined_f1f2.raw', 'wb') as out:
        for i in range(half_height):
            out.write(f1_data[i*line_size:(i+1)*line_size])
            out.write(f2_data[i*line_size:(i+1)*line_size])

    # F2 odd lines, F1 even lines
    with open('/tmp/combined_f2f1.raw', 'wb') as out:
        for i in range(half_height):
            out.write(f2_data[i*line_size:(i+1)*line_size])
            out.write(f1_data[i*line_size:(i+1)*line_size])

print("  Combined files created")
EOF

    OUT_BASE="${OUTDIR}/${PREFIX}_${TIMESTAMP}"
    ffmpeg -y -f rawvideo -pix_fmt uyvy422 -s ${WIDTH}x${HEIGHT} \
           -i /tmp/combined_f1f2.raw "${OUT_BASE}_combined_f1f2.png" 2>/dev/null && \
           echo "  Combined F1+F2 -> ${OUT_BASE}_combined_f1f2.png"
    ffmpeg -y -f rawvideo -pix_fmt uyvy422 -s ${WIDTH}x${HEIGHT} \
           -i /tmp/combined_f2f1.raw "${OUT_BASE}_combined_f2f1.png" 2>/dev/null && \
           echo "  Combined F2+F1 -> ${OUT_BASE}_combined_f2f1.png"
fi

# Summary
echo ""
echo "=== Output files ==="
ls -la ${OUTDIR}/${PREFIX}_${TIMESTAMP}_*.png 2>/dev/null | awk '{print "  " $9 " (" $5 " bytes)"}'

echo ""
echo "=== Analysis complete ==="

# Cleanup
rm -f /tmp/frame_*.raw /tmp/combined_*.raw
