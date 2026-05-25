# VIDC encode/decode test harness

ARM V4L2 m2m clients used to validate the encoder/decoder driver additions
(see ../vidc-encoder-upstream-audit-2026-05-25.md).

## Build (host)
    arm-linux-gnueabihf-gcc -O2 -static -o vidc_encode vidc_encode.c
    arm-linux-gnueabihf-gcc -O2 -static -o vidc_capture vidc_capture.c

## Encoder (/dev/video7) — verifies keyframe flags, timestamp passthrough,
## GOP cadence, forced-IDR. Input = concatenated NV12MT 320x240 (147456 B/frame).
    ./vidc_encode in.tile out.h264 /dev/video7
    VIDC_GOP=15 ./vidc_encode flat45.tile g.h264 /dev/video7   # IDRs at 0,15,30
    VIDC_FORCEKEY=10 ./vidc_encode flat45.tile fk.h264 /dev/video7
    # env: VIDC_GOP, VIDC_BITRATE, VIDC_FORCEKEY(frame idx)

## Decoder (/dev/video6) — verifies MIN_BUFFERS_FOR_CAPTURE, G_SELECTION crop,
## B-frame reorder timestamp restore (DPB headroom + frame-tag table), EOS drain.
    ./vidc_capture in.264 out.tile /dev/video6

## Test clips (regenerate with ffmpeg/libx264 from any source):
    # B-frame (reorder + DPB headroom): Main profile, 2 B-frames
    ffmpeg -ss 60 -i src.mp4 -frames:v 40 -vf scale=320:240 -c:v libx264 \
      -profile:v main -x264-params bframes=2:keyint=15:scenecut=0 \
      -an -bsf:v h264_mp4toannexb -f h264 bframe.264
    # Cropped (G_SELECTION): 318x238 -> coded 320x240, 2px crop
    ffmpeg -ss 60 -i src.mp4 -frames:v 30 -vf scale=318:238 -c:v libx264 \
      -profile:v baseline -an -bsf:v h264_mp4toannexb -f h264 crop.264

## Known gap (as of g68599f53): EOS drain — DECODER_CMD STOP flushes held
## B-frames instead of draining; LAST/EOS not emitted. Needs VIDC_OP_LAST_FRAME
## pump (audit issue #6).

## Linear NV12 encoder input (VIDC_LINEAR=1)
The encoder now accepts linear V4L2_PIX_FMT_NV12 (no tile swizzle):
    VIDC_LINEAR=1 ./vidc_encode real_nv12.raw out.h264 /dev/video7
Generate linear NV12 from any image (Y plane then interleaved CbCr,
contiguous, 16-aligned stride): 320x240 = 115200 bytes/frame.
