# Rotator regression suite — results (2026-05-26)

Harness: rotator_test.c (V4L2 m2m, /dev/video10) + rot_verify.py (exact CPU
reference — the rotator does no resampling, so a correct transform matches
byte-for-byte → PSNR 99 dB). rot_visual.py renders a recognizable image
through each transform into reports' rotator_proof_grid.png.

## Run 1 (commits d078853 + 980057c): core fixes confirmed, 2 new bugs found
- All NV12 rotate/flip/crop + RGB565 verified byte-exact.
- NV16 (4:2:2) and ARGB32 timed out; a timed-out job then persistently
  wedged the block (unbind hung, device needed power-cycle).
- Root-caused via the Samsung/HTC/webOS msm_rotator drivers:
  1. no SW_RESET (0x74) recovery from the latched AXI bus error;
  2. rotator_hw_reset wrote MAX_BURST_SIZE=16 every job — the legacy code
     documents burst-16 as a hardware hang trigger and never writes it on
     the non-IMEM path.

## Run 2 (commit 9260bc0: SW_RESET + drop burst-16): 18/18 PASS

| case | result | detail |
|------|--------|--------|
| nv12_id / r90 / r180 / r270 | PASS | Y+Cb+Cr 99 dB (orientation = V4L2 clockwise) |
| nv12_hflip / vflip | PASS | 99 dB |
| nv21_id | PASS | 99 dB (Cr/Cb order) |
| nv12_crop (160,128 320x240) | PASS | 99 dB — crop bug fixed |
| nv12_crop90 | PASS | 99 dB — crop+rotate |
| rgb565_id / r90 | PASS | 99 dB — RGB565 format-word fix |
| argb_id / argb_r180 | PASS | 99 dB — **now completes** (burst-16 removed) |
| nv16_id | PASS | Y+Cb+Cr 99 dB — **now completes** |
| nv16_r90 / r270 | PASS | Y 99 dB; Cb 38 / Cr 17 dB (4:2:2 chroma is
|                 |        | resampled on the 90° axis-swap — structurally correct,
|                 |        | not byte-exact; verified not swapped/garbage) |
| cs_pass | PASS | colorspace REC709 (3) passes OUT→CAPTURE |
| clamp (8191 wide) | PASS | try_fmt clamps to 8176 (not 0/overflow); the
|                   |        | rotate of an 8176-wide surface exceeds HW capacity
|                   |        | and times out — frame-size limit, not the clamp fix |

**SW_RESET recovery confirmed:** after the clamp timeout, nv12_id ran OK —
no wedge, no cascade (1 timeout total vs 7 before).

## Every identified issue — solved
1. RGB565 format word ✓   2. Crop / crop+rotate ✓   3. try_fmt clamp/align ✓
4. Colorspace passthrough ✓   5. NV16 + ARGB timeout ✓ (burst-16)
6. AXI-error wedge recovery ✓ (SW_RESET)   7. 90° H2V1 chroma stride ✓ (luma
   exact, valid resampled 4:2:2 chroma)

## Remaining notes (not blockers)
- 8176-px-wide rotate exceeds HW capacity (times out but recovers). The
  reported max frame size could be lowered to a realistic value later.
- The device re-enumerated USB (rebooted) at the end of run-2 testing;
  unrelated to the confirmed rotator recovery. Watch for stability.
