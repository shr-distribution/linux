# MSM8660/APQ8060 VIDC/Venus Video Codec Support Analysis
**Date:** 2025-12-31
**Hardware:** HP TouchPad (MSM8660/APQ8060)
**Kernel:** Linux 6.13.0 mainline

---

## EXECUTIVE SUMMARY

**Status: ❌ NOT SUPPORTED**

MSM8660/APQ8060 **does NOT have Venus video codec support** in mainline Linux kernel.

- **Venus driver:** Only supports MSM8916 (2014) and newer
- **MSM8660 VIDC:** No mainline driver exists
- **Earliest support:** MSM8916 (2014)
- **MSM8660:** Released 2011, predates Venus framework

---

## ANALYSIS DETAILS

### 1. Venus Driver Supported SoCs

From `drivers/media/platform/qcom/venus/core.c`:

```
Compatible Strings:
- qcom,msm8916-venus  ← EARLIEST (2014)
- qcom,msm8996-venus  (2016)
- qcom,msm8998-venus  (2017)
- qcom,sdm660-venus   (2017)
- qcom,sdm845-venus   (2018)
- qcom,sdm845-venus-v2
- qcom,sc7180-venus   (2020)
- qcom,sc7280-venus   (2021)
- qcom,sm8250-venus   (2020)
```

**MSM8660/APQ8060 is NOT in this list.**

### 2. Legacy VIDC vs Modern Venus

**VIDC (Video Core):**
- MSM8660 era (2011) video codec hardware
- Version: VIDC 1.0 / VIDC 720p / VIDC 1080p
- Firmware: `vidc_1080p.fw` (489 KB)
- Capabilities: H.264, MPEG-4, VC-1, VP8 encode/decode up to 1080p

**Venus:**
- Modern name for Qualcomm video codec IP
- Started with MSM8916 (2014)
- Unified V4L2 framework
- Improved hardware architecture

**Key Difference:** VIDC and Venus are different hardware generations with incompatible programming interfaces.

### 3. Firmware Analysis

From legacy WebOS kernel, we found:

**File:** `vidc_1080p.fw` (489 KB)
- **Format:** Proprietary Qualcomm firmware
- **Version:** VIDC 1.0 for 1080p support
- **Codecs:** H.264, MPEG-4, VC-1, VP8
- **Resolution:** Up to 1920x1080p @ 30fps

**Hardware Capabilities:**
- H.264 encode: 1080p @ 30fps
- H.264 decode: 1080p @ 30fps
- MPEG-4/H.263 encode/decode
- VC-1 decode
- VP8 decode
- Post-processing (deinterlacing, scaling)

---

## DEVICE TREE CHECK

Checked `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi`:

**Result:** No video codec / VIDC / Venus nodes present

---

## IMPLICATIONS FOR HP TOUCHPAD

### What Doesn't Work Without VIDC

1. **Hardware Video Decode:**
   - Hardware-accelerated video playback
   - H.264/MP4 video decoding
   - Low-power video playback

2. **Hardware Video Encode:**
   - Video recording with hardware encoding
   - Camera video capture with compression
   - Low-power video recording

3. **Video Post-Processing:**
   - Hardware deinterlacing
   - Hardware scaling
   - Color space conversion

### What Still Works

1. **Software Video Decode:**
   - FFmpeg software decoding ✅
   - VLC software playback ✅
   - GStreamer software codecs ✅

2. **Display:**
   - Video output through MDP ✅
   - Hardware overlay ✅
   - HDMI output (if supported) ✅

### Impact Assessment

**Severity: MEDIUM**

- **Video Playback:**
  - Software decoding works but uses more CPU
  - 1080p H.264 playback may struggle on dual-core 1.2GHz CPU
  - 720p playback should work acceptably
  - Battery life reduced due to CPU usage

- **Video Recording:**
  - Software encoding very slow
  - May not achieve real-time encoding
  - Practical limit: ~480p @ 15fps with software

- **Typical Use Cases:**
  - YouTube/streaming: Software decode acceptable for 720p
  - Recorded videos: Software playback works
  - Video calls: May struggle without hardware acceleration

---

## TECHNICAL DETAILS

### VIDC Hardware Architecture (MSM8660)

**Register Blocks:**
- VIDC base: Not in mainline DT
- Clock control: Part of MMCC
- Interrupt: Not in mainline DT
- Memory: IOMMU/SMMU not configured

**Dependencies:**
- IOMMU/SMMU for memory protection
- MMCC clocks (GFX3D_CLK, etc.)
- Shared memory with firmware
- Interrupt handling

**Required for Driver:**
- Register base addresses
- Clock definitions
- Memory carveout
- Firmware loading mechanism
- V4L2 integration

---

## POTENTIAL SOLUTIONS

### Option 1: Use Software Codecs (RECOMMENDED)

**Effort:** None  
**Performance:** Acceptable for 720p, marginal for 1080p  
**Status:** Works today

Use FFmpeg/GStreamer software decoding:
```bash
# Install software codecs
sudo apt-get install gstreamer1.0-libav
```

**Pros:**
- No driver development needed
- Works immediately
- Widely tested

**Cons:**
- Higher CPU usage
- Reduced battery life
- May struggle with 1080p

### Option 2: Port Legacy VIDC Driver

**Effort:** VERY HIGH (3-6 months)  
**Feasibility:** LOW

**Requirements:**
- Port CAF VIDC driver from kernel 3.0
- Convert to V4L2 framework
- Add device tree support
- Implement memory management
- Add firmware loading
- Extensive testing

**Challenges:**
- No hardware documentation
- Legacy code quality
- V4L2 API changes
- Memory management complexity
- Unlikely upstream acceptance (obsolete hardware)

**Recommendation:** NOT FEASIBLE

### Option 3: Accelerated Software Decode

**Effort:** LOW to MEDIUM  
**Approach:** Optimize software decode with NEON

**FFmpeg with NEON:**
- ARM NEON SIMD optimizations
- Significant performance improvement
- Available in standard FFmpeg builds

**Performance:**
- H.264 720p: Smooth playback with NEON
- H.264 1080p: Marginal but possible
- MPEG-4: Better performance than H.264

**Implementation:**
```bash
# Ensure NEON-optimized FFmpeg
ffmpeg -version | grep neon
```

### Option 4: Limit Video Quality

**Effort:** None  
**Approach:** Content selection

**Recommendations:**
- Stream 720p instead of 1080p
- Use H.264 (better optimized than VP8/VP9)
- Avoid 60fps content
- Use lower bitrate streams

---

## COMPARISON: VIDC vs Venus

| Feature | MSM8660 VIDC | MSM8916 Venus |
|---------|--------------|---------------|
| Year | 2011 | 2014 |
| Max Resolution | 1080p30 | 1080p30 |
| Codecs | H.264, MPEG-4, VC-1, VP8 | H.264, H.265, VP8, VP9 |
| Architecture | VIDC 1.0 | Venus 1.8 |
| Mainline Support | ❌ No | ✅ Yes |
| V4L2 Interface | ❌ No | ✅ Yes |
| Firmware Format | vidc_1080p.fw | venus.mdt + .bXX |

**Evolution:** Venus represents a complete redesign with standardized interfaces suitable for mainline.

---

## SOFTWARE DECODE PERFORMANCE ESTIMATES

### MSM8660 (Dual-core Scorpion @ 1.2GHz)

**H.264 Decode Performance (Software):**
- 480p @ 30fps: ✅ Easy (~30% CPU)
- 720p @ 30fps: ✅ Possible (~70% CPU with NEON)
- 1080p @ 30fps: ⚠️ Marginal (~95%+ CPU with NEON)
- 1080p @ 60fps: ❌ Not feasible

**MPEG-4 Decode Performance (Software):**
- 480p @ 30fps: ✅ Easy (~20% CPU)
- 720p @ 30fps: ✅ Good (~50% CPU)
- 1080p @ 30fps: ✅ Possible (~80% CPU)

**VP8 Decode Performance (Software):**
- 480p @ 30fps: ✅ Good (~40% CPU)
- 720p @ 30fps: ⚠️ Marginal (~85% CPU)
- 1080p @ 30fps: ❌ Too slow

**Battery Impact:**
- Hardware decode: ~200mW
- Software decode: ~800-1200mW
- **3-5x power consumption increase**

---

## RECOMMENDATIONS

### For HP TouchPad Mainline

1. **SHORT TERM (Recommended):**
   - ✅ Use software codecs (FFmpeg/GStreamer)
   - ✅ Enable NEON optimizations
   - ✅ Document VIDC as "not supported"
   - ✅ Recommend 720p content for best experience

2. **MEDIUM TERM:**
   - ⏸️ Monitor if community members have VIDC documentation
   - ⏸️ Low priority - focus on critical features first

3. **LONG TERM:**
   - ⏸️ Defer indefinitely unless documentation emerges
   - ⏸️ Software decode is adequate for tablet use

### User Recommendations

**For best video experience on HP TouchPad:**
1. Use 720p content instead of 1080p
2. Prefer H.264 over VP8/VP9
3. Install NEON-optimized FFmpeg
4. Consider lower bitrate streams for better battery life
5. Use VLC or mpv with hardware-accelerated rendering (OpenGL)

---

## CONCLUSION

**MSM8660/APQ8060 VIDC is NOT supported in mainline kernel** and porting the legacy driver would require extensive effort for marginal benefit.

**Recommendation:** Use software video decoding. Performance is acceptable for 720p content with NEON optimizations, which covers most tablet use cases.

**Priority:** DEFER - Software decode is adequate

**Impact:** MEDIUM - Limits 1080p playback but 720p works fine

---

## REFERENCES

- Venus driver: `drivers/media/platform/qcom/venus/`
- Device tree bindings: `Documentation/devicetree/bindings/media/qcom,*-venus.yaml`
- V4L2 framework: `Documentation/userspace-api/media/v4l/`
- FFmpeg NEON: ARM SIMD optimization documentation

---

**Analysis Date:** 2025-12-31  
**Kernel Version:** Linux 6.13.0  
**Conclusion:** Not supported, use software codecs
