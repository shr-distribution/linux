# HP TouchPad Display Artifacts: Alpha Blending Analysis Report

**Date:** February 6, 2026
**Status:** Root Cause Found - Freedreno A2XX Driver Bug
**Hardware:** HP TouchPad (Topaz), Qualcomm APQ8060, Adreno 220 GPU

---

## Executive Summary

Visual display artifacts affecting semi-transparent UI elements (StatusBar, LaunchBar, buttons) on the HP TouchPad running LuneOS with Qt6 have been traced to **bugs in the Mesa freedreno A2XX driver**, NOT the kernel or Qt specifically.

**Key Finding:** Testing with glmark2 (bypassing Qt entirely) reveals the same rendering issues:
- **Color channel swap**: Cat model shows green instead of blue
- **Tessellation artifacts**: Smooth surfaces show triangular facets
- Some tests work correctly (linaro, linaro+tux)

This confirms the issue is in the **base freedreno A2XX driver**, not Qt6's SPIRV-Cross translation.

---

## glmark2 Test Results (Feb 6, 2026)

```
=======================================================
    glmark2 2023.01
=======================================================
    OpenGL Information
    GL_VENDOR:      freedreno
    GL_RENDERER:    FD220
    GL_VERSION:     OpenGL ES 2.0 Mesa 24.0.7
    Surface Config: buf=32 r=8 g=8 b=8 a=8 depth=24 stencil=0 samples=0
    Surface Size:   1024x768 fullscreen
=======================================================
```

| Test | Result |
|------|--------|
| Linaro logo | **CORRECT** - Colors and rendering OK |
| Linaro + Tux windows | **CORRECT** - No visible issues |
| Cat model (smooth) | **BROKEN** - Shows triangular facets |
| Cat model (color) | **BROKEN** - Green instead of blue |
| Build benchmark | 20 FPS |

### Root Cause: Freedreno A2XX Driver Bugs

1. **Color Channel Swap (BGR vs RGB)**
   - Blue objects render as green
   - Suggests texture format swizzle issue in `fd2_util.c`
   - `util_format_compose_swizzles()` may not handle all formats correctly

2. **Smooth Shading / Interpolation**
   - Smooth surfaces show triangular tessellation
   - Possible varying interpolation issue
   - May be shader precision or NIR lowering problem

---

## Symptoms

| Element | Issue |
|---------|-------|
| StatusBar background | Wrong color "most of the time" |
| LaunchBar background | Wrong color (uses gradient with opacity 0.2) |
| "Continue" button (FirstUse) | Wrong color |

**Pattern:** Affects elements with alpha/opacity. Appears "z-index related with wrong colors." Intermittent but frequent.

---

## Architecture Analysis

### Display Pipeline

```
Qt Scene Graph (CPU)
    ↓
OpenGL ES 2.0 rendering (Adreno 220 GPU)
    ↓
Single framebuffer (ARGB8888)
    ↓
MDP4 Display Controller (1 plane only)
    ↓
LCDC → Panel
```

### Two Separate Blending Systems

| System | Location | MOD_ALPHA Support | Role |
|--------|----------|-------------------|------|
| GPU Blending | Mesa freedreno A2XX | **NO** | Qt compositing |
| Display Blending | Kernel MDP4 | YES | Multi-plane overlay |

**Critical:** With only 1 DRM plane active, MDP4 display blending is NOT involved. All compositing happens in GPU rendering.

---

## Freedreno A2XX Blend Analysis

### Hardware Capabilities

| Feature | Adreno 220 (A2XX) | Later Adreno |
|---------|-------------------|--------------|
| Premultiplied alpha compensation | **NO** | No (software) |
| MOD_ALPHA equivalent | **NO** | No |
| sRGB blending | **NO** | Yes (A4XX+) |
| Advanced blend modes | **NO** | Yes (A5XX+) |
| Dual-source blending | **NO** | Yes (A3XX+) |

### Blend Implementation

**File:** `mesa/src/gallium/drivers/freedreno/a2xx/fd2_blend.c`

```c
so->rb_blendcontrol =
    A2XX_RB_BLEND_CONTROL_COLOR_SRCBLEND(fd_blend_factor(rt->rgb_src_factor)) |
    A2XX_RB_BLEND_CONTROL_COLOR_COMB_FCN(blend_func(rt->rgb_func)) |
    A2XX_RB_BLEND_CONTROL_COLOR_DESTBLEND(fd_blend_factor(rt->rgb_dst_factor));
```

The A2XX applies standard OpenGL blend equations:
```
result = src_color × src_factor + dst_color × dst_factor
```

### Premultiplied Alpha Problem

Qt uses `Format_ARGB32_Premultiplied` with typical blend factors:
- Source: `GL_ONE`
- Destination: `GL_ONE_MINUS_SRC_ALPHA`

**Correct (premultiplied input):**
```
Input:  (R×α, G×α, B×α, α)
Blend:  (R×α)×1 + bg×(1-α) = R×α + bg×(1-α)
Result: Correct
```

**Double-alpha bug (if alpha applied twice):**
```
Input:  (R×α, G×α, B×α, α)
Blend:  (R×α)×α + bg×(1-α) = R×α² + bg×(1-α)
Result: Colors too dark
```

**Wrong blend factors:**
```
Input:  (R×α, G×α, B×α, α)
Blend:  (R×α)×(1-α) + bg×α
Result: Inverted alpha - wrong z-order appearance
```

---

## Kernel Investigation Summary

### Fixes Applied (Working)

| Commit | Fix | Status |
|--------|-----|--------|
| `64a226329f6f` | Touch: single-touch axis emulation | Working |
| `c16804d0b02d` | Touch: input_mt_sync_frame | Working |
| `a6146036f983` | ADM DMA: flush-only not errors | Working |
| `3433f28fe0d1` | GPUMMU: wait for GPU idle before unmap | Reduces page faults |

### Fixes Applied (No Effect on Artifacts)

| Commit | Change | Why No Effect |
|--------|--------|---------------|
| `3ba8aba486bc` | CO3=0 for blend stage | Only affects display blending (not used) |
| `44e3e283dd0b` | Cache sync for display | Display path working correctly |
| `d42bbc5ed593` | Remove DEFLKR_EN | Only for interlaced displays |

### MOD_ALPHA Attempt (Reverted)

| Commit | Change | Result |
|--------|--------|--------|
| `bae6adab5d2b` | Add MOD_ALPHA to MDP4 | Complete display corruption |
| `1a74613d5a50` | Revert MOD_ALPHA | Restored normal display |

**Why it failed:** MOD_ALPHA affects MDP4 display-level blending of multiple planes. With only 1 plane active, it has no effect on Qt's GPU-composited content.

---

## Root Cause Hypothesis

The symptoms ("z-index related with wrong colors" on semi-transparent elements) suggest one of:

1. **Qt Scene Graph batch ordering issue**
   - Transparent items must render back-to-front
   - Batching may break this order on A2XX

2. **Blend state configuration error**
   - Wrong blend factors being set
   - Premultiplied vs non-premultiplied mismatch

3. **Texture atlas issues**
   - Small atlas (512×512) causing texture coordinate problems
   - Elements sharing atlas pages may interfere

4. **Qt6 + GLES 2.0 compatibility**
   - Qt6 may assume features A2XX doesn't have
   - Fallback code paths may have bugs

---

## Recommended Debugging Steps

### 1. Qt Scene Graph Debug

```bash
# Show render order and batching
export QSG_RENDERER_DEBUG=render,batches
export QSG_INFO=1

# Visualize overdraw (helps identify z-order issues)
export QSG_VISUALIZE=overdraw
```

### 2. Disable Texture Atlas

```bash
# Eliminate atlas as variable
export QSG_ATLAS_WIDTH=0
export QSG_ATLAS_HEIGHT=0
```

### 3. Force Software Rendering

```bash
# Confirm issue is GPU-specific
export QT_QUICK_BACKEND=software
```

### 4. Mesa Debug

```bash
export MESA_DEBUG=1
export LIBGL_DEBUG=verbose
export FD_MESA_DEBUG=msgs
```

### 5. Compare With Working System

If a similar device works correctly with Qt6, capture:
- `QSG_RENDERER_DEBUG=render,batches` output
- Mesa blend state logs
- Compare batch ordering and blend factors

---

## Files Reference

### Kernel (Reference Only - Not the Issue)

| File | Purpose |
|------|---------|
| `drivers/gpu/drm/msm/disp/mdp4/mdp4_crtc.c` | Display blend setup |
| `drivers/gpu/drm/msm/disp/mdp4/mdp4_plane.c` | Plane configuration |
| `drivers/gpu/drm/msm/adreno/a2xx_gpummu.c` | GPU MMU |

### Mesa Freedreno (Investigation Target)

| File | Purpose |
|------|---------|
| `src/gallium/drivers/freedreno/a2xx/fd2_blend.c` | Blend state setup |
| `src/gallium/drivers/freedreno/a2xx/fd2_texture.c` | Texture/alpha handling |
| `src/gallium/drivers/freedreno/a2xx/fd2_emit.c` | State emission |
| `src/gallium/drivers/freedreno/freedreno_util.c` | Blend factor mapping |

### Qt/Wayland (Investigation Target)

| File | Purpose |
|------|---------|
| `qtdeclarative/src/quick/scenegraph/` | Scene Graph rendering |
| `luna-surfacemanager/modules/weboscompositor/` | Wayland compositor |
| `luna-next-cardshell/qml/StatusBar/` | Affected UI |
| `luna-next-cardshell/qml/LaunchBar/` | Affected UI |

---

---

## KGSL vs Freedreno Shader Differences

**Critical Context:** Qt6 worked previously with the proprietary KGSL driver but has issues with freedreno Mesa.

### Kernel Driver Comparison

| Aspect | KGSL (Proprietary) | Freedreno/MSM (Mainline) |
|--------|-------------------|--------------------------|
| Shader compilation | Proprietary compiler, pre-compiled binaries | Mesa NIR/IR2 runtime compilation |
| Shader state preservation | Explicit 6KB+6KB+6KB shadow memory per context | No shader state shadow, userspace managed |
| Context switching | Saves/restores shader instruction memory | Relies on userspace to manage state |
| Precision handling | Built-in mediump/highp per operation | NIR lowering passes |
| GMEM operations | Hardcoded binary shader programs | Dynamically generated |

### KGSL Shader Memory Layout

```
18K Shader Instruction Shadow:
  - 6K vertex  (32-byte aligned)
  - 6K pixel   (32-byte aligned)
  - 6K shared  (32-byte aligned)
+ 8K ALU constants shadow
+ 4K Register shadow
+ 768 bytes Texture constants
```

**Freedreno has NO equivalent shader instruction caching/shadowing.**

### Pre-compiled Shaders in KGSL

KGSL embeds binary GPU shader programs for GMEM operations:
- `gmem2sys_vtx_pgm` (18 DWORDS) - GMEM to system vertex shader
- `gmem2sys_frag_pgm` (12 DWORDS) - GMEM to system fragment shader
- `sys2gmem_vtx_pgm` (24 DWORDS) - System to GMEM vertex shader
- `sys2gmem_frag_pgm` (15 DWORDS) - System to GMEM fragment shader

These are **binary GPU instructions**, not GLSL. Mesa/freedreno generates these dynamically.

### Adreno 220 (Leia) Specific Workarounds

KGSL has Leia-specific settings:
- `PM_OVERRIDE2 = 0x1a0` for clock gating
- `CONFIG1 = 0x00032f07` for 1K boundary check
- Different handling vs Yamato (A200)

**Freedreno may be missing some of these Leia-specific optimizations.**

---

## Qt5 vs Qt6 Shader Handling

### The Critical Difference

| Aspect | Qt5.15 | Qt6.8 |
|--------|--------|-------|
| Shader source | **Native GLSL ES 100 strings** in QML | Vulkan-style GLSL → SPIR-V → translated |
| Compilation | Runtime by GLES driver | Offline via `qsb` tool |
| Blend control | **Full glBlendFunc/glBlendEquation** | **Limited - equation always GL_FUNC_ADD** |
| Uniform handling | Direct uniform setting | **std140 uniform blocks required** |
| OpenGL access | Direct API calls in updateState() | No direct graphics API access |

### Qt5 Native Shader Example

```qml
// Qt5 - Direct GLSL ES 100
ShaderEffect {
    vertexShader: "
        uniform highp mat4 qt_Matrix;
        attribute highp vec4 qt_Vertex;
        void main() {
            gl_Position = qt_Matrix * qt_Vertex;
        }"
}
```

### Qt6 Shader Pipeline

```
Vulkan-style GLSL (v440)
    ↓ glslang
SPIR-V bytecode
    ↓ SPIRV-Cross
Multiple targets:
  - GLSL ES 100 (GLES 2.0)
  - GLSL 120 (OpenGL 2.1)
  - GLSL 150 (OpenGL 3.2)
  - HLSL 5.0, Metal 1.2
    ↓
Packaged in .qsb file
```

### SPIRV-Cross Translation Issues

When SPIR-V is translated to GLSL ES 100:
- Precision qualifiers may be handled differently
- Uniform block layout may not map cleanly
- Some constructs may not translate optimally
- **The generated code is NOT the same as hand-written GLSL ES 100**

### Qt6 Blend Equation Restriction

**Critical:** Qt6 restricts blend equations to `GL_FUNC_ADD` only.

```cpp
// Qt6 - Only blend factors can be customized
// Blend equation is ALWAYS ADD
updateGraphicsPipelineState() {
    // NO access to glBlendEquation()
}
```

If Qt5 code relied on different blend equations, it will break in Qt6.

---

## Freedreno A2XX Shader Limitations

### GLSL Version

**OpenGL ES 2.0 / GLSL ES 100 only**

### Hardware Limits

| Resource | Limit |
|----------|-------|
| Registers | 64 total (4 components each = 256 slots) |
| Instructions | 384 scheduled max |
| Loop unroll | 32 iterations max |
| Fetch instructions | 64 max |

### Unsupported Features

| Feature | Status | Notes |
|---------|--------|-------|
| Depth writing | **NOT SUPPORTED** | Shaders writing FRAG_RESULT_DEPTH rejected |
| Integer operations | **NOT SUPPORTED** | All ints lowered to floats |
| Boolean operations | **NOT SUPPORTED** | Lowered to floats |
| Flat shading | **PROBLEMATIC** | ES 2.0 limitation |
| Texture projection | LOWERED | All txp operations lowered |
| 1D/3D textures | **NOT SUPPORTED** | Causes compile error |
| Indirect array access | UNROLLED | Force-unrolled, max 32 iterations |

### NIR Lowering Passes

```c
// From ir2_nir.c
.no_integers = true              // NO integer support
.lower_fpow = true               // pow() lowered
.lower_flrp32 = true             // lerp() lowered
.lower_fmod = true               // fmod() lowered
.lower_fdiv = true               // Division lowered
.lower_fceil = true              // ceil() lowered
.lower_bitops = true             // Bitwise ops lowered
.lower_vector_cmp = true         // Vector comparisons lowered
.force_indirect_unrolling = all  // ALL indirect access unrolled
```

### Shader Compilation Failure Modes

If shader exceeds 64 registers:
```c
assert(idx != 64); /* TODO ran out of register space.. */
// HARD FAILURE - no fallback
```

---

## Root Cause Analysis: Why KGSL Worked But Freedreno Doesn't

### Most Likely Causes

1. **SPIRV-Cross Translation Artifacts**
   - Qt5 used native GLSL ES 100 compiled directly by KGSL
   - Qt6 uses SPIRV-Cross to generate GLSL ES 100 from SPIR-V
   - Generated code may have subtle differences in precision/operations

2. **Shader State Loss on Context Switch**
   - KGSL explicitly saves/restores 18KB of shader state per context
   - Freedreno relies on userspace to manage shader state
   - Qt6 may assume KGSL-style implicit state preservation

3. **Blend Equation Differences**
   - Qt5 had full control over blend equations
   - Qt6 is restricted to GL_FUNC_ADD
   - Combined with freedreno's blend handling, may cause issues

4. **Precision Mode Mismatches**
   - KGSL had specific precision handling (mediump vs highp per operation)
   - Mesa uses NIR lowering passes with different semantics
   - Float precision differences could affect alpha calculations

5. **Missing Leia Workarounds**
   - KGSL has Adreno 220 specific optimizations
   - Freedreno may be missing some of these

### The "Translation Stack" Problem

```
Qt5 + KGSL:
  GLSL ES 100 (hand-written) → KGSL compiler → GPU

Qt6 + Freedreno:
  Vulkan GLSL → SPIR-V → SPIRV-Cross → GLSL ES 100 → Mesa NIR → IR2 → GPU
```

**Each translation step can introduce subtle differences.**

---

## Conclusion

The display artifacts are confirmed to be a **userspace issue** in the Qt6/Mesa/freedreno rendering stack, not the kernel. The combination of:

1. Qt6's SPIRV-Cross shader translation (vs Qt5's native GLSL ES 100)
2. Freedreno's different shader compiler and state management (vs KGSL)
3. A2XX hardware limitations (no integers, limited registers, forced lowering)
4. Qt6's restricted blend equation control

...creates a "perfect storm" for rendering artifacts on the Adreno 220.

**Recommended Investigation:**
1. Compare SPIRV-Cross generated GLSL ES 100 with original Qt5 shaders
2. Check if blend factors are being set correctly by Qt6 RHI GLES2 backend
3. Verify shader compilation is succeeding (no silent failures)
4. Consider Qt5.15 fallback if Qt6 issues cannot be resolved

---

## Detailed Shader Comparison: Qt5 vs Qt6

### Qt5 Native GLSL ES 100 Shaders

**flatcolor.vert:**
```glsl
attribute highp vec4 vCoord;
uniform highp mat4 matrix;

void main()
{
    gl_Position = matrix * vCoord;
}
```

**flatcolor.frag:**
```glsl
uniform lowp vec4 color;

void main()
{
    gl_FragColor = color;
}
```

**texture.frag:**
```glsl
varying highp vec2 qt_TexCoord;
uniform sampler2D qt_Texture;
uniform lowp float opacity;

void main()
{
    gl_FragColor = texture2D(qt_Texture, qt_TexCoord) * opacity;
}
```

**smoothcolor.vert (complex):**
```glsl
uniform highp vec2 pixelSize;
uniform highp mat4 matrix;
uniform lowp float opacity;
attribute highp vec4 vertex;
attribute lowp vec4 vertexColor;
attribute highp vec2 vertexOffset;
varying lowp vec4 color;

void main()
{
    highp vec4 pos = matrix * vertex;
    gl_Position = pos;
    // ... complex anti-aliasing offset calculations ...
    color = vertexColor * opacity;
}
```

### Qt6 Vulkan GLSL 440 Source (Pre-Translation)

**flatcolor.vert:**
```glsl
#version 440
layout(location = 0) in vec4 vertexCoord;
layout(std140, binding = 0) uniform buf {
    vec4 matrix[4];  // mat4 stored as 4 vec4s
    vec4 color;
};

void main()
{
    gl_Position = mat4(matrix[0],matrix[1],matrix[2],matrix[3]) * vertexCoord;
}
```

**flatcolor.frag:**
```glsl
#version 440
layout(location = 0) out vec4 fragColor;
layout(std140, binding = 0) uniform buf {
    vec4 matrix[4];
    vec4 color;
};

void main()
{
    fragColor = color;
}
```

**texture.frag:**
```glsl
#version 440
layout(location = 0) in vec2 qt_TexCoord;
layout(location = 0) out vec4 fragColor;
layout(std140, binding = 0) uniform buf {
    vec4 qt_Matrix[4];
    float opacity;
};
layout(binding = 1) uniform sampler2D qt_Texture;

void main()
{
    fragColor = texture(qt_Texture, qt_TexCoord) * opacity;
}
```

### SPIRV-Cross Translation to GLSL ES 100

When SPIRV-Cross translates Qt6 shaders to GLSL ES 100 with `--flatten-ubo`:

```glsl
// Uniform block becomes vec4 array:
// uniform buf { vec4 matrix[4]; vec4 color; }
// becomes:
uniform vec4 buf[5];  // 4 for matrix + 1 for color

// Access pattern changes:
// color → buf[4]
// matrix → mat4(buf[0], buf[1], buf[2], buf[3])
```

### Critical Differences Summary

| Aspect | Qt5 Native | Qt6 via SPIRV-Cross |
|--------|-----------|---------------------|
| **Uniform type** | Individual uniforms | vec4 array (flattened UBO) |
| **mat4 handling** | Direct `uniform mat4` | `vec4[4]` + reconstruction |
| **Precision** | Explicit (`highp`, `lowp`) | May be added by SPIRV-Cross |
| **Attributes** | `attribute vec4` | `attribute vec4` (translated) |
| **Varyings** | `varying vec4` | `varying vec4` (translated) |
| **Texture** | `texture2D()` | `texture2D()` (translated) |
| **Output** | `gl_FragColor` | `gl_FragColor` (translated) |

### Potential Issues Identified

1. **Uniform Array Indexing (Critical)**
   - GLES 2.0 spec allows limited support for array indexing
   - `buf[4]` to access color requires constant index support
   - Some Adreno 220 drivers may have bugs with uniform array access

2. **mat4 Reconstruction Overhead**
   - Qt6: `mat4(buf[0], buf[1], buf[2], buf[3])` every vertex
   - Qt5: Direct `matrix * vertex` with native mat4
   - May have precision or performance differences

3. **std140 Layout Assumptions**
   - SPIRV-Cross assumes std140 alignment (16-byte)
   - Flattened to vec4 array maintains this
   - Driver must handle this correctly

4. **Precision Qualifier Mapping**
   - Qt5: Explicit precision per variable
   - Qt6: SPIRV-Cross may add default precision
   - freedreno A2XX lowers all to float anyway (no integers)

5. **Opacity Multiplication**
   - Both multiply by opacity in same way
   - But uniform access path differs

---

## Qt6 Native GLSL ES 100 Bypass Options

### Option 1: Replace Shaders in .qsb Files

The `qsb` tool supports replacing shader variants:

```bash
# Replace GLSL ES 100 variant with hand-written shader
qsb -r glsl,100es,custom_flatcolor.frag flatcolor.frag.qsb
```

This allows injecting native GLSL ES 100 shaders into Qt6's .qsb packages.

### Option 2: Custom QSGMaterial with Native Shaders

Create custom materials that load native GLSL ES 100:

```cpp
class NativeFlatColorMaterial : public QSGMaterial {
    QSGMaterialShader *createShader() {
        // Load native GLSL ES 100 shader
    }
};
```

### Option 3: Qt Build Configuration

Check if Qt6 can be built with native shader paths:
- `QT_QUICK_BACKEND=software` - bypasses GPU entirely
- Custom `QSG_RHI_BACKEND` settings
- Build-time shader generation options

### Option 4: Patch SPIRV-Cross Output

Modify the SPIRV-Cross translation to generate Qt5-compatible shaders:
- Avoid vec4 array flattening
- Use individual uniforms
- Match Qt5 precision qualifiers

---

## Recommended Actions

### Immediate Testing

1. **Compare shader compilation logs:**
   ```bash
   export MESA_GLSL=dump
   # Run Qt app, check for shader compilation errors
   ```

2. **Check uniform array access:**
   - Monitor if `buf[N]` array access works on freedreno A2XX
   - May need Mesa patches for correct handling

3. **Test with replaced shaders:**
   - Replace Qt6 .qsb shaders with Qt5-style native GLSL ES 100
   - Use `qsb -r` replacement option

### Build-Time Changes

4. **Rebuild Qt6 with custom shader options:**
   - Check `qt6_add_shaders()` CMake options
   - May need to disable SPIRV-Cross translation
   - Generate native GLSL ES 100 directly

5. **Patch luna-surfacemanager:**
   - Override default Qt materials with native shaders
   - Custom compositor materials for Adreno 220

---

## References

- [Mesa Freedreno Documentation](https://docs.mesa3d.org/drivers/freedreno.html)
- [Adreno 220 Specifications](https://en.wikipedia.org/wiki/Adreno)
- [Mesa GitLab - A508 artifacts issue](https://gitlab.freedesktop.org/mesa/mesa/-/issues/8442)
- [KGSL Kernel Wiki](https://github.com/freedreno/freedreno/wiki/kgsl-kernel)
- [Journey Through Freedreno](https://fryzekconcepts.com/notes/freedreno_journey.html)
- [Graphics in Qt 6.0: QRhi, Qt Quick, Qt Quick 3D](https://www.qt.io/blog/graphics-in-qt-6.0-qrhi-qt-quick-qt-quick-3d)
- [Qt Shader Tools Overview](https://doc.qt.io/qt-6/qtshadertools-overview.html)
- [QSB Manual](https://doc.qt.io/qt-6/qtshadertools-qsb.html)
- [Changes to Qt Quick (Qt5 to Qt6)](https://doc.qt.io/qt-6/quick-changes-qt6.html)
- [Qualcomm Adreno OpenGL ES Developer Guide](https://usermanual.wiki/Document/QualcommC2AE20AdrenoE284A220OpenGL20ES20Developer20Guide20.1618769787/amp)
- [OpenGL ES Shading Language Potholes](https://bitiotic.com/blog/2013/09/24/opengl-es-shading-language-potholes-and-problems/)
- [SPIRV-Cross GitHub](https://github.com/KhronosGroup/SPIRV-Cross)
- [SPIRV-Cross flatten-ubo documentation](https://manpages.ubuntu.com/manpages/jammy/man1/spirv-cross.1.html)
- [Qt5 QSGMaterialShader](https://doc.qt.io/archives/qt-5.15/qsgmaterialshader.html)
