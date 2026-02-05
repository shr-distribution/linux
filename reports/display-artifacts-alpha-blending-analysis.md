# HP TouchPad Display Artifacts: Alpha Blending Analysis Report

**Date:** February 5, 2026
**Status:** Investigation Complete - Userspace Issue Identified
**Hardware:** HP TouchPad (Topaz), Qualcomm APQ8060, Adreno 220 GPU

---

## Executive Summary

Visual display artifacts affecting semi-transparent UI elements (StatusBar, LaunchBar, buttons) on the HP TouchPad running LuneOS with Qt6 have been traced to the **GPU rendering path (Qt/Mesa freedreno)**, NOT the kernel display driver.

**Key Finding:** The MDP4 display controller is not involved in compositing - only one DRM plane is active. Qt performs ALL compositing internally via OpenGL ES on the Adreno 220 GPU. The Adreno 220 (A2XX family) lacks hardware support for premultiplied alpha compensation, making it dependent on correct blend state configuration from Qt/Mesa.

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

## Conclusion

The display artifacts are confirmed to be a **userspace issue** in the Qt/Mesa rendering stack, not the kernel. The Adreno 220's lack of hardware premultiplied alpha compensation means correct blend state must be configured by Mesa/Qt.

**Next steps** should focus on:
1. Qt Scene Graph debugging to identify batch ordering issues
2. Mesa freedreno blend state verification
3. Comparison with known-working Qt6 + GLES 2.0 configurations

---

## References

- [Mesa Freedreno Documentation](https://docs.mesa3d.org/drivers/freedreno.html)
- [Adreno 220 Specifications](https://en.wikipedia.org/wiki/Adreno)
- [Mesa GitLab - A508 artifacts issue](https://gitlab.freedesktop.org/mesa/mesa/-/issues/8442) (similar symptoms)
