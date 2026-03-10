# glmark2 Test Diagnostic Guide for Adreno 220 (A22X)

This guide maps visual symptoms to potential GPU issues for diagnosing the intermittent faceted rendering problem.

---

## Quick Reference: Symptom to Cause

| Visual Symptom | Likely Cause | Related Register/Feature |
|----------------|--------------|--------------------------|
| Faceted/flat shading on smooth models | Varying interpolation failure | SQ_INTERPOLATOR_CNTL |
| Grey/washed out textures | Texture cache coherency | TC_CNTL_STATUS, L2 cache |
| Flickering between correct/incorrect | Race condition timing | WFI, cache flush sync |
| Blocky lighting gradients | Per-vertex data not interpolated | Varying linkage |
| Missing specular highlights | Normal vector corruption | Varying interpolation |
| Texture seams/discontinuities | Texture coordinate interpolation | SQ_INTERPOLATOR_CNTL |
| GPU hang/freeze | Command processor stall | VGT_DMA, sync events |
| Completely black output | Shader execution failure | SQ_GPR_MANAGEMENT |
| Wrong colors but correct geometry | Fragment shader input issue | PS input linkage |

---

## Test-by-Test Analysis

### 1. BUILD Test

**What it renders:** 3D models (horse, buddha, dragon, bunny, armadillo, angel)

**Shaders:** `light-basic.vert` / `light-basic.frag`

**Technique:** Gouraud shading (per-vertex lighting)

**Varyings used:**
```glsl
varying vec4 Color;  // Computed in VS, interpolated to FS
```

**Visual symptoms and causes:**

| Symptom | Cause | Confidence |
|---------|-------|------------|
| Faceted triangles visible | `Color` varying flat interpolated | HIGH |
| Correct shape, wrong shading | SQ_INTERPOLATOR_CNTL race | HIGH |
| Intermittent smooth/faceted | Timing-dependent cache issue | HIGH |
| Model looks "low-poly" | Normal data not reaching VS | MEDIUM |

**Why this test is critical:** Simplest lighting test - if this fails, the issue is fundamental to varying interpolation, not complex shader features.

---

### 2. SHADING Test

**What it renders:** Rotating torus or other model

**Shading modes tested:**

#### 2a. Gouraud (`shading=gouraud`)
- **Shaders:** `light-basic.vert/frag`
- **Same as BUILD test**

#### 2b. Blinn-Phong (`shading=blinn-phong-inf`)
- **Shaders:** `light-advanced.vert/frag`
- **Varyings:**
```glsl
varying vec3 vertex_normal;    // For per-pixel lighting
varying vec4 vertex_position;  // For specular calculation
```

#### 2c. Phong (`shading=phong`)
- **Shaders:** `light-phong.vert/frag`
- **Varyings:**
```glsl
varying vec3 vertex_normal;
varying vec4 vertex_position;
```
- **Fragment shader:**
```glsl
vec3 normalized_normal = normalize(vertex_normal);  // Requires smooth interpolation!
```

#### 2d. Cel (`shading=cel`)
- **Shaders:** `light-phong.vert` + `light-cel.frag`
- **Cartoon-style banding based on normal dot product**

**Visual symptoms and causes:**

| Symptom | Test Affected | Cause |
|---------|---------------|-------|
| Faceted on Gouraud only | gouraud | Color varying issue |
| Faceted on ALL modes | all | SQ_INTERPOLATOR_CNTL fundamental |
| Gouraud OK, Phong faceted | phong, blinn-phong | Normal varying interpolation |
| Specular missing/blocky | blinn-phong, phong | Position varying issue |
| Cel looks correct | cel | Banding hides interpolation errors |

**Diagnostic value:** If Gouraud fails but Phong works, investigate Color vs Normal varying differences. If all fail, the problem is universal to varying interpolation.

---

### 3. TEXTURE Test

**What it renders:** Textured 3D model rotating

**Shaders:** `light-basic-tex.vert/frag` or `light-basic-texgen.vert`

**Filter modes:**
- `nearest` - GL_NEAREST (no filtering)
- `linear` - GL_LINEAR (hardware bilinear)
- `mipmap` - GL_LINEAR_MIPMAP_LINEAR
- `linear-shader` - Bilinear in shader using GL_NEAREST

**Varyings:**
```glsl
varying vec4 Color;           // Lighting
varying vec2 TextureCoord;    // UV coordinates
```

**Visual symptoms and causes:**

| Symptom | Filter Mode | Cause |
|---------|-------------|-------|
| Faceted lighting + correct texture | all | Color varying only |
| Texture appears blocky/wrong | linear, mipmap | Texture cache (TC) issue |
| Texture seams visible | all | TextureCoord interpolation |
| Texture correct, lighting flat | all | Color varying flat |
| Grey texture | all | L2 cache not invalidated |

**Diagnostic value:** Separates texture sampling issues from varying interpolation. If texture looks correct but lighting is faceted, the problem is specifically Color varying, not general cache coherency.

---

### 4. BUFFER Test

**What it renders:** Animated sine wave grid mesh

**Technique:** Tests VBO update performance (map vs subdata)

**Shaders:** Basic vertex color shaders

**Varyings:**
```glsl
varying vec4 Color;
```

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| Wave animation stutters | VBO update timing |
| Faceted wave surface | Color varying interpolation |
| Mesh corruption/garbage | VGT DMA not complete |
| Partial updates visible | Buffer coherency |

**Diagnostic value:** Tests dynamic vertex buffer updates. If this shows corruption but static tests (build) work, the issue is VBO timing/coherency.

---

### 5. JELLYFISH Test

**What it renders:** Animated translucent jellyfish with caustics

**Shaders:** `jellyfish.vert/frag` + `gradient.vert/frag`

**Varyings:**
```glsl
varying vec2 vTextureCoord;   // For main texture
varying vec4 vWorld;          // World position (caustics UV)
varying vec3 vDiffuse;        // Per-vertex diffuse
varying vec3 vAmbient;        // Per-vertex ambient
varying vec3 vFresnel;        // Rim lighting
```

**Features:**
- 33 textures (1 main + 32 animated caustics)
- Alpha blending
- Animated vertex positions (sine waves)
- Per-vertex lighting (Gouraud-style)

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| Faceted jellyfish body | vDiffuse/vAmbient/vFresnel flat |
| Missing rim glow | vFresnel interpolation |
| Blocky caustic patterns | vWorld interpolation wrong |
| Texture flickering | Texture cache timing |
| Animation jerky | Vertex buffer timing |
| Transparency wrong | Alpha blending state |

**Diagnostic value:** Complex test combining multiple features. If jellyfish fails but simpler tests pass, look for interaction effects.

---

### 6. SHADOW Test

**What it renders:** Horse model with shadow on ground plane

**Technique:** Two-pass shadow mapping:
1. Depth pass from light perspective (to FBO)
2. Shadow pass comparing depths

**Requires:** FBO support, depth textures

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| No shadow visible | FBO/depth texture failure |
| Shadow in wrong position | Matrix calculation (not GPU) |
| Shadow edges blocky | Depth texture precision |
| Horse faceted | Same as BUILD test |
| Shadow flickers | Depth buffer race |

**Diagnostic value:** Tests FBO and depth texture functionality. If shadow works but horse is faceted, confirms varying issue is separate from FBO path.

---

### 7. BUMP Test

**What it renders:** Asteroid model with surface detail

**Modes:**
- `off` - Flat shading
- `high-poly` - Actual geometry
- `normals` - Object-space normal mapping
- `normals-tangent` - Tangent-space normal mapping
- `height` - Height-map based bumps

**Varyings (tangent mode):**
```glsl
varying vec2 TextureCoord;
varying vec3 Normal;
varying vec3 Tangent;
varying vec3 Bitangent;  // computed
```

**Visual symptoms and causes:**

| Symptom | Mode Affected | Cause |
|---------|---------------|-------|
| Bumps look flat | normals, height | Normal map sampling |
| Lighting faceted | all | Base normal interpolation |
| Bumps face wrong way | normals-tangent | Tangent/bitangent wrong |
| High-poly looks correct | high-poly | Confirms geometry path OK |

**Diagnostic value:** If high-poly mode looks smooth but normal-mapped modes are wrong, the issue is texture-based normal handling, not varying interpolation.

---

### 8. REFRACT Test

**What it renders:** Refractive object (bunny, etc.)

**Technique:** Two-pass with depth/distance map

**Varyings:**
```glsl
varying vec3 Normal;
varying vec3 Position;
```

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| Refraction looks blocky | Normal interpolation |
| No refraction visible | FBO/texture setup |
| Wrong refraction angle | Index of refraction calc (shader) |

---

### 9. TERRAIN Test

**What it renders:** Procedural terrain with optional bloom/tilt-shift

**Technique:**
- Simplex noise height map (256x256)
- Vertex texture fetch for displacement
- Multi-pass post-processing

**Requires:** Vertex texture fetch support

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| Flat terrain | Vertex texture fetch failed |
| Blocky terrain | Height map precision |
| Lighting faceted | Normal generation from height |
| Bloom missing | FBO post-process failure |

**Diagnostic value:** Tests vertex texture fetch (uncommon feature). If terrain is flat, VTF may not be supported/working.

---

### 10. PULSAR Test

**What it renders:** Rotating textured quads with alpha blending

**Shaders:** `light-basic.vert` + `light-basic-tex.frag` or `light-basic.frag`

**Features:**
- Multiple quads with independent rotation
- Optional texturing
- Alpha blending (0.4 alpha per vertex)

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| Faceted quad lighting | Color varying (same as BUILD) |
| Blending wrong | Blend state setup |
| Texture missing | Texture binding |

---

### 11. IDEAS Test

**What it renders:** Animated SGI logo with shadows

**Technique:**
- Shadow by geometry flattening
- Multiple light sources
- Spline-based camera animation

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| Logo faceted | Varying interpolation |
| Shadow missing/wrong | Depth/culling state |
| Animation wrong | Not GPU related |

---

### 12. DESKTOP Test

**What it renders:** Composited windows with blur/shadow effects

**Technique:**
- Render to FBO per window
- Gaussian blur via shader convolution
- Alpha compositing

**Visual symptoms and causes:**

| Symptom | Cause |
|---------|-------|
| No blur visible | FBO or shader issue |
| Blur looks blocky | Texture filtering |
| Windows faceted | Unlikely (2D quads) |
| Compositing wrong | Blend state |

---

### 13. CONDITIONALS / LOOP / FUNCTION Tests

**What they render:** Grid patterns

**Purpose:** Shader branching/loop/function call performance

**Varyings:** Minimal (position, color)

**Visual symptoms:** These are shader stress tests. If they show visual artifacts, the shader compiler or execution unit has issues - not typically related to varying interpolation.

---

### 14. EFFECT-2D Test

**What it renders:** 2D image with convolution filter

**Kernels:** Edge detection, blur, custom

**Visual symptoms:**

| Symptom | Cause |
|---------|-------|
| No effect visible | Texture sampling issue |
| Wrong colors | Convolution kernel error |
| Blocky output | Texture coordinate precision |

---

## Diagnostic Decision Tree

```
START: Run glmark2 build test 10 times
       │
       ├─ Always faceted ─────────────► Fundamental SQ_INTERPOLATOR_CNTL issue
       │                                Check: Register not being set
       │
       ├─ Always smooth ──────────────► A22X fixes working
       │
       └─ Intermittent (X% smooth) ──► Timing/race condition
                │
                ├─ Run texture test
                │   ├─ Texture OK, lighting faceted ──► Varying-specific issue
                │   └─ Texture also wrong ────────────► General cache coherency
                │
                ├─ Run jellyfish test
                │   ├─ Same intermittent rate ────────► Universal timing issue
                │   └─ Different rate ────────────────► Test-specific factors
                │
                └─ Compare Gouraud vs Phong in shading test
                    ├─ Both faceted ──────────────────► All varyings affected
                    └─ Only Gouraud faceted ──────────► Color varying specific
```

---

## Recording Visual Feedback

When testing, record:

1. **Test name and options** (e.g., `build:model=horse`)
2. **Result:** Smooth / Faceted / Grey / Black / Hang
3. **Consistency:** Same result on repeat? Or intermittent?
4. **Screenshot if possible**

Example format:
```
Test: build:model=horse
Run 1: Faceted
Run 2: Smooth
Run 3: Faceted
Run 4: Faceted
Run 5: Smooth
Result: 40% smooth (2/5)
```

---

## Correlation Table

Fill this in during testing:

| Test | Always OK | Always Fail | Intermittent | Notes |
|------|-----------|-------------|--------------|-------|
| build | | | | |
| shading:gouraud | | | | |
| shading:phong | | | | |
| texture | | | | |
| buffer | | | | |
| jellyfish | | | | |
| shadow | | | | |
| bump:normals | | | | |
| ideas | | | | |

If a pattern emerges (e.g., all Gouraud tests fail but Phong works), that narrows the cause significantly.
