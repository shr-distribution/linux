/*
 * vidc_gl_play — VIDC → GLES → MDP4 RGB playback (legacy webOS path).
 *
 * Replicates the path TextureVideoSink in webOS gstpdksink took: the GPU
 * (Adreno 220) samples the VIDC tile-NV12 dma-buf as an EGL external image,
 * YUV→RGB + scale happens in the shader, output renders to a GBM-backed
 * RGB surface, and MDP4 just scans out a single RGB primary plane — no VG
 * overlay, no FIR scaler, no SMI contention with VIDC writes (the GPU reads
 * from SMI, but its fetch is bounded by frame rate, not panel scanout).
 *
 * Pipeline:
 *   /dev/video6 (VIDC) → tile NV12 dma-buf
 *     → EGLImageKHR via EGL_EXT_image_dma_buf_import_modifiers
 *     → GL_TEXTURE_EXTERNAL_OES
 *     → fullscreen quad with GL_OES_EGL_image_external fragment shader
 *     → GBM surface (1024x768 ARGB8888)
 *     → DRM dumb buffer (via gbm_bo_get_handle)
 *     → atomic commit on /dev/dri/card0 primary plane
 *
 * Build (host with ARM cross-toolchain + Yocto sysroot):
 *   SYSROOT=/media/.../recipe-sysroot
 *   arm-linux-gnueabihf-gcc -O2 -Wall --sysroot=$SYSROOT \
 *     -I$SYSROOT/usr/include/libdrm \
 *     reports/vidc-tests/vidc_gl_play.c -o /tmp/vidc_gl_play \
 *     -lEGL -lGLESv2 -lgbm -ldrm
 *
 * Run (NO surface-manager, NO compositor):
 *   /tmp/vidc_gl_play /tmp/michael_30s.264
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <linux/videodev2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#define die(m) do { perror(m); exit(1); } while (0)
#define MAX_CAP 10

/* ------------------------------------------------------------------ DRM/GBM */

static int drm_fd, gbm_fd;
static struct gbm_device *gbm;
static struct gbm_surface *gbm_surf;
static EGLDisplay egl_dpy;
static EGLContext egl_ctx;
static EGLSurface egl_surf;

static uint32_t crtc_id, conn_id, primary_id;
static drmModeModeInfo mode;
static drmModeCrtc *saved_crtc;

/* atomic plane property ids */
static uint32_t p_fb_id, p_crtc_id, p_src_x, p_src_y, p_src_w, p_src_h;
static uint32_t p_crtc_x, p_crtc_y, p_crtc_w, p_crtc_h, p_rotation;

static PFNEGLCREATEIMAGEKHRPROC  pf_eglCreateImageKHR;
static PFNEGLDESTROYIMAGEKHRPROC pf_eglDestroyImageKHR;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC pf_glEGLImageTargetTexture2DOES;

static void resolve_plane_props(uint32_t plane)
{
	drmModeObjectProperties *props =
		drmModeObjectGetProperties(drm_fd, plane, DRM_MODE_OBJECT_PLANE);
	if (!props) die("plane props");
	for (uint32_t i = 0; i < props->count_props; i++) {
		drmModePropertyRes *pr = drmModeGetProperty(drm_fd,
							    props->props[i]);
		if (!pr) continue;
		#define MAP(s, v) if (!strcmp(pr->name, s)) v = pr->prop_id
		MAP("FB_ID",   p_fb_id);
		MAP("CRTC_ID", p_crtc_id);
		MAP("SRC_X",   p_src_x);
		MAP("SRC_Y",   p_src_y);
		MAP("SRC_W",   p_src_w);
		MAP("SRC_H",   p_src_h);
		MAP("CRTC_X",  p_crtc_x);
		MAP("CRTC_Y",  p_crtc_y);
		MAP("CRTC_W",  p_crtc_w);
		MAP("CRTC_H",  p_crtc_h);
		MAP("rotation", p_rotation);
		#undef MAP
		drmModeFreeProperty(pr);
	}
	drmModeFreeObjectProperties(props);
}

static void drm_init(const char *card)
{
	drm_fd = open(card, O_RDWR | O_CLOEXEC);
	if (drm_fd < 0) die("open card");

	/* Allow GBM buffers in primary plane scanout. */
	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1))
		die("DRM_CLIENT_CAP_UNIVERSAL_PLANES");
	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1))
		die("DRM_CLIENT_CAP_ATOMIC");

	drmModeRes *res = drmModeGetResources(drm_fd);
	if (!res) die("getResources");

	/* Find first connected connector. */
	drmModeConnector *conn = NULL;
	for (int i = 0; i < res->count_connectors; i++) {
		conn = drmModeGetConnector(drm_fd, res->connectors[i]);
		if (conn && conn->connection == DRM_MODE_CONNECTED &&
		    conn->count_modes > 0)
			break;
		drmModeFreeConnector(conn);
		conn = NULL;
	}
	if (!conn) die("no connected connector");
	conn_id = conn->connector_id;
	mode = conn->modes[0];

	drmModeEncoder *enc = drmModeGetEncoder(drm_fd, conn->encoder_id);
	crtc_id = enc->crtc_id;
	drmModeFreeEncoder(enc);

	int crtc_idx = 0;
	for (int i = 0; i < res->count_crtcs; i++)
		if (res->crtcs[i] == crtc_id) { crtc_idx = i; break; }
	saved_crtc = drmModeGetCrtc(drm_fd, crtc_id);

	/* Find primary plane (RGB) bound to this CRTC. */
	drmModePlaneRes *pr = drmModeGetPlaneResources(drm_fd);
	for (uint32_t i = 0; i < pr->count_planes; i++) {
		drmModePlane *pl = drmModeGetPlane(drm_fd, pr->planes[i]);
		if (pl && (pl->possible_crtcs & (1u << crtc_idx))) {
			/* primary plane usually advertises XRGB8888 */
			for (uint32_t f = 0; f < pl->count_formats; f++)
				if (pl->formats[f] == DRM_FORMAT_XRGB8888 ||
				    pl->formats[f] == DRM_FORMAT_ARGB8888) {
					primary_id = pl->plane_id;
					break;
				}
			if (primary_id) {
				drmModeFreePlane(pl);
				break;
			}
		}
		drmModeFreePlane(pl);
	}
	drmModeFreePlaneResources(pr);
	drmModeFreeConnector(conn);
	drmModeFreeResources(res);
	if (!primary_id) die("no primary plane");

	resolve_plane_props(primary_id);

	fprintf(stderr, "DRM: %dx%d@%d crtc=%u conn=%u plane=%u\n",
		mode.hdisplay, mode.vdisplay, mode.vrefresh,
		crtc_id, conn_id, primary_id);
}

static void egl_init(void)
{
	gbm = gbm_create_device(drm_fd);
	if (!gbm) die("gbm_create_device");

	gbm_surf = gbm_surface_create(gbm, mode.hdisplay, mode.vdisplay,
				      GBM_FORMAT_XRGB8888,
				      GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
	if (!gbm_surf) die("gbm_surface_create");

	PFNEGLGETPLATFORMDISPLAYEXTPROC getPlatform =
		(PFNEGLGETPLATFORMDISPLAYEXTPROC)
		eglGetProcAddress("eglGetPlatformDisplayEXT");
	egl_dpy = getPlatform ? getPlatform(EGL_PLATFORM_GBM_MESA, gbm, NULL)
				: eglGetDisplay((EGLNativeDisplayType)gbm);
	if (egl_dpy == EGL_NO_DISPLAY) die("eglGetDisplay");

	EGLint maj, min;
	if (!eglInitialize(egl_dpy, &maj, &min)) die("eglInitialize");
	fprintf(stderr, "EGL %d.%d vendor='%s'\n", maj, min,
		eglQueryString(egl_dpy, EGL_VENDOR));

	const char *ext = eglQueryString(egl_dpy, EGL_EXTENSIONS);
	if (!strstr(ext, "EGL_EXT_image_dma_buf_import_modifiers")) {
		fprintf(stderr, "EGL_EXT_image_dma_buf_import_modifiers missing\n");
		exit(1);
	}

	if (!eglBindAPI(EGL_OPENGL_ES_API)) die("eglBindAPI");

	EGLint cfg_attr[] = {
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_RED_SIZE,     8,
		EGL_GREEN_SIZE,   8,
		EGL_BLUE_SIZE,    8,
		EGL_ALPHA_SIZE,   0,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};
	EGLConfig cfg;
	EGLint n;
	if (!eglChooseConfig(egl_dpy, cfg_attr, &cfg, 1, &n) || n != 1)
		die("eglChooseConfig");

	EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	egl_ctx = eglCreateContext(egl_dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
	if (egl_ctx == EGL_NO_CONTEXT) die("eglCreateContext");

	egl_surf = eglCreateWindowSurface(egl_dpy, cfg,
					  (EGLNativeWindowType)gbm_surf, NULL);
	if (egl_surf == EGL_NO_SURFACE) die("eglCreateWindowSurface");
	if (!eglMakeCurrent(egl_dpy, egl_surf, egl_surf, egl_ctx))
		die("eglMakeCurrent");

	pf_eglCreateImageKHR =
		(PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
	pf_eglDestroyImageKHR =
		(PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
	pf_glEGLImageTargetTexture2DOES =
		(PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)
		eglGetProcAddress("glEGLImageTargetTexture2DOES");
	if (!pf_eglCreateImageKHR || !pf_glEGLImageTargetTexture2DOES)
		die("missing EGL/GLES extension fns");

	fprintf(stderr, "GLES renderer='%s'\n", glGetString(GL_RENDERER));
	fprintf(stderr, "GLES version='%s'\n", glGetString(GL_VERSION));
}

/* ---------------------------------------------------------------- GLES shader */

/* VIDC-reported geometry; set in dec_after_src_change(), consumed in gles_draw. */
static unsigned width, height, dec_bytesperline, dec_chroma_off;

static GLuint prog;
static GLint  loc_pos, loc_uv_attr;
static GLint  loc_s_y, loc_s_uv;
static GLint  loc_lin_size_y, loc_tex_size_y, loc_tex_size_uv, loc_tiles_x;

static const char *vs_src =
	"attribute vec2 a_pos;\n"
	"attribute vec2 a_uv;\n"
	"varying vec2 v_uv;\n"
	/* TouchPad LCD panel scan direction differs from the GL framebuffer
	 * by a vertical flip only. The quad_uv table already X-mirrors
	 * (NDC-left maps to UV.u=1), so negating only NDC Y here aligns the
	 * displayed image with the physical panel orientation without
	 * introducing a mirror. (Negating both axes mirrors the image.) */
	"void main(){ v_uv = a_uv; gl_Position = vec4(a_pos.x, -a_pos.y, 0.0, 1.0); }\n";

/*
 * De-tile + YUV->RGB fragment shader.
 *
 * VIDC's CAPTURE buffer is Samsung 64x32 tiled NV12. We can't use
 * samplerExternalOES because Mesa's external sampler insists the Y and
 * UV planes are read at matching linear positions, so we can de-tile
 * only one plane consistently. Instead we import Y and UV as two
 * separate single-channel R8 textures and do the YUV->RGB ourselves.
 *
 * Tile index serpentine (matches vidc_tile_index from the reverted
 * kernel commit 9269213441f9):
 *   k = ty >> 1; bx = tx >> 1; wbc = tx & 1;
 *   row = (bx & 1) ? (1 - (ty & 1)) : (ty & 1);
 *   idx = k * (tiles_x >> 1) * 4 + bx * 4 + row * 2 + wbc
 *
 * Y plane: tiles_x_Y * tiles_y_Y tiles, each 64x32 bytes.
 * UV plane: tiles_x_UV * tiles_y_UV (same width, half height) — each
 *   tile still 64x32 bytes but the bytes are interleaved U/V pairs, so
 *   the chroma plane covers w*(h/2) bytes total.
 *
 * For each output pixel (lin_x, lin_y):
 *  1. Y lookup: de-tile (lin_x, lin_y) on the Y tile grid -> sample Y.
 *  2. UV lookup: at chroma-subsampled location (lin_x/2, lin_y/2), but
 *     the chroma plane is stored "bytes-per-pixel = 2" so the byte
 *     coord is (lin_x & ~1, lin_y/2). De-tile that -> sample U at the
 *     even-x byte and V at the odd-x byte. Since the byte coord we
 *     compute has even x, sample x and x+1.
 */
static const char *fs_src =
	/* Force-LOD0 sampling via GL_EXT_shader_texture_lod to bypass the
	 * derivative-computed LOD path in A220's fragment shader sampler.
	 * Our de-tile lookup has extreme du/dv at tile boundaries (jumping
	 * by 2048 bytes per 64-pixel column), and the implicit-LOD code path
	 * in A220 may misbehave with such derivatives even when NEAREST
	 * filtering + BASEMAP mip filter should make LOD irrelevant. */
	"#extension GL_EXT_shader_texture_lod : enable\n"
	"#define TEX2D(s, c) texture2DLodEXT(s, c, 0.0)\n"
	"precision highp float;\n"
	/* CRITICAL: GLES2 default precision for sampler2D in a fragment
	 * shader is lowp. lowp gives only ~1/256 norm-coord precision,
	 * which on a 1024-wide texture is ~4 px of rounding error —
	 * exactly the granularity at which 64-px tile-column boundaries
	 * misalign and produce the every-other-column vertical stagger.
	 * NIR dump (FD_MESA_DEBUG=disasm) confirmed s_y/s_uv were lowp.
	 */
	"precision highp sampler2D;\n"
	"varying vec2 v_uv;\n"
	"uniform sampler2D s_y;       /* Y plane, R8, full visible size */\n"
	"uniform sampler2D s_uv;      /* UV plane, R8 raw bytes, w * h/2 */\n"
	"uniform vec2 u_lin_size_y;   /* visible Y width x height */\n"
	"uniform vec2 u_tex_size_y;   /* tiled Y texture w x h */\n"
	"uniform vec2 u_tex_size_uv;  /* tiled UV texture w x h (bytes) */\n"
	"uniform float u_tiles_x;     /* tile count across (same for Y,UV) */\n"
	"\n"
	"/* mod() can lose precision when its result is supposed to be exactly\n"
	" * 0 at a multiple of the divisor (Mesa A220 produces off-by-one\n"
	" * sample positions at every 64-pixel tile boundary, which manifests\n"
	" * as the alternating-column stagger that frame capture exposed).\n"
	" * Replace with explicit highp subtraction. */\n"
	"float imod(float a, float b) { return a - b * floor(a / b); }\n"
	"\n"
	"vec2 detile_lookup(vec2 lin_xy, vec2 tex_size, float tile_h) {\n"
	"  /* Precision-safe variant. Each tile is 64 * tile_h bytes. In a\n"
	"   * tex_size.x-wide R8 texture, one tile spans (64*tile_h /\n"
	"   * tex_size.x) texture rows. For tex_size.x = 1024 and\n"
	"   * tile_h = 32: each tile = 2048 bytes = 2 tex rows. For\n"
	"   * tile_h = 16 (chroma): 1024 bytes = 1 tex row.\n"
	"   *\n"
	"   * Computing src_off = idx * 64 * tile_h + py * 64 + px and then\n"
	"   * dividing by tex_size.x can blow past 16-bit float precision on\n"
	"   * A220 (highp in fragment shaders is optional in GLES2 and the\n"
	"   * Adreno 220 silently uses mediump = half-float = 11-bit mantissa,\n"
	"   * unable to represent values up to ~786K accurately). Instead,\n"
	"   * factor through small intermediate values that stay under 2048.\n"
	"   */\n"
	"  float tx = floor(lin_xy.x / 64.0);\n"
	"  float ty = floor(lin_xy.y / tile_h);\n"
	"  float px = lin_xy.x - tx * 64.0;\n"
	"  float py = lin_xy.y - ty * tile_h;\n"
	"  float k   = floor(ty * 0.5);\n"
	"  float bx  = floor(tx * 0.5);\n"
	"  float wbc = tx - bx * 2.0;\n"
	"  float ty_lsb = ty - k * 2.0;\n"
	"  float bx_lsb = bx - floor(bx * 0.5) * 2.0;\n"
	"  float row = mix(ty_lsb, 1.0 - ty_lsb, bx_lsb);\n"
	"  float idx = k * floor(u_tiles_x * 0.5) * 4.0 + bx * 4.0\n"
	"            + row * 2.0 + wbc;\n"
	"  /* tex_rows_per_tile is small: 2 for luma (tile_h=32), 1 for chroma\n"
	"   * (tile_h=16). 64 * tile_h / tex_size.x. */\n"
	"  float tex_rows_per_tile = (64.0 * tile_h) / tex_size.x;\n"
	"  /* Within-tile offset (0..2047). */\n"
	"  float intile = py * 64.0 + px;\n"
	"  /* Row offset inside the tile (0 or 1 for luma 2-row tiles). */\n"
	"  float intile_row = floor(intile / tex_size.x);\n"
	"  float intile_col = intile - intile_row * tex_size.x;\n"
	"  /* idx and tex_rows_per_tile are small (idx <= ~384 for 1024x768\n"
	"   * luma, tex_rows_per_tile <= 2) so this multiplication stays\n"
	"   * within mediump-float range. */\n"
	"  float src_y = idx * tex_rows_per_tile + intile_row;\n"
	"  float src_x = intile_col;\n"
	"  return (vec2(src_x, src_y) + 0.5) / tex_size;\n"
	"}\n"
	"\n"
	"void main(){\n"
	"  vec2 lin = v_uv * u_lin_size_y;\n"
	"#ifdef RAW_DEBUG\n"
	"  float yraw = texture2D(s_y, v_uv).r;\n"
	"  gl_FragColor = vec4(vec3(yraw), 1.0); return;\n"
	"#endif\n"
	"#ifdef IDX_DEBUG\n"
	"  /* Visualize: compute the detile_lookup's src_y (= which texture\n"
	"   * row my shader thinks the data is on) and output it as red.\n"
	"   * Within a single tile we should see a flat colour (one src_y\n"
	"   * per tile-row). The pattern of colours across tile columns\n"
	"   * should match the kernel formula's serpentine. */\n"
	"  float tx = floor(lin.x / 64.0);\n"
	"  float ty = floor(lin.y / 32.0);\n"
	"  float k = floor(ty * 0.5);\n"
	"  float bx = floor(tx * 0.5);\n"
	"  float wbc = tx - bx * 2.0;\n"
	"  float ty_lsb = ty - k * 2.0;\n"
	"  float bx_lsb = bx - floor(bx * 0.5) * 2.0;\n"
	"  float row = mix(ty_lsb, 1.0 - ty_lsb, bx_lsb);\n"
	"  float idx_v = k * floor(u_tiles_x * 0.5) * 4.0 + bx * 4.0\n"
	"              + row * 2.0 + wbc;\n"
	"  /* Visualise tx (lin_x / 64) as red, normalised. For correct\n"
	"   * 64-pixel tile columns we expect 16 vertical stripes of\n"
	"   * monotonically-changing red (0,1,2,...,15 -> 0/15, 1/15, ...).\n"
	"   * Anything else means tx is not what I think it is. */\n"
	"  gl_FragColor = vec4(tx / 15.0, 0.0, 0.0, 1.0); return;\n"
	"#endif\n"
	"#ifdef CPU_DETILED\n"
	"  /* Texture is already linear NV12 from CPU detile. Sample\n"
	"   * straight at v_uv * lin_size, no GLSL de-tile lookup. */\n"
	"  vec2 y_uv = (lin + 0.5) / u_tex_size_y;\n"
	"#else\n"
	"  /* Y sample: 64x32 luma tiles. */\n"
	"  vec2 y_uv = detile_lookup(lin, u_tex_size_y, 32.0);\n"
	"#endif\n"
	"  float Y = texture2D(s_y, y_uv).r;\n"
	"  /* UV sample: chroma-subsampled location, 64x32 byte tiles. */\n"
	"  float ux_byte = floor(lin.x / 2.0) * 2.0;\n"
	"  float uy      = floor(lin.y / 2.0);\n"
	"#ifdef CPU_DETILED\n"
	"  vec2 u_uv = (vec2(ux_byte,       uy) + 0.5) / u_tex_size_uv;\n"
	"  vec2 v_uv = (vec2(ux_byte + 1.0, uy) + 0.5) / u_tex_size_uv;\n"
	"#else\n"
	/* U and V are interleaved bytes within the SAME chroma tile (64-wide
	 * tiles, adjacent bytes => same tile, same row, +1 column). Compute
	 * the serpentine lookup once; V is one texel to the right. Saves
	 * roughly half the chroma-side fragment-shader cost. */
	"  vec2 u_uv = detile_lookup(vec2(ux_byte, uy), u_tex_size_uv, 32.0);\n"
	"  vec2 v_uv = u_uv + vec2(1.0 / u_tex_size_uv.x, 0.0);\n"
	"#endif\n"
	"  float U = texture2D(s_uv, u_uv).r;\n"
	"  float V = texture2D(s_uv, v_uv).r;\n"
	"  /* BT.601 limited-range YUV->RGB. */\n"
	"  float yp = (Y - 16.0/255.0) * 1.164;\n"
	"  float up = U - 128.0/255.0;\n"
	"  float vp = V - 128.0/255.0;\n"
	"  vec3 rgb = vec3(\n"
	"    yp + 1.596 * vp,\n"
	"    yp - 0.392 * up - 0.813 * vp,\n"
	"    yp + 2.017 * up\n"
	"  );\n"
	"  gl_FragColor = vec4(clamp(rgb, 0.0, 1.0), 1.0);\n"
	"}\n";

static GLuint compile(GLenum type, const char *src)
{
	GLuint s = glCreateShader(type);
	/* Inject preprocessor defines that mirror C-level build flags so
	 * we can use shader-side #ifdef CPU_DETILED / RAW_DEBUG. */
	const char *defines =
#if defined(CPU_DETILE)
		"#define CPU_DETILED 1\n"
#endif
#if defined(RAW_DEBUG)
		"#define RAW_DEBUG 1\n"
#endif
#if defined(IDX_DEBUG)
		"#define IDX_DEBUG 1\n"
#endif
		"";
	const char *srcs[2] = { defines, src };
	GLint lens[2] = { (GLint)strlen(defines), (GLint)strlen(src) };
	glShaderSource(s, 2, srcs, lens);
	glCompileShader(s);
	GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		char log[1024]; glGetShaderInfoLog(s, 1024, NULL, log);
		fprintf(stderr, "shader compile: %s\n", log);
		exit(1);
	}
	return s;
}

static void gles_init(void)
{
	GLuint vs = compile(GL_VERTEX_SHADER, vs_src);
	GLuint fs = compile(GL_FRAGMENT_SHADER, fs_src);
	prog = glCreateProgram();
	glAttachShader(prog, vs);
	glAttachShader(prog, fs);
	glLinkProgram(prog);
	GLint ok = 0; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
	if (!ok) {
		char log[1024]; glGetProgramInfoLog(prog, 1024, NULL, log);
		fprintf(stderr, "link: %s\n", log);
		exit(1);
	}
	loc_pos         = glGetAttribLocation(prog,  "a_pos");
	loc_uv_attr     = glGetAttribLocation(prog,  "a_uv");
	loc_s_y         = glGetUniformLocation(prog, "s_y");
	loc_s_uv        = glGetUniformLocation(prog, "s_uv");
	loc_lin_size_y  = glGetUniformLocation(prog, "u_lin_size_y");
	loc_tex_size_y  = glGetUniformLocation(prog, "u_tex_size_y");
	loc_tex_size_uv = glGetUniformLocation(prog, "u_tex_size_uv");
	loc_tiles_x     = glGetUniformLocation(prog, "u_tiles_x");
	glViewport(0, 0, mode.hdisplay, mode.vdisplay);
}

/*
 * TouchPad LVDS panel mounted physically upside-down. OpenGL's NDC has
 * y=+1 at top, while a DRM scanout framebuffer has y=0 at the top of
 * the physical buffer. With the panel mounted upside-down, the
 * physical top of the screen is what scans out FIRST (at fb y=0). So
 * to render an image that appears right-way-up to the user, the "top
 * of the source image" (texture v=0) must land at the BOTTOM of the
 * NDC (y=-1), which then gets stored at the END of the framebuffer,
 * which then arrives at the visual TOP of the upside-down panel.
 *
 * Hence: vertex (-1,-1) (bottom-left of NDC, scanned out LAST) samples
 * the BOTTOM of the source (texture v=1); vertex (-1,+1) (top-left of
 * NDC, scanned out FIRST) samples the TOP of the source (texture v=0).
 *
 * (Equivalent to setting DRM_MODE_ROTATE_180 on the plane, which
 * vidc_play uses on the MDP4 VG-pipe path, but we don't get that
 * property on the primary plane here.)
 */
static const GLfloat quad_pos[]  = { -1,-1,  1,-1, -1, 1,  1, 1 };
static const GLfloat quad_uv[]   = {  1, 1,  0, 1,  1, 0,  0, 0 };

static void gles_draw(GLuint tex_y, GLuint tex_uv
#ifdef CPU_UPLOAD
		      , const void *mmap_data
#endif
		      )
{
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	glUseProgram(prog);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex_y);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glUniform1i(loc_s_y, 0);
#ifdef CPU_UPLOAD
	/* CPU upload the Y plane bytes from the V4L2 mmap. */
	/* Mesa A2xx LUMINANCE upload is byte-by-byte when alignment=1.
	 * 4-byte alignment lets Mesa use 32-bit word writes which is much
	 * faster on write-combining GPU memory; our pitch (1024) is already
	 * a multiple of 4 so no padding effect. */
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
#ifdef CPU_DETILE
	/* CPU-detile the tile NV12 Y plane to a linear scratch buffer
	 * before uploading. With this + a no-detile shader, we should
	 * see a clean image — proving the rest of the GLES pipeline
	 * works and the bug is in the GLSL de-tile path itself. */
	{
		static unsigned char y_lin[1024 * 1024];
		const unsigned char *src = mmap_data;
		unsigned tiled_w = dec_bytesperline;
		unsigned tiles_x = tiled_w / 64;
		for (unsigned ty = 0; ty * 32 < height; ty++) {
			for (unsigned tx = 0; tx < tiles_x; tx++) {
				unsigned k = ty >> 1, bx = tx >> 1;
				unsigned wbc = tx & 1;
				unsigned base = k * (tiles_x >> 1) * 4 + bx * 4;
				unsigned row = (bx & 1) ? (1 - (ty & 1)) : (ty & 1);
				unsigned idx = base + row * 2 + wbc;
				const unsigned char *tile = src + idx * (64 * 32);
				unsigned rows = (ty * 32 + 32 > height)
					? height - ty * 32 : 32;
				unsigned cols = (tx * 64 + 64 > width)
					? width - tx * 64 : 64;
				for (unsigned py = 0; py < rows; py++)
					memcpy(&y_lin[(ty * 32 + py) * dec_bytesperline + tx * 64],
					       tile + py * 64, cols);
			}
		}
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
				dec_bytesperline, (height + 31) & ~31u,
				GL_LUMINANCE, GL_UNSIGNED_BYTE, y_lin);
	}
#else
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
			dec_bytesperline, (height + 31) & ~31u,
			GL_LUMINANCE, GL_UNSIGNED_BYTE, mmap_data);
#endif
#endif

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, tex_uv);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glUniform1i(loc_s_uv, 1);
#ifdef CPU_UPLOAD
#ifdef CPU_DETILE
	/* CPU-detile chroma too. */
	{
		static unsigned char c_lin[1024 * 512];
		const unsigned char *src =
			(const unsigned char *)mmap_data + dec_chroma_off;
		unsigned tiled_w = dec_bytesperline;
		unsigned tiles_x = tiled_w / 64;
		unsigned chh = height / 2;
		for (unsigned ty = 0; ty * 32 < chh; ty++) {
			for (unsigned tx = 0; tx < tiles_x; tx++) {
				unsigned k = ty >> 1, bx = tx >> 1;
				unsigned wbc = tx & 1;
				unsigned base = k * (tiles_x >> 1) * 4 + bx * 4;
				unsigned row = (bx & 1) ? (1 - (ty & 1)) : (ty & 1);
				unsigned idx = base + row * 2 + wbc;
				const unsigned char *tile = src + idx * (64 * 32);
				unsigned rows = (ty * 32 + 32 > chh)
					? chh - ty * 32 : 32;
				unsigned cols = (tx * 64 + 64 > width)
					? width - tx * 64 : 64;
				for (unsigned py = 0; py < rows; py++)
					memcpy(&c_lin[(ty * 32 + py) * dec_bytesperline + tx * 64],
					       tile + py * 64, cols);
			}
		}
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
				dec_bytesperline, ((height + 31) & ~31u) / 2,
				GL_LUMINANCE, GL_UNSIGNED_BYTE, c_lin);
	}
#else
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
			dec_bytesperline, ((height + 31) & ~31u) / 2,
			GL_LUMINANCE, GL_UNSIGNED_BYTE,
			(const unsigned char *)mmap_data + dec_chroma_off);
#endif
#endif

	unsigned tiled_w = dec_bytesperline ? dec_bytesperline
					    : ((width + 127) & ~127u);
	unsigned tex_h_y  = (height + 31) & ~31u;
	unsigned tex_h_uv = tex_h_y / 2;
	/* Both paths use POT-padded textures so src_y / tex_size.y hits the
	 * correct physical row (A220 NPOT sampler quirk re-introduces the
	 * tile-column stagger if we feed it NPOT). */
	{
		unsigned pot = 1; while (pot < tex_h_y) pot <<= 1;
		tex_h_y = pot;
		tex_h_uv = pot / 2;
	}
	glUniform2f(loc_lin_size_y,  (float)width, (float)height);
	glUniform2f(loc_tex_size_y,  (float)tiled_w, (float)tex_h_y);
	glUniform2f(loc_tex_size_uv, (float)tiled_w, (float)tex_h_uv);
	glUniform1f(loc_tiles_x,     (float)(tiled_w / 64));

	glVertexAttribPointer(loc_pos,     2, GL_FLOAT, GL_FALSE, 0, quad_pos);
	glVertexAttribPointer(loc_uv_attr, 2, GL_FLOAT, GL_FALSE, 0, quad_uv);
	glEnableVertexAttribArray(loc_pos);
	glEnableVertexAttribArray(loc_uv_attr);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

/* ----------------------------------------------------------- VIDC + EGLImage */

static int vfd;
static struct cap_slot {
	int      dma_fd;
	void    *map;
	size_t   map_len;
	EGLImage img_y, img_uv;
	GLuint   tex_y, tex_uv;
} cap[MAX_CAP];
static unsigned cap_count;

/*
 * Import one byte-plane of the VIDC dma-buf as a single-channel R8
 * texture (so the shader gets raw bytes, no Mesa YUV magic). The whole
 * thing is treated as linear — the de-tile shader does the tile lookup.
 */
static EGLImage import_dmabuf_plane(int dma_fd, unsigned w, unsigned h,
				    unsigned stride, unsigned offset)
{
	uint64_t mod = DRM_FORMAT_MOD_LINEAR;
	EGLint attr[] = {
		EGL_WIDTH,                          (EGLint)w,
		EGL_HEIGHT,                         (EGLint)h,
		EGL_LINUX_DRM_FOURCC_EXT,           DRM_FORMAT_R8,
		EGL_DMA_BUF_PLANE0_FD_EXT,          dma_fd,
		EGL_DMA_BUF_PLANE0_OFFSET_EXT,      (EGLint)offset,
		EGL_DMA_BUF_PLANE0_PITCH_EXT,       (EGLint)stride,
		EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, (EGLint)(mod & 0xffffffff),
		EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, (EGLint)(mod >> 32),
		EGL_NONE
	};
	EGLImage img = pf_eglCreateImageKHR(egl_dpy, EGL_NO_CONTEXT,
					    EGL_LINUX_DMA_BUF_EXT, NULL, attr);
	if (img == EGL_NO_IMAGE_KHR)
		fprintf(stderr, "eglCreateImageKHR plane failed (egl err 0x%x)\n",
			eglGetError());
	return img;
}

static void dec_open(const char *path)
{
	vfd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (vfd < 0) die("open decoder");

	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	f.fmt.pix_mp.width = 320; f.fmt.pix_mp.height = 240;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f.fmt.pix_mp.num_planes = 1;
	f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
	if (ioctl(vfd, VIDIOC_S_FMT, &f)) die("S_FMT OUT");

	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	fc.fmt.pix_mp.width = 320; fc.fmt.pix_mp.height = 240;
	fc.fmt.pix_mp.pixelformat = v4l2_fourcc('T','M','1','2');
	fc.fmt.pix_mp.num_planes = 1;
	if (ioctl(vfd, VIDIOC_S_FMT, &fc)) die("S_FMT CAP");

	struct v4l2_event_subscription es = { .type = V4L2_EVENT_SOURCE_CHANGE };
	ioctl(vfd, VIDIOC_SUBSCRIBE_EVENT, &es);
}

static unsigned out_bufs;
static void *out_mmap[4]; static size_t out_size[4]; static int out_avail[4];

static void dec_setup_output(void)
{
	struct v4l2_requestbuffers rb = { .count = 4,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(vfd, VIDIOC_REQBUFS, &rb)) die("REQBUFS OUT");
	out_bufs = rb.count;
	for (unsigned i = 0; i < out_bufs; i++) {
		struct v4l2_plane pl = {0};
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		if (ioctl(vfd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF OUT");
		out_size[i] = pl.length;
		out_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE,
				   MAP_SHARED, vfd, pl.m.mem_offset);
		if (out_mmap[i] == MAP_FAILED) die("mmap OUT");
		out_avail[i] = 1;
	}
}

static void dec_after_src_change(void)
{
	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	if (ioctl(vfd, VIDIOC_G_FMT, &fc)) die("G_FMT CAP");
	width  = fc.fmt.pix_mp.width;
	height = fc.fmt.pix_mp.height;
	dec_bytesperline = fc.fmt.pix_mp.plane_fmt[0].bytesperline;
	{
		unsigned stride = dec_bytesperline ? dec_bytesperline
						   : ((width + 127) & ~127u);
		unsigned y = stride * ((height + 31) & ~31u);
		dec_chroma_off = (y + 8191u) & ~8191u;
	}
	fprintf(stderr, "VIDC CAP: %ux%u bpl=%u chroma_off=%u\n",
		width, height, dec_bytesperline, dec_chroma_off);

	struct v4l2_requestbuffers rb = { .count = MAX_CAP,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(vfd, VIDIOC_REQBUFS, &rb)) die("REQBUFS CAP");
	cap_count = rb.count;
	if (cap_count > MAX_CAP) cap_count = MAX_CAP;

	for (unsigned i = 0; i < cap_count; i++) {
		struct v4l2_plane pl = {0};
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		if (ioctl(vfd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF CAP");
		cap[i].map_len = pl.length;
		cap[i].map = mmap(NULL, pl.length, PROT_READ, MAP_SHARED, vfd,
				  pl.m.mem_offset);
		if (cap[i].map == MAP_FAILED) cap[i].map = NULL;

		struct v4l2_exportbuffer eb = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.index = i, .plane = 0, .flags = O_CLOEXEC };
		if (ioctl(vfd, VIDIOC_EXPBUF, &eb)) die("EXPBUF");
		cap[i].dma_fd = eb.fd;

#ifdef CPU_UPLOAD
		/* CPU upload variant: allocate at NEXT POWER-OF-TWO HEIGHT,
		 * since A220 may round NPOT texture height internally. We
		 * then advertise the padded height as u_tex_size_y.y so
		 * the shader's src_y / tex_size.y math hits the right
		 * physical row. */
		unsigned tex_h_y_pot = 1;
		while (tex_h_y_pot < ((height + 31) & ~31u)) tex_h_y_pot <<= 1;
		unsigned tex_h_uv_pot = tex_h_y_pot / 2;
		(void)cap[i].img_y; (void)cap[i].img_uv;
		glGenTextures(1, &cap[i].tex_y);
		glBindTexture(GL_TEXTURE_2D, cap[i].tex_y);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
			     dec_bytesperline, tex_h_y_pot, 0,
			     GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL);
		glGenTextures(1, &cap[i].tex_uv);
		glBindTexture(GL_TEXTURE_2D, cap[i].tex_uv);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
			     dec_bytesperline, tex_h_uv_pot, 0,
			     GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL);
		fprintf(stderr, "  slot %u: (CPU upload POT) tex_y=%u tex_uv=%u "
			"phys=%ux%u (y) %ux%u (uv)\n",
			i, cap[i].tex_y, cap[i].tex_uv,
			dec_bytesperline, tex_h_y_pot,
			dec_bytesperline, tex_h_uv_pot);
#else
		/* A220 NPOT linear-texture sampling has a quirk that re-introduces
		 * the every-other-64-px-column stagger on the de-tile shader when
		 * the imported texture height is non-power-of-2 (the precision
		 * highp sampler2D fix resolves it for POT CPU-uploaded textures
		 * but not for NPOT dma-buf imports). Pad the import height to
		 * next-POT. The de-tile shader's max src_y is bounded by the tile
		 * count (~2 * tiles_total) which is well below the POT height,
		 * so the padded rows (768..pot) are never actually sampled. */
		unsigned tex_h_y_pot = 1;
		while (tex_h_y_pot < ((height + 31) & ~31u)) tex_h_y_pot <<= 1;
		unsigned tex_h_uv_pot = tex_h_y_pot / 2;
		/* Y plane: padded to POT, stride remains bytesperline. */
		cap[i].img_y = import_dmabuf_plane(cap[i].dma_fd,
						   dec_bytesperline,
						   tex_h_y_pot,
						   dec_bytesperline, 0);
		/* UV plane: padded to POT/2. */
		cap[i].img_uv = import_dmabuf_plane(cap[i].dma_fd,
						    dec_bytesperline,
						    tex_h_uv_pot,
						    dec_bytesperline,
						    dec_chroma_off);
		if (cap[i].img_y == EGL_NO_IMAGE_KHR ||
		    cap[i].img_uv == EGL_NO_IMAGE_KHR) {
			fprintf(stderr, "slot %u: R8 plane import failed\n", i);
			if (saved_crtc)
				drmModeSetCrtc(drm_fd, saved_crtc->crtc_id,
					       saved_crtc->buffer_id,
					       saved_crtc->x, saved_crtc->y,
					       &conn_id, 1, &saved_crtc->mode);
			exit(1);
		}
		glGenTextures(1, &cap[i].tex_y);
		glBindTexture(GL_TEXTURE_2D, cap[i].tex_y);
		pf_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, cap[i].img_y);
		glGenTextures(1, &cap[i].tex_uv);
		glBindTexture(GL_TEXTURE_2D, cap[i].tex_uv);
		pf_glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, cap[i].img_uv);
		fprintf(stderr, "  slot %u: dma_fd=%d tex_y=%u tex_uv=%u\n",
			i, cap[i].dma_fd, cap[i].tex_y, cap[i].tex_uv);
#endif

		struct v4l2_plane qp = {0};
		struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &qp, .length = 1 };
		if (ioctl(vfd, VIDIOC_QBUF, &q)) die("QBUF CAP");
	}
	int t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("STREAMON CAP");
}

/* -------------------------------------------------------------- present BO */

struct bo_fb { struct gbm_bo *bo; uint32_t fb_id; };

static struct bo_fb *bo_to_fb(struct gbm_bo *bo)
{
	struct bo_fb *r = gbm_bo_get_user_data(bo);
	if (r) return r;
	r = calloc(1, sizeof(*r));
	r->bo = bo;
	uint32_t w = gbm_bo_get_width(bo);
	uint32_t h = gbm_bo_get_height(bo);
	uint32_t stride = gbm_bo_get_stride(bo);
	uint32_t handle = gbm_bo_get_handle(bo).u32;
	uint32_t handles[4] = { handle, 0, 0, 0 };
	uint32_t strides[4] = { stride, 0, 0, 0 };
	uint32_t offsets[4] = { 0, 0, 0, 0 };
	if (drmModeAddFB2(drm_fd, w, h, DRM_FORMAT_XRGB8888,
			  handles, strides, offsets, &r->fb_id, 0))
		die("AddFB2 GBM bo");
	gbm_bo_set_user_data(bo, r, NULL);
	return r;
}

static int flip_pending;
static void flip_cb(int fd, unsigned seq, unsigned s, unsigned us,
		    unsigned cid, void *data)
{ (void)fd;(void)seq;(void)s;(void)us;(void)cid;(void)data; flip_pending = 0; }

static void drain_flip(int timeout_ms)
{
	if (!flip_pending) return;
	struct pollfd pfd = { .fd = drm_fd, .events = POLLIN };
	if (poll(&pfd, 1, timeout_ms) <= 0) return;
	drmEventContext ev = { .version = 3, .page_flip_handler2 = flip_cb };
	drmHandleEvent(drm_fd, &ev);
}

static void present(struct gbm_bo *bo)
{
	struct bo_fb *r = bo_to_fb(bo);
	drmModeAtomicReq *req = drmModeAtomicAlloc();
	#define ADD(p, v) drmModeAtomicAddProperty(req, primary_id, p, v)
	ADD(p_fb_id,   r->fb_id);
	ADD(p_crtc_id, crtc_id);
	ADD(p_src_x,   0);
	ADD(p_src_y,   0);
	ADD(p_src_w,   (uint64_t)mode.hdisplay << 16);
	ADD(p_src_h,   (uint64_t)mode.vdisplay << 16);
	ADD(p_crtc_x,  0);
	ADD(p_crtc_y,  0);
	ADD(p_crtc_w,  mode.hdisplay);
	ADD(p_crtc_h,  mode.vdisplay);
	#undef ADD
	uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT;
	int ret = drmModeAtomicCommit(drm_fd, req, flags, NULL);
	if (ret == -EBUSY) { drain_flip(50);
		ret = drmModeAtomicCommit(drm_fd, req, flags, NULL); }
	if (!ret) flip_pending = 1;
	else fprintf(stderr, "AtomicCommit: %s\n", strerror(-ret));
	drmModeAtomicFree(req);
}

/* ---------------------------------------------------------------- AU walker */

static size_t *au_off; static size_t au_n;
static unsigned char *in_data; static off_t in_total;

static void build_au_table(const char *path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) die("open input");
	in_total = lseek(fd, 0, SEEK_END); lseek(fd, 0, SEEK_SET);
	in_data = malloc(in_total);
	if (read(fd, in_data, in_total) != in_total) die("read input");
	close(fd);
	size_t cap = 2048;
	au_off = malloc(cap * sizeof(*au_off));
	int in_au = 0;
	for (off_t i = 0; i + 4 < in_total; i++) {
		int sc = 0; size_t off_nal = 0;
		if (!in_data[i] && !in_data[i+1] && !in_data[i+2] &&
		    in_data[i+3] == 1) { sc = 4; off_nal = i + 4; }
		else if (!in_data[i] && !in_data[i+1] && in_data[i+2] == 1)
			{ sc = 3; off_nal = i + 3; }
		if (!sc) continue;
		int nt = in_data[off_nal] & 0x1f, vcl = (nt >= 1 && nt <= 5);
		if (!in_au) {
			if (au_n == cap) { cap *= 2;
				au_off = realloc(au_off, cap * sizeof(*au_off)); }
			au_off[au_n++] = i;
			in_au = 1;
		}
		if (vcl) in_au = 0;
		i += sc - 1;
	}
	if (!au_n) { au_off[0] = 0; au_n = 1; }
	fprintf(stderr, "Built %zu AUs from %lld bytes\n",
		au_n, (long long)in_total);
}

/* ----------------------------------------------------------------- main */

int main(int argc, char **argv)
{
	const char *in_path = argc > 1 ? argv[1] : "/tmp/in.264";
	const char *card    = argc > 2 ? argv[2] : "/dev/dri/card0";
	const char *vdev    = argc > 3 ? argv[3] : "/dev/video6";

	drm_init(card);
	egl_init();
	gles_init();
	dec_open(vdev);
	dec_setup_output();
	build_au_table(in_path);

	/* prime first AU and STREAMON OUTPUT */
	size_t next_au = 0;
	{
		size_t s = au_off[0];
		size_t e = (au_n > 1) ? au_off[1] : (size_t)in_total;
		size_t len = e - s; if (len > out_size[0]) len = out_size[0];
		memcpy(out_mmap[0], in_data + s, len);
		struct v4l2_plane pl = { .bytesused = len };
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = 0,
			.m.planes = &pl, .length = 1 };
		ioctl(vfd, VIDIOC_QBUF, &b);
		out_avail[0] = 0; next_au++;
	}
	int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("STREAMON OUT");

	int got_src = 0, loops = 0;
	while (!got_src && loops++ < 30) {
		struct pollfd pfd = { .fd = vfd, .events = POLLPRI|POLLOUT };
		if (poll(&pfd, 1, 1000) < 0) die("poll src_change");
		if (pfd.revents & POLLPRI) {
			struct v4l2_event ev;
			while (ioctl(vfd, VIDIOC_DQEVENT, &ev) == 0)
				if (ev.type == V4L2_EVENT_SOURCE_CHANGE) got_src = 1;
		}
		if (pfd.revents & POLLOUT) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0)
				out_avail[b.index] = 1;
		}
	}
	if (!got_src) die("no SOURCE_CHANGE");
	dec_after_src_change();

	/* bootstrap remaining OUT slots */
	for (unsigned i = 0; i < out_bufs && next_au < au_n; i++) {
		if (!out_avail[i]) continue;
		size_t s = au_off[next_au];
		size_t e = (next_au + 1 < au_n) ? au_off[next_au+1]
						: (size_t)in_total;
		size_t len = e - s; if (len > out_size[i]) len = out_size[i];
		memcpy(out_mmap[i], in_data + s, len);
		struct v4l2_plane pl = { .bytesused = len };
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		if (!ioctl(vfd, VIDIOC_QBUF, &b)) { out_avail[i] = 0; next_au++; }
	}

	/* Pre-fill the primary plane with a black BO before the first VIDC
	 * frame arrives. Otherwise the panel scans out whatever was in the
	 * GBM front-buffer at allocation (Mesa/GBM doesn't zero its BOs)
	 * which is BT.601-converted from NV12 (Y=0, UV=0) ≈ (R=0, G=135, B=0)
	 * — bright green — for the first ~second until the decoder warms up. */
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	eglSwapBuffers(egl_dpy, egl_surf);
	{
		struct gbm_bo *bo = gbm_surface_lock_front_buffer(gbm_surf);
		if (bo) {
			present(bo);
			drain_flip(50);
			gbm_surface_release_buffer(gbm_surf, bo);
		}
	}

	struct timespec t0, tlast;
	clock_gettime(CLOCK_MONOTONIC, &t0); tlast = t0;
	size_t frames = 0, frames_last = 0;
	unsigned idle = 0;
	while (idle < 8) {
		struct pollfd pfd = { .fd = vfd,
			.events = POLLIN | POLLOUT | POLLPRI };
		int pr = poll(&pfd, 1, 500);
		if (pr < 0) die("main poll");
		if (pr == 0) { idle++; continue; }
		idle = 0;

		if (pfd.revents & POLLIN) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0 && pl.bytesused) {
				/* Render GLES quad sampling the just-decoded slot. */
				gles_draw(cap[b.index].tex_y,
					  cap[b.index].tex_uv
#ifdef CPU_UPLOAD
					  , cap[b.index].map
#endif
					);
				/* Capture frame 30 (mid-clip) for analysis. */
				if (frames == 30) {
					int rgba_size = mode.hdisplay *
							mode.vdisplay * 4;
					unsigned char *rgba = malloc(rgba_size);
					glReadPixels(0, 0, mode.hdisplay,
						     mode.vdisplay, GL_RGBA,
						     GL_UNSIGNED_BYTE, rgba);
					int fd = open("/tmp/vidc_gl_frame.rgba",
						      O_WRONLY|O_CREAT|O_TRUNC,
						      0644);
					if (fd >= 0) {
						ssize_t n = write(fd, rgba, rgba_size);
						(void)n;
						close(fd);
						fprintf(stderr,
							"captured frame 30 -> /tmp/vidc_gl_frame.rgba (%dx%d)\n",
							mode.hdisplay,
							mode.vdisplay);
					}
					free(rgba);
				}
				eglSwapBuffers(egl_dpy, egl_surf);
				struct gbm_bo *bo =
					gbm_surface_lock_front_buffer(gbm_surf);
				if (bo) {
					drain_flip(50);
					present(bo);
					/* GBM holds at most ~2 BOs alive; release
					 * the previous lock so it can be reused. */
					static struct gbm_bo *prev_bo;
					if (prev_bo)
						gbm_surface_release_buffer(gbm_surf, prev_bo);
					prev_bo = bo;
				}
				frames++;
				/* Recycle the VIDC CAPTURE slot. The GPU has
				 * sampled it during gles_draw (synchronous
				 * draw before eglSwapBuffers issues commit). */
				struct v4l2_plane qp = {0};
				struct v4l2_buffer rq = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
					.memory = V4L2_MEMORY_MMAP, .index = b.index,
					.m.planes = &qp, .length = 1 };
				ioctl(vfd, VIDIOC_QBUF, &rq);

				struct timespec tn;
				clock_gettime(CLOCK_MONOTONIC, &tn);
				double dt = (tn.tv_sec - tlast.tv_sec) +
				            (tn.tv_nsec - tlast.tv_nsec) / 1e9;
				if (dt >= 1.0) {
					double tdt = (tn.tv_sec - t0.tv_sec) +
					             (tn.tv_nsec - t0.tv_nsec) / 1e9;
					fprintf(stderr,
					    "  [%.1fs] %.1f fps (avg %.1f)\n",
					    tdt, (frames - frames_last) / dt,
					    frames / tdt);
					tlast = tn; frames_last = frames;
				}
			}
		}
		if (pfd.revents & POLLOUT) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0) {
				out_avail[b.index] = 1;
				if (next_au < au_n) {
					size_t s = au_off[next_au];
					size_t e = (next_au + 1 < au_n)
					           ? au_off[next_au + 1]
					           : (size_t)in_total;
					size_t len = e - s;
					if (len > out_size[b.index])
						len = out_size[b.index];
					memcpy(out_mmap[b.index], in_data + s, len);
					struct v4l2_plane qp = { .bytesused = len };
					struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
						.memory = V4L2_MEMORY_MMAP, .index = b.index,
						.m.planes = &qp, .length = 1 };
					if (!ioctl(vfd, VIDIOC_QBUF, &q))
						{ out_avail[b.index] = 0; next_au++; }
				}
			}
		}
	}

	fprintf(stderr, "vidc_gl_play: %zu frames\n", frames);

	if (saved_crtc) {
		drmModeSetCrtc(drm_fd, saved_crtc->crtc_id, saved_crtc->buffer_id,
			       saved_crtc->x, saved_crtc->y, &conn_id, 1,
			       &saved_crtc->mode);
		drmModeFreeCrtc(saved_crtc);
	}
	close(drm_fd);
	close(vfd);
	return 0;
}
