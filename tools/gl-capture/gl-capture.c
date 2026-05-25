/*
 * gl-capture - off-screen GLES2 render to PNG-like raw RGBA via DRM render node
 *
 * Renders a fixed test scene to a 1024x768 GBM/EGL pbuffer-equivalent on
 * /dev/dri/renderD128 (no DRM master needed - runs alongside LSM), reads
 * pixels with glReadPixels, writes raw RGBA8888 to stdout.
 *
 * Build (host, cross):
 *   arm-linux-gnueabihf-gcc -O2 gl-capture.c \
 *     --sysroot=/path/to/yocto/cortexa8t2hf-neon \
 *     -lEGL -lGLESv2 -lgbm -ldrm -o gl-capture
 *
 * Run (device):
 *   ./gl-capture > /tmp/capture.bin     # 1024*768*4 = 3145728 bytes
 *
 * Convert to PNG (host):
 *   convert -size 1024x768 -depth 8 RGBA:capture.bin capture.png
 *
 * The rendered scene is intentionally simple and DETERMINISTIC so two runs
 * should produce byte-identical output unless a state leak corrupts it:
 *   - Cleared to known background (R=0.10, G=0.20, B=0.30, A=1.0)
 *   - One full-screen triangle with per-vertex colors (red, green, blue)
 *   - Fragment shader does smooth interpolation only (no branching, no loops)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#define W 1024
#define H 768

#define DIE(fmt, ...) do { fprintf(stderr, "ERROR: " fmt "\n", ##__VA_ARGS__); exit(1); } while (0)

static const char *VS_SRC =
    "attribute vec2 a_pos;\n"
    "attribute vec3 a_color;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  v_color = a_color;\n"
    "  gl_Position = vec4(a_pos, 0.0, 1.0);\n"
    "}\n";

static const char *FS_SRC =
    "precision mediump float;\n"
    "varying vec3 v_color;\n"
    "void main() { gl_FragColor = vec4(v_color, 1.0); }\n";

/* Cube VS: takes pre-rotated/projected clip-space xyz (vec3), z carries depth. */
static const char *VS_CUBE_SRC =
    "attribute vec3 a_pos;\n"
    "attribute vec3 a_color;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  v_color = a_color;\n"
    "  gl_Position = vec4(a_pos, 1.0);\n"
    "}\n";

static GLuint compile_shader(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024]; GLsizei len = 0;
        glGetShaderInfoLog(s, sizeof(log), &len, log);
        fprintf(stderr, "shader compile failed: %.*s\n", len, log);
        exit(1);
    }
    return s;
}

int main(int argc, char **argv) {
    const char *node = (argc > 1) ? argv[1] : "/dev/dri/renderD128";
    int fd = open(node, O_RDWR | O_CLOEXEC);
    if (fd < 0) DIE("open(%s): %s", node, strerror(errno));

    struct gbm_device *gbm = gbm_create_device(fd);
    if (!gbm) DIE("gbm_create_device");

    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
        (void*)eglGetProcAddress("eglGetPlatformDisplayEXT");
    EGLDisplay dpy = eglGetPlatformDisplayEXT
        ? eglGetPlatformDisplayEXT(EGL_PLATFORM_GBM_KHR, gbm, NULL)
        : eglGetDisplay((EGLNativeDisplayType)gbm);
    if (dpy == EGL_NO_DISPLAY) DIE("eglGetDisplay");
    if (!eglInitialize(dpy, NULL, NULL)) DIE("eglInitialize: 0x%x", eglGetError());

    eglBindAPI(EGL_OPENGL_ES_API);

    /* Render-node EGL on Mesa is surfaceless. Use surfaceless ctx + FBO. */
    EGLint cfg_attr[] = {
        EGL_SURFACE_TYPE, EGL_DONT_CARE,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };
    EGLConfig cfg; EGLint ncfg = 0;
    if (!eglChooseConfig(dpy, cfg_attr, &cfg, 1, &ncfg) || ncfg < 1)
        DIE("eglChooseConfig: 0x%x ncfg=%d", eglGetError(), ncfg);

    EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
    if (ctx == EGL_NO_CONTEXT) DIE("eglCreateContext: 0x%x", eglGetError());

    if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx))
        DIE("eglMakeCurrent (surfaceless): 0x%x", eglGetError());

    /* Set up an FBO with renderbuffer to render into */
    GLuint fbo = 0, rbo = 0;
    glGenFramebuffers(1, &fbo);
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, W, H);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, rbo);
    /* depth renderbuffer (for FD_CUBE depth-tested mode).
     * FD_DEPTH24=1 selects 24-bit depth (GL_DEPTH_COMPONENT24_OES) instead of
     * the default 16-bit. Used to test whether the A22X GMEM depth fast-clear
     * stripe noise is specific to the 16-bit-depth fast-clear geometry (Z16
     * packs two depths per 32-bit color-clear cell; Z24 is one-per-cell). */
    GLuint dbo = 0;
    glGenRenderbuffers(1, &dbo);
    glBindRenderbuffer(GL_RENDERBUFFER, dbo);
    GLenum depthfmt = GL_DEPTH_COMPONENT16;
    if (getenv("FD_DEPTH24")) {
        depthfmt = 0x81A6; /* GL_DEPTH_COMPONENT24_OES */
        fprintf(stderr, "DEPTH: 24-bit (GL_DEPTH_COMPONENT24_OES)\n");
    } else {
        fprintf(stderr, "DEPTH: 16-bit (GL_DEPTH_COMPONENT16)\n");
    }
    glRenderbufferStorage(GL_RENDERBUFFER, depthfmt, W, H);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, dbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    GLenum st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (st != GL_FRAMEBUFFER_COMPLETE) {
        /* Try GL_RGBA4 which is core-required */
        glRenderbufferStorage(GL_RENDERBUFFER, 0x8056 /* GL_RGBA4 */, W, H);
        st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (st != GL_FRAMEBUFFER_COMPLETE)
            DIE("FBO incomplete 0x%x", st);
    }

    fprintf(stderr, "GL_VENDOR:   %s\n", glGetString(GL_VENDOR));
    fprintf(stderr, "GL_RENDERER: %s\n", glGetString(GL_RENDERER));
    fprintf(stderr, "GL_VERSION:  %s\n", glGetString(GL_VERSION));

    /* Build program */
    GLuint vs = compile_shader(GL_VERTEX_SHADER, VS_SRC);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, FS_SRC);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glBindAttribLocation(prog, 0, "a_pos");
    glBindAttribLocation(prog, 1, "a_color");
    glLinkProgram(prog);
    GLint ok = 0; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; GLsizei len = 0;
        glGetProgramInfoLog(prog, sizeof(log), &len, log);
        fprintf(stderr, "link failed: %.*s\n", len, log);
        return 1;
    }
    glUseProgram(prog);

    /* One triangle covering most of the viewport, with per-vertex colors:
     *   bottom-left red, bottom-right green, top-center blue.
     * Smooth interpolation produces a known continuous gradient.
     */
    static const GLfloat verts[] = {
        /* x      y     r    g    b */
        -0.9f, -0.9f,  1.0f, 0.0f, 0.0f,
         0.9f, -0.9f,  0.0f, 1.0f, 0.0f,
         0.0f,  0.9f,  0.0f, 0.0f, 1.0f,
    };

    glViewport(0, 0, W, H);
    glClearColor(0.10f, 0.20f, 0.30f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    /*
     * FD_CUBE=1: depth-tested, back-face-culled, smooth-shaded 3D cube,
     * each of the 6 faces drawn as a SEPARATE glDrawArrays in one submit.
     * This mirrors kmscube/glmark "gears"/"build"/"shading": multi-draw +
     * depth + culling + smooth per-vertex colour. If period-8 leaks into a
     * depth-tested draw, a face comes out FACETED (flat colour) or BLACK
     * (stale/uninitialised param bank) - the exact intermittent symptom
     * seen on real 3D content. Deterministic (fixed rotation).
     */
    /*
     * FD_GRID=G: one BIG depth-tested draw - a GxG grid of quads (G*G*6 verts,
     * e.g. G=32 -> 6144 verts) filling the screen, each vertex coloured and
     * z-displaced by position so depth testing is exercised. Single glDrawArrays.
     * Purpose: a high-vertex-count depth-tested draw to test whether a
     * TRUNCATED warmup (FD_A22X_WARMUP_MAXVERTS) still primes the param cache
     * correctly - if a few-vert warmup keeps this correct, big-mesh 3D FPS is
     * recovered. Deterministic.
     */
    const char *egrid = getenv("FD_GRID");
    if (egrid && atoi(egrid) > 1) {
        int G = atoi(egrid);
        GLuint gvs = compile_shader(GL_VERTEX_SHADER, VS_CUBE_SRC);
        GLuint gprog = glCreateProgram();
        glAttachShader(gprog, gvs);
        glAttachShader(gprog, fs);
        glBindAttribLocation(gprog, 0, "a_pos");
        glBindAttribLocation(gprog, 1, "a_color");
        glLinkProgram(gprog);
        glUseProgram(gprog);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glClearDepthf(1.0f);
        glClear(GL_DEPTH_BUFFER_BIT);
        int nverts = G * G * 6;
        GLfloat *gv = malloc((size_t)nverts * 6 * sizeof(GLfloat));
        if (!gv) DIE("malloc grid");
        int o = 0;
        for (int gy = 0; gy < G; gy++) for (int gx = 0; gx < G; gx++) {
            float x0 = -0.95f + 1.9f * gx / G,     x1 = -0.95f + 1.9f * (gx + 1) / G;
            float y0 = -0.95f + 1.9f * gy / G,     y1 = -0.95f + 1.9f * (gy + 1) / G;
            /* z displaced by a deterministic function of cell -> depth varies */
            float z = 0.4f * sinf(gx * 0.5f) * cosf(gy * 0.5f);
            float q[4][2] = {{x0,y0},{x1,y0},{x1,y1},{x0,y1}};
            int idx[6] = {0,1,2,0,2,3};
            for (int t = 0; t < 6; t++) {
                float px = q[idx[t]][0], py = q[idx[t]][1];
                gv[o++] = px; gv[o++] = py; gv[o++] = z;
                gv[o++] = (px+1)*0.5f; gv[o++] = (py+1)*0.5f; gv[o++] = (z+1)*0.5f;
            }
        }
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), gv);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), gv+3);
        glDrawArrays(GL_TRIANGLES, 0, nverts);
        free(gv);
        glDisable(GL_DEPTH_TEST);
        fprintf(stderr, "GRID: %dx%d = %d verts, one depth-tested draw\n", G, G, nverts);
        glFinish();
        goto readback;
    }

    const char *ecube = getenv("FD_CUBE");
    if (ecube && atoi(ecube) > 0) {
        GLuint cvs = compile_shader(GL_VERTEX_SHADER, VS_CUBE_SRC);
        GLuint cprog = glCreateProgram();
        glAttachShader(cprog, cvs);
        glAttachShader(cprog, fs);
        glBindAttribLocation(cprog, 0, "a_pos");
        glBindAttribLocation(cprog, 1, "a_color");
        glLinkProgram(cprog);
        glUseProgram(cprog);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        glClearDepthf(1.0f);
        glClear(GL_DEPTH_BUFFER_BIT);

        /* 6 faces, 4 CCW-from-outside corners each */
        static const float faces[6][4][3] = {
            {{1,-1,1},{1,-1,-1},{1,1,-1},{1,1,1}},     /* +X */
            {{-1,-1,-1},{-1,-1,1},{-1,1,1},{-1,1,-1}}, /* -X */
            {{-1,1,1},{1,1,1},{1,1,-1},{-1,1,-1}},     /* +Y */
            {{-1,-1,-1},{1,-1,-1},{1,-1,1},{-1,-1,1}}, /* -Y */
            {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}},     /* +Z */
            {{1,-1,-1},{-1,-1,-1},{-1,1,-1},{1,1,-1}}, /* -Z */
        };
        const float ax = 0.5f, ay = 0.6f, s = 0.55f;
        const float ca = cosf(ax), sa = sinf(ax), cb = cosf(ay), sb = sinf(ay);
        for (int f = 0; f < 6; f++) {
            /* project the 4 corners (Ry then Rx, ortho scale) */
            float p[4][3], col[4][3];
            for (int v = 0; v < 4; v++) {
                float x = faces[f][v][0], y = faces[f][v][1], z = faces[f][v][2];
                float x1 = x * cb + z * sb, z1 = -x * sb + z * cb, y1 = y;
                float y2 = y1 * ca - z1 * sa, z2 = y1 * sa + z1 * ca, x2 = x1;
                p[v][0] = x2 * s; p[v][1] = y2 * s; p[v][2] = z2 * s;
                col[v][0] = (faces[f][v][0] + 1) * 0.5f;
                col[v][1] = (faces[f][v][1] + 1) * 0.5f;
                col[v][2] = (faces[f][v][2] + 1) * 0.5f;
            }
            /* two triangles (0,1,2) (0,2,3) -> 6 interleaved verts pos3+col3 */
            int idx[6] = {0, 1, 2, 0, 2, 3};
            GLfloat fv[6 * 6];
            for (int t = 0; t < 6; t++) {
                int c = idx[t];
                fv[t*6+0]=p[c][0]; fv[t*6+1]=p[c][1]; fv[t*6+2]=p[c][2];
                fv[t*6+3]=col[c][0]; fv[t*6+4]=col[c][1]; fv[t*6+5]=col[c][2];
            }
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), fv);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), fv+3);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        fprintf(stderr, "CUBE: 6 depth-tested culled faces, separate draws\n");
        glFinish();
        goto readback;
    }

    /*
     * FD_PULSAR=N: glmark2-pulsar repro (light=false, texture=false). N small
     * quads arranged radially, each a SEPARATE glDrawArrays, per-vertex colour,
     * additive blending, NO texture, NO depth - matching pulsar's profile.
     *
     * The point is BUFFER CHURN: with FD_FRAMES=F (default 1) the scene is
     * rendered F times, and unless FD_NOCHURN=1 a FRESH VBO is glGenBuffers'd,
     * filled, drawn and glDeleteBuffers'd every frame. That hammers the a2xx
     * gpummu map()/unmap() path (the freed VBO's GPU VA is unmapped, the next
     * frame's VBO reuses VAs) and forces a per-frame submit (glFinish). This is
     * the off-device, deterministic stand-in for "pulsar hangs the device":
     * if a GPU client fetches GPU VA 0 (null base) under churn, it should fault
     * here too. Try e.g. FD_PULSAR=5 FD_FRAMES=2000.
     */
    const char *epulsar = getenv("FD_PULSAR");
    if (epulsar) {
        int nq = atoi(epulsar); if (nq < 1) nq = 5; /* glmark2 default quads=5 */
        int frames = getenv("FD_FRAMES") ? atoi(getenv("FD_FRAMES")) : 1;
        if (frames < 1) frames = 1;
        int churn = getenv("FD_NOCHURN") ? 0 : 1;
        int nverts = nq * 6;
        GLfloat *pv = malloc((size_t)nverts * 5 * sizeof(GLfloat));
        if (!pv) DIE("malloc pulsar");

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE); /* additive, like pulsar */

        GLuint vbo = 0;
        if (!churn) glGenBuffers(1, &vbo);

        for (int fr = 0; fr < frames; fr++) {
            float t = fr * 0.05f;
            int o = 0;
            for (int q = 0; q < nq; q++) {
                float ang = 2.0f * 3.14159265f * q / nq + t;
                float cx = 0.40f * cosf(ang), cy = 0.40f * sinf(ang);
                float ca = cosf(ang), sa = sinf(ang);
                const float hw = 0.25f, hh = 0.12f;
                float corner[4][2] = {{-hw,-hh},{hw,-hh},{hw,hh},{-hw,hh}};
                float col[3] = { 0.5f + 0.5f * sinf(ang),
                                 0.5f + 0.5f * cosf(ang * 1.3f),
                                 0.5f + 0.5f * sinf(ang * 0.7f) };
                int idx[6] = {0,1,2,0,2,3};
                for (int v = 0; v < 6; v++) {
                    float lx = corner[idx[v]][0], ly = corner[idx[v]][1];
                    pv[o++] = cx + lx * ca - ly * sa;
                    pv[o++] = cy + lx * sa + ly * ca;
                    pv[o++] = col[0]; pv[o++] = col[1]; pv[o++] = col[2];
                }
            }

            if (churn) glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, (size_t)nverts * 5 * sizeof(GLfloat),
                         pv, GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat), (void*)0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat),
                                  (void*)(2*sizeof(GLfloat)));

            glClear(GL_COLOR_BUFFER_BIT);
            for (int q = 0; q < nq; q++)
                glDrawArrays(GL_TRIANGLES, q * 6, 6); /* separate draw per quad */

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if (churn) glDeleteBuffers(1, &vbo); /* free -> gpummu unmap churn */
            glFinish(); /* per-frame submit */

            if (frames > 50 && (fr % 200) == 0)
                fprintf(stderr, "PULSAR: frame %d/%d\n", fr, frames);
        }
        if (!churn) glDeleteBuffers(1, &vbo);
        glDisable(GL_BLEND);
        free(pv);
        fprintf(stderr, "PULSAR: %d quads x %d frames, churn=%d (per-frame VBO alloc/free)\n",
                nq, frames, churn);
        goto readback;
    }

    /*
     * FD_MULTIDRAW=N: render N triangles in ONE frame/submit, each via a
     * SEPARATE glDrawArrays, tiled into N vertical columns across the width.
     * Each triangle carries the same RGB gradient, so a fully-correct render
     * is a deterministic row of N identical gradient triangles on the navy
     * clear. This exercises the multi-draw path: if later draws read stale
     * param-cache banks (period-8), their columns show wrong/rotated colours.
     * Unset / <=1 -> original single full-screen triangle.
     */
    const char *emd = getenv("FD_MULTIDRAW");
    int ndraw = emd ? atoi(emd) : 0;
    if (ndraw <= 1) {
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), verts);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), verts + 2);
        glDrawArrays(GL_TRIANGLES, 0, 3);
    } else {
        for (int d = 0; d < ndraw; d++) {
            /* column d spans NDC x in [-1 + 2d/N, -1 + 2(d+1)/N], small margin */
            float x0 = -1.0f + 2.0f * d / ndraw + 0.02f;
            float x1 = -1.0f + 2.0f * (d + 1) / ndraw - 0.02f;
            float xc = 0.5f * (x0 + x1);
            GLfloat tri[] = {
                x0, -0.9f,  1.0f, 0.0f, 0.0f,
                x1, -0.9f,  0.0f, 1.0f, 0.0f,
                xc,  0.9f,  0.0f, 0.0f, 1.0f,
            };
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), tri);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), tri + 2);
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }
        fprintf(stderr, "MULTIDRAW: %d separate draws in one submit\n", ndraw);
    }
    glFinish();

readback: ;
    /* Read back. glReadPixels origin is bottom-left so the buffer comes out
     * vertically flipped vs typical image coords. We flip rows here so the
     * resulting RGBA blob converts straight via:
     *   convert -size 1024x768 -depth 8 RGBA:cap.bin cap.png
     */
    uint8_t *pix = malloc(W * H * 4);
    if (!pix) DIE("malloc");
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pix);
    GLenum gle = glGetError();
    if (gle != GL_NO_ERROR) fprintf(stderr, "glReadPixels: 0x%x\n", gle);

    uint8_t *flipped = malloc(W * H * 4);
    if (!flipped) DIE("malloc");
    for (int y = 0; y < H; y++)
        memcpy(flipped + (H - 1 - y) * W * 4, pix + y * W * 4, W * 4);

    if (write(STDOUT_FILENO, flipped, W * H * 4) != W * H * 4)
        DIE("write: %s", strerror(errno));

    free(flipped); free(pix);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &rbo);
    eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(dpy, ctx);
    eglTerminate(dpy);
    gbm_device_destroy(gbm);
    close(fd);
    return 0;
}
