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
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), verts);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), verts + 2);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glFinish();

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
