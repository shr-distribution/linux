/*
 * gl-readpixels-twice
 *
 * Render the standard test triangle once, then call glReadPixels TWICE
 * on the same FBO with no rendering in between. Compare the two byte
 * buffers. If they match, the rendered output is stable post-render
 * (variance is in the render itself or in pipeline state). If they
 * differ, there's a CPU/GPU coherency / readback race — the FBO
 * contents are still settling between calls.
 *
 * Output:
 *   stderr: "MATCH" or "DIFFER (offset=X)"
 *   /tmp/cap-A.bin   first glReadPixels result
 *   /tmp/cap-B.bin   second glReadPixels result
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
    int fd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
    if (fd < 0) DIE("open: %s", strerror(errno));

    struct gbm_device *gbm = gbm_create_device(fd);
    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
        (void*)eglGetProcAddress("eglGetPlatformDisplayEXT");
    EGLDisplay dpy = eglGetPlatformDisplayEXT
        ? eglGetPlatformDisplayEXT(EGL_PLATFORM_GBM_KHR, gbm, NULL)
        : eglGetDisplay((EGLNativeDisplayType)gbm);
    eglInitialize(dpy, NULL, NULL);
    eglBindAPI(EGL_OPENGL_ES_API);

    EGLint cfg_attr[] = {
        EGL_SURFACE_TYPE, EGL_DONT_CARE,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };
    EGLConfig cfg; EGLint ncfg = 0;
    eglChooseConfig(dpy, cfg_attr, &cfg, 1, &ncfg);
    EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
    eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx);

    GLuint fbo = 0, rbo = 0;
    glGenFramebuffers(1, &fbo);
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, W, H);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, rbo);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glRenderbufferStorage(GL_RENDERBUFFER, 0x8056, W, H);
    }

    GLuint vs = compile_shader(GL_VERTEX_SHADER, VS_SRC);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, FS_SRC);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glBindAttribLocation(prog, 0, "a_pos");
    glBindAttribLocation(prog, 1, "a_color");
    glLinkProgram(prog);
    glUseProgram(prog);

    static const GLfloat verts[] = {
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

    /* Two consecutive glReadPixels of the SAME FBO, no rendering in between.
     * Both reads should see the same already-rendered pixels. */
    uint8_t *A = malloc(W * H * 4);
    uint8_t *B = malloc(W * H * 4);
    if (!A || !B) DIE("malloc");

    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, A);
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, B);

    /* Compare */
    int match = (memcmp(A, B, W * H * 4) == 0);
    if (match) {
        fprintf(stderr, "MATCH (both reads identical)\n");
    } else {
        /* Find first byte that differs and count total diffs */
        int first = -1;
        size_t total_diffs = 0;
        for (size_t i = 0; i < W * H * 4; i++) {
            if (A[i] != B[i]) {
                if (first < 0) first = i;
                total_diffs++;
            }
        }
        fprintf(stderr, "DIFFER first_diff_offset=%d total_byte_diffs=%zu (%.2f%% of buffer)\n",
                first, total_diffs, (100.0 * total_diffs) / (W * H * 4));
        fprintf(stderr, "  at offset %d: A=%02x %02x %02x %02x  B=%02x %02x %02x %02x\n",
                first,
                A[first&~3], A[(first&~3)+1], A[(first&~3)+2], A[(first&~3)+3],
                B[first&~3], B[(first&~3)+1], B[(first&~3)+2], B[(first&~3)+3]);
    }

    /* Save both, flipped (so they convert to PNG cleanly) */
    uint8_t *flipA = malloc(W * H * 4);
    uint8_t *flipB = malloc(W * H * 4);
    for (int y = 0; y < H; y++) {
        memcpy(flipA + (H - 1 - y) * W * 4, A + y * W * 4, W * 4);
        memcpy(flipB + (H - 1 - y) * W * 4, B + y * W * 4, W * 4);
    }
    FILE *fa = fopen("/tmp/cap-A.bin", "wb"); fwrite(flipA, 1, W * H * 4, fa); fclose(fa);
    FILE *fb = fopen("/tmp/cap-B.bin", "wb"); fwrite(flipB, 1, W * H * 4, fb); fclose(fb);
    fprintf(stderr, "wrote /tmp/cap-A.bin /tmp/cap-B.bin\n");

    free(A); free(B); free(flipA); free(flipB);
    eglDestroyContext(dpy, ctx);
    eglTerminate(dpy);
    gbm_device_destroy(gbm);
    close(fd);
    return match ? 0 : 1;
}
