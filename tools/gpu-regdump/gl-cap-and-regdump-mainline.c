/*
 * gl-cap-and-regdump-mainline
 *
 * Mainline-side companion to gl-cap-and-regdump-webos. Renders the same
 * test triangle and dumps GPU register state in the same canonical
 * format - all in one process, with the regdump captured immediately
 * after glFinish() so we sample post-render state (not idle GPU).
 *
 * The mainline freedreno driver doesn't expose an MMIO regread ioctl.
 * Instead the msm DRM driver dumps registers via text in
 * /sys/kernel/debug/dri/0/{cp,rbbm,mh,tp,rb,sq,...}. This tool reads
 * those debugfs files inline and parses the "  NAME: HEXVALUE" lines
 * out of them, then prints them in the same "0xOFFSET  NAME  = 0xVAL"
 * format as gpu-regdump-webos so the two outputs can be diffed
 * directly.
 *
 * Build (cross with the Yocto sysroot):
 *   SR=/media/herrie/LuneOS/scarthgap/webos-ports/tmp-glibc/sysroots-components/cortexa8t2hf-neon
 *   arm-linux-gnueabihf-gcc -O2 gl-cap-and-regdump-mainline.c \
 *     -I $SR/mesa/usr/include -I $SR/libdrm/usr/include -I $SR/libdrm/usr/include/libdrm \
 *     -L $SR/mesa/usr/lib \
 *     -Wl,--allow-shlib-undefined,--unresolved-symbols=ignore-in-shared-libs \
 *     -lEGL -lGLESv2 -lgbm -o gl-cap-and-regdump-mainline
 *
 * Outputs:
 *   /tmp/cap.bin   - 1024x768 RGBA8 framebuffer
 *   /tmp/regs.txt  - register dump (canonical format, diffable against
 *                    the webOS-side regs.txt)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#define W 1024
#define H 768
#define CAP_PATH  "/tmp/cap.bin"
#define REGS_PATH "/tmp/regs.txt"

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

/* Each entry says: "in debugfs file <file>, look for a line containing
 * '<dbg_name>:'; emit it as `0x<off>  <name>  = 0xVAL`". */
struct reg_info {
    unsigned int offset;
    const char *name;
    const char *file;
    const char *dbg_name;
};

static const struct reg_info regs[] = {
    /* CP - /sys/kernel/debug/dri/0/cp */
    { 0x01C0, "CP_RB_BASE",         "cp",   "RB_BASE" },
    { 0x01C1, "CP_RB_CNTL",         "cp",   "RB_CNTL" },
    { 0x01C5, "CP_RB_RPTR",         "cp",   "RB_RPTR" },
    { 0x01C6, "CP_RB_WPTR",         "cp",   "RB_WPTR" },
    { 0x017E, "CP_INT_CNTL",        "cp",   "INT_CNTL" },
    { 0x01F4, "CP_INT_STATUS",      "cp",   "INT_STATUS" },
    { 0x0578, "SCRATCH_REG0",       "cp",   "SCRATCH_REG0" },
    { 0x0579, "SCRATCH_REG1",       "cp",   "SCRATCH_REG1" },
    { 0x057A, "SCRATCH_REG2",       "cp",   "SCRATCH_REG2" },
    { 0x057B, "SCRATCH_REG3",       "cp",   "SCRATCH_REG3" },
    { 0x057C, "SCRATCH_REG4",       "cp",   "SCRATCH_REG4" },
    { 0x057D, "SCRATCH_REG5",       "cp",   "SCRATCH_REG5" },
    { 0x057E, "SCRATCH_REG6",       "cp",   "SCRATCH_REG6" },
    { 0x057F, "SCRATCH_REG7",       "cp",   "SCRATCH_REG7" },
    /* RBBM */
    { 0x0058, "RBBM_PM_OVERRIDE1",  "rbbm", "PM_OVERRIDE1" },
    { 0x0059, "RBBM_PM_OVERRIDE2",  "rbbm", "PM_OVERRIDE2" },
    { 0x005C, "RBBM_DEBUG",         "rbbm", "DEBUG" },
    { 0x017F, "RBBM_STATUS",        "rbbm", "STATUS" },
    { 0x0061, "RBBM_INT_CNTL",      "rbbm", "INT_CNTL" },
    { 0x0062, "RBBM_INT_STATUS",    "rbbm", "INT_STATUS" },
    /* MH */
    { 0x0050, "MH_INTERRUPT_MASK",   "mh",  "INTERRUPT_MASK" },
    { 0x0051, "MH_INTERRUPT_STATUS", "mh",  "INTERRUPT_STATUS" },
    { 0x0052, "MH_AXI_ERROR",        "mh",  "AXI_ERROR" },
    { 0x040C, "MH_MMU_CONFIG",       "mh",  "MMU_CONFIG" },
    /* Misc */
    { 0x03B7, "MASTER_INT_SIGNAL",  "rbbm", "MASTER_INT_SIGNAL" },
    { 0x0E1E, "TP0_CHICKEN",        "tp",   "TP0_CHICKEN" },
    { 0x0F01, "RB_BC_CONTROL",      "rb",   "BC_CONTROL" },
    /* SQ debug — kernel `sq` debugfs already exposes these; the
     * matchers below pick them up from that file. Used to look for
     * binner-cycle phase indicators. */
    { 0x0D00, "SQ_GPR_MANAGEMENT",  "sq",   "GPR_MANAGEMENT" },
    { 0x0D02, "SQ_INST_STORE_MGMT", "sq",   "INST_STORE_MGMT" },
    { 0x0D05, "SQ_DEBUG_MISC",      "sq",   "DEBUG_MISC" },
    { 0x0DAE, "SQ_DEBUG_INPUT_FSM", "sq",   "DEBUG_INPUT_FSM" },
    { 0x0DAF, "SQ_DEBUG_CONST_MGR", "sq",   "DEBUG_CONST_MGR" },
    { 0x0DB0, "SQ_DEBUG_TP_FSM",    "sq",   "DEBUG_TP_FSM" },
    { 0x0DB1, "SQ_DEBUG_FSM_ALU_0", "sq",   "DEBUG_FSM_ALU_0" },
    { 0x0DB2, "SQ_DEBUG_FSM_ALU_1", "sq",   "DEBUG_FSM_ALU_1" },
    { 0x0DB3, "SQ_DEBUG_EXP_ALLOC", "sq",   "DEBUG_EXP_ALLOC" },
    { 0x0DB4, "SQ_DEBUG_PTR_BUFF",  "sq",   "DEBUG_PTR_BUFF" },
    { 0x0DB5, "SQ_DEBUG_GPR_VTX",   "sq",   "DEBUG_GPR_VTX" },
    { 0x0DB6, "SQ_DEBUG_GPR_PIX",   "sq",   "DEBUG_GPR_PIX" },
    /* VGT/VSC — kernel `vgt` debugfs exposes these.
     * VSC_PIPE_DATA_LENGTH after submit = binner's bytes-written per pipe.
     * Comparing across the 8 cycle hashes would localise cycle to binner. */
    { 0x2200, "VGT_CURRENT_BIN_MIN", "vgt",  "CURRENT_BIN_MIN" },
    { 0x2201, "VGT_CURRENT_BIN_MAX", "vgt",  "CURRENT_BIN_MAX" },
    { 0x0C01, "VSC_BIN_SIZE",       "vgt",  "VSC_BIN_SIZE" },
    { 0x0C00, "VSC_REG_0xC00",      "vgt",  "VSC_REG_0xC00" },
    /* VSC_PIPE[0..7] CONFIG/ADDR/LEN. The kernel formats these as
     * "VSC_PIPE[N]: CFG=... ADDR=... LEN=..." so we use a substring match.
     * find_reg_value() does prefix-match on dbg_name+":". The format line
     * starts with "VSC_PIPE[N]:" so we just match that. */
    { 0x0C06, "VSC_PIPE_0",         "vgt",  "VSC_PIPE[0]" },
    { 0x0C09, "VSC_PIPE_1",         "vgt",  "VSC_PIPE[1]" },
    { 0x0C0C, "VSC_PIPE_2",         "vgt",  "VSC_PIPE[2]" },
    { 0x0C0F, "VSC_PIPE_3",         "vgt",  "VSC_PIPE[3]" },
    { 0x0C12, "VSC_PIPE_4",         "vgt",  "VSC_PIPE[4]" },
    { 0x0C15, "VSC_PIPE_5",         "vgt",  "VSC_PIPE[5]" },
    { 0x0C18, "VSC_PIPE_6",         "vgt",  "VSC_PIPE[6]" },
    { 0x0C1B, "VSC_PIPE_7",         "vgt",  "VSC_PIPE[7]" },
    /* A22X-specific */
    { 0x2209, "A220_RB_LRZ_VSC_CONTROL", "rb", "LRZ_VSC_CONTROL" },
    { 0x2210, "A220_GRAS_CONTROL",  "rb",   "GRAS_CONTROL" },
    { 0, NULL, NULL, NULL }
};

/*
 * Read /sys/kernel/debug/dri/0/<file> looking for a line of the form
 *   "<whitespace><dbg_name>:<whitespace><HEX>"
 * Returns the parsed value, or -1 if not found. Uses uint64_t so the
 * caller can detect "not found" via the negative-cast (any 32-bit
 * register value is a valid uint32, so we use UINT64_MAX as sentinel).
 */
static int find_reg_value(const char *file, const char *dbg_name,
                          uint32_t *out) {
    char path[256];
    snprintf(path, sizeof(path), "/sys/kernel/debug/dri/0/%s", file);

    FILE *f = fopen(path, "r");
    if (!f) return -1;

    char line[1024];
    char prefix[128];
    int prefix_len = snprintf(prefix, sizeof(prefix), "%s:", dbg_name);

    int found = -1;
    while (fgets(line, sizeof(line), f)) {
        const char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, prefix, prefix_len) != 0) continue;
        p += prefix_len;
        while (*p == ' ' || *p == '\t') p++;
        /* Skip optional 0x prefix */
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        char *endp = NULL;
        unsigned long v = strtoul(p, &endp, 16);
        if (endp != p) {
            *out = (uint32_t)v;
            found = 0;
            break;
        }
    }
    fclose(f);
    return found;
}

static void dump_regs(const char *path) {
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "fopen %s: %s\n", path, strerror(errno));
        return;
    }
    fprintf(f, "# A2XX MMIO register dump via /sys/kernel/debug/dri/0/* (mainline DRM)\n");
    fprintf(f, "# captured immediately after rendering the gl-capture test triangle\n");
    fprintf(f, "# format: offset  name  = value\n");
    int i;
    for (i = 0; regs[i].name; i++) {
        uint32_t v = 0;
        if (find_reg_value(regs[i].file, regs[i].dbg_name, &v) == 0) {
            fprintf(f, "0x%04x  %-22s  = 0x%08x\n",
                    regs[i].offset, regs[i].name, v);
        } else {
            fprintf(f, "0x%04x  %-22s  = N/A\n",
                    regs[i].offset, regs[i].name);
        }
    }
    fclose(f);
    fprintf(stderr, "wrote %s\n", path);
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
    if (!eglInitialize(dpy, NULL, NULL)) DIE("eglInitialize");
    eglBindAPI(EGL_OPENGL_ES_API);

    EGLint cfg_attr[] = {
        EGL_SURFACE_TYPE, EGL_DONT_CARE,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };
    EGLConfig cfg; EGLint ncfg = 0;
    if (!eglChooseConfig(dpy, cfg_attr, &cfg, 1, &ncfg) || ncfg < 1)
        DIE("eglChooseConfig: 0x%x", eglGetError());
    EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
    if (ctx == EGL_NO_CONTEXT) DIE("eglCreateContext");
    if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx))
        DIE("eglMakeCurrent");

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
        glRenderbufferStorage(GL_RENDERBUFFER, 0x8056 /* GL_RGBA4 */, W, H);
        st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (st != GL_FRAMEBUFFER_COMPLETE) DIE("FBO incomplete 0x%x", st);
    }

    fprintf(stderr, "GL_VENDOR:   %s\n", glGetString(GL_VENDOR));
    fprintf(stderr, "GL_RENDERER: %s\n", glGetString(GL_RENDERER));
    fprintf(stderr, "GL_VERSION:  %s\n", glGetString(GL_VERSION));

    GLuint vs = compile_shader(GL_VERTEX_SHADER, VS_SRC);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, FS_SRC);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glBindAttribLocation(prog, 0, "a_pos");
    glBindAttribLocation(prog, 1, "a_color");
    glLinkProgram(prog);
    GLint ok = 0; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) DIE("link failed");
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

    /* >>> sample registers HERE - GPU just finished, GL context still active */
    dump_regs(REGS_PATH);

    /* Read pixels back */
    uint8_t *pix = malloc(W * H * 4);
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pix);

    uint8_t *flipped = malloc(W * H * 4);
    int y;
    for (y = 0; y < H; y++)
        memcpy(flipped + (H - 1 - y) * W * 4, pix + y * W * 4, W * 4);

    FILE *f = fopen(CAP_PATH, "wb");
    if (!f) DIE("fopen %s", CAP_PATH);
    fwrite(flipped, 1, W * H * 4, f);
    fclose(f);
    fprintf(stderr, "wrote %s\n", CAP_PATH);

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
