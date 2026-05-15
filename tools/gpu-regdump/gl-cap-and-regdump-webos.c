/*
 * gl-cap-and-regdump-webos
 *
 * Combined tool: renders the same triangle as gl-capture-webos AND
 * captures GPU register state via KGSL ioctl in the same process,
 * before the GL context is torn down. This guarantees the register
 * snapshot reflects state from immediately after the known render
 * rather than from an idle GPU some time later.
 *
 * Outputs two files:
 *   /media/internal/cap.bin   - 1024x768 RGBA8 framebuffer (same format
 *                               as gl-capture-webos for direct diff
 *                               against gl-capture mainline output)
 *   /media/internal/regs.txt  - register dump in canonical format
 *                               (matches gpu-regdump-webos output)
 *
 * Key sequence:
 *   1. SDL/GLES2 init
 *   2. Render triangle (same as gl-capture-webos)
 *   3. glFinish() to ensure all GPU work has completed
 *   4. Open /dev/kgsl-3d0 and dump registers
 *   5. SwapBuffers to make the render visible on-screen briefly
 *      (so the user can visually verify the right scene rendered)
 *   6. glReadPixels and write cap.bin
 *
 * Build:
 *   PSDK=/home/herrie/Downloads/palmsdk/opt/PalmPDK
 *   $PSDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc-4.3.3 -O2 \
 *     gl-cap-and-regdump-webos.c \
 *     -I $PSDK/include -I $PSDK/include/SDL \
 *     -L $PSDK/device/lib -lSDL-1.2 -lGLESv2 -lpdl -lm \
 *     -Wl,--allow-shlib-undefined,--unresolved-symbols=ignore-in-shared-libs \
 *     -o gl-cap-and-regdump-webos
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <SDL.h>
#include <PDL.h>
#include <GLES2/gl2.h>

#define W 1024
#define H 768
#define CAP_PATH  "/media/internal/cap.bin"
#define REGS_PATH "/media/internal/regs.txt"

#define KGSL_IOC_TYPE 0x09
struct kgsl_device_regread {
    unsigned int offsetwords;
    unsigned int value;
};
#define IOCTL_KGSL_DEVICE_REGREAD \
    _IOWR(KGSL_IOC_TYPE, 0x3, struct kgsl_device_regread)

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

struct reg_info { unsigned int offset; const char *name; };

static const struct reg_info regs[] = {
    /* RBBM */
    { 0x0040, "RBBM_PATCH_REV" },
    { 0x003B, "RBBM_PERIPHID0" },
    { 0x003C, "RBBM_PERIPHID1" },
    { 0x003D, "RBBM_PERIPHID2" },
    { 0x0058, "RBBM_PM_OVERRIDE1" },
    { 0x0059, "RBBM_PM_OVERRIDE2" },
    { 0x005C, "RBBM_DEBUG" },
    { 0x005F, "RBBM_DEBUG_OUT" },
    { 0x0061, "RBBM_INT_CNTL" },
    { 0x0062, "RBBM_INT_STATUS" },
    { 0x0064, "RBBM_INT_ACK" },
    { 0x017F, "RBBM_STATUS" },
    /* CP */
    { 0x01C0, "CP_RB_BASE" },
    { 0x01C1, "CP_RB_CNTL" },
    { 0x01C5, "CP_RB_RPTR" },
    { 0x01C6, "CP_RB_WPTR" },
    { 0x01D5, "CP_IB1_BASE" },
    { 0x01D6, "CP_IB1_BUFSZ" },
    { 0x01D7, "CP_IB2_BASE" },
    { 0x01D8, "CP_IB2_BUFSZ" },
    { 0x017E, "CP_INT_CNTL" },
    { 0x01F4, "CP_INT_STATUS" },
    /* MH */
    { 0x040D, "MH_DEBUG_CTRL" },
    { 0x040F, "MH_DEBUG_DATA" },
    { 0x0050, "MH_INTERRUPT_MASK" },
    { 0x0051, "MH_INTERRUPT_STATUS" },
    { 0x0052, "MH_AXI_ERROR" },
    { 0x040C, "MH_MMU_CONFIG" },
    { 0x040E, "MH_MMU_VA_RANGE" },
    /* SCRATCH */
    { 0x0578, "SCRATCH_REG0" },
    { 0x0579, "SCRATCH_REG1" },
    { 0x057A, "SCRATCH_REG2" },
    { 0x057B, "SCRATCH_REG3" },
    { 0x057C, "SCRATCH_REG4" },
    { 0x057D, "SCRATCH_REG5" },
    { 0x057E, "SCRATCH_REG6" },
    { 0x057F, "SCRATCH_REG7" },
    { 0x0581, "SCRATCH_UMSK" },
    /* MASTER int */
    { 0x03B7, "MASTER_INT_SIGNAL" },
    /* TC, chicken, BC */
    { 0x0E00, "TC_CNTL_STATUS" },
    { 0x0E1E, "TP0_CHICKEN" },
    { 0x0F01, "RB_BC_CONTROL" },
    /*
     * Shader-constant SRAM probes (state-leak suspects).
     *
     * The KGSL_DEVICE_REGREAD ioctl takes a word offset and the A2XX
     * shader-constant SRAM is MMIO-mapped per a2xx.xml at:
     *   0x4000  SQ_CONSTANT_0    (vec4 ALU constants, 256 entries x 4 dw)
     *   0x4800  SQ_FETCH_0       (32 texture-fetch constants x 6 dw)
     *   0x4900  Boolean constants (8 dwords, 256 bool flags)
     *   0x4908  Loop control     (56 dwords)
     *
     * Sample slot 0 of each (where Mesa's clear color and texture[0]
     * end up) and a few additional vec4 slots so we can see if any of
     * them carry stale data across runs.
     */
    /* vec4 slot 0 (PS const slot 0 / clear-color target) */
    { 0x4000, "ALU_CONST_0_R" },
    { 0x4001, "ALU_CONST_0_G" },
    { 0x4002, "ALU_CONST_0_B" },
    { 0x4003, "ALU_CONST_0_A" },
    /* vec4 slots 1..3 - early VS uniform slots */
    { 0x4004, "ALU_CONST_1_R" },
    { 0x4005, "ALU_CONST_1_G" },
    { 0x4006, "ALU_CONST_1_B" },
    { 0x4007, "ALU_CONST_1_A" },
    { 0x4008, "ALU_CONST_2_R" },
    { 0x400C, "ALU_CONST_3_R" },
    /* vec4 slot 32 = VS_CONST_BASE (Mesa's first VS user uniform) */
    { 0x4080, "ALU_CONST_32_R_VS" },
    { 0x4081, "ALU_CONST_32_G_VS" },
    /* vec4 slot 288 = PS_CONST_BASE (Mesa's first PS user uniform).
     * In word-offset terms: 0x4000 + 288*4 = 0x4480 */
    { 0x4480, "ALU_CONST_288_R_PS" },
    { 0x4481, "ALU_CONST_288_G_PS" },
    /* texture-fetch constant 0 (first 6 dwords) */
    { 0x4800, "TEX_FETCH_0_W0" },
    { 0x4801, "TEX_FETCH_0_W1" },
    { 0x4802, "TEX_FETCH_0_W2" },
    { 0x4803, "TEX_FETCH_0_W3" },
    /* All 8 dwords of bool constants (256 flags) */
    { 0x4900, "BOOL_DW0" },
    { 0x4901, "BOOL_DW1" },
    { 0x4902, "BOOL_DW2" },
    { 0x4903, "BOOL_DW3" },
    { 0x4904, "BOOL_DW4" },
    { 0x4905, "BOOL_DW5" },
    { 0x4906, "BOOL_DW6" },
    { 0x4907, "BOOL_DW7" },
    /* Loop constants 0..7 (8 entries of 56-dword bank) */
    { 0x4908, "LOOP_DW0" },
    { 0x4909, "LOOP_DW1" },
    { 0x490A, "LOOP_DW2" },
    { 0x490B, "LOOP_DW3" },
    { 0, NULL }
};

static int dump_regs(const char *path) {
    int kfd = open("/dev/kgsl-3d0", O_RDWR);
    if (kfd < 0) {
        fprintf(stderr, "open(/dev/kgsl-3d0): %s\n", strerror(errno));
        return -1;
    }
    FILE *f = fopen(path, "w");
    if (!f) {
        close(kfd);
        return -1;
    }
    fprintf(f, "# A2XX MMIO register dump via /dev/kgsl-3d0 (KGSL backend)\n");
    fprintf(f, "# captured immediately after rendering the gl-capture test triangle\n");
    fprintf(f, "# format: offset  name  = value\n");
    int i;
    for (i = 0; regs[i].name; i++) {
        struct kgsl_device_regread req = { .offsetwords = regs[i].offset };
        if (ioctl(kfd, IOCTL_KGSL_DEVICE_REGREAD, &req) < 0) {
            fprintf(f, "0x%04x  %-22s  = ERR\n",
                    regs[i].offset, regs[i].name);
            continue;
        }
        fprintf(f, "0x%04x  %-22s  = 0x%08x\n",
                regs[i].offset, regs[i].name, req.value);
    }

    /*
     * SoC-reserved sample dump: full ALU slots 0..31 (128 dwords) and
     * texture-fetch slots 0..31 (192 dwords). These are the values the
     * proprietary stack relies on for the fixed-function vertex
     * pipeline + texture sampler descriptors, and we need to bake
     * them into the mainline shadow BO so PM4_LOAD_CONSTANT_CONTEXT
     * doesn't blow them away when we re-enable it. Output as a
     * separate easy-to-parse section.
     */
    fprintf(f, "\n# === SoC-reserved ALU slots 0..31 (full 128 dwords) ===\n");
    {
        unsigned slot, ch;
        for (slot = 0; slot < 32; slot++) {
            for (ch = 0; ch < 4; ch++) {
                unsigned woff = 0x4000 + slot * 4 + ch;
                struct kgsl_device_regread req = { .offsetwords = woff };
                fprintf(f, "0x%04x  ALU[%2u].%c              = ",
                        woff, slot, "RGBA"[ch]);
                if (ioctl(kfd, IOCTL_KGSL_DEVICE_REGREAD, &req) < 0)
                    fprintf(f, "ERR\n");
                else
                    fprintf(f, "0x%08x\n", req.value);
            }
        }
    }

    fprintf(f, "\n# === Texture-fetch slots 0..31 (full 192 dwords, 6 per slot) ===\n");
    {
        unsigned slot, dw;
        for (slot = 0; slot < 32; slot++) {
            for (dw = 0; dw < 6; dw++) {
                unsigned woff = 0x4800 + slot * 6 + dw;
                struct kgsl_device_regread req = { .offsetwords = woff };
                fprintf(f, "0x%04x  TEX[%2u].dw%u            = ",
                        woff, slot, dw);
                if (ioctl(kfd, IOCTL_KGSL_DEVICE_REGREAD, &req) < 0)
                    fprintf(f, "ERR\n");
                else
                    fprintf(f, "0x%08x\n", req.value);
            }
        }
    }

    fclose(f);
    close(kfd);
    return 0;
}

int main(int argc, char *argv[]) {
    PDL_Init(0);
    if (SDL_Init(SDL_INIT_VIDEO) < 0) DIE("SDL_Init: %s", SDL_GetError());
    if (!SDL_SetVideoMode(W, H, 0, SDL_OPENGLES))
        DIE("SDL_SetVideoMode: %s", SDL_GetError());

    fprintf(stderr, "GL_VENDOR:   %s\n", glGetString(GL_VENDOR));
    fprintf(stderr, "GL_RENDERER: %s\n", glGetString(GL_RENDERER));
    fprintf(stderr, "GL_VERSION:  %s\n", glGetString(GL_VERSION));

    /* Build same triangle program as gl-capture-webos */
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

    /* >>> register dump RIGHT HERE - GPU just finished render, GL context
     * still active, before any teardown. */
    if (dump_regs(REGS_PATH) == 0)
        fprintf(stderr, "wrote %s\n", REGS_PATH);
    else
        fprintf(stderr, "regdump failed\n");

    /* Read pixels back */
    uint8_t *pix = malloc(W * H * 4);
    if (!pix) DIE("malloc");
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pix);

    uint8_t *flipped = malloc(W * H * 4);
    if (!flipped) DIE("malloc");
    {
        int y;
        for (y = 0; y < H; y++)
            memcpy(flipped + (H - 1 - y) * W * 4, pix + y * W * 4, W * 4);
    }
    FILE *f = fopen(CAP_PATH, "wb");
    if (!f) DIE("fopen %s", CAP_PATH);
    fwrite(flipped, 1, W * H * 4, f);
    fclose(f);
    fprintf(stderr, "wrote %s\n", CAP_PATH);

    free(flipped); free(pix);

    /* Show on-screen briefly so the user can visually verify the right
     * scene rendered. Without this the screen looks black during the run. */
    SDL_GL_SwapBuffers();
    SDL_Delay(2000);

    SDL_Quit();
    PDL_Quit();
    return 0;
}
