/*
 * gl-capture-webos - same triangle test as gl-capture, for legacy webOS
 *
 * Renders an identical scene to gl-capture.c (single triangle, smooth
 * vertex-color interpolation, dark navy clear) using SDL+GLES2 on legacy
 * webOS (KGSL kernel + proprietary HP/Adreno libGLESv2.so userspace).
 * Reads pixels via glReadPixels and writes raw RGBA8888 to a file.
 *
 * The output is binary-comparable to gl-capture's output on mainline.
 * Diffing the two reveals exactly which pixels mainline freedreno gets
 * wrong vs the legacy proprietary stack on the same hardware.
 *
 * Build (host, with palmsdk):
 *   PSDK=/home/herrie/Downloads/palmsdk/opt/PalmPDK
 *   $PSDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc-4.3.3 -O2 \
 *     gl-capture-webos.c \
 *     -I $PSDK/include -I $PSDK/include/SDL \
 *     -L $PSDK/device/lib -lSDL-1.2 -lGLESv2 -lpdl -lm \
 *     -Wl,--allow-shlib-undefined -o gl-capture-webos
 *
 * Deploy + run:
 *   novacom put file:///media/internal/gl-capture-webos < gl-capture-webos
 *   novacom run file://media/internal/gl-capture-webos
 *
 * Pull:
 *   novacom get file:///media/internal/cap.bin > gl-capture-webos.bin
 *
 * Convert (host):
 *   convert -size 1024x768 -depth 8 RGBA:gl-capture-webos.bin out.png
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <SDL.h>
#include <PDL.h>
#include <GLES2/gl2.h>

#define W 1024
#define H 768
#define OUT_PATH "/media/internal/cap.bin"

#define DIE(fmt, ...) do { fprintf(stderr, "ERROR: " fmt "\n", ##__VA_ARGS__); exit(1); } while (0)

/* Same shaders as gl-capture.c for binary-comparable output */
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

int main(int argc, char *argv[]) {
    PDL_Init(0);
    if (SDL_Init(SDL_INIT_VIDEO) < 0) DIE("SDL_Init: %s", SDL_GetError());

    if (!SDL_SetVideoMode(W, H, 0, SDL_OPENGLES))
        DIE("SDL_SetVideoMode: %s", SDL_GetError());

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
    if (!ok) {
        char log[1024]; GLsizei len = 0;
        glGetProgramInfoLog(prog, sizeof(log), &len, log);
        fprintf(stderr, "link failed: %.*s\n", len, log);
        return 1;
    }
    glUseProgram(prog);

    /* Same vertex data as gl-capture.c */
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

    uint8_t *pix = (uint8_t *)malloc(W * H * 4);
    if (!pix) DIE("malloc");
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pix);
    GLenum gle = glGetError();
    if (gle != GL_NO_ERROR) fprintf(stderr, "glReadPixels: 0x%x\n", gle);

    /* Flip rows so the binary file converts straight to PNG via:
     *   convert -size 1024x768 -depth 8 RGBA:cap.bin cap.png
     * (matches gl-capture.c's flipping)
     */
    uint8_t *flipped = (uint8_t *)malloc(W * H * 4);
    if (!flipped) DIE("malloc");
    {
        int y;
        for (y = 0; y < H; y++)
            memcpy(flipped + (H - 1 - y) * W * 4, pix + y * W * 4, W * 4);
    }

    FILE *f = fopen(OUT_PATH, "wb");
    if (!f) DIE("fopen %s: %s", OUT_PATH, strerror(errno));
    if (fwrite(flipped, 1, W * H * 4, f) != W * H * 4) DIE("fwrite short");
    fclose(f);
    fprintf(stderr, "wrote %s (%d bytes)\n", OUT_PATH, W * H * 4);

    free(flipped); free(pix);

    /* Show the rendered frame on screen briefly so user can verify visually */
    SDL_GL_SwapBuffers();
    SDL_Delay(2000);

    SDL_Quit();
    PDL_Quit();
    return 0;
}
