/*
 * gpu-regdump-webos - dump A2XX GPU register state via KGSL ioctl
 *
 * Reads a curated set of A2XX MMIO registers via /dev/kgsl-3d0 and
 * IOCTL_KGSL_DEVICE_REGREAD (offset 0x3, KGSL_IOC_TYPE 0x09). Output
 * is plain text, one register per line:
 *   0xNNNN  REG_NAME  = 0xVVVVVVVV
 *
 * The output format is identical to the mainline-side companion tool
 * (gpu-regdump-mainline) so we can `diff` them after running the
 * same scene on both stacks.
 *
 * Build (cross with palmsdk):
 *   PSDK=/home/herrie/Downloads/palmsdk/opt/PalmPDK
 *   $PSDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc-4.3.3 -O2 \
 *     gpu-regdump-webos.c -o gpu-regdump-webos
 *
 * Deploy + run on webOS:
 *   novacom put file:///media/internal/gpu-regdump-webos < gpu-regdump-webos
 *   novacom run file://bin/chmod -- u+x /media/internal/gpu-regdump-webos
 *   novacom run file://media/internal/gpu-regdump-webos > regs-webos.txt
 *
 * Note: KGSL only allows direct reads of MMIO registers. Constant-memory-
 * mapped state (the CP_SET_CONSTANT register space, e.g. RB_*, PA_*, SQ_*,
 * VGT_*) cannot be read directly via this ioctl - those would need
 * CP_REG_TO_MEM packets. This tool covers the MMIO-readable subset which
 * is plenty to verify "what state is the GPU initialized into" across
 * stacks.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#define KGSL_IOC_TYPE 0x09

struct kgsl_device_regread {
    unsigned int offsetwords;
    unsigned int value;
};

#define IOCTL_KGSL_DEVICE_REGREAD \
    _IOWR(KGSL_IOC_TYPE, 0x3, struct kgsl_device_regread)

/* Curated A2XX MMIO register list.
 *
 * MMIO registers (offsets in WORDS, not bytes - KGSL_IOC takes word offsets).
 * Names match adreno_pm4.xml / yamato_reg.h conventions.
 *
 * The CP_SET_CONSTANT-space registers (0x2000+) are NOT directly readable
 * via this ioctl and are excluded - they need a CP_REG_TO_MEM round-trip.
 */
struct reg_info {
    unsigned int offset;
    const char *name;
};

static const struct reg_info regs[] = {
    /* RBBM (Resource-Based Bandwidth Manager / global control) */
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

    /* CP (Command Processor) */
    { 0x01C0, "CP_RB_BASE" },
    { 0x01C1, "CP_RB_CNTL" },
    { 0x01C5, "CP_RB_RPTR" },
    { 0x01C6, "CP_RB_WPTR" },
    { 0x01C7, "CP_RB_WPTR_DELAY" },
    { 0x01D5, "CP_IB1_BASE" },
    { 0x01D6, "CP_IB1_BUFSZ" },
    { 0x01D7, "CP_IB2_BASE" },
    { 0x01D8, "CP_IB2_BUFSZ" },
    { 0x017E, "CP_INT_CNTL" },
    { 0x01F4, "CP_INT_STATUS" },

    /* MH (Memory Hub / GPUMMU) - corrected offsets per freedreno
     * a2xx.xml and KGSL yamato_reg.h. Two register banks: 0x0040..0x0047
     * for MH_MMU_*, and 0x0a40..0x0a55 for ARBITER/INTERRUPT/CLNT_INTF. */
    { 0x0040, "MH_MMU_CONFIG" },
    { 0x0041, "MH_MMU_VA_RANGE" },
    { 0x0042, "MH_MMU_PT_BASE" },
    { 0x0043, "MH_MMU_PAGE_FAULT" },
    { 0x0044, "MH_MMU_TRAN_ERROR" },
    { 0x0046, "MH_MMU_MPU_BASE" },
    { 0x0047, "MH_MMU_MPU_END" },
    { 0x0a40, "MH_ARBITER_CONFIG" },
    { 0x0a42, "MH_INTERRUPT_MASK" },
    { 0x0a43, "MH_INTERRUPT_STATUS" },
    { 0x0a45, "MH_AXI_ERROR" },
    { 0x0a4e, "MH_DEBUG_CTRL" },
    { 0x0a4f, "MH_DEBUG_DATA" },
    { 0x0a54, "MH_CLNT_INTF_CTRL_CONFIG1" },
    { 0x0a55, "MH_CLNT_INTF_CTRL_CONFIG2" },

    /* SCRATCH (drivers can stash arbitrary values here) */
    { 0x0578, "SCRATCH_REG0" },
    { 0x0579, "SCRATCH_REG1" },
    { 0x057A, "SCRATCH_REG2" },
    { 0x057B, "SCRATCH_REG3" },
    { 0x057C, "SCRATCH_REG4" },
    { 0x057D, "SCRATCH_REG5" },
    { 0x057E, "SCRATCH_REG6" },
    { 0x057F, "SCRATCH_REG7" },
    { 0x0581, "SCRATCH_UMSK" },

    /* MASTER interrupt routing */
    { 0x03B7, "MASTER_INT_SIGNAL" },

    /* MMIO-mapped texture-cache and chicken bits */
    { 0x0E00, "TC_CNTL_STATUS" },
    { 0x0E1E, "TP0_CHICKEN" },

    /* RB BC control (a20x specific - included for completeness) */
    { 0x0F01, "RB_BC_CONTROL" },

    /* Sentinel */
    { 0, NULL }
};

int main(int argc, char **argv) {
    const char *node = (argc > 1) ? argv[1] : "/dev/kgsl-3d0";
    int fd = open(node, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "open(%s): %s\n", node, strerror(errno));
        return 1;
    }

    fprintf(stdout, "# A2XX MMIO register dump via %s (KGSL backend)\n", node);
    fprintf(stdout, "# format: offset  name  = value\n");

    int i;
    for (i = 0; regs[i].name != NULL; i++) {
        struct kgsl_device_regread req = { .offsetwords = regs[i].offset };
        if (ioctl(fd, IOCTL_KGSL_DEVICE_REGREAD, &req) < 0) {
            fprintf(stdout, "0x%04x  %-22s  = ERR(%s)\n",
                    regs[i].offset, regs[i].name, strerror(errno));
            continue;
        }
        fprintf(stdout, "0x%04x  %-22s  = 0x%08x\n",
                regs[i].offset, regs[i].name, req.value);
    }

    /*
     * Probe undocumented MH register gaps - 0x0a40..0x0a5f range.
     * Documented: 0x0a40 ARBITER, 0x0a42-44 INTERRUPT_*, 0x0a45 AXI_ERROR,
     * 0x0a46-4d PERFCOUNTER, 0x0a4e-4f DEBUG, 0x0a54-55 CLNT_INTF.
     * Gaps: 0x0a41, 0x0a50-53, 0x0a56-5f. Read everything in range to
     * see what's nonzero (= real register holding state) vs zero (=
     * unimplemented or zero-init register).
     */
    fprintf(stdout, "\n# MH register-range sweep (0x0a40..0x0a5f) - probe undocumented offsets\n");
    int k;
    for (k = 0x0a40; k <= 0x0a5f; k++) {
        struct kgsl_device_regread req2 = { .offsetwords = (unsigned int)k };
        if (ioctl(fd, IOCTL_KGSL_DEVICE_REGREAD, &req2) < 0) {
            fprintf(stdout, "0x%04x  = ERR(%s)\n", k, strerror(errno));
        } else {
            fprintf(stdout, "0x%04x  = 0x%08x%s\n", k, req2.value,
                    req2.value ? " [NONZERO]" : "");
        }
    }
    /* Also sweep 0x0040..0x0050 - MMU range + adjacent unknowns */
    fprintf(stdout, "\n# MH MMU-range sweep (0x0040..0x0050)\n");
    for (k = 0x0040; k <= 0x0050; k++) {
        struct kgsl_device_regread req2 = { .offsetwords = (unsigned int)k };
        if (ioctl(fd, IOCTL_KGSL_DEVICE_REGREAD, &req2) < 0) {
            fprintf(stdout, "0x%04x  = ERR(%s)\n", k, strerror(errno));
        } else {
            fprintf(stdout, "0x%04x  = 0x%08x%s\n", k, req2.value,
                    req2.value ? " [NONZERO]" : "");
        }
    }

    /*
     * MH_DEBUG_CTRL sweep (0..63) - matches KGSL postmortem dump.
     * KGSL exposes the same data via /sys/kernel/debug/kgsl/mh_debug
     * but webOS doesn't have debugfs mounted, so we do the sweep
     * manually via the regread ioctl. Compare against mainline's
     * a2xx_gpu.c diagnostic output to spot which MH internal slots
     * differ between the "good" KGSL state and "broken" mainline state.
     */
    fprintf(stdout, "\n# MH_DEBUG_CTRL sweep (0..63)\n");
    int j;
    for (j = 0; j < 64; j++) {
        struct kgsl_device_regread w_req = { .offsetwords = 0x0a4e, .value = j };
        struct kgsl_device_regread r_req = { .offsetwords = 0x0a4f };
        /* KGSL_IOC_TYPE 0x09, opcode 0x4 = REGWRITE */
        struct kgsl_device_regwrite { unsigned int offsetwords, value; };
        #define IOCTL_KGSL_DEVICE_REGWRITE \
            _IOW(KGSL_IOC_TYPE, 0x4, struct kgsl_device_regwrite)
        struct kgsl_device_regwrite ww = { .offsetwords = 0x0a4e,
                                           .value = (unsigned int)j };
        if (ioctl(fd, IOCTL_KGSL_DEVICE_REGWRITE, &ww) < 0) {
            fprintf(stdout, "MH_DEBUG[%2d] WRITE_ERR(%s)\n", j, strerror(errno));
            continue;
        }
        if (ioctl(fd, IOCTL_KGSL_DEVICE_REGREAD, &r_req) < 0) {
            fprintf(stdout, "MH_DEBUG[%2d] READ_ERR(%s)\n", j, strerror(errno));
            continue;
        }
        fprintf(stdout, "MH_DEBUG[%2d] = 0x%08x\n", j, r_req.value);
        (void)w_req;
    }

    close(fd);
    return 0;
}
