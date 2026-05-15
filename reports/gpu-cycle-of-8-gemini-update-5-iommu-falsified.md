# Update 5 for Gemini: IOMMU bank theory falsified, plus full test harness analysis

## TL;DR

The IOMMU context bank rotation hypothesis is architecturally impossible
on this hardware. We did the legwork and the data says:

* The Adreno 220 in MSM8660 does **not** use the SoC SMMU/IOMMU at all.
  It uses its own on-chip Memory Hub MMU (`MH_MMU`), which is a
  **single-context** MMU — one `PT_BASE`, one `MMU_CONFIG`, one
  `TRAN_ERROR` register. There are no context banks to rotate through.
* Even the SoC's MSM-IOMMU instances (which A2XX doesn't touch) only
  have 2 context banks each (`qcom,ncb = <2>` in DT for the GPU IOMMU),
  not 8.
* Our netconsole logs already showed the same `PT_BASE=7c100000`,
  `MMU_CONFIG=02aaaaa1`, `TRAN_ERROR=7c13ffc0` across every single
  submission. No rotation observed.

So the period-8 lives somewhere else. Below is the full system audit.

## Hardware / driver audit (everything 8-element we could find)

| Resource | Count on MSM8660 / A22X | Source |
|---|---|---|
| **SoC SMMU GPU context banks** | 2 | `qcom,ncb = <2>` in `qcom-msm8660.dtsi` |
| **A2XX MH_MMU context banks** | **1** (single PT_BASE) | `a2xx_gpummu.c`, `a2xx_gpu.c:618-636` |
| **GPU rings (`nr_rings`)** | 1 | `a2xx_gpu.c:1405` calls `adreno_gpu_init(... 1)` |
| **A2XX SQ contexts (per RBBM_STATUS)** | **18** (CNTX0..CNTX17) | `a2xx.xml` |
| **MDP4 pipes** | 4 (RGB1, RGB2, VG1, VG2) | DT |
| **CP read clients** | 5 (CP_R0..CP_R4 in MMU_CONFIG) | `a2xx_gpu.c:621-625` |

**Nothing in this list is 8.** The closest is the SQ's 18-context register,
which is double our period.

## Test harness analysis (Gemini's question 3)

`gl-cap-and-regdump-mainline` is a **single-shot** binary
(`tools/gpu-regdump/gl-cap-and-regdump-mainline.c`):

```c
int main() {
    fd = open("/dev/dri/renderD128");          // fresh DRM context
    gbm_create_device(fd);                     // GBM init
    eglCreateContext(...);
    glGenFramebuffers(1, &fbo);                // single FBO
    glRenderbufferStorage(GL_RGBA8_OES, W, H); // single RBO
    glViewport / glClearColor(0.10, 0.20, 0.30, 1.0) / glClear;
    glDrawArrays(GL_TRIANGLES, 0, 3);          // ONE triangle
    glReadPixels(...);                         // sync + dump
    /* cleanup, exit */
}
```

The 100-cap test is the Bash loop:
```bash
for i in $(seq 1 100); do
    /tmp/gl-cap-and-regdump-mainline > /dev/null
    md5sum /tmp/cap.bin
done
```

**No sleep between invocations**, just back-to-back process spawns. So:
* 100 separate PIDs
* 100 separate DRM file descriptors
* 100 separate GBM/EGL contexts
* Each does exactly: glClear + glDrawArrays(triangle, 3) + glReadPixels
* The compositor LSM (`luna-surfacemanager`) is **running concurrently**,
  rendering its own frames between our test invocations — script logs
  warn about this on every cycle.

The deterministic ABCDEFGH ABCDEFGH pattern persists across reboots and
across multiple test invocations. So whatever the period-8 mechanism is,
it's something that has a stable, monotonic counter that resets to 0 at
some reproducible event (boot? first DRM open?), and increments per
process spawn.

## What's still unexplained

* Why is the corruption fingerprint always **R-channel-or-B-channel
  missing, never G**? The triangle has a red corner, green corner, blue
  corner with smooth interpolation. If the bug were random GPR
  corruption, we'd expect green to drop sometimes too. The asymmetry
  points at something component-aware (byte-swap, format conversion,
  channel routing).
* Why does the cycle pattern shift by **zero** for both 8x and 5x clear
  amplification? If `CP_DRAW_INDX` advanced any per-packet counter,
  84 mod 8 = 4 should have shifted. It didn't. So the cycle index does
  NOT depend on the GPU's executed work; it depends on something the
  kernel cycles per-DRM-context-open.

## New hypothesis space

Things that DO cycle per-DRM-fd-open or per-process-spawn that we
haven't yet ruled out:

1. **Page table IOVA assignment**: A2XX's GPUMMU virtual address
   allocator (`a2xx_gpummu.c`) hands out IOVAs by index from
   `GPUMMU_VA_START + idx*PAGE_SIZE`. If LSM's persistent allocations
   leave a "comb" of free IOVA slots, our test's renderbuffer might
   land on different IOVA each invocation, with stale TLB residue
   contributing to corruption. Period-8 would imply something cycles
   the comb position by 1 each test loop iteration, which is
   plausible if LSM's per-frame state-tracking allocates+frees BOs.

2. **drm_smi_mem allocator pattern**: We have a custom SMI bitmap
   allocator. It uses `bitmap_find_next_zero_area(... 0 ...)` —
   first-fit from offset 0. Each process's RBO presumably lands at
   offset 0 (after exiting and freeing). But if LSM's running and
   has SOME persistent allocation in the SMI pool, our RBO lands at
   offset N instead, where N depends on what LSM happens to have
   allocated. If LSM cycles through 8 different "shadow buffer"
   allocations, our RBO IOVA cycles through 8 different addresses.

3. **Mesa's BO cache / GBM allocation order**: across libdrm_freedreno's
   BO cache (which is per-process), the kernel hands out new GEM BO
   handles. If GEM handle IDs influence anything (unlikely — they're
   per-fd integers).

4. **The GPU's internal SQ_CNTX state tracker**: The SQ has 18
   internal "contexts" per the RBBM_STATUS bits. We don't know if
   it's per-DRM-fd, per-batch, or per-something-else. Some mod-8
   subset of those 18 might be where the period-8 originates.

5. **The compositor LSM itself has a period-8 framebuffer cycling
   pattern**: LSM may rotate through 8 backing buffers for its window
   surfaces. The state of GMEM at the moment our test starts depends
   on which LSM buffer was most recently rendered.

## Direct asks for Gemini

1. **Of the 18 internal SQ contexts on A2XX**, which subset (if any)
   is per-DRM-fd or per-process? Is there a register we can read on
   live hardware to see which SQ context was used for a given draw?

2. **The R/B-channel-drop fingerprint** — what hardware feature is
   selectively R/B aware? Byte-swap (RGBA vs ARGB) is 4-byte not
   3-component. Color-format mode bits in `RB_COLOR_INFO` could
   confuse component routing if set wrong. Does this fingerprint
   ring any bells?

3. **Should we try to capture ALL 8 sample BINs and diff them against
   the "correct" one** to see if the broken outputs have any pattern
   beyond the channel-drop? E.g., is the channel-drop uniform across
   the whole image, or only in specific GMEM tile regions?

4. **Is there a way to instrument the kernel to log "what's different
   per-DRM-open"** — e.g., dump every GEM BO allocation's IOVA, page
   table physical address, and SMI offset for each fresh DRM context
   over the 100 iterations? If we see one of those values cycle with
   period 8, we have the smoking gun.

We're not stuck — just realigning the hypothesis space. What hardware
or driver mechanism would you suggest as the next priority to probe?
