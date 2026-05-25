# APQ8060 / MSM8660 (HP TouchPad) memory-map audit — mainline vs legacy webOS

Sources: mainline DT (`qcom-apq8060-tenderloin-common.dtsi`), on-device `/proc/iomem`
+ `/proc/device-tree/reserved-memory`, legacy webOS msm-3.0.5 (`kernel-3.0.5.txt`,
tenderloin block ~line 293703 + `grp3d`/`mdp` `msm_bus` vectors).

## Two physical memory types

| Type | Range | Kernel-managed? | Who can reach it |
|---|---|---|---|
| **EBI / main DDR** | `0x40000000`–`0x7fffffff` (PHYS_OFFSET `0x40200000`) | yes (System RAM) | **all** masters, via apps fabric `AFAB_SLV_EBI_CH0` |
| **SMI** (System Memory Interface, on-MMSS fast RAM) | `0x38000000`–`0x3bffffff` (64 MiB) | **no** (reserved, not in memblock) | **only MMSS masters** (MDP, VIDC, GPU) via `MMFAB_SLV_SMI`. **VIDC fw can ONLY reach SMI.** |

On-device System RAM (`/proc/iomem`): `0x40200000-0x466fffff` + `0x48000000-0x7f5fffff`.
SMI is **not** in System RAM (correct — it's separate MMSS memory).

## SMI layout (64 MiB @ 0x38000000) — mainline vs legacy

| Region | Legacy webOS | Mainline DT | Match |
|---|---|---|---|
| VIDC firmware + scratch | `PMEM_KERNEL_SMI` `0x38000000` **3 MiB** | `vidc-fw@38000000` **3 MiB** no-map | ✅ exact |
| VIDC codec I/O buffers | `MSM_PMEM_SMIPOOL` `0x38300000` **61 MiB** | `vidc-smipool@38300000` **61 MiB** no-map | ✅ exact |
| **Display/GPU framebuffers** | **none — NOT in SMI** | **none** (scanout moved to EBI) | ✅ |

## EBI / DDR layout — mainline vs legacy

| Purpose | Legacy webOS (PMEM) | Mainline DT (reserved-memory) | Notes |
|---|---|---|---|
| RPM/SMEM shared mem | (fixed) | `smem@40000000` 2 MiB | ✅ |
| Audio DSP (LPASS) | part of `PMEM_ADSP` 32 MiB | `lpass@46700000` 6 MiB | smaller |
| Sensors DSP | — | `dsps@46d00000` 1 MiB | new |
| **Display framebuffers** | `MSM_FB` ~5 MiB **+ `MSM_PMEM_SF` 64 MiB** (EBI) | **`scanout-fb@60000000` 32 MiB** pinned no-map | ⚠️ **smaller (32 vs ~69 MiB)** |
| Camera (VFE) / general DMA | `pmem_adsp`/`pmem_camera` (EBI) | `linux,cma@7c000000` 32 MiB (`cma-default`) **+ global CMA @0x7a000000 32 MiB** | ⚠️ **two CMA pools (64 MiB)** |
| Crash log | — | `ramoops@7f500000` 1 MiB | new |
| Bootloader splash | `MSM_FB_BASE` | `fb0@7f600000` | ✅ |

## Interconnect (fabric votes) — who writes where

| Master | Legacy `msm_bus` (ab/ib) | Mainline interconnects | Match |
|---|---|---|---|
| **GPU** (GRAPHICS_3D) | **EBI only**, 2008/2008 MB/s | `gfx-mem` → **EBI only** (`a2xx_icc_bw_for_freq`≈2 GB/s) | ✅ (SMI-vote attempt reverted) |
| **MDP** (display) | SMI 147/184 + EBI 334/417 MB/s | `mdp{0,1}-smi` + `mdp{0,1}-ebi` (both voted) | ✅ |
| **VIDC** (HD_CODEC) | SMI + EBI | `MMFAB_MAS_HD_CODEC_PORT0` → SMI + EBI | ✅ |
| **VFE** (camera) | EBI | EBI / CMA `0x7c000000` (physical, no SMI) | ✅ |

## Audit findings

1. ✅ **SMI usage is exactly legacy**: VIDC firmware (3 MiB) + codec pool (61 MiB), nothing else.
2. ✅ **GPU is EBI-only, matching `grp3d_max_vectors`** — the SMI-vote detour was wrong and is reverted.
3. ✅ **Display FBs belong in EBI** (legacy `MSM_FB` + `MSM_PMEM_SF`), now done via the pinned `scanout-fb` carveout. SMI scanout = stripes/underrun, confirmed both ways.
4. ⚠️ **scanout-fb is 32 MiB vs legacy ~69 MiB** (`MSM_FB` ~5 + `MSM_PMEM_SF` 64). Likely fine for one compositor + glmark, but tight under multi-client; candidate to enlarge if FB allocation pressure appears.
5. ⚠️ **Two CMA pools (64 MiB total)**: `linux,cma@7c000000` (DT, `cma-default`) **plus** a global CMA at `0x7a000000` (from `CONFIG_CMA_SIZE_MBYTES`/`cma=` cmdline). The global one is redundant with the DT default **and** is what `scanout-fb`'s dynamic placement collided with. Recommend dropping the kernel-config/cmdline CMA so only the DT `linux,cma` exists — reclaims 32 MiB and removes the collision class entirely.
6. ⚠️ **`scanout-fb` now fixed at `0x60000000`** (was racing the global CMA at `0x7a000000`). With the global CMA removed (finding 5), dynamic placement would be safe again.

## Resolution (2026-05-25)
- **Finding 5 fixed:** `CONFIG_CMA_SIZE_MBYTES` set `32 -> 0` in all three defconfigs
  (`tenderloin_defconfig`, `tenderloin_debug_defconfig`, `tenderloin_fast_defconfig`).
  The redundant global CMA at `0x7a000000` is gone; only the DT `linux,cma@7c000000`
  (`cma-default`) remains. This also removes the `scanout-fb` collision class.
- **Resulting clean split:** 32 MiB `scanout-fb` @ `0x60000000` (pinned display FBs)
  + 32 MiB `linux,cma` @ `0x7c000000` (camera/VFE + misc DMA) — no overlap, each
  sized for its consumer (VFE needs ~30 MiB; FBs no longer compete for CMA).

## Net
The SMI/EBI/interconnect split now matches legacy exactly ("FBs in EBI, SMI for
VIDC only, GPU votes EBI"), and the CMA layout is a clean non-overlapping
32+32 MiB (display carveout + camera CMA). Remaining open item: display pool is
32 vs legacy ~69 MiB — enlarge only if FB-allocation pressure appears.
WATCH: total CMA dropped 64->32 MiB; with FBs out of CMA this should be ample
for the camera, but if VFE hits cma_alloc failures, enlarge linux,cma@7c000000.
