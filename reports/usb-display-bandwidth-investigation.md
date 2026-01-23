# USB/Display Bandwidth Contention Investigation

## Problem Statement

USB (g_ether gadget) crashes when DRM/display activity is occurring on the HP TouchPad (APQ8060). The crash happens:
- During kmscube rendering (both hardware and software rendering)
- During modetest pattern display (after several seconds)
- When kmscube is killed (originally thought to be cleanup, but actually happens during rendering)

## Hardware Architecture

The APQ8060/MSM8660 has a fabric-based bus architecture:

```
              +------------------+
              |   APPSS Fabric   |  (CPU, L2, Memory/EBI)
              +--------+---------+
                       |
         +-------------+-------------+
         |                           |
  +------+------+            +-------+-------+
  | MMSS Fabric |            | System Fabric |
  | (MDP, GPU,  |            | (USB, DMA,    |
  |  Camera)    |            |  Peripherals) |
  +------+------+            +-------+-------+
         |                           |
    +----+----+              +-------+-------+
    |   SMI   |              | Daytona Fab   |
    | 64MB    |              | (SDCC, ADM)   |
    +---------+              +---------------+
```

**Key insight**: MDP can access memory via two paths:
1. **EBI (External Bus Interface)** - Main system RAM on APPSS fabric (shares with USB)
2. **SMI (System Memory Interface)** - 64MB dedicated video memory on MMSS fabric (no contention)

## Solution: SMI Memory for Display

### The Root Cause

The original problem was that MDP display scanout and USB were competing for EBI memory bandwidth on the APPSS fabric. No amount of bandwidth tuning could reliably eliminate USB crashes because both were fundamentally competing for the same bus.

### The Fix

**Route MDP display traffic to SMI memory instead of EBI.** This completely eliminates contention:

- MDP reads framebuffers from SMI on the **MMSS fabric**
- USB accesses memory via the **System/APPSS fabric**
- No shared path = no contention

### WebOS Behavior

WebOS used SMI memory for display via `pmem_smipool`:
- Base address: `0x38300000`
- Size: 61MB (`0x3D00000`)
- Used for: framebuffers allocated by userspace (via pmem driver)
- WebOS did NOT use CMA for display allocations

Full SMI layout in webOS:
```
0x38000000 - 0x382FFFFF: 3MB kernel reserved
0x38300000 - 0x3BFFFFFF: 61MB pmem_smipool (userspace framebuffers)
```

## Implementation (Committed)

### Commit 1: ARM: dts: qcom: tenderloin: Use SMI memory for display framebuffers

1. **Added SMI memory node** at `0x38000000` (64MB)

2. **Added reserved memory region** matching webOS pmem_smipool:
   ```dts
   drm_smi_mem: framebuffer@38300000 {
       compatible = "shared-dma-pool";
       reg = <0x38300000 0x3d00000>;  /* 61MB - exact webOS size */
       no-map;
   };
   ```

3. **Updated MDP interconnects** to route to SMI instead of EBI:
   ```dts
   interconnects = <&mmss_fabric MMFAB_MAS_MDP_PORT0 &mmss_fabric MMFAB_SLV_SMI>,
                   <&mmss_fabric MMFAB_MAS_MDP_PORT1 &mmss_fabric MMFAB_SLV_SMI>;
   interconnect-names = "mdp0-smi", "mdp1-smi";
   ```

4. **Added memory-region reference** so MDP uses SMI for DMA allocations:
   ```dts
   memory-region = <&drm_smi_mem>;
   ```

5. **Set MDP bandwidth** to webOS mdp_app values:
   ```dts
   qcom,icc-bw-avg-kbps = <368640>;   /* 377 MB/s */
   qcom,icc-bw-peak-kbps = <460800>;  /* 471 MB/s */
   ```

### Commit 2: ARM: dts: qcom: tenderloin: Remove USB bandwidth voting

Removed explicit USB interconnect bandwidth properties since:
- WebOS didn't use explicit bandwidth voting for USB
- WebOS used `dfab_usb_hs_clk` as a fabric clock voter instead
- With MDP now using SMI, there's no EBI contention anyway

### Driver Changes (drm/msm/disp/mdp4/mdp4_kms.c)

1. Added `of_reserved_mem_device_init()` to use SMI reserved region for framebuffers
2. Updated interconnect name lookup to support both "mdp0-smi" and "mdp0-mem"

### Config Changes (tenderloin_debug_defconfig)

Disabled CMA to force SMI usage (matching webOS):
```
# CONFIG_CMA is not set
# CONFIG_DMA_CMA is not set
```

## Test Results

### Failed Approaches (Bandwidth Tuning)

| Test | MDP BW | USB BW | Result |
|------|--------|--------|--------|
| Test 1 | 334/417 MB/s (home) | 300/600 MB/s | USB crashes |
| Test 2 | 377/471 MB/s (app) | 300/600 MB/s | USB crashes |
| Test 3 | 377/471 MB/s | 400/800 MB/s | USB crashes |
| Test 4 | 377/471 MB/s | 600/1200 MB/s | USB still crashes |

**Conclusion**: Bandwidth tuning alone cannot fix the contention - both paths share EBI.

### Successful Approach (SMI Memory)

- MDP successfully allocates framebuffer from SMI reserved region
- Kernel log shows: `assigned reserved memory node framebuffer@38300000`
- Display works: 2x Tux penguins visible, fbcon active
- **Pending**: USB stability testing with DRM applications

## Next Steps

1. **Deploy and test** the SMI configuration with DRM tests:
   - modetest pattern display
   - kmscube hardware rendering
   - Verify USB stability during display activity

2. **If USB still crashes** (unlikely), investigate:
   - Z180 GPU memory path (currently still uses EBI)
   - Consider routing GPU to SMI as well

3. **Verify memory sizing**:
   - 61MB should be sufficient for dual 1024x768x4 framebuffers (~6MB each)
   - Monitor if larger allocations are needed

## Architecture Diagram (Final)

```
                    +------------------+
                    |   APPSS Fabric   |
                    +--------+---------+
                             |
               +-------------+-------------+
               |                           |
        +------+------+            +-------+-------+
        | MMSS Fabric |            | System Fabric |
        +------+------+            +-------+-------+
               |                           |
     +---------+---------+                 |
     |                   |                 |
+----+----+        +-----+-----+    +------+------+
|   SMI   |        |    EBI    |    |     USB     |
| 64MB    |        | Main RAM  |    | (no longer  |
| (MDP)   |        | (CPU,GPU) |    |  competing) |
+---------+        +-----------+    +-------------+
```

## References

- webOS kernel: `kernel-3.0.5.txt`
  - pmem_smipool: lines around board-tenderloin.c
  - MDP bus scaling: lines 320886+
  - dfab_usb_hs_clk voter: line 347546
- Commits:
  - `181b2a06d4c1` ARM: dts: qcom: tenderloin: Use SMI memory for display framebuffers
  - `c33d3be62f7d` ARM: dts: qcom: tenderloin: Remove USB bandwidth voting
