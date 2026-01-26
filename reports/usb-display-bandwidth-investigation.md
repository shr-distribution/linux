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

---

## UPDATE: GPU (Adreno) USB Contention (2026-01-25)

### Problem

MDP display works with USB (thanks to SMI routing), but **GPU (Adreno 220) 3D rendering still kills USB**. The GPU uses EBI memory, not SMI.

### Root Cause: GPU devfreq Stuck at Minimum

GPU devfreq (`simple_ondemand` governor) doesn't scale frequency:
- GPU stuck at 27 MHz (minimum)
- Interconnect bandwidth stuck at 170 kBps
- At this low bandwidth, GPU EBI access starves USB

WebOS handled this via `ebi1_kgsl_clk` clock voter system:
```c
CLK_VOTER("ebi1_kgsl_clk", EBI_KGSL_CLK, "ebi1_clk", NULL, 0),
CLK_VOTER("ebi1_usb_clk", EBI_USB_CLK, "ebi1_clk", NULL, 0),
```

### Why devfreq Doesn't Scale

The `df->suspended` flag stays true, causing `msm_devfreq_get_dev_status()` to always return 0% utilization:

1. devfreq starts suspended in `msm_devfreq_init()`
2. `msm_devfreq_resume()` should clear the flag via `adreno_runtime_resume()`
3. But something prevents proper resume cycle

### Workaround: Performance Governor

```bash
echo performance > /sys/class/devfreq/4300000.adreno/governor
```

Results:
- GPU at 320 MHz (max)
- Interconnect at 2008 MBps
- USB remains stable

### GPU Bandwidth OPP Table

| Frequency | Bandwidth | Power |
|-----------|-----------|-------|
| 27 MHz | 170 kBps | Minimum |
| 320 MHz | 2008 kBps | Maximum |

### Recommendation

Until devfreq is fixed, use one of:
1. **Performance governor** for GPU (wastes power)
2. **Avoid 3D GPU usage** while USB is needed
3. **Route GPU to SMI** (not yet implemented)

See `usb_drm_coexistence_investigation.md` for full technical details.

---

## UPDATE: Display Output Cross-Check (2026-01-25)

### Problem: Blue Vertical Lines

When running kmscube or modetest, display shows **blue vertical lines** instead of expected content. This indicates a display format configuration mismatch.

### webOS vs Mainline Configuration Cross-Check

| Setting | webOS (lcdc_lg_xga.c) | Mainline (panel-lvds) |
|---------|----------------------|----------------------|
| Panel BPP | 18 (pinfo->bpp) | 24 (data-mapping: vesa-24) |
| Output Format | RGB565 (default) | RGB888 (bpc=8) |
| Pack Align | **LSB** | **MSB** (default) |
| Pack Pattern | 0x21 (RGB) | 0x21 (RGB) |
| Dithering | Enabled | Disabled for 8bpc |
| Pixel Clock | 96 MHz | 96 MHz |

### Key Findings

#### 1. Output Bit Depth

webOS used **16-bit output** (RGB565: G=6, B=5, R=5) despite panel declaring 18bpp:

```c
// webOS mdp_lcdc.c lines 160-163
if (fb_panel->fb_data->output_format == MSM_MDP_OUT_IF_FMT_RGB666)
    dma_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;  // 18-bit
else
    dma_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;  // 16-bit (default)
```

Mainline uses **24-bit output** (RGB888: R=8, G=8, B=8) for "vesa-24" mapping.

#### 2. Pack Alignment (CRITICAL DIFFERENCE)

webOS used **LSB alignment** while mainline uses **MSB alignment** by default:

```c
// webOS mdp_lcdc.c line 154
dma_cfg |= (DMA_PACK_ALIGN_LSB | DMA_PACK_PATTERN_RGB | DMA_DITHER_EN);
```

The webOS mdp.h comment explains:
```c
/* use DMA_PACK_ALIGN_MSB if the upper 6 bits from 8 bits output
 * from LCDC block maps into 6 pins out to the panel */
#define DMA_PACK_ALIGN_MSB  BIT(7)
```

Mainline defaults to MSB unless `qcom,lcdc-align-lsb` DT property is present.

#### 3. Physical Data Lines

The TouchPad has **24 physical data lines** (8 per channel):
- lcdc_red7 through lcdc_red0 (GPIO 4-11)
- lcdc_grn7 through lcdc_grn0 (GPIO 12-19)
- lcdc_blu7 through lcdc_blu0 (GPIO 20-27)

This confirms hardware supports 24-bit, but webOS chose 16-bit output (likely for bandwidth).

### Potential Fix

Add `qcom,lcdc-align-lsb` property to device tree to match webOS LSB alignment:

```dts
&mdp {
    qcom,lcdc-align-lsb;
};
```

Or try 18-bit mode to match webOS more closely:

```dts
lg_panel: panel {
    data-mapping = "jeida-18";  // Instead of "vesa-24"
};
```

### Files Modified for bpc Fix

Added bpc field to panel-lvds.c to properly communicate bit depth to MDP4:

```c
// drivers/gpu/drm/panel/panel-lvds.c
struct panel_lvds {
    ...
    unsigned int bpc;  /* bits per color channel (6, 8, or 10) */
};

// In panel_lvds_parse_dt():
switch (bus_format) {
case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
    lvds->bpc = 6;
    break;
case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
    lvds->bpc = 8;
    break;
...
}

// In panel_lvds_get_modes():
connector->display_info.bpc = lvds->bpc;
```

### Next Steps

1. **Try LSB alignment**: Add `qcom,lcdc-align-lsb` to MDP DT node
2. **Try 18-bit mode**: Change data-mapping to "jeida-18"
3. **Debug with modetest**: Use simpler test pattern to isolate issue
4. **Check LVDS timing**: Verify LVDS serializer is properly initialized

---

## UPDATE: webOS EBI Clock Voter System Analysis (2026-01-25)

### Key Discovery: Dual-Bus Architecture

The webOS kernel used a **dual-bus architecture** that fundamentally separated USB traffic from GPU/MDP traffic:

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
     +---------+---------+        +--------+--------+
     |         |         |        |        |        |
+----+----+  +-+---+  +--+--+  +--+---+  +-+--+  +--+---+
|   SMI   |  | EBI |  | GPU |  | DFAB |  | USB|  | ADM  |
| 64MB    |  | RAM |  | 3D  |  | Clk  |  |    |  |      |
+---------+  +-----+  +-----+  +------+  +----+  +------+
```

### CLK_VOTER System: Take-The-Maximum Algorithm

webOS used a clock voter system defined in `arch/arm/mach-msm/clock-voter.c`:

```c
// Each peripheral votes on parent clock frequency
struct clk_voter {
    unsigned count;              // Enable/disable reference count
    unsigned rate;               // Requested rate by this voter
    struct clk *aggregator_clk;  // Parent clock (ebi1_clk or dfab_clk)
};

// Aggregation: parent runs at MAX of all active voters
static unsigned voter_clk_aggregate_rate(const struct clk *parent) {
    unsigned rate = 0;
    hlist_for_each_entry(clkh, pos, &parent->voters, voter_list)
        if (clkh->count)
            rate = max(clkh->rate, rate);
    return rate;
}
```

### Critical: USB Used DFAB, Not EBI

**This is the key insight**: USB voted on `dfab_clk` (Daytona Fabric), completely separate from EBI:

```c
// From kernel-3.0.5.txt line 347546
CLK_VOTER("dfab_usb_hs_clk", DFAB_USB_HS_CLK, "dfab_clk", NULL, 0),

// USB platform data (line 292929)
static struct msm_otg_platform_data msm_otg_pdata = {
    /* if usb link is in sps there is no need for
     * usb pclk as dayatona fabric clock will be used instead */
    .pclk_src_name = "dfab_usb_hs_clk",
    ...
};
```

**DFAB Voters** (independent from EBI):
- `dfab_usb_hs_clk` - USB
- `dfab_dsps_clk` - Digital signal processor
- `dfab_sdc_clk` - SD card controllers

**EBI Voters** (memory bandwidth):
- `ebi1_kgsl_clk` - GPU
- `ebi1_lcdc_clk` - Display (LCDC)
- `ebi1_mdp_clk` - MDP
- `ebi1_usb_clk` - USB (secondary, for DMA)
- `ebi1_msmbus_clk` - Bus fabric
- `ebi1_adm_clk` - ADM DMA engine

### MDP Bus Scaling: Dual-Path Voting

webOS MDP voted on **BOTH** SMI and EBI simultaneously (kernel-3.0.5.txt lines 326670-326827):

```c
// MDP votes on SMI for framebuffer access
static struct msm_bus_vectors mdp_app_vectors[] = {
    {
        .src = MSM_BUS_MASTER_MDP_PORT0,
        .dst = MSM_BUS_SLAVE_SMI,
        .ab = 377487360,   // 360 MB/s average
        .ib = 471859200,   // 450 MB/s peak
    },
    // MDP ALSO votes on EBI for compositing/blitting
    {
        .src = MSM_BUS_MASTER_MDP_PORT0,
        .dst = MSM_BUS_SLAVE_EBI_CH0,
        .ab = 377487360,
        .ib = 471859200,
    },
};
```

**Use Cases** (mdp_bus_scale_usecases):
| Index | Use Case | Bandwidth (avg/peak) |
|-------|----------|---------------------|
| 0 | init | 0/0 |
| 1 | home | 334/417 MB/s (1 layer) |
| 2 | app | 377/471 MB/s (2 layers) |
| 3 | lowres | 414/518 MB/s (720x576 video) |
| 4 | pip | 432/540 MB/s (2 VGA layers) |
| 5 | 720p | 460/575 MB/s |
| 6 | 1080p | 564/705 MB/s |

### GPU Bus Scaling: EBI Only

GPU voted on EBI only, not SMI (kernel-3.0.5.txt lines 346271-346304):

```c
static struct msm_bus_vectors grp3d_max_vectors[] = {
    {
        .src = MSM_BUS_MASTER_GRAPHICS_3D,
        .dst = MSM_BUS_SLAVE_EBI_CH0,
        .ab = 2008000000U,  // 2 GB/s (!!)
        .ib = 2008000000U,
    },
};
```

### What Mainline Kernel is Missing

| Feature | webOS | Mainline | Impact |
|---------|-------|----------|--------|
| USB clock source | dfab_usb_hs_clk (DFAB voter) | Interconnect framework | USB competes with GPU/MDP on EBI |
| MDP EBI voting | Votes on both SMI AND EBI | Only votes on SMI | Missing secondary bandwidth guarantee |
| GPU devfreq | Proper KGSL_PWRFLAGS_AXI_ON/OFF | Stuck at minimum | GPU bandwidth vote stuck at 170 kBps |
| CLK_VOTER aggregation | Take-the-maximum algorithm | Not implemented | No automatic bandwidth scaling |
| DFAB independence | Separate fabric clock | Not separated | USB shares interconnect with GPU |

### Recommended Fixes

1. **Add DFAB clock for USB**:
   - USB should use a separate fabric clock, not compete for EBI bandwidth
   - Need to add `dfab_clk` voter support to the clock driver

2. **Fix GPU devfreq**:
   - GPU devfreq is stuck at minimum (27 MHz / 170 kBps)
   - Need to fix `msm_devfreq_resume()` to clear suspended flag
   - Alternative: force performance governor

3. **MDP dual-path voting**:
   - Add EBI bandwidth voting alongside SMI voting
   - Current interconnect only votes on MMSS fabric

4. **Implement CLK_VOTER equivalent**:
   - Add take-the-maximum aggregation for shared clocks
   - This ensures all consumers get sufficient bandwidth

### Interim Workaround

Until proper fixes are implemented:
```bash
# Force GPU to max frequency (fixes devfreq)
echo performance > /sys/class/devfreq/4300000.adreno/governor

# This ensures GPU votes for 2 GB/s on EBI
# USB gets adequate bandwidth as a side effect
```

---

## References

- webOS kernel: `kernel-3.0.5.txt`
  - CLK_VOTER system: lines 337423-337433, 347544-347551
  - clock-voter.c implementation: lines 93400-93530
  - MDP bus scaling vectors: lines 326670-326827
  - GPU bus scaling: lines 346271-346339
  - USB dfab_usb_hs_clk: lines 292924-292929, 322018-322023
  - pmem_smipool: lines around board-tenderloin.c
  - dfab_usb_hs_clk voter: line 347546
- Commits:
  - `181b2a06d4c1` ARM: dts: qcom: tenderloin: Use SMI memory for display framebuffers
  - `c33d3be62f7d` ARM: dts: qcom: tenderloin: Remove USB bandwidth voting

---

## UPDATE: Bandwidth Values Comparison (2026-01-26)

### Analysis: Legacy webOS vs Current Mainline Bandwidth Values

#### MDP (Display) Bandwidth - CORRECT ✓

| Use Case | Legacy ab/ib (B/s) | Legacy (kBps) | Current (kBps) | Status |
|----------|-------------------|---------------|----------------|--------|
| mdp_home (1 layer) | 334,080,000 / 417,600,000 | 326,250 / 407,812 | - | |
| **mdp_app (2 layers)** | 377,487,360 / 471,859,200 | **368,640 / 460,800** | **368,640 / 460,800** | ✓ MATCH |
| mdp_720p | 460,431,360 / 575,539,200 | 449,640 / 562,050 | - | |
| mdp_1080p | 564,111,360 / 705,139,200 | 550,890 / 688,610 | - | |

**MDP values are correct** - Current kernel matches legacy mdp_app_vectors exactly.

**MDP paths**: Legacy voted on 2 paths (SMI + EBI). Current DT defines 4 paths (mdp0-smi, mdp1-smi, mdp0-ebi, mdp1-ebi) and votes on all 4. The "8 votes" seen in interconnect_summary is due to interconnect framework internal bookkeeping (forward/reverse paths), but bandwidth values are correct.

#### GPU Bandwidth - DEVFREQ ISSUE ⚠️

| Condition | Legacy (B/s) | Legacy (kBps) | Current (kBps) | Status |
|-----------|-------------|---------------|----------------|--------|
| Idle | 0 | 0 | 0 | ✓ |
| Min (27 MHz) | - | - | 170,000 | ← STUCK HERE |
| Max (320 MHz) | 2,008,000,000 | ~1,960,000 | 2,008,000 | ✓ Correct OPP |

**GPU stuck at 170 kBps** - This indicates devfreq is stuck at 27 MHz (minimum frequency). The OPP table correctly defines 2,008,000 kBps at 320 MHz. Problem: devfreq governor sees 0% utilization because `df->suspended` flag stays true.

**Root cause**: `msm_devfreq_resume()` never called because GPU pm_runtime doesn't wake up unless 3D app explicitly requests it.

#### USB Bandwidth - FIXED ✓

| Component | Legacy | Previous (Excessive) | Current (Fixed) |
|-----------|--------|---------------------|-----------------|
| USB->EBI bandwidth | **NO voting** | 307,200 / 614,400 kBps | **61,440 / 61,440 kBps** |
| DFAB clock voter | dfab_usb_hs_clk | 131,072 kBps | 131,072 kBps |

**FIXED**: Reduced USB->EBI bandwidth from 300/600 MB/s to 60/60 MB/s (USB 2.0 HS max).

Legacy webOS did **NOT** use explicit bandwidth voting for USB->EBI memory path. It only used `dfab_usb_hs_clk` as a clock voter to keep DFAB running. The excessive 300/600 MB/s voting was competing with MDP for fabric priority and causing display underflows.

### Changes Made (2026-01-26)

**drivers/usb/chipidea/ci_hdrc_msm.c**:
```c
/* Before: Excessive bandwidth competing with MDP */
#define USB_HS_DEFAULT_BW_AVG_KBPS  (300 * 1024)  /* 300 MB/s */
#define USB_HS_DEFAULT_BW_PEAK_KBPS (600 * 1024)  /* 600 MB/s */

/* After: Minimal bandwidth matching USB 2.0 HS max */
#define USB_HS_DEFAULT_BW_AVG_KBPS  (60 * 1024)   /* 60 MB/s (USB 2.0 max) */
#define USB_HS_DEFAULT_BW_PEAK_KBPS (60 * 1024)   /* 60 MB/s */
```

### Summary Table

| Component | Legacy | Current | Status |
|-----------|--------|---------|--------|
| MDP bandwidth | 368,640 / 460,800 kBps | 368,640 / 460,800 kBps | ✓ CORRECT |
| MDP paths | 2 (SMI + EBI) | 4 (mdp0/1-smi + mdp0/1-ebi) | ✓ OK |
| GPU max bandwidth | ~1,960,000 kBps | 2,008,000 kBps | ✓ CORRECT |
| GPU current | varies | 170,000 kBps (stuck) | ⚠️ DEVFREQ BUG |
| USB->EBI | NONE | 61,440 kBps | ✓ FIXED |
| USB DFAB voter | dfab_usb_hs_clk | 131,072 kBps | ✓ OK |

### Remaining Issue: GPU devfreq

GPU devfreq doesn't scale frequency, causing bandwidth to stay at minimum (170 kBps). This starves GPU 3D rendering and can cause issues.

**Workaround**:
```bash
echo performance > /sys/class/devfreq/4300000.adreno/governor
```

**Proper fix needed**: Modify `msm_devfreq_active()` to ensure devfreq is resumed when GPU activity is detected.
