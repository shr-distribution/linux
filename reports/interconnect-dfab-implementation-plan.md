# Interconnect DFAB Implementation Plan

**Date:** 2026-01-22
**Status:** Planning
**Related:** `reports/memory-bimodality-investigation.md`, `reports/interconnect_requirements.md`

---

## Executive Summary

Investigation of memory performance bimodality, slow eMMC/USB speeds, and USB disconnects during GPU activity has identified the root cause: **missing Daytona Fabric (DFAB) in the mainline interconnect driver** and **lack of bandwidth voting for key peripherals**.

This document outlines a 3-phase implementation plan to fix these issues.

---

## Problem Statement

### Symptoms Observed
1. **Bimodal RAM performance** - Two distinct speed levels during memory benchmarks
2. **USB killed by GPU activity** - USB device disconnects when GPU is active
3. **Slow USB speeds** - Below expected throughput
4. **Slow eMMC speeds** - Below expected throughput

### Root Cause Analysis

The webOS 2.6 kernel used two mechanisms for bus bandwidth management:

| Mechanism | Purpose | Mainline Status |
|-----------|---------|-----------------|
| `msm_bus_scale` API | High-bandwidth masters (GPU, Display) | Replaced by interconnect framework |
| Voter clocks (`dfab_*_clk`) | Peripherals (SDCC, USB, DMA) | **NOT IMPLEMENTED** |

#### Missing Components

1. **DFAB (Daytona Fabric)** - A real hardware fabric connecting peripherals to system fabric
   - Has dedicated RPM clock: `RPM_DAYTONA_FABRIC_CLK`
   - Has bridge clocks: `SFAB_DFAB_M_A_CLK`, `DFAB_SFAB_M_A_CLK`
   - Has reset lines in GCC
   - **Not present in mainline interconnect driver**

2. **GPU bandwidth voting** - Adreno 220 generates ~2 GB/s memory traffic
   - webOS: Used `msm_bus_scale` with 2 GB/s instantaneous bandwidth
   - Mainline: No bandwidth vote (relies on clock scaling alone)

3. **eMMC bandwidth voting** - mmci driver has no interconnect support
   - webOS: Used `dfab_sdc1_clk` voter clock (64 MHz DFAB vote)
   - Mainline: No bandwidth vote at all

#### Fabric Architecture

```
                    +------------------+
                    |   APPSS Fabric   |  (CPU, L2, Memory - EBI_CH0)
                    +--------+---------+
                             |
           +-----------------+-----------------+
           |                 |                 |
   +-------v-------+ +-------v-------+ +-------v-------+
   |  MMSS Fabric  | | System Fabric | |    DFAB       |  <-- MISSING!
   |  (GPU, MDP,   | | (USB, LPASS,  | | (SDCC, ADM,   |
   |   Camera)     | |  Modem)       | |  Peripherals) |
   +---------------+ +-------+-------+ +---------------+
                             |
                     +-------v-------+
                     |   FPB Buses   |
                     | (SFPB, CFPB)  |
                     +---------------+
```

---

## Implementation Plan

### Phase 1: Quick Fix - Minimum Clock Floor
**Goal:** Validate theory and provide immediate improvement
**Risk:** Low
**Effort:** ~30 minutes

Add a minimum rate floor to prevent fabric clocks from dropping to minimum when no bandwidth is requested.

#### Changes Required

**File:** `drivers/interconnect/qcom/msm8660.c`

```c
/* Minimum fabric clock rates to prevent starvation */
#define MSM8660_AFAB_MIN_RATE   200000000   /* 200 MHz - APPSS fabric */
#define MSM8660_SFAB_MIN_RATE   128000000   /* 128 MHz - System fabric */
#define MSM8660_MMFAB_MIN_RATE  128000000   /* 128 MHz - MMSS fabric */

static int msm8660_icc_set(struct icc_node *src, struct icc_node *dst)
{
    ...
    rate = max(sum_bw, max_peak_bw);
    do_div(rate, src_qn->buswidth);

    /* Apply minimum floor to prevent fabric starvation */
    rate = max_t(u64, rate, MSM8660_AFAB_MIN_RATE);

    rate = min_t(u32, rate, INT_MAX);
    ...
}
```

#### Testing
1. Run `scripts/benchmark-ram.sh` - check for bimodality elimination
2. Test USB file transfer during GPU activity
3. Measure eMMC read/write speeds

---

### Phase 2: Proper Fix - Add DFAB Fabric
**Goal:** Implement correct architectural solution
**Risk:** Medium
**Effort:** ~2-3 hours

Add Daytona Fabric (DFAB) to the interconnect driver following MSM8974's PNOC pattern.

#### Changes Required

**File 1:** `include/dt-bindings/interconnect/qcom,msm8660.h`

Add DFAB node IDs:
```c
/* Daytona Fabric (DFAB) - Peripheral bus */
#define DFAB_MAS_SDC1           0
#define DFAB_MAS_SDC2           1
#define DFAB_MAS_SDC3           2
#define DFAB_MAS_SDC4           3
#define DFAB_MAS_SDC5           4
#define DFAB_MAS_ADM0           5
#define DFAB_MAS_ADM1           6
#define DFAB_TO_SFAB            7
#define DFAB_SLV_DFAB           8
```

**File 2:** `drivers/interconnect/qcom/msm8660.c`

Add DFAB fabric definition:
```c
/* DFAB clock configuration */
static const struct clk_bulk_data msm8660_dfab_clocks[] = {
    { .id = "bus" },      /* RPM_DAYTONA_FABRIC_CLK */
    { .id = "bus_a" },    /* RPM_DAYTONA_FABRIC_A_CLK */
};

/* DFAB nodes - Daytona Fabric (peripheral bus) */
DEFINE_QNODE(dfab_mas_sdc1, MSM8660_DFAB_MAS_SDC1, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_mas_sdc2, MSM8660_DFAB_MAS_SDC2, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_mas_sdc3, MSM8660_DFAB_MAS_SDC3, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_mas_sdc4, MSM8660_DFAB_MAS_SDC4, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_mas_sdc5, MSM8660_DFAB_MAS_SDC5, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_mas_adm0, MSM8660_DFAB_MAS_ADM0, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_mas_adm1, MSM8660_DFAB_MAS_ADM1, 8, MSM8660_DFAB_TO_SFAB);
DEFINE_QNODE(dfab_to_sfab, MSM8660_DFAB_TO_SFAB, 8,
             MSM8660_SFAB_TO_DFAB, MSM8660_AFAB_SLV_EBI_CH0);
DEFINE_QNODE(dfab_slv_dfab, MSM8660_DFAB_SLV_DFAB, 8);

static struct msm8660_icc_node * const msm8660_dfab_nodes[] = {
    [DFAB_MAS_SDC1] = &dfab_mas_sdc1,
    [DFAB_MAS_SDC2] = &dfab_mas_sdc2,
    [DFAB_MAS_SDC3] = &dfab_mas_sdc3,
    [DFAB_MAS_SDC4] = &dfab_mas_sdc4,
    [DFAB_MAS_SDC5] = &dfab_mas_sdc5,
    [DFAB_MAS_ADM0] = &dfab_mas_adm0,
    [DFAB_MAS_ADM1] = &dfab_mas_adm1,
    [DFAB_TO_SFAB] = &dfab_to_sfab,
    [DFAB_SLV_DFAB] = &dfab_slv_dfab,
};

static const struct msm8660_icc_desc msm8660_dfab = {
    .nodes = msm8660_dfab_nodes,
    .num_nodes = ARRAY_SIZE(msm8660_dfab_nodes),
    .bus_clks = msm8660_dfab_clocks,
    .num_clks = ARRAY_SIZE(msm8660_dfab_clocks),
};
```

Add SFAB↔DFAB gateway nodes to System Fabric:
```c
/* Add to SFAB nodes */
DEFINE_QNODE(sfab_to_dfab, MSM8660_SFAB_TO_DFAB, 8, MSM8660_DFAB_TO_SFAB);
```

Add compatible string:
```c
static const struct of_device_id msm8660_noc_of_match[] = {
    { .compatible = "qcom,msm8660-apps-fabric", .data = &msm8660_afab },
    { .compatible = "qcom,msm8660-system-fabric", .data = &msm8660_sfab },
    { .compatible = "qcom,msm8660-mmss-fabric", .data = &msm8660_mmfab },
    { .compatible = "qcom,msm8660-daytona-fabric", .data = &msm8660_dfab },  /* NEW */
    { },
};
```

**File 3:** `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi`

Add DFAB node:
```dts
daytona_fabric: interconnect@3 {
    compatible = "qcom,msm8660-daytona-fabric";
    #interconnect-cells = <1>;
    clocks = <&rpmcc RPM_DAYTONA_FABRIC_CLK>,
             <&rpmcc RPM_DAYTONA_FABRIC_A_CLK>;
    clock-names = "bus", "bus_a";
};
```

#### Testing
1. Verify DFAB probes successfully (check dmesg)
2. Verify cross-fabric paths work (DFAB → SFAB → AFAB → EBI)
3. Run RAM benchmark to verify no regression

---

### Phase 3: Complete Fix - mmci Interconnect Support
**Goal:** Enable eMMC to request bandwidth properly
**Risk:** Medium
**Effort:** ~2-3 hours

Add interconnect framework support to the ARM PL18x (mmci) driver.

#### Changes Required

**File 1:** `drivers/mmc/host/mmci.c`

Add interconnect support:
```c
#include <linux/interconnect.h>

struct mmci_host {
    ...
    struct icc_path *icc_path;
};

static int mmci_probe(struct amba_device *dev, const struct amba_id *id)
{
    ...
    /* Get interconnect path (optional) */
    host->icc_path = devm_of_icc_get(&dev->dev, "sdc-mem");
    if (IS_ERR(host->icc_path)) {
        ret = PTR_ERR(host->icc_path);
        if (ret != -ENODATA)
            return ret;
        host->icc_path = NULL;
    }

    /* Request bandwidth if path exists */
    if (host->icc_path) {
        /* 64 MB/s average, 128 MB/s peak for HS mode */
        ret = icc_set_bw(host->icc_path,
                         MBps_to_icc(64), MBps_to_icc(128));
        if (ret)
            dev_warn(&dev->dev, "Failed to set ICC bandwidth\n");
    }
    ...
}

static void mmci_remove(struct amba_device *dev)
{
    ...
    if (host->icc_path)
        icc_set_bw(host->icc_path, 0, 0);
    ...
}
```

**File 2:** `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi`

Update SDCC nodes with interconnect paths:
```dts
sdcc1: mmc@12400000 {
    ...
    interconnects = <&daytona_fabric DFAB_MAS_SDC1
                     &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "sdc-mem";
};

sdcc4: mmc@121c0000 {
    ...
    interconnects = <&daytona_fabric DFAB_MAS_SDC4
                     &apps_fabric AFAB_SLV_EBI_CH0>;
    interconnect-names = "sdc-mem";
};
```

#### Testing
1. Verify mmci probes with interconnect path
2. Run eMMC benchmarks (`hdparm -t`, `dd` tests)
3. Compare speeds before/after

---

## Validation Checklist

### Phase 1 Validation
- [ ] RAM benchmark shows stable performance (no bimodality)
- [ ] Fabric clocks stay above minimum (check debugfs)
- [ ] No USB disconnects during idle

### Phase 2 Validation
- [ ] DFAB interconnect probes successfully
- [ ] `dmesg | grep -i dfab` shows registration
- [ ] Cross-fabric paths resolve correctly
- [ ] No performance regression

### Phase 3 Validation
- [ ] mmci probes with interconnect path
- [ ] eMMC speeds improved
- [ ] USB stable during eMMC + GPU activity
- [ ] System stable under combined load

---

## Risk Assessment

| Phase | Risk | Mitigation |
|-------|------|------------|
| 1 | Low | Simple change, easy to revert |
| 2 | Medium | May affect existing interconnect paths; test thoroughly |
| 3 | Medium | mmci is upstream driver; changes need careful review |

---

## Upstream Submission Plan

1. **Phase 1** - Local only (workaround, not for upstream)
2. **Phase 2** - Submit as patch series:
   - Patch 1: dt-bindings: interconnect: qcom: Add MSM8660 DFAB nodes
   - Patch 2: interconnect: qcom: msm8660: Add Daytona Fabric support
   - Patch 3: arm: dts: qcom: msm8660: Add Daytona Fabric node
3. **Phase 3** - Submit separately:
   - Patch 1: mmc: mmci: Add interconnect framework support
   - Patch 2: arm: dts: qcom: msm8660: Add SDCC interconnect paths

---

## References

- MSM8974 PNOC implementation: `drivers/interconnect/qcom/msm8974.c`
- webOS bus scaling: `arch/arm/mach-msm/msm_bus_board_8660.c`
- webOS voter clocks: `arch/arm/mach-msm/clock-voter.c`
- Interconnect framework: `Documentation/interconnect/`

---

## Appendix: Legacy webOS Bandwidth Values

From `devices-msm8x60.c` and `clock-voter.c`:

| Component | Average BW | Peak BW | Fabric |
|-----------|------------|---------|--------|
| GPU 3D | 0 | 2096 MB/s | MMSS |
| GPU 2D | 128 MB/s | 256 MB/s | MMSS |
| MDP | 500 MB/s | 700 MB/s | MMSS |
| USB HS | MAX | MAX | DFAB |
| SDC1 (eMMC) | 64 MHz vote | - | DFAB |
| ADM DMA | 27 MHz vote | - | EBI |
