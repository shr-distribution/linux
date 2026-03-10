# Debug Logging Audit Report

Audit of development/debug logging added by tenderloin commits that should be
removed or downgraded before upstream submission.

## CRITICAL: Hot-path logging (performance impact)

### drivers/input/touchscreen/cy8ctma395_ts.c

Touch data path logging that fires during normal use:

| Line | Rate-limited? | Message |
|------|---------------|---------|
| 389 | 5s | `pr_info("cy8ctma395: calc_point called, ...")` |
| 458 | **NO** | `pr_info("cy8ctma395: touch detected at ...")` |
| 590 | **NO** | `pr_info("cy8ctma395: reporting %d touch(es)")` |
| 670 | 5s | `pr_info("cy8ctma395: valid frame ...")` |
| 689 | 5s | `dev_info("UART RX: %zu bytes total, ...")` |
| 712 | 10s | `pr_info("cy8ctma395: data analysis ...")` + `print_hex_dump()` |
| 694 | one-shot | `print_hex_dump("cy8ctma395 first RX: ...")` |

**Action**: Remove all, or convert to `dev_dbg`/`pr_debug`.

### drivers/mmc/host/mmci.c

DMA transfer and IRQ path logging on every I/O operation:

| Line | Context | Message |
|------|---------|---------|
| 632-641 | `mmci_start_data()` | `dev_info("DMA#%u ...")` with static counter - every transfer |
| 1623-1631 | `mmci_data_irq()` | `dev_info("DATAEND#%u ...")` with static counter - every completion |
| 2098-2107 | `mmci_irq()` | `dev_info("SDIO IRQ in raw=...")` - capped at 3 prints |
| 2373 | `mmci_enable_sdio_irq()` | `dev_info("mmci_enable_sdio_irq called: ...")` - every toggle |

**Action**: Remove all. The DMA/DATAEND counters are especially damaging to
MMC/WiFi I/O performance.

### sound/soc/qcom/qdsp6/q6asm.c

Audio buffer write path:

| Line | Message |
|------|---------|
| ~write path | `dev_info("Legacy write: phys=0x%x len=%d buf=%d")` - every buffer |
| ~write done | `dev_info("Legacy write done: token=0x%x")` - every completion |

**Action**: Remove or downgrade to `dev_dbg`.

### sound/soc/qcom/qdsp6/q6afe.c, q6adm.c

| Lines | Message |
|-------|---------|
| various | `dev_info("Legacy AFE port start: ...")` |
| various | `dev_info("Legacy AFE config: ...")` |
| various | `dev_info("Legacy ADM COPP open: ...")` |

**Action**: Downgrade to `dev_dbg`.

## HIGH: Probe-path excess logging (boot log noise)

### drivers/gpu/drm/msm/msm_drv.c

| Line | Message | Action |
|------|---------|--------|
| 114 | `dev_info("MSM_DRV: msm_drm_init start")` | Remove |
| 165 | `dev_info("MSM_DRV: calling msm_drm_kms_init")` | Remove |
| 178 | `dev_warn("MSM_DRV: component_bind_all failed: ...")` | Keep (error path) |
| 181-187 | `#if 0` disabled `msm_gem_shrinker_init` | Re-enable (see below) |
| 1037 | `dev_info("MSM_DRV: bind - calling msm_drm_init")` | Remove |

**Dead code**: `msm_gem_shrinker_init()` was disabled with `#if 0` during early
bring-up due to a misdiagnosed USB failure. The actual USB issue was missing bus
fabric arbitration (AXI bandwidth), which has since been fixed by the MSM8660
interconnect driver (`drivers/interconnect/qcom/msm8660.c`). The shrinker is
safe to re-enable. The cleanup function `msm_gem_shrinker_cleanup()` at line 88
is already guarded by `if (priv->shrinker)` so it's a safe no-op when init is
skipped, but it should be properly paired now.

### drivers/gpu/drm/msm/disp/mdp4/mdp4_lvds_pll.c

| Line | Message | Action |
|------|---------|--------|
| 167 | `dev_info("mdp4_get_lcdc_clock: trying devm_clk_get")` | Remove |
| 175 | `dev_info("mdp4_get_lcdc_clock: got LCDC clock from device tree")` | Remove |
| 179 | `dev_info("mdp4_get_lcdc_clock: devm_clk_get failed: ...")` | Downgrade to `dev_dbg` |

### drivers/dma/qcom/qcom_adm.c

| Line | Message | Action |
|------|---------|--------|
| 253 | `dev_info("ADM descriptor pool: %d descs, ...")` | Downgrade to `dev_dbg` |
| 1122 | `dev_info("ADM probe: base=%p EE=%d, initial CH_CONF[0]=...")` | Remove |
| 1150 | `dev_info("ADM probe: after init CH_CONF[0]=... RSLT_CONF[0]=...")` | Remove |

### drivers/mmc/host/mmci_qcom_dml.c

| Line | Message | Action |
|------|---------|--------|
| 186 | `dev_info("ADM DMA: enabled (no DML)")` | Remove |
| 276 | `dev_info("qcom_variant_init: setting qcom_variant_ops")` | Remove |

### drivers/mmc/host/mmci.c (probe path)

| Line | Message | Action |
|------|---------|--------|
| 2602 | `dev_info("interconnect bandwidth voting enabled")` | Remove |
| 2713 | `dev_info("SDIO IRQ check: variant_support=...")` | Remove |
| 2722 | `dev_info("SDIO IRQ enabled: ops->enable_sdio_irq=...")` | Remove |

### drivers/input/touchscreen/cy8ctma395_ts.c (probe path)

15 step-by-step `dev_info` calls logging GPIO/I2C init sequence (lines 782-843).
Standard upstream practice is a single probe-completion message.

**Action**: Remove all except the final "CY8CTMA395 touchscreen initialized".

### drivers/leds/leds-lm8502.c

| Line | Message | Action |
|------|---------|--------|
| 188 | `dev_info("I2C communication OK, ENGINE_CNTRL1=...")` | Remove |
| 241-247 | `dev_info("After init: ENGINE_CNTRL1=... (expect ...)")` x4 | Remove |

Keep "Chip initialized, boost enabled" (250) and final probe message (405).

### drivers/clk/qcom/apcs-msm8660.c

| Line | Message | Action |
|------|---------|--------|
| 349 | `dev_info("APCS probe starting")` | Remove |

### drivers/gpu/drm/msm/z180/z180_gpu.c

| Line | Message | Action |
|------|---------|--------|
| 244 | `dev_info("Command stream initialized: rb@... mem@...")` | Downgrade to `dev_dbg` |
| 286 | `dev_info("Command stream started")` | Downgrade to `dev_dbg` |

### sound/soc/qcom/qdsp6/q6asm-dai.c

| Lines | Message | Action |
|-------|---------|--------|
| probe | `dev_info("q6asm-dais: registering component ...")` | Remove |
| probe | `dev_info("q6asm-dais: component registered ...")` | Remove |

## MEDIUM: Dead code / artifacts

| File | Line | Issue | Action |
|------|------|-------|--------|
| `cy8ctma395.c` | 1 | `//#define DEBUG` commented out | Remove |
| `msm_drv.c` | 181-187 | `#if 0` disabled shrinker | Re-enable |

## Summary

~71 logging statements to remove or downgrade across 13 files, plus 2 dead code
blocks to clean up.

**Cleaned up in this pass**: mmci.c, msm_drv.c, mdp4_lvds_pll.c, qcom_adm.c,
mmci_qcom_dml.c.

**Remaining for future cleanup**: cy8ctma395_ts.c (22 statements),
q6adm/afe/asm*.c (16 statements), leds-lm8502.c (5), apcs-msm8660.c (1),
z180_gpu.c (2), q6asm-dai.c (2), cy8ctma395.c (1 commented-out #define).
