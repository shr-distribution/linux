# HP TouchPad Upstream Patch Submission Plan

## Overview

Split the monolithic HP TouchPad commits into small, focused patches suitable
for Linux kernel mailing list submission. Patches are grouped by subsystem
and ordered by dependencies.

## Submission Order

Patches must be submitted in dependency order:
1. dt-bindings (headers) first
2. Driver changes second
3. Device tree nodes last (they depend on drivers/bindings)

---

## SERIES 1: Clock Infrastructure (8 patches)
**Maintainers:** Stephen Boyd, Michael Turquette (linux-clk@vger.kernel.org)
**Also CC:** linux-arm-msm@vger.kernel.org

### 1.1 dt-bindings: clock: qcom: gcc-msm8660: Add PLL4_VOTE clock
```
File: include/dt-bindings/clock/qcom,gcc-msm8660.h
Change: Add #define PLL4_VOTE
```

### 1.2 clk: qcom: gcc-msm8660: Add PLL4_VOTE clock support
```
File: drivers/clk/qcom/gcc-msm8660.c
Change: Add PLL4_VOTE clock for LPASS PLL voting
Depends: 1.1
```

### 1.3 dt-bindings: clock: qcom: Add LCC bindings for MSM8660
```
File: include/dt-bindings/clock/qcom,lcc-msm8660.h (NEW)
Change: Add LCC clock/reset bindings for MSM8660/APQ8060
```

### 1.4 clk: qcom: lcc-msm8960: Add MSM8660/APQ8060 support
```
File: drivers/clk/qcom/lcc-msm8960.c
Change: Add qcom,lcc-msm8660 compatible and MSM8660-specific handling
Depends: 1.3
```

### 1.5 dt-bindings: clock: qcom: mmcc-msm8960: Add MDP LCDC clocks
```
File: include/dt-bindings/clock/qcom,mmcc-msm8960.h
Change: Add MDP_PIXEL_SRC, MDP_PIXEL_CLK, MDP_LCDC_CLK (IDs 129-131)
```

### 1.6 clk: qcom: mmcc-msm8960: Add MDP pixel clock definitions
```
File: drivers/clk/qcom/mmcc-msm8960.c
Change: Add mdp_pixel_src, mdp_pixel_clk, mdp_lcdc_clk clock structures
Depends: 1.5
```

### 1.7 clk: qcom: mmcc-msm8960: Add MSM8660 clock array
```
File: drivers/clk/qcom/mmcc-msm8960.c
Change: Add mmcc_msm8660_clks[], mmcc_msm8660_resets[]
Depends: 1.6
```

### 1.8 clk: qcom: mmcc-msm8960: Add MSM8660 compatible
```
File: drivers/clk/qcom/mmcc-msm8960.c
Change: Add mmcc_msm8660_desc, compatible entry, PLL2 enable in probe
Depends: 1.7
```

---

## SERIES 2: DRM/GPU Fix (1 patch)
**Maintainers:** Rob Clark, Sean Paul (dri-devel@lists.freedesktop.org)
**Also CC:** linux-arm-msm@vger.kernel.org, freedreno@lists.freedesktop.org

### 2.1 drm/msm/a2xx: Add page table cache sync for non-coherent platforms
```
File: drivers/gpu/drm/msm/adreno/a2xx_gpummu.c
Change: Add dma_sync_single_for_device() calls in map/unmap
Note: Fixes GPU hangs on MSM8660 with PL310 L2 cache
```

---

## SERIES 3: Input - Cypress TTSP Enhancements (4 patches)
**Maintainers:** Dmitry Torokhov (linux-input@vger.kernel.org)

### 3.1 Input: cyttsp: Add power state management
```
Files: drivers/input/touchscreen/cyttsp_core.c, cyttsp_core.h
Change: Add CY_PWR_IDLE/ACTIVE/LOW/SLEEP states, power management functions
```

### 3.2 Input: cyttsp: Add IRQ counter validation
```
Files: drivers/input/touchscreen/cyttsp_core.c, cyttsp_core.h
Change: Add IRQ counting for firmware health monitoring
```

### 3.3 Input: cyttsp: Add enhanced touch tracking
```
Files: drivers/input/touchscreen/cyttsp_core.c, cyttsp_core.h
Change: Add position history tracking, spurious reset detection
```

### 3.4 Input: cyttsp: Add device tree properties for optional features
```
Files: drivers/input/touchscreen/cyttsp_core.c
Change: Add use-deep-sleep, disable-sleep, use-irq-counter, enhanced-tracking
```

---

## SERIES 4: Input - HP TouchPad Driver (1 patch)
**Maintainers:** Dmitry Torokhov (linux-input@vger.kernel.org)

### 4.1 Input: cy8ctma395: Add Cypress CY8CTMA395 touchscreen driver
```
Files: drivers/input/touchscreen/cy8ctma395.c (NEW)
       drivers/input/touchscreen/Kconfig
       drivers/input/touchscreen/Makefile
Change: Add driver for HP TouchPad touchscreen with SWD programmer mode
Note: Large driver, may need further splitting
```

---

## SERIES 5: Media - Camera Sensor (2 patches)
**Maintainers:** Mauro Carvalho Chehab (linux-media@vger.kernel.org)

### 5.1 media: i2c: Add VX6953 camera sensor driver
```
Files: drivers/media/i2c/vx6953.c (NEW)
       drivers/media/i2c/Kconfig
       drivers/media/i2c/Makefile
Change: Add STMicroelectronics VX6953 5.1MP EDOF camera sensor driver
```

### 5.2 media: qcom: camss: Add MSM8660 MIPI CSI-2 support
```
File: drivers/media/platform/qcom/camss/camss.c
Change: Add CSIPHY/CSID resources for MSM8660
```

---

## SERIES 6: Media - Video Codec (3 patches)
**Maintainers:** Mauro Carvalho Chehab (linux-media@vger.kernel.org)
**Also CC:** linux-arm-msm@vger.kernel.org
**Note:** This may be controversial - old hardware, proprietary firmware

### 6.1 media: qcom: vidc: Add VIDC 1080p core driver
```
Files: drivers/media/platform/qcom/vidc/vidc_core.c (NEW)
       drivers/media/platform/qcom/vidc/vidc_core.h (NEW)
       drivers/media/platform/qcom/vidc/Kconfig (NEW)
       drivers/media/platform/qcom/vidc/Makefile (NEW)
Change: Add core driver with firmware loading, clocks, IRQ handling
```

### 6.2 media: qcom: vidc: Add V4L2 M2M decoder
```
Files: drivers/media/platform/qcom/vidc/vidc_dec.c (NEW)
       drivers/media/platform/qcom/vidc/vidc_dec.h (NEW)
Change: Add H.264/MPEG4/H.263/MPEG2/VC1 decoder
Depends: 6.1
```

### 6.3 media: qcom: vidc: Add V4L2 M2M encoder
```
Files: drivers/media/platform/qcom/vidc/vidc_enc.c (NEW)
       drivers/media/platform/qcom/vidc/vidc_enc.h (NEW)
Change: Add H.264/MPEG4/H.263 encoder
Depends: 6.1
```

---

## SERIES 7: Remoteproc - LPASS Q6 (1 patch)
**Maintainers:** Bjorn Andersson, Mathieu Poirier (linux-remoteproc@vger.kernel.org)
**Also CC:** linux-arm-msm@vger.kernel.org

### 7.1 remoteproc: qcom: Add MSM8660 LPASS Q6v2 PIL driver
```
Files: drivers/remoteproc/qcom_q6v2_lpass.c (NEW)
       drivers/remoteproc/Kconfig
       drivers/remoteproc/Makefile
Change: Add QDSP6v2 peripheral image loader for MSM8660/APQ8060 LPASS
```

---

## SERIES 8: Device Tree - MSM8660 Base (5 patches)
**Maintainers:** Bjorn Andersson, Konrad Dybcio (linux-arm-msm@vger.kernel.org)
**Also CC:** devicetree@vger.kernel.org, Rob Herring

### 8.1 ARM: dts: qcom: msm8660: Add GSBI5 serial/i2c node
```
File: arch/arm/boot/dts/qcom/qcom-msm8660.dtsi
Change: Add gsbi5 node for GPS UART
```

### 8.2 ARM: dts: qcom: msm8660: Add LCC node
```
File: arch/arm/boot/dts/qcom/qcom-msm8660.dtsi
Change: Add lcc@28000000 LPASS clock controller node
Depends: Series 1 (LCC driver)
```

### 8.3 ARM: dts: qcom: msm8660: Add LPASS remoteproc node
```
File: arch/arm/boot/dts/qcom/qcom-msm8660.dtsi
Change: Add lpass@28800000 with reserved memory
Depends: Series 7 (LPASS driver)
```

### 8.4 ARM: dts: qcom: msm8660: Add VIDC video codec node
```
File: arch/arm/boot/dts/qcom/qcom-msm8660.dtsi
Change: Add video-codec@4400000 node
Depends: Series 6 (VIDC driver) - optional, can submit without driver
```

### 8.5 ARM: dts: qcom: msm8660: Add MIPI CSI-2 to CAMSS
```
File: arch/arm/boot/dts/qcom/qcom-msm8660.dtsi
Change: Add CSI0/CSI1 ports, clocks, registers to camss node
Depends: Series 5 (CAMSS patch)
```

---

## SERIES 9: Device Tree - HP TouchPad (3 patches)
**Maintainers:** Bjorn Andersson, Konrad Dybcio (linux-arm-msm@vger.kernel.org)
**Also CC:** devicetree@vger.kernel.org

### 9.1 ARM: dts: qcom: Add HP TouchPad common device tree
```
File: arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi (NEW)
Change: Add common dtsi for all TouchPad variants
Note: Large file (~2000 lines), well-documented
```

### 9.2 ARM: dts: qcom: Add HP TouchPad Topaz device trees
```
Files: arch/arm/boot/dts/qcom/qcom-apq8060-topaz.dts (NEW)
       arch/arm/boot/dts/qcom/qcom-apq8060-topaz-3g.dts (NEW)
       arch/arm/boot/dts/qcom/Makefile
Change: Add 9.7" TouchPad WiFi and 3G variants
Depends: 9.1
```

### 9.3 ARM: dts: qcom: Add HP TouchPad Opal device trees
```
Files: arch/arm/boot/dts/qcom/qcom-apq8060-opal.dts (NEW)
       arch/arm/boot/dts/qcom/qcom-apq8060-opal-3g.dts (NEW)
       arch/arm/boot/dts/qcom/Makefile
Change: Add 7" TouchPad Go WiFi and 3G variants
Depends: 9.1
```

---

## SERIES 10: Defconfig (1 patch)
**Maintainers:** Arnd Bergmann, Olof Johansson (linux-arm-kernel@lists.infradead.org)

### 10.1 ARM: configs: Add tenderloin_defconfig for HP TouchPad
```
File: arch/arm/configs/tenderloin_defconfig (NEW)
Change: Add kernel configuration for HP TouchPad
Note: Submit after device tree is accepted
```

---

## NOT FOR UPSTREAM (keep in out-of-tree branch)

These changes are development/debug specific and should not be submitted:

- Debug kernel command line (drm.debug, initcall_debug, etc.)
- CONFIG_CMDLINE_FORCE (bypasses bootloader)
- Full defconfig (minimal preferred for upstream)

---

## Submission Strategy

### Phase 1: Foundation (submit first)
1. Series 1 (Clocks) - No dependencies
2. Series 2 (DRM fix) - No dependencies

### Phase 2: Drivers (after Phase 1 accepted)
3. Series 3 (CYTTSP enhancements)
4. Series 4 (cy8ctma395 driver)
5. Series 5 (Camera)
6. Series 7 (Remoteproc)

### Phase 3: Device Trees (after Phase 2 accepted)
7. Series 8 (MSM8660 base DT)
8. Series 9 (TouchPad DTs)

### Phase 4: Config (after Phase 3 accepted)
9. Series 10 (defconfig)

### Maybe Later / Controversial
- Series 6 (VIDC) - Old hardware, needs discussion with maintainers

---

## Patch Format Requirements

1. Each patch must have:
   - Proper Signed-off-by
   - Detailed commit message explaining WHY
   - Reference to hardware documentation if available

2. Use `git format-patch` with:
   ```
   git format-patch --cover-letter -o patches/ <range>
   ```

3. Check with:
   ```
   ./scripts/checkpatch.pl patches/*.patch
   ```

4. Send with:
   ```
   git send-email --to=<maintainer> --cc=<lists> patches/
   ```

---

## Estimated Total: 28 patches across 10 series
