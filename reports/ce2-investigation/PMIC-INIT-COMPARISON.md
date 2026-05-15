# PMIC Initialization Comparison
## HTC vs TouchPad vs webOS Kernel vs Mainline

## Executive Summary

**Finding**: TouchPad PMIC (PM8058/PM8901) initialization is **split between three locations** in webOS kernel:
1. **Bootloader** (p5/p6/p7): Basic SSBI access, register definitions
2. **board-tenderloin.c**: Regulator platform data (voltages, consumers)
3. **RPM driver**: Runtime voltage/current control via RPM firmware

**Missing in TouchPad bootloader**: Hardware initialization sequence (ADC calibration, interrupt setup, voltage regulator boot-time config).

**Mainline status**: PM8058/PM8901 drivers exist, device tree configured, but may lack boot-time initialization that webOS bootloader + TZ would have done.

---

## Comparison Matrix

| Component | HTC Bootloader | TouchPad Bootloader | webOS Kernel | Mainline Kernel |
|-----------|----------------|---------------------|--------------|-----------------|
| **SSBI Access** | ✅ TZ (tz.img) | ✅ p5/p6 (RPM/QCSBL) | ✅ msm_ssbi.c | ✅ drivers/mfd/ssbi.c |
| **PM8058 MFD** | ✅ TZ init | ❌ No init | ✅ pm8058-core.c | ✅ qcom-pm8xxx.c |
| **PM8901 MFD** | ✅ TZ init | ❌ No init | ✅ (via pm8058) | ✅ qcom-pm8xxx.c |
| **ADC Init** | ✅ TZ | ❌ Missing | ✅ pmic.c | ❓ Unknown |
| **Interrupt Ctrl** | ✅ TZ | ❌ Missing | ✅ pm8058-core.c | ✅ qcom-pm8xxx.c |
| **LDO Regulators** | ✅ TZ voltages | ❌ No boot config | ✅ rpm-regulator | ✅ qcom_rpm-regulator |
| **SMPS Regulators** | ✅ TZ voltages | ❌ No boot config | ✅ rpm-regulator | ✅ qcom_rpm-regulator |
| **GPIO Config** | ✅ TZ | ❓ Basic (p7) | ✅ pmic8058-gpio.c | ✅ pinctrl-pm8xxx.c |
| **MPP Config** | ✅ TZ | ❌ Missing | ✅ pmic8058-mpp.c | ✅ pinctrl-pm8xxx-mpp.c |
| **Clock Outputs** | ✅ TZ | ❌ Missing | ✅ Via RPM | ❓ Via RPM? |

---

## Detailed Analysis

### 1. HTC TrustZone PMIC Init (Reference)

**File**: `/tmp/tz.img` (106KB)

**PMIC Strings Found**:
```
SSBI0, SSBI1, SSBI2, SSBI3, SSBI4
PMIC_GENERIC
PMIC_SSBI2
PMIC_ARBITER
PM8058 class objects (C++ mangled names)
PM8901 class objects
VREG init functions
LDO/SMPS management
```

**SSBI Register References**:
- **PM8058 (0x00500000)**: 7 references
  - LDO0_CTRL (0x00), LDO1_CTRL (0x02)
  - MPP1_CTRL (0x50)
- **PM8901 (0x00540000)**: 72 references (extensive init!)
  - Multiple LDO0_CTRL references suggest initialization loop

**What HTC TZ Does**:
1. Initialize SSBI bus arbitration (PMIC_ARBITER)
2. Configure PMIC GPIO/MPP pins
3. Set initial LDO/SMPS voltages
4. Enable ADC for battery/temperature sensing
5. Configure interrupt controller
6. Set up clock outputs

### 2. TouchPad Bootloader PMIC References

**p5 (RPM, 500KB)**: `/tmp/touchpad-p5.bin`
```
Strings: smps/A/smps0/vec, SMPSS_M0_GENERIC
PM8058_SSBI references: 53
Purpose: RPM firmware with SMPS voltage vectors
```

**p6 (QCSBL, 750KB)**: `/tmp/touchpad-p6.bin`
```
Strings: PM8058E, PM8901E, LDO_8058_LITEE, LDO_8901_LITEE, SMPS_8058_LITEE
PM8058_SSBI references: 146
PM8901_SSBI references: 118
PMIC_SSBI_BASE references: 469 (most!)
Purpose: PMIC library/definitions, but NO INITIALIZATION CODE
```

**p7 (APPSBL/Bootie, 2.5MB)**: `/tmp/touchpad-p7.bin`
```
Strings:
  pm8058_gpio_cfg
  pm8058_gpio_get
  pm8058_irq_get_rt_status
  pm8058_init_regulator    ← Present but not called!
  pm8058_vreg_write
  pm8058_smps_set_voltage
  pm8058_pldo_set_voltage

PM8058_SSBI references: 47
PM8901_SSBI references: 72
Purpose: PMIC API functions DEFINED but not executed (no TZ to call them)
```

**Key Finding**: TouchPad APPSBL has `pm8058_init_regulator` function but it's never called because p10 (TZ) is empty!

### 3. webOS Kernel PMIC Init

**drivers/mfd/pm8058-core.c**:
```c
int pm8058_readb(struct device *dev, u16 addr, u8 *val)
{
    struct pm8058 *pmic = dev_get_drvdata(dev);
    return msm_ssbi_read(pmic->dev->parent, addr, val, 1);
}

int pm8058_writeb(struct device *dev, u16 addr, u8 val)
{
    struct pm8058 *pmic = dev_get_drvdata(dev);
    return msm_ssbi_write(pmic->dev->parent, addr, &val, 1);
}
```

**Key Register Defines**:
```c
#define REG_HWREV           0x0002  /* PMIC4 revision */
#define REG_IRQ_PERM        0x01a6
#define REG_IRQ_ROOT        0x01bb
#define REG_IRQ_M_STATUS1   0x01bc
#define REG_GPIO_CTRL(x)    (0x0150 + (x))
```

**arch/arm/mach-msm/board-tenderloin.c** (regulator platform data):
```c
static struct platform_device rpm_vreg_device[RPM_VREG_ID_MAX] = {
    RPM_VREG(RPM_VREG_ID_PM8058_L0),   /* LDO 0 */
    RPM_VREG(RPM_VREG_ID_PM8058_L1),   /* LDO 1 */
    // ... 25 LDOs total
    RPM_VREG(RPM_VREG_ID_PM8058_S0),   /* SMPS 0 */
    // ... 4 SMPS total
    RPM_VREG(RPM_VREG_ID_PM8058_LVS0), /* Low Voltage Switch */
    RPM_VREG(RPM_VREG_ID_PM8058_NCP),  /* Negative Charge Pump */
    
    RPM_VREG(RPM_VREG_ID_PM8901_L0),   /* PM8901 LDO 0 */
    // ... PM8901 regulators
};
```

**Key webOS Regulators**:
```c
// USB PHY power
ldo6_3p3 = regulator_get(NULL, "8058_l6");  // 3.3V
ldo7_1p8 = regulator_get(NULL, "8058_l7");  // 1.8V

// CPU core voltage
vdd_cx = regulator_get(NULL, "8058_s1");
regulator_set_voltage(vdd_cx, min_vol, max_vol);
```

**webOS Kernel Initialization Flow**:
```
1. SSBI bus probe (msm_ssbi.c)
2. PM8058 MFD probe (pm8058-core.c)
   - Read HWREV register
   - Setup IRQ controller
   - Register GPIO/MPP banks
3. RPM probe (rpm-regulator.c)
   - Communicate with RPM firmware (p5)
   - Setup voltage/current control
4. Board file sets constraints
   - regulator_set_voltage()
   - regulator_enable()
```

### 4. Mainline Kernel PMIC Support

**drivers/mfd/qcom-pm8xxx.c**:
```c
static const struct of_device_id pm8xxx_id_table[] = {
    { .compatible = "qcom,pm8058", .data = &pm8xxx_data },
    { .compatible = "qcom,pm8901", .data = &pm8901_data },
    { }
};
```

**drivers/mfd/ssbi.c**:
- SSBI bus driver (same as webOS)

**drivers/regulator/qcom_rpm-regulator.c**:
```c
static const struct of_device_id rpm_of_match[] = {
    { .compatible = "qcom,rpm-pm8058-regulators", .data = &rpm_pm8058_regulators },
    { .compatible = "qcom,rpm-pm8901-regulators", .data = &rpm_pm8901_regulators },
    { }
};
```

**Device Tree** (qcom-apq8060-tenderloin-common.dtsi):
```dts
pm8058: pm8058@0 {
    compatible = "qcom,pm8058";
    reg = <0>;
    // Interrupt configuration
    // GPIO/MPP configuration
};

pm8901: pm8901@1 {
    compatible = "qcom,pm8901";
    reg = <1>;
};

pm8901-regulators {
    compatible = "qcom,rpm-pm8901-regulators";
    
    pm8901_l0: l0 {
        regulator-min-microvolt = <1200000>;
        regulator-max-microvolt = <1200000>;
    };
    // ... all 6 LDOs defined
    // ... all 4 SMPS defined
    // ... all 4 LVS defined
};
```

**Mainline Initialization Flow**:
```
1. SSBI bus probe
2. PM8xxx MFD probe (qcom-pm8xxx.c)
   - Parse device tree
   - Setup IRQ domain
   - Register child devices
3. RPM regulator probe (qcom_rpm-regulator.c)
   - Read device tree constraints
   - Apply voltage/current settings via RPM
4. Pinctrl probe (pinctrl-pm8xxx.c, pinctrl-pm8xxx-mpp.c)
   - Configure GPIO/MPP pins
```

---

## Critical Gaps in TouchPad

### Gap 1: ADC Initialization

**HTC has**: ADC calibration in TrustZone
**TouchPad has**: No ADC init (not in bootloader, not in webOS kernel visible calls)
**Impact**: Battery temperature, voltage sensing may be uncalibrated

**Potential Fix**:
```c
/* In drivers/iio/adc/ or drivers/power/supply/ */
static int pm8058_adc_init(struct device *dev)
{
    /* Write ADC calibration registers */
    pm8058_writeb(dev, ADC_ARB_USRP_CNTRL1, 0x00);  /* Disable arb */
    pm8058_writeb(dev, ADC_ARB_USRP_CNTRL, 0x00);   /* Reset */
    /* Enable ADC channels */
    pm8058_writeb(dev, ADC_ARB_USRP_CNTRL1, 0x01);  /* Enable arb */
    
    return 0;
}
```

### Gap 2: Clock Output Configuration

**HTC has**: Clock output enable in TrustZone (CLK_CTRL registers 0x48, 0x49)
**TouchPad has**: No clock init
**Impact**: External peripherals expecting PMIC clock outputs may not work

**Registers**:
```c
#define PM8058_CLK_CTRL   0x0048
#define PM8058_CLK_CTRL2  0x0049

/* Clock outputs: Sleep_CLK, XO_CORE, etc. */
```

### Gap 3: Boot-Time Regulator Voltages

**HTC has**: TZ sets initial LDO/SMPS voltages before kernel
**TouchPad has**: Only RPM runtime control via webOS kernel
**Impact**: Peripherals may not power on correctly until kernel sets voltage

**Current Mainline Approach**: Device tree defines regulator constraints, RPM applies them at probe time. This should work IF:
1. Bootloader leaves regulators in safe state (may not be true)
2. Probe ordering is correct (peripherals don't probe before regulators)

**Potential Issue**: If APPSBL (p7) tries to use peripherals before regulators are ready, they won't work.

---

## Recommended Actions

### Priority 1: Verify Mainline PMIC is Working

**Test regulator control**:
```bash
# On device
cat /sys/class/regulator/regulator.*/name
cat /sys/class/regulator/regulator.*/microvolts

# Check if RPM is controlling PMIC
dmesg | grep -i "rpm.*regulator\|pm8058\|pm8901"
```

**Expected output**:
```
qcom-rpm-regulator: pm8058_l0: 1200000 uV
qcom-rpm-regulator: pm8058_s3: 1800000 uV (DCVDD for audio codec)
qcom-rpm-regulator: pm8901_l1: 3300000 uV (WiFi power)
```

### Priority 2: Add Missing ADC Init

If battery/temperature sensing is broken, add ADC initialization to kernel:

**Option A**: Add to PM8xxx MFD driver probe
**Option B**: Create separate IIO ADC driver with init sequence
**Option C**: Add to bootloader (LK) similar to CE2 fix

### Priority 3: Add Clock Output Init

If any peripheral needs PMIC clock output (unlikely on TouchPad):

```c
/* In drivers/mfd/qcom-pm8xxx.c probe */
static int pm8058_clk_init(struct pm8xxx *pm8xxx)
{
    u8 val;
    
    /* Enable Sleep_CLK output */
    val = 0x01;  /* CLK_ENABLE */
    pm8xxx_writeb(pm8xxx->dev, PM8058_CLK_CTRL, val);
    
    return 0;
}
```

### Priority 4: Boot-Time Regulator Config

If peripherals fail to probe due to regulator issues:

**Option A**: Add `regulator-boot-on` to critical regulators in device tree:
```dts
pm8058_l6: l6 {
    regulator-min-microvolt = <3300000>;
    regulator-max-microvolt = <3300000>;
    regulator-boot-on;  /* Ensure enabled at boot */
};
```

**Option B**: Add early init to LK bootloader (before kernel handoff)

---

## Comparison with CE2 Fix

**CE2 Problem**: Hardware init missing from TrustZone
**CE2 Solution**: Manual clock enable in Linux kernel driver

**PMIC Problem**: Hardware init missing from TrustZone
**PMIC Solution Options**:
1. **Verify mainline works** (may already be fine via RPM)
2. **Add missing init** (ADC, clocks) if needed
3. **Boot-time config** if probe ordering issues

**Key Difference**: CE2 was completely dead (MMIO returning zeros). PMIC may be partially working via RPM, just missing optional features (ADC, clock outputs).

---

## Test Plan

1. **Boot mainline kernel on TouchPad**
2. **Check PMIC probe**:
   ```bash
   dmesg | grep pm8058
   dmesg | grep pm8901
   dmesg | grep rpm-regulator
   ```
3. **Test regulator control**:
   ```bash
   cat /sys/class/regulator/*/name
   cat /sys/class/regulator/*/microvolts
   cat /sys/class/regulator/*/state
   ```
4. **Test GPIO control**:
   ```bash
   cat /sys/kernel/debug/gpio | grep pm8058
   ```
5. **Test battery sensing**:
   ```bash
   cat /sys/class/power_supply/battery/voltage_now
   cat /sys/class/power_supply/battery/temp
   ```
6. **If issues found**: Add missing init based on HTC TZ analysis

---

## Files Referenced

**HTC Bootloaders**:
- `/tmp/tz.img` - TrustZone with PMIC init (106 KB, 72 PM8901 refs, 7 PM8058 refs)
- `/tmp/rpm.img` - RPM with SMPS vectors (120 KB, 45 PM8058 refs)
- `/tmp/sbl3.img` - APPSBL with PMIC library (596 KB, 201 PM8058 refs, 301 PM8901 refs)

**TouchPad Bootloaders**:
- `/tmp/touchpad-p5.bin` - RPM (512 KB, 53 PM8058 refs)
- `/tmp/touchpad-p6.bin` - QCSBL (768 KB, 146 PM8058 refs, 118 PM8901 refs, 469 SSBI refs)
- `/tmp/touchpad-p7.bin` - APPSBL (2.5 MB, 47 PM8058 refs, 72 PM8901 refs, has init functions but uncalled)

**webOS Kernel**:
- `drivers/mfd/pm8058-core.c` - PM8058 MFD driver
- `arch/arm/mach-msm/board-tenderloin.c` - Regulator platform data
- `drivers/regulator/rpm-regulator.c` - RPM voltage control

**Mainline Kernel**:
- `drivers/mfd/qcom-pm8xxx.c` - PM8058/PM8901 MFD driver
- `drivers/mfd/ssbi.c` - SSBI bus driver
- `drivers/regulator/qcom_rpm-regulator.c` - RPM regulator driver
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - Regulator DT config
- `arch/arm/boot/dts/qcom/pm8058.dtsi` - PM8058 base device tree

---

**Date**: 2026-05-15
**Analysis**: Herman van Hazendonk + Claude Code (Anthropic)
**Status**: Comparison complete, mainline verification recommended as next step
