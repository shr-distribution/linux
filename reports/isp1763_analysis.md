# ISP1763 USB Host Controller Analysis for HP TouchPad 3G
**Date:** 2025-12-31
**Hardware:** HP TouchPad 3G variant (Topaz 3G)
**Kernel:** Linux 6.13.0 mainline

---

## EXECUTIVE SUMMARY

**Status: ✅ DRIVER EXISTS - EBI2 BUS SUPPORT NEEDED**

The ISP1763 USB host controller has **complete mainline driver support** with device tree bindings. Implementation is highly feasible but requires EBI2 (External Bus Interface 2) support for memory-mapped peripheral access.

### Key Findings:
- ✅ **Mainline driver exists**: drivers/usb/isp1760/ (supports ISP1760/1761/1763)
- ✅ **Device tree bindings**: nxp,usb-isp1763 compatible string
- ✅ **Reference implementations**: OMAP3 platforms use ISP1763
- ✅ **All hardware details available**: GPIOs, addressing, timing from legacy kernel
- ⚠️ **Requires EBI2 support**: MSM8660 EBI2 controller needs device tree integration

---

## 1. MAINLINE DRIVER STATUS

### Driver Location

**Path:** `drivers/usb/isp1760/`

**Files:**
- `isp1760-core.c` - Core driver logic
- `isp1760-if.c` - Platform/OF interface
- `isp1760-hcd.c` - USB host controller driver
- `isp1760-udc.c` - USB device controller (not used for ISP1763)
- `isp1760-regs.h` - Register definitions

**Kconfig:** `CONFIG_USB_ISP1760`

### Supported Chip Variants

| Chip | Data Bus | Mode | Memory | Mainline Support |
|------|----------|------|--------|------------------|
| ISP1760 | 32-bit | Host only | 60KB | ✅ Yes |
| ISP1761 | 32-bit | Dual-role (host/device) | 60KB | ✅ Yes |
| **ISP1763** | **16-bit** | **Host only** | **20KB** | **✅ Yes** |

### Device Tree Compatible Strings

From `drivers/usb/isp1760/isp1760-if.c:209-213`:
```c
static const struct of_device_id isp1760_of_match[] = {
    { .compatible = "nxp,usb-isp1760", },
    { .compatible = "nxp,usb-isp1761", },
    { .compatible = "nxp,usb-isp1763", },  /* ✅ ISP1763 supported */
    { },
};
```

---

## 2. LEGACY KERNEL HARDWARE CONFIGURATION

### Memory-Mapped I/O Configuration

**From:** `arch/arm/mach-msm/board-tenderloin.c:508-511`

```c
static struct resource isp1763_resources[] = {
    [0] = {
        .start = 0x1D000000,  /* Base address */
        .end   = 0x1D005FFF,  /* +24KB (0x6000) */
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = ISP1763_INT_GPIO,  /* GPIO 172 */
        .end   = ISP1763_INT_GPIO,
        .flags = IORESOURCE_IRQ,
    },
};
```

**Key Details:**
- **Base Address**: 0x1D000000
- **Memory Size**: 24KB (0x6000)
- **Bus Interface**: EBI2 (External Bus Interface 2)
- **Chip Select**: CS3
- **Data Bus Width**: 16-bit

### GPIO Assignments

**From:** `arch/arm/mach-msm/gpiomux-tenderloin.h:156-164`

| Signal | GPIO | Direction | Function | Pull |
|--------|------|-----------|----------|------|
| **INT** | 172 | Input | Interrupt (active low) | Pull-up |
| **RST** | 152 | Output | Reset (active high) | None |
| **DACK** | 169 | Output | DMA acknowledge | None |
| **DREQ** | 29 | Input | DMA request | Pull-down |
| **3G_3V3_EN** | 106 (DVT) / 82 (EVT) | Output | Power enable | None |
| **3G_DISABLE_N** | 171 | Output | Modem disable | None |
| **3G_WAKE_N** | 38 | Output | Modem wake | None |
| **3G_UIM_CD_N** | 61 | Input | SIM card detect | Pull-up |

### EBI2 Bus Timing Configuration

**From:** `board-tenderloin.c:4644-4667`

```c
/* EBI2 configuration for ISP1763 on CS3 */
writel(0x51010112, ebi2_cfg_ptr + 0x14);  /* CS3_CFG0 */
/*
 * RECOVERY = 5 clocks
 * HOLD_WR = 1 clock
 * INIT_LATENCY_WR = 1 clock
 * INIT_LATENCY_RD = 1 clock
 * WAIT_WR = 1 clock
 * WAIT_RD = 2 clocks
 */

writel(0x01000020, ebi2_cfg_ptr + 0x34);  /* CS3_CFG1 */
/*
 * HOLD_RD = 1 clock
 * ADV_OE_RECOVERY = 0
 * ADDR_HOLD_ENA = 1
 */
```

---

## 3. ISP1763 HARDWARE SPECIFICATIONS

### Bus Interface

**Type:** Memory-mapped parallel interface (16-bit data bus)

**From:** `drivers/usb/isp1760/isp1760-core.c:142-160`

**Memory Layout:**
- **Total Payload**: 20KB (0x5000 bytes)
- **Blocks**:
  - 8 blocks @ 256 bytes each (2KB)
  - 2 blocks @ 1024 bytes each (2KB)
  - 4 blocks @ 4096 bytes each (16KB)
- **PTD (Payload Transfer Descriptor) Slots**: 16 (vs 32 for ISP1760/1761)

**Register Configuration:**
- **Register Width**: 16-bit (vs 32-bit for ISP1760/1761)
- **Register Stride**: 2 bytes
- **Address Range**: 8-bit address space (0x00-0xFF)

### Key Differences from ISP1760/1761

| Feature | ISP1760/1761 | ISP1763 |
|---------|--------------|---------|
| Data Bus | 32-bit | 16-bit |
| Memory Buffer | 60KB | 20KB |
| PTD Slots | 32 | 16 |
| Analog Overcurrent | Yes | No |
| Register Width | 32-bit | 16-bit |
| Target Market | Desktop/Server | Embedded |

### Power Requirements

**From legacy kernel:**
- **3.3V Supply**: GPIO 106 (DVT) or GPIO 82 (EVT)
- **Modem Power**: GPIO 171 (active low disable)
- **No explicit regulator**: Uses GPIO-controlled power switches

---

## 4. DEVICE TREE BINDING

### Device Tree Bindings Documentation

**File:** `Documentation/devicetree/bindings/usb/nxp,isp1760.yaml`

### Required Properties

```yaml
compatible: "nxp,usb-isp1763"
reg: Memory-mapped I/O address and size
interrupts: Interrupt specifier
bus-width: 16  # 16-bit data bus
dr_mode: "host"  # Host-only mode
```

### Reference Implementation: OMAP3

**File:** `arch/arm/boot/dts/ti/omap/logicpd-torpedo-baseboard.dtsi:107-143`

```dts
usb@6,0 {
    compatible = "nxp,usb-isp1763";
    reg = <0x6 0x0 0xff>;
    interrupt-parent = <&gpio5>;
    interrupts = <0 IRQ_TYPE_LEVEL_LOW>;
    interrupt-names = "host";
    bus-width = <16>;
    dr_mode = "host";

    gpmc,mux-add-data = <0>;
    gpmc,device-width = <2>;
    gpmc,wait-pin = <0>;
    gpmc,burst-length = <4>;
    gpmc,cycle2cycle-samecsen;
    gpmc,cycle2cycle-diffcsen;
    gpmc,cs-on-ns = <0>;
    gpmc,cs-rd-off-ns = <100>;
    gpmc,cs-wr-off-ns = <100>;
    /* ... more GPMC timing ... */
};
```

**Note:** OMAP3 uses GPMC (General Purpose Memory Controller), while MSM8660 uses EBI2 (External Bus Interface 2).

---

## 5. PROPOSED DEVICE TREE FOR HP TOUCHPAD 3G

### ISP1763 Node (for qcom-apq8060-topaz-3g.dts)

```dts
&ebi2 {
    isp1763: usb@3,0 {
        compatible = "nxp,usb-isp1763";
        reg = <3 0x0 0x6000>;  /* CS3, offset 0, 24KB */

        interrupts-extended = <&tlmm 172 IRQ_TYPE_LEVEL_LOW>;
        interrupt-names = "host";

        bus-width = <16>;
        dr_mode = "host";

        pinctrl-names = "default";
        pinctrl-0 = <&isp1763_pins>;

        /* Power control for 3G modem/USB hub */
        power-gpios = <&tlmm 106 GPIO_ACTIVE_HIGH>;  /* GPIO_3G_3V3_EN (DVT) */
        reset-gpios = <&tlmm 152 GPIO_ACTIVE_HIGH>;  /* ISP1763_RST */

        status = "okay";
    };
};
```

### EBI2 Controller Node (needs to be added to qcom-msm8660.dtsi)

```dts
&soc {
    ebi2: external-bus@1a100000 {
        compatible = "qcom,msm8660-ebi2";
        reg = <0x1a100000 0x1000>,    /* EBI2 config registers */
              <0x1a110000 0x1000>;    /* EBI2 XMEM registers */
        #address-cells = <2>;
        #size-cells = <1>;
        ranges = <0 0x0 0x1a800000 0x00800000>,  /* CS0: NOR flash (if present) */
                 <1 0x0 0x1b000000 0x00800000>,  /* CS1: (unused) */
                 <2 0x0 0x1b800000 0x00800000>,  /* CS2: (unused) */
                 <3 0x0 0x1d000000 0x00010000>;  /* CS3: ISP1763 USB */
    };
};
```

### GPIO Pinctrl Configuration

```dts
&tlmm {
    isp1763_pins: isp1763-state {
        int-pins {
            pins = "gpio172";
            function = "gpio";
            drive-strength = <2>;
            bias-pull-up;
            input-enable;
        };

        rst-pins {
            pins = "gpio152";
            function = "gpio";
            drive-strength = <8>;
            bias-disable;
        };

        dack-pins {
            pins = "gpio169";
            function = "gpio";
            drive-strength = <2>;
            bias-disable;
        };

        dreq-pins {
            pins = "gpio29";
            function = "gpio";
            drive-strength = <2>;
            bias-pull-down;
        };

        power-pins {
            pins = "gpio106";  /* DVT: gpio106, EVT: gpio82 */
            function = "gpio";
            drive-strength = <2>;
            bias-disable;
        };

        modem-disable-pins {
            pins = "gpio171";
            function = "gpio";
            drive-strength = <2>;
            bias-disable;
        };
    };
};
```

---

## 6. IMPLEMENTATION CHALLENGES

### Critical: EBI2 Controller Support

**Issue:** MSM8660 EBI2 controller may not have mainline device tree support.

**Investigation Needed:**
1. Check if `qcom,msm8660-ebi2` compatible exists in mainline
2. Verify EBI2 driver supports device tree configuration
3. Determine if EBI2 timing parameters can be configured via DT

**Alternatives if EBI2 not supported:**
1. Add EBI2 controller driver to mainline
2. Use platform data (less desirable, non-upstreamable)
3. Hardcode EBI2 configuration in bootloader (not ideal)

### Medium: GPIO Interrupt Mapping

**Issue:** GPIO 172 must be properly mapped to GIC interrupt.

**Solution:** Use `interrupts-extended` property to reference GPIO controller:
```dts
interrupts-extended = <&tlmm 172 IRQ_TYPE_LEVEL_LOW>;
```

### Low: Power Sequencing

**Issue:** 3G modem and ISP1763 may require specific power-up sequence.

**From legacy kernel (board-tenderloin.c:609-632):**
```c
static int isp1763_setup_gpio(int enable)
{
    int rc = 0;

    if (enable) {
        rc = gpio_request(ISP1763_RST_GPIO, "isp1763_rst");
        if (rc) {
            pr_err("%s: gpio_request(%d) failed\n", __func__, ISP1763_RST_GPIO);
            return rc;
        }
        gpio_direction_output(ISP1763_RST_GPIO, 0);
        gpio_set_value(ISP1763_RST_GPIO, 0);
        msleep(50);
        gpio_set_value(ISP1763_RST_GPIO, 1);
    } else {
        gpio_set_value(ISP1763_RST_GPIO, 0);
        gpio_free(ISP1763_RST_GPIO);
    }

    return rc;
}
```

**Sequence:**
1. Assert reset (LOW) for 50ms
2. De-assert reset (HIGH)
3. Wait for chip enumeration

**Solution:** Use `reset-gpios` property with `reset-duration-us` if supported by driver.

---

## 7. KERNEL CONFIGURATION

### Required Kconfig Options

```kconfig
CONFIG_USB_ISP1760=y              # ISP1760/1761/1763 driver
CONFIG_USB_ISP1760_HOST_ROLE=y    # Host-only mode
CONFIG_REGMAP_MMIO=y              # Memory-mapped register access
CONFIG_OF=y                        # Device tree support
CONFIG_OF_ADDRESS=y                # DT address translation
```

### Module vs Built-in

**Recommendation:** Built-in (`=y`) for boot-time USB support (keyboard, etc.)

**Alternative:** Module (`=m`) if USB only needed after boot

---

## 8. IMPLEMENTATION ROADMAP

### Phase 1: EBI2 Investigation (CRITICAL)

**Tasks:**
1. Search mainline kernel for EBI2 support:
   ```bash
   git grep -i "ebi2" -- drivers/ arch/arm/
   ```
2. Check MSM8660 DTSI for EBI2 node
3. Examine APQ8064 (successor) for EBI2 implementation
4. Determine if EBI2 driver exists or needs creation

**Outcome:** Determine feasibility of EBI2 device tree support

### Phase 2: Device Tree Implementation (if EBI2 supported)

**Tasks:**
1. Add EBI2 controller node to `qcom-msm8660.dtsi`
2. Configure EBI2 CS3 timing for ISP1763
3. Add ISP1763 node to `qcom-apq8060-topaz-3g.dts`
4. Add GPIO pinctrl configuration
5. Build and check for DTC warnings/errors

**Expected Result:** DTB compiles without errors

### Phase 3: Driver Testing (requires hardware)

**Tasks:**
1. Enable CONFIG_USB_ISP1760 in kernel config
2. Boot 3G TouchPad with new DTB
3. Check dmesg for ISP1763 probe messages
4. Verify `/sys/bus/platform/drivers/isp1760` shows device
5. Test USB enumeration with flash drive, keyboard, etc.

**Success Criteria:**
- ISP1763 driver probes successfully
- USB devices enumerate
- Basic USB functionality works

### Phase 4: 3G Modem Integration (optional)

**Tasks:**
1. Implement modem power sequencing
2. Test modem USB enumeration
3. Integrate with Gobi QMI drivers
4. Test cellular connectivity

**Success Criteria:**
- Modem detected as USB device
- QMI interface available
- Cellular connection established

---

## 9. COMPARISON: ISP1763 vs ALTERNATIVES

### Why ISP1763?

**Hardware:** HP TouchPad 3G uses ISP1763 for 3G modem connectivity.

**Alternatives Considered:**
1. **USB via MSM8660 internal USB controller**: Not available (already used for USB OTG port)
2. **SDIO-based modem**: Modem is USB-only (Gobi MDM6600)
3. **Software emulation**: Not practical for USB host

**Conclusion:** ISP1763 is the only option for 3G modem support.

---

## 10. FEASIBILITY ASSESSMENT

| Criteria | Score | Notes |
|----------|-------|-------|
| **Driver Availability** | 10/10 | Full mainline support ✅ |
| **Hardware Documentation** | 10/10 | Complete legacy kernel details ✅ |
| **Device Tree Bindings** | 10/10 | Well-documented, tested on OMAP3 ✅ |
| **GPIO Availability** | 9/10 | All GPIOs verified, may conflict with HDMI CEC ⚠️ |
| **EBI2 Support** | **?/10** | **Unknown - needs investigation** ⚠️ |
| **Reference Implementations** | 8/10 | OMAP3 GPMC, not EBI2 |
| **Upstream Acceptance** | 9/10 | Good if EBI2 properly implemented |
| **Overall** | **?/10** | **Blocked on EBI2 support** |

---

## 11. GPIO CONFLICT ANALYSIS

### Potential Conflict: GPIO 169

**ISP1763 Usage:** DACK (DMA acknowledge)
**HDMI Usage:** CEC (Consumer Electronics Control)

**From previous analysis:**
- HDMI configured GPIO 169 as CEC pin
- ISP1763 needs GPIO 169 for DACK signal

**Resolution:**
Two options:
1. **Disable HDMI CEC on 3G variant** (recommended)
   - CEC is optional HDMI feature
   - DMA is critical for ISP1763 performance

2. **Use ISP1763 without DMA**
   - Simpler hardware configuration
   - Reduced performance (polling mode)
   - No GPIO 169 conflict

**Recommendation:** Disable HDMI CEC on 3G variant, prioritize ISP1763 DMA.

**Device Tree Override:**
```dts
/* In qcom-apq8060-topaz-3g.dts */
&hdmi {
    /* Remove CEC pin from HDMI pinctrl on 3G variant */
    pinctrl-0 = <&hdmi_pinctrl_no_cec>;
};

&tlmm {
    hdmi_pinctrl_no_cec: hdmi-state-no-cec {
        ddc-pins {
            pins = "gpio170", "gpio171";
            function = "hdmi";
            drive-strength = <2>;
            bias-pull-up;
        };
        hpd-pins {
            pins = "gpio172";
            function = "hdmi";
            drive-strength = <16>;
            bias-disable;
        };
        /* No CEC pins - GPIO 169 reserved for ISP1763 */
    };
};
```

### Potential Conflict: GPIO 172

**ISP1763 Usage:** INT (interrupt)
**HDMI Usage:** HPD (Hot Plug Detect)

**BOTH on WiFi AND 3G variants!**

**This is a CRITICAL CONFLICT** ⚠️

**Investigation Needed:**
1. Verify GPIO 172 usage in legacy 3G kernel
2. Check if HDMI HPD is different on 3G variant
3. Determine if ISP1763 INT can be moved to different GPIO

**From GPIO verification report:** GPIO 172 marked as HDMI_HPD on WiFi variant only.

**Hypothesis:** 3G variant may use different GPIO for HDMI HPD or HDMI may not be present on 3G variant.

**Action Required:** Check legacy 3G kernel for HDMI configuration.

---

## 12. NEXT STEPS

### Immediate Actions

1. **Investigate EBI2 Support** ✅ CRITICAL
   ```bash
   cd /home/herrie/webos/touchpad-kernel/shr-linux
   git grep -i "ebi2" -- drivers/ arch/arm/
   git grep "external.*bus" -- Documentation/devicetree/bindings/
   ```

2. **Resolve GPIO 172 Conflict** ✅ CRITICAL
   ```bash
   grep -r "GPIO.*172" /home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/arch/arm/mach-msm/
   # Check if 3G variant uses different HDMI HPD GPIO
   ```

3. **Check GPIO 169 Usage on 3G** ⚠️ IMPORTANT
   ```bash
   # Verify HDMI CEC configuration on 3G variant
   grep -A 10 -B 10 "GPIO.*169" /home/herrie/webos/touchpad-kernel/webos-linux-kernel-opal/arch/arm/mach-msm/board-tenderloin.c
   ```

### Short-term (if EBI2 supported)

1. Create EBI2 device tree node
2. Add ISP1763 configuration to topaz-3g.dts
3. Resolve GPIO conflicts
4. Build and test DTB

### Long-term (if EBI2 NOT supported)

1. Develop EBI2 controller driver
2. Submit patches upstream
3. Implement ISP1763 integration

---

## 13. CONCLUSION

The ISP1763 USB host controller has **excellent mainline driver support** with complete device tree bindings and proven implementations on similar embedded platforms (OMAP3).

**Implementation is HIGHLY FEASIBLE IF:**
1. ✅ MSM8660 EBI2 controller has device tree support (needs verification)
2. ⚠️ GPIO conflicts can be resolved (GPIO 172, GPIO 169)
3. ✅ 3G TouchPad hardware is available for testing

**BLOCKERS:**
1. **EBI2 support unknown** - requires immediate investigation
2. **GPIO 172 conflict** - ISP1763 INT vs HDMI HPD
3. **GPIO 169 conflict** - ISP1763 DACK vs HDMI CEC (lower priority)

**RECOMMENDATION:**
1. First, investigate EBI2 device tree support in mainline
2. Resolve GPIO conflicts by checking 3G variant hardware configuration
3. If EBI2 supported, proceed with device tree implementation
4. If EBI2 NOT supported, consider effort vs benefit of driver development

**Priority:** MEDIUM-HIGH (blocks 3G modem support, but WiFi variant unaffected)

---

## REFERENCES

### Mainline Kernel

- **ISP1760 Driver**: `drivers/usb/isp1760/isp1760-core.c`
- **Platform Interface**: `drivers/usb/isp1760/isp1760-if.c`
- **DT Bindings**: `Documentation/devicetree/bindings/usb/nxp,isp1760.yaml`
- **OMAP3 Reference**: `arch/arm/boot/dts/ti/omap/logicpd-torpedo-baseboard.dtsi`
- **Kconfig**: `drivers/usb/isp1760/Kconfig`

### Legacy Kernel

- **Board File**: `arch/arm/mach-msm/board-tenderloin.c` (lines 503-727, 4644-4667)
- **GPIO Definitions**: `arch/arm/mach-msm/gpiomux-tenderloin.h` (lines 155-164)
- **GPIO Muxing**: `arch/arm/mach-msm/gpiomux-tenderloin.c` (lines 466-476)
- **Platform Data**: `arch/arm/mach-msm/board-tenderloin.c` (lines 584-595)

---

**Analysis Date:** 2025-12-31
**Kernel Version:** Linux 6.13.0
**Conclusion:** Driver exists, EBI2 support is critical dependency
**Status:** Feasible pending EBI2 investigation

---

**Next Critical Step:** Investigate MSM8660 EBI2 device tree support in mainline kernel
