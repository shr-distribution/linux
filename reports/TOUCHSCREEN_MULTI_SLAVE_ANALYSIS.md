# HP TouchPad CY8CTMA395 Multi-Slave Touchscreen Analysis

## Executive Summary

The HP TouchPad uses a unique multi-slave touchscreen architecture with **1 master controller (CY8CTMA395) and 5 slave controllers (CY8CTMA375)**. The original webOS driver was proprietary and never released as open source.

**CRITICAL DISCOVERY:** The touchscreen communicates with the host processor over **UART** (not I²C) at `/dev/ttyMSM2` via GSBI10. PostmarketOS confirmed this in their working implementation using a userspace driver (`ts-srv`) adapted from Android. The I²C interface is only used for firmware programming and configuration, not for touch data transmission.

The mainline Linux cyttsp I²C driver approach is therefore incorrect for runtime touch data and is limited to 4 simultaneous touches.

## Hardware Architecture

### Physical Configuration

Based on HP TouchPad Technical Specifications (pages 18-19):

- **Master Controller:** 1× Cypress CY8CTMA395
  - I²C Address: 0x67
  - Connected to: APQ8060 GSBI10 I²C bus
  - Role: Aggregates touch data from all 5 slaves and reports to host processor

- **Slave Controllers:** 5× Cypress CY8CTMA375
  - I²C Addresses: Unknown (likely 0x20-0x28 range based on firmware patterns)
  - Connected to: CY8CTMA395 master via I²C
  - Role: Each covers a portion of the 9.7" touchscreen panel (1024×768 resolution)

### Touchscreen Coverage Area

The 9.7" display (1024×768 pixels) is divided among the 6 controllers:
- **Likely configuration:** Each slave covers approximately 1/5 of the screen
- **Touch capacity:** Each CY8CTMA375 can handle 2-4 touches
- **Total theoretical capacity:** 10-20 simultaneous touches (5 slaves × 2-4 each)

## Communication Architecture

### UART vs I²C: The Critical Discovery

**PostmarketOS Implementation Analysis** (Sources: [PostmarketOS Wiki](https://wiki.postmarketos.org/wiki/HP_TouchPad_(hp-tenderloin)), [GitLab Commit 6090c88c](https://gitlab.postmarketos.org/postmarketOS/pmaports/-/commit/6090c88cc49f9be652c94ea5e9a73e63306dfee8))

The HP TouchPad touchscreen uses **UART for touch data transmission**, NOT I²C!

#### Evidence from Device Tree

File: `/arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`

```dts
aliases {
    serial0 = &gsbi12_serial;  // /dev/ttyMSM0
    serial1 = &gsbi6_serial;   // /dev/ttyMSM1
#if !USE_I2C_TOUCHSCREEN_DRIVER
    serial2 = &gsbi10_serial;  // /dev/ttyMSM2 <-- TOUCHSCREEN UART!
#endif
};

&gsbi10 {
    status = "okay";

#if USE_I2C_TOUCHSCREEN_DRIVER
    qcom,mode = <GSBI_PROT_I2C>;        // I2C mode (current - WRONG!)
#else
    qcom,mode = <GSBI_PROT_I2C_UART>;   // I2C+UART mode (correct!)
#endif
};
```

**Key Findings:**

1. **GSBI10 Dual Mode:**
   - Can operate as pure I²C (`GSBI_PROT_I2C`)
   - Can operate as I²C+UART simultaneously (`GSBI_PROT_I2C_UART`)

2. **Original webOS Configuration:**
   - Used `GSBI_PROT_I2C_UART` mode
   - Touch data transmitted over UART at `/dev/ttyMSM2`
   - I²C used only for configuration and firmware updates

3. **Current Mainline Attempt:**
   - Set to `GSBI_PROT_I2C` mode (incorrect)
   - Tries to use cyttsp I²C driver
   - **This is why touchscreen doesn't work properly!**

#### PostmarketOS Working Implementation

PostmarketOS successfully implemented touchscreen support using:

- **Userspace driver**: `ts-srv` package (adapted from Android/Evervolv)
- **Communication**: UART via `/dev/ttyMSM2` on GSBI10
- **Protocol**: Custom binary protocol (proprietary, reverse-engineered)
- **Approach**: Reads touch data packets from UART, processes in userspace, injects into Linux input subsystem

**From PostmarketOS documentation:**
> "The touchscreen controller appears to be implemented using custom firmware on a Cypress CY8CTMA395 microcontroller, and primarily communicates with the processor using UART."

**Device Detection:**
```bash
# Udev rule from PostmarketOS
SUBSYSTEM=="input", ATTRS{name}=="HPTouchpad",
ENV{ID_INPUT}="1", ENV{ID_INPUT_TOUCHSCREEN}="1"
```

#### Why UART Instead of I²C?

**Advantages of UART for Touch Data:**
1. **Higher throughput** for continuous touch data streaming
2. **Lower latency** - no I²C arbitration needed
3. **Simpler protocol** for multi-touch packets
4. **Interrupt-driven** - UART RX triggers on new data
5. **DMA support** - hardware offload for data transfer

**I²C Still Used For:**
1. Firmware programming (via SWD/HSSP protocol)
2. Configuration registers (calibration, power, reset)
3. Device ID and version queries
4. `catnip` tool commands

#### GPIO Pin Assignment for UART Mode

```dts
gsbi10_serial_pins: gsbi10-serial-state {
    reset-pins {
        pins = "gpio70";  // TP_PWR_RST (Reset - shared with I²C)
        function = "gsbi10";
    };
    tx-pins {
        pins = "gpio71";  // UART TX from CY8CTMA395 to host
        function = "gsbi10";
    };
    // RX is implicit on gpio72 (when in UART mode, otherwise I2C SDA)
    // Clock is gpio73 (when in I2C mode, otherwise UART RX)
};
```

**Pin Muxing on GSBI10:**
- GPIO 70: Reset (both modes)
- GPIO 71: UART TX (UART mode) / unused (I²C mode)
- GPIO 72: UART RX (UART mode) / I²C SDA (I²C mode)
- GPIO 73: UART CTS (UART mode) / I²C SCL (I²C mode)

When `GSBI_PROT_I2C_UART` mode is set:
- GPIO 72/73 multiplex between I²C and UART as needed
- I²C for configuration, UART for continuous touch data

## Firmware Analysis

### Available Firmware Files

Located in `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/`:

| File | Size | Purpose |
|------|------|---------|
| `cy8ctma395.fw` | 82KB | Master controller firmware (binary) |
| `cy8ctma395.hex` | 159KB | Master controller firmware (Intel HEX) |
| `cy8ctma395_hssp_bridge.fw` | 82KB | HSSP bridge firmware for SWD programming |
| `cy8ctma395_hssp_bridge.hex` | 159KB | HSSP bridge firmware (Intel HEX) |
| `cy8ctma375.hex` | 36KB | **Slave controller firmware (Intel HEX)** |
| `cy8ctma395.ver` | 7 bytes | Version file: "15 0 0" (firmware v15.0.0) |

### Critical Discovery: PmTpUpdater Script Analysis

The `/usr/bin/PmTpUpdater` shell script reveals the **actual slave communication architecture**:

**Key Findings:**
```bash
INDIUM_TOTAL=5          # Confirms 5 slave controllers
BUS_NUM=5               # Slaves accessed via I²C bus 5
ADDR=0x69               # Bootloader I²C address (NOT 0x20-0x24!)
INDIUM_HEX="/lib/firmware/cy8ctma375.hex"  # Slave firmware
```

**Slave Programming Protocol (Cypress Bootloader Host):**
- `0x35` - Acquire Device (with INDIUM_TOTAL=5 parameter)
- `0x38` - Erase All
- `0x39` - Send Data (50 bytes at a time)
- `0x3A` - Program Row (28 bytes)
- `0x3B` - Verify Row
- `0x3E` - Read Checksum
- Silicon ID check: 0x05, 0x27

**Firmware Update Sequence:**
1. Stop `hidd` (HID daemon)
2. Power on CY8CTMA395 via `/sys/devices/platform/cy8ctma395/vdd`
3. Resume panel controller with `catnip` tool
4. Flash HSSP bridge firmware to master: `cy8ctma395_hssp_bridge.fw`
5. Flash slave firmware to all 5 CY8CTMA375 via I²C bus 5 at 0x69
6. Flash main firmware to master: `cy8ctma395.fw`
7. Calibrate panel with `catnip` (GIDAC and LIDAC tables)
8. Suspend panel and restart `hidd`

**Architecture Revelation:**

The slaves are **NOT on the main system I²C bus**. They're on a **private/internal I²C bus** accessible only through the CY8CTMA395 master controller. When the master is flashed with the HSSP bridge firmware, it acts as an I²C bridge/passthrough to program the 5 slaves.

This explains why:
- No GSBI5 I²C bus exists in the device tree (only GSBI3, 4, 6, 7, 8, 10, 12)
- The "bus 5" is likely an internal designation within the HSSP bridge firmware
- All 5 slaves use the same bootloader address 0x69 (selected via internal mechanism)
- The mainline driver cannot directly communicate with slaves

### Firmware Patterns (Secondary Evidence)

Analysis of `cy8ctma395.fw` reveals repeated patterns that may be internal slave references:

```
00002400  00 00 20 00 00 40  <- 0x20
00002520  00 00 21 00 00 40  <- 0x21
00002640  00 00 22 00 00 40  <- 0x22
00002760  00 00 23 00 00 40  <- 0x23
00002880  00 00 24 00 00 40  <- 0x24
```

These may be internal slave identifiers or addresses used by the master firmware for slave communication.

## Current Driver Limitations

### Mainline Linux cyttsp Driver

File: `/home/herrie/webos/touchpad-kernel/shr-linux/drivers/input/touchscreen/cyttsp_core.c`

**Critical Limitations:**

```c
#define CY_MAX_FINGER  4   // Line 67: HARDCODED 4 TOUCHES!
#define CY_MAX_ID      16  // 16 tracking IDs

struct cyttsp_xydata {
    u8 hst_mode;
    u8 tt_mode;
    u8 tt_stat;
    struct cyttsp_tch tch1;  // Touch 1
    u8 touch12_id;
    struct cyttsp_tch tch2;  // Touch 2
    u8 gest_cnt;
    u8 gest_id;
    struct cyttsp_tch tch3;  // Touch 3
    u8 touch34_id;
    struct cyttsp_tch tch4;  // Touch 4 (STOPS HERE!)
    // NO tch5, tch6, etc.
} __packed;
```

**Issues:**
1. ✗ Only supports 4 simultaneous touches (not 10+)
2. ✗ No multi-slave device support
3. ✗ Assumes single I²C device at one address
4. ✗ Cannot aggregate data from 5 slave controllers

### Firmware Programmer Driver

File: `/home/herrie/webos/touchpad-kernel/shr-linux/drivers/input/touchscreen/cy8ctma395.c`

**Purpose:** This is **NOT an input driver** - it's a firmware flash programmer using SWD protocol.

**Features:**
- SWD (Serial Wire Debug) protocol implementation
- Flash programming via HSSP (Host Source Serial Programming)
- Device ID and revision reading
- Sysfs interface for firmware updates

**GPIO Configuration:**
```c
#define GPIO_CTP_SCL             73   // SWDCK (Serial Wire Debug Clock)
#define GPIO_CTP_SDA             72   // SWDIO (Serial Wire Debug I/O)
#define GPIO_CY8CTMA395_XRES     70   // Reset pin (active low)
#define GPIO_CTP_WAKE           123   // Wake/Interrupt
```

## Device Tree Configuration

### Current Configuration

File: `/home/herrie/webos/touchpad-kernel/shr-linux/arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`

```dts
&gsbi10_i2c {
    clock-frequency = <400000>;

    touchscreen: cy8ctma395@67 {
        compatible = "cypress,cy8ctma340";  // Generic fallback
        reg = <0x67>;  // Master I²C address only

        vcpin-supply = <&vdd50_boost>;   // 5V analog supply
        vdd-supply = <&pm8058_l10>;      // 3.05V digital supply

        interrupts-extended = <&tlmm 123 IRQ_TYPE_LEVEL_LOW>;
        reset-gpios = <&tlmm 70 GPIO_ACTIVE_LOW>;

        touchscreen-size-x = <1024>;
        touchscreen-size-y = <768>;
    };
};
```

**Missing:** No slave device definitions!

## Original webOS Driver

### Evidence from webOS Kernel

File: `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/boot/config-2.6.35-palm-tenderloin`

```
CONFIG_TOUCHSCREEN_CY8CTMA395=y
```

**Status:** Driver was **compiled into kernel** (not as module)

**Search Results:**
- ✗ No .ko kernel module found in `/lib/modules/`
- ✗ No source code in webos-linux-kernel repository
- ✗ Driver was proprietary and never released
- ✓ Firmware files were released
- ✓ Firmware programmer driver was released

## Driver Implementation Guide

### ⚠️ REVISED APPROACH BASED ON PmTpUpdater ANALYSIS

**Key Insight:** The 5 CY8CTMA375 slaves are on an **internal/private I²C bus** accessible only through the CY8CTMA395 master firmware. The Linux driver **does NOT need to communicate with slaves directly**. The master firmware aggregates all touch data from the 5 slaves and presents it to the host as a single unified touch stream.

**This significantly simplifies the driver implementation!**

Instead of creating a complex multi-slave driver with device tree entries for each slave, we only need to:
1. Read aggregated touch data from the master at address 0x67
2. Parse potentially >4 simultaneous touches
3. Report events to the Linux input subsystem

**No device tree changes needed for slaves** (they don't exist on the system I²C bus).

### Option 1: Extend cyttsp_core.c (RECOMMENDED - Now Much Simpler!)

**Approach:** Modify the existing mainline cyttsp driver to support more than 4 simultaneous touches.

**Rationale:** Since the master firmware handles all slave communication internally and presents aggregated touch data, we only need to extend the single-device cyttsp driver to parse more touches. This is the simplest and most appropriate solution.

**Changes Required:**

1. **Increase touch limit:**
```c
// Change from:
#define CY_MAX_FINGER  4

// To:
#define CY_MAX_FINGER  10  // Or 16 for safety
```

2. **Extend cyttsp_xydata structure:**
```c
struct cyttsp_xydata {
    u8 hst_mode;
    u8 tt_mode;
    u8 tt_stat;
    struct cyttsp_tch tch1;
    u8 touch12_id;
    struct cyttsp_tch tch2;
    u8 gest_cnt;
    u8 gest_id;
    struct cyttsp_tch tch3;
    u8 touch34_id;
    struct cyttsp_tch tch4;
    u8 touch56_id;        // ADD
    struct cyttsp_tch tch5;  // ADD
    struct cyttsp_tch tch6;  // ADD
    // ... continue for tch7-tch10
} __packed;
```

3. **Update touch parsing loops** in `cyttsp_extract_track_ids()` and `cyttsp_report_tchdata()`

**Limitations:**
- Still treats it as a single device
- May not work if master actually reports multi-slave data differently
- Needs hardware testing to verify protocol

**Files to modify:**
- `drivers/input/touchscreen/cyttsp_core.c`
- `drivers/input/touchscreen/cyttsp_core.h`

### Option 2: Create New cy8ctma395 Driver (Only If Option 1 Fails)

**Status:** ⚠️ Likely **NOT NECESSARY** based on new findings.

**Only consider this if:** The master firmware uses a completely different protocol than standard CYTTSP, and extending cyttsp_core.c doesn't work.

**Original Approach:** Write a new driver specifically for the CY8CTMA395 multi-slave architecture.

**Note:** The complex multi-slave device tree architecture shown below is **NOT needed** since slaves aren't on the system bus. Keeping this section for reference only.

**Original Architecture (OUTDATED):**

```
┌─────────────────────────────────────────┐
│  Linux Input Subsystem                  │
│  (Reports aggregated multi-touch events)│
└────────────────┬────────────────────────┘
                 │
┌────────────────▼────────────────────────┐
│  cy8ctma395.c (New Multi-Slave Driver)  │
│  - Probe master + 5 slaves              │
│  - Aggregate touch data from all 6      │
│  - Transform coordinates to screen space│
│  - Report as unified multi-touch device │
└────────────────┬────────────────────────┘
                 │ I²C
     ┌───────────┴───────────┬───────────────┬─────────────┬─────────────┐
     │                       │               │             │             │
┌────▼──────┐   ┌────────────▼──┐   ┌───────▼─────┐  ┌────▼─────┐  ┌───▼──────┐
│CY8CTMA375 │   │  CY8CTMA375   │   │ CY8CTMA375  │  │CY8CTMA375│  │CY8CTMA375│
│ Slave 1   │   │  Slave 2      │   │  Slave 3    │  │ Slave 4  │  │ Slave 5  │
│ @0x20     │   │  @0x21        │   │  @0x22      │  │ @0x23    │  │ @0x24    │
│ Area:     │   │  Area:        │   │  Area:      │  │ Area:    │  │ Area:    │
│ 0-204px   │   │  205-409px    │   │  410-614px  │  │ 615-819px│  │ 820-1024px│
└───────────┘   └───────────────┘   └─────────────┘  └──────────┘  └──────────┘
```

**Implementation Steps:**

#### 1. Device Tree Bindings

Create `/Documentation/devicetree/bindings/input/touchscreen/cypress,cy8ctma395.yaml`:

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/input/touchscreen/cypress,cy8ctma395.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Cypress CY8CTMA395 Multi-Slave Touchscreen Controller

maintainers:
  - Your Name <your.email@example.com>

description:
  The Cypress CY8CTMA395 is a master touchscreen controller that coordinates
  with up to 5 CY8CTMA375 slave controllers to provide multi-touch input
  across large touchscreen panels. Used in the HP TouchPad (9.7" display).

allOf:
  - $ref: touchscreen.yaml#

properties:
  compatible:
    const: cypress,cy8ctma395

  reg:
    description: I2C address of the master controller
    maxItems: 1

  slave-controllers:
    description: Phandles to slave CY8CTMA375 controllers
    $ref: /schemas/types.yaml#/definitions/phandle-array
    minItems: 1
    maxItems: 5

  interrupts:
    maxItems: 1

  reset-gpios:
    maxItems: 1

  vcpin-supply:
    description: Analog power supply (typically 5V)

  vdd-supply:
    description: Digital power supply (typically 3.0V)

required:
  - compatible
  - reg
  - interrupts
  - reset-gpios
  - vcpin-supply
  - vdd-supply
  - slave-controllers

examples:
  - |
    #include <dt-bindings/gpio/gpio.h>
    #include <dt-bindings/interrupt-controller/irq.h>

    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        /* Slave controllers */
        touchscreen_slave1: cy8ctma375@20 {
            compatible = "cypress,cy8ctma375";
            reg = <0x20>;
        };

        touchscreen_slave2: cy8ctma375@21 {
            compatible = "cypress,cy8ctma375";
            reg = <0x21>;
        };

        touchscreen_slave3: cy8ctma375@22 {
            compatible = "cypress,cy8ctma375";
            reg = <0x22>;
        };

        touchscreen_slave4: cy8ctma375@23 {
            compatible = "cypress,cy8ctma375";
            reg = <0x23>;
        };

        touchscreen_slave5: cy8ctma375@24 {
            compatible = "cypress,cy8ctma375";
            reg = <0x24>;
        };

        /* Master controller */
        touchscreen@67 {
            compatible = "cypress,cy8ctma395";
            reg = <0x67>;

            interrupts-extended = <&tlmm 123 IRQ_TYPE_LEVEL_LOW>;
            reset-gpios = <&tlmm 70 GPIO_ACTIVE_LOW>;

            vcpin-supply = <&vdd50_boost>;
            vdd-supply = <&pm8058_l10>;

            touchscreen-size-x = <1024>;
            touchscreen-size-y = <768>;

            slave-controllers = <&touchscreen_slave1>,
                                <&touchscreen_slave2>,
                                <&touchscreen_slave3>,
                                <&touchscreen_slave4>,
                                <&touchscreen_slave5>;
        };
    };
```

#### 2. Driver Structure

Create `/drivers/input/touchscreen/cy8ctma395.c` (new input driver, not the firmware programmer):

```c
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cypress CY8CTMA395 Multi-Slave Touchscreen Driver
 *
 * Copyright (C) 2025 Your Name
 *
 * This driver supports the CY8CTMA395 master controller with
 * up to 5 CY8CTMA375 slave controllers as used in the HP TouchPad.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#define CY8CTMA395_MAX_SLAVES        5
#define CY8CTMA395_MAX_TOUCHES       16  // Conservative estimate
#define CY8CTMA395_TOUCH_DATA_SIZE   32  // To be determined from protocol analysis

/* Register addresses - TO BE DETERMINED from firmware analysis */
#define CY8CTMA395_REG_TOUCH_DATA    0x00
#define CY8CTMA395_REG_NUM_TOUCHES   0x02
#define CY8CTMA395_REG_DEVICE_MODE   0x10

/* Touch data structure - TO BE DETERMINED */
struct cy8ctma395_touch {
    u16 x;
    u16 y;
    u8 pressure;
    u8 id;
} __packed;

struct cy8ctma395_slave {
    struct i2c_client *client;
    int index;  /* 0-4 */
    u16 x_offset;  /* Screen coordinate offset */
    u16 x_max;     /* Local coordinate maximum */
};

struct cy8ctma395_data {
    struct i2c_client *client;        /* Master controller */
    struct input_dev *input;
    struct gpio_desc *reset_gpio;
    struct regulator *vcpin_supply;
    struct regulator *vdd_supply;

    struct cy8ctma395_slave slaves[CY8CTMA395_MAX_SLAVES];
    int num_slaves;

    struct touchscreen_properties prop;

    u8 touch_buf[CY8CTMA395_TOUCH_DATA_SIZE];
};

/* Helper: Reset the touchscreen controller */
static void cy8ctma395_hard_reset(struct cy8ctma395_data *ts)
{
    gpiod_set_value_cansleep(ts->reset_gpio, 1);
    msleep(10);
    gpiod_set_value_cansleep(ts->reset_gpio, 0);
    msleep(50);  /* Allow time for boot */
}

/* Read touch data from master controller */
static int cy8ctma395_read_touches(struct cy8ctma395_data *ts)
{
    int ret;

    /* Read touch data register */
    ret = i2c_smbus_read_i2c_block_data(ts->client,
                                        CY8CTMA395_REG_TOUCH_DATA,
                                        sizeof(ts->touch_buf),
                                        ts->touch_buf);
    if (ret < 0) {
        dev_err(&ts->client->dev, "Failed to read touch data: %d\n", ret);
        return ret;
    }

    return 0;
}

/* Parse and report touch events */
static void cy8ctma395_report_touches(struct cy8ctma395_data *ts)
{
    struct cy8ctma395_touch *touch;
    int i, num_touches;

    /* Parse number of active touches - protocol TBD */
    num_touches = ts->touch_buf[0];  /* PLACEHOLDER */

    for (i = 0; i < num_touches && i < CY8CTMA395_MAX_TOUCHES; i++) {
        touch = (struct cy8ctma395_touch *)&ts->touch_buf[2 + i * sizeof(*touch)];

        /* Report touch to input subsystem */
        input_mt_slot(ts->input, touch->id);
        input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);

        touchscreen_report_pos(ts->input, &ts->prop,
                              touch->x, touch->y, true);
        input_report_abs(ts->input, ABS_MT_PRESSURE, touch->pressure);
    }

    input_mt_sync_frame(ts->input);
    input_sync(ts->input);
}

/* IRQ handler */
static irqreturn_t cy8ctma395_irq(int irq, void *dev_id)
{
    struct cy8ctma395_data *ts = dev_id;

    if (cy8ctma395_read_touches(ts) == 0)
        cy8ctma395_report_touches(ts);

    return IRQ_HANDLED;
}

/* Probe slave controllers */
static int cy8ctma395_probe_slaves(struct cy8ctma395_data *ts)
{
    struct device *dev = &ts->client->dev;
    struct device_node *slave_node;
    struct i2c_client *slave_client;
    int i = 0;

    /* Iterate through slave-controllers property */
    while (i < CY8CTMA395_MAX_SLAVES) {
        slave_node = of_parse_phandle(dev->of_node, "slave-controllers", i);
        if (!slave_node)
            break;

        slave_client = of_find_i2c_device_by_node(slave_node);
        of_node_put(slave_node);

        if (!slave_client) {
            dev_err(dev, "Failed to find slave %d I2C device\n", i);
            return -ENODEV;
        }

        ts->slaves[i].client = slave_client;
        ts->slaves[i].index = i;

        /* Calculate screen area coverage - assuming equal horizontal division */
        ts->slaves[i].x_offset = (ts->prop.max_x / CY8CTMA395_MAX_SLAVES) * i;
        ts->slaves[i].x_max = ts->prop.max_x / CY8CTMA395_MAX_SLAVES;

        dev_info(dev, "Slave %d at address 0x%02x, X offset %d\n",
                 i, slave_client->addr, ts->slaves[i].x_offset);

        i++;
    }

    ts->num_slaves = i;
    dev_info(dev, "Configured with %d slave controllers\n", ts->num_slaves);

    return 0;
}

/* Probe function */
static int cy8ctma395_probe(struct i2c_client *client)
{
    struct cy8ctma395_data *ts;
    int ret;

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
        return -ENOMEM;

    ts->client = client;
    i2c_set_clientdata(client, ts);

    /* Get reset GPIO */
    ts->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(ts->reset_gpio))
        return PTR_ERR(ts->reset_gpio);

    /* Get power supplies */
    ts->vcpin_supply = devm_regulator_get(&client->dev, "vcpin");
    if (IS_ERR(ts->vcpin_supply))
        return PTR_ERR(ts->vcpin_supply);

    ts->vdd_supply = devm_regulator_get(&client->dev, "vdd");
    if (IS_ERR(ts->vdd_supply))
        return PTR_ERR(ts->vdd_supply);

    /* Enable power */
    ret = regulator_enable(ts->vcpin_supply);
    if (ret)
        return ret;

    ret = regulator_enable(ts->vdd_supply);
    if (ret)
        goto err_disable_vcpin;

    /* Reset controller */
    cy8ctma395_hard_reset(ts);

    /* Allocate input device */
    ts->input = devm_input_allocate_device(&client->dev);
    if (!ts->input) {
        ret = -ENOMEM;
        goto err_disable_vdd;
    }

    ts->input->name = "Cypress CY8CTMA395 Multi-Touch";
    ts->input->id.bustype = BUS_I2C;

    /* Parse touchscreen properties from DT */
    input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 1023, 0, 0);
    input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 767, 0, 0);
    input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

    touchscreen_parse_properties(ts->input, true, &ts->prop);

    ret = input_mt_init_slots(ts->input, CY8CTMA395_MAX_TOUCHES,
                             INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
    if (ret)
        goto err_disable_vdd;

    /* Probe slave controllers */
    ret = cy8ctma395_probe_slaves(ts);
    if (ret)
        goto err_disable_vdd;

    /* Register input device */
    ret = input_register_device(ts->input);
    if (ret)
        goto err_disable_vdd;

    /* Request IRQ */
    ret = devm_request_threaded_irq(&client->dev, client->irq,
                                   NULL, cy8ctma395_irq,
                                   IRQF_ONESHOT,
                                   "cy8ctma395", ts);
    if (ret) {
        dev_err(&client->dev, "Failed to request IRQ: %d\n", ret);
        goto err_disable_vdd;
    }

    dev_info(&client->dev, "CY8CTMA395 multi-slave touchscreen initialized\n");
    return 0;

err_disable_vdd:
    regulator_disable(ts->vdd_supply);
err_disable_vcpin:
    regulator_disable(ts->vcpin_supply);
    return ret;
}

static void cy8ctma395_remove(struct i2c_client *client)
{
    struct cy8ctma395_data *ts = i2c_get_clientdata(client);

    regulator_disable(ts->vdd_supply);
    regulator_disable(ts->vcpin_supply);
}

static const struct of_device_id cy8ctma395_of_match[] = {
    { .compatible = "cypress,cy8ctma395" },
    { }
};
MODULE_DEVICE_TABLE(of, cy8ctma395_of_match);

static const struct i2c_device_id cy8ctma395_id[] = {
    { "cy8ctma395", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, cy8ctma395_id);

static struct i2c_driver cy8ctma395_driver = {
    .driver = {
        .name = "cy8ctma395",
        .of_match_table = cy8ctma395_of_match,
    },
    .probe = cy8ctma395_probe,
    .remove = cy8ctma395_remove,
    .id_table = cy8ctma395_id,
};

module_i2c_driver(cy8ctma395_driver);

MODULE_AUTHOR("Your Name <your.email@example.com>");
MODULE_DESCRIPTION("Cypress CY8CTMA395 Multi-Slave Touchscreen Driver");
MODULE_LICENSE("GPL");
```

#### 3. Device Tree Update

Update `/arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`:

```dts
&gsbi10_i2c {
    clock-frequency = <400000>;

    /* Slave touchscreen controllers */
    touchscreen_slave1: cy8ctma375@20 {
        compatible = "cypress,cy8ctma375";
        reg = <0x20>;
        status = "okay";
    };

    touchscreen_slave2: cy8ctma375@21 {
        compatible = "cypress,cy8ctma375";
        reg = <0x21>;
        status = "okay";
    };

    touchscreen_slave3: cy8ctma375@22 {
        compatible = "cypress,cy8ctma375";
        reg = <0x22>;
        status = "okay";
    };

    touchscreen_slave4: cy8ctma375@23 {
        compatible = "cypress,cy8ctma375";
        reg = <0x23>;
        status = "okay";
    };

    touchscreen_slave5: cy8ctma375@24 {
        compatible = "cypress,cy8ctma375";
        reg = <0x24>;
        status = "okay";
    };

    /* Master touchscreen controller */
    touchscreen: cy8ctma395@67 {
        compatible = "cypress,cy8ctma395";
        reg = <0x67>;

        vcpin-supply = <&vdd50_boost>;   // 5V analog supply
        vdd-supply = <&pm8058_l10>;      // 3.05V digital supply

        interrupts-extended = <&tlmm 123 IRQ_TYPE_LEVEL_LOW>;
        reset-gpios = <&tlmm 70 GPIO_ACTIVE_LOW>;

        touchscreen-size-x = <1024>;
        touchscreen-size-y = <768>;

        slave-controllers = <&touchscreen_slave1>,
                            <&touchscreen_slave2>,
                            <&touchscreen_slave3>,
                            <&touchscreen_slave4>,
                            <&touchscreen_slave5>;
    };
};
```

#### 4. Kconfig Entry

Add to `/drivers/input/touchscreen/Kconfig`:

```kconfig
config TOUCHSCREEN_CY8CTMA395
    tristate "Cypress CY8CTMA395 multi-slave touchscreen"
    depends on I2C
    select REGULATOR
    help
      Say Y here if you have a Cypress CY8CTMA395 touchscreen
      controller with CY8CTMA375 slave controllers, as used in
      the HP TouchPad.

      This driver supports the multi-slave architecture where
      one master controller coordinates with up to 5 slave
      controllers to provide multi-touch input across large
      touchscreen panels.

      To compile this driver as a module, choose M here: the
      module will be called cy8ctma395.
```

#### 5. Makefile Entry

Add to `/drivers/input/touchscreen/Makefile`:

```makefile
obj-$(CONFIG_TOUCHSCREEN_CY8CTMA395) += cy8ctma395.o
```

## Critical Unknown Information

### Now Confirmed via PmTpUpdater Analysis

1. **✓ Slave Architecture:**
   - **Confirmed:** 5 CY8CTMA375 slaves exist (INDIUM_TOTAL=5)
   - **Confirmed:** Slaves on private/internal I²C bus, NOT on system bus
   - **Confirmed:** Master firmware handles all slave communication internally
   - **Confirmed:** Bootloader address 0x69 (for programming only, not runtime)

2. **✓ Firmware Loading:**
   - **Confirmed:** Firmware is pre-loaded (not runtime loaded by driver)
   - **Confirmed:** Update process uses HSSP bridge mode of master
   - **Confirmed:** Slave firmware: `cy8ctma375.hex`, Master firmware: `cy8ctma395.fw`
   - **Tool:** `PmTpUpdater` script + `catnip` binary

### Now Confirmed via HidTouchpanel_evt1.xml Analysis

The webOS HID daemon configuration file (`/etc/hidd/HidTouchpanel_evt1.xml`) provides
authoritative answers to many previously unknown parameters:

3. **✓ Sensor Matrix & Resolution:**
   - **Confirmed:** 30 columns × 40 rows capacitive sensor matrix
   - **Confirmed:** Screen resolution 768×1024 pixels
   - **Confirmed:** 90° rotation angle (landscape to portrait)
   - **Confirmed:** X/Y translation offset: 6 pixels each
   - **Confirmed:** Grid size: 145.44mm × 194.62mm

4. **✓ Multi-Touch Capability:**
   - **Confirmed:** Maximum 10 simultaneous fingers (not 4!)
   - **Confirmed:** Finger detection threshold: 20
   - **Confirmed:** Finger down threshold: 40
   - **Confirmed:** Noise threshold: 10

5. **✓ Timing Parameters:**
   - **Confirmed:** Scan rate: 100 Hz (range 2-500 Hz)
   - **Confirmed:** Boot delay: 50 ms (bootNs = 50000000)
   - **Confirmed:** Wake delay: 500 µs (wakeNs = 500000)
   - **Confirmed:** UART timeout: 100 ms
   - **Confirmed:** Idle scan rate: 30 Hz (power saving)
   - **Confirmed:** Idle timeout: 15 seconds

6. **✓ Hardware Paths:**
   - **Confirmed:** UART path: `/dev/ctp_uart` (maps to `/dev/ttyMSM2`)
   - **Confirmed:** I²C bus: 5 (webOS numbering), address: 0x67 (103 decimal)
   - **Confirmed:** Reset sysfs: `/sys/devices/platform/cy8ctma395/xres`
   - **Confirmed:** Power sysfs: `/sys/devices/platform/cy8ctma395/vdd`
   - **Confirmed:** Wake GPIO: `/sys/user_hw/pins/ctp/wake/level`

7. **✓ Coordinate Mapping:**
   - **Confirmed:** NOT divided among slaves - master aggregates all data
   - **Confirmed:** Single unified 30×40 matrix presented to host
   - **Confirmed:** Rotation handled in software (90° rotation)
   - **Confirmed:** Margin settings: 9 pixels on all edges

### Now Confirmed via Hardware Capture (2026-01-11)

8. **✓ Raw Sensor Data Format:**
   - **Confirmed:** `catnip raw` returns 1200 integer values (30×40 matrix)
   - **Confirmed:** Values are raw capacitance readings (~160-195 baseline)
   - **Confirmed:** Touch reduces capacitance (lower value = touch detected)
   - **Confirmed:** Threshold ~20 below average indicates touch

9. **✓ Touch Detection Example:**
   - Baseline average: ~178
   - Touch threshold: ~158 (avg - 20)
   - Touch point at Row 24-25, Col 17: value 110-149 (clear touch)
   - Matrix orientation: 40 rows × 30 columns

### Still Unknown (Requires UART Capture with hidd Stopped)

1. **UART Packet Details:**
   - **Partially known:** Packet headers 0xFF 0x43 (data) and 0xFF 0x47 (complete)
   - **Challenge:** /dev/ctp_uart locked by hidd kernel driver
   - **Workaround:** Use `catnip raw` via I²C for sensor data

2. **Touch Blob Detection:**
   - **Known:** Thresholds from XML (fingerThreshold=20, blobThreshDiff=5/30)
   - **Known:** Raw value ~20 below average = touch
   - **ts-srv:** Reference implementation available

3. **I²C Initialization Sequence:**
   - **Partially known:** 7-command sequence from ts-srv source
   - **Known:** Commands work (I2C accessible after hidd starts)

## WebOS Tools for Debugging

### Available Utilities

Located in `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/usr/bin/`:

#### 1. `catnip` (87KB ARM binary)

Touch panel controller interface tool with the following capabilities:

**Commands:**
- `catnip ver -a` - Read firmware version from CY8CTMA395
- `catnip cal -s 64` - Calibrate panel with signal target 64
- `catnip gidac -u` - Retrieve global IDAC (Digital-to-Analog Converter) table
- `catnip lidac -u` - Retrieve local IDAC table
- `catnip resume` - Resume panel controller operation
- `catnip suspend` - Suspend panel controller operation
- `catnip --i2c-only` - Force I²C-only mode (bypass HID interface)

**Options:**
- `--i2c-bus <num>` - Specify I²C bus number
- `--i2c-addr <addr>` - Specify I²C slave address
- `--i2c-retries <num>` - Set I²C retry attempts

**Sysfs Interface:**
- `/sys/devices/platform/cy8ctma395/vdd` - Power control (write 1 to enable)
- `/sys/devices/platform/cy8ctma395/xres` - Reset control
- `/sys/devices/platform/cy8ctma395/id` - Read device ID
- `/sys/devices/platform/cy8ctma395/flash` - Firmware flash interface

**Usage Example:**
```bash
# Get current firmware version
catnip --i2c-only ver -a

# Calibrate touch panel
catnip --i2c-only cal -s 64

# Read calibration tables
catnip gidac -u
catnip lidac -u
```

#### 2. `i2c-drv` (16KB ARM binary)

Low-level I²C bus access utility for reading/writing arbitrary I²C devices.

**Usage:**
```bash
# Write to I²C device
i2c-drv w <bus> <addr> "0x01 0x02 0x03"

# Read from I²C device
i2c-drv r <bus> <addr> <num_bytes>
```

**Example (from PmTpUpdater):**
```bash
# Acquire bootloader device (command 0x35)
i2c-drv w 5 0x69 "0x01 0x35 0x01 0x00 0x05 <checksum> 0x17"

# Read response (9 bytes)
result=$(i2c-drv r 5 0x69 9)
```

#### 3. `ihex` Tool

Intel HEX file parser (used by PmTpUpdater script):

**Commands:**
- `ihex size <file> <section>` - Get data section size
- `ihex read <file> <addr> <length>` - Read bytes from HEX file

**Example:**
```bash
# Get firmware data section size
ihex size /lib/firmware/cy8ctma375.hex 0

# Read 50 bytes from address 0
ihex read /lib/firmware/cy8ctma375.hex 0 50
```

#### 4. Luna Service Commands

The webOS Luna bus provides high-level access to touchpanel functions:

**Raw Data Dump (captures sensor matrix):**
```bash
# Start raw data capture (10 seconds)
luna-send -n 1 luna://com.palm.hidd/HidTouchpanel/RawDataDumpState '{"mode":"set","value":"on"}'
sleep 10
luna-send -n 1 luna://com.palm.hidd/HidTouchpanel/RawDataDumpState '{"mode":"set","value":"off"}'
# Data saved to: /var/tmp/hidd/touchpanel_raw*
```

**Complete Data Dump (includes processed coordinates):**
```bash
luna-send -n 1 luna://com.palm.hidd/HidTouchpanel/CompleteDumpState '{"mode":"set","value":"on"}'
sleep 12
luna-send -n 1 luna://com.palm.hidd/HidTouchpanel/CompleteDumpState '{"mode":"set","value":"off"}'
# Data saved to: /var/tmp/hidd/
```

#### 5. `touchpanel_control` Script

Convenience script for common operations:

```bash
touchpanel_control enable_wot      # Enable wake-on-touch
touchpanel_control disable_wot     # Disable wake-on-touch
touchpanel_control start_recording # Start raw data recording
touchpanel_control stop_recording  # Stop raw data recording
touchpanel_control always_record   # Always record to /media/internal
touchpanel_control enable_transient  # Enable transient debug
touchpanel_control disable_transient # Disable transient debug
```

#### 6. `touchpanel-measure` Binary

Low-level measurement tool for touchpanel diagnostics (ARM binary).

### Data Gathering Commands for webOS Device

Run these commands on a TouchPad booted into webOS to gather protocol information:

**Quick Information Gathering:**
```bash
# 1. Get firmware version
catnip --i2c-only ver -a

# 2. Get calibration tables (reveals internal configuration)
catnip gidac -u
catnip lidac -u

# 3. Read I²C registers from master controller
i2c-drv r 5 0x67 32   # Read 32 bytes from 0x67

# 4. Check sysfs interfaces
cat /sys/devices/platform/cy8ctma395/id
cat /sys/devices/platform/cy8ctma395/vdd
```

**Capture Raw Touch Data:**
```bash
# Method 1: Using Luna service (recommended)
luna-send -n 1 luna://com.palm.hidd/HidTouchpanel/RawDataDumpState '{"mode":"set","value":"on"}'
# Touch the screen in various patterns for 10-15 seconds
sleep 15
luna-send -n 1 luna://com.palm.hidd/HidTouchpanel/RawDataDumpState '{"mode":"set","value":"off"}'

# Retrieve the data
tar -czf touchpanel_capture.tar.gz /var/tmp/hidd/touchpanel_raw*

# Method 2: Enable persistent recording
touchpanel_control always_record
# Data continuously written to /media/internal/touchpaneldump/
```

**Capture UART Traffic (requires root/dev access):**
```bash
# Stop hidd to release UART
stop hidd

# Power on touchscreen
echo 1 > /sys/devices/platform/cy8ctma395/vdd
sleep 0.1

# Initialize via I²C (from ts-srv sequence)
i2c-drv w 5 0x67 "0x08 0x00"
i2c-drv w 5 0x67 "0x31 0x01 0x08 0x0C 0x0D 0x0A"
i2c-drv w 5 0x67 "0x30 0x0F"
i2c-drv w 5 0x67 "0x40 0x02"
i2c-drv w 5 0x67 "0x41 0x10"
i2c-drv w 5 0x67 "0x0A 0x04"
i2c-drv w 5 0x67 "0x08 0x03"  # Start UART streaming

# Capture UART data (need stty for 4Mbps or use hexdump)
cat /dev/ctp_uart | hexdump -C > uart_capture.hex &
# Touch screen, then kill the capture
```

### Using These Tools for Protocol Analysis

**Step 1: Boot webOS and extract tools**
```bash
# Copy binaries from webOS image
cp doctor305/nova-cust-image-topaz.rootfs/usr/bin/catnip /tmp/
cp doctor305/nova-cust-image-topaz.rootfs/usr/bin/i2c-drv /tmp/
chmod +x /tmp/catnip /tmp/i2c-drv
```

**Step 2: Monitor touch controller communication**
```bash
# Enable I²C debugging
echo "file drivers/i2c/* +p" > /sys/kernel/debug/dynamic_debug/control

# Read firmware version
/tmp/catnip --i2c-only --i2c-bus 10 --i2c-addr 0x67 ver -a

# Monitor dmesg for I²C traffic
dmesg -w | grep i2c
```

**Step 3: Capture touch data**

If webOS is running with original driver:
```bash
# Use catnip to read touch data (commands TBD - reverse engineer from binary)
# Or monitor /dev/input/eventX with evtest while using i2c-drv to capture
```

## Testing Strategy

### Phase 1: Hardware Reconnaissance

1. **Boot original webOS and capture I²C traffic:**
   ```bash
   # Enable I²C debugging
   echo "file drivers/i2c/* +p" > /sys/kernel/debug/dynamic_debug/control

   # Or use I²C bus sniffer hardware
   ```

2. **Scan I²C bus on Linux:**
   ```bash
   i2cdetect -y 10  # GSBI10 I²C bus
   ```
   Expected output: 0x20-0x24 (slaves) + 0x67 (master)

3. **Read registers from master:**
   ```bash
   i2cdump -y 10 0x67  # Dump all registers from master
   ```

4. **Monitor touch events:**
   ```bash
   evtest /dev/input/event0  # Original driver
   # Compare with I²C traffic during touches
   ```

### Phase 2: Protocol Reverse Engineering

1. Analyze register dumps during idle vs. touch states
2. Correlate touch positions with coordinate data in registers
3. Map slave addresses to screen regions
4. Document packet format and timing

### Phase 3: Driver Development

1. Implement basic driver with hardcoded protocol assumptions
2. Test on hardware with minimal functionality
3. Iterate based on actual hardware behavior
4. Add error handling and edge cases

### Phase 4: Mainline Integration

1. Clean up code to meet kernel coding standards
2. Write comprehensive device tree binding documentation
3. Submit patches to linux-input mailing list
4. Address review comments

## References

- **HP TouchPad Technical Specifications:** `/home/herrie/webos/touchpad-kernel/specs/Tech-Specs-hptouchpad-120110090301-phpapp02.1.pdf` (pages 18-19)
- **Firmware Files:** `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/cy8ctma*`
- **Mainline cyttsp Driver:** `/home/herrie/webos/touchpad-kernel/shr-linux/drivers/input/touchscreen/cyttsp_core.c`
- **Firmware Programmer:** `/home/herrie/webos/touchpad-kernel/shr-linux/drivers/input/touchscreen/cy8ctma395.c`
- **webOS Kernel Config:** `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/boot/config-2.6.35-palm-tenderloin`
- **Device Tree:** `/home/herrie/webos/touchpad-kernel/shr-linux/arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`

## Summary of Key Discoveries

### Critical Findings from PmTpUpdater Analysis

1. **✓ Multi-Slave Architecture Confirmed**
   - 5× CY8CTMA375 slave controllers (INDIUM_TOTAL=5)
   - Slaves on **private/internal I²C bus**, NOT directly accessible from host
   - Master CY8CTMA395 firmware aggregates all slave touch data

2. **✓ Simplified Driver Requirements**
   - **No need for multi-slave driver complexity**
   - **No device tree entries needed for slaves**
   - Driver only needs to communicate with master at 0x67 on GSBI10
   - UART driver required (not I²C cyttsp driver)

3. **✓ Firmware Update Process Documented**
   - Uses `PmTpUpdater` script + `catnip` + `i2c-drv` tools
   - Master loads HSSP bridge firmware to program slaves
   - Slaves flashed via Cypress Bootloader Host protocol (0x35-0x3E commands)
   - Calibration data stored as GIDAC and LIDAC tables

4. **✓ Available Tools for Testing**
   - `catnip` - Full-featured touch panel interface tool
   - `i2c-drv` - Low-level I²C read/write utility
   - `ihex` - Intel HEX firmware parser
   - Luna service commands for raw data capture
   - `touchpanel_control` script for quick operations

### Critical Findings from HidTouchpanel_evt1.xml Analysis

5. **✓ Sensor Configuration Confirmed**
   - 30×40 capacitive sensor matrix
   - 768×1024 screen resolution with 90° rotation
   - **10 finger multi-touch** (not 4!)
   - Finger threshold: 20, Down threshold: 40

6. **✓ Timing Parameters Confirmed**
   - Scan rate: 100 Hz (range 2-500 Hz)
   - Boot delay: 50 ms
   - UART timeout: 100 ms
   - Idle scan rate: 30 Hz

7. **✓ Hardware Paths Confirmed**
   - UART: `/dev/ctp_uart` → `/dev/ttyMSM2`
   - I²C: Bus 5 (webOS) / GSBI10 (Linux), address 0x67
   - Sysfs: `/sys/devices/platform/cy8ctma395/{vdd,xres,id,flash}`

### Knowledge Status Summary

| Parameter | Status | Value |
|-----------|--------|-------|
| Sensor matrix | ✅ Confirmed | 30×40 (1200 cells) |
| Screen resolution | ✅ Confirmed | 768×1024 |
| Max fingers | ✅ Confirmed | 10 |
| Rotation | ✅ Confirmed | 90° |
| I²C address | ✅ Confirmed | 0x67 |
| UART baud | ✅ Confirmed | 4 Mbps |
| Scan rate | ✅ Confirmed | 100 Hz |
| Packet headers | ✅ Confirmed | 0xFF 0x43/0x47 |
| Slave count | ✅ Confirmed | 5× CY8CTMA375 |
| Raw data format | ✅ Confirmed | 1200 integers via `catnip raw` |
| Baseline values | ✅ Confirmed | ~160-195 (capacitance) |
| Touch threshold | ✅ Confirmed | ~20 below average |
| Firmware version | ✅ Confirmed | 15.0.0 |
| Device ID | ✅ Confirmed | 0x1e01e069 rev 3 |
| GIDAC calibration | ✅ Confirmed | 150 values (30×5) |
| LIDAC calibration | ✅ Confirmed | 1200 values (30×40) |
| UART packet layout | ⏳ Partial | Locked by kernel driver |
| I²C init sequence | ⏳ Partial | 7 commands known |

### Impact on Driver Development

**Before Analysis:**
- Assumed I²C-only communication
- Planned to use/extend cyttsp driver
- Expected complex multi-slave management

**After Analysis:**
- Confirmed UART for touch data, I²C for config only
- Need UART/serdev driver (ts-srv or custom)
- Slaves handled internally by master firmware
- Much clearer implementation path

## Conclusion

### The Complete Picture

Based on all evidence gathered, here's how the HP TouchPad touchscreen actually works:

```
┌─────────────────────────────────────────────────────────────┐
│  APQ8060 Host Processor                                     │
│                                                              │
│  ┌──────────────────────────────────────────┐              │
│  │  GSBI10 Interface                         │              │
│  │                                           │              │
│  │  ┌─────────┐         ┌─────────┐         │              │
│  │  │  UART   │────────►│  I²C    │         │              │
│  │  │(ttyMSM2)│         │(bus 10) │         │              │
│  │  └─────────┘         └─────────┘         │              │
│  │      │                    │               │              │
│  └──────┼────────────────────┼───────────────┘              │
│         │                    │                              │
└─────────┼────────────────────┼──────────────────────────────┘
          │                    │
          │ Touch Data         │ Config/Firmware
          │ (Runtime)          │ (Init/Update)
          │                    │
       GPIO71,72               GPIO72,73
          ↓                    ↓
┌─────────────────────────────────────────────────────────────┐
│  Cypress CY8CTMA395 Master Controller                       │
│  - Runs custom webOS firmware                               │
│  - Aggregates touch data from 5 slaves                      │
│  - Transmits via UART to host (binary protocol)             │
│  - Receives config via I²C (calibration, power, firmware)   │
│                                                              │
│  Internal/Private I²C Bus (not accessible from host)        │
│  ├── CY8CTMA375 Slave 1 (screen region 1)                  │
│  ├── CY8CTMA375 Slave 2 (screen region 2)                  │
│  ├── CY8CTMA375 Slave 3 (screen region 3)                  │
│  ├── CY8CTMA375 Slave 4 (screen region 4)                  │
│  └── CY8CTMA375 Slave 5 (screen region 5)                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### REVISED Recommended Approach

**❌ Option 1: Extend cyttsp_core.c** (ABANDONED)
- This approach is fundamentally wrong
- cyttsp is an I²C driver, but touch data comes over UART
- Cannot work for runtime touch events

**✅ NEW Option: Implement UART-based Touchscreen Driver**

Two implementation paths:

#### Path A: Userspace Driver (PostmarketOS Approach - WORKING)

**Pros:**
- Already proven to work by PostmarketOS
- Simpler to develop and debug
- No kernel module needed
- Can be updated without kernel recompile

**Cons:**
- Not suitable for mainline Linux
- Higher latency than kernel driver
- More complex user-space setup

**Implementation:**
1. Set `USE_I2C_TOUCHSCREEN_DRIVER=0` in device tree
2. Enable GSBI10 UART mode (`GSBI_PROT_I2C_UART`)
3. Port PostmarketOS `ts-srv` to modern Linux
4. Reverse engineer UART protocol from `ts-srv` source
5. Read from `/dev/ttyMSM2`, parse touch packets, inject via uinput

#### Path B: Kernel UART Driver (Mainline Path - RECOMMENDED)

**Pros:**
- Proper kernel integration
- Lower latency
- Suitable for mainline Linux
- Follows kernel driver model

**Cons:**
- More complex development
- Requires UART protocol reverse engineering
- Needs serdev framework integration

**Implementation:**
1. Create new `drivers/input/touchscreen/cy8ctma395_uart.c`
2. Use Linux `serdev` framework for UART communication
3. Reverse engineer UART packet format from `ts-srv`
4. Implement as serio/serdev touch device
5. Device tree binding for serdev touchscreen

**Driver Structure:**
```c
// Simplified kernel driver approach
#include <linux/serdev.h>
#include <linux/input.h>
#include <linux/input/mt.h>

struct cy8ctma395_uart_data {
    struct serdev_device *serdev;
    struct input_dev *input;
    u8 rx_buf[256];  // UART receive buffer
    // Protocol state machine
};

static int cy8ctma395_uart_receive_buf(struct serdev_device *serdev,
                                       const unsigned char *buf,
                                       size_t count)
{
    // Parse UART packets
    // Extract touch coordinates
    // Report to input subsystem
}

static int cy8ctma395_uart_probe(struct serdev_device *serdev)
{
    // Allocate input device
    // Set up serdev callbacks
    // Initialize touch controller via I2C (separate i2c_client)
    // Start UART reception
}
```

**Device Tree:**
```dts
&gsbi10 {
    status = "okay";
    qcom,mode = <GSBI_PROT_I2C_UART>;  // Enable both!
};

&gsbi10_serial {
    status = "okay";

    touchscreen {
        compatible = "cypress,cy8ctma395-uart";
        // No reg needed - it's the serial device

        // I2C for config (phandle to i2c device)
        config-i2c = <&touchscreen_i2c>;

        touchscreen-size-x = <1024>;
        touchscreen-size-y = <768>;
    };
};

&gsbi10_i2c {
    status = "okay";

    touchscreen_i2c: cy8ctma395@67 {
        compatible = "cypress,cy8ctma395-config";
        reg = <0x67>;

        vcpin-supply = <&vdd50_boost>;
        vdd-supply = <&pm8058_l10>;
        reset-gpios = <&tlmm 70 GPIO_ACTIVE_LOW>;
    };
};
```

### Implementation Roadmap

Now that we have complete protocol knowledge from the `ts-srv` source code analysis:

#### Phase 1: Device Tree Configuration (IMMEDIATE)

**File:** `/arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`

**Changes Required:**

1. **Set UART mode:**
   ```dts
   // Change from:
   #define USE_I2C_TOUCHSCREEN_DRIVER 1

   // To:
   #define USE_I2C_TOUCHSCREEN_DRIVER 0
   ```

2. **This automatically enables:**
   - GSBI10 dual mode: `GSBI_PROT_I2C_UART`
   - `/dev/ttyMSM2` serial device
   - Both I²C and UART interfaces active

3. **Verify UART pins are configured:**
   ```dts
   &gsbi10_serial {
       status = "okay";
       pinctrl-0 = <&gsbi10_serial_pins>;
   };
   ```

#### Phase 2: Quick Userspace Prototype (1-2 days)

**Goal:** Verify UART communication works on hardware

**Steps:**

1. **Build kernel with UART mode enabled**
   ```bash
   # In device tree
   USE_I2C_TOUCHSCREEN_DRIVER=0

   # Rebuild DTBs
   make dtbs
   ```

2. **Create simple UART test program:**
   ```c
   #include <stdio.h>
   #include <fcntl.h>
   #include <termios.h>
   #include <unistd.h>

   int main() {
       int fd = open("/dev/ttyMSM2", O_RDONLY | O_NOCTTY);
       struct termios tty;

       // Configure for 4 Mbps (custom baud rate via ioctl)
       // Or use standard baud rate for testing
       cfsetispeed(&tty, B3000000);  // Closest standard rate
       tcsetattr(fd, TCSANOW, &tty);

       unsigned char buf[1540];
       while (1) {
           int n = read(fd, buf, sizeof(buf));
           if (n > 0) {
               // Look for 0xFF 0x43 and 0xFF 0x47 packets
               for (int i = 0; i < n-1; i++) {
                   if (buf[i] == 0xFF && buf[i+1] == 0x43)
                       printf("Found scan data packet at offset %d\n", i);
                   if (buf[i] == 0xFF && buf[i+1] == 0x47)
                       printf("Found scan complete packet at offset %d\n", i);
               }
           }
       }
   }
   ```

3. **Send I²C initialization via command line:**
   ```bash
   # Power on sequence
   echo 1 > /sys/devices/platform/cy8ctma395/xres
   echo 1 > /sys/devices/platform/cy8ctma395/vdd
   sleep 0.05
   echo 1 > /sys/devices/platform/cy8ctma395/wake
   sleep 0.05
   echo 0 > /sys/devices/platform/cy8ctma395/xres
   sleep 0.05
   echo 0 > /sys/devices/platform/cy8ctma395/wake
   sleep 0.05

   # I²C configuration (use i2c-tools)
   i2cset -y 10 0x67 0x08 0x00 i
   i2cset -y 10 0x67 0x31 0x01 0x08 0x0C 0x0D 0x0A i
   i2cset -y 10 0x67 0x30 0x0F i
   i2cset -y 10 0x67 0x40 0x02 i
   i2cset -y 10 0x67 0x41 0x10 i
   i2cset -y 10 0x67 0x0A 0x04 i
   i2cset -y 10 0x67 0x08 0x03 i  # Start UART stream
   ```

4. **Run test program:**
   ```bash
   ./uart_test
   # Should see packet detection when touching screen
   ```

#### Phase 3: Port ts-srv to Modern Linux (1 week)

**Goal:** Working touchscreen with userspace driver

**Approach:**

1. **Clone and adapt Evervolv ts-srv:**
   ```bash
   git clone https://github.com/Evervolv/android_device_hp_tenderloin-common.git
   cd touchscreen_drv
   ```

2. **Remove Android dependencies:**
   - Replace `<cutils/log.h>` with standard `syslog()`
   - Change `/dev/ctp_uart` to `/dev/ttyMSM2`
   - Replace Android input injection with `uinput`

3. **Modify for standard Linux:**
   ```c
   // Replace HSUART_IOCTL_* with standard termios
   struct termios tty;
   cfsetispeed(&tty, B3000000);  // Or custom ioctl for 4 Mbps
   cfmakeraw(&tty);
   tcsetattr(fd, TCSANOW, &tty);

   // Replace Android input with uinput
   int uinput_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
   // Configure MT slots, ABS_X, ABS_Y, etc.
   ```

4. **Integrate power management:**
   - Add I²C initialization via `i2c-dev`
   - Use sysfs for GPIO control
   - Add systemd service file

5. **Test and iterate:**
   - Run `ts-srv` daemon
   - Test with `evtest` to see input events
   - Calibrate thresholds if needed

#### Phase 4: Kernel serdev Driver (2-3 weeks)

**Goal:** Proper mainline kernel driver

**File:** `/drivers/input/touchscreen/cy8ctma395_uart.c`

**Implementation:**

```c
// Skeleton structure
#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>

#define PACKET_SCAN_DATA   0x43
#define PACKET_SCAN_COMPLETE 0x47
#define MAX_TOUCHES 10

struct cy8ctma395_data {
    struct serdev_device *serdev;
    struct input_dev *input;
    struct i2c_client *i2c_client;  // For initialization

    u8 rx_buf[1540];
    int rx_pos;

    struct touchpoint touches[MAX_TOUCHES];
    // Add state machine for packet parsing
};

static int cy8ctma395_receive_buf(struct serdev_device *serdev,
                                  const unsigned char *buf,
                                  size_t count)
{
    struct cy8ctma395_data *ts = serdev_device_get_drvdata(serdev);

    // Packet parsing state machine
    // Look for 0xFF 0x43 / 0xFF 0x47
    // Extract matrix data
    // Detect touch blobs
    // Report to input subsystem

    return count;
}

static const struct serdev_device_ops cy8ctma395_serdev_ops = {
    .receive_buf = cy8ctma395_receive_buf,
};

static int cy8ctma395_probe(struct serdev_device *serdev)
{
    // Allocate data structure
    // Set up serdev at 4 Mbps
    // Find I²C device for init
    // Allocate input device
    // Configure MT protocol B
    // Send I²C init sequence
    // Start UART reception
}

// Device tree binding, module init, etc.
```

**Device Tree Binding:**
```yaml
# Documentation/devicetree/bindings/input/touchscreen/cypress,cy8ctma395-uart.yaml

title: Cypress CY8CTMA395 UART Touchscreen

properties:
  compatible:
    const: cypress,cy8ctma395-uart

  config-i2c:
    description: Phandle to I2C device for configuration
    $ref: /schemas/types.yaml#/definitions/phandle
```

#### Phase 5: Testing and Refinement (1-2 weeks)

1. **Hardware testing:**
   - Multi-touch accuracy
   - Touch tracking reliability
   - Edge detection
   - Palm rejection

2. **Performance optimization:**
   - Latency profiling
   - DMA for UART RX
   - Threshold tuning

3. **Integration testing:**
   - Wayland/X11 compatibility
   - Suspend/resume
   - Power management

#### Phase 6: Mainline Submission (2-4 weeks)

1. **Documentation:**
   - Protocol specification document
   - Device tree binding YAML
   - Kconfig and Makefile entries

2. **Code cleanup:**
   - Kernel coding style
   - Proper error handling
   - Remove debug code

3. **Submission:**
   - linux-input mailing list
   - linux-serial mailing list (for serdev aspects)
   - Respond to review comments
   - Iterate until accepted

### Estimated Timeline

- **Userspace prototype:** 1-2 days
- **Working ts-srv port:** 1 week
- **Kernel driver:** 2-3 weeks
- **Testing:** 1-2 weeks
- **Mainline process:** 2-4 weeks

**Total:** 6-12 weeks for complete mainline integration

### Key Realizations

1. **The I²C driver approach was fundamentally wrong** - all previous analysis about extending cyttsp is invalid
2. **UART is the primary communication channel** for touch data
3. **I²C is only for configuration** - firmware, calibration, power management
4. **PostmarketOS already solved this** - we should study their implementation
5. **Both I²C and UART can coexist** on GSBI10 in `GSBI_PROT_I2C_UART` mode

## UART Protocol Reverse Engineering

### Source Code Analysis

**Source:** [Evervolv android_device_hp_tenderloin-common](https://github.com/Evervolv/android_device_hp_tenderloin-common/tree/ng-7.0/touchscreen_drv)

The Android `ts-srv` driver provides complete UART protocol details:

### UART Configuration

**Device Path:**
```c
/dev/ctp_uart  // Custom UART device (likely mapped to GSBI10/ttyMSM2)
```

**Baud Rate:**
```c
uart_mode.speed = 0x3D0900;  // ~4,000,000 bps (4 Mbps!)
ioctl(uart_fd, HSUART_IOCTL_SET_UARTMODE, &uart_mode);
```

**High-speed UART** at 4 Mbps for sufficient bandwidth to stream multi-touch data from 30×40 sensor matrix.

**UART Initialization:**
```c
open("/dev/ctp_uart", O_RDONLY|O_NONBLOCK);
ioctl(uart_fd, HSUART_IOCTL_GET_UARTMODE, &uart_mode);
ioctl(uart_fd, HSUART_IOCTL_SET_UARTMODE, &uart_mode);
ioctl(uart_fd, HSUART_IOCTL_FLUSH, 0x9);  // Flush both RX and TX
```

### Packet Protocol

**Two Packet Types:**

#### 1. Scan Line Data Packet (0xFF 0x43)

**Size:** 44 bytes fixed

**Format:**
```
Offset  Size  Description
------  ----  -----------
0       1     Magic byte: 0xFF
1       1     Packet type: 0x43 (scan line data)
2-43    42    Digitizer matrix data
```

**Purpose:** Contains sensor readings from the 30×40 capacitive touch matrix.

#### 2. Scan Complete Packet (0xFF 0x47)

**Size:** Variable (`cline[2] + 4` bytes)

**Format:**
```
Offset  Size       Description
------  ----       -----------
0       1          Magic byte: 0xFF
1       1          Packet type: 0x47 (scan complete)
2       1          Data length (N bytes)
3-(2+N) N          Additional data
```

**Purpose:** Signals end of digitizer matrix scan frame.

**Packet Validation:**
```c
#define RECV_BUF_SIZE 1540

// Check for start packet
if (cline[0] == 0xff && cline[1] == 0x43 && cidx == 44)
    // Process scan line data

// Check for complete packet
if (cline[0] == 0xff && cline[1] == 0x47 && cidx == (cline[2]+4))
    // Process scan complete
```

### Touch Data Structure

**Digitizer Matrix:**
- **Resolution:** 30 columns × 40 rows = 1200 sensor points
- **Scan Rate:** Multiple packets per frame
- **Raw Data:** Capacitive sensor values (pressure/proximity)

**Touch Point Structure:**
```c
#define MAX_TOUCH 10  // Maximum 10 simultaneous touches

struct touchpoint {
    int pw;                    // Power/weight (pressure value)
    float i, j;                // Matrix coordinates (40×30 grid)
    int tracking_id;           // Unique touch identifier
    int prev_loc;              // Previous frame index
    float direction;           // Movement angle (radians)
    int distance;              // Movement distance (pixels)
    int touch_major;           // Contact area size
    int x, y;                  // Screen coordinates (filtered)
    int unfiltered_x, unfiltered_y;  // Raw screen coords
    int highest_val;           // Peak pressure in touch
    int touch_delay;           // Threshold delay counter
    int hover_x, hover_y;      // Debounce position
    int hover_delay;           // Debounce counter
};
```

**Triple Buffering:**
```c
struct touchpoint tp[3][MAX_TOUCH];  // Current, previous, previous-previous
```

### Coordinate Transformation

**Matrix to Screen Conversion:**
```c
#define X_AXIS 30
#define Y_AXIS 40
#define X_RESOLUTION 1024
#define Y_RESOLUTION 768

#define X_LOCATION_VALUE ((float)X_RESOLUTION / (float)(Y_AXIS-1))
#define Y_LOCATION_VALUE ((float)Y_RESOLUTION / (float)(X_AXIS-1))

// Conversion (note: axes are swapped and inverted)
tp[tpoint][tpc].x = X_RESOLUTION_MINUS1 - tp[tpoint][tpc].j * X_LOCATION_VALUE;
tp[tpoint][tpc].y = Y_RESOLUTION_MINUS1 - tp[tpoint][tpc].i * Y_LOCATION_VALUE;
```

**Coordinate Axes Mapping:**
- Digitizer **J-axis (0-29)** → Screen **X-axis (1024-0)** (inverted)
- Digitizer **I-axis (0-39)** → Screen **Y-axis (768-0)** (inverted)

### Touch Detection Algorithm

**Pressure Thresholds:**
```c
// Finger mode thresholds
#define INITIAL_REPORT 32       // Initial touch detection
#define CONTINUE_REPORT 26      // Continue tracking existing touch
#define DELAYED_THRESHOLD 28    // Delayed touch confirmation
#define LARGE_UNPRESS 22        // Large area release threshold

// Stylus mode (different thresholds)
#define STYLUS_INITIAL_REPORT 10
#define STYLUS_CONTINUE_REPORT 8
```

**Processing Steps:**

1. **Weighted Centroid Calculation:**
   ```c
   // Use 1.5-power scaling of pressure values
   weight = pow(pressure, 1.5);
   weighted_sum_x += x * weight;
   weighted_sum_y += y * weight;
   total_weight += weight;

   centroid_x = weighted_sum_x / total_weight;
   centroid_y = weighted_sum_y / total_weight;
   ```

2. **Touch Tracking (Nearest Neighbor):**
   ```c
   // Calculate Euclidean distance to previous touches
   distance = sqrt(pow(new_x - prev_x, 2) + pow(new_y - prev_y, 2));

   // Assign tracking ID from closest previous touch
   if (distance < min_distance)
       tracking_id = previous_tracking_id;
   ```

3. **Jump Filtering:**
   ```c
   #define MAX_JUMP_DISTANCE 130  // pixels

   // Reject impossible jumps unless motion is consistent
   if (distance > MAX_JUMP_DISTANCE && !consistent_direction)
       // Treat as lift-off, assign new tracking ID
   ```

4. **Debounce/Hover Filtering:**
   ```c
   // Track position consistency over multiple frames
   if (abs(x - hover_x) < threshold && abs(y - hover_y) < threshold)
       hover_delay++;
   else
       hover_delay = 0;

   // Only report after stable position
   if (hover_delay >= DEBOUNCE_FRAMES)
       report_touch();
   ```

### I²C Initialization Sequence

**Power On Sequence** (from `digitizer.c`):

```c
// 1. Reset and power on
write("/sys/devices/platform/cy8ctma395/xres", "1");  // Assert reset
write("/sys/devices/platform/cy8ctma395/vdd", "1");   // Power on
usleep(50000);  // 50ms delay

// 2. Wake sequence
write("/sys/devices/platform/cy8ctma395/wake", "1");  // Assert wake
usleep(50000);
write("/sys/devices/platform/cy8ctma395/xres", "0");  // Release reset
usleep(50000);
write("/sys/devices/platform/cy8ctma395/wake", "0");  // Release wake
usleep(50000);

// 3. I²C configuration commands (address 0x67)
i2c_write(0x67, {0x08, 0x00});                              // Command 1
i2c_write(0x67, {0x31, 0x01, 0x08, 0x0C, 0x0D, 0x0A});     // Command 2 (6 bytes)
i2c_write(0x67, {0x30, 0x0F});                              // Command 3
i2c_write(0x67, {0x40, 0x02});                              // Command 4
i2c_write(0x67, {0x41, 0x10});                              // Command 5
i2c_write(0x67, {0x0A, 0x04});                              // Command 6
i2c_write(0x67, {0x08, 0x03});                              // Command 7 (start UART stream)
```

**Command Analysis:**
- **0x08, 0x00**: Enter configuration mode
- **0x31, 0x01, 0x08, 0x0C, 0x0D, 0x0A**: Multi-byte configuration (scan settings?)
- **0x30, 0x0F**: Configuration parameter
- **0x40, 0x02**: Configuration parameter
- **0x41, 0x10**: Configuration parameter
- **0x0A, 0x04**: Configuration parameter
- **0x08, 0x03**: Exit config mode / start UART streaming

**Power Off Sequence:**
```c
write("/sys/devices/platform/cy8ctma395/vdd", "0");   // Power off
write("/sys/devices/platform/cy8ctma395/xres", "1");  // Assert reset
usleep(10000);
write("/sys/devices/platform/cy8ctma395/xres", "0");  // Release reset
usleep(80000);
```

### Multi-Touch Protocol

**Linux Input Events (Protocol B):**
```c
// For each active touch slot
input_event(EV_ABS, ABS_MT_SLOT, slot_id);
input_event(EV_ABS, ABS_MT_TRACKING_ID, tracking_id);
input_event(EV_ABS, ABS_MT_POSITION_X, x);
input_event(EV_ABS, ABS_MT_POSITION_Y, y);
input_event(EV_ABS, ABS_MT_TOUCH_MAJOR, touch_major);
input_event(EV_ABS, ABS_MT_PRESSURE, pressure);

// For lift-off
input_event(EV_ABS, ABS_MT_SLOT, slot_id);
input_event(EV_ABS, ABS_MT_TRACKING_ID, -1);  // Release

input_event(EV_SYN, SYN_REPORT, 0);  // Frame sync
```

### Complete Protocol Summary

**Initialization:**
1. Power on via sysfs (VDD, reset sequence)
2. Send 7 I²C configuration commands to 0x67
3. Controller starts streaming UART data at 4 Mbps

**Runtime Operation:**
1. Read UART packets from `/dev/ctp_uart`
2. Receive 0xFF 0x43 packets (44 bytes, digitizer matrix data)
3. Receive 0xFF 0x47 packet (variable, scan complete marker)
4. Parse 30×40 capacitive matrix values
5. Detect touch blobs using pressure thresholds
6. Calculate weighted centroids for each touch
7. Track touches across frames (nearest neighbor matching)
8. Filter jumps and apply debouncing
9. Transform matrix coords (i,j) to screen coords (x,y)
10. Report via Linux input subsystem (Protocol B)

**Data Flow:**
```
CY8CTMA395 (30×40 matrix scan)
  ↓ Internal I²C bus
5× CY8CTMA375 slaves (sensor readings)
  ↓ Aggregation in master firmware
UART @ 4 Mbps (0xFF 0x43/0x47 packets)
  ↓ /dev/ctp_uart
ts-srv userspace driver
  ↓ Touch blob detection & tracking
Linux input event device
  ↓ libinput / Wayland / X11
Applications
```

## UART Hardware Requirements for Mainline Linux

### Summary

The HP TouchPad touchscreen UART operates at 4 Mbps over GSBI10. To achieve this
bandwidth reliably, the mainline kernel requires:

1. **GSBI10 configured in I2C+UART mode** (`GSBI_PROT_I2C_UART` = 0x6)
2. **ADM DMA controller** for high-speed UART data transfer
3. **msm_serial driver** with DMA support enabled

### Legacy webOS Kernel Configuration

From `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/boot/config-2.6.35-palm-tenderloin`:

```
CONFIG_HSUART=y                # Palm's High-Speed UART driver
CONFIG_MSM_UARTDM=y           # MSM UART Data Mover (DMA)
CONFIG_SERIAL_MSM_HS=y        # MSM High-Speed serial
CONFIG_MSM_ADM3=y             # ADM3 DMA controller
```

### GSBI10 UART DMA Channel Assignment

Based on analysis of Palm's legacy kernel (`kernel-3.0.5.txt` and `webos-linux-kernel-opal`):

**ADM DMA Channel Definitions for GSBI10 (HSUART2):**

| Parameter | Value | Notes |
|-----------|-------|-------|
| DMA Controller | ADM0 | Application DMA controller 0 |
| TX Channel | 8 | `DMOV_HSUART2_TX_CHAN` |
| RX Channel | 8 | `DMOV_HSUART2_RX_CHAN` (bidirectional) |
| TX CRCI (base) | 9 | `ADM3_0_B_GSBI10_OUT_CRCI` |
| RX CRCI (base) | 10 | `ADM3_0_B_GSBI10_IN_CRCI` |
| TX CRCI (full) | 25 | `(1 << 4) + 9` = MUX_SEL + base CRCI |
| RX CRCI (full) | 26 | `(1 << 4) + 10` = MUX_SEL + base CRCI |

**CRCI (Client Request Controller Interface)** values control flow control between
the UART FIFO and DMA controller.

**Source:** `arch/arm/mach-msm/include/mach/dma.h` from webos-linux-kernel-opal:
```c
#define ADM3_0_B_GSBI10_OUT_CRCI    9
#define ADM3_0_B_GSBI10_IN_CRCI     10

#define DMOV_HSUART2_TX_CHAN   8
#define DMOV_HSUART2_TX_CRCI   ((1 << 4) + ADM3_0_B_GSBI10_OUT_CRCI)  // = 25

#define DMOV_HSUART2_RX_CHAN   8
#define DMOV_HSUART2_RX_CRCI   ((1 << 4) + ADM3_0_B_GSBI10_IN_CRCI)   // = 26
```

### UART Device Resources (Legacy Platform)

From `kernel-3.0.5.txt` line 345897-345945, the legacy `msm_device_uart_dm2` definition:

```c
static struct resource msm_uart_dm2_resources[] = {
    {
        .start = MSM_GSBI10_UART_DM_PHYS,     // 0x19A40000
        .end   = MSM_GSBI10_UART_DM_PHYS + PAGE_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = GSBI10_UARTDM_IRQ,           // GIC_SPI_START + 191
        .end   = GSBI10_UARTDM_IRQ,
        .flags = IORESOURCE_IRQ,
    },
    {
        .start = MSM_GSBI10_PHYS,             // 0x19A00000 (GSBI control)
        .end   = MSM_GSBI10_PHYS + 4 - 1,
        .name  = "gsbi_resource",
        .flags = IORESOURCE_MEM,
    },
    {
        .start = TCSR_BASE_PHYS,              // Top CSR for mux control
        .end   = TCSR_BASE_PHYS + 0x80 - 1,
        .name  = "tcsr_resource",
        .flags = IORESOURCE_MEM,
    },
    {
        .start = DMOV_HSUART2_TX_CHAN,        // 8 (TX DMA channel)
        .end   = DMOV_HSUART2_RX_CHAN,        // 8 (RX DMA channel)
        .name  = "uartdm_channels",
        .flags = IORESOURCE_DMA,
    },
    {
        .start = DMOV_HSUART2_TX_CRCI,        // 25 (TX flow control)
        .end   = DMOV_HSUART2_RX_CRCI,        // 26 (RX flow control)
        .name  = "uartdm_crci",
        .flags = IORESOURCE_DMA,
    },
};
```

### Mainline Kernel Support

The mainline kernel already has the required components:

#### 1. ADM DMA Controller Driver

**File:** `drivers/dma/qcom/qcom_adm.c`

**Status:** ✅ Available and functional

**Features:**
- DMA channel management
- CRCI (flow control) support
- MUX_SEL bit handling for GSBI multiplexing

**DT Binding:** `Documentation/devicetree/bindings/dma/qcom,adm.yaml`

#### 2. MSM Serial Driver with DMA

**File:** `drivers/tty/serial/msm_serial.c`

**Status:** ✅ Available with DMA support

**Key Features:**
```c
#include <linux/dma/qcom_adm.h>    // ADM-specific configuration

static void msm_request_rx_dma(struct msm_port *msm_port, resource_size_t base)
{
    // Read CRCI from device tree
    of_property_read_u32(dev->of_node, "qcom,rx-crci", &crci);

    // Configure DMA with CRCI for flow control
    if (crci) {
        conf.peripheral_config = &periph_conf;
        conf.peripheral_size = sizeof(periph_conf);
        periph_conf.crci = crci;
    }
}
```

**DT Properties:**
- `dmas` - DMA channel phandle and specifier
- `dma-names` - "rx" and/or "tx"
- `qcom,rx-crci` - RX flow control identifier
- `qcom,tx-crci` - TX flow control identifier

### Current Device Tree Configuration

**File:** `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`

The current configuration includes:

```dts
/* ADM0 DMA Controller */
adm_dma0: dma-controller@18320000 {
    compatible = "qcom,adm";
    reg = <0x18320000 0x100000>;
    interrupts = <GIC_SPI 171 IRQ_TYPE_LEVEL_HIGH>;
    #dma-cells = <1>;

    clocks = <&gcc ADM0_CLK>, <&gcc ADM0_PBUS_CLK>;
    clock-names = "core", "iface";

    resets = <&gcc ADM0_RESET>,
             <&gcc ADM0_PBUS_RESET>,
             <&gcc ADM0_C0_RESET>,
             <&gcc ADM0_C1_RESET>,
             <&gcc ADM0_C2_RESET>;
    reset-names = "clk", "pbus", "c0", "c1", "c2";
    qcom,ee = <1>;
};

/* GSBI10 Serial Port (Touchscreen UART) */
gsbi10_serial: serial@19a40000 {
    compatible = "qcom,msm-uartdm-v1.3", "qcom,msm-uartdm";
    reg = <0x19a40000 0x1000>,
          <0x19a00000 0x1000>;
    interrupts = <GIC_SPI 191 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&gcc GSBI10_UART_CLK>, <&gcc GSBI10_H_CLK>;
    clock-names = "core", "iface";

    /* DMA Configuration */
    dmas = <&adm_dma0 8>;      /* Channel 8 for RX */
    dma-names = "rx";
    qcom,rx-crci = <10>;       /* Base CRCI value */

    status = "disabled";
};
```

### Required Changes for Touchscreen UART

#### Option A: Verify Current Configuration Works

The current DT configuration may already work. Test steps:

1. Enable GSBI10 in I2C+UART mode
2. Enable gsbi10_serial
3. Test DMA operation at high baud rates

#### Option B: Add TX DMA Support (If Needed)

If bidirectional DMA is required, add TX DMA configuration:

```dts
gsbi10_serial: serial@19a40000 {
    /* ... existing config ... */

    /* Full DMA Configuration */
    dmas = <&adm_dma0 8>, <&adm_dma0 8>;  /* Both RX and TX on channel 8 */
    dma-names = "rx", "tx";
    qcom,rx-crci = <10>;   /* or <26> if full value needed */
    qcom,tx-crci = <9>;    /* or <25> if full value needed */
};
```

#### Option C: CRCI MUX_SEL Handling

If the MUX_SEL bit (BIT(4) = 0x10) is required, two approaches:

**Approach 1:** Use full CRCI value (25/26) which includes MUX_SEL:
```dts
qcom,rx-crci = <26>;   /* (1 << 4) + 10 */
qcom,tx-crci = <25>;   /* (1 << 4) + 9 */
```

**Approach 2:** The driver should extract and handle MUX_SEL automatically.

Looking at `qcom_adm.c`:
```c
crci = achan->crci & 0xf;  // Extract base CRCI (0-15)
// MUX_SEL is encoded in bit 4 of achan->crci
```

The mainline driver handles this correctly - the base CRCI value (10) is used
for the actual CRCI selection, and the MUX_SEL is handled separately.

### HSUART vs Standard msm_serial

**Legacy HSUART Driver** (`drivers/misc/hsuart.c`):
- Palm's proprietary High-Speed UART driver
- Creates `/dev/ctp_uart` device
- Custom IOCTLs for mode configuration
- Tight integration with ADM DMA

**Mainline msm_serial Driver** (`drivers/tty/serial/msm_serial.c`):
- Standard Linux serial driver
- Creates `/dev/ttyMSMx` devices
- Uses dmaengine API (compatible with ADM)
- Device tree configured

**Migration Path:**
- Userspace code must use `/dev/ttyMSM2` instead of `/dev/ctp_uart`
- Standard termios/ioctl for configuration
- Custom baud rate (4 Mbps) may need `BOTHER` termios flag

### Testing DMA Configuration

**Step 1: Verify ADM DMA Controller:**
```bash
# Check ADM is probed
dmesg | grep adm

# Check DMA channels
ls /sys/class/dma/
```

**Step 2: Verify UART DMA:**
```bash
# Check UART driver loaded
dmesg | grep -i "tty.*MSM\|uartdm"

# Look for DMA configuration messages
dmesg | grep -i "dma.*uart\|uart.*dma"
```

**Step 3: Test High-Speed UART:**
```bash
# Configure serial port (may need custom tool for 4 Mbps)
stty -F /dev/ttyMSM2 raw

# Read data
cat /dev/ttyMSM2 | hexdump -C
```

### Potential Issues

1. **4 Mbps Baud Rate:**
   - Standard Linux termios may not support exactly 4 Mbps
   - May need `BOTHER` flag or custom ioctl
   - Verify UART clock source can achieve this rate

2. **DMA Buffer Sizing:**
   - 1540 bytes per frame (from ts-srv)
   - Ensure DMA buffer is sufficiently large

3. **CRCI MUX_SEL:**
   - If UART doesn't work, try CRCI values 25/26 instead of 9/10
   - Check driver handles MUX_SEL bit correctly

4. **GSBI Mode Switching:**
   - I2C and UART share GSBI10 pins
   - Ensure mode is set to `GSBI_PROT_I2C_UART` (0x6) not just I2C

### Summary

The mainline kernel has all required components for high-speed UART with DMA:

| Component | Mainline Status | Notes |
|-----------|-----------------|-------|
| ADM DMA Controller | ✅ Available | `qcom_adm.c` |
| MSM Serial Driver | ✅ Available | `msm_serial.c` with DMA |
| DT Bindings | ✅ Configured | DMA and CRCI properties |
| GSBI10 UART | ✅ Defined | In tenderloin DTS |
| High-Speed Support | ⚠️ Untested | May need BOTHER for 4 Mbps |

The primary work remaining is:
1. Test UART operation at 4 Mbps
2. Verify DMA transfers work correctly
3. Implement/port touchscreen userspace driver or kernel serdev driver

---

## Implementation Progress (January 2026)

### Summary of Fixes Applied

#### 1. GPIO 71 Pinctrl Fix (CRITICAL)

**Problem:** GPIO 71 (UART RX for GSBI10) was UNCLAIMED in the pinmux, preventing UART data reception.

**Root Cause:** The serial device (`19a40000.serial`) had no pinctrl configuration. The msm_serial driver doesn't explicitly request pinctrl - it must be specified in the device tree.

**Solution:** Added explicit pinctrl configuration for GPIO 71:

```dts
/* In pinctrl@800000 section */
gsbi10_uart_pins: uart10-state {
    rx-pins {
        pins = "gpio71";
        function = "gsbi10";
        drive-strength = <2>;
        bias-pull-up;  /* RX line idle high */
    };
};

/* Reference from serial node */
&gsbi10_serial {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&gsbi10_uart_pins>;
    /* ... rest of configuration ... */
};
```

**Verification:**
```bash
# Before fix (GPIO 71 UNCLAIMED):
cat /sys/kernel/debug/pinctrl/800000.pinctrl/pinmux-pins | grep gpio71
# (no output)

# After fix:
cat /sys/kernel/debug/pinctrl/800000.pinctrl/pinmux-pins | grep gpio71
pin 71 (GPIO_71): device 19a40000.serial function gsbi10 group gpio71
```

#### 2. ADM DMA #dma-cells Fix

**Problem:** Previous commit incorrectly changed `adm_dma1` from `#dma-cells = <1>` to `#dma-cells = <2>`.

**Solution:** Reverted back to correct value:
```dts
adm_dma1: dma-controller@18420000 {
    #dma-cells = <1>;  /* Correct - ADM uses single cell for channel */
    /* ... */
};
```

#### 3. GPIO Pin Assignment Clarification

**GSBI10 I2C_UART Mode Pin Usage:**
| GPIO | Function | Usage |
|------|----------|-------|
| GPIO 70 | Reset | Touchscreen XRES (active low) |
| GPIO 71 | UART RX | Touch data from CY8CTMA395 to host |
| GPIO 72 | I2C SDA | Configuration commands to touchscreen |
| GPIO 73 | I2C SCL | Configuration commands to touchscreen |

**Important:** GPIO 70 is used as the touchscreen reset GPIO, so it cannot be used for UART TX. The touchscreen only sends data to the host (RX only), so this is acceptable.

### Current Status

#### UART Data Reception: WORKING

After applying the GPIO 71 pinctrl fix, UART data reception is now functional:

```
[   83.436099] cy8ctma395-ts serial1-0: UART RX: 1 bytes total, last 1 bytes
[   83.453532] cy8ctma395 first RX: 00
[  254.904161] cy8ctma395-ts serial1-0: UART RX: 193 bytes total, last 192 bytes
[  259.909943] cy8ctma395-ts serial1-0: UART RX: 554316 bytes total, last 192 bytes
[  335.268576] cy8ctma395-ts serial1-0: UART RX: 8251711 bytes total, last 192 bytes
[  395.568733] cy8ctma395-ts serial1-0: UART RX: 13100174 bytes total, last 408 bytes
```

**Key Metrics:**
- Baud rate: 4,000,000 bps (4 Mbps) - ACHIEVED
- Data throughput: ~500 KB every 5 seconds (~100 KB/s average)
- Total received: 13+ million bytes

#### Touch Event Parsing: IN PROGRESS

The kernel driver (`cy8ctma395_ts.c`) is receiving UART data but may not be correctly parsing touch frames.

**Frame Format Expected:**
- Frame start: `0xFF`
- Row data: `0x43` (followed by row index and sensor data)
- Scan complete: `0x47` (followed by touch count)

**Current Issue:** First byte received is `0x00` instead of `0xFF`. This could indicate:
1. Sync issue at startup
2. Different frame format than expected
3. Need to wait for valid frame start

**Debug Output Added:** The driver now prints:
- First 64 bytes received (hex dump)
- Sample data every 10 seconds
- Valid frame count and type when parsed

### Device Tree Configuration (Current)

```dts
&gsbi10 {
    status = "okay";
    qcom,mode = <GSBI_PROT_I2C_UART>;  /* Combined I2C + UART */
};

&gsbi10_i2c {
    status = "okay";
    clock-frequency = <100000>;
    /* I2C used for configuration commands */
};

&gsbi10_serial {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&gsbi10_uart_pins>;  /* GPIO 71 for UART RX */

    touchscreen {
        compatible = "cypress,cy8ctma395-ts";
        i2c-bus = <&gsbi10_i2c>;
        vdd-supply = <&pm8058_l15>;
        reset-gpios = <&tlmm 70 GPIO_ACTIVE_LOW>;
        wake-gpios = <&tlmm 123 GPIO_ACTIVE_HIGH>;
        /* ... */
    };
};
```

### Pinctrl Configuration (Current)

```dts
/* GSBI10 I2C pins (SDA/SCL) */
gsbi10_i2c_pins: gsbi10-i2c-state {
    sda-scl-pins {
        pins = "gpio72", "gpio73";
        function = "gsbi10";
        drive-strength = <16>;
        bias-disable;
    };
};

/* GSBI10 UART RX pin */
gsbi10_uart_pins: uart10-state {
    rx-pins {
        pins = "gpio71";
        function = "gsbi10";
        drive-strength = <2>;
        bias-pull-up;
    };
};
```

### Final Solution: Touch Calculation Trigger Fix

#### The Problem

The driver was waiting for `FRAME_SCAN_COMPLETE (0x47)` frames to trigger touch calculation:

```c
if (ts->cline[1] == FRAME_SCAN_COMPLETE) {
    ret = cy8ctma395_ts_calc_point(ts);  // Never called!
}
```

However, the CY8CTMA395 touchscreen **never sends 0x47 frames**. It only sends `FRAME_ROW_DATA (0x43)` frames.

#### Frame Format Discovery

Analysis of hex dumps revealed the actual frame format:

```
ff 43 80 01 01 01 01 01 01 01 01 01 01 01 01 01  .C..............
01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01  ................
01 01 01 01 01 01 01 01 01 01 01 ff 43 01 ...   ............C...
```

- `0xFF` = Frame start marker
- `0x43` = Frame type (row data)
- `0x80` = Row index with **bit 7 set** indicating **new scan start**
- Following 40 bytes = capacitance sensor data for that row

**Key Discovery:** Bit 7 (0x80) of the row index indicates the start of a new scan cycle. This is the trigger point for touch calculation, NOT a separate 0x47 frame.

#### The Fix

Modified `cy8ctma395_ts_consume_frame()` to trigger touch calculation when a new scan starts:

```c
if (ts->cline[1] == FRAME_ROW_DATA) {
    int row = ts->cline[2] & 0x1F;

    /* Start of new scan - calculate touches from previous scan, then clear */
    if (ts->cline[2] & 0x80) {
        /* Calculate touches from the completed scan before clearing */
        if (ts->rows_received > 0) {
            ret = cy8ctma395_ts_calc_point(ts);
        }
        memset(ts->matrix, 0, sizeof(ts->matrix));
        ts->rows_received = 0;
    }

    /* Copy row data into matrix */
    if (row < X_AXIS_POINTS) {
        for (i = 0; i < Y_AXIS_POINTS; i++)
            ts->matrix[row][i] = ts->cline[i + 3];
        ts->rows_received++;
    }
}
```

### TOUCHSCREEN NOW WORKING

After applying all fixes, the touchscreen is fully functional:

```
[  186.216541] cy8ctma395: reporting 1 touch(es)
[  186.228807] cy8ctma395: touch detected at (551,609) val=77 weight=2899
[  186.302189] cy8ctma395: calc_point called, rows=30, max_val=77, thresh=26
```

**Verified functionality:**
- Touch detection at correct coordinates
- Touch values (77-78) properly exceed threshold (26)
- Input events sent to `/dev/input/event3`
- Multi-row scanning (30 rows per scan cycle)

### Summary of All Fixes

| Issue | Root Cause | Solution |
|-------|------------|----------|
| No UART data | GPIO 71 not configured for gsbi10 | Added `gsbi10_uart_pins` pinctrl |
| DMA issues | `#dma-cells` incorrectly set to 2 | Reverted to `#dma-cells = <1>` |
| No touch events | Waiting for 0x47 frames that never come | Trigger on new scan (bit 7 of row index) |

### Commits

| Commit | Description |
|--------|-------------|
| `e097635debb2` | GPIO 71 pinctrl for UART RX, ADM DMA #dma-cells fix |
| `b9d0390b53fe` | Touch calculation trigger on new scan start |

### Files Modified

| File | Changes |
|------|---------|
| `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` | Added gsbi10_uart_pins, fixed adm_dma1 #dma-cells |
| `drivers/input/touchscreen/cy8ctma395_ts.c` | Fixed touch calculation trigger, added rows_received tracking |

### Future Improvements

1. **Remove debug output** - The pr_info statements can be removed or converted to pr_debug
2. **Test multi-touch** - Verify multiple simultaneous touches work correctly
3. **Performance tuning** - Profile and optimize if needed
4. **Upstream preparation** - Clean up code for potential mainline submission
