# HP TouchPad Hardware Revisions (Topaz/Opal)

This document describes the hardware differences between HP TouchPad board revisions
as discovered from the legacy webOS kernel source code.

## Board Revision Timeline

### WiFi Model (Topaz WiFi)
| Revision | Bootloader String | Notes |
|----------|-------------------|-------|
| EVT1 | topaz-Wifi-evt1, topaz-3rdbuild-Wifi | Early engineering samples |
| EVT2 | topaz-Wifi-evt2, topaz-4thbuild-Wifi | Engineering samples |
| EVT3 | topaz-Wifi-evt3, topaz-5thbuild-Wifi | Late engineering samples |
| DVT | topaz-Wifi-dvt, topaz-6thbuild-Wifi | Design Verification Test |
| PVT | topaz-Wifi-pvt, topaz-7thbuild-Wifi | Production units |

### 3G Model (Topaz 3G)
| Revision | Bootloader String | Notes |
|----------|-------------------|-------|
| EVT1 | topaz-3G-evt1, topaz-3rdbuild-3G | Early engineering samples |
| EVT2 | topaz-3G-evt2, topaz-4thbuild-3G | Engineering samples |
| EVT3 | topaz-3G-evt3, topaz-5thbuild-3G | Late engineering samples |
| EVT4 | topaz-3G-evt4 | Transitional (uses DVT A6 GPIOs) |
| DVT | topaz-3G-dvt, topaz-6thbuild-3G | Design Verification Test |
| PVT | topaz-3G-pvt, topaz-7thbuild-3G | Production units |

## GPIO Assignment Differences

### A6 Battery Controller GPIO Changes

The A6 battery controllers (TI MSP430 microcontrollers) use different GPIO pins
for interrupt signals between pre-DVT and DVT/PVT hardware.

#### WiFi Model

| Signal | EVT1-EVT3 (Pre-DVT) | DVT/PVT |
|--------|---------------------|---------|
| A6_0 TCK | GPIO 157 | GPIO 157 (unchanged) |
| A6_0 WAKEUP | GPIO 155 | GPIO 155 (unchanged) |
| A6_0 TDIO | GPIO 158 | GPIO 158 (unchanged) |
| **A6_0 IRQ** | **GPIO 156** | **GPIO 37** |
| A6_1 TCK | GPIO 115 | GPIO 115 (unchanged) |
| A6_1 WAKEUP | GPIO 141 | GPIO 141 (unchanged) |
| A6_1 TDIO | GPIO 116 | GPIO 116 (unchanged) |
| **A6_1 IRQ** | **GPIO 132** | **GPIO 94** |

#### 3G Model

| Signal | EVT1-EVT3 | EVT4/DVT/PVT |
|--------|-----------|--------------|
| **A6_0 TCK** | **GPIO 68** | **GPIO 156** |
| A6_0 WAKEUP | GPIO 155 | GPIO 155 (unchanged) |
| A6_0 TDIO | GPIO 170 | GPIO 170 (unchanged) |
| **A6_0 IRQ** | **GPIO 156** | **GPIO 37** |
| A6_1 TCK | GPIO 115 | GPIO 115 (unchanged) |
| A6_1 WAKEUP | GPIO 78 | GPIO 78 (unchanged) |
| A6_1 TDIO | GPIO 116 | GPIO 116 (unchanged) |
| **A6_1 IRQ** | **GPIO 132** | **GPIO 94** |

### Other 3G Model Differences

| Signal | Pre-DVT | DVT/PVT |
|--------|---------|---------|
| GPIO_3G_3V3_EN | GPIO 82 | GPIO 106 |

### Other Hardware Differences (WiFi vs 3G)

These differences are between WiFi and 3G models, not revision-specific:

| Component | WiFi Model | 3G Model |
|-----------|------------|----------|
| Touchscreen IRQ | GPIO 123 | GPIO 45 |
| Bluetooth RST_N | GPIO 138 | GPIO 122 |
| Bluetooth POWER | GPIO 130 | GPIO 110 |
| Bluetooth HOST_WAKE | GPIO 129 | GPIO 50 |
| Volume Up | GPIO 103 | PM8058 GPIO 6 |
| Volume Down | GPIO 104 | PM8058 GPIO 7 |
| WLAN RST_N | GPIO 135 | GPIO 28 |
| HOST_WAKE_WL | GPIO 137 | GPIO 80 |
| Gyro INT | GPIO 125 | GPIO 75 |
| LM8502 INT | GPIO 128 | GPIO 77 |
| MAX8903B USB_CHG_MODE | GPIO 133 | GPIO 134 |
| MAX8903B DC_OK | GPIO 140 | GPIO 86 |

## Device Tree Implications

### Current Status

The mainline device tree (`qcom-apq8060-tenderloin-common.dtsi`) currently uses
**pre-DVT GPIO assignments** for the A6 battery controllers:

```dts
/* A6_0 - uses GPIO 156 for IRQ (pre-DVT) */
a6_0: battery@31 {
    interrupts = <156 IRQ_TYPE_EDGE_FALLING>;
};

/* A6_1 - uses GPIO 132 for IRQ (pre-DVT) */
a6_1: battery@32 {
    interrupts = <132 IRQ_TYPE_EDGE_FALLING>;
};
```

### Required Changes for DVT/PVT Support

For DVT/PVT devices (most consumer units), the A6 IRQ GPIOs need to change:

**WiFi DVT/PVT:**
- A6_0 IRQ: GPIO 156 -> GPIO 37
- A6_1 IRQ: GPIO 132 -> GPIO 94

**3G EVT4/DVT/PVT:**
- A6_0 TCK: GPIO 68 -> GPIO 156
- A6_0 IRQ: GPIO 156 -> GPIO 37
- A6_1 IRQ: GPIO 132 -> GPIO 94

### Recommended Approach

Since PVT (production) units are the most common, the device tree should:

1. Default to DVT/PVT GPIO assignments in the common dtsi
2. Provide device tree overlays for pre-DVT development boards if needed

## Detecting Board Revision

The bootloader passes the board revision via kernel command line:
```
boardtype=topaz-Wifi-evt2
boardtype=topaz-Wifi-dvt
boardtype=topaz-Wifi-pvt
```

This can be read at runtime from `/proc/cmdline` or potentially used with
device tree overlays selected by the bootloader.

## Summary

| Target Device | A6_0 IRQ | A6_1 IRQ | Notes |
|--------------|----------|----------|-------|
| EVT1-EVT3 (Pre-DVT) | GPIO 156 | GPIO 132 | Development boards |
| DVT/PVT | GPIO 37 | GPIO 94 | Production units |

Most end-users have PVT (production) devices, so the device tree should be
updated to use DVT/PVT GPIO assignments by default.
