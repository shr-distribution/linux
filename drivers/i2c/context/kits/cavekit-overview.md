# Cavekit Overview: MT9M113 Camera Sensor Driver

**Created:** 2026-04-14
**Target:** Linux kernel mainline (drivers/media/i2c)
**Device:** Aptina MT9M113 1.3MP SOC CMOS Image Sensor

## Summary

This cavekit defines requirements for a V4L2 subdevice driver for the MT9M113 sensor. The MT9M113 is a System-on-Chip camera sensor with integrated Image Flow Processor (IFP), supporting dual-context operation for preview (640x480) and capture (1280x1024) modes.

## Kit Index

| Kit | Description | Requirements |
|-----|-------------|--------------|
| [cavekit-core.md](cavekit-core.md) | Chip ID, register access, MCU variables, dual subdev, formats, resolutions | 6 |
| [cavekit-controls.md](cavekit-controls.md) | V4L2 controls, context switching, AE/AWB/FD, color effects | 11 |
| [cavekit-power.md](cavekit-power.md) | Power sequencing, PLL, standby, runtime PM, regulators, GPIOs | 10 |

**Total Requirements:** 27
**Total Acceptance Criteria:** 67

## Dependency Graph

```
                    +---------------+
                    | cavekit-power |
                    +-------+-------+
                            |
            Power-on required before any register access
                            |
                            v
                    +---------------+
                    | cavekit-core  |
                    +-------+-------+
                            |
            MCU variable access functions used by controls
                            |
                            v
                  +-----------------+
                  | cavekit-controls|
                  +-----------------+
```

## Cross-Reference Summary

- **Core -> Power:** Core initialization requires power sequencing completed first
- **Controls -> Core:** Controls use MCU variable read/write functions from Core
- **Controls -> Power:** Context switching may affect power states; refresh commands require stable power

## Hardware Overview

- **Chip ID:** 0x2480 at register 0x0000
- **MCU Access:** Address register 0x098C, Data register 0x0990
- **Dual Context:**
  - Context A: Preview mode, 640x480, binned output
  - Context B: Capture mode, 1280x1024, full resolution
- **Output Formats:** UYVY, YUYV, RGB565, SGRBG10 (raw Bayer)
- **Interface:** MIPI CSI-2 or parallel

## Verification Strategy

All acceptance criteria are designed for automated verification:

1. **Register verification:** Read-back checks after writes
2. **MCU variable verification:** Poll for expected values after commands
3. **Timing verification:** Measure delays against datasheet requirements
4. **State machine verification:** Check seq_cmd returns to 0 after operations
5. **Format enumeration:** Verify reported formats match capability list
