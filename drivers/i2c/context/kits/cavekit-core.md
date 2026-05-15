# Cavekit: MT9M113 Core

**Created:** 2026-04-14
**Domain:** Chip identification, register access, MCU variables, dual subdev architecture, format enumeration

## Scope

This cavekit covers the fundamental driver infrastructure:
- Chip identification and validation
- I2C register read/write operations
- MCU variable access via indirect addressing
- Dual V4L2 subdevice registration (Pixel Array + IFP)
- Media entity creation and linking
- Format enumeration for all supported media bus codes
- Resolution support for both contexts

## Requirements

### R1: Chip Identification

**Description:** The driver must verify the sensor identity by reading the chip ID register and confirming the expected value.

**Acceptance Criteria:**
- [ ] Reading register 0x0000 returns a 16-bit value
- [ ] Chip ID value 0x2480 is accepted as valid MT9M113
- [ ] Any other chip ID value causes probe to fail with an error
- [ ] Chip ID verification occurs after power sequencing completes

**Dependencies:** cavekit-power R2 (clock configuration), R3 (hardware reset)

---

### R2: MCU Variable Access

**Description:** The driver must provide MCU variable read and write operations using the indirect access registers.

**Acceptance Criteria:**
- [ ] Writing an MCU variable writes the address to register 0x098C, then writes the value to register 0x0990
- [ ] Reading an MCU variable writes the address to register 0x098C, then reads the value from register 0x0990
- [ ] MCU variable addresses in range 0x0000-0xFFFF are supported
- [ ] MCU variable values are 16-bit unsigned integers
- [ ] A polling function exists that reads an MCU variable repeatedly until it matches an expected value or times out

**Dependencies:** R1 (chip must be identified first)

---

### R3: Dual Subdevice Registration

**Description:** The driver must register two V4L2 subdevices representing the Pixel Array (PA) and Image Flow Processor (IFP).

**Acceptance Criteria:**
- [ ] A Pixel Array subdevice is registered with exactly one source pad
- [ ] An IFP subdevice is registered with exactly one sink pad and one source pad
- [ ] A media entity link connects PA source pad to IFP sink pad
- [ ] Both subdevices are registered with the V4L2 async framework
- [ ] Removing the driver unregisters both subdevices cleanly
- [ ] Each subdevice has its own control handler

**Dependencies:** None (registration infrastructure)

---

### R4: Format Enumeration

**Description:** The driver must enumerate all supported media bus formats on each subdevice pad.

**Acceptance Criteria:**
- [ ] IFP source pad enumerates UYVY8_1X16 format
- [ ] IFP source pad enumerates YUYV8_1X16 format
- [ ] IFP source pad enumerates RGB565_1X16 format (if hardware supports)
- [ ] PA source pad enumerates SGRBG10_1X10 format (raw Bayer)
- [ ] Format enumeration returns -EINVAL for invalid index values
- [ ] get_fmt returns the currently configured format
- [ ] set_fmt validates and applies format changes

**Dependencies:** R3 (subdevices must be registered)

---

### R5: Resolution Support

**Description:** The driver must support the two hardware contexts with their respective resolutions.

**Acceptance Criteria:**
- [ ] Context A resolution 640x480 is enumerable and selectable
- [ ] Context B resolution 1280x1024 is enumerable and selectable
- [ ] Frame size enumeration returns both supported resolutions
- [ ] Setting an unsupported resolution is rejected or clamped to nearest supported
- [ ] Resolution changes are applied to the correct context registers

**Dependencies:** R2 (MCU variables for context configuration)

---

### R6: Frame Interval Enumeration

**Description:** The driver must report achievable frame intervals based on PLL and timing parameters.

**Acceptance Criteria:**
- [ ] Frame interval enumeration returns at least one valid interval per resolution
- [ ] Frame intervals are calculated from pixel clock, line length, and frame length
- [ ] get_frame_interval returns the current frame interval
- [ ] set_frame_interval adjusts timing parameters if supported
- [ ] Frame rate of 30fps is achievable for Context A (640x480)

**Dependencies:** cavekit-power R4 (PLL configuration affects pixel clock)

## Out of Scope

- V4L2 control implementation (see cavekit-controls.md)
- Power sequencing and PLL configuration (see cavekit-power.md)
- MIPI CSI-2 lane configuration (hardware-specific, out of driver scope)
- Firmware loading (MT9M113 has internal ROM, no external firmware)
- ISP tuning parameters beyond basic format/resolution

## Cross-References

- See also: [cavekit-controls.md](cavekit-controls.md) - Uses MCU variable access from R2
- See also: [cavekit-power.md](cavekit-power.md) - R1 depends on power being enabled first
