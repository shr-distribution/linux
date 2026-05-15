# Cavekit: MT9M113 Controls

**Created:** 2026-04-14
**Domain:** V4L2 controls, context switching, auto-exposure, auto-white-balance, flicker detection

## Scope

This cavekit covers all V4L2 control implementation:
- Image orientation controls (flip/mirror)
- Test pattern generation
- Exposure controls (auto and manual)
- Gain controls
- White balance controls
- Color effect controls
- Flicker detection (power line frequency)
- Saturation control
- Context A/B switching via sequencer commands

## Requirements

### R1: Horizontal Flip Control

**Description:** The driver must implement V4L2_CID_HFLIP to mirror the image horizontally.

**Acceptance Criteria:**
- [ ] V4L2_CID_HFLIP control is registered with range 0-1
- [ ] Setting HFLIP=1 sets bit 0 of MCU variable mode_sensor_read_mode_a (0x2717)
- [ ] Setting HFLIP=1 sets bit 0 of MCU variable mode_sensor_read_mode_b (0x272D)
- [ ] Setting HFLIP=0 clears bit 0 in both context read mode variables
- [ ] Control change triggers a refresh command (R11)

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R2: Vertical Flip Control

**Description:** The driver must implement V4L2_CID_VFLIP to flip the image vertically.

**Acceptance Criteria:**
- [ ] V4L2_CID_VFLIP control is registered with range 0-1
- [ ] Setting VFLIP=1 sets bit 1 of MCU variable mode_sensor_read_mode_a (0x2717)
- [ ] Setting VFLIP=1 sets bit 1 of MCU variable mode_sensor_read_mode_b (0x272D)
- [ ] Setting VFLIP=0 clears bit 1 in both context read mode variables
- [ ] Control change triggers a refresh command (R11)

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R3: Test Pattern Control

**Description:** The driver must implement V4L2_CID_TEST_PATTERN to enable hardware test patterns.

**Acceptance Criteria:**
- [ ] V4L2_CID_TEST_PATTERN control is registered as a menu control
- [ ] Menu includes at least: Disabled, Solid Color, Color Bars, Fade to Gray
- [ ] Setting test pattern writes to MCU variable mode_common_mode_settings_test_mode
- [ ] Disabled (0) resumes normal sensor output
- [ ] Test patterns are visible in captured frames when enabled
- [ ] Control change triggers a refresh command (R11)

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R4: Auto Exposure Control

**Description:** The driver must implement V4L2_CID_EXPOSURE_AUTO to enable/disable automatic exposure.

**Acceptance Criteria:**
- [ ] V4L2_CID_EXPOSURE_AUTO control is registered as a menu control
- [ ] Menu includes V4L2_EXPOSURE_AUTO and V4L2_EXPOSURE_MANUAL
- [ ] Setting AUTO enables the sensor's internal AE algorithm
- [ ] Setting MANUAL disables AE and allows manual exposure control (R5)
- [ ] AE state is readable via control get operation

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R5: Manual Exposure Control

**Description:** The driver must implement V4L2_CID_EXPOSURE for manual exposure time control.

**Acceptance Criteria:**
- [ ] V4L2_CID_EXPOSURE control is registered with appropriate min/max range
- [ ] Control is only effective when V4L2_CID_EXPOSURE_AUTO is set to MANUAL
- [ ] Setting exposure writes to coarse_integration_time register (0x3012)
- [ ] Exposure value is in units of line periods
- [ ] Reading exposure returns the current integration time setting

**Dependencies:** R4 (auto exposure must be disabled for manual control)

---

### R6: Analogue Gain Control

**Description:** The driver must implement V4L2_CID_ANALOGUE_GAIN for sensor gain adjustment.

**Acceptance Criteria:**
- [ ] V4L2_CID_ANALOGUE_GAIN control is registered with hardware-supported range
- [ ] Setting gain writes to the sensor core gain registers
- [ ] Gain is applied in conjunction with exposure for brightness control
- [ ] Reading gain returns the current gain setting

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R7: Auto White Balance Control

**Description:** The driver must implement V4L2_CID_AUTO_WHITE_BALANCE to enable/disable AWB.

**Acceptance Criteria:**
- [ ] V4L2_CID_AUTO_WHITE_BALANCE control is registered with range 0-1
- [ ] Setting AWB=1 enables the sensor's internal white balance algorithm
- [ ] Setting AWB=0 disables AWB and uses fixed color gains
- [ ] AWB state is readable via control get operation

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R8: Color Effect Control

**Description:** The driver must implement V4L2_CID_COLORFX for special color effects.

**Acceptance Criteria:**
- [ ] V4L2_CID_COLORFX control is registered as a menu control
- [ ] COLORFX_NONE writes 0x6440 to mode_spec_effects_a (0x2759) and mode_spec_effects_b (0x275B)
- [ ] COLORFX_BW (monochrome) writes 0x6441 to both context effect registers
- [ ] COLORFX_SEPIA writes 0x6442 to both context effect registers
- [ ] COLORFX_NEGATIVE writes 0x6443 to both context effect registers
- [ ] COLORFX_SOLARIZE writes 0x6445 to both context effect registers
- [ ] Control change triggers a refresh command (R11)

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R9: Power Line Frequency Control

**Description:** The driver must implement V4L2_CID_POWER_LINE_FREQUENCY for flicker detection.

**Acceptance Criteria:**
- [ ] V4L2_CID_POWER_LINE_FREQUENCY control is registered as a menu control
- [ ] Menu includes: Disabled, 50Hz, 60Hz, Auto
- [ ] Setting 50Hz writes appropriate value to fd_mode MCU variable (0xA404)
- [ ] Setting 60Hz writes appropriate value to fd_mode MCU variable (0xA404)
- [ ] Flicker detection affects AE to avoid banding artifacts

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R10: Saturation Control

**Description:** The driver must implement V4L2_CID_SATURATION for color saturation adjustment.

**Acceptance Criteria:**
- [ ] V4L2_CID_SATURATION control is registered with range 0-255
- [ ] Default value is 128 (0x80), representing 100% saturation
- [ ] Setting saturation writes to awb_saturation MCU variable (0xA354)
- [ ] Value 0 produces grayscale output, 255 produces maximum saturation
- [ ] Control change triggers a refresh command (R11)

**Dependencies:** cavekit-core R2 (MCU variable access)

---

### R11: Context Switching

**Description:** The driver must implement sequencer commands for switching between Context A and Context B.

**Acceptance Criteria:**
- [ ] Writing seq_cmd=0x0001 to MCU variable 0xA103 enters Preview mode (Context A)
- [ ] Writing seq_cmd=0x0002 to MCU variable 0xA103 enters Capture mode (Context B)
- [ ] Writing seq_cmd=0x0005 to MCU variable 0xA103 issues a Refresh command
- [ ] After any seq_cmd write, driver polls MCU variable 0xA103 until it returns 0
- [ ] Polling timeout is at least 500ms before returning an error
- [ ] Context switch completion is verified before returning from set_fmt
- [ ] All control changes that modify context variables issue a Refresh (0x0005) command

**Dependencies:** cavekit-core R2 (MCU variable access)

## Out of Scope

- Custom vendor-specific controls beyond V4L2 standard controls
- Per-context separate control values (controls apply to both contexts)
- Focus control (MT9M113 has fixed focus)
- Zoom control (digital zoom not implemented)
- Region of interest / metering areas
- Raw control of individual ISP blocks

## Cross-References

- See also: [cavekit-core.md](cavekit-core.md) - All controls use MCU variable access (R2)
- See also: [cavekit-power.md](cavekit-power.md) - Controls require sensor to be powered
