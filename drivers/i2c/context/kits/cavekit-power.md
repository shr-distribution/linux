# Cavekit: MT9M113 Power

**Created:** 2026-04-14
**Domain:** Power sequencing, PLL configuration, standby modes, runtime PM, regulators, GPIOs

## Scope

This cavekit covers all power management aspects:
- Power rail sequencing (VDD, VAA, VDD_IO, VDD_PLL)
- External clock configuration
- Hardware reset timing
- PLL initialization and lock
- Soft standby enter/exit
- Runtime power management
- Double buffer control for safe register updates
- Regulator and GPIO handling from device tree

## Requirements

### R1: Power Rail Sequencing

**Description:** The driver must enable power rails in the correct order per datasheet requirements.

**Acceptance Criteria:**
- [ ] VDD (1.8V digital core) is enabled before other rails
- [ ] VAA (2.8V analog), VDD_IO (I/O), and VDD_PLL are enabled after VDD
- [ ] Power-down sequence reverses the power-up order
- [ ] Minimum stabilization delay between rail enables is respected
- [ ] All regulators are disabled on driver removal or probe failure

**Dependencies:** None (first step in initialization)

---

### R2: Clock Configuration

**Description:** The driver must configure and enable the external clock input.

**Acceptance Criteria:**
- [ ] External clock (EXTCLK) of 24MHz is requested and enabled
- [ ] Clock is enabled before deasserting reset
- [ ] Wait time of at least 100 EXTCLK cycles occurs before first I2C access
- [ ] Clock is disabled during power-down sequence
- [ ] Clock rate is verified to be within sensor's supported range (6-27MHz)

**Dependencies:** R1 (power must be stable before clock)

---

### R3: Hardware Reset

**Description:** The driver must perform hardware reset with correct timing.

**Acceptance Criteria:**
- [ ] Reset GPIO is asserted (active low) for at least 70 EXTCLK cycles
- [ ] Reset GPIO is deasserted after the minimum assert time
- [ ] Wait time of at least 6000 EXTCLK cycles occurs after reset deassert for ROM boot
- [ ] At 24MHz EXTCLK, reset assert time is at least 3 microseconds
- [ ] At 24MHz EXTCLK, post-reset wait time is at least 250 microseconds
- [ ] First I2C access occurs only after post-reset wait completes

**Dependencies:** R2 (clock must be running for reset timing)

---

### R4: PLL Configuration

**Description:** The driver must configure the PLL for the required pixel clock.

**Acceptance Criteria:**
- [ ] PLL M and N dividers are written to register 0x0010
- [ ] PLL P dividers are written to register 0x0012
- [ ] PLL is enabled via register 0x0014
- [ ] Driver waits for PLL lock before proceeding
- [ ] PLL configuration produces a valid pixel clock for target frame rate
- [ ] PLL bypass mode is available if PLL is not required

**Dependencies:** R3 (reset must complete before PLL setup)

---

### R5: Soft Standby Enter

**Description:** The driver must be able to enter soft standby mode to reduce power consumption.

**Acceptance Criteria:**
- [ ] Setting bit 0 of register 0x0018 initiates standby entry
- [ ] Driver polls bit 14 of register 0x0018 until it becomes set
- [ ] Polling timeout of at least 100ms before returning error
- [ ] Sensor stops outputting frames after standby entry
- [ ] I2C interface remains accessible during standby

**Dependencies:** R4 (sensor must be initialized before standby)

---

### R6: Soft Standby Exit

**Description:** The driver must be able to exit soft standby mode and resume streaming.

**Acceptance Criteria:**
- [ ] Clearing bit 0 of register 0x0018 initiates standby exit
- [ ] Driver polls bit 14 of register 0x0018 until it becomes clear
- [ ] Polling timeout of at least 100ms before returning error
- [ ] Sensor resumes frame output after standby exit
- [ ] Previous configuration is retained after standby cycle

**Dependencies:** R5 (must be in standby to exit)

---

### R7: Runtime Power Management

**Description:** The driver must implement runtime PM for automatic power management.

**Acceptance Criteria:**
- [ ] Runtime PM is enabled during probe
- [ ] runtime_suspend callback enters soft standby or powers off
- [ ] runtime_resume callback exits standby or powers on
- [ ] PM runtime get/put are called around I2C transactions when needed
- [ ] Autosuspend delay is configurable (default reasonable value)
- [ ] Sensor enters low power state when not streaming

**Dependencies:** R5, R6 (standby operations), R1 (power control)

---

### R8: Double Buffer Control

**Description:** The driver must control double buffering to ensure atomic configuration updates.

**Acceptance Criteria:**
- [ ] Setting bit 15 of register 0x0248 suspends double buffer updates
- [ ] Clearing bit 15 of register 0x0248 resumes double buffer updates
- [ ] Configuration changes are bracketed by suspend/resume when atomicity required
- [ ] Double buffer state is restored on error paths

**Dependencies:** cavekit-core R2 (register access)

---

### R9: Regulator Handling

**Description:** The driver must acquire and control regulators specified in device tree.

**Acceptance Criteria:**
- [ ] Regulator "vdd" (digital core 1.8V) is acquired from device tree
- [ ] Regulator "vdd_io" (I/O voltage) is acquired from device tree
- [ ] Regulator "vaa" (analog 2.8V) is acquired from device tree
- [ ] Missing regulators cause probe to fail with appropriate error
- [ ] Regulator enable/disable errors are propagated to caller
- [ ] Bulk regulator APIs are used for efficient management

**Dependencies:** None (probe-time acquisition)

---

### R10: GPIO Handling

**Description:** The driver must acquire and control GPIOs specified in device tree.

**Acceptance Criteria:**
- [ ] GPIO "reset-gpios" is acquired from device tree
- [ ] Reset GPIO is configured as output
- [ ] Optional "powerdown-gpios" is supported if present
- [ ] Missing required GPIOs cause probe to fail with appropriate error
- [ ] GPIO states are set correctly during power sequences
- [ ] GPIOs are released on driver removal

**Dependencies:** None (probe-time acquisition)

## Out of Scope

- Voltage level configuration (assumes device tree specifies correct levels)
- Clock source selection (assumes device tree configures clock properly)
- Power domain management (handled by platform, not driver)
- Thermal management and throttling
- Power consumption measurement
- Low-power I2C modes

## Cross-References

- See also: [cavekit-core.md](cavekit-core.md) - Core depends on power being established (R1-R4)
- See also: [cavekit-controls.md](cavekit-controls.md) - Controls require sensor to be powered and not in standby
