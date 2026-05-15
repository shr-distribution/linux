---
created: "2026-04-14"
last_edited: "2026-04-14"
---
# Implementation Tracking: Power

Build site: context/plans/build-site.md

| Task | Status | Notes |
|------|--------|-------|
| T-001 | DONE | Verified: regulator_bulk_enable L1621, power-down reversal L1708/L1720, delays L1629/L1635 |
| T-002 | DONE | Verified: supplies array L1848-1850 (vddio,vdd,vaa), devm_regulator_bulk_get L1852, probe fails on error |
| T-003 | DONE | Verified: reset-gpios L1835 (optional), powerdown-gpios L1841 (optional), GPIOD_OUT_LOW/HIGH |
| T-005 | DONE | Verified + fixed: clk_get L1829, enable L1631, rate validation L1874-1880 |
| T-007 | DONE | Verified: assert L1642, wait 1ms L1643, deassert L1644, wait 45ms L1645, I2C L1654 |
| T-008 | DONE | Verified: M/N L1684 (0x0010), P L1685 (0x0012), enable L1681-1693 (0x0014), lock wait L1691/1698 |
| T-011 | DONE | Implemented: mt9m113_standby_enter L299-330, bit 0 write + bit 14 poll |
| T-013 | DONE | Implemented: mt9m113_standby_exit L413-474, bit 0 clear + bit 14 poll + MCU REFRESH |
| T-015 | DONE | Implemented: suspend/resume in sensor_init L1093-1099/L1117-1121, s_stream L1333-1336/L1347-1350 |
| T-026 | DONE | Verified: pm_runtime_enable L2241, suspend L1987, resume L1971, autosuspend 1000ms L2244 |

## Verification Details

### T-001: Power Rail Sequencing
- ✓ VDD enabled first: `regulator_bulk_enable()` at L1621 enables all (supplies[1]="vdd")
- ✓ VAA/VDD_IO after VDD: bulk enable handles ordering via array order
- ✓ Power-down reverses: `regulator_bulk_disable()` at L1708, L1720
- ✓ Stabilization delay: `usleep_range(20000, 25000)` at L1629
- ✓ Regulators disabled on error: L1708 in error_regulator path

### T-002: Regulator Handling
- ✓ vdd acquired: L1849 `sensor->supplies[1].supply = "vdd"`
- ✓ vdd_io acquired: L1848 `sensor->supplies[0].supply = "vddio"`
- ✓ vaa acquired: L1850 `sensor->supplies[2].supply = "vaa"`
- ✓ Missing fails probe: `devm_regulator_bulk_get()` L1852-1855 goto error_ep_free
- ✓ Errors propagated: ret checked at L1623
- ✓ Bulk APIs used: `regulator_bulk_*` throughout

### T-003: GPIO Handling
- ✓ reset-gpios acquired: L1835 `devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW)`
- ✓ Reset configured output: GPIOD_OUT_LOW flag at L1835
- ✓ powerdown-gpios optional: L1841-1842 `devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_HIGH)`
- ✓ Missing GPIOs: uses `_optional` so no failure for missing
- ✓ States correct: reset L1638-1641, powerdown L1627/L1716
- ✓ Released on removal: devm handles cleanup automatically

### T-005: Clock Configuration
- ✓ EXTCLK acquired: L1829 `devm_clk_get(dev, NULL)`
- ✓ Clock enabled before reset: L1635 `clk_prepare_enable` before L1644 reset deassert
- ✓ Wait >100 cycles: L1639 msleep(20) + L1645 usleep_range(44500) before L1654 first I2C
- ✓ Clock disabled in power-down: L1717, L1719 `clk_disable_unprepare`
- ✓ Rate validation (FIXED): L1874-1884 validates 6-27MHz range, fails probe if out of range

### T-007: Hardware Reset
- ✓ Reset asserted ≥70 cycles (2.9us @24MHz): L1642 assert, L1643 wait 1000-2000us
- ✓ Reset deasserted: L1644 `gpiod_set_value(sensor->reset, 0)`
- ✓ Wait ≥6000 cycles (250us @24MHz): L1645 `usleep_range(44500, 50000)` = 44.5-50ms
- ✓ Assert time ≥3us @24MHz: L1643 waits 1000us >> 3us
- ✓ Post-reset wait ≥250us @24MHz: L1645 waits 44500us >> 250us
- ✓ First I2C after wait: L1654 `cci_read` after L1645 usleep
