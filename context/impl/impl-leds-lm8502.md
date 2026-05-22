---
domain: leds-lm8502
created: "2026-05-22"
last_updated: "2026-05-22"
status: RESOLVED-leds-light-from-boot-via-deferred-chip-init
---

# Implementation: LM8502 LED controller (HP TouchPad navi LEDs)

## ✅ RESOLVED 2026-05-22 — fix `58c3e5d96904`

Both HP TouchPad navi LEDs now light up cleanly from a fresh boot.
Verified on-device with the Knight-Rider animation (left/right
ping-pong + sync fade via `/sys/class/leds/`), brightness gradation
(128 = visible half-lit), and the kernel boot log:

```
lm8502 2-0033: LM8502 LED controller registered with 2 LEDs (chip_init deferred 3 s)
lm8502 2-0033: Sending software reset to LM8502
lm8502 2-0033: Deferred chip_init complete; brightness writes now active
```

**Root cause:** `regmap_write()` calls issued by `chip_init()` at
probe time (~2 s into boot) silently don't reach the chip — the
mainline i2c-qup driver does not propagate NACK on writes, so probe
thinks the init succeeded but the bytes never landed. The exact
same init sequence done via raw `I2C_RDWR` ioctl ~30 s+ into boot
DOES wake the chip; once awake the regmap path is fine. Likely the
QUP controller / RPM HPM transition / something else isn't fully
settled at probe time.

**Fix:** defer `chip_init()` to a `delayed_work` scheduled 3 s after
probe (retry 5 s on failure). `brightness_set()` returns `-EAGAIN`
until init completes instead of swallowing writes silently.
Companion userspace tools dropped at `tools/lm8502/`:
- `lm-wake.py` — manual wake via I2C_RDWR ioctl
- `lm-knight-rider.sh` — left/right ping-pong + fade demo

**Caveats / known cosmetics:**
- The "After init: ENGINE_CNTRL1=0xff" log lines are still noisy
  because mainline i2c-qup returns 0xff (bus pull-up) without
  errno on reads when the chip doesn't ACK. Writes still get
  through, so this is purely cosmetic. Future cleanup: convert
  the diagnostic block to `i2c_transfer` direct, or just remove.
- If you ever unbind the lm8502 driver, regulator l16 drops back
  to LPM (~1 mA), the chip dies, and the kernel can't wake it
  again without a fresh power cycle. `tools/lm8502/lm-wake.py` is
  a workaround in that case. Mostly only matters for development.

See memory note `project_lm8502_deferred_init.md` for the long-form
diagnosis trail.

## Historical investigation (kept for reference)

## Status: Chip not responding on mainline; webOS confirms HW is alive

The two HP TouchPad navigation-bar LEDs (left/right) are wired to D1
and D2 outputs of a TI LM8502 LED controller on I2C bus 2 (GSBI8 @
0x19880000), address 0x33. The chip is enabled by GPIO 121 active
high, powered by PM8058_L16 (1.8 V, always-on, allow-set-load → HPM
on probe).

**On mainline:** the LM8502 chip stays silent on I2C. Every transfer
NAKs (`-ENXIO` from `/dev/i2c-2` userspace). The driver's
`After init: …=0xff` reads were actually silent failures — the
`regmap_read` errored without us checking, leaving `val` at 0xff
(bus pull-up state). The A6 battery controllers at addresses 0x31
and 0x32 on the *same* bus probe and respond normally, so the bus
is healthy — the issue is chip-specific.

**On webOS (debug uImage trace 2026-05-22):** the legacy 2.6.35
LM8502 driver gets a clean ACK on the very first I2C transfer
10 ms after toggling gpio121 high:
```
[2.246879] LM8502-DBG: gpio_request OK, before set_dir gpio=121 value=0
[2.246893] LM8502-DBG: enable GPIO set high, gpio_get_value=1
[2.256917] LM8502-DBG: +10ms settle, about to issue first I2C (RESET write)
[2.257263] LM8502-DBG: write reg 0x3d = 0xff (i2c_transfer ret=1)   ← ACK
[2.327559] qup_i2c qup_i2c.3: I2C slave addr:0x33 not connected      ← 1st read fails
[2.334821] LM8502-DBG: read reg 0x01 -> 0x00 (i2c_transfer ret=2)    ← then OK
[2.387782] LM8502-DBG: read reg 0x36 -> 0x42 (i2c_transfer ret=2)    ← MISC default
[2.395473] LM8502-DBG: read reg 0x3a -> 0x40 (i2c_transfer ret=2)    ← STATUS
```
So:
- Chip is healthy, at address 0x33
- Wakes up within 10 ms of enable GPIO rising edge
- Even legacy has a one-shot first-read failure that the bus recovers from
- All subsequent transactions succeed

## What works on mainline

- L16 regulator: enabled at 1.8 V via `regulator-always-on` +
  `regulator-allow-set-load`; driver calls `regulator_set_load(100 mA)`
  → sysfs shows `requested_microamps: 100000`, `state: enabled`
- GPIO 121: configured as GPIO function via pinctrl
  (`lm8502_pins`), 8 mA drive, no bias
- I2C bus 2 (GSBI8): functional — A6 batteries at 0x31, 0x32
  respond
- LED class devices `lm8502:white:navi_left/right` register in
  `/sys/class/leds/` and accept brightness writes (no userspace
  error)

## What doesn't work

- Every I2C transfer to address 0x33 NAKs (-ENXIO from userspace
  raw I2C; silently from `regmap_read` in the driver)
- LEDs do not physically light despite brightness=255 being written

## Fixes attempted (chronological, all pushed)

| Commit | Change | Result |
|--------|--------|--------|
| `d5d8f243c313` | regulator-allow-set-load + `regulator_set_load(100mA)` in driver | sysfs requested_microamps=100000 ✓, chip still silent |
| `fdcf5f008bcd` | Reset BEFORE I2C verification read | did not help |
| `a2e06ff6b476` | Retry first write on `-ENXIO`, drop "verification" read | LED class registers, brightness writes succeed (silently), LEDs still dark |
| `ccdca45d8bd9` | gpio121 drive 2 → 8 mA + force LOW→HIGH 10 ms transition | did not help |
| `c658c7e959af` | BT + camera pins bumped to 8 mA (bundled) | did not help (cleanup along the way) |
| `37c2dac16713` | Remove bogus `&gsbi12_i2c=okay`; drop GSBI8 I2C pin drive 16 → 2 mA (match webOS) | pending rebuild |
| `73b5ab1453e2` | 10 → 100 ms hold-low on gpio121, 20 ms post-`set_load`, check read errors explicitly | pending rebuild |

## Why we believe the chip is healthy

- Same physical TouchPad, same bus, same address, same enable GPIO,
  same supply rail — webOS reads register defaults (`MISC = 0x42`,
  `STATUS = 0x40`) cleanly and the chip ACKs writes
- A6 batteries on the same bus respond fine in mainline
- Therefore the bus and the chip are both OK; mainline is missing
  something at the chip enable / I2C transaction layer

## Open hypotheses

1. **GPIO 121 bootloader state.** webOS gpiomux sets gpio121 LOW
   at board-init (so the chip is off through all of Linux boot,
   then deliberately powered on by the LM8502 driver at probe).
   In mainline, the pin's state between bootloader and driver
   probe is unknown — if moboot leaves it HIGH (chip on), our
   "force LOW for 10 ms" was too short to fully discharge the
   chip's supply, so the chip never sees a clean power-up edge.
   Bumped to 100 ms in `73b5ab1453e2`.

2. **RPM HPM transition latency.** `regulator_set_load(100 mA)`
   sends an IPC to RPM; the actual rail-mode change is not
   synchronous. We may have been toggling gpio121 / writing
   registers while L16 was still in LPM (~1 mA), starving the
   chip. Added 20 ms sleep after set_load in `73b5ab1453e2`.

3. **GSBI8 I2C pin drive too strong (16 mA).** Open-drain falling
   edges may have been overshooting/ringing enough that the
   LM8502 mis-decoded its address byte (despite the A6 batteries
   tolerating it). Dropped to 2 mA in `37c2dac16713`.

4. **Phantom GSBI12_I2C controller.** `&gsbi12_i2c { status =
   "okay"; }` was probing on hardware that's configured as UART,
   registering a phantom Linux i2c bus. Removed in `37c2dac16713`.
   Probably not directly causing the chip silence but was wrong.

## Open questions / next steps

- Once Yocto rebuilds with all of the above, retest. Look for
  "After init: ENGINE_CNTRL1 read failed: -6" → confirms NAK is
  the cause (and current fixes aren't enough). Or
  "ENGINE_CNTRL1=0x40 (expect 0x40)" → it worked.
- If still silent, consider whether moboot is holding the chip in
  some special state we need to undo (e.g. via a different GPIO
  or PMIC switch we haven't found). The PM8901_LVS0 always-on
  in webOS may gate a rail we haven't audited; needs schematic
  review.
- If the chip remains silent: capture an I2C bus trace
  (logic analyzer) to confirm whether our SLA+W actually reaches
  the chip and whether the chip is driving SDA low at the ACK
  bit. That distinguishes "chip not seeing the transaction" from
  "chip rejecting the address".

## Files

- `drivers/leds/leds-lm8502.c`
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`
  (lm8502 node, lm8502_pins, gsbi8_i2c_pins, pm8058_l16)
- Legacy reference: `webos-linux-kernel-touchpad/drivers/leds/
  leds-lm8502.c`, `arch/arm/mach-msm/board-tenderloin.c`,
  `gpiomux-tenderloin.c`
- Debug uImage with full I2C trace: `/uboot/uImage.webOSdebug`
  on device (rebuild from
  `webos-linux-kernel-touchpad/drivers/leds/leds-lm8502.c` with
  added `printk(... ret=%d ...)` on every i2c_transfer)
