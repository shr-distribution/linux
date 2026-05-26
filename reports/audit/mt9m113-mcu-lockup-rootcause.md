# MT9M113 MCU lockup — root cause (webOS trace vs mainline) + fix

**Date:** 2026-05-26
**Method:** instrumented the known-good webOS 2.6.35 mt9m113 driver with a per-I2C
`mt9m113-trace` printk + power/mode markers, ran the camera app through a full
workflow (photo, video, photo) and several app restarts, pulled dmesg + /var/log.
Traces in `reports/webos-mt9m113-*`.

## Symptom
On mainline, after some camera use the MT9M113 MCU wedges: `mt9m113: MCU var
0xa103 timeout`, `s_stream(1) ... failed: -110`. Only a **physical power cycle**
recovers it; a soft reboot does not. Hit ~4× during this session.

## webOS lifecycle (known-good), from the trace
Per camera-app **open** (observed 4×, at 186/532/544/557 s):
```
PWDN gpio107 -> 0 (enable)        + 20 ms
MCLK = 24 MHz                     + 5 ms
reg_init: 480-entry table         then poll 0xA103 (SEQ REFRESH) until idle
set_sensor_mode(preview/snapshot) on the LIVE sensor (no re-init)
```
Per camera-app **close** (observed at 564 s):
```
__msm_release -> vfe_release -> s_release -> msm_camio_sensor_clk_off  (+ PWDN -> 1)
```
i.e. **full hardware power-down on close, full power-up + reg-init on open**, every
time. Mode switches happen on the live, freshly-initialised MCU.

## Mainline divergence (root cause)

`mt9m113_runtime_resume()` / `mt9m113_runtime_suspend()`:

```c
static int mt9m113_runtime_resume(struct device *dev) {
	...
	if (sensor->powerdown)
		return 0;                 /* <-- bails before power_on + sensor_init */
	ret = mt9m113_power_on(sensor);
	...
	return mt9m113_sensor_init(sensor);
}
static int mt9m113_runtime_suspend(...) {
	if (sensor->powerdown)
		return 0;                 /* <-- bails before power_off */
	mt9m113_power_off(sensor);
}
```

The TouchPad **has** a `powerdown` GPIO (DT `powerdown-gpios`), so on this device
**both runtime-PM callbacks are no-ops**. Consequences (call-site map):

| step | webOS (per open/close) | mainline on TouchPad |
|------|------------------------|----------------------|
| `power_on()` (reset + MCU boot + PLL) | every open | **once at probe only** (runtime_resume skips it) |
| `sensor_init()` (reg init) | every open | once at probe + only on start_streaming *recovery* |
| `power_off()` (PWDN high, clk off) | every close | **never** during operation (runtime_suspend skips it) |
| powerdown GPIO | toggled every open/close | stuck at 0 after probe |
| MCLK / clock | on per open, off per close | left on permanently |

So mainline initialises the sensor **once at boot** and then only issues SEQ_CMD
to a never-power-cycled MCU. Once the MCU wedges, nothing resets it → permanent
`0xA103` timeout until physical power-off. webOS resets the MCU (PWDN + clk) on
every session, so it is always clean.

## Fix

Remove the two `if (sensor->powerdown) return 0;` early returns so runtime PM does
its job on this device:
- `runtime_resume` → `mt9m113_power_on()` (toggles powerdown→0, enables clk, soft
  reset + MCU boot + PLL) → `mt9m113_sensor_init()` (reg init). = webOS "open".
- `runtime_suspend` → `mt9m113_power_off()` (powerdown→1, clk off). = webOS "close".

`pm_runtime` autosuspend keeps the sensor resumed for the whole capture session
(refcounted by `pm_runtime_resume_and_get` in s_stream / set_fmt) and powers it
down after the last put + autosuspend delay — matching webOS (powered through a
session, down on close). `power_on()`'s existing "clocks already running + SEQ_CMD
== 0 → skip re-init" fast path and "SEQ_CMD stuck → soft reset" recovery remain
correct: after a real `power_off()` the clocks are off, so the next resume always
runs the full boot.

Likely the early-return was an attempt to avoid per-resume power churn, but it
disabled the very reset cycle the MCU needs. The webOS trace shows per-open
power-cycle is the correct, reliable behaviour.

## Validation plan
1. Build + deploy; from a clean (power-cycled) start, run multiple camera
   open/stream/close cycles. Expect: no `0xa103` timeout, no `s_stream -110`,
   across repeated opens — the case that previously needed a physical power cycle.
2. Confirm a `power_off`/`power_on` (PWDN toggle + clk off/on) happens around each
   session via dmesg, and that the MCU boot runs on each resume.
3. Watch that autosuspend does not power down mid-session (between preview and
   snapshot) — if it does, raise the autosuspend delay.
