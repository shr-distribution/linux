# gemini_reg_poll

Live tracer for the Qualcomm Gemini JPEG encoder on MSM8660.

Maps `/dev/mem` at the Gemini base (`0x04600000`, 4 KB) and the MMCC
clock controller (`0x04000000`, 64 KB), polls every interesting register
at ~10 kHz, and snapshots the full register state at three sync points:

- `before` — at startup, before any activity
- `during` — when the encoder first becomes active (FE/PIPELINE/IRQ
  registers go non-zero)
- `after` — once the encoder has been idle for >500 ms following activity

Every register-value transition between snapshots lands in
`gemini_delta.log` with a CLOCK_MONOTONIC timestamp.

## Build

Either via the local Makefile (cross-toolchain in `$PATH`):

```
make
```

Or directly:

```
arm-linux-gnueabihf-gcc -O2 -static -Wall \
    gemini_reg_poll.c -o gemini_reg_poll
```

The output is a static ARMv7 binary that runs on both LuneOS and webOS
rootfs without library deps.

## Use on webOS (capture OPAL ground truth)

Boot to webOS, drop the binary somewhere writable (`/media/internal/`),
make it executable, then:

```
# from a host shell (novacom or telnet to debug shell)
./gemini_reg_poll &
# now open the Camera app and take ONE photo, then leave the app idle
# poller exits automatically once the encoder has been idle for 500 ms

# pull results
ls -la /tmp/gemini_*
```

You should see four files:

```
/tmp/gemini_snapshot_before.txt
/tmp/gemini_snapshot_during.txt
/tmp/gemini_snapshot_after.txt
/tmp/gemini_delta.log
```

`*_during.txt` is the most useful single file — it captures the full
state of every Gemini register at the moment the encoder kicks off.

`gemini_delta.log` is the time-ordered list of every register-value
change we caught — read it top-to-bottom to reconstruct the exact write
sequence OPAL issues.

## Use on LuneOS (capture mainline state for diff)

Same procedure but trigger the encode via the test harness:

```
./gemini_reg_poll &
sleep 1
/tmp/test_gemini -w 320 -h 240
# poller exits ~500 ms after the test finishes
```

## Diffing the two captures

After collecting both `gemini_snapshot_during.txt` files:

```
diff -u opal_during.txt mainline_during.txt
```

Any line that differs between the two represents either:

1. A register OPAL writes that we don't (the line will appear in
   `opal_during.txt` with a non-zero value but `0x00000000` in
   `mainline_during.txt`).
2. A register OPAL writes with a different value than we use.
3. (less likely) A register we set but OPAL leaves at reset.

Cross-reference each diff line against
`reports/gemini-cross-vendor-register-map.md` to identify what the
register controls.

## Caveats

- Reading `TABLE_DATA` (`0x012C`) auto-increments `TABLE_INDEX`. The
  poller skips it in the tight poll loop and the snapshot dump notes
  it as "(skipped, side effects)". To capture quant/Huffman table
  contents, a separate one-shot reader that resets `TABLE_INDEX`
  afterwards would be needed.
- `IRQ_CLEAR` (`0x0018`) is write-1-to-clear. Reading it is harmless
  but the value isn't meaningful.
- The MMCC dump uses register names that cross-reference our mainline
  `mmcc-msm8660.c` — they are not exhaustive, just the clocks/resets
  most likely to affect Gemini.
- Polling at 10 kHz is fast enough to catch most encoder transitions
  but a single MCU encode at 153 MHz core clock can complete in <1 ms,
  so very short bursts of activity may show up as instantaneous
  transitions in the delta log. The "during" snapshot is taken on the
  first detected change, which sidesteps that limitation.
- On stock webOS the kernel may have `STRICT_DEVMEM=y`, which restricts
  `/dev/mem` mappings to "safe" regions. If `mmap` fails with EPERM,
  patching the running kernel's `devmem_is_allowed` is out of scope,
  but the doctor306-opal kernel is permissive enough for this to work.
