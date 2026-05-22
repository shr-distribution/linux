#!/bin/sh
# HP TouchPad navi-LED Knight-Rider demo via sysfs.
# Requires the LM8502 chip to already be awake — run
# tools/lm8502/lm-wake.py first on a fresh boot if the chip is silent.
set -e
L=/sys/class/leds/lm8502:white:navi_left/brightness
R=/sys/class/leds/lm8502:white:navi_right/brightness

[ -w "$L" ] || { echo "navi LEDs not present at $L"; exit 1; }

echo 0 > "$L"; echo 0 > "$R"

# Phase 1: left/right ping-pong
for i in 1 2 3 4 5 6 7 8 9 10; do
  echo 255 > "$L"; echo 0   > "$R"; sleep 0.2
  echo 0   > "$L"; echo 255 > "$R"; sleep 0.2
done

# Phase 2: synchronous fade
for v in 50 100 150 200 255 200 150 100 50 0; do
  echo $v > "$L"; echo $v > "$R"
  sleep 0.15
done
