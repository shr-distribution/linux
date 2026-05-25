#!/bin/sh
for d in /sys/class/regulator/regulator.*; do
  n=$(cat $d/name 2>/dev/null)
  u=$(cat $d/microvolts 2>/dev/null)
  s=$(cat $d/state 2>/dev/null)
  echo "$n|${u}|$s"
done
