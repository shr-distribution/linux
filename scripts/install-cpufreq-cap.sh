#!/bin/sh
# install-cpufreq-cap.sh
# Push the cpufreq-cap.service unit to the TouchPad and enable it.
# After this, every boot caps scaling_max_freq at 1512 MHz on both CPUs.
# To unlock at runtime:
#   systemctl stop cpufreq-cap
#   echo 1836000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
#   echo 1836000 > /sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq
# To remove permanently:
#   systemctl disable --now cpufreq-cap
#   rm /etc/systemd/system/cpufreq-cap.service

set -eu

HOST="${1:-root@172.16.42.2}"
UNIT="$(dirname "$0")/cpufreq-cap.service"

if [ ! -f "$UNIT" ]; then
    echo "error: $UNIT not found" >&2
    exit 1
fi

echo "==> Copying $UNIT to $HOST:/etc/systemd/system/"
scp -P 22 "$UNIT" "$HOST:/etc/systemd/system/cpufreq-cap.service"

echo "==> Enabling + starting the unit"
ssh -p 22 "$HOST" '
    systemctl daemon-reload
    systemctl enable cpufreq-cap.service
    systemctl start cpufreq-cap.service
    echo "--- status ---"
    systemctl --no-pager status cpufreq-cap.service || true
    echo "--- current caps ---"
    for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq; do
        printf "%s = " "$c"; cat "$c"
    done
    echo "--- governor + current freq ---"
    for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        printf "%s = " "$f"; cat "$f"
    done
    for f in /sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_cur_freq; do
        printf "%s = " "$f"; cat "$f"
    done
'

echo "==> Done."
