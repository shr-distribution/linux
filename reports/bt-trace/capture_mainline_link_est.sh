#!/bin/bash
# Capture the BT UART register state during mainline cold BT bring-up.
# Run from HOST after the device has booted into LuneOS/mainline and BEFORE
# the BT modules have loaded (i.e. via custom init or `systemctl mask` BT
# services so the BT chip is still in cold-boot state).
#
# This pipes /tmp/rdmem_bt (statically-linked ARM tool from rdmem_bt.c) into
# a tight poll loop while modprobing hci_uart, so we catch the very first
# milliseconds of BCSP link-establishment when MR1/MR2/IMR are at link-est
# values (8-N-1, no auto-flow) rather than the operational values webOS
# already showed us (MR1=0xf4, MR2=0x36).
#
# Pairs with reports/bt-trace/webos_uart_snapshot.sh (the equivalent capture
# from working webOS via novacom).

set -eu
DEV=root@172.16.42.2

# Push rdmem_bt to device
scp /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/bt-trace/rdmem_bt $DEV:/tmp/rdmem_bt
ssh $DEV chmod +x /tmp/rdmem_bt

ssh $DEV bash <<'EOF'
set -eu
# Make sure BT modules are NOT already loaded (chip should be cold).
if lsmod | grep -q hci_uart; then
    echo "WARNING: hci_uart already loaded — chip may be already past link-est."
    echo "Unloading..."
    rmmod hci_uart 2>/dev/null || true
    rmmod btbcm 2>/dev/null || true
    rmmod bluetooth 2>/dev/null || true
    sleep 1
fi

# Start a fast poll loop in background, then trigger BT modprobe.
# Poll cadence ~5ms, snapshot only when output changes.
( prev=""
  for i in $(seq 1 400); do
    cur=$(/tmp/rdmem_bt 2>&1 | tr '\n' ' ' | sed 's/  */ /g')
    if [ "$cur" != "$prev" ]; then
        printf "[i=%03d t=%s]\n%s\n\n" "$i" "$(date +%s.%N)" "$cur"
        prev="$cur"
    fi
    usleep 5000   # 5 ms
  done
) > /tmp/mainline_link_est_capture.txt 2>&1 &
POLL_PID=$!

# Give the poll loop 100ms to start, then bring up BT.
usleep 100000
echo "==> Loading BT modules at $(date +%s.%N)"
modprobe bluetooth
modprobe btbcm
modprobe hci_uart skip_pskeys=0 bt_linkest_burst=0 serdev_debug=1
sleep 5
hciconfig hci0 up >/dev/null 2>&1 &
sleep 10

# Stop poll loop
kill $POLL_PID 2>/dev/null || true
wait $POLL_PID 2>/dev/null || true

echo "==> Capture saved to /tmp/mainline_link_est_capture.txt"
wc -l /tmp/mainline_link_est_capture.txt
EOF

# Pull the capture back
scp $DEV:/tmp/mainline_link_est_capture.txt \
    /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/bt-trace/mainline_link_est_capture_$(date +%Y%m%d_%H%M%S).txt

echo "==> Capture complete. Compare against webOS operational snapshot:"
echo "    MR1=0xf4 MR2=0x36 IMR=0x203 IPR=0x200 DMEN=0x02 TFWR=0x20 RFWR=0x20"
echo "Expected at link-est: MR2=0x34 (8-N-1), MR1 bits 6,7 = 0 (no auto-flow)"
