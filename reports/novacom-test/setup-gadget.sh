#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
#
# setup-gadget.sh — runs ON the TouchPad. Atomically swaps the built-in
# legacy g_ether gadget for a configfs-based composite gadget that
# carries BOTH ECM (so SSH stays reachable) AND novacom.
#
# This intentionally goes through configfs even for ECM so a single
# gadget driver owns the UDC. After this runs, kill the existing usb0
# interface on the host, replug logically, and SSH will come back via
# the new ECM at the same 172.16.42.2 address.
#
# Run detached from SSH because the swap kills the running session:
#   nohup sh /usr/local/sbin/setup-gadget.sh > /tmp/setup-gadget.log 2>&1 &
#   disown
#   exit
#
# Rollback: if any step fails the script re-binds g_ether and exits
# non-zero. Worst case (kernel-level failure), reboot recovers.

set -e

CFG=/sys/kernel/config/usb_gadget
GADGET=g_nova
UDC_NAME=ci_hdrc.0
LOG=/tmp/setup-gadget.log

# Pin the ECM MAC addresses so the host's USB-Ethernet interface name
# does NOT change (avoids losing SSH to a renamed enx<MAC> iface).
# Override via env: HOST_MAC=... DEV_MAC=... sh setup-gadget.sh
HOST_MAC="${HOST_MAC:-ae:c7:6a:09:cb:4b}"
DEV_MAC="${DEV_MAC:-7a:e6:c4:4c:e6:f0}"

log() { echo "[setup-gadget] $*" | tee -a "$LOG"; }

rollback() {
    log "rollback: re-binding g_ether to UDC"
    echo "" > "$CFG/$GADGET/UDC" 2>/dev/null || true
    echo "$UDC_NAME" > /sys/bus/gadget/drivers/g_ether/bind 2>/dev/null \
        || echo "$UDC_NAME" > /sys/class/udc/$UDC_NAME/device/driver/bind 2>/dev/null \
        || true
    exit 1
}

trap rollback EXIT

# Sanity
[ -d /sys/kernel/config ] || mount -t configfs none /sys/kernel/config
[ -d "$CFG" ] || { log "no usb_gadget configfs"; exit 1; }

# 1. Unbind g_ether from the UDC
if [ -e "/sys/bus/gadget/drivers/g_ether/unbind" ]; then
    log "unbinding g_ether"
    echo gadget.0 > /sys/bus/gadget/drivers/g_ether/unbind
fi

# Wait briefly for the UDC to settle
sleep 1

# 2. Tear down any pre-existing g_nova gadget, then build fresh
if [ -d "$CFG/$GADGET" ]; then
    log "removing pre-existing $GADGET"
    echo "" > "$CFG/$GADGET/UDC" 2>/dev/null || true
    find "$CFG/$GADGET/configs" -mindepth 2 -maxdepth 2 -type l -delete 2>/dev/null || true
    find "$CFG/$GADGET/configs" -mindepth 3 -type d -empty -delete 2>/dev/null || true
    find "$CFG/$GADGET/configs" -mindepth 1 -maxdepth 2 -type d -empty -delete 2>/dev/null || true
    find "$CFG/$GADGET/functions" -mindepth 1 -maxdepth 1 -type d -empty -delete 2>/dev/null || true
    find "$CFG/$GADGET/strings" -mindepth 1 -maxdepth 1 -type d -empty -delete 2>/dev/null || true
    rmdir "$CFG/$GADGET" 2>/dev/null || true
fi

log "creating $GADGET"
mkdir "$CFG/$GADGET"
cd "$CFG/$GADGET"

# Use HP/TouchPad vendor+product so the host-side novacomd recognises
# this device. novacomd hardcodes the 0x0830 / 0x8002 pair (and other
# Palm/HP webOS IDs) and won't enumerate Linux Foundation defaults.
echo 0x0830 > idVendor
echo 0x8002 > idProduct
echo 0x0316 > bcdDevice
echo 0x0200 > bcdUSB

mkdir -p strings/0x409
echo "Hewlett-Packard"             > strings/0x409/manufacturer
echo "HP TouchPad"                 > strings/0x409/product
echo "touchpad-novacom-01"         > strings/0x409/serialnumber

mkdir -p configs/c.1
mkdir -p configs/c.1/strings/0x409
echo "ECM + Novacom" > configs/c.1/strings/0x409/configuration
echo 250             > configs/c.1/MaxPower

# 3. ECM function (keeps SSH alive)
log "adding ecm.usb0 (host_addr=$HOST_MAC dev_addr=$DEV_MAC)"
mkdir functions/ecm.usb0
echo "$HOST_MAC" > functions/ecm.usb0/host_addr
echo "$DEV_MAC"  > functions/ecm.usb0/dev_addr

# 4. Novacom function (the thing under test)
log "adding novacom.0"
mkdir functions/novacom.0

# Link both into the config
ln -s functions/ecm.usb0    configs/c.1/ecm.usb0
ln -s functions/novacom.0   configs/c.1/novacom.0

# 5. Bind to UDC
log "binding to $UDC_NAME"
echo "$UDC_NAME" > UDC

# 6. Bring up ECM interface
sleep 1
ifconfig usb0 172.16.42.2 netmask 255.255.255.0 up 2>/dev/null || \
    ip addr add 172.16.42.2/24 dev usb0 2>/dev/null || true
ip link set usb0 up 2>/dev/null || true

log "done. /dev/novacom_* should now exist:"
ls -l /dev/novacom_* 2>&1 | tee -a "$LOG"

# success — don't roll back on exit
trap - EXIT
log "setup-gadget complete"
exit 0
