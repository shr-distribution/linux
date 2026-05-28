#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# Driver script that brings up the FunctionFS gadget AND launches
# novacomd, end-to-end. Intended for development/testing -- production
# would split this into systemd units (see comments inside).
#
# Strategy:
#   1. Stop any running gadget (g_ether / g_nova)
#   2. mount functionfs at /dev/ffs-novacom
#   3. Build configfs gadget with ffs.novacom (+ optional ECM)
#   4. Start novacomd in the background; it opens ep0 and writes
#      FunctionFS descriptors+strings
#   5. Wait until ep1/ep2 are ready (descriptors accepted)
#   6. Echo UDC name to bind -- host enumerates, novacomd's ENABLE
#      event arrives, IO threads start

set -e

CFG=/sys/kernel/config/usb_gadget
GADGET=g_nova
UDC_NAME=ci_hdrc.0
FFS_DIR=/dev/ffs-novacom
NOVACOMD=${NOVACOMD:-/usr/sbin/novacomd}

# ECM MACs (override to match host so SSH stays alive across re-binds)
HOST_MAC="${HOST_MAC:-}"
DEV_MAC="${DEV_MAC:-}"

log() { echo "[ffs-setup] $*"; }

# 0. Stop existing
if [ -e /sys/bus/gadget/drivers/g_ether/unbind ]; then
    log "unbinding g_ether"
    echo gadget.0 > /sys/bus/gadget/drivers/g_ether/unbind 2>/dev/null || true
fi
pkill -x novacomd 2>/dev/null || true
sleep 0.3

# Tear down any pre-existing g_nova
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

# 1. Mount FunctionFS
mkdir -p "$FFS_DIR"
if ! mountpoint -q "$FFS_DIR"; then
    log "mounting functionfs at $FFS_DIR (source=novacom)"
    mount -t functionfs novacom "$FFS_DIR"
fi

# 2. Build configfs gadget
[ -d /sys/kernel/config ] || mount -t configfs none /sys/kernel/config
log "building $GADGET"
mkdir "$CFG/$GADGET"
cd "$CFG/$GADGET"

echo 0x0830 > idVendor
echo 0x8002 > idProduct
echo 0x0316 > bcdDevice
echo 0x0200 > bcdUSB

mkdir -p strings/0x409
echo "Hewlett-Packard"     > strings/0x409/manufacturer
echo "HP TouchPad"         > strings/0x409/product
echo "touchpad-functionfs" > strings/0x409/serialnumber

mkdir -p configs/c.1
mkdir -p configs/c.1/strings/0x409
echo "Novacom" > configs/c.1/strings/0x409/configuration
echo 250       > configs/c.1/MaxPower

log "adding ffs.novacom function"
mkdir functions/ffs.novacom

# Optional ECM
if [ -n "$HOST_MAC" ] && [ -n "$DEV_MAC" ]; then
    log "adding ecm.usb0 (host=$HOST_MAC dev=$DEV_MAC)"
    mkdir functions/ecm.usb0
    echo "$HOST_MAC" > functions/ecm.usb0/host_addr
    echo "$DEV_MAC"  > functions/ecm.usb0/dev_addr
    ln -s functions/ecm.usb0 configs/c.1/ecm.usb0
fi

ln -s functions/ffs.novacom configs/c.1/ffs.novacom

# 3. Start novacomd (it will open ep0 and write descriptors)
log "starting novacomd"
nohup "$NOVACOMD" > /tmp/novacomd.log 2>&1 < /dev/null &
disown

# 4. Wait for ep1/ep2 to appear (descriptors accepted)
log "waiting for ep1/ep2"
for i in 1 2 3 4 5 6 7 8 9 10; do
    if [ -e "$FFS_DIR/ep1" ] && [ -e "$FFS_DIR/ep2" ]; then
        break
    fi
    sleep 0.5
done
if [ ! -e "$FFS_DIR/ep1" ]; then
    log "ERROR: ep1 didn't appear; novacomd log:"
    cat /tmp/novacomd.log
    exit 1
fi

# 5. Bind UDC
log "binding UDC $UDC_NAME"
echo "$UDC_NAME" > "$CFG/$GADGET/UDC"

# Reapply IP on the ECM interface if we have one
if [ -n "$HOST_MAC" ]; then
    sleep 1
    ifconfig usb0 172.16.42.2 netmask 255.255.255.0 up 2>/dev/null || \
        ip addr add 172.16.42.2/24 dev usb0 2>/dev/null || true
    ip link set usb0 up 2>/dev/null || true
fi

log "done. Host should see Palm 0830:8002. Try 'novacom -l' on host."
exit 0
