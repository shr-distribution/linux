#!/bin/sh
# T7: ConfigFS unlink/rmdir + rebuild. Runs detached -- the unbind
# drops the gadget so SSH (over ECM) dies until we re-link and rebind.

LOG=/tmp/t7.log
CFG=/sys/kernel/config/usb_gadget/g_nova
: > "$LOG"
log() { echo "$(date +%T) $*" >> "$LOG"; }

log "stop loopback"
pkill -9 -f loopback 2>/dev/null
sleep 0.5

log "unbind UDC"
echo "" > "$CFG/UDC"
sleep 1

log "/dev nodes should be gone:"
ls /dev/novacom_* 2>&1 >> "$LOG"

log "unlink + rmdir function"
rm "$CFG/configs/c.1/novacom.0"
rmdir "$CFG/functions/novacom.0"
sleep 1

log "recreate function"
mkdir "$CFG/functions/novacom.0"
ln -s "$CFG/functions/novacom.0" "$CFG/configs/c.1/novacom.0"
sleep 1

log "/dev nodes back?"
ls /dev/novacom_* 2>&1 >> "$LOG"

log "rebind UDC"
echo ci_hdrc.0 > "$CFG/UDC"
sleep 2

log "restart loopback-mt"
nohup /tmp/loopback-mt > /tmp/loopback-mt.log 2>&1 < /dev/null &
disown
log "done"
