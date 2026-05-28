#!/bin/sh
# T6: soft-disconnect / soft-reconnect cycle. Runs detached because
# the disconnect drops ECM -> SSH. After this returns, host reconnects
# and the data plane is verified by re-running the host-tester.

LOG=/tmp/t6.log
: > "$LOG"
log() { echo "$(date +%T) $*" >> "$LOG"; }

log "starting soft-cycle"
log "stopping loopback (if any)"
pkill -9 -f loopback 2>/dev/null
sleep 0.5

log "soft disconnect"
echo disconnect > /sys/class/udc/ci_hdrc.0/soft_connect

sleep 3

log "soft connect"
echo connect > /sys/class/udc/ci_hdrc.0/soft_connect

sleep 5

log "restart loopback-mt"
nohup /tmp/loopback-mt > /tmp/loopback-mt.log 2>&1 < /dev/null &
disown
log "done"
