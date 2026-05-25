#!/bin/sh
stop bluetooth 2>/dev/null
sleep 1
( i=0; while [ $i -lt 250 ]; do /tmp/rdmem; echo "==="; i=$((i+1)); done ) > /tmp/btcap.txt 2>&1 &
LP=$!
start bluetooth 2>/dev/null
sleep 5
kill $LP 2>/dev/null
echo "done; snapshots=$(grep -c '===' /tmp/btcap.txt)"
