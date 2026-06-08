#!/bin/sh
# webOS BT UART register snapshot — run in `novacom run file://bin/sh`
# Captures GSBI6 UART + TLMM pinmux + clock config from the WORKING webOS chip
# for comparison against mainline's link-est failure case.

R=/tmp/rdmem
[ -x $R ] || { echo "ERROR: /tmp/rdmem missing — run 'gcc /tmp/rdmem.c -o /tmp/rdmem' first"; exit 1; }

dump() {
    label=$1; addr=$2
    v=$($R "$addr" 2>/dev/null | tail -1)
    printf "  %-12s @ %s = %s\n" "$label" "$addr" "$v"
}

# Snapshot at three moments to compare states.
snap() {
    state=$1
    echo "============================================================"
    echo "STATE: $state  (uptime=$(cut -d. -f1 /proc/uptime)s)"
    echo "============================================================"
    echo "GSBI6 UART (0x16540000):"
    dump "MR1"   0x16540000
    dump "MR2"   0x16540004
    dump "CSR/SR" 0x16540008    # write=CSR, read=SR
    dump "TF(skip)" "0x16540070"  # don't actually read TF, will pop FIFO
    dump "CR(skip)" "0x16540010"  # CR is write-only, reads garbage
    dump "IMR"   0x16540014
    dump "IPR"   0x16540018
    dump "TFWR"  0x1654001C
    dump "RFWR"  0x16540020
    dump "DMEN"  0x1654003C
    dump "NCF_TX" 0x16540040
    dump "RXFS"  0x1654006C
    echo
    echo "GSBI6 CTRL @ 0x16500000:"
    dump "GSBI_CTRL" 0x16500000
    echo
    echo "TCSR ADM1 CRCI mux:"
    dump "ADM1-A 0x78" 0x16b00078
    dump "ADM1-B 0x7c" 0x16b0007c
    echo
    echo "TLMM pin CFG (offset 0x801000 + 0x10*gpio):"
    for gpio in 51 52 53 54 130 131; do
        addr=$(printf "0x%x" $((0x801000 + gpio * 16)))
        v=$($R "$addr" 2>/dev/null | tail -1)
        printf "  gpio%-3s CFG @ %s = %s\n" "$gpio" "$addr" "$v"
    done
    echo
}

# 1) Snapshot the CURRENT (operational) state
snap "OPERATIONAL (BT up + chip configured)"

# 2) Snapshot during BT restart — captures link-est window
echo
echo "Killing BT stack to capture link-est window..."
killall PmBtEngine PmBtStack PmBtContacts BluetoothMonitor 2>/dev/null
sleep 2
snap "AFTER kill, before restart (chip in unknown / idle UART)"

# 3) Restart BT and poll for link-est window
echo
echo "Restarting PmBtStart..."
/usr/bin/PmBtStart >/tmp/bt_restart.log 2>&1 &
BTPID=$!
# Poll briefly while link-est is happening
for i in 1 2 3 4 5; do
    sleep 0.3
    snap "DURING-RESTART t=${i}*300ms"
done

echo "Done. Output above is the diagnostic. Press Ctrl-C when ready."
echo "Background PmBtStart PID = $BTPID"
wait $BTPID
