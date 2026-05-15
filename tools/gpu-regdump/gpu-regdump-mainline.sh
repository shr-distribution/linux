#!/bin/sh
#
# gpu-regdump-mainline.sh - dump A2XX GPU register state via msm DRM debugfs
#
# Companion to gpu-regdump-webos. Reads the same set of MMIO registers via
# /sys/kernel/debug/dri/0/{cp,rbbm,mh,sq,...} and prints them in the same
# text format so the two outputs can be diffed.
#
# Format: offset  REG_NAME  = 0xVALUE
#   (matches gpu-regdump-webos's output exactly)
#
# Usage on device (mainline):
#   ./gpu-regdump-mainline.sh > regs-mainline-after-render.txt
#
# Then on host: diff regs-webos-after-render.txt regs-mainline-after-render.txt
#

DRI_DBG="/sys/kernel/debug/dri/0"

if [ ! -d "$DRI_DBG" ]; then
    echo "ERROR: $DRI_DBG not present (is this mainline freedreno?)" >&2
    exit 1
fi

echo "# A2XX MMIO register dump via $DRI_DBG (mainline DRM debugfs backend)"
echo "# format: offset  name  = value"

# Helper: extract a "NAME: VALUE" line from a debugfs file and emit
# in the canonical 'offset  NAME  = 0xVALUE' format. The mainline
# driver's a2xx_debugfs.c prints e.g.:
#   RB_BASE:     <hex>
# (no leading 0x). Normalize.
emit() {
    local off=$1 name=$2 file=$3 dbg_name=$4
    local val
    val=$(grep -E "^\s+${dbg_name}:" "$DRI_DBG/$file" 2>/dev/null \
            | head -1 | awk '{print $NF}')
    if [ -z "$val" ]; then
        printf "0x%04x  %-22s  = N/A\n" "$off" "$name"
    else
        # Strip leading 0x if present
        val=${val#0x}
        printf "0x%04x  %-22s  = 0x%08x\n" "$off" "$name" "0x$val"
    fi
}

# CP block
emit 0x01c0 CP_RB_BASE         cp RB_BASE
emit 0x01c1 CP_RB_CNTL         cp RB_CNTL
emit 0x01c5 CP_RB_RPTR         cp RB_RPTR
emit 0x01c6 CP_RB_WPTR         cp RB_WPTR
emit 0x017e CP_INT_CNTL        cp INT_CNTL
emit 0x01f4 CP_INT_STATUS      cp INT_STATUS

# Scratch
for i in 0 1 2 3 4 5 6 7; do
    emit $((0x0578 + i)) SCRATCH_REG$i cp SCRATCH_REG$i
done

# RBBM block
emit 0x0058 RBBM_PM_OVERRIDE1  rbbm PM_OVERRIDE1
emit 0x0059 RBBM_PM_OVERRIDE2  rbbm PM_OVERRIDE2
emit 0x005c RBBM_DEBUG         rbbm DEBUG
emit 0x017f RBBM_STATUS        rbbm STATUS
emit 0x0061 RBBM_INT_CNTL      rbbm INT_CNTL
emit 0x0062 RBBM_INT_STATUS    rbbm INT_STATUS

# MH block
emit 0x0050 MH_INTERRUPT_MASK    mh INTERRUPT_MASK
emit 0x0051 MH_INTERRUPT_STATUS  mh INTERRUPT_STATUS
emit 0x0052 MH_AXI_ERROR         mh AXI_ERROR
emit 0x040c MH_MMU_CONFIG        mh MMU_CONFIG

# Misc MMIO (these may not all be in debugfs files - emit best-effort)
emit 0x03b7 MASTER_INT_SIGNAL  rbbm MASTER_INT_SIGNAL
emit 0x0e1e TP0_CHICKEN        tp TP0_CHICKEN
emit 0x0f01 RB_BC_CONTROL      rb BC_CONTROL

# Also dump full debugfs files at the end as raw context for things
# the curated list doesn't cover. Useful for spot-checking unfamiliar
# state.
echo
echo "# === raw debugfs dumps below ==="
for f in cp rbbm mh sq tp vgt pa rb summary; do
    if [ -r "$DRI_DBG/$f" ]; then
        echo "# --- $f ---"
        cat "$DRI_DBG/$f" 2>/dev/null
    fi
done
