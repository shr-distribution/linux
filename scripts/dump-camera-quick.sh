#!/bin/bash
#
# dump-camera-quick.sh - Quick camera register dump (critical regs only)
#
# Usage: Run this DURING camera streaming on both webOS and mainline
#        to capture the active register state.
#

# Auto-detect devmem tool
if command -v devmem2 &>/dev/null; then
    r() { devmem2 $1 w 2>/dev/null | grep "Value at" | awk '{print $NF}'; }
elif command -v devmem &>/dev/null; then
    r() { devmem $1 2>/dev/null; }
else
    r() { dd if=/dev/mem bs=4 count=1 skip=$((${1}/4)) 2>/dev/null | hexdump -e '1/4 "0x%08X\n"'; }
fi

echo "=== Quick Camera Reg Dump - $(uname -r) ==="
echo ""
echo "CLOCKS:"
echo "  VFE_CC(0x104)=$(r 0x04000104)  [CSI0_VFE=b12, CSI1_VFE=b10]"
echo "  MISC_CC(0x58)=$(r 0x04000058)  [pix_sel=b25-26, rdi_sel=b12-13]"
echo ""
echo "CSIPHY1 (MT9M113):"
echo "  PROTOCOL(0x04)=$(r 0x04900004)  [LONG_PKT=b21, DECODE_ID=b18, ECC=b17]"
echo "  D1_CTRL(0x20)=$(r 0x04900020)   [PHY_EN=b8-9]"
echo "  D0_CTRL2(0x38)=$(r 0x04900038)  [settle=b31-24]"
echo "  CAM_CNTL(0x24)=$(r 0x04900024)  [lanes=b0-2, assign=b8-15]"
echo "  IRQ_STAT(0x08)=$(r 0x04900008)  [SOT=b4, ECC=b5, FS=b16, FE=b17]"
echo ""
echo "VFE31:"
echo "  AXI_OUT(0x40)=$(r 0x04500040)   [0x60=CAMIF_TO_AXI]"
echo "  CAMIF_CMD(0x1E0)=$(r 0x045001E0)"
echo "  CAMIF_STATUS(0x204)=$(r 0x04500204)  [0x80000000=HALTED]"
echo "  FRAME_CFG(0x1E8)=$(r 0x045001E8)  [pix|lines<<16]"
echo "  EFS_CFG(0x1E4)=$(r 0x045001E4)   [0=APS mode]"
echo "  IRQ_STAT0(0x30)=$(r 0x04500030)  [SOF=b0, EOF=b1]"
echo "  MODULE_CFG(0x10)=$(r 0x04500010)"
echo ""
echo "=== End Dump ==="
