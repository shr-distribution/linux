#!/bin/bash
#
# dump-camera-regs.sh - Dump camera subsystem registers for debugging
#
# Usage: ./dump-camera-regs.sh [output_file]
#
# This script dumps CSIPHY, VFE, and clock registers for comparing
# between webOS 2.6 kernel and mainline Linux.
#
# Run on both kernels during camera streaming to compare states.
#

OUTPUT="${1:-/tmp/camera-regs-$(date +%Y%m%d-%H%M%S).txt}"

# Check for devmem2 or busybox devmem
if command -v devmem2 &>/dev/null; then
    DEVMEM="devmem2"
    read_reg() { devmem2 $1 w 2>/dev/null | grep "Value at" | awk '{print $NF}'; }
elif command -v devmem &>/dev/null; then
    DEVMEM="devmem"
    read_reg() { devmem $1 2>/dev/null; }
else
    # Fallback: use /dev/mem directly with dd and hexdump
    DEVMEM="dd"
    read_reg() {
        local addr=$1
        local offset=$((addr & 0xFFF))
        local page=$((addr & ~0xFFF))
        dd if=/dev/mem bs=4 count=1 skip=$((addr/4)) 2>/dev/null | hexdump -e '1/4 "0x%08X\n"'
    }
fi

echo "Camera Register Dump - $(date)" | tee "$OUTPUT"
echo "Kernel: $(uname -r)" | tee -a "$OUTPUT"
echo "======================================" | tee -a "$OUTPUT"

#
# Base addresses for MSM8660/APQ8060
#
MMCC_BASE=0x04000000      # Multimedia Clock Controller
CSIPHY0_BASE=0x04800000   # CSIPHY0
CSIPHY1_BASE=0x04900000   # CSIPHY1 (MT9M113 front camera)
VFE_BASE=0x04500000       # VFE31

echo "" | tee -a "$OUTPUT"
echo "=== MMCC Clock Registers ===" | tee -a "$OUTPUT"
echo "VFE_CC_REG      (0x04000104): $(read_reg 0x04000104)" | tee -a "$OUTPUT"
echo "  - BIT(12)=CSI0_VFE_CLK, BIT(10)=CSI1_VFE_CLK" | tee -a "$OUTPUT"
echo "MISC_CC_REG     (0x04000058): $(read_reg 0x04000058)" | tee -a "$OUTPUT"
echo "  - BIT(25/26)=csi_pix mux, BIT(12/13)=csi_rdi mux" | tee -a "$OUTPUT"
echo "CAMCLK_CC_REG   (0x04000140): $(read_reg 0x04000140)" | tee -a "$OUTPUT"
echo "CSI0_CC_REG     (0x04000040): $(read_reg 0x04000040)" | tee -a "$OUTPUT"
echo "CSI1_CC_REG     (0x04000044): $(read_reg 0x04000044)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== CSIPHY0 Registers (0x04800000) ===" | tee -a "$OUTPUT"
echo "PHY_CONTROL     (0x00): $(read_reg 0x04800000)" | tee -a "$OUTPUT"
echo "PROTOCOL_CTRL   (0x04): $(read_reg 0x04800004)" | tee -a "$OUTPUT"
echo "IRQ_STATUS      (0x08): $(read_reg 0x04800008)" | tee -a "$OUTPUT"
echo "IRQ_MASK        (0x0C): $(read_reg 0x0480000C)" | tee -a "$OUTPUT"
echo "CALIBRATION     (0x18): $(read_reg 0x04800018)" | tee -a "$OUTPUT"
echo "D1_CONTROL      (0x20): $(read_reg 0x04800020)" | tee -a "$OUTPUT"
echo "CAMERA_CNTL     (0x24): $(read_reg 0x04800024)" | tee -a "$OUTPUT"
echo "D0_CONTROL      (0x34): $(read_reg 0x04800034)" | tee -a "$OUTPUT"
echo "D0_CONTROL2     (0x38): $(read_reg 0x04800038)" | tee -a "$OUTPUT"
echo "D1_CONTROL2     (0x3C): $(read_reg 0x0480003C)" | tee -a "$OUTPUT"
echo "CL_CONTROL      (0x48): $(read_reg 0x04800048)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== CSIPHY1 Registers (0x04900000) - MT9M113 ===" | tee -a "$OUTPUT"
echo "PHY_CONTROL     (0x00): $(read_reg 0x04900000)" | tee -a "$OUTPUT"
echo "PROTOCOL_CTRL   (0x04): $(read_reg 0x04900004)" | tee -a "$OUTPUT"
echo "IRQ_STATUS      (0x08): $(read_reg 0x04900008)" | tee -a "$OUTPUT"
echo "IRQ_MASK        (0x0C): $(read_reg 0x0490000C)" | tee -a "$OUTPUT"
echo "CALIBRATION     (0x18): $(read_reg 0x04900018)" | tee -a "$OUTPUT"
echo "D1_CONTROL      (0x20): $(read_reg 0x04900020)" | tee -a "$OUTPUT"
echo "CAMERA_CNTL     (0x24): $(read_reg 0x04900024)" | tee -a "$OUTPUT"
echo "D0_CONTROL      (0x34): $(read_reg 0x04900034)" | tee -a "$OUTPUT"
echo "D0_CONTROL2     (0x38): $(read_reg 0x04900038)" | tee -a "$OUTPUT"
echo "  - settle_cnt in bits[31:24], HS_TERM_IMP[23:16], LP_REC_EN[4], ERR_SOT_HS_EN[3]" | tee -a "$OUTPUT"
echo "D1_CONTROL2     (0x3C): $(read_reg 0x0490003C)" | tee -a "$OUTPUT"
echo "CL_CONTROL      (0x48): $(read_reg 0x04900048)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== VFE31 Registers (0x04500000) ===" | tee -a "$OUTPUT"
echo "HW_VERSION      (0x00): $(read_reg 0x04500000)" | tee -a "$OUTPUT"
echo "GLOBAL_RESET    (0x04): $(read_reg 0x04500004)" | tee -a "$OUTPUT"
echo "MODULE_CFG      (0x10): $(read_reg 0x04500010)" | tee -a "$OUTPUT"
echo "CGC_OVERRIDE    (0x14): $(read_reg 0x04500014)" | tee -a "$OUTPUT"
echo "AXI_OUT_MODE    (0x40): $(read_reg 0x04500040)" | tee -a "$OUTPUT"
echo "  - 0x60=CAMIF_TO_AXI, 0x200=OUTPUT_2" | tee -a "$OUTPUT"
echo "IRQ_MASK_0      (0x1C): $(read_reg 0x0450001C)" | tee -a "$OUTPUT"
echo "IRQ_MASK_1      (0x20): $(read_reg 0x04500020)" | tee -a "$OUTPUT"
echo "IRQ_STATUS_0    (0x30): $(read_reg 0x04500030)" | tee -a "$OUTPUT"
echo "IRQ_STATUS_1    (0x34): $(read_reg 0x04500034)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== VFE31 AXI Config (0x38-0xF0) ===" | tee -a "$OUTPUT"
echo "AXI_CFG         (0x38): $(read_reg 0x04500038)" | tee -a "$OUTPUT"
echo "AXI_CMD         (0x1D8): $(read_reg 0x045001D8)" | tee -a "$OUTPUT"
echo "AXI_STATUS      (0x1DC): $(read_reg 0x045001DC)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== VFE31 CAMIF Registers (0x1E0-0x208) ===" | tee -a "$OUTPUT"
echo "CAMIF_CMD       (0x1E0): $(read_reg 0x045001E0)" | tee -a "$OUTPUT"
echo "  - 0x1=START, 0x2=STOP_IMMEDIATE" | tee -a "$OUTPUT"
echo "EFS_CFG         (0x1E4): $(read_reg 0x045001E4)" | tee -a "$OUTPUT"
echo "  - 0x0 for APS mode (no embedded sync)" | tee -a "$OUTPUT"
echo "FRAME_CFG       (0x1E8): $(read_reg 0x045001E8)" | tee -a "$OUTPUT"
echo "  - pixels[15:0] | lines[31:16]" | tee -a "$OUTPUT"
echo "WINDOW_WIDTH    (0x1EC): $(read_reg 0x045001EC)" | tee -a "$OUTPUT"
echo "WINDOW_HEIGHT   (0x1F0): $(read_reg 0x045001F0)" | tee -a "$OUTPUT"
echo "SUBSAMPLE_CFG_0 (0x1F4): $(read_reg 0x045001F4)" | tee -a "$OUTPUT"
echo "SUBSAMPLE_CFG_1 (0x1F8): $(read_reg 0x045001F8)" | tee -a "$OUTPUT"
echo "EPOCH_CFG       (0x1FC): $(read_reg 0x045001FC)" | tee -a "$OUTPUT"
echo "CAMIF_STATUS    (0x204): $(read_reg 0x04500204)" | tee -a "$OUTPUT"
echo "  - 0x80000000=HALTED, lower bits=line count" | tee -a "$OUTPUT"
echo "CAMIF_MISR      (0x208): $(read_reg 0x04500208)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== VFE31 DEMUX Registers ===" | tee -a "$OUTPUT"
echo "DEMUX_CFG       (0x284): $(read_reg 0x04500284)" | tee -a "$OUTPUT"
echo "DEMUX_GAIN_0    (0x288): $(read_reg 0x04500288)" | tee -a "$OUTPUT"
echo "DEMUX_GAIN_1    (0x28C): $(read_reg 0x0450028C)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "=== VFE31 Write Master Registers ===" | tee -a "$OUTPUT"
for i in 0 1 2 3 4 5 6; do
    base=$((0x04500050 + i * 0x18))
    printf "WM%d_PADDR      (0x%03X): %s\n" $i $((0x50 + i * 0x18)) "$(read_reg $base)" | tee -a "$OUTPUT"
done

echo "" | tee -a "$OUTPUT"
echo "=== Framedrop Registers ===" | tee -a "$OUTPUT"
echo "FRAMEDROP_ENC_Y   (0x504): $(read_reg 0x04500504)" | tee -a "$OUTPUT"
echo "FRAMEDROP_ENC_CB  (0x508): $(read_reg 0x04500508)" | tee -a "$OUTPUT"
echo "FRAMEDROP_ENC_CR  (0x50C): $(read_reg 0x0450050C)" | tee -a "$OUTPUT"
echo "FRAMEDROP_VIEW_Y  (0x510): $(read_reg 0x04500510)" | tee -a "$OUTPUT"

echo "" | tee -a "$OUTPUT"
echo "======================================" | tee -a "$OUTPUT"
echo "Dump complete. Output saved to: $OUTPUT" | tee -a "$OUTPUT"
echo "" | tee -a "$OUTPUT"
echo "To compare two dumps:" | tee -a "$OUTPUT"
echo "  diff -y webos-dump.txt mainline-dump.txt | less" | tee -a "$OUTPUT"
