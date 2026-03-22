#!/bin/bash
#
# Dump VFE31 and MMCC registers for MSM8660 camera debugging
# Run this on the device (TouchPad) while camera is streaming
#
# Usage on webOS: Run camera app, then SSH in and run this script
# Usage on mainline: Run while camera capture is attempted
#

VFE_BASE=0x04500000
MMCC_BASE=0x04000000
CSIPHY1_BASE=0x04900000

echo "=== VFE31 & MMCC Register Dump ==="
echo "Date: $(date)"
echo ""

# Check if devmem2 is available
if ! command -v devmem2 &> /dev/null; then
    echo "ERROR: devmem2 not found. Install with: opkg install devmem2"
    exit 1
fi

# Helper function to read register
read_reg() {
    local addr=$1
    devmem2 $addr w 2>/dev/null | tail -1 | awk '{print $NF}'
}

echo "=============================================="
echo "=== CRITICAL: Clock HALT Status Registers ==="
echo "=============================================="
echo "If any bit is 1, that clock is HALTED (not running)"
echo ""

printf "CLK_HALT_STATEC (0x104):  0x%08x\n" $(read_reg $((MMCC_BASE + 0x104)))
printf "CLK_HALT_STATED (0x108):  0x%08x\n" $(read_reg $((MMCC_BASE + 0x108)))
printf "CLK_HALT_STATEE (0x10C):  0x%08x\n" $(read_reg $((MMCC_BASE + 0x10C)))
printf "DBG_BUS_VEC_A (0x110):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x110)))
printf "DBG_BUS_VEC_B (0x114):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x114)))
printf "DBG_BUS_VEC_C (0x118):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x118)))
printf "DBG_BUS_VEC_D (0x11C):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x11C)))
printf "DBG_BUS_VEC_E (0x120):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x120)))
printf "DBG_BUS_VEC_F (0x124):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x124)))
printf "DBG_BUS_VEC_G (0x128):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x128)))
printf "DBG_BUS_VEC_H (0x12C):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x12C)))
printf "DBG_BUS_VEC_I (0x130):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x130)))
echo ""

# Decode VFE clock halt bits from DBG_BUS_VEC_B (based on mmcc-msm8660.c)
echo "=== VFE Clock Halt Status (from DBG_BUS_VEC_B) ==="
DBG_B=$(read_reg $((MMCC_BASE + 0x114)))
echo "DBG_BUS_VEC_B raw: $DBG_B"
# Common halt bit positions for VFE clocks (may need adjustment)
echo "  vfe_csi1_clk halt (bit 8): $(((DBG_B >> 8) & 1))"
echo "  vfe_csi0_clk halt (bit 9): $(((DBG_B >> 9) & 1))"
echo ""

echo "============================================"
echo "=== Software Reset Registers ==="
echo "============================================"
printf "SW_RESET_CORE (0x020):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x020)))
printf "SW_RESET_AHB (0x024):     0x%08x\n" $(read_reg $((MMCC_BASE + 0x024)))
printf "SW_RESET_AHB2 (0x028):    0x%08x\n" $(read_reg $((MMCC_BASE + 0x028)))
printf "SW_RESET_ALL (0x02C):     0x%08x\n" $(read_reg $((MMCC_BASE + 0x02C)))
echo ""

echo "============================================"
echo "=== MMCC Clock Enable/Select Registers ==="
echo "============================================"
printf "CSI_CC_REG (0x040):       0x%08x\n" $(read_reg $((MMCC_BASE + 0x040)))
printf "CSI1_CC_REG (0x044):      0x%08x\n" $(read_reg $((MMCC_BASE + 0x044)))
printf "CSI_NS_REG (0x048):       0x%08x\n" $(read_reg $((MMCC_BASE + 0x048)))
printf "MISC_CC_REG (0x058):      0x%08x\n" $(read_reg $((MMCC_BASE + 0x058)))
printf "MISC_CC2_REG (0x05C):     0x%08x\n" $(read_reg $((MMCC_BASE + 0x05C)))
printf "VFE_CC_REG (0x104-dup):   0x%08x\n" $(read_reg $((MMCC_BASE + 0x104)))
printf "VFE_NS_REG (0x014):       0x%08x\n" $(read_reg $((MMCC_BASE + 0x014)))
printf "VFE_CC2_REG (0x018):      0x%08x\n" $(read_reg $((MMCC_BASE + 0x018)))
echo ""

echo "=== Decoding MISC_CC_REG (0x058) ==="
MISC_CC=$(read_reg $((MMCC_BASE + 0x058)))
echo "  Raw value: $MISC_CC"
echo "  csi_pix_en (bit 26):  $(((MISC_CC >> 26) & 1))"
echo "  csi_pix_sel (bit 25): $(((MISC_CC >> 25) & 1)) (0=CSI0, 1=CSI1)"
echo "  csi_rdi_en (bit 13):  $(((MISC_CC >> 13) & 1))"
echo "  csi_rdi_sel (bit 12): $(((MISC_CC >> 12) & 1)) (0=CSI0, 1=CSI1)"
echo ""

echo "============================================"
echo "=== VFE Core Configuration Registers ==="
echo "============================================"
printf "VFE_HW_VERSION (0x000):   0x%08x\n" $(read_reg $((VFE_BASE + 0x000)))
printf "VFE_GLOBAL_RESET (0x004): 0x%08x\n" $(read_reg $((VFE_BASE + 0x004)))
printf "VFE_CGC_OVERRIDE (0x00C): 0x%08x\n" $(read_reg $((VFE_BASE + 0x00C)))
printf "VFE_MODULE_CFG (0x010):   0x%08x  <-- Module enables\n" $(read_reg $((VFE_BASE + 0x010)))
printf "VFE_CFG_OFF (0x014):      0x%08x  <-- pixelPattern\n" $(read_reg $((VFE_BASE + 0x014)))
printf "VFE_IRQ_CMD (0x018):      0x%08x\n" $(read_reg $((VFE_BASE + 0x018)))
printf "VFE_IRQ_MASK_0 (0x01C):   0x%08x\n" $(read_reg $((VFE_BASE + 0x01C)))
printf "VFE_IRQ_MASK_1 (0x020):   0x%08x\n" $(read_reg $((VFE_BASE + 0x020)))
printf "VFE_IRQ_CLEAR_0 (0x024):  0x%08x\n" $(read_reg $((VFE_BASE + 0x024)))
printf "VFE_IRQ_CLEAR_1 (0x028):  0x%08x\n" $(read_reg $((VFE_BASE + 0x028)))
printf "VFE_IRQ_STATUS_0 (0x02C): 0x%08x\n" $(read_reg $((VFE_BASE + 0x02C)))
printf "VFE_IRQ_STATUS_1 (0x030): 0x%08x\n" $(read_reg $((VFE_BASE + 0x030)))
echo ""

echo "=== VFE AXI/Bus Configuration ==="
printf "VFE_BUS_CMD (0x038):      0x%08x\n" $(read_reg $((VFE_BASE + 0x038)))
printf "AXI_OUT_MODE (0x040):     0x%08x\n" $(read_reg $((VFE_BASE + 0x040)))
printf "AXI_CFG_1 (0x044):        0x%08x\n" $(read_reg $((VFE_BASE + 0x044)))
printf "AXI_STATUS (0x1DC):       0x%08x\n" $(read_reg $((VFE_BASE + 0x1DC)))
echo ""

echo "=== VFE CAMIF Registers (Critical!) ==="
printf "CAMIF_CMD (0x1E0):        0x%08x\n" $(read_reg $((VFE_BASE + 0x1E0)))
printf "EFS_CFG (0x1E4):          0x%08x  <-- 0=APS mode\n" $(read_reg $((VFE_BASE + 0x1E4)))
printf "FRAME_CFG (0x1E8):        0x%08x  <-- width|height\n" $(read_reg $((VFE_BASE + 0x1E8)))
printf "WINDOW_WIDTH (0x1EC):     0x%08x\n" $(read_reg $((VFE_BASE + 0x1EC)))
printf "WINDOW_HEIGHT (0x1F0):    0x%08x\n" $(read_reg $((VFE_BASE + 0x1F0)))
printf "SUBSAMPLE_0 (0x1F4):      0x%08x\n" $(read_reg $((VFE_BASE + 0x1F4)))
printf "SUBSAMPLE_1 (0x1F8):      0x%08x\n" $(read_reg $((VFE_BASE + 0x1F8)))
printf "EPOCH_CFG (0x1FC):        0x%08x\n" $(read_reg $((VFE_BASE + 0x1FC)))
printf "RAW_CROP_WIDTH (0x200):   0x%08x\n" $(read_reg $((VFE_BASE + 0x200)))
printf "CAMIF_STATUS (0x204):     0x%08x  <-- Check for errors\n" $(read_reg $((VFE_BASE + 0x204)))
echo ""

echo "=== VFE ISP Pipeline Config ==="
printf "DEMUX_CFG (0x284):        0x%08x\n" $(read_reg $((VFE_BASE + 0x284)))
printf "DEMUX_GAIN_0 (0x288):     0x%08x\n" $(read_reg $((VFE_BASE + 0x288)))
printf "DEMUX_GAIN_1 (0x28C):     0x%08x\n" $(read_reg $((VFE_BASE + 0x28C)))
printf "CHROMA_UP (0x35C):        0x%08x\n" $(read_reg $((VFE_BASE + 0x35C)))
printf "REG_UPDATE_CMD (0x260):   0x%08x\n" $(read_reg $((VFE_BASE + 0x260)))
echo ""

echo "=== VFE VBIF/CDC Registers (if exist) ==="
printf "VBIF_CLK_ON? (0x400):     0x%08x\n" $(read_reg $((VFE_BASE + 0x400))) 2>/dev/null || echo "N/A"
printf "VBIF_AXI_CFG? (0x404):    0x%08x\n" $(read_reg $((VFE_BASE + 0x404))) 2>/dev/null || echo "N/A"
echo ""

echo "============================================"
echo "=== CSIPHY1 Registers ==="
echo "============================================"
printf "PHY_CONTROL (0x00):       0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x00)))
printf "PROTOCOL_CONTROL (0x04): 0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x04)))
printf "IRQ_STATUS (0x08):        0x%08x  <-- SOT/ECC IRQs\n" $(read_reg $((CSIPHY1_BASE + 0x08)))
printf "IRQ_MASK (0x0C):          0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x0C)))
printf "CALIBRATION_CTRL (0x18): 0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x18)))
printf "D1_CONTROL (0x20):        0x%08x  <-- PHY enable\n" $(read_reg $((CSIPHY1_BASE + 0x20)))
printf "CAMERA_CNTL (0x24):       0x%08x  <-- Lane config\n" $(read_reg $((CSIPHY1_BASE + 0x24)))
printf "D0_CONTROL (0x34):        0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x34)))
printf "D0_CONTROL2 (0x38):       0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x38)))
printf "CL_CONTROL (0x48):        0x%08x\n" $(read_reg $((CSIPHY1_BASE + 0x48)))
echo ""

echo "============================================"
echo "=== ANALYSIS ==="
echo "============================================"

echo ""
echo "--- VFE_CFG_OFF (0x014) Decode ---"
VFE_CFG=$(read_reg $((VFE_BASE + 0x014)))
PIXEL_PATTERN=$((VFE_CFG & 0x7))
echo "  Raw value: $VFE_CFG"
echo "  pixelPattern (bits 0-2): $PIXEL_PATTERN"
case $PIXEL_PATTERN in
    0) echo "    -> VFE_BAYER_RGRGRG" ;;
    1) echo "    -> VFE_BAYER_GRGRGR" ;;
    2) echo "    -> VFE_BAYER_BGBGBG" ;;
    3) echo "    -> VFE_BAYER_GBGBGB" ;;
    4) echo "    -> VFE_YUV_YCbYCr (YUYV)" ;;
    5) echo "    -> VFE_YUV_YCrYCb (YVYU)" ;;
    6) echo "    -> VFE_YUV_CbYCrY (UYVY) <-- Expected for MT9M113" ;;
    7) echo "    -> VFE_YUV_CrYCbY (VYUY)" ;;
esac
echo ""

echo "--- VFE_MODULE_CFG (0x010) Decode ---"
MODULE_CFG=$(read_reg $((VFE_BASE + 0x010)))
echo "  Raw value: $MODULE_CFG"
echo "  bit 2 (demuxEnable):    $(((MODULE_CFG >> 2) & 1))"
echo "  bit 3 (chromaUpsample): $(((MODULE_CFG >> 3) & 1))"
echo "  bit 23 (scaleEnc):      $(((MODULE_CFG >> 23) & 1))"
echo "  bit 27 (cropEnc):       $(((MODULE_CFG >> 27) & 1))"
echo ""

echo "--- FRAME_CFG (0x1E8) Decode ---"
FRAME_CFG=$(read_reg $((VFE_BASE + 0x1E8)))
FRAME_WIDTH=$((FRAME_CFG & 0xFFFF))
FRAME_HEIGHT=$(((FRAME_CFG >> 16) & 0xFFFF))
echo "  Raw value: $FRAME_CFG"
echo "  Width:  $FRAME_WIDTH"
echo "  Height: $FRAME_HEIGHT"
echo ""

echo "--- CSIPHY PROTOCOL_CONTROL (0x04) Decode ---"
PROTOCOL=$(read_reg $((CSIPHY1_BASE + 0x04)))
echo "  Raw value: $PROTOCOL"
echo "  ECC_EN (bit 17):     $(((PROTOCOL >> 17) & 1))"
echo "  DECODE_ID (bit 18):  $(((PROTOCOL >> 18) & 1))"
echo "  DATA_FORMAT (19-20): $(((PROTOCOL >> 19) & 0x3))"
echo "  LONG_PKT_CAP (21):   $(((PROTOCOL >> 21) & 1))"
echo ""

echo "=== DUMP COMPLETE ==="
echo ""
echo "KEY THINGS TO CHECK:"
echo "1. CLK_HALT_STATE* - Any 1 bits = clock halted"
echo "2. VFE_MODULE_CFG - demuxEnable should be 1 for YUV"
echo "3. VFE_CFG pixelPattern - Should be 6 (UYVY) for MT9M113"
echo "4. CSIPHY IRQ_STATUS - Should show SOT/ECC activity"
echo "5. CAMIF_STATUS - Check for error flags"
