#!/bin/bash
#
# Dump VFE31 registers from a running system
# Run this on the device (TouchPad) while camera is streaming
#
# Usage on webOS: Run camera app, then SSH in and run this script
# Usage on mainline: Run while camera capture is attempted
#

VFE_BASE=0x04500000
MMCC_BASE=0x04000000
CSIPHY1_BASE=0x04900000

echo "=== VFE31 Register Dump ==="
echo "Date: $(date)"
echo ""

# Check if devmem2 is available
if ! command -v devmem2 &> /dev/null; then
    echo "ERROR: devmem2 not found. Install with: opkg install devmem2"
    exit 1
fi

echo "=== MMCC Clock Registers ==="
printf "CSI_CC_REG (0x040):     0x%08x\n" $(devmem2 $((MMCC_BASE + 0x040)) w | tail -1 | awk '{print $NF}')
printf "MISC_CC_REG (0x058):    0x%08x\n" $(devmem2 $((MMCC_BASE + 0x058)) w | tail -1 | awk '{print $NF}')
printf "VFE_CC_REG (0x104):     0x%08x\n" $(devmem2 $((MMCC_BASE + 0x104)) w | tail -1 | awk '{print $NF}')
echo ""

echo "=== VFE Core Config Registers ==="
printf "VFE_GLOBAL_RESET (0x004): 0x%08x\n" $(devmem2 $((VFE_BASE + 0x004)) w | tail -1 | awk '{print $NF}')
printf "VFE_CGC_OVERRIDE (0x00C): 0x%08x\n" $(devmem2 $((VFE_BASE + 0x00C)) w | tail -1 | awk '{print $NF}')
printf "VFE_MODULE_CFG (0x010):   0x%08x\n" $(devmem2 $((VFE_BASE + 0x010)) w | tail -1 | awk '{print $NF}')
printf "VFE_CFG_OFF (0x014):      0x%08x  <-- inputSource & pixelPattern\n" $(devmem2 $((VFE_BASE + 0x014)) w | tail -1 | awk '{print $NF}')
printf "VFE_IRQ_CMD (0x018):      0x%08x\n" $(devmem2 $((VFE_BASE + 0x018)) w | tail -1 | awk '{print $NF}')
printf "VFE_IRQ_MASK_0 (0x01C):   0x%08x\n" $(devmem2 $((VFE_BASE + 0x01C)) w | tail -1 | awk '{print $NF}')
printf "VFE_IRQ_MASK_1 (0x020):   0x%08x\n" $(devmem2 $((VFE_BASE + 0x020)) w | tail -1 | awk '{print $NF}')
printf "VFE_IRQ_STATUS_0 (0x02C): 0x%08x\n" $(devmem2 $((VFE_BASE + 0x02C)) w | tail -1 | awk '{print $NF}')
printf "VFE_IRQ_STATUS_1 (0x030): 0x%08x\n" $(devmem2 $((VFE_BASE + 0x030)) w | tail -1 | awk '{print $NF}')
echo ""

echo "=== VFE AXI/Bus Registers ==="
printf "VFE_BUS_CMD (0x038):      0x%08x\n" $(devmem2 $((VFE_BASE + 0x038)) w | tail -1 | awk '{print $NF}')
printf "AXI_OUT_MODE (0x040):     0x%08x\n" $(devmem2 $((VFE_BASE + 0x040)) w | tail -1 | awk '{print $NF}')
printf "AXI_STATUS (0x1DC):       0x%08x\n" $(devmem2 $((VFE_BASE + 0x1DC)) w | tail -1 | awk '{print $NF}')
echo ""

echo "=== VFE CAMIF Registers ==="
printf "CAMIF_CMD (0x1E0):        0x%08x\n" $(devmem2 $((VFE_BASE + 0x1E0)) w | tail -1 | awk '{print $NF}')
printf "EFS_CFG (0x1E4):          0x%08x\n" $(devmem2 $((VFE_BASE + 0x1E4)) w | tail -1 | awk '{print $NF}')
printf "FRAME_CFG (0x1E8):        0x%08x\n" $(devmem2 $((VFE_BASE + 0x1E8)) w | tail -1 | awk '{print $NF}')
printf "WINDOW_WIDTH (0x1EC):     0x%08x\n" $(devmem2 $((VFE_BASE + 0x1EC)) w | tail -1 | awk '{print $NF}')
printf "WINDOW_HEIGHT (0x1F0):    0x%08x\n" $(devmem2 $((VFE_BASE + 0x1F0)) w | tail -1 | awk '{print $NF}')
printf "SUBSAMPLE_0 (0x1F4):      0x%08x\n" $(devmem2 $((VFE_BASE + 0x1F4)) w | tail -1 | awk '{print $NF}')
printf "SUBSAMPLE_1 (0x1F8):      0x%08x\n" $(devmem2 $((VFE_BASE + 0x1F8)) w | tail -1 | awk '{print $NF}')
printf "EPOCH_CFG (0x1FC):        0x%08x\n" $(devmem2 $((VFE_BASE + 0x1FC)) w | tail -1 | awk '{print $NF}')
printf "CAMIF_STATUS (0x204):     0x%08x\n" $(devmem2 $((VFE_BASE + 0x204)) w | tail -1 | awk '{print $NF}')
echo ""

echo "=== VFE ISP Config (0x280-0x2A0) ==="
printf "DEMUX_CFG (0x284):        0x%08x\n" $(devmem2 $((VFE_BASE + 0x284)) w | tail -1 | awk '{print $NF}')
printf "DEMUX_GAIN_0 (0x288):     0x%08x\n" $(devmem2 $((VFE_BASE + 0x288)) w | tail -1 | awk '{print $NF}')
printf "DEMUX_GAIN_1 (0x28C):     0x%08x\n" $(devmem2 $((VFE_BASE + 0x28C)) w | tail -1 | awk '{print $NF}')
printf "CHROMA_UP (0x35C):        0x%08x\n" $(devmem2 $((VFE_BASE + 0x35C)) w | tail -1 | awk '{print $NF}')
echo ""

echo "=== VFE REG_UPDATE & REALIGN ==="
printf "REG_UPDATE_CMD (0x260):   0x%08x\n" $(devmem2 $((VFE_BASE + 0x260)) w | tail -1 | awk '{print $NF}')
printf "REALIGN_BUF (0x357):      0x%08x\n" $(devmem2 $((VFE_BASE + 0x357)) w | tail -1 | awk '{print $NF}') 2>/dev/null || echo "N/A"
echo ""

echo "=== CSIPHY1 Registers ==="
printf "PHY_CONTROL (0x00):       0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x00)) w | tail -1 | awk '{print $NF}')
printf "PROTOCOL_CONTROL (0x04): 0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x04)) w | tail -1 | awk '{print $NF}')
printf "IRQ_STATUS (0x08):        0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x08)) w | tail -1 | awk '{print $NF}')
printf "IRQ_MASK (0x0C):          0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x0C)) w | tail -1 | awk '{print $NF}')
printf "CALIBRATION_CTRL (0x18): 0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x18)) w | tail -1 | awk '{print $NF}')
printf "D1_CONTROL (0x20):        0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x20)) w | tail -1 | awk '{print $NF}')
printf "CAMERA_CNTL (0x24):       0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x24)) w | tail -1 | awk '{print $NF}')
printf "D0_CONTROL (0x34):        0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x34)) w | tail -1 | awk '{print $NF}')
printf "D0_CONTROL2 (0x38):       0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x38)) w | tail -1 | awk '{print $NF}')
printf "CL_CONTROL (0x48):        0x%08x\n" $(devmem2 $((CSIPHY1_BASE + 0x48)) w | tail -1 | awk '{print $NF}')
echo ""

echo "=== Decoding VFE_CFG_OFF (0x014) ==="
VFE_CFG=$(devmem2 $((VFE_BASE + 0x014)) w | tail -1 | awk '{print $NF}')
PIXEL_PATTERN=$((VFE_CFG & 0x7))
INPUT_SOURCE=$(((VFE_CFG >> 16) & 0x3))

echo "  Raw value: $VFE_CFG"
echo "  pixelPattern (bits 0-2): $PIXEL_PATTERN"
case $PIXEL_PATTERN in
    0) echo "    -> VFE_BAYER_RGRGRG" ;;
    1) echo "    -> VFE_BAYER_GRGRGR" ;;
    2) echo "    -> VFE_BAYER_BGBGBG" ;;
    3) echo "    -> VFE_BAYER_GBGBGB" ;;
    4) echo "    -> VFE_YUV_YCbYCr (YUYV)" ;;
    5) echo "    -> VFE_YUV_YCrYCb (YVYU)" ;;
    6) echo "    -> VFE_YUV_CbYCrY (UYVY)" ;;
    7) echo "    -> VFE_YUV_CrYCbY (VYUY)" ;;
esac

echo "  inputSource (bits 16-17): $INPUT_SOURCE"
case $INPUT_SOURCE in
    0) echo "    -> VFE_START_INPUT_SOURCE_CAMIF" ;;
    1) echo "    -> VFE_START_INPUT_SOURCE_TESTGEN" ;;
    2) echo "    -> VFE_START_INPUT_SOURCE_AXI" ;;
    *) echo "    -> UNKNOWN" ;;
esac
echo ""

echo "=== Decoding MISC_CC_REG (0x058) ==="
MISC_CC=$(devmem2 $((MMCC_BASE + 0x058)) w | tail -1 | awk '{print $NF}')
echo "  Raw value: $MISC_CC"
echo "  csi_pix_en (bit 26):  $(((MISC_CC >> 26) & 1))"
echo "  csi_pix_sel (bit 25): $(((MISC_CC >> 25) & 1)) (0=CSI0, 1=CSI1)"
echo "  csi_rdi_en (bit 13):  $(((MISC_CC >> 13) & 1))"
echo "  csi_rdi_sel (bit 12): $(((MISC_CC >> 12) & 1)) (0=CSI0, 1=CSI1)"
echo ""

echo "=== Dump Complete ==="
