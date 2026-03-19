#!/bin/bash
#
# Restore webOS Audio After Mainline Kernel Testing
#
# The mainline kernel can leave the WM8958 codec mixer switches in
# a disabled state. This script restores the critical audio routing
# for speaker output via the LINEOUT path (external Class-D amplifier).
#
# Usage: ./scripts/restore-webos-audio.sh
#
# Requirements: Device running webOS and connected via novacom
#
# =============================================================================
# HP TouchPad Audio Architecture
# =============================================================================
#
# The TouchPad uses LINEOUT (not SPKOUT) for speaker output, connected to an
# external TPA2016 Class-D amplifier. The internal WM8958 SPKOUT is NOT used.
#
# Audio Path:
#   AIF1 → DAC1 → Output Mixer → Output PGA → LINEOUT → External Class-D Amp → Speakers
#
# =============================================================================
# Working DAPM Widget State (during playback)
# =============================================================================
#
#   Speaker: On
#   DAC1L/DAC1R: On
#   DAC1L/DAC1R Mixer: On
#   Left/Right Output Mixer: On
#   Left/Right Output PGA: On
#   LINEOUT1P/1N Driver: On
#   LINEOUT2P/2N Driver: On
#   LINEOUT1/2 Mixer: On
#   AIF1CLK: On
#   CLK_SYS: On
#
# =============================================================================
# Working Codec Register State
# =============================================================================
#
#   Register   Value   Meaning
#   ---------  ------  -------------------------------------------------------
#   0x01 (PM1)  0x323   Headphone + VMID + BIAS enabled
#   0x03 (PM3)  0x3cf0  LINEOUT1/2 + MIXOUT + Output PGAs enabled
#   0x05 (PM5)  0x303   AIF1DAC1 + DAC1L/R enabled
#   0x2d        0x01    DAC1L → Left Output Mixer
#   0x2e        0x01    DAC1R → Right Output Mixer
#   0x34        0x01    MIXOUTL → LINEOUT1
#   0x35        0x01    MIXOUTR → LINEOUT2
#
# =============================================================================
# Mixer Controls Reference (by numid)
# =============================================================================
#
#   numid  Control Name                      Purpose
#   -----  --------------------------------  ----------------------------------
#   144    DAC1 Switch                       Enable DAC1 L/R
#   206    DAC1L Mixer AIF1.1 Switch         Route AIF1 to DAC1L
#   201    DAC1R Mixer AIF1.1 Switch         Route AIF1 to DAC1R
#   257    Left Output Mixer DAC Switch      Route DAC to Left Output Mixer
#   249    Right Output Mixer DAC Switch     Route DAC to Right Output Mixer
#   191    SPKL DAC1 Switch                  Speaker Left DAC1 input
#   186    SPKR DAC1 Switch                  Speaker Right DAC1 input
#   190    SPKL Output Switch                Speaker Left output enable
#   185    SPKR Output Switch                Speaker Right output enable
#   87     Speaker Switch                    Main speaker enable
#   237    SPKL Boost SPKL Switch            Speaker Left boost
#   235    SPKR Boost SPKR Switch            Speaker Right boost
#   232    LINEOUT1 Mixer Output Switch      LINEOUT1 mixer from output mixer
#   229    LINEOUT2 Mixer Output Switch      LINEOUT2 mixer from output mixer
#   95     LINEOUT1N Switch                  LINEOUT1 negative enable
#   96     LINEOUT1P Switch                  LINEOUT1 positive enable
#   98     LINEOUT2N Switch                  LINEOUT2 negative enable
#   99     LINEOUT2P Switch                  LINEOUT2 positive enable
#   97     LINEOUT1 Volume                   LINEOUT1 volume (0-1)
#   100    LINEOUT2 Volume                   LINEOUT2 volume (0-1)
#   85     Speaker Mixer Volume              Speaker mixer volume (0-3)
#   86     Speaker Volume                    Speaker output volume (0-63)
#   112    AIF1DAC1 Volume                   AIF1 DAC1 volume (0-96)
#   143    DAC1 Volume                       DAC1 output volume (0-96)
#
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=== webOS Audio Restore Script ==="
echo ""

# Check novacom connection
if ! novacom -l 2>/dev/null | grep -q "topaz"; then
    echo -e "${RED}ERROR: Device not connected via novacom${NC}"
    echo "Make sure the device is running webOS and connected via USB"
    exit 1
fi

echo -e "${GREEN}Device connected.${NC} Restoring audio routing..."
echo ""

# Full audio routing restore
# Note: webOS amixer outputs full mixer state, so we redirect to /dev/null
novacom run file://bin/sh -- -c '
echo "Enabling DAC1..."
amixer -c 0 cset numid=144 on,on >/dev/null 2>&1       # DAC1 Switch

echo "Enabling AIF1 -> DAC1 routing..."
amixer -c 0 cset numid=206 on >/dev/null 2>&1          # DAC1L Mixer AIF1.1 Switch
amixer -c 0 cset numid=201 on >/dev/null 2>&1          # DAC1R Mixer AIF1.1 Switch

echo "Enabling Output Mixer DAC path..."
amixer -c 0 cset numid=257 on >/dev/null 2>&1          # Left Output Mixer DAC Switch
amixer -c 0 cset numid=249 on >/dev/null 2>&1          # Right Output Mixer DAC Switch

echo "Enabling Speaker mixer routing..."
amixer -c 0 cset numid=191 on >/dev/null 2>&1          # SPKL DAC1 Switch
amixer -c 0 cset numid=186 on >/dev/null 2>&1          # SPKR DAC1 Switch
amixer -c 0 cset numid=85 3,3 >/dev/null 2>&1          # Speaker Mixer Volume

echo "Enabling Speaker output..."
amixer -c 0 cset numid=190 on >/dev/null 2>&1          # SPKL Output Switch
amixer -c 0 cset numid=185 on >/dev/null 2>&1          # SPKR Output Switch
amixer -c 0 cset numid=87 on,on >/dev/null 2>&1        # Speaker Switch
amixer -c 0 cset numid=86 57,57 >/dev/null 2>&1        # Speaker Volume

echo "Enabling Speaker boost..."
amixer -c 0 cset numid=237 on >/dev/null 2>&1          # SPKL Boost SPKL Switch
amixer -c 0 cset numid=235 on >/dev/null 2>&1          # SPKR Boost SPKR Switch

echo "Enabling LINEOUT path (external Class-D amp)..."
amixer -c 0 cset numid=232 on >/dev/null 2>&1          # LINEOUT1 Mixer Output Switch
amixer -c 0 cset numid=229 on >/dev/null 2>&1          # LINEOUT2 Mixer Output Switch
amixer -c 0 cset numid=95 on >/dev/null 2>&1           # LINEOUT1N Switch
amixer -c 0 cset numid=96 on >/dev/null 2>&1           # LINEOUT1P Switch
amixer -c 0 cset numid=98 on >/dev/null 2>&1           # LINEOUT2N Switch
amixer -c 0 cset numid=99 on >/dev/null 2>&1           # LINEOUT2P Switch
amixer -c 0 cset numid=97 1 >/dev/null 2>&1            # LINEOUT1 Volume
amixer -c 0 cset numid=100 1 >/dev/null 2>&1           # LINEOUT2 Volume

echo "Setting DAC/AIF volumes..."
amixer -c 0 cset numid=112 96,96 >/dev/null 2>&1       # AIF1DAC1 Volume
amixer -c 0 cset numid=143 96,96 >/dev/null 2>&1       # DAC1 Volume

echo ""
echo "Audio routing restored!"
'

echo ""
echo -e "${GREEN}=== Done ===${NC}"
echo ""
echo "Audio should now work. Test by playing a sound on the device."
echo ""
echo "Controls restored:"
echo "  - DAC1 and AIF1->DAC1 routing"
echo "  - Output Mixer DAC path"
echo "  - Speaker mixer and output switches"
echo "  - Speaker boost"
echo "  - LINEOUT path (for external Class-D amplifier)"
echo "  - Volume levels"
