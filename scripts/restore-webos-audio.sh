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
# Mixer Controls Reference
# =============================================================================
#
# NOTE: webOS amixer is broken for numid-based commands (cset/cget).
# Use control names with sset/sget instead.
#
#   Control Name                      Purpose
#   --------------------------------  ----------------------------------
#   DAC1                              Enable DAC1 L/R + volume
#   DAC1L Mixer AIF1.1                Route AIF1 to DAC1L
#   DAC1R Mixer AIF1.1                Route AIF1 to DAC1R
#   Left Output Mixer DAC             Route DAC to Left Output Mixer
#   Right Output Mixer DAC            Route DAC to Right Output Mixer
#   SPKL DAC1                         Speaker Left DAC1 input
#   SPKR DAC1                         Speaker Right DAC1 input
#   SPKL Output                       Speaker Left output enable
#   SPKR Output                       Speaker Right output enable
#   Speaker                           Main speaker enable + volume
#   SPKL Boost SPKL                   Speaker Left boost
#   SPKR Boost SPKR                   Speaker Right boost
#   LINEOUT1 Mixer Output             LINEOUT1 mixer from output mixer
#   LINEOUT2 Mixer Output             LINEOUT2 mixer from output mixer
#   LINEOUT1N                         LINEOUT1 negative enable
#   LINEOUT1P                         LINEOUT1 positive enable
#   LINEOUT2N                         LINEOUT2 negative enable
#   LINEOUT2P                         LINEOUT2 positive enable
#   LINEOUT1                          LINEOUT1 volume (0-1)
#   LINEOUT2                          LINEOUT2 volume (0-1)
#   Speaker Mixer                     Speaker mixer volume (0-3)
#   AIF1DAC1                          AIF1 DAC1 volume (0-96)
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

# Full audio routing restore using sset with control names
# Note: webOS amixer ignores numid-based cset commands, so we use sset with names
novacom run file://bin/sh -- -c '
echo "Enabling DAC1..."
amixer -c 0 sset "DAC1" on >/dev/null 2>&1
amixer -c 0 sset "DAC1" 96 >/dev/null 2>&1

echo "Enabling AIF1 -> DAC1 routing..."
amixer -c 0 sset "DAC1L Mixer AIF1.1" on >/dev/null 2>&1
amixer -c 0 sset "DAC1R Mixer AIF1.1" on >/dev/null 2>&1

echo "Enabling Output Mixer DAC path..."
amixer -c 0 sset "Left Output Mixer DAC" on >/dev/null 2>&1
amixer -c 0 sset "Right Output Mixer DAC" on >/dev/null 2>&1

echo "Enabling Speaker mixer routing..."
amixer -c 0 sset "SPKL DAC1" on >/dev/null 2>&1
amixer -c 0 sset "SPKR DAC1" on >/dev/null 2>&1
amixer -c 0 sset "Speaker Mixer" 3 >/dev/null 2>&1

echo "Enabling Speaker output..."
amixer -c 0 sset "SPKL Output" on >/dev/null 2>&1
amixer -c 0 sset "SPKR Output" on >/dev/null 2>&1
amixer -c 0 sset "Speaker" on >/dev/null 2>&1
amixer -c 0 sset "Speaker" 57 >/dev/null 2>&1

echo "Enabling Speaker boost..."
amixer -c 0 sset "SPKL Boost SPKL" on >/dev/null 2>&1
amixer -c 0 sset "SPKR Boost SPKR" on >/dev/null 2>&1

echo "Enabling LINEOUT path (external Class-D amp)..."
amixer -c 0 sset "LINEOUT1 Mixer Output" on >/dev/null 2>&1
amixer -c 0 sset "LINEOUT2 Mixer Output" on >/dev/null 2>&1
amixer -c 0 sset "LINEOUT1N" on >/dev/null 2>&1
amixer -c 0 sset "LINEOUT1P" on >/dev/null 2>&1
amixer -c 0 sset "LINEOUT2N" on >/dev/null 2>&1
amixer -c 0 sset "LINEOUT2P" on >/dev/null 2>&1
amixer -c 0 sset "LINEOUT1" 1 >/dev/null 2>&1
amixer -c 0 sset "LINEOUT2" 1 >/dev/null 2>&1

echo "Setting AIF1DAC1 volume..."
amixer -c 0 sset "AIF1DAC1" 96 >/dev/null 2>&1

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
