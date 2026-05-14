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
# NOTE: webOS amixer (2.6.35-palm-tenderloin) silently no-ops `sset "Name" on`
# and `cset name="Name"` for BOOLEAN switch controls — only `cset numid=N`
# actually toggles them. Earlier versions of this script used `sset` and
# appeared to succeed (volumes stuck, exit 0) but left the switches off, so
# audio failed.
#
# The numids below were captured from `amixer -c 0 contents` on the device.
# If a future webOS update reorders them, re-derive with:
#     novacom run file:///usr/bin/amixer -- -c 0 contents | grep -B1 "<name>"
#
#   numid  Control Name                       Purpose
#   -----  ---------------------------------  ----------------------------------
#    144   DAC1 Switch                        Enable DAC1 L/R
#    143   DAC1 Volume                        DAC1 volume (0-96)
#    206   DAC1L Mixer AIF1.1 Switch          Route AIF1.1 to DAC1L
#    201   DAC1R Mixer AIF1.1 Switch          Route AIF1.1 to DAC1R
#    257   Left Output Mixer DAC Switch       Route DAC to Left Output Mixer
#    249   Right Output Mixer DAC Switch      Route DAC to Right Output Mixer
#    191   SPKL DAC1 Switch                   Speaker Left DAC1 input
#    186   SPKR DAC1 Switch                   Speaker Right DAC1 input
#    190   SPKL Output Switch                 Speaker Left output enable
#    185   SPKR Output Switch                 Speaker Right output enable
#     87   Speaker Switch                     Main speaker enable
#    237   SPKL Boost SPKL Switch             Speaker Left boost
#    235   SPKR Boost SPKR Switch             Speaker Right boost
#    232   LINEOUT1 Mixer Output Switch       LINEOUT1 mixer from output mixer
#    229   LINEOUT2 Mixer Output Switch       LINEOUT2 mixer from output mixer
#     95   LINEOUT1N Switch                   LINEOUT1 negative enable
#     96   LINEOUT1P Switch                   LINEOUT1 positive enable
#     98   LINEOUT2N Switch                   LINEOUT2 negative enable
#     99   LINEOUT2P Switch                   LINEOUT2 positive enable
#     97   LINEOUT1 Volume                    LINEOUT1 volume (0-1)
#    100   LINEOUT2 Volume                    LINEOUT2 volume (0-1)
#     85   Speaker Mixer Volume               Speaker mixer volume (0-3)
#    112   AIF1DAC1 Volume                    AIF1 DAC1 volume (0-119)
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

# Full audio routing restore using cset numid.
# DO NOT switch this back to `sset "Name" on` — it silently no-ops on webOS
# amixer for BOOLEAN switches. See header note.
#
# Stage the restore script on the device and run it in one shell, so each
# cset call doesn't pay novacom round-trip latency.
REMOTE_SCRIPT=/tmp/restore-webos-audio-body.sh
LOCAL_TMP=$(mktemp)
cat > "$LOCAL_TMP" <<'REMOTE_EOF'
#!/bin/sh
set -e
A() { /usr/bin/amixer -c 0 cset "numid=$1" "$2" >/dev/null; }

echo "Enabling DAC1..."
A 144 on        # DAC1 Switch
A 143 96        # DAC1 Volume

echo "Enabling AIF1.1 -> DAC1 routing..."
A 206 on        # DAC1L Mixer AIF1.1 Switch
A 201 on        # DAC1R Mixer AIF1.1 Switch

echo "Enabling Output Mixer DAC path..."
A 257 on        # Left Output Mixer DAC Switch
A 249 on        # Right Output Mixer DAC Switch

echo "Enabling Speaker mixer routing..."
A 191 on        # SPKL DAC1 Switch
A 186 on        # SPKR DAC1 Switch
A 85 3          # Speaker Mixer Volume

echo "Enabling Speaker output..."
A 190 on        # SPKL Output Switch
A 185 on        # SPKR Output Switch
A 87 on         # Speaker Switch

echo "Enabling Speaker boost..."
A 237 on        # SPKL Boost SPKL Switch
A 235 on        # SPKR Boost SPKR Switch

echo "Enabling LINEOUT path (external Class-D amp)..."
A 232 on        # LINEOUT1 Mixer Output Switch
A 229 on        # LINEOUT2 Mixer Output Switch
A 95  on        # LINEOUT1N Switch
A 96  on        # LINEOUT1P Switch
A 98  on        # LINEOUT2N Switch
A 99  on        # LINEOUT2P Switch
A 97  1         # LINEOUT1 Volume
A 100 1         # LINEOUT2 Volume

echo "Setting AIF1DAC1 volume..."
A 112 96        # AIF1DAC1 Volume

echo "Audio routing restored!"
REMOTE_EOF

novacom put file://"$REMOTE_SCRIPT" < "$LOCAL_TMP" >/dev/null
rm -f "$LOCAL_TMP"
novacom run file://bin/sh -- "$REMOTE_SCRIPT"
novacom run file://bin/rm -- "$REMOTE_SCRIPT" >/dev/null 2>&1 || true

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
