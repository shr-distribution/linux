#!/bin/sh
#
# dump-mmcc-plls.sh - Dump MMCC PLL and clock info on HP TouchPad
#
# Run on device via: novacom run file://bin/sh < scripts/dump-mmcc-plls.sh
# Or copy to device and execute directly
#
# NOTE: Direct register access requires either:
#   1. CONFIG_STRICT_DEVMEM=n in kernel config, or
#   2. Running on webOS 2.6 stock kernel with devmem2
#
# This script uses the clock debugfs interface when available.
#

echo "=============================================="
echo "MMCC PLL/Clock Analysis - HP TouchPad"
echo "=============================================="
echo "Date: $(date 2>/dev/null || echo 'unknown')"
echo ""

# Check if clock debugfs is available
if [ -d /sys/kernel/debug/clk ]; then
    echo "Using clock debugfs interface"
    echo ""

    echo "=== PLL Source Analysis ==="
    echo ""
    echo "Known PLLs:"
    echo "  PXO      = 27 MHz (crystal)"
    echo "  PLL8     = 384 MHz (general purpose)"
    echo "  MM_PLL1  = 800 MHz (multimedia)"
    echo "  MM_PLL0  = 1320 MHz (high-perf graphics - L=48, M=8, N=9)"
    echo ""

    echo "=== Active Clocks (rate > 0) ==="
    for dir in /sys/kernel/debug/clk/*/; do
        rate=$(cat "$dir/rate" 2>/dev/null)
        if [ "$rate" != "0" ] && [ "$rate" != "" ] && [ "$rate" != "1" ]; then
            name=$(basename "$dir")
            mhz=$((rate / 1000000))
            khz=$(((rate % 1000000) / 1000))
            printf "  %-28s %10d Hz (%3d.%03d MHz)\n" "$name" "$rate" "$mhz" "$khz"
        fi
    done | sort -t'(' -k2 -rn
    echo ""

    echo "=== Graphics Clock Analysis ==="
    for clk in gfx3d_clk gfx2d0_clk gfx2d1_clk mdp_clk pixel_mdp_clk rot_clk; do
        dir="/sys/kernel/debug/clk/$clk"
        if [ -d "$dir" ]; then
            rate=$(cat "$dir/rate" 2>/dev/null)
            mhz=$((rate / 1000000))
            echo "--- $clk: $mhz MHz ---"
            echo "  Available rates:"
            cat "$dir/list_rates" 2>/dev/null | while read r; do
                if [ "$r" != "0" ] && [ -n "$r" ]; then
                    rmhz=$((r / 1000000))
                    printf "    %d MHz\n" "$rmhz"
                fi
            done
            echo ""
        fi
    done

    echo "=== PLL0 Usage Detection ==="
    echo ""
    echo "Checking if any frequencies require PLL0 (1320 MHz)..."
    echo ""
    echo "Expected PLL0 frequencies (not from PLL8/MM_PLL1):"
    echo "  300 MHz = 1320 * 2/11 / pre_div"
    echo "  330 MHz = 1320 / 4"
    echo "  440 MHz = 1320 / 3"
    echo "  660 MHz = 1320 / 2"
    echo ""

    found_pll0=0
    for dir in /sys/kernel/debug/clk/*/; do
        if [ -f "$dir/list_rates" ]; then
            cat "$dir/list_rates" 2>/dev/null | while read r; do
                case "$r" in
                    300000000|330000000|440000000|660000000)
                        name=$(basename "$dir")
                        echo "  FOUND: $name has $(($r/1000000)) MHz (PLL0 derivative)"
                        found_pll0=1
                        ;;
                esac
            done
        fi
    done

    if [ "$found_pll0" = "0" ]; then
        echo "  No PLL0-only frequencies found in any clock's list_rates"
        echo "  -> PLL0 appears to NOT be enabled/used"
    fi
    echo ""

else
    echo "Clock debugfs not available, trying direct register access..."
    echo ""

    MMCC_BASE=0x04000000

    read_reg() {
        local addr=$1
        local name=$2

        if command -v devmem2 >/dev/null 2>&1; then
            val=$(devmem2 $addr 2>/dev/null | grep "Read" | awk '{print $NF}')
        elif command -v busybox >/dev/null 2>&1; then
            val=$(busybox devmem $addr 2>/dev/null)
        elif [ -c /dev/mem ]; then
            val=$(dd if=/dev/mem bs=4 count=1 skip=$((addr/4)) 2>/dev/null | \
                  hexdump -e '1/4 "0x%08x"' 2>/dev/null)
        else
            val="(no access)"
        fi

        printf "  %-12s [0x%08X]: %s\n" "$name" "$addr" "$val"
    }

    echo "--- MM_PLL0 (1320 MHz) ---"
    read_reg $((MMCC_BASE + 0x0300)) "MODE"
    read_reg $((MMCC_BASE + 0x0304)) "L_VAL"
    read_reg $((MMCC_BASE + 0x0308)) "M_VAL"
    read_reg $((MMCC_BASE + 0x030C)) "N_VAL"
    echo ""

    echo "--- MM_PLL1 (800 MHz, mainline: pll2) ---"
    read_reg $((MMCC_BASE + 0x031C)) "MODE"
    read_reg $((MMCC_BASE + 0x0320)) "L_VAL"
    read_reg $((MMCC_BASE + 0x0324)) "M_VAL"
    read_reg $((MMCC_BASE + 0x0328)) "N_VAL"
    echo ""

    echo "--- MDP Pixel Clock ---"
    read_reg $((MMCC_BASE + 0x00D4)) "PIXEL_CC"
    read_reg $((MMCC_BASE + 0x00D8)) "PIXEL_MD"
    read_reg $((MMCC_BASE + 0x00DC)) "PIXEL_NS"
    echo ""
fi

echo "=============================================="
echo "Summary"
echo "=============================================="
echo ""
echo "Current graphics clock sources (webOS 2.6 analysis):"
echo ""
echo "  gfx3d_clk @ 266.667 MHz  = 800/3      (MM_PLL1)"
echo "  gfx2d0_clk @ 228.571 MHz = 800*2/7    (MM_PLL1)"
echo "  mdp_clk @ 200 MHz        = 800/4      (MM_PLL1)"
echo "  rot_clk @ 160 MHz        = 800/5      (MM_PLL1)"
echo "  pixel_mdp_clk @ 96 MHz   = 384/4      (PLL8)"
echo ""
echo "PLL0 (1320 MHz) Status:"
echo "  - NOT currently active in normal power state"
echo "  - Would be used for turbo GPU frequencies (300+ MHz)"
echo "  - webOS may enable it dynamically under heavy GPU load"
echo ""
echo "For mainline Linux:"
echo "  - Mainline does NOT implement MM_PLL0"
echo "  - Uses only PLL8 (384 MHz) and pll2/MM_PLL1 (800 MHz)"
echo "  - This limits max GPU frequency compared to webOS"
echo ""
echo "=============================================="
