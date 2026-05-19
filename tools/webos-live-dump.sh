#!/bin/sh
# webos-live-dump.sh — dump ADM/SDCC/clock/AHB live state while booted into webOS
#
# Run on the TouchPad after booting webOS (not LuneOS).
# Either:
#   novacom put file:///tmp/webos-live-dump.sh < webos-live-dump.sh
#   novacom run file:///bin/sh -- /tmp/webos-live-dump.sh > webos-dump.txt
# or paste into a novacom-launched sh.
#
# Output is a single dump that we can paste into the analysis.

set -u

DEVMEM=devmem
if ! command -v devmem >/dev/null 2>&1; then
	# webOS busybox sometimes has it under a different name
	if command -v busybox >/dev/null 2>&1 && busybox --list 2>/dev/null | grep -q devmem; then
		DEVMEM="busybox devmem"
	else
		echo "ERROR: no devmem in PATH or busybox; try /usr/sbin/devmem or scp a static devmem2 binary"
		exit 1
	fi
fi

dump32() {
	# $1 = label, $2 = phys addr (hex)
	val=$($DEVMEM "$2" 32 2>/dev/null || echo "ERR")
	printf "  %-40s @ %s = %s\n" "$1" "$2" "$val"
}

echo "==================================================================="
echo "webOS live ADM/SDCC/clock dump"
echo "==================================================================="
echo "uname  : $(uname -a)"
echo "uptime : $(uptime)"
echo "date   : $(date)"
echo

echo "===== /proc/version ====="
cat /proc/version 2>/dev/null
echo

echo "===== ADM1 CH_CONF — live (EE=1, offset 0xA40) — all 16 channels ====="
# ch2 = sdcc1 eMMC; ch5 = sdcc4 SDIO/WiFi; ch10-15 = modem
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18420A40 + i * 4)))
	dump32 "ADM1 ch$i CH_CONF (EE=1)" "$addr"
	i=$((i+1))
done
echo

echo "===== ADM1 CH_CONF — alt EE=0 (offset 0x240) — same channels for comparison ====="
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18420240 + i * 4)))
	dump32 "ADM1 ch$i CH_CONF (EE=0)" "$addr"
	i=$((i+1))
done
echo

echo "===== ADM1 CH_RSLT_CONF — live (EE=1, offset 0xB00) ====="
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18420B00 + i * 4)))
	dump32 "ADM1 ch$i CH_RSLT_CONF (EE=1)" "$addr"
	i=$((i+1))
done
echo

echo "===== ADM1 CRCI_CTL — live (EE=1, offset 0xC00) — all 16 CRCIs ====="
# CRCI 1 = sdcc1 eMMC; CRCI 5 = sdcc4 SDIO/WiFi; CRCI 4 = QCE CE_IN; ... (varies)
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18420C00 + i * 4)))
	dump32 "ADM1 CRCI_CTL[$i] (EE=1)" "$addr"
	i=$((i+1))
done
echo

echo "===== ADM1 CRCI_CTL — alt EE=0 (offset 0x400) — for comparison ====="
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18420400 + i * 4)))
	dump32 "ADM1 CRCI_CTL[$i] (EE=0)" "$addr"
	i=$((i+1))
done
echo

echo "===== ADM0 (crypto + audio) CH_CONF live (EE=1) — all 16 channels ====="
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18320A40 + i * 4)))
	dump32 "ADM0 ch$i CH_CONF (EE=1)" "$addr"
	i=$((i+1))
done
echo

echo "===== ADM0 CRCI_CTL live (EE=1) — all 16 CRCIs ====="
i=0
while [ $i -lt 16 ]; do
	addr=$(printf "0x%X" $((0x18320C00 + i * 4)))
	dump32 "ADM0 CRCI_CTL[$i] (EE=1)" "$addr"
	i=$((i+1))
done
echo

echo "===== SDCC1 / SDCC4 host registers (DPSM state, clkreg, datactrl, status) ====="
# MMCI register block per SDCC instance; same offsets as PL18x
# SDCC1 @ 0x12400000, SDCC4 @ 0x121C0000
for sdcc_label in "SDCC1 (eMMC) 0x12400000" "SDCC4 (WiFi) 0x121C0000"; do
	label=$(echo "$sdcc_label" | cut -d' ' -f1)
	base=$(echo "$sdcc_label" | awk '{print $NF}')
	echo "--- $label base=$base ---"
	dump32 "$label MMCIPOWER  (0x000)" "$(printf "0x%X" $((base + 0x000)))"
	dump32 "$label MMCICLOCK  (0x004)" "$(printf "0x%X" $((base + 0x004)))"
	dump32 "$label MMCIARG    (0x008)" "$(printf "0x%X" $((base + 0x008)))"
	dump32 "$label MMCICMD    (0x00C)" "$(printf "0x%X" $((base + 0x00C)))"
	dump32 "$label MMCIDATATIMER (0x024)" "$(printf "0x%X" $((base + 0x024)))"
	dump32 "$label MMCIDATALENGTH (0x028)" "$(printf "0x%X" $((base + 0x028)))"
	dump32 "$label MMCIDATACTRL  (0x02C)" "$(printf "0x%X" $((base + 0x02C)))"
	dump32 "$label MMCISTATUS    (0x034)" "$(printf "0x%X" $((base + 0x034)))"
	dump32 "$label MMCIMASK0     (0x03C)" "$(printf "0x%X" $((base + 0x03C)))"
	dump32 "$label MMCIMASK1     (0x040)" "$(printf "0x%X" $((base + 0x040)))"
done
echo

echo "===== /sys/kernel/debug/clk (rates and enable counts) ====="
if [ -d /sys/kernel/debug/clk ]; then
	for f in /sys/kernel/debug/clk/*sdc*/rate \
	         /sys/kernel/debug/clk/*sdc*/enable_count \
	         /sys/kernel/debug/clk/*dfab*/rate \
	         /sys/kernel/debug/clk/*dfab*/enable_count \
	         /sys/kernel/debug/clk/*afab*/rate \
	         /sys/kernel/debug/clk/*afab*/enable_count \
	         /sys/kernel/debug/clk/*adm*/rate \
	         /sys/kernel/debug/clk/*adm*/enable_count ; do
		[ -r "$f" ] && printf "  %-50s = %s\n" "$f" "$(cat "$f")"
	done
else
	echo "  /sys/kernel/debug/clk not available (CONFIG_DEBUG_FS off, or debugfs not mounted)"
fi
echo

echo "===== /proc/msm_clock_summary (if legacy debugfs not available) ====="
[ -r /proc/msm_clock_summary ] && cat /proc/msm_clock_summary
echo

echo "===== /proc/interrupts (ADM, mmci, sdc lines) ====="
grep -E "adm|mmci|sdc" /proc/interrupts 2>/dev/null || echo "  no matching lines"
echo

echo "===== /sys/kernel/debug/msm_dmov (per-channel state if exported) ====="
if [ -d /sys/kernel/debug/msm_dmov ]; then
	find /sys/kernel/debug/msm_dmov -type f -readable 2>/dev/null | while read f; do
		printf "%s :\n" "$f"
		head -c 4096 "$f" 2>/dev/null
		echo
	done
fi
echo

echo "===== AR6003 SDIO state (CCCR registers + function 1 host_int_status) ====="
# SDIO standard CCCR space is exposed by sdio driver in sysfs; addr=0x00..0xFF on function 0
# In webOS the ath6kl path differs; just try the easy ones
[ -r /sys/bus/sdio/devices ] && ls /sys/bus/sdio/devices 2>&1
for d in /sys/bus/sdio/devices/*/; do
	[ -d "$d" ] || continue
	for f in "$d"vendor "$d"device "$d"class "$d"revision; do
		[ -r "$f" ] && printf "  %-50s = %s\n" "$f" "$(cat "$f")"
	done
done
echo

echo "===== AHB / fabric priority candidates (read-only probe — these may EFAULT) ====="
# These addresses are speculative; webOS may or may not have them mapped at EE=0
# Tagged with ERR if not readable
dump32 "AFAB GLOBAL CTL 0x02000000 (guess)" "0x02000000"
dump32 "DFAB GLOBAL CTL 0x02100000 (guess)" "0x02100000"
dump32 "BIMC LIKE @ 0x00900000 (guess)"      "0x00900000"
echo "  (any ERR above just means that address isn't mapped at HLOS EE; that's fine, it tells us)"
echo

echo "==================================================================="
echo "Dump complete."
echo "==================================================================="
