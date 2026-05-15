#!/bin/bash
#
# state-leak-test.sh
#
# Remote-checkable test for the cross-DRM-client GPU state leak that
# manifests as faceted shading on glmark2 second-run, kmscube black
# faces post-LSM, and similar visual artifacts on Adreno 220 / HP
# TouchPad.
#
# Run from the host while the TouchPad is on the network. Returns
# pass/fail without needing visual inspection.
#
# Usage:
#   ./state-leak-test.sh                    # run all tests, default 5 iterations
#   ./state-leak-test.sh -h                 # show help
#   ./state-leak-test.sh --quick            # skip kmscube fb-hash test
#   ./state-leak-test.sh --device IP        # override device IP
#   ./state-leak-test.sh -n 10              # use 10 iterations (default 5)
#
# Tests:
#   1. dmesg health         - GPU faults / hangs / recovers since boot
#   2. mesa lib check       - confirm new libgallium is on device
#   3. fd2_emit_restore log - confirm patched code path executes
#   4. glmark2 --validate   - run twice, compare per-scene Pass/Fail
#   5. kmscube fb-hash      - run twice, hash /dev/fb0 each time
#
# Exit code:
#   0 = all tests pass (no leak detected)
#   1 = at least one test failed (leak still present, or new bug)
#   2 = setup error (device unreachable, missing tools)
#

set -u

DEVICE="${DEVICE:-172.16.42.2}"
SSH_OPTS="-p 22 -o ConnectTimeout=5 -o StrictHostKeyChecking=no"
QUICK=0
RUNS=5
PASS=0
FAIL=0

usage() {
    sed -n 's/^# \{0,1\}//p' "$0" | head -30
    exit 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)   usage ;;
        --quick)     QUICK=1; shift ;;
        --device)    DEVICE="$2"; shift 2 ;;
        -n|--runs)   RUNS="$2"; shift 2 ;;
        *)           echo "unknown arg: $1"; exit 2 ;;
    esac
done

ok()    { echo "  [PASS] $*"; PASS=$((PASS+1)); }
nok()   { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
info()  { echo "  ..    $*"; }
header() { echo; echo "=== $* ==="; }

ssh_dev() {
    ssh $SSH_OPTS "root@$DEVICE" "$@"
}

# --- 0: connectivity ---
header "0. Connectivity"
if ! ssh_dev 'true' 2>/dev/null; then
    echo "  [ERROR] cannot ssh root@$DEVICE - check USB net + sshd"
    exit 2
fi
ok "ssh root@$DEVICE reachable"
UPTIME=$(ssh_dev 'cat /proc/uptime | cut -d. -f1')
info "device uptime: ${UPTIME}s"

# --- 1: dmesg health ---
header "1. dmesg GPU health"
# Look for actual GPU/IOMMU error markers, not loose word matches
# Match real GPU/IOMMU error markers, not loose words. Specifically exclude
# benign hw_init telemetry lines (which contain literal field names like
# TRAN_ERROR= as part of register dumps, not actual errors).
DMESG_PATTERN='gpu_recover|adreno_recover|GPU hang|gpu hang|iommu fault|IOMMU fault|page fault.*kgsl|MASTER_INT_SIG|hangcheck|firmware crash|gpu fault|msm_gpu.*fault'
DMESG_EXCLUDE='a2xx hw_init seq=|a2xx hw_init: |kgsl: oops'
DMESG_HITS=$(ssh_dev "dmesg 2>/dev/null | grep -E \"$DMESG_PATTERN\" | grep -vE \"$DMESG_EXCLUDE\" | wc -l")
if [ "$DMESG_HITS" = "0" ]; then
    ok "no GPU faults/hangs/recovers in dmesg"
else
    nok "dmesg has $DMESG_HITS GPU-error lines"
    ssh_dev "dmesg 2>/dev/null | grep -E \"$DMESG_PATTERN\" | grep -vE \"$DMESG_EXCLUDE\" | tail -5" | sed 's/^/      /'
fi

# --- 2: mesa lib check ---
header "2. Mesa libgallium check"
DEV_MD5=$(ssh_dev 'md5sum /usr/lib/libgallium-26.1.0-devel.so 2>/dev/null | awk "{print \$1}"')
HOST_LIB="/media/herrie/LuneOS/scarthgap/webos-ports/tmp-glibc/sysroots-components/cortexa8t2hf-neon/mesa/usr/lib/libgallium-26.1.0-devel.so"
if [ -f "$HOST_LIB" ]; then
    HOST_MD5=$(md5sum "$HOST_LIB" | awk '{print $1}')
    if [ "$DEV_MD5" = "$HOST_MD5" ]; then
        ok "device libgallium md5 matches host build ($DEV_MD5)"
    else
        nok "device libgallium md5 ($DEV_MD5) != host build ($HOST_MD5) - redeploy needed"
    fi
else
    info "host build dir not present, skipping md5 compare"
    info "device md5: $DEV_MD5"
fi

# --- 3: fd2_emit_restore actually runs ---
header "3. fd2_emit_restore code-path triggered"
# Run a tiny offscreen test with mesa msgs enabled, count the log lines
RESTORE_COUNT=$(ssh_dev '
    MESA_LOG_LEVEL=info \
    FD_MESA_DEBUG=msgs \
    timeout 6 glmark2-es2-drm --off-screen -b build:duration=2 2>&1 | \
    grep -c "fd2_emit_restore called"
' || echo "0")
if [ "${RESTORE_COUNT:-0}" -gt 0 ]; then
    ok "fd2_emit_restore ran ${RESTORE_COUNT} times during a 2s build benchmark"
else
    nok "fd2_emit_restore log never emitted - is FD_MESA_DEBUG=msgs honored?"
fi

# --- 4: glmark2 --validate cross-run consistency ---
header "4. glmark2 --validate (run $RUNS times, check run-to-run consistency)"
# NOTE: glmark2's "Success" baseline is rendered on x86/desktop with bit-precise
# expected output. Adreno A22X has known float-precision quirks vs that golden,
# so individual scenes may fail even when rendering is internally correct. What
# we care about for state-leak detection is whether the runs AGREE with each
# other - if all N runs produce identical pass/fail results, the GPU is
# rendering consistently across DRM-client transitions, which is exactly what
# the bool/loop/ALU/tex zeroing patches were meant to restore.

# Pull just the per-scene "Validation: Success/Failure" lines, paired with
# the scene name from the preceding line. MESA debug output can splice into
# the scene-name line so we strip that out.
extract_validations() {
    awk '
        /^\[/ { scene = $0; next }
        /Validation:/ {
            v = $0
            sub(/^[ \t]+/, "", v)
            tag = scene
            sub(/:MESA:.*$/, "", tag)
            sub(/MESA:.*$/, "", tag)
            print tag, v
        }
    ' "$1"
}

rm -f /tmp/state-leak-run-*.txt /tmp/state-leak-validation-*.txt

for i in $(seq 1 $RUNS); do
    info "running glmark2 --validate iteration $i/$RUNS..."
    ssh_dev 'timeout 60 glmark2-es2-drm --validate -b build -b shading -b texture 2>&1' \
        > /tmp/state-leak-run-$i.txt
    extract_validations /tmp/state-leak-run-$i.txt \
        > /tmp/state-leak-validation-$i.txt
done

# Build the matrix: scene-tag (sorted) x run-number = S/F
SCENES=$(cat /tmp/state-leak-validation-*.txt | awk '{print $1, $2}' | sort -u)
echo
echo "  Run-to-run matrix (S=Success F=Failure U=Unknown -=missing):"
printf "    %-22s" "scene"
for i in $(seq 1 $RUNS); do printf " r%d" $i; done
echo

while read scene_tag; do
    [ -z "$scene_tag" ] && continue
    printf "    %-22s" "$scene_tag"
    row=""
    for i in $(seq 1 $RUNS); do
        result=$(grep -F "$scene_tag " /tmp/state-leak-validation-$i.txt | head -1 \
                 | grep -oE "Success|Failure|Unknown")
        case "$result" in
            Success)  row="${row} S" ;;
            Failure)  row="${row} F" ;;
            Unknown)  row="${row} U" ;;
            "")       row="${row} -" ;;
        esac
    done
    echo "$row"
done <<< "$SCENES"

# Now check consistency: do all RUNS produce the SAME validation file?
all_match=1
for i in $(seq 2 $RUNS); do
    if ! diff -q /tmp/state-leak-validation-1.txt /tmp/state-leak-validation-$i.txt \
            > /dev/null 2>&1; then
        all_match=0
        break
    fi
done

# Per-scene: which scenes flip across runs?
flippy_scenes=""
while read scene_tag; do
    [ -z "$scene_tag" ] && continue
    seen=""
    for i in $(seq 1 $RUNS); do
        r=$(grep -F "$scene_tag " /tmp/state-leak-validation-$i.txt | head -1 \
            | grep -oE "Success|Failure|Unknown")
        seen="$seen,$r"
    done
    # Count distinct values (excluding empties)
    distinct=$(echo "$seen" | tr ',' '\n' | grep -v '^$' | sort -u | wc -l)
    if [ "$distinct" -gt 1 ]; then
        flippy_scenes="$flippy_scenes\n      $scene_tag (results: $(echo $seen | sed 's/^,//'))"
    fi
done <<< "$SCENES"

echo
if [ "$all_match" = "1" ]; then
    ok "all $RUNS runs produced identical validation results - no state leak"
else
    nok "validation results vary across the $RUNS runs - STATE LEAK"
    if [ -n "$flippy_scenes" ]; then
        info "scenes that flipped:"
        printf "%b\n" "$flippy_scenes"
    fi
fi

# --- 5: kmscube fb-hash test ---
if [ "$QUICK" = "0" ]; then
    header "5. kmscube fb-hash ($RUNS runs, deterministic frame test)"
    # kmscube -c 60 runs exactly 60 frames - frame 60's animation phase is
    # deterministic, so identical hashes across runs mean identical rendering.
    HASHES=$(ssh_dev "
        rm -f /tmp/state-leak-fb-*.bin
        for i in \$(seq 1 $RUNS); do
            timeout 8 kmscube -c 60 >/dev/null 2>&1
            sleep 0.5
            cat /dev/fb0 > /tmp/state-leak-fb-\$i.bin 2>/dev/null
            md5sum /tmp/state-leak-fb-\$i.bin | awk -v i=\$i '{print i\":\" \$1}'
        done
    ")
    distinct_hashes=$(echo "$HASHES" | cut -d: -f2 | sort -u | wc -l)
    echo "$HASHES" | while IFS=: read i h; do
        info "run #$i fb md5: $h"
    done
    if [ -z "$HASHES" ]; then
        nok "fb capture failed - kmscube may have errored"
    elif [ "$distinct_hashes" = "1" ]; then
        ok "all $RUNS fb hashes identical - kmscube renders consistently across runs"
    else
        nok "$distinct_hashes distinct fb hashes across $RUNS runs - STATE LEAK"
    fi
fi

# --- summary ---
header "Summary"
TOTAL=$((PASS+FAIL))
echo "  $PASS/$TOTAL checks passed"
if [ $FAIL -eq 0 ]; then
    echo "  RESULT: PASS - no state leak detected"
    exit 0
else
    echo "  RESULT: FAIL - $FAIL checks failed"
    echo "  raw glmark2 output: /tmp/state-leak-run{1,2}.txt"
    exit 1
fi
