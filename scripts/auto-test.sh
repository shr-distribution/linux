#!/bin/bash
#
# auto-test.sh — full mesa/kernel build → deploy → reboot → USB-up →
# 100-cap test pipeline for the cycle-of-8 bisect on Adreno 220 / TouchPad.
#
# Usage (called from inside a Monitor command):
#   auto-test.sh mesa <prev_md5> <test_dirname>
#   auto-test.sh kernel <prev_kernel_commit_short> <test_dirname>
#   auto-test.sh both <prev_mesa_md5> <prev_kernel_commit_short> <test_dirname>
#
# Each step echoes a status line so the host monitor can follow progress.

set -u

WEBOS=/media/herrie/LuneOS/scarthgap/webos-ports
TOPAZ_HOST=root@172.16.42.2
SSH_OPTS="-p 22 -o ConnectTimeout=5 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
SCP_OPTS="-P 22 -o ConnectTimeout=5 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
SUDO_PASS="M0n1qu3@@"
GLCAP=/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/tools/gpu-regdump/gl-cap-and-regdump-mainline

PKG=$WEBOS/tmp-glibc/work/cortexa8t2hf-neon-webos-linux-gnueabi/mesa/26.1.0+git/packages-split
LIBGAL_LOCAL=$PKG/libgallium/usr/lib/libgallium-26.1.0-devel.so
KIMG_LOCAL=$WEBOS/tmp-glibc/deploy/images/tenderloin/uImage-dtb-zImage
ZIMG_LOCAL=$WEBOS/tmp-glibc/deploy/images/tenderloin/zImage-tenderloin.bin
KERNEL_SRC=/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin
KRECIPE=$WEBOS/meta-smartphone/meta-hp/recipes-kernel/linux/linux-hp-tenderloin_git.bb

# ----- helpers ---------------------------------------------------------------

log() { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }

bitbake_target() {
    local target="$1"
    local logfile="/tmp/bitbake-${target}.log"
    log "bitbake -f $target (logging to $logfile)"
    ( cd "$WEBOS" && bash -c ". setup-env && MACHINE=tenderloin bitbake -f $target" \
        >"$logfile" 2>&1 ) &
    local pid=$!
    while kill -0 $pid 2>/dev/null; do sleep 30; done
    # bitbake's success summary varies: "all succeeded" or "0 failed".
    # An ERROR line on the failure path appears unconditionally.
    if grep -qE '^ERROR:' "$logfile"; then
        log "BUILD FAILED for $target — last 30 lines of log:"
        tail -30 "$logfile"
        return 1
    fi
    if ! grep -qE 'Tasks Summary: .*(all succeeded|0 failed)' "$logfile"; then
        log "BUILD STATUS UNCLEAR for $target — last 30 lines of log:"
        tail -30 "$logfile"
        return 1
    fi
    log "build OK: $target"
    return 0
}

deploy_mesa() {
    log "scp mesa libs to device"
    scp $SCP_OPTS \
        "$PKG/libgallium/usr/lib/libgallium-26.1.0-devel.so" \
        "$PKG/libegl-mesa/usr/lib/libEGL.so.1.0.0" \
        "$PKG/libgles2-mesa/usr/lib/libGLESv2.so.2.0.0" \
        "$PKG/libgles1-mesa/usr/lib/libGLESv1_CM.so.1.1.0" \
        "$PKG/libgbm/usr/lib/libgbm.so.1.0.0" \
        "$TOPAZ_HOST:/usr/lib/" 2>&1 | tail -1
    scp $SCP_OPTS "$PKG/libgbm/usr/lib/gbm/dri_gbm.so" "$TOPAZ_HOST:/usr/lib/gbm/" 2>&1 | tail -1
    log "mesa md5 on device: $(ssh $SSH_OPTS $TOPAZ_HOST 'md5sum /usr/lib/libgallium-26.1.0-devel.so | awk "{print \$1}"' </dev/null 2>&1)"
}

deploy_kernel() {
    local local_md5
    local_md5=$(md5sum "$KIMG_LOCAL" | awk '{print $1}')
    log "scp kernel to device /uboot/uImage.LuneOS"
    ssh $SSH_OPTS $TOPAZ_HOST 'mount -o remount,rw /uboot' </dev/null 2>&1
    scp $SCP_OPTS "$KIMG_LOCAL" "$TOPAZ_HOST:/uboot/uImage.LuneOS" 2>&1 | tail -1
    local dev_md5
    dev_md5=$(ssh $SSH_OPTS $TOPAZ_HOST 'md5sum /uboot/uImage.LuneOS | awk "{print \$1}"' </dev/null 2>&1)
    if [ "$dev_md5" != "$local_md5" ]; then
        log "KERNEL DEPLOY md5 MISMATCH: dev=$dev_md5 local=$local_md5"
        return 1
    fi
    ssh $SSH_OPTS $TOPAZ_HOST 'echo LuneOS > /uboot/moboot.next; sync' </dev/null 2>&1
    log "kernel md5 on device: $dev_md5"
}

sysrq_reboot() {
    log "sysrq reboot"
    ssh $SSH_OPTS $TOPAZ_HOST '
        sync
        echo s > /proc/sysrq-trigger
        echo u > /proc/sysrq-trigger
        echo b > /proc/sysrq-trigger
    ' </dev/null 2>&1 | tail -1 || true
    sleep 5  # let kernel actually reboot before we look for the iface
}

usb_up_and_wait_ssh() {
    # The ECM iface comes back with a new name (random MAC) on every reboot,
    # may flicker during enumeration, and SSH only works after we've assigned
    # 172.16.42.1/24 + brought the link up + told NetworkManager to leave
    # it alone. Run the user's verbatim USB-up command every poll iteration
    # (idempotent) so that a newly-appearing iface gets configured
    # immediately - BEFORE we attempt SSH on this iteration.
    local attempt
    log "waiting for ECM iface + SSH (USB-up each iteration BEFORE SSH check)"
    for attempt in $(seq 1 90); do
        # User's verbatim USB-up command (literal sudo password)
        IFACE=$(ip link | grep -o 'enx[a-f0-9]*' | head -1) && \
            echo M0n1qu3@@ | sudo -S ip addr add 172.16.42.1/24 dev $IFACE 2>/dev/null; \
            echo M0n1qu3@@ | sudo -S ip link set $IFACE up 2>/dev/null; \
            echo M0n1qu3@@ | sudo -S nmcli device set $IFACE managed no 2>/dev/null;
        # SSH check AFTER USB-up
        if ssh $SSH_OPTS $TOPAZ_HOST 'true' </dev/null >/dev/null 2>&1; then
            log "ssh up after $((attempt*2))s on iface ${IFACE:-none}"
            return 0
        fi
        sleep 2
    done
    log "TIMEOUT: ssh never came back (iface=${IFACE:-none})"
    return 1
}

deploy_glcap_and_test() {
    local outdir="$1"
    log "deploy gl-cap-and-regdump-mainline + run 100-cap"
    scp $SCP_OPTS "$GLCAP" "$TOPAZ_HOST:/tmp/" 2>&1 | tail -1

    ssh $SSH_OPTS $TOPAZ_HOST "
        chmod +x /tmp/gl-cap-and-regdump-mainline
        pgrep -af 'surface-manager|LunaSysMgr' >/dev/null && echo 'WARNING: compositor running'
        mkdir -p /tmp/$outdir
        rm -f /tmp/$outdir/*
        > /tmp/$outdir/hashes.txt
        F0=\$(dmesg | grep -c 'GPUMMU fault')
        for i in \$(seq 1 100); do
            rm -f /tmp/cap.bin
            /tmp/gl-cap-and-regdump-mainline > /dev/null 2>&1
            if [ ! -s /tmp/cap.bin ]; then echo MISSING >> /tmp/$outdir/hashes.txt; continue; fi
            h=\$(md5sum /tmp/cap.bin | awk '{print \$1}')
            echo \"\$h\" >> /tmp/$outdir/hashes.txt
            [ -f /tmp/$outdir/sample-\$h.bin ] || cp /tmp/cap.bin /tmp/$outdir/sample-\$h.bin
        done
        F1=\$(dmesg | grep -c 'GPUMMU fault')
        echo
        echo \"unique samples: \$(ls /tmp/$outdir/sample-*.bin 2>/dev/null | wc -l)\"
        echo \"MMU faults: \$((F1-F0))\"
        echo 'hash freq:'
        awk '{print \$1}' /tmp/$outdir/hashes.txt | sort | uniq -c | sort -rn | head
        echo 'cycle:'
        awk '{
            h=substr(\$1,1,8)
            if (!(h in tag)) tag[h]=sprintf(\"%c\",65+n++)
            printf \"%s\",tag[h]
            if (NR%8==0) printf \"\\n\"
        }' /tmp/$outdir/hashes.txt | head -3
    " </dev/null 2>&1
}

# ----- main pipeline ---------------------------------------------------------

mode="${1:-}"
case "$mode" in
    mesa)
        prev_md5="${2:-}"
        outdir="${3:-r100auto}"
        bitbake_target mesa || exit 1
        new_md5=$(md5sum "$LIBGAL_LOCAL" 2>/dev/null | awk '{print $1}')
        if [ "$new_md5" = "$prev_md5" ] || [ -z "$new_md5" ]; then
            log "mesa md5 didn't change ($new_md5) — sstate cache reuse?"
            exit 1
        fi
        log "new mesa md5: $new_md5"
        # Ensure host USB iface up + device reachable BEFORE deploy attempt
        usb_up_and_wait_ssh || exit 1
        deploy_mesa
        sysrq_reboot
        usb_up_and_wait_ssh || exit 1
        deploy_glcap_and_test "$outdir"
        ;;
    kernel)
        prev_kver_short="${2:-}"
        outdir="${3:-r100auto}"
        # Update SRCREV_machine to the current git HEAD of our local kernel
        new_sha=$(git -C "$KERNEL_SRC" rev-parse HEAD)
        log "updating SRCREV_machine to $new_sha"
        sed -i -E "s|^SRCREV_machine = \"[a-f0-9]+\"|SRCREV_machine = \"$new_sha\"|" "$KRECIPE"
        grep "SRCREV_machine" "$KRECIPE"
        bitbake_target linux-hp-tenderloin || exit 1
        # Ensure host USB iface up + device reachable BEFORE deploy attempt
        usb_up_and_wait_ssh || exit 1
        new_kver=$(python3 -c "
import zlib, re, sys
with open('$ZIMG_LOCAL','rb') as f: data = f.read()
for m in re.finditer(b'\x1f\x8b\x08', data):
    try:
        d = zlib.decompress(data[m.start():], 15+32)
        idx = d.find(b'luneos-g')
        if idx > 0:
            print(d[idx:idx+30].decode('utf-8', errors='replace').split()[0])
            sys.exit(0)
    except Exception: continue
sys.exit(1)
")
        if [[ "$new_kver" == *"$prev_kver_short"* ]]; then
            log "kernel hash didn't change ($new_kver) — rebuild needed?"
            exit 1
        fi
        log "new kernel: $new_kver"
        deploy_kernel || exit 1
        sysrq_reboot
        usb_up_and_wait_ssh || exit 1
        deploy_glcap_and_test "$outdir"
        ;;
    both)
        prev_mesa="${2:-}"
        prev_kver_short="${3:-}"
        outdir="${4:-r100auto}"
        new_sha=$(git -C "$KERNEL_SRC" rev-parse HEAD)
        log "updating SRCREV_machine to $new_sha"
        sed -i -E "s|^SRCREV_machine = \"[a-f0-9]+\"|SRCREV_machine = \"$new_sha\"|" "$KRECIPE"
        bitbake_target mesa || exit 1
        bitbake_target linux-hp-tenderloin || exit 1
        usb_up_and_wait_ssh || exit 1
        deploy_mesa
        deploy_kernel || exit 1
        sysrq_reboot
        usb_up_and_wait_ssh || exit 1
        deploy_glcap_and_test "$outdir"
        ;;
    *)
        echo "usage: $0 {mesa|kernel|both} [args] [outdir]"
        exit 1
        ;;
esac
