# Novacom gadget — bring-up & test runbook

This walks through getting `f_novacom` enumerated and exercised on a
TouchPad running mainline 6.18. None of this has hit hardware yet — every
step is also a verification step.

## 0. Pre-flight

- Defconfigs (`tenderloin_defconfig`, `tenderloin_debug_defconfig`,
  `tenderloin_fast_defconfig`) all carry `CONFIG_USB_CONFIGFS_F_NOVACOM=y`.
  That auto-selects `USB_F_NOVACOM` (the function driver) and pulls
  `USB_CONFIGFS` (already on).
- Yocto rebuild + deploy of `uImage.LuneOS` is required after picking up
  the kernel commits.
- LuneOS already runs the ECM gadget at 172.16.42.2 during initramfs. We
  need to stop that gadget before binding a novacom-only one (the UDC
  can only carry one composite at a time without a multi-function config).

## 1. T1 — enumeration smoke

On device, after fresh boot and SSH login:

```
# Inspect current UDC binding
cat /sys/class/udc/*/state
ls /sys/class/udc/                         # expect: ci_hdrc.0

# Stop the existing gadget userspace (whatever brought up usb0).
# On stock LuneOS this is the netadapter that drives ECM.
ifdown usb0 2>/dev/null
echo "" > /sys/kernel/config/usb_gadget/g1/UDC 2>/dev/null

# Build the novacom gadget
modprobe usb_f_novacom
mkdir -p /sys/kernel/config/usb_gadget/g_nova
cd /sys/kernel/config/usb_gadget/g_nova

echo 0x0830 > idVendor
echo 0x8002 > idProduct
echo 0x0100 > bcdDevice

mkdir -p strings/0x409
echo HP             > strings/0x409/manufacturer
echo "webOS Device" > strings/0x409/product
echo touchpad-01    > strings/0x409/serialnumber

mkdir -p configs/c.1
mkdir -p configs/c.1/strings/0x409
echo "Novacom"      > configs/c.1/strings/0x409/configuration

mkdir functions/novacom.0
ln -s functions/novacom.0 configs/c.1/

ls /dev/novacom_*                          # three nodes expected
echo ci_hdrc.0 > UDC
```

Replug USB to host. On host:

```
lsusb -d 0830:8002 -v 2>&1 | tee novacom-enum.txt
```

**Pass criteria (T1):**

- `lsusb` lists the device with `idVendor 0x0830`, `idProduct 0x8002`.
- The interface descriptor shows class `ff`, subclass `47`, protocol `11`.
- Two bulk endpoints, one IN one OUT, each 512-byte wMaxPacketSize on HS.
- `iInterface` is non-zero and dereferences to the string `Novacom` (R1).
- On device, `dmesg | grep novacom` shows `novacom: IN/<ep> OUT/<ep>`.

## 2. T2 — bulk loopback

Build the artifacts:

```
# On dev workstation (cross-compile or scp + compile on device)
arm-linux-gnueabihf-gcc -O2 -Wall \
    -o reports/novacom-test/loopback \
       reports/novacom-test/loopback.c

# On host
cc -O2 -Wall -o reports/novacom-test/host-tester \
       reports/novacom-test/host-tester.c -lusb-1.0
```

Deploy `loopback` to device (`scp` to `/usr/local/bin/`). With the gadget
bound (T1 complete), on device:

```
/usr/local/bin/loopback &
```

On host:

```
sudo reports/novacom-test/host-tester | tee novacom-t2.log
```

**Pass criteria (T2):**

- Every size in `test_sizes[]` reports `OK` with zero data mismatch.
- The 256 KiB + 1 size returns OK with the host sending 256K+1 bytes total
  (driver caps each `write()` to 256 KiB → loopback iterates → host's
  matching IN reads see the full 256K+1 because the kernel only caps
  per-syscall, not per-USB-transfer).
- `loopback` final `in=`/`out=` counters match the sum of host transfers.

## 3. T3 — ep0 event reader

```
# On device, in a separate console
/usr/local/bin/ep0-reader &
```

Trigger events from host:

```
# Disconnect via soft-disconnect
echo "" > /sys/kernel/config/usb_gadget/g_nova/UDC      # on device
echo ci_hdrc.0 > /sys/kernel/config/usb_gadget/g_nova/UDC
```

Or pull/replug the cable.

**Pass criteria (T3):**

- A `DISCONNECT` event appears on disconnect.
- A `SETUP bReq=0x09 wVal=0x0001` appears on each re-binding (i.e.
  `SET_CONFIGURATION` with value 1).

## 4. T4 — wrong-direction halt

Covered automatically by `host-tester.c::test_wrong_dir_stall`. Verifies
that a bulk IN on the OUT endpoint address produces a STALL.

**Pass criteria (T4):** the tester reports `[wrong-dir] OK (STALL)`.

## 5. T5 — interrupted I/O

```
# On device — make a read block and then signal it
sleep 60 < /dev/novacom_ep_out &
SLEEP_PID=$!
sleep 1
kill -INT $SLEEP_PID
wait $SLEEP_PID; echo "exit=$?"
```

**Pass criteria (T5):** the `sleep` exits promptly (within ~1 s); no
kernel oops; `dmesg` shows the `novacom: %s i/o interrupted` line.

## 6. T6 — UDC soft-disconnect / reconnect

```
echo "" > /sys/kernel/config/usb_gadget/g_nova/UDC
sleep 1
echo ci_hdrc.0 > /sys/kernel/config/usb_gadget/g_nova/UDC
```

**Pass criteria (T6):**

- Host re-enumerates the device.
- `dmesg` on device shows novacom `reset` then `activate`.
- Re-run `host-tester` afterwards; still passes.
- No `WARN` or `BUG` in dmesg across the cycle.

## 7. T7 — configfs rmdir / re-mkdir

```
echo "" > /sys/kernel/config/usb_gadget/g_nova/UDC
rm /sys/kernel/config/usb_gadget/g_nova/configs/c.1/novacom.0
rmdir /sys/kernel/config/usb_gadget/g_nova/functions/novacom.0
ls /dev/novacom_*                                # all three gone

mkdir /sys/kernel/config/usb_gadget/g_nova/functions/novacom.0
ln -s ../../functions/novacom.0 \
      /sys/kernel/config/usb_gadget/g_nova/configs/c.1/novacom.0
ls /dev/novacom_*                                # back
echo ci_hdrc.0 > /sys/kernel/config/usb_gadget/g_nova/UDC
```

Re-run T2. **Pass criteria (T7):** clean teardown, /dev nodes vanish then
re-appear, T2 still passes after rebind, kmemleak (if enabled) reports
no new leaks.

## 8. Real novacomd (optional, end-to-end)

Skip on first pass — the kernel side is what we care about for the
mainline submission. Once T1–T7 are green and we have an interest in
validating against the actual host daemon:

```
# On device
systemctl stop usb-gadget-ecm           # or whatever brings up usb0
/sbin/novacomd

# On host
novacom -l           # should list the TouchPad
novacom run file:///bin/sh
```

## 9. What to capture

For the cover-letter / patchset README, save under
`reports/fix-verified/novacom-bringup-<date>/`:

- `novacom-enum.txt` — `lsusb -v` output from T1
- `novacom-t2.log` — `host-tester` output
- `novacom-t3-events.log` — `ep0-reader` output across a connect/disconnect
- `dmesg-novacom.txt` — `dmesg | grep -i -E 'novacom|usb|gadget'` after
  the full run
- `usbmon-t2.bin` — optional: `cat /sys/kernel/debug/usb/usbmon/0u > t2.bin`
  while T2 runs, useful for low-level packet-by-packet review

## Known things that will NOT be tested by this runbook

- SuperSpeed — descriptors removed; no SS UDC on tenderloin anyway.
- Multi-instance — singleton enforced; `mkdir functions/novacom.1` will
  return `-EBUSY`. That is verified implicitly by ConfigFS but worth
  noting in the patchset.
- `g_novacom` legacy gadget — left out of defconfigs. Test only if the
  legacy path is actually requested by a reviewer.
