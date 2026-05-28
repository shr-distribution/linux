# Novacom gadget — first hardware bring-up

**Kernel:** 6.18.0-luneos-gc5961d1a6a76 (post-`b510425298b5`
"usb: gadget: f_novacom: prep for mainline submission" +
`b7859c98609d` "tenderloin: enable CONFIG_USB_CONFIGFS_F_NOVACOM").

**Setup:** legacy `g_ether` was the bound gadget at boot. The
`reports/novacom-test/setup-gadget.sh` script unbound g_ether and rebuilt
the gadget under ConfigFS as an ECM + novacom composite (so SSH stays
alive over ECM while novacom is exercised). Bound to `ci_hdrc.0`.
ECM `host_addr` was pinned to the running interface MAC so SSH did not
notice the swap.

## Results

### T1 — enumeration ✅ PASS
`lsusb -v` (see `lsusb-v.txt`) on bus 3 dev 106:
- VID 1d6b:0104 (multifunction composite default, kept to coexist with ECM)
- Interface 2: class `ff` / subclass `47` / protocol `11`, two bulk
  endpoints, 512-byte HS MPS.
- `iInterface = 10` (non-zero — confirms the R1 `usb_gstrings_attach`
  plumbing reaches the descriptor).
- Three character devices appeared on the device:
  `/dev/novacom_ep0`, `/dev/novacom_ep_in`, `/dev/novacom_ep_out`.

### T2 — bulk loopback ✅ PASS (after driver fixes)

All 11 host-tester sizes round-trip byte-perfect:

```
[    48] OK    [   513] OK    [  4096] OK    [262144] OK
[    64] OK    [  1024] OK    [ 65536] OK    [262145] OK
[   128] OK    [   512] OK    [262143] OK
```

See `t2-pass-all-sizes.log`. This includes every exact-MPS-multiple
(512, 1024, 4096, 65536, 262144) — those used to be the hard case.

Three driver patches landed during T2 bring-up:
1. `537c00ae64f6` — split per-syscall I/O into MPS-aligned URBs so
   the chipidea UDC sees short-packet termination cleanly.
2. `13d34c89feaa` — on the OUT side, queue exactly one MPS URB per
   syscall and return what arrives (matches f_serial / f_loopback
   streaming semantics). The earlier multi-chunk OUT loop trapped
   itself waiting on data the host had no intention of sending,
   and conflated consecutive host transfers into one device read.
3. `0714ff309b09` — drop `req->zero` on IN. ZLPs from exact-MPS
   data writes overflowed into the host's *next* BULK IN URB and
   terminated it with actual=0. The userspace protocol (novacomd)
   pre-frames transfers so the host always reads back the exact
   length the device wrote — no ZLP terminator is needed.

The previous failure modes (driver wedged on short packets, conflated
transfers, ZLP overflow) are all captured in the intermediate logs:
`t2-rerun-with-chunking-only.log`,
`t2-rerun-chunking+zlp-still-conflate.log`,
`t2-mt-loopback-zlp-overflow.log`.

A multi-threaded device-side loopback (`loopback-mt.c`) is required
to exercise the largest sizes: the host-tester's sync pattern (full
bulk_OUT before bulk_IN) means the device-side must buffer the entire
host transfer before posting any IN data. Ring sized at 768×512 =
384 KiB to comfortably hold the 256 KiB+1 test case.

### T3 — ep0 event delivery ✅ PASS
`ep0-reader` capture (see `t3-ep0-events.log`) across a soft-disconnect
/ soft-reconnect cycle:

- `event type=SETUP bReq=0x09 wVal=0x0001` (SET_CONFIGURATION=1) emitted
  on `ep0_open` because the gadget was already CONNECTED.
- `event type=DISCONNECT` emitted on `echo disconnect > soft_connect`.
- `event type=SETUP bReq=0x09 wVal=0x0001` re-emitted on
  `echo connect > soft_connect` when the host re-enumerated.
- Signal teardown (kill -TERM on the reader) cleanly closes ep0 — read()
  returns `EBADF`, reader exits.

This confirms `novacom_next_event()`, `novacom_ep0_send_event()`, the
`ep0_open`/`ep0_release` lifecycle, and the `connect_state` machine.

### T4 — wrong-direction STALL ✅ PASS
On-device tester `wrong-dir.c` opens both data nodes and tries the
opposite direction. See `t4-t5-t6-t7.log`:

```
/dev/novacom_ep_in        read  -> -1 errno=74 (Bad message)  [PASS]
/dev/novacom_ep_out       write -> -1 errno=74 (Bad message)  [PASS]
```

dmesg confirms `usb_ep_set_halt()` fired on both endpoints:

```
novacom: ep_in  halt (wrong dir)
novacom: ep_out halt (wrong dir)
```

### T5 — interrupted read ✅ PASS
`interrupt-read.c` forks a child that sends SIGINT to the parent after
1 s while the parent is blocked in `read(/dev/novacom_ep_out, ...)`.
The read returned `-1 errno=EINTR` and dmesg confirmed the URB was
dequeued cleanly:

```
read -> -1 errno=4 (Interrupted system call)  [PASS]
novacom: ep_out i/o interrupted
```

### T6 — UDC soft-disconnect/reconnect ✅ PASS
`t6-soft-cycle.sh` writes `disconnect` then `connect` to
`/sys/class/udc/ci_hdrc.0/soft_connect`. dmesg shows the clean lifecycle:

```
novacom: deactivated
novacom: ep_out: NOVACOM_EP_DISABLED
novacom: ep_in:  NOVACOM_EP_DISABLED
novacom: activate
novacom: ep_in:  NOVACOM_EP_ENABLED
novacom: ep_out: NOVACOM_EP_ENABLED
```

After the cycle, `/tmp/probe` round-trips at 4 KiB and 65535 bytes
byte-perfect: data plane survives the cycle.

### T7 — ConfigFS unlink + rebuild ✅ PASS
`t7-cfgfs-cycle.sh` clears the UDC, unlinks the function from the
config, rmdirs the function dir, recreates it, re-links, and rebinds
the UDC. dmesg shows the full lifecycle:

```
novacom: deactivated
unbind function 'novacom'/f4725a99
adding 'novacom'/d67d0155 to config 'c'/6fbbabbc   <-- fresh ptr
novacom: activate
novacom: ep_in:  NOVACOM_EP_ENABLED
novacom: ep_out: NOVACOM_EP_ENABLED
```

After the cycle, `/tmp/probe` round-trips at 4 KiB / 65536 / 262143
byte-perfect.

Note: the /dev/novacom_* nodes persist throughout the unlink/rmdir
window because the function instance keeps the misc devices live until
all references drop. They get recreated implicitly on `mkdir
functions/novacom.0`. This matches the driver's lifecycle: misc
devices are registered in `novacom_alloc()` (= `usb_get_function()`,
fires on `ln -s`) and deregistered in `novacom_free()` (= when refcount
goes to zero on rmdir).

## Artifacts captured here

- `lsusb-v.txt` — full lsusb -v from host
- `dmesg-novacom.txt` — device-side novacom dmesg trail across T1+T2

## Reproduction recipe

Device-side (after deploy):
```
sh /tmp/setup-gadget.sh                  # one-time gadget swap
/tmp/lb-test 65536 &                     # OR /tmp/loopback-512 &
```

Host-side:
```
sudo chmod 666 /dev/bus/usb/<bus>/<dev>  # one-time, see lsusb
/tmp/host-tester                         # full battery
/tmp/probe <size>                        # single transfer
```
