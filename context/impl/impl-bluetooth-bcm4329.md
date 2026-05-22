---
domain: bluetooth-bcm4329
created: "2026-05-21"
last_updated: "2026-05-22"
status: skip-sync-patch-landed-pending-on-device-verification
---

# Implementation: Bluetooth BCM4329 (BCSP over GSBI6 UART)

## Status: Driver mostly implemented, blocked on module deployment

The HP TouchPad's BCM4329 BT chip is wired via UART on GSBI6, NOT
the standard mainline H4 protocol — it uses the **BCSP (BlueCore
Serial Protocol)** which is SLIP-framed with retransmission. The
mainline `hci_bcm.c` (H4) is **incompatible**.

A custom Palm/HP-specific driver path has been built up in
`drivers/bluetooth/hci_bcsp.c` (2724 lines) with the PSKEY tables
in `drivers/bluetooth/hci_bcsp_touchpad_pskeys.h` (283 lines). It
supports both line-discipline (legacy `hciattach`) and serdev modes;
serdev is the target for upstream-style DT integration.

## Driver implementation status

### Recent iteration history (last ~10 commits, May 2026)

```
fd0a8d27192a  hci_bcsp: Silence per-packet / per-timer-tick BT_INFO spam
43a1c03149d3  hci_bcsp: Disable CRC for LE, enable flow control
b8dec32adc48  hci_bcsp: Fix CRC and race condition for BCM4329
b2f89d483a27  hci_serdev: Add verbose TX/RX debug logging
0bb0406bbb56  hci_bcsp: Disable flow control for link establishment
106a58cc063b  hci_bcsp: Disable CRC for Link Establishment packets
fbdc9ec115b9  hci_bcsp: Fix serdev detection for PSKEY wait
b5614230ff53  hci_bcsp: Fix DT node detection for palm,bcm4329-bcsp
60cd700d771a  hci_bcsp: Enable CRC for all BCSP packets including LE
e2c6227e0ac2  hci_bcsp: Enable hardware flow control for serdev
```

Driver is **actively iterated** — most recent fix May 19, 2026. The
ground-truth analysis report is `reports/bcm4329-bluetooth-analysis.md`.

### What works (per analysis report)

- BCSP link establishment via `hciattach`
- BD address configuration via kernel driver (`bdaddr` parameter)
- HCI device registration (`hci0` appears)
- 12 critical PSKEYs sent before BDADDR
- TX_POWER_LEVEL table (60 words extracted via Ghidra from webOS
  `libPmBtBsaif.so` — RF calibration data)

### What doesn't work / not yet verified

- **Device discovery/scanning** — TouchPad not visible to other devices
- **RF transmission** — may now work with TX_POWER_LEVEL table; needs test
- Of `~50 PSKEYs` that webOS's `bcattach` sends, ~37 may still be
  missing if optional ones turn out to be required

## Architecture

### Hardware

| Item | Value |
|------|-------|
| Chip | Broadcom BCM4329 (combo WiFi+BT) |
| UART | GSBI6, base `0x16540000` |
| Baud | 115200 default, supports up to 3686400 |
| Protocol | BCSP (SLIP-framed, 0xC0 delimiters) |
| HOST_WAKE | GPIO 129 (input, active high) |
| BT_PWR | GPIO 130 (output, active high) |
| BT_WAKE | GPIO 131 (output, active high) |
| BT_RST | GPIO 138 (output, active low) |

### DT node (tenderloin)

```dts
&gsbi6_serial {
    bluetooth {
        compatible = "palm,bcm4329-bcsp";
        local-bd-address = [A9 64 85 FE 1D 00];  /* webOS BToADDR */
        shutdown-gpios = <&tlmm 130 GPIO_ACTIVE_HIGH>;     /* BT_PWR */
        device-wakeup-gpios = <&tlmm 131 GPIO_ACTIVE_HIGH>; /* BT_WAKE */
        reset-gpios = <&tlmm 138 GPIO_ACTIVE_LOW>;         /* BT_RST_N */
        ...
    };
};
```

Custom `palm,bcm4329-bcsp` compatible was needed because the generic
`brcm,bcm4329-bt` matches `hci_bcm.c` (H4 protocol) which the chip
doesn't speak.

## Current blocker: BCSP SYNC handshake timeout

After deploying the `modules-tenderloin.tgz` tarball from Yocto's
deploy/images/tenderloin/ directory and `modprobe`-ing bluetooth +
hci_uart + btbcm, the BT subsystem comes up far enough that:

| Check | Result |
|-------|--------|
| Modules loaded | ✅ `bluetooth`, `hci_uart`, `btbcm` |
| `hci0` registered | ✅ `Type=Primary Bus=UART` |
| GPIO 130 (BT_PWR) | ✅ OUT high — chip powered |
| GPIO 131 (BT_WAKE) | ✅ OUT high — host wake-up asserted |
| GPIO 138 (BT_RST_N) | ✅ OUT high — chip NOT in reset (active-low) |
| GPIO 129 (HOST_WAKE) | ✅ IN low pull-down — chip-to-host input |
| Driver bound | ✅ `hci_uart_bcsp` on `serial0-0` |
| Probe info line | `BCM4329 BCSP Bluetooth, init-speed=115200` |
| DT PSKEYs loaded | ✅ 13 PSKEYs + TX_POWER_LEVEL table |
| DT BD addr | ✅ `00:1d:fe:85:64:a9` parsed |
| Baud setup | ✅ 115200 |
| Flow control | ✅ HW flow control enabled |
| Initial sync TX | ✅ Sent — `BCSP: Sent initial sync to wake chip` |

But:

```
Bluetooth: BCSP: Serdev mode - waiting for link establishment
Bluetooth: BCSP: Timeout waiting for link establishment   (5 s later)
```

RX byte counter is increasing (~1.4 KB / few seconds) so SOMETHING
comes back from the chip — but the BCSP layer doesn't decode it as
a valid `sync_resp`. The bring-up hangs at the BCSP link
establishment phase.

`hciconfig hci0 up` returns `Connection timed out (110)` because
the kernel never sees the chip transition to a usable HCI link.

### Why modules weren't deployed initially

On first investigation `/lib/modules/<running_kernel>/` was empty
on the device. The Yocto build had produced the modules
(`modules-tenderloin.tgz` in `tmp-glibc/deploy/images/tenderloin/`)
but they weren't being installed into the running image — separate
deployment step. Once the tarball was scp'd to the device and
extracted at `/`, the modules became loadable.

This is a **Yocto image-install issue**, separate from the kernel
work. Tracked as a future item: confirm the recipe installs the
modules at build time so manual scp+tar isn't needed.

## Root cause found via legacy wire trace (2026-05-22)

A debug-instrumented legacy webOS kernel
(`/uboot/uImage.webOSdebug`, md5 `dcfdd89ff56b4fe55262bc7dd322f29f`)
with `print_hex_dump` injected into `drivers/misc/hsuart.c`'s
`hsuart_copy_buf_to_user` (RX path) and `hsuart_copy_user_to_buf`
(TX path) captured the full BCSP wire exchange between webOS userspace
and the BCM4329 chip over `/dev/bt_uart`. Trace saved at
`reports/bt-trace/webos-bcsp-only-2026-05-22.log`.

### Key observation: webOS skips BCSP link establishment / SYNC

The very first TX from webOS userspace after BT power-up is a real
BCCMD (BlueCore Command) packet with payload `02 04 00`, NOT a BCSP
SYNC packet:

```
[  101.253647] TX> c0 da 35 00 f0 02 04 00 75 18 c0     <- first host TX
[  101.259137] RX< c0 60 00 00 9f dd 6f c0              <- chip's ack-only response
```

The chip responds with a normal BCSP ack-only frame. Subsequent
traffic is all BCCMD/PSKEY/HCI wrapped in BCSP framing
(`0xC0 ... 0xC0` SLIP delimiters). Full sequence of ~13 BT packets
captured.

A canonical BCSP SYNC packet would be `c0 da dc ed ed c0` (raw
`01 7e` after SLIP-escaping the 0xC0 inside the payload). **No such
packet appears** anywhere in the webOS trace.

### Implication for mainline driver

The BCM4329 on the HP TouchPad is **already in BCSP mode** at chip
power-up — likely a factory PSKEY setting in the BT chip's NVRAM.
webOS userspace skips link-establishment SYNC and goes straight to
configured-link state, assuming `txseq=0` / `rxack=0` align with the
chip.

Our mainline driver `drivers/bluetooth/hci_bcsp.c` insists on doing
the SYNC handshake first (the "initial sync to wake chip" we saw on
on-device test). The chip never returns a `sync_rsp` because it
doesn't need to — it's already past that state. Our driver times out
at the SYNC phase and never reaches operational state.

### Action plan

Add a DT property `qcom,bcsp-skip-sync;` (or a serdev driver flag)
to the existing `bluetooth { compatible = "palm,bcm4329-bcsp"; ... }`
node. When set, the driver skips:
  - sync_req TX
  - waiting for sync_rsp
  - the BCSP `LINK_UNINIT` → `LINK_INIT` transition

and starts directly in the operational state with `txseq=0` /
`rxack=0`. The standard ack/seq flow then handles the configured
link from packet #1 forward.

This is a small, contained patch — probably 30-50 lines plus 1 DT
property. Test by booting LuneOS, modprobing the BT modules, and
checking if `hciconfig hci0 up` succeeds (which it currently times
out at).

## Active investigation: BCSP SYNC (closed)



The recent commit stream (May 2026) shows the driver author
iterating on exactly this issue:

```
fd0a8d27192a  hci_bcsp: Silence per-packet / per-timer-tick BT_INFO spam
43a1c03149d3  hci_bcsp: Disable CRC for LE, enable flow control
b8dec32adc48  hci_bcsp: Fix CRC and race condition for BCM4329
0bb0406bbb56  hci_bcsp: Disable flow control for link establishment
106a58cc063b  hci_bcsp: Disable CRC for Link Establishment packets
fbdc9ec115b9  hci_bcsp: Fix serdev detection for PSKEY wait
b5614230ff53  hci_bcsp: Fix DT node detection for palm,bcm4329-bcsp
60cd700d771a  hci_bcsp: Enable CRC for all BCSP packets including LE
e2c6227e0ac2  hci_bcsp: Enable hardware flow control for serdev
```

The pattern (CRC on/off cycle, flow-control on/off cycle) suggests
the SYNC handshake interacts with serial framing in ways that
weren't fully nailed down at the last commit. Next debugging step
would be to enable `serdev_debug=1` modparam and add dynamic-debug
for `hci_bcsp` to see the raw TX/RX bytes, then compare against
webOS `bcattach`'s wire trace.

This is **an in-flight driver effort**, not a fresh investigation
to start from scratch. Continuing it requires understanding the
existing commit chain.

## 2026-05-22: Skip-SYNC patch + DT opt-in

webOS hsuart wire trace confirms the host never sends a BCSP SYNC
packet — it issues HCI traffic immediately and the chip ACKs. BT
on/off cycles also skip SYNC; only a WARM_RESET BCCMD is sent.

Conclusion: the BCM4329 ships with `PSKEY_HOST_INTERFACE = BCSP`
persistently in EEPROM, and the chip starts in BCSP-operational
state (txseq=0/rxack=0) at every power-on. Driving SYNC from the
host confuses the chip → our mainline driver times out at link
establishment.

Implemented:

- `drivers/bluetooth/hci_bcsp.c`:
  - New `skip_sync` bool in `bcsp_struct`.
  - `bcsp_read_pskeys_from_dt()` reads `qcom,bcsp-skip-sync` from
    the BCSP node and sets `skip_sync`.
  - `bcsp_open()` reordered: PSKEY DT load now happens before the
    sync packet would be queued. When `skip_sync` is set, the driver
    forces `link_state = BCSP_LINK_ACTIVE`, marks `link_established`,
    signals `link_up`, and does **not** arm `tbcsp` or queue a sync
    frame. `bcsp_setup()` (which waits on `link_up`) proceeds
    straight to the PSKEY replay path.
  - `bcsp_timed_event()` defensively bails on sync/conf TX when
    `skip_sync` is set (stale timer protection).
  - `bcsp_setup()` PENDING-path: after WARM_RESET, when `skip_sync`
    is in effect we explicitly call `bcsp_serdev_power_cycle()` +
    reset seq numbers/queues. The chip-SYNC-after-reset path that
    normally triggers this won't fire because the chip stays in
    BCSP-operational state across WARM_RESET.

- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`:
  added `qcom,bcsp-skip-sync;` on the `bluetooth { compatible =
  "palm,bcm4329-bcsp"; }` subnode.

PSKEY replay path is unchanged — the existing in-driver sequence
(Common + Palm Platform + TX power table + BD address + WARM_RESET)
already mirrors the libPmBtBsaif boot sequence. The webOS
BT-off/on trace captured a *different* set of PSKEYs (0x17/0x1d/
0x21/0x27-2b/0x31/0x39/0x3a/0xb3/0xb6/0xba/0xbf/0xc7/0xca/0xe1/
0xf7/0xf8), but those run on top of the boot configuration that
the legacy ROM flashes once at first power-on — not at every
chip wakeup. Mainline replays the cold-boot sequence; the on/off
delta can be layered later if needed.

Expected next state on-device:
- No `BCSP: Timeout waiting for link establishment` error.
- `BCSP: skip-sync — link forced ACTIVE, no SYNC sent` in dmesg.
- PSKEY+WARM_RESET sent.
- `bdaddr_state = BCSP_BDADDR_DONE` reached.
- HCI core issues HCI Reset / Read Local Version → chip responds.
- `/sys/class/bluetooth/hci0` appears, `hciconfig hci0 up` works.

## Recommendations

1. **Switch defconfigs to `BT=y` / `BT_HCIUART=y`** as the immediate
   unblock. This lets us test the driver iterations that have already
   landed.

2. **After driver verified working on-device**, revisit modularity
   (or document why this device ships built-in BT).

3. **Continue the PSKEY tuning** that was in flight at fd0a8d27192a
   et al. The TX_POWER_LEVEL table is the latest critical addition;
   need to verify it makes the chip discoverable + transmittable.

4. **Coordinate with WiFi** — BCM4329 is a combo chip. If WiFi work
   is also touching the chip-side init, BT may be affected.

## Cross-references

- **Driver**: `drivers/bluetooth/hci_bcsp.c` (2724 lines)
- **PSKEY tables**: `drivers/bluetooth/hci_bcsp_touchpad_pskeys.h`
- **Analysis report**: `reports/bcm4329-bluetooth-analysis.md`
- **DT node**: `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi`
  (`&gsbi6_serial { bluetooth { compatible = "palm,bcm4329-bcsp"; ... } }`)
- **Legacy reference**: `webos-linux-kernel-touchpad/...` (BCSP not in
  legacy mach-msm — webOS used userspace `bcattach` over hci_uart
  line-discipline, not a kernel BCSP driver)
- **Build-system question**: open — see Path B above. Likely lives in
  the meta-mainline kernel recipe.

## Cavekit / kit status

No `cavekit-bluetooth-bcm4329.md` exists yet. Should be written if
the BT work continues — would track:
- R1: BT subsystem initialises (`/sys/class/bluetooth` exists)
- R2: BCSP link established with chip
- R3: PSKEYs + WARM_RESET sequence completes
- R4: Device discoverable + can connect
- R5: Data transfer (e.g. A2DP audio over BT)

Out of scope for this impl tracking pass.
