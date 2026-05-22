---
domain: bluetooth-bcm4329
created: "2026-05-21"
last_updated: "2026-05-22"
status: CRC fix verified (TX byte-perfect) but chip RX still dead; reset-pulse+1s-settle fix pushed (8054d7305229) pending rebuild
---

## 2026-05-22 (evening): CRC fix VERIFIED working at TX level; chip RX still dead → reset-timing fix (8054d7305229)

Rebuilt with CRC fix (kernel g196cf9e5f841, includes 3a7ac6dd0f58).
On-device: our SYNC is now `c0 40 41 00 7e da dc ed ed a9 7a c0` —
**byte-identical to the chip's SYNC and to webOS bcattach's**. The CRC
fix worked. But the chip STILL only streams SYNC, never SYNC-RSP.

**Exhaustive on-device verification — everything on our side is correct:**
- GPIOs (TLMM io regs via devmem): BT_POWER(130)=high, BT_WAKE(131)=high
  (asserted, ctl_reg 0x2C0 = GPIO mode + OE + 8mA), BT_RST_N(138)=high
  (not in reset), HOST_WAKE(129)=low.
- GSBI6 UART (devmem 0x16540000): MR1=0x34 → CTS_CTL(bit6)=0, no hardware
  flow-control gating. MR2=0x34 → 8N1. SR=0xAC → TXRDY+TXEMT (TX FIFO and
  shifter EMPTY = our bytes physically clocked out the pin).
- 1902/1902 RX payloads = `da dc ed ed` (SYNC), ZERO framing/checksum/
  short/unknown errors. We are not dropping anything; the chip simply
  sends nothing but SYNC.

So: chip TX healthy, our TX byte-perfect and physically transmitted, no
flow-control gating, RX clean — yet the chip's UART RX never acts on our
bytes. flow-control falsified (again), CRC confirmed-but-insufficient.

**Remaining difference from webOS = the bring-up sequence.** webOS
bcattach (webOS-ports/utilities tenderloin-halium/bcattach/main.c):
- reset: write 0 (assert), `usleep(100000)` (100 ms hold), write 1
  (deassert), `usleep(1000000)` (**1 s settle**), THEN start UART traffic.
- UART flags=0x9 (0x4 flow-control bit clear → no flow control).
- no wake bytes; goes straight to SYNC after the 1 s.

Our `bcsp_serdev_set_power(true)` deasserted reset with NO hold (asserted
at devm_gpiod_get, released ~µs later) and waited only msleep(100) before
`hci_uart_register_device()` → bcsp_open queued the first SYNC. So the
chip got essentially no reset pulse AND we transmitted during its boot
window. Either under-resets the chip / corrupts its early RX framing,
leaving RX dead while the ROM SYNC TX runs autonomously — matches the
symptom and its determinism.

**Fix (8054d7305229):** set_power(true) now does power-on → assert reset
100 ms → deassert → msleep(1000) before returning. Initial SYNC is queued
only after set_power completes, so we stay silent for ~1 s during boot,
matching webOS.

Caveat: a UART normally resyncs framing after idle, so "premature SYNC
corrupts RX forever" has a weakness; the under-reset (µs pulse) angle is
the stronger mechanism. If this fix doesn't work, next suspects:
msm_serial GSBI6 TX-path vs the webOS hsuart driver (confirm msm_serial
TX is proven on another GSBI UART), and the hsuart flags=0x9 / RXLAT
specifics.

Expected next-state after rebuild: chip emits SYNC-RSP (`ac af ef ee`) →
`sync_rsp received, moving to INIT` → conf → `Link established` → PSKEY/
BDADDR/WARM_RESET → HCI Reset OK → `hciconfig hci0 up`.

### Cheap check (still BLOCKED on link-up) — psmemtype / chiprev / psget 0x01f9
Unchanged; needs a working BCSP link to send BCCMDs.

---

## 2026-05-22 (later): flow-control was a red herring — real blocker is CRC on LE packets (3a7ac6dd0f58)

Rebuilt with the flow-control fix (kernel g83025300533e, includes
3149a6f9c1ab). On-device retest: dmesg shows
`BCSP: Hardware flow control disabled (webOS FLOW_CTRL_NONE)` but the
handshake behaviour is **identical** to before — chip loops SYNC, never
sends SYNC-RSP, `Timeout waiting for link establishment`. So disabling
flow control changed nothing; it was NOT the blocker. (Kept anyway —
webOS uses FLOW_CTRL_NONE, so it's the correct webOS-matching state.)

The wire diff is the real smoking gun:
- chip SYNC:  `c0 40 41 00 7e da dc ed ed a9 7a c0`  (byte0=0x40 = CRC bit + trailing CRC)
- our SYNC:   `c0 00 41 00 be da dc ed ed c0`         (byte0=0x00 = no CRC)

`bcsp_prepare_pkt()` had `pkt_crc = (chan != 1) && use_crc` — it stripped
CRC from channel-1 (link establishment) packets on a "LE without CRC per
spec" assumption. Wrong for this CSR chip: it drops our non-CRC
SYNC/SYNC-RSP, never sees our SYNC, never replies.

**webOS bcattach wire trace (github webOS-ports/utilities,
tenderloin-halium/bcattach/main.c) is authoritative** — it sends every LE
packet WITH CRC:
- SYNC:     `c0 40 41 00 7e da dc ed ed a9 7a c0`
- SYNC-RSP: `c0 40 41 00 7e ac af ef ee bb 84 c0`
With CRC on, our SYNC becomes byte-identical to the chip's.

**Fix (3a7ac6dd0f58):** drop the `(chan != 1)` exclusion →
`pkt_crc = bcsp->use_crc` (txcrc default true). Matches mainline
hci_bcsp + webOS. RX path already validated the chip's CRC'd packets, so
only TX needed it.

Expected next-state after Yocto rebuild + module redeploy:
- chip emits SYNC-RSP (`ac af ef ee`) → `sync_rsp received, moving to INIT`
- conf exchange → `Link established` → PSKEY+BDADDR+WARM_RESET →
  re-handshake → HCI Reset succeeds → `hciconfig hci0 up`, events > 0

### Cheap check (still BLOCKED on link-up) — do once BCSP link is up
The CSR persistence/H4 question needs a working link to send BCCMDs.
Once `hciconfig hci0 up` works, read (via our BCCMD machinery or a
`bccmd` binary if LuneOS ships one):
- `psmemtype` — Flash(0x00)/EEPROM(0x01) ⇒ persistent H4 possible;
  RAM(0x02)/ROM(0x03) ⇒ volatile, drop the H4 idea
- `chiprev` / `chipver` — exact BlueCore part
- `psget 0x01f9` — firmware's actual BCSP value (to know the H4 value)

---

## 2026-05-22 (late): HW flow control was gating host TX — fix pushed (3149a6f9c1ab)

Kernel `g8165758913da` (includes fc36dcf860b7) deployed via Yocto;
BT modules extracted from `modules-tenderloin.tgz` (matching version)
and scp'd to `/lib/modules/$(uname -r)`, `depmod -a`, modprobe
btbcm + hci_uart. On-device dynamic-debug (`modprobe hci_uart dyndbg=+p`)
captured the real handshake state:

```
BCSP: TX LE pkt: c0 00 41 00 be da dc ed ed c0   ← our SYNC
BCSP: RX LE payload: da dc ed ed                  ← chip SYNC
BCSP: sync received
BCSP: sending sync_rsp
BCSP: TX LE pkt: c0 00 41 00 be ac af ef ee c0    ← our SYNC_RSP
(chip keeps sending SYNC forever, NEVER sends SYNC_RSP back)
```

fc36dcf860b7 worked as intended on the *response* side — we now answer
the chip's SYNC with SYNC_RSP (no more skip-sync ignoring). But the chip
never ACKs **our** SYNC, so it loops in BCSP_LINK_UNINIT. In BCSP both
peers send SYNC and each emits SYNC_RSP on *receiving* the peer's SYNC.
The chip not emitting SYNC_RSP = it is not receiving our TX. chip→host
RX is clean at 115200, so this is a host→chip TX stall.

**Root cause:** `bcsp_open()` called `serdev_device_set_flow_control(true)`
→ CRTSCTS. The msm UART gates TX on the chip's CTS line, and the BCM4329
holds CTS deasserted until it has been talked to (chicken-and-egg). So
every host TX (incl. our SYNC_RSP) stalls in the UART FIFO and never
reaches the chip.

The driver comment ("WebOS used UART_WITH_FLOW_CONTROL for GSBI6")
misread the legacy board file. `board-tenderloin.c btuart_data` sets
`.uart_mode = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE`.
`UART_WITH_FLOW_CONTROL` passed to `board_gsbi6_init()` only muxes the
RTS/CTS *pins* as GSBI6 function (our DT already muxes gpio53-56);
it does NOT turn on CRTSCTS. webOS managed RTS manually via
`p_board_rts_pin_deassert_cb` and never CTS-gated TX.

**Fix (3149a6f9c1ab):** `bcsp_open()` now calls
`serdev_device_set_flow_control(hu->serdev, false)`. hci_serdev already
leaves it off (hci_serdev.c:394, runs before proto->open); we just stop
flipping it back on.

Expected next-state after Yocto rebuild + module redeploy:
- `BCSP: Hardware flow control disabled (webOS FLOW_CTRL_NONE)` in dmesg
- chip emits SYNC_RSP (`ac af ef ee`) → `BCSP: sync_rsp received, moving
  to INIT state` → conf exchange → `BCSP: Link established (first time)`
- bcsp_setup PSKEY+BDADDR+WARM_RESET → re-handshake → HCI Reset succeeds
- `hciconfig hci0 up` works, BD `00:1d:fe:85:64:a9`, events > 0

**Module deploy note (Yocto image-install gap persists):** the running
image still ships with no BT modules under `/lib/modules/$(uname -r)`.
Workflow that works: extract `tmp-glibc/deploy/images/tenderloin/
modules-tenderloin.tgz` (symlink → version matching `uname -r`), scp the
`lib/modules/<rel>` dir to device, `depmod -a`, modprobe.

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

## 2026-05-22 (evening): skip-sync hypothesis FALSIFIED — chip needs handshake (fc36dcf860b7)

After deploying the double-power-cycle fix the chip got past PSKEY
config + WARM_RESET cleanly, but HCI Reset (opcode 0x0c03) timed
out -110. dmesg showed `events:0` on `hciconfig -a` despite RX
bytes incrementing.

With `hci_bcsp.c +p` dynamic-debug enabled the truth came out:

```
BCSP: RX LE hdr: 40 41 00 7e
BCSP: RX LE payload: da dc ed ed     ← SYNC magic
BCSP: skip-sync — ignoring LE packet ...
(repeats indefinitely)
```

The chip emits SYNC packets continuously after WARM_RESET and our
skip-sync code dutifully ignores them. The chip never reaches
operational state; nothing gets decoded as HCI events.

**Root cause of the bad hypothesis:** the webOS hsuart wire trace
(`reports/bt-trace/webos-bcsp-only-2026-05-22.log`) was captured
*after* userspace `bcattach` had already done the handshake. The
first TX in the trace (`c0 da 35 00 f0 02 04 00 ...`) has BCSP
header byte = `0xda` = seq 2 ack 3, not seq 0 ack 0. So the trace
tells us nothing about the chip's state at fresh power-up — only
its steady-state.

**Fix (commit `fc36dcf860b7`):**

- DT: drop `qcom,bcsp-skip-sync` from the bluetooth subnode in
  `qcom-apq8060-tenderloin-common.dtsi`.
- `hci_bcsp.c`:
  - Sync handler: remove the GPIO power-cycle path (PSRAM-PSKEY
    wipe). Reset our seq/queues, set `link_state = UNINIT`,
    `link_established = false`, `reinit_completion(link_up)`, then
    fall through to send SYNC_RSP and let the standard state
    machine drive CONF / CONF_RSP / LINK_ACTIVE.
  - `bcsp_setup()`: reinit `link_up` + clear `link_established`
    *before* sending WARM_RESET (so we don't race with the sync
    handler's reinit). After WARM_RESET, replace the
    `msleep(1500)` with `wait_for_completion_timeout(link_up, 5 s)`
    — only return once the re-handshake actually finished. Without
    this, HCI core's first command (HCI Reset) races the
    in-progress handshake and times out -110.
  - Drop the now-dead skip-sync early-return at the top of the LE
    handler. The `skip_sync` field is still in the struct (for some
    hypothetical chip that genuinely boots operational), but no DT
    sets it on tenderloin.

Expected next-state on-device after Yocto rebuild:

- Initial probe: `BCSP: Sent initial sync to wake chip` →
  `BCSP: sync received` → `BCSP: sync_rsp received, moving to INIT
  state` → `BCSP: conf received, responding with conf_rsp` →
  `BCSP: Link established (first time)`
- bcsp_setup runs PSKEY + BDADDR + WARM_RESET
- After WARM_RESET: `BCSP: post-WARM_RESET sync — link returned to
  UNINIT` → handshake repeats → `BCSP: Link re-established after
  WARM_RESET`
- `hci0: Opcode 0x0c03 failed: -110` GONE
- `hciconfig hci0 up` succeeds; chip BD `00:1d:fe:85:64:a9`
  visible; `events:` counter > 0

Awaiting Yocto rebuild + redeploy to verify.

## 2026-05-22 (PM): Double power-cycle race wiping PSKEYs — fixed (65125445c55e)

On-device test of the skip-SYNC code revealed a second class of bug:
PSKEYs were being wiped right after we sent them, leaving the chip in
EEPROM-default state and the BCSP framing layer in disarray.

Two stacked bugs:

**Bug 1 — race between sync handler and bcsp_setup() power cycles**

After WARM_RESET, `bcsp_setup()` calls `msleep(1000)` then checks
`warm_reset_sent` and (if still set) fires `bcsp_serdev_power_cycle()`.
But `bcsp_serdev_power_cycle()` contains its OWN `msleep(2000)` for
chip settling. So when the sync handler (triggered by post-WARM_RESET
chip noise/SYNC) entered the power-cycle path FIRST, it took ~2.1 s
to clear `warm_reset_sent`. `bcsp_setup()` woke up after only 1 s,
saw the flag still set, and fired a SECOND overlapping power cycle.

Wire evidence:
```
[273.602] BCSP: Serdev mode - power cycling for clean restart   (sync handler)
[274.556] BCSP: skip-sync — explicit power cycle after WARM_RESET (bcsp_setup)
[275.916] BCSP: Waiting for chip to send sync after power cycle... (sync handler exits)
[298.624] Bluetooth: Error in BCSP hdr checksum                  (×many)
```

**Bug 2 — the hard power cycle itself was wrong**

PSKEYs are sent with `PSKEY_STORES_PSRAM = 0x08` (volatile store,
matching webOS). A GPIO-driven power cycle clears PSRAM and wipes
*everything we just configured*. The legacy/webOS path uses
**WARM_RESET BCCMD only** — chip restarts but PSRAM is kept intact.

By doing a hard power cycle after WARM_RESET (twice, no less), we
were dropping the chip back to EEPROM defaults — no PSKEY overrides,
no BD address, no RF calibration table.

**Fix (commit `65125445c55e`):**

- `bcsp_handle_le_pkt()`: early-return when `skip_sync` is set. Any
  inbound link-establishment packet is spurious in this mode (chip
  is supposed to be operational from EEPROM). Drops the race entry
  point entirely.
- `bcsp_setup()` (both BDADDR_PENDING and BDADDR_NONE paths): drop
  the `bcsp_serdev_power_cycle()` call. Wait 1.5 s for chip restart
  (matches webOS spacing) then resync driver-side BCSP sequence
  numbers, queues, and RX state. No GPIO toggle, PSRAM survives.

Expected next-state on-device after Yocto rebuild:
- Single log line `BCSP: skip-sync — resyncing driver state after WARM_RESET`
- No more `Power cycling Bluetooth chip…` after WARM_RESET
- No `Error in BCSP hdr checksum` / `Short BCSP packet` floods
- HCI Reset / Read Local Version succeed
- `hciconfig hci0 up` completes; chip BD addr `00:1d:fe:85:64:a9`

Awaiting Yocto rebuild + redeploy to verify.

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
