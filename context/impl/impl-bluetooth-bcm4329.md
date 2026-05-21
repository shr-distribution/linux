---
domain: bluetooth-bcm4329
created: "2026-05-21"
last_updated: "2026-05-21"
status: driver-mostly-implemented-module-deployment-blocker
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

## Current blocker: module deployment

The BT subsystem on tenderloin is **built as kernel modules**:
- `CONFIG_BT=m`
- `CONFIG_BT_HCIUART=m`
- `CONFIG_BT_HCIUART_BCSP=y` (compiled into hci_uart.ko per Makefile)
- `CONFIG_BT_HCIUART_BCM=y` (also into hci_uart.ko)

On the running kernel (e.g. `gd15db9a7ce28`):

```
/lib/modules/  has dirs for many OTHER kernel builds (gfd0a8d27192a,
gc49a03dba9ea both have bluetooth.ko.gz + hci_uart.ko.gz + btbcm.ko.gz)
                BUT NOT for currently running kernel.
```

So `modprobe bluetooth` and `modprobe hci_uart` both fail with
"Module not found in directory /lib/modules/<uname>". The
serdev framework therefore has no driver registered for the
`palm,bcm4329-bcsp` compatible, the DT subnode is orphan, and
nothing initialises. `hciconfig` returns
"Address family not supported by protocol" because the BT core
isn't loaded.

### Why modules aren't deployed

Open question — likely a Yocto image build / install path mismatch.
On a deploy that built a new uImage but didn't rebuild + install the
modules, /lib/modules ends up populated for the OLD kernel versions
but not the CURRENT one.

This is a **build-system issue**, not a kernel issue.

## Two paths to unblock testing

### Path A: Build BT in-tree (drop module flag)

```diff
- CONFIG_BT=m
+ CONFIG_BT=y
- CONFIG_BT_HCIUART=m
+ CONFIG_BT_HCIUART=y
```

Pros:
- Works regardless of /lib/modules deployment
- Kernel auto-binds serdev when bluetooth { ... } DT node appears
- One-line defconfig change per file

Cons:
- ~300-500 KB larger kernel image
- Mainline upstream prefers modular; would need re-evaluation before
  final submission

Upstream-style mainline keeps BT modular; in-tree is a tactical
choice for this specific port to remove the build-system blocker.

### Path B: Fix Yocto module deployment

Investigate why /lib/modules/<running_kernel> isn't populated for
the current build. Likely in the meta-mainline kernel recipe or
image install.

Pros:
- Keeps mainline-friendly modular config
- Doesn't bloat kernel image

Cons:
- Yocto integration work
- Out of kernel-tree scope

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
