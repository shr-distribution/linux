# BCM4329 Bluetooth Analysis for HP TouchPad

**Date:** March 2026
**Status:** Partially working - HCI device comes up but RF not working (not discoverable)

## Current Implementation Status

### What Works
- BCSP link establishment via hciattach
- BD address configuration via kernel driver (hci_bcsp.c bdaddr parameter)
- HCI device registration (hci0 appears)
- 12 critical PSKEYs sent before BDADDR

### What Doesn't Work
- **Device discovery/scanning** - TouchPad not visible to other devices
- **RF transmission** - Likely missing RF calibration PSKEYs

### Root Cause Analysis
The bcattach tool sends **~50 PSKEYs total**:
- 12 PSKEYs BEFORE BDADDR (now implemented in hci_bcsp.c)
- 1 BDADDR
- **~37 PSKEYs AFTER BDADDR** (NOT implemented - RF calibration tables!)
- 1 WARM_RESET

The missing ~37 PSKEYs include large RF calibration tables (power tables,
frequency compensation, etc.) which are likely required for proper radio operation.

### Next Steps
1. **Decompile webOS PmBtEngine** - The binary at `/usr/bin/.debug/PmBtEngine` is
   NOT stripped and has debug_info. Use Ghidra to extract exact PSKEY sequence.
2. **Extract from libPmBtBsaif.so** - Contains `palmPlatformCommonPskeys` and
   `palmPlatformSpecificPskeys` data tables at symbols 0x000e42cc and 0x000e4308.
3. **Port full bcattach sequence** - Add all ~50 PSKEYs to kernel driver or
   create userspace tool.

## Summary

The HP TouchPad's BCM4329 Bluetooth chip requires the **BCSP (BlueCore Serial Protocol)**
instead of the standard H4 or BCM protocols assumed by the mainline Linux hci_bcm driver.
This was discovered through analysis of webOS libraries and confirmed through testing.

## Hardware Configuration

- **Chip:** Broadcom BCM4329 (combo WiFi/Bluetooth)
- **Interface:** UART via GSBI6 (0x16540000)
- **Baud Rate:** 115200 (default), supports up to 3686400
- **Protocol:** BCSP (SLIP-framed)

### GPIO Assignments

| GPIO | Function | Active Level | Sysfs (offset 512) |
|------|----------|--------------|-------------------|
| 129  | HOST_WAKE (input) | High | gpio641 |
| 130  | BT_PWR | High | gpio642 |
| 131  | BT_WAKE | High | gpio643 |
| 138  | BT_RST | Low (active) | gpio650 |

## Protocol Discovery

### Initial Symptoms

When using the hci_bcm driver (H4 protocol), the chip responded with:
```
Frame reassembly failed (-84)
```

Direct UART communication at 115200 baud revealed the chip responds with
SLIP-framed data regardless of input:
```
c0 40 41 00 7e da dc ed ed a9 7a c0
```

The `0xC0` bytes are SLIP frame delimiters, indicating a SLIP-based protocol.

### WebOS Analysis

Analysis of webOS Bluetooth libraries revealed the protocol:

```bash
$ strings /usr/lib/libPmBtBsaif.so | grep -i uart
uartConfigBcsp
```

The `uartConfigBcsp` string confirmed that webOS used **BCSP protocol**.

### BCSP Protocol

BCSP (BlueCore Serial Protocol) is a reliable transport protocol that:
- Uses SLIP framing (0xC0 delimiters)
- Provides link establishment handshaking
- Supports retransmission for reliability
- Was commonly used by CSR/Cambridge Silicon Radio chips

The BCM4329's HCI reports "Cambridge Silicon Radio" as manufacturer, suggesting
this chip may have CSR-derived Bluetooth silicon or firmware.

## Working Configuration

### Required Kernel Config

```
CONFIG_BT=m
CONFIG_BT_HCIUART=m
CONFIG_BT_HCIUART_H4=y
CONFIG_BT_HCIUART_BCSP=y      # Required for HP TouchPad
CONFIG_BT_HCIUART_3WIRE=y
CONFIG_BT_HCIUART_BCM=y
```

### Initialization Sequence

```bash
# 1. Load Bluetooth modules
modprobe bluetooth
modprobe btbcm
modprobe hci_uart

# 2. Power on Bluetooth chip via GPIOs
echo 642 > /sys/class/gpio/export
echo 643 > /sys/class/gpio/export
echo 650 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio642/direction
echo out > /sys/class/gpio/gpio643/direction
echo out > /sys/class/gpio/gpio650/direction

echo 1 > /sys/class/gpio/gpio642/value  # BT_PWR on
echo 1 > /sys/class/gpio/gpio643/value  # BT_WAKE on
echo 0 > /sys/class/gpio/gpio650/value  # Reset assert
sleep 0.1
echo 1 > /sys/class/gpio/gpio650/value  # Reset release

# 3. Attach UART with BCSP protocol
hciattach /dev/ttyMSM1 bcsp 115200

# 4. Unblock RF-kill and bring up interface
rfkill unblock bluetooth
hciconfig hci0 up
```

### Verified Working Output

```
hci0:   Type: Primary  Bus: UART
        BD Address: 00:02:5B:00:A5:A5  ACL MTU: 310:10  SCO MTU: 64:8
        UP RUNNING
        RX bytes:1792 acl:0 sco:0 events:46 errors:0
        TX bytes:1607 acl:0 sco:0 commands:47 errors:0
        Features: 0xff 0xff 0x8f 0xfe 0x9b 0xff 0x59 0x83
        Name: 'tenderloin'
        HCI Version: 2.1 (0x4)  Revision: 0x12e9
        LMP Version: 2.1 (0x4)  Subversion: 0x12e9
        Manufacturer: Cambridge Silicon Radio (10)
```

**Note:** The address `00:02:5B:00:A5:A5` shown above is a default/uninitialized
address. The real BD address must be read from the device tokens (see below).

## BD Address from Device Tokens

The BCM4329 chip does not have its BD address programmed in firmware. Instead,
the address is stored in the device's **token partition** and must be set by
userspace during initialization.

### Token Partition Location

On HP TouchPad, tokens are stored in **partition 12** (`/dev/mmcblk0p12`).

### Reading Tokens

```bash
# Read BToADDR token
dd if=/dev/mmcblk0p12 bs=4096 2>/dev/null | strings -n 6 | grep -A1 'BToADDR'
```

Example output:
```
BToADDR
00:1D:FE:85:64:A9
```

### Token Format

| Token | Description | Example |
|-------|-------------|---------|
| BToADDR | Bluetooth MAC address | 00:1D:FE:xx:xx:xx |
| WIFIoADDR | WiFi MAC address | 00:1D:FE:xx:xx:xx |
| ProdSN | Product serial number | 5CL1251ANS |

The OUI `00:1D:FE` is registered to **Palm, Inc.**

### Setting the BD Address - KNOWN LIMITATION

The default address `00:02:5B:00:A5:A5` is the **CSR chip default** corresponding to
PSKEY_BDADDR default value `{ 0x00A5A5, 0x5b, 0x0002 }`.

**✗ ATTEMPTED: BD Address via Kernel Module Parameter**

We implemented a `bdaddr` module parameter in `hci_bcsp.c` that sends a BCCMD
packet on BCSP channel 2 during `bcsp_setup()`. However, **this approach did not
work** - the BD address remains at the default value.

```bash
# Attempted but did not work:
modprobe hci_uart bdaddr=00:1D:FE:85:64:A9
hciattach /dev/ttyMSM1 bcsp 115200
# BD address remains 00:02:5B:00:A5:A5
```

**Root Cause:** Analysis of the Android bcattach tool revealed that the chip
requires a **full PSKEY initialization sequence** (~50 BCCMD commands) before
the BDADDR PSKEY will be accepted. Sending only PSKEY_BDADDR is not sufficient.

See the "Android bcattach Tool Analysis" section below for details.

**Why Standard Methods Don't Work:**

1. **Kernel module param (our attempt)** - Only sends BDADDR, chip needs full init
2. **bdaddr tool (CSR method)** - Sends PSKEY after HCI is up; chip ignores it
3. **Broadcom vendor command (0x3F 0x0001)** - Chip uses CSR firmware, not Broadcom
4. **bccmd tool** - Not in BlueZ 5.x; would have same timing issue

**Solution Required:** Use the bcattach userspace tool to send the full PSKEY
initialization sequence before running hciattach. See recommendations below.

### Token Information (Reference Only)

The device's intended BD address is stored in the token partition but cannot
currently be applied to the chip:

| Token | Description | Example |
|-------|-------------|---------|
| BToADDR | Bluetooth MAC address | 00:1D:FE:xx:xx:xx |
| WIFIoADDR | WiFi MAC address | 00:1D:FE:xx:xx:xx |
| ProdSN | Product serial number | 5CL1251ANS |

The OUI `00:1D:FE` is registered to **Palm, Inc.**

### Future Investigation

To resolve the BD address issue, potential approaches include:

1. **Reverse engineer BluetoothMonitor** - Extract and analyze the proprietary
   webOS Bluetooth daemon to find how it configured the BD address

2. **BSA documentation** - Find Broadcom Server Application SDK documentation
   for the specific vendor commands used

3. **Firmware analysis** - Examine if there's a firmware update mechanism that
   could load a patched ROM with the correct address

## BCCMD Packet Format (Raw Protocol Analysis)

Analysis of the Android `bcattach` tool from webOS-ports/utilities revealed that
BD address can be set via **raw BCCMD commands sent over BCSP before hciattach**.
This is different from the post-attach HCI commands that were tested above.

### Source Reference

- Repository: https://github.com/webOS-ports/utilities
- File: `tenderloin-halium/bcattach/main.c` line 284

### BCCMD Packet Structure

The BCCMD (BlueCore Command) is wrapped in BCSP/SLIP framing:

```
┌─────────┬────────────────┬───────────────────────────────────────────┬─────────┬──────────┐
│  SLIP   │  BCSP Header   │              BCCMD Payload                │   CRC   │   SLIP   │
│  START  │   (4 bytes)    │                                           │(2 bytes)│   END    │
├─────────┼────────────────┼───────────────────────────────────────────┼─────────┼──────────┤
│   c0    │  f5 82 01 87   │  02 00 0c 00 00 00 03 70 00 00 ...        │  xx xx  │    c0    │
└─────────┴────────────────┴───────────────────────────────────────────┴─────────┴──────────┘
```

### BCCMD Header (10 bytes, all little-endian uint16)

| Offset | Field   | Value      | Description                        |
|--------|---------|------------|------------------------------------|
| 0-1    | Type    | 0x0002     | SETREQ (set request)               |
| 2-3    | Length  | 0x000C     | 12 words (24 bytes total payload)  |
| 4-5    | SeqNo   | 0x0000     | Sequence number                    |
| 6-7    | VarID   | 0x7003     | CSR_VARID_PS (persistent store)    |
| 8-9    | Status  | 0x0000     | OK                                 |

### PS Payload Structure (for PSKEY commands)

| Offset | Field   | Value      | Description                        |
|--------|---------|------------|------------------------------------|
| 0-1    | PSKey   | 0x0001     | PSKEY_BDADDR                       |
| 2-3    | Stores  | 0x0004     | Number of uint16 words             |
| 4-5    | Length  | 0x0008     | Length in bytes                    |
| 6+     | Data    | (variable) | PSKEY value                        |

### PSKEY_BDADDR Format (8 bytes / 4 words)

The BD address is stored in CSR's proprietary format:

```
BD Address: 00:1D:FE:86:13:AD
            ├────┤├─┤├──────┤
              NAP UAP   LAP

CSR Format (little-endian uint16 words):
  Word 0: 0x0086  (LAP bits 23-16)
  Word 1: 0x13AD  (LAP bits 15-0)
  Word 2: 0x00FE  (UAP)
  Word 3: 0x001D  (NAP)

Raw bytes: 86 00 ad 13 fe 00 1d 00
```

### Example: Complete PSKEY_BDADDR Packet

From bcattach main.c line 284 (sets BD address 00:1D:FE:86:13:AD):

```
c0 f5 82 01 87 02 00 0c 00 00 00 03 70 00 00 01 00 04 00 08 00 86 00 ad 13 fe 00 1d 00 c8 07 c0
│  └──────────┘ └─────────────────────────────────────────────────────────────────────┘ └────┘ │
│   BCSP hdr              BCCMD: Type Len  Seq  VarID Stat PSKey Stor Len  BD_ADDR_DATA  CRC   │
SLIP                                                                                        SLIP
```

### PSKEY_HOST_INTERFACE (Protocol Switching)

The chip's UART protocol can potentially be changed via PSKEY_HOST_INTERFACE (0x01FE):

| Value | Protocol |
|-------|----------|
| 0x0001 | BCSP (current) |
| 0x0003 | H4 (standard HCI UART) |

**Theoretical H4 switching command:**
```
BCCMD payload for PSKEY_HOST_INTERFACE = H4:
  Type:   02 00  (SETREQ)
  Length: 08 00  (8 words)
  SeqNo:  00 00
  VarID:  03 70  (PS)
  Status: 00 00
  PSKey:  fe 01  (0x01FE = HOST_INTERFACE)
  Stores: 01 00  (1 word)
  Length: 02 00  (2 bytes)
  Value:  03 00  (0x0003 = H4)
```

After setting HOST_INTERFACE, a **WARM_RESET** (VarID 0x4002) is required:
```
BCCMD for WARM_RESET:
  Type:   02 00  (SETREQ)
  Length: 05 00  (5 words, no payload)
  SeqNo:  00 00
  VarID:  02 40  (0x4002 = WARM_RESET)
  Status: 00 00
```

### Challenges with Protocol Switching

1. **Timing**: After WARM_RESET, the host must immediately switch UART handling
   from BCSP to H4 - requires custom tooling

2. **Persistence**: PSKEY changes may not persist across power cycles if the
   chip lacks EEPROM/flash for PS storage

3. **Untested**: H4 mode has not been verified to work on this chip

### Key Insight: Pre-Attach BD Address Setting

The bcattach tool sets PSKEY_BDADDR **before** calling hciattach, using raw
BCSP packets on the UART. This bypasses the HCI layer entirely and may be
the only way to set the BD address on this chip.

**Recommended approach for BD address:**
1. Open UART directly (not via hciattach)
2. Perform BCSP link establishment
3. Send PSKEY_BDADDR via raw BCCMD
4. Close UART
5. Run standard hciattach

This requires porting/adapting the bcattach tool from the Android utilities.

## Device Tree Status

The Bluetooth node in the device tree is currently **commented out** because:

1. The mainline `hci_bcm` driver uses H4 protocol, not BCSP
2. There is no serdev driver for BCSP in the Linux kernel
3. Manual `hciattach` with BCSP protocol is required

### Current DT Node (disabled)

```dts
/* Bluetooth requires BCSP protocol - use hciattach manually
bluetooth {
    compatible = "brcm,bcm4329-bt";
    vddio-supply = <&pm8058_s3>;
    vbat-supply = <&pm8901_l3>;
    clocks = <&sleep_clk>;
    clock-names = "lpo";
    max-speed = <3686400>;
    pinctrl-names = "default";
    pinctrl-0 = <&bt_pin>;
    reset-gpios = <&tlmm 138 GPIO_ACTIVE_LOW>;
    shutdown-gpios = <&tlmm 130 GPIO_ACTIVE_HIGH>;
    device-wakeup-gpios = <&tlmm 131 GPIO_ACTIVE_HIGH>;
    interrupt-parent = <&tlmm>;
    interrupts = <129 IRQ_TYPE_EDGE_RISING>;
    interrupt-names = "host-wakeup";
};
*/
```

## Future Work

### Option 1: Userspace Service (Recommended)

Create a systemd service to run hciattach at boot:

```ini
[Unit]
Description=HP TouchPad Bluetooth
After=sys-devices-platform-soc-16500000.gsbi-16540000.serial.device

[Service]
Type=forking
ExecStartPre=/usr/bin/bt-power-on.sh
ExecStart=/usr/bin/hciattach /dev/ttyMSM1 bcsp 115200
ExecStartPost=/usr/bin/rfkill unblock bluetooth

[Install]
WantedBy=bluetooth.target
```

### Option 2: Kernel Module Parameter (IMPLEMENTED BUT INSUFFICIENT)

BD address setting is implemented in `hci_bcsp.c` but **does not work** because
the chip requires the full PSKEY init sequence:

- Module parameter: `hci_uart.bdaddr=XX:XX:XX:XX:XX:XX`
- BCCMD sent during `bcsp_setup()` callback
- Code sends only PSKEY_BDADDR, which the chip ignores without full init

The implementation remains in the kernel for potential future use if a minimal
PSKEY subset can be identified.

### Option 3: Kernel Driver Enhancement

Extend hci_bcm or create a new serdev driver to support BCSP protocol for
chips that require it. This would require:

1. Adding BCSP protocol support to serdev Bluetooth infrastructure
2. New device tree binding for protocol selection
3. Upstream submission and review

### Option 4: H4 Protocol Switching (Experimental)

Test if the chip can be switched from BCSP to H4 protocol:

1. Set PSKEY_HOST_INTERFACE = 0x0003 (H4) via raw BCCMD
2. Send WARM_RESET (VarID 0x4002)
3. Immediately switch UART to H4 mode
4. Use standard hciattach with H4 protocol

If successful, this would allow using the mainline hci_bcm driver.

### Option 5: Firmware Investigation

Investigate whether the BCM4329 can be configured to use H4 protocol
through firmware patchram. The webOS ROM did not contain any .hcd firmware
files, suggesting the chip may not require or support firmware patching.

## Android bcattach Tool Analysis

### Source Location

- Repository: https://github.com/webOS-ports/utilities
- Path: `tenderloin-halium/bcattach/main.c` (667 lines)

### Key Finding: No External Firmware File

**The bcattach tool does NOT load any external firmware file.** All BCCMD commands are
hardcoded as raw byte arrays directly in the source code. This means the "firmware"
for the BCM4329 Bluetooth chip is actually a sequence of **PSKEY configuration commands**.

### Initialization Sequence Overview

The bcattach tool performs the following sequence:

| Phase | Lines | Description |
|-------|-------|-------------|
| 1. GPIO Setup | 108-170 | Reset chip via `/sys/user_hw/pins/bt/reset/level` |
| 2. UART Config | 174-206 | Configure UART at 115200 baud with HSUART ioctls |
| 3. BCSP Link | 216-238 | Link establishment (sync/conf packets, up to 100 retries) |
| 4. PSKEY Writes | 239-436 | ~50 BCCMD PSKEY configuration commands |
| 5. WARM_RESET | 436 | Apply all PSKEY changes (VarID 0x4002) |
| 6. Close UART | 442 | Done with initialization |

### PSKEY Commands Sent (Partial List)

The tool sends configuration for many persistent store keys:

| Line | PSKEY | Description |
|------|-------|-------------|
| 241 | 0x2819 | Unknown (first command after link) |
| 246-282 | various | RF parameters, power levels |
| **284** | **0x0001** | **PSKEY_BDADDR** (the BD address) |
| 287-436 | various | More RF, audio, features |
| **436** | **0x4002** | **WARM_RESET** (applies all changes) |

### PSKEY_BDADDR Location

The BD address setting is at **line 284**:

```c
write_fd(uart_fd,"\xc0\xf5\x82\x01\x87\x02\x00\x0c\x00\x00\x00\x03\x70\x00\x00\x01\x00\x04\x00\x08\x00\x86\x00\xad\x13\xfe\x00\x1d\x00\xc8\x07\xc0",32);
```

Decoded:
- BD Address set: `00:1D:FE:86:13:AD` (hardcoded in the tool!)
- This is NOT read from device tokens - it's a fixed value

### WARM_RESET Command (Line 436)

After all PSKEYs are written, a warm reset is required:

```c
write_fd(uart_fd,"\xc0\xc7\x22\x01\x15\x02\x00\x09\x00\x00\x00\x02\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf5\xd2\xc0",26);
```

Decoded:
- VarID: `0x4002` (WARM_RESET)
- This tells the chip to apply all PSKEY changes

### Dead Code / Planned Features (Lines 447-666)

After `return 0;` at line 445, there is unreachable code that shows **planned but unused features**:

1. **Baud rate switching to 3.68MHz** (line 454: `uart_mode.speed = 0x384000`)
2. **Re-establishing BCSP link at high speed**
3. **Device name setting** (line 575: "CSR - bc6")
4. **N_HCI line discipline attachment** (lines 632-641)

This dead code suggests the tool was meant to:
1. Initialize at 115200
2. Switch to 3.68MHz for faster operation
3. Set device name
4. Attach to kernel HCI via ioctl

### Why Our Kernel-Only BDADDR Didn't Work

Our implementation in `hci_bcsp.c` sends **only** the PSKEY_BDADDR command. Testing showed
the BD address doesn't change because:

1. **The chip requires the full PSKEY initialization sequence** - not just BDADDR
2. **Many PSKEYs must be set before BDADDR** to properly configure the RF subsystem
3. **A WARM_RESET is required after all PSKEYs** to apply the changes
4. Sending BDADDR alone (even with timing delays) is not sufficient

### Required Solution

To properly set the BD address, we must either:

1. **Use the bcattach userspace tool** (recommended)
   - Modify it to read BD address from device tokens instead of hardcoded value
   - Run it before hciattach

2. **Port full PSKEY sequence to kernel** (not recommended)
   - Would require ~200 lines of hardcoded BCCMD packets
   - Not maintainable or suitable for upstream

3. **Investigate minimal PSKEY set** (future research)
   - Find which PSKEYs are truly required before BDADDR
   - May reduce to a smaller subset

### Critical PSKEYs Before BDADDR

Analysis shows bcattach sends **12 PSKEYs before BDADDR**:

| # | PSKEY | Value | Description |
|---|-------|-------|-------------|
| 1 | 0x01FE HOST_INTERFACE | 0x6590 | BCSP mode configuration |
| 2 | 0x01BE PCM_MIN_CPU_CLOCK | 0x3AFC | PCM clock setting |
| 3 | 0x01AB H_HC_FC_MAX_ACL | 0x0001 | ACL packet length |
| 4 | 0x01B0 H_HC_FC_MAX_SCO | 0x0001 | SCO packet length |
| 5 | 0x01B9 PCM_SAMPLE_SIZE | 0x0008 | PCM sample size |
| 6 | **0x01F6 ANA_FREQ** | **0x0019** | **26MHz crystal (critical!)** |
| 7 | 0x0011 LC_MAX_TX_POWER | 0x0154 | Max TX power |
| 8 | 0x0013 LC_DEFAULT_TX_POWER | 0x000B | Default TX power |
| 9 | 0x024D LC_MAX_TX_POWER_NO_RSSI | 0x0000 | Max TX power (no RSSI) |
| 10 | 0x000E ENC_KEY_LMIN | 0x0001 | Encryption key length |
| 11 | 0x01F9 XTAL_FTRIM | 0x0001 | Crystal fine trim |
| 12 | 0x025D LC_DEFAULT_TX_POWER_NO_RSSI | 0x0001 | Default TX (no RSSI) |

**Most Critical:** PSKEY_ANA_FREQ (0x01F6) = 25 (0x19) sets the 26MHz external crystal.
Without this, the chip clock may not be properly configured.

### bcattach vs PmBtStack Comparison

Surprisingly, bcattach and PmBtStack use **almost completely different PSKEY sets**:

- **bcattach focus:** TX power, host interface, crystal/clock settings
- **PmBtStack focus:** PCM/audio config, UART config, SCO audio

Only PSKEY_BDADDR (0x0001) is common. This suggests:
1. bcattach is a minimal initialization tool for basic BT operation
2. PmBtStack adds audio/SCO configuration for headset support
3. Both approaches are valid but serve different purposes

### Recommendations

**Short term:** Use modified bcattach tool that:
1. Reads BD address from `/dev/mmcblk0p12` token partition
2. Sends the 12 critical PSKEYs before BDADDR
3. Sends BDADDR with correct address
4. Sends WARM_RESET (0x4002) to apply changes
5. Then run standard hciattach

**Minimal kernel approach:** Could add just the critical PSKEYs to hci_bcsp.c:
1. PSKEY_ANA_FREQ = 25 (26MHz crystal)
2. PSKEY_HOST_INTERFACE = BCSP mode
3. PSKEY_BDADDR
4. WARM_RESET

**Long term:** Consider if bcattach can be simplified or if chip behavior can be
better understood to reduce the required PSKEY set.

## webOS 2.6 Kernel Analysis

The webOS 2.6 kernel (`webos-linux-kernel-touchpad`) provides **no PSKEY/BCCMD initialization**.

### Kernel Role (Minimal)

The kernel only handles GPIO power control:

```c
// arch/arm/mach-msm/gpiomux-tenderloin.h
#define BT_RST_N      138  // Reset (active low)
#define BT_POWER      130  // Power enable
#define BT_WAKE       131  // Wake chip
#define BT_HOST_WAKE  129  // Chip signals host

// drivers/misc/bluetooth-power-pe.c
// Just exports sysfs interface for power on/off
```

### Key Finding: No BCSP in Kernel

```
# arch/arm/configs/tenderloin_defconfig
CONFIG_BLUETOOTH_POWER_STATE=y
# CONFIG_BT_HCIUART_BCSP is NOT set!
```

The webOS kernel has **BCSP disabled**. All Bluetooth initialization is done entirely
in userspace by **PmBtStack**, which:

1. Opens `/dev/ttyS2` (or `/dev/bt_uart` at 3.68MHz) directly
2. Sends BCSP link establishment packets
3. Sends all PSKEYs via raw BCCMD
4. Reads BD address from `-X` option (passed from `/dev/tokens/BToADDR`)
5. Continues running as the complete Bluetooth stack (replaces BlueZ!)

### webOS Binaries for Ghidra Analysis

The following binaries from webOS 3.0.5 (doctor305) are available for decompilation:

| Binary | Path | Status | Size |
|--------|------|--------|------|
| PmBtEngine | `/usr/bin/.debug/PmBtEngine` | NOT stripped, has debug_info | 772KB |
| libPmBtBsaif.so | `/usr/lib/libPmBtBsaif.so` | Has symbols | 96KB |
| libPmBtOs.so | `/usr/lib/libPmBtOs.so` | Has symbols | 40KB |

**Key symbols in libPmBtBsaif.so:**
- `palmPlatformCommonPskeys` (0x000e42cc) - Common PSKEY table
- `palmPlatformSpecificPskeys` (0x000e4308) - Device-specific PSKEYs
- `CsrBuildPsKeyCommand` - Function to build PSKEY BCCMD
- `CsrBccmdWritePsValueReqSend` - Function to send PSKEY value
- `PmBtBsaifBccmdGetNumPskeys` - Returns number of PSKEYs to configure

**Ghidra location:** `/opt/Ghidra`

### Implication for Mainline Linux

Since mainline uses BlueZ + hciattach (not PmBtStack), we need to add PSKEY
initialization either:

1. **In kernel** - Add to `hci_bcsp.c` (our current approach, needs more PSKEYs)
2. **In userspace** - Port bcattach to run before hciattach

## References

- WebOS kernel: `drivers/serial/bcm_bt_lpm.c` (GPIO handling only)
- WebOS libraries: `libPmBtBsaif.so`, `libPmBtOs.so`
- Linux BCSP driver: `drivers/bluetooth/hci_bcsp.c`
- Linux BCM driver: `drivers/bluetooth/hci_bcm.c`
- Android bcattach tool: https://github.com/webOS-ports/utilities (`tenderloin-halium/bcattach/`)
  - `main.c` - Raw BCCMD sequences including PSKEY_BDADDR (line 284)
  - `hciattach.c` - BCSP link establishment and protocol handling

## Test Scripts

- `scripts/bt-test.sh` - Automated Bluetooth testing
- `scripts/deploy-bt.sh` - Module deployment

## Commits

- `be1a0c7f1b2e` - Enable H4 and H5/3WIRE protocols in defconfigs
- `afcddbff22a8` - Enable BCSP protocol in defconfigs
