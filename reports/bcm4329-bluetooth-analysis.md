# BCM4329 Bluetooth Analysis for HP TouchPad

**Date:** March 2026
**Status:** Working with manual hciattach using BCSP protocol (BD address limitation - see below)

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

**⚠️ IMPORTANT: BD Address Cannot Be Changed**

Despite the chip reporting as "Cambridge Silicon Radio" (manufacturer 10), testing
has confirmed that **none of the standard methods work** to change the BD address:

1. **bdaddr tool (CSR method)** - Reports "Address changed" but HCI Read_BD_ADDR
   still returns the original address. The PSKEY write appears to be ignored.

2. **Broadcom vendor command (0x3F 0x0001)** - No effect. The chip doesn't
   respond to Broadcom-specific BD address commands.

3. **bccmd tool** - Not available in BlueZ 5.x (was in BlueZ 4.x deprecated tools).
   Even if available, likely would have same issue as bdaddr.

4. **Sysfs interface** - Not available for UART-attached HCI devices.

**Root Cause Analysis:**

The BCM4329 in the HP TouchPad uses CSR-compatible firmware but appears to have
the BD address hardcoded in ROM. Unlike chips with EEPROM or flash-based PSKEY
storage, this chip's PSKEY values cannot be modified at runtime.

The original webOS used **BSA (Broadcom Server Application)** stack with the
proprietary `BluetoothMonitor` daemon. This may have used a different, undocumented
method to configure the BD address, or webOS may have simply accepted the default
address.

**Practical Impact:**

- Bluetooth **functionality works** with the default address
- Device will be identified as "00:02:5B:00:A5:A5" instead of the Palm-assigned address
- Pairing and connections work normally
- The Palm-assigned address (e.g., 00:1D:FE:85:64:A9) from tokens is unused

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

### Option 2: Port bcattach Tool (For BD Address)

Adapt the Android `bcattach` tool to set BD address before hciattach:

1. Extract BCSP link establishment code from `hciattach.c`
2. Extract BCCMD packet construction from `main.c`
3. Read BD address from token partition
4. Create minimal tool that:
   - Opens UART
   - Performs BCSP link establishment
   - Sends PSKEY_BDADDR via raw BCCMD
   - Exits (let hciattach handle the rest)

This is the most promising approach for BD address configuration.

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

## References

- WebOS kernel: `drivers/serial/bcm_bt_lpm.c` (GPIO handling only)
- WebOS libraries: `libPmBtBsaif.so`, `libPmBtOs.so`
- Linux BCSP driver: `drivers/bluetooth/hci_bcsp.c`
- Linux BCM driver: `drivers/bluetooth/hci_bcm.c`
- Android bcattach tool: https://github.com/webOS-ports/utilities (`tenderloin-halium/bcattach/`)
  - `main.c` - Raw BCCMD sequences including PSKEY_BDADDR
  - `hciattach.c` - BCSP link establishment and protocol handling

## Test Scripts

- `scripts/bt-test.sh` - Automated Bluetooth testing
- `scripts/deploy-bt.sh` - Module deployment

## Commits

- `be1a0c7f1b2e` - Enable H4 and H5/3WIRE protocols in defconfigs
- `afcddbff22a8` - Enable BCSP protocol in defconfigs
