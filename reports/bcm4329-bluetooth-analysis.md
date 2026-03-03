# BCM4329 Bluetooth Analysis for HP TouchPad

**Date:** March 2026
**Status:** Working with manual hciattach using BCSP protocol

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

### Option 2: Kernel Driver Enhancement

Extend hci_bcm or create a new serdev driver to support BCSP protocol for
chips that require it. This would require:

1. Adding BCSP protocol support to serdev Bluetooth infrastructure
2. New device tree binding for protocol selection
3. Upstream submission and review

### Option 3: Firmware Investigation

Investigate whether the BCM4329 can be configured to use H4 protocol
through firmware patchram. The webOS ROM did not contain any .hcd firmware
files, suggesting the chip may not require or support firmware patching.

## References

- WebOS kernel: `drivers/serial/bcm_bt_lpm.c` (GPIO handling only)
- WebOS libraries: `libPmBtBsaif.so`, `libPmBtOs.so`
- Linux BCSP driver: `drivers/bluetooth/hci_bcsp.c`
- Linux BCM driver: `drivers/bluetooth/hci_bcm.c`

## Test Scripts

- `scripts/bt-test.sh` - Automated Bluetooth testing
- `scripts/deploy-bt.sh` - Module deployment

## Commits

- `be1a0c7f1b2e` - Enable H4 and H5/3WIRE protocols in defconfigs
- `afcddbff22a8` - Enable BCSP protocol in defconfigs
