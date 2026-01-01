HP Pre3 (mantaray) Mainline Linux Status
=========================================

:Date: January 2026
:Author: webOS Community

Overview
--------

The HP Pre3 (codename "mantaray", board "rib") is a smartphone released in 2011
running webOS. It uses the Qualcomm MSM7230 SoC from the MSM7x30 family.

Hardware Specifications
-----------------------

- **SoC**: Qualcomm MSM7230 (single-core Scorpion ARMv7 @ 1GHz)
- **RAM**: 512MB
- **Storage**: 8GB/16GB eMMC
- **Display**: 3.58" 480x800 MDDI LCD
- **Touchscreen**: Cypress CY8CTMA300 (SPI)
- **Keyboard**: Physical QWERTY slide-out
- **PMIC**: PM8058 via SSBI
- **WiFi/BT**: Broadcom BCM4329
- **Cameras**: 5MP rear (MT9P013), VGA front (OV7739)
- **Audio**: Wolfson WM8958 codec
- **Sensors**: ST LIS331DLH accel, ST LSM303DLH mag, ISL29040 prox

Branch Information
------------------

- **Repository**: https://github.com/shr-distribution/linux
- **Branch**: mantaray/mainline
- **Base**: Linux 6.13+

Build Instructions
------------------

::

    make ARCH=arm mantaray_defconfig
    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc)
    # Outputs:
    #   arch/arm/boot/zImage
    #   arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dtb

Driver Status
-------------

Core SoC Drivers (NEW)
~~~~~~~~~~~~~~~~~~~~~~

+----------------------+------------------+----------------------------------+
| Component            | Status           | Notes                            |
+======================+==================+==================================+
| Clock Controller     | DONE             | gcc-msm7x30.c, ~53 clocks        |
+----------------------+------------------+----------------------------------+
| Pinctrl              | DONE             | pinctrl-msm7x30.c, 182 GPIOs     |
+----------------------+------------------+----------------------------------+
| VIC IRQ Controller   | DONE             | irq-qcom-vic.c, 128 interrupts   |
+----------------------+------------------+----------------------------------+
| Timer                | DONE             | timer-msm.c, DGT/GPT             |
+----------------------+------------------+----------------------------------+

Mainline Drivers (Working)
~~~~~~~~~~~~~~~~~~~~~~~~~~

+----------------------+------------------+----------------------------------+
| Component            | Status           | Driver                           |
+======================+==================+==================================+
| SSBI Bus             | WORKING          | drivers/mfd/ssbi.c               |
+----------------------+------------------+----------------------------------+
| PM8058 PMIC          | WORKING          | drivers/mfd/qcom-pm8xxx.c        |
+----------------------+------------------+----------------------------------+
| Serial Console       | WORKING          | drivers/tty/serial/msm_serial.c  |
+----------------------+------------------+----------------------------------+
| UART DM (BT)         | WORKING          | drivers/tty/serial/msm_serial.c  |
+----------------------+------------------+----------------------------------+
| I2C QUP              | WORKING          | drivers/i2c/busses/i2c-qup.c     |
+----------------------+------------------+----------------------------------+
| eMMC/SD (MMCI)       | WORKING          | drivers/mmc/host/mmci.c          |
+----------------------+------------------+----------------------------------+
| USB OTG              | WORKING          | drivers/usb/chipidea/            |
+----------------------+------------------+----------------------------------+
| GPIO Keys            | WORKING          | drivers/input/keyboard/gpio_keys |
+----------------------+------------------+----------------------------------+
| PM8058 Keypad        | WORKING          | drivers/input/keyboard/pmic8xxx  |
+----------------------+------------------+----------------------------------+
| PM8058 LEDs          | WORKING          | drivers/leds/leds-pm8058.c       |
+----------------------+------------------+----------------------------------+
| PM8058 RTC           | WORKING          | drivers/rtc/rtc-pm8xxx.c         |
+----------------------+------------------+----------------------------------+
| WiFi (BCM4329)       | WORKING          | brcmfmac (needs firmware)        |
+----------------------+------------------+----------------------------------+
| Bluetooth            | WORKING          | hci_bcm + btbcm                  |
+----------------------+------------------+----------------------------------+

Sensors (DT Added)
~~~~~~~~~~~~~~~~~~

+----------------------+------------------+----------------------------------+
| Component            | Status           | Notes                            |
+======================+==================+==================================+
| Accelerometer        | DT_ADDED         | ST LIS331DLH, I2C4 @ 0x19        |
+----------------------+------------------+----------------------------------+
| Magnetometer         | DT_ADDED         | ST LSM303DLH, I2C4 @ 0x1e        |
+----------------------+------------------+----------------------------------+
| Proximity/Light      | WORKING          | ISL29040, I2C4 @ 0x44            |
+----------------------+------------------+----------------------------------+

Missing/TODO
~~~~~~~~~~~~

+----------------------+------------------+----------------------------------+
| Component            | Status           | Notes                            |
+======================+==================+==================================+
| Touchscreen          | BLOCKED          | Cypress CY8CTMA300, no driver    |
+----------------------+------------------+----------------------------------+
| SPI Controller       | DONE             | spi-qcom-qsd.c @ 0xA8000000      |
+----------------------+------------------+----------------------------------+
| Display (MDP4)       | TODO             | DRM driver exists, need panel    |
+----------------------+------------------+----------------------------------+
| MDDI Panel           | TODO             | Need panel driver                |
+----------------------+------------------+----------------------------------+
| Audio (WM8958)       | PARTIAL          | Codec driver exists, need DAI    |
+----------------------+------------------+----------------------------------+
| MSM Audio (LPASS)    | TODO             | Need platform driver             |
+----------------------+------------------+----------------------------------+
| LM8502 LED           | DONE             | leds-lm8502.c, I2C4 @ 0x33       |
+----------------------+------------------+----------------------------------+
| LM3528 Backlight     | DONE             | lm3528_bl.c, I2C4 @ 0x36         |
+----------------------+------------------+----------------------------------+
| Battery (A6 MCU)     | DT_ADDED         | Palm proprietary, I2C0 @ 0x31    |
+----------------------+------------------+----------------------------------+
| Charger (SMB339)     | DT_ADDED         | Summit, I2C0 @ 0x2a              |
+----------------------+------------------+----------------------------------+
| Main Camera          | DT_ADDED         | Aptina MT9P013 5MP               |
+----------------------+------------------+----------------------------------+
| Front Camera         | DT_ADDED         | OmniVision OV7739 VGA            |
+----------------------+------------------+----------------------------------+
| Camera ISP (VFE)     | TODO             | MSM VFE driver                   |
+----------------------+------------------+----------------------------------+
| GPU (Adreno 205)     | TODO             | freedreno might work             |
+----------------------+------------------+----------------------------------+

Device Tree Files
-----------------

- ``arch/arm/boot/dts/qcom/qcom-msm7x30.dtsi`` - SoC definitions
- ``arch/arm/boot/dts/qcom/qcom-msm7230-hp-pre3.dts`` - Board definitions

Known Issues
------------

1. Regulators are currently fixed/dummy - need proper PCOM-based driver
2. SPI controller (QSD) has no mainline driver - blocks touchscreen support
3. Modem/AMSS communication not implemented (PCOM/SMD)

Testing
-------

Boot testing requires:

1. Build zImage and DTB
2. Create bootable image (uImage format for legacy bootloader)
3. Flash or boot via USB

Contact
-------

- webOS Community: https://webos-ports.org
- GitHub: https://github.com/shr-distribution/linux
