# HP TouchPad Mainline Kernel Test Plan

**Version:** 1.0
**Date:** 2026-01-02
**Kernel:** Linux 6.18 LTS
**Branch:** `tenderloin/6.18/upstream-patches`

---

## Table of Contents
1. [Test Environment Setup](#1-test-environment-setup)
2. [Boot and Basic System](#2-boot-and-basic-system)
3. [Display Subsystem](#3-display-subsystem)
4. [Touchscreen](#4-touchscreen)
5. [WiFi](#5-wifi)
6. [Bluetooth](#6-bluetooth)
7. [GPS](#7-gps)
8. [Audio](#8-audio)
9. [Sensors](#9-sensors)
10. [LED Controller](#10-led-controller)
11. [Battery and Charging](#11-battery-and-charging)
12. [USB](#12-usb)
13. [GPU](#13-gpu)
14. [Cameras](#14-cameras)
15. [HDMI Output](#15-hdmi-output)
16. [3G Modem](#16-3g-modem)
17. [Video Codec (VIDC)](#17-video-codec-vidc)
18. [Audio DSP (LPASS)](#18-audio-dsp-lpass)
19. [Power Management](#19-power-management)
20. [Stress Testing](#20-stress-testing)

---

## 1. Test Environment Setup

### 1.1 Required Hardware
- [ ] HP TouchPad WiFi (Topaz WiFi) - primary test device
- [ ] HP TouchPad 3G (Topaz 3G) - for 3G/ISP1763 testing
- [ ] HP TouchPad Go WiFi (Opal WiFi) - for Opal-specific features
- [ ] HP TouchPad Go 3G (Opal 3G) - for complete coverage
- [ ] USB cable (micro-USB)
- [ ] USB OTG adapter
- [ ] USB keyboard/mouse for OTG testing
- [ ] HDMI cable and monitor (for WiFi variants)
- [ ] 3.5mm headphones
- [ ] External speakers (for line-out testing)
- [ ] WiFi access point
- [ ] Bluetooth device (keyboard, speaker, or phone)
- [ ] GPS with clear sky view or GPS simulator
- [ ] USB charger (5V/2A minimum)
- [ ] Touchpad dock with charger (if available)

### 1.2 Required Software
- [ ] Build environment with ARM cross-compiler
- [ ] fastboot and adb tools
- [ ] Root filesystem (LuneOS, postmarketOS, or custom)
- [ ] Test utilities:
  - `evtest` - input device testing
  - `wpa_supplicant` - WiFi
  - `bluez` - Bluetooth
  - `gpsd` - GPS
  - `alsa-utils` - Audio
  - `iio-sensor-proxy` - Sensors
  - `v4l-utils` - Camera/video
  - `glmark2-es2` - GPU benchmarking
  - `stress-ng` - System stress testing

### 1.3 Build the Kernel
```bash
cd /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

# Configure for TouchPad
make qcom_defconfig
# Or use device-specific config if available

# Build
make -j$(nproc) zImage modules dtbs

# Output files:
# - arch/arm/boot/zImage
# - arch/arm/boot/dts/qcom/qcom-apq8060-hp-tenderloin.dtb (Topaz WiFi)
# - arch/arm/boot/dts/qcom/qcom-apq8060-hp-tenderloin-3g.dtb (Topaz 3G)
# - arch/arm/boot/dts/qcom/qcom-apq8060-hp-tenderloin-opal.dtb (Opal WiFi)
# - arch/arm/boot/dts/qcom/qcom-apq8060-hp-tenderloin-opal-3g.dtb (Opal 3G)
```

### 1.4 Flash the Kernel
```bash
# Enter fastboot mode (hold Volume Up while booting)
fastboot flash boot boot.img
# Or use moboot/kexec for dual-boot setups
```

---

## 2. Boot and Basic System

### 2.1 Kernel Boot
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Cold boot | Power on device | Kernel boots to login prompt | [ ] |
| Boot messages | Check `dmesg` output | No critical errors, all drivers load | [ ] |
| Device tree | `cat /proc/device-tree/model` | Shows correct device variant | [ ] |
| CPU detection | `cat /proc/cpuinfo` | Shows dual Scorpion cores @ 1.2GHz | [ ] |
| Memory | `free -h` | Shows ~1GB RAM (minus reserved) | [ ] |
| Kernel version | `uname -a` | Shows 6.18.x kernel | [ ] |

### 2.2 Driver Loading
```bash
# Check all expected modules loaded
lsmod

# Check for driver errors
dmesg | grep -i error
dmesg | grep -i fail
dmesg | grep -i warn
```

| Driver | Module/Built-in | Status |
|--------|-----------------|--------|
| PM8058 PMIC | Built-in | [ ] |
| PM8901 PMIC | Built-in | [ ] |
| GSBI/I2C | Built-in | [ ] |
| GSBI/SPI | Built-in | [ ] |
| GSBI/UART | Built-in | [ ] |
| GPIO/Pinctrl | Built-in | [ ] |
| Clock controllers | Built-in | [ ] |

---

## 3. Display Subsystem

### 3.1 LVDS Panel
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Panel detection | `cat /sys/class/drm/card0-*/status` | Shows "connected" | [ ] |
| Resolution | `cat /sys/class/drm/card0-*/modes` | 1024x768 @ 60Hz | [ ] |
| Framebuffer | `cat /sys/class/graphics/fb0/virtual_size` | 1024x768 or larger | [ ] |
| Display output | Boot to GUI | Display shows correctly | [ ] |
| Color accuracy | Display test pattern | Colors appear correct | [ ] |

### 3.2 Backlight Control
```bash
# Find backlight device
ls /sys/class/backlight/

# Test brightness levels
echo 0 > /sys/class/backlight/*/brightness    # Off
echo 128 > /sys/class/backlight/*/brightness  # 50%
echo 255 > /sys/class/backlight/*/brightness  # 100%
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Backlight device | Check sysfs | Device present | [ ] |
| Minimum brightness | Set to 0 | Screen dims to minimum | [ ] |
| Maximum brightness | Set to max | Screen at full brightness | [ ] |
| Gradual change | Sweep 0-255 | Smooth brightness transition | [ ] |

---

## 4. Touchscreen

### 4.1 Basic Touch
```bash
# Find touchscreen device
cat /proc/bus/input/devices | grep -A5 -i touch

# Test with evtest
evtest /dev/input/eventX  # Replace X with touchscreen device
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device detection | Check /dev/input/ | Touchscreen device present | [ ] |
| Single touch | Touch screen | Reports coordinates | [ ] |
| Multi-touch | Touch with 2+ fingers | Reports multiple contacts | [ ] |
| Touch accuracy | Touch corners and center | Coordinates match position | [ ] |
| Touch release | Lift finger | Reports release event | [ ] |
| Swipe gesture | Swipe across screen | Continuous coordinate updates | [ ] |
| Pinch gesture | Pinch with two fingers | Reports both contacts correctly | [ ] |

### 4.2 Touchscreen Power Management
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Sleep mode | Idle device | Touchscreen enters low power | [ ] |
| Wake on touch | Touch sleeping device | Screen wakes up | [ ] |
| Firmware health | Check IRQ counter | Counter increments normally | [ ] |

### 4.3 Cypress SWD Programmer (if firmware update needed)
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| SWD interface | Check dmesg for SWD init | SWD initialized | [ ] |
| Firmware version | Read firmware info | Version reported | [ ] |

---

## 5. WiFi

### 5.1 Basic WiFi
```bash
# Check wireless interface
ip link show wlan0
iw dev

# Scan for networks
iw dev wlan0 scan | grep SSID

# Connect to network
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
dhclient wlan0
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Interface detection | `ip link` | wlan0 present | [ ] |
| Firmware load | Check dmesg | ath6kl firmware loaded | [ ] |
| Network scan | `iw dev wlan0 scan` | Networks discovered | [ ] |
| WPA2 connection | Connect to WPA2 network | Connected, IP assigned | [ ] |
| Internet access | `ping 8.8.8.8` | Ping successful | [ ] |
| DNS resolution | `ping google.com` | Name resolves, ping works | [ ] |
| Signal strength | `iw dev wlan0 link` | Shows signal level | [ ] |
| Throughput | `iperf3` to local server | Reasonable speeds (>20 Mbps) | [ ] |

### 5.2 WiFi Power Management
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Power save mode | Enable PSM | Device enters power save | [ ] |
| Wake on packet | Send packet to device | Device wakes, receives | [ ] |
| Reconnect after sleep | Resume from suspend | WiFi reconnects | [ ] |

---

## 6. Bluetooth

### 6.1 Basic Bluetooth
```bash
# Start bluetooth service
systemctl start bluetooth

# Check adapter
bluetoothctl
> power on
> agent on
> scan on
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Adapter detection | `hciconfig` | hci0 present | [ ] |
| Power on | `bluetoothctl power on` | Adapter powers on | [ ] |
| Device scan | `bluetoothctl scan on` | Discovers nearby devices | [ ] |
| Pairing | Pair with phone/keyboard | Pairing successful | [ ] |
| A2DP audio | Play audio to BT speaker | Audio plays correctly | [ ] |
| HID input | Connect BT keyboard | Keyboard works | [ ] |
| File transfer | Send file via OBEX | Transfer completes | [ ] |

---

## 7. GPS

### 7.1 Basic GPS (GSBI5 UART)
```bash
# Check serial device
ls -la /dev/ttyHS*  # or /dev/ttyMSM*

# Start gpsd
gpsd /dev/ttyHS0 -n

# Check GPS data
gpsmon
# or
cgps -s
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| UART device | Check /dev/ | GPS serial device present | [ ] |
| gpsd connection | Start gpsd | Connects to device | [ ] |
| Satellite search | Run cgps outdoors | Searches for satellites | [ ] |
| Position fix | Wait for fix | Gets 3D position fix | [ ] |
| Accuracy | Compare to known location | Within 10m accuracy | [ ] |
| Time sync | Check GPS time | Time matches UTC | [ ] |

---

## 8. Audio

### 8.1 Audio Playback
```bash
# List audio devices
aplay -l
arecord -l

# Test speaker output
speaker-test -c 2 -t wav

# Play audio file
aplay test.wav
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Codec detection | Check dmesg for wm8958 | Codec initialized | [ ] |
| Speaker output | Play audio | Sound from speakers | [ ] |
| Headphone output | Insert headphones, play | Sound in headphones | [ ] |
| Headphone detect | Insert/remove headphones | Detection works | [ ] |
| Volume control | Adjust volume | Volume changes | [ ] |
| Microphone input | Record audio | Recording works | [ ] |
| Line out | Connect to external amp | Audio output works | [ ] |

### 8.2 Audio Mixer
```bash
# Open mixer
alsamixer

# Check available controls
amixer contents
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Master volume | Adjust in alsamixer | Changes output level | [ ] |
| Mute | Mute output | Audio stops | [ ] |
| Mic gain | Adjust mic gain | Recording level changes | [ ] |
| Routing | Switch outputs | Audio routes correctly | [ ] |

---

## 9. Sensors

### 9.1 Gyroscope (MPU3050)
```bash
# Find IIO device
cat /sys/bus/iio/devices/iio:device*/name | grep -n mpu

# Read raw values
cat /sys/bus/iio/devices/iio:device0/in_anglvel_x_raw
cat /sys/bus/iio/devices/iio:device0/in_anglvel_y_raw
cat /sys/bus/iio/devices/iio:device0/in_anglvel_z_raw
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device detection | Check IIO devices | mpu3050 present | [ ] |
| X-axis rotation | Rotate around X | Value changes | [ ] |
| Y-axis rotation | Rotate around Y | Value changes | [ ] |
| Z-axis rotation | Rotate around Z | Value changes | [ ] |
| Calibration | Keep device still | Values near zero | [ ] |

### 9.2 Accelerometer (LSM303DLH)
```bash
# Find device and read values
cat /sys/bus/iio/devices/iio:device*/in_accel_x_raw
cat /sys/bus/iio/devices/iio:device*/in_accel_y_raw
cat /sys/bus/iio/devices/iio:device*/in_accel_z_raw
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device detection | Check IIO devices | lsm303 accel present | [ ] |
| Gravity detection | Hold flat | Z shows ~1g | [ ] |
| Tilt response | Tilt device | X/Y values change | [ ] |
| Screen rotation | Rotate 90 degrees | Triggers rotation event | [ ] |

### 9.3 Magnetometer (LSM303DLH)
```bash
cat /sys/bus/iio/devices/iio:device*/in_magn_x_raw
cat /sys/bus/iio/devices/iio:device*/in_magn_y_raw
cat /sys/bus/iio/devices/iio:device*/in_magn_z_raw
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device detection | Check IIO devices | lsm303 magn present | [ ] |
| North detection | Point device north | Consistent reading | [ ] |
| Compass function | Rotate device | Values change predictably | [ ] |

### 9.4 Light Sensor (ISL29023)
```bash
cat /sys/bus/iio/devices/iio:device*/in_illuminance_input
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device detection | Check IIO devices | isl29023 present | [ ] |
| Dark reading | Cover sensor | Low value | [ ] |
| Light reading | Expose to light | High value | [ ] |
| Auto-brightness | Enable auto-brightness | Screen adjusts to light | [ ] |

---

## 10. LED Controller

### 10.1 LM8502 LED Controller
```bash
# Find LED devices
ls /sys/class/leds/

# Test LEDs
echo 255 > /sys/class/leds/*/brightness
echo 0 > /sys/class/leds/*/brightness
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device detection | Check /sys/class/leds/ | LED devices present | [ ] |
| Center button LED | Set brightness | LED illuminates | [ ] |
| LED patterns | Set pattern/trigger | Pattern plays | [ ] |
| Charging indicator | Plug in charger | LED indicates charging | [ ] |

---

## 11. Battery and Charging

### 11.1 A6 Battery Controller
```bash
# Check power supply devices
ls /sys/class/power_supply/

# Read battery status
cat /sys/class/power_supply/battery/status
cat /sys/class/power_supply/battery/capacity
cat /sys/class/power_supply/battery/voltage_now
cat /sys/class/power_supply/battery/current_now
cat /sys/class/power_supply/battery/temp
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Battery detection | Check power_supply | Battery device present | [ ] |
| Capacity reading | Read capacity | Shows 0-100% | [ ] |
| Voltage reading | Read voltage | ~3.7-4.2V range | [ ] |
| Current reading | Read current | Shows mA draw | [ ] |
| Temperature | Read temp | Reasonable value (20-40C) | [ ] |
| Status updates | Monitor over time | Values update | [ ] |

### 11.2 MAX8903B Charger
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Charger detection | Plug in USB | Charger detected | [ ] |
| Charging status | Check status | Shows "Charging" | [ ] |
| Charge current | Read current | Positive (charging) | [ ] |
| Full charge | Charge to 100% | Shows "Full" | [ ] |
| Unplug detection | Remove charger | Shows "Discharging" | [ ] |
| Dock charging | Use dock | Charges at higher rate | [ ] |

### 11.3 Dual A6 Controllers
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| A6_0 (primary) | Check I2C 0x31 | Responds | [ ] |
| A6_1 (secondary) | Check I2C 0x32 | Responds | [ ] |
| Consistent readings | Compare both | Values match | [ ] |

---

## 12. USB

### 12.1 USB Device Mode (Gadget)
```bash
# Check USB gadget
ls /sys/class/udc/

# Enable mass storage gadget (example)
modprobe g_mass_storage file=/path/to/image.img
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| UDC detection | Check /sys/class/udc/ | UDC present | [ ] |
| Mass storage | Enable gadget | Shows as USB drive on PC | [ ] |
| ADB gadget | Enable ADB | adb devices shows device | [ ] |
| RNDIS/ECM | Enable network gadget | Network interface on PC | [ ] |

### 12.2 USB OTG Host Mode
```bash
# Connect USB device via OTG adapter
lsusb
dmesg | tail -20
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| OTG detection | Connect OTG cable | Switches to host mode | [ ] |
| USB keyboard | Connect keyboard | Input device appears | [ ] |
| USB mouse | Connect mouse | Input device appears | [ ] |
| USB storage | Connect flash drive | Block device appears | [ ] |
| USB hub | Connect via hub | Devices work through hub | [ ] |

### 12.3 ISP1763 USB Host (3G variant only)
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Controller detection | Check dmesg | isp1763 initialized | [ ] |
| Device enumeration | Connect USB device | Device detected | [ ] |
| Modem connection | Check modem USB | Modem interfaces appear | [ ] |

---

## 13. GPU

### 13.1 Adreno A220 Graphics
```bash
# Check DRM device
cat /sys/class/drm/card0/device/uevent

# Run GPU benchmark
glmark2-es2
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| GPU detection | Check DRM | adreno detected | [ ] |
| OpenGL ES 2.0 | Run glmark2-es2 | Benchmark completes | [ ] |
| 3D rendering | Run 3D application | Renders correctly | [ ] |
| GPU frequency | Check OPP table | Shows 200/266 MHz | [ ] |
| GPU thermal | Check temp under load | Stays within limits | [ ] |

### 13.2 GPU Power Management
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Idle downlock | Leave GPU idle | Frequency drops | [ ] |
| Load scaling | Run benchmark | Frequency increases | [ ] |
| Power gating | Extended idle | GPU powers down | [ ] |

---

## 14. Cameras

### 14.1 Topaz Front Camera (Parallel Interface)
```bash
# Check video devices
v4l2-ctl --list-devices

# Capture test frame
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=YUYV
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=1 --stream-to=test.raw
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Camera detection | v4l2-ctl --list-devices | Camera present | [ ] |
| Preview | Open camera app | Live preview works | [ ] |
| Photo capture | Take photo | Image saved correctly | [ ] |
| Video recording | Record video | Video file created | [ ] |
| Resolution modes | Switch resolutions | All modes work | [ ] |

### 14.2 Opal Front Camera (MT9M113 - MIPI CSI-1)
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Camera detection | v4l2-ctl --list-devices | MT9M113 present | [ ] |
| MIPI interface | Check dmesg | CSI-1 link established | [ ] |
| 1.3MP capture | Take photo | 1280x960 image | [ ] |
| Preview | Open camera app | Live preview works | [ ] |

### 14.3 Opal Rear Camera (VX6953 - MIPI CSI-0)
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Camera detection | v4l2-ctl --list-devices | VX6953 present | [ ] |
| MIPI interface | Check dmesg | CSI-0 link established | [ ] |
| 5.1MP capture | Take photo | 2608x1960 image | [ ] |
| EDOF focus | Capture at various distances | All in focus (EDOF) | [ ] |
| Flash LED | Take photo with flash | Flash fires | [ ] |

---

## 15. HDMI Output

### 15.1 HDMI (WiFi variants only - GPIO 172 conflict on 3G)
```bash
# Check HDMI connector
cat /sys/class/drm/card0-HDMI-A-1/status

# List modes
cat /sys/class/drm/card0-HDMI-A-1/modes
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| HDMI detection | Connect HDMI cable | Connector shows "connected" | [ ] |
| Hot plug | Connect while running | Display detected | [ ] |
| 720p output | Set 720p mode | Output works | [ ] |
| 1080p output | Set 1080p mode | Output works | [ ] |
| Audio over HDMI | Play audio | Sound on TV/monitor | [ ] |
| Mirror mode | Enable mirroring | Both displays show same | [ ] |
| Extended mode | Enable extended | Separate displays | [ ] |

---

## 16. 3G Modem

### 16.1 MDM6600 Modem (3G variants only)
```bash
# Check modem USB interfaces
lsusb | grep Qualcomm

# Check modem AT interface
ls /dev/ttyUSB*
screen /dev/ttyUSB0 115200
AT
OK
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Modem detection | lsusb | MDM6600 present | [ ] |
| AT commands | Send AT | Returns OK | [ ] |
| SIM detection | AT+CPIN? | SIM status reported | [ ] |
| Network registration | AT+CREG? | Registered to network | [ ] |
| Signal strength | AT+CSQ | Signal level reported | [ ] |
| Data connection | Establish PPP/QMI | Internet via 3G | [ ] |
| SMS | Send/receive SMS | Messages work | [ ] |
| Voice call | Make/receive call | Audio works | [ ] |

---

## 17. Video Codec (VIDC)

### 17.1 Hardware Video Decoding
```bash
# Check V4L2 M2M device
v4l2-ctl --list-devices | grep -A2 vidc

# Test decode (requires gstreamer or ffmpeg with V4L2 M2M support)
gst-launch-1.0 filesrc location=test.mp4 ! qtdemux ! h264parse ! v4l2h264dec ! videoconvert ! autovideosink
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| VIDC detection | Check V4L2 devices | vidc decoder/encoder present | [ ] |
| H.264 decode | Decode H.264 video | Plays smoothly | [ ] |
| MPEG4 decode | Decode MPEG4 video | Plays smoothly | [ ] |
| H.263 decode | Decode H.263 video | Plays smoothly | [ ] |
| 720p decode | Decode 720p H.264 | Smooth playback | [ ] |
| 1080p decode | Decode 1080p H.264 | Smooth playback | [ ] |

### 17.2 Hardware Video Encoding
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| H.264 encode | Encode to H.264 | Valid output file | [ ] |
| MPEG4 encode | Encode to MPEG4 | Valid output file | [ ] |
| Camera recording | Record from camera | Hardware-encoded video | [ ] |

---

## 18. Audio DSP (LPASS)

### 18.1 LPASS Remoteproc
```bash
# Check remoteproc
cat /sys/class/remoteproc/remoteproc*/name
cat /sys/class/remoteproc/remoteproc*/state

# Start LPASS
echo start > /sys/class/remoteproc/remoteproc0/state
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Firmware load | Check dmesg | LPASS firmware loaded | [ ] |
| DSP boot | Start remoteproc | State shows "running" | [ ] |
| Audio routing | Play audio via DSP | Audio works | [ ] |
| DSP effects | Enable effects | Effects applied | [ ] |

---

## 19. Power Management

### 19.1 Suspend/Resume
```bash
# Check available sleep states
cat /sys/power/state

# Suspend to RAM
echo mem > /sys/power/state
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Suspend entry | Echo mem to power/state | Device suspends | [ ] |
| Resume (power button) | Press power button | Device resumes | [ ] |
| Resume (touch) | Touch screen | Device resumes | [ ] |
| WiFi after resume | Check WiFi | Reconnects | [ ] |
| Audio after resume | Play audio | Works | [ ] |
| Touch after resume | Touch screen | Responds correctly | [ ] |

### 19.2 Runtime PM
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Device idle | Leave idle | Peripherals power down | [ ] |
| USB runtime PM | Check USB power | Suspends when unused | [ ] |
| GPU runtime PM | Check GPU | Powers down when idle | [ ] |

### 19.3 Thermal Management
```bash
# Check thermal zones
cat /sys/class/thermal/thermal_zone*/type
cat /sys/class/thermal/thermal_zone*/temp
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| Temperature reading | Read thermal zones | Reasonable values | [ ] |
| Throttling | Stress test | Throttles at limit | [ ] |
| Cooling device | Check cooling | Cooling activates | [ ] |

---

## 20. Stress Testing

### 20.1 System Stress
```bash
# CPU stress
stress-ng --cpu 2 --timeout 300s

# Memory stress
stress-ng --vm 2 --vm-bytes 512M --timeout 300s

# I/O stress
stress-ng --io 2 --timeout 300s
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| CPU stress (5 min) | Run stress-ng --cpu | No crashes | [ ] |
| Memory stress (5 min) | Run stress-ng --vm | No OOM | [ ] |
| Combined stress | CPU + memory + I/O | System stable | [ ] |
| Extended stress (1 hr) | Run all stressors | No degradation | [ ] |

### 20.2 Suspend/Resume Cycling
```bash
# Automated suspend/resume test
for i in $(seq 1 100); do
    echo "Cycle $i"
    echo mem > /sys/power/state
    sleep 2
    # Check all devices still work
done
```

| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| 10 cycles | Suspend/resume 10x | All succeed | [ ] |
| 100 cycles | Suspend/resume 100x | All succeed | [ ] |
| With WiFi | Cycles with WiFi active | Reconnects each time | [ ] |
| With BT | Cycles with BT active | Reconnects each time | [ ] |

### 20.3 Peripheral Stress
| Test | Steps | Expected Result | Status |
|------|-------|-----------------|--------|
| WiFi throughput | iperf3 for 1 hour | No disconnects | [ ] |
| BT audio | Stream for 1 hour | No dropouts | [ ] |
| Camera recording | Record for 30 min | No issues | [ ] |
| Touch stress | Rapid multi-touch | No missed touches | [ ] |

---

## Test Results Summary

### Device: _______________
### Date: _______________
### Tester: _______________
### Kernel Version: _______________

| Category | Passed | Failed | Skipped | Notes |
|----------|--------|--------|---------|-------|
| Boot/Basic | /6 | | | |
| Display | /5 | | | |
| Touchscreen | /10 | | | |
| WiFi | /11 | | | |
| Bluetooth | /7 | | | |
| GPS | /6 | | | |
| Audio | /11 | | | |
| Sensors | /13 | | | |
| LEDs | /4 | | | |
| Battery | /14 | | | |
| USB | /13 | | | |
| GPU | /8 | | | |
| Cameras | /13 | | | |
| HDMI | /7 | | | |
| 3G Modem | /8 | | | |
| VIDC | /9 | | | |
| LPASS | /4 | | | |
| Power Mgmt | /10 | | | |
| Stress | /10 | | | |
| **TOTAL** | /169 | | | |

---

## Known Issues / Limitations

1. **3G variant**: HDMI output not available (GPIO 172 conflict with ISP1763)
2. **A6 simple_strtoul**: One instance cannot be converted to kstrtoul (loop parser)
3. **VIDC**: Requires firmware files from device
4. **LPASS**: Requires Hexagon DSP firmware from device

---

## Firmware Requirements

| Component | Firmware File | Source |
|-----------|---------------|--------|
| WiFi (ath6kl) | ath6k/AR6003/hw2.1.1/* | linux-firmware |
| Bluetooth (BCM4330) | bcm4330.hcd | Device extraction |
| VIDC | vidc_1080p.fw | Device extraction |
| LPASS | q6.mdt, q6.b* | Device extraction |
| Touchscreen | cy8ctma395.fw | Device extraction |

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-02 | Claude | Initial test plan |
