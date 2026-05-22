#!/usr/bin/env python3
"""HP TouchPad LM8502 wake helper.

The kernel driver's regmap-based chip_init runs at probe time (~2 s
into boot) and somehow doesn't wake the chip on mainline — every
register read returns 0xff (bus pull-up state) and brightness writes
have no visible effect. Doing the same init sequence through raw
I2C_RDWR ioctl later in boot DOES wake the chip; once awake, the
kernel sysfs path (`/sys/class/leds/lm8502:.../brightness`) works
normally.

Run this once after boot to make the navi LEDs usable. Then drive
brightness via sysfs or run lm-knight-rider.sh.

The chip dies again if the lm8502 driver unbinds (regulator l16 drops
to LPM ~1 mA without the driver's set_load(100 mA) vote).
"""
import fcntl, os, time, ctypes

I2C_SLAVE_FORCE = 0x0706
I2C_RDWR        = 0x0707
I2C_M_RD        = 0x0001
BUS, ADDR       = "/dev/i2c-2", 0x33

class i2c_msg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16), ("buf", ctypes.POINTER(ctypes.c_uint8))]

class i2c_rdwr_ioctl_data(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(i2c_msg)), ("nmsgs", ctypes.c_uint32)]

def w(fd, reg, val):
    b = (ctypes.c_uint8 * 2)(reg, val)
    m = (i2c_msg * 1)(i2c_msg(addr=ADDR, flags=0, len=2,
                              buf=ctypes.cast(b, ctypes.POINTER(ctypes.c_uint8))))
    fcntl.ioctl(fd, I2C_RDWR, i2c_rdwr_ioctl_data(msgs=m, nmsgs=1))

fd = os.open(BUS, os.O_RDWR)
fcntl.ioctl(fd, I2C_SLAVE_FORCE, ADDR)

# webOS-style init: RESET → CHIP_EN → MISC → per-LED CONTROL/CURRENT
w(fd, 0x3D, 0xFF); time.sleep(0.05)       # software reset
w(fd, 0x00, 0x40); time.sleep(0.01)       # ENGINE_CNTRL1: CHIP_EN
w(fd, 0x36, 0x6A); time.sleep(0.01)       # MISC: POWER_SAVE + BOOST_EN + PWM_INPUT
for led in range(10):
    w(fd, 0x06 + led, 0x10)               # D*_CONTROL: max_current=2 (9 mA)
    w(fd, 0x26 + led, 0x00)               # D*_CURRENT_CTRL: 0 (off)

os.close(fd)
print("LM8502 woken; sysfs brightness should now work")
