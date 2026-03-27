# HP TouchPad Connection Guide

## Prerequisites
- HP TouchPad with moboot bootloader
- USB cable connected to host
- Kernel built and packed as uImage.LuneOS
- initramfs-uImage.bin in parent directory (../initramfs-uImage.bin)

## uImage Format Requirements

**CRITICAL**: moboot has specific requirements for the uImage format:

1. **Multi-file uImage**: Load/entry address must be `0x00000000`
2. **Kernel uImage (Image 0)**: Load/entry address `0x40208000`
3. **Initramfs uImage (Image 1)**: Compression header must be `none` (NOT `gzip`)
   - Even if the data is gzip compressed, the header must say "uncompressed"
   - moboot checks the header and rejects images marked as gzip

The `pack-uimage.sh` script handles all of this automatically.

## Building and Deploying the Kernel

### 1. Build the kernel
```bash
cd /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc) zImage modules dtbs
```

### 2. Pack the kernel into uImage format
```bash
./scripts/pack-uimage.sh topaz
# Output: ../build-output/uImage.LuneOS
# Also creates: ../build-output/moboot.next
```

### 3. Deploy to TouchPad (via novacom - requires webOS booted)
```bash
./scripts/deploy-to-touchpad.sh
```

Or manually:
```bash
# Remount /boot read-write
novacom run file://bin/mount -- -o remount,rw /boot

# Push kernel and moboot.next (for auto-boot)
novacom put file:///boot/uImage.LuneOS < ../build-output/uImage.LuneOS
novacom put file:///boot/moboot.next < ../build-output/moboot.next

# Sync and reboot
novacom run file://bin/sync
novacom run file://sbin/tellbootie -- reboot
```

## Connecting After Boot

### 1. Wait for device to enumerate
After moboot loads LuneOS, the USB gadget will enumerate. Check with:
```bash
lsusb | grep -i linux
```

Expected output shows kernel version:
```
Bus 001 Device XXX: ID 0525:a4a2 Netchip Technology, Inc. Linux-USB Ethernet/RNDIS Gadget
```

Check dmesg for manufacturer string:
```bash
dmesg | tail -20
```
Should show: `Manufacturer: Linux 6.18.0-XXXXX with ci_hdrc_msm`

### 2. Find the USB network interface
```bash
ip addr show | grep enx
```
Interface will be named like `enxXXXXXXXXXXXX` (MAC-based name)

### 3. Configure host network
The initramfs configures the device with IP `172.16.42.2/16`. Configure the host:
```bash
# Find interface name first
IFACE=$(ip -o link show | grep -oP 'enx[a-f0-9]+')

# Configure host IP
sudo ip addr add 172.16.42.1/16 dev $IFACE
sudo ip link set $IFACE up
```

### 4. Test connectivity
```bash
ping -c 3 172.16.42.2
```

### 5. Connect to the device

**Debug/initramfs mode** (telnet):
```bash
telnet 172.16.42.2
```

**Full LuneOS boot** (SSH):
```bash
ssh root@172.16.42.2
# Password: (blank - just press Enter)
```

## Troubleshooting

### Device not enumerating
- Check USB cable connection
- Device may still be at moboot menu - select LuneOS
- Device may be out of battery - charge it
- Check `lsusb | grep -E "0525|0830"` for device state

### No route to host / Packet filtered
- Host IP not configured on USB interface
- Run the sudo commands to configure the interface IP
- Check `ip addr show` to verify the interface has 172.16.42.1

### Connection refused
- SSH (port 22): Not available in initramfs debug mode, use telnet
- Telnet (port 23): telnetd may not have started yet, wait a few seconds

### moboot shows "compression not supported"
The initramfs uImage has wrong compression header. The script should auto-fix this,
but if using a custom initramfs, ensure it's packed with `-C none`:
```bash
mkimage -A arm -O linux -T ramdisk -C none -a 0 -e 0 -n "initramfs" -d initramfs.cpio.gz initramfs-uImage.bin
```

## Network Configuration Summary

| Device | IP Address | Netmask |
|--------|------------|---------|
| TouchPad | 172.16.42.2 | /16 (255.255.0.0) |
| Host | 172.16.42.1 | /16 (255.255.0.0) |

## Quick Reference Commands

```bash
# Build everything
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc) zImage modules dtbs

# Pack uImage (creates ../build-output/uImage.LuneOS and moboot.next)
./scripts/pack-uimage.sh topaz

# Deploy (webOS must be running)
novacom run file://bin/mount -- -o remount,rw /boot
novacom put file:///boot/uImage.LuneOS < ../build-output/uImage.LuneOS
novacom put file:///boot/moboot.next < ../build-output/moboot.next
novacom run file://bin/sync
novacom run file://sbin/tellbootie -- reboot

# After reboot, configure host network
IFACE=$(ip -o link show | grep -oP 'enx[a-f0-9]+')
sudo ip addr add 172.16.42.1/16 dev $IFACE
sudo ip link set $IFACE up

# Connect (debug mode uses telnet, full boot uses SSH)
telnet 172.16.42.2      # initramfs debug mode
ssh root@172.16.42.2    # full LuneOS boot
```
