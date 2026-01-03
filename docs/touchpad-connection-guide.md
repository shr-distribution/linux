# HP TouchPad Connection Guide

## Prerequisites
- HP TouchPad with moboot bootloader
- USB cable connected to host
- Kernel built and packed as uImage.LuneOS

## Building and Deploying the Kernel

### 1. Build the kernel
```bash
cd /home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc) zImage modules dtbs
```

### 2. Pack the kernel into uImage format
```bash
./scripts/pack-uimage.sh topaz
# Output: ../uImage.LuneOS
```

### 3. Deploy to TouchPad (via novacom - requires webOS booted)
```bash
./scripts/deploy-to-touchpad.sh
```

Or manually:
```bash
# Remount /boot read-write
novacom run "file:///bin/mount" -- -o remount,rw /boot

# Push kernel
novacom put file:///boot/uImage.LuneOS < ../uImage.LuneOS

# Set default boot to LuneOS
echo "LuneOS" | novacom put file:///boot/moboot.default

# Remount /boot read-only
novacom run "file:///bin/mount" -- -o remount,ro /boot

# Reboot
novacom run file:///sbin/reboot
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

### 5. SSH into the device
```bash
ssh root@172.16.42.2
# Password: (blank - just press Enter)
```

## Troubleshooting

### Device not enumerating
- Check USB cable connection
- Device may still be at moboot menu - select LuneOS
- Device may be out of battery - charge it

### No route to host
- Check if interface is up: `ip link show $IFACE`
- Reconfigure IP if needed

### SSH connection reset
- Device may be in early boot (SSH not ready yet)
- Wait a few more seconds and retry
- Check if dropbear/SSH daemon is running on device

### Device boots to recovery/telnet instead of SSH
The initramfs has two modes:
- **Normal boot**: Mounts luneos-root and switches to it (SSH via dropbear on rootfs)
- **Recovery mode**: Stays in initramfs with telnetd (connect via telnet, not SSH)

If telnet works but SSH doesn't, the device may be stuck in recovery mode.

## Network Configuration Summary

| Device | IP Address | Netmask |
|--------|------------|---------|
| TouchPad | 172.16.42.2 | /16 (255.255.0.0) |
| Host | 172.16.42.1 | /16 (255.255.0.0) |

## Quick Reference Commands

```bash
# Build everything
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc) zImage modules dtbs

# Pack uImage
./scripts/pack-uimage.sh topaz

# Deploy (webOS must be running)
./scripts/deploy-to-touchpad.sh

# After reboot, configure host network
IFACE=$(ip -o link show | grep -oP 'enx[a-f0-9]+')
sudo ip addr add 172.16.42.1/16 dev $IFACE
sudo ip link set $IFACE up

# Connect
ssh root@172.16.42.2
```
