# Next Session Plan - HP TouchPad Kernel Development

## Session Summary (2026-01-23)

Successfully implemented SMI memory support for display framebuffers, which should eliminate USB/display bandwidth contention. The root cause was MDP and USB competing for EBI memory bandwidth on the APPSS fabric. By routing MDP to SMI memory on the MMSS fabric, there's no longer a shared path.

### Commits Made

1. `181b2a06d4c1` - ARM: dts: qcom: tenderloin: Use SMI memory for display framebuffers
2. `c33d3be62f7d` - ARM: dts: qcom: tenderloin: Remove USB bandwidth voting

### What's Working

- MDP successfully allocates framebuffer from SMI reserved region at 0x38300000
- Display shows 2x Tux penguins (fbcon active)
- CMA disabled to force SMI usage (matches webOS)
- MDP bandwidth set to webOS mdp_app values (377/471 MB/s)

---

## Priority 1: Test SMI + USB Stability

**Goal**: Verify that SMI memory usage eliminates USB crashes during display activity

### Steps

1. **Deploy kernel** (deployment was failing with MD5 mismatch)
   ```bash
   ./scripts/deploy-to-touchpad.sh --reboot
   ```
   If MD5 issues persist, try manual deployment:
   ```bash
   # Start HTTP server
   python3 -m http.server 8080 &
   # On device via telnet:
   wget -O /mnt/boot/uImage.LuneOS http://172.16.42.1:8080/build-output/uImage.LuneOS
   reboot
   ```

2. **Deploy DRM tests**
   ```bash
   ./scripts/pack-drm-tests.sh
   scp ../build-output/drm-tests.tar.gz root@172.16.42.2:/tmp/
   # On device:
   cd /tmp && tar xzf drm-tests.tar.gz
   ```

3. **Run USB stability tests**
   ```bash
   # Test 1: modetest pattern (MDP only)
   ./diagnose-drm.sh 2
   # Keep running for 60+ seconds, verify USB responsive

   # Test 2: kmscube hardware rendering (MDP + GPU)
   ./diagnose-drm.sh 8
   # Keep running for 30+ seconds, verify USB responsive

   # Test 3: Combined stress
   # Run kmscube while doing network activity
   ```

### Expected Results

- USB should remain stable during all display activity
- If USB still crashes, the GPU (Z180) may also need SMI routing

---

## Priority 2: If USB Still Crashes - Route GPU to SMI

The Z180 GPU is currently using EBI for memory access. If USB issues persist:

1. **Check current GPU interconnect configuration**
   ```bash
   grep -A5 "z180" arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi
   ```

2. **Update GPU to use SMI** (similar to MDP changes)
   - Add GPU interconnects to MMFAB_SLV_SMI
   - May need separate reserved region or share with MDP

3. **Consider**: GPU uses much less bandwidth than MDP scanout, so this may not be necessary

---

## Priority 3: Audio Bring-up (WIP)

There's a WIP audio commit (`451f3cd9a9b3`) that needs completion.

### Current State

- ASoC machine driver exists for APQ8060
- Codec (WM8958) needs verification
- Audio paths need testing

### Next Steps

1. Check if audio codec is probing correctly
2. Test audio playback via speaker and headphone
3. Debug any missing clock or regulator configurations

---

## Priority 4: Cleanup Uncommitted Changes

Current uncommitted changes that may need attention:

```
modified:   drivers/media/platform/qcom/vidc/vidc_core.c
modified:   drivers/media/platform/qcom/vidc/vidc_core.h
modified:   scripts/pack-drm-tests.sh
untracked:  reports/usb-display-bandwidth-investigation.md
```

Decide whether to:
- Commit the VIDC changes (if they're ready)
- Discard if they were experimental
- Add the investigation report to git

---

## Reference Commands

```bash
# Build kernel
export ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
make -j$(nproc) zImage dtbs

# Pack and deploy
./scripts/pack-uimage.sh topaz
./scripts/deploy-to-touchpad.sh --reboot

# Check device logs
telnet 172.16.42.2
cat /proc/kmsg | head -100
dmesg | grep -E "(mdp|usb|smi|drm)"

# Monitor USB health
cat /sys/class/udc/ci_hdrc.0/state
```

---

## Architecture Reference

```
SMI Memory Layout (webOS compatible):
0x38000000 - 0x382FFFFF: 3MB kernel reserved
0x38300000 - 0x3BFFFFFF: 61MB drm_smi_mem (framebuffers)

Memory Paths:
- MDP → MMSS Fabric → SMI (no EBI contention)
- GPU → MMSS Fabric → EBI (still uses main RAM)
- USB → System Fabric → EBI
```
