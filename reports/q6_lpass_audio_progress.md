# Q6 LPASS Audio Subsystem - Progress Report

**Date:** 2026-01-09 (Updated)
**Device:** HP TouchPad (APQ8060/MSM8660)
**Kernel:** Linux 6.18
**Build:** #117

## Executive Summary

Significant progress has been made in bringing up the Q6 LPASS (Low Power Audio Subsystem) on the HP TouchPad with mainline Linux. The Q6 DSP boots successfully, creates SMD channels (apr_audio_svc, DIAG, DIAG_CNTL), and the IPC mechanism is working. However, Q6 doesn't respond to SMD channel open requests yet.

## What's Working

### 1. Q6 Firmware Loading
- **MDT loader** successfully loads Q6 firmware segments
- **TCM segments** (at 0x28400000) are now properly skipped instead of causing errors
- Firmware relocates to 0x46700000 with 6MB memory allocation

### 2. Q6 DSP Boot Sequence
- **PLL4** enabled via RPM successfully powers up LPASS domain
- **LCC (LPASS Clock Controller)** registers accessible
- **QDSP6SS** registers programmed correctly:
  - RST_EVB (boot address)
  - STRAP_TCM
  - STRAP_AHB
- **Q6 starts successfully**: "remote processor 28800000.remoteproc is now up"

### 3. SMD Subdevice Registration
- `qcom_add_smd_subdev()` now called in q6v2_lpass driver
- SMD edge node found in device tree
- `qcom_smd_register_edge()` returns success (0)

### 4. SMD Channel Discovery
- **Channels found by Q6**: apr_audio_svc, DIAG, DIAG_CNTL
- Q6 firmware creates these channels in SMEM
- Mainline SMD driver successfully scans and finds them

### 5. SMSM/SMD Infrastructure
- **SMSM probe succeeds** with shared IRQ handling
- **Legacy SMSM state flags (0x129)** set during probe:
  - SMSM_INIT (0x01), SMSM_SMDINIT (0x08), SMSM_RPCINIT (0x20), SMSM_RUN (0x100)
- **IRQ sharing** between SMSM and SMD on GIC_SPI 89 working

### 6. IPC Mechanism
- **LPASS IPC register** at 0xC0182000 (gcc_lpass syscon)
- **IPC signals sent successfully**: offset=0x8, bit=8
- Confirmed by debug logging: `SMD_IPC: signal 'apr_audio_svc' offset=0x8 bit=8 val=0x100 ret=0`

### 7. Device Tree Configuration
- LPASS remoteproc node with smd-edge child
- **gcc_lpass** syscon at 0xC0182000 for LPASS IPC (different from KPSS GCC at 0x02082000)
- IPC configured: `qcom,ipc = <&gcc_lpass 0x8 8>` for LPASS interrupts
- SMSM IPC properties: ipc-1 (RPM), ipc-2 (LPASS)
- APR services defined (q6core, q6afe, q6asm, q6adm)

## Key Code Changes

### drivers/soc/qcom/mdt_loader.c
```c
// Skip segments outside allocated memory range (TCM/LPM)
if (offset < 0 || offset + phdr->p_memsz > mem_size) {
    dev_info(dev, "skipping segment %d at 0x%08x...\n", ...);
    continue;
}
```

### drivers/soc/qcom/smsm.c
```c
// Handle -ENXIO as "not found" (legacy SMEM compatibility)
if (ret == -ENOENT || ret == -ENXIO)
    smsm->state = state;

// Use IRQF_SHARED for IRQ sharing with SMD
ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, smsm_intr,
    IRQF_ONESHOT | IRQF_SHARED, "smsm", smsm);

// Set legacy SMSM flags for MSM8660/APQ8060 compatibility
#define SMSM_INIT      0x00000001
#define SMSM_SMDINIT   0x00000008
#define SMSM_RPCINIT   0x00000020
#define SMSM_RUN       0x00000100
smsm_update_bits(smsm, 0xffffffff, SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT | SMSM_RUN);
```

### drivers/rpmsg/qcom_smd.c
```c
// Use threaded IRQ with IRQF_SHARED for IRQ sharing with SMSM
ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, qcom_smd_edge_intr,
    IRQF_ONESHOT | IRQF_SHARED, node->name, edge);

// IPC debug logging
pr_info("SMD_IPC: signal '%s' offset=0x%x bit=%d val=0x%x ret=%d\n",
    channel->name, edge->ipc_offset, edge->ipc_bit, BIT(edge->ipc_bit), ret);
```

### drivers/remoteproc/qcom_q6v2_lpass.c
```c
// Added SMD subdevice support
struct qcom_rproc_subdev smd_subdev;
...
qcom_add_smd_subdev(rproc, &q6v2->smd_subdev);
```

### arch/arm/boot/dts/qcom/qcom-msm8660.dtsi
```dts
// Separate syscon for LPASS IPC (not KPSS GCC)
gcc_lpass: gcc@c0182000 {
    compatible = "syscon";
    reg = <0xc0182000 0x1000>;
};

lpass_mem: lpass@46700000 {
    reg = <0x46700000 0x600000>;  // 6MB
    no-map;
};

smd-edge {
    interrupts = <GIC_SPI 89 IRQ_TYPE_EDGE_RISING>;
    qcom,ipc = <&gcc_lpass 0x8 8>;  // LPASS IPC via gcc_lpass
    qcom,smd-edge = <1>;
    label = "lpass";

    apr: apr { ... };
};

// SMSM IPC configuration
qcom,ipc-1 = <&l2cc 0x8 5>;        // RPM IPC
qcom,ipc-2 = <&gcc_lpass 0x8 8>;   // LPASS IPC
```

## Current Status

### SMD Edge Registration (SUCCESS)
```
SMD_SUBDEV: qcom_add_smd_subdev called for rproc 28800000.remoteproc
SMD_SUBDEV: found smd-edge node /soc/remoteproc@28800000/smd-edge
SMD_SUBDEV: subdev added successfully
SMD_SUBDEV: start() called, dev=remoteproc0 node=smd-edge
SMD_SUBDEV: qcom_smd_register_edge returned 0
```

### SMD Channels Found (SUCCESS)
```
qcom_smd: apr_audio_svc found on lpass edge (info=0, ref=12)
qcom_smd: DIAG found on lpass edge (info=1, ref=12)
qcom_smd: DIAG_CNTL found on lpass edge (info=2, ref=12)
```

### SMSM State Set (SUCCESS)
```
qcom-smsm smsm: set legacy SMSM state: 0x129
```

### IPC Signals Sent (SUCCESS)
```
SMD_IPC: signal 'apr_audio_svc' offset=0x8 bit=8 val=0x100 ret=0
```

### What's NOT Working Yet

1. **Q6 Not Responding to Channel Open**
   - IPC signals sent correctly (ret=0)
   - But remote_state stays 0 (not entering OPENING state)
   - Timeout: "remote side did not enter opening state"

2. **No RPMSG Devices**
   - `/sys/bus/rpmsg/devices/` empty
   - APR driver not probing due to channel not opening

3. **No Sound Cards**
   - No ALSA sound cards registered
   - WM8994 codec detected on I2C but not bound to audio

## Remaining Work

### High Priority (Current Blocker)
1. **Debug Q6 not responding to SMD channel open**
   - IPC is sent correctly, but Q6 ignores it
   - Possible causes:
     - Q6 interrupt line not connected/enabled
     - Firmware not in correct state to process IPC
     - Wrong IPC bit (webOS may use different bit for different channels)
     - SMSM handshake incomplete
   - Investigation needed:
     - Check webOS kernel for exact IPC sequence
     - Verify Q6 interrupt configuration in firmware
     - Check if additional SMSM states needed

2. **APR communication**
   - Once SMD channels open, verify APR messages
   - Test basic APR commands to Q6

### Medium Priority
3. **Audio path configuration**
   - Configure Q6AFE for audio routing
   - Set up WM8994 codec via Q6 DSP

4. **ALSA integration**
   - Verify qdsp6 ASoC drivers load
   - Test PCM playback path

## Technical Details

### Memory Map
| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| SMEM | 0x40000000 | 2MB | Shared memory |
| LPASS | 0x46700000 | 6MB | Q6 firmware |
| TCM | 0x28400000 | N/A | Q6 internal (skipped) |
| GCC_LPASS | 0xC0182000 | 4KB | LPASS IPC register |
| KPSS_GCC | 0x02082000 | 4KB | RPM IPC register |

### IPC Mechanism
- **APPS → LPASS**: Write to 0xC0182008 (gcc_lpass + 0x8), bit 8
  - **NOT** L2CC (0x02082008) which is for RPM
  - webOS kernel uses MSM_GCC_BASE (0xC0182000) for DSP signaling
- **LPASS → APPS**: GIC SPI interrupt 89

### SMD Edge Configuration
- Edge ID: 1 (LPASS)
- Channels found: apr_audio_svc, DIAG, DIAG_CNTL
- Protocol: APR v2

### SMSM State Machine
- **Legacy state flags**: 0x129 (SMSM_INIT|SMSM_SMDINIT|SMSM_RPCINIT|SMSM_RUN)
- Must be set before Q6 will respond to SMD open requests

## Files Modified

1. `drivers/soc/qcom/mdt_loader.c` - TCM segment skip
2. `drivers/soc/qcom/smsm.c` - ENXIO handling, IRQF_SHARED, legacy state flags
3. `drivers/rpmsg/qcom_smd.c` - IRQF_SHARED/ONESHOT, IPC debug logging
4. `drivers/remoteproc/qcom_q6v2_lpass.c` - SMD subdev registration
5. `drivers/remoteproc/qcom_common.c` - SMD subdev debug logging
6. `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi` - LPASS memory, gcc_lpass syscon, IPC
7. `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - APR enable
8. `arch/arm/configs/tenderloin_defconfig` - Audio configs

## Debug Commands

```bash
# Check remoteproc status
cat /sys/class/remoteproc/remoteproc0/state

# Check SMD edge
ls /sys/bus/rpmsg/devices/

# Check for sound cards
cat /proc/asound/cards

# SMD/APR dmesg
dmesg | grep -i "smd\|apr\|rpmsg\|channel"
```

## References

### webOS kernel sources (MSM8660 reference)
- `arch/arm/mach-msm/smd.c` - Original SMD implementation, smsm_change_state()
- `arch/arm/mach-msm/smd_private.h` - msm_a2m_int() IPC function
- `arch/arm/mach-msm/include/mach/msm_iomap-7x30.h` - MSM_GCC_PHYS = 0xC0182000
- `arch/arm/mach-msm/qdsp6v2/` - Q6 audio drivers

### Mainline Linux
- `drivers/rpmsg/qcom_smd.c` - SMD implementation
- `drivers/soc/qcom/smsm.c` - SMSM state machine
- `drivers/remoteproc/qcom_q6v2_lpass.c` - Q6 remoteproc driver
- `sound/soc/qcom/qdsp6/` - QDSP6 audio drivers

## Commits

- `a59e6831b159` - soc/rpmsg/dts: Enable Q6 LPASS SMD communication on MSM8660/APQ8060
- `fc45483bee85` - remoteproc: qcom: Add SMD subdevice to Q6V2 LPASS driver
