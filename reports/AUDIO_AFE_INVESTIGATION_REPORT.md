# HP TouchPad Audio AFE Investigation Report

**Date:** 2026-01-21
**Status:** Investigation in progress
**Author:** Generated with assistance from Claude Code

## Executive Summary

Audio playback on the HP TouchPad with mainline Linux 6.18 fails due to the Q6 LPASS DSP not responding to AFE (Audio Front End) commands. Despite implementing legacy AFE protocol support matching the webOS kernel, the DSP times out (-110 ETIMEDOUT) on all AFE commands.

Root cause analysis reveals that while the Q6 DSP boots and runs (remoteproc state = "running"), it **never acknowledges SMD channel opens**, causing all APR (Audio Packet Router) communication to fail.

## Update: IPC Bit Fix (2026-01-21) - CRITICAL BREAKTHROUGH

### Root Cause: Wrong IPC Register and Bits

The device tree was using **incorrect IPC configuration** for LPASS:

**Before (Wrong):**
```dts
gcc_lpass: gcc@c0182000 {  /* MSM7x30 address, NOT MSM8660! */
    compatible = "syscon";
    reg = <0xc0182000 0x1000>;
};

smd-edge {
    qcom,ipc = <&gcc_lpass 0x8 8>;  /* Wrong register, wrong bit */
};

smsm {
    qcom,ipc-1 = <&l2cc 0x8 5>;     /* Wrong bit for modem */
    qcom,ipc-2 = <&gcc_lpass 0x8 8>; /* Wrong everything for LPASS */
};
```

**After (Correct - from webOS kernel):**
```dts
/* All IPC uses KPSS GCC at 0x02082000 (l2cc node) */
smd-edge {
    qcom,ipc = <&l2cc 0x8 15>;  /* MSM_TRIG_A2Q6_SMD_INT = bit 15 */
};

smsm {
    qcom,ipc-1 = <&l2cc 0x8 4>;   /* MSM_TRIG_A2M_SMSM_INT = bit 4 */
    qcom,ipc-2 = <&l2cc 0x8 14>;  /* MSM_TRIG_A2Q6_SMSM_INT = bit 14 */
};
```

### IPC Bit Mapping for MSM8660/APQ8060

All IPC uses register at 0x02082008 (l2cc + 0x8):

| Target | SMD bit | SMSM bit | webOS Macro |
|--------|---------|----------|-------------|
| RPM    | -       | 2        | MSM_TRIG_A2LPASS_INT |
| Modem  | 3       | 4        | MSM_TRIG_A2M_SMD/SMSM_INT |
| LPASS  | 15      | 14       | MSM_TRIG_A2Q6_SMD/SMSM_INT |

### Result: Sound Card Now Registers!

With the IPC fix, SMD channels are discovered and the sound card registers:

```
/dev/snd/controlC0
/dev/snd/pcmC0D0p
/dev/snd/pcmC0D0c
/dev/snd/pcmC0D1p
/dev/snd/pcmC0D1c
/dev/snd/timer
```

Mixer controls are accessible via tinymix.

### Remaining Issue: Q6 Still Doesn't Respond

Despite correct IPC, Q6 still doesn't acknowledge SMD channel opens:

```
/proc/interrupts:
 42:          0          0 GIC-0 121 Edge      smsm, smd-edge
```

**Critical observation:** Interrupt count is 0 - Q6 never sends interrupts back to APPS!

- Q6 populates SMEM channel table (channels discovered)
- Q6 never responds to channel opens (remote_state=0)
- Q6 never sends interrupts back (GIC 121 count = 0)
- AFE commands timeout (-110)

---

## Update: SMSM Investigation (2026-01-21)

### Finding: SMSM State Timing Issue

The SMSM (Shared Memory State Machine) driver sets the `SMSM_RUN` flag at **probe time (~8s)**, but Q6 boots much later (~23s). When SMSM sets the state, Q6's subscription mask is 0 (Q6 not running), so no IPC interrupt is sent.

**Timeline:**
- **8.6s**: SMSM probe sets state 0x129 (INIT|SMDINIT|RPCINIT|RUN)
- **8.6s**: All host subscriptions are 0, no kicks sent
- **22.6s**: Q6 boot starts
- **23.5s**: Q6 starts running, populates SMEM channel table
- **23.7s**: SMSM kick sent (after we added the fix)
- **24.0s**: SMD channel created for 'apr_audio_svc'
- **27.0s**: SMD channel still shows remote_state=0 after 3s

### Implemented Fix: Post-Boot SMSM Kick

Added code to kick SMSM hosts after Q6 boots:

1. **smsm.c**: Added `qcom_smsm_kick_hosts()` function that kicks all hosts regardless of subscription
2. **qcom_common.c**: Call `qcom_smsm_kick_hosts()` in `smd_subdev_start()` after SMD edge registers

**Verified correct bits after IPC fix:**
```
SMSM_KICK: host 1 offset=0x8 bit=4 val=0x10   (modem - correct)
SMSM_KICK: host 2 offset=0x8 bit=14 val=0x4000 (LPASS - correct)
```

### Result: Kick Works But Q6 Still Unresponsive

Despite the correct SMSM kick after Q6 boots, the Q6 firmware still doesn't acknowledge SMD channel opens. The remote_state remains at 0.

**Possible reasons:**
1. Q6 SMD code might not be interrupt-driven for SMSM
2. Q6 might check SMSM state only at boot time (before our kick)
3. Q6 might require additional handshake not present in mainline
4. Q6's interrupt handler for APPS IPC might not be configured

### Key Observation

The SMEM channel table IS populated by Q6, confirming Q6 is running. The channels include 'apr_audio_svc' at cid=13. This means Q6's boot sequence is working, but its SMD channel handling isn't responding to opens from APPS.

## Hardware Overview

- **SoC:** Qualcomm APQ8060 (dual-core Scorpion ARMv7 @ 1.5GHz)
- **Audio DSP:** Hexagon QDSP6v2 (LPASS - Low Power Audio SubSystem)
- **Audio Codec:** Wolfson WM8958 (I2S interface)
- **Communication:** SMD (Shared Memory Driver) via APR protocol

## Problem Description

### Symptoms

When attempting audio playback with tinyplay:
```
$ tinyplay /tmp/notification.wav
error playing sample. cannot prepare channel: Connection timed out
```

### Kernel Log (Mainline)
```
qcom-q6afe: Using legacy AFE protocol for MSM8660/APQ8060
qcom-q6afe: Legacy AFE: port 0x1000 -> 6
qcom-q6afe: Legacy AFE config: port=6 bitwidth=16 line=1 channel=3 ws=0
SMD_IPC: signal 'apr_audio_svc' offset=0x8 bit=8 val=0x100 ret=0
qcom-q6afe: AFE legacy config for port 6 failed -110
q6afe-dai: fail to start AFE port 10 (PRIMARY_MI2S_RX)
```

The SMD_IPC signal confirms the packet reaches the SMD channel, but the DSP never responds.

### SMD Channel State
```
SMD_OPEN: LPASS 'apr_audio_svc' remote_state=0 after 3s (continuing anyway)
```

The `remote_state=0` indicates the Q6 DSP is not acknowledging the SMD channel.

## Working Reference: webOS Kernel

### webOS Kernel Log (Working)
```
[24.256179] smd_alloc_channel() 'DAL_AQ_VID' cid=6
[24.271123] smd_alloc_channel() 'DAL_AQ_AUD' cid=7
[24.390012] smd_alloc_channel() 'apr_audio_svc' cid=13
[24.416324] apr_tal:Q6 Is Up
[24.437578] SMD: ch 13 0 -> 1
[24.453373] SMD: ch 13 1 -> 2
[24.463927] apr_tal: SMD_EVENT_OPEN
[24.678516] afe_open: Register AFE
```

Key observations:
1. SMD channels are created by scanning shared memory after Q6 boots
2. Channel state transitions: 0 → 1 → 2 (both sides acknowledge)
3. "Q6 Is Up" is printed when platform device is registered
4. AFE opens successfully after channel is ready

## Architecture Comparison

### webOS Audio Stack

```
┌─────────────────────────────────────────────────────────────┐
│                    Userspace (audiod)                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              /dev/msm_pcm_out, /dev/msm_acdb               │
│                   (MSM-specific devices)                    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    q6afe.c (AFE driver)                     │
│  - Uses legacy opcodes: 0x000100ca, 0x000100cb, 0x000100d3 │
│  - Port IDs: MI2S_RX=6, MI2S_TX=7                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    apr.c / apr_tal.c                        │
│  - apr_register("ADSP", "AFE", callback)                   │
│  - apr_send_pkt() via SMD                                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                         smd.c                               │
│  - smd_alloc_channel() creates platform devices            │
│  - platform_device_register(&ch->pdev) triggers probe      │
│  - Channel state machine: CLOSED → OPENING → OPENED        │
└─────────────────────────────────────────────────────────────┘
                              │
                    ══════════════════════
                    Shared Memory (SMEM)
                    ══════════════════════
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Q6 LPASS DSP Firmware                     │
│  - Populates SMEM channel allocation table                 │
│  - Acknowledges channel opens                              │
│  - Processes AFE commands                                   │
└─────────────────────────────────────────────────────────────┘
```

### Mainline Audio Stack

```
┌─────────────────────────────────────────────────────────────┐
│                 Userspace (tinyplay/ALSA)                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                /dev/snd/pcmC0D0p (ALSA PCM)                │
│                    (Standard ALSA interface)                │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              ASoC Machine Driver (apq8060.c)                │
│  - DPCM frontend/backend DAI links                         │
│  - WM8994 FLL clock configuration                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│         q6afe.c (with legacy protocol support)              │
│  - Detects qcom,legacy-afe-protocol DT property            │
│  - Converts port IDs: 0x1000 → 6                           │
│  - Converts mono/stereo: 1 → 3                             │
│  - Uses legacy opcodes                                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    apr.c (mainline)                         │
│  - Uses RPMSG/SMD transport                                │
│  - apr_send_pkt() via qcom_smd                             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              qcom_smd.c / rpmsg subsystem                   │
│  - Opens channel directly (no platform device)             │
│  - Channel stays at remote_state=0 ← FAILURE POINT         │
└─────────────────────────────────────────────────────────────┘
                              │
                    ══════════════════════
                    Shared Memory (SMEM)
                    ══════════════════════
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Q6 LPASS DSP Firmware                     │
│  - Boots via remoteproc (state = "running")                │
│  - NEVER populates SMEM channel table ← ROOT CAUSE         │
│  - NEVER acknowledges channel opens                        │
└─────────────────────────────────────────────────────────────┘
```

## Root Cause Analysis

### Q6 Boot Sequence Comparison

**webOS PIL (Peripheral Image Loader):**
```c
// peripheral-reset.c: reset_q6_untrusted()
1. local_src_enable(PLL_4)           // Enable LPASS PLL
2. Put Q6 into reset (Q6SS_SS_ARES | STOP_CORE | CORE_ARES)
3. Turn on Q6 memory (CORE_L1_MEM_CORE_EN | CORE_TCM_MEM_*)
4. Take Q6 out of reset
5. Program boot address (QDSP6SS_RST_EVB)
6. Program TCM/AHB straps
7. Clear STOP_CORE → Q6 starts executing
```

**Mainline remoteproc:**
```c
// qcom_q6v2_lpass.c: q6v2_lpass_start_untrusted()
1. clk_prepare_enable(pll)           // Enable LPASS PLL via RPM
2. Put Q6 into reset (identical register writes)
3. Turn on Q6 memory (identical register writes)
4. Take Q6 out of reset (identical register writes)
5. Program boot address (identical)
6. Program TCM/AHB straps (identical)
7. Clear STOP_CORE → Q6 starts executing
```

**Conclusion:** The hardware initialization sequence is **identical**. The Q6 DSP boots and runs in both cases.

### SMD Channel Initialization Difference

**webOS:**
1. Q6 boots and populates SMEM channel allocation table (`ID_CH_ALLOC_TBL`)
2. `smd_channel_probe_worker()` scans this table
3. For each channel found, `smd_alloc_channel()` is called
4. `platform_device_register(&ch->pdev)` creates device with channel name
5. Platform driver (e.g., apr_q6_driver with name "apr_audio_svc") probes
6. `apr_smd_probe()` prints "Q6 Is Up", wakes waiting clients
7. `smd_named_open_on_edge()` opens channel, Q6 acknowledges (state 0→1→2)

**Mainline:**
1. Q6 boots via remoteproc
2. APR driver calls `qcom_smd_open_channel()` directly
3. SMD/RPMSG tries to open channel
4. Q6 **never populates SMEM table / never acknowledges**
5. Channel stays at `remote_state=0`
6. All APR commands timeout

### Key Insight

The Q6 firmware appears to be **waiting for something** before initializing its SMD channels. In webOS, there may be:

1. A specific SMEM region that needs to be initialized first
2. An interrupt or signal that triggers Q6's SMD initialization
3. A handshake protocol we're not implementing

## Implementation Status

### Completed Work (Committed)

1. **Legacy AFE Protocol Support** (commit c6fe3531d54e)
   - Port ID conversion: 0x1000 → 6 (MI2S_RX), 0x1001 → 7 (MI2S_TX)
   - Mono/stereo conversion: mainline 0/1 → legacy 0/3
   - Legacy opcodes: AFE_PORT_AUDIO_IF_CONFIG (0x000100d3), etc.
   - Packet size: 36 bytes (matching webOS structure)
   - DT property: `qcom,legacy-afe-protocol`

2. **DPCM Routing Fix** (commit 2218aa834aee)
   - Backend DAI links use q6routing instead of q6asmdai

3. **WM8994 FLL Clock Configuration** (commit 2218aa834aee)
   - FLL configured from BCLK (1,536,000 Hz → 12.288 MHz sysclk)

4. **Q6 Firmware Loading** (commits 1061f39, 7113f8e)
   - `qcom,no-auto-boot` support in LPASS driver
   - Initramfs starts Q6 after firmware is mounted

### Verified Correct

- APR header fields: src_domain=5, dest_domain=4, src_svc=4, dest_svc=4
- APR domain in DT: `qcom,domain = <APR_DOMAIN_ADSP>` (4)
- SMD channel name: 'apr_audio_svc'
- Q6 remoteproc state: "running"
- Legacy AFE packet structure matches webOS

## File References

### Mainline Kernel
- `sound/soc/qcom/qdsp6/q6afe.c` - AFE driver with legacy support
- `sound/soc/qcom/apq8060.c` - Machine driver
- `drivers/remoteproc/qcom_q6v2_lpass.c` - LPASS remoteproc driver
- `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` - Device tree

### webOS Kernel (Reference)
- `arch/arm/mach-msm/qdsp6v2/q6afe.c` - Legacy AFE driver
- `arch/arm/mach-msm/qdsp6v2/apr.c` - APR implementation
- `arch/arm/mach-msm/qdsp6v2/apr_tal.c` - APR transport layer
- `arch/arm/mach-msm/smd.c` - SMD driver with platform device registration
- `arch/arm/mach-msm/peripheral-reset.c` - PIL Q6 boot sequence
- `arch/arm/mach-msm/include/mach/qdsp6v2/apr_audio.h` - Command definitions

## Next Steps

### Immediate Investigation

1. **Q6 Interrupt Handler Configuration**
   - Q6 populates SMEM but never sends interrupts back
   - GIC 121 (GIC_SPI 89) shows 0 interrupts from Q6
   - Need to understand how Q6's interrupt to APPS is configured
   - May need initialization of Q6's GIC or interrupt controller

2. **DAL Channel Investigation**
   - webOS opens DAL_AQ_AUD (cid=7) and DAL_AQ_VID (cid=6) before apr_audio_svc
   - These channels have flags=0x1 (not packet mode) and are skipped by mainline
   - May be required to "prime" Q6 before APR works
   - Need to investigate if opening these channels triggers Q6 initialization

3. **SMSM State Machine Handshake**
   - webOS smsm_irq_handler() responds to Q6 state changes
   - When Q6 sets SMSM_INIT, APPS responds with SMSM_INIT | SMSM_SMDINIT
   - This triggers do_smd_probe() to discover channels
   - Mainline may be missing this bidirectional handshake

4. **Q6 SMD Module Load Timing**
   - In webOS, SMD channels go 0→1→2 quickly after Q6 boots
   - Mainline channels stay at remote_state=0 indefinitely
   - Q6's SMD module may not be fully initialized when we try to open

### Alternative Approaches

1. **Port webOS SMD State Machine**
   - Implement SMSM state response logic from webOS smsm_irq_handler()
   - May require responding to Q6's SMSM state with APPS state update

2. **DAL Channel Support**
   - Add support for stream (non-packet) SMD channels
   - Open DAL_AQ_AUD before attempting APR communication

3. **Q6 Firmware Analysis**
   - Examine firmware for interrupt configuration
   - Check if firmware expects specific SMEM initialization

## Appendix A: Key Register Addresses

| Register | Address | Purpose |
|----------|---------|---------|
| LCC_Q6_FUNC | 0x28800000 + 0x1C | Q6 power/clock/reset control |
| QDSP6SS_RST_EVB | 0x28800000 | Boot address register |
| QDSP6SS_STRAP_TCM | 0x2880001C | TCM configuration |
| QDSP6SS_STRAP_AHB | 0x28800020 | AHB configuration |

## Appendix B: Legacy AFE Commands

| Command | Opcode | Description |
|---------|--------|-------------|
| AFE_PORT_AUDIO_IF_CONFIG | 0x000100d3 | Configure port parameters |
| AFE_PORT_CMD_START | 0x000100ca | Start port with gain/rate |
| AFE_PORT_CMD_STOP | 0x000100cb | Stop port |

## Appendix C: Port ID Mapping

| Mainline Port ID | Legacy Port ID | Name |
|------------------|----------------|------|
| 0x1000 | 6 | PRIMARY_MI2S_RX / MI2S_RX |
| 0x1001 | 7 | PRIMARY_MI2S_TX / MI2S_TX |
| 0x0000 | 0 | PRIMARY_I2S_RX |
| 0x0001 | 1 | PRIMARY_I2S_TX |

## Appendix D: Related webOS Processes

| Process | PID | Function |
|---------|-----|----------|
| audiod | 1911 | Main audio daemon |
| mediaserver | 1889 | Media playback service |

webOS uses `/dev/msm_pcm_out`, `/dev/msm_acdb` instead of standard ALSA.
