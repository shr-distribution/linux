# HP TouchPad Audio AFE Investigation Report

**Date:** 2026-01-22 (Updated)
**Status:** Audio data path working, debugging DSP timing issues
**Author:** Generated with assistance from Claude Code

## Executive Summary

**MAJOR PROGRESS:** Audio playback on the HP TouchPad with mainline Linux 6.18 is now partially working. Short audio files (176KB) play completely through the DSP pipeline. Longer files fail mid-playback due to a ~9 second delay in the write_done callback causing buffer underruns.

### Current Status
- ✅ Sound card registers (`HP-TouchPad`)
- ✅ AFE legacy protocol working
- ✅ ASM legacy protocol working
- ✅ ADM legacy protocol working
- ✅ LCC 540MHz frequency tables implemented
- ✅ Short audio files play completely (176KB notification.wav)
- ⚠️ Long files fail mid-playback (~524KB of 990KB plays)
- ⚠️ DSP session times out after playback (ADM COPP close fails)
- ❓ **No audible sound confirmed yet** - need user verification

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

## Update: Legacy ASM/ADM Protocol Implementation (2026-01-21) - MAJOR PROGRESS

### Summary

Implemented full legacy ASM (Audio Stream Manager) and ADM (Audio Device Manager) protocol support for MSM8660/APQ8060. **Audio data now flows successfully through the entire pipeline to the codec**, but no audible output due to FLL clock not locking.

### Legacy Protocol Implementation Status

| Component | Status | Notes |
|-----------|--------|-------|
| AFE (q6afe.c) | ✅ Working | Already implemented |
| ASM open_write | ✅ Working | Opcode 0x10BCA |
| ASM media format | ✅ Working | Opcode 0x10BDC, legacy struct |
| ASM RUN | ✅ Working | Opcode 0x10BD2, swapped timestamp fields |
| ASM write | ✅ Working | Opcode 0x10BD9, 32-bit phys addresses |
| ASM write_done | ✅ Working | Event 0x10BDF callback |
| ASM memory map | ⚠️ Bypassed | Not needed for legacy write |
| ADM COPP open | ✅ Working | Opcode 0x10304 |
| ADM COPP close | ✅ Working | Opcode 0x10305 |
| ADM matrix map | ✅ Working | Opcode 0x10301 |

### Audio Data Path - WORKING

Full 176KB notification.wav plays successfully:
```
$ ./tinyplay notification.wav -p 16384 -n 8
playing 'notification.wav': 2 ch, 44100 hz, 16-bit signed PCM
Played 176400 bytes. Remains 0 bytes.
```

dmesg shows complete command flow:
```
Legacy ASM open_write: session=1 stream_id=1 format=0x10be5
  hdr: src_port=0x101 dest_port=0x101 token=0x1 opcode=0x10bca
Legacy ADM COPP open: port=4096 path=1 topology=0x10312 rate=48000
Legacy PCM format: ch=2 bps=16 rate=44100
Legacy RUN: session=1 flags=0x0
Legacy write: phys=0x50b00000 len=65536 buf=0
Legacy write done: token=0x0
```

### Critical Bug Fixes Applied

1. **APR Header dest_port**: webOS uses `((session << 8) | 0x01)` for BOTH src_port AND dest_port (not just session for dest_port)

2. **POPP Topology**: Changed from 0x0 to 0x10be4 (DEFAULT_POPP_TOPOLOGY)

3. **Buffer Size**: Default tinyplay buffers (~8KB) cause timeout; use `-p 16384 -n 8` (128KB) for full playback

### Speaker Routing Discovery - CRITICAL

**The device tree audio-routing was WRONG.** Per webOS board-tenderloin.c:
> "Line outputs are not actually connected on the board."

TouchPad speakers are connected to:
- **SPKOUT pins** (SPKOUTLP, SPKOUTLN, SPKOUTRP, SPKOUTRN)
- Via external amplifier controlled by **WM8958 GPIO1**

NOT to LINEOUT pins as originally configured.

**Device Tree Fix:**
```dts
audio-routing =
    "Speaker", "SPKOUTLP",
    "Speaker", "SPKOUTLN",
    "Speaker", "SPKOUTRP",
    "Speaker", "SPKOUTRN",
    ...

/* Enable amplifier via GPIO1 */
wlf,gpio-cfg = <
    0x0041  /* GPIO1: output high to enable amp */
    ...
>;
```

### Remaining Issue: FLL Clock Not Locking

**Symptom:** No audible audio output despite successful data playback

**Root Cause:** WM8994 FLL (Frequency Locked Loop) times out:
```
wm8994-codec wm8994-codec: Timed out waiting for FLL lock
```

Without FLL lock, the codec DAC won't output audio.

**Attempted Solutions:**
1. Internal oscillator (`WM8994_FLL_SRC_INTERNAL`) - times out
2. BCLK source in prepare callback - causes hw_params failure (BCLK not available during setup)

**webOS Approach:**
- Initially clocks from MCLK1
- Switches to FLL with BCLK source once audio starts playing
- Uses `WM8994_FLL_SRC_BCLK` with `bclk_rate = rate * 32`

**Needed:** Find a way to either:
- Make internal oscillator FLL lock work
- Configure FLL from BCLK after AFE starts but before playback
- Provide external MCLK to the codec

---

## Update: Audio Playback Test Results (2026-01-21)

### Test Setup

Successfully deployed tinyalsa tools to the device:
- `/tmp/tinymixer` - Mixer control utility (cross-compiled for ARM)
- `/tmp/tinyplay` - Audio playback utility
- `/tmp/notification.wav` - Test audio file (16-bit stereo 44100Hz)

The WM8958 codec is detected with 1294 mixer controls accessible via tinymixer.

### Issue 1: WM8994 FLL Clock Lock Timeout

**Problem:** The WM8994 codec's FLL (Frequency Locked Loop) times out when trying to lock.

**Initial Attempt - BCLK Source:**
```c
ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1,
                          WM8994_FLL_SRC_BCLK,
                          bclk, rate * 256);
```

**Result:** FLL timeout because BCLK isn't available during `hw_params` - the AFE port hasn't started yet (that happens in `prepare`).

**Second Attempt - Internal Oscillator:**
```c
ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1,
                          WM8994_FLL_SRC_INTERNAL,
                          12000000, rate * 256);
```

**Result:** Still times out. The WM8994 driver waits for FLL lock completion via IRQ even with internal oscillator mode.

**Kernel log:**
```
wm8994-codec wm8994-codec: Timed out waiting for FLL lock
```

**Root Cause Analysis:**
- The codec driver's `fll_locked_irq` is true (IRQ was registered successfully)
- The driver waits 10ms for FLL lock completion interrupt
- Even with internal oscillator (WM8994_FLL_SRC_INTERNAL), the IRQ doesn't fire within timeout
- The FLL timeout is currently just a warning - playback continues to fail for other reasons

### Issue 2: Q6 ASM Memory Mapping Failure (Critical)

**Problem:** The Q6 ASM (Audio Stream Manager) returns error when trying to map audio buffers.

**Kernel log:**
```
qcom-q6asm aprsvc:service:4:7: DSP returned error[8]
q6asm-dai ...: Memory_map_regions failed
q6asm-dai ...: Audio Start: Buffer Allocation failed rc = -22
```

**Error Analysis:**
- Error 8 = `ADSP_EHANDLE` (Invalid handle)
- The DSP doesn't recognize the memory map session/handle
- This indicates the ASM session wasn't properly established

**Root Cause:**
The q6asm.c driver uses modern ASM command IDs:
- `ASM_STREAM_CMD_OPEN_WRITE_V3` = 0x00010DB3
- `ASM_CMD_SHARED_MEM_MAP_REGIONS` = 0x00010D92

The MSM8660/APQ8060 LPASS firmware likely uses different (legacy) command IDs, similar to how the AFE needed legacy protocol support.

**Comparison with AFE:**
| Component | Mainline Support | MSM8660 Support |
|-----------|------------------|-----------------|
| q6afe.c | ✅ Legacy protocol added | ✅ Working |
| q6asm.c | ❌ No legacy support | ❌ ADSP_EHANDLE |

### Issue 3: Mixer Routing Configuration

For reference, the mixer path configuration attempted:
```sh
/tmp/tinymixer set "PRI_MI2S_RX Audio Mixer MultiMedia1" 1  # Route PCM to MI2S
/tmp/tinymixer set "DAC1L Mixer AIF1.1 Switch" 1            # AIF1 to DAC1L
/tmp/tinymixer set "DAC1R Mixer AIF1.1 Switch" 1            # AIF1 to DAC1R
/tmp/tinymixer set "DAC1 Switch" 1 1                         # Enable DAC1
/tmp/tinymixer set "Left Output Mixer DAC Switch" 1          # DAC to output
/tmp/tinymixer set "Right Output Mixer DAC Switch" 1         # DAC to output
/tmp/tinymixer set "SPKL DAC1 Switch" 1                      # DAC to speaker L
/tmp/tinymixer set "SPKR DAC1 Switch" 1                      # DAC to speaker R
```

These controls were accepted but playback still failed due to ASM issues.

### Summary of Audio Playback Blockers

| Layer | Status | Issue |
|-------|--------|-------|
| Sound card registration | ✅ Working | - |
| Mixer controls | ✅ Working | 1294 controls accessible |
| PCM device | ✅ Working | pcmC0D0p available |
| AFE port config | ✅ Working | Legacy protocol implemented |
| ASM session | ❌ Failing | ADSP_EHANDLE (needs legacy support) |
| FLL clock | ⚠️ Warning | Times out but not blocking |

### Required Work: Legacy ASM Protocol Implementation

Analysis of webOS kernel (`arch/arm/mach-msm/qdsp6v2/q6asm.c`) reveals the legacy ASM command IDs and packet formats needed.

#### Command ID Comparison

| Function | Mainline (Modern) | webOS (Legacy) |
|----------|-------------------|----------------|
| Open Write | `ASM_STREAM_CMD_OPEN_WRITE_V3` (0x00010DB3) | `ASM_STREAM_CMD_OPEN_WRITE` (0x00010BCA) |
| Open Read | `ASM_STREAM_CMD_OPEN_READ_V3` (0x00010DB4) | `ASM_STREAM_CMD_OPEN_READ` (0x00010BCB) |
| Memory Map | `ASM_CMD_SHARED_MEM_MAP_REGIONS` (0x00010D92) | `ASM_SESSION_CMD_MEMORY_MAP_REGIONS` (0x00010C45) |
| Memory Unmap | `ASM_CMD_SHARED_MEM_UNMAP_REGIONS` (0x00010D94) | `ASM_SESSION_CMD_MEMORY_UNMAP_REGIONS` (0x00010C46) |
| Run | `ASM_SESSION_CMD_RUN_V2` (0x00010DAA) | `ASM_SESSION_CMD_RUN` (0x00010BD2) |
| Pause | `ASM_SESSION_CMD_PAUSE` (0x00010BD3) | `ASM_SESSION_CMD_PAUSE` (0x00010BD3) - **same!** |
| Data Write | `ASM_DATA_CMD_WRITE_V2` (0x00010DAB) | `ASM_DATA_CMD_WRITE` (0x00010BD9) |
| Data Read | `ASM_DATA_CMD_READ_V2` (0x00010DAC) | `ASM_DATA_CMD_READ` (0x00010BDA) |

#### Legacy Packet Structures (from webOS apr_audio.h)

**Open Write (Legacy):**
```c
struct asm_stream_cmd_open_write {
    struct apr_hdr hdr;
    u32            uMode;         /* STREAM_PRIORITY_* */
    u16            sink_endpoint; /* ASM_END_POINT_DEVICE_MATRIX */
    u16            stream_handle; /* 0 */
    u32            post_proc_top; /* DEFAULT_POPP_TOPOLOGY */
    u32            format;        /* LINEAR_PCM = 0x00010BE5 */
} __packed;
```

**Memory Map Regions (Legacy):**
```c
struct asm_stream_cmd_memory_map_regions {
    struct apr_hdr hdr;
    u16            mempool_id;    /* 0 = EBI */
    u16            nregions;      /* number of regions */
} __packed;

struct asm_memory_map_regions {
    u32            phys;          /* 32-bit physical address */
    u32            buf_size;
} __packed;
```

**Key Differences:**
1. Modern commands use 64-bit addresses (lsw/msw), legacy uses 32-bit
2. Modern memory map uses `mem_pool_id`, `property_flag`; legacy uses just `mempool_id`
3. Modern open_write has `bits_per_sample`, legacy has `uMode` for priority
4. Format IDs: Modern uses `ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2` (0x00010DA5), legacy uses `LINEAR_PCM` (0x00010BE5)

#### Implementation Plan

1. **Add to q6asm.c:**
   - Add `bool use_legacy_commands` field to `struct q6asm`
   - Add legacy command ID definitions
   - Add legacy packet structures
   - Create `q6asm_open_write_legacy()` function
   - Create `q6asm_memory_map_regions_legacy()` function
   - Modify `q6asm_open_write()` to call legacy variant when flag set
   - Modify `__q6asm_memory_map_regions()` to call legacy variant when flag set

2. **Add DT property:**
   - Add `qcom,legacy-asm-protocol` property check in probe (same pattern as q6afe.c)

3. **Update Device Tree:**
   - Add `qcom,legacy-asm-protocol;` to `aprsvc:service:4:7` node

---

## Next Steps

### Immediate Priority: Fix FLL Clock Issue

The audio data path is complete - all that remains is getting the codec to output audio.

1. **Investigate FLL Lock Failure**
   - Internal oscillator mode should work but IRQ never fires
   - May need to disable FLL lock wait or increase timeout
   - Check if WM8958 variant has different behavior

2. **Try MCLK Source**
   - webOS initially uses MCLK1 before switching to FLL
   - Need to determine if MCLK is available on APQ8060
   - May need to configure clock from MMCC or PMIC

3. **BCLK Timing**
   - webOS configures FLL from BCLK during playback
   - Need proper ASoC callback ordering to configure after AFE starts

### Completed Work

1. **Legacy ASM Protocol** - DONE
   - All commands working (open_write, media_format, run, write)
   - Memory map bypassed (legacy write uses physical addresses directly)
   - Write done event callback implemented

2. **Legacy ADM Protocol** - DONE
   - COPP open/close working
   - Matrix routing working

3. **Speaker Routing** - DONE
   - Fixed device tree to use SPKOUT instead of LINEOUT
   - Enabled GPIO1 for external amplifier

### Audio Test Script

Created `scripts/test-audio.sh` for automated testing:
- Transfers firmware and tools to device
- Configures all mixer controls for SPKOUT path
- Tests playback with notification.wav and phone.wav
- Uses large buffers to avoid ALSA timeout

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

---

## Update: LCC 540MHz Fix & Audio Playback Test (2026-01-22)

### LCC Clock Controller Fix

**Critical Discovery:** The mainline LCC (LPASS Clock Controller) driver only had frequency tables for PLL4 at 393MHz (default) and 492MHz (MSM8960). The HP TouchPad uses PLL4 at **540.672 MHz** (24.576 MHz × 22, L=0x16).

**Fix Applied to `drivers/clk/qcom/lcc-msm8960.c`:**

```c
/* MSM8660/APQ8060 uses PLL4 at 540.672 MHz (24.576 MHz * 22, L=0x16) */
static const struct freq_tbl clk_tbl_aif_osr_540[] = {
    {   768000, P_PLL4, 4, 1, 176 },
    {  1024000, P_PLL4, 4, 1, 132 },
    {  1536000, P_PLL4, 4, 1,  88 },
    {  2048000, P_PLL4, 4, 1,  66 },
    {  3072000, P_PLL4, 4, 1,  44 },
    {  4096000, P_PLL4, 4, 1,  33 },
    {  6144000, P_PLL4, 4, 1,  22 },
    {  8192000, P_PLL4, 2, 1,  33 },
    { 12288000, P_PLL4, 4, 1,  11 },
    { 24576000, P_PLL4, 2, 1,  11 },
    { 27000000, P_PXO,  1, 0,   0 },
    { }
};
```

**Important Note:** AIF_OSR clocks have 8-bit MN counters (max N=255), so 512kHz is not achievable at 540MHz (would need N=264). PCM clocks have 16-bit counters and include 512kHz.

The probe function detects PLL4 L value and selects appropriate frequency table:
```c
regmap_read(regmap, 0x4, &val);
if (val == 0x16) {
    /* L=22: PLL4 at 540.672 MHz (MSM8660/APQ8060) */
    dev_info(&pdev->dev, "PLL4 L=0x%x, using 540MHz frequency plan\n", val);
    mi2s_osr_src.freq_tbl = clk_tbl_aif_osr_540;
    pcm_src.freq_tbl = clk_tbl_pcm_540;
    /* ... apply to all audio clock sources */
}
```

### LCC Register Cross-Check

Verified ALL LCC registers match between webOS and mainline:

| Register | webOS | Mainline | Status |
|----------|-------|----------|--------|
| PLL4 MODE | 0x0000 | pll4.mode_reg | ✅ |
| PLL4 L | 0x0004 | pll4.l_reg | ✅ |
| PLL4 M | 0x0008 | pll4.m_reg | ✅ |
| PLL4 N | 0x000C | pll4.n_reg | ✅ |
| PLL4 CONFIG | 0x0014 | pll4.config_reg | ✅ |
| PLL4 STATUS | 0x0018 | pll4.status_reg | ✅ |
| LCC_Q6_FUNC | 0x001C | qcom_q6v2_lpass.c | ✅ (remoteproc) |
| MI2S_NS/MD/STATUS | 0x48/4C/50 | mi2s_osr_src | ✅ |
| PCM_NS/MD/STATUS | 0x54/58/5C | pcm_src | ✅ |
| CODEC_I2S_* | 0x60-0x74 | codec_i2s_* | ✅ |
| SPARE_I2S_* | 0x78-0x8C | spare_i2s_* | ✅ |
| PRI_PLL_CLK_CTL | 0x00C4 | probe: write 0x1 | ✅ |

### Audio Playback Test Results

**Test 1: Short file (notification.wav, 177KB)**
```
$ ./tinyplay notification.wav -p 16384 -n 8
playing 'notification.wav': 2 ch, 44100 hz, 16-bit signed PCM
Played 176400 bytes. Remains 0 bytes.
```
**Result: SUCCESS** - Full file played through DSP pipeline.

**Test 2: Long file (phone.wav, 990KB)**
```
$ ./tinyplay phone.wav -p 16384 -n 8
playing 'phone.wav': 2 ch, 44100 hz, 16-bit signed PCM
error playing sample. cannot read/write stream data: Input/output error
Played 524288 bytes. Remains 466464 bytes.
```
**Result: PARTIAL** - About 53% played before I/O error.

### Write_Done Callback Timing Issue

The kernel log reveals a critical timing problem:
```
[ 2957.112610] q6asm-dai: Legacy write: phys=0x50b00000 len=65536 buf=0
[ 2966.020374] q6asm-dai: Legacy write done: token=0x0
```

**~9 second delay** between write and write_done callback! This should be milliseconds.

Possible causes:
1. DSP processing stalls or runs at wrong clock rate
2. IRQ delivery from DSP to APPS delayed
3. SMD channel state machine issue
4. Missing interrupt acknowledgment

### DSP Session Timeout After Playback

After first playback, subsequent attempts fail:
```
[ 3392.225251] qcom-q6adm: ADM copp cmd timedout
[ 3392.225305] qcom-q6adm: Failed to close copp -110
[ 3432.097321] qcom-q6afe: AFE legacy config for port 6 failed -110
```

The ADM COPP close command times out, leaving DSP in bad state. LPASS restart required but causes device crash.

### WM8994 FLL Lock Fix

**Problem:** FLL lock IRQ never fires, causing 10ms timeout on every playback.

**Root Cause:** TouchPad doesn't have codec IRQ line connected.

**Fix Applied to `sound/soc/codecs/wm8994.c`:**
```c
/*
 * Disable FLL lock IRQ wait - the webOS WM8994 driver from 2012
 * doesn't wait for FLL lock at all. On HP TouchPad, the FLL lock
 * IRQ is not delivered (no main codec IRQ line connected).
 */
wm8994->fll_locked_irq = false;
```

Now using internal oscillator at 12MHz → FLL output 12.288MHz (256 × 48kHz).

### MI2S Pin Configuration

**Discovery:** TouchPad uses **SD3 (GPIO 107)** for codec data, not SD0/SD1.

Added to device tree:
```dts
pri_mi2s_pins: pri-mi2s-state {
    clk-pins {
        pins = "gpio101", "gpio102", "gpio103";  /* WS, SCLK, MCLK */
        function = "mi2s";
    };
    data-pins {
        pins = "gpio107";  /* SD3 for codec data */
        function = "mi2s";
    };
};

&q6afedai {
    dai@16 {
        reg = <PRIMARY_MI2S_RX>;
        qcom,sd-lines = <3>;  /* Use SD3, not SD0 */
    };
};
```

### Remaining Issues

1. **No audible sound confirmed** - Need user to verify if speakers produce sound
2. **~9s write_done delay** - Causes buffer underrun on long files
3. **DSP session stuck after playback** - ADM COPP close times out
4. **LPASS restart crashes device** - Cannot recover from stuck state

### Files Modified (WIP)

| File | Changes |
|------|---------|
| `drivers/clk/qcom/lcc-msm8960.c` | 540MHz frequency tables (committed) |
| `sound/soc/qcom/apq8060.c` | Internal oscillator FLL, simplified hw_params |
| `sound/soc/codecs/wm8994.c` | Disable FLL lock IRQ wait |
| `sound/soc/qcom/qdsp6/q6afe.c` | Change debug to info for visibility |
| `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` | MI2S pins, SD3 line |
| `scripts/test-audio.sh` | Test script improvements |
