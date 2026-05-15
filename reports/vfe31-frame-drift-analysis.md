# VFE31 Frame Drift / Buffer Swap Analysis

## Problem
When capturing multiple frames via v4l2 streaming (ping-pong double buffering), frames after the first show progressive vertical drift (~30 lines per frame at 640x480). At 1280x1024, the issue manifests differently.

## Hardware
- APQ8060 (MSM8660 family), VFE HW version 0x00030217
- MT9M113 sensor: Context A = 640x480 binned (~15fps), Context B = 1280x1024 full (~7fps)
- VFE31 uses Unified Buffer (UB) SRAM with 64-byte burst writes to external memory
- Ping-pong double buffering: PING and PONG address registers per Write Master (WM)
- COMPOSITE_DONE IRQ fires when all WMs in a group complete a frame

## Buffer Swap Mechanism
1. COMPOSITE_DONE fires → ISR runs
2. ISR reads PING_PONG_STATUS to determine which buffer just completed
3. ISR gets next buffer from v4l2 pending queue
4. ISR writes new address to the INACTIVE buffer's register (PING or PONG)
5. ISR returns completed buffer to userspace
6. At next frame boundary, DMA automatically toggles PP bit and should use the other buffer

## Key Register: BUS_CMD (0x038)
- `BIT(0)-BIT(6)`: Reload WM0-WM6 (forces DMA to re-read address registers)
- `BIT(7)-BIT(13)`: Unknown function (NOT ping/pong split as initially hypothesized)
- `BIT(14)`: Unknown (possibly global reload)
- Full reload value: `0x7FFF` (used during VFE reset/init when no DMA is active)

## What We Tried

### 1. No bus_reload in wm_done (current state)
- **640x480**: Progressive drift ~30 lines/frame (DMA doesn't pick up new addresses fast enough)
- **1280x1024**: PERFECT - all frames aligned (slower frame rate gives DMA time to auto-read)

### 2. bus_reload BIT(wm) after address update
- **640x480**: Frames 1-2 aligned, frame 3 has ~30 line shift (one-time glitch)
- **1280x1024**: Frame 3 has 64-byte vertical stripe pattern (UB SRAM corruption)

### 3. bus_reload BIT(wm)|BIT(wm+7) after address update
- **640x480**: Same as #2
- **1280x1024**: Blank frames, erratic alignment (worse than #2)

## Root Cause Analysis

### The UB SRAM Corruption (1280x1024 stripes)
Frame 3 of video1280-nv16 shows vertical stripes with discontinuities at exact 64-byte column intervals:
```
col 16:  255->73  (diff=182)
col 64:  53->255  (diff=202)
col 80:  255->45  (diff=210)
col 128: 31->255  (diff=224)
col 144: 255->32  (diff=223)
Intervals: 48, 16, 48, 16 (repeating = 64 byte UB burst)
```
This proves bus_reload during active DMA causes the UB to mix data from old and new buffer addresses at burst boundaries. The UB has partially-written bursts from the old address when reload forces it to switch to the new address.

### The Progressive Drift (640x480 without bus_reload)
Without bus_reload, the DMA doesn't immediately pick up new PING/PONG addresses written by the ISR. At 640x480 (~15fps, ~67ms per frame), the address register update happens in the COMPOSITE_DONE ISR but the DMA continues using the old address for the next frame. Each frame accumulates ~30 lines of offset because the DMA write pointer doesn't reset to the start of the new buffer.

At 1280x1024 (~7fps, ~143ms per frame), the DMA has enough inter-frame time to naturally latch the new address, so no drift occurs.

### Why Samsung Doesn't Have This Issue
Samsung's VFE31 HAL runs in userspace. The buffer management (address updates + bus_reload) happens in a daemon process, not in a kernel ISR. The timing is fundamentally different:
- Samsung: COMPOSITE_DONE → kernel IRQ → signal to userspace → userspace updates addresses → bus_reload
- Our driver: COMPOSITE_DONE → kernel ISR → immediate address update + bus_reload

Samsung's approach has more latency between COMPOSITE_DONE and bus_reload, which may avoid the UB SRAM conflict. By the time userspace runs, the DMA has fully completed the buffer switch and the UB has been flushed.

Additionally, Samsung's kernel (msm_vfe31.c) uses `vfe31_update_ping_pong_addr()` which writes the address and calls bus_reload. But this runs in a tasklet (bottom half), not hard IRQ context, adding a small delay.

## Possible Solutions

### A. Use REG_UPDATE instead of bus_reload
REG_UPDATE (VFE_REG_UPDATE_CMD = 0x260) latches shadow registers at the next VSYNC/frame boundary. If PING/PONG addresses are shadow registers, REG_UPDATE would cause them to take effect exactly at the frame boundary - no UB corruption, no timing issues.

### B. Defer bus_reload to REG_UPDATE IRQ
Instead of bus_reload in COMPOSITE_DONE ISR, set a flag and issue bus_reload in the next REG_UPDATE IRQ handler (which fires at frame boundary). This ensures bus_reload happens when the DMA is between frames, not during active writing.

### C. Use framedrop to skip frame 3
Samsung uses FRAMEDROP registers (0x504-0x520) for frame rate control. We could skip the first frame after a buffer swap to avoid the corrupted frame. This is a workaround, not a fix.

### D. Triple buffering
Instead of ping-pong (2 buffers), use 3 buffers: one being written, one being processed, one ready. This gives a full frame's worth of time for the address update to take effect. Would require changes to the v4l2 buffer management.

## Current Register Configuration (Working)

### PIX/VIDEO Mode (AXI=0x01)
- MODULE_CFG (0x010) = 0x01C00C0C (DEMUX + ISP modules enabled)
- CORE_CFG (0x014) = 0x46 (UYVY pattern + input mux enable bit 6)
- BUS_CFG (0x03C) = 0x02AAA771
- AXI_OUT_MODE (0x040) = 0x01
- XBAR_CFG1 (0x044) = 0x1A1B (Y→WM0, CbCr→WM4)
- WMs enabled BEFORE CAMIF start (Samsung approach)

### RDI Raw Bypass Mode (AXI=0x60) - NOT WORKING
- MODULE_CFG = 0x00 (all ISP disabled)
- CORE_CFG = pixel_pattern only (NO input mux enable bit 6)
- BUS_CFG = 0x02AAA771 (8-bit) / 0x02AAA775 (10-bit)
- AXI_OUT_MODE = 0x60
- CAMIF receives zero data - bypass path may not work on APQ8060

## File Locations
- VFE31 driver: `drivers/media/platform/qcom/camss/camss-vfe-3-1.c`
- Gen1 framework: `drivers/media/platform/qcom/camss/camss-vfe-gen1.c`
- Video device: `drivers/media/platform/qcom/camss/camss-video.c`
- Test script: `scripts/test-camera.sh`
- Samsung reference: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c`
