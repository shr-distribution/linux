# A22X VGT DMA Workaround Analysis

## Date: February 23, 2026

## Executive Summary

Investigation into whether the A20x VGT DMA workaround applies to A22X (Leia) reveals that **Leia explicitly skips this workaround** in the legacy KGSL driver. The dummy draw with `CP_DRAW_INDX_BIN` causes GPU hangs on A22X.

---

## Key Findings

### 1. Legacy KGSL Skips VGT DMA Workaround for Leia

**kgsl_yamato.c:373-410:**
```c
if (flags & KGSL_MMUFLAGS_PTUPDATE &&
    device->chip_id != KGSL_CHIPID_LEIA_REV470) {  // <-- SKIPS Leia!

    // VGT DMA workaround with dummy DRAW_INDX_BIN packets
    *cmds++ = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
    *cmds++ = 0;           // viz query info
    *cmds++ = 0x0003C004;  // draw indicator
    *cmds++ = 0;           // bin base
    *cmds++ = 3;           // bin size
    *cmds++ = device->mmu.dummyspace.gpuaddr;  // dma base
    *cmds++ = 6;           // dma size
    // ... second dummy draw ...
}
```

**This workaround is ONLY for Yamato (A200), NOT Leia (A220).**

### 2. Leia Uses Different Draw Packet Format

When Leia needs to issue draws during GMEM operations, it uses **PM4_DRAW_INDX_2** (0x36), NOT PM4_DRAW_INDX_BIN (0x34):

**kgsl_drawctxt.c:835-841:**
```c
if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
    *cmds++ = 0xc0043600;  // PM4_DRAW_INDX_2 packet (opcode 0x36)
    *cmds++ = 0x0;
    *cmds++ = 0x00004046;  // tristrip
    *cmds++ = 0x00000004;  // NUM_INDICES = 4
    *cmds++ = 0x00010000;  // index: 0x00, 0x01
    *cmds++ = 0x00030002;  // index: 0x02, 0x03
} else {
    *cmds++ = pm4_type3_packet(PM4_DRAW_INDX, 2);
    *cmds++ = 0;
    *cmds++ = 0x00030088;  // RectList, 3 indices
}
```

### 3. CP_DRAW_INDX_BIN Causes GPU Hang on A22X

**Test Result:**
When we added the A20x-style workaround with `CP_DRAW_INDX_BIN` and `0x0003c004` to A22X, the GPU immediately hung:
```
[drm:a2xx_idle] *ERROR* 2.2.0.0: timeout waiting for GPU to idle!
[drm:hangcheck_handler] *ERROR* 2.2.0.0: hangcheck detected gpu lockup rb 0!
```

This confirms the legacy driver's approach was correct - Leia cannot use this workaround.

---

## What We've Tried and Results

| Patch | Description | Result |
|-------|-------------|--------|
| 0014 | WFI after constant emission | No improvement (~10% smooth) |
| 0015 | WFI after vertex buffer setup | No improvement (~10% smooth) |
| 0016 (v1) | CP_WAIT_REG_EQ for VGT_BUSY_NO_DMA only | **Improved to 40% smooth** |
| 0016 (v2) | + A20x-style dummy draw | **GPU HANG** |
| 0016 (v3) | Back to just CP_WAIT_REG_EQ | 40% smooth |

---

## Remaining Investigation Items

Based on the kgsl-vs-freedreno analysis, these items haven't been tried:

### Priority: HIGH

1. **VSC Pipe Registers (0x0C01-0x0C1D)**
   - KGSL saves/restores 8 VSC registers during context switch
   - freedreno does NOT handle these for A22x
   - May cause visibility state corruption between draws

2. **SQ_PROGRAM_CNTL bit 17**
   - KGSL sets to 0x10018001 for Leia (bit 17 set)
   - vs 0x10010001 for Yamato
   - May affect shader execution mode

### Priority: MEDIUM

3. **RB_DEPTHCONTROL in GMEM (0x08 vs 0x00)**
   - KGSL uses 0x08 for Leia during GMEM operations
   - May affect depth buffer handling

4. **Leia-specific Draw Format (PM4_DRAW_INDX_2)**
   - If we need a dummy draw, must use 0xc0043600 format
   - NOT CP_DRAW_INDX_BIN

### Priority: LOW

5. **PC-based vs VGT-based registers**
   - Leia uses REG_LEIA_PC_* instead of REG_*_VGT_*
   - May need to use correct register set

---

## Hypothesis: Root Cause

Given that:
1. The faceted rendering is **intermittent** (40% smooth with DMA wait)
2. Register dumps show **identical state** between smooth and faceted runs
3. The issue affects **uniform-based shaders** (inputs_count=0)
4. VGT DMA wait helps but doesn't fix

The issue is likely related to:
- **VSC (Visibility Stream Cache) state corruption** between draws
- **Shader execution timing** (SQ_PROGRAM_CNTL bit 17)
- Some **A22x-specific initialization** we're missing

---

## Next Steps

1. Add VSC pipe register initialization/save/restore for A22x
2. Investigate SQ_PROGRAM_CNTL bit 17 setting for A22x
3. Check if RB_DEPTHCONTROL needs to be 0x08 during GMEM ops for Leia

---

## Files Referenced

### Legacy KGSL:
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/kgsl_yamato.c`
- `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/kgsl_drawctxt.c`

### Mesa freedreno:
- `src/gallium/drivers/freedreno/a2xx/fd2_draw.c`
- `src/gallium/drivers/freedreno/a2xx/fd2_emit.c`
- `src/gallium/drivers/freedreno/a2xx/fd2_gmem.c`

### Reports:
- `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/kgsl-vs-freedreno-a220-analysis.md`
- `/home/herrie/webos/touchpad-kernel/linux-6.18-tenderloin/reports/leia_firmware_report.md`
