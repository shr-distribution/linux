# Proprietary libGLESv2.so Register Map (Ghidra Decompilation)

## leia_init_hw() - Hardware Initialization Sequence

Decoded from PM4 packets in the decompiled code:

| Register | Address | Value | Description |
|----------|---------|-------|-------------|
| PA_SC_WINDOW_OFFSET | 0x2316 | 0x3b | Window offset |
| SQ_CONTEXT_MISC | 0x2181 | 4 | Context misc flags |
| **SQ_INTERPOLATOR_CNTL** | 0x2182 | **0xffffffff** | All smooth interp |
| PA_CL_VTE_CNTL | 0x2204 | 0 | VTE control |
| PA_CL_CLIP_CNTL | 0x2206 | 0x43f | Clip control |
| PA_SC_WINDOW_SCISSOR_TL | 0x2302 | 1 | Scissor top-left |
| RB_COLORCONTROL | 0x2208 | 4 | Color control |
| RB_BLENDCONTROL | 0x2201 | 0x10001 | Blend control |
| RB_BC_CONTROL | 0x2202 | 0x20 | BC control |
| PA_SC_LINE_STIPPLE | 0x2301 | 0 | Line stipple |
| PA_SC_AA_MASK | 0x2312 | 0xffff | AA mask |
| RB_MODECONTROL | 0x220a | 0x88888888 | Mode control |
| (unknown) | 0x2326 | 0xffffffff | |
| PA_SC_WINDOW_SCISSOR_BR | 0x2303 | (calculated) | Scissor bottom-right |
| SQ_WRAPPING_0 | 0x2183 | 0 | Texture wrap 0 |
| SQ_WRAPPING_1 | 0x2184 | 0 | Texture wrap 1 |

### Low Registers (0x0XXX range)

| Register | Address | Value | Where Written |
|----------|---------|-------|---------------|
| (unknown) | 0x0e1e | 0 or 2 | leia_init_hw |
| (unknown) | 0x0f02 | (calculated) | leia_init_hw |
| SQ_INST_STORE_MANAGMENT | 0x0d02 | 0x180 or 0x300 | leia_repartition_instruction_store |
| SQ_GPR_MANAGEMENT | 0x0d00 | (dynamic) | leia_perform_resolve ONLY |

### CONFIRMED NOT WRITTEN

| Register | Address | Notes |
|----------|---------|-------|
| **SQ_RESOURCE_MANAGMENT** | 0x0d03 | NOT in driver - uses HW default |
| **SQ_PIX_IN_CNTL** | 0x0d0c | NOT in driver - uses HW default |

## SQ Register Comparison

| Register | Proprietary | freedreno | Match? |
|----------|-------------|-----------|--------|
| SQ_INTERPOLATOR_CNTL (0x2182) | 0xffffffff | 0xffffffff | ✅ YES |
| SQ_CONTEXT_MISC (0x2181) | 4 | Per-draw | ⚠️ Different |
| SQ_GPR_MANAGEMENT (0x0d00) | Dynamic in resolve | 0x00040401 | ⚠️ Different |
| SQ_INST_STORE_MANAGMENT (0x0d02) | 0x180/0x300 | 0x00000180 | ✅ YES |
| SQ_RESOURCE_MANAGMENT (0x0d03) | NOT WRITTEN | Patch 0017 writes 0 | ❌ Caused hang |
| SQ_PIX_IN_CNTL (0x0d0c) | NOT WRITTEN | Patch 0017 writes 0xffffffff | ❌ Caused hang |

## Key Findings

1. **Patch 0017 caused GPU hang** because it wrote to registers (0x0d03, 0x0d0c) that:
   - The proprietary driver NEVER writes
   - May not exist on Leia hardware
   - Have correct hardware defaults

2. **SQ_INTERPOLATOR_CNTL** is correctly set to 0xffffffff in both drivers

3. **SQ_CONTEXT_MISC** is set to 4 during init, but freedreno sets it per-draw

4. **Dynamic GPR allocation**: Proprietary driver calculates GPR split dynamically based on shader needs. freedreno uses fixed allocation.

## Registers 0x0e1e and 0x0f02

These appear to be:
- 0x0e1e: Possibly LRZ (Low Resolution Z) control
- 0x0f02: Possibly binning/visibility control

Both are written during initialization with calculated values based on surface size.

## Recommendations

1. **Remove patch 0017** - Writing to undefined registers causes hangs
2. **Keep patch 0016** - Atomic SQ writes are correct
3. **Investigate SQ_CONTEXT_MISC** - Value 4 during init might be important
4. **Consider dynamic GPR allocation** - May improve complex shader handling

---
*Generated: Feb 19, 2026*
*Source: Ghidra 12.0.1 decompilation of libGLESv2.so*
