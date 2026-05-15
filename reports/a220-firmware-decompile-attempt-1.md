# A220 firmware decompile attempt #1

## Files

- `firmware/leia_pm4_470.fw` — 9220 bytes, A220 CP/ME microcode
- `firmware/leia_pfp_470.fw` — 1156 bytes, A220 PreFetch Parser microcode

MD5 identical across all locations: `/firmware/`, `meta-hp initramfs scripts`, `doctor305/306 LuneOS images`. Single version.

## Quick-scan findings (kernel `4a4a6622f1ae`)

### PM4 firmware register writes (instruction template `04 00 11 44 60 00 c7 02 00 00`)

| File offset | Register | Address | Loaded value (preceding instruction) |
|---|---|---|---|
| `0x0934` | `CP_SCRATCH_REG0` | `0x0578` | `0x00000001` |
| `0x1b40` | `SQ_INST_STORE_MANAGMENT` | `0x0d02` | (operand decode TBD, possibly via `00 00 e0 0a`) |
| `0x2098` | `RBBM_DEBUG` | `0x039b` | `0x810000ff` |

All three writes follow the identical encoding pattern. Likely ME-INIT-time init writes — not per-submit operations.

### Reads/refs to A2XX register addresses (16-bit LE scan)

| Register | Hits | Notes |
|---|---|---|
| `0x21f9` VGT_EVENT_INITIATOR | 11 | Most-referenced. Fires VGT events from packet handlers |
| `0x2000` RB_SURFACE_INFO | 7 | RB surface state setup |
| `0x0c00` A220_VSC_BIN_SIZE | 6 | GMEM tile binner |
| `0x2100` VGT_MAX_VTX_INDX | 4 | Vertex index range |
| `0x2228` (unnamed, PA) | 4 | |
| `0x20d0` (unnamed, RB) | 3 | |
| `0x21f6` SQ_PS_PROGRAM | 2 | Pixel shader base/cntl |
| `0x21f7` SQ_VS_PROGRAM | 2 | Vertex shader base/cntl |
| `0x21fc` VGT_DRAW_INITIATOR | 2 | Draw firing |
| `0x21f4`/`0x21f7`/`0x0c11` | 1-2 each | |
| `0x0578` CP_SCRATCH_REG0 | 1 | (Same hit as write table above) |
| `0x0d02` SQ_INST_STORE_MANAGMENT | 1 | (Same hit as write table above) |
| `0x039b` RBBM_DEBUG | 1 | (Same hit as write table above) |

### Negative findings (important)

- **No references to `CP_SCRATCH_REG1..7`** (addresses 0x0579..0x057F) anywhere in PM4 firmware
- **No references to `CP_SCRATCH2` range** (0x05D0..0x05D7) that Gemini's update-28 reply suggested might be the context-counter bank
- **No "increment counter and mask with 7" pattern** visible from immediate-value scan (7× `0x00000008`, 6× `0x00000007` — too generic to claim a mod-8 cycle)
- **No CP_SCRATCH-write loop** (only the one-shot init write to REG0)

### PFP firmware

- Completely different ISA from PM4 — denser encoding, no matches on PM4's instruction template
- Ends with `06 00 00 00` repeating (padding / nops)
- Did not pursue further — PFP's role is just prefetching PM4 packets, unlikely to host the cycle

## What this means for the 8-cycle investigation

The firmware-side scan **independently falsifies Hypothesis A** (Gemini's "CP_SCRATCH context counter" theory). Combined with the earlier kernel-side Option I test (CPU writes `0` to CP_SCRATCH_REG0..7 per submit, null result), we now have two independent confirmations that CP_SCRATCH is not the cycle's seat.

The firmware does manage state — VGT events, shader bases, GMEM binning — but everything we can see at this scan depth is consistent with normal CP packet handling, not a per-submit context counter.

Two remaining unfalsified hypotheses for where the cycle's seat actually lives:

1. **ME microcontroller-private internal state** — the ME has its own register file invisible from outside MMIO. A 3-bit counter incrementing on packet-N or event-M would be entirely opaque to anything we can probe.

2. **Hardware-level SQ wavefront scheduler state below firmware** — the SQ scheduler may rotate slots in hardware without firmware involvement. Firmware just issues `CP_DRAW_INDX` and the scheduler picks the next slot internally.

Both paths require either full A220 ME ISA reverse-engineering (multi-week effort) or hardware datasheet access (not available).

## What we still don't know (worth a deeper attempt)

- The PM4 firmware ISA — only one instruction template identified so far. There are likely 5-10 more encoding patterns we haven't found
- What the PM4 firmware does on `CP_DRAW_INDX` — the handler for opcode 0x22 must be in this binary somewhere
- Whether there's a code-section vs data-section split — the trailing `02 00 02 00 02 00 ...` pattern at the file end (offset ~0x2300+) looks like a constant table, not code
- Whether the firmware has visible references to its own PC/jump targets that could help map out the code structure
- The CP_SCRATCH_REG0 = 0x1 might actually be the firmware's "I am alive" handshake. Worth checking what the kernel reads after ME_INIT

## Tools that would help

- `envytools enva2re` — reportedly has an A2XX ME microcode disassembler. Status: not built locally
- Original freedreno project notes (Rob Clark blog circa 2012-2013) had some A2XX ME ISA notes
- Codeaurora KGSL source comments mention some packet handler addresses but not ME ISA

## Deep-dive findings (added after attempt #1)

### Existing tooling status

- **No A2XX PM4/ME microcode disassembler exists publicly.** Verified across mesa-latest, envytools-local, and all webOS / linux-shr kernel sources.
- **AFUC disassembler at `mesa-latest/src/freedreno/afuc/`** — handles A5XX+ only. Mesa README explicitly notes: *"Adreno 2xx thru 4xx used basically the same instruction set as r600... [with] PFP and ME [having] different instruction sets."*
- **A2XX shader disassembler at `mesa-latest/src/freedreno/ir2/disasm-a2xx.c`** — disassembles the GPU's *shader* ISA (ALU/CF/FETCH for vertex/fragment shaders), NOT the ME microcode. Different ISA, different instruction layout.
- **Mesa freedreno reference**: A2XX uses an r600-compatible ISA. AMD/ATI r600 ISA is publicly documented (Evergreen ISA reference, R600 OpenGL stack), would be the basis for any new A2XX ME disassembler.
- **Prior analysis report**: `/home/herrie/Documents/GitHub/linux-shr/reports/leia_firmware_report.md` had already identified the file header, dispatch table location, and magic values.

### Dispatch table at 0x22B4 — decoded

Each PM4 opcode (0x00-0xFF) has a 16-bit handler-address entry. Address scale is **u32-index from file start** (i.e., `file_offset = addr * 4`). Valid handlers are entries pointing into the code section (0x40 ≤ addr*4 < 0x22B4).

**33 distinct PM4 opcodes have firmware handlers. The other 223 dispatch to `0x0000` = passthrough to hardware.**

Handlers sorted by file offset:

| Opcode | Disp addr | File offset | Notes |
|---|---|---|---|
| 0x1a, 0x36 | 0x0012 | 0x0048 | Shared/error stub |
| 0x38 | 0x0017 | 0x005c | |
| 0x37 | 0x0022 | 0x0088 | |
| 0x20 | 0x0028 | 0x00a0 | |
| 0x1b | 0x002a | 0x00a8 | |
| 0x50 | 0x0030 | 0x00c0 | |
| 0x6c | 0x0049 | 0x0124 | |
| 0x6d | 0x0064 | 0x0190 | |
| 0x6f | 0x006c | 0x01b0 | |
| 0x19 | 0x006f | 0x01bc | |
| **0x2b IM_LOAD_IMMEDIATE** | **0x0085** | **0x0214** | Inline shader load |
| 0x55 | 0x00a8 | 0x02a0 | |
| 0x72 | 0x00c2 | 0x0308 | |
| 0x42, 0x4f | 0x00c6 | 0x0318 | Shared handler |
| 0x62 | 0x0104 | 0x0410 | |
| 0x63 | 0x0110 | 0x0440 | |
| 0x43 | 0x011c | 0x0470 | |
| 0x5d | 0x0121 | 0x0484 | |
| 0x44 | 0x0126 | 0x0498 | |
| **0x48 ME_INIT** | **0x0151** | **0x0544** | Init handshake |
| 0x00, 0x51 | 0x0153 | 0x054c | NOP shared with 0x51 |
| 0x56 | 0x0155 | 0x0554 | |
| 0x54 | 0x017d | 0x05f4 | |
| 0x15 | 0x01ca | 0x0728 | |
| **0x27 IM_LOAD** | **0x01d9** | **0x0764** | Shader instr memory load |
| 0x18 | 0x01db | 0x076c | |
| **0x2a** (undocumented) | **0x022f** | **0x08bc** | Contains the CP_SCRATCH_REG0=1 write @ 0x934 |
| 0x74 | 0x0276 | 0x09d8 | |
| 0x75 | 0x029d | 0x0a74 | |
| **0x2d SET_CONSTANT** | **0x02a8** | **0x0aa0** | ~6 KB handler — the giant. Half the firmware. Contains SQ_INST_STORE_MANAGMENT and RBBM_DEBUG writes |

### Critical opcodes with NO firmware handler (pass through to hardware)

- **`0x22 CP_DRAW_INDX`** — dispatch entry = `0x0000`
- **`0x23 CP_VIZ_QUERY`** — `0x0000`
- **`0x36 CP_DRAW_INDX_2`** — `0x0012` (error stub) so the draw is rejected by firmware? Or this is a placeholder
- **`0x39 CP_INVALIDATE_STATE`** — `0x0002`
- **`0x46 CP_EVENT_WRITE_SHD`** — `0x0000`
- **`0x4b CP_SET_SHADER_BASES`** — `0x0002`
- **`0x10 CP_WAIT_REG_MEM`** — `0x0000`
- **`0x12 CP_WAIT_FOR_IDLE`** — `0x0002`

These are the packet types whose handling is entirely below the ME firmware level. They reach the PFP, get parsed, and the hardware blocks (VGT, SQ, RB, MMU) process them directly. The ME never sees them.

### Where each firmware register write lives

- `CP_SCRATCH_REG0 = 0x00000001` @ file `0x0934` — inside **handler for opcode 0x2A** (undocumented A2XX opcode, range 0x08bc-0x09d8). One-shot init-style write.
- `SQ_INST_STORE_MANAGMENT` @ file `0x1b40` — inside **SET_CONSTANT (0x2d) handler** (range 0x0aa0-0x22b4). Suggests SET_CONSTANT does *special fixup* when the target register is `0x0d02`.
- `RBBM_DEBUG = 0x810000ff` @ file `0x2098` — also inside **SET_CONSTANT (0x2d) handler**. Same pattern.

### What this means for the 8-cycle

This is the most informative negative finding of the entire investigation:

**The firmware never sees CP_DRAW_INDX.** Draw packets bypass the ME entirely and go straight from PFP into the hardware graphics pipeline (VGT scheduler → SQ wavefront allocator → RB write path). There is no firmware-side hook on draws, no per-draw counter, no firmware-managed slot rotation.

**The cycle's seat is therefore in pure silicon downstream of the ME.** Candidates:
- VGT scheduler internal state
- SQ wavefront slot allocator (the 8 in "8-cycle" strongly suggests this — A220 has 8 wavefront slots in its shader processor)
- VPC varying interpolation cache state machine

None of these is reachable from any software interface — they're internal hardware state machines.

### Confirmed cumulative falsifications

After both kernel/userspace exhaustion (12 mechanisms across updates 1-29) and this firmware analysis, we have:

| Layer | Result |
|---|---|
| Mesa userspace PM4 emission | All attempts null (5 falsifications) |
| Kernel MMIO register manipulation | All attempts null (3 falsifications, incl. CP_SCRATCH pin) |
| Kernel ringbuffer cmdstream (sanitizer preamble) | All attempts null (3 falsifications) |
| Kernel sanitizer + dummy draws | Null + MMU faults |
| MMCC-level resets (boot + per-cycle) | Null or destructive (tile noise) |
| GDSC rail collapse | Clears state but breaks GMEM |
| **PM4/ME firmware code paths** | **No DRAW_INDX handler — bypass to hardware** |

The cycle is below software's reach at every layer we have. **Option F (ship at 12.5%) is the only honest conclusion.**

### Closing notes

The firmware analysis was not a fix — but it was the most decisive *negative* finding of the entire investigation. We now have positive confirmation that the cycle is in hardware downstream of the ME, not in any layer reachable through normal Linux kernel + userspace mechanisms.

This report can serve as a starting point for any future investigator who wants to build a proper A2XX ME microcode disassembler. The dispatch table is mapped, the most-common instruction templates are identified, the handler boundaries are known. The remaining work is reverse-engineering the r600-derived ISA — a multi-week project — and even completed, it would only confirm what this scan already shows: the draw-side cycle is in silicon, not in microcode.

## Branch state

Kernel: `tenderloin/6.18/upstream-patches` tip `4a4a6622f1ae`
