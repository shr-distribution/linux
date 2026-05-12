# v8 clean 16-cycle baseline + register sweep

**Date:** 2026-05-12
**Mesa libgallium md5:** `b725f49bd54612a3db478dfcb77d03b2` (0098 v8)
**Kernel:** `6.18.0-luneos-gb6dc680b4167`
**Env:** NO env-vars (Test 4 baseline path)
**State:** Fresh after sysrq reboot to clear prior FD2_SKIP_* contamination

## The full 16-cycle in 16 captures (one of each phase)

| cap | phase | hash | what's visible |
|-----|-------|------|----------------|
| 01 | 0 | `5adc3160` ✓ | **CORRECT** clean RGB triangle (blue top, red BL, green BR) |
| 02 | 1 | `4477e60a` | rotated colors (red top, green BL, yellow BR) + black top-right tile |
| 03 | 2 | `bebc09c6` | correct top half, rotated bottom half |
| 04 | 3 | `3625b67f` | rotated + yellow BL tile + green BR tile |
| 05 | 4 | `03dee03a` | correct top, but mid-strip flipped |
| 06 | 5 | `9fb336c8` | rotated with 4 distinct tile states |
| 07 | 6 | `e93d10aa` | rotated + bright green BL tile |
| 08 | 7 | `202cfe9f` | correct top + orange top-left tile |
| 09 | 8 | `e21b9529` | correct top + dark mid-right tile |
| 10 | 9 | `3c9c950b` | correct top + bright yellow BR tile |
| 11 | 10 | `ab2c6c00` | rotated + dark top-left + yellow BR |
| 12 | 11 | `6c10867f` | rotated + yellow BL tile |
| 13 | 12 | `0cbed90a` | rotated + correct-ish top, dark mid |
| 14 | 13 | `c60a6009` | rotated + brown BR tile |
| 15 | 14 | `5908eca8` | rotated + small wrong tile top-right |
| 16 | 15 | `7325d713` | rotated + dark green mid-left tile |

After cap 16 the cycle loops back to `5adc3160` at cap 17.

See `cycle-contact-sheet.png` for all 16 side-by-side.

## Two clear "modes" alternating roughly every few caps

**Mode A — vertex color mode toggle.**  Look at where each vertex sends which color:
- ✓ correct mode: blue at top, red at bottom-left, green at bottom-right (caps 1, 3, 5, 8, 9, 10, 13)
- ✗ rotated mode: red at top, green at bottom-left, yellow at bottom-right (caps 2, 4, 6, 7, 11, 12, 14, 15, 16)

Yellow at BR is `R + G` mixed — the bottom-right vertex is driving BOTH color channels.  So this isn't a simple "rotate vertex order"; vector components are crossed across vertices.

**Mode B — per-tile state mosaic.**  Within each mode, individual 384×384 tiles (binner pipe partitions) render with different vertex-attribute snapshots.  Each tile can pick one of several different "wrong" or correct configurations.

The 16-state cycle is best interpreted as **(2 vertex-color modes) × (8 tile-state variations) = 16**.  Fork A/B (patch 0093) likely added the 2-mode outer toggle by introducing a second IB (the binning IB) per render — see project_a22x_period_cycle_truth memory note.

## Register sweep — NEGATIVE result

The cap binary samples 53 registers (CP, RBBM, MH, SQ_DEBUG_*, VGT, VSC) after every cap.  **Zero of them vary across the 16 captures.**  Every register has identical post-`glFinish()` value at cap 1 and cap 16.

This means the cycle source is either:
- In GPU SRAM that is **not exposed via MMIO** (e.g., wavefront-slot constant cache, ALU register banks, binner internal FIFOs)
- In a register we **don't sample** (a2xx_reg.h has ~600 registers; we dump 53)
- **Quenched by `glFinish()` before our dump** — DURING rendering the state may differ but is reset by the time we sample

Need to either:
1. Sample MORE registers (extend the dump list)
2. Sample BEFORE glFinish() (mid-frame, harder to do)
3. Dump GPU SRAM directly if exposed in debugfs

## Files

| file | meaning |
|---|---|
| `cap_NN_<hash>.png` | rendered framebuffer at cap NN (1..16) |
| `cap_NN_<hash>.bin` | raw 1024×768 RGBA8 dump |
| `regs_NN_<hash>.txt` | register snapshot after cap NN |
| `cycle-contact-sheet.png` | 4×4 mosaic of all 16 phases for visual inspection |
