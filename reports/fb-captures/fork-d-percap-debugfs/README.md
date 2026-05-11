# Fork D + new debugfs — per-hash register state capture (2026-05-11)

Tested kernel `g6044076c975e` (adds VSC_PIPE_DATA_LENGTH + 0xC00 readback
to vgt debugfs) with Fork D mesa (md5 `c17b080b010e93d8d2fd23e9d8f8f01d`).

Captured `vgt` and `sq` debugfs after each unique hash in the 8-cycle.

## Result: All 8 register dumps byte-identical

```
$ for h in (7 wrong hashes); do diff vgt.5adc3160 vgt.$h; done
# (no output — files are identical)

$ for h in (7 wrong hashes); do diff sq.5adc3160 sq.$h; done
# (no output — files are identical)
```

Every register sampled — VSC_PIPE[0..7] CONFIG/ADDR/LEN, VSC_BIN_SIZE,
0xC00, VGT_CURRENT_BIN_ID, SQ_DEBUG_INPUT_FSM, SQ_DEBUG_EXP_ALLOC,
SQ_DEBUG_PTR_BUFF, SQ_DEBUG_GPR_VTX/PIX, SQ_PROGRAM_CNTL,
SQ_INST_STORE_MGMT, SQ_GPR_MANAGEMENT — **same across all 8 cycle phases**.

## Implications

1. **Cycle source is not in MMIO registers we can read.** Whatever drives
   the 8 different visibility outcomes is in:
   - The actual VSC pipe BO byte contents (binner wrote different
     visibility streams per phase, even though DATA_LENGTH is the same)
   - Internal hardware state machines hidden from the register interface
   - Latched state in functional units (rasterizer, EDRAM controller, ...)

2. **VSC_REG_0xC00 reads back as 0** despite Mesa writing 1 every submit.
   Either the register doesn't exist at offset 0xC00, or it's write-only
   / self-clearing. Our "VSC enable" theory needs revisiting.

3. **VSC pipe state is correct and consistent.** The kernel sees:
   - 6 pipes configured (VSC_PIPE[0..5]) with CONFIG = (H=1)(W=1)(Y)(X)
     forming a 2×3 grid for our 2×3 GMEM tile layout
   - VSC_PIPE[6..7] unused (CFG=0) — correct for 6-tile layout
   - All pipes allocated 256KB (LEN=0x40000) at distinct iovas
   - VSC_BIN_SIZE = 0x110 → tile = 16×8 in 32-pixel units = 512×256 pixels
     (2×3 tiles × 512×256 = 1024×768 framebuffer — checks out)

## Sample VGT dump

```
A22X VSC:
  VSC_BIN_SIZE:      00000110
  VSC_REG_0xC00:     00000000
  VSC_PIPE[0]:       CFG=01100000 ADDR=6642d000 LEN=00040000
  VSC_PIPE[1]:       CFG=01100001 ADDR=6646d000 LEN=00040000
  VSC_PIPE[2]:       CFG=01100400 ADDR=664ad000 LEN=00040000
  VSC_PIPE[3]:       CFG=01100401 ADDR=664ed000 LEN=00040000
  VSC_PIPE[4]:       CFG=01100800 ADDR=6652d000 LEN=00040000
  VSC_PIPE[5]:       CFG=01100801 ADDR=6656d000 LEN=00040000
  VSC_PIPE[6]:       CFG=00000000 ADDR=665ad000 LEN=00040000
  VSC_PIPE[7]:       CFG=00000000 ADDR=665ed000 LEN=00040000
```

## Strategic implications

This rules out a large class of register-pulse fixes. The cycle isn't
addressable via any register we've enumerated — Fork D's SQ_GPR pulse
(per-batch + cold-start) had cosmetic effect only because of this.

Remaining viable diagnostics:
- Read VSC pipe BO content bytes after a submit (requires CPU mapping
  the BO or kernel debugfs that walks iommu page tables)
- Read EDRAM tile-binning hash state (if any register exposes it)

Remaining viable fixes:
- Full Fork A/B: real binning shader pass + visibility filtering per
  tile (the webOS / freedreno-a20x model)
- Accept 12.5% baseline as ship floor
