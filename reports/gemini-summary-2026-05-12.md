# Adreno 220 (A22X / Leia / REV470) period-N render cycle — status for Gemini

## TL;DR

Mainline-Linux + freedreno renders the GLES2 "Hello Triangle" deterministically wrong with a **period-16 cycle across submits**. Only phase 0 (`5adc3160`) is bit-exact correct; the other 15 phases show vertex-color rotation and per-tile mosaic corruption. The cycle is **invisible in 53 MMIO registers sampled post-`glFinish()`** — must be in GPU SRAM or in state we don't probe.

## Hardware / software

- HP TouchPad: APQ8060 dual-core Scorpion 1.5 GHz + Adreno 220 (A220 / Leia / REV470)
- Kernel 6.18 mainline (drm/msm a2xx) with various local SoC fixes
- Mesa 26.1+git freedreno A2XX driver + 0001..0098 stacked patches (in meta-mainline Yocto layer)
- Reference: webOS proprietary `libGLESv2.so` ghidra decomp (`leia_perform_resolve`, `leia_configure_binning_pass`, `leia_configure_binid_groups`)

## Cycle observations (confirmed 2026-05-12)

### Empirical pattern
- Fresh-reboot 16-cap default sweep produces all 16 phases in order (cap 1=`5adc3160` correct, cap 16=`7325d713`, cap 17 loops to cap 1)
- Each submit advances cycle by +1 mod 16, deterministically
- `FD2_PIN_BIN_ID=<n>` (override `VGT_CURRENT_BIN_ID_MIN/MAX` to constant `<n>` for all tiles) only shifts the *starting phase* by +5 mod 16; cycle length and `+1 per submit` advance are unchanged

### Visual structure (contact sheet of 16)
Cycle decomposes as **2 × 8 = 16**:
- **Mode A (outer, period 2)**: vertex color mode toggle
  - Correct mode: blue@top, red@BL, green@BR
  - Rotated mode: red@top, green@BL, **yellow@BR** (R+G crossed — vertex vector components mixed)
- **Mode B (inner, period 8)**: per-tile state mosaic — individual 384×384 tiles (binner pipe partitions) render with different vertex-attribute snapshots

### Period changed 8→16 around 2026-05-11 when patch 0093 (Fork A/B HW binning integration) was merged into the mesa Yocto stack. Hypothesis: 0093 added a second IB per render (the binning IB via `fd2_emit_ib(ring, batch->binning)`), introducing the Mode A outer toggle. Pre-0093 cycle was period 8 (Mode B only).

## Things tried and falsified

| approach | result |
|---|---|
| Kernel SQ_GPR_MANAGEMENT pulse @ hw_init | no effect, cycle phase shifts only |
| Mesa SQ_GPR_MANAGEMENT pulse per batch | no effect |
| 8-iteration SQ wavefront slot scrub (patch 0070 — currently shipping) | no cycle break, defensive only |
| 16-iteration scrub | no improvement over 8 |
| GFX3D_RESET pulse via gdsc.c | no effect on cycle |
| force_collapse (full GDSC drop between submits) | falsified — introduces per-pixel noise without breaking cycle |
| CPU performance governor / affinity | invariant |
| GPU clock 96..320 MHz | invariant |
| ondemand vs performance governor | invariant (ondemand also induces unrelated lockups) |
| `FD2_PIN_BIN_ID=0..7` (force same `VGT_CURRENT_BIN_ID` per tile) | cycle persists, only starting phase shifts |
| webOS-style A22X CP_SET_BIN_DATA visibility consume (0098 v3..v7) | binning IB doesn't produce real visibility data (binner writes zero bytes), SET_BIN_DATA reads garbage → GPU hang |
| CACHE_FLUSH event + RBBM_STATUS poll between binning IB and resolve | safe (validated v3), doesn't break cycle on its own |
| webOS-undocumented `0xC04 = 0` register write | safe when present, didn't break cycle either |
| Per-tile `CP_WAIT_REG_EQ` on `RBBM_DEBUG.A220_BINNER_DONE` (bit 24) | bit never latches on mainline path; polls forever, hangcheck |
| `FD2_SKIP_PRELUDE=1` / `FD2_SKIP_BINNING_IB=1` (try to revert to pre-Fork-A/B behaviour) | hangs — prelude is load-bearing once 0093 ships, runtime falsifier not possible |

## What we know about webOS's render path (from decomp)

webOS `leia_perform_resolve` per-tile sequence (uVar23 bit 6 guard = "binner used"):

```
# Pre-loop (once per renderpass, after binning IB):
CP_SET_CONSTANT  A220_RB_LRZ_VSC_CONTROL = 0           # disengage binner
OUT_PKT0         A220_VSC_PIPE_PARTITIONING (0xC00) = 0  # master disengage
CP_EVENT_WRITE   CACHE_FLUSH (event 6)                 # flush binner FIFOs
CP_WAIT_REG_EQ   RBBM_STATUS, val=0, mask=0x5f601000   # 3D pipe idle
                                                       # (mask covers VGT/SX/TPC/
                                                       # SC_CNTX/PA/RB_CNTX/SQ_CNTX0+17)

# Per tile:
CP_WAIT_REG_EQ   RBBM_DEBUG, val=0x01000000, mask=0x01000000, poll=1   # bit 24 = binner done
CP_SET_BIN_DATA  bin_data_addr = VSC_PIPE[tile->p].DATA_ADDRESS
                 bin_size_addr = VSC_SIZE_ADDRESS + p*4
                 [last_tile_flag if 3-dword variant]
CP_DRAW_INDX     ...
```

webOS `leia_configure_binid_groups` (once per renderpass):
```
OUT_PKT0  0xC01 (VSC_BIN_SIZE)        = width/height in 32-pix units
OUT_PKT0  0xC02 (VSC_SIZE_ADDRESS)    = size_bo_iova + state->offset_4
OUT_PKT0  0xC04                       = 0                    # undocumented
OUT_PKT0  0xC06+ (VSC_PIPE × 8)       = 24 dwords pipe config
```

webOS `leia_configure_binning_pass` (per draw in binning pass):
```
OUT_PKT0         A220_VSC_PIPE_PARTITIONING (0xC00) = 1
CP_SET_CONSTANT  A220_RB_LRZ_VSC_CONTROL = 3   # engage binner
CP_WAIT_FOR_IDLE
OUT_PKT0         SQ_GPR_MANAGEMENT = (some value)
```

## What we have NOT been able to test or observe

1. **The cycle source in MMIO.** Sampled 53 registers post-`glFinish()` across the 16 caps — all identical. Either source is GPU SRAM (not memory-mapped), in a register we don't sample (a2xx has ~600 regs total), or quenched by `glFinish` before our read.

2. **Whether the binning IB actually produces visibility data.** Prior FD2_VSC_DUMP=1 testing showed binner writes ZERO bytes to pipe BOs on Mesa path. Ir2_nir patch 0095 strips A20X memexport CFs for A22X without adding an A22X equivalent — A22X is supposed to use the HW binner, but the binner needs the VS to feed it properly, and apparently doesn't.

3. **Why `RBBM_DEBUG.A220_BINNER_DONE` (bit 24) never latches on mainline.** webOS's per-tile poll loop relies on it. On mainline, polling never succeeds even after CACHE_FLUSH event. Possibly tied to (2) — if the binner isn't streaming visibility, it doesn't latch its done bit.

## Open questions for Gemini

1. **Where does the period-16 state live?** Suggestions for which GPU-internal SRAM or unsampled MMIO registers could host a 16-state counter that advances exactly +1 per IB submission?

2. **Mode A toggle source.** If patch 0093's second IB (binning IB) added the period-2 outer toggle, what specific GPU state would the binning IB perturb that the per-tile loop doesn't reset?  Candidates we considered: SQ wavefront-slot constant cache, ALU register bank (only 1 of N banks holds correct vertex constants), interpolation parameter cache.

3. **Why "yellow at BR" in rotated mode?** Yellow = R+G. The bottom-right vertex shouldn't be driving both. This isn't a simple vertex-index rotation; vector COMPONENTS appear to cross between vertices. Is this consistent with a known A22X / A220 SQ FSM bug we should be aware of?

4. **Mainline kernel diff vs KGSL that could matter.** KGSL initializes the full 0xC01..0xC1D VSC register block to zero at context start; mainline drm/msm a2xx doesn't. Are there other large register blocks KGSL zeros at context start that mainline misses? We've covered SQ_INST_STORE_MGMT, SQ_GPR_MGMT, RBBM_PM_OVERRIDE1/2, MH_MMU_CONFIG, but not the full SQ_CONST_*/ALU_CONST/BOOL/LOOP banks.

5. **The "binner writes zero" mystery.** webOS engages the binner with similar prelude (LRZ_VSC_CONTROL=3, 0xC00=1, SQ_GPR_MGMT pulse) and the binner produces visibility data into the pipe BOs. Mesa engages with the same registers and the binner writes nothing. What setup is webOS doing that we miss? Candidates: an extra PA_SC_* / VGT_* register, a specific shader-export flag (memexport or stream-out variant), a specific draw-init flag we're not setting.

## Reproduction recipe

1. mainline 6.18 kernel + mesa 26.1+git with patches 0001..0098 (v8 of 0098) applied via meta-mainline bbappend
2. Sysrq reboot to clean state
3. Run `/tmp/gl-cap-and-regdump-mainline` 16 times in a row with no env vars
4. Expect 16 unique hashes in deterministic order starting `5adc3160` (cap 1)
5. `FD2_PIN_BIN_ID=<n>` shifts starting phase but cycle persists
6. `FD2_USE_BIN_DATA=1` and friends DO hang the GPU — those env-vars are dead ends and should not be tried

## Files for reference

- `reports/fb-captures/v8-clean-16cycle-2026-05-12/` — clean 16-cap baseline + register dumps + contact sheet
- `reports/fb-captures/v8-pin-sweep-2026-05-12/` — FD2_PIN_BIN_ID sweep proving cycle is per-submit not per-tile
- `reports/fb-captures/v6-t1-default-2026-05-12/` — initial visual analysis identifying per-tile mosaic + vertex-color rotation
- `reports/ghidra-decomp/decomp-txt/webos_libGLESv2.so.decomp.txt` — full decomp of webOS proprietary GLES driver
- Yocto patches: `meta-mainline/recipes-graphics/mesa/files/0001..0098*.patch`
- Kernel branch: `tenderloin/6.18/upstream-patches` on `shr-distribution/linux`
