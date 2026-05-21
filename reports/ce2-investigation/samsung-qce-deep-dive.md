# Samsung SGH-I727 qce.ko deep-dive vs our CE2 cipher path

Date: 2026-05-18
Reference module: `/tmp/samsung-i727/qce.ko` (MSM8660 native CE2 driver,
v1.15 from CodeAurora msm-3.0.8 era, Mona Hossain/codeaurora.org)
Decompile: `/tmp/ghidra-qce/qce_decompiled.c` (33 module functions)

## Architectural map

```
qce_open()      one-shot init.  Allocates 8 KB DMA-coherent buffer,
                splits into:
                  +0x000  CE_IN  SG list  (128 × 8 B entries)
                  +0x400  CE_OUT SG list  (128 × 8 B entries)
                  +0x800  CE_IN  ADM cmd block (2 chained dmov_s, 32 B)
                  +0x840  CE_OUT ADM cmd block (2 chained dmov_s, 32 B)
                  +0x860  pad SG list (128 × 8 B, pre-filled with
                          DATA_SHADOW0 phys for cipher-OUT 'discard')
                  +0xc60  result/pad buffer (1 KB)
                  +0x1070 CE_IN  CMD_PTR (1 entry → cmd block 1)
                  +0x1080 CE_OUT CMD_PTR (1 entry → cmd block 2)
                  +0x1084 spare result area
                Calls FUN_00010b54: SW_RST → DSB → CONFIG=0x78 →
                read VERSION, ENGINES_AVAIL → cache flags.

qce_ablk_cipher_req()  per request.
                  1. dma_map_sg(src, dst, ...)
                  2. FUN_00011d0c builds INPUT SG list in pre-alloc
                     buffer (coalesces contiguous phys, splits at
                     32 KB, max 128 entries).
                  3. FUN_000120c8 builds OUTPUT SG list.
                  4. Pads to AES-block boundary by adding one more
                     SG entry pointing at static pad buffer.
                  5. FUN_0001020c / FUN_000102fc finalize: set LAST
                     bit on the tail SG entry, write total length
                     into the ADM cmd block's len field.
                  6. _ce_setup (FUN_00010f08): writes all engine
                     registers, full AES key schedule, IV, SEG_CFG,
                     SEG_SIZE, GOPROC.  Per-op clk_enable on entry.
                  7. FUN_0001050c: msm_dmov_enqueue_cmd on both
                     CE_IN and CE_OUT channels.

_ablk_cipher_ce_{in,out}_call_back()  on ADM completion.
                  Tracks both channels with refcount.  When second
                  finishes: dma_unmap_sg, read back updated IV from
                  CNTR0..3 (FUN_00012714: BE-word → bytes), clk_disable,
                  call user callback.
```

## Key engine setup pattern (`_ce_setup` / FUN_00010f08)

```c
clk_enable(qce->clk);                              // per-op clock

// For AES (param_2[3] != 0 && != 1):
*(qce + REG_CNTR_MASK) = 0xffff;                   // ALWAYS, not just CTR

iv_path: BE-pack iv, write CNTR0_IV0..CNTR3_IV3.

key_path:
  if (key matches cache) skip_to_seg_cfg;
  if (!hw_key_mode) {
    // FIPS-197 full expansion from BE-packed seed words
    for k in [0..keylen/4):    enckey32[k] = BE_pack(user_key + k*4);
    expanded[0..keylen/4) = enckey32;
    for k in [keylen/4..N):    expanded[k] = compute FIPS round;
    // Write all 60 dwords
    for k in [0..60):  writel(expanded[k+1], qce + RNDKEY0 + k*4);
    // (index +1 in source; auStack_160[0] is preserved/unused)
  } else {
    // USE_HW_KEY: write raw keylen bytes only
  }
  cache_key = enckey32;

seg_cfg = FIRST | LAST | ENCR_ALG_AES | key_size_bits;
encr_seg_cfg = (cryptlen << 16) | (coffset & 0xffff);
if (encrypt_direction) {
    seg_cfg |= ENCODE | AUTH_POS;   // bits 16+14
    encr_dir_mod = 0x10000;
}
writel(encr_seg_cfg, qce + REG_ENCR_SEG_CFG);
writel(seg_cfg | encr_dir_mod, qce + REG_SEG_CFG);
writel(cryptlen,                  qce + REG_SEG_SIZE);
writel(1,                         qce + REG_GOPROC);
DataSynchronizationBarrier(0xf);
```

CONFIG register: written to `0x78` ONCE at probe.  `0x78 =
MASK_DOUT_INTR | MASK_DIN_INTR | MASK_AUTH_DONE_INTR | MASK_ERR_INTR`.
Never touched per-op.

## ADM command word pattern (from qce_open)

CE_IN cmd: `0xc001 | (crci<<7)`
  bit 0  = CMD_MODE_SG (scatter-gather)
  bit 14 = CMD_DST_SWAP_BYTES   ← ADM hw byte-swap
  bit 15 = CMD_DST_SWAP_SHORTS  ← ADM hw short-swap
  bits 7-10 = DST CRCI (4 for CE_IN)
                = together: byte+short swap = full 32-bit byte reverse

CE_OUT cmd: `0x1801 | (crci<<3)`
  bit 0  = CMD_MODE_SG
  bit 11 = CMD_SRC_SWAP_BYTES
  bit 12 = CMD_SRC_SWAP_SHORTS
  bits 3-6 = SRC CRCI (5 for CE_OUT)

The trailing cmd in each chain (auth/IV readback): `0x80001800 | (crci<<3)`
  bit 31 = CMD_LC (last-command terminator)
  bits 11,12 = SRC byte+short swap
  bit 0  = CMD_MODE_SINGLE

So Samsung's ADM does **full 32-bit byte-reverse in hardware** during
the data transfer between user memory (LE) and the engine port (BE).

## Side-by-side: our driver vs Samsung's

| Aspect | Samsung | Our common.c | Notes |
|---|---|---|---|
| Per-op clock gating | `clk_enable` on entry, `clk_disable` on completion | `reset_control_assert/deassert` per op (1ms+1ms) | We can't `clk_disable` cleanly because mainline CE clock has different gating |
| Per-op SW_RST | NO | YES (10us+10us pulse) | Maybe redundant |
| CONFIG register writes/op | NONE | 3 (SW_RST=1, 0, final) | Setup churn |
| AES key path | Full FIPS-197 expansion → 60 dwords to RNDKEY0..59 | Raw `keylen` bytes only | "fastaes=1" path; engine claims to expand internally |
| Key cache | YES | NO | Each op re-writes (or recomputes) |
| CNTR_MASK | 0xffff ALWAYS for AES | 0xffff only when CTR | Possible deviation |
| Data path direction | SG-mode ADM with `CMD_{DST,SRC}_SWAP_BYTES \| _SWAP_SHORTS` | BOX-mode ADM, no swap flags; software byte-swap pre-DMA | Functionally equivalent on the wire, but pacing/timing may differ |
| DMA buffer alloc | Static pre-alloc at open() | dma_alloc_coherent twice per op | 4 extra allocator calls/op |
| Bounce/copy | NONE — user SG passed directly | sg_copy_to_buffer src→bounce, sg_copy_from_buffer bounce→dst | Extra memcpy per op |
| SG max entry length | 32 KB | N/A (single contiguous bounce buffer) | Samsung walks user SG natively |
| ADM cmd type | `CMD_MODE_SG` (scatter-gather descriptor list) | `BOX` (qcom_adm always uses BOX for FC) | Different DMA topology |
| GOPROC write order | last, after SEG_SIZE | last, after CONFIG | Same |
| DSB after setup | Explicit DSB(0xf) | Implicit via writel barriers | Probably equivalent |
| IV readback | Per-completion (in callback) via BE-word→bytes | Per-op (synchronous) via cpu_to_be32 + memcpy | Same end result |

## Confirmed empirical findings

1. **Pre-expanded 60-key write did NOT lift the 4-block cap** — our
   reverted commit `be484e004d3c0fa6b` (re-applied as 9e531d24ddb5)
   used `aes_expandkey()` + `cpu_to_be32()` and still hit blocks-5+
   wrong.  **However** the implementation had a bug: aes_expandkey
   seeds from `get_unaligned_le32(key)` while Samsung seeds from
   BE-packed words.  These produce different expansion schedules
   from the same input bytes, so the test was inconclusive — the
   keys we wrote past index 0 were probably wrong.

2. **Software byte-swap is functionally equivalent to ADM hw swap**:
   both deliver the same 32-bit value at the DATA_SHADOW0 port.
   The 4-block cap can't be blamed on swap direction alone.

3. **DES handles 32 KB per op fine** — same DMA path, same swap, same
   reset.  AES-only problem.  Points at AES engine, not DMA.

4. **CRCI handshake granularity tested** — burst sizes {16, 32, 64,
   128, 192, 256 B} all give the same 4-block cap.

## Best-guess root cause

The AES engine on this CE2 silicon has an internal pipeline limit of
~4 blocks per "session".  Either the round-key autoexpand state
machine wraps wrong after 4 blocks, or the CBC chain register
register wraps without proper DOUT_RDY pacing.

Samsung sidesteps it by:
  (a) writing a fully pre-expanded round-key table (bypassing the
      engine's internal expander), AND
  (b) using `CMD_MODE_SG` ADM which delivers data one SG segment
      at a time with CRCI handshake between segments (a different
      pacing pattern from `BOX` rows).

Either or both could be load-bearing.

## Plan — incremental experiments

Each step is a single commit, testable in isolation with both
`/tmp/qce-test/qce-cipher-test` (NIST vectors ≤ 32 B, should always
pass) and `qce-streamenc` at sizes 64 B → 1 KB → 32 KB.

### Step 1: CORRECT pre-expanded key schedule

Re-do `be484e004d3c0fa6b` but with **byte-reversed input** to
`aes_expandkey()` so the LE-read inside produces BE-packed seeds
matching Samsung's expansion.  After expansion, write `key_enc[k]`
raw (no second cpu_to_be32) to RNDKEY0..59.

```c
__be32 enckey_be[8];          /* zero-padded to AES-256 */
struct crypto_aes_ctx ctx_aes = {0};
u32 expanded[60] = {0};

qce_cpu_to_be32p_array(enckey_be, ctx->enc_key, keylen);
/* aes_expandkey LE-reads bytes; pass enckey_be (stored as byte-
 * reversed in memory on LE ARM) so LE-read = original BE word.
 */
aes_expandkey(&ctx_aes, (const u8 *)enckey_be, keylen);
/* ctx_aes.key_enc[0..Nk-1] are the BE-packed seeds; rest is FIPS
 * expansion thereof — matches Samsung's expanded buffer.
 */
for (k = 0; k < 60; k++)
    writel(ctx_aes.key_enc[k], qce->base + CE2_REG_AES_RNDKEY0 + k*4);
memzero_explicit(&ctx_aes, sizeof(ctx_aes));
```

Risk: cross-algo interference (3DES-after-AES) if AES_RNDKEY isn't
cleared by per-op reset.  Mitigation: write 60 zeros to RNDKEY for
DES/3DES paths.

**Expected outcome**: if the cap is from engine's internal expander,
this lifts it.  If the cap is from FIFO/CRCI pacing, no change.

### Step 2: CNTR_MASK = 0xffff for all AES modes

webOS line 1503 writes 0xffff for all AES (ECB/CBC/CTR).  Samsung does
the same in `_ce_setup` for the AES branch.  We only write for CTR.

```c
if (!IS_DES(flags) && !IS_3DES(flags))
    writel(0xffff, qce->base + CE2_REG_CNTR_MASK);
```

**Expected outcome**: small chance — CNTR_MASK is supposed to be
CTR-mode-specific.  But matching legacy exactly is cheap.

### Step 3: Drop per-op SW_RST (keep `reset_control`)

webOS does SW_RST only at `_init_ce_engine` (probe time), never
per-op.  Samsung never does SW_RST at all in `_ce_setup`.  Our
per-op SW_RST pulse may be destabilizing the engine.

```c
// Remove lines 1693-1696
```

**Expected outcome**: was originally needed for 3DES cold-start, but
that was before we dropped CLR_CNTXT (commit 895a8401f090).  May be
safe to drop now.  Worst case: 3DES cold-start regresses.

### Step 4: Bypass dmaengine — emit `CMD_MODE_SG` ADM cmd directly

This is the bigger change.  Patch `drivers/dma/qcom/qcom_adm.c` to
expose `peripheral_config.cmd_flags` so a peripheral can request
byte-swap + SG mode.  Then change `qce_ce2_dma_inout_cipher` to set
the swap flags on the slave config (and drop our software swap).

**Expected outcome**: if the cap is from BOX vs SG pacing, this
lifts it.  Higher implementation cost.

### Step 5 (if all above fail): Direct register-poke PIO path

Skip ADM entirely, copy webOS slow path: poll `DIN_RDY` /
`DOUT_RDY` in `STATUS`, writel/readl one block at a time via
`DATA_IN`/`DATA_OUT` (NOT DATA_SHADOW0).  This is what HTC sbl3
does — known working but ~10× slower than DMA.

Useful only as a "does this silicon even support multi-block?" sanity
check.

## What we won't try (already disproven)

- Burst-size sweep ({16, 32, 64, 128, 192, 256}) — same cap on all
- Adding CLR_CNTXT — caps AES at 4 blocks (this is what's currently
  removed, 895a8401f090)
- Force AUTH_POS=1 for 3DES decrypt — broke encryption (376e22522baf)
- Dummy AES warmup before real op — destabilized other algorithms
  (630ea34a1d4a)
- Different ADM descriptor row counts — qcom_adm builds same way

## Test matrix per step

| Step | qce-cipher-test (NIST ≤32 B) | qce-streamenc 64 B | qce-streamenc 1 KB | qce-streamenc 32 KB |
|---|---|---|---|---|
| current | 12/12 ✓ | match | match first 4 blk | match first 4 blk |
| 1 (correct key sched) | ? | ? | ? | ? |
| 2 (CNTR_MASK always) | ? | ? | ? | ? |
| 3 (drop SW_RST) | ? | ? | ? | ? |
| 4 (ADM swap+SG mode) | ? | ? | ? | ? |

After each step, capture `dev_info` SEG_CFG/STATUS values pre- and
post-op so we can correlate behavior with engine state.
